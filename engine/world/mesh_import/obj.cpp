// world/mesh_import/obj.cpp -- the OBJ reader. See world/mesh_import.hpp for
// the contract and the refusals.
//
// One pass, line by line: the vertices into one list, and each face as its
// vertex indices with the o and g names in force (a polygon fanned into
// triangles). At the end the faces are grouped by o, or by g in a file with
// no o, into objects in the order of their first face. Each object gathers the vertices its faces name in
// first-use order, then goes to the shared exact weld and make_mesh().
#include "world/mesh_import.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <expected>
#include <fstream>
#include <limits>
#include <map>
#include <new>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "core/path_text.hpp"
#include "core/utf8.hpp"
#include "world/detail/weld.hpp"

namespace spade {
namespace {

constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();
constexpr std::string_view kBom = "\xEF\xBB\xBF";

[[nodiscard]] std::string num(uint64_t v) {
    return std::to_string(v);
}

[[nodiscard]] bool blank(char c) noexcept {
    return c == ' ' || c == '\t';
}

[[nodiscard]] std::string_view trimmed(std::string_view s) noexcept {
    while (!s.empty() && blank(s.front())) s.remove_prefix(1);
    while (!s.empty() && blank(s.back())) s.remove_suffix(1);
    return s;
}

// A number's text with one leading '+' dropped, as std::from_chars takes none.
[[nodiscard]] std::string_view unsigned_plus(std::string_view s) noexcept {
    if (s.size() > 1 && s[0] == '+' && s[1] != '+' && s[1] != '-') s.remove_prefix(1);
    return s;
}

enum class Number : uint32_t { ok, not_a_number, out_of_range, not_finite };

[[nodiscard]] Number read_number(std::string_view field, double& out) noexcept {
    const std::string_view s = unsigned_plus(field);
    const char* const end = s.data() + s.size();
    const std::from_chars_result r = std::from_chars(s.data(), end, out);
    if (r.ec == std::errc::invalid_argument || r.ptr != end) return Number::not_a_number;
    if (r.ec == std::errc::result_out_of_range) return Number::out_of_range;
    if (!std::isfinite(out)) return Number::not_finite;
    return Number::ok;
}

// Names in the order first met, each with its slot; slot 0 is the file's stem.
class Names {
  public:
    explicit Names(const std::string& stem) { (void)slot(stem); }

    [[nodiscard]] uint32_t slot(const std::string& name) {
        const auto [it, added] = slots_.try_emplace(name, static_cast<uint32_t>(names_.size()));
        if (added) names_.push_back(name);
        return it->second;
    }
    [[nodiscard]] const std::string& name(uint32_t slot) const { return names_[slot]; }
    [[nodiscard]] std::size_t size() const noexcept { return names_.size(); }

  private:
    std::map<std::string, uint32_t> slots_;
    std::vector<std::string> names_;
};

struct Face {
    std::array<uint32_t, 3> v{};
    uint32_t o = 0;  // the o name in force: its slot
    uint32_t g = 0;  // the g name in force
};

// One kind of element a corner names.
struct Kind {
    std::string_view one;
    std::string_view many;
};
constexpr Kind kVertex{"vertex", "vertices"};
constexpr Kind kTexture{"texture coordinate", "texture coordinates"};
constexpr Kind kNormal{"normal", "normals"};

// Whether a face of more than three corners can be fanned, or why not (the
// header's POLYGONS). `p` are its corners in face order.
enum class Polygon : uint32_t { ok, no_area, not_planar, not_convex, winds_again };

struct PolygonCheck {
    Polygon verdict = Polygon::ok;
    std::size_t corner = 0;  // the first offending corner, 0-based (not_planar, not_convex)
};

[[nodiscard]] PolygonCheck check_polygon(std::span<const glm::dvec3> p) {
    const std::size_t n = p.size();
    glm::dvec3 centre(0.0);
    for (const glm::dvec3& q : p) centre += q;
    centre /= static_cast<double>(n);

    // Newell's normal (twice the area vector) of the centred corners, and the
    // corners' bounding box.
    glm::dvec3 normal(0.0);
    glm::dvec3 lo = p[0];
    glm::dvec3 hi = p[0];
    for (std::size_t i = 0; i < n; ++i) {
        normal += glm::cross(p[i] - centre, p[(i + 1) % n] - centre);
        lo = glm::min(lo, p[i]);
        hi = glm::max(hi, p[i]);
    }
    const double nn = glm::dot(normal, normal);
    if (!(nn > 0.0)) return {Polygon::no_area};

    // Planar: every corner within tolerance x width of the plane, the width
    // being 2 area / diagonal = |normal| / diagonal, so a thin face is held to
    // its own width (a fan's fold stays below about the tolerance in radians).
    // With d the corner's offset along the normal: (d / |normal|)^2 <=
    // (tolerance |normal| / diagonal)^2, i.e. d^2 diagonal^2 <= tolerance^2
    // |normal|^4, compared squared so no root is taken.
    const glm::dvec3 extent = hi - lo;
    const double diagonal2 = glm::dot(extent, extent);
    const double limit = kObjPlanarTolerance * kObjPlanarTolerance * nn * nn;
    for (std::size_t i = 0; i < n; ++i) {
        const double d = glm::dot(p[i] - centre, normal);
        if (d * d * diagonal2 > limit) return {Polygon::not_planar, i};
    }

    // Strictly convex, in the projection along the normal's largest component
    // (the lowest axis on a tie), oriented so the face's own winding is
    // positive: every corner turns left, and the edges' u components change
    // sign exactly twice around (a face that winds round twice, a star,
    // changes them four times).
    std::size_t axis = 0;
    for (std::size_t k = 1; k < 3; ++k) {
        const double a = normal[static_cast<glm::length_t>(k)];
        const double b = normal[static_cast<glm::length_t>(axis)];
        if ((a < 0.0 ? -a : a) > (b < 0.0 ? -b : b)) axis = k;
    }
    const auto u = static_cast<glm::length_t>((axis + 1) % 3);
    const auto w = static_cast<glm::length_t>((axis + 2) % 3);
    const double sign = normal[static_cast<glm::length_t>(axis)] > 0.0 ? 1.0 : -1.0;
    for (std::size_t i = 0; i < n; ++i) {
        const glm::dvec3& prev = p[(i + n - 1) % n];
        const glm::dvec3& next = p[(i + 1) % n];
        const double au = p[i][u] - prev[u];
        const double aw = p[i][w] - prev[w];
        const double bu = next[u] - p[i][u];
        const double bw = next[w] - p[i][w];
        if (!((au * bw - aw * bu) * sign > 0.0)) return {Polygon::not_convex, i};
    }
    int first = 0;  // the sign of the first edge's u component that is not 0
    int last = 0;
    std::size_t changes = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double du = p[(i + 1) % n][u] - p[i][u];
        const int s = du > 0.0 ? 1 : du < 0.0 ? -1 : 0;
        if (s == 0) continue;
        if (first == 0) first = s;
        if (last != 0 && s != last) ++changes;
        last = s;
    }
    if (last != first) ++changes;
    if (changes > 2) return {Polygon::winds_again};
    return {};
}

class Reader {
  public:
    Reader(std::string file, const std::string& stem, MeshUnit unit)
        : file_(std::move(file)), unit_(unit), o_names_(stem), g_names_(stem) {}

    [[nodiscard]] Result<void> line(std::string_view text);
    [[nodiscard]] Result<std::vector<MeshObject>> objects() const;
    [[nodiscard]] uint64_t line_number() const noexcept { return line_; }

  private:
    [[nodiscard]] std::unexpected<Error> fail(Code code, const std::string& detail) const {
        return std::unexpected(Error{code, file_ + ": line " + num(line_) + ": " + detail});
    }
    [[nodiscard]] Result<void> vertex();
    [[nodiscard]] Result<void> face();
    [[nodiscard]] Result<uint32_t> name_slot(std::string_view text, Names& names) const;
    [[nodiscard]] Result<uint32_t> resolve(std::string_view index, uint64_t defined, const Kind& kind) const;

    std::string file_;
    MeshUnit unit_;
    uint64_t line_ = 0;
    std::vector<std::string_view> fields_;  // this line's, reused

    std::vector<glm::dvec3> positions_;
    uint64_t texcoords_ = 0;
    uint64_t normals_ = 0;
    std::vector<Face> faces_;
    Names o_names_;
    Names g_names_;
    uint32_t o_ = 0;
    uint32_t g_ = 0;
    bool any_o_ = false;
};

Result<void> Reader::line(std::string_view text) {
    ++line_;
    if (line_ == 1 && text.starts_with(kBom)) text.remove_prefix(kBom.size());
    if (!text.empty() && text.back() == '\r') text.remove_suffix(1);

    fields_.clear();
    for (std::size_t i = 0; i < text.size();) {
        if (blank(text[i])) {
            ++i;
            continue;
        }
        const std::size_t start = i;
        while (i < text.size() && !blank(text[i])) ++i;
        fields_.push_back(text.substr(start, i - start));
    }
    if (fields_.empty() || fields_.front().front() == '#') return {};
    if (fields_.back().back() == '\\') {
        return fail(Code::unavailable,
                    "a line continued by a trailing '\\', which this build does not join (export without line "
                    "continuation)");
    }

    const std::string_view keyword = fields_.front();
    if (keyword == "v") return vertex();
    if (keyword == "f") return face();
    if (keyword == "vt") {
        ++texcoords_;
        return {};
    }
    if (keyword == "vn") {
        ++normals_;
        return {};
    }
    if (keyword == "o" || keyword == "g") {
        // The name is the rest of the line, after the keyword.
        const std::size_t after = static_cast<std::size_t>(keyword.data() - text.data()) + keyword.size();
        const bool is_o = keyword == "o";
        const Result<uint32_t> slot = name_slot(text.substr(after), is_o ? o_names_ : g_names_);
        if (!slot) return std::unexpected(slot.error());
        if (is_o) {
            o_ = *slot;
            any_o_ = true;
        } else {
            g_ = *slot;
        }
        return {};
    }
    if (keyword == "s" || keyword == "usemtl" || keyword == "mtllib") return {};
    return fail(Code::unavailable, "'" + std::string(keyword) +
                                       "' elements are not read: this build reads v and f (points, lines, "
                                       "free-form geometry and display attributes are not converted)");
}

Result<uint32_t> Reader::name_slot(std::string_view text, Names& names) const {
    const std::string_view name = trimmed(text);
    if (!is_utf8(name)) return fail(Code::invalid_argument, "the name is not UTF-8");
    if (name.empty()) return 0u;
    return names.slot(std::string(name));
}

Result<void> Reader::vertex() {
    const std::size_t n = fields_.size() - 1;
    if (n < 3 || n > 7) {
        return fail(Code::invalid_argument,
                    "v has " + num(n) + " numbers; it takes three, and up to four more (a weight or a colour)");
    }
    std::array<double, 3> p{};
    for (std::size_t k = 1; k <= n; ++k) {
        const std::string field(fields_[k]);
        double value = 0.0;
        const Number read = read_number(field, value);
        if (read == Number::not_a_number) return fail(Code::invalid_argument, "'" + field + "' is not a number");
        if (k > 3) continue;  // a weight or a colour: read, and ignored
        if (read == Number::out_of_range) {
            return fail(Code::invalid_argument, "'" + field + "' is out of double's range");
        }
        if (read == Number::not_finite) return fail(Code::invalid_argument, "'" + field + "' is not a finite number");
        p[k - 1] = value;
    }
    if (positions_.size() >= std::numeric_limits<uint32_t>::max()) {
        return fail(Code::capacity_exceeded,
                    "more than the " + num(std::numeric_limits<uint32_t>::max()) + " vertices a file may index");
    }
    positions_.emplace_back(p[0], p[1], p[2]);
    return {};
}

Result<uint32_t> Reader::resolve(std::string_view index, uint64_t defined, const Kind& kind) const {
    const std::string text(index);
    const std::string_view s = unsigned_plus(index);
    int64_t i = 0;
    const char* const end = s.data() + s.size();
    const std::from_chars_result r = std::from_chars(s.data(), end, i);
    const bool integer = r.ptr == end && (r.ec == std::errc{} || r.ec == std::errc::result_out_of_range);
    if (!integer || (r.ec == std::errc{} && i == 0)) {
        return fail(Code::invalid_argument, "index '" + text + "' is not a nonzero integer");
    }
    const auto d = static_cast<int64_t>(defined);
    if (r.ec == std::errc{} && i > 0 && i <= d) return static_cast<uint32_t>(i - 1);
    if (r.ec == std::errc{} && i < 0 && i >= -d) return static_cast<uint32_t>(d + i);
    return fail(Code::invalid_argument, std::string(kind.one) + " index " + text + " names none of the " +
                                            num(defined) + " " + std::string(kind.many) + " defined above it");
}

Result<void> Reader::face() {
    const std::size_t n = fields_.size() - 1;
    if (n < 3) return fail(Code::invalid_argument, "a face of " + num(n) + " corners; a face takes three or more");
    std::vector<uint32_t> corners(n);
    for (std::size_t k = 0; k < n; ++k) {
        const std::string_view corner = fields_[k + 1];
        // v, v/vt, v/vt/vn or v//vn: v never empty, vt empty only between
        // two slashes, vn never empty.
        const auto slashes = static_cast<std::size_t>(std::count(corner.begin(), corner.end(), '/'));
        const std::size_t first = corner.find('/');
        const std::size_t second = slashes == 2 ? corner.find('/', first + 1) : std::string_view::npos;
        const std::string_view vpart = corner.substr(0, first);
        std::string_view tpart;
        std::string_view npart;
        if (slashes == 1) tpart = corner.substr(first + 1);
        if (slashes == 2) {
            tpart = corner.substr(first + 1, second - first - 1);
            npart = corner.substr(second + 1);
        }
        if (slashes > 2 || vpart.empty() || (slashes == 1 && tpart.empty()) || (slashes == 2 && npart.empty())) {
            return fail(Code::invalid_argument,
                        "corner '" + std::string(corner) + "' is not v, v/vt, v/vt/vn or v//vn");
        }
        const Result<uint32_t> v = resolve(vpart, positions_.size(), kVertex);
        if (!v) return std::unexpected(v.error());
        if (!tpart.empty()) {
            if (const Result<uint32_t> vt = resolve(tpart, texcoords_, kTexture); !vt) {
                return std::unexpected(vt.error());
            }
        }
        if (!npart.empty()) {
            if (const Result<uint32_t> vn = resolve(npart, normals_, kNormal); !vn) {
                return std::unexpected(vn.error());
            }
        }
        corners[k] = *v;
    }
    if (n > 3) {
        std::vector<glm::dvec3> p(n);
        for (std::size_t k = 0; k < n; ++k) p[k] = positions_[corners[k]];
        const PolygonCheck check = check_polygon(p);
        const std::string face = "a face of " + num(n) + " vertices ";
        const std::string rest = "; this build fans only planar, strictly convex polygons (triangulate on export)";
        switch (check.verdict) {
            case Polygon::ok: break;
            case Polygon::no_area: return fail(Code::unavailable, face + "with no area" + rest);
            case Polygon::not_planar:
                return fail(Code::unavailable,
                            face + "that is not planar (at corner " + num(check.corner + 1) + ")" + rest);
            case Polygon::not_convex:
                return fail(Code::unavailable,
                            face + "that is not strictly convex (at corner " + num(check.corner + 1) + ")" + rest);
            case Polygon::winds_again:
                return fail(Code::unavailable,
                            face + "that is not strictly convex (it winds round more than once)" + rest);
        }
    }
    // Fanned from the first corner, keeping the face's winding.
    for (std::size_t k = 1; k + 1 < n; ++k) {
        Face f;
        f.o = o_;
        f.g = g_;
        f.v = {corners[0], corners[k], corners[k + 1]};
        faces_.push_back(f);
    }
    return {};
}

Result<std::vector<MeshObject>> Reader::objects() const {
    const auto refuse = [this](Code code, const std::string& detail) {
        return std::unexpected(Error{code, file_ + ": " + detail});
    };
    if (faces_.empty()) return refuse(Code::invalid_argument, "no faces");

    // The objects: by o, or by g in a file with no o, in the order of their
    // first face. object_of[slot] is a name's object.
    const Names& names = any_o_ ? o_names_ : g_names_;
    const auto slot_of = [this](const Face& f) { return any_o_ ? f.o : f.g; };
    std::vector<uint32_t> object_of(names.size(), kNone);
    std::vector<uint32_t> slots;
    std::vector<std::size_t> first(1, 0);  // object k's faces are order[first[k], first[k + 1])
    for (const Face& f : faces_) {
        uint32_t& k = object_of[slot_of(f)];
        if (k == kNone) {
            k = static_cast<uint32_t>(slots.size());
            slots.push_back(slot_of(f));
            first.push_back(0);
        }
        ++first[k + 1];
    }
    for (std::size_t k = 0; k < slots.size(); ++k) {
        const std::size_t triangles = first[k + 1];
        if (triangles > kMaxMeshTriangles) {
            return refuse(Code::capacity_exceeded, "object '" + names.name(slots[k]) + "': " + num(triangles) +
                                                       " triangles, more than the " + num(kMaxMeshTriangles) +
                                                       " a mesh may hold");
        }
        first[k + 1] += first[k];
    }
    // Each object's faces, in file order.
    std::vector<std::size_t> order(faces_.size());
    std::vector<std::size_t> next(first.begin(), first.end() - 1);
    for (std::size_t i = 0; i < faces_.size(); ++i) order[next[object_of[slot_of(faces_[i])]]++] = i;

    std::vector<MeshObject> out;
    out.reserve(slots.size());
    std::vector<uint32_t> local(positions_.size(), kNone);  // a file vertex's number in this object
    for (std::size_t k = 0; k < slots.size(); ++k) {
        std::vector<glm::dvec3> positions;
        std::vector<uint32_t> indices;
        indices.reserve(3 * (first[k + 1] - first[k]));
        for (std::size_t j = first[k]; j < first[k + 1]; ++j) {
            for (const uint32_t v : faces_[order[j]].v) {
                if (local[v] == kNone) {
                    local[v] = static_cast<uint32_t>(positions.size());
                    positions.push_back(positions_[v]);
                }
                indices.push_back(local[v]);
            }
        }
        for (std::size_t j = first[k]; j < first[k + 1]; ++j) {
            for (const uint32_t v : faces_[order[j]].v) local[v] = kNone;
        }
        const std::string& name = names.name(slots[k]);
        const MeshInput input = detail::weld_exact(positions, indices, unit_);
        auto built = make_mesh(input, file_ + ": object '" + name + "'");
        if (!built) return std::unexpected(built.error());
        out.push_back(MeshObject{name, std::move(*built)});
    }
    return out;
}

[[nodiscard]] Result<std::vector<MeshObject>> read(const std::filesystem::path& path, const std::string& file,
                                                   MeshUnit unit) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::unexpected(Error{Code::io_error, file + ": cannot read it"});
    Reader reader(file, path_text(path.stem()), unit);
    for (std::string text; std::getline(in, text);) {
        if (Result<void> r = reader.line(text); !r) return std::unexpected(r.error());
    }
    if (in.bad()) {
        return std::unexpected(
            Error{Code::io_error, file + ": cannot read it past line " + num(reader.line_number())});
    }
    return reader.objects();
}

}  // namespace

Result<std::vector<MeshObject>> read_obj(const std::filesystem::path& path, MeshUnit unit) {
    const std::string file = path_text(path);
    try {
        if (const std::string problem = detail::unit_problem(unit); !problem.empty()) {
            return std::unexpected(Error{Code::invalid_argument, file + ": " + problem});
        }
        return read(path, file, unit);
    } catch (const std::bad_alloc&) {
        return std::unexpected(Error{Code::capacity_exceeded, file + ": out of memory reading it"});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, file + ": unexpected failure: " + e.what()});
    }
}

}  // namespace spade
