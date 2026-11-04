// world/mesh_import/threemf.cpp -- the 3MF reader. See world/mesh_import.hpp
// for the contract, the refusals and their order.
//
// The archive is read whole and opened in memory with miniz (so the path
// never reaches a narrow-character fopen). _rels/.rels names the model part;
// each part's uncompressed size is checked before it is extracted. The model
// is parsed with pugixml, which is not namespace-aware, so the core
// namespace's prefix is found on the root and every core element is looked
// up by its qualified name. Objects are read in file order into a table;
// each build item is then taken down its components with an explicit stack,
// its transforms composed in double, and the joined soup goes to the shared
// exact weld and make_mesh().
#include "world/mesh_import.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <expected>
#include <fstream>
#include <ios>
#include <iterator>
#include <limits>
#include <map>
#include <new>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>
#include <miniz.h>
#include <pugixml.hpp>

#include "core/path_text.hpp"
#include "world/detail/weld.hpp"

namespace spade {
namespace {

constexpr std::string_view kCoreNs = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";
constexpr std::string_view kModelRelationship = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";
constexpr uint64_t kMaxVertices = std::numeric_limits<uint32_t>::max();

[[nodiscard]] std::string num(uint64_t v) {
    return std::to_string(v);
}

[[nodiscard]] Error fail(Code code, std::string detail) {
    return Error{code, std::move(detail)};
}

// A transform, a point (x, y, z) going to x m0j + y m1j + z m2j + m3j: the
// 3MF order, m00 m01 m02 m10 ... m32. The default is the identity.
struct Affine {
    std::array<double, 12> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
};

// `a`, then `b`.
[[nodiscard]] Affine then(const Affine& a, const Affine& b) noexcept {
    Affine r;
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            double v = a.m[i * 3] * b.m[j] + a.m[i * 3 + 1] * b.m[3 + j] + a.m[i * 3 + 2] * b.m[6 + j];
            if (i == 3) v += b.m[9 + j];
            r.m[i * 3 + j] = v;
        }
    }
    return r;
}

[[nodiscard]] glm::dvec3 apply(const Affine& t, const glm::dvec3& p) noexcept {
    const auto& m = t.m;
    return {p.x * m[0] + p.y * m[3] + p.z * m[6] + m[9], p.x * m[1] + p.y * m[4] + p.z * m[7] + m[10],
            p.x * m[2] + p.y * m[5] + p.z * m[8] + m[11]};
}

[[nodiscard]] double determinant(const Affine& t) noexcept {
    const auto& m = t.m;
    return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

[[nodiscard]] std::string_view without_plus(std::string_view s) noexcept {
    if (!s.empty() && s.front() == '+') s.remove_prefix(1);
    return s;
}

[[nodiscard]] std::optional<double> finite_number(std::string_view s) {
    s = without_plus(s);
    double v = 0.0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (s.empty() || ec != std::errc{} || ptr != s.data() + s.size() || !std::isfinite(v)) return std::nullopt;
    return v;
}

[[nodiscard]] std::optional<uint32_t> index_number(std::string_view s) {
    s = without_plus(s);
    uint32_t v = 0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (s.empty() || ec != std::errc{} || ptr != s.data() + s.size()) return std::nullopt;
    return v;
}

[[nodiscard]] bool xml_space(char c) noexcept {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

// The whitespace-separated fields of an attribute's value.
[[nodiscard]] std::vector<std::string_view> fields(std::string_view s) {
    std::vector<std::string_view> out;
    std::size_t at = 0;
    while (at < s.size()) {
        while (at < s.size() && xml_space(s[at])) ++at;
        std::size_t end = at;
        while (end < s.size() && !xml_space(s[end])) ++end;
        if (end > at) out.push_back(s.substr(at, end - at));
        at = end;
    }
    return out;
}

// A transform attribute: absent is the identity; otherwise twelve finite
// numbers, or the refusal's text.
[[nodiscard]] std::expected<Affine, std::string> transform_of(const pugi::xml_node& node) {
    const pugi::xml_attribute attr = node.attribute("transform");
    Affine t;
    if (!attr) return t;
    const std::vector<std::string_view> f = fields(attr.value());
    if (f.size() != 12) {
        return std::unexpected("transform \"" + std::string(attr.value()) + "\" is not twelve numbers");
    }
    for (std::size_t i = 0; i < 12; ++i) {
        const std::optional<double> v = finite_number(f[i]);
        if (!v) {
            return std::unexpected("transform \"" + std::string(attr.value()) + "\": entry " + num(i) +
                                   " is not a finite number");
        }
        t.m[i] = *v;
    }
    return t;
}

struct Component {
    std::size_t object = 0;  // its index in the object table
    Affine transform;
};

struct Object {
    uint32_t id = 0;
    std::string type;
    std::string name;
    bool is_mesh = false;
    std::vector<glm::dvec3> positions;
    std::vector<uint32_t> indices;
    std::vector<Component> components;
    // What taking it down its components gives, saturating past the caps:
    // triangles, vertices, and the objects visited.
    uint64_t triangles = 0;
    uint64_t vertices = 0;
    uint64_t visits = 0;
};

[[nodiscard]] uint64_t saturating_add(uint64_t a, uint64_t b) noexcept {
    return a > std::numeric_limits<uint64_t>::max() - b ? std::numeric_limits<uint64_t>::max() : a + b;
}

// The archive, opened in memory; ended on scope exit.
class Archive {
public:
    explicit Archive(std::span<const unsigned char> bytes) {
        ok_ = mz_zip_reader_init_mem(&zip_, bytes.data(), bytes.size(), 0) != 0;
    }
    ~Archive() {
        if (ok_) mz_zip_reader_end(&zip_);
    }
    Archive(const Archive&) = delete;
    Archive& operator=(const Archive&) = delete;

    [[nodiscard]] bool ok() const noexcept { return ok_; }
    [[nodiscard]] std::string last_error() { return mz_zip_get_error_string(mz_zip_get_last_error(&zip_)); }

    // The part named `name` (matched without regard to case): its archive
    // name and bytes; nullopt when the archive holds none; or the refusal.
    [[nodiscard]] Result<std::optional<std::pair<std::string, std::string>>> part(const std::string& name) {
        const int index = mz_zip_reader_locate_file(&zip_, name.c_str(), nullptr, 0);
        if (index < 0) return std::optional<std::pair<std::string, std::string>>{};
        const auto at = static_cast<mz_uint>(index);
        mz_zip_archive_file_stat stat{};
        if (!mz_zip_reader_file_stat(&zip_, at, &stat)) {
            return std::unexpected(fail(Code::invalid_argument, "part '" + name + "' cannot be read: " + last_error()));
        }
        std::string found = stat.m_filename;
        if (stat.m_uncomp_size > kMax3mfModelBytes) {
            return std::unexpected(fail(Code::capacity_exceeded,
                                        "part '" + found + "' is " + num(stat.m_uncomp_size) +
                                            " bytes uncompressed, more than the " + num(kMax3mfModelBytes) +
                                            " a part may hold"));
        }
        std::string bytes(static_cast<std::size_t>(stat.m_uncomp_size), '\0');
        if (!bytes.empty() && !mz_zip_reader_extract_to_mem(&zip_, at, bytes.data(), bytes.size(), 0)) {
            return std::unexpected(
                fail(Code::invalid_argument, "part '" + found + "' cannot be extracted: " + last_error()));
        }
        return std::optional<std::pair<std::string, std::string>>{std::pair{std::move(found), std::move(bytes)}};
    }

private:
    mz_zip_archive zip_{};
    bool ok_ = false;
};

[[nodiscard]] Result<pugi::xml_document> parse(const std::string& part, const std::string& bytes) {
    pugi::xml_document doc;
    const pugi::xml_parse_result parsed = doc.load_buffer(bytes.data(), bytes.size());
    if (!parsed) {
        const std::string at = num(static_cast<uint64_t>(parsed.offset));
        return std::unexpected(
            fail(Code::invalid_argument, part + ": not XML (" + parsed.description() + " at byte " + at + ")"));
    }
    return doc;
}

// The model part's name, from _rels/.rels.
[[nodiscard]] Result<std::string> model_part_name(Archive& zip) {
    const auto rels = zip.part("_rels/.rels");
    if (!rels) return std::unexpected(rels.error());
    if (!*rels) {
        return std::unexpected(fail(Code::invalid_argument, "no _rels/.rels part, so no model part is named"));
    }
    const auto doc = parse((*rels)->first, (*rels)->second);
    if (!doc) return std::unexpected(doc.error());
    for (const pugi::xml_node& r : doc->document_element().children()) {
        std::string_view n = r.name();
        if (const std::size_t colon = n.find(':'); colon != std::string_view::npos) n.remove_prefix(colon + 1);
        if (n != "Relationship" || std::string_view(r.attribute("Type").value()) != kModelRelationship) continue;
        std::string target = r.attribute("Target").value();
        if (target.starts_with('/')) target.erase(0, 1);
        if (target.empty()) {
            return std::unexpected(
                fail(Code::invalid_argument, (*rels)->first + ": the model relationship names no part"));
        }
        return target;
    }
    return std::unexpected(fail(Code::invalid_argument, (*rels)->first + ": no relationship of type " +
                                                            std::string(kModelRelationship)));
}

[[nodiscard]] std::optional<MeshUnit> unit_named(std::string_view unit) noexcept {
    if (unit == "micron") return MeshUnit::micrometre;
    if (unit == "millimeter") return MeshUnit::millimetre;
    if (unit == "centimeter") return MeshUnit::centimetre;
    if (unit == "inch") return MeshUnit::inch;
    if (unit == "foot") return MeshUnit::foot;
    if (unit == "meter") return MeshUnit::metre;
    return std::nullopt;
}

[[nodiscard]] bool known_type(std::string_view type) noexcept {
    return type == "model" || type == "solidsupport" || type == "support" || type == "surface" || type == "other";
}

// One <object>: its id, type and name, then its mesh or its components.
// `table` and `by_id` hold the objects above it.
[[nodiscard]] Result<Object> read_object(const pugi::xml_node& node, std::size_t ordinal, const std::string& cp,
                                         const std::vector<Object>& table,
                                         const std::map<uint32_t, std::size_t>& by_id) {
    Object o;
    const std::optional<uint32_t> id = index_number(node.attribute("id").value());
    if (!id || *id == 0) {
        return std::unexpected(
            fail(Code::invalid_argument, "object " + num(ordinal) + " of the resources: no positive id"));
    }
    o.id = *id;
    const std::string where = "object " + num(o.id) + ": ";
    if (by_id.contains(o.id)) return std::unexpected(fail(Code::invalid_argument, where + "its id is used twice"));
    const pugi::xml_attribute type = node.attribute("type");
    o.type = type ? type.value() : "model";
    if (!known_type(o.type)) {
        return std::unexpected(fail(Code::invalid_argument, where + "type '" + o.type +
                                                                "' is not model, solidsupport, support, surface "
                                                                "or other"));
    }
    const pugi::xml_attribute name = node.attribute("name");
    o.name = name ? name.value() : "object " + num(o.id);

    const pugi::xml_node mesh = node.child((cp + "mesh").c_str());
    const pugi::xml_node components = node.child((cp + "components").c_str());
    if (!mesh && !components) {
        return std::unexpected(fail(Code::invalid_argument, where + "neither a mesh nor components"));
    }
    if (mesh && components) {
        return std::unexpected(fail(Code::invalid_argument, where + "both a mesh and components"));
    }

    if (mesh) {
        o.is_mesh = true;
        std::size_t v = 0;
        const pugi::xml_node vertices = mesh.child((cp + "vertices").c_str());
        const std::string vertex_tag = cp + "vertex";
        for (const pugi::xml_node& vertex : vertices.children(vertex_tag.c_str())) {
            glm::dvec3 p{};
            for (int axis = 0; axis < 3; ++axis) {
                const char* const key = axis == 0 ? "x" : axis == 1 ? "y" : "z";
                const pugi::xml_attribute a = vertex.attribute(key);
                const std::string at = where + "vertex " + num(v) + ": " + key;
                if (!a) return std::unexpected(fail(Code::invalid_argument, at + " is missing"));
                const std::optional<double> value = finite_number(a.value());
                if (!value) {
                    return std::unexpected(fail(Code::invalid_argument,
                                                at + " \"" + std::string(a.value()) + "\" is not a finite number"));
                }
                p[axis] = *value;
            }
            o.positions.push_back(p);
            ++v;
        }
        // A named string: children() keeps the pointer it is given, past the
        // temporary a range-for's initialiser would make of an expression.
        const std::string triangle_tag = cp + "triangle";
        std::size_t t = 0;
        for (const pugi::xml_node& triangle :
             mesh.child((cp + "triangles").c_str()).children(triangle_tag.c_str())) {
            if (t >= kMaxMeshTriangles) {
                return std::unexpected(fail(Code::capacity_exceeded, where + "more than the " +
                                                                         num(kMaxMeshTriangles) +
                                                                         " triangles a mesh may hold"));
            }
            for (const char* const key : {"v1", "v2", "v3"}) {
                const pugi::xml_attribute a = triangle.attribute(key);
                const std::optional<uint32_t> index = index_number(a.value());
                const std::string at = where + "triangle " + num(t) + ": " + key;
                if (!a) return std::unexpected(fail(Code::invalid_argument, at + " is missing"));
                if (!index) {
                    return std::unexpected(
                        fail(Code::invalid_argument, at + " \"" + std::string(a.value()) + "\" is not an index"));
                }
                if (*index >= o.positions.size()) {
                    return std::unexpected(fail(Code::invalid_argument, at + " " + num(*index) + " is past its " +
                                                                            num(o.positions.size()) + " vertices"));
                }
                o.indices.push_back(*index);
            }
            ++t;
        }
        o.triangles = o.indices.size() / 3;
        o.vertices = o.positions.size();
        o.visits = 1;
        return o;
    }

    o.visits = 1;
    std::size_t c = 0;
    const std::string component_tag = cp + "component";
    for (const pugi::xml_node& component : components.children(component_tag.c_str())) {
        const std::string at = where + "component " + num(c) + ": ";
        const pugi::xml_attribute ref = component.attribute("objectid");
        const std::optional<uint32_t> child = index_number(ref.value());
        const auto found = child ? by_id.find(*child) : by_id.end();
        if (found == by_id.end()) {
            return std::unexpected(fail(Code::invalid_argument, at + "objectid " + std::string(ref.value()) +
                                                                    " names no object defined above it"));
        }
        const Object& target = table[found->second];
        if (target.type != "model") {
            return std::unexpected(fail(Code::invalid_argument, at + "objectid " + num(target.id) + " is of type " +
                                                                    target.type + ", not model"));
        }
        auto transform = transform_of(component);
        if (!transform) return std::unexpected(fail(Code::invalid_argument, at + transform.error()));
        o.components.push_back(Component{found->second, *transform});
        o.triangles = saturating_add(o.triangles, target.triangles);
        o.vertices = saturating_add(o.vertices, target.vertices);
        o.visits = saturating_add(o.visits, target.visits);
        ++c;
    }
    return o;
}

// One build item's object, taken down its components and joined: positions
// in the model's unit, the winding reversed under a mirroring transform.
void gather(const std::vector<Object>& table, std::size_t root, const Affine& item, std::vector<glm::dvec3>& positions,
            std::vector<uint32_t>& indices) {
    std::vector<std::pair<std::size_t, Affine>> stack{{root, item}};
    while (!stack.empty()) {
        const auto [index, transform] = stack.back();
        stack.pop_back();
        const Object& o = table[index];
        if (o.is_mesh) {
            const auto base = static_cast<uint32_t>(positions.size());
            for (const glm::dvec3& p : o.positions) positions.push_back(apply(transform, p));
            const bool mirrored = determinant(transform) < 0.0;
            for (std::size_t t = 0; t < o.indices.size(); t += 3) {
                indices.push_back(base + o.indices[t]);
                indices.push_back(base + o.indices[t + (mirrored ? 2 : 1)]);
                indices.push_back(base + o.indices[t + (mirrored ? 1 : 2)]);
            }
            continue;
        }
        // Pushed last to first, so the first component is taken first.
        for (auto c = o.components.rbegin(); c != o.components.rend(); ++c) {
            stack.emplace_back(c->object, then(c->transform, transform));
        }
    }
}

[[nodiscard]] Result<std::vector<MeshObject>> read(const std::filesystem::path& path, const std::string& file) {
    const auto refuse = [&file](const Error& e) { return std::unexpected(Error{e.code, file + ": " + e.context}); };

    std::vector<unsigned char> bytes;
    {
        std::ifstream in(path, std::ios::binary);
        if (!in) return refuse(fail(Code::io_error, "cannot read it"));
        bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) return refuse(fail(Code::io_error, "cannot read it"));
    }
    Archive zip(bytes);
    if (!zip.ok()) {
        return refuse(
            fail(Code::invalid_argument, "not a ZIP archive, which a 3MF package is (" + zip.last_error() + ")"));
    }
    const Result<std::string> target = model_part_name(zip);
    if (!target) return refuse(target.error());
    const auto part = zip.part(*target);
    if (!part) return refuse(part.error());
    if (!*part) {
        return refuse(fail(Code::invalid_argument,
                           "_rels/.rels names the model part '" + *target + "', which the archive does not hold"));
    }
    const std::string& part_name = (*part)->first;
    const auto in_part = [&](const Error& e) { return refuse(Error{e.code, part_name + ": " + e.context}); };
    const auto doc = parse(part_name, (*part)->second);
    if (!doc) return refuse(doc.error());

    // The root, and the core namespace's prefix ("" or "<prefix>:").
    const pugi::xml_node root = doc->document_element();
    const std::string_view root_name = root.name();
    std::string cp;
    const std::size_t colon = root_name.find(':');
    if (colon != std::string_view::npos) cp = std::string(root_name.substr(0, colon + 1));
    const std::string declares = cp.empty() ? "xmlns" : "xmlns:" + cp.substr(0, cp.size() - 1);
    const std::string_view root_ns = root.attribute(declares.c_str()).value();
    if (root_name.substr(cp.size()) != "model" || root_ns != kCoreNs) {
        return in_part(fail(Code::invalid_argument, "the root <" + std::string(root_name) +
                                                        "> is not a core <model> (namespace " + std::string(kCoreNs) +
                                                        ")"));
    }
    // This build reads no extension, so the first required one is refused.
    if (const auto required = fields(root.attribute("requiredextensions").value()); !required.empty()) {
        const std::string prefix(required.front());
        const pugi::xml_attribute ns = root.attribute(("xmlns:" + prefix).c_str());
        if (!ns) {
            return in_part(fail(Code::invalid_argument,
                                "requiredextensions names '" + prefix + "', a prefix the model does not declare"));
        }
        return in_part(fail(Code::unavailable, "it requires the extension '" + prefix + "' (" + ns.value() +
                                                   "), which this build does not read"));
    }
    const pugi::xml_attribute unit_attr = root.attribute("unit");
    const std::optional<MeshUnit> unit = unit_attr ? unit_named(unit_attr.value()) : MeshUnit::millimetre;
    if (!unit) {
        return in_part(fail(Code::invalid_argument,
                            "unit '" + std::string(unit_attr.value()) +
                                "' is not micron, millimeter, centimeter, inch, foot or meter"));
    }

    std::vector<Object> table;
    std::map<uint32_t, std::size_t> by_id;
    std::size_t ordinal = 0;
    const std::string object_tag = cp + "object";
    for (const pugi::xml_node& node : root.child((cp + "resources").c_str()).children(object_tag.c_str())) {
        Result<Object> o = read_object(node, ordinal++, cp, table, by_id);
        if (!o) return in_part(o.error());
        by_id.emplace(o->id, table.size());
        table.push_back(std::move(*o));
    }

    std::vector<MeshObject> out;
    std::size_t k = 0;
    const std::string item_tag = cp + "item";
    for (const pugi::xml_node& item : root.child((cp + "build").c_str()).children(item_tag.c_str())) {
        const std::string at = "build item " + num(k++) + ": ";
        const pugi::xml_attribute ref = item.attribute("objectid");
        const std::optional<uint32_t> id = index_number(ref.value());
        const auto found = id ? by_id.find(*id) : by_id.end();
        if (found == by_id.end()) {
            return in_part(
                fail(Code::invalid_argument, at + "objectid " + std::string(ref.value()) + " names no object"));
        }
        auto transform = transform_of(item);
        if (!transform) return in_part(fail(Code::invalid_argument, at + transform.error()));
        const Object& o = table[found->second];
        if (o.type != "model") continue;
        if (o.triangles > kMaxMeshTriangles || o.visits > kMaxMeshTriangles) {
            return in_part(fail(Code::capacity_exceeded, at + "object " + num(o.id) + " holds more than the " +
                                                             num(kMaxMeshTriangles) + " triangles a mesh may hold"));
        }
        if (o.vertices > kMaxVertices) {
            return in_part(fail(Code::capacity_exceeded, at + "object " + num(o.id) + " holds more than the " +
                                                             num(kMaxVertices) + " vertices an object may index"));
        }
        std::vector<glm::dvec3> positions;
        std::vector<uint32_t> indices;
        positions.reserve(static_cast<std::size_t>(o.vertices));
        indices.reserve(static_cast<std::size_t>(3 * o.triangles));
        gather(table, found->second, *transform, positions, indices);
        const MeshInput input = detail::weld_exact(positions, indices, *unit);
        auto built = make_mesh(input, file + ": object '" + o.name + "'");
        if (!built) return std::unexpected(built.error());
        out.push_back(MeshObject{o.name, std::move(*built), *unit});
    }
    if (out.empty()) return in_part(fail(Code::invalid_argument, "no build item of type model"));
    return out;
}

}  // namespace

Result<std::vector<MeshObject>> read_3mf(const std::filesystem::path& path) {
    const std::string file = path_text(path);
    try {
        return read(path, file);
    } catch (const std::bad_alloc&) {
        return std::unexpected(Error{Code::capacity_exceeded, file + ": out of memory reading it"});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, file + ": unexpected failure: " + e.what()});
    }
}

}  // namespace spade
