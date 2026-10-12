// world/mesh.cpp -- see mesh.hpp for the contract, the refusal order and the
// content hash's spelling.
//
// Every check is a function that returns its refusal's detail, or nothing,
// and make_mesh() runs them in mesh.hpp's order. Each runs in index order;
// every sort is under a total order (ties broken by index), so a refusal's
// count and its first elements are the same on every platform. Geometry runs
// in double, on the stored fp32 positions promoted exactly, with only
// + - * / and sqrt (TD-3).
#include "world/mesh.hpp"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <optional>
#include <string>
#include <utility>

#include "core/rng.hpp"
#include "core/validate.hpp"

namespace spade {
namespace {

// A refusal lists at most this many offending elements.
constexpr std::size_t kMaxReported = 8;

using Detail = std::optional<std::string>;

// Counts every offender and keeps the first kMaxReported descriptions. The
// description is built only for the ones kept.
class Offenders {
public:
    template <class Describe>
    void add(Describe&& describe) {
        ++count_;
        if (first_.size() < kMaxReported) first_.push_back(describe());
    }

    // "<count> <noun>, first <e> <e> ...", or nothing when there is none.
    [[nodiscard]] Detail detail(std::string_view one, std::string_view many) const {
        if (count_ == 0) return std::nullopt;
        std::string s = std::to_string(count_);
        s += ' ';
        s += count_ == 1 ? one : many;
        s += ", first";
        for (const std::string& e : first_) {
            s += ' ';
            s += e;
        }
        return s;
    }

private:
    std::size_t count_ = 0;
    std::vector<std::string> first_;
};

[[nodiscard]] std::string pair_text(uint64_t a, uint64_t b) {
    return "(" + std::to_string(a) + ", " + std::to_string(b) + ")";
}

[[nodiscard]] std::string shortest(double v) {
    char buf[32];
    const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, r.ptr);
}

[[nodiscard]] glm::dvec3 widen(const glm::vec3& p) noexcept {
    return {static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)};
}

// Pinned forms: each component left to right, so the order is the source's.
[[nodiscard]] glm::dvec3 cross_d(const glm::dvec3& a, const glm::dvec3& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

[[nodiscard]] double dot_d(const glm::dvec3& a, const glm::dvec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Union-find whose root is always the smallest element of its set (the larger
// root joins the smaller), so the result never depends on the join order.
class UnionFind {
public:
    explicit UnionFind(std::size_t n) : parent_(n) {
        std::iota(parent_.begin(), parent_.end(), std::size_t{0});
    }

    [[nodiscard]] std::size_t find(std::size_t x) noexcept {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }

    void join(std::size_t a, std::size_t b) noexcept {
        a = find(a);
        b = find(b);
        if (a < b) {
            parent_[b] = a;
        } else if (b < a) {
            parent_[a] = b;
        }
    }

private:
    std::vector<std::size_t> parent_;
};

// One use of an edge by a triangle: the edge as (lower index, higher index),
// and whether the triangle traverses it lower to higher (dir 0) or back (1).
struct EdgeUse {
    uint32_t lo = 0;
    uint32_t hi = 0;
    uint32_t tri = 0;
    uint32_t dir = 0;
};

// --- the checks, in mesh.hpp's order ---------------------------------------

[[nodiscard]] Detail check_index_range(const std::vector<uint32_t>& idx, std::size_t vertex_count) {
    Offenders bad;
    for (std::size_t i = 0; i < idx.size(); ++i) {
        if (idx[i] >= vertex_count) {
            bad.add([&] { return std::to_string(idx[i]) + " (triangle " + std::to_string(i / 3) + ")"; });
        }
    }
    const std::string of = " out of range of " + std::to_string(vertex_count) + " vertices";
    return bad.detail("index" + of, "indices" + of);
}

[[nodiscard]] Detail check_input_finite(const std::vector<glm::dvec3>& positions) {
    Offenders bad;
    for (std::size_t v = 0; v < positions.size(); ++v) {
        const glm::dvec3& p = positions[v];
        if (!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))) {
            bad.add([v] { return std::to_string(v); });
        }
    }
    return bad.detail("position not finite", "positions not finite");
}

// Scales to metres in double and rounds to fp32 once: float(v * scale).
[[nodiscard]] std::vector<glm::vec3> to_metres(const std::vector<glm::dvec3>& positions, double scale) {
    std::vector<glm::vec3> out;
    out.reserve(positions.size());
    for (const glm::dvec3& p : positions) {
        out.emplace_back(static_cast<float>(p.x * scale), static_cast<float>(p.y * scale),
                         static_cast<float>(p.z * scale));
    }
    return out;
}

[[nodiscard]] Detail check_scaled_finite(const std::vector<glm::vec3>& positions) {
    Offenders bad;
    for (std::size_t v = 0; v < positions.size(); ++v) {
        if (!finite(positions[v])) bad.add([v] { return std::to_string(v); });
    }
    return bad.detail("position not finite after scaling to fp32 metres",
                      "positions not finite after scaling to fp32 metres");
}

[[nodiscard]] Detail check_referenced(const std::vector<uint32_t>& idx, std::size_t vertex_count) {
    std::vector<uint8_t> referenced(vertex_count, 0);
    for (const uint32_t i : idx) referenced[i] = 1;
    Offenders bad;
    for (std::size_t v = 0; v < vertex_count; ++v) {
        if (referenced[v] == 0) bad.add([v] { return std::to_string(v); });
    }
    return bad.detail("unreferenced vertex", "unreferenced vertices");
}

// Value equality on the fp32 positions, so -0 equals +0. Sorted by
// (x, y, z, index); in each run of equal positions the lowest index is kept
// and every other vertex is reported against it.
[[nodiscard]] Detail check_coincident(const std::vector<glm::vec3>& positions) {
    // Every vertex is referenced by a uint32_t index (checked before this),
    // so every vertex index fits in one.
    std::vector<uint32_t> order(positions.size());
    std::iota(order.begin(), order.end(), uint32_t{0});
    std::sort(order.begin(), order.end(), [&positions](uint32_t a, uint32_t b) {
        const glm::vec3& pa = positions[a];
        const glm::vec3& pb = positions[b];
        if (pa.x != pb.x) return pa.x < pb.x;
        if (pa.y != pb.y) return pa.y < pb.y;
        if (pa.z != pb.z) return pa.z < pb.z;
        return a < b;
    });
    const auto same = [&positions](uint32_t a, uint32_t b) {
        const glm::vec3& pa = positions[a];
        const glm::vec3& pb = positions[b];
        return pa.x == pb.x && pa.y == pb.y && pa.z == pb.z;
    };
    std::vector<std::pair<uint32_t, uint32_t>> repeats;  // (vertex, the earlier vertex it sits on)
    for (std::size_t i = 0; i < order.size();) {
        std::size_t j = i + 1;
        while (j < order.size() && same(order[j], order[i])) {
            repeats.emplace_back(order[j], order[i]);
            ++j;
        }
        i = j;
    }
    std::sort(repeats.begin(), repeats.end());
    Offenders bad;
    for (const auto& [v, earlier] : repeats) {
        bad.add([v, earlier] { return std::to_string(v) + " (= " + std::to_string(earlier) + ")"; });
    }
    return bad.detail("vertex at the position of an earlier vertex (weld first)",
                      "vertices at the position of an earlier vertex (weld first)");
}

[[nodiscard]] Detail check_repeated_index(const std::vector<uint32_t>& idx) {
    Offenders bad;
    for (std::size_t t = 0; t < idx.size() / 3; ++t) {
        const uint32_t a = idx[3 * t];
        const uint32_t b = idx[3 * t + 1];
        const uint32_t c = idx[3 * t + 2];
        if (a == b || b == c || a == c) {
            bad.add([&] {
                return std::to_string(t) + " (" + std::to_string(a) + ", " + std::to_string(b) + ", " +
                       std::to_string(c) + ")";
            });
        }
    }
    return bad.detail("triangle repeats an index", "triangles repeat an index");
}

// |e1 x e2| / 2 <= 1e-12 * D^2, in double, D the bounds' diagonal.
[[nodiscard]] Detail check_degenerate(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& idx,
                                      const glm::dvec3& lo, const glm::dvec3& hi) {
    const glm::dvec3 diagonal = hi - lo;
    const double floor_area = 1e-12 * dot_d(diagonal, diagonal);
    Offenders bad;
    for (std::size_t t = 0; t < idx.size() / 3; ++t) {
        const glm::dvec3 a = widen(positions[idx[3 * t]]);
        const glm::dvec3 b = widen(positions[idx[3 * t + 1]]);
        const glm::dvec3 c = widen(positions[idx[3 * t + 2]]);
        const glm::dvec3 n = cross_d(b - a, c - a);
        const double area = 0.5 * std::sqrt(dot_d(n, n));
        if (area <= floor_area) bad.add([t] { return std::to_string(t); });
    }
    return bad.detail("degenerate triangle (area <= 1e-12 D^2, D the bounds' diagonal)",
                      "degenerate triangles (area <= 1e-12 D^2, D the bounds' diagonal)");
}

// The same three vertices in either orientation. Each triangle's sorted
// indices, sorted with the triangle index as the tie-break; in each run the
// lowest triangle is kept and every other is reported against it.
[[nodiscard]] Detail check_duplicate_triangles(const std::vector<uint32_t>& idx) {
    struct Key {
        uint32_t v[3];
        uint32_t tri;
    };
    std::vector<Key> keys;
    keys.reserve(idx.size() / 3);
    for (std::size_t t = 0; t < idx.size() / 3; ++t) {
        Key k{{idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]}, static_cast<uint32_t>(t)};
        std::sort(std::begin(k.v), std::end(k.v));
        keys.push_back(k);
    }
    const auto same = [](const Key& a, const Key& b) {
        return a.v[0] == b.v[0] && a.v[1] == b.v[1] && a.v[2] == b.v[2];
    };
    std::sort(keys.begin(), keys.end(), [](const Key& a, const Key& b) {
        if (a.v[0] != b.v[0]) return a.v[0] < b.v[0];
        if (a.v[1] != b.v[1]) return a.v[1] < b.v[1];
        if (a.v[2] != b.v[2]) return a.v[2] < b.v[2];
        return a.tri < b.tri;
    });
    std::vector<std::pair<uint32_t, uint32_t>> repeats;  // (triangle, the earlier triangle it repeats)
    for (std::size_t i = 0; i < keys.size();) {
        std::size_t j = i + 1;
        while (j < keys.size() && same(keys[j], keys[i])) {
            repeats.emplace_back(keys[j].tri, keys[i].tri);
            ++j;
        }
        i = j;
    }
    std::sort(repeats.begin(), repeats.end());
    Offenders bad;
    for (const auto& [t, earlier] : repeats) {
        bad.add([t, earlier] { return std::to_string(t) + " (= " + std::to_string(earlier) + ")"; });
    }
    return bad.detail("duplicate triangle", "duplicate triangles");
}

// Edge topology: one sorted vector of (lower index, higher index, triangle,
// direction). No hash map.
[[nodiscard]] std::vector<EdgeUse> edge_uses(const std::vector<uint32_t>& idx) {
    std::vector<EdgeUse> uses;
    uses.reserve(idx.size());
    for (std::size_t t = 0; t < idx.size() / 3; ++t) {
        for (std::size_t k = 0; k < 3; ++k) {
            const uint32_t a = idx[3 * t + k];
            const uint32_t b = idx[3 * t + (k + 1) % 3];
            uses.push_back({std::min(a, b), std::max(a, b), static_cast<uint32_t>(t), a < b ? 0u : 1u});
        }
    }
    std::sort(uses.begin(), uses.end(), [](const EdgeUse& a, const EdgeUse& b) {
        if (a.lo != b.lo) return a.lo < b.lo;
        if (a.hi != b.hi) return a.hi < b.hi;
        return a.tri < b.tri;
    });
    return uses;
}

// Open, then non-manifold, then misoriented: the first kind present is the
// refusal. When none is, every edge has exactly two uses, adjacent in `uses`.
[[nodiscard]] Detail check_edges(const std::vector<EdgeUse>& uses) {
    Offenders open;
    Offenders crowded;
    Offenders misoriented;
    for (std::size_t i = 0; i < uses.size();) {
        std::size_t j = i + 1;
        while (j < uses.size() && uses[j].lo == uses[i].lo && uses[j].hi == uses[i].hi) ++j;
        const EdgeUse& e = uses[i];
        const auto edge = [&e] { return pair_text(e.lo, e.hi); };
        if (j - i == 1) {
            open.add(edge);
        } else if (j - i >= 3) {
            crowded.add(edge);
        } else if (uses[i].dir == uses[i + 1].dir) {
            misoriented.add(edge);
        }
        i = j;
    }
    if (Detail d = open.detail("open edge", "open edges")) return d;
    if (Detail d = crowded.detail("non-manifold edge (three or more triangles)",
                                  "non-manifold edges (three or more triangles)")) {
        return d;
    }
    return misoriented.detail("misoriented edge (two triangles traverse it in the same direction)",
                              "misoriented edges (two triangles traverse them in the same direction)");
}

// The corner of triangle t at vertex v, as 3t + k. v is one of t's indices.
[[nodiscard]] std::size_t corner_of(const std::vector<uint32_t>& idx, uint32_t t, uint32_t v) noexcept {
    const std::size_t base = 3 * static_cast<std::size_t>(t);
    return idx[base] == v ? base : (idx[base + 1] == v ? base + 1 : base + 2);
}

// PRECONDITION: check_edges() passed. Corners at a vertex are joined across
// each edge at that vertex; the triangles around a vertex then form one fan
// (one set of corners) on a manifold vertex, and more on a non-manifold one.
[[nodiscard]] Detail check_fans(const std::vector<uint32_t>& idx, const std::vector<EdgeUse>& uses,
                                std::size_t vertex_count) {
    UnionFind corners(idx.size());
    for (std::size_t i = 0; i < uses.size(); i += 2) {
        const EdgeUse& a = uses[i];
        const EdgeUse& b = uses[i + 1];
        corners.join(corner_of(idx, a.tri, a.lo), corner_of(idx, b.tri, a.lo));
        corners.join(corner_of(idx, a.tri, a.hi), corner_of(idx, b.tri, a.hi));
    }
    constexpr std::size_t kNone = static_cast<std::size_t>(-1);
    std::vector<std::size_t> fan(vertex_count, kNone);
    std::vector<uint8_t> split(vertex_count, 0);
    for (std::size_t c = 0; c < idx.size(); ++c) {
        const std::size_t root = corners.find(c);
        std::size_t& seen = fan[idx[c]];
        if (seen == kNone) {
            seen = root;
        } else if (seen != root) {
            split[idx[c]] = 1;
        }
    }
    Offenders bad;
    for (std::size_t v = 0; v < vertex_count; ++v) {
        if (split[v] != 0) bad.add([v] { return std::to_string(v); });
    }
    return bad.detail("non-manifold vertex (more than one fan of triangles)",
                      "non-manifold vertices (more than one fan of triangles)");
}

// The total signed volume, sum of a . (b x c) / 6 over the triangles in
// order, taken about the bounds' centre to limit cancellation.
[[nodiscard]] Detail check_volume(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& idx,
                                  const glm::dvec3& lo, const glm::dvec3& hi) {
    const glm::dvec3 centre = (lo + hi) * 0.5;
    double six_volume = 0.0;
    for (std::size_t t = 0; t < idx.size() / 3; ++t) {
        const glm::dvec3 a = widen(positions[idx[3 * t]]) - centre;
        const glm::dvec3 b = widen(positions[idx[3 * t + 1]]) - centre;
        const glm::dvec3 c = widen(positions[idx[3 * t + 2]]) - centre;
        six_volume += dot_d(a, cross_d(b, c));
    }
    const double volume = six_volume / 6.0;
    if (volume > 0.0) return std::nullopt;
    return "inside out: the total signed volume is " + shortest(volume) +
           " m^3, not positive (triangles must be counter-clockwise seen from outside)";
}

// PRECONDITION: check_edges() passed. Triangles joined across shared edges;
// shells numbered in order of their lowest triangle index.
[[nodiscard]] std::vector<uint32_t> label_shells(const std::vector<EdgeUse>& uses, std::size_t triangle_count,
                                                 uint32_t& shell_count) {
    UnionFind tris(triangle_count);
    for (std::size_t i = 0; i < uses.size(); i += 2) tris.join(uses[i].tri, uses[i + 1].tri);
    std::vector<uint32_t> shell(triangle_count, 0);
    shell_count = 0;
    for (std::size_t t = 0; t < triangle_count; ++t) {
        const std::size_t root = tris.find(t);
        // The root is the smallest triangle of its shell, so it is labelled first.
        shell[t] = root == t ? shell_count++ : shell[root];
    }
    return shell;
}

// FNV-1a 64, spelt as mesh.hpp says.
[[nodiscard]] uint64_t content_hash(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& idx) {
    uint64_t h = rng::kFnv1aOffsetBasis;
    const auto u32 = [&h](uint32_t v) {
        for (int i = 0; i < 4; ++i) {
            h ^= static_cast<uint8_t>(v >> (8 * i));
            h *= rng::kFnv1aPrime;
        }
    };
    // Both counts fit: a closed 2-manifold has at most as many vertices as
    // triangles, and there are at most kMaxMeshTriangles of those.
    u32(static_cast<uint32_t>(positions.size()));
    u32(static_cast<uint32_t>(idx.size() / 3));
    for (const glm::vec3& p : positions) {
        u32(std::bit_cast<uint32_t>(p.x));
        u32(std::bit_cast<uint32_t>(p.y));
        u32(std::bit_cast<uint32_t>(p.z));
    }
    for (const uint32_t i : idx) u32(i);
    return h;
}

}  // namespace

double mesh_unit_scale(MeshUnit unit) noexcept {
    switch (unit) {
        case MeshUnit::metre: return 1.0;
        case MeshUnit::millimetre: return 0.001;
        case MeshUnit::centimetre: return 0.01;
        case MeshUnit::micrometre: return 1e-6;
        case MeshUnit::inch: return 0.0254;
        case MeshUnit::foot: return 0.3048;
        case MeshUnit::unspecified: break;
    }
    return 0.0;
}

Result<std::shared_ptr<const TriangleMesh>> make_mesh(const MeshInput& input, std::string_view name) {
    const auto refuse = [name](Code code, std::string_view detail) {
        std::string context(name);
        context += ": ";
        context += detail;
        return std::unexpected(Error{code, std::move(context)});
    };

    const double scale = mesh_unit_scale(input.unit);
    if (scale == 0.0) {
        if (input.unit == MeshUnit::unspecified) {
            return refuse(Code::invalid_argument,
                          "the unit is unspecified (declare the unit the positions are in)");
        }
        return refuse(Code::invalid_argument,
                      "unit " + std::to_string(static_cast<uint32_t>(input.unit)) + " is not a MeshUnit");
    }

    const std::vector<uint32_t>& idx = input.indices;
    if (idx.empty()) return refuse(Code::invalid_argument, "no triangles");
    if (idx.size() % 3 != 0) {
        return refuse(Code::invalid_argument, std::to_string(idx.size()) + " indices, not a multiple of 3");
    }
    const std::size_t triangle_count = idx.size() / 3;
    if (triangle_count > kMaxMeshTriangles) {
        return refuse(Code::capacity_exceeded, std::to_string(triangle_count) + " triangles, more than the " +
                                                   std::to_string(kMaxMeshTriangles) + " a mesh may hold");
    }

    const std::size_t vertex_count = input.positions.size();
    if (Detail d = check_index_range(idx, vertex_count)) return refuse(Code::invalid_argument, *d);
    if (Detail d = check_input_finite(input.positions)) return refuse(Code::invalid_argument, *d);
    std::vector<glm::vec3> positions = to_metres(input.positions, scale);
    if (Detail d = check_scaled_finite(positions)) return refuse(Code::invalid_argument, *d);
    if (Detail d = check_referenced(idx, vertex_count)) return refuse(Code::invalid_argument, *d);
    if (Detail d = check_coincident(positions)) return refuse(Code::invalid_argument, *d);
    if (Detail d = check_repeated_index(idx)) return refuse(Code::invalid_argument, *d);

    // The bounds, in double. Every vertex is referenced, so each counts.
    glm::dvec3 lo = widen(positions[0]);
    glm::dvec3 hi = lo;
    for (const glm::vec3& p : positions) {
        const glm::dvec3 q = widen(p);
        lo = {std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
        hi = {std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
    }

    if (Detail d = check_degenerate(positions, idx, lo, hi)) return refuse(Code::invalid_argument, *d);
    if (Detail d = check_duplicate_triangles(idx)) return refuse(Code::invalid_argument, *d);
    const std::vector<EdgeUse> uses = edge_uses(idx);
    if (Detail d = check_edges(uses)) return refuse(Code::invalid_argument, *d);
    if (Detail d = check_fans(idx, uses, vertex_count)) return refuse(Code::invalid_argument, *d);
    if (Detail d = check_volume(positions, idx, lo, hi)) return refuse(Code::invalid_argument, *d);

    std::shared_ptr<TriangleMesh> mesh(new TriangleMesh());
    mesh->triangle_shell_ = label_shells(uses, triangle_count, mesh->shell_count_);
    mesh->content_hash_ = content_hash(positions, idx);
    mesh->positions_ = std::move(positions);
    mesh->indices_ = idx;
    return std::shared_ptr<const TriangleMesh>(std::move(mesh));
}

// RED stubs (Task B2 step 2): the tests fail on behaviour.
MeshSample mesh_sample(const TriangleMesh&, glm::vec3) noexcept {
    return {};
}

float mesh_eval(const TriangleMesh&, glm::vec3) noexcept {
    return kSdfEmptyDistance;
}

}  // namespace spade
