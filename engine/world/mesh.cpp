// world/mesh.cpp -- see mesh.hpp for the contract, the refusal order and the
// content hash's spelling.
//
// Every check is a function that returns its refusal's detail, or nothing,
// and make_mesh() runs them in mesh.hpp's order. Each runs in index order;
// every sort is under a total order (ties broken by index), so a refusal's
// count and its first elements are the same on every platform. Geometry runs
// in double, on the stored fp32 positions promoted exactly, with only
// + - * / and sqrt (TD-3). The query (TriangleMesh::query, at the end) is
// fp32, every expression spelt in its evaluation order, and its kernel is
// core/math_ops_f64's closest_on_triangle.
#include "world/mesh.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <utility>

#include "core/math_ops_f64.hpp"
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

// --- Task B2: the pseudonormals, the BVH, the query, and the geometric checks

using detail::MeshBvhNode;
using math::TriangleFeature;

// A BVH leaf holds at most this many items.
constexpr std::size_t kLeafSize = 4;

// The walks' fixed stacks. A median split halves its items at every level, so
// a tree of at most 2^31 items is at most 30 levels deep, and a walk holds at
// most one pending sibling per level, plus the node in hand.
constexpr std::size_t kWalkStack = 64;

[[nodiscard]] glm::vec3 min3(glm::vec3 a, glm::vec3 b) noexcept {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

[[nodiscard]] glm::vec3 max3(glm::vec3 a, glm::vec3 b) noexcept {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

[[nodiscard]] bool boxes_touch(glm::vec3 alo, glm::vec3 ahi, glm::vec3 blo, glm::vec3 bhi) noexcept {
    return alo.x <= bhi.x && blo.x <= ahi.x && alo.y <= bhi.y && blo.y <= ahi.y && alo.z <= bhi.z &&
           blo.z <= ahi.z;
}

// v / |v| in double, rounded to fp32 once; zero for a zero v.
[[nodiscard]] glm::vec3 unit_f32(const glm::dvec3& v) noexcept {
    const double len = std::sqrt(dot_d(v, v));
    if (!(len > 0.0)) return glm::vec3(0.0f);
    return {static_cast<float>(v.x / len), static_cast<float>(v.y / len), static_cast<float>(v.z / len)};
}

// The slot of the edge {u, w} in triangle t: slot k runs from corner k to
// corner k + 1. {u, w} is one of t's edges.
[[nodiscard]] std::size_t edge_slot(const std::vector<uint32_t>& idx, uint32_t t, uint32_t u, uint32_t w) noexcept {
    const std::size_t base = 3 * static_cast<std::size_t>(t);
    for (std::size_t k = 0; k < 2; ++k) {
        const uint32_t a = idx[base + k];
        const uint32_t b = idx[base + k + 1];
        if ((a == u && b == w) || (a == w && b == u)) return k;
    }
    return 2;
}

struct Pseudonormals {
    std::vector<glm::vec3> face;    // per triangle
    std::vector<glm::vec3> edge;    // 3 per triangle, by slot
    std::vector<glm::vec3> vertex;  // per vertex
};

// PRECONDITION: check_degenerate() and check_edges() passed. In double, in
// index order (an edge's two normals in triangle order, a vertex's corners in
// corner order 3t + k), each rounded to fp32 once.
[[nodiscard]] Pseudonormals derive_pseudonormals(const std::vector<glm::vec3>& positions,
                                                 const std::vector<uint32_t>& idx,
                                                 const std::vector<EdgeUse>& uses) {
    const std::size_t triangle_count = idx.size() / 3;
    Pseudonormals out;

    std::vector<glm::dvec3> face(triangle_count);
    out.face.reserve(triangle_count);
    for (std::size_t t = 0; t < triangle_count; ++t) {
        const glm::dvec3 a = widen(positions[idx[3 * t]]);
        const glm::dvec3 n = cross_d(widen(positions[idx[3 * t + 1]]) - a, widen(positions[idx[3 * t + 2]]) - a);
        const double len = std::sqrt(dot_d(n, n));  // > 0: no triangle is degenerate
        face[t] = {n.x / len, n.y / len, n.z / len};
        out.face.emplace_back(static_cast<float>(face[t].x), static_cast<float>(face[t].y),
                              static_cast<float>(face[t].z));
    }

    // An edge: its two unit face normals summed. Every edge has exactly two
    // uses, adjacent in `uses`, the lower triangle first.
    out.edge.assign(3 * triangle_count, glm::vec3(0.0f));
    for (std::size_t i = 0; i < uses.size(); i += 2) {
        const EdgeUse& one = uses[i];
        const EdgeUse& two = uses[i + 1];
        const glm::dvec3& n1 = face[one.tri];
        const glm::dvec3& n2 = face[two.tri];
        const glm::vec3 n = unit_f32({n1.x + n2.x, n1.y + n2.y, n1.z + n2.z});
        out.edge[3 * static_cast<std::size_t>(one.tri) + edge_slot(idx, one.tri, one.lo, one.hi)] = n;
        out.edge[3 * static_cast<std::size_t>(two.tri) + edge_slot(idx, two.tri, one.lo, one.hi)] = n;
    }

    // A vertex: its corners' unit face normals, each weighted by the corner's
    // angle (math::corner_angle: Spade's own atan2, no libm).
    std::vector<glm::dvec3> sum(positions.size(), glm::dvec3(0.0));
    for (std::size_t t = 0; t < triangle_count; ++t) {
        for (std::size_t k = 0; k < 3; ++k) {
            const uint32_t v = idx[3 * t + k];
            const glm::dvec3 at = widen(positions[v]);
            const glm::dvec3 next = widen(positions[idx[3 * t + (k + 1) % 3]]) - at;
            const glm::dvec3 prev = widen(positions[idx[3 * t + (k + 2) % 3]]) - at;
            const double angle = math::corner_angle(next, prev);
            glm::dvec3& s = sum[v];
            s = {s.x + angle * face[t].x, s.y + angle * face[t].y, s.z + angle * face[t].z};
        }
    }
    out.vertex.reserve(positions.size());
    for (const glm::dvec3& s : sum) out.vertex.push_back(unit_f32(s));
    return out;
}

// One item of a BVH under construction: a triangle or a shell.
struct BuildItem {
    glm::vec3 lo{0.0f};
    glm::vec3 hi{0.0f};
    glm::dvec3 centroid{0.0};
    uint32_t id = 0;
};

// Builds items[first, last) into nodes[slot], top down: the items' box; a
// leaf at kLeafSize items or fewer; otherwise the items sorted by (centroid on
// the longest axis of the centroids' bounds, ties to the lower axis; id) and
// split at the median, count / 2 to the left. Children are appended in pairs.
void build_node(std::vector<MeshBvhNode>& nodes, std::size_t slot, std::vector<BuildItem>& items, std::size_t first,
                std::size_t last) {
    MeshBvhNode node;
    node.lo = items[first].lo;
    node.hi = items[first].hi;
    glm::dvec3 clo = items[first].centroid;
    glm::dvec3 chi = clo;
    for (std::size_t i = first + 1; i < last; ++i) {
        node.lo = min3(node.lo, items[i].lo);
        node.hi = max3(node.hi, items[i].hi);
        const glm::dvec3& c = items[i].centroid;
        clo = {std::min(clo.x, c.x), std::min(clo.y, c.y), std::min(clo.z, c.z)};
        chi = {std::max(chi.x, c.x), std::max(chi.y, c.y), std::max(chi.z, c.z)};
    }
    const std::size_t count = last - first;
    if (count <= kLeafSize) {
        node.first = static_cast<uint32_t>(first);
        node.count = static_cast<uint32_t>(count);
        nodes[slot] = node;
        return;
    }
    const glm::dvec3 extent{chi.x - clo.x, chi.y - clo.y, chi.z - clo.z};
    const int axis = (extent.x >= extent.y && extent.x >= extent.z) ? 0 : (extent.y >= extent.z ? 1 : 2);
    std::sort(items.begin() + static_cast<std::ptrdiff_t>(first), items.begin() + static_cast<std::ptrdiff_t>(last),
              [axis](const BuildItem& a, const BuildItem& b) {
                  if (a.centroid[axis] != b.centroid[axis]) return a.centroid[axis] < b.centroid[axis];
                  return a.id < b.id;
              });
    const std::size_t mid = first + count / 2;
    const std::size_t left = nodes.size();
    nodes.resize(left + 2);
    node.first = static_cast<uint32_t>(left);
    node.count = 0;
    nodes[slot] = node;
    build_node(nodes, left, items, first, mid);
    build_node(nodes, left + 1, items, mid, last);
}

struct Bvh {
    std::vector<MeshBvhNode> shell_nodes;
    std::vector<uint32_t> shell_items;
    std::vector<MeshBvhNode> triangle_nodes;
    std::vector<uint32_t> triangle_items;
    std::vector<uint32_t> shell_root;
};

// The two-level BVH: one tree per shell over its triangles (gathered in index
// order), then one tree over the shells' boxes. A triangle's centroid is
// (a + b + c) / 3 in double, a shell's the centre of its box.
[[nodiscard]] Bvh build_bvh(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& idx,
                            const std::vector<uint32_t>& triangle_shell, uint32_t shell_count) {
    const std::size_t triangle_count = idx.size() / 3;
    std::vector<std::size_t> begin(static_cast<std::size_t>(shell_count) + 1, 0);
    for (const uint32_t s : triangle_shell) ++begin[static_cast<std::size_t>(s) + 1];
    for (std::size_t s = 0; s < shell_count; ++s) begin[s + 1] += begin[s];

    std::vector<BuildItem> items(triangle_count);
    std::vector<std::size_t> fill(begin.begin(), begin.end() - 1);
    for (std::size_t t = 0; t < triangle_count; ++t) {
        const glm::vec3 a = positions[idx[3 * t]];
        const glm::vec3 b = positions[idx[3 * t + 1]];
        const glm::vec3 c = positions[idx[3 * t + 2]];
        BuildItem& item = items[fill[triangle_shell[t]]++];
        item.lo = min3(min3(a, b), c);
        item.hi = max3(max3(a, b), c);
        const glm::dvec3 wa = widen(a);
        const glm::dvec3 wb = widen(b);
        const glm::dvec3 wc = widen(c);
        item.centroid = {(wa.x + wb.x + wc.x) / 3.0, (wa.y + wb.y + wc.y) / 3.0, (wa.z + wb.z + wc.z) / 3.0};
        item.id = static_cast<uint32_t>(t);
    }

    Bvh out;
    out.shell_root.resize(shell_count);
    for (std::size_t s = 0; s < shell_count; ++s) {
        const std::size_t root = out.triangle_nodes.size();
        out.triangle_nodes.emplace_back();
        build_node(out.triangle_nodes, root, items, begin[s], begin[s + 1]);
        out.shell_root[s] = static_cast<uint32_t>(root);
    }
    out.triangle_items.reserve(triangle_count);
    for (const BuildItem& item : items) out.triangle_items.push_back(item.id);

    std::vector<BuildItem> shells(shell_count);
    for (std::size_t s = 0; s < shell_count; ++s) {
        const MeshBvhNode& root = out.triangle_nodes[out.shell_root[s]];
        const glm::dvec3 lo = widen(root.lo);
        const glm::dvec3 hi = widen(root.hi);
        shells[s].lo = root.lo;
        shells[s].hi = root.hi;
        shells[s].centroid = {(lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5, (lo.z + hi.z) * 0.5};
        shells[s].id = static_cast<uint32_t>(s);
    }
    out.shell_nodes.emplace_back();
    build_node(out.shell_nodes, 0, shells, 0, shell_count);
    out.shell_items.reserve(shell_count);
    for (const BuildItem& item : shells) out.shell_items.push_back(item.id);
    return out;
}

// The squared distance from p to a node's box, fp32, spelt as a triangle's
// squared distance is (closest_on_triangle: x, then y, then z). Every closest
// point a triangle returns lies in its box, so this is never more than any of
// the node's triangles' squared distances, rounding included: each
// coordinate's gap is no larger, and fp32 rounding is monotone.
[[nodiscard]] float box_distance2(glm::vec3 p, const MeshBvhNode& n) noexcept {
    const float dx = p.x < n.lo.x ? n.lo.x - p.x : (p.x > n.hi.x ? p.x - n.hi.x : 0.0f);
    const float dy = p.y < n.lo.y ? n.lo.y - p.y : (p.y > n.hi.y ? p.y - n.hi.y : 0.0f);
    const float dz = p.z < n.lo.z ? n.lo.z - p.z : (p.z > n.hi.z ? p.z - n.hi.z : 0.0f);
    return dx * dx + dy * dy + dz * dz;
}

// Depth first, the nearer child first (a tie: the left), pruning a node only
// when its box is FARTHER than `best` -- so nothing that could win or tie is
// pruned, and the answer is the brute-force answer whatever the order.
template <class Leaf>
void nearest_walk(const std::vector<MeshBvhNode>& nodes, uint32_t root, glm::vec3 p, const float& best,
                  Leaf&& leaf) noexcept {
    struct Entry {
        uint32_t node;
        float bound;
    };
    std::array<Entry, kWalkStack> stack{};
    std::size_t top = 0;
    stack[top++] = {root, box_distance2(p, nodes[root])};
    while (top > 0) {
        const Entry e = stack[--top];
        if (e.bound > best) continue;
        const MeshBvhNode& n = nodes[e.node];
        if (n.count > 0) {
            leaf(n.first, n.count);
            continue;
        }
        const float left = box_distance2(p, nodes[n.first]);
        const float right = box_distance2(p, nodes[n.first + 1]);
        if (right < left) {
            stack[top++] = {n.first, left};
            stack[top++] = {n.first + 1, right};
        } else {
            stack[top++] = {n.first + 1, right};
            stack[top++] = {n.first, left};
        }
    }
}

// Every leaf whose box touches [lo, hi].
template <class Leaf>
void overlap_walk(const std::vector<MeshBvhNode>& nodes, uint32_t root, glm::vec3 lo, glm::vec3 hi, Leaf&& leaf) {
    std::array<uint32_t, kWalkStack> stack{};
    std::size_t top = 0;
    stack[top++] = root;
    while (top > 0) {
        const MeshBvhNode& n = nodes[stack[--top]];
        if (!boxes_touch(n.lo, n.hi, lo, hi)) continue;
        if (n.count > 0) {
            leaf(n.first, n.count);
            continue;
        }
        stack[top++] = n.first + 1;
        stack[top++] = n.first;
    }
}

// Whether triangles s and t meet anywhere but what they share (mesh.hpp's
// refusal list): no shared vertex, Moller's test; one, each far edge against
// the other triangle; an edge, a flat fold only.
[[nodiscard]] bool triangles_cross(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& idx,
                                   uint32_t s, uint32_t t) {
    const std::size_t sb = 3 * static_cast<std::size_t>(s);
    const std::size_t tb = 3 * static_cast<std::size_t>(t);
    glm::dvec3 a[3];
    glm::dvec3 b[3];
    int match[3] = {-1, -1, -1};  // match[k]: the corner of t at s's corner k, or -1
    int shared = 0;
    for (std::size_t k = 0; k < 3; ++k) {
        a[k] = widen(positions[idx[sb + k]]);
        b[k] = widen(positions[idx[tb + k]]);
        for (std::size_t j = 0; j < 3; ++j) {
            if (idx[sb + k] == idx[tb + j]) {
                match[k] = static_cast<int>(j);
                ++shared;
            }
        }
    }
    if (shared == 0) return math::triangles_meet(a, b);
    if (shared == 1) {
        const int ks = match[0] >= 0 ? 0 : (match[1] >= 0 ? 1 : 2);
        const int kt = match[ks];
        return math::segment_meets_triangle(a[(ks + 1) % 3], a[(ks + 2) % 3], b[0], b[1], b[2]) ||
               math::segment_meets_triangle(b[(kt + 1) % 3], b[(kt + 2) % 3], a[0], a[1], a[2]);
    }
    // A shared edge: a fold, if t's far corner lies in s's plane (a zero
    // determinant) and the two normals are opposed.
    int far_t = 0;
    while (far_t == match[0] || far_t == match[1] || far_t == match[2]) ++far_t;
    if (math::orient3d(a[0], a[1], a[2], b[far_t]) != 0.0) return false;
    const glm::dvec3 ns = cross_d(a[1] - a[0], a[2] - a[0]);
    const glm::dvec3 nt = cross_d(b[1] - b[0], b[2] - b[0]);
    return dot_d(ns, nt) < 0.0;
}

// Every pair of triangles whose boxes touch, from the BVH, in (lower, higher)
// index order.
[[nodiscard]] Detail check_self_intersection(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& idx,
                                             const Bvh& bvh) {
    Offenders bad;
    std::vector<uint32_t> near;
    const auto box_of = [&](uint32_t t, glm::vec3& lo, glm::vec3& hi) {
        const glm::vec3 a = positions[idx[3 * static_cast<std::size_t>(t)]];
        const glm::vec3 b = positions[idx[3 * static_cast<std::size_t>(t) + 1]];
        const glm::vec3 c = positions[idx[3 * static_cast<std::size_t>(t) + 2]];
        lo = min3(min3(a, b), c);
        hi = max3(max3(a, b), c);
    };
    const uint32_t triangle_count = static_cast<uint32_t>(idx.size() / 3);
    for (uint32_t s = 0; s < triangle_count; ++s) {
        glm::vec3 lo{0.0f};
        glm::vec3 hi{0.0f};
        box_of(s, lo, hi);
        near.clear();
        overlap_walk(bvh.shell_nodes, 0, lo, hi, [&](uint32_t first, uint32_t count) {
            for (uint32_t i = first; i < first + count; ++i) {
                const uint32_t shell = bvh.shell_items[i];
                overlap_walk(bvh.triangle_nodes, bvh.shell_root[shell], lo, hi, [&](uint32_t f, uint32_t n) {
                    for (uint32_t j = f; j < f + n; ++j) {
                        const uint32_t t = bvh.triangle_items[j];
                        if (t <= s) continue;
                        glm::vec3 tlo{0.0f};
                        glm::vec3 thi{0.0f};
                        box_of(t, tlo, thi);
                        if (boxes_touch(lo, hi, tlo, thi)) near.push_back(t);
                    }
                });
            }
        });
        std::sort(near.begin(), near.end());
        for (const uint32_t t : near) {
            if (triangles_cross(positions, idx, s, t)) bad.add([s, t] { return pair_text(s, t); });
        }
    }
    return bad.detail("self-intersecting triangle pair", "self-intersecting triangle pairs");
}

// PRECONDITION: no self-intersection, so each shell lies wholly inside or
// wholly outside each other one. `distance_in_shell(x, t)` is the signed
// distance from x to shell t alone.
template <class DistanceInShell>
[[nodiscard]] Detail check_nesting(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& idx,
                                   const std::vector<uint32_t>& triangle_shell, uint32_t shell_count,
                                   const std::vector<MeshBvhNode>& triangle_nodes,
                                   const std::vector<uint32_t>& shell_root, const glm::dvec3& lo,
                                   const glm::dvec3& hi, DistanceInShell&& distance_in_shell) {
    // Each shell's signed volume (six times it), in triangle order, about the
    // bounds' centre, and its lowest-indexed vertex.
    const glm::dvec3 centre = (lo + hi) * 0.5;
    std::vector<double> six_volume(shell_count, 0.0);
    std::vector<uint32_t> lowest(shell_count, std::numeric_limits<uint32_t>::max());
    for (std::size_t t = 0; t < idx.size() / 3; ++t) {
        const uint32_t s = triangle_shell[t];
        const glm::dvec3 a = widen(positions[idx[3 * t]]) - centre;
        const glm::dvec3 b = widen(positions[idx[3 * t + 1]]) - centre;
        const glm::dvec3 c = widen(positions[idx[3 * t + 2]]) - centre;
        six_volume[s] += dot_d(a, cross_d(b, c));
        for (std::size_t k = 0; k < 3; ++k) lowest[s] = std::min(lowest[s], idx[3 * t + k]);
    }

    Offenders bad;
    for (uint32_t s = 0; s < shell_count; ++s) {
        if (six_volume[s] == 0.0) {
            bad.add([s] { return std::to_string(s) + " (encloses no volume)"; });
            continue;
        }
        const bool outward = six_volume[s] > 0.0;
        const glm::vec3 x = positions[lowest[s]];
        int sum = 0;
        for (uint32_t t = 0; t < shell_count; ++t) {
            if (t == s || six_volume[t] == 0.0) continue;
            const MeshBvhNode& box = triangle_nodes[shell_root[t]];
            if (!boxes_touch(box.lo, box.hi, x, x)) continue;  // outside t's box: outside t
            const float d = distance_in_shell(x, t);
            if (six_volume[t] > 0.0 && d < 0.0f) {
                ++sum;  // inside an outward shell
            } else if (six_volume[t] < 0.0 && d > 0.0f) {
                --sum;  // inside an inward shell (its normals point into what it bounds)
            }
        }
        const int need = outward ? 0 : 1;
        if (sum != need) {
            bad.add([&] {
                return std::to_string(s) + (outward ? " (outward" : " (inward") + ", sum " + std::to_string(sum) +
                       ", needs " + std::to_string(need) + ")";
            });
        }
    }
    return bad.detail("wrongly nested shell (the winding number must be 0 or 1 everywhere)",
                      "wrongly nested shells (the winding number must be 0 or 1 everywhere)");
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

    uint32_t shell_count = 0;
    std::vector<uint32_t> triangle_shell = label_shells(uses, triangle_count, shell_count);
    if (shell_count > kMaxMeshShells) {
        return refuse(Code::capacity_exceeded, std::to_string(shell_count) + " shells, more than the " +
                                                   std::to_string(kMaxMeshShells) + " a mesh may hold");
    }

    // The geometric half (Task B2): derive, then refuse what breaks the sign.
    Pseudonormals normals = derive_pseudonormals(positions, idx, uses);
    Bvh bvh = build_bvh(positions, idx, triangle_shell, shell_count);
    if (Detail d = check_self_intersection(positions, idx, bvh)) return refuse(Code::invalid_argument, *d);

    std::shared_ptr<TriangleMesh> mesh(new TriangleMesh());
    mesh->triangle_shell_ = std::move(triangle_shell);
    mesh->shell_count_ = shell_count;
    mesh->content_hash_ = content_hash(positions, idx);
    mesh->positions_ = std::move(positions);
    mesh->indices_ = idx;
    mesh->face_normal_ = std::move(normals.face);
    mesh->edge_normal_ = std::move(normals.edge);
    mesh->vertex_normal_ = std::move(normals.vertex);
    mesh->shell_nodes_ = std::move(bvh.shell_nodes);
    mesh->shell_items_ = std::move(bvh.shell_items);
    mesh->triangle_nodes_ = std::move(bvh.triangle_nodes);
    mesh->triangle_items_ = std::move(bvh.triangle_items);
    mesh->shell_root_ = std::move(bvh.shell_root);

    const TriangleMesh& built = *mesh;
    const auto distance_in_shell = [&built](glm::vec3 x, uint32_t s) { return built.query(x, s).distance; };
    if (Detail d = check_nesting(built.positions_, built.indices_, built.triangle_shell_, shell_count,
                                 built.triangle_nodes_, built.shell_root_, lo, hi, distance_in_shell)) {
        return refuse(Code::invalid_argument, *d);
    }
    return std::shared_ptr<const TriangleMesh>(std::move(mesh));
}

MeshSample TriangleMesh::query(glm::vec3 p, uint32_t shell) const noexcept {
    if (!finite(p)) return {};
    constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();
    float best = std::numeric_limits<float>::infinity();
    uint32_t best_triangle = kNone;
    math::TriangleClosest best_hit;

    // The minimum (squared distance, triangle index), whatever the order.
    const auto triangles = [&](uint32_t first, uint32_t count) {
        for (uint32_t i = first; i < first + count; ++i) {
            const uint32_t t = triangle_items_[i];
            const uint32_t a = indices_[3 * static_cast<std::size_t>(t)];
            const uint32_t b = indices_[3 * static_cast<std::size_t>(t) + 1];
            const uint32_t c = indices_[3 * static_cast<std::size_t>(t) + 2];
            const math::TriangleClosest hit =
                math::closest_on_triangle(p, positions_[a], positions_[b], positions_[c], a, b, c);
            if (hit.distance2 < best || (hit.distance2 == best && t < best_triangle)) {
                best = hit.distance2;
                best_triangle = t;
                best_hit = hit;
            }
        }
    };
    const auto one_shell = [&](uint32_t s) { nearest_walk(triangle_nodes_, shell_root_[s], p, best, triangles); };
    if (shell != kEveryShell) {
        one_shell(shell);
    } else {
        nearest_walk(shell_nodes_, 0, p, best, [&](uint32_t first, uint32_t count) {
            for (uint32_t i = first; i < first + count; ++i) one_shell(shell_items_[i]);
        });
    }
    if (best_triangle == kNone) return {};

    // The closest feature's pseudonormal signs the distance.
    const std::size_t t = best_triangle;
    glm::vec3 n{0.0f};
    switch (best_hit.feature) {
        case TriangleFeature::face: n = face_normal_[t]; break;
        case TriangleFeature::edge_ab: n = edge_normal_[3 * t]; break;
        case TriangleFeature::edge_bc: n = edge_normal_[3 * t + 1]; break;
        case TriangleFeature::edge_ca: n = edge_normal_[3 * t + 2]; break;
        case TriangleFeature::vertex_a: n = vertex_normal_[indices_[3 * t]]; break;
        case TriangleFeature::vertex_b: n = vertex_normal_[indices_[3 * t + 1]]; break;
        case TriangleFeature::vertex_c: n = vertex_normal_[indices_[3 * t + 2]]; break;
    }
    const glm::vec3 diff{p.x - best_hit.point.x, p.y - best_hit.point.y, p.z - best_hit.point.z};
    const float side = diff.x * n.x + diff.y * n.y + diff.z * n.z;

    MeshSample out;
    out.triangle = best_triangle;
    if (best > 0.0f) {
        const float len = std::sqrt(best);
        const glm::vec3 g{diff.x / len, diff.y / len, diff.z / len};
        const bool inside = side < 0.0f;
        out.distance = inside ? -len : len;
        out.gradient = inside ? -g : g;
    } else {
        out.distance = 0.0f;
        out.gradient = n;
    }
    return out;
}

MeshSample mesh_sample(const TriangleMesh& mesh, glm::vec3 p) noexcept {
    return mesh.query(p, TriangleMesh::kEveryShell);
}

float mesh_eval(const TriangleMesh& mesh, glm::vec3 p) noexcept {
    return mesh_sample(mesh, p).distance;
}

}  // namespace spade
