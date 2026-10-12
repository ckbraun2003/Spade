#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "core/math_ops_f64.hpp"
#include "core/rng.hpp"
#include "world/mesh.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// world/mesh.hpp's field: exact distance through the BVH, the sign from the
// closest feature's angle-weighted pseudonormal, and the refusals that keep
// the sign's precondition true (the parts and meshes plan, Task B2; CORE-6).
// core/math_ops_f64.hpp's kernels: Spade's own atan2 (TD-3) and the
// per-triangle closest point.
//
// Every expectation is the fixture's own geometry, worked by hand or by an
// independent computation in this file (TD-4): plane tests for convex
// solids, coordinates for boxes, std::atan2 as the atan2 oracle (tests may
// call libm; the engine may not). The self-intersection pairs were listed by
// an exact rational separating-axis check, outside the engine.
// ---------------------------------------------------------------------------

namespace {

using spade::Code;
using spade::MeshInput;
using spade::MeshSample;
using spade::MeshUnit;
using spade::TriangleMesh;
using spade::math::TriangleClosest;
using spade::math::TriangleFeature;

using MeshPtr = std::shared_ptr<const TriangleMesh>;

// The cube's twelve triangles, counter-clockwise seen from outside, its
// vertices numbered x + 2y + 4z (as tests/test_mesh.cpp numbers them).
constexpr uint32_t kCubeTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3},  // z = lo, normal -z
    {4, 5, 6}, {5, 7, 6},  // z = hi, normal +z
    {0, 1, 4}, {1, 5, 4},  // y = lo, normal -y
    {2, 6, 3}, {3, 6, 7},  // y = hi, normal +y
    {0, 4, 2}, {2, 4, 6},  // x = lo, normal -x
    {1, 3, 5}, {3, 7, 5},  // x = hi, normal +x
};

MeshInput cube(glm::dvec3 lo, double side) {
    MeshInput m;
    m.unit = MeshUnit::metre;
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::dvec3 corner{static_cast<double>(v & 1u), static_cast<double>((v >> 1) & 1u),
                                static_cast<double>((v >> 2) & 1u)};
        m.positions.push_back(lo + side * corner);
    }
    for (const auto& t : kCubeTriangles) m.indices.insert(m.indices.end(), {t[0], t[1], t[2]});
    return m;
}

MeshInput unit_cube() {
    return cube({0.0, 0.0, 0.0}, 1.0);
}

// Every triangle reversed: the normals point in, so as a shell of a larger
// mesh it bounds a cavity.
MeshInput flipped(MeshInput m) {
    for (std::size_t t = 0; t < m.indices.size(); t += 3) std::swap(m.indices[t + 1], m.indices[t + 2]);
    return m;
}

// `a` then `b` as one input, welding b's exact duplicates of a's positions.
MeshInput merged(const MeshInput& a, const MeshInput& b) {
    MeshInput m = a;
    std::vector<uint32_t> remap;
    for (const glm::dvec3& p : b.positions) {
        const auto it = std::find(m.positions.begin(), m.positions.end(), p);
        if (it == m.positions.end()) {
            remap.push_back(static_cast<uint32_t>(m.positions.size()));
            m.positions.push_back(p);
        } else {
            remap.push_back(static_cast<uint32_t>(it - m.positions.begin()));
        }
    }
    for (const uint32_t i : b.indices) m.indices.push_back(remap[i]);
    return m;
}

// A box [0, length] x [0, thickness] x [0, width] cut into `segments` slabs
// along x, so its long faces are strips of quads. Thin and narrow, its
// thickness faces are slivers: a blade-like plate.
MeshInput plate(double length, double thickness, double width, uint32_t segments) {
    MeshInput m;
    m.unit = MeshUnit::metre;
    for (uint32_t i = 0; i <= segments; ++i) {
        const double x = length * static_cast<double>(i) / static_cast<double>(segments);
        m.positions.push_back({x, 0.0, 0.0});              // 4i:     y = 0, z = 0
        m.positions.push_back({x, thickness, 0.0});        // 4i + 1: y = t, z = 0
        m.positions.push_back({x, 0.0, width});            // 4i + 2: y = 0, z = w
        m.positions.push_back({x, thickness, width});      // 4i + 3: y = t, z = w
    }
    const auto quad = [&m](uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
        m.indices.insert(m.indices.end(), {a, b, c, a, c, d});
    };
    for (uint32_t i = 0; i < segments; ++i) {
        const uint32_t s = 4 * i;
        quad(s, s + 4, s + 6, s + 2);      // y = 0, normal -y
        quad(s + 1, s + 3, s + 7, s + 5);  // y = t, normal +y
        quad(s, s + 1, s + 5, s + 4);      // z = 0, normal -z
        quad(s + 2, s + 6, s + 7, s + 3);  // z = w, normal +z
    }
    const uint32_t e = 4 * segments;
    quad(0, 2, 3, 1);          // x = 0, normal -x
    quad(e, e + 1, e + 3, e + 2);  // x = length, normal +x
    return m;
}

MeshInput tetrahedron() {
    MeshInput m;
    m.unit = MeshUnit::metre;
    m.positions = {{0.0, 0.0, 0.0}, {1.3, 0.0, 0.0}, {0.2, 0.9, 0.0}, {0.3, 0.25, 1.1}};
    m.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    return m;
}

// The icosahedron subdivided `levels` times onto the unit sphere: 20 * 4^levels
// triangles. Midpoints are shared through an ordered map (no hashing).
MeshInput icosphere(int levels) {
    MeshInput m;
    m.unit = MeshUnit::metre;
    const double g = (1.0 + std::sqrt(5.0)) / 2.0;
    const glm::dvec3 base[12] = {{-1, g, 0}, {1, g, 0}, {-1, -g, 0}, {1, -g, 0}, {0, -1, g}, {0, 1, g},
                                 {0, -1, -g}, {0, 1, -g}, {g, 0, -1}, {g, 0, 1}, {-g, 0, -1}, {-g, 0, 1}};
    for (const glm::dvec3& p : base) m.positions.push_back(glm::normalize(p));
    std::vector<uint32_t> tris = {0, 11, 5, 0, 5, 1,  0, 1, 7,  0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4,
                                  11, 10, 2, 10, 7, 6, 7, 1, 8, 3, 9, 4,  3, 4, 2,   3, 2, 6, 3, 6, 8,
                                  3, 8, 9,  4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7,   9, 8, 1};
    for (int level = 0; level < levels; ++level) {
        std::map<std::pair<uint32_t, uint32_t>, uint32_t> mid;
        const auto midpoint = [&](uint32_t a, uint32_t b) {
            const std::pair<uint32_t, uint32_t> key{std::min(a, b), std::max(a, b)};
            const auto it = mid.find(key);
            if (it != mid.end()) return it->second;
            const uint32_t v = static_cast<uint32_t>(m.positions.size());
            m.positions.push_back(glm::normalize(m.positions[a] + m.positions[b]));
            mid.emplace(key, v);
            return v;
        };
        std::vector<uint32_t> next;
        for (std::size_t t = 0; t < tris.size(); t += 3) {
            const uint32_t a = tris[t];
            const uint32_t b = tris[t + 1];
            const uint32_t c = tris[t + 2];
            const uint32_t ab = midpoint(a, b);
            const uint32_t bc = midpoint(b, c);
            const uint32_t ca = midpoint(c, a);
            next.insert(next.end(), {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
        }
        tris = std::move(next);
    }
    m.indices = std::move(tris);
    return m;
}

// The L prism: the L (0,0) (2,0) (2,1) (1,1) (1,2) (0,2) extruded over z in
// [0, 1]. Vertex i is the L's corner i at z = 0, vertex 6 + i at z = 1; the
// reflex corner is vertex 3 (bottom) and vertex 9 (top), at (1, 1, z).
MeshInput l_prism() {
    MeshInput m;
    m.unit = MeshUnit::metre;
    const double xy[6][2] = {{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}};
    for (double z : {0.0, 1.0}) {
        for (const auto& c : xy) m.positions.push_back({c[0], c[1], z});
    }
    // The caps: a fan from the reflex corner (counter-clockwise seen from +z).
    const uint32_t fan[4][3] = {{3, 4, 5}, {3, 5, 0}, {3, 0, 1}, {3, 1, 2}};
    for (const auto& t : fan) m.indices.insert(m.indices.end(), {t[0] + 6, t[1] + 6, t[2] + 6});  // top, +z
    for (const auto& t : fan) m.indices.insert(m.indices.end(), {t[0], t[2], t[1]});              // bottom, -z
    for (uint32_t i = 0; i < 6; ++i) {  // the walls, outward on the L's right
        const uint32_t j = (i + 1) % 6;
        m.indices.insert(m.indices.end(), {i, j, j + 6, i, j + 6, i + 6});
    }
    return m;
}

MeshPtr build(const MeshInput& in, const char* name) {
    auto m = spade::make_mesh(in, name);
    EXPECT_TRUE(m.has_value()) << name << ": " << (m.has_value() ? std::string() : m.error().context);
    return m.has_value() ? *m : nullptr;
}

// Uniform double in [0, 1) from a seeded stream (53 bits).
double unit_double(spade::rng::Stream& s) {
    return static_cast<double>(s.next_u64() >> 11) * 0x1.0p-53;
}

glm::dvec3 widen(const glm::vec3& v) {
    return {static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)};
}

// The brute-force answer: every triangle, in index order, through the same
// per-triangle kernel, keeping the minimum (squared distance, triangle index).
struct Brute {
    float distance2 = std::numeric_limits<float>::infinity();
    glm::vec3 point{0.0f};
    uint32_t triangle = 0;
};

Brute brute_force(const TriangleMesh& mesh, glm::vec3 p) {
    Brute best;
    const auto pos = mesh.positions();
    const auto idx = mesh.indices();
    for (uint32_t t = 0; t < mesh.triangle_count(); ++t) {
        const uint32_t a = idx[3 * t];
        const uint32_t b = idx[3 * t + 1];
        const uint32_t c = idx[3 * t + 2];
        const TriangleClosest k = spade::math::closest_on_triangle(p, pos[a], pos[b], pos[c], a, b, c);
        if (k.distance2 < best.distance2) {  // index order: a tie keeps the lower index
            best = {k.distance2, k.point, t};
        }
    }
    return best;
}

// Inside (-1), outside (+1) or too near the surface to say (0), for a convex
// mesh: inside iff on the inner side of every face plane, in double.
int convex_side(const TriangleMesh& mesh, glm::vec3 pf, double margin) {
    const glm::dvec3 p = widen(pf);
    const auto pos = mesh.positions();
    const auto idx = mesh.indices();
    double worst = -std::numeric_limits<double>::infinity();
    for (uint32_t t = 0; t < mesh.triangle_count(); ++t) {
        const glm::dvec3 a = widen(pos[idx[3 * t]]);
        const glm::dvec3 n = glm::normalize(glm::cross(widen(pos[idx[3 * t + 1]]) - a, widen(pos[idx[3 * t + 2]]) - a));
        worst = std::max(worst, glm::dot(p - a, n));
    }
    if (std::abs(worst) < margin) return 0;
    return worst < 0.0 ? -1 : 1;
}

// The same for an axis-aligned box [lo, hi].
int box_side(glm::dvec3 lo, glm::dvec3 hi, glm::vec3 pf, double margin) {
    const glm::dvec3 p = widen(pf);
    double worst = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) worst = std::max({worst, lo[i] - p[i], p[i] - hi[i]});
    if (std::abs(worst) < margin) return 0;
    return worst < 0.0 ? -1 : 1;
}

}  // namespace

TEST(MeshField, ACubesDistanceIsExactInEveryRegion) {
    const MeshPtr cube = build(unit_cube(), "cube");  // [0, 1]^3
    ASSERT_NE(cube, nullptr);
    EXPECT_EQ(spade::mesh_eval(*cube, {2.0f, 0.5f, 0.5f}), 1.0f) << "face region";
    EXPECT_NEAR(spade::mesh_eval(*cube, {2.0f, 2.0f, 0.5f}), std::sqrt(2.0f), 1e-6f) << "edge region";
    EXPECT_NEAR(spade::mesh_eval(*cube, {2.0f, 2.0f, 2.0f}), std::sqrt(3.0f), 1e-6f) << "vertex region";
    EXPECT_EQ(spade::mesh_eval(*cube, {0.5f, 0.5f, 0.25f}), -0.25f) << "inside, nearest the z = 0 face";
    EXPECT_EQ(spade::mesh_eval(*cube, {-0.5f, 0.25f, 0.75f}), 0.5f) << "face region, x = 0";
    EXPECT_EQ(spade::mesh_eval(*cube, {0.75f, 0.5f, 0.5f}), -0.25f) << "inside, nearest the x = 1 face";
}

TEST(MeshField, EachFeatureSignsWithItsOwnPseudonormal) {
    // 1. The per-triangle kernel: Ericson's seven regions, by hand. The
    // triangle (0,0,0) (4,0,0) (0,4,0) with vertex indices 10, 11, 12.
    {
        const glm::vec3 a{0.0f, 0.0f, 0.0f};
        const glm::vec3 b{4.0f, 0.0f, 0.0f};
        const glm::vec3 c{0.0f, 4.0f, 0.0f};
        struct Case {
            const char* why;
            glm::vec3 p;
            glm::vec3 point;
            float distance2;
            TriangleFeature feature;
        };
        const Case cases[] = {
            {"face", {1.0f, 1.0f, 3.0f}, {1.0f, 1.0f, 0.0f}, 9.0f, TriangleFeature::face},
            {"vertex a", {-1.0f, -2.0f, 1.0f}, a, 6.0f, TriangleFeature::vertex_a},
            {"vertex b", {5.0f, -1.0f, 0.0f}, b, 2.0f, TriangleFeature::vertex_b},
            {"vertex c", {-1.0f, 5.0f, 2.0f}, c, 6.0f, TriangleFeature::vertex_c},
            {"edge ab", {2.0f, -3.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, 9.0f, TriangleFeature::edge_ab},
            {"edge bc", {3.0f, 3.0f, 1.0f}, {2.0f, 2.0f, 0.0f}, 3.0f, TriangleFeature::edge_bc},
            {"edge ca", {-2.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 4.0f, TriangleFeature::edge_ca},
        };
        for (const Case& k : cases) {
            const TriangleClosest got = spade::math::closest_on_triangle(k.p, a, b, c, 10, 11, 12);
            EXPECT_EQ(got.point, k.point) << k.why;
            EXPECT_EQ(got.distance2, k.distance2) << k.why;
            EXPECT_EQ(got.feature, k.feature) << k.why;
        }
    }

    // 2. A shared edge gives the same bits from both of its triangles: each
    // projects onto the edge from its lower-indexed end. (a, b, c) and
    // (b, a, d) share the edge a-b (indices 20 and 7); p sits off the edge,
    // away from both triangles.
    {
        const glm::vec3 a{0.1f, 0.2f, 0.3f};
        const glm::vec3 b{1.7f, 0.45f, -0.2f};
        const glm::vec3 c{0.4f, 1.3f, 0.5f};
        const glm::vec3 d{0.9f, -0.8f, 0.6f};
        const glm::dvec3 ad = widen(a);
        const glm::dvec3 e = glm::normalize(widen(b) - ad);
        const auto across = [&](glm::vec3 q) {  // q - a with its component along the edge removed
            const glm::dvec3 r = widen(q) - ad;
            return glm::normalize(r - glm::dot(r, e) * e);
        };
        const glm::dvec3 on_edge = ad + 0.37 * (widen(b) - ad);
        const glm::dvec3 off = -(across(c) + across(d));
        const glm::vec3 p(on_edge + 0.25 * glm::normalize(off));
        const TriangleClosest one = spade::math::closest_on_triangle(p, a, b, c, 20, 7, 30);
        const TriangleClosest two = spade::math::closest_on_triangle(p, b, a, d, 7, 20, 31);
        EXPECT_EQ(one.feature, TriangleFeature::edge_ab);
        EXPECT_EQ(two.feature, TriangleFeature::edge_ab);
        EXPECT_EQ(std::bit_cast<uint32_t>(one.point.x), std::bit_cast<uint32_t>(two.point.x));
        EXPECT_EQ(std::bit_cast<uint32_t>(one.point.y), std::bit_cast<uint32_t>(two.point.y));
        EXPECT_EQ(std::bit_cast<uint32_t>(one.point.z), std::bit_cast<uint32_t>(two.point.z));
        EXPECT_EQ(std::bit_cast<uint32_t>(one.distance2), std::bit_cast<uint32_t>(two.distance2));
        EXPECT_NEAR(std::sqrt(one.distance2), 0.25f, 1e-6f);
    }

    // 3. Through the mesh: the closest feature's pseudonormal signs the
    // distance, and is the gradient on the surface itself.
    const MeshPtr cube = build(unit_cube(), "cube");
    ASSERT_NE(cube, nullptr);
    const float r2 = static_cast<float>(1.0 / std::sqrt(2.0));
    const float r3 = static_cast<float>(1.0 / std::sqrt(3.0));
    struct Probe {
        const char* why;
        glm::vec3 p;
        float distance;
        glm::vec3 gradient;
        uint32_t triangle;
    };
    const Probe probes[] = {
        // Faces: the face normal.
        {"above the z = 1 face", {0.25f, 0.5f, 1.5f}, 0.5f, {0.0f, 0.0f, 1.0f}, 2},
        {"on the z = 1 face", {0.25f, 0.5f, 1.0f}, 0.0f, {0.0f, 0.0f, 1.0f}, 2},
        {"inside, under the z = 1 face", {0.5f, 0.25f, 0.875f}, -0.125f, {0.0f, 0.0f, 1.0f}, 2},
        // An edge between two faces: the two face normals summed. Triangles 3
        // (5, 7, 6) and 11 (3, 7, 5) share the edge 5-7; the tie goes to 3.
        {"beyond the edge x = z = 1", {1.5f, 0.5f, 1.5f}, std::sqrt(0.5f), {r2, 0.0f, r2}, 3},
        {"on the edge x = z = 1", {1.0f, 0.5f, 1.0f}, 0.0f, {r2, 0.0f, r2}, 3},
        // An edge inside one face (the diagonal 5-6): its two normals agree.
        {"on the z = 1 face's diagonal", {0.5f, 0.5f, 1.0f}, 0.0f, {0.0f, 0.0f, 1.0f}, 2},
        // A vertex: its corner normals weighted by corner angle. Vertex 1 sits
        // in two -z triangles (45 degrees each), two -y triangles (45 each) and
        // one +x triangle (90), so the angle-weighted normal is (1, -1, -1)/sqrt(3);
        // counting triangles would give (1, -2, -2)/3. Triangle 0 is the lowest
        // holding it.
        {"on vertex 1", {1.0f, 0.0f, 0.0f}, 0.0f, {r3, -r3, -r3}, 0},
        {"beyond vertex 1", {1.5f, -0.5f, -0.5f}, std::sqrt(0.75f), {r3, -r3, -r3}, 0},
    };
    for (const Probe& k : probes) {
        const MeshSample s = spade::mesh_sample(*cube, k.p);
        EXPECT_EQ(s.distance, k.distance) << k.why;
        EXPECT_FLOAT_EQ(s.gradient.x, k.gradient.x) << k.why;
        EXPECT_FLOAT_EQ(s.gradient.y, k.gradient.y) << k.why;
        EXPECT_FLOAT_EQ(s.gradient.z, k.gradient.z) << k.why;
        EXPECT_EQ(s.triangle, k.triangle) << k.why;
        EXPECT_EQ(spade::mesh_eval(*cube, k.p), s.distance) << k.why;
    }
}

TEST(MeshField, TheSignIsRightAtAReflexVertex) {
    // The L prism's top reflex corner (1, 1, 1): its two reflex walls meet
    // the top cap, whose corner there is 270 degrees. 24 probes within 1e-3 m
    // of it, every sign of each offset. By the L's own geometry, a probe is
    // inside iff it is under the top (z < 1) and not in the notch (x < 1 or
    // y < 1).
    const MeshPtr l = build(l_prism(), "L");
    ASSERT_NE(l, nullptr);
    const float mags[3][3] = {{3e-4f, 5e-4f, 7e-4f}, {7e-4f, 3e-4f, 5e-4f}, {5e-4f, 7e-4f, 3e-4f}};
    int probes = 0;
    int inside_probes = 0;
    for (const auto& m : mags) {
        for (int signs = 0; signs < 8; ++signs) {
            const float sx = (signs & 1) != 0 ? 1.0f : -1.0f;
            const float sy = (signs & 2) != 0 ? 1.0f : -1.0f;
            const float sz = (signs & 4) != 0 ? 1.0f : -1.0f;
            const glm::vec3 p{1.0f + sx * m[0], 1.0f + sy * m[1], 1.0f + sz * m[2]};
            const bool inside = p.z < 1.0f && (p.x < 1.0f || p.y < 1.0f);
            const float d = spade::mesh_eval(*l, p);
            EXPECT_EQ(d < 0.0f, inside) << "(" << p.x << ", " << p.y << ", " << p.z << ") d = " << d;
            EXPECT_LT(std::abs(d), 1e-3f) << "(" << p.x << ", " << p.y << ", " << p.z << ")";
            ++probes;
            inside_probes += inside ? 1 : 0;
        }
    }
    EXPECT_EQ(probes, 24);
    EXPECT_EQ(inside_probes, 9);  // z < 1 (12 probes) less the notch's 3
}

TEST(MeshField, TheSignIsRightWhereNaiveVertexNormalsFail) {
    // A knife: the triangle (0, 0) (-1, 0.1) (-1, -0.1) extruded over z in
    // [0, 1]. Vertex 3, (0, 0, 1), is the tip of its sharp edge. Its upper
    // flank has two triangles there and its lower flank one, so a vertex
    // normal that counts triangles, or weights them by area, leans to the
    // upper flank. Probes just past the lower flank, in vertex 3's own
    // region, are then misread as inside. The angle-weighted normal cannot
    // misread them (Baerentzen and Aanaes 2005).
    const double h = 0.1;
    MeshInput knife;
    knife.unit = MeshUnit::metre;
    knife.positions = {{0, 0, 0}, {-1, h, 0}, {-1, -h, 0}, {0, 0, 1}, {-1, h, 1}, {-1, -h, 1}};
    knife.indices = {0, 2, 1,  3, 4, 5,  1, 2, 5,  1, 5, 4,   // caps, back
                     0, 1, 3,  1, 4, 3,                       // upper flank: two at vertex 3
                     0, 3, 5,  0, 5, 2};                      // lower flank: one at vertex 3
    const MeshPtr mesh = build(knife, "knife");
    ASSERT_NE(mesh, nullptr);

    // The naive normals, computed here from the knife's own triangles.
    const auto pos = mesh->positions();
    const auto idx = mesh->indices();
    glm::dvec3 by_count{0.0};
    glm::dvec3 by_area{0.0};
    for (uint32_t t = 0; t < mesh->triangle_count(); ++t) {
        if (idx[3 * t] != 3 && idx[3 * t + 1] != 3 && idx[3 * t + 2] != 3) continue;
        const glm::dvec3 a = widen(pos[idx[3 * t]]);
        const glm::dvec3 n = glm::cross(widen(pos[idx[3 * t + 1]]) - a, widen(pos[idx[3 * t + 2]]) - a);
        by_count += glm::normalize(n);
        by_area += 0.5 * n;
    }

    // Outside probes: tip + 1e-3 * (n_lower + eps * z + delta * away), with
    // `away` pointing off the lower top edge. Each is past the lower flank's
    // plane (outside the convex knife) and in the tip's vertex region, so its
    // distance is its distance to the tip.
    const glm::dvec3 tip{0.0, 0.0, 1.0};
    const glm::dvec3 n_lower = glm::normalize(glm::dvec3{h, -1.0, 0.0});
    const glm::dvec3 away = glm::normalize(glm::dvec3{1.0, h, 0.0});
    int count_misreads = 0;
    int area_misreads = 0;
    for (const double eps : {0.05, 0.15, 0.3}) {
        for (const double delta : {0.05, 0.15, 0.3}) {
            const glm::dvec3 dir = glm::normalize(n_lower + eps * glm::dvec3{0, 0, 1} + delta * away);
            const glm::vec3 p(tip + 1e-3 * dir);
            const glm::dvec3 off = widen(p) - tip;
            ASSERT_GT(glm::dot(off, n_lower), 0.0) << "the probe must be past the lower flank";
            count_misreads += glm::dot(off, by_count) < 0.0 ? 1 : 0;
            area_misreads += glm::dot(off, by_area) < 0.0 ? 1 : 0;
            const MeshSample s = spade::mesh_sample(*mesh, p);
            EXPECT_GT(s.distance, 0.0f) << "eps " << eps << " delta " << delta;
            EXPECT_NEAR(s.distance, glm::length(off), 1e-9) << "eps " << eps << " delta " << delta;
            EXPECT_EQ(s.triangle, 1u) << "the lowest triangle at the tip, (3, 4, 5)";
        }
    }
    EXPECT_EQ(count_misreads, 9) << "a triangle-counting vertex normal misreads every probe";
    EXPECT_EQ(area_misreads, 9) << "an area-weighted vertex normal misreads every probe";

    // Inside probes near the tip, between the flanks and under the top.
    for (const double side : {-0.02, 0.0, 0.02}) {
        const glm::vec3 p(tip + 1e-3 * glm::normalize(glm::dvec3{-1.0, side, -0.3}));
        EXPECT_LT(spade::mesh_eval(*mesh, p), 0.0f) << "side " << side;
    }
}

TEST(MeshField, AgreesWithTheSdfBoxAtSeededPoints) {
    const glm::vec3 half{0.15f, 0.05f, 0.1f};  // a 0.3 x 0.1 x 0.2 m box about the origin
    MeshInput box = cube({-1.0, -1.0, -1.0}, 2.0);
    for (glm::dvec3& p : box.positions) p *= widen(half);
    const MeshPtr mesh = build(box, "box");
    ASSERT_NE(mesh, nullptr);

    spade::SdfProgram sdf;
    sdf.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode node;
    node.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    node.op = static_cast<uint32_t>(spade::SdfOp::none);
    node.transform = 0;
    node.params = glm::vec4(half, 0.0f);
    sdf.nodes.push_back(node);
    ASSERT_TRUE(sdf.validate().has_value());

    spade::rng::Stream s{0x5eedb0c5ULL, 0.0f, 0u};
    for (int i = 0; i < 10000; ++i) {
        const glm::vec3 p{s.next_float() - 0.5f, s.next_float() - 0.5f, s.next_float() - 0.5f};  // a 1 m cube
        const float want = spade::eval(sdf, p);
        const float got = spade::mesh_eval(*mesh, p);
        ASSERT_NEAR(got, want, 1e-6f) << "point " << i << " (" << p.x << ", " << p.y << ", " << p.z << ")";
        if (std::abs(want) > 1e-6f) {
            ASSERT_EQ(got < 0.0f, want < 0.0f) << "point " << i;
        }
    }
}

TEST(MeshField, TheBvhAnswerEqualsBruteForceBitForBit) {
    struct Fixture {
        const char* name;
        MeshInput input;
        // -1 inside, +1 outside, 0 too near the surface to say.
        std::function<int(const TriangleMesh&, glm::vec3)> side;
    };
    const auto convex = [](const TriangleMesh& m, glm::vec3 p) { return convex_side(m, p, 1e-6); };
    const Fixture fixtures[] = {
        {"cube", cube({-0.3, 0.1, 0.7}, 0.8), convex},
        {"tetrahedron", tetrahedron(), convex},
        {"icosphere", icosphere(3), convex},
        {"plate", plate(1.0, 0.002, 0.1, 8), convex},
        {"cube with a cavity", merged(unit_cube(), flipped(cube({0.25, 0.3, 0.35}, 0.4))),
         [](const TriangleMesh&, glm::vec3 p) {
             const int outer = box_side({0, 0, 0}, {1, 1, 1}, p, 1e-6);
             const int inner = box_side({0.25, 0.3, 0.35}, {0.65, 0.7, 0.75}, p, 1e-6);
             if (outer == 0 || inner == 0) return 0;
             return (outer < 0 && inner > 0) ? -1 : 1;
         }},
    };
    for (const Fixture& f : fixtures) {
        SCOPED_TRACE(f.name);
        const MeshPtr mesh = build(f.input, f.name);
        ASSERT_NE(mesh, nullptr);
        if (std::string(f.name) == "icosphere") {
            ASSERT_EQ(mesh->triangle_count(), 1280u);
        }
        const auto pos = mesh->positions();
        const auto idx = mesh->indices();
        glm::dvec3 lo = widen(pos[0]);
        glm::dvec3 hi = lo;
        for (const glm::vec3& v : pos) {
            lo = glm::min(lo, widen(v));
            hi = glm::max(hi, widen(v));
        }
        const glm::dvec3 size = hi - lo;
        const double diagonal = glm::length(size);

        // 2000 seeded points: 1000 in the bounds grown by half on each side,
        // 600 just off random triangles (both sides, 1e-2 to 1e-4 of the
        // diagonal), 200 on vertices and 200 near edge midpoints.
        spade::rng::Stream s{0xb1a5edULL ^ static_cast<uint64_t>(mesh->triangle_count()), 0.0f, 0u};
        std::vector<glm::vec3> points;
        for (int i = 0; i < 1000; ++i) {
            const glm::dvec3 u{unit_double(s), unit_double(s), unit_double(s)};
            points.emplace_back(lo - 0.5 * size + 2.0 * size * u);
        }
        for (int i = 0; i < 600; ++i) {
            const uint32_t t = static_cast<uint32_t>(s.next_u64() % mesh->triangle_count());
            double r1 = unit_double(s);
            double r2 = unit_double(s);
            if (r1 + r2 > 1.0) {
                r1 = 1.0 - r1;
                r2 = 1.0 - r2;
            }
            const glm::dvec3 a = widen(pos[idx[3 * t]]);
            const glm::dvec3 b = widen(pos[idx[3 * t + 1]]);
            const glm::dvec3 c = widen(pos[idx[3 * t + 2]]);
            const glm::dvec3 n = glm::normalize(glm::cross(b - a, c - a));
            const double scale = diagonal * (i % 3 == 0 ? 1e-2 : (i % 3 == 1 ? 1e-3 : 1e-4));
            const double sign = (i % 2 == 0) ? 1.0 : -1.0;
            points.emplace_back(a + r1 * (b - a) + r2 * (c - a) + sign * scale * n);
        }
        for (int i = 0; i < 200; ++i) points.push_back(pos[s.next_u64() % pos.size()]);
        for (int i = 0; i < 200; ++i) {
            const uint32_t t = static_cast<uint32_t>(s.next_u64() % mesh->triangle_count());
            const uint32_t k = static_cast<uint32_t>(s.next_u64() % 3);
            points.emplace_back(0.5 * (widen(pos[idx[3 * t + k]]) + widen(pos[idx[3 * t + (k + 1) % 3]])));
        }

        int signs_checked = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const glm::vec3 p = points[i];
            const Brute want = brute_force(*mesh, p);
            const MeshSample got = spade::mesh_sample(*mesh, p);
            ASSERT_EQ(got.triangle, want.triangle) << "point " << i;
            const float len = std::sqrt(want.distance2);
            ASSERT_EQ(std::bit_cast<uint32_t>(std::abs(got.distance)), std::bit_cast<uint32_t>(len)) << "point " << i;
            if (want.distance2 > 0.0f) {
                // gradient = sign * (p - c) / |p - c|, componentwise.
                const glm::vec3 diff = p - want.point;
                glm::vec3 g = diff / len;
                if (got.distance < 0.0f) g = -g;
                ASSERT_EQ(std::bit_cast<uint32_t>(got.gradient.x), std::bit_cast<uint32_t>(g.x)) << "point " << i;
                ASSERT_EQ(std::bit_cast<uint32_t>(got.gradient.y), std::bit_cast<uint32_t>(g.y)) << "point " << i;
                ASSERT_EQ(std::bit_cast<uint32_t>(got.gradient.z), std::bit_cast<uint32_t>(g.z)) << "point " << i;
            }
            const int side = f.side(*mesh, p);
            if (side != 0) {
                ASSERT_EQ(got.distance < 0.0f, side < 0) << "point " << i << " (" << p.x << ", " << p.y << ", " << p.z
                                                         << ") d = " << got.distance;
                ++signs_checked;
            }
        }
        EXPECT_GT(signs_checked, 1500);
    }
}

namespace {

// The distance between two doubles in units in the last place: their
// positions in the ordered sequence of doubles (-0 and +0 are one position).
uint64_t ulp_distance(double a, double b) {
    const auto key = [](double v) {
        const int64_t i = std::bit_cast<int64_t>(v);
        return i < 0 ? std::numeric_limits<int64_t>::min() - i : i;
    };
    const int64_t ka = key(a);
    const int64_t kb = key(b);
    return ka > kb ? static_cast<uint64_t>(ka) - static_cast<uint64_t>(kb)
                   : static_cast<uint64_t>(kb) - static_cast<uint64_t>(ka);
}

bool same_bits(double a, double b) {
    return std::bit_cast<uint64_t>(a) == std::bit_cast<uint64_t>(b);
}

}  // namespace

TEST(MeshField, CornerAnglesUseNoLibm) {
    // corner_angle against literal constants: the doubles nearest pi/2, pi/3,
    // pi/4 and 2 pi/3 (to 200 bits, mpmath), in hex. Near 2 pi/3 a double's
    // spacing is 4.4e-16, so 4e-16 there means the nearest double exactly.
    const double r = std::sqrt(0.75);
    EXPECT_NEAR(spade::math::corner_angle({1, 0, 0}, {0, 2, 0}), 0x1.921fb54442d18p+0, 4e-16);
    EXPECT_NEAR(spade::math::corner_angle({1, 2, 3}, {-2, 1, 0}), 0x1.921fb54442d18p+0, 4e-16);
    EXPECT_NEAR(spade::math::corner_angle({1, 0, 0}, {0.5, r, 0}), 0x1.0c152382d7366p+0, 4e-16);
    EXPECT_NEAR(spade::math::corner_angle({1, 0, 0}, {1, 1, 0}), 0x1.921fb54442d18p-1, 4e-16);
    EXPECT_NEAR(spade::math::corner_angle({1, 0, 0}, {-0.5, r, 0}), 0x1.0c152382d7366p+1, 4e-16);

    // Exact on the axes and the diagonals: the doubles nearest pi/2, pi, pi/4
    // and 3 pi/4, with every sign of zero.
    const double pi = 0x1.921fb54442d18p+1;
    const double half_pi = 0x1.921fb54442d18p+0;
    const double quarter_pi = 0x1.921fb54442d18p-1;
    const double three_quarter_pi = 0x1.2d97c7f3321d2p+1;
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(0.0, 2.0), 0.0));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(-0.0, 2.0), -0.0));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(0.0, -2.0), pi));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(-0.0, -2.0), -pi));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(3.0, 0.0), half_pi));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(3.0, -0.0), half_pi));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(-3.0, 0.0), -half_pi));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(5.0, 5.0), quarter_pi));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(-5.0, 5.0), -quarter_pi));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(5.0, -5.0), three_quarter_pi));
    EXPECT_TRUE(same_bits(spade::math::atan2_arith(-5.0, -5.0), -three_quarter_pi));

    // Every special operand pair agrees with the oracle bit for bit: signed
    // zeros, infinities, the subnormal and normal extremes.
    const double inf = std::numeric_limits<double>::infinity();
    const double specials[] = {0.0, -0.0, 1.0, -1.0, inf, -inf, 0x1p-1074, -0x1p-1074,
                               std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), 2.5, -3.75};
    for (const double y : specials) {
        for (const double x : specials) {
            EXPECT_TRUE(same_bits(spade::math::atan2_arith(y, x), std::atan2(y, x)))
                << "atan2(" << y << ", " << x << ") = " << spade::math::atan2_arith(y, x) << ", libm "
                << std::atan2(y, x);
        }
    }
    EXPECT_TRUE(std::isnan(spade::math::atan2_arith(std::numeric_limits<double>::quiet_NaN(), 1.0)));
    EXPECT_TRUE(std::isnan(spade::math::atan2_arith(1.0, std::numeric_limits<double>::quiet_NaN())));

    // 10 000 seeded points in each quadrant, across the whole domain: both
    // magnitudes in (0, 1); one exponent with the other within 2^+-40; two
    // independent exponents anywhere from the subnormals to the largest
    // finite; and ratios near 1, where the octants meet. Within 2 ulp of
    // std::atan2 (the bound: atan2_arith is within 1 ulp of the correctly
    // rounded value, and so is any faithful libm).
    spade::rng::Stream s{0xa7a2a7a2ULL, 0.0f, 0u};
    uint64_t worst = 0;
    double worst_y = 0.0;
    double worst_x = 0.0;
    int checked = 0;
    for (int i = 0; i < 40000; ++i) {
        const double sy = (i & 1) != 0 ? -1.0 : 1.0;
        const double sx = (i & 2) != 0 ? -1.0 : 1.0;
        double y = 0.0;
        double x = 0.0;
        switch ((i >> 2) % 4) {
            case 0:
                y = unit_double(s);
                x = unit_double(s);
                break;
            case 1: {
                const int e = static_cast<int>(s.next_u64() % 2001) - 1000;
                y = std::ldexp(1.0 + unit_double(s), e);
                x = std::ldexp(1.0 + unit_double(s), e + static_cast<int>(s.next_u64() % 81) - 40);
                break;
            }
            case 2:
                y = std::ldexp(1.0 + unit_double(s), static_cast<int>(s.next_u64() % 2098) - 1074);
                x = std::ldexp(1.0 + unit_double(s), static_cast<int>(s.next_u64() % 2098) - 1074);
                break;
            default:
                y = 1.0 + unit_double(s);
                x = y * (1.0 + (unit_double(s) - 0.5) * 0x1p-20);
                break;
        }
        y *= sy;
        x *= sx;
        if (y == 0.0 || x == 0.0 || !std::isfinite(y) || !std::isfinite(x)) continue;
        const uint64_t d = ulp_distance(spade::math::atan2_arith(y, x), std::atan2(y, x));
        if (d > worst) {
            worst = d;
            worst_y = y;
            worst_x = x;
        }
        ++checked;
    }
    std::printf("atan2_arith worst %llu ulp from std::atan2 over %d seeded points (at y = %a, x = %a)\n",
                static_cast<unsigned long long>(worst), checked, worst_y, worst_x);
    EXPECT_LE(worst, 2u) << "at y = " << worst_y << ", x = " << worst_x;
    EXPECT_GT(checked, 39000);
}

TEST(MeshField, ASelfIntersectingMeshIsRefusedNamingThePair) {
    // Two cubes that interpenetrate, one input: the first's top, back and
    // right faces cut the second's bottom, front and left faces.
    const auto two = spade::make_mesh(merged(unit_cube(), cube({0.4, 0.3, 0.2}, 1.0)), "fixture");
    ASSERT_FALSE(two.has_value());
    EXPECT_EQ(two.error().code, Code::invalid_argument);
    EXPECT_EQ(two.error().context,
              "fixture: 18 self-intersecting triangle pairs, first (2, 16) (2, 17) (2, 20) (2, 21) (3, 17) "
              "(3, 21) (6, 12) (6, 13)");

    // A cube with vertex 7 pushed through the bottom face: still closed,
    // oriented and of positive volume, but its three triangles at vertex 7
    // now pierce the bottom. (1, 7) and (1, 11) share vertex 3: the far-edge
    // test catches them.
    MeshInput pushed = unit_cube();
    pushed.positions[7] = {0.3, 0.4, -0.6};
    const auto bad = spade::make_mesh(pushed, "fixture");
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error().code, Code::invalid_argument);
    EXPECT_EQ(bad.error().context,
              "fixture: 5 self-intersecting triangle pairs, first (0, 3) (0, 7) (0, 11) (1, 7) (1, 11)");

    // What a fan of coplanar triangles is not: the L prism's caps share a
    // vertex in one plane, and are accepted (TheSignIsRightAtAReflexVertex).
    EXPECT_TRUE(spade::make_mesh(l_prism(), "L").has_value());
}

TEST(MeshField, ACavityIsAcceptedAndANestedSolidIsRefused) {
    // A cube with an inward inner cube: a cavity. The cavity is outside the solid.
    const auto cavity = spade::make_mesh(merged(unit_cube(), flipped(cube({0.25, 0.25, 0.25}, 0.5))), "cavity");
    ASSERT_TRUE(cavity.has_value()) << cavity.error().context;
    EXPECT_EQ((*cavity)->shell_count(), 2u);
    EXPECT_EQ(spade::mesh_eval(**cavity, {0.5f, 0.5f, 0.5f}), 0.25f) << "the cavity's centre";
    EXPECT_EQ(spade::mesh_eval(**cavity, {0.125f, 0.5f, 0.5f}), -0.125f) << "in the wall";
    EXPECT_EQ(spade::mesh_eval(**cavity, {1.5f, 0.5f, 0.5f}), 0.5f) << "outside";

    // An outward cube inside another: the winding number would be 2 inside it.
    const auto nested = spade::make_mesh(merged(unit_cube(), cube({0.25, 0.25, 0.25}, 0.5)), "fixture");
    ASSERT_FALSE(nested.has_value());
    EXPECT_EQ(nested.error().code, Code::invalid_argument);
    EXPECT_EQ(nested.error().context,
              "fixture: 1 wrongly nested shell (the winding number must be 0 or 1 everywhere), first 1 (outward, "
              "sum 1, needs 0)");

    // An inward cube outside every solid: a cavity in nothing.
    const auto stray = spade::make_mesh(merged(unit_cube(), flipped(cube({3.0, 0.0, 0.0}, 0.5))), "fixture");
    ASSERT_FALSE(stray.has_value());
    EXPECT_EQ(stray.error().code, Code::invalid_argument);
    EXPECT_EQ(stray.error().context,
              "fixture: 1 wrongly nested shell (the winding number must be 0 or 1 everywhere), first 1 (inward, "
              "sum 0, needs 1)");

    // Two disjoint outward cubes, and an island in a cavity, are solids.
    const auto apart = spade::make_mesh(merged(unit_cube(), cube({3.0, 0.0, 0.0}, 1.0)), "apart");
    ASSERT_TRUE(apart.has_value()) << apart.error().context;
    EXPECT_EQ((*apart)->shell_count(), 2u);
    const auto island = spade::make_mesh(
        merged(merged(cube({0, 0, 0}, 4.0), flipped(cube({1, 1, 1}, 2.0))), cube({1.5, 1.5, 1.5}, 1.0)), "island");
    ASSERT_TRUE(island.has_value()) << island.error().context;
    EXPECT_EQ((*island)->shell_count(), 3u);
    EXPECT_EQ(spade::mesh_eval(**island, {2.0f, 2.0f, 2.0f}), -0.5f) << "in the island";
    EXPECT_EQ(spade::mesh_eval(**island, {1.25f, 2.0f, 2.0f}), 0.25f) << "in the cavity, beside the island";

    // More shells than the nesting check takes: 4097 disjoint tetrahedra.
    MeshInput many;
    many.unit = MeshUnit::metre;
    const MeshInput tet = tetrahedron();
    for (uint32_t k = 0; k <= spade::kMaxMeshShells; ++k) {
        const uint32_t base = static_cast<uint32_t>(many.positions.size());
        for (const glm::dvec3& p : tet.positions) many.positions.push_back(p + glm::dvec3{2.0 * k, 0.0, 0.0});
        for (const uint32_t i : tet.indices) many.indices.push_back(base + i);
    }
    const auto crowd = spade::make_mesh(many, "fixture");
    ASSERT_FALSE(crowd.has_value());
    EXPECT_EQ(crowd.error().code, Code::capacity_exceeded);
    EXPECT_EQ(crowd.error().context, "fixture: 4097 shells, more than the 4096 a mesh may hold");
}
