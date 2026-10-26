// The library drones' generated shapes (L7a of the realistic drones plan,
// docs/design/core/plans/2026-10-26-l7-realistic-drones-draft.md):
// world/mesh_shapes.hpp's rounded box, capped tube, motor bell and propeller.
//
// What is pinned here:
//   - EACH SHAPE IS A MESH make_mesh() ACCEPTS: closed, outward, one shell
//     (a propeller: its hub and one per blade), its min_feature its thinnest
//     dimension.
//   - THE GEOMETRY ITS NUMBERS ASK FOR: the box's extents exactly, its every
//     edge rounded; the tube's span; the bell under the rotor plane; the
//     propeller through its stations (chord and pitch at each, the tip at R,
//     nothing in the hub gap), its leading edge leading its turn, and the cw
//     hand the ccw mirrored bit for bit and re-wound.
//   - A BAD SPEC IS REFUSED BY ITS FIELD, before make_mesh() sees it.
//   - THE BITS: each shape's counts and content_hash(), pinned from MSVC; the
//     Docker gcc leg reproduces them (TD-12).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "core/fp32_math.hpp"
#include "world/mesh.hpp"
#include "world/mesh_shapes.hpp"

namespace {

namespace sh = spade::shapes;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)

constexpr double kPi = 3.141592653589793;

struct Bounds {
    glm::vec3 lo{std::numeric_limits<float>::max()};
    glm::vec3 hi{-std::numeric_limits<float>::max()};
};

[[nodiscard]] Bounds bounds_of(const spade::TriangleMesh& mesh) {
    Bounds b;
    for (const glm::vec3& p : mesh.positions()) {
        b.lo = glm::min(b.lo, p);
        b.hi = glm::max(b.hi, p);
    }
    return b;
}

[[nodiscard]] double radial(const glm::vec3& p) {
    const double x = p.x;
    const double z = p.z;
    return std::sqrt(x * x + z * z);
}

// The Micro Invent 5x4 three-blade (assets/library/parts/uiuc_mit_5x4.prop.yaml),
// with an assumed section.
[[nodiscard]] sh::PropMeshSpec mit_5x4() {
    sh::PropMeshSpec s;
    s.diameter = 0.127;
    s.blades = 3;
    s.r_over_r = {0.15, 0.2,  0.25, 0.3, 0.35, 0.4,  0.45, 0.5,  0.55,
                  0.6,  0.65, 0.7,  0.75, 0.8, 0.85, 0.9, 0.95, 1.0};
    s.c_over_r = {0.1121, 0.1297, 0.1477, 0.1639, 0.1764, 0.1855, 0.192,  0.1947, 0.1932,
                  0.1884, 0.1826, 0.176,  0.1673, 0.1559, 0.1402, 0.1175, 0.085,  0.0112};
    s.beta_deg = {23.418, 28.485, 31.062, 31.635, 30.435, 28.445, 26.557, 24.911, 23.346,
                  21.695, 20.204, 18.79,  17.386, 16.05,  15.075, 14.595, 14.073, 13.817};
    s.hub_cutout = 0.15;
    s.hub_gap = 0.0005;
    s.hub_height = 0.006;
    s.hub_segments = 16;
    s.section = sh::PropSection{0.04, 0.4, 0.12, 0.08, 8};
    return s;
}

// The Crazyflie's two-blade (assets/library/parts/uiuc_cfnq_45t1.prop.yaml),
// with an assumed section.
[[nodiscard]] sh::PropMeshSpec cfnq_45t1() {
    sh::PropMeshSpec s;
    s.diameter = 0.04699;
    s.blades = 2;
    s.r_over_r = {0.15, 0.2,  0.25, 0.3, 0.35, 0.4,  0.45, 0.5,  0.55,
                  0.6,  0.65, 0.7,  0.75, 0.8, 0.85, 0.9, 0.95, 1.0};
    s.c_over_r = {0.2476, 0.2652, 0.2841, 0.3034, 0.3193, 0.3353, 0.3515, 0.3649, 0.3734,
                  0.3806, 0.3837, 0.3843, 0.3821, 0.3741, 0.3541, 0.3158, 0.2341, 0.0532};
    s.beta_deg = {29.21,  26.737, 24.255, 22.505, 21.187, 19.917, 18.828, 17.896, 17.284,
                  16.874, 16.482, 16.034, 15.644, 15.206, 14.891, 15.176, 15.23,  13.154};
    s.hub_cutout = 0.15;
    s.hub_gap = 0.0005;
    s.hub_height = 0.004;
    s.hub_segments = 16;
    s.section = sh::PropSection{0.04, 0.4, 0.12, 0.08, 8};
    return s;
}

// Blade 0's section at station j (THE PROPELLER, world/mesh_shapes.hpp): its
// vertices in the plane x = r_j, its leading edge the least along the chord
// and its trailing edge the midpoint of the two most.
struct Section {
    glm::dvec3 le{0.0};
    glm::dvec3 te{0.0};
    std::size_t count = 0;
};

[[nodiscard]] Section section_at(const spade::TriangleMesh& mesh, const sh::PropMeshSpec& s, std::size_t j,
                                 double chord_z, double chord_y) {
    const float x = static_cast<float>(s.r_over_r[j] * s.diameter * 0.5);
    std::vector<std::pair<double, glm::dvec3>> along;
    for (const glm::vec3& p : mesh.positions()) {
        if (p.x != x) continue;
        const glm::dvec3 d(p);
        along.emplace_back(d.z * chord_z + d.y * chord_y, d);
    }
    Section out;
    out.count = along.size();
    if (along.size() < 3) return out;
    std::sort(along.begin(), along.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    out.le = along.front().second;
    out.te = 0.5 * (along[along.size() - 1].second + along[along.size() - 2].second);
    return out;
}

[[nodiscard]] float deg_to_rad(double deg) { return static_cast<float>(deg * kPi / 180.0); }

}  // namespace

TEST(MeshShapes, EachShapeIsAClosedOutwardMeshThatMakeMeshAccepts) {
    const auto box = sh::rounded_box(glm::dvec3(0.18, 0.05, 0.14), 0.008, 4);
    const auto tube = sh::capped_tube(0.008, 0.1, 16);
    const auto bell = sh::motor_bell(0.014, 0.02, 0.002, 24);
    const auto ccw = sh::prop(mit_5x4(), sh::PropHand::ccw);
    const auto cw = sh::prop(mit_5x4(), sh::PropHand::cw);
    ASSERT_OK(box);
    ASSERT_OK(tube);
    ASSERT_OK(bell);
    ASSERT_OK(ccw);
    ASSERT_OK(cw);
    EXPECT_EQ(box->mesh->shell_count(), 1u);
    EXPECT_EQ(tube->mesh->shell_count(), 1u);
    EXPECT_EQ(bell->mesh->shell_count(), 1u);
    EXPECT_EQ(ccw->mesh->shell_count(), 4u) << "the hub and three blades";
    EXPECT_EQ(cw->mesh->shell_count(), 4u);
    for (const sh::GeneratedMesh* g : {&*box, &*tube, &*bell, &*ccw, &*cw}) {
        EXPECT_GT(g->mesh->mass_properties().volume, 0.0);
        EXPECT_GT(g->min_feature, 0.0f);
    }
    EXPECT_EQ(box->min_feature, 0.05f);
    EXPECT_EQ(tube->min_feature, 0.016f);
    EXPECT_EQ(bell->min_feature, 0.02f);
    // The thinnest section: t c, smallest at the tip, 0.08 * 0.0112 * R.
    EXPECT_NEAR(ccw->min_feature, 0.08 * 0.0112 * 0.0635, 1e-9);
    // The two-blade Crazyflie prop, too.
    const auto two = sh::prop(cfnq_45t1(), sh::PropHand::ccw);
    ASSERT_OK(two);
    EXPECT_EQ(two->mesh->shell_count(), 3u);
}

TEST(MeshShapes, TheRoundedBoxKeepsItsExtentsAndRoundsEveryEdge) {
    const glm::dvec3 size(0.18, 0.05, 0.14);
    const double rho = 0.008;
    const auto box = sh::rounded_box(size, rho, 4);
    ASSERT_OK(box);
    const Bounds b = bounds_of(*box->mesh);
    for (int a = 0; a < 3; ++a) {
        EXPECT_EQ(b.hi[a], static_cast<float>(size[a] * 0.5)) << "axis " << a;
        EXPECT_EQ(b.lo[a], -static_cast<float>(size[a] * 0.5)) << "axis " << a;
    }
    // Every edge rounded: no vertex lies on two of the box's faces at once.
    for (const glm::vec3& p : box->mesh->positions()) {
        int on = 0;
        for (int a = 0; a < 3; ++a) on += (p[a] == b.hi[a] || p[a] == b.lo[a]) ? 1 : 0;
        ASSERT_LE(on, 1) << "(" << p.x << ", " << p.y << ", " << p.z << ") is on an unrounded edge";
    }
    // The outermost corner vertex sits on the corner sphere: rho from the
    // inner box's corner.
    const glm::dvec3 inner = size * 0.5 - glm::dvec3(rho);
    glm::dvec3 corner(0.0);
    double best = -1.0;
    for (const glm::vec3& p : box->mesh->positions()) {
        const double s = static_cast<double>(p.x) + p.y + p.z;
        if (s > best) {
            best = s;
            corner = glm::dvec3(p);
        }
    }
    EXPECT_NEAR(glm::length(corner - inner), rho, 1e-7);
    // The volume: more than the inner box with its six face slabs, less than
    // the exact rounded box (its vertices lie on the convex surface).
    const glm::dvec3 in = size - glm::dvec3(2.0 * rho);
    const double slabs = in.x * in.y * in.z + 2.0 * rho * (in.x * in.y + in.y * in.z + in.z * in.x);
    const double exact = slabs + kPi * rho * rho * (in.x + in.y + in.z) + 4.0 / 3.0 * kPi * rho * rho * rho;
    const double v = box->mesh->mass_properties().volume;
    EXPECT_GT(v, slabs);
    EXPECT_LT(v, exact);
}

TEST(MeshShapes, TheCappedTubeSpansItsLengthPlusItsCaps) {
    const double r = 0.008;
    const double len = 0.1;
    const auto tube = sh::capped_tube(r, len, 16);
    ASSERT_OK(tube);
    const Bounds b = bounds_of(*tube->mesh);
    EXPECT_EQ(b.hi.y, static_cast<float>(len * 0.5 + r));
    EXPECT_EQ(b.lo.y, -static_cast<float>(len * 0.5 + r));
    double widest = 0.0;
    for (const glm::vec3& p : tube->mesh->positions()) widest = std::max(widest, radial(p));
    EXPECT_NEAR(widest, r, 1e-8) << "the cylinder's chords end on its radius";
    const double exact = kPi * r * r * len + 4.0 / 3.0 * kPi * r * r * r;
    const double v = tube->mesh->mass_properties().volume;
    EXPECT_LT(v, exact);
    EXPECT_GT(v, 0.95 * exact) << "16 chords lose under 3% of a circle's area";
}

TEST(MeshShapes, TheMotorBellSitsBelowTheRotorPlane) {
    const auto bell = sh::motor_bell(0.014, 0.02, 0.002, 24);
    ASSERT_OK(bell);
    const Bounds b = bounds_of(*bell->mesh);
    EXPECT_EQ(b.hi.y, 0.0f) << "the top face is the rotor plane";
    EXPECT_EQ(b.lo.y, -0.02f);
    double widest = 0.0;
    for (const glm::vec3& p : bell->mesh->positions()) widest = std::max(widest, radial(p));
    EXPECT_NEAR(widest, 0.014, 1e-8);
    // The chamfer: no vertex at the top face's full radius.
    for (const glm::vec3& p : bell->mesh->positions()) {
        if (p.y == 0.0f) {
            EXPECT_LE(radial(p), 0.012 + 1e-8);
        }
    }
}

TEST(MeshShapes, ThePropFollowsItsBladeStations) {
    const sh::PropMeshSpec s = cfnq_45t1();
    const auto prop = sh::prop(s, sh::PropHand::ccw);
    ASSERT_OK(prop);
    const double big_r = s.diameter * 0.5;
    for (std::size_t j = 0; j < s.r_over_r.size(); ++j) {
        SCOPED_TRACE("station " + std::to_string(j));
        const float beta = deg_to_rad(s.beta_deg[j]);
        const double cz = spade::math::cos32(beta);
        const double cy = -static_cast<double>(spade::math::sin32(beta));
        const Section sec = section_at(*prop->mesh, s, j, cz, cy);
        ASSERT_EQ(sec.count, 2u * s.section.chord_points + 1u) << "one ring of the section's points";
        const glm::dvec3 chord = sec.te - sec.le;
        const double c = s.c_over_r[j] * big_r;
        EXPECT_NEAR(glm::length(chord), c, 2e-6 * big_r);
        EXPECT_NEAR(chord.z / glm::length(chord), cz, 2e-5);
        EXPECT_NEAR(chord.y / glm::length(chord), cy, 2e-5);
        // The pitch turns about the quarter chord, on the radius line.
        EXPECT_NEAR((sec.le + 0.25 * chord).z, 0.0, 2e-6 * big_r);
        EXPECT_NEAR((sec.le + 0.25 * chord).y, 0.0, 2e-6 * big_r);
    }
    const Bounds b = bounds_of(*prop->mesh);
    EXPECT_EQ(b.hi.x, static_cast<float>(big_r)) << "blade 0's tip reaches R";
    // Nothing in the gap between the hub and the blades' roots.
    const double hub = s.hub_cutout * big_r - s.hub_gap;
    const double root = s.r_over_r.front() * big_r;
    for (const glm::vec3& p : prop->mesh->positions()) {
        const double rr = radial(p);
        ASSERT_TRUE(rr <= hub * (1.0 + 1e-6) || rr >= root * (1.0 - 1e-6))
            << "(" << p.x << ", " << p.y << ", " << p.z << ") is " << rr << " m out, in the gap";
    }
}

TEST(MeshShapes, ACcwBladeLeadsWithItsSpin) {
    const sh::PropMeshSpec s = cfnq_45t1();
    const std::size_t j = 8;  // mid-blade
    const float beta = deg_to_rad(s.beta_deg[j]);
    const double cz = spade::math::cos32(beta);
    const double cy = -static_cast<double>(spade::math::sin32(beta));
    const auto ccw = sh::prop(s, sh::PropHand::ccw);
    const auto cw = sh::prop(s, sh::PropHand::cw);
    ASSERT_OK(ccw);
    ASSERT_OK(cw);
    // A +psi turn about +Y carries +X toward -Z (vehicles/part_pose.hpp), so
    // the ccw blade leads on -z, its leading edge above its trailing edge.
    const Section a = section_at(*ccw->mesh, s, j, cz, cy);
    EXPECT_LT(a.le.z, 0.0);
    EXPECT_GT(a.te.z, 0.0);
    EXPECT_GT(a.le.y, a.te.y);
    // The cw blade turns the other way, so it leads on +z.
    const Section b = section_at(*cw->mesh, s, j, -cz, cy);
    EXPECT_GT(b.le.z, 0.0);
    EXPECT_LT(b.te.z, 0.0);
    EXPECT_GT(b.le.y, b.te.y);
}

TEST(MeshShapes, TheCwHandIsTheCcwMirroredAndReWound) {
    const auto ccw = sh::prop(mit_5x4(), sh::PropHand::ccw);
    const auto cw = sh::prop(mit_5x4(), sh::PropHand::cw);
    ASSERT_OK(ccw);
    ASSERT_OK(cw);
    const auto pa = ccw->mesh->positions();
    const auto pb = cw->mesh->positions();
    ASSERT_EQ(pa.size(), pb.size());
    for (std::size_t i = 0; i < pa.size(); ++i) {
        ASSERT_EQ(pb[i].x, pa[i].x) << i;
        ASSERT_EQ(pb[i].y, pa[i].y) << i;
        ASSERT_EQ(pb[i].z, -pa[i].z) << i;
    }
    const auto ia = ccw->mesh->indices();
    const auto ib = cw->mesh->indices();
    ASSERT_EQ(ia.size(), ib.size());
    for (std::size_t t = 0; t < ia.size(); t += 3) {
        ASSERT_EQ(ib[t], ia[t]) << t / 3;
        ASSERT_EQ(ib[t + 1], ia[t + 2]) << t / 3;
        ASSERT_EQ(ib[t + 2], ia[t + 1]) << t / 3;
    }
    EXPECT_NEAR(cw->mesh->mass_properties().volume, ccw->mesh->mass_properties().volume,
                1e-12 * ccw->mesh->mass_properties().volume);
    EXPECT_EQ(cw->min_feature, ccw->min_feature);
}

TEST(MeshShapes, ABadSpecIsRefusedByName) {
    const auto refused = [](const auto& r, std::string_view field) {
        if (r) return testing::AssertionFailure() << "accepted; expected a refusal naming " << field;
        if (r.error().code != spade::Code::invalid_argument) {
            return testing::AssertionFailure() << "code " << static_cast<int>(r.error().code);
        }
        if (r.error().context.find(field) == std::string::npos) {
            return testing::AssertionFailure() << "'" << r.error().context << "' does not name " << field;
        }
        return testing::AssertionSuccess();
    };
    const glm::dvec3 size(0.1, 0.05, 0.1);
    EXPECT_TRUE(refused(sh::rounded_box(size, 0.025, 4), "edge_radius")) << "half the smallest extent";
    EXPECT_TRUE(refused(sh::rounded_box(size, 0.0, 4), "edge_radius"));
    EXPECT_TRUE(refused(sh::rounded_box(glm::dvec3(0.1, 0.0, 0.1), 0.01, 4), "size"));
    EXPECT_TRUE(refused(sh::rounded_box(size, 0.01, 0), "segments"));
    EXPECT_TRUE(refused(sh::capped_tube(0.0, 0.1, 16), "radius"));
    EXPECT_TRUE(refused(sh::capped_tube(0.01, 0.1, 2), "segments"));
    EXPECT_TRUE(refused(sh::motor_bell(0.014, 0.02, 0.014, 16), "chamfer"));
    EXPECT_TRUE(refused(sh::motor_bell(0.014, 0.02, 0.0, 16), "chamfer"));

    const auto bad = [](auto edit) {
        sh::PropMeshSpec s = mit_5x4();
        edit(s);
        return sh::prop(s, sh::PropHand::ccw);
    };
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.blades = 0; }), "blades"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.diameter = 0.0; }), "diameter"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.c_over_r[3] = 0.0; }), "c_over_r[3]"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.r_over_r[5] = s.r_over_r[4]; }), "r_over_r[5]"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.r_over_r.back() = 1.01; }), "r_over_r[17]"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.beta_deg[2] = 90.0; }), "beta_deg[2]"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.c_over_r.pop_back(); }), "c_over_r"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.hub_cutout = 0.2; }), "hub_cutout"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.hub_gap = 0.01; }), "hub_gap"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.hub_height = 0.0; }), "hub_height"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.hub_segments = 2; }), "hub_segments"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.section.chord_points = 1; }), "chord_points"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.section.thickness_tip = 0.0; }), "thickness_tip"));
    EXPECT_TRUE(refused(bad([](sh::PropMeshSpec& s) { s.section.camber_at = 1.0; }), "camber_at"));
}

// The bits, pinned from MSVC; the Docker gcc leg reproduces them (TD-12). A
// moved pin needs a named cause (TD-1): the generator, make_mesh(), or
// sin32 / cos32.
TEST(MeshShapes, TheShapesHashesArePinned) {
    struct Pin {
        const char* name;
        spade::Result<sh::GeneratedMesh> made;
        std::size_t vertices;
        uint32_t triangles;
        uint64_t hash;
    };
    const std::array<Pin, 5> pins{{
        {"rounded_box", sh::rounded_box(glm::dvec3(0.18, 0.05, 0.14), 0.008, 4), 488, 972, 0x0ull},
        {"capped_tube", sh::capped_tube(0.008, 0.1, 16), 130, 256, 0x0ull},
        {"motor_bell", sh::motor_bell(0.014, 0.02, 0.002, 24), 74, 144, 0x0ull},
        {"prop ccw", sh::prop(mit_5x4(), sh::PropHand::ccw), 952, 1888, 0x0ull},
        {"prop cw", sh::prop(mit_5x4(), sh::PropHand::cw), 952, 1888, 0x0ull},
    }};
    for (const Pin& p : pins) {
        SCOPED_TRACE(p.name);
        ASSERT_OK(p.made);
        const spade::TriangleMesh& m = *p.made->mesh;
        EXPECT_EQ(m.positions().size(), p.vertices);
        EXPECT_EQ(m.triangle_count(), p.triangles);
        std::printf("%s: %zu vertices, %u triangles, content_hash 0x%016llx\n", p.name, m.positions().size(),
                    m.triangle_count(), static_cast<unsigned long long>(m.content_hash()));
        EXPECT_EQ(m.content_hash(), p.hash);
    }
}
