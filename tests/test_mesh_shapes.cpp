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
//     hand the ccw mirrored bit for bit and re-wound; every library
//     propeller's blade lofts in both hands.
//   - A BAD SPEC IS REFUSED BY ITS FIELD, in the generator's own words,
//     before make_mesh() sees it, and so is every count past its limit.
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
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "core/fp32_math.hpp"
#include "scene/part_file.hpp"
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
    // min_feature is the lesser of the diameter and the height.
    EXPECT_EQ(bell->min_feature, 0.02f) << "the height";
    const auto can = sh::motor_bell(0.0035, 0.016, 0.001, 16);  // the Crazyflie's 7 x 16 mm can
    ASSERT_OK(can);
    EXPECT_EQ(can->min_feature, 0.007f) << "the diameter";
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
    // The same through the pose's own turn (vehicles/part_pose.cpp): the
    // quaternion (cos32(a/2), 0, sin32(a/2), 0) at a = spin_dir psi carries
    // the blade toward its leading edge.
    const auto motion = [](float turn, const glm::dvec3& at) {
        const glm::quat q(spade::math::cos32(0.5f * turn), 0.0f, spade::math::sin32(0.5f * turn), 0.0f);
        const glm::vec3 p(at);
        return glm::dvec3(q * p) - glm::dvec3(p);
    };
    EXPECT_GT(glm::dot(a.le - a.te, motion(+0.01f, 0.5 * (a.le + a.te))), 0.0) << "spin_dir +1";
    EXPECT_GT(glm::dot(b.le - b.te, motion(-0.01f, 0.5 * (b.le + b.te))), 0.0) << "spin_dir -1";
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

// Every propeller in the default library lofts in both hands with an
// assumed section: the blades L7c1 puts on the library drones' hubs.
TEST(MeshShapes, EveryLibraryBladeMakesBothHands) {
    const std::filesystem::path parts = std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "parts";
    std::vector<std::filesystem::path> files;
    for (const auto& e : std::filesystem::directory_iterator(parts)) {
        if (e.path().filename().string().ends_with(".prop.yaml")) files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());
    EXPECT_EQ(files.size(), 5u) << "L4's five library propellers";
    for (const std::filesystem::path& path : files) {
        SCOPED_TRACE(path.filename().string());
        std::ifstream in(path, std::ios::binary);
        ASSERT_TRUE(in);
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const auto part = spade::scene::prop_from_yaml(text);
        ASSERT_OK(part);
        ASSERT_TRUE(part->blade.has_value()) << "every library propeller carries its blade (L4a)";
        sh::PropMeshSpec s;
        s.diameter = part->block.diameter;
        s.blades = part->block.blades;
        s.r_over_r = part->blade->r_over_r;
        s.c_over_r = part->blade->c_over_r;
        s.beta_deg = part->blade->beta_deg;
        s.hub_cutout = part->blade->hub_cutout;
        s.hub_gap = 0.0005;
        s.hub_height = 0.004;
        s.hub_segments = 16;
        s.section = sh::PropSection{0.04, 0.4, 0.12, 0.08, 8};
        for (const sh::PropHand hand : {sh::PropHand::ccw, sh::PropHand::cw}) {
            const auto prop = sh::prop(s, hand);
            ASSERT_OK(prop);
            EXPECT_EQ(prop->mesh->shell_count(), s.blades + 1u);
            EXPECT_GT(prop->mesh->mass_properties().volume, 0.0);
        }
    }
}

TEST(MeshShapes, ABadSpecIsRefusedByName) {
    // Refused by the generator itself, before make_mesh(): its own prefix,
    // then the field.
    const auto refused = [](const auto& r, std::string_view fn, std::string_view field) {
        if (r) return testing::AssertionFailure() << "accepted; expected a refusal naming " << field;
        const std::string& said = r.error().context;
        if (r.error().code != spade::Code::invalid_argument) {
            return testing::AssertionFailure() << "code " << static_cast<int>(r.error().code) << ": " << said;
        }
        if (said.rfind(std::string(fn) + ": ", 0) != 0) {
            return testing::AssertionFailure() << "'" << said << "' is not " << fn << "'s own refusal";
        }
        if (said.find(field) == std::string::npos) {
            return testing::AssertionFailure() << "'" << said << "' does not name " << field;
        }
        return testing::AssertionSuccess();
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const glm::dvec3 size(0.1, 0.05, 0.1);
    EXPECT_TRUE(refused(sh::rounded_box(size, 0.025, 4), "rounded_box", "edge_radius")) << "half the smallest extent";
    EXPECT_TRUE(refused(sh::rounded_box(size, 0.0, 4), "rounded_box", "edge_radius"));
    EXPECT_TRUE(refused(sh::rounded_box(size, nan, 4), "rounded_box", "edge_radius"));
    EXPECT_TRUE(refused(sh::rounded_box(glm::dvec3(0.1, 0.0, 0.1), 0.01, 4), "rounded_box", "size"));
    EXPECT_TRUE(refused(sh::rounded_box(glm::dvec3(0.1, nan, 0.1), 0.01, 4), "rounded_box", "size"));
    EXPECT_TRUE(refused(sh::rounded_box(size, 0.01, 0), "rounded_box", "segments"));
    EXPECT_TRUE(refused(sh::rounded_box(size, 0.01, sh::kMaxBoxSegments + 1), "rounded_box", "segments"));
    EXPECT_TRUE(refused(sh::rounded_box(size, 0.01, 0xFFFFFFFFu), "rounded_box", "segments"));
    EXPECT_TRUE(refused(sh::capped_tube(0.0, 0.1, 16), "capped_tube", "radius"));
    EXPECT_TRUE(refused(sh::capped_tube(0.01, 0.0, 16), "capped_tube", "length"));
    EXPECT_TRUE(refused(sh::capped_tube(0.01, nan, 16), "capped_tube", "length"));
    EXPECT_TRUE(refused(sh::capped_tube(0.01, 0.1, 2), "capped_tube", "segments"));
    EXPECT_TRUE(refused(sh::capped_tube(0.01, 0.1, sh::kMaxRingSegments + 1), "capped_tube", "segments"));
    EXPECT_TRUE(refused(sh::motor_bell(0.0, 0.02, 0.002, 16), "motor_bell", "radius"));
    EXPECT_TRUE(refused(sh::motor_bell(0.014, 0.0, 0.002, 16), "motor_bell", "height"));
    EXPECT_TRUE(refused(sh::motor_bell(0.014, 0.02, 0.014, 16), "motor_bell", "chamfer"));
    EXPECT_TRUE(refused(sh::motor_bell(0.014, 0.02, 0.0, 16), "motor_bell", "chamfer"));
    EXPECT_TRUE(refused(sh::motor_bell(0.014, 0.02, 0.002, 2), "motor_bell", "segments"));
    EXPECT_TRUE(refused(sh::motor_bell(0.014, 0.02, 0.002, sh::kMaxRingSegments + 1), "motor_bell", "segments"));

    const auto bad = [&](auto edit, std::string_view field) {
        sh::PropMeshSpec s = mit_5x4();
        edit(s);
        return refused(sh::prop(s, sh::PropHand::ccw), "prop", field);
    };
    using Spec = sh::PropMeshSpec;
    EXPECT_TRUE(bad([](Spec& s) { s.diameter = 0.0; }, "diameter"));
    EXPECT_TRUE(bad([&](Spec& s) { s.diameter = nan; }, "diameter"));
    EXPECT_TRUE(bad([](Spec& s) { s.blades = 0; }, "blades"));
    EXPECT_TRUE(bad([](Spec& s) { s.blades = sh::kMaxPropBlades + 1; }, "blades"));
    EXPECT_TRUE(bad(
        [](Spec& s) {
            s.r_over_r = {0.5};
            s.c_over_r = {0.1};
            s.beta_deg = {20.0};
        },
        "r_over_r"));
    EXPECT_TRUE(bad(
        [](Spec& s) {
            const std::size_t n = sh::kMaxPropStations + 1;
            s.r_over_r.clear();
            for (std::size_t j = 0; j < n; ++j) s.r_over_r.push_back(0.15 + 0.85 * static_cast<double>(j) / (n - 1));
            s.c_over_r.assign(n, 0.1);
            s.beta_deg.assign(n, 20.0);
        },
        "r_over_r"));
    EXPECT_TRUE(bad([](Spec& s) { s.c_over_r.pop_back(); }, "c_over_r"));
    EXPECT_TRUE(bad([](Spec& s) { s.beta_deg.pop_back(); }, "beta_deg"));
    EXPECT_TRUE(bad([](Spec& s) { s.hub_cutout = -0.1; }, "hub_cutout"));
    EXPECT_TRUE(bad([](Spec& s) { s.hub_cutout = 0.2; }, "hub_cutout")) << "r_over_r[0] starts below it";
    EXPECT_TRUE(bad([](Spec& s) { s.r_over_r[5] = s.r_over_r[4]; }, "r_over_r[5]"));
    EXPECT_TRUE(bad([](Spec& s) { s.r_over_r.back() = 1.01; }, "r_over_r[17]"));
    EXPECT_TRUE(bad([](Spec& s) { s.c_over_r[3] = 0.0; }, "c_over_r[3]"));
    EXPECT_TRUE(bad([](Spec& s) { s.beta_deg[2] = 90.0; }, "beta_deg[2]"));
    EXPECT_TRUE(bad([&](Spec& s) { s.beta_deg[2] = nan; }, "beta_deg[2]"));
    EXPECT_TRUE(bad([](Spec& s) { s.hub_gap = 0.0; }, "hub_gap"));
    EXPECT_TRUE(bad([](Spec& s) { s.hub_gap = 0.01; }, "hub_gap")) << "it leaves no hub";
    EXPECT_TRUE(bad([](Spec& s) { s.hub_height = 0.0; }, "hub_height"));
    EXPECT_TRUE(bad([](Spec& s) { s.hub_segments = 2; }, "hub_segments"));
    EXPECT_TRUE(bad([](Spec& s) { s.hub_segments = sh::kMaxRingSegments + 1; }, "hub_segments"));
    EXPECT_TRUE(bad([](Spec& s) { s.section.camber = 0.2; }, "camber"));
    EXPECT_TRUE(bad([](Spec& s) { s.section.camber_at = 1.0; }, "camber_at"));
    EXPECT_TRUE(bad([](Spec& s) { s.section.thickness_root = 0.0; }, "thickness_root"));
    EXPECT_TRUE(bad([](Spec& s) { s.section.thickness_tip = 0.5; }, "thickness_tip"));
    EXPECT_TRUE(bad([](Spec& s) { s.section.chord_points = 1; }, "chord_points"));
    EXPECT_TRUE(bad([](Spec& s) { s.section.chord_points = sh::kMaxChordPoints + 1; }, "chord_points"));
    EXPECT_TRUE(bad([](Spec& s) { s.section.chord_points = 0xFFFFFFFFu; }, "chord_points"));
}

// A shape's tensor has its shape's symmetry to rounding (L7b2): a mesh worn
// as a mass-bearing part must not tilt a symmetric drone's principal axes.
// The rounded box is mirror symmetric in all three axes, so its products of
// inertia vanish; a lathe of n = 4m chords has quarter-turn symmetry about Y,
// so its products vanish and Ixx = Izz. "Vanish" is the composite's own
// zeroing rule (composite_inertia.cpp): at most 1e-12 of the trace, far under
// float output. One diagonal in every rounded-box cell gave about 1e-6, and
// ring vertices from cos32 / sin32 of each angle apart about 1e-10.
TEST(MeshShapes, EachShapesTensorHasItsShapesSymmetry) {
    struct Case {
        const char* name;
        spade::Result<sh::GeneratedMesh> made;
        bool lathe;  // quarter-turn symmetric about Y: Ixx = Izz too
    };
    const std::array<Case, 4> cases{{
        {"rounded_box", sh::rounded_box(glm::dvec3(0.18, 0.05, 0.14), 0.008, 4), false},
        {"rounded_box, 1 segment", sh::rounded_box(glm::dvec3(0.03, 0.02, 0.05), 0.004, 1), false},
        {"capped_tube", sh::capped_tube(0.008, 0.1, 32), true},
        {"motor_bell", sh::motor_bell(0.014, 0.02, 0.002, 24), true},
    }};
    for (const Case& c : cases) {
        SCOPED_TRACE(c.name);
        ASSERT_OK(c.made);
        const glm::dmat3& t = c.made->mesh->mass_properties().inertia;
        const double trace = t[0][0] + t[1][1] + t[2][2];
        const double tolerance = 1e-12 * trace;
        EXPECT_LE(std::fabs(t[1][0]), tolerance) << "Ixy " << t[1][0] << " of trace " << trace;
        EXPECT_LE(std::fabs(t[2][0]), tolerance) << "Ixz " << t[2][0] << " of trace " << trace;
        EXPECT_LE(std::fabs(t[2][1]), tolerance) << "Iyz " << t[2][1] << " of trace " << trace;
        if (c.lathe) {
            EXPECT_LE(std::fabs(t[0][0] - t[2][2]), tolerance) << "Ixx - Izz of trace " << trace;
        }
    }
}

// The bits, pinned from MSVC; the Docker gcc leg reproduces them (TD-12). A
// moved pin needs a named cause (TD-1): the generator, make_mesh(), or
// sin32 / cos32. L7b2 moved all five for the generators' symmetry
// (EachShapesTensorHasItsShapesSymmetry): a rounded box's cells split toward
// each face's centre, and a ring of 4m chords -- a tube's, a bell's, a prop's
// hub -- built from its first quarter by exact quarter turns.
TEST(MeshShapes, TheShapesHashesArePinned) {
    struct Pin {
        const char* name;
        spade::Result<sh::GeneratedMesh> made;
        std::size_t vertices;
        uint32_t triangles;
        uint64_t hash;
    };
    const std::array<Pin, 5> pins{{
        {"rounded_box", sh::rounded_box(glm::dvec3(0.18, 0.05, 0.14), 0.008, 4), 488, 972, 0x1d1bea82bc1ae81full},
        {"capped_tube", sh::capped_tube(0.008, 0.1, 16), 130, 256, 0x0daa45297fbef734ull},
        {"motor_bell", sh::motor_bell(0.014, 0.02, 0.002, 24), 74, 144, 0xf03a29362915f3f7ull},
        {"prop ccw", sh::prop(mit_5x4(), sh::PropHand::ccw), 952, 1888, 0xf244ae80035931dcull},
        {"prop cw", sh::prop(mit_5x4(), sh::PropHand::cw), 952, 1888, 0x4d06f5840a5162f4ull},
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
