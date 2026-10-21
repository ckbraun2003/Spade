// The solid field (the solids design, sections 2-4; P2's Tasks 1, 2 and 8):
// apertures from one sampler, labelled sheets, the classifier, the measured
// safeguard and the viscous wall links. Shapes and bounds are analytic
// (tests/airflow_solid_shapes.hpp); expectations are worked from the
// geometry, not read from the code (TD-4).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/solids.hpp"
#include "physics/airflow/viscous.hpp"

namespace {

using namespace spade::physics::airflow;
using spade::Result;
using airflow_test::Kind;
using airflow_test::Scene;
using airflow_test::Shape;

[[nodiscard]] SolidField built(const GridShape& shape, const Scene& scene, std::array<bool, 3> periodic = {}) {
    SolidField f = *make_solid_field(shape);
    const std::vector<SolidPart> parts = scene.parts();
    const Result<void> ok = build_solids(f, shape, periodic, parts, scene.sampler());
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
    return f;
}

[[nodiscard]] float theta_at(const SolidField& f, const GridShape& s, uint32_t c, uint32_t i, uint32_t j, uint32_t k) {
    const std::size_t idx = c == 0 ? u_index(s, i, j, k) : (c == 1 ? v_index(s, i, j, k) : w_index(s, i, j, k));
    return f.theta[c][idx];
}

TEST(AirflowSolids, TheClassifierSortsWallsSheetsAndWires) {
    const float dx = 0.01f;
    SolidPart wall{.name = "hull", .porous = false, .min_feature = 0.03f, .extent = {0.10f, 0.05f, 0.04f}};
    const auto w = classify_part(wall, dx);
    ASSERT_TRUE(w.has_value());
    EXPECT_EQ(w->role, SolidRole::wall);

    SolidPart plate{.name = "frame_top", .porous = false, .min_feature = 0.002f, .extent = {0.10f, 0.002f, 0.10f}};
    const auto p = classify_part(plate, dx);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p->role, SolidRole::sheet);
    EXPECT_EQ(p->thickness, 0.002f);

    // Declared thick, but its bounds are a 4 mm plate: the extent catches it.
    SolidPart misdeclared{.name = "board", .porous = false, .min_feature = 0.05f, .extent = {0.10f, 0.004f, 0.10f}};
    const auto m = classify_part(misdeclared, dx);
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->role, SolidRole::sheet);
    EXPECT_EQ(m->thickness, 0.004f);

    SolidPart wire{.name = "antenna", .porous = false, .min_feature = 0.002f, .extent = {0.002f, 0.003f, 0.20f}};
    const auto r = classify_part(wire, dx);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("antenna"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("needs dx <= 1.0 mm"), std::string::npos) << r.error().context;

    SolidPart screen{.name = "screen", .porous = true, .min_feature = 0.03f, .extent = {0.1f, 0.1f, 0.1f}};
    const auto s = classify_part(screen, dx);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("porous"), std::string::npos) << s.error().context;
}

// A sphere of radius 4 cells centred on a cell corner: every sampled face takes
// the ramp's aperture clamp(0.5 + phi / dx), mirror-symmetric on all three
// axes, and the faces far from it stay open.
TEST(AirflowSolids, ASphereGivesTheRampsApertureAndIsMirrorSymmetric) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "ball", .kind = Kind::sphere, .centre = {8.0f, 8.0f, 8.0f}, .radius = 4.0f,
                                 .min_feature = 8.0f});
    const SolidField f = built(shape, scene);
    EXPECT_TRUE(f.any);
    uint32_t closed_faces = 0;
    for (uint32_t c = 0; c < 3; ++c) {
        for_each_sample(shape, c, [&](std::size_t idx, glm::vec3 x) {
            const glm::vec3 d = x - glm::vec3(8.0f);
            const float phi = std::sqrt(glm::dot(d, d)) - 4.0f;
            const float expected = std::min(std::max(0.5f + phi, 0.0f), 1.0f);
            EXPECT_EQ(f.theta[c][idx], expected) << c << " " << idx;
            if (f.theta[c][idx] == 0.0f) ++closed_faces;
            EXPECT_EQ(f.wall[c][idx], 0.0f);
        });
    }
    EXPECT_GT(closed_faces, 0u);
    for (uint32_t k = 0; k < 16; ++k) {
        for (uint32_t j = 0; j < 16; ++j) {
            for (uint32_t i = 0; i <= 16; ++i) {
                EXPECT_EQ(theta_at(f, shape, 0, i, j, k), theta_at(f, shape, 0, 16 - i, j, k));
                EXPECT_EQ(theta_at(f, shape, 0, i, j, k), theta_at(f, shape, 0, i, 15 - j, k));
            }
        }
    }
}

// A sheet 0.25 dx thick across a periodic channel closes exactly one plane of
// u faces: the one it lies on, or, on a cell-centre plane, the lower of the
// two (the half-open rule), and no v or w face.
TEST(AirflowSolids, ASheetClosesExactlyOneFacePlane) {
    const GridShape shape{32, 8, 8, 1.0f};
    for (const float at : {16.0f, 17.0f, 16.5f}) {
        Scene scene;
        scene.shapes.push_back(Shape{.name = "plate", .kind = Kind::box, .centre = {at, 4.0f, 4.0f},
                                     .half = {0.125f, 20.0f, 20.0f}, .min_feature = 0.25f});
        SolidField f = *make_solid_field(shape);
        std::vector<SolidPart> parts = scene.parts();
        // The plate spans the whole section: its box is clipped to the grid
        // across the periodic y and z axes, as a caller's posed box would be.
        parts[0].box_min = glm::vec3(at - 0.125f, 0.0f, 0.0f);
        parts[0].box_max = glm::vec3(at + 0.125f, 8.0f, 8.0f);
        parts[0].extent = glm::vec3(0.25f, 8.0f, 8.0f);
        ASSERT_TRUE(build_solids(f, shape, {true, true, true}, parts, scene.sampler()).has_value());
        const uint32_t plane = at == 17.0f ? 17u : 16u;
        for (uint32_t k = 0; k < 8; ++k) {
            for (uint32_t j = 0; j < 8; ++j) {
                for (uint32_t i = 0; i <= 32; ++i) {
                    EXPECT_EQ(theta_at(f, shape, 0, i, j, k), i == plane ? 0.0f : 1.0f) << at << ": " << i;
                }
            }
        }
        for (uint32_t c = 1; c < 3; ++c) {
            for (const float t : f.theta[c]) EXPECT_EQ(t, 1.0f);
        }
        EXPECT_NE(f.run_line.find("plate: sheet (250.0 mm < 2 dx = 2000.0 mm)"), std::string::npos) << f.run_line;
    }
}

TEST(AirflowSolids, APartWhollyOutsideTheGridIsRefused) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "stray", .kind = Kind::sphere, .centre = {40.0f, 8.0f, 8.0f}, .radius = 2.0f,
                                 .min_feature = 4.0f});
    SolidField f = *make_solid_field(shape);
    const std::vector<SolidPart> parts = scene.parts();
    const Result<void> r = build_solids(f, shape, {}, parts, scene.sampler());
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("stray"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("wholly outside the grid"), std::string::npos) << r.error().context;
    EXPECT_FALSE(f.any) << "nothing written";
}

// A part reaching past the grid is clipped, not refused: a wall filling the
// domain's edge (S2's tube) is ordinary validation geometry.
TEST(AirflowSolids, APartReachingPastTheGridIsClipped) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "slab", .kind = Kind::box, .centre = {8.0f, 8.0f, 0.0f},
                                 .half = {20.0f, 20.0f, 3.0f}, .min_feature = 6.0f});
    const SolidField f = built(shape, scene);
    EXPECT_TRUE(f.any);
    EXPECT_EQ(theta_at(f, shape, 2, 8, 8, 1), 0.0f) << "z = 1: 2 dx inside the slab";
    EXPECT_EQ(theta_at(f, shape, 2, 8, 8, 6), 1.0f) << "z = 6: 3 dx outside it";
}

TEST(AirflowSolids, ABadSamplerIsRefusedByName) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "ball", .kind = Kind::sphere, .centre = {8.0f, 8.0f, 8.0f}, .radius = 3.0f,
                                 .min_feature = 6.0f});
    const std::vector<SolidPart> parts = scene.parts();
    const auto nan_distance = [](const void*, glm::vec3) noexcept {
        return SolidQuery{std::numeric_limits<float>::quiet_NaN(), glm::vec3(1.0f, 0.0f, 0.0f), 0u, glm::vec3(0.0f)};
    };
    const auto past_the_list = [](const void*, glm::vec3) noexcept {
        return SolidQuery{-1.0f, glm::vec3(1.0f, 0.0f, 0.0f), 7u, glm::vec3(0.0f)};
    };
    const auto wild_velocity = [](const void*, glm::vec3) noexcept {
        return SolidQuery{-1.0f, glm::vec3(1.0f, 0.0f, 0.0f), 0u, glm::vec3(std::numeric_limits<float>::infinity())};
    };
    const struct {
        SolidQuery (*fn)(const void*, glm::vec3) noexcept;
        const char* says;
    } cases[] = {{nan_distance, "non-finite distance"}, {past_the_list, "part 7"}, {wild_velocity, "non-finite velocity"}};
    for (const auto& c : cases) {
        SolidField f = *make_solid_field(shape);
        const Result<void> r = build_solids(f, shape, {}, parts, SolidSampler{&scene, c.fn, &Scene::one});
        ASSERT_FALSE(r.has_value()) << c.says;
        EXPECT_NE(r.error().context.find(c.says), std::string::npos) << r.error().context;
        EXPECT_FALSE(f.any);
        for (uint32_t comp = 0; comp < 3; ++comp) {
            for (const float t : f.theta[comp]) ASSERT_EQ(t, 1.0f) << "a refused build leaves every face open";
        }
    }
}

TEST(AirflowSolids, NoPartsLeavesEveryFaceOpen) {
    const GridShape shape{8, 8, 8, 1.0f};
    SolidField f = *make_solid_field(shape);
    const Scene scene;
    ASSERT_TRUE(build_solids(f, shape, {}, {}, scene.sampler()).has_value());
    EXPECT_FALSE(f.any);
    for (uint32_t c = 0; c < 3; ++c) {
        for (const float t : f.theta[c]) EXPECT_EQ(t, 1.0f);
    }
    EXPECT_EQ(f.run_line, "solids: none");
}

// --- The measured safeguard (design section 4; P2 Task 2) -----------------

[[nodiscard]] Result<void> build_scene(SolidField& f, const GridShape& shape, const std::vector<SolidPart>& parts,
                                       const Scene& scene) {
    return build_solids(f, shape, {}, parts, scene.sampler());
}

// A sheet's seal (P2 Task 11, the final review's finding 1): the faces whose
// centre-to-centre segment the mid-surface m crosses (m changes sign between
// the two cell centres) at a point `on` accepts, left open. A watertight sheet
// leaves none. The grid's origin is 0.
template <class Mid, class On>
[[nodiscard]] uint32_t open_crossings(const SolidField& f, const GridShape& shape, Mid m, On on) {
    uint32_t open = 0;
    const float dx = shape.dx;
    for (uint32_t c = 0; c < 3; ++c) {
        const uint32_t n[3] = {shape.nx + (c == 0 ? 1u : 0u), shape.ny + (c == 1 ? 1u : 0u),
                               shape.nz + (c == 2 ? 1u : 0u)};
        for (uint32_t k = 0; k < n[2]; ++k) {
            for (uint32_t j = 0; j < n[1]; ++j) {
                for (uint32_t i = 0; i < n[0]; ++i) {
                    const glm::vec3 x((static_cast<float>(i) + (c == 0 ? 0.0f : 0.5f)) * dx,
                                      (static_cast<float>(j) + (c == 1 ? 0.0f : 0.5f)) * dx,
                                      (static_cast<float>(k) + (c == 2 ? 0.0f : 0.5f)) * dx);
                    glm::vec3 e(0.0f);
                    e[static_cast<glm::length_t>(c)] = 0.5f * dx;
                    const float ma = m(x - e);
                    const float mb = m(x + e);
                    if (!(ma * mb < 0.0f)) continue;
                    const glm::vec3 cross = x - e + (ma / (ma - mb)) * (2.0f * e);
                    if (!on(cross)) continue;
                    const std::size_t idx = i + std::size_t{n[0]} * (j + std::size_t{n[1]} * k);
                    if (!closed(f.theta[c][idx])) ++open;
                }
            }
        }
    }
    return open;
}

// A 12-cell hub with a fin 0.3 dx thick on its +z face, centred on the face
// plane x = 16, 2 cells tall: every x line through the fin passes centres
// 0.35 dx outside on both sides (the screen's 0.7 dx), and the face between
// them has theta = 0.35. One part, declared thick.
[[nodiscard]] Shape hub_with_fin(float fin_length, float fin_half_height) {
    return Shape{.name = "hub_fin", .kind = Kind::two_boxes, .centre = {16.0f, 16.0f, 16.0f},
                 .half = {6.0f, 6.0f, 6.0f}, .centre2 = {16.0f, 16.0f, 22.0f + 0.5f * fin_length},
                 .half2 = {0.15f, fin_half_height, 0.5f * fin_length}, .min_feature = 12.0f};
}

// A fin 8 cells long: its leaking runs reach 8 cells past the hub's closed
// faces, beyond the 4-cell edge reach. Refused. Under the screen mutation
// phi1 + phi2 < dx/2 the fin's runs (0.7 dx) are skipped and this fails.
TEST(AirflowSolidsSafeguard, ALongFinOnAHubIsRefused) {
    const GridShape shape{32, 32, 36, 1.0f};
    Scene scene;
    scene.shapes.push_back(hub_with_fin(8.0f, 3.0f));
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("hub_fin"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("without meeting a closed face"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("declare min_feature or refine dx"), std::string::npos) << r.error().context;
}

// The same hub with a fin 3 cells long and 2 tall: six leaking runs, each
// within 3 cells of the hub's closed faces, open area 6 x 0.35 dx^2 = 2.1 dx^2
// against 2% of the bounds' largest face (12 x 15 = 180 dx^2: 3.6). Announced.
TEST(AirflowSolidsSafeguard, AShortFinOnAHubIsAnnouncedWithItsOpenArea) {
    const GridShape shape{32, 32, 32, 1.0f};
    Scene scene;
    scene.shapes.push_back(hub_with_fin(3.0f, 1.0f));
    SolidField f = *make_solid_field(shape);
    ASSERT_TRUE(build_scene(f, shape, scene.parts(), scene).has_value());
    EXPECT_EQ(f.classes[0].role, SolidRole::wall);
    EXPECT_NE(f.run_line.find("hub_fin: leaks at an edge (6 runs, open area 2.100 dx^2, 1.17% of its largest face)"),
              std::string::npos)
        << f.run_line;
}

// A fin 1.5 dx thick centred on the cell-centre plane x = 16.5: the centre in
// the fin is inside (k = 1), its two faces have theta = 0.25. Found and refused.
TEST(AirflowSolidsSafeguard, AFinOneAndAHalfCellsThickIsFound) {
    const GridShape shape{32, 32, 36, 1.0f};
    Scene scene;
    Shape s = hub_with_fin(8.0f, 3.0f);
    s.centre2.x = 16.5f;
    s.half2.x = 0.75f;
    scene.shapes.push_back(s);
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("without meeting a closed face"), std::string::npos) << r.error().context;
}

// A sphere 2 dx across, centred on a cell corner: the nearest face centres
// are 0.71 dx from its centre (theta 0.21), so it closes no face. Every line
// through it passes 0.71 R off centre, where n_in . n_out = 2 rho^2 - 1 = 0:
// no thin crossing, so not sheet-like either. Refused.
TEST(AirflowSolidsSafeguard, ATwoDxSphereAtItsWorstAlignmentIsRefused) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "bead", .kind = Kind::sphere, .centre = {8.0f, 8.0f, 8.0f}, .radius = 1.0f,
                                 .min_feature = 2.0f});
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("bead"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("closes no face"), std::string::npos) << r.error().context;
}

// A 4 dx box turned 45 degrees about y. Lines along x and z enter and leave
// through adjacent faces (n_in . n_out = 0); lines along y oppose but run the
// box's full 4 dx (tau = 4 dx). Not flagged.
TEST(AirflowSolidsSafeguard, ABoxTurnedFortyFiveDegreesIsNotFlagged) {
    const GridShape shape{32, 32, 32, 1.0f};
    const float h = 0.70710678f;
    Scene scene;
    scene.shapes.push_back(Shape{.name = "turned", .kind = Kind::box, .centre = {16.0f, 16.0f, 16.0f},
                                 .half = {2.0f, 2.0f, 2.0f},
                                 .to_local = glm::mat3(h, 0.0f, -h, 0.0f, 1.0f, 0.0f, h, 0.0f, h),
                                 .min_feature = 4.0f});
    SolidField f = *make_solid_field(shape);
    ASSERT_TRUE(build_scene(f, shape, scene.parts(), scene).has_value());
    EXPECT_EQ(f.classes[0].role, SolidRole::wall);
    EXPECT_EQ(f.run_line.find("leaks"), std::string::npos) << f.run_line;
}

// A 30-degree wedge (half-angle 15 degrees), 64 cells of chord, 8 of span:
// its tip leaks within the 4-cell reach of the closed faces that begin 2-4
// cells back, and the leak's open area is far under 2% of its planform
// (64 x 34.3 dx^2). Announced, with the count and the area. Measured on the
// first run and pinned (TD-2; 2026-10-13, MSVC release): 16 runs, 3.859 dx^2,
// 0.18% of the planform.
TEST(AirflowSolidsSafeguard, AThirtyDegreeWedgesTipIsAnnounced) {
    const GridShape shape{80, 12, 40, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "wedge", .kind = Kind::wedge_y, .centre = {8.0f, 6.0f, 20.0f},
                                 .half = {64.0f, 4.0f, 0.0f}, .radius = 0.26179939f, .min_feature = 4.0f});
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(f.classes[0].role, SolidRole::wall);
    EXPECT_NE(f.run_line.find("wedge: leaks at an edge (16 runs, open area 3.859 dx^2, 0.18% of its largest face)"),
              std::string::npos)
        << f.run_line;
}

// A plate 0.5 dx thick, declared 4 dx and turned 30 degrees about z inside
// its own frame, so its bounds look thick (10.6 x 6.4 x 12 dx) and the
// classifier calls it a wall. It closes no face, and every line crossing it
// is thin (tau = 0.5 dx): sheet-like. Sheeted with its measured thickness,
// announced, and never "at an edge".
TEST(AirflowSolidsSafeguard, ATiltedThinPlateDeclaredThickIsSheeted) {
    const GridShape shape{32, 32, 32, 1.0f};
    const float c = 0.8660254f;
    const float s = 0.5f;
    Scene scene;
    scene.shapes.push_back(Shape{.name = "tilted", .kind = Kind::box, .centre = {16.0f, 16.0f, 16.0f},
                                 .half = {6.0f, 0.25f, 6.0f},
                                 .to_local = glm::mat3(c, -s, 0.0f, s, c, 0.0f, 0.0f, 0.0f, 1.0f),
                                 .min_feature = 4.0f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = parts[0].box_max - parts[0].box_min;  // a mesh tilted in its own frame
    SolidField f = *make_solid_field(shape);
    const Result<void> built_ok = build_scene(f, shape, parts, scene);
    ASSERT_TRUE(built_ok.has_value()) << built_ok.error().context;
    EXPECT_EQ(f.classes[0].role, SolidRole::sheet);
    EXPECT_NEAR(f.classes[0].thickness, 0.5f, 0.05f);
    EXPECT_NE(f.run_line.find("tilted: sheet (measured"), std::string::npos) << f.run_line;
    EXPECT_EQ(f.run_line.find("at an edge"), std::string::npos) << f.run_line;
    uint32_t closed_faces = 0;
    for (uint32_t comp = 0; comp < 3; ++comp) {
        for (const float t : f.theta[comp]) closed_faces += t == 0.0f ? 1u : 0u;
    }
    EXPECT_GT(closed_faces, 0u) << "as a sheet it closes the faces it crosses";
    // Watertight (the final review's F6): every line through the plate's
    // middle crosses a closed face. Its measured thickness is its own, so the
    // class-thickness mutation passes here; the canopy and the cambered plate
    // below are the red tests for that.
    const glm::mat3 to_local = scene.shapes[0].to_local;
    const glm::vec3 centre = scene.shapes[0].centre;
    const auto mid = [&](glm::vec3 x) { return (to_local * (x - centre)).y; };
    const auto on = [&](glm::vec3 x) {
        const glm::vec3 l = to_local * (x - centre);
        return std::fabs(l.x) <= 5.0f && std::fabs(l.z) <= 5.0f;
    };
    EXPECT_EQ(open_crossings(f, shape, mid, on), 0u) << "a line through the plate's middle crosses no closed face";
}

// A plate 0.5 dx thick with no declared feature, its part-frame box 1.5 dx
// thick (a cambered mesh plate): the classifier calls it a sheet of 1.5 dx,
// its bounds' smallest extent. Its faces must still close where its true
// mid-surface crosses (P2 Task 11, the final review's finding 1). Mutation:
// the mid-surface from the class thickness, which leaves every line open.
TEST(AirflowSolidsSafeguard, ASheetThinnerThanItsClassStillSeals) {
    const GridShape shape{32, 32, 32, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "cambered", .kind = Kind::box, .centre = {16.0f, 16.3f, 16.0f},
                                 .half = {6.0f, 0.25f, 6.0f}, .min_feature = 0.0f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = glm::vec3(12.0f, 1.5f, 12.0f);
    SolidField f = *make_solid_field(shape);
    const Result<void> built_ok = build_scene(f, shape, parts, scene);
    ASSERT_TRUE(built_ok.has_value()) << built_ok.error().context;
    ASSERT_EQ(f.classes[0].role, SolidRole::sheet);
    EXPECT_FLOAT_EQ(f.classes[0].thickness, 1.5f);
    const auto mid = [](glm::vec3 x) { return x.y - 16.3f; };
    const auto on = [](glm::vec3 x) { return std::fabs(x.x - 16.0f) <= 5.0f && std::fabs(x.z - 16.0f) <= 5.0f; };
    EXPECT_EQ(open_crossings(f, shape, mid, on), 0u);
}

// A sheet running out of a wall (a thin arm leaving a body): the body's face
// at x = 20.3 is 0.2 dx from the y faces at x = 20.5, nearer than the sheet's
// surface, so the wall won them and its ramp left them 0.7 open along the
// join, though the sheet's mid-surface (y = 16.45) crosses them (P2 Task 11,
// the final review's finding 2). Mutation: theta from the nearest part only.
TEST(AirflowSolidsSafeguard, ASheetLeavingAWallLeavesNoSlotAtTheJoin) {
    const GridShape shape{40, 32, 32, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "body", .kind = Kind::box, .centre = {16.0f, 16.0f, 16.0f},
                                 .half = {4.3f, 4.3f, 4.3f}, .min_feature = 8.6f});
    scene.shapes.push_back(Shape{.name = "arm", .kind = Kind::box, .centre = {23.0f, 16.45f, 16.0f},
                                 .half = {7.0f, 0.125f, 3.0f}, .min_feature = 0.25f});
    SolidField f = *make_solid_field(shape);
    const Result<void> built_ok = build_scene(f, shape, scene.parts(), scene);
    ASSERT_TRUE(built_ok.has_value()) << built_ok.error().context;
    ASSERT_EQ(f.classes[1].role, SolidRole::sheet);
    const auto mid = [](glm::vec3 x) { return x.y - 16.45f; };
    const auto on = [](glm::vec3 x) { return x.x >= 20.3f && x.x <= 29.0f && std::fabs(x.z - 16.0f) <= 2.0f; };
    EXPECT_EQ(open_crossings(f, shape, mid, on), 0u) << "the arm seals right up to the body";
}

// The grid does not wrap a part across a periodic seam, so a part within 2 dx
// of one periodic face but not the other is refused by name; one spanning the
// whole axis is allowed (P2 Task 11, the final review's finding 5).
TEST(AirflowSolidsSafeguard, APartAcrossAPeriodicSeamIsRefused) {
    const GridShape shape{32, 32, 32, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "seam_ball", .kind = Kind::sphere, .centre = {1.0f, 16.0f, 16.0f},
                                 .radius = 4.0f, .min_feature = 8.0f});
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_solids(f, shape, {true, false, false}, scene.parts(), scene.sampler());
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("seam_ball"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("periodic x face"), std::string::npos) << r.error().context;
    EXPECT_TRUE(build_solids(f, shape, {false, false, false}, scene.parts(), scene.sampler()).has_value());
}

// A square rod 0.8 dx across and 12 dx long, its axis along (1, 1, 1) inside
// its own frame, so its bounds look thick (about 7.4 dx each way) and the
// classifier calls it a wall. It closes no face. Lines along x and y cross it
// thin through its faces normal to a = (1, -1, 0)/sqrt(2); lines along z
// through those normal to b = (1, 1, -2)/sqrt(6). At each, 2 dx across the rod
// along a tangent lands about 1.6 dx outside it either way: narrow, a wire and
// not a sheet. Refused (the lead, 2026-10-13: a sheet is wide along both
// tangents at every thin crossing). Mutation: no width check, and it is
// sheeted.
TEST(AirflowSolidsSafeguard, ATiltedThinRodDeclaredThickIsRefused) {
    const GridShape shape{32, 32, 32, 1.0f};
    const glm::vec3 d = glm::normalize(glm::vec3(1.0f, 1.0f, 1.0f));
    const glm::vec3 a = glm::normalize(glm::cross(d, glm::vec3(0.0f, 0.0f, 1.0f)));
    const glm::vec3 b = glm::cross(d, a);
    Scene scene;
    scene.shapes.push_back(Shape{.name = "rod", .kind = Kind::box, .centre = {16.0f, 16.0f, 16.0f},
                                 .half = {0.4f, 0.4f, 6.0f}, .to_local = glm::transpose(glm::mat3(a, b, d)),
                                 .min_feature = 4.0f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = parts[0].box_max - parts[0].box_min;  // a mesh tilted in its own frame
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, parts, scene);
    ASSERT_FALSE(r.has_value()) << f.run_line;
    EXPECT_NE(r.error().context.find("rod"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("thin in more than one direction"), std::string::npos) << r.error().context;
}

// A hemispherical canopy shell, its mid-surface 10 dx in radius and 0.5 dx
// thick, declared thick (an imported CAD fairing): its bounds (20.5 x 10.25 x
// 20.5 dx) look thick and it closes no face. Its thin crossings' normals span
// the hemisphere, yet it is a sheet everywhere: wide along both tangents at
// every crossing (2 dx along a tangent leaves the shell by 4/20 = 0.2 dx, inside
// its 0.25 dx half-thickness). Sheeted (the lead, 2026-10-13: the criterion is
// local). Mutation: the global first-normal clause, which refuses it.
TEST(AirflowSolidsSafeguard, ACurvedThinShellDeclaredThickIsSheeted) {
    const GridShape shape{32, 32, 32, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "canopy", .kind = Kind::hemi_shell, .centre = {16.0f, 10.0f, 16.0f},
                                 .half = {0.25f, 0.0f, 0.0f}, .radius = 10.0f, .min_feature = 4.0f});
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(f.classes[0].role, SolidRole::sheet);
    EXPECT_LT(f.classes[0].thickness, 2.0f);
    EXPECT_NE(f.run_line.find("canopy: sheet (measured"), std::string::npos) << f.run_line;
    // Watertight over the cap (the final review's finding 1): grazing
    // crossings near the crown measure up to about 1 dx, twice its thickness.
    const glm::vec3 centre(16.0f, 10.0f, 16.0f);
    const auto mid = [&](glm::vec3 x) { return glm::length(x - centre) - 10.0f; };
    const auto on = [&](glm::vec3 x) { return x.y >= centre.y + 1.0f; };
    EXPECT_EQ(open_crossings(f, shape, mid, on), 0u) << "a line through the cap crosses no closed face";
}

// A bead 1.8 dx across centred on a cell centre: its face centres lie 0.4 dx
// inside (theta 0.1), so it closes no face, and lines through its middle
// cross it thin (tau 1.8 dx) on every axis. Its bounds are conservative, as an
// SDF part's sdf_bounds() may be (4 dx each way), so the classifier calls it a
// wall. Narrow along every tangent: refused, never sheeted. Mutation: no width
// check, and it is sheeted.
TEST(AirflowSolidsSafeguard, ABeadIsRefusedNotSheeted) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "bead", .kind = Kind::sphere, .centre = {8.5f, 8.5f, 8.5f}, .radius = 0.9f,
                                 .min_feature = 4.0f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = glm::vec3(4.0f);  // conservative bounds
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, parts, scene);
    ASSERT_FALSE(r.has_value()) << f.run_line;
    EXPECT_NE(r.error().context.find("thin in more than one direction"), std::string::npos) << r.error().context;
}

// The free-slip interior (design section 3, approved 2026-10-13): a closed
// sample next to an open one of its component takes their mean; a closed
// sample next to that first layer takes the mean of its first-layer
// neighbours; deeper ones keep the rigid velocity. A wall filling x >= 8.
TEST(AirflowSolids, TheFreeSlipFillCopiesTheOpenSideIntoTwoLayers) {
    const GridShape shape{16, 8, 8, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "wall", .kind = Kind::box, .centre = {16.0f, 4.0f, 4.0f},
                                 .half = {8.0f, 20.0f, 20.0f}, .min_feature = 16.0f});
    const SolidField f = built(shape, scene);
    MacGrid g = *make_grid(shape, DomainBc{});
    for_each_sample(shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = 1.0f + 0.1f * x.y; });
    for_each_sample(shape, 1, [&](std::size_t idx, glm::vec3 x) { g.v[idx] = 0.3f + 0.1f * x.z; });
    apply_solid_velocities(g, f, SolidInterior::extrapolate);
    for (uint32_t k = 0; k < 8; ++k) {
        for (uint32_t j = 0; j < 8; ++j) {
            // u: x = 8 is open (theta 0.5), x = 9 the first layer, x = 10 the second.
            EXPECT_EQ(g.u[u_index(shape, 9, j, k)], g.u[u_index(shape, 8, j, k)]) << j << "," << k;
            EXPECT_EQ(g.u[u_index(shape, 10, j, k)], g.u[u_index(shape, 9, j, k)]);
            EXPECT_EQ(g.u[u_index(shape, 11, j, k)], 0.0f) << "deeper: the rigid (still) wall";
        }
        for (uint32_t j = 0; j <= 8; ++j) {
            // v: x = 7.5 is open, x = 8.5 the first layer, x = 9.5 the second.
            EXPECT_EQ(g.v[v_index(shape, 8, j, k)], g.v[v_index(shape, 7, j, k)]);
            EXPECT_EQ(g.v[v_index(shape, 9, j, k)], g.v[v_index(shape, 8, j, k)]);
            EXPECT_EQ(g.v[v_index(shape, 10, j, k)], 0.0f);
        }
    }
}

// Design test 11 (the lead's condition): a wall a hair from a velocity sample.
// The box's face lies about 1e-6 dx beyond the u sample at x = 4, so its link
// to the inside neighbour would cross at f ~ 1e-6; the floor holds it at 0.01,
// so the term is 100 alpha, not 1e6 alpha, and the solve stays finite and
// within the values it blends. Mutation: no clamp.
TEST(AirflowSolids, TheCrossingFractionsFloorHoldsTheStencil) {
    const GridShape shape{8, 8, 8, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "slab", .kind = Kind::box, .centre = {12.000001f, 4.0f, 4.0f},
                                 .half = {8.0f, 20.0f, 20.0f}, .omega = {0.0f, 0.0f, 1.0f}, .min_feature = 8.0f});
    const SolidField f = built(shape, scene);
    const std::size_t s = u_index(shape, 4, 3, 3);
    ASSERT_GE(f.phi[0][s], 0.0f);
    ASSERT_LT(f.phi[0][u_index(shape, 5, 3, 3)], 0.0f);
    ASSERT_NE(f.link[0][s], kNoWallLinks);
    const WallLinks& wl = f.links[f.link[0][s]];
    EXPECT_EQ(wl.inv_f[1], 1.0f / 0.01f) << "+x: the floor";
    EXPECT_EQ(wl.inv_f[0], 0.0f) << "-x: no wall";

    MacGrid g = *make_grid(shape, DomainBc{});
    for_each_sample(shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = 0.5f + 0.05f * x.z; });
    float lo = 0.0f;
    float hi = 0.0f;
    for (const float v : g.u) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    for (const float v : f.wall[0]) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    std::vector<float> rhs(g.u.size());
    diffuse(g, DomainBc{}, 1.0f, 1.0f, 8, rhs, &f);
    EXPECT_TRUE(std::isfinite(g.u[s]));
    EXPECT_GE(g.u[s], lo);
    EXPECT_LE(g.u[s], hi);
}

// A sheet links the samples either side of it: the v samples at x = 15.5 and
// 16.5 either side of a sheet on face plane 16 each cross it halfway along
// their link.
TEST(AirflowSolids, ASheetLinksTheSamplesEitherSide) {
    const GridShape shape{32, 8, 8, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "plate", .kind = Kind::box, .centre = {16.0f, 4.0f, 4.0f},
                                 .half = {0.125f, 20.0f, 20.0f}, .min_feature = 0.25f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = glm::vec3(0.25f, 8.0f, 8.0f);
    SolidField f = *make_solid_field(shape);
    ASSERT_TRUE(build_solids(f, shape, {}, parts, scene.sampler()).has_value());
    const uint32_t left = f.link[1][v_index(shape, 15, 3, 3)];
    const uint32_t right = f.link[1][v_index(shape, 16, 3, 3)];
    ASSERT_NE(left, kNoWallLinks);
    ASSERT_NE(right, kNoWallLinks);
    EXPECT_EQ(f.links[left].inv_f[1], 2.0f) << "+x, at f = 0.5";
    EXPECT_EQ(f.links[right].inv_f[0], 2.0f) << "-x, at f = 0.5";
    EXPECT_EQ(f.link[1][v_index(shape, 13, 3, 3)], kNoWallLinks) << "two cells off: no link";
}

}  // namespace
