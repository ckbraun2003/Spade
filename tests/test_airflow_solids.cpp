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

}  // namespace
