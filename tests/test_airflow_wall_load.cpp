// wall_load() (P3; the solids design, section 6): the pressure force and
// moment on the solids, F = -sum p_c A_c over the cells with an open face,
// against pressure fields written into the grid after set_solids().
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/wall_load.hpp"

namespace {

using namespace spade::physics::airflow;
using airflow_test::Kind;
using airflow_test::Scene;
using airflow_test::Shape;

constexpr float kPi = 3.14159265358979f;

struct Rig {
    GridShape shape;
    MacGrid g;
    FluidScratch s;
};

// A sphere of radius R = 6 dx centred on the cell corner at the middle of a
// 32^3 box, set as solids, the pressure left 0.
[[nodiscard]] Rig sphere_rig(float dx) {
    const GridShape shape{32, 32, 32, dx};
    const DomainBc bc;
    MacGrid g = *make_grid(shape, bc);
    FluidScratch s = *make_fluid_scratch(g, bc, FluidConfig{.h = 0.001f});
    Scene scene;
    scene.shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = glm::vec3(16.0f * dx),
                                 .radius = 6.0f * dx, .min_feature = 12.0f * dx});
    EXPECT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value()) << s.solids.run_line;
    return {shape, std::move(g), std::move(s)};
}

// p(x) in every fluid cell, NaN in every solid one (never read).
template <class P>
void write_pressure(Rig& r, P p) {
    const GridShape& sh = r.shape;
    for (uint32_t k = 0; k < sh.nz; ++k) {
        for (uint32_t j = 0; j < sh.ny; ++j) {
            for (uint32_t i = 0; i < sh.nx; ++i) {
                const std::size_t c = cell_index(sh, i, j, k);
                const glm::vec3 x((static_cast<float>(i) + 0.5f) * sh.dx, (static_cast<float>(j) + 0.5f) * sh.dx,
                                  (static_cast<float>(k) + 0.5f) * sh.dx);
                r.g.p[c] = r.s.regions.solid[c] != 0u ? std::numeric_limits<float>::quiet_NaN() : p(x);
            }
        }
    }
}

// d'Alembert's zero for a uniform pressure: the cut cells' area vectors close
// on the body, so only rounding is left (measured 6.0e-8 of p times the
// sphere's area; band 1e-5). Mutation: the solid cells' pressure read (NaN),
// which poisons the sum.
TEST(AirflowWallLoad, AUniformPressureGivesNoForce) {
    const float dx = 0.01f;
    Rig r = sphere_rig(dx);
    write_pressure(r, [](glm::vec3) { return 1000.0f; });
    const WallLoad load = wall_load(r.g, r.s.solids, r.s.regions, glm::vec3(16.0f * dx));
    const float scale = 1000.0f * 4.0f * kPi * 36.0f * dx * dx;  // p times the sphere's area
    std::printf("uniform 1000 Pa: |F| / (p A_sphere) = %.3e, |M| = %.3e N m\n",
                static_cast<double>(glm::length(load.force) / scale), static_cast<double>(glm::length(load.moment)));
    EXPECT_LE(glm::length(load.force), 1.0e-5f * scale);
    EXPECT_TRUE(std::isfinite(load.moment.x) && std::isfinite(load.moment.y) && std::isfinite(load.moment.z));
}

// Archimedes: a pressure falling upward, p = G (y_c - y), pushes the sphere
// up with G V, V = (4/3) pi R^3, through its centre. The apertures stand for
// the wall, so the discrete volume differs from V by the ramp's chord error
// at a curved wall, (dx/R)^2 = 0.028 at R/dx 6: the band, 0.03. Measured
// 2026-10-22 (MSVC 14.43 release): F_y / (G V) = 1.00688, F_x = F_z = 0;
// about the centre |M| / (G V R) = 4.4e-8, and about a point o the moment
// is (c - o) x F to 1.2e-7 (rounding; band 1e-4). Mutations: the sign
// flipped (the sphere pushed down); the moment taken about the grid's origin.
TEST(AirflowWallLoad, APressureFallingUpwardBuoysTheSphereThroughItsCentre) {
    const float dx = 0.01f;
    const float G = 1000.0f;  // Pa/m
    Rig r = sphere_rig(dx);
    const glm::vec3 c(16.0f * dx);
    write_pressure(r, [&](glm::vec3 x) { return G * (c.y - x.y); });
    const float R = 6.0f * dx;
    const float GV = G * (4.0f / 3.0f) * kPi * R * R * R;
    const WallLoad at_c = wall_load(r.g, r.s.solids, r.s.regions, c);
    const glm::vec3 o = c + glm::vec3(0.05f, -0.02f, 0.03f);
    const WallLoad at_o = wall_load(r.g, r.s.solids, r.s.regions, o);
    const glm::vec3 expect_o = glm::cross(c - o, at_c.force) + at_c.moment;
    std::printf("buoyancy: F / (G V) = (%.5f, %.5f, %.5f); |M_c| / (G V R) = %.3e; |M_o - (c - o) x F - M_c| / "
                "(G V R) = %.3e\n",
                static_cast<double>(at_c.force.x / GV), static_cast<double>(at_c.force.y / GV),
                static_cast<double>(at_c.force.z / GV), static_cast<double>(glm::length(at_c.moment) / (GV * R)),
                static_cast<double>(glm::length(at_o.moment - expect_o) / (GV * R)));
    EXPECT_NEAR(at_c.force.y / GV, 1.0f, 0.03f);
    EXPECT_LE(std::fabs(at_c.force.x), 1.0e-4f * GV);
    EXPECT_LE(std::fabs(at_c.force.z), 1.0e-4f * GV);
    EXPECT_LE(glm::length(at_c.moment), 1.0e-4f * GV * R);
    EXPECT_LE(glm::length(at_o.moment - expect_o), 1.0e-4f * GV * R);
}

// A sheet across a periodic channel, the pressure p_l on its -x side and p_r
// on its +x side: F = (p_l - p_r) A along +x, A its 8 x 8 cell section, to
// rounding. The sheet path is the cut-cell path (each side's closed face).
TEST(AirflowWallLoad, ASheetCarriesThePressureJumpAcrossIt) {
    const float dx = 0.01f;
    const GridShape shape{32, 8, 8, dx};
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid(shape, bc);
    FluidScratch s = *make_fluid_scratch(g, bc, FluidConfig{.h = 0.001f});
    Scene scene;
    scene.shapes.push_back(Shape{.name = "sheet", .kind = Kind::box, .centre = {16.0f * dx, 4.0f * dx, 4.0f * dx},
                                 .half = {0.125f * dx, 20.0f * dx, 20.0f * dx}, .min_feature = 0.25f * dx});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = glm::vec3(0.25f * dx, 8.0f * dx, 8.0f * dx);
    ASSERT_TRUE(set_solids(s, g, bc, parts, scene.sampler()).has_value()) << s.solids.run_line;
    ASSERT_EQ(s.solids.classes[0].role, SolidRole::sheet);
    const float p_l = 2000.0f;
    const float p_r = 500.0f;
    for (uint32_t k = 0; k < 8; ++k) {
        for (uint32_t j = 0; j < 8; ++j) {
            for (uint32_t i = 0; i < 32; ++i) g.p[cell_index(shape, i, j, k)] = i < 16u ? p_l : p_r;
        }
    }
    const WallLoad load = wall_load(g, s.solids, s.regions, glm::vec3(16.0f * dx, 4.0f * dx, 4.0f * dx));
    const float expect = (p_l - p_r) * 64.0f * dx * dx;
    EXPECT_NEAR(load.force.x / expect, 1.0f, 1.0e-6f);
    EXPECT_EQ(load.force.y, 0.0f);
    EXPECT_EQ(load.force.z, 0.0f);
    EXPECT_LE(glm::length(load.moment), 1.0e-6f * expect * 8.0f * dx);
}

// No solids: zero, whatever the pressure.
TEST(AirflowWallLoad, NoSolidsIsZero) {
    const GridShape shape{8, 8, 8, 0.01f};
    MacGrid g = *make_grid(shape, DomainBc{});
    std::fill(g.p.begin(), g.p.end(), 1000.0f);
    const FluidScratch s = *make_fluid_scratch(g, DomainBc{}, FluidConfig{.h = 0.001f});
    const WallLoad load = wall_load(g, s.solids, s.regions, glm::vec3(0.04f));
    EXPECT_EQ(load.force, glm::vec3(0.0f));
    EXPECT_EQ(load.moment, glm::vec3(0.0f));
}

}  // namespace
