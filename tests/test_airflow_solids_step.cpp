// The fluid step with solids (the solids design; P2 Tasks 6-9): S3's sheet
// across a channel, S1's P2 checks, S2's Couette flow, rebuilds, plan A's
// bits without solids, and the posed drone. Bands are Test/Docs'
// (test-docs/2026-10-13-solids-tolerances.md) or measured and pinned (TD-2),
// as each says.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "airflow_record.hpp"
#include "airflow_solid_shapes.hpp"
#include "reference_airframe.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/wall_load.hpp"
#include "state/layout.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/part_pose.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/mesh.hpp"
#include "world/sdf.hpp"

namespace {

using namespace spade::physics::airflow;
using airflow_test::Kind;
using airflow_test::Scene;
using airflow_test::Shape;

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] bool same_bits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

[[nodiscard]] DomainBc all_periodic() {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    return bc;
}

// S3's channel: L = 32 dx = 1 m, an 8 x 8 section, every axis periodic, and a
// sheet 0.25 dx thick across the section with its mid-plane on face plane
// `plane`.
// The compiler and the build a record was measured on (V1's form).
using airflow_test::build_line;

// Whether this is a debug build: TD-5's "debug: runs in release" skips name it.
[[nodiscard]] bool debug_build() {
#if defined(NDEBUG)
    return false;
#else
    return true;
#endif
}

[[nodiscard]] std::vector<SolidPart> s3_sheet(Scene& scene, uint32_t plane, float dx) {
    const float x = static_cast<float>(plane) * dx;
    scene.shapes.push_back(Shape{.name = "sheet", .kind = Kind::box, .centre = {x, 4.0f * dx, 4.0f * dx},
                                 .half = {0.125f * dx, 20.0f * dx, 20.0f * dx}, .min_feature = 0.25f * dx});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = glm::vec3(0.25f * dx, 8.0f * dx, 8.0f * dx);
    return parts;
}

// Design test 3 / S3 (Test/Docs' tolerances): one projection of u = U blocked
// by the sheet, from a cold start, n_v = 32 CG iterations, on an even and an
// odd plane. Exact in 1D: u = 0 everywhere after it, and p rises along x by
// rho U / h per metre, dropping by rho U (L - dx) / h across the sheet
// between the cell centres either side.
TEST(AirflowSolidsStep, S3ASheetAcrossAChannelStopsItsFlow) {
    const float dx = 1.0f / 32.0f;
    const GridShape shape{32, 8, 8, dx};
    const DomainBc bc = all_periodic();
    const float rho = 1.225f;
    const float h = 0.001f;
    const float U = 1.0f;
    for (const uint32_t plane : {16u, 17u}) {
        MacGrid g = *make_grid(shape, bc);
        std::fill(g.u.begin(), g.u.end(), U);
        const FluidConfig cfg{.rho = rho, .nu = 0.0f, .h = h, .n_v = 32};
        FluidScratch s = *make_fluid_scratch(g, bc, cfg);
        Scene scene;
        const std::vector<SolidPart> parts = s3_sheet(scene, plane, dx);
        ASSERT_TRUE(set_solids(s, g, bc, parts, scene.sampler()).has_value());
        apply_solid_velocities(g, s.solids, SolidInterior::rigid);
        ASSERT_EQ(check_regions(s.regions, shape, bc, s.solids).region, kNoRegion);
        const ProjectionReport rep = project(g, bc, rho, h, cfg.n_v, s.projection, &s.solids, &s.regions);

        float u_max = 0.0f;
        for (const std::vector<float>* a : {&g.u, &g.v, &g.w}) {
            for (const float v : *a) u_max = std::max(u_max, std::fabs(v));
        }
        uint32_t off_wall = 0;  // closed faces not at the wall velocity
        for (uint32_t k = 0; k < 8; ++k) {
            for (uint32_t j = 0; j < 8; ++j) {
                off_wall += g.u[u_index(shape, plane, j, k)] == 0.0f ? 0u : 1u;
            }
        }
        const double step = static_cast<double>(g.p[cell_index(shape, plane - 1u, 3, 3)]) -
                            static_cast<double>(g.p[cell_index(shape, plane, 3, 3)]);
        const double expected = static_cast<double>(rho) * U * (1.0 - dx) / h;
        // P3's force on the sheet over rho U L A / h (L = 1 m, A = (8 dx)^2):
        // each side's pressure is read at the next cell centre, so the scheme
        // gives 1 - dx/L.
        const glm::vec3 on_sheet(static_cast<float>(plane) * dx, 0.125f, 0.125f);
        const double force = static_cast<double>(wall_load(g, s.solids, s.regions, on_sheet).force.x) * h /
                             (static_cast<double>(rho) * U * 1.0 * 0.0625);
        std::printf("=== validation record: S3 sheet across a channel, plane %u (%s) ===\n"
                    "path: CPU reference, %s\n"
                    "config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho %.3f, U %.1f, h %.4f, one "
                    "projection, n_v 32 CG iterations from a cold start; %s\n"
                    "measure: closed faces off their wall velocity = %u\n"
                    "reference: 0 (a closed face carries its wall's velocity)\n"
                    "tolerance: 0, exact (the solids design, section 3)\n"
                    "verdict: %s\n"
                    "measure: max |u| / U after the projection = %.3e\n"
                    "reference: 0 (exact in 1D)\n"
                    "tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                    "verdict: %s\n"
                    "measure: the pressure step dp h / (rho U (L - dx)) = %.6f\n"
                    "reference: 1 (exact: the stencil is exact for a linear field)\n"
                    "tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                    "verdict: %s\n"
                    "measure: P3's force on the sheet F h / (rho U L A) from wall_load() = %.6f\n"
                    "reference: 1 (exact in 1D; the scheme reads each side half a cell off: 1 - dx/L)\n"
                    "tolerance: 1 +/- 0.033 (test-docs/2026-10-13-solids-tolerances.md, S3)\n"
                    "verdict: %s\n"
                    "measure: the solve's final relative residual = %.3e\n"
                    "reference: a converged solve\n"
                    "tolerance: none (report only)\n"
                    "verdict: report only\n",
                    plane, plane % 2u == 0u ? "even" : "odd", build_line().c_str(), static_cast<double>(rho),
                    static_cast<double>(U), static_cast<double>(h), s.solids.run_line.c_str(),
                    off_wall, off_wall == 0u ? "within" : "MISS",
                    static_cast<double>(u_max / U), u_max / U <= 1.0e-3f ? "within" : "MISS", step / expected,
                    std::fabs(step / expected - 1.0) <= 1.0e-3 ? "within" : "MISS", force,
                    std::fabs(force - 1.0) <= 0.033 ? "within" : "MISS", static_cast<double>(rep.residual));
        EXPECT_EQ(off_wall, 0u) << plane;
        EXPECT_LE(u_max / U, 1.0e-3f) << plane;
        EXPECT_NEAR(step / expected, 1.0, 1.0e-3) << plane;
        EXPECT_NEAR(force, 1.0, 0.033) << plane;
        EXPECT_NE(s.solids.run_line.find("sheet: sheet (7.8 mm < 2 dx = 62.5 mm)"), std::string::npos)
            << s.solids.run_line;
    }
}

// With n_v = 1 on the odd plane the solve cannot converge: the step reports
// its relative residual and announces it (L6), and does not refuse.
TEST(AirflowSolidsStep, AnUnconvergedSolveIsAnnounced) {
    const float dx = 1.0f / 32.0f;
    const GridShape shape{32, 8, 8, dx};
    const DomainBc bc = all_periodic();
    MacGrid g = *make_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), 1.0f);
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.001f, .n_v = 1};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    ASSERT_TRUE(set_solids(s, g, bc, s3_sheet(scene, 17, dx), scene.sampler()).has_value());
    const FluidHealth health = fluid_step(g, bc, cfg, s);
    EXPECT_EQ(health.fault, FluidFault::none);
    EXPECT_GT(health.pressure_residual, kResidualAnnounce);
    EXPECT_TRUE(health.unconverged);
    EXPECT_NE(describe(health, shape).find("announced"), std::string::npos) << describe(health, shape);
}

[[nodiscard]] uint64_t digest(const MacGrid& g) {
    uint64_t h = 1469598103934665603ull;
    for (const std::vector<float>* v : {&g.u, &g.v, &g.w, &g.p}) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(v->data());
        for (std::size_t i = 0; i < v->size() * sizeof(float); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ull;
        }
    }
    return h;
}

// Design test 6: with no parts -- never set, set empty, or set and then
// cleared -- four steps of a walled box with a moving lid are plan A's bits.
TEST(AirflowSolidsStep, NoPartsIsPlanABitForBit) {
    const GridShape shape{12, 10, 8, 0.1f};
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.01f, .h = 0.01f, .n_v = 2};
    const auto run = [&](int how) {
        MacGrid g = *make_grid(shape, bc);
        Lcg rng;
        for (std::vector<float>* v : {&g.u, &g.v, &g.w}) {
            for (float& x : *v) x = 0.1f * rng.next();
        }
        FluidScratch s = *make_fluid_scratch(g, bc, cfg);
        Scene scene;
        if (how >= 1) {
            EXPECT_TRUE(set_solids(s, g, bc, {}, scene.sampler()).has_value());
        }
        if (how == 2) {
            scene.shapes.push_back(Shape{.name = "ball", .kind = Kind::sphere, .centre = {0.6f, 0.5f, 0.4f},
                                         .radius = 0.2f, .min_feature = 0.4f});
            const std::vector<SolidPart> parts = scene.parts();
            EXPECT_TRUE(set_solids(s, g, bc, parts, scene.sampler()).has_value());
            EXPECT_TRUE(s.solids.any);
            EXPECT_TRUE(set_solids(s, g, bc, {}, scene.sampler()).has_value());
        }
        if (how == 3) {  // stepped WITH the ball first, so every solids path ran (the final review)
            scene.shapes.push_back(Shape{.name = "ball", .kind = Kind::sphere, .centre = {0.6f, 0.5f, 0.4f},
                                         .radius = 0.2f, .min_feature = 0.4f});
            const std::vector<SolidPart> parts = scene.parts();
            EXPECT_TRUE(set_solids(s, g, bc, parts, scene.sampler()).has_value());
            MacGrid other = g;
            for (int step = 0; step < 2; ++step) EXPECT_EQ(fluid_step(other, bc, cfg, s).fault, FluidFault::none);
            EXPECT_TRUE(set_solids(s, g, bc, {}, scene.sampler()).has_value());
        }
        for (int step = 0; step < 4; ++step) EXPECT_EQ(fluid_step(g, bc, cfg, s).fault, FluidFault::none);
        return digest(g);
    };
    const uint64_t plain = run(0);
    EXPECT_EQ(run(1), plain);
    EXPECT_EQ(run(2), plain);
    EXPECT_EQ(run(3), plain);
}

// With viscosity, a domain face a solid covers ends the step at the solid's
// velocity, not the boundary's: an open inflow face under a still box (P2
// Task 11, the final review's finding 3). Mutation: no re-application after
// the boundaries, which leaves the inflow's 1 m/s there.
TEST(AirflowSolidsStep, ADomainFaceUnderASolidKeepsTheSolidsVelocity) {
    const GridShape shape{16, 8, 8, 0.1f};
    DomainBc bc;
    bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
    const FluidConfig cfg{.rho = 1.225f, .nu = 1.0e-3f, .h = 0.005f, .n_v = kSolidsCgIterations};
    MacGrid g = *make_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), 1.0f);
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    scene.shapes.push_back(Shape{.name = "block", .kind = Kind::box, .centre = {0.0f, 0.4f, 0.4f},
                                 .half = {0.25f, 0.2f, 0.2f}, .min_feature = 0.4f});
    ASSERT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value()) << s.solids.run_line;
    ASSERT_EQ(fluid_step(g, bc, cfg, s).fault, FluidFault::none);
    uint32_t covered = 0;
    float worst = 0.0f;
    for (uint32_t k = 0; k < 8; ++k) {
        for (uint32_t j = 0; j < 8; ++j) {
            const std::size_t idx = u_index(shape, 0, j, k);
            if (!closed(s.solids.theta[0][idx])) continue;
            ++covered;
            worst = std::max(worst, std::fabs(g.u[idx]));
        }
    }
    EXPECT_GT(covered, 0u);
    EXPECT_EQ(worst, 0.0f) << "a closed inflow face holds the still block's 0, not the inflow";
}

// Solids without their regions are not projected as plan A (P2 Task 11, the
// final review's finding 4): the grid is left untouched and the residual is
// infinite, which a step announces.
TEST(AirflowSolidsStep, ProjectingSolidsWithoutRegionsIsRefused) {
    const float dx = 1.0f / 32.0f;
    const GridShape shape{32, 8, 8, dx};
    const DomainBc bc = all_periodic();
    MacGrid g = *make_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), 1.0f);
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.001f, .n_v = kSolidsCgIterations};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    ASSERT_TRUE(set_solids(s, g, bc, s3_sheet(scene, 17, dx), scene.sampler()).has_value());
    const uint64_t before = digest(g);
    const ProjectionReport rep = project(g, bc, cfg.rho, cfg.h, cfg.n_v, s.projection, &s.solids, nullptr);
    EXPECT_TRUE(std::isinf(rep.residual));
    EXPECT_EQ(digest(g), before);
}

// Review focus 1: a rebuild at a new pose leaves nothing of the last one and
// grows no array: the scratch rebuilt from pose A to pose B equals a fresh
// scratch built at B, bit for bit, with the same capacities as after A.
TEST(AirflowSolidsStep, ARebuildLeavesNothingOfTheLastPose) {
    const GridShape shape{24, 24, 24, 0.05f};
    const DomainBc bc;
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.002f, .n_v = 4};
    MacGrid g = *make_grid(shape, bc);
    const auto pose = [](float x) {
        Scene scene;
        scene.shapes.push_back(Shape{.name = "body", .kind = Kind::box, .centre = {x, 0.6f, 0.6f},
                                     .half = {0.15f, 0.1f, 0.2f}, .omega = {0.0f, 2.0f, 0.0f}, .min_feature = 0.2f});
        return scene;
    };
    FluidScratch moved = *make_fluid_scratch(g, bc, cfg);
    const Scene a = pose(0.45f);
    ASSERT_TRUE(set_solids(moved, g, bc, a.parts(), a.sampler()).has_value());
    const std::size_t cap_theta = moved.solids.theta[0].capacity();
    const std::size_t cap_label = moved.regions.label.capacity();
    const std::size_t cap_x = moved.projection.pcg.x.capacity();
    const Scene b = pose(0.75f);
    ASSERT_TRUE(set_solids(moved, g, bc, b.parts(), b.sampler()).has_value());
    FluidScratch fresh = *make_fluid_scratch(g, bc, cfg);
    ASSERT_TRUE(set_solids(fresh, g, bc, b.parts(), b.sampler()).has_value());
    for (uint32_t c = 0; c < 3; ++c) {
        EXPECT_TRUE(same_bits(moved.solids.theta[c], fresh.solids.theta[c])) << c;
        EXPECT_TRUE(same_bits(moved.solids.wall[c], fresh.solids.wall[c])) << c;
        EXPECT_TRUE(same_bits(moved.solids.phi[c], fresh.solids.phi[c])) << c;
        EXPECT_EQ(moved.solids.link[c], fresh.solids.link[c]) << c;
        // The rest of the field and the regions (the final review's F6).
        EXPECT_EQ(moved.solids.fill1[c], fresh.solids.fill1[c]) << c;
        EXPECT_EQ(moved.solids.fill2[c], fresh.solids.fill2[c]) << c;
        EXPECT_EQ(moved.solids.layer[c], fresh.solids.layer[c]) << c;
        EXPECT_EQ(moved.solids.range[c].lo, fresh.solids.range[c].lo) << c;
        EXPECT_EQ(moved.solids.range[c].hi, fresh.solids.range[c].hi) << c;
        EXPECT_EQ(moved.solids.range[c].empty, fresh.solids.range[c].empty) << c;
    }
    ASSERT_EQ(moved.solids.links.size(), fresh.solids.links.size());
    EXPECT_TRUE(moved.solids.links.empty() ||
                std::memcmp(moved.solids.links.data(), fresh.solids.links.data(),
                            moved.solids.links.size() * sizeof(WallLinks)) == 0);
    EXPECT_EQ(moved.solids.run_line, fresh.solids.run_line);
    EXPECT_EQ(moved.regions.label, fresh.regions.label);
    EXPECT_EQ(moved.regions.solid, fresh.regions.solid);
    EXPECT_EQ(moved.regions.count, fresh.regions.count);
    EXPECT_EQ(moved.regions.cells, fresh.regions.cells);
    EXPECT_EQ(moved.regions.first, fresh.regions.first);
    for (std::size_t l = 0; l < fresh.projection.mg.levels.size(); ++l) {
        for (uint32_t c = 0; c < 3; ++c) {
            EXPECT_TRUE(same_bits(moved.projection.mg.levels[l].w[c], fresh.projection.mg.levels[l].w[c])) << l;
        }
    }
    EXPECT_EQ(moved.solids.theta[0].capacity(), cap_theta);
    EXPECT_EQ(moved.regions.label.capacity(), cap_label);
    EXPECT_EQ(moved.projection.pcg.x.capacity(), cap_x);
}

// Review focus 5: a closed box shell (a sheet, by its 1 mm wall) seals a
// pocket of cells in an open domain. The pocket is its own closed region and
// keeps its gauge -- its mean pressure 0 -- step after step, while the outside
// stays pinned by the outlets.
TEST(AirflowSolidsStep, ASealedPocketKeepsItsGaugeOverManySteps) {
    const GridShape shape{24, 24, 24, 0.05f};
    const DomainBc bc;
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.002f, .n_v = 8};
    MacGrid g = *make_grid(shape, bc);
    Lcg rng;
    for (std::vector<float>* v : {&g.u, &g.v, &g.w}) {
        for (float& x : *v) x = rng.next();
    }
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    scene.shapes.push_back(Shape{.name = "canopy", .kind = Kind::box_shell, .centre = {0.6f, 0.6f, 0.6f},
                                 .half = {0.22f, 0.22f, 0.22f}, .radius = 0.001f, .min_feature = 0.001f});
    ASSERT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value());
    ASSERT_EQ(s.regions.count, 2u) << "outside and the pocket";
    for (int step = 0; step < 40; ++step) {
        const FluidHealth health = fluid_step(g, bc, cfg, s);
        ASSERT_EQ(health.fault, FluidFault::none) << describe(health, shape);
        uint32_t pocket = kNoRegion;
        for (uint32_t r = 0; r < s.regions.count; ++r) {
            if (s.regions.closed[r] != 0u) pocket = r;
        }
        ASSERT_NE(pocket, kNoRegion);
        double sum = 0.0;
        float p_max = 0.0f;
        for (std::size_t c = 0; c < g.p.size(); ++c) {
            if (s.regions.label[c] != pocket) continue;
            sum += static_cast<double>(g.p[c]);
            p_max = std::max(p_max, std::fabs(g.p[c]));
        }
        EXPECT_LE(std::fabs(sum / s.regions.cells[pocket]), 1.0e-5 * std::max(p_max, 1.0f)) << "step " << step;
    }
    EXPECT_EQ(describe_regions(s.regions), "fluid regions: 2 (1 with an outlet, 1 closed)");
}

// The free-slip interior is the inviscid lean mode: with nu > 0 it is refused.
TEST(AirflowSolidsStep, TheFreeSlipInteriorIsRefusedWithViscosity) {
    const GridShape shape{8, 8, 8, 0.1f};
    const MacGrid g = *make_grid(shape, DomainBc{});
    const FluidConfig cfg{.rho = 1.225f, .nu = 1.0e-5f, .h = 0.001f, .n_v = 2, .interior = SolidInterior::extrapolate};
    const auto s = make_fluid_scratch(g, DomainBc{}, cfg);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("free-slip"), std::string::npos) << s.error().context;
}

// Design test 1, its P2 checks (S1's configuration, Test/Docs): a sphere
// R = 6 dx in a uniform stream, free slip, inviscid, a box of side 8R with the
// sphere centred on a cell corner, inflow on -x and the other faces open,
// U h / dx = 0.1, n_v = 16. After step 1 (from a cold start):
//   - every closed face carries the wall velocity, 0, exactly;
//   - the solve's final relative residual is at most 1e-4 (Test/Docs: an
//     unconverged solve must show as a miss, not bias C_M; printed);
//   - the aperture divergence left is at most 2e-4 of the right side's, the
//     residual's own measure carried through the velocity update.
// C_p at step 3 and the added mass at step 1 read P3 and join it.
TEST(AirflowSolidsStep, S1TheSpheresStepOneConvergesAndHoldsItsWall) {
    const float dx = 0.01f;
    const GridShape shape{48, 48, 48, dx};
    DomainBc bc;
    bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
    const float U = 1.0f;
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.1f * dx / U, .n_v = 16,
                          .interior = SolidInterior::extrapolate};
    MacGrid g = *make_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), U);
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    scene.shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = {0.24f, 0.24f, 0.24f},
                                 .radius = 6.0f * dx, .min_feature = 12.0f * dx});
    ASSERT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value());
    const FluidHealth health = fluid_step(g, bc, cfg, s);
    ASSERT_EQ(health.fault, FluidFault::none) << describe(health, shape);

    uint32_t off_wall = 0;  // closed faces not at the wall velocity
    const std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < vel[c]->size(); ++i) {
            if (s.solids.theta[c][i] == 0.0f) off_wall += (*vel[c])[i] == 0.0f ? 0u : 1u;
        }
    }
    const float b_coef = cfg.rho * dx * dx / cfg.h;
    float div_max = 0.0f;
    float b_max = 0.0f;
    const auto flux = [&](uint32_t c, std::size_t idx) {
        const float t = s.solids.theta[c][idx];
        return t * (*vel[c])[idx] + (1.0f - t) * s.solids.wall[c][idx];
    };
    for (uint32_t k = 0; k < 48; ++k) {
        for (uint32_t j = 0; j < 48; ++j) {
            for (uint32_t i = 0; i < 48; ++i) {
                const std::size_t c = cell_index(shape, i, j, k);
                if (s.regions.label[c] == kNoRegion) continue;
                const float div = ((flux(0, u_index(shape, i + 1u, j, k)) - flux(0, u_index(shape, i, j, k))) +
                                   (flux(1, v_index(shape, i, j + 1u, k)) - flux(1, v_index(shape, i, j, k))) +
                                   (flux(2, w_index(shape, i, j, k + 1u)) - flux(2, w_index(shape, i, j, k)))) /
                                  dx;
                div_max = std::max(div_max, std::fabs(div));
                b_max = std::max(b_max, std::fabs(s.projection.pcg.b[c]));
            }
        }
    }
    const float div_ratio = div_max * b_coef / b_max;
    std::printf("=== validation record: S1 sphere in a uniform stream, R/dx 6, P2 checks ===\n"
                "path: CPU reference, %s\n"
                "config: 48^3 at dx 1 cm, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on "
                "-x, other faces open, U h/dx 0.1, n_v 16 CG iterations, step 1 from a cold start; %s\n"
                "measure: closed faces off their wall velocity = %u\n"
                "reference: 0 (a closed face carries its wall's velocity)\n"
                "tolerance: 0, exact (the solids design, section 3)\n"
                "verdict: %s\n"
                "measure: the solve's final relative residual = %.3e\n"
                "reference: a converged solve\n"
                "tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)\n"
                "verdict: %s\n"
                "measure: the aperture divergence left, over the right side's = %.3e\n"
                "reference: 0 (a solved projection leaves rounding only)\n"
                "tolerance: <= 2e-4 (twice the residual's band: the update's rounding; P2 plan, Task 7, fixed "
                "before the first run)\n"
                "verdict: %s\n",
                build_line().c_str(), s.solids.run_line.c_str(), off_wall, off_wall == 0u ? "within" : "MISS",
                static_cast<double>(health.pressure_residual),
                health.pressure_residual <= 1.0e-4f ? "within" : "MISS", static_cast<double>(div_ratio),
                div_ratio <= 2.0e-4f ? "within" : "MISS");
    EXPECT_EQ(off_wall, 0u);
    EXPECT_LE(health.pressure_residual, 1.0e-4f);
    EXPECT_LE(div_ratio, 2.0e-4f);
}

// V1's opt-in (test_airflow_validation.cpp): set SPADE_AIRFLOW_VALIDATION.
[[nodiscard]] bool validation_enabled() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_AIRFLOW_VALIDATION") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// --- S1's P3 checks (design test 1; Test/Docs' tolerances, S1) ------------

struct S1Result {
    double c_m = 0.0;           // F_x h / (rho V U) from wall_load() at step 1
    double cp_err = 0.0;        // max |C_p - C_p,pot| one cell out along the normal (step 3)
    double cp_err_fixed = 0.0;  // the same at r = 7R/6, the order check's radius (step 3)
    double drag = 0.0;          // |F_x| / (1/2 rho U^2 pi R^2) at step 3
    double side = 0.0;          // max(|F_y|, |F_z|) over the same, at step 3
    float residual = 0.0f;      // the worse of steps 1 and 3's final relative residuals
    uint32_t samples = 0;
    std::string run_line;
};

// The cell-centre pressure, trilinear at x (the grid's frame), each corner
// read as the published volume holds it: a fluid cell its own pressure, a
// solid one the mean of its fluid face-neighbours (Publish's rule in solid
// cells, P6), the weights renormalised over the corners that have a value.
// RND-8's sample, so the record reads what the view would.
[[nodiscard]] double published_pressure_at(const MacGrid& g, const FluidRegions& r, glm::dvec3 x) {
    const GridShape& s = g.shape;
    const auto value = [&](int i, int j, int k, double& out) {
        const std::size_t c = cell_index(s, static_cast<uint32_t>(i), static_cast<uint32_t>(j), static_cast<uint32_t>(k));
        if (r.solid[c] == 0u) {
            out = g.p[c];
            return true;
        }
        double sum = 0.0;
        int n = 0;
        const int d[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
        for (const auto& o : d) {
            const std::size_t q = cell_index(s, static_cast<uint32_t>(i + o[0]), static_cast<uint32_t>(j + o[1]),
                                             static_cast<uint32_t>(k + o[2]));
            if (r.solid[q] == 0u) {
                sum += g.p[q];
                ++n;
            }
        }
        if (n == 0) return false;
        out = sum / n;
        return true;
    };
    const glm::dvec3 q = x / static_cast<double>(s.dx) - 0.5;
    const glm::ivec3 b(static_cast<int>(std::floor(q.x)), static_cast<int>(std::floor(q.y)),
                       static_cast<int>(std::floor(q.z)));
    const glm::dvec3 t = q - glm::dvec3(b);
    double p = 0.0;
    double weight = 0.0;
    for (int n = 0; n < 8; ++n) {
        const int ox = n & 1;
        const int oy = (n >> 1) & 1;
        const int oz = (n >> 2) & 1;
        const double w = (ox != 0 ? t.x : 1.0 - t.x) * (oy != 0 ? t.y : 1.0 - t.y) * (oz != 0 ? t.z : 1.0 - t.z);
        double v = 0.0;
        if (value(b.x + ox, b.y + oy, b.z + oz, v)) {
            p += w * v;
            weight += w;
        }
    }
    return p / weight;
}

// Potential flow's C_p at radius r and polar angle theta from +x (Batchelor
// 1967, ch. 6), in double.
[[nodiscard]] double cp_potential(double R, double r, double theta) {
    const double a = (R / r) * (R / r) * (R / r);
    const double ur = std::cos(theta) * (1.0 - a);
    const double ut = -std::sin(theta) * (1.0 + 0.5 * a);
    return 1.0 - (ur * ur + ut * ut);
}

// S1 at R/dx = n: R = 0.06 m, a box of side 8R, the sphere centred on its
// middle cell corner, inflow U = 1 m/s on -x and the other faces open, free
// slip, inviscid, U h / dx = 0.1, n_v 16. C_M from step 1; C_p, drag and side
// forces from step 3; samples every 15 degrees in theta on four meridians.
[[nodiscard]] S1Result run_s1(uint32_t n) {
    const double R = 0.06;
    const float dx = static_cast<float>(R / n);
    const uint32_t cells = 8u * n;
    const GridShape shape{cells, cells, cells, dx};
    DomainBc bc;
    bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
    const float U = 1.0f;
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.1f * dx / U, .n_v = 16,
                          .interior = SolidInterior::extrapolate};
    MacGrid g = *make_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), U);
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    const float mid = 4.0f * static_cast<float>(n) * dx;
    scene.shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = glm::vec3(mid),
                                 .radius = static_cast<float>(R), .min_feature = static_cast<float>(2.0 * R)});
    S1Result out;
    EXPECT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value());
    out.run_line = s.solids.run_line;
    const double V = 4.0 / 3.0 * 3.141592653589793 * R * R * R;
    const double q_area = 0.5 * cfg.rho * U * U * 3.141592653589793 * R * R;
    const FluidHealth h1 = fluid_step(g, bc, cfg, s);
    EXPECT_EQ(h1.fault, FluidFault::none) << describe(h1, shape);
    out.c_m = static_cast<double>(wall_load(g, s.solids, s.regions, glm::vec3(mid)).force.x) * cfg.h /
              (cfg.rho * V * U);
    out.residual = h1.pressure_residual;
    FluidHealth h3{};
    for (int step = 2; step <= 3; ++step) h3 = fluid_step(g, bc, cfg, s);
    EXPECT_EQ(h3.fault, FluidFault::none) << describe(h3, shape);
    out.residual = std::max(out.residual, h3.pressure_residual);
    const WallLoad load = wall_load(g, s.solids, s.regions, glm::vec3(mid));
    out.drag = std::fabs(static_cast<double>(load.force.x)) / q_area;
    out.side = std::max(std::fabs(static_cast<double>(load.force.y)), std::fabs(static_cast<double>(load.force.z))) /
               q_area;
    const double half_rho_u2 = 0.5 * cfg.rho * U * U;
    const glm::dvec3 c(mid);
    for (int m = 0; m < 4; ++m) {
        const double phi = 0.5 * 3.141592653589793 * m;
        for (int a = 0; a <= 12; ++a) {
            const double theta = 3.141592653589793 * a / 12.0;
            const glm::dvec3 nrm(std::cos(theta), std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi));
            const double r_out = R + static_cast<double>(dx);
            const double r_fix = 7.0 * R / 6.0;
            const double cp_out = published_pressure_at(g, s.regions, c + r_out * nrm) / half_rho_u2;
            const double cp_fix = published_pressure_at(g, s.regions, c + r_fix * nrm) / half_rho_u2;
            out.cp_err = std::max(out.cp_err, std::fabs(cp_out - cp_potential(R, r_out, theta)));
            out.cp_err_fixed = std::max(out.cp_err_fixed, std::fabs(cp_fix - cp_potential(R, r_fix, theta)));
            ++out.samples;
        }
    }
    return out;
}

[[nodiscard]] const char* verdict(bool within) { return within ? "within" : "MISS"; }

void print_s1_p3(uint32_t n, const S1Result& r, double cm_band, double cp_band, double drag_band) {
    std::printf("=== validation record: S1 sphere in a uniform stream, R/dx %u, P3 checks ===\n"
                "path: CPU reference, %s\n"
                "config: %u^3 at dx %.4g m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on "
                "-x, other faces open, U h/dx 0.1, n_v 16 CG iterations from a cold start; %s\n"
                "measure: the added mass C_M = F_x h / (rho V U) from wall_load(), step 1 = %.4f\n"
                "reference: 0.5 (potential flow, an impulsively started sphere; Batchelor 1967, ch. 6)\n"
                "tolerance: 0.5 +/- %.2g (test-docs/2026-10-13-solids-tolerances.md, S1)\n"
                "verdict: %s\n"
                "measure: max |C_p - C_p,pot| over %u samples one cell out along the normal, step 3 = %.4f\n"
                "reference: potential flow's C_p at the same points, 1 - |u|^2/U^2 (Batchelor 1967, ch. 6)\n"
                "tolerance: <= %.2g (test-docs/2026-10-13-solids-tolerances.md, S1)\n"
                "verdict: %s\n"
                "measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3 = %.4f\n"
                "reference: 0 (d'Alembert)\n"
                "tolerance: <= %.2g (test-docs/2026-10-13-solids-tolerances.md, S1)\n"
                "verdict: %s\n"
                "measure: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3 = %.2e\n"
                "reference: 0 (axisymmetric)\n"
                "tolerance: <= 0.01 (test-docs/2026-10-13-solids-tolerances.md, S1)\n"
                "verdict: %s\n"
                "measure: the worse of steps 1 and 3's final relative residuals = %.3e\n"
                "reference: a converged solve\n"
                "tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)\n"
                "verdict: %s\n",
                n, build_line().c_str(), 8u * n, 0.06 / n, r.run_line.c_str(), r.c_m, cm_band,
                verdict(std::fabs(r.c_m - 0.5) <= cm_band), r.samples, r.cp_err, cp_band, verdict(r.cp_err <= cp_band),
                r.drag, drag_band, verdict(r.drag <= drag_band), r.side, verdict(r.side <= 0.01),
                static_cast<double>(r.residual), verdict(r.residual <= 1.0e-4f));
}

// S1's P3 checks at R/dx 6. Mutations: wall_load() returning 0 (C_M = 0) and
// its sign flipped (C_M = -0.5) both fail.
TEST(AirflowSolidsStep, S1TheSpheresAddedMassPressureAndForcesFromP3) {
    const S1Result r = run_s1(6);
    print_s1_p3(6, r, 0.24, 0.32, 0.06);
    EXPECT_NEAR(r.c_m, 0.5, 0.24);
    EXPECT_LE(r.cp_err, 0.32);
    EXPECT_LE(r.drag, 0.06);
    EXPECT_LE(r.side, 0.01);
    EXPECT_LE(r.residual, 1.0e-4f);
}

// S1 at R/dx 12 and the order check: C_p at the fixed radius 7R/6 must fall
// to at most 0.7 of R/dx 6's error (reported, not checked, when R/dx 6's is
// under a tenth of its band). Opt-in: SPADE_AIRFLOW_VALIDATION.
TEST(AirflowSolidsStep, S1At12CellsPerRadiusAndItsOrder) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run S1 at R/dx 12 and its order";
    const S1Result coarse = run_s1(6);
    const S1Result fine = run_s1(12);
    print_s1_p3(12, fine, 0.12, 0.20, 0.04);
    const bool checked = coarse.cp_err_fixed >= 0.1 * 0.32;
    const double ratio = fine.cp_err_fixed / coarse.cp_err_fixed;
    std::printf("measure: the order check, e(12) / e(6) for C_p at r = 7R/6 = %.3f (e(6) %.4f, e(12) %.4f)\n"
                "reference: first order or better\n"
                "tolerance: %s\n"
                "verdict: %s\n",
                ratio, coarse.cp_err_fixed, fine.cp_err_fixed,
                checked ? "<= 0.7 (test-docs/2026-10-13-solids-tolerances.md, order checks)"
                        : "none: e(6) is under a tenth of its band, so the order is reported, not checked",
                checked ? verdict(ratio <= 0.7) : "report only");
    EXPECT_NEAR(fine.c_m, 0.5, 0.12);
    EXPECT_LE(fine.cp_err, 0.20);
    EXPECT_LE(fine.drag, 0.04);
    EXPECT_LE(fine.side, 0.01);
    EXPECT_LE(fine.residual, 1.0e-4f);
    if (checked) {
        EXPECT_LE(ratio, 0.7);
    }
}

struct CouetteResult {
    double error;        // max |u_theta - exact| / (Omega R1)
    uint32_t off_wall;   // closed faces not carrying the rigid velocity, bit for bit
    double circulation;  // the mean u_theta over the measured cells, m/s
    float residual;      // the last step's relative pressure residual
};

// S2 (Test/Docs): circular Couette flow. The cylinder R1 turns at Omega about
// y inside a fixed tube R2 = 2 R1, in a one-cell slab periodic in y, both
// no-slip; Re = Omega R1 (R2 - R1) / nu = 10, Omega R1 h / dx = 0.5, run to
// T = 2 (R2 - R1)^2 / nu. Exact: u_theta = A r + B / r, A = -Omega / 3,
// B = 4 Omega R1^2 / 3 (Landau & Lifshitz, Fluid Mechanics, section 18).
[[nodiscard]] CouetteResult couette(uint32_t r1_cells, uint32_t n_v) {
    const float dx = 1.0e-3f;
    const float r1 = static_cast<float>(r1_cells) * dx;
    const float r2 = 2.0f * r1;
    const uint32_t n = 2u * (2u * r1_cells) + 4u;
    const GridShape shape{n, 1, n, dx};
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.face[kYMinus] = FaceBc::periodic;
    bc.face[kYPlus] = FaceBc::periodic;
    const float omega = 0.1f / r1;  // Omega R1 = 0.1 m/s
    const float nu = 0.1f * (r2 - r1) / 10.0f;
    const float h = 0.5f * dx / 0.1f;
    const FluidConfig cfg{.rho = 1.0f, .nu = nu, .h = h, .n_v = n_v, .visc_sweeps = 8};
    MacGrid g = *make_grid(shape, bc);
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const glm::vec3 centre(0.5f * static_cast<float>(n) * dx, 0.5f * dx, 0.5f * static_cast<float>(n) * dx);
    Scene scene;
    scene.shapes.push_back(Shape{.name = "cylinder", .kind = Kind::cylinder_y, .centre = centre, .radius = r1,
                                 .omega = {0.0f, omega, 0.0f}, .min_feature = 2.0f * r1});
    scene.shapes.push_back(Shape{.name = "tube", .kind = Kind::outside_cylinder_y, .centre = centre, .radius = r2,
                                 .min_feature = 4.0f * dx});
    EXPECT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value());
    const auto steps = static_cast<uint32_t>(2.0f * (r2 - r1) * (r2 - r1) / nu / h + 0.5f);
    float residual = 0.0f;
    for (uint32_t k = 0; k < steps; ++k) {
        const FluidHealth health = fluid_step(g, bc, cfg, s);
        EXPECT_EQ(health.fault, FluidFault::none) << describe(health, shape);
        if (health.fault != FluidFault::none) break;
        residual = health.pressure_residual;
    }
    CouetteResult out{0.0, 0u, 0.0, residual};
    const std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < vel[c]->size(); ++i) {
            if (s.solids.theta[c][i] == 0.0f) out.off_wall += (*vel[c])[i] == s.solids.wall[c][i] ? 0u : 1u;
        }
    }
    const double A = -static_cast<double>(omega) / 3.0;
    const double B = 4.0 * static_cast<double>(omega) * r1 * r1 / 3.0;
    double sum = 0.0;
    uint32_t count = 0;
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t i = 0; i < n; ++i) {
            const double rx = (static_cast<double>(i) + 0.5) * dx - centre.x;
            const double rz = (static_cast<double>(k) + 0.5) * dx - centre.z;
            const double r = std::sqrt(rx * rx + rz * rz);
            if (r < r1 + dx || r > r2 - dx) continue;
            const double uc = 0.5 * (static_cast<double>(g.u[u_index(shape, i, 0, k)]) + g.u[u_index(shape, i + 1u, 0, k)]);
            const double wc = 0.5 * (static_cast<double>(g.w[w_index(shape, i, 0, k)]) + g.w[w_index(shape, i, 0, k + 1u)]);
            const double ut = (uc * rz - wc * rx) / r;
            out.error = std::max(out.error, std::fabs(ut - (A * r + B / r)) / 0.1);
            sum += ut;
            ++count;
        }
    }
    out.circulation = sum / count;
    return out;
}

// S2's record (Test/Docs' format: every measure with its reference, tolerance
// and verdict). The last step's residual shows the run reached its fixed
// point, on which a steady measure carries no iteration error.
void print_couette(const char* title, uint32_t r1_cells, uint32_t n_v, const CouetteResult& r, double band) {
    const uint32_t n = 2u * (2u * r1_cells) + 4u;
    std::printf("=== validation record: %s ===\n"
                "path: CPU reference, %s\n"
                "config: a %ux1x%u slab (y periodic), dx 1 mm, R1 %u mm turning at Omega R1 = 0.1 m/s inside a "
                "fixed tube R2 = 2 R1, both no-slip, Re 10, Omega R1 h/dx 0.5, T = 2 (R2 - R1)^2 / nu, n_v %u, "
                "visc_sweeps 8\n"
                "measure: max |u_theta - u_theta,exact| / (Omega R1) over R1 + dx <= r <= R2 - dx = %.4f\n"
                "reference: A r + B / r (Landau & Lifshitz, Fluid Mechanics, section 18)\n"
                "tolerance: <= %g (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                "verdict: %s\n"
                "measure: closed faces off their rigid velocity = %u\n"
                "reference: 0 (a closed face carries its wall's velocity)\n"
                "tolerance: 0, exact (the solids design, section 3)\n"
                "verdict: %s\n"
                "measure: the mean u_theta over the measured cells = %.5f m/s\n"
                "reference: positive, the rotation's sense\n"
                "tolerance: > 0 (the cylinder turns the air its own way)\n"
                "verdict: %s\n"
                "measure: the last step's relative pressure residual = %.3e\n"
                "reference: a converged solve (the step's fixed point)\n"
                "tolerance: <= %.0e (kResidualAnnounce)\n"
                "verdict: %s\n",
                title, build_line().c_str(), n, n, r1_cells, n_v, r.error, band, r.error <= band ? "within" : "MISS",
                r.off_wall, r.off_wall == 0u ? "within" : "MISS", r.circulation,
                r.circulation > 0.0 ? "within" : "MISS", static_cast<double>(r.residual),
                static_cast<double>(kResidualAnnounce), r.residual <= kResidualAnnounce ? "within" : "MISS");
}

// Design test 2 / S2 at R1 = 16 dx (the gate case; R1 = 32 dx and the order
// check are opt-in). Tolerance <= 0.11 (Test/Docs). The exact checks: closed
// faces carry the rigid velocity bit for bit, and the air turns with the
// cylinder. Mutation: the rigid velocity at closed faces only (the links'
// terms ignored), which Test/Docs expects to miss by up to a factor of two.
TEST(AirflowSolidsStep, S2CircularCouetteFlow) {
    if (debug_build()) {
        GTEST_SKIP() << "debug build: S2 takes 100 s in debug (measured 2026-10-20; ctest's limit is 60 s); runs in "
                        "the release gate (15 s) and every leg (TD-5; the planar Couette check covers the wall "
                        "links in debug)";
    }
    const CouetteResult r = couette(16, 4);
    print_couette("S2 circular Couette flow, R1/dx 16", 16, 4, r, 0.11);
    EXPECT_LE(r.error, 0.11);
    EXPECT_EQ(r.off_wall, 0u);
    EXPECT_GT(r.circulation, 0.0);
    EXPECT_LE(r.residual, kResidualAnnounce);
}

TEST(AirflowSolidsStep, S2CircularCouetteFlowAt32CellsAndItsOrder) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run S2 at R1/dx 32 and its order";
    const CouetteResult coarse = couette(16, 4);
    const CouetteResult fine = couette(32, 4);
    print_couette("S2 circular Couette flow, R1/dx 32, and the order check", 32, 4, fine, 0.054);
    // Below a tenth of 0.11 the order is reported only: the band carries no
    // tolerance there (Test/Docs' S2 note).
    const bool order_checked = coarse.error >= 0.011;
    std::printf("measure: e(32) / e(16) = %.3f\n"
                "reference: e(16) = %.4f, the R1/dx 16 record\n"
                "tolerance: %s\n"
                "verdict: %s\n",
                fine.error / coarse.error, coarse.error,
                order_checked ? "<= 0.7 (an observed order of at least 0.5; test-docs/2026-10-13-solids-tolerances.md)"
                              : "none (report only: e(16) < 0.011; above it, <= 0.7)",
                order_checked ? (fine.error <= 0.7 * coarse.error ? "within" : "MISS") : "report only");
    EXPECT_LE(fine.error, 0.054);
    EXPECT_EQ(fine.off_wall, 0u);
    EXPECT_GT(fine.circulation, 0.0);
    EXPECT_LE(fine.residual, kResidualAnnounce);
    if (coarse.error >= 0.011) {
        EXPECT_LE(fine.error, 0.7 * coarse.error);
    }
}

// The viscous wall links' discriminating check (the lead's ruling, 2026-10-19:
// S2 at R1/dx 16 cannot tell the links from closed faces alone -- 0.0052
// against 0.0602, both inside its 0.11). Planar Couette flow between a still
// plate below and a plate moving at U above, their surfaces a quarter cell
// off the face planes (y = 3.25 dx and 12.75 dx), x and z periodic. The u
// samples nearest the walls sit 0.25 dx from them, so each link crosses at
// f = 0.25 and its term (alpha / f)(u_w - u) cancels the far neighbour's
// exactly for the linear profile: the exact solution is the discrete
// operator's fixed point, held to rounding. Closed faces alone put each wall
// at the closed sample, 0.75 dx too far, and the profile drifts toward the
// wrong line. Started on the exact profile, run half a diffusion time. Band
// measured and pinned (TD-2). Mutation: closed faces only (the links unread).
// Measured 2026-10-20 on MSVC 14.43 (_MSC_VER 1943), release: 5.624e-8 with
// the links, 6.459e-2 closed faces only; a gcc leg prints its own build line.
TEST(AirflowSolidsStep, PlanarCouetteFlowHoldsItsLinearProfile) {
    const float dx = 1.0e-3f;
    const GridShape shape{4, 16, 1, dx};
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.face[kXMinus] = FaceBc::periodic;
    bc.face[kXPlus] = FaceBc::periodic;
    bc.face[kZMinus] = FaceBc::periodic;
    bc.face[kZPlus] = FaceBc::periodic;
    const float U = 0.1f;
    const float y0 = 3.25f * dx;
    const float y1 = 12.75f * dx;
    const FluidConfig cfg{.rho = 1.0f, .nu = 1.0e-3f, .h = 1.0e-3f, .n_v = 4, .visc_sweeps = 8};
    // links: false is the negative control, the rigid velocity at closed and
    // inside samples only (every link index cleared after set_solids()).
    const auto run = [&](bool links) {
        MacGrid g = *make_grid(shape, bc);
        for_each_sample(shape, 0, [&](std::size_t idx, glm::vec3 x) {
            g.u[idx] = std::min(std::max(U * (x.y - y0) / (y1 - y0), 0.0f), U);
        });
        FluidScratch s = *make_fluid_scratch(g, bc, cfg);
        Scene scene;
        scene.shapes.push_back(Shape{.name = "floor", .kind = Kind::box, .centre = {2.0f * dx, y0 - 2.625f * dx, 0.5f * dx},
                                     .half = {20.0f * dx, 2.625f * dx, 20.0f * dx}, .min_feature = 5.25f * dx});
        scene.shapes.push_back(Shape{.name = "belt", .kind = Kind::box, .centre = {2.0f * dx, y1 + 2.625f * dx, 0.5f * dx},
                                     .half = {20.0f * dx, 2.625f * dx, 20.0f * dx}, .velocity = {U, 0.0f, 0.0f},
                                     .min_feature = 5.25f * dx});
        EXPECT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value()) << s.solids.run_line;
        if (!links) {
            for (std::vector<uint32_t>& l : s.solids.link) std::fill(l.begin(), l.end(), kNoWallLinks);
        }
        // Half a diffusion time, (y1 - y0)^2 / nu / 2 = 45 steps.
        for (int k = 0; k < 45; ++k) {
            const FluidHealth health = fluid_step(g, bc, cfg, s);
            EXPECT_EQ(health.fault, FluidFault::none) << describe(health, shape);
        }
        double err = 0.0;
        for_each_sample(shape, 0, [&](std::size_t idx, glm::vec3 x) {
            if (x.y <= y0 || x.y >= y1) return;
            const double exact = static_cast<double>(U) * (x.y - y0) / (y1 - y0);
            err = std::max(err, std::fabs(static_cast<double>(g.u[idx]) - exact) / U);
        });
        return err;
    };
    const double with_links = run(true);
    const double closed_only = run(false);
    std::printf("=== validation record: planar Couette, plates a quarter cell off the faces ===\n"
                "path: CPU reference, %s\n"
                "config: 4x16x1 (x, z periodic), dx 1 mm, plates at y = 3.25 dx (still) and 12.75 dx (U 0.1 m/s), "
                "nu 1e-3, h 1 ms, 45 steps (half a diffusion time) from the exact profile, n_v 4, visc_sweeps 8\n"
                "measure: max |u - u_exact| / U, with the wall links = %.3e\n"
                "reference: 0 (the linear profile is the discrete operator's fixed point)\n"
                "tolerance: <= 1e-5 (TD-2: measured 5.6e-08 on MSVC 1943 release, rounding)\n"
                "verdict: %s\n"
                "measure: max |u - u_exact| / U, closed faces only (the negative control) = %.3e\n"
                "reference: the linear profile, missed: each wall sits 0.75 dx too far\n"
                "tolerance: > 1e-5 (must exceed the band)\n"
                "verdict: %s\n",
                build_line().c_str(), with_links, with_links <= 1.0e-5 ? "within" : "MISS", closed_only,
                closed_only > 1.0e-5 ? "within" : "MISS");
    EXPECT_LE(with_links, 1.0e-5);
    EXPECT_GT(closed_only, 1.0e-5) << "the negative control must fail the band";
}

// --- The posed body from A4 and B4 (P2 Task 9; design test 7) ---------------

// The compiled reference quad (tests/reference_airframe.hpp) holding only its
// body box, parts[0]: analytic (A4), or (B4) replaced by a mesh of the same
// box at the same mount, its half extents the analytic box's own parameters
// (part_bounds() is conservative: sdf_bounds() pads by about 1e-6 m).
[[nodiscard]] spade::vehicles::ModelType body_box_quad(bool as_mesh) {
    spade::vehicles::ModelType m =
        spade::vehicles::compile_airframe(spade::vehicles::fixtures::reference_quad()).value().model;
    m.parts.resize(1);
    if (!as_mesh) return m;
    spade::vehicles::PartDesc& part = m.parts[0];
    EXPECT_EQ(part.shape.nodes.size(), 1u);
    EXPECT_EQ(part.shape.nodes[0].kind, static_cast<uint32_t>(spade::SdfPrim::box));
    const glm::dvec3 hi(glm::vec3(part.shape.nodes[0].params));
    const glm::dvec3 lo = -hi;
    constexpr uint32_t kBoxTriangles[12][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                                               {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
    std::vector<glm::dvec3> positions;
    for (uint32_t v = 0; v < 8; ++v) {
        positions.push_back(glm::dvec3((v & 1u) != 0 ? hi.x : lo.x, ((v >> 1) & 1u) != 0 ? hi.y : lo.y,
                                       ((v >> 2) & 1u) != 0 ? hi.z : lo.z));
    }
    std::vector<uint32_t> indices;
    for (const auto& t : kBoxTriangles) indices.insert(indices.end(), {t[0], t[1], t[2]});
    part.mesh = spade::make_mesh(spade::MeshInput{.positions = std::move(positions),
                                                  .indices = std::move(indices),
                                                  .unit = spade::MeshUnit::metre},
                                 part.name)
                    .value();
    part.shape = spade::SdfProgram{};
    return m;
}

// Design test 7: one body, tilted, moving and turning, at the D1 nest's dx =
// 6 mm, sampled from A4's analytic box and from B4's mesh of it through the
// same adapter (vehicles::sample_parts(), vehicles::part_bounds()): the same
// apertures, wall velocities and distances within the distance's fp32
// rounding. An implementation comparison (TD-2), measured 2026-10-20 on MSVC
// 14.43 release: theta 2.98e-8 (half an ulp near 0.5), wall velocity 0 (one
// formula, part_wall_velocity()), phi 2.2e-11 m, 4336 closed faces in each.
// Pinned so another compiler's last-bit rounding stays inside (Test/Docs'
// TD-2 point, 2026-10-20): theta at two ulps of 1 (1.2e-7), which covers one
// ulp of phi wherever theta is not clamped; phi per sample at 4 ulps of
// max(|phi_A|, |phi_B|, dx), since one ulp of a distance is 4.7e-10 m at dx
// and 1.5e-8 m at 0.2 m; the walls equal; no face closed in one form and
// open beyond theta's band in the other (a face at the clamp's edge may round
// either way). Mutations: B4's box 1 um wider, or built from part_bounds()'
// padded box (0.9 um wider), fail it (about 2,100 ulps at dx).
TEST(AirflowSolidsStep, ThePosedBodyGivesTheSameAperturesFromA4AndB4) {
    const float dx = 0.006f;
    const GridShape shape{48, 48, 48, dx};
    spade::BodyState body{};
    body.pos = glm::vec3(1.0f, 2.0f, 3.0f);
    body.orient = glm::angleAxis(0.4f, glm::normalize(glm::vec3(0.3f, 1.0f, -0.2f)));
    body.vel = glm::vec3(0.5f, 0.0f, 0.0f);
    body.omega_body = glm::vec3(0.0f, 2.0f, 0.0f);
    const glm::vec3 origin = body.pos - glm::vec3(24.0f * dx);
    const std::array<float, 4> omega{};
    const std::array<float, 4> psi{};
    const auto build = [&](const spade::vehicles::ModelType& m) {
        SolidField f = *make_solid_field(shape);
        const auto posed = spade::vehicles::posed_parts(m, body, omega, psi, spade::vehicles::PartFilter::rigid_only);
        EXPECT_TRUE(posed.has_value()) << (posed ? "" : posed.error().context);
        if (!posed) return f;
        const spade::vehicles::AirflowSolids v(m, *posed, origin);
        const auto parts = v.parts();
        EXPECT_TRUE(parts.has_value()) << (parts ? "" : parts.error().context);
        if (!parts) return f;
        const auto ok = build_solids(f, shape, {}, *parts, v.sampler());
        EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
        return f;
    };
    const spade::vehicles::ModelType a4 = body_box_quad(false);
    const spade::vehicles::ModelType b4 = body_box_quad(true);
    const SolidField fa = build(a4);
    const SolidField fb = build(b4);
    ASSERT_TRUE(fa.any && fb.any);
    constexpr float kThetaBand = 1.2e-7f;
    float d_theta = 0.0f;
    float d_wall = 0.0f;
    double phi_ulps = 0.0;  // the worst |phi_A - phi_B| in ulps of max(|phi_A|, |phi_B|, dx)
    uint32_t closed_a = 0;
    uint32_t closed_b = 0;
    uint32_t flips = 0;  // closed in one form, open beyond theta's band in the other
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < fa.theta[c].size(); ++i) {
            const float ta = fa.theta[c][i];
            const float tb = fb.theta[c][i];
            d_theta = std::max(d_theta, std::fabs(ta - tb));
            d_wall = std::max(d_wall, std::fabs(fa.wall[c][i] - fb.wall[c][i]));
            const float pa = fa.phi[c][i];
            const float pb = fb.phi[c][i];
            if (pa < 1.0e29f && pb < 1.0e29f) {
                const float m = std::max({std::fabs(pa), std::fabs(pb), dx});
                const float ulp = std::nextafter(m, std::numeric_limits<float>::infinity()) - m;
                phi_ulps = std::max(phi_ulps, static_cast<double>(std::fabs(pa - pb)) / ulp);
            }
            closed_a += closed(ta) ? 1u : 0u;
            closed_b += closed(tb) ? 1u : 0u;
            if (closed(ta) != closed(tb) && std::max(ta, tb) > kThetaBand) ++flips;
        }
    }
    std::printf("=== the posed body, A4 analytic against B4 mesh (design test 7) ===\n"
                "path: CPU reference, %s\n"
                "config: the compiled reference quad's 0.12 x 0.03 x 0.12 m body box, tilted 0.4 rad, moving 0.5 m/s, "
                "turning 2 rad/s; 48^3 at dx 6 mm\n"
                "measure: closed faces, A4 and B4 = %u and %u\n"
                "reference: the body's cut, in each form\n"
                "tolerance: none (report only; the theta band below compares them face by face)\n"
                "verdict: report only\n"
                "measure: max |theta_A - theta_B| = %.3e\n"
                "reference: 0 (one box, two forms)\n"
                "tolerance: <= 1.2e-7 (TD-2: two ulps of 1; measured 3.0e-08 on MSVC 1943 release)\n"
                "verdict: %s\n"
                "measure: max |wall_A - wall_B| = %.3e m/s\n"
                "reference: 0 (one formula, part_wall_velocity())\n"
                "tolerance: 0, exact (one formula)\n"
                "verdict: %s\n"
                "measure: the worst |phi_A - phi_B|, in ulps of max(|phi_A|, |phi_B|, dx) = %.2f\n"
                "reference: 0 (one box, two forms)\n"
                "tolerance: <= 4 (TD-2: the distance's fp32 rounding, per sample; Test/Docs, 2026-10-20)\n"
                "verdict: %s\n"
                "measure: faces closed in one form and open beyond theta's band in the other = %u\n"
                "reference: 0 (one box, two forms)\n"
                "tolerance: 0, exact (a face at the clamp's edge may round either way inside theta's band)\n"
                "verdict: %s\n",
                build_line().c_str(), closed_a, closed_b, static_cast<double>(d_theta),
                d_theta <= kThetaBand ? "within" : "MISS",
                static_cast<double>(d_wall), d_wall == 0.0f ? "within" : "MISS", phi_ulps,
                phi_ulps <= 4.0 ? "within" : "MISS", flips, flips == 0u ? "within" : "MISS");
    EXPECT_GT(closed_a, 0u);
    EXPECT_EQ(flips, 0u);
    EXPECT_LE(d_theta, kThetaBand);
    EXPECT_EQ(d_wall, 0.0f);
    EXPECT_LE(phi_ulps, 4.0);
}

// The chosen count holds (P2 Task 9): the worst case the study measured, the
// sheet on the odd plane from a cold start, solves to below the announcement
// threshold in kSolidsCgIterations, and reaches 1e-3 within it (the design's
// convergence check, design test 3: the port took 4, pinned at 8). Mutation:
// 4 iterations, which leave it at 1.8e-3, announced.
TEST(AirflowSolidsStep, TheChosenIterationCountConvergesTheSheetFromCold) {
    const float dx = 1.0f / 32.0f;
    const GridShape shape{32, 8, 8, dx};
    const DomainBc bc = all_periodic();
    const auto solve = [&](uint32_t plane, uint32_t n_v) {
        MacGrid g = *make_grid(shape, bc);
        std::fill(g.u.begin(), g.u.end(), 1.0f);
        const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.001f, .n_v = n_v};
        FluidScratch s = *make_fluid_scratch(g, bc, cfg);
        Scene scene;
        EXPECT_TRUE(set_solids(s, g, bc, s3_sheet(scene, plane, dx), scene.sampler()).has_value());
        const FluidHealth health = fluid_step(g, bc, cfg, s);
        EXPECT_EQ(health.fault, FluidFault::none) << describe(health, shape);
        return health;
    };
    for (const uint32_t plane : {16u, 17u}) {
        uint32_t to_1e3 = 0;
        for (uint32_t n = 1; n <= kSolidsCgIterations && to_1e3 == 0; ++n) {
            if (solve(plane, n).pressure_residual <= 1.0e-3f) to_1e3 = n;
        }
        const FluidHealth health = solve(plane, kSolidsCgIterations);
        std::printf("=== the chosen iteration count, the sheet on plane %u from cold ===\n"
                    "path: CPU reference, %s\n"
                    "config: S3's channel (32x8x8 periodic, a sheet 0.25 dx thick), one step from a cold start\n"
                    "measure: CG iterations to a relative residual of 1e-3 = %u\n"
                    "reference: the port, 4\n"
                    "tolerance: <= %u (kSolidsCgIterations; 0 means not within it)\n"
                    "verdict: %s\n"
                    "measure: the relative residual after %u iterations = %.3e\n"
                    "reference: a converged solve\n"
                    "tolerance: <= %.0e (kResidualAnnounce)\n"
                    "verdict: %s\n",
                    plane, build_line().c_str(), to_1e3, kSolidsCgIterations, to_1e3 != 0u ? "within" : "MISS",
                    kSolidsCgIterations, static_cast<double>(health.pressure_residual),
                    static_cast<double>(kResidualAnnounce), health.unconverged ? "MISS" : "within");
        EXPECT_NE(to_1e3, 0u) << plane;
        EXPECT_FALSE(health.unconverged) << describe(health, shape);
    }
}

}  // namespace
