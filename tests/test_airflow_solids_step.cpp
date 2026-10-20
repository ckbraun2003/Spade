// The fluid step with solids (the solids design; P2 Tasks 6-9): S3's sheet
// across a channel, S1's P2 checks, S2's Couette flow, rebuilds, plan A's
// bits without solids, and the posed drone. Bands are Test/Docs'
// (test-docs/2026-10-13-solids-tolerances.md) or measured and pinned (TD-2),
// as each says.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/fluid_step.hpp"

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
        bool closed_hold_the_wall = true;
        for (uint32_t k = 0; k < 8; ++k) {
            for (uint32_t j = 0; j < 8; ++j) {
                closed_hold_the_wall = closed_hold_the_wall && g.u[u_index(shape, plane, j, k)] == 0.0f;
            }
        }
        const double step = static_cast<double>(g.p[cell_index(shape, plane - 1u, 3, 3)]) -
                            static_cast<double>(g.p[cell_index(shape, plane, 3, 3)]);
        const double expected = static_cast<double>(rho) * U * (1.0 - dx) / h;
        std::printf("=== validation record: S3 sheet across a channel, plane %u (%s) ===\n"
                    "config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho %.3f, U %.1f, h %.4f, one "
                    "projection, n_v 32 CG iterations from a cold start\n"
                    "measure: every closed face carries the wall velocity = %s\n"
                    "measure: max |u| / U after the projection = %.3e\n"
                    "reference: 0 (exact in 1D)\n"
                    "tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                    "verdict: %s\n"
                    "measure: the pressure step dp h / (rho U (L - dx)) = %.6f\n"
                    "reference: 1 (exact: the stencil is exact for a linear field)\n"
                    "tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                    "verdict: %s\n"
                    "solve: relative residual %.3e\n"
                    "%s\n",
                    plane, plane % 2u == 0u ? "even" : "odd", static_cast<double>(rho), static_cast<double>(U),
                    static_cast<double>(h), closed_hold_the_wall ? "yes" : "NO",
                    static_cast<double>(u_max / U), u_max / U <= 1.0e-3f ? "within" : "OUTSIDE", step / expected,
                    std::fabs(step / expected - 1.0) <= 1.0e-3 ? "within" : "OUTSIDE",
                    static_cast<double>(rep.residual), s.solids.run_line.c_str());
        EXPECT_TRUE(closed_hold_the_wall) << plane;
        EXPECT_LE(u_max / U, 1.0e-3f) << plane;
        EXPECT_NEAR(step / expected, 1.0, 1.0e-3) << plane;
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
        for (int step = 0; step < 4; ++step) EXPECT_EQ(fluid_step(g, bc, cfg, s).fault, FluidFault::none);
        return digest(g);
    };
    const uint64_t plain = run(0);
    EXPECT_EQ(run(1), plain);
    EXPECT_EQ(run(2), plain);
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
    }
    EXPECT_EQ(moved.regions.label, fresh.regions.label);
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

    bool walls_hold = true;
    const std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < vel[c]->size(); ++i) {
            if (s.solids.theta[c][i] == 0.0f) walls_hold = walls_hold && (*vel[c])[i] == 0.0f;
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
                "config: 48^3 at dx 1 cm, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on "
                "-x, other faces open, U h/dx 0.1, n_v 16 CG iterations, step 1 from a cold start\n"
                "measure: every closed face carries the wall velocity = %s\n"
                "measure: the solve's final relative residual = %.3e\n"
                "reference: a converged solve\n"
                "tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)\n"
                "verdict: %s\n"
                "measure: the aperture divergence left, over the right side's = %.3e\n"
                "tolerance: <= 2e-4 (twice the residual's band: the update's rounding)\n"
                "verdict: %s\n"
                "%s\n",
                walls_hold ? "yes" : "NO", static_cast<double>(health.pressure_residual),
                health.pressure_residual <= 1.0e-4f ? "within" : "OUTSIDE", static_cast<double>(div_ratio),
                div_ratio <= 2.0e-4f ? "within" : "OUTSIDE", s.solids.run_line.c_str());
    EXPECT_TRUE(walls_hold);
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

struct CouetteResult {
    double error;        // max |u_theta - exact| / (Omega R1)
    bool walls_hold;     // every closed face carries the rigid velocity, bit for bit
    double circulation;  // the mean u_theta over the measured cells, m/s
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
    for (uint32_t k = 0; k < steps; ++k) {
        const FluidHealth health = fluid_step(g, bc, cfg, s);
        EXPECT_EQ(health.fault, FluidFault::none) << describe(health, shape);
        if (health.fault != FluidFault::none) break;
    }
    CouetteResult out{0.0, true, 0.0};
    const std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < vel[c]->size(); ++i) {
            if (s.solids.theta[c][i] == 0.0f) out.walls_hold = out.walls_hold && (*vel[c])[i] == s.solids.wall[c][i];
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

// Design test 2 / S2 at R1 = 16 dx (the gate case; R1 = 32 dx and the order
// check are opt-in). Tolerance <= 0.11 (Test/Docs). The exact checks: closed
// faces carry the rigid velocity bit for bit, and the air turns with the
// cylinder. Mutation: the rigid velocity at closed faces only (the links'
// terms ignored), which Test/Docs expects to miss by up to a factor of two.
TEST(AirflowSolidsStep, S2CircularCouetteFlow) {
    const CouetteResult r = couette(16, 4);
    std::printf("=== validation record: S2 circular Couette flow, R1/dx 16 ===\n"
                "config: a 68x1x68 slab (y periodic), dx 1 mm, R1 16 mm turning at Omega R1 = 0.1 m/s inside a "
                "fixed tube R2 = 2 R1, both no-slip, Re 10, Omega R1 h/dx 0.5, T = 2 (R2 - R1)^2 / nu, n_v 4, "
                "visc_sweeps 8\n"
                "measure: max |u_theta - u_theta,exact| / (Omega R1) over R1 + dx <= r <= R2 - dx = %.4f\n"
                "reference: A r + B / r (Landau & Lifshitz, Fluid Mechanics, section 18)\n"
                "tolerance: <= 0.11 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                "verdict: %s\n"
                "measure: closed faces carry the rigid velocity = %s; mean u_theta = %.5f m/s (the rotation's sense: "
                "positive)\n",
                r.error, r.error <= 0.11 ? "within" : "OUTSIDE", r.walls_hold ? "yes" : "NO", r.circulation);
    EXPECT_LE(r.error, 0.11);
    EXPECT_TRUE(r.walls_hold);
    EXPECT_GT(r.circulation, 0.0);
}

TEST(AirflowSolidsStep, S2CircularCouetteFlowAt32CellsAndItsOrder) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run S2 at R1/dx 32 and its order";
    const CouetteResult coarse = couette(16, 4);
    const CouetteResult fine = couette(32, 4);
    std::printf("=== validation record: S2 circular Couette flow, R1/dx 32, and the order check ===\n"
                "measure: max |u_theta - u_theta,exact| / (Omega R1) at R1/dx 32 = %.4f\n"
                "tolerance: <= 0.054 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                "verdict: %s\n"
                "measure: e(32) / e(16) = %.3f\n"
                "tolerance: <= 0.7 (an observed order of at least 0.5; reported only if e(16) < 0.011)\n"
                "verdict: %s\n",
                fine.error, fine.error <= 0.054 ? "within" : "OUTSIDE", fine.error / coarse.error,
                coarse.error < 0.011 ? "report only" : (fine.error <= 0.7 * coarse.error ? "within" : "OUTSIDE"));
    EXPECT_LE(fine.error, 0.054);
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
    ASSERT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value()) << s.solids.run_line;
    // Half a diffusion time, (y1 - y0)^2 / nu / 2 = 45 steps.
    for (int k = 0; k < 45; ++k) {
        const FluidHealth health = fluid_step(g, bc, cfg, s);
        ASSERT_EQ(health.fault, FluidFault::none) << describe(health, shape);
    }
    double err = 0.0;
    for_each_sample(shape, 0, [&](std::size_t idx, glm::vec3 x) {
        if (x.y <= y0 || x.y >= y1) return;
        const double exact = static_cast<double>(U) * (x.y - y0) / (y1 - y0);
        err = std::max(err, std::fabs(static_cast<double>(g.u[idx]) - exact) / U);
    });
    std::printf("=== planar Couette, plates a quarter cell off the faces ===\n"
                "measure: max |u - u_exact| / U over the fluid after 45 steps = %.3e\n",
                err);
    EXPECT_LE(err, 1.0e-4);
}

}  // namespace
