// V1 of the airflow spec's validation (§11.6): C1, C0 and C8 (airflow core
// plan, Task 11). Not golden-feeding: reference values are computed in double
// with std:: functions.
//
// THE RECORDS. Each case prints one record block in the shape Test/Docs fixed
// (docs/design/test-docs/01-verification.md): an opening
// "=== validation record: <case> ===" line, then one "key: value" per line --
// path, config, then for each measure in order: measure, reference,
// tolerance, verdict. A measure line is "measure: <quantity> = <value>", with
// a unit after the value where there is one. The verdict is computed from the
// same comparison the EXPECT uses, so the record cannot disagree with the
// test. Whoever pastes a block into tests/data/airflow/records/ adds
// "commit:" and "date:" after its opening line.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <string>
#include <vector>

#include "data/airflow/ghia1982_cavity.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"

namespace {

using namespace spade::physics::airflow;
namespace ref = spade::testing::airflow_reference;

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

// "CPU reference", the compiler and the preset.
[[nodiscard]] std::string path_line() {
#if defined(NDEBUG)
    const char* preset = "release";
#else
    const char* preset = "debug";
#endif
#if defined(_MSC_VER)
    return "CPU reference, MSVC " + std::to_string(_MSC_VER) + ", " + preset;
#else
    return std::string("CPU reference, ") + __VERSION__ + ", " + preset;
#endif
}

[[nodiscard]] const char* verdict(bool within) { return within ? "within" : "MISS"; }

// u on the line x = 0.5 at height y. The u faces at i = n/2 sit at
// y_j = (j + 1/2) / n; below the first, u falls linearly to the floor's 0;
// above the last, it rises linearly to the lid's 1.
[[nodiscard]] double centreline_u(const MacGrid& g, double y) {
    const uint32_t n = g.shape.ny;
    const uint32_t i = g.shape.nx / 2u;
    const auto at = [&](uint32_t j) { return static_cast<double>(g.u[u_index(g.shape, i, j, 0)]); };
    const double pos = y * n - 0.5;
    if (pos <= 0.0) return at(0) * (y * n / 0.5);
    if (pos >= n - 1.0) return at(n - 1u) + (pos - (n - 1.0)) / 0.5 * (1.0 - at(n - 1u));
    const auto j0 = static_cast<uint32_t>(pos);
    return at(j0) + (pos - j0) * (at(j0 + 1u) - at(j0));
}

// v on the line y = 0.5 at x: v faces at j = n/2, x_i = (i + 1/2) / n; the
// side walls hold v = 0.
[[nodiscard]] double centreline_v(const MacGrid& g, double x) {
    const uint32_t n = g.shape.nx;
    const uint32_t j = g.shape.ny / 2u;
    const auto at = [&](uint32_t i) { return static_cast<double>(g.v[v_index(g.shape, i, j, 0)]); };
    const double pos = x * n - 0.5;
    if (pos <= 0.0) return at(0) * (x * n / 0.5);
    if (pos >= n - 1.0) return at(n - 1u) * (1.0 - (pos - (n - 1.0)) / 0.5);
    const auto i0 = static_cast<uint32_t>(pos);
    return at(i0) + (pos - i0) * (at(i0 + 1u) - at(i0));
}

struct CavityResult {
    double u_error = 0.0;
    double v_error = 0.0;
};

// The cavity at `re` on an n x n slab to time t_end; the two measures against
// Ghia's tables, each checked against `tolerance`. With `skip_v_point_117`,
// Table II's point 117 is left out of the v maximum (Re 400: printed
// -0.23827, a suspected typo in the original; Task 10's note).
[[nodiscard]] CavityResult run_cavity(uint32_t n, double re, double t_end, const double* u_ref, const double* v_ref,
                                      double tolerance, bool skip_v_point_117) {
    DomainBc bc;
    bc.face = {FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::periodic, FaceBc::periodic};
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    MacGrid g = *make_grid({n, n, 1, 1.0f / static_cast<float>(n)}, bc);
    const FluidConfig cfg{.rho = 1.0f, .nu = static_cast<float>(1.0 / re), .h = 0.5f / static_cast<float>(n),
                          .n_v = 2, .visc_sweeps = 8};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const auto steps = static_cast<uint32_t>(t_end / static_cast<double>(cfg.h));
    for (uint32_t k = 0; k < steps; ++k) {
        const FluidHealth h = fluid_step(g, bc, cfg, s);
        EXPECT_EQ(h.fault, FluidFault::none) << describe(h, g.shape);
        if (h.fault != FluidFault::none) break;
    }
    CavityResult r;
    for (int q = 0; q < 17; ++q) {
        r.u_error = std::max(r.u_error, std::fabs(centreline_u(g, ref::kGhiaY[q]) - u_ref[q]));
        if (skip_v_point_117 && q == ref::kGhiaTableIIPoint117) continue;
        r.v_error = std::max(r.v_error, std::fabs(centreline_v(g, ref::kGhiaX[q]) - v_ref[q]));
    }
    const std::string v_quantity = skip_v_point_117
                                       ? "max |v - v_Ghia| on y = 0.5, Table II point 117 left out (printed -0.23827, a "
                                         "suspected typo in the original)"
                                       : "max |v - v_Ghia| on y = 0.5";
    std::printf("=== validation record: C1 lid-driven cavity, Re %.0f, %ux%u ===\n"
                "path: %s\n"
                "config: %ux%u (a quasi-2D slab, z periodic), Re %.0f, T %.0f (%u steps of h = %.6f), n_v 2, "
                "visc_sweeps 8, rho 1, U 1\n"
                "measure: max |u - u_Ghia| on x = 0.5 = %.4f\n"
                "reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)\n"
                "tolerance: <= %.2f (plan A Task 11, spec §11.2, fixed before the first run)\n"
                "verdict: %s\n"
                "measure: %s = %.4f\n"
                "reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)\n"
                "tolerance: <= %.2f (plan A Task 11, spec §11.2, fixed before the first run)\n"
                "verdict: %s\n",
                re, n, n, path_line().c_str(), n, n, re, t_end, steps, static_cast<double>(cfg.h), r.u_error,
                tolerance, verdict(r.u_error <= tolerance), v_quantity.c_str(), r.v_error, tolerance,
                verdict(r.v_error <= tolerance));
    return r;
}

TEST(AirflowValidation, C1CavityAtRe100On32CellsIsWithinTheGateBand) {
#if !defined(NDEBUG)
    // The lead, 2026-10-12: 1280 steps take 67 s in debug, past the airflow
    // tests' debug-gate budget (spec §4.6). The release gate runs it; this skip
    // goes when the CPU-performance task lands, with the new debug time.
    GTEST_SKIP() << "runs in the release gate; 67 s in debug (until the airflow CPU-performance task)";
#else
    const CavityResult r = run_cavity(32, 100.0, 20.0, ref::kGhiaU100, ref::kGhiaV100, 0.06, false);
    EXPECT_LE(r.u_error, 0.06);
    EXPECT_LE(r.v_error, 0.06);
#endif
}

TEST(AirflowValidation, C1CavityAtRe100On128Cells) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the 128x128 cavity at Re 100";
    const CavityResult r = run_cavity(128, 100.0, 40.0, ref::kGhiaU100, ref::kGhiaV100, 0.02, false);
    EXPECT_LE(r.u_error, 0.02);
    EXPECT_LE(r.v_error, 0.02);
}

TEST(AirflowValidation, C1CavityAtRe400On128Cells) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the 128x128 cavity at Re 400";
    const CavityResult r = run_cavity(128, 400.0, 60.0, ref::kGhiaU400, ref::kGhiaV400, 0.02, true);
    EXPECT_LE(r.u_error, 0.02);
    EXPECT_LE(r.v_error, 0.02);
}

// The kinetic energy per unit mass, from the faces, the periodic alias faces left out.
[[nodiscard]] double kinetic_energy(const MacGrid& g) {
    const GridShape& s = g.shape;
    double sum[3] = {0.0, 0.0, 0.0};
    for (uint32_t c = 0; c < 3; ++c) {
        const std::vector<float>& f = c == 0 ? g.u : c == 1 ? g.v : g.w;
        for (uint32_t k = 0; k < s.nz; ++k) {
            for (uint32_t j = 0; j < s.ny; ++j) {
                for (uint32_t i = 0; i < s.nx; ++i) {
                    const std::size_t idx =
                        c == 0 ? u_index(s, i, j, k) : c == 1 ? v_index(s, i, j, k) : w_index(s, i, j, k);
                    sum[c] += static_cast<double>(f[idx]) * f[idx];
                }
            }
        }
    }
    const double cells = static_cast<double>(s.nx) * s.ny * s.nz;
    return 0.5 * (sum[0] + sum[1] + sum[2]) / cells;
}

TEST(AirflowValidation, C0TaylorGreenDissipationIsRecorded) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the 64^3 Taylor-Green vortex";
    const uint32_t n = 64;
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid({n, n, n, static_cast<float>(2.0 * std::numbers::pi / n)}, bc);
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) {
        g.u[idx] = static_cast<float>(std::sin(double(x.x)) * std::cos(double(x.y)) * std::cos(double(x.z)));
    });
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) {
        g.v[idx] = static_cast<float>(-std::cos(double(x.x)) * std::sin(double(x.y)) * std::cos(double(x.z)));
    });
    const FluidConfig cfg{.rho = 1.0f, .nu = 0.0f, .h = 0.5f * g.shape.dx, .n_v = 2};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const double ke0 = kinetic_energy(g);
    std::printf("=== validation record: C0 Taylor-Green vortex, 64^3 ===\n"
                "path: %s\n"
                "config: 64^3 periodic 2 pi box, inviscid (the numerics' own dissipation), h = %.6f, n_v 2\n",
                path_line().c_str(), static_cast<double>(cfg.h));
    const auto report = [](double t, double ke) {
        std::printf("measure: kinetic energy per unit mass at t = %.2f = %.6f\n"
                    "reference: none (report only; DNS data enter at V3, spec §11.6)\n"
                    "tolerance: none (report only)\n"
                    "verdict: report only\n",
                    t, ke);
    };
    report(0.0, ke0);
    double t = 0.0;
    double ke = ke0;
    double next_report = 2.5;
    while (t < 10.0) {
        const FluidHealth h = fluid_step(g, bc, cfg, s);
        ASSERT_EQ(h.fault, FluidFault::none) << describe(h, g.shape);
        t += cfg.h;
        if (t >= next_report) {
            ke = kinetic_energy(g);
            ASSERT_TRUE(std::isfinite(ke));
            report(next_report, ke);
            next_report += 2.5;
        }
    }
    EXPECT_LT(ke, ke0);
}

TEST(AirflowValidation, C8ActuatorDiscFarWakeAgainstMomentumTheory) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the actuator-disc wake (~6 min)";
    const GridShape shape{64, 128, 64, 0.01f};
    const DomainBc bc;  // open faces, still air: every face an outlet
    MacGrid g = *make_grid(shape, bc);
    const float rho = 1.225f;
    const float vh = 4.0f;
    const float radius = 0.1f;
    const float area = static_cast<float>(std::numbers::pi) * radius * radius;
    const ActuatorDisc disc{.centre = {0.32f, 0.96f, 0.32f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = radius,
                            .thrust = 2.0f * rho * area * vh * vh};
    ASSERT_TRUE(check_disc(disc, shape).has_value());
    const FluidConfig cfg{.rho = rho, .nu = 0.0f, .h = shape.dx / 8.0f, .n_v = 2};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const uint32_t steps = 384;
    const uint32_t window = 128;
    const uint32_t j = 76;  // y = 0.76 m = 2R below the disc
    double sum = 0.0;
    for (uint32_t k = 0; k < steps; ++k) {
        spread_actuator(disc, shape, rho, cfg.h, s.du, s.dv, s.dw);
        const FluidHealth h = fluid_step(g, bc, cfg, s);
        ASSERT_EQ(h.fault, FluidFault::none) << describe(h, g.shape);
        if (k >= steps - window) {
            const auto v = [&](uint32_t i, uint32_t kk) { return static_cast<double>(g.v[v_index(shape, i, j, kk)]); };
            sum += -0.25 * (v(31, 31) + v(32, 31) + v(31, 32) + v(32, 32));
        }
    }
    const double measured = sum / window;
    const double expected = vh * (1.0 + 2.0 / std::sqrt(5.0));
    const bool within = std::fabs(measured - expected) <= 0.1 * expected;
    std::printf("=== validation record: C8 actuator-disc far wake ===\n"
                "path: %s\n"
                "config: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T %.4f N, v_h 4 m/s, still air, inviscid, "
                "h = %.6f, n_v 2, %u steps\n"
                "measure: axial velocity on the axis 2R below the disc, mean of the last %u steps = %.4f m/s\n"
                "reference: momentum theory, v_h (1 + 2/sqrt 5) = %.4f m/s (tests/data/airflow/README.md, C8)\n"
                "tolerance: within 10%% (plan A Task 11, spec §11.2, fixed before the first run)\n"
                "verdict: %s\n",
                path_line().c_str(), static_cast<double>(disc.thrust), static_cast<double>(cfg.h), steps, window,
                measured, expected, verdict(within));
    EXPECT_TRUE(within) << measured << " against " << expected;
}

}  // namespace
