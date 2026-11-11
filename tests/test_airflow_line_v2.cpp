// ---------------------------------------------------------------------------
// test_airflow_line_v2.cpp -- L61's V2: ONE rotor hovering in a CPU grid box, the actuator line (engine/physics/airflow/line.hpp)
// against the BEMT with Prandtl's tip loss (tests/bemt_reference.hpp), on D1's rotor: triblade_6inch_scaled's stations scaled to
// R 0.0762 m, 801 rad/s (7650 rpm), the ESTIMATED polar. Bands: Test/Docs, docs/design/test-docs/2026-11-11-l61-actuator-line-bands.md,
// fixed before the first run:
//   thrust within 10% of the BEMT; power (torque x omega) within 15%; the momentum flux rho * sum(w^2 dA) through the plane 1.5 R
//   below the rotor, over the line's own thrust, within 10% of 1. Reported, not banded: figure of merit, the far-wake speed.
//   The window: the mean over 10 R/v_h starting at 20 R/v_h (v_h from the BEMT); the run is UNSETTLED when the last two such
//   windows' thrusts differ by more than 2.5% (it then runs longer; the band is not widened).
//
// Opt-in: SPADE_V2_RDX=<19|25> (R/dx; 19 is 4 mm, 25 is 3 mm); optional SPADE_V2_NSUB (sub-azimuths, default 8) and
// SPADE_V2_STEPS (a short run for timing: no bands are asserted when it is set). The CPU's cost is the reason it is opt-in.
// It prints the record's block (tests/data/airflow/records/l61-actuator-line.md).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <string>
#include <vector>

#include "bemt_reference.hpp"
#include "library_props.gen.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/line.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace bt = spade::testing;
namespace lp = spade::test::library_props;

constexpr float kRadius = 0.0762f;
constexpr float kOmega = 801.0f;
constexpr float kRho = 1.225f;

uint32_t env_u32(const char* name, uint32_t fallback) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* v = std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    return v != nullptr ? static_cast<uint32_t>(std::strtoul(v, nullptr, 10)) : fallback;
}

af::BladeShape d1_blade() {
    for (const lp::Prop& p : lp::kProps) {
        if (std::string(p.id) != "triblade_6inch_scaled") continue;
        af::BladeShape b;
        b.blades = static_cast<uint32_t>(p.blades);
        b.radius = kRadius;
        for (std::size_t i = 0; i < p.r_over_r.size(); ++i) {
            b.stations.push_back({static_cast<float>(p.r_over_r[i]) * kRadius, static_cast<float>(p.c_over_r[i]) * kRadius,
                                  static_cast<float>(p.beta_deg[i])});
        }
        return b;
    }
    return {};
}

TEST(AirflowLineHover, V2OneRotorInAGridBox) {
    const uint32_t rdx = env_u32("SPADE_V2_RDX", 0);
    if (rdx == 0) GTEST_SKIP() << "opt-in: SPADE_V2_RDX=19 or 25 (the CPU run takes tens of minutes)";
    const uint32_t n_sub = env_u32("SPADE_V2_NSUB", 8);
    const uint32_t steps_override = env_u32("SPADE_V2_STEPS", 0);

    const af::BladeShape blade = d1_blade();
    ASSERT_TRUE(af::check_blade(blade).has_value());
    const af::PolarTable polar = af::estimated_polar();
    const auto bemt = bt::bemt(bt::BemtInput{.blade = &blade, .polar = &polar, .omega = kOmega, .rho = kRho, .climb = 0.0f,
                                             .annuli = 128, .tip_loss = true});
    ASSERT_TRUE(bemt.has_value());
    const double area = std::numbers::pi * kRadius * kRadius;
    const double v_h = std::sqrt(bemt->thrust / (2.0 * kRho * area));
    const double t_rv = kRadius / v_h;  // R / v_h, s

    // The box: 0.40 m across, the rotor 0.13 m under the top, 0.50 m of air below it. All faces open, still air.
    const float dx = kRadius / static_cast<float>(rdx);
    const auto cells = [&](float m) { return static_cast<uint32_t>(std::ceil(m / dx)); };
    af::GridShape shape{cells(0.40f), cells(0.63f), cells(0.40f), dx};
    af::DomainBc bc;
    auto grid = af::make_grid(shape, bc);
    ASSERT_TRUE(grid.has_value()) << grid.error().context;
    af::FluidConfig cfg;
    cfg.rho = kRho;
    cfg.h = 0.00038f * dx / 0.004f;  // D1's step at 4 mm, CFL held
    auto scratch = af::make_fluid_scratch(*grid, bc, cfg);
    ASSERT_TRUE(scratch.has_value()) << scratch.error().context;

    af::LineRotor rotor;
    rotor.centre = glm::vec3(0.5f * static_cast<float>(shape.nx) * dx, static_cast<float>(shape.ny) * dx - 0.13f,
                             0.5f * static_cast<float>(shape.nz) * dx);
    rotor.axis = glm::vec3(0.0f, 1.0f, 0.0f);  // the thrust pushes the drone up and the air down
    rotor.omega = kOmega;
    rotor.blade = &blade;
    rotor.polar = &polar;

    const uint32_t total = steps_override != 0u
                               ? steps_override
                               : static_cast<uint32_t>(std::ceil(40.0 * t_rv / static_cast<double>(cfg.h)));
    const auto step_of = [&](double t_rvs) { return static_cast<uint32_t>(std::ceil(t_rvs * t_rv / static_cast<double>(cfg.h))); };
    const std::array<uint32_t, 4> edge{step_of(20.0), step_of(30.0), step_of(40.0), total};
    std::printf("V2 setup: R/dx %u (dx %.3g mm), box %ux%ux%u = %.2f M cells, h %.3g ms, n_sub %u, %u steps (40 R/v_h = %.3f s); "
                "BEMT thrust %.4f N, torque %.5f N m, v_h %.3f m/s\n",
                rdx, static_cast<double>(dx) * 1000.0, shape.nx, shape.ny, shape.nz,
                static_cast<double>(shape.nx) * shape.ny * shape.nz / 1.0e6, static_cast<double>(cfg.h) * 1000.0, n_sub, total,
                40.0 * t_rv, bemt->thrust, bemt->torque, v_h);

    const uint32_t plane_j = static_cast<uint32_t>(std::floor((rotor.centre.y - 1.5f * kRadius) / dx));
    std::array<double, 2> thrust_sum{}, power_sum{}, flux_sum{};
    std::array<uint32_t, 2> count{};
    double wall = 0.0;
    double psi = 0.0;
    for (uint32_t step = 0; step < total; ++step) {
        const auto t0 = std::chrono::steady_clock::now();
        rotor.psi = static_cast<float>(psi);
        const auto rep = af::spread_line(rotor, *grid, kRho, cfg.h, n_sub, scratch->du, scratch->dv, scratch->dw);
        ASSERT_TRUE(rep.has_value()) << rep.error().context;
        const af::FluidHealth h = af::fluid_step(*grid, bc, cfg, *scratch);
        ASSERT_EQ(h.fault, af::FluidFault::none) << af::describe(h, grid->shape);
        psi = std::fmod(psi + static_cast<double>(kOmega) * cfg.h, 2.0 * std::numbers::pi);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        wall += ms;
        if (step < 5 || (step + 1) % 100 == 0) {
            std::printf("V2 step %u: %.0f ms (mean %.0f ms), line thrust %.4f N power %.3f W\n", step + 1, ms,
                        wall / (step + 1), static_cast<double>(rep->thrust), static_cast<double>(rep->power));
            std::fflush(stdout);
        }
        for (uint32_t w = 0; w < 2; ++w) {
            if (step >= edge[w] && step < edge[w + 1]) {
                double flux = 0.0;  // rho * sum of w^2 over the plane's cells, the downward speed squared
                for (uint32_t k = 0; k < shape.nz; ++k) {
                    for (uint32_t i = 0; i < shape.nx; ++i) {
                        const double v = 0.5 * (grid->v[af::v_index(shape, i, plane_j, k)] +
                                                grid->v[af::v_index(shape, i, plane_j + 1u, k)]);
                        if (v < 0.0) flux += v * v;
                    }
                }
                thrust_sum[w] += rep->thrust;
                power_sum[w] += rep->power;
                flux_sum[w] += kRho * flux * static_cast<double>(dx) * dx;
                ++count[w];
            }
        }
    }
    std::printf("V2 wall: %.1f s over %u steps (%.0f ms a step)\n", wall / 1000.0, total, wall / total);
    if (steps_override != 0u || count[0] == 0 || count[1] == 0) return;  // a timing run: no bands

    std::array<double, 2> thrust{}, power{}, flux{};
    for (std::size_t w = 0; w < 2; ++w) {
        thrust[w] = thrust_sum[w] / count[w];
        power[w] = power_sum[w] / count[w];
        flux[w] = flux_sum[w] / count[w];
    }
    const double drift = std::abs(thrust[1] - thrust[0]) / thrust[1];
    const double p_bemt = bemt->torque * static_cast<double>(kOmega);
    const double fm = std::pow(thrust[1], 1.5) / (std::sqrt(2.0 * kRho * area) * power[1]);
    std::printf("V2 R/dx %u windows [20,30] [30,40] R/v_h: line thrust %.4f, %.4f N (differ %.2f%%: %s); BEMT %.4f N\n", rdx,
                thrust[0], thrust[1], 100.0 * drift, drift <= 0.025 ? "settled" : "UNSETTLED", bemt->thrust);
    std::printf("V2 thrust %+.1f%% of BEMT (band 10%%); power %.3f W vs %.3f W: %+.1f%% (band 15%%); flux plane 1.5 R below / line "
                "thrust %.3f (band 0.9..1.1); FM %.3f (report)\n",
                100.0 * (thrust[1] / bemt->thrust - 1.0), power[1], p_bemt, 100.0 * (power[1] / p_bemt - 1.0), flux[1] / thrust[1],
                fm);
    EXPECT_LE(drift, 0.025) << "unsettled: run longer";
    EXPECT_NEAR(thrust[1], bemt->thrust, 0.10 * bemt->thrust);
    EXPECT_NEAR(power[1], p_bemt, 0.15 * p_bemt);
    EXPECT_NEAR(flux[1] / thrust[1], 1.0, 0.10);
}

}  // namespace
