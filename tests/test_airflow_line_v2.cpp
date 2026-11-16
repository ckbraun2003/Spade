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
    rotor.eps_cells = static_cast<float>(env_u32("SPADE_V2_EPS_X100", 200)) / 100.0f;  // a diagnostic sweep: the kernel width in cells

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
    double line_ms = 0.0;  // the line's own part of the wall (spread_line)
    double psi = 0.0;
    std::printf("V2 kernel eps %.2f cells = %.2f mm\n", static_cast<double>(rotor.eps_cells), static_cast<double>(rotor.eps_cells) * dx * 1000.0);
    for (uint32_t step = 0; step < total; ++step) {
        const auto t0 = std::chrono::steady_clock::now();
        rotor.psi = static_cast<float>(psi);
        const auto rep = af::spread_line(rotor, *grid, kRho, cfg.h, n_sub, scratch->du, scratch->dv, scratch->dw);
        ASSERT_TRUE(rep.has_value()) << rep.error().context;
        const auto after_line = std::chrono::steady_clock::now();
        line_ms += std::chrono::duration<double, std::milli>(after_line - t0).count();
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

    if (env_u32("SPADE_V2_PROBE", 0) != 0u) {
        // The power-gap diagnosis: the final field, the line's elements at 8 azimuths of one turn, per radius: what the line feels and
        // loads against the BEMT's annulus at the same radius, and the air sampled with a NARROWER Gaussian (eps 1 and 0.5 dx) in place of 2 dx.
        rotor.psi = static_cast<float>(psi);
        const std::vector<af::LineElement> el = af::line_elements(rotor, dx, 2.0f * static_cast<float>(std::numbers::pi) / kOmega, 8);
        const uint32_t per = static_cast<uint32_t>(el.size()) / (8u * blade.blades);
        const auto bm = bt::bemt(bt::BemtInput{.blade = &blade, .polar = &polar, .omega = kOmega, .rho = kRho, .climb = 0.0f,
                                               .annuli = per, .tip_loss = true});
        ASSERT_TRUE(bm.has_value());
        const auto narrow = [&](glm::vec3 pos, double eps_cells) {  // Gaussian eps_cells * dx, +-3 eps, normalised, per component
            glm::dvec3 out(0.0);
            for (uint32_t c = 0; c < 3; ++c) {
                const std::vector<float>& data = c == 0 ? grid->u : c == 1 ? grid->v : grid->w;
                double acc = 0.0, sum = 0.0;
                const int reach = static_cast<int>(std::ceil(3.0 * eps_cells));
                std::array<int, 3> lo{};
                std::array<double, 3> f{};
                for (uint32_t a = 0; a < 3; ++a) {
                    f[a] = pos[static_cast<glm::length_t>(a)] / dx - (a == c ? 0.0 : 0.5);
                    lo[a] = static_cast<int>(std::floor(f[a])) - reach;
                }
                for (int k = lo[2]; k <= lo[2] + 2 * reach + 1; ++k) {
                    for (int j = lo[1]; j <= lo[1] + 2 * reach + 1; ++j) {
                        for (int i = lo[0]; i <= lo[0] + 2 * reach + 1; ++i) {
                            const int n_x = static_cast<int>(shape.nx) + (c == 0 ? 1 : 0), n_y = static_cast<int>(shape.ny) + (c == 1 ? 1 : 0),
                                      n_z = static_cast<int>(shape.nz) + (c == 2 ? 1 : 0);
                            if (i < 0 || j < 0 || k < 0 || i >= n_x || j >= n_y || k >= n_z) continue;
                            const double d2 = ((i - f[0]) * (i - f[0]) + (j - f[1]) * (j - f[1]) + (k - f[2]) * (k - f[2])) /
                                              (eps_cells * eps_cells);
                            const double w = std::exp(-d2);
                            const std::size_t idx = c == 0 ? af::u_index(shape, i, j, k) : c == 1 ? af::v_index(shape, i, j, k)
                                                                                                 : af::w_index(shape, i, j, k);
                            acc += w * data[idx];
                            sum += w;
                        }
                    }
                }
                out[static_cast<glm::length_t>(c)] = sum > 0.0 ? acc / sum : 0.0;
            }
            return out;
        };
        struct Acc { double ua = 0, ua1 = 0, ua05 = 0, wt = 0, dt = 0, dq = 0; uint32_t n = 0; };
        std::vector<Acc> acc(per);
        for (std::size_t i = 0; i < el.size(); ++i) {
            const af::LineElement& e = el[i];
            const glm::vec3 air = af::line_sample(*grid, e.pos);
            const double ua = -glm::dot(air, rotor.axis);
            const double wt = static_cast<double>(kOmega) * e.r - glm::dot(air, e.motion);
            const af::SectionLoad l = af::section_load(polar, e.chord, e.sin_beta, e.cos_beta, static_cast<float>(ua),
                                                       static_cast<float>(wt), kRho);
            Acc& a = acc[i % per];
            a.ua += ua;
            a.ua1 += -glm::dot(narrow(e.pos, 1.0), glm::dvec3(rotor.axis));
            a.ua05 += -glm::dot(narrow(e.pos, 0.5), glm::dvec3(rotor.axis));
            a.wt += wt;
            a.dt += static_cast<double>(blade.blades) * l.axial;
            a.dq += static_cast<double>(blade.blades) * l.tangent * e.r;
            ++a.n;
        }
        const double pi = std::numbers::pi;
        std::printf("V2 probe R/dx %u, final field, 8 azimuths (24 blade samples a radius); eps_cells in the line: 2 (the line's own)\n", rdx);
        std::printf("V2 probe  r/R   chord/R | BEMT: v   phi  alpha  dT/dr  dQ/dr | line ua(eps2) ua(eps1) ua(eps.5) phi alpha dT/dr dQ/dr\n");
        double pi_b = 0, pp_b = 0, pi_l = 0, pp_l = 0;
        for (uint32_t k = 0; k < per; ++k) {
            const bt::AnnulusLoad& b = bm->annulus[k];
            const Acc& a = acc[k];
            const double ua = a.ua / a.n;
            const double wt = a.wt / a.n;
            const double phi_b = std::atan2(b.ua, b.wt), phi_l = std::atan2(ua, wt);
            const double dt_l = a.dt / a.n, dq_l = a.dq / a.n;
            const double dr = (blade.radius - blade.stations.front().r) / per;
            pi_b += b.dt_dr * b.ua * dr;
            pp_b += (b.dq_dr * kOmega - b.dt_dr * b.ua) * dr;
            pi_l += dt_l * ua * dr;
            pp_l += (dq_l * kOmega - dt_l * ua) * dr;
            std::printf("V2 probe  %.3f | %6.3f %6.2f %6.2f %7.3f %8.5f | %6.3f %6.3f %6.3f %6.2f %6.2f %7.3f %8.5f\n", b.r / kRadius, b.v,
                        phi_b * 180 / pi, (b.beta - phi_b) * 180 / pi, b.dt_dr, b.dq_dr, ua, a.ua1 / a.n, a.ua05 / a.n, phi_l * 180 / pi,
                        (b.beta - phi_l) * 180 / pi, dt_l, dq_l);
        }
        std::printf("V2 probe power split (W): BEMT induced %.3f profile %.3f total %.3f; line induced %.3f profile %.3f total %.3f\n",
                    pi_b, pp_b, pi_b + pp_b, pi_l, pp_l, pi_l + pp_l);
    }
    std::printf("V2 wall: %.1f s over %u steps (%.0f ms a step, of which the line %.1f ms)\n", wall / 1000.0, total, wall / total,
                line_ms / total);
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
