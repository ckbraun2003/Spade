// ---------------------------------------------------------------------------
// test_airflow_line_torque_gap.cpp -- L72h's measure-first items 1 and 3 on the actuator line's C_Q gap (the line makes C_Q 13%
// low on the UIUC 10x5 and 29% low on the MIT 5x4 at the 12 mm width, C_T 1 to 6% high). Read-only, CPU, seconds; no kernel and
// no polar change; no band: it prints what it measures (tests/data/airflow/records/l61-actuator-line.md's style) and asserts only
// that the numbers are finite.
//   item 1: the grid-free BEMT (tests/bemt_reference.hpp) on the same stations and the same estimated polar, against UIUC's
//           static C_T and C_Q (the library's, at the prop's run rpm). If the BEMT alone is as low in C_Q as the line, the gap is the
//           polar, not the line.
//   item 3: the load integration with ZERO induced velocity: the engine's own section_load() over the line's element set with the
//           air at rest (ua 0, wt omega r), summed in closed form, against the line's report on a grid of still air (its first step).
//           It separates the integration from the flow.
// Also printed, not adopted: the drag scale at which the BEMT's C_Q meets UIUC's (a sensitivity, the polar is not touched).
//
// Opt-in: SPADE_TORQUE_GAP=1.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

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

constexpr float kRho = 1.225f;
constexpr double kInch = 0.0254;

bool opted_in() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_TORQUE_GAP") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

af::BladeShape blade_of(const lp::Prop& p, double diameter) {
    af::BladeShape b;
    b.blades = static_cast<uint32_t>(p.blades);
    b.radius = static_cast<float>(0.5 * diameter);
    for (std::size_t i = 0; i < p.r_over_r.size(); ++i) {
        b.stations.push_back({static_cast<float>(p.r_over_r[i] * 0.5 * diameter),
                              static_cast<float>(p.c_over_r[i] * 0.5 * diameter), static_cast<float>(p.beta_deg[i])});
    }
    return b;
}

struct Case {
    const char* id;
    double diameter;
};

// The BEMT's thrust and torque on `blade` with the polar shape `shape`, Prandtl's tip loss on or off.
bt::RotorLoads solve(const af::BladeShape& blade, const af::EstimatedPolarShape& shape, float omega, bool tip_loss) {
    const af::PolarTable polar = af::estimated_polar(shape);
    const auto r = bt::bemt(bt::BemtInput{.blade = &blade, .polar = &polar, .omega = omega, .rho = kRho, .climb = 0.0f,
                                          .annuli = 128, .tip_loss = tip_loss});
    return r.has_value() ? *r : bt::RotorLoads{};
}

TEST(AirflowLineTorqueGap, BemtAndTheZeroInductionLoadAgainstUiuc) {
    if (!opted_in()) GTEST_SKIP() << "opt-in: SPADE_TORQUE_GAP=1 (a measurement, CPU, seconds)";
    const Case cases[] = {{"uiuc_apce_10x5", 10.0 * kInch}, {"uiuc_mit_5x4", 5.0 * kInch}};
    for (const Case& c : cases) {
        const lp::Prop* prop = nullptr;
        for (const lp::Prop& p : lp::kProps) {
            if (std::string(p.id) == c.id) prop = &p;
        }
        ASSERT_NE(prop, nullptr) << c.id;
        const double d = c.diameter;
        const double n_rev = prop->run_rpm / 60.0;
        const float omega = static_cast<float>(2.0 * std::numbers::pi * n_rev);
        const double dim_t = static_cast<double>(kRho) * n_rev * n_rev * std::pow(d, 4);
        const double dim_q = dim_t * d;
        const double ct_uiuc = prop->ct_static;
        const double cq_uiuc = prop->cq_static;
        const af::BladeShape blade = blade_of(*prop, d);
        ASSERT_TRUE(af::check_blade(blade).has_value());
        std::printf("torque-gap %s: D %.4f m, %.0f rpm, %u blades; UIUC static C_T %.5f C_Q %.6f (C_Q/C_T %.4f)\n",
                    c.id, d, prop->run_rpm, blade.blades, ct_uiuc, cq_uiuc, cq_uiuc / ct_uiuc);

        // Item 1: the BEMT alone.
        for (const bool tip_loss : {true, false}) {
            const bt::RotorLoads b = solve(blade, {}, omega, tip_loss);
            const double ct = static_cast<double>(b.thrust) / dim_t;
            const double cq = static_cast<double>(b.torque) / dim_q;
            std::printf("torque-gap %s item 1 BEMT (tip loss %s): C_T %.5f (%+.1f%% vs UIUC), C_Q %.6f (%+.1f%%), "
                        "C_Q/C_T %.4f, mean induced %.3f m/s\n",
                        c.id, tip_loss ? "on" : "off", ct, 100.0 * (ct / ct_uiuc - 1.0), cq,
                        100.0 * (cq / cq_uiuc - 1.0), cq / ct, static_cast<double>(b.inflow_mean));
            EXPECT_TRUE(std::isfinite(ct) && std::isfinite(cq));
        }
        // The drag scale at which the BEMT's C_Q (tip loss on) meets UIUC's, by bisection: a sensitivity, not a change.
        {
            double lo = 0.5, hi = 4.0;
            for (int i = 0; i < 40; ++i) {
                const double mid = 0.5 * (lo + hi);
                const af::EstimatedPolarShape trial{-3.0f, static_cast<float>(mid)};
                const bt::RotorLoads b = solve(blade, trial, omega, true);
                (static_cast<double>(b.torque) / dim_q < cq_uiuc ? lo : hi) = mid;
            }
            const double scale = 0.5 * (lo + hi);
            const af::EstimatedPolarShape fitted{-3.0f, static_cast<float>(scale)};
            const bt::RotorLoads b = solve(blade, fitted, omega, true);
            std::printf("torque-gap %s sensitivity (not adopted): drag scale %.3f makes the BEMT's C_Q meet UIUC's, "
                        "with C_T %.5f (%+.1f%%)\n",
                        c.id, scale, static_cast<double>(b.thrust) / dim_t,
                        100.0 * (static_cast<double>(b.thrust) / dim_t / ct_uiuc - 1.0));
        }

        // Item 3: zero induced velocity, closed form over the line's own elements, against the line on still air.
        const af::PolarTable polar = af::estimated_polar();
        const uint32_t rdx = 16;
        const float radius = blade.radius;
        const float dx = radius / static_cast<float>(rdx);
        const auto cells = [&](double m) { return static_cast<uint32_t>(std::ceil(m / static_cast<double>(dx))); };
        const af::GridShape shape{cells(5.25 * radius), cells(8.3 * radius), cells(5.25 * radius), dx};
        const af::DomainBc bc;
        auto grid = af::make_grid(shape, bc);
        ASSERT_TRUE(grid.has_value());
        af::FluidConfig cfg;
        cfg.rho = kRho;
        cfg.h = 0.00038f * dx / 0.004f;
        auto scratch = af::make_fluid_scratch(*grid, bc, cfg);
        ASSERT_TRUE(scratch.has_value());
        af::LineRotor rotor;
        rotor.centre = glm::vec3(0.5f * static_cast<float>(shape.nx) * dx,
                                 static_cast<float>(shape.ny) * dx - 1.7f * radius,
                                 0.5f * static_cast<float>(shape.nz) * dx);
        rotor.axis = glm::vec3(0.0f, 1.0f, 0.0f);
        rotor.omega = omega;
        rotor.blade = &blade;
        rotor.polar = &polar;
        constexpr uint32_t kSub = 8;
        const auto rep = af::spread_line(rotor, *grid, kRho, cfg.h, kSub, scratch->du, scratch->dv, scratch->dw);
        ASSERT_TRUE(rep.has_value());
        double thrust = 0.0, torque = 0.0;
        for (const af::LineElement& e : af::line_elements(rotor, dx, cfg.h, kSub)) {
            const af::SectionLoad s = af::section_load(polar, e.chord, e.sin_beta, e.cos_beta, 0.0f, omega * e.r, kRho);
            thrust += static_cast<double>(s.axial) * static_cast<double>(e.dr);
            torque += static_cast<double>(s.tangent) * static_cast<double>(e.r) * static_cast<double>(e.dr);
        }
        thrust /= kSub;
        torque /= kSub;
        const double ct0 = thrust / dim_t;
        const double cq0 = torque / dim_q;
        std::printf("torque-gap %s item 3 zero induced velocity: closed form C_T %.5f C_Q %.6f (C_Q/C_T %.4f); "
                    "the line on still air (R/dx %u, first step) C_T %.5f C_Q %.6f; differ %.3g%% / %.3g%%\n",
                    c.id, ct0, cq0, cq0 / ct0, rdx, static_cast<double>(rep->thrust) / dim_t,
                    static_cast<double>(rep->torque) / dim_q,
                    100.0 * (static_cast<double>(rep->thrust) / thrust - 1.0),
                    100.0 * (static_cast<double>(rep->torque) / torque - 1.0));
        EXPECT_TRUE(std::isfinite(ct0) && std::isfinite(cq0));
    }
}

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

// Item 2 (L72h, the lead's word): what the line's elements feel and load, per station, in a settled CPU hover of the UIUC 10x5
// in the V2b box, against the BEMT's annulus at the same radius. Read-only: the line, its sampling and the polar are the
// engine's, unchanged. The air is sampled where the blade IS, as spread_line() does it (the grid as the step begins, at the
// step's sub-azimuths), and averaged over the last 10% of the run; its sum is checked against the line's own report. The run is
// the CPU line (bit equal to the GPU's, tests/test_gpu_airflow_line.cpp); R/dx 16 keeps it to minutes, the width held in mm (the
// gap's resolution dependence at a fixed width is 0.3-0.5 points, the 2026-11-16 record).
// Opt-in: SPADE_TORQUE_GAP=1 and SPADE_TORQUE_GAP_EPS_MM10=<width in tenths of a mm> (R/dx SPADE_TORQUE_GAP_RDX, default 16).
TEST(AirflowLineTorqueGap, SampledInflowAndAngleOfAttackPerStationAgainstTheBemt) {
    if (!opted_in()) GTEST_SKIP() << "opt-in: SPADE_TORQUE_GAP=1 and SPADE_TORQUE_GAP_EPS_MM10=120 (CPU, minutes)";
    const double eps_mm = static_cast<double>(env_u32("SPADE_TORQUE_GAP_EPS_MM10", 0)) / 10.0;
    if (eps_mm <= 0.0) GTEST_SKIP() << "set SPADE_TORQUE_GAP_EPS_MM10";
    const uint32_t rdx = env_u32("SPADE_TORQUE_GAP_RDX", 16);
    const lp::Prop* prop = nullptr;
    for (const lp::Prop& p : lp::kProps) {
        if (std::string(p.id) == "uiuc_apce_10x5") prop = &p;
    }
    ASSERT_NE(prop, nullptr);
    const double d = 10.0 * kInch;
    const double n_rev = prop->run_rpm / 60.0;
    const float omega = static_cast<float>(2.0 * std::numbers::pi * n_rev);
    const af::BladeShape blade = blade_of(*prop, d);
    const af::PolarTable polar = af::estimated_polar();
    const float radius = blade.radius;
    const double area = std::numbers::pi * static_cast<double>(radius) * static_cast<double>(radius);
    const double v_h = std::sqrt(prop->ct_static * static_cast<double>(kRho) * n_rev * n_rev * std::pow(d, 4) /
                                 (2.0 * static_cast<double>(kRho) * area));
    const double t_rv = static_cast<double>(radius) / v_h;

    const float dx = radius / static_cast<float>(rdx);
    const double eps_cells = eps_mm * 1.0e-3 / static_cast<double>(dx);
    ASSERT_TRUE(eps_cells >= 1.0 && eps_cells <= 3.0) << eps_mm << " mm is " << eps_cells << " cells";
    const auto cells = [&](double m) { return static_cast<uint32_t>(std::ceil(m / static_cast<double>(dx))); };
    const af::GridShape shape{cells(5.25 * radius), cells(8.3 * radius), cells(5.25 * radius), dx};
    const af::DomainBc bc;
    auto grid = af::make_grid(shape, bc);
    ASSERT_TRUE(grid.has_value());
    af::FluidConfig cfg;
    cfg.rho = kRho;
    cfg.h = 0.00038f * dx / 0.004f;
    auto scratch = af::make_fluid_scratch(*grid, bc, cfg);
    ASSERT_TRUE(scratch.has_value());
    af::LineRotor rotor;
    rotor.centre = glm::vec3(0.5f * static_cast<float>(shape.nx) * dx,
                             static_cast<float>(shape.ny) * dx - 1.7f * radius,
                             0.5f * static_cast<float>(shape.nz) * dx);
    rotor.axis = glm::vec3(0.0f, 1.0f, 0.0f);
    rotor.omega = omega;
    rotor.blade = &blade;
    rotor.polar = &polar;
    rotor.eps_cells = static_cast<float>(eps_cells);
    constexpr uint32_t kSub = 8;
    const uint32_t total = static_cast<uint32_t>(std::ceil(30.0 * t_rv / static_cast<double>(cfg.h)));
    std::printf("torque-gap item 2: 10x5, %.0f rpm, R/dx %u (dx %.3f mm), box %ux%ux%u, kernel %.1f mm = %.2f cells, "
                "%u steps (30 R/v_h = %.3f s)\n",
                prop->run_rpm, rdx, static_cast<double>(dx) * 1000.0, shape.nx, shape.ny, shape.nz, eps_mm, eps_cells,
                total, 30.0 * t_rv);

    struct Station {
        double ua = 0.0, wt = 0.0, alpha = 0.0, dt = 0.0, dq = 0.0, r = 0.0, dr = 0.0;
        uint32_t n = 0;
    };
    std::vector<Station> st;
    double thrust_last = 0.0, torque_last = 0.0;
    uint32_t last_n = 0;
    constexpr double kRad = 180.0 / std::numbers::pi;
    double psi = 0.0;
    for (uint32_t step = 0; step < total; ++step) {
        rotor.psi = static_cast<float>(psi);
        const bool last = step + total / 10u >= total;  // the last 10% of the run
        if (last) {
            const std::vector<af::LineElement> el = af::line_elements(rotor, dx, cfg.h, kSub);
            const uint32_t per = static_cast<uint32_t>(el.size()) / (kSub * blade.blades);
            if (st.empty()) st.resize(per);
            for (uint32_t k = 0; k < kSub * blade.blades; ++k) {
                for (uint32_t i = 0; i < per; ++i) {
                    const af::LineElement& e = el[k * per + i];
                    const glm::vec3 air = af::line_sample(*grid, e.pos, rotor.eps_cells);
                    const float ua = -glm::dot(air, rotor.axis);
                    const float wt = omega * e.r - glm::dot(air, e.motion);
                    const af::SectionLoad s = af::section_load(polar, e.chord, e.sin_beta, e.cos_beta, ua, wt, kRho);
                    const double beta = std::atan2(static_cast<double>(e.sin_beta), static_cast<double>(e.cos_beta));
                    Station& a = st[i];
                    a.ua += static_cast<double>(ua);
                    a.wt += static_cast<double>(wt / (omega * e.r));
                    a.alpha += (beta - std::atan2(static_cast<double>(ua), static_cast<double>(wt))) * kRad;
                    a.dt += static_cast<double>(s.axial);
                    a.dq += static_cast<double>(s.tangent) * static_cast<double>(e.r);
                    a.r += static_cast<double>(e.r);
                    a.dr = static_cast<double>(e.dr);
                    ++a.n;
                }
            }
        }
        const auto rep = af::spread_line(rotor, *grid, kRho, cfg.h, kSub, scratch->du, scratch->dv, scratch->dw);
        ASSERT_TRUE(rep.has_value());
        const af::FluidHealth h = af::fluid_step(*grid, bc, cfg, *scratch);
        ASSERT_EQ(h.fault, af::FluidFault::none) << af::describe(h, grid->shape);
        psi = std::fmod(psi + static_cast<double>(omega) * cfg.h, 2.0 * std::numbers::pi);
        if (last) {
            thrust_last += static_cast<double>(rep->thrust);
            torque_last += static_cast<double>(rep->torque);
            ++last_n;
        }
    }
    const uint32_t per = static_cast<uint32_t>(st.size());
    const auto bm = bt::bemt(bt::BemtInput{.blade = &blade, .polar = &polar, .omega = omega, .rho = kRho, .climb = 0.0f,
                                           .annuli = per, .tip_loss = true});
    ASSERT_TRUE(bm.has_value());
    ASSERT_EQ(bm->annulus.size(), per);
    std::printf("torque-gap item 2 per station (the line: the mean over the last %u steps at the blades' own "
                "positions; the BEMT: Prandtl, %u annuli); ua is the air's axial speed through the rotor (m/s), "
                "alpha the angle of attack (deg), dT/dr and dQ/dr all blades (N/m, N m/m)\n",
                last_n, per);
    std::printf("  r/R    ua_line ua_bemt   alpha_line alpha_bemt   dT/dr_line dT/dr_bemt   dQ/dr_line dQ/dr_bemt   "
                "wt_line/(omega r)\n");
    double sum_t = 0.0, sum_q = 0.0;
    for (uint32_t i = 0; i < per; ++i) {
        const Station& a = st[i];
        const double m = static_cast<double>(a.n);
        const double dt = a.dt / m * blade.blades;
        const double dq = a.dq / m * blade.blades;
        const bt::AnnulusLoad& b = bm->annulus[i];
        const double alpha_b = (b.beta - std::atan2(b.ua, b.wt)) * kRad;
        std::printf("  %.3f  %7.3f %7.3f   %9.2f %10.2f   %10.3f %10.3f   %10.5f %10.5f   %.4f\n",
                    a.r / m / static_cast<double>(radius), a.ua / m, b.ua, a.alpha / m, alpha_b, dt, b.dt_dr, dq,
                    b.dq_dr, a.wt / m);
        sum_t += dt * a.dr;
        sum_q += dq * a.dr;
    }
    const double t_line = thrust_last / last_n;
    const double q_line = torque_last / last_n;
    std::printf("torque-gap item 2 totals: the stations' sum thrust %.4f N torque %.5f N m; the line's own report "
                "thrust %.4f N torque %.5f N m; BEMT thrust %.4f N torque %.5f N m (line/BEMT %+.1f%% thrust, %+.1f%% "
                "torque)\n",
                sum_t, sum_q, t_line, q_line, static_cast<double>(bm->thrust), static_cast<double>(bm->torque),
                100.0 * (t_line / static_cast<double>(bm->thrust) - 1.0),
                100.0 * (q_line / static_cast<double>(bm->torque) - 1.0));
    EXPECT_TRUE(std::isfinite(sum_t) && std::isfinite(sum_q));
}

}  // namespace
