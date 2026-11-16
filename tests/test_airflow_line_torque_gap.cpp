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

}  // namespace
