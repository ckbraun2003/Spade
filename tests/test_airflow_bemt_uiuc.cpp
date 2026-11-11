// ---------------------------------------------------------------------------
// test_airflow_bemt_uiuc.cpp -- L61's V1: the grid-free BEMT (tests/bemt_reference.hpp, Prandtl's tip loss on) on the library's
// uiuc_apce_10x5 stations (tests/library_props.gen.hpp: the UIUC propeller database's own geometry and measured C_T(J), C_Q(J)),
// with the ESTIMATED polar of engine/physics/airflow/blade.hpp, against UIUC's measurements. Bands: Test/Docs,
// docs/design/test-docs/2026-11-11-l61-actuator-line-bands.md, fixed before the first run.
//
//   V1a  the static point, a TRIGGER: C_T within 10% and C_Q within 15% of UIUC's (C_T 0.09636, C_Q 0.005870), and the polar stands;
//        outside either, the polar is calibrated ONCE to the static point (alpha_0 and the drag scale, two numbers for two),
//        and V1a then holds by construction.
//   V1b  the held-out J 0.2, 0.3, 0.4: |C - C_UIUC(J)| at most the band, as a share of the STATIC UIUC value. After a calibration
//        6.3, 8.7, 12% (C_T) and 6.4, 11, 16% (C_Q); if the polar stood 17, 19, 22% and 22, 26, 31% (the merged note, recomputed).
//   A second calibration is refused: a miss after one is a finding about the polar's shape.
//
// The configuration is the measured one: D 0.254 m, 5000 rpm, rho 1.225. The test prints the record's block (for
// tests/data/airflow/records/l61-bemt-uiuc.md) and asserts V1b.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include "bemt_reference.hpp"
#include "library_props.gen.hpp"
#include "physics/airflow/blade.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace bt = spade::testing;
namespace lp = spade::test::library_props;

constexpr double kD = 0.254;
constexpr double kRpm = 5000.0;
constexpr double kRho = 1.225;

af::BladeShape apce_10x5() {
    const lp::Prop& p = lp::kProps[0];  // uiuc_apce_10x5
    af::BladeShape b;
    b.blades = static_cast<uint32_t>(p.blades);
    b.radius = static_cast<float>(0.5 * kD);
    for (std::size_t i = 0; i < p.r_over_r.size(); ++i) {
        b.stations.push_back({static_cast<float>(p.r_over_r[i] * 0.5 * kD), static_cast<float>(p.c_over_r[i] * 0.5 * kD),
                              static_cast<float>(p.beta_deg[i])});
    }
    return b;
}

struct Coefs {
    double ct = 0.0;
    double cq = 0.0;
};

Coefs run(const af::BladeShape& blade, const af::PolarTable& polar, double j) {
    const double n = kRpm / 60.0;
    const auto loads = bt::bemt(bt::BemtInput{.blade = &blade,
                                              .polar = &polar,
                                              .omega = static_cast<float>(2.0 * std::numbers::pi * n),
                                              .rho = static_cast<float>(kRho),
                                              .climb = static_cast<float>(j * n * kD),
                                              .annuli = 128,
                                              .tip_loss = true});
    EXPECT_TRUE(loads.has_value()) << (loads ? "" : loads.error().context);
    if (!loads) return {};
    return {loads->thrust / (kRho * n * n * kD * kD * kD * kD), loads->torque / (kRho * n * n * kD * kD * kD * kD * kD)};
}

double table_at(std::span<const std::array<double, 2>> t, double j) {
    for (const auto& row : t) {
        if (std::abs(row[0] - j) < 1e-9) return row[1];
    }
    ADD_FAILURE() << "no UIUC row at J " << j;
    return 0.0;
}

TEST(AirflowBemtUiuc, V1TheEstimatedPolarAgainstTheMeasured10x5) {
    const lp::Prop& p = lp::kProps[0];
    ASSERT_EQ(std::string(p.id), "uiuc_apce_10x5");
    const af::BladeShape blade = apce_10x5();
    ASSERT_TRUE(af::check_blade(blade).has_value());
    const double ct0 = p.ct_static;
    const double cq0 = p.cq_static;

    af::EstimatedPolarShape shape;  // alpha_0 -3 deg, drag x1: as written before any comparison
    af::PolarTable polar = af::estimated_polar(shape);
    Coefs s = run(blade, polar, 0.0);
    const double e_ct = (s.ct - ct0) / ct0;
    const double e_cq = (s.cq - cq0) / cq0;
    const bool stood = std::abs(e_ct) <= 0.10 && std::abs(e_cq) <= 0.15;
    std::printf("V1a  estimated polar (alpha_0 %.2f deg, drag x%.3f): static C_T %.5f vs UIUC %.5f (%+.1f%%), C_Q %.6f vs %.6f (%+.1f%%): %s\n",
                static_cast<double>(shape.alpha0_deg), static_cast<double>(shape.cd_scale), s.ct, ct0, 100.0 * e_ct, s.cq, cq0,
                100.0 * e_cq, stood ? "stands" : "outside the trigger: calibrate once");

    if (!stood) {
        // ONE calibration: Newton on (alpha_0, log drag scale) for the two static numbers, numerical Jacobian, damped.
        double a0 = shape.alpha0_deg;
        double ls = 0.0;
        const auto eval = [&](double a, double l) {
            af::EstimatedPolarShape sh{static_cast<float>(a), static_cast<float>(std::exp(l))};
            const af::PolarTable t = af::estimated_polar(sh);
            const Coefs c = run(blade, t, 0.0);
            return std::array<double, 2>{std::log(c.ct / ct0), std::log(c.cq / cq0)};
        };
        for (int it = 0; it < 40; ++it) {
            const std::array<double, 2> f = eval(a0, ls);
            if (std::abs(f[0]) < 1e-4 && std::abs(f[1]) < 1e-4) break;
            const double h = 1e-3;
            const std::array<double, 2> fa = eval(a0 + h, ls);
            const std::array<double, 2> fl = eval(a0, ls + h);
            const double j00 = (fa[0] - f[0]) / h, j01 = (fl[0] - f[0]) / h;
            const double j10 = (fa[1] - f[1]) / h, j11 = (fl[1] - f[1]) / h;
            const double det = j00 * j11 - j01 * j10;
            ASSERT_NE(det, 0.0);
            double da = -(j11 * f[0] - j01 * f[1]) / det;
            double dl = -(-j10 * f[0] + j00 * f[1]) / det;
            const double cap = 2.0 / std::max(1.0, std::max(std::abs(da), std::abs(dl) * 5.0));  // damp a big step
            da *= std::min(1.0, cap);
            dl *= std::min(1.0, cap);
            a0 += da;
            ls += dl;
        }
        shape.alpha0_deg = static_cast<float>(a0);
        shape.cd_scale = static_cast<float>(std::exp(ls));
        polar = af::estimated_polar(shape);
        s = run(blade, polar, 0.0);
        std::printf("V1a  CALIBRATED once to the static point: alpha_0 %.3f deg, drag x%.4f: static C_T %.5f (%+.2f%%), C_Q %.6f (%+.2f%%)\n",
                    static_cast<double>(shape.alpha0_deg), static_cast<double>(shape.cd_scale), s.ct, 100.0 * (s.ct - ct0) / ct0,
                    s.cq, 100.0 * (s.cq - cq0) / cq0);
        EXPECT_NEAR(s.ct, ct0, 2e-3 * ct0);
        EXPECT_NEAR(s.cq, cq0, 2e-3 * cq0);
    }

    // V1b: the held-out J, as a share of the static UIUC value.
    const std::array<double, 3> js{0.2, 0.3, 0.4};
    const std::array<double, 3> band_ct = stood ? std::array<double, 3>{0.17, 0.19, 0.22} : std::array<double, 3>{0.063, 0.087, 0.12};
    const std::array<double, 3> band_cq = stood ? std::array<double, 3>{0.22, 0.26, 0.31} : std::array<double, 3>{0.064, 0.11, 0.16};
    for (std::size_t i = 0; i < js.size(); ++i) {
        const Coefs c = run(blade, polar, js[i]);
        const double ref_ct = table_at(p.ct_table, js[i]);
        const double ref_cq = table_at(p.cq_table, js[i]);
        const double d_ct = std::abs(c.ct - ref_ct) / ct0;
        const double d_cq = std::abs(c.cq - ref_cq) / cq0;
        std::printf("V1b  J %.1f: C_T %.5f vs UIUC %.5f, |diff| %.1f%% of static (band %.1f%%); C_Q %.6f vs %.6f, |diff| %.1f%% (band %.1f%%)\n",
                    js[i], c.ct, ref_ct, 100.0 * d_ct, 100.0 * band_ct[i], c.cq, ref_cq, 100.0 * d_cq, 100.0 * band_cq[i]);
        EXPECT_LE(d_ct, band_ct[i]) << "C_T at J " << js[i];
        EXPECT_LE(d_cq, band_cq[i]) << "C_Q at J " << js[i];
    }
    // Reported, not banded (the bands note): J 0.5.
    const Coefs c5 = run(blade, polar, 0.5);
    std::printf("V1   report J 0.5: C_T %.5f vs UIUC %.5f, C_Q %.6f vs %.6f\n", c5.ct, table_at(p.ct_table, 0.5), c5.cq,
                table_at(p.cq_table, 0.5));
}

}  // namespace
