// P9's reference data (tests/data/airflow/caradonna1981_rotor.hpp): Caradonna
// & Tung's hovering model rotor at the low-Mach condition, as the report
// prints it. Nothing reads it before D3; these pin its shape to Table 17's,
// the condition's numbers to each other, and the pressures to the report's
// own integral. Host only, milliseconds.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <iterator>
#include <numbers>
#include <span>
#include <vector>

#include "data/airflow/caradonna1981_rotor.hpp"

namespace {

namespace ref = spade::testing::airflow_reference;

// Table 17's stations, and each surface's tap count as printed.
TEST(AirflowCaradonnaData, TheStationsAreTable17s) {
    const double r_over_R[] = {0.50, 0.68, 0.80, 0.89, 0.96};
    const std::size_t upper[] = {7, 14, 17, 18, 16};
    const std::size_t lower[] = {5, 8, 9, 9, 9};
    ASSERT_EQ(ref::kCaradonnaTable17.size(), 5u);
    std::size_t taps = 0;
    for (std::size_t s = 0; s < ref::kCaradonnaTable17.size(); ++s) {
        const ref::CaradonnaStation& st = ref::kCaradonnaTable17[s];
        EXPECT_EQ(st.r_over_R, r_over_R[s]);
        EXPECT_EQ(st.upper.size(), upper[s]) << st.r_over_R;
        EXPECT_EQ(st.lower.size(), lower[s]) << st.r_over_R;
        taps += st.upper.size() + st.lower.size();
        for (const std::span<const ref::CaradonnaTap> side : {st.upper, st.lower}) {
            for (std::size_t i = 0; i < side.size(); ++i) {
                EXPECT_GE(side[i].x_c, 0.0) << st.r_over_R;
                EXPECT_LE(side[i].x_c, 1.0) << st.r_over_R;
                if (i > 0) {
                    EXPECT_GT(side[i].x_c, side[i - 1].x_c) << st.r_over_R << " tap " << i;
                }
            }
        }
    }
    EXPECT_EQ(taps, 112u) << "72 upper and 40 lower taps";
}

// The rotor speed, radius and tip Mach number agree: omega R is 149.6 m/s,
// so the printed tip Mach implies a speed of sound of about 341 m/s, air's
// at about 16 C. The chord is the radius over the aspect ratio, 0.1905 m.
TEST(AirflowCaradonnaData, TheConditionsNumbersAgree) {
    const double omega_r = ref::kCaradonnaRpm * 2.0 * std::numbers::pi / 60.0 * ref::kCaradonnaRadiusM;
    EXPECT_NEAR(omega_r, 149.6, 0.05);
    const double sound = omega_r / ref::kCaradonnaTipMach;
    EXPECT_GT(sound, 335.0);
    EXPECT_LT(sound, 347.0);
    EXPECT_NEAR(ref::kCaradonnaRadiusM / ref::kCaradonnaAspectRatio, 0.1905, 5e-5);
    EXPECT_NEAR(ref::kCaradonnaThrustCoefficientFig4, ref::kCaradonnaThrustCoefficientFig6, 1.5e-5)
        << "figures 4 and 6 print the run's C_T a digit apart";
}

// Each station's pressures integrate to near its printed C_L, a check on the
// reading against the report's own integral: the normal force over the
// chord, -C_p's upper integral less its lower, each surface held at its end
// taps out to the leading edge and 0 at the trailing edge, by trapezoids.
// The report's own scheme is not printed, so the band is wide: 10%, where
// the reading measured 0.925 to 1.055 (Test/Docs, 2026-11-03). A misread
// leading digit moves a station well past it.
TEST(AirflowCaradonnaData, ThePressuresIntegrateNearEachStationsLift) {
    const auto integral = [](std::span<const ref::CaradonnaTap> side) {
        std::vector<double> x;
        std::vector<double> v;
        if (side.front().x_c > 0.0) {
            x.push_back(0.0);
            v.push_back(side.front().minus_cp);
        }
        for (const ref::CaradonnaTap& t : side) {
            x.push_back(t.x_c);
            v.push_back(t.minus_cp);
        }
        x.push_back(1.0);
        v.push_back(0.0);
        double sum = 0.0;
        for (std::size_t i = 1; i < x.size(); ++i) sum += 0.5 * (v[i] + v[i - 1]) * (x[i] - x[i - 1]);
        return sum;
    };
    for (const ref::CaradonnaStation& st : ref::kCaradonnaTable17) {
        const double c_n = integral(st.upper) - integral(st.lower);
        EXPECT_NEAR(c_n / st.c_l, 1.0, 0.10) << "r/R " << st.r_over_R << ": " << c_n << " against " << st.c_l;
    }
}

// Figure 9's points: ages rising within the plot's 0 to 450 deg, the vortex
// descending and contracting with age, each inside the plot's ranges.
TEST(AirflowCaradonnaData, TheVortexPathDescendsAndContracts) {
    for (const std::span<const ref::CaradonnaVortexPoint> path :
         {std::span<const ref::CaradonnaVortexPoint>(ref::kCaradonnaVortexDescent),
          std::span<const ref::CaradonnaVortexPoint>(ref::kCaradonnaVortexRadius)}) {
        for (std::size_t i = 0; i < path.size(); ++i) {
            EXPECT_GT(path[i].age_deg, 0.0);
            EXPECT_LT(path[i].age_deg, 450.0);
            if (i > 0) {
                EXPECT_GT(path[i].age_deg, path[i - 1].age_deg);
            }
        }
    }
    for (std::size_t i = 1; i < std::size(ref::kCaradonnaVortexDescent); ++i) {
        EXPECT_GT(ref::kCaradonnaVortexDescent[i].value, ref::kCaradonnaVortexDescent[i - 1].value);
    }
    for (std::size_t i = 1; i < std::size(ref::kCaradonnaVortexRadius); ++i) {
        EXPECT_LT(ref::kCaradonnaVortexRadius[i].value, ref::kCaradonnaVortexRadius[i - 1].value);
    }
    for (const ref::CaradonnaVortexPoint& p : ref::kCaradonnaVortexDescent) {
        EXPECT_GE(p.value, 0.0);
        EXPECT_LE(p.value, 0.4);
    }
    for (const ref::CaradonnaVortexPoint& p : ref::kCaradonnaVortexRadius) {
        EXPECT_GE(p.value, 0.8);
        EXPECT_LE(p.value, 1.0);
    }
}

}  // namespace
