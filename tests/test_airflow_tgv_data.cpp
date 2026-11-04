// C0b's reference data (tests/data/airflow/tgv1600_spectral.hpp): the
// Taylor-Green vortex at Re 1600, UCL's 512^3 pseudo-spectral solution from
// the High-Order CFD Workshop's case C3.5. Nothing reads it before L1's C0b;
// these pin the transcription to the file's own relations. Host only,
// milliseconds.
#include <gtest/gtest.h>

#include <cstddef>
#include <iterator>

#include "data/airflow/tgv1600_spectral.hpp"

namespace {

namespace ref = spade::testing::airflow_reference;

TEST(AirflowTaylorGreenData, TwoThousandSamplesStartFromTheFieldsEnergy) {
    static_assert(std::size(ref::kTgv1600KineticEnergy) == ref::kTgv1600Samples);
    static_assert(std::size(ref::kTgv1600Dissipation) == ref::kTgv1600Samples);
    // The field's mean of |u|^2 / 2 is 1/8, and at t* = 0 its dissipation is
    // 2 nu times its enstrophy 3/8, with nu = 1/1600: 4.6875e-4.
    EXPECT_EQ(ref::kTgv1600KineticEnergy[0], 0.125);
    EXPECT_NEAR(ref::kTgv1600Dissipation[0], 2.0 * 0.375 / 1600.0, 1e-7);
    for (int k = 1; k < ref::kTgv1600Samples; ++k) {
        EXPECT_LT(ref::kTgv1600KineticEnergy[k], ref::kTgv1600KineticEnergy[k - 1]) << "sample " << k;
        EXPECT_GT(ref::kTgv1600Dissipation[k], 0.0) << "sample " << k;
    }
}

// The file's eps is its own central difference of E_k: a digit dropped or
// moved in either column breaks it at that sample.
TEST(AirflowTaylorGreenData, TheDissipationIsTheEnergysCentralDifference) {
    for (int k = 1; k + 1 < ref::kTgv1600Samples; ++k) {
        const double central = -(ref::kTgv1600KineticEnergy[k + 1] - ref::kTgv1600KineticEnergy[k - 1]) /
                               (2.0 * ref::kTgv1600Dt);
        EXPECT_NEAR(ref::kTgv1600Dissipation[k], central, 1e-9) << "sample " << k;
    }
}

// The peak spec §11.2 calls "about 0.0127 near t* 9": the file's own.
TEST(AirflowTaylorGreenData, ThePeakIsAtEightPointNineSeven) {
    std::size_t peak = 0;
    for (std::size_t k = 1; k < std::size(ref::kTgv1600Dissipation); ++k) {
        if (ref::kTgv1600Dissipation[k] > ref::kTgv1600Dissipation[peak]) peak = k;
    }
    EXPECT_EQ(peak, 897u);
    EXPECT_EQ(ref::kTgv1600Dissipation[peak], 0.012857528187);
}

}  // namespace
