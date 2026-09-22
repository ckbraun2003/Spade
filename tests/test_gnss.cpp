// ---------------------------------------------------------------------------
// test_gnss.cpp -- the GNSS row, its layout, and the kind vocabulary that lets
// a second sensor exist at all.
//
// ⛔ WHY THIS FILE EXISTS AT ALL, AND IT IS NOT "COVERAGE". sensors/gnss.hpp is
// a header. Its static_asserts ARE its layout test -- the 96-byte stride, every
// offset, the no-implicit-padding sum -- and a static_assert in a header that no
// translation unit includes NEVER FIRES. On 2026-09-21 a build of spade_tests
// reached 40 of 47 targets with zero compiler errors and did not compile
// gnss.hpp once, because nothing included it.
//
// ⭐⭐⭐ A HEADER THAT NOTHING INCLUDES IS NOT COMPILED BY A BUILD OF EVERYTHING.
//
// So the first job of this file is the `#include` below. Everything after it is
// the second job.
//
// AND THE LAYOUT CASES ARE DELIBERATELY RUNTIME EXPECTs OVER THE SAME FACTS the
// static_asserts already pin. That is not redundancy. A compile-time assert
// proves the layout and then VANISHES: it leaves no name in `ctest -V`, so
// nothing downstream can say WHICH fact was checked, and a later edit that
// stops including this header would take the proof with it and report the same
// green. The runtime cases give the layout a NAME IN THE TEST LIST, which is
// the only form a pass count cannot silently satisfy.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sensors/kinds.hpp"

namespace {

using spade::sensors::GnssFix;
using spade::sensors::GnssSensorRow;
namespace kind = spade::sensors::sensor_kind;

// ---------------------------------------------------------------------------
// Layout. Every number here is also a static_assert in gnss.hpp; these exist so
// the facts have names (see the header comment above).
// ---------------------------------------------------------------------------

TEST(GnssLayout, FixMatchesTheStd430RowPlan) {
    EXPECT_EQ(sizeof(GnssFix), std::size_t{48}) << "std430 array stride";
    EXPECT_EQ(alignof(GnssFix), std::size_t{16}) << "std430 base alignment";

    // row 0  position | sigma_h   row 1  velocity | sigma_v   row 2  index | tick
    EXPECT_EQ(offsetof(GnssFix, position), std::size_t{0});
    EXPECT_EQ(offsetof(GnssFix, sigma_h), std::size_t{12});
    EXPECT_EQ(offsetof(GnssFix, velocity), std::size_t{16});
    EXPECT_EQ(offsetof(GnssFix, sigma_v), std::size_t{28});
    EXPECT_EQ(offsetof(GnssFix, index), std::size_t{32});
    EXPECT_EQ(offsetof(GnssFix, tick), std::size_t{40});

    // Each vec3 starts a 16-byte row; each uint64 is 8-byte aligned.
    EXPECT_EQ(offsetof(GnssFix, position) % 16, std::size_t{0});
    EXPECT_EQ(offsetof(GnssFix, velocity) % 16, std::size_t{0});
    EXPECT_EQ(offsetof(GnssFix, index) % 8, std::size_t{0});

    // Every byte belongs to a named field -- what makes a ring snapshot
    // comparable with memcmp.
    EXPECT_EQ(sizeof(GnssFix::position) + sizeof(GnssFix::sigma_h) +
                  sizeof(GnssFix::velocity) + sizeof(GnssFix::sigma_v) +
                  sizeof(GnssFix::index) + sizeof(GnssFix::tick),
              sizeof(GnssFix))
        << "GnssFix has implicit padding";
}

TEST(GnssLayout, SensorRowMatchesTheStd430RowPlan) {
    EXPECT_EQ(sizeof(GnssSensorRow), std::size_t{112}) << "std430 array stride";
    EXPECT_EQ(alignof(GnssSensorRow), std::size_t{16}) << "std430 base alignment";

    EXPECT_EQ(offsetof(GnssSensorRow, body_slot), std::size_t{0});
    EXPECT_EQ(offsetof(GnssSensorRow, kind), std::size_t{4});
    EXPECT_EQ(offsetof(GnssSensorRow, rate_divider), std::size_t{8});
    EXPECT_EQ(offsetof(GnssSensorRow, phase), std::size_t{12});
    EXPECT_EQ(offsetof(GnssSensorRow, mount_pos), std::size_t{16});
    EXPECT_EQ(offsetof(GnssSensorRow, bias), std::size_t{32});
    EXPECT_EQ(offsetof(GnssSensorRow, sigma_h), std::size_t{48});
    EXPECT_EQ(offsetof(GnssSensorRow, sigma_v), std::size_t{52});
    EXPECT_EQ(offsetof(GnssSensorRow, sigma_vel), std::size_t{56});
    EXPECT_EQ(offsetof(GnssSensorRow, bias_tau_s), std::size_t{60});
    EXPECT_EQ(offsetof(GnssSensorRow, noise), std::size_t{64});
    EXPECT_EQ(offsetof(GnssSensorRow, last_index), std::size_t{80});

    // THE APPEND. Every pin above is unchanged from the 96-byte row this grew
    // out of; the three fields below were added at the tail when the row went
    // to 112, which is what makes it an APPEND rather than a reshuffle. If any
    // pin ABOVE this comment ever moves, that is a different and much more
    // expensive kind of change -- every committed digest moves with it and no
    // continuation argument survives.
    EXPECT_EQ(offsetof(GnssSensorRow, bias_retention), std::size_t{88});
    EXPECT_EQ(offsetof(GnssSensorRow, bias_drive), std::size_t{92});
    EXPECT_EQ(offsetof(GnssSensorRow, sigma_bias), std::size_t{96});
    EXPECT_EQ(offsetof(GnssSensorRow, _reserved0), std::size_t{104});

    // `kind` sits at offset 4 on BOTH row types, which is what lets a reader
    // identify a row before it knows which row it is holding.
    EXPECT_EQ(offsetof(GnssSensorRow, kind), offsetof(spade::sensors::ImuSensorRow, kind))
        << "the kind tag must sit at the same offset in every sensor row";
}

TEST(GnssLayout, TheRowIsNotACopyOfTheImuRow) {
    // A row that came out the same size as the IMU's would be a sign it had
    // been copied rather than designed. This is the assertion that says so out
    // loud: GNSS carries no mount orientation (an antenna is a point) and one
    // bias vector instead of two.
    EXPECT_NE(sizeof(GnssSensorRow), sizeof(spade::sensors::ImuSensorRow));
    EXPECT_LT(sizeof(GnssSensorRow), sizeof(spade::sensors::ImuSensorRow));
}

// ---------------------------------------------------------------------------
// The kind vocabulary.
// ---------------------------------------------------------------------------

TEST(SensorKind, GnssAndImuAreDistinctLiveKinds) {
    EXPECT_NE(kind::gnss, kind::imu);
    EXPECT_NE(kind::gnss, kind::none);
    EXPECT_TRUE(kind::is_live(kind::imu));
    EXPECT_TRUE(kind::is_live(kind::gnss));
    EXPECT_FALSE(kind::is_live(kind::none)) << "none is an inert slot, not a sensor";
}

TEST(SensorKind, UnknownTagsAreNeitherKnownNorLive) {
    EXPECT_TRUE(kind::is_known(kind::none));
    EXPECT_TRUE(kind::is_known(kind::imu));
    EXPECT_TRUE(kind::is_known(kind::gnss));

    // `count` is the first value no kind uses -- a bound, not a kind.
    EXPECT_FALSE(kind::is_known(kind::count));
    EXPECT_FALSE(kind::is_live(kind::count));
    EXPECT_FALSE(kind::is_known(0xFFFFFFFFu)) << "a row from a newer build is not interpretable";
}

TEST(SensorKind, PollPermitsNoneButRejectsAnotherKnownKind) {
    // ⭐ THIS IS THE PREDICATE THE ARENA DID NOT HAVE, and the asymmetry is the
    // whole point. `none` MUST stay legal: a row whose init is still queued has
    // kind == none and last_index == 0, and polling a sensor you just added,
    // before the next step, is documented-legal and reports "nothing yet"
    // (sim/simulation.cpp, poll_imu). A validator that rejected `none` would
    // break that case.
    EXPECT_TRUE(kind::poll_permits(kind::none, kind::imu)) << "queued init must stay pollable";
    EXPECT_TRUE(kind::poll_permits(kind::none, kind::gnss));

    // An exact match is obviously fine.
    EXPECT_TRUE(kind::poll_permits(kind::imu, kind::imu));
    EXPECT_TRUE(kind::poll_permits(kind::gnss, kind::gnss));

    // And the case that matters: a DIFFERENT known kind. The slot is allocated,
    // the partition agrees, the ref is valid -- and the bytes belong to another
    // row type. That is a type confusion reached through a valid ref, and it is
    // the thing a second sensor kind makes possible for the first time.
    EXPECT_FALSE(kind::poll_permits(kind::gnss, kind::imu))
        << "polling a GNSS row as an IMU must be refused";
    EXPECT_FALSE(kind::poll_permits(kind::imu, kind::gnss))
        << "polling an IMU row as a GNSS must be refused";
}

// ---------------------------------------------------------------------------
// The only behaviour leg 1 has.
// ---------------------------------------------------------------------------

TEST(GnssBiasRetention, NonPositiveTauOrDtGivesZero) {
    // tau <= 0 means "no correlated bias": the factor is 0 and the bias is
    // redrawn white each fix. Total rather than rejecting, so a pass never has
    // to branch on a validation the spawn path already did.
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(0.1f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(0.1f, -1.0f), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(0.0f, 60.0f), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(-0.1f, 60.0f), 0.0f);
}

TEST(GnssBiasRetention, AtOneTimeConstantItIsExpMinusOne) {
    // The defining point of a first-order Gauss-Markov decay: after one
    // correlation time, what remains is 1/e.
    const float k = spade::sensors::gnss_bias_retention(60.0f, 60.0f);
    EXPECT_NEAR(k, std::exp(-1.0f), 1e-6f);
}

TEST(GnssBiasRetention, DecaysMonotonicallyAndStaysInTheOpenUnitInterval) {
    constexpr float kTau = 120.0f;

    // Anti-vacuity first: the inputs must actually produce a live factor, or
    // the monotonicity below would hold trivially over a run of zeros.
    const float first = spade::sensors::gnss_bias_retention(1.0f, kTau);
    ASSERT_GT(first, 0.0f) << "a 1 s step at tau=120 s must retain almost everything";
    ASSERT_LT(first, 1.0f) << "and must not retain all of it";

    float previous = first;
    for (float dt = 2.0f; dt <= 600.0f; dt += 37.0f) {
        const float k = spade::sensors::gnss_bias_retention(dt, kTau);
        EXPECT_GT(k, 0.0f) << "dt=" << dt;
        EXPECT_LT(k, previous) << "retention must fall as the step grows; dt=" << dt;
        previous = k;
    }
    EXPECT_LT(previous, first) << "the sweep must have moved";
}

// ---------------------------------------------------------------------------
// The property the header asserts in prose, asserted here in code.
// ---------------------------------------------------------------------------

TEST(GnssNoise, TheDomainTagSeparatesCoLocatedSensors) {
    // An IMU and a receiver mounted on the same body can land on the same
    // world-local slot index in their own arenas. If they derived their streams
    // the same way they would draw THE SAME NUMBERS, and the two sensors' errors
    // would be perfectly correlated -- which would look like a plausible
    // simulation and be silently wrong. The domain tag is what keeps them apart,
    // not the derivation, which is deliberately identical.
    constexpr uint64_t kSeed = 0x9E3779B97F4A7C15ull;

    EXPECT_NE(spade::sensors::kGnssNoiseDomainTag, spade::sensors::kImuNoiseDomainTag);

    for (uint32_t slot = 0; slot < 8; ++slot) {
        const auto g = spade::sensors::gnss_noise_stream(kSeed, slot);
        const auto i = spade::sensors::imu_noise_stream(kSeed, slot);
        EXPECT_NE(g.state, i.state) << "co-located sensors share a draw at slot " << slot;
    }

    // And two receivers in the same world must differ from each other too.
    EXPECT_NE(spade::sensors::gnss_noise_stream(kSeed, 0).state,
              spade::sensors::gnss_noise_stream(kSeed, 1).state);
}

}  // namespace
