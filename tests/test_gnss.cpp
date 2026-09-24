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
#include <limits>
#include <memory>
#include <type_traits>
#include <span>
#include <string_view>
#include <vector>

#include <glm/vec3.hpp>

#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sensors/kinds.hpp"
#include "sensors/rings.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"

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
    // ⛔ ERRATUM. This comment used to end "...and the bias is redrawn white
    // each fix", which is FALSE and was false from the moment gnss_bias_drive()
    // was added. It describes a receiver whose bias survives as white noise.
    // There is no such receiver: gnss_bias_drive() carries the SAME guard (see
    // its header, "SAME DOMAIN CONTRACT ... deliberately"), so tau <= 0 zeroes
    // BOTH coefficients and sensors/gnss.cpp:137 collapses to
    // `bias = 0 * 0 + 0 * walk`. THE BIAS IS IDENTICALLY ZERO FOREVER, which
    // gnss.cpp's own comment states correctly seven lines above that line.
    //
    // ⭐ IT WAS TRUE WHEN IT WAS WRITTEN. GnssSensorRow shipped with a
    // correlation time and no magnitude (gnss.hpp says so), so back then the
    // only thing left at tau <= 0 really was the white position/velocity noise.
    // Adding the drive coefficient falsified this sentence and touched neither
    // the test's name nor its four assertions -- all still correct -- so
    // nothing in the suite could notice. A COMMENT THAT WAS TRUE WHEN WRITTEN,
    // IN A TEST WHOSE ASSERTIONS NEVER CHANGED, HAS NO FAILURE MODE.
    //
    // ⛔⛔ AND IT WAS LOAD-BEARING. A GPU-parity discriminator for the GNSS
    // bias divergence was designed directly on this sentence -- set tau <= 0 to
    // "break the recursion and keep the draws", then compare bias at a zero
    // band. That design DELETES THE QUANTITY instead of breaking the mechanism:
    // both backends hold an identically-zero bias, the comparison is bit-exact,
    // and the run reports success having proved nothing. A DISCRIMINATOR BUILT
    // ON A FALSE MECHANISM FAILS GREEN, which is the one way a falsifier can be
    // worse than absent. The working design is the underflow route, pinned
    // below by CollapsesToSigmaBiasWhenRetentionUnderflows.
    //
    // Total rather than rejecting, so a pass never has to branch on a
    // validation the spawn path already did.
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(0.1f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(0.1f, -1.0f), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(0.0f, 60.0f), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(-0.1f, 60.0f), 0.0f);
}

// ---------------------------------------------------------------------------
// THE OTHER HALF OF THE CONTRACT, WHICH HAD NO TEST OF ITS OWN.
//
// gnss_bias_drive() was called from this file twice before this block, both
// times as an ORACLE: `EXPECT_FLOAT_EQ(row->bias_drive, gnss_bias_drive(...))`
// compares a stored field against the function that computed it, which pins the
// PLUMBING (the fix clock reached the right field) and cannot pin the FUNCTION.
// Nothing asserted any property of the value. So the half of the Gauss-Markov
// pair that silently deleted a discriminator was the half with no behaviour
// test -- and the pair's agreement, the thing that actually bites, lived only
// in a header sentence.
// ---------------------------------------------------------------------------

TEST(GnssBiasDrive, NonPositiveTauOrDtGivesZero) {
    // sigma_bias is deliberately LARGE and positive in every case: a zero sigma
    // would return 0 through the multiply rather than through the guard, and
    // the test could not tell those apart.
    constexpr float kSigma = 0.8f;
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_drive(0.1f, 0.0f, kSigma), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_drive(0.1f, -1.0f, kSigma), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_drive(0.0f, 60.0f, kSigma), 0.0f);
    EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_drive(-0.1f, 60.0f, kSigma), 0.0f);

    // ANTI-VACUITY: a drive that returned 0 for every input would pass all four.
    EXPECT_GT(spade::sensors::gnss_bias_drive(0.1f, 60.0f, kSigma), 0.0f)
        << "a live (dt, tau) must produce a live drive, or the four zeros above "
        << "are satisfied by a function that always returns zero";
}

TEST(GnssBiasCoefficients, BothHalvesAgreeOnTheDegenerateDomain) {
    // gnss.hpp states this as prose -- "both coefficients must agree about that
    // or a receiver could be driven without decaying" -- and until this test
    // nothing enforced it. The ASYMMETRIC failure is the dangerous one: a drive
    // that survived a domain the retention rejected would give a bias driven by
    // white noise and never decaying, i.e. a random walk with no stationary
    // distribution. That is not the documented model, and it would look like
    // plausible receiver noise in every plot anyone drew of it.
    constexpr float kSigma = 0.8f;
    struct Case {
        float dt;
        float tau;
        const char* what;
    };
    constexpr Case kDegenerate[] = {
        {0.1f, 0.0f, "tau zero"},   {0.1f, -1.0f, "tau negative"},
        {0.0f, 60.0f, "dt zero"},   {-0.1f, 60.0f, "dt negative"},
        {0.0f, 0.0f, "both zero"},
    };

    for (const Case& c : kDegenerate) {
        EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_retention(c.dt, c.tau), 0.0f) << c.what;
        EXPECT_FLOAT_EQ(spade::sensors::gnss_bias_drive(c.dt, c.tau, kSigma), 0.0f)
            << c.what << ": the drive survived a domain the retention rejected -- "
            << "the bias would be driven without ever decaying";
    }
}

TEST(GnssBiasDrive, CollapsesToSigmaBiasWhenRetentionUnderflows) {
    // ⭐ THIS IS THE GPU-PARITY DISCRIMINATOR'S PREMISE, PINNED HERE SO IT IS A
    // TESTED PROPERTY RATHER THAN AN ARGUMENT IN A DESIGN NOTE.
    //
    // A tau far BELOW the fix interval -- rather than at or below zero -- drives
    // the exponent past core/fp32_math.cpp's explicit `if (x <= -104.0f) return
    // 0.0f;`, so the retention underflows to exactly zero WITHOUT tripping the
    // domain guard that would also kill the drive. Then variance == 1 - 0 == 1,
    // sqrt(1) == 1 exactly, and the drive is sigma_bias exactly.
    //
    // WHY THAT MATTERS: the bias advance collapses to `bias = sigma_bias * walk`
    // -- a single multiply by a byte-identical constant, which cannot introduce
    // a cross-backend difference. So under this configuration `bias` is
    // bit-exact IF AND ONLY IF every gaussian draw was bit-exact, which is the
    // biconditional a parity discriminator needs. Set tau <= 0 instead and both
    // coefficients vanish, the bias is identically zero on both sides, and the
    // comparison proves nothing while reporting success.
    //
    // EXACT EQUALITY, NOT EXPECT_FLOAT_EQ: the 4-ULP latitude would hide exactly
    // the property being claimed. `sigma_bias * 1.0f` is sigma_bias, or the
    // claim is false.
    constexpr float kSigma = 0.8f;
    constexpr float kFixDt = 1.0e-3f;
    constexpr float kTinyTau = 1.0e-6f;

    EXPECT_EQ(spade::sensors::gnss_bias_retention(kFixDt, kTinyTau), 0.0f)
        << "the retention must underflow to exactly zero, or the collapse below "
        << "is not the one being claimed";
    EXPECT_EQ(spade::sensors::gnss_bias_drive(kFixDt, kTinyTau, kSigma), kSigma)
        << "the drive must be sigma_bias EXACTLY; if fp32_math's -104 cutoff has "
        << "moved, the parity discriminator built on this collapse is no longer "
        << "valid and must be re-derived, NOT re-banded";

    // ANTI-VACUITY: the collapse must be a property of this CONFIGURATION and
    // not of the function. At a realistic tau the drive is strictly below
    // sigma_bias, because the variance is strictly below 1.
    const float ordinary = spade::sensors::gnss_bias_drive(kFixDt, 60.0f, kSigma);
    EXPECT_GT(ordinary, 0.0f);
    EXPECT_LT(ordinary, kSigma)
        << "at a realistic tau the drive must be strictly under sigma_bias, or "
        << "the collapse above is not telling the two regimes apart";
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


// ===========================================================================
// THE LIFECYCLE. Everything above this line tests the HEADER; everything below
// drives a real Simulation, because a header test cannot tell you whether the
// structural queue ever writes the row it describes.
//
// ⭐ WHY THIS SECTION EXISTS AT ALL, stated so it is not mistaken for routine
// coverage: before add_gnss_sensor() there was NO WAY TO MAKE A GNSS ROW LIVE,
// so every row in every scenario had kind == none, both backends' synthesis
// returned immediately, and the CPU/GPU parity band proved NOTHING about GNSS.
// The green was a filter matching nothing. These cases are the first thing in
// the tree that puts a live row in front of the code that reads it.
// ===========================================================================

using spade::Simulation;
using spade::GnssSensorRef;
using spade::GnssSensorSpawn;
using spade::GnssPoll;
using spade::ImuSensorSpawn;
using spade::BodyRef;
using spade::BodySpawn;
using spade::Capacities;
using spade::Environment;
using spade::TurbulenceLevel;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code)
                                       << "] " << r.error().context;
}

#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// -1 stands for "it succeeded": calling .error() on a Result holding a value is
// UB, so a negative test that accidentally passes must not reach for it.
template <class T>
[[nodiscard]] int code_of(const spade::Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}

[[nodiscard]] constexpr int code(spade::Code c) { return static_cast<int>(c); }

[[nodiscard]] spade::Result<WorldSetDesc> void_world_set(uint32_t sensors) {
    Environment env;
    env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;

    Capacities caps;
    caps.bodies = 4;
    caps.force_elements = 1;
    caps.sensors = sensors;
    caps.contacts = 1;

    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());

    spade::physics::ContactParams contacts;
    contacts.restitution_e = 0.0f;
    contacts.friction_mu = 0.0f;
    contacts.proxy_radius = 0.0f;

    spade::physics::GridParams grid;
    grid.cell_size = 1.0f;

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0x5EEDu;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts = contacts;
    instance.grid = grid;
    return WorldSetDesc{{instance}};
}

[[nodiscard]] BodySpawn unit_body() {
    BodySpawn b;
    b.mass = 1.0f;
    b.inv_inertia_diag = glm::vec3(1.0f);
    return b;
}

// A receiver with every field distinct and non-default, so a field-wise write
// that drops one is visible rather than absorbed by a zero.
[[nodiscard]] GnssSensorSpawn distinct_receiver() {
    GnssSensorSpawn g;
    g.mount_pos = glm::vec3(0.25f, -0.5f, 0.75f);
    g.rate_divider = 200u;
    g.sigma_h = 1.5f;
    g.sigma_v = 2.5f;
    g.sigma_vel = 0.125f;
    g.bias_tau_s = 60.0f;
    g.sigma_bias = 0.8f;
    return g;
}

// ---------------------------------------------------------------------------
// ⛔⛔ THE ONE TEST THAT ENCODES THE CLOCK. Nothing else in this suite can.
//
// The Gauss-Markov bias advances ONCE PER EMITTED FIX, so the coefficients are
// derived from rate_divider * substep_h(), not substep_h(). A receiver built on
// the substep clock at rate_divider = 200 decays 200x too slowly AND EVERY
// OTHER INSTRUMENT STAYS GREEN: both backends agree bit for bit, every digest
// is self-consistent, and the corpus pins the wrong number forever.
//
//   A CONSTANT COMPUTED FROM THE WRONG CLOCK IS STILL DETERMINISTIC, AND
//   DETERMINISM IS WHAT THIS SUITE CHECKS.
//
// ⭐ THE NEGATIVE HALF IS THE WHOLE TEST. Asserting the right value alone would
// pass for a implementation that happened to agree at rate_divider == 1; the
// EXPECT_NE against the substep-clock value is what makes this a discriminator
// rather than a restatement, and it is why rate_divider is 200 and not 1.
// ---------------------------------------------------------------------------
TEST(GnssLifecycle, TheBiasCoefficientsAreOnTheFixClockNotTheSubstepClock) {
    const spade::Result<WorldSetDesc> set = void_world_set(2);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    const GnssSensorSpawn desc = distinct_receiver();
    const spade::Result<GnssSensorRef> ref = sim->add_gnss_sensor(*body, desc);
    ASSERT_OK(ref);
    ASSERT_OK(sim->flush_structural());

    const spade::Result<const spade::sensors::GnssSensorRow*> row = sim->gnss_sensor(*ref);
    ASSERT_OK(row);

    const float h = sim->substep_h();
    const float fix_dt = static_cast<float>(desc.rate_divider) * h;

    EXPECT_FLOAT_EQ((*row)->bias_retention,
                    spade::sensors::gnss_bias_retention(fix_dt, desc.bias_tau_s));
    EXPECT_FLOAT_EQ((*row)->bias_drive,
                    spade::sensors::gnss_bias_drive(fix_dt, desc.bias_tau_s, desc.sigma_bias));

    // THE DISCRIMINATOR. If either coefficient were built on the substep clock
    // the two assertions above would still be the only ones anyone wrote, and
    // they would be written against whatever the code produced.
    EXPECT_NE((*row)->bias_retention, spade::sensors::gnss_bias_retention(h, desc.bias_tau_s))
        << "bias_retention was computed on the SUBSTEP clock; it must be the FIX clock "
           "(rate_divider * substep_h()) -- see add_gnss_sensor's contract";
    EXPECT_NE((*row)->bias_drive,
              spade::sensors::gnss_bias_drive(h, desc.bias_tau_s, desc.sigma_bias))
        << "bias_drive was computed on the SUBSTEP clock; it must be the FIX clock";

    // And the premise the discriminator rests on: the two clocks must actually
    // differ here, or both EXPECT_NEs are vacuous.
    ASSERT_GT(desc.rate_divider, 1u) << "a rate_divider of 1 makes the two clocks equal and "
                                        "this test unable to tell them apart";
}

// ===========================================================================
// THE REF TYPES ARE MUTUALLY UNASSIGNABLE, AND THIS IS A PROOF RATHER THAN A
// REVIEW. Compile-time, so it adds no ctest slot and cannot be skipped.
//
// WHY THIS EXISTS: GnssSensorRef and ImuSensorRef are structurally identical --
// a world index and a global slot -- and index DIFFERENT ARRAYS. An IMU ref
// passed to poll_gnss() would name a real, allocated slot in the GNSS
// partition: a type confusion reached through a perfectly VALID ref, which no
// runtime validation can catch because there is nothing wrong with the value.
// Only the type can. The dedup that shares poll_sensor<Row, Sample> between the
// two families is exactly the change that could take this away while looking
// like a tidy-up, so the guarantee is asserted rather than trusted.
// ===========================================================================

// POSITIVE CONTROL FIRST. is_invocable_v is false for a signature that is wrong
// in ANY way, so without this the negative assertions below would pass just as
// happily if the whole expression were malformed -- green for the wrong reason,
// which is the failure this file already catches in three other places.
static_assert(std::is_invocable_v<decltype(&Simulation::poll_gnss), const Simulation&,
                                  GnssSensorRef, spade::sensors::SampleIndex,
                                  std::span<spade::sensors::GnssFix>>,
              "POSITIVE CONTROL FAILED: the matching call does not compile either, so the "
              "negative assertions below prove nothing about ref distinctness");
static_assert(std::is_invocable_v<decltype(&Simulation::poll_imu), const Simulation&,
                                  spade::ImuSensorRef, spade::sensors::SampleIndex,
                                  std::span<spade::sensors::ImuSample>>,
              "POSITIVE CONTROL FAILED: see above");

// THE GUARANTEE. Not "the types differ" -- THE CALL IS ILL-FORMED. A
// !is_convertible check alone would pass while an implicit conversion through
// some third type kept the call legal.
static_assert(!std::is_invocable_v<decltype(&Simulation::poll_gnss), const Simulation&,
                                   spade::ImuSensorRef, spade::sensors::SampleIndex,
                                   std::span<spade::sensors::GnssFix>>,
              "an ImuSensorRef compiles against poll_gnss -- the ref types have stopped being "
              "distinct, and a type confusion through a VALID ref is now reachable");
static_assert(!std::is_invocable_v<decltype(&Simulation::poll_imu), const Simulation&,
                                   GnssSensorRef, spade::sensors::SampleIndex,
                                   std::span<spade::sensors::ImuSample>>,
              "a GnssSensorRef compiles against poll_imu -- see above");

// And the weaker properties, kept because they localise a failure: if these
// fire, the cause is the TYPES; if only the is_invocable pair fires, the cause
// is an overload or a conversion somewhere else.
static_assert(!std::is_same_v<spade::ImuSensorRef, GnssSensorRef>);
static_assert(!std::is_convertible_v<spade::ImuSensorRef, GnssSensorRef>);
static_assert(!std::is_convertible_v<GnssSensorRef, spade::ImuSensorRef>);

// ---------------------------------------------------------------------------
// The two-phase shape, in both directions: inert before the boundary, live
// after it, and polling in between is LEGAL rather than an error.
// ---------------------------------------------------------------------------
TEST(GnssLifecycle, ASpawnedReceiverIsInertUntilTheBoundary) {
    const spade::Result<WorldSetDesc> set = void_world_set(1);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    const spade::Result<GnssSensorRef> ref = sim->add_gnss_sensor(*body, distinct_receiver());
    ASSERT_OK(ref);

    {
        const spade::Result<const spade::sensors::GnssSensorRow*> row = sim->gnss_sensor(*ref);
        ASSERT_OK(row);
        EXPECT_EQ((*row)->kind, kind::none) << "a reserved row must be inert to the synthesis pass";
    }
    {
        // Polling a receiver you just added, before the next step, is legal and
        // reports nothing -- the same contract poll_imu states, and the reason
        // poll_permits() lets `none` through.
        std::vector<spade::sensors::GnssFix> buffer(spade::sensors::kRingDepth);
        const spade::Result<GnssPoll> poll = sim->poll_gnss(*ref, 0, buffer);
        ASSERT_OK(poll);
        EXPECT_TRUE(poll->samples.empty());
        EXPECT_EQ(poll->dropped, 0u);
    }

    ASSERT_OK(sim->flush_structural());

    const spade::Result<const spade::sensors::GnssSensorRow*> row = sim->gnss_sensor(*ref);
    ASSERT_OK(row);
    EXPECT_EQ((*row)->kind, kind::gnss);
}

// ---------------------------------------------------------------------------
// Every spawn field reaches the row. Written with all-distinct inputs so a
// dropped field shows up rather than being absorbed by a matching zero.
// ---------------------------------------------------------------------------
TEST(GnssLifecycle, TheRowIsWrittenFieldWiseFromTheSpawn) {
    const spade::Result<WorldSetDesc> set = void_world_set(1);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    const GnssSensorSpawn desc = distinct_receiver();
    const spade::Result<GnssSensorRef> ref = sim->add_gnss_sensor(*body, desc);
    ASSERT_OK(ref);
    ASSERT_OK(sim->flush_structural());

    const spade::Result<const spade::sensors::GnssSensorRow*> row = sim->gnss_sensor(*ref);
    ASSERT_OK(row);

    EXPECT_EQ((*row)->body_slot, 0u);
    EXPECT_EQ((*row)->rate_divider, desc.rate_divider);
    EXPECT_EQ((*row)->phase, 0u);
    EXPECT_EQ((*row)->mount_pos, desc.mount_pos);
    EXPECT_FLOAT_EQ((*row)->sigma_h, desc.sigma_h);
    EXPECT_FLOAT_EQ((*row)->sigma_v, desc.sigma_v);
    EXPECT_FLOAT_EQ((*row)->sigma_vel, desc.sigma_vel);
    EXPECT_FLOAT_EQ((*row)->bias_tau_s, desc.bias_tau_s);
    EXPECT_FLOAT_EQ((*row)->sigma_bias, desc.sigma_bias);
    EXPECT_EQ((*row)->bias, glm::vec3(0.0f));
    EXPECT_EQ((*row)->last_index, 0u);
    // The growth slot must stay zero or a later versioned field inherits junk.
    EXPECT_EQ((*row)->_reserved0, 0u);
    // The stream is the world's seed and the WORLD-LOCAL slot, not the global.
    EXPECT_EQ((*row)->noise.state, spade::sensors::gnss_noise_stream(0x5EEDu, 0).state);
}

// ---------------------------------------------------------------------------
// ⭐ THE NON-VACUOUS POPULATION. A floor proves the loop ran; this proves a
// member of the population exists at all -- a live row that actually emits.
// Without it every other GNSS assertion in the tree is about an empty set.
// ---------------------------------------------------------------------------
TEST(GnssEmission, ALiveReceiverEmitsOnItsOwnClockAndTheIndicesAreMonotonic) {
    const spade::Result<WorldSetDesc> set = void_world_set(1);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    GnssSensorSpawn desc = distinct_receiver();
    desc.rate_divider = 10u;  // small enough that a short run emits several
    const spade::Result<GnssSensorRef> ref = sim->add_gnss_sensor(*body, desc);
    ASSERT_OK(ref);
    ASSERT_OK(sim->flush_structural());

    constexpr uint64_t kSteps = 100;
    for (uint64_t t = 0; t < kSteps; ++t) ASSERT_OK(sim->step(1));

    std::vector<spade::sensors::GnssFix> buffer(spade::sensors::kRingDepth);
    const spade::Result<GnssPoll> poll = sim->poll_gnss(*ref, 0, buffer);
    ASSERT_OK(poll);

    // THE FLOOR IS BELOW THE EXPECTED COUNT, NEVER EQUAL TO IT. A floor set
    // equal to the count is not a floor, it is a change-detector: it goes red
    // on every legitimate change to the run length and gets raised reflexively
    // until someone deletes it for noise. A floor BELOW asserts the MECHANISM
    // works; the exact count is asserted separately, below, where a change to
    // it is a real statement about the clock.
    ASSERT_GE(poll->samples.size(), 2u)
        << "the receiver emitted fewer than two fixes in " << kSteps
        << " substeps -- the synthesis pass is not reaching this row at all";

    // And the exact count, which IS the fix clock observed from the outside:
    // one fix every rate_divider substeps.
    EXPECT_EQ(poll->samples.size(), kSteps / desc.rate_divider);

    spade::sensors::SampleIndex previous = 0;
    for (const spade::sensors::GnssFix& fix : poll->samples) {
        EXPECT_GT(fix.index, previous) << "fix indices must increase strictly";
        previous = fix.index;
        EXPECT_FLOAT_EQ(fix.sigma_h, desc.sigma_h);
        EXPECT_FLOAT_EQ(fix.sigma_v, desc.sigma_v);
    }
    EXPECT_EQ(poll->dropped, 0u);
}

// ---------------------------------------------------------------------------
// Despawn cascades to the receiver AND clears its ring. The ring is a second,
// direct-indexed array that free_slot() does not touch, so this is the half
// that would rot silently.
// ---------------------------------------------------------------------------
TEST(GnssLifecycle, DespawningTheBodyFreesTheReceiverAndClearsItsRing) {
    const spade::Result<WorldSetDesc> set = void_world_set(1);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    GnssSensorSpawn desc = distinct_receiver();
    desc.rate_divider = 5u;
    const spade::Result<GnssSensorRef> ref = sim->add_gnss_sensor(*body, desc);
    ASSERT_OK(ref);
    ASSERT_OK(sim->flush_structural());
    for (uint64_t t = 0; t < 20; ++t) ASSERT_OK(sim->step(1));

    // PREMISE: there is something in the ring to clear. Without this the
    // post-despawn assertion passes over an already-empty ring.
    {
        std::vector<spade::sensors::GnssFix> buffer(spade::sensors::kRingDepth);
        const spade::Result<GnssPoll> poll = sim->poll_gnss(*ref, 0, buffer);
        ASSERT_OK(poll);
        ASSERT_FALSE(poll->samples.empty()) << "nothing was emitted, so clearing proves nothing";
    }

    const spade::Result<uint32_t> before = sim->live_gnss_sensor_count(0);
    ASSERT_OK(before);
    EXPECT_EQ(*before, 1u);

    ASSERT_OK(sim->despawn(*body));
    ASSERT_OK(sim->flush_structural());

    const spade::Result<uint32_t> after = sim->live_gnss_sensor_count(0);
    ASSERT_OK(after);
    EXPECT_EQ(*after, 0u);

    // The ref is dead: the slot is no longer allocated.
    std::vector<spade::sensors::GnssFix> buffer(spade::sensors::kRingDepth);
    EXPECT_EQ(code_of(sim->poll_gnss(*ref, 0, buffer)), code(spade::Code::not_found));
}

// ---------------------------------------------------------------------------
// Validation, including the NaN cases the `!(x >= 0)` spelling exists for. A
// NaN compares false against both `< 0` and `>= 0`, so the naive spelling
// ACCEPTS it and the row carries a NaN into every fix it ever produces.
// ---------------------------------------------------------------------------
TEST(GnssLifecycle, ValidationRejectsEachBadFieldIncludingNaN) {
    const spade::Result<WorldSetDesc> set = void_world_set(1);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const int invalid = code(spade::Code::invalid_argument);

    {
        GnssSensorSpawn g = distinct_receiver();
        g.rate_divider = 0u;
        EXPECT_EQ(code_of(sim->add_gnss_sensor(*body, g)), invalid) << "rate_divider 0";
    }
    {
        GnssSensorSpawn g = distinct_receiver();
        g.mount_pos = glm::vec3(0.0f, nan, 0.0f);
        EXPECT_EQ(code_of(sim->add_gnss_sensor(*body, g)), invalid) << "non-finite mount_pos";
    }
    for (int field = 0; field < 5; ++field) {
        for (float bad : {-1.0f, nan}) {
            GnssSensorSpawn g = distinct_receiver();
            switch (field) {
                case 0: g.sigma_h = bad; break;
                case 1: g.sigma_v = bad; break;
                case 2: g.sigma_vel = bad; break;
                case 3: g.sigma_bias = bad; break;
                default: g.bias_tau_s = bad; break;
            }
            EXPECT_EQ(code_of(sim->add_gnss_sensor(*body, g)), invalid)
                << "field " << field << " = " << bad << " was accepted";
        }
    }

    // And the whole point of the negative battery: a VALID receiver still goes
    // through, so the rejections above are the validator working rather than
    // add_gnss_sensor refusing everything.
    EXPECT_OK(sim->add_gnss_sensor(*body, distinct_receiver()));
}

// ---------------------------------------------------------------------------
// `capacities.sensors` bounds EACH sensor arena, not their sum. The two arenas
// are independent, which is the whole reason gnss_sensors is its own registered
// array rather than a kind-tagged row in the IMU one -- and a reader who
// assumed a shared pool would find this test rather than a surprise.
// ---------------------------------------------------------------------------
TEST(GnssLifecycle, TheDeclaredSensorCapacityBoundsEachArenaNotTheirSum) {
    const spade::Result<WorldSetDesc> set = void_world_set(1);  // ONE sensor declared
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    EXPECT_OK(sim->add_imu_sensor(*body, ImuSensorSpawn{}));
    EXPECT_OK(sim->add_gnss_sensor(*body, distinct_receiver()));

    // The second of EITHER kind is what the declared capacity refuses.
    EXPECT_EQ(code_of(sim->add_gnss_sensor(*body, distinct_receiver())),
              code(spade::Code::capacity_exceeded));
    EXPECT_EQ(code_of(sim->add_imu_sensor(*body, ImuSensorSpawn{})),
              code(spade::Code::capacity_exceeded));
}

}  // namespace
