#pragma once

// <cmath> IS BACK, AND THE DISTINCTION IS THE WHOLE REASON IT LEFT. It was
// removed when this file's exp() became math::exp32: BitPortability.NoLibm-
// TranscendentalInEngineOrGoldenTestSource bans std::sin/cos/exp/log/pow and
// the inverse-trig family, because those are libm-implementation-defined and
// break bit-identity. std::sqrt IS NOT ON THAT LIST and is not an oversight:
// IEEE 754 MANDATES a correctly-rounded sqrt, so it is already bit-exact
// everywhere. core/rng.hpp:222 keeps its Box-Muller sqrt for exactly this
// reason and says so. gnss_bias_drive() below is the only user here.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include <glm/glm.hpp>

#include "core/fp32_math.hpp"
#include "core/rng.hpp"
#include "sensors/kinds.hpp"
#include "sensors/rings.hpp"
#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// sensors/gnss.hpp -- a satellite-navigation receiver: a world-frame position
// and velocity fix, at a low rate, with a slowly-varying error.
//
// WHY THIS IS THE SECOND SENSOR AND NOT A CAMERA. It needs no render path, so
// it exercises the thing that actually had to change -- an arena that
// discriminates by TAG rather than by TYPE NAME (sensors/kinds.hpp) -- without
// dragging the renderer into the first use of it. A camera would have proved
// the renderer works and left the arena question untested.
//
// ⛔ AND IT IS DELIBERATELY NOT A COPY OF imu.hpp'S SHAPE. Mirroring the IMU
// path -- a second hand-typed row with its own hand-typed poll entry point --
// would reproduce, one file later, exactly the welding that made a second
// sensor hard in the first place. What is shared here is the RING, the kind
// TAG and the noise-stream DISCIPLINE. What is not shared is the row: a GNSS
// antenna is a point, so this row has no `mount_orient`, and its error is
// dominated by a correlated bias rather than by white noise, so it has a
// correlation time where the IMU has a random-walk step.
//
// ---------------------------------------------------------------------------
// §1 FRAME, AND THE SIMPLIFICATION THAT IS STATED RATHER THAN IMPLIED
//
// A fix is reported in the WORLD frame -- metres, right-handed, Y-up, the same
// frame world files use -- and NOT in geodetic coordinates. This engine has no
// earth model, no datum and no projection, and inventing one here would put a
// second definition of "where things are" into a simulator that already has
// one.
//
// So this is a LOCAL-TANGENT-PLANE receiver: it reports the antenna's position
// in the world it is flying in. A consumer that needs latitude and longitude
// owns the datum and the origin, because only the consumer knows which patch of
// earth the world is standing in. That is a real limitation and it belongs to
// whoever wires this to a flight stack; it is not a defect of the fix.
//
// ---------------------------------------------------------------------------
// §2 THE ERROR MODEL, AND WHY IT IS NOT THE IMU'S
//
// An IMU's error is white noise plus a bias that RANDOM-WALKS: unbounded, and
// the integral of it is what makes inertial navigation drift. A GNSS receiver's
// position error does not random-walk -- it is bounded, and dominated by
// ionospheric delay and multipath, which are strongly correlated over minutes
// and then decay. The standard model is first-order Gauss-Markov:
//
//     bias <- bias * exp(-dt / tau)  +  N(0, sigma_bias * sqrt(1 - exp(-2 dt / tau)))
//     fix  =  true_position + bias + N(0, sigma_h / sigma_v)
//
// `bias_tau_s` IS THE FIELD THE IMU HAS NO EQUIVALENT OF, and it is the reason
// this row could not have been the IMU row with different numbers in it.
//
// VELOCITY IS MORE ACCURATE THAN POSITION, by an order of magnitude, and this
// is not a detail: a receiver derives velocity from carrier-phase Doppler,
// which is nearly immune to the delays that dominate the position error. A
// fusion filter that trusts this fix's velocity far more than its position is
// behaving correctly, which is why `sigma_vel` is its own field and not a
// scaling of `sigma_h`.
//
// ---------------------------------------------------------------------------
// §3 RATE, AND THE FIRST SENSOR FOR WHICH rate_divider EARNS ITS KEEP
//
// A receiver fixes at 1-10 Hz where an IMU samples every substep. `rate_divider`
// already existed on the IMU row and was very nearly always 1; here it is the
// normal case, and a consumer that polls faster than the divider gets the same
// `last_index` back and an empty span -- which is the ring's documented "nothing
// new" answer (sensors/rings.hpp), not an error.
// ---------------------------------------------------------------------------

namespace spade::sensors {

// The rng domain tag every GNSS noise stream is derived under (core/rng.hpp's
// make_stream(world_seed, tag, index)). PINNED: changing this string re-seeds
// every receiver in every world and invalidates every recorded snapshot and
// every committed determinism digest. It must differ from kImuNoiseDomainTag,
// or two sensors on one body would draw the same numbers.
inline constexpr std::string_view kGnssNoiseDomainTag = "sensor.gnss";

// ---------------------------------------------------------------------------
// GnssFix -- one tick-stamped fix. One element of the world-partitioned
// "gnss_ring" array; see sensors/rings.hpp for the TA5 index convention.
//
// Three 16-byte rows, following state/layout.hpp's std430 discipline:
//   row 0  position | sigma_h
//   row 1  velocity | sigma_v
//   row 2  index    | tick
//
// THE ACCURACY ESTIMATES TRAVEL WITH THE FIX rather than being read back from
// the row, because a real receiver reports its own accuracy per fix and a
// filter weights each fix by the number that came with it. It also means the
// two dead pad lanes ImuSample carries are named fields here.
// ---------------------------------------------------------------------------
// ⛔ sigma_h IS PER-AXIS, NOT RADIAL, AND THE FIELD NAME DEFAULTS TO THE OTHER
// READING. Clarified 2026-09-24 after Runtime read "horizontal accuracy" the
// standard way and asked before writing -- which is the only reason this was
// caught before a consumer shipped on it.
//
// In GNSS practice "horizontal accuracy" is a RADIAL 2D quantity (x (+) y).
// HERE IT IS THE PER-AXIS SIGMA: sensors/gnss.cpp:174 and the Slang twin both
// apply it independently to X and Z --
//     position += (sigma_h * white.x, sigma_v * white.y, sigma_h * white.z)
// -- where white.x and white.z are INDEPENDENT standard normals. So the radial
// 2D error of a fix is Rayleigh with scale sigma_h, and its RMS is
// sigma_h * sqrt(2), NOT sigma_h.
//
// ⛔ THE ERROR DIRECTION IS THE DANGEROUS ONE. A consumer reading this as
// radial infers a per-axis sigma of sigma_h/sqrt(2) and therefore UNDERSTATES
// the true per-axis uncertainty by a factor of sqrt(2). An estimator weighting
// a fix by that number OVER-TRUSTS the measurement, which is the failure mode
// that does not announce itself -- the filter simply converges too confidently.
//
// A DOMAIN TERM CARRIES ITS DOMAIN'S MEANING WHETHER OR NOT THE IMPLEMENTATION
// AGREES, so the disagreement has to be written down at the field rather than
// left to whoever reads the multiply. Any wire type carrying this value should
// say per-axis in its own name or its own comment; the default reading of the
// bare name is the wrong one.
struct alignas(kStd430StructAlignment) GnssFix {
    glm::vec3 position;  // antenna position, WORLD frame, m (§1)
    float sigma_h;       // PER-AXIS horizontal 1-sigma, m -- see the note above, NOT radial
    glm::vec3 velocity;  // antenna velocity, WORLD frame, m/s (§2: Doppler-derived)
    float sigma_v;       // vertical 1-sigma, m -- ONE axis (section 1 names which), so no
                         // radial ambiguity here. Section 1 is the only place in this header
                         // that says which axis is up, deliberately: see the note above.
    SampleIndex index;   // monotonically increasing, per sensor, starts at 1 (TA5)
    uint64_t tick;       // the step this fix was produced in
};

static_assert(std::is_standard_layout_v<GnssFix>, "GnssFix must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<GnssFix>, "GnssFix must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<GnssFix>, "arena slots are never individually destroyed");
static_assert(alignof(GnssFix) == 16, "std430 base alignment");
static_assert(sizeof(GnssFix) == 48, "std430 array stride");

static_assert(offsetof(GnssFix, position) == 0);
static_assert(offsetof(GnssFix, sigma_h) == 12);
static_assert(offsetof(GnssFix, velocity) == 16);
static_assert(offsetof(GnssFix, sigma_v) == 28);
static_assert(offsetof(GnssFix, index) == 32);
static_assert(offsetof(GnssFix, tick) == 40);

static_assert(offsetof(GnssFix, position) % 16 == 0);
static_assert(offsetof(GnssFix, velocity) % 16 == 0);
static_assert(offsetof(GnssFix, index) % 8 == 0, "uint64 needs 8-byte alignment in std430 too");

// Named fields account for every byte: no implicit padding. This is what makes
// ring_write()'s whole-object assignment byte-deterministic (see rings.hpp) and
// what makes a snapshot of the ring comparable with memcmp.
static_assert(sizeof(GnssFix::position) + sizeof(GnssFix::sigma_h) + sizeof(GnssFix::velocity) +
                  sizeof(GnssFix::sigma_v) + sizeof(GnssFix::index) + sizeof(GnssFix::tick) ==
                  sizeof(GnssFix),
              "GnssFix has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// GnssSensorRow -- one receiver's table row. One element of the
// world-partitioned "gnss_sensors" array.
//
// 96 bytes against ImuSensorRow's 128, and the difference is the point: no
// mount orientation (an antenna is a point), no gyro channel, and one bias
// vector instead of two. A row type that came out the same size as the IMU's
// would be a sign it had been copied rather than designed.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) GnssSensorRow {
    uint32_t body_slot;     // WORLD-LOCAL index into this world's bodies span
    uint32_t kind;          // sensor_kind::; 0 == none == inert (active-high liveness)
    uint32_t rate_divider;  // emit one fix every N substeps; §3 -- normally >> 1 here
    uint32_t phase;         // substeps elapsed in the current sensor period
    glm::vec3 mount_pos;    // antenna lever arm, BODY-frame offset from the COM, m
    float _p0;              // std430 pad -- keeps `bias` on row 2
    glm::vec3 bias;         // correlated position error state, WORLD frame, m (§2)
    float _p1;              // std430 pad
    float sigma_h;          // horizontal white noise, per-fix stddev, m
    float sigma_v;          // vertical white noise, per-fix stddev, m
    float sigma_vel;        // velocity white noise, per-axis per-fix stddev, m/s (§2)
    float bias_tau_s;       // Gauss-Markov correlation time, s; <= 0 disables the bias.
                            // THE SOURCE, KEPT DELIBERATELY -- see bias_retention.
    rng::Stream noise;      // this receiver's own stream; ALL of its randomness
    SampleIndex last_index; // index of the newest fix written; 0 == none yet

    // -----------------------------------------------------------------------
    // THE TWO DERIVED CONSTANTS, COMPUTED ONCE ON THE CPU AT SPAWN AND NEVER ON
    // THE STEP PATH. §2's model is
    //
    //   bias <- bias * exp(-dt/tau) + N(0, sigma_bias * sqrt(1 - exp(-2 dt/tau)))
    //
    // and BOTH coefficients depend only on `dt` -- IMMUTABLE AT CREATION -- and
    // on `tau`, fixed per receiver. They are PER-SENSOR CONSTANTS, so
    // add_gnss_sensor() evaluates them once with math::exp32 and the synthesis
    // pass just multiplies.
    //
    // THAT IS A PARITY DECISION, NOT AN OPTIMISATION. shaders/fp32_math.slang
    // records that "an OpFDiv is not merely less accurate, it is
    // DEVICE-DEPENDENT", which is why log32_div() is a restoring integer long
    // division and is domain-restricted. Other kernels divide freely, but those
    // are BANDED physics paths where an ulp sits inside a band. A RECURSIVE BIAS
    // FILTER DOES NOT TOLERATE ERROR, IT ACCUMULATES IT: an ulp of retention
    // error compounds on every emission and walks out of any band eventually.
    // Keeping the transcendental off the step path makes the Slang twin
    // bit-identical BY CONSTRUCTION rather than by tolerance.
    // -----------------------------------------------------------------------
    float bias_retention;   // exp(-dt / bias_tau_s); 0 == no bias (tau <= 0)
    float bias_drive;       // sigma_bias * sqrt(1 - exp(-2 dt / bias_tau_s))

    // THE SECOND SOURCE, AND IT IS WHY THE MODEL IS NOW EXPRESSIBLE AT ALL.
    // §2 has always documented a `sigma_bias`; the row shipped without one, so
    // the bias process had a correlation time and NO MAGNITUDE -- a bias that
    // decayed toward zero and was never driven. Stored rather than folded away
    // into bias_drive for the same reason bias_tau_s is: a derived constant
    // cannot answer "derived from WHAT?", and the alternative is a later author
    // recovering it by inverting a float.
    float sigma_bias;       // stationary stddev of the correlated bias, m (§2)
    float _p2;              // std430 pad -- keeps `_reserved0` 8-byte aligned
    uint64_t _reserved0;    // reserved for versioned growth; must stay 0
};

static_assert(std::is_standard_layout_v<GnssSensorRow>, "GnssSensorRow must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<GnssSensorRow>, "GnssSensorRow must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<GnssSensorRow>, "arena slots are never individually destroyed");
static_assert(alignof(GnssSensorRow) == 16, "std430 base alignment");
static_assert(sizeof(GnssSensorRow) == 112, "std430 array stride");

static_assert(offsetof(GnssSensorRow, body_slot) == 0);
static_assert(offsetof(GnssSensorRow, kind) == 4);
static_assert(offsetof(GnssSensorRow, rate_divider) == 8);
static_assert(offsetof(GnssSensorRow, phase) == 12);
static_assert(offsetof(GnssSensorRow, mount_pos) == 16);
static_assert(offsetof(GnssSensorRow, _p0) == 28);
static_assert(offsetof(GnssSensorRow, bias) == 32);
static_assert(offsetof(GnssSensorRow, _p1) == 44);
static_assert(offsetof(GnssSensorRow, sigma_h) == 48);
static_assert(offsetof(GnssSensorRow, sigma_v) == 52);
static_assert(offsetof(GnssSensorRow, sigma_vel) == 56);
static_assert(offsetof(GnssSensorRow, bias_tau_s) == 60);
static_assert(offsetof(GnssSensorRow, noise) == 64);
static_assert(offsetof(GnssSensorRow, last_index) == 80);
// THE APPEND. Every offset at or below 88 above is unchanged from the 96-byte
// row this grew out of, which is what makes the growth an APPEND rather than a
// reshuffle -- and is why the shipped pins did not have to be rewritten.
static_assert(offsetof(GnssSensorRow, bias_retention) == 88);
static_assert(offsetof(GnssSensorRow, bias_drive) == 92);
static_assert(offsetof(GnssSensorRow, sigma_bias) == 96);
static_assert(offsetof(GnssSensorRow, _p2) == 100);
static_assert(offsetof(GnssSensorRow, _reserved0) == 104);

// Every vec3 starts a 16-byte row; the stream and the two uint64s are 8-byte
// aligned, which std430 requires of them too.
static_assert(offsetof(GnssSensorRow, mount_pos) % 16 == 0);
static_assert(offsetof(GnssSensorRow, bias) % 16 == 0);
static_assert(offsetof(GnssSensorRow, noise) % 8 == 0);
static_assert(offsetof(GnssSensorRow, last_index) % 8 == 0);

// Named fields account for every byte: no implicit padding, same discipline as
// state/layout.hpp's BodyState. (rng::Stream carries the same guarantee for its
// own 16 bytes -- see core/rng.hpp.)
static_assert(4 * sizeof(uint32_t) + 2 * (sizeof(glm::vec3) + sizeof(float)) +
                  4 * sizeof(float) + sizeof(rng::Stream) + sizeof(SampleIndex) +
                  4 * sizeof(float) + sizeof(uint64_t) ==
                  sizeof(GnssSensorRow),
              "GnssSensorRow has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// The stream one receiver draws its noise from, derived exactly once here so
// the pass, the spawn path and any test agree by construction.
//
// The index is the sensor's WORLD-LOCAL slot, not its global arena slot: a
// world's randomness must not depend on where that world sits in the set, or
// the batching-invariance property (D8; engine/testing/replay.hpp) is false.
// This mirrors imu_noise_stream() deliberately -- the DOMAIN TAG is what keeps
// the two apart, not the derivation.
// ---------------------------------------------------------------------------
[[nodiscard]] constexpr rng::Stream gnss_noise_stream(uint64_t world_seed,
                                                      uint32_t local_slot) noexcept {
    return rng::make_stream(world_seed, kGnssNoiseDomainTag, local_slot);
}

// ---------------------------------------------------------------------------
// The Gauss-Markov retention factor for a step of `dt` seconds at correlation
// time `tau` (§2). Pulled out so the pass, a test and any future Slang kernel
// compute it the same way rather than each spelling the exponential.
//
// tau <= 0 means "no correlated bias": the factor is 0, the bias is redrawn
// white each fix, and a caller that also sets sigma_bias to 0 gets a receiver
// whose only error is the per-fix white noise. Returning 0 rather than
// rejecting tau <= 0 keeps this total -- a row is data, and a pass must not
// have to branch on a validation the spawn path already did.
//
// ---------------------------------------------------------------------------
// THE EXPONENTIAL IS spade::math::exp32, NOT std::exp, AND THAT IS THE WHOLE
// POINT OF THE LINE BELOW.
//
// IEEE-754 mandates correct rounding for + - * / sqrt and the comparisons, and
// mandates NOTHING for exp. The MSVC CRT and glibc are both excellent and they
// disagree by an ulp on roughly one input in a hundred, so an engine whose
// determinism contract is bit-exactness cannot call libm and still call itself
// deterministic. core/fp32_math.hpp is this engine's answer: exp32 is built
// exclusively from IEEE-mandated operations over pinned groupings, is within
// one ulp of the correctly-rounded exponential across the whole finite domain,
// and HAS A SLANG TWIN (engine/shaders/fp32_math.slang) that returns the same
// bits. tests/test_m1b_bar.cpp's BitPortability guard sweeps engine source for
// libm transcendentals and fails the build's test run if one appears.
//
// ⛔ ERRATUM, 2026-09-21, BEFORE THIS EVER RAN: this comment block previously
// said the OPPOSITE -- "THIS VALUE IS COMPUTED ON THE HOST, ONCE PER STEP, AND
// UPLOADED. A GPU PASS MUST NEVER RECOMPUTE IT" -- and justified it correctly
// from the premise that a CPU exp and a Slang exp() are not bit-identical. The
// premise was true of std::exp and FALSE of this engine, which had already
// solved it at S5 Task 1. I diagnosed a real hazard, designed a real structural
// workaround for it, and wrote it down at the site -- WITHOUT LOOKING FOR THE
// REMEDY THAT WAS ALREADY IN core/. The BitPortability guard is what caught it,
// which is exactly the job it was written for.
//
// ⭐ SO A GPU PASS MAY RECOMPUTE THIS FREELY. Uploading the factor as a per-row
// scalar is still a legitimate optimisation (dt and tau are per-row constants
// within a step) but it is now a CHOICE ABOUT WORK, not a correctness
// requirement, and nothing downstream should treat it as one.
// ---------------------------------------------------------------------------
[[nodiscard]] inline float gnss_bias_retention(float dt_s, float tau_s) noexcept {
    if (!(tau_s > 0.0f) || !(dt_s > 0.0f)) return 0.0f;
    return math::exp32(-dt_s / tau_s);
}

// ---------------------------------------------------------------------------
// The OTHER half of §2's Gauss-Markov step: the stddev of the white term that
// DRIVES the bias, sigma_bias * sqrt(1 - exp(-2 dt / tau)).
//
// IT EXISTS BECAUSE THE ROW COULD NOT EXPRESS THE MODEL WITHOUT IT. §2 has
// documented this process since the file was written, but GnssSensorRow shipped
// with a correlation time and NO MAGNITUDE -- so a receiver built from it had a
// bias that decayed toward zero and was never driven. The header described a
// model the struct implemented a subset of, and nothing in this tree compares
// the two.
//
// SAME DOMAIN CONTRACT AS gnss_bias_retention, deliberately: tau <= 0 or
// dt <= 0 means "no correlated bias", and both coefficients must agree about
// that or a receiver could be driven without decaying. The sqrt is evaluated at
// SPAWN, on the CPU, exactly once -- it never reaches a kernel, for the same
// reason the exp does not.
//
// The 1 - exp(-2 dt/tau) form is the stationary-variance solution: it is what
// makes the bias's steady-state stddev equal sigma_bias regardless of dt, so
// changing the substep size does not silently change how noisy a receiver is.
// ---------------------------------------------------------------------------
[[nodiscard]] inline float gnss_bias_drive(float dt_s, float tau_s, float sigma_bias) noexcept {
    if (!(tau_s > 0.0f) || !(dt_s > 0.0f)) return 0.0f;
    const float retention_sq = math::exp32(-2.0f * dt_s / tau_s);
    const float variance = 1.0f - retention_sq;
    if (!(variance > 0.0f)) return 0.0f;
    return sigma_bias * std::sqrt(variance);
}

// ---------------------------------------------------------------------------
// synthesize_gnss() -- the SensorSynthesis pass's GNSS contribution, for ONE
// world's spans. The SECOND batched call in that pass, not a second pass:
// physics/schedule.hpp ruled that shape before this sensor existed.
//
// NINE STANDARD NORMALS PER EMITTED FIX, ALWAYS -- bias-walk, position-white,
// velocity-white, each x/y/z -- whatever the sigmas are, so that the stream
// position is a function of the emitted-fix count alone and configuring a
// receiver noise-free does not shift every other draw in the world. Same
// discipline imu.hpp section 4 states for its twelve.
//
// NINE IS ODD AND THAT IS FINE, WHICH IS WORTH SAYING BECAUSE THE NEIGHBOURING
// FILE'S EVEN COUNT READS AS A RULE. sensor_imu.slang calls its even twelve "a
// small dividend": rng::Stream's Box-Muller cache is empty again at the end of
// every sample. Nine leaves it FULL -- and that is still bit-identical across
// backends, because core/rng.hpp's Stream and layouts.slang's RngStream mirror
// each other INCLUDING `cached_gauss`/`has_cached`, so both sides carry the
// same spare forward. AN UNSTATED NON-REQUIREMENT BECOMES A REQUIREMENT BY
// INHERITANCE, so the non-event is stated.
//
// Same precondition as synthesize_imu's, and unchecked for the same reason:
// every sensors[i].body_slot with kind == gnss is a valid index into `bodies`.
//
// Allocates nothing, reads no clock, evaluates no transcendental and performs
// no division (see GnssSensorRow's bias_retention/bias_drive), and draws only
// from each row's own stream.
// ---------------------------------------------------------------------------
void synthesize_gnss(std::span<const BodyState> bodies, std::span<GnssSensorRow> sensors,
                     std::span<GnssFix> rings, uint64_t tick) noexcept;

}  // namespace spade::sensors
