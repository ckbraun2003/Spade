#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <type_traits>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/rng.hpp"
#include "sensors/kinds.hpp"
#include "sensors/rings.hpp"
#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// THE IMU -- the imu.synthesize pass (engine design spec §3 "The step model":
// "SensorSynthesis // only on sensor-rate boundaries; writes sensor output
// rings", and §5's "Specific force (acceleration minus gravity, body frame) is
// captured inside Integrate for the IMU pass -- computed once, not
// reconstructed").
//
// This is the first sensor, and it ships ONE kind. `kind` exists as a field so
// the table is a SENSOR table rather than an IMU table, but there is no
// dispatch, no registry and no second kind: a barometer that does not exist is
// a barometer that cannot be wrong.
//
// ===========================================================================
// 1. THE SIGN CONVENTIONS, PINNED
//
// AN ACCELEROMETER MEASURES SPECIFIC FORCE, NOT ACCELERATION -- the
// non-gravitational force per unit mass acting on its proof mass. The two
// consequences everyone rediscovers, stated here so nobody has to:
//
//   * A BODY AT REST IN GRAVITY READS +g ALONG THE AXIS OPPOSITE GRAVITY.
//     Spade's worlds are Y-UP (world/builder.hpp's Environment defaults
//     gravity to (0, -9.80665, 0)), so a LEVEL body held stationary reads
//     accel = (0, +9.80665, 0) in the body frame. Written in the Z-up spelling
//     the editor/spec text uses, that is the familiar "(0, 0, +g)"; the axis
//     differs, the SIGN -- opposite gravity, not along it -- is the contract.
//   * A BODY IN FREE FALL READS ZERO, on every axis. Not -g.
//
// Where the number comes from: physics/integrator.cpp step 2 stores
//
//     BodyState::specific_force = conjugate(orient) * (force_acc / mass)
//
// i.e. the accumulated NON-gravitational force per unit mass, rotated from the
// world frame into the body frame by the conjugate of the body->world
// orientation. Gravity is added to the dynamics in step 3 and never enters
// force_acc, so this is an exact capture rather than a (a - g) subtraction
// that would catastrophically cancel in fp32 near free fall. THIS FIELD IS THE
// ACCELEROMETER'S PHYSICAL INPUT; this pass rotates it to the mount and adds
// the error model, and does not recompute it.
//
// THE GYRO reads the body's angular velocity, rotated into the mount frame.
// BodyState::omega_body is body-frame rad/s and angular velocity is the same
// for every point of a rigid body, so there is no lever-arm term for the gyro
// and no ambiguity about where on the vehicle it is bolted.
//
// A MOUNT POSE IS `mount` -> `body`. `mount_pos` is the body-frame offset of
// the sensor from the body's COM (m) and `mount_orient` rotates a MOUNT-frame
// vector into the BODY frame -- the same direction convention as BodyState's
// `orient` (body -> world) and DragBodyRow's `local_orient` (local -> body).
// Reading a body-frame quantity in the mount frame therefore applies the
// CONJUGATE, exactly as the integrator applies conjugate(orient) to reach the
// body frame from the world frame.
//
// ===========================================================================
// 2. THE LEVER ARM: THE CENTRIPETAL TERM IS IN, THE ANGULAR-ACCELERATION TERM
//    IS OUT (v2)
//
// For a sensor at body-frame offset r from the COM of a body with angular
// velocity omega and angular acceleration alpha, the specific force at the
// mount is
//
//     f_mount = f_com + omega x (omega x r) + alpha x r
//               \_____/   \______________/   \_______/
//                COM        CENTRIPETAL        EULER
//
// v2 implements the first two terms and DELIBERATELY OMITS the third.
//
//   * THE CENTRIPETAL TERM IS KEPT because it is the one an off-axis mount
//     produces at constant spin -- a quadrotor IMU 5 cm off the COM at 10 rad/s
//     sees 5 m/s^2 from it, half a g of pure geometry that no filter should be
//     asked to explain. It is also free: omega is already in hand.
//   * THE EULER TERM IS OMITTED because alpha is not state. Nothing in
//     BodyState stores angular acceleration, and the honest ways to get it are
//     to recompute the whole angular dynamics here (duplicating the
//     integrator's op order in a second place, which is exactly the parity
//     hazard D11 exists to prevent) or to finite-difference omega across
//     substeps (which requires holding a previous omega per BODY, in registered
//     state, and yields a quantity one substep stale). Both are real designs;
//     neither is worth carrying before something needs the term. The effect is
//     an accelerometer error proportional to |alpha| * |r| during ANGULAR
//     TRANSIENTS only -- zero at constant spin, and zero for a COM mount. When
//     it is wanted, the integrator is the place that already knows alpha and
//     the place to publish it from.
//
// ===========================================================================
// 3. THE TWO CHANNELS OF ONE SAMPLE ARE TAKEN AT DIFFERENT INSTANTS, AND THAT
//    IS ACCEPTED RATHER THAN OVERLOOKED
//
// This pass runs AFTER Integrate (spec §3's order), so by the time it reads a
// body, `integrate_bodies()` has already advanced that body from the substep's
// start instant t to its end instant t+h. The two fields this sensor reads did
// NOT survive that advance in the same state:
//
//   * `specific_force` IS A SUBSTEP-START QUANTITY. integrator.cpp step 2
//     captures it BEFORE steps 4 and 6 update omega and orientation, and it
//     does so deliberately: force_acc was built by the force passes using the
//     PRE-update orientation, so undoing that rotation with the conjugate of
//     the very same quaternion is an exact round trip back to the body-frame
//     sum. Using the post-update orientation would "smear one substep of
//     rotation into every IMU sample -- a bias proportional to omega*h that no
//     downstream filter could distinguish from a real accelerometer error"
//     (integrator.cpp's own words). That ruling is not reopened here.
//   * `omega_body` IS A SUBSTEP-END QUANTITY. It is a plain state field, and
//     integrator.cpp step 4c has already added this substep's angular
//     acceleration to it by the time this pass looks. There is no
//     pre-update copy anywhere in the state.
//
// SO ONE ImuSample MIXES INSTANTS: its accel is the specific force acting over
// [t, t+h) expressed in the body frame at t, and its gyro is omega at t+h. (The
// centripetal term inherits the gyro's instant, since it is built from the same
// omega, which is a second-order version of the same thing.)
//
// THE ERROR THIS COSTS is exactly one substep of angular acceleration:
//
//     gyro_reported - gyro_at_t  =  h * I^-1 * tau  =  alpha * h
//
// bounded by |alpha| * h, ZERO whenever the applied torque is zero, and of the
// same order and the same regime as the Euler lever-arm term §2 already omits
// and accepts (|alpha| * |r|). At a 1 kHz substep, a racing quadrotor's ~100
// rad/s^2 transient gives ~0.1 rad/s against gyro readings of 10-20 rad/s --
// under a percent, during transients only, and comparable to the white noise a
// real MEMS gyro carries at that rate anyway.
//
// WHY IT IS NOT FIXED, weighed rather than waved away. Removing it means
// capturing a pre-update omega, and every home for that capture costs more than
// the error is worth:
//
//   * IN BodyState -- it is exactly 128 bytes of eight FULL std430 rows with
//     every byte named and a per-field offsetof table, single-sourced for the
//     S6 Slang generator. There is no spare slot (the `_pN` fields are 4-byte
//     pads inside vec3 rows). A ninth row makes it 144 bytes: +12.5% on the
//     hottest array in the engine and on every S6 device buffer, a changed
//     std430 array stride, a changed snapshot schema hash, and every golden
//     digest in the determinism corpus moved again.
//   * IN A NEW REGISTERED ARRAY -- the same schema and golden churn, plus an
//     extra array and slot->world map, plus a store per body per substep in the
//     integrator's inner loop that only sensors would ever read.
//   * IN THE SENSOR ROW, carried over from the previous substep -- structurally
//     cheapest, but it introduces "what is the previous omega before the first
//     substep?" and "what is it after the body was inactive for a while?", and
//     it makes the gyro disagree with the body state at the sample's own tick.
//
// All three to remove a bounded |alpha|*h term. So: DOCUMENTED, NOT FIXED --
// and if it is ever fixed, the integrator is the place that already has the
// pre-update value in hand, exactly as with §2's alpha.
//
// AND IT IS PINNED BY TESTS, so a future change to either instant fails loudly
// instead of silently. Note that the suite's other gyro tests CANNOT catch such
// a change: they run at zero torque with unit inertia to get bit-exact
// assertions, which is precisely the configuration in which pre- and
// post-update omega are identical. Two tests exist specifically to break that
// degeneracy -- test_imu.cpp's GyroReadsThePostUpdateOmegaOfItsSubstep (nonzero
// torque, so the two candidate values differ by exactly h*I^-1*tau, and the
// reported one is asserted bit-exactly) and AccelUsesTheSubstepStartOrientation
// (a spinning body under a world-frame force, where the post-update orientation
// would rotate the reading measurably off-axis).
//
// ===========================================================================
// 4. THE NOISE MODEL, AND ITS UNITS
//
//     accel_measured = R_mount^-1 * (f_com + omega x (omega x r)) + bias_a + sigma_a * n
//     gyro_measured  = R_mount^-1 * omega                         + bias_g + sigma_g * n
//     bias_a += sigma_ba * n     (a random walk, one step per SAMPLE)
//     bias_g += sigma_bg * n
//
// with every `n` an independent standard normal from this sensor's own stream.
//
// SIGMAS ARE PER-SAMPLE STANDARD DEVIATIONS, NOT SPECTRAL DENSITIES. This is a
// ruling, not an oversight, and it is the one thing to get right when
// configuring a sensor from a datasheet:
//
//     sigma_a  = accel_noise_density [m/s^2/sqrt(Hz)] * sqrt(f_sensor)
//     sigma_ba = accel_bias_rw_density [m/s^3/sqrt(Hz)] / sqrt(f_sensor)
//
// where f_sensor = 1 / (rate_divider * substep_h) is this sensor's own output
// rate. Discrete sigmas are what the pass can apply without knowing its own
// rate, what an Allan-variance test compares against directly, and what a
// config layer can compute once at authoring time. A rate change therefore
// requires recomputing them -- which is the honest cost of the simpler model,
// and is stated here rather than discovered.
//
// DRAW ORDER AND DRAW COUNT ARE PINNED (see synthesize_imu). Exactly TWELVE
// standard normals are consumed per EMITTED sample, always, whatever the sigmas
// are -- the same discipline world/medium.hpp's Dryden advance states for its
// five ("the stream position is a function of the substep count alone and not
// of the turbulence level"). A sensor configured noise-free therefore occupies
// the same stream positions as a noisy one, so turning noise off does not
// perturb anything else in the world.
//
// ===========================================================================
// 5. WHAT THIS SENSOR CANNOT SEE, WHICH IS NOT A BUG BUT IS A SURPRISE
//
// force_acc is the accelerometer's whole input, and CONTACT RESPONSE DOES NOT
// GO THROUGH IT: physics/contacts.cpp resolves contacts at the VELOCITY level
// (impulses applied straight to BodyState::vel) and at the POSITION level
// (Baumgarte), never as a force. So a body resting on the world SDF reads
// approximately ZERO, not +g -- the ground's normal force is not modelled as a
// force. A test that wants a static +g reading holds the body up with
// Simulation::apply_wrench(), which is a force and does flow through force_acc.
// Impulse-based contact and force-based sensing is a real seam; making the two
// agree is a contact-model question (S7+), not a sensor question.
// ---------------------------------------------------------------------------

namespace spade::sensors {

// ---------------------------------------------------------------------------
// ImuSensorRow::kind values LIVE IN sensors/kinds.hpp, not here.
//
// They were declared in this file until 2026-09-21, which made the tag that
// separates sensor KINDS a member of one kind's header: a second kind could not
// name its own tag without including this file's row, noise model and std430
// layout. The vocabulary that separates two things cannot live inside one of
// them. `sensor_kind::imu` is the value this file's row carries; kinds.hpp
// states the rules for adding another and why `none` must stay legal on the
// poll path.
// ---------------------------------------------------------------------------

// The rng domain tag every IMU noise stream is derived under (core/rng.hpp's
// make_stream(world_seed, tag, index)). PINNED: changing this string re-seeds
// every IMU in every world and invalidates every recorded snapshot and every
// committed determinism digest.
inline constexpr std::string_view kImuNoiseDomainTag = "sensor.imu";

// ---------------------------------------------------------------------------
// ImuSample -- one tick-stamped reading. One element of the world-partitioned
// "imu_ring" array; see sensors/rings.hpp for the TA5 index convention.
//
// Three 16-byte rows, following state/layout.hpp's std430 discipline:
//   row 0  accel | _p0
//   row 1  gyro  | _p1
//   row 2  index | tick
//
// FRAMES AND UNITS: `accel` and `gyro` are MOUNT frame (m/s^2 and rad/s) --
// the frame the physical sensor's own axes define, which is what a consumer
// fusing this against a mounted camera needs. `tick` is the step the sample was
// produced in (core/time.hpp); every substep of step k stamps k, because Tick
// counts steps and is incremented after the last substep.
//
// THE TWO CHANNELS ARE NOT TAKEN AT THE SAME INSTANT within the substep -- accel
// is a substep-START quantity and gyro a substep-END one, differing by alpha*h.
// See §3 above for the full statement, the bound, and why it is accepted; a
// consumer doing tight attitude/accel fusion should read it.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) ImuSample {
    glm::vec3 accel;    // mount-frame specific force, m/s^2 (sign: §1; instant: §3)
    float _p0;          // std430 pad -- keeps `gyro` on row 1
    glm::vec3 gyro;     // mount-frame angular velocity, rad/s (instant: §3)
    float _p1;          // std430 pad
    SampleIndex index;  // monotonically increasing, per sensor, starts at 1 (editor tech spec TA5)
    uint64_t tick;      // the step this sample was produced in
};

static_assert(std::is_standard_layout_v<ImuSample>, "ImuSample must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<ImuSample>, "ImuSample must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<ImuSample>, "arena slots are never individually destroyed");
static_assert(alignof(ImuSample) == 16, "std430 base alignment");
static_assert(sizeof(ImuSample) == 48, "std430 array stride");

static_assert(offsetof(ImuSample, accel) == 0);
static_assert(offsetof(ImuSample, _p0) == 12);
static_assert(offsetof(ImuSample, gyro) == 16);
static_assert(offsetof(ImuSample, _p1) == 28);
static_assert(offsetof(ImuSample, index) == 32);
static_assert(offsetof(ImuSample, tick) == 40);

static_assert(offsetof(ImuSample, accel) % 16 == 0);
static_assert(offsetof(ImuSample, gyro) % 16 == 0);
static_assert(offsetof(ImuSample, index) % 8 == 0, "uint64 needs 8-byte alignment in std430 too");

// Named fields account for every byte: no implicit padding. This is what makes
// ring_write()'s whole-object assignment byte-deterministic (see rings.hpp) and
// what makes a snapshot of the ring comparable with memcmp.
static_assert(sizeof(ImuSample::accel) + sizeof(ImuSample::_p0) + sizeof(ImuSample::gyro) +
                  sizeof(ImuSample::_p1) + sizeof(ImuSample::index) + sizeof(ImuSample::tick) ==
                  sizeof(ImuSample),
              "ImuSample has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// ImuSensorRow -- one sensor's table row. One element of the world-partitioned
// "imu_sensors" array.
//
// Eight 16-byte rows:
//   row 0  body_slot | kind | rate_divider | phase
//   row 1  mount_pos | _p0
//   row 2  mount_orient (quat)
//   row 3  sigma_a | sigma_g | sigma_ba | sigma_bg
//   row 4  bias_a | _p1
//   row 5  bias_g | _p2
//   row 6  noise (rng::Stream)
//   row 7  last_index | _reserved0
//
// EVERYTHING THE SENSOR NEEDS TO CONTINUE IS IN THIS ROW, and that is the
// design rule rather than a convenience: the row is registered state, so the
// bias random walk, the phase counter, the rng stream and the ring write cursor
// all ride every snapshot automatically. A stream cached in a Simulation member,
// or a bias held in a pass-local, would be precisely the state a replay misses.
//
// `body_slot` indexes into the SAME world's `bodies` span synthesize_imu() is
// handed -- WORLD-LOCAL, not a global arena slot, exactly like
// DragBodyRow::body_slot and for the same reasons (the pass never consults a
// world id, and a world's state must not depend on its index or batching
// invariance breaks).
//
// `phase` is the rate divider's substep counter: incremented once per substep,
// and a sample is emitted (and the counter reset) when it reaches
// `rate_divider`. So a sensor fires on the substep that COMPLETES each full
// sensor period -- the sample instants are t = k*h, 2k*h, ... -- and exactly
// N/k samples come out of N substeps when k divides N.
//
// THE RING REFERENCE IS IMPLICIT, deliberately. A sensor at global slot g owns
// ring slots [g * kRingDepth, (g+1) * kRingDepth). Storing that base in the row
// would be a second source of truth for a number the slot already determines,
// and the two would disagree the first time a slot was recycled.
//
// `_reserved0` must stay 0. It is the growth room a versioned layout owes
// itself (same discipline as WorldParams::_reserved0), and it is what keeps
// row 7 a full 16 bytes so nothing moves when it is spent.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) ImuSensorRow {
    uint32_t body_slot;      // WORLD-LOCAL index into this world's bodies span
    uint32_t kind;           // sensor_kind::; 0 == none == inert (active-high liveness)
    uint32_t rate_divider;   // emit one sample every N substeps; 1 == every substep
    uint32_t phase;          // substeps elapsed in the current sensor period
    glm::vec3 mount_pos;     // r, BODY-frame offset from the body's COM, m
    float _p0;               // std430 pad -- keeps `mount_orient` on row 2
    glm::quat mount_orient;  // mount -> body rotation, unit quaternion
    float sigma_a;           // accel white noise, per-sample stddev, m/s^2
    float sigma_g;           // gyro white noise, per-sample stddev, rad/s
    float sigma_ba;          // accel bias random-walk step stddev, m/s^2 per sample
    float sigma_bg;          // gyro bias random-walk step stddev, rad/s per sample
    glm::vec3 bias_a;        // accel bias state, MOUNT frame, m/s^2
    float _p1;               // std430 pad
    glm::vec3 bias_g;        // gyro bias state, MOUNT frame, rad/s
    float _p2;               // std430 pad
    rng::Stream noise;       // this sensor's own stream; ALL of its randomness
    SampleIndex last_index;  // index of the newest sample written; 0 == none yet
    uint64_t _reserved0;     // reserved for versioned growth; must stay 0
};

static_assert(std::is_standard_layout_v<ImuSensorRow>, "ImuSensorRow must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<ImuSensorRow>, "ImuSensorRow must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<ImuSensorRow>, "arena slots are never individually destroyed");
static_assert(alignof(ImuSensorRow) == 16, "std430 base alignment");
static_assert(sizeof(ImuSensorRow) == 128, "std430 array stride");

static_assert(offsetof(ImuSensorRow, body_slot) == 0);
static_assert(offsetof(ImuSensorRow, kind) == 4);
static_assert(offsetof(ImuSensorRow, rate_divider) == 8);
static_assert(offsetof(ImuSensorRow, phase) == 12);
static_assert(offsetof(ImuSensorRow, mount_pos) == 16);
static_assert(offsetof(ImuSensorRow, _p0) == 28);
static_assert(offsetof(ImuSensorRow, mount_orient) == 32);
static_assert(offsetof(ImuSensorRow, sigma_a) == 48);
static_assert(offsetof(ImuSensorRow, sigma_g) == 52);
static_assert(offsetof(ImuSensorRow, sigma_ba) == 56);
static_assert(offsetof(ImuSensorRow, sigma_bg) == 60);
static_assert(offsetof(ImuSensorRow, bias_a) == 64);
static_assert(offsetof(ImuSensorRow, _p1) == 76);
static_assert(offsetof(ImuSensorRow, bias_g) == 80);
static_assert(offsetof(ImuSensorRow, _p2) == 92);
static_assert(offsetof(ImuSensorRow, noise) == 96);
static_assert(offsetof(ImuSensorRow, last_index) == 112);
static_assert(offsetof(ImuSensorRow, _reserved0) == 120);

// Every vec3/quat starts a 16-byte row; the stream and the two uint64s are
// 8-byte aligned, which std430 requires of them too.
static_assert(offsetof(ImuSensorRow, mount_pos) % 16 == 0);
static_assert(offsetof(ImuSensorRow, mount_orient) % 16 == 0);
static_assert(offsetof(ImuSensorRow, bias_a) % 16 == 0);
static_assert(offsetof(ImuSensorRow, bias_g) % 16 == 0);
static_assert(offsetof(ImuSensorRow, noise) % 8 == 0);
static_assert(offsetof(ImuSensorRow, last_index) % 8 == 0);

// Named fields account for every byte: no implicit padding, same discipline as
// state/layout.hpp's BodyState. (rng::Stream carries the same guarantee for its
// own 16 bytes -- see core/rng.hpp.)
static_assert(4 * sizeof(uint32_t) + sizeof(glm::vec3) + sizeof(float) + sizeof(glm::quat) +
                  4 * sizeof(float) + 2 * (sizeof(glm::vec3) + sizeof(float)) + sizeof(rng::Stream) +
                  sizeof(SampleIndex) + sizeof(uint64_t) ==
                  sizeof(ImuSensorRow),
              "ImuSensorRow has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// The stream one sensor draws its noise from, derived exactly once here so the
// pass, the spawn path and any test agree by construction.
//
// The index is the sensor's WORLD-LOCAL slot, not its global arena slot: a
// world's randomness must not depend on where that world sits in the set, or
// the batching-invariance property (D8; engine/testing/replay.hpp) is false.
// ---------------------------------------------------------------------------
[[nodiscard]] constexpr rng::Stream imu_noise_stream(uint64_t world_seed, uint32_t local_slot) noexcept {
    return rng::make_stream(world_seed, kImuNoiseDomainTag, local_slot);
}

// ---------------------------------------------------------------------------
// synthesize_imu() -- the imu.synthesize pass's work, for ONE
// world. Advances every live sensor's rate divider by one substep and, for
// those that reach a rate boundary, draws noise, advances the bias random walk,
// and writes one tick-stamped sample into that sensor's ring.
//
// `bodies` and `sensors` must be spans over the SAME world's partitions (see
// ImuSensorRow::body_slot); `rings` is that world's ring partition, whose size
// is exactly sensors.size() * kRingDepth. `tick` is the step being executed.
//
// SKIPS a sensor whose kind is not `imu` (free or reserved-but-uninitialized
// slots, by the active-high convention) and a sensor whose body is not
// body_flags::active -- WITHOUT advancing its phase or drawing, the same
// posture apply_drag() and integrate_bodies() take toward inactive bodies. A
// deactivated body's sensors therefore resume mid-period rather than
// re-phasing, which is deterministic either way and is the cheaper rule.
//
// A `rate_divider` of 0 behaves as 1 (the comparison below can never hold), so
// a corrupt or hand-built row degrades to "every substep" rather than to a
// division by zero. The spawn path rejects 0 outright.
//
// PRECONDITION, documented and not runtime-checked (same posture as
// apply_drag()): every sensors[i].body_slot with kind == imu is a valid index
// into `bodies`. A caller that builds both spans from the same world's
// partitions satisfies it for free.
//
// Allocates nothing, reads no clock, and draws only from each row's own stream.
// ---------------------------------------------------------------------------
void synthesize_imu(std::span<const BodyState> bodies, std::span<ImuSensorRow> sensors,
                    std::span<ImuSample> rings, uint64_t tick) noexcept;

}  // namespace spade::sensors
