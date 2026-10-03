// ---------------------------------------------------------------------------
// sensor_imu.slang (S6 Task 8) -- the imu.synthesize pass, ported from
// engine/sensors/imu.cpp's synthesize_imu() (declared at sensors/imu.hpp:442).
// A pass of the Sensors phase, beside gnss.synthesize: the two share no
// quantity, so the schedule may order them either way.
//
// TWO-INSTANT SYNTHESIS PLUS A RING WRITE, and the two instants are the port's
// most important inherited property rather than a wrinkle. sensors/imu.hpp
// section 3 states it at length: this pass runs AFTER Integrate, so
// `specific_force` is a SUBSTEP-START quantity (integrator.cpp step 2 captured
// it against the PRE-update orientation, deliberately) while `omega_body` is a
// SUBSTEP-END one (integrator.cpp step 4c has already advanced it). One
// ImuSample therefore mixes instants, by a documented and accepted alpha*h.
// This kernel reproduces that EXACTLY -- it reads the same two fields at the
// same point in the chain -- and reproducing it is the whole job. A port that
// "fixed" the asymmetry would be a different sensor.
//
// ===========================================================================
// ONE THREAD PER GLOBAL SENSOR SLOT -- a THIRD dispatch grid, and the reason
// this file needs one.
//
// Seven of the schedule's dispatches run one thread per BODY slot, and the
// CollisionDynamic chain adds a per-key-entry and a per-WORLD grid. This pass
// is dispatched over the `imu_sensors` slot space (world_count *
// sensor_capacity), because that is the array whose rows it advances and the
// axis along which the work is genuinely independent: each sensor owns its own
// phase counter, its own bias random walk, its own rng stream and its own ring
// window, and touches no other sensor's. There is nothing to serialize and no
// accumulator to race over.
//
// WHY THAT IS SAFE EVEN THOUGH SENSORS SHARE BODIES. Two IMUs bolted to one
// body would run in two threads -- and both only READ that body (`bodies`
// narrows to a const span on the CPU side too, for exactly this reason: "this
// pass READS body state and writes only sensor rows and rings"). No write
// hazard exists to order.
//
// NO WORKGROUP-SHARED ANYTHING: every write is per-item, so T9's
// workgroup_size {32, 64, 128} invariance holds by construction.
//
// ===========================================================================
// THE TWELVE DRAWS, AND WHY THE COUNT IS PART OF THE CONTRACT
//
// imu.hpp section 4: "Exactly TWELVE standard normals are consumed per EMITTED
// sample, always, whatever the sigmas are", in the order accel-bias-walk,
// gyro-bias-walk, accel-white, gyro-white, each x/y/z. A sensor configured
// noise-free occupies the same stream positions as a noisy one -- which is what
// keeps turning noise off from perturbing anything else in the world.
//
// The count being EVEN is a small dividend the port inherits: rng::Stream's
// Box-Muller cache is empty again at the end of every sample, so a row's 16
// stream bytes are in the same canonical shape after each emission on both
// backends. (Those bytes are compared directly by the parity table, so this is
// load-bearing rather than decorative.)
//
// draw_gauss3() exists on both sides to FORCE THE SEQUENCING. imu.cpp's own
// note: writing `vec3(s.next_gauss(), s.next_gauss(), s.next_gauss())` would be
// a determinism bug that compiles, because argument evaluation order is
// unspecified. Three named locals cannot be reordered. The same hazard exists
// in Slang, and the same fix is applied.
//
// ===========================================================================
// THE RING, AND THE ONE ARITHMETIC IDENTITY THAT MAKES IT WORK WITHOUT INT64
//
// sensors/rings.hpp: sensor slot g owns ring slots [g*kRingDepth,
// (g+1)*kRingDepth), and a sample lands at `(index - 1) % kRingDepth`. `index`
// is a 64-bit monotonic counter, mirrored here as a uint2 (shaders/shared/
// layouts.slang's header note -- this device has no shaderInt64).
//
// kRingDepth is 64, a power of two, so `(index - 1) % 64` is the LOW SIX BITS
// of index - 1 -- which live entirely in the low word, and the low word of a
// 64-bit subtraction is the wrapping 32-bit subtraction regardless of any
// borrow. So `(index.x - 1) & 63` IS the CPU's expression, not an approximation
// of it, and the case index.x == 0 (which borrows) is covered rather than
// excluded: 0 - 1 wraps to 0xFFFFFFFF on both sides.
//
// ===========================================================================
// DIVISION AND SQRT SITES: none in this file. rng.slang's Box-Muller sqrt is
// the only banded operation on this path, and it reaches here through the
// twelve draws.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import mirrors;
import rng;
import u64;

static const uint kInvalidWorld = 0xFFFFFFFFu;
static const uint kBodyFlagActive = 1u;

// sensors/imu.hpp's sensor_kind:: values. `none` == 0 is ALSO the liveness
// predicate, active-high exactly like BodyState::flags -- a zero-filled slot
// (freed, or reserved but not yet initialized) has kind == none and is skipped
// without any further lookup. That is why ImuSensorRow has no separate
// `enabled` field.
static const uint kSensorKindImu = 1u;

// sensors/rings.hpp:82 -- the ring depth, in samples. FIXED AT COMPILE TIME on
// both sides, which is what makes the device buffer's stride a constant rather
// than a per-world lookup.
static const uint kRingDepth = 64u;

// ---------------------------------------------------------------------------
// imu.cpp:22-27 -- three standard normals as a vector, IN A PINNED ORDER: x,
// then y, then z. See this file's header for why it is a function.
// ---------------------------------------------------------------------------
float3 draw_gauss3(inout RngStream stream) {
    const float x = rng_next_gauss(stream);
    const float y = rng_next_gauss(stream);
    const float z = rng_next_gauss(stream);
    return float3(x, y, z);
}

[shader("compute")]
// LOCAL SIZE COMES FROM THE BUILD, NOT FROM THIS LINE (S6 Task 9b).
// cmake/SpadeSlang.cmake's spade_slang_kernel_variants() compiles this kernel
// once per size in SPADE_SLANG_WORKGROUP_SIZES (= compute/backend.hpp's
// kSupportedWorkgroupSizes), passing -DSPADE_WG=<size>, so that
// BackendDesc::workgroup_size selects a module that was actually compiled for
// it. There is DELIBERATELY no #ifndef fallback: a missing -D must fail the
// build loudly, not silently produce a 64-wide kernel that step_recorder.cpp
// then dispatches on a grid divided by some other number.
[numthreads(SPADE_WG, 1, 1)]
void sensor_imu(uint3 tid : SV_DispatchThreadID) {
    const uint slot = tid.x;
    if (slot >= pass_params.world_count * pass_params.sensor_capacity) {
        return;
    }

    // Global sensor slot -> owning world. `imu_sensors` IS slot-allocated
    // (alloc_slot/free_slot), so unlike `dryden` this map carries real
    // information and bindings.slang binds it for exactly this dispatch
    // (its section B: "SensorSynthesis dispatches per sensor").
    const uint world = imu_sensors_slot_to_world[slot];
    if (world == kInvalidWorld) {
        return;
    }

    ImuSensorRow row = imu_sensors[slot];

    // imu.cpp:38 -- active-high, inert when zero.
    if (row.kind != kSensorKindImu) {
        return;
    }

    // imu.cpp:42 -- ImuSensorRow::body_slot is WORLD-LOCAL (imu.hpp), exactly
    // like DragBodyRow::body_slot and RotorRow::body_slot, so it is rebased
    // onto this world's `bodies` partition. The precondition that it is a valid
    // index is documented and not runtime-checked on the CPU side either.
    const uint body_slot = world * pass_params.body_capacity + row.body_slot;

    // imu.cpp:43 -- an inactive body's sensor is skipped WITHOUT advancing its
    // phase or drawing. imu.hpp is explicit that this is the chosen rule and
    // what it implies: "A deactivated body's sensors therefore resume
    // mid-period rather than re-phasing, which is deterministic either way and
    // is the cheaper rule."
    if ((bodies[body_slot].flags & kBodyFlagActive) == 0u) {
        return;
    }

    // -------------------------------------------------------------------
    // THE RATE BOUNDARY (imu.cpp:51-53). `phase` counts substeps within the
    // current sensor period; a sample is emitted on the substep that completes
    // it. Spelled `<` so a rate_divider of 0 degrades to "every substep" rather
    // than looping or dividing by zero -- the comparison can never hold.
    // -------------------------------------------------------------------
    row.phase += 1u;
    if (row.phase < row.rate_divider) {
        // The ONLY field the CPU changed on this path. Written on its own
        // rather than storing the whole row back, so "this pass wrote nothing
        // else" is true of the bytes and not merely of the values.
        imu_sensors[slot].phase = row.phase;
        return;
    }
    row.phase = 0u;

    // imu.cpp:67-70 -- the twelve draws, in the pinned order, unconditionally.
    const float3 bias_walk_a = draw_gauss3(row.noise);
    const float3 bias_walk_g = draw_gauss3(row.noise);
    const float3 white_a = draw_gauss3(row.noise);
    const float3 white_g = draw_gauss3(row.noise);

    // imu.cpp:74-75 -- the random walk advances once per SAMPLE (not per
    // substep): it is the sensor's own error process, so it runs on the
    // sensor's own clock.
    row.bias_a = row.bias_a + row.sigma_ba * bias_walk_a;
    row.bias_g = row.bias_g + row.sigma_bg * bias_walk_g;

    // -------------------------------------------------------------------
    // THE TRUE QUANTITIES AT THE MOUNT, body frame (imu.cpp:98-108).
    //
    // The centripetal term omega x (omega x r) is the lever arm's whole v2
    // contribution; the Euler term alpha x r is deliberately absent (imu.hpp
    // section 2) -- zero for a COM mount and zero at zero spin, so the common
    // case pays nothing for it either way.
    //
    // THE TWO READS ARE NOT AT THE SAME INSTANT; see this file's header.
    // -------------------------------------------------------------------
    const float3 omega = bodies[body_slot].omega_body;
    const float3 centripetal = glm_cross(omega, glm_cross(omega, row.mount_pos));
    const float3 specific_force_mount_body = bodies[body_slot].specific_force + centripetal;

    // mount_orient is mount -> body, so its CONJUGATE reads a body-frame vector
    // in the mount frame -- the same construction, for the same reason, as
    // integrator.cpp's conjugate(orient) reaching the body frame from the world
    // frame.
    const float4 body_to_mount = glm_quat_conjugate(row.mount_orient);
    const float3 accel_true = glm_quat_rotate(body_to_mount, specific_force_mount_body);
    const float3 gyro_true = glm_quat_rotate(body_to_mount, omega);

    // -------------------------------------------------------------------
    // THE MEASUREMENT (imu.cpp:117-121). Bias and white noise are added AFTER
    // the rotation into the mount frame, because both are properties of the
    // SENSOR (its own axes), not of the body. Left-to-right: true, then bias,
    // then noise -- fp32 addition is not associative, so this grouping is part
    // of the parity contract like every other op order in the engine.
    // -------------------------------------------------------------------
    ImuSample sample;
    sample.accel = accel_true + row.bias_a + row.sigma_a * white_a;
    sample._p0 = 0.0f;
    sample.gyro = gyro_true + row.bias_g + row.sigma_g * white_g;
    sample._p1 = 0.0f;

    // imu.cpp:124-126 -- monotonic, per sensor, starting at 1 (editor tech spec
    // TA5). u64_add rather than `+= 1`: `last_index` is a uint2 pair on this
    // device (see this file's header and layouts.slang's).
    row.last_index = u64_add(row.last_index, uint2(1u, 0u));
    sample.index = row.last_index;
    sample.tick = uint2(step_params[0].tick_lo, step_params[0].tick_hi);

    // ring_write() (sensors/rings.hpp:111) into THIS sensor's window. See the
    // header for why the modulo reduces to a mask on the low word.
    const uint ring_slot = (row.last_index.x - 1u) & (kRingDepth - 1u);
    imu_ring[slot * kRingDepth + ring_slot] = sample;

    // The five fields the CPU pass mutates, written back field-wise rather than
    // as a whole-row store: everything else in the row (body_slot, kind,
    // rate_divider, the mount pose, the four sigmas, `_reserved0` and the three
    // std430 pads) is configuration this pass must leave exactly as it found
    // it, and a whole-row store would make that a property of the local copy
    // rather than of the code.
    imu_sensors[slot].phase = row.phase;
    imu_sensors[slot].bias_a = row.bias_a;
    imu_sensors[slot].bias_g = row.bias_g;
    imu_sensors[slot].noise = row.noise;
    imu_sensors[slot].last_index = row.last_index;
}
