// ---------------------------------------------------------------------------
// sensor_gnss.slang -- GNSS fix synthesis on the device, ported
// from engine/sensors/gnss.cpp's synthesize_gnss(). It is the gnss.synthesize
// pass of the Sensors phase, beside imu.synthesize: neither writes what the
// other reads, so the schedule may order them either way. (Before the module API they
// shared one SensorSynthesis slot as two dispatches of one pass.)
//
// WRITTEN BESIDE ITS CPU TWIN, sentence for sentence. That is the only method
// that makes two implementations agree bit-for-bit, and sensor_imu.slang /
// imu.cpp are the precedent.
//
// ===========================================================================
// WHAT THIS KERNEL DOES NOT CONTAIN, AND WHY THAT IS THE DESIGN
//
// NO exp(). NO DIVISION. gnss.hpp section 2's Gauss-Markov model is
//
//   bias <- bias * exp(-dt/tau) + N(0, sigma_bias * sqrt(1 - exp(-2 dt/tau)))
//
// and both coefficients depend only on `dt` (IMMUTABLE AT CREATION) and `tau`
// (fixed per sensor). They are PER-SENSOR CONSTANTS, computed once on the CPU
// at add_gnss_sensor() time and stored as `bias_retention` and `bias_drive`.
// This kernel multiplies.
//
// That is not an optimisation, it is what makes this port a BIT-IDENTITY claim
// rather than a banded one. fp32_math.slang:83-88: "an OpFDiv is not merely
// less accurate, it is DEVICE-DEPENDENT" -- which is why log32_div() is a
// restoring integer long division, domain-restricted, and must not be reached
// for as a general divide. Other kernels do divide (collision_dynamic.slang),
// but those are banded physics paths where an ulp sits inside a band.
//
//   A RECURSIVE BIAS FILTER DOES NOT TOLERATE ERROR, IT ACCUMULATES IT.
//
// An ulp of retention error compounds on every emission and walks out of any
// band eventually, so this is the one sensor path where "close enough" is a
// different answer next hour. Deleting the computation beats porting it.
//
// ===========================================================================
// NINE DRAWS PER EMITTED FIX, AND NINE IS ODD
//
// bias-walk, position-white, velocity-white, each x/y/z, unconditionally --
// the stream position is a function of the emitted-fix count alone, so a
// noise-free receiver does not shift every other draw in the world.
//
// sensor_imu.slang's TWELVE is even and calls that "a small dividend": the
// Box-Muller cache is empty again at the end of every sample. NINE LEAVES IT
// FULL, and that is still bit-identical across backends, because core/rng.hpp's
// Stream and layouts.slang's RngStream mirror each other INCLUDING
// `cached_gauss`/`has_cached` -- both sides carry the same spare forward.
//
// Said out loud because the neighbouring file's even count reads as a rule:
// AN UNSTATED NON-REQUIREMENT BECOMES A REQUIREMENT BY INHERITANCE.
//
// ===========================================================================
// ONE THREAD PER GLOBAL SENSOR SLOT, over the `gnss_sensors` slot space. Each
// receiver owns its own phase counter, bias state, rng stream and ring window
// and touches no other's, so there is nothing to serialize. Two receivers on
// one body only READ that body. No workgroup-shared anything, so T9's
// workgroup-size invariance holds by construction.
//
// DIVISION AND SQRT SITES: none in this file. rng.slang's Box-Muller sqrt is
// the only banded operation on this path and it arrives through the nine draws.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import mirrors;
import rng;
import u64;

static const uint kInvalidWorld = 0xFFFFFFFFu;
static const uint kBodyFlagActive = 1u;

// sensors/kinds.hpp's sensor_kind:: values. `none` == 0 is ALSO the liveness
// predicate, active-high exactly like BodyState::flags.
static const uint kSensorKindGnss = 2u;

// sensors/rings.hpp:82 -- FIXED AT COMPILE TIME on both sides, which is what
// makes the device buffer's stride a constant rather than a per-world lookup.
static const uint kRingDepth = 64u;

// ---------------------------------------------------------------------------
// gnss.cpp -- three standard normals as a vector, IN A PINNED ORDER: x, then y,
// then z. A FUNCTION, not an initialiser list: `float3(rng_next_gauss(s),
// rng_next_gauss(s), rng_next_gauss(s))` is a determinism bug that compiles,
// because argument evaluation order is unspecified. Three named locals cannot
// be reordered. The hazard and the fix are identical on the CPU side.
// ---------------------------------------------------------------------------
float3 draw_gauss3(inout RngStream stream) {
    const float x = rng_next_gauss(stream);
    const float y = rng_next_gauss(stream);
    const float z = rng_next_gauss(stream);
    return float3(x, y, z);
}

[shader("compute")]
// LOCAL SIZE COMES FROM THE BUILD, NOT FROM THIS LINE. See sensor_imu.slang's
// note: spade_slang_kernel_variants() compiles this kernel once per size in
// SPADE_SLANG_WORKGROUP_SIZES, and there is DELIBERATELY no #ifndef fallback --
// a missing -D must fail the build rather than silently pick a size.
[numthreads(SPADE_WG, 1, 1)]
void sensor_gnss(uint3 tid : SV_DispatchThreadID) {
    const uint slot = tid.x;
    if (slot >= pass_params.world_count * pass_params.sensor_capacity) {
        return;
    }

    // Global sensor slot -> owning world. `gnss_sensors` IS slot-allocated, so
    // this map carries real information and bindings.slang binds it for this
    // dispatch.
    const uint world = gnss_sensors_slot_to_world[slot];
    if (world == kInvalidWorld) {
        return;
    }

    GnssSensorRow row = gnss_sensors[slot];

    // gnss.cpp -- active-high, inert when zero.
    if (row.kind != kSensorKindGnss) {
        return;
    }

    // GnssSensorRow::body_slot is WORLD-LOCAL, like every other row type's, so
    // it is rebased onto this world's `bodies` partition. The CPU side indexes
    // a world-local span directly and reaches the same element.
    const uint body_slot = world * pass_params.body_capacity + row.body_slot;

    // An inactive body's receiver is skipped WITHOUT advancing its phase or
    // drawing -- the same chosen rule as the IMU's, so a deactivated body's
    // sensors resume mid-period rather than re-phasing.
    if ((bodies[body_slot].flags & kBodyFlagActive) == 0u) {
        return;
    }

    // -------------------------------------------------------------------
    // THE RATE BOUNDARY. Spelled `<` so a rate_divider of 0 degrades to "every
    // substep" rather than looping or dividing by zero. This is the first
    // sensor for which the divider is the normal case (gnss.hpp section 3), so
    // this early return is the COMMON path here, not the rare one.
    // -------------------------------------------------------------------
    row.phase += 1u;
    if (row.phase < row.rate_divider) {
        // The ONLY field the CPU changed on this path. Written on its own
        // rather than storing the whole row back, so "this pass wrote nothing
        // else" is true of the BYTES and not merely of the values.
        gnss_sensors[slot].phase = row.phase;
        return;
    }
    row.phase = 0u;

    // gnss.cpp -- the nine draws, in the pinned order, unconditionally.
    const float3 bias_walk = draw_gauss3(row.noise);
    const float3 white_pos = draw_gauss3(row.noise);
    const float3 white_vel = draw_gauss3(row.noise);

    // THE GAUSS-MARKOV ADVANCE -- two multiplies and an add, no exp. Advances
    // once per EMITTED FIX, not per substep: the receiver's own error process
    // runs on the receiver's own clock.
    row.bias = row.bias * row.bias_retention + row.bias_drive * bias_walk;

    // -------------------------------------------------------------------
    // THE TRUE QUANTITIES AT THE ANTENNA, WORLD frame.
    //
    // An antenna is a POINT -- no mount orientation to conjugate through, which
    // is the structural difference from sensor_imu.slang. That kernel works in
    // the BODY frame throughout and never touches `orient`; this one reports in
    // the WORLD frame and must rotate the lever arm through it.
    //
    // omega is stored BODY-frame, so it is rotated FIRST and the cross product
    // is taken with both operands in world frame -- never a mixed one.
    // -------------------------------------------------------------------
    const float4 orient = bodies[body_slot].orient;
    const float3 lever_world = glm_quat_rotate(orient, row.mount_pos);
    const float3 omega_world = glm_quat_rotate(orient, bodies[body_slot].omega_body);
    const float3 pos_true = bodies[body_slot].pos + lever_world;
    const float3 vel_true = bodies[body_slot].vel + glm_cross(omega_world, lever_world);

    // -------------------------------------------------------------------
    // THE MEASUREMENT.
    //
    // HORIZONTAL IS X AND Z; VERTICAL IS Y -- the world frame is right-handed
    // and Y-UP (gnss.hpp section 1). Applying one sigma to all three axes would
    // silently discard the h/v asymmetry the row was shaped around.
    //
    // Left-to-right: true, then bias, then noise. fp32 addition is not
    // associative, so this grouping is part of the parity contract and the CPU
    // twin groups it identically.
    // -------------------------------------------------------------------
    GnssFix fix;
    fix.position = pos_true + row.bias +
                   float3(row.sigma_h * white_pos.x, row.sigma_v * white_pos.y,
                          row.sigma_h * white_pos.z);
    fix.sigma_h = row.sigma_h;
    fix.velocity = vel_true + row.sigma_vel * white_vel;
    fix.sigma_v = row.sigma_v;

    // Monotonic, per sensor, starting at 1. u64_add rather than `+= 1`:
    // `last_index` is a uint2 pair on this device.
    row.last_index = u64_add(row.last_index, uint2(1u, 0u));
    fix.index = row.last_index;
    fix.tick = uint2(step_params[0].tick_lo, step_params[0].tick_hi);

    // ring_write() into THIS receiver's window. kRingDepth is a power of two,
    // so `(index - 1) % 64` is the low six bits of index - 1, which live
    // entirely in the low word -- and the low word of a 64-bit subtraction is
    // the wrapping 32-bit subtraction regardless of any borrow. So this IS the
    // CPU's expression rather than an approximation of it.
    const uint ring_slot = (row.last_index.x - 1u) & (kRingDepth - 1u);
    gnss_ring[slot * kRingDepth + ring_slot] = fix;

    // The four fields the CPU pass mutates, written back FIELD-WISE rather than
    // as a whole-row store: everything else (body_slot, kind, rate_divider,
    // mount_pos, the three sigmas, bias_tau_s, bias_retention, bias_drive,
    // _reserved0 and the pads) is configuration this pass must leave exactly as
    // it found it, and a whole-row store would make that a property of the
    // local copy rather than of the code.
    gnss_sensors[slot].phase = row.phase;
    gnss_sensors[slot].bias = row.bias;
    gnss_sensors[slot].noise = row.noise;
    gnss_sensors[slot].last_index = row.last_index;
}
