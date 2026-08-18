// ---------------------------------------------------------------------------
// integrate.slang (S6 Task 6) -- the Integrate pass, ported. One thread per
// GLOBAL body slot; one substep of semi-implicit (symplectic) Euler plus the
// specific-force capture the IMU pass will read.
//
// THE MIRRORED SOURCE IS engine/physics/integrator.cpp, whose numbered comments
// its own header calls "normative for that mirror". Every step below cites the
// line range it reproduces. Two groupings that look like free choices and are
// not, quoted from that file because they are the parity contract:
//
//   * step 1 DIVIDES by mass. It does not multiply by a precomputed 1/mass:
//     for a general float m, fl(x / m) != fl(x * fl(1/m)).
//   * the parentheses in steps 3 and 4 are load-bearing: (a + g) * h and
//     a*h + g*h are different numbers in fp32.
//
// ---------------------------------------------------------------------------
// GLM IS PART OF THE CONTRACT, NOT AN IMPLEMENTATION DETAIL. The CPU twin
// expresses every vector and quaternion operation through glm, and each glm
// helper has ONE pinned expression. The helpers below reproduce those
// expressions literally (glm 1.0.1, vendored; file and reasoning per helper).
// Writing `mul()`, `dot()` or `normalize()` instead would be a different
// summation order -- and OpDot is forbidden outright anyway
// (engine/testing/spirv_scan.hpp rule P2: SPIR-V does not specify a dot
// product's accumulation order, and slangc does not decorate it
// NoContraction).
//
// ---------------------------------------------------------------------------
// DIVISION AND SQRT SITES (allowed here -- this is a downstream kernel, not
// fp32_math; the global constraint's "downstream kernels MAY use div/sqrt" is
// exactly why CPU<->GPU is banded). Each mirrors the CPU's own operation:
//
//   DIV   force_acc / mass                        (integrator.cpp:46)
//         1.0f / inv_inertia_diag[i]              (integrator.cpp:104-106)
//         half_angle_sq / 6.0f                    (math_ops.cpp:25)
//         sin32(half_angle) / half_angle          (math_ops.cpp:43)
//         1.0f / length(q)                        (glm normalize(qua))
//   SQRT  length(omega_body)                      (math_ops.cpp:15)
//         length(q) inside normalize              (glm normalize(qua))
//
// TRANSCENDENTALS: sin32/cos32 from the T4 Slang port, NEVER the driver's --
// the GPU corollary of the bit-portability rule. math_ops.cpp already calls the
// CPU kernels of the same name, so this is a port, not a substitution.
//
// ---------------------------------------------------------------------------
// THIS KERNEL IS ALSO THE FIRST DEVICE-SIDE CONSUMER OF StepParams (S6 Task 5,
// review round 1, finding C2 -- deferred there, closed here). See
// engine/compute/step_params.hpp's StepWitness doc comment for why wave A
// publishes a witness rather than inventing a physical dependency on a tick
// none of its four passes needs, and for what each witness field proves.
//
// NO WORKGROUP-SHARED ANYTHING: every write is per-item (one body slot per
// thread) and the single witness write is made by exactly one thread, so
// nothing here depends on the local size -- Task 9's {32, 64, 128} invariance
// holds by construction.
// ---------------------------------------------------------------------------

import bindings;
import fp32_math;
// The glm expression mirrors this kernel used to declare for itself
// (glm_cross/glm_length3/glm_quat_conjugate/glm_quat_rotate/glm_quat_mul/
// glm_quat_normalize). ONE definition as of S6 Task 8 -- mirrors.slang's header
// carries the T7 review finding (minor M3) and the per-helper glm citations
// that used to live below.
import mirrors;

// state/arenas.hpp's kInvalidWorld -- a SENTINEL, not a layout, so it is
// spelled locally here, exactly as every other kernel that needs it does
// (layouts.slang mirrors byte images; a constant is not one).
static const uint kInvalidWorld = 0xFFFFFFFFu;

// physics/integrator.hpp's body_flags::active (slot 0, ACTIVE-HIGH so a
// zero-filled arena slot is inert).
static const uint kBodyFlagActive = 1u;

// ---------------------------------------------------------------------------
// core/math_ops.cpp's integrate_orientation(), lines 7-48: the quaternion
// exponential map, exact for omega constant across the substep, renormalized
// on the way out so drift cannot accumulate.
//
// The small-angle branch (theta < 1e-6f) is the Taylor pair that avoids the
// sin(x)/x removable singularity; the two branches agree to fp32 precision at
// the switchover. BOTH branches are ported -- dropping the small-angle one
// would make every near-zero-omega body (every ball in `bounce`, every body of
// `gate`) take the sin32/cos32 path the CPU does not take.
// ---------------------------------------------------------------------------
float4 integrate_orientation(float4 q, float3 omega_body, float dt) {
    // math_ops.cpp:14 -- `0.5f * dt * omega_body` associates as
    // `(0.5f * dt) * omega_body`: a scalar product first, then the broadcast.
    const float3 half_dt_omega = (0.5f * dt) * omega_body;
    const float theta = glm_length3(omega_body) * dt;
    const float half_angle = 0.5f * theta;

    float cos_half;
    float sinc_half;
    if (theta < 1e-6f) {
        const float half_angle_sq = half_angle * half_angle;
        cos_half = 1.0f - 0.5f * half_angle_sq;
        sinc_half = 1.0f - half_angle_sq / 6.0f;
    } else {
        cos_half = cos32(half_angle);
        sinc_half = sin32(half_angle) / half_angle;
    }

    // glm::quat delta{cos_half, sinc_half * half_dt_omega} -- the (w, vec3)
    // constructor, stored (x, y, z, w).
    const float3 delta_v = sinc_half * half_dt_omega;
    const float4 delta = float4(delta_v.x, delta_v.y, delta_v.z, cos_half);
    return glm_quat_normalize(glm_quat_mul(q, delta));
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
void integrate(uint3 tid : SV_DispatchThreadID) {
    const uint slot = tid.x;
    if (slot >= pass_params.world_count * pass_params.body_capacity) {
        return;
    }

    // ---------------------------------------------------------------------
    // The StepParams round trip (see this file's header note and
    // compute/step_params.hpp). Deliberately BEFORE the liveness and activity
    // checks below: the witness must be written on every dispatch of this
    // pass regardless of whether slot 0 happens to hold a live body, so a
    // test can read it after any run. Exactly one thread writes it, so there
    // is no ordering question and no dependence on the workgroup size.
    // ---------------------------------------------------------------------
    if (slot == 0) {
        step_witness[0].tick_lo = step_params[0].tick_lo;
        step_witness[0].tick_hi = step_params[0].tick_hi;
        step_witness[0].substep = pass_params.substep;
        step_witness[0]._r0 = 0u;
    }

    // Global body slot -> owning world (bindings.slang section B: the dispatch
    // is over the global slot space and liveness is not otherwise knowable).
    const uint world = bodies_slot_to_world[slot];
    if (world == kInvalidWorld) {
        return;
    }

    // integrator.cpp:22-24 -- an inert slot is left byte-for-byte untouched,
    // INCLUDING its accumulators (step 7 does not clear a skipped body's).
    const uint flags = bodies[slot].flags;
    if ((flags & kBodyFlagActive) == 0u) {
        return;
    }

    // integrator.cpp:14 -- hoisted once, per world, exactly as the CPU hoists
    // it outside the body loop.
    const float3 gravity = world_params[world].gravity;
    const float h = pass_params.h;

    const float3 force_acc = bodies[slot].force_acc;
    const float3 torque_acc = bodies[slot].torque_acc;
    const float4 orient = bodies[slot].orient;
    const float3 omega_body = bodies[slot].omega_body;
    const float3 inv_i = bodies[slot].inv_inertia_diag;
    const float mass = bodies[slot].mass;

    // 1. External (non-gravitational) acceleration, world frame -- a DIVISION
    //    by mass, never a multiply by a reciprocal (integrator.cpp:42-46).
    const float3 accel_ext = float3(force_acc.x / mass, force_acc.y / mass, force_acc.z / mass);

    // 2. Specific force, body frame, from the PRE-update orientation
    //    (integrator.cpp:48-76). The conjugate of a unit quaternion is its
    //    inverse, so this takes the world-frame acceleration into the body
    //    frame the sensor is bolted to.
    bodies[slot].specific_force = glm_quat_rotate(glm_quat_conjugate(orient), accel_ext);

    // 3. Linear velocity. Gravity enters the dynamics HERE and only here --
    //    the Gravity pass slot is inert on both backends (integrator.cpp:78-81,
    //    physics/schedule.cpp's pass_gravity()).
    const float3 vel = bodies[slot].vel + (accel_ext + gravity) * h;
    bodies[slot].vel = vel;

    // 4a. The layout stores INVERSE principal moments; recover I_body for the
    //     gyroscopic term by componentwise reciprocal, guarding 1/0 to a
    //     finite 0 so 4c's multiply cannot produce 0 * inf == NaN
    //     (integrator.cpp:102-110).
    const float3 i_diag = float3(inv_i.x != 0.0f ? 1.0f / inv_i.x : 0.0f,
                                 inv_i.y != 0.0f ? 1.0f / inv_i.y : 0.0f,
                                 inv_i.z != 0.0f ? 1.0f / inv_i.z : 0.0f);

    // 4b. Total body-frame torque, at the PRE-update omega. `I_body * omega`
    //     is glm's mat3*vec3 over a DIAGONAL matrix, and the two exactly-zero
    //     off-diagonal products are written out rather than folded away: they
    //     are real fp32 additions in the CPU expression
    //     (`m[0][i]*v.x + m[1][i]*v.y + m[2][i]*v.z`, left-associated), and
    //     `x + (-0.0f)` versus `x` differ for x == +0.0f.
    const float3 i_omega =
        float3((i_diag.x * omega_body.x + 0.0f * omega_body.y) + 0.0f * omega_body.z,
               (0.0f * omega_body.x + i_diag.y * omega_body.y) + 0.0f * omega_body.z,
               (0.0f * omega_body.x + 0.0f * omega_body.y) + i_diag.z * omega_body.z);
    // math_ops.cpp:64-66 -- gyroscopic_torque = -cross(omega, I*omega).
    const float3 gyroscopic = -glm_cross(omega_body, i_omega);
    const float3 torque_total = torque_acc + gyroscopic;

    // 4c. Componentwise: the inertia is diagonal in the body frame
    //     (integrator.cpp:123-125). `(inv_I * torque_total) * h` -- the
    //     componentwise product FIRST, then the substep scale.
    const float3 omega_new =
        omega_body + (float3(inv_i.x * torque_total.x, inv_i.y * torque_total.y,
                             inv_i.z * torque_total.z)) * h;
    bodies[slot].omega_body = omega_new;

    // 5. Position, from the UPDATED velocity -- the whole of "symplectic"
    //    (integrator.cpp:127-131).
    bodies[slot].pos = bodies[slot].pos + vel * h;

    // 6. Orientation, from the UPDATED angular velocity (integrator.cpp:133-138).
    bodies[slot].orient = integrate_orientation(orient, omega_new, h);

    // 7. Consume the wrench: the accumulators are per-substep, so the next
    //    substep's force passes start from a clean zero (integrator.cpp:140-145).
    bodies[slot].force_acc = float3(0.0f, 0.0f, 0.0f);
    bodies[slot].torque_acc = float3(0.0f, 0.0f, 0.0f);
}
