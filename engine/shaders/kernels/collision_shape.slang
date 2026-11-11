// ---------------------------------------------------------------------------
// collision_shape.slang (L68) -- the rigid_contact.resolve pass, ported from engine/physics/rigid_contact.cpp's
// resolve_rigid_contacts(). One thread per GLOBAL body slot: a body with a shape row (the set-1 binding contact_shapes, one 288-byte row per body
// slot) finds the contacts of its points against the world SDF once, sweeps the velocity solve over them with accumulated impulses (the normal one
// clamped at >= 0, the friction one inside the cone |j_t| <= mu j_n), and takes the positional correction of its deepest point.
//
// THE NUMBERED OP ORDER IN rigid_contact.cpp (A: the contacts, B: the sweeps, C: the correction) IS THE PARITY CONTRACT; every line below cites the
// step it mirrors, and each expression keeps the CPU's grouping (glm's operators, spelled with mirrors.slang's glm_* functions).
//
// DIVISIONS AND SQUARE ROOTS go through fp32_math.slang's div32_ftz and sqrt32_ftz (the CPU's divss and sqrtss), not the driver's own, so the
// CPU-to-device gap in this pass is the SDF walk's (sdf_eval.slang) and nothing of this kernel's. Sites: |gradient|, gradient / |gradient|, 1 / mass,
// the normal impulse's (bias - v_n) / K_n, |v_t|, v_t / |v_t|, |v_t| / K_t, |next|, cap / |next|, and |g| of the settle floor.
//
// The row (physics/rigid_contact.hpp's ShapeRow, as 72 floats): [0] kind, [1] count, [2] radius, [3] rest_speed, [4] friction, [5] restitution,
// [6] iterations, [7] reserved, then 16 points of {x, y, z, 0} from word 8. A row of all zeros is "no shape" and the thread leaves.
//
// NO WORKGROUP-SHARED ANYTHING: one body per thread, no cross-lane traffic; the per-contact arrays are thread-local.
// ---------------------------------------------------------------------------
import layouts;
import bindings;
import sdf_eval;
import fp32_math;
import mirrors;

static const uint kInvalidWorld = 0xFFFFFFFFu;
static const uint kBodyFlagActive = 1u;
static const uint kShapeRowFloats = 72u;
static const uint kShapeMaxPoints = 16u;
static const uint kDefaultSweeps = 4u;
static const uint kMaxSweeps = 16u;

float len3_ftz(float3 v) { return sqrt32_ftz((v.x * v.x + v.y * v.y) + v.z * v.z); }

float3 div3_ftz(float3 v, float s) { return float3(div32_ftz(v.x, s), div32_ftz(v.y, s), div32_ftz(v.z, s)); }

[shader("compute")]
// LOCAL SIZE COMES FROM THE BUILD (cmake/SpadeSlang.cmake's spade_slang_kernel_variants), as collision_static's does.
[numthreads(SPADE_WG, 1, 1)]
void collision_shape(uint3 tid : SV_DispatchThreadID) {
    const uint slot = tid.x;
    if (slot >= pass_params.world_count * pass_params.body_capacity) {
        return;
    }
    const uint world = bodies_slot_to_world[slot];
    if (world == kInvalidWorld) {
        return;
    }
    // An inert slot is left untouched; a body with no shape is skipped by this one test.
    if ((bodies[slot].flags & kBodyFlagActive) == 0u) {
        return;
    }
    const uint base = slot * kShapeRowFloats;
    const uint count_word = asuint(contact_shapes[base + 1u]);
    if (count_word == 0u) {
        return;
    }
    if (!(bodies[slot].mass > 0.0f)) {
        return;  // no inverse mass to apply an impulse with
    }

    const ContactParams material = contact_params[world];
    const float beta = material.baumgarte_beta;
    const float slop = material.slop;
    const float3 gravity = world_params[world].gravity;
    // The settle floor: 2 |g| h, grouped (2 |g|) h as the CPU writes it.
    const float settle_floor = (2.0f * len3_ftz(gravity)) * pass_params.h;

    const float row_radius = contact_shapes[base + 2u];
    const float row_rest = contact_shapes[base + 3u];
    const float row_mu = contact_shapes[base + 4u];
    const float row_e = contact_shapes[base + 5u];
    const uint row_sweeps = asuint(contact_shapes[base + 6u]);

    const float e = (row_e >= 0.0f) ? row_e : material.restitution_e;
    const float mu = (row_mu >= 0.0f) ? row_mu : material.friction_mu;
    const float rest_speed = glm_max((row_rest > 0.0f) ? row_rest : 0.1f, settle_floor);
    const float radius = row_radius;
    const float inv_m = div32_ftz(1.0f, bodies[slot].mass);
    const float3 inv_i = bodies[slot].inv_inertia_diag;
    const float4 orient = bodies[slot].orient;
    const float4 orient_conj = glm_quat_conjugate(orient);
    const float3 pos = bodies[slot].pos;
    const float3 vel_in = bodies[slot].vel;
    float3 vel = vel_in;
    float3 omega = bodies[slot].omega_body;
    const uint sweeps = (row_sweeps == 0u) ? kDefaultSweeps : ((kMaxSweeps < row_sweeps) ? kMaxSweeps : row_sweeps);

    // A. THE CONTACTS, found once. STRICT `<`, a NaN distance fails; |grad| == 0 has no normal and is skipped.
    float3 c_n[16];
    float3 c_r[16];
    float c_kn[16];
    float c_bias[16];
    float c_acc_n[16];
    float3 c_acc_t[16];
    uint found = 0u;
    float deepest = 0.0f;
    float3 deepest_n = float3(0.0f, 0.0f, 0.0f);
    const uint count = (count_word < kShapeMaxPoints) ? count_word : kShapeMaxPoints;
    for (uint k = 0u; k < count; ++k) {
        const uint at = base + 8u + k * 4u;
        const float3 local = float3(contact_shapes[at], contact_shapes[at + 1u], contact_shapes[at + 2u]);
        const float3 p = pos + glm_quat_rotate(orient, local);  // contact_point_world
        const SdfSample s = sdf_sample(world, p);
        if (!(s.distance < radius)) {
            continue;
        }
        const float grad_len = len3_ftz(s.gradient);
        if (!(grad_len > 0.0f)) {
            continue;
        }
        const float3 n = div3_ftz(s.gradient, grad_len);
        const float depth = radius - s.distance;
        const float3 lever = (p - n * radius) - pos;
        // The restitution bias from the approach speed BEFORE any impulse of this substep; 0 (plastic) under the rest speed.
        const float3 v_c = vel + glm_cross(glm_quat_rotate(orient, omega), lever);
        const float v_n0 = glm_dot3(v_c, n);
        const float3 a = glm_quat_rotate(orient_conj, glm_cross(lever, n));
        c_n[found] = n;
        c_r[found] = lever;
        c_kn[found] = inv_m + glm_dot3(a, a * inv_i);
        c_bias[found] = ((-v_n0) > rest_speed) ? e * (-v_n0) : 0.0f;
        c_acc_n[found] = 0.0f;
        c_acc_t[found] = float3(0.0f, 0.0f, 0.0f);
        // The deepest point (strict `>`: the first in row order wins a tie) takes the positional correction.
        if (found == 0u || depth > deepest) {
            deepest = depth;
            deepest_n = n;
        }
        ++found;
    }
    if (found == 0u) {
        return;
    }

    // B. THE SWEEPS.
    for (uint it = 0u; it < sweeps; ++it) {
        for (uint c = 0u; c < found; ++c) {
            const float3 n = c_n[c];
            const float3 lever = c_r[c];
            // Normal: drive the point's normal speed to its bias; the accumulated impulse stays >= 0.
            float3 v_c = vel + glm_cross(glm_quat_rotate(orient, omega), lever);
            const float v_n = glm_dot3(v_c, n);
            const float wanted = c_acc_n[c] + div32_ftz(c_bias[c] - v_n, c_kn[c]);
            const float acc_n = glm_max(wanted, 0.0f);
            const float j_n = acc_n - c_acc_n[c];
            c_acc_n[c] = acc_n;
            vel = vel + (j_n * inv_m) * n;
            omega = omega + inv_i * glm_quat_rotate(orient_conj, glm_cross(lever, j_n * n));

            // Friction on the velocity after that: one tangent; the accumulated impulse stays in the cone |j_t| <= mu acc_n.
            v_c = vel + glm_cross(glm_quat_rotate(orient, omega), lever);
            const float3 v_t = v_c - glm_dot3(v_c, n) * n;
            const float v_t_len = len3_ftz(v_t);
            if (v_t_len > 0.0f) {
                const float3 t = div3_ftz(v_t, v_t_len);
                const float3 b = glm_quat_rotate(orient_conj, glm_cross(lever, t));
                const float k_t = inv_m + glm_dot3(b, b * inv_i);
                float3 next = c_acc_t[c] - div32_ftz(v_t_len, k_t) * t;
                const float cap = mu * acc_n;
                const float next_len = len3_ftz(next);
                if (next_len > cap) {
                    next = next * div32_ftz(cap, next_len);
                }
                const float3 dj = next - c_acc_t[c];
                c_acc_t[c] = next;
                vel = vel + inv_m * dj;
                omega = omega + inv_i * glm_quat_rotate(orient_conj, glm_cross(lever, dj));
            }
        }
    }

    // C. THE POSITIONAL CORRECTION of the deepest point, on `pos`; and the linear velocity change to the contact scratch (PHY-7).
    bodies[slot].vel = vel;
    bodies[slot].omega_body = omega;
    bodies[slot].pos = pos + deepest_n * (beta * glm_max(deepest - slop, 0.0f));
    contact_dv[slot] = float4(contact_dv[slot].xyz + (vel - vel_in), 0.0f);
}
