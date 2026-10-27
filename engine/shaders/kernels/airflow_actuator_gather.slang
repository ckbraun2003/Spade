// ---------------------------------------------------------------------------
// airflow_actuator_gather.slang (plan B, task B4) -- the rotor discs on the
// GPU: engine/physics/airflow/actuator.cpp's spread_actuator(), term for term
// (its CPU twin, and the authority for every expression), as a gather: each
// face of the dispatched component sums Peskin's 4-point weights over every
// disc's points itself, so nothing scatters and nothing is atomic.
//
// One component per dispatch, one invocation per face of it (1D, guarded by
// the push's face_count, word 2 as in every airflow kernel). Each face reads
// its increment, adds each disc's contribution in disc order, as the CPU's
// spread_actuator() calls do one disc at a time, and writes it back. A disc
// is skipped for a component its axis has no part of and that has no swirl,
// and for a face outside its reach box, as the CPU skips them.
//
// Reads airflow_params and airflow_discs; reads and writes
// airflow_increments (du, dv, dw packed like the velocity: u at 0, v at
// faces_u, w after). The fluid step's chain clears the increments, as
// fluid_step's add_and_clear() does.
//
// DISCS. One record of kDiscStride floats per disc, packed on the host from
// disc_spread() (actuator.hpp), the CPU's own points, tangents and scales:
//   [0-2] centre, grid frame   [3] reach   [4-6] axis   [7] scale
//   [8] swirl_scale   [9] swirl (1 or 0)   [10-15] unused
//   [16-159] the 48 points' xyz   [160-303] the 48 tangents-over-radius' xyz
//
// PARITY. -fp-mode precise keeps the arithmetic uncontracted, every value
// comes from the host or is a correctly rounded add, subtract or multiply,
// and peskin4's square roots are fp32_math's sqrt32_ftz, sqrtss under the
// CPU's FTZ|DAZ to the bit: the gather is spread_actuator()'s bit for bit
// (TD-14 as amended 2026-10-22), and the module is scanned under the exact
// profile, so no bare sqrt can return.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct GatherPush {
    uint component;   // 0 u, 1 v, 2 w
    uint unused;
    uint face_count;  // the component's faces
    uint disc_count;
};

[[vk::push_constant]] ConstantBuffer<GatherPush> gather_push;

// --- the disc records ------------------------------------------------------
static const uint kDiscStride = 304u;
static const uint kDiscPoints = 48u;
static const uint kPointsAt = 16u;
static const uint kTangentsAt = 160u;

float3 disc_vec3(uint at) { return float3(airflow_discs[at], airflow_discs[at + 1u], airflow_discs[at + 2u]); }

// Component c's samples along x and y (its own axis holds n + 1).
uint samples_x(uint c) { return c == 0u ? grid_dims().x + 1u : grid_dims().x; }
uint samples_y(uint c) { return c == 1u ? grid_dims().y + 1u : grid_dims().y; }

// Where component c's faces start in airflow_increments.
uint component_base(uint c) {
    const uint3 n = grid_dims();
    const uint faces_u = (n.x + 1u) * n.y * n.z;
    const uint faces_v = n.x * (n.y + 1u) * n.z;
    return c == 0u ? 0u : (c == 1u ? faces_u : faces_u + faces_v);
}

// Face `index` of component c: its point in the grid's frame
// (for_each_sample's expression).
float3 face_point(uint c, uint index) {
    const uint cx = samples_x(c);
    const uint cy = samples_y(c);
    const uint i = index % cx;
    const uint j = (index / cx) % cy;
    const uint k = index / (cx * cy);
    const float ox = c == 0u ? 0.0 : 0.5;
    const float oy = c == 1u ? 0.0 : 0.5;
    const float oz = c == 2u ? 0.0 : 0.5;
    const float dx = grid_dx();
    return float3((float(i) + ox) * dx, (float(j) + oy) * dx, (float(k) + oz) * dx);
}

// Peskin's 4-point kernel, in cells (actuator.cpp's peskin4).
float peskin4(float r) {
    const float a = r < 0.0 ? -r : r;
    if (a < 1.0) return 0.125 * (3.0 - 2.0 * a + sqrt32_ftz(1.0 + 4.0 * a - 4.0 * a * a));
    if (a < 2.0) return 0.125 * (5.0 - 2.0 * a - sqrt32_ftz(-7.0 + 12.0 * a - 4.0 * a * a));
    return 0.0;
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_actuator_gather(uint3 tid : SV_DispatchThreadID) {
    const uint index = tid.x;
    if (index >= gather_push.face_count) { return; }
    const uint c = gather_push.component;
    const float3 x = face_point(c, index);
    const float inv_dx = grid_inv_dx();
    const uint at = component_base(c) + index;
    float out = airflow_increments[at];
    for (uint d = 0u; d < gather_push.disc_count; ++d) {
        const uint base = d * kDiscStride;
        const float3 centre = disc_vec3(base);
        const float reach = airflow_discs[base + 3u];
        const float axis_c = airflow_discs[base + 4u + c];
        const float scale = airflow_discs[base + 7u];
        const float swirl_scale = airflow_discs[base + 8u];
        const bool swirl = airflow_discs[base + 9u] != 0.0;
        if (axis_c == 0.0 && !swirl) continue;
        // Outside this box every point's weight is exactly 0, so skipping it changes nothing.
        if (abs(x.x - centre.x) > reach || abs(x.y - centre.y) > reach || abs(x.z - centre.z) > reach) continue;
        float sum = 0.0;
        float swirl_sum = 0.0;
        for (uint i = 0u; i < kDiscPoints; ++i) {
            const float3 f = (x - disc_vec3(base + kPointsAt + 3u * i)) * inv_dx;
            const float w = peskin4(f.x) * peskin4(f.y) * peskin4(f.z);
            sum += w;
            if (swirl) swirl_sum += airflow_discs[base + kTangentsAt + 3u * i + c] * w;
        }
        out += swirl ? scale * axis_c * sum + swirl_scale * swirl_sum : scale * axis_c * sum;
    }
    airflow_increments[at] = out;
}
