// ---------------------------------------------------------------------------
// airflow_line_spread_binned.slang (L86i) -- airflow_line_spread's gather with the elements binned by ROTOR: the same expression for the
// same face and element, in the same element order, so the same bits (airflow_line_spread.slang is the authority for every term). A face
// of the dispatched component first walks the rotors in order and skips a rotor whose face box (the min and max of its elements'
// stencils on this component, line_ranges()' floats the host packs every step) does not hold it, then walks that rotor's elements in
// order. An element whose stencil does not hold the face contributes nothing, and a rotor's box holds every one of its elements'
// stencils, so skipping the rotor skips only elements that would have been skipped: the sum is the unbinned one's. The cost is the
// rotors' boxes' faces x their elements, not every face x every element.
//
// The push: word 0 the component, word 1 the rotor count, word 2 the component's faces, word 3 the word offset of the rotor boxes in
// airflow_line_elems (kLineRangeWords floats a rotor: [0] the first element, [1] the count, then lo (x, y, z) and hi of each component).
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct LineBinnedPush {
    uint component;    // word 0: 0 u, 1 v, 2 w
    uint rotor_count;  // word 1
    uint count;        // word 2: the component's faces
    uint ranges_at;    // word 3: where the rotor boxes start in airflow_line_elems
};

[[vk::push_constant]] ConstantBuffer<LineBinnedPush> line_binned_push;

static const uint kLineTaps = 13u;
static const uint kLineHeader = 16u;
static const uint kLineComponent = 7u + 3u * kLineTaps;
static const uint kLineStride = kLineHeader + 3u * kLineComponent;
static const uint kForceStride = 8u;
static const uint kRangeWords = 20u;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_line_spread_binned(uint3 tid : SV_DispatchThreadID) {
    const uint index = tid.x;
    if (index >= line_binned_push.count) { return; }
    const uint c = line_binned_push.component;
    const uint3 dims = grid_dims();
    const uint cx = c == 0u ? dims.x + 1u : dims.x;
    const uint cy = c == 1u ? dims.y + 1u : dims.y;
    const int i = int(index % cx);
    const int j = int((index / cx) % cy);
    const int k = int(index / (cx * cy));
    const uint faces_u = (dims.x + 1u) * dims.y * dims.z;
    const uint faces_v = dims.x * (dims.y + 1u) * dims.z;
    const uint base = c == 0u ? 0u : (c == 1u ? faces_u : faces_u + faces_v);
    const uint at = base + index;
    float acc = airflow_increments[at];
    for (uint r = 0u; r < line_binned_push.rotor_count; ++r) {
        const uint box = line_binned_push.ranges_at + r * kRangeWords;
        const uint first = uint(airflow_line_elems[box]);
        const uint count = uint(airflow_line_elems[box + 1u]);
        const uint b = box + 2u + 6u * c;
        if (i < int(airflow_line_elems[b]) || i >= int(airflow_line_elems[b + 3u])) { continue; }
        if (j < int(airflow_line_elems[b + 1u]) || j >= int(airflow_line_elems[b + 4u])) { continue; }
        if (k < int(airflow_line_elems[b + 2u]) || k >= int(airflow_line_elems[b + 5u])) { continue; }
        for (uint e = first; e < first + count; ++e) {
            const uint rec = e * kLineStride;
            const uint comp = rec + kLineHeader + c * kLineComponent;
            const int di = i - int(airflow_line_elems[comp]);
            const int dj = j - int(airflow_line_elems[comp + 1u]);
            const int dk = k - int(airflow_line_elems[comp + 2u]);
            if (di < 0 || di >= int(airflow_line_elems[comp + 3u])) { continue; }
            if (dj < 0 || dj >= int(airflow_line_elems[comp + 4u])) { continue; }
            if (dk < 0 || dk >= int(airflow_line_elems[comp + 5u])) { continue; }
            const float s = airflow_line_elems[comp + 6u];
            const float amount = airflow_line_force[e * kForceStride + c] * airflow_line_elems[rec + 12u];
            if (!(s > 0.0) || amount == 0.0) { continue; }
            const float coef = div32_ftz(amount, s);
            const float wj = airflow_line_elems[comp + 7u + kLineTaps + uint(dj)];
            const float wk = airflow_line_elems[comp + 7u + 2u * kLineTaps + uint(dk)];
            const float wjk = wj * wk;
            acc += coef * (airflow_line_elems[comp + 7u + uint(di)] * wjk);
        }
    }
    airflow_increments[at] = acc;
}
