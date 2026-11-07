// ---------------------------------------------------------------------------
// lbm_lattice_dump.slang (L1 GPU twin, T1) -- the device's D3Q27 tables written
// out, so a test compares them to engine/physics/lbm/lattice.hpp's bit for bit
// (lbm_lattice.slang's header). Entry i of the lattice writes five floats at
// lbm_f[5 i]: cx, cy, cz as floats (small integers, exact), the weight, and the
// opposite direction's index as a float. One invocation per direction. After the
// 27 x 5 = 135 table words it ECHOES the push block, word for word, at lbm_f[135 + k]
// (as raw bits), so the host proves lbm_params.slang's LbmPush is laid out as its
// C++ twin is. 27 directions plus 25 push words = 52 invocations; the first 27
// write the tables, the rest the echo.
//
// SET 1, THE LBM BINDINGS (bindings.slang section F, bindings 40 and 41): this
// kernel writes lbm_f only; the push block is lbm_params.slang's.
// ---------------------------------------------------------------------------
import bindings;
import lbm_lattice;
import lbm_params;

[[vk::push_constant]] ConstantBuffer<LbmPush> push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void lbm_lattice_dump(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= kLbmQ + kLbmPushWords) { return; }
    if (i >= kLbmQ) {
        const uint k = i - kLbmQ;
        uint bits = 0u;
        if (k < 4u) {
            bits = (k == 0u) ? push.nx : ((k == 1u) ? push.ny : ((k == 2u) ? push.nz : push.parity));
        } else if (k == 4u) {
            bits = asuint(push.nu);
        } else if (k == 5u) {
            bits = asuint(push.cw);
        } else if (k == 6u) {
            bits = push.periodic_mask;
        } else {
            bits = asuint(push.wall_u[k - 7u]);
        }
        lbm_f[5u * kLbmQ + k] = asfloat(bits);
        return;
    }
    lbm_f[5u * i + 0u] = float(kLbmCx[i]);
    lbm_f[5u * i + 1u] = float(kLbmCy[i]);
    lbm_f[5u * i + 2u] = float(kLbmCz[i]);
    lbm_f[5u * i + 3u] = lbm_weight(i);
    lbm_f[5u * i + 4u] = float(lbm_opposite(i));
}
