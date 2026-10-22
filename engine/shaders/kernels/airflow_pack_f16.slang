// ---------------------------------------------------------------------------
// airflow_pack_f16.slang (plan B, task B6) -- publish's halves on the GPU:
// airflow_publish_fields' eight fp32 channels per cell packed to binary16 by
// core/fp16.hpp's from_f32, term for term (its CPU twin, and the rule: TD-9,
// one rule in one place).
//
// PURE INTEGER BIT MANIPULATION, as the CPU's: the channels are read as their
// bits (asuint), and nothing here is float arithmetic, so the halves match
// the CPU's to the bit wherever the fp32 channels do. Round to nearest, ties
// to even; subnormals and the sign of zero kept; a NaN, an infinity or
// |x| >= 65520 is fp16::kNoDataBits (a quiet NaN), never saturated (L6).
//
// One invocation per cell (1D, guarded by the push's cell_count). Writes two
// words of airflow_publish_v (u, v, w, dye) and two of airflow_publish_s
// (p, |omega|, q-hat, lambda2-hat) per cell, the lower channel in the low
// half: on a little-endian host the words read back as FlowVolume::v and
// ::s's uint16 order.
// ---------------------------------------------------------------------------

import layouts;
import bindings;

// The airflow push convention (airflow_params_layout.slang): the count in
// word 2.
struct PackPush {
    uint unused0;
    uint unused1;
    uint cell_count;
    uint unused2;
};

[[vk::push_constant]] ConstantBuffer<PackPush> pack_push;

static const uint kChannels = 8u;
static const uint kNoDataBits = 0x7E00u;  // fp16::kNoDataBits

// The fp32 channels airflow_publish_fields wrote into airflow_advect.
uint channel_bits(uint cell, uint c) { return asuint(airflow_advect[kChannels * cell + c]); }

// `m >> s`, rounded to nearest with ties to even, for 1 <= s <= 31.
uint shift_right_rne(uint m, uint s) {
    const uint kept = m >> s;
    const uint rest = m & ((1u << s) - 1u);
    const uint half = 1u << (s - 1u);
    return kept + ((rest > half || (rest == half && (kept & 1u) != 0u)) ? 1u : 0u);
}

// fp16::from_f32's bits.
uint half_bits(uint f) {
    const uint sign = (f >> 16) & 0x8000u;
    const uint exp = (f >> 23) & 0xFFu;
    const uint mant = f & 0x7FFFFFu;
    if (exp == 0xFFu) return kNoDataBits;  // NaN or infinity
    // Below 2^-25, under half the smallest subnormal (f32 subnormals
    // included): rounds to zero, keeping the sign.
    if (exp < 102u) return sign;
    // The 24-bit significand with its implicit bit, worth sig * 2^(exp - 150).
    const uint sig = mant | 0x800000u;
    uint magnitude = 0u;
    if (exp < 113u) {
        // A half subnormal: units of 2^-24, so shift by 126 - exp (14 to 24).
        // A carry out of the mantissa lands on the smallest normal, 0x0400.
        magnitude = shift_right_rne(sig, 126u - exp);
    } else {
        // A half normal: rebias the exponent and round the mantissa to 10
        // bits. A carry rolls into the exponent, and past 30 into infinity.
        magnitude = ((exp - 112u) << 10) + shift_right_rne(sig, 13u) - 0x400u;
    }
    if (magnitude >= 0x7C00u) return kNoDataBits;  // |x| >= 65520: would round to infinity
    return sign | magnitude;
}

uint packed_pair(uint cell, uint c) {
    return half_bits(channel_bits(cell, c)) | (half_bits(channel_bits(cell, c + 1u)) << 16);
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_pack_f16(uint3 tid : SV_DispatchThreadID) {
    const uint cell = tid.x;
    if (cell >= pack_push.cell_count) { return; }
    airflow_publish_v[2u * cell] = packed_pair(cell, 0u);
    airflow_publish_v[2u * cell + 1u] = packed_pair(cell, 2u);
    airflow_publish_s[2u * cell] = packed_pair(cell, 4u);
    airflow_publish_s[2u * cell + 1u] = packed_pair(cell, 6u);
}
