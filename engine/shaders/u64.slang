// ---------------------------------------------------------------------------
// u64.slang (S6 Task 8) -- EXACT 64-BIT UNSIGNED INTEGER ARITHMETIC ON A PAIR
// OF 32-BIT LANES. No floating point anywhere in this file, and no 64-bit type
// anywhere in this file: that is the whole point.
//
// ===========================================================================
// WHY THIS MODULE EXISTS -- A MEASUREMENT, NOT A PREFERENCE
//
// S6 Task 8's plan-mandated probe read VkPhysicalDeviceFeatures::shaderInt64 on
// this program's CORRECTNESS DEVICE -- the Intel Iris Plus every parity band in
// this engine is calibrated on -- BEFORE a line of rng.slang was written. The
// answer is VK_FALSE:
//
//     [GpuContext] shaderInt64 advertised=0 device=Intel(R) Iris(R) Plus Graphics
//
// (tests/test_compute_context.cpp's GpuContext.Int64ProbeMatchesTheDevice; the
// task report carries the ctest transcript.) A SPIR-V module may only declare a
// capability whose feature the logical device enabled, so on this device a
// module carrying `OpCapability Int64` is UNDEFINED BEHAVIOUR rather than a
// reported error -- the same shape of hazard the denormal mode was.
//
// core/rng.hpp's splitmix64 state IS a uint64, and its header is explicit that
// the constants, the step order and the whole construction are a WIRE CONTRACT
// pinned by golden vectors ("a refactor that changes any of them silently
// invalidates every recorded snapshot"). So the GPU cannot approximate it, and
// cannot use the native type either. It emulates it, exactly.
//
// ===========================================================================
// EXACT, AND WHY THAT WORD IS EARNED RATHER THAN CLAIMED
//
// Every operation below is composed from 32-bit unsigned integer operations
// that SPIR-V specifies to WRAP MODULO 2^32 (OpIAdd, OpIMul, OpShiftRightLogical
// -- none of them has an implementation-defined rounding the way OpFDiv and
// GLSL.std.450 Sqrt do). There is no tolerance to band and no ulp to lose: for
// every input, these produce the SAME 64 bits the CPU's native `uint64_t`
// operators produce. That is what makes the GPU's rng STREAM POSITION and STATE
// a bit-identity claim, and it is why the parity tables can pin `dryden`'s and
// `imu_sensors`' integer lanes at exactly zero rather than at a band.
//
// LANE ORDER: `.x` IS THE LOW WORD, `.y` THE HIGH WORD, matching the
// little-endian byte image the host's `uint64_t` has in the same storage. See
// shaders/shared/layouts.slang's header note on why that assumption is already
// unavoidable engine-wide (snapshots are memcpy, digests are byte folds).
//
// ===========================================================================
// WHAT IS *NOT* HERE, stated so nobody adds it speculatively: no division, no
// modulo, no signed forms, no comparison. rng.slang and sensor_imu.slang are
// this module's only callers and between them they need addition, multiplication
// modulo 2^64, logical right shift and xor -- and xor is not even a function
// here, because `a ^ b` on a uint2 is already componentwise and already exact.
// A helper nobody calls is a helper nobody tests.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 32x32 -> 64 unsigned multiply, the one primitive everything else needs and
// the one thing 32-bit hardware cannot do in a single operation.
//
// SPLIT INTO 16-BIT HALVES, so every partial product fits a uint32 exactly:
// a = a1*2^16 + a0 and b = b1*2^16 + b0 with all four halves < 2^16, so each
// of the four products a_i*b_j is < 2^32 and no intermediate is lost.
//
//     a*b = p11*2^32 + (p01 + p10)*2^16 + p00
//
// `mid` gathers everything that lands in bit positions 16..31: the top half of
// p00 plus the bottom halves of p01 and p10. It cannot overflow a uint32 --
// its three terms are each < 2^16, so mid < 3*2^16 -- which is exactly why the
// carry out of it (`mid >> 16`) can simply be added into the high word.
//
// Returns (low 32 bits, high 32 bits).
// ---------------------------------------------------------------------------
uint2 u64_mul32x32(uint a, uint b) {
    const uint a0 = a & 0xFFFFu;
    const uint a1 = a >> 16;
    const uint b0 = b & 0xFFFFu;
    const uint b1 = b >> 16;

    const uint p00 = a0 * b0;
    const uint p01 = a0 * b1;
    const uint p10 = a1 * b0;
    const uint p11 = a1 * b1;

    const uint mid = (p00 >> 16) + (p01 & 0xFFFFu) + (p10 & 0xFFFFu);
    const uint lo = (p00 & 0xFFFFu) | (mid << 16);
    const uint hi = p11 + (p01 >> 16) + (p10 >> 16) + (mid >> 16);
    return uint2(lo, hi);
}

// a + b, modulo 2^64.
//
// THE CARRY IS DETECTED BY THE WRAP ITSELF, which is the standard idiom and is
// exact rather than approximate: uint32 addition wraps, so `lo < a.x` holds if
// and only if the sum of the low words exceeded 2^32 - 1. (`lo < b.x` would do
// equally well and for the same reason.) The high-word addition is then allowed
// to wrap in its turn, which is precisely the mod-2^64 the CPU's `+=` performs.
uint2 u64_add(uint2 a, uint2 b) {
    const uint lo = a.x + b.x;
    const uint carry = (lo < a.x) ? 1u : 0u;
    const uint hi = a.y + b.y + carry;
    return uint2(lo, hi);
}

// a * b, modulo 2^64 -- i.e. exactly what a C++ `uint64_t *` produces, which is
// defined to wrap.
//
// Only THREE of the four cross products matter: a.y*b.y lands entirely at bit
// 64 and above and is discarded by the modulus, so it is not computed. The
// other two cross terms contribute only their low 32 bits, for the same reason,
// which is why plain `a.x * b.y` (wrapping uint32 multiply) is not a loss here.
uint2 u64_mul(uint2 a, uint2 b) {
    const uint2 low = u64_mul32x32(a.x, b.x);
    const uint hi = low.y + a.x * b.y + a.y * b.x;
    return uint2(low.x, hi);
}

// a >> n, LOGICAL (zero-filling), for n in [0, 63].
//
// THE THREE CASES ARE ALL REAL AND THE FIRST IS NOT PEDANTRY. In SPIR-V, as in
// C++, a shift by 32 or more of a 32-bit value is not defined to produce 0 --
// it is undefined -- so `(a.x >> n) | (a.y << (32 - n))` is WRONG at n == 0
// (it would shift the high word left by 32) even though the answer there is
// trivially `a`. splitmix64 only ever asks for 27, 30 and 31, and next_float
// asks for 40, so no caller in this engine reaches n == 0; the case is handled
// anyway because a helper that is correct only for its current callers is a
// trap for the next one.
uint2 u64_shr(uint2 a, uint n) {
    if (n == 0u) {
        return a;
    }
    if (n < 32u) {
        return uint2((a.x >> n) | (a.y << (32u - n)), a.y >> n);
    }
    // n in [32, 63]: the low word is gone entirely and the high word supplies
    // everything. `n - 32` is in [0, 31], which IS a defined shift distance.
    return uint2(a.y >> (n - 32u), 0u);
}
