// ---------------------------------------------------------------------------
// rng.slang (S6 Task 8) -- THE SLANG PORT OF engine/core/rng.hpp's
// spade::rng::Stream: splitmix64, next_float and next_gauss, operating on
// shaders/shared/layouts.slang's RngStream (the mirror of the same 16 bytes).
//
// ===========================================================================
// WHAT IS EXACT HERE AND WHAT IS NOT -- THE ONE THING TO READ FIRST
//
// core/rng.hpp states the contract this file inherits: the splitmix64 constants
// and step order, the derivation, "next_float's exact construction (top 24
// bits, scaled by 2^-24)" and "next_gauss's operation order (u1 before u2; sin
// term returned first, cos term cached) and its clamp constant" are FIXED, and
// "a refactor that changes any of them silently invalidates every recorded
// snapshot and every replay in the determinism corpus".
//
//   * THE INTEGER HALF IS BIT-EXACT, BY CONSTRUCTION. Every 64-bit operation
//     goes through engine/shaders/u64.slang, which composes them from 32-bit
//     integer operations SPIR-V specifies to wrap mod 2^32. There is no
//     rounding to lose. Two streams that have drawn the same number of values
//     therefore hold BYTE-IDENTICAL state on both backends -- which is what
//     lets the parity table pin `dryden`'s and `imu_sensors`' stream lanes at
//     exactly zero rather than at a band, and what keeps the two runs' DRAW
//     COUNTS in step so that comparing them means anything at all.
//
//   * THE GAUSSIAN'S FLOAT HALF IS BANDED, AND ONLY BY sqrt. log32, sin32 and
//     cos32 are the T4 Slang ports, bit-identical to the host on every
//     device measured (tests/test_gpu_fp32_math.cpp; their subnormal paths are
//     integer, so no denormal mode changes them) -- though CPU<->GPU parity is
//     banded (CORE-3 as amended, TD-14), not a bit-identity requirement. The ONE
//     remaining operation the CPU performs that this file cannot promise bit
//     for bit is `std::sqrt`, which IEEE 754 mandates correctly rounded and
//     Vulkan specifies only to <= 2.5 ulp. That is the global constraint's
//     sanctioned downstream use of sqrt, and it is the sole reason a gust or a
//     sensor sample can differ in a last bit at all.
//
//     rng.hpp says of that sqrt: "deliberately KEPT: IEEE mandates it, so it is
//     already bit-identical everywhere and replacing it would trade a guarantee
//     for a polynomial". On the GPU the guarantee is weaker, and respelling it
//     as a polynomial here would trade a <= 2.5-ulp difference for a
//     GUARANTEED one -- so it stays, and the difference is measured rather than
//     argued.
//
//     THE BAND IS EXERCISED. DECLARED GRADE, MEASURED 2026-09-24 (R3).
//         scenario   one GNSS receiver, 200 substeps at rate_divider 1,
//                    bias_tau_s = 1e-6 so the Gauss-Markov retention underflows
//                    to exactly zero, mount_pos = 0 so the body's orientation
//                    cannot reach the report through the lever arm
//         compared   gnss_ring/position over the whole ring, ZERO band
//         n          64 ring elements; 11 OUTSIDE
//         measured   max |abs| 4.76837158e-07, max rel 5.06893741e-07
//         worst      element 22 component 2, cpu 3.761487961e+00 vs
//                    gpu 3.761488438e+00
//         device     Intel Iris Plus Graphics (integrated), Vulkan 1.3.215,
//                    driver 31.0.101.2125, msvc-ninja-release
//         and        NVIDIA GeForce RTX 3060 Ti, Vulkan 1.4.303, driver 572.83,
//                    msvc-ninja-release: max |abs| 4.76837158e-07, max rel
//                    1.06963546e-06, 19 of 64 elements differ (CORE-3 as
//                    amended, 2026-10-05)
//     11 of 64 rather than 64 of 64, at roughly 1 ulp: THAT IS WHAT A <= 2.5-ULP
//     sqrt LICENCE LOOKS LIKE, not a systematically different algorithm. It is
//     what makes banding this honest rather than resigned.
//
//     THIS PARAGRAPH USED TO SAY THE BAND WAS "CURRENTLY UNEXERCISED", on the
//     evidence that THE CORPUS's gaussian draws came back bit-identical. That
//     was true of the corpus and it was never a statement about the operation.
//     No corpus scenario ISOLATES a draw: the gaussian reaches a comparable
//     quantity only through sums whose other terms carry their own divergence,
//     so a 1-ulp difference in a draw had nowhere to show itself.
//
//     A BAND IS "UNEXERCISED" ONLY RELATIVE TO THE SCENARIOS THAT EXIST, AND
//     THAT IS A FACT ABOUT THE CORPUS RATHER THAN ABOUT THE ARITHMETIC. The
//     run above was built to isolate the draws and found the divergence on its
//     first execution.
//
//     It remains a real allowance and NOT a claim that Vulkan's sqrt is
//     correctly rounded. A tighter implementation (an integer correctly
//     rounded sqrt, as log32_div is for division) is allowed but not
//     required: CORE-3 as amended by the user's ruling of 2026-10-05, under
//     which CPU<->GPU results are banded everywhere (TD-14).
//
//     LIVE REPRODUCTION, kept alongside the declaration deliberately:
//     tests/test_gpu_parity.cpp's GnssDrawsMatchTheCpuWithinTheCore3Band
//     (once GnssDrawsDivergeAcrossBackends_KNOWN_OPEN, which asserted the
//     divergence was present) holds the draws to bands::core3::kGaussianDraw
//     over the whole ring, on every device of record (TD-14). A DECLARED BAND
//     AND A LIVE CHECK OF IT ARE DIFFERENT INSTRUMENTS and this path carries
//     both.
//
//     SCOPE, BY CALL AND NOT BY INCLUDE: rng_next_gauss reaches dryden.slang,
//     sensor_gnss.slang and sensor_imu.slang. forces_drag.slang and
//     medium_update.slang include this header and call only the integer/float
//     draws -- no sqrt on that path, so the grade does not apply to them.
//
// ===========================================================================
// STATE IS PASSED `inout`, NOT READ AND WRITTEN THROUGH A BUFFER
//
// Every caller (medium_update.slang, sensor_imu.slang) loads its row's
// RngStream into a LOCAL at the top of the thread, draws from it, and stores
// the local back exactly once at the end. That is not an optimization: a
// stream is single-owner by design (rng.hpp's THREADING note -- "plain mutable
// state with no synchronization, and there is deliberately none to add"), so a
// thread that owns a world or a sensor owns that row's stream outright, and a
// read-modify-write through the buffer on every draw would be the same
// arithmetic with more traffic and one more way to interleave.
//
// NO WORKGROUP-SHARED ANYTHING, and no cross-lane communication: every draw is
// a function of one thread's own local. T9's workgroup_size {32, 64, 128}
// invariance holds here by construction.
// ---------------------------------------------------------------------------

import layouts;
import fp32_math;
import u64;

// ---------------------------------------------------------------------------
// The splitmix64 constants (core/rng.hpp:71-73), each split into its two
// 32-bit lanes -- low word first, matching u64.slang's convention and the
// little-endian byte image the host's own uint64_t has.
//
// SPELLED AS THE SPLIT HALVES OF THE CPU'S OWN HEX LITERAL, with that literal
// quoted beside each one, so the transcription is checkable by eye rather than
// by trusting this comment:
//
//     kSplitmixGamma  0x9E3779B97F4A7C15   (2^64 / phi, odd)
//     kSplitmixMulA   0xBF58476D1CE4E5B9
//     kSplitmixMulB   0x94D049BB133111EB
// ---------------------------------------------------------------------------
static const uint2 kSplitmixGamma = uint2(0x7F4A7C15u, 0x9E3779B9u);
static const uint2 kSplitmixMulA = uint2(0x1CE4E5B9u, 0xBF58476Du);
static const uint2 kSplitmixMulB = uint2(0x133111EBu, 0x94D049BBu);

// core/rng.hpp:142 -- 2^-24, next_float's scale AND next_gauss's clamp floor.
// One constant with two jobs, deliberately the same number; see that header.
static const float kUniformQuantum = 0x1.0p-24;

// core/rng.hpp:146 -- 2*pi in fp32, spelled with the SAME decimal text the CPU
// spells it with (so both sides round the same literal rather than two
// different approximations of it). It is the float 0x40C90FDB; a wrong
// transcription here would not be subtle -- it would rotate every Box-Muller
// angle and move every gust and every noise sample in the corpus.
static const float kTwoPi = 6.28318530717958647692;

// ---------------------------------------------------------------------------
// One splitmix64 step -- core/rng.hpp:83-89, statement for statement:
//
//     state += kSplitmixGamma;
//     uint64_t z = state;
//     z = (z ^ (z >> 30)) * kSplitmixMulA;
//     z = (z ^ (z >> 27)) * kSplitmixMulB;
//     return z ^ (z >> 31);
//
// `^` on a uint2 is componentwise and therefore IS the 64-bit xor, with no
// helper needed and no carry to propagate -- which is why u64.slang does not
// provide one.
// ---------------------------------------------------------------------------
uint2 splitmix64_step(inout uint2 state) {
    state = u64_add(state, kSplitmixGamma);
    uint2 z = state;
    z = u64_mul(z ^ u64_shr(z, 30u), kSplitmixMulA);
    z = u64_mul(z ^ u64_shr(z, 27u), kSplitmixMulB);
    return z ^ u64_shr(z, 31u);
}

// core/rng.hpp:173 -- the primitive; everything else is built on it.
uint2 rng_next_u64(inout RngStream stream) { return splitmix64_step(stream.state); }

// ---------------------------------------------------------------------------
// Uniform float in [0, 1) -- core/rng.hpp:183-185.
//
//     return static_cast<float>(next_u64() >> 40) * kUniformQuantum;
//
// `>> 40` keeps bits 40..63, i.e. the TOP 24 bits, which is a value below 2^24
// and therefore exactly representable in fp32 -- so the integer->float
// conversion loses nothing on either side and the two agree bit for bit. The
// shift is spelled through u64_shr rather than as the equivalent `state.y >> 8`
// so that this line reads as the CPU line it mirrors; u64_shr's own n >= 32
// branch reduces it to exactly that.
// ---------------------------------------------------------------------------
float rng_next_float(inout RngStream stream) {
    const uint2 shifted = u64_shr(rng_next_u64(stream), 40u);
    return float(shifted.x) * kUniformQuantum;
}

// ---------------------------------------------------------------------------
// Standard normal, mean 0, stddev 1 -- basic (non-polar) Box-Muller.
// core/rng.hpp:231-258, in the SEVEN-STEP ORDER that header pins:
//
//   1. if a partner value is cached, consume and return it (NO draw);
//   2. otherwise draw u1 = next_float(), THEN u2 = next_float();
//   3. clamp u1 up to kUniformQuantum if it is 0;
//   4. r     = sqrt(-2 * ln(u1));
//   5. theta = kTwoPi * u2;
//   6. cache r * cos(theta) as the partner;
//   7. RETURN r * sin(theta) -- the SIN term first.
//
// CLEARING cached_gauss AS WELL AS has_cached ON CONSUMPTION is mirrored, and
// it is not tidiness on either side: rng.hpp's own note explains that leaving
// the consumed partner behind would let two streams with identical futures
// differ byte-wise, turning a snapshot memcmp -- and, here, a CPU<->GPU
// comparison of the stream lanes -- into a false difference. Dropping the
// clear on this side would make the GPU's 16 bytes differ from the CPU's after
// an odd number of draws while every future value still agreed.
//
// THE THREE TRANSCENDENTALS ARE THE ENGINE'S OWN (fp32_math.slang), never the
// driver's -- the GPU corollary of the bit-portability rule. `sqrt` is the
// device's, deliberately; see this file's header for why it stays.
// ---------------------------------------------------------------------------
float rng_next_gauss(inout RngStream stream) {
    if (stream.has_cached != 0u) {
        const float value = stream.cached_gauss;
        stream.cached_gauss = 0.0f;
        stream.has_cached = 0u;
        return value;
    }

    float u1 = rng_next_float(stream);
    const float u2 = rng_next_float(stream);
    if (u1 < kUniformQuantum) u1 = kUniformQuantum;

    const float r = sqrt(-2.0f * log32(u1));
    const float theta = kTwoPi * u2;

    stream.cached_gauss = r * cos32(theta);
    stream.has_cached = 1u;
    return r * sin32(theta);
}
