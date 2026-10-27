// ---------------------------------------------------------------------------
// test_fp32_exact.cpp -- THE CPU PROOF of engine/shaders/fp32_math.slang's
// exact division and square root, div32_ftz and sqrt32_ftz (Core's plan,
// docs/design/core/plans/2026-10-22-exact-div-sqrt-plan.md, section 2).
//
// WHAT IS UNDER TEST IS THE INTEGER CORRECTION, through
// tests/fp32_exact_battery.hpp's line-cited port of div32_ftz_with and
// sqrt32_ftz_with, held bit for bit to the host's divss and sqrtss under MXCSR
// FTZ|DAZ. The estimate is the one input a device decides (Vulkan: OpFDiv
// within 2.5 ulp, InverseSqrt within 2), so the battery supplies it: the
// correctly rounded 1/bm or 1/sqrt(xm), pushed by every k within the bound and
// far past it. A correction exact for every estimate in that window is exact
// on any conforming device, and
// GpuFp32Math.ExactCorrectionRoundsInjectedEstimatesOnTheDevice shows the
// device runs this same correction, bit for bit.
//
// THE WINDOW. "+-3 ulps off" is every float within 3 ulps of the correctly
// rounded estimate, the ulp being its own binade's spacing (the battery's
// lowest_k): k in [-3, 3] floats, and further down where the window crosses a
// power of two -- [-6, 3] at bm = 1 and xm = 1, where Vulkan's ulp is the gap
// above, and [-5, 3] one float above a power of two (bm = 2 - 2^-23). That
// holds every float the Vulkan bound allows, so the sqrt row, over its whole
// input space, is a proof by exhaustion.
//
// THE PROOF'S BOUNDS, and so what the far-past rows assert:
//   division  exact while |d0| < 128 units of the quotient (the remainder's
//             32-bit wrap). k = +-32 ulps of y gives |d0| about 66, inside it:
//             ASSERTED. Past it (about +-63) the first failing k is printed.
//   sqrt      exact while |e0| < 64 units (63 proven; fp32_math.slang's
//             proof). At worst, xm -> 4, that is about +-31 ulps of r (|e0|
//             about 2k), so +-16 is ASSERTED, and +-32 sits on the bound's
//             edge: REPORTED over every xm in [3.9, 4), where misses are
//             expected, and never asserted.
//
// SIZES. The default rows are the plan's (its budget: 5 s release and 40 s
// debug under ctest, rows past that move behind the gate). SPADE_EXACT_SWEEP=1
// runs the large ones:
//
//     $env:SPADE_EXACT_SWEEP = 1
//     powershell -File scripts\test.ps1 -Preset release -Filter Fp32Exact
//
// COUNTS. Every row prints `[Fp32Exact] <row>: n=...  misses=...`. The inputs
// are enumerated or drawn from splitmix64 with fixed seeds, single-threaded,
// so the lines must match between MSVC and the leg's gcc-13.
//
// x86-64 only, as engine/core/detail/flush_subnormals.hpp is (its #error).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <utility>
#include <vector>

#include "core/detail/flush_subnormals.hpp"
#include "fp32_exact_battery.hpp"

namespace {

namespace ex = spade::testing::exact;
using ex::exact_sweep;
using ex::Stop;

// A row's tally: results compared, misses, and the first miss, to name.
struct Row {
    uint64_t n = 0;
    uint64_t misses = 0;
    uint32_t in0 = 0;       // the first miss: a (division) or x (sqrt), as bits
    uint32_t in1 = 0;       // b, or 0 for sqrt
    uint32_t estimate = 0;  // y or r, as bits
    int k = 0;              // its push from the correctly rounded estimate
    uint32_t got = 0;       // the port's answer
    uint32_t want = 0;      // the host's

    void add(uint32_t a, uint32_t b, int push, float e, uint32_t port, uint32_t host) {
        ++n;
        if (port == host) return;
        if (misses == 0) {
            in0 = a;
            in1 = b;
            estimate = ex::bits(e);
            k = push;
            got = port;
            want = host;
        }
        ++misses;
    }
};

void report(const char* row_name, const Row& row) {
    std::printf("[Fp32Exact] %s: n=%" PRIu64 "  misses=%" PRIu64 "\n", row_name, row.n, row.misses);
    if (row.misses != 0) {
        std::printf("[Fp32Exact]   first miss: in 0x%08" PRIX32 " 0x%08" PRIX32 "  estimate 0x%08" PRIX32
                    " (k=%d)  port 0x%08" PRIX32 "  host 0x%08" PRIX32 "\n",
                    row.in0, row.in1, row.estimate, row.k, row.got, row.want);
    }
    std::fflush(stdout);
}

// Which estimates a row tries per input: the window within `ulps`, or exactly
// the ks in `list`.
struct Ks {
    int ulps = 0;
    std::span<const int> list = {};
};

// Inputs per reference call.
constexpr std::size_t kBlock = std::size_t{1} << 16;

// Division over `count` pairs, `fill(base, n, a, b)` writing a block of them
// as bits; each pair with every estimate in `ks`, the port stopped at `stop`,
// against divss under the flush.
template <class Fill>
[[nodiscard]] Row divide_row(uint64_t count, Fill fill, Ks ks, Stop stop = Stop::full) {
    Row row;
    std::vector<uint32_t> a(kBlock), b(kBlock), want(kBlock);
    for (uint64_t base = 0; base < count; base += kBlock) {
        const std::size_t n = static_cast<std::size_t>(std::min<uint64_t>(kBlock, count - base));
        fill(base, n, a.data(), b.data());
        ex::reference_divide(std::span(a.data(), n), std::span(b.data(), n), std::span(want.data(), n));
        for (std::size_t j = 0; j < n; ++j) {
            const float e = ex::div_estimate(b[j]);
            if (ks.list.empty()) {
                for (int k = ex::lowest_k(e, ks.ulps); k <= ks.ulps; ++k) {
                    const float y = ex::nudge(e, k);
                    row.add(a[j], b[j], k, y, ex::port_div(a[j], b[j], y, stop), want[j]);
                }
            } else {
                for (const int k : ks.list) {
                    const float y = ex::nudge(e, k);
                    row.add(a[j], b[j], k, y, ex::port_div(a[j], b[j], y, stop), want[j]);
                }
            }
        }
    }
    return row;
}

// The square root likewise: `fill(base, n, x)`, against sqrtss under the flush.
template <class Fill>
[[nodiscard]] Row sqrt_row(uint64_t count, Fill fill, Ks ks, Stop stop = Stop::full) {
    Row row;
    std::vector<uint32_t> x(kBlock), want(kBlock);
    for (uint64_t base = 0; base < count; base += kBlock) {
        const std::size_t n = static_cast<std::size_t>(std::min<uint64_t>(kBlock, count - base));
        fill(base, n, x.data());
        ex::reference_sqrt(std::span(x.data(), n), std::span(want.data(), n));
        for (std::size_t j = 0; j < n; ++j) {
            const float e = ex::sqrt_estimate(x[j]);
            if (ks.list.empty()) {
                for (int k = ex::lowest_k(e, ks.ulps); k <= ks.ulps; ++k) {
                    const float r = ex::nudge(e, k);
                    row.add(x[j], 0u, k, r, ex::port_sqrt(x[j], r, stop), want[j]);
                }
            } else {
                for (const int k : ks.list) {
                    const float r = ex::nudge(e, k);
                    row.add(x[j], 0u, k, r, ex::port_sqrt(x[j], r, stop), want[j]);
                }
            }
        }
    }
    return row;
}

// Fills for the rows below.

// a over every significand in [1, 2), b fixed.
[[nodiscard]] auto every_a_over(uint32_t b) {
    return [b](uint64_t base, std::size_t n, uint32_t* a, uint32_t* bs) {
        for (std::size_t j = 0; j < n; ++j) {
            a[j] = 0x3F800000u | static_cast<uint32_t>(base + j);
            bs[j] = b;
        }
    };
}

// a fixed, b over every significand in [1, 2).
[[nodiscard]] auto every_b_under(uint32_t a) {
    return [a](uint64_t base, std::size_t n, uint32_t* as, uint32_t* b) {
        for (std::size_t j = 0; j < n; ++j) {
            as[j] = a;
            b[j] = 0x3F800000u | static_cast<uint32_t>(base + j);
        }
    };
}

// Random significand pairs in [1, 2), random signs: the correction sees only
// the significands, so the exponents stay fixed (the specials row varies them).
[[nodiscard]] auto random_significands(ex::SplitMix& rng) {
    return [&rng](uint64_t, std::size_t n, uint32_t* a, uint32_t* b) {
        for (std::size_t j = 0; j < n; ++j) {
            const uint64_t r = rng.next();
            a[j] = (static_cast<uint32_t>(r >> 63) << 31) | 0x3F800000u | (static_cast<uint32_t>(r) & 0x7FFFFFu);
            b[j] = (static_cast<uint32_t>((r >> 62) & 1u) << 31) | 0x3F800000u |
                   (static_cast<uint32_t>(r >> 23) & 0x7FFFFFu);
        }
    };
}

// Uniformly random bit patterns: every class, specials about 1 in 128.
[[nodiscard]] auto random_bit_pairs(ex::SplitMix& rng) {
    return [&rng](uint64_t, std::size_t n, uint32_t* a, uint32_t* b) {
        for (std::size_t j = 0; j < n; ++j) {
            const uint64_t r = rng.next();
            a[j] = static_cast<uint32_t>(r);
            b[j] = static_cast<uint32_t>(r >> 32);
        }
    };
}

// near_ties(), one block at a time, each block from its own seed.
[[nodiscard]] auto near_tie_pairs(uint64_t seed) {
    return [seed](uint64_t base, std::size_t n, uint32_t* a, uint32_t* b) {
        std::vector<uint32_t> as, bs;
        ex::near_ties(n, seed + base, as, bs);
        std::copy(as.begin(), as.begin() + static_cast<std::ptrdiff_t>(n), a);
        std::copy(bs.begin(), bs.begin() + static_cast<std::ptrdiff_t>(n), b);
    };
}

// x over a run of bit patterns from `first`.
[[nodiscard]] auto run_from(uint32_t first) {
    return [first](uint64_t base, std::size_t n, uint32_t* x) {
        for (std::size_t j = 0; j < n; ++j) x[j] = first + static_cast<uint32_t>(base + j);
    };
}

// x from a list.
[[nodiscard]] auto from_list(const std::vector<uint32_t>& list) {
    return [&list](uint64_t base, std::size_t n, uint32_t* x) {
        std::copy_n(list.begin() + static_cast<std::ptrdiff_t>(base), n, x);
    };
}

}  // namespace

// ===========================================================================
// 1. The reference
// ===========================================================================

// divss and sqrtss under the scope flush, and without it are IEEE's. The
// canary is 0x00FFFFFF / 2 = FLT_MIN (1 - 2^-24): +0 under FTZ, FLT_MIN under
// gradual underflow (the exact quotient is a tie on the subnormal grid and
// rounds to even, upward). Each check runs through the four-lane path and the
// one-lane tail (lanes 0 and 4).
TEST(Fp32Exact, TheHostReferenceFlushes) {
    const std::array<uint32_t, 5> num = {0x00FFFFFFu, 0x00000001u, 0x3F800000u, 0x40400000u, 0x00FFFFFFu};
    const std::array<uint32_t, 5> den = {0x40000000u, 0x00000002u, 0x40400000u, 0x3F800000u, 0x40000000u};
    const std::array<uint32_t, 5> neg = {0x80000001u, 0x40800000u, 0x3F800000u, 0x00000001u, 0x80000001u};
    std::array<uint32_t, 5> q_flushed{}, q_ieee{}, r_flushed{}, r_ieee{};
    ex::reference_divide(num, den, q_flushed);
    ex::reference_sqrt(neg, r_flushed);
    ex::host_divide(num.data(), den.data(), q_ieee.data(), q_ieee.size());  // the default MXCSR word
    ex::host_sqrt(neg.data(), r_ieee.data(), r_ieee.size());

    const struct {
        const char* what;
        uint32_t got;
        uint32_t want;
    } checks[] = {
        {"0x00FFFFFF / 2 under FTZ is +0 (the canary)", q_flushed[0], 0x00000000u},
        {"subnormal / subnormal under DAZ is 0 / 0, the default NaN", q_flushed[1], 0xFFC00000u},
        {"sqrt(-subnormal) under DAZ is -0", r_flushed[0], 0x80000000u},
        {"0x00FFFFFF / 2 without the scope is FLT_MIN", q_ieee[0], 0x00800000u},
        {"subnormal / subnormal without the scope is 0.5", q_ieee[1], 0x3F000000u},
        {"sqrt(-subnormal) without the scope is the default NaN", r_ieee[0], 0xFFC00000u},
        {"the canary in the one-lane tail", q_flushed[4], 0x00000000u},
        {"sqrt(+subnormal) under DAZ is +0", r_flushed[3], 0x00000000u},
        {"sqrt(-subnormal) in the one-lane tail", r_flushed[4], 0x80000000u},
    };
    uint64_t misses = 0;
    for (const auto& c : checks) {
        if (c.got != c.want) ++misses;
        EXPECT_EQ(c.got, c.want) << c.what;
    }
    std::printf("[Fp32Exact] HostReference: n=%zu  misses=%" PRIu64 "\n", std::size(checks), misses);
    std::fflush(stdout);
}

// ===========================================================================
// 2. +-3 ulps: the division, and the square root over its whole input space
// ===========================================================================

// Every fa for b = 1 and b = 2 - 2^-23, the two ends of the reciprocal (y = 1,
// a power of two, and y one float above 1/2); random significand pairs; and
// Rendering's near-ties, the closest any quotient comes to a midpoint.
TEST(Fp32Exact, DivisionRoundsExactlyWithEstimatesThreeUlpsOff) {
    const bool sweep = exact_sweep();
    constexpr Ks kThree{.ulps = 3};
    constexpr uint64_t kEveryFraction = uint64_t{1} << 23;

    const Row b_one = divide_row(kEveryFraction, every_a_over(0x3F800000u), kThree);
    report("DivisionEveryAOverOne", b_one);
    EXPECT_EQ(b_one.misses, 0u) << "every a / 1";
    const Row b_two = divide_row(kEveryFraction, every_a_over(0x3FFFFFFFu), kThree);
    report("DivisionEveryAOverTwoLessAnUlp", b_two);
    EXPECT_EQ(b_two.misses, 0u) << "every a / (2 - 2^-23)";

    if (sweep) {
        // 16 divisors and 8 dividends: both ends, the halves and thirds, sqrt(2),
        // and a few with no structure.
        constexpr uint32_t kB[] = {0x000000u, 0x000001u, 0x000002u, 0x0F0F0Fu, 0x123456u, 0x19999Au,
                                   0x200000u, 0x2AAAABu, 0x3504F3u, 0x400000u, 0x4CCCCDu, 0x555555u,
                                   0x600000u, 0x6DB6DBu, 0x7FFFFEu, 0x7FFFFFu};
        constexpr uint32_t kA[] = {0x000000u, 0x000001u, 0x2AAAABu, 0x3504F3u,
                                   0x400000u, 0x555555u, 0x7FFFFEu, 0x7FFFFFu};
        uint64_t misses = 0;
        for (const uint32_t fb : kB) {
            const Row row = divide_row(kEveryFraction, every_a_over(0x3F800000u | fb), kThree);
            report("DivisionEveryAOverSixteenB", row);
            misses += row.misses;
        }
        for (const uint32_t fa : kA) {
            const Row row = divide_row(kEveryFraction, every_b_under(0x3F800000u | fa), kThree);
            report("DivisionEightAOverEveryB", row);
            misses += row.misses;
        }
        EXPECT_EQ(misses, 0u) << "the sweep's fixed-operand rows";
    }

    ex::SplitMix rng{0xD1F0ull};
    const Row random =
        divide_row(sweep ? uint64_t{1} << 28 : uint64_t{1} << 22, random_significands(rng), kThree);
    report("DivisionRandomSignificands", random);
    EXPECT_EQ(random.misses, 0u) << "random significand pairs";

    const Row ties = divide_row(sweep ? uint64_t{1} << 25 : uint64_t{1} << 20, near_tie_pairs(0x71E5ull), kThree);
    report("DivisionNearTies", ties);
    EXPECT_EQ(ties.misses, 0u) << "near-ties, |A 2^(s+1) - B (2Q+1)| = 1 or 3";
}

// Every float in [1, 4): all 2^24 (fx, odd), so with the window this is every
// estimate within the bound for every input the correction can see.
TEST(Fp32Exact, SqrtRoundsExactlyWithEstimatesThreeUlpsOffOverItsWholeInputSpace) {
    const int ulps = exact_sweep() ? 8 : 3;
    const Row row = sqrt_row(uint64_t{1} << 24, run_from(0x3F800000u), Ks{.ulps = ulps});
    report(ulps == 3 ? "SqrtEveryInputInOneToFour" : "SqrtEveryInputInOneToFourEightUlps", row);
    EXPECT_EQ(row.misses, 0u) << "every x in [1, 4), every estimate within " << ulps << " ulps";
}

// ===========================================================================
// 3. Far past the bound
// ===========================================================================

// Division at +-32 ulps of y and sqrt at +-16 of r: ASSERTED (both inside the
// proof's wrap bound, this file's header). Sqrt at +-32 over xm in [3.9, 4),
// the bound's edge, and the first failing k of each: REPORTED only.
TEST(Fp32Exact, TheCorrectionHoldsFarPastTheVulkanBound) {
    constexpr uint64_t kCount = uint64_t{1} << 20;
    constexpr int kDivK[] = {-32, 32};
    constexpr int kSqrtK[] = {-16, 16};

    ex::SplitMix rng{0xFA57ull};
    const Row div = divide_row(kCount, random_significands(rng), Ks{.list = kDivK});
    report("DivisionThirtyTwoUlps", div);
    EXPECT_EQ(div.misses, 0u) << "division, y 32 ulps off";

    ex::SplitMix xs{0x5A17ull};
    const Row root = sqrt_row(
        kCount,
        [&xs](uint64_t, std::size_t n, uint32_t* x) {
            for (std::size_t j = 0; j < n; ++j) {
                x[j] = 0x3F800000u + (static_cast<uint32_t>(xs.next()) & 0x00FFFFFFu);  // [1, 4)
            }
        },
        Ks{.list = kSqrtK});
    report("SqrtSixteenUlps", root);
    EXPECT_EQ(root.misses, 0u) << "sqrt, r 16 ulps off";

    // The bound's edge: reported, never asserted.
    constexpr int kEdgeK[] = {-32, 32};
    const Row edge = sqrt_row(ex::kBoundEdgeCount, run_from(ex::kBoundEdgeFirst), Ks{.list = kEdgeK});
    report("SqrtThirtyTwoUlpsAtTheBoundEdge (report only)", edge);

    // The first failing k of each, from just past the asserted one: division
    // over 2^16 of its random pairs, sqrt over every fourth xm in [3.9, 4).
    ex::SplitMix again{0xFA57ull};
    std::vector<uint32_t> a(kBlock), b(kBlock);
    random_significands(again)(0, kBlock, a.data(), b.data());
    int div_first = 0;
    for (int k = 33; k <= 160 && div_first == 0; ++k) {
        const int ks[] = {-k, k};
        const Row row = divide_row(
            kBlock,
            [&a, &b](uint64_t, std::size_t n, uint32_t* as, uint32_t* bs) {
                std::copy_n(a.begin(), n, as);
                std::copy_n(b.begin(), n, bs);
            },
            Ks{.list = ks});
        if (row.misses != 0) div_first = k;
    }
    int sqrt_first = 0;
    for (int k = 17; k <= 64 && sqrt_first == 0; ++k) {
        const int ks[] = {-k, k};
        const Row row = sqrt_row(
            ex::kBoundEdgeCount / 4u,
            [](uint64_t base, std::size_t n, uint32_t* x) {
                for (std::size_t j = 0; j < n; ++j) x[j] = ex::kBoundEdgeFirst + 4u * static_cast<uint32_t>(base + j);
            },
            Ks{.list = ks});
        if (row.misses != 0) sqrt_first = k;
    }
    std::printf("[Fp32Exact] FirstFailingK (report only): division +-%d (0: none through +-160), "
                "sqrt +-%d (0: none through +-64)\n",
                div_first, sqrt_first);
    std::fflush(stdout);
}

// ===========================================================================
// 4. The specials and the exponent edges
// ===========================================================================

// Every pair of the 48 specials; 2^16 pairs per result exponent -1..2 (the
// FTZ edge) and 253..256 (overflow); random bit patterns; and the square root
// over the specials, the biased exponents 0, 1, 254 and 255 of both signs, and
// random bit patterns. Estimates k in {-3, 0, 3}.
TEST(Fp32Exact, SpecialsAndExponentEdgesMatchTheHostBitForBit) {
    const bool sweep = exact_sweep();
    constexpr int kK[] = {-3, 0, 3};
    const Ks kSet{.list = kK};
    constexpr std::size_t kN = ex::kSpecials.size();

    const Row specials = divide_row(
        kN * kN,
        [](uint64_t base, std::size_t n, uint32_t* a, uint32_t* b) {
            for (std::size_t j = 0; j < n; ++j) {
                a[j] = ex::kSpecials[static_cast<std::size_t>((base + j) / kN)];
                b[j] = ex::kSpecials[static_cast<std::size_t>((base + j) % kN)];
            }
        },
        kSet);
    report("DivisionSpecialPairs", specials);
    EXPECT_EQ(specials.misses, 0u) << "every pair of the specials";

    for (const int er : {-1, 0, 1, 2, 253, 254, 255, 256}) {
        ex::SplitMix rng{0xED6Eull + static_cast<uint64_t>(er + 1)};
        const Row row = divide_row(
            kBlock,
            [&rng, er](uint64_t, std::size_t n, uint32_t* a, uint32_t* b) {
                for (std::size_t j = 0; j < n; ++j) {
                    const std::pair<uint32_t, uint32_t> p = ex::pair_with_result_exponent(rng, er);
                    a[j] = p.first;
                    b[j] = p.second;
                }
            },
            kSet);
        char name[64];
        std::snprintf(name, sizeof name, "DivisionResultExponent%d", er);
        report(name, row);
        EXPECT_EQ(row.misses, 0u) << "result exponent " << er;
    }

    ex::SplitMix rng{0xB175ull};
    const Row random = divide_row(sweep ? uint64_t{1} << 28 : uint64_t{1} << 22, random_bit_pairs(rng), kSet);
    report("DivisionRandomBits", random);
    EXPECT_EQ(random.misses, 0u) << "random bit patterns";

    const std::vector<uint32_t> special_x(ex::kSpecials.begin(), ex::kSpecials.end());
    const Row root_specials = sqrt_row(special_x.size(), from_list(special_x), kSet);
    report("SqrtSpecials", root_specials);
    EXPECT_EQ(root_specials.misses, 0u) << "sqrt of every special";

    ex::SplitMix xs{0x5EDCull};
    const Row root_edges = sqrt_row(
        8 * kBlock,
        [&xs](uint64_t base, std::size_t n, uint32_t* x) {
            // Block by block: biased exponent 0, 1, 254, 255, then the same negated.
            constexpr uint32_t kExponents[] = {0u, 1u, 254u, 255u};
            const uint64_t block = base / kBlock;
            const uint32_t sign = block >= 4u ? 0x80000000u : 0u;
            const uint32_t exponent = kExponents[block % 4u];
            for (std::size_t j = 0; j < n; ++j) {
                x[j] = sign | (exponent << 23) | (static_cast<uint32_t>(xs.next()) & 0x7FFFFFu);
            }
        },
        kSet);
    report("SqrtExponentEdges", root_edges);
    EXPECT_EQ(root_edges.misses, 0u) << "sqrt over biased exponents 0, 1, 254 and 255, both signs";

    ex::SplitMix patterns{0x5B17ull};
    const Row root_random = sqrt_row(
        uint64_t{1} << 20,
        [&patterns](uint64_t, std::size_t n, uint32_t* x) {
            for (std::size_t j = 0; j < n; ++j) x[j] = static_cast<uint32_t>(patterns.next());
        },
        kSet);
    report("SqrtRandomBits", root_random);
    EXPECT_EQ(root_random.misses, 0u) << "sqrt of random bit patterns";
}

// ===========================================================================
// 5. The battery can see an uncorrected answer, and the port is the port
// ===========================================================================

// The correction's two steps each matter: with y or r 3 ulps off, the
// estimate's truncation alone (Q0, S0) misses, and so does the answer after
// the rounded k-step without the final +-1 (Q1, S1). On the hardest inputs:
// Rendering's near-ties for division, sqrt_near_ties() for the square root.
TEST(Fp32Exact, InjectedErrorIsVisibleWithoutTheCorrection) {
    constexpr int kK[] = {-3, 3};
    const Ks kSet{.list = kK};

    const Row div_q0 = divide_row(kBlock, near_tie_pairs(0x1D1Eull), kSet, Stop::at_estimate);
    report("DivisionStoppedAtQ0", div_q0);
    EXPECT_GT(div_q0.misses, 0u) << "division stopped at Q0: the battery cannot see a missing correction";
    const Row div_q1 = divide_row(kBlock, near_tie_pairs(0x1D1Eull), kSet, Stop::after_step);
    report("DivisionStoppedAfterTheStep", div_q1);
    EXPECT_GT(div_q1.misses, 0u) << "division without the final +-1: the battery cannot see it missing";

    const std::vector<uint32_t> ties = ex::sqrt_near_ties(256);
    const Row sqrt_s0 = sqrt_row(ties.size(), from_list(ties), kSet, Stop::at_estimate);
    report("SqrtStoppedAtS0", sqrt_s0);
    EXPECT_GT(sqrt_s0.misses, 0u) << "sqrt stopped at S0: the battery cannot see a missing correction";
    const Row sqrt_s1 = sqrt_row(ties.size(), from_list(ties), kSet, Stop::after_step);
    report("SqrtStoppedAfterTheStep", sqrt_s1);
    EXPECT_GT(sqrt_s1.misses, 0u) << "sqrt without the final +-1: the battery cannot see it missing";
}

namespace {

// The port over arrays, as the bodies of the flush comparison below: noinline,
// so the scoped wrappers call them once and no float op crosses the MXCSR
// write (core/detail/flush_subnormals.hpp's rule).
SPADE_NOINLINE void port_divide_all(const uint32_t* a, const uint32_t* b, const float* y, uint32_t* out,
                                    std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) out[i] = ex::port_div(a[i], b[i], y[i]);
}

SPADE_NOINLINE void port_sqrt_all(const uint32_t* x, const float* r, uint32_t* out, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) out[i] = ex::port_sqrt(x[i], r[i]);
}

void port_divide_all_flushed(const uint32_t* a, const uint32_t* b, const float* y, uint32_t* out, std::size_t n) {
    const spade::math::ScopedFlushSubnormals flush;
    port_divide_all(a, b, y, out, n);
}

void port_sqrt_all_flushed(const uint32_t* x, const float* r, uint32_t* out, std::size_t n) {
    const spade::math::ScopedFlushSubnormals flush;
    port_sqrt_all(x, r, out, n);
}

}  // namespace

// The port's float ops (the estimate's products, float(R), + 0.5, floor) never
// meet a subnormal, so the device's flushing cannot reach them: the same bits
// under the default MXCSR word and under FTZ|DAZ, over every pair of the
// specials and random bit patterns, estimates k in {-3, 0, 3}.
TEST(Fp32Exact, ThePortIsIndependentOfTheFlushMode) {
    constexpr int kK[] = {-3, 0, 3};
    constexpr std::size_t kN = ex::kSpecials.size();
    std::vector<uint32_t> a, b;
    for (const uint32_t x : ex::kSpecials) {
        for (const uint32_t z : ex::kSpecials) {
            a.push_back(x);
            b.push_back(z);
        }
    }
    ex::SplitMix rng{0xF1A6ull};
    for (std::size_t i = 0; i < kBlock; ++i) {
        const uint64_t r = rng.next();
        a.push_back(static_cast<uint32_t>(r));
        b.push_back(static_cast<uint32_t>(r >> 32));
    }

    // Division: each pair with each estimate, laid out flat.
    std::vector<uint32_t> da, db;
    std::vector<float> y;
    for (std::size_t i = 0; i < a.size(); ++i) {
        for (const int k : kK) {
            da.push_back(a[i]);
            db.push_back(b[i]);
            y.push_back(ex::nudge(ex::div_estimate(b[i]), k));
        }
    }
    std::vector<uint32_t> div_default(da.size()), div_flushed(da.size());
    port_divide_all(da.data(), db.data(), y.data(), div_default.data(), da.size());
    port_divide_all_flushed(da.data(), db.data(), y.data(), div_flushed.data(), da.size());
    uint64_t div_misses = 0;
    for (std::size_t i = 0; i < da.size(); ++i) div_misses += div_default[i] != div_flushed[i] ? 1u : 0u;
    std::printf("[Fp32Exact] FlushModeDivision: n=%zu  misses=%" PRIu64 "\n", da.size(), div_misses);

    // The square root: every a above as x.
    std::vector<uint32_t> x;
    std::vector<float> r;
    for (const uint32_t v : a) {
        for (const int k : kK) {
            x.push_back(v);
            r.push_back(ex::nudge(ex::sqrt_estimate(v), k));
        }
    }
    std::vector<uint32_t> sqrt_default(x.size()), sqrt_flushed(x.size());
    port_sqrt_all(x.data(), r.data(), sqrt_default.data(), x.size());
    port_sqrt_all_flushed(x.data(), r.data(), sqrt_flushed.data(), x.size());
    uint64_t sqrt_misses = 0;
    for (std::size_t i = 0; i < x.size(); ++i) sqrt_misses += sqrt_default[i] != sqrt_flushed[i] ? 1u : 0u;
    std::printf("[Fp32Exact] FlushModeSqrt: n=%zu  misses=%" PRIu64 "\n", x.size(), sqrt_misses);
    std::fflush(stdout);

    EXPECT_EQ(div_misses, 0u) << "the division port's bits depend on the MXCSR flush (" << kN << " specials)";
    EXPECT_EQ(sqrt_misses, 0u) << "the sqrt port's bits depend on the MXCSR flush";
}
