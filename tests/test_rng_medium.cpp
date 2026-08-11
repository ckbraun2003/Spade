#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <glm/vec3.hpp>

#include "core/rng.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"
#include "world/medium.hpp"

// ---------------------------------------------------------------------------
// Task 8 -- the determinism substrate.
//
// Two things are under test and they carry very different burdens of proof:
//
//   * spade::rng is a WIRE CONTRACT. Every recorded snapshot and every entry
//     in the determinism-replay corpus is only replayable while these exact
//     bits keep coming out, so the tests here PIN VALUES, not properties.
//     The golden vectors below were computed once from the derivation in
//     core/rng.hpp and committed; a refactor that changes them has broken
//     compatibility, whether or not the new numbers are "just as random".
//     Distributional tests (moments, sign tests) are the second line only --
//     they cannot see a swapped sin/cos or a reordered u1/u2, which is
//     precisely why the sequences themselves are pinned.
//
//   * spade::Medium is a SEAM. v0 has almost no behaviour, so the tests worth
//     writing are the ones about the shape: sampling through the abstract
//     interface, position-independence, and statelessness.
// ---------------------------------------------------------------------------

namespace {

using spade::rng::Stream;

// One pinned derivation case: inputs, the derived initial state, and the
// first three u64 draws.
struct GoldenVector {
    const char* tag;
    uint64_t seed;
    uint64_t index;
    uint64_t initial_state;
    std::array<uint64_t, 3> first_three;
};

// GOLDEN VECTORS. Computed once from
//   Stream{ splitmix64(seed ^ fnv1a64(tag) ^ index) }
// and committed as literals. These are the numbers a future refactor has to
// reproduce bit-exactly. Chosen to span the input space rather than to look
// tidy: zero seed, all-ones seed, empty tag, a dotted tag, index 0, a large
// index.
constexpr std::array<GoldenVector, 5> kGoldenVectors{{
    {"world", 0x0000000000000000ULL, 0ULL, 0xF30DBE2821EAC7ECULL,
     {0x0D305306D37B140EULL, 0x601DC74F8BA6AC5DULL, 0x7CF3CC6D66E39258ULL}},
    {"dryden", 0xDEADBEEFCAFEF00DULL, 3ULL, 0x3E94A03DE57FF116ULL,
     {0xE94CF97A64AE0906ULL, 0x0D313260A5FEF82EULL, 0xA3750610445DE8A1ULL}},
    {"sensor.noise", 0x0000000000000001ULL, 0ULL, 0x0AD5B26EFBBA567EULL,
     {0x4E5AA38F9533F951ULL, 0x8F72B42E259E96B5ULL, 0x9B5CD89F5E93786EULL}},
    {"spawn", 0x123456789ABCDEF0ULL, 4095ULL, 0x012F2323753819D2ULL,
     {0xA2C8D399512E7828ULL, 0x882418CB9906D7BCULL, 0xB768C6BFADC80CE1ULL}},
    {"", 0xFFFFFFFFFFFFFFFFULL, 1ULL, 0x4F4600B689EF10EEULL,
     {0xCB6E509E560B4F34ULL, 0xB193ABCB9458CB5FULL, 0x45A7DC6675B76510ULL}},
}};

// Counts how often two streams' next_float() draws land on the same side of
// 0.5 over `count` draws. Two independent streams agree on ~half of them; a
// correlated pair does not. This is the sign test.
uint32_t sign_agreements(Stream a, Stream b, uint32_t count) {
    uint32_t agree = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const bool high_a = a.next_float() >= 0.5f;
        const bool high_b = b.next_float() >= 0.5f;
        if (high_a == high_b) ++agree;
    }
    return agree;
}

std::vector<float> draw_gauss(Stream& s, int count) {
    std::vector<float> out;
    out.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) out.push_back(s.next_gauss());
    return out;
}

// A snapshot blob, in the only form this task is allowed to assume: whatever
// a StateRegistry walk hands out, keyed by the array's stable name. Task 7
// owns the real versioned format; nothing here depends on it.
using Blob = std::map<std::string, std::vector<std::byte>>;

Blob save_walk(const spade::StateRegistry& registry) {
    Blob blob;
    registry.for_each_array([&blob](const spade::RegisteredArray& array) {
        blob[array.name].assign(array.data, array.data + array.byte_size());
    });
    return blob;
}

void restore_walk(const spade::StateRegistry& registry, const Blob& blob) {
    registry.for_each_array([&blob](const spade::RegisteredArray& array) {
        const auto it = blob.find(array.name);
        ASSERT_NE(it, blob.end()) << array.name;
        ASSERT_EQ(it->second.size(), array.byte_size()) << array.name;
        std::memcpy(array.data, it->second.data(), it->second.size());
    });
}

}  // namespace

// ---------------------------------------------------------------------------
// The mixers, pinned against published vectors.
// ---------------------------------------------------------------------------

TEST(RngMixers, Fnv1a64MatchesThePublishedTestVectors) {
    // The canonical FNV-1a-64 vectors. Pinning these -- rather than only our
    // own outputs -- is what proves we implemented the NAMED hash, which is
    // the whole point of errata-R4 naming a mixer instead of saying "hash the
    // tag somehow".
    EXPECT_EQ(spade::rng::fnv1a64(""), 0xCBF29CE484222325ULL);
    EXPECT_EQ(spade::rng::fnv1a64("a"), 0xAF63DC4C8601EC8CULL);
    EXPECT_EQ(spade::rng::fnv1a64("foobar"), 0x85944171F73967E8ULL);

    // The empty tag hashes to the offset basis, NOT to zero. R4's warning is
    // that `splitmix64(seed ^ 0) == splitmix64(seed)`, so a derivation whose
    // neutral tag were 0 would hand the untagged stream out twice. This is
    // the property that makes that impossible.
    EXPECT_NE(spade::rng::fnv1a64(""), 0ULL);
}

TEST(RngMixers, Fnv1a64HashesBytesNotCharactersAndIsPrefixSensitive) {
    // Runs over the string's bytes with no terminator and no length prefix:
    // hashing "ab" must equal hashing "a" then continuing with 'b', which is
    // only true if nothing extra is folded in at either end.
    EXPECT_NE(spade::rng::fnv1a64("ab"), spade::rng::fnv1a64("ba"));
    EXPECT_NE(spade::rng::fnv1a64("wind"), spade::rng::fnv1a64("wind "));

    // std::string_view over an embedded NUL is hashed in full -- proof the
    // implementation uses the view's size, not strlen.
    const std::string_view embedded("a\0b", 3);
    EXPECT_EQ(embedded.size(), 3u);
    EXPECT_NE(spade::rng::fnv1a64(embedded), spade::rng::fnv1a64("a"));
}

TEST(RngMixers, Splitmix64IsTheStepFunctionOverAStateSeededWithItsArgument) {
    // splitmix64(x) and Stream::next_u64() must be the SAME arithmetic --
    // core/rng.hpp routes both through splitmix64_step for exactly this
    // reason, and this is the test that would catch them drifting apart.
    for (const uint64_t x : {0ULL, 1ULL, 0x9E3779B97F4A7C15ULL, 0xFFFFFFFFFFFFFFFFULL}) {
        uint64_t state = x;
        const uint64_t stepped = spade::rng::splitmix64_step(state);
        EXPECT_EQ(spade::rng::splitmix64(x), stepped);
        EXPECT_EQ(state, x + spade::rng::kSplitmixGamma) << "the step must advance the state by gamma";
    }

    // A mixer, not a permutation of the low bits: zero must not map to zero.
    EXPECT_NE(spade::rng::splitmix64(0ULL), 0ULL);
}

// ---------------------------------------------------------------------------
// The derivation -- errata-R4 discipline.
// ---------------------------------------------------------------------------

TEST(RngDerivation, GoldenVectorsAreReproducedBitExactly) {
    for (const GoldenVector& v : kGoldenVectors) {
        Stream s = spade::rng::make_stream(v.seed, v.tag, v.index);
        EXPECT_EQ(s.state, v.initial_state) << "tag=" << v.tag << " index=" << v.index;
        EXPECT_EQ(s.cached_gauss, 0.0f) << "a derived stream starts with an empty Box-Muller cache";
        EXPECT_EQ(s.has_cached, 0u) << "a derived stream starts with an empty Box-Muller cache";

        for (std::size_t i = 0; i < v.first_three.size(); ++i) {
            EXPECT_EQ(s.next_u64(), v.first_three[i]) << "tag=" << v.tag << " index=" << v.index << " draw " << i;
        }
    }
}

TEST(RngDerivation, MatchesTheWrittenFormulaComponentByComponent) {
    // The golden vectors above prove "these bits"; this proves "these bits
    // BECAUSE of this formula", so a reader can check the header's claim
    // without recomputing a hash by hand.
    for (const GoldenVector& v : kGoldenVectors) {
        const uint64_t expected = spade::rng::splitmix64(v.seed ^ spade::rng::fnv1a64(v.tag) ^ v.index);
        EXPECT_EQ(spade::rng::make_stream(v.seed, v.tag, v.index).state, expected) << "tag=" << v.tag;
    }
}

TEST(RngDerivation, IsConstexprEvaluable) {
    // Not decoration: constexpr evaluability means the whole derivation is a
    // pure function of its arguments with no I/O, no clock read and no
    // hidden global -- checked by the compiler, for free.
    constexpr Stream s = spade::rng::make_stream(0ULL, "world", 0ULL);
    static_assert(s.state == 0xF30DBE2821EAC7ECULL);

    constexpr uint64_t first = [] {
        Stream local = spade::rng::make_stream(0ULL, "world", 0ULL);
        return local.next_u64();
    }();
    static_assert(first == 0x0D305306D37B140EULL);
    SUCCEED();
}

TEST(RngDerivation, EveryAxisSeparatesTheStream) {
    constexpr uint64_t kSeed = 0xA5A5A5A5A5A5A5A5ULL;
    const Stream base = spade::rng::make_stream(kSeed, "wind", 0);

    EXPECT_NE(base.state, spade::rng::make_stream(kSeed + 1, "wind", 0).state) << "seed axis";
    EXPECT_NE(base.state, spade::rng::make_stream(kSeed, "gust", 0).state) << "tag axis";
    EXPECT_NE(base.state, spade::rng::make_stream(kSeed, "wind", 1).state) << "index axis";

    // R4's named trap, tested directly: an untagged derivation would make
    // index 0 collide with the bare mixed seed -- "a naive index XOR hands
    // drone 0 the same stream the world would take". Even the EMPTY tag must
    // not reproduce splitmix64(seed).
    EXPECT_NE(spade::rng::make_stream(kSeed, "", 0).state, spade::rng::splitmix64(kSeed));

    // No collisions across a grid of tags x indices.
    const std::array<const char*, 4> tags{"world", "dryden", "sensor.noise", "spawn"};
    std::vector<uint64_t> states;
    for (const char* tag : tags) {
        for (uint64_t index = 0; index < 16; ++index) {
            states.push_back(spade::rng::make_stream(kSeed, tag, index).state);
        }
    }
    std::vector<uint64_t> unique = states;
    std::sort(unique.begin(), unique.end());
    unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
    EXPECT_EQ(unique.size(), states.size()) << "tag x index derivation collided";
}

TEST(RngDomainSeparation, AdjacentTagsSeedsAndIndicesAreUncorrelatedBySignTest) {
    // Differing streams is necessary but nowhere near sufficient: two streams
    // seeded one apart could still march in lockstep. The sign test asks
    // whether their draws agree on the coin flip "is this >= 0.5" more often
    // than chance.
    //
    // Under independence the agreement count is Binomial(N, 1/2): mean N/2,
    // stddev sqrt(N)/2 = 158 at N = 100k. The 1000 tolerance is ~6.3 sigma,
    // so this is a structural-failure detector, not a flaky statistic -- and
    // being seeded, it is exactly reproducible anyway. Observed deviations
    // are all under 300.
    constexpr uint32_t kDraws = 100'000;
    constexpr uint32_t kTolerance = 1000;
    constexpr uint64_t kSeed = 0xA5A5A5A5A5A5A5A5ULL;

    struct Pair {
        const char* what;
        Stream a;
        Stream b;
    };
    const std::array<Pair, 6> pairs{{
        {"adjacent index 0/1", spade::rng::make_stream(kSeed, "wind", 0), spade::rng::make_stream(kSeed, "wind", 1)},
        {"adjacent index 1/2", spade::rng::make_stream(kSeed, "wind", 1), spade::rng::make_stream(kSeed, "wind", 2)},
        {"one-char-apart tags", spade::rng::make_stream(kSeed, "noise0", 7), spade::rng::make_stream(kSeed, "noise1", 7)},
        {"short tags a/b", spade::rng::make_stream(kSeed, "a", 0), spade::rng::make_stream(kSeed, "b", 0)},
        {"adjacent seeds", spade::rng::make_stream(kSeed, "wind", 0), spade::rng::make_stream(kSeed + 1, "wind", 0)},
        {"far-apart indices", spade::rng::make_stream(kSeed, "wind", 0),
         spade::rng::make_stream(kSeed, "wind", 0xFFFFFFFFULL)},
    }};

    for (const Pair& p : pairs) {
        const uint32_t agree = sign_agreements(p.a, p.b, kDraws);
        const int32_t deviation = static_cast<int32_t>(agree) - static_cast<int32_t>(kDraws / 2);
        EXPECT_LT(deviation, static_cast<int32_t>(kTolerance))
            << p.what << ": " << agree << " agreements in " << kDraws;
        EXPECT_GT(deviation, -static_cast<int32_t>(kTolerance))
            << p.what << ": " << agree << " agreements in " << kDraws;
    }
}

// ---------------------------------------------------------------------------
// next_float -- the uniform construction.
// ---------------------------------------------------------------------------

TEST(RngUniform, IsBuiltFromTheTopTwentyFourBitsScaledByTheQuantum) {
    // The construction, pinned against a parallel raw stream. This is what
    // stops a well-meaning "use all 53 bits" change from silently rewriting
    // every recorded run.
    Stream floats = spade::rng::make_stream(99, "uniform.construction", 0);
    Stream raw = spade::rng::make_stream(99, "uniform.construction", 0);

    for (int i = 0; i < 4096; ++i) {
        const float expected = static_cast<float>(raw.next_u64() >> 40) * spade::rng::kUniformQuantum;
        EXPECT_EQ(floats.next_float(), expected) << "draw " << i;
    }
    EXPECT_EQ(spade::rng::kUniformQuantum, 1.0f / 16777216.0f);
}

TEST(RngUniform, StaysInTheHalfOpenUnitIntervalAndIsExactlyRepresentable) {
    Stream s = spade::rng::make_stream(777, "uniform.range", 0);
    float lowest = 2.0f;
    float highest = -1.0f;
    for (int i = 0; i < 200'000; ++i) {
        const float u = s.next_float();
        ASSERT_GE(u, 0.0f) << "draw " << i;
        ASSERT_LT(u, 1.0f) << "draw " << i;
        // Every output must sit exactly on the 2^-24 grid: multiplying back
        // up must land on an integer with no rounding error at all.
        const float scaled = u * 16777216.0f;
        ASSERT_EQ(scaled, std::floor(scaled)) << "draw " << i << " is not on the 2^-24 grid";
        lowest = std::min(lowest, u);
        highest = std::max(highest, u);
    }
    // Sanity that the range is actually being covered, not clustered.
    EXPECT_LT(lowest, 0.001f);
    EXPECT_GT(highest, 0.999f);
}

TEST(RngUniform, MeanIsOneHalfOverAMillionDraws) {
    Stream s = spade::rng::make_stream(777, "uniform.moments", 0);
    double sum = 0.0;  // double ACCUMULATOR only: the draws are fp32 (the
                       // engine's contract); summing a million of them in
                       // fp32 would measure the summation, not the generator.
    constexpr int kDraws = 1'000'000;
    for (int i = 0; i < kDraws; ++i) sum += static_cast<double>(s.next_float());
    EXPECT_NEAR(sum / kDraws, 0.5, 3e-3);
}

// ---------------------------------------------------------------------------
// next_gauss -- Box-Muller, and the operation order that makes it replayable.
// ---------------------------------------------------------------------------

TEST(RngGauss, OperationOrderIsPinnedDrawU1ThenU2AndReturnTheSinTermFirst) {
    // The single most important test in this file. Every alternative
    // formulation -- u2 before u1, cos returned first, polar instead of
    // basic -- yields a perfectly good standard normal, so the moments test
    // below cannot tell them apart. Only recomputing the exact expression
    // from the same raw draws can.
    Stream gauss = spade::rng::make_stream(4242, "gauss.order", 11);
    Stream raw = spade::rng::make_stream(4242, "gauss.order", 11);

    for (int pair = 0; pair < 64; ++pair) {
        // Order under test: u1 first, then u2, both from the same stream.
        const float u1_drawn = static_cast<float>(raw.next_u64() >> 40) * spade::rng::kUniformQuantum;
        const float u2 = static_cast<float>(raw.next_u64() >> 40) * spade::rng::kUniformQuantum;
        const float u1 = (u1_drawn < spade::rng::kUniformQuantum) ? spade::rng::kUniformQuantum : u1_drawn;

        // The engine's own log/sin/cos, NOT the platform's -- next_gauss calls
        // those, and re-deriving with std::log/std::sin/std::cos would make
        // this test assert that the two agree, which is exactly the thing that
        // is not true (core/fp32_math.hpp). What is under test here is the
        // ORDER, so the primitives on both sides must be the same primitives.
        const float r = std::sqrt(-2.0f * spade::math::log32(u1));
        const float theta = spade::rng::kTwoPi * u2;

        EXPECT_EQ(gauss.next_gauss(), r * spade::math::sin32(theta))
            << "pair " << pair << ": sin term must come FIRST";
        EXPECT_EQ(gauss.next_gauss(), r * spade::math::cos32(theta))
            << "pair " << pair << ": cos term must be the cached one";
    }

    // Both streams consumed the same number of u64 draws: two uniforms per
    // gaussian PAIR, never per gaussian.
    EXPECT_EQ(gauss.state, raw.state);
}

TEST(RngGauss, TheSecondValueOfAPairCostsNoDraw) {
    Stream s = spade::rng::make_stream(4242, "gauss.cache", 0);

    const uint64_t before = s.state;
    (void)s.next_gauss();
    const uint64_t after_first = s.state;
    EXPECT_NE(after_first, before) << "the first of a pair draws two uniforms";
    EXPECT_EQ(s.has_cached, 1u);

    (void)s.next_gauss();
    EXPECT_EQ(s.state, after_first) << "the second of a pair must consume no randomness";
    EXPECT_EQ(s.has_cached, 0u);
}

TEST(RngGauss, MomentsOverAMillionDraws) {
    Stream s = spade::rng::make_stream(12345, "gauss.moments", 0);
    constexpr int kDraws = 1'000'000;

    // double accumulators over fp32 draws -- see the note in the uniform
    // moments test. Tolerances come from the task brief; for reference the
    // standard error of the mean at this N is 1e-3, so 3e-3 is a 3-sigma
    // band, and this is a fixed seed, so the outcome is not a coin flip.
    double sum = 0.0;
    double sum_sq = 0.0;
    for (int i = 0; i < kDraws; ++i) {
        const double x = static_cast<double>(s.next_gauss());
        sum += x;
        sum_sq += x * x;
    }
    const double mean = sum / kDraws;
    const double variance = sum_sq / kDraws - mean * mean;

    EXPECT_NEAR(mean, 0.0, 3e-3);
    EXPECT_NEAR(std::sqrt(variance), 1.0, 3e-3);
}

TEST(RngGauss, ClampsTheZeroUniformInsteadOfReturningInfinity) {
    // u1 == 0 makes log(u1) == -inf, and next_float CAN return exactly 0
    // (probability 2^-24). kUniformQuantum is the clamp, chosen to be the
    // smallest non-zero draw so it perturbs nothing else.
    //
    // 0x55BB00 is a state -- found by search, pinned here -- whose next draw
    // has all-zero top 24 bits, i.e. the case that actually exercises the
    // clamp. If a mixer change ever invalidates it the first assertion fails
    // loudly, which is the correct outcome: the pinned bits changed.
    Stream zero_case{0x000000000055BB00ULL, 0.0f, 0u};
    Stream probe = zero_case;
    ASSERT_EQ(probe.next_float(), 0.0f) << "the pinned state no longer produces a zero uniform";

    const float g = zero_case.next_gauss();
    EXPECT_TRUE(std::isfinite(g));
    EXPECT_TRUE(std::isfinite(zero_case.cached_gauss));

    // The clamp is what bounds the radius, and therefore |next_gauss()|.
    const float max_radius = std::sqrt(-2.0f * std::log(spade::rng::kUniformQuantum));
    EXPECT_TRUE(std::isfinite(max_radius));
    EXPECT_NEAR(max_radius, 5.768f, 1e-2f);
    EXPECT_LE(std::abs(g), max_radius);
}

// ---------------------------------------------------------------------------
// No hidden state.
// ---------------------------------------------------------------------------

TEST(RngStreamLayout, IsAPodWithPinnedOffsets) {
    // Restates the header's static_asserts at runtime so a layout regression
    // reads as a named failure and the numbers appear in test output --
    // same convention as test_state.cpp's layout test.
    EXPECT_EQ(sizeof(Stream), 16u);
    EXPECT_EQ(alignof(Stream), 8u);
    EXPECT_EQ(offsetof(Stream, state), 0u);
    EXPECT_EQ(offsetof(Stream, cached_gauss), 8u);
    EXPECT_EQ(offsetof(Stream, has_cached), 12u);
    EXPECT_TRUE(std::is_trivially_copyable_v<Stream>);
    EXPECT_TRUE(std::is_standard_layout_v<Stream>);
}

TEST(RngNoHiddenState, TheSixteenBytesAreTheWholeStream) {
    // The claim: a Stream's entire future is determined by its 16 bytes. Copy
    // them out mid-pair (so the Box-Muller cache is live), let the original
    // run on, then reconstitute a stream from the bytes alone and demand the
    // same continuation. Anything held in a static, a thread_local or an
    // engine-global would survive the memcpy and break this.
    Stream original = spade::rng::make_stream(2026, "hidden.state", 8);
    (void)original.next_gauss();  // leaves a cached partner behind
    (void)original.next_float();

    std::array<std::byte, sizeof(Stream)> bytes{};
    std::memcpy(bytes.data(), &original, sizeof(Stream));

    const std::vector<float> from_original = draw_gauss(original, 8);

    Stream reconstituted{};
    std::memcpy(&reconstituted, bytes.data(), sizeof(Stream));
    const std::vector<float> from_bytes = draw_gauss(reconstituted, 8);

    EXPECT_EQ(from_original, from_bytes);
}

TEST(RngNoHiddenState, TheByteImageIsCanonicalForAGivenFuture) {
    // The converse of "no hidden state": no SURPLUS state either. Two streams
    // that will produce identical futures must be byte-identical, or a
    // snapshot comparison cannot be a memcmp and a determinism run reports
    // differences that are not differences.
    //
    // The case that catches a consumed-but-not-cleared Box-Muller partner:
    // both orderings below make three u64 draws and leave the cache empty, so
    // their futures are identical -- but the stale partner they would carry
    // differs.
    Stream a = spade::rng::make_stream(31337, "canonical", 0);
    (void)a.next_gauss();
    (void)a.next_gauss();
    (void)a.next_float();

    Stream b = spade::rng::make_stream(31337, "canonical", 0);
    (void)b.next_float();
    (void)b.next_gauss();
    (void)b.next_gauss();

    ASSERT_EQ(a.state, b.state) << "the two orderings must consume the same number of u64 draws";
    ASSERT_EQ(a.has_cached, 0u);
    ASSERT_EQ(b.has_cached, 0u);
    EXPECT_EQ(a.cached_gauss, 0.0f) << "a consumed partner must be cleared, not left in the bytes";
    EXPECT_EQ(a, b) << "identical futures must be byte-identical";
    EXPECT_EQ(std::memcmp(&a, &b, sizeof(Stream)), 0);

    // ...and the futures really are identical.
    EXPECT_EQ(draw_gauss(a, 4), draw_gauss(b, 4));
}

TEST(RngNoHiddenState, InterleavedStreamsDoNotShareABoxMullerCache) {
    // The textbook Box-Muller keeps its spare value in a `static float` with
    // a `static bool have_spare`. That is the exact defect this test exists
    // to catch: with a shared cache, interleaving two streams hands each of
    // them the other's partner value. Both streams here start IDENTICAL, so a
    // shared cache would still look plausible on a single-stream test.
    Stream a = spade::rng::make_stream(5150, "interleave", 0);
    Stream b = spade::rng::make_stream(5150, "interleave", 1);

    Stream a_solo = a;
    Stream b_solo = b;
    const std::vector<float> a_expected = draw_gauss(a_solo, 6);
    const std::vector<float> b_expected = draw_gauss(b_solo, 6);

    std::vector<float> a_actual;
    std::vector<float> b_actual;
    for (int i = 0; i < 6; ++i) {
        a_actual.push_back(a.next_gauss());
        b_actual.push_back(b.next_gauss());
    }

    EXPECT_EQ(a_actual, a_expected);
    EXPECT_EQ(b_actual, b_expected);
}

// ---------------------------------------------------------------------------
// Snapshot safety -- streams are registered state.
//
// These tests use ONLY the StateRegistry walk (RegisteredArray::data +
// byte_size). They deliberately do not touch Task 7's snapshot format: the
// claim under test is that a stream needs nothing from the format beyond its
// raw bytes, which is what makes it snapshot-safe in the first place.
// ---------------------------------------------------------------------------

TEST(RngSnapshot, StreamsRegisterAsStateAndAppearInTheWalk) {
    spade::ArenaSet arenas(2);
    const auto streams = arenas.register_array<Stream>("rng.streams", 4);
    ASSERT_TRUE(streams.has_value());

    const spade::RegisteredArray* entry = arenas.registry().find("rng.streams");
    ASSERT_NE(entry, nullptr) << "an unregistered rng stream is state a snapshot would miss";
    EXPECT_EQ(entry->elem_size, sizeof(Stream));
    EXPECT_EQ(entry->world_count, 2u);
    EXPECT_EQ(entry->capacity_per_world, 4u);
    EXPECT_EQ(entry->byte_size(), sizeof(Stream) * 8u);
}

TEST(RngSnapshot, ARegistryWalkRoundTripResumesTheIdenticalDrawSequence) {
    constexpr uint32_t kWorlds = 2;
    constexpr uint32_t kCapacity = 3;
    constexpr uint64_t kWorldSeed = 0xC0FFEE0000BEEFULL;

    spade::ArenaSet saved(kWorlds);
    const auto saved_id = saved.register_array<Stream>("rng.streams", kCapacity);
    ASSERT_TRUE(saved_id.has_value());

    // Seed one stream per slot, domain-separated by slot index, then draw an
    // uneven number of values from each so the saved states are all
    // different and some sit mid-Box-Muller-pair.
    {
        const auto slots = saved.array(*saved_id);
        ASSERT_TRUE(slots.has_value());
        for (uint32_t i = 0; i < slots->size(); ++i) {
            ASSERT_TRUE(saved.alloc_slot(*saved_id, i / kCapacity).has_value());
            (*slots)[i] = spade::rng::make_stream(kWorldSeed, "sensor.noise", i);
            for (uint32_t d = 0; d < i; ++d) (void)(*slots)[i].next_gauss();
            (void)(*slots)[i].next_float();
        }
    }

    // What the untouched originals would produce next -- the control.
    std::vector<std::vector<float>> expected;
    {
        const auto slots = saved.array(*saved_id);
        ASSERT_TRUE(slots.has_value());
        for (Stream s : *slots) expected.push_back(draw_gauss(s, 5));  // copies, originals untouched
    }

    const Blob blob = save_walk(saved.registry());
    EXPECT_EQ(blob.size(), 2u) << "one arena contributes its elements and its slot_to_world map";

    spade::ArenaSet restored(kWorlds);
    const auto restored_id = restored.register_array<Stream>("rng.streams", kCapacity);
    ASSERT_TRUE(restored_id.has_value());
    restore_walk(restored.registry(), blob);
    ASSERT_TRUE(restored.resync_from_slot_to_world(*restored_id).has_value());

    const auto restored_slots = restored.array(*restored_id);
    ASSERT_TRUE(restored_slots.has_value());
    ASSERT_EQ(restored_slots->size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(draw_gauss((*restored_slots)[i], 5), expected[i]) << "slot " << i;
    }
}

TEST(RngSnapshot, SavingBetweenTheTwoHalvesOfABoxMullerPairResumesIdentically) {
    // The case a `static float spare` implementation gets wrong and a
    // moments test never notices: snapshot AFTER the first of a pair, restore
    // into a fresh arena, and the second value must still be the partner
    // computed before the save.
    constexpr uint64_t kWorldSeed = 0x1234ULL;

    // Control: one uninterrupted stream.
    Stream control = spade::rng::make_stream(kWorldSeed, "dryden", 0);
    const float control_first = control.next_gauss();
    const float control_second = control.next_gauss();
    const float control_third = control.next_gauss();

    spade::ArenaSet saved(1);
    const auto saved_id = saved.register_array<Stream>("rng.streams", 1);
    ASSERT_TRUE(saved_id.has_value());
    ASSERT_TRUE(saved.alloc_slot(*saved_id, 0).has_value());
    {
        const auto slots = saved.array(*saved_id);
        ASSERT_TRUE(slots.has_value());
        (*slots)[0] = spade::rng::make_stream(kWorldSeed, "dryden", 0);
        EXPECT_EQ((*slots)[0].next_gauss(), control_first);
        // Mid-pair: the partner is computed but not yet handed out.
        ASSERT_EQ((*slots)[0].has_cached, 1u) << "the cache must be live for this test to mean anything";
    }

    const Blob blob = save_walk(saved.registry());

    spade::ArenaSet restored(1);
    const auto restored_id = restored.register_array<Stream>("rng.streams", 1);
    ASSERT_TRUE(restored_id.has_value());
    restore_walk(restored.registry(), blob);
    ASSERT_TRUE(restored.resync_from_slot_to_world(*restored_id).has_value());

    const auto restored_slots = restored.array(*restored_id);
    ASSERT_TRUE(restored_slots.has_value());
    Stream& resumed = (*restored_slots)[0];
    EXPECT_EQ(resumed.has_cached, 1u) << "the Box-Muller cache must live in the snapshotted bytes";
    EXPECT_EQ(resumed.next_gauss(), control_second) << "the cached partner did not survive the round trip";
    EXPECT_EQ(resumed.next_gauss(), control_third) << "the stream did not resume in step after the pair";
}

// ---------------------------------------------------------------------------
// Medium v0.
// ---------------------------------------------------------------------------

namespace {

spade::WorldParams make_world_params() {
    spade::WorldParams params{};
    params.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    params.air_density = 1.225f;
    params.wind = glm::vec3(3.5f, -0.25f, -1.75f);
    params.body_capacity = 8;
    params.body_count = 1;
    params.seed = 0xABCDEF0123456789ULL;
    return params;
}

}  // namespace

TEST(MediumV0, ConstantMediumReturnsTheWorldParamsConstantsVerbatim) {
    const spade::WorldParams params = make_world_params();
    const spade::ConstantMedium medium;

    const spade::MediumSample sample = medium.sample(params, glm::vec3(0.0f));
    EXPECT_EQ(sample.density, params.air_density);
    EXPECT_EQ(sample.wind, params.wind);
}

TEST(MediumV0, ConstantMediumIgnoresPosition) {
    const spade::WorldParams params = make_world_params();
    const spade::ConstantMedium medium;
    const spade::MediumSample reference = medium.sample(params, glm::vec3(0.0f));

    for (const glm::vec3 pos : {glm::vec3(1000.0f, -500.0f, 250.0f), glm::vec3(-1e6f, 1e6f, 0.0f),
                                glm::vec3(0.001f, 0.0f, -0.001f)}) {
        const spade::MediumSample sample = medium.sample(params, pos);
        EXPECT_EQ(sample.density, reference.density);
        EXPECT_EQ(sample.wind, reference.wind);
    }
}

TEST(MediumV0, SamplesThroughTheAbstractInterfaceAndIsPerWorld) {
    // The seam is the deliverable: callers hold `const Medium&` so a stateful
    // implementation can replace this one without touching them. And ONE
    // instance serves every world -- the WorldParams row selects the world,
    // which is what makes ConstantMedium's statelessness a property rather
    // than an accident.
    const spade::ConstantMedium concrete;
    const spade::Medium& medium = concrete;

    spade::WorldParams world_a = make_world_params();
    spade::WorldParams world_b = make_world_params();
    world_b.air_density = 0.4135f;  // ~10 km altitude
    world_b.wind = glm::vec3(-12.0f, 0.0f, 4.0f);

    const spade::MediumSample a = medium.sample(world_a, glm::vec3(1.0f, 2.0f, 3.0f));
    const spade::MediumSample b = medium.sample(world_b, glm::vec3(1.0f, 2.0f, 3.0f));

    EXPECT_EQ(a.density, world_a.air_density);
    EXPECT_EQ(a.wind, world_a.wind);
    EXPECT_EQ(b.density, world_b.air_density);
    EXPECT_EQ(b.wind, world_b.wind);
    EXPECT_NE(a.density, b.density);
}

TEST(MediumV0, SampleIsFpThirtyTwo) {
    // Global constraint: fp32 only in engine state and math. A double that
    // sneaks into the medium would show up here.
    static_assert(std::is_same_v<decltype(spade::MediumSample::density), float>);
    static_assert(std::is_same_v<decltype(spade::MediumSample::wind), glm::vec3>);
    static_assert(sizeof(spade::MediumSample) == 16);
    SUCCEED();
}
