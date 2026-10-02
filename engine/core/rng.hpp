#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "core/fp32_math.hpp"

// ---------------------------------------------------------------------------
// spade::rng -- the determinism substrate. EVERY random draw in the engine
// flows through this header (engine design spec section 3: "every stochastic
// element (Dryden, sensor noise, spawn randomization) draws from per-world,
// per-system splitmix64 streams domain-separated from the world seed"; see
// docs/design/core/02-state-and-snapshot.md, "Randomness").
//
// The global constraints make the negative half of that rule explicit and
// review-rejectable: no std::random_device, no rand(), no std::mt19937 seeded
// from anything, and no wall-clock read anywhere under engine/. A
// stream's entire future is a function of (world seed, domain tag, index) and
// of how many values have been drawn from it -- nothing else.
//
// WHAT IS PINNED HERE, AND WHY IT CANNOT BE "CLEANED UP"
//
// Everything in this header is a wire contract in the same sense that
// state/layout.hpp's field offsets are. A snapshot blob written by one build
// is replayed by another; the CPU reference twin and the S6 GPU path must
// produce the same numbers bit for bit (P1/P2). So the following are FIXED,
// and the golden vectors in tests/test_rng_medium.cpp are the enforcement:
//
//   * the splitmix64 constants and step order;
//   * the FNV-1a-64 constants, and that the hash runs over the tag's STRING
//     BYTES (no terminator, no length prefix);
//   * the derivation `splitmix64(world_seed ^ fnv1a64(tag) ^ index)`;
//   * next_float's exact construction (top 24 bits, scaled by 2^-24);
//   * next_gauss's operation order (u1 before u2; sin term returned first,
//     cos term cached) and its clamp constant.
//
// A refactor that changes any of them silently invalidates every recorded
// snapshot and every replay in the determinism corpus. If one must change,
// it is a versioned change with new golden vectors, not a tidy-up.
//
// THREADING. A Stream is single-owner by design: it is plain mutable state
// with no synchronization, and there is deliberately none to add. Passes are
// scheduled, not raced; each pass owns the streams it draws from, and two
// systems that need independent randomness take two DOMAIN TAGS rather than
// sharing one stream under a lock. That is also what makes the CPU and GPU
// paths able to agree -- a lock-ordered interleaving is not reproducible on a
// GPU, but "world w, system `tag`, index i" is.
//
// NO HIDDEN STATE, AND NO SURPLUS STATE. Every bit a Stream needs to
// reproduce its future lives in the Stream POD, including the Box-Muller
// partner value. There is no static, no thread_local, and no engine-global
// generator: that is precisely what makes a stream snapshot-safe by merely
// being copied, and it is why next_gauss caches into a member rather than the
// textbook `static bool`.
//
// The converse also holds and is enforced: the 16 bytes contain nothing that
// does NOT affect the future. next_gauss clears the cache slot when it
// consumes it (see there), so two streams with identical futures are always
// byte-identical -- which is what lets a snapshot comparison be a memcmp.
// ---------------------------------------------------------------------------

namespace spade::rng {

// --- splitmix64 -------------------------------------------------------------

// The golden-ratio increment (2^64 / phi, odd). Advancing by an odd constant
// is what gives splitmix64 its full 2^64 period.
inline constexpr uint64_t kSplitmixGamma = 0x9E3779B97F4A7C15ULL;
inline constexpr uint64_t kSplitmixMulA = 0xBF58476D1CE4E5B9ULL;
inline constexpr uint64_t kSplitmixMulB = 0x94D049BB133111EBULL;

// One splitmix64 step: advance `state` by gamma, then run the two-multiply
// xor-shift finalizer over the ADVANCED state and return it.
//
// Spelled exactly once in the engine. Both the seed derivation (via
// splitmix64() below) and every draw (via Stream::next_u64) route through
// this function, so the two can never drift onto different constants -- a
// class of bug that would show up only as a determinism-corpus failure weeks
// later.
constexpr uint64_t splitmix64_step(uint64_t& state) noexcept {
    state += kSplitmixGamma;
    uint64_t z = state;
    z = (z ^ (z >> 30)) * kSplitmixMulA;
    z = (z ^ (z >> 27)) * kSplitmixMulB;
    return z ^ (z >> 31);
}

// splitmix64 as the one-argument MIXER: one step over a state initialized to
// `x`. The mixer is named rather than left to taste because XOR alone is a
// poor mixer, and two implementations that pick differently would silently
// diverge -- so this is the named one, not an equivalent-looking substitute.
constexpr uint64_t splitmix64(uint64_t x) noexcept {
    uint64_t state = x;
    return splitmix64_step(state);
}

// --- FNV-1a 64 --------------------------------------------------------------

inline constexpr uint64_t kFnv1aOffsetBasis = 0xCBF29CE484222325ULL;
inline constexpr uint64_t kFnv1aPrime = 0x100000001B3ULL;

// FNV-1a over the tag's raw string bytes: no NUL terminator, no length
// prefix, byte order as written. Tags are ASCII identifiers by convention
// ("world", "dryden", "sensor.noise"), so there is no encoding question to
// get wrong -- but the hash is defined over bytes, not characters, and a
// non-ASCII tag would hash its UTF-8 bytes.
//
// Note that fnv1a64("") is the offset basis, NOT zero. That is load-bearing:
// `splitmix64(scene_seed ^ 0)` equals `splitmix64(scene_seed)`,
// so an untagged derivation hands index 0 the stream the raw seed would take.
// Because even the empty tag contributes the offset basis, this formula has
// no such collision.
constexpr uint64_t fnv1a64(std::string_view tag) noexcept {
    uint64_t hash = kFnv1aOffsetBasis;
    for (const char c : tag) {
        hash ^= static_cast<uint64_t>(static_cast<unsigned char>(c));
        hash *= kFnv1aPrime;
    }
    return hash;
}

// --- the uniform and gaussian constructions ---------------------------------

// 2^-24. Two jobs, deliberately the same number:
//
//   1. next_float's scale. 24 bits is float's full significand, so every
//      value next_float can return is exactly representable and the mapping
//      integer -> float loses nothing. A wider draw would round, and rounding
//      up from the top of the range is how a "[0,1)" generator quietly starts
//      returning 1.0f.
//   2. next_gauss's clamp floor on u1. It is exactly the smallest NON-ZERO
//      value next_float can produce, so the clamp changes the result for one
//      input only -- u1 == 0, where log(u1) would be -inf -- and maps it onto
//      the neighbouring representable draw rather than onto some arbitrary
//      epsilon. Consequence, stated so nobody rediscovers it as a bug: the
//      radius is capped at sqrt(-2 * ln(2^-24)) ~ 5.768, so |next_gauss()| is
//      bounded by ~5.768 sigma. For a 25 Hz / 200 Hz sim that ceiling is
//      never reached by chance anyway (p ~ 8e-9 per draw).
inline constexpr float kUniformQuantum = 0x1.0p-24f;

// 2*pi in fp32. Spelled as a constant rather than 2.0f * pi so the product is
// not recomputed (and possibly re-rounded differently) per call site.
inline constexpr float kTwoPi = 6.28318530717958647692f;

// ---------------------------------------------------------------------------
// Stream -- one seeded, domain-separated random sequence.
//
// A POD by construction (trivially copyable, standard layout, no implicit
// padding, 16 bytes) so it can live in a registered state array and ride a
// snapshot blob as raw bytes, exactly like state/layout.hpp's structs. The
// static_asserts below are the enforcement; see the "NO HIDDEN STATE" note at
// the top of this header for why `cached_gauss`/`has_cached` are MEMBERS.
//
// Layout, two 8-byte rows:
//   row 0  state
//   row 1  cached_gauss | has_cached
//
// A Stream is never an array of its own: it is embedded in registered rows
// (DrydenState, ImuSensorRow, GnssSensorRow) that the Vulkan mirror uploads
// byte for byte, so it is 8-byte aligned rather than padded to 16. The
// offsets asserted here are what shaders/shared/layouts.slang's RngStream
// reproduces.
// ---------------------------------------------------------------------------
struct Stream {
    uint64_t state = 0;         // the splitmix64 state; ALL of the entropy
    float cached_gauss = 0.0f;  // the unconsumed half of a Box-Muller pair
    uint32_t has_cached = 0;    // 1 iff cached_gauss holds a live value

    // Uniform 64-bit draw. The primitive; everything else is built on it.
    constexpr uint64_t next_u64() noexcept { return splitmix64_step(state); }

    // Uniform float in [0, 1).
    //
    // CONSTRUCTION, PINNED: take the TOP 24 bits of the 64-bit draw
    // (`>> 40`), convert that integer in [0, 2^24) to float exactly, and
    // scale by 2^-24. The top bits are used rather than the low ones because
    // that is the half a truncating generator is judged on, and because it
    // makes the mapping a plain shift on both CPU and GPU. The result is
    // uniform on a 2^-24 grid, includes 0.0f, and can never reach 1.0f.
    constexpr float next_float() noexcept {
        return static_cast<float>(next_u64() >> 40) * kUniformQuantum;
    }

    // Standard normal draw, mean 0, stddev 1. Basic (non-polar) Box-Muller.
    //
    // OPERATION ORDER, PINNED -- this is the parity-relevant part, and the
    // reason the textbook formulation is not good enough as documentation:
    //
    //   1. if a partner value is cached, consume and return it (no draw);
    //   2. otherwise draw u1 = next_float(), THEN u2 = next_float() --
    //      in that order, from the same stream;
    //   3. clamp u1 up to kUniformQuantum if it is 0 (see that constant);
    //   4. r     = sqrt(-2 * ln(u1));
    //   5. theta = kTwoPi * u2;
    //   6. cache r * cos(theta) as the partner;
    //   7. RETURN r * sin(theta) -- the SIN term first.
    //
    // Steps 6/7 are a coin flip that had to be called; sin-first is the call.
    // Swapping them, or drawing u2 before u1, produces a perfectly valid
    // normal distribution and a completely different sequence -- which is why
    // a moments test cannot catch the mistake and test_rng_medium.cpp pins
    // the sequence itself.
    //
    // All arithmetic is fp32 (global constraint).
    //
    // THE LOGARITHM AND THE TWO CIRCULAR FUNCTIONS ARE THE ENGINE'S OWN
    // (core/fp32_math.hpp), NOT THE PLATFORM'S. This is the one place in the
    // engine where that distinction is load-bearing enough to be worth
    // restating at the call site: IEEE 754 mandates correct rounding for
    // +, -, *, / and sqrt, and mandates NOTHING for log, sin and cos. The MSVC
    // CRT and glibc both implement them to well under one ulp, and they
    // disagree by one ulp on roughly one input in a hundred -- which made the
    // committed determinism corpus a recording of whichever libm generated it.
    // It was caught exactly that way: the `bounce` golden reproduced on
    // MSVC and failed on gcc, and the whole of the divergence was
    // cos(0x3FA9EA7E) and sin(0x3F8A17F6) landing one ulp apart inside
    // dryden_init's five draws for two of that scenario's four worlds.
    //
    // std::sqrt below is deliberately KEPT: IEEE mandates it, so it is already
    // bit-identical everywhere and replacing it would trade a guarantee for a
    // polynomial.
    //
    // Note for S6: with log32/sin32/cos32 the gaussian sequence is now an
    // EXACT parity claim rather than a tolerance one -- the GPU mirror
    // implements these same three routines rather than calling the device's
    // intrinsics. Draw COUNTS were already exact, which is what keeps the two
    // paths' streams in step.
    float next_gauss() noexcept {
        if (has_cached != 0u) {
            // Clearing the slot as well as the flag is not tidiness -- it is
            // what makes these 16 bytes a CANONICAL function of the stream's
            // future. Leaving the consumed partner behind, two streams with
            // identical futures could still differ byte-wise: `gauss, gauss,
            // float` and `float, gauss, gauss` both land on the same `state`
            // with an empty cache, but would carry different stale partners.
            // memcmp on a snapshot blob (and the defaulted operator== below)
            // would then report a difference where there is none -- a false
            // determinism failure, the most expensive kind to chase.
            const float value = cached_gauss;
            cached_gauss = 0.0f;
            has_cached = 0u;
            return value;
        }

        float u1 = next_float();
        const float u2 = next_float();
        if (u1 < kUniformQuantum) u1 = kUniformQuantum;

        const float r = std::sqrt(-2.0f * math::log32(u1));
        const float theta = kTwoPi * u2;

        cached_gauss = r * math::cos32(theta);
        has_cached = 1u;
        return r * math::sin32(theta);
    }

    friend constexpr bool operator==(const Stream&, const Stream&) noexcept = default;
};

static_assert(std::is_standard_layout_v<Stream>, "Stream must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<Stream>, "Stream must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<Stream>, "registered array slots are never individually destroyed");
static_assert(sizeof(Stream) == 16, "Stream layout is a snapshot contract");
static_assert(alignof(Stream) == 8, "Stream layout is a snapshot contract");
static_assert(offsetof(Stream, state) == 0);
static_assert(offsetof(Stream, cached_gauss) == 8);
static_assert(offsetof(Stream, has_cached) == 12);

// Named fields account for every byte: no implicit padding, so a byte-wise
// snapshot of a Stream array carries no indeterminate bytes and two blobs are
// comparable with memcmp. Same discipline (and same reason) as BodyState's
// sum assert in state/layout.hpp.
static_assert(sizeof(Stream::state) + sizeof(Stream::cached_gauss) + sizeof(Stream::has_cached) == sizeof(Stream),
              "Stream has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// The derivation (docs/design/core/02-state-and-snapshot.md, "Randomness"):
//
//     stream(world_seed, domain_tag, index)
//         = Stream{ splitmix64(world_seed ^ fnv1a64(domain_tag) ^ index) }
//
// The three inputs are the three axes randomness varies along in this engine:
// WHICH WORLD (world_seed -- itself derived from a scene seed by the caller),
// WHICH SYSTEM (domain_tag -- "dryden", "sensor.noise", "spawn",
// ...), and WHICH INSTANCE within that system (index -- body slot, sensor
// slot, rollout, ...).
//
// Why domain separation is mandatory rather than tidy:
// XOR-ing an index into a raw seed makes index 0 collide with the untagged
// stream, so a naive index XOR hands instance 0 the same stream the world would
// take -- correlated wind and sensor noise on precisely the vehicle being
// watched, reproducible enough to be mistaken for physics. Tags are what
// break that, and fnv1a64("") being the non-zero offset basis means even an
// empty tag is separated from the bare seed.
//
// Streams are derived, never stored-and-shared: two call sites that want the
// same sequence pass the same three inputs. Adding a fourth body therefore
// does not perturb the first three, and adding a new SYSTEM (a new tag)
// perturbs nothing at all -- the property that makes the determinism-replay
// corpus survive engine growth.
//
// The returned Stream starts with an empty Box-Muller cache, so a freshly
// derived stream and a snapshot-restored one at draw 0 are byte-identical.
// ---------------------------------------------------------------------------
constexpr Stream make_stream(uint64_t world_seed, std::string_view domain_tag, uint64_t index) noexcept {
    return Stream{splitmix64(world_seed ^ fnv1a64(domain_tag) ^ index), 0.0f, 0u};
}

}  // namespace spade::rng
