// ---------------------------------------------------------------------------
// test_gpu_fp32_math.cpp -- THE BIT-EXACTNESS BATTERIES for the Slang port of
// core/fp32_math.cpp. The standard here is BIT-IDENTITY, not a tolerance: every
// argument is evaluated on the device and on the host and the two answers are
// compared AS UINT32 BIT PATTERNS. Zero mismatches, or the test fails and
// prints the argument and both patterns.
//
// WHY BITS AND NOT ULPS. tests/test_fp32_math.cpp already proves ACCURACY (<= 1
// ulp against a double reference) and it proves it for the host. Repeating that
// measurement on the GPU would answer a question nobody asked. The question this
// program actually has is whether the GPU and the CPU twin produce THE SAME
// TRAJECTORY, and the only measurement that answers it is equality of bits. A
// 1-ulp GPU/CPU disagreement is a divergence; calling it "within tolerance"
// would be exactly the mistake the whole fp32_math exercise exists to avoid.
//
// WHAT IS COVERED, and why these sets and not others (the task brief's list,
// made concrete):
//
//   (a) THE COMMITTED DECIDING WINDOWS of tests/test_fp32_math.cpp, re-evaluated
//       on the device. That file chose its windows because they DECIDE the
//       accuracy bound -- log32's whole next_float domain, exp32's reduction
//       boundary, exp32's largest quotients and subnormal tail, and a strided
//       pass over everything else. A window that decides the host's bound is
//       exactly a window where a port's op-order slip would show, so the same
//       windows decide the parity claim.  [six suites below]
//
//   (b) ALL 2^24 BOX-MULLER-REACHABLE ANGLES through sin32 and cos32. Not a
//       sample: the set of angles rng.hpp's next_gauss can ever present is
//       finite and is enumerated in full, so the result is a fact rather than
//       an estimate.  [two suites]
//
//   (c) exp32 over the CORPUS-LIVE ARGUMENTS -- the step ratios the five
//       committed scenarios actually produce, derived here from
//       dryden_step_ratio() rather than transcribed -- plus a deterministic
//       1M-point lattice spanning [-104, 104], the whole non-saturating
//       domain.  [one suite]
//
//   (d) THE EDGE CASES: +-infinity, NaN, +-0, the subnormal-scaling boundary on
//       both sides (log32's 2^24 pre-scale and exp32's subnormal-result tail),
//       and the kMaxReducibleAngle pins.  [one suite]
//
//   plus the two things the batteries above depend on and could not diagnose on
//   their own: the CONSTANTS the device actually compiled
//   (KernelConstantsAreBitIdenticalToTheHostConstants) and the correctly-rounded
//   DIVISION that replaces log32's one OpFDiv
//   (CorrectlyRoundedDivisionMatchesTheHostDivide).
//
// EVERY SUITE IS Gpu*-PREFIXED (tests/AppendSpadeLabels.cmake gives those the
// "gpu" ctest label) and every one begins with the mandated
// `if (!vulkan_available()) GTEST_SKIP()` -- a runner without a device skips,
// never fails and never silently passes-without-running.
//
// THE COUNTS ARE PRINTED BY THE TESTS THEMSELVES. Each battery emits one
// `[GpuFp32Math] <name>: n=... mismatches=... chunks=...` line, because a
// report that says "zero mismatches" without saying over how many arguments has
// not said anything.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"
#include "compute/vulkan/probe_runner.hpp"
#include "core/fp32_math.hpp"
#include "core/rng.hpp"
#include "world/medium.hpp"

#include "fp32_math_probe.spv.gen.hpp"

namespace {

using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::run_probe;
using spade::compute::VulkanContext;
using spade::compute::vulkan_available;

// The probe kernel's argument-buffer protocol (see
// engine/shaders/kernels/fp32_math_probe.slang): args[0] is the mode, bit-cast
// from a uint, and args[1..] are the arguments proper.
constexpr uint32_t kModeLog = 0;
constexpr uint32_t kModeExp = 1;
constexpr uint32_t kModeSin = 2;
constexpr uint32_t kModeCos = 3;
constexpr uint32_t kModeDiv = 4;
constexpr uint32_t kModeConstants = 5;
// S6 Task 8 review (M9): u64.slang's shift, one output WORD per thread --
// see engine/shaders/kernels/fp32_math_probe.slang's mode-6 note for the
// three-arguments-per-two-results layout and for why an integer kernel rides
// the fp32_math probe.
constexpr uint32_t kModeU64Shr = 6;

// Results per dispatch. 2^21 floats is an 8 MB argument buffer and an 8 MB
// result buffer -- comfortably inside the global constraint's 64 MB-per-dispatch
// ceiling on a device with ~1 GB of shared memory, and 2^21 / 64 = 32,768
// workgroups, inside the 65,535 Vulkan guarantees on every implementation. Two
// lanes' correctness tests may share the device concurrently; a chunk this size
// finishes in single-digit milliseconds, so neither starves the other.
constexpr uint32_t kChunkElements = 1u << 21;

// How many mismatching arguments are printed before the rest are only counted.
// A divergence is either a handful of arguments (an edge case, a constant) or
// millions (an op-order slip); sixteen distinguishes the two and neither floods
// the log nor hides the shape.
constexpr int kMaxPrintedMismatches = 16;

[[nodiscard]] float float_of(uint32_t u) {
    float x = 0.0f;
    std::memcpy(&x, &u, sizeof(x));
    return x;
}

[[nodiscard]] uint32_t bits_of(float x) {
    uint32_t u = 0;
    std::memcpy(&u, &x, sizeof(u));
    return u;
}

// A decorrelating integer hash (the well-known `lowbias32` finalizer), used to
// spread a loop counter over a mantissa field. Deliberately NOT `i * someOddK`:
// bit k of a multiply depends only on bits <= k of its inputs, so the low 23
// bits of such a product march almost in lockstep with i and the high mantissa
// bits barely move. The shift/multiply/shift rounds below mix high bits down.
// Deterministic and reproducible -- the point is spread, not randomness.
[[nodiscard]] uint32_t mix32(uint64_t v) {
    uint32_t x = static_cast<uint32_t>(v) ^ static_cast<uint32_t>(v >> 32);
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

[[nodiscard]] std::span<const std::byte> probe_spirv() {
    return std::as_bytes(std::span<const uint32_t>(spade::compute::gen::kSpv_fp32_math_probe,
                                                   spade::compute::gen::kSpvWordCount_fp32_math_probe));
}

// A device context per test. Creating one costs a few milliseconds and buys
// total isolation between suites -- no state survives from one battery to the
// next, so a failure is never explained by "the test before it".
[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    BackendDesc desc{.kind = BackendKind::vulkan};
    auto result = VulkanContext::create(desc);
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

struct Tally {
    uint64_t n = 0;
    uint64_t mismatches = 0;
    uint32_t chunks = 0;
    bool dispatch_failed = false;
};

void report(const char* name, const Tally& tally) {
    std::printf("[GpuFp32Math] %s: n=%" PRIu64 "  mismatches=%" PRIu64 "  chunks=%u\n", name,
                tally.n, tally.mismatches, tally.chunks);
    std::fflush(stdout);
}

// The mismatch diagnostic. Deliberately verbose and deliberately identical
// everywhere: an argument, its bit pattern, and BOTH answers as bits and as
// values. That is what a future divergence needs -- "the GPU differs" is not
// actionable, "at x=0x3F317200 the GPU said 0x3F800001 and the host said
// 0x3F800000" is.
void print_mismatch(const char* name, uint64_t index, float x, uint32_t gpu_bits,
                    uint32_t host_bits) {
    std::printf(
        "[GpuFp32Math] MISMATCH %s[%" PRIu64 "]  arg=0x%08" PRIX32 " (%.9g)  gpu=0x%08" PRIX32
        " (%.9g)  host=0x%08" PRIX32 " (%.9g)\n",
        name, index, bits_of(x), static_cast<double>(x), gpu_bits,
        static_cast<double>(float_of(gpu_bits)), host_bits,
        static_cast<double>(float_of(host_bits)));
}

// ---------------------------------------------------------------------------
// The sweep driver. Walks `count` arguments in chunks, dispatching each chunk
// and comparing it before generating the next, so peak host memory stays at two
// chunk buffers regardless of how many arguments the battery enumerates.
//
// `arg_at(i)` yields the i-th argument; `host_fn(x)` is the CPU kernel. Both
// are templates rather than std::function so the argument generation inlines --
// which matters in the Debug preset, where these suites still have to finish
// inside the 60 s per-test CTest timeout.
// ---------------------------------------------------------------------------
template <class ArgAt, class HostFn>
[[nodiscard]] Tally sweep(VulkanContext& ctx, const char* name, uint32_t mode, uint64_t count,
                          ArgAt arg_at, HostFn host_fn) {
    Tally tally;
    tally.n = count;

    std::vector<float> args(static_cast<std::size_t>(kChunkElements) + 1);
    std::vector<float> out(kChunkElements);
    args[0] = float_of(mode);

    int printed = 0;
    for (uint64_t base = 0; base < count; base += kChunkElements) {
        const uint32_t n =
            static_cast<uint32_t>(std::min<uint64_t>(kChunkElements, count - base));
        for (uint32_t j = 0; j < n; ++j) args[j + 1] = arg_at(base + j);

        const auto result = run_probe(ctx, probe_spirv(),
                                      std::span<const float>(args.data(), std::size_t{n} + 1),
                                      std::span<float>(out.data(), n));
        if (!result.has_value()) {
            ADD_FAILURE() << name << ": run_probe failed at chunk " << tally.chunks << ": "
                          << result.error().context;
            tally.dispatch_failed = true;
            return tally;
        }
        ++tally.chunks;

        for (uint32_t j = 0; j < n; ++j) {
            const float x = args[j + 1];
            const uint32_t gpu = bits_of(out[j]);
            const uint32_t host = bits_of(host_fn(x));
            if (gpu != host) {
                ++tally.mismatches;
                if (printed < kMaxPrintedMismatches) {
                    print_mismatch(name, base + j, x, gpu, host);
                    ++printed;
                }
            }
        }
    }
    return tally;
}

// Same driver for the two-argument division probe: arguments come in pairs, so
// a chunk of n results carries 2n argument floats.
template <class PairAt>
[[nodiscard]] Tally sweep_div(VulkanContext& ctx, const char* name, uint64_t count, PairAt pair_at) {
    Tally tally;
    tally.n = count;

    // Half as many results per dispatch as a unary battery, so the ARGUMENT
    // buffer stays the same 8 MB.
    constexpr uint32_t kDivChunk = kChunkElements / 2;

    std::vector<float> args(static_cast<std::size_t>(kDivChunk) * 2 + 1);
    std::vector<float> out(kDivChunk);
    args[0] = float_of(kModeDiv);

    int printed = 0;
    for (uint64_t base = 0; base < count; base += kDivChunk) {
        const uint32_t n = static_cast<uint32_t>(std::min<uint64_t>(kDivChunk, count - base));
        for (uint32_t j = 0; j < n; ++j) {
            const std::pair<float, float> pair = pair_at(base + j);
            args[1 + 2 * std::size_t{j}] = pair.first;
            args[2 + 2 * std::size_t{j}] = pair.second;
        }

        const auto result =
            run_probe(ctx, probe_spirv(),
                      std::span<const float>(args.data(), 2 * std::size_t{n} + 1),
                      std::span<float>(out.data(), n));
        if (!result.has_value()) {
            ADD_FAILURE() << name << ": run_probe failed at chunk " << tally.chunks << ": "
                          << result.error().context;
            tally.dispatch_failed = true;
            return tally;
        }
        ++tally.chunks;

        for (uint32_t j = 0; j < n; ++j) {
            const float a = args[1 + 2 * std::size_t{j}];
            const float b = args[2 + 2 * std::size_t{j}];
            const uint32_t gpu = bits_of(out[j]);
            const uint32_t host = bits_of(a / b);
            if (gpu != host) {
                ++tally.mismatches;
                if (printed < kMaxPrintedMismatches) {
                    std::printf("[GpuFp32Math] MISMATCH %s[%" PRIu64 "]  a=0x%08" PRIX32
                                " b=0x%08" PRIX32 "  gpu=0x%08" PRIX32 "  host=0x%08" PRIX32 "\n",
                                name, base + j, bits_of(a), bits_of(b), gpu, host);
                    ++printed;
                }
            }
        }
    }
    return tally;
}

// One small battery over an explicit argument list -- the edge cases, where the
// interesting thing is WHICH arguments were checked, not how many.
[[nodiscard]] Tally sweep_list(VulkanContext& ctx, const char* name, uint32_t mode,
                               const std::vector<float>& arguments, float (*host_fn)(float)) {
    return sweep(
        ctx, name, mode, arguments.size(),
        [&arguments](uint64_t i) { return arguments[static_cast<std::size_t>(i)]; },
        [host_fn](float x) { return host_fn(x); });
}

// The four kernels, so a list battery can name one.
float host_log32(float x) { return spade::math::log32(x); }
float host_exp32(float x) { return spade::math::exp32(x); }
float host_sin32(float x) { return spade::math::sin32(x); }
float host_cos32(float x) { return spade::math::cos32(x); }

}  // namespace

// ===========================================================================
// 0. The device, and the float-controls behaviour every battery below depends
//    on
// ===========================================================================

// PROVENANCE, and one real dependency. The batteries assert bit-identity; this
// prints WHICH DEVICE produced it (name, API version, driver) and what that
// device says about its own float controls -- specifically
// shaderDenormPreserveFloat32, because two lines of the port (log32's 2^24
// subnormal pre-scale and exp32's subnormal-result tail) would answer
// differently on a device that flushed fp32 denormals to zero.
//
// The property is PRINTED, not asserted, and the distinction is the point: what
// the batteries actually require is the BEHAVIOUR, which
// EdgeCasesAreBitIdenticalToTheHost measures directly at both subnormal
// boundaries. A device that advertised denorm-preserve and did not do it would
// pass an assertion here and fail there, which is the right way round.
TEST(GpuFp32Math, ReportsTheDeviceAndItsFloatControls) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr) << "VulkanContext::create failed on a device-present box";

    VkPhysicalDeviceFloatControlsProperties float_controls{};
    float_controls.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FLOAT_CONTROLS_PROPERTIES;

    VkPhysicalDeviceDriverProperties driver{};
    driver.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DRIVER_PROPERTIES;
    driver.pNext = &float_controls;

    VkPhysicalDeviceProperties2 properties{};
    properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    properties.pNext = &driver;
    vkGetPhysicalDeviceProperties2(ctx->physical_device(), &properties);

    std::printf("[GpuFp32Math] device=%s  api=%u.%u.%u  driver=%s (%s)  driverVersion=0x%08X\n",
                properties.properties.deviceName, VK_API_VERSION_MAJOR(ctx->api_version()),
                VK_API_VERSION_MINOR(ctx->api_version()), VK_API_VERSION_PATCH(ctx->api_version()),
                driver.driverName, driver.driverInfo, properties.properties.driverVersion);
    std::printf(
        "[GpuFp32Math] float controls: denormPreserveF32=%u denormFlushToZeroF32=%u "
        "roundingModeRTEF32=%u signedZeroInfNanPreserveF32=%u denormBehaviorIndependence=%u\n",
        static_cast<unsigned>(float_controls.shaderDenormPreserveFloat32),
        static_cast<unsigned>(float_controls.shaderDenormFlushToZeroFloat32),
        static_cast<unsigned>(float_controls.shaderRoundingModeRTEFloat32),
        static_cast<unsigned>(float_controls.shaderSignedZeroInfNanPreserveFloat32),
        static_cast<unsigned>(float_controls.denormBehaviorIndependence));
    std::printf("[GpuFp32Math] maxComputeWorkGroupCount[0]=%u  probe chunk=%u results (%u groups)\n",
                properties.properties.limits.maxComputeWorkGroupCount[0], kChunkElements,
                kChunkElements / spade::compute::kProbeWorkgroupSize);
    std::fflush(stdout);

    EXPECT_GT(properties.properties.limits.maxComputeWorkGroupCount[0],
              kChunkElements / spade::compute::kProbeWorkgroupSize)
        << "the chunk size this file dispatches exceeds the device's workgroup-count limit";

    // The probe pipeline itself runs: one argument in, one answer out. If this
    // fails, every battery below fails for the same reason and none of their
    // counts mean anything.
    const std::vector<float> args = {float_of(kModeExp), 0.0f};
    std::vector<float> out(1);
    const auto result = run_probe(*ctx, probe_spirv(), args, out);
    ASSERT_TRUE(result.has_value()) << result.error().context;
    EXPECT_EQ(bits_of(out[0]), bits_of(spade::math::exp32(0.0f)));
}

// ===========================================================================
// 1. The constants the device actually compiled
// ===========================================================================

// engine/shaders/fp32_math.slang spells its constants exactly as
// core/fp32_math.cpp spells them -- exact rationals for the coefficients, hex
// float literals for the irrational splits -- which is only correct if slangc
// folds `1.0f / 5040.0f` to the same bits MSVC does. That is a MEASUREMENT, not
// a language guarantee, so it is measured, here, per build, on the device.
//
// The right-hand sides below are TRANSCRIBED from fp32_math.cpp rather than
// exported from it, for the reason test_fp32_math.cpp gives for its own
// Cody-Waite transcription: a test that read the same constant object the
// implementation uses would prove nothing about the constant.
TEST(GpuFp32Math, KernelConstantsAreBitIdenticalToTheHostConstants) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    // fp32_math.slang's fp32_math_constant() roster, in its pinned order.
    // APPEND ONLY over there; this list is the other half of that contract.
    const struct {
        const char* name;
        float value;
    } kExpected[] = {
        {"kLn2Hi", 0x1.62e4p-1f},      {"kLn2Lo", 0x1.7f7d1cp-20f},
        {"kSqrt2", 0x1.6a09e6p+0f},    {"kInvLn2", 0x1.715476p+0f},
        {"kE2", 1.0f / 2.0f},          {"kE3", 1.0f / 6.0f},
        {"kE4", 1.0f / 24.0f},         {"kE5", 1.0f / 120.0f},
        {"kE6", 1.0f / 720.0f},        {"kE7", 1.0f / 5040.0f},
        {"kE8", 1.0f / 40320.0f},      {"kTwoOverPi", 0x1.45f306p-1f},
        {"kPio2A", 0x1.921f8p+0f},     {"kPio2B", 0x1.aa22p-19f},
        {"kPio2C", 0x1.68cp-39f},      {"kS1", -1.0f / 6.0f},
        {"kS2", 1.0f / 120.0f},        {"kS3", -1.0f / 5040.0f},
        {"kS4", 1.0f / 362880.0f},     {"kS5", -1.0f / 39916800.0f},
        {"kC1", -1.0f / 2.0f},         {"kC2", 1.0f / 24.0f},
        {"kC3", -1.0f / 720.0f},       {"kC4", 1.0f / 40320.0f},
        {"kC5", -1.0f / 3628800.0f},   {"R 2/3", 2.0f / 3.0f},
        {"R 2/5", 2.0f / 5.0f},        {"R 2/7", 2.0f / 7.0f},
        {"R 2/9", 2.0f / 9.0f},        {"kMaxReducibleAngle", spade::math::kMaxReducibleAngle},
        {"log32 pre-scale 2^24", 0x1.0p24f}, {"exp32 post-scale 2^-64", 0x1.0p-64f},
    };
    constexpr std::size_t kCount = std::size(kExpected);

    const std::vector<float> args = {float_of(kModeConstants)};
    std::vector<float> out(kCount);
    const auto result = run_probe(*ctx, probe_spirv(), args, out);
    ASSERT_TRUE(result.has_value()) << result.error().context;

    uint64_t mismatches = 0;
    for (std::size_t i = 0; i < kCount; ++i) {
        const uint32_t gpu = bits_of(out[i]);
        const uint32_t host = bits_of(kExpected[i].value);
        if (gpu != host) {
            ++mismatches;
            std::printf("[GpuFp32Math] MISMATCH constant[%zu] %s: gpu=0x%08" PRIX32
                        " host=0x%08" PRIX32 "\n",
                        i, kExpected[i].name, gpu, host);
        }
    }
    std::printf("[GpuFp32Math] KernelConstants: n=%zu  mismatches=%" PRIu64 "\n", kCount,
                mismatches);
    std::fflush(stdout);
    EXPECT_EQ(mismatches, 0u);
}

// ===========================================================================
// 2. The correctly-rounded division that replaces log32's one OpFDiv
// ===========================================================================

// engine/shaders/fp32_math.slang's division audit explains why log32's
// `f / (2.0f + f)` could not be respelled as a multiply by a constant (the
// divisor is a runtime value) and is implemented as an exact integer long
// division instead. This is the direct proof of that implementation against the
// host's own `/`, over two argument sets:
//
//   * EVERY (f, 2 + f) PAIR log32 CAN FORM. f = mm - 1 where mm is the folded
//     mantissa, so the complete set of f values is the 2^23 floats in
//     [sqrt(2)/2, sqrt(2)) minus one, mapped through the same two lines log32
//     uses. This is not a sample of the call site -- it is the call site's
//     whole input space.
//   * A GENERAL LATTICE of unrelated normal operand pairs, so the routine is
//     shown to be a correctly-rounded divide rather than something that happens
//     to agree on log32's narrow domain.
TEST(GpuFp32Math, CorrectlyRoundedDivisionMatchesTheHostDivide) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    // ---- every mantissa log32's fold can produce -------------------------
    // The folded mantissa mm lies in [sqrt(2)/2, sqrt(2)); enumerate every
    // float in [0.5, sqrt(2)) whose fold log32 would keep, i.e. walk the
    // mantissa bit patterns of [1, 2) and apply the same halving rule.
    constexpr uint32_t kMantissaCount = 1u << 23;  // every float in [1, 2)
    const Tally call_site = sweep_div(
        *ctx, "Log32DivOverEveryCallSitePair", kMantissaCount, [](uint64_t i) {
            const float m = float_of(0x3F800000u + static_cast<uint32_t>(i));  // [1, 2)
            const float mm = m > 0x1.6a09e6p+0f ? m * 0.5f : m;                // log32's fold
            const float f = mm - 1.0f;
            return std::pair<float, float>{f, 2.0f + f};
        });
    report("Log32DivOverEveryCallSitePair", call_site);
    ASSERT_FALSE(call_site.dispatch_failed);
    EXPECT_EQ(call_site.mismatches, 0u);

    // ---- a general lattice of normal operand pairs -----------------------
    //
    // WHAT ACTUALLY RUNS, stated exactly, because an earlier version of this
    // battery claimed "every binade" while masking both exponents to a single
    // fixed one (a in [0.5, 1), b in [2, 4)) -- a one-dimensional sweep wearing
    // a two-dimensional comment. It is now genuinely two-dimensional:
    //
    //   * BOTH EXPONENTS VARY, INDEPENDENTLY, over the 121 biased exponent
    //     fields [67, 187] -- a +-120 binade spread, i.e. operands from ~1e-18
    //     to ~1e+18. `i % 121` and `(i / 121) % 121` are a genuine 2-D lattice:
    //     over 2^20 samples every one of the 121 x 121 = 14,641 (ea, eb)
    //     combinations is visited ~71 times. (Two coprime strides mod 121 would
    //     NOT have done this -- both walks would share period 121 and only 121
    //     of the 14,641 combinations would ever occur, which is the same
    //     correlation bug in different clothes.)
    //   * BOTH MANTISSAS VARY over the whole 23-bit field, from two decorrelated
    //     integer hashes (the lowbias32 finalizer, offset for the second), not
    //     from low bits of a multiply -- whose bit k depends only on bits <= k
    //     and would leave the high mantissa bits nearly constant.
    //   * BOTH SIGNS on both operands.
    //
    // THE EXPONENT RANGE IS A DOMAIN OBLIGATION, not a convenience: log32_div's
    // precondition requires a NORMAL quotient. ea - eb lands in [-120, 120], so
    // the result's biased exponent is ea - eb + 126 or + 127, i.e. [6, 247]
    // (248 after a rounding carry) -- never 0 and never 255, so nothing in this
    // lattice underflows to subnormal or overflows to infinity.
    constexpr uint64_t kGeneralPairs = 1u << 20;
    constexpr uint32_t kExpLow = 67;    // 2^-60
    constexpr uint32_t kExpSpan = 121;  // through 187, i.e. 2^+60
    const Tally general = sweep_div(
        *ctx, "Log32DivOverAGeneralNormalLattice", kGeneralPairs, [](uint64_t i) {
            const uint32_t ea = kExpLow + static_cast<uint32_t>(i % kExpSpan);
            const uint32_t eb = kExpLow + static_cast<uint32_t>((i / kExpSpan) % kExpSpan);
            const uint32_t ma = mix32(i) & 0x007FFFFFu;
            const uint32_t mb = mix32(i + 0x9E3779B9ull) & 0x007FFFFFu;
            const float a = float_of((ea << 23) | ma);
            const float b = float_of((eb << 23) | mb);
            return std::pair<float, float>{(i & 1u) ? -a : a, (i & 2u) ? -b : b};
        });
    report("Log32DivOverAGeneralNormalLattice", general);
    ASSERT_FALSE(general.dispatch_failed);
    EXPECT_EQ(general.mismatches, 0u);
}

// ===========================================================================
// 3. Battery (a) -- the committed deciding windows of test_fp32_math.cpp
// ===========================================================================

// test_fp32_math.cpp's Fp32Log.IsWithinOneUlpAcrossEveryValueNextFloatCanProduce,
// re-evaluated on the device. next_float() returns k * 2^-24 for k in
// [0, 2^24) and next_gauss clamps k == 0 up to 1, so this is the COMPLETE set
// of arguments log32 can ever see inside the engine's gaussian -- 16,777,215
// of them, every one of them, no sampling.
TEST(GpuFp32Math, Log32MatchesTheHostBitForBitOverEveryNextFloatArgument) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr uint64_t kCount = (1u << 24) - 1;
    const Tally tally = sweep(
        *ctx, "Log32EveryNextFloatArgument", kModeLog, kCount,
        [](uint64_t i) {
            return static_cast<float>(static_cast<uint32_t>(i) + 1u) * spade::rng::kUniformQuantum;
        },
        host_log32);
    report("Log32EveryNextFloatArgument", tally);
    ASSERT_FALSE(tally.dispatch_failed);
    EXPECT_EQ(tally.mismatches, 0u);
}

// test_fp32_math.cpp's Fp32Log.IsWithinOneUlpAcrossTheWholeNormalRange: every
// 4096th representable positive normal float, which covers every binade and
// every mantissa region. log32 is a general routine and the port has to be one
// too.
TEST(GpuFp32Math, Log32MatchesTheHostBitForBitAcrossTheWholeNormalRange) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr uint32_t kFirst = 0x00800000u;
    constexpr uint32_t kLast = 0x7F800000u;
    constexpr uint32_t kStride = 4096u;
    constexpr uint64_t kCount = (kLast - kFirst) / kStride;
    const Tally tally = sweep(
        *ctx, "Log32WholeNormalRange", kModeLog, kCount,
        [](uint64_t i) { return float_of(kFirst + static_cast<uint32_t>(i) * kStride); },
        host_log32);
    report("Log32WholeNormalRange", tally);
    ASSERT_FALSE(tally.dispatch_failed);
    EXPECT_EQ(tally.mismatches, 0u);

    // The powers of two, where log32's exact-product reconstruction does all
    // the work (test_fp32_math.cpp's MatchesTheLibraryOnExactPowersOfTwo).
    std::vector<float> powers;
    for (int n = -126; n <= 127; ++n) powers.push_back(std::ldexp(1.0f, n));
    const Tally pow_tally = sweep_list(*ctx, "Log32ExactPowersOfTwo", kModeLog, powers, host_log32);
    report("Log32ExactPowersOfTwo", pow_tally);
    EXPECT_EQ(pow_tally.mismatches, 0u);
}

// test_fp32_math.cpp's
// Fp32Exp.IsWithinOneUlpAcrossTheReductionBoundaryAtFullFloatDensity: every
// float in [0.25, 0.5) and its negation. |r| reaches its maximum ln(2)/2 inside
// this window and the quotient n flips between 0 and +-1 at exactly that point,
// so both the polynomial's worst case and the reduction's discontinuity are
// enumerated here.
TEST(GpuFp32Math, Exp32MatchesTheHostBitForBitAcrossTheReductionBoundary) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr uint32_t kFirst = 0x3E800000u;  // 0.25
    constexpr uint32_t kLast = 0x3F000000u;   // 0.5 (exclusive)
    constexpr uint64_t kCount = 2ull * (kLast - kFirst);
    const Tally tally = sweep(
        *ctx, "Exp32ReductionBoundary", kModeExp, kCount,
        [](uint64_t i) {
            const float x = float_of(kFirst + static_cast<uint32_t>(i >> 1));
            return (i & 1u) ? -x : x;
        },
        host_exp32);
    report("Exp32ReductionBoundary", tally);
    ASSERT_FALSE(tally.dispatch_failed);
    EXPECT_EQ(tally.mismatches, 0u);
}

// test_fp32_math.cpp's
// Fp32Exp.IsWithinOneUlpAcrossTheLargestQuotientsAndTheSubnormalTail: every
// float in +-[64, 104]. |n| runs to 150 here, which is where n*kLn2Lo -- the
// reduction's only rounding -- is largest; the negative half is the entire
// subnormal-result region and both saturation boundaries.
TEST(GpuFp32Math, Exp32MatchesTheHostBitForBitOverTheLargestQuotientsAndTheSubnormalTail) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr uint32_t kFirst = 0x42800000u;  // 64
    constexpr uint32_t kLast = 0x42D00000u;   // 104 (inclusive)
    constexpr uint64_t kCount = 2ull * (kLast - kFirst + 1);
    const Tally tally = sweep(
        *ctx, "Exp32LargestQuotientsAndSubnormalTail", kModeExp, kCount,
        [](uint64_t i) {
            const float x = float_of(kFirst + static_cast<uint32_t>(i >> 1));
            return (i & 1u) ? -x : x;
        },
        host_exp32);
    report("Exp32LargestQuotientsAndSubnormalTail", tally);
    ASSERT_FALSE(tally.dispatch_failed);
    EXPECT_EQ(tally.mismatches, 0u);
}

// test_fp32_math.cpp's Fp32Exp.IsWithinOneUlpAndTotalAcrossTheWholeFiniteDomain
// (the strided pass over everything else) plus its
// SaturatesOnExactlyTheSameFloatTheCorrectlyRoundedExponentialDoes windows --
// 4096 floats either side of each saturation boundary, at stride 1, which is
// where a guard placed an ulp out of position on one side and not the other
// would show up as a bit divergence rather than as an accuracy one.
TEST(GpuFp32Math, Exp32MatchesTheHostBitForBitAcrossTheWholeFiniteDomainAndBothSaturationEdges) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr uint32_t kLast = 0x42D00000u;
    constexpr uint32_t kStride = 256u;
    constexpr uint64_t kStrided = 2ull * (kLast / kStride + 1);
    const Tally strided = sweep(
        *ctx, "Exp32WholeFiniteDomainStrided", kModeExp, kStrided,
        [](uint64_t i) {
            const float x = float_of(static_cast<uint32_t>(i >> 1) * kStride);
            return (i & 1u) ? -x : x;
        },
        host_exp32);
    report("Exp32WholeFiniteDomainStrided", strided);
    ASSERT_FALSE(strided.dispatch_failed);
    EXPECT_EQ(strided.mismatches, 0u);

    // Overflow boundary: last finite result at 88.7228317, first +infinity at
    // 88.7228394.
    constexpr uint32_t kOverflowCentre = 0x42B17218u;
    constexpr uint64_t kWindow = 2u * 4096u + 1u;
    const Tally overflow = sweep(
        *ctx, "Exp32OverflowBoundary", kModeExp, kWindow,
        [](uint64_t i) { return float_of(kOverflowCentre - 4096u + static_cast<uint32_t>(i)); },
        host_exp32);
    report("Exp32OverflowBoundary", overflow);
    EXPECT_EQ(overflow.mismatches, 0u);

    // Underflow boundary: last non-zero result at -103.972076, first +0 at
    // -103.972084.
    constexpr uint32_t kUnderflowCentre = 0x42CFF1B5u;
    const Tally underflow = sweep(
        *ctx, "Exp32UnderflowBoundary", kModeExp, kWindow,
        [](uint64_t i) { return -float_of(kUnderflowCentre - 4096u + static_cast<uint32_t>(i)); },
        host_exp32);
    report("Exp32UnderflowBoundary", underflow);
    EXPECT_EQ(underflow.mismatches, 0u);
}

// ===========================================================================
// 4. Battery (b) -- every Box-Muller-reachable angle
// ===========================================================================
//
// theta = kTwoPi * u2 with u2 = k * 2^-24, so the complete set of angles
// rng.hpp's gaussian can present is these 2^24 products. Enumerated in full,
// exactly as test_fp32_math.cpp's
// Fp32SinCos.AreWithinOneUlpAcrossEveryBoxMullerAngle does on the host.
//
// The ANGLES ARE COMPUTED ON THE HOST and uploaded, deliberately: what is under
// test is sin32/cos32, not the argument arithmetic that feeds them. A port that
// recomputed `kTwoPi * (k * kUniformQuantum)` on the device would be testing
// two things at once and could not say which one had moved.
//
// Split into two suites, one per function, so each stays well inside the 60 s
// per-test CTest timeout in the Debug preset as well as Release.

namespace {
[[nodiscard]] float box_muller_angle(uint64_t k) {
    const float u2 = static_cast<float>(static_cast<uint32_t>(k)) * spade::rng::kUniformQuantum;
    return spade::rng::kTwoPi * u2;
}
}  // namespace

TEST(GpuFp32Math, Sin32MatchesTheHostBitForBitOverEveryBoxMullerAngle) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    const Tally tally =
        sweep(*ctx, "Sin32EveryBoxMullerAngle", kModeSin, 1ull << 24, box_muller_angle, host_sin32);
    report("Sin32EveryBoxMullerAngle", tally);
    ASSERT_FALSE(tally.dispatch_failed);
    EXPECT_EQ(tally.mismatches, 0u);
}

TEST(GpuFp32Math, Cos32MatchesTheHostBitForBitOverEveryBoxMullerAngle) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    const Tally tally =
        sweep(*ctx, "Cos32EveryBoxMullerAngle", kModeCos, 1ull << 24, box_muller_angle, host_cos32);
    report("Cos32EveryBoxMullerAngle", tally);
    ASSERT_FALSE(tally.dispatch_failed);
    EXPECT_EQ(tally.mismatches, 0u);
}

// The documented accuracy domain is wider than the Box-Muller interval, and
// world/sdf.cpp's heightfield feeds these functions unbounded arguments, so the
// port is also swept across |x| <= kMaxAccurateAngle on both signs and then out
// through the pinned region past kMaxReducibleAngle -- the two regimes with
// different rules (computed vs pinned), both of which the port must land on the
// same float as the host.
TEST(GpuFp32Math, SinCos32MatchTheHostBitForBitAcrossTheAccuracyDomainAndThePinnedRegion) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    // |x| <= 100, the documented accuracy domain, at the same 1,000,001-point
    // resolution test_fp32_math.cpp uses.
    constexpr int kSteps = 500'000;
    constexpr uint64_t kDomainCount = 2ull * kSteps + 1;
    const auto domain_arg = [](uint64_t i) {
        const int step = static_cast<int>(i) - kSteps;
        return spade::math::kMaxAccurateAngle * (static_cast<float>(step) / kSteps);
    };
    const Tally sin_domain =
        sweep(*ctx, "Sin32AccuracyDomain", kModeSin, kDomainCount, domain_arg, host_sin32);
    report("Sin32AccuracyDomain", sin_domain);
    EXPECT_EQ(sin_domain.mismatches, 0u);
    const Tally cos_domain =
        sweep(*ctx, "Cos32AccuracyDomain", kModeCos, kDomainCount, domain_arg, host_cos32);
    report("Cos32AccuracyDomain", cos_domain);
    EXPECT_EQ(cos_domain.mismatches, 0u);

    // The huge-argument regime: a stride over the whole float range above the
    // accuracy domain, both signs, up to and well past the boundedness breach
    // and the former UB point. The guard's pinned answers (0 and 1) and the
    // computed answers below it must agree bit for bit on both sides.
    constexpr uint32_t kHugeFirst = 0x42C80000u;  // 100.0f == kMaxAccurateAngle
    constexpr uint32_t kHugeLast = 0x7F7FFFFFu;   // FLT_MAX
    constexpr uint32_t kHugeStride = 9973u;       // prime; ~854k samples per sign
    constexpr uint64_t kHugeCount = 2ull * ((kHugeLast - kHugeFirst) / kHugeStride + 1);
    const auto huge_arg = [](uint64_t i) {
        const float x = float_of(kHugeFirst + static_cast<uint32_t>(i >> 1) * kHugeStride);
        return (i & 1u) ? -x : x;
    };
    const Tally sin_huge = sweep(*ctx, "Sin32HugeArguments", kModeSin, kHugeCount, huge_arg, host_sin32);
    report("Sin32HugeArguments", sin_huge);
    EXPECT_EQ(sin_huge.mismatches, 0u);
    const Tally cos_huge = sweep(*ctx, "Cos32HugeArguments", kModeCos, kHugeCount, huge_arg, host_cos32);
    report("Cos32HugeArguments", cos_huge);
    EXPECT_EQ(cos_huge.mismatches, 0u);
}

// ===========================================================================
// 5. Battery (c) -- exp32 over the corpus-live arguments and a 1M lattice
// ===========================================================================

// THE CORPUS-LIVE ARGUMENTS ARE DERIVED, NOT TRANSCRIBED. Every committed
// scenario steps at dt_ns / substeps == 1 ms (ballistic 5 ms / 5, bounce
// 1 ms / 1, quad_hover, shower and two_world_isolation 2 ms / 2), and
// DrydenParams' defaults give V = 5 m/s with L_u = L_v = 200 m and L_w = 50 m.
// dryden_step_ratio() turns those into the step ratios the filter actually
// runs at, and world/medium.cpp evaluates exp32 at -theta (both coefficient
// functions) and at -2*theta (the closed forms of Q00/Q11). Deriving them here
// means a corpus change moves this test's arguments with it instead of leaving
// a stale literal behind.
//
// The rotor lag's exp32 site is deliberately NOT represented: no committed
// world sets a rotor `tau`, so `tau <= 0` disables the lag and that call site
// is not corpus-live today. Stated rather than silently omitted.
TEST(GpuFp32Math, Exp32MatchesTheHostBitForBitOverTheCorpusLiveArgumentsAndADeterministicLattice) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    const spade::DrydenParams params{};
    constexpr float kSubstepSeconds = 0.001f;  // every committed scenario

    std::vector<float> corpus;
    for (const float scale : {params.scale_u, params.scale_v, params.scale_w}) {
        const float theta =
            spade::dryden_step_ratio(kSubstepSeconds, params.reference_airspeed, scale);
        std::printf("[GpuFp32Math] corpus theta (L=%.1f m, V=%.1f m/s, h=%g s) = %.9g\n",
                    static_cast<double>(scale), static_cast<double>(params.reference_airspeed),
                    static_cast<double>(kSubstepSeconds), static_cast<double>(theta));
        corpus.push_back(-theta);          // medium.cpp:218, :228
        corpus.push_back(-2.0f * theta);   // medium.cpp:99, :119
    }
    std::fflush(stdout);

    const Tally corpus_tally = sweep_list(*ctx, "Exp32CorpusLive", kModeExp, corpus, host_exp32);
    report("Exp32CorpusLive", corpus_tally);
    ASSERT_FALSE(corpus_tally.dispatch_failed);
    EXPECT_EQ(corpus_tally.mismatches, 0u);

    // A deterministic 1,000,001-point lattice spanning the whole
    // non-saturating domain. Computed in double and rounded once to float, so
    // the argument set is a property of the arithmetic rather than of an
    // accumulation order -- and it is reproducible by anyone reading this line.
    constexpr uint64_t kLatticePoints = 1'000'001;
    const Tally lattice = sweep(
        *ctx, "Exp32DeterministicLattice", kModeExp, kLatticePoints,
        [](uint64_t i) {
            return static_cast<float>(-104.0 + 208.0 * (static_cast<double>(i) / 1'000'000.0));
        },
        host_exp32);
    report("Exp32DeterministicLattice", lattice);
    ASSERT_FALSE(lattice.dispatch_failed);
    EXPECT_EQ(lattice.mismatches, 0u);
}

// ===========================================================================
// 6. Battery (d) -- the edge cases
// ===========================================================================

// The arguments no sweep reaches by stride and every one of which is answered
// by a branch rather than by arithmetic: the non-finites, both zeros, the
// subnormal-scaling boundary on BOTH sides (log32 scales a subnormal INPUT up
// by 2^24; exp32 scales a subnormal RESULT down by 2^-64), and the
// kMaxReducibleAngle pins where sin32/cos32 stop computing and start returning
// phase zero.
//
// This suite is also the port's DENORM-BEHAVIOUR MEASUREMENT. If this device
// flushed fp32 denormals to zero, log32(denorm_min) would answer -infinity
// instead of -103.28 and exp32(-100) would answer +0 instead of a subnormal --
// so a green run here is the evidence that the two denormal-dependent lines of
// the port behave as the host's do, on this device, rather than an inference
// from the driver's advertised float-controls properties.
TEST(GpuFp32Math, EdgeCasesAreBitIdenticalToTheHost) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr float kInf = std::numeric_limits<float>::infinity();
    const float kNaN = std::numeric_limits<float>::quiet_NaN();
    constexpr float kDenormMin = std::numeric_limits<float>::denorm_min();
    constexpr float kMinNormal = (std::numeric_limits<float>::min)();
    const float kMaxReducible = spade::math::kMaxReducibleAngle;

    // Shared non-finite / zero set, handed to all four kernels.
    const std::vector<float> non_finite = {
        0.0f, -0.0f, kInf, -kInf, kNaN, -kNaN, 1.0f, -1.0f,
    };

    // ---- log32: subnormal inputs and the scaling boundary ----------------
    std::vector<float> log_args = non_finite;
    for (const float x : {kDenormMin,
                          float_of(0x00000002u),  // 2 * denorm_min
                          float_of(0x00400000u),  // mid-subnormal
                          float_of(0x007FFFFFu),  // the largest subnormal
                          kMinNormal,             // the first normal -- the boundary itself
                          float_of(0x00800001u),
                          kMinNormal * 0.5f,
                          0x1.6a09e6p+0f,  // kSqrt2, the fold's tipping point
                          std::nextafter(0x1.6a09e6p+0f, 0.0f),
                          std::nextafter(0x1.6a09e6p+0f, 4.0f),
                          (std::numeric_limits<float>::max)()}) {
        log_args.push_back(x);
    }
    const Tally log_tally = sweep_list(*ctx, "Log32EdgeCases", kModeLog, log_args, host_log32);
    report("Log32EdgeCases", log_tally);
    ASSERT_FALSE(log_tally.dispatch_failed);
    EXPECT_EQ(log_tally.mismatches, 0u);
    // The claim behind the count: a flush-to-zero device would answer
    // -infinity here, and this is what says it did not.
    EXPECT_TRUE(std::isfinite(spade::math::log32(kDenormMin)))
        << "host log32(denorm_min) must be finite for the GPU comparison above to mean anything";

    // ---- exp32: the saturation guards and the subnormal-result tail ------
    std::vector<float> exp_args = non_finite;
    for (const float x : {89.0f, -104.0f, 88.7228394f, 88.7228317f, -103.972084f, -103.972076f,
                          -87.3365479f, -88.0f, -95.0f, -100.0f, -103.0f, -103.9f, 104.0f, -1.0e6f,
                          1.0e6f, 1.0e30f, -1.0e30f, (std::numeric_limits<float>::max)(),
                          -(std::numeric_limits<float>::max)(), kDenormMin, -kDenormMin}) {
        exp_args.push_back(x);
    }
    const Tally exp_tally = sweep_list(*ctx, "Exp32EdgeCases", kModeExp, exp_args, host_exp32);
    report("Exp32EdgeCases", exp_tally);
    ASSERT_FALSE(exp_tally.dispatch_failed);
    EXPECT_EQ(exp_tally.mismatches, 0u);
    // The subnormal-result tail really is subnormal on the host, so the
    // comparison above really did exercise the 2^-64 rescale.
    EXPECT_GT(spade::math::exp32(-100.0f), 0.0f);
    EXPECT_LT(spade::math::exp32(-100.0f), kMinNormal);

    // ---- sin32 / cos32: the kMaxReducibleAngle pins ----------------------
    std::vector<float> trig_args = non_finite;
    for (const float x : {kMaxReducible, -kMaxReducible, std::nextafter(kMaxReducible, 0.0f),
                          -std::nextafter(kMaxReducible, 0.0f), std::nextafter(kMaxReducible, kInf),
                          2.0e7f, -2.0e7f, 3.373259e9f, -3.373259e9f, 3.5e9f, 1.0e30f, -1.0e30f,
                          (std::numeric_limits<float>::max)(), -(std::numeric_limits<float>::max)(),
                          spade::math::kMaxAccurateAngle, -spade::math::kMaxAccurateAngle,
                          6.2831853f, -6.2831853f, 1.5707963f, kDenormMin, -kDenormMin}) {
        trig_args.push_back(x);
    }
    const Tally sin_tally = sweep_list(*ctx, "Sin32EdgeCases", kModeSin, trig_args, host_sin32);
    report("Sin32EdgeCases", sin_tally);
    ASSERT_FALSE(sin_tally.dispatch_failed);
    EXPECT_EQ(sin_tally.mismatches, 0u);

    const Tally cos_tally = sweep_list(*ctx, "Cos32EdgeCases", kModeCos, trig_args, host_cos32);
    report("Cos32EdgeCases", cos_tally);
    ASSERT_FALSE(cos_tally.dispatch_failed);
    EXPECT_EQ(cos_tally.mismatches, 0u);

    // The pins themselves are the pinned values, not merely equal to each
    // other -- so a port that pinned the WRONG pair on both sides could not
    // pass this suite by agreeing with itself.
    EXPECT_EQ(spade::math::sin32(kMaxReducible), 0.0f);
    EXPECT_EQ(spade::math::cos32(kMaxReducible), 1.0f);
}


// ===========================================================================
// engine/shaders/u64.slang's SHIFT, AT EVERY DISTANCE (S6 Task 8 review, minor
// M9).
//
// WHY THIS EXISTS. u64.slang is the exact 64-bit integer arithmetic
// engine/shaders/rng.slang performs on a uint2 pair, written because this
// program's correctness device cannot give the primitive
// (VkPhysicalDeviceFeatures::shaderInt64 == VK_FALSE, Task 8's plan-mandated
// probe). Its claim is the same shape as log32's -- "this reproduces the host
// exactly" -- and so it deserves the same shape of evidence: a device sweep
// against host-computed expectations, compared as bit patterns.
//
// WHAT IS OTHERWISE UNCOVERED, which is the specific gap the review named.
// `u64_add` and `u64_mul` are exercised on every substep of every scenario
// (splitmix64 runs them per draw), and their correctness is transitively
// proven by every bit-exact `stream.state` row in tests/test_gpu_parity.cpp.
// `u64_shr` is NOT: splitmix64 asks for distances 30, 27 and 31, and
// rng_next_float asks for 40. So of the function's three branches, `n == 0` is
// never taken at all, the `n < 32` branch is taken at exactly three values, and
// the `n >= 32` branch at exactly one. This test takes all sixty-four.
//
// THE n == 0 BRANCH IS NOT DECORATION, and that is why it is worth a test
// rather than an argument: without it the natural spelling
// `(a.x >> n) | (a.y << (32 - n))` shifts a 32-bit value by 32 at n == 0,
// which is UNDEFINED in SPIR-V exactly as it is in C++ -- not "zero". A future
// caller reaching for a general shift would get whatever the driver happened to
// do.
//
// THE ARGUMENTS ARE ADVERSARIAL RATHER THAN ARBITRARY: all-ones (so every
// vacated bit position is visible as a zero), the two single-word patterns (so
// a branch that crossed the word boundary the wrong way is obvious), the two
// sign-bit-only patterns (the ones a signed shift would corrupt), a mixed
// pattern from the decorrelating hash, and splitmix64's own three constants --
// the values this helper actually carries in production.
// ===========================================================================

namespace {

// One (value, distance) case. Two output WORDS per case, so the probe dispatch
// is sized at 2 * cases (see fp32_math_probe.slang's mode 6 note).
struct ShiftCase {
    uint64_t value;
    uint32_t distance;
};

}  // namespace

TEST(GpuU64, ShiftMatchesTheHostAtEveryDistance) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const std::unique_ptr<VulkanContext> ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    const std::vector<uint64_t> values = {
        0xFFFFFFFFFFFFFFFFull,  // all ones -- every vacated bit is visible
        0x00000000FFFFFFFFull,  // low word only
        0xFFFFFFFF00000000ull,  // high word only
        0x8000000000000000ull,  // the 64-bit sign bit, which a signed shift would smear
        0x0000000080000000ull,  // the LOW word's top bit, i.e. the word boundary itself
        0x0123456789ABCDEFull,  // a mixed pattern with no repeating structure
        0x9E3779B97F4A7C15ull,  // kSplitmixGamma  -- the constants this helper
        0xBF58476D1CE4E5B9ull,  // kSplitmixMulA      actually carries in
        0x94D049BB133111EBull,  // kSplitmixMulB      production
        0x0000000000000001ull,  // the value every distance annihilates but n == 0
    };

    std::vector<ShiftCase> cases;
    cases.reserve(values.size() * 64u);
    for (const uint64_t v : values) {
        for (uint32_t n = 0; n < 64u; ++n) {
            cases.push_back(ShiftCase{v, n});
        }
    }

    // args[0] is the mode; then three words per case (lo, hi, n), bit-cast.
    std::vector<float> args;
    args.reserve(1u + 3u * cases.size());
    args.push_back(float_of(kModeU64Shr));
    for (const ShiftCase& c : cases) {
        args.push_back(float_of(static_cast<uint32_t>(c.value)));
        args.push_back(float_of(static_cast<uint32_t>(c.value >> 32)));
        args.push_back(float_of(c.distance));
    }

    std::vector<float> out(cases.size() * 2u, 0.0f);
    const auto result = run_probe(*ctx, probe_spirv(), args, std::span<float>(out));
    ASSERT_TRUE(result.has_value()) << result.error().context;

    uint64_t mismatches = 0;
    int printed = 0;
    for (std::size_t i = 0; i < cases.size(); ++i) {
        const ShiftCase& c = cases[i];
        // The host answer, computed with the native operator this emulation
        // exists to reproduce. `distance` is always < 64 here, so the shift is
        // well defined on this side too.
        const uint64_t expected = c.value >> c.distance;
        const uint64_t got = static_cast<uint64_t>(bits_of(out[2 * i])) |
                             (static_cast<uint64_t>(bits_of(out[2 * i + 1])) << 32);
        if (got == expected) continue;

        ++mismatches;
        if (printed < kMaxPrintedMismatches) {
            ++printed;
            std::printf("[GpuU64] mismatch: 0x%016" PRIx64 " >> %u  host=0x%016" PRIx64
                        "  gpu=0x%016" PRIx64 "\n",
                        c.value, c.distance, expected, got);
        }
    }

    std::printf("[GpuU64] ShiftMatchesTheHostAtEveryDistance: n=%zu  mismatches=%" PRIu64 "\n",
                cases.size(), mismatches);

    // NON-VACUITY: the sweep covered every distance and both word halves, so a
    // buffer that came back untouched (all zeros) would be caught by the
    // n == 0 cases, whose expected answer is the input value itself.
    ASSERT_EQ(cases.size(), values.size() * 64u);
    EXPECT_EQ(mismatches, 0u)
        << "engine/shaders/u64.slang's u64_shr disagrees with the host's 64-bit >> . The "
        << "emulation is what rng.slang's splitmix64 runs on, so a disagreement here is a "
        << "different random sequence on the device -- not a rounding difference.";
}
