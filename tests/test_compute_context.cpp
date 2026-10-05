#include <gtest/gtest.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/error.hpp"
#include "gpu_skip.hpp"

namespace {

using spade::Code;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::VulkanContext;
using spade::compute::vulkan_available;

// RAII SPADE_FORCE_NO_VULKAN override -- guarantees the env var is restored
// even if an ASSERT_* below fires and returns from the test body early;
// without this guard a failed ASSERT_ would leave the override set for
// every test that runs afterward in this same spade_tests process.
class ScopedForceNoVulkan {
public:
    ScopedForceNoVulkan() {
#ifdef _WIN32
        _putenv_s("SPADE_FORCE_NO_VULKAN", "1");
#else
        setenv("SPADE_FORCE_NO_VULKAN", "1", 1);
#endif
    }
    ~ScopedForceNoVulkan() {
#ifdef _WIN32
        _putenv_s("SPADE_FORCE_NO_VULKAN", "");
#else
        unsetenv("SPADE_FORCE_NO_VULKAN");
#endif
    }
    ScopedForceNoVulkan(const ScopedForceNoVulkan&) = delete;
    ScopedForceNoVulkan& operator=(const ScopedForceNoVulkan&) = delete;
};

}  // namespace

// ---------------------------------------------------------------------------
// ADMITTED AND ANNOUNCED, ON REAL HARDWARE (CORE-5; the banded-parity plan,
// T4). No kernel requests an fp32 denormal mode (SPIR-V rule P3), so a device
// that cannot preserve fp32 denormals runs every kernel legally with its own
// default, and is admitted. It is announced, never silent (L6): the context
// records the device, the driver and the float controls, and describe() says
// that no denormal mode was requested.
//
// The capability is read straight from the driver, independent of the engine.
// This RTX 3060 Ti reports shaderDenormPreserveFloat32 = false, so the whole
// test runs here; a preserving device checks admission and skips the rest.
// ---------------------------------------------------------------------------
TEST(GpuContext, ADeviceWithoutFp32DenormalPreservationIsAdmittedAndAnnounced) {
    if (spade::testing::forced_no_vulkan()) GTEST_SKIP() << "Vulkan unavailable: SPADE_FORCE_NO_VULKAN=1";
    const spade::testing::DefaultDeviceDenorms denorms = spade::testing::read_default_device_denorms();
    if (!denorms.device) GTEST_SKIP() << "Vulkan unavailable: no Vulkan loader or physical device";
    if (denorms.api_version < VK_API_VERSION_1_1) GTEST_SKIP() << "'" << denorms.name << "' reports Vulkan < 1.1";

    EXPECT_TRUE(vulkan_available()) << "'" << denorms.name << "' must be admitted";
    BackendDesc desc{.kind = BackendKind::vulkan};
    auto context = VulkanContext::create(desc);
    ASSERT_TRUE(context.has_value()) << context.error().context;

    if (!denorms.queryable || denorms.preserve_f32 == VK_TRUE) {
        GTEST_SKIP() << "the default device ('" << denorms.name
                     << "') preserves fp32 denormals or cannot say; the announcement needs one that does not";
    }
    const spade::compute::DeviceReport& report = (*context)->device_report();
    EXPECT_FALSE(report.denorm_preserve_f32);
    const std::string line = spade::compute::describe(report);
    EXPECT_NE(line.find(denorms.name), std::string::npos) << line;
    EXPECT_NE(line.find("no mode requested"), std::string::npos) << line;
    EXPECT_NE(line.find("preserve supported: no"), std::string::npos) << line;
}

// The report's fields equal a direct read of the driver (gpu_skip.hpp's, an
// independent source: TD-4), and describe() prints them on one line.
TEST(GpuContext, ReportsTheDeviceDriverAndFloatControls) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::testing::DefaultDeviceDenorms direct = spade::testing::read_default_device_denorms();
    ASSERT_TRUE(direct.device);
    BackendDesc desc{.kind = BackendKind::vulkan};
    auto context = VulkanContext::create(desc);
    ASSERT_TRUE(context.has_value()) << context.error().context;

    const spade::compute::DeviceReport& report = (*context)->device_report();
    EXPECT_EQ(report.device_name, direct.name);
    EXPECT_EQ(report.vendor_id, direct.vendor_id);
    EXPECT_EQ(report.device_id, direct.device_id);
    EXPECT_EQ(report.api_version, direct.api_version);
    EXPECT_EQ(report.driver_version, direct.driver_version);
    EXPECT_EQ(report.float_controls_queryable, direct.queryable);
    if (direct.queryable) {
        EXPECT_EQ(report.driver_name, direct.driver_name);
        EXPECT_EQ(report.driver_info, direct.driver_info);
        EXPECT_EQ(report.denorm_preserve_f32, direct.preserve_f32 == VK_TRUE);
        EXPECT_EQ(report.denorm_flush_to_zero_f32, direct.flush_to_zero_f32 == VK_TRUE);
    }
    std::cout << "[GpuContext] " << spade::compute::describe(report) << "\n";
}

// ---------------------------------------------------------------------------
// Device-executing: ctest label "gpu" (AppendSpadeLabels.cmake's Gpu*.*
// name-prefix rule, this task), each begins with the GTEST_SKIP() guard the
// global constraint requires ("Every device-executing test begins
// if (!compute::vulkan_available()) GTEST_SKIP()"). Runs for real on this
// program's box (Intel Iris Plus Graphics, Vulkan 1.3.215); skips wherever
// vulkan_available() is false -- CI runners (no device), or
// SPADE_FORCE_NO_VULKAN=1 local runs.
// ---------------------------------------------------------------------------

TEST(GpuContext, CreatesOnAvailableDevice) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    BackendDesc desc{.kind = BackendKind::vulkan};
    auto result = VulkanContext::create(desc);
    ASSERT_TRUE(result.has_value()) << result.error().context;

    std::unique_ptr<VulkanContext> ctx = std::move(*result);
    EXPECT_FALSE(ctx->device_name().empty());
    EXPECT_GT(ctx->api_version(), 0u);

    // Literal device-name/api-version line this task's report captures via
    // `ctest -V` -- the program rule's per-claim verification requirement
    // for the box's own run (not a substitute for the ctest transcript
    // itself, which the report also carries).
    std::cout << "[GpuContext] device_name=" << ctx->device_name()
              << " api_version=" << VK_API_VERSION_MAJOR(ctx->api_version()) << "."
              << VK_API_VERSION_MINOR(ctx->api_version()) << "."
              << VK_API_VERSION_PATCH(ctx->api_version()) << std::endl;
}

// ---------------------------------------------------------------------------
// S6 TASK 8's INT64 PROBE (plan-mandated, run before rng.slang was written).
//
// WHAT IT ANSWERS AND WHY IT IS PERMANENT. shaderInt64 decides how the engine
// may spell a 64-bit integer on the device -- splitmix64's state, a sensor's
// monotonic sample index, a tick. The answer on this program's correctness
// device is NO (VK_FALSE), which is why engine/shaders/u64.slang exists and why
// shaders/shared/layouts.slang mirrors every 64-bit field as a `uint2`.
//
// IT ASSERTS CONSISTENCY, NOT A BOX FACT. Hardcoding "shaderInt64 is false"
// would be asserting a property of this one GPU rather than of the code, and it
// would fail (correctly, but uselessly) the first time this suite ran on a
// device that has it. What IS box-independent -- and what actually protects the
// engine -- is tests/test_slang_layouts.cpp's SlangSpirv.FloatControlsPinned
// rule P4: NO compiled module may declare OpCapability Int64, on any device.
// That is the assertion with teeth; this one records the measurement, prints it
// for the report, and pins that the accessor reports the device rather than
// something else.
// ---------------------------------------------------------------------------
TEST(GpuContext, Int64ProbeMatchesTheDevice) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    BackendDesc desc{.kind = BackendKind::vulkan};
    auto result = VulkanContext::create(desc);
    ASSERT_TRUE(result.has_value()) << result.error().context;
    std::unique_ptr<VulkanContext> ctx = std::move(*result);

    // Read independently of the context, so the comparison below is a real
    // cross-check of the accessor rather than a tautology.
    VkPhysicalDeviceFeatures features{};
    vkGetPhysicalDeviceFeatures(ctx->physical_device(), &features);

    std::cout << "[GpuContext] shaderInt64 advertised=" << (features.shaderInt64 == VK_TRUE ? 1 : 0)
              << " device=" << ctx->device_name() << std::endl;

    EXPECT_EQ(ctx->supports_int64(), features.shaderInt64 == VK_TRUE)
        << "VulkanContext::supports_int64() disagrees with vkGetPhysicalDeviceFeatures for the very "
        << "device it was created on";

    // A CONTEXT EXISTS AT ALL -- which is the substantive claim, given the
    // answer above: this engine runs on a device WITHOUT shaderInt64, because
    // no kernel needs it. If a future change reintroduced a 64-bit type into a
    // kernel, rule P4 would fail the build long before this line; if somebody
    // deleted P4 too, the device would be executing a module declaring an
    // unsupported capability, which is undefined behaviour rather than an
    // error. Stated here so the two tests are read as one argument.
    EXPECT_NE(ctx->device(), VK_NULL_HANDLE);
}

// S6 Task 5: the debug messenger wiring itself, not merely the absence of a
// validation message -- proving the messenger EXISTS is the positive half;
// the GpuStateMirrorTest suite's clean pass under the debug preset (no
// message fired) is only the negative half, which is indistinguishable from
// "no messenger was ever created" on its own.
//
// CONSISTENCY, NOT A HARDCODED EXPECTATION: this box's Vulkan installation
// carries the driver and loader but NOT the LunarG Vulkan SDK's validation
// layers (`vulkaninfo`'s own "Instance Layers:" section enumerates zero on
// this box -- this task's report has the transcript), so
// VK_LAYER_KHRONOS_validation is legitimately ABSENT here even under the
// debug preset. context.cpp's own "IF present -- absence is not an error"
// posture (this task's brief, carried over from Task 1's layer check) means
// has_debug_messenger() being false on THIS box is correct, not a bug --
// asserting it unconditionally true would be asserting a fact about this
// one box's software inventory, not about the code. What this test CAN
// assert regardless of environment: has_debug_messenger() agrees with
// whether the layer was actually enumerable, on both presets.
TEST(GpuContext, DebugMessengerExistsIffValidationLayerIsPresent) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    uint32_t layer_count = 0;
    vkEnumerateInstanceLayerProperties(&layer_count, nullptr);
    std::vector<VkLayerProperties> available_layers(layer_count);
    vkEnumerateInstanceLayerProperties(&layer_count, available_layers.data());
    bool validation_layer_present = false;
    for (const auto& layer : available_layers) {
        if (std::string_view(layer.layerName) == "VK_LAYER_KHRONOS_validation") {
            validation_layer_present = true;
            break;
        }
    }

    BackendDesc desc{.kind = BackendKind::vulkan};
    auto result = VulkanContext::create(desc);
    ASSERT_TRUE(result.has_value()) << result.error().context;
    std::unique_ptr<VulkanContext> ctx = std::move(*result);

#ifndef NDEBUG
    // Debug preset: the messenger exists exactly when the layer does --
    // context.cpp only requests VK_EXT_debug_utils (and therefore only
    // creates a messenger) when the layer was found first.
    EXPECT_EQ(ctx->has_debug_messenger(), validation_layer_present)
        << "validation_layer_present=" << validation_layer_present;
#else
    // Release: NDEBUG skips the whole layer/extension probe unconditionally,
    // so no messenger is ever requested regardless of what is installed.
    EXPECT_FALSE(ctx->has_debug_messenger()) << "a Release build must never request the validation layer";
#endif

    std::cout << "[GpuContext] validation_layer_present=" << validation_layer_present
              << " has_debug_messenger=" << ctx->has_debug_messenger() << std::endl;
}

TEST(GpuContext, ComputeAndTransferQueuesAreValid) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    BackendDesc desc{.kind = BackendKind::vulkan};
    auto result = VulkanContext::create(desc);
    ASSERT_TRUE(result.has_value()) << result.error().context;

    std::unique_ptr<VulkanContext> ctx = std::move(*result);
    // Real handles, not VK_NULL_HANDLE -- proves both
    // vkGetDeviceQueue() calls (context.cpp) actually ran, whether or not
    // this device's compute/transfer families turned out to be the same
    // family (the Iris case -- see pick_transfer_family()'s fallback).
    EXPECT_NE(ctx->compute_queue(), nullptr);
    EXPECT_NE(ctx->transfer_queue(), nullptr);
    EXPECT_NE(ctx->device(), nullptr);
    EXPECT_NE(ctx->instance(), nullptr);
    EXPECT_NE(ctx->physical_device(), nullptr);
}

TEST(GpuContext, DeviceIndexOutOfRangeIsInvalidArgument) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    BackendDesc desc{.kind = BackendKind::vulkan, .device_index = 0xffffffffu};
    auto result = VulkanContext::create(desc);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, Code::invalid_argument);
}

TEST(GpuContext, MoveTransfersOwnershipAndLeavesSourceInert) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;

    BackendDesc desc{.kind = BackendKind::vulkan};
    auto result = VulkanContext::create(desc);
    ASSERT_TRUE(result.has_value()) << result.error().context;

    std::unique_ptr<VulkanContext> ctx = std::move(*result);
    VulkanContext moved(std::move(*ctx));
    EXPECT_FALSE(moved.device_name().empty());
    EXPECT_NE(moved.device(), nullptr);
    // `*ctx` (moved-from) and `moved` both destruct at scope exit --
    // exactly the rule-of-five claim under test here: destroy() must be a
    // safe no-op on the released source and a real teardown on the new
    // owner, in either destruction order. A double-destroy would surface as
    // a driver-validation/ASan failure, not a gtest assertion, which is why
    // this test's pass/fail alone is not the whole proof -- the box run
    // completing without a crash is the other half.
}

// ---------------------------------------------------------------------------
// Host-only: deliberately NOT Gpu*-prefixed (AppendSpadeLabels.cmake's rule
// only tags device-executing suites with "gpu"), so it carries no "gpu"
// label and runs on every box/CI runner regardless of Vulkan presence -- it
// never reaches vulkan_available()'s real device probe, only its
// SPADE_FORCE_NO_VULKAN env-override short-circuit (checked first, inside
// vulkan_available() itself).
// ---------------------------------------------------------------------------

TEST(ComputeBackendAvailability, ForcedUnavailableReturnsUnavailable) {
    ScopedForceNoVulkan guard;

    EXPECT_FALSE(vulkan_available());
    EXPECT_EQ(spade::testing::vulkan_skip_reason(), std::optional<std::string>("Vulkan unavailable: SPADE_FORCE_NO_VULKAN=1"))
        << "a forced skip names the override";

    BackendDesc desc{.kind = BackendKind::vulkan};
    auto result = VulkanContext::create(desc);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, Code::unavailable);

    // AND THE OVERRIDE SHORT-CIRCUITS BEFORE ANY DRIVER CALL (S6 Task 4, review
    // fix round 2). vulkan_available() gained a second reason to answer false --
    // a present device that cannot preserve fp32 denormals -- and that clause
    // creates an instance, enumerates a device and queries its properties.
    // SPADE_FORCE_NO_VULKAN exists so a host-only test can get its "no" WITHOUT
    // touching a driver, so the env check must stay FIRST; and "first" is a
    // property of the code that a reordering could silently break.
    //
    // TWO THINGS PROVE IT, and they are different claims:
    //
    //   1. THE ASSERTIONS ABOVE, on this program's box. A fully usable device
    //      is present here (the Iris reports shaderDenormPreserveFloat32=1), so
    //      an override that did NOT win would leave create() succeeding --
    //      ASSERT_FALSE would have caught it. That is the ordering evidence,
    //      and it is only available where a device exists.
    //   2. THE DISCRIMINATOR BELOW, everywhere. `SPIR-V 1.3` (CORE-5: the one
    //      capability left is Vulkan 1.1, because the kernels are SPIR-V 1.3)
    //      appears verbatim in create()'s PER-DEVICE capability rejection and
    //      never in its predicate-failure message (context.cpp says so at both
    //      sites), so its absence pins that the answer came from the predicate
    //      rather than from a device query -- i.e. that the forced path never
    //      reached one. This holds on a device-less CI runner too, where claim
    //      1 is vacuous.
    EXPECT_EQ(result.error().context.find("SPIR-V 1.3"), std::string::npos)
        << "SPADE_FORCE_NO_VULKAN must short-circuit before any device is queried, but the "
        << "rejection came from the per-device capability check: " << result.error().context;
}
