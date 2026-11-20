// test_gpu_step_semaphore.cpp -- the optional device features and the step's timeline semaphore (camera design section 12,
// item 2; compute/device_features.hpp and compute/vulkan/pose_view.hpp state the contract). The probe is checked against an
// independent read of the device (this file's own instance and queries); the semaphore is checked with this file's own
// Vulkan calls. Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <volk.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include "compute/backend.hpp"
#include "compute/device_features.hpp"
#include "compute/vulkan/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/pose_view.hpp"
#include "gpu_skip.hpp"
#include "set_body_state_support.hpp"
#include "sim/simulation.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::BodyState;
using spade::Simulation;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::DeviceFeature;
using spade::compute::FeatureSet;
using spade::compute::VulkanContext;
using spade::compute::kDeviceFeatureCount;
using spade::compute::step_semaphore;
using spade::compute::with_dependencies;
using spade::test::bare_body;
using spade::test::floor_world_set;
using spade::test::row_of;

// What the default device offers, read straight from the driver by this file's own instance (not through the engine's probe).
[[nodiscard]] FeatureSet independent_probe() {
    FeatureSet out;
    if (volkInitialize() != VK_SUCCESS) return out;
    VkApplicationInfo app{};
    app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "spade_test_feature_read";
    app.apiVersion = VK_API_VERSION_1_3;
    VkInstanceCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    info.pApplicationInfo = &app;
    VkInstance instance = VK_NULL_HANDLE;
    if (vkCreateInstance(&info, nullptr, &instance) != VK_SUCCESS) return out;
    volkLoadInstanceOnly(instance);
    uint32_t count = 1;
    VkPhysicalDevice device = VK_NULL_HANDLE;
    const VkResult listed = vkEnumeratePhysicalDevices(instance, &count, &device);
    if ((listed == VK_SUCCESS || listed == VK_INCOMPLETE) && count > 0) {
        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(device, &props);
        uint32_t n = 0;
        vkEnumerateDeviceExtensionProperties(device, nullptr, &n, nullptr);
        std::vector<VkExtensionProperties> ext(n);
        vkEnumerateDeviceExtensionProperties(device, nullptr, &n, ext.data());
        const auto has = [&](const char* name) {
            for (const VkExtensionProperties& e : ext) {
                if (std::strcmp(e.extensionName, name) == 0) return true;
            }
            return false;
        };
        VkPhysicalDeviceVulkan12Features v12{};
        v12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
        VkPhysicalDevice16BitStorageFeatures sixteen{};
        sixteen.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES;
        VkPhysicalDeviceAccelerationStructureFeaturesKHR as{};
        as.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
        VkPhysicalDeviceRayQueryFeaturesKHR rq{};
        rq.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR;
        v12.pNext = &sixteen;
        sixteen.pNext = &as;
        as.pNext = &rq;
        VkPhysicalDeviceFeatures2 features{};
        features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
        features.pNext = &v12;
        // Only structs the device may legally be asked about: Vulkan12Features from 1.2, the two KHR ones when advertised.
        if (!has("VK_KHR_acceleration_structure")) sixteen.pNext = nullptr;
        else if (!has("VK_KHR_ray_query")) as.pNext = nullptr;
        if (props.apiVersion >= VK_API_VERSION_1_2) {
            vkGetPhysicalDeviceFeatures2(device, &features);
            if (v12.timelineSemaphore) out.add(DeviceFeature::timeline_semaphore);
            if (v12.bufferDeviceAddress) out.add(DeviceFeature::buffer_device_address);
            if (v12.storageBuffer8BitAccess && v12.shaderInt8) out.add(DeviceFeature::storage_8bit);
            if (sixteen.storageBuffer16BitAccess && features.features.shaderInt16) out.add(DeviceFeature::storage_16bit);
            if (has("VK_KHR_acceleration_structure") && has("VK_KHR_deferred_host_operations") && as.accelerationStructure) {
                out.add(DeviceFeature::acceleration_structure);
            }
            if (has("VK_KHR_ray_query") && has("VK_KHR_acceleration_structure") && rq.rayQuery) out.add(DeviceFeature::ray_query);
        }
        if (has("VK_KHR_external_memory_win32")) out.add(DeviceFeature::external_memory_win32);
        if (has("VK_KHR_external_semaphore_win32")) {
            VkSemaphoreTypeCreateInfo type{};
            type.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
            type.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
            VkPhysicalDeviceExternalSemaphoreInfo ask{};
            ask.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_SEMAPHORE_INFO;
            ask.pNext = &type;
            ask.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT;
            VkExternalSemaphoreProperties answer{};
            answer.sType = VK_STRUCTURE_TYPE_EXTERNAL_SEMAPHORE_PROPERTIES;
            vkGetPhysicalDeviceExternalSemaphoreProperties(device, &ask, &answer);
            if ((answer.externalSemaphoreFeatures & VK_EXTERNAL_SEMAPHORE_FEATURE_EXPORTABLE_BIT) != 0) {
                out.add(DeviceFeature::external_semaphore_win32);
            }
        }
    }
    vkDestroyInstance(instance, nullptr);
    return out;
}

[[nodiscard]] VkSemaphore make_timeline(VkDevice device, uint64_t initial) {
    VkSemaphoreTypeCreateInfo type{};
    type.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
    type.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
    type.initialValue = initial;
    VkSemaphoreCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    info.pNext = &type;
    VkSemaphore s = VK_NULL_HANDLE;
    EXPECT_EQ(vkCreateSemaphore(device, &info, nullptr, &s), VK_SUCCESS);
    return s;
}

struct Rig {
    Simulation sim;
    BodyRef body;
};

// A flying, turning body in a one-world floor set, on Vulkan with the timeline asked for or not.
[[nodiscard]] Rig make_rig(bool timeline, FeatureSet features = {}) {
    const spade::Result<spade::WorldSetDesc> set = floor_world_set(2);
    EXPECT_TRUE(set.has_value()) << set.error().context;
    BackendDesc desc{.kind = BackendKind::vulkan};
    desc.step_timeline = timeline;
    desc.device_features = features;
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 2, desc);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    BodySpawn flying = bare_body(glm::vec3(0.0f, 3.0f, 0.0f));
    flying.orient = glm::quat(0.95f, 0.1f, -0.2f, 0.2f);
    flying.vel = glm::vec3(0.4f, 0.0f, -0.2f);
    flying.omega_body = glm::vec3(0.3f, -0.2f, 0.5f);
    const spade::Result<BodyRef> body = sim->spawn(0, flying);
    EXPECT_TRUE(body.has_value()) << body.error().context;
    EXPECT_TRUE(sim->flush_structural().has_value());
    return Rig{std::move(*sim), *body};
}

[[nodiscard]] uint64_t counter_of(VulkanContext& ctx, VkSemaphore s) {
    uint64_t v = ~uint64_t{0};
    EXPECT_EQ(vkGetSemaphoreCounterValue(ctx.device(), s, &v), VK_SUCCESS);
    return v;
}

}  // namespace

// THE PROBE: the device report says what the device offers, matching an independent read; and a context made without asking
// enables none of it.
TEST(GpuStepSemaphore, TheProbeListsWhatTheDeviceOffersAndADefaultDeviceEnablesNone) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto context = VulkanContext::create(BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(context.has_value()) << context.error().context;
    const FeatureSet expected = independent_probe();
    const spade::compute::DeviceReport& report = (*context)->device_report();
    EXPECT_EQ(report.optional_supported, expected)
        << "engine probe: '" << report.optional_supported.describe() << "', independent read: '" << expected.describe() << "'";
    EXPECT_TRUE(report.optional_enabled.empty()) << "nothing asked, nothing enabled: " << report.optional_enabled.describe();
    std::fprintf(stderr, "[ PROBE    ] %s offers: %s\n", std::string((*context)->device_name()).c_str(),
                 report.optional_supported.describe().c_str());
}

// ASKING: every feature the device offers can be asked for (alone), is reported enabled with what it depends on, and the
// timeline one works (a semaphore of this device counts and signals).
TEST(GpuStepSemaphore, AskingForAFeatureTheDeviceOffersEnablesItAndItsDependencies) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const FeatureSet offered = independent_probe();
    if (offered.empty()) GTEST_SKIP() << "the device offers none of the optional features";
    for (uint32_t i = 0; i < kDeviceFeatureCount; ++i) {
        const auto feature = static_cast<DeviceFeature>(i);
        if (!offered.contains(feature)) continue;
        BackendDesc desc{.kind = BackendKind::vulkan};
        desc.device_features = FeatureSet{feature};
        auto context = VulkanContext::create(desc);
        ASSERT_TRUE(context.has_value()) << spade::compute::feature_name(feature) << ": " << context.error().context;
        EXPECT_EQ((*context)->device_report().optional_enabled, with_dependencies(FeatureSet{feature}))
            << spade::compute::feature_name(feature);
        if (feature == DeviceFeature::timeline_semaphore) {
            const VkDevice device = (*context)->device();
            const VkSemaphore s = make_timeline(device, 2);
            ASSERT_NE(s, VkSemaphore{VK_NULL_HANDLE});
            EXPECT_EQ(counter_of(**context, s), 2u);
            const VkSemaphoreSignalInfo signal{VK_STRUCTURE_TYPE_SEMAPHORE_SIGNAL_INFO, nullptr, s, 9};
            EXPECT_EQ(vkSignalSemaphore(device, &signal), VK_SUCCESS);
            EXPECT_EQ(counter_of(**context, s), 9u);
            vkDestroySemaphore(device, s, nullptr);
        }
    }
}

// REFUSING: a feature the device lacks fails create() by name (on a device that offers all six this test has nothing to ask
// for; the rule itself is DeviceFeatures.* on the CPU).
TEST(GpuStepSemaphore, AskingForAFeatureTheDeviceLacksIsRefusedByNameAndTheDeviceIsNotCreated) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const FeatureSet offered = independent_probe();
    uint32_t asked = 0;
    for (uint32_t i = 0; i < kDeviceFeatureCount; ++i) {
        const auto feature = static_cast<DeviceFeature>(i);
        if (offered.contains(feature)) continue;
        ++asked;
        BackendDesc desc{.kind = BackendKind::vulkan};
        desc.device_features = FeatureSet{feature};
        const auto context = VulkanContext::create(desc);
        ASSERT_FALSE(context.has_value()) << spade::compute::feature_name(feature);
        EXPECT_EQ(context.error().code, spade::Code::unavailable) << context.error().context;
        EXPECT_NE(context.error().context.find(std::string(spade::compute::feature_name(feature))), std::string::npos)
            << context.error().context;
    }
    if (asked == 0) GTEST_SKIP() << "this device offers all eight; the refusal is DeviceFeatures.AFeatureTheDeviceLacks...";
}

// THE SEMAPHORE IS OPT-IN: a backend made without step_timeline has none to hand out.
TEST(GpuStepSemaphore, TheStepSemaphoreIsRefusedUnlessAskedFor) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Rig rig = make_rig(false);
    ASSERT_NE(rig.sim.vulkan_backend(), nullptr);
    const auto s = step_semaphore(*rig.sim.vulkan_backend());
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::unavailable) << s.error().context;
    EXPECT_NE(s.error().context.find("step_timeline"), std::string::npos) << s.error().context;
    EXPECT_TRUE(rig.sim.vulkan_backend()->device_report().optional_enabled.empty())
        << "without the timeline the device is today's";
}

// THE COUNTER: the semaphore's value is the number of tick submits completed since create, read by this test's own
// vkGetSemaphoreCounterValue; a wait on the current value returns at once; the handle is the same for the backend's life.
TEST(GpuStepSemaphore, TheSemaphoreCountsTheTickSubmitsAndAWaitOnTheCurrentValueReturns) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!independent_probe().contains(DeviceFeature::timeline_semaphore)) GTEST_SKIP() << "no timeline semaphores";
    Rig rig = make_rig(true);
    EXPECT_TRUE(rig.sim.vulkan_backend()->device_report().optional_enabled.contains(DeviceFeature::timeline_semaphore));
    auto s = step_semaphore(*rig.sim.vulkan_backend());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    ASSERT_NE(s->context, nullptr);
    ASSERT_NE(s->semaphore, VkSemaphore{VK_NULL_HANDLE});
    EXPECT_EQ(s->value, 0u);
    EXPECT_EQ(counter_of(*s->context, s->semaphore), 0u);
    const VkSemaphore handle = s->semaphore;

    uint64_t ticks = 0;
    for (const uint64_t n : {1u, 1u, 1u, 4u}) {
        ASSERT_TRUE(rig.sim.step(n).has_value());
        ticks += n;
        s = step_semaphore(*rig.sim.vulkan_backend());
        ASSERT_TRUE(s.has_value()) << s.error().context;
        EXPECT_EQ(s->semaphore, handle);
        EXPECT_EQ(s->value, ticks);
        EXPECT_EQ(counter_of(*s->context, s->semaphore), ticks) << "the device's own counter after " << ticks << " ticks";
        // A wait for the current value is already satisfied: zero timeout, so it cannot block the test.
        const VkSemaphoreWaitInfo wait{VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO, nullptr, 0, 1, &s->semaphore, &s->value};
        EXPECT_EQ(vkWaitSemaphores(s->context->device(), &wait, 0), VK_SUCCESS);
        // The next tick's value is not.
        const uint64_t next = ticks + 1;
        const VkSemaphoreWaitInfo ahead{VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO, nullptr, 0, 1, &s->semaphore, &next};
        EXPECT_EQ(vkWaitSemaphores(s->context->device(), &ahead, 0), VK_TIMEOUT);
    }
    EXPECT_FALSE(s->exportable) << "not asked for an export";
}

// EXPORT: asked for with external_semaphore_win32, the semaphore is exportable as an OPAQUE_WIN32 handle.
TEST(GpuStepSemaphore, WithTheWin32ExportAskedForTheSemaphoreYieldsAHandle) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const FeatureSet offered = independent_probe();
    if (!offered.contains(DeviceFeature::timeline_semaphore) || !offered.contains(DeviceFeature::external_semaphore_win32)) {
        GTEST_SKIP() << "no exportable timeline semaphore on this device";
    }
#ifndef _WIN32
    GTEST_SKIP() << "the win32 handle is a Windows object";
#else
    Rig rig = make_rig(true, FeatureSet{DeviceFeature::external_semaphore_win32});
    const auto s = step_semaphore(*rig.sim.vulkan_backend());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_TRUE(s->exportable);
    ASSERT_TRUE(rig.sim.step(2).has_value());

    // VkSemaphoreGetWin32HandleInfoKHR, spelled here so this file needs no platform Vulkan header.
    struct GetWin32HandleInfo {
        VkStructureType sType;
        const void* pNext;
        VkSemaphore semaphore;
        VkExternalSemaphoreHandleTypeFlagBits handleType;
    };
    using GetHandleFn = VkResult(VKAPI_PTR*)(VkDevice, const GetWin32HandleInfo*, HANDLE*);
    const auto fetch = reinterpret_cast<GetHandleFn>(vkGetDeviceProcAddr(s->context->device(), "vkGetSemaphoreWin32HandleKHR"));
    ASSERT_NE(fetch, nullptr) << "VK_KHR_external_semaphore_win32 was enabled";
    const GetWin32HandleInfo info{static_cast<VkStructureType>(1000078003), nullptr, s->semaphore,
                                  VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT};
    HANDLE handle = nullptr;
    ASSERT_EQ(fetch(s->context->device(), &info, &handle), VK_SUCCESS);
    EXPECT_NE(handle, nullptr);
    if (handle != nullptr) CloseHandle(handle);
#endif
}

// A RESULT-NEUTRAL KNOB (engine A7): the same run with and without the timeline ends in bit-identical state.
TEST(GpuStepSemaphore, TheTimelineChangesNoResult) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!independent_probe().contains(DeviceFeature::timeline_semaphore)) GTEST_SKIP() << "no timeline semaphores";
    Rig plain = make_rig(false);
    Rig timed = make_rig(true);
    for (const uint64_t n : {1u, 2u, 5u, 1u}) {
        ASSERT_TRUE(plain.sim.step(n).has_value());
        ASSERT_TRUE(timed.sim.step(n).has_value());
        const BodyState a = row_of(plain.sim, plain.body);
        const BodyState b = row_of(timed.sim, timed.body);
        EXPECT_EQ(std::memcmp(&a, &b, sizeof(BodyState)), 0) << "after a step of " << n;
    }
    // The command buffers are the same: the same dispatch and barrier counts.
    const auto plain_chain = plain.sim.vulkan_recorded_chain();
    const auto timed_chain = timed.sim.vulkan_recorded_chain();
    ASSERT_TRUE(plain_chain.has_value() && timed_chain.has_value());
    EXPECT_EQ(plain_chain->dispatches, timed_chain->dispatches);
    EXPECT_EQ(plain_chain->barriers, timed_chain->barriers);
}
