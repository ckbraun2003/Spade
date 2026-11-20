// test_device_features.cpp -- the optional device features and the rule that resolves a request against what a device offers
// (camera design section 12, item 2; compute/device_features.hpp). Pure data and one pure function: no device, no Vulkan.
#include <gtest/gtest.h>

#include <string>
#include <string_view>

#include "compute/device_features.hpp"

namespace {

using spade::Code;
using spade::compute::DeviceFeature;
using spade::compute::FeatureSet;
using spade::compute::feature_name;
using spade::compute::kDeviceFeatureCount;
using spade::compute::resolve_features;
using spade::compute::with_dependencies;

constexpr std::string_view kDevice = "Test Device 9000";

[[nodiscard]] FeatureSet everything() {
    FeatureSet all;
    for (uint32_t i = 0; i < kDeviceFeatureCount; ++i) all.add(static_cast<DeviceFeature>(i));
    return all;
}

[[nodiscard]] bool contains(const std::string& text, std::string_view needle) { return text.find(needle) != std::string::npos; }

}  // namespace

TEST(DeviceFeatures, TheEightFeaturesHaveTheirNamesAndTheSetListsThemInOrder) {
    EXPECT_EQ(kDeviceFeatureCount, 8u);
    EXPECT_EQ(feature_name(DeviceFeature::timeline_semaphore), "timeline_semaphore");
    EXPECT_EQ(feature_name(DeviceFeature::external_memory_win32), "external_memory_win32");
    EXPECT_EQ(feature_name(DeviceFeature::external_semaphore_win32), "external_semaphore_win32");
    EXPECT_EQ(feature_name(DeviceFeature::ray_query), "ray_query");
    EXPECT_EQ(feature_name(DeviceFeature::acceleration_structure), "acceleration_structure");
    EXPECT_EQ(feature_name(DeviceFeature::buffer_device_address), "buffer_device_address");
    EXPECT_EQ(feature_name(DeviceFeature::storage_8bit), "storage_8bit");
    EXPECT_EQ(feature_name(DeviceFeature::storage_16bit), "storage_16bit");
    EXPECT_TRUE(FeatureSet{}.empty());
    EXPECT_EQ(FeatureSet{}.describe(), "");
    const FeatureSet two{DeviceFeature::ray_query, DeviceFeature::timeline_semaphore};
    EXPECT_FALSE(two.empty());
    EXPECT_TRUE(two.contains(DeviceFeature::ray_query));
    EXPECT_FALSE(two.contains(DeviceFeature::acceleration_structure));
    EXPECT_EQ(two.describe(), "timeline_semaphore, ray_query") << "enumerator order, not the order asked";
}

TEST(DeviceFeatures, TheStorageFeaturesDependOnNothingAndAreRefusedByNameLikeTheRest) {
    EXPECT_EQ(with_dependencies(FeatureSet{DeviceFeature::storage_8bit, DeviceFeature::storage_16bit}),
              (FeatureSet{DeviceFeature::storage_8bit, DeviceFeature::storage_16bit}));
    const auto refused = resolve_features(FeatureSet{DeviceFeature::storage_8bit}, FeatureSet{DeviceFeature::storage_16bit}, kDevice);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, Code::unavailable);
    for (const std::string_view needle : {"storage_8bit", "storageBuffer8BitAccess", "shaderInt8", "Test Device 9000"}) {
        EXPECT_TRUE(contains(refused.error().context, needle)) << "missing '" << needle << "' in: " << refused.error().context;
    }
}

TEST(DeviceFeatures, ARayQueryDragsInTheAccelerationStructureAndItsDeviceAddresses) {
    const FeatureSet closure = with_dependencies(FeatureSet{DeviceFeature::ray_query});
    EXPECT_EQ(closure, (FeatureSet{DeviceFeature::ray_query, DeviceFeature::acceleration_structure,
                                   DeviceFeature::buffer_device_address}))
        << closure.describe();
    EXPECT_EQ(with_dependencies(FeatureSet{DeviceFeature::timeline_semaphore}), FeatureSet{DeviceFeature::timeline_semaphore});
    EXPECT_EQ(with_dependencies(FeatureSet{}), FeatureSet{});
}

TEST(DeviceFeatures, NothingAskedEnablesNothingWhateverTheDeviceOffers) {
    for (const FeatureSet offered : {FeatureSet{}, everything()}) {
        const auto resolved = resolve_features(FeatureSet{}, offered, kDevice);
        ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
        EXPECT_TRUE(resolved->empty()) << "today's device: no extension, no feature chain";
    }
}

TEST(DeviceFeatures, OnlyWhatIsAskedForAndItsDependenciesIsEnabledNotEverythingOffered) {
    auto resolved = resolve_features(FeatureSet{DeviceFeature::timeline_semaphore}, everything(), kDevice);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(*resolved, FeatureSet{DeviceFeature::timeline_semaphore});
    resolved = resolve_features(FeatureSet{DeviceFeature::ray_query}, everything(), kDevice);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(*resolved, (FeatureSet{DeviceFeature::ray_query, DeviceFeature::acceleration_structure,
                                     DeviceFeature::buffer_device_address}));
}

TEST(DeviceFeatures, AFeatureTheDeviceLacksIsRefusedByNameWithTheDeviceAndWhatItWouldTake) {
    const FeatureSet offered{DeviceFeature::timeline_semaphore, DeviceFeature::buffer_device_address};
    const auto resolved = resolve_features(FeatureSet{DeviceFeature::timeline_semaphore, DeviceFeature::ray_query}, offered,
                                           kDevice);
    ASSERT_FALSE(resolved.has_value());
    EXPECT_EQ(resolved.error().code, Code::unavailable);
    const std::string& why = resolved.error().context;
    // The first feature of the request's closure the device lacks, in enumerator order: ray_query (3) before
    // acceleration_structure (4), and the one that was asked for, so it says so.
    for (const std::string_view needle : {"ray_query", "VK_KHR_ray_query", "Test Device 9000", "asked for"}) {
        EXPECT_TRUE(contains(why, needle)) << "missing '" << needle << "' in: " << why;
    }
}

TEST(DeviceFeatures, ADependencyTheDeviceLacksIsRefusedNamingItAndTheFeatureThatNeededIt) {
    // The device offers ray_query and the acceleration structure but not the device addresses the structure is built from.
    const FeatureSet offered{DeviceFeature::ray_query, DeviceFeature::acceleration_structure};
    const auto resolved = resolve_features(FeatureSet{DeviceFeature::ray_query}, offered, kDevice);
    ASSERT_FALSE(resolved.has_value());
    EXPECT_EQ(resolved.error().code, Code::unavailable);
    const std::string& why = resolved.error().context;
    for (const std::string_view needle : {"buffer_device_address", "needed by", "acceleration_structure", "Test Device 9000"}) {
        EXPECT_TRUE(contains(why, needle)) << "missing '" << needle << "' in: " << why;
    }
}
