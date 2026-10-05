// gpu_skip.hpp -- why a device-gated test skips (TD-6).
//
// Every Gpu* test that needs a Vulkan device begins with
//     if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
// so a log says WHY it skipped, not only that it did.
#pragma once

#include <optional>
#include <string>

namespace spade::testing {

[[nodiscard]] inline std::optional<std::string> vulkan_skip_reason() {
    return std::nullopt;
}

}  // namespace spade::testing
