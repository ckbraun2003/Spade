// scoped_no_vulkan.hpp -- a test body that must see a box with no Vulkan device, whatever this machine has.
//
// SPADE_FORCE_NO_VULKAN=1 is the override tests/gpu_skip.hpp and the engine's vulkan_available() both honour (TD-6).
// The guard sets it for its scope and clears it after, _putenv_s on Windows where /W4 /WX rejects std::getenv.
#pragma once

#include <cstdlib>

namespace spade::testing {

class ScopedNoVulkan {
  public:
    ScopedNoVulkan() {
#ifdef _WIN32
        _putenv_s("SPADE_FORCE_NO_VULKAN", "1");
#else
        setenv("SPADE_FORCE_NO_VULKAN", "1", 1);
#endif
    }
    ~ScopedNoVulkan() {
#ifdef _WIN32
        _putenv_s("SPADE_FORCE_NO_VULKAN", "");
#else
        unsetenv("SPADE_FORCE_NO_VULKAN");
#endif
    }
    ScopedNoVulkan(const ScopedNoVulkan&) = delete;
    ScopedNoVulkan& operator=(const ScopedNoVulkan&) = delete;
};

}  // namespace spade::testing
