// admit.hpp -- every device-limit check of a created shape, in one pure function (module API stage 7, Task A1;
// docs/design/core/plans/2026-11-06-module-api-stage7-plan.md; design 7.1).
//
// B0 built the checks (compute/device_limits.hpp) and only the spike's ChainRunner called them. admit() is the call the
// engine makes: create() describes what it is about to create as an AdmitShape -- the pipeline layouts with their
// bindings per set, every buffer, every kernel's local size, every dispatch's group counts, the push block -- and
// admit() refuses the first thing the device cannot hold, by name, BEFORE the first vkCreate*. Plain data and a pure
// function with no Vulkan types, so it tests on every machine with synthetic limits.
//
// THE LAYOUT RULE (the lead's 2026-11-06 ruling): maxPerStageDescriptorStorageBuffers and
// maxDescriptorSetStorageBuffers count the bindings of EVERY set in a pipeline layout, so a layout's count is the sum
// over its sets (set 0's 30 plus the set-1 subset a module kernel names), not the largest set.
//
// Order of checks: layouts, push block, kernels, buffers, dispatches. The first refusal is returned; its code is
// Code::capacity_exceeded and its text names what was asked, the limit by its Vulkan name and value, and the device.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "compute/device_limits.hpp"
#include "core/error.hpp"

namespace spade::compute {

struct AdmitLayout {
    std::string name{};                        // the pipeline layout, as the refusal names it
    std::vector<uint32_t> bindings_per_set{};  // storage-buffer bindings in set 0, set 1, ...
};

struct AdmitBuffer {
    std::string name{};
    uint64_t bytes = 0;
};

struct AdmitKernel {
    std::string name{};
    std::array<uint32_t, 3> local{1, 1, 1};
};

struct AdmitDispatch {
    std::string pass{};
    std::array<uint32_t, 3> groups{1, 1, 1};
};

struct AdmitShape {
    std::vector<AdmitLayout> layouts{};
    std::vector<AdmitBuffer> buffers{};
    std::vector<AdmitKernel> kernels{};
    std::vector<AdmitDispatch> dispatches{};
    uint32_t push_bytes = 0;
};

// The first thing in `shape` that `limits` cannot hold, or success. Every refusal is B0's check (device_limits.hpp),
// prefixed with what asked where a layout is concerned: its name and the bindings of each set, so the count is
// readable as a sum ("pipeline layout 'state+points' (set 0: 30, set 1: 3): a layout of 33 storage buffers, over ...").
[[nodiscard]] inline Result<void> admit(const AdmitShape& shape, const DeviceLimits& limits, std::string_view device) {
    for (const AdmitLayout& layout : shape.layouts) {
        uint64_t total = 0;
        std::string sets;
        for (std::size_t set = 0; set < layout.bindings_per_set.size(); ++set) {
            total += layout.bindings_per_set[set];
            sets += (set == 0 ? "" : ", ") + std::string("set ") + std::to_string(set) + ": " +
                    std::to_string(layout.bindings_per_set[set]);
        }
        const uint32_t count = total > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(total);
        if (Result<void> ok = check_storage_buffers(count, limits, device); !ok) {
            return std::unexpected(Error{ok.error().code, "pipeline layout '" + layout.name + "' (" + sets + "): " +
                                                              ok.error().context});
        }
    }
    if (Result<void> ok = check_push(shape.push_bytes, limits, device); !ok) return ok;
    for (const AdmitKernel& kernel : shape.kernels) {
        if (Result<void> ok = check_workgroup(kernel.name, kernel.local, limits, device); !ok) return ok;
    }
    for (const AdmitBuffer& buffer : shape.buffers) {
        if (Result<void> ok = check_buffer(buffer.name, buffer.bytes, limits, device); !ok) return ok;
    }
    for (const AdmitDispatch& dispatch : shape.dispatches) {
        if (Result<void> ok = check_groups(dispatch.pass, dispatch.groups, limits, device); !ok) return ok;
    }
    return {};
}

}  // namespace spade::compute
