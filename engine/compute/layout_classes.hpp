// layout_classes.hpp -- the pipeline layout of each kernel (module API stage 7, Task B1; the stage 7 plan, decision 1).
//
// bindings.slang's section F stays the single list of set-1 bindings. A kernel's pipeline layout carries set 0 in full (any
// kernel that reads state needs the whole state set) and, if it reads module data, a set-1 layout holding only the bindings
// the kernel NAMES, at the declaration's numbers. maxPerStageDescriptorStorageBuffers and maxDescriptorSetStorageBuffers
// count every set of a layout, so a module kernel with a set-1 subset of k bindings costs 30 + k, and every kernel
// without one costs 30, as today. Vulkan-free: the recorder builds the VkDescriptorSetLayout from it, the mirror writes
// the descriptors, and admit() counts it, all from this one answer.
#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/admit.hpp"
#include "core/error.hpp"

namespace spade::compute {

// The layout of `kernel` with the set-1 bindings it names (numbers from bindings.slang section F, each below
// gen::kBindingCount_airflow and named once). An empty list is the plain state layout: one set of
// gen::kBindingCount_state bindings. A binding out of range or named twice is invalid_argument naming the kernel and the
// binding.
[[nodiscard]] inline Result<AdmitLayout> kernel_layout(std::string_view kernel,
                                                      std::span<const uint32_t> set1_bindings) {
    std::vector<bool> named(gen::kBindingCount_airflow, false);
    for (const uint32_t b : set1_bindings) {
        if (b >= gen::kBindingCount_airflow) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "kernel '" + std::string(kernel) + "': set-1 binding " + std::to_string(b) +
                                             " is not declared (bindings.slang section F has " +
                                             std::to_string(gen::kBindingCount_airflow) + ")"});
        }
        if (named[b]) {
            return std::unexpected(Error{Code::invalid_argument, "kernel '" + std::string(kernel) + "': set-1 binding " +
                                                                     std::to_string(b) + " is named twice"});
        }
        named[b] = true;
    }
    AdmitLayout layout{.name = std::string(kernel), .bindings_per_set = {gen::kBindingCount_state}};
    if (!set1_bindings.empty()) layout.bindings_per_set.push_back(static_cast<uint32_t>(set1_bindings.size()));
    return layout;
}

}  // namespace spade::compute
