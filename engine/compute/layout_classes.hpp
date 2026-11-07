// layout_classes.hpp -- the pipeline layout of each kernel (module API stage 7, Task B1; the stage 7 plan, decision 1).
//
// bindings.slang's section F stays the single list of set-1 bindings. A kernel's pipeline layout carries set 0 in full (any
// kernel that reads state needs the whole state set) and, if it reads module data, a set-1 layout holding only the bindings
// the kernel NAMES, at the declaration's numbers. maxPerStageDescriptorStorageBuffers and maxDescriptorSetStorageBuffers
// count every set of a layout, so a module kernel with a set-1 subset of k bindings costs 30 + k, and every kernel
// without one costs 30, as today. Vulkan-free: the recorder builds the VkDescriptorSetLayout from it, the mirror writes
// the descriptors, and admit() counts it, all from this one answer.
//
// RED STUB (stage 7 Task B1): kernel_layout() answers an empty layout.
#pragma once

#include <cstdint>
#include <span>
#include <string_view>

#include "compute/admit.hpp"
#include "core/error.hpp"

namespace spade::compute {

// The layout of `kernel` with the set-1 bindings it names (numbers from bindings.slang section F, each below
// gen::kBindingCount_airflow and named once). An empty list is the plain state layout: one set of
// gen::kBindingCount_state bindings. A binding out of range or named twice is invalid_argument naming the kernel and the
// binding.
[[nodiscard]] inline Result<AdmitLayout> kernel_layout(std::string_view /*kernel*/,
                                                      std::span<const uint32_t> /*set1_bindings*/) {
    return AdmitLayout{};
}

}  // namespace spade::compute
