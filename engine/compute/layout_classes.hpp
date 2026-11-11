// layout_classes.hpp -- the pipeline layout of each kernel (module API stage 7, Task B1; the stage 7 plan, decision 1).
//
// bindings.slang's section F stays the single list of set-1 bindings. A kernel's pipeline layout carries set 0 in full (any
// kernel that reads state needs the whole state set) and, if it reads module data, a set-1 layout holding only the bindings
// the kernel NAMES, at the declaration's numbers. maxPerStageDescriptorStorageBuffers and maxDescriptorSetStorageBuffers
// count every set of a layout, so a module kernel with a set-1 subset of k bindings costs 30 + k, and every kernel
// without one costs 30, as today. Vulkan-free: the recorder builds the VkDescriptorSetLayout from it, the mirror writes
// the descriptors, and admit() counts it, all from this one answer.
#pragma once

#include <algorithm>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "compute/admit.hpp"
#include "compute/set_one_bindings.hpp"  // kStateBindingCount (Vulkan-free)
#include "core/error.hpp"

namespace spade::compute {

// Set 1's binding space: numbers 0..47 (28 in use, six new for the fluid module, six spare, eight reserved for the LBM twin).
// The declared count (kAirflowDeclaredCount) is NOT the bound -- the numbers are sparse -- so a number inside the space
// that bindings.slang declares nowhere is refused by kernel_layout() below, by name, when the layout is made.
inline constexpr uint32_t kSet1BindingSlots = 48;

// Whether bindings.slang section F declares set-1 binding `b`: the airflow kernels' 0..27, then the stage 7 appendices declared
// so far (airflow_points, airflow_config, airflow_centred, contact_shapes, the four airflow_line_*) and the LBM twin's lbm_f and lbm_u. A batch that declares another
// binding adds it here, and the count assert below fails until it does.
[[nodiscard]] constexpr bool is_declared_set1(uint32_t b) noexcept {
    return b <= kAirflowWallLoadBinding || b == kAirflowPointsBinding || b == kAirflowConfigBinding ||
           b == kAirflowCentredBinding || b == kContactShapesBinding ||
           (b >= kAirflowLineFirstBinding && b < kAirflowLineFirstBinding + kAirflowLineBufferCount) || b == kLbmFBinding ||
           b == kLbmUBinding;
}
static_assert(
    [] {
        uint32_t declared = 0;
        for (uint32_t b = 0; b < kSet1BindingSlots; ++b) declared += is_declared_set1(b) ? 1u : 0u;
        return declared == kAirflowDeclaredCount;
    }(),
    "is_declared_set1 lists exactly the bindings bindings.slang section F declares (kAirflowDeclaredCount)");

// The layout of `kernel` with the set-1 bindings it names (numbers from bindings.slang section F, each below
// kSet1BindingSlots, declared in bindings.slang, and named once). An empty list is the plain state layout: one set of
// kStateBindingCount bindings. A binding out of range, undeclared or named twice is invalid_argument naming the kernel and
// the binding.
[[nodiscard]] inline Result<AdmitLayout> kernel_layout(std::string_view kernel,
                                                      std::span<const uint32_t> set1_bindings) {
    std::vector<bool> named(kSet1BindingSlots, false);
    for (const uint32_t b : set1_bindings) {
        if (b >= kSet1BindingSlots) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "kernel '" + std::string(kernel) + "': set-1 binding " + std::to_string(b) +
                                             " is past set 1's " + std::to_string(kSet1BindingSlots) +
                                             " binding slots (bindings.slang section F, 0-" +
                                             std::to_string(kSet1BindingSlots - 1u) + ")"});
        }
        if (!is_declared_set1(b)) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "kernel '" + std::string(kernel) + "': set-1 binding " + std::to_string(b) +
                                             " is undeclared (bindings.slang section F declares 0-27, " +
                                             std::to_string(kAirflowPointsBinding) + ", " +
                                             std::to_string(kAirflowConfigBinding) + ", " +
                                             std::to_string(kAirflowCentredBinding) + ", " +
                                             std::to_string(kContactShapesBinding) + ", " +
                                             std::to_string(kAirflowLineFirstBinding) + "-" +
                                             std::to_string(kAirflowLineFirstBinding + kAirflowLineBufferCount - 1) + ", " +
                                             std::to_string(kLbmFBinding) + " and " +
                                             std::to_string(kLbmUBinding) + ")"});
        }
        if (named[b]) {
            return std::unexpected(Error{Code::invalid_argument, "kernel '" + std::string(kernel) + "': set-1 binding " +
                                                                     std::to_string(b) + " is named twice"});
        }
        named[b] = true;
    }
    AdmitLayout layout{.name = std::string(kernel), .bindings_per_set = {kStateBindingCount}};
    if (!set1_bindings.empty()) layout.bindings_per_set.push_back(static_cast<uint32_t>(set1_bindings.size()));
    return layout;
}

// A kernel's set-1 subset as the ONE key of its descriptor set layout: sorted, and so independent of the order the recipe
// names the bindings in. Vulkan requires a descriptor set to be allocated from a layout identically defined to the one the
// pipeline layout names, so the mirror builds one set-1 layout per distinct key and the recorder looks a kernel's up by it
// (module API stage 7, Task C2). Duplicates are kernel_layout()'s to refuse, by name, before this is called.
[[nodiscard]] inline std::vector<uint32_t> normalized_set1_subset(std::span<const uint32_t> set1_bindings) {
    std::vector<uint32_t> key(set1_bindings.begin(), set1_bindings.end());
    std::sort(key.begin(), key.end());
    return key;
}

}  // namespace spade::compute
