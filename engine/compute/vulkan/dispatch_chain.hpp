#pragma once
// ---------------------------------------------------------------------------
// dispatch_chain.hpp -- THE ONE RECORD-ONCE DISPATCH CHAIN (the airflow chain
// runner's design note, §1, approved by Core 2026-10-12). Moved out of
// StepRecorder unchanged, so that StepRecorder, plan B's chain runner and
// stage 7's airflow module record a chain the same way:
//
//   * every dispatch binds its pipeline, pushes its constants and dispatches;
//   * ONE whole-memory VkMemoryBarrier (SHADER_WRITE -> SHADER_READ|WRITE,
//     COMPUTE_SHADER -> COMPUTE_SHADER) after every dispatch except the last
//     one in the list -- "which dispatch is last" is read off the list itself
//     (step_recorder.hpp's barrier rule, and its 2026-10-01 history);
//   * a timestamp mark is recorded where the list puts it, through the
//     caller's callback.
//
// A pure move: the recorded command stream is the one StepRecorder recorded
// before (recorded_chain()'s counts and the GPU parity suites pin it).
// ---------------------------------------------------------------------------
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>

#include <volk.h>

#include "compute/spirv_variants.hpp"
#include "core/error.hpp"

namespace spade::compute {

// Vulkan's guaranteed minimum maxPushConstantsSize: every chain's push block fits.
inline constexpr uint32_t kMaxChainPushBytes = 128;

// One entry of a chain: a dispatch, or a timestamp mark.
struct ChainCommand {
    bool is_dispatch = false;
    uint32_t pipeline = 0;                             // dispatch: an index into the chain's pipelines
    std::array<std::byte, kMaxChainPushBytes> push{};  // dispatch: its push constant's bytes
    std::array<uint32_t, 3> groups{0, 0, 0};           // dispatch: its group counts
    uint32_t mark_a = 0;                               // mark: the callback's two arguments
    uint32_t mark_b = 0;
};

struct ChainCounts {
    uint32_t dispatches = 0;
    uint32_t barriers = 0;
};

using ChainMark = std::function<void(VkCommandBuffer, uint32_t, uint32_t)>;

// Records `commands` into `cmd`, which the caller has begun and will end:
// each dispatch binds pipelines[pipeline], pushes `push_size` bytes at
// `push_offset` and dispatches its groups, with the barrier above after every
// dispatch but the last; each mark calls `mark(cmd, mark_a, mark_b)`.
// PRECONDITION: push_offset + push_size <= kMaxChainPushBytes, since each push
// is read from a command's kMaxChainPushBytes block (asserted; a caller with a
// run-time push size refuses a larger one by name first).
ChainCounts record_dispatch_chain(VkCommandBuffer cmd, VkPipelineLayout layout, std::span<const VkPipeline> pipelines,
                                  std::span<const ChainCommand> commands, uint32_t push_offset, uint32_t push_size,
                                  const ChainMark& mark);

// One VkShaderModule + compute VkPipeline from the variant of `variants`
// compiled for `workgroup_size`, on `layout`. Refused (invalid_argument,
// naming `who` and the kernel) when no such variant was compiled: a module
// dispatched on a grid divided by another size gives wrong physics silently.
[[nodiscard]] Result<void> make_compute_pipeline(VkDevice device, VkPipelineLayout layout,
                                                 const SpirvVariantSet& variants, uint32_t workgroup_size,
                                                 const char* who, VkShaderModule& out_module,
                                                 VkPipeline& out_pipeline);

}  // namespace spade::compute
