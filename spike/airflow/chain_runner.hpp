#pragma once
// ---------------------------------------------------------------------------
// chain_runner.hpp -- THE AIRFLOW CHAIN RUNNER (plan B, task B1;
// docs/design/physics/plans/2026-10-12-airflow-chain-runner-design.md §2).
//
// A spike and bench harness in its own target, spade_airflow_runner (G1): not
// linked into Simulation, no StepShape or mirror change, no installed header.
// Plan B's parity tests, its benchmark and the sandbox's demonstration harness
// use it; plan C retires it, except as a bench (design §5).
//
// WHAT IT DOES. It owns a list of device-local storage buffers and a list of
// compute pipelines, and records chains of dispatches through Core's
// dispatch_chain (G2): a chain is recorded ONCE into its own command buffer and
// resubmitted, so a fluid step is one chain and publish is a second.
//
//   * Buffer i binds at set 1, the airflow set, binding binding_base + i
//     (Core's B0 plan, decision 1: the airflow block is its own descriptor
//     set, so set 0, the mirror and StepShape are untouched). The pipeline
//     layout is an EMPTY set 0 plus the runner's set 1; only set 1 is bound.
//     B0's named block in bindings.slang takes over the kernels' declarations;
//     binding_base stays 0 unless it moves.
//   * Pipeline i is kernels[i], compiled for `workgroup_size`, all on one
//     layout with one push range of `push_bytes` at offset 0.
//   * Every buffer is device-local. One host-visible staging buffer, the size
//     of the largest buffer that can cross to the host, carries every
//     upload() and read(); each is a fenced copy.
//
// REFUSALS BY NAME (L6). create() and record() refuse a request past a device
// limit or the budget with a message naming the limit and the value asked
// for, and they refuse BEFORE ANY ALLOCATION: no buffer, memory, pipeline or
// command buffer exists when they return the error.
//
// ONE QUEUE (A5). The runner submits on the context's compute queue and waits
// on its own fence after every submit. It is single-threaded; a caller that
// shares the context with a Vulkan Simulation serializes the two's submits.
// It has its own descriptor pool and command pool, never the backend's.
// ---------------------------------------------------------------------------
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <volk.h>

#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "compute/vulkan/timestamps.hpp"
#include "core/error.hpp"

namespace spade::airflow_gpu {

// The airflow descriptor set (Core's B0 plan): set 0 stays the state registry's.
inline constexpr uint32_t kAirflowSet = 1;

// What a buffer is, as stage 7's mirror will see it (design §2; stage 7 design
// §2, §7.3), so the list maps one-to-one onto ScratchShape and the resident
// arrays.
enum class BufferKind : uint8_t {
    device_only,  // scratch: written on the device before it is read; never crosses to the host
    resident,     // state that lives on the device, synced on demand
    staged,       // crosses to the host: inputs, published outputs, diagnostics
};

struct RunnerBuffer {
    std::string name{};
    uint64_t bytes = 0;
    BufferKind kind = BufferKind::device_only;
};

struct RunnerDesc {
    std::span<const RunnerBuffer> buffers{};                     // set 1, binding binding_base + i = buffers[i]
    std::span<const compute::SpirvVariantSet* const> kernels{};  // pipeline i = kernels[i]
    uint32_t push_bytes = 0;                                     // one push range, every kernel; a multiple of 4
    uint32_t workgroup_size = 64;                                // the variant every kernel is built from
    uint32_t binding_base = 0;                                   // B0's airflow block start
};

// The device limits the runner refuses against, read once from the context's
// physical device. max_memory_allocation_size is VkPhysicalDeviceMaintenance3-
// Properties' (Vulkan 1.1, which every admitted device has).
struct RunnerLimits {
    uint64_t max_storage_buffer_range = 0;
    uint64_t max_memory_allocation_size = 0;
    uint32_t max_per_stage_storage_buffers = 0;
    uint32_t max_set_storage_buffers = 0;
    uint32_t max_push_constants_size = 0;
    std::array<uint32_t, 3> max_group_count{};
    uint32_t max_group_size_x = 0;
    uint32_t max_group_invocations = 0;
};

[[nodiscard]] RunnerLimits read_runner_limits(const compute::VulkanContext& ctx);

// The bytes a runner may hold on the device, in total: its buffers plus its
// staging buffer. Until Core's VulkanContext::memory_budget() lands (B0) it is
// the 4 GiB airflow cap, kGridByteCap. B0 replaces this body with
// min(4 GiB, budget - usage - 1 GiB reserve) (plan B, the lead's decision 4).
[[nodiscard]] uint64_t runner_budget_bytes(compute::VulkanContext& ctx);

// What every runner in this process has allocated through vkAllocateMemory:
// the bytes held now, and the count of allocations ever made. The count only
// grows, so a test reads it either side of a refused create() to show the
// refusal came before any allocation. (The half-heap budget fallback, A2,
// subtracts live_bytes.)
struct RunnerMemoryStats {
    uint64_t live_bytes = 0;
    uint64_t allocations = 0;
};

[[nodiscard]] RunnerMemoryStats runner_memory_stats() noexcept;

class ChainRunner {
public:
    // Refuses, with Code::capacity_exceeded and the limit's name: a buffer over
    // maxStorageBufferRange or maxMemoryAllocationSize; more buffers than
    // maxPerStageDescriptorStorageBuffers or maxDescriptorSetStorageBuffers;
    // push_bytes over maxPushConstantsSize or kMaxChainPushBytes; a workgroup
    // size over maxComputeWorkGroupSize[0] or maxComputeWorkGroupInvocations;
    // the total over runner_budget_bytes(). With Code::invalid_argument: no
    // buffers, a buffer of 0 bytes, a null kernel, push_bytes 0 or not a
    // multiple of 4, a workgroup size with no compiled variant. All before
    // any allocation.
    [[nodiscard]] static Result<std::unique_ptr<ChainRunner>> create(compute::VulkanContext& ctx,
                                                                     const RunnerDesc& desc);

    // Held only by unique_ptr: neither copyable nor movable.
    ChainRunner(const ChainRunner&) = delete;
    ChainRunner& operator=(const ChainRunner&) = delete;
    ~ChainRunner();

    // Copies `bytes` into the start of buffer `buffer` through the staging
    // buffer, and waits for the copy. Refused for a device_only buffer, an
    // index out of range, or more bytes than the buffer holds. Empty is a
    // no-op.
    [[nodiscard]] Result<void> upload(uint32_t buffer, std::span<const std::byte> bytes);

    // Copies the start of buffer `buffer` into `out`, fenced. Refused as
    // upload() is.
    [[nodiscard]] Result<void> read(uint32_t buffer, std::span<std::byte> out) const;

    // Records `chain` once, into a command buffer of its own, and returns its
    // id. Each dispatch binds its pipeline, pushes the runner's push_bytes of
    // its block and dispatches, with dispatch_chain's barrier after every
    // dispatch but the last. A mark records a timestamp; marks are numbered
    // in the order they appear, and their mark_a/mark_b are not read.
    // Refuses, before anything is allocated: a pipeline index out of range
    // (invalid_argument), groups over maxComputeWorkGroupCount on an axis
    // (capacity_exceeded).
    [[nodiscard]] Result<uint32_t> record(std::span<const compute::ChainCommand> chain);

    // Submits chain `chain` n times, waiting for each.
    [[nodiscard]] Result<void> submit(uint32_t chain, uint64_t n);

    // Per mark of chain `chain`, the nanoseconds since the previous mark (the
    // first: since the chain began), from its most recent submit. Empty when
    // the chain has no marks, has not been submitted, or the device cannot
    // time compute work: never an error for those.
    [[nodiscard]] Result<std::vector<uint64_t>> durations_ns(uint32_t chain) const;

    // What dispatch_chain recorded for chain `chain`: its dispatches and the
    // barriers between them. Zero counts for an unknown id.
    [[nodiscard]] compute::ChainCounts recorded_counts(uint32_t chain) const noexcept;

    [[nodiscard]] uint32_t buffer_count() const noexcept { return static_cast<uint32_t>(buffers_.size()); }
    [[nodiscard]] uint32_t chain_count() const noexcept { return static_cast<uint32_t>(chains_.size()); }

private:
    ChainRunner() = default;
    void destroy() noexcept;
    [[nodiscard]] Result<void> check_transfer(uint32_t buffer, std::size_t bytes, const char* op) const;
    [[nodiscard]] Result<void> run_transfer(bool to_device, uint32_t buffer, std::size_t bytes) const;
    [[nodiscard]] Result<void> submit_and_wait(VkCommandBuffer cmd, const char* op) const;

    struct Buffer {
        std::string name;
        uint64_t bytes = 0;
        BufferKind kind = BufferKind::device_only;
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        uint64_t allocated = 0;  // the allocation's size, for runner_memory_stats()
    };

    struct Chain {
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        std::unique_ptr<compute::PassTimestamps> timestamps;  // null when the chain has no marks
        compute::ChainCounts counts{};
        uint32_t marks = 0;
        uint64_t submits = 0;
    };

    VkDevice device_ = VK_NULL_HANDLE;
    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    compute::VulkanContext* ctx_ = nullptr;  // non-owning; outlives the runner
    uint32_t push_bytes_ = 0;
    uint32_t workgroup_size_ = 0;
    RunnerLimits limits_{};

    std::vector<Buffer> buffers_;
    VkBuffer staging_ = VK_NULL_HANDLE;
    VkDeviceMemory staging_memory_ = VK_NULL_HANDLE;
    uint64_t staging_allocated_ = 0;
    uint64_t staging_bytes_ = 0;
    void* staging_mapped_ = nullptr;

    VkDescriptorSetLayout empty_set_layout_ = VK_NULL_HANDLE;  // set 0: no bindings
    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;        // set 1: the buffers
    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool_ = VK_NULL_HANDLE;
    VkDescriptorSet set_ = VK_NULL_HANDLE;
    std::vector<VkShaderModule> shaders_;
    std::vector<VkPipeline> pipelines_;

    VkCommandPool command_pool_ = VK_NULL_HANDLE;
    VkCommandBuffer transfer_cmd_ = VK_NULL_HANDLE;  // re-recorded for every upload() and read()
    VkFence fence_ = VK_NULL_HANDLE;
    mutable bool in_flight_ = false;  // a fence wait failed: destroy() waits for the device first

    std::vector<Chain> chains_;
};

}  // namespace spade::airflow_gpu
