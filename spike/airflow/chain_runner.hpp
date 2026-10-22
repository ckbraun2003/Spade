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
//     The kernels declare nothing of their own: they import bindings.slang,
//     whose section F names every binding of set 1, and kAirflowSetBuffers
//     below lists the same bindings in order, so buffers[i] is section F's
//     binding i (binding_base 0).
//   * Pipeline i is kernels[i], compiled for `workgroup_size`, all on one
//     layout with one push range of `push_bytes` at offset 0.
//   * Every buffer is device-local. One host-visible staging buffer, the size
//     of the largest buffer that can cross to the host, carries every
//     upload() and read(); each is a fenced copy. Its memory is host-cached
//     where the device has a cached coherent type (kStagingMemoryPreferences).
//
// REFUSALS BY NAME (L6). create() and record() refuse a request past a device
// limit or the budget with a message naming the limit and the value asked
// for, and they refuse BEFORE ANY ALLOCATION: no buffer, memory, pipeline or
// command buffer exists when they return the error. The checks are Core's
// (compute/device_limits.hpp, B0), called and never copied, against the
// limits the context recorded (DeviceReport::limits).
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
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <volk.h>

#include "bindings.gen.hpp"
#include "compute/device_limits.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "compute/vulkan/timestamps.hpp"
#include "core/error.hpp"

namespace spade::airflow_gpu {

// The airflow descriptor set, bindings.slang's "@set 1 airflow" (Core's B0
// plan): set 0 stays the state registry's.
inline constexpr uint32_t kAirflowSet = compute::gen::kSet_airflow;

// What a buffer is, as stage 7's mirror will see it (design §2; stage 7 design
// §2, §7.3), so the list maps one-to-one onto ScratchShape and the resident
// arrays.
enum class BufferKind : uint8_t {
    device_only,  // scratch: written on the device before it is read; never crosses to the host
    resident,     // state that lives on the device, synced on demand
    staged,       // crosses to the host: inputs, published outputs, diagnostics
};

// THE AIRFLOW SET, binding by binding: bindings.slang section F's names, in
// binding order, each with its kind from the confirmed list (Core's B0 plan's
// table, R/D/S), with one change: airflow_advect is staged, not device-only,
// because B6's P8 measurement reads advection's fp32 scratch back (Rendering,
// 2026-10-21). A runner for the airflow set lists its buffers in this order,
// and a test may tag a scratch buffer staged to read it. Append-only, as
// section F is; the static_asserts below pin the order to the generated
// bindings.
struct AirflowSetBuffer {
    std::string_view name;
    uint32_t binding = 0;
    BufferKind kind = BufferKind::device_only;
};

inline constexpr std::array<AirflowSetBuffer, 28> kAirflowSetBuffers{{
    {"airflow_params", compute::gen::kBinding_airflow_params, BufferKind::resident},
    {"airflow_u", compute::gen::kBinding_airflow_u, BufferKind::resident},
    {"airflow_v", compute::gen::kBinding_airflow_v, BufferKind::resident},
    {"airflow_w", compute::gen::kBinding_airflow_w, BufferKind::resident},
    {"airflow_pressure", compute::gen::kBinding_airflow_pressure, BufferKind::resident},
    {"airflow_next", compute::gen::kBinding_airflow_next, BufferKind::device_only},
    {"airflow_advect", compute::gen::kBinding_airflow_advect, BufferKind::staged},
    {"airflow_increments", compute::gen::kBinding_airflow_increments, BufferKind::device_only},
    {"airflow_viscous_rhs", compute::gen::kBinding_airflow_viscous_rhs, BufferKind::device_only},
    {"airflow_rhs", compute::gen::kBinding_airflow_rhs, BufferKind::device_only},
    {"airflow_residual", compute::gen::kBinding_airflow_residual, BufferKind::device_only},
    {"airflow_multigrid", compute::gen::kBinding_airflow_multigrid, BufferKind::device_only},
    {"airflow_mg_weights", compute::gen::kBinding_airflow_mg_weights, BufferKind::resident},
    {"airflow_apertures", compute::gen::kBinding_airflow_apertures, BufferKind::resident},
    {"airflow_wall_velocity", compute::gen::kBinding_airflow_wall_velocity, BufferKind::resident},
    {"airflow_phi", compute::gen::kBinding_airflow_phi, BufferKind::resident},
    {"airflow_face_tags", compute::gen::kBinding_airflow_face_tags, BufferKind::resident},
    {"airflow_wall_links", compute::gen::kBinding_airflow_wall_links, BufferKind::resident},
    {"airflow_region_labels", compute::gen::kBinding_airflow_region_labels, BufferKind::resident},
    {"airflow_region_table", compute::gen::kBinding_airflow_region_table, BufferKind::device_only},
    {"airflow_pcg", compute::gen::kBinding_airflow_pcg, BufferKind::device_only},
    {"airflow_scalars", compute::gen::kBinding_airflow_scalars, BufferKind::device_only},
    {"airflow_discs", compute::gen::kBinding_airflow_discs, BufferKind::resident},
    {"airflow_partials", compute::gen::kBinding_airflow_partials, BufferKind::device_only},
    {"airflow_health", compute::gen::kBinding_airflow_health, BufferKind::staged},
    {"airflow_publish_v", compute::gen::kBinding_airflow_publish_v, BufferKind::staged},
    {"airflow_publish_s", compute::gen::kBinding_airflow_publish_s, BufferKind::staged},
    {"airflow_wall_load", compute::gen::kBinding_airflow_wall_load, BufferKind::staged},
}};

static_assert(kAirflowSetBuffers.size() == compute::gen::kBindingCount_airflow,
              "kAirflowSetBuffers lists every binding of bindings.slang section F");
static_assert(
    [] {
        for (uint32_t i = 0; i < kAirflowSetBuffers.size(); ++i) {
            if (kAirflowSetBuffers[i].binding != i) return false;
        }
        return true;
    }(),
    "kAirflowSetBuffers is in binding order: entry i is section F's binding i");

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
    uint32_t binding_base = 0;                                   // section F starts at binding 0
};

// The VRAM a runner leaves for everything else on the device, the GL
// renderer's textures first: 1 GiB until B7 measures the renderer during the
// D1 film (plan B, the lead's decision 4).
inline constexpr uint64_t kRunnerReserveBytes = uint64_t{1} << 30;

// The bytes a runner may hold on the device, in total: its buffers plus its
// staging buffer. min(4 GiB, budget - usage - kRunnerReserveBytes), from
// VulkanContext::memory_budget() and Core's vram_cap() (B0; plan B, the
// lead's decision 4; design A1/A2). With the half-heap fallback the usage is
// what this process's runners hold (runner_memory_stats().live_bytes); with
// VK_EXT_memory_budget the heap's usage already counts it. Read fresh at each
// call: the budget moves as other programs allocate.
[[nodiscard]] uint64_t runner_budget_bytes(compute::VulkanContext& ctx);

// THE STAGING BUFFER'S MEMORY (Rendering's finding, 2026-10-21: a read from
// uncached host memory ran at about 230 MB/s, 808 ms for 184 MiB on the 3060
// Ti, against 19 ms cached). In order of preference: host-visible, coherent
// and cached; then host-visible and coherent, which Vulkan guarantees. Never
// a cached type that is not coherent, so no read needs
// vkInvalidateMappedMemoryRanges and no upload a flush.
inline constexpr std::array<VkMemoryPropertyFlags, 2> kStagingMemoryPreferences{
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
};

// The first memory type, in the device's own order, that `type_bits` allows
// and that has every flag of the earliest preference any allowed type meets.
// nullopt when none meets any. Pure, so it tests without a device.
[[nodiscard]] std::optional<uint32_t> choose_memory_type(const VkPhysicalDeviceMemoryProperties& props,
                                                         uint32_t type_bits,
                                                         std::span<const VkMemoryPropertyFlags> preferences) noexcept;

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
    // Refuses, with Code::capacity_exceeded and the limit's name (Core's
    // checks, compute/device_limits.hpp): a buffer over maxStorageBufferRange
    // or maxMemoryAllocationSize; more buffers than
    // maxPerStageDescriptorStorageBuffers or maxDescriptorSetStorageBuffers;
    // push_bytes over maxPushConstantsSize, or over kMaxChainPushBytes (the
    // runner's own); a workgroup size over maxComputeWorkGroupSize[0] or
    // maxComputeWorkGroupInvocations; the total over runner_budget_bytes()
    // (check_vram). With Code::invalid_argument: no buffers, a buffer of 0
    // bytes, a null kernel, push_bytes 0 or not a multiple of 4, a workgroup
    // size with no compiled variant. All before any allocation.
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

    // Whether the staging buffer's memory is host-cached (the first of
    // kStagingMemoryPreferences). False when no buffer crosses to the host.
    [[nodiscard]] bool staging_cached() const noexcept { return staging_cached_; }

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
    compute::DeviceLimits limits_{};
    std::string device_name_;

    std::vector<Buffer> buffers_;
    VkBuffer staging_ = VK_NULL_HANDLE;
    VkDeviceMemory staging_memory_ = VK_NULL_HANDLE;
    uint64_t staging_allocated_ = 0;
    uint64_t staging_bytes_ = 0;
    void* staging_mapped_ = nullptr;
    bool staging_cached_ = false;

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
