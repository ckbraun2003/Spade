#pragma once

// ---------------------------------------------------------------------------
// state_mirror.hpp (S6 Task 5) -- the device buffer table: one storage buffer
// (device-local) plus one staging buffer (host-visible, persistently mapped)
// per entry of the FULL registered walk (state/arenas.hpp's
// ArenaSet::registry(): the nine registered arrays plus their nine
// `.slot_to_world` siblings, 18 entries, global constraint: "Registered state
// is FROZEN at 18 walk entries"), plus one more buffer for dryden_params
// (bindings.slang binding 13 -- derived, backend-internal, not part of the
// registered walk).
//
// NO register_array CALL ANYWHERE IN THIS FILE. Every buffer here is
// backend-internal derived storage: a device-side MIRROR of state the
// ArenaSet already owns and registers, never a second place that state
// lives authoritatively. Upload copies FROM the arena walk TO these buffers;
// readback copies back. The arena remains the one authoritative CPU-side
// store, exactly as it is on the cpu backend.
//
// GENERIC OVER THE WALK, BY NAME, NOT BY C++ TYPE. This class does not
// `#include` a single one of the nine arrays' element types (BodyState,
// WorldParams, DragBodyRow, ...) and does not need to: state/registry.hpp's
// RegisteredArray already carries everything a byte-copying mirror needs
// (name, elem_size, world_count, capacity_per_world, a data pointer), and
// ArenaSet::registry().for_each_array() is the walk itself. The one place
// this class DOES need to know a size ahead of an ArenaSet existing is at
// create() time, sizing buffers from StepShape before any Simulation has
// registered anything -- see state_mirror.cpp's kArrayShapes table for that
// one necessary exception, and its own comment for why the sizes there are
// safe to hardcode (upload()/readback() cross-check every one of them
// against the live registry on every call, so a drift is a returned
// invalid_argument, never a silent truncation).
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <volk.h>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/error.hpp"
#include "state/arenas.hpp"

namespace spade::compute {

class StateMirror {
public:
    [[nodiscard]] static Result<std::unique_ptr<StateMirror>> create(VulkanContext& ctx,
                                                                       const StepShape& shape);

    ~StateMirror();
    StateMirror(const StateMirror&) = delete;
    StateMirror& operator=(const StateMirror&) = delete;
    StateMirror(StateMirror&&) noexcept;
    StateMirror& operator=(StateMirror&&) noexcept;

    // Copies every entry of `arenas`' registered walk into its matching
    // device buffer, staging through host-visible memory. invalid_argument
    // if the walk's shape (name set, elem_size, world_count,
    // capacity_per_world -- in that check order) does not match what this
    // mirror was created for.
    [[nodiscard]] Result<void> upload(const ArenaSet& arenas);

    // The reverse of upload(): device buffers -> `arenas`. Same shape check,
    // same failure taxonomy. On any failure, no destination byte for the
    // array that failed (or any array after it in the walk) is written --
    // see backend.hpp's readback() doc comment for why that is load-bearing.
    [[nodiscard]] Result<void> readback(ArenaSet& arenas);

    // dryden_params (binding 13): raw bytes, `elem_size` per world,
    // `world_count` worlds -- see backend.hpp's upload_dryden_params() for
    // why this takes raw bytes rather than a concrete DrydenParams span (the
    // same "generic over the walk" reasoning as upload()/readback() above,
    // extended to the one buffer that isn't part of the walk at all).
    [[nodiscard]] Result<void> upload_dryden_params(std::span<const std::byte> params_bytes,
                                                      uint32_t elem_size, uint32_t world_count);

    // -----------------------------------------------------------------------
    // S6 Task 6's three additional DERIVED uploads. Same category and same
    // raw-bytes shape as upload_dryden_params() above, for its reasons: none
    // is a registered array (no register_array call is added anywhere), each
    // is per-run configuration the host owns and the device only reads, and
    // taking bytes keeps this class from needing a single one of the concrete
    // element types on its include path.
    //
    // upload_sdf_program() takes all THREE SDF buffers in one call rather than
    // three: they are one object (the flattened program set -- see
    // compute/sdf_program.hpp), the ranges buffer indexes into the other two,
    // and a partial upload would leave the device holding a self-inconsistent
    // program. One call, one shape check, one copy batch.
    // -----------------------------------------------------------------------
    [[nodiscard]] Result<void> upload_sdf_program(std::span<const std::byte> node_bytes,
                                                    std::span<const std::byte> transform_bytes,
                                                    std::span<const std::byte> range_bytes);

    [[nodiscard]] Result<void> upload_contact_params(std::span<const std::byte> params_bytes,
                                                       uint32_t elem_size, uint32_t world_count);

    // The PER-WORLD broad-phase grid config (binding 20, S6 Task 6b -- the
    // checkpoint-1 heterogeneous-worlds ruling). Same shape and same
    // reasoning as upload_contact_params() immediately above: derived,
    // backend-internal, one GridParams per world, dead until T7's
    // CollisionDynamic kernel reads it.
    [[nodiscard]] Result<void> upload_grid_params(std::span<const std::byte> params_bytes, uint32_t elem_size,
                                                    uint32_t world_count);

    // The persistently-mapped host-visible StepParams buffer (binding 17).
    //
    // OWNED HERE, NOT BY StepRecorder, as of this task -- and the move is what
    // makes the buffer bindable at all. A descriptor set is written once, at
    // create() time, from buffers that must already exist; StateMirror::create()
    // is where that happens, so a buffer StepRecorder created afterwards could
    // never appear in the set. StepRecorder still WRITES it (once per submit,
    // through this pointer) and still owns the tick bookkeeping -- only the
    // allocation moved. Never null after a successful create().
    [[nodiscard]] void* step_params_mapped() const noexcept { return step_params_.staging_mapped; }

    // The StepWitness row (binding 18) the Integrate kernel publishes into --
    // copied device->host and returned as raw bytes, the reverse of the
    // uploads above. `out_bytes` must be exactly sizeof(StepWitness); this
    // class deliberately does not name the type (see the header note on being
    // generic over the walk).
    [[nodiscard]] Result<void> read_step_witness(std::span<std::byte> out_bytes);

    // The CollisionDynamic key array (binding 21, S6 Task 7) as the last
    // CollisionDynamic pass left it -- i.e. SORTED, since the sweep is the
    // last dispatch of the chain and never reorders. Same shape and same
    // "raw bytes, name no type" reasoning as read_step_witness() above;
    // `out_bytes` must be exactly grid_entries_byte_size().
    //
    // A DIAGNOSTIC, NOT A STEP-PATH CALL. It costs one device->host copy and
    // one queue submit, and nothing in the engine invokes it: it exists so
    // tests/test_gpu_parity.cpp's GpuGridSort cases can compare the device's
    // sorted keys against std::sort + physics::grid_entry_less over the same
    // bodies -- the one instrument that checks the sort NETWORK rather than
    // the physics it feeds.
    [[nodiscard]] Result<void> read_grid_entries(std::span<std::byte> out_bytes);

    // Bytes in that buffer, so a caller can size its own span without
    // recomputing grid_domain_of().
    [[nodiscard]] std::size_t grid_entries_byte_size() const noexcept {
        return static_cast<std::size_t>(grid_entries_.byte_size);
    }

    [[nodiscard]] VkDescriptorSetLayout descriptor_set_layout() const noexcept { return set_layout_; }
    [[nodiscard]] VkDescriptorSet descriptor_set() const noexcept { return set_; }

    [[nodiscard]] uint64_t upload_count() const noexcept { return upload_count_; }

private:
    StateMirror() = default;
    void destroy() noexcept;

    // One registered-walk entry's device+staging buffer pair, plus the
    // shader binding it occupies when it has one (five of the 18 -- the
    // `.slot_to_world` siblings bindings.slang's section B documents as
    // deliberately unbound -- do not; see kNoBinding).
    struct Entry {
        std::string name;
        uint32_t elem_size = 0;
        uint32_t world_count = 0;
        uint32_t capacity_per_world = 0;
        VkDeviceSize byte_size = 0;
        VkBuffer device_buffer = VK_NULL_HANDLE;
        VkDeviceMemory device_memory = VK_NULL_HANDLE;
        VkBuffer staging_buffer = VK_NULL_HANDLE;
        VkDeviceMemory staging_memory = VK_NULL_HANDLE;
        void* staging_mapped = nullptr;
        uint32_t binding = 0;
        bool has_binding = false;
    };

    [[nodiscard]] Entry* find(const std::string& name) noexcept;
    [[nodiscard]] const Entry* find(const std::string& name) const noexcept;

    // Records and submits ONE command buffer's worth of vkCmdCopyBuffer
    // calls, one per entry in `targets` (staging<->device, direction per
    // `to_device`), and blocks until it completes. A private static member
    // (not a free function in state_mirror.cpp's anonymous namespace) purely
    // because it takes `Entry*` -- a private nested type free functions
    // outside this class cannot name.
    [[nodiscard]] static Result<void> run_copy_batch(VkDevice device, VkCommandBuffer cmd, VkQueue queue,
                                                        VkFence fence, bool to_device,
                                                        const std::vector<Entry*>& targets);

    // Unmaps/destroys/frees one Entry's buffers+memory; safe on a
    // partially-constructed or already-destroyed Entry (every handle it
    // touches is individually VK_NULL_HANDLE-guarded). Same "must be a
    // member, not a free function" reason as run_copy_batch above.
    static void destroy_entry(VkDevice device, Entry& e) noexcept;

    VkDevice device_ = VK_NULL_HANDLE;               // non-owning
    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;  // non-owning
    VkQueue queue_ = VK_NULL_HANDLE;                 // non-owning; transfer queue
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;

    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
    VkDescriptorPool desc_pool_ = VK_NULL_HANDLE;
    VkDescriptorSet set_ = VK_NULL_HANDLE;

    std::vector<Entry> entries_;  // the 18-entry registered walk mirror

    // The DERIVED buffers -- none is part of the registered walk, all are
    // backend-internal (bindings.slang sections C and C'). Declared as Entry
    // values rather than pushed into `entries_` so that upload()/readback()'s
    // walk-shaped loops, which cross-check every element against the LIVE
    // ArenaSet registry, cannot accidentally reach one that has no registry
    // counterpart to be checked against.
    Entry dryden_params_;    // binding 13
    Entry sdf_nodes_;        // binding 14
    Entry sdf_transforms_;   // binding 15
    Entry sdf_ranges_;       // binding 16
    Entry step_params_;      // binding 17 -- HOST-VISIBLE device buffer, see below
    Entry step_witness_;     // binding 18 -- device-written, read back on demand
    Entry contact_params_;   // binding 19
    Entry grid_params_;      // binding 20 (S6 Task 6b)

    // binding 21 (S6 Task 7) -- the dynamic broad phase's key array. The one
    // derived buffer that is neither uploaded nor part of a round trip: the
    // device builds it, sorts it and consumes it entirely within one
    // CollisionDynamic pass, so the host only ever zero-fills it at create()
    // (so a pass that somehow read it before writing would see the defined
    // sentinel-free zero rather than driver garbage) and copies it out for the
    // GpuGridSort diagnostic. Sized from compute/grid_entry.hpp's
    // grid_domain_of(shape) -- the SAME function StepRecorder sizes its
    // dispatch grids from, so the two cannot disagree about how big the domain
    // is.
    Entry grid_entries_;

    // step_params_ IS THE ONE ENTRY WITH NO DEVICE HALF. Every other buffer
    // here is device-local with a host-visible staging partner and an explicit
    // copy between them; step_params_ has only the staging half -- a
    // HOST_VISIBLE|HOST_COHERENT buffer, persistently mapped -- and it is that
    // buffer the descriptor set binds. The reason is the access pattern, not
    // convenience: the host rewrites its eight bytes immediately before EVERY
    // vkQueueSubmit, so routing them through a staging copy would mean a
    // transfer command buffer and a second submit per tick to move two words,
    // and the whole point of the record-once design is that a step costs one
    // submit. `device_buffer`/`device_memory` stay VK_NULL_HANDLE for it;
    // destroy_entry() is individually handle-guarded, so it tears down
    // correctly regardless of which half is present.
    uint64_t upload_count_ = 0;
};

}  // namespace spade::compute
