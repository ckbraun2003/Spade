#include "compute/vulkan/state_mirror.hpp"

#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

#include "bindings.gen.hpp"

#include "compute/grid_entry.hpp"
#include "compute/sdf_program.hpp"
#include "compute/step_params.hpp"
#include "physics/contacts.hpp"
#include "physics/field_row.hpp"
#include "physics/grid.hpp"
#include "world/medium.hpp"

namespace spade::compute {

// NO `namespace gen = spade::compute::gen;` ALIAS HERE, unlike
// tests/test_slang_layouts.cpp's identical-looking line: that file is at
// GLOBAL scope, where `gen` is a genuinely new name. This file is already
// INSIDE namespace spade::compute, where `gen` (bindings.gen.hpp's
// `namespace spade::compute::gen { ... }`) is already reachable as a nested
// namespace via ordinary unqualified lookup -- `gen::kBinding_...` below
// resolves directly. Declaring an alias of the identical name here is a
// redeclaration MSVC rejects (C2386), caught by this task's own build.

namespace {

// ---------------------------------------------------------------------------
// Vulkan VkResult -> spade::Error, the one taxonomy every fault path in this
// file (and step_recorder.cpp/backend.cpp) routes through -- spec section 11:
// "VK_ERROR_DEVICE_LOST and allocation failure map to distinct spade::Error
// codes". Deliberately reuses the EXISTING Code enum (core/error.hpp) rather
// than adding a new value: capacity_exceeded already means "the request does
// not fit" everywhere else in this tree (ArenaSet::register_array,
// world_set.hpp's validate_world_set), and an out-of-memory Vulkan allocation
// is exactly that fact reported one layer down. internal is the existing
// catch-all for "the backend broke", which device-lost and any other
// unmapped VkResult both are.
// ---------------------------------------------------------------------------
[[nodiscard]] Error map_vk_error(VkResult result, std::string_view op) {
    switch (result) {
        case VK_ERROR_DEVICE_LOST:
            return Error{Code::internal, "Vulkan device lost during " + std::string(op)};
        case VK_ERROR_OUT_OF_DEVICE_MEMORY:
        case VK_ERROR_OUT_OF_HOST_MEMORY:
            return Error{Code::capacity_exceeded,
                         "Vulkan allocation failed (VkResult " + std::to_string(static_cast<int>(result)) +
                             ") during " + std::string(op)};
        default:
            return Error{Code::internal, "Vulkan call failed (VkResult " +
                                              std::to_string(static_cast<int>(result)) + ") during " +
                                              std::string(op)};
    }
}

[[nodiscard]] Result<uint32_t> find_memory_type(VkPhysicalDevice phys, uint32_t type_bits,
                                                 VkMemoryPropertyFlags want) {
    VkPhysicalDeviceMemoryProperties props{};
    vkGetPhysicalDeviceMemoryProperties(phys, &props);
    for (uint32_t i = 0; i < props.memoryTypeCount; ++i) {
        if ((type_bits & (1u << i)) && (props.memoryTypes[i].propertyFlags & want) == want) {
            return i;
        }
    }
    return std::unexpected(Error{Code::internal, "no Vulkan memory type satisfies the requested properties"});
}

// Creates one buffer + backing memory, optionally persistently mapped
// (staging buffers only -- device-local memory is not host-visible on a
// discrete GPU and need not be mapped on this box's integrated one either).
// On ANY failure, tears down whatever this call itself created before
// returning -- the caller's own destroy() only has to handle buffers it
// successfully finished creating, matching every other RAII owner in this
// tree (VulkanContext::destroy()'s "idempotent, safe on partial state" note).
[[nodiscard]] Result<void> create_buffer(VkDevice device, VkPhysicalDevice phys, VkDeviceSize size,
                                          VkBufferUsageFlags usage, VkMemoryPropertyFlags mem_props, bool map,
                                          VkBuffer& out_buffer, VkDeviceMemory& out_memory, void*& out_mapped) {
    out_buffer = VK_NULL_HANDLE;
    out_memory = VK_NULL_HANDLE;
    out_mapped = nullptr;

    // Vulkan forbids a zero-size buffer (VUID-VkBufferCreateInfo-size); a
    // registered array can legitimately have zero capacity (an unused
    // sensor/element budget), so this mirror floors every allocation at a
    // trivial 4 bytes. upload()/readback() never copy more than the
    // logical, possibly-zero byte_size the Entry itself records, so the
    // floor is purely to keep every descriptor slot backed by a valid
    // buffer -- it never changes what gets copied.
    const VkDeviceSize alloc_size = size == 0 ? VkDeviceSize{4} : size;

    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = alloc_size;
    buffer_info.usage = usage;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VkResult r = vkCreateBuffer(device, &buffer_info, nullptr, &out_buffer);
    if (r != VK_SUCCESS) {
        out_buffer = VK_NULL_HANDLE;
        return std::unexpected(map_vk_error(r, "vkCreateBuffer"));
    }

    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, out_buffer, &req);

    Result<uint32_t> type = find_memory_type(phys, req.memoryTypeBits, mem_props);
    if (!type) {
        vkDestroyBuffer(device, out_buffer, nullptr);
        out_buffer = VK_NULL_HANDLE;
        return std::unexpected(type.error());
    }

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = req.size;
    alloc_info.memoryTypeIndex = *type;

    r = vkAllocateMemory(device, &alloc_info, nullptr, &out_memory);
    if (r != VK_SUCCESS) {
        vkDestroyBuffer(device, out_buffer, nullptr);
        out_buffer = VK_NULL_HANDLE;
        out_memory = VK_NULL_HANDLE;
        return std::unexpected(map_vk_error(r, "vkAllocateMemory"));
    }

    r = vkBindBufferMemory(device, out_buffer, out_memory, 0);
    if (r != VK_SUCCESS) {
        vkFreeMemory(device, out_memory, nullptr);
        vkDestroyBuffer(device, out_buffer, nullptr);
        out_buffer = VK_NULL_HANDLE;
        out_memory = VK_NULL_HANDLE;
        return std::unexpected(map_vk_error(r, "vkBindBufferMemory"));
    }

    if (map) {
        r = vkMapMemory(device, out_memory, 0, alloc_size, 0, &out_mapped);
        if (r != VK_SUCCESS) {
            vkFreeMemory(device, out_memory, nullptr);
            vkDestroyBuffer(device, out_buffer, nullptr);
            out_buffer = VK_NULL_HANDLE;
            out_memory = VK_NULL_HANDLE;
            out_mapped = nullptr;
            return std::unexpected(map_vk_error(r, "vkMapMemory"));
        }
    }

    return {};
}

// ---------------------------------------------------------------------------
// ONE BUFFER'S SIZE, COUNTED IN 64 BITS (module-API stage 4). `count` is the
// buffer's element count: world_count * capacity_per_world for a walk entry,
// the element count for a derived buffer -- a product the caller takes in 64
// bits, because a uint32 product wraps (2^16 worlds of 2^16 bodies is 2^32
// slots, which wraps to 0).
//
// Refused with capacity_exceeded, naming the buffer, when the count exceeds
// 2^32-1: a slot index is a uint32 everywhere -- in the arena
// (ArenaSet::register_array() refuses the same product), in every kernel, and
// in Entry's own fields -- so a larger buffer could not be addressed. Below
// that, the byte size cannot wrap: both factors are under 2^32, so the product
// fits VkDeviceSize and the size_t a host copy takes (64 bits, asserted).
// Whether the device can ALLOCATE it is still the driver's answer, and
// tests/test_gpu_state_mirror.cpp's absurd-shape fault path relies on that
// answer being asked.
// ---------------------------------------------------------------------------
static_assert(sizeof(VkDeviceSize) >= 8 && sizeof(std::size_t) >= 8,
              "buffer_bytes() assumes a 64-bit byte count (uint32 elements * uint32 stride)");

[[nodiscard]] Result<VkDeviceSize> buffer_bytes(std::string_view name, uint32_t elem_size, uint64_t count) {
    if (count > std::numeric_limits<uint32_t>::max()) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "StateMirror::create: buffer '" + std::string(name) + "' would hold " +
                                         std::to_string(count) +
                                         " elements, past what a uint32 slot index addresses (2^32-1)"});
    }
    return static_cast<VkDeviceSize>(count) * static_cast<VkDeviceSize>(elem_size);
}

// Binding index for the 16 of the standard walk's 22 entries that
// bindings.slang binds (the eleven registered arrays plus the five
// `.slot_to_world` siblings a kernel dispatched over a global slot space needs
// -- bindings.slang sections B and E). Returns false for every other entry: the
// standard walk's other six siblings, and any module array no kernel reads (a
// developer's, say, and its map).
//
// EVERY ONE OF THEM IS STILL ALLOCATED, UPLOADED AND READ BACK -- they are simply
// never written into the descriptor set. The per-entry has_binding flag below is
// what makes REGISTERED and BOUND separable. The buffers come from the shape's
// walk (StepShape::arrays); only the bindings are by name here, because they
// belong to the generated registry (bindings.gen.hpp).
//
// THE GNSS ARRAYS WERE IN THAT UNBOUND SET FOR EXACTLY ONE LEG. While no kernel
// read them, binding them would have been a descriptor slot and two Slang
// mirrors spent on buffers nothing could read. sensor_gnss.slang is that
// kernel, so they are bound now -- and the earlier note here predicted the
// change would be "a two-line change plus the Slang mirrors it would then need
// for a reason", which is what it turned out to be.
[[nodiscard]] bool binding_for(const std::string& name, uint32_t& out) {
    static const std::pair<const char*, uint32_t> kTable[] = {
        {"world_params", gen::kBinding_world_params},
        {"bodies", gen::kBinding_bodies},
        {"body_generation", gen::kBinding_body_generation},
        {"drag_bodies", gen::kBinding_drag_bodies},
        {"dryden", gen::kBinding_dryden},
        {"imu_sensors", gen::kBinding_imu_sensors},
        {"imu_ring", gen::kBinding_imu_ring},
        {"rotors", gen::kBinding_rotors},
        {"replay_config", gen::kBinding_replay_config},
        {"bodies.slot_to_world", gen::kBinding_bodies_slot_to_world},
        {"drag_bodies.slot_to_world", gen::kBinding_drag_bodies_slot_to_world},
        {"imu_sensors.slot_to_world", gen::kBinding_imu_sensors_slot_to_world},
        {"rotors.slot_to_world", gen::kBinding_rotors_slot_to_world},
        // The second sensor kind. Its slot->world sibling IS bound, on
        // bindings.slang section B's rule -- sensor_gnss.slang dispatches over
        // the GNSS slot space and reads the map to find the owning world.
        // gnss_ring's own sibling is not: nothing dispatches over a ring.
        {"gnss_sensors", gen::kBinding_gnss_sensors},
        {"gnss_ring", gen::kBinding_gnss_ring},
        {"gnss_sensors.slot_to_world", gen::kBinding_gnss_sensors_slot_to_world},
    };
    for (const auto& [candidate, binding] : kTable) {
        if (name == candidate) {
            out = binding;
            return true;
        }
    }
    return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// create
// ---------------------------------------------------------------------------

Result<std::unique_ptr<StateMirror>> StateMirror::create(VulkanContext& ctx, const StepShape& shape) {
    // -----------------------------------------------------------------------
    // THE REGISTERED WALK IS THE SHAPE'S (module-API stage 4): one buffer per
    // StepShape::arrays entry, which Simulation::create() fills from the
    // module set's declarations (sim/simulation.hpp's state_array_shapes()).
    // This file keeps no list of the walk's arrays or their sizes; binding_for()
    // above names only the generated registry's bindings. An empty list is
    // refused rather than read as "no state": every Simulation registers at
    // least the core's four arrays, so an empty walk means a caller that never
    // said what it was.
    //
    // Every entry is sized BEFORE anything is created, so a shape the mirror
    // cannot size is refused by name with nothing to tear down.
    // -----------------------------------------------------------------------
    if (shape.arrays.empty()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::create: StepShape::arrays is empty; it carries the registered "
                                     "walk, one entry per device buffer (sim/simulation.hpp's "
                                     "state_array_shapes())"});
    }
    std::vector<VkDeviceSize> walk_bytes;
    walk_bytes.reserve(shape.arrays.size());
    for (const StateArrayShape& array : shape.arrays) {
        Result<VkDeviceSize> bytes = buffer_bytes(
            array.name, array.elem_size, uint64_t{shape.world_count} * uint64_t{array.capacity_per_world});
        if (!bytes) return std::unexpected(bytes.error());
        walk_bytes.push_back(*bytes);
    }

    auto self = std::unique_ptr<StateMirror>(new StateMirror());
    self->device_ = ctx.device();
    self->physical_device_ = ctx.physical_device();
    self->queue_ = ctx.transfer_queue();

    VkCommandPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool_info.queueFamilyIndex = ctx.transfer_queue_family();
    if (VkResult r = vkCreateCommandPool(self->device_, &pool_info, nullptr, &self->pool_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateCommandPool (StateMirror)"));
    }

    VkCommandBufferAllocateInfo cmd_info{};
    cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmd_info.commandPool = self->pool_;
    cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmd_info.commandBufferCount = 1;
    if (VkResult r = vkAllocateCommandBuffers(self->device_, &cmd_info, &self->cmd_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkAllocateCommandBuffers (StateMirror)"));
    }

    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (VkResult r = vkCreateFence(self->device_, &fence_info, nullptr, &self->fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateFence (StateMirror)"));
    }

    // -----------------------------------------------------------------------
    // The registered walk: one entry per StepShape::arrays element, in its
    // order -- each array and then its `.slot_to_world` map, as
    // state/arenas.hpp's register_array() contributes them. The order is not
    // itself a contract (upload() and readback() find each entry by NAME), but
    // keeping the walk's lets a reader check the two against each other.
    // -----------------------------------------------------------------------
    // NO LEAK ON A MID-LOOP FAILURE: each Entry is push_back()ed onto
    // self->entries_ the moment its NAME/SHAPE is known, BEFORE either of
    // its two buffers is created, and every field this loop fills in after
    // that reaches the vector element directly (never a separate local that
    // could go out of scope still owning a live handle nothing will free).
    // If create_buffer() fails partway -- the absurd-size fault-path test
    // relies on exactly this -- self->entries_ already holds every handle
    // successfully created so far, and the caller's early
    // `return std::unexpected(...)` runs this object's destructor, whose
    // destroy() walks entries_ and frees precisely those, nothing orphaned.
    // The reserve() means no push_back below reallocates, so `e` stays valid.
    self->entries_.reserve(shape.arrays.size());
    for (std::size_t i = 0; i < shape.arrays.size(); ++i) {
        constexpr VkBufferUsageFlags kDeviceUsage =
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        constexpr VkBufferUsageFlags kStagingUsage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

        const StateArrayShape& array = shape.arrays[i];
        self->entries_.push_back(Entry{});
        Entry& e = self->entries_.back();
        e.name = array.name;
        e.elem_size = array.elem_size;
        e.world_count = shape.world_count;
        e.capacity_per_world = array.capacity_per_world;
        e.byte_size = walk_bytes[i];
        e.has_binding = binding_for(e.name, e.binding);

        void* unused_device_mapped = nullptr;  // create_buffer(map=false) always writes null here; never read
        if (Result<void> made = create_buffer(self->device_, self->physical_device_, e.byte_size, kDeviceUsage,
                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, false, e.device_buffer,
                                               e.device_memory, unused_device_mapped);
            !made) {
            return std::unexpected(made.error());
        }
        if (Result<void> made =
                create_buffer(self->device_, self->physical_device_, e.byte_size, kStagingUsage,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true,
                              e.staging_buffer, e.staging_memory, e.staging_mapped);
            !made) {
            return std::unexpected(made.error());
        }
    }

    // -----------------------------------------------------------------------
    // The DERIVED buffers (bindings.slang sections C and C') -- none of them
    // part of the registered walk. Sized here so the descriptor set below has
    // something real to bind; the matching upload_*() calls fill them in,
    // separately, once Simulation::create() has the per-world configuration in
    // scope.
    // -----------------------------------------------------------------------
    constexpr VkBufferUsageFlags kDeviceUsage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    constexpr VkBufferUsageFlags kStagingUsage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

    // Builds one derived entry's device+staging pair. A lambda rather than a
    // fifth copy of the twenty lines dryden_params used to spell inline: S6
    // Task 6 takes the derived-buffer count from one to seven, and seven
    // hand-unrolled copies of the same sequence is exactly the kind of
    // transcription a reviewer cannot check.
    //
    // `count` is 64 bits so a caller sizing a buffer by a product (world_count
    // * body_capacity, say) passes it unwrapped, and buffer_bytes() refuses it
    // by name past 2^32-1.
    const auto make_derived = [&](Entry& e, const char* name, uint32_t elem_size, uint64_t count,
                                   uint32_t binding) -> Result<void> {
        Result<VkDeviceSize> bytes = buffer_bytes(name, elem_size, count);
        if (!bytes) return std::unexpected(bytes.error());
        e.name = name;
        e.elem_size = elem_size;
        e.world_count = static_cast<uint32_t>(count);  // element COUNT for the non-per-world buffers
        e.capacity_per_world = 1u;
        e.byte_size = *bytes;
        e.has_binding = true;
        e.binding = binding;
        void* unused_device_mapped = nullptr;
        if (Result<void> made = create_buffer(self->device_, self->physical_device_, e.byte_size, kDeviceUsage,
                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, false, e.device_buffer,
                                               e.device_memory, unused_device_mapped);
            !made) {
            return made;
        }
        return create_buffer(self->device_, self->physical_device_, e.byte_size, kStagingUsage,
                             VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true,
                             e.staging_buffer, e.staging_memory, e.staging_mapped);
    };

    if (Result<void> made = make_derived(self->dryden_params_, "dryden_params",
                                          static_cast<uint32_t>(sizeof(DrydenParams)), shape.world_count,
                                          gen::kBinding_dryden_params);
        !made) {
        return std::unexpected(made.error());
    }
    if (Result<void> made = make_derived(self->sdf_nodes_, "sdf_nodes",
                                          static_cast<uint32_t>(sizeof(SdfNodeRow)), shape.sdf_node_count,
                                          gen::kBinding_sdf_nodes);
        !made) {
        return std::unexpected(made.error());
    }
    if (Result<void> made = make_derived(self->sdf_transforms_, "sdf_transforms",
                                          static_cast<uint32_t>(sizeof(SdfTransformRow)),
                                          shape.sdf_transform_count, gen::kBinding_sdf_transforms);
        !made) {
        return std::unexpected(made.error());
    }
    if (Result<void> made = make_derived(self->sdf_ranges_, "sdf_world_ranges",
                                          static_cast<uint32_t>(sizeof(SdfWorldRange)), shape.world_count,
                                          gen::kBinding_sdf_world_ranges);
        !made) {
        return std::unexpected(made.error());
    }
    if (Result<void> made = make_derived(self->step_witness_, "step_witness",
                                          static_cast<uint32_t>(sizeof(StepWitness)), 1u,
                                          gen::kBinding_step_witness);
        !made) {
        return std::unexpected(made.error());
    }
    if (Result<void> made = make_derived(self->contact_params_, "contact_params",
                                          static_cast<uint32_t>(sizeof(physics::ContactParams)),
                                          shape.world_count, gen::kBinding_contact_params);
        !made) {
        return std::unexpected(made.error());
    }
    if (Result<void> made = make_derived(self->grid_params_, "grid_params",
                                          static_cast<uint32_t>(sizeof(physics::GridParams)), shape.world_count,
                                          gen::kBinding_grid_params);
        !made) {
        return std::unexpected(made.error());
    }
    // The CollisionDynamic key array (binding 21, S6 Task 7). Sized from
    // grid_domain_of() -- the same function StepRecorder sizes its dispatch
    // grids from -- and refused here, by name, if the shape asks for more than
    // compute/grid_entry.hpp's ceiling, rather than letting a 48 GB
    // vkAllocateMemory be the diagnostic.
    {
        const GridDomain domain = grid_domain_of(shape);
        if (!domain.ok) {
            return std::unexpected(Error{Code::capacity_exceeded,
                                         "StateMirror::create: the CollisionDynamic key array for this shape "
                                         "would exceed compute/grid_entry.hpp's kMaxGridEntries ceiling"});
        }
        if (Result<void> made = make_derived(self->grid_entries_, "grid_entries",
                                              static_cast<uint32_t>(sizeof(GridEntryRow)), domain.entry_count,
                                              gen::kBinding_grid_entries);
            !made) {
            return std::unexpected(made.error());
        }

        // The gather's shadow, one row per entry -- the SAME domain, from the
        // same grid_domain_of(shape) call, so a sizing change cannot move one
        // without the other.
        if (Result<void> made = make_derived(self->body_snapshot_, "body_snapshot",
                                              static_cast<uint32_t>(sizeof(spade::physics::GatherBody)),
                                              domain.entry_count, gen::kBinding_body_snapshot);
            !made) {
            return std::unexpected(made.error());
        }
    }

    // The field sample rows (binding 26, module-API stage 3), one per world.
    if (Result<void> made = make_derived(self->field_samples_, "field_samples",
                                          static_cast<uint32_t>(sizeof(spade::physics::FieldSampleRow)),
                                          shape.world_count, gen::kBinding_field_samples);
        !made) {
        return std::unexpected(made.error());
    }

    // -----------------------------------------------------------------------
    // step_params (binding 17) -- the ONE buffer with no device half. See
    // state_mirror.hpp's member note for why: the host rewrites it before
    // every submit, so it is host-visible, coherent and persistently mapped,
    // and the descriptor binds that buffer directly. Zeroed at creation so a
    // read before the first submit sees a defined 0 rather than driver
    // garbage.
    // -----------------------------------------------------------------------
    self->step_params_.name = "step_params";
    self->step_params_.elem_size = static_cast<uint32_t>(sizeof(StepParams));
    self->step_params_.world_count = 1u;
    self->step_params_.capacity_per_world = 1u;
    self->step_params_.byte_size = static_cast<VkDeviceSize>(sizeof(StepParams));
    self->step_params_.has_binding = true;
    self->step_params_.binding = gen::kBinding_step_params;
    if (Result<void> made = create_buffer(
            self->device_, self->physical_device_, self->step_params_.byte_size,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true,
            self->step_params_.staging_buffer, self->step_params_.staging_memory,
            self->step_params_.staging_mapped);
        !made) {
        return std::unexpected(made.error());
    }
    std::memset(self->step_params_.staging_mapped, 0, sizeof(StepParams));

    // -----------------------------------------------------------------------
    // ZERO-FILL EVERY DEVICE-LOCAL DERIVED BUFFER, ONCE, HERE. Not tidiness:
    // device-local memory comes back from vkAllocateMemory UNDEFINED, and a
    // caller is not obliged to upload every derived buffer (a Simulation
    // does upload all of them, but tests/test_gpu_state_mirror.cpp
    // constructs a backend directly to exercise the mirror alone, and nothing
    // in this class's contract says it must). An unuploaded
    // `sdf_world_ranges` would hand the CollisionStatic kernel a garbage
    // node_count and send it walking off the end of `sdf_nodes` -- a device
    // fault or a hang, from a test that never mentioned the SDF.
    //
    // Zeroed, every one of them is the DEFINED degenerate case the CPU twin
    // already has a documented answer for: a zero-node program is the empty
    // world world/sdf.cpp answers kSdfEmptyDistance for, and a zeroed
    // ContactParams is the inert material physics/contacts.hpp's own defaults
    // describe ("a zero-radius proxy only ever contacts when the body ORIGIN
    // is inside the solid").
    //
    // ONE EXCEPTION TO "ZERO IS THE DEFINED DEGENERATE CASE", NAMED RATHER
    // THAN LEFT FOR A READER TO DISCOVER (S6 hygiene: T7 review M1).
    // `grid_entries_` is NOT one of the seven buffers the paragraph above
    // describes: compute/grid_entry.hpp's GridEntryRow decodes an
    // all-zero row as world 0, cell (0,0,0), slot 0 -- a LIVE key, not the
    // sentinel that means "this slot is empty" (GridEntryRow::world's real
    // default is kGridInvalidWorld == 0xFFFFFFFF, and grid_entry.hpp's own
    // header is explicit that every slot of the padded domain must carry
    // one). Zero-filling this buffer here does not zero-fill it to its
    // "empty" representation the way the other seven are; it zero-fills it
    // to something that would misread as real body 0 data if anything ever
    // consulted it in that state. THE INVARIANT THAT MAKES THIS SAFE ANYWAY:
    // per grid_entry.hpp's own doc, this buffer is DERIVED, BACKEND-INTERNAL
    // storage that is "never snapshotted, never digested, never uploaded
    // from the host, and is rebuilt from scratch by the first dispatch of
    // every CollisionDynamic pass" -- grid_build writes a real key or an
    // explicit sentinel to EVERY entry in the domain before the sort or the
    // resolve stage (or any test's readback) ever reads one. The zero-fill
    // here exists for the same reason the other seven buffers get one
    // (device-local memory comes back UNDEFINED from vkAllocateMemory, and a
    // caller need not upload every derived buffer before first use), not
    // because zero is this buffer's meaningful empty state.
    // -----------------------------------------------------------------------
    {
        // Every derived buffer with a device half; step_params_ has none.
        std::vector<Entry*> derived;
        for (Entry* e : self->derived_entries()) {
            if (e->device_buffer != VK_NULL_HANDLE) derived.push_back(e);
        }
        for (Entry* e : derived) {
            if (e->byte_size > 0) std::memset(e->staging_mapped, 0, static_cast<std::size_t>(e->byte_size));
        }
        if (Result<void> zeroed = run_copy_batch(self->device_, self->cmd_, self->queue_, self->fence_,
                                                  /*to_device=*/true, derived);
            !zeroed) {
            return std::unexpected(zeroed.error());
        }
    }

    // -----------------------------------------------------------------------
    // Descriptor set layout + pool + set: kBindingCount_state bindings (the 13
    // bound walk entries plus the nine derived buffers -- S6 Task 7's
    // `grid_entries` is the ninth), all
    // VK_DESCRIPTOR_TYPE_STORAGE_BUFFER -- bindings.slang binds both
    // RWStructuredBuffer and (read-only) StructuredBuffer as storage buffers;
    // Vulkan's descriptor type does not distinguish read/write access, only
    // the shader-side qualifier does.
    // -----------------------------------------------------------------------
    std::vector<VkDescriptorSetLayoutBinding> layout_bindings;
    std::vector<VkWriteDescriptorSet> writes;
    std::vector<VkDescriptorBufferInfo> buffer_infos;
    buffer_infos.reserve(gen::kBindingCount_state);
    writes.reserve(gen::kBindingCount_state);
    layout_bindings.reserve(gen::kBindingCount_state);

    auto add_binding = [&](uint32_t binding, VkBuffer buffer, VkDeviceSize size) {
        VkDescriptorSetLayoutBinding lb{};
        lb.binding = binding;
        lb.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        lb.descriptorCount = 1;
        lb.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        layout_bindings.push_back(lb);

        VkDescriptorBufferInfo info{};
        info.buffer = buffer;
        info.offset = 0;
        info.range = size == 0 ? VkDeviceSize{4} : size;
        buffer_infos.push_back(info);
    };

    for (const Entry& e : self->entries_) {
        if (e.has_binding) add_binding(e.binding, e.device_buffer, e.byte_size);
    }
    // The derived buffers, from the one list (state_mirror.hpp). step_params_
    // binds its STAGING buffer -- it has no device half at all.
    for (Entry* e : self->derived_entries()) {
        add_binding(e->binding, e->device_buffer != VK_NULL_HANDLE ? e->device_buffer : e->staging_buffer,
                    e->byte_size);
    }

    self->bound_binding_count_ = static_cast<uint32_t>(layout_bindings.size());

    VkDescriptorSetLayoutCreateInfo set_layout_info{};
    set_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    set_layout_info.bindingCount = static_cast<uint32_t>(layout_bindings.size());
    set_layout_info.pBindings = layout_bindings.data();
    if (VkResult r = vkCreateDescriptorSetLayout(self->device_, &set_layout_info, nullptr, &self->set_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorSetLayout"));
    }

    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    pool_size.descriptorCount = static_cast<uint32_t>(layout_bindings.size());

    VkDescriptorPoolCreateInfo desc_pool_info{};
    desc_pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    desc_pool_info.maxSets = 1;
    desc_pool_info.poolSizeCount = 1;
    desc_pool_info.pPoolSizes = &pool_size;
    if (VkResult r = vkCreateDescriptorPool(self->device_, &desc_pool_info, nullptr, &self->desc_pool_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorPool"));
    }

    VkDescriptorSetAllocateInfo desc_alloc_info{};
    desc_alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    desc_alloc_info.descriptorPool = self->desc_pool_;
    desc_alloc_info.descriptorSetCount = 1;
    desc_alloc_info.pSetLayouts = &self->set_layout_;
    if (VkResult r = vkAllocateDescriptorSets(self->device_, &desc_alloc_info, &self->set_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkAllocateDescriptorSets"));
    }

    for (std::size_t i = 0; i < layout_bindings.size(); ++i) {
        VkWriteDescriptorSet w{};
        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w.dstSet = self->set_;
        w.dstBinding = layout_bindings[i].binding;
        w.descriptorCount = 1;
        w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        w.pBufferInfo = &buffer_infos[i];
        writes.push_back(w);
    }
    vkUpdateDescriptorSets(self->device_, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

    return self;
}

// ---------------------------------------------------------------------------
// lookup
// ---------------------------------------------------------------------------

StateMirror::Entry* StateMirror::find(const std::string& name) noexcept {
    for (Entry& e : entries_) {
        if (e.name == name) return &e;
    }
    return nullptr;
}

const StateMirror::Entry* StateMirror::find(const std::string& name) const noexcept {
    for (const Entry& e : entries_) {
        if (e.name == name) return &e;
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// upload / readback
// ---------------------------------------------------------------------------

// Records and submits a batch of vkCmdCopyBuffer commands, one per entry in
// `targets`, all in ONE command buffer / ONE submit (not one per entry) --
// the transfer analogue of step_recorder.hpp's "record once, submit once"
// discipline. Blocks (fence wait) until the copy completes, so the caller's
// subsequent host-side memcpy (readback) or the caller's claim that upload()
// completed is never racing the GPU.
Result<void> StateMirror::run_copy_batch(VkDevice device, VkCommandBuffer cmd, VkQueue queue, VkFence fence,
                                          bool to_device, const std::vector<Entry*>& targets) {
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (VkResult r = vkBeginCommandBuffer(cmd, &begin); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkBeginCommandBuffer (StateMirror copy batch)"));
    }

    for (const Entry* e : targets) {
        if (e->byte_size == 0) continue;
        VkBufferCopy region{};
        region.srcOffset = 0;
        region.dstOffset = 0;
        region.size = e->byte_size;
        if (to_device) {
            vkCmdCopyBuffer(cmd, e->staging_buffer, e->device_buffer, 1, &region);
        } else {
            vkCmdCopyBuffer(cmd, e->device_buffer, e->staging_buffer, 1, &region);
        }
    }

    if (VkResult r = vkEndCommandBuffer(cmd); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkEndCommandBuffer (StateMirror copy batch)"));
    }

    if (VkResult r = vkResetFences(device, 1, &fence); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkResetFences (StateMirror copy batch)"));
    }

    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    if (VkResult r = vkQueueSubmit(queue, 1, &submit, fence); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkQueueSubmit (StateMirror copy batch)"));
    }

    if (VkResult r = vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkWaitForFences (StateMirror copy batch)"));
    }

    return {};
}

Result<void> StateMirror::upload(const ArenaSet& arenas) {
    std::vector<Entry*> touched;
    touched.reserve(entries_.size());
    bool ok = true;
    Error err{Code::internal, ""};

    arenas.registry().for_each_array([&](const RegisteredArray& array) {
        if (!ok) return;
        Entry* e = find(array.name);
        if (e == nullptr) {
            ok = false;
            err = Error{Code::invalid_argument, "StateMirror::upload: no device buffer for registered array '" +
                                                     array.name + "'"};
            return;
        }
        if (e->elem_size != array.elem_size || e->world_count != array.world_count ||
            e->capacity_per_world != array.capacity_per_world) {
            ok = false;
            err = Error{Code::invalid_argument,
                        "StateMirror::upload: shape mismatch for '" + array.name + "' (mirror expects elem_size=" +
                            std::to_string(e->elem_size) + " world_count=" + std::to_string(e->world_count) +
                            " capacity_per_world=" + std::to_string(e->capacity_per_world) + ", got elem_size=" +
                            std::to_string(array.elem_size) + " world_count=" + std::to_string(array.world_count) +
                            " capacity_per_world=" + std::to_string(array.capacity_per_world) + ")"};
            return;
        }
        if (e->byte_size > 0) {
            std::memcpy(e->staging_mapped, array.data, static_cast<std::size_t>(e->byte_size));
        }
        touched.push_back(e);
    });
    if (!ok) return std::unexpected(err);
    if (touched.size() != entries_.size()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::upload: arenas registry has " + std::to_string(touched.size()) +
                                         " entries, this mirror expects " + std::to_string(entries_.size())});
    }

    if (Result<void> copied = run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/true, touched); !copied) {
        return copied;
    }
    ++upload_count_;
    return {};
}

Result<void> StateMirror::readback(ArenaSet& arenas) {
    std::vector<Entry*> touched;
    touched.reserve(entries_.size());
    bool ok = true;
    Error err{Code::internal, ""};

    // Pass 1: validate every entry's shape WITHOUT writing a single
    // destination byte -- see backend.hpp's readback() doc comment. A
    // mismatch discovered at entry 7 of 18 must not have already overwritten
    // entries 0..6 with device bytes while leaving 8..17 untouched and the
    // call reporting failure; validating the whole walk first, and only then
    // running the copy + the memcpy-back pass, is what makes that true.
    arenas.registry().for_each_array([&](const RegisteredArray& array) {
        if (!ok) return;
        // Non-const overload: readback() is itself non-const, so `this` is
        // non-const here and ordinary overload resolution already picks
        // `Entry* find(...)` -- no const_cast needed to put it in `touched`.
        Entry* e = find(array.name);
        if (e == nullptr || e->elem_size != array.elem_size || e->world_count != array.world_count ||
            e->capacity_per_world != array.capacity_per_world) {
            ok = false;
            err = Error{Code::invalid_argument, "StateMirror::readback: shape mismatch for '" + array.name + "'"};
            return;
        }
        touched.push_back(e);
    });
    if (!ok) return std::unexpected(err);
    if (touched.size() != entries_.size()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::readback: arenas registry has " + std::to_string(touched.size()) +
                                         " entries, this mirror expects " + std::to_string(entries_.size())});
    }

    if (Result<void> copied = run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/false, touched); !copied) {
        return copied;
    }

    // Pass 2: every device->staging copy above has completed (the fence
    // wait inside run_copy_batch already blocked for it), so it is now safe
    // to fold the staging bytes back into the arenas the caller owns.
    arenas.registry().for_each_array([&](const RegisteredArray& array) {
        const Entry* e = find(array.name);
        if (e != nullptr && e->byte_size > 0) {
            std::memcpy(array.data, e->staging_mapped, static_cast<std::size_t>(e->byte_size));
        }
    });
    return {};
}

Result<void> StateMirror::upload_dryden_params(std::span<const std::byte> params_bytes, uint32_t elem_size,
                                                uint32_t world_count) {
    if (elem_size != dryden_params_.elem_size || world_count != dryden_params_.world_count) {
        return std::unexpected(
            Error{Code::invalid_argument, "StateMirror::upload_dryden_params: shape mismatch (mirror expects "
                                               "elem_size=" +
                                               std::to_string(dryden_params_.elem_size) +
                                               " world_count=" + std::to_string(dryden_params_.world_count) +
                                               ", got elem_size=" + std::to_string(elem_size) +
                                               " world_count=" + std::to_string(world_count) + ")"});
    }
    const std::size_t expected = static_cast<std::size_t>(dryden_params_.byte_size);
    if (params_bytes.size() != expected) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::upload_dryden_params: expected " + std::to_string(expected) +
                                         " bytes, got " + std::to_string(params_bytes.size())});
    }

    if (expected > 0) {
        std::memcpy(dryden_params_.staging_mapped, params_bytes.data(), expected);
    }

    std::vector<Entry*> targets{&dryden_params_};
    return run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/true, targets);
}

// ---------------------------------------------------------------------------
// S6 Task 6's derived uploads and the one readback. Each validates its byte
// count against the size this mirror was CREATED for and copies nothing on a
// mismatch -- the same all-or-nothing posture upload_dryden_params() above and
// readback() further up already take, and for readback()'s stated reason: a
// partially-written destination is worse than a reported failure.
// ---------------------------------------------------------------------------

Result<void> StateMirror::upload_sdf_program(std::span<const std::byte> node_bytes,
                                               std::span<const std::byte> transform_bytes,
                                               std::span<const std::byte> range_bytes) {
    const auto check = [](std::string_view what, std::span<const std::byte> bytes, VkDeviceSize expected)
        -> Result<void> {
        if (bytes.size() != static_cast<std::size_t>(expected)) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "StateMirror::upload_sdf_program: " + std::string(what) +
                                             " expected " + std::to_string(expected) + " bytes, got " +
                                             std::to_string(bytes.size())});
        }
        return {};
    };
    if (Result<void> r = check("sdf_nodes", node_bytes, sdf_nodes_.byte_size); !r) return r;
    if (Result<void> r = check("sdf_transforms", transform_bytes, sdf_transforms_.byte_size); !r) return r;
    if (Result<void> r = check("sdf_world_ranges", range_bytes, sdf_ranges_.byte_size); !r) return r;

    if (!node_bytes.empty()) {
        std::memcpy(sdf_nodes_.staging_mapped, node_bytes.data(), node_bytes.size());
    }
    if (!transform_bytes.empty()) {
        std::memcpy(sdf_transforms_.staging_mapped, transform_bytes.data(), transform_bytes.size());
    }
    if (!range_bytes.empty()) {
        std::memcpy(sdf_ranges_.staging_mapped, range_bytes.data(), range_bytes.size());
    }

    std::vector<Entry*> targets{&sdf_nodes_, &sdf_transforms_, &sdf_ranges_};
    return run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/true, targets);
}

Result<void> StateMirror::upload_contact_params(std::span<const std::byte> params_bytes, uint32_t elem_size,
                                                  uint32_t world_count) {
    if (elem_size != contact_params_.elem_size || world_count != contact_params_.world_count) {
        return std::unexpected(Error{
            Code::invalid_argument,
            "StateMirror::upload_contact_params: shape mismatch (mirror expects elem_size=" +
                std::to_string(contact_params_.elem_size) +
                " world_count=" + std::to_string(contact_params_.world_count) +
                ", got elem_size=" + std::to_string(elem_size) +
                " world_count=" + std::to_string(world_count) + ")"});
    }
    const std::size_t expected = static_cast<std::size_t>(contact_params_.byte_size);
    if (params_bytes.size() != expected) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::upload_contact_params: expected " +
                                         std::to_string(expected) + " bytes, got " +
                                         std::to_string(params_bytes.size())});
    }
    if (expected > 0) {
        std::memcpy(contact_params_.staging_mapped, params_bytes.data(), expected);
    }
    std::vector<Entry*> targets{&contact_params_};
    return run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/true, targets);
}

Result<void> StateMirror::upload_grid_params(std::span<const std::byte> params_bytes, uint32_t elem_size,
                                              uint32_t world_count) {
    if (elem_size != grid_params_.elem_size || world_count != grid_params_.world_count) {
        return std::unexpected(Error{
            Code::invalid_argument,
            "StateMirror::upload_grid_params: shape mismatch (mirror expects elem_size=" +
                std::to_string(grid_params_.elem_size) +
                " world_count=" + std::to_string(grid_params_.world_count) +
                ", got elem_size=" + std::to_string(elem_size) +
                " world_count=" + std::to_string(world_count) + ")"});
    }
    const std::size_t expected = static_cast<std::size_t>(grid_params_.byte_size);
    if (params_bytes.size() != expected) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::upload_grid_params: expected " +
                                         std::to_string(expected) + " bytes, got " +
                                         std::to_string(params_bytes.size())});
    }
    if (expected > 0) {
        std::memcpy(grid_params_.staging_mapped, params_bytes.data(), expected);
    }
    std::vector<Entry*> targets{&grid_params_};
    return run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/true, targets);
}

Result<void> StateMirror::read_step_witness(std::span<std::byte> out_bytes) {
    const std::size_t expected = static_cast<std::size_t>(step_witness_.byte_size);
    if (out_bytes.size() != expected) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::read_step_witness: expected " + std::to_string(expected) +
                                         " bytes, got " + std::to_string(out_bytes.size())});
    }
    std::vector<Entry*> targets{&step_witness_};
    if (Result<void> copied = run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/false, targets);
        !copied) {
        return copied;
    }
    std::memcpy(out_bytes.data(), step_witness_.staging_mapped, expected);
    return {};
}

Result<void> StateMirror::read_field_samples(std::span<std::byte> out_bytes) {
    const std::size_t expected = static_cast<std::size_t>(field_samples_.byte_size);
    if (out_bytes.size() != expected) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::read_field_samples: expected " + std::to_string(expected) +
                                         " bytes, got " + std::to_string(out_bytes.size())});
    }
    if (expected == 0) return {};
    std::vector<Entry*> targets{&field_samples_};
    if (Result<void> copied = run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/false, targets);
        !copied) {
        return copied;
    }
    std::memcpy(out_bytes.data(), field_samples_.staging_mapped, expected);
    return {};
}

Result<void> StateMirror::read_grid_entries(std::span<std::byte> out_bytes) {
    const std::size_t expected = static_cast<std::size_t>(grid_entries_.byte_size);
    if (out_bytes.size() != expected) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::read_grid_entries: expected " + std::to_string(expected) +
                                         " bytes, got " + std::to_string(out_bytes.size())});
    }
    if (expected == 0) return {};
    std::vector<Entry*> targets{&grid_entries_};
    if (Result<void> copied = run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/false, targets);
        !copied) {
        return copied;
    }
    std::memcpy(out_bytes.data(), grid_entries_.staging_mapped, expected);
    return {};
}

// ---------------------------------------------------------------------------
// teardown
// ---------------------------------------------------------------------------

void StateMirror::destroy_entry(VkDevice device, Entry& e) noexcept {
    if (e.staging_mapped != nullptr && e.staging_memory != VK_NULL_HANDLE) {
        vkUnmapMemory(device, e.staging_memory);
        e.staging_mapped = nullptr;
    }
    if (e.staging_buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device, e.staging_buffer, nullptr);
        e.staging_buffer = VK_NULL_HANDLE;
    }
    if (e.staging_memory != VK_NULL_HANDLE) {
        vkFreeMemory(device, e.staging_memory, nullptr);
        e.staging_memory = VK_NULL_HANDLE;
    }
    if (e.device_buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device, e.device_buffer, nullptr);
        e.device_buffer = VK_NULL_HANDLE;
    }
    if (e.device_memory != VK_NULL_HANDLE) {
        vkFreeMemory(device, e.device_memory, nullptr);
        e.device_memory = VK_NULL_HANDLE;
    }
}

void StateMirror::destroy() noexcept {
    if (device_ == VK_NULL_HANDLE) return;  // never fully constructed

    if (desc_pool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, desc_pool_, nullptr);
        desc_pool_ = VK_NULL_HANDLE;
        set_ = VK_NULL_HANDLE;  // freed implicitly with the pool
    }
    if (set_layout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, set_layout_, nullptr);
        set_layout_ = VK_NULL_HANDLE;
    }

    for (Entry* e : derived_entries()) destroy_entry(device_, *e);
    for (Entry& e : entries_) destroy_entry(device_, e);
    entries_.clear();

    if (fence_ != VK_NULL_HANDLE) {
        vkDestroyFence(device_, fence_, nullptr);
        fence_ = VK_NULL_HANDLE;
    }
    if (pool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, pool_, nullptr);  // frees cmd_ implicitly
        pool_ = VK_NULL_HANDLE;
        cmd_ = VK_NULL_HANDLE;
    }

    device_ = VK_NULL_HANDLE;
}

StateMirror::~StateMirror() { destroy(); }

}  // namespace spade::compute
