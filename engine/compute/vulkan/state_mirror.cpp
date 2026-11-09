#include "compute/vulkan/state_mirror.hpp"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <iterator>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "bindings.gen.hpp"

#include "compute/grid_entry.hpp"
#include "compute/layout_classes.hpp"
#include "compute/optional_bindings.hpp"
#include "compute/sdf_program.hpp"
#include "compute/set_one_bindings.hpp"
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
//
// THE ALLOCATION LEDGER (stage 7 Task A3): every successful vkAllocateMemory is counted in g_mirror_allocations, and the
// memory type chosen is written to `out_type`, so memory_report() can say which type and heap each buffer got.
std::atomic<uint64_t> g_mirror_allocations{0};

[[nodiscard]] Result<void> create_buffer(VkDevice device, VkPhysicalDevice phys, VkDeviceSize size,
                                          VkBufferUsageFlags usage, VkMemoryPropertyFlags mem_props, bool map,
                                          VkBuffer& out_buffer, VkDeviceMemory& out_memory, void*& out_mapped,
                                          uint32_t* out_type = nullptr) {
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
    g_mirror_allocations.fetch_add(1, std::memory_order_relaxed);
    if (out_type != nullptr) *out_type = *type;

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
        // The translation lock's rows (module API stage 6), the first OPTIONAL binding: bound to the walk entry when
        // the set holds the lock module, and to a zeroed buffer (below) when it does not.
        {"lock_rows", gen::kBinding_lock_rows},
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
// The derived buffers' table (state_mirror.hpp). The hand-listed eleven keep
// the sizes, counts and bindings create() gave them one by one before Task 7b;
// only the place they are spelt moved.
// ---------------------------------------------------------------------------
Result<std::vector<DerivedBufferShape>> derived_buffer_shapes(const StepShape& shape, uint32_t binding_count) {
    // The CollisionDynamic key array (binding 21, S6 Task 7) and the gather's
    // shadow (binding 22) are sized from grid_domain_of() -- the same function
    // StepRecorder sizes its dispatch grids from -- and refused here, by name,
    // if the shape asks for more than compute/grid_entry.hpp's ceiling, rather
    // than letting a 48 GB vkAllocateMemory be the diagnostic. Both from one
    // call, so a sizing change cannot move one without the other.
    const GridDomain domain = grid_domain_of(shape);
    if (!domain.ok) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "StateMirror::create: the CollisionDynamic key array for this shape "
                                     "would exceed compute/grid_entry.hpp's kMaxGridEntries ceiling"});
    }

    struct Row {
        const char* name;
        uint32_t elem_size;
        uint64_t count;
        uint32_t binding;
    };
    // In derived_entries() order; create() pairs them by position.
    const Row hand_listed[] = {
        {.name = "dryden_params",
         .elem_size = static_cast<uint32_t>(sizeof(DrydenParams)),
         .count = shape.world_count,
         .binding = gen::kBinding_dryden_params},
        {.name = "sdf_nodes",
         .elem_size = static_cast<uint32_t>(sizeof(SdfNodeRow)),
         .count = shape.sdf_node_count,
         .binding = gen::kBinding_sdf_nodes},
        {.name = "sdf_transforms",
         .elem_size = static_cast<uint32_t>(sizeof(SdfTransformRow)),
         .count = shape.sdf_transform_count,
         .binding = gen::kBinding_sdf_transforms},
        {.name = "sdf_world_ranges",
         .elem_size = static_cast<uint32_t>(sizeof(SdfWorldRange)),
         .count = shape.world_count,
         .binding = gen::kBinding_sdf_world_ranges},
        // The one buffer with no device half (state_mirror.hpp): one StepParams.
        {.name = "step_params",
         .elem_size = static_cast<uint32_t>(sizeof(StepParams)),
         .count = 1u,
         .binding = gen::kBinding_step_params},
        {.name = "step_witness",
         .elem_size = static_cast<uint32_t>(sizeof(StepWitness)),
         .count = 1u,
         .binding = gen::kBinding_step_witness},
        {.name = "contact_params",
         .elem_size = static_cast<uint32_t>(sizeof(physics::ContactParams)),
         .count = shape.world_count,
         .binding = gen::kBinding_contact_params},
        {.name = "grid_params",
         .elem_size = static_cast<uint32_t>(sizeof(physics::GridParams)),
         .count = shape.world_count,
         .binding = gen::kBinding_grid_params},
        {.name = "grid_entries",
         .elem_size = static_cast<uint32_t>(sizeof(GridEntryRow)),
         .count = domain.entry_count,
         .binding = gen::kBinding_grid_entries},
        {.name = "body_snapshot",
         .elem_size = static_cast<uint32_t>(sizeof(spade::physics::GatherBody)),
         .count = domain.entry_count,
         .binding = gen::kBinding_body_snapshot},
        // The field sample rows (binding 26, module-API stage 3), one per world.
        {.name = "field_samples",
         .elem_size = static_cast<uint32_t>(sizeof(spade::physics::FieldSampleRow)),
         .count = shape.world_count,
         .binding = gen::kBinding_field_samples},
        // The contact scratch (binding 27, PHY-7) is not here: integrate
        // declares it (sim/standard_modules.cpp, a ScratchDecl), so it reaches
        // the mirror through StepShape::scratch like any declared scratch.
    };

    std::vector<DerivedBufferShape> out;
    out.reserve(std::size(hand_listed) + shape.scratch.size());
    for (const Row& row : hand_listed) {
        Result<VkDeviceSize> bytes = buffer_bytes(row.name, row.elem_size, row.count);
        if (!bytes) return std::unexpected(bytes.error());
        out.push_back(DerivedBufferShape{
            .name = row.name, .elem_size = row.elem_size, .count = row.count, .byte_size = *bytes, .binding = row.binding});
    }

    // THE DECLARED SCRATCH (Task 7b). Every binding already held, with what
    // holds it, so a refusal names both sides: the walk entries a kernel reads
    // (binding_for()), the hand-listed buffers above, then each scratch as it
    // is placed.
    std::vector<std::pair<uint32_t, std::string>> held;
    for (const StateArrayShape& array : shape.arrays) {
        uint32_t binding = 0;
        if (binding_for(array.name, binding)) held.emplace_back(binding, "walk entry '" + array.name + "'");
    }
    for (const DerivedBufferShape& derived : out) {
        held.emplace_back(derived.binding, "derived buffer '" + derived.name + "'");
    }
    for (const ScratchShape& scratch : shape.scratch) {
        if (scratch.binding == kNoBinding) continue;  // CPU-only: no device buffer
        const std::string where = "StateMirror::create: scratch '" + scratch.name + "'";
        if (scratch.binding >= binding_count) {
            return std::unexpected(Error{Code::invalid_argument,
                                         where + ": binding " + std::to_string(scratch.binding) +
                                             " is outside the generated range [0, " + std::to_string(binding_count) +
                                             "); a scratch's binding is a gen::kBinding_* constant bindings.slang "
                                             "declares"});
        }
        if (const auto taken = std::ranges::find(held, scratch.binding, &std::pair<uint32_t, std::string>::first);
            taken != held.end()) {
            return std::unexpected(Error{Code::invalid_argument, where + ": binding " +
                                                                     std::to_string(scratch.binding) +
                                                                     " is already bound to " + taken->second});
        }
        const uint64_t count = uint64_t{shape.world_count} * uint64_t{scratch.capacity_per_world};
        Result<VkDeviceSize> bytes = buffer_bytes(scratch.name, scratch.elem_size, count);
        if (!bytes) return std::unexpected(bytes.error());
        held.emplace_back(scratch.binding, "scratch '" + scratch.name + "'");
        out.push_back(DerivedBufferShape{.name = scratch.name,
                                         .elem_size = scratch.elem_size,
                                         .count = count,
                                         .byte_size = *bytes,
                                         .binding = scratch.binding});
    }
    // THE FIXED BINDINGS OF ABSENT OPTIONAL MODULES (module API stage 6; compute/optional_bindings.hpp). An optional
    // binding whose array the walk does not carry gets a zeroed buffer of the table's `dummy_bytes`, so the descriptor
    // set is complete and the recorder's switch keeps the kernel from reading it. It rides with the scratch (it is
    // zero-filled at create, never uploaded, never read back), before the tables.
    for (const OptionalBinding& optional : kOptionalBindings) {
        const bool held_by_walk = std::ranges::any_of(
            shape.arrays, [&optional](const StateArrayShape& array) { return array.name == optional.name; });
        if (held_by_walk) continue;
        const std::string name(optional.name);
        const std::string where = "StateMirror::create: optional binding '" + name + "'";
        uint32_t binding = 0;
        if (!binding_for(name, binding)) {
            return std::unexpected(Error{Code::internal, where + " has no entry in binding_for()"});
        }
        if (const auto taken = std::ranges::find(held, binding, &std::pair<uint32_t, std::string>::first);
            taken != held.end()) {
            return std::unexpected(Error{Code::invalid_argument, where + ": binding " + std::to_string(binding) +
                                                                     " is already bound to " + taken->second});
        }
        Result<VkDeviceSize> bytes = buffer_bytes(name, optional.dummy_bytes, 1);
        if (!bytes) return std::unexpected(bytes.error());
        held.emplace_back(binding, "optional binding '" + name + "' (absent)");
        out.push_back(DerivedBufferShape{.name = name + " (absent)",
                                         .elem_size = optional.dummy_bytes,
                                         .count = 1,
                                         .byte_size = *bytes,
                                         .binding = binding});
    }
    // THE BOUND CONFIGURATION TABLES (L10), after every scratch and by the same
    // rules: a float buffer at the table's capacity, its binding checked
    // against the range and against everything already held.
    for (const TableShape& table : shape.tables) {
        if (table.binding == kNoBinding) continue;
        const std::string where = "StateMirror::create: table '" + table.name + "'";
        if (table.binding >= binding_count) {
            return std::unexpected(Error{Code::invalid_argument,
                                         where + ": binding " + std::to_string(table.binding) +
                                             " is outside the generated range [0, " + std::to_string(binding_count) +
                                             ")"});
        }
        if (const auto taken = std::ranges::find(held, table.binding, &std::pair<uint32_t, std::string>::first);
            taken != held.end()) {
            return std::unexpected(Error{Code::invalid_argument, where + ": binding " +
                                                                     std::to_string(table.binding) +
                                                                     " is already bound to " + taken->second});
        }
        Result<VkDeviceSize> bytes = buffer_bytes(table.name, static_cast<uint32_t>(sizeof(float)), table.capacity);
        if (!bytes) return std::unexpected(bytes.error());
        held.emplace_back(table.binding, "table '" + table.name + "'");
        out.push_back(DerivedBufferShape{.name = table.name,
                                         .elem_size = static_cast<uint32_t>(sizeof(float)),
                                         .count = table.capacity,
                                         .byte_size = *bytes,
                                         .binding = table.binding});
    }
    return out;
}

// ---------------------------------------------------------------------------
// create
// ---------------------------------------------------------------------------

Result<std::unique_ptr<StateMirror>> StateMirror::create(VulkanContext& ctx, const StepShape& shape,
                                                         std::span<const std::vector<uint32_t>> set1_subsets) {
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

    // THE CONFIGURATION IMAGE AND SET 1's NAMES (stage 7 Task C1/C2), refused by name before anything is made. The image and
    // its binding come together; the binding is inside set 1's space and is held by nothing else (a walk array or scratch
    // whose name is in set_one_bindings.hpp holds the number that table gives it).
    if (shape.config_image.empty() != (shape.config_binding == kNoBinding)) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::create: a configuration image (" +
                                         std::to_string(shape.config_image.size()) + " bytes) and its binding (" +
                                         (shape.config_binding == kNoBinding ? std::string("none")
                                                                             : std::to_string(shape.config_binding)) +
                                         ") come together: StepShape::config_image and config_binding"});
    }
    if (shape.config_binding != kNoBinding) {
        if (shape.config_binding >= kSet1BindingSlots) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "StateMirror::create: the configuration image's binding " +
                                             std::to_string(shape.config_binding) + " is past set 1's " +
                                             std::to_string(kSet1BindingSlots) + " binding slots"});
        }
        for (const StateArrayShape& array : shape.arrays) {
            if (set_one_binding_of(array.name) == shape.config_binding) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "StateMirror::create: the configuration image's binding " +
                                                 std::to_string(shape.config_binding) + " is already bound to walk array '" +
                                                 array.name + "'"});
            }
        }
        for (const ScratchShape& scratch : shape.scratch) {
            if (scratch.binding == kNoBinding && set_one_binding_of(scratch.name) == shape.config_binding) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "StateMirror::create: the configuration image's binding " +
                                                 std::to_string(shape.config_binding) + " is already bound to scratch '" +
                                                 scratch.name + "'"});
            }
        }
    }
    // A scratch that names no set-0 binding but whose name is in the set-1 table is a set-1 scratch, sized like any scratch.
    std::vector<std::pair<const ScratchShape*, VkDeviceSize>> set1_scratch;
    for (const ScratchShape& scratch : shape.scratch) {
        if (scratch.binding != kNoBinding || set_one_binding_of(scratch.name) == kNoBinding) continue;
        Result<VkDeviceSize> bytes = buffer_bytes(scratch.name, scratch.elem_size,
                                                  uint64_t{shape.world_count} * uint64_t{scratch.capacity_per_world});
        if (!bytes) return std::unexpected(bytes.error());
        set1_scratch.emplace_back(&scratch, *bytes);
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
        e.resident = array.resident;

        void* unused_device_mapped = nullptr;  // create_buffer(map=false) always writes null here; never read
        if (Result<void> made = create_buffer(self->device_, self->physical_device_, e.byte_size, kDeviceUsage,
                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, false, e.device_buffer,
                                               e.device_memory, unused_device_mapped, &e.device_type);
            !made) {
            return std::unexpected(made.error());
        }
        // A RESIDENT array is device-only (stage 7 Task C2): the host never writes or reads it per step, so it has no
        // staging twin; the one shared staging buffer below serves upload_resident() and readback_resident().
        if (e.resident) continue;
        if (Result<void> made =
                create_buffer(self->device_, self->physical_device_, e.byte_size, kStagingUsage,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true,
                              e.staging_buffer, e.staging_memory, e.staging_mapped, &e.staging_type);
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
    //
    // FROM ONE TABLE (module-API stage 4, Task 7b): derived_buffer_shapes()
    // spells every derived buffer's name, size and binding -- the hand-listed
    // eleven in derived_entries() order, then every declared scratch that names
    // a binding -- and refuses, by name, a shape or a scratch binding the
    // mirror cannot take. It runs after the walk's buffers are made, where the
    // derived sizing always ran, so a shape the walk cannot allocate still
    // fails there first (AbsurdShapeAllocationFailureIsReportedNotCrashed).
    // -----------------------------------------------------------------------
    constexpr VkBufferUsageFlags kDeviceUsage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    constexpr VkBufferUsageFlags kStagingUsage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

    Result<std::vector<DerivedBufferShape>> derived = derived_buffer_shapes(shape, gen::kBindingCount_state);
    if (!derived) return std::unexpected(derived.error());
    const std::array<Entry*, 11> hand_listed = self->derived_entries();
    if (derived->size() < hand_listed.size()) {
        return std::unexpected(Error{Code::internal, "StateMirror::create: the derived table lost a hand-listed row"});
    }
    // Sized once, before any buffer is made, so no Entry moves after. The
    // derived rows are the hand-listed ones, the bound scratch, then the bound
    // tables (derived_buffer_shapes()' order).
    const auto table_count = static_cast<std::size_t>(
        std::ranges::count_if(shape.tables, [](const TableShape& t) { return t.binding != kNoBinding; }));
    if (derived->size() < hand_listed.size() + table_count) {
        return std::unexpected(Error{Code::internal, "StateMirror::create: the derived table lost a table row"});
    }
    self->scratch_entries_.resize(derived->size() - hand_listed.size() - table_count);
    self->table_entries_.resize(table_count);

    // Builds one derived entry's device+staging pair from its row. A lambda
    // rather than a copy of the twenty lines dryden_params used to spell inline
    // per buffer: S6 Task 6 took the derived-buffer count from one to seven,
    // and hand-unrolled copies of the same sequence are exactly the kind of
    // transcription a reviewer cannot check.
    // `staged` false is a DEVICE-ONLY buffer (stage 7 Task A2, design 7.3): nothing on the host writes or reads it, so it
    // has no host-visible twin. That is the declared scratch (never uploaded or read back), body_snapshot (the kernels'
    // own copy) and the dummy bound for an absent optional binding (zero-filled once, below).
    const auto make_derived = [&](Entry& e, const DerivedBufferShape& row, bool staged) -> Result<void> {
        e.name = row.name;
        e.elem_size = row.elem_size;
        e.world_count = static_cast<uint32_t>(row.count);  // element COUNT for the non-per-world buffers
        e.capacity_per_world = 1u;
        e.byte_size = row.byte_size;
        e.has_binding = true;
        e.binding = row.binding;
        void* unused_device_mapped = nullptr;
        if (Result<void> made = create_buffer(self->device_, self->physical_device_, e.byte_size, kDeviceUsage,
                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, false, e.device_buffer,
                                               e.device_memory, unused_device_mapped, &e.device_type);
            !made) {
            return made;
        }
        if (!staged) return {};
        return create_buffer(self->device_, self->physical_device_, e.byte_size, kStagingUsage,
                             VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true,
                             e.staging_buffer, e.staging_memory, e.staging_mapped, &e.staging_type);
    };

    for (std::size_t i = 0; i < derived->size(); ++i) {
        const DerivedBufferShape& row = (*derived)[i];
        const std::size_t scratch_end = hand_listed.size() + self->scratch_entries_.size();
        Entry& e = i < hand_listed.size() ? *hand_listed[i]
                   : i < scratch_end      ? self->scratch_entries_[i - hand_listed.size()]
                                          : self->table_entries_[i - scratch_end];
        if (&e != &self->step_params_) {
            const bool device_only = (i >= hand_listed.size() && i < scratch_end) || &e == &self->body_snapshot_;
            if (Result<void> made = make_derived(e, row, !device_only); !made) return std::unexpected(made.error());
            continue;
        }
        // -------------------------------------------------------------------
        // step_params (binding 17) -- the ONE buffer with no device half. See
        // state_mirror.hpp's member note for why: the host rewrites it before
        // every submit, so it is host-visible, coherent and persistently
        // mapped, and the descriptor binds that buffer directly. Zeroed at
        // creation so a read before the first submit sees a defined 0 rather
        // than driver garbage.
        // -------------------------------------------------------------------
        e.name = row.name;
        e.elem_size = row.elem_size;
        e.world_count = 1u;
        e.capacity_per_world = 1u;
        e.byte_size = row.byte_size;
        e.has_binding = true;
        e.binding = row.binding;
        if (Result<void> made = create_buffer(
                self->device_, self->physical_device_, e.byte_size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true, e.staging_buffer,
                e.staging_memory, e.staging_mapped, &e.staging_type);
            !made) {
            return std::unexpected(made.error());
        }
        std::memset(e.staging_mapped, 0, static_cast<std::size_t>(e.byte_size));
    }

    // -----------------------------------------------------------------------
    // SET 1's OWN BUFFERS (stage 7 Task C1/C2), after the set-0 ones and by the same builder. A set with none of them makes
    // none of them (the standard set: nothing here runs).
    //   * the set-1 scratch: device-only, zero-filled with the rest below, bound at its name's set-1 binding;
    //   * the one shared staging buffer of the resident arrays, sized to the largest (host-visible, no device half);
    //   * the configuration image: staged, so it is uploaded here and read back on demand.
    // -----------------------------------------------------------------------
    self->set_one_scratch_.resize(set1_scratch.size());
    for (std::size_t i = 0; i < set1_scratch.size(); ++i) {
        const ScratchShape& scratch = *set1_scratch[i].first;
        const DerivedBufferShape row{.name = scratch.name,
                                     .elem_size = scratch.elem_size,
                                     .count = uint64_t{shape.world_count} * uint64_t{scratch.capacity_per_world},
                                     .byte_size = set1_scratch[i].second,
                                     .binding = set_one_binding_of(scratch.name)};
        if (Result<void> made = make_derived(self->set_one_scratch_[i], row, /*staged=*/false); !made) {
            return std::unexpected(made.error());
        }
    }
    {
        VkDeviceSize largest = 0;
        bool any_resident = false;
        for (const Entry& e : self->entries_) {
            if (!e.resident) continue;
            any_resident = true;
            largest = std::max(largest, e.byte_size);
        }
        if (any_resident) {
            Entry& staging = self->resident_staging_;
            staging.name = "resident_staging";
            staging.byte_size = largest;
            if (Result<void> made = create_buffer(
                    self->device_, self->physical_device_, largest, kStagingUsage,
                    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true,
                    staging.staging_buffer, staging.staging_memory, staging.staging_mapped, &staging.staging_type);
                !made) {
                return std::unexpected(made.error());
            }
        }
    }
    if (shape.config_binding != kNoBinding) {
        const DerivedBufferShape row{.name = "airflow_config",
                                     .elem_size = 1u,
                                     .count = shape.config_image.size(),
                                     .byte_size = shape.config_image.size(),
                                     .binding = shape.config_binding};
        if (shape.config_image.size() > std::numeric_limits<uint32_t>::max()) {
            return std::unexpected(Error{Code::capacity_exceeded, "StateMirror::create: the configuration image is " +
                                                                      std::to_string(shape.config_image.size()) +
                                                                      " bytes, past what a uint32 addresses"});
        }
        if (Result<void> made = make_derived(self->config_, row, /*staged=*/true); !made) {
            return std::unexpected(made.error());
        }
    }

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
    //
    // A DECLARED SCRATCH (Task 7b) is zero-filled here for its own reason: zero
    // is its state at every substep boundary (sim/module.hpp's ScratchDecl),
    // and this is the only time the mirror writes it. After this its module's
    // kernels keep it zero; nothing uploads or reads it back.
    // -----------------------------------------------------------------------
    {
        // Every derived buffer with a device half; step_params_ has none.
        // A staged buffer is zeroed through its twin; a device-only one has none, so vkCmdFillBuffer zeroes it (same
        // submit, same fence).
        std::vector<Entry*> derived_buffers;
        std::vector<Entry*> device_only;
        self->for_each_derived([&](Entry& e) {
            if (e.device_buffer == VK_NULL_HANDLE) return;
            (e.staging_buffer != VK_NULL_HANDLE ? derived_buffers : device_only).push_back(&e);
        });
        for (Entry* e : derived_buffers) {
            if (e->byte_size > 0) std::memset(e->staging_mapped, 0, static_cast<std::size_t>(e->byte_size));
        }
        // Set 1's (stage 7 Task C1/C2): a resident array and a set-1 scratch are device-only, so they are filled with zeros;
        // the configuration image is copied up from its twin, in this same submit (and counted as uploaded).
        for (Entry& e : self->entries_) {
            if (e.resident) device_only.push_back(&e);
        }
        for (Entry& e : self->set_one_scratch_) device_only.push_back(&e);
        if (self->config_.device_buffer != VK_NULL_HANDLE) {
            std::memcpy(self->config_.staging_mapped, shape.config_image.data(), shape.config_image.size());
            derived_buffers.push_back(&self->config_);
            self->transfer_.uploaded += shape.config_image.size();
        }
        if (Result<void> zeroed = run_copy_batch(self->device_, self->cmd_, self->queue_, self->fence_,
                                                  /*to_device=*/true, derived_buffers, &device_only);
            !zeroed) {
            return std::unexpected(zeroed.error());
        }
    }

    // -----------------------------------------------------------------------
    // Descriptor set layout + pool + set: every bound walk entry, every
    // hand-listed derived buffer and every bound scratch -- for the standard
    // set, kBindingCount_state bindings, the census
    // DescriptorSetBindsEveryRegistryBinding pins -- all
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
    // The derived buffers, hand-listed then scratch, through the one walk
    // (state_mirror.hpp's for_each_derived()). step_params_ binds its STAGING
    // buffer -- it has no device half at all.
    self->for_each_derived([&add_binding](Entry& e) {
        add_binding(e.binding, e.device_buffer != VK_NULL_HANDLE ? e.device_buffer : e.staging_buffer, e.byte_size);
    });

    self->bound_binding_count_ = static_cast<uint32_t>(layout_bindings.size());

    VkDescriptorSetLayoutCreateInfo set_layout_info{};
    set_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    set_layout_info.bindingCount = static_cast<uint32_t>(layout_bindings.size());
    set_layout_info.pBindings = layout_bindings.data();
    if (VkResult r = vkCreateDescriptorSetLayout(self->device_, &set_layout_info, nullptr, &self->set_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorSetLayout"));
    }

    // ONE pool for set 0 and every set-1 set (stage 7 Task C2): the standard set asks for exactly the one set it always did.
    uint32_t pool_descriptors = static_cast<uint32_t>(layout_bindings.size());
    for (const std::vector<uint32_t>& subset : set1_subsets) pool_descriptors += static_cast<uint32_t>(subset.size());

    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    pool_size.descriptorCount = pool_descriptors;

    VkDescriptorPoolCreateInfo desc_pool_info{};
    desc_pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    desc_pool_info.maxSets = 1u + static_cast<uint32_t>(set1_subsets.size());
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

    // -----------------------------------------------------------------------
    // SET 1 (stage 7 Tasks C1/C2; plan decisions 1 and 2): one descriptor set layout and one set per DISTINCT subset a kernel
    // of the schedule names, holding exactly those bindings at the declaration's numbers. Each is written from the buffer
    // whose name is in set_one_bindings.hpp (a walk array's device buffer, resident or not; a set-1 scratch) or the
    // configuration image. A set with no set-1 user was given no subsets, and nothing below runs.
    // -----------------------------------------------------------------------
    struct SetOneBuffer {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceSize size = 0;
    };
    const auto provider_of = [&self](uint32_t binding) -> SetOneBuffer {
        for (const Entry& e : self->entries_) {
            if (set_one_binding_of(e.name) == binding) return {e.device_buffer, e.byte_size};
        }
        for (const Entry& e : self->set_one_scratch_) {
            if (e.binding == binding) return {e.device_buffer, e.byte_size};
        }
        if (self->config_.device_buffer != VK_NULL_HANDLE && self->config_.binding == binding) {
            return {self->config_.device_buffer, self->config_.byte_size};
        }
        return {};
    };
    self->set1_.reserve(set1_subsets.size());
    for (const std::vector<uint32_t>& subset : set1_subsets) {
        self->set1_.push_back(SetOneSet{.bindings = subset});
        SetOneSet& one = self->set1_.back();

        std::vector<VkDescriptorSetLayoutBinding> one_bindings(subset.size());
        std::vector<VkDescriptorBufferInfo> one_infos(subset.size());
        for (std::size_t i = 0; i < subset.size(); ++i) {
            const SetOneBuffer provided = provider_of(subset[i]);
            if (provided.buffer == VK_NULL_HANDLE) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "StateMirror::create: a kernel names set-1 binding " +
                                                 std::to_string(subset[i]) +
                                                 ", which no walk array, scratch or configuration image of the set "
                                                 "provides (names are the contract: compute/set_one_bindings.hpp)"});
            }
            one_bindings[i].binding = subset[i];
            one_bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            one_bindings[i].descriptorCount = 1;
            one_bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
            one_infos[i].buffer = provided.buffer;
            one_infos[i].offset = 0;
            one_infos[i].range = provided.size == 0 ? VkDeviceSize{4} : provided.size;
        }

        VkDescriptorSetLayoutCreateInfo one_layout_info{};
        one_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        one_layout_info.bindingCount = static_cast<uint32_t>(one_bindings.size());
        one_layout_info.pBindings = one_bindings.data();
        if (VkResult r = vkCreateDescriptorSetLayout(self->device_, &one_layout_info, nullptr, &one.layout);
            r != VK_SUCCESS) {
            return std::unexpected(map_vk_error(r, "vkCreateDescriptorSetLayout (set 1)"));
        }
        VkDescriptorSetAllocateInfo one_alloc{};
        one_alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        one_alloc.descriptorPool = self->desc_pool_;
        one_alloc.descriptorSetCount = 1;
        one_alloc.pSetLayouts = &one.layout;
        if (VkResult r = vkAllocateDescriptorSets(self->device_, &one_alloc, &one.set); r != VK_SUCCESS) {
            return std::unexpected(map_vk_error(r, "vkAllocateDescriptorSets (set 1)"));
        }
        std::vector<VkWriteDescriptorSet> one_writes(subset.size());
        for (std::size_t i = 0; i < subset.size(); ++i) {
            one_writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            one_writes[i].dstSet = one.set;
            one_writes[i].dstBinding = subset[i];
            one_writes[i].descriptorCount = 1;
            one_writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            one_writes[i].pBufferInfo = &one_infos[i];
        }
        vkUpdateDescriptorSets(self->device_, static_cast<uint32_t>(one_writes.size()), one_writes.data(), 0, nullptr);
    }

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
                                          bool to_device, const std::vector<Entry*>& targets,
                                          const std::vector<Entry*>* zero_fill) {
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (VkResult r = vkBeginCommandBuffer(cmd, &begin); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkBeginCommandBuffer (StateMirror copy batch)"));
    }

    if (zero_fill != nullptr) {
        for (const Entry* e : *zero_fill) {
            if (e->byte_size > 0) vkCmdFillBuffer(cmd, e->device_buffer, 0, e->byte_size, 0u);
        }
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

// run_copy_batch() on this mirror's own queue, command buffer and fence, counting what crossed the host/device boundary: the
// byte size of each region it issued (an entry with no bytes issues none). The count moves only when the batch completed.
Result<void> StateMirror::copy_batch(bool to_device, const std::vector<Entry*>& targets) {
    if (Result<void> copied = run_copy_batch(device_, cmd_, queue_, fence_, to_device, targets); !copied) return copied;
    uint64_t moved = 0;
    for (const Entry* e : targets) moved += e->byte_size;
    (to_device ? transfer_.uploaded : transfer_.read_back) += moved;
    return {};
}

// One resident array, whole, between its device buffer and the shared staging buffer: run_copy_batch() copies between an
// entry's own two halves, so it is handed a COPY of the entry whose staging half is the shared buffer (the copy owns
// nothing; destroy_entry() is only ever called on the originals).
Result<void> StateMirror::copy_resident(Entry& e, bool to_device) {
    Entry view = e;
    view.staging_buffer = resident_staging_.staging_buffer;
    std::vector<Entry*> targets{&view};
    return copy_batch(to_device, targets);
}

Result<void> StateMirror::upload(const ArenaSet& arenas) {
    std::vector<Entry*> touched;
    touched.reserve(entries_.size());
    std::size_t seen = 0;  // every walk entry, resident ones too: the registry must still match the mirror entry for entry
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
        ++seen;
        if (e->resident) return;  // the step does not upload a resident array (upload_resident() does)
        if (e->byte_size > 0) {
            std::memcpy(e->staging_mapped, array.data, static_cast<std::size_t>(e->byte_size));
        }
        touched.push_back(e);
    });
    if (!ok) return std::unexpected(err);
    if (seen != entries_.size()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::upload: arenas registry has " + std::to_string(seen) +
                                         " entries, this mirror expects " + std::to_string(entries_.size())});
    }

    if (Result<void> copied = copy_batch(true, touched); !copied) {
        return copied;
    }
    ++upload_count_;
    return {};
}

Result<void> StateMirror::readback(ArenaSet& arenas) {
    std::vector<Entry*> touched;
    touched.reserve(entries_.size());
    std::size_t seen = 0;
    bool ok = true;
    Error err{Code::internal, ""};

    // Pass 1: validate every entry's shape WITHOUT writing a single
    // destination byte -- see backend.hpp's readback() doc comment. A
    // mismatch discovered at entry 7 of 22 must not have already overwritten
    // entries 0..6 with device bytes while leaving 8..21 untouched and the
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
        ++seen;
        if (!e->resident) touched.push_back(e);  // the step does not read a resident array back (readback_resident() does)
    });
    if (!ok) return std::unexpected(err);
    if (seen != entries_.size()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::readback: arenas registry has " + std::to_string(seen) +
                                         " entries, this mirror expects " + std::to_string(entries_.size())});
    }

    if (Result<void> copied = copy_batch(false, touched); !copied) {
        return copied;
    }

    // Pass 2: every device->staging copy above has completed (the fence
    // wait inside run_copy_batch already blocked for it), so it is now safe
    // to fold the staging bytes back into the arenas the caller owns.
    arenas.registry().for_each_array([&](const RegisteredArray& array) {
        const Entry* e = find(array.name);
        if (e != nullptr && !e->resident && e->byte_size > 0) {
            std::memcpy(array.data, e->staging_mapped, static_cast<std::size_t>(e->byte_size));
        }
    });
    return {};
}

// Both resident moves check every resident array's shape against `arenas` before a byte moves: a resident array the registry
// does not carry, or carries in another shape, is invalid_argument naming it.
Result<void> StateMirror::upload_resident(const ArenaSet& arenas) {
    std::vector<std::pair<Entry*, const void*>> moves;
    bool ok = true;
    Error err{Code::internal, ""};
    arenas.registry().for_each_array([&](const RegisteredArray& array) {
        if (!ok) return;
        Entry* e = find(array.name);
        if (e == nullptr || !e->resident) return;
        if (e->elem_size != array.elem_size || e->world_count != array.world_count ||
            e->capacity_per_world != array.capacity_per_world) {
            ok = false;
            err = Error{Code::invalid_argument,
                        "StateMirror::upload_resident: shape mismatch for resident array '" + array.name + "'"};
            return;
        }
        moves.emplace_back(e, array.data);
    });
    if (!ok) return std::unexpected(err);
    const auto resident_count = static_cast<std::size_t>(
        std::ranges::count_if(entries_, [](const Entry& e) { return e.resident; }));
    if (moves.size() != resident_count) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::upload_resident: arenas registry holds " + std::to_string(moves.size()) +
                                         " of the " + std::to_string(resident_count) + " resident arrays"});
    }
    // One array at a time through the one staging buffer: the next array's bytes may overwrite it only once this copy is done.
    for (const auto& [e, data] : moves) {
        if (e->byte_size == 0) continue;
        std::memcpy(resident_staging_.staging_mapped, data, static_cast<std::size_t>(e->byte_size));
        if (Result<void> copied = copy_resident(*e, /*to_device=*/true); !copied) return copied;
    }
    return {};
}

Result<void> StateMirror::readback_resident(ArenaSet& arenas) {
    std::vector<std::pair<Entry*, void*>> moves;
    bool ok = true;
    Error err{Code::internal, ""};
    arenas.registry().for_each_array([&](const RegisteredArray& array) {
        if (!ok) return;
        Entry* e = find(array.name);
        if (e == nullptr || !e->resident) return;
        if (e->elem_size != array.elem_size || e->world_count != array.world_count ||
            e->capacity_per_world != array.capacity_per_world) {
            ok = false;
            err = Error{Code::invalid_argument,
                        "StateMirror::readback_resident: shape mismatch for resident array '" + array.name + "'"};
            return;
        }
        moves.emplace_back(e, array.data);
    });
    if (!ok) return std::unexpected(err);
    const auto resident_count = static_cast<std::size_t>(
        std::ranges::count_if(entries_, [](const Entry& e) { return e.resident; }));
    if (moves.size() != resident_count) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::readback_resident: arenas registry holds " + std::to_string(moves.size()) +
                                         " of the " + std::to_string(resident_count) + " resident arrays"});
    }
    for (const auto& [e, data] : moves) {
        if (e->byte_size == 0) continue;
        if (Result<void> copied = copy_resident(*e, /*to_device=*/false); !copied) return copied;
        std::memcpy(data, resident_staging_.staging_mapped, static_cast<std::size_t>(e->byte_size));
    }
    return {};
}

Result<std::vector<std::byte>> StateMirror::read_config_image() {
    if (config_.device_buffer == VK_NULL_HANDLE) {
        return std::unexpected(Error{Code::unavailable,
                                     "StateMirror::read_config_image: no module bound a configuration image "
                                     "(StepShape::config_image / config_binding)"});
    }
    // A diagnostic read: run_copy_batch() directly, so the module's own transfer count is not moved by looking.
    std::vector<Entry*> targets{&config_};
    if (Result<void> copied = run_copy_batch(device_, cmd_, queue_, fence_, /*to_device=*/false, targets); !copied) {
        return std::unexpected(copied.error());
    }
    std::vector<std::byte> out(static_cast<std::size_t>(config_.byte_size));
    std::memcpy(out.data(), config_.staging_mapped, out.size());
    return out;
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
    return copy_batch(true, targets);
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
    return copy_batch(true, targets);
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
    return copy_batch(true, targets);
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
    return copy_batch(true, targets);
}

Result<void> StateMirror::upload_table(uint32_t index, std::span<const std::byte> bytes) {
    if (index >= table_entries_.size()) {
        return std::unexpected(Error{Code::invalid_argument, "StateMirror::upload_table: table " +
                                                                 std::to_string(index) + " of " +
                                                                 std::to_string(table_entries_.size())});
    }
    Entry& e = table_entries_[index];
    const auto size = static_cast<std::size_t>(e.byte_size);
    if (bytes.size() > size) {
        return std::unexpected(Error{Code::invalid_argument, "StateMirror::upload_table: table '" + e.name +
                                                                 "' holds " + std::to_string(size) + " bytes, got " +
                                                                 std::to_string(bytes.size())});
    }
    // The floats, then zeros to the capacity, so the device holds exactly the
    // host's table and nothing an earlier, longer one left behind.
    if (!bytes.empty()) std::memcpy(e.staging_mapped, bytes.data(), bytes.size());
    if (size > bytes.size()) {
        std::memset(static_cast<std::byte*>(e.staging_mapped) + bytes.size(), 0, size - bytes.size());
    }
    std::vector<Entry*> targets{&e};
    return copy_batch(true, targets);
}

Result<void> StateMirror::read_table(uint32_t index, std::span<std::byte> out_bytes) {
    if (index >= table_entries_.size()) {
        return std::unexpected(Error{Code::invalid_argument, "StateMirror::read_table: table " +
                                                                 std::to_string(index) + " of " +
                                                                 std::to_string(table_entries_.size())});
    }
    Entry& e = table_entries_[index];
    if (out_bytes.size() != static_cast<std::size_t>(e.byte_size)) {
        return std::unexpected(Error{Code::invalid_argument, "StateMirror::read_table: table '" + e.name +
                                                                 "' is " + std::to_string(e.byte_size) +
                                                                 " bytes, got a span of " +
                                                                 std::to_string(out_bytes.size())});
    }
    std::vector<Entry*> targets{&e};
    if (Result<void> copied = copy_batch(false, targets);
        !copied) {
        return copied;
    }
    if (!out_bytes.empty()) std::memcpy(out_bytes.data(), e.staging_mapped, out_bytes.size());
    return {};
}

Result<void> StateMirror::read_step_witness(std::span<std::byte> out_bytes) {
    const std::size_t expected = static_cast<std::size_t>(step_witness_.byte_size);
    if (out_bytes.size() != expected) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StateMirror::read_step_witness: expected " + std::to_string(expected) +
                                         " bytes, got " + std::to_string(out_bytes.size())});
    }
    std::vector<Entry*> targets{&step_witness_};
    if (Result<void> copied = copy_batch(false, targets);
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
    if (Result<void> copied = copy_batch(false, targets);
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
    if (Result<void> copied = copy_batch(false, targets);
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
    for (SetOneSet& one : set1_) {
        if (one.layout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device_, one.layout, nullptr);
    }
    set1_.clear();  // the sets themselves were freed with the pool

    for (Entry& e : set_one_scratch_) destroy_entry(device_, e);
    set_one_scratch_.clear();
    destroy_entry(device_, resident_staging_);
    destroy_entry(device_, config_);
    for_each_derived([this](Entry& e) { destroy_entry(device_, e); });
    scratch_entries_.clear();
    table_entries_.clear();
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

uint64_t mirror_allocation_count() noexcept { return g_mirror_allocations.load(std::memory_order_relaxed); }

MemoryReport StateMirror::memory_report() {
    MemoryReport report;
    VkPhysicalDeviceMemoryProperties props{};
    vkGetPhysicalDeviceMemoryProperties(physical_device_, &props);
    const auto add = [&](const Entry& e, MemoryKind kind) {
        const auto record = [&](bool staging, uint32_t type) {
            MemoryRecord r;
            r.name = e.name;
            r.kind = kind;
            r.staging = staging;
            r.bytes = e.byte_size;
            r.memory_type = type;
            if (type < props.memoryTypeCount) {
                r.heap = props.memoryTypes[type].heapIndex;
                r.device_local = (props.memoryTypes[type].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0;
            }
            (staging ? report.staging_bytes : report.device_bytes) += r.bytes;
            report.buffers.push_back(std::move(r));
        };
        if (e.device_buffer != VK_NULL_HANDLE) record(false, e.device_type);
        if (e.staging_buffer != VK_NULL_HANDLE) record(true, e.staging_type);
    };
    for (const Entry& e : entries_) {
        add(e, MemoryKind::walk);
        report.host_arena_bytes += e.byte_size;
    }
    for (Entry* e : derived_entries()) add(*e, MemoryKind::derived);
    for (const Entry& e : scratch_entries_) {
        add(e, e.name.ends_with(" (absent)") ? MemoryKind::dummy : MemoryKind::scratch);
    }
    for (const Entry& e : table_entries_) add(e, MemoryKind::table);
    // Set 1's (stage 7 Tasks C1/C2): the device-only scratch, then the ONE shared staging buffer of the resident arrays (a
    // resident array's own record has no staging twin; this is the single twin, staging only) and the configuration image.
    for (const Entry& e : set_one_scratch_) add(e, MemoryKind::scratch);
    if (resident_staging_.staging_buffer != VK_NULL_HANDLE) add(resident_staging_, MemoryKind::walk);
    if (config_.device_buffer != VK_NULL_HANDLE) add(config_, MemoryKind::table);
    return report;
}

Result<AdmitShape> admit_shape_for(const StepShape& shape, uint32_t /*workgroup_size*/) {
    AdmitShape out;
    // Set 0 is the only set the mirror builds; module kernels' set-1 subsets are the layout classes' to add (batch B).
    out.layouts.push_back(AdmitLayout{.name = "state", .bindings_per_set = {gen::kBindingCount_state}});
    out.push_bytes = gen::kPushConstantSize;
    // Every walk entry first, as create() makes them, then the derived buffers (derived_buffer_shapes' order).
    for (const StateArrayShape& array : shape.arrays) {
        Result<VkDeviceSize> bytes =
            buffer_bytes(array.name, array.elem_size, uint64_t{shape.world_count} * uint64_t{array.capacity_per_world});
        if (!bytes) return std::unexpected(bytes.error());
        out.buffers.push_back(AdmitBuffer{.name = array.name, .bytes = *bytes});
    }
    Result<std::vector<DerivedBufferShape>> derived = derived_buffer_shapes(shape, gen::kBindingCount_state);
    if (!derived) return std::unexpected(derived.error());
    for (const DerivedBufferShape& d : *derived) {
        out.buffers.push_back(AdmitBuffer{.name = d.name, .bytes = d.byte_size});
    }
    // Set 1's buffers (stage 7 Tasks C1/C2): the set-1 scratch (a scratch naming no set-0 binding whose name is in
    // set_one_bindings.hpp), the resident arrays' one shared staging buffer, the configuration image.
    for (const ScratchShape& scratch : shape.scratch) {
        if (scratch.binding != kNoBinding || set_one_binding_of(scratch.name) == kNoBinding) continue;
        Result<VkDeviceSize> bytes = buffer_bytes(scratch.name, scratch.elem_size,
                                                  uint64_t{shape.world_count} * uint64_t{scratch.capacity_per_world});
        if (!bytes) return std::unexpected(bytes.error());
        out.buffers.push_back(AdmitBuffer{.name = scratch.name, .bytes = *bytes});
    }
    uint64_t largest_resident = 0;
    bool any_resident = false;
    for (const StateArrayShape& array : shape.arrays) {
        if (!array.resident) continue;
        any_resident = true;
        largest_resident = std::max<uint64_t>(
            largest_resident, uint64_t{shape.world_count} * uint64_t{array.capacity_per_world} * uint64_t{array.elem_size});
    }
    if (any_resident) out.buffers.push_back(AdmitBuffer{.name = "resident_staging", .bytes = largest_resident});
    if (!shape.config_image.empty()) {
        out.buffers.push_back(AdmitBuffer{.name = "airflow_config", .bytes = shape.config_image.size()});
    }
    return out;
}

bool StateMirror::has_staging(std::string_view name) {
    for (const Entry& e : entries_) {
        if (e.name == name) return e.staging_buffer != VK_NULL_HANDLE;
    }
    bool found = false;
    bool staged = false;
    for_each_derived([&](Entry& e) {
        if (!found && e.name == name) {
            found = true;
            staged = e.staging_buffer != VK_NULL_HANDLE;
        }
    });
    return found && staged;
}

uint64_t StateMirror::host_visible_bytes() {
    uint64_t total = 0;
    for (const Entry& e : entries_) {
        if (e.staging_buffer != VK_NULL_HANDLE) total += e.byte_size;
    }
    for_each_derived([&](Entry& e) {
        if (e.staging_buffer != VK_NULL_HANDLE) total += e.byte_size;
    });
    if (resident_staging_.staging_buffer != VK_NULL_HANDLE) total += resident_staging_.byte_size;
    if (config_.staging_buffer != VK_NULL_HANDLE) total += config_.byte_size;
    return total;
}

}  // namespace spade::compute
