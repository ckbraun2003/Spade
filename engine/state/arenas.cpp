#include "state/arenas.hpp"

#include <cstring>

namespace spade {

namespace {

Error bad_array(const char* what) { return Error{Code::not_found, std::string{"ArenaSet: "} + what}; }

Error bad_arg(const std::string& what) { return Error{Code::invalid_argument, "ArenaSet: " + what}; }

}  // namespace

Result<ArrayIndex> ArenaSet::register_array_impl(std::string name, uint32_t elem_size, uint32_t capacity_per_world) {
    if (capacity_per_world == 0) {
        return std::unexpected(bad_arg("register_array('" + name + "'): capacity_per_world must be non-zero"));
    }

    // Global element count must stay addressable by a uint32 slot index --
    // slot indices are uint32 everywhere (including the slot_to_world map
    // that the GPU reads), so this ceiling is a layout fact, not a guess.
    const uint64_t requested = uint64_t{world_count_} * capacity_per_world;
    if (requested > std::numeric_limits<uint32_t>::max()) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "ArenaSet::register_array('" + name + "'): world_count * capacity_per_world exceeds 2^32-1"});
    }
    const auto total_slots = static_cast<uint32_t>(requested);

    static_assert(sizeof(std::size_t) >= 8,
                  "byte_size below is computed in size_t and assumes a 64-bit target (uint32 slots * uint32 stride)");
    const std::size_t byte_size = std::size_t{total_slots} * elem_size;

    Arena arena;
    arena.name = std::move(name);
    arena.elem_size = elem_size;
    arena.capacity_per_world = capacity_per_world;
    arena.world_count = world_count_;

    if (byte_size != 0) {
        // nothrow: no exceptions cross an engine module boundary, so an
        // allocation failure becomes capacity_exceeded rather than a
        // std::bad_alloc escaping through Result<>. Array form + align_val_t
        // here must match AlignedArenaDelete exactly.
        void* raw = ::operator new[](byte_size, std::align_val_t{kStd430StructAlignment}, std::nothrow);
        if (raw == nullptr) {
            return std::unexpected(
                Error{Code::capacity_exceeded, "ArenaSet::register_array('" + arena.name + "'): arena allocation failed"});
        }
        // Zero-fill so that every byte of the arena -- including struct tail
        // padding and every not-yet-allocated slot -- is deterministic from
        // the moment the array exists.
        std::memset(raw, 0, byte_size);
        arena.bytes.reset(static_cast<std::byte*>(raw));
    }

    arena.slot_to_world.assign(total_slots, kInvalidWorld);
    arena.worlds.resize(world_count_);

    // Register BEFORE the arena is moved into arenas_, and reserve the slot
    // it will land in first. Both `data` pointers below are already final at
    // this point -- the byte block comes from operator new[] and the map from
    // the vector's own buffer, and moving an Arena (into arenas_, or during a
    // later reallocation, or when the whole ArenaSet is moved) transfers both
    // pointers rather than copying the storage. So the ordering that matters
    // is: reserve (may fail) -> register (may fail) -> push_back (cannot
    // fail), which leaves no half-registered state on any path.
    arenas_.reserve(arenas_.size() + 1);

    std::vector<RegisteredArray> descs;
    descs.reserve(2);
    descs.push_back(RegisteredArray{
        .name = arena.name,
        .elem_size = elem_size,
        .world_count = world_count_,
        .capacity_per_world = capacity_per_world,
        .data = arena.bytes.get(),
    });
    descs.push_back(RegisteredArray{
        .name = arena.name + std::string{kSlotToWorldSuffix},
        .elem_size = sizeof(uint32_t),
        .world_count = world_count_,
        .capacity_per_world = capacity_per_world,
        .data = reinterpret_cast<std::byte*>(arena.slot_to_world.data()),
    });
    // All-or-nothing: a duplicate name on either entry leaves the registry
    // untouched, and the local `arena` is simply destroyed on the way out.
    if (Result<void> registered = registry_.register_arrays(std::move(descs)); !registered) {
        return std::unexpected(registered.error());
    }

    static_assert(std::is_nothrow_move_constructible_v<Arena>,
                  "push_back below must not throw after the registry has been committed");
    arenas_.push_back(std::move(arena));
    return ArrayIndex{static_cast<uint32_t>(arenas_.size() - 1)};
}

Result<void> ArenaSet::resync_from_slot_to_world(ArrayIndex array) {
    Result<const Arena*> found = arena_for(array, 0);
    if (!found) return std::unexpected(found.error());
    Arena& arena = arenas_[array.value];

    // Validate the whole map before mutating anything: a restored blob whose
    // map disagrees with this arena's partition shape must be rejected, not
    // half-applied.
    for (uint32_t slot = 0; slot < arena.total_slots(); ++slot) {
        const uint32_t world = arena.slot_to_world[slot];
        if (world == kInvalidWorld) continue;
        if (world >= arena.world_count || slot / arena.capacity_per_world != world) {
            return std::unexpected(bad_arg("array '" + arena.name + "': slot_to_world entry names the wrong world"));
        }
    }

    for (uint32_t world = 0; world < arena.world_count; ++world) {
        WorldSlots rebuilt;
        const uint32_t begin = world * arena.capacity_per_world;
        // The bump cursor's canonical value is one past the highest live
        // slot; everything free below that goes on the ordered free list.
        for (uint32_t i = 0; i < arena.capacity_per_world; ++i) {
            if (arena.slot_to_world[begin + i] != kInvalidWorld) {
                ++rebuilt.live;
                rebuilt.next_unused = i + 1;
            }
        }
        for (uint32_t i = 0; i < rebuilt.next_unused; ++i) {
            if (arena.slot_to_world[begin + i] == kInvalidWorld) rebuilt.freed.push(begin + i);
        }
        arena.worlds[world] = std::move(rebuilt);
    }
    return {};
}

Result<const ArenaSet::Arena*> ArenaSet::arena_for(ArrayIndex array, uint32_t expected_elem_size) const {
    if (array.value >= arenas_.size()) {
        return std::unexpected(bad_array("unknown array id"));
    }
    const Arena& arena = arenas_[array.value];
    // expected_elem_size == 0 means "untyped caller, do not check".
    if (expected_elem_size != 0 && arena.elem_size != expected_elem_size) {
        return std::unexpected(bad_arg("array '" + arena.name + "': element size mismatch (wrong ArrayId type, or an id from another ArenaSet)"));
    }
    return &arena;
}

Result<WorldRange> ArenaSet::range(ArrayIndex array, uint32_t world) const {
    Result<const Arena*> found = arena_for(array, 0);
    if (!found) return std::unexpected(found.error());
    const Arena& arena = **found;
    if (world >= arena.world_count) {
        return std::unexpected(bad_arg("array '" + arena.name + "': world id out of range"));
    }
    // The partition arithmetic, in one place. begin cannot overflow: it is
    // bounded by total_slots, which register_array_impl capped at 2^32-1.
    return WorldRange{.begin = world * arena.capacity_per_world, .count = arena.capacity_per_world};
}

Result<uint32_t> ArenaSet::alloc_slot(ArrayIndex array, uint32_t world) {
    Result<const Arena*> found = arena_for(array, 0);
    if (!found) return std::unexpected(found.error());
    Arena& arena = arenas_[array.value];
    if (world >= arena.world_count) {
        return std::unexpected(bad_arg("array '" + arena.name + "': world id out of range"));
    }

    WorldSlots& slots = arena.worlds[world];
    const uint32_t begin = world * arena.capacity_per_world;

    uint32_t slot = 0;
    if (!slots.freed.empty()) {
        // Ordered free list: the heap minimum is the lowest free slot in
        // this partition, because every freed slot sits below next_unused.
        slot = slots.freed.top();
        slots.freed.pop();
    } else if (slots.next_unused < arena.capacity_per_world) {
        slot = begin + slots.next_unused;
        ++slots.next_unused;
    } else {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "ArenaSet::alloc_slot: array '" + arena.name + "' world partition is full"});
    }

    arena.slot_to_world[slot] = world;
    ++slots.live;
    return slot;
}

Result<void> ArenaSet::free_slot(ArrayIndex array, uint32_t slot) {
    Result<const Arena*> found = arena_for(array, 0);
    if (!found) return std::unexpected(found.error());
    Arena& arena = arenas_[array.value];
    if (slot >= arena.total_slots()) {
        return std::unexpected(bad_arg("array '" + arena.name + "': slot index out of range"));
    }

    const uint32_t world = arena.slot_to_world[slot];
    if (world == kInvalidWorld) {
        return std::unexpected(
            Error{Code::not_found, "ArenaSet::free_slot: array '" + arena.name + "' slot is not allocated (double free?)"});
    }

    // Push onto the free list FIRST: it is the only step here that can
    // allocate, so if it throws nothing has been mutated yet and the slot is
    // still cleanly live. Everything after it is noexcept.
    WorldSlots& slots = arena.worlds[world];
    slots.freed.push(slot);
    arena.slot_to_world[slot] = kInvalidWorld;
    --slots.live;

    // A freed slot reads back as zeroes, exactly like a never-allocated one.
    // Without this an arena's bytes would depend on data that is logically
    // gone, and two runs at the same logical state could produce different
    // snapshot blobs.
    std::memset(arena.bytes.get() + std::size_t{slot} * arena.elem_size, 0, arena.elem_size);
    return {};
}

Result<std::span<const uint32_t>> ArenaSet::slot_to_world(ArrayIndex array) const {
    Result<const Arena*> found = arena_for(array, 0);
    if (!found) return std::unexpected(found.error());
    return std::span<const uint32_t>((*found)->slot_to_world);
}

Result<uint32_t> ArenaSet::live_count(ArrayIndex array, uint32_t world) const {
    Result<const Arena*> found = arena_for(array, 0);
    if (!found) return std::unexpected(found.error());
    const Arena& arena = **found;
    if (world >= arena.world_count) {
        return std::unexpected(bad_arg("array '" + arena.name + "': world id out of range"));
    }
    return arena.worlds[world].live;
}

}  // namespace spade
