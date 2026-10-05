// state/state_view.hpp -- a pass's view of one module array it declared
// (module-API stage 4, Task 6; sim/module.hpp's CompiledBinding).
//
// A pass reads and writes the module state it declares through
// physics::SubstepContext::state: one StateView per declared access, in the
// pass's own declaration order. A view is untyped -- the arena's bytes, all
// worlds, world-contiguous -- and world_rows<T>() types one world's rows.
//
// ABSENT is a view with no data. A core quantity, a field, a stateless
// module's token and an optional read of a module the set does not contain all
// bind absent views, so a pass indexes ctx.state by its own access list
// whatever the set holds.
//
// Non-owning, like every view a pass sees: Simulation refills the views in
// place from the arenas, which never move, so nothing here is allocated.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace spade {

struct StateView {
    std::byte* data = nullptr;        // null when absent
    uint32_t elem_size = 0;           // bytes per row
    uint32_t world_count = 0;
    uint32_t capacity_per_world = 0;  // rows per world; world w owns rows [w * capacity, (w + 1) * capacity)
    [[nodiscard]] bool present() const noexcept { return data != nullptr; }
};

// World `world`'s rows of the array, indexed by world-local slot, free rows
// included (they read as zeroes). Empty if the view is absent, if T is not the
// array's row size, or if the world is outside the set.
template <class T>
[[nodiscard]] std::span<T> world_rows(const StateView& v, uint32_t world) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "a row is raw arena bytes");
    if (v.data == nullptr || sizeof(T) != v.elem_size || world >= v.world_count) return {};
    std::byte* const begin = v.data + static_cast<std::size_t>(world) * v.capacity_per_world * v.elem_size;
    return std::span<T>(reinterpret_cast<T*>(begin), v.capacity_per_world);
}

}  // namespace spade
