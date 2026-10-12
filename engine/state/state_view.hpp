// state/state_view.hpp -- a pass's view of one module array, configuration
// table or scratch it declared (module-API stage 4, Task 6; sim/module.hpp's
// CompiledBinding).
//
// A pass reads and writes the module state it declares through
// physics::SubstepContext::state: one StateView per declared access, in the
// pass's own declaration order. A view is untyped -- the bytes of every world,
// world-contiguous -- and three accessors type it:
//   world_rows<T>(view, w)  world w's rows;
//   all_rows<T>(view)       every world's rows at once, world w's starting at
//                           row w * capacity_per_world, as a batched sweep
//                           reads them;
//   table_rows<T>(view)     a configuration table's rows, with no world.
//
// A CONFIGURATION TABLE (Task 7) is SHARED (stage 5, Task 1): one set of rows,
// elem_size 4, that every world reads. Its view carries the set's world_count
// and `shared`, and world_rows returns the whole table for every world in the
// set, so a pass that reads its table inside its world loop reads the same rows
// in every world. table_rows reads it with no world. A table is read-only by
// declaration (compile_schedule refuses a write or accumulate; the view's bytes
// are not const). A declared scratch (Task 7b) is viewed exactly like an array,
// over rows the Simulation allocated beside the arenas rather than in them.
//
// ABSENT is a view with no data. A core quantity, a field, a stateless
// module's token and an optional read of a module the set does not contain all
// bind absent views, so a pass indexes ctx.state by its own access list
// whatever the set holds. Every accessor reads an absent view as empty, and
// that is no fault.
//
// A MISUSE IS A FAULT, NEVER A SILENT EMPTY SPAN (L6): a row type whose size is
// not the view's elem_size, a world outside the set, table_rows on a view that
// is not a table, or all_rows on a table. The accessor records the misuse in
// the ViewFault its view points at, and returns an empty span: a pass is
// noexcept and must be able to keep calling the accessors, so nothing is
// thrown. The record is STICKY: the first misuse stays, and no later call,
// correct or not, clears or overwrites it. Simulation::step() reads it at the
// end of the step and refuses with invalid_argument, naming the module, the
// pass and the view; the Simulation is then poisoned, refusing every later
// step and snapshot until a restore (sim/simulation.hpp's step()). The checks
// are the same in every build, Release included. A view with no ViewFault --
// one built by hand, not by a Simulation -- has nowhere to record, and records
// nothing.
//
// Non-owning, like every view a pass sees: Simulation refills the views in
// place from storage that never moves, so nothing here is allocated.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace spade {

// What a pass did wrong with a view.
enum class ViewMisuse : uint8_t {
    none = 0,
    row_type = 1,   // sizeof(T) is not the view's elem_size
    world = 2,      // a world outside the set
    not_table = 3,  // table_rows on a view that is not a configuration table
    table = 4,      // all_rows on a configuration table
};

// The first misuse of any view that points here, kept until Simulation::step()
// reports it.
struct ViewFault {
    ViewMisuse misuse = ViewMisuse::none;
    uint32_t view = 0;      // the misused view's StateView::id
    uint32_t row_size = 0;  // the sizeof(T) the accessor was called with
    uint32_t world = 0;     // the world asked for (ViewMisuse::world)
};

struct StateView {
    std::byte* data = nullptr;        // null when absent
    uint32_t elem_size = 0;           // bytes per row
    uint32_t world_count = 0;         // the set's worlds, a table's included
    uint32_t capacity_per_world = 0;  // rows per world; world w owns rows [w * capacity, (w + 1) * capacity)
    bool shared = false;              // a configuration table: one set of rows every world reads
    uint32_t id = 0;                  // which view this is, to the Simulation that built it
    ViewFault* fault = nullptr;       // where a misuse is recorded; null records nothing
    [[nodiscard]] bool present() const noexcept { return data != nullptr; }
};

namespace view_detail {
// Reached only by a misuse, never by a correct call.
inline void record_misuse(const StateView& v, ViewMisuse misuse, std::size_t row_size, uint32_t world) noexcept {
    if (v.fault == nullptr || v.fault->misuse != ViewMisuse::none) return;  // sticky: the first misuse stays
    *v.fault = ViewFault{.misuse = misuse, .view = v.id, .row_size = static_cast<uint32_t>(row_size), .world = world};
}
}  // namespace view_detail

// World `world`'s rows of the array, indexed by world-local slot, free rows
// included (they read as zeroes); for a configuration table, the whole table,
// whichever world is asked for. Empty for an absent view. A row type of another
// size, or a world outside the set, records a fault and reads as empty.
template <class T>
[[nodiscard]] std::span<T> world_rows(const StateView& v, uint32_t world) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "a row is raw arena bytes");
    if (v.data == nullptr) return {};
    if (sizeof(T) != v.elem_size) [[unlikely]] {
        view_detail::record_misuse(v, ViewMisuse::row_type, sizeof(T), world);
        return {};
    }
    if (world >= v.world_count) [[unlikely]] {
        view_detail::record_misuse(v, ViewMisuse::world, sizeof(T), world);
        return {};
    }
    const std::size_t first = v.shared ? 0 : static_cast<std::size_t>(world) * v.capacity_per_world;
    return std::span<T>(reinterpret_cast<T*>(v.data + first * v.elem_size), v.capacity_per_world);
}

// A configuration table's rows, read with no world. Empty for an absent view.
// A view that is not a table, or a row type of another size, records a fault
// and reads as empty.
template <class T>
[[nodiscard]] std::span<const T> table_rows(const StateView& v) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "a row is raw arena bytes");
    if (v.data == nullptr) return {};
    if (!v.shared) [[unlikely]] {
        view_detail::record_misuse(v, ViewMisuse::not_table, sizeof(T), 0);
        return {};
    }
    if (sizeof(T) != v.elem_size) [[unlikely]] {
        view_detail::record_misuse(v, ViewMisuse::row_type, sizeof(T), 0);
        return {};
    }
    return std::span<const T>(reinterpret_cast<const T*>(v.data), v.capacity_per_world);
}

// Every world's rows of an array or a scratch, contiguous: world w's rows are
// [w * capacity_per_world, (w + 1) * capacity_per_world). Empty for an absent
// view. A configuration table (read it with table_rows), or a row type of
// another size, records a fault and reads as empty.
template <class T>
[[nodiscard]] std::span<T> all_rows(const StateView& v) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "a row is raw arena bytes");
    if (v.data == nullptr) return {};
    if (v.shared) [[unlikely]] {
        view_detail::record_misuse(v, ViewMisuse::table, sizeof(T), 0);
        return {};
    }
    if (sizeof(T) != v.elem_size) [[unlikely]] {
        view_detail::record_misuse(v, ViewMisuse::row_type, sizeof(T), 0);
        return {};
    }
    return std::span<T>(reinterpret_cast<T*>(v.data), static_cast<std::size_t>(v.world_count) * v.capacity_per_world);
}

}  // namespace spade
