#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace spade {

// Slot-stable value container.
//
// create() returns a slot index (uint32_t) that identifies the stored value
// for the rest of that value's lifetime. destroy() tombstones the slot --
// it destroys the stored T and marks the slot free for reuse -- but it NEVER
// moves or renumbers any OTHER slot ("no swap-and-pop"). This is
// load-bearing for later engine layers (GPU-mirrored per-slot state arrays;
// see the engine design spec's ECS section): once a caller has a slot
// index, that index keeps meaning the same thing until that specific slot
// is destroyed, full stop.
//
// Freed slots are recycled through a free-list (LIFO) so create() prefers
// reusing the most-recently-freed slot over growing the backing array. LIFO
// (as opposed to FIFO, or "lowest free index") is an implementation choice,
// but it must stay *deterministic*: Registry's churn-determinism test
// depends on two identical create/destroy sequences producing identical
// slot assignments, which only holds if free-list selection order never
// varies run to run.
//
// Pool<T> is a bare mechanism: it knows nothing about entities, components,
// or generations. Registry composes it with entity/generation bookkeeping
// and a sparse entity-index -> slot map to build the component storage the
// ECS actually exposes (see registry.hpp).
template <class T>
class Pool {
public:
    Pool() = default;
    ~Pool() = default;
    Pool(const Pool&) = default;
    Pool& operator=(const Pool&) = default;
    Pool(Pool&&) = default;
    Pool& operator=(Pool&&) = default;

    // Stores `value` in a slot (reused from the free-list if one is
    // available, else appended to the backing array) and returns that
    // slot's index.
    uint32_t create(T value) {
        if (!free_list_.empty()) {
            uint32_t slot = free_list_.back();
            free_list_.pop_back();
            slots_[slot].emplace(std::move(value));
            return slot;
        }
        slots_.emplace_back(std::move(value));
        return static_cast<uint32_t>(slots_.size() - 1);
    }

    // Tombstones `slot`: destroys the stored value and pushes the slot onto
    // the free-list. No-op (returns false) if `slot` is out of range or
    // already tombstoned, so callers that may double-destroy don't need to
    // guard themselves first. Returns true if a live value was removed.
    bool destroy(uint32_t slot) {
        if (!alive(slot)) return false;
        slots_[slot].reset();
        free_list_.push_back(slot);
        return true;
    }

    [[nodiscard]] bool alive(uint32_t slot) const noexcept {
        return slot < slots_.size() && slots_[slot].has_value();
    }

    [[nodiscard]] T* get(uint32_t slot) noexcept { return alive(slot) ? &*slots_[slot] : nullptr; }
    [[nodiscard]] const T* get(uint32_t slot) const noexcept {
        return alive(slot) ? &*slots_[slot] : nullptr;
    }

    // Backing array size, including tombstoned slots.
    [[nodiscard]] size_t capacity() const noexcept { return slots_.size(); }

    // Count of currently-live slots.
    [[nodiscard]] size_t size() const noexcept { return slots_.size() - free_list_.size(); }

    // Ascending slot-order visitation, skipping tombstones. fn is called as
    // fn(slot_index, T&).
    template <class F>
    void for_each(F&& fn) {
        for (uint32_t i = 0; i < slots_.size(); ++i) {
            if (slots_[i].has_value()) fn(i, *slots_[i]);
        }
    }
    template <class F>
    void for_each(F&& fn) const {
        for (uint32_t i = 0; i < slots_.size(); ++i) {
            if (slots_[i].has_value()) fn(i, *slots_[i]);
        }
    }

private:
    std::vector<std::optional<T>> slots_;
    std::vector<uint32_t> free_list_;
};

}  // namespace spade
