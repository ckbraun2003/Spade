#pragma once

#include <cstdint>

namespace spade {

// The engine's only notion of time: a monotonic count of physics steps,
// advanced by exactly one per step by the schedule (Task 13). Nothing under
// spade/engine/ reads the wall clock (see the plan's global constraints) --
// Tick is the deterministic substitute, and structural changes (spawn,
// despawn, world load) apply only at Tick boundaries via the structural
// queue.
struct Tick {
    uint64_t value = 0;

    friend constexpr bool operator==(const Tick&, const Tick&) noexcept = default;
    friend constexpr auto operator<=>(const Tick&, const Tick&) noexcept = default;

    constexpr Tick& operator++() noexcept {
        ++value;
        return *this;
    }

    constexpr Tick operator++(int) noexcept {
        Tick prev = *this;
        ++value;
        return prev;
    }

    // Offset by a step count in either direction.
    friend constexpr Tick operator+(Tick lhs, uint64_t steps) noexcept {
        return Tick{lhs.value + steps};
    }
    friend constexpr Tick operator-(Tick lhs, uint64_t steps) noexcept {
        return Tick{lhs.value - steps};
    }

    // Distance between two ticks, in steps.
    friend constexpr uint64_t operator-(Tick lhs, Tick rhs) noexcept {
        return lhs.value - rhs.value;
    }
};

}  // namespace spade
