#pragma once

#include <cstdint>

namespace spade {

// Generational handle: an index into a slot-based pool plus a generation
// counter the pool bumps every time that slot is despawned and reused.
// Equality compares both fields, so a handle captured before its slot was
// recycled compares unequal to (and is thus stale relative to) a fresh
// handle issued for the same index -- that comparison IS the invalidation
// check; there is no separate "is this handle still alive" query here
// because that would require a live registry tracking every issued handle's
// current generation, and this header is deliberately dependency-free (no
// registry type named or included) so any resource kind can reuse Handle<Tag>
// without pulling one in.
//
// Tag is a phantom type: it only exists to keep handles for different
// resource kinds (bodies, worlds, ...) from being interchangeable at the
// type level. It is never instantiated and need not be a complete type.
template <class Tag>
struct Handle {
    uint32_t index = 0;
    uint32_t generation = 0;

    [[nodiscard]] constexpr bool is_null() const noexcept { return generation == 0; }

    friend constexpr bool operator==(const Handle&, const Handle&) noexcept = default;
};

// Pools reserve generation 0 to mean "never issued", so a default-constructed
// Handle<Tag>{} is always null and always equal to kNull<Tag>.
template <class Tag>
inline constexpr Handle<Tag> kNull{};

}  // namespace spade
