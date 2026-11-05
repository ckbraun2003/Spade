// physics/lock.hpp -- the translation lock's rows (module API stage 6, Task 1;
// docs/design/core/plans/2026-11-05-module-api-stage6-plan.md; spec section 9).
//
// The built-in `lock` module (sim/module.hpp's lock_module()) holds one
// per_body array, `lock_rows`: a body's anchor and the world axes held at it.
// Integrate reads it (an optional read, so a set without the module computes
// today's bits) and, after its ordinary arithmetic, selects per locked axis:
// pos = anchor, vel = 0, and the body's specific force reads -g on that axis.
// The rows are registered state, so a snapshot carries them.
//
// This header holds the row and the spawn record only: no sim/ include, so
// physics/integrator.hpp can read a row without the module API.
#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <glm/vec3.hpp>

namespace spade::physics {

// The axes a lock holds, as bits of LockRow::axes: world x, y, z.
inline constexpr uint32_t kLockAxisX = 1u;
inline constexpr uint32_t kLockAxisY = 2u;
inline constexpr uint32_t kLockAxisZ = 4u;
inline constexpr uint32_t kLockAllAxes = kLockAxisX | kLockAxisY | kLockAxisZ;

// One row, at the body's own slot. A ZERO ROW IS FREE (axes == 0), so a fresh
// or despawned body is unlocked with no init, and the despawn cascade's
// zero-fill is the unlock.
struct LockRow {
    glm::vec3 anchor{0.0f};  // world position the held axes are pinned to, m
    uint32_t axes = 0;       // kLockAxis* bits; 0 is free
};
static_assert(sizeof(LockRow) == 16, "a LockRow is a 16-byte std430 row on the device (layouts.slang)");
static_assert(std::is_trivially_copyable_v<LockRow> && std::is_trivially_destructible_v<LockRow>);

// WHERE INTEGRATE REACHES THE ROWS: the lock's optional read is the SIXTH access of Integrate's list
// (sim/standard_modules.cpp's kIntegrateAccess, which static_asserts the name there), after the contact scratch at 3
// and the gravity field at 4. A set without the lock module binds an absent view there, which reads as empty.
inline constexpr std::size_t kIntegratePassLockAccess = 5;

// The attach_row record Simulation::lock_translation() and unlock() hand the
// array's init: the anchor is taken at the CALL (the init cannot see a
// BodyState), and `axes` is 0 for an unlock.
struct LockSpawn {
    float anchor[3];
    uint32_t axes;
};
static_assert(std::is_trivially_copyable_v<LockSpawn>);

}  // namespace spade::physics
