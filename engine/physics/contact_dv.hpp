#pragma once

#include <cstddef>

#include <glm/glm.hpp>

namespace spade::physics {

// ---------------------------------------------------------------------------
// ONE ROW OF THE CONTACT SCRATCH (PHY-7): the world-axes velocity change the
// contact passes gave one body this substep. Integrate reads it into the
// specific force and zeroes it, for every slot, before any skip.
//
// A DECLARED SCRATCH (module-API stage 4, Task 7b): `integrate` owns it,
// per body, at binding 27 (sim/standard_modules.cpp); both contact passes
// accumulate into it. Not walked, digested or snapshotted. 16 bytes on both
// sides, so one row size sizes the CPU rows and the device buffer's stride:
// the GPU's float4 rows (shaders/shared/bindings.slang) carry `dv` in xyz and
// leave w unused, as `_pad` is here.
// ---------------------------------------------------------------------------
// WHERE THE PASSES REACH IT: its index in each pass's access list
// (sim/standard_modules.cpp's kContactAccess and kIntegrateAccess). The passes
// read ctx.state at these indices (physics/schedule.cpp), and
// standard_modules.cpp static_asserts that the lists name the scratch there,
// so reordering a list is a compile error rather than a wrong view (Core's
// suggestion, 2026-10-06).
inline constexpr std::size_t kContactPassContactDvAccess = 1;
inline constexpr std::size_t kIntegratePassContactDvAccess = 3;

struct ContactDvRow {
    glm::vec3 dv{0.0f};
    float _pad = 0.0f;
};
static_assert(sizeof(ContactDvRow) == 16, "the GPU's float4 rows");

}  // namespace spade::physics
