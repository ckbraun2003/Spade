#pragma once

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
struct ContactDvRow {
    glm::vec3 dv{0.0f};
    float _pad = 0.0f;
};
static_assert(sizeof(ContactDvRow) == 16, "the GPU's float4 rows");

}  // namespace spade::physics
