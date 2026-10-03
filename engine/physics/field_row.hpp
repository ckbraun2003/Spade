#pragma once

// physics/field_row.hpp -- the layout of a world's field sample row (module-API
// plan, stage 3). A provider pass writes its field's floats here once per world
// per substep, in the Fields phase; readers read them in later phases. The row
// is scratch: not registered state, recomputed every substep before any reader.
//
// The built-in fields sit at fixed places at the front of every row, whether
// or not the module set provides them, so the GPU row (layouts.slang) and a
// developer field's offset never depend on which built-ins are present. Every
// other field follows from kFieldBuiltinFloats, in the order the compiled
// schedule's registry gives (sim/module.hpp).
//
// Here rather than in sim/module.hpp because the passes that write and read
// the row live in physics/, which does not include sim/.

#include <cstddef>
#include <cstdint>

#include <glm/vec3.hpp>

#include "state/layout.hpp"  // kStd430StructAlignment

namespace spade::physics {

inline constexpr uint32_t kFieldGravityOffset = 0;  // vec3, m/s^2: floats 0..2
inline constexpr uint32_t kFieldDensityOffset = 3;  // scalar, kg/m^3: float 3
inline constexpr uint32_t kFieldWindOffset = 4;     // vec3, m/s: floats 4..6 (float 7 is padding)
inline constexpr uint32_t kFieldBuiltinFloats = 8;

// The built-in prefix as one std430 row: the device's field_samples buffer
// holds one per world (shaders/shared/layouts.slang's FieldSampleRow, checked
// against this by layout_check.gen.hpp). The CPU keeps its rows as floats at
// the same offsets, so the two agree float for float.
struct alignas(kStd430StructAlignment) FieldSampleRow {
    glm::vec3 gravity{0.0f};  // m/s^2
    float density = 0.0f;     // kg/m^3
    glm::vec3 wind{0.0f};     // m/s
    float _pad = 0.0f;
};
static_assert(sizeof(FieldSampleRow) == kFieldBuiltinFloats * sizeof(float));
static_assert(offsetof(FieldSampleRow, gravity) == kFieldGravityOffset * sizeof(float));
static_assert(offsetof(FieldSampleRow, density) == kFieldDensityOffset * sizeof(float));
static_assert(offsetof(FieldSampleRow, wind) == kFieldWindOffset * sizeof(float));

}  // namespace spade::physics
