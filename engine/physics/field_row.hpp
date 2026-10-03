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

#include <cstdint>

namespace spade::physics {

inline constexpr uint32_t kFieldGravityOffset = 0;  // vec3, m/s^2: floats 0..2
inline constexpr uint32_t kFieldDensityOffset = 3;  // scalar, kg/m^3: float 3
inline constexpr uint32_t kFieldWindOffset = 4;     // vec3, m/s: floats 4..6 (float 7 is padding)
inline constexpr uint32_t kFieldBuiltinFloats = 8;

}  // namespace spade::physics
