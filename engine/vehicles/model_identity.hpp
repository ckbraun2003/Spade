#pragma once

// ---------------------------------------------------------------------------
// vehicles/model_identity.hpp -- a model type's identity, and the model
// registry's (snapshot format v3; Core's drone-builder plan, Task C).
//
// A Simulation's snapshot carries the registry's identity beside the module
// set's (sim/module.hpp), and restore() refuses a blob taken under another
// registry (L2). A model's own identity is what the airframe compiler and Kat
// hash a compiled airframe by. Both live here, in vehicles/, so neither caller
// needs sim/.
//
// SPELT BYTE BY BYTE, so they are the same on every platform. FNV-1a 64 (the
// core/rng.hpp constants), every float as its IEEE-754 bit pattern and every
// integer little-endian.
//
// model_identity(m) folds, in this order:
//   name, then a 0 byte; version, param_schema_id (u32);
//   body.mass, body.inertia_diag x y z, proxy_radius;
//   design_to_principal w x y z, com_offset x y z;
//   the rotor count (u32), then each rotor's local_pos x y z, local_orient
//     w x y z, spin_dir, tau, radius, thrust_coeff, torque_coeff;
//   the drag-body count, then each one's mode (u32), area, coeffs x y z,
//     local_pos x y z, local_orient w x y z;
//   the IMU-mount count, then each one's mount_pos x y z, mount_orient
//     w x y z, rate_divider (u32), sigma_a, sigma_g, sigma_ba, sigma_bg.
// visual_ref is NOT folded: it is render data and never reaches the step (L5).
// design_to_principal is folded as stored. register_model() stores
// canonical_design_rotation()'s value, so hash a model as it would be
// registered. A field added to ModelType that reaches state or the design
// frame must join this list, in the order it is declared.
//
// model_registry_identity(models) folds the model count (u32), then each
// model_identity (u64), in REGISTRATION order.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>

#include "vehicles/model_type.hpp"

namespace spade::vehicles {

[[nodiscard]] uint64_t model_identity(const ModelType& model) noexcept;

[[nodiscard]] uint64_t model_registry_identity(std::span<const ModelType> models) noexcept;

}  // namespace spade::vehicles
