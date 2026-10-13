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
// parts are NEVER folded here (CORE-7, as agreed with Interface and the lead
// on 2026-10-12): this is the base identity, and it keeps this meaning, so a
// model that gains parts keeps its identity and every stored v3 blob
// restores. parts_identity() below is the parts' own identity; stage 7 folds
// parts into the snapshot's registry identity, not into this one.
// design_to_principal is folded as stored. register_model() stores
// canonical_design_rotation()'s value, so hash a model as it would be
// registered. A field added to ModelType that reaches state or the design
// frame must join this list, in the order it is declared (parts excepted).
//
// model_registry_identity(models) folds the model count (u32), then each
// model_identity (u64), in REGISTRATION order.
//
// parts_identity(m) is the identity of m.parts alone: the one rule for a
// part's identity (TD-9), so nothing else spells one. A consumer that must
// notice a geometry change (Interface's CFD case) pins the pair
// (model_identity, parts_identity). With no parts it is kNoPartsIdentity.
// Otherwise it folds, in this order:
//   the DOMAIN TAG: the byte 0xFF, then the 14 ASCII bytes "parts_identity"
//     (no length, no terminator). model_identity's stream opens with the
//     model's name, and 0xFF never begins UTF-8 text, so the two streams
//     differ from their first byte;
//   the part count (u32);
//   then each part, in DECLARATION order (the order is part of the identity):
//     name: its length (u32), then its bytes; role: the same;
//     attach (u32), rotor (u32);
//     mount_pos x y z; mount_orient w x y z, as stored (not normalised);
//     the geometry, as part_sample() reads it:
//       a mesh part (mesh non-null): the tag 1 (u32), then the mesh's
//         content_hash() (u64). The mesh's declared unit is not folded: a
//         TriangleMesh keeps only its fp32 metre positions, so the same
//         solid read from millimetres or from metres is the same geometry;
//       otherwise an SdfProgram part: the tag 0 (u32); the node count
//         (u32), then each node FIELD BY FIELD: kind, op, transform (u32
//         each), params x y z w; the transform count (u32), then each
//         transform's world_to_local, column by column (m[0][0], m[0][1],
//         ..., m[3][3]), then its scale. The _pad lanes are never read, and
//         node_materials (render data, empty on a valid part) is not folded;
//     surface.kind (u32), surface.inv_permeability, surface.inertial_coeff;
//     min_feature.
// Every float by its bit pattern, so -0 and +0 differ, as in model_identity.
// It does not validate: hash a model that passed validate() (exactly one of
// shape and mesh).
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>

#include "vehicles/model_type.hpp"

namespace spade::vehicles {

[[nodiscard]] uint64_t model_identity(const ModelType& model) noexcept;

[[nodiscard]] uint64_t model_registry_identity(std::span<const ModelType> models) noexcept;

// parts_identity() of a model with no parts: the fold above over the domain
// tag and a zero part count, fixed here so no consumer re-derives it. Not 0,
// so a field never computed cannot pass for "no parts".
inline constexpr uint64_t kNoPartsIdentity = 0x1bffc7475bebbf17ull;

[[nodiscard]] uint64_t parts_identity(const ModelType& model) noexcept;

}  // namespace spade::vehicles
