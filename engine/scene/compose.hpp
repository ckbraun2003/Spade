// ---------------------------------------------------------------------------
// scene/compose.hpp -- composing a scene into a runnable world (spade::scene).
//
// The drone-builder joint spec, section 3: a scene (a world reference plus
// assets and vehicles) composes into one runnable world. Core owns this
// module's meaning, because it is the world model; Interface implements it.
// Plan: docs/design/interface/plans/2026-10-03-scene-composer-plan.md.
//
// This first cut holds the composer's two arithmetic helpers, which need none
// of the scene-file types. compose(), compose_file() and instantiate() follow
// when Core lands transform_of() and the scene-file schema (SceneDesc).
// ---------------------------------------------------------------------------
#pragma once

#include <glm/gtc/quaternion.hpp>

#include "sim/simulation.hpp"  // VehicleSpawn
#include "world/sdf.hpp"       // SdfTransform

namespace spade::scene {

// Poses an asset's collider transform by the asset's own transform, as
// transform_of(asset.pose) gives it. A world point goes into the asset's frame
// and then into the collider's, so world_to_local is collider x asset, and
// the distance scales multiply.
//
// Two exact short-cuts, because a 4x4 product is not bitwise
// identity-preserving ((-0)*1 + (+0) is +0, and config_hash folds bytes):
//   (a) an asset transform that is exactly the identity returns the collider
//       unchanged;
//   (b) a collider transform that is exactly the identity returns the asset's
//       transform verbatim, so a one-primitive asset gets exactly the bytes
//       WorldBuilder gives that primitive at the asset's pose.
// "Exactly the identity" compares values, so a -0 entry counts as 0: the
// builder stores the identity pose's translation as -0.
[[nodiscard]] SdfTransform compose_transform(const SdfTransform& collider, const SdfTransform& asset_pose);

// Turns a vehicle's start from its design frame into its principal-axis body
// frame (DBE-013), using the model's design_to_principal rotation q:
//   orientation  orient_design x conj(q)
//   body rates   rotate(q, omega_body_design)
// Velocity is world-frame and unchanged, and so are the position and
// rotor_omega. An exactly-identity q, (1, 0, 0, 0) or (-1, 0, 0, 0), returns
// the start untouched, so an airframe whose axes already agree spawns
// bit-identical to a direct spawn().
[[nodiscard]] VehicleSpawn design_to_principal(const VehicleSpawn& start_design, const glm::quat& q_d2p);

}  // namespace spade::scene
