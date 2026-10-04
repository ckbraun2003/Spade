// ---------------------------------------------------------------------------
// scene/compose.hpp -- composing a scene into a runnable world (spade::scene).
//
// The drone-builder joint spec, section 3: a scene (a world reference plus
// materials, models, assets and vehicles; scene/scene_file.hpp) composes into
// one runnable world. Core owns this module's meaning, because it is the world
// model; Interface implements the composer.
// Plan: docs/design/interface/plans/2026-10-03-scene-composer-plan.md.
//
//   scene file --load--> SceneDesc --+
//   world file --load--> WorldDesc --+--compose--> ComposedScene --instantiate--> SceneRun
//
// compose() is a pure function of its two inputs, and instantiate() registers
// and spawns in a fixed order. Both are deterministic (DBE-010): every list is
// read in file order and appended in that order, and nothing is sorted.
//
// A vehicle's start stays in its design frame here: instantiate() hands it to
// Simulation::spawn() unchanged, and spawn applies the model's design-to-body
// rotation and centre-of-mass offset (DBP-45, DBP-46; Core owns spawn).
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "compute/backend.hpp"      // BackendDesc
#include "core/error.hpp"
#include "scene/scene_file.hpp"     // SceneDesc
#include "sim/simulation.hpp"       // Simulation, VehicleRef, VehicleSpawn
#include "sim/world_set.hpp"        // WorldInstanceDesc
#include "vehicles/model_type.hpp"  // ModelType
#include "world/builder.hpp"        // WorldDesc
#include "world/sdf.hpp"            // SdfTransform

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
// "Exactly the identity" compares values, so a -0 entry counts as 0, as it
// does in a transform built analytically from a zero position.
[[nodiscard]] SdfTransform compose_transform(const SdfTransform& collider, const SdfTransform& asset_pose);

// A vehicle of a composed scene.
struct VehiclePlacement {
    std::string name{};    // the scene's name for it; errors use it, the run never sees it
    uint32_t model = 0;    // index into ComposedScene::models
    VehicleSpawn start{};  // in the design frame, as the scene wrote it: spawn() converts it (DBP-45, DBP-46)
};

// A scene composed into one runnable world.
struct ComposedScene {
    // The world with the scene's materials appended to its palette, each
    // asset's collider joined to its SDF program and its visual added as a
    // prop, and its capacities sized (SCN-007).
    WorldDesc world{};
    // The scene's models, in the models: section's order. instantiate()
    // registers them in this order, so models[i] gets ModelTypeId i + 1.
    std::vector<vehicles::ModelType> models{};
    // The scene's vehicles, in scene order.
    std::vector<VehiclePlacement> vehicles{};
};

// Composes `scene` over `world`. Pure, and all or nothing: on failure nothing
// is returned in part. In this order (the plan's "Composition"):
//   1. validate_scene(scene), because a SceneDesc can be built in memory
//      (Kat's compiler, the editor) as well as read;
//   2. world_hash(world) must equal scene.world.hash (SCN-001), checked here
//      so that no entry point skips it;
//   3. the scene's materials are appended after the world's palette; a name
//      the world's palette already has is refused (SCN-004);
//   4. assets, in scene order (SCN-005): each collider transform is posed by
//      compose_transform(t, transform_of(asset.pose)), the collider's nodes
//      are appended with their primitives' transform indices offset, and one
//      union_ joins them to the program before them (none for the first
//      geometry of an empty program). Node materials resolve by name against
//      the combined palette. An empty node_materials stays empty while no
//      asset names a material, so a plain scene keeps the world's bytes. Once
//      the array is full length (the world's own, or filled at the first named
//      node with 0 for every node before it), each appended node adds its
//      resolved index, or 0 where nothing names one, and each union_ adds 0.
//      A visual becomes a PropDesc at the asset's pose;
//   5. models are copied in order, and each vehicle's model name becomes an
//      index into them;
//   6. capacities (SCN-007): each field is the world's count, plus what the
//      scene's vehicles need, plus the scene's spare count. A vehicle needs
//      one body, one force element per rotor and drag body, and one sensor
//      per IMU mount; contacts take the world's count plus spare;
//   7. validate_world_desc() on the composed world, the one validation every
//      WorldDesc passes.
// A material name is resolved to exactly one palette entry: a name the
// world's palette holds twice is refused when an asset uses it.
//
// Codes: invalid_argument (the scene does not validate, a world-hash
// mismatch, a material name that is reused, unknown or ambiguous, or a
// composed world that does not validate); capacity_exceeded (a capacity
// beyond 2^32 - 1, an SDF program that would hold more than 2^32 - 1
// transforms, or one deeper than kMaxSdfDepth once composed); and whatever
// world_hash() and transform_of() return.
[[nodiscard]] Result<ComposedScene> compose(const SceneDesc& scene, const WorldDesc& world);

// Loads the scene file, loads the world it names (world.file, relative to the
// scene file's directory), and composes them. Every error names the scene
// file; a world that does not load also names the world file.
[[nodiscard]] Result<ComposedScene> compose_file(const std::filesystem::path& scene_file);

// A composed scene, running.
struct SceneRun {
    Simulation sim;
    std::vector<VehicleRef> vehicles{};  // scene order
};

// One world instance of a composed scene. `instance` supplies the run's seed
// and the world's physics records (turbulence, contacts, grid); its `world`
// is replaced by the composed world. Creates a FRESH Simulation, registers the
// models in order, spawns the vehicles in order into world 0, and flushes
// once, so the returned vehicles are live. A start reaches spawn() exactly as
// the scene wrote it. Errors name the model or vehicle that failed, with the
// Simulation's code.
[[nodiscard]] Result<SceneRun> instantiate(const ComposedScene& scene, WorldInstanceDesc instance,
                                           uint64_t dt_ns, uint32_t substeps,
                                           const compute::BackendDesc& backend = {});

}  // namespace spade::scene
