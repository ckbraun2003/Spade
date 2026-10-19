// ---------------------------------------------------------------------------
// scene/scene_file.hpp -- the scene file: a world reference plus the objects
// placed in it (spade::scene).
//
// The schema is Core's: docs/design/interface/plans/2026-10-03-scene-file-draft.md
// (SCN-001..009), under the drone-builder joint spec (DBE-013, DBE-014). A
// world holds physics and environment; a scene holds a world reference plus
// materials, vehicle models, static assets and vehicles with their start
// states. scene/compose.hpp turns one into a runnable world.
//
// Properties, the world file's (world/world_file.hpp):
//   1. CANONICAL TEXT (SCN-002). scene_to_yaml() is a pure function of the
//      SceneDesc, every float survives the trip to text and back bit for bit,
//      and `spare` is always written, zeros included.
//   2. ONE VALIDATOR (SCN-003). validate_scene() is the only definition of a
//      valid scene. The reader calls it after parsing, the writer before
//      emitting, and a builder of a SceneDesc in memory (Kat's compiler, the
//      editor, compose()) calls it too. Unknown, missing and duplicate keys
//      are refused at every level of the text.
//   3. A VERSION GATE. A scene_version this build does not read is
//      schema_mismatch, so a caller can tell "upgrade" from "fix your file".
//      The version WRITTEN is a function of the content: the oldest version
//      that carries everything in the scene. Version 2 adds a model's parts
//      (CORE-7), so a scene whose models have none is written as version 1,
//      byte for byte the text it always was, and an older build reading a
//      scene with parts says "upgrade" rather than "unknown key". Version 3
//      adds a mesh part (CORE-6), so only a scene with one is version 3.
//
// MESH PARTS name their geometry; the text never holds it. A part whose
// geometry is a triangle mesh is written as `shape: {mesh: {file, hash}}`:
// the .spmesh sidecar (world/mesh_file.hpp) by a path relative to the scene
// file's directory, and the mesh by its content hash, as the scene names its
// world (SCN-001). The reader loads and checks each sidecar; the writer
// writes only the name and the hash, and save_mesh_file() writes the sidecar.
//
// What it checks is what the scene alone decides. What needs the world -- the
// world hash (SCN-001) and a scene material reusing a world material's name
// (SCN-004) -- compose() checks, on every path.
//
// Names, never indices (SCN-009): a visual's material and an asset collider's
// node materials are material names, resolved by compose() against the world's
// palette with the scene's materials appended.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "sim/simulation.hpp"       // VehicleSpawn
#include "vehicles/model_type.hpp"  // ModelType
#include "world/builder.hpp"        // MaterialDesc, SdfPose, WorldDesc
#include "world/sdf.hpp"            // SdfProgram

namespace spade::scene {

// The newest scene-file schema this build reads, and the oldest. It reads
// every version in between, and writes each scene as the oldest version that
// carries it (property 3):
//   1  materials, models, assets, vehicles and spare (SCN-001..009);
//   2  adds `parts` to a model (CORE-7), written only when a model has parts;
//   3  adds a part's `shape: {mesh: {file, hash}}` (CORE-6), written only
//      when a part holds a mesh.
inline constexpr uint32_t kSceneFileVersion = 3;
inline constexpr uint32_t kSceneFileOldestVersion = 1;

// The world a scene is placed in (SCN-001).
struct SceneWorldRef {
    // Relative to the scene file's directory, with forward slashes, kept as
    // written: compose_file() resolves it, nothing here does.
    std::string file{};
    // world_hash() of the world it names. compose() refuses another.
    uint64_t hash = 0;
};

// An asset's render half. Both empty means the asset has no visual; otherwise
// both are set. `material` is a material name.
struct SceneVisual {
    std::string mesh_ref{};
    std::string material{};
};

// Static scenery: a pose, a collider, a visual, at least one of the two halves.
struct SceneAsset {
    std::string name{};
    SdfPose pose{};
    // The world file's SDF program, in the asset's frame. No nodes means the
    // asset does not collide. Its node_materials (indices) stays EMPTY: a
    // scene names materials, in collider_materials below.
    SdfProgram collider{};
    // Material names parallel to collider.nodes, or empty for "every node uses
    // the palette's default", as SdfProgram::node_materials is.
    std::vector<std::string> collider_materials{};
    SceneVisual visual{};
};

// A vehicle: a model by name, and its start state in the model's DESIGN frame
// (DBE-013). Simulation::spawn() applies the model's design_to_principal and
// com_offset (DBP-45, DBP-46); the scene and the composer never do.
struct SceneVehicle {
    std::string name{};
    std::string model{};
    VehicleSpawn start{};
};

// Slots reserved for objects that appear at runtime (SCN-007), in the world
// file's capacity fields. Each count is 0 when the file omits it.
struct SceneSpare {
    uint32_t bodies = 0;
    uint32_t force_elements = 0;
    uint32_t sensors = 0;
    uint32_t contacts = 0;
};

// A parsed scene file. Every list is in file order, and that order is part of
// the configuration (SCN-005): models register in `models` order, assets and
// vehicles compose in theirs.
struct SceneDesc {
    std::string name{};
    SceneWorldRef world{};
    // Appended after the world's palette by compose().
    std::vector<MaterialDesc> materials{};
    // Full model types, version and design frame included (SCN-009). A model
    // no vehicle uses is allowed: it is registered and never spawned.
    std::vector<vehicles::ModelType> models{};
    std::vector<SceneAsset> assets{};
    std::vector<SceneVehicle> vehicles{};
    SceneSpare spare{};
};

// The one validator (SCN-003). invalid_argument naming the first violation:
//   - names: the scene's non-empty; unique within materials, within models,
//     and across assets and vehicles together (SCN-004);
//   - world.file: non-empty, relative, forward slashes only;
//   - materials: the world palette's rules (non-empty name, finite colour,
//     known shading);
//   - models: ModelType::validate(), its parts' rules included (CORE-7: the
//     model's own message, which names the part and the field), and
//     design_to_principal a unit quaternion (SCN-009); each part's shape
//     expressible in the SDF text encoding (check_sdf_representable), with
//     the model, the part and the node named; a part's mesh_file, when set,
//     relative with forward slashes only, and naming a mesh the part holds
//     (a mesh part may leave it empty in memory: only the writer needs it).
//     These per-model rules are the model file's too, one function for both
//     (scene/model_file.hpp, TD-9);
//   - assets: a finite pose with scale > 0 and a unit orientation; a collider
//     that SdfProgram::validate() accepts, with no indices in node_materials
//     and collider_materials empty or one non-empty name per node, and no
//     transforms without nodes; a visual with both names or neither; and at
//     least one of the two halves;
//   - vehicles: a model the scene declares, and a start with a finite state,
//     a unit orientation and rotor_omega >= 0.
// capacity_exceeded passes through from a collider deeper than kMaxSdfDepth.
[[nodiscard]] Result<void> validate_scene(const SceneDesc& scene);

// Text -> SceneDesc. Parses, then runs validate_scene(). Codes:
// schema_mismatch for a scene_version this build does not read;
// invalid_argument for everything else, with the offending key's dotted path
// and its line and column. A model's `parts` is read from version 2 (in a
// version-1 file it is an unknown key, blamed on version 1), and a shape's
// `mesh` from version 3.
//
// A mesh part's sidecar is read from base_dir / file (the file's UTF-8 path),
// by load_mesh_file() against the hash the text names: its refusals -- a
// missing file (io_error), another version (schema_mismatch), a changed mesh
// with both hashes, a corrupt or truncated file, every make_mesh() refusal --
// come back with their own code, after the part's dotted path, and with the
// line. With an empty base_dir a mesh part is refused: read a scene with
// mesh parts with load_scene_file(), or give a base directory.
[[nodiscard]] Result<SceneDesc> scene_from_yaml(std::string_view yaml_text,
                                              const std::filesystem::path& base_dir = {});

// SceneDesc -> canonical text (SCN-002). Runs validate_scene() first, so the
// writer never emits a document the reader would refuse. Writes version 3
// only when a part holds a mesh, version 2 only when a model has parts, and
// version 1 otherwise (property 3). A mesh part is written as its
// mesh_file and its mesh's content hash; one with no mesh_file is refused,
// naming the model and the part. It never writes a sidecar.
[[nodiscard]] Result<std::string> scene_to_yaml(const SceneDesc& scene);

// Reads `path` and runs scene_from_yaml() with the file's directory as the
// base directory ("." for a bare file name). io_error when it cannot be
// read; a parse diagnostic comes back prefixed with the path.
[[nodiscard]] Result<SceneDesc> load_scene_file(const std::filesystem::path& path);

// The world hash a scene pins its world by (SCN-001): FNV-1a 64 over
// world_to_yaml(world), the world file's canonical text of the loaded world --
// not the file's bytes, so a v1 world that upgrades on load, or a hand edit
// that changes only comments or spacing, keeps it. It covers the world's own
// content only, never the meshes or textures it names (L5). Fails as
// world_to_yaml() does, on a world validate_world_desc() refuses.
[[nodiscard]] Result<uint64_t> world_hash(const WorldDesc& world);

}  // namespace spade::scene
