// ---------------------------------------------------------------------------
// scene/object_file.hpp -- the object file: one placeable object, and the
// materials it brings that a world does not ship (spade::scene).
//
// It is the unit Spade's default library holds for everything that is not a
// drone -- props, obstacles, gates, floor tiles, poles, walls and the four
// primitives, all SDF assets (the editor end-goal design, section 3) -- and
// what the editor (AddAsset) and Kat's builder place into a scene, through ONE
// function, add_object() (TD-9), never by copying fields.
//
// THE TEXT, top-level keys in this order:
//   object_version: 1
//   object:      the scene file's per-asset block (scene/scene_file.hpp), every
//                key in the scene's order with the scene's encodings, as a
//                mapping: ONE emitter and ONE parser for both files (TD-9).
//                object_version 1 reads it with the key set of scene version
//                3, the newest (an asset's keys are the same in versions 1 to
//                3).
//   materials:   OPTIONAL, written only when the list is not empty: the scene
//                file's material encoding, one flow mapping per item, in file
//                order. Floats are written as the scene writes them.
//
// Properties, the scene file's:
//   1. CANONICAL TEXT. object_to_yaml() is a pure function of the ObjectFile,
//      with a fixed key order, a fixed float spelling and fixed comments;
//      every float survives the trip to text and back bit for bit, -0
//      included.
//   2. ONE VALIDATOR. validate_object_file() is the only definition of a
//      valid object file. The reader calls it after parsing and the writer
//      before emitting. Unknown, missing and duplicate keys are refused at
//      every level of the text.
//   3. A VERSION GATE. An object_version this build does not read is
//      schema_mismatch, read before any other key, so a caller can tell
//      "upgrade" from "fix your file". Every diagnostic blames the file's own
//      schema, "object schema v1" -- never the scene schema whose key set the
//      object block borrows.
//
// MATERIALS BY NAME (SCN-009). The object names its materials, in
// collider_materials and visual.material, as a scene asset does. `materials`
// lists the ones it brings. A name the object uses and does not list is a
// WORLD material, resolved when the scene is composed (compose(), SCN-004), so
// nothing here requires every name the object uses to be listed. Every LISTED
// material must be named by the object, so a library file carries no dead
// entries.
//
// NO SIDECARS. An asset's visual names its mesh by mesh_ref, a name the
// renderer resolves, and its collider is an SDF program held in the text, so
// an object file has no file beside it and its reader takes no base
// directory. A mesh obstacle (a collider from a .spmesh) would be a later
// object_version.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "scene/scene_file.hpp"  // SceneAsset, SceneDesc
#include "world/builder.hpp"     // MaterialDesc, SdfPose

namespace spade::scene {

// The object-file schema this build reads and writes (the only one so far).
inline constexpr uint32_t kObjectFileVersion = 1;

struct ObjectFile {
    // Exactly the scene's per-asset block. Its pose is the object's as the
    // file holds it; add_object() places it at the pose it is given.
    SceneAsset object{};
    // The materials it names that a world doesn't ship, in file order.
    std::vector<MaterialDesc> materials{};
};

// The one validator (property 2). invalid_argument naming the first
// violation, in this order:
//   - the object: the scene's per-asset rules, with the scene's messages (the
//     same function validate_scene() runs), except that an SDF the text cannot
//     carry is blamed on "object schema v1" and a missing name reads "object
//     has no name"; capacity_exceeded passes through from a collider deeper
//     than kMaxSdfDepth;
//   - each material: the world palette's rules (non-empty name, finite colour,
//     known shading; validate_material), as "object file material <i>";
//   - material names unique within the file;
//   - every listed material named by the object, in collider_materials or
//     visual.material: an unused one is refused by name.
// Not every name the object uses need be listed (MATERIALS BY NAME, above).
[[nodiscard]] Result<void> validate_object_file(const ObjectFile& file);

// ObjectFile -> canonical text. Runs validate_object_file() first, so the
// writer never emits a document the reader would refuse.
[[nodiscard]] Result<std::string> object_to_yaml(const ObjectFile& file);

// Text -> ObjectFile. Parses, then runs validate_object_file(). Codes:
// schema_mismatch for an object_version this build does not read;
// invalid_argument for everything else, with the offending key's dotted path
// ("object.collider.sdf.nodes[1]", "materials[0].shading") and its line and
// column.
[[nodiscard]] Result<ObjectFile> object_from_yaml(std::string_view text);

// Reads `path` and runs object_from_yaml(). io_error when it cannot be read;
// a parse diagnostic comes back prefixed with the path.
[[nodiscard]] Result<ObjectFile> load_object_file(const std::filesystem::path& path);

// PLACING (the lead's ruling, TD-9: the editor and Kat's builder both call
// this). Appends the object as asset `name` at `pose` -- the given pose
// replaces the file's own -- and merges its materials into scene.materials,
// in the file's order:
//   - a scene material of the same name and an IDENTICAL definition (the
//     same base_color bit for bit, and the same shading) is reused, not added
//     again;
//   - a different one is refused by name, naming EVERY field that differs
//     with both values, the object file's first ("here"), floats in their
//     shortest round-trip spelling, so Interface can show it verbatim:
//       "material 'red': base_color differs (0.8 0.1 0.1 1 here, 0.9 0.1 0.1 1
//       in the scene), shading differs (unlit here, lambert in the scene)"
//     and every conflicting material, each its own clause, joined by "; ";
//   - one the scene does not have is appended.
// A `name` an asset or a vehicle of the scene already holds is refused, naming
// it. It NEVER renames: the editor makes names unique.
// ALL OR NOTHING: the object is validated (validate_object_file()), the change
// is made on a copy, and the copy must pass validate_scene(); `scene` is
// assigned only then, so on any refusal it is unchanged. invalid_argument,
// or capacity_exceeded passed through from either validator. A world material
// the object names and does not list is not checked here: compose() resolves
// it against the world's palette (SCN-004).
[[nodiscard]] Result<void> add_object(SceneDesc& scene, const ObjectFile& object, std::string_view name,
                                      const SdfPose& pose);

}  // namespace spade::scene
