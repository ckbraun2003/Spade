// ---------------------------------------------------------------------------
// scene/model_file.hpp -- the model file: one vehicle model, its propulsion
// chain included when it has one (spade::scene).
//
// It is the unit the editor adds a drone from (add_model(), below), Spade's
// default model library holds, an analysis case's ModelRef.file names
// (analysis/case.hpp), and Kat's builder writes. A live run maps a throttle
// through its chain (vehicles::rotor_omega_for_duty) and pins the case by
// model_identity, parts_identity and propulsion_identity
// (vehicles/model_identity.hpp).
//
// THE CHAIN IS THE MODEL'S (N5a, the lead's ruling of 2026-10-19):
// vehicles::ModelType::propulsion. So the file holds one model and nothing
// beside it, and every copy, rename, duplicate or removal of the model -- in
// a scene, in the editor -- carries its chain by construction.
//
// THE TEXT, top-level keys in this order:
//   model_version: 1
//   model:        the scene file's per-model block (scene/scene_file.hpp),
//                 every key in the scene's order with the scene's encodings,
//                 as a mapping: ONE emitter and ONE parser for both files
//                 (TD-9). model_version 1 reads it with the key set of scene
//                 version 4, which carries everything (parts, mesh parts and
//                 the chain). Its LAST key, `propulsion`, OPTIONAL and
//                 written only when the model has a chain, is every field of
//                 vehicles::PropulsionChain by its own name, in declaration
//                 order; the propeller's {j_min, j_max, ct, cq} as the
//                 mapping `table`. Floats are written as the scene writes
//                 them, doubles as yaml_text's format_double (the fewest
//                 digits that read back to the same double), counts as
//                 decimal integers.
// Until N5a the chain was a top-level `propulsion:` after `model:` (N1,
// 13390f3). No file in that layout was committed; one is refused, its
// `propulsion` an unknown key.
//
// Properties, the scene file's:
//   1. CANONICAL TEXT. model_to_yaml() is a pure function of the ModelFile,
//      with a fixed key order, a fixed number spelling and fixed comments;
//      every float and double survives the trip to text and back bit for
//      bit, -0 included. A model without a chain is written as it was before
//      N5a, byte for byte.
//   2. ONE VALIDATOR. validate_model_file() is the only definition of a
//      valid model file: the scene's per-model rules (the same function
//      validate_scene() runs), ModelType::validate() among them, which checks
//      a set chain by vehicles::validate_propulsion_chain(). The reader calls
//      it after parsing and the writer before emitting. Unknown, missing and
//      duplicate keys are refused at every level of the text.
//   3. A VERSION GATE. A model_version this build does not read is
//      schema_mismatch, read before any other key, so a caller can tell
//      "upgrade" from "fix your file". Every diagnostic blames the file's own
//      schema, "model schema v1" -- never the scene schema whose key set the
//      model block borrows.
//
// MESH PARTS name their geometry, as in a scene: `shape: {mesh: {file,
// hash}}`, the .spmesh sidecar by a path relative to the MODEL file's
// directory and the mesh by its content hash. The reader loads and checks
// each sidecar (load_model_file() gives it the file's directory); the writer
// writes only the name and the hash, never a sidecar (save_mesh_file()
// writes one).
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include "core/error.hpp"
#include "scene/scene_file.hpp"     // SceneDesc
#include "vehicles/model_type.hpp"  // ModelType, its PropulsionChain
#include "world/mesh.hpp"           // TriangleMesh

namespace spade::scene {

// The model-file schema this build reads and writes (the only one so far).
inline constexpr uint32_t kModelFileVersion = 1;

// One model, its propulsion chain included (model.propulsion): the chain is
// part of the model (N5a, the lead's ruling), so every copy of the model
// carries it.
struct ModelFile {
    vehicles::ModelType model{};
};

// The one validator (property 2). invalid_argument naming the first
// violation: the model's, with the scene's messages, except that a mesh
// part's sidecar is relative to the "model file" and a part's shape is
// blamed on "model schema v1". A bad chain is the model's last rule:
// validate_propulsion_chain()'s words after the model's name ("model type
// 'quad': propulsion chain: kv must be finite and > 0").
[[nodiscard]] Result<void> validate_model_file(const ModelFile& file);

// ModelFile -> canonical text. Runs validate_model_file() first, so the
// writer never emits a document the reader would refuse. A mesh part is
// written as its mesh_file and its mesh's content hash; one with no
// mesh_file is refused, naming the model and the part. It never writes a
// sidecar.
[[nodiscard]] Result<std::string> model_to_yaml(const ModelFile& file);

// Text -> ModelFile. Parses, then runs validate_model_file(). Codes:
// schema_mismatch for a model_version this build does not read;
// invalid_argument for everything else, with the offending key's dotted path
// ("model.parts[0].shape", "model.propulsion.table.ct[3]") and its line and
// column.
// A mesh part's sidecar is read from base_dir / file, its refusals with
// their own codes (a missing sidecar is io_error); with an empty base_dir a
// mesh part is refused: read a model with mesh parts with load_model_file(),
// or give a base directory.
[[nodiscard]] Result<ModelFile> model_from_yaml(std::string_view text,
                                                const std::filesystem::path& base_dir = {});

// Reads `path` and runs model_from_yaml() with the file's directory as the
// base directory ("." for a bare file name). io_error when it cannot be
// read; a parse diagnostic comes back prefixed with the path.
[[nodiscard]] Result<ModelFile> load_model_file(const std::filesystem::path& path);

// Registers file.model in `scene` as `model_name` (the editor's per-vehicle
// copy, EDT-019), its chain with it: the chain is part of the model. ALL OR
// NOTHING: a copy, validate_scene() on it, assigned only on success.
// Refused by name: a model with mesh parts (below), a model_name the scene's
// models already use (it never renames; the editor makes names unique), and
// anything validate_model_file() or validate_scene() refuses.
//
// The model's own name is not its scene name: model_name replaces it, and
// model_identity() folds the name, so the copy's model_identity is the
// renamed model's (parts_identity and propulsion_identity fold no name, so
// they are the file's). Another namespace's name -- an asset's, a vehicle's,
// a material's -- is free for a model (SCN-004).
//
// A MODEL WITH MESH PARTS IS REFUSED (N3, the lead's ruling), the scene
// unchanged: "add_model: model '<name>' has mesh parts; their sidecars would
// need copying beside the scene, which is not yet supported", <name> the
// file's model's own name. A mesh part's mesh_file names its sidecar
// relative to the model file's directory, and a scene reads it relative to
// the scene file's, so placing one needs its sidecars copied beside the
// scene first: add_model_with_meshes(), below, places one.
[[nodiscard]] Result<void> add_model(SceneDesc& scene, const ModelFile& file, std::string_view model_name);

// The sidecar a placed mesh part names, relative to the SCENE file (L7b0 of
// the realistic drones plan): "imported/<h>.spmesh", h the mesh's
// content_hash() as 16 lowercase hex digits. imported/ is the folder the
// editor's Save carries beside the scene (N15), and the hash makes the name
// the mesh's own: two parts, or two placed drones, that share a mesh share it.
[[nodiscard]] std::string placed_mesh_file(const TriangleMesh& mesh);

// add_model() for a model that may have mesh parts (L7b0): the same checks
// and the same registration, all or nothing, except that each mesh part's
// mesh_file is re-pointed to placed_mesh_file(*mesh) in the placed copy (the
// file's own model is untouched). Its mesh stays as loaded. Writing the
// sidecars is the caller's: the editor writes each one that no folder holds
// yet into its staging folder before Save (sandbox/editor_workspace.hpp).
// mesh_file is part of no identity, so the placed model's identities are
// add_model()'s. A model with no mesh parts is placed exactly as add_model()
// places it.
[[nodiscard]] Result<void> add_model_with_meshes(SceneDesc& scene, const ModelFile& file,
                                                 std::string_view model_name);

}  // namespace spade::scene
