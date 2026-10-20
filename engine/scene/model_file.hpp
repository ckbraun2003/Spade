// ---------------------------------------------------------------------------
// scene/model_file.hpp -- the model file: one vehicle model and, optionally,
// its propulsion chain (spade::scene).
//
// It is the unit the editor adds a drone from, Spade's default model library
// holds, an analysis case's ModelRef.file names (analysis/case.hpp), and
// Kat's builder writes. A live run maps a throttle through its chain
// (vehicles::rotor_omega_for_duty) and pins the case by model_identity,
// parts_identity and propulsion_identity (vehicles/model_identity.hpp).
//
// THE TEXT, top-level keys in this order:
//   model_version: 1
//   model:        the scene file's per-model block (scene/scene_file.hpp),
//                 every key in the scene's order with the scene's encodings,
//                 as a mapping: ONE emitter and ONE parser for both files
//                 (TD-9). model_version 1 reads it with the key set of scene
//                 version 3, which carries everything (parts, mesh parts).
//   propulsion:   OPTIONAL, written only when set: every field of
//                 vehicles::PropulsionChain by its own name, in declaration
//                 order; the propeller's {j_min, j_max, ct, cq} as the
//                 mapping `table`. Floats are written as the scene writes
//                 them, doubles as yaml_text's format_double (the fewest
//                 digits that read back to the same double), counts as
//                 decimal integers.
//
// Properties, the scene file's:
//   1. CANONICAL TEXT. model_to_yaml() is a pure function of the ModelFile,
//      with a fixed key order, a fixed number spelling and fixed comments;
//      every float and double survives the trip to text and back bit for
//      bit, -0 included.
//   2. ONE VALIDATOR. validate_model_file() is the only definition of a
//      valid model file: the scene's per-model rules (the same function
//      validate_scene() runs) and, when a chain is set,
//      vehicles::validate_propulsion_chain(). The reader calls it after
//      parsing and the writer before emitting. Unknown, missing and duplicate
//      keys are refused at every level of the text.
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
// blamed on "model schema v1"; then the chain's, in
// validate_propulsion_chain()'s words.
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
// ("model.parts[0].shape", "propulsion.table.ct[3]") and its line and column.
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
// Refused by name: a model_name the scene's models already use (it never
// renames; the editor makes names unique), and anything validate_model_file()
// or validate_scene() refuses.
[[nodiscard]] Result<void> add_model(SceneDesc& scene, const ModelFile& file, std::string_view model_name);

}  // namespace spade::scene
