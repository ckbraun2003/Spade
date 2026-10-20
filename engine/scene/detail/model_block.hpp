#pragma once

// ---------------------------------------------------------------------------
// scene/detail/model_block.hpp -- one vehicle model's text block, shared by
// the scene file (each item of a scene's `models:`) and the model file (its
// `model:` mapping; scene/model_file.hpp): ONE encoding of a model, emitted,
// parsed and validated in one place (TD-9). Beside it, a propulsion chain's
// block, the model block's own last key `propulsion:` (N5a: the chain is the
// model's), in both files.
//
// Defined in scene_file.cpp, beside the scene's other blocks, whose helpers
// (the SDF lists, the hashes, the enum names) it shares.
//
// INTERNAL, NOT INSTALLED. It names yaml-cpp types, which no installed header
// may expose (world/detail/yaml_text.hpp's note); the install rule for scene/
// excludes detail/. Include it only from a .cpp.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace spade::scene::detail {

// The file a model block is read from, for its key set and its diagnostics.
struct ModelBlockFormat {
    // The SCENE version whose model key set the block is read with: `parts`
    // from 2, a part's shape `mesh` from 3, `propulsion` from 4. A scene
    // passes its own version; the model file maps its model_version to one
    // (model_file.cpp).
    uint32_t key_version = 0;
    // The schema every diagnostic blames: the file's OWN, "scene schema v2" or
    // "model schema v1" (scene_file.cpp's schema() note).
    std::string_view schema{};
    // "scene" or "model": the file a sidecar's path is relative to, and the
    // loader that gives a reader its base directory (load_<kind>_file).
    std::string_view kind{};
};

// What is wrong with `file` as a path relative to the <kind> file's directory
// with forward slashes (a world's file, a mesh part's sidecar), or empty.
// `file` is not empty.
[[nodiscard]] std::string relative_path_problem(std::string_view file, std::string_view kind);

// One model's keys, in ModelType's text order. The first line (`name`) starts
// with `lead` and every other key with `indent`, both the same width, so a
// scene writes a list item ("  - ", "    ") and the model file a mapping
// ("  ", "  "). Nested lists sit two spaces deeper, and so do the keys of the
// chain (emit_propulsion), the last key, written only when the model has one.
// The caller has validated the model (validate_model, check_model_writable).
void emit_model(std::string& out, const vehicles::ModelType& m, std::string_view lead, std::string_view indent);

// One model block. Only the text's shape is checked here: unknown, missing and
// duplicate keys at every level, by `format`'s key set and blamed on its
// schema, with the dotted path from `what` and the line. A mesh part's sidecar
// is read from base_dir / file (scene_file.hpp's MESH PARTS); an empty
// base_dir refuses a mesh part. The model's rules are validate_model()'s.
[[nodiscard]] Result<vehicles::ModelType> parse_model(const YAML::Node& node, const std::string& what,
                                                      const ModelBlockFormat& format,
                                                      const std::filesystem::path& base_dir);

// The one set of per-model rules a file adds to ModelType::validate()
// (scene_file.hpp's validate_scene() lists them under "models"):
// ModelType::validate() itself; design_to_principal a unit quaternion; each
// part's shape expressible in the SDF text encoding, blamed on `schema`; a
// part's mesh_file, when set, relative to the <kind> file with forward slashes
// and naming a mesh the part holds. invalid_argument naming the model, and the
// part where there is one.
[[nodiscard]] Result<void> validate_model(const vehicles::ModelType& m, std::string_view schema,
                                          std::string_view kind);

// The writer's one extra rule: the text names a mesh part's sidecar and never
// holds the mesh, so a mesh part needs its mesh_file (CORE-6). invalid_argument
// naming the model and the part.
[[nodiscard]] Result<void> check_model_writable(const vehicles::ModelType& m, std::string_view kind);

// A propulsion chain's block (a model block's `propulsion:`, in a scene from
// version 4 and in the model file; emit_model writes it): EVERY field
// of vehicles::PropulsionChain by its own name, in declaration order, one
// key per line at `indent`; the propeller table's j_min, j_max, ct and cq
// under `table:`, two spaces deeper. Doubles are format_double's, floats
// format_float's and float_list's, counts decimal. The caller has validated
// the chain (vehicles::validate_propulsion_chain).
void emit_propulsion(std::string& out, const vehicles::PropulsionChain& chain, std::string_view indent);

// One chain block. Only the text's shape is checked here -- every key
// required, and unknown, missing and duplicate keys refused, blamed on
// `schema`, with the dotted path from `what` and the line. The chain's rules
// are validate_propulsion_chain()'s.
[[nodiscard]] Result<vehicles::PropulsionChain> parse_propulsion(const YAML::Node& node, const std::string& what,
                                                                 std::string_view schema);

}  // namespace spade::scene::detail
