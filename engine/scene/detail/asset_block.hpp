#pragma once

// ---------------------------------------------------------------------------
// scene/detail/asset_block.hpp -- one static asset's text block, shared by the
// scene file (each item of a scene's `assets:`) and the object file (its
// `object:` mapping; scene/object_file.hpp): ONE encoding of an asset,
// emitted, parsed and validated in one place (TD-9), as model_block.hpp is for
// a model. Beside it, the rules a material list passes, which both files'
// `materials:` share.
//
// Defined in scene_file.cpp, beside the scene's other blocks, whose helpers
// (the SDF lists, the unit-quaternion test) it shares.
//
// INTERNAL, NOT INSTALLED. It names yaml-cpp types, which no installed header
// may expose (world/detail/yaml_text.hpp's note); the install rule for scene/
// excludes detail/. Include it only from a .cpp.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "scene/scene_file.hpp"  // SceneAsset
#include "world/builder.hpp"     // MaterialDesc

namespace spade::scene::detail {

// The scene version that last changed an asset's key set. parse_asset() reads
// that set, and every scene version from it to kSceneFileVersion has it. A
// version that changes the set gates its keys by version, as
// ModelBlockFormat::key_version gates a model's, and raises this; a file that
// borrows the block asserts its own mapping against it (object_file.cpp).
inline constexpr uint32_t kAssetKeySetVersion = 1;

// One asset's keys, in SceneAsset's text order. The first line (`name`)
// starts with `lead` and every other key with `indent`, both the same width,
// so a scene writes a list item ("  - ", "    ") and the object file a mapping
// ("  ", "  "). The collider's `sdf` sits two spaces deeper and its lists four.
// The caller has validated the asset (validate_asset).
void emit_asset(std::string& out, const SceneAsset& a, std::string_view lead, std::string_view indent);

// One asset block. Only the text's shape is checked here: unknown, missing and
// duplicate keys at every level, blamed on `schema`, with the dotted path from
// `what` and the line. The asset's rules are validate_asset()'s.
[[nodiscard]] Result<SceneAsset> parse_asset(const YAML::Node& node, const std::string& what,
                                             std::string_view schema);

// The one set of per-asset rules (scene_file.hpp's validate_scene() lists them
// under "assets"): a name, refused as "<unnamed> has no name"; a finite pose
// with scale > 0 and a unit orientation; a collider that SdfProgram::validate()
// accepts and the SDF text encoding can carry (check_sdf_representable, blamed
// on `schema`), with no indices in node_materials, collider_materials empty or
// one non-empty name per node, and no transforms without nodes; a visual with
// both names or neither, its mesh_ref one world/mesh_ref.hpp's rule accepts
// (N3: "visual mesh_ref '<ref>' is not a valid mesh_ref: <problem>"); and at
// least one of the two halves. invalid_argument
// naming the asset ("asset '<name>': ..."); capacity_exceeded passes through
// from a collider deeper than kMaxSdfDepth. Whether the name is unique is the
// scene's to decide, not the asset's.
[[nodiscard]] Result<void> validate_asset(const SceneAsset& a, std::string_view unnamed, std::string_view schema);

// The rules a material list passes (a scene's `materials:`, an object file's):
// each material passes validate_material() (world/builder.hpp, the palette's
// one rule set: a name, a finite colour, a known shading), labelled
// "<label> <index>", and the names are unique within the list. invalid_argument
// naming the first violation.
[[nodiscard]] Result<void> validate_materials(std::span<const MaterialDesc> materials, std::string_view label);

}  // namespace spade::scene::detail
