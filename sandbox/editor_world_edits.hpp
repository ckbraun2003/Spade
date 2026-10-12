// The editor's world edits and the new-world template. Display-free (SL15b).
//
// Each edit changes a copy of the world and commits it through the document,
// so validate_world_desc() judges it (EDT-003, EDT-015) and it can be undone.
// Spawn points are addressed by name, as the world file looks them up.
// Materials and props are addressed by index, because that is their identity
// in the world file: node_materials and PropDesc::material refer to a
// material by index, a material's name need not be unique, and a prop has no
// name. So a material is only appended or changed, never removed, which keeps
// every reference to it stable.
//
// Terrain (the SDF program) is edited through editor_sdf_edit.hpp (Task 15),
// which Core reviews; an exact no-op there commits nothing here.
//
// docs/design/interface/plans/2026-10-05-editor-design.md §6;
// 2026-10-05-editor-plan.md Task 4.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <glm/glm.hpp>

#include "editor_document.hpp"
#include "editor_scene_edits.hpp"
#include "editor_sdf_edit.hpp"

namespace spade::sandbox::editor {

struct RenameWorld {
    std::string name;
};
struct SetEnvironment {
    Environment environment;
};
struct SetCapacities {
    Capacities capacities;
};
struct AddWorldMaterial {
    MaterialDesc material;  // appended: its index is the palette's old size
};
struct SetWorldMaterial {
    uint32_t index = 0;
    MaterialDesc material;
};
struct SetLighting {
    LightingDesc lighting;
};
struct AddSpawn {
    SpawnPoint spawn;  // its name is made unique among the spawns
};
struct SetSpawn {
    std::string name;  // the spawn to change
    SpawnPoint spawn;  // its new value, which may rename it
};
struct RemoveSpawn {
    std::string name;
};
struct AddProp {
    PropDesc prop;
};
struct SetProp {
    std::size_t index = 0;
    PropDesc prop;
};
struct RemoveProp {
    std::size_t index = 0;
};
// Terrain, through editor_sdf_edit.hpp (Task 15). Pieces and nodes are
// addressed by their place in the program, which is their only identity.
struct AddTerrainPrimitive {
    SdfPrim kind = SdfPrim::box;
    glm::vec4 params{0.0f};  // sdf.hpp's packing for `kind`
    SdfPose pose;
    uint32_t material = 0;
};
struct RemoveTerrainPiece {
    std::size_t piece = 0;
};
struct ReposeTerrainPiece {
    std::size_t piece = 0;
    SdfPose pose;
};
struct SetTerrainParams {
    std::size_t node = 0;
    glm::vec4 params{0.0f};
};

using WorldEdit = std::variant<RenameWorld, SetEnvironment, SetCapacities, AddWorldMaterial, SetWorldMaterial,
                               SetLighting, AddSpawn, SetSpawn, RemoveSpawn, AddProp, SetProp, RemoveProp,
                               AddTerrainPrimitive, RemoveTerrainPiece, ReposeTerrainPiece, SetTerrainParams>;

namespace detail {

[[nodiscard]] inline bool spawn_taken(const WorldDesc& w, std::string_view name) {
    for (const SpawnPoint& s : w.spawns) {
        if (s.name == name) return true;
    }
    return false;
}

[[nodiscard]] inline Error no_spawn(std::string_view what, std::string_view name) {
    return Error{Code::invalid_argument, std::string(what) + ": no spawn point is named '" + std::string(name) +
                                             "' in this world; check the name in the world panel"};
}

[[nodiscard]] inline Error past_end(std::string_view what, std::size_t index, std::size_t count) {
    return Error{Code::invalid_argument, std::string(what) + " " + std::to_string(index) + ": the world has " +
                                             std::to_string(count) + ", numbered from 0"};
}

inline Result<bool> change(WorldDesc& w, const RenameWorld& e) {
    if (e.name.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "rename the world: a world needs a name"});
    }
    if (e.name == w.name) return false;
    w.name = e.name;
    return true;
}

inline Result<bool> change(WorldDesc& w, const SetEnvironment& e) {
    w.environment = e.environment;
    return true;
}

inline Result<bool> change(WorldDesc& w, const SetCapacities& e) {
    w.capacities = e.capacities;
    return true;
}

inline Result<bool> change(WorldDesc& w, const AddWorldMaterial& e) {
    w.materials.push_back(e.material);
    return true;
}

inline Result<bool> change(WorldDesc& w, const SetWorldMaterial& e) {
    if (e.index >= w.materials.size()) return std::unexpected(past_end("material", e.index, w.materials.size()));
    w.materials[e.index] = e.material;
    return true;
}

inline Result<bool> change(WorldDesc& w, const SetLighting& e) {
    w.lighting = e.lighting;
    return true;
}

inline Result<bool> change(WorldDesc& w, const AddSpawn& e) {
    SpawnPoint s = e.spawn;
    s.name = lowest_free(s.name.empty() ? "spawn" : s.name, [&](std::string_view n) { return spawn_taken(w, n); });
    w.spawns.push_back(std::move(s));
    return true;
}

inline Result<bool> change(WorldDesc& w, const SetSpawn& e) {
    SpawnPoint* target = find_named(w.spawns, e.name);
    if (target == nullptr) return std::unexpected(no_spawn("set the spawn point", e.name));
    if (e.spawn.name != e.name && spawn_taken(w, e.spawn.name)) {
        return std::unexpected(Error{Code::invalid_argument, "rename spawn point '" + e.name + "' to '" +
                                                                 e.spawn.name + "': '" + e.spawn.name +
                                                                 "' is already a spawn point in this world; "
                                                                 "pick another name"});
    }
    *target = e.spawn;
    return true;
}

inline Result<bool> change(WorldDesc& w, const RemoveSpawn& e) {
    const auto it = std::find_if(w.spawns.begin(), w.spawns.end(), [&](const auto& s) { return s.name == e.name; });
    if (it == w.spawns.end()) return std::unexpected(no_spawn("remove the spawn point", e.name));
    w.spawns.erase(it);
    return true;
}

inline Result<bool> change(WorldDesc& w, const AddProp& e) {
    w.props.push_back(e.prop);
    return true;
}

inline Result<bool> change(WorldDesc& w, const SetProp& e) {
    if (e.index >= w.props.size()) return std::unexpected(past_end("prop", e.index, w.props.size()));
    w.props[e.index] = e.prop;
    return true;
}

// A terrain edit's result: committed unless the helper found an exact no-op
// and returned the program as it was.
[[nodiscard]] inline Result<bool> set_terrain(WorldDesc& w, Result<SdfProgram> next) {
    if (!next) return std::unexpected(next.error());
    const bool same = next->nodes.size() == w.sdf.nodes.size() &&
                      next->transforms.size() == w.sdf.transforms.size() &&
                      next->node_materials == w.sdf.node_materials &&
                      std::memcmp(next->nodes.data(), w.sdf.nodes.data(), next->nodes.size() * sizeof(SdfNode)) == 0 &&
                      std::memcmp(next->transforms.data(), w.sdf.transforms.data(),
                                  next->transforms.size() * sizeof(SdfTransform)) == 0;
    if (same) return false;
    w.sdf = std::move(*next);
    return true;
}

inline Result<bool> change(WorldDesc& w, const AddTerrainPrimitive& e) {
    return set_terrain(w, add_primitive(w.sdf, e.kind, e.params, e.pose, e.material));
}

inline Result<bool> change(WorldDesc& w, const RemoveTerrainPiece& e) {
    return set_terrain(w, remove_piece(w.sdf, e.piece));
}

inline Result<bool> change(WorldDesc& w, const ReposeTerrainPiece& e) {
    return set_terrain(w, repose_piece(w.sdf, e.piece, e.pose));
}

inline Result<bool> change(WorldDesc& w, const SetTerrainParams& e) {
    return set_terrain(w, set_node_params(w.sdf, e.node, e.params));
}

inline Result<bool> change(WorldDesc& w, const RemoveProp& e) {
    if (e.index >= w.props.size()) return std::unexpected(past_end("prop", e.index, w.props.size()));
    w.props.erase(w.props.begin() + static_cast<std::ptrdiff_t>(e.index));
    return true;
}

}  // namespace detail

// Applies `edit` to the document's world and commits it. A refusal, from the
// edit or from validate_world_desc(), leaves the document unchanged and says
// why; an edit that changes nothing commits nothing.
[[nodiscard]] inline Result<void> apply(WorldDocument& doc, const WorldEdit& edit) {
    WorldDesc next = doc.desc();
    const Result<bool> changed = std::visit([&](const auto& e) { return detail::change(next, e); }, edit);
    if (!changed) return std::unexpected(changed.error());
    if (!*changed) return {};
    return doc.commit(std::move(next));
}

// The "new world" template: a ground plane at y = 0, the default material,
// default environment and lighting, no spawns or props, and every capacity 1;
// compose() sizes a scene's own need on top of it (SCN-007).
[[nodiscard]] inline Result<WorldDesc> new_world(std::string name) {
    if (name.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "new world: a world needs a name"});
    }
    return WorldBuilder{}
        .name(std::move(name))
        .capacities(Capacities{1, 1, 1, 1})
        .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
        .build();
}

}  // namespace spade::sandbox::editor
