// visual_mesh.hpp -- --visual-mesh FILE (the lead, 2026-11-17: an accurate X500 for the web clips, PRESENTATION ONLY).
//
// A glTF mesh (render/gltf.hpp; KAT's x500.gltf is the whole drone in one file, metres, y up, +X forward, origin the centre of mass)
// drawn at the held body's pose in place of the model's proxy parts. The flow's solids are the model's and are untouched: nothing
// here reaches the solve, and the item is no wall in the flow's colouring. The file's own palette (MeshData::source_materials, the
// baseColorFactor of each material) is appended to the scene's and the copy's submesh_material moves by the offset, as
// scene_from_world() does for a resolved mesh (an out-of-range index to the file's first material); the caller's mesh is not changed.
#pragma once

#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"

namespace flow_movie {

// Refused, naming "visual mesh": no triangles, normals that do not match the positions. The scene is left as it was.
[[nodiscard]] inline spade::Result<void> add_visual_mesh(spade::render::RenderScene& scene, const spade::render::MeshData& mesh,
                                                         glm::vec3 pos, glm::quat orient) {
    const auto bad = [](const std::string& why) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "visual mesh: " + why});
    };
    if (mesh.positions.empty() || mesh.indices.size() < 3u) return bad("has no triangles");
    if (mesh.normals.size() != mesh.positions.size()) {
        return bad("its normals (" + std::to_string(mesh.normals.size()) + ") do not match its positions (" +
                   std::to_string(mesh.positions.size()) + ")");
    }
    spade::render::MeshData copy = mesh;
    if (!copy.source_materials.empty()) {
        const auto offset = static_cast<uint32_t>(scene.materials.size());
        const auto count = static_cast<uint32_t>(copy.source_materials.size());
        for (uint32_t& index : copy.submesh_material) index = offset + (index < count ? index : 0u);
        scene.materials.insert(scene.materials.end(), copy.source_materials.begin(), copy.source_materials.end());
        copy.source_materials.clear();  // the scene owns the palette now
    }
    spade::render::DrawItem item;
    item.mesh_index = static_cast<uint32_t>(scene.meshes.size());
    item.local_to_world = glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(orient);
    scene.meshes.push_back(std::move(copy));
    scene.statics.push_back(item);
    return {};
}

}  // namespace flow_movie
