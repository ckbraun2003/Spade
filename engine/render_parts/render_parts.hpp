// ---------------------------------------------------------------------------
// render_parts/render_parts.hpp -- drawing a model's parts (spade::render_parts).
//
// RND-9 (the lead, 2026-10-13); docs/design/rendering/plans/
// 2026-10-13-render-parts-plan.md. A model's parts, posed by Core's
// vehicles::posed_parts(), as items of a render scene: a mesh part as a draw
// item of its own triangles, a shape part split as scene_from_world() splits a
// world (render/csg_mesh.hpp's split_program()): each union-primitive leaf
// tessellated, each CSG root meshed and marched per pixel (RS3).
//
// prepare_parts() builds every part's geometry ONCE, in the part's own frame;
// bind_parts() poses it for one frame. The library sits above render/ and
// vehicles/, and nothing there includes it, so dependencies point strictly
// downward. flow_movie (D1) and the sandbox (D2) draw a drone through it.
//
// bind_parts() writes RenderScene::statics, because a CsgSubtree indexes
// statics: a caller that poses a moving model rebuilds its scene each frame,
// as flow_movie does. A scene built once and posed per frame (RS2) needs
// moving CSG subtrees, which no renderer has yet (the backlog's row).
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/part_pose.hpp"
#include "world/sdf.hpp"

namespace spade::render_parts {

// How the parts draw. Materials are indices into the scene's palette. A rotor
// part is no wall in the flow under the actuator disc and line tiers, which
// never see its surface (RND-8); resolved blades (P5) set
// rotor_parts_are_walls. A porous part is never a wall.
struct PartsLook {
    uint32_t body = 0;    // a body part's material
    uint32_t rotor = 0;   // a rotor part's
    uint32_t porous = 0;  // a porous part's, on the body or a rotor
    bool rotor_parts_are_walls = false;
};

// One piece of a part's geometry, in the part's frame.
struct PartPiece {
    render::MeshData mesh;
    glm::mat4 part_from_piece{1.0f};  // the piece's placement in the part frame
    // A CSG root: its subtree program and the box it is sampled in, both in
    // the part frame. Empty for a tessellated piece or a mesh part's.
    bool is_csg = false;
    SdfProgram csg{};
    render::Aabb csg_bounds{};
};

// One part, prepared: its pieces and what decides its look.
struct PreparedPart {
    std::vector<PartPiece> pieces;
    bool on_rotor = false;
    bool porous = false;
};

// A model's parts, prepared: parts[i] is the model's parts[i].
struct PreparedParts {
    std::vector<PreparedPart> parts;
};

// Prepares every part of `model`. invalid_argument for a part with neither
// shape nor mesh; whatever part_bounds(), split_program(),
// tessellate_primitive() or mesh_csg_subtree() refuse, prefixed with the
// part's name.
[[nodiscard]] Result<PreparedParts> prepare_parts(const vehicles::ModelType& model);

// Appends one frame's posed parts to `scene`: for each posed part, in order,
// each piece's mesh and a draw item at the part's world pose times the
// piece's own, with the part's material and flow_wall (a wall on the body; a
// rotor part only if look says so; never porous); and a CsgSubtree per CSG
// piece, its program composed into the world as posed_program() composes and
// its bounds the posed box's. invalid_argument, naming it, for a posed part
// that is not one of the prepared model's, or a look material not in the
// scene's palette; nothing is appended then.
[[nodiscard]] Result<void> bind_parts(const PreparedParts& prepared, const vehicles::PosedParts& posed,
                                      const PartsLook& look, render::RenderScene& scene);

}  // namespace spade::render_parts
