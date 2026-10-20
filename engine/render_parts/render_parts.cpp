#include "render/mesh_resolver.hpp"
#include "render_parts/render_parts.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/csg_mesh.hpp"    // split_program, csg_subtree_*, mesh_csg_subtree
#include "render/tessellate.hpp"  // tessellate_primitive
#include "vehicles/model_part.hpp"
#include "world/aabb.hpp"
#include "world/builder.hpp"  // compose_transform
#include "world/mesh.hpp"

namespace spade::render_parts {
namespace {

[[nodiscard]] Error prefixed(const std::string& label, const Error& e) { return Error{e.code, label + ": " + e.context}; }

// A shape part's pieces, in its part frame, split as scene_from_world() splits
// a world: each union-primitive leaf tessellated at its own transform, each
// CSG root meshed over its bounds and kept as its subtree for the march, in
// node order (split_program()'s two ascending lists merged).
[[nodiscard]] Result<std::vector<PartPiece>> shape_pieces(const vehicles::PartDesc& part, const std::string& label) {
    const Result<Aabb> box = vehicles::part_bounds(part);
    if (!box) return std::unexpected(prefixed(label + ": bounds", box.error()));
    const render::Aabb bounds{.min = box->min, .max = box->max};
    const Result<render::SubtreeSplit> split = render::split_program(part.shape);
    if (!split) return std::unexpected(prefixed(label + ": shape", split.error()));
    const std::vector<uint32_t>& leaves = split->union_primitive_nodes;
    const std::vector<uint32_t>& roots = split->csg_roots;
    std::vector<PartPiece> pieces;
    pieces.reserve(leaves.size() + roots.size());
    std::size_t next_leaf = 0, next_root = 0;
    while (next_leaf < leaves.size() || next_root < roots.size()) {
        const bool take_leaf =
            next_root >= roots.size() || (next_leaf < leaves.size() && leaves[next_leaf] < roots[next_root]);
        PartPiece piece;
        if (take_leaf) {
            const SdfNode& node = part.shape.nodes[leaves[next_leaf++]];
            Result<render::MeshData> mesh =
                render::tessellate_primitive(static_cast<SdfPrim>(node.kind), node.params, bounds);
            if (!mesh) return std::unexpected(prefixed(label + ": tessellating", mesh.error()));
            piece.mesh = std::move(*mesh);
            if (node.transform < part.shape.transforms.size()) {
                piece.part_from_piece = glm::inverse(part.shape.transforms[node.transform].world_to_local);
            }
        } else {
            const uint32_t root = roots[next_root++];
            const Result<render::Aabb> sub = render::csg_subtree_world_bounds(part.shape, root, bounds);
            if (!sub) return std::unexpected(prefixed(label + ": CSG bounds", sub.error()));
            Result<render::MeshData> mesh = render::mesh_csg_subtree(part.shape, root, *sub);
            if (!mesh) return std::unexpected(prefixed(label + ": meshing CSG", mesh.error()));
            piece.mesh = std::move(*mesh);
            piece.is_csg = true;
            piece.csg = render::csg_subtree_program(part.shape, root);
            piece.csg_bounds = render::csg_subtree_sample_box(part.shape, root, *sub);
        }
        pieces.push_back(std::move(piece));
    }
    return pieces;
}

// `program`, a subtree in the part frame, composed into the world as
// vehicles::posed_program() composes a part: transforms[0] the identity, each
// part transform composed with world_to_part, only primitives re-indexed.
[[nodiscard]] SdfProgram world_program(const SdfProgram& program, const SdfTransform& world_to_part) {
    SdfProgram out{};
    out.transforms.push_back(SdfTransform{});
    for (const SdfTransform& t : program.transforms) out.transforms.push_back(compose_transform(t, world_to_part));
    for (SdfNode node : program.nodes) {
        if (node.op == static_cast<uint32_t>(SdfOp::none)) node.transform += 1u;
        out.nodes.push_back(node);
    }
    return out;
}

// The world box holding `box` (part frame) under `world_from_part`.
[[nodiscard]] render::Aabb posed_box(const render::Aabb& box, const glm::mat4& world_from_part) {
    render::Aabb out{.min = glm::vec3(std::numeric_limits<float>::max()),
                     .max = glm::vec3(std::numeric_limits<float>::lowest())};
    for (uint32_t c = 0; c < 8; ++c) {
        const glm::vec3 corner((c & 1u) != 0 ? box.max.x : box.min.x, (c & 2u) != 0 ? box.max.y : box.min.y,
                               (c & 4u) != 0 ? box.max.z : box.min.z);
        const glm::vec3 w = glm::vec3(world_from_part * glm::vec4(corner, 1.0f));
        out.min = glm::min(out.min, w);
        out.max = glm::max(out.max, w);
    }
    return out;
}

}  // namespace

Result<PreparedParts> prepare_parts(const vehicles::ModelType& model) {
    PreparedParts out;
    out.parts.reserve(model.parts.size());
    for (std::size_t i = 0; i < model.parts.size(); ++i) {
        const vehicles::PartDesc& part = model.parts[i];
        const std::string label =
            "prepare_parts: model '" + model.name + "': part " + std::to_string(i) + " '" + part.name + "'";
        PreparedPart prepared;
        prepared.on_rotor = part.attach == vehicles::PartAttach::rotor;
        prepared.porous = part.surface.kind == vehicles::PartSurface::Kind::porous;
        if (part.mesh != nullptr) {
            PartPiece piece;
            piece.mesh = render::flat_mesh(*part.mesh);  // one conversion with the mesh resolver's (TD-9)
            prepared.pieces.push_back(std::move(piece));
        } else if (!part.shape.empty()) {
            Result<std::vector<PartPiece>> pieces = shape_pieces(part, label);
            if (!pieces) return std::unexpected(pieces.error());
            prepared.pieces = std::move(*pieces);
        } else {
            return std::unexpected(Error{Code::invalid_argument, label + " has neither shape nor mesh"});
        }
        out.parts.push_back(std::move(prepared));
    }
    return out;
}

Result<void> bind_parts(const PreparedParts& prepared, const vehicles::PosedParts& posed, const PartsLook& look,
                        render::RenderScene& scene) {
    // Everything is checked before anything is appended.
    const std::size_t palette = scene.materials.size();
    const std::array<std::pair<const char*, uint32_t>, 3> materials{
        {{"body", look.body}, {"rotor", look.rotor}, {"porous", look.porous}}};
    for (const auto& [name, index] : materials) {
        if (index >= palette) {
            return std::unexpected(Error{Code::invalid_argument, std::string("bind_parts: look.") + name +
                                                                     " is material " + std::to_string(index) +
                                                                     ", past the scene's " + std::to_string(palette)});
        }
    }
    for (const vehicles::PosedPart& p : posed.parts) {
        if (p.part >= prepared.parts.size()) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "bind_parts: posed part " + std::to_string(p.part) +
                                             " is not one of the prepared model's " +
                                             std::to_string(prepared.parts.size()) +
                                             " parts; pose the model that was prepared"});
        }
    }

    for (const vehicles::PosedPart& p : posed.parts) {
        const PreparedPart& part = prepared.parts[p.part];
        const glm::mat4 world_from_part = glm::translate(glm::mat4(1.0f), p.origin) * glm::mat4_cast(p.orient);
        const uint32_t material = part.porous ? look.porous : part.on_rotor ? look.rotor : look.body;
        const bool wall = !part.porous && (!part.on_rotor || look.rotor_parts_are_walls);
        for (const PartPiece& piece : part.pieces) {
            render::DrawItem item;
            item.mesh_index = static_cast<uint32_t>(scene.meshes.size());
            item.local_to_world = world_from_part * piece.part_from_piece;
            item.material_override = material;
            item.flow_wall = wall;
            scene.meshes.push_back(piece.mesh);
            scene.statics.push_back(item);
            if (piece.is_csg) {
                scene.csg_subtrees.push_back(render::CsgSubtree{
                    .program = world_program(piece.csg, p.world_to_part),
                    .bounds = posed_box(piece.csg_bounds, world_from_part),
                    .draw_item = static_cast<uint32_t>(scene.statics.size() - 1u),
                });
            }
        }
    }
    return {};
}

}  // namespace spade::render_parts
