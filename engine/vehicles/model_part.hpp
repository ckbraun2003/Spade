#pragma once

// ---------------------------------------------------------------------------
// vehicles/model_part.hpp -- a model's solid geometry, part by part (CORE-7;
// docs/design/core/2026-10-12-model-parts-design.md, and the parts and meshes
// plan, Task A1). vehicles/model_type.hpp includes this header, so every
// includer of ModelType sees these types.
//
// A PART IS DESCRIPTION, LIKE THE REST OF A ModelType. It carries no mass:
// BodyTemplate stays the only mass the step reads, and the airframe compile's
// part record is the one source for both (TD-9). Nothing in step() reads a
// part. register_model() validates and copies them once.
//
// FRAMES. A part's shape is in its own part frame. Its mount takes the part
// frame to the BODY frame (the principal frame every other mount uses), or,
// for a part on rotor k, to that rotor's HUB frame (+Y is the thrust axis),
// with the blade azimuth psi = 0 at the authored pose. Authors work in the
// design frame through AirframeSpec; the compile converts each mount once.
//
// GEOMETRY: EXACTLY ONE OF TWO FORMS (CORE-6, import only; the parts and
// meshes plan, Task B4). A part's solid is an SdfProgram (`shape`) or a
// triangle mesh (`mesh`), never both and never neither. A mesh is the
// authoring path: geometry comes from CAD, through make_mesh(). One world
// SDF cannot hold a mesh (an SdfProgram has no mesh node), so the two forms
// stay apart, and part_sample() is the one interface that reads either.
//
// NOT FOLDED. Parts do not enter model_identity() until a step reads them
// (stage 7, CORE-7), so adding parts moves no identity and refuses no stored
// v3 blob. A mesh part folds nothing either; its TriangleMesh::content_hash()
// is what stage 7 will fold for its geometry.
//
// Every rule below is checked by ModelType::issues(), which names the part
// (index and name) and the field.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "world/mesh.hpp"
#include "world/sdf.hpp"

namespace spade::vehicles {

// What a part is fixed to. Later kinds (a wheel, a hinge; F1) append here.
enum class PartAttach : uint32_t { body = 0, rotor = 1 };

// What the fluid sees (Physics' answer, 2026-10-12). A wall is a solid
// boundary moving with the part. A porous part is a Darcy-Forchheimer sink
// over its volume; with both coefficients 0 it is transparent. A wall's
// coefficients are both 0. A part's surface never touches a drag element:
// whether a drag element applies follows the airflow coupling mode.
struct PartSurface {
    enum class Kind : uint32_t { wall = 0, porous = 1 };
    Kind kind = Kind::wall;
    float inv_permeability = 0.0f;  // porous: 1/K, 1/m^2 (Darcy); finite, >= 0; 0 on a wall
    float inertial_coeff = 0.0f;    // porous: C2, 1/m (Forchheimer); finite, >= 0; 0 on a wall
};

// "No part": the index a per-part query returns when no part answers.
inline constexpr uint32_t kNoPart = std::numeric_limits<uint32_t>::max();

// One part. Every member has a default, so gcc's missing-field-initializers
// never fires on a designated initializer that leaves one out.
//
//   name          non-empty, unique in the model.
//   role          a free label ("body", "arm", "motor"); the engine never
//                 reads it (L7).
//   attach/rotor  rotor: the part turns with rotor `rotor`'s azimuth, and
//                 `rotor` < rotors.size(); body: `rotor` is 0.
//   mount         part frame -> BODY frame (a rotor part: -> its hub frame),
//                 m; finite, and an orientation of non-zero length. psi = 0
//                 at the authored pose.
//   shape         the solid as an SdfProgram, in the part frame: valid,
//                 bounded (a plane or a heightfield only inside an intersect,
//                 or cut from a bounded shape), with no node_materials (render
//                 data). Empty on a mesh part.
//   mesh          the solid as a triangle mesh, in the part frame, in metres
//                 (make_mesh() scaled it once; the mount places it). Shared
//                 and immutable: copying a part, as register_model() does,
//                 shares it. Every TriangleMesh came from make_mesh(), so a
//                 non-null mesh is valid. Null on an SdfProgram part. Exactly
//                 one of shape and mesh is set (refused at field `shape`).
//   min_feature   m, the smallest feature the grid must resolve (CORE-8 D5);
//                 finite and > 0. An authored part declares it; the airframe
//                 compile will fill it from its primitives' sizes (Task A4).
struct PartDesc {
    std::string name{};
    std::string role{};
    PartAttach attach = PartAttach::body;
    uint32_t rotor = 0;
    glm::vec3 mount_pos{0.0f};
    glm::quat mount_orient{1.0f, 0.0f, 0.0f, 0.0f};
    SdfProgram shape{};
    std::shared_ptr<const TriangleMesh> mesh{};
    PartSurface surface{};
    float min_feature = 0.0f;
};

// One part's field at p_part, a point in its PART frame (m): the one
// evaluation interface a solver reads a part through, whichever form it has.
// An SdfProgram part answers sample(shape, p_part). A mesh part answers
// mesh_sample(*mesh, p_part): the exact distance (negative inside) and the
// gradient, by the same {distance, gradient} contract -- a unit vector
// pointing out of the solid. Posing a part (its mount, its rotor's turn) is
// the caller's: it maps a world point into the part frame first.
//
// A host call, noexcept, and it never allocates: it reads the mesh through the
// shared pointer without copying it. PRECONDITION: `part` passed
// ModelType::validate() (exactly one form set).
[[nodiscard]] SdfSample part_sample(const PartDesc& part, glm::vec3 p_part) noexcept;

}  // namespace spade::vehicles
