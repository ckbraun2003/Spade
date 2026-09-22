#pragma once

// ---------------------------------------------------------------------------
// sdf_program.hpp (S6 Task 6) -- the DEVICE-SIDE image of every world's static
// SDF program, and the flattening that produces it.
//
// WHY THE SDF NEEDS A DEVICE IMAGE AT ALL. CollisionStatic tests every active
// body against its world's signed distance field (physics/contacts.cpp), and
// that field is a per-world spade::SdfProgram held by Simulation as
// CONFIGURATION -- it is not a registered array, so it never appears in the
// ArenaSet walk StateMirror::upload()/readback() exchange. The GPU pass needs
// it anyway. Uploading it is therefore backend-internal DERIVED STORAGE, the
// same category dryden_params (bindings.slang section C) already occupies and
// exactly the category backend work is allowed to add: derived storage, no
// register_array call.
//
// The canonical wording is compute/step_params.hpp's and
// compute/vulkan/backend.hpp's: global constraint: "S6 adds NO register_array
// call". (This site said "registered state is what is frozen, at 18 walk
// entries" -- a paraphrase that generalised past its own subject and acquired a
// number; see compute/vulkan/state_mirror.hpp. The point made here survives
// verbatim: what makes this buffer legal is that it is DERIVED, not that the
// walk cannot grow.)
//
// ---------------------------------------------------------------------------
// THREE BUFFERS, NOT ONE, AND WHY THE SPLIT IS THIS ONE
//
// A world set holds N independent programs, each an array of nodes plus an
// array of transforms, and a dispatch covering N worlds must be able to walk
// ITS OWN world's program. So:
//
//   sdf_nodes        -- every world's nodes, concatenated in world order.
//   sdf_transforms   -- every world's transforms, concatenated in world order.
//   sdf_world_ranges -- one SdfWorldRange per world: where that world's nodes
//                       start, how many there are, and where its transforms
//                       start.
//
// A node's `transform` field stays WORLD-LOCAL (it is copied verbatim out of
// the SdfProgram, unrebased), and the kernel adds `range.transform_begin` when
// it indexes. Rebasing at upload time was the alternative and was rejected: it
// would make the uploaded node bytes differ from the CPU program's, so a byte
// comparison of the two -- the cheapest possible check that the upload is
// faithful -- would stop being available.
//
// ---------------------------------------------------------------------------
// WHY SdfNodeRow AND SdfTransformRow EXIST INSTEAD OF MIRRORING spade::SdfNode
// AND spade::SdfTransform DIRECTLY. Two independent reasons, each sufficient.
//
//   1. ALIGNMENT. layouts.slang's generated checks assert
//      `alignof(C++) % <std430 alignment> == 0`, and a Slang struct containing
//      a float4 reports std430 alignment 16. glm::vec4 with this project's glm
//      configuration has alignof 4 (no GLM_FORCE_DEFAULT_ALIGNED_GENTYPES), so
//      spade::SdfNode -- which is not alignas(16) and has no reason to be, it
//      is a host-side POD in a std::vector -- would fail that assert. The rows
//      below are alignas(kStd430StructAlignment), like every other mirrored
//      state row in this engine.
//   2. MATRIX ORDER. spade::SdfTransform holds a glm::mat4. Slang's float4x4
//      and glm's mat4 disagree about whether the first 16 bytes are a row or a
//      column, and P2 forbids OpMatrixTimesVector outright anyway (the
//      accumulation order of a matrix product is unspecified), so the kernel
//      has to spell the transform out column by column regardless. Naming the
//      four columns as four float4 FIELDS removes the ambiguity at the layout
//      level rather than leaving it to a compile flag.
//
// The static_asserts at the bottom of this file pin SdfNodeRow's byte image
// against spade::SdfNode's, field by field, so the two cannot drift -- which is
// what makes the memcpy-shaped conversion in flatten_sdf_programs() honest.
// SdfTransformRow's columns are likewise pinned against the glm::mat4's own
// column-major storage.
//
// HEADER-ONLY, DELIBERATELY. flatten_sdf_programs() copies PODs into vectors
// and calls nothing out of spade_world's compiled objects (never eval(), never
// validate()), so this adds no link edge to spade_compute -- the same posture
// compute/vulkan/state_mirror.cpp already takes toward physics/forces.hpp and
// world/medium.hpp: include for the declarations, link nothing.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

#include <glm/vec4.hpp>

#include "state/layout.hpp"  // kStd430StructAlignment
#include "world/sdf.hpp"

namespace spade::compute {

// ---------------------------------------------------------------------------
// SdfNodeRow -- one flattened program node, device image. Byte-identical to
// spade::SdfNode (asserted below); the separate type exists only to carry the
// std430 alignment the generated layout checks require. Field names match
// spade::SdfNode's exactly, because the generated asserts index BOTH by name.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) SdfNodeRow {
    uint32_t kind = 0;       // SdfPrim; meaningful only when op == SdfOp::none
    uint32_t op = 0;         // SdfOp
    uint32_t transform = 0;  // WORLD-LOCAL index; the kernel adds transform_begin
    uint32_t _pad = 0;       // keeps params on a 16-byte boundary for std430
    glm::vec4 params{0.0f};
};

// ---------------------------------------------------------------------------
// SdfTransformRow -- one node pose, device image: the pre-inverted
// world_to_local matrix spelled as its four COLUMNS in glm's own storage order
// (glm::mat4 is column-major, so col0 is its first 16 bytes), plus the uniform
// scale and world/sdf.hpp's three reserved lanes.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) SdfTransformRow {
    glm::vec4 col0{1.0f, 0.0f, 0.0f, 0.0f};  // world_to_local[0]
    glm::vec4 col1{0.0f, 1.0f, 0.0f, 0.0f};  // world_to_local[1]
    glm::vec4 col2{0.0f, 0.0f, 1.0f, 0.0f};  // world_to_local[2]
    glm::vec4 col3{0.0f, 0.0f, 0.0f, 1.0f};  // world_to_local[3]
    float scale = 1.0f;
    float _r0 = 0.0f;  // reserved; must stay 0 (SdfTransform::_pad[3])
    float _r1 = 0.0f;
    float _r2 = 0.0f;
};

// ---------------------------------------------------------------------------
// SdfWorldRange -- where world w's program lives in the two concatenated
// buffers. `node_count == 0` means an EMPTY program, which is a legal world
// (tests/golden/worlds/ballistic.world.yaml has no geometry at all) and which
// the kernel must answer exactly as world/sdf.cpp's eval()/sample() do for an
// empty node list: kSdfEmptyDistance and a zero gradient.
// ---------------------------------------------------------------------------
struct SdfWorldRange {
    uint32_t node_begin = 0;
    uint32_t node_count = 0;
    uint32_t transform_begin = 0;
    uint32_t _r0 = 0;  // reserved; must stay 0
};

// What flatten_sdf_programs() produces: the three buffers, ready to upload.
struct SdfUpload {
    std::vector<SdfNodeRow> nodes;
    std::vector<SdfTransformRow> transforms;
    std::vector<SdfWorldRange> ranges;  // exactly one per world, in world order
};

// ---------------------------------------------------------------------------
// Concatenates every world's program into one upload. `programs[w]` is world
// w's SdfProgram; a null pointer is treated as an empty program.
//
// Total-function, allocating only the three vectors; it validates nothing (the
// programs it is handed have already passed SdfProgram::validate() at
// Simulation::create(), exactly as the CPU pass's own precondition says).
// ---------------------------------------------------------------------------
[[nodiscard]] inline SdfUpload flatten_sdf_programs(std::span<const SdfProgram* const> programs) {
    SdfUpload out;
    out.ranges.reserve(programs.size());

    std::size_t total_nodes = 0;
    std::size_t total_transforms = 0;
    for (const SdfProgram* program : programs) {
        if (program == nullptr) continue;
        total_nodes += program->nodes.size();
        total_transforms += program->transforms.size();
    }
    out.nodes.reserve(total_nodes);
    out.transforms.reserve(total_transforms);

    for (const SdfProgram* program : programs) {
        SdfWorldRange range;
        range.node_begin = static_cast<uint32_t>(out.nodes.size());
        range.transform_begin = static_cast<uint32_t>(out.transforms.size());
        range.node_count = program == nullptr ? 0u : static_cast<uint32_t>(program->nodes.size());

        if (program != nullptr) {
            for (const SdfTransform& t : program->transforms) {
                SdfTransformRow row;
                row.col0 = t.world_to_local[0];
                row.col1 = t.world_to_local[1];
                row.col2 = t.world_to_local[2];
                row.col3 = t.world_to_local[3];
                row.scale = t.scale;
                row._r0 = t._pad[0];
                row._r1 = t._pad[1];
                row._r2 = t._pad[2];
                out.transforms.push_back(row);
            }
            for (const SdfNode& n : program->nodes) {
                SdfNodeRow row;
                row.kind = n.kind;
                row.op = n.op;
                row.transform = n.transform;  // WORLD-LOCAL, unrebased -- see the header note
                row._pad = n._pad;
                row.params = n.params;
                out.nodes.push_back(row);
            }
        }
        out.ranges.push_back(range);
    }
    return out;
}

// ---------------------------------------------------------------------------
// The layout battery, in state/layout.hpp's style and for its reason. The first
// group pins the device rows' own std430 images; the second pins SdfNodeRow
// against spade::SdfNode field for field, which is what makes "byte-identical
// to the host node, only re-aligned" a checked statement rather than a claim.
// ---------------------------------------------------------------------------
static_assert(std::is_standard_layout_v<SdfNodeRow>);
static_assert(std::is_trivially_copyable_v<SdfNodeRow>);
static_assert(sizeof(SdfNodeRow) == 32, "std430 array stride");
static_assert(alignof(SdfNodeRow) == 16, "std430 base alignment");
static_assert(offsetof(SdfNodeRow, kind) == 0);
static_assert(offsetof(SdfNodeRow, op) == 4);
static_assert(offsetof(SdfNodeRow, transform) == 8);
static_assert(offsetof(SdfNodeRow, _pad) == 12);
static_assert(offsetof(SdfNodeRow, params) == 16);

static_assert(sizeof(SdfNodeRow) == sizeof(SdfNode),
              "SdfNodeRow must be spade::SdfNode's byte image, only re-aligned");
static_assert(offsetof(SdfNodeRow, kind) == offsetof(SdfNode, kind));
static_assert(offsetof(SdfNodeRow, op) == offsetof(SdfNode, op));
static_assert(offsetof(SdfNodeRow, transform) == offsetof(SdfNode, transform));
static_assert(offsetof(SdfNodeRow, _pad) == offsetof(SdfNode, _pad));
static_assert(offsetof(SdfNodeRow, params) == offsetof(SdfNode, params));

static_assert(std::is_standard_layout_v<SdfTransformRow>);
static_assert(std::is_trivially_copyable_v<SdfTransformRow>);
static_assert(sizeof(SdfTransformRow) == 80, "std430 array stride");
static_assert(alignof(SdfTransformRow) == 16, "std430 base alignment");
static_assert(offsetof(SdfTransformRow, col0) == 0);
static_assert(offsetof(SdfTransformRow, col1) == 16);
static_assert(offsetof(SdfTransformRow, col2) == 32);
static_assert(offsetof(SdfTransformRow, col3) == 48);
static_assert(offsetof(SdfTransformRow, scale) == 64);
static_assert(offsetof(SdfTransformRow, _r0) == 68);
static_assert(offsetof(SdfTransformRow, _r1) == 72);
static_assert(offsetof(SdfTransformRow, _r2) == 76);
// The four columns occupy exactly the glm::mat4's own 64 bytes, in its own
// storage order, and `scale` follows immediately -- i.e. SdfTransformRow's
// image IS spade::SdfTransform's, with the matrix's columns named.
static_assert(sizeof(SdfTransformRow) == sizeof(SdfTransform),
              "SdfTransformRow must be spade::SdfTransform's byte image, columns named");
static_assert(offsetof(SdfTransformRow, scale) == offsetof(SdfTransform, scale));
static_assert(offsetof(SdfTransformRow, _r0) == offsetof(SdfTransform, _pad));

static_assert(std::is_standard_layout_v<SdfWorldRange>);
static_assert(std::is_trivially_copyable_v<SdfWorldRange>);
static_assert(sizeof(SdfWorldRange) == 16, "std430 array stride");
static_assert(offsetof(SdfWorldRange, node_begin) == 0);
static_assert(offsetof(SdfWorldRange, node_count) == 4);
static_assert(offsetof(SdfWorldRange, transform_begin) == 8);
static_assert(offsetof(SdfWorldRange, _r0) == 12);

}  // namespace spade::compute
