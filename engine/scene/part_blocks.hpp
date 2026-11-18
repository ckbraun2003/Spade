#pragma once

// ---------------------------------------------------------------------------
// scene/part_blocks.hpp -- the version-2 part blocks and their one validator
// (spade::scene; L88, docs/design/core/plans/2026-11-17-spec-b-revision-
// proposal.md, Revision 2, A1 to A10).
//
// A catalogue part is ONE file, KAT's, written by KAT's editor. It holds KAT's
// keys (`provides`, `limits`, `emits`, `senses`, its own provenance) and five
// blocks that are Spade's: `geometry`, `mass_properties`, `install_points`,
// `source` and `material`, with one citation per block field under `sources`;
// and two for a part whose solid is already in the part store (L88l): `meshes`
// and `shapes`.
// Spade reads those blocks, and only those, through the two functions here;
// every other key is ignored, so the engine never depends on KAT's keys (D-3,
// as amended). The text is JSON or YAML (JSON is read as YAML).
//
// THE BLOCKS (every key of a block is listed; an unknown key is refused by its
// path):
//   id:              the part's id, a non-empty string (named in every issue).
//   geometry:        where the solid comes from.
//     source:        {format: step | 3mf | gltf | stl | obj | spmesh,
//                     sha256: 64 lower-case hex characters, the file's;
//                     object: optional, the object's name inside the file,
//                     unit: metre | millimetre | centimetre | micrometre |
//                     inch | foot, required for stl and obj, refused for the
//                     others, which declare their own}
//     to_part:       optional {axes: [a, b, c], scale: s}. `axes` is the part
//                    frame's x, y and z, each one of x, -x, y, -y, z, -z of
//                    the source, a permutation with signs whose determinant
//                    is +1 (a mirror reverses winding and is refused); the
//                    default is [x, y, z]. `scale` is above 0, finite,
//                    default 1. Applied once, exactly.
//     fallback:      optional, the generated stand-in the CI builds from when
//                    the file is absent: {kind: box, size: [x, y, z]} |
//                    {kind: tube, size: [radius, length]} | {kind: cylinder,
//                    size: [radius, height]}, every size finite and above 0.
//   mass_properties: spec-sheet numbers that override density x volume, in
//                    the part frame: `mass` (kg, above 0, required), `com` (m,
//                    optional), `inertia` (9 numbers, row-major, kg m^2,
//                    about the centre of mass; optional, and only with `com`).
//                    A tensor must be symmetric, positive definite and obey
//                    the triangle inequality of its principal moments.
//   install_points:  a list of {name, kind, pos: [x, y, z], orient: [w, x, y,
//                    z], spin, adjust_range}. `name` is unique in the part;
//                    `kind` is one of kInstallPointKinds; `orient` is a unit
//                    quaternion to 1e-6; `spin` (+1 or -1) is allowed on a
//                    `motor` point only. The compile never reads `kind`: it
//                    takes poses. `adjust_range` (optional) is how far a
//                    drone's parts_map `adjust` may move a part on this point:
//                    {pos_min: [x, y, z], pos_max: [x, y, z], yaw_deg: [min, max],
//                    pitch_deg: [min, max], roll_deg: [min, max]}, metres and
//                    degrees (the keys of the library contract's `adjust_range`);
//                    every key optional, a missing one is no range (0, 0);
//                    every min is at most its max; an angle is within 360.
//   meshes:          a non-empty list of {store: {sha256, file}, hash, role},
//                    the part's solid as objects already in the part store,
//                    every one in the part's own frame, metres (no conversion
//                    is applied). `store.sha256` is 64 lower-case hex and
//                    `store.file` a bare file name (no directory part); `hash`
//                    (optional) is the mesh's 16 lower-case hex content hash,
//                    spade_part_import's; `role` is `solid` (the default) or
//                    `visual` (drawn only: it is not a solid and no pass
//                    counts it). The same {sha256, file} twice is refused.
//   shapes:          a non-empty list of dimensioned stand-ins, each {kind:
//                    box, size: [x, y, z]} or {kind: cylinder, size: [radius,
//                    length]} (its axis the part frame's z), every size finite
//                    and above 0, centred on the part frame's origin unless
//                    it gives `pos` and `orient` (as an install point does).
//   source:          {id, url, license, retrieved: YYYY-MM-DD}, every one a
//                    non-empty string; the licence is recorded, not enforced.
//   material:        a name, reserved for L88a: a non-empty string of at most
//                    64 characters with no control character; stored and
//                    ignored by every pass.
//   sources:         a mapping from a block field's path to its citation
//                    {source: <id>, grade: <grade>, note: "<text>"} (scene/
//                    library_sources.hpp's form). The cited paths are
//                    mass_properties.mass, mass_properties.com and
//                    mass_properties.inertia (each when declared) and
//                    install_points.<name> (one for each point); entries for
//                    other paths are KAT's and ignored. A grade other than
//                    assumed cites a source; derived and assumed carry a note.
//                    Registry membership of a source id is the library's
//                    check, not this one: KAT's files have no registry.
// A document needs at least one block; `geometry` needs `source`.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "scene/library_sources.hpp"
#include "world/mesh.hpp"

namespace spade::scene {

inline constexpr std::array<std::string_view, 12> kInstallPointKinds = {
    "motor", "prop_hub", "battery", "esc", "fc", "companion", "camera", "gnss", "radio", "imu", "vtx", "antenna"};

enum class GeometryFormat : uint32_t { step = 0, threemf, gltf, stl, obj, spmesh };

// The text spelling: step, 3mf, gltf, stl, obj, spmesh.
[[nodiscard]] std::string_view geometry_format_name(GeometryFormat format) noexcept;

struct GeometryFallback {
    enum class Kind : uint32_t { box = 0, tube, cylinder };
    Kind kind = Kind::box;
    std::vector<double> size{};  // box: x, y, z; tube: radius, length; cylinder: radius, height
};

struct PartGeometry {
    GeometryFormat format = GeometryFormat::spmesh;
    std::string sha256{};
    std::string object{};  // empty: the file's only object
    MeshUnit unit = MeshUnit::unspecified;
    // The part frame's x, y, z as a signed, 1-based source axis: +1 is the
    // source's x, -3 its -z. A permutation with signs, determinant +1.
    std::array<int, 3> axes{1, 2, 3};
    double scale = 1.0;
    std::optional<GeometryFallback> fallback{};
};

struct PartMassProperties {
    double mass = 0.0;                 // kg
    std::optional<glm::dvec3> com{};   // m, part frame
    std::optional<glm::dmat3> inertia{};  // kg m^2 about the centre of mass, part frame, symmetric
};

// How far a parts_map `adjust` may move a part on an install point: metres and degrees, each pair [min, max].
struct AdjustRange {
    glm::dvec3 pos_min{0.0};
    glm::dvec3 pos_max{0.0};
    std::array<double, 2> yaw_deg{0.0, 0.0};
    std::array<double, 2> pitch_deg{0.0, 0.0};
    std::array<double, 2> roll_deg{0.0, 0.0};
};

struct PartInstallPoint {
    std::string name{};
    std::string kind{};  // one of kInstallPointKinds
    glm::dvec3 pos{0.0};
    glm::dquat orient{1.0, 0.0, 0.0, 0.0};
    int spin = 0;  // +1 or -1 on a motor point; 0 none
    std::optional<AdjustRange> adjust_range{};
};

// One object of the part's solid, in the part store (a store_ref {sha256, file}).
struct PartMesh {
    std::string sha256{};
    std::string file{};
    std::string hash{};  // the mesh's content hash, 16 hex; empty: none declared
    bool visual = false;  // role: visual
};

struct PartShape {
    enum class Kind : uint32_t { box = 0, cylinder };
    Kind kind = Kind::box;
    std::vector<double> size{};  // box: x, y, z; cylinder: radius, length (axis z)
    glm::dvec3 pos{0.0};
    glm::dquat orient{1.0, 0.0, 0.0, 0.0};
};

struct PartSourceRecord {
    std::string id{};
    std::string url{};
    std::string license{};
    std::string retrieved{};  // YYYY-MM-DD
};

// The validated blocks of one part.
struct PartBlocks {
    std::string id{};
    std::optional<PartGeometry> geometry{};
    std::optional<PartMassProperties> mass_properties{};
    std::vector<PartInstallPoint> install_points{};
    std::optional<PartSourceRecord> source{};
    std::string material{};  // empty: none
    std::vector<PartMesh> meshes{};
    std::vector<PartShape> shapes{};
    // Citations by field path ("mass_properties.mass", "install_points.base"), only the cited paths of the blocks.
    std::map<std::string, SourceRef> sources{};
};

// One problem in a document: the code (invalid_argument for a rule broken,
// schema_mismatch is not used), the part's id (empty when the id itself is
// missing), the field's dotted path ("geometry.to_part.axes",
// "install_points[2].spin"; "" for the document) and a text the user can act on.
struct PartIssue {
    Code code = Code::invalid_argument;
    std::string part_id{};
    std::string field{};
    std::string message{};
};

// Every issue of the document, in document order, not only the first; empty
// means the blocks are valid. Never throws, reads no disk; a document that is
// not a mapping, or does not parse, is one issue with an empty field.
[[nodiscard]] std::vector<PartIssue> validate_part_blocks(std::string_view text);

// The validated blocks. On any issue the first one comes back as the Error:
// its code, and its context "part '<id>': <field>: <message>".
[[nodiscard]] Result<PartBlocks> read_part_blocks(std::string_view text);

}  // namespace spade::scene
