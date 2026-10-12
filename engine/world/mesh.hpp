#pragma once

// ---------------------------------------------------------------------------
// world/mesh.hpp -- a part's triangle mesh (CORE-6; the parts and meshes plan,
// Part B). The engine's one imported geometry: every reader (glTF, STL, OBJ,
// 3MF, the STEP converter's sidecars) ends in make_mesh(), and nothing else
// builds a TriangleMesh.
//
// WHAT make_mesh() GUARANTEES. A returned mesh is a closed, oriented,
// 2-manifold solid in declared units. That is the precondition of the sign
// test Task B2 adds (angle-weighted pseudonormals, Baerentzen and Aanaes
// 2005), so it is ENFORCED here, never assumed (L6: refuse, never repair).
// Task B2 adds the geometric half: self-intersection and shell nesting.
//
// UNITS ARE PART OF CONSTRUCTION. MeshInput::unit defaults to `unspecified`,
// which is refused: STL and OBJ carry no unit, so the caller must name one.
// Positions are scaled to metres in double and rounded to fp32 ONCE
// (float(v * mesh_unit_scale(unit))), in the part frame.
//
// THE CHECKS SEE THE STORED MESH. Every check below runs on the fp32 metre
// positions the mesh will hold, promoted to double (exact), so a mesh is
// validated as it is stored, not as it was handed in. Topology is by
// position: two vertices at one fp32 position are refused ("weld first";
// readers weld exact duplicates, value equality, before calling).
//
// REFUSALS, in this order (the first that applies is returned). Each message
// starts "<name>: " and reports the count and up to eight offending elements:
//   invalid_argument -- an unspecified (or unknown) unit;
//                       no triangles, or an index count not a multiple of 3;
//   capacity_exceeded -- more than kMaxMeshTriangles triangles (checked before
//                       any per-element pass);
//   invalid_argument -- an index out of range;
//                       a non-finite position, before or after scaling;
//                       an unreferenced vertex;
//                       two vertices at one position;
//                       a triangle that repeats an index;
//                       a degenerate triangle: in double,
//                         |e1 x e2| / 2 <= 1e-12 * D^2, D the bounds' diagonal;
//                       a duplicate triangle, in either orientation;
//                       an open edge (one triangle);
//                       a non-manifold edge (three or more triangles);
//                       a misoriented edge (two triangles, same direction);
//                       a non-manifold vertex (more than one fan);
//                       a total signed volume <= 0 ("inside out").
//
// DETERMINISM. Import-time work runs in double, in index order, with every
// sort under a total order and no unordered container; edge topology is one
// sorted vector of (min index, max index, triangle, direction). Only + - * /
// and sqrt (TD-3).
//
// THE CONTENT HASH is FNV-1a 64 (core/rng.hpp's constants), spelt byte by byte
// as model_identity is: the vertex count (u32), the triangle count (u32), each
// position's x, y, z as IEEE-754 float bits (u32), then every index (u32), all
// little-endian. It is computed once, at make_mesh(). It is what the YAML
// names a mesh by, and what stage 7 folds; nothing folds it yet.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"

namespace spade {

// A mesh's declared length unit. Underlying type fixed at uint32_t so it
// round-trips through a file field.
enum class MeshUnit : uint32_t {
    unspecified = 0,
    metre,
    millimetre,
    centimetre,
    micrometre,
    inch,
    foot,
};

// Metres per unit, exact by definition (the inch is 0.0254 m and the foot
// 0.3048 m, by the 1959 agreement). 0 for `unspecified` and for any value that
// is not a MeshUnit.
[[nodiscard]] double mesh_unit_scale(MeshUnit unit) noexcept;

// The most triangles a mesh may hold: 2^31.
inline constexpr uint64_t kMaxMeshTriangles = uint64_t{1} << 31;

// What a reader hands make_mesh().
struct MeshInput {
    std::vector<glm::dvec3> positions;  // in `unit`
    std::vector<uint32_t> indices;      // 3 per triangle, counter-clockwise seen from outside
    MeshUnit unit = MeshUnit::unspecified;
};

class TriangleMesh;

// Builds a mesh, or refuses it (the list above). `name` prefixes every
// message: "<name>: 3 open edges, first (4, 9) (9, 12) ...".
[[nodiscard]] Result<std::shared_ptr<const TriangleMesh>> make_mesh(const MeshInput& input,
                                                                    std::string_view name);

// Immutable and shared: only make_mesh() builds one, so every TriangleMesh
// meets the precondition above.
class TriangleMesh {
public:
    // fp32 metres, in the part frame, in input order.
    [[nodiscard]] std::span<const glm::vec3> positions() const noexcept { return positions_; }
    // As given: 3 per triangle, counter-clockwise seen from outside.
    [[nodiscard]] std::span<const uint32_t> indices() const noexcept { return indices_; }
    [[nodiscard]] uint32_t triangle_count() const noexcept {
        return static_cast<uint32_t>(indices_.size() / 3);
    }
    // Connected components (triangles joined across shared edges).
    [[nodiscard]] uint32_t shell_count() const noexcept { return shell_count_; }
    // FNV-1a 64, spelt as the header comment says.
    [[nodiscard]] uint64_t content_hash() const noexcept { return content_hash_; }

private:
    friend Result<std::shared_ptr<const TriangleMesh>> make_mesh(const MeshInput& input,
                                                                 std::string_view name);
    TriangleMesh() = default;

    std::vector<glm::vec3> positions_;
    std::vector<uint32_t> indices_;
    // Each triangle's shell, shells numbered in order of their lowest
    // triangle index. Task B2's two-level BVH and nesting check read it.
    std::vector<uint32_t> triangle_shell_;
    uint32_t shell_count_ = 0;
    uint64_t content_hash_ = 0;
};

}  // namespace spade
