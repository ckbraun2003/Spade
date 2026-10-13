#pragma once

// ---------------------------------------------------------------------------
// world/mesh.hpp -- a part's triangle mesh (CORE-6; the parts and meshes plan,
// Part B). The engine's one imported geometry: every reader (glTF, STL, OBJ,
// 3MF, the STEP converter's sidecars) ends in make_mesh(), and nothing else
// builds a TriangleMesh.
//
// WHAT make_mesh() GUARANTEES. A returned mesh is a closed, oriented,
// 2-manifold solid in declared units, with no self-intersection, whose
// shells nest so the winding number is 0 or 1 everywhere. That is exactly
// the precondition of mesh_sample()'s sign test, so it is ENFORCED here,
// never assumed (L6: refuse, never repair): the combinatorial half first,
// then the geometric half.
//
// THE FIELD: mesh_sample() and mesh_eval(), fp32, host calls (no step reads
// a mesh yet).
//   * DISTANCE is exact: the closest point of the closest triangle, by
//     Ericson's region classification (math::closest_on_triangle), found
//     through a two-level BVH and then square-rooted once.
//   * THE SIGN is (p - c) . n, n the angle-weighted pseudonormal of the
//     feature c lies on (Baerentzen and Aanaes 2005): a face's normal; an
//     edge's two face normals summed; a vertex's face normals weighted by
//     their corner angles. On a closed, oriented, 2-manifold surface with no
//     self-intersection, that sign is provably right at every point -- the
//     reason for every refusal below. Triangle-counted or area-weighted
//     vertex normals are not (tests/test_mesh_field.cpp shows a knife's tip).
//     WHY NOT A WINDING NUMBER: it needs an atan2 per triangle per query
//     (TD-3 forbids a libm one on a state path) and is O(n) per query; its
//     one gain, a sign for open or broken meshes, buys nothing here, because
//     those are refused.
//   * THE TIE RULE: the closest triangle is the minimum of (squared
//     distance, triangle index), and an edge is projected from its
//     lower-indexed end (a shared edge gives both triangles the same bits),
//     so the answer never depends on the traversal: a BVH query equals the
//     brute-force loop bit for bit. The BVH prunes a node only when its
//     box's squared distance, computed as a triangle's is, EXCEEDS the best
//     so far, and every closest point lies in its triangle's box, so no
//     candidate that could win or tie is ever pruned.
//   * THE PSEUDONORMALS and the corner angles are computed once, at
//     make_mesh(), in double (the angles by math::atan2_arith, Spade's own:
//     + - * / and sqrt only), and rounded to fp32 once. The query is + - * /
//     and sqrt in fp32 in a pinned order, so MSVC and gcc agree bit for bit
//     (TD-12), and it is written to be mirrored on the GPU.
//
// THE BVH is two-level -- shells, then each shell's triangles -- so a query
// can keep to one shell (the nesting check does). Each level is built top
// down: a node's items are sorted by (centroid on the longest axis of the
// centroids' bounds, index) -- ties to the lower axis -- and split at the
// median, count / 2 to the left; a node of 4 items or fewer is a leaf. A
// triangle's centroid is (a + b + c) / 3 in double, a shell's the centre of
// its box; a box is the fp32 min and max of the vertices (exact). The build
// depends on the input order alone, and the tree is never stored.
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
//                       a total signed volume <= 0 ("inside out");
//   capacity_exceeded -- more than kMaxMeshShells shells (connected
//                       components);
//   invalid_argument -- a self-intersection: every pair of triangles whose
//                         boxes touch, from the BVH, in (lower, higher)
//                         index order, tested in double. A pair sharing no
//                         vertex meets if Moller's plane test cannot part
//                         them and an edge of one meets the other; a pair
//                         sharing a vertex meets if either's far edge meets
//                         the other; a pair sharing an edge only if it folds
//                         flat (coplanar, normals opposed). Touching counts,
//                         and so does a zero determinant (conservative);
//                       wrong nesting: each shell's signed volume gives its
//                         orientation (outward > 0, inward < 0, a cavity's
//                         wall). For shell s, sum +1 for each other outward
//                         shell that contains s's lowest-indexed vertex and
//                         -1 for each inward one, by the sign test on that
//                         shell alone. An outward shell needs 0, an inward
//                         one 1: exactly "the winding number is 0 or 1
//                         everywhere". A shell of zero volume is refused too.
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
// names a mesh by, and what vehicles::parts_identity() folds for a mesh part.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "world/sdf.hpp"

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

// The most shells a mesh may hold: the nesting check is O(shells^2) queries.
inline constexpr uint32_t kMaxMeshShells = 4096;

// The field at a point (the header comment's THE FIELD).
struct MeshSample {
    float distance = kSdfEmptyDistance;  // exact Euclidean distance to the surface, negative inside
    glm::vec3 gradient{0.0f};            // sign * (p - c) / |p - c|; on the surface, the feature's unit pseudonormal
    uint32_t triangle = 0;               // the closest: minimum (distance squared, triangle index)
};

// The distance, sign and gradient at p, in the mesh's frame (metres). The
// gradient is (p - c) / |p - c| componentwise, negated inside, c the closest
// point; at the surface (|p - c| = 0) it is the closest feature's unit
// pseudonormal and the distance is +0. A non-finite p returns MeshSample{}.
// p must keep |p - c|^2 finite in fp32 (|p| well under 1e19 m).
[[nodiscard]] MeshSample mesh_sample(const TriangleMesh& mesh, glm::vec3 p) noexcept;

// mesh_sample(mesh, p).distance.
[[nodiscard]] float mesh_eval(const TriangleMesh& mesh, glm::vec3 p) noexcept;

namespace detail {

// One node of a mesh's BVH: internal layout, derived at make_mesh() and never
// stored. A leaf holds `count` (1 to 4) items from `first`; an inner node has
// count 0 and its two children at `first` and `first + 1`.
struct MeshBvhNode {
    glm::vec3 lo{0.0f};
    uint32_t first = 0;
    glm::vec3 hi{0.0f};
    uint32_t count = 0;
};

}  // namespace detail

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
    friend MeshSample mesh_sample(const TriangleMesh& mesh, glm::vec3 p) noexcept;
    TriangleMesh() = default;

    // The query, over every shell or over `shell` alone (kEveryShell: all).
    static constexpr uint32_t kEveryShell = 0xFFFFFFFFu;
    [[nodiscard]] MeshSample query(glm::vec3 p, uint32_t shell) const noexcept;

    std::vector<glm::vec3> positions_;
    std::vector<uint32_t> indices_;
    // Each triangle's shell, shells numbered in order of their lowest
    // triangle index.
    std::vector<uint32_t> triangle_shell_;
    uint32_t shell_count_ = 0;
    uint64_t content_hash_ = 0;

    // Derived at make_mesh() and never stored: unit pseudonormals, rounded
    // to fp32 once, and the two-level BVH.
    std::vector<glm::vec3> face_normal_;    // per triangle
    std::vector<glm::vec3> edge_normal_;    // 3 per triangle: slot k is the edge from corner k to corner k + 1
    std::vector<glm::vec3> vertex_normal_;  // per vertex, angle-weighted
    std::vector<detail::MeshBvhNode> shell_nodes_;     // the top level, root 0: its leaves hold shells
    std::vector<uint32_t> shell_items_;                // shell indices, as the top level's leaves order them
    std::vector<detail::MeshBvhNode> triangle_nodes_;  // every shell's tree, one after another
    std::vector<uint32_t> triangle_items_;             // triangle indices, as the leaves order them
    std::vector<uint32_t> shell_root_;                 // each shell's root in triangle_nodes_
};

}  // namespace spade
