#pragma once

// ---------------------------------------------------------------------------
// ray_scene -- the CPU ray scene: a ray against a RenderScene (camera sensor design, sections 4 and 9; Phase 1, P1.2). The CPU reference's geometry, the oracle the
// GPU's acceleration structures are banded against. It has no camera type in it (the LiDAR is a second ray generator over the same scene; open item 8, L111).
//
// WHAT A RAY MEETS. Three kinds of geometry share one nearest-hit answer, as the design's table says:
//   * TRIANGLES of the scene's meshes: every draw item of RenderScene::statics, then RenderScene::dynamics, each with its own local_to_world, baked to world space
//     at build and held in a binary BVH built here. NOT physics' TriangleMesh BVH (engine/world/mesh.hpp): that tree is private derived data inside a physics mesh,
//     built for closest-point queries over collision geometry; the camera sees render MeshData (visual meshes, submeshes with materials, per-vertex normals) in
//     the RenderScene, so it needs its own tree over those, built once per scene and traversed by a ray. It is rebuilt when the dynamics move (a frame's scene).
//   * the scene's analytic GROUND PLANES (RenderScene::ground_planes), closed form: solid is dot(p, n) <= offset, so a plane is hit from the open side only. The
//     finite tessellated item scene_from_world() also makes for a standalone plane is left out of the BVH (GroundPlane::item): one ground, not two.
//   * a CSG SUBTREE (RenderScene::csg_subtrees), MARCHED exactly: render/raymarch's sphere_trace over the subtree's own program, clipped to its bounds, with
//     kRaymarchSurfaceEpsilon. The subtree's tessellated mesh is not in the BVH. (A union leaf is still its tessellated mesh here; marching each leaf exactly needs a
//     per-leaf program the scene does not keep, and is Phase 2's with the GPU.)
//
// CONVENTIONS. A ray is double precision with a UNIT direction. t is the distance along it, measured from the origin; a hit needs t_min < t <= t_max. The nearest
// wins; two hits at the same t go to the lower triangle id (the build's own deterministic order), so traversal order never changes an answer. The BVH is built
// with a total order (sort on centroid, then id) so the same scene gives the same tree on every platform. No libm (SR-14): + - * / and sqrt only. A RayScene copies
// what it needs; the RenderScene it was built from can change or go away.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace spade::render {

enum class RayHitKind : uint8_t { none, triangle, sdf, ground };

struct Ray {
    glm::dvec3 origin{0.0};
    glm::dvec3 direction{0.0, 0.0, 1.0};  // unit
    double t_min = 0.0;                   // a hit needs t > t_min
    double t_max = std::numeric_limits<double>::infinity();  // and t <= t_max
};

struct RayHit {
    RayHitKind kind = RayHitKind::none;
    double t = 0.0;                 // distance along the ray
    glm::dvec3 point{0.0};          // origin + direction * t
    glm::vec3 normal{0.0f};         // world, unit, facing against the ray
    bool front_face = false;        // the surface's own normal faced the ray (false: the ray met the back)
    uint32_t item = kNoMesh;        // the draw item: statics in order, then dynamics (statics.size() + i); a CSG hit names the subtree's item; the ground: kNoMesh
    uint32_t triangle = kNoMesh;    // the triangle's index within its mesh (index buffer offset / 3); kNoMesh unless kind is triangle
    uint32_t material = 0;          // index into RenderScene::materials, resolved as the raster resolves it
    uint32_t label = kNoLabel;      // the item's object_label, or the ground plane's label: what the instance-id channel reads
    uint16_t sdf_label = 0;         // kind sdf only: the host label of the LEAF that owns the surface at the hit (sdf_hit_label, schema v4 node_labels), 0 when the nodes carry none
};

class RayScene {
public:
    RayScene() = default;

    // Builds the world-space triangles, the BVH, and copies the ground planes and CSG subtrees. Refuses a CSG subtree whose draw item is not a static.
    [[nodiscard]] static Result<RayScene> build(const RenderScene& scene);

    [[nodiscard]] RayHit trace(const Ray& ray) const;

    [[nodiscard]] std::size_t triangle_count() const { return triangles_.size(); }
    [[nodiscard]] std::size_t node_count() const { return nodes_.size(); }

private:
    struct Triangle {
        glm::dvec3 a{0.0}, e1{0.0}, e2{0.0};          // world space: vertex a and the edges b - a, c - a
        glm::vec3 n0{0.0f}, n1{0.0f}, n2{0.0f};       // world unit vertex normals (the geometric normal when the mesh has none)
        uint32_t item = 0, index = 0, material = 0;
        uint32_t id = 0;                              // the build's own order (statics, dynamics, triangles in mesh order): the tie-break
    };
    struct Node {
        glm::dvec3 lo{0.0}, hi{0.0};
        uint32_t first = 0;  // a leaf: the first triangle; an inner node: the left child's index (the right child is `right`)
        uint32_t count = 0;  // a leaf: the triangle count (above 0); an inner node: 0
        uint32_t right = 0;
    };
    struct Subtree {
        CsgSubtree csg;
        uint32_t material = 0;
        uint32_t label = kNoLabel;
    };
    struct Ground {
        glm::dvec3 normal{0.0, 1.0, 0.0};
        double offset = 0.0;
        uint32_t material = 0;
        uint32_t label = kNoLabel;
    };

    std::vector<Triangle> triangles_;  // in BVH leaf order
    std::vector<Node> nodes_;
    std::vector<Subtree> subtrees_;
    std::vector<Ground> grounds_;
    std::vector<uint32_t> labels_;     // each draw item's object_label, statics then dynamics
};

}  // namespace spade::render
