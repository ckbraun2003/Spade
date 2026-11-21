// test_ray_scene.cpp -- the CPU ray scene: a ray against a RenderScene (camera sensor Phase 1, P1.2; design sections 4 and 9).
//
// WHAT IS PINNED. The camera's CPU reference casts a ray into the render scene and gets back the nearest hit: its distance, point, a world normal facing against
// the ray, which draw item and triangle it was, the material and the object label the shading and the instance-id channel need. Three kinds of geometry share one
// answer: triangles of the scene's meshes (statics and dynamics, each item's own transform, a binary BVH built render-side over the world-space triangles), the
// scene's analytic ground planes (closed form, hit only from the open side), and a CSG subtree, which is MARCHED exactly (render/raymarch's sphere_trace inside the
// subtree's bounds) and never read from its tessellated mesh. A ray is double precision and its direction a unit vector. The BVH is checked against a brute-force scan
// of the same triangles written here independently (the same Moller-Trumbore, no tree), on a seeded soup; THIS FILE uses libm on purpose, as an independent check.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/ray_scene.hpp"
#include "render/raymarch.hpp"
#include "render/scene.hpp"
#include "world/builder.hpp"

namespace rd = spade::render;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// A unit-half-width square in the z = 0 plane, normals +z: two triangles, one submesh on the mesh's own material 0.
[[nodiscard]] rd::MeshData square_mesh() {
    rd::MeshData m;
    m.positions = {{-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {-1.0f, 1.0f, 0.0f}};
    m.normals = {{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}};
    m.indices = {0, 1, 2, 0, 2, 3};
    m.submesh_first_index = {0};
    m.submesh_index_count = {6};
    m.submesh_material = {0};
    return m;
}

[[nodiscard]] glm::mat4 at(const glm::vec3& p) { return glm::translate(glm::mat4(1.0f), p); }

[[nodiscard]] rd::RenderScene two_materials() {
    rd::RenderScene s;
    s.materials.resize(2);
    s.materials[1].base_color = glm::vec4(0.9f, 0.1f, 0.1f, 1.0f);
    return s;
}

[[nodiscard]] rd::Ray ray(const glm::dvec3& o, const glm::dvec3& d) {
    rd::Ray r;
    r.origin = o;
    r.direction = glm::normalize(d);
    return r;
}

[[nodiscard]] rd::RayScene built(const rd::RenderScene& scene) {
    spade::Result<rd::RayScene> rs = rd::RayScene::build(scene);
    EXPECT_TRUE(rs.has_value()) << (rs ? "" : rs.error().context);
    return rs ? std::move(*rs) : rd::RayScene{};
}

}  // namespace

// Two instances of one mesh, a static and a dynamic: the nearer wins from either side, with its own transform, label and material; the normal faces against
// the ray whichever side it came from, and front_face says which side that was.
TEST(RayScene, TheNearestInstanceWinsWithItsOwnLabelMaterialAndAFacingNormal) {
    rd::RenderScene scene = two_materials();
    scene.meshes.push_back(square_mesh());
    rd::DrawItem a;  // static, at z = 0, the mesh's own material, no label
    a.mesh_index = 0;
    a.local_to_world = at({0.0f, 0.0f, 0.0f});
    rd::DrawItem b;  // dynamic, at z = -3, material override 1, label 7
    b.mesh_index = 0;
    b.local_to_world = at({0.0f, 0.0f, -3.0f});
    b.material_override = 1;
    b.object_label = 7;
    scene.statics.push_back(a);
    scene.dynamics.push_back(b);
    const rd::RayScene rs = built(scene);
    EXPECT_EQ(rs.triangle_count(), 4u);

    const rd::RayHit from_above = rs.trace(ray({0.25, 0.0, 5.0}, {0.0, 0.0, -1.0}));
    ASSERT_EQ(from_above.kind, rd::RayHitKind::triangle);
    EXPECT_NEAR(from_above.t, 5.0, 1e-12);
    EXPECT_NEAR(from_above.point.z, 0.0, 1e-12);
    EXPECT_EQ(from_above.item, 0u) << "the static, the first of statics then dynamics";
    EXPECT_EQ(from_above.material, 0u) << "the submesh's own material";
    EXPECT_EQ(from_above.label, rd::kNoLabel);
    EXPECT_TRUE(from_above.front_face);
    EXPECT_NEAR(from_above.normal.z, 1.0f, 1e-6f);

    const rd::RayHit from_below = rs.trace(ray({0.0, 0.25, -10.0}, {0.0, 0.0, 1.0}));
    ASSERT_EQ(from_below.kind, rd::RayHitKind::triangle);
    EXPECT_NEAR(from_below.t, 7.0, 1e-12);
    EXPECT_EQ(from_below.item, 1u) << "the dynamic is item statics.size() + 0";
    EXPECT_EQ(from_below.material, 1u) << "the item's override replaces the submesh's";
    EXPECT_EQ(from_below.label, 7u);
    EXPECT_FALSE(from_below.front_face) << "the ray met the back of the quad";
    EXPECT_NEAR(from_below.normal.z, -1.0f, 1e-6f) << "the normal faces against the ray";

    EXPECT_EQ(rs.trace(ray({3.0, 0.0, 5.0}, {0.0, 0.0, -1.0})).kind, rd::RayHitKind::none) << "beside the quads";
    EXPECT_EQ(rs.trace(ray({0.0, 0.0, 5.0}, {0.0, 0.0, 1.0})).kind, rd::RayHitKind::none) << "pointing away";
    rd::Ray capped = ray({0.0, 0.0, 5.0}, {0.0, 0.0, -1.0});
    capped.t_max = 4.0;
    EXPECT_EQ(rs.trace(capped).kind, rd::RayHitKind::none) << "t_max cuts the ray before the quad";
}

// The BVH against a brute-force scan of the same triangles: a seeded soup of 600 triangles in two posed items, 4000 rays. Every ray's nearest hit and its
// triangle agree; the counts of hits and misses are non-trivial so the check cannot pass by both being empty.
TEST(RayScene, TheBvhAgreesWithABruteForceScanOfTheSameTriangles) {
    uint64_t state = 0x9E3779B97F4A7C15ull;
    auto next = [&state]() {  // a fixed integer generator: the test is a pure function of its seed
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<double>(state >> 11) / static_cast<double>(1ull << 53);
    };
    auto between = [&next](double lo, double hi) { return lo + (hi - lo) * next(); };

    rd::RenderScene scene = two_materials();
    for (int item = 0; item < 2; ++item) {
        rd::MeshData m;
        for (int i = 0; i < 300; ++i) {
            const glm::vec3 c(static_cast<float>(between(-4, 4)), static_cast<float>(between(-4, 4)), static_cast<float>(between(-4, 4)));
            for (int k = 0; k < 3; ++k) {
                m.positions.push_back(c + glm::vec3(static_cast<float>(between(-0.6, 0.6)), static_cast<float>(between(-0.6, 0.6)),
                                                    static_cast<float>(between(-0.6, 0.6))));
                m.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
                m.indices.push_back(static_cast<uint32_t>(m.positions.size() - 1u));
            }
        }
        m.submesh_first_index = {0};
        m.submesh_index_count = {static_cast<uint32_t>(m.indices.size())};
        m.submesh_material = {0};
        scene.meshes.push_back(std::move(m));
        rd::DrawItem d;
        d.mesh_index = static_cast<uint32_t>(item);
        d.local_to_world = at({static_cast<float>(item) * 2.0f, 0.0f, 0.0f}) * glm::mat4_cast(glm::angleAxis(0.7f * static_cast<float>(item), glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f))));
        scene.statics.push_back(d);
    }
    const rd::RayScene rs = built(scene);
    ASSERT_EQ(rs.triangle_count(), 600u);

    // the oracle: every triangle in world space, Moller-Trumbore in double, nearest wins
    struct Tri { glm::dvec3 a, b, c; uint32_t item, index; };
    std::vector<Tri> tris;
    for (uint32_t item = 0; item < 2; ++item) {
        const glm::mat4& l2w = scene.statics[item].local_to_world;
        const rd::MeshData& m = scene.meshes[item];
        for (uint32_t t = 0; t + 2 < m.indices.size(); t += 3) {
            auto w = [&](uint32_t v) { return glm::dvec3(glm::dmat4(l2w) * glm::dvec4(glm::dvec3(m.positions[m.indices[v]]), 1.0)); };  // the transform in double, as the ray scene bakes it
            tris.push_back({w(t), w(t + 1), w(t + 2), item, t / 3u});
        }
    }
    auto brute = [&](const rd::Ray& r, double& best_t, uint32_t& best_item, uint32_t& best_tri) {
        best_t = kInf;
        for (const Tri& tr : tris) {
            const glm::dvec3 e1 = tr.b - tr.a, e2 = tr.c - tr.a;
            const glm::dvec3 p = glm::cross(r.direction, e2);
            const double det = glm::dot(e1, p);
            if (std::abs(det) < 1e-14) continue;
            const glm::dvec3 s = r.origin - tr.a;
            const double u = glm::dot(s, p) / det;
            if (u < 0.0 || u > 1.0) continue;
            const glm::dvec3 q = glm::cross(s, e1);
            const double v = glm::dot(r.direction, q) / det;
            if (v < 0.0 || u + v > 1.0) continue;
            const double t = glm::dot(e2, q) / det;
            if (t > 0.0 && t < best_t) {
                best_t = t;
                best_item = tr.item;
                best_tri = tr.index;
            }
        }
    };
    int hits = 0, misses = 0, disagree = 0;
    for (int i = 0; i < 4000; ++i) {
        const rd::Ray r = ray({between(-6, 6), between(-6, 6), between(-6, 6)}, {between(-1, 1), between(-1, 1), between(-1, 1)});
        double bt = kInf;
        uint32_t bi = 0, btri = 0;
        brute(r, bt, bi, btri);
        const rd::RayHit h = rs.trace(r);
        if (bt == kInf) {
            ++misses;
            if (h.kind != rd::RayHitKind::none) ++disagree;
        } else {
            ++hits;
            if (h.kind != rd::RayHitKind::triangle || std::abs(h.t - bt) > 1e-9 || h.item != bi || h.triangle != btri) ++disagree;
        }
    }
    std::printf("[ ray scene ] BVH against brute force: %d hits, %d misses, %d disagreements; %zu BVH nodes for %zu triangles\n", hits, misses, disagree,
                rs.node_count(), rs.triangle_count());
    EXPECT_EQ(disagree, 0);
    EXPECT_GT(hits, 300);
    EXPECT_GT(misses, 300);
}

// The ground is closed form: a plane with solid at dot(p, n) <= offset, hit from the open side only, with the plane's label; the nearer of the ground and a mesh
// wins either way round.
TEST(RayScene, TheGroundIsAClosedFormPlaneHitFromTheOpenSideAndTheNearerSurfaceWins) {
    rd::RenderScene scene = two_materials();
    scene.meshes.push_back(square_mesh());
    rd::DrawItem q;  // a quad facing up, at height 1 (rotated -90 degrees about x: its +z normal becomes +y)
    q.mesh_index = 0;
    q.local_to_world = at({0.0f, 1.0f, 0.0f}) * glm::mat4_cast(glm::angleAxis(-1.5707963f, glm::vec3(1.0f, 0.0f, 0.0f)));
    scene.statics.push_back(q);
    rd::GroundPlane g;
    g.normal = {0.0f, 1.0f, 0.0f};
    g.offset = 0.0f;
    g.material = 1;
    g.label = 4;
    scene.ground_planes.push_back(g);
    scene.has_ground = true;
    const rd::RayScene rs = built(scene);

    const rd::RayHit ground = rs.trace(ray({5.0, 3.0, 0.0}, {0.0, -1.0, 0.0}));
    ASSERT_EQ(ground.kind, rd::RayHitKind::ground);
    EXPECT_NEAR(ground.t, 3.0, 1e-12);
    EXPECT_NEAR(ground.point.y, 0.0, 1e-12);
    EXPECT_NEAR(ground.normal.y, 1.0f, 1e-6f);
    EXPECT_EQ(ground.material, 1u);
    EXPECT_EQ(ground.label, 4u);
    EXPECT_EQ(ground.item, rd::kNoMesh) << "the ground is no draw item";

    const rd::RayHit over_quad = rs.trace(ray({0.0, 3.0, 0.0}, {0.0, -1.0, 0.0}));
    ASSERT_EQ(over_quad.kind, rd::RayHitKind::triangle) << "the quad at height 1 is nearer than the ground under it";
    EXPECT_NEAR(over_quad.t, 2.0, 1e-9);

    // a slanted ray reaches the ground at the closed-form distance
    const glm::dvec3 d = glm::normalize(glm::dvec3(1.0, -1.0, 0.5));
    const rd::RayHit slanted = rs.trace(ray({20.0, 4.0, 0.0}, d));
    ASSERT_EQ(slanted.kind, rd::RayHitKind::ground);
    EXPECT_NEAR(slanted.t, 4.0 / -d.y, 1e-9);

    EXPECT_EQ(rs.trace(ray({5.0, 3.0, 0.0}, {0.0, 1.0, 0.0})).kind, rd::RayHitKind::none) << "away from the ground";
    EXPECT_EQ(rs.trace(ray({5.0, -3.0, 0.0}, {0.0, 1.0, 0.0})).kind, rd::RayHitKind::none) << "from inside the solid side: not hit";
    EXPECT_EQ(rs.trace(ray({5.0, 3.0, 0.0}, {1.0, 0.0, 0.0})).kind, rd::RayHitKind::none) << "parallel to the ground";
}

// A CSG subtree is marched, not read from its tessellated mesh: a hollow ball (a radius 1 sphere minus a small box at its centre), hit on the ball's outer
// surface at the closed-form distance within the march's own epsilon, a distance a 5 cm tessellation of a sphere cannot give; a ray down the cavity's axis...
// is not one (the cavity is closed); a ray that misses the ball misses.
TEST(RayScene, ACsgSubtreeIsMarchedExactlyAndNeverReadFromItsMesh) {
    spade::WorldBuilder b;
    b.name("ray-scene-test").capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    b.sphere(1.0f).box(glm::vec3(0.2f, 0.2f, 0.2f)).subtract();
    const spade::Result<spade::WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << (world ? "" : world.error().context);
    spade::Result<rd::RenderScene> scene = rd::scene_from_world(*world, {});
    ASSERT_TRUE(scene.has_value()) << (scene ? "" : scene.error().context);
    ASSERT_EQ(scene->csg_subtrees.size(), 1u);
    const rd::RayScene rs = built(*scene);
    EXPECT_EQ(rs.triangle_count(), 0u) << "the subtree's own mesh is not in the BVH: it is marched";

    // from +x at y = 0.5, z = 0.4: the ball's surface is at x = sqrt(1 - 0.25 - 0.16)
    const double exact_x = std::sqrt(1.0 - 0.5 * 0.5 - 0.4 * 0.4);
    const rd::RayHit h = rs.trace(ray({5.0, 0.5, 0.4}, {-1.0, 0.0, 0.0}));
    ASSERT_EQ(h.kind, rd::RayHitKind::sdf);
    std::printf("[ ray scene ] CSG ball: marched t %.6f against the closed form %.6f\n", h.t, 5.0 - exact_x);
    EXPECT_NEAR(h.t, 5.0 - exact_x, 2.0 * rd::kRaymarchSurfaceEpsilon);
    EXPECT_NEAR(h.point.x, exact_x, 2.0 * rd::kRaymarchSurfaceEpsilon);
    const glm::dvec3 n = glm::normalize(glm::dvec3(exact_x, 0.5, 0.4));
    EXPECT_NEAR(h.normal.x, static_cast<float>(n.x), 1e-3f);
    EXPECT_NEAR(h.normal.y, static_cast<float>(n.y), 1e-3f);
    EXPECT_NEAR(h.normal.z, static_cast<float>(n.z), 1e-3f);
    EXPECT_EQ(h.item, scene->csg_subtrees[0].draw_item) << "the hit names the subtree's draw item";

    EXPECT_EQ(rs.trace(ray({5.0, 2.0, 0.0}, {-1.0, 0.0, 0.0})).kind, rd::RayHitKind::none) << "a ray that misses the ball";
    EXPECT_EQ(rs.trace(ray({5.0, 0.0, 0.0}, {1.0, 0.0, 0.0})).kind, rd::RayHitKind::none) << "pointing away";
}

// A CSG hit reads the object label of the LEAF that owns the surface at the hit (world/sdf.hpp's sdf_hit_label, node_labels in lockstep with the nodes), so two objects
// touching in one subtree each read their own label on their own side of the seam: a radius-1 ball (label 7) with a box (label 8) cut into its +x side from x = 0.4. A ray
// down the notch meets the cut face, the box's surface in the result (8); a ray above the notch meets the ball (7). A subtree whose nodes carry no labels reads 0.
TEST(RayScene, ACsgHitReadsTheLabelOfTheLeafThatOwnsTheSurface) {
    spade::WorldBuilder b;
    b.name("ray-scene-label-test").capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    b.sphere(1.0f).label_for_last_node(7);
    b.box(glm::vec3(0.5f, 0.3f, 0.3f), spade::SdfPose{.position = glm::vec3(0.9f, 0.0f, 0.0f)}).label_for_last_node(8);
    b.subtract();
    const spade::Result<spade::WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << (world ? "" : world.error().context);
    spade::Result<rd::RenderScene> scene = rd::scene_from_world(*world, {});
    ASSERT_TRUE(scene.has_value()) << (scene ? "" : scene.error().context);
    ASSERT_EQ(scene->csg_subtrees.size(), 1u);
    const rd::RayScene rs = built(*scene);

    const rd::RayHit notch = rs.trace(ray({5.0, 0.1, 0.1}, {-1.0, 0.0, 0.0}));
    ASSERT_EQ(notch.kind, rd::RayHitKind::sdf);
    EXPECT_NEAR(notch.point.x, 0.4, 2.0 * rd::kRaymarchSurfaceEpsilon) << "the cut face";
    EXPECT_EQ(notch.sdf_label, 8u) << "the cut face is the box's surface";
    const rd::RayHit ball = rs.trace(ray({5.0, 0.6, 0.0}, {-1.0, 0.0, 0.0}));
    ASSERT_EQ(ball.kind, rd::RayHitKind::sdf);
    EXPECT_EQ(ball.sdf_label, 7u) << "above the notch the surface is the ball's";

    // no labels on the nodes: 0
    spade::WorldBuilder plain;
    plain.name("ray-scene-nolabel-test").capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    plain.sphere(1.0f).box(glm::vec3(0.2f, 0.2f, 0.2f)).subtract();
    const spade::Result<spade::WorldDesc> unlabelled = plain.build();
    ASSERT_TRUE(unlabelled.has_value());
    spade::Result<rd::RenderScene> scene2 = rd::scene_from_world(*unlabelled, {});
    ASSERT_TRUE(scene2.has_value());
    const rd::RayHit h = built(*scene2).trace(ray({5.0, 0.5, 0.4}, {-1.0, 0.0, 0.0}));
    ASSERT_EQ(h.kind, rd::RayHitKind::sdf);
    EXPECT_EQ(h.sdf_label, 0u);
}

// The same scene and ray give the same bytes every time (a CPU reference is a pure function of its inputs), and a built RayScene does not hold the RenderScene:
// the scene can change after the build without moving a hit.
TEST(RayScene, ATraceIsAPureFunctionOfTheSceneAndTheRay) {
    rd::RenderScene scene = two_materials();
    scene.meshes.push_back(square_mesh());
    rd::DrawItem a;
    a.mesh_index = 0;
    a.local_to_world = at({0.0f, 0.0f, -1.0f});
    scene.statics.push_back(a);
    const rd::RayScene rs = built(scene);
    const rd::Ray r = ray({0.3, 0.2, 4.0}, {0.01, 0.02, -1.0});
    const rd::RayHit first = rs.trace(r);
    ASSERT_EQ(first.kind, rd::RayHitKind::triangle);
    scene.statics.clear();
    scene.meshes.clear();
    const rd::RayHit again = rs.trace(r);
    EXPECT_EQ(first.t, again.t);
    EXPECT_EQ(first.point, again.point);
    EXPECT_EQ(first.normal, again.normal);
    EXPECT_EQ(first.triangle, again.triangle);
}

// A standalone plane in a world is the analytic ground, and the finite tessellated grid scene_from_world() also makes for it is NOT in the BVH: one ground, closed
// form, so a ray far outside the grid's extent still reaches it, and no ray meets the plane twice.
TEST(RayScene, AStandalonePlaneIsTheClosedFormGroundAndItsTessellatedGridIsNotInTheBvh) {
    spade::WorldBuilder b;
    b.name("ray-scene-ground-test").capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    const spade::Result<spade::WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << (world ? "" : world.error().context);
    spade::Result<rd::RenderScene> scene = rd::scene_from_world(*world, {});
    ASSERT_TRUE(scene.has_value()) << (scene ? "" : scene.error().context);
    ASSERT_EQ(scene->ground_planes.size(), 1u);
    ASSERT_FALSE(scene->statics.empty()) << "the plane also has a tessellated item";
    const rd::RayScene rs = built(*scene);
    EXPECT_EQ(rs.triangle_count(), 0u) << "the grid is the ground's own item: it is not a second surface";
    const rd::RayHit far = rs.trace(ray({1.0e4, 5.0, -3.0e3}, {0.0, -1.0, 0.0}));
    ASSERT_EQ(far.kind, rd::RayHitKind::ground);
    EXPECT_NEAR(far.t, 5.0, 1e-9);
    EXPECT_NEAR(far.normal.y, 1.0f, 1e-6f);
}
