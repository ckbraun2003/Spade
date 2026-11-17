// The geo terrain's mesh and its ground rule (render/terrain.hpp, SP-4; docs/design/rendering/plans/2026-11-17-geo-terrain-design.md).
//
//   TerrainMesh    a decoded height grid, in metres from the anchor, becomes one mesh: a vertex lands at (east, height, -north),
//                  the edge takes its nearest sample, every triangle faces up, normals follow the slope, a coarse grid is cut to
//                  max_cells, the stats name the gap to the source, and the same input builds the same bytes.
//   TerrainRefused a map that cannot be drawn is refused by name and leaves the scene exactly as it was (a missing map is plain
//                  ground, REQ-G14); a side one source cell over 1 000 m passes, a longer one does not (REQ-G07).
//   TerrainGround  the render copy of each floor plane goes just under the map's lowest vertex (items and records), a plane
//                  already lower, or not a floor, is left alone, ground_y and the physics ground are untouched (REQ-G16), the
//                  sun's static shadow map is rebuilt without the terrain (it casts none), and in a frame the ground never
//                  shows through the map.
//
// No map data is committed (REQ-G10): every grid here is made in code.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/shadow.hpp"
#include "render/target.hpp"
#include "render/terrain.hpp"
#include "world/terrain_input.hpp"

namespace {

using spade::world::TerrainInput;
using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::GroundPlane;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::PixelFormat;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::TerrainSpec;
using spade::render::TerrainStats;

constexpr float kTol = 1e-5f;

// A map over [e0, e1] x [n0, n1] metres, nx x ny samples at the cell centres, rows north to south, each the value of
// height(east, north) there. The image is a valid 2 x 2.
[[nodiscard]] TerrainInput make_map(uint32_t nx, uint32_t ny, double e0, double e1, double n0, double n1,
                                  const std::function<float(double, double)>& height) {
    TerrainInput map;
    map.nx = nx;
    map.ny = ny;
    map.extent = {e0, e1, n0, n1};
    map.heights.resize(static_cast<size_t>(nx) * ny);
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            const double east = e0 + (i + 0.5) * (e1 - e0) / nx;
            const double north = n1 - (j + 0.5) * (n1 - n0) / ny;
            map.heights[static_cast<size_t>(j) * nx + i] = height(east, north);
        }
    }
    map.image.width = 2;
    map.image.height = 2;
    map.image.rgb.assign(2u * 2u * 3u, 200u);
    return map;
}

// h = 0.25 east + 0.5 north: a plane, so the bilinear sample is exact inside the first and last sample centres.
[[nodiscard]] float slope(double east, double north) { return static_cast<float>(0.25 * east + 0.5 * north); }

[[nodiscard]] RenderScene empty_scene() {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.bounds = {glm::vec3(-60.0f, -5.0f, -60.0f), glm::vec3(60.0f, 5.0f, 60.0f)};
    return scene;
}

[[nodiscard]] const MeshData& terrain_mesh(const RenderScene& scene) { return scene.meshes.back(); }
[[nodiscard]] const DrawItem& terrain_item(const RenderScene& scene) { return scene.statics.back(); }

// A 100 m square floor at y = 0 facing up, drawn in material 1, as scene_from_world draws a plane node, with its
// GroundPlane record pointing at the item.
void add_floor(RenderScene& scene, float y, uint32_t material) {
    MeshData mesh;
    mesh.positions = {glm::vec3(-50.0f, 0.0f, -50.0f), glm::vec3(-50.0f, 0.0f, 50.0f), glm::vec3(50.0f, 0.0f, -50.0f),
                      glm::vec3(50.0f, 0.0f, 50.0f)};
    mesh.normals.assign(4, glm::vec3(0.0f, 1.0f, 0.0f));
    mesh.indices = {0, 1, 2, 2, 1, 3};
    const uint32_t mesh_index = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(std::move(mesh));
    const uint32_t label = spade::render::intern_label(scene, "ground");
    const uint32_t item = static_cast<uint32_t>(scene.statics.size());
    scene.statics.push_back(DrawItem{.mesh_index = mesh_index,
                                     .local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, y, 0.0f)),
                                     .material_override = material,
                                     .object_label = label});
    scene.ground_planes.push_back(GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f),
                                              .offset = y,
                                              .material = material,
                                              .label = label,
                                              .item = item});
}

// What a refusal must not touch.
struct Snapshot {
    size_t meshes, statics, materials, labels, planes;
    bool shadow;
    explicit Snapshot(const RenderScene& s)
        : meshes(s.meshes.size()), statics(s.statics.size()), materials(s.materials.size()), labels(s.labels.size()),
          planes(s.ground_planes.size()), shadow(s.static_shadow.has_value()) {}
    [[nodiscard]] bool matches(const RenderScene& s) const {
        return meshes == s.meshes.size() && statics == s.statics.size() && materials == s.materials.size() &&
               labels == s.labels.size() && planes == s.ground_planes.size() && shadow == s.static_shadow.has_value();
    }
};

void expect_refused(const TerrainInput& map, const TerrainSpec& spec, const std::string& what) {
    RenderScene scene = empty_scene();
    add_floor(scene, 0.0f, 0u);
    const Snapshot before(scene);
    const glm::mat4 floor_before = scene.statics[0].local_to_world;
    const float offset_before = scene.ground_planes[0].offset;
    const auto result = spade::render::attach_terrain(scene, map, spec);
    EXPECT_FALSE(result.has_value()) << what;
    if (!result.has_value()) {
        EXPECT_NE(result.error().context.find("terrain"), std::string::npos) << what << ": " << result.error().context;
    }
    EXPECT_TRUE(before.matches(scene)) << what << ": the scene changed";
    EXPECT_EQ(scene.statics[0].local_to_world, floor_before) << what;
    EXPECT_EQ(scene.ground_planes[0].offset, offset_before) << what;
}

constexpr uint32_t kW = 160;
constexpr uint32_t kH = 120;

struct Bgr {
    uint8_t b, g, r;
};

[[nodiscard]] std::vector<Bgr> render_frame(const RenderScene& scene, const Camera& camera) {
    std::vector<uint8_t> storage(static_cast<size_t>(kW) * kH * 4u, 0u);
    RenderTarget target{.pixels = std::span<uint8_t>(storage), .width = kW, .height = kH, .stride = kW * 4u,
                        .format = PixelFormat::bgrx8};
    RenderOptions options;
    options.overlays = false;
    if (auto drew = spade::render::render(scene, camera, options, target); !drew) {
        ADD_FAILURE() << "render() failed: " << drew.error().context;
        return {};
    }
    std::vector<Bgr> pixels(static_cast<size_t>(kW) * kH);
    for (size_t p = 0; p < pixels.size(); ++p) pixels[p] = Bgr{storage[p * 4u], storage[p * 4u + 1u], storage[p * 4u + 2u]};
    return pixels;
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
// TerrainMesh
// ---------------------------------------------------------------------------------------------------------------------

TEST(TerrainMesh, AVertexLandsAtEastUpAndMinusNorth) {
    RenderScene scene = empty_scene();
    const TerrainInput map = make_map(8, 4, -40.0, 40.0, -20.0, 20.0, slope);
    const auto stats = spade::render::attach_terrain(scene, map, TerrainSpec{});
    ASSERT_TRUE(stats.has_value()) << stats.error().context;
    EXPECT_EQ(stats->cells_x, 8u);
    EXPECT_EQ(stats->cells_z, 4u);
    ASSERT_EQ(scene.meshes.size(), 1u);
    const MeshData& mesh = terrain_mesh(scene);
    ASSERT_EQ(mesh.positions.size(), 9u * 5u);
    // Vertex (i = 3, j = 1): east = -40 + 3 x 10, north = 20 - 1 x 10, drawn at z = -north. Row j = 0 is the NORTH edge.
    const glm::vec3 at = mesh.positions[1u * 9u + 3u];
    EXPECT_NEAR(at.x, -10.0f, kTol);
    EXPECT_NEAR(at.z, -10.0f, kTol);
    EXPECT_NEAR(at.y, slope(-10.0, 10.0), kTol);  // 2.5: inside the first and last sample centres the plane is exact
    // The north-west corner is row 0, column 0: z = -20 (north 20), x = -40.
    EXPECT_NEAR(mesh.positions[0].x, -40.0f, kTol);
    EXPECT_NEAR(mesh.positions[0].z, -20.0f, kTol);
    EXPECT_EQ(terrain_item(scene).local_to_world, glm::mat4(1.0f));
    EXPECT_EQ(terrain_item(scene).mesh_index, 0u);
}

TEST(TerrainMesh, AnEdgeVertexTakesItsNearestSample) {
    RenderScene scene = empty_scene();
    const TerrainInput map = make_map(8, 4, -40.0, 40.0, -20.0, 20.0, slope);
    ASSERT_TRUE(spade::render::attach_terrain(scene, map, TerrainSpec{}).has_value());
    // The corner (east -40, north 20) lies half a cell outside the first sample (east -35, north 15): the height is that
    // sample's, not the plane's extension.
    EXPECT_NEAR(terrain_mesh(scene).positions[0].y, slope(-35.0, 15.0), kTol);
}

TEST(TerrainMesh, EveryTriangleFacesUpAndEveryNormalIsAUnitVectorTowardTheSky) {
    RenderScene scene = empty_scene();
    ASSERT_TRUE(spade::render::attach_terrain(scene, make_map(8, 4, -40.0, 40.0, -20.0, 20.0, slope), TerrainSpec{})
                    .has_value());
    const MeshData& mesh = terrain_mesh(scene);
    ASSERT_EQ(mesh.indices.size(), 2u * 8u * 4u * 3u);
    for (size_t t = 0; t < mesh.indices.size(); t += 3) {
        const glm::vec3& a = mesh.positions[mesh.indices[t]];
        const glm::vec3& b = mesh.positions[mesh.indices[t + 1]];
        const glm::vec3& c = mesh.positions[mesh.indices[t + 2]];
        EXPECT_GT(glm::cross(b - a, c - a).y, 0.0f) << "triangle " << t / 3;
    }
    ASSERT_EQ(mesh.normals.size(), mesh.positions.size());
    for (const glm::vec3& n : mesh.normals) {
        EXPECT_NEAR(glm::length(n), 1.0f, kTol);
        EXPECT_GT(n.y, 0.0f);
    }
}

TEST(TerrainMesh, AFlatMapHasStraightUpNormals) {
    RenderScene scene = empty_scene();
    ASSERT_TRUE(spade::render::attach_terrain(
                    scene, make_map(6, 6, -30.0, 30.0, -30.0, 30.0, [](double, double) { return 1.5f; }), TerrainSpec{})
                    .has_value());
    for (const glm::vec3& n : terrain_mesh(scene).normals) EXPECT_EQ(n, glm::vec3(0.0f, 1.0f, 0.0f));
    for (const glm::vec3& p : terrain_mesh(scene).positions) EXPECT_EQ(p.y, 1.5f);
}

TEST(TerrainMesh, NormalsFollowTheSlope) {
    RenderScene scene = empty_scene();
    ASSERT_TRUE(spade::render::attach_terrain(scene, make_map(8, 4, -40.0, 40.0, -20.0, 20.0, slope), TerrainSpec{})
                    .has_value());
    // h = 0.25 east + 0.5 north = 0.25 x - 0.5 z, so the normal is (-0.25, 1, +0.5) normalised, at an interior vertex.
    const glm::vec3 want = glm::normalize(glm::vec3(-0.25f, 1.0f, 0.5f));
    const glm::vec3 got = terrain_mesh(scene).normals[2u * 9u + 3u];
    EXPECT_NEAR(got.x, want.x, 1e-4f);
    EXPECT_NEAR(got.y, want.y, 1e-4f);
    EXPECT_NEAR(got.z, want.z, 1e-4f);
}

TEST(TerrainMesh, ACoarseMapIsCutToMaxCells) {
    RenderScene scene = empty_scene();
    TerrainSpec spec;
    spec.max_cells = 64;
    const auto stats = spade::render::attach_terrain(scene, make_map(300, 300, -40.0, 40.0, -40.0, 40.0, slope), spec);
    ASSERT_TRUE(stats.has_value()) << stats.error().context;
    EXPECT_EQ(stats->cells_x, 64u);
    EXPECT_EQ(stats->cells_z, 64u);
    EXPECT_EQ(stats->triangles, 2u * 64u * 64u);
    EXPECT_EQ(terrain_mesh(scene).positions.size(), 65u * 65u);
    // A map with fewer samples than the cap is used one to one.
    RenderScene small = empty_scene();
    const auto one = spade::render::attach_terrain(small, make_map(10, 7, -40.0, 40.0, -40.0, 40.0, slope), spec);
    ASSERT_TRUE(one.has_value());
    EXPECT_EQ(one->cells_x, 10u);
    EXPECT_EQ(one->cells_z, 7u);
}

TEST(TerrainMesh, TheStatsNameTheGapToTheSource) {
    TerrainSpec spec;
    spec.max_cells = 16;
    // A plane loses only what the edge clamp costs.
    RenderScene ramp = empty_scene();
    const auto flat = spade::render::attach_terrain(ramp, make_map(400, 400, -200.0, 200.0, -200.0, 200.0, slope), spec);
    ASSERT_TRUE(flat.has_value());
    EXPECT_LT(flat->max_gap_m, 0.5f);
    // A 10 m ripple, 1 m high, sampled every metre and meshed at 25 m cells: the mesh cannot follow it, and says so.
    RenderScene ripple = empty_scene();
    const auto rough = spade::render::attach_terrain(
        ripple, make_map(400, 400, -200.0, 200.0, -200.0, 200.0, [](double e, double) {
            return static_cast<float>(std::sin(e * 0.6283185307179586));
        }),
        spec);
    ASSERT_TRUE(rough.has_value());
    EXPECT_GT(rough->max_gap_m, 0.3f);
    EXPECT_LT(rough->max_gap_m, 1.5f);
    // The lowest vertex is reported too (the ground rule's input).
    float lowest = std::numeric_limits<float>::infinity();
    for (const glm::vec3& p : terrain_mesh(ripple).positions) lowest = std::min(lowest, p.y);
    EXPECT_EQ(rough->lowest_y_m, lowest);
}

TEST(TerrainMesh, TheSameMapBuildsTheSameBytesAndNamesTheLabelAndTheMaterial) {
    const TerrainInput map = make_map(33, 21, -50.0, 50.0, -30.0, 30.0, [](double e, double n) {
        return static_cast<float>(std::sin(e * 0.1) + 0.5 * std::cos(n * 0.07));
    });
    TerrainSpec spec;
    spec.base_color = glm::vec3(0.8f, 0.7f, 0.6f);
    RenderScene a = empty_scene(), b = empty_scene();
    ASSERT_TRUE(spade::render::attach_terrain(a, map, spec).has_value());
    ASSERT_TRUE(spade::render::attach_terrain(b, map, spec).has_value());
    EXPECT_EQ(terrain_mesh(a).positions, terrain_mesh(b).positions);
    EXPECT_EQ(terrain_mesh(a).normals, terrain_mesh(b).normals);
    EXPECT_EQ(terrain_mesh(a).indices, terrain_mesh(b).indices);
    const DrawItem& item = terrain_item(a);
    ASSERT_LT(item.object_label, a.labels.size());
    EXPECT_EQ(a.labels[item.object_label], "terrain");
    ASSERT_LT(item.material_override, a.materials.size());
    EXPECT_EQ(glm::vec3(a.materials[item.material_override].base_color), spec.base_color);
    EXPECT_EQ(a.materials[item.material_override].shading, 0u);  // lambert by default (L85b)
    TerrainSpec unlit = spec;
    unlit.lit = false;
    RenderScene c = empty_scene();
    ASSERT_TRUE(spade::render::attach_terrain(c, map, unlit).has_value());
    EXPECT_EQ(c.materials[terrain_item(c).material_override].shading, 1u);
    // It is the world's own bounds that stay: the map is not part of the world.
    EXPECT_EQ(a.bounds.min, empty_scene().bounds.min);
    EXPECT_EQ(a.bounds.max, empty_scene().bounds.max);
}

// ---------------------------------------------------------------------------------------------------------------------
// TerrainRefused
// ---------------------------------------------------------------------------------------------------------------------

TEST(TerrainRefused, ABadMapIsRefusedByNameAndTheSceneIsLeftAsItWas) {
    const auto good = [] { return make_map(8, 8, -40.0, 40.0, -40.0, 40.0, slope); };
    const TerrainSpec spec;
    {
        TerrainInput m = good();
        m.heights[10] = std::numeric_limits<float>::quiet_NaN();
        expect_refused(m, spec, "a NaN height");
    }
    {
        TerrainInput m = good();
        m.heights[3] = std::numeric_limits<float>::infinity();
        expect_refused(m, spec, "an infinite height");
    }
    {
        TerrainInput m = good();
        std::swap(m.extent.east_min, m.extent.east_max);
        expect_refused(m, spec, "a reversed east extent");
    }
    {
        TerrainInput m = good();
        m.extent.north_max = m.extent.north_min;
        expect_refused(m, spec, "an empty north extent");
    }
    {
        TerrainInput m = good();
        m.extent.east_max = std::numeric_limits<double>::quiet_NaN();
        expect_refused(m, spec, "a NaN extent");
    }
    {
        TerrainInput m = good();
        m.nx = 1;
        m.heights.resize(8);
        expect_refused(m, spec, "a grid under 2 x 2");
    }
    {
        TerrainInput m = good();
        m.heights.pop_back();
        expect_refused(m, spec, "a height array that is not nx x ny");
    }
    {
        TerrainInput m = good();
        m.image.rgb.pop_back();
        expect_refused(m, spec, "an image array that is not width x height x 3");
    }
    {
        TerrainInput m = good();
        m.image.width = 1;
        m.image.rgb.resize(1u * 2u * 3u);
        expect_refused(m, spec, "an image under 2 x 2");
    }
    {
        TerrainSpec zero;
        zero.max_cells = 0;
        expect_refused(good(), zero, "max_cells 0");
    }
    // A side more than one source cell over 1 000 m (REQ-G07): 13 samples across 1 300 m is 100 m cells.
    expect_refused(make_map(13, 13, -650.0, 650.0, -650.0, 650.0, slope), spec, "a 1 300 m side");
}

TEST(TerrainRefused, AFullSizeAreaPasses) {
    // Interface's 1 001-sample grid at 1 m cells covers 1 001 m: one cell over, which is allowed.
    RenderScene scene = empty_scene();
    const TerrainInput map = make_map(1001, 4, -500.5, 500.5, -2.0, 2.0, [](double, double) { return 0.0f; });
    const auto stats = spade::render::attach_terrain(scene, map, TerrainSpec{});
    EXPECT_TRUE(stats.has_value()) << (stats.has_value() ? "" : stats.error().context);
}

// ---------------------------------------------------------------------------------------------------------------------
// TerrainGround
// ---------------------------------------------------------------------------------------------------------------------

namespace {

// A bowl: -0.3 m at the middle, up to -0.2 m at the corners of a 40 m square (all under a floor at y = 0).
[[nodiscard]] float bowl(double e, double n) { return static_cast<float>(-0.3 + 0.1 * (e * e + n * n) / 800.0); }

}  // namespace

TEST(TerrainGround, TheRenderFloorIsLoweredJustUnderTheMapAndNothingElseMoves) {
    RenderScene scene = empty_scene();
    scene.ground_y = 0.0f;
    scene.has_ground = true;
    add_floor(scene, 0.0f, 0u);
    // A wall plane (normal +x) and a floor already far below the map: neither moves.
    scene.ground_planes.push_back(GroundPlane{.normal = glm::vec3(1.0f, 0.0f, 0.0f), .offset = 3.0f, .item = spade::render::kNoMesh});
    add_floor(scene, -5.0f, 0u);
    const glm::mat4 low_floor = scene.statics[1].local_to_world;
    const auto stats =
        spade::render::attach_terrain(scene, make_map(40, 40, -20.0, 20.0, -20.0, 20.0, bowl), TerrainSpec{});
    ASSERT_TRUE(stats.has_value()) << stats.error().context;
    const float lowest = stats->lowest_y_m;
    EXPECT_LT(lowest, -0.29f);
    const float want_y = lowest - 0.02f;
    // The record and the item of the floor at y = 0.
    EXPECT_NEAR(scene.ground_planes[0].offset, want_y, kTol);
    EXPECT_NEAR(scene.statics[0].local_to_world[3].y, want_y, kTol);
    EXPECT_NEAR(stats->ground_lowered_m, -want_y, kTol);
    EXPECT_EQ(stats->planes_lowered, 1u);
    // The wall and the floor at -5 are as they were.
    EXPECT_EQ(scene.ground_planes[1].offset, 3.0f);
    EXPECT_EQ(scene.ground_planes[2].offset, -5.0f);
    EXPECT_EQ(scene.statics[1].local_to_world, low_floor);
    // The physics ground's mirror in the scene is untouched (REQ-G16).
    EXPECT_EQ(scene.ground_y, 0.0f);
}

TEST(TerrainGround, AMapAboveTheFloorMovesNothing) {
    RenderScene scene = empty_scene();
    add_floor(scene, -5.0f, 0u);
    const glm::mat4 before = scene.statics[0].local_to_world;
    const auto stats = spade::render::attach_terrain(
        scene, make_map(8, 8, -40.0, 40.0, -40.0, 40.0, [](double, double) { return 1.0f; }), TerrainSpec{});
    ASSERT_TRUE(stats.has_value());
    EXPECT_EQ(stats->planes_lowered, 0u);
    EXPECT_EQ(stats->ground_lowered_m, 0.0f);
    EXPECT_EQ(scene.statics[0].local_to_world, before);
    EXPECT_EQ(scene.ground_planes[0].offset, -5.0f);
}

TEST(TerrainGround, TheTerrainCastsNoShadowAndTheStaticMapIsRebuiltForTheLoweredFloor) {
    RenderScene scene = empty_scene();
    add_floor(scene, 0.0f, 0u);
    scene.static_shadow = spade::render::build_static_shadow_map(scene).value();
    const std::vector<float> stale = scene.static_shadow->depth;
    const auto stats =
        spade::render::attach_terrain(scene, make_map(40, 40, -20.0, 20.0, -20.0, 20.0, bowl), TerrainSpec{});
    ASSERT_TRUE(stats.has_value());
    EXPECT_FALSE(terrain_item(scene).casts_shadow);
    ASSERT_TRUE(scene.static_shadow.has_value());
    // The map is what a fresh build gives (the floor lowered, the terrain left out) and is not the stale one.
    const auto fresh = spade::render::build_static_shadow_map(scene);
    ASSERT_TRUE(fresh.has_value());
    EXPECT_EQ(scene.static_shadow->depth, fresh->depth);
    EXPECT_NE(scene.static_shadow->depth, stale);
    // A scene whose only static is the terrain has nothing in its map.
    RenderScene alone = empty_scene();
    ASSERT_TRUE(spade::render::attach_terrain(alone, make_map(8, 8, -20.0, 20.0, -20.0, 20.0, bowl), TerrainSpec{})
                    .has_value());
    const auto only = spade::render::build_static_shadow_map(alone);
    ASSERT_TRUE(only.has_value());
    for (const float d : only->depth) ASSERT_EQ(d, spade::render::kNoOccluder);
}

TEST(TerrainGround, InAFrameTheFloorNeverShowsThroughTheMap) {
    RenderScene scene = empty_scene();
    scene.materials.push_back(Material{glm::vec4(0.1f, 0.1f, 0.9f, 1.0f), 0u});  // the floor: blue
    add_floor(scene, 0.0f, 1u);
    TerrainSpec spec;
    spec.base_color = glm::vec3(0.9f, 0.1f, 0.1f);  // the map: red
    // Every height is under the floor, so a floor left at y = 0 would hide the whole map.
    ASSERT_TRUE(spade::render::attach_terrain(scene, make_map(40, 40, -20.0, 20.0, -20.0, 20.0, bowl), spec).has_value());
    Camera camera;
    camera.position = glm::vec3(0.0f, 15.0f, 0.0f);
    camera.orientation = glm::angleAxis(-1.5707963267948966f, glm::vec3(1.0f, 0.0f, 0.0f));  // straight down
    const std::vector<Bgr> frame = render_frame(scene, camera);
    ASSERT_EQ(frame.size(), static_cast<size_t>(kW) * kH);
    size_t red = 0, blue = 0;
    for (const Bgr& p : frame) {
        if (p.r > p.b + 40) ++red;
        if (p.b > p.r + 40) ++blue;
    }
    EXPECT_EQ(blue, 0u) << "the floor shows through the map";
    EXPECT_GT(red, frame.size() * 9 / 10) << "the map should fill the frame (winding, culling)";
}
