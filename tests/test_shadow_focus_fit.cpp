// ---------------------------------------------------------------------------
// test_shadow_focus_fit.cpp -- fit_focus_shadow_map(), the dynamic casters' focus map frame (L74b, Cameron's quick wins).
//
//   ShadowFocusFit  the fit that skips the items that cannot move the footprint is BYTE-IDENTICAL to the exhaustive fit
//                   (every vertex of every item, the original loop, kept as fit_focus_shadow_map_exhaustive): the
//                   matrix's every bit, the size, the bias, and whether there is a map at all, over random scenes of
//                   every shape the fit may meet (clusters, lines, a pile at one point, huge offsets, shears, bottom
//                   rows that are not (0, 0, 0, 1), see-through items, an out-of-range mesh, a NaN pose); and on the
//                   training view's grid of drones it transforms a small part of the vertices.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/scene.hpp"
#include "render/shadow.hpp"

namespace {

using spade::render::DrawItem;
using spade::render::FocusFitStats;
using spade::render::MeshData;
using spade::render::RenderScene;
using spade::render::ShadowMap;

// A lat-long sphere of `radius` about `centre`: positions and a triangle list, nothing else (the fit reads positions).
[[nodiscard]] MeshData sphere(float radius, glm::vec3 centre, uint32_t segments, uint32_t rings) {
    MeshData mesh;
    for (uint32_t r = 0; r <= rings; ++r) {
        const float phi = 3.14159265f * static_cast<float>(r) / static_cast<float>(rings);
        for (uint32_t s = 0; s <= segments; ++s) {
            const float theta = 6.28318531f * static_cast<float>(s) / static_cast<float>(segments);
            mesh.positions.push_back(centre + radius * glm::vec3(std::sin(phi) * std::cos(theta), std::cos(phi),
                                                                 std::sin(phi) * std::sin(theta)));
        }
    }
    for (uint32_t r = 0; r < rings; ++r) {
        for (uint32_t s = 0; s < segments; ++s) {
            const uint32_t a = r * (segments + 1u) + s, b = a + segments + 1u;
            for (const uint32_t i : {a, b, a + 1u, a + 1u, b, b + 1u}) mesh.indices.push_back(i);
        }
    }
    return mesh;
}

// A long thin lumpy mesh (a body that is far from a sphere): a box-ish strip with its vertices off a grid.
[[nodiscard]] MeshData strip(std::mt19937& rng) {
    std::uniform_real_distribution<float> d(-1.0f, 1.0f);
    MeshData mesh;
    for (uint32_t i = 0; i < 90; ++i) {
        mesh.positions.push_back(glm::vec3(0.9f * d(rng), 0.05f * d(rng), 0.12f * d(rng)));
    }
    for (uint32_t i = 0; i + 2 < 90; ++i) {
        for (const uint32_t k : {i, i + 1u, i + 2u}) mesh.indices.push_back(k);
    }
    return mesh;
}

// A coarse static map over a huge world: the focus map is finer than it unless the bodies are as spread as the world.
[[nodiscard]] ShadowMap coarse_statics(float half_world) {
    RenderScene world;
    world.bounds = spade::render::Aabb{.min = glm::vec3(-half_world), .max = glm::vec3(half_world)};
    world.lighting.sun_direction = glm::normalize(glm::vec3(0.4f, 0.8f, 0.6f));
    auto map = spade::render::build_static_shadow_map(world, 64);
    EXPECT_TRUE(map.has_value());
    return map ? std::move(*map) : ShadowMap{};
}

[[nodiscard]] bool same_bits(const std::optional<ShadowMap>& a, const std::optional<ShadowMap>& b) {
    if (a.has_value() != b.has_value()) return false;
    if (!a) return true;
    return a->size == b->size && std::memcmp(&a->depth_bias, &b->depth_bias, sizeof(float)) == 0 &&
           std::memcmp(&a->light_view_proj, &b->light_view_proj, sizeof(glm::mat4)) == 0;
}

// Both fits of `scene`; they must agree bit for bit. Returns the skip fit's stats.
[[nodiscard]] FocusFitStats expect_identical(const RenderScene& scene, const ShadowMap& statics, const std::string& what,
                                             float filter_texels = 1.0f) {
    FocusFitStats stats;
    const auto fast = spade::render::fit_focus_shadow_map(scene, 1024, filter_texels, statics, &stats);
    const auto full = spade::render::fit_focus_shadow_map_exhaustive(scene, 1024, filter_texels, statics);
    EXPECT_TRUE(same_bits(fast, full)) << what << ": the fits differ";
    EXPECT_LE(stats.vertices_transformed, stats.vertices_total) << what;
    return stats;
}

// The training view's scene: n drones a metre apart on a grid, each posed (position, yaw), one shared mesh.
[[nodiscard]] RenderScene puppet_grid(uint32_t n, const MeshData& drone, float t) {
    RenderScene scene;
    scene.meshes.push_back(drone);
    const uint32_t cols = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(n))));
    const float extent = std::max(static_cast<float>(cols), 4.0f);
    scene.lighting.sun_direction = glm::normalize(glm::vec3(0.4f, 0.8f, 0.6f));
    scene.dynamics.assign(n, DrawItem{.mesh_index = 0});
    for (uint32_t i = 0; i < n; ++i) {
        const glm::vec3 at(static_cast<float>(i % cols) - 0.5f * extent, 1.0f + 0.2f * std::sin(t + 0.3f * static_cast<float>(i)),
                           static_cast<float>(i / cols) - 0.5f * extent);
        scene.dynamics[i].local_to_world = glm::rotate(glm::translate(glm::mat4(1.0f), at),
                                                       0.5f * t + 0.1f * static_cast<float>(i), glm::vec3(0.0f, 1.0f, 0.0f));
    }
    return scene;
}

}  // namespace

TEST(ShadowFocusFit, OnATrainingViewGridOfDronesMostVerticesAreNeverTransformed) {
    const MeshData drone = sphere(0.18f, glm::vec3(0.0f), 48, 32);
    const ShadowMap statics = coarse_statics(1.0e4f);
    for (const uint32_t n : {128u, 256u, 1024u}) {
        const RenderScene scene = puppet_grid(n, drone, 0.7f);
        const FocusFitStats stats = expect_identical(scene, statics, "grid of " + std::to_string(n));
        EXPECT_EQ(stats.vertices_total, static_cast<uint64_t>(n) * drone.positions.size());
        // Only the drones that can set the footprint's edge (a strip of them along each side) are transformed.
        EXPECT_LT(static_cast<double>(stats.vertices_transformed), 0.45 * static_cast<double>(stats.vertices_total))
            << n << " drones";
    }
    const FocusFitStats big = expect_identical(puppet_grid(1024, drone, 0.7f), statics, "1024");
    EXPECT_LT(static_cast<double>(big.vertices_transformed), 0.15 * static_cast<double>(big.vertices_total));
}

TEST(ShadowFocusFit, ABodyThatCanMoveTheFootprintIsAlwaysTransformedWhole) {
    const MeshData drone = sphere(0.18f, glm::vec3(0.0f), 24, 16);
    const ShadowMap statics = coarse_statics(1.0e4f);
    // One drone: nothing to skip it for.
    const FocusFitStats one = expect_identical(puppet_grid(1, drone, 0.0f), statics, "one drone");
    EXPECT_EQ(one.vertices_transformed, one.vertices_total);
    // Every drone at one point: none lies strictly inside another's extent, so all are transformed.
    RenderScene pile = puppet_grid(64, drone, 0.0f);
    for (DrawItem& item : pile.dynamics) item.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(3.0f, 1.0f, -2.0f));
    const FocusFitStats pile_stats = expect_identical(pile, statics, "a pile at one point");
    EXPECT_EQ(pile_stats.vertices_transformed, pile_stats.vertices_total);
}

TEST(ShadowFocusFit, TheSkipFitIsByteIdenticalToTheExhaustiveFitOverRandomScenes) {
    std::mt19937 rng(0x51ADE5u);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f), signed_unit(-1.0f, 1.0f);
    const std::vector<MeshData> meshes = {sphere(0.2f, glm::vec3(0.0f), 16, 10), strip(rng),
                                          sphere(0.05f, glm::vec3(0.3f, -0.1f, 0.2f), 8, 6), MeshData{}};
    const ShadowMap statics = coarse_statics(1.0e6f);
    for (uint32_t scene_index = 0; scene_index < 400; ++scene_index) {
        RenderScene scene;
        scene.meshes = meshes;
        glm::vec3 sun(signed_unit(rng), 0.15f + 0.85f * unit(rng), signed_unit(rng));
        if (scene_index % 17 == 0) sun = glm::vec3(0.0f, 1.0f, 0.0f);  // straight up: the other basis branch
        scene.lighting.sun_direction = sun;
        const uint32_t n = 1u + static_cast<uint32_t>(unit(rng) * (scene_index % 5 == 0 ? 600.0f : 40.0f));
        const float spread = std::pow(10.0f, -2.0f + 5.0f * unit(rng));  // 1 cm to 1 km
        const float offset = scene_index % 11 == 0 ? 1.0e6f * signed_unit(rng) : 0.0f;
        const uint32_t clusters = 1u + scene_index % 4u;
        const bool line = scene_index % 7 == 0;
        for (uint32_t i = 0; i < n; ++i) {
            DrawItem item;
            item.mesh_index = static_cast<uint32_t>(unit(rng) * 3.999f);
            if (scene_index % 13 == 0 && i % 9 == 0) item.mesh_index = 99u;  // out of range
            if (scene_index % 3 == 0 && i % 5 == 0) item.see_through = true;
            const float cluster = static_cast<float>(i % clusters);
            glm::vec3 at = spread * glm::vec3(signed_unit(rng), line ? 0.0f : signed_unit(rng), line ? 0.0f : signed_unit(rng));
            at += glm::vec3(offset + 3.0f * spread * cluster, offset, offset);
            const glm::quat q = glm::normalize(glm::quat(signed_unit(rng), signed_unit(rng), signed_unit(rng), signed_unit(rng)) +
                                               glm::quat(0.0f, 0.0f, 0.0f, 1.0e-3f));
            glm::mat4 m = glm::translate(glm::mat4(1.0f), at) * glm::mat4_cast(q);
            if (scene_index % 4 == 1) m = m * glm::scale(glm::mat4(1.0f), glm::vec3(0.2f + 3.0f * unit(rng), 0.2f + unit(rng), 1.0f));
            if (scene_index % 6 == 2) {  // a shear and a bottom row that is not (0, 0, 0, 1): the fit ignores w
                m[1][0] += 0.3f * signed_unit(rng);
                m[0][3] = 0.2f * signed_unit(rng);
                m[3][3] = 0.5f + unit(rng);
            }
            item.local_to_world = m;
            scene.dynamics.push_back(item);
        }
        expect_identical(scene, statics, "random scene " + std::to_string(scene_index), scene_index % 3 == 0 ? 0.0f : 2.5f);
    }
}

TEST(ShadowFocusFit, ANonFinitePoseOrMeshTakesTheExhaustivePathAndAgreesBitForBit) {
    const MeshData drone = sphere(0.18f, glm::vec3(0.0f), 16, 10);
    const ShadowMap statics = coarse_statics(1.0e4f);
    RenderScene nan_pose = puppet_grid(50, drone, 0.3f);
    nan_pose.dynamics[17].local_to_world[3][1] = std::numeric_limits<float>::quiet_NaN();
    expect_identical(nan_pose, statics, "a NaN position");
    RenderScene inf_pose = puppet_grid(50, drone, 0.3f);
    inf_pose.dynamics[3].local_to_world[0][0] = std::numeric_limits<float>::infinity();
    expect_identical(inf_pose, statics, "an infinite scale");
    RenderScene nan_mesh = puppet_grid(50, drone, 0.3f);
    nan_mesh.meshes[0].positions[5].y = std::numeric_limits<float>::quiet_NaN();
    expect_identical(nan_mesh, statics, "a NaN vertex");
}

TEST(ShadowFocusFit, NothingToFitStillMeansNoMapAndTheSameAnswerAsBefore) {
    const MeshData drone = sphere(0.18f, glm::vec3(0.0f), 16, 10);
    const ShadowMap statics = coarse_statics(1.0e4f);
    RenderScene empty = puppet_grid(4, drone, 0.0f);
    empty.dynamics.clear();
    EXPECT_FALSE(spade::render::fit_focus_shadow_map(empty, 1024, 1.0f, statics).has_value());
    RenderScene all_see_through = puppet_grid(4, drone, 0.0f);
    for (DrawItem& item : all_see_through.dynamics) item.see_through = true;
    EXPECT_FALSE(spade::render::fit_focus_shadow_map(all_see_through, 1024, 1.0f, statics).has_value());
    RenderScene empty_mesh = puppet_grid(4, drone, 0.0f);
    empty_mesh.meshes[0].positions.clear();
    EXPECT_FALSE(spade::render::fit_focus_shadow_map(empty_mesh, 1024, 1.0f, statics).has_value());
    EXPECT_FALSE(spade::render::fit_focus_shadow_map(puppet_grid(4, drone, 0.0f), 0, 1.0f, statics).has_value());
    // A map no finer than the statics': the answer is "no map", skip fit or not.
    const ShadowMap fine = coarse_statics(0.5f);
    expect_identical(puppet_grid(4, drone, 0.0f), fine, "statics finer than the focus map");
}
