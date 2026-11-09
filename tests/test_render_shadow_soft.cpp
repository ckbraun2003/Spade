// ---------------------------------------------------------------------------
// test_render_shadow_soft.cpp -- rendering/soft-shadows (Cameron: "the shadow is
// pixelated", Kat's 170 m world: one 1024-texel map, ~17 cm stair steps under a
// 30 cm drone, sampled one texel with no filtering). The lead's rulings, 2026-11-09:
//
//   SoftShadow      a shadow edge across a texel boundary is a ramp at filter 1, a
//                   step at filter 0; the ramp never invents a shadow on a lit face
//                   (Kat's sawtooth fix, 2026-10-28, survives the filter);
//   FocusShadow     a small dynamic caster on a large world gets texels at or under
//                   2 mm and its shadow's edge lands within a centimetre; a body
//                   past scene.bounds casts (SR-17 lifted for dynamics, L42, which
//                   closes M6) while statics past it stay unshadowed; bodies so far
//                   apart that a map of their own would be no finer than the world's
//                   fall back to folding, never worse than today's folded result;
//   ShadowDefaults  an untouched RenderOptions is soft and tight, and render() is
//                   byte-identical across two calls.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/shadow.hpp"
#include "render/target.hpp"

namespace {

using spade::render::Aabb;
using spade::render::build_focus_shadow_map;
using spade::render::build_static_shadow_map;
using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::kNoMaterial;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::sample_shadow;
using spade::render::ShadowMap;
using spade::render::ShadowSet;
using spade::render::shadow_texel_metres;

[[nodiscard]] MeshData make_box_mesh(float half_extent) {
    MeshData mesh;
    const float h = half_extent;
    struct Face {
        glm::vec3 normal;
        glm::vec3 corners[4];
    };
    const Face faces[6] = {
        {{1.0f, 0.0f, 0.0f}, {{h, -h, -h}, {h, h, -h}, {h, h, h}, {h, -h, h}}},
        {{-1.0f, 0.0f, 0.0f}, {{-h, -h, h}, {-h, h, h}, {-h, h, -h}, {-h, -h, -h}}},
        {{0.0f, 1.0f, 0.0f}, {{-h, h, h}, {h, h, h}, {h, h, -h}, {-h, h, -h}}},
        {{0.0f, -1.0f, 0.0f}, {{-h, -h, -h}, {h, -h, -h}, {h, -h, h}, {-h, -h, h}}},
        {{0.0f, 0.0f, 1.0f}, {{h, -h, h}, {h, h, h}, {-h, h, h}, {-h, -h, h}}},
        {{0.0f, 0.0f, -1.0f}, {{-h, -h, -h}, {-h, h, -h}, {h, h, -h}, {h, -h, -h}}},
    };
    for (const Face& f : faces) {
        const uint32_t base = static_cast<uint32_t>(mesh.positions.size());
        for (const glm::vec3& c : f.corners) {
            mesh.positions.push_back(c);
            mesh.normals.push_back(f.normal);
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return mesh;
}

// A flat +Y quad at y = 0, 4 km square around the origin.
[[nodiscard]] MeshData make_ground_quad() {
    MeshData mesh;
    const glm::vec3 corners[4] = {{-2000.0f, 0.0f, 2000.0f}, {2000.0f, 0.0f, 2000.0f}, {2000.0f, 0.0f, -2000.0f},
                                  {-2000.0f, 0.0f, -2000.0f}};
    for (const glm::vec3& c : corners) {
        mesh.positions.push_back(c);
        mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
    }
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return mesh;
}

[[nodiscard]] glm::mat4 translation(glm::vec3 t) {
    glm::mat4 m(1.0f);
    m[3] = glm::vec4(t, 1.0f);
    return m;
}

// VQ1's world, as the existing sawtooth test builds it: ~172 m wide, a ground that casts, so ~15 cm texels.
[[nodiscard]] RenderScene big_world() {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.bounds = Aabb{.min = glm::vec3(-80.0f, -1.0f, -10.0f), .max = glm::vec3(92.0f, 30.0f, 10.0f)};
    scene.meshes.push_back(make_box_mesh(1.0f));   // 0: a unit-half box
    scene.meshes.push_back(make_ground_quad());    // 1: the ground
    scene.statics.push_back(DrawItem{.mesh_index = 1, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});
    return scene;
}

// Whether the ray from `q` toward the sun meets an axis-aligned box [lo, hi] grown by `grow` (shrunk when negative).
[[nodiscard]] bool ray_meets_box(const glm::vec3& sun, glm::vec3 q, glm::vec3 lo, glm::vec3 hi, float grow) {
    lo -= glm::vec3(grow);
    hi += glm::vec3(grow);
    float t_in = 0.0f, t_out = 1000.0f;
    for (int a = 0; a < 3; ++a) {
        float t0 = (lo[a] - q[a]) / sun[a], t1 = (hi[a] - q[a]) / sun[a];
        if (t0 > t1) std::swap(t0, t1);
        t_in = std::max(t_in, t0);
        t_out = std::min(t_out, t1);
    }
    return t_in < t_out;
}

// ---------------------------------------------------------------------------
// SoftShadow
// ---------------------------------------------------------------------------

// A 4 m box on the ground of a 172 m world: its shadow's edge across the ground, sampled every centimetre along a
// line through several 15 cm texels. One texel (filter 0) steps from lit to shadowed at a texel boundary; a filter of
// one texel makes that step a ramp, monotone, at least a texel wide, and fully lit and fully shadowed away from it.
TEST(SoftShadow, AnEdgeAcrossATexelBoundaryIsARampAtFilterOneAndAStepAtZero) {
    RenderScene scene = big_world();
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = translation(glm::vec3(0.0f, 1.0f, 0.0f)), .material_override = kNoMaterial});
    const auto map = build_static_shadow_map(scene);
    ASSERT_TRUE(map.has_value()) << map.error().context;
    const float texel = shadow_texel_metres(*map);
    ASSERT_GT(texel, 0.12f) << "the map must be as coarse as VQ1's";
    const glm::vec3 up(0.0f, 1.0f, 0.0f);

    // Along +x on the ground, a line through the shadow's middle in z, from 12 m one side to 12 m the other: lit,
    // then the shadow, then lit, so it crosses the shadow's two edges.
    const glm::vec3 sun = scene.lighting.sun_direction;
    const float z = -sun.z / sun.y;
    std::vector<float> hard, soft;
    for (int i = 0; i <= 2400; ++i) {
        const glm::vec3 q(-12.0f + 0.01f * static_cast<float>(i), 0.0f, z);
        hard.push_back(sample_shadow(*map, q, up, 0.0f));
        soft.push_back(sample_shadow(*map, q, up, 1.0f));
    }
    size_t hard_mid = 0, soft_mid = 0;
    float biggest_hard_step = 0.0f, biggest_soft_step = 0.0f;
    for (size_t i = 0; i < hard.size(); ++i) {
        if (hard[i] > 0.05f && hard[i] < 0.95f) ++hard_mid;
        if (soft[i] > 0.05f && soft[i] < 0.95f) ++soft_mid;
        if (i > 0) {
            biggest_hard_step = std::max(biggest_hard_step, std::abs(hard[i] - hard[i - 1]));
            biggest_soft_step = std::max(biggest_soft_step, std::abs(soft[i] - soft[i - 1]));
        }
    }
    EXPECT_EQ(hard_mid, 0u) << "filter 0 is a single texel: only 0 and 1";
    EXPECT_GE(biggest_hard_step, 0.99f) << "filter 0 steps from lit to shadowed in one centimetre";
    EXPECT_GE(soft_mid, static_cast<size_t>(0.6f * texel / 0.01f))
        << "filter 1 spreads the edge over about a texel of centimetre samples";
    EXPECT_LE(biggest_soft_step, 0.2f) << "no step in the ramp: a centimetre moves it by at most a fifth";
    EXPECT_FLOAT_EQ(*std::min_element(soft.begin(), soft.end()), 0.0f) << "deep in the shadow is fully shadowed";
    EXPECT_FLOAT_EQ(*std::max_element(soft.begin(), soft.end()), 1.0f) << "well clear of it is fully lit";
}

// Kat's sawtooth (2026-10-28) must survive the filter: a lit face steep to the sun, sampled from 10 cm inside its
// edges, is lit at filter 1 exactly as at 0, every tap carrying its own receiver plane. The filter must not bring
// back the self-shadow bands the plane removed.
TEST(SoftShadow, TheFilterDoesNotShadowALitFaceOrTheGroundBesideIt) {
    RenderScene scene = big_world();
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = translation(glm::vec3(0.0f, 1.0f, 0.0f)), .material_override = kNoMaterial});
    const auto map = build_static_shadow_map(scene);
    ASSERT_TRUE(map.has_value());
    struct Face {
        glm::vec3 normal, centre, u, v;
    };
    const Face faces[3] = {
        {{1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.0f, 2.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},
    };
    uint32_t dark_at_0 = 0, dark_at_1 = 0;
    for (const Face& f : faces) {
        for (int i = -18; i <= 18; ++i) {
            for (int j = -18; j <= 18; ++j) {
                const glm::vec3 p = f.centre + f.u * (0.05f * static_cast<float>(i)) + f.v * (0.05f * static_cast<float>(j));
                dark_at_0 += sample_shadow(*map, p, f.normal, 0.0f) < 0.999f ? 1u : 0u;
                dark_at_1 += sample_shadow(*map, p, f.normal, 1.0f) < 0.999f ? 1u : 0u;
            }
        }
    }
    EXPECT_EQ(dark_at_0, dark_at_1) << "the faces: filter 1 darkens no point that filter 0 leaves lit";
    EXPECT_EQ(dark_at_1, 0u) << "a lit face is lit";
}

// ---------------------------------------------------------------------------
// FocusShadow
// ---------------------------------------------------------------------------

[[nodiscard]] RenderScene world_with_a_small_dynamic_box(glm::vec3 where, float half_extent) {
    RenderScene scene = big_world();
    scene.meshes.push_back(make_box_mesh(half_extent));  // 2: the small body
    scene.dynamics.push_back(DrawItem{.mesh_index = 2, .local_to_world = translation(where), .material_override = kNoMaterial});
    return scene;
}

// A 0.5 m body 1 m above the ground of a 172 m world: its map has texels at or under 2 mm (the world's are 15 cm),
// and its shadow's edge on the ground lands within a centimetre of the geometric one on both sides.
TEST(FocusShadow, ASmallDynamicCasterOnALargeWorldGetsMillimetreTexelsAndAnEdgeWithinACentimetre) {
    RenderScene scene = world_with_a_small_dynamic_box(glm::vec3(0.0f, 1.0f, 0.0f), 0.25f);
    const auto statics = build_static_shadow_map(scene);
    ASSERT_TRUE(statics.has_value());
    ASSERT_GT(shadow_texel_metres(*statics), 0.12f);
    const auto focus = build_focus_shadow_map(scene, 1024, 1.0f, *statics);
    ASSERT_TRUE(focus.has_value()) << "a lone small body gets a map of its own";
    EXPECT_LE(shadow_texel_metres(*focus), 0.002f) << "2 mm or under for a 0.5 m caster at size 1024";
    EXPECT_FLOAT_EQ(focus->depth_bias, spade::render::kFocusShadowDepthBias);

    const ShadowSet set{.statics = &*statics, .focus = &*focus, .filter_texels = 1.0f};
    const glm::vec3 sun = scene.lighting.sun_direction;
    const glm::vec3 lo(-0.25f, 0.75f, -0.25f), hi(0.25f, 1.25f, 0.25f);
    uint32_t inside_points = 0, inside_shadowed = 0, outside_points = 0, outside_lit = 0;
    for (int i = -60; i <= 60; ++i) {
        for (int j = -60; j <= 60; ++j) {
            // The ground under where the shadow falls: -sun from the body, a few metres either way.
            const glm::vec3 q(0.02f * static_cast<float>(i) - sun.x / sun.y, 0.0f, 0.02f * static_cast<float>(j) - sun.z / sun.y);
            const bool meets_shrunk = ray_meets_box(sun, q, lo, hi, -0.01f);
            const bool meets_grown = ray_meets_box(sun, q, lo, hi, 0.01f);
            const float lit = set.sample(q, glm::vec3(0.0f, 1.0f, 0.0f));
            if (meets_shrunk) {
                ++inside_points;
                inside_shadowed += lit < 0.5f ? 1u : 0u;
            } else if (!meets_grown) {
                ++outside_points;
                outside_lit += lit > 0.5f ? 1u : 0u;
            }
        }
    }
    ASSERT_GT(inside_points, 100u);
    ASSERT_GT(outside_points, 100u);
    EXPECT_EQ(inside_shadowed, inside_points) << "every ground point 1 cm or more inside the shadow is shadowed";
    EXPECT_EQ(outside_lit, outside_points) << "every ground point 1 cm or more outside it is lit";
}

// SR-17 is lifted for dynamics (L42, closing M6): a body past scene.bounds casts through its own map, where the world
// map never saw it. Statics past the bounds are still unshadowed, as SR-17 says.
TEST(FocusShadow, ADynamicBodyPastTheWorldsBoundsCastsAndAStaticOneStillDoesNot) {
    const glm::vec3 far(300.0f, 1.0f, 0.0f);  // 208 m past the +x bound
    RenderScene scene = world_with_a_small_dynamic_box(far, 0.25f);
    scene.statics.push_back(DrawItem{.mesh_index = 2, .local_to_world = translation(far + glm::vec3(0.0f, 0.0f, 40.0f)),
                                     .material_override = kNoMaterial});  // a static twin 40 m along z
    const auto statics = build_static_shadow_map(scene);
    ASSERT_TRUE(statics.has_value());
    const auto focus = build_focus_shadow_map(scene, 1024, 1.0f, *statics);
    ASSERT_TRUE(focus.has_value());
    const ShadowSet set{.statics = &*statics, .focus = &*focus, .filter_texels = 0.0f};
    const glm::vec3 sun = scene.lighting.sun_direction;
    const glm::vec3 under_dynamic(far.x - sun.x / sun.y, 0.0f, far.z - sun.z / sun.y);
    const glm::vec3 under_static(far.x - sun.x / sun.y, 0.0f, far.z + 40.0f - sun.z / sun.y);
    const glm::vec3 up(0.0f, 1.0f, 0.0f);
    EXPECT_LT(set.sample(under_dynamic, up), 0.5f) << "the dynamic body's shadow lands on the ground past the bounds";
    EXPECT_GT(set.sample(under_static, up), 0.5f) << "a static caster past the bounds casts nothing (SR-17)";
}

// Bodies so far apart that their own map would be no finer than the world's: build_focus_shadow_map() declines, so
// render() folds them into the static copy as before, and the result is never worse than today's.
TEST(FocusShadow, BodiesSoSpreadOutThatTheirMapWouldBeNoFinerFallBackToFolding) {
    RenderScene scene = world_with_a_small_dynamic_box(glm::vec3(-70.0f, 1.0f, 0.0f), 0.25f);
    scene.dynamics.push_back(
        DrawItem{.mesh_index = 2, .local_to_world = translation(glm::vec3(300.0f, 1.0f, 0.0f)), .material_override = kNoMaterial});
    const auto statics = build_static_shadow_map(scene);
    ASSERT_TRUE(statics.has_value());
    EXPECT_FALSE(build_focus_shadow_map(scene, 1024, 1.0f, *statics).has_value())
        << "370 m apart, wider than the world: a 1024 map of their union is no finer than the world's";
    // Nearer together it is finer than the world's, so it is kept; and never coarser than the world's when kept.
    scene.dynamics.back().local_to_world = translation(glm::vec3(-60.0f, 1.0f, 0.0f));
    const auto kept = build_focus_shadow_map(scene, 1024, 1.0f, *statics);
    ASSERT_TRUE(kept.has_value());
    EXPECT_LT(shadow_texel_metres(*kept), shadow_texel_metres(*statics));
    // And no dynamics, no map.
    scene.dynamics.clear();
    EXPECT_FALSE(build_focus_shadow_map(scene, 1024, 1.0f, *statics).has_value());
}

// ---------------------------------------------------------------------------
// EarlyOut
// ---------------------------------------------------------------------------

// The filtered lookup returns "lit" without the plane arithmetic when no tap's stored depth is nearer than the point
// (the lead's ruling, 2026-11-09: exact, so every image and every digest is as before it). Held to the full loop, bit
// for bit, over a dense off-grid sample of ground and box faces at filters 0, 0.5, 1 and 2, on the world's map and on a
// dynamic body's own map, with edges in both (so that both the skipped and the unskipped branches are exercised).
TEST(EarlyOut, TheFilteredLookupIsBitIdenticalToTheFullLoopWithItOff) {
    RenderScene scene = world_with_a_small_dynamic_box(glm::vec3(3.0f, 1.0f, 1.0f), 0.25f);
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = translation(glm::vec3(0.0f, 1.0f, 0.0f)), .material_override = kNoMaterial});
    const auto statics = build_static_shadow_map(scene);
    ASSERT_TRUE(statics.has_value());
    const auto focus = build_focus_shadow_map(scene, 1024, 1.0f, *statics);
    ASSERT_TRUE(focus.has_value());

    const std::array<glm::vec3, 3> normals = {glm::vec3(0.0f, 1.0f, 0.0f), glm::normalize(glm::vec3(0.6f, 0.8f, 0.0f)),
                                              glm::vec3(1.0f, 0.0f, 0.0f)};
    const std::array<float, 4> filters = {0.0f, 0.5f, 1.0f, 2.0f};
    const std::array<const ShadowMap*, 2> maps = {&*statics, &*focus};
    uint64_t points = 0, differing = 0, partial = 0, skipped_candidates = 0;
    for (int i = -150; i <= 150; ++i) {
        for (int j = -90; j <= 90; ++j) {
            const float x = 0.0371f * static_cast<float>(i) + 0.0013f, z = 0.0359f * static_cast<float>(j) - 0.0007f;
            for (const float y : {0.0f, 2.0f}) {  // the ground, and the static box's top (the same plane height)
                const glm::vec3 q(x, y, z);
                for (const glm::vec3& n : normals) {
                    for (const ShadowMap* map : maps) {
                        for (const float f : filters) {
                            const float a = sample_shadow(*map, q, n, f);
                            const float b = spade::render::sample_shadow_without_early_out(*map, q, n, f);
                            ++points;
                            differing += std::bit_cast<uint32_t>(a) != std::bit_cast<uint32_t>(b) ? 1u : 0u;
                            partial += (f == 1.0f && a > 0.0f && a < 1.0f) ? 1u : 0u;
                            skipped_candidates += a == 1.0f ? 1u : 0u;
                        }
                    }
                }
            }
        }
    }
    EXPECT_EQ(differing, 0u) << differing << " of " << points << " samples differ between the lookup and the full loop";
    EXPECT_GT(partial, 200u) << "the sample must reach penumbrae, or it proves nothing about the unskipped branch";
    EXPECT_GT(skipped_candidates, points / 4u) << "and lit points, or it proves nothing about the skip";
}

// ---------------------------------------------------------------------------
// ShadowDefaults
// ---------------------------------------------------------------------------

// What an untouched RenderOptions does (Kat's host leaves it at its defaults): soft and tight. `shadows` stays on/off.
TEST(ShadowDefaults, AnUntouchedRenderOptionsIsSoftAndTight) {
    const RenderOptions options;
    EXPECT_TRUE(options.shadows);
    EXPECT_FLOAT_EQ(options.shadow_filter_texels, 1.0f);
    EXPECT_EQ(options.dynamic_shadow_size, 1024u);
}

// A frame with a small dynamic caster, at the defaults: byte-identical across two calls, and different from the same
// frame with shadows off.
TEST(ShadowDefaults, RenderIsByteIdenticalTwiceAndShadowsOffStillDiffers) {
    RenderScene scene = world_with_a_small_dynamic_box(glm::vec3(0.0f, 1.0f, 0.0f), 0.25f);
    const auto statics = build_static_shadow_map(scene);
    ASSERT_TRUE(statics.has_value());
    scene.static_shadow = *statics;
    Camera camera;
    camera.position = glm::vec3(3.0f, 2.5f, 3.0f);
    camera.orientation = glm::quatLookAt(glm::normalize(glm::vec3(0.0f, 0.5f, 0.0f) - camera.position), glm::vec3(0.0f, 1.0f, 0.0f));
    constexpr uint32_t w = 160, h = 120;
    const auto frame = [&](const RenderOptions& options) {
        std::vector<uint8_t> pixels(static_cast<size_t>(w) * h * 4u, 0u);
        spade::render::RenderTarget target{.pixels = std::span<uint8_t>(pixels), .width = w, .height = h, .stride = w * 4u};
        const auto drew = spade::render::render(scene, camera, options, target);
        EXPECT_TRUE(drew.has_value());
        return pixels;
    };
    RenderOptions on;
    on.overlays = false;
    RenderOptions off = on;
    off.shadows = false;
    EXPECT_EQ(frame(on), frame(on));
    EXPECT_NE(frame(on), frame(off));
}

}  // namespace
