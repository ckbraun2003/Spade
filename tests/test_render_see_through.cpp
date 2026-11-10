// ---------------------------------------------------------------------------
// test_render_see_through.cpp -- DrawItem::see_through (RND-11, L47) on the CPU renderer, the reference GL is held
// to: drawn after the opaques, farther first, src-over with the material's alpha, depth-tested with no depth write,
// receiving shadows and never casting them, shaded frames only. The geometry is chosen so the bytes are exact: a
// 33 x 33 frame, the camera 5 m back on the axis, flat unlit quads facing it, black sky, so a pixel at the centre is
// the blend's alone and to_byte() = lround(x * 255) gives it.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/flow_lines.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/shadow.hpp"
#include "world/sdf.hpp"

namespace {

using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::DrawMode;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

constexpr uint32_t kW = 33, kH = 33;

struct Frame {
    std::vector<uint8_t> px;
    [[nodiscard]] glm::ivec3 rgb(uint32_t x, uint32_t y) const {
        const size_t i = (static_cast<size_t>(y) * kW + x) * 4u;
        return glm::ivec3(px[i + 2], px[i + 1], px[i + 0]);
    }
    [[nodiscard]] glm::ivec3 centre() const { return rgb(16, 16); }
};

[[nodiscard]] Frame draw(const RenderScene& scene, RenderOptions options = {}, Camera camera = {},
                         spade::Result<void>* result = nullptr) {
    Frame f;
    f.px.assign(static_cast<size_t>(kW) * kH * 4u, 0xABu);
    RenderTarget target{.pixels = std::span<uint8_t>(f.px), .width = kW, .height = kH, .stride = kW * 4u};
    if (camera.position == glm::vec3(0.0f)) camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    options.overlays = false;
    const spade::Result<void> r = spade::render::render(scene, camera, options, target);
    if (result != nullptr) {
        *result = r;
    } else {
        EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().context);
    }
    return f;
}

// A quad of half-width 2 facing +Z (counter-clockwise seen from +Z), one submesh on material 0.
[[nodiscard]] MeshData quad_mesh() {
    MeshData m;
    const float h = 2.0f;
    m.positions = {{h, -h, 0.0f}, {h, h, 0.0f}, {-h, h, 0.0f}, {-h, -h, 0.0f}};
    m.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    m.indices = {0, 1, 2, 0, 2, 3};
    m.submesh_first_index = {0};
    m.submesh_index_count = {6};
    m.submesh_material = {0};
    return m;
}

struct Layer {
    glm::vec4 colour;  // rgb unlit, a the alpha
    float z = 0.0f;
    bool see_through = false;
    bool dynamic = false;
};

// One shared quad mesh, one unlit material a layer (the item overrides to it), black sky.
[[nodiscard]] RenderScene scene_of(const std::vector<Layer>& layers) {
    RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    scene.meshes.push_back(quad_mesh());
    scene.materials.push_back(Material{});  // the mesh's own submesh material: opaque grey, never the layers'
    for (const Layer& l : layers) {
        scene.materials.push_back(Material{.base_color = l.colour, .shading = 1});
        DrawItem item;
        item.mesh_index = 0;
        item.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, l.z));
        item.material_override = static_cast<uint32_t>(scene.materials.size() - 1u);
        item.see_through = l.see_through;
        (l.dynamic ? scene.dynamics : scene.statics).push_back(item);
    }
    return scene;
}

constexpr glm::vec4 kRed{1.0f, 0.0f, 0.0f, 1.0f};
constexpr glm::vec4 kGreen{0.0f, 1.0f, 0.0f, 1.0f};
constexpr glm::vec4 kBlue{0.0f, 0.0f, 1.0f, 1.0f};
[[nodiscard]] glm::vec4 with_alpha(glm::vec4 c, float a) {
    c.a = a;
    return c;
}

// ---- the default ----------------------------------------------------------------------------------------

// Unflagged is opaque whatever the alpha, as every frame so far was.
TEST(SeeThrough, AnUnflaggedItemIgnoresItsAlphaAndDrawsOpaque) {
    const Frame faint = draw(scene_of({Layer{with_alpha(kRed, 0.3f), 0.0f, false}}));
    const Frame solid = draw(scene_of({Layer{with_alpha(kRed, 1.0f), 0.0f, false}}));
    EXPECT_EQ(faint.px, solid.px);
    EXPECT_EQ(faint.centre(), glm::ivec3(255, 0, 0));
}

// ---- the blend --------------------------------------------------------------------------------------------

// out = to_byte(dst / 255 * (1 - a) + src * a), float, one rounding.
TEST(SeeThrough, ABlendIsSrcOverWithTheMaterialAlphaInOneRounding) {
    // Over the black sky: red at 0.5 is lround(0.5 * 255) = 128.
    EXPECT_EQ(draw(scene_of({Layer{with_alpha(kRed, 0.5f), 0.0f, true}})).centre(), glm::ivec3(128, 0, 0));
    // Over an opaque green behind it (z -1): 0.5 of each.
    const Frame over_green =
        draw(scene_of({Layer{kGreen, -1.0f, false}, Layer{with_alpha(kRed, 0.5f), 0.0f, true}}));
    EXPECT_EQ(over_green.centre(), glm::ivec3(128, 128, 0));
    // Alpha 0.25: red is lround(0.25 * 255 = 63.75) = 64, green is lround(0.75 * 255 = 191.25) = 191.
    const Frame quarter =
        draw(scene_of({Layer{kGreen, -1.0f, false}, Layer{with_alpha(kRed, 0.25f), 0.0f, true}}));
    EXPECT_EQ(quarter.centre(), glm::ivec3(64, 191, 0));
    // Outside the quad nothing changes.
    EXPECT_EQ(over_green.rgb(1, 16), glm::ivec3(0, 0, 0));
}

TEST(SeeThrough, AlphaIsClampedToZeroToOneAndTheOverrideMaterialWins) {
    EXPECT_EQ(draw(scene_of({Layer{kGreen, -1.0f, false}, Layer{with_alpha(kRed, 1.7f), 0.0f, true}})).centre(),
              glm::ivec3(255, 0, 0));
    EXPECT_EQ(draw(scene_of({Layer{kGreen, -1.0f, false}, Layer{with_alpha(kRed, -0.5f), 0.0f, true}})).centre(),
              glm::ivec3(0, 255, 0));
    // The submesh's own material 0 is opaque grey; the override (material 1, blue at 0.5) is what blends.
    EXPECT_EQ(draw(scene_of({Layer{with_alpha(kBlue, 0.5f), 0.0f, true}})).centre(), glm::ivec3(0, 0, 128));
    // With no override the submesh's own material is the one: material 0 at alpha 0.5, unlit white.
    RenderScene own = scene_of({Layer{with_alpha(kBlue, 0.5f), 0.0f, true}});
    own.statics[0].material_override = spade::render::kNoMaterial;
    own.materials[0] = Material{.base_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.5f), .shading = 1};
    EXPECT_EQ(draw(own).centre(), glm::ivec3(128, 128, 128));
}

// Depth-tested against the opaque depth, and writing none.
TEST(SeeThrough, ItIsHiddenBehindAnOpaqueItemAndWritesNoDepthOfItsOwn) {
    // Behind the opaque green: not drawn at all.
    EXPECT_EQ(draw(scene_of({Layer{kGreen, 0.0f, false}, Layer{with_alpha(kRed, 0.5f), -1.0f, true}})).centre(),
              glm::ivec3(0, 255, 0));
    // No depth write: a line behind the see-through quad is drawn over it afterwards, at its own colour.
    RenderScene scene = scene_of({Layer{with_alpha(kRed, 0.5f), 1.0f, true}});
    scene.flow_polylines.push_back(spade::render::FlowPolyline{
        {glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)}, {glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f)}});
    EXPECT_EQ(draw(scene).centre(), glm::ivec3(0, 0, 255)) << "lines draw after, depth-tested on the opaque depth only";
}

// ---- the order --------------------------------------------------------------------------------------------

TEST(SeeThrough, FartherFirstWhateverTheListingOrder) {
    // Red at z = 0 (farther from the camera at z = 5), blue at z = 1 (nearer), both at 0.5 over black:
    // red first: r = lround(0.5 * 255) = 128, then blue: r = lround(128 / 255 * 0.5 * 255) = 64, b = 128.
    const Frame forward =
        draw(scene_of({Layer{with_alpha(kRed, 0.5f), 0.0f, true}, Layer{with_alpha(kBlue, 0.5f), 1.0f, true}}));
    const Frame reversed =
        draw(scene_of({Layer{with_alpha(kBlue, 0.5f), 1.0f, true}, Layer{with_alpha(kRed, 0.5f), 0.0f, true}}));
    EXPECT_EQ(forward.centre(), glm::ivec3(64, 0, 128));
    EXPECT_EQ(forward.px, reversed.px);
    // A dynamic item and a static one at depths 1 and 0 draw by depth, not by the list they sit in.
    const Frame mixed = draw(scene_of(
        {Layer{with_alpha(kBlue, 0.5f), 1.0f, true, false}, Layer{with_alpha(kRed, 0.5f), 0.0f, true, true}}));
    EXPECT_EQ(mixed.px, forward.px);
}

TEST(SeeThrough, TiesFollowTheItemIndexStaticsFirstThenDynamics) {
    // Same depth: the lower index draws first. Red then blue: (64, 0, 128); blue then red: (128, 0, 64).
    EXPECT_EQ(draw(scene_of({Layer{with_alpha(kRed, 0.5f), 0.0f, true}, Layer{with_alpha(kBlue, 0.5f), 0.0f, true}}))
                  .centre(),
              glm::ivec3(64, 0, 128));
    EXPECT_EQ(draw(scene_of({Layer{with_alpha(kBlue, 0.5f), 0.0f, true}, Layer{with_alpha(kRed, 0.5f), 0.0f, true}}))
                  .centre(),
              glm::ivec3(128, 0, 64));
    // A static and a dynamic at the same depth: the static (the lower index) draws first.
    EXPECT_EQ(draw(scene_of({Layer{with_alpha(kBlue, 0.5f), 0.0f, true, true}, Layer{with_alpha(kRed, 0.5f), 0.0f, true, false}}))
                  .centre(),
              glm::ivec3(64, 0, 128))
        << "the red static first, then the blue dynamic";
}

TEST(SeeThrough, TheOrderIsFartherFirstByViewDepthAndACentreBehindTheEyeKeepsItsKey) {
    // Camera at z = 5 looking down -Z. Centres at z = 0 (depth 5), 3 (depth 2), 4.5 (depth 0.5), 8 (behind: -3).
    RenderScene scene = scene_of({Layer{kRed, 3.0f, true}, Layer{kRed, 0.0f, true}, Layer{kRed, 8.0f, true},
                                  Layer{kRed, 4.5f, true}});
    scene.statics.push_back(scene.statics[0]);  // an unflagged opaque in the list is not in the order
    scene.statics.back().see_through = false;
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    const std::vector<uint32_t> order = spade::render::see_through_order(scene, camera);
    // Farthest first: index 1 (depth 5), 0 (2), 3 (0.5), 2 (-3).
    EXPECT_EQ(order, (std::vector<uint32_t>{1, 0, 3, 2}));
    // A flagged dynamic is indexed after the statics.
    RenderScene with_dynamic = scene;
    with_dynamic.dynamics.push_back(with_dynamic.statics[1]);
    with_dynamic.dynamics.back().local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -2.0f));
    with_dynamic.dynamics.back().see_through = true;
    const std::vector<uint32_t> longer = spade::render::see_through_order(with_dynamic, camera);
    ASSERT_EQ(longer.size(), 5u);
    EXPECT_EQ(longer.front(), static_cast<uint32_t>(with_dynamic.statics.size()));
    // A mesh-less item is left out.
    RenderScene bare = scene;
    bare.statics[0].mesh_index = spade::render::kNoMesh;
    EXPECT_EQ(spade::render::see_through_order(bare, camera), (std::vector<uint32_t>{1, 3, 2}));
    // Deterministic.
    EXPECT_EQ(spade::render::see_through_order(scene, camera), order);
}

// ---- shadows ----------------------------------------------------------------------------------------------

TEST(SeeThrough, ASeeThroughItemNeverCastsAShadowIntoAnyOfTheMaps) {
    RenderScene scene = scene_of({Layer{kRed, 0.0f, true}});
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-4.0f), .max = glm::vec3(4.0f)};
    const auto map = spade::render::build_static_shadow_map(scene, 64);
    ASSERT_TRUE(map.has_value());
    EXPECT_TRUE(std::all_of(map->depth.begin(), map->depth.end(),
                            [](float d) { return d == spade::render::kNoOccluder; }))
        << "a flagged static casts nothing into the static map";
    RenderScene opaque = scene;
    opaque.statics[0].see_through = false;
    const auto control = spade::render::build_static_shadow_map(opaque, 64);
    ASSERT_TRUE(control.has_value());
    EXPECT_FALSE(std::all_of(control->depth.begin(), control->depth.end(),
                             [](float d) { return d == spade::render::kNoOccluder; }))
        << "the same item unflagged does";

    // The fold: a flagged dynamic leaves the map as it was.
    RenderScene dyn = scene_of({Layer{kRed, 0.0f, true, true}});
    dyn.bounds = scene.bounds;
    spade::render::ShadowMap folded = *map;
    spade::render::rasterize_shadow_casters(dyn.meshes, dyn.dynamics, folded);
    EXPECT_EQ(folded.depth, map->depth);
    // The focus map has nothing to fit.
    EXPECT_FALSE(spade::render::fit_focus_shadow_map(dyn, 256, 1.0f, *map).has_value());
    RenderScene dyn_opaque = dyn;
    dyn_opaque.dynamics[0].see_through = false;
    EXPECT_TRUE(spade::render::fit_focus_shadow_map(dyn_opaque, 256, 1.0f, *map).has_value()) << "the unflagged one fits";
}

// It does receive them: a lit see-through floor under an opaque caster darkens in the caster's shadow.
TEST(SeeThrough, ASeeThroughItemReceivesAShadow) {
    RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    scene.lighting.sun_direction = glm::vec3(0.0f, 1.0f, 0.0f);  // straight overhead
    scene.materials = {Material{.base_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.5f), .shading = 0},  // the floor, lambert
                       Material{.base_color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), .shading = 1}};  // the caster, unlit
    MeshData floor;  // facing +Y, in the XZ plane, 4 m a side
    floor.positions = {{-2.0f, 0.0f, 2.0f}, {2.0f, 0.0f, 2.0f}, {2.0f, 0.0f, -2.0f}, {-2.0f, 0.0f, -2.0f}};
    floor.normals.assign(4, glm::vec3(0.0f, 1.0f, 0.0f));
    floor.indices = {0, 1, 2, 0, 2, 3};
    floor.submesh_first_index = {0};
    floor.submesh_index_count = {6};
    floor.submesh_material = {0};
    MeshData roof = floor;  // the caster: the same quad, 0.5 m up and 1 m a side, drawn from below so it is seen
    for (glm::vec3& p : roof.positions) p = glm::vec3(p.x * 0.25f, 1.5f, p.z * 0.25f);
    roof.submesh_material = {1};
    scene.meshes = {floor, roof};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = spade::render::kNoMaterial, .see_through = true});
    scene.statics.push_back(DrawItem{.mesh_index = 1, .local_to_world = glm::mat4(1.0f), .material_override = spade::render::kNoMaterial});
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-3.0f, -1.0f, -3.0f), .max = glm::vec3(3.0f, 3.0f, 3.0f)};
    auto map = spade::render::build_static_shadow_map(scene);
    ASSERT_TRUE(map.has_value());
    scene.static_shadow = std::move(*map);
    Camera camera;
    camera.position = glm::vec3(0.0f, 4.0f, 3.5f);
    camera.orientation = glm::angleAxis(glm::radians(-48.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    RenderOptions on;
    on.shadows = true;
    RenderOptions off;
    off.shadows = false;
    const Frame lit = draw(scene, off, camera);
    const Frame shadowed = draw(scene, on, camera);
    size_t darker = 0, brighter = 0;
    for (size_t i = 0; i < lit.px.size(); i += 4) {
        const int a = lit.px[i] + lit.px[i + 1] + lit.px[i + 2];
        const int b = shadowed.px[i] + shadowed.px[i + 1] + shadowed.px[i + 2];
        darker += b < a ? 1u : 0u;
        brighter += b > a ? 1u : 0u;
    }
    EXPECT_GT(darker, 10u) << "the see-through floor takes the caster's shadow";
    EXPECT_EQ(brighter, 0u);
}

// ---- the modes ----------------------------------------------------------------------------------------------

TEST(SeeThrough, SensorModesAndVelocityLeaveItOutAndWireframeDrawsItsEdgesLikeAnyItem) {
    const RenderScene flagged = scene_of({Layer{with_alpha(kRed, 0.5f), 0.0f, true}});
    RenderScene empty = flagged;
    empty.statics.clear();
    for (const DrawMode mode : {DrawMode::normals, DrawMode::velocity}) {
        RenderOptions options;
        options.mode = mode;
        EXPECT_EQ(draw(flagged, options).px, draw(empty, options).px) << "mode " << static_cast<int>(mode);
    }
    RenderOptions wire;
    wire.mode = DrawMode::wireframe;
    RenderScene unflagged = flagged;
    unflagged.statics[0].see_through = false;
    EXPECT_EQ(draw(flagged, wire).px, draw(unflagged, wire).px) << "wireframe ignores the flag";
    EXPECT_NE(draw(flagged, wire).px, draw(empty, wire).px) << "and draws the edges";
}

TEST(SeeThrough, AnItemACsgSubtreeMarchesIsRefusedByName) {
    RenderScene scene = scene_of({Layer{kRed, 0.0f, true}});
    spade::SdfProgram box;
    spade::SdfTransform at;
    at.world_to_local = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 3.0f));
    box.transforms.push_back(at);
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    node.params = glm::vec4(0.5f, 0.5f, 0.5f, 0.0f);
    box.nodes.push_back(node);
    scene.csg_subtrees.push_back(spade::render::CsgSubtree{
        .program = box,
        .bounds = spade::render::Aabb{.min = glm::vec3(-0.6f, -0.6f, -3.6f), .max = glm::vec3(0.6f, 0.6f, -2.4f)},
        .draw_item = 0});
    spade::Result<void> result;
    const Frame f = draw(scene, {}, {}, &result);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::invalid_argument);
    EXPECT_NE(result.error().context.find("see-through"), std::string::npos) << result.error().context;
    EXPECT_NE(result.error().context.find("CSG subtree 0"), std::string::npos) << result.error().context;
    EXPECT_TRUE(std::all_of(f.px.begin(), f.px.end(), [](uint8_t b) { return b == 0xABu; })) << "refused before any pixel";
}

// ---- the atmosphere -------------------------------------------------------------------------------------------

// A see-through surface takes the horizon term as a shaded one does: with the term at full strength and the surface
// well past its onset, the blend is over a hazed colour, so a red quad is no longer pure red under it.
TEST(SeeThrough, ASeeThroughSurfaceTakesTheAtmosphereAsAShadedSurfaceDoes) {
    RenderScene scene = scene_of({Layer{with_alpha(kRed, 0.5f), 0.0f, true}});
    scene.lighting.sky_zenith = glm::vec3(0.0f, 0.0f, 1.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f, 0.0f, 1.0f);
    RenderOptions hazy;
    hazy.horizon_blend_strength = 1.0f;
    hazy.horizon_blend_onset = 1.0f;  // the quad is 5 m away: far past it
    RenderOptions clear;
    clear.horizon_blend_strength = 0.0f;
    const glm::ivec3 with_haze = draw(scene, hazy).centre();
    const glm::ivec3 without = draw(scene, clear).centre();
    EXPECT_NE(with_haze, without) << "the term reaches the blended surface";
    EXPECT_LT(with_haze.r, without.r) << "red is hazed toward the blue sky";
}

}  // namespace
