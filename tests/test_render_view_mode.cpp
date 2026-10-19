// render/view_mode.hpp: what a view shows, a base plus flow layers. The
// names are a menu's and a saved view's, so they are pinned as literals;
// supports() is held to the CPU's real draw on every case (view_mode_cases.hpp,
// the lead's condition; GL's twin is in test_gpu_gl_renderer.cpp); apply_view
// builds one FlowLayer from the volume and iso layers, and a legend per layer
// with the range the renderers resolve.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "render/raster_cpu.hpp"
#include "render/view_mode.hpp"
#include "view_mode_cases.hpp"

namespace {

using spade::render::DrawMode;
using spade::render::FlowView;
using spade::render::FlowViewSpec;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::ViewBackend;
using spade::render::ViewBase;
using spade::render::ViewSpec;
using spade::test::layer;

TEST(ViewMode, NamesAreStableAndReadBack) {
    EXPECT_EQ(spade::render::view_base_name(ViewBase::shaded), "shaded");
    EXPECT_EQ(spade::render::view_base_name(ViewBase::wireframe), "wireframe");
    EXPECT_EQ(spade::render::view_base_name(ViewBase::velocity), "velocity");
    EXPECT_EQ(spade::render::view_base_name(ViewBase::exact_sdf), "exact_sdf");
    EXPECT_EQ(spade::render::flow_view_name(FlowView::volume), "volume");
    EXPECT_EQ(spade::render::flow_view_name(FlowView::vortex_iso), "vortex_iso");
    EXPECT_EQ(spade::render::flow_view_name(FlowView::streamlines), "streamlines");
    EXPECT_EQ(spade::render::flow_view_name(FlowView::slice), "slice");
    EXPECT_EQ(spade::render::flow_view_name(FlowView::surface_cp), "surface_cp");
    for (uint32_t b = 0; b < spade::render::kViewBaseCount; ++b) {
        const auto base = static_cast<ViewBase>(b);
        EXPECT_EQ(spade::render::view_base_from_name(spade::render::view_base_name(base)), base);
    }
    for (uint32_t v = 0; v < spade::render::kFlowViewCount; ++v) {
        const auto view = static_cast<FlowView>(v);
        EXPECT_EQ(spade::render::flow_view_from_name(spade::render::flow_view_name(view)), view);
    }
    EXPECT_EQ(spade::render::view_base_from_name("raymarch"), std::nullopt);
    EXPECT_EQ(spade::render::flow_view_from_name(""), std::nullopt);
}

// The lead's condition: for every case, supports() says drawable exactly when
// apply_view() and render() draw it, and names the reason they refuse with.
TEST(ViewMode, EveryViewDrawsOnTheCpuExactlyWhenSupportsSaysSo) {
    constexpr uint32_t kSide = 32;
    std::vector<uint8_t> pixels(kSide * kSide * 4u);
    spade::render::Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 4.0f);
    spade::test::expect_view_parity(ViewBackend::cpu, [&](const RenderScene& scene, const RenderOptions& options) {
        spade::render::RenderTarget target{.pixels = std::span<uint8_t>(pixels), .width = kSide, .height = kSide,
                                           .stride = kSide * 4u};
        return spade::render::render(scene, camera, options, target);
    });
}

TEST(ViewMode, TheBaseSetsTheDrawMode) {
    const spade::FlowVolume volume = spade::test::view_mode_volume();
    const spade::SdfProgram sdf = spade::test::view_mode_sdf();
    const std::pair<ViewBase, DrawMode> pairs[] = {{ViewBase::shaded, DrawMode::shaded},
                                                   {ViewBase::wireframe, DrawMode::wireframe},
                                                   {ViewBase::velocity, DrawMode::velocity},
                                                   {ViewBase::exact_sdf, DrawMode::raymarch}};
    for (const auto& [base, mode] : pairs) {
        RenderScene scene = spade::test::view_mode_scene(true, &sdf);
        RenderOptions options;
        options.mode = DrawMode::wireframe == mode ? DrawMode::shaded : DrawMode::wireframe;
        ASSERT_TRUE(spade::render::apply_view(ViewSpec{base, {}}, &volume, scene, options).has_value());
        EXPECT_EQ(options.mode, mode) << spade::render::view_base_name(base);
    }
}

// Volume and iso layers over one volume march as ONE FlowLayer, so GL's one
// layer per frame holds a whole spec; the rest go where the renderers draw them.
TEST(ViewMode, VolumeAndIsoLayersMergeIntoOneFlowLayer) {
    const spade::FlowVolume volume = spade::test::view_mode_volume();
    RenderScene scene = spade::test::view_mode_scene(true, nullptr);
    RenderOptions options;
    const ViewSpec spec{ViewBase::shaded,
                        {layer(FlowView::volume), layer(FlowView::vortex_iso), layer(FlowView::volume),
                         layer(FlowView::streamlines), layer(FlowView::slice), layer(FlowView::surface_cp)}};
    auto legends = spade::render::apply_view(spec, &volume, scene, options);
    ASSERT_TRUE(legends.has_value()) << legends.error().context;
    ASSERT_EQ(scene.flow_layers.size(), 1u);
    EXPECT_EQ(scene.flow_layers[0].volume, &volume);
    EXPECT_EQ(scene.flow_layers[0].transfers.size(), 2u);
    ASSERT_EQ(scene.flow_layers[0].isos.size(), 1u);
    EXPECT_EQ(scene.flow_layers[0].isos[0].quantity, spade::render::FlowQuantity::lambda2_hat);
    EXPECT_EQ(scene.flow_layers[0].isos[0].value, -2.0f);
    ASSERT_TRUE(scene.flow_layers[0].isos[0].colour_by.has_value());
    EXPECT_EQ(scene.flow_layers[0].isos[0].colour_by->quantity, spade::render::FlowQuantity::speed);
    EXPECT_FALSE(scene.flow_polylines.empty());
    EXPECT_EQ(scene.field_layers.size(), 1u);
    ASSERT_TRUE(scene.flow_surface_colouring.has_value());
    EXPECT_TRUE(scene.flow_surface_colouring->walls_only);
    EXPECT_EQ(scene.flow_surface_colouring->volume, &volume);
    EXPECT_EQ(legends->size(), spec.layers.size());
}

// A legend per layer, in spec order: what the colour shows, its unit, and the
// range the renderers resolve (resolve_range(); for C_p, back in C_p).
TEST(ViewMode, ALegendNamesEachLayersScale) {
    const spade::FlowVolume volume = spade::test::view_mode_volume();
    RenderScene scene = spade::test::view_mode_scene(true, nullptr);
    RenderOptions options;
    FlowViewSpec vorticity = layer(FlowView::volume);
    vorticity.quantity = spade::render::FlowQuantity::vorticity;
    vorticity.colours = spade::render::ColourMapId::inferno;
    vorticity.range_max = 30.0f;
    const ViewSpec spec{ViewBase::shaded, {vorticity, layer(FlowView::vortex_iso), layer(FlowView::streamlines),
                                           layer(FlowView::slice), layer(FlowView::surface_cp)}};
    auto legends = spade::render::apply_view(spec, &volume, scene, options);
    ASSERT_TRUE(legends.has_value()) << legends.error().context;
    ASSERT_EQ(legends->size(), 5u);
    const auto& l = *legends;
    EXPECT_EQ(l[0].layer, FlowView::volume);
    EXPECT_EQ(l[0].quantity, "vorticity |w|");
    EXPECT_EQ(l[0].unit, "1/s");
    EXPECT_EQ(l[0].colours, spade::render::ColourMapId::inferno);
    EXPECT_EQ(l[0].range.min, 0.0f);
    EXPECT_EQ(l[0].range.max, 30.0f);
    EXPECT_EQ(l[1].layer, FlowView::vortex_iso);  // coloured by speed, not by lambda2
    EXPECT_EQ(l[1].quantity, "speed |u|");
    EXPECT_EQ(l[1].unit, "m/s");
    EXPECT_EQ(l[1].range.max, 2.0f);
    EXPECT_EQ(l[2].quantity, "speed |u|");
    EXPECT_EQ(l[3].quantity, "gauge pressure");
    EXPECT_EQ(l[3].unit, "Pa");
    EXPECT_EQ(l[3].range.min, -10.0f);
    EXPECT_EQ(l[3].range.max, 10.0f);
    EXPECT_EQ(l[4].quantity, "C_p");
    EXPECT_EQ(l[4].unit, "");
    EXPECT_NEAR(l[4].range.min, -1.0f, 1e-6f);
    EXPECT_NEAR(l[4].range.max, 1.0f, 1e-6f);
    for (const auto& legend : l) EXPECT_FALSE(legend.range.from_header);
}

// A refusal changes nothing: the scene and options are as they were.
TEST(ViewMode, ARefusedViewChangesNothing) {
    const spade::FlowVolume volume = spade::test::view_mode_volume();
    RenderScene scene = spade::test::view_mode_scene(true, nullptr);
    RenderOptions options;
    options.mode = DrawMode::wireframe;
    const ViewSpec two_cp{ViewBase::shaded, {layer(FlowView::slice), layer(FlowView::surface_cp), layer(FlowView::surface_cp)}};
    auto applied = spade::render::apply_view(two_cp, &volume, scene, options);
    ASSERT_FALSE(applied.has_value());
    EXPECT_EQ(options.mode, DrawMode::wireframe);
    EXPECT_TRUE(scene.field_layers.empty());
    EXPECT_FALSE(scene.flow_surface_colouring.has_value());
}

// The lead's (a), 2026-10-19: N views of one generation trace their lines
// once through a ViewCache, and draw what an uncached apply_view() draws.
// Another colouring reuses the lines; new seeds, new options or a new
// generation trace again; generation 0 traces every time.
TEST(ViewMode, NViewsOfOneGenerationTraceTheirLinesOnce) {
    spade::FlowVolume volume = spade::test::view_mode_volume();
    volume.generation = spade::next_flow_generation();
    const ViewSpec spec{ViewBase::shaded, {layer(FlowView::streamlines)}};
    RenderOptions options;
    RenderScene uncached = spade::test::view_mode_scene(true, nullptr);
    ASSERT_TRUE(spade::render::apply_view(spec, &volume, uncached, options).has_value());
    ASSERT_FALSE(uncached.flow_polylines.empty());

    spade::render::ViewCache cache;
    const auto apply = [&](const ViewSpec& view) {
        RenderScene scene = spade::test::view_mode_scene(true, nullptr);
        auto applied = spade::render::apply_view(view, &volume, scene, options, cache);
        EXPECT_TRUE(applied.has_value()) << applied.error().context;
        return scene;
    };
    for (int view = 0; view < 4; ++view) {
        const RenderScene scene = apply(spec);
        ASSERT_EQ(scene.flow_polylines.size(), uncached.flow_polylines.size());
        for (std::size_t i = 0; i < scene.flow_polylines.size(); ++i) {
            EXPECT_EQ(scene.flow_polylines[i].points, uncached.flow_polylines[i].points) << "view " << view;
            EXPECT_EQ(scene.flow_polylines[i].colours, uncached.flow_polylines[i].colours) << "view " << view;
        }
    }
    EXPECT_EQ(cache.traces(), 1u) << "four views of one generation trace once";

    ViewSpec recoloured = spec;
    recoloured.layers[0].colours = spade::render::ColourMapId::inferno;
    const RenderScene other = apply(recoloured);
    EXPECT_EQ(cache.traces(), 1u) << "another colouring reuses the lines";
    ASSERT_FALSE(other.flow_polylines.empty());
    EXPECT_NE(other.flow_polylines[0].colours, uncached.flow_polylines[0].colours);

    ViewSpec reseeded = spec;
    reseeded.layers[0].seeds.push_back(glm::vec3(-0.8f, -0.3f, 0.0f));
    (void)apply(reseeded);
    EXPECT_EQ(cache.traces(), 2u) << "new seeds trace";
    ViewSpec shorter = spec;
    shorter.layers[0].lines.max_steps = 100;
    (void)apply(shorter);
    EXPECT_EQ(cache.traces(), 3u) << "new options trace";
    (void)apply(spec);
    EXPECT_EQ(cache.traces(), 3u) << "the first seeds and options are still held";

    volume.generation = spade::next_flow_generation();
    (void)apply(spec);
    (void)apply(spec);
    EXPECT_EQ(cache.traces(), 4u) << "a new generation traces once";
    volume.generation = 0;
    (void)apply(spec);
    (void)apply(spec);
    EXPECT_EQ(cache.traces(), 6u) << "generation 0 traces every time";
}

}  // namespace
