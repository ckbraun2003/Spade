// The editor's per-view state and its mode menu (sandbox/editor_views.hpp):
// N19, the end-goal design §5. Display-free. The menu is supports()' answer
// for each base on this view's backend and inputs; choosing a mode the view
// cannot draw is refused in supports()' own words, and the view keeps what it
// draws (INT-2). A view whose mode stops being drawable (the backend or the
// inputs changed under it) draws shaded with the reason shown, never blank.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/vec3.hpp>

#include "core/flow_volume.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/view_mode.hpp"
#include "world/builder.hpp"

#include "../sandbox/editor_views.hpp"

using namespace spade::sandbox::editor;
using spade::render::kViewBaseCount;
using spade::render::ViewBackend;
using spade::render::ViewBase;
using spade::render::ViewInputs;
using spade::render::ViewSpec;

namespace {

ViewInputs inputs(bool has_sdf) {
    ViewInputs in;
    in.has_sdf = has_sdf;
    return in;
}

// A ground plane's world and its scene, which borrows the world's SDF. Void,
// so a failed setup stops the test; callers ASSERT_NO_FATAL_FAILURE.
void ground_scene(spade::WorldDesc& world, spade::render::RenderScene& out) {
    auto built = spade::WorldBuilder()
                     .name("editor_views")
                     .environment(spade::Environment{})
                     .capacities(spade::Capacities{1, 1, 1, 1})
                     .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                     .build();
    ASSERT_TRUE(built.has_value()) << built.error().context;
    world = std::move(*built);
    auto scene = spade::render::scene_from_world(world, {});
    ASSERT_TRUE(scene.has_value()) << scene.error().context;
    out = std::move(*scene);
}

}  // namespace

TEST(EditorViews, TheDefaultViewIsShadedWithNoLayers) {
    const ViewState v;
    EXPECT_EQ(v.spec.base, ViewBase::shaded);
    EXPECT_TRUE(v.spec.layers.empty());
    EXPECT_TRUE(v.legends.empty());
    EXPECT_TRUE(v.refusal.empty());
}

TEST(EditorViews, TheMenuListsEveryBaseInOrderByItsStableName) {
    const std::vector<ModeEntry> menu = mode_menu(ViewState{}, ViewBackend::cpu, inputs(true));
    ASSERT_EQ(menu.size(), kViewBaseCount);
    for (uint32_t i = 0; i < kViewBaseCount; ++i) {
        EXPECT_EQ(menu[i].base, static_cast<ViewBase>(i));
        EXPECT_EQ(menu[i].name, spade::render::view_base_name(static_cast<ViewBase>(i)));
    }
}

// The menu never answers for itself: each entry is supports()' answer for
// this view's spec with that base, on this backend, with these inputs.
TEST(EditorViews, EachEntryIsSupportsAnswerForThisView) {
    for (const ViewBackend backend : {ViewBackend::cpu, ViewBackend::gl}) {
        for (const bool has_sdf : {false, true}) {
            const std::vector<ModeEntry> menu = mode_menu(ViewState{}, backend, inputs(has_sdf));
            ASSERT_EQ(menu.size(), kViewBaseCount);
            for (const ModeEntry& e : menu) {
                ViewSpec spec;
                spec.base = e.base;
                const spade::render::ViewSupport want = spade::render::supports(spec, backend, inputs(has_sdf));
                EXPECT_EQ(e.drawable, want.drawable) << e.name << " backend " << static_cast<int>(backend)
                                                     << " sdf " << has_sdf;
                EXPECT_EQ(e.why_not, want.why_not) << e.name;
            }
        }
    }
}

TEST(EditorViews, GlDrawsTheRasterModesAndRefusesTheExactSdfWithAReason) {
    const std::vector<ModeEntry> menu = mode_menu(ViewState{}, ViewBackend::gl, inputs(true));
    ASSERT_EQ(menu.size(), kViewBaseCount);
    EXPECT_TRUE(menu[static_cast<uint32_t>(ViewBase::shaded)].drawable);
    EXPECT_TRUE(menu[static_cast<uint32_t>(ViewBase::wireframe)].drawable);
    EXPECT_TRUE(menu[static_cast<uint32_t>(ViewBase::velocity)].drawable);
    const ModeEntry& sdf = menu[static_cast<uint32_t>(ViewBase::exact_sdf)];
    EXPECT_FALSE(sdf.drawable);
    EXPECT_FALSE(sdf.why_not.empty());
}

TEST(EditorViews, TheCurrentModeIsMarkedAndOnlyIt) {
    ViewState v;
    v.spec.base = ViewBase::wireframe;
    const std::vector<ModeEntry> menu = mode_menu(v, ViewBackend::cpu, inputs(true));
    int current = 0;
    for (const ModeEntry& e : menu) {
        if (e.current) {
            ++current;
            EXPECT_EQ(e.base, ViewBase::wireframe);
        }
    }
    EXPECT_EQ(current, 1);
}

TEST(EditorViews, ChoosingADrawableModeSwitchesTheViewAndClearsTheRefusal) {
    ViewState v;
    v.refusal = "an earlier refusal";
    v.legends.push_back(spade::render::ViewLegend{});
    const spade::Result<void> r = choose_mode(v, ViewBase::wireframe, ViewBackend::cpu, inputs(true));
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(v.spec.base, ViewBase::wireframe);
    EXPECT_TRUE(v.refusal.empty());
    EXPECT_TRUE(v.legends.empty());  // the next draw fills them
}

TEST(EditorViews, ChoosingAModeTheViewCannotDrawIsRefusedInSupportsWords) {
    ViewState v;
    v.spec.base = ViewBase::velocity;
    const spade::Result<void> r = choose_mode(v, ViewBase::exact_sdf, ViewBackend::gl, inputs(true));
    ASSERT_FALSE(r.has_value());
    ViewSpec want;
    want.base = ViewBase::exact_sdf;
    const std::string why = spade::render::supports(want, ViewBackend::gl, inputs(true)).why_not;
    ASSERT_FALSE(why.empty());
    EXPECT_NE(r.error().context.find(why), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("exact_sdf"), std::string::npos) << r.error().context;
    EXPECT_EQ(v.spec.base, ViewBase::velocity);  // it keeps what it draws
    EXPECT_EQ(v.refusal, r.error().context);     // and the panel shows why
}

TEST(EditorViews, ApplyingAViewSetsTheRenderModeAndItsLegends) {
    struct Case {
        ViewBase base;
        spade::render::DrawMode mode;
    };
    for (const Case c : {Case{ViewBase::shaded, spade::render::DrawMode::shaded},
                         Case{ViewBase::wireframe, spade::render::DrawMode::wireframe},
                         Case{ViewBase::velocity, spade::render::DrawMode::velocity}}) {
        ViewState v;
        v.spec.base = c.base;
        spade::render::RenderScene scene;
        spade::render::RenderOptions options;
        spade::render::ViewCache cache;
        const spade::Result<void> r = apply_to(v, nullptr, ViewBackend::gl, inputs(false), scene, options, cache);
        ASSERT_TRUE(r.has_value()) << r.error().context;
        EXPECT_EQ(options.mode, c.mode) << spade::render::view_base_name(c.base);
        EXPECT_TRUE(v.legends.empty());  // no layers, no legends
        EXPECT_TRUE(v.refusal.empty());
    }
}

// The backend or the inputs can change under a chosen mode: the exact SDF on
// the CPU, then the GPU path comes up. The view draws shaded with the reason
// shown rather than blank, and returns to the chosen mode when it can.
TEST(EditorViews, AModeThatStopsBeingDrawableDrawsShadedWithTheReason) {
    ViewState v;
    ASSERT_TRUE(choose_mode(v, ViewBase::exact_sdf, ViewBackend::cpu, inputs(true)).has_value());
    spade::WorldDesc world;
    spade::render::RenderScene scene;
    ASSERT_NO_FATAL_FAILURE(ground_scene(world, scene));
    spade::render::RenderOptions options;
    spade::render::ViewCache cache;

    ASSERT_TRUE(apply_to(v, nullptr, ViewBackend::gl, inputs(true), scene, options, cache).has_value());
    EXPECT_EQ(options.mode, spade::render::DrawMode::shaded);
    EXPECT_EQ(v.spec.base, ViewBase::exact_sdf);  // the choice is kept
    ViewSpec want;
    want.base = ViewBase::exact_sdf;
    const std::string why = spade::render::supports(want, ViewBackend::gl, inputs(true)).why_not;
    EXPECT_NE(v.refusal.find(why), std::string::npos) << v.refusal;

    const spade::Result<void> back = apply_to(v, nullptr, ViewBackend::cpu, inputs(true), scene, options, cache);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(options.mode, spade::render::DrawMode::raymarch);
    EXPECT_TRUE(v.refusal.empty());
}

// The inputs a view is judged with come from the scene the way apply_view()
// reads them, so the menu and the draw cannot disagree: an SDF only when the
// scene has a non-empty one, walls when any item is a flow wall, a volume when
// one is published.
TEST(EditorViews, TheInputsComeFromTheSceneAsApplyViewReadsThem) {
    spade::render::RenderScene bare;
    const ViewInputs none = inputs_of(bare, nullptr, true);
    EXPECT_FALSE(none.has_sdf);
    EXPECT_FALSE(none.has_walls);
    EXPECT_FALSE(none.has_volume);
    EXPECT_TRUE(none.gl_flow_ready);

    const spade::SdfProgram empty;
    bare.sdf = &empty;
    EXPECT_FALSE(inputs_of(bare, nullptr, false).has_sdf);  // apply_view refuses an empty program
    EXPECT_FALSE(inputs_of(bare, nullptr, false).gl_flow_ready);

    spade::WorldDesc world;
    spade::render::RenderScene ground;
    ASSERT_NO_FATAL_FAILURE(ground_scene(world, ground));
    EXPECT_TRUE(inputs_of(ground, nullptr, true).has_sdf);

    spade::render::DrawItem wall;
    wall.flow_wall = true;
    ground.dynamics.push_back(wall);
    EXPECT_TRUE(inputs_of(ground, nullptr, true).has_walls);

    const spade::FlowVolume volume;
    EXPECT_TRUE(inputs_of(ground, &volume, true).has_volume);
}

// ===========================================================================
// Rendering's sensor bases (ee79a68): depth and normals. The menu offers
// them where supports() says they draw; a depth view is ranged from the
// scene's size; a flow layer on a sensor base is refused in Rendering's
// words; and the view carries its base's legend for the panel.
// ===========================================================================

TEST(EditorViews, TheMenuOffersTheSensorBasesWhereTheyDraw) {
    for (const ViewBackend backend : {ViewBackend::cpu, ViewBackend::gl}) {
        const std::vector<ModeEntry> menu = mode_menu(ViewState{}, backend, inputs(true));
        for (const ViewBase base : {ViewBase::depth, ViewBase::normals}) {
            ASSERT_LT(static_cast<uint32_t>(base), menu.size());
            const ModeEntry& e = menu[static_cast<uint32_t>(base)];
            EXPECT_EQ(e.name, spade::render::view_base_name(base));
            EXPECT_TRUE(e.drawable) << e.name << " on backend " << static_cast<int>(backend) << ": " << e.why_not;
        }
    }
}

// near 0.1 m; far twice the diagonal of the box around what the scene draws,
// never under 1 m.
TEST(EditorViews, ADepthViewIsRangedFromTheSceneSize) {
    spade::render::RenderScene scene;
    spade::render::DrawItem a;
    a.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(-3.0f, 0.0f, 0.0f));
    spade::render::DrawItem b;
    b.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(3.0f, 4.0f, 0.0f));
    scene.statics.push_back(a);
    scene.dynamics.push_back(b);
    const spade::render::DepthRange r = depth_range_for(scene);
    EXPECT_FLOAT_EQ(r.near_m, 0.1f);
    EXPECT_NEAR(r.far_m, 2.0f * std::sqrt(36.0f + 16.0f), 1e-4f);

    const spade::render::DepthRange tiny = depth_range_for(spade::render::RenderScene{});
    EXPECT_FLOAT_EQ(tiny.far_m, 1.0f);
}

TEST(EditorViews, AFlowLayerOnASensorBaseIsRefusedInRenderingsWords) {
    ViewState v;
    v.spec.layers.push_back(spade::render::FlowViewSpec{});
    ViewInputs in = inputs(true);
    in.has_volume = true;
    const spade::Result<void> r = choose_mode(v, ViewBase::depth, ViewBackend::cpu, in);
    ASSERT_FALSE(r.has_value());
    ViewSpec want = v.spec;
    want.base = ViewBase::depth;
    const std::string why = spade::render::supports(want, ViewBackend::cpu, in).why_not;
    ASSERT_FALSE(why.empty());
    EXPECT_NE(r.error().context.find(why), std::string::npos) << r.error().context;
    EXPECT_EQ(v.spec.base, ViewBase::shaded);  // it keeps what it draws
}

TEST(EditorViews, AnAppliedViewCarriesItsBaseLegendAndItsDepthRange) {
    ViewState v;
    ASSERT_TRUE(choose_mode(v, ViewBase::depth, ViewBackend::cpu, inputs(false)).has_value());
    spade::render::RenderScene scene;
    spade::render::DrawItem far_item;
    far_item.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(10.0f, 0.0f, 0.0f));
    scene.statics.push_back(far_item);
    scene.statics.push_back(spade::render::DrawItem{});
    spade::render::RenderOptions options;
    spade::render::ViewCache cache;
    const spade::Result<void> r = apply_to(v, nullptr, ViewBackend::cpu, inputs(false), scene, options, cache);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(v.spec.depth.far_m, depth_range_for(scene).far_m);
    const spade::render::BaseLegend want = spade::render::base_legend(v.spec, scene);
    EXPECT_EQ(v.base_legend.base, ViewBase::depth);
    EXPECT_EQ(v.base_legend.quantity, want.quantity);
    EXPECT_EQ(v.base_legend.unit, want.unit);
    EXPECT_EQ(v.base_legend.range.max, want.range.max);
}
