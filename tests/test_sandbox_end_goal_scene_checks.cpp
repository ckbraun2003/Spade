// The end-goal smoke's checks, batch 2 (N20, docs/design/test-docs/
// 2026-10-19-end-goal-checks.md §1, §2, §3, §5, §6, §7, §8): the library, the
// identities a copy keeps, an import's name, the region's bookkeeping, the
// views, save and reopen, and the measuring helpers for a placed object.
// Each check fails on a stub of the fault it exists to catch and passes on a
// stub of the right state.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "../sandbox/end_goal_checks.hpp"
#include "core/flow_volume.hpp"
#include "render/scene.hpp"
#include "render/view_mode.hpp"
#include "world/sdf.hpp"

namespace eg = spade::sandbox::end_goal;

namespace {

[[nodiscard]] bool has(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

[[nodiscard]] bool judged_fail(const eg::Verdict& v) {
    return !v.ok && !has(v.detail, "absent:") && !has(v.detail, "mis-staged:") && !has(v.detail, "under-resolved:");
}

// ---------------------------------------------------------------------------
// Images
// ---------------------------------------------------------------------------

constexpr eg::Rgb kClear{20, 24, 30};

// An RGBA8 image of `w` x `h` in `fill`, with the rectangle [x0, x1) x [y0, y1) in `ink`.
struct Image {
    uint32_t w = 0, h = 0;
    std::vector<uint8_t> rgba;
    [[nodiscard]] eg::PixelsView view() const { return eg::pixels(w, h, rgba); }
};

[[nodiscard]] Image image(uint32_t w, uint32_t h, eg::Rgb fill, eg::Rgb ink = {}, uint32_t x0 = 0, uint32_t y0 = 0,
                          uint32_t x1 = 0, uint32_t y1 = 0) {
    Image img{w, h, std::vector<uint8_t>(std::size_t{w} * h * 4u)};
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const bool in = x >= x0 && x < x1 && y >= y0 && y < y1;
            const eg::Rgb c = in ? ink : fill;
            uint8_t* p = img.rgba.data() + (std::size_t{y} * w + x) * 4u;
            p[0] = c[0];
            p[1] = c[1];
            p[2] = c[2];
            p[3] = 255;
        }
    }
    return img;
}

// 64 x 48 on the clear colour, with a 16 x 19 drawn rectangle: 304 pixels, 9.9%.
[[nodiscard]] Image drawn(eg::Rgb ink, uint32_t x0 = 10) { return image(64, 48, kClear, ink, x0, 10, x0 + 16, 29); }

// ---------------------------------------------------------------------------
// Library, identities, import, region
// ---------------------------------------------------------------------------

[[nodiscard]] std::vector<eg::LibraryEntry> index_of_nine() {
    return {{"reference quad", "drones", "drones/ref_quad.model.yaml"},
            {"box", "primitives", "objects/box.object.yaml"},
            {"sphere", "primitives", "objects/sphere.object.yaml"},
            {"cylinder", "primitives", "objects/cylinder.object.yaml"},
            {"capsule", "primitives", "objects/capsule.object.yaml"},
            {"gate", "obstacles", "objects/gate.object.yaml"},
            {"floor tile", "floors", "objects/floor_tile.object.yaml"},
            {"pole", "obstacles", "objects/pole.object.yaml"},
            {"wall", "obstacles", "objects/wall.object.yaml"}};
}

[[nodiscard]] eg::ImportShown good_import() {
    eg::ImportShown m;
    m.shown_name = "bench";
    m.mesh_name = "bench";
    m.file = "imports/park_bench.gltf";
    m.shown_source = "imports/park_bench.gltf";
    m.sidecar_loaded = true;
    m.gltf_vertices = m.sidecar_vertices = 1200;
    m.gltf_triangles = m.sidecar_triangles = 2000;
    return m;
}

[[nodiscard]] spade::render::ViewLegend speed_legend(float max) {
    spade::render::ViewLegend l;
    l.layer = spade::render::FlowView::streamlines;
    l.quantity = "speed |u|";
    l.unit = "m/s";
    l.range.min = 0.0f;
    l.range.max = max;
    return l;
}

// ---------------------------------------------------------------------------
// SDF programs, postfix
// ---------------------------------------------------------------------------

[[nodiscard]] spade::SdfNode leaf(spade::SdfPrim kind, glm::vec4 params = glm::vec4(0.5f, 0.5f, 0.5f, 0.0f)) {
    spade::SdfNode n;
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = params;
    return n;
}

[[nodiscard]] spade::SdfNode op(spade::SdfOp o) {
    spade::SdfNode n;
    n.op = static_cast<uint32_t>(o);
    return n;
}

[[nodiscard]] spade::SdfProgram program(std::vector<spade::SdfNode> nodes) {
    spade::SdfProgram p;
    p.nodes = std::move(nodes);
    p.transforms.push_back(spade::SdfTransform{});
    p.node_materials.assign(p.nodes.size(), 0u);
    return p;
}

// The ground: the plane y = 0, solid below.
[[nodiscard]] spade::SdfProgram ground() {
    return program({leaf(spade::SdfPrim::plane, glm::vec4(0.0f, 1.0f, 0.0f, 0.0f))});
}

// A 1 m box drawable whose bottom face sits at `bottom_y`.
[[nodiscard]] spade::render::RenderScene box_scene(float bottom_y) {
    spade::render::RenderScene scene;
    spade::render::MeshData mesh;
    for (const float x : {-0.5f, 0.5f}) {
        for (const float y : {-0.5f, 0.5f}) {
            for (const float z : {-0.5f, 0.5f}) mesh.positions.emplace_back(x, y, z);
        }
    }
    scene.meshes.push_back(mesh);
    spade::render::DrawItem item;
    item.mesh_index = 0;
    item.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(2.0f, bottom_y + 0.5f, -1.0f));
    scene.statics.push_back(item);
    return scene;
}

[[nodiscard]] eg::RoundTrip good_round_trip() {
    eg::RoundTrip t;
    t.saved = "scene_version: 4\nairflow:\n  - kind: follow\n";
    t.reemitted = t.saved;
    t.identities = {{"scene", 0x1111u, 0x1111u}, {"region", 0x2222u, 0x2222u}, {"propulsion", 0x3333u, 0x3333u}};
    t.steps_before = t.steps_after = eg::kRoundTripSteps;
    spade::BodyState b{};
    b.pos = glm::vec3(0.1f, 1.25f, -0.3f);
    b.vel = glm::vec3(0.0f, 0.42f, 0.0f);
    b.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    t.bodies_before = {b, b};
    t.bodies_after = t.bodies_before;
    return t;
}

}  // namespace

// ===========================================================================
// Measuring helpers
// ===========================================================================

TEST(SandboxEndGoalMeasure, CountsTheDrawablesAColliderDeclares) {
    using spade::SdfOp;
    using spade::SdfPrim;
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box)})), 1u);
    // A gate: a torus and two posts, under union only.
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::torus), leaf(SdfPrim::cylinder), op(SdfOp::union_),
                                              leaf(SdfPrim::cylinder), op(SdfOp::union_)})),
              3u);
    // A box with a hole: one CSG root, whatever it holds.
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box), leaf(SdfPrim::sphere), op(SdfOp::subtract)})), 1u);
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box), leaf(SdfPrim::sphere), leaf(SdfPrim::capsule),
                                              op(SdfOp::union_), op(SdfOp::intersect)})),
              1u);
    // A sphere beside a holed box: two.
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::sphere), leaf(SdfPrim::box), leaf(SdfPrim::sphere),
                                              op(SdfOp::smooth_union), op(SdfOp::union_)})),
              2u);
}

TEST(SandboxEndGoalMeasure, AMalformedColliderDeclaresNothing) {
    using spade::SdfOp;
    using spade::SdfPrim;
    EXPECT_EQ(eg::declared_drawables(program({op(SdfOp::union_)})), 0u);
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box), leaf(SdfPrim::box)})), 0u);
    EXPECT_EQ(eg::declared_drawables(program({})), 0u);
}

TEST(SandboxEndGoalMeasure, PosesEveryVertexOfTheDrawItems) {
    const spade::render::RenderScene scene = box_scene(0.0f);
    const auto points = eg::posed_vertices(scene, scene.statics);
    ASSERT_EQ(points.size(), 8u);
    EXPECT_FLOAT_EQ(points.front().x, 1.5f);
    EXPECT_FLOAT_EQ(points.front().y, 0.0f);
    EXPECT_FLOAT_EQ(points.front().z, -1.5f);
    EXPECT_FLOAT_EQ(points.back().y, 1.0f);
}

TEST(SandboxEndGoalMeasure, TheRestGapIsTheLeastDistanceToTheWorldWithoutTheObject) {
    const spade::SdfProgram world = ground();
    for (const float bottom : {0.0f, 0.004f, -0.01f}) {
        const spade::render::RenderScene scene = box_scene(bottom);
        const auto points = eg::posed_vertices(scene, scene.statics);
        EXPECT_NEAR(eg::rest_gap(world, points), bottom, 1e-6f) << bottom;
    }
    EXPECT_TRUE(std::isnan(eg::rest_gap(world, {})));
}

// ===========================================================================
// Images
// ===========================================================================

TEST(SandboxEndGoalImages, DiffersCountsPixelsPastEightLevels) {
    const Image base = image(10, 10, {100, 100, 100});
    const Image by8 = image(10, 10, {100, 100, 100}, {108, 100, 100}, 0, 0, 10, 10);
    const Image by9 = image(10, 10, {100, 100, 100}, {100, 91, 100}, 0, 0, 10, 10);
    EXPECT_EQ(eg::differ_fraction(base.view(), by8.view()), 0.0);
    EXPECT_EQ(eg::differ_fraction(base.view(), by9.view()), 1.0);
    const Image one = drawn({200, 180, 40});
    EXPECT_NEAR(eg::differ_fraction(one.view(), kClear), 304.0 / (64.0 * 48.0), 1e-12);
}

TEST(SandboxEndGoalImages, ARegionOfAFrameIsItsHalf) {
    // Left half clear, right half drawn.
    const Image frame = image(64, 48, kClear, {200, 30, 30}, 32, 0, 64, 48);
    const eg::PixelsView left = eg::region_of(frame.view(), 0, 0, 32, 48);
    const eg::PixelsView right = eg::region_of(frame.view(), 32, 0, 32, 48);
    EXPECT_EQ(eg::differ_fraction(left, kClear), 0.0);
    EXPECT_EQ(eg::differ_fraction(right, kClear), 1.0);
}

TEST(SandboxEndGoalImages, NamesASpecByItsBaseAndLayers) {
    spade::render::ViewSpec spec;
    spec.base = spade::render::ViewBase::wireframe;
    spec.layers.resize(2);
    spec.layers[0].kind = spade::render::FlowView::vortex_iso;
    spec.layers[1].kind = spade::render::FlowView::streamlines;
    EXPECT_EQ(eg::spec_name(spec), "wireframe+vortex_iso+streamlines");
    EXPECT_EQ(eg::spec_name(spade::render::ViewSpec{}), "shaded");
}

// ===========================================================================
// editor.library
// ===========================================================================

TEST(SandboxEndGoalLibrary, PassesThePanelListingTheIndexInOrder) {
    const auto index = index_of_nine();
    EXPECT_TRUE(eg::lists_index(index, index).ok);
}

TEST(SandboxEndGoalLibrary, FailsADroppedOrReorderedOrRenamedEntry) {
    const auto index = index_of_nine();
    auto dropped = index;
    dropped.pop_back();
    auto reordered = index;
    std::swap(reordered[2], reordered[3]);
    auto other_file = index;
    other_file[5].file = "objects/gate_v2.object.yaml";
    for (const auto* panel : {&dropped, &reordered, &other_file}) {
        const eg::Verdict v = eg::lists_index(index, *panel);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalLibrary, FailsAnIndexShortOfTheFirstEntries) {
    auto index = index_of_nine();
    index.pop_back();
    const eg::Verdict v = eg::lists_index(index, index);
    EXPECT_TRUE(judged_fail(v)) << v.detail;
}

TEST(SandboxEndGoalLibrary, PassesThumbnailsThatAreDrawn) {
    std::vector<Image> images;
    for (int i = 0; i < 9; ++i) images.push_back(drawn({200, static_cast<uint8_t>(20 * i), 40}));
    std::vector<eg::PixelsView> views;
    for (const Image& img : images) views.push_back(img.view());
    const eg::Verdict v = eg::thumbnails(views, 9);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalLibrary, FailsABlankOrMissingThumbnail) {
    std::vector<Image> images;
    for (int i = 0; i < 9; ++i) images.push_back(drawn({200, 120, 40}));
    images[4] = image(64, 48, {128, 128, 128});  // a placeholder: one colour
    std::vector<eg::PixelsView> views;
    for (const Image& img : images) views.push_back(img.view());
    const eg::Verdict blank = eg::thumbnails(views, 9);
    EXPECT_TRUE(judged_fail(blank)) << blank.detail;
    views[4] = eg::PixelsView{};
    const eg::Verdict missing = eg::thumbnails(views, 9);
    EXPECT_TRUE(judged_fail(missing)) << missing.detail;
    views.pop_back();
    const eg::Verdict short_by_one = eg::thumbnails(views, 9);
    EXPECT_TRUE(judged_fail(short_by_one)) << short_by_one.detail;
}

// ===========================================================================
// propulsion-kept (and every identity a copy keeps)
// ===========================================================================

TEST(SandboxEndGoalIdentities, PassesEveryIdentityKept) {
    const std::vector<eg::Identity> ids = {
        {"model", 0xA1u, 0xA1u}, {"parts", 0xB2u, 0xB2u}, {"propulsion", 0xC3u, 0xC3u}};
    const eg::Verdict v = eg::identities_kept("propulsion-kept", ids);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalIdentities, FailsAChainLostInTheCopyAndNamesIt) {
    const std::vector<eg::Identity> ids = {{"model", 0xA1u, 0xA1u}, {"parts", 0xB2u, 0xB2u}, {"propulsion", 0xC3u, 0u}};
    const eg::Verdict v = eg::identities_kept("propulsion-kept", ids);
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    EXPECT_TRUE(has(v.detail, "propulsion")) << v.detail;
}

TEST(SandboxEndGoalIdentities, IsAbsentWithAnUnsetOrNoIdentity) {
    const std::vector<eg::Identity> unset = {{"model", 0u, 0u}};
    const eg::Verdict a = eg::identities_kept("propulsion-kept", unset);
    EXPECT_FALSE(a.ok) << a.detail;
    EXPECT_TRUE(has(a.detail, "absent:")) << a.detail;
    const eg::Verdict b = eg::identities_kept("propulsion-kept", {});
    EXPECT_FALSE(b.ok) << b.detail;
    EXPECT_TRUE(has(b.detail, "absent:")) << b.detail;
}

// ===========================================================================
// import-named
// ===========================================================================

TEST(SandboxEndGoalImport, PassesAMeshNamedAsItsFileNamesIt) {
    const eg::Verdict v = eg::import_named(good_import());
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalImport, NamesAnUnnamedMeshByItsFilesStem) {
    eg::ImportShown m = good_import();
    m.mesh_name.clear();
    m.shown_name = "park_bench";
    const eg::Verdict v = eg::import_named(m);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalImport, FailsAWrongNameOrSourceOrSidecar) {
    eg::ImportShown wrong_name = good_import();
    wrong_name.shown_name = "Mesh";
    eg::ImportShown wrong_source = good_import();
    wrong_source.shown_source = "park_bench.gltf";
    eg::ImportShown no_sidecar = good_import();
    no_sidecar.sidecar_loaded = false;
    eg::ImportShown short_sidecar = good_import();
    short_sidecar.sidecar_triangles = 1999;
    for (const eg::ImportShown* m : {&wrong_name, &wrong_source, &no_sidecar, &short_sidecar}) {
        const eg::Verdict v = eg::import_named(*m);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

// ===========================================================================
// region-resolved, publishes, run-line-shown
// ===========================================================================

TEST(SandboxEndGoalRegion, PassesTheResolveShownNumberForNumber) {
    const eg::GridNumbers g{32, 32, 32, 0.025f, 3.048f};
    const eg::Verdict v = eg::region_resolved(g, g);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalRegion, FailsAShownGridThatIsNotTheResolve) {
    const eg::GridNumbers resolved{32, 32, 32, 0.025f, 3.048f};
    eg::GridNumbers shown = resolved;
    shown.dx = 0.0125f;
    const eg::Verdict v = eg::region_resolved(shown, resolved);
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    const eg::Verdict none = eg::region_resolved(shown, eg::GridNumbers{});
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "absent:")) << none.detail;
}

TEST(SandboxEndGoalRegion, ReadsAPublishedVolume) {
    spade::FlowVolume v;
    v.header.cell = 0.1f;
    v.header.nx = v.header.ny = v.header.nz = 2;
    v.v.assign(8u * spade::kFlowHalvesPerCell, 0u);
    v.s.assign(8u * spade::kFlowHalvesPerCell, 0u);
    v.generation = spade::next_flow_generation();
    const eg::Published p = eg::published(v);
    EXPECT_EQ(p.generation, v.generation);
    EXPECT_TRUE(p.valid);
    EXPECT_EQ(p.no_data, 0u);
    v.header.cell = 0.0f;
    EXPECT_FALSE(eg::published(v).valid);
}

TEST(SandboxEndGoalRegion, PassesANewValidVolumeEveryFrame) {
    const std::vector<eg::Published> frames = {{11, true, 0}, {12, true, 0}, {15, true, 0}};
    const eg::Verdict v = eg::publishes(frames);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalRegion, FailsARepeatedOrInvalidOrHoledVolume) {
    const std::vector<std::vector<eg::Published>> bad = {{{11, true, 0}, {11, true, 0}},
                                                         {{11, true, 0}, {12, false, 0}},
                                                         {{11, true, 0}, {12, true, 3}},
                                                         {{0, true, 0}}};
    for (const auto& frames : bad) {
        const eg::Verdict v = eg::publishes(frames);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
    const eg::Verdict none = eg::publishes({});
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "absent:")) << none.detail;
}

TEST(SandboxEndGoalRegion, TheRunLineShownIsLiveRunsExactly) {
    const std::string line = "actuator disc \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3 \xc2\xb7 t = 0.250 s";
    EXPECT_TRUE(eg::run_line_shown(line, line).ok);
    EXPECT_TRUE(judged_fail(eg::run_line_shown("actuator disc", line)));
    const eg::Verdict none = eg::run_line_shown(line, "");
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "absent:")) << none.detail;
}

// ===========================================================================
// editor.views
// ===========================================================================

TEST(SandboxEndGoalViews, PassesAModeDrawnAndChangedFromTheLast) {
    const Image shaded = drawn({200, 180, 40});
    const Image wire = drawn({240, 240, 240}, 30);
    eg::ModeShown m;
    m.kind = "wireframe";
    m.supported = m.applied = m.drawn = true;
    m.frame = wire.view();
    m.previous = shaded.view();
    m.clear = kClear;
    const eg::Verdict v = eg::mode_drawn(m);
    EXPECT_TRUE(v.ok) << v.detail;
    m.first = true;
    m.previous = {};
    const eg::Verdict first = eg::mode_drawn(m);
    EXPECT_TRUE(first.ok) << first.detail;
}

TEST(SandboxEndGoalViews, FailsABlankOrUnchangedOrUnhonouredMode) {
    const Image shaded = drawn({200, 180, 40});
    const Image blank = image(64, 48, kClear);
    eg::ModeShown base;
    base.kind = "vortices";
    base.supported = base.applied = base.drawn = true;
    base.clear = kClear;
    base.previous = shaded.view();

    eg::ModeShown is_blank = base;
    is_blank.frame = blank.view();
    eg::ModeShown unchanged = base;
    unchanged.frame = shaded.view();
    const Image other = drawn({10, 200, 90}, 30);
    eg::ModeShown unhonoured = base;
    unhonoured.frame = other.view();
    unhonoured.unhonoured = "mode";
    eg::ModeShown refused_apply = base;
    refused_apply.frame = other.view();
    refused_apply.applied = false;
    for (const eg::ModeShown* m : {&is_blank, &unchanged, &unhonoured, &refused_apply}) {
        const eg::Verdict v = eg::mode_drawn(*m);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalViews, ARefusedKindShowsItsWhyNotVerbatim) {
    eg::ModeShown m;
    m.kind = "exact SDF";
    m.supported = false;
    m.why_not = "the exact SDF is drawn on the CPU only";
    m.shown_text = m.why_not;
    const eg::Verdict v = eg::mode_drawn(m);
    EXPECT_TRUE(v.ok) << v.detail;
    m.shown_text.clear();
    const eg::Verdict blank = eg::mode_drawn(m);
    EXPECT_TRUE(judged_fail(blank)) << blank.detail;
    m.why_not.clear();
    const eg::Verdict none = eg::mode_drawn(m);
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "mis-staged:")) << none.detail;
}

TEST(SandboxEndGoalViews, PassesTheLegendsApplyViewReturned) {
    eg::LegendCheck c;
    c.returned = {speed_legend(12.0f)};
    c.shown = c.returned;
    c.region_running = true;
    const eg::Verdict v = eg::legend_shown(c);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalViews, FailsAnotherLegendOrAStillField) {
    eg::LegendCheck other;
    other.returned = {speed_legend(12.0f)};
    other.shown = other.returned;
    other.shown[0].unit = "km/h";
    eg::LegendCheck still;
    still.returned = {speed_legend(0.3f)};
    still.shown = still.returned;
    still.region_running = true;
    eg::LegendCheck flat;
    flat.returned = {speed_legend(0.0f)};
    flat.shown = flat.returned;
    flat.region_running = true;
    eg::LegendCheck missing;
    missing.returned = {speed_legend(12.0f), speed_legend(12.0f)};
    missing.shown = {speed_legend(12.0f)};
    for (const eg::LegendCheck* c : {&other, &still, &flat, &missing}) {
        const eg::Verdict v = eg::legend_shown(*c);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalViews, PassesTwoDifferentViewsSideBySide) {
    const Image left = drawn({200, 180, 40});
    const Image right = drawn({240, 240, 240}, 30);
    const eg::Verdict v = eg::two_views("shaded", "wireframe+vortex_iso", left.view(), right.view(), kClear);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalViews, FailsOneSpecTwiceOrABlankOrAMatchingPair) {
    const Image a = drawn({200, 180, 40});
    const Image b = drawn({240, 240, 240}, 30);
    const Image blank = image(64, 48, kClear);
    const eg::Verdict same_spec = eg::two_views("shaded", "shaded", a.view(), b.view(), kClear);
    const eg::Verdict one_blank = eg::two_views("shaded", "wireframe", a.view(), blank.view(), kClear);
    const eg::Verdict matching = eg::two_views("shaded", "wireframe", a.view(), a.view(), kClear);
    for (const eg::Verdict* v : {&same_spec, &one_blank, &matching}) EXPECT_TRUE(judged_fail(*v)) << v->detail;
}

// ===========================================================================
// editor.save_reopen
// ===========================================================================

TEST(SandboxEndGoalRoundTrip, PassesAnExactRoundTrip) {
    const eg::Verdict v = eg::round_trip(good_round_trip());
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalRoundTrip, FailsAByteOrAnIdentityOrABodyThatDiffers) {
    eg::RoundTrip byte = good_round_trip();
    byte.reemitted[17] = 'X';
    eg::RoundTrip identity = good_round_trip();
    identity.identities[1].got = 0x2223u;
    eg::RoundTrip ulp = good_round_trip();
    ulp.bodies_after[1].vel.y = std::nextafter(ulp.bodies_after[1].vel.y, 1.0f);
    eg::RoundTrip zero = good_round_trip();
    zero.bodies_before[0].vel.x = 0.0f;
    zero.bodies_after[0].vel.x = -0.0f;  // equal by ==, not bit for bit
    for (const eg::RoundTrip* t : {&byte, &identity, &ulp, &zero}) {
        const eg::Verdict v = eg::round_trip(*t);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalRoundTrip, IsMisStagedOrAbsentWhenTheRunsDoNotMatch) {
    eg::RoundTrip short_run = good_round_trip();
    short_run.steps_after = 59;
    const eg::Verdict a = eg::round_trip(short_run);
    EXPECT_FALSE(a.ok) << a.detail;
    EXPECT_TRUE(has(a.detail, "mis-staged:")) << a.detail;
    eg::RoundTrip no_bodies = good_round_trip();
    no_bodies.bodies_before.clear();
    no_bodies.bodies_after.clear();
    const eg::Verdict b = eg::round_trip(no_bodies);
    EXPECT_FALSE(b.ok) << b.detail;
    EXPECT_TRUE(has(b.detail, "absent:")) << b.detail;
}
