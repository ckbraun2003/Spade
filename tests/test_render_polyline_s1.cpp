// ---------------------------------------------------------------------------
// test_render_polyline_s1.cpp -- D182 S1 (KAT's drawing spec, P1; the lead approved the header, 2026-11-18): a polyline's per-vertex alpha, its shading
// (as_before / gradient / flat_mean), its caps (round / flat), depth_test, aa and slot mask, and RenderOptions' slot and ui_scale, on the CPU renderer,
// the reference GL is held to. Everything here is OFF by default and byte for byte the old drawing: the defaults are tested first.
// The geometry is test_render_flow_line_style.cpp's: a 33 x 33 frame, the camera 5 m back on the axis, a line along x at y = 0 on row 16's centre
// line, so a pixel row's distance from it is a whole number of pixels, and x from -1 to 1 spans columns 10.78 to 22.22.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/flow_lines.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"

namespace {

using spade::render::Camera;
using spade::render::FlowLineStyle;
using spade::render::FlowPolyline;
using spade::render::LineCap;
using spade::render::LineShading;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

constexpr uint32_t kW = 33, kH = 33;
constexpr float kLevel = 200.0f / 255.0f;  // a channel that is exactly byte 200

struct Frame {
    std::vector<uint8_t> px;
    [[nodiscard]] glm::ivec3 rgb(uint32_t x, uint32_t y) const {
        const size_t i = (static_cast<size_t>(y) * kW + x) * 4u;
        return glm::ivec3(px[i + 2], px[i + 1], px[i + 0]);
    }
};

[[nodiscard]] RenderScene black_scene() {
    RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    return scene;
}

[[nodiscard]] Frame draw(const RenderScene& scene, RenderOptions options = {}, spade::Result<void>* result = nullptr) {
    Frame f;
    f.px.assign(static_cast<size_t>(kW) * kH * 4u, 0xABu);
    RenderTarget target{.pixels = std::span<uint8_t>(f.px), .width = kW, .height = kH, .stride = kW * 4u};
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    options.overlays = false;
    const spade::Result<void> r = spade::render::render(scene, camera, options, target);
    if (result != nullptr) {
        *result = r;
    } else {
        EXPECT_TRUE(r.has_value()) << r.error().context;
    }
    return f;
}

[[nodiscard]] FlowPolyline horizontal(glm::vec3 c0, glm::vec3 c1, float z = 0.0f) {
    return FlowPolyline{{glm::vec3(-1.0f, 0.0f, z), glm::vec3(1.0f, 0.0f, z)}, {c0, c1}};
}

[[nodiscard]] RenderScene with_lines(float width, std::vector<FlowPolyline> lines) {
    RenderScene scene = black_scene();
    scene.flow_line_style = FlowLineStyle{width, 0.0f, 0.35f};
    scene.flow_polylines = std::move(lines);
    return scene;
}

constexpr glm::vec3 kRed(kLevel, 0.0f, 0.0f);
constexpr glm::vec3 kGreen(0.0f, kLevel, 0.0f);
constexpr glm::vec3 kBlack(0.0f, 0.0f, 0.0f);

}  // namespace

TEST(RenderPolylineS1, EveryNewFieldDefaultsToTheOldDrawing) {
    const FlowPolyline line;
    EXPECT_EQ(line.shading, LineShading::as_before);
    EXPECT_EQ(line.cap, LineCap::round);
    EXPECT_TRUE(line.depth_test);
    EXPECT_TRUE(line.aa);
    EXPECT_EQ(line.slots, 0xFFFFFFFFu) << "every slot, as lines were drawn before";
    const RenderOptions options;
    EXPECT_EQ(options.slot, 0u);
    EXPECT_FLOAT_EQ(options.ui_scale, 1.0f);
    EXPECT_EQ(spade::render::kMaxFlowLineVertices, 262144u) << "KAT's host total";
}

// KAT's solidStrokeIsTheGradientsMidColour: a stroke of one colour is the gradient's mid colour, and a gradient runs from the one end's colour to the
// other's along the segment. Colours red (200) to blue (200), width 3 (the middle row is covered in full); the pixel at column c has t = (c + 0.5 -
// 10.784) / 11.432 along the segment.
TEST(RenderPolylineS1, AGradientRunsFromOneEndsColourToTheOthersAndTheMidPointIsTheMean) {
    FlowPolyline line = horizontal(kRed, glm::vec3(0.0f, 0.0f, kLevel));
    line.shading = LineShading::gradient;
    const Frame got = draw(with_lines(3.0f, {line}));
    const glm::ivec3 mid = got.rgb(16, 16);
    EXPECT_NEAR(mid.r, 100, 3);
    EXPECT_NEAR(mid.b, 100, 3);
    const glm::ivec3 left = got.rgb(12, 16);  // t = 0.150
    EXPECT_NEAR(left.r, 170, 3);
    EXPECT_NEAR(left.b, 30, 3);
    const glm::ivec3 right = got.rgb(21, 16);  // t = 0.937
    EXPECT_NEAR(right.r, 12, 3);
    EXPECT_NEAR(right.b, 188, 3);
    // as_before is today's rule: the mean of the two ends, the same everywhere along the segment
    FlowPolyline before = horizontal(kRed, glm::vec3(0.0f, 0.0f, kLevel));
    const Frame mean = draw(with_lines(3.0f, {before}));
    EXPECT_EQ(mean.rgb(12, 16), mean.rgb(21, 16));
    EXPECT_NEAR(mean.rgb(12, 16).r, 100, 1);
    // flat_mean is the same mean in whole 8-bit channels, (a + b + 1) / 2
    FlowPolyline flat = horizontal(kRed, glm::vec3(0.0f, 0.0f, kLevel));
    flat.shading = LineShading::flat_mean;
    const Frame fm = draw(with_lines(3.0f, {flat}));
    EXPECT_EQ(fm.rgb(12, 16), glm::ivec3(100, 0, 100)) << "(200 + 0 + 1) / 2 = 100 in whole channels";
    EXPECT_EQ(fm.rgb(12, 16), fm.rgb(21, 16));
}

// KAT's paintedTrailIsAGradientThatFades: alpha runs from the tail (0.08) to the head (1) along each segment; the CPU draws alphas now (it refused them).
TEST(RenderPolylineS1, ATrailFadesFromItsTailToItsHeadAndTheCpuDrawsAlphas) {
    FlowPolyline line = horizontal(kRed, kRed);
    line.alphas = {0.08f, 1.0f};
    line.shading = LineShading::gradient;
    const Frame got = draw(with_lines(3.0f, {line}));
    int last = -1;
    for (uint32_t c = 11; c <= 22; ++c) {
        const int r = got.rgb(c, 16).r;
        EXPECT_GE(r, last) << "brighter toward the head, column " << c;
        last = r;
    }
    EXPECT_LT(got.rgb(11, 16).r, 60) << "near the tail it is faint";
    EXPECT_GT(got.rgb(22, 16).r, 170) << "at the head it is the colour";
    // as_before with alphas: the colour is the mean (one colour here) and the alpha is interpolated: the same fade
    FlowPolyline before = horizontal(kRed, kRed);
    before.alphas = {0.08f, 1.0f};
    const Frame b = draw(with_lines(3.0f, {before}));
    EXPECT_NEAR(b.rgb(16, 16).r, got.rgb(16, 16).r, 2);
    // flat_mean takes the mean alpha in whole channels, (20 + 255 + 1) / 2 = 138: one level along the whole segment
    FlowPolyline flat = horizontal(kRed, kRed);
    flat.alphas = {0.08f, 1.0f};
    flat.shading = LineShading::flat_mean;
    const Frame f = draw(with_lines(3.0f, {flat}));
    EXPECT_EQ(f.rgb(12, 16), f.rgb(21, 16));
    EXPECT_NEAR(f.rgb(16, 16).r, 200.0f * 138.0f / 255.0f, 2.0f);
}

TEST(RenderPolylineS1, AFlatCapEndsSquareAtTheSegmentsEndWhereARoundOneReachesHalfAWidthPast) {
    FlowPolyline round = horizontal(kRed, kRed);
    const Frame r = draw(with_lines(8.0f, {round}));
    EXPECT_GT(r.rgb(9, 16).r, 0) << "the round cap reaches 4 px past the end at column 10.78";
    FlowPolyline flat = horizontal(kRed, kRed);
    flat.cap = LineCap::flat;
    const Frame f = draw(with_lines(8.0f, {flat}));
    EXPECT_EQ(f.rgb(9, 16).r, 0) << "the flat cap draws nothing past the end";
    EXPECT_EQ(f.rgb(11, 16).r, 200) << "and the stroke is full up to it";
    EXPECT_EQ(f.rgb(16, 13).r, 200) << "its width is the width: 4 px to the side of the centre line";
    EXPECT_EQ(f.rgb(16, 11).r, 0);
}

// depth_test false: drawn over the frame, in submission order, reading no depth and writing none.
TEST(RenderPolylineS1, DepthTestFalseDrawsOverNearerGeometryAndWritesNoDepth) {
    FlowPolyline near_green = horizontal(kGreen, kGreen, 1.0f);
    FlowPolyline far_red = horizontal(kRed, kRed, -1.0f);
    const Frame tested = draw(with_lines(3.0f, {near_green, far_red}));
    EXPECT_EQ(tested.rgb(16, 16), glm::ivec3(0, 200, 0)) << "default: the nearer green wins";
    far_red.depth_test = false;
    const Frame over = draw(with_lines(3.0f, {near_green, far_red}));
    EXPECT_EQ(over.rgb(16, 16), glm::ivec3(200, 0, 0)) << "no depth test: the red is painted over, submitted later";
    // and it writes no depth: a farther one-pixel line (drawn after the capsules, depth-tested) still shows over the untested near capsule
    FlowPolyline near_green_untested = horizontal(kGreen, kGreen, 1.0f);
    near_green_untested.width_px = 3.0f;
    near_green_untested.depth_test = false;
    FlowPolyline far_red_thin = horizontal(kRed, kRed, -1.0f);  // width 0 and the scene's width 0: the one-pixel line
    RenderScene scene = black_scene();
    scene.flow_polylines = {near_green_untested, far_red_thin};
    const Frame none = draw(scene);
    EXPECT_EQ(none.rgb(16, 16), glm::ivec3(200, 0, 0)) << "the untested green left no depth for the red to lose to";
    near_green_untested.depth_test = true;
    scene.flow_polylines = {near_green_untested, far_red_thin};
    EXPECT_EQ(draw(scene).rgb(16, 16), glm::ivec3(0, 200, 0)) << "a tested near green does write depth, and the red loses";
}

TEST(RenderPolylineS1, AaFalseIsAHardEdgeAtHalfCoverage) {
    FlowPolyline soft = horizontal(kRed, kRed);
    const Frame a = draw(with_lines(2.0f, {soft}));
    EXPECT_EQ(a.rgb(16, 15).r, 100) << "anti-aliased: coverage 0.5 at 1 px";
    FlowPolyline hard = horizontal(kRed, kRed);
    hard.aa = false;
    const Frame h = draw(with_lines(2.0f, {hard}));
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const int r = h.rgb(x, y).r;
            EXPECT_TRUE(r == 0 || r == 200) << "a hard edge has no partial coverage: (" << x << "," << y << ") = " << r;
        }
    }
    EXPECT_EQ(h.rgb(16, 15).r, 200) << "coverage 0.5 is in";
    EXPECT_EQ(h.rgb(16, 14).r, 0);
}

TEST(RenderPolylineS1, ASlotMaskDrawsAPolylineOnlyForTheSlotsItNames) {
    FlowPolyline line = horizontal(kRed, kRed);
    line.slots = 0b10u;  // slot 1 only
    RenderOptions slot0;
    slot0.slot = 0u;
    RenderOptions slot1;
    slot1.slot = 1u;
    EXPECT_EQ(draw(with_lines(3.0f, {line}), slot0).rgb(16, 16), glm::ivec3(0, 0, 0)) << "slot 0 is not in the mask: nothing drawn";
    EXPECT_EQ(draw(with_lines(3.0f, {line}), slot1).rgb(16, 16), glm::ivec3(200, 0, 0));
    FlowPolyline all = horizontal(kRed, kRed);
    EXPECT_EQ(draw(with_lines(3.0f, {all}), slot0).rgb(16, 16), glm::ivec3(200, 0, 0)) << "the default mask is every slot";
    // the one-pixel line has a mask too
    FlowPolyline thin = horizontal(kRed, kRed);
    thin.slots = 0b10u;
    RenderScene scene = black_scene();
    scene.flow_polylines = {thin};
    EXPECT_EQ(draw(scene, slot0).rgb(16, 16), glm::ivec3(0, 0, 0));
    EXPECT_EQ(draw(scene, slot1).rgb(16, 16), glm::ivec3(200, 0, 0));
    RenderOptions bad;
    bad.slot = 32u;
    spade::Result<void> result;
    (void)draw(with_lines(3.0f, {all}), bad, &result);
    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().context.find("slot"), std::string::npos) << result.error().context;
}

TEST(RenderPolylineS1, UiScaleMultipliesAPolylinesWidthAndAnOutOfRangeOrTooWideOneIsRefusedByName) {
    FlowPolyline line = horizontal(kRed, kRed);
    const Frame one = draw(with_lines(3.0f, {line}));
    EXPECT_EQ(one.rgb(16, 14).r, 0) << "width 3: coverage 0 at 2 px";
    RenderOptions two;
    two.ui_scale = 2.0f;
    const Frame doubled = draw(with_lines(3.0f, {line}), two);
    EXPECT_EQ(doubled.rgb(16, 14).r, 200) << "width 3 at ui_scale 2 is 6: covered at 2 px";
    EXPECT_EQ(doubled.rgb(16, 12).r, 0);
    spade::Result<void> result;
    RenderOptions low;
    low.ui_scale = 0.1f;
    (void)draw(with_lines(3.0f, {line}), low, &result);
    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().context.find("ui_scale"), std::string::npos) << result.error().context;
    RenderOptions high;
    high.ui_scale = 9.0f;
    (void)draw(with_lines(3.0f, {line}), high, &result);
    EXPECT_FALSE(result.has_value());
    (void)draw(with_lines(40.0f, {line}), two, &result);  // 80 px scaled
    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().context.find("width"), std::string::npos) << result.error().context;
    (void)draw(with_lines(32.0f, {line}), two, &result);  // 64 px scaled: the limit, allowed
    EXPECT_TRUE(result.has_value());
}

TEST(RenderPolylineS1, OneSceneDrawsAtMostTheCapsTotalOfVerticesAndRefusesTheRestByName) {
    FlowPolyline big;
    big.points.assign(spade::render::kMaxFlowLineVertices, glm::vec3(0.0f));
    big.colours.assign(spade::render::kMaxFlowLineVertices, kRed);
    std::vector<FlowPolyline> at_cap{big};
    EXPECT_TRUE(spade::render::validate_flow_line_budget(at_cap).has_value()) << "exactly the cap is drawn";
    FlowPolyline one_more;
    one_more.points = {glm::vec3(0.0f), glm::vec3(0.0f)};
    one_more.colours = {kRed, kRed};
    std::vector<FlowPolyline> over{big, one_more};
    const auto refused = spade::render::validate_flow_line_budget(over);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("262144"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("vertices"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find(std::to_string(spade::render::kMaxFlowLineVertices + 2u)), std::string::npos) << "it names the count";
    RenderScene scene = black_scene();
    scene.flow_polylines = over;
    spade::Result<void> result;
    (void)draw(scene, RenderOptions{}, &result);
    ASSERT_FALSE(result.has_value()) << "render() refuses it before drawing";
    EXPECT_NE(result.error().context.find("262144"), std::string::npos);
}

TEST(RenderPolylineS1, OnlyAdditiveIsStillGlOnly) {
    FlowPolyline faded = horizontal(kRed, kRed);
    faded.alphas = {0.2f, 0.9f};
    const FlowLineStyle width3{3.0f, 0.0f, 0.35f};
    EXPECT_EQ(spade::render::flow_lines_gl_only_reason(std::span<const FlowPolyline>(&faded, 1), width3), "") << "alphas are drawn by both now";
    FlowLineStyle additive = width3;
    additive.additive = true;
    EXPECT_NE(spade::render::flow_lines_gl_only_reason(std::span<const FlowPolyline>(&faded, 1), additive).find("additive"), std::string::npos);
}
