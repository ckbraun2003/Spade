// ---------------------------------------------------------------------------
// test_render_flow_line_style.cpp -- render/flow_lines.hpp's FlowLineStyle on the CPU renderer, the reference GL's
// flow-line program is held to (the lead, 2026-11-10): streamlines drawn as capsules of a width in pixels, with
// coverage anti-aliasing and a glow, off by default. The geometry is chosen so the numbers are exact: a 33 x 33
// frame, the camera 5 m back on the axis, a line along x at y = 0 lies on row 16's centre line (its centre y is
// 16.5), so a pixel row's distance from it is a whole number of pixels.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
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
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

constexpr uint32_t kW = 33, kH = 33;
constexpr float kLevel = 200.0f / 255.0f;  // a line channel that is exactly byte 200

struct Frame {
    std::vector<uint8_t> px;
    [[nodiscard]] glm::ivec3 rgb(uint32_t x, uint32_t y) const {
        const size_t i = (static_cast<size_t>(y) * kW + x) * 4u;
        return glm::ivec3(px[i + 2], px[i + 1], px[i + 0]);
    }
};

// A pure black background, so the bytes are the blend's alone.
[[nodiscard]] RenderScene black_scene() {
    RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    return scene;
}

[[nodiscard]] Frame draw(const RenderScene& scene, spade::Result<void>* result = nullptr) {
    Frame f;
    f.px.assign(static_cast<size_t>(kW) * kH * 4u, 0xABu);
    RenderTarget target{.pixels = std::span<uint8_t>(f.px), .width = kW, .height = kH, .stride = kW * 4u};
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    RenderOptions options;
    options.overlays = false;
    const spade::Result<void> r = spade::render::render(scene, camera, options, target);
    if (result != nullptr) {
        *result = r;
    } else {
        EXPECT_TRUE(r.has_value()) << r.error().context;
    }
    return f;
}

// y = 0 at depth 5 - z: row 16's centre line. x from -1 to 1 spans columns 10.8 to 22.2.
[[nodiscard]] FlowPolyline horizontal(glm::vec3 colour, float z = 0.0f) {
    return FlowPolyline{{glm::vec3(-1.0f, 0.0f, z), glm::vec3(1.0f, 0.0f, z)}, {colour, colour}};
}

[[nodiscard]] FlowPolyline vertical(glm::vec3 colour, float z, float half_length) {
    return FlowPolyline{{glm::vec3(0.0f, -half_length, z), glm::vec3(0.0f, half_length, z)}, {colour, colour}};
}

[[nodiscard]] RenderScene styled(FlowLineStyle style, FlowPolyline line) {
    RenderScene scene = black_scene();
    scene.flow_line_style = style;
    scene.flow_polylines.push_back(std::move(line));
    return scene;
}

constexpr glm::vec3 kRed(kLevel, 0.0f, 0.0f);
constexpr glm::vec3 kGreen(0.0f, kLevel, 0.0f);

TEST(RenderFlowLineStyle, ADefaultStyleIsOffAndTheLineIsTheOnePixelLine) {
    const FlowLineStyle style;
    EXPECT_FALSE(style.on());
    const Frame got = draw(styled(style, horizontal(kRed)));
    EXPECT_EQ(got.rgb(16, 16), glm::ivec3(200, 0, 0));
    EXPECT_EQ(got.rgb(16, 15), glm::ivec3(0, 0, 0)) << "one row, no coverage above";
    EXPECT_EQ(got.rgb(16, 17), glm::ivec3(0, 0, 0)) << "one row, no coverage below";
}

// A pixel whose centre is d from the segment has coverage clamp(w / 2 + 0.5 - d, 0, 1), and the byte is
// to_byte(behind (1 - a) + colour a) with `behind` black here. Rows 15 and 17 are 1 pixel from the line, 14 and
// 18 are 2, 13 and 19 are 3.
TEST(RenderFlowLineStyle, CoverageIsTheCapsulesAndTheBytesAreTheBlendsExactly) {
    struct Row {
        float width;
        int expect[7];  // rows 13 to 19, column 16: the red channel
    };
    const Row rows[] = {
        {1.0f, {0, 0, 0, 200, 0, 0, 0}},         // half 0.5: only the centre row has full coverage
        {2.0f, {0, 0, 100, 200, 100, 0, 0}},      // half 1: rows at 1 px have coverage 0.5
        {3.0f, {0, 0, 200, 200, 200, 0, 0}},      // half 1.5: coverage 1 at 1 px, 0 at 2
        {4.0f, {0, 100, 200, 200, 200, 100, 0}},  // half 2: coverage 0.5 at 2 px
    };
    for (const Row& r : rows) {
        const Frame got = draw(styled(FlowLineStyle{r.width, 0.0f, 0.35f}, horizontal(kRed)));
        for (int i = 0; i < 7; ++i) {
            EXPECT_EQ(got.rgb(16, static_cast<uint32_t>(13 + i)).r, r.expect[i]) << "width " << r.width << " row " << 13 + i;
            EXPECT_EQ(got.rgb(16, static_cast<uint32_t>(13 + i)).g, 0);
        }
    }
    // The capsule's end is round: the line ends at column 10.8, so its end pixel is covered by what lies within half
    // the width of that point, and a pixel well past it is not.
    const Frame wide = draw(styled(FlowLineStyle{4.0f, 0.0f, 0.35f}, horizontal(kRed)));
    EXPECT_GT(wide.rgb(9, 16).r, 0) << "the cap reaches 2 px past the end";
    EXPECT_EQ(wide.rgb(5, 16).r, 0) << "and no further";
}

// The glow is a halo past the edge: alpha strength (1 - (d - half) / glow)^2, combined with the coverage c as
// a = c + (1 - c) g. Width 2 (half 1), glow 2, strength 0.5, on black, a line of byte 200.
//   row 16, d 0: c 1, a 1                                 -> 200
//   rows 15, 17, d 1: c 0.5, g 0.5, a 0.75                -> 150
//   rows 14, 18, d 2: c 0, g 0.5 (1 - 1/2)^2 = 0.125     -> 25
//   rows 13, 19, d 3: past the glow                       -> 0
TEST(RenderFlowLineStyle, TheGlowIsAHaloPastTheEdgeInTheStatedBytes) {
    const Frame got = draw(styled(FlowLineStyle{2.0f, 2.0f, 0.5f}, horizontal(kRed)));
    const int expect[7] = {0, 25, 150, 200, 150, 25, 0};
    for (int i = 0; i < 7; ++i) {
        EXPECT_EQ(got.rgb(16, static_cast<uint32_t>(13 + i)).r, expect[i]) << "row " << 13 + i;
    }
    const Frame none = draw(styled(FlowLineStyle{2.0f, 0.0f, 0.5f}, horizontal(kRed)));
    EXPECT_EQ(none.rgb(16, 14).r, 0) << "no glow asked, none drawn";
}

// A halo writes no depth: a line behind it still shows through. A near red line with a glow crosses a far green
// vertical one. Where the far line's core meets the near line's halo (column 16, row 14) green is exact; where it
// meets the near core (row 16) red wins by depth, whichever is submitted first.
TEST(RenderFlowLineStyle, AHaloNeverHidesALineBehindItAndACoreStillDoes) {
    for (const bool near_first : {true, false}) {
        RenderScene scene = black_scene();
        scene.flow_line_style = FlowLineStyle{2.0f, 2.0f, 0.5f};
        const FlowPolyline near_line = horizontal(kRed, 1.0f);  // depth 4
        const FlowPolyline far_line = vertical(kGreen, -1.0f, 1.0f);  // depth 6
        if (near_first) {
            scene.flow_polylines = {near_line, far_line};
        } else {
            scene.flow_polylines = {far_line, near_line};
        }
        const Frame got = draw(scene);
        EXPECT_EQ(got.rgb(16, 14), glm::ivec3(0, 200, 0)) << "the far core under the near halo, near first " << near_first;
        EXPECT_EQ(got.rgb(16, 16), glm::ivec3(200, 0, 0)) << "the near core over the far line, near first " << near_first;
    }
}

// A segment with an end behind the camera is clipped to the view, as the one-pixel line is, not skipped.
TEST(RenderFlowLineStyle, ASegmentPastTheCameraIsClippedNotDropped) {
    FlowPolyline into_the_camera{{glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 10.0f)}, {kRed, kRed}};
    const Frame got = draw(styled(FlowLineStyle{2.0f, 0.0f, 0.35f}, into_the_camera));
    EXPECT_EQ(got.rgb(16, 16), glm::ivec3(200, 0, 0));
}

// A styled line is depth-tested against what is drawn before it, like the one-pixel line.
TEST(RenderFlowLineStyle, ALineBehindASurfaceIsHiddenByIt) {
    spade::render::FieldLayer wall;
    wall.center = glm::vec3(0.0f, 0.0f, 1.0f);
    wall.width = 3.0f;
    wall.height = 3.0f;
    wall.values = {0.0f};
    RenderScene alone = black_scene();
    alone.field_layers.push_back(wall);
    const glm::ivec3 surface = draw(alone).rgb(16, 16);
    RenderScene scene = styled(FlowLineStyle{2.0f, 2.0f, 0.5f}, horizontal(kRed, -1.0f));
    scene.field_layers.push_back(wall);
    const Frame got = draw(scene);
    EXPECT_EQ(got.rgb(16, 16), surface) << "the wall in front of the line, glow and all";
    EXPECT_EQ(got.rgb(16, 14), draw(alone).rgb(16, 14));
}

TEST(RenderFlowLineStyle, ABadStyleIsRefusedByNameBeforeAnyPixel) {
    const auto refused = [](FlowLineStyle style, const char* part) {
        spade::Result<void> r;
        const Frame f = draw(styled(style, horizontal(kRed)), &r);
        ASSERT_FALSE(r.has_value()) << part;
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << part;
        EXPECT_NE(r.error().context.find(part), std::string::npos) << r.error().context;
        EXPECT_TRUE(std::all_of(f.px.begin(), f.px.end(), [](uint8_t b) { return b == 0xABu; })) << part;
    };
    refused(FlowLineStyle{0.2f, 0.0f, 0.35f}, "the width is not 0 or 0.5 to 64");
    refused(FlowLineStyle{65.0f, 0.0f, 0.35f}, "the width is not 0 or 0.5 to 64");
    refused(FlowLineStyle{2.0f, -1.0f, 0.35f}, "the glow is not 0 to 64");
    refused(FlowLineStyle{2.0f, 65.0f, 0.35f}, "the glow is not 0 to 64");
    refused(FlowLineStyle{0.0f, 3.0f, 0.35f}, "a glow needs a width");
    refused(FlowLineStyle{2.0f, 3.0f, 1.5f}, "the glow strength is not 0 to 1");
    refused(FlowLineStyle{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.35f}, "not finite");
}

}  // namespace
