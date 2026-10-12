// Drawing streamlines and tracers through render(): a polyline lands on its
// projection in its colours, depth-tested both ways; a point is a square of
// its size; colours come from the speeds; malformed input is refused before
// any pixel.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "render/flow_lines.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"

namespace {

using spade::render::Camera;
using spade::render::FieldLayer;
using spade::render::FlowPoints;
using spade::render::FlowPolyline;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

constexpr uint32_t kW = 33, kH = 33;  // odd: the centre pixel's ray is the axis

struct Frame {
    std::vector<uint8_t> px;
    [[nodiscard]] glm::vec3 rgb(uint32_t x, uint32_t y) const {
        const size_t i = (static_cast<size_t>(y) * kW + x) * 4u;
        return glm::vec3(px[i + 2], px[i + 1], px[i + 0]);
    }
};

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

// At y = 0.05, a quarter pixel above row 16's centre line rather than on a
// row boundary, so no rounding decides the row.
[[nodiscard]] FlowPolyline red_line(float z) {
    return FlowPolyline{{glm::vec3(-1.0f, 0.05f, z), glm::vec3(1.0f, 0.05f, z)},
                        {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)}};
}

// x = +-1 at distance 5 projects to columns 16.5 +- 16.5 tan(30 deg)^-1 / 5,
// about 10.8 to 22.2; row 16 holds y = 0.05.
TEST(RenderFlowDraw, APolylineDrawsAlongItsProjectionInItsColours) {
    RenderScene scene;
    scene.flow_polylines.push_back(red_line(0.0f));
    const Frame got = draw(scene);
    // Every one of the 10 columns the line crosses. rasterizeLine() once
    // truncated its step count and skipped column 16 here (rendering/
    // line-gaps, fixed: it steps ceil(max(|dx|, |dy|)) times).
    uint32_t red = 0;
    for (uint32_t x = 12; x <= 21; ++x) red += got.rgb(x, 16) == glm::vec3(255.0f, 0.0f, 0.0f) ? 1u : 0u;
    EXPECT_EQ(red, 10u);
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            if (got.rgb(x, y) == glm::vec3(255.0f, 0.0f, 0.0f)) {
                EXPECT_EQ(y, 16u) << "red only on the line's row";
                EXPECT_TRUE(x >= 10u && x <= 22u) << "red only between the line's ends, column " << x;
            }
        }
    }
    EXPECT_NE(got.rgb(5, 16), glm::vec3(255.0f, 0.0f, 0.0f)) << "past the line's end";
    EXPECT_NE(got.rgb(16, 10), glm::vec3(255.0f, 0.0f, 0.0f)) << "off the line";

    FlowPolyline mixed = red_line(0.0f);
    mixed.colours[1] = glm::vec3(0.0f, 0.0f, 1.0f);
    RenderScene two;
    two.flow_polylines.push_back(mixed);
    const glm::vec3 mid = draw(two).rgb(18, 16);  // a drawn column (16 is the known gap)
    EXPECT_NEAR(mid.x, 128.0f, 1.0f) << "a segment takes the mean of its ends";
    EXPECT_NEAR(mid.z, 128.0f, 1.0f);
}

[[nodiscard]] FieldLayer square_at(float z) {
    FieldLayer layer;
    layer.center = glm::vec3(0.0f, 0.0f, z);
    layer.width = 3.0f;
    layer.height = 3.0f;
    layer.values = {0.0f};
    return layer;
}

TEST(RenderFlowDraw, LinesAndPointsAreDepthTestedBothWays) {
    RenderScene front;
    front.field_layers.push_back(square_at(1.0f));
    const glm::vec3 surface = draw(front).rgb(16, 16);
    front.flow_polylines.push_back(red_line(0.0f));
    front.flow_points = FlowPoints{{glm::vec3(0.0f)}, {glm::vec3(0.0f, 1.0f, 0.0f)}, 3};
    EXPECT_EQ(draw(front).rgb(16, 16), surface) << "a surface in front hides the line and the point";

    RenderScene behind;
    behind.field_layers.push_back(square_at(-1.0f));
    behind.flow_polylines.push_back(red_line(0.0f));
    EXPECT_EQ(draw(behind).rgb(18, 16), glm::vec3(255.0f, 0.0f, 0.0f)) << "a line in front of a surface shows";
}

TEST(RenderFlowDraw, APointIsASquareOfItsSize) {
    RenderScene scene;
    scene.flow_points = FlowPoints{{glm::vec3(0.0f)}, {glm::vec3(0.0f, 1.0f, 0.0f)}, 3};
    const Frame got = draw(scene);
    const glm::vec3 green(0.0f, 255.0f, 0.0f);
    for (uint32_t y = 15; y <= 17; ++y) {
        for (uint32_t x = 15; x <= 17; ++x) EXPECT_EQ(got.rgb(x, y), green) << x << ", " << y;
    }
    EXPECT_NE(got.rgb(13, 16), green);
    EXPECT_NE(got.rgb(16, 19), green);
}

TEST(RenderFlowDraw, StreamlineAndTracerColoursComeFromTheirSources) {
    spade::render::Streamline line;
    line.points = {glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(2.0f, 0.0f, 0.0f)};
    line.speeds = {0.0f, 0.5f, 1.0f};
    spade::render::TransferFunction speed;
    const auto prepared = spade::render::prepare_transfer(speed, spade::FlowVolumeHeader{});
    const FlowPolyline coloured = spade::render::colour_streamline(line, prepared);
    ASSERT_EQ(coloured.points, line.points);
    ASSERT_EQ(coloured.colours.size(), 3u);
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(coloured.colours[i], spade::render::colour_map_at(spade::render::ColourMapId::viridis, line.speeds[i]));
    }

    spade::render::TracerState state;
    state.positions = {glm::vec3(1.0f), glm::vec3(2.0f)};
    state.ages = {0.1f, 0.2f};
    state.sources = {0u, 0u};
    const FlowPoints points = spade::render::tracer_points(state, glm::vec3(0.9f), 4);
    EXPECT_EQ(points.positions, state.positions);
    EXPECT_EQ(points.colours, std::vector<glm::vec3>(2, glm::vec3(0.9f)));
    EXPECT_EQ(points.size_px, 4u);
}

// L6: refused before any pixel is written.
TEST(RenderFlowDraw, MalformedLinesAndPointsAreRefused) {
    const auto refused = [](const RenderScene& scene, const char* what) {
        spade::Result<void> r;
        const Frame f = draw(scene, &r);
        ASSERT_FALSE(r.has_value()) << what << " must be refused";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << what;
        EXPECT_TRUE(std::all_of(f.px.begin(), f.px.end(), [](uint8_t b) { return b == 0xABu; })) << what;
    };
    RenderScene scene;
    scene.flow_polylines.push_back(red_line(0.0f));
    scene.flow_polylines[0].colours.pop_back();
    refused(scene, "a line missing a colour");
    scene.flow_polylines[0] = red_line(0.0f);
    scene.flow_polylines[0].points[1].x = std::numeric_limits<float>::quiet_NaN();
    refused(scene, "a NaN point");
    scene.flow_polylines[0] = red_line(0.0f);
    scene.flow_polylines[0].colours[0].y = 1.5f;
    refused(scene, "a colour past 1");
    RenderScene dots;
    dots.flow_points = FlowPoints{{glm::vec3(0.0f)}, {glm::vec3(0.5f)}, 0};
    refused(dots, "a zero point size");
    dots.flow_points = FlowPoints{{glm::vec3(0.0f)}, {}, 3};
    refused(dots, "a point missing its colour");
}

}  // namespace
