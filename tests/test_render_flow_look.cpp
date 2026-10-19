// test_render_flow_look.cpp -- one look for the airflow views
// (render/flow_look.hpp): the helpers the films and the sandbox's tour share.
// Expectations come from the engine's own views called directly
// (slice_flow_volume, colour_map_at) or are worked by hand, never read back
// from flow_look (TD-4).

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "flow_synthetic.hpp"
#include "render/flow_look.hpp"

using spade::render::ColourMapId;
using spade::render::FieldLayer;
using spade::render::FlowQuantity;
using spade::render::RenderScene;

namespace {

// A box of air moving along +x at 1 + x m/s (0.25 m cells, from -1 to 1 on
// each axis): the speed rises along x, so a slice and a line both show it.
[[nodiscard]] spade::FlowVolume rising_wind() {
    return spade::test::make_flow_volume(glm::vec3(-1.0f), 0.25f, 8, 8, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowU] = 1.0f + p.x;
        return c;
    });
}

[[nodiscard]] spade::FlowVolume uniform_wind() {
    return spade::test::make_flow_volume(glm::vec3(-1.0f), 0.25f, 8, 8, 8, [](glm::vec3) {
        spade::test::FlowChannels c{};
        c[spade::kFlowU] = 1.0f;
        return c;
    });
}

}  // namespace

TEST(FlowLook, ColoursOverIsColourOnlyOverItsRange) {
    const auto tf = spade::render::colours_over(FlowQuantity::lambda2_hat, ColourMapId::inferno, -3.0f, 7.0f);
    EXPECT_EQ(tf.quantity, FlowQuantity::lambda2_hat);
    EXPECT_EQ(tf.colours, ColourMapId::inferno);
    EXPECT_EQ(tf.range_min, -3.0f);
    EXPECT_EQ(tf.range_max, 7.0f);
    EXPECT_FALSE(tf.auto_range);
    EXPECT_FALSE(tf.constant_colour.has_value());
}

TEST(FlowLook, VolumeTransferAbsorbsFromItsOnsetToSigmaAtTheTop) {
    const auto tf = spade::render::volume_transfer(FlowQuantity::vorticity, ColourMapId::inferno, 400.0f, 0.25f, 12.0f);
    EXPECT_EQ(tf.quantity, FlowQuantity::vorticity);
    EXPECT_EQ(tf.colours, ColourMapId::inferno);
    EXPECT_EQ(tf.range_min, 0.0f);
    EXPECT_EQ(tf.range_max, 400.0f);
    ASSERT_EQ(tf.extinction.size(), 2u);
    EXPECT_EQ(tf.extinction[0].value, 100.0f);
    EXPECT_EQ(tf.extinction[0].sigma, 0.0f);
    EXPECT_EQ(tf.extinction[1].value, 400.0f);
    EXPECT_EQ(tf.extinction[1].sigma, 12.0f);
}

// The slice is slice_flow_volume()'s, value for value, coloured in the map
// over the given range in 128 bins; a geometry it refuses appends nothing.
TEST(FlowLook, ASliceIsTheVolumesSliceColouredOverItsRange) {
    const spade::FlowVolume volume = rising_wind();
    FieldLayer plane;
    plane.width = 1.5f;
    plane.height = 1.0f;
    plane.cells_u = 6;
    plane.cells_v = 4;
    RenderScene scene;
    const auto added = spade::render::add_slice(scene, volume, FlowQuantity::speed, plane, ColourMapId::cool_warm,
                                                0.5f, 1.5f);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    ASSERT_EQ(scene.field_layers.size(), 1u);
    const FieldLayer& got = scene.field_layers[0];
    const auto want = spade::render::slice_flow_volume(volume, FlowQuantity::speed, plane);
    ASSERT_TRUE(want.has_value());
    EXPECT_EQ(got.values, want->values);
    EXPECT_EQ(got.colour_map.bins, 128u);
    EXPECT_EQ(got.colour_map.range_min, 0.5f);
    EXPECT_EQ(got.colour_map.range_max, 1.5f);
    EXPECT_EQ(got.colour_map.palette.stops, spade::render::field_palette_of(ColourMapId::cool_warm).stops);
    EXPECT_GT(got.values.back(), got.values.front()) << "the speed rises along +x";

    FieldLayer empty = plane;
    empty.cells_u = 0;
    EXPECT_FALSE(spade::render::add_slice(scene, volume, FlowQuantity::speed, empty, ColourMapId::viridis, 0.0f, 1.0f)
                     .has_value());
    EXPECT_EQ(scene.field_layers.size(), 1u);
}

// Lines through uniform 1 m/s wind take the map's colour at 1 m/s (halfway up
// 0..2), or the constant colour when given; a seed outside the box traces no
// line and adds none.
TEST(FlowLook, StreamlinesAreColouredBySpeedOrConstant) {
    const spade::FlowVolume volume = uniform_wind();
    const std::array<glm::vec3, 3> seeds{glm::vec3(-0.5f, 0.0f, 0.0f), glm::vec3(-0.5f, 0.3f, 0.2f),
                                         glm::vec3(-0.5f, -0.4f, 0.1f)};
    const auto by_speed = spade::render::colours_over(FlowQuantity::speed, ColourMapId::viridis, 0.0f, 2.0f);
    RenderScene scene;
    const auto added = spade::render::add_streamlines(scene, volume, seeds, spade::render::StreamlineOptions{}, by_speed);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    ASSERT_EQ(scene.flow_polylines.size(), 3u);
    const glm::vec3 halfway = spade::render::colour_map_at(ColourMapId::viridis, 0.5f);
    for (const spade::render::FlowPolyline& line : scene.flow_polylines) {
        ASSERT_GE(line.points.size(), 2u);
        ASSERT_EQ(line.colours.size(), line.points.size());
        for (const glm::vec3& c : line.colours) EXPECT_LE(glm::length(c - halfway), 1.0e-3f);
    }

    const glm::vec3 white(1.0f);
    RenderScene plain;
    ASSERT_TRUE(spade::render::add_streamlines(plain, volume, seeds, spade::render::StreamlineOptions{}, by_speed,
                                               &white)
                    .has_value());
    ASSERT_EQ(plain.flow_polylines.size(), 3u);
    for (const glm::vec3& c : plain.flow_polylines[0].colours) EXPECT_EQ(c, white);

    RenderScene outside;
    const std::array<glm::vec3, 1> away{glm::vec3(5.0f, 5.0f, 5.0f)};
    ASSERT_TRUE(spade::render::add_streamlines(outside, volume, away, spade::render::StreamlineOptions{}, by_speed)
                    .has_value());
    EXPECT_TRUE(outside.flow_polylines.empty());
}

// A rim: closed, every point on the radius and in the plane square to the
// axis, every point in the colour.
TEST(FlowLook, ACircleIsClosedOnItsRadiusAboutItsAxis) {
    const glm::vec3 centre(1.0f, 2.0f, 3.0f), axis(1.0f, 1.0f, 0.0f), colour(1.0f, 0.5f, 0.0f);
    const spade::render::FlowPolyline rim = spade::render::circle(centre, axis, 0.5f, colour, 24);
    ASSERT_EQ(rim.points.size(), 25u);
    ASSERT_EQ(rim.colours.size(), 25u);
    EXPECT_LE(glm::length(rim.points.front() - rim.points.back()), 1.0e-6f);
    const glm::vec3 n = glm::normalize(axis);
    for (std::size_t i = 0; i < rim.points.size(); ++i) {
        const glm::vec3 r = rim.points[i] - centre;
        EXPECT_NEAR(glm::length(r), 0.5f, 1.0e-6f) << i;
        EXPECT_LE(std::abs(glm::dot(r, n)), 1.0e-6f) << i;
        EXPECT_EQ(rim.colours[i], colour) << i;
    }
    // Consecutive points a twenty-fourth of a turn apart.
    const float chord = glm::length(rim.points[1] - rim.points[0]);
    EXPECT_NEAR(chord, 2.0f * 0.5f * std::sin(3.14159265f / 24.0f), 1.0e-5f);
}
