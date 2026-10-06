// render/flow_render.hpp through render(): the CPU flow-volume march.
// Checked against Beer-Lambert on every pixel of a slab (oblique rays end
// in partial steps), against a shifted box, against opaque surfaces in
// front of, inside and behind the volume, and for no-data, refusals, the
// range report, and transfer functions marching together. Test-only double
// and std::exp; this file feeds no golden.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "flow_synthetic.hpp"
#include "render/flow_render.hpp"
#include "render/raster_cpu.hpp"
#include "render/raymarch.hpp"
#include "render/scene.hpp"

namespace {

using spade::render::Camera;
using spade::render::FieldLayer;
using spade::render::FlowFrameReport;
using spade::render::FlowLayer;
using spade::render::FlowQuantity;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::TransferFunction;

constexpr uint32_t kW = 33;  // odd, so the centre pixel's ray is the axis
constexpr uint32_t kH = 33;
constexpr float kCell = 0.1f;

[[nodiscard]] Camera camera_on_plus_z() {
    Camera c;
    c.position = glm::vec3(0.0f, 0.0f, 5.0f);
    c.near_plane = 0.1f;
    c.far_plane = 100.0f;
    return c;
}

struct Frame {
    std::vector<uint8_t> px;
    [[nodiscard]] glm::vec3 rgb(uint32_t x, uint32_t y) const {
        const size_t i = (static_cast<size_t>(y) * kW + x) * 4u;
        return glm::vec3(px[i + 2], px[i + 1], px[i + 0]);
    }
};

[[nodiscard]] Frame draw(const RenderScene& scene, FlowFrameReport* report = nullptr,
                         spade::Result<void>* result = nullptr) {
    Frame f;
    f.px.assign(static_cast<size_t>(kW) * kH * 4u, 0xABu);
    RenderTarget target{.pixels = std::span<uint8_t>(f.px), .width = kW, .height = kH, .stride = kW * 4u};
    RenderOptions options;
    options.overlays = false;
    const spade::Result<void> r = spade::render::render(scene, camera_on_plus_z(), options, target, nullptr, report);
    if (result != nullptr) {
        *result = r;
    } else {
        EXPECT_TRUE(r.has_value()) << r.error().context;
    }
    return f;
}

// A box from (-4, -4, z0) to (4, 4, z0 + nz cell): wider than the view.
[[nodiscard]] spade::FlowVolume slab(float z0, uint32_t nz, float x_shift = 0.0f) {
    return spade::test::make_flow_volume(glm::vec3(-4.0f + x_shift, -4.0f, z0), kCell, 80, 80, nz,
                                         spade::test::uniform_flow({1.0f, 0.0f, 0.0f}));
}

// Constant extinction and colour, whatever the value.
[[nodiscard]] TransferFunction absorber(float sigma, glm::vec3 colour) {
    TransferFunction tf;
    tf.quantity = FlowQuantity::speed;
    tf.constant_colour = colour;
    tf.extinction = {{0.0f, sigma}, {1.0f, sigma}};
    return tf;
}

[[nodiscard]] RenderScene scene_with(const spade::FlowVolume& volume, std::vector<TransferFunction> transfers) {
    RenderScene scene;
    scene.flow_layers.push_back(FlowLayer{&volume, std::move(transfers)});
    return scene;
}

[[nodiscard]] float max_channel_diff(const Frame& a, const Frame& b) {
    float worst = 0.0f;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const glm::vec3 d = glm::abs(a.rgb(x, y) - b.rgb(x, y));
            worst = std::max({worst, d.x, d.y, d.z});
        }
    }
    return worst;
}

// Every pixel: C (1 - T) + T C_behind with T = exp(-sigma d / cos theta).
// Off-axis rays cross the slab in a length no step divides, so their last
// step is partial.
TEST(RenderFlowVolume, ASlabTransmitsBeerLambertOnEveryPixel) {
    const Frame behind = draw(RenderScene{});
    const spade::FlowVolume volume = slab(-0.25f, 5);  // 0.5 m thick
    const float sigma = 1.4f;
    const glm::vec3 colour(0.9f, 0.6f, 0.2f);
    const Frame got = draw(scene_with(volume, {absorber(sigma, colour)}));
    const spade::render::RayGrid grid = spade::render::ray_grid(camera_on_plus_z(), kW, kH);
    float worst = 0.0f;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const double cos_theta = -spade::render::camera_ray(grid, x, y).dir_cam.z;
            const double t = std::exp(-static_cast<double>(sigma) * 0.5 / cos_theta);
            for (int c = 0; c < 3; ++c) {
                const double want = 255.0 * colour[c] * (1.0 - t) + t * behind.rgb(x, y)[c];
                worst = std::max(worst, static_cast<float>(std::fabs(got.rgb(x, y)[c] - want)));
            }
        }
    }
    EXPECT_LE(worst, 1.0f) << "bytes, against Beer-Lambert in double";
    EXPECT_NE(got.rgb(16, 16), behind.rgb(16, 16));
}

// The same slab a fraction of a step deeper and a fraction of a cell to the
// side: the steps fall elsewhere, and the pixels do not move.
TEST(RenderFlowVolume, ShiftingTheBoxDoesNotMoveThePixels) {
    const spade::FlowVolume here = slab(-0.25f, 5);
    const spade::FlowVolume there = slab(-0.25f - 0.037f, 5, 0.063f);
    const TransferFunction tf = absorber(2.0f, glm::vec3(0.3f, 0.8f, 0.5f));
    EXPECT_LE(max_channel_diff(draw(scene_with(here, {tf})), draw(scene_with(there, {tf}))), 1.0f);
}

[[nodiscard]] FieldLayer opaque_square_at(float z) {
    FieldLayer layer;
    layer.center = glm::vec3(0.0f, 0.0f, z);
    layer.width = 1.0f;
    layer.height = 1.0f;
    layer.values = {0.0f};
    return layer;
}

// Opaque geometry and the volume, both ways: a surface in front hides the
// air; one inside sees only the air in front of it; one behind sees all.
TEST(RenderFlowVolume, OpaqueSurfacesHideTheAirBehindThemAndNotInFront) {
    const spade::FlowVolume volume = slab(-1.0f, 20);  // z from -1 to 1
    const float sigma = 0.5f;
    const TransferFunction black = absorber(sigma, glm::vec3(0.0f));
    for (const float z : {2.0f, 0.0f, -3.0f}) {
        RenderScene bare;
        bare.field_layers.push_back(opaque_square_at(z));
        const glm::vec3 surface = draw(bare).rgb(16, 16);
        RenderScene scene = scene_with(volume, {black});
        scene.field_layers.push_back(opaque_square_at(z));
        const glm::vec3 got = draw(scene).rgb(16, 16);
        const double air_in_front = z >= 1.0f ? 0.0 : (z >= -1.0f ? 1.0 - z : 2.0);
        const double t = std::exp(-static_cast<double>(sigma) * air_in_front);
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(got[c], t * surface[c], 1.0) << "surface at z = " << z << ", channel " << c;
        }
    }
}

// L6: no-data draws clear, and the report counts the pixels it touched.
TEST(RenderFlowVolume, NoDataDrawsClearAndIsCounted) {
    const spade::FlowVolume volume =
        spade::test::make_flow_volume(glm::vec3(-4.0f, -4.0f, -0.5f), kCell, 80, 80, 10, [](glm::vec3 p) {
            spade::test::FlowChannels c{};
            c[spade::kFlowDye] = p.x < 0.0f ? std::numeric_limits<float>::quiet_NaN() : 1.0f;
            return c;
        });
    ASSERT_EQ(volume.header.no_data, 40u * 80u * 10u);
    TransferFunction tf = absorber(3.0f, glm::vec3(0.0f));
    tf.quantity = FlowQuantity::dye;
    const Frame behind = draw(RenderScene{});
    FlowFrameReport report;
    const Frame got = draw(scene_with(volume, {tf}), &report);
    EXPECT_EQ(got.rgb(2, 16), behind.rgb(2, 16)) << "a ray through no-data only is clear";
    EXPECT_LT(got.rgb(30, 16).x + got.rgb(30, 16).y + got.rgb(30, 16).z,
              behind.rgb(30, 16).x + behind.rgb(30, 16).y + behind.rgb(30, 16).z);
    ASSERT_EQ(report.layers.size(), 1u);
    EXPECT_EQ(report.layers[0].volume_no_data, volume.header.no_data);
    EXPECT_GE(report.layers[0].pixels_crossing_no_data, 16u * kH);
    EXPECT_LT(report.layers[0].pixels_crossing_no_data, static_cast<uint64_t>(kW) * kH);
}

// Q5: the report says which range each transfer function drew with, and
// flags the auto one for announcement.
TEST(RenderFlowVolume, TheReportAnnouncesAnAutoRange) {
    const spade::FlowVolume volume = slab(-0.25f, 5);
    TransferFunction fixed = absorber(1.0f, glm::vec3(0.5f));
    fixed.range_min = 0.0f;
    fixed.range_max = 2.0f;
    TransferFunction chosen;
    chosen.quantity = FlowQuantity::u;
    chosen.auto_range = true;
    FlowFrameReport report;
    (void)draw(scene_with(volume, {fixed, chosen}), &report);
    ASSERT_EQ(report.layers.size(), 1u);
    ASSERT_EQ(report.layers[0].ranges.size(), 2u);
    EXPECT_FALSE(report.layers[0].ranges[0].from_header);
    EXPECT_EQ(report.layers[0].ranges[0].max, 2.0f);
    EXPECT_TRUE(report.layers[0].ranges[1].from_header);
    EXPECT_EQ(report.layers[0].ranges[1].min, 1.0f);
    EXPECT_EQ(report.layers[0].ranges[1].max, 1.0f);
}

// A layer's transfer functions march together: extinctions add, colours
// mix by extinction.
TEST(RenderFlowVolume, TransferFunctionsMarchTogether) {
    const spade::FlowVolume volume = slab(-0.25f, 5);
    const Frame two = draw(scene_with(volume, {absorber(0.6f, glm::vec3(0.0f)), absorber(0.8f, glm::vec3(0.0f))}));
    const Frame one = draw(scene_with(volume, {absorber(1.4f, glm::vec3(0.0f))}));
    EXPECT_LE(max_channel_diff(two, one), 1.0f) << "extinctions add";

    const spade::FlowVolume thick = slab(-1.0f, 20);
    const Frame mixed =
        draw(scene_with(thick, {absorber(10.0f, glm::vec3(1.0f, 0.0f, 0.0f)), absorber(30.0f, glm::vec3(0.0f, 0.0f, 1.0f))}));
    const glm::vec3 c = mixed.rgb(16, 16);
    EXPECT_NEAR(c.x, 255.0 * 0.25, 2.0) << "red carries a quarter of the extinction";
    EXPECT_NEAR(c.y, 0.0, 1.0);
    EXPECT_NEAR(c.z, 255.0 * 0.75, 2.0);
}

// L6: a malformed layer is refused before any pixel is written.
TEST(RenderFlowVolume, AMalformedLayerIsRefusedBeforeAnyPixel) {
    const spade::FlowVolume good = slab(-0.25f, 5);
    spade::FlowVolume bad = slab(-0.25f, 5);
    bad.header.cell = 0.0f;
    TransferFunction broken;
    broken.range_max = broken.range_min;
    const TransferFunction fine = absorber(1.0f, glm::vec3(0.5f));
    const auto refused = [](const RenderScene& scene, const char* what) {
        spade::Result<void> r;
        const Frame f = draw(scene, nullptr, &r);
        ASSERT_FALSE(r.has_value()) << what << " must be refused";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << what;
        EXPECT_TRUE(std::all_of(f.px.begin(), f.px.end(), [](uint8_t b) { return b == 0xABu; }))
            << what << ": no pixel may be written";
    };
    {
        RenderScene scene;
        scene.flow_layers.push_back(FlowLayer{nullptr, {fine}});
        refused(scene, "a layer with no volume");
    }
    refused(scene_with(bad, {fine}), "a malformed volume");
    refused(scene_with(good, {}), "no transfer function");
    refused(scene_with(good, {fine, fine, fine, fine, fine}), "five transfer functions");
    refused(scene_with(good, {fine, broken}), "a malformed transfer function");
}

TEST(RenderFlowVolume, AnEmptyIntervalLeavesThePixelAlone) {
    const spade::FlowVolume volume = slab(-0.25f, 5);
    const auto prepared = spade::render::prepare_flow_layer(FlowLayer{&volume, {absorber(1.0f, glm::vec3(1.0f))}});
    const auto r = spade::render::march_flow_ray(prepared, glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, -1.0f),
                                                 5.0f, 4.0f);
    EXPECT_EQ(r.transmittance, 1.0f);
    EXPECT_EQ(r.colour, glm::vec3(0.0f));
    EXPECT_FALSE(r.crossed_no_data);
}

}  // namespace
