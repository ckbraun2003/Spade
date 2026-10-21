// The flow views over the reference quad's airframe as solids in the airflow
// (D1's drawing side; the lead, 2026-10-21): the rig of solid_drone_rig.hpp,
// stepped by P2 and published with P6's solid mask, drawn on the CPU.
//
// What a C_p view on it may claim (Physics, 2026-10-21): the pressure of the
// fluid next to the wall, to first order in dx, and where stagnation and
// suction are; not validated C_p magnitudes, sharp corner values, deeper
// solid cells (no-data) or separation (the lean mode is inviscid).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "flow_synthetic.hpp"
#include "render/transfer_function.hpp"
#include "render/flow_render.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/view_mode.hpp"
#include "solid_drone_rig.hpp"

namespace {

using spade::render::Camera;
using spade::render::ColourMapId;
using spade::render::FlowView;
using spade::render::FlowViewSpec;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::ViewBase;
using spade::render::ViewLegend;
using spade::render::ViewSpec;
using spade::test::SolidDrone;

constexpr uint32_t kPx = 160;
constexpr float kCpMin = -1.5f;
constexpr float kCpMax = 1.0f;

// The rig is built once: a set_solids and three steps of a 0.3M-cell grid.
[[nodiscard]] const SolidDrone& rig() {
    static const SolidDrone drone = spade::test::solid_drone();
    return drone;
}

[[nodiscard]] Camera looking(glm::vec3 eye, glm::vec3 at, glm::vec3 up) {
    Camera c;
    c.position = eye;
    c.orientation = glm::quatLookAt(glm::normalize(at - eye), up);
    c.near_plane = 0.01f;
    return c;
}

[[nodiscard]] FlowViewSpec cp_layer(float q_ref) {
    FlowViewSpec s;
    s.kind = FlowView::surface_cp;
    s.colours = ColourMapId::cool_warm;
    s.range_min = kCpMin;
    s.range_max = kCpMax;
    s.q_ref = q_ref;
    s.p_ref = 0.0f;
    return s;
}

struct Frame {
    std::vector<uint8_t> px;
    spade::render::FlowFrameReport report;
    std::vector<ViewLegend> legends;
};

// `spec` over the rig's scene, on the CPU.
[[nodiscard]] Frame draw(const spade::FlowVolume& volume, const RenderScene& base, const ViewSpec& spec,
                         const Camera& camera) {
    Frame f;
    RenderScene scene = base;
    RenderOptions options;
    options.overlays = false;
    auto legends = spade::render::apply_view(spec, &volume, scene, options);
    if (!legends) {
        ADD_FAILURE() << legends.error().context;
        return f;
    }
    f.legends = std::move(*legends);
    f.px.assign(static_cast<size_t>(kPx) * kPx * 4u, 0u);
    RenderTarget target{.pixels = std::span<uint8_t>(f.px), .width = kPx, .height = kPx, .stride = kPx * 4u};
    if (auto drew = spade::render::render(scene, camera, options, target, nullptr, &f.report); !drew) {
        ADD_FAILURE() << drew.error().context;
    }
    return f;
}

// The C_p of each pixel the C_p layer coloured: each pixel that differs from
// the same view without it, its colour turned back through the map. A pixel
// no map entry is within 3 levels of is not counted.
[[nodiscard]] std::vector<float> cp_of_coloured(const Frame& with, const Frame& without) {
    std::array<glm::vec3, 256> map{};
    for (int e = 0; e < 256; ++e) {
        map[e] = 255.0f * spade::render::colour_map_at(ColourMapId::cool_warm, static_cast<float>(e) / 255.0f);
    }
    std::vector<float> cps;
    for (size_t i = 0; i + 3 < with.px.size(); i += 4) {
        if (with.px[i] == without.px[i] && with.px[i + 1] == without.px[i + 1] && with.px[i + 2] == without.px[i + 2]) {
            continue;
        }
        const glm::vec3 rgb(with.px[i + 2], with.px[i + 1], with.px[i + 0]);
        int best = 0;
        float best_d = std::numeric_limits<float>::max();
        for (int e = 0; e < 256; ++e) {
            const glm::vec3 d = rgb - map[e];
            if (const float d2 = glm::dot(d, d); d2 < best_d) {
                best_d = d2;
                best = e;
            }
        }
        if (best_d <= 27.0f) cps.push_back(kCpMin + (kCpMax - kCpMin) * static_cast<float>(best) / 255.0f);
    }
    return cps;
}

TEST(RenderSolidDrone, TheRigMakesTheAirframeSolid) {
    const SolidDrone& d = rig();
    ASSERT_TRUE(d.error.empty()) << d.error;
    std::printf("[ solid drone ] %s; %u first-layer cells extrapolated\n", d.run_line.c_str(), d.extrapolated);
    EXPECT_NE(d.run_line.find("10 walls"), std::string::npos) << "Physics' 4 mm: every part a wall: " << d.run_line;
    EXPECT_GT(d.extrapolated, 0u);
    EXPECT_EQ(d.volume.header.extrapolated_cells, d.extrapolated);
    EXPECT_FALSE(d.scene.statics.empty());
}

// At the wall, a C_p read reaches into the deeper solid cells, which are
// no-data (P6), and the airframe goes partly uncoloured; one cell off, on the
// seen side (R2.4's choice), it reads the fluid's first layer everywhere.
TEST(RenderSolidDrone, TheCpViewColoursEveryPixelOfTheAirframe) {
    const SolidDrone& d = rig();
    ASSERT_TRUE(d.error.empty()) << d.error;
    const Camera camera = looking(glm::vec3(-0.25f, 0.2f, 0.3f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const Frame with = draw(d.volume, d.scene, ViewSpec{ViewBase::shaded, {cp_layer(d.q_ref)}}, camera);
    const Frame without = draw(d.volume, d.scene, ViewSpec{ViewBase::shaded, {}}, camera);
    const std::vector<float> cps = cp_of_coloured(with, without);
    std::printf("[ solid drone ] C_p view: %zu px coloured, %llu no-data\n", cps.size(),
                static_cast<unsigned long long>(with.report.surface_pixels_no_data));
    EXPECT_GT(cps.size(), 1000u) << "the airframe is in view and coloured";
    EXPECT_EQ(with.report.surface_pixels_no_data, 0u) << "every pixel of the airframe reads the fluid's pressure";
}

// The legend says what the colour is: with the walls in the flow (the volume
// says it extrapolated their first layer), the fluid's pressure one cell off
// the wall, first order, qualitative; with the walls not in the flow, that it
// is the flow passing through them, not a wall's pressure.
TEST(RenderSolidDrone, TheCpLegendSaysWhatTheWallReadingIs) {
    const SolidDrone& d = rig();
    ASSERT_TRUE(d.error.empty()) << d.error;
    const Camera camera = looking(glm::vec3(-0.25f, 0.2f, 0.3f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const ViewSpec spec{ViewBase::shaded, {cp_layer(d.q_ref)}};
    const Frame in_flow = draw(d.volume, d.scene, spec, camera);
    ASSERT_EQ(in_flow.legends.size(), 1u);
    const std::string& note = in_flow.legends[0].note;
    EXPECT_NE(note.find("one cell off the wall"), std::string::npos) << note;
    EXPECT_NE(note.find("first order"), std::string::npos) << note;
    EXPECT_NE(note.find("not validated"), std::string::npos) << note;

    const spade::FlowVolume passing = spade::test::make_flow_volume(
        d.volume.header.origin, d.volume.header.cell, 16, 16, 16,
        spade::test::uniform_flow(glm::vec3(spade::test::kSolidDroneWind, 0.0f, 0.0f)));
    ASSERT_EQ(passing.header.extrapolated_cells, 0u);
    const Frame not_in_flow = draw(passing, d.scene, spec, camera);
    ASSERT_EQ(not_in_flow.legends.size(), 1u);
    EXPECT_NE(not_in_flow.legends[0].note.find("not in this flow"), std::string::npos) << not_in_flow.legends[0].note;
}

// Where stagnation and suction are, qualitatively (Physics' claim at 4 mm):
// the faces the wind meets read positive C_p, and the tops it passes over
// negative. Magnitudes are not claimed; the bounds are loose on purpose.
TEST(RenderSolidDrone, TheCpViewFindsStagnationUpstreamAndSuctionAbove) {
    const SolidDrone& d = rig();
    ASSERT_TRUE(d.error.empty()) << d.error;
    const ViewSpec cp{ViewBase::shaded, {cp_layer(d.q_ref)}};
    const ViewSpec plain{ViewBase::shaded, {}};
    const Camera upstream = looking(glm::vec3(-0.35f, 0.03f, 0.05f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const Camera above = looking(glm::vec3(0.0f, 0.35f, 0.0f), glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    const std::vector<float> front =
        cp_of_coloured(draw(d.volume, d.scene, cp, upstream), draw(d.volume, d.scene, plain, upstream));
    const std::vector<float> top = cp_of_coloured(draw(d.volume, d.scene, cp, above), draw(d.volume, d.scene, plain, above));
    ASSERT_FALSE(front.empty());
    ASSERT_FALSE(top.empty());
    const float front_max = *std::max_element(front.begin(), front.end());
    const float top_min = *std::min_element(top.begin(), top.end());
    std::printf("[ solid drone ] seen from upstream: %zu px, max C_p %.2f; from above: %zu px, min C_p %.2f\n",
                front.size(), static_cast<double>(front_max), top.size(), static_cast<double>(top_min));
    EXPECT_GT(front_max, 0.3f) << "a stagnation region on the faces the wind meets";
    EXPECT_LT(top_min, -0.2f) << "suction where the wind passes over the airframe";
}

// Every flow view draws with the airframe in the flow: a volume and a vortex
// surface clear in the deeper solid cells, streamlines from upstream that
// meet it, and a slice through it.
TEST(RenderSolidDrone, EveryFlowViewDrawsWithTheSolidAirframe) {
    const SolidDrone& d = rig();
    ASSERT_TRUE(d.error.empty()) << d.error;
    const Camera camera = looking(glm::vec3(-0.25f, 0.2f, 0.3f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    FlowViewSpec volume;
    volume.kind = FlowView::volume;
    volume.range_max = 2.0f * spade::test::kSolidDroneWind;
    FlowViewSpec vortex;
    vortex.kind = FlowView::vortex_iso;
    vortex.quantity = spade::render::FlowQuantity::lambda2_hat;
    vortex.iso_value = -2.0f;
    vortex.range_max = 2.0f * spade::test::kSolidDroneWind;
    FlowViewSpec lines;
    lines.kind = FlowView::streamlines;
    lines.seeds = {glm::vec3(-0.18f, 0.0f, 0.0f), glm::vec3(-0.18f, 0.0f, 0.11f), glm::vec3(-0.18f, 0.02f, -0.05f)};
    lines.range_max = 2.0f * spade::test::kSolidDroneWind;
    FlowViewSpec slice;
    slice.kind = FlowView::slice;
    slice.quantity = spade::render::FlowQuantity::pressure;
    slice.plane.width = 0.36f;
    slice.plane.height = 0.12f;
    slice.plane.cells_u = 90;
    slice.plane.cells_v = 30;
    slice.range_min = -d.q_ref;
    slice.range_max = d.q_ref;
    for (const FlowViewSpec& layer : {volume, vortex, lines, slice}) {
        SCOPED_TRACE(static_cast<int>(layer.kind));
        const Frame f = draw(d.volume, d.scene, ViewSpec{ViewBase::shaded, {layer}}, camera);
        EXPECT_EQ(f.legends.size(), 1u);
    }
}

}  // namespace
