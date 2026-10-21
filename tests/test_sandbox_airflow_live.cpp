// The tour's airflow chapter, display-free (sandbox/airflow_live.hpp): the
// library's reference quad (N4) flown live at 32^3, the views drawn over it,
// and the frames the on-screen downwash check (Test/Docs' kDownwashPresent)
// reads. The window draws what these decide.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "render/raster_cpu.hpp"
#include "render/target.hpp"
#include "render_parts/render_parts.hpp"
#include "render/view_mode.hpp"
#include "vehicles/rotor.hpp"

#include "../sandbox/airflow_live.hpp"
#include "../sandbox/orbit_camera.hpp"
#include "../sandbox/png_writer.hpp"
#include "../sandbox/vehicle_draw.hpp"

namespace an = spade::analysis;
namespace af = spade::sandbox::airflow;
namespace eg = spade::sandbox::end_goal;
namespace fs = std::filesystem;
using spade::render::FlowView;

namespace {

fs::path library_quad() {
    return fs::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
}

[[nodiscard]] bool env_gate_is_set(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name) != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// Four discs in an X, 0.12 m out, 0.06 m in radius, pushing down.
std::vector<eg::RotorProbe> x_quad() {
    std::vector<eg::RotorProbe> r;
    for (const glm::vec2 at : {glm::vec2(1, 1), glm::vec2(-1, 1), glm::vec2(-1, -1), glm::vec2(1, -1)}) {
        eg::RotorProbe p;
        p.hub = glm::vec3(0.12f * at.x, 0.02f, 0.12f * at.y);
        p.radius = 0.06f;
        p.thrust = 2.0f;
        r.push_back(p);
    }
    return r;
}

// The chapter's grid, with no data: what airflow_view() reads is the header.
spade::FlowVolume header_only(float deepest_lambda2) {
    spade::FlowVolume v;
    v.header.nx = v.header.ny = v.header.nz = 32;
    v.header.cell = 0.025f;
    v.header.origin = glm::vec3(-0.4f);
    v.header.channel_min[spade::kFlowLambda2Hat] = deepest_lambda2;
    return v;
}

}  // namespace

// The library quad flies with its parts cleared (thinner than a 25 mm cell)
// and is drawn with them; the case is the chapter's 32^3, flown under the
// showcase, armed at hover at 0.2 s, until its hover ends at 1.2 s.
TEST(SandboxAirflowLive, TheLibraryQuadResolvesAt32Cubed) {
    const spade::Result<af::LiveQuad> q = af::live_quad(library_quad());
    ASSERT_TRUE(q.has_value()) << q.error().context;
    EXPECT_FALSE(q->draw_model.parts.empty());
    EXPECT_TRUE(q->run_model.parts.empty());
    EXPECT_EQ(q->run_model.rotors.size(), q->draw_model.rotors.size());
    EXPECT_EQ(q->resolved.base.nx, 32u);
    EXPECT_EQ(q->resolved.base.ny, 32u);
    EXPECT_EQ(q->resolved.base.nz, 32u);
    ASSERT_TRUE(af::armed_at(q->program).has_value());
    EXPECT_FLOAT_EQ(*af::armed_at(q->program), 0.2f);
    ASSERT_TRUE(af::hover_ends_at(q->program).has_value());
    EXPECT_FLOAT_EQ(*af::hover_ends_at(q->program), 1.2f);
    EXPECT_FLOAT_EQ(q->resolved.spec.time.duration, 1.2f) << "the chapter flies the hover the check judges";
}

TEST(SandboxAirflowLive, AMissingModelFileIsRefusedNamingIt) {
    const spade::Result<af::LiveQuad> q = af::live_quad(library_quad().parent_path() / "no_such.model.yaml");
    ASSERT_FALSE(q.has_value());
    EXPECT_NE(q.error().context.find("no_such.model.yaml"), std::string::npos) << q.error().context;
}

// The check's premise: the controls are the ones set at arming, from arming
// until the program next changes them (the pitch at 1.2 s), never after.
TEST(SandboxAirflowLive, TheControlsAreConstantThroughTheHoverOnly) {
    const an::ControlProgram p = an::showcase_program(an::ShowcaseTrim{0.4f, 0.6f});
    EXPECT_FALSE(af::controls_constant(p, 0.1));
    EXPECT_TRUE(af::controls_constant(p, 0.2));
    EXPECT_TRUE(af::controls_constant(p, 1.19));
    EXPECT_FALSE(af::controls_constant(p, 1.2));
    EXPECT_FALSE(af::controls_constant(p, 3.5));
    EXPECT_FALSE(af::controls_constant(an::ControlProgram{}, 1.0)) << "a program that never arms";
}

// Flown live, the quad's rotors push the air down through the probes under
// them, and each frame from arming is read for the check: timed from
// arming, one probe per rotor, the controls still the hover's. On a 40 mm
// grid (20^3) and armed at once, so it stays cheap in a debug suite: the
// chapter's 32^3 costs about 40 ms a step in release.
TEST(SandboxAirflowLive, TheRunPushesTheAirDownThroughTheProbes) {
    const spade::Result<af::LiveQuad> q = af::live_quad(library_quad(), 0.04f);
    ASSERT_TRUE(q.has_value()) << q.error().context;
    const an::Controls hover = an::controls_at(q->program, *af::armed_at(q->program));
    const an::ControlProgram program{{{0.0f, hover}, {1.0f, an::Controls{}}}};
    spade::Result<an::LiveRun> run = an::LiveRun::start(q->resolved, q->run_model, q->propulsion);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    spade::FlowVolume volume;
    std::vector<eg::DownwashFrame> frames;
    while (run->state().t_s < 0.15) {  // five frames: about 23 s in debug, under ctest's 60
        const an::Controls c = an::controls_at(program, static_cast<float>(run->state().t_s));
        const spade::Result<an::LiveFrame> f = run->advance_frame(c, volume);
        ASSERT_TRUE(f.has_value()) << f.error().context;
        if (auto frame = af::downwash_frame(volume, *run, program)) frames.push_back(std::move(*frame));
    }
    ASSERT_FALSE(frames.empty());
    EXPECT_GE(frames.front().t_s, 0.0);
    EXPECT_LT(frames.front().t_s, 0.05) << "the first frame read is the one after arming";
    const eg::DownwashFrame& last = frames.back();
    EXPECT_TRUE(last.controls_constant);
    ASSERT_EQ(last.rotors.size(), q->run_model.rotors.size());
    for (std::size_t i = 0; i < last.rotors.size(); ++i) {
        EXPECT_EQ(last.probe[i], eg::ProbeStatus::ok) << "rotor " << i;
        EXPECT_GT(last.u_down[i], 0.0f) << "rotor " << i << ": no downwash at its probe 0.17 s after arming";
    }
}

// Over the drone while it flies: the vortex cores once there are any, and
// streamlines seeded under each disc; GL draws both.
TEST(SandboxAirflowLive, TheViewShowsTheCoresAndTheLines) {
    const std::vector<eg::RotorProbe> rotors = x_quad();
    const spade::FlowVolume v = header_only(-30.0f);
    const spade::render::ViewSpec spec = af::airflow_view(v, rotors, 8.0f);
    EXPECT_EQ(spec.base, spade::render::ViewBase::shaded);
    ASSERT_EQ(spec.layers.size(), 2u);
    EXPECT_EQ(spec.layers[0].kind, FlowView::vortex_iso);
    EXPECT_FLOAT_EQ(spec.layers[0].iso_value, -30.0f * af::kVortexFraction);
    EXPECT_EQ(spec.layers[1].kind, FlowView::streamlines);
    EXPECT_EQ(spec.layers[1].seeds.size(), 8u * rotors.size());
    for (const glm::vec3& s : spec.layers[1].seeds) EXPECT_LT(s.y, 0.02f) << "a seed above the discs";
    // The lines over [0, top], the far wake's speed; the cores, which sit in
    // the jet's slower shear layer, over [0, top / 2] (the induced speed at
    // hover), so they don't all draw in the colour map's near-black end.
    EXPECT_FLOAT_EQ(spec.layers[1].range_max, 8.0f);
    EXPECT_FLOAT_EQ(spec.layers[0].range_max, 4.0f);

    spade::render::ViewInputs in;
    in.has_volume = true;
    const spade::render::ViewSupport s = spade::render::supports(spec, spade::render::ViewBackend::gl, in);
    EXPECT_TRUE(s.drawable) << s.why_not;
}

TEST(SandboxAirflowLive, NoCoresAreDrawnBeforeThereAreAny) {
    const spade::render::ViewSpec spec = af::airflow_view(header_only(-1.0f), x_quad(), 8.0f);
    ASSERT_EQ(spec.layers.size(), 1u);
    EXPECT_EQ(spec.layers[0].kind, FlowView::streamlines);
}

// The wake's slice, shown on its own after the run (over the drone it hides
// half the wake): speed on a plane standing through the two farthest-apart
// hubs, faced by a camera at facing_yaw().
TEST(SandboxAirflowLive, TheSliceStandsThroughTheFarthestHubsAndIsFaced) {
    const spade::render::ViewSpec spec = af::wake_slice(header_only(-30.0f), x_quad(), 8.0f);
    ASSERT_EQ(spec.layers.size(), 1u);
    const spade::render::FlowViewSpec& slice = spec.layers[0];
    EXPECT_EQ(slice.kind, FlowView::slice);
    EXPECT_EQ(slice.quantity, spade::render::FlowQuantity::speed);
    EXPECT_FLOAT_EQ(slice.range_max, 8.0f);
    EXPECT_NEAR(std::abs(slice.plane.right.x), std::sqrt(0.5f), 1e-5f) << "not along the X's diagonal";
    EXPECT_NEAR(slice.plane.right.y, 0.0f, 1e-6f);
    EXPECT_NEAR(slice.plane.center.x, 0.0f, 1e-6f);
    EXPECT_NEAR(slice.plane.center.y, 0.0f, 1e-6f);

    // The orbit camera's eye sits along (sin yaw, cos yaw): the plane's normal.
    const float yaw = af::facing_yaw(slice.plane);
    const glm::vec3 eye(std::sin(yaw), 0.0f, std::cos(yaw));
    EXPECT_NEAR(glm::dot(eye, slice.plane.right), 0.0f, 1e-5f) << "the camera is not square to the slice";

    spade::render::ViewInputs in;
    in.has_volume = true;
    EXPECT_TRUE(spade::render::supports(spec, spade::render::ViewBackend::gl, in).drawable);
}

// The colours' top: twice the momentum-theory induced speed at hover.
TEST(SandboxAirflowLive, TheSpeedScaleIsTheFarWakesAtHover) {
    const spade::Result<af::LiveQuad> q = af::live_quad(library_quad());
    ASSERT_TRUE(q.has_value()) << q.error().context;
    const auto& m = q->draw_model;
    const float thrust = m.body.mass * 9.80665f / static_cast<float>(m.rotors.size());
    const float v_h = spade::vehicles::rotor_hover_induced_velocity(thrust, 1.225f, m.rotors.front().radius);
    EXPECT_GT(v_h, 1.0f);
    EXPECT_FLOAT_EQ(af::speed_top(m, 1.225f, 9.80665f), 2.0f * v_h);
}

// ===========================================================================
// THE POSTER FRAME, MEASURED (Rendering's near-black cores, 2026-10-21). A
// measurement, not a check: with SPADE_AIRFLOW_POSTER=1 it flies the tour's
// quad to its poster frame (armed + 0.9 s), prints what the cores' colour is
// made from (the deepest lambda2-hat, the iso value, the colour range, the
// speed of the cells inside the cores), and draws the frame on the CPU, as the
// tour's AirflowSession does on GL, to <test output>/airflow_poster_cpu.png:
// one frame both renderers can be compared on. About a minute in release.
// ===========================================================================

TEST(SandboxAirflowLive, ThePosterFrameMeasured) {
    if (!env_gate_is_set("SPADE_AIRFLOW_POSTER")) {
        GTEST_SKIP() << "a measurement: set SPADE_AIRFLOW_POSTER=1 to run it";
    }
    const spade::Result<af::LiveQuad> quad = af::live_quad(library_quad());
    ASSERT_TRUE(quad.has_value()) << quad.error().context;
    spade::Result<an::LiveRun> run = an::LiveRun::start(quad->resolved, quad->run_model, quad->propulsion);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const double poster = af::armed_at(quad->program).value_or(0.0) + 0.9;
    spade::FlowVolume volume;
    while (run->state().t_s < poster) {
        const an::Controls c = an::controls_at(quad->program, static_cast<float>(run->state().t_s));
        const spade::Result<an::LiveFrame> f = run->advance_frame(c, volume);
        ASSERT_TRUE(f.has_value()) << f.error().context;
    }

    const float top = af::speed_top(quad->run_model, quad->resolved.spec.fluid.density, 9.80665f);
    const spade::render::ViewSpec spec = af::airflow_view(volume, af::probes_of(*run), top);
    const auto iso = std::find_if(spec.layers.begin(), spec.layers.end(),
                                  [](const spade::render::FlowViewSpec& l) { return l.kind == FlowView::vortex_iso; });
    ASSERT_NE(iso, spec.layers.end()) << "no cores at the poster frame";

    // The speed of every cell inside a core (lambda2-hat at or below the iso
    // value): the iso surface is coloured by the speed there.
    const spade::FlowVolumeHeader& h = volume.header;
    const std::size_t cells = static_cast<std::size_t>(h.nx) * h.ny * h.nz;
    std::vector<float> core_speeds;
    for (std::size_t i = 0; i < cells; ++i) {
        const uint16_t l2 = volume.s[i * spade::kFlowHalvesPerCell + (spade::kFlowLambda2Hat % spade::kFlowHalvesPerCell)];
        if (l2 == spade::fp16::kNoDataBits || !(spade::fp16::to_f32(l2) <= iso->iso_value)) continue;
        float sq = 0.0f;
        for (uint32_t c = spade::kFlowU; c <= spade::kFlowW; ++c) {
            const float u = spade::fp16::to_f32(volume.v[i * spade::kFlowHalvesPerCell + c]);
            sq += u * u;
        }
        core_speeds.push_back(std::sqrt(sq));
    }
    ASSERT_FALSE(core_speeds.empty());
    std::sort(core_speeds.begin(), core_speeds.end());
    std::printf("poster frame: t = %.3f s, %ux%ux%u, dx %.4f m\n", run->state().t_s, h.nx, h.ny, h.nz,
                static_cast<double>(h.cell));
    std::printf("  lambda2-hat: channel_min %.4g, channel_max %.4g; iso %.4g (1/3 of the deepest)\n",
                static_cast<double>(h.channel_min[spade::kFlowLambda2Hat]),
                static_cast<double>(h.channel_max[spade::kFlowLambda2Hat]), static_cast<double>(iso->iso_value));
    std::printf("  speed: top %.4g m/s (2 v_h); the cores' colour range [%.4g, %.4g] m/s\n", static_cast<double>(top),
                static_cast<double>(iso->range_min), static_cast<double>(iso->range_max));
    std::printf("  inside the cores: %zu cells, speed min %.4g, median %.4g, max %.4g m/s\n", core_speeds.size(),
                static_cast<double>(core_speeds.front()), static_cast<double>(core_speeds[core_speeds.size() / 2]),
                static_cast<double>(core_speeds.back()));

    // The frame on the CPU, as AirflowSession draws it on GL: the drone's
    // parts, no shadows or overlays, the view applied, the default lighting.
    spade::render::RenderScene scene;
    spade::render_parts::PartsLook look;
    look.body = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(spade::sandbox::detail::lambert(0.35f, 0.37f, 0.40f));
    look.rotor = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(spade::sandbox::detail::lambert(0.12f, 0.12f, 0.13f));
    look.porous = look.body;
    look.object_name = quad->draw_model.name;
    const spade::Result<spade::render_parts::PreparedParts> prepared =
        spade::render_parts::prepare_parts(quad->draw_model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    const spade::Result<void> bound =
        spade::sandbox::append_vehicle_parts(scene, *prepared, quad->draw_model, run->body(), look);
    ASSERT_TRUE(bound.has_value()) << bound.error().context;
    spade::render::RenderOptions options;
    options.shadows = false;
    options.overlays = false;
    spade::render::ViewCache cache;
    const spade::Result<std::vector<spade::render::ViewLegend>> legends =
        spade::render::apply_view(spec, &volume, scene, options, cache);
    ASSERT_TRUE(legends.has_value()) << legends.error().context;
    spade::sandbox::OrbitCamera camera;
    camera.target = glm::vec3(0.0f, -0.12f * af::kBoxM, 0.0f);
    camera.yaw = 0.52f;
    camera.pitch = 0.35f;
    camera.distance = 1.5f * af::kBoxM;
    constexpr uint32_t kW = 1280;
    constexpr uint32_t kH = 720;
    std::vector<uint8_t> pixels(static_cast<std::size_t>(kW) * kH * 4u);
    spade::render::RenderTarget target{pixels, kW, kH, kW * 4u};
    const spade::Result<void> drawn = spade::render::render(scene, camera.to_render_camera(), options, target);
    ASSERT_TRUE(drawn.has_value()) << drawn.error().context;
    for (std::size_t i = 0; i < pixels.size(); i += 4) {  // the target is BGRX; the PNG is RGBA
        std::swap(pixels[i], pixels[i + 2]);
        pixels[i + 3] = 255u;
    }
    const fs::path out = fs::path(SPADE_TEST_OUTPUT_DIR) / "airflow_poster_cpu.png";
    fs::create_directories(out.parent_path());
    ASSERT_TRUE(spade::sandbox::png::write_file(out, spade::sandbox::png::encode_rgba(pixels.data(), kW, kH, false)));
    std::printf("  the CPU's frame: %s\n", out.string().c_str());
}
