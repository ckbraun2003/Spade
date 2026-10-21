// The tour's airflow chapter, display-free (sandbox/airflow_live.hpp): the
// library's reference quad (N4) flown live at 32^3, the views drawn over it,
// and the frames the on-screen downwash check (Test/Docs' kDownwashPresent)
// reads. The window draws what these decide.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <vector>

#include <glm/glm.hpp>

#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/flow_volume.hpp"
#include "render/view_mode.hpp"
#include "vehicles/rotor.hpp"

#include "../sandbox/airflow_live.hpp"

namespace an = spade::analysis;
namespace af = spade::sandbox::airflow;
namespace eg = spade::sandbox::end_goal;
namespace fs = std::filesystem;
using spade::render::FlowView;

namespace {

fs::path library_quad() {
    return fs::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
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
