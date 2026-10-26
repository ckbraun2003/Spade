// The editor's airflow run (INT-15, the plan
// docs/design/interface/plans/2026-10-22-editor-airflow-run-plan.md),
// display-free (sandbox/editor_airflow.hpp): T1's resolve line, what the
// inspector's Airflow section shows for a region before it runs.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <format>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"

#include "../sandbox/editor_airflow.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "editor_test_support.hpp"

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
namespace an = spade::analysis;
using spade::Result;

namespace {

[[nodiscard]] spade::scene::SceneDesc with_region() {
    auto d = ed::make_scene_document(support::sample_scene(), {});
    const Result<void> added = ed::apply(d, ed::AddAirflowRegion{"hover_quad_0"});
    EXPECT_TRUE(added.has_value()) << (added ? "" : added.error().context);
    return d.desc();
}

// The library's reference quad (it has a propulsion chain; the hover
// scene's quad has none), placed at (1, 2, 3), with its region; `small`
// shrinks the region to 20^3 at 40 mm so the debug suite stays cheap
// (SandboxAirflowLive's 40 mm grid: about 5 s a frame in debug).
[[nodiscard]] spade::scene::SceneDesc with_reference_quad(bool small = true) {
    auto d = ed::make_scene_document(support::sample_scene(), {});
    ed::AddModelFile add;
    add.file = (std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml")
                   .generic_string();
    add.vehicle.name = "ref";
    add.vehicle.start.pos = glm::vec3(1.0f, 2.0f, 3.0f);
    const Result<void> added = ed::apply(d, add);
    EXPECT_TRUE(added.has_value()) << (added ? "" : added.error().context);
    const Result<void> region = ed::apply(d, ed::AddAirflowRegion{"ref"});
    EXPECT_TRUE(region.has_value()) << (region ? "" : region.error().context);
    spade::scene::SceneDesc s = d.desc();
    if (small && !s.airflow.empty()) s.airflow.back().cells = spade::physics::airflow::GridShape{20, 20, 20, 0.04f};
    return s;
}

}  // namespace

// The region's case resolves as LiveRun will run it, and the line says so as
// LiveRun's label does: "32³, dx 25 mm, R/dx 3". The reference quad, since
// the hover scene's quad has no propulsion chain (refused since T2).
TEST(EditorAirflow, ARegionsLineIsItsResolvedGrid) {
    const spade::scene::SceneDesc s = with_reference_quad(false);
    ASSERT_EQ(s.airflow.size(), 1u);
    const Result<spade::analysis::ResolvedCase> resolved = ed::air_case(s, s.airflow[0]);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->base.nx, 32u);
    EXPECT_FLOAT_EQ(resolved->base.dx, s.airflow[0].cells.dx);
    const std::string line = ed::region_line(s, s.airflow[0]);
    EXPECT_TRUE(line.starts_with(std::format("32\xc2\xb3, dx {:g} mm, R/dx ", s.airflow[0].cells.dx * 1000.0f))) << line;
}

// A drone placed turned is refused before a step (the lead, 2026-10-26):
// LiveRun starts its drone level, and the air's box never turns.
TEST(EditorAirflow, ATurnedDroneIsRefusedNamingIt) {
    spade::scene::SceneDesc s = with_region();
    ASSERT_EQ(s.airflow.size(), 1u);
    for (auto& v : s.vehicles) v.start.orient = glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
    const Result<spade::analysis::ResolvedCase> resolved = ed::air_case(s, s.airflow[0]);
    ASSERT_FALSE(resolved.has_value());
    EXPECT_NE(resolved.error().context.find("'hover_quad_0'"), std::string::npos) << resolved.error().context;
    EXPECT_NE(resolved.error().context.find("Run air needs the drone unrotated"), std::string::npos)
        << resolved.error().context;
    EXPECT_EQ(ed::region_line(s, s.airflow[0]), resolved.error().context) << "the refusal, verbatim";
}

// The viewport draws each follow region as a wireframe box around its drone
// (T1): its 12 edges, centred where the drone is placed (LiveRun holds it at
// the box's centre), n dx on a side, in the region's line colour.
TEST(EditorAirflow, ARegionIsDrawnAsABoxAroundItsDrone) {
    spade::scene::SceneDesc s = with_region();
    ASSERT_EQ(s.airflow.size(), 1u);
    for (auto& v : s.vehicles) v.start.pos = glm::vec3(0.0f);
    spade::render::Camera cam;
    cam.position = glm::vec3(0.0f, 0.0f, 3.0f);
    const std::vector<ed::GizmoLine> lines = ed::region_box_lines(s, cam, 800, 600);
    ASSERT_EQ(lines.size(), 12u);
    const float half = 0.5f * static_cast<float>(s.airflow[0].cells.nx) * s.airflow[0].cells.dx;
    const std::optional<glm::vec2> centre = ed::project(cam, glm::vec3(0.0f), 800, 600);
    const std::optional<glm::vec2> near_corner = ed::project(cam, glm::vec3(half), 800, 600);
    ASSERT_TRUE(centre.has_value());
    ASSERT_TRUE(near_corner.has_value());
    float lo = 1e9f;
    float hi = -1e9f;
    for (const ed::GizmoLine& l : lines) {
        EXPECT_EQ(l.axis, ed::kRegionLine);
        lo = std::min({lo, l.a.x, l.b.x});
        hi = std::max({hi, l.a.x, l.b.x});
    }
    EXPECT_NEAR(hi, near_corner->x, 0.5f) << "the box's near face is its widest";
    EXPECT_NEAR(lo + hi, 2.0f * centre->x, 0.5f) << "centred on the drone";

    s.airflow.clear();
    EXPECT_TRUE(ed::region_box_lines(s, cam, 800, 600).empty());
}

// ===========================================================================
// T2: AirflowRun, the air on a worker thread (the plan's §9, approved by the
// lead 2026-10-26). The worker owns LiveRun alone; the window takes the last
// AirFrame, an immutable copy made fresh at each publish.
// ===========================================================================

namespace {

constexpr an::Controls kHalf{true, 0.5f, 0.0f, 0.0f, 0.0f};

// The run's latest frame once it has published `n` frames or stopped on an
// error; at most 50 s (debug ctest's limit is 60).
[[nodiscard]] std::shared_ptr<const ed::AirFrame> wait_for_frames(const ed::AirflowRun& run, uint64_t n) {
    for (int i = 0; i < 1000; ++i) {
        std::shared_ptr<const ed::AirFrame> f = run.latest();
        if (f && (f->frames >= n || !f->error.empty())) return f;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return run.latest();
}

void nan_into_du(void*, std::span<float> du, std::span<float>, std::span<float>) {
    if (!du.empty()) du[du.size() / 2] = std::numeric_limits<float>::quiet_NaN();
}

}  // namespace

// Determinism (§9): a program's run on the worker gives, at each frame it
// hands over, the volume a one-thread LiveRun gives at that frame, to the
// bit, its origin shifted to where the drone is placed.
TEST(EditorAirflow, AProgramsRunOnTheWorkerIsBitIdenticalToOneThread) {
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const spade::scene::SceneAirflowRegion& region = s.airflow.back();
    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(s, region, program);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**run, 2);
    (*run)->stop();
    ASSERT_NE(frame, nullptr);
    ASSERT_TRUE(frame->error.empty()) << frame->error;
    ASSERT_GE(frame->frames, 2u);

    const Result<an::ResolvedCase> resolved = ed::air_case(s, region);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    const Result<spade::vehicles::ModelType> model = ed::air_model(s, region);
    ASSERT_TRUE(model.has_value()) << model.error().context;
    Result<an::LiveRun> one = an::LiveRun::start(*resolved, *model);
    ASSERT_TRUE(one.has_value()) << one.error().context;
    spade::FlowVolume v;
    while (one->state().frames < frame->frames) {
        const an::Controls c = an::controls_at(program, static_cast<float>(one->state().t_s));
        ASSERT_TRUE(one->advance_frame(c, v).has_value());
    }
    EXPECT_EQ(frame->steps, one->state().steps);
    EXPECT_EQ(frame->t_s, one->state().t_s);
    EXPECT_TRUE(frame->volume.v == v.v) << "the velocities differ from one thread's";
    EXPECT_TRUE(frame->volume.s == v.s) << "the scalars differ from one thread's";
    const glm::vec3 shift = frame->volume.header.origin - v.header.origin;
    EXPECT_NEAR(shift.x, 1.0f, 1e-6f);
    EXPECT_NEAR(shift.y, 2.0f, 1e-6f);
    EXPECT_NEAR(shift.z, 3.0f, 1e-6f);
    EXPECT_FALSE(frame->run_label.empty());
}

// Stop (§9): stop() joins the worker, nothing is handed over after it, and
// a second stop() is harmless.
TEST(EditorAirflow, StopJoinsAndNothingIsHandedOverAfter) {
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(s, s.airflow.back());
    ASSERT_TRUE(run.has_value()) << run.error().context;
    (*run)->set_controls(kHalf);  // the Pilot section's slot
    ASSERT_NE(wait_for_frames(**run, 1), nullptr);
    (*run)->stop();
    const std::shared_ptr<const ed::AirFrame> last = (*run)->latest();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    EXPECT_EQ((*run)->latest(), last) << "a frame was handed over after stop()";
    (*run)->stop();
}

// The hand-off (§9, the lead's two changes): a frame the window holds never
// changes while the worker publishes on, and it outlives the run.
TEST(EditorAirflow, AHeldFrameNeverChangesAndOutlivesTheRun) {
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(s, s.airflow.back(), program);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const std::shared_ptr<const ed::AirFrame> held = wait_for_frames(**run, 1);
    ASSERT_NE(held, nullptr);
    ASSERT_TRUE(held->error.empty()) << held->error;
    const std::vector<uint16_t> bytes = held->volume.v;
    const uint64_t generation = held->volume.generation;
    const std::shared_ptr<const ed::AirFrame> later = wait_for_frames(**run, held->frames + 1);
    ASSERT_NE(later, nullptr);
    EXPECT_NE(later, held);
    EXPECT_NE(later->volume.generation, generation) << "each publish is a new generation";
    run->reset();  // stop, join and destroy the run
    EXPECT_EQ(held->volume.generation, generation);
    EXPECT_TRUE(held->volume.v == bytes) << "a held frame changed";
}

// A failed step (§9) ends the run with its error in the last frame, never
// std::terminate: C8q's nudge puts a NaN into the increments.
TEST(EditorAirflow, AFailedStepReachesTheLastFrame) {
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(
        s, s.airflow.back(), program, [](an::LiveRun& r) { r.set_test_increment_nudge(&nan_into_du, nullptr); });
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const std::shared_ptr<const ed::AirFrame> last = wait_for_frames(**run, 1);
    ASSERT_NE(last, nullptr);
    EXPECT_FALSE(last->error.empty()) << "the NaN was not refused";
    EXPECT_NE(last->error.find("the air stopped"), std::string::npos) << last->error;
}

// Refused before a thread exists, naming why: rotors with no propulsion
// chain (the hover scene's quad; air_case() says so, so the inspector shows
// it before Run air), and a drone placed turned.
TEST(EditorAirflow, ARunIsRefusedBeforeItsThreadNamingWhy) {
    const spade::scene::SceneDesc chainless = with_region();
    ASSERT_FALSE(chainless.airflow.empty());
    const Result<std::unique_ptr<ed::AirflowRun>> no_chain = ed::AirflowRun::start(chainless, chainless.airflow[0]);
    ASSERT_FALSE(no_chain.has_value());
    EXPECT_NE(no_chain.error().context.find("no propulsion chain"), std::string::npos) << no_chain.error().context;
    EXPECT_EQ(ed::region_line(chainless, chainless.airflow[0]), no_chain.error().context);

    spade::scene::SceneDesc turned = with_reference_quad();
    ASSERT_FALSE(turned.airflow.empty());
    for (auto& v : turned.vehicles) v.start.orient = glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
    const Result<std::unique_ptr<ed::AirflowRun>> refused = ed::AirflowRun::start(turned, turned.airflow.back());
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("Run air needs the drone unrotated"), std::string::npos)
        << refused.error().context;
}
