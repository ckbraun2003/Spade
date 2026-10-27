// The editor's airflow run (INT-15, the plan
// docs/design/interface/plans/2026-10-22-editor-airflow-run-plan.md),
// display-free (sandbox/editor_airflow.hpp): T1's resolve line, what the
// inspector's Airflow section shows for a region before it runs.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"

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

// Every u increment a NaN: one alone can land on a boundary face, which the
// boundaries overwrite (du's middle at 20^3 is i = 0, j = 0).
void nan_into_du(void*, std::span<float> du, std::span<float>, std::span<float>) {
    std::fill(du.begin(), du.end(), std::numeric_limits<float>::quiet_NaN());
}

// `frame` against a one-thread LiveRun of the same case and program, started
// as Run air started it before Run air on the GPU (LiveRun::start() on the
// CPU): the same steps, time and halves, to the bit.
void expect_one_threads_frame(const spade::scene::SceneDesc& s, const spade::scene::SceneAirflowRegion& region,
                              const an::ControlProgram& program, const ed::AirFrame& frame) {
    const Result<an::ResolvedCase> resolved = ed::air_case(s, region);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    const Result<spade::vehicles::ModelType> model = ed::air_model(s, region);
    ASSERT_TRUE(model.has_value()) << model.error().context;
    Result<an::LiveRun> one = an::LiveRun::start(*resolved, *model);
    ASSERT_TRUE(one.has_value()) << one.error().context;
    spade::FlowVolume v;
    while (one->state().frames < frame.frames) {
        const an::Controls c = an::controls_at(program, static_cast<float>(one->state().t_s));
        ASSERT_TRUE(one->advance_frame(c, v).has_value());
    }
    EXPECT_EQ(frame.steps, one->state().steps);
    EXPECT_EQ(frame.t_s, one->state().t_s);
    EXPECT_TRUE(frame.volume.v == v.v) << "the velocities differ from one thread's";
    EXPECT_TRUE(frame.volume.s == v.s) << "the scalars differ from one thread's";
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

// The inspector's line is resolved once per scene state (the lead,
// 2026-10-26: air_case() hashes and validates the model): again after an
// edit (a new serial) or in another document (opened again, whose serials
// start over), never on a frame that changed nothing.
TEST(EditorAirflow, TheInspectorsLineResolvesOncePerSceneState) {
    const spade::scene::SceneDesc s = with_reference_quad(false);
    ASSERT_FALSE(s.airflow.empty());
    const spade::scene::SceneAirflowRegion& region = s.airflow.back();
    ed::AirLineCache cache;
    const ed::AirLineCache& first = ed::air_line(cache, 1, 5, s, region);
    EXPECT_EQ(first.text, ed::region_line(s, region));
    EXPECT_FALSE(first.refused);
    for (int i = 0; i < 3; ++i) (void)ed::air_line(cache, 1, 5, s, region);
    EXPECT_EQ(cache.resolves, 1u) << "a frame that changed nothing resolved again";
    (void)ed::air_line(cache, 1, 6, s, region);
    EXPECT_EQ(cache.resolves, 2u) << "an edit";
    (void)ed::air_line(cache, 2, 6, s, region);
    EXPECT_EQ(cache.resolves, 3u) << "another document at the same serial";

    spade::scene::SceneDesc turned = s;
    for (auto& v : turned.vehicles) v.start.orient = glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
    const ed::AirLineCache& refused = ed::air_line(cache, 2, 7, turned, turned.airflow.back());
    EXPECT_TRUE(refused.refused);
    EXPECT_EQ(refused.text, ed::region_line(turned, turned.airflow.back()));
}

// The band's speed (§9): simulated time over wall time across the last few
// frames handed over, "air at 1/30 speed"; a frame seen again counts once.
TEST(EditorAirflow, TheSpeedIsSimulatedTimeOverWallTime) {
    ed::AirSpeed speed;
    EXPECT_EQ(speed.line(), "waiting for the air's first frame");
    speed.frame(1, 1.0 / 30.0, 10.0);
    EXPECT_EQ(speed.line(), "the air is starting");
    speed.frame(2, 2.0 / 30.0, 11.0);
    EXPECT_EQ(speed.line(), "air at 1/30 speed");
    speed.frame(2, 2.0 / 30.0, 11.4);  // the same frame, read again by the next displayed frame
    EXPECT_EQ(speed.line(), "air at 1/30 speed");
    speed.frame(3, 3.0 / 30.0, 11.6);
    EXPECT_EQ(speed.line(), "air at 1/24 speed");
    ed::AirSpeed fast;
    fast.frame(1, 0.5, 1.0);
    fast.frame(2, 1.5, 2.0);
    EXPECT_EQ(fast.line(), "air at full speed");
}

// The band (§9) and its floor (the lead's ruling A, 2026-10-27, and Run air
// on the GPU's): the speed, the run's kind, where its fluid steps ("GPU", or
// "CPU: " and why), then the rest of the run's label; elided at its end, it
// never loses its speed, the run's kind or its backend's name (the CPU's why
// goes with the rest).
TEST(EditorAirflow, TheBandsFloorIsItsSpeedTheRunsKindAndItsBackend) {
    const std::string label = "actuator disc \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3";
    const std::string gpu = ed::air_band("air at 1/2 speed", label, "GPU");
    EXPECT_EQ(gpu, "air at 1/2 speed \xc2\xb7 actuator disc \xc2\xb7 GPU \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3");
    EXPECT_EQ(ed::band_floor(gpu), "air at 1/2 speed \xc2\xb7 actuator disc \xc2\xb7 GPU");
    const std::string cpu = ed::air_band("air at 1/43 speed", label, "CPU: no Vulkan device");
    EXPECT_EQ(cpu,
              "air at 1/43 speed \xc2\xb7 actuator disc \xc2\xb7 CPU: no Vulkan device \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3");
    EXPECT_EQ(ed::band_floor(cpu), "air at 1/43 speed \xc2\xb7 actuator disc \xc2\xb7 CPU");
    EXPECT_EQ(ed::air_band("waiting for the air's first frame", "", "GPU"), "waiting for the air's first frame");
    EXPECT_EQ(ed::band_floor("waiting for the air's first frame"), "waiting for the air's first frame");
}

// The library's Phantom (the lead, 2026-10-27), which editor.airflow runs
// since L7c1 gave it props (mesh parts): its air starts, its chain resolves
// and a frame is handed over with no error at its own dx (R/3, to the
// millimetre below), on 20^3 so the debug suite stays cheap, its span
// inside. Run air is actuator discs (the lead's ruling): its parts are not
// the air's solids.
TEST(EditorAirflow, ThePlacedPhantomsAirStartsAndPublishes) {
    auto d = ed::make_scene_document(support::sample_scene(), {});
    ed::AddModelFile add;
    add.file = (std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "phantom3_class.model.yaml")
                   .generic_string();
    add.vehicle.name = "phantom";
    add.vehicle.start.pos = glm::vec3(1.0f, 2.0f, 3.0f);
    const Result<void> added = ed::apply(d, add);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    const Result<void> region = ed::apply(d, ed::AddAirflowRegion{"phantom"});
    ASSERT_TRUE(region.has_value()) << region.error().context;
    spade::scene::SceneDesc s = d.desc();
    ASSERT_EQ(s.airflow.size(), 1u);
    float radius = 0.0f;
    for (const spade::scene::SceneVehicle& v : s.vehicles) {
        if (v.name != "phantom") continue;
        for (const spade::vehicles::ModelType& m : s.models) {
            if (m.name == v.model) radius = m.rotors.front().radius;
        }
    }
    ASSERT_GT(radius, 0.0f);
    EXPECT_FLOAT_EQ(s.airflow[0].cells.dx, std::floor(radius / 3.0f * 1000.0f) / 1000.0f);
    s.airflow[0].cells = spade::physics::airflow::GridShape{20, 20, 20, s.airflow[0].cells.dx};
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(s, s.airflow[0]);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**run, 1);
    ASSERT_NE(frame, nullptr) << "no frame in 50 s";
    EXPECT_TRUE(frame->error.empty()) << frame->error;
    EXPECT_GE(frame->frames, 1u);
    EXPECT_TRUE(frame->run_label.starts_with("actuator disc")) << frame->run_label;
}

// The CPU fallback (Run air on the GPU, slice 3; the lead, 2026-10-27): a run
// its starter puts on the CPU is today's run to the bit, and says why it
// isn't on the GPU. The starter Run air takes when given none is that one,
// with no device.
TEST(EditorAirflow, TheCpuFallbackIsTodaysRunToTheBitAndSaysWhy) {
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const spade::scene::SceneAirflowRegion& region = s.airflow.back();
    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> run =
        ed::AirflowRun::start(s, region, program, {}, ed::cpu_air_starter("the device was lost"));
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_EQ((*run)->backend(), "CPU: the device was lost");
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**run, 2);
    (*run)->stop();
    ASSERT_NE(frame, nullptr);
    ASSERT_TRUE(frame->error.empty()) << frame->error;
    ASSERT_GE(frame->frames, 2u);
    expect_one_threads_frame(s, region, program, *frame);

    Result<std::unique_ptr<ed::AirflowRun>> plain = ed::AirflowRun::start(s, region, program);
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    EXPECT_EQ((*plain)->backend(), "CPU: no Vulkan device");
}

// Run air's guard (the lead, 2026-10-27): a published frame whose fastest
// cell, by its u, v and w, is past ten times the run's top speed is a fault
// naming the frame, the cell and both speeds; at the bound it is not. The
// device can step a NaN into a finite flow where the CPU faults, so this is
// the guard on both backends; a non-finite half is the publish's to refuse,
// and the scan passes over it.
TEST(EditorAirflow, AFrameFasterThanTenTimesTheTopSpeedIsAFault) {
    spade::FlowVolume v;
    v.header.nx = v.header.ny = v.header.nz = 2;
    v.v.assign(8u * spade::kFlowHalvesPerCell, 0);
    v.s.assign(8u * spade::kFlowHalvesPerCell, 0);
    const std::size_t at = spade::flow_cell_index(v.header, 1, 0, 1) * spade::kFlowHalvesPerCell;
    v.v[at + spade::kFlowU] = spade::fp16::from_f32(60.0f).bits;
    v.v[at + spade::kFlowV] = spade::fp16::from_f32(80.0f).bits;
    v.v[spade::kFlowW] = spade::fp16::kNoDataBits;  // cell (0, 0, 0)
    EXPECT_EQ(ed::air_frame_fault(v, 10.0f, 7), "") << "100 m/s is at the bound, not past it";
    v.v[at + spade::kFlowW] = spade::fp16::from_f32(10.0f).bits;
    EXPECT_EQ(ed::air_frame_fault(v, 10.0f, 7),
              "frame 7: the fastest cell, (1, 0, 1), is at 100.5 m/s, past ten times the top speed (100.0 m/s)");
}
