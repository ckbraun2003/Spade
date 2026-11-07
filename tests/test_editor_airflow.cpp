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
#include <string_view>
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
// the hover scene's quad has no propulsion chain (refused since T2), at its
// region's own grid (Run air's rule since item 1).
TEST(EditorAirflow, ARegionsLineIsItsResolvedGrid) {
    const spade::scene::SceneDesc s = with_reference_quad(false);
    ASSERT_EQ(s.airflow.size(), 1u);
    const spade::physics::airflow::GridShape& g = s.airflow[0].cells;
    const Result<spade::analysis::ResolvedCase> resolved = ed::air_case(s, s.airflow[0]);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->base.nx, g.nx);
    EXPECT_FLOAT_EQ(resolved->base.dx, g.dx);
    const std::string line = ed::region_line(s, s.airflow[0]);
    EXPECT_TRUE(line.starts_with(std::format("{}\xc2\xb3, dx {:g} mm, R/dx ", g.nx, g.dx * 1000.0f))) << line;
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

// `frame` against a one-thread LiveRun of the case the CPU's starter flies
// (air_cases()'s discs) and the same program, LiveRun::start() on the CPU:
// the same steps, time and halves, to the bit.
void expect_one_threads_frame(const spade::scene::SceneDesc& s, const spade::scene::SceneAirflowRegion& region,
                              const an::ControlProgram& program, const ed::AirFrame& frame) {
    const Result<ed::AirCases> cases = ed::air_cases(s, region);
    ASSERT_TRUE(cases.has_value()) << cases.error().context;
    Result<an::LiveRun> one = an::LiveRun::start(cases->discs.resolved, cases->discs.model);
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

    const Result<ed::AirCases> cases = ed::air_cases(s, region);
    ASSERT_TRUE(cases.has_value()) << cases.error().context;
    Result<an::LiveRun> one = an::LiveRun::start(cases->discs.resolved, cases->discs.model);
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

// Run air's flight readout rides on every frame (editor_flight_readout.hpp): the reference quad held at the showcase
// trim's hover throttle in calm air has no relative wind, four rotors turning, and their static k_T w^2 thrust
// summing to the weight within the trim's own tolerance (5%: the trim aims the chain's thrust, and the readout is the
// model's static curve; the READY reports the figure).
TEST(EditorAirflow, TheFrameCarriesTheFlightReadoutAndHoverThrustHoldsTheWeight) {
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const spade::scene::SceneAirflowRegion& region = s.airflow.back();
    const Result<ed::AirCases> cases = ed::air_cases(s, region);
    ASSERT_TRUE(cases.has_value()) << cases.error().context;
    const Result<an::ShowcaseTrim> trim = an::showcase_trim(cases->discs.resolved.spec, cases->discs.model);
    ASSERT_TRUE(trim.has_value()) << trim.error().context;
    const an::ControlProgram program{{{0.0f, an::Controls{true, trim->hover, 0.0f, 0.0f, 0.0f}}}};
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(s, region, program);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**run, 5);
    (*run)->stop();
    ASSERT_NE(frame, nullptr);
    ASSERT_TRUE(frame->error.empty()) << frame->error;

    const ed::FlightReadout& f = frame->flight;
    ASSERT_EQ(f.rotors.size(), cases->discs.model.rotors.size());
    EXPECT_FALSE(f.flow) << "calm air: no relative wind";
    EXPECT_EQ(f.airspeed_mps, 0.0f);
    for (const ed::RotorReadout& r : f.rotors) EXPECT_GT(r.rpm, 0.0f);
    EXPECT_GT(f.weight_n, 0.0f);
    EXPECT_NEAR(f.thrust_n, f.weight_n, 0.05f * f.weight_n) << "thrust " << f.thrust_n << " N, weight " << f.weight_n << " N";
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
// frames handed over, as a share of real time (D10): "air at 3.3% of real
// time"; a frame seen again counts once.
TEST(EditorAirflow, TheSpeedIsSimulatedTimeOverWallTime) {
    ed::AirSpeed speed;
    EXPECT_EQ(speed.line(), "waiting for the air's first frame");
    speed.frame(1, 1.0 / 30.0, 10.0);
    EXPECT_EQ(speed.line(), "the air is starting");
    speed.frame(2, 2.0 / 30.0, 11.0);
    EXPECT_EQ(speed.line(), "air at 3.3% of real time");
    speed.frame(2, 2.0 / 30.0, 11.4);  // the same frame, read again by the next displayed frame
    EXPECT_EQ(speed.line(), "air at 3.3% of real time");
    speed.frame(3, 3.0 / 30.0, 11.6);
    EXPECT_EQ(speed.line(), "air at 4.2% of real time");
    ed::AirSpeed fast;
    fast.frame(1, 0.5, 1.0);
    fast.frame(2, 1.5, 2.0);
    EXPECT_EQ(fast.line(), "air in real time");
    // Rendering's review of 65ce8c16: {:.2g} prints 100 as "1e+02", so from
    // 10% the share is whole percent, floored (99.6% is not 100%).
    EXPECT_EQ(ed::real_time_share(0.04), "at 4% of real time");
    EXPECT_EQ(ed::real_time_share(0.375), "at 37% of real time");
    EXPECT_EQ(ed::real_time_share(0.996), "at 99% of real time");
    EXPECT_EQ(ed::real_time_share(1.5), "in real time");
}

// The band (§9) and its floor (the lead's rulings, 2026-10-27, and Run air
// on the GPU's): the speed, the run's kind, where its fluid steps ("GPU", or
// "CPU: " and why), then the rest of the run's label. When the bar is short
// the kind goes first, so the backend's name and the grid's dx outlast it;
// then the band is elided at its end, never below its speed and its
// backend's name (the CPU's why goes with the rest).
TEST(EditorAirflow, TheBandShedsTheRunsKindFirstAndItsFloorIsItsSpeedAndBackend) {
    const std::string label = "actuator disc \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3";
    const std::string gpu = ed::air_band("air at 1/2 speed", label, "GPU");
    EXPECT_EQ(gpu, "air at 1/2 speed \xc2\xb7 actuator disc \xc2\xb7 GPU \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3");
    EXPECT_EQ(ed::band_without_kind(gpu), "air at 1/2 speed \xc2\xb7 GPU \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3");
    EXPECT_EQ(ed::band_floor(gpu), "air at 1/2 speed \xc2\xb7 GPU");
    const std::string cpu = ed::air_band("air at 1/43 speed", label, "CPU: no Vulkan device");
    EXPECT_EQ(cpu,
              "air at 1/43 speed \xc2\xb7 actuator disc \xc2\xb7 CPU: no Vulkan device \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3");
    EXPECT_EQ(ed::band_without_kind(cpu),
              "air at 1/43 speed \xc2\xb7 CPU: no Vulkan device \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3");
    EXPECT_EQ(ed::band_floor(cpu), "air at 1/43 speed \xc2\xb7 CPU");
    const std::string waiting = "waiting for the air's first frame";
    EXPECT_EQ(ed::air_band(waiting, "", "GPU"), waiting);
    EXPECT_EQ(ed::band_without_kind(waiting), waiting);
    EXPECT_EQ(ed::band_floor(waiting), waiting);

    // The bar's choice, each byte a unit wide: whole when it fits, else
    // without its kind (which the bar then elides at its end).
    const auto bytes = [](std::string_view t) { return static_cast<float>(t.size()); };
    EXPECT_EQ(ed::band_shown(gpu, static_cast<float>(gpu.size()), bytes), gpu);
    EXPECT_EQ(ed::band_shown(gpu, static_cast<float>(gpu.size()) - 1.0f, bytes), ed::band_without_kind(gpu));
    EXPECT_EQ(ed::band_shown(waiting, 1.0f, bytes), waiting);
}

// The library's Phantom (the lead, 2026-10-27), which editor.airflow runs
// since L7c1 gave it props (mesh parts): its region is Run air's cell for
// its body (104^3 at 12 mm), and its air starts, its chain resolves and a
// frame is handed over with no error at R/3 (39 mm), on 20^3 so the debug
// suite stays cheap, its span inside. At 39 mm every part is drawn only,
// so on the CPU it is discs only and says nothing of the body.
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
    EXPECT_EQ(s.airflow[0].cells.nx, 104u);
    EXPECT_FLOAT_EQ(s.airflow[0].cells.dx, 0.012f);
    s.airflow[0].cells = spade::physics::airflow::GridShape{20, 20, 20, std::floor(radius / 3.0f * 1000.0f) / 1000.0f};
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(s, s.airflow[0]);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_EQ((*run)->backend(), "CPU: no Vulkan device");
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**run, 1);
    ASSERT_NE(frame, nullptr) << "no frame in 50 s";
    EXPECT_TRUE(frame->error.empty()) << frame->error;
    EXPECT_GE(frame->frames, 1u);
    EXPECT_TRUE(frame->run_label.starts_with("actuator disc")) << frame->run_label;
}

// The CPU fallback (Run air on the GPU, slice 3; the lead, 2026-10-27): a run
// its starter puts on the CPU is a one-thread run of its discs case to the
// bit, and says why it isn't on the GPU. The starter Run air takes when
// given none is that one, with no device.
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

// The cases Run air flies (item 1; the lead's rulings, 2026-11-03): the
// body's, the region's grid with the drone's parts and Physics' step for
// its cell; and the CPU's fallback, discs only on R/3's grid over the same
// box (32 cells at 39 mm for the Phantom's 104^3 at 12 mm), since a grid
// that resolves the body takes the CPU seconds a step. The fallback's band
// says the body needs the GPU, its floor "... CPU" as ever. At a cell that
// holds no part (20^3 at 39 mm) both are the region's grid, and the band
// says nothing of the body.
TEST(EditorAirflow, TheCpuFallbackFliesDiscsOnR3sGridOverTheSameBoxAndSaysSo) {
    const auto placed = [](const char* file) {
        auto d = ed::make_scene_document(support::sample_scene(), {});
        ed::AddModelFile add;
        add.file = (std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / file).generic_string();
        add.vehicle.name = "drone";
        const Result<void> added = ed::apply(d, add);
        EXPECT_TRUE(added.has_value()) << (added ? "" : added.error().context);
        const Result<void> region = ed::apply(d, ed::AddAirflowRegion{"drone"});
        EXPECT_TRUE(region.has_value()) << (region ? "" : region.error().context);
        return d.desc();
    };
    const auto grid = [](const ed::AirCase& c) { return ed::grid_line(c.resolved); };
    spade::scene::SceneDesc s = placed("phantom3_class.model.yaml");
    ASSERT_FALSE(s.airflow.empty());
    const Result<ed::AirCases> cases = ed::air_cases(s, s.airflow.back());
    ASSERT_TRUE(cases.has_value()) << cases.error().context;
    EXPECT_EQ(grid(cases->body), "104\xc2\xb3, dx 12 mm, R/dx 10");
    EXPECT_FALSE(cases->body.model.parts.empty()) << "the body's case keeps the drone's parts";
    EXPECT_EQ(grid(cases->discs), "32\xc2\xb3, dx 39 mm, R/dx 3");
    EXPECT_TRUE(cases->discs.model.parts.empty()) << "the fallback is discs only";
    const auto box = [](const ed::AirCase& c) { return static_cast<float>(c.resolved.base.nx) * c.resolved.base.dx; };
    EXPECT_NEAR(box(cases->discs), box(cases->body), 1e-6f) << "over the same box";
    EXPECT_TRUE(cases->loses_parts) << "the body is a part a 12 mm cell holds";

    Result<ed::AirStart> started = ed::cpu_air_starter("told so")(*cases);
    ASSERT_TRUE(started.has_value()) << started.error().context;
    EXPECT_EQ(started->backend, "CPU: told so; discs only, the body needs the GPU");
    EXPECT_EQ(started->run.run_label(), "actuator disc \xc2\xb7 32\xc2\xb3, dx 39 mm, R/dx 3, h 1 ms");
    const std::string band = ed::air_band("air at 1/43 speed", started->run.run_label(), started->backend);
    EXPECT_EQ(band, "air at 1/43 speed \xc2\xb7 actuator disc \xc2\xb7 CPU: told so; discs only, the body needs the GPU "
                    "\xc2\xb7 32\xc2\xb3, dx 39 mm, R/dx 3, h 1 ms");
    EXPECT_EQ(ed::band_floor(band), "air at 1/43 speed \xc2\xb7 CPU");

    s.airflow.back().cells = spade::physics::airflow::GridShape{20, 20, 20, 0.039f};
    const Result<ed::AirCases> coarse = ed::air_cases(s, s.airflow.back());
    ASSERT_TRUE(coarse.has_value()) << coarse.error().context;
    EXPECT_EQ(grid(coarse->body), "20\xc2\xb3, dx 39 mm, R/dx 3");
    EXPECT_EQ(grid(coarse->discs), grid(coarse->body));
    EXPECT_FALSE(coarse->loses_parts) << "at 39 mm every part is drawn only";
    Result<ed::AirStart> plain = ed::cpu_air_starter("told so")(*coarse);
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    EXPECT_EQ(plain->backend, "CPU: told so");

    // Detailed's step for the body's cell is L6's (L22): the Crazyflie's
    // 1.75 mm over its wake speed, under 1 ms.
    const spade::scene::SceneDesc small = placed("crazyflie_class.model.yaml");
    ASSERT_FALSE(small.airflow.empty());
    const Result<ed::AirCases> fine = ed::air_cases(small, small.airflow.back());
    ASSERT_TRUE(fine.has_value()) << fine.error().context;
    ASSERT_TRUE(fine->body.resolved.spec.time.step.has_value());
    EXPECT_EQ(*fine->body.resolved.spec.time.step,
              an::live_step_s(fine->body.resolved.spec, fine->body.model, fine->body.resolved.base.dx));
    EXPECT_LT(*fine->body.resolved.spec.time.step, 0.001f);
}

// L13 (4) (the lead, 2026-11-04; Physics' wake-cost measurement): each level
// states its cost in a sentence under the inspector's Physics combo, against
// the same run at CFL 0.5 (the lead's review, 2026-11-04: the reference is
// named so a reader can check it). Real time's wake is within a few percent (the
// Phantom's 1%; the reference quad's own window noise is 2-4%) and its pressure
// solve is loose on nearly every step, which moves neither the thrust nor the
// wake past noise (L32: the run line's word "loose", kLiveLooseResidual;
// "unconverged" is for past 7.1e-2); Detailed's wake is within 4% and the load
// on the body within about 40%.
TEST(EditorAirflow, EachLevelStatesItsCostInOneSentence) {
    EXPECT_STREQ(ed::air_level_cost(ed::AirLevel::real_time),
                 "Discs only: the wake is within a few percent of the same run at CFL 0.5, and the pressure is loose "
                 "on nearly every step, moving neither the thrust nor the wake past noise.");
    EXPECT_STREQ(ed::air_level_cost(ed::AirLevel::detailed),
                 "The body in the air: the wake is within 4% of the same run at CFL 0.5, and the load on the body "
                 "within about 40%.");
}

// D10 (Cameron's direction via the lead, 2026-11-03): the session's physics
// level picks what Run air resolves. Detailed, the default, is the body's
// case; Real time is discs only (L17): 48^3 on the device, 16^3 on the
// CPU, over the region's box, so its band says nothing of the body. The inspector's line follows the level, resolved
// again when it changes.
TEST(EditorAirflow, TheRealTimeLevelFliesTheDiscsAndTheLineFollowsTheLevel) {
    auto d = ed::make_scene_document(support::sample_scene(), {});
    ed::AddModelFile add;
    add.file = (std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "phantom3_class.model.yaml")
                   .generic_string();
    add.vehicle.name = "phantom";
    ASSERT_TRUE(ed::apply(d, add).has_value());
    ASSERT_TRUE(ed::apply(d, ed::AddAirflowRegion{"phantom"}).has_value());
    const spade::scene::SceneDesc s = d.desc();
    const spade::scene::SceneAirflowRegion& region = s.airflow.back();
    EXPECT_EQ(ed::EditorPanels{}.air_level, ed::AirLevel::detailed) << "Detailed is the session's default";
    const Result<ed::AirCases> fast = ed::air_cases(s, region, ed::AirLevel::real_time);
    ASSERT_TRUE(fast.has_value()) << fast.error().context;
    EXPECT_EQ(ed::grid_line(fast->body.resolved), "48\xc2\xb3, dx 26 mm, R/dx 5") << "the device's";
    EXPECT_EQ(ed::grid_line(fast->discs.resolved), "16\xc2\xb3, dx 78 mm, R/dx 2") << "the CPU's";
    EXPECT_TRUE(fast->body.model.parts.empty());
    EXPECT_FALSE(fast->loses_parts);
    // Only Real time steps at L13's CFL 2 (L22): the Phantom's 26 mm takes
    // the 4 ms cap, where Detailed's body at 12 mm keeps L6's 1 ms.
    ASSERT_TRUE(fast->body.resolved.spec.time.step.has_value());
    EXPECT_EQ(*fast->body.resolved.spec.time.step, an::kLiveLevelStepCapS);
    const Result<ed::AirCases> detailed = ed::air_cases(s, region);
    ASSERT_TRUE(detailed.has_value()) << detailed.error().context;
    ASSERT_TRUE(detailed->body.resolved.spec.time.step.has_value());
    EXPECT_EQ(*detailed->body.resolved.spec.time.step, 0.001f);
    Result<ed::AirStart> started = ed::cpu_air_starter("told so")(*fast);
    ASSERT_TRUE(started.has_value()) << started.error().context;
    EXPECT_EQ(started->backend, "CPU: told so");

    ed::AirLineCache cache;
    EXPECT_EQ(ed::air_line(cache, 1, 1, s, region).text, "104\xc2\xb3, dx 12 mm, R/dx 10");
    EXPECT_EQ(ed::air_line(cache, 1, 1, s, region, ed::AirLevel::real_time).text, "48\xc2\xb3, dx 26 mm, R/dx 5");
    EXPECT_EQ(cache.resolves, 2u);
    EXPECT_EQ(ed::air_line(cache, 1, 1, s, region, ed::AirLevel::real_time).resolves, 2u) << "the same level: kept";
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
