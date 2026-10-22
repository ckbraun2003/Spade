// The tour's airflow chapter, display-free: the library's reference quad
// (N4) flown live at 32^3 through LiveRun, the flow views drawn over it, and
// the frames the on-screen downwash check reads. The window (AirflowSession,
// scene_sessions.hpp) only draws what this decides.
//
// TWO MODELS, ONE DRONE. The run flies the quad with its parts cleared: at
// 32^3 (25 mm cells) its arms and props are thinner than a cell, so the
// rotors act on the air as actuator discs and nothing solid sits in the
// grid. The window draws the same model with its parts, posed by the run's
// body, so what is seen is the drone that flies.
//
// THE CHECK'S PREMISE. Test/Docs' downwash() judges frames taken while the
// controls are the ones set at arming (DownwashFrame::controls_constant);
// showcase_program() arms at hover at 0.2 s and holds it until the pitch at
// 1.2 s, so the hover segment is what is judged.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "end_goal_checks.hpp"
#include "render/flow_lines.hpp"
#include "render/view_mode.hpp"
#include "scene/model_file.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/rotor.hpp"

namespace spade::sandbox::airflow {

inline constexpr float kBoxM = 0.8f;
inline constexpr float kSpacingM = 0.025f;  // 32^3, live
inline constexpr float kStepS = 0.001f;
// Until the chain pin moves them into the case (INT-11): the idle duty and
// the battery's state of charge the run spins the chain at.
inline constexpr double kIdleDuty = 0.05;
inline constexpr double kStateOfCharge = 1.0;
// The vortex cores: an iso of lambda2-hat at this fraction of the frame's
// deepest value, drawn only once the deepest is below the floor (flow_movie's
// floor: a shallower minimum is noise, not a core). A third, not flow_movie's
// sixth at 64^3: at 32^3 a sixth draws the noise in the slow air too.
inline constexpr float kVortexFraction = 1.0f / 3.0f;
inline constexpr float kVortexFloor = -5.0f;
// The window plays the run at a quarter of real speed, said so on screen
// (a stated speed, Videos/CATEGORIES.md's Feature rule): the 1.2 s flown
// would be over in 1.2 s of video otherwise.
inline constexpr float kPlayback = 0.25f;

// The view's HUD status line (the lead, 2026-10-22): the run's pressure line
// (LiveRun::pressure_line()), then what the render path does not draw
// (render_gap_line()), joined by a dot. In the warning tone when the pressure
// was unconverged, so a stalled or capped run catches the eye; empty for a
// converged run on a path that draws everything.
struct StatusLine {
    std::string text;
    bool warning = false;
};

[[nodiscard]] inline StatusLine status_line(std::string_view pressure, std::string_view gap) {
    StatusLine out{std::string(pressure), !pressure.empty()};
    if (!gap.empty()) out.text += (out.text.empty() ? "" : " \xc2\xb7 ") + std::string(gap);
    return out;
}

// The library quad, as the chapter flies and draws it.
struct LiveQuad {
    vehicles::ModelType draw_model;  // with its parts: what the window draws
    vehicles::ModelType run_model;   // its parts cleared: what the run flies
    analysis::ResolvedCase resolved;
    analysis::ControlProgram program;  // showcase_program(showcase_trim())
};

namespace detail {

[[nodiscard]] inline bool same_controls(const analysis::Controls& a, const analysis::Controls& b) noexcept {
    return a.armed == b.armed && a.throttle == b.throttle && a.pitch_deg == b.pitch_deg &&
           a.roll_deg == b.roll_deg && a.yaw_deg == b.yaw_deg;
}

}  // namespace detail

// When `program` first arms the drone; none when it never does.
[[nodiscard]] inline std::optional<float> armed_at(const analysis::ControlProgram& program) {
    for (const analysis::ControlPoint& p : program.points) {
        if (p.controls.armed) return p.t_s;
    }
    return std::nullopt;
}

// When `program` first changes the controls set at arming: the end of the
// hover the check judges. None when it never does.
[[nodiscard]] inline std::optional<float> hover_ends_at(const analysis::ControlProgram& program) {
    const std::optional<float> armed = armed_at(program);
    if (!armed) return std::nullopt;
    const analysis::Controls at_arming = analysis::controls_at(program, *armed);
    for (const analysis::ControlPoint& p : program.points) {
        if (p.t_s > *armed && !detail::same_controls(p.controls, at_arming)) return p.t_s;
    }
    return std::nullopt;
}

// Whether the controls at `t_s` are still the ones set at arming: from the
// arming point until the program next changes them, never again after.
// Times compare in float, as controls_at() compares them.
[[nodiscard]] inline bool controls_constant(const analysis::ControlProgram& program, double t_s) {
    const std::optional<float> armed = armed_at(program);
    const float t = static_cast<float>(t_s);
    if (!armed || t < *armed) return false;
    const std::optional<float> ends = hover_ends_at(program);
    return !ends || t < *ends;
}

// Loads `model_file` (the library's reference_quad.model.yaml) and resolves
// the chapter's case for it: box 0.8 m, `spacing` cells (25 mm, 32^3, unless
// a test asks for coarser), 1 ms steps, flown under the showcase program
// until its hover ends (the pitch at 1.2 s): what the check judges, at
// 32^3's live cost. Refused, in the refusal's words, when the file does not
// load, the model has no propulsion chain, the chain cannot trim the
// showcase, or the case does not resolve.
[[nodiscard]] inline Result<LiveQuad> live_quad(const std::filesystem::path& model_file,
                                                float spacing = kSpacingM) {
    Result<scene::ModelFile> file = scene::load_model_file(model_file);
    if (!file) return std::unexpected(file.error());
    LiveQuad q;
    q.draw_model = std::move(file->model);
    if (!q.draw_model.propulsion) {
        return std::unexpected(Error{Code::invalid_argument, "'" + q.draw_model.name + "' (" +
                                                                 model_file.generic_string() +
                                                                 ") has no propulsion chain, so its rotors can't turn"});
    }
    q.run_model = q.draw_model;
    q.run_model.parts.clear();

    analysis::AnalysisCase c;
    c.model = analysis::pin_model(model_file.generic_string(), q.run_model);
    c.power = analysis::Power{.idle = static_cast<float>(kIdleDuty),
                              .state_of_charge = static_cast<float>(kStateOfCharge)};
    c.rotors.assign(q.run_model.rotors.size(), analysis::RotorSetup{});
    c.grid.box = kBoxM;
    c.grid.spacing = spacing;
    c.time.step = kStepS;
    const Result<analysis::ShowcaseTrim> trim = analysis::showcase_trim(c, q.run_model);
    if (!trim) return std::unexpected(trim.error());
    q.program = analysis::showcase_program(*trim);
    c.time.duration = hover_ends_at(q.program).value_or(analysis::showcase_seconds());
    auto resolved = analysis::resolve(c, q.run_model);
    if (!resolved) {
        const analysis::CaseIssue& first = resolved.error().front();
        return std::unexpected(Error{Code::invalid_argument, "the airflow case: " + first.field + ": " + first.message});
    }
    q.resolved = std::move(*resolved);
    return q;
}

// The rotors as the checks see them: posed from the run's rows by its body.
[[nodiscard]] inline std::vector<end_goal::RotorProbe> probes_of(const analysis::LiveRun& run) {
    std::vector<end_goal::RotorProbe> probes;
    probes.reserve(run.rotor_rows().size());
    for (const vehicles::RotorRow& row : run.rotor_rows()) probes.push_back(end_goal::probe_rotor(row, run.body()));
    return probes;
}

// The frame just published, as the downwash check reads it, timed from
// arming; none before the drone is armed.
[[nodiscard]] inline std::optional<end_goal::DownwashFrame> downwash_frame(const FlowVolume& volume,
                                                                           const analysis::LiveRun& run,
                                                                           const analysis::ControlProgram& program) {
    const std::optional<float> armed = armed_at(program);
    const double t = run.state().t_s;
    if (!armed || static_cast<float>(t) < *armed) return std::nullopt;
    return end_goal::sample_downwash(volume, probes_of(run), std::max(0.0, t - static_cast<double>(*armed)),
                                     end_goal::tilt_of(run.body().orient), controls_constant(program, t));
}

// The colour scale's top: twice the momentum-theory induced speed at hover,
// the far wake's, so the colours hold still while the wake develops.
[[nodiscard]] inline float speed_top(const vehicles::ModelType& model, float density, float gravity) {
    if (model.rotors.empty()) return 1.0f;
    const float thrust = model.body.mass * gravity / static_cast<float>(model.rotors.size());
    return std::max(1.0f, 2.0f * vehicles::rotor_hover_induced_velocity(thrust, density, model.rotors.front().radius));
}

// What the chapter shows over the drone while it flies, for this frame's
// volume: the vortex cores (once there are any) and streamlines through each
// disc, coloured by speed: the lines over [0, top], the cores over
// [0, top / 2]. The slice has its own view
// (wake_slice()): over the drone it hides half the wake.
[[nodiscard]] inline render::ViewSpec airflow_view(const FlowVolume& volume,
                                                   std::span<const end_goal::RotorProbe> rotors, float top) {
    render::ViewSpec spec;
    spec.base = render::ViewBase::shaded;

    const float deepest = volume.header.channel_min[kFlowLambda2Hat];
    if (deepest < kVortexFloor) {
        render::FlowViewSpec iso;
        iso.kind = render::FlowView::vortex_iso;
        iso.quantity = render::FlowQuantity::lambda2_hat;
        iso.iso_value = deepest * kVortexFraction;
        iso.range_max = 0.5f * top;  // the induced speed: the cores sit in the jet's slower shear layer
        spec.layers.push_back(std::move(iso));
    }

    render::FlowViewSpec lines;
    lines.kind = render::FlowView::streamlines;
    lines.range_max = top;
    lines.lines = render::StreamlineOptions::live();
    for (const end_goal::RotorProbe& r : rotors) {
        for (const glm::vec3& s : render::seed_ring(r.hub - r.axis * (0.5f * r.radius), r.axis, 0.7f * r.radius, 8)) {
            lines.seeds.push_back(s);
        }
    }
    if (!lines.seeds.empty()) spec.layers.push_back(std::move(lines));
    return spec;
}

// The wake's slice, shown on its own after the run: speed over [0, top] on a
// plane standing through the two farthest-apart hubs, as tall and wide as the
// box.
[[nodiscard]] inline render::ViewSpec wake_slice(const FlowVolume& volume,
                                                 std::span<const end_goal::RotorProbe> rotors, float top) {
    render::ViewSpec spec;
    spec.base = render::ViewBase::shaded;
    render::FlowViewSpec slice;
    slice.kind = render::FlowView::slice;
    slice.quantity = render::FlowQuantity::speed;
    slice.range_max = top;
    const float size = volume.header.cell * static_cast<float>(volume.header.nx);
    slice.plane.center = volume.header.origin + 0.5f * size * glm::vec3(1.0f);
    slice.plane.width = size;
    slice.plane.height = size;
    slice.plane.cells_u = 2u * volume.header.nx;
    slice.plane.cells_v = 2u * volume.header.ny;
    float widest = 0.0f;
    for (std::size_t i = 0; i < rotors.size(); ++i) {
        for (std::size_t j = i + 1; j < rotors.size(); ++j) {
            const glm::vec3 d = rotors[j].hub - rotors[i].hub;
            const glm::vec3 flat(d.x, 0.0f, d.z);
            if (glm::length(flat) > widest) {
                widest = glm::length(flat);
                slice.plane.right = flat / widest;
                slice.plane.center = glm::vec3(0.5f * (rotors[i].hub.x + rotors[j].hub.x), slice.plane.center.y,
                                               0.5f * (rotors[i].hub.z + rotors[j].hub.z));
            }
        }
    }
    spec.layers.push_back(std::move(slice));
    return spec;
}

// The orbit camera's yaw that puts its eye square to `plane` (the eye sits
// along (sin yaw, cos yaw) from its target; orbit_camera.hpp).
[[nodiscard]] inline float facing_yaw(const render::FieldLayer& plane) {
    return std::atan2(-plane.right.z, plane.right.x);
}

}  // namespace spade::sandbox::airflow
