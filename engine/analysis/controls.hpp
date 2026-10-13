// A control program: the controls a run is flown with over time (I1; the CFD
// spec §5, "the control program"). The controls are held piecewise
// constant: each point holds from its time until the next point's.
//
// One format, read and written only here (TD-9): a run folder holds it,
// LiveRun records the live flight as one, and flow_movie (--controls) and
// Kat's host fly it. showcase_program() is the one built-in program every
// front end flies, so a film can name it.
//
//   version: 1
//   points:
//     - {t: 0, armed: false, throttle: 0, pitch_deg: 0, roll_deg: 0, yaw_deg: 0}
//     - {t: 0.2, armed: true, throttle: 0.5, pitch_deg: 0, roll_deg: 0, yaw_deg: 0}
//
// Display-free. Owned by Interface (INT-6).

#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "analysis/case.hpp"
#include "core/error.hpp"

namespace spade::analysis {

inline constexpr uint32_t kControlsFileVersion = 1;

struct ControlPoint {
    float t_s = 0.0f;  // simulated s from the run's start
    Controls controls;
};

// Points in increasing time, the first at 0.
struct ControlProgram {
    std::vector<ControlPoint> points;
};

// The controls in force at `t_s`: the last point at or before it, the last
// point after the end, and disarmed for an empty program.
[[nodiscard]] Controls controls_at(const ControlProgram& program, float t_s) noexcept;

// Every problem, by field ("points[2].t"): no points, a first point not at 0,
// times that don't increase or aren't finite, and each point's controls as
// a case checks them (throttle in [0, 1], finite angles).
[[nodiscard]] std::vector<CaseIssue> program_issues(const ControlProgram& program);

// The canonical text, and back. Writing refuses an invalid program, and
// reading refuses an unknown key, another version (schema_mismatch) or an
// invalid program, naming its first problem's field.
[[nodiscard]] Result<std::string> program_to_yaml(const ControlProgram& program);
[[nodiscard]] Result<ControlProgram> program_from_yaml(std::string_view text);
[[nodiscard]] Result<void> save_program(const ControlProgram& program, const std::filesystem::path& file);
[[nodiscard]] Result<ControlProgram> load_program(const std::filesystem::path& file);

// The built-in program: disarmed, armed at mid throttle (the spin-up sheds the
// start-up ring), then a pitch, a roll, more throttle and a yaw, so a film of
// it shows the drone in several attitudes and thrust states. Mid throttle is
// not a hover trim: the drone is held, so it need not be.
[[nodiscard]] const ControlProgram& showcase_program();
// How long to fly the showcase program: past its last point by its hold.
[[nodiscard]] float showcase_seconds() noexcept;

}  // namespace spade::analysis
