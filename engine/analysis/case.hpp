// The drone CFD analysis's case: one drone held at the centre of a box of air,
// flown by hand or by a constant program, and what a run of it needs (INT-5,
// INT-6; docs/design/interface/plans/2026-10-12-drone-builder-cfd-design.md
// §4, §7). Display-free. A host (Kat's builder, the sandbox's airflow scene)
// reads and writes a case only through this header (DBC-004).
//
// THE FILE, `.case.yaml`, version 2 (INT-11: it pins the model's propulsion
// chain and carries the drone's power; a version-1 file is refused, never
// migrated). Written in one fixed order, every float
// with format_float's 9 significant digits, so a case saved twice is the same
// bytes. An absent value (std::nullopt) is absent from the file, never a 0
// standing for "default" or "none". Angles are DEGREES here as in the file
// (the `_deg` fields): blade tables and polars arrive in degrees, and a degree
// written, read and written again is the same text. Physics gets radians.
//
// THE FRAME is the drone's body frame, Spade's and Kat's alike: +X the nose,
// +Y up, +Z the right side. Lengths are metres, times seconds.
//
// Owned by Interface (INT-6).

#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "vehicles/model_type.hpp"

namespace spade::analysis {

inline constexpr uint32_t kCaseFileVersion = 2;

// The flight condition (Physics' plan A domain conditions); they differ at the
// floor. Forward flight at v moves the air AND the ground past the drone at
// v (the floor a rolling road); hover in a wind moves only the air.
enum class FlightKind : uint32_t { hover = 0, forward = 1, hover_in_wind = 2 };

enum class RotorModel : uint32_t { disc = 0, line = 1, blade_resolved = 2 };

// The model a case is for, as a scene pins its world: by file, and by its
// three identities when the case was saved. A changed model, changed parts
// or a changed propulsion chain is refused until the case is re-pinned.
struct ModelRef {
    std::string file;         // relative to the case file's folder, so a project can move
    uint64_t identity = 0;    // vehicles::model_identity(): the body, rotors, drag bodies and mounts
    uint64_t parts = 0;       // vehicles::parts_identity(): the parts' geometry, mounts and surfaces
    uint64_t propulsion = 0;  // vehicles::propulsion_identity() of the model's chain; 0 for none
};

// The model's three pins under `file`, as a host pins a case to it.
[[nodiscard]] ModelRef pin_model(std::string file, const vehicles::ModelType& model);

// The drone-box inputs (Q4), held for the whole run (A4's constant program).
// The attitude is the drone sim box's: yaw about +Y, then pitch about +Z,
// then roll about +X. + pitch puts the nose up, + roll the right side down,
// + yaw turns the nose left.
struct Controls {
    bool armed = false;      // disarmed rotors are stopped
    float throttle = 0.0f;   // 0..1
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float yaw_deg = 0.0f;
};

// The drone's power (INT-11): the arm rule's idle and the pack's state of
// charge. The chain itself is the model's (ModelType::propulsion). Absent,
// each is Physics' announced default, which resolve() fills and reports.
struct Power {
    std::optional<float> idle;             // the armed duty at zero throttle, [0, 1)
    std::optional<float> state_of_charge;  // the pack's, (0, 1]
};

inline constexpr float kDefaultIdleDuty = 0.05f;      // Physics, 2026-10-19
inline constexpr float kDefaultStateOfCharge = 1.0f;  // a full pack

struct Flight {
    FlightKind kind = FlightKind::hover;
    float speed = 0.0f;        // m/s: the forward speed, or the wind's; 0 in hover
    // Where the air comes from, about +Y from the nose: 0 head-on (straight
    // forward flight), + from the left, - from the right (sideslip, or a
    // crosswind). Unused in hover.
    float heading_deg = 0.0f;
    std::optional<float> floor_height;  // m below the hub plane; absent for no floor
};

// ISA at sea level, 15 C.
struct Fluid {
    float density = 1.225f;                  // kg/m3
    float kinematic_viscosity = 1.4607e-5f;  // m2/s
};

struct GridSetup {
    std::optional<float> box;      // the cube's side, m; absent for kDefaultBoxReaches x the drone's reach
    std::optional<float> spacing;  // the base cell, m; absent for the box / kDefaultCellsAcross
    uint32_t nests = 0;            // static 2:1 levels around the drone (R1), 0..kMaxNests
};

// A blade (Q3). Physics' P4 reads the same fields in radians, with the radius
// its rotor's (resolve() takes it from the model; the case never holds a
// second one that could disagree).
struct BladeStation {
    float r_over_radius = 0.0f;  // (0, 1], increasing from hub to tip
    float chord = 0.0f;          // m
    float twist_deg = 0.0f;      // about the pitch axis, + nose up
    std::string airfoil;         // an Airfoil's id; one without polars is a flat plate, "estimated"
};

struct BladeDefinition {
    std::string name;  // what a rotor's setup names
    uint32_t blade_count = 2;
    float hub_radius = 0.0f;  // m; below each rotor's radius that uses it
    float collective_deg = 0.0f;
    std::vector<BladeStation> stations;
};

struct Polar {
    float reynolds = 0.0f;
    std::vector<float> alpha_deg;  // increasing
    std::vector<float> cl;
    std::vector<float> cd;
};

struct Airfoil {
    std::string id;
    std::string source;  // the file and citation, or "estimated"
    std::vector<Polar> polars;
};

// rotors[i] is the model's rotor i; resolve() refuses a count that differs.
struct RotorSetup {
    RotorModel model = RotorModel::disc;
    std::string blade;  // a BladeDefinition's name; the line and blade-resolved need one
};

struct TimeSetup {
    float duration = 1.0f;      // simulated s
    float startup = 0.0f;       // s discarded before averaging, below the duration
    std::optional<float> step;  // the fluid step, s; absent for the solver's own rule
};

struct Outputs {
    std::optional<float> history_every;   // s between force and moment samples; absent for every step
    std::optional<float> snapshot_every;  // s between field snapshots; absent for none
};

struct AnalysisCase {
    ModelRef model;
    Controls controls;
    Power power;
    Flight flight;
    Fluid fluid;
    GridSetup grid;
    std::vector<RotorSetup> rotors;
    std::vector<BladeDefinition> blades;
    std::vector<Airfoil> airfoils;
    TimeSetup time;
    Outputs outputs;
};

inline constexpr float kDefaultBoxReaches = 10.0f;   // the default box: this many of the drone's reach
inline constexpr uint32_t kDefaultCellsAcross = 128;  // the default base grid (the airflow spec's 128^3)
inline constexpr uint32_t kMaxNests = 3;              // R1
inline constexpr float kWallCells = 2.0f;             // a wall feature's cells (P2)
inline constexpr float kLineCellsPerRadius = 20.0f;   // the actuator line's R/dx (P4)
inline constexpr float kBladeCellsPerChord = 32.0f;   // blade-resolved's cells across the chord (P5)

// One problem: the field it names ("blades[0].stations[1].chord") and why,
// with the value it needs (DBC-003).
struct CaseIssue {
    std::string field;
    std::string message;
};

// Every problem in the case's own fields, without the model, in field order:
// what a host's form shows at once. Empty for a valid case.
[[nodiscard]] std::vector<CaseIssue> case_issues(const AnalysisCase& c);

// The first of case_issues(), as "field: why".
[[nodiscard]] Result<void> validate_case(const AnalysisCase& c);

// The file's text, and back. Both validate. Reading refuses an unknown or
// duplicate key with its line, and a version other than kCaseFileVersion as
// a schema mismatch.
[[nodiscard]] Result<std::string> case_to_yaml(const AnalysisCase& c);
[[nodiscard]] Result<AnalysisCase> case_from_yaml(std::string_view text);
[[nodiscard]] Result<AnalysisCase> load_case(const std::filesystem::path& file);
[[nodiscard]] Result<void> save_case(const AnalysisCase& c, const std::filesystem::path& file);

// Where a resolved value came from (honesty rule 1).
enum class Source : uint32_t { given = 0, default_value = 1, derived = 2 };

struct ResolvedRotor {
    RotorModel model = RotorModel::disc;
    float radius = 0.0f;                     // m
    Source radius_source = Source::derived;  // always the model rotor's
    float cells_across_radius = 0.0f;        // R over the finest cell
    bool estimated_polars = false;           // a station's airfoil has no polars: flat plate
};

// A case with its box and spacing filled, and what follows from it. `spec`
// is exactly what a run receives (DBC-016); `hash` names the run. The other
// absent values keep their meaning (no floor, the solver's step, every step,
// no snapshots): they are not defaults to fill.
struct ResolvedCase {
    AnalysisCase spec;
    uint64_t hash = 0;  // rng::fnv1a64 of case_to_yaml(spec)
    Source box_source = Source::given;
    Source spacing_source = Source::given;
    Source idle_source = Source::given;  // spec.power.idle's
    Source soc_source = Source::given;   // spec.power.state_of_charge's
    float finest_spacing = 0.0f;         // spacing / 2^nests, around the drone
    physics::airflow::GridShape base{};  // the base grid
    physics::airflow::DomainBc domain{};  // the drone box's faces: open, the wind as ambient, the floor a wall
    uint64_t base_bytes = 0;             // physics::airflow::fluid_bytes(base); the nests' come with R1
    std::vector<ResolvedRotor> rotors;
};

// The case against `model`: every problem, not the first. Refused, besides
// case_issues(): an invalid model; a model changed since the case pinned it
// (any of its three pins); a rotor without its setup; an idle at or above the
// model's hover duty (armed at zero throttle it would climb); a missing
// blade, or one whose
// hub reaches its rotor's radius; and whatever the finest cell cannot resolve
// (a wall feature, the line's R, blade-resolved's chord) or the solver cannot
// hold, each with the spacing it needs (DBC-013..017).
[[nodiscard]] std::expected<ResolvedCase, std::vector<CaseIssue>> resolve(const AnalysisCase& c,
                                                                          const vehicles::ModelType& model);

// The case's model, read from its file beside the case (`case_file`'s
// folder) and checked against all three pins in one place. Refused naming
// the file: one that does not load, or a model changed since the case
// pinned it, naming the pin ("model.propulsion: ... re-pin").
[[nodiscard]] Result<vehicles::ModelType> load_case_model(const AnalysisCase& c,
                                                          const std::filesystem::path& case_file);

}  // namespace spade::analysis
