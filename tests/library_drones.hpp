// ---------------------------------------------------------------------------
// tests/library_drones.hpp -- the default library's six drones (Task L5 of
// docs/design/core/plans/2026-10-21-default-drone-library-plan.md):
// phantom3_class, iris_plus_class, fpv_5inch and crazyflie_class as direct
// AirframeSpecs, and basic_quad_6inch and basic_quad_competition through
// vehicles::basic_quad_airframe(). tests/test_library_drones.cpp generates each
// one's model file and sources sidecar (scene/model_sources.hpp) from these,
// and checks them.
//
// ONE TABLE PER DRONE (TD-9). Every number a drone takes that is not a part
// file's lives in its DroneInputs: the input's field path, its value, its
// citation and, for a value from Kat's files, Kat's own provenance. The
// builder reads the spec from that table alone, and refuses a row it never
// reads; the generator writes the same table as the sidecar's inputs, and the
// checks read their numbers from it. A value another function makes (a fit's
// remainder, basic_quad's proxy radius) is recorded in the table and checked
// against that function's, bit for bit, never computed twice. The blocks are
// the committed library's part files (load_library_part()), never typed here.
//
// THE PATHS (scene/model_sources.hpp, INPUTS). A direct drone's are its
// AirframeSpec's ("parts[0].mass"; "rotors[2].hub.position.x", a component,
// where a vector's components have different grades). A basic quad's are its
// BasicQuadParams' ("half_span_x", "battery.mount.position"), and the spec's
// for what the params do not hold: the operating point, comp's drag, the
// proxy radius and the remainder the fit made. The Crazyflie's fit targets
// are "mass" and "inertia", as a basic quad's are.
//
// THE NUMBERS are the L5 draft's (Core, 2026-10-22) under the rulings on it:
// the lead's "L5's shape" and Physics' "L5's checks" (the plan's Rulings).
// The parts are, in order: body (parts[0]), the four arms (parts[1..4]) and,
// with a fit, the remainder (parts[5]). Rotor k is front-right, front-left,
// rear-left, rear-right with spins +1, -1, +1, -1, basic_quad's THE FRAME.
// ---------------------------------------------------------------------------
#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "library_props.gen.hpp"
#include "scene/library.hpp"
#include "scene/library_sources.hpp"
#include "scene/model_sources.hpp"
#include "scene/part_file.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/basic_quad.hpp"
#include "vehicles/composite_inertia.hpp"
#include "world/mesh_shapes.hpp"

namespace spade::test::library_drones {

using scene::Grade;
using scene::InputValue;
using scene::KatProvenance;
using scene::ModelCheck;
using scene::ModelInput;
using scene::SourceRef;
using vehicles::AirframeSpec;
using vehicles::BasicQuadParams;
using vehicles::PartInertia;
using vehicles::PartShape;

// -- The sources cited (assets/library/sources.yaml) -------------------------

inline constexpr std::string_view kNasa = "nasa_ames_2016";
inline constexpr std::string_view kDji = "dji_phantom3";
inline constexpr std::string_view kIrisPlus = "retail_iris_plus";
inline constexpr std::string_view kUiuc = "uiuc_propdb";
inline constexpr std::string_view kCfDatasheet = "bitcraze_cf21_datasheet";
inline constexpr std::string_view kCfThrust = "bitcraze_pwm_thrust";
inline constexpr std::string_view kLuisLeNy = "luis_leny_2016";
inline constexpr std::string_view kKat700 = "kat_competition_700";

// -- The check points: the cited numbers the checks compare against ----------

// NASA's hover at 2.8 lb, 12.46 N, and its rpm (Table 2; ruling 7, its stated
// numbers).
inline constexpr double kNasaHoverThrust = 12.46;  // N
inline constexpr double kPhantom3HoverRpm = 5300.0;
inline constexpr double kIrisPlusHoverRpm = 5400.0;
// Bitcraze's thrust table weighed the whole Crazyflie 2.0, four motors. The
// row nearest hover: 28.6 g at 17,174 rpm, so one motor lifts 7.15 g. The top
// row, full duty: 23,882 rpm. Hover at 29 g by the table: about 17,270 rpm.
// (Physics, 2026-10-22.)
inline constexpr double kCfRowRpm = 17174.0;
inline constexpr double kCfRowFourMotorsGrams = 28.6;
inline constexpr double kCfTopRowRpm = 23882.0;
inline constexpr double kCfTableHoverRpm = 17270.0;
// Bitcraze's rows as loaded (Physics, 2026-10-22). The top row is at 3.30 V
// under load, PWM 93.75%, 4.44 A (cf_7x16_coreless's resistance is fitted
// there), and the 17,174 rpm row at 3.67 V, PWM 56.25%, 2.11 A. The board alone
// draws 0.24 A (the 0% row), so each motor's bus current is (I - 0.24) / 4.
inline constexpr double kCfTopRowBusVoltage = 3.30;
inline constexpr double kCfTopRowDuty = 0.9375;
inline constexpr double kCfTopRowCraftCurrent = 4.44;
inline constexpr double kCfRowBusVoltage = 3.67;
inline constexpr double kCfRowDuty = 0.5625;
inline constexpr double kCfRowCraftCurrent = 2.11;
inline constexpr double kCfBoardCurrent = 0.24;
// Kat's curve (competition-700.airframe.json, thrust_curve): its hover point,
// norm 0.264 at 9.712 m/s^2, and motor.thrust_per_motor, derived from its
// peak.
inline constexpr double kKatHoverAccel = 9.712;     // m/s^2
inline constexpr double kKatThrustPerMotor = 9.18792;  // N
// The 5-inch class (Physics, 2026-10-22): report-only, no single source.
inline constexpr double kFpvClassLow = 8.0;
inline constexpr double kFpvClassHigh = 14.0;

// -- The masses the derived body masses close on -------------------------------

inline constexpr double kPhantom3Mass = 1.280;  // kg, DJI's (dji_phantom3, published)
inline constexpr double kIrisPlusMass = 1.282;  // kg, with battery (retail_iris_plus)
// The L4a header's estimate (tests/library_props.gen.hpp, uiuc_mit_5x4's run
// choice: "5-inch FPV (0.5 kg, ...)"), which its run and its 8735 rpm assume.
inline constexpr double kFpvMassEstimate = 0.500;  // kg

inline constexpr double kSqrtHalf = 0.7071067811865476;  // sqrt(1/2), as tests/reference_airframe.hpp spells it

// -- Section 4's kappa rule (the lead and Physics, 2026-10-22) ---------------

// Both basic quads' sidecars carry it in `assumptions`, and comp's `kappa`
// note starts with it.
inline constexpr std::string_view kKappaRule =
    "kappa (C_Q = kappa C_T / D) applies only to a prop without a table; no other L5 drone uses it (Physics, "
    "2026-10-22). One exception: basic_quad_competition keeps kappa = 0.022 m from kat_competition_700, graded "
    "assumed. No yaw-torque measurement exists. The legacy CTBR sim never read kappa, so nothing ever tested it. "
    "And it is about twice what catalog props give (k_Q/k_T 9-12 mm). (The lead, 2026-10-22.)";

// The Crazyflie's remainder (Physics, 2026-10-22), verbatim.
inline constexpr std::string_view kCfRemainder =
    "mass-only at the CoM so the composite meets Luis & Le Ny's tensor";

// The FPV frame (the lead, 2026-10-22), verbatim.
inline constexpr std::string_view kFpvFrame = "frame mass = the 0.500 kg estimate minus the parts";

// The comp caveat: Kat's simulator, and kappa's cost (Physics, 2026-10-22).
inline constexpr std::string_view kCompSimulator = "fitted to the AI Grand Prix VQ1 simulator, not hardware";
inline constexpr std::string_view kCompKappaCost =
    "current-limited at 40 A on kappa's torque; the table's C_Q would reach the curve";

inline constexpr std::string_view kDragEstimated =
    "drag: none is given, so the compile estimates it from the parts' shapes (best-effort; "
    "vehicles/airframe_compile.hpp)";

// -- Small helpers -------------------------------------------------------------

[[nodiscard]] inline Error invalid(std::string context) { return Error{Code::invalid_argument, std::move(context)}; }

[[nodiscard]] inline SourceRef cite(Grade grade, std::string_view source, std::string_view note = {}) {
    return SourceRef{std::string(source), grade, std::string(note)};
}

[[nodiscard]] inline SourceRef assumed(std::string_view note) {
    return SourceRef{"", Grade::assumed, std::string(note)};
}

[[nodiscard]] inline InputValue vec_value(const glm::dvec3& v) { return std::vector<double>{v.x, v.y, v.z}; }

// [w, x, y, z], as the sidecar writes an orientation.
[[nodiscard]] inline InputValue quat_value(const glm::dquat& q) {
    return std::vector<double>{q.w, q.x, q.y, q.z};
}

// Row-major, as a part file writes a tensor (glm is column-major: m[col][row]).
[[nodiscard]] inline InputValue mat_value(const glm::dmat3& m) {
    std::vector<double> out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out.push_back(m[c][r]);
    }
    return out;
}

[[nodiscard]] inline glm::dquat identity_turn() { return glm::dquat(1.0, 0.0, 0.0, 0.0); }

// Two values, bit for bit: the same kind, the same length, the same doubles.
[[nodiscard]] inline bool same_value(const InputValue& a, const InputValue& b) {
    if (a.index() != b.index()) return false;
    if (const double* x = std::get_if<double>(&a)) {
        return std::bit_cast<std::uint64_t>(*x) == std::bit_cast<std::uint64_t>(std::get<double>(b));
    }
    const std::vector<double>& u = std::get<std::vector<double>>(a);
    const std::vector<double>& v = std::get<std::vector<double>>(b);
    if (u.size() != v.size()) return false;
    for (std::size_t i = 0; i < u.size(); ++i) {
        if (std::bit_cast<std::uint64_t>(u[i]) != std::bit_cast<std::uint64_t>(v[i])) return false;
    }
    return true;
}

// The turn taking +Y onto the horizontal unit vector (dx, 0, dz): about
// Y x d = (dz, 0, -dx) by 90 degrees, basic_quad's arm turn (sqrt only).
[[nodiscard]] inline glm::dquat arm_turn(double dx, double dz) {
    const double s = 1.0 / std::sqrt(2.0);
    return glm::dquat(s, dz * s, 0.0, -dx * s);
}

// A note basic_quad_assumptions() gives `field`, verbatim; empty when it
// gives none (and the writer then refuses the row by name).
[[nodiscard]] inline std::string default_note(std::string_view field) {
    for (const vehicles::BasicQuadAssumption& a : vehicles::basic_quad_assumptions()) {
        if (a.field == field) return std::string(a.note);
    }
    return {};
}

// The shared-wake line every spec's model leaves out, and the tri-blade
// stand-in's line: basic_quad_assumptions()' own words (TD-9).
[[nodiscard]] inline std::string wake_line() { return default_note("rotors"); }
[[nodiscard]] inline std::string triblade_line() { return default_note("prop.ct_table"); }

// -- The inputs table (TD-9) ---------------------------------------------------

struct DroneInputs {
    std::vector<ModelInput> rows{};

    void add(std::string path, InputValue value, SourceRef ref, std::optional<KatProvenance> kat = std::nullopt) {
        rows.push_back(ModelInput{std::move(path), std::move(value), std::move(ref), std::move(kat)});
    }

    [[nodiscard]] const ModelInput* find(std::string_view path) const {
        for (const ModelInput& row : rows) {
            if (row.path == path) return &row;
        }
        return nullptr;
    }
};

// What a builder reads the table through. The first row missing, or of the
// wrong shape, is kept and reported by problem(); status() also refuses a row
// no read touched, since every row must feed the spec.
class InputReader {
public:
    explicit InputReader(const DroneInputs& inputs) : inputs_(&inputs), used_(inputs.rows.size(), false) {}

    [[nodiscard]] double number(std::string_view path) {
        const ModelInput* row = take(path);
        if (row == nullptr) return 0.0;
        if (const double* v = std::get_if<double>(&row->value)) return *v;
        fail(path, "is a vector, not a number");
        return 0.0;
    }

    [[nodiscard]] glm::dvec3 vec3(std::string_view path) {
        const std::vector<double>* v = numbers(path, 3);
        return v == nullptr ? glm::dvec3(0.0) : glm::dvec3((*v)[0], (*v)[1], (*v)[2]);
    }

    [[nodiscard]] glm::dquat quat(std::string_view path) {
        const std::vector<double>* v = numbers(path, 4);
        return v == nullptr ? identity_turn() : glm::dquat((*v)[0], (*v)[1], (*v)[2], (*v)[3]);
    }

    [[nodiscard]] glm::dmat3 mat3(std::string_view path) {
        const std::vector<double>* v = numbers(path, 9);
        glm::dmat3 m(0.0);
        if (v == nullptr) return m;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) m[c][r] = (*v)[static_cast<std::size_t>(r * 3 + c)];
        }
        return m;
    }

    // A row that records what another function made: it must be that value,
    // bit for bit.
    void same(std::string_view path, const InputValue& made) {
        const ModelInput* row = take(path);
        if (row != nullptr && !same_value(row->value, made)) {
            fail(path, "is not the value the build makes: regenerate the table with the build");
        }
    }

    [[nodiscard]] Result<void> problem() const {
        if (!problem_.empty()) return std::unexpected(invalid(problem_));
        return {};
    }

    [[nodiscard]] Result<void> status() const {
        if (Result<void> r = problem(); !r) return r;
        for (std::size_t i = 0; i < used_.size(); ++i) {
            if (!used_[i]) {
                return std::unexpected(invalid("inputs row '" + inputs_->rows[i].path +
                                               "' is never read: every row feeds the spec (TD-9)"));
            }
        }
        return {};
    }

private:
    [[nodiscard]] const ModelInput* take(std::string_view path) {
        for (std::size_t i = 0; i < inputs_->rows.size(); ++i) {
            if (inputs_->rows[i].path == path) {
                used_[i] = true;
                return &inputs_->rows[i];
            }
        }
        fail(path, "is not in the drone's inputs");
        return nullptr;
    }

    [[nodiscard]] const std::vector<double>* numbers(std::string_view path, std::size_t n) {
        const ModelInput* row = take(path);
        if (row == nullptr) return nullptr;
        const std::vector<double>* v = std::get_if<std::vector<double>>(&row->value);
        if (v == nullptr || v->size() != n) {
            fail(path, "is not a vector of " + std::to_string(n) + " numbers");
            return nullptr;
        }
        return v;
    }

    void fail(std::string_view path, const std::string& why) {
        if (problem_.empty()) problem_ = "inputs row '" + std::string(path) + "' " + why;
    }

    const DroneInputs* inputs_;
    std::vector<bool> used_;
    std::string problem_{};
};

// -- The part files ------------------------------------------------------------

// The order a sidecar pins them in (scene/model_sources.hpp, `parts`).
inline constexpr std::array<scene::PartKind, 4> kPartKinds{scene::PartKind::prop, scene::PartKind::motor,
                                                           scene::PartKind::esc, scene::PartKind::battery};

// A drone's four part files, as the index spells them, and their blocks.
struct DroneParts {
    std::array<std::string, 4> files{};  // in kPartKinds' order
    vehicles::PropBlock prop{};
    vehicles::MotorBlock motor{};
    vehicles::EscBlock esc{};
    vehicles::BatteryBlock battery{};
    std::optional<scene::BladeGeometry> blade{};  // the prop file's, which its mesh is drawn from (L7c1)
};

[[nodiscard]] inline std::string part_file(scene::PartKind kind, std::string_view id) {
    return "parts/" + std::string(id) + std::string(scene::part_suffix(kind));
}

// The part files named by `ids` (in kPartKinds' order), each through its index
// entry and checked against the registry (load_library_part()).
[[nodiscard]] inline Result<DroneParts> load_drone_parts(const scene::Library& library,
                                                         const scene::SourceRegistry& registry,
                                                         const std::array<std::string_view, 4>& ids) {
    DroneParts out;
    for (std::size_t i = 0; i < kPartKinds.size(); ++i) {
        const std::string file = part_file(kPartKinds[i], ids[i]);
        const scene::LibraryEntry* entry = nullptr;
        for (const scene::LibraryEntry& e : library.entries) {
            if (e.file == file) entry = &e;
        }
        if (entry == nullptr) return std::unexpected(invalid("library.yaml has no entry whose file is '" + file + "'"));
        Result<scene::PartFile> part = scene::load_library_part(library, *entry, registry);
        if (!part) return std::unexpected(part.error());
        if (scene::part_kind(*part) != kPartKinds[i]) {
            return std::unexpected(invalid("'" + file + "' is not a " +
                                           std::string(scene::part_kind_name(kPartKinds[i])) + " part"));
        }
        if (const auto* p = std::get_if<scene::PropPart>(&*part)) {
            out.prop = p->block;
            out.blade = p->blade;
        }
        if (const auto* m = std::get_if<scene::MotorPart>(&*part)) out.motor = m->block;
        if (const auto* e = std::get_if<scene::EscPart>(&*part)) out.esc = e->block;
        if (const auto* b = std::get_if<scene::BatteryPart>(&*part)) out.battery = b->block;
        out.files[i] = file;
    }
    return out;
}

// -- The six drones ------------------------------------------------------------

// A drone: its spec's name and file stem, its index name (Interface owns the
// category) and its part files' ids, in kPartKinds' order.
struct DroneDef {
    std::string_view stem;
    std::string_view index_name;
    std::array<std::string_view, 4> parts;
};

inline constexpr std::string_view kCategory = "drones";

inline constexpr std::array<DroneDef, 6> kDrones{{
    {"phantom3_class", "Phantom 3-class quad",
     {{"uiuc_apce_10x5", "phantom3_2312_960kv", "phantom3_4x20a", "phantom3_4s_4480"}}},
    {"iris_plus_class", "Iris+-class quad", {{"uiuc_apce_9x4.5", "iris_920kv", "iris_4x20a", "iris_3s_5100"}}},
    {"fpv_5inch", "5-inch FPV quad", {{"uiuc_mit_5x4", "fpv_2207_1855kv", "fpv_4in1_45a", "fpv_6s_1300"}}},
    {"crazyflie_class", "Crazyflie-class micro",
     {{"uiuc_cfnq_45t1", "cf_7x16_coreless", "cf_brushed_4ch", "cf_1s_250"}}},
    {"basic_quad_6inch", "Basic quad, 6-inch",
     {{"triblade_6inch_scaled", "fpv_2306_1700kv", "basic_4in1_45a", "fpv_6s_1300"}}},
    {"basic_quad_competition", "Basic quad, competition",
     {{"triblade_6inch_scaled", "comp_2207_2450kv", "comp_4in1_45a", "comp_4s"}}},
}};

[[nodiscard]] inline const DroneDef* find_drone(std::string_view stem) {
    for (const DroneDef& d : kDrones) {
        if (d.stem == stem) return &d;
    }
    return nullptr;
}

// The model file and its sidecar, as the index spells them.
[[nodiscard]] inline std::string model_file(std::string_view stem) {
    return "drones/" + std::string(stem) + ".model.yaml";
}
[[nodiscard]] inline std::string sources_file(std::string_view stem) {
    return "drones/" + std::string(stem) + ".sources.yaml";
}

// What a sidecar says beside the inputs.
struct DroneNotes {
    std::string built_by{};
    std::vector<ModelCheck> checks{};
    std::vector<std::string> not_carried{};
    std::vector<std::string> assumptions{};
    std::string caveat{};
};

// -- Rows every drone shares ---------------------------------------------------

inline constexpr std::array<double, 4> kSignX{1.0, 1.0, -1.0, -1.0};
inline constexpr std::array<double, 4> kSignZ{1.0, -1.0, -1.0, 1.0};
inline constexpr std::array<double, 4> kSpins{1.0, -1.0, 1.0, -1.0};

[[nodiscard]] inline std::string indexed(std::string_view list, std::size_t k) {
    return std::string(list) + "[" + std::to_string(k) + "]";
}

// The compile's operating point (the draft's section 2).
inline void add_operating_point(DroneInputs& in) {
    in.add("air_density", 1.225, assumed("ISA sea level: the compile's operating point"));
    in.add("gravity", 9.80665, assumed("standard gravity: the compile's operating point"));
    in.add("state_of_charge", 1.0, assumed("a full pack: the compile's operating point"));
}

inline void read_operating_point(AirframeSpec& s, InputReader& r) {
    s.air_density = r.number("air_density");
    s.gravity = r.number("gravity");
    s.state_of_charge = r.number("state_of_charge");
}

[[nodiscard]] inline vehicles::MountPose read_pose(InputReader& r, const std::string& at) {
    vehicles::MountPose m;
    m.position = r.vec3(at + ".position");
    m.orientation = r.quat(at + ".orientation");
    return m;
}

// -- The props (L7c1) ----------------------------------------------------------

// The props' mesh rows, every drone's: what shapes::prop() needs beyond the
// part file's blade, its diameter and its blade count. No source gives a
// section or a hub, so each is assumed (the realistic drones plan's rule: a
// mesh parameter no source gives is a row).
inline void add_prop_rows(DroneInputs& in, const DroneParts& parts) {
    in.add("meshes.prop.camber", 0.04, assumed("a 4% camber line, drawn: the part file gives no section"));
    in.add("meshes.prop.camber_at", 0.4, assumed("the camber's peak at 40% of the chord"));
    in.add("meshes.prop.thickness_root", 0.12, assumed("12% thick at the blade's first station"));
    in.add("meshes.prop.thickness_tip", 0.08, assumed("8% thick at its last station"));
    in.add("meshes.prop.chord_points", 8.0, assumed("8 points on each surface: a drawing's resolution"));
    in.add("meshes.prop.hub_gap", 0.0005, assumed("0.5 mm between the hub and each blade's root"));
    in.add("meshes.prop.hub_height", 0.05 * parts.prop.diameter, assumed("the hub 5% of the diameter tall"));
    in.add("meshes.prop.hub_segments", 16.0, assumed("16 chords around the hub"));
}

// The props' mesh: the part file's blade, the prop block's diameter and blade
// count, and the rows. Read before the reader's status(), which refuses an
// unread row.
[[nodiscard]] inline shapes::PropMeshSpec read_prop_mesh(const DroneParts& parts, InputReader& r) {
    shapes::PropMeshSpec m;
    m.diameter = parts.prop.diameter;
    m.blades = parts.prop.blades;
    if (parts.blade) {
        m.r_over_r = parts.blade->r_over_r;
        m.c_over_r = parts.blade->c_over_r;
        m.beta_deg = parts.blade->beta_deg;
        m.hub_cutout = parts.blade->hub_cutout;
    }
    m.section.camber = r.number("meshes.prop.camber");
    m.section.camber_at = r.number("meshes.prop.camber_at");
    m.section.thickness_root = r.number("meshes.prop.thickness_root");
    m.section.thickness_tip = r.number("meshes.prop.thickness_tip");
    m.section.chord_points = static_cast<std::uint32_t>(r.number("meshes.prop.chord_points"));
    m.hub_gap = r.number("meshes.prop.hub_gap");
    m.hub_height = r.number("meshes.prop.hub_height");
    m.hub_segments = static_cast<std::uint32_t>(r.number("meshes.prop.hub_segments"));
    return m;
}

// The props' mesh as the table gives it, for a reader of the rows (as
// basic_quad_params() is): problem() only, since the rest of the table is
// another builder's.
[[nodiscard]] inline Result<shapes::PropMeshSpec> prop_mesh_params(const DroneParts& parts,
                                                                   const DroneInputs& inputs) {
    InputReader r(inputs);
    shapes::PropMeshSpec m = read_prop_mesh(parts, r);
    if (Result<void> st = r.problem(); !st) return std::unexpected(st.error());
    return m;
}

// Every hub's prop: prop[k] on rotor k, in its hub's frame at psi = 0 (an
// identity mount), the ccw hand on a +1 rotor and the cw on a -1, one mesh
// per hand. Massless: the propeller's mass is the prop block's
// (AirframeSpec::rotor_parts).
[[nodiscard]] inline Result<void> wear_props(AirframeSpec& s, const DroneParts& parts,
                                             const shapes::PropMeshSpec& m) {
    if (!parts.blade) return std::unexpected(invalid("'" + parts.files[0] + "' carries no blade to draw its prop"));
    const Result<shapes::GeneratedMesh> ccw = shapes::prop(m, shapes::PropHand::ccw);
    if (!ccw) return std::unexpected(ccw.error());
    const Result<shapes::GeneratedMesh> cw = shapes::prop(m, shapes::PropHand::cw);
    if (!cw) return std::unexpected(cw.error());
    for (std::size_t k = 0; k < s.rotors.size(); ++k) {
        const shapes::GeneratedMesh& hand = s.rotors[k].spin_dir > 0.0 ? *ccw : *cw;
        s.rotor_parts.push_back(vehicles::RotorPartSpec{.name = indexed("prop", k),
                                                        .role = "prop",
                                                        .rotor = static_cast<std::uint32_t>(k),
                                                        .mount = {},  // the hub's own frame
                                                        .mesh = hand.mesh,
                                                        .min_feature = hand.min_feature});
    }
    return {};
}

// -- A direct drone ------------------------------------------------------------

// A direct drone's own numbers and their citations (the draft's section 3).
struct DirectQuad {
    double half_x = 0.0;  // m, each hub's x, along the nose
    SourceRef half_x_ref{};
    double half_z = 0.0;  // m, each hub's z, lateral
    SourceRef half_z_ref{};
    double hub_y = 0.0;  // m, each propeller's plane
    SourceRef hub_y_ref{};
    glm::dvec3 body_size{0.0};  // a box at the origin
    SourceRef body_size_ref{};
    double body_mass = 0.0;
    SourceRef body_mass_ref{};
    double arm_radius = 0.0;  // a tube from the origin to each hub, in the y = 0 plane
    SourceRef arm_ref{};
    double arm_mass = 0.0;  // kg, each
    SourceRef arm_mass_ref{};
    glm::dvec3 esc_at{0.0};
    SourceRef esc_ref{};
    glm::dvec3 battery_at{0.0};
    SourceRef battery_ref{};
    SourceRef proxy_ref{};
};

[[nodiscard]] inline double arm_length(const DirectQuad& q) {
    return std::sqrt(q.half_x * q.half_x + q.half_z * q.half_z);
}

// The parts' mass but the body's: the arms, a motor and a propeller per rotor,
// the ESC and the pack.
[[nodiscard]] inline double mass_but_body(const DirectQuad& q, const DroneParts& parts) {
    return 4.0 * q.arm_mass + 4.0 * (parts.motor.mass + parts.prop.mass) + parts.esc.mass + parts.battery.mass;
}

// A direct drone's rows, in the generator's order.
inline void add_direct_rows(DroneInputs& in, const DirectQuad& q, const DroneParts& parts) {
    add_operating_point(in);
    for (std::size_t k = 0; k < 4; ++k) {
        const std::string rotor = indexed("rotors", k);
        in.add(rotor + ".hub.position.x", kSignX[k] * q.half_x, q.half_x_ref);
        in.add(rotor + ".hub.position.y", q.hub_y, q.hub_y_ref);
        in.add(rotor + ".hub.position.z", kSignZ[k] * q.half_z, q.half_z_ref);
        in.add(rotor + ".hub.orientation", quat_value(identity_turn()),
               assumed("identity: thrust along +Y, a tractor prop on top (pusher false)"));
        in.add(rotor + ".spin_dir", kSpins[k],
               assumed("basic_quad's convention: the diagonals turn alike; no hand is published"));
    }
    in.add("parts[0].size", vec_value(q.body_size), q.body_size_ref);
    in.add("parts[0].mass", q.body_mass, q.body_mass_ref);
    in.add("parts[0].position", vec_value(glm::dvec3(0.0)), assumed("at the design origin"));
    in.add("parts[0].orientation", quat_value(identity_turn()), assumed("axis-aligned"));
    const double length = arm_length(q);
    for (std::size_t k = 0; k < 4; ++k) {
        const std::string arm = indexed("parts", k + 1);
        const double x = kSignX[k] * q.half_x;
        const double z = kSignZ[k] * q.half_z;
        in.add(arm + ".size", vec_value(glm::dvec3(q.arm_radius, length, 0.0)), q.arm_ref);
        in.add(arm + ".mass", q.arm_mass, q.arm_mass_ref);
        in.add(arm + ".position", vec_value(glm::dvec3(x / 2, 0.0, z / 2)),
               assumed("the arm's midpoint, from the origin to its hub"));
        in.add(arm + ".orientation", quat_value(arm_turn(x / length, z / length)),
               assumed("+Y turned onto the arm, about Y x d by 90 degrees (basic_quad's turn)"));
    }
    in.add("esc.mount.position", vec_value(q.esc_at), q.esc_ref);
    in.add("esc.mount.orientation", quat_value(identity_turn()), assumed("axis-aligned"));
    in.add("battery.mount.position", vec_value(q.battery_at), q.battery_ref);
    in.add("battery.mount.orientation", quat_value(identity_turn()), assumed("axis-aligned"));
    in.add("imus[0].mount.position", vec_value(glm::dvec3(0.0)), assumed("at the design origin"));
    in.add("imus[0].mount.orientation", quat_value(identity_turn()), assumed("design axes"));
    in.add("imus[0].rate_divider", 1.0, assumed("every step"));
    for (const char* sigma : {"sigma_a", "sigma_g", "sigma_ba", "sigma_bg"}) {
        in.add(std::string("imus[0].") + sigma, 0.0, assumed("noise-free: L5 adds no sensor model"));
    }
    in.add("proxy_radius", length + 0.5 * parts.prop.diameter, q.proxy_ref);
    add_prop_rows(in, parts);
}

[[nodiscard]] inline PartInertia read_record(InputReader& r, const std::string& at, std::string name,
                                             std::string role, PartShape shape) {
    PartInertia p;
    p.mass = r.number(at + ".mass");
    p.position = r.vec3(at + ".position");
    p.orientation = r.quat(at + ".orientation");
    p.shape = shape;
    p.size = r.vec3(at + ".size");
    p.geometry.name = std::move(name);
    p.geometry.role = std::move(role);
    return p;
}

// A direct drone's spec from its rows and its part files: the body box, the
// four arm tubes, the rotors, the mounts, one IMU and the proxy radius. Drag is
// left empty (estimated).
[[nodiscard]] inline AirframeSpec read_direct_quad(const DroneParts& parts, InputReader& r, std::string name) {
    AirframeSpec s;
    s.name = std::move(name);
    s.param_schema_id = 1;
    s.version = 1;
    s.motor = parts.motor;
    s.prop = parts.prop;
    s.esc = parts.esc;
    s.battery = parts.battery;
    read_operating_point(s, r);
    for (std::size_t k = 0; k < 4; ++k) {
        const std::string at = indexed("rotors", k);
        vehicles::RotorSpec rotor;
        rotor.hub.position = glm::dvec3(r.number(at + ".hub.position.x"), r.number(at + ".hub.position.y"),
                                        r.number(at + ".hub.position.z"));
        rotor.hub.orientation = r.quat(at + ".hub.orientation");
        rotor.spin_dir = r.number(at + ".spin_dir");
        s.rotors.push_back(rotor);
    }
    s.parts.push_back(read_record(r, "parts[0]", "body", "body", PartShape::box));
    for (std::size_t k = 0; k < 4; ++k) {
        s.parts.push_back(read_record(r, indexed("parts", k + 1), indexed("arm", k), "arm", PartShape::tube));
    }
    s.esc.mount = read_pose(r, "esc.mount");
    s.battery.mount = read_pose(r, "battery.mount");
    vehicles::ImuSpec imu;
    imu.mount = read_pose(r, "imus[0].mount");
    imu.rate_divider = static_cast<std::uint32_t>(r.number("imus[0].rate_divider"));
    imu.sigma_a = r.number("imus[0].sigma_a");
    imu.sigma_g = r.number("imus[0].sigma_g");
    imu.sigma_ba = r.number("imus[0].sigma_ba");
    imu.sigma_bg = r.number("imus[0].sigma_bg");
    s.imus.push_back(imu);
    s.proxy_radius = r.number("proxy_radius");
    return s;
}

[[nodiscard]] inline Result<AirframeSpec> direct_spec(const DroneParts& parts, const DroneInputs& inputs,
                                                      std::string name) {
    InputReader r(inputs);
    AirframeSpec s = read_direct_quad(parts, r, std::move(name));
    const shapes::PropMeshSpec props = read_prop_mesh(parts, r);
    if (Result<void> st = r.status(); !st) return std::unexpected(st.error());
    if (Result<void> w = wear_props(s, parts, props); !w) return std::unexpected(w.error());
    return s;
}

// The proxy note every direct drone shares, after its own source.
inline constexpr std::string_view kDirectProxy =
    "the hub's reach in the rotor plane plus the propeller's radius, basic_quad's rule";

// -- Phantom 3-class (the draft's 3.1) -----------------------------------------

[[nodiscard]] inline Result<DroneInputs> phantom3_class_inputs(const DroneParts& parts) {
    DirectQuad q;
    q.half_x = 0.12446;
    q.half_x_ref = cite(Grade::measured, kNasa, "NASA's 9.8 in, halved; putting the 9.8 in along the nose is assumed");
    q.half_z = 0.12573;
    q.half_z_ref = cite(Grade::measured, kNasa, "NASA's 9.9 in, halved; the 353.8 mm diagonal checks DJI's 350 mm");
    q.hub_y = 0.030;
    q.hub_y_ref = assumed("a 25 mm motor on the arm: the props sit 5 mm above the shell");
    q.body_size = glm::dvec3(0.18, 0.05, 0.14);
    q.body_size_ref = assumed("the shell around the pack");
    q.arm_radius = 0.008;
    q.arm_ref = assumed("a 16 mm moulded arm from the origin to its hub; the motors sink 3 mm into it");
    q.arm_mass = 0.20 * arm_length(q);
    q.arm_mass_ref = assumed("0.20 kg/m, a moulded arm, times its length");
    q.body_mass = kPhantom3Mass - mass_but_body(q, parts);
    q.body_mass_ref = cite(Grade::derived, kDji,
                           "DJI's 1.280 kg less every other part (tests/library_drones.hpp): the gimbal, camera, "
                           "legs and avionics");
    q.esc_ref = assumed("the four ESCs as one central block, at the origin");
    q.battery_ref = assumed("the pack inside the shell, at the origin, its 120 mm along the nose");
    q.proxy_ref = cite(Grade::derived, kNasa, kDirectProxy);
    DroneInputs in;
    add_direct_rows(in, q, parts);
    return in;
}

[[nodiscard]] inline Result<AirframeSpec> phantom3_class(const DroneParts& parts, const DroneInputs& inputs) {
    return direct_spec(parts, inputs, "phantom3_class");
}

[[nodiscard]] inline DroneNotes phantom3_class_notes() {
    DroneNotes n;
    n.built_by = "tests/library_drones.hpp: phantom3_class(), a direct AirframeSpec";
    n.checks.push_back(ModelCheck{
        "Phantom3HoversAtNasasBaselineRpm", kPhantom3HoverRpm, "rpm",
        cite(Grade::measured, kNasa,
             "hover at 2.8 lb (12.46 N) at 5,300 rpm (NASA Table 2): the rpm at which the model's four k_T carry "
             "12.46 N. The curve is UIUC's APC TE 10x5 at 9.4 in, not DJI's 9450; NASA's figures are stated "
             "numbers (ruling 7); Physics owns the band"),
        std::array<double, 2>{-0.05, 0.05}, false});
    n.assumptions = {wake_line(), std::string(kDragEstimated)};
    n.caveat = "the prop curve is UIUC's APC Thin Electric 10x5 at 9.4 in, a near match, not DJI's 9450; the inertia "
               "is the parts', since none is published";
    return n;
}

// -- Iris+-class (the draft's 3.2) ---------------------------------------------

[[nodiscard]] inline Result<DroneInputs> iris_plus_class_inputs(const DroneParts& parts) {
    DirectQuad q;
    q.half_x = 0.13208;
    q.half_x_ref = cite(Grade::measured, kNasa,
                        "NASA's 10.4 in, halved; the short side along the nose (the wide stance) is assumed");
    q.half_z = 0.20447;
    q.half_z_ref = cite(Grade::measured, kNasa,
                        "NASA's 16.1 in, halved; the diagonal is 486.8 mm, not the retailers' 550 mm");
    q.hub_y = 0.030;
    q.hub_y_ref = assumed("a 24 mm motor on the arm: the props sit 5 mm above the shell");
    q.body_size = glm::dvec3(0.20, 0.05, 0.12);
    q.body_size_ref = assumed("the shell around the 145 mm pack");
    q.arm_radius = 0.008;
    q.arm_ref = assumed("a 16 mm moulded arm from the origin to its hub");
    q.arm_mass = 0.20 * arm_length(q);
    q.arm_mass_ref = assumed("0.20 kg/m, a moulded arm, times its length");
    q.body_mass = kIrisPlusMass - mass_but_body(q, parts);
    q.body_mass_ref = cite(Grade::derived, kIrisPlus,
                           "the retailers' 1.282 kg with battery less every other part (tests/library_drones.hpp): "
                           "the shell, Pixhawk, GPS and legs");
    q.esc_ref = assumed("the four ESCs as one central block, at the origin");
    q.battery_ref = assumed("the pack inside the shell, at the origin, its 145 mm along the nose");
    q.proxy_ref = cite(Grade::derived, kNasa, kDirectProxy);
    DroneInputs in;
    add_direct_rows(in, q, parts);
    return in;
}

[[nodiscard]] inline Result<AirframeSpec> iris_plus_class(const DroneParts& parts, const DroneInputs& inputs) {
    return direct_spec(parts, inputs, "iris_plus_class");
}

[[nodiscard]] inline DroneNotes iris_plus_class_notes() {
    DroneNotes n;
    n.built_by = "tests/library_drones.hpp: iris_plus_class(), a direct AirframeSpec";
    n.checks.push_back(ModelCheck{
        "IrisPlusHoversAtNasasBaselineRpm", kIrisPlusHoverRpm, "rpm",
        cite(Grade::measured, kNasa,
             "hover at 2.8 lb (12.46 N) at 5,400 rpm (NASA Table 2), like for like. It reads about -4.9%: the "
             "9x4.5's static C_T sits about 10% above its low-J points, and sqrt(1.10) is about 4.9% in rpm; its "
             "own mass is report-only (Physics, 2026-10-22)"),
        std::array<double, 2>{-0.05, 0.05}, false});
    n.assumptions = {wake_line(), std::string(kDragEstimated)};
    n.caveat = "the prop curve is UIUC's APC Thin Electric 9x4.5 at 9.6 in, a near match; NASA's spacing gives a "
               "486.8 mm diagonal, not the 550 mm the retailers quote";
    return n;
}

// -- 5-inch FPV (the draft's 3.3) ----------------------------------------------

[[nodiscard]] inline Result<DroneInputs> fpv_5inch_inputs(const DroneParts& parts) {
    DirectQuad q;
    const SourceRef layout = assumed("the plan's 220 mm motor to motor, true X (0.11 m x sqrt(1/2)), until a frame is "
                                     "cited");
    q.half_x = 0.11 * kSqrtHalf;
    q.half_x_ref = layout;
    q.half_z = 0.11 * kSqrtHalf;
    q.half_z_ref = layout;
    q.hub_y = 0.020;
    q.hub_y_ref = assumed("a 20 mm bell on the arm");
    q.body_size = glm::dvec3(0.07, 0.03, 0.04);
    q.body_size_ref = assumed("the stack: flight controller, camera, VTX, receiver and wiring");
    q.body_mass = 0.070;
    q.body_mass_ref = assumed("the stack's mass");
    q.arm_radius = 0.006;
    q.arm_ref = assumed("12 mm across for the solids rule (the real arm is a 5 mm plate), from the origin to its hub");
    // The frame closes on the estimate (the lead, 2026-10-22): the arms carry
    // what the parts and the stack leave of it.
    q.arm_mass = (kFpvMassEstimate - (4.0 * (parts.motor.mass + parts.prop.mass) + parts.esc.mass +
                                      parts.battery.mass) - q.body_mass) / 4.0;
    q.arm_mass_ref = assumed(std::string(kFpvFrame) + " (the L4a header's estimate): the four arms carry it, less the "
                                                      "0.070 kg stack");
    q.esc_ref = assumed("in the stack, at the origin");
    q.battery_at = glm::dvec3(0.0, 0.028, 0.0);
    q.battery_ref = assumed("strapped on top, sunk 2 mm into the stack");
    q.proxy_ref = assumed(std::string(kDirectProxy) + ", on the assumed layout");
    DroneInputs in;
    add_direct_rows(in, q, parts);
    return in;
}

[[nodiscard]] inline Result<AirframeSpec> fpv_5inch(const DroneParts& parts, const DroneInputs& inputs) {
    return direct_spec(parts, inputs, "fpv_5inch");
}

[[nodiscard]] inline DroneNotes fpv_5inch_notes() {
    DroneNotes n;
    n.built_by = "tests/library_drones.hpp: fpv_5inch(), a direct AirframeSpec";
    // Report-only (Physics, 2026-10-22): the band records the class's 8 to 14
    // as the relative error against 8; the test holds none.
    n.checks.push_back(ModelCheck{"FpvThrustToWeightIsReported", kFpvClassLow, "1",
                                  assumed("typical 5-inch racers sit around 8-14 (no single source)"),
                                  std::array<double, 2>{0.0, (kFpvClassHigh - kFpvClassLow) / kFpvClassLow}, true});
    n.assumptions = {wake_line(), std::string(kDragEstimated)};
    n.caveat = "the frame is assumed (220 mm motor to motor, true X) until one is cited, and its mass closes on the "
               "L4a header's 0.500 kg estimate";
    return n;
}

// -- Crazyflie-class (the draft's 3.4) -----------------------------------------

// The fit's targets: Bitcraze's 29 g and Luis & Le Ny's tensor, as design
// (x roll, y yaw, z pitch).
struct FitTargets {
    double mass = 0.0;
    glm::dvec3 inertia{0.0};
};

[[nodiscard]] inline DirectQuad crazyflie_layout() {
    DirectQuad q;
    const SourceRef arm = cite(Grade::measured, kLuisLeNy,
                               "Luis & Le Ny's 39.73 mm arm (Table 2.3.1; second-hand) along the diagonals; the X "
                               "layout is assumed (the firmware's default); Bitcraze's 92 mm span is a cross-check "
                               "only");
    q.half_x = 0.03973 * kSqrtHalf;
    q.half_x_ref = arm;
    q.half_z = 0.03973 * kSqrtHalf;
    q.half_z_ref = arm;
    q.hub_y = 0.012;
    q.hub_y_ref = assumed("the props on the 16 mm can, 9 mm above the board");
    q.body_size = glm::dvec3(0.033, 0.006, 0.033);
    q.body_size_ref = assumed("the board's outline, drawn");
    q.body_mass = 0.001;
    q.body_mass_ref = assumed("1 g drawn: the rest of the board's mass is in the remainder, so the composite can "
                              "meet Luis & Le Ny's tensor (Physics, 2026-10-22)");
    q.arm_radius = 0.0025;
    q.arm_ref = assumed("the board's arms and motor mounts, from the origin to each hub");
    q.arm_mass = 0.0002;
    q.arm_mass_ref = assumed("the board's arms and motor mounts");
    q.esc_ref = assumed("on the flight board: no record, its mass 0");
    q.battery_at = glm::dvec3(0.0, 0.0065, 0.0);
    q.battery_ref = assumed("on top of the board");
    q.proxy_ref = cite(Grade::derived, kLuisLeNy, kDirectProxy);
    return q;
}

[[nodiscard]] inline Result<DroneInputs> crazyflie_class_inputs(const DroneParts& parts) {
    DroneInputs in;
    add_direct_rows(in, crazyflie_layout(), parts);
    Result<AirframeSpec> base = direct_spec(parts, in, "crazyflie_class");
    if (!base) return std::unexpected(base.error());

    const double mass = 0.029;
    const glm::dvec3 inertia(1.395e-5, 2.173e-5, 1.436e-5);
    in.add("mass", mass,
           cite(Grade::published, kCfDatasheet, "fit_remainder()'s mass target: the Crazyflie 2.1's 29 g"));
    in.add("inertia", vec_value(inertia),
           cite(Grade::measured, kLuisLeNy,
                "fit_remainder()'s inertia target: Luis & Le Ny's (Ixx, Iyy, Izz) = (1.395, 1.436, 2.173)e-5 kg m^2 "
                "of a Crazyflie 2.0 (second-hand) as design (x roll, y yaw, z pitch), not scaled by 29/27 (Physics, "
                "2026-10-22)"));
    Result<PartInertia> rest = vehicles::fit_remainder(*base, mass, inertia);
    if (!rest) return std::unexpected(rest.error());
    const std::string fitted_note = std::string(kCfRemainder) + ": the targets less the parts, by fit_remainder()";
    in.add("parts[5].mass", rest->mass, cite(Grade::fitted, kCfDatasheet, fitted_note));
    in.add("parts[5].position", vec_value(rest->position), cite(Grade::fitted, kLuisLeNy, fitted_note));
    in.add("parts[5].tensor", mat_value(rest->tensor), cite(Grade::fitted, kLuisLeNy, fitted_note));
    return in;
}

[[nodiscard]] inline Result<FitTargets> crazyflie_class_targets(const DroneInputs& inputs) {
    InputReader r(inputs);
    FitTargets t;
    t.mass = r.number("mass");
    t.inertia = r.vec3("inertia");
    if (Result<void> st = r.problem(); !st) return std::unexpected(st.error());
    return t;
}

// The direct spec, and its remainder: the rows' record, which must be
// fit_remainder()'s of the targets (one rule, TD-9).
[[nodiscard]] inline Result<AirframeSpec> crazyflie_class(const DroneParts& parts, const DroneInputs& inputs) {
    InputReader r(inputs);
    AirframeSpec s = read_direct_quad(parts, r, "crazyflie_class");
    const shapes::PropMeshSpec props = read_prop_mesh(parts, r);
    const double mass = r.number("mass");
    const glm::dvec3 inertia = r.vec3("inertia");
    if (Result<void> st = r.problem(); !st) return std::unexpected(st.error());
    Result<PartInertia> rest = vehicles::fit_remainder(s, mass, inertia);
    if (!rest) return std::unexpected(rest.error());
    r.same("parts[5].mass", rest->mass);
    r.same("parts[5].position", vec_value(rest->position));
    r.same("parts[5].tensor", mat_value(rest->tensor));
    if (Result<void> st = r.status(); !st) return std::unexpected(st.error());
    s.parts.push_back(std::move(*rest));
    if (Result<void> w = wear_props(s, parts, props); !w) return std::unexpected(w.error());
    return s;
}

[[nodiscard]] inline DroneNotes crazyflie_class_notes() {
    DroneNotes n;
    n.built_by = "tests/library_drones.hpp: crazyflie_class(), its remainder by vehicles::fit_remainder()";
    n.checks.push_back(ModelCheck{
        "CrazyflieMatchesBitcrazesThrustTable", kCfRowFourMotorsGrams / 4.0, "g",
        cite(Grade::measured, kCfThrust,
             "one motor's k_T w^2 / g at 17,174 rpm against a quarter of the whole craft's 28.6 g, the row nearest "
             "hover (not the 9,374 rpm row). UIUC measured an isolated prop and Bitcraze weighed the whole craft, so "
             "the model reads about +8.8%; the band is not widened (Physics, 2026-10-22)"),
        std::array<double, 2>{-0.10, 0.10}, false});
    n.checks.push_back(ModelCheck{"CrazyflieMatchesBitcrazesThrustTable", kCfTopRowRpm, "rpm",
                                  cite(Grade::measured, kCfThrust,
                                       "duty 0.9375 at the state of charge whose loaded bus reads 3.30 V: the top "
                                       "row R was fitted to, so it closes the fit's loop, not an independent check"),
                                  std::nullopt, true});
    // The model runs this row about 2.4% fast and draws about 21.5% less bus
    // current. Both point to a load torque 25-30% below the real one. It's the
    // prop, not the motor (Physics, 2026-10-22): cfnq's C_Q is one static value,
    // and a 47 mm prop's profile drag rises as Re falls.
    n.checks.push_back(ModelCheck{"CrazyflieMatchesBitcrazesThrustTable", kCfRowRpm, "rpm",
                                  cite(Grade::measured, kCfThrust,
                                       "duty 0.5625 at the state of charge whose loaded bus reads 3.67 V: the "
                                       "17,174 rpm row; the static C_Q misses the low-Re rise; see Physics' "
                                       "Crazyflie J/Re item"),
                                  std::nullopt, true});
    n.checks.push_back(ModelCheck{"CrazyflieMatchesBitcrazesThrustTable", kCfTopRowRpm, "rpm",
                                  cite(Grade::measured, kCfThrust, "full duty at SoC 1: full pack"), std::nullopt,
                                  true});
    n.assumptions = {wake_line(), std::string(kDragEstimated)};
    n.caveat = "a Crazyflie 2.0's measured tensor at the 2.1's 29 g, not scaled by 29/27 (Physics, 2026-10-22); the "
               "prop is static only, its C_T held over J";
    return n;
}

// -- The basic quads (the draft's 3.5 and 3.6) ---------------------------------

// A default the drone uses unchanged, with basic_quad_assumptions()' note for
// `assumption` verbatim (check 8).
inline void add_default(DroneInputs& in, std::string path, InputValue value, std::string_view assumption) {
    in.add(std::move(path), std::move(value), assumed(default_note(assumption)));
}

// The layout, body and arm defaults every basic quad here keeps unchanged.
inline void add_frame_defaults(DroneInputs& in, const BasicQuadParams& d) {
    add_default(in, "hub_height", d.hub_height, "hub_height");
    add_default(in, "body_size", vec_value(d.body_size), "body_size");
    add_default(in, "body_mass", d.body_mass, "body_mass");
    add_default(in, "arm_radius", d.arm_radius, "arm_radius");
    add_default(in, "arm_mass_per_metre", d.arm_mass_per_metre, "arm_mass_per_metre");
    add_default(in, "esc.mount.position", vec_value(d.esc.mount.position), "esc.mount");
    add_default(in, "esc.mount.orientation", quat_value(d.esc.mount.orientation), "esc.mount");
}

// The params: the part files' blocks, the drone's mounts (a part file carries
// none) and the table's rows; with `fitted`, the fit targets and kappa.
[[nodiscard]] inline BasicQuadParams read_basic_params(const DroneParts& parts, InputReader& r, std::string name,
                                                       bool fitted) {
    BasicQuadParams p;
    p.name = std::move(name);
    p.half_span_x = r.number("half_span_x");
    p.half_span_z = r.number("half_span_z");
    p.hub_height = r.number("hub_height");
    p.body_size = r.vec3("body_size");
    p.body_mass = r.number("body_mass");
    p.arm_radius = r.number("arm_radius");
    p.arm_mass_per_metre = r.number("arm_mass_per_metre");
    p.motor = parts.motor;
    p.prop = parts.prop;
    p.esc = parts.esc;
    p.battery = parts.battery;
    p.esc.mount = read_pose(r, "esc.mount");
    p.battery.mount = read_pose(r, "battery.mount");
    if (fitted) {
        p.mass = r.number("mass");
        p.inertia = r.vec3("inertia");
        p.kappa = r.number("kappa");
    }
    return p;
}

[[nodiscard]] inline bool is_competition(std::string_view stem) { return stem == "basic_quad_competition"; }

[[nodiscard]] inline Result<BasicQuadParams> basic_quad_params(std::string_view stem, const DroneParts& parts,
                                                               const DroneInputs& inputs) {
    InputReader r(inputs);
    BasicQuadParams p = read_basic_params(parts, r, std::string(stem), is_competition(stem));
    if (Result<void> st = r.problem(); !st) return std::unexpected(st.error());
    return p;
}

inline constexpr std::string_view kBasicProxy =
    "basic_quad_airframe()'s rule: the arm's length plus the propeller's radius";

[[nodiscard]] inline Result<DroneInputs> basic_quad_6inch_inputs(const DroneParts& parts) {
    const BasicQuadParams d{};  // the defaults, never retyped (TD-9)
    DroneInputs in;
    add_default(in, "half_span_x", d.half_span_x, "half_span_x");
    add_default(in, "half_span_z", d.half_span_z, "half_span_z");
    add_frame_defaults(in, d);
    add_default(in, "battery.mount.position", vec_value(d.battery.mount.position), "battery.mount");
    add_default(in, "battery.mount.orientation", quat_value(d.battery.mount.orientation), "battery.mount");
    add_operating_point(in);
    add_prop_rows(in, parts);
    Result<BasicQuadParams> p = basic_quad_params("basic_quad_6inch", parts, in);
    if (!p) return std::unexpected(p.error());
    Result<AirframeSpec> spec = vehicles::basic_quad_airframe(*p);
    if (!spec) return std::unexpected(spec.error());
    in.add("proxy_radius", spec->proxy_radius, assumed(kBasicProxy));
    return in;
}

[[nodiscard]] inline Result<AirframeSpec> basic_quad_6inch(const DroneParts& parts, const DroneInputs& inputs) {
    InputReader r(inputs);
    const BasicQuadParams p = read_basic_params(parts, r, "basic_quad_6inch", false);
    if (Result<void> st = r.problem(); !st) return std::unexpected(st.error());
    Result<AirframeSpec> s = vehicles::basic_quad_airframe(p);
    if (!s) return std::unexpected(s.error());
    read_operating_point(*s, r);
    r.same("proxy_radius", s->proxy_radius);
    const shapes::PropMeshSpec props = read_prop_mesh(parts, r);
    if (Result<void> st = r.status(); !st) return std::unexpected(st.error());
    if (Result<void> w = wear_props(*s, parts, props); !w) return std::unexpected(w.error());
    return s;
}

// The L4a header's hover rpm for the 6-inch stand-in (tests/
// library_props.gen.hpp), 0 when the header has no such prop.
[[nodiscard]] inline double triblade_header_hover_rpm() {
    for (const library_props::Prop& p : library_props::kProps) {
        if (p.id == "triblade_6inch_scaled") return p.hover_rpm;
    }
    return 0.0;
}

[[nodiscard]] inline DroneNotes basic_quad_6inch_notes() {
    DroneNotes n;
    n.built_by = "vehicles::basic_quad_airframe()";
    n.checks.push_back(ModelCheck{
        "BasicQuadDefaultsHover", triblade_header_hover_rpm(), "rpm",
        cite(Grade::derived, kUiuc,
             "the L4a header's hover_rpm for triblade_6inch_scaled (tests/library_props.gen.hpp), the tool's at "
             "about 0.602 kg; a larger gap means the header's run choice must be re-checked"),
        std::array<double, 2>{-0.001, 0.001}, false});
    n.assumptions = {std::string(kKappaRule), triblade_line(), wake_line(), std::string(kDragEstimated)};
    n.caveat = "every number is a default basic_quad assumes: a generic 6-inch quad, its prop the tri-blade stand-in";
    return n;
}

// Kat's provenance (content/airframes/competition-700.airframe.json, read
// 2026-10-21), by the JSON's block: arm_x, arm_y and kappa are its `motor`.
[[nodiscard]] inline KatProvenance kat_motor_block() { return KatProvenance{"derived", "medium", "2026-07-29"}; }
[[nodiscard]] inline KatProvenance kat_mass_block() { return KatProvenance{"tuned", "medium", "2026-07-27"}; }
[[nodiscard]] inline KatProvenance kat_inertia_block() { return KatProvenance{"assumed", "low", ""}; }  // undated
[[nodiscard]] inline KatProvenance kat_drag_block() { return KatProvenance{"tuned", "high", "2026-07-29"}; }

// The documentary layout (the lead, 2026-10-22).
inline constexpr std::string_view kKatArms =
    "Kat calls arm_x and arm_y documentary in the CTBR path, so assumed by the kappa precedent (the lead, 2026-10-22)";

[[nodiscard]] inline Result<DroneInputs> basic_quad_competition_inputs(const DroneParts& parts) {
    const BasicQuadParams d{};
    DroneInputs in;
    in.add("half_span_x", 0.075,
           cite(Grade::assumed, kKat700,
                "Kat's arm_x, +-0.075 m; " + std::string(kKatArms) + "; the same-side discs overlap by 2.4 mm "
                "(advisory)"),
           kat_motor_block());
    in.add("half_span_z", 0.10, cite(Grade::assumed, kKat700, "Kat's arm_y, +-0.10 m; " + std::string(kKatArms)),
           kat_motor_block());
    add_frame_defaults(in, d);
    in.add("battery.mount.position", vec_value(glm::dvec3(0.0, 0.032, 0.0)),
           assumed("the 33 mm pack on top of the body, sunk 2 mm into it"));
    add_default(in, "battery.mount.orientation", quat_value(d.battery.mount.orientation), "battery.mount");
    in.add("mass", 0.7,
           cite(Grade::fitted, kKat700, "Kat's mass, fitted jointly with the drag to the VQ1 simulator; met by the "
                                        "remainder"),
           kat_mass_block());
    in.add("inertia", vec_value(glm::dvec3(0.0025, 0.0043, 0.0021)),
           cite(Grade::assumed, kKat700,
                "Kat's (Ixx, Iyy, Izz) = (0.0025, 0.0021, 0.0043) in FLU, as design (x roll, y yaw, z pitch): "
                "NeuroBEM's published tensor, inert in CTBR; met by the remainder"),
           kat_inertia_block());
    in.add("kappa", 0.022,
           cite(Grade::assumed, kKat700,
                std::string(kKappaRule) + " Here it sets prop.cq_static = kappa ct_static / D and prop.cq_table = "
                "kappa C_T(J) / D at the ct_table's J points (basic_quad_airframe()), replacing the part file's "
                "torque."),
           kat_motor_block());
    add_operating_point(in);
    add_prop_rows(in, parts);

    // What basic_quad_airframe() makes of them: the remainder, at the centre of
    // mass, which also places the drag.
    Result<BasicQuadParams> p = basic_quad_params("basic_quad_competition", parts, in);
    if (!p) return std::unexpected(p.error());
    Result<AirframeSpec> spec = vehicles::basic_quad_airframe(*p);
    if (!spec) return std::unexpected(spec.error());
    if (spec->parts.size() != 6) return std::unexpected(invalid("basic_quad_competition: no remainder (parts[5])"));
    const PartInertia& rest = spec->parts[5];

    in.add("drag[0].coeffs", vec_value(glm::dvec3(0.028, 0.028, 0.028)),
           cite(Grade::fitted, kKat700,
                "Kat's F_i = -0.028 |v_i| v_i on the body axes is the engine's componentwise law (physics/"
                "forces.cpp), in kg/m with no rho conversion (Physics, 2026-10-22)"),
           kat_drag_block());
    in.add("drag[0].mount.position", vec_value(rest.position),
           assumed("at the centre of mass, the remainder's place, so the drag adds no torque, as in Kat's law"));
    in.add("drag[0].mount.orientation", quat_value(identity_turn()), assumed("the body axes"));
    in.add("proxy_radius", spec->proxy_radius,
           assumed(std::string(kBasicProxy) + ", the geometric reach (Physics, 2026-10-22); Kat's 0.3 m collision "
                                              "radius is a planner's margin (not_carried)"));
    const std::string rest_note =
        "Kat's 0.700 kg and tensor less the assumed parts, at their centre of mass (fit_remainder(), inside "
        "basic_quad_airframe()); assumed, since the parts are (Physics, 2026-10-22); with the body and arms it is "
        "Physics' 0.336 kg";
    in.add("parts[5].mass", rest.mass, assumed(rest_note));
    in.add("parts[5].position", vec_value(rest.position), assumed(rest_note));
    in.add("parts[5].tensor", mat_value(rest.tensor), assumed(rest_note));
    return in;
}

[[nodiscard]] inline Result<AirframeSpec> basic_quad_competition(const DroneParts& parts, const DroneInputs& inputs) {
    InputReader r(inputs);
    const BasicQuadParams p = read_basic_params(parts, r, "basic_quad_competition", true);
    if (Result<void> st = r.problem(); !st) return std::unexpected(st.error());
    Result<AirframeSpec> s = vehicles::basic_quad_airframe(p);
    if (!s) return std::unexpected(s.error());
    read_operating_point(*s, r);
    vehicles::DragSpec drag;
    drag.mode = physics::drag_mode::componentwise;
    drag.coeffs = r.vec3("drag[0].coeffs");
    drag.mount = read_pose(r, "drag[0].mount");
    s->drag.push_back(drag);
    r.same("proxy_radius", s->proxy_radius);
    if (s->parts.size() != 6) return std::unexpected(invalid("basic_quad_competition: no remainder (parts[5])"));
    r.same("parts[5].mass", s->parts[5].mass);
    r.same("parts[5].position", vec_value(s->parts[5].position));
    r.same("parts[5].tensor", mat_value(s->parts[5].tensor));
    const shapes::PropMeshSpec props = read_prop_mesh(parts, r);
    if (Result<void> st = r.status(); !st) return std::unexpected(st.error());
    if (Result<void> w = wear_props(*s, parts, props); !w) return std::unexpected(w.error());
    return s;
}

[[nodiscard]] inline DroneNotes basic_quad_competition_notes() {
    DroneNotes n;
    n.built_by = "vehicles::basic_quad_airframe(), then Kat's drag as drag[0]";
    // A guard (Physics, 2026-10-22), not a check of the parts: the curve was
    // never withdrawn, but the parts meant to meet it came from the withdrawn
    // datapack 7.2, so the full-duty figure is report-only.
    n.checks.push_back(ModelCheck{
        "BasicQuadCompetitionHoversAtTheCurvesHoverPoint", kKatHoverAccel, "m/s^2",
        cite(Grade::fitted, kKat700,
             "a guard that the mass input is consistent with the curve: the compiled hover thrust per rotor against "
             "0.700 kg at the curve's hover point, norm 0.264 at 9.712 m/s^2 (Kat: measured, high, 2026-07-29); the "
             "+0.97% is the curve's own implied g (Physics, 2026-10-22)"),
        std::array<double, 2>{-0.02, 0.02}, false});
    n.checks.push_back(ModelCheck{
        "BasicQuadCompetitionHoversAtTheCurvesHoverPoint", kKatThrustPerMotor, "N",
        cite(Grade::fitted, kKat700,
             "full duty per motor against Kat's thrust_per_motor (derived from its curve): " +
                 std::string(kCompKappaCost) + "; reported, not banded (Physics, 2026-10-22)"),
        std::nullopt, true});
    n.not_carried = {
        "thrust_curve: Kat's measured throttle-to-thrust curve (ruling 6: a later plan)",
        "airspeed_thrust: the curve's airspeed loss (ruling 6)",
        "motor.tau: 0.033 s, a controller value; the compile fits its own",
        "envelope: a controller value; the airframe has no field for it",
        "rate_loop: a controller value; the airframe has no field for it",
        "collision_radius: 0.3 m, a planner's margin; proxy_radius is the geometric reach (Physics, 2026-10-22)",
    };
    n.assumptions = {std::string(kKappaRule), triblade_line(), wake_line()};
    n.caveat = std::string(kCompSimulator) + "; " + std::string(kCompKappaCost);
    return n;
}

// -- A drone, built --------------------------------------------------------------

struct BuiltDrone {
    DroneDef def{};
    DroneParts parts{};
    DroneInputs inputs{};
    std::optional<BasicQuadParams> params{};  // a basic quad's
    std::optional<FitTargets> targets{};      // the Crazyflie's
    shapes::PropMeshSpec prop_mesh{};         // what every drone's props are drawn from (L7c1)
    AirframeSpec spec{};
    DroneNotes notes{};
};

[[nodiscard]] inline Result<DroneInputs> drone_inputs(std::string_view stem, const DroneParts& parts) {
    if (stem == "phantom3_class") return phantom3_class_inputs(parts);
    if (stem == "iris_plus_class") return iris_plus_class_inputs(parts);
    if (stem == "fpv_5inch") return fpv_5inch_inputs(parts);
    if (stem == "crazyflie_class") return crazyflie_class_inputs(parts);
    if (stem == "basic_quad_6inch") return basic_quad_6inch_inputs(parts);
    if (stem == "basic_quad_competition") return basic_quad_competition_inputs(parts);
    return std::unexpected(invalid("no L5 drone is named '" + std::string(stem) + "'"));
}

[[nodiscard]] inline Result<AirframeSpec> drone_spec(std::string_view stem, const DroneParts& parts,
                                                     const DroneInputs& inputs) {
    if (stem == "phantom3_class") return phantom3_class(parts, inputs);
    if (stem == "iris_plus_class") return iris_plus_class(parts, inputs);
    if (stem == "fpv_5inch") return fpv_5inch(parts, inputs);
    if (stem == "crazyflie_class") return crazyflie_class(parts, inputs);
    if (stem == "basic_quad_6inch") return basic_quad_6inch(parts, inputs);
    if (stem == "basic_quad_competition") return basic_quad_competition(parts, inputs);
    return std::unexpected(invalid("no L5 drone is named '" + std::string(stem) + "'"));
}

[[nodiscard]] inline DroneNotes drone_notes(std::string_view stem) {
    if (stem == "phantom3_class") return phantom3_class_notes();
    if (stem == "iris_plus_class") return iris_plus_class_notes();
    if (stem == "fpv_5inch") return fpv_5inch_notes();
    if (stem == "crazyflie_class") return crazyflie_class_notes();
    if (stem == "basic_quad_6inch") return basic_quad_6inch_notes();
    return basic_quad_competition_notes();
}

// The drone named `stem`, from the committed library's part files: its table,
// its spec (the table's alone), its props' mesh, its params or fit targets,
// and its notes.
[[nodiscard]] inline Result<BuiltDrone> build_drone(const scene::Library& library,
                                                    const scene::SourceRegistry& registry, std::string_view stem) {
    const DroneDef* def = find_drone(stem);
    if (def == nullptr) return std::unexpected(invalid("no L5 drone is named '" + std::string(stem) + "'"));
    Result<DroneParts> parts = load_drone_parts(library, registry, def->parts);
    if (!parts) return std::unexpected(parts.error());
    BuiltDrone out;
    out.def = *def;
    out.parts = std::move(*parts);
    Result<DroneInputs> inputs = drone_inputs(stem, out.parts);
    if (!inputs) return std::unexpected(inputs.error());
    out.inputs = std::move(*inputs);
    Result<AirframeSpec> spec = drone_spec(stem, out.parts, out.inputs);
    if (!spec) return std::unexpected(spec.error());
    out.spec = std::move(*spec);
    Result<shapes::PropMeshSpec> mesh = prop_mesh_params(out.parts, out.inputs);
    if (!mesh) return std::unexpected(mesh.error());
    out.prop_mesh = std::move(*mesh);
    if (stem == "basic_quad_6inch" || is_competition(stem)) {
        Result<BasicQuadParams> p = basic_quad_params(stem, out.parts, out.inputs);
        if (!p) return std::unexpected(p.error());
        out.params = std::move(*p);
    }
    if (stem == "crazyflie_class") {
        Result<FitTargets> t = crazyflie_class_targets(out.inputs);
        if (!t) return std::unexpected(t.error());
        out.targets = *t;
    }
    out.notes = drone_notes(stem);
    return out;
}

}  // namespace spade::test::library_drones
