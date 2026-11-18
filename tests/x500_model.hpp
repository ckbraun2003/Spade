#pragma once

// ---------------------------------------------------------------------------
// x500_model.hpp -- the website clips' flow model of the PX4 X500 (L86k, the lead, 2026-11-17): built from KAT's
// content/airframes/x500.airframe.json (at KAT's commit c3350dcd2790bbdccdb9b4c3768143c14a1ecf3b), not the default library's. The
// generator writes tools/flow_movie/data/x500/ (x500.model.yaml, parts/x500_10in_apc10x5.prop.yaml and x500.notes.yaml's text);
// tests/test_x500_case.cpp checks the files against this builder and, with SPADE_X500_WRITE set, writes them.
//
// WHAT IS KAT'S (x500.airframe.json, read 2026-11-17): mass 2.0643 kg and inertia (0.02167, 0.02167, 0.04) kg m^2 (x roll, y pitch, z yaw
// in KAT's body frame, thrust along +z); rotors at (+-0.174, +-0.174) m, the hubs 0.06 m above the centre of mass (rotor_visuals);
// PX4's X layout (front-right and rear-left CCW, front-left and rear-right CW); the arms 16 mm carbon tubes and the body plates as
// x500-body.gltf's extents say. Its own provenance grades the mass ASSUMED (the PX4-gazebo-models sum, base 2.0 kg plus four 0.016077 kg
// rotor links; no take-off weight is published for the Holybro X500 V2), confidence low.
//
// THE PROPELLER (L86p, the lead, 2026-11-18, correcting L86k): the real X500 V2 kit ships 1045 propellers (10 x 4.5 in; Holybro's kit page,
// docs.holybro.com/drone-development-kit/px4-development-kit-x500v2, and KAT's own x500.airframe.json evidence "500 mm wheelbase, 2216 KV920,
// 1045 propellers"); the 13-inch 1345 STL is the Gazebo sim's prop, whose thrust constants (k_T 1.23e-5 N s^2/rad^2, 642 rad/s to hover) are
// the sim's, not the kit's. So R is 0.127 m and the blade is the library's UIUC APC 10x5 (measured geometry, C_T and C_Q tables) at UIUC's own
// 10 in; the library's part applies it at 9.4 in (NASA's rotor), so only the diameter (and the mass-derived inertia) differ. Graded derived: the
// 5 in pitch against the kit's 4.5 in is said in the notes. Its hover speed is the speed the grid-free blade-element-momentum solve gives for a
// quarter of the weight (test_x500_case.cpp), ~6100 rpm; the line's thrust at the chain's own speed is checked against the same quarter.
// ---------------------------------------------------------------------------

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "bemt_reference.hpp"
#include "core/error.hpp"
#include "physics/airflow/blade.hpp"
#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "scene/part_file.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/basic_quad.hpp"

namespace spade::testing::x500 {

inline constexpr const char* kKatCommit = "c3350dcd2790bbdccdb9b4c3768143c14a1ecf3b";  // KAT: the last change to x500.airframe.json
inline constexpr double kMass = 2.0643;                                                // kg, KAT's (the Gazebo model's sum)
inline constexpr double kArm = 0.174;                                                  // m, KAT's motor.arm_x and arm_y
inline constexpr double kDiameter = 0.254;                                             // m, the kit's 1045: 10 in tip to tip
inline constexpr double kHubAboveCentre = 0.06;                                        // m, KAT's rotor_visuals hub_m y
inline constexpr double kPropMass = 0.013;                                             // kg, the library's APC 10x5 (assumed, a typical 9-10 in moulded prop)

[[nodiscard]] inline std::filesystem::path data_dir() {
    return std::filesystem::path(SPADE_ASSETS_DIR).parent_path() / "tools" / "flow_movie" / "data" / "x500";  // assets/ is in the repo root
}
inline constexpr const char* kModelFile = "x500.model.yaml";
inline constexpr const char* kPropFile = "parts/x500_10in_apc10x5.prop.yaml";
inline constexpr const char* kNotesFile = "x500.notes.yaml";

// Where each number of x500.model.yaml came from (the lead, 2026-11-17: name the PX4 and KAT sources in the model's sources). Plain text
// the generator writes beside the model; tests/test_x500_case.cpp compares it. The grades are the library's words.
[[nodiscard]] inline std::string notes_text() {
    return std::string(R"NOTES(# The website clips' flow model of an open-source quadcopter (the PX4 X500 kit): where each number came from.
# Written by tests/x500_model.hpp notes_text() and compared by tests/test_x500_case.cpp (SPADE_X500_WRITE=1 regenerates it). Grades are
# the library's: measured, published, retailer, derived, fitted, assumed. Anything not carried is listed at the end.
model: "x500.model.yaml"
prop: "parts/x500_10in_apc10x5.prop.yaml"
built_by: "tests/x500_model.hpp airframe(): vehicles::basic_quad_airframe() at KAT's layout, mass and inertia"
kat:
  file: "content/airframes/x500.airframe.json"
  commit: ")NOTES") + kKatCommit + R"NOTES("
  note: "KAT's own provenance for it: PX4-gazebo-models commit e5997f455e33ec071c7c677fc59a65781de496a4 (x500_base/model.sdf), the Holybro X500 V2 pages (opened 2026-11-10); confidence low"
licence: "the PX4 x500 model is BSD-3-Clause; KAT's records hold the text"
inputs:
  mass: {value: 2.0643, unit: kg, grade: assumed, note: "KAT's mass: the Gazebo model's own sum, a 2.0 kg base plus four 0.016077 kg rotors; no take-off weight is published for the Holybro X500 V2"}
  inertia: {value: [0.0216667, 0.0216667, 0.04], unit: kg m^2, grade: assumed, note: "KAT's inertia (x roll, y pitch, z yaw about the centre of mass, thrust along +z), met by basic_quad's remainder part"}
  rotors: {value: "(+-0.174, +-0.174) m", grade: assumed, note: "KAT's motor.arm_x and arm_y; PX4's X layout: front-right and rear-left counter-clockwise, front-left and rear-right clockwise"}
  hub_height: {value: "0.06 m above the centre of mass", grade: assumed, note: "KAT's rotor_visuals hub_m; the model's hubs come out 0.0614 m above it (the fit's centre of mass)"}
  rotor_radius: {value: 0.127, unit: m, grade: derived, note: "the kit's 1045 propeller (10 x 4.5 in; Holybro's X500 V2 kit page and KAT's x500.airframe.json evidence, '1045 propellers'): 10 in = 0.254 m, halved; it is the propeller part's diameter / 2. The 13-inch 1345 STL in PX4's Gazebo model is the sim's prop, not the kit's"}
  body: {value: "box 0.15 x 0.065 x 0.15 m, 0.60 kg", grade: assumed, note: "the electronics stack of x500-body.gltf's extents; a proxy solid for the flow"}
  arms: {value: "tubes of 8 mm radius, 0.10 kg/m", grade: assumed, note: "Holybro's 16 mm carbon arm tubes; proxy solids"}
  motor: {value: "the library's iris_920kv", grade: assumed, note: "the X500's 2216 KV920, a stand-in of the same class"}
  esc_battery: {value: "the library's iris_4x20a and iris_3s_5100", grade: assumed, note: "stand-ins; the chain only fixes the hover throttle, and the rotors turn at the speed the propeller part gives for a quarter of the weight"}
  propeller: {value: "parts/x500_10in_apc10x5.prop.yaml", grade: derived, note: "the library's UIUC APC 10x5 blade (measured geometry, C_T and C_Q tables) used at UIUC's own 10 in, the kit's 1045 size; the pitch is UIUC's 5 in against the kit's 4.5 in; the library's part applies the same tables at 9.4 in. PX4's Gazebo thrust constants (642 rad/s to hover) are the sim's"}
checks:
  balance: "the grid-free blade-element-momentum solve (tests/bemt_reference.hpp) for a quarter of the weight, 5.0610 N: see X500Balance's printout"
  box: "266 x 237 x 266 at 2.5 mm, the drone lowered 18 rows (tests/d1_case.hpp d1_x500_hold()): the rotors' centres 0.1913 m (1.5 R) under the top face, 0.401 m of wake below, 0.0165 m past the kernel's reach at the sides"
not_carried: "landing legs, the camera and GPS mounts, the wiring, the propellers' hubs and the motors' bells as solids; the mass is the Gazebo model's, to be replaced by the parts' datasheet masses (L88l)"
)NOTES";
}

[[nodiscard]] inline scene::SourceRef estimated(std::string note) {
    return scene::SourceRef{{}, scene::Grade::assumed, "ESTIMATED: " + std::move(note)};
}

// The propeller: the library's UIUC APC 10x5 at the kit's 10 in (R 0.127 m). Its blade and tables are the library's, as they are; the rows whose value
// changed (diameter, inertia) say so, and the pitch row says the 5 in against the kit's 4.5 in. The rows the library's part grades from UIUC at
// "9.4 in" keep their grade and source, with the X500's use said after them.
[[nodiscard]] inline Result<scene::PropPart> prop_part() {
    const std::filesystem::path file = std::filesystem::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_apce_10x5.prop.yaml";
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::unexpected(Error{Code::io_error, "cannot read " + file.string()});
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    auto base = scene::prop_from_yaml(text);
    if (!base) return std::unexpected(base.error());
    scene::PropPart p = *base;
    p.block.diameter = kDiameter;
    p.block.mass = kPropMass;
    p.block.inertia = kPropMass * kDiameter * kDiameter / 12.0;
    p.block.shape.reset();
    const std::string used =
        " [the X500: used at UIUC's own 10 in, the kit's 1045 size; the library's part applies it at 9.4 in (NASA's rotor)]";
    const auto derived = [](std::string note) { return scene::SourceRef{"uiuc_propdb", scene::Grade::derived, std::move(note)}; };
    for (scene::FieldSource& s : p.sources) {
        if (s.field == "diameter") {
            s.ref = derived("derived: UIUC's APC Thin Electric 10x5 is a 10 in propeller; the X500 V2 kit's propeller is the 1045, 10 in (Holybro's kit page; "
                            "KAT's x500.airframe.json evidence), so R is 0.127 m");
        } else if (s.field == "pitch") {
            s.ref = derived("derived: 5.0 in as UIUC's 10x5; the kit's 1045 is 4.5 in (Holybro's kit page), so the blade's pitch is 11% above the kit's");
        } else if (s.field == "inertia") {
            s.ref = derived("derived: m D^2 / 12 from this file's mass and the 10 in diameter");
        } else if (s.field == "blades" || s.field == "mass" || s.field == "blade.airfoil") {
            // as the library grades them
        } else {
            s.ref.note += used;
        }
    }
    return p;
}

// The model: a basic quad at KAT's layout, mass and inertia, with the 10-inch propeller and the library's 920 KV motor class (the
// X500's 2216 KV920), the Iris class's ESC and 3S pack.
[[nodiscard]] inline Result<vehicles::AirframeSpec> airframe(const scene::PropPart& prop) {
    namespace sc = scene;
    const std::filesystem::path parts = std::filesystem::path(SPADE_ASSETS_DIR) / "library/parts";
    const auto read = [&](const char* name) -> Result<std::string> {
        std::ifstream in(parts / name, std::ios::binary);
        if (!in) return std::unexpected(Error{Code::io_error, std::string("cannot read ") + name});
        return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    };
    const auto motor_text = read("iris_920kv.motor.yaml");
    const auto esc_text = read("iris_4x20a.esc.yaml");
    const auto battery_text = read("iris_3s_5100.battery.yaml");
    if (!motor_text) return std::unexpected(motor_text.error());
    if (!esc_text) return std::unexpected(esc_text.error());
    if (!battery_text) return std::unexpected(battery_text.error());
    const auto motor = sc::motor_from_yaml(*motor_text);
    const auto esc = sc::esc_from_yaml(*esc_text);
    const auto battery = sc::battery_from_yaml(*battery_text);
    if (!motor) return std::unexpected(motor.error());
    if (!esc) return std::unexpected(esc.error());
    if (!battery) return std::unexpected(battery.error());

    vehicles::BasicQuadParams p;
    p.name = "x500";
    p.half_span_x = kArm;
    p.half_span_z = kArm;
    p.hub_height = kHubAboveCentre + 0.015;  // KAT's hubs 0.06 m above the centre; its arm tubes' axes 0.015 m below it
    p.body_size = glm::dvec3(0.15, 0.065, 0.15);
    p.body_mass = 0.60;
    p.arm_radius = 0.008;  // the X500's 16 mm carbon tubes
    p.arm_mass_per_metre = 0.10;
    p.motor = motor->block;
    p.prop = prop.block;
    p.esc = esc->block;
    p.battery = battery->block;
    p.mass = kMass;
    p.inertia = glm::dvec3(0.02166666666666667, 0.04, 0.02166666666666667);  // KAT's (x roll, z yaw, y pitch) as design (x, y, z)
    return vehicles::basic_quad_airframe(p);
}

// The rotor speed (rad/s) at which the grid-free blade-element-momentum solve (tests/bemt_reference.hpp, Prandtl's tip loss on) gives
// `thrust` newtons: a fixed 60 bisections on [10, 2000] rad/s, so the same bits every run. invalid_argument when the bracket does not hold
// it (a thrust above the blade's at 2000 rad/s, or below its at 10).
[[nodiscard]] inline Result<float> balance_omega(const physics::airflow::BladeShape& blade, const physics::airflow::PolarTable& polar,
                                                 float thrust) {
    const auto at = [&](float omega) -> Result<float> {
        BemtInput in;
        in.blade = &blade;
        in.polar = &polar;
        in.omega = omega;
        in.tip_loss = true;
        const auto loads = bemt(in);
        if (!loads) return std::unexpected(loads.error());
        return loads->thrust;
    };
    float lo = 10.0f;
    float hi = 2000.0f;
    const auto t_lo = at(lo);
    const auto t_hi = at(hi);
    if (!t_lo) return std::unexpected(t_lo.error());
    if (!t_hi) return std::unexpected(t_hi.error());
    if (!(*t_lo <= thrust && thrust <= *t_hi)) {
        return std::unexpected(Error{Code::invalid_argument, "balance_omega: " + std::to_string(thrust) + " N is outside the blade's [" +
                                                                 std::to_string(*t_lo) + ", " + std::to_string(*t_hi) + "] N over 10 to 2000 rad/s"});
    }
    for (int i = 0; i < 60; ++i) {
        const float mid = 0.5f * (lo + hi);
        const auto t = at(mid);
        if (!t) return std::unexpected(t.error());
        (*t < thrust ? lo : hi) = mid;
    }
    return 0.5f * (lo + hi);
}

}  // namespace spade::testing::x500
