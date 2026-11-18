#pragma once

// ---------------------------------------------------------------------------
// x500_model.hpp -- the website clips' flow model of the PX4 X500 (L86k, the lead, 2026-11-17): built from KAT's
// content/airframes/x500.airframe.json (at KAT's commit c3350dcd2790bbdccdb9b4c3768143c14a1ecf3b), not the default library's. The
// generator writes tools/flow_movie/data/x500/ (x500.model.yaml, parts/x500_10in_apc10x5.prop.yaml and x500.notes.yaml's text);
// tests/test_x500_case.cpp checks the files against this builder and, with SPADE_X500_WRITE set, writes them.
//
// WHAT IS KAT'S (x500.airframe.json, read 2026-11-17): the layout, rotors at (+-0.174, +-0.174) m (motor.arm_x and arm_y); PX4's X layout
// (front-right and rear-left CCW, front-left and rear-right CW, the engine's spin_dir +1, -1, +1, -1); the arms 16 mm carbon tubes. NOT KAT'S
// any more: its mass 2.0643 kg and inertia were the PX4-gazebo-models sum (a 2.0 kg base plus four rotor links, graded assumed, confidence low).
//
// MASS, CENTRE OF MASS AND INERTIA (L88l, the lead, 2026-11-18) are Core's build table's: tools/flow_movie/data/x500_build/parts.yaml on
// core/x500-pose-table, commit 1031a020d44c8412d19ef1d29321b5641598e87f: 13 parts, 1.6389 kg, each with its own mass grade there (the frame
// 0.610 kg published, the pack 0.4365 kg retailer, the misc carrier 0.175 kg assumed, ...). Here each part is a record at its table pose
// (the table's frame is Y up, +X forward, +Z right, origin its centre of mass: the engine's design frame, shifted kDesignAboveTable in y so the
// arms keep basic_quad's y = 0 plane): the frame is a box (the plate stack from the tray's underside to the damper deck's top) and four arm
// tubes, the motors and propellers are the blocks, the pack a box under the stack (a solid in the flow), the ESC block a box in it, and the
// controller, Pi, camera, radio, GPS, mast and the misc carrier ONE tensor record (no solid) at their common centre of mass. Every solid and the
// hubs sit at the table's poses, exactly. The compile's composite gives the centre of mass and the inertia, which are the table's parts' own:
// the centre of mass comes out a few millimetres from the table's origin (the table says the origin IS the centre of mass; its parts' sum says
// otherwise), which a drawing of the table must allow for (notes: centre_of_mass).
//
// PRODUCTS OF INERTIA. The engine's model is principal-axes only: the compile turns the design frame to the composite's principal axes, and
// this tensor is nearly isotropic in x and z (I_x 0.0145, I_z 0.0156), so the parts' own products (xy 3.8e-4, xz 1.6e-4, yz 2.3e-4 kg m^2,
// 1 to 3% of the diagonal) turned the body frame by degrees and every rotor with it. The avionics record carries the cancelling products, so the
// model's inertia is the DIAGONAL of the table's tensor about its centre of mass, and design_to_principal is the identity.
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
inline constexpr double kMass = 1.6389;                                                // kg, the build table's 13 parts (Core, 1031a020)
inline constexpr const char* kTableCommit = "1031a020d44c8412d19ef1d29321b5641598e87f";  // core/x500-pose-table: x500_build/parts.yaml
inline constexpr double kDesignAboveTable = 0.0136;                                    // m, design y = table y + this (the arms' plane is design y = 0)
inline constexpr double kArm = 0.174;                                                  // m, KAT's motor.arm_x and arm_y
inline constexpr double kDiameter = 0.254;                                             // m, the kit's 1045: 10 in tip to tip
inline constexpr double kHubAboveCentre = 0.0614;                                      // m, the table's hubs above its origin (its centre of mass)
inline constexpr double kPackMass = 0.4365;                                            // kg, Tattu G-Tech 5200 mAh 4S (Grepow listing, retailer)
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
built_by: "tests/x500_model.hpp airframe(): vehicles::basic_quad_airframe() at the kit's layout, with the parts of Core's build table as records"
table:
  file: "tools/flow_movie/data/x500_build/parts.yaml"
  branch: "core/x500-pose-table"
  commit: ")NOTES") + kTableCommit + R"NOTES("
  note: "13 parts at measured or assumed poses on the frame mesh, each mass graded there: frame 0.610 kg published (Holybro kit page), Pixhawk 6C 34.6 g, GPS 32 g and SiK radio 23.5 g published (Holybro docs), Raspberry Pi 5 45 g and the 4S pack 436.5 g retailer, camera 4 g, GPS mast 6.3 g, misc ESCs/wiring/fasteners 175 g and the four motors 55 g with their propellers assumed; the sum is nobody's take-off weight"
kat:
  file: "content/airframes/x500.airframe.json"
  commit: ")NOTES" + kKatCommit + R"NOTES("
  note: "the layout and PX4's spins only (KAT's own provenance: PX4-gazebo-models commit e5997f455e33ec071c7c677fc59a65781de496a4, x500_base/model.sdf, and the Holybro X500 V2 pages, opened 2026-11-10); its mass 2.0643 kg and inertia (the Gazebo model's sum) are replaced by the build table's"
licence: "the PX4 x500 model is BSD-3-Clause; KAT's records hold the text"
inputs:
  mass: {value: 1.6389, unit: kg, grade: derived, note: "the sum of the build table's 13 parts' masses, each graded there (frame published, pack retailer, misc and rotors assumed); it replaces the Gazebo model's 2.0643 kg"}
  centre_of_mass: {value: "see the X500Model printout (table frame, m)", grade: derived, note: "the compile's sum over the parts at the table's poses, not tuned and not the table's origin: the table calls its origin the centre of mass, its parts' sum puts it a few millimetres off (forward, low, to port), where the model puts the frame stack's mass at the plates' middle (6 mm under the origin) and the arm tubes 13.6 mm under it. Every solid and the hubs sit at the table's poses exactly, so a drawing of the table on this model's body origin is shifted by that offset"}
  inertia: {value: "see x500.model.yaml's body.inertia", unit: kg m^2, grade: derived, note: "the diagonal of the airframe compile's composite of the same parts, about the centre of mass: point masses for the controller, Pi, camera, radio, GPS and mast and a box for the misc carrier (0.07 x 0.016 x 0.07 m), lumped in one tensor record; boxes for the frame stack (0.15 x 0.0809 x 0.15 m), the pack and the ESC block; thin tubes for the four arms (8 mm radius, 0.10 kg/m, 0.0984 kg in all, the rest of the frame's 0.610 kg in the stack); the motors as cylinders. The products of inertia (xy 3.8e-4, xz 1.6e-4, yz 2.3e-4 kg m^2) are dropped: the engine's model is principal-axes only, and with I_x and I_z this close they would turn the body frame by degrees. An independent hand sum agrees within 3%"}
  rotors: {value: "(+-0.174, +-0.174) m", grade: assumed, note: "KAT's motor.arm_x and arm_y, and the build table's hubs; PX4's X layout: front-right and rear-left counter-clockwise (spin_dir +1), front-left and rear-right clockwise (-1)"}
  hub_height: {value: "0.0614 m above the table's origin", grade: assumed, note: "the build table's hub pose, exactly; the compiled centre of mass is a few millimetres from that origin, so the hubs are that much further from it"}
  rotor_radius: {value: 0.127, unit: m, grade: derived, note: "the kit's 1045 propeller (10 x 4.5 in; Holybro's X500 V2 kit page and KAT's x500.airframe.json evidence, '1045 propellers'): 10 in = 0.254 m, halved; it is the propeller part's diameter / 2. The 13-inch 1345 STL in PX4's Gazebo model is the sim's prop, not the kit's"}
  body: {value: "box 0.15 x 0.0809 x 0.15 m, 0.5116 kg", grade: derived, note: "the plate stack from the tray's underside (y -0.0466) to the damper deck's top (0.0343) of the build table's frame, a proxy solid for the flow; its mass is the frame's 0.610 kg less the four arm tubes' 0.0984 kg"}
  arms: {value: "tubes of 8 mm radius, 0.10 kg/m", grade: assumed, note: "Holybro's 16 mm carbon arm tubes; proxy solids"}
  battery: {value: "box 0.133 x 0.0335 x 0.045 m, 0.4365 kg, under the tray", grade: retailer, note: "the Tattu G-Tech 5200 mAh 4S pack (Grepow listing), its top face on the tray's underside (y -0.0466), long axis along +X; a solid in the flow, in the propellers' downwash; the pose is assumed (a layout choice the frame leaves open)"}
  motor: {value: "the library's iris_920kv, 0.055 kg", grade: assumed, note: "the X500's 2216 KV920, a stand-in of the same class"}
  esc_battery: {value: "the library's iris_4x20a (0.028 kg) and phantom3_4s_4480 cells", grade: assumed, note: "stand-ins; the pack's mass and shape are the table's. The chain only fixes the hover throttle; the rotors turn at the speed the propeller part gives for a quarter of the weight. The table's 0.175 kg misc carrier covers the ESCs, so the carrier here is 0.147 kg beside the ESC block's 0.028 kg"}
  propeller: {value: "parts/x500_10in_apc10x5.prop.yaml", grade: derived, note: "the library's UIUC APC 10x5 blade (measured geometry, C_T and C_Q tables) used at UIUC's own 10 in, the kit's 1045 size; the pitch is UIUC's 5 in against the kit's 4.5 in; the library's part applies the same tables at 9.4 in. PX4's Gazebo thrust constants (642 rad/s to hover) are the sim's"}
checks:
  balance: "the grid-free blade-element-momentum solve (tests/bemt_reference.hpp) for a quarter of the weight, 4.0181 N: see X500Balance's printout"
  box: "266 x 237 x 266 at 2.5 mm, the drone lowered 18 rows (tests/d1_case.hpp d1_x500_hold()): the rotors' centres 0.1913 m (1.5 R) under the top face, 0.401 m of wake below, 0.0165 m past the kernel's reach at the sides"
not_carried: "landing legs, the camera and GPS mounts, the wiring, the propellers' hubs and the motors' bells as solids; the controller, Pi, camera, radio, GPS and mast are one point-mass tensor record, not solids (they sit inside the stack's top or above the hub plane, and the GPS mast does not matter to the downwash); the frame's plates are one box, with no gaps between them"
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

// The avionics as ONE tensor record (see PRODUCTS OF INERTIA): the table's 6C, GPS, Pi, camera, radio, mast and the misc carrier (0.147 kg beside the
// ESC block's 0.028), at their common centre of mass, with their own tensor about it less `cancel` (a symmetric matrix; zero for the true one).
// Table frame in, design frame out (y + kDesignAboveTable).
[[nodiscard]] inline vehicles::PartInertia avionics_record(const glm::dmat3& cancel, double esc_mass) {
    struct Item {
        double mass;
        glm::dvec3 at;     // the table's pose
        glm::dvec3 dims;   // a box, or 0 for a point mass
    };
    const Item items[] = {
        {0.0346, {0.0005, 0.0402, 0.0004}, {0.0, 0.0, 0.0}},      // Pixhawk 6C
        {0.032, {-0.0577, 0.171, -0.0405}, {0.0, 0.0, 0.0}},      // GPS puck on its mast
        {0.045, {0.0982, 0.001, -0.044}, {0.0, 0.0, 0.0}},        // Raspberry Pi 5
        {0.004, {0.1568, -0.0004, -0.0119}, {0.0, 0.0, 0.0}},     // Camera Module 3
        {0.0235, {-0.0577, 0.0304, 0.0405}, {0.0, 0.0, 0.0}},     // SiK radio
        {0.0063, {-0.0577, 0.095, -0.0405}, {0.0, 0.0, 0.0}},     // GPS mast
        {0.175 - esc_mass, {0.0, 0.0113, 0.0}, {0.07, 0.016, 0.07}},  // ESCs, power module, wiring, fasteners, less the ESC block
    };
    double mass = 0.0;
    glm::dvec3 centre(0.0);
    for (const Item& i : items) {
        mass += i.mass;
        centre += i.mass * i.at;
    }
    centre /= mass;
    glm::dmat3 tensor(0.0);
    for (const Item& i : items) {
        const glm::dvec3 r = i.at - centre;
        const double r2 = glm::dot(r, r);
        for (int c = 0; c < 3; ++c) {
            for (int k = 0; k < 3; ++k) tensor[c][k] += i.mass * ((c == k ? r2 : 0.0) - r[c] * r[k]);
        }
        tensor[0][0] += i.mass * (i.dims.y * i.dims.y + i.dims.z * i.dims.z) / 12.0;
        tensor[1][1] += i.mass * (i.dims.x * i.dims.x + i.dims.z * i.dims.z) / 12.0;
        tensor[2][2] += i.mass * (i.dims.x * i.dims.x + i.dims.y * i.dims.y) / 12.0;
    }
    vehicles::PartInertia rec;
    rec.mass = mass;
    rec.position = glm::dvec3(centre.x, centre.y + kDesignAboveTable, centre.z);
    rec.shape = vehicles::PartShape::tensor;
    rec.tensor = tensor - cancel;
    rec.geometry.name = "avionics";
    rec.geometry.role = "avionics";
    return rec;
}

// The model: a basic quad at KAT's layout, the build table's parts as records (THE MASS, above), the 10-inch propeller and the library's 920 KV
// motor class (the X500's 2216 KV920). The pack is the library's phantom3 4S cells with the table's Tattu 5200 mass and box under the stack.
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
    const auto battery_text = read("phantom3_4s_4480.battery.yaml");
    if (!motor_text) return std::unexpected(motor_text.error());
    if (!esc_text) return std::unexpected(esc_text.error());
    if (!battery_text) return std::unexpected(battery_text.error());
    const auto motor = sc::motor_from_yaml(*motor_text);
    const auto esc = sc::esc_from_yaml(*esc_text);
    const auto battery = sc::battery_from_yaml(*battery_text);
    if (!motor) return std::unexpected(motor.error());
    if (!esc) return std::unexpected(esc.error());
    if (!battery) return std::unexpected(battery.error());

    const double arm_length = std::sqrt(2.0 * kArm * kArm);
    constexpr double kArmPerMetre = 0.10;  // kg/m, Holybro's 16 mm carbon tubes (assumed)
    constexpr double kFrameMass = 0.610;   // kg, the kit's bare frame (Holybro kit page, published)
    const double d = kDesignAboveTable;

    const auto make = [&](const glm::dmat3& cancel) -> Result<vehicles::AirframeSpec> {
        vehicles::BasicQuadParams p;
        p.name = "x500";
        p.half_span_x = kArm;
        p.half_span_z = kArm;
        p.hub_height = kHubAboveCentre + d;  // the table's hub pose, exactly
        p.body_size = glm::dvec3(0.15, 0.0809, 0.15);  // the plate stack: tray underside (table y -0.0466) to the damper deck's top (0.0343)
        p.body_mass = kFrameMass - 4.0 * kArmPerMetre * arm_length;
        p.arm_radius = 0.008;  // the X500's 16 mm carbon tubes
        p.arm_mass_per_metre = kArmPerMetre;
        p.motor = motor->block;
        p.prop = prop.block;
        p.esc = esc->block;
        p.esc.mount.position = glm::dvec3(0.0, 0.0113 + d, 0.0);  // with the misc carrier, between the plates
        p.battery = battery->block;
        p.battery.mass = kPackMass;
        p.battery.mount.position = glm::dvec3(0.0, -0.0634 + d, 0.0);  // the pack's centre; its top face on the tray's underside
        p.battery.shape = vehicles::BlockShape{vehicles::PartShape::box, glm::dvec3(0.133, 0.0335, 0.045), glm::dmat3(0.0)};
        auto spec = vehicles::basic_quad_airframe(p);
        if (!spec) return spec;
        spec->parts[0].position = glm::dvec3(0.0, -0.0061 + d, 0.0);  // the stack's middle: (-0.0466 + 0.0343) / 2 in the table's y
        spec->parts.push_back(avionics_record(cancel, esc->block.mass));
        return spec;
    };

    // Two passes: the composite's products of inertia from the true avionics tensor, then the avionics tensor less them (linear: the centre of
    // mass does not move, and the composite's products go to zero).
    const auto first = make(glm::dmat3(0.0));
    if (!first) return first;
    const auto compiled = vehicles::compile_airframe(*first);
    if (!compiled) return std::unexpected(compiled.error());
    glm::dmat3 products(0.0);
    for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) {
            if (c != k) products[c][k] = compiled->inertia.inertia_design[c][k];
        }
    }
    return make(products);
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
