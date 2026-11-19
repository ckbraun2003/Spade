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
// MASS, CENTRE OF MASS AND INERTIA (L88l, L104, the lead, 2026-11-19) are the kit's: Core's build table (tools/flow_movie/data/x500_build/parts.yaml,
// commit b1a5a7f0d5a9366398fd48e6bf8d9abb213cd2b3 on core/x500-hubs-measured): 16 entries, 1.6326 kg. The table's frame is the kit file's (Y up, +X forward,
// +Z right, the origin the bottom plate's lower face, so NOT the centre of mass); it IS the design frame here. The composite's mass, centre of mass and full
// tensor are the table's parts' own, from their meshes where they have them (kKitMass, kKitCom, kKitTensor below; tests/test_x500_kit_inertia.cpp computes
// them from the part store, opt-in, and checks these constants): each part with store meshes (the frame's 61 objects, the motors, the controller, the GPS, the
// Pi, the camera) from TriangleMesh::mass_properties(), exact for the shape, at uniform density scaled to the part's measured mass (the frame's 0.610 kg split
// by object volume); the pack, the radio and the misc carrier as uniform boxes; each prop as a thin disc of R 0.127 m. The meshes are local (L88b), so the
// numbers are recorded here. Here the airframe is the compile's: proxy solids for the flow (the frame stack, four arm tubes, the pack, the ESC block, the
// motors) at the table's poses, plus ONE tensor record, "kit_remainder", that is the kit's composite less those solids and blocks, so the compile's
// composite is the kit's: mass, centre of mass and the whole tensor. The record has no solid.
//
// PRODUCTS OF INERTIA. The engine's model is principal-axes only, and compile_airframe turns the design frame to the composite's principal axes. This tensor
// has products (xy 4.85e-4, yz -5.5e-5, xz -1.7e-6 kg m^2; 2% of the diagonal) that turn the principal axes 5.4 degrees about z from the design axes, so the
// model file carries design_to_principal as the compile gives it (a rotation of 5.4 degrees about z), and every body-frame field in it (rotors, mounts, parts)
// is the design-frame one turned by it and shifted by -com_offset: r_body = R (p_design - com_offset). Nothing is dropped and nothing cancelled. Equal
// density across the frame's 61 objects is assumed (the same assumption as Core's centre of mass for the frame).
//
// THE PROPELLER (L86p, the lead, 2026-11-18, correcting L86k): the real X500 V2 kit ships 1045 propellers (10 x 4.5 in; Holybro's kit page,
// docs.holybro.com/drone-development-kit/px4-development-kit-x500v2, and KAT's own x500.airframe.json evidence "500 mm wheelbase, 2216 KV920,
// 1045 propellers"); the 13-inch 1345 STL is the Gazebo sim's prop, whose thrust constants (k_T 1.23e-5 N s^2/rad^2, 642 rad/s to hover) are
// the sim's, not the kit's. So R is 0.127 m and the blade is the library's UIUC APC 10x5 (measured geometry, C_T and C_Q tables) at UIUC's own
// 10 in; the library's part applies it at 9.4 in (NASA's rotor), so only the diameter (and the mass-derived inertia) differ. Graded derived: the
// 5 in pitch against the kit's 4.5 in is said in the notes. Its hover speed is the speed the grid-free blade-element-momentum solve gives for a
// quarter of the weight (test_x500_case.cpp), ~5450 rpm at the table's mass; the line's thrust at the chain's own speed is checked against the same quarter.
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
inline constexpr double kMass = 1.6326;                                                // kg, the build table's 16 entries (Core, b1a5a7f0)
inline constexpr const char* kTableCommit = "b1a5a7f0d5a9366398fd48e6bf8d9abb213cd2b3";  // core/x500-hubs-measured: x500_build/parts.yaml
inline constexpr double kArmAxisY = 0.016;                                             // m, the arm tubes' axes in the table's frame (assumed: between the plates, y 0.001 and 0.031)
inline constexpr double kArm = 0.17689;                                                // m, the motor axes' x and z (Interface, a circle fit on the kit's mesh, L98)
inline constexpr double kDiameter = 0.254;                                             // m, the kit's 1045: 10 in tip to tip
inline constexpr double kDiscHeight = 0.0607;                                          // m, the actuator disc above the table's origin (L99a: the blade mid-plane)
inline constexpr double kPackMass = 0.4365;                                            // kg, Tattu G-Tech 5200 mAh 4S (Grepow listing, retailer)
inline constexpr double kPropMass = 0.013;                                             // kg, the library's APC 10x5 (assumed, a typical 9-10 in moulded prop)

// The kit's composite in the table's frame (see MASS, above): mass, centre of mass, and the tensor about it (kg m^2, design axes), as
// tests/test_x500_kit_inertia.cpp prints them from the part store. L107: at 17 significant digits (a double round-trips), so Core's compile can pin
// them tightly; they are the same numbers as before at 9 digits, so the model files do not change. The mass is the table's sum, 1.6326 (the test sums 1.6326000000000001).
inline constexpr double kKitMass = 1.6326;
inline const glm::dvec3 kKitCom{-0.0044646534873306106, -0.025430141162826263, 0.00087805400318969771};
inline constexpr double kKitXX = 0.023792895661676021, kKitYY = 0.028919492384880705, kKitZZ = 0.023395635476505722;
inline constexpr double kKitXY = 0.00048523054049281355, kKitXZ = -1.7069101480298322e-06, kKitYZ = -5.5412022481528728e-05;

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
built_by: "tests/x500_model.hpp airframe(): vehicles::basic_quad_airframe() at the kit's layout, proxy solids at the build table's poses, and one tensor record (kit_remainder) that completes the kit's composite from the table's meshes"
table:
  file: "tools/flow_movie/data/x500_build/parts.yaml"
  branch: "core/x500-hubs-measured"
  commit: ")NOTES") + kTableCommit + R"NOTES("
  note: "16 entries on the kit file's frame, each mass graded there: the frame 0.610 kg published (Holybro kit page) as 61 mesh objects, Pixhawk 6C 34.6 g, GPS 32 g and SiK radio 23.5 g published (Holybro docs), Raspberry Pi 5 45 g and the 4S pack 436.5 g retailer, camera 4 g, misc ESCs/wiring/fasteners 175 g and the four motors 55 g and propellers 13 g assumed; the sum, 1.6326 kg, is nobody's take-off weight"
kat:
  file: "content/airframes/x500.airframe.json"
  commit: ")NOTES" + kKatCommit + R"NOTES("
  note: "the layout and PX4's spins only (KAT's own provenance: PX4-gazebo-models commit e5997f455e33ec071c7c677fc59a65781de496a4, x500_base/model.sdf, and the Holybro X500 V2 pages, opened 2026-11-10); its mass 2.0643 kg and inertia (the Gazebo model's sum) are replaced by the build table's"
licence: "the PX4 x500 model is BSD-3-Clause; KAT's records hold the text"
inputs:
  mass: {value: 1.6326, unit: kg, grade: derived, note: "the sum of the build table's 16 entries' masses, each graded there (frame published, pack retailer, misc, motors and propellers assumed); it replaces the Gazebo model's 2.0643 kg"}
  centre_of_mass: {value: "model.com_offset in x500.model.yaml (the table's frame, m): (-0.00446, -0.02543, 0.00088)", grade: derived, note: "the kit's parts' mass-weighted centre from their meshes (TriangleMesh::mass_properties() per part at uniform density scaled to the part's mass, the frame's 0.610 kg split by its 61 objects' volumes; boxes for the pack, the radio and the misc carrier; thin discs for the propellers), 0.02, 0.38 and 0.32 mm from Core's own (-0.00448, -0.02581, 0.00056), which used the parts' boxes. The design frame is the table's frame (its origin the bottom plate's lower face), so com_offset is that point and the model's body origin is the centre of mass"}
  inertia: {value: "see x500.model.yaml's body.inertia and design_to_principal", unit: kg m^2, grade: derived, note: "the kit's tensor about the centre of mass from the same meshes: Ixx 0.0237929, Iyy 0.0289195, Izz 0.0233956, products xy 4.85e-4, yz -5.54e-5, xz -1.7e-6 kg m^2. The products turn the principal axes 5.4 degrees about z from the design axes, so the file's design_to_principal is that rotation and body.inertia the principal moments (0.0237474, 0.0289656, 0.0233951); nothing is dropped or cancelled, and every body-frame field is the design-frame one turned by it and shifted by -com_offset. Equal density across the frame's 61 objects is assumed (carbon, aluminium and steel alike). It replaces the earlier model's (0.0146, 0.0226, 0.0158 at 1.6389 kg), a box-and-tube sum that put the frame's mass in the stack"}
  rotors: {value: "(+-0.17689, +-0.17689) m", grade: measured, note: "the kit's motor shaft axes, a circle fit through the shaft and the can on the kit's mesh (Interface, L98), 500.3 mm motor to motor on the diagonal; PX4's X layout: front-right and rear-left counter-clockwise (spin_dir +1), front-left and rear-right clockwise (-1)"}
  disc_height: {value: "0.0607 m above the table's origin", grade: measured, note: "the actuator disc is the blade mid-plane: the propeller's hub on the motor's bell boss at 0.054 m, the mid-plane 6.7 mm above the hub's bottom (Core, measured on the rotor glTF, L99a); the drawing's rotor origin is 0.070 m, which is not the disc. The model's rotor local_pos is R (hub - com_offset) with R the file's design_to_principal"}
  rotor_radius: {value: 0.127, unit: m, grade: derived, note: "the kit's 1045 propeller (10 x 4.5 in; Holybro's X500 V2 kit page and KAT's x500.airframe.json evidence, '1045 propellers'): 10 in = 0.254 m, halved; it is the propeller part's diameter / 2. The 13-inch 1345 STL in PX4's Gazebo model is the sim's prop, not the kit's"}
  body: {value: "box 0.15 x 0.0805 x 0.15 m, 0.5116 kg", grade: derived, note: "the plate stack from the tray pad's underside (y -0.0485) to the top plate's top face (0.032) of the build table's frame, a proxy solid for the flow; its mass is the frame's 0.610 kg less the four arm tubes' 0.0984 kg"}
  arms: {value: "tubes of 8 mm radius, 0.10 kg/m, axes at y 0.016", grade: assumed, note: "Holybro's 16 mm carbon arm tubes between the plates (y 0.001 and 0.031); proxy solids"}
  battery: {value: "box 0.133 x 0.0335 x 0.045 m, 0.4365 kg, under the tray", grade: retailer, note: "the Tattu G-Tech 5200 mAh 4S pack (Grepow listing), its top face on the pad's underside (y -0.0485), centre y -0.0653, long axis along +X; a solid in the flow, in the propellers' downwash; the pose is assumed (a layout choice the frame leaves open)"}
  motor: {value: "the library's iris_920kv, 0.055 kg", grade: assumed, note: "the X500's 2216 KV920, a stand-in of the same class"}
  esc_battery: {value: "the library's iris_4x20a (0.028 kg) and phantom3_4s_4480 cells", grade: assumed, note: "stand-ins; the pack's mass and shape are the table's. The chain only fixes the hover throttle; the rotors turn at the speed the propeller part gives for a quarter of the weight. The table's 0.175 kg misc carrier covers the ESCs: the ESC block's 0.028 kg is inside that mass and the kit_remainder record holds the rest"}
  propeller: {value: "parts/x500_10in_apc10x5.prop.yaml", grade: derived, note: "the library's UIUC APC 10x5 blade (measured geometry, C_T and C_Q tables) used at UIUC's own 10 in, the kit's 1045 size; the pitch is UIUC's 5 in against the kit's 4.5 in; the library's part applies the same tables at 9.4 in. PX4's Gazebo thrust constants (642 rad/s to hover) are the sim's"}
checks:
  balance: "the grid-free blade-element-momentum solve (tests/bemt_reference.hpp) for a quarter of the weight, 4.0027 N: see tests/test_x500_case.cpp X500Balance"
  line_thrust: "re-measured on the kit model after L90 (the open-outlet fix): docs/design/physics/plans/2026-11-19-x500-kit-remeasure.md; the figures on the earlier 1.6389 kg model, before L90, are in this file's git history"
  settled_from: "see the re-measure record above"
  step: "see the re-measure record above: before L90 the thrust at a fixed rotor speed depended on h (the open faces refused inflow, L90), after it the share of momentum is h-independent"
  box: "270 x 237 x 270 at 2.5 mm (4 cells wider since the motor axes are 0.17689 m, L99a), the drone lowered 13 rows (tests/d1_case.hpp d1_x500_hold()): the discs' centres 0.1930 m (1.5 R is 0.1905 m) under the top face, about 0.40 m of wake below, 0.0186 m past the kernel's reach at the sides"
not_carried: "landing legs, the camera and GPS mounts, the wiring, the propellers' hubs and the motors' bells as solids; everything that is not a proxy solid or a block (the plates beyond the stack box, the arm clamps, the landing gear, the controller, Pi, camera, radio, GPS, mast and the misc carrier) is the kit_remainder tensor record, with no solid; the frame's plates are one box, with no gaps between them"
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

// The inertia of a point mass `m` at `r` from the reference point, as a tensor.
[[nodiscard]] inline glm::dmat3 parallel_axis(double m, const glm::dvec3& r) {
    glm::dmat3 out(0.0);
    const double r2 = glm::dot(r, r);
    for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) out[c][k] = m * ((c == k ? r2 : 0.0) - r[c] * r[k]);
    }
    return out;
}

[[nodiscard]] inline glm::dmat3 kit_tensor() {
    glm::dmat3 t(0.0);
    t[0][0] = kKitXX;
    t[1][1] = kKitYY;
    t[2][2] = kKitZZ;
    t[1][0] = t[0][1] = kKitXY;
    t[2][0] = t[0][2] = kKitXZ;
    t[2][1] = t[1][2] = kKitYZ;
    return t;
}

// The kit's composite less what `other` (the solids and blocks, compiled) already holds: one tensor record, no solid (see MASS and PRODUCTS OF
// INERTIA). Its mass is what is left, its centre where the kit's centre of mass and the others' balance, its tensor about that centre what completes
// the kit's tensor about the kit's centre of mass.
[[nodiscard]] inline Result<vehicles::PartInertia> kit_remainder(const vehicles::CompositeInertia& other) {
    const double m_o = static_cast<double>(other.mass);
    const glm::dvec3 c_o(other.center_of_mass);
    const double m_r = kKitMass - m_o;
    if (!(m_r > 0.0)) return std::unexpected(Error{Code::invalid_argument, "the kit's mass is not above the proxy solids' and blocks'"});
    const glm::dvec3 c_r = (kKitMass * kKitCom - m_o * c_o) / m_r;
    const glm::dmat3 other_at_com = other.inertia_design + parallel_axis(m_o, c_o - kKitCom);
    const glm::dmat3 remainder = kit_tensor() - other_at_com - parallel_axis(m_r, c_r - kKitCom);
    vehicles::PartInertia rec;
    rec.mass = m_r;
    rec.position = c_r;
    rec.shape = vehicles::PartShape::tensor;
    rec.tensor = remainder;
    rec.geometry.name = "kit_remainder";
    rec.geometry.role = "avionics";
    return rec;
}

// The model: a basic quad at KAT's layout with the kit's hubs, proxy solids at the table's poses and the kit's composite completed by one tensor record
// (MASS, above), the 10-inch propeller and the library's 920 KV motor class (the X500's 2216 KV920). The pack is the library's phantom3 4S cells with the
// table's Tattu 5200 mass and box under the stack.
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

    vehicles::BasicQuadParams p;
    p.name = "x500";
    p.half_span_x = kArm;
    p.half_span_z = kArm;
    p.hub_height = kDiscHeight;                    // the disc plane in the table's frame, exactly
    p.body_size = glm::dvec3(0.15, 0.0805, 0.15);  // the plate stack: the tray pad's underside (table y -0.0485) to the top plate's top face (0.032)
    p.body_mass = kFrameMass - 4.0 * kArmPerMetre * arm_length;
    p.arm_radius = 0.008;  // the X500's 16 mm carbon tubes
    p.arm_mass_per_metre = kArmPerMetre;
    p.motor = motor->block;
    p.prop = prop.block;
    p.esc = esc->block;
    p.esc.mount.position = glm::dvec3(0.0, 0.016, 0.0);  // with the misc carrier, between the plates
    p.battery = battery->block;
    p.battery.mass = kPackMass;
    p.battery.mount.position = glm::dvec3(0.0, -0.0653, 0.0);  // the pack's centre; its top face on the pad's underside (-0.0485)
    p.battery.shape = vehicles::BlockShape{vehicles::PartShape::box, glm::dvec3(0.133, 0.0335, 0.045), glm::dmat3(0.0)};
    auto spec = vehicles::basic_quad_airframe(p);
    if (!spec) return spec;
    spec->parts[0].position = glm::dvec3(0.0, -0.00825, 0.0);  // the stack's middle: (-0.0485 + 0.032) / 2 in the table's y
    for (std::size_t k = 1; k <= 4; ++k) spec->parts[k].position.y = kArmAxisY;  // basic_quad's arms are in y = 0: put them between the plates
    // The proxy solids and blocks compile as they are; the remainder completes the kit's composite.
    const auto first = vehicles::compile_airframe(*spec);
    if (!first) return std::unexpected(first.error());
    auto remainder = kit_remainder(first->inertia);
    if (!remainder) return std::unexpected(remainder.error());
    spec->parts.push_back(*remainder);
    return spec;
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
