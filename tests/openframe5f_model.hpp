#pragma once

// ---------------------------------------------------------------------------
// openframe5f_model.hpp -- the website clips' flow model of the OpenFrame-5F, the 5-inch FPV kit of OpenDrone (Incutec BV) that Cameron chose (O138, the
// lead, 2026-11-21): built from Interface's build table (tools/flow_movie/data/openframe5f_build/parts.yaml at commit 3157ff931a43b2b5a2337be1417d2b05c876c8c9
// on interface/openframe-5f-parts: 14 entries, 0.5665 kg). Made as the X500's (tests/x500_model.hpp): the generator writes tools/flow_movie/data/openframe5f/
// (openframe5f.model.yaml, parts/openframe5f_5in_mit5x4.prop.yaml and openframe5f.notes.yaml's text); tests/test_openframe5f_case.cpp checks the files against
// this builder and, with SPADE_OPENFRAME_WRITE set, writes them.
//
// THE DESIGN FRAME is the table's (Y up, +X forward, +Z right, the origin the bottom plate's lower face). The model's body origin is the composite's centre
// of mass; the rotors, mounts and parts are the design-frame ones shifted by -com_offset and turned by design_to_principal (the same rule as the X500's).
//
// MASS, CENTRE OF MASS AND INERTIA are the table's: kKitMass, kKitCom and the tensor below are what tests/test_openframe5f_inertia.cpp prints from the part
// store's meshes (opt-in, local store): each frame object at uniform density (the frame's 0.170 kg shared by mesh volume), the boards and the pack as uniform
// boxes, each 2306 stand-in as a uniform cylinder, each prop as a thin disc of R 0.0635 m. As the X500's, the airframe is the compile's: proxy solids for the flow
// (three plates, four arms, the ESC board, the pack, four motors), and ONE tensor record, "kit_remainder", that completes the table's composite (mass, centre
// of mass, whole tensor); it has no solid. Its parts are the Cross, the camera side plates, the standoffs, the printed TPU parts, the flight controller and the
// receiver (55 g). The tensor's products turn the principal axes 2.9 degrees about z, which the model file carries.
//
// THE PROXY SOLIDS. A frame object's mass is the frame's 0.170 kg by its mesh volume (the carbon, the two aluminium side plates and the standoffs
// share one density, 1915 kg/m3: an assumption, as the X500's). Each proxy is at its mesh's centroid and sized so its second moments are the mesh's (read from the
// part store by the inertia test, [kit-obj]): the meshes have cut-outs and an arm has a boss, so a box of the bounding extents puts the mass in the wrong place and
// leaves the remainder record no body at all (the compile refuses a tensor that breaks the triangle inequality; the first build did). A plate is a box of the
// mesh's thickness and the x, z sizes that give its moments (within 1 cm of the mesh's extents); an arm is a box 6 mm thick (the kit's arms are 6 mm carbon),
// 16.35 mm wide (its volume) and 128.6 mm long (its moment about the vertical), turned along the bore. The ESC and the pack are the table's boxes at the
// table's poses. A motor is the table's cylinder (phi 29.3 mm, 30.6 mm long, 32 g) at the table's pose, 0.0243 m up: its own part record (a cylinder), not the
// motor block's, because the compile puts a block's motor half its length under the hub (0.0283 m, 4 mm above the arm, and 4 mm of the disc plane's gap below
// the table's); the block's mass is 0, "counted elsewhere", and the block carries the chain's kv and electrical numbers only.
//
// THE PROPELLER is a stand-in (kPropCaveat): UIUC's measured 5 x 4 in three-blade (MIT) tables and the B1-fitted blade of the library's uiuc_mit_5x4, 4 g
// assumed, at the kit's 5 in. The HQProp 5X4.3X3 V2S the maker's build names has no published data or CAD.
//
// THE MOTOR is the library's 2306 with the OpenMotor 2306 6S 1950KV's published kv (1950 rpm/V); its resistance, no-load current, current limit, poles and rotor
// inertia are the library's typical-2306 estimates (assumed); its 32 g and size are the table's (assumed, a retailer's stand-in). The pack is the library's 6S 1300 mAh
// cells at the table's 0.200 kg and 78 x 38 x 39 mm. The chain only fixes the hover throttle; the rotors turn at the speed the propeller gives for a quarter of the weight.
// ---------------------------------------------------------------------------

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "physics/airflow/blade.hpp"
#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "scene/part_file.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/basic_quad.hpp"
#include "x500_model.hpp"

namespace spade::testing::openframe5f {

inline constexpr const char* kTableCommit = "3157ff931a43b2b5a2337be1417d2b05c876c8c9";  // interface/openframe-5f-parts: openframe5f_build/parts.yaml
inline constexpr double kMass = 0.5665;       // kg, the table's 14 entries
inline constexpr double kArmX = 0.069534;     // m, the motor axes along the kit's +X (Interface, measured on Arm.step)
inline constexpr double kArmZ = 0.089000;     // m, across (+Z right)
inline constexpr double kDiscHeight = 0.0436; // m, the disc plane above the table's origin (the motor's top 0.0396 m and 4 mm, assumed)
inline constexpr double kDiameter = 0.127;    // m, 5 in
inline constexpr double kPropMass = 0.004;    // kg each, assumed
inline constexpr double kMotorMass = 0.032;   // kg each, assumed
inline constexpr double kMotorRadius = 0.01465;       // m, the table's 29.3 mm
inline constexpr double kMotorLength = 0.0306;  // m, the table's
inline constexpr double kMotorCentreY = 0.0243;       // m, the table's motor centre
inline constexpr double kPackMass = 0.200;    // kg, assumed 6S 1300 mAh
inline constexpr double kEscMass = 0.015;     // kg, the maker's estimate
inline constexpr double kFrameMass = 0.170;   // kg, the maker's, without the TPU parts
inline constexpr double kArmThickness = 0.006;  // m
inline constexpr double kPlateBottomVolume = 1.3034e-5, kPlateMiddleVolume = 1.2124e-5, kPlateTopVolume = 1.0124e-5, kArmVolume = 1.1077e-5;  // m3, the meshes'
// The frame's objects' total mesh volume, m3: 3 plates, the Cross (1.4757e-6), four arms, two side plates (3.3647e-6 each), four standoffs (2.4889e-7 each).
inline constexpr double kFrameVolume = 1.3034e-5 + 1.2124e-5 + 1.0124e-5 + 1.4757e-6 + 4.0 * 1.1077e-5 + 2.0 * 3.3647e-6 + 4.0 * 2.4889e-7;

// The kit's composite in the table's frame: mass, centre of mass, and the tensor about it (kg m^2, design axes), at 17 digits, as tests/test_openframe5f_inertia.cpp
// prints them from the part store (the table at kTableCommit).
inline constexpr double kKitMass = 0.5665;
inline const glm::dvec3 kKitCom{0.0019101285138302412, 0.030043893093994901, -2.2717493251024976e-06};
inline constexpr double kKitXX = 0.0018792221444677254, kKitYY = 0.0029701013928679076, kKitZZ = 0.0015936648328637719;
inline constexpr double kKitXY = 5.507092446054248e-05, kKitXZ = -6.0353443814192425e-08, kKitYZ = 1.4963343780912801e-08;

[[nodiscard]] inline std::filesystem::path data_dir() {
    return std::filesystem::path(SPADE_ASSETS_DIR).parent_path() / "tools" / "flow_movie" / "data" / "openframe5f";
}
inline constexpr const char* kModelFile = "openframe5f.model.yaml";
inline constexpr const char* kPropFile = "parts/openframe5f_5in_mit5x4.prop.yaml";
inline constexpr const char* kNotesFile = "openframe5f.notes.yaml";

// The line the card carries about the propeller (the lead, 2026-11-21).
inline constexpr std::string_view kPropCaveat =
    "The propeller is a stand-in: UIUC's measured 5 x 4 in three-blade (MIT) tables and the library's fitted blade stand for the kit's HQProp 5X4.3X3, which has "
    "no published data; the blade's polar was fitted at 6,057 rpm with a 12 mm kernel and is used here at about 9,400 rpm with a 1.25 mm cell, and the lines "
    "carry about 0.9 of a quarter of the weight at the hover speed.";

[[nodiscard]] inline std::string notes_text() {
    return std::string(R"NOTES(# The website clips' flow model of an open-source 5 inch FPV quadcopter (the OpenFrame-5F kit): where each number came from.
# Written by tests/openframe5f_model.hpp notes_text() and compared by tests/test_openframe5f_case.cpp (SPADE_OPENFRAME_WRITE=1 regenerates it). Grades are
# the library's: measured, published, retailer, derived, fitted, assumed. Anything not carried is listed at the end.
model: "openframe5f.model.yaml"
prop: "parts/openframe5f_5in_mit5x4.prop.yaml"
built_by: "tests/openframe5f_model.hpp airframe(): vehicles::basic_quad_airframe() at the kit's hubs, proxy solids at the build table's poses, and one tensor record (kit_remainder) that completes the table's composite from the part meshes"
table:
  file: "tools/flow_movie/data/openframe5f_build/parts.yaml"
  branch: "interface/openframe-5f-parts"
  commit: ")NOTES") + kTableCommit + R"NOTES("
  note: "14 entries on the design frame (+X forward, +Y up, +Z right, the origin the bottom plate's lower face), each mass graded there: the frame 170 g published (the maker's README, without the TPU parts), the printed TPU parts 29.7 g assumed (an upper bound from their CAD volume), the flight controller 7 g, ESC 15 g and receiver 0.8 g the maker's own estimates (published), the four 2306 motors 32 g each and the four propellers 4 g each assumed, the 6S 1300 mAh pack 200 g assumed; the sum, 0.5665 kg, is nobody's take-off weight (no camera, video transmitter, antennas, wiring or fasteners beyond the frame's own are in it)"
licence: "the OpenFrame-5F design files are CERN-OHL-S-2.0 (OpenDrone, Incutec BV; the table's header has the release and commit)"
inputs:
  mass: {value: 0.5665, unit: kg, grade: derived, note: "the sum of the build table's 14 entries' masses, each graded there"}
  centre_of_mass: {value: "model.com_offset in openframe5f.model.yaml (the table's frame, m): (0.00191, 0.03004, -0.0000023)", grade: derived, note: "the parts' mass-weighted centre from their meshes (TriangleMesh::mass_properties() per frame object at uniform density, the frame's 170 g shared by mesh volume; boxes for the boards, receiver and pack; uniform cylinders for the motors; thin discs for the propellers), tests/test_openframe5f_inertia.cpp on the shared part store. The design frame is the table's frame, so com_offset is that point and the body origin is the centre of mass"}
  inertia: {value: "see openframe5f.model.yaml's body.inertia and design_to_principal", unit: kg m^2, grade: derived, note: "the table's tensor about the centre of mass from the same meshes: Ixx 1.8792e-3, Iyy 2.9701e-3, Izz 1.5937e-3, xy 5.51e-5, xz -6e-8, yz 1.5e-8. The xy product turns the principal axes 2.9 degrees about z, which design_to_principal carries; nothing is dropped. Equal density across the frame's objects is assumed (carbon, aluminium and steel alike)"}
  rotors: {value: "(+-0.069534, +-0.089000) m", grade: measured, note: "the motor bores' axes measured on the exported Arm (Interface; 225.9 mm across the diagonal against the maker's 226 mm wheelbase): a stretched X; rotor 0 front-right (+X, +Z) spin +1, 1 front-left -1, 2 rear-left +1, 3 rear-right -1, the library's fpv_5inch order and spins. The other three arms are derived from the one exported arm (two hands) by the table's stated rule"}
  disc_height: {value: "0.0436 m above the table's origin", grade: assumed, note: "the 2306 stand-in stands on the arm's top face (0.009 m), 30.6 mm tall (a retailer's BETAFPV Snow 2306 listing: no motor CAD exists), so its top is 0.0396 m; the propeller's disc plane is put 4 mm above it (the hub is not published)"}
  rotor_radius: {value: 0.0635, unit: m, grade: published, note: "a 5 inch propeller: 5 in = 0.127 m, halved (the maker's build names the HQProp 5X4.3X3 V2S)"}
  body: {value: "three plate boxes and four 6 mm arm boxes, 0.152 kg", grade: derived, note: "proxy solids for the flow: the bottom, middle and top plates as boxes at their meshes' centroids with the meshes' second moments, each carrying its mesh's share of the frame's 170 g, and the four arms as 6 mm boxes 16.35 mm wide and 128.6 mm long at each arm mesh's centroid, turned along the bore; the Cross, the camera side plates, the standoffs, the printed parts, the controller and the receiver are in the kit_remainder tensor record, with no solid"}
  battery: {value: "box 0.078 x 0.038 x 0.039 m, 0.200 kg, on the top plate's side of the stack", grade: assumed, note: "a Tattu R-Line 6S 1300 mAh class pack (retailer listings: 78 x 39 x 38 mm, 200 g +-20 g); the table's pose (-0.00385, 0.0535, 0), a layout choice the maker's build leaves open"}
  motor: {value: "the library's fpv_2306_1700kv with kv 1950 rpm/V, 0.032 kg", grade: assumed, note: "the OpenMotor 2306 6S 1950KV: its published kv; the resistance, no-load current, current limit, pole pairs and rotor inertia are the library's typical-2306 estimates; the 32 g and the 29.3 x 30.6 mm cylinder are the table's (a retailer's BETAFPV Snow 2306 stand-in; the maker's README says the motor mass is not yet measured). The solid is the table's cylinder as its own part record at the table's pose, centre 0.0243 m up, standing on the arm's top face (the compile's block rule would put it 4 mm higher)"}
  esc_battery: {value: "the library's fpv_4in1_45a (0.015 kg, 41.6 x 42.5 x 5 mm) and fpv_6s_1300 cells", grade: assumed, note: "stand-ins; the chain only fixes the hover throttle, the rotors turn at the speed the propeller part gives for a quarter of the weight. The ESC's pose is the table's (stack holes, Y 0.0155 m)"}
  propeller: {value: "parts/openframe5f_5in_mit5x4.prop.yaml", grade: derived, note: "A STAND-IN: UIUC's measured 5 x 4 in three-blade (MIT) tables and the library's fitted blade for the HQProp 5X4.3X3 V2S the maker names, which has no published data or CAD. The polar (alpha_0 -4.3344, cd_scale 2.2885) was fitted at 6,057 rpm with a 12 mm kernel at R/dx 15 and is used here at about 9,400 rpm and a 1.25 mm cell (R/dx 50.8); the line's one-rotor thrust in the film box is about 0.9 of a quarter of the weight at hover (the X500's is 0.86 to 0.89)"}
checks:
  balance: "the grid-free blade-element-momentum solve (tests/bemt_reference.hpp) for a quarter of the weight: see tests/test_openframe5f_case.cpp OpenFrame5fBalance"
  box: "252 x 237 x 284 at 1.25 mm (tests/d1_case.hpp kD1FpvGrid, x along the kit, z across), the drone held by d1_fpv_hold(0.0436)"
not_carried: "the Cross, camera side plates, standoffs, TPU parts (boots, bumper, camera and VTX mounts, anti-slip pad), the flight controller and receiver as solids (they are the kit_remainder tensor record's mass, with no solid); the camera, video transmitter, antennas, wiring and fasteners (not in the table's 0.5665 kg); the propellers' hubs and the motors' bells as solids"
)NOTES";
}

// Another propeller's measured tables, used as the kit's stand-in: the library's uiuc_mit_5x4 with every source row's note saying so.
[[nodiscard]] inline Result<scene::PropPart> prop_part() {
    const std::filesystem::path file = std::filesystem::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_mit_5x4.prop.yaml";
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::unexpected(Error{Code::io_error, "cannot read " + file.string()});
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    auto base = scene::prop_from_yaml(text);
    if (!base) return std::unexpected(base.error());
    scene::PropPart p = *base;
    p.block.shape.reset();
    const std::string stand_in =
        " [the OpenFrame-5F: a stand-in for the kit's HQProp 5X4.3X3 V2S, which has no published data; the library's polar was fitted at 6,057 rpm with a 12 mm kernel]";
    for (scene::FieldSource& s : p.sources) {
        if (s.field == "ct_static" || s.field == "cq_static" || s.field == "ct_table" || s.field == "cq_table") {
            s.ref.grade = scene::Grade::derived;
            s.ref.note += stand_in;
        } else if (s.field == "mass") {
            s.ref.note += " [the OpenFrame-5F build table's 4 g, assumed]";
        } else if (s.field == "blade.airfoil" || s.field == "blades" || s.field == "diameter" || s.field == "pitch" || s.field == "inertia") {
            s.ref.note += stand_in;
        }
    }
    return p;
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

// The kit's composite less what `other` (the solids and blocks, compiled) already holds: one tensor record, no solid (as x500::kit_remainder).
[[nodiscard]] inline Result<vehicles::PartInertia> kit_remainder(const vehicles::CompositeInertia& other) {
    const double m_o = static_cast<double>(other.mass);
    const glm::dvec3 c_o(other.center_of_mass);
    const double m_r = kKitMass - m_o;
    if (!(m_r > 0.0)) return std::unexpected(Error{Code::invalid_argument, "the kit's mass is not above the proxy solids' and blocks'"});
    const glm::dvec3 c_r = (kKitMass * kKitCom - m_o * c_o) / m_r;
    const glm::dmat3 other_at_com = other.inertia_design + x500::parallel_axis(m_o, c_o - kKitCom);
    const glm::dmat3 remainder = kit_tensor() - other_at_com - x500::parallel_axis(m_r, c_r - kKitCom);
    vehicles::PartInertia rec;
    rec.mass = m_r;
    rec.position = c_r;
    rec.shape = vehicles::PartShape::tensor;
    rec.tensor = remainder;
    rec.geometry.name = "kit_remainder";
    rec.geometry.role = "avionics";
    return rec;
}

[[nodiscard]] inline vehicles::PartInertia proxy_box(const char* name, const char* role, double mass, const glm::dvec3& position, const glm::dvec3& size,
                                                     const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0)) {
    vehicles::PartInertia p;
    p.mass = mass;
    p.position = position;
    p.orientation = orientation;
    p.shape = vehicles::PartShape::box;
    p.size = size;
    p.geometry.name = name;
    p.geometry.role = role;
    return p;
}

// The model: a basic quad at the kit's hubs and the table's proxy solids, the stand-in propeller, the 2306 at 1950 KV and the 6S pack (see the header).
[[nodiscard]] inline Result<vehicles::AirframeSpec> airframe(const scene::PropPart& prop) {
    namespace sc = scene;
    const std::filesystem::path parts = std::filesystem::path(SPADE_ASSETS_DIR) / "library/parts";
    const auto read = [&](const char* name) -> Result<std::string> {
        std::ifstream in(parts / name, std::ios::binary);
        if (!in) return std::unexpected(Error{Code::io_error, std::string("cannot read ") + name});
        return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    };
    const auto motor_text = read("fpv_2306_1700kv.motor.yaml");
    const auto esc_text = read("fpv_4in1_45a.esc.yaml");
    const auto battery_text = read("fpv_6s_1300.battery.yaml");
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
    p.name = "openframe5f";
    p.half_span_x = kArmX;
    p.half_span_z = kArmZ;
    p.hub_height = kDiscHeight;
    p.motor = motor->block;
    p.motor.kv = 1950.0;  // the OpenMotor 2306 6S 1950KV's published kv
    p.motor.mass = 0.0;  // the four motors are part records below, at the table's pose
    p.motor.shape.reset();
    p.prop = prop.block;
    p.esc = esc->block;
    p.esc.mass = kEscMass;
    p.esc.mount.position = glm::dvec3(0.01275, 0.0155, 0.0);
    p.esc.shape = vehicles::BlockShape{vehicles::PartShape::box, glm::dvec3(0.0425, 0.005, 0.0416), glm::dmat3(0.0)};
    p.battery = battery->block;
    p.battery.mass = kPackMass;
    p.battery.mount.position = glm::dvec3(-0.00385, 0.0535, 0.0);
    p.battery.shape = vehicles::BlockShape{vehicles::PartShape::box, glm::dvec3(0.078, 0.038, 0.039), glm::dmat3(0.0)};
    auto spec = vehicles::basic_quad_airframe(p);
    if (!spec) return spec;

    // The proxy solids replace basic_quad's body box and arm tubes.
    spec->parts.clear();
    const double rho = kFrameMass / kFrameVolume;  // kg/m3: the frame's objects share one density
    // Each proxy sits at its mesh's centroid, sized so its inertia is the mesh's (the meshes have cut-outs and a boss: a box of the bounding extents has the
    // mass in the wrong place, and the remainder record then is not a body): plate boxes of the thickness and the x, z sizes that give the mesh's second
    // moments about its centroid (tests/test_openframe5f_inertia.cpp, [kit-obj]); an arm 6 mm thick, 16.35 mm wide (its mesh volume) and 128.6 mm long (its
    // moment about the vertical), turned along the bore, at the mesh's centroid (0.04365, 0.00600, 0.05507) m.
    spec->parts.push_back(proxy_box("plate_bottom", "body", rho * kPlateBottomVolume, glm::dvec3(0.02261, 0.0015, 0.00001), glm::dvec3(0.1053, 0.003, 0.0523)));
    spec->parts.push_back(proxy_box("plate_middle", "body", rho * kPlateMiddleVolume, glm::dvec3(-0.01552, 0.01048, 0.0), glm::dvec3(0.0943, 0.003, 0.0517)));
    spec->parts.push_back(proxy_box("plate_top", "body", rho * kPlateTopVolume, glm::dvec3(0.00131, 0.03325, -0.00008), glm::dvec3(0.1444, 0.0025, 0.0348)));
    const double arm_width = kArmVolume / kArmThickness / 0.1286;
    const std::array<glm::dvec2, 4> corners = {glm::dvec2(kArmX, kArmZ), glm::dvec2(kArmX, -kArmZ), glm::dvec2(-kArmX, -kArmZ), glm::dvec2(-kArmX, kArmZ)};
    for (std::size_t k = 0; k < corners.size(); ++k) {
        const double theta = std::atan2(-corners[k].y, corners[k].x);  // about +Y: local x onto the bore's direction
        const glm::dquat turn(std::cos(0.5 * theta), 0.0, std::sin(0.5 * theta), 0.0);
        const glm::dvec3 centroid(corners[k].x > 0.0 ? 0.04365 : -0.04365, 0.006, corners[k].y > 0.0 ? 0.05507 : -0.05507);
        spec->parts.push_back(proxy_box(("arm[" + std::to_string(k) + "]").c_str(), "arm", rho * kArmVolume, centroid,
                                        glm::dvec3(0.1286, kArmThickness, arm_width), turn));
    }
    for (std::size_t k = 0; k < corners.size(); ++k) {
        vehicles::PartInertia motor_part;
        motor_part.mass = kMotorMass;
        motor_part.position = glm::dvec3(corners[k].x, kMotorCentreY, corners[k].y);
        motor_part.shape = vehicles::PartShape::cylinder;
        motor_part.size = glm::dvec3(kMotorRadius, kMotorLength, 0.0);
        motor_part.geometry.name = "motor[" + std::to_string(k) + "]";
        motor_part.geometry.role = "motor";
        spec->parts.push_back(std::move(motor_part));
    }
    // The proxy solids and blocks compile as they are; the remainder completes the table's composite.
    const auto first = vehicles::compile_airframe(*spec);
    if (!first) return std::unexpected(first.error());
    auto remainder = kit_remainder(first->inertia);
    if (!remainder) return std::unexpected(remainder.error());
    spec->parts.push_back(*remainder);
    return spec;
}

// The rotor speed (rad/s) at which the grid-free blade-element-momentum solve gives `thrust` newtons (tests/x500_model.hpp's, the same 60 bisections).
[[nodiscard]] inline Result<float> balance_omega(const physics::airflow::BladeShape& blade, const physics::airflow::PolarTable& polar, float thrust) {
    return x500::balance_omega(blade, polar, thrust);
}

}  // namespace spade::testing::openframe5f
