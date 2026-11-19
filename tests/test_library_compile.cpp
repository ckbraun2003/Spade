// ---------------------------------------------------------------------------
// test_library_compile.cpp -- compile_drone (scene/library_compile.hpp; L88l, L103, L107): a drone's parts map to the AirframeSpec compile_airframe takes.
// A small synthetic quad in a temp root, every part a box or a point (no store), pins the assembly: the one frame change, the masses and the centre of
// mass, the rotors from the motor points and each prop's disc, the blocks from the parts' spec blocks, and the refusals.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "scene/library_compile.hpp"
#include "scene/library_query.hpp"
#include "scene/library_set.hpp"
#include "library_spec_text.hpp"
#include "unique_temp.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "x500_model.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;
namespace vh = spade::vehicles;

fs::path fresh(const std::string& name) {
    static std::vector<std::unique_ptr<spade::test::TempDir>> dirs;
    dirs.push_back(std::make_unique<spade::test::TempDir>("library_compile_" + name));
    return dirs.back()->path();
}

void put(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

std::string slurp(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

const fs::path kParts = fs::path(SPADE_ASSETS_DIR) / "library" / "parts";

// A quad in the way the X500's parts are authored: every part in the axes of its own file (+X forward, +Y up, +Z right; a prop's thrust is its +Y), the frame
// placed at Rx(+90 deg) into the library's design frame (+X forward, +Y left, +Z up), so the compile's one frame change (Rx(-90)) brings every pose back to the
// authored axes. Every part its own boxes, in a root `r`.
//   frame   0.6 kg, a 0.2 x 0.04 x 0.2 box at its origin; motor points at (+-0.12, 0.02, +-0.12) with spin +1 -1 +1 -1 (front right, front left, rear
//           left, rear right); a battery point at y -0.04; an esc point at y +0.03
//   motor   0.055 kg, a 0.03 x 0.025 x 0.03 box at its origin; its prop_hub at y +0.02
//   prop    0.013 kg; its `disc` point at y +0.005 (the hub is the disc)
//   pack    0.4 kg, a 0.12 x 0.05 x 0.04 box;  esc carrier 0.1 kg, a 0.04 x 0.01 x 0.04 box
// Total 1.372 kg.
const std::string kBlocksHead = "part_version: 2\n";

std::string frame_text() {
    return kBlocksHead + R"(id: "part.frame.f"
mass_properties: {mass: 0.6}
install_points:
  - {name: motor_fr, kind: motor, pos: [0.12, 0.02, 0.12], orient: [1, 0, 0, 0], spin: 1}
  - {name: motor_fl, kind: motor, pos: [0.12, 0.02, -0.12], orient: [1, 0, 0, 0], spin: -1}
  - {name: motor_rl, kind: motor, pos: [-0.12, 0.02, -0.12], orient: [1, 0, 0, 0], spin: 1}
  - {name: motor_rr, kind: motor, pos: [-0.12, 0.02, 0.12], orient: [1, 0, 0, 0], spin: -1}
  - {name: battery_pad, kind: battery, pos: [0, -0.04, 0], orient: [1, 0, 0, 0]}
  - {name: esc_pad, kind: esc, pos: [0, 0.03, 0], orient: [1, 0, 0, 0]}
shapes:
  - {kind: box, size: [0.2, 0.04, 0.2]}
sources:
  mass_properties.mass: {grade: assumed, note: "a test frame"}
  install_points.motor_fr: {grade: assumed, note: "a test point"}
  install_points.motor_fl: {grade: assumed, note: "a test point"}
  install_points.motor_rl: {grade: assumed, note: "a test point"}
  install_points.motor_rr: {grade: assumed, note: "a test point"}
  install_points.battery_pad: {grade: assumed, note: "a test point"}
  install_points.esc_pad: {grade: assumed, note: "a test point"}
)";
}

std::string motor_text() {
    return kBlocksHead + R"(id: "part.motor.m"
mass_properties: {mass: 0.055}
install_points:
  - {name: prop_hub, kind: prop_hub, pos: [0, 0.02, 0], orient: [1, 0, 0, 0]}
shapes:
  - {kind: box, size: [0.03, 0.025, 0.03]}
sources:
  mass_properties.mass: {grade: assumed, note: "a test motor"}
  install_points.prop_hub: {grade: assumed, note: "a test point"}
)" + spade::test::as_spec(slurp(kParts / "iris_920kv.motor.yaml"));
}

std::string prop_text() {
    return kBlocksHead + R"(id: "part.prop.p"
mass_properties: {mass: 0.013}
install_points:
  - {name: disc, kind: prop_hub, pos: [0, 0.005, 0], orient: [1, 0, 0, 0]}
shapes:
  - {kind: box, size: [0.2, 0.02, 0.002]}
sources:
  mass_properties.mass: {grade: assumed, note: "a test prop"}
  install_points.disc: {grade: assumed, note: "a test point"}
)" + spade::test::as_spec(slurp(kParts / "uiuc_apce_10x5.prop.yaml"));
}

std::string pack_text() {
    return kBlocksHead + R"(id: "part.battery.b"
mass_properties: {mass: 0.4}
shapes:
  - {kind: box, size: [0.12, 0.05, 0.04]}
sources:
  mass_properties.mass: {grade: assumed, note: "a test pack"}
)" + spade::test::as_spec(slurp(kParts / "phantom3_4s_4480.battery.yaml"));
}

std::string esc_text() {
    return kBlocksHead + R"(id: "part.payload.e"
mass_properties: {mass: 0.1}
shapes:
  - {kind: box, size: [0.04, 0.01, 0.04]}
sources:
  mass_properties.mass: {grade: assumed, note: "a test carrier"}
)" + spade::test::as_spec(slurp(kParts / "iris_4x20a.esc.yaml"));
}

// `drone` is the parts map's lines (each starting with six spaces); `extra_parts` more entries and files.
sc::LibrarySet open_quad(const std::string& name, const std::string& drone, const fs::path& store = {}, const std::string& frame = frame_text()) {
    const fs::path root = fresh(name) / "r";
    put(root / "library.yaml", "library_version: 3\nname: r\ndepends: []\nindexes:\n  - index.yaml\n");
    put(root / "index.yaml",
        "index_version: 1\nentries:\n"
        "  - id: part.frame.f\n    name: frame\n    category: Parts\n    file: parts/f.frame.yaml\n"
        "  - id: part.motor.m\n    name: motor\n    category: Parts\n    file: parts/m.motor.yaml\n"
        "  - id: part.prop.p\n    name: prop\n    category: Parts\n    file: parts/p.prop.yaml\n"
        "  - id: part.battery.b\n    name: pack\n    category: Parts\n    file: parts/b.battery.yaml\n"
        "  - id: part.payload.e\n    name: esc carrier\n    category: Parts\n    file: parts/e.payload.yaml\n"
        "  - id: drone.quad\n    name: quad\n    category: drones\n    parts_map:\n" + drone);
    put(root / "parts/f.frame.yaml", frame);
    put(root / "parts/m.motor.yaml", motor_text());
    put(root / "parts/p.prop.yaml", prop_text());
    put(root / "parts/b.battery.yaml", pack_text());
    put(root / "parts/e.payload.yaml", esc_text());
    const std::vector<sc::LibraryRootSpec> roots = {{"r", root, false}};
    auto set = sc::open_libraries(roots, store);
    EXPECT_TRUE(set.has_value()) << (set.has_value() ? "" : set.error().message);
    return set.has_value() ? std::move(*set) : sc::LibrarySet{};
}

const std::string kQuad =
    "      - {slot: frame, part: \"r:part.frame.f\", placement: {pos: [0, 0, 0], quat: [0.707106781187, 0.707106781187, 0, 0]}}\n"
    "      - {slot: m_fr, part: \"r:part.motor.m\", parent: frame, install_point: motor_fr}\n"
    "      - {slot: m_fl, part: \"r:part.motor.m\", parent: frame, install_point: motor_fl}\n"
    "      - {slot: m_rl, part: \"r:part.motor.m\", parent: frame, install_point: motor_rl}\n"
    "      - {slot: m_rr, part: \"r:part.motor.m\", parent: frame, install_point: motor_rr}\n"
    "      - {slot: pack, part: \"r:part.battery.b\", parent: frame, install_point: battery_pad}\n"
    "      - {slot: esc, part: \"r:part.payload.e\", parent: frame, install_point: esc_pad}\n"
    "      - {slot: p_fr, part: \"r:part.prop.p\", parent: m_fr, install_point: prop_hub}\n"
    "      - {slot: p_fl, part: \"r:part.prop.p\", parent: m_fl, install_point: prop_hub}\n"
    "      - {slot: p_rl, part: \"r:part.prop.p\", parent: m_rl, install_point: prop_hub}\n"
    "      - {slot: p_rr, part: \"r:part.prop.p\", parent: m_rr, install_point: prop_hub}\n";

sc::CompileOptions with_zero_drag() {
    sc::CompileOptions o;
    o.drag = vh::DragSpec{};
    return o;
}

TEST(LibraryCompile, AQuadOfBoxesCompilesToTheSumOfItsPartsInTheSpadeFrame) {
    const sc::LibrarySet set = open_quad("sum", kQuad);
    const auto got = sc::compile_drone(set, "r:drone.quad", with_zero_drag());
    ASSERT_TRUE(got.has_value()) << got.error().code << ": " << got.error().message;
    // mass: 0.6 + 4 x 0.055 + 4 x 0.013 + 0.4 + 0.1
    EXPECT_NEAR(got->compiled.inertia.mass, 1.372, 1e-6);
    // centre of mass: y = (0.22 x 0.02 + 0.052 x 0.04 + 0.4 x -0.04 + 0.1 x 0.03) / 1.372 = -0.0047522 (the propellers at their parts' origins, y 0.04)
    EXPECT_NEAR(got->compiled.inertia.center_of_mass.x, 0.0, 1e-6);
    EXPECT_NEAR(got->compiled.inertia.center_of_mass.y, -0.00652 / 1.372, 1e-6);
    EXPECT_NEAR(got->compiled.inertia.center_of_mass.z, 0.0, 1e-6);
    // the assembled spec is in the Spade frame, which is the authored axes: the first rotor (front right) is at x 0.12, z +0.12, its hub at the disc: y 0.02 + 0.02 + 0.005
    ASSERT_EQ(got->spec.rotors.size(), 4u);
    EXPECT_NEAR(got->spec.rotors[0].hub.position.x, 0.12, 1e-9);
    EXPECT_NEAR(got->spec.rotors[0].hub.position.y, 0.045, 1e-9);
    EXPECT_NEAR(got->spec.rotors[0].hub.position.z, 0.12, 1e-9);
    // the rotors in the model are about the centre of mass, in the body frame
    ASSERT_EQ(got->compiled.model.rotors.size(), 4u);
    EXPECT_NEAR(got->compiled.model.rotors[0].local_pos.y, 0.045 + 0.00652 / 1.372, 1e-5);
    // spins from the motor points, in order
    EXPECT_EQ(got->spec.rotors[0].spin_dir, 1.0);
    EXPECT_EQ(got->spec.rotors[1].spin_dir, -1.0);
    EXPECT_EQ(got->spec.rotors[2].spin_dir, 1.0);
    EXPECT_EQ(got->spec.rotors[3].spin_dir, -1.0);
}

TEST(LibraryCompile, TheBlocksComeFromThePartsSpecsWithTheirMassesOnTheRecords) {
    const sc::LibrarySet set = open_quad("blocks", kQuad);
    const auto got = sc::compile_drone(set, "r:drone.quad", with_zero_drag());
    ASSERT_TRUE(got.has_value()) << got.error().code << ": " << got.error().message;
    EXPECT_EQ(got->spec.motor.kv, 920.0) << "rpm/V, as the v1 block spells it";
    EXPECT_EQ(got->spec.motor.mass, 0.0) << "the motor part's records are the mass";
    EXPECT_EQ(got->spec.esc.mass, 0.0);
    EXPECT_EQ(got->spec.battery.mass, 0.0);
    EXPECT_EQ(got->spec.prop.mass, 0.0) << "the propeller's mass is its disc record's, at its part's origin";
    EXPECT_EQ(got->spec.prop.diameter > 0.0, true);
    EXPECT_EQ(got->spec.battery.cells_series, 4u);
    EXPECT_EQ(got->spec.esc.channels, 4u);
    EXPECT_EQ(got->spec.drag.size(), 1u) << "the zero DragSpec given";
}

TEST(LibraryCompile, DragIsRequiredAndTheRefusalsAreNamed) {
    const sc::LibrarySet set = open_quad("refuse", kQuad);
    const auto no_drag = sc::compile_drone(set, "r:drone.quad", sc::CompileOptions{});
    ASSERT_FALSE(no_drag.has_value());
    EXPECT_EQ(no_drag.error().code, "invalid_request");
    EXPECT_EQ(no_drag.error().field, "drag");
    const auto absent = sc::compile_drone(set, "r:drone.nope", with_zero_drag());
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error().code, "entry_not_found");
    const auto bare = sc::compile_drone(set, "drone.quad", with_zero_drag());
    ASSERT_FALSE(bare.has_value());
    EXPECT_EQ(bare.error().code, "bare_id");
    const auto not_a_drone = sc::compile_drone(set, "r:part.frame.f", with_zero_drag());
    ASSERT_FALSE(not_a_drone.has_value());
    EXPECT_EQ(not_a_drone.error().code, "invalid_entry");
}

std::string store_from_env() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* v = std::getenv("SPADE_PART_STORE");
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    return v != nullptr ? v : "";
}

// A cube of side `side` metres from the origin, saved as <store>/<sha>/<file>.
void save_cube(const fs::path& store, const std::string& sha, const std::string& file, double side) {
    static constexpr uint32_t kTriangles[12][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                                                   {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
    spade::MeshInput in;
    in.unit = spade::MeshUnit::metre;
    for (uint32_t v = 0; v < 8; ++v) {
        in.positions.push_back(glm::dvec3(static_cast<double>(v & 1u), static_cast<double>((v >> 1) & 1u), static_cast<double>((v >> 2) & 1u)) * side);
    }
    for (const auto& t : kTriangles) in.indices.insert(in.indices.end(), {t[0], t[1], t[2]});
    auto mesh = spade::make_mesh(in, "cube");
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;
    fs::create_directories(store / sha);
    auto saved = spade::save_mesh_file(store / sha / file, **mesh);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
}

// The frame with a cube mesh (0.1 m from its origin) as its solid, and its boxes kept as the stand-ins: the meshes win.
std::string frame_with_cube(const std::string& sha) {
    std::string t = frame_text();
    const std::size_t at = t.find("shapes:\n");
    t.insert(at, "meshes:\n  - {store: {sha256: \"" + sha + "\", file: \"cube.spmesh\"}}\n");
    return t;
}

// THE RULE: a part with meshes is its meshes for physics, never its stand-in boxes; and a mesh needs the store.
TEST(LibraryCompile, AMeshPartIsATensorRecordAtItsCentroidAndNeedsTheStore) {
    const std::string sha(64, 'c');
    const fs::path store = fresh("store");
    save_cube(store, sha, "cube.spmesh", 0.1);
    const sc::LibrarySet set = open_quad("mesh", kQuad, store, frame_with_cube(sha));
    const auto got = sc::compile_drone(set, "r:drone.quad", with_zero_drag());
    ASSERT_TRUE(got.has_value()) << got.error().code << ": " << got.error().message;
    EXPECT_NEAR(got->compiled.inertia.mass, 1.372, 1e-6);
    // the frame's 0.6 kg is at the cube's centroid (0.05, 0.05, 0.05), not the box's origin: x = z = 0.03 / 1.372, y = (0.03 - 0.00652) / 1.372
    EXPECT_NEAR(got->compiled.inertia.center_of_mass.x, 0.03 / 1.372, 1e-6);
    EXPECT_NEAR(got->compiled.inertia.center_of_mass.y, (0.03 - 0.00652) / 1.372, 1e-6);
    EXPECT_NEAR(got->compiled.inertia.center_of_mass.z, 0.03 / 1.372, 1e-6);
    // one tensor record for the cube: a cube's inertia is m a^2 / 6 about its centroid
    std::size_t tensors = 0;
    for (const vh::PartInertia& r : got->spec.parts) {
        if (r.shape != vh::PartShape::tensor || r.mass < 0.1) continue;  // the propellers' discs are tensor records too, of 13 g
        ++tensors;
        EXPECT_NEAR(r.mass, 0.6, 1e-9);
        EXPECT_NEAR(r.tensor[0][0], 0.6 * 0.1 * 0.1 / 6.0, 1e-9);
    }
    EXPECT_EQ(tensors, 1u);
    // without the store the same drone is refused, never given its boxes
    const sc::LibrarySet no_store = open_quad("mesh_nostore", kQuad, {}, frame_with_cube(sha));
    const auto refused = sc::compile_drone(no_store, "r:drone.quad", with_zero_drag());
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, "store_file_missing");
}

// The acceptance (L103, L107): the X500 drone, from the part store, is the kit Physics' flow model carries: its mass, centre of mass and whole tensor are the
// kit constants (tests/x500_model.hpp: the same meshes, the same uniform density per part, the same parallel axis), the principal frame 5.4 degrees about z.
// Opt-in: SPADE_PART_STORE.
// The kit constants are the table's parts from their own meshes; the entries reuse ONE motor mesh (the kit's DJ-2216, re-origined on its shaft axis, Spade's
// ruling for the entries) at the four points, where the table has the four distinct instances (DJ-2216-KV880_1_1 .. _4). That is the one difference this
// test finds (a motor's centroid is a fraction of a millimetre off its axis, and the four differ): the mass is exact, the centre of mass is within 5e-5 m and
// the tensor within 0.2 per cent. Tighten to 1e-9 when the constants are computed from the entries' one motor (Physics, L107).
constexpr double kMassPin = 1e-9;      // relative
constexpr double kComPin = 5e-5;       // m
constexpr double kTensorPin = 2e-3;    // relative to the diagonal; the products are compared to the same absolute size

TEST(LibraryCompile, TheX500DroneIsTheKitPhysicsFlowModelCarries) {
    const std::string store = store_from_env();
    if (store.empty()) GTEST_SKIP() << "SPADE_PART_STORE is not set";
    namespace kit = spade::testing::x500;
    const std::vector<sc::LibraryRootSpec> roots = {{"default", fs::path(SPADE_ASSETS_DIR) / "library", false}};
    auto set = sc::open_libraries(roots, fs::path(store));
    ASSERT_TRUE(set.has_value()) << set.error().message;
    // the frame's objects: 61, all solid
    const auto frame = sc::part_meshes(*set, "default:part.frame.x500_kit_frame");
    ASSERT_TRUE(frame.has_value()) << frame.error().message;
    ASSERT_EQ(frame->size(), 61u) << "the frame's store objects";
    for (const sc::PartMeshObject& o : *frame) EXPECT_FALSE(o.visual) << o.file;
    const auto got = sc::compile_drone(*set, "default:drone.x500_build", with_zero_drag());
    ASSERT_TRUE(got.has_value()) << got.error().code << ": " << got.error().message;
    // the records' sums in double, in the compile's order (the propellers are thin-disc records at their parts' origins, as the kit constants have them)
    double mass = 0.0;
    glm::dvec3 com{0.0};
    for (const vh::PartInertia& r : got->spec.parts) {
        mass += r.mass;
        com += r.mass * r.position;
    }
    com /= mass;
    EXPECT_NEAR(mass, kit::kKitMass, kMassPin * kit::kKitMass);
    EXPECT_NEAR(com.x, kit::kKitCom.x, kComPin);
    EXPECT_NEAR(com.y, kit::kKitCom.y, kComPin);
    EXPECT_NEAR(com.z, kit::kKitCom.z, kComPin);
    const glm::dmat3& I = got->compiled.inertia.inertia_design;
    EXPECT_NEAR(I[0][0], kit::kKitXX, kTensorPin * kit::kKitXX);
    EXPECT_NEAR(I[1][1], kit::kKitYY, kTensorPin * kit::kKitYY);
    EXPECT_NEAR(I[2][2], kit::kKitZZ, kTensorPin * kit::kKitZZ);
    EXPECT_NEAR(I[0][1], kit::kKitXY, kTensorPin * kit::kKitXX);
    EXPECT_NEAR(I[0][2], kit::kKitXZ, kTensorPin * kit::kKitXX);
    EXPECT_NEAR(I[1][2], kit::kKitYZ, kTensorPin * kit::kKitXX);
    // the principal frame: about 5.4 degrees about z
    const glm::quat q = got->compiled.model.design_to_principal;
    const double angle = 2.0 * std::acos(std::min(1.0, static_cast<double>(std::abs(q.w)))) * 180.0 / 3.14159265358979323846;
    EXPECT_NEAR(angle, 5.4, 0.1);
    EXPECT_GT(std::abs(q.z), 0.95f * std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z)) << "about z (the products xz and yz tilt the axis a little)";
    // the rotors: the kit's four, at the table's hub height (the disc)
    ASSERT_EQ(got->spec.rotors.size(), 4u);
    EXPECT_NEAR(got->spec.rotors[0].hub.position.y, kit::kDiscHeight, 1e-4) << "the disc: 0.070 - 0.00926; the constant rounds it to 0.0607";
    EXPECT_NEAR(std::abs(got->spec.rotors[0].hub.position.x), kit::kArm, 1e-9);
}

}  // namespace
