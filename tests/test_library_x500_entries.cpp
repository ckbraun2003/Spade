// ---------------------------------------------------------------------------
// test_library_x500_entries.cpp -- L88l: the default library's X500 build (drone.x500_build and its parts) is the pose table
// (tools/flow_movie/data/x500_build/parts.yaml) that Rendering draws from.
//
// The table is y up, +X forward, +Z right; the library's design frame is z up, +X forward, +Y left, so a table point (X, Y, Z) is (X, -Z, Y) there
// (the rotation Rx(+90)). A part's mesh keeps its file's own axes; the kit's frame file is the table's frame turned 180 degrees about Y.
//
//   LibraryX500Entries.EveryPartOfTheTableIsAnEntryOfTheDroneInItsPose   the drone's parts_map composed up its parent chain (compose_placement): the
//                                                  frame, every payload part and every rotor at its table pose to 1e-9; each motor at its table
//                                                  axis (x and z) and orientation, its height the motor part's own mount
//   LibraryX500Entries.EveryPartCarriesTheTablesMassAndMeshes          mass equals mass_kg, the meshes (store sha256 and file, in order) equal
//                                                  the table's objects, a rotor's meshes are visual; the masses sum to 1.6326 kg (the 4 motors one
//                                                  part used four times); a dimensioned part has its box and no mesh
//   LibraryX500Entries.TheFramesMotorPointsAreTheKitsAxesWithTheEnginesSpins   four motor points at x and z +-0.17689 m (the kit's motor axes) with the
//                                                  model's spins +1, -1, +1, -1 in rotor order
//   LibraryX500Entries.TheRotorPartsCarryTheSeatAndTheDiscAndTheMotorTheBoss   the motor's prop_hub is the bell boss (+0.032); each rotor part has
//                                                  `seat` (glTF y -0.0160) and `disc` (y -0.00926); composed, the rotor origin is 0.070 up and the disc 0.0607
//   LibraryX500Entries.TheStoreHasEveryObjectWhenOneIsGiven            only with SPADE_PART_STORE set: part_meshes() reads every part's objects
// Without SPADE_PART_STORE the tests run on the committed text alone.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <yaml-cpp/yaml.h>

#include "scene/library_place.hpp"
#include "scene/library_query.hpp"
#include "scene/library_set.hpp"
#include "scene/part_blocks.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;

const fs::path kRoot = fs::path(SPADE_ASSETS_DIR) / "library";
const std::string kTable = std::string(SPADE_REPO_DIR) + "/tools/flow_movie/data/x500_build/parts.yaml";
constexpr double kHub = 0.17689;  // the kit motors' shaft axes, measured (Interface's circle fit)

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

std::string slurp(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

sc::LibrarySet open_default() {
    const std::string store = store_from_env();
    const std::vector<sc::LibraryRootSpec> roots = {{"default", kRoot, false}};
    auto set = sc::open_libraries(roots, store.empty() ? fs::path{} : fs::path(store));
    EXPECT_TRUE(set.has_value()) << (set.has_value() ? "" : set.error().message);
    return set.has_value() ? std::move(*set) : sc::LibrarySet{};
}

const sc::SetEntry* entry_of(const sc::LibrarySet& set, const std::string& id) {
    for (const sc::SetEntry& e : set.entries) {
        if (e.id == id) return &e;
    }
    return nullptr;
}

sc::PartBlocks blocks_of(const sc::LibrarySet& set, const std::string& full_id) {
    const sc::SetEntry* e = entry_of(set, full_id);
    EXPECT_NE(e, nullptr) << full_id;
    if (e == nullptr) return {};
    auto blocks = sc::read_part_blocks(slurp(e->file));
    EXPECT_TRUE(blocks.has_value()) << full_id << ": " << (blocks.has_value() ? "" : blocks.error().context);
    return blocks.has_value() ? *blocks : sc::PartBlocks{};
}

// ---- the table ----

struct TableObject {
    std::string sha256;
    std::string file;
};
struct TablePart {
    std::string name;
    std::string kind;
    double mass = 0.0;
    double pos[3] = {0, 0, 0};
    double q[4] = {1, 0, 0, 0};
    std::vector<TableObject> objects;
    std::vector<double> box;
    int spin = 0;
};

std::vector<TablePart> read_table() {
    std::vector<TablePart> out;
    const YAML::Node doc = YAML::LoadFile(kTable);
    for (const YAML::Node& n : doc["parts"]) {
        TablePart p;
        p.name = n["name"].as<std::string>();
        p.kind = n["kind"].as<std::string>();
        p.mass = n["mass_kg"].as<double>();
        for (int i = 0; i < 3; ++i) p.pos[i] = n["pose"]["position"][i].as<double>();
        for (int i = 0; i < 4; ++i) p.q[i] = n["pose"]["orientation"][i].as<double>();
        if (n["objects"]) {
            for (const YAML::Node& o : n["objects"]) {
                p.objects.push_back({o["store_ref"]["sha256"].as<std::string>(), o["store_ref"]["file"].as<std::string>()});
            }
        }
        if (n["box"]) {
            for (const YAML::Node& v : n["box"]) p.box.push_back(v.as<double>());
        }
        if (n["spin"]) p.spin = n["spin"].as<int>();
        out.push_back(std::move(p));
    }
    return out;
}

// ---- pose maths, independent of the library's (the test's own) ----

using Q = std::array<double, 4>;  // w x y z

Q qmul(const Q& a, const Q& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

Q positive(Q q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (double& c : q) c /= n;
    for (double c : q) {
        if (std::fabs(c) > 1e-12) {
            if (c < 0) {
                for (double& d : q) d = -d;
            }
            break;
        }
    }
    return q;
}

// A table pose as the design frame's: the rotation Rx(+90) after it.
struct Pose {
    std::array<double, 3> p;
    Q q;
};
Pose to_design(const double p[3], const double q[4]) {
    const double r = std::sqrt(0.5);
    return {{p[0], -p[2], p[1]}, positive(qmul({r, r, 0, 0}, {q[0], q[1], q[2], q[3]}))};
}

// The drone's slots composed up their parent chains.
std::map<std::string, Pose> compose_drone(const sc::LibrarySet& set) {
    std::map<std::string, Pose> out;
    auto found = sc::find_entry(set, "default:drone.x500_build", {});
    EXPECT_TRUE(found.has_value()) << (found.has_value() ? "" : found.error().message);
    if (!found) return out;
    std::map<std::string, std::string> part_of;
    for (const sc::PartsMapSlot& slot : found->entry->parts_map) {
        sc::PlacementItem item;
        item.key = slot.slot;
        if (slot.placement) {
            item.install_point = *slot.placement;
        } else {
            const auto parent = out.find(slot.parent);
            EXPECT_NE(parent, out.end()) << "slot " << slot.slot << " names a parent not placed before it: " << slot.parent;
            if (parent == out.end()) continue;
            sc::SetPose pp;
            pp.pos = parent->second.p;
            pp.quat = parent->second.q;
            item.parent = pp;
            bool seen = false;
            for (const sc::PartInstallPoint& ip : blocks_of(set, part_of[slot.parent]).install_points) {
                if (ip.name != slot.install_point) continue;
                seen = true;
                item.install_point.pos = {ip.pos.x, ip.pos.y, ip.pos.z};
                item.install_point.quat = {ip.orient.w, ip.orient.x, ip.orient.y, ip.orient.z};
            }
            EXPECT_TRUE(seen) << "no install point '" << slot.install_point << "' on " << part_of[slot.parent];
            item.adjust = slot.adjust;
            // a part with a `seat` point stands on its parent's point by that seat: its own mount offset is the seat's inverse (the rotor's hub bottom on the bell boss)
            for (const sc::PartInstallPoint& seat : blocks_of(set, slot.part).install_points) {
                if (seat.name != "seat") continue;
                EXPECT_EQ(seat.orient, glm::dquat(1.0, 0.0, 0.0, 0.0)) << slot.part << ": the test inverts a seat with no turn only";
                sc::SetPose mount;
                mount.pos = {-seat.pos.x, -seat.pos.y, -seat.pos.z};
                item.part_to_mount = mount;
            }
        }
        part_of[slot.slot] = slot.part;
        const auto placed = sc::compose_placement({item});
        EXPECT_EQ(placed.size(), 1u);
        EXPECT_TRUE(placed[0].pose.has_value()) << slot.slot;
        if (!placed[0].pose) continue;
        out[slot.slot] = {placed[0].pose->pos, {placed[0].pose->quat[0], placed[0].pose->quat[1], placed[0].pose->quat[2], placed[0].pose->quat[3]}};
    }
    return out;
}

// The slot each table part sits in.
const std::map<std::string, std::string> kSlotOfTablePart = {
    {"x500_frame", "frame"},
    {"pixhawk_6c", "fc"},
    {"gps_m8n_m9n_v1", "gnss"},
    {"raspberry_pi_5", "companion"},
    {"camera_module_3", "camera"},
    {"lipo_4s_5200", "battery"},
    {"sik_radio_v3", "radio"},
    {"electronics_misc", "esc"},
    {"motor_front_right", "motor_front_right"},
    {"motor_front_left", "motor_front_left"},
    {"motor_rear_left", "motor_rear_left"},
    {"motor_rear_right", "motor_rear_right"},
    {"rotor_front_right", "prop_front_right"},
    {"rotor_front_left", "prop_front_left"},
    {"rotor_rear_left", "prop_rear_left"},
    {"rotor_rear_right", "prop_rear_right"},
};

void expect_pose(const Pose& got, const Pose& want, const std::string& what) {
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(got.p[i], want.p[i], 1e-9) << what << " position " << i;
    for (int i = 0; i < 4; ++i) EXPECT_NEAR(got.q[i], want.q[i], 1e-9) << what << " orientation " << i;
}

TEST(LibraryX500Entries, EveryPartOfTheTableIsAnEntryOfTheDroneInItsPose) {
    const sc::LibrarySet set = open_default();
    const std::map<std::string, Pose> poses = compose_drone(set);
    const std::vector<TablePart> table = read_table();
    ASSERT_EQ(table.size(), kSlotOfTablePart.size());
    for (const TablePart& part : table) {
        const auto slot = kSlotOfTablePart.find(part.name);
        ASSERT_NE(slot, kSlotOfTablePart.end()) << part.name;
        const auto got = poses.find(slot->second);
        ASSERT_NE(got, poses.end()) << "no slot " << slot->second << " for " << part.name;
        if (part.kind == "motor") {
            // the motor part is one object about its own axis: the table has the kit's motor in place, so the pin is the axis (x and z) and the turn
            const Pose want = to_design(part.pos, part.q);
            const double hub_x = part.name.find("front") != std::string::npos ? kHub : -kHub;
            const double hub_z = part.name.find("right") != std::string::npos ? kHub : -kHub;
            EXPECT_NEAR(got->second.p[0], hub_x, 1e-9) << part.name;
            EXPECT_NEAR(got->second.p[1], -hub_z, 1e-9) << part.name << " (the design frame's y is the table's -z)";
            for (int i = 0; i < 4; ++i) EXPECT_NEAR(got->second.q[i], want.q[i], 1e-9) << part.name << " orientation " << i;
        } else {
            expect_pose(got->second, to_design(part.pos, part.q), part.name);
        }
    }
}

TEST(LibraryX500Entries, EveryPartCarriesTheTablesMassAndMeshes) {
    const sc::LibrarySet set = open_default();
    auto found = sc::find_entry(set, "default:drone.x500_build", {});
    ASSERT_TRUE(found.has_value()) << found.error().message;
    std::map<std::string, std::string> part_of;
    for (const sc::PartsMapSlot& s : found->entry->parts_map) part_of[s.slot] = s.part;
    double sum = 0.0;
    for (const TablePart& part : read_table()) {
        const std::string slot = kSlotOfTablePart.at(part.name);
        ASSERT_NE(part_of.count(slot), 0u) << slot;
        const sc::PartBlocks blocks = blocks_of(set, part_of[slot]);
        ASSERT_TRUE(blocks.mass_properties.has_value()) << part.name;
        EXPECT_NEAR(blocks.mass_properties->mass, part.mass, 1e-12) << part.name;
        sum += blocks.mass_properties->mass;
        if (part.kind == "motor") continue;  // the motor part is the kit's motor re-origined: its own store object, not the table's in-place one
        if (!part.objects.empty()) {
            ASSERT_EQ(blocks.meshes.size(), part.objects.size()) << part.name;
            for (std::size_t i = 0; i < part.objects.size(); ++i) {
                EXPECT_EQ(blocks.meshes[i].sha256, part.objects[i].sha256) << part.name << " object " << i;
                EXPECT_EQ(blocks.meshes[i].file, part.objects[i].file) << part.name << " object " << i;
                EXPECT_EQ(blocks.meshes[i].visual, part.kind == "rotor") << part.name << " object " << i;
            }
        } else {
            EXPECT_TRUE(blocks.meshes.empty()) << part.name;
            ASSERT_EQ(blocks.shapes.size(), 1u) << part.name;
            EXPECT_EQ(blocks.shapes[0].size, part.box) << part.name;
        }
    }
    EXPECT_NEAR(sum, 1.6326, 1e-4);
}

TEST(LibraryX500Entries, TheFramesMotorPointsAreTheKitsAxesWithTheEnginesSpins) {
    const sc::LibrarySet set = open_default();
    const sc::PartBlocks frame = blocks_of(set, "default:part.frame.x500_kit_frame");
    std::map<std::string, const sc::PartInstallPoint*> points;
    for (const sc::PartInstallPoint& p : frame.install_points) points[p.name] = &p;
    // rotor order and spins: Physics' x500 model (front-right +1, front-left -1, rear-left +1, rear-right -1)
    const struct {
        const char* name;
        int spin;
        double x_table;
        double z_table;
    } rotors[] = {{"motor_front_right", 1, kHub, kHub}, {"motor_front_left", -1, kHub, -kHub}, {"motor_rear_left", 1, -kHub, -kHub}, {"motor_rear_right", -1, -kHub, kHub}};
    for (const auto& r : rotors) {
        ASSERT_NE(points.count(r.name), 0u) << r.name;
        const sc::PartInstallPoint& p = *points[r.name];
        EXPECT_EQ(p.kind, "motor") << r.name;
        EXPECT_EQ(p.spin, r.spin) << r.name;
        // in the kit file's axes (the table turned 180 degrees about Y): x = -x_table, z = -z_table
        EXPECT_NEAR(p.pos.x, -r.x_table, 1e-9) << r.name;
        EXPECT_NEAR(p.pos.z, -r.z_table, 1e-9) << r.name;
    }
}

TEST(LibraryX500Entries, TheRotorPartsCarryTheSeatAndTheDiscAndTheMotorTheBoss) {
    const sc::LibrarySet set = open_default();
    // the motor: its prop_hub is the bell boss, 0.032 above its origin (the flange), measured by Interface on the re-origined motor
    const sc::PartBlocks motor = blocks_of(set, "default:part.motor.dj2216_kv880");
    ASSERT_EQ(motor.install_points.size(), 1u);
    EXPECT_EQ(motor.install_points[0].name, "prop_hub");
    EXPECT_NEAR(motor.install_points[0].pos.y, 0.032, 1e-12);
    // each rotor part: the seat is the glTF's hub bottom face (0.0160 below its origin) and the disc the blade mid-plane (0.00926 below it), measured by Core
    for (const char* id : {"x500_rotor_front_ccw", "x500_rotor_front_cw", "x500_rotor_rear_ccw", "x500_rotor_rear_cw"}) {
        const sc::PartBlocks rotor = blocks_of(set, std::string("default:part.prop.") + id);
        std::map<std::string, glm::dvec3> at;
        for (const sc::PartInstallPoint& p : rotor.install_points) at[p.name] = p.pos;
        ASSERT_EQ(at.count("seat"), 1u) << id;
        ASSERT_EQ(at.count("disc"), 1u) << id;
        EXPECT_NEAR(at["seat"].y, -0.0160, 1e-12) << id;
        EXPECT_NEAR(at["disc"].y, -0.00926, 1e-12) << id;
    }
    // composed: the rotor origin 0.070 up (the table) and its disc 0.0607 up, in the design frame's z
    const std::map<std::string, Pose> poses = compose_drone(set);
    for (const char* slot : {"prop_front_right", "prop_front_left", "prop_rear_left", "prop_rear_right"}) {
        ASSERT_EQ(poses.count(slot), 1u) << slot;
        EXPECT_NEAR(poses.at(slot).p[2], 0.070, 1e-9) << slot << " origin height";
        EXPECT_NEAR(poses.at(slot).p[2] - 0.00926, 0.0607, 1e-4) << slot << " disc height";
    }
}

TEST(LibraryX500Entries, TheStoreHasEveryObjectWhenOneIsGiven) {
    if (store_from_env().empty()) GTEST_SKIP() << "SPADE_PART_STORE is not set";
    const sc::LibrarySet set = open_default();
    std::size_t objects = 0;
    for (const sc::SetEntry& e : set.entries) {
        if (e.kind != "part" || e.rel_file.rfind("parts/x500/", 0) != 0) continue;
        const auto got = sc::part_meshes(set, e.id);
        ASSERT_TRUE(got.has_value()) << e.id << ": " << got.error().message << " " << got.error().path;
        objects += got->size();
    }
    EXPECT_GT(objects, 60u);
}

}  // namespace
