// ---------------------------------------------------------------------------
// test_x500_build_table.cpp -- L88l, L88s: the X500 build's pose table (tools/flow_movie/data/x500_build/parts.yaml), on the Holybro kit's own CAD,
// against the engine's rotor convention and Physics' x500 model (tools/flow_movie/data/x500/x500.model.yaml).
//
//   X500BuildTable.ARotorTakesTheFileWhoseBladesLiftTurningTheEnginesWay   spin_dir +1 is counter-clockwise seen from above
//                                                (vehicles/rotor.hpp, consumers.md L7a): a +1 rotor takes the *-ccw file, a -1 rotor
//                                                the *-cw file; a front file sits at +X, a rear file at -X
//   X500BuildTable.TheHubsAreOnTheKitsMotorAxes   the four rotors keep the model's order and spin_dir and the model's signs; each hub is
//                                                +-0.17689 m on x and on z (the kit's motor shaft axes, measured by a circle fit: 500.3 mm motor to motor on the diagonal), 0.070 m up (the glTF's hub bottom on the bell boss), the disc at 0.0607
//   X500BuildTable.TheFrameIsTheKitsOwnCadAndTheMotorsAreTheKitsMotors   61 kit objects in the frame (no drawing-only glTF), the 4 kit
//                                                motors one entry each, every kit object posed by the one turn of the file to the design frame
//   X500BuildTable.EachRotorHidesItsOwnBell   a rotor entry's own `hide_materials` names its glTF's bell (59 mm across; the kit's motor is
//                                                drawn instead); the top-level key (the frame's) is gone
//   X500BuildTable.EveryEntryCarriesItsMassAndItsGrades   mass_kg, mass_grade, mass_source and pose_grade on every entry; the sum is 1.6326 kg
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

const std::string kTable = std::string(SPADE_REPO_DIR) + "/tools/flow_movie/data/x500_build/parts.yaml";
const std::string kModel = std::string(SPADE_REPO_DIR) + "/tools/flow_movie/data/x500/x500.model.yaml";

struct Rotor {
    int index = -1;
    int spin = 0;
    double x = 0.0, y = 0.0, z = 0.0;
    std::string file;
};

std::vector<Rotor> table_rotors() {
    std::vector<Rotor> out;
    const YAML::Node doc = YAML::LoadFile(kTable);
    for (const YAML::Node& part : doc["parts"]) {
        if (part["kind"].as<std::string>() != "rotor") continue;
        Rotor r;
        r.index = part["rotor"].as<int>();
        r.spin = part["spin"].as<int>();
        const YAML::Node p = part["pose"]["position"];
        r.x = p[0].as<double>();
        r.y = p[1].as<double>();
        r.z = p[2].as<double>();
        EXPECT_EQ(part["objects"].size(), 1u) << "a rotor is one glTF";
        r.file = part["objects"][0]["store_ref"]["file"].as<std::string>();
        out.push_back(r);
    }
    return out;
}

TEST(X500BuildTable, ARotorTakesTheFileWhoseBladesLiftTurningTheEnginesWay) {
    const std::vector<Rotor> rotors = table_rotors();
    ASSERT_EQ(rotors.size(), 4u);
    std::set<int> seen;
    for (const Rotor& r : rotors) {
        seen.insert(r.index);
        ASSERT_TRUE(r.spin == 1 || r.spin == -1) << r.file;
        const bool ccw_file = r.file.find("-ccw.gltf") != std::string::npos;
        const bool cw_file = r.file.find("-cw.gltf") != std::string::npos;
        EXPECT_TRUE(ccw_file != cw_file) << r.file;
        EXPECT_EQ(ccw_file, r.spin == 1) << "spin_dir +1 is counter-clockwise: rotor " << r.index << " takes " << r.file;
        const bool front_file = r.file.find("-front-") != std::string::npos;
        EXPECT_EQ(front_file, r.x > 0.0) << "a front file sits at +X: " << r.file;
    }
    EXPECT_EQ(seen, (std::set<int>{0, 1, 2, 3}));
}

TEST(X500BuildTable, TheHubsAreOnTheKitsMotorAxes) {
    const std::vector<Rotor> rotors = table_rotors();
    const YAML::Node model = YAML::LoadFile(kModel)["model"]["rotors"];
    ASSERT_EQ(model.size(), 4u);
    ASSERT_EQ(rotors.size(), 4u);
    for (const Rotor& r : rotors) {
        const YAML::Node m = model[static_cast<std::size_t>(r.index)];
        const YAML::Node pos = m["local_pos"];
        // the model's order, spin and sides (its magnitudes follow the kit once Physics moves its rotors)
        EXPECT_EQ(r.spin, m["spin_dir"].as<int>()) << "rotor " << r.index;
        EXPECT_EQ(r.x > 0.0, pos[0].as<double>() > 0.0) << "rotor " << r.index;
        EXPECT_EQ(r.z > 0.0, pos[2].as<double>() > 0.0) << "rotor " << r.index;
        // the kit's motor axes: +-176.89 mm on each axis (Interface's circle fit through the shaft and the can wall, 0.03 mm; the earlier 177.4 was a bounding-box
        // centre, 0.5 mm outward), 500.3 mm motor to motor
        EXPECT_NEAR(std::fabs(r.x), 0.17689, 1e-6) << "rotor " << r.index;
        EXPECT_NEAR(std::fabs(r.z), 0.17689, 1e-6) << "rotor " << r.index;
        // the seat, measured: Interface's re-origined motor has its bell boss top 0.032 above the flange (y 0.022), so 0.054; the rotor glTF's hub bottom face
        // is 0.016 below the glTF origin (Core, measured on x500-rotor-front-ccw.gltf), so the origin, the drawn pose, is at 0.070
        EXPECT_NEAR(r.y, 0.070, 1e-6) << "rotor " << r.index;
        // the disc (what Physics takes) is the blade mid-plane at 0.25 to 0.35 R, 0.00926 below the glTF origin (Core, measured): 0.0607
        EXPECT_NEAR(r.y - 0.00926, 0.0607, 1e-4) << "rotor " << r.index;
    }
    EXPECT_NEAR(std::hypot(2 * 0.17689, 2 * 0.17689), 0.5003, 1e-4);
}

TEST(X500BuildTable, TheFrameIsTheKitsOwnCadAndTheMotorsAreTheKitsMotors) {
    const YAML::Node doc = YAML::LoadFile(kTable);
    int frames = 0;
    int motors = 0;
    std::set<std::string> motor_files;
    for (const YAML::Node& part : doc["parts"]) {
        const std::string kind = part["kind"].as<std::string>();
        const YAML::Node listed = part["objects"];
        if (listed) {
            for (const YAML::Node& object : listed) {
                const std::string file = object["store_ref"]["file"].as<std::string>();
                EXPECT_EQ(file.find("x500-body"), std::string::npos) << "the drawing-only PX4 mesh is retired: " << file;
                EXPECT_EQ(object["store_ref"]["sha256"].as<std::string>().size(), 64u) << file;
            }
        }
        if (kind != "frame" && kind != "motor") continue;
        // every kit object is posed by the one turn of the file (forward at its -X) to the design frame (forward at +X)
        const YAML::Node pose = part["pose"];
        for (int i = 0; i < 3; ++i) EXPECT_EQ(pose["position"][i].as<double>(), 0.0) << part["name"].as<std::string>();
        const double q[4] = {0.0, 0.0, 1.0, 0.0};
        for (int i = 0; i < 4; ++i) EXPECT_EQ(pose["orientation"][i].as<double>(), q[i]) << part["name"].as<std::string>();
        if (kind == "frame") {
            ++frames;
            EXPECT_EQ(part["objects"].size(), 61u) << "the kit's plates, arm tubes, mounts, clamps, base plates, landing gear, GPS stand and tray";
            for (const YAML::Node& object : part["objects"]) {
                EXPECT_TRUE(object["store_ref"]["file"].as<std::string>().ends_with(".spmesh"));
            }
        } else {
            ++motors;
            ASSERT_EQ(part["objects"].size(), 1u);
            motor_files.insert(part["objects"][0]["store_ref"]["file"].as<std::string>());
        }
    }
    EXPECT_EQ(frames, 1);
    EXPECT_EQ(motors, 4);
    EXPECT_EQ(motor_files.size(), 4u) << "four motors, four kit objects";
    for (const std::string& file : motor_files) EXPECT_EQ(file.rfind("DJ-2216-KV880_1_", 0), 0u) << file;
}

TEST(X500BuildTable, EachRotorHidesItsOwnBell) {
    const YAML::Node doc = YAML::LoadFile(kTable);
    // Rendering applies a top-level hide_materials to the frame only; a rotor entry's own list hides that rotor's bell (the glTF's 59 mm bell,
    // where the kit's 28.7 mm motor is drawn).
    EXPECT_FALSE(doc["hide_materials"]) << "the frame hides nothing: the PX4 mesh and its autopilot block are gone";
    int seen = 0;
    for (const YAML::Node& part : doc["parts"]) {
        if (part["kind"].as<std::string>() != "rotor") continue;
        const std::string file = part["objects"][0]["store_ref"]["file"].as<std::string>();
        const std::string want = "drone/" + file.substr(0, file.size() - std::string(".gltf").size()) + "/bell";
        const YAML::Node hide = part["hide_materials"];
        ASSERT_TRUE(hide.IsSequence()) << file;
        ASSERT_EQ(hide.size(), 1u) << file;
        EXPECT_EQ(hide[0].as<std::string>(), want);
        ++seen;
    }
    EXPECT_EQ(seen, 4);
}

TEST(X500BuildTable, EveryEntryCarriesItsMassAndItsGrades) {
    const YAML::Node doc = YAML::LoadFile(kTable);
    const std::set<std::string> mass_grades = {"measured", "published", "retailer", "derived", "assumed"};
    const std::set<std::string> pose_grades = {"measured", "derived", "assumed"};
    double sum = 0.0;
    for (const YAML::Node& part : doc["parts"]) {
        const std::string name = part["name"].as<std::string>();
        EXPECT_GT(part["mass_kg"].as<double>(), 0.0) << name;
        EXPECT_EQ(mass_grades.count(part["mass_grade"].as<std::string>()), 1u) << name;
        EXPECT_FALSE(part["mass_source"].as<std::string>().empty()) << name;
        EXPECT_EQ(pose_grades.count(part["pose_grade"].as<std::string>()), 1u) << name;
        sum += part["mass_kg"].as<double>();
    }
    EXPECT_NEAR(sum, 1.6326, 1e-4);
}

}  // namespace
