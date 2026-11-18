// ---------------------------------------------------------------------------
// test_x500_build_table.cpp -- L88l: the X500 build's pose table (tools/flow_movie/data/x500_build/parts.yaml) against the engine's
// rotor convention and Physics' x500 model (tools/flow_movie/data/x500/x500.model.yaml).
//
//   X500BuildTable.ARotorTakesTheFileWhoseBladesLiftTurningTheEnginesWay   spin_dir +1 is counter-clockwise seen from above
//                                                (vehicles/rotor.hpp, consumers.md L7a): a +1 rotor takes the *-ccw file, a -1 rotor
//                                                the *-cw file; a front file sits at +X, a rear file at -X
//   X500BuildTable.TheRotorsAreTheModelsRotors   the four rotors are the model's, in its order: the hub less the model's
//                                                com_offset (the model is in the body frame; to 1e-5) and the spin_dir
//   X500BuildTable.TheFrameHidesItsBuiltInAutopilotThroughOneTopLevelKey   Rendering reads `hide_materials` at the top level
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

TEST(X500BuildTable, TheRotorsAreTheModelsRotors) {
    const std::vector<Rotor> rotors = table_rotors();
    const YAML::Node whole = YAML::LoadFile(kModel)["model"];
    const YAML::Node model = whole["rotors"];
    // The model's rotor local_pos is in its body frame, whose origin is the centre of mass; the table's hubs are in the design frame, so
    // local_pos = hub - com_offset (design_to_principal is the identity).
    const YAML::Node com = whole["com_offset"];
    ASSERT_EQ(com.size(), 3u);
    ASSERT_EQ(whole["design_to_principal"][0].as<double>(), 1.0);
    ASSERT_EQ(model.size(), 4u);
    ASSERT_EQ(rotors.size(), 4u);
    for (const Rotor& r : rotors) {
        const YAML::Node m = model[static_cast<std::size_t>(r.index)];
        const YAML::Node pos = m["local_pos"];
        EXPECT_NEAR(r.x - com[0].as<double>(), pos[0].as<double>(), 1e-5) << "rotor " << r.index;
        EXPECT_NEAR(r.y - com[1].as<double>(), pos[1].as<double>(), 1e-5) << "rotor " << r.index;
        EXPECT_NEAR(r.z - com[2].as<double>(), pos[2].as<double>(), 1e-5) << "rotor " << r.index;
        EXPECT_EQ(r.spin, m["spin_dir"].as<int>()) << "rotor " << r.index;
    }
}

TEST(X500BuildTable, TheFrameHidesItsBuiltInAutopilotThroughOneTopLevelKey) {
    const YAML::Node doc = YAML::LoadFile(kTable);
    ASSERT_TRUE(doc["hide_materials"].IsSequence());
    ASSERT_EQ(doc["hide_materials"].size(), 1u);
    EXPECT_EQ(doc["hide_materials"][0].as<std::string>(), "drone/x500-body/autopilot");
}

}  // namespace
