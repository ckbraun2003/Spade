// ---------------------------------------------------------------------------
// test_openframe5f_build_table.cpp -- the OpenFrame-5F 5 inch kit's pose table (tools/flow_movie/data/openframe5f_build/parts.yaml), on the maker's own
// STEP files (record docs/design/interface/records/2026-11-21-fpv-kit-sources.md, sections 5 and 6), in the form of the X500's (test_x500_build_table.cpp).
//
//   OpenFrame5fBuildTable.TheTableLoadsThroughTheBuildLoader   the flow_movie loader reads it: unique names, known kinds, store refs, materials, poses
//   OpenFrame5fBuildTable.TheFrameIsTurnedToTheDesignFrameByOneProperRotation   the maker's part-studio frame (x across, y along with forward at -y, z up)
//                                                to the design frame (+X forward, +Y up, +Z right): (X, Y, Z) = (-y, z + 3 mm, -x), quaternion
//                                                [0.5, -0.5, 0.5, 0.5], determinant +1; the one pose every frame object shares
//   OpenFrame5fBuildTable.EveryObjectOffsetIsAProperTurnAboutTheMakersZ   no mirrored placement: an offset is the identity, a half turn about z, or a
//                                                translation (the converter refuses mirrored axes, so the other hand of an arm is its own mesh)
//   OpenFrame5fBuildTable.TheFourArmsAreTwoHandsInPlace   two arm meshes (the exported one and its mirror image), each at its place and its half turn:
//                                                the exported arm's quadrant at (+x, +y) and the mirror's at (+x, -y); no rotation of the exported arm fits
//                                                the (+x, -y) quadrant (a boolean check found 64 mm3 of overlap with the cross)
//   OpenFrame5fBuildTable.TheFourStandoffsSitOnTheMiddlePlatesHoles   the one exported M3 standoff is copied to the four holes of the middle plate:
//                                                (+-12.75, 24.0) and (+-10.25, 69.5) mm
//   OpenFrame5fBuildTable.TheStackSitsOnTheMakersHoles   the FC and the ESC (the maker's 30x30 boards) are centred on the 30.5 mm stack holes at the
//                                                studio's (0, -12.75) mm = X +0.01275, Z 0, their sizes the maker's, their heights assumed
//   OpenFrame5fBuildTable.TheMotorsAndTheRotorsAreOnTheArmsBores   four motors (retailer-dimension stand-ins, 32 g assumed) and four rotors at the
//                                                measured bores (+-0.069534, +-0.089000) m, the library fpv_5inch rotor order and spins (rotor 0
//                                                front-right, spin +1 takes prop_ccw), radius 0.0635 m, the prop stand-ins the library's own meshes by file hash
//   OpenFrame5fBuildTable.ThePackIsAnAssumedTypicalSixS   one 6S 1300 mAh pack, 200 g, 78 x 39 x 38 mm, graded assumed, on the top plate
//   OpenFrame5fBuildTable.EveryEntryCarriesItsMassAndItsGrades   mass_kg, mass_grade, mass_source and pose_grade on every entry; the frame's 0.170 kg is
//                                                the maker's, the printed parts' mass assumed; the kit sums to 0.5657 kg
//   OpenFrame5fBuildTable.TheStoreRefsResolveAndTheArmsAreTheHandsTheyClaim   OPT-IN (SPADE_PART_STORE): every store_ref is a file in the store, and the
//                                                exported arm's centroid is at +y, its mirror's at -y with the same |x|
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <yaml-cpp/yaml.h>

#include "../tools/flow_movie/build_parts.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "world/sha256.hpp"

namespace {

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
const char* env_text(const char* name) {
    return std::getenv(name);
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

const std::string kTable = std::string(SPADE_REPO_DIR) + "/tools/flow_movie/data/openframe5f_build/parts.yaml";

const char* kArmFile = "Arm.spmesh";
const char* kMirroredArmFile = "Open CASCADE STEP translator 7.9 1.spmesh";  // the mirror image of the arm, written by OCP and so named by it
const char* kStandoffFile = "Standoff M3 F-F 20 OD5.spmesh";

YAML::Node frame_entry(const YAML::Node& doc) {
    for (const YAML::Node& part : doc["parts"]) {
        if (part["name"].as<std::string>() == "openframe_frame") return part;
    }
    return YAML::Node();
}

glm::dquat quat_of(const YAML::Node& n) {
    return glm::normalize(glm::dquat(n[0].as<double>(), n[1].as<double>(), n[2].as<double>(), n[3].as<double>()));
}

glm::dvec3 vec_of(const YAML::Node& n) { return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()}; }

TEST(OpenFrame5fBuildTable, TheTableLoadsThroughTheBuildLoader) {
    std::ifstream in(kTable, std::ios::binary);
    ASSERT_TRUE(in.good()) << kTable;
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto table = flow_movie::parse_build_parts(text);
    ASSERT_TRUE(table.has_value()) << table.error().context;
    ASSERT_FALSE(table->empty());
    std::set<std::string> names;
    for (const auto& part : *table) EXPECT_TRUE(names.insert(part.name).second) << "a name is unique: " << part.name;
    EXPECT_EQ(names.count("openframe_frame"), 1u);
}

TEST(OpenFrame5fBuildTable, TheFrameIsTurnedToTheDesignFrameByOneProperRotation) {
    const YAML::Node frame = frame_entry(YAML::LoadFile(kTable));
    ASSERT_TRUE(frame.IsDefined() && frame.IsMap()) << "an entry named openframe_frame";
    EXPECT_EQ(frame["kind"].as<std::string>(), "frame");
    const glm::dquat q = quat_of(frame["pose"]["orientation"]);
    const glm::dvec3 pos = vec_of(frame["pose"]["position"]);
    // (X, Y, Z) = (-y, z, -x): the studio's x axis lands on -Z, y on -X, z on +Y
    const glm::dvec3 ex = q * glm::dvec3(1, 0, 0);
    const glm::dvec3 ey = q * glm::dvec3(0, 1, 0);
    const glm::dvec3 ez = q * glm::dvec3(0, 0, 1);
    EXPECT_NEAR(glm::length(ex - glm::dvec3(0, 0, -1)), 0.0, 1e-9);
    EXPECT_NEAR(glm::length(ey - glm::dvec3(-1, 0, 0)), 0.0, 1e-9);
    EXPECT_NEAR(glm::length(ez - glm::dvec3(0, 1, 0)), 0.0, 1e-9);
    EXPECT_NEAR(glm::dot(glm::cross(ex, ey), ez), 1.0, 1e-9) << "a proper rotation";
    // the origin is the lower face of the bottom plate, which is 3 mm below the studio's z = 0
    EXPECT_NEAR(pos.x, 0.0, 1e-12);
    EXPECT_NEAR(pos.y, 0.003, 1e-12);
    EXPECT_NEAR(pos.z, 0.0, 1e-12);
    // the front motor of the (+x, -y) arm, at the studio's (89.000, -69.534) mm and mid height of the 6 mm arm, lands at the front and the starboard-negative (left) side of the design frame
    const glm::dvec3 motor = pos + q * glm::dvec3(0.089000, -0.069534, 0.003);
    EXPECT_NEAR(motor.x, 0.069534, 1e-6) << "forward is the studio's -y";
    EXPECT_NEAR(motor.y, 0.006, 1e-9);
    EXPECT_NEAR(motor.z, -0.089000, 1e-6);
}

TEST(OpenFrame5fBuildTable, EveryObjectOffsetIsAProperTurnAboutTheMakersZ) {
    const YAML::Node doc = YAML::LoadFile(kTable);
    int checked = 0;
    for (const YAML::Node& part : doc["parts"]) {
        if (!part["objects"]) continue;
        for (const YAML::Node& object : part["objects"]) {
            if (!object["offset"]) continue;
            const glm::dquat q = quat_of(object["offset"]["orientation"]);
            const bool identity = std::abs(std::abs(q.w) - 1.0) < 1e-9;
            const bool half_turn_z = std::abs(std::abs(q.z) - 1.0) < 1e-9;
            EXPECT_TRUE(identity || half_turn_z) << part["name"].as<std::string>() << ": an offset is the identity or a half turn about z";
            ++checked;
        }
    }
    EXPECT_GT(checked, 0);
}

TEST(OpenFrame5fBuildTable, TheFourArmsAreTwoHandsInPlace) {
    const YAML::Node frame = frame_entry(YAML::LoadFile(kTable));
    ASSERT_TRUE(frame.IsMap());
    int arm = 0, mirrored = 0;
    int arm_turned = 0, mirrored_turned = 0;
    for (const YAML::Node& object : frame["objects"]) {
        const std::string file = object["store_ref"]["file"].as<std::string>();
        const bool turned = object["offset"] && std::abs(quat_of(object["offset"]["orientation"]).z) > 0.5;
        if (file == kArmFile) {
            ++arm;
            arm_turned += turned ? 1 : 0;
        } else if (file == kMirroredArmFile) {
            ++mirrored;
            mirrored_turned += turned ? 1 : 0;
        }
    }
    EXPECT_EQ(arm, 2) << "the exported arm, and its half turn (the (-x, -y) quadrant)";
    EXPECT_EQ(mirrored, 2) << "the mirror image, and its half turn (the (+x, -y) and (-x, +y) quadrants)";
    EXPECT_EQ(arm_turned, 1);
    EXPECT_EQ(mirrored_turned, 1);
}

TEST(OpenFrame5fBuildTable, TheFourStandoffsSitOnTheMiddlePlatesHoles) {
    const YAML::Node frame = frame_entry(YAML::LoadFile(kTable));
    ASSERT_TRUE(frame.IsMap());
    // the exported standoff is at (-10.25, 69.5) mm in the maker's frame; its copies go to the plate's other three holes
    const glm::dvec2 exported(-0.01025, 0.0695);
    std::set<std::pair<long, long>> at;
    for (const YAML::Node& object : frame["objects"]) {
        if (object["store_ref"]["file"].as<std::string>() != kStandoffFile) continue;
        const glm::dvec3 d = object["offset"] ? vec_of(object["offset"]["position"]) : glm::dvec3(0.0);
        EXPECT_NEAR(d.z, 0.0, 1e-12);
        at.insert({std::lround((exported.x + d.x) * 1e5), std::lround((exported.y + d.y) * 1e5)});  // 0.01 mm
    }
    const std::set<std::pair<long, long>> want = {{-1025, 6950}, {1025, 6950}, {-1275, 2400}, {1275, 2400}};
    EXPECT_EQ(at, want);
}

TEST(OpenFrame5fBuildTable, EveryEntryCarriesItsMassAndItsGrades) {
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
    const YAML::Node frame = frame_entry(doc);
    ASSERT_TRUE(frame.IsMap());
    EXPECT_NEAR(frame["mass_kg"].as<double>(), 0.170, 1e-12) << "the maker's 170 g (carbon, aluminium and hardware, without the printed parts)";
    EXPECT_EQ(frame["mass_grade"].as<std::string>(), "published") << "the README says measured, the product page says est.: not graded measured by Spade";
    EXPECT_NEAR(sum, 0.5657, 1e-4) << "frame 170 g, printed parts 29.7 g, FC 7, ESC 15, RX 0.8, four motors 128, pack 200, four props 15.2: 565.7 g, no camera, VTX or wiring";
}

TEST(OpenFrame5fBuildTable, TheStackSitsOnTheMakersHoles) {
    const YAML::Node doc = YAML::LoadFile(kTable);
    int seen = 0;
    for (const YAML::Node& part : doc["parts"]) {
        const std::string name = part["name"].as<std::string>();
        if (name != "openfc_lite_30x30" && name != "openesc_30x30") continue;
        ++seen;
        const glm::dvec3 at = vec_of(part["pose"]["position"]);
        EXPECT_NEAR(at.x, 0.01275, 1e-9) << name << ": the stack holes are about the studio's (0, -12.75) mm, forward -y";
        EXPECT_NEAR(at.z, 0.0, 1e-9) << name;
        EXPECT_EQ(part["mass_grade"].as<std::string>(), "published") << name << ": the maker's estimate, marked as it says";
        EXPECT_TRUE(part["mass_source"].as<std::string>().find("est.") != std::string::npos) << name;
        const glm::dvec3 box = vec_of(part["box"]);
        if (name == "openfc_lite_30x30") {
            EXPECT_NEAR(part["mass_kg"].as<double>(), 0.007, 1e-12);
            EXPECT_NEAR(box.x, 0.0379, 1e-9);
            EXPECT_NEAR(box.z, 0.0379, 1e-9);
        } else {
            EXPECT_NEAR(part["mass_kg"].as<double>(), 0.015, 1e-12);
            EXPECT_NEAR(box.x, 0.0425, 1e-9);
            EXPECT_NEAR(box.z, 0.0416, 1e-9);
        }
        EXPECT_GT(at.y, 0.012) << name << ": above the middle plate's top face (design Y 0.012 m)";
        EXPECT_LT(at.y + 0.5 * box.y, 0.0320) << name << ": under the top plate's lower face (design Y 0.032 m)";
    }
    EXPECT_EQ(seen, 2);
}

TEST(OpenFrame5fBuildTable, TheMotorsAndTheRotorsAreOnTheArmsBores) {
    const YAML::Node doc = YAML::LoadFile(kTable);
    int motors = 0, rotors = 0;
    std::set<int> seen_rotor;
    for (const YAML::Node& part : doc["parts"]) {
        const std::string kind = part["kind"].as<std::string>();
        if (kind != "motor" && kind != "rotor") continue;
        const std::string name = part["name"].as<std::string>();
        const glm::dvec3 at = vec_of(part["pose"]["position"]);
        EXPECT_NEAR(std::abs(at.x), 0.069534, 1e-6) << name << ": the bore's 69.534 mm along";
        EXPECT_NEAR(std::abs(at.z), 0.089000, 1e-6) << name << ": the bore's 89.000 mm across";
        if (kind == "motor") {
            ++motors;
            EXPECT_NEAR(part["mass_kg"].as<double>(), 0.032, 1e-12) << name;
            EXPECT_EQ(part["mass_grade"].as<std::string>(), "assumed") << name << ": no maker mass for the 2306 is published";
            EXPECT_NEAR(part["cylinder"]["radius"].as<double>(), 0.01465, 1e-9) << name << ": phi 29.3 mm (a retailer's figure)";
            EXPECT_NEAR(part["cylinder"]["length"].as<double>(), 0.0306, 1e-9) << name << ": 30.6 mm (a retailer's figure)";
            EXPECT_NEAR(at.y, 0.009 + 0.0153, 1e-9) << name << ": standing on the arm's top face (design Y 0.009 m)";
            continue;
        }
        ++rotors;
        const int index = part["rotor"].as<int>();
        seen_rotor.insert(index);
        const int spin = part["spin"].as<int>();
        // the library's fpv_5inch order: 0 front-right (+X, +Z) spin +1, 1 front-left (+X, -Z) -1, 2 rear-left (-X, -Z) +1, 3 rear-right (-X, +Z) -1
        const int want_spin[4] = {1, -1, 1, -1};
        const double want_x[4] = {1, 1, -1, -1};
        const double want_z[4] = {1, -1, -1, 1};
        ASSERT_TRUE(index >= 0 && index < 4) << name;
        EXPECT_EQ(spin, want_spin[index]) << name;
        EXPECT_GT(at.x * want_x[index], 0.0) << name;
        EXPECT_GT(at.z * want_z[index], 0.0) << name;
        EXPECT_NEAR(part["radius"].as<double>(), 0.0635, 1e-12) << name;
        EXPECT_NEAR(at.y, 0.0436, 1e-9) << name << ": the disc plane, 4 mm above the motor's top (assumed: the hub is not published)";
        ASSERT_EQ(part["objects"].size(), 1u) << name;
        const std::string file = part["objects"][0]["store_ref"]["file"].as<std::string>();
        EXPECT_EQ(file, spin == 1 ? "prop_ccw.spmesh" : "prop_cw.spmesh") << name << ": spin +1 is counter-clockwise from above and takes the ccw file";
        EXPECT_NEAR(part["mass_kg"].as<double>(), 0.0038, 1e-12) << name;
        EXPECT_EQ(part["mass_grade"].as<std::string>(), "retailer") << name << ": 3.8 g, the retailers' figure for the HQProp V2S";
        // the part store's entry is the library's own mesh, byte for byte
        const std::filesystem::path lib = std::filesystem::path(SPADE_REPO_DIR) / "assets/library/drones/fpv_5inch" / file;
        std::ifstream in(lib, std::ios::binary);
        ASSERT_TRUE(in.good()) << lib.string();
        const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        EXPECT_EQ(part["objects"][0]["store_ref"]["sha256"].as<std::string>(), spade::sha256_hex(bytes)) << name << ": the file hash of " << file;
    }
    EXPECT_EQ(motors, 4);
    EXPECT_EQ(rotors, 4);
    EXPECT_EQ(seen_rotor, (std::set<int>{0, 1, 2, 3}));
}

TEST(OpenFrame5fBuildTable, ThePackIsAnAssumedTypicalSixS) {
    const YAML::Node doc = YAML::LoadFile(kTable);
    int seen = 0;
    for (const YAML::Node& part : doc["parts"]) {
        if (part["kind"].as<std::string>() != "battery") continue;
        ++seen;
        EXPECT_NEAR(part["mass_kg"].as<double>(), 0.200, 1e-12);
        EXPECT_EQ(part["mass_grade"].as<std::string>(), "assumed");
        EXPECT_EQ(part["pose_grade"].as<std::string>(), "assumed");
        const glm::dvec3 box = vec_of(part["box"]);
        EXPECT_NEAR(box.x, 0.078, 1e-9) << "78 mm long, along X";
        EXPECT_NEAR(box.y, 0.038, 1e-9) << "38 mm tall";
        EXPECT_NEAR(box.z, 0.039, 1e-9) << "39 mm across";
        const glm::dvec3 at = vec_of(part["pose"]["position"]);
        EXPECT_NEAR(at.y - 0.5 * box.y, 0.0345, 1e-9) << "its underside on the top plate's top face (design Y 0.0345 m)";
    }
    EXPECT_EQ(seen, 1);
}

TEST(OpenFrame5fBuildTable, TheStoreRefsResolveAndTheArmsAreTheHandsTheyClaim) {
    const char* store_text = env_text("SPADE_PART_STORE");
    if (store_text == nullptr) GTEST_SKIP() << "set SPADE_PART_STORE=<the shared part store> to check the table's store refs against the meshes (the CAD stays local, L88b)";
    const std::filesystem::path store(store_text);
    const YAML::Node doc = YAML::LoadFile(kTable);
    glm::dvec3 arm_centroid(0.0), mirrored_centroid(0.0);
    for (const YAML::Node& part : doc["parts"]) {
        if (!part["objects"]) continue;
        for (const YAML::Node& object : part["objects"]) {
            const std::string sha = object["store_ref"]["sha256"].as<std::string>();
            const std::string file = object["store_ref"]["file"].as<std::string>();
            const std::filesystem::path path = store / sha / file;
            ASSERT_TRUE(std::filesystem::is_regular_file(path)) << path.string();
            if (file == kArmFile || file == kMirroredArmFile) {
                const auto loaded = spade::load_mesh_file(path);
                ASSERT_TRUE(loaded.has_value()) << path.string();
                (file == kArmFile ? arm_centroid : mirrored_centroid) = loaded->mesh->mass_properties().centroid;
            }
        }
    }
    EXPECT_GT(arm_centroid.y, 0.0) << "the exported arm is in the (+x, +y) quadrant";
    EXPECT_LT(mirrored_centroid.y, 0.0) << "its mirror image is across y = 0";
    EXPECT_NEAR(mirrored_centroid.x, arm_centroid.x, 1e-6);
    EXPECT_NEAR(mirrored_centroid.y, -arm_centroid.y, 1e-6);
}

}  // namespace
