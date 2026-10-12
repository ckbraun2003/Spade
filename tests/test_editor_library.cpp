// The editor's library: the built-in items and placement by copy (editor plan
// Task 9; EDT-012).
//
// The built-in items ship with the editor: the quadrotor template, and the
// primitive assets (box, sphere, cylinder, capsule). Placing an item copies it
// into the scene: a model once per placed vehicle, under its own name, and an
// asset as an asset. A scene never refers to a library item.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_library.hpp"
#include "editor_test_support.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::Result;
using spade::SdfPose;

[[nodiscard]] const ed::LibraryItem& item_named(const std::vector<ed::LibraryItem>& lib, std::string_view name) {
    for (const ed::LibraryItem& i : lib) {
        if (i.name == name) return i;
    }
    ADD_FAILURE() << "no library item named " << name;
    return lib.front();
}

[[nodiscard]] SdfPose pose_at(glm::vec3 p, float yaw = 0.0f, float scale = 1.0f) {
    SdfPose at;
    at.position = p;
    at.rotation = glm::angleAxis(yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    at.scale = scale;
    return at;
}

[[nodiscard]] ed::SceneDocument hover_document() {
    return ed::make_scene_document(support::sample_scene(), {});
}

}  // namespace

TEST(EditorLibrary, TheBuiltInItemsAreTheQuadrotorAndFourPrimitives) {
    const std::vector<ed::LibraryItem> lib = ed::builtin_library();
    std::vector<std::string> names;
    for (const ed::LibraryItem& i : lib) names.push_back(i.name);
    EXPECT_EQ(names, (std::vector<std::string>{"quadrotor", "box", "sphere", "cylinder", "capsule"}));
    for (const ed::LibraryItem& i : lib) {
        if (i.kind == ed::LibraryItem::Kind::model) {
            ASSERT_TRUE(i.model.has_value()) << i.name;
            EXPECT_FALSE(i.asset.has_value()) << i.name;
            EXPECT_TRUE(i.model->validate().has_value()) << i.name;
        } else {
            ASSERT_TRUE(i.asset.has_value()) << i.name;
            EXPECT_FALSE(i.model.has_value()) << i.name;
        }
    }
}

TEST(EditorLibrary, PlacingAnAssetTwiceGivesTwoUniquelyNamedCopiesHoldingNoPath) {
    ed::SceneDocument doc = hover_document();
    const ed::LibraryItem& box = item_named(ed::builtin_library(), "box");
    const Result<std::string> first = ed::place(doc, box, pose_at(glm::vec3(1.0f, 0.5f, 0.0f)));
    const Result<std::string> second = ed::place(doc, box, pose_at(glm::vec3(-1.0f, 0.5f, 0.0f)));
    ASSERT_TRUE(first.has_value()) << first.error().context;
    ASSERT_TRUE(second.has_value()) << second.error().context;
    EXPECT_EQ(*first, "box");
    EXPECT_EQ(*second, "box_2");
    ASSERT_EQ(doc.desc().assets.size(), 2u);
    for (const spade::scene::SceneAsset& a : doc.desc().assets) {
        EXPECT_TRUE(a.visual.mesh_ref.empty()) << a.name;   // no file behind it
        EXPECT_FALSE(a.collider.nodes.empty()) << a.name;   // the shape itself, copied in
    }
    EXPECT_EQ(doc.desc().assets[1].pose.position, glm::vec3(-1.0f, 0.5f, 0.0f));
}

TEST(EditorLibrary, PlacingTheQuadrotorTwiceGivesTwoVehiclesAndTwoModels) {
    ed::SceneDocument doc = hover_document();
    const std::size_t vehicles = doc.desc().vehicles.size();
    const std::size_t models = doc.desc().models.size();
    const ed::LibraryItem& quad = item_named(ed::builtin_library(), "quadrotor");
    const Result<std::string> first = ed::place(doc, quad, pose_at(glm::vec3(2.0f, 0.0f, 0.0f)));
    const Result<std::string> second = ed::place(doc, quad, pose_at(glm::vec3(4.0f, 0.0f, 0.0f)));
    ASSERT_TRUE(first.has_value()) << first.error().context;
    ASSERT_TRUE(second.has_value()) << second.error().context;
    EXPECT_NE(*first, *second);
    ASSERT_EQ(doc.desc().vehicles.size(), vehicles + 2);
    ASSERT_EQ(doc.desc().models.size(), models + 2);
    const auto& a = doc.desc().vehicles[vehicles];
    const auto& b = doc.desc().vehicles[vehicles + 1];
    EXPECT_EQ(a.name, *first);
    EXPECT_EQ(b.name, *second);
    EXPECT_NE(a.model, b.model);  // a model once per placed vehicle (EDT-012)
}

TEST(EditorLibrary, APlacedModelEqualsTheLibrarysExceptForItsName) {
    ed::SceneDocument doc = hover_document();
    const ed::LibraryItem& quad = item_named(ed::builtin_library(), "quadrotor");
    ASSERT_TRUE(ed::place(doc, quad, pose_at(glm::vec3(2.0f, 0.0f, 0.0f))).has_value());
    ASSERT_TRUE(ed::place(doc, quad, pose_at(glm::vec3(4.0f, 0.0f, 0.0f))).has_value());
    for (std::size_t i = doc.desc().vehicles.size() - 2; i < doc.desc().vehicles.size(); ++i) {
        spade::vehicles::ModelType placed = support::model_named(doc.desc(), doc.desc().vehicles[i].model);
        spade::vehicles::ModelType library = *quad.model;
        placed.name = "same";
        library.name = "same";
        EXPECT_EQ(support::text_of(support::scene_with(placed)), support::text_of(support::scene_with(library)));
    }
}

TEST(EditorLibrary, APlacedVehicleStartsAtThePose) {
    ed::SceneDocument doc = hover_document();
    const SdfPose at = pose_at(glm::vec3(3.0f, 0.25f, -2.0f), 0.7f);
    const Result<std::string> name = ed::place(doc, item_named(ed::builtin_library(), "quadrotor"), at);
    ASSERT_TRUE(name.has_value()) << name.error().context;
    const spade::scene::SceneVehicle& v = doc.desc().vehicles.back();
    EXPECT_EQ(v.start.pos, at.position);
    EXPECT_EQ(v.start.orient, at.rotation);
    EXPECT_EQ(v.start.vel, glm::vec3(0.0f));
}

// A vehicle has no scale; one is refused rather than dropped (L6).
TEST(EditorLibrary, AScaledVehicleIsRefusedAndNothingIsPlaced) {
    ed::SceneDocument doc = hover_document();
    const Result<std::string> r =
        ed::place(doc, item_named(ed::builtin_library(), "quadrotor"), pose_at(glm::vec3(1.0f, 0.0f, 0.0f), 0.0f, 2.0f));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("scale"), std::string::npos) << r.error().context;
    EXPECT_FALSE(doc.can_undo());
}

TEST(EditorLibrary, APlacementIsOneUndo) {
    ed::SceneDocument doc = hover_document();
    const std::string before = support::text_of(doc.desc());
    ASSERT_TRUE(ed::place(doc, item_named(ed::builtin_library(), "quadrotor"), pose_at(glm::vec3(2.0f, 0, 0))).has_value());
    ASSERT_TRUE(doc.undo());
    EXPECT_EQ(support::text_of(doc.desc()), before);  // the vehicle and its model, both gone
    EXPECT_FALSE(doc.can_undo());
}

// The kind comes from the model, so a new kind needs no change to the panel
// (editor spec §4). Drones are the only kind today.
TEST(EditorLibrary, TheTemplatesVehicleKindIsQuadrotor) {
    const spade::vehicles::ModelType& quad = *item_named(ed::builtin_library(), "quadrotor").model;
    EXPECT_EQ(ed::vehicle_kind_of(quad), "quadrotor");
    spade::vehicles::ModelType six = quad;
    six.rotors.push_back(six.rotors.front());
    six.rotors.push_back(six.rotors.front());
    EXPECT_EQ(ed::vehicle_kind_of(six), "multirotor");
}
