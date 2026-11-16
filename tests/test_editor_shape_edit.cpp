// L82, the editor half: the scene edit that sets or clears a model's contact shape (SetModelShape) and its text form.
//
// A shape lives on the MODEL (ModelType::shape, the lead's ruling Q1/Q4, L82a). The edit reaches the model the named
// vehicle flies, copying a model another vehicle shares first (EDT-019, as every model parameter does). A shape the engine
// or the editor refuses leaves the document as it was and says why (EDT-009).

#include <gtest/gtest.h>

#include <optional>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_edit_text.hpp"
#include "../sandbox/editor_files.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "../sandbox/editor_shapes.hpp"
#include "editor_test_support.hpp"
#include "physics/rigid_contact.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::physics::ShapeDesc;
using spade::physics::ShapeKind;

[[nodiscard]] ed::SceneDocument doc_of(spade::scene::SceneDesc s) { return ed::make_scene_document(std::move(s), {}); }

[[nodiscard]] const spade::vehicles::ModelType* model_of(const spade::scene::SceneDesc& s, const std::string& vehicle) {
    for (const auto& v : s.vehicles) {
        if (v.name != vehicle) continue;
        for (const auto& m : s.models) {
            if (m.name == v.model) return &m;
        }
    }
    return nullptr;
}

TEST(EditorShapeEdit, SettingAShapePutsItOnTheVehiclesModel) {
    auto d = doc_of(support::sample_scene());
    ASSERT_FALSE(model_of(d.desc(), "hover_quad_0")->shape.has_value());
    ASSERT_TRUE(ed::apply(d, ed::SetModelShape{"hover_quad_0", spade::physics::sphere_shape(0.1f)}).has_value());
    const auto* m = model_of(d.desc(), "hover_quad_0");
    ASSERT_TRUE(m->shape.has_value());
    EXPECT_EQ(m->shape->kind, ShapeKind::sphere);
    EXPECT_EQ(m->shape->radius, 0.1f);
}

TEST(EditorShapeEdit, TheSameShapeAgainCommitsNothingAndClearingRemovesIt) {
    auto d = doc_of(support::sample_scene());
    ShapeDesc box = spade::physics::box_shape(glm::vec3(0.1f, 0.05f, 0.1f));
    box.friction = 0.4f;
    ASSERT_TRUE(ed::apply(d, ed::SetModelShape{"hover_quad_0", box}).has_value());
    const uint64_t serial = d.serial();
    ASSERT_TRUE(ed::apply(d, ed::SetModelShape{"hover_quad_0", box}).has_value());
    EXPECT_EQ(d.serial(), serial) << "no change, no commit";

    ASSERT_TRUE(ed::apply(d, ed::SetModelShape{"hover_quad_0", std::nullopt}).has_value());
    EXPECT_FALSE(model_of(d.desc(), "hover_quad_0")->shape.has_value());
    EXPECT_GT(d.serial(), serial);
    // Clearing what is not there commits nothing either.
    const uint64_t cleared = d.serial();
    ASSERT_TRUE(ed::apply(d, ed::SetModelShape{"hover_quad_0", std::nullopt}).has_value());
    EXPECT_EQ(d.serial(), cleared);
}

TEST(EditorShapeEdit, AShapeEditIsOneUndoStep) {
    auto d = doc_of(support::sample_scene());
    ASSERT_TRUE(ed::apply(d, ed::SetModelShape{"hover_quad_0", spade::physics::sphere_shape(0.1f)}).has_value());
    ASSERT_TRUE(d.undo());
    EXPECT_FALSE(model_of(d.desc(), "hover_quad_0")->shape.has_value());
    ASSERT_TRUE(d.redo());
    EXPECT_TRUE(model_of(d.desc(), "hover_quad_0")->shape.has_value());
}

TEST(EditorShapeEdit, ASharedModelIsCopiedForTheEditedVehicle) {
    spade::scene::SceneDesc s = support::sample_scene();
    spade::scene::SceneVehicle twin = s.vehicles.at(0);
    twin.name = "twin";
    s.vehicles.push_back(twin);  // two vehicles, one model
    auto d = doc_of(std::move(s));
    const std::string shared = model_of(d.desc(), "hover_quad_0")->name;
    ASSERT_EQ(model_of(d.desc(), "twin")->name, shared);

    ASSERT_TRUE(ed::apply(d, ed::SetModelShape{"twin", spade::physics::sphere_shape(0.1f)}).has_value());
    EXPECT_NE(model_of(d.desc(), "twin")->name, shared) << "the twin flies its own copy now";
    EXPECT_TRUE(model_of(d.desc(), "twin")->shape.has_value());
    EXPECT_EQ(model_of(d.desc(), "hover_quad_0")->name, shared);
    EXPECT_FALSE(model_of(d.desc(), "hover_quad_0")->shape.has_value()) << "the other vehicle keeps its model";
}

TEST(EditorShapeEdit, ARefusedShapeLeavesTheDocumentAndSaysWhy) {
    auto d = doc_of(support::sample_scene());
    const uint64_t serial = d.serial();

    const auto zero = ed::apply(d, ed::SetModelShape{"hover_quad_0", spade::physics::sphere_shape(0.0f)});
    ASSERT_FALSE(zero.has_value());
    EXPECT_NE(zero.error().context.find("set_body_shape:"), std::string::npos) << zero.error().context;

    const auto nobody = ed::apply(d, ed::SetModelShape{"ghost", spade::physics::sphere_shape(0.1f)});
    ASSERT_FALSE(nobody.has_value());
    EXPECT_NE(nobody.error().context.find("ghost"), std::string::npos) << nobody.error().context;

    // Q7: a box on a model whose design frame is turned 45 degrees.
    ASSERT_TRUE(ed::apply(d, ed::SetModelParam{"hover_quad_0", spade::vehicles::ModelIssue::Element::model, 0,
                                               "design_to_principal",
                                               glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f))})
                    .has_value());
    const uint64_t turned = d.serial();
    ASSERT_GT(turned, serial);
    const auto box = ed::apply(d, ed::SetModelShape{"hover_quad_0", spade::physics::box_shape(glm::vec3(0.1f))});
    ASSERT_FALSE(box.has_value());
    EXPECT_NE(box.error().context.find("design_to_principal"), std::string::npos) << box.error().context;
    EXPECT_EQ(d.serial(), turned);
    EXPECT_FALSE(model_of(d.desc(), "hover_quad_0")->shape.has_value());
}

// ---- the text form --------------------------------------------------------------------------------------------------

[[nodiscard]] ed::SetModelShape round_trip(const ed::SetModelShape& e) {
    const spade::Result<std::string> text = ed::format_edit(e);
    EXPECT_TRUE(text.has_value()) << (text ? "" : text.error().context);
    if (!text) return {};
    const spade::Result<ed::SceneEdit> back = ed::parse_edit(*text);
    EXPECT_TRUE(back.has_value()) << *text << ": " << (back ? "" : back.error().context);
    if (!back || !std::holds_alternative<ed::SetModelShape>(*back)) return {};
    return std::get<ed::SetModelShape>(*back);
}

TEST(EditorShapeEdit, EveryShapeHasATextFormThatReadsBackBitForBit) {
    ShapeDesc capsule = spade::physics::capsule_shape(glm::vec3(-0.3f, 0.0f, 0.1f), glm::vec3(0.3f, 0.0f, 0.1f), 0.05f);
    capsule.friction = 0.3f;
    capsule.restitution = 0.1f;
    capsule.iterations = 4;
    ShapeDesc box = spade::physics::box_shape(glm::vec3(0.1f, 0.2f, 0.3f), 0.002f);
    box.rest_speed = 0.05f;
    ShapeDesc hull = spade::physics::hull_shape({glm::vec3(0.1f, 0.0f, 0.0f), glm::vec3(-0.1f, 0.0f, 0.0f),
                                                 glm::vec3(0.0f, 0.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.1f)});
    for (const std::optional<ShapeDesc>& shape : {std::optional<ShapeDesc>{}, std::optional<ShapeDesc>{spade::physics::sphere_shape(0.25f)},
                                                  std::optional<ShapeDesc>{capsule}, std::optional<ShapeDesc>{box},
                                                  std::optional<ShapeDesc>{hull}}) {
        const ed::SetModelShape in{"hover_quad_0", shape};
        const ed::SetModelShape out = round_trip(in);
        EXPECT_EQ(out.vehicle, "hover_quad_0");
        EXPECT_TRUE(ed::same_shape(out.shape, in.shape)) << (shape ? "kind " + std::to_string(static_cast<int>(shape->kind)) : "none");
    }
}

TEST(EditorShapeEdit, TheTextFormIsReadableAndTheWorldsMaterialIsTheWordWorld) {
    const spade::Result<std::string> text =
        ed::format_edit(ed::SetModelShape{"hover_quad_0", spade::physics::sphere_shape(0.25f)});
    ASSERT_TRUE(text.has_value());
    EXPECT_EQ(*text, "scene set-model-shape hover_quad_0 sphere radius 0.25 friction world restitution world rest-speed 0 iterations 0");
    const spade::Result<std::string> none = ed::format_edit(ed::SetModelShape{"hover_quad_0", std::nullopt});
    ASSERT_TRUE(none.has_value());
    EXPECT_EQ(*none, "scene set-model-shape hover_quad_0 none");
}

TEST(EditorShapeEdit, ABadLineIsRefusedWithAPlaceInIt) {
    const spade::Result<ed::SceneEdit> bad = ed::parse_edit("scene set-model-shape hover_quad_0 cone radius 1");
    ASSERT_FALSE(bad.has_value());
    EXPECT_NE(bad.error().context.find("cone"), std::string::npos) << bad.error().context;
}

}  // namespace
