// L82, the editor half: the scene edit that sets or clears a model's contact shape (SetModelShape) and its text form.
//
// A shape lives on the MODEL (ModelType::shape, the lead's ruling Q1/Q4, L82a). The edit reaches the model the named
// vehicle flies, copying a model another vehicle shares first (EDT-019, as every model parameter does). A shape the engine
// or the editor refuses leaves the document as it was and says why (EDT-009).

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_edit_text.hpp"
#include "../sandbox/editor_files.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "../sandbox/editor_shapes.hpp"
#include "../sandbox/editor_workspace.hpp"
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

// ---- the file and the Run (Core's half: scene version 6, instantiate's module set) --------------------------------------

namespace fs = std::filesystem;

[[nodiscard]] std::string read_text(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

[[nodiscard]] std::string version_line(const std::string& yaml) {
    const std::size_t at = yaml.find("scene_version:");
    return at == std::string::npos ? std::string() : yaml.substr(at, yaml.find_first_of("\r\n", at) - at);
}

TEST(EditorShapeEdit, AShapeSurvivesSavingAndReopeningTheScene) {
    const support::TempDir tmp;
    const fs::path copy = support::mirror({fs::path(SPADE_ASSETS_DIR), support::hover_scene()}, tmp.path());
    const std::string before = version_line(read_text(copy));
    ASSERT_FALSE(before.empty());
    spade::Result<ed::OpenedScene> opened = ed::open_scene(copy);
    ASSERT_TRUE(opened.has_value());

    ShapeDesc hull = spade::physics::hull_shape({glm::vec3(0.1f, 0.0f, 0.0f), glm::vec3(-0.1f, 0.0f, 0.0f),
                                                 glm::vec3(0.0f, 0.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.1f)},
                                                0.002f);
    hull.friction = 0.4f;
    hull.iterations = 3;
    ASSERT_TRUE(ed::apply(opened->scene, ed::SetModelShape{"hover_quad_0", hull}).has_value());
    ASSERT_TRUE(ed::save_scene(opened->scene, opened->world).has_value());
    const std::string saved = read_text(copy);
    EXPECT_EQ(version_line(saved), "scene_version: 6") << "a shape is what makes it version 6";
    EXPECT_NE(saved.find("collision:"), std::string::npos);

    spade::Result<ed::OpenedScene> again = ed::open_scene(copy);
    ASSERT_TRUE(again.has_value()) << (again ? "" : again.error().context);
    const auto* m = model_of(again->scene.desc(), "hover_quad_0");
    ASSERT_NE(m, nullptr);
    EXPECT_TRUE(ed::same_shape(m->shape, hull)) << "the shape reads back bit for bit";

    // Cleared and saved, the file is the version it was before and carries no collision key.
    ASSERT_TRUE(ed::apply(again->scene, ed::SetModelShape{"hover_quad_0", std::nullopt}).has_value());
    ASSERT_TRUE(ed::save_scene(again->scene, again->world).has_value());
    const std::string cleared = read_text(copy);
    EXPECT_EQ(version_line(cleared), before);
    EXPECT_EQ(cleared.find("collision:"), std::string::npos);
}

[[nodiscard]] std::unique_ptr<ed::EditorWorkspace> hover_workspace() {
    spade::Result<ed::OpenedScene> opened = ed::open_scene(support::hover_scene());
    EXPECT_TRUE(opened.has_value()) << (opened ? "" : opened.error().context);
    spade::Result<std::unique_ptr<ed::EditorWorkspace>> w = ed::EditorWorkspace::create(std::move(*opened));
    EXPECT_TRUE(w.has_value()) << (w ? "" : w.error().context);
    return w ? std::move(*w) : nullptr;
}

// The hover quad with a box shape from its parts, dropped tilted from 0.6 m, run 5 s: its final design state.
[[nodiscard]] std::optional<spade::FrameState> drop_the_box(std::string* why) {
    auto w = hover_workspace();
    if (!w) return std::nullopt;
    const spade::scene::SceneDesc& s = w->documents().scene.desc();
    const std::string quad = s.vehicles.at(0).name;
    const auto* model = model_of(s, quad);
    spade::Result<ShapeDesc> box = ed::shape_for_kind(*model, ShapeKind::box);
    if (!box) {
        if (why) *why = box.error().context;
        return std::nullopt;
    }
    box->friction = 0.5f;
    box->restitution = 0.1f;
    // The scene's model rests on a contact sphere of radius 0.2 (proxy_radius) that holds the body up with no torque, before any
    // box face reaches the ground: a crate has a 1 mm one (0 means the world's radius, which is 0.2 here).
    w->panels().pending.push_back(
        ed::SetModelParam{quad, spade::vehicles::ModelIssue::Element::model, 0, "proxy_radius", 0.001f});
    w->panels().pending.push_back(ed::SetModelShape{quad, *box});
    spade::VehicleSpawn start = s.vehicles.at(0).start;
    start.pos.y = 0.6f;
    start.orient = glm::angleAxis(0.6f, glm::normalize(glm::vec3(1.0f, 0.0f, 0.4f)));
    start.vel = glm::vec3(0.0f);
    start.omega_body = glm::vec3(0.0f);
    start.rotor_omega = 0.0f;  // the scene starts the rotors at hover speed: they would fly it, not drop it
    w->panels().pending.push_back(ed::SetVehicleStart{quad, start});
    w->apply_panels();
    w->panels().commands.push_back(ed::Command::run);
    w->apply_panels();
    if (w->run() == nullptr) {
        if (why) *why = w->panels().status;
        return std::nullopt;
    }
    std::string trace;
    for (int i = 0; i < 300; ++i) {
        w->advance(1.0f / 60.0f);
        if (i % 30 == 29) {
            const auto s3 = w->run()->state_of(quad);
            const glm::vec3 ub = glm::conjugate(s3->orient) * glm::vec3(0.0f, 1.0f, 0.0f);
            trace += " [t" + std::to_string(i + 1) + " y " + std::to_string(s3->pos.y) + " |v| " +
                     std::to_string(glm::length(s3->vel)) + " |w| " + std::to_string(glm::length(s3->omega)) + " up " +
                     std::to_string(ub.x) + " " + std::to_string(ub.y) + " " + std::to_string(ub.z) + "]";
        }
    }
    if (why) *why = w->panels().status;  // a refused step pauses the run and says why here
    if (why) {
        const auto s2 = w->run()->state_of(quad);
        *why += " | tick " + std::to_string(w->run()->tick()) + " box " + std::to_string(box->half_extents.x) + " " +
                std::to_string(box->half_extents.y) + " " + std::to_string(box->half_extents.z) + " mass " +
                std::to_string(model->body.mass) + " pos " + std::to_string(s2->pos.x) + " " + std::to_string(s2->pos.y) +
                " " + std::to_string(s2->pos.z) + " vel " + std::to_string(s2->vel.x) + " " + std::to_string(s2->vel.y) +
                " " + std::to_string(s2->vel.z) + trace;
    }
    return w->run()->state_of(quad);
}

TEST(EditorShapeEdit, ABoxDroppedFromATiltLandsOnAFaceAndTwoRunsAreBitEqual) {
    std::string why;
    const std::optional<spade::FrameState> a = drop_the_box(&why);
    ASSERT_TRUE(a.has_value()) << "the Run did not start: " << why;
    EXPECT_TRUE(why.empty()) << "the Run said: " << why;
    // Resting on a face: one of the body's axes is within 3 degrees of the world's up (cos 3 deg = 0.99863), it is nearly
    // still, and it came down from 0.6 m.
    const glm::vec3 up_in_body = glm::conjugate(a->orient) * glm::vec3(0.0f, 1.0f, 0.0f);
    const float aligned = std::max({std::fabs(up_in_body.x), std::fabs(up_in_body.y), std::fabs(up_in_body.z)});
    EXPECT_GE(aligned, 0.99863f) << "up in the body frame: " << up_in_body.x << " " << up_in_body.y << " " << up_in_body.z;
    EXPECT_LT(glm::length(a->vel), 0.05f);
    EXPECT_LT(glm::length(a->omega), 0.2f);
    EXPECT_LT(a->pos.y, 0.3f);

    const std::optional<spade::FrameState> b = drop_the_box(&why);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(std::memcmp(&a->pos, &b->pos, sizeof a->pos), 0);
    EXPECT_EQ(std::memcmp(&a->orient, &b->orient, sizeof a->orient), 0);
}

TEST(EditorShapeEdit, ARunWithoutAShapeBuildsTheStandardSetAsBefore) {
    auto w = hover_workspace();
    ASSERT_NE(w, nullptr);
    w->panels().commands.push_back(ed::Command::run);
    w->apply_panels();
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
}

}  // namespace
