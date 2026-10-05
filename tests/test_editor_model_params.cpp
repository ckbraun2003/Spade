// The inspector's model parameters (editor plan Task 3; EDT-013, EDT-014,
// EDT-019).
//
// Every ModelType parameter is in the table and can be read and written by
// path (element, index, field). An edit reaches the selected vehicle alone: a
// model another vehicle shares is copied for it first (the user's Q1). An
// invalid value is refused with ModelType::issues(), naming its field. There is
// no edit that adds or removes a part.

#include <gtest/gtest.h>

#include <set>
#include <string>

#include "../sandbox/editor_model_params.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "editor_test_support.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using Element = spade::vehicles::ModelIssue::Element;
using spade::Result;

[[nodiscard]] std::size_t parts(const spade::vehicles::ModelType& m) {
    return m.rotors.size() * 1000000u + m.drag_bodies.size() * 1000u + m.imu_mounts.size();
}

}  // namespace

TEST(EditorModelParams, TheTableCoversEveryParameterOnce) {
    std::set<std::pair<int, std::string>> seen;
    for (const ed::ModelParam& p : ed::model_params()) {
        EXPECT_TRUE(seen.insert({static_cast<int>(p.element), std::string(p.field)}).second) << p.field;
    }
    // 8 model fields, 7 per rotor, 5 per drag body, 7 per IMU mount: every
    // field of ModelType and its parts except the model's name, its identity.
    EXPECT_EQ(seen.size(), 8u + 7u + 5u + 7u);
}

TEST(EditorModelParams, EveryFieldInTheTableReadsAndWritesBack) {
    const spade::vehicles::ModelType m = support::quad_model();
    ASSERT_FALSE(m.rotors.empty());
    ASSERT_FALSE(m.drag_bodies.empty());
    ASSERT_FALSE(m.imu_mounts.empty());
    for (const ed::ModelParam& p : ed::model_params()) {
        const Result<ed::ParamValue> v = ed::get_model_param(m, p.element, 0, p.field);
        ASSERT_TRUE(v.has_value()) << p.field << ": " << v.error().context;
        auto d = ed::make_scene_document(support::scene_with(m), {});
        const Result<void> set = ed::apply(d, ed::SetModelParam{"quad_0", p.element, 0, std::string(p.field), *v});
        EXPECT_TRUE(set.has_value()) << p.field << ": " << (set ? "" : set.error().context);
        EXPECT_EQ(support::text_of(d.desc()), support::text_of(support::scene_with(m))) << p.field;
    }
}

TEST(EditorModelParams, AnEditReachesTheModelItNames) {
    auto d = ed::make_scene_document(support::scene_with(support::quad_model()), {});
    ASSERT_TRUE(ed::apply(d, ed::SetModelParam{"quad_0", Element::rotor, 2, "radius", 0.15f}).has_value());
    EXPECT_EQ(support::model_named(d.desc(), "quad").rotors[2].radius, 0.15f);
    EXPECT_EQ(d.desc().models.size(), 1u);  // not shared, so not copied
}

TEST(EditorModelParams, AnEditToASharedModelReachesThatVehicleAlone) {
    const spade::scene::SceneDesc before = support::two_vehicles_sharing("quad");
    auto d = ed::make_scene_document(before, {});
    ASSERT_TRUE(ed::apply(d, ed::SetModelParam{"quad_1", Element::rotor, 0, "radius", 0.15f}).has_value());
    EXPECT_EQ(d.desc().vehicles[0].model, "quad");
    EXPECT_EQ(d.desc().vehicles[1].model, "quad_2");
    EXPECT_EQ(support::model_named(d.desc(), "quad").rotors[0].radius,
              support::model_named(before, "quad").rotors[0].radius);
    EXPECT_EQ(support::model_named(d.desc(), "quad_2").rotors[0].radius, 0.15f);
    ASSERT_TRUE(d.undo());
    EXPECT_EQ(support::text_of(d.desc()), support::text_of(before));
}

// The editor plan's review focus 1.
TEST(EditorModelParams, OpeningASharedModelSceneCopiesNothing) {
    const spade::scene::SceneDesc s = support::two_vehicles_sharing("quad");
    const auto d = ed::make_scene_document(s, {});
    EXPECT_EQ(support::text_of(d.desc()), support::text_of(s));
}

TEST(EditorModelParams, AnInvalidValueIsRefusedNamingItsField) {
    auto d = ed::make_scene_document(support::scene_with(support::quad_model()), {});
    const std::string before = support::text_of(d.desc());
    const Result<void> r = ed::apply(d, ed::SetModelParam{"quad_0", Element::rotor, 2, "radius", -1.0f});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("rotor 2"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("radius"), std::string::npos) << r.error().context;
    EXPECT_EQ(support::text_of(d.desc()), before);
}

TEST(EditorModelParams, AFailedEditToASharedModelCopiesNothing) {
    const spade::scene::SceneDesc before = support::two_vehicles_sharing("quad");
    auto d = ed::make_scene_document(before, {});
    EXPECT_FALSE(ed::apply(d, ed::SetModelParam{"quad_1", Element::rotor, 0, "radius", -1.0f}).has_value());
    EXPECT_EQ(support::text_of(d.desc()), support::text_of(before));
}

TEST(EditorModelParams, NoEditChangesAPartCount) {
    const spade::vehicles::ModelType m = support::quad_model();
    for (const ed::ModelParam& p : ed::model_params()) {
        auto d = ed::make_scene_document(support::scene_with(m), {});
        const Result<ed::ParamValue> v = ed::get_model_param(m, p.element, 0, p.field);
        ASSERT_TRUE(v.has_value());
        (void)ed::apply(d, ed::SetModelParam{"quad_0", p.element, 0, std::string(p.field), *v});
        EXPECT_EQ(parts(d.desc().models.at(0)), parts(m)) << p.field;
    }
}

TEST(EditorModelParams, APartPastTheModelsCountIsRefusedNotAdded) {
    auto d = ed::make_scene_document(support::scene_with(support::quad_model()), {});
    const Result<void> r = ed::apply(d, ed::SetModelParam{"quad_0", Element::rotor, 4, "radius", 0.1f});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("4 rotors"), std::string::npos) << r.error().context;
    EXPECT_EQ(d.desc().models.at(0).rotors.size(), 4u);
}

TEST(EditorModelParams, AValueOfTheWrongKindOrAnUnknownFieldIsRefused) {
    auto d = ed::make_scene_document(support::scene_with(support::quad_model()), {});
    EXPECT_FALSE(ed::apply(d, ed::SetModelParam{"quad_0", Element::rotor, 0, "radius", glm::vec3(1.0f)}).has_value());
    const Result<void> r = ed::apply(d, ed::SetModelParam{"quad_0", Element::rotor, 0, "wingspan", 1.0f});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("wingspan"), std::string::npos) << r.error().context;
}
