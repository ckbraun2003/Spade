// The editor's edit text (editor plan Task 8; EDT-002).
//
// Every scene and world edit has a one-line text form, so the command line
// and a script can make any edit the window can. Floats are written with 9
// significant digits, so they come back bit for bit. A name with a space, a
// quote or nothing in it is quoted. A line that does not parse names its first
// bad token.
//
// Round trips are checked BY EFFECT: an edit and its re-parsed twin are
// applied to two copies of the same document, and the documents' canonical
// texts must be equal. That compares every field the file carries, not only
// the ones the text form knows about.

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "../sandbox/editor_edit_text.hpp"
#include "editor_test_support.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using Element = spade::vehicles::ModelIssue::Element;
using spade::Result;

// The hover scene plus a plain box "box_1" and a visual panel "panel".
[[nodiscard]] spade::scene::SceneDesc base_scene() {
    spade::scene::SceneDesc s = support::with_asset(support::sample_scene(), "box_1");
    spade::scene::SceneAsset panel = support::asset_named("panel");
    panel.visual.mesh_ref = "panel.glb";
    panel.visual.material = "red";
    s.assets.push_back(panel);
    return s;
}

// The hover world plus a spawn "pad" and a prop.
[[nodiscard]] spade::WorldDesc base_world() {
    spade::WorldDesc w = support::sample_world();
    spade::SpawnPoint pad;
    pad.name = "pad";
    pad.position = glm::vec3(1.0f, 0.0f, 1.0f);
    w.spawns.push_back(pad);
    spade::PropDesc crate;
    crate.mesh_ref = "crate.glb";
    w.props.push_back(crate);
    return w;
}

[[nodiscard]] spade::SdfPose pose_of(glm::vec3 p, float angle, float scale) {
    spade::SdfPose pose;
    pose.position = p;
    pose.rotation = glm::angleAxis(angle, glm::normalize(glm::vec3(0.3f, 1.0f, -0.2f)));
    pose.scale = scale;
    return pose;
}

// An asset no shorthand covers: two transforms, a union, collider materials and a visual.
[[nodiscard]] spade::scene::SceneAsset compound_asset() {
    spade::scene::SceneAsset a;
    a.name = "gantry";
    a.pose = pose_of(glm::vec3(0.0f, 1.5f, -3.0f), 0.25f, 1.0f);
    a.collider.transforms.push_back(spade::SdfTransform{});
    spade::SdfTransform shifted;
    shifted.world_to_local = glm::translate(glm::mat4(1.0f), glm::vec3(-0.75f, 0.0f, 0.1f));
    a.collider.transforms.push_back(shifted);
    spade::SdfNode box{};
    box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    box.params = glm::vec4(1.0f, 0.1f, 0.2f, 0.0f);
    spade::SdfNode ball{};
    ball.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    ball.transform = 1;
    ball.params = glm::vec4(0.3f, 0.0f, 0.0f, 0.0f);
    spade::SdfNode join{};
    join.op = static_cast<uint32_t>(spade::SdfOp::union_);
    a.collider.nodes = {box, ball, join};
    a.collider_materials = {"steel", "steel", "steel"};
    a.visual.mesh_ref = "gantry mesh.glb";
    a.visual.material = "steel";
    return a;
}

[[nodiscard]] std::vector<ed::SceneEdit> scene_samples() {
    const spade::scene::SceneDesc base = base_scene();
    spade::scene::SceneAsset crate = support::asset_named("crate");
    crate.pose = pose_of(glm::vec3(1.0f, 0.5f, -2.0f), 0.7f, 1.25f);
    spade::scene::SceneVehicle launched = base.vehicles.at(0);
    launched.name = "quad 2";
    launched.start.vel = glm::vec3(1.0f, 0.5f, -0.25f);
    spade::vehicles::ModelType model = support::quad_model();
    model.rotors.at(1).radius = 0.15f;
    spade::VehicleSpawn start = base.vehicles.at(0).start;
    start.vel = glm::vec3(0.1f, -0.0f, 3.40282347e38f);
    start.omega_body = glm::vec3(0.2f, 0.0f, -0.3f);
    start.rotor_omega = 512.5f;
    spade::MaterialDesc blue;
    blue.name = "blue paint";
    blue.base_color = glm::vec4(0.1f, 0.2f, 0.3f, 1.0f);
    blue.shading = spade::MaterialShading::unlit;
    return {
        ed::AddAsset{crate},
        ed::AddAsset{compound_asset()},
        ed::AddVehicle{launched, model},
        ed::RemoveObject{"box_1"},
        ed::RenameObject{"box_1", "big box"},
        ed::DuplicateObject{"hover_quad_0"},
        ed::MoveObject{"panel", 0},
        ed::SetAssetPose{"box_1", pose_of(glm::vec3(-0.0f, 0.1f, 1e-5f), 2.5f, 0.75f)},
        ed::SetAssetMaterial{"panel", "blue paint"},
        ed::SetVehicleStart{"hover_quad_0", start},
        ed::AddMaterial{blue},
        ed::RepointWorld{"../worlds/other world.world.yaml", 0x0123456789abcdefULL},
        ed::SetModelParam{"hover_quad_0", Element::rotor, 2, "radius", 0.15f},
        ed::SetModelParam{"hover_quad_0", Element::model, 0, "visual_ref", std::string("quad mesh.glb")},
        ed::SetModelParam{"hover_quad_0", Element::imu_mount, 0, "rate_divider", uint32_t{2}},
        ed::SetModelParam{"hover_quad_0", Element::drag_body, 0, "coeffs", glm::vec3(0.5f, 0.6f, 0.7f)},
        ed::SetModelParam{"hover_quad_0", Element::rotor, 0, "local_orient",
                          glm::angleAxis(0.05f, glm::vec3(0.0f, 0.0f, 1.0f))},
    };
}

[[nodiscard]] std::vector<ed::WorldEdit> world_samples() {
    const spade::WorldDesc base = base_world();
    spade::Environment env = base.environment;
    env.wind = glm::vec3(3.0f, 0.0f, -1.5f);
    env.temperature_k = 290.5f;
    env.seed = 0xFFFFFFFFFFFFFFFFULL;
    spade::MaterialDesc grass;
    grass.name = "grass";
    grass.base_color = glm::vec4(0.2f, 0.6f, 0.1f, 1.0f);
    grass.shading = spade::MaterialShading::emissive;
    spade::MaterialDesc plain = base.materials.at(0);
    plain.base_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);
    spade::LightingDesc light = base.lighting;
    light.sun_intensity = 2.5f;
    light.sky_horizon = glm::vec3(0.9f, 0.8f, 0.7f);
    spade::SpawnPoint roof;
    roof.name = "roof top";
    roof.position = glm::vec3(0.0f, 10.0f, 0.0f);
    roof.orientation = glm::angleAxis(1.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    spade::SpawnPoint helipad = roof;
    helipad.name = "helipad";
    spade::PropDesc barrel;
    barrel.mesh_ref = "barrel.glb";
    barrel.pose = pose_of(glm::vec3(2.0f, 0.0f, 2.0f), 0.3f, 1.5f);
    return {
        ed::RenameWorld{"windy field"},
        ed::SetEnvironment{env},
        ed::SetCapacities{spade::Capacities{2, 3, 4, 5}},
        ed::AddWorldMaterial{grass},
        ed::SetWorldMaterial{0, plain},
        ed::SetLighting{light},
        ed::AddSpawn{roof},
        ed::SetSpawn{"pad", helipad},
        ed::RemoveSpawn{"pad"},
        ed::AddProp{barrel},
        ed::SetProp{0, barrel},
        ed::RemoveProp{0},
    };
}

}  // namespace

TEST(EditorEditText, EverySceneEditRoundTripsByEffect) {
    std::set<std::size_t> kinds;
    for (const ed::SceneEdit& e : scene_samples()) {
        kinds.insert(e.index());
        const Result<std::string> line = ed::format_edit(e);
        ASSERT_TRUE(line.has_value()) << "kind " << e.index() << ": " << line.error().context;
        EXPECT_TRUE(line->starts_with("scene ")) << *line;
        const Result<ed::SceneEdit> back = ed::parse_edit(*line);
        ASSERT_TRUE(back.has_value()) << *line << "\n" << back.error().context;
        EXPECT_EQ(back->index(), e.index()) << *line;
        EXPECT_EQ(ed::format_edit(*back).value_or("<no form>"), *line);
        auto a = ed::make_scene_document(base_scene(), {});
        auto b = ed::make_scene_document(base_scene(), {});
        const Result<void> ra = ed::apply(a, e);
        const Result<void> rb = ed::apply(b, *back);
        ASSERT_TRUE(ra.has_value()) << *line << "\n" << ra.error().context;
        ASSERT_TRUE(rb.has_value()) << *line << "\n" << rb.error().context;
        EXPECT_EQ(support::text_of(a.desc()), support::text_of(b.desc())) << *line;
    }
    EXPECT_EQ(kinds.size(), std::variant_size_v<ed::SceneEdit>);
}

TEST(EditorEditText, EveryWorldEditRoundTripsByEffect) {
    std::set<std::size_t> kinds;
    for (const ed::WorldEdit& e : world_samples()) {
        kinds.insert(e.index());
        const Result<std::string> line = ed::format_world_edit(e);
        ASSERT_TRUE(line.has_value()) << "kind " << e.index() << ": " << line.error().context;
        EXPECT_TRUE(line->starts_with("world ")) << *line;
        const Result<ed::WorldEdit> back = ed::parse_world_edit(*line);
        ASSERT_TRUE(back.has_value()) << *line << "\n" << back.error().context;
        EXPECT_EQ(ed::format_world_edit(*back).value_or("<no form>"), *line);
        auto a = ed::make_world_document(base_world(), {});
        auto b = ed::make_world_document(base_world(), {});
        const Result<void> ra = ed::apply(a, e);
        const Result<void> rb = ed::apply(b, *back);
        ASSERT_TRUE(ra.has_value()) << *line << "\n" << ra.error().context;
        ASSERT_TRUE(rb.has_value()) << *line << "\n" << rb.error().context;
        EXPECT_EQ(support::world_text_of(a.desc()), support::world_text_of(b.desc())) << *line;
    }
    EXPECT_EQ(kinds.size(), std::variant_size_v<ed::WorldEdit>);
}

TEST(EditorEditText, FloatsComeBackBitForBit) {
    spade::SdfPose p;
    p.position = glm::vec3(-0.0f, 0.1f, 3.40282347e38f);
    p.rotation = glm::quat(0.99999994f, 1.17549435e-38f, -1e-7f, 0.0f);
    p.scale = 1.20000004e-05f;
    const Result<ed::SceneEdit> back = ed::parse_edit(ed::format_edit(ed::SetAssetPose{"box_1", p}).value());
    ASSERT_TRUE(back.has_value()) << back.error().context;
    const spade::SdfPose& q = std::get<ed::SetAssetPose>(*back).pose;
    const auto bits = [](float f) { return std::bit_cast<uint32_t>(f); };
    for (int i = 0; i < 3; ++i) EXPECT_EQ(bits(q.position[i]), bits(p.position[i])) << i;
    EXPECT_EQ(bits(q.rotation.w), bits(p.rotation.w));
    EXPECT_EQ(bits(q.rotation.x), bits(p.rotation.x));
    EXPECT_EQ(bits(q.rotation.y), bits(p.rotation.y));
    EXPECT_EQ(bits(q.scale), bits(p.scale));
}

TEST(EditorEditText, APrimitiveAssetUsesTheShortForm) {
    const std::string line = ed::format_edit(ed::AddAsset{support::asset_named("crate")}).value();
    EXPECT_TRUE(line.starts_with("scene add-asset crate box 0.5 0.5 0.5 pos 0 0 0 rot 1 0 0 0 scale 1")) << line;
    const std::string full = ed::format_edit(ed::AddAsset{compound_asset()}).value();
    EXPECT_NE(full.find(" transforms 2 "), std::string::npos) << full;
}

TEST(EditorEditText, NamesThatNeedItAreQuoted) {
    const std::string line = ed::format_edit(ed::RenameObject{"box_1", "big \"box\""}).value();
    EXPECT_EQ(line, R"(scene rename box_1 "big \"box\"")");
    const Result<ed::SceneEdit> back = ed::parse_edit(line);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(std::get<ed::RenameObject>(*back).to, "big \"box\"");
    EXPECT_EQ(ed::format_edit(ed::RenameObject{"", "x"}).value(), R"(scene rename "" x)");
}

TEST(EditorEditText, AMalformedLineNamesItsFirstBadToken) {
    const auto error_of = [](std::string_view line) {
        const Result<ed::SceneEdit> r = ed::parse_edit(line);
        return r ? std::string("<parsed>") : r.error().context;
    };
    const std::string bad_number = error_of("scene set-asset-pose box_1 pos 1 2 x rot 1 0 0 0 scale 1");
    EXPECT_NE(bad_number.find("token 7"), std::string::npos) << bad_number;
    EXPECT_NE(bad_number.find("'x'"), std::string::npos) << bad_number;
    EXPECT_NE(error_of("scene frobnicate a").find("'frobnicate'"), std::string::npos);
    EXPECT_NE(error_of("scene remove a extra").find("'extra'"), std::string::npos);
    EXPECT_NE(error_of(R"(scene rename "unterminated)").find("unterminated"), std::string::npos);
    EXPECT_NE(error_of("scene").find("end of the line"), std::string::npos);
    EXPECT_NE(error_of("scene move box_1 -1").find("'-1'"), std::string::npos);
    EXPECT_NE(error_of("world rename x").find("'world'"), std::string::npos);  // a world line is not a scene edit
    const Result<ed::WorldEdit> short_line = ed::parse_world_edit("world set-capacities bodies 1 force-elements 2 sensors 3");
    ASSERT_FALSE(short_line.has_value());
    EXPECT_NE(short_line.error().context.find("contacts"), std::string::npos) << short_line.error().context;
}

TEST(EditorEditText, AModelParameterIsReadByItsTableKind) {
    const Result<ed::SceneEdit> e = ed::parse_edit("scene set-model-param q imu-mount 0 rate_divider 4");
    ASSERT_TRUE(e.has_value()) << e.error().context;
    EXPECT_EQ(std::get<uint32_t>(std::get<ed::SetModelParam>(*e).value), 4u);
    const Result<ed::SceneEdit> unknown = ed::parse_edit("scene set-model-param q rotor 0 wingspan 1");
    ASSERT_FALSE(unknown.has_value());
    EXPECT_NE(unknown.error().context.find("'wingspan'"), std::string::npos) << unknown.error().context;
    // A value of another kind than the table's has no text form.
    EXPECT_FALSE(ed::format_edit(ed::SetModelParam{"q", Element::rotor, 0, "radius", glm::vec3(1.0f)}).has_value());
}

TEST(EditorEditText, ALineIsRoutedByItsFirstWord) {
    const Result<ed::AnyEdit> w = ed::parse_any_edit("world rename field");
    ASSERT_TRUE(w.has_value());
    EXPECT_TRUE(std::holds_alternative<ed::WorldEdit>(*w));
    const Result<ed::AnyEdit> s = ed::parse_any_edit("  scene remove box_1  ");
    ASSERT_TRUE(s.has_value());
    EXPECT_TRUE(std::holds_alternative<ed::SceneEdit>(*s));
    const Result<ed::AnyEdit> neither = ed::parse_any_edit("model remove x");
    ASSERT_FALSE(neither.has_value());
    EXPECT_NE(neither.error().context.find("'model'"), std::string::npos) << neither.error().context;
}
