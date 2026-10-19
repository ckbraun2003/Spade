// The editor's scene edits (editor plan Task 2; EDT-003, EDT-008).
//
// Each edit is a pure change to a copy of the scene, committed through the
// document, so it is validated by validate_scene() and can be undone. Objects
// are addressed by name; a new name is unique across assets and vehicles
// together (SCN-004), and each vehicle gets its own model (the user's Q1).

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "../sandbox/editor_files.hpp"
#include "../sandbox/editor_library.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "editor_test_support.hpp"
#include "render/scene.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::Result;

[[nodiscard]] ed::SceneDocument doc_of(spade::scene::SceneDesc s) { return ed::make_scene_document(std::move(s), {}); }

[[nodiscard]] const spade::scene::SceneVehicle* vehicle(const spade::scene::SceneDesc& s, const std::string& name) {
    for (const auto& v : s.vehicles) {
        if (v.name == name) return &v;
    }
    return nullptr;
}

// The colour the scene's draw gives the static item standing at `where`: the
// scene and world composed as the editor composes them, then turned into the
// render scene the editor's view draws (EditorSession::rebuild_scene()).
[[nodiscard]] std::optional<glm::vec4> drawn_colour_at(const spade::scene::SceneDesc& s, const spade::WorldDesc& w,
                                                       glm::vec3 where) {
    const Result<spade::scene::ComposedScene> composed = ed::compose_together(s, w);
    EXPECT_TRUE(composed.has_value()) << (composed ? "" : composed.error().context);
    if (!composed) return std::nullopt;
    const Result<spade::render::RenderScene> drawn = spade::render::scene_from_world(composed->world, {});
    EXPECT_TRUE(drawn.has_value()) << (drawn ? "" : drawn.error().context);
    if (!drawn) return std::nullopt;
    for (const spade::render::DrawItem& item : drawn->statics) {
        if (glm::length(glm::vec3(item.local_to_world[3]) - where) > 1e-4f) continue;
        const uint32_t m = item.material_override == spade::render::kNoMaterial ? 0u : item.material_override;
        if (m >= drawn->materials.size()) return std::nullopt;
        return drawn->materials[m].base_color;
    }
    return std::nullopt;
}

}  // namespace

// The editor plan's review focus 2.
TEST(EditorSceneEdits, ANewNameIsUniqueAcrossAssetsAndVehicles) {
    spade::scene::SceneDesc s = support::sample_scene();  // holds vehicle "hover_quad_0"
    s.assets.push_back(support::asset_named("gate_2"));
    EXPECT_EQ(ed::unique_name(s, "gate"), "gate");
    s.assets.push_back(support::asset_named("gate"));
    EXPECT_EQ(ed::unique_name(s, "gate"), "gate_3");  // gate_2 is taken
    EXPECT_EQ(ed::unique_name(s, "hover_quad_0"), "hover_quad_0_2");
    EXPECT_EQ(ed::unique_name(s, ""), "object");
}

TEST(EditorSceneEdits, AModelNameIsUniqueAmongModels) {
    const spade::scene::SceneDesc s = support::sample_scene();  // holds model "hover_quad"
    EXPECT_EQ(ed::unique_model_name(s, "hover_quad"), "hover_quad_2");
    EXPECT_EQ(ed::unique_model_name(s, "other"), "other");
}

TEST(EditorSceneEdits, AddingAnAssetMakesItsNameUnique) {
    auto d = doc_of(support::with_asset(support::sample_scene(), "box"));
    ASSERT_TRUE(ed::apply(d, ed::AddAsset{support::asset_named("box")}).has_value());
    ASSERT_EQ(d.desc().assets.size(), 2u);
    EXPECT_EQ(d.desc().assets[1].name, "box_2");
}

TEST(EditorSceneEdits, AddingAVehicleCopiesItsModelForItAlone) {
    auto d = doc_of(support::sample_scene());
    const spade::scene::SceneVehicle base = d.desc().vehicles[0];
    const spade::vehicles::ModelType model = d.desc().models[0];
    ASSERT_TRUE(ed::apply(d, ed::AddVehicle{base, model}).has_value());
    ASSERT_EQ(d.desc().vehicles.size(), 2u);
    EXPECT_EQ(d.desc().vehicles[1].name, "hover_quad_0_2");
    EXPECT_EQ(d.desc().vehicles[1].model, "hover_quad_2");
    EXPECT_EQ(d.desc().models.size(), 2u);
}

TEST(EditorSceneEdits, DuplicatingAVehicleCopiesItsModelToo) {
    auto d = doc_of(support::sample_scene());
    ASSERT_TRUE(ed::apply(d, ed::DuplicateObject{"hover_quad_0"}).has_value());
    ASSERT_EQ(d.desc().vehicles.size(), 2u);
    EXPECT_NE(d.desc().vehicles[0].model, d.desc().vehicles[1].model);  // one model per vehicle (Q1)
}

TEST(EditorSceneEdits, DuplicatingAnAssetCopiesItUnderTheLowestFreeName) {
    auto d = doc_of(support::with_asset(support::sample_scene(), "rock"));
    ASSERT_TRUE(ed::apply(d, ed::DuplicateObject{"rock"}).has_value());
    ASSERT_TRUE(ed::apply(d, ed::DuplicateObject{"rock"}).has_value());
    ASSERT_EQ(d.desc().assets.size(), 3u);
    EXPECT_EQ(d.desc().assets[1].name, "rock_2");
    EXPECT_EQ(d.desc().assets[2].name, "rock_3");
}

TEST(EditorSceneEdits, ARenameOntoATakenNameIsRefusedAndChangesNothing) {
    auto d = doc_of(support::with_asset(support::sample_scene(), "rock"));
    const std::string before = support::text_of(d.desc());
    const Result<void> r = ed::apply(d, ed::RenameObject{"rock", "hover_quad_0"});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("hover_quad_0"), std::string::npos) << r.error().context;
    EXPECT_EQ(support::text_of(d.desc()), before);
    ASSERT_TRUE(ed::apply(d, ed::RenameObject{"rock", "boulder"}).has_value());
    EXPECT_EQ(d.desc().assets[0].name, "boulder");
}

TEST(EditorSceneEdits, AnUnknownNameIsRefusedNamingIt) {
    auto d = doc_of(support::sample_scene());
    for (const ed::SceneEdit& e : {ed::SceneEdit{ed::RemoveObject{"ghost"}}, ed::SceneEdit{ed::RenameObject{"ghost", "x"}},
                                   ed::SceneEdit{ed::DuplicateObject{"ghost"}}, ed::SceneEdit{ed::MoveObject{"ghost", 0}},
                                   ed::SceneEdit{ed::SetAssetPose{"ghost", {}}},
                                   ed::SceneEdit{ed::SetVehicleStart{"ghost", {}}}}) {
        const Result<void> r = ed::apply(d, e);
        ASSERT_FALSE(r.has_value());
        EXPECT_NE(r.error().context.find("'ghost'"), std::string::npos) << r.error().context;
    }
}

TEST(EditorSceneEdits, RemovingAVehicleRemovesItsOwnModelButNotASharedOne) {
    auto d = doc_of(support::sample_scene());
    ASSERT_TRUE(ed::apply(d, ed::DuplicateObject{"hover_quad_0"}).has_value());  // two vehicles, two models
    ASSERT_TRUE(ed::apply(d, ed::RemoveObject{"hover_quad_0_2"}).has_value());
    EXPECT_EQ(d.desc().vehicles.size(), 1u);  // the live smoke's builder.delete/removes-one, for a vehicle
    EXPECT_EQ(vehicle(d.desc(), "hover_quad_0_2"), nullptr);
    EXPECT_EQ(d.desc().models.size(), 1u);
    // Two vehicles sharing one model (as a Kat scene may): removing one keeps it.
    spade::scene::SceneDesc shared = support::sample_scene();
    spade::scene::SceneVehicle twin = shared.vehicles[0];
    twin.name = "twin";
    shared.vehicles.push_back(twin);
    auto s = doc_of(shared);
    ASSERT_TRUE(ed::apply(s, ed::RemoveObject{"twin"}).has_value());
    EXPECT_EQ(s.desc().vehicles.size(), 1u);
    EXPECT_EQ(s.desc().models.size(), 1u);
    EXPECT_NE(vehicle(s.desc(), "hover_quad_0"), nullptr);
}

// builder.delete/removes-one, for a placed box: one object fewer, the named one.
TEST(EditorSceneEdits, RemovingAnAssetLeavesTheOthers) {
    auto d = doc_of(support::with_asset(support::with_asset(support::sample_scene(), "box_1"), "box_2"));
    ASSERT_TRUE(ed::apply(d, ed::RemoveObject{"box_1"}).has_value());
    ASSERT_EQ(d.desc().assets.size(), 1u);
    EXPECT_EQ(d.desc().assets[0].name, "box_2");
}

TEST(EditorSceneEdits, ReorderingIsAnEditAndIsUndone) {
    auto d = doc_of(support::with_asset(support::with_asset(support::sample_scene(), "a"), "b"));
    const std::string before = support::text_of(d.desc());
    ASSERT_TRUE(ed::apply(d, ed::MoveObject{"b", 0}).has_value());
    EXPECT_EQ(d.desc().assets[0].name, "b");
    ASSERT_TRUE(d.undo());
    EXPECT_EQ(support::text_of(d.desc()), before);
    EXPECT_FALSE(ed::apply(d, ed::MoveObject{"a", 2}).has_value());  // past the end
}

TEST(EditorSceneEdits, AnEditTheValidatorRefusesNamesTheCause) {
    auto d = doc_of(support::with_asset(support::sample_scene(), "box_1"));
    spade::SdfPose p;
    p.scale = 0.0f;
    const Result<void> r = ed::apply(d, ed::SetAssetPose{"box_1", p});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("scale"), std::string::npos) << r.error().context;
}

TEST(EditorSceneEdits, PoseStartMaterialAndWorldEditsLandAndUndo) {
    auto d = doc_of(support::with_asset(support::sample_scene(), "box_1"));
    const std::string before = support::text_of(d.desc());
    spade::SdfPose p;
    p.position = glm::vec3(1.0f, 2.0f, 3.0f);
    ASSERT_TRUE(ed::apply(d, ed::SetAssetPose{"box_1", p}).has_value());
    spade::VehicleSpawn start = d.desc().vehicles[0].start;
    start.pos.y += 1.0f;
    ASSERT_TRUE(ed::apply(d, ed::SetVehicleStart{"hover_quad_0", start}).has_value());
    spade::MaterialDesc red;
    red.name = "red";
    ASSERT_TRUE(ed::apply(d, ed::AddMaterial{red}).has_value());
    ASSERT_TRUE(ed::apply(d, ed::RepointWorld{"other.world.yaml", 42u}).has_value());
    EXPECT_EQ(d.desc().assets[0].pose.position, p.position);
    EXPECT_EQ(d.desc().vehicles[0].start.pos, start.pos);
    EXPECT_EQ(d.desc().materials.back().name, "red");
    EXPECT_EQ(d.desc().world.file, "other.world.yaml");
    EXPECT_EQ(d.desc().world.hash, 42u);
    for (int i = 0; i < 4; ++i) ASSERT_TRUE(d.undo());
    EXPECT_EQ(support::text_of(d.desc()), before);
}

TEST(EditorSceneEdits, AMaterialGoesOnlyOnAnAssetWithAVisual) {
    auto d = doc_of(support::with_asset(support::sample_scene(), "box_1"));  // no visual
    const Result<void> r = ed::apply(d, ed::SetAssetMaterial{"box_1", "red"});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("visual"), std::string::npos) << r.error().context;
}

// The live smoke's builder.recolour/reaches-material, on the editor's own
// library box: a box placed from the library has no visual, so it draws as its
// collider, and a collider material is what colours it. Undo puts the old
// colour back, read the same way.
TEST(EditorSceneEdits, AColliderMaterialColoursALibraryBoxAndUndoes) {
    auto d = doc_of(support::sample_scene());
    const spade::WorldDesc world = support::sample_world();
    const std::vector<ed::LibraryItem> library = ed::builtin_library();
    const auto box = std::find_if(library.begin(), library.end(), [](const auto& i) { return i.name == "box"; });
    ASSERT_NE(box, library.end());
    spade::SdfPose at;
    at.position = glm::vec3(3.0f, 0.5f, -2.0f);
    const Result<std::string> name = ed::place(d, *box, at);
    ASSERT_TRUE(name.has_value()) << name.error().context;
    spade::MaterialDesc red;
    red.name = "red";
    red.base_color = glm::vec4(1.0f, 0.1f, 0.05f, 1.0f);
    ASSERT_TRUE(ed::apply(d, ed::AddMaterial{red}).has_value());

    const std::optional<glm::vec4> before = drawn_colour_at(d.desc(), world, at.position);
    ASSERT_TRUE(before.has_value()) << "no item draws the placed box at its position";
    ASSERT_NE(*before, red.base_color);

    ASSERT_TRUE(ed::apply(d, ed::SetColliderMaterial{*name, "red"}).has_value());
    EXPECT_EQ(drawn_colour_at(d.desc(), world, at.position), std::optional<glm::vec4>(red.base_color))
        << "the box does not draw in the material it was given";

    ASSERT_TRUE(d.undo());
    EXPECT_EQ(drawn_colour_at(d.desc(), world, at.position), before) << "undo did not restore the box's colour";
}

TEST(EditorSceneEdits, AColliderMaterialNeedsACollider) {
    spade::scene::SceneDesc s = support::sample_scene();
    spade::scene::SceneAsset ghost;
    ghost.name = "ghost";  // no nodes: it neither collides nor draws
    s.assets.push_back(ghost);
    auto d = doc_of(s);
    const Result<void> r = ed::apply(d, ed::SetColliderMaterial{"ghost", "red"});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("no collider"), std::string::npos) << r.error().context;
}
