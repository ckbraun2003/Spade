// The editor's world edits and the new-world template (editor plan Task 4;
// EDT-015).
//
// Each edit changes a copy of the world and commits it through the document,
// so validate_world_desc() judges it and it can be undone. Spawn points are
// addressed by name (the world file looks them up by name). Materials and
// props are addressed by index, because that is their identity in the world
// file: node_materials and PropDesc::material refer to a material by index, a
// material's name need not be unique, and a prop has no name.

#include <gtest/gtest.h>

#include <string>

#include "../sandbox/editor_world_edits.hpp"
#include "editor_test_support.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::Result;

[[nodiscard]] ed::WorldDocument doc_of(spade::WorldDesc w) { return ed::make_world_document(std::move(w), {}); }

[[nodiscard]] spade::SpawnPoint spawn_named(std::string name, glm::vec3 at = glm::vec3(0.0f, 1.0f, 0.0f)) {
    spade::SpawnPoint s;
    s.name = std::move(name);
    s.position = at;
    return s;
}

[[nodiscard]] spade::MaterialDesc material_named(std::string name, glm::vec4 color = glm::vec4(1, 0, 0, 1)) {
    spade::MaterialDesc m;
    m.name = std::move(name);
    m.base_color = color;
    return m;
}

[[nodiscard]] spade::PropDesc prop_of(std::string mesh, uint32_t material = 0) {
    spade::PropDesc p;
    p.mesh_ref = std::move(mesh);
    p.material = material;
    return p;
}

}  // namespace

TEST(EditorWorldEdits, ANewWorldIsAGroundPlaneThatValidatesAndRoundTrips) {
    const Result<spade::WorldDesc> w = ed::new_world("field");
    ASSERT_TRUE(w.has_value()) << w.error().context;
    EXPECT_EQ(w->name, "field");
    EXPECT_TRUE(spade::validate_world_desc(*w).has_value());
    ASSERT_EQ(w->sdf.nodes.size(), 1u);
    EXPECT_EQ(w->sdf.nodes[0].kind, static_cast<uint32_t>(spade::SdfPrim::plane));
    EXPECT_EQ(w->capacities.bodies, 1u);  // compose() sizes a scene's own need on top (SCN-007)
    const std::string text = support::world_text_of(*w);
    const Result<spade::WorldDesc> back = spade::world_from_yaml(text);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(support::world_text_of(*back), text);
}

TEST(EditorWorldEdits, ANewWorldNeedsAName) {
    const Result<spade::WorldDesc> w = ed::new_world("");
    ASSERT_FALSE(w.has_value());
    EXPECT_NE(w.error().context.find("name"), std::string::npos) << w.error().context;
}

TEST(EditorWorldEdits, NameEnvironmentCapacitiesAndLightingLandAndUndo) {
    auto d = doc_of(support::sample_world());
    const std::string before = support::world_text_of(d.desc());
    spade::Environment env = d.desc().environment;
    env.wind = glm::vec3(3.0f, 0.0f, -1.0f);
    spade::Capacities caps = d.desc().capacities;
    caps.bodies = 8;
    spade::LightingDesc light = d.desc().lighting;
    light.sun_intensity = 2.5f;
    ASSERT_TRUE(ed::apply(d, ed::RenameWorld{"windy"}).has_value());
    ASSERT_TRUE(ed::apply(d, ed::SetEnvironment{env}).has_value());
    ASSERT_TRUE(ed::apply(d, ed::SetCapacities{caps}).has_value());
    ASSERT_TRUE(ed::apply(d, ed::SetLighting{light}).has_value());
    EXPECT_EQ(d.desc().name, "windy");
    EXPECT_EQ(d.desc().environment.wind, env.wind);
    EXPECT_EQ(d.desc().capacities.bodies, 8u);
    EXPECT_EQ(d.desc().lighting.sun_intensity, 2.5f);
    for (int i = 0; i < 4; ++i) ASSERT_TRUE(d.undo());
    EXPECT_EQ(support::world_text_of(d.desc()), before);
}

TEST(EditorWorldEdits, AnEmptyWorldNameIsRefused) {
    auto d = doc_of(support::sample_world());
    const Result<void> r = ed::apply(d, ed::RenameWorld{""});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("name"), std::string::npos) << r.error().context;
}

TEST(EditorWorldEdits, AZeroCapacityIsRefusedNamingTheField) {
    auto d = doc_of(support::sample_world());
    const std::string before = support::world_text_of(d.desc());
    spade::Capacities caps = d.desc().capacities;
    caps.contacts = 0;
    const Result<void> r = ed::apply(d, ed::SetCapacities{caps});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("contacts"), std::string::npos) << r.error().context;
    EXPECT_EQ(support::world_text_of(d.desc()), before);
}

TEST(EditorWorldEdits, AnAddedSpawnGetsAUniqueName) {
    auto d = doc_of(support::sample_world());
    ASSERT_TRUE(ed::apply(d, ed::AddSpawn{spawn_named("pad")}).has_value());
    ASSERT_TRUE(ed::apply(d, ed::AddSpawn{spawn_named("pad")}).has_value());
    ASSERT_EQ(d.desc().spawns.size(), 2u);
    EXPECT_EQ(d.desc().spawns[0].name, "pad");
    EXPECT_EQ(d.desc().spawns[1].name, "pad_2");
}

TEST(EditorWorldEdits, SpawnsAreEditedAndRemovedByName) {
    auto d = doc_of(support::sample_world());
    ASSERT_TRUE(ed::apply(d, ed::AddSpawn{spawn_named("pad")}).has_value());
    ASSERT_TRUE(ed::apply(d, ed::AddSpawn{spawn_named("roof")}).has_value());
    ASSERT_TRUE(ed::apply(d, ed::SetSpawn{"pad", spawn_named("helipad", glm::vec3(5, 1, 5))}).has_value());
    EXPECT_EQ(d.desc().spawns[0].name, "helipad");
    EXPECT_EQ(d.desc().spawns[0].position, glm::vec3(5, 1, 5));
    const Result<void> taken = ed::apply(d, ed::SetSpawn{"helipad", spawn_named("roof")});
    ASSERT_FALSE(taken.has_value());
    EXPECT_NE(taken.error().context.find("'roof'"), std::string::npos) << taken.error().context;
    ASSERT_TRUE(ed::apply(d, ed::RemoveSpawn{"roof"}).has_value());
    ASSERT_EQ(d.desc().spawns.size(), 1u);
    for (const ed::WorldEdit& e : {ed::WorldEdit{ed::SetSpawn{"ghost", spawn_named("x")}},
                                   ed::WorldEdit{ed::RemoveSpawn{"ghost"}}}) {
        const Result<void> r = ed::apply(d, e);
        ASSERT_FALSE(r.has_value());
        EXPECT_NE(r.error().context.find("'ghost'"), std::string::npos) << r.error().context;
    }
}

TEST(EditorWorldEdits, ASpawnTheValidatorRefusesChangesNothing) {
    auto d = doc_of(support::sample_world());
    ASSERT_TRUE(ed::apply(d, ed::AddSpawn{spawn_named("pad")}).has_value());
    const std::string before = support::world_text_of(d.desc());
    spade::SpawnPoint bad = spawn_named("pad");
    bad.orientation = glm::quat(2.0f, 0.0f, 0.0f, 0.0f);  // not unit
    EXPECT_FALSE(ed::apply(d, ed::SetSpawn{"pad", bad}).has_value());
    EXPECT_EQ(support::world_text_of(d.desc()), before);
}

TEST(EditorWorldEdits, MaterialsAreAppendedAndEditedByIndex) {
    auto d = doc_of(support::sample_world());
    const std::size_t n = d.desc().materials.size();
    ASSERT_TRUE(ed::apply(d, ed::AddWorldMaterial{material_named("red")}).has_value());
    ASSERT_EQ(d.desc().materials.size(), n + 1);
    EXPECT_EQ(d.desc().materials[n].name, "red");
    const auto idx = static_cast<uint32_t>(n);
    ASSERT_TRUE(ed::apply(d, ed::SetWorldMaterial{idx, material_named("blue", glm::vec4(0, 0, 1, 1))}).has_value());
    EXPECT_EQ(d.desc().materials[n].name, "blue");
    const Result<void> past = ed::apply(d, ed::SetWorldMaterial{idx + 5, material_named("x")});
    ASSERT_FALSE(past.has_value());
    EXPECT_NE(past.error().context.find(std::to_string(idx + 5)), std::string::npos) << past.error().context;
    EXPECT_FALSE(ed::apply(d, ed::SetWorldMaterial{idx, material_named("")}).has_value());  // the validator: empty name
}

TEST(EditorWorldEdits, PropsAreAddedEditedAndRemovedByIndex) {
    auto d = doc_of(support::sample_world());
    const std::string before = support::world_text_of(d.desc());
    const std::size_t n = d.desc().props.size();
    ASSERT_TRUE(ed::apply(d, ed::AddProp{prop_of("crate")}).has_value());
    spade::PropDesc moved = prop_of("crate");
    moved.pose.position = glm::vec3(2.0f, 0.0f, 0.0f);
    ASSERT_TRUE(ed::apply(d, ed::SetProp{n, moved}).has_value());
    EXPECT_EQ(d.desc().props[n].pose.position, moved.pose.position);
    EXPECT_FALSE(ed::apply(d, ed::SetProp{n, prop_of("crate", 99)}).has_value());  // no material 99
    const Result<void> past = ed::apply(d, ed::RemoveProp{n + 1});
    ASSERT_FALSE(past.has_value());
    EXPECT_NE(past.error().context.find("prop"), std::string::npos) << past.error().context;
    ASSERT_TRUE(ed::apply(d, ed::RemoveProp{n}).has_value());
    EXPECT_EQ(d.desc().props.size(), n);
    for (int i = 0; i < 3; ++i) ASSERT_TRUE(d.undo());
    EXPECT_EQ(support::world_text_of(d.desc()), before);
}
