// The scene follows its world (editor plan Task 6; EDT-016, EDT-017).
//
// Saving a world re-pins the open scene to the world's new hash, but only if
// the scene still composes with it; otherwise the world is saved, the scene
// keeps its pin, and compose()'s reason says why. The re-pin is an edit, so it
// can be undone. Other scenes in the project that name the world are listed,
// never changed, until the user re-pins one, which is refused if it would not
// compose. compose_conflict() reports what the open pair would hit if saved
// together, ignoring only the hash an unsaved world edit has not written yet.

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "../sandbox/editor_files.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "../sandbox/editor_world_edits.hpp"
#include "editor_test_support.hpp"
#include "scene/compose.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
namespace fs = std::filesystem;
using spade::Result;

[[nodiscard]] fs::path hover_copy(const fs::path& dir) {
    return support::mirror({fs::path(SPADE_ASSETS_DIR), support::hover_scene()}, dir);
}

[[nodiscard]] spade::Environment windy(const spade::WorldDesc& w) {
    spade::Environment env = w.environment;
    env.wind = glm::vec3(2.0f, 0.0f, 0.0f);
    return env;
}

[[nodiscard]] spade::MaterialDesc red() {
    spade::MaterialDesc m;
    m.name = "red";
    m.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    return m;
}

}  // namespace

TEST(EditorHashFollow, ASavedWorldEditRepinsTheOpenScene) {
    const support::TempDir tmp;
    const fs::path copy = hover_copy(tmp.path());
    Result<ed::OpenedScene> o = ed::open_scene(copy);
    ASSERT_TRUE(o.has_value());
    const uint64_t pinned = o->scene.desc().world.hash;
    ASSERT_TRUE(ed::apply(o->world, ed::SetEnvironment{windy(o->world.desc())}).has_value());
    const Result<ed::WorldSaved> r = ed::save_world_and_follow(o->world, o->scene, tmp.path());
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->old_hash, pinned);
    EXPECT_EQ(r->new_hash, spade::scene::world_hash(o->world.desc()).value());
    EXPECT_NE(r->new_hash, r->old_hash);
    EXPECT_TRUE(r->repinned);
    EXPECT_EQ(r->why_not, "");
    EXPECT_TRUE(r->others.empty());
    EXPECT_FALSE(o->world.dirty());
    EXPECT_EQ(o->scene.desc().world.hash, r->new_hash);
    EXPECT_TRUE(o->scene.dirty());  // re-pinned in the document; on disk once the scene is saved
    ASSERT_TRUE(ed::save_scene(o->scene, o->world).has_value());
    const auto composed = spade::scene::compose_file(copy);
    EXPECT_TRUE(composed.has_value()) << composed.error().context;
}

TEST(EditorHashFollow, TheRepinIsAnEditThatUndoes) {
    const support::TempDir tmp;
    Result<ed::OpenedScene> o = ed::open_scene(hover_copy(tmp.path()));
    ASSERT_TRUE(o.has_value());
    const uint64_t pinned = o->scene.desc().world.hash;
    ASSERT_TRUE(ed::apply(o->world, ed::SetEnvironment{windy(o->world.desc())}).has_value());
    ASSERT_TRUE(ed::save_world_and_follow(o->world, o->scene, tmp.path()).has_value());
    ASSERT_TRUE(o->scene.undo());
    EXPECT_EQ(o->scene.desc().world.hash, pinned);
    EXPECT_FALSE(o->scene.dirty());
}

TEST(EditorHashFollow, AWorldEditThatConflictsIsSavedButNotRepinned) {
    const support::TempDir tmp;
    const fs::path copy = hover_copy(tmp.path());
    Result<ed::OpenedScene> o = ed::open_scene(copy);
    ASSERT_TRUE(o.has_value());
    ASSERT_TRUE(ed::apply(o->scene, ed::AddMaterial{red()}).has_value());
    ASSERT_TRUE(ed::save_scene(o->scene, o->world).has_value());
    const uint64_t pinned = o->scene.desc().world.hash;
    ASSERT_TRUE(ed::apply(o->world, ed::AddWorldMaterial{red()}).has_value());
    const Result<ed::WorldSaved> r = ed::save_world_and_follow(o->world, o->scene, tmp.path());
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_FALSE(r->repinned);
    EXPECT_NE(r->why_not.find("'red'"), std::string::npos) << r->why_not;
    EXPECT_FALSE(o->world.dirty());  // the world is saved regardless
    EXPECT_EQ(o->scene.desc().world.hash, pinned);
    EXPECT_FALSE(o->scene.dirty());
    EXPECT_NE(ed::compose_conflict(o->scene, o->world).find("'red'"), std::string::npos);
}

TEST(EditorHashFollow, OtherScenesNamingTheWorldAreListedAndLeftAlone) {
    const support::TempDir tmp;
    const fs::path copy = hover_copy(tmp.path());
    const fs::path twin = tmp.path() / "scenes" / "twin.scene.yaml";
    support::write_bytes(twin, support::read_bytes(copy));
    const fs::path gate =
        support::mirror({fs::path(SPADE_ASSETS_DIR), fs::path(SPADE_ASSETS_DIR) / "scenes" / "gate.scene.yaml"},
                        tmp.path());  // names another world
    const std::string twin_before = support::read_bytes(twin);
    const std::string gate_before = support::read_bytes(gate);
    Result<ed::OpenedScene> o = ed::open_scene(copy);
    ASSERT_TRUE(o.has_value());
    ASSERT_TRUE(ed::apply(o->world, ed::SetEnvironment{windy(o->world.desc())}).has_value());
    const Result<ed::WorldSaved> r = ed::save_world_and_follow(o->world, o->scene, tmp.path());
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_EQ(r->others.size(), 1u);
    EXPECT_TRUE(fs::equivalent(r->others[0], twin)) << r->others[0];
    EXPECT_EQ(support::read_bytes(twin), twin_before);
    EXPECT_EQ(support::read_bytes(gate), gate_before);
    ASSERT_TRUE(ed::repin_scene_file(twin, o->world.desc()).has_value());
    const auto composed = spade::scene::compose_file(twin);
    EXPECT_TRUE(composed.has_value()) << composed.error().context;
}

TEST(EditorHashFollow, RepinningASceneThatWouldNotComposeIsRefusedAndLeavesIt) {
    const support::TempDir tmp;
    const fs::path copy = hover_copy(tmp.path());
    const fs::path twin = tmp.path() / "scenes" / "twin.scene.yaml";
    {
        Result<ed::OpenedScene> t = ed::open_scene(copy);
        ASSERT_TRUE(t.has_value());
        ASSERT_TRUE(ed::apply(t->scene, ed::AddMaterial{red()}).has_value());
        ASSERT_TRUE(ed::save_scene_as(t->scene, t->world, twin).has_value());
    }
    Result<ed::OpenedScene> o = ed::open_scene(copy);  // has no material "red"
    ASSERT_TRUE(o.has_value());
    ASSERT_TRUE(ed::apply(o->world, ed::AddWorldMaterial{red()}).has_value());
    const Result<ed::WorldSaved> r = ed::save_world_and_follow(o->world, o->scene, tmp.path());
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_TRUE(r->repinned);
    ASSERT_EQ(r->others.size(), 1u);
    const std::string before = support::read_bytes(twin);
    const Result<void> repin = ed::repin_scene_file(twin, o->world.desc());
    ASSERT_FALSE(repin.has_value());
    EXPECT_NE(repin.error().context.find("'red'"), std::string::npos) << repin.error().context;
    EXPECT_NE(repin.error().context.find("twin.scene.yaml"), std::string::npos) << repin.error().context;
    EXPECT_EQ(support::read_bytes(twin), before);
}

TEST(EditorHashFollow, AComposeConflictIgnoresOnlyTheUnsavedHash) {
    Result<ed::OpenedScene> o = ed::open_scene(support::hover_scene());
    ASSERT_TRUE(o.has_value());
    EXPECT_EQ(ed::compose_conflict(o->scene, o->world), "");
    ASSERT_TRUE(ed::apply(o->world, ed::SetEnvironment{windy(o->world.desc())}).has_value());
    EXPECT_EQ(ed::compose_conflict(o->scene, o->world), "");  // the pin is stale, but saving would re-pin it
    ASSERT_TRUE(ed::apply(o->scene, ed::AddMaterial{red()}).has_value());
    ASSERT_TRUE(ed::apply(o->world, ed::AddWorldMaterial{red()}).has_value());
    EXPECT_NE(ed::compose_conflict(o->scene, o->world).find("'red'"), std::string::npos);
}

TEST(EditorHashFollow, AWorldWithNoFileCannotBeFollowed) {
    ed::WorldDocument w = ed::make_world_document(ed::new_world("field").value(), {});
    ed::SceneDocument s = ed::make_scene_document(support::sample_scene(), {});
    const Result<ed::WorldSaved> r = ed::save_world_and_follow(w, s, fs::temp_directory_path());
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("Save As"), std::string::npos) << r.error().context;
}
