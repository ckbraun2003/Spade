// The editor's files: open, save, save as (editor plan Task 5; EDT-005,
// EDT-006, EDT-015).
//
// A save writes a temporary file beside the target and renames it over the
// target, so a save that cannot complete leaves the old file as it was, no
// temporary file behind, and the document dirty (the plan's review focus 3).
// An unedited scene saves back byte for byte. A scene names its world by a
// path relative to itself, so Save As elsewhere rewrites that path. A dirty
// world is saved before its scene, which then pins the new world hash.

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "../sandbox/editor_files.hpp"
#include "../sandbox/editor_world_edits.hpp"
#include "editor_test_support.hpp"
#include "scene/compose.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
namespace fs = std::filesystem;
using spade::Result;

// The hover scene and its world, copied into `dir` as scenes/ and worlds/.
[[nodiscard]] fs::path hover_copy(const fs::path& dir) {
    return support::mirror({fs::path(SPADE_ASSETS_DIR), support::hover_scene()}, dir);
}

[[nodiscard]] spade::VehicleSpawn raised(const spade::scene::SceneDesc& s) {
    spade::VehicleSpawn start = s.vehicles.at(0).start;
    start.pos.y += 1.0f;
    return start;
}

}  // namespace

// EDT-005: open, then save unedited, reproduces every committed scene.
TEST(EditorFiles, EveryCommittedSceneSavesBackByteForByte) {
    const support::TempDir tmp;
    const auto files = support::committed_scene_files();
    ASSERT_GE(files.size(), 15u);
    for (const support::CommittedScene& c : files) {
        const fs::path copy = support::mirror(c, tmp.path());
        Result<ed::OpenedScene> opened = ed::open_scene(copy);
        ASSERT_TRUE(opened.has_value()) << c.file << ": " << opened.error().context;
        EXPECT_FALSE(opened->scene.dirty());
        EXPECT_FALSE(opened->world.dirty());
        ASSERT_TRUE(ed::save_scene(opened->scene, opened->world).has_value()) << c.file;
        EXPECT_EQ(support::read_bytes(copy), support::canonical_bytes(c.file)) << c.file;
    }
}

TEST(EditorFiles, OpeningASceneOpensItsWorldAndRemembersBothPaths) {
    Result<ed::OpenedScene> opened = ed::open_scene(support::hover_scene());
    ASSERT_TRUE(opened.has_value()) << opened.error().context;
    EXPECT_EQ(opened->scene.path, support::hover_scene());
    EXPECT_EQ(fs::weakly_canonical(opened->world.path), fs::weakly_canonical(support::hover_world()));
    EXPECT_EQ(support::world_text_of(opened->world.desc()), support::world_text_of(support::sample_world()));
}

TEST(EditorFiles, OpeningASceneWhoseWorldIsMissingNamesTheWorld) {
    const support::TempDir tmp;
    const fs::path lone = tmp.path() / "scenes" / "hover.scene.yaml";
    support::write_bytes(lone, support::read_bytes(support::hover_scene()));
    const Result<ed::OpenedScene> opened = ed::open_scene(lone);
    ASSERT_FALSE(opened.has_value());
    EXPECT_NE(opened.error().context.find("hover.world.yaml"), std::string::npos) << opened.error().context;
}

TEST(EditorFiles, ASavedEditIsOnDiskAndTheDocumentIsClean) {
    const support::TempDir tmp;
    const fs::path copy = hover_copy(tmp.path());
    Result<ed::OpenedScene> opened = ed::open_scene(copy);
    ASSERT_TRUE(opened.has_value());
    const spade::VehicleSpawn start = raised(opened->scene.desc());
    ASSERT_TRUE(ed::apply(opened->scene, ed::SetVehicleStart{"hover_quad_0", start}).has_value());
    EXPECT_TRUE(opened->scene.dirty());
    ASSERT_TRUE(ed::save_scene(opened->scene, opened->world).has_value());
    EXPECT_FALSE(opened->scene.dirty());
    const Result<spade::scene::SceneDesc> back = spade::scene::load_scene_file(copy);
    ASSERT_TRUE(back.has_value());
    EXPECT_EQ(back->vehicles.at(0).start.pos, start.pos);
    EXPECT_FALSE(fs::exists(fs::path(copy.string() + ".tmp-spade")));
}

// The editor plan's review focus 3, portably: the target is a folder, so the
// rename over it fails on every platform, after the temporary file was written.
TEST(EditorFiles, AFailedSaveLeavesTheTargetAndTheDocumentDirty) {
    const support::TempDir tmp;
    const fs::path copy = hover_copy(tmp.path());
    Result<ed::OpenedScene> opened = ed::open_scene(copy);
    ASSERT_TRUE(opened.has_value());
    ASSERT_TRUE(ed::apply(opened->scene, ed::SetVehicleStart{"hover_quad_0", raised(opened->scene.desc())}).has_value());
    const fs::path blocked = tmp.path() / "scenes" / "blocked.scene.yaml";
    support::write_bytes(blocked / "keep.txt", "old");
    const std::string before = support::text_of(opened->scene.desc());
    const Result<void> r = ed::save_scene_as(opened->scene, opened->world, blocked);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("blocked.scene.yaml"), std::string::npos) << r.error().context;
    EXPECT_EQ(support::read_bytes(blocked / "keep.txt"), "old");
    EXPECT_FALSE(fs::exists(fs::path(blocked.string() + ".tmp-spade")));
    EXPECT_TRUE(opened->scene.dirty());
    EXPECT_EQ(opened->scene.path, copy);
    EXPECT_EQ(support::text_of(opened->scene.desc()), before);
}

TEST(EditorFiles, ASaveIntoAMissingFolderFailsAndLeavesNothing) {
    const support::TempDir tmp;
    Result<ed::OpenedScene> opened = ed::open_scene(hover_copy(tmp.path()));
    ASSERT_TRUE(opened.has_value());
    const fs::path nowhere = tmp.path() / "no_such_folder" / "x.scene.yaml";
    const Result<void> r = ed::save_scene_as(opened->scene, opened->world, nowhere);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("x.scene.yaml"), std::string::npos) << r.error().context;
    EXPECT_FALSE(fs::exists(nowhere.parent_path()));
}

#ifdef _WIN32
// Review focus 3 as a user meets it on Windows: the file is read-only.
TEST(EditorFiles, ASaveOverAReadOnlyFileFailsAndLeavesItsBytes) {
    const support::TempDir tmp;
    const fs::path copy = hover_copy(tmp.path());
    Result<ed::OpenedScene> opened = ed::open_scene(copy);
    ASSERT_TRUE(opened.has_value());
    ASSERT_TRUE(ed::apply(opened->scene, ed::SetVehicleStart{"hover_quad_0", raised(opened->scene.desc())}).has_value());
    const std::string old_bytes = support::read_bytes(copy);
    support::make_read_only(copy);
    const Result<void> r = ed::save_scene(opened->scene, opened->world);
    support::make_writable(copy);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("hover.scene.yaml"), std::string::npos) << r.error().context;
    EXPECT_EQ(support::read_bytes(copy), old_bytes);
    EXPECT_FALSE(fs::exists(fs::path(copy.string() + ".tmp-spade")));
    EXPECT_TRUE(opened->scene.dirty());
}
#endif

TEST(EditorFiles, SaveAsElsewhereRewritesTheWorldPathSoItStillComposes) {
    const support::TempDir tmp;
    Result<ed::OpenedScene> opened = ed::open_scene(hover_copy(tmp.path()));
    ASSERT_TRUE(opened.has_value());
    const fs::path out = tmp.path() / "deeper" / "still" / "copy.scene.yaml";
    fs::create_directories(out.parent_path());
    ASSERT_TRUE(ed::save_scene_as(opened->scene, opened->world, out).has_value());
    EXPECT_EQ(opened->scene.desc().world.file, "../../worlds/hover.world.yaml");
    EXPECT_EQ(opened->scene.path, out);
    EXPECT_FALSE(opened->scene.dirty());
    const Result<spade::scene::ComposedScene> composed = spade::scene::compose_file(out);
    EXPECT_TRUE(composed.has_value()) << composed.error().context;
}

TEST(EditorFiles, SavingASceneSavesItsDirtyWorldFirstAndPinsItsNewHash) {
    const support::TempDir tmp;
    const fs::path copy = hover_copy(tmp.path());
    Result<ed::OpenedScene> opened = ed::open_scene(copy);
    ASSERT_TRUE(opened.has_value());
    spade::Environment env = opened->world.desc().environment;
    env.wind = glm::vec3(2.0f, 0.0f, 0.0f);
    ASSERT_TRUE(ed::apply(opened->world, ed::SetEnvironment{env}).has_value());
    ASSERT_TRUE(ed::save_scene(opened->scene, opened->world).has_value());
    EXPECT_FALSE(opened->world.dirty());
    EXPECT_FALSE(opened->scene.dirty());
    const Result<spade::WorldDesc> world_back = spade::load_world_file(opened->world.path);
    ASSERT_TRUE(world_back.has_value());
    EXPECT_EQ(world_back->environment.wind, env.wind);
    EXPECT_EQ(opened->scene.desc().world.hash, spade::scene::world_hash(*world_back).value());
    const Result<spade::scene::ComposedScene> composed = spade::scene::compose_file(copy);
    EXPECT_TRUE(composed.has_value()) << composed.error().context;
}

TEST(EditorFiles, AWorldWithNoFileIsSavedWithSaveAs) {
    const support::TempDir tmp;
    ed::WorldDocument w = ed::make_world_document(ed::new_world("field").value(), {});
    const Result<void> r = ed::save_world(w);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("Save As"), std::string::npos) << r.error().context;
    const fs::path out = tmp.path() / "field.world.yaml";
    ASSERT_TRUE(ed::save_world_as(w, out).has_value());
    EXPECT_EQ(w.path, out);
    EXPECT_FALSE(w.dirty());
    const Result<ed::WorldDocument> back = ed::open_world(out);
    ASSERT_TRUE(back.has_value());
    EXPECT_EQ(support::world_text_of(back->desc()), support::world_text_of(w.desc()));
}

TEST(EditorFiles, RelativeWorldPathsUseForwardSlashes) {
    EXPECT_EQ(ed::relative_world_path("C:/a/scenes", "C:/a/worlds/w.world.yaml").value(), "../worlds/w.world.yaml");
    EXPECT_EQ(ed::relative_world_path("C:/a", "C:/a/w.world.yaml").value(), "w.world.yaml");
    EXPECT_EQ(ed::relative_world_path("C:/a/b/c", "C:/a/w.world.yaml").value(), "../../w.world.yaml");
}
