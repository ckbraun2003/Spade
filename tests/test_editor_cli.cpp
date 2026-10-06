// The editor's command line: spade_sandbox --edit (editor plan Task 8;
// EDT-002, the design's §11 "Headless").
//
//   spade_sandbox --edit <scene.yaml> [--apply <edits.txt>] [--save <scene.yaml>]
//                 [--save-world <world.yaml>] [--run <steps>]
//
// It opens a scene and its world, applies an edits file (one edit per line,
// "#" comments and blank lines skipped), saves, and runs, with no window.
// Exit codes: 0 done; 1 an edit, a save or the run was refused (the message
// names the file and line); 2 bad arguments. main() hands every argument
// after --edit to edit_cli(), so these tests run the command line's own code.
// tests/CMakeLists.txt also runs the real executable once (EditorCli.*).

#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "../sandbox/editor_cli.hpp"
#include "editor_test_support.hpp"
#include "scene/compose.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
namespace fs = std::filesystem;
using spade::Result;

struct Ran {
    int code = -1;
    std::string out;
    std::string err;
};

[[nodiscard]] Ran run(const std::vector<std::string>& args) {
    std::ostringstream out;
    std::ostringstream err;
    Ran r;
    r.code = ed::edit_cli(args, out, err);
    r.out = out.str();
    r.err = err.str();
    return r;
}

[[nodiscard]] std::string windy_line(const spade::WorldDesc& w) {
    spade::Environment env = w.environment;
    env.wind = glm::vec3(2.0f, 0.0f, 0.0f);
    return ed::format_world_edit(ed::SetEnvironment{env}).value();
}

}  // namespace

TEST(EditorCli, ArgumentsParseAndBadOnesAreNamed) {
    const Result<ed::EditCliArgs> a = ed::parse_edit_cli({"a.scene.yaml", "--apply", "e.txt", "--save", "o.scene.yaml",
                                                          "--save-world", "o.world.yaml", "--run", "10"});
    ASSERT_TRUE(a.has_value()) << a.error().context;
    EXPECT_EQ(a->scene, fs::path("a.scene.yaml"));
    EXPECT_EQ(a->apply, fs::path("e.txt"));
    EXPECT_EQ(a->save, fs::path("o.scene.yaml"));
    EXPECT_EQ(a->save_world, fs::path("o.world.yaml"));
    EXPECT_EQ(a->run, 10u);
    EXPECT_FALSE(ed::parse_edit_cli({}).has_value());
    const auto unknown = ed::parse_edit_cli({"a.scene.yaml", "--frob"});
    ASSERT_FALSE(unknown.has_value());
    EXPECT_NE(unknown.error().context.find("--frob"), std::string::npos) << unknown.error().context;
    const auto bad_run = ed::parse_edit_cli({"a.scene.yaml", "--run", "ten"});
    ASSERT_FALSE(bad_run.has_value());
    EXPECT_NE(bad_run.error().context.find("ten"), std::string::npos) << bad_run.error().context;
    EXPECT_FALSE(ed::parse_edit_cli({"a.scene.yaml", "--run"}).has_value());
}

TEST(EditorCli, BadArgumentsExitTwo) {
    const Ran r = run({"a.scene.yaml", "--frob"});
    EXPECT_EQ(r.code, 2);
    EXPECT_NE(r.err.find("--frob"), std::string::npos) << r.err;
}

TEST(EditorCli, ItEditsSavesAndRunsAScene) {
    const support::TempDir tmp;
    const fs::path copy = support::mirror({fs::path(SPADE_ASSETS_DIR), support::hover_scene()}, tmp.path());
    const fs::path edits = tmp.path() / "edits.txt";
    support::write_bytes(edits, "# two edits\n" + ed::format_edit(ed::AddAsset{support::asset_named("crate")}).value() +
                                    "\n\n" + windy_line(support::sample_world()) + "\r\n");
    const fs::path out = tmp.path() / "out" / "edited.scene.yaml";
    fs::create_directories(out.parent_path());
    const Ran r = run({copy.string(), "--apply", edits.string(), "--save", out.string(), "--run", "20"});
    ASSERT_EQ(r.code, 0) << r.err;
    EXPECT_NE(r.out.find("done: applied 2 edits, ran 20 steps"), std::string::npos) << r.out;
    const auto composed = spade::scene::compose_file(out);
    ASSERT_TRUE(composed.has_value()) << composed.error().context;
    // No --save-world, so the edited world was saved where it was opened.
    const auto world = spade::load_world_file(tmp.path() / "worlds" / "hover.world.yaml");
    ASSERT_TRUE(world.has_value());
    EXPECT_EQ(world->environment.wind, glm::vec3(2.0f, 0.0f, 0.0f));
}

TEST(EditorCli, SaveWorldWritesTheWorldElsewhereAndTheSceneNamesIt) {
    const support::TempDir tmp;
    const fs::path copy = support::mirror({fs::path(SPADE_ASSETS_DIR), support::hover_scene()}, tmp.path());
    const std::string world_before = support::read_bytes(tmp.path() / "worlds" / "hover.world.yaml");
    const fs::path edits = tmp.path() / "edits.txt";
    support::write_bytes(edits, windy_line(support::sample_world()) + "\n");
    const fs::path new_world = tmp.path() / "worlds" / "windy.world.yaml";
    const fs::path out = tmp.path() / "scenes" / "windy.scene.yaml";
    const Ran r = run({copy.string(), "--apply", edits.string(), "--save-world", new_world.string(), "--save",
                       out.string()});
    ASSERT_EQ(r.code, 0) << r.err;
    EXPECT_EQ(support::read_bytes(tmp.path() / "worlds" / "hover.world.yaml"), world_before);
    const auto saved = spade::scene::load_scene_file(out);
    ASSERT_TRUE(saved.has_value());
    EXPECT_EQ(saved->world.file, "../worlds/windy.world.yaml");
    EXPECT_TRUE(spade::scene::compose_file(out).has_value());
}

TEST(EditorCli, ARefusedEditExitsOneNamingItsLineAndSavesNothing) {
    const support::TempDir tmp;
    const fs::path copy = support::mirror({fs::path(SPADE_ASSETS_DIR), support::hover_scene()}, tmp.path());
    const std::string world_before = support::read_bytes(tmp.path() / "worlds" / "hover.world.yaml");
    const fs::path edits = tmp.path() / "edits.txt";
    support::write_bytes(edits, windy_line(support::sample_world()) + "\nscene remove ghost\n");
    const fs::path out = tmp.path() / "out.scene.yaml";
    const Ran r = run({copy.string(), "--apply", edits.string(), "--save", out.string(), "--run", "5"});
    EXPECT_EQ(r.code, 1);
    EXPECT_NE(r.err.find("edits.txt:2:"), std::string::npos) << r.err;
    EXPECT_NE(r.err.find("'ghost'"), std::string::npos) << r.err;
    EXPECT_FALSE(fs::exists(out));
    EXPECT_EQ(support::read_bytes(tmp.path() / "worlds" / "hover.world.yaml"), world_before);
}

TEST(EditorCli, AMalformedLineExitsOneNamingItsLine) {
    const support::TempDir tmp;
    const fs::path edits = tmp.path() / "edits.txt";
    support::write_bytes(edits, "# fine\nscene set-asset-pose\n");
    const Ran r = run({support::hover_scene().string(), "--apply", edits.string()});
    EXPECT_EQ(r.code, 1);
    EXPECT_NE(r.err.find("edits.txt:2:"), std::string::npos) << r.err;
}

TEST(EditorCli, AMissingSceneExitsOne) {
    const Ran r = run({"no_such.scene.yaml"});
    EXPECT_EQ(r.code, 1);
    EXPECT_NE(r.err.find("no_such.scene.yaml"), std::string::npos) << r.err;
}
