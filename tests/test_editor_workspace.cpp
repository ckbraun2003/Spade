// The editor's workspace: what the window's panels ask for, applied to the
// documents and the run (editor plan Task 10, step c). Display-free.
//
// The panels only read and queue (INT-2): an edit, a toolbar command, a click
// in the view. The workspace decides. A refusal changes nothing and is shown
// verbatim in the status line.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include <glm/glm.hpp>

#include "../sandbox/editor_workspace.hpp"
#include "editor_test_support.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using ed::Command;
using ed::Selection;

const glm::vec3 kDown(0.0f, -1.0f, 0.0f);
const glm::vec3 kForward(0.0f, 0.0f, -1.0f);

[[nodiscard]] std::unique_ptr<ed::EditorWorkspace> hover() {
    spade::Result<ed::OpenedScene> opened = ed::open_scene(support::hover_scene());
    EXPECT_TRUE(opened.has_value()) << (opened ? "" : opened.error().context);
    spade::Result<std::unique_ptr<ed::EditorWorkspace>> w = ed::EditorWorkspace::create(std::move(*opened));
    EXPECT_TRUE(w.has_value()) << (w ? "" : w.error().context);
    return w ? std::move(*w) : nullptr;
}

[[nodiscard]] const spade::scene::SceneDesc& scene_of(const ed::EditorWorkspace& w) {
    return w.documents().scene.desc();
}

[[nodiscard]] glm::vec3 quad_start(const ed::EditorWorkspace& w) { return scene_of(w).vehicles.at(0).start.pos; }

void command(ed::EditorWorkspace& w, Command c) {
    w.panels().commands.push_back(c);
    w.apply_panels();
}

[[nodiscard]] Selection object(std::string name) { return Selection{Selection::Kind::object, std::move(name)}; }

}  // namespace

TEST(EditorWorkspace, TheHierarchyIsTheSceneTheWorldThenTheObjects) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    w->refresh_panels();
    const auto& rows = w->panels().hierarchy;
    ASSERT_EQ(rows.size(), 3u);
    EXPECT_EQ(rows[0].target.kind, Selection::Kind::scene);
    EXPECT_EQ(rows[1].target.kind, Selection::Kind::world);
    EXPECT_EQ(rows[2].target, object("hover_quad_0"));
}

TEST(EditorWorkspace, AQueuedEditIsAppliedOnceAndARefusalIsShownVerbatim) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    spade::VehicleSpawn moved = scene_of(*w).vehicles.at(0).start;
    moved.pos.x += 1.0f;
    w->panels().pending.push_back(ed::SetVehicleStart{"hover_quad_0", moved});
    w->apply_panels();
    EXPECT_EQ(scene_of(*w).vehicles.at(0).start.pos, moved.pos);
    EXPECT_TRUE(w->panels().pending.empty());
    EXPECT_TRUE(w->panels().status.empty()) << w->panels().status;

    const std::string before = support::text_of(scene_of(*w));
    w->panels().pending.push_back(ed::RemoveObject{"nobody"});
    w->apply_panels();
    EXPECT_NE(w->panels().status.find("'nobody'"), std::string::npos) << w->panels().status;
    EXPECT_EQ(support::text_of(scene_of(*w)), before);
}

TEST(EditorWorkspace, AClickSelectsWhatIsUnderItAndTheGroundClearsIt) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    w->click(quad_start(*w) + glm::vec3(0.0f, 0.0f, 6.0f), kForward);
    EXPECT_EQ(w->panels().selection, object("hover_quad_0"));
    w->click(glm::vec3(-10.0f, 5.0f, -10.0f), kDown);
    EXPECT_EQ(w->panels().selection.kind, Selection::Kind::none);
}

TEST(EditorWorkspace, AnArmedItemIsPlacedRestingWhereTheClickMeetsTheGround) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    w->refresh_panels();
    const auto& library = w->panels().library;
    for (std::size_t i = 0; i < library.size(); ++i) {
        if (library[i].name == "Box") w->panels().armed = i;
    }
    ASSERT_TRUE(w->panels().armed.has_value());
    w->click(glm::vec3(4.0f, 5.0f, 4.0f), kDown);
    ASSERT_EQ(scene_of(*w).assets.size(), 1u);
    const glm::vec3 at = scene_of(*w).assets[0].pose.position;
    EXPECT_NEAR(at.x, 4.0f, 1e-4f);
    EXPECT_NEAR(at.y, 0.5f, 1e-2f);  // a unit box rests on the ground at y = 0
    EXPECT_NEAR(at.z, 4.0f, 1e-4f);
    EXPECT_FALSE(w->panels().armed.has_value());
    EXPECT_EQ(w->panels().selection, object("box"));
    command(*w, Command::undo);
    EXPECT_TRUE(scene_of(*w).assets.empty());
}

TEST(EditorWorkspace, RunPauseStepAndStop) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_EQ(w->run()->tick(), 0u);
    w->advance(0.1f);  // 25 steps of 4 ms
    EXPECT_EQ(w->run()->tick(), 25u);
    command(*w, Command::pause);
    w->advance(0.1f);
    EXPECT_EQ(w->run()->tick(), 25u);
    command(*w, Command::step);
    EXPECT_GT(w->run()->tick(), 25u);
    command(*w, Command::stop);
    EXPECT_EQ(w->run(), nullptr);
}

TEST(EditorWorkspace, UndoAndRedoFollowTheOrderOfEditsAcrossSceneAndWorld) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    spade::Environment windy = w->documents().world.desc().environment;
    windy.wind = glm::vec3(2.0f, 0.0f, 0.0f);
    w->panels().pending_world.push_back(ed::SetEnvironment{windy});
    w->apply_panels();
    spade::VehicleSpawn moved = scene_of(*w).vehicles.at(0).start;
    const glm::vec3 start = moved.pos;
    moved.pos.x += 1.0f;
    w->panels().pending.push_back(ed::SetVehicleStart{"hover_quad_0", moved});
    w->apply_panels();

    command(*w, Command::undo);  // the scene edit, the later one
    EXPECT_EQ(scene_of(*w).vehicles.at(0).start.pos, start);
    EXPECT_EQ(w->documents().world.desc().environment.wind, windy.wind);
    command(*w, Command::undo);  // then the world's
    EXPECT_EQ(w->documents().world.desc().environment.wind, glm::vec3(0.0f));
    command(*w, Command::redo);
    EXPECT_EQ(w->documents().world.desc().environment.wind, windy.wind);
    EXPECT_EQ(scene_of(*w).vehicles.at(0).start.pos, start);
    command(*w, Command::redo);
    EXPECT_EQ(scene_of(*w).vehicles.at(0).start.pos, moved.pos);
}

TEST(EditorWorkspace, AnEditWhileRunningKeepsTheRunGoing) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    command(*w, Command::run);
    w->advance(0.2f);
    ASSERT_NE(w->run(), nullptr);
    const uint64_t tick = w->run()->tick();
    w->refresh_panels();
    for (std::size_t i = 0; i < w->panels().library.size(); ++i) {
        if (w->panels().library[i].name == "Box") w->panels().armed = i;
    }
    w->click(glm::vec3(3.0f, 5.0f, 3.0f), kDown);
    ASSERT_EQ(scene_of(*w).assets.size(), 1u);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_EQ(w->run()->tick(), tick);  // carried, not restarted (Task 7)
    EXPECT_TRUE(w->panels().status.empty()) << w->panels().status;
}

TEST(EditorWorkspace, DeleteRemovesTheSelectionAndClearsIt) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    w->click(quad_start(*w) + glm::vec3(0.0f, 0.0f, 6.0f), kForward);
    ASSERT_EQ(w->panels().selection, object("hover_quad_0"));
    w->key_delete();
    EXPECT_TRUE(scene_of(*w).vehicles.empty());
    EXPECT_EQ(w->panels().selection.kind, Selection::Kind::none);
}

TEST(EditorWorkspace, ASelectionThatUndoRemovesIsCleared) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    w->refresh_panels();
    for (std::size_t i = 0; i < w->panels().library.size(); ++i) {
        if (w->panels().library[i].name == "Sphere") w->panels().armed = i;
    }
    w->click(glm::vec3(2.0f, 5.0f, 2.0f), kDown);
    ASSERT_EQ(w->panels().selection, object("sphere"));
    command(*w, Command::undo);
    EXPECT_EQ(w->panels().selection.kind, Selection::Kind::none);
}

// F frames the selection: a vehicle within its reach, an asset within its box.
TEST(EditorWorkspace, TheSelectionsFramingSurroundsIt) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    EXPECT_FALSE(w->selection_framing().has_value());  // nothing selected
    w->click(quad_start(*w) + glm::vec3(0.0f, 0.0f, 6.0f), kForward);
    const std::optional<ed::Framing> drone = w->selection_framing();
    ASSERT_TRUE(drone.has_value());
    EXPECT_EQ(drone->centre, quad_start(*w));
    EXPECT_GT(drone->radius, 0.25f);  // its arms and rotors
    EXPECT_LT(drone->radius, 0.5f);
    w->panels().selection = Selection{Selection::Kind::world, {}};
    EXPECT_FALSE(w->selection_framing().has_value());
}

// The File menu's scene picker (Task 11): opened on a built-in scene, the
// workspace lists the built-in scenes, the open one among them.
TEST(EditorWorkspace, TheFileMenuListsTheBuiltInScenes) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const auto& sources = w->panels().sources;
    ASSERT_FALSE(sources.empty());
    const bool lists_hover = std::any_of(sources.begin(), sources.end(), [](const ed::SceneSource& s) {
        return s.name == "hover" && s.kind == ed::SceneSource::Kind::builtin;
    });
    EXPECT_TRUE(lists_hover);
}

// The gizmo's drag: however many poses one drag passes through, it is one
// undo step; the next drag is a step of its own.
TEST(EditorWorkspace, OneDragIsOneUndoStep) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::string quad = scene_of(*w).vehicles.at(0).name;
    const auto start = scene_of(*w).vehicles.at(0).start;
    w->panels().selection = object(quad);
    for (int i = 1; i <= 3; ++i) {
        auto moved = start;
        moved.pos.x += 0.1f * static_cast<float>(i);
        w->drag(ed::SetVehicleStart{quad, moved});
    }
    w->end_drag();
    const float dragged = quad_start(*w).x;
    EXPECT_NEAR(dragged, start.pos.x + 0.3f, 1e-6f);
    command(*w, Command::undo);
    EXPECT_EQ(quad_start(*w), start.pos);
    command(*w, Command::redo);
    EXPECT_EQ(quad_start(*w).x, dragged);

    auto again = start;
    again.pos.z += 1.0f;
    w->drag(ed::SetVehicleStart{quad, again});
    w->end_drag();
    command(*w, Command::undo);
    EXPECT_EQ(quad_start(*w).x, dragged);
    EXPECT_EQ(quad_start(*w).z, start.pos.z);
}

// The panel lists the default library, in its order (INT-14, Q3a): the
// built-in items left it.
TEST(EditorWorkspace, ItsLibraryIsTheDefaultLibrary) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const ed::LibraryLoad load =
        ed::load_library_items(std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "library.yaml");
    ASSERT_FALSE(load.items.empty());
    EXPECT_TRUE(w->panels().library_problems.empty()) << w->panels().library_problems.front();
    const auto& library = w->panels().library;
    ASSERT_EQ(library.size(), load.items.size());
    for (std::size_t i = 0; i < library.size(); ++i) {
        EXPECT_EQ(library[i].name, load.items[i].name);
        EXPECT_EQ(library[i].category, load.items[i].category);
    }
}

// Load from file adds the file's item to the panel and arms it, as an entry
// is armed; the click places it.
TEST(EditorWorkspace, LoadFromFileArmsTheFilesItem) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::size_t before = w->panels().library.size();
    w->panels().item_path =
        (std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "obstacles" / "gate.object.yaml").string();
    command(*w, Command::load_item);
    ASSERT_EQ(w->panels().library.size(), before + 1) << w->panels().status;
    ASSERT_TRUE(w->panels().armed.has_value());
    EXPECT_EQ(*w->panels().armed, before);
    EXPECT_EQ(w->panels().library.back().category, "from file");
    w->click(glm::vec3(4.0f, 5.0f, 4.0f), kDown);
    ASSERT_EQ(scene_of(*w).assets.size(), 1u) << w->panels().status;
    EXPECT_EQ(scene_of(*w).assets[0].name, "gate");
}

// Save to library (EDT-020): the selection goes into the project library,
// <the scene's folder>/library/, and the panel lists it after the default
// library's items, then and when the scene is opened again.
TEST(EditorWorkspace, SaveToLibrarySavesTheSelectionAndThePanelListsItAfterTheDefaultLibrary) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const support::TempDir tmp;
    const std::filesystem::path mine = tmp.path() / "hover.scene.yaml";
    w->panels().path = mine.string();
    command(*w, Command::save_as);
    ASSERT_TRUE(std::filesystem::exists(mine)) << w->panels().status;
    const std::size_t before = w->panels().library.size();
    w->panels().selection = object("hover_quad_0");
    command(*w, Command::save_to_library);
    ASSERT_EQ(w->panels().library.size(), before + 1) << w->panels().status;
    EXPECT_EQ(w->panels().library.back().name, "hover_quad_0");
    EXPECT_EQ(w->panels().library.back().category, "drones");
    EXPECT_TRUE(std::filesystem::exists(tmp.path() / "library" / "library.yaml"));
    EXPECT_NE(w->panels().status.find("hover_quad_0"), std::string::npos) << w->panels().status;

    w->panels().path = mine.string();
    command(*w, Command::open);
    ASSERT_EQ(w->panels().library.size(), before + 1) << "reopened: the default library, then the project's";
    EXPECT_EQ(w->panels().library.back().name, "hover_quad_0");
}

// In a built-in scene it is refused by name: the default library is never
// written. Without a selection there is nothing to save.
TEST(EditorWorkspace, SaveToLibraryInABuiltInSceneOrWithNothingSelectedIsRefusedByName) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::size_t before = w->panels().library.size();
    w->panels().selection = object("hover_quad_0");
    command(*w, Command::save_to_library);
    EXPECT_EQ(w->panels().library.size(), before);
    EXPECT_NE(w->panels().status.find("built-in"), std::string::npos) << w->panels().status;
    EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(SPADE_ASSETS_DIR) / "scenes" / "library"));

    w->panels().selection = Selection{};
    command(*w, Command::save_to_library);
    EXPECT_EQ(w->panels().library.size(), before);
    EXPECT_NE(w->panels().status.find("select"), std::string::npos) << w->panels().status;
}

// One that won't load is named in the status, and nothing is armed.
TEST(EditorWorkspace, LoadFromFileOfAnUnreadableFileIsNamed) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::size_t before = w->panels().library.size();
    w->panels().item_path = "no/such.object.yaml";
    command(*w, Command::load_item);
    EXPECT_EQ(w->panels().library.size(), before);
    EXPECT_FALSE(w->panels().armed.has_value());
    EXPECT_NE(w->panels().status.find("no/such.object.yaml"), std::string::npos) << w->panels().status;
}
