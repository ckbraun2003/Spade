// The editor's workspace: what the window's panels ask for, applied to the
// documents and the run (editor plan Task 10, step c). Display-free.
//
// The panels only read and queue (INT-2): an edit, a toolbar command, a click
// in the view. The workspace decides. A refusal changes nothing and is shown
// verbatim in the status line.

#include <gtest/gtest.h>

#include <algorithm>
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
        if (library[i].name == "box") w->panels().armed = i;
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
        if (w->panels().library[i].name == "box") w->panels().armed = i;
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
        if (w->panels().library[i].name == "sphere") w->panels().armed = i;
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
