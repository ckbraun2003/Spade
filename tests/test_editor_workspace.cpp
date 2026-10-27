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
#include "../sandbox/pilot_keys.hpp"
#include "analysis/case.hpp"
#include "vehicles/propulsion_steady.hpp"
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

// ===========================================================================
// N17: THE PILOT. Run flies one drone through its chain (flight_controller.hpp)
// from the pilot's Controls (panels().pilot): the selection when Run starts if
// it is a drone with a chain, else the first such drone; every other chained
// drone sits disarmed, and a chainless one keeps its held spawn command. A
// chained drone starts disarmed, its rotors at exactly 0 (the lead's Q2, Q3).
// ===========================================================================

namespace {

// The library's reference quad placed resting at (x, 0, z); its vehicle's
// name (placing selects it).
[[nodiscard]] std::string place_reference_quad(ed::EditorWorkspace& w, float x, float z) {
    w.refresh_panels();
    for (std::size_t i = 0; i < w.panels().library.size(); ++i) {
        if (w.panels().library[i].name == "Reference quad (test fixture)") w.panels().armed = i;
    }
    EXPECT_TRUE(w.panels().armed.has_value());
    w.click(glm::vec3(x, 5.0f, z), kDown);
    return w.panels().selection.name;
}

[[nodiscard]] std::vector<float> rotor_speeds(const ed::EditorWorkspace& w, const std::string& vehicle) {
    std::vector<float> out;
    if (w.run() == nullptr) {
        ADD_FAILURE() << "not running";
        return out;
    }
    const std::optional<spade::VehicleRef> ref = w.run()->vehicle_ref(vehicle);
    if (!ref) {
        ADD_FAILURE() << "no vehicle " << vehicle;
        return out;
    }
    for (uint32_t i = 0; i < ref->rotor_count; ++i) {
        const spade::Result<const spade::vehicles::RotorRow*> row = w.run()->sim().rotor(*ref, i);
        EXPECT_TRUE(row.has_value());
        if (row) out.push_back((*row)->omega);
    }
    return out;
}

// The drone's idle speed: the arm rule's idle duty through its chain.
[[nodiscard]] float idle_speed(const ed::EditorWorkspace& w, const std::string& vehicle) {
    const spade::scene::SceneDesc& s = scene_of(w);
    std::string model;
    for (const spade::scene::SceneVehicle& v : s.vehicles) {
        if (v.name == vehicle) model = v.model;
    }
    const spade::vehicles::ModelType& m = support::model_named(s, model);
    if (!m.propulsion) {
        ADD_FAILURE() << vehicle << " has no chain";
        return 0.0f;
    }
    const spade::Result<spade::vehicles::DutyCommand> d = spade::vehicles::rotor_omega_for_duty(
        *m.propulsion, spade::analysis::kDefaultIdleDuty, w.documents().world.desc().environment.air_density,
        spade::analysis::kDefaultStateOfCharge);
    EXPECT_TRUE(d.has_value()) << (d ? "" : d.error().context);
    return d ? d->omega_cmd : 0.0f;
}

}  // namespace

TEST(EditorPilot, RunPilotsTheSelectedChainedDroneDisarmedAtRestAndArmingIdlesIt) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::string quad = place_reference_quad(*w, 4.0f, 4.0f);
    ASSERT_FALSE(quad.empty()) << w->panels().status;
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_EQ(w->panels().pilot.vehicle, quad);
    EXPECT_FALSE(w->panels().pilot.controls.armed);
    w->advance(0.5f);
    for (const float s : rotor_speeds(*w, quad)) EXPECT_EQ(s, 0.0f) << "disarmed, at rest";

    w->panels().pilot.controls.armed = true;
    w->advance(0.5f);
    const float idle = idle_speed(*w, quad);
    ASSERT_GT(idle, 0.0f);
    for (const float s : rotor_speeds(*w, quad)) EXPECT_GE(s, 0.9f * idle) << "idle " << idle;
}

// Nothing selected: the first chained drone; selected: that one. Either way
// every other chained drone stays disarmed when the pilot arms.
TEST(EditorPilot, ThePilotIsTheSelectionElseTheFirstChainedDroneAndTheOthersStayDisarmed) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::string first = place_reference_quad(*w, 4.0f, 4.0f);
    const std::string second = place_reference_quad(*w, -4.0f, 4.0f);
    ASSERT_NE(first, second);

    w->panels().selection = Selection{};
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_EQ(w->panels().pilot.vehicle, first);
    w->panels().pilot.controls.armed = true;
    w->advance(0.5f);
    for (const float s : rotor_speeds(*w, second)) EXPECT_EQ(s, 0.0f) << "not piloted: disarmed";
    for (const float s : rotor_speeds(*w, first)) EXPECT_GT(s, 0.0f);
    command(*w, Command::stop);
    EXPECT_FALSE(w->panels().pilot.controls.armed) << "stopping disarms";

    w->panels().selection = object(second);
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_EQ(w->panels().pilot.vehicle, second);
}

// The hover scene's own quad has no chain: nothing is piloted, the panel says
// why, and its rotors keep the speed it was spawned holding.
TEST(EditorPilot, AChainlessDroneIsNotPilotedAndKeepsItsHeldSpeed) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const float held = scene_of(*w).vehicles.at(0).start.rotor_omega;
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_TRUE(w->panels().pilot.vehicle.empty());
    EXPECT_NE(w->panels().pilot.why_not.find("propulsion chain"), std::string::npos) << w->panels().pilot.why_not;
    w->advance(0.5f);
    for (const float s : rotor_speeds(*w, "hover_quad_0")) EXPECT_NEAR(s, held, 1e-3f * std::max(1.0f, held));
}

// An edit while flying rebuilds the run but carries the drone: its rotors
// keep turning and the pilot stays armed (never zeroed as a fresh start is).
TEST(EditorPilot, AnEditWhileFlyingKeepsThePilotedDronesRotorsTurning) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::string quad = place_reference_quad(*w, 4.0f, 4.0f);
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    w->panels().pilot.controls.armed = true;
    w->advance(0.5f);
    for (std::size_t i = 0; i < w->panels().library.size(); ++i) {
        if (w->panels().library[i].name == "Box") w->panels().armed = i;
    }
    w->click(glm::vec3(-3.0f, 5.0f, -3.0f), kDown);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_TRUE(w->panels().pilot.controls.armed);
    EXPECT_EQ(w->panels().pilot.vehicle, quad);
    w->advance(0.04f);
    const float idle = idle_speed(*w, quad);
    for (const float s : rotor_speeds(*w, quad)) EXPECT_GE(s, 0.8f * idle) << "carried, not spun down";
}

// The pilot's keys act only while Run flies a drone: not before Run, and not
// when nothing is piloted (the window passes them only when ImGui doesn't
// have the keyboard).
TEST(EditorPilot, ThePilotsKeysActOnlyWhileRunFliesADrone) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    spade::sandbox::flight::PilotKeys space;
    space.arm_pressed = true;
    w->pilot_keys(space, 0.004f);
    EXPECT_FALSE(w->panels().pilot.controls.armed) << "not running";

    command(*w, Command::run);
    ASSERT_TRUE(w->panels().pilot.vehicle.empty());
    w->pilot_keys(space, 0.004f);
    EXPECT_FALSE(w->panels().pilot.controls.armed) << "nothing piloted";
    command(*w, Command::stop);

    const std::string quad = place_reference_quad(*w, 4.0f, 4.0f);
    command(*w, Command::run);
    ASSERT_EQ(w->panels().pilot.vehicle, quad);
    w->pilot_keys(space, 0.004f);
    EXPECT_TRUE(w->panels().pilot.controls.armed);
}

// The Pilot section's buttons: Arm toggles arming, Level levels the tilt.
TEST(EditorPilot, TheArmAndLevelCommandsActOnThePilot) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::string quad = place_reference_quad(*w, 4.0f, 4.0f);
    command(*w, Command::run);
    ASSERT_EQ(w->panels().pilot.vehicle, quad);
    command(*w, Command::arm);
    EXPECT_TRUE(w->panels().pilot.controls.armed);
    w->panels().pilot.controls.pitch_deg = -20.0f;
    w->panels().pilot.controls.roll_deg = 5.0f;
    command(*w, Command::level);
    EXPECT_EQ(w->panels().pilot.controls.pitch_deg, 0.0f);
    EXPECT_EQ(w->panels().pilot.controls.roll_deg, 0.0f);
    command(*w, Command::arm);
    EXPECT_FALSE(w->panels().pilot.controls.armed);
}

// INT-14's grades: an inspected library part stays in the inspector until
// anything in the scene is selected (the lead's Q1, 2026-10-22).
TEST(EditorWorkspace, ASelectionEndsAPartsInspection) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    std::optional<std::size_t> part;
    for (std::size_t i = 0; i < w->panels().library.size() && !part; ++i) {
        if (w->panels().library[i].kind == ed::LibraryItem::Kind::part) part = i;
    }
    ASSERT_TRUE(part.has_value()) << "the default library has parts (L4)";
    ASSERT_TRUE(ed::inspect_item(w->panels(), *part));
    w->apply_panels();
    EXPECT_EQ(w->panels().inspected, part);
    w->panels().selection = ed::Selection{ed::Selection::Kind::scene, ""};
    w->apply_panels();
    EXPECT_FALSE(w->panels().inspected.has_value());
    EXPECT_EQ(w->panels().selection.kind, ed::Selection::Kind::scene);
}

// A library drone with its sidecar (L5) is inspected as a part is, the
// inspection kept while nothing is selected and ended by a selection.
TEST(EditorWorkspace, ASelectionEndsADronesInspection) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    std::optional<std::size_t> drone;
    for (std::size_t i = 0; i < w->panels().library.size() && !drone; ++i) {
        if (w->panels().library[i].sources) drone = i;
    }
    ASSERT_TRUE(drone.has_value()) << "the default library's drones have sidecars (L5)";
    ASSERT_TRUE(ed::inspect_item(w->panels(), *drone));
    w->apply_panels();
    EXPECT_EQ(w->panels().inspected, drone);
    w->panels().selection = ed::Selection{ed::Selection::Kind::scene, ""};
    w->apply_panels();
    EXPECT_FALSE(w->panels().inspected.has_value());
}

// ===========================================================================
// T3 of the editor's airflow run (INT-15; the plan's §1, D1 and §9): "Run
// air" runs the selected drone's region on the worker, its drone held and
// flown by the Pilot section; Stop ends it. The rest of the scene is not
// simulated; a change to the scene stops the air, whose drone was placed
// where the scene had it at Run air.
// ===========================================================================

namespace {

// The reference quad placed and selected, with its airflow region.
[[nodiscard]] std::string quad_with_region(ed::EditorWorkspace& w) {
    const std::string quad = place_reference_quad(w, 3.0f, 0.0f);
    w.panels().pending.push_back(ed::AddAirflowRegion{quad});
    w.apply_panels();
    EXPECT_FALSE(scene_of(w).airflow.empty()) << w.panels().status;
    return quad;
}

}  // namespace

TEST(EditorWorkspaceAir, RunAirRunsTheSelectedDronesRegionAndStopEndsIt) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::string quad = quad_with_region(*w);
    command(*w, Command::run_air);
    ASSERT_NE(w->air(), nullptr) << w->panels().status;
    EXPECT_TRUE(w->panels().running);
    EXPECT_EQ(w->panels().pilot.vehicle, quad) << "the Pilot section flies the held drone";
    EXPECT_EQ(w->run(), nullptr) << "the rest of the scene is not simulated";
    command(*w, Command::stop);
    EXPECT_EQ(w->air(), nullptr);
    EXPECT_FALSE(w->panels().running);
}

TEST(EditorWorkspaceAir, RunAirWithoutARegionIsRefusedNamingWhatToDo) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    command(*w, Command::run_air);
    EXPECT_EQ(w->air(), nullptr);
    EXPECT_NE(w->panels().status.find("Add airflow region"), std::string::npos) << w->panels().status;
}

TEST(EditorWorkspaceAir, RunAndRunAirExcludeEachOther) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    (void)quad_with_region(*w);
    command(*w, Command::run_air);
    ASSERT_NE(w->air(), nullptr) << w->panels().status;
    command(*w, Command::run);
    EXPECT_EQ(w->run(), nullptr);
    EXPECT_NE(w->panels().status.find("Stop"), std::string::npos) << w->panels().status;
    command(*w, Command::stop);
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    command(*w, Command::run_air);
    EXPECT_EQ(w->air(), nullptr);
    EXPECT_NE(w->panels().status.find("Stop"), std::string::npos) << w->panels().status;
}

TEST(EditorWorkspaceAir, AChangeToTheSceneStopsTheAirNamingWhy) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    const std::string quad = quad_with_region(*w);
    command(*w, Command::run_air);
    ASSERT_NE(w->air(), nullptr) << w->panels().status;
    spade::VehicleSpawn moved = scene_of(*w).vehicles.back().start;
    moved.pos.x += 0.5f;
    w->panels().pending.push_back(ed::SetVehicleStart{quad, moved});
    w->apply_panels();
    EXPECT_EQ(w->air(), nullptr);
    EXPECT_NE(w->panels().status.find("the scene changed"), std::string::npos) << w->panels().status;
}

// Pause and Step are a Run's: while the air runs the bar has them off, and a
// Pause sent anyway leaves nothing paused (no "Resume" over air that runs on).
TEST(EditorWorkspaceAir, PauseAndStepAreARunsNotTheAirs) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    (void)quad_with_region(*w);
    command(*w, Command::run_air);
    ASSERT_NE(w->air(), nullptr) << w->panels().status;
    EXPECT_TRUE(w->panels().air);
    command(*w, Command::pause);
    EXPECT_FALSE(w->panels().paused);
    command(*w, Command::stop);
    EXPECT_FALSE(w->panels().air);
}

// Run flies a placed Phantom (the lead, 2026-10-27: the tour's editor
// chapters place it since L7c1 gave it props as mesh parts): Run starts, the
// Pilot section flies it, and the run has its state. The smoke at 5f3e43f
// found Run absent for it (editor.fly/piloted, add_drone's placed-at).
TEST(EditorWorkspaceRun, RunFliesAPlacedLibraryPhantom) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    w->refresh_panels();
    for (std::size_t i = 0; i < w->panels().library.size(); ++i) {
        if (w->panels().library[i].name == "Phantom 3-class quad") w->panels().armed = i;
    }
    ASSERT_TRUE(w->panels().armed.has_value());
    w->click(glm::vec3(3.0f, 5.0f, 0.0f), kDown);
    const std::string phantom = w->panels().selection.name;
    ASSERT_FALSE(phantom.empty()) << w->panels().status;
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_EQ(w->panels().pilot.vehicle, phantom) << w->panels().pilot.why_not;
    EXPECT_TRUE(w->run()->state_of(phantom).has_value());
}

// The inspector's Grid cell row is honest (the lead, 2026-10-27): while a Run
// whose cell was sized for a drone is going, it reads the run's cell and why;
// otherwise the records' cell, as before.
TEST(EditorWorkspaceRun, TheGridCellRowShowsTheRunsSizedCellAndWhy) {
    auto w = hover();
    ASSERT_NE(w, nullptr);
    w->refresh_panels();
    for (std::size_t i = 0; i < w->panels().library.size(); ++i) {
        if (w->panels().library[i].name == "Phantom 3-class quad") w->panels().armed = i;
    }
    ASSERT_TRUE(w->panels().armed.has_value());
    w->click(glm::vec3(3.0f, 5.0f, 0.0f), kDown);
    const std::string phantom = w->panels().selection.name;
    ASSERT_FALSE(phantom.empty()) << w->panels().status;
    const float records = w->records().grid.cell_size;
    const std::string before = std::format("{:.3f} m", records);
    EXPECT_EQ(w->panels().grid_cell, before);
    command(*w, Command::run);
    ASSERT_NE(w->run(), nullptr) << w->panels().status;
    EXPECT_EQ(w->panels().grid_cell, std::format("{:.3f} m, sized for '{}'; the records' {:.3f} m",
                                                 w->run()->grid_cell(), phantom, records));
    command(*w, Command::stop);
    EXPECT_EQ(w->panels().grid_cell, before);
}
