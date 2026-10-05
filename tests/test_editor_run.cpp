// The editor's run (editor plan Task 7; EDT-001, EDT-007, EDT-009, EDT-018).
//
// A run is built from the documents, never edited in place (EDT-001): start()
// composes the scene with its world -- re-pinned to the world's current hash,
// so unsaved world edits run -- and instantiates it with the scene's physics
// records. rebuild() does the same for edited documents and carries every
// vehicle that keeps its name: its design-frame pose, velocity and rates, and
// its rotors' mean speed (per-rotor in Task 16). A renamed vehicle starts
// fresh, a removed one is dropped, and a refused rebuild leaves the run as it
// was. The tick count continues across rebuilds.

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>

#include "../sandbox/editor_run.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "../sandbox/editor_world_edits.hpp"
#include "editor_test_support.hpp"
#include "scene/compose.hpp"
#include "testing/replay.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::Result;
using Element = spade::vehicles::ModelIssue::Element;

constexpr float kCarryTol = 1e-5f;

[[nodiscard]] ed::EditorRun start_hover() {
    Result<ed::EditorRun> run =
        ed::EditorRun::start(support::sample_scene(), support::sample_world(), ed::records_for(support::hover_scene()));
    EXPECT_TRUE(run.has_value()) << (run ? "" : run.error().context);
    return std::move(*run);
}

void expect_near(const spade::FrameState& a, const spade::FrameState& b, float tol) {
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(a.pos[i], b.pos[i], tol) << "pos " << i;
        EXPECT_NEAR(a.vel[i], b.vel[i], tol) << "vel " << i;
        EXPECT_NEAR(a.omega[i], b.omega[i], tol) << "omega " << i;
    }
    EXPECT_NEAR(std::abs(glm::dot(a.orient, b.orient)), 1.0f, tol);
}

}  // namespace

TEST(EditorRun, StartingFromTheDocumentsIsInstantiatingTheirFile) {
    ed::EditorRun run = start_hover();
    const auto composed = spade::scene::compose_file(support::hover_scene());
    ASSERT_TRUE(composed.has_value());
    const ed::BuiltinRecords records = ed::records_for(support::hover_scene());
    auto from_file = spade::scene::instantiate(*composed, spade::sandbox::instance_of(records),
                                               spade::sandbox::kBuiltinStepNs, spade::sandbox::kBuiltinSubsteps);
    ASSERT_TRUE(from_file.has_value()) << from_file.error().context;
    EXPECT_EQ(spade::testing::state_digest(run.sim()), spade::testing::state_digest(from_file->sim));
    ASSERT_TRUE(run.step(50).has_value());
    ASSERT_TRUE(from_file->sim.step(50).has_value());
    EXPECT_EQ(run.tick(), 50u);
    EXPECT_EQ(spade::testing::state_digest(run.sim()), spade::testing::state_digest(from_file->sim));
}

TEST(EditorRun, AnUnsavedWorldEditRuns) {
    ed::WorldDocument world = ed::make_world_document(support::sample_world(), {});
    spade::Environment env = world.desc().environment;
    env.wind = glm::vec3(3.0f, 0.0f, 0.0f);
    ASSERT_TRUE(ed::apply(world, ed::SetEnvironment{env}).has_value());  // the scene's pin is now stale
    const Result<ed::EditorRun> run =
        ed::EditorRun::start(support::sample_scene(), world.desc(), ed::records_for(support::hover_scene()));
    EXPECT_TRUE(run.has_value()) << (run ? "" : run.error().context);
}

TEST(EditorRun, ARebuildCarriesEveryVehicleThatKeepsItsName) {
    ed::EditorRun run = start_hover();
    ASSERT_TRUE(run.step(200).has_value());
    const std::optional<spade::FrameState> before = run.state_of("hover_quad_0");
    ASSERT_TRUE(before.has_value());
    ASSERT_GT(std::abs(before->pos.y - support::sample_scene().vehicles[0].start.pos.y), 1e-3f);  // it moved
    ed::SceneDocument scene = ed::make_scene_document(support::sample_scene(), {});
    ASSERT_TRUE(ed::apply(scene, ed::AddAsset{support::asset_named("box")}).has_value());
    ASSERT_TRUE(run.rebuild(scene.desc(), support::sample_world()).has_value());
    EXPECT_EQ(run.tick(), 200u);
    const std::optional<spade::FrameState> after = run.state_of("hover_quad_0");
    ASSERT_TRUE(after.has_value());
    expect_near(*after, *before, kCarryTol);
}

TEST(EditorRun, ARebuildCarriesTheRotorsMeanSpeed) {
    ed::EditorRun run = start_hover();
    ASSERT_TRUE(run.step(100).has_value());
    const spade::VehicleRef& ref = run.vehicle_ref("hover_quad_0").value();
    float mean = 0.0f;
    for (uint32_t i = 0; i < ref.rotor_count; ++i) mean += run.sim().rotor(ref, i).value()->omega;
    mean /= static_cast<float>(ref.rotor_count);
    ASSERT_TRUE(run.rebuild(support::sample_scene(), support::sample_world()).has_value());
    const spade::VehicleRef& now = run.vehicle_ref("hover_quad_0").value();
    for (uint32_t i = 0; i < now.rotor_count; ++i) {
        EXPECT_FLOAT_EQ(run.sim().rotor(now, i).value()->omega, mean) << "rotor " << i;
    }
}

TEST(EditorRun, ARefusedRebuildKeepsTheRun) {
    ed::EditorRun run = start_hover();
    ASSERT_TRUE(run.step(100).has_value());
    const uint64_t digest = spade::testing::state_digest(run.sim());
    spade::scene::SceneDesc clash = support::sample_scene();
    spade::MaterialDesc m;
    m.name = support::sample_world().materials.at(0).name;  // a scene material may not reuse a world name
    clash.materials.push_back(m);
    const Result<void> r = run.rebuild(clash, support::sample_world());
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("'" + m.name + "'"), std::string::npos) << r.error().context;
    EXPECT_EQ(run.tick(), 100u);
    EXPECT_EQ(spade::testing::state_digest(run.sim()), digest);
}

// The editor plan's review focus 5.
TEST(EditorRun, ARenamedVehicleStartsFreshAndARemovedOneIsDropped) {
    ed::EditorRun run = start_hover();
    ASSERT_TRUE(run.step(200).has_value());
    ed::SceneDocument scene = ed::make_scene_document(support::sample_scene(), {});
    ASSERT_TRUE(ed::apply(scene, ed::RenameObject{"hover_quad_0", "q"}).has_value());
    ASSERT_TRUE(run.rebuild(scene.desc(), support::sample_world()).has_value());
    EXPECT_FALSE(run.state_of("hover_quad_0").has_value());
    const std::optional<spade::FrameState> fresh = run.state_of("q");
    ASSERT_TRUE(fresh.has_value());
    const spade::VehicleSpawn& start = scene.desc().vehicles[0].start;
    expect_near(*fresh, spade::FrameState{start.pos, start.orient, start.vel, start.omega_body}, kCarryTol);
    ASSERT_TRUE(ed::apply(scene, ed::RemoveObject{"q"}).has_value());
    ASSERT_TRUE(run.rebuild(scene.desc(), support::sample_world()).has_value());
    EXPECT_FALSE(run.state_of("q").has_value());
    EXPECT_EQ(run.vehicle_count(), 0u);
}

TEST(EditorRun, ARemodelledVehicleKeepsItsPose) {
    ed::EditorRun run = start_hover();
    ASSERT_TRUE(run.step(200).has_value());
    const std::optional<spade::FrameState> before = run.state_of("hover_quad_0");
    ASSERT_TRUE(before.has_value());
    ed::SceneDocument scene = ed::make_scene_document(support::sample_scene(), {});
    ASSERT_TRUE(ed::apply(scene, ed::SetModelParam{"hover_quad_0", Element::rotor, 0, "radius", 0.15f}).has_value());
    ASSERT_TRUE(run.rebuild(scene.desc(), support::sample_world()).has_value());
    const std::optional<spade::FrameState> after = run.state_of("hover_quad_0");
    ASSERT_TRUE(after.has_value());
    expect_near(*after, *before, kCarryTol);
}

TEST(EditorRun, TheRecordsAreTheSceneFilesUntilWorldFileV3) {
    EXPECT_FALSE(ed::EditorRun::records_saved_with_world());
    const ed::BuiltinRecords hover = ed::records_for(support::hover_scene());
    EXPECT_EQ(hover.seed, spade::sandbox::builtin_records("hover")->seed);
    // A scene the editor saved has no built-in records; it runs on the default.
    const ed::BuiltinRecords other = ed::records_for("C:/elsewhere/my_new.scene.yaml");
    EXPECT_GE(other.grid.cell_size, 2.0f * other.contacts.proxy_radius);
    EXPECT_GT(other.grid.cell_size, 0.0f);
}
