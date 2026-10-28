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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "../sandbox/editor_run.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "../sandbox/editor_world_edits.hpp"
#include "editor_test_support.hpp"
#include "physics/blade_azimuth.hpp"
#include "scene/compose.hpp"
#include "scene/model_file.hpp"
#include "vehicles/propulsion_steady.hpp"
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
    // The hover scene starts in trim and barely moves, so a restart would pass
    // for a carry; launch it instead, so the carried state is far from its start.
    ed::SceneDocument scene = ed::make_scene_document(support::sample_scene(), {});
    spade::VehicleSpawn launched = scene.desc().vehicles[0].start;
    launched.vel = glm::vec3(1.0f, 0.5f, -0.5f);
    launched.omega_body = glm::vec3(0.2f, 0.1f, 0.0f);
    ASSERT_TRUE(ed::apply(scene, ed::SetVehicleStart{"hover_quad_0", launched}).has_value());
    Result<ed::EditorRun> started =
        ed::EditorRun::start(scene.desc(), support::sample_world(), ed::records_for(support::hover_scene()));
    ASSERT_TRUE(started.has_value()) << started.error().context;
    ed::EditorRun run = std::move(*started);
    ASSERT_TRUE(run.step(200).has_value());
    const std::optional<spade::FrameState> before = run.state_of("hover_quad_0");
    ASSERT_TRUE(before.has_value());
    ASSERT_GT(glm::length(before->pos - launched.pos), 0.3f);  // a restart would put it back here
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
    const spade::VehicleRef ref = run.vehicle_ref("hover_quad_0").value();
    float mean = 0.0f;
    for (uint32_t i = 0; i < ref.rotor_count; ++i) mean += run.sim().rotor(ref, i).value()->omega;
    mean /= static_cast<float>(ref.rotor_count);
    // The edited scene starts the rotors elsewhere, so only a carry keeps them at the running speed.
    ed::SceneDocument scene = ed::make_scene_document(support::sample_scene(), {});
    spade::VehicleSpawn slower = scene.desc().vehicles[0].start;
    slower.rotor_omega = 0.5f * mean;
    ASSERT_TRUE(ed::apply(scene, ed::SetVehicleStart{"hover_quad_0", slower}).has_value());
    ASSERT_TRUE(run.rebuild(scene.desc(), support::sample_world()).has_value());
    const spade::VehicleRef now = run.vehicle_ref("hover_quad_0").value();
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

// Every rebuild makes the flown vehicles' controllers again from the scene as
// it is now, so the duty table each one builds once follows an edited chain
// (the lead's condition, 2026-10-22). No edit changes a chain yet; a motor or
// part edit will rebuild the run, as every edit while running does.
TEST(EditorRun, ARebuildFliesTheChainAsEdited) {
    spade::scene::SceneDesc s = support::sample_scene();
    const Result<spade::scene::ModelFile> file = spade::scene::load_model_file(
        std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml");
    ASSERT_TRUE(file.has_value()) << file.error().context;
    s.models.push_back(file->model);
    spade::scene::SceneVehicle v;
    v.name = "quad";
    v.model = file->model.name;
    v.start.pos = glm::vec3(4.0f, 1.0f, 4.0f);
    s.vehicles.push_back(v);
    const spade::WorldDesc world = support::sample_world();
    Result<ed::EditorRun> run = ed::EditorRun::start(s, world, ed::records_for(support::hover_scene()));
    ASSERT_TRUE(run.has_value()) << run.error().context;

    const double density = world.environment.air_density;
    const double soc = spade::analysis::kDefaultStateOfCharge;
    const auto exact = [&](const spade::vehicles::PropulsionChain& c, double thrust) {
        return spade::vehicles::steady_state_for_thrust(c, thrust, density, 0.0, soc).duty;
    };
    const spade::vehicles::PropulsionChain before = *file->model.propulsion;
    const double thrust = 0.5 * spade::vehicles::steady_state_at_duty(before, 1.0, density, 0.0, soc).thrust;
    const spade::sandbox::flight::FlightController* fc = run->controller_of("quad");
    ASSERT_NE(fc, nullptr) << run->why_not("quad");
    EXPECT_NEAR(fc->duty_for_thrust(thrust), exact(before, thrust), 1e-5);

    spade::scene::SceneDesc edited = s;
    spade::vehicles::PropulsionChain& after = *edited.models.back().propulsion;
    after.kv *= 1.25;  // another motor
    ASSERT_TRUE(run->rebuild(edited, world).has_value());
    fc = run->controller_of("quad");
    ASSERT_NE(fc, nullptr) << run->why_not("quad");
    EXPECT_GT(std::abs(exact(after, thrust) - exact(before, thrust)), 1e-3) << "the edit moves the inverse";
    EXPECT_NEAR(fc->duty_for_thrust(thrust), exact(after, thrust), 1e-5);
}

namespace {

// The hover scene with the library's drone from `stem`.model.yaml placed as
// `name` at `at`.
[[nodiscard]] spade::scene::SceneDesc with_library_drone(spade::scene::SceneDesc s, std::string_view stem,
                                                         std::string_view name, glm::vec3 at) {
    const Result<spade::scene::ModelFile> file = spade::scene::load_model_file(
        std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / (std::string(stem) + ".model.yaml"));
    EXPECT_TRUE(file.has_value()) << (file ? "" : file.error().context);
    if (!file) return s;
    s.models.push_back(file->model);
    spade::scene::SceneVehicle v;
    v.name = std::string(name);
    v.model = file->model.name;
    v.start.pos = at;
    s.vehicles.push_back(v);
    return s;
}

}  // namespace

// A scene whose drones all fit the records' cell (the reference quad's proxy
// sphere is 0.150 m against the hover records' 0.5 m) runs on the records'
// own instance, bit for bit as before Run sized its cell (the lead,
// 2026-10-27): no cell sized, and the digest is the records' instance's at
// the start and 200 ticks on (the quad lands), its rotors commanded as Run
// commands a disarmed drone's, zero.
TEST(EditorRun, ARunWhoseDronesFitTheRecordsCellIsBitForBitTheRecords) {
    const spade::scene::SceneDesc s =
        with_library_drone(support::sample_scene(), "reference_quad", "quad", glm::vec3(4.0f, 0.3f, 4.0f));
    const spade::WorldDesc world = support::sample_world();
    const ed::BuiltinRecords records = ed::records_for(support::hover_scene());
    Result<ed::EditorRun> run = ed::EditorRun::start(s, world, records);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_EQ(run->grid_cell(), records.grid.cell_size);
    EXPECT_TRUE(run->cell_sized_for().empty()) << run->cell_sized_for();

    // The records' instance, as Run built it before: chained drones at rest.
    spade::scene::SceneDesc at_rest = s;
    for (spade::scene::SceneVehicle& v : at_rest.vehicles) {
        if (run->flies(v.name)) v.start.rotor_omega = 0.0f;
    }
    const Result<spade::scene::ComposedScene> composed = ed::compose_together(at_rest, world);
    ASSERT_TRUE(composed.has_value()) << composed.error().context;
    auto direct = spade::scene::instantiate(*composed, spade::sandbox::instance_of(records),
                                            spade::sandbox::kBuiltinStepNs, spade::sandbox::kBuiltinSubsteps);
    ASSERT_TRUE(direct.has_value()) << direct.error().context;
    ASSERT_TRUE(direct->sim.flush_structural().has_value());
    EXPECT_EQ(spade::testing::state_digest(run->sim()), spade::testing::state_digest(direct->sim));
    for (int tick = 0; tick < 200; ++tick) {
        for (std::size_t i = 0; i < s.vehicles.size(); ++i) {
            if (!run->flies(s.vehicles[i].name)) continue;
            const spade::VehicleRef& ref = direct->vehicles[i];
            const std::vector<float> zero(ref.rotor_count, 0.0f);
            ASSERT_TRUE(direct->sim.set_rotor_commands(ref, zero).has_value());
        }
        ASSERT_TRUE(direct->sim.step(1).has_value());
    }
    ASSERT_TRUE(run->step(200).has_value());
    EXPECT_EQ(spade::testing::state_digest(run->sim()), spade::testing::state_digest(direct->sim));
}

// A drone too big for the records' cell sizes it (the lead, 2026-10-27):
// spawn() refuses a cell under twice a model's proxy radius, so the run's is
// twice the largest, and it names the drone it was sized for.
TEST(EditorRun, ARunSizesItsCellForItsLargestDroneNamingIt) {
    const spade::scene::SceneDesc s = with_library_drone(
        with_library_drone(support::sample_scene(), "reference_quad", "quad", glm::vec3(4.0f, 0.3f, 4.0f)),
        "phantom3_class", "phantom", glm::vec3(-4.0f, 0.4f, 4.0f));
    const ed::BuiltinRecords records = ed::records_for(support::hover_scene());
    Result<ed::EditorRun> run = ed::EditorRun::start(s, support::sample_world(), records);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const float proxy = s.models.back().proxy_radius;
    ASSERT_GT(2.0f * proxy, records.grid.cell_size) << "the Phantom needs more than the records' cell";
    EXPECT_EQ(run->grid_cell(), 2.0f * proxy);
    EXPECT_EQ(run->cell_sized_for(), "phantom");
    EXPECT_TRUE(run->state_of("phantom").has_value());
}

// The props spin (L7c2, Core's posing rule): each vehicle's blade azimuths
// start at 0 (the authored pose) and turn after every step by P11's rule,
// with the rows the step left and h the step's duration; a still rotor stays
// at 0; a rebuild carries them by name. The hover quads hold their spawn
// speed (no chain); the Phantom is flown, disarmed, its rotors at 0.
TEST(EditorRun, TheEditorRunTurnsEachPropByItsRotorsOmegaEachTick) {
    const spade::scene::SceneDesc s =
        with_library_drone(support::sample_scene(), "phantom3_class", "phantom", glm::vec3(-4.0f, 0.4f, 4.0f));
    Result<ed::EditorRun> run = ed::EditorRun::start(s, support::sample_world(), ed::records_for(support::hover_scene()));
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const spade::VehicleRef hover = run->vehicle_ref("hover_quad_0").value();
    ASSERT_GT(hover.rotor_count, 0u);
    std::vector<float> rule(hover.rotor_count, 0.0f);
    EXPECT_TRUE(std::ranges::equal(run->blade_azimuth("hover_quad_0"), rule)) << "the authored pose";
    const float h = run->sim().substep_h() * static_cast<float>(run->sim().substeps());
    std::vector<spade::vehicles::RotorRow> rows(hover.rotor_count);
    for (int t = 1; t <= 30; ++t) {
        ASSERT_TRUE(run->step(1).has_value());
        for (uint32_t r = 0; r < hover.rotor_count; ++r) rows[r] = *run->sim().rotor(hover, r).value();
        ASSERT_TRUE(spade::physics::advance_blade_azimuths(rule, rows, h));
        ASSERT_TRUE(std::ranges::equal(run->blade_azimuth("hover_quad_0"), rule)) << "tick " << t;
        const std::span<const float> omega = run->rotor_omega("hover_quad_0");
        ASSERT_EQ(omega.size(), rows.size());
        for (uint32_t r = 0; r < hover.rotor_count; ++r) EXPECT_EQ(omega[r], rows[r].omega) << "rotor " << r;
    }
    EXPECT_GT(rule.front(), 0.0f) << "the hover quad's rotors turn";
    const std::span<const float> phantom = run->blade_azimuth("phantom");
    ASSERT_EQ(phantom.size(), run->vehicle_ref("phantom")->rotor_count);
    EXPECT_TRUE(std::all_of(phantom.begin(), phantom.end(), [](float p) { return p == 0.0f; }))
        << "a still rotor stays at 0";
    EXPECT_TRUE(run->blade_azimuth("nobody").empty());
    EXPECT_TRUE(run->rotor_omega("nobody").empty());
    EXPECT_EQ(run->azimuth_refused(), "");

    ASSERT_TRUE(run->rebuild(s, support::sample_world()).has_value());
    EXPECT_TRUE(std::ranges::equal(run->blade_azimuth("hover_quad_0"), rule)) << "carried by name";
}

// Drawn only (L7c2): the azimuth is never read back. A Run that turns its
// props, stepping one at a time because its drones have rotors, is bit for
// bit the scene file's run stepped in one batch.
TEST(EditorRun, TheAzimuthIsNeverReadBack) {
    ed::EditorRun run = start_hover();
    const auto composed = spade::scene::compose_file(support::hover_scene());
    ASSERT_TRUE(composed.has_value());
    auto from_file = spade::scene::instantiate(*composed, spade::sandbox::instance_of(ed::records_for(support::hover_scene())),
                                               spade::sandbox::kBuiltinStepNs, spade::sandbox::kBuiltinSubsteps);
    ASSERT_TRUE(from_file.has_value()) << from_file.error().context;
    ASSERT_TRUE(run.step(120).has_value());
    ASSERT_TRUE(from_file->sim.step(120).has_value());
    ASSERT_FALSE(run.blade_azimuth("hover_quad_0").empty());
    EXPECT_GT(run.blade_azimuth("hover_quad_0").front(), 0.0f) << "the props turned";
    EXPECT_EQ(spade::testing::state_digest(run.sim()), spade::testing::state_digest(from_file->sim));
}

// A refusal never fails a step (the posing rule): an azimuth put outside
// [0, 2 pi) holds, the Run steps on with the vehicle's other props turning,
// and the Run names the first refusal, once, for the status line.
TEST(EditorRun, ARefusedAzimuthHoldsAndIsNamedOnceWithoutFailingAStep) {
    ed::EditorRun run = start_hover();
    const uint32_t rotors = run.vehicle_ref("hover_quad_0")->rotor_count;
    ASSERT_GE(rotors, 2u);
    std::vector<float> psi(rotors, 0.0f);
    psi.front() = spade::physics::kAzimuthTwoPi;
    run.set_test_blade_azimuth("hover_quad_0", psi);
    ASSERT_TRUE(run.step(2).has_value()) << "a refused azimuth never fails a step";
    EXPECT_EQ(run.blade_azimuth("hover_quad_0").front(), spade::physics::kAzimuthTwoPi) << "the refused rotor holds";
    EXPECT_GT(run.blade_azimuth("hover_quad_0").back(), 0.0f) << "the others turn";
    EXPECT_EQ(run.azimuth_refused(),
              "tick 1: a blade azimuth of 'hover_quad_0' was refused (advance_blade_azimuths()); it holds");
}

// Run's frame time on the Phantom scene (the lead, 2026-10-27): a frame is 4
// steps (60 fps of 4 ms steps), timed batched, one sim.step(4) as Run
// stepped a scene with no drone flown before the props turned, against one
// at a time, EditorRun::step(4) with each step's controllers and azimuths.
// Opt-in: SPADE_EDITOR_TIMINGS.
TEST(EditorRun, FrameTimeBatchedAgainstOneAtATimeOnThePhantomScene) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const bool opted = std::getenv("SPADE_EDITOR_TIMINGS") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (!opted) GTEST_SKIP() << "opt-in: SPADE_EDITOR_TIMINGS";
    const spade::scene::SceneDesc s = with_library_drone(support::sample_scene(), "phantom3_class", "phantom3_class",
                                                         glm::vec3(0.0f, 0.025f, 1.5f));
    const spade::WorldDesc world = support::sample_world();
    ed::BuiltinRecords records = ed::records_for(support::hover_scene());
    Result<ed::EditorRun> run = ed::EditorRun::start(s, world, records);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    records.grid.cell_size = run->grid_cell();
    const Result<spade::scene::ComposedScene> composed = ed::compose_together(s, world);
    ASSERT_TRUE(composed.has_value()) << composed.error().context;
    Result<spade::scene::SceneRun> batched =
        spade::scene::instantiate(*composed, spade::sandbox::instance_of(records), spade::sandbox::kBuiltinStepNs,
                                  spade::sandbox::kBuiltinSubsteps);
    ASSERT_TRUE(batched.has_value()) << batched.error().context;
    constexpr int kFrames = 600;
    constexpr uint32_t kStepsPerFrame = 4;
    using Clock = std::chrono::steady_clock;
    const Clock::time_point t0 = Clock::now();
    for (int f = 0; f < kFrames; ++f) ASSERT_TRUE(batched->sim.step(kStepsPerFrame).has_value());
    const Clock::time_point t1 = Clock::now();
    for (int f = 0; f < kFrames; ++f) ASSERT_TRUE(run->step(kStepsPerFrame, "phantom3_class").has_value());
    const Clock::time_point t2 = Clock::now();
    const double one_batch = std::chrono::duration<double, std::milli>(t1 - t0).count() / kFrames;
    const double one_at_a_time = std::chrono::duration<double, std::milli>(t2 - t1).count() / kFrames;
    std::printf("Run's frame (%u steps) on the Phantom scene: batched %.4f ms, one at a time %.4f ms (%d frames)\n",
                kStepsPerFrame, one_batch, one_at_a_time, kFrames);
    EXPECT_GT(run->blade_azimuth("hover_quad_0").front(), 0.0f);
}

// Core's review (2026-10-27): a rebuild that changes a vehicle's rotor count
// starts its props at 0, the authored pose, one per rotor of its new model;
// only a vehicle whose rotor count holds carries its azimuths.
TEST(EditorRun, ARebuildThatChangesARotorCountStartsItsPropsAtZero) {
    ed::EditorRun run = start_hover();
    ASSERT_TRUE(run.step(30).has_value());
    ASSERT_FALSE(run.blade_azimuth("hover_quad_0").empty());
    ASSERT_GT(run.blade_azimuth("hover_quad_0").front(), 0.0f) << "the props turned";

    spade::scene::SceneDesc s = support::sample_scene();
    const auto vehicle = std::find_if(s.vehicles.begin(), s.vehicles.end(),
                                      [](const spade::scene::SceneVehicle& v) { return v.name == "hover_quad_0"; });
    ASSERT_NE(vehicle, s.vehicles.end());
    const auto model = std::find_if(s.models.begin(), s.models.end(),
                                    [&](const spade::vehicles::ModelType& m) { return m.name == vehicle->model; });
    ASSERT_NE(model, s.models.end());
    const std::size_t rotors = model->rotors.size();
    for (std::size_t k = 0; k < 2; ++k) {
        auto rotor = model->rotors[k];
        rotor.local_pos *= 1.5f;
        model->rotors.push_back(rotor);
    }
    const Result<void> rebuilt = run.rebuild(s, support::sample_world());
    ASSERT_TRUE(rebuilt.has_value()) << rebuilt.error().context;
    const std::span<const float> after = run.blade_azimuth("hover_quad_0");
    ASSERT_EQ(after.size(), rotors + 2);
    EXPECT_TRUE(std::all_of(after.begin(), after.end(), [](float p) { return p == 0.0f; }))
        << "a changed rotor count starts at 0";
}
