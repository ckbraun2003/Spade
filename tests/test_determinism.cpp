#include <gtest/gtest.h>

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "physics/integrator.hpp"
#include "physics/schedule.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"
#include "testing/scenario_file.hpp"
#include "world/builder.hpp"

// ---------------------------------------------------------------------------
// Task 13 -- the pass schedule, Simulation, and the determinism replay corpus.
//
// FOUR BURDENS OF PROOF, carried by four different kinds of test because no one
// kind can carry all four:
//
//   * THE SCHEDULE IS THE SPEC'S SCHEDULE. Checked STRUCTURALLY, by comparing
//     the declared pass list against §3's eight names in order, and
//     BEHAVIOURALLY for the one pass whose correctness is an absence (Gravity
//     must not double-apply what Integrate already applies).
//
//   * THE ENGINE IS DETERMINISTIC. Checked by DIGEST: two runs of the same
//     scenario must produce the same 64-bit fold of every registered byte. A
//     digest is the only assertion that covers state nobody thought to look at.
//
//   * THE REPLAY GUARANTEE HOLDS (charter P2/P4). Checked by INTERRUPTION:
//     snapshot at tick k, restore into a FRESH Simulation, replay the same
//     input script from k, and land on the uninterrupted run's digest. This is
//     the test the whole state layer exists to make passable.
//
//   * THE ANSWER IS BATCH-INVARIANT (D8). Checked by SHAPE: a world stepped
//     alone and the same world stepped inside a four-world set must produce
//     identical per-world digests. This is what "cross-world interaction is
//     structurally impossible" means operationally.
//
// Plus the corpus itself: six scenarios, each a COMMITTED DATA FILE under
// tests/golden/scenarios/ carrying its own digest, re-asserted here, so that an
// unintended change to ANY pinned op order, rng construction, layout offset or
// schedule position fails a test in this file rather than surfacing as a parity
// mystery at S6.
//
// ---------------------------------------------------------------------------
// THE CORPUS IS DATA (S5 Task 7, spec Addendum A section 15)
//
// It was not always. S1-S4 built these scenarios as BUILDER LAMBDAS in this
// file and kept their digests in tests/golden/*.digest, and S5 Task 7 moved
// both into one artifact per scenario: engine/testing/scenario_file.hpp parses
// tests/golden/scenarios/<name>.scenario.yaml into exactly the replay.hpp
// Scenario the lambda used to return, and the digest lives in that file's
// `expected_digest` field. The .digest files are retired.
//
// THE SUBSTITUTION WAS ABSORBED, NOT ASSUMED. Before the lambdas were deleted
// they were run HEAD TO HEAD against the loaded data files, scenario by
// scenario, and every digest matched. That run's literal output is recorded in
// GoldenCorpus.TheDataScenariosReproduceTheRetiredBuilderCorpus below, which is
// the committed record that the S1-S4 builder corpus and the S5 data corpus are
// THE SAME CORPUS -- and which kept asserting those four numbers from a second,
// independently-spelled source, so an accidental edit to a scenario file's own
// `expected_digest` could not pass unnoticed. Since PHY-7 it asserts
// ballistic's alone; the other three left it (see its note).
// ---------------------------------------------------------------------------

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::Capacities;
using spade::DragElementSpawn;
using spade::Environment;
using spade::Simulation;
using spade::SnapshotBlob;
using spade::Tick;
using spade::TurbulenceLevel;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::testing::Scenario;

// ---------------------------------------------------------------------------
// GoogleTest plumbing for spade::Result. `ASSERT_OK(r)` on a failure prints the
// Error's context, which is the difference between "a test failed" and "the
// world set was rejected because world 2's cell_size is below the contact
// diameter".
// ---------------------------------------------------------------------------
template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code)
                                       << "] " << r.error().context;
}

#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// The error code of a Result that is EXPECTED to have failed, as an int, with
// -1 standing for "it succeeded". Calling .error() on a Result that holds a
// value is undefined behaviour, so a negative test that accidentally passes must
// not be allowed to reach for it.
template <class T>
[[nodiscard]] int code_of(const spade::Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}

[[nodiscard]] constexpr int code(spade::Code c) { return static_cast<int>(c); }

// ---------------------------------------------------------------------------
// Scenario building blocks
// ---------------------------------------------------------------------------

[[nodiscard]] Environment default_environment() {
    Environment env;
    env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    return env;
}

[[nodiscard]] Capacities capacities(uint32_t bodies, uint32_t elements) {
    Capacities caps;
    caps.bodies = bodies;
    caps.force_elements = elements;
    caps.sensors = 1;   // declared, unused until Task 19
    caps.contacts = 1;  // declared, unused: contacts are not an arena array in v1
    return caps;
}

[[nodiscard]] spade::physics::ContactParams contacts(float restitution, float mu, float proxy) {
    spade::physics::ContactParams c;
    c.restitution_e = restitution;
    c.friction_mu = mu;
    c.proxy_radius = proxy;
    return c;
}

[[nodiscard]] spade::physics::GridParams grid(float cell_size) {
    spade::physics::GridParams g;
    g.cell_size = cell_size;
    return g;
}

// A deterministic, transcendental-free per-tick input. Depends on the tick and
// the body's index only -- NEVER on the world index, because the
// batching-invariance test compares one world run at index 0 against the same
// world run at index 2 and any world-dependence would confound it.
[[nodiscard]] glm::vec3 scripted_force(uint64_t tick, uint32_t body_index) {
    const float a = static_cast<float>((tick + body_index) % 7u) - 3.0f;
    const float b = static_cast<float>((tick * 3u + body_index) % 5u) - 2.0f;
    return glm::vec3(a * 0.05f, 0.0f, b * 0.05f);
}

// ---------------------------------------------------------------------------
// THE STRUCTURAL-CHURN SCENARIO -- the one scenario still built in code, and
// the only one that should be. It is NOT a corpus member (its job is the queue,
// not a golden), and it is the one shape a scenario FILE cannot express: its
// input script SPAWNS AND DESPAWNS as it goes, on a schedule that depends on
// the live body count. A data format that could say that would be a language,
// which is exactly the line engine/testing/scenario_file.hpp's schema draws --
// so this stays a lambda, deliberately, rather than motivating a `script:` key.
//
// It is a full Scenario nonetheless, so it can be replayed from a resume point
// alongside the corpus in SnapshotRestoreIntoAFreshSimulationResumesIdentically.
//
// ITS INPUT SCRIPT IS A PURE FUNCTION OF (tick, current state) -- it looks bodies
// up by slot through Simulation::body_ref_at() rather than holding refs captured
// at setup. That is what makes it resume-safe: a ref captured before a snapshot
// would be stale after a restore rewound the generation counters.
// ---------------------------------------------------------------------------
[[nodiscard]] Scenario churn_scenario() {
    Scenario s;
    s.name = "structural_churn";
    s.dt_ns = 1'000'000;
    s.substeps = 1;
    s.steps = 220;

    s.build = []() -> spade::Result<WorldSetDesc> {
        const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                          .name("ground")
                                                          .environment(default_environment())
                                                          .capacities(capacities(12, 6))
                                                          .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                          .build();
        if (!world) return std::unexpected(world.error());

        WorldInstanceDesc prototype;
        prototype.world = *world;
        prototype.turbulence = spade::dryden_params(TurbulenceLevel::light);
        prototype.contacts = contacts(0.5f, 0.3f, 0.1f);
        prototype.grid = grid(0.2f);
        return spade::replicate(prototype, 2, 0xC40FFEEULL);
    };

    s.setup = [](Simulation& sim) -> spade::Result<void> {
        for (uint32_t w = 0; w < sim.world_count(); ++w) {
            for (uint32_t i = 0; i < 4; ++i) {
                BodySpawn body;
                body.pos = glm::vec3(0.3f * static_cast<float>(i), 1.5f, 0.0f);
                body.vel = glm::vec3(0.0f, 0.0f, 0.1f * static_cast<float>(i));
                body.mass = 0.4f;
                body.inv_inertia_diag = glm::vec3(250.0f);
                const spade::Result<BodyRef> ref = sim.spawn(w, body);
                if (!ref) return std::unexpected(ref.error());
                DragElementSpawn drag;
                drag.mode = spade::physics::drag_mode::componentwise;
                drag.coeffs = glm::vec3(0.02f, 0.03f, 0.02f);
                const spade::Result<spade::DragElementRef> elem = sim.add_drag_element(*ref, drag);
                if (!elem) return std::unexpected(elem.error());
            }
        }
        return {};
    };

    s.input = [](Simulation& sim, Tick tick) -> spade::Result<void> {
        for (uint32_t w = 0; w < sim.world_count(); ++w) {
            const spade::Result<uint32_t> live = sim.live_body_count(w);
            if (!live) return std::unexpected(live.error());

            if (tick.value % 13u == 0u && *live < 12u) {
                BodySpawn body;
                body.pos = glm::vec3(0.15f * static_cast<float>(tick.value % 11u), 2.0f, 0.4f);
                body.mass = 0.4f;
                body.inv_inertia_diag = glm::vec3(250.0f);
                const spade::Result<BodyRef> ref = sim.spawn(w, body);
                if (!ref) return std::unexpected(ref.error());
            }
            if (tick.value % 17u == 0u && *live > 2u) {
                const uint32_t local = static_cast<uint32_t>(tick.value % 12u);
                const spade::Result<BodyRef> victim = sim.body_ref_at(w, local);
                if (victim) {
                    if (spade::Result<void> r = sim.despawn(*victim); !r) return r;
                }
            }
            // A wrench on whatever occupies local slot 0, if anything does.
            const spade::Result<BodyRef> head = sim.body_ref_at(w, 0);
            if (head) {
                if (spade::Result<void> r =
                        sim.apply_wrench(*head, scripted_force(tick.value, 0), glm::vec3(0.0f));
                    !r) {
                    // A body spawned this same tick is still queued; skipping it
                    // is deterministic (it depends only on the tick), so this is
                    // a property of the script, not a source of divergence.
                    if (r.error().code != spade::Code::not_found) return r;
                }
            }
        }
        return {};
    };

    return s;
}

// ---------------------------------------------------------------------------
// The two-forms tripwire fixture (see the tests at the end of this file).
//
// A HETEROGENEOUS set -- the worlds disagree about restitution, so
// WorldSetLayout::uniform_dynamic_params is false and CollisionDynamic takes its
// PER-WORLD form -- whose worlds each hold a CLUSTER OF MUTUALLY CONTACTING
// bodies, so that form actually has pairs to resolve. That second property is
// what a single-body world cannot give: with one body per world the sorted grid
// finds no pair at all, and a divergence in the pair math would slip through a
// comparison of the two forms unnoticed.
//
// Spacing 0.15 m against a 0.1 m proxy radius puts every neighbour inside the
// 0.2 m contact diameter from the first substep, and cell_size == 2 * proxy is
// the cheapest correct grid setting (physics/grid.hpp).
// ---------------------------------------------------------------------------
[[nodiscard]] spade::Result<WorldSetDesc> contact_ladder_set(uint32_t world_count) {
    const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                      .name("ground")
                                                      .environment(default_environment())
                                                      .capacities(capacities(8, 2))
                                                      .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                      .build();
    if (!world) return std::unexpected(world.error());

    const float ladder[4] = {0.0f, 0.25f, 0.5f, 0.75f};
    WorldSetDesc set;
    for (uint32_t i = 0; i < world_count; ++i) {
        WorldInstanceDesc instance;
        instance.world = *world;
        instance.seed = 0xC1057E4ULL + i;
        instance.turbulence = spade::dryden_params(TurbulenceLevel::moderate);
        instance.contacts = contacts(ladder[i % 4u], 0.3f, 0.1f);
        instance.grid = grid(0.2f);
        set.worlds.push_back(instance);
    }
    return set;
}

// Six bodies per world, in a 3x2 lattice tight enough to be in contact on spawn.
// World-index independent, so a world spawned alone gets byte-identical bodies to
// the same world spawned inside a set.
[[nodiscard]] spade::Result<void> spawn_contact_cluster(Simulation& sim) {
    for (uint32_t w = 0; w < sim.world_count(); ++w) {
        for (uint32_t i = 0; i < 6; ++i) {
            BodySpawn body;
            body.pos = glm::vec3(0.15f * static_cast<float>(i % 3u),
                                 0.6f + 0.15f * static_cast<float>(i / 3u), 0.0f);
            body.mass = 0.5f;
            body.inv_inertia_diag = glm::vec3(200.0f);
            const spade::Result<BodyRef> ref = sim.spawn(w, body);
            if (!ref) return std::unexpected(ref.error());
        }
    }
    return {};
}

// How many pairs of live bodies in `world` are within the contact diameter.
// Used to prove the dynamic pass had work to do -- a tripwire on a cluster that
// silently drifted apart would be no tripwire at all.
[[nodiscard]] uint32_t contacting_pairs(const Simulation& sim, uint32_t world, float proxy_radius) {
    const spade::Result<std::span<const spade::BodyState>> bodies = sim.world_bodies(world);
    if (!bodies) return 0;
    uint32_t pairs = 0;
    for (std::size_t i = 0; i < bodies->size(); ++i) {
        if (((*bodies)[i].flags & spade::physics::body_flags::active) == 0u) continue;
        for (std::size_t j = i + 1; j < bodies->size(); ++j) {
            if (((*bodies)[j].flags & spade::physics::body_flags::active) == 0u) continue;
            if (glm::length((*bodies)[i].pos - (*bodies)[j].pos) < 2.0f * proxy_radius) ++pairs;
        }
    }
    return pairs;
}

// ===========================================================================
// THE DATA CORPUS -- tests/golden/scenarios/*.scenario.yaml.
//
// Located by SPADE_GOLDEN_DIR, an absolute path baked in at configure time, so
// nothing here depends on the working directory (the no-CWD rule: a compile
// definition cannot be defeated by running the exe from
// elsewhere).
//
// THE DIRECTORY IS THE CORPUS. Every *.scenario.yaml in it is a member, so
// adding a scenario is committing a file rather than editing a list here --
// which is the whole point of the corpus being data. They are loaded in
// FILENAME order, because directory_iterator's order is unspecified and a
// failure message should name a stable sequence.
// ===========================================================================

using spade::testing::LoadedScenario;
using spade::testing::ScenarioData;
using spade::testing::ScenarioSpawn;

[[nodiscard]] std::filesystem::path scenario_dir() {
    return std::filesystem::path(SPADE_GOLDEN_DIR) / "scenarios";
}

// One scenario by name. Sound because ScenarioCorpus.EveryFileIsNamedAfterIts-
// Scenario pins filename stem == the `name` field.
[[nodiscard]] spade::Result<LoadedScenario> load_scenario(std::string_view name) {
    return spade::testing::scenario_from_yaml(scenario_dir() /
                                              (std::string(name) + ".scenario.yaml"));
}

[[nodiscard]] std::vector<LoadedScenario> corpus() {
    std::vector<std::filesystem::path> paths;
    std::error_code ec;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(scenario_dir(), ec)) {
        // ".scenario.yaml", not merely ".yaml": this directory holds one kind
        // of file today and the predicate should still be right when it does
        // not.
        if (entry.is_regular_file() && entry.path().filename().string().ends_with(".scenario.yaml")) {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());

    std::vector<LoadedScenario> loaded;
    loaded.reserve(paths.size());
    for (const std::filesystem::path& path : paths) {
        const spade::Result<LoadedScenario> scenario = spade::testing::scenario_from_yaml(path);
        if (!scenario) {
            ADD_FAILURE() << "failed to load " << path.string() << ": ["
                          << static_cast<int>(scenario.error().code) << "] "
                          << scenario.error().context;
            continue;
        }
        loaded.push_back(*scenario);
    }
    // WITHOUT THIS, A BROKEN SPADE_GOLDEN_DIR WOULD MAKE EVERY `for (scenario :
    // corpus())` TEST IN THIS FILE PASS VACUOUSLY -- an empty range asserts
    // nothing, cheerfully. The membership test below pins the count too; this
    // is the floor under every other user of corpus().
    if (loaded.empty()) {
        ADD_FAILURE() << "the scenario corpus is empty: " << scenario_dir().string();
    }
    return loaded;
}

// ---------------------------------------------------------------------------
// ONE WORLD OF A DATA SCENARIO, RUN ALONE -- same world instance, same step
// decomposition, the same spawns and the same inputs, with every world index
// remapped to 0.
//
// This is what makes the isolation and two-forms comparisons below comparisons
// of THE SAME EXPERIMENT run two ways, rather than of two transcriptions that
// can drift. Before the corpus was data these tests re-spelled each scenario's
// spawns inline, and a retuned scenario would have left them silently comparing
// two different experiments and passing anyway.
//
// The world-local addressing carries over unchanged: a spawn's `local_body_slot`
// and `vehicle_ordinal` count only entries in ITS OWN world, and this function
// preserves their relative order, so an input script that addresses body slot 3
// of world 2 addresses body slot 3 of the solo run.
// ---------------------------------------------------------------------------
[[nodiscard]] spade::Result<Simulation> run_world_alone(const ScenarioData& data, uint32_t world) {
    WorldSetDesc solo;
    solo.worlds.push_back(data.worlds.worlds[world]);
    spade::Result<Simulation> sim = Simulation::create(solo, data.dt_ns, data.substeps);
    if (!sim) return sim;

    std::vector<spade::ModelTypeId> ids;
    ids.reserve(data.models.size());
    for (const spade::vehicles::ModelType& model : data.models) {
        const spade::Result<spade::ModelTypeId> id = sim->register_model(model);
        if (!id) return std::unexpected(id.error());
        ids.push_back(*id);
    }

    for (const ScenarioSpawn& spawn : data.spawns) {
        if (spawn.world != world) continue;
        if (spawn.kind == ScenarioSpawn::Kind::body) {
            const spade::Result<BodyRef> ref = sim->spawn(0, spawn.body);
            if (!ref) return std::unexpected(ref.error());
            for (const spade::DragElementSpawn& drag : spawn.drag_elements) {
                const spade::Result<spade::DragElementRef> element = sim->add_drag_element(*ref, drag);
                if (!element) return std::unexpected(element.error());
            }
        } else {
            const spade::Result<spade::VehicleRef> ref =
                sim->spawn(0, ids[spawn.model_index], spawn.vehicle);
            if (!ref) return std::unexpected(ref.error());
        }
    }
    if (spade::Result<void> r = sim->flush_structural(); !r) return std::unexpected(r.error());

    for (uint64_t t = 0; t < data.steps; ++t) {
        for (const spade::testing::ScenarioInput& input : data.inputs) {
            if (input.tick != t) continue;
            const ScenarioSpawn& target = data.spawns[input.spawn_index];
            if (target.world != world) continue;
            if (input.kind == spade::testing::ScenarioInput::Kind::wrench) {
                const spade::Result<BodyRef> ref = sim->body_ref_at(0, target.local_body_slot);
                if (!ref) return std::unexpected(ref.error());
                if (spade::Result<void> r = sim->apply_wrench(*ref, input.force, input.torque); !r) {
                    return std::unexpected(r.error());
                }
            } else {
                const spade::Result<spade::VehicleRef> ref =
                    sim->vehicle_ref_at(0, target.vehicle_ordinal);
                if (!ref) return std::unexpected(ref.error());
                if (spade::Result<void> r = sim->set_rotor_commands(*ref, input.omega); !r) {
                    return std::unexpected(r.error());
                }
            }
        }
        if (spade::Result<void> r = sim->step(1); !r) return std::unexpected(r.error());
    }
    return sim;
}

[[nodiscard]] std::string hex64(uint64_t value) {
    std::ostringstream out;
    out << "0x" << std::hex << value;
    return out.str();
}

}  // namespace

// ===========================================================================
// 1. The schedule is the spec's schedule
// ===========================================================================

// There is no Gravity pass: integrate_bodies() applies gravity itself (engine
// A9), so anything else that also accumulated m*g would double it. A body
// in a world with no geometry, no drag and no turbulence must fall by exactly
// one g -- checked against the closed form of symplectic Euler over n substeps,
// v = -g*n*h, which is exact here because nothing else touches the body.
TEST(Schedule, GravityIsAppliedExactlyOnce) {
    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(1, 1)).build();
    ASSERT_OK(world);

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 7;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts = contacts(0.0f, 0.0f, 0.0f);
    instance.grid = grid(1.0f);

    spade::Result<Simulation> sim = Simulation::create(WorldSetDesc{{instance}}, 4'000'000, 4);
    ASSERT_OK(sim);

    BodySpawn body;
    body.pos = glm::vec3(0.0f, 1000.0f, 0.0f);
    body.mass = 2.0f;
    const spade::Result<BodyRef> ref = sim->spawn(0, body);
    ASSERT_OK(ref);
    ASSERT_OK(sim->step(1));

    const spade::Result<const spade::BodyState*> state = sim->body(*ref);
    ASSERT_OK(state);

    const float h = sim->substep_h();
    const float expected = -9.80665f * h * 4.0f;  // four substeps, one g each
    EXPECT_NEAR((*state)->vel.y, expected, 1e-6f);
    // The failure this test exists for: a second gravity term that also accumulated
    // m*g would land on twice this.
    EXPECT_GT((*state)->vel.y, 1.5f * expected);
}

// ===========================================================================
// 2. Step decomposition and the ns -> s conversion
// ===========================================================================

TEST(Simulation, SubstepDurationIsTheCorrectlyRoundedNanosecondQuotient) {
    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(1, 1)).build();
    ASSERT_OK(world);
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.contacts = contacts(0.0f, 0.0f, 0.0f);
    instance.grid = grid(1.0f);

    spade::Result<Simulation> sim = Simulation::create(WorldSetDesc{{instance}}, 5'000'000, 5);
    ASSERT_OK(sim);
    EXPECT_EQ(sim->substep_dt_ns(), 1'000'000u);
    // Exactly the float the documented formula produces -- NOT 0.001f-as-written
    // by accident, but by the same computation, so a future refactor that
    // introduced a double intermediate would fail here.
    EXPECT_EQ(sim->substep_h(), static_cast<float>(1'000'000) / 1.0e9f);
    EXPECT_EQ(sim->dt_ns(), 5'000'000u);
    EXPECT_EQ(sim->substeps(), 5u);

    // Indivisible decompositions are rejected rather than silently truncated.
    const spade::Result<Simulation> bad = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 3);
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(code_of(bad), code(spade::Code::invalid_argument));

    const spade::Result<Simulation> zero_dt = Simulation::create(WorldSetDesc{{instance}}, 0, 1);
    ASSERT_FALSE(zero_dt.has_value());
    const spade::Result<Simulation> zero_sub = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 0);
    ASSERT_FALSE(zero_sub.has_value());
}

// ===========================================================================
// 3. Determinism: the same scenario twice
// ===========================================================================

// ---------------------------------------------------------------------------
// ⛔ THE ANTI-VACUITY ARMS BELOW WERE ADDED 2026-09-23 AND THEY ARE NOT
// DECORATION -- WITHOUT THEM THESE TWO TESTS ARE THE PUREST FAIL-GREEN SHAPE
// THIS SUITE CONTAINS.
//
// Both assert that two runs AGREE. An agreement test is satisfied by any
// instrument that returns a CONSTANT, and both instruments here have a named,
// reachable way to do exactly that: state_digest() folds to zero when the
// registry walk finds nothing, and memcmp over a zero-length snapshot returns 0
// for any two empty buffers. In both cases the run reports success having
// compared nothing.
//
// ⭐⭐ AND THE FLOOR WAS ALREADY HERE, ONE LEVEL UP. corpus() carries a careful
// guard for precisely this -- "WITHOUT THIS, A BROKEN SPADE_GOLDEN_DIR WOULD
// MAKE EVERY `for (scenario : corpus())` TEST IN THIS FILE PASS VACUOUSLY -- an
// empty range asserts nothing, cheerfully." That reasoning was applied to the
// COLLECTION and never to the QUANTITY, so the loop was proven non-empty while
// the thing compared inside it could still be nothing at all. A FLOOR UNDER THE
// RANGE IS NOT A FLOOR UNDER THE VALUE.
//
// ⭐ THE SAME GUARD ALREADY EXISTS IN THE DERIVED TEST AND NOT IN THIS, THE
// ORIGINAL. test_gpu_invariance.cpp's A7 sweep spells out both arms --
// "state_digest() folded to zero -- the registry walk found nothing" and "the
// digest did not move across the run -- a frozen device would satisfy
// bit-identity below vacuously" -- and its header says it took its non-vacuity
// posture FROM the two-run tests it generalizes. The posture travelled in the
// comment and the arms did not.
//
// That is also why `at_rest` is safe to assert against rather than a guess: the
// GPU sweep already asserts "the digest moved" over this same corpus, on the
// same scenarios, and passes. This adds the CPU side of a property the other
// backend has been proving all along.
// ---------------------------------------------------------------------------

TEST(Determinism, SameScenarioTwiceProducesTheSameDigest) {
    for (const LoadedScenario& loaded : corpus()) {
        const Scenario& scenario = loaded.scenario;

        // Built and NOT stepped: the digest of the scenario at rest, which is
        // what "the run moved" is measured against.
        const spade::Result<Simulation> fresh = spade::testing::start_scenario(scenario);
        ASSERT_OK(fresh) << scenario.name;
        const uint64_t at_rest = spade::testing::state_digest(*fresh);

        const spade::Result<uint64_t> first = spade::testing::run_scenario(scenario);
        ASSERT_OK(first) << scenario.name;
        const spade::Result<uint64_t> second = spade::testing::run_scenario(scenario);
        ASSERT_OK(second) << scenario.name;

        EXPECT_NE(*first, 0u) << scenario.name
                              << ": state_digest() folded to zero -- the registry walk found "
                                 "nothing, and two runs of nothing agree";
        EXPECT_NE(*first, at_rest)
            << scenario.name
            << ": the digest is unchanged from the un-stepped simulation -- a scenario that "
               "did not run satisfies the equality below vacuously";

        EXPECT_EQ(*first, *second) << scenario.name;
    }
}

TEST(Determinism, SameScenarioTwiceProducesByteIdenticalSnapshots) {
    for (const LoadedScenario& loaded : corpus()) {
        const Scenario& scenario = loaded.scenario;
        std::vector<std::byte> bytes[2];
        for (int run = 0; run < 2; ++run) {
            spade::Result<Simulation> sim = spade::testing::start_scenario(scenario);
            ASSERT_OK(sim) << scenario.name;
            ASSERT_OK(spade::testing::advance_scenario(scenario, *sim, scenario.steps)) << scenario.name;
            const spade::Result<SnapshotBlob> blob = sim->snapshot();
            ASSERT_OK(blob) << scenario.name;
            bytes[run].assign(blob->bytes().begin(), blob->bytes().end());
        }
        ASSERT_EQ(bytes[0].size(), bytes[1].size()) << scenario.name;

        // `memcmp(a, b, 0)` IS 0. Two empty snapshots compare equal, and the
        // EXPECT below would report a clean pass over zero bytes.
        ASSERT_GT(bytes[0].size(), std::size_t{0})
            << scenario.name << ": the snapshot is empty -- memcmp over zero bytes is a pass "
            << "that compared nothing";

        EXPECT_EQ(std::memcmp(bytes[0].data(), bytes[1].data(), bytes[0].size()), 0) << scenario.name;
    }
}

// ===========================================================================
// 4. THE REPLAY GUARANTEE (charter P2/P4)
// ===========================================================================

TEST(Determinism, SnapshotRestoreIntoAFreshSimulationResumesIdentically) {
    std::vector<Scenario> scenarios;
    for (const LoadedScenario& loaded : corpus()) scenarios.push_back(loaded.scenario);
    scenarios.push_back(churn_scenario());

    for (const Scenario& scenario : scenarios) {
        const uint64_t k = scenario.steps / 2;

        // The uninterrupted reference run.
        spade::Result<Simulation> reference = spade::testing::start_scenario(scenario);
        ASSERT_OK(reference) << scenario.name;
        ASSERT_OK(spade::testing::advance_scenario(scenario, *reference, scenario.steps)) << scenario.name;
        const uint64_t expected = spade::testing::state_digest(*reference);

        // The interrupted run: stop at k, snapshot.
        spade::Result<Simulation> interrupted = spade::testing::start_scenario(scenario);
        ASSERT_OK(interrupted) << scenario.name;
        ASSERT_OK(spade::testing::advance_scenario(scenario, *interrupted, k)) << scenario.name;
        const spade::Result<SnapshotBlob> blob = interrupted->snapshot();
        ASSERT_OK(blob) << scenario.name;
        EXPECT_EQ(blob->tick().value, k) << scenario.name;

        // A FRESH Simulation -- not the one that produced the blob. This is the
        // load-bearing part: it proves the blob carries everything, rather than
        // the resumed run quietly benefiting from state the original object
        // still happened to hold.
        spade::Result<Simulation> resumed = spade::testing::start_scenario(scenario);
        ASSERT_OK(resumed) << scenario.name;
        ASSERT_OK(resumed->restore(*blob)) << scenario.name;
        EXPECT_EQ(resumed->tick().value, k) << scenario.name;
        ASSERT_OK(spade::testing::advance_scenario(scenario, *resumed, scenario.steps)) << scenario.name;

        EXPECT_EQ(spade::testing::state_digest(*resumed), expected) << scenario.name;
    }
}

// ===========================================================================
// 5. Batching invariance (D8)
// ===========================================================================

TEST(Determinism, AWorldSteppedAloneMatchesTheSameWorldInsideAFourWorldSet) {
    const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                      .name("ground")
                                                      .environment(default_environment())
                                                      .capacities(capacities(8, 8))
                                                      .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                      .build();
    ASSERT_OK(world);

    WorldInstanceDesc prototype;
    prototype.world = *world;
    prototype.turbulence = spade::dryden_params(TurbulenceLevel::moderate);
    prototype.contacts = contacts(0.35f, 0.25f, 0.1f);
    prototype.grid = grid(0.2f);

    // Four worlds, identical in everything but their rng roots.
    const WorldSetDesc four = spade::replicate(prototype, 4, 0xBA7C4EDULL);

    // Spawns and inputs that depend on the body index but NEVER on the world
    // index -- see scripted_force().
    const auto spawn_bodies = [](Simulation& sim) -> spade::Result<void> {
        for (uint32_t w = 0; w < sim.world_count(); ++w) {
            for (uint32_t i = 0; i < 5; ++i) {
                BodySpawn body;
                body.pos = glm::vec3(0.18f * static_cast<float>(i), 1.0f + 0.05f * static_cast<float>(i),
                                     0.1f * static_cast<float>(i % 2u));
                body.vel = glm::vec3(0.0f, 0.0f, 0.05f * static_cast<float>(i));
                body.mass = 0.6f;
                body.inv_inertia_diag = glm::vec3(150.0f);
                const spade::Result<BodyRef> ref = sim.spawn(w, body);
                if (!ref) return std::unexpected(ref.error());
                DragElementSpawn drag;
                drag.mode = spade::physics::drag_mode::quadratic;
                drag.area = 0.02f;
                drag.coeffs = glm::vec3(1.0f, 0.0f, 0.0f);
                const spade::Result<spade::DragElementRef> elem = sim.add_drag_element(*ref, drag);
                if (!elem) return std::unexpected(elem.error());
            }
        }
        return {};
    };

    const auto drive = [](Simulation& sim, uint64_t steps) -> spade::Result<void> {
        for (uint64_t t = 0; t < steps; ++t) {
            for (uint32_t w = 0; w < sim.world_count(); ++w) {
                for (uint32_t i = 0; i < 5; ++i) {
                    const spade::Result<BodyRef> ref = sim.body_ref_at(w, i);
                    if (!ref) continue;
                    if (spade::Result<void> r =
                            sim.apply_wrench(*ref, scripted_force(t, i), glm::vec3(0.0f));
                        !r) {
                        return r;
                    }
                }
            }
            if (spade::Result<void> r = sim.step(1); !r) return r;
        }
        return {};
    };

    constexpr uint64_t kSteps = 250;
    constexpr uint64_t kDtNs = 2'000'000;
    constexpr uint32_t kSubsteps = 2;

    spade::Result<Simulation> batched = Simulation::create(four, kDtNs, kSubsteps);
    ASSERT_OK(batched);
    ASSERT_TRUE(batched->layout().uniform_dynamic_params);  // the batched sweep really is batched
    ASSERT_OK(spawn_bodies(*batched));
    ASSERT_OK(batched->flush_structural());
    ASSERT_OK(drive(*batched, kSteps));

    for (uint32_t w = 0; w < 4; ++w) {
        WorldSetDesc solo;
        solo.worlds.push_back(four.worlds[w]);
        spade::Result<Simulation> alone = Simulation::create(solo, kDtNs, kSubsteps);
        ASSERT_OK(alone) << "world " << w;
        ASSERT_OK(spawn_bodies(*alone)) << "world " << w;
        ASSERT_OK(alone->flush_structural()) << "world " << w;
        ASSERT_OK(drive(*alone, kSteps)) << "world " << w;

        EXPECT_EQ(spade::testing::world_digest(*alone, 0), spade::testing::world_digest(*batched, w))
            << "world " << w << " diverged between the solo and batched runs";
    }
}

TEST(Determinism, TwoWorldsAtOverlappingCoordinatesDoNotInteract) {
    // The isolation scenario puts world 1's two bodies inside world 0's cluster.
    // Running world 1 alone must reproduce it exactly.
    const spade::Result<LoadedScenario> loaded = load_scenario("two_world_isolation");
    ASSERT_OK(loaded);
    const Scenario& scenario = loaded->scenario;

    // -----------------------------------------------------------------------
    // ⛔ THE PREMISE, ASSERTED (2026-09-23). EVERYTHING THIS TEST MEANS DEPENDS
    // ON THE WORLDS ACTUALLY OVERLAPPING, AND THAT FACT LIVED ONLY IN A YAML
    // FILE AND IN THE COMMENT ABOVE.
    //
    // Two worlds whose bodies are nowhere near each other trivially do not
    // interact. Move a spawn in two_world_isolation.scenario.yaml -- retune the
    // cluster, separate the coordinates, drop world 1's bodies somewhere else --
    // and the equality below still holds, still passes, and proves nothing about
    // the broad phase. The scenario would go on being named "isolation" while
    // testing that two distant things do not touch.
    //
    // ⭐ THE ADJACENT SECTION ALREADY KNEW THIS. Section 5b's two tests assert
    // the batch-branch flag on each side "so neither can quietly degenerate back
    // into batched-vs-batched", and its mutual-contact test says outright that a
    // single-body world "finds no pair at all, which would make the comparison
    // vacuous". The guard was written for the CODE PATH one test down and never
    // for the SCENARIO GEOMETRY here, which is the same boundary this branch's
    // other two commits ran into.
    //
    // THE THRESHOLD IS READ FROM THE SCENARIO, NOT TRANSCRIBED: a body pair
    // closer than the contact proxy DIAMETER is a pair the dynamic sweep would
    // resolve if the broad phase leaked across worlds. That is exactly the
    // leak this test exists to detect, so it is the right distance to demand.
    // (As committed, world 1's two bodies sit at the SAME coordinates as two of
    // world 0's, so the measured distance is 0 and the margin is the whole
    // diameter.)
    // -----------------------------------------------------------------------
    const ScenarioData& data = *loaded->data;
    ASSERT_GE(data.worlds.worlds.size(), std::size_t{2}) << "the scenario must have two worlds";
    const float contact_reach = 2.0f * data.worlds.worlds[0].contacts.proxy_radius;
    ASSERT_GT(contact_reach, 0.0f) << "a zero proxy radius makes the overlap check below meaningless";

    std::size_t world0_bodies = 0;
    std::size_t world1_bodies = 0;
    for (const ScenarioSpawn& spawn : data.spawns) {
        if (spawn.kind != ScenarioSpawn::Kind::body) continue;
        if (spawn.world == 0) ++world0_bodies;
        if (spawn.world == 1) ++world1_bodies;
    }
    ASSERT_GT(world0_bodies, std::size_t{0}) << "world 0 has no bodies to overlap WITH";
    ASSERT_GT(world1_bodies, std::size_t{0}) << "world 1 has no bodies to overlap";

    for (const ScenarioSpawn& one : data.spawns) {
        if (one.kind != ScenarioSpawn::Kind::body || one.world != 1) continue;
        float nearest = std::numeric_limits<float>::infinity();
        for (const ScenarioSpawn& zero : data.spawns) {
            if (zero.kind != ScenarioSpawn::Kind::body || zero.world != 0) continue;
            nearest = std::min(nearest, glm::length(one.body.pos - zero.body.pos));
        }
        EXPECT_LT(nearest, contact_reach)
            << "a world-1 body at (" << one.body.pos.x << ", " << one.body.pos.y << ", "
            << one.body.pos.z << ") is " << nearest << " from the nearest world-0 body, beyond the "
            << contact_reach << " the contact proxies reach. THE WORLDS NO LONGER OVERLAP, so the "
               "equality below would hold for two worlds that could never have interacted and this "
               "test has stopped detecting a broad-phase leak";
    }

    spade::Result<Simulation> both = spade::testing::start_scenario(scenario);
    ASSERT_OK(both);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *both, scenario.steps));

    // World 1's OWN spawns, read from the scenario data rather than transcribed
    // -- see run_world_alone()'s note on why that difference matters.
    const spade::Result<Simulation> alone = run_world_alone(*loaded->data, 1);
    ASSERT_OK(alone);

    EXPECT_EQ(spade::testing::world_digest(*alone, 0), spade::testing::world_digest(*both, 1));
}

// ===========================================================================
// 5b. THE TWO CollisionDynamic FORMS AGREE
//
// schedule.cpp branches on ctx.batch_dynamic_collision: one batched sweep over
// every world when the set's materials are uniform, one sweep per world when
// they are not. Their equivalence was, until these two tests, asserted only in
// PROSE -- and nothing in the suite compared them:
//
//   * AWorldSteppedAloneMatchesTheSameWorldInsideAFourWorldSet builds its solo
//     side as a ONE-WORLD set, and validate_world_set() leaves
//     uniform_dynamic_params TRUE for those (its comparison loop starts at
//     index 1 and never runs). So both sides took the batched path: the
//     ASSERT_TRUE on the batched side silently held for the solo side too.
//   * `bounce` is the only scenario that exercises the per-world form, and it
//     only pins its own golden -- a divergence between the forms would move
//     bounce's golden and nothing else, and would then be indistinguishable
//     from an intended physics change.
//
// A divergence would therefore have left every golden green and every
// invariance test green. The shape below is the only one that puts the two
// code paths on OPPOSITE sides of an equality: the same world instance run
// alone (one world => uniform => BATCHED) against that world inside a
// heterogeneous set (=> PER-WORLD). Both tests assert the branch flag on each
// side, so neither can quietly degenerate back into batched-vs-batched.
//
// The equivalence itself is a property of grid.hpp: the sort key leads with the
// world id and its final tiebreak is the slot, so within one world the sorted
// sub-sequence -- and hence the Gauss-Seidel sweep order that IS the pass's
// parity contract -- is identical whether that world was swept alone or as part
// of a batch. These tests are the tripwire that keeps it true through S6.
// ===========================================================================

// The strong form: worlds holding a cluster of MUTUALLY CONTACTING bodies, so
// the per-world sweep actually resolves pairs. A single-body world finds no pair
// at all, which would make the comparison vacuous for everything except the
// pass's bookkeeping.
TEST(Determinism, BatchedAndPerWorldCollisionDynamicAgreeWithBodiesInMutualContact) {
    constexpr uint64_t kDtNs = 2'000'000;
    constexpr uint32_t kSubsteps = 2;
    constexpr uint64_t kSteps = 250;
    constexpr float kProxy = 0.1f;

    const spade::Result<WorldSetDesc> ladder = contact_ladder_set(4);
    ASSERT_OK(ladder);

    spade::Result<Simulation> heterogeneous = Simulation::create(*ladder, kDtNs, kSubsteps);
    ASSERT_OK(heterogeneous);
    // THE PER-WORLD BRANCH, asserted rather than assumed.
    ASSERT_FALSE(heterogeneous->layout().uniform_dynamic_params)
        << "the ladder set must be heterogeneous, or this test degenerates into batched-vs-batched";
    ASSERT_OK(spawn_contact_cluster(*heterogeneous));
    ASSERT_OK(heterogeneous->step(kSteps));

    for (uint32_t w = 0; w < 4; ++w) {
        // The cluster must still be a cluster, or the per-world sweep resolved
        // nothing and the comparison below proves nothing about pair math.
        EXPECT_GT(contacting_pairs(*heterogeneous, w, kProxy), 0u)
            << "world " << w << ": no bodies in contact, so CollisionDynamic had no pairs to resolve";

        WorldSetDesc solo;
        solo.worlds.push_back(ladder->worlds[w]);
        spade::Result<Simulation> alone = Simulation::create(solo, kDtNs, kSubsteps);
        ASSERT_OK(alone) << "world " << w;
        // THE BATCHED BRANCH: a one-world set is trivially uniform.
        ASSERT_TRUE(alone->layout().uniform_dynamic_params) << "world " << w;
        ASSERT_OK(spawn_contact_cluster(*alone)) << "world " << w;
        ASSERT_OK(alone->step(kSteps)) << "world " << w;

        EXPECT_EQ(spade::testing::world_digest(*alone, 0), spade::testing::world_digest(*heterogeneous, w))
            << "world " << w << ": the batched and per-world CollisionDynamic forms disagree";
    }
}

// The corpus form: bounce's world 1 verbatim, so the scenario whose golden is
// committed is itself covered on both code paths.
TEST(Determinism, BatchedAndPerWorldCollisionDynamicAgreeOnTheCorpusBounceWorld) {
    const spade::Result<LoadedScenario> loaded = load_scenario("bounce");
    ASSERT_OK(loaded);
    const Scenario& scenario = loaded->scenario;

    spade::Result<Simulation> four = spade::testing::start_scenario(scenario);
    ASSERT_OK(four);
    // bounce's restitution ladder is what makes the set heterogeneous.
    ASSERT_FALSE(four->layout().uniform_dynamic_params);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *four, scenario.steps));

    // Same world instance, same capacities, bounce's OWN drop -- read from the
    // scenario data, never transcribed. THE BATCHED BRANCH: a one-world set is
    // trivially uniform.
    const spade::Result<Simulation> alone = run_world_alone(*loaded->data, 1);
    ASSERT_OK(alone);
    ASSERT_TRUE(alone->layout().uniform_dynamic_params);

    EXPECT_EQ(spade::testing::world_digest(*alone, 0), spade::testing::world_digest(*four, 1));
}

// ===========================================================================
// 6. The committed golden corpus
// ===========================================================================

// THE MEMBERSHIP, PINNED. Every other test in this file iterates corpus(), and
// an iteration asserts nothing about what it did NOT see: a scenario file that
// vanished -- deleted in a merge, lost to a bad rebase, renamed out of the glob
// -- would quietly stop being checked while every test stayed green. This is
// the one place that says what the corpus IS.
//
// SIX SINCE PHY-6 (2026-10-02): gnss_tumble joined, and this test was renamed
// from IsExactlyTheFiveCommittedScenarios because its name stated the count.
TEST(ScenarioCorpus, IsExactlyTheSixCommittedScenarios) {
    std::vector<std::string> names;
    for (const LoadedScenario& loaded : corpus()) names.push_back(loaded.data->name);

    const std::vector<std::string> expected = {"ballistic", "bounce", "gnss_tumble", "quad_hover",
                                               "shower", "two_world_isolation"};  // filename order
    EXPECT_EQ(names, expected)
        << "the scenario corpus changed. Adding one is deliberate (commit the file and this "
           "list); losing one is not.";
}

// What makes load_scenario("bounce") sound, and what keeps a failure message's
// scenario name usable as the path to the file that produced it.
TEST(ScenarioCorpus, EveryFileIsNamedAfterItsScenario) {
    std::error_code ec;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(scenario_dir(), ec)) {
        const std::string filename = entry.path().filename().string();
        if (!filename.ends_with(".scenario.yaml")) continue;
        const std::string stem = filename.substr(0, filename.size() - std::strlen(".scenario.yaml"));
        const spade::Result<LoadedScenario> loaded = load_scenario(stem);
        ASSERT_OK(loaded) << filename;
        EXPECT_EQ(loaded->data->name, stem) << filename;
    }
}

// ALSO THE BEHAVIOR-SLOT INERTNESS GUARD (24th spec SL6, Plan A Task 7). The
// two Behaviors* passes were added to the schedule with empty bodies, and "they
// are inert" is a claim this test is what verifies -- a slot that did anything
// at all moves a digest. Anyone weakening or re-blessing this is also removing
// the only thing standing between a behavior slot and a silent parity
// regression. GoldenCorpus.TheDataScenariosReproduceTheRetiredBuilderCorpus
// below is the stronger half: its expectations are spelled in C++ here, so
// unlike this test it cannot be satisfied by editing a scenario file.
TEST(Determinism, DigestsMatchTheCommittedGoldenCorpus) {
    for (const LoadedScenario& loaded : corpus()) {
        const Scenario& scenario = loaded.scenario;
        const spade::Result<uint64_t> digest = spade::testing::run_scenario(scenario);
        ASSERT_OK(digest) << scenario.name;

        EXPECT_EQ(*digest, loaded.data->expected_digest)
            << "scenario '" << scenario.name << "' digest changed: got " << hex64(*digest)
            << ", golden " << hex64(loaded.data->expected_digest)
            << "\nThis is a PINNED op order / layout / rng / schedule change. If it is intended, "
               "the scenario file's expected_digest is updated deliberately, with the reason "
               "recorded in that file's provenance header -- never as a drive-by.";
    }
}

// ---------------------------------------------------------------------------
// THE SUBSTITUTION-ABSORPTION RECORD (spec Addendum A section 15 note (iii)) --
// the S5 exit proof's core, kept as a permanent test rather than as a paragraph
// in a commit message.
//
// WHAT HAPPENED. S1-S4's four scenarios were BUILDER LAMBDAS in this file
// (ballistic_scenario(), bounce_scenario(), shower_scenario(),
// two_world_isolation_scenario()), each ~50 lines of WorldBuilder calls, spawn
// records and an input closure, with its digest in tests/golden/<name>.digest.
// S5 Task 7 replaced all four with data files and retired the .digest files.
//
// THE PROOF THAT NOTHING MOVED. Before the lambdas were deleted, this test ran
// them HEAD TO HEAD against the loaded data files -- builder digest against data
// digest, same process, same build -- and every pair matched. Its literal output
// on msvc-ninja-release, 2026-08-12:
//
//   ballistic:           builder=0x234f74d4c3c53563 data=0x234f74d4c3c53563 match=YES
//   bounce:              builder=0x8b6bd5f0df9ff4fa data=0x8b6bd5f0df9ff4fa match=YES
//   shower:              builder=0xbc94f5b048b45473 data=0xbc94f5b048b45473 match=YES
//   two_world_isolation: builder=0xaab013d47c0eba42 data=0xaab013d47c0eba42 match=YES
//
// -- and those four values are, byte for byte, the ones the retired .digest
// files held. The migration is a CONTAINER CHANGE: same worlds, same seeds,
// same materials, same spawns, same script, same digests.
//
// WHAT HAPPENED, PART TWO. Registering the GNSS sensor arena appended four
// entries to the state walk (gnss_sensors, gnss_ring and each one's
// .slot_to_world companion), so every digest in the corpus moved and the
// scenario files now carry the new values.
//
// THE PROBLEM THAT CREATED, STATED PLAINLY. The four numbers above are what
// made this test a SECOND, INDEPENDENTLY SPELLED source: they came from code
// that no longer exists, so no edit to a scenario file could satisfy them.
// Overwriting them with freshly generated values would have destroyed exactly
// that -- the new constants would be "the value Spade produced the day I ran
// it", recorded in two places, and this test would have silently become a
// two-site tamper check while still reading like an independence proof. The
// head-to-head is UNREPEATABLE; the builder lambdas are gone.
//
// SO THE FOUR DIGESTS STAY, AND THE TEST ASSERTS THE CONTINUATION RATHER THAN
// THE ENDPOINT:
//
//     fold(builder_digest, <every walk entry after replay_config>) == today's digest
//
// state_digest() is a forward FNV-1a stream over the registry walk, and
// state/registry.hpp pins walk order to registration order, so a digest taken
// after N entries and folded with entries N+1.. IS the digest over all of them
// -- by construction of the fold, not by observation. The builder digest covers
// walk entries 0..17 (through replay_config and its companion); the GNSS arena
// appended 18..21. Asserting the continuation therefore PINS ENTRIES 0..17 TO A
// NUMBER NO CURRENT CODE CAN PRODUCE. The independence claim survives a
// RECONSTRUCTION where it would not have survived a RE-DERIVATION.
//
// AND IT INTRODUCES NO NEW CONSTANT AT ALL. A note claiming independence that
// rested on a hand-copied chain would be worse than one admitting the test had
// become a tamper check, so the suffix is folded at runtime out of the live
// registry, through the same detail::fold_* helpers state_digest() itself uses.
// There is nothing here for a future regeneration to transcribe wrongly. A
// second implementation of FNV -- which state/snapshot.hpp warns is "a second
// answer to 'did these bytes change'" -- is never written.
//
// ON REACHING INTO detail::. Deliberate, and the lesser evil: the alternative
// is a parallel fold, which is the thing that header warns about. The honest
// fix is to promote the per-entry fold into a named helper that state_digest()
// and this test both call -- its own commit, for the same reason fnv1a64 has
// not moved to core/hash.hpp yet.
//
// IF THIS TEST IS EVER THE THING IN THE WAY. The remedy is never to refresh the
// four constants. Either the chain still reconstructs (fix what moved), or a
// deliberate layout change moved replay_config -- in which case every golden's
// suffix-continuation argument is INVALIDATED, NOT MERELY EXTENDED, and this
// test must be replaced by an honest statement that the corpus has become a
// two-site tamper check. Say that plainly; do not paper it.
//
// WHAT HAPPENED, PART THREE: PHY-7 (2026-10-05). bounce, shower and
// two_world_isolation LEFT THIS TEST. PHY-7 put the contact response into
// the IMU's specific force, a field of the bodies array, so it changed the
// CONTENT of walk entries 0..17 in every scenario with a contact -- the first
// deliberate change to pre-GNSS entry content since the builder retired.
// No fold of appended entries can reach a digest whose earlier entries
// changed, so for those three the continuation is gone, and refreshing their
// builder digests would only have recorded today's value twice. They are
// pinned from now on by their scenario files alone (the corpus tests above),
// cross-checked by the gcc leg under TD-12: a tamper check, not an
// independence proof. Their builder digests, kept here as a record and never
// to be asserted again: bounce 0x8b6bd5f0df9ff4fa, shower 0xbc94f5b048b45473,
// two_world_isolation 0xaab013d47c0eba42.
//
// ballistic has no contact, so its entries 0..17 are untouched and it stays:
// the corpus's one remaining independent pin. If a later deliberate change
// moves it too (PHY-8 may), this test retires then, with the same kind of
// statement. Never refresh the constant.
// ---------------------------------------------------------------------------
TEST(GoldenCorpus, TheDataScenariosReproduceTheRetiredBuilderCorpus) {
    struct Migrated {
        const char* name;
        uint64_t builder_digest;  // what the S1-S4 lambda produced; see the note above
    };
    // NOT REGENERATED, EVER. This is the retired builder's own output and the
    // only reason this test is independent of the file it loads. The other
    // three left at PHY-7 (see the note above).
    const Migrated migrated[] = {
        {"ballistic", 0x234f74d4c3c53563ULL},
    };

    // The latch: the builder-era walk ended with replay_config's elements and
    // then its slot->world map. Everything strictly after that companion is
    // what was appended since.
    //
    // A LATCH, NOT A LIST OF NEW ARRAY NAMES. An ArenaSet array contributes TWO
    // walk entries (state/arenas.cpp registers `name + kSlotToWorldSuffix`
    // alongside it), so "gnss_sensors and gnss_ring" is four entries, not two,
    // and a name list would have folded half the suffix and reported a corpus
    // change. The latch is correct without knowing how many entries a future
    // array adds.
    const std::string latch =
        std::string(spade::kReplayConfigArray) + std::string(spade::kSlotToWorldSuffix);

    for (const Migrated& m : migrated) {
        const spade::Result<LoadedScenario> loaded = load_scenario(m.name);
        ASSERT_OK(loaded) << m.name;

        // run_scenario() is start + advance + state_digest; this is that same
        // run, kept open so the walk can be inspected at the same tick.
        spade::Result<Simulation> sim = spade::testing::start_scenario(loaded->scenario);
        ASSERT_OK(sim) << m.name;
        ASSERT_OK(spade::testing::advance_scenario(loaded->scenario, *sim, loaded->scenario.steps))
            << m.name;
        const uint64_t digest = spade::testing::state_digest(*sim);

        // (a) The run reaches what the file claims.
        EXPECT_EQ(digest, loaded->data->expected_digest)
            << m.name << ": the scenario file's expected_digest is not what the run produces";

        // (b) THE CONTINUATION. Seed with the retired builder's digest and fold
        // only what was appended after it.
        uint64_t seed = m.builder_digest;
        bool latched = false;
        std::size_t folded = 0;
        sim->arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
            if (!latched) {
                if (array.name == latch) latched = true;
                return;
            }
            seed = spade::testing::detail::fold_name(seed, array.name);
            seed = spade::testing::detail::fold_value(seed, array.elem_size);
            seed = spade::testing::detail::fold_value(seed, array.world_count);
            seed = spade::testing::detail::fold_value(seed, array.capacity_per_world);
            seed = spade::testing::detail::fold_bytes(
                seed, std::span<const std::byte>(array.data, array.byte_size()));
            ++folded;
        });

        // A FOLD THAT MATCHED NOTHING WOULD OTHERWISE BE INDISTINGUISHABLE FROM
        // A CORPUS THAT NEVER MOVED. Both of these fail on their own rather than
        // leaving the comparison below to report a mismatch whose real cause is
        // that the walk never reached the latch.
        ASSERT_TRUE(latched) << m.name << ": '" << latch
                             << "' is not in the state walk, so the suffix could not be located. "
                                "replay_config moved or was renamed -- see the note above: every "
                                "golden's continuation argument is then invalidated, not extended.";
        ASSERT_GT(folded, 0u) << m.name
                              << ": nothing is registered after replay_config, so this test folded "
                                 "an empty suffix and proved nothing.";

        // (c) And the chain lands exactly on today's digest.
        EXPECT_EQ(seed, digest)
            << m.name << ": the fold chain from the retired builder corpus no longer reaches "
            << "today's digest. builder=" << hex64(m.builder_digest) << " + " << folded
            << " appended walk entries = " << hex64(seed) << ", but the run produced "
            << hex64(digest)
            << ".\nSomething in walk entries 0..17 changed -- a pinned op order, layout, rng or "
               "schedule change in the PRE-GNSS state, which is exactly what this number "
               "exists to catch. Do NOT refresh it.";
    }
}

// ---------------------------------------------------------------------------
// A GOLDEN OVER GARBAGE IS STILL A GOLDEN, which is exactly why this test
// exists: a digest is a perfectly stable fingerprint of a field full of NaNs,
// of a hundred bodies that tunnelled through the bowl, or of a world where
// nothing ever moved. Pinning the corpus without also pinning that the runs
// MEAN something would be pinning noise.
//
// So each scenario carries a small number of coarse physical expectations --
// coarse on purpose. These are not trajectory assertions (the goldens are
// that); they are the claim that the scenario exercises the physics it is
// named for.
// ---------------------------------------------------------------------------
TEST(Determinism, CorpusScenariosProduceFiniteAndPhysicallySaneStates) {
    // Runs one corpus scenario to completion. Fatal assertions inside a helper
    // cannot return from the caller, so this hands back a Result and every call
    // site ASSERT_OKs it.
    const auto run = [](std::string_view name) -> spade::Result<Simulation> {
        const spade::Result<LoadedScenario> loaded = load_scenario(name);
        if (!loaded) return std::unexpected(loaded.error());
        spade::Result<Simulation> sim = spade::testing::start_scenario(loaded->scenario);
        if (!sim) return sim;
        if (spade::Result<void> r =
                spade::testing::advance_scenario(loaded->scenario, *sim, loaded->scenario.steps);
            !r) {
            return std::unexpected(r.error());
        }
        return sim;
    };

    const auto every_live_body_is_sane = [](const Simulation& sim, const char* what) {
        for (uint32_t w = 0; w < sim.world_count(); ++w) {
            const spade::Result<std::span<const spade::BodyState>> bodies = sim.world_bodies(w);
            ASSERT_OK(bodies) << what;
            for (const spade::BodyState& b : *bodies) {
                if ((b.flags & spade::physics::body_flags::active) == 0u) continue;
                ASSERT_TRUE(std::isfinite(b.pos.x) && std::isfinite(b.pos.y) && std::isfinite(b.pos.z))
                    << what << ": non-finite position";
                ASSERT_TRUE(std::isfinite(b.vel.x) && std::isfinite(b.vel.y) && std::isfinite(b.vel.z))
                    << what << ": non-finite velocity";
                EXPECT_NEAR(glm::length(b.orient), 1.0f, 1e-4f) << what << ": orientation drifted off the unit sphere";
            }
        }
    };

    // ballistic: one second of fall from y = 100, with drag and gusts. Free fall
    // alone is 4.9 m; drag and turbulence perturb that but cannot reverse it.
    {
        const spade::Result<Simulation> sim = run("ballistic");
        ASSERT_OK(sim);
        every_live_body_is_sane(*sim, "ballistic");
        const spade::Result<std::span<const spade::BodyState>> bodies = sim->world_bodies(0);
        ASSERT_OK(bodies);
        EXPECT_LT((*bodies)[0].pos.y, 99.0f) << "ballistic body did not fall";
        EXPECT_GT((*bodies)[0].pos.y, 90.0f) << "ballistic body fell far more than one g would explain";
        EXPECT_LT((*bodies)[0].vel.y, -3.0f) << "ballistic body is not descending";
    }

    // bounce: all four bodies have come to rest ON the plane (within the slop
    // band), and the ladder actually differed -- a restitution sweep in which
    // every world lands on the identical height is a sweep that did nothing.
    {
        const spade::Result<Simulation> sim = run("bounce");
        ASSERT_OK(sim);
        every_live_body_is_sane(*sim, "bounce");
        std::vector<float> heights;
        for (uint32_t w = 0; w < 4; ++w) {
            const spade::Result<std::span<const spade::BodyState>> bodies = sim->world_bodies(w);
            ASSERT_OK(bodies);
            const float y = (*bodies)[0].pos.y;
            EXPECT_GT(y, -0.2f) << "world " << w << " tunnelled through the ground plane";
            EXPECT_LT(y, 2.0f) << "world " << w << " never fell";
            heights.push_back(y);
        }
        bool any_difference = false;
        for (std::size_t i = 1; i < heights.size(); ++i) {
            if (heights[i] != heights[0]) any_difference = true;
        }
        EXPECT_TRUE(any_difference) << "the restitution ladder produced four identical trajectories";
    }

    // shower: every sphere is still inside the bowl's cavity, and the pile has
    // settled below where it started.
    {
        const spade::Result<Simulation> sim = run("shower");
        ASSERT_OK(sim);
        every_live_body_is_sane(*sim, "shower");
        const spade::Result<std::span<const spade::BodyState>> bodies = sim->world_bodies(0);
        ASSERT_OK(bodies);
        float lowest = 1e30f;
        uint32_t counted = 0;
        for (const spade::BodyState& b : *bodies) {
            if ((b.flags & spade::physics::body_flags::active) == 0u) continue;
            ++counted;
            EXPECT_LT(glm::length(b.pos - glm::vec3(0.0f, 0.4f, 0.0f)), 3.9f)
                << "a sphere escaped the bowl";
            lowest = std::min(lowest, b.pos.y);
        }
        EXPECT_EQ(counted, 100u);
        EXPECT_LT(lowest, -1.0f) << "the shower never reached the bottom of the bowl";
    }

    // two-world isolation: both worlds' bodies stayed above their ground plane.
    {
        const spade::Result<Simulation> sim = run("two_world_isolation");
        ASSERT_OK(sim);
        every_live_body_is_sane(*sim, "two_world_isolation");
        for (uint32_t w = 0; w < 2; ++w) {
            const spade::Result<std::span<const spade::BodyState>> bodies = sim->world_bodies(w);
            ASSERT_OK(bodies);
            for (const spade::BodyState& b : *bodies) {
                if ((b.flags & spade::physics::body_flags::active) == 0u) continue;
                EXPECT_GT(b.pos.y, -0.2f) << "world " << w << " tunnelled through the ground plane";
            }
        }
    }

    // quad_hover: both quadrotors are still flying, near where they started,
    // with all four rotors and their IMU alive.
    //
    // THE VERTICAL BAND IS +-5 m OVER 1.8 s, AND IT IS WIDE ON PURPOSE. A
    // moderate-turbulence world hands an episode a NEAR-CONSTANT GUST BIAS
    // rather than a buffet -- sim/world_set.hpp's note gives the arithmetic
    // (tau_w is 10 s at the default reference airspeed, so 1.8 s of flight sees
    // essentially one draw) -- and the two worlds drew different ones, which is
    // exactly why they make a usable isolation pair. The collective script's
    // own effect is asserted where it can be measured cleanly, in
    // QuadHover.TheCollectiveStepsAreWhatMovesTheQuadrotor.
    {
        const spade::Result<Simulation> sim = run("quad_hover");
        ASSERT_OK(sim);
        every_live_body_is_sane(*sim, "quad_hover");
        for (uint32_t w = 0; w < 2; ++w) {
            const spade::Result<std::span<const spade::BodyState>> bodies = sim->world_bodies(w);
            ASSERT_OK(bodies);
            ASSERT_EQ(bodies->size(), 1u) << "world " << w;
            const spade::BodyState& b = (*bodies)[0];
            // Spawned at 50 m in trim. A vehicle that fell out of the sky or
            // climbed away would still have a perfectly stable digest.
            EXPECT_GT(b.pos.y, 45.0f) << "world " << w << ": the quadrotor fell out of the sky";
            EXPECT_LT(b.pos.y, 55.0f) << "world " << w << ": the quadrotor climbed away";
            // A level airframe under a pure collective has no lateral authority
            // at all; the only sideways forces are the gust and its own drag.
            EXPECT_LT(glm::length(glm::vec3(b.pos.x, 0.0f, b.pos.z)), 2.0f)
                << "world " << w << ": the quadrotor drifted far more than the gusts explain";
            // Four rotors turning, one IMU filling its ring.
            const spade::Result<uint32_t> rotors = sim->live_rotor_count(w);
            ASSERT_OK(rotors);
            EXPECT_EQ(*rotors, 4u) << "world " << w;
            const spade::Result<uint32_t> sensors = sim->live_imu_sensor_count(w);
            ASSERT_OK(sensors);
            EXPECT_EQ(*sensors, 1u) << "world " << w;
        }
    }

    // gnss_tumble: one live receiver per world, on its own clock, reporting the
    // antenna it is mounted on. The truth is recomputed in DOUBLE from the
    // final body state -- the last fix is synthesized after the last
    // Integrate, so that state is exactly what it read -- as "position + R r,
    // velocity + omega x r", spelled here rather than read from gnss.cpp.
    //
    // THE BOUNDS ARE 6 SIGMA PER AXIS from the receiver's declared noise:
    // position sqrt(sigma^2 + sigma_bias^2), velocity sigma_vel. Position at
    // that width catches a gross error, not a lever arm. VELOCITY IS THE SHARP
    // ONE: omega x r is about 2 m/s here against a 0.75 m/s bound, so a
    // dropped or sign-flipped cross product fails it.
    {
        const spade::Result<Simulation> sim = run("gnss_tumble");
        ASSERT_OK(sim);
        every_live_body_is_sane(*sim, "gnss_tumble");

        constexpr uint64_t kFixes = 1600 / 8;  // substeps / rate_divider
        constexpr uint64_t kLastTick = 399;    // steps - 1, the step the last fix was synthesized in
        const glm::dvec3 mount(0.25, -0.5, 0.75);
        const double sigma_bias = 0.800000012;
        const double pos_bound_h = 6.0 * std::sqrt(1.5 * 1.5 + sigma_bias * sigma_bias);
        const double pos_bound_v = 6.0 * std::sqrt(2.5 * 2.5 + sigma_bias * sigma_bias);
        const double vel_bound = 6.0 * 0.125;

        spade::sensors::GnssFix newest_fix[2]{};
        spade::BodyState body[2]{};
        for (uint32_t w = 0; w < 2; ++w) {
            const spade::Result<uint32_t> live = sim->live_gnss_sensor_count(w);
            ASSERT_OK(live);
            ASSERT_EQ(*live, 1u) << "world " << w;

            const auto rows = sim->arenas().world_slice(sim->gnss_sensors_array(), w);
            ASSERT_OK(rows);
            const auto row = std::find_if(rows->begin(), rows->end(),
                                          [](const spade::sensors::GnssSensorRow& r) { return r.kind != 0u; });
            ASSERT_NE(row, rows->end()) << "world " << w;
            EXPECT_EQ(row->last_index, kFixes) << "world " << w << ": the rate clock emitted the wrong number of fixes";
            EXPECT_EQ(row->phase, 0u) << "world " << w << ": the final substep should have closed a fix period";

            const auto ring = sim->arenas().world_slice(sim->gnss_ring_array(), w);
            ASSERT_OK(ring);
            const auto newest = std::find_if(ring->begin(), ring->end(),
                                             [&](const spade::sensors::GnssFix& f) { return f.index == kFixes; });
            ASSERT_NE(newest, ring->end()) << "world " << w << ": the newest fix is not in the ring";
            EXPECT_EQ(newest->tick, kLastTick) << "world " << w;
            newest_fix[w] = *newest;

            const spade::Result<std::span<const spade::BodyState>> bodies = sim->world_bodies(w);
            ASSERT_OK(bodies);
            ASSERT_FALSE(bodies->empty()) << "world " << w;
            body[w] = (*bodies)[0];
            ASSERT_NE(body[w].flags & spade::physics::body_flags::active, 0u) << "world " << w;

            const glm::dquat q(body[w].orient);
            const glm::dvec3 lever = q * mount;
            const glm::dvec3 omega_world = q * glm::dvec3(body[w].omega_body);
            const glm::dvec3 pos_true = glm::dvec3(body[w].pos) + lever;
            const glm::dvec3 vel_true = glm::dvec3(body[w].vel) + glm::cross(omega_world, lever);
            const glm::dvec3 pos_err = glm::dvec3(newest->position) - pos_true;
            const glm::dvec3 vel_err = glm::dvec3(newest->velocity) - vel_true;
            EXPECT_LT(std::abs(pos_err.x), pos_bound_h) << "world " << w;
            EXPECT_LT(std::abs(pos_err.y), pos_bound_v) << "world " << w;
            EXPECT_LT(std::abs(pos_err.z), pos_bound_h) << "world " << w;
            for (int k = 0; k < 3; ++k) {
                EXPECT_LT(std::abs(vel_err[k]), vel_bound) << "world " << w << " velocity axis " << k;
            }
        }

        // THE ISOLATION PAIR. The worlds differ only in their rng roots and no
        // drag element reads the medium, so the truth is bit-identical and only
        // the draws differ. Equal fixes would mean one stream fed both.
        EXPECT_TRUE(body[0].pos == body[1].pos && body[0].vel == body[1].vel &&
                    body[0].orient == body[1].orient && body[0].omega_body == body[1].omega_body)
            << "the two worlds' bodies diverged, but nothing random reaches either one";
        EXPECT_TRUE(newest_fix[0].position != newest_fix[1].position)
            << "both worlds reported the same fix from different seeds";
    }
}

// ===========================================================================
// 7. The structural queue
// ===========================================================================

TEST(StructuralQueue, TwoIdenticalChurnRunsAreByteIdentical) {
    const Scenario scenario = churn_scenario();
    std::vector<std::byte> bytes[2];
    for (int run = 0; run < 2; ++run) {
        spade::Result<Simulation> sim = spade::testing::start_scenario(scenario);
        ASSERT_OK(sim);
        ASSERT_OK(spade::testing::advance_scenario(scenario, *sim, scenario.steps));
        const spade::Result<SnapshotBlob> blob = sim->snapshot();
        ASSERT_OK(blob);
        bytes[run].assign(blob->bytes().begin(), blob->bytes().end());
    }
    ASSERT_EQ(bytes[0].size(), bytes[1].size());
    EXPECT_EQ(std::memcmp(bytes[0].data(), bytes[1].data(), bytes[0].size()), 0);
}

TEST(StructuralQueue, SpawnIsInertUntilTheStepBoundary) {
    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(4, 4)).build();
    ASSERT_OK(world);
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.contacts = contacts(0.0f, 0.0f, 0.0f);
    instance.grid = grid(1.0f);
    spade::Result<Simulation> sim = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(sim);

    BodySpawn body;
    body.pos = glm::vec3(0.0f, 5.0f, 0.0f);
    body.mass = 1.0f;
    const spade::Result<BodyRef> ref = sim->spawn(0, body);
    ASSERT_OK(ref);
    EXPECT_EQ(sim->pending_structural_ops(), 1u);

    // The slot is reserved -- so live_count sees it -- but the BYTES are still
    // the arena's zeroes, so flags == 0 and every pass skips it.
    const spade::Result<std::span<const spade::BodyState>> bodies = sim->world_bodies(0);
    ASSERT_OK(bodies);
    EXPECT_EQ((*bodies)[0].flags, 0u);
    EXPECT_EQ((*bodies)[0].pos.y, 0.0f);

    // A wrench on a not-yet-flushed body is reported, not silently discarded by
    // the pending initialization.
    const spade::Result<void> wrench = sim->apply_wrench(*ref, glm::vec3(1.0f), glm::vec3(0.0f));
    ASSERT_FALSE(wrench.has_value());
    EXPECT_EQ(code_of(wrench), code(spade::Code::not_found));

    // Snapshotting over a pending queue would silently drop it.
    const spade::Result<SnapshotBlob> refused = sim->snapshot();
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(code_of(refused), code(spade::Code::invalid_argument));

    ASSERT_OK(sim->flush_structural());
    EXPECT_EQ(sim->pending_structural_ops(), 0u);
    const spade::Result<std::span<const spade::BodyState>> after = sim->world_bodies(0);
    ASSERT_OK(after);
    EXPECT_EQ((*after)[0].flags, spade::physics::body_flags::active);
    EXPECT_EQ((*after)[0].pos.y, 5.0f);
    EXPECT_OK(sim->apply_wrench(*ref, glm::vec3(1.0f), glm::vec3(0.0f)));
    EXPECT_OK(sim->snapshot());
}

TEST(StructuralQueue, BodyCountMirrorsLiveCountAndDespawnCascadesToElements) {
    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(4, 4)).build();
    ASSERT_OK(world);
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.contacts = contacts(0.0f, 0.0f, 0.0f);
    instance.grid = grid(1.0f);
    spade::Result<Simulation> sim = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(sim);

    BodySpawn body;
    body.pos = glm::vec3(0.0f, 5.0f, 0.0f);
    body.mass = 1.0f;
    const spade::Result<BodyRef> a = sim->spawn(0, body);
    ASSERT_OK(a);
    const spade::Result<BodyRef> b = sim->spawn(0, body);
    ASSERT_OK(b);
    DragElementSpawn drag;
    drag.mode = spade::physics::drag_mode::componentwise;
    drag.coeffs = glm::vec3(0.1f);
    ASSERT_OK(sim->add_drag_element(*a, drag));
    ASSERT_OK(sim->add_drag_element(*a, drag));
    ASSERT_OK(sim->add_drag_element(*b, drag));
    ASSERT_OK(sim->step(1));

    const spade::Result<const spade::WorldParams*> params = sim->world_params(0);
    ASSERT_OK(params);
    EXPECT_EQ((*params)->body_count, 2u);
    EXPECT_EQ(*sim->live_body_count(0), 2u);

    // Despawning `a` must take its TWO elements with it, leaving b's one.
    ASSERT_OK(sim->despawn(*a));
    ASSERT_OK(sim->step(1));
    EXPECT_EQ(*sim->live_body_count(0), 1u);
    const spade::Result<const spade::WorldParams*> after = sim->world_params(0);
    ASSERT_OK(after);
    EXPECT_EQ((*after)->body_count, 1u);
    const spade::Result<uint32_t> live_elements =
        sim->arenas().live_count(sim->drag_elements_array(), 0);
    ASSERT_OK(live_elements);
    EXPECT_EQ(*live_elements, 1u);

    // The ref is dead the moment despawn() returns, so a second despawn is
    // rejected rather than double-freeing a slot a later spawn may have taken.
    const spade::Result<void> again = sim->despawn(*a);
    ASSERT_FALSE(again.has_value());
    EXPECT_EQ(code_of(again), code(spade::Code::not_found));

    // And a stale ref does not address the body that reuses its slot -- which
    // is the LIKELY case, since allocation is lowest-free-first.
    const spade::Result<BodyRef> reused = sim->spawn(0, body);
    ASSERT_OK(reused);
    EXPECT_EQ(reused->slot, a->slot);
    EXPECT_NE(reused->generation, a->generation);
    ASSERT_OK(sim->step(1));
    EXPECT_FALSE(sim->body(*a).has_value());
    EXPECT_TRUE(sim->body(*reused).has_value());
}

// ---------------------------------------------------------------------------
// REGRESSION (independent review, C1): a drag element reserved while a despawn
// of world-local body 0 is already queued must NOT be swept up by that body's
// cascade.
//
// The hole: a reserved-but-uninitialized element row is the arena's zeroes, so
// its `body_slot` reads 0 -- and 0 is a legitimate world-local body index. The
// cascade identifies a body's elements by exactly that field, so it would free
// the innocent row, after which its queued init_row would write a LIVE row
// into a slot the arena considers free: `live_count` under-counts, the next
// reservation aliases the same row, and the orphan then applies drag to
// whatever body later occupies that body slot. Every step of that is silent.
//
// The fix writes the row's identity at RESERVATION time (leaving it disabled),
// so the cascade's test is exact from the instant the slot exists.
// ---------------------------------------------------------------------------
TEST(StructuralQueue, ADespawnCascadeDoesNotClaimAnElementReservedForAnotherBody) {
    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(4, 4)).build();
    ASSERT_OK(world);
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.contacts = contacts(0.0f, 0.0f, 0.0f);
    instance.grid = grid(1.0f);
    spade::Result<Simulation> sim = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(sim);

    BodySpawn body;
    body.pos = glm::vec3(0.0f, 5.0f, 0.0f);
    body.mass = 1.0f;
    const spade::Result<BodyRef> a = sim->spawn(0, body);  // world-local slot 0
    ASSERT_OK(a);
    const spade::Result<BodyRef> b = sim->spawn(0, body);  // world-local slot 1
    ASSERT_OK(b);
    ASSERT_OK(sim->step(1));
    ASSERT_EQ(a->slot, 0u);

    // THE ORDER THAT BREAKS IT: the despawn is queued FIRST, so when the
    // element is reserved a moment later its row is still zeroes.
    ASSERT_OK(sim->despawn(*a));
    DragElementSpawn drag;
    drag.mode = spade::physics::drag_mode::componentwise;
    drag.coeffs = glm::vec3(0.1f);
    const spade::Result<spade::DragElementRef> elem = sim->add_drag_element(*b, drag);
    ASSERT_OK(elem);
    ASSERT_OK(sim->step(1));

    // b's element survived a's cascade.
    const spade::Result<uint32_t> live_elements =
        sim->arenas().live_count(sim->drag_elements_array(), 0);
    ASSERT_OK(live_elements);
    EXPECT_EQ(*live_elements, 1u) << "the cascade freed an element belonging to a different body";

    // And it points at b, enabled, in a slot the arena agrees is allocated.
    const spade::Result<std::span<const spade::physics::DragBodyRow>> rows =
        sim->arenas().array(sim->drag_elements_array());
    ASSERT_OK(rows);
    const spade::Result<std::span<const uint32_t>> map =
        sim->arenas().slot_to_world(sim->drag_elements_array());
    ASSERT_OK(map);
    EXPECT_EQ((*map)[elem->slot], 0u) << "a live element row is sitting in a slot the arena thinks is free";
    EXPECT_EQ((*rows)[elem->slot].enabled, 1u);
    EXPECT_EQ((*rows)[elem->slot].body_slot, 1u);

    // The next reservation must get a DIFFERENT slot -- aliasing two refs onto
    // one row is the downstream symptom the freed-slot bug produced.
    const spade::Result<spade::DragElementRef> second = sim->add_drag_element(*b, drag);
    ASSERT_OK(second);
    EXPECT_NE(second->slot, elem->slot);
}

// ---------------------------------------------------------------------------
// REGRESSION (independent review, I2): between despawn() and the step boundary
// the slot is still ALLOCATED, so body_ref_at() -- the state-first door to a
// ref, and the one the replay harness recommends for resume safety -- would
// re-mint the very reference despawn just killed, with the freshly bumped
// generation, and every check in validate_ref() would pass it.
//
// Generation PARITY closes it: odd = live, even = pending release / dead.
// ---------------------------------------------------------------------------
TEST(StructuralQueue, ARefCannotBeReMintedForABodyWhoseDespawnIsQueued) {
    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(4, 4)).build();
    ASSERT_OK(world);
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.contacts = contacts(0.0f, 0.0f, 0.0f);
    instance.grid = grid(1.0f);
    spade::Result<Simulation> sim = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(sim);

    BodySpawn body;
    body.pos = glm::vec3(0.0f, 5.0f, 0.0f);
    body.mass = 1.0f;
    const spade::Result<BodyRef> a = sim->spawn(0, body);
    ASSERT_OK(a);
    ASSERT_OK(sim->spawn(0, body));
    ASSERT_OK(sim->step(1));

    // Live: odd generation, and body_ref_at agrees with what spawn returned.
    const spade::Result<BodyRef> before = sim->body_ref_at(0, 0);
    ASSERT_OK(before);
    EXPECT_EQ(*before, *a);
    EXPECT_EQ(before->generation & 1u, 1u) << "a live slot must carry an odd generation";

    ASSERT_OK(sim->despawn(*a));

    // The window: still allocated, release queued. Every door must refuse.
    const spade::Result<BodyRef> during = sim->body_ref_at(0, 0);
    EXPECT_EQ(code_of(during), code(spade::Code::not_found))
        << "body_ref_at re-minted a ref for a body whose despawn is queued";
    EXPECT_EQ(code_of(sim->body(*a)), code(spade::Code::not_found));
    EXPECT_EQ(code_of(sim->despawn(*a)), code(spade::Code::not_found));
    EXPECT_EQ(code_of(sim->apply_wrench(*a, glm::vec3(1.0f), glm::vec3(0.0f))),
              code(spade::Code::not_found));
    DragElementSpawn drag;
    drag.coeffs = glm::vec3(0.1f);
    EXPECT_EQ(code_of(sim->add_drag_element(*a, drag)), code(spade::Code::not_found));

    // The queue still holds exactly the one free the first despawn queued, so
    // the step applies cleanly rather than failing on a double free.
    EXPECT_EQ(sim->pending_structural_ops(), 1u);
    EXPECT_OK(sim->step(1));
    EXPECT_EQ(*sim->live_body_count(0), 1u);
    EXPECT_EQ(code_of(sim->body_ref_at(0, 0)), code(spade::Code::not_found));

    // Respawning the slot returns it to odd, and hands out a fresh ref.
    const spade::Result<BodyRef> reused = sim->spawn(0, body);
    ASSERT_OK(reused);
    EXPECT_EQ(reused->slot, a->slot);
    EXPECT_EQ(reused->generation & 1u, 1u);
    ASSERT_OK(sim->step(1));
    EXPECT_EQ(*sim->body_ref_at(0, 0), *reused);
}

// ===========================================================================
// 8. Spawn validation
// ===========================================================================

TEST(Spawn, RejectsDegenerateAndBuriedPoses) {
    // A solid half-space y <= 0 with a 0.25 m proxy: a body at y = -0.3 is
    // buried deeper than one radius and would be ejected by an uncapped
    // Baumgarte correction.
    const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                      .name("ground")
                                                      .environment(default_environment())
                                                      .capacities(capacities(2, 1))
                                                      .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                      .build();
    ASSERT_OK(world);
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.contacts = contacts(0.0f, 0.0f, 0.25f);
    instance.grid = grid(0.5f);
    spade::Result<Simulation> sim = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(sim);

    BodySpawn ok;
    ok.pos = glm::vec3(0.0f, 1.0f, 0.0f);
    ok.mass = 1.0f;
    EXPECT_OK(sim->spawn(0, ok));

    BodySpawn buried = ok;
    buried.pos = glm::vec3(0.0f, -0.3f, 0.0f);
    EXPECT_EQ(code_of(sim->spawn(0, buried)), code(spade::Code::invalid_argument));

    // Grazing contact (within one proxy radius) is legitimate and accepted.
    BodySpawn grazing = ok;
    grazing.pos = glm::vec3(0.0f, -0.2f, 0.0f);
    EXPECT_OK(sim->spawn(0, grazing));

    BodySpawn zero_mass = ok;
    zero_mass.mass = 0.0f;
    EXPECT_EQ(code_of(sim->spawn(0, zero_mass)), code(spade::Code::invalid_argument));

    BodySpawn nan_pos = ok;
    nan_pos.pos = glm::vec3(std::numeric_limits<float>::quiet_NaN(), 1.0f, 0.0f);
    EXPECT_EQ(code_of(sim->spawn(0, nan_pos)), code(spade::Code::invalid_argument));

    BodySpawn zero_quat = ok;
    zero_quat.orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_EQ(code_of(sim->spawn(0, zero_quat)), code(spade::Code::invalid_argument));

    EXPECT_EQ(code_of(sim->spawn(1, ok)), code(spade::Code::invalid_argument));  // world out of range
}

TEST(Spawn, NormalizesOrientationAndEnforcesDeclaredCapacity) {
    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(2, 1)).build();
    ASSERT_OK(world);
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.contacts = contacts(0.0f, 0.0f, 0.0f);
    instance.grid = grid(1.0f);
    spade::Result<Simulation> sim = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(sim);

    BodySpawn body;
    body.pos = glm::vec3(0.0f, 1.0f, 0.0f);
    body.mass = 1.0f;
    body.orient = glm::quat(2.0f, 0.0f, 0.0f, 0.0f);  // deliberately un-normalized
    const spade::Result<BodyRef> ref = sim->spawn(0, body);
    ASSERT_OK(ref);
    ASSERT_OK(sim->step(1));
    const spade::Result<const spade::BodyState*> state = sim->body(*ref);
    ASSERT_OK(state);
    EXPECT_NEAR(glm::length((*state)->orient), 1.0f, 1e-6f);

    ASSERT_OK(sim->spawn(0, body));
    // The world declared two bodies; the third is refused.
    EXPECT_EQ(code_of(sim->spawn(0, body)), code(spade::Code::capacity_exceeded));
}

// ===========================================================================
// 9. World-set validation
// ===========================================================================

TEST(WorldSet, RejectsMisconfiguredSets) {
    EXPECT_EQ(code_of(spade::validate_world_set(WorldSetDesc{})), code(spade::Code::invalid_argument));

    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(1, 1)).build();
    ASSERT_OK(world);

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.contacts = contacts(0.0f, 0.0f, 0.5f);
    instance.grid = grid(0.4f);  // below 2 * proxy_radius: silently loses contacts
    EXPECT_EQ(code_of(spade::validate_world_set(WorldSetDesc{{instance}})), code(spade::Code::invalid_argument));

    instance.grid = grid(1.0f);
    instance.contacts.restitution_e = 1.5f;
    EXPECT_EQ(code_of(spade::validate_world_set(WorldSetDesc{{instance}})), code(spade::Code::invalid_argument));

    instance.contacts.restitution_e = 0.5f;
    instance.turbulence.reference_airspeed = 0.0f;
    EXPECT_EQ(code_of(spade::validate_world_set(WorldSetDesc{{instance}})), code(spade::Code::invalid_argument));
}

TEST(WorldSet, ReplicateDerivesDistinctSeedsAndAUniformLayout) {
    const spade::Result<spade::WorldDesc> world =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(2, 2)).build();
    ASSERT_OK(world);
    WorldInstanceDesc prototype;
    prototype.world = *world;
    prototype.contacts = contacts(0.1f, 0.1f, 0.1f);
    prototype.grid = grid(0.5f);

    const WorldSetDesc set = spade::replicate(prototype, 4, 12345);
    ASSERT_EQ(set.worlds.size(), 4u);
    for (std::size_t i = 0; i < set.worlds.size(); ++i) {
        for (std::size_t j = 0; j < i; ++j) {
            EXPECT_NE(set.worlds[i].seed, set.worlds[j].seed) << i << " vs " << j;
        }
    }
    const spade::Result<spade::WorldSetLayout> layout = spade::validate_world_set(set);
    ASSERT_OK(layout);
    EXPECT_EQ(layout->world_count, 4u);
    EXPECT_EQ(layout->body_capacity, 2u);
    EXPECT_TRUE(layout->uniform_dynamic_params);

    // Every world's seed reaches WorldParams, which is what every stochastic
    // system in that world derives its stream from.
    spade::Result<Simulation> sim = Simulation::create(set, 1'000'000, 1);
    ASSERT_OK(sim);
    for (uint32_t w = 0; w < 4; ++w) {
        const spade::Result<const spade::WorldParams*> params = sim->world_params(w);
        ASSERT_OK(params);
        EXPECT_EQ((*params)->seed, set.worlds[w].seed);
    }
}

// ===========================================================================
// 10. Restore hygiene
// ===========================================================================

TEST(Restore, DiscardsThePendingStructuralQueueAndResetsTheTick) {
    const spade::Result<LoadedScenario> loaded = load_scenario("bounce");
    ASSERT_OK(loaded);
    spade::Result<Simulation> sim = spade::testing::start_scenario(loaded->scenario);
    ASSERT_OK(sim);
    ASSERT_OK(sim->step(20));
    const spade::Result<SnapshotBlob> blob = sim->snapshot();
    ASSERT_OK(blob);

    ASSERT_OK(sim->step(30));
    EXPECT_EQ(sim->tick().value, 50u);

    // Queue something, then restore over it: spec §4's "restore = reverse +
    // structural-queue flush".
    BodySpawn body;
    body.pos = glm::vec3(0.0f, 3.0f, 0.0f);
    body.mass = 1.0f;
    ASSERT_OK(sim->spawn(0, body));
    EXPECT_EQ(sim->pending_structural_ops(), 1u);

    ASSERT_OK(sim->restore(*blob));
    EXPECT_EQ(sim->pending_structural_ops(), 0u);
    EXPECT_EQ(sim->tick().value, 20u);

    // And a blob from a differently-shaped world set is refused.
    const spade::Result<spade::WorldDesc> other =
        WorldBuilder().name("void").environment(default_environment()).capacities(capacities(1, 1)).build();
    ASSERT_OK(other);
    WorldInstanceDesc instance;
    instance.world = *other;
    instance.contacts = contacts(0.0f, 0.0f, 0.0f);
    instance.grid = grid(1.0f);
    spade::Result<Simulation> mismatched = Simulation::create(WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(mismatched);
    const spade::Result<void> refused = mismatched->restore(*blob);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(code_of(refused), code(spade::Code::schema_mismatch));
}

// ===========================================================================
// 11. The replay configuration (ticket M-1)
//
// A blob carries STATE and no configuration, so before this existed
// `sim.restore(blob)` accepted any blob whose array SHAPES matched -- and two
// world sets that disagree about restitution, about turbulence, about the SDF
// geometry, or that run at a different substep h, have byte-identical shapes.
// The state loaded correctly and every subsequent step ran different physics.
//
// The `replay_config` array closes that: (dt_ns, substeps, config_hash) is
// registered state, so it is in every blob, and restore() compares it before
// writing a byte. These tests carry four burdens:
//
//   * THE RECORD IS WHAT IT CLAIMS. Every world's row holds the run's identity,
//     and the array is registered LAST -- which is the premise the committed
//     goldens' provenance blocks argue from, so it is asserted here rather than
//     left to a reader's inspection of create().
//   * THE HASH SEPARATES WHAT IT COVERS. One mutation per folded field, each
//     of which must move the hash; plus the one thing it deliberately does not
//     cover.
//   * THE CHECK REJECTS, AND REJECTS CLEANLY. Every rejection leaves the target
//     BYTE-IDENTICAL -- proven by digest, not by inspection of a field or two.
//   * IT COMPOSES WITH reseed(). config_hash is the hash of the CREATING desc,
//     never of live seed state, and those two facts have to hold together for
//     a training loop that reseeds between episodes and snapshots across them.
// ===========================================================================

namespace {

// A two-world set with everything config_hash folds set to a distinctive,
// non-default value, so a mutation test that forgets to change something is
// visible rather than accidentally passing on a default.
[[nodiscard]] spade::Result<WorldSetDesc> config_probe_set() {
    const spade::Result<spade::WorldDesc> ground =
        WorldBuilder()
            .name("ground")
            .environment(default_environment())
            .capacities(capacities(3, 2))
            .spawn("start", glm::vec3(0.0f, 1.0f, 0.0f))
            .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
            .build();
    if (!ground) return std::unexpected(ground.error());

    const spade::Result<spade::WorldDesc> shapes =
        WorldBuilder()
            .name("shapes")
            .environment(default_environment())
            .capacities(capacities(2, 4))
            .sphere(0.75f)
            .box(glm::vec3(0.5f, 0.25f, 0.5f))
            .union_()
            .build();
    if (!shapes) return std::unexpected(shapes.error());

    WorldSetDesc set;

    WorldInstanceDesc a;
    a.world = *ground;
    a.seed = 0x1111'2222'3333'4444ULL;
    a.turbulence = spade::dryden_params(TurbulenceLevel::light);
    a.contacts = contacts(0.25f, 0.4f, 0.1f);
    a.grid = grid(0.5f);
    set.worlds.push_back(a);

    WorldInstanceDesc b;
    b.world = *shapes;
    b.seed = 0x5555'6666'7777'8888ULL;
    b.turbulence = spade::dryden_params(TurbulenceLevel::moderate);
    b.contacts = contacts(0.8f, 0.1f, 0.2f);
    b.grid = grid(0.75f);
    set.worlds.push_back(b);

    return set;
}

// The whole replay_config array, read back out of a live Simulation.
[[nodiscard]] std::vector<spade::ReplayConfig> replay_config_rows(const Simulation& sim) {
    const spade::Result<std::span<const spade::ReplayConfig>> rows =
        sim.arenas().array(sim.replay_config_array());
    if (!rows) return {};
    return std::vector<spade::ReplayConfig>(rows->begin(), rows->end());
}

}  // namespace

TEST(ReplayConfig, EveryWorldRowCarriesTheRunsIdentityAndNothingElse) {
    const spade::Result<WorldSetDesc> set = config_probe_set();
    ASSERT_OK(set);

    constexpr uint64_t kDtNs = 4'000'000;
    constexpr uint32_t kSubsteps = 4;
    spade::Result<Simulation> sim = Simulation::create(*set, kDtNs, kSubsteps);
    ASSERT_OK(sim);

    const std::vector<spade::ReplayConfig> rows = replay_config_rows(*sim);
    ASSERT_EQ(rows.size(), set->worlds.size()) << "one row per world";

    const uint64_t expected = spade::config_hash(*set);
    for (std::size_t w = 0; w < rows.size(); ++w) {
        EXPECT_EQ(rows[w].dt_ns, kDtNs) << "world " << w;
        EXPECT_EQ(rows[w].substeps, kSubsteps) << "world " << w;
        EXPECT_EQ(rows[w].config_hash, expected) << "world " << w;
        // The two reserved lanes are named fields, not compiler padding, and
        // they are part of every blob and every digest -- so "they are zero" is
        // a layout contract, not a formality.
        EXPECT_EQ(rows[w]._pad, 0u) << "world " << w;
        EXPECT_EQ(rows[w]._reserved0, 0u) << "world " << w;
        // The record is SET-WIDE: every world holds the identical bytes, which
        // is what lets restore() compare row 0 alone.
        EXPECT_EQ(std::memcmp(&rows[w], &rows[0], sizeof(spade::ReplayConfig)), 0) << "world " << w;
    }
}

// THE PREMISE OF EVERY GOLDEN'S PROVENANCE BLOCK, asserted rather than left to
// a reader's inspection of create(): replay_config joined the walk as a pure
// SUFFIX. state_digest() folds arrays in registration order, so a new array
// that leaves every earlier array's POSITION alone makes every earlier digest a
// prefix of the new one -- the argument tests/golden/*.digest makes for the
// regeneration that added this array. Inserting a future array between two
// existing ones would silently invalidate that argument for all four digests at
// once, not merely add a fifth to check.
//
// IT PINS THE POSITION, NOT THE LAST-NESS, and the difference is the whole
// point of the test. An assertion that replay_config is LAST would fail the
// moment an array is appended after it -- which is the SANCTIONED move, and
// gnss_sensors and gnss_ring took it -- and its failure message would then
// read as "make replay_config last again", whose easiest remedy is to
// register the new array BEFORE it. That is exactly the
// mid-list insertion this test exists to prevent, arrived at by obeying the
// test. Pinning the INDEX instead means appending is silent (the index does not
// move) and only an insertion at or before it can fail.
TEST(ReplayConfig, OccupiesItsPinnedWalkPosition) {
    // Eight registered arrays come before it, each contributing its elements
    // then its slot->world map, so replay_config's elements are walk entry 16
    // and its map entry 17. See sim/simulation.cpp's registration block.
    constexpr std::size_t kReplayConfigWalkIndex = 16;
    const char* const kRemedy =
        "a new array must be appended AFTER replay_config, never before it -- appending leaves "
        "this index untouched (regenerate the corpus for the new suffix). If a deliberate layout "
        "change really did move replay_config, update this index -- and know that every golden's "
        "suffix-continuation argument is then invalidated, not merely extended";

    const spade::Result<WorldSetDesc> set = config_probe_set();
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    std::vector<std::string> names;
    sim->arenas().registry().for_each_array(
        [&names](const spade::RegisteredArray& array) { names.push_back(array.name); });

    ASSERT_GT(names.size(), kReplayConfigWalkIndex + 1);
    EXPECT_EQ(names[kReplayConfigWalkIndex], std::string(spade::kReplayConfigArray)) << kRemedy;
    EXPECT_EQ(names[kReplayConfigWalkIndex + 1],
              std::string(spade::kReplayConfigArray) + std::string(spade::kSlotToWorldSuffix))
        << "an ArenaSet array contributes its elements then its slot->world map; " << kRemedy;

    // And it appears exactly once, at that position.
    EXPECT_EQ(std::count(names.begin(), names.end(), std::string(spade::kReplayConfigArray)), 1);
}

// ONE MUTATION PER FOLDED FIELD. The fold order in sim/world_set.hpp is a
// contract; this is the test that keeps it honest, because a field quietly
// dropped from the fold is invisible to every other test in the suite -- the
// hash would still be stable, still be deterministic, and simply stop
// separating two configurations that replay differently.
TEST(ReplayConfigHash, SeparatesEveryFieldItFolds) {
    const spade::Result<WorldSetDesc> base = config_probe_set();
    ASSERT_OK(base);
    const uint64_t reference = spade::config_hash(*base);

    // Stability first: the same desc hashes the same, twice, and a copy of it
    // hashes the same as the original. Without this the mutations below could
    // "pass" against a hash that is simply noisy.
    EXPECT_EQ(spade::config_hash(*base), reference);
    const WorldSetDesc copy = *base;
    EXPECT_EQ(spade::config_hash(copy), reference);

    struct Mutation {
        const char* what;
        std::function<void(WorldSetDesc&)> apply;
    };

    const Mutation mutations[] = {
        {"world count", [](WorldSetDesc& d) { d.worlds.push_back(d.worlds[0]); }},
        {"world order", [](WorldSetDesc& d) { std::swap(d.worlds[0], d.worlds[1]); }},
        {"world name", [](WorldSetDesc& d) { d.worlds[1].world.name = "renamed"; }},
        {"sdf node parameter", [](WorldSetDesc& d) { d.worlds[1].world.sdf.nodes[0].params.x = 0.76f; }},
        {"sdf node kind", [](WorldSetDesc& d) { std::swap(d.worlds[1].world.sdf.nodes[0].kind,
                                                          d.worlds[1].world.sdf.nodes[1].kind); }},
        {"sdf node count", [](WorldSetDesc& d) { d.worlds[1].world.sdf.nodes.pop_back(); }},
        {"sdf transform", [](WorldSetDesc& d) { d.worlds[0].world.sdf.transforms[0].scale = 2.0f; }},
        {"sdf transform count",
         [](WorldSetDesc& d) { d.worlds[0].world.sdf.transforms.push_back(spade::SdfTransform{}); }},
        {"environment.gravity", [](WorldSetDesc& d) { d.worlds[0].world.environment.gravity.y = -3.71f; }},
        {"environment.wind", [](WorldSetDesc& d) { d.worlds[0].world.environment.wind.z = 1.5f; }},
        {"environment.air_density", [](WorldSetDesc& d) { d.worlds[1].world.environment.air_density = 0.9f; }},
        {"environment.temperature_k", [](WorldSetDesc& d) { d.worlds[1].world.environment.temperature_k = 300.0f; }},
        {"environment.seed", [](WorldSetDesc& d) { d.worlds[0].world.environment.seed = 99; }},
        {"capacities.bodies", [](WorldSetDesc& d) { d.worlds[0].world.capacities.bodies = 7; }},
        {"capacities.force_elements", [](WorldSetDesc& d) { d.worlds[0].world.capacities.force_elements = 7; }},
        {"capacities.sensors", [](WorldSetDesc& d) { d.worlds[1].world.capacities.sensors = 3; }},
        {"capacities.contacts", [](WorldSetDesc& d) { d.worlds[1].world.capacities.contacts = 5; }},
        {"instance seed", [](WorldSetDesc& d) { d.worlds[1].seed ^= 1u; }},
        {"turbulence.sigma_u", [](WorldSetDesc& d) { d.worlds[0].turbulence.sigma_u += 0.5f; }},
        {"turbulence.reference_airspeed",
         [](WorldSetDesc& d) { d.worlds[1].turbulence.reference_airspeed = 40.0f; }},
        {"contacts.restitution_e", [](WorldSetDesc& d) { d.worlds[0].contacts.restitution_e = 0.9f; }},
        {"contacts.friction_mu", [](WorldSetDesc& d) { d.worlds[1].contacts.friction_mu = 0.95f; }},
        {"grid.cell_size", [](WorldSetDesc& d) { d.worlds[0].grid.cell_size = 1.25f; }},
    };

    std::vector<uint64_t> seen{reference};
    for (const Mutation& mutation : mutations) {
        WorldSetDesc mutated = *base;
        mutation.apply(mutated);
        const uint64_t hash = spade::config_hash(mutated);
        EXPECT_NE(hash, reference) << "config_hash does not cover " << mutation.what;
        // Not merely different from the reference -- different from every OTHER
        // mutation too, which is what catches a fold that collapses two fields
        // into one (a length prefix dropped, say, or two adjacent u32s folded
        // as one u64).
        EXPECT_EQ(std::count(seen.begin(), seen.end(), hash), 0)
            << "mutation '" << mutation.what << "' collides with an earlier one";
        seen.push_back(hash);
    }
}

// The documented non-coverage, pinned so it stays deliberate. Spawn points are
// authoring anchors: a caller reads one to place a body, and the placed body's
// state is in the snapshot. Two descs differing only there replay a restored
// blob identically, so folding them would reject sound pairings.
TEST(ReplayConfigHash, IgnoresSpawnPoints) {
    const spade::Result<WorldSetDesc> base = config_probe_set();
    ASSERT_OK(base);

    WorldSetDesc with_spawn = *base;
    // Third member omitted deliberately: SpawnPoint::orientation carries a
    // default member initializer, and spelling `{}` for it would VALUE-init a
    // glm::quat instead, which glm leaves indeterminate without
    // GLM_FORCE_CTOR_INIT.
    with_spawn.worlds[0].world.spawns.push_back(spade::SpawnPoint{"extra", glm::vec3(4.0f, 5.0f, 6.0f)});
    EXPECT_EQ(spade::config_hash(with_spawn), spade::config_hash(*base));
}

TEST(RestoreConfigCheck, AcceptsABlobFromTheSameConfigurationAndLeavesTheRowIntact) {
    const spade::Result<WorldSetDesc> set = config_probe_set();
    ASSERT_OK(set);

    spade::Result<Simulation> source = Simulation::create(*set, 4'000'000, 4);
    ASSERT_OK(source);
    BodySpawn body;
    body.pos = glm::vec3(0.0f, 5.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    ASSERT_OK(source->spawn(0, body));
    ASSERT_OK(source->step(25));
    const spade::Result<SnapshotBlob> blob = source->snapshot();
    ASSERT_OK(blob);

    // A SEPARATE Simulation from an equal-but-distinct desc: the check compares
    // configuration, not object identity.
    const spade::Result<WorldSetDesc> twin_set = config_probe_set();
    ASSERT_OK(twin_set);
    spade::Result<Simulation> twin = Simulation::create(*twin_set, 4'000'000, 4);
    ASSERT_OK(twin);

    const std::vector<spade::ReplayConfig> before = replay_config_rows(*twin);
    ASSERT_OK(twin->restore(*blob));
    const std::vector<spade::ReplayConfig> after = replay_config_rows(*twin);

    // The row is registered state, so the restore overwrote it -- with bytes it
    // had just proven identical. A successful restore is therefore a no-op on
    // this one array, which is the property that keeps it a stable identity
    // across any number of round trips.
    ASSERT_EQ(before.size(), after.size());
    for (std::size_t w = 0; w < before.size(); ++w) {
        EXPECT_EQ(std::memcmp(&before[w], &after[w], sizeof(spade::ReplayConfig)), 0) << "world " << w;
    }

    ASSERT_OK(source->step(25));
    ASSERT_OK(twin->step(25));
    EXPECT_EQ(spade::testing::state_digest(*source), spade::testing::state_digest(*twin))
        << "a run resumed from a same-configuration blob diverged";
}

// THE REJECTIONS, AND THE PROOF THAT THEY COST NOTHING. Each case restores a
// blob into a Simulation whose configuration differs in exactly one way, and
// asserts (a) the code, (b) that the message names the field that diverged, and
// (c) that the whole-state digest is BYTE-IDENTICAL across the failed call.
// (c) is the important one: the check runs before the arena restore, so a
// rejection must leave every registered byte -- not just the ones a human
// thought to look at -- exactly as it was.
TEST(RestoreConfigCheck, RejectsAMismatchedRunWithoutTouchingAByte) {
    const spade::Result<WorldSetDesc> set = config_probe_set();
    ASSERT_OK(set);

    // The source run: 4 ms in 4 substeps, over the probe set.
    spade::Result<Simulation> source = Simulation::create(*set, 4'000'000, 4);
    ASSERT_OK(source);
    BodySpawn body;
    body.pos = glm::vec3(0.0f, 5.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    ASSERT_OK(source->spawn(0, body));
    ASSERT_OK(source->step(25));
    const spade::Result<SnapshotBlob> blob = source->snapshot();
    ASSERT_OK(blob);

    // A different world set: world 0's restitution alone. THIS IS THE EXACT
    // HOLE the old restore() doc comment documented and handed to the caller --
    // same arrays, same element sizes, same capacities, same world count, so
    // every shape check in the state layer passes.
    WorldSetDesc other_material = *set;
    other_material.worlds[0].contacts.restitution_e = 0.95f;

    struct Case {
        const char* what;
        WorldSetDesc desc;
        uint64_t dt_ns;
        uint32_t substeps;
        const char* names;  // the one field the message must name
    };
    const Case cases[] = {
        {"a different step duration", *set, 8'000'000, 4, "dt_ns"},
        {"a different substep count", *set, 4'000'000, 2, "substeps"},
        {"a different restitution", other_material, 4'000'000, 4, "config_hash"},
    };
    // The message's fixed prefix names all three fields ("a different (dt_ns,
    // substeps, config_hash)"), so searching the WHOLE message for a field name
    // would pass vacuously. Only the "Diverged:" tail is searched, and the two
    // fields that did NOT diverge must be absent from it.
    const char* const all_fields[] = {"dt_ns", "substeps", "config_hash"};

    for (const Case& c : cases) {
        spade::Result<Simulation> target = Simulation::create(c.desc, c.dt_ns, c.substeps);
        ASSERT_OK(target) << c.what;
        ASSERT_OK(target->spawn(0, body)) << c.what;
        ASSERT_OK(target->step(7)) << c.what;

        // The shape checks must NOT be what rejects this, or the test proves
        // nothing about the config check.
        ASSERT_EQ(spade::schema_hash(target->arenas().registry()), blob->schema_hash())
            << c.what << ": the two runs must be schema-identical, or this case is vacuous";

        const uint64_t before = spade::testing::state_digest(*target);
        const uint64_t tick_before = target->tick().value;

        const spade::Result<void> refused = target->restore(*blob);
        ASSERT_FALSE(refused.has_value()) << c.what;
        EXPECT_EQ(code_of(refused), code(spade::Code::invalid_argument)) << c.what;

        const std::string& context = refused.error().context;
        EXPECT_NE(context.find("replay different physics"), std::string::npos) << c.what;
        const std::size_t at = context.find("Diverged: ");
        ASSERT_NE(at, std::string::npos) << c.what << ": got: " << context;
        const std::string diverged = context.substr(at);
        for (const char* field : all_fields) {
            const bool named = diverged.find(field) != std::string::npos;
            EXPECT_EQ(named, std::string(field) == c.names)
                << c.what << ": '" << field << "' named=" << named << " in: " << diverged;
        }

        EXPECT_EQ(spade::testing::state_digest(*target), before)
            << c.what << ": a rejected restore changed registered state";
        EXPECT_EQ(target->tick().value, tick_before) << c.what << ": a rejected restore moved the clock";
    }
}

// THE CHECK'S ACCEPT TEST IS THE WHOLE PAYLOAD, NOT THE THREE NAMED FIELDS.
//
// restore() overwrites all 32 bytes of every world's row; a check that accepted
// on `dt_ns == && substeps == && config_hash ==` would be restoring the other
// eight bytes, and every row above world 0, on trust. This test tampers with
// exactly those places -- a reserved lane in row 0, and world 1's dt_ns -- so
// row 0's three named fields still agree on both sides and only a byte-wise
// comparison can tell the blobs apart. Both must be refused, and the message
// must say so honestly rather than name a field the caller would go looking for
// and find identical.
TEST(RestoreConfigCheck, RejectsAPayloadThatDiffersOutsideTheThreeNamedFields) {
    const spade::Result<WorldSetDesc> set = config_probe_set();
    ASSERT_OK(set);
    ASSERT_EQ(set->worlds.size(), 2u) << "this test tampers with world 1's row";

    spade::Result<Simulation> source = Simulation::create(*set, 4'000'000, 4);
    ASSERT_OK(source);
    ASSERT_OK(source->step(4));
    const spade::Result<SnapshotBlob> blob = source->snapshot();
    ASSERT_OK(blob);

    // Where the replay_config payload sits inside the blob, via the engine's own
    // locator -- no second reader of the format in this test either.
    const spade::Result<spade::BlobSection> section =
        spade::find_section(*blob, spade::kReplayConfigArray);
    ASSERT_OK(section);
    ASSERT_EQ(section->payload.size(), 2u * sizeof(spade::ReplayConfig));
    const std::size_t at = static_cast<std::size_t>(section->payload.data() - blob->bytes().data());

    struct Case {
        const char* what;
        std::size_t byte;  // offset within the payload
    };
    const Case cases[] = {
        {"a reserved lane in world 0's row", offsetof(spade::ReplayConfig, _reserved0)},
        {"world 1's dt_ns", sizeof(spade::ReplayConfig) + offsetof(spade::ReplayConfig, dt_ns)},
    };

    for (const Case& c : cases) {
        std::vector<std::byte> raw(blob->bytes().begin(), blob->bytes().end());
        raw[at + c.byte] ^= std::byte{0x01};
        spade::Result<SnapshotBlob> tampered = spade::SnapshotBlob::from_bytes(std::move(raw));
        ASSERT_OK(tampered) << c.what;

        spade::Result<Simulation> target = Simulation::create(*set, 4'000'000, 4);
        ASSERT_OK(target) << c.what;
        // Untouched header, untouched shapes: only the config check can reject.
        ASSERT_EQ(tampered->schema_hash(), spade::schema_hash(target->arenas().registry())) << c.what;

        const uint64_t before = spade::testing::state_digest(*target);
        const spade::Result<void> refused = target->restore(*tampered);
        ASSERT_FALSE(refused.has_value()) << c.what << ": a byte outside the three named fields was restored on trust";
        EXPECT_EQ(code_of(refused), code(spade::Code::invalid_argument)) << c.what;
        EXPECT_NE(refused.error().context.find("differ outside (dt_ns, substeps, config_hash)"),
                  std::string::npos)
            << c.what << ": got: " << refused.error().context;
        EXPECT_EQ(spade::testing::state_digest(*target), before) << c.what;
    }
}

// THE LAYERS DO NOT SUBSUME ONE ANOTHER, and the precedence between them is a
// decision, not an accident. A set with a different capacity has BOTH a
// different schema hash and a different config_hash (capacities are folded into
// both), so without a rule the caller's error message would depend on which
// check happened to run first. The rule is that the config check stands down
// whenever the schema already disagrees: "array 'bodies': shape 2x5 in the
// blob, 2x4 in the registry" tells a caller what to fix, "you paired the wrong
// blob" does not. This test is that rule.
TEST(RestoreConfigCheck, AShapeMismatchIsStillReportedAsSchemaMismatch) {
    const spade::Result<WorldSetDesc> set = config_probe_set();
    ASSERT_OK(set);
    spade::Result<Simulation> source = Simulation::create(*set, 4'000'000, 4);
    ASSERT_OK(source);
    ASSERT_OK(source->step(3));
    const spade::Result<SnapshotBlob> blob = source->snapshot();
    ASSERT_OK(blob);

    // Same world COUNT, same arrays, same names -- one capacity larger, so the
    // arenas are a different shape and the schema hash differs.
    WorldSetDesc wider = *set;
    wider.worlds[0].world.capacities.bodies += 1;
    spade::Result<Simulation> target = Simulation::create(wider, 4'000'000, 4);
    ASSERT_OK(target);
    ASSERT_NE(spade::schema_hash(target->arenas().registry()), blob->schema_hash());

    const uint64_t before = spade::testing::state_digest(*target);
    const spade::Result<void> refused = target->restore(*blob);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(code_of(refused), code(spade::Code::schema_mismatch))
        << "got: " << refused.error().context;
    EXPECT_EQ(spade::testing::state_digest(*target), before);
}

// ---------------------------------------------------------------------------
// THE reseed() COMPOSITION -- the one interaction where "config_hash is the
// hash of the CREATING desc" and "seeds are state" have to hold together.
//
// reseed() rewrites every world's WorldParams::seed (and the streams derived
// from it) without touching the desc the Simulation was created from, so a
// post-reseed blob carries the ORIGINAL config_hash alongside the NEW seeds.
// Restoring it into a twin built from the ORIGINAL desc must therefore:
//   * PASS the config check (same desc => same hash), and
//   * adopt the new seeds from the blob, because those are state.
// Both halves matter. If config_hash tracked live seed state instead, the first
// would fail and a training loop that reseeds between episodes could never
// snapshot across one.
//
// M1B.ReseedSurvivesASnapshotIntoATwin (tests/test_m1b_bar.cpp) already pins
// the end-to-end continuation; this pins the MECHANISM underneath it, which is
// the part that would otherwise only be argued in a comment.
// ---------------------------------------------------------------------------
TEST(RestoreConfigCheck, ReseedLeavesTheConfigIdentityAloneAndTheBlobStillRestores) {
    const spade::Result<WorldSetDesc> set = config_probe_set();
    ASSERT_OK(set);

    spade::Result<Simulation> source = Simulation::create(*set, 4'000'000, 4);
    ASSERT_OK(source);
    BodySpawn body;
    body.pos = glm::vec3(0.0f, 5.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    ASSERT_OK(source->spawn(0, body));
    ASSERT_OK(source->step(10));

    const std::vector<spade::ReplayConfig> before_reseed = replay_config_rows(*source);
    const spade::Result<const spade::WorldParams*> seed_before = source->world_params(0);
    ASSERT_OK(seed_before);
    const uint64_t old_world_seed = (*seed_before)->seed;

    ASSERT_OK(source->reseed(0xFEED'BEEF'CAFE'1234ULL));

    // The two halves, asserted separately: the SEEDS moved, the CONFIG IDENTITY
    // did not.
    const spade::Result<const spade::WorldParams*> seed_after = source->world_params(0);
    ASSERT_OK(seed_after);
    ASSERT_NE((*seed_after)->seed, old_world_seed) << "the reseed must actually have reseeded";

    const std::vector<spade::ReplayConfig> after_reseed = replay_config_rows(*source);
    ASSERT_EQ(before_reseed.size(), after_reseed.size());
    for (std::size_t w = 0; w < before_reseed.size(); ++w) {
        EXPECT_EQ(std::memcmp(&before_reseed[w], &after_reseed[w], sizeof(spade::ReplayConfig)), 0)
            << "world " << w << ": reseed() moved the config identity, which is a hash of the DESC";
    }
    EXPECT_EQ(after_reseed[0].config_hash, spade::config_hash(*set));

    ASSERT_OK(source->step(10));
    const spade::Result<SnapshotBlob> blob = source->snapshot();
    ASSERT_OK(blob);

    // The twin is built from the ORIGINAL desc and never reseeded.
    spade::Result<Simulation> twin = Simulation::create(*set, 4'000'000, 4);
    ASSERT_OK(twin);
    ASSERT_OK(twin->spawn(0, body));
    ASSERT_OK(twin->flush_structural());
    {
        const spade::Result<const spade::WorldParams*> twin_seed = twin->world_params(0);
        ASSERT_OK(twin_seed);
        ASSERT_EQ((*twin_seed)->seed, old_world_seed) << "the twin must start from the ORIGINAL seeds";
    }

    ASSERT_OK(twin->restore(*blob)) << "the config check rejected a blob from the same desc";

    // The seeds came from the blob (state), the config identity did not move.
    const spade::Result<const spade::WorldParams*> twin_seed = twin->world_params(0);
    ASSERT_OK(twin_seed);
    EXPECT_EQ((*twin_seed)->seed, (*seed_after)->seed);
    EXPECT_EQ(replay_config_rows(*twin)[0].config_hash, spade::config_hash(*set));

    // And the continuation is identical -- the whole point.
    ASSERT_OK(source->step(20));
    ASSERT_OK(twin->step(20));
    EXPECT_EQ(spade::testing::state_digest(*source), spade::testing::state_digest(*twin))
        << "a run resumed from a post-reseed blob diverged from the run that produced it";
}

// THE ONE CONSERVATIVE REJECTION THIS DESIGN ACCEPTS, pinned so that it is a
// recorded decision rather than a surprise.
//
// A world's rng root is REGISTERED STATE (WorldParams::seed), so it is restored
// from the blob -- which means a twin built from a desc whose only difference
// is its per-world seeds would in fact replay a restored blob identically.
// config_hash folds those seeds anyway (sim/world_set.hpp pins the fold order),
// so that pairing is REFUSED. The refusal is not protecting against a divergence;
// it is the hash being an identity of the DESC, and a false rejection ("you
// paired a blob with a set you did not create it from") costs a caller an
// explicit error while the opposite mistake costs them a silently wrong run.
TEST(RestoreConfigCheck, ADescDifferingOnlyInItsSeedsIsRefusedDeliberately) {
    const spade::Result<WorldSetDesc> set = config_probe_set();
    ASSERT_OK(set);
    spade::Result<Simulation> source = Simulation::create(*set, 4'000'000, 4);
    ASSERT_OK(source);
    ASSERT_OK(source->step(5));
    const spade::Result<SnapshotBlob> blob = source->snapshot();
    ASSERT_OK(blob);

    WorldSetDesc reseeded_desc = *set;
    for (WorldInstanceDesc& world : reseeded_desc.worlds) world.seed ^= 0xABCDULL;

    spade::Result<Simulation> target = Simulation::create(reseeded_desc, 4'000'000, 4);
    ASSERT_OK(target);
    // Schema-identical, so only the config check can reject it.
    ASSERT_EQ(spade::schema_hash(target->arenas().registry()), blob->schema_hash());

    const spade::Result<void> refused = target->restore(*blob);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(code_of(refused), code(spade::Code::invalid_argument));
    const std::string& context = refused.error().context;
    const std::size_t at = context.find("Diverged: ");
    ASSERT_NE(at, std::string::npos) << context;
    const std::string diverged = context.substr(at);
    EXPECT_NE(diverged.find("config_hash"), std::string::npos) << diverged;
    EXPECT_EQ(diverged.find("dt_ns"), std::string::npos) << diverged;
}

// ===========================================================================
// 12. THE SCENARIO FILE -- the corpus as data (S5 Task 7)
//
// engine/testing/scenario_file.hpp's own contract, as opposed to the corpus it
// loads. Three burdens:
//
//   * THE STRICT POSTURE IS REAL. Unknown keys, a version this build does not
//     read, a malformed number, a script that addresses something that is not
//     there -- every one of them is an ERROR, at every level. A loader that
//     merely documented that would be a loader that silently dropped a v2
//     field, which is the exact failure the world file's schema note exists to
//     prevent.
//   * THE LOADED SCENARIO IS THE FILE. The parsed data must be what the
//     document says, not what a default happened to supply.
//   * IT COMPOSES WITH THE REST OF S5. A scenario names its world by relative
//     path, and the world file's own diagnostics come through unmangled.
// ===========================================================================

namespace {

// Writes `text` to a scratch file and loads it. The corpus's own files are
// committed data and are never edited by a test; these are one-off documents
// for the negative cases, so they live in the temp directory beside the ones
// test_world_file.cpp writes.
[[nodiscard]] spade::Result<LoadedScenario> load_scratch_scenario(const std::string& stem,
                                                                  const std::string& text) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / ("spade_test_" + stem + ".scenario.yaml");
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
    }
    return spade::testing::scenario_from_yaml(path);
}

// A minimal, VALID scenario, spelled once. Every negative case below is this
// document with exactly ONE thing wrong, so no case can pass for the wrong
// reason. Its `world` is absolute (the corpus's own bounce world), because a
// scenario written to the temp directory has no ../worlds/ beside it.
[[nodiscard]] std::string minimal_scenario_text() {
    const std::string world =
        (std::filesystem::path(SPADE_GOLDEN_DIR) / "worlds" / "bounce.world.yaml").generic_string();
    return "scenario_version: 1\n"
           "name: \"scratch\"\n"
           "world: \"" +
           world +
           "\"\n"
           "instances:\n"
           "  - seed: 7\n"
           "    contacts: {restitution_e: 0.5, friction_mu: 0.25, baumgarte_beta: 0.200000003, "
           "slop: 0.00100000005, proxy_radius: 0.100000001}\n"
           "    grid: {cell_size: 0.5}\n"
           "    turbulence: light\n"
           "step: {dt_ns: 1000000, substeps: 1, steps: 10}\n"
           "spawns:\n"
           "  - world: 0\n"
           "    body:\n"
           "      pos: [0, 2, 0]\n"
           "      orient: [1, 0, 0, 0]\n"
           "      vel: [0, 0, 0]\n"
           "      omega_body: [0, 0, 0]\n"
           "      mass: 1\n"
           "      inv_inertia_diag: [100, 100, 100]\n"
           "inputs:\n"
           "  - {tick: 3, wrench: {body: 0, force: [1, 0, 0], torque: [0, 0, 0]}}\n"
           "expected_digest: \"0x0\"\n";
}

// `text` with the first occurrence of `from` replaced by `to`.
[[nodiscard]] std::string replaced(std::string text, std::string_view from, std::string_view to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << "the minimal scenario no longer contains: " << from;
    if (at == std::string::npos) return text;
    return text.replace(at, from.size(), to);
}

[[nodiscard]] std::string why_scenario(const spade::Result<LoadedScenario>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

// The minimal scenario with one `gnss_receivers` entry on its body, spelled
// `entry` (a flow mapping, or anything else a malformed case needs there).
[[nodiscard]] std::string with_gnss_receiver(std::string_view entry) {
    const std::string_view inertia = "      inv_inertia_diag: [100, 100, 100]\n";
    return replaced(minimal_scenario_text(), inertia,
                    std::string(inertia) + "      gnss_receivers:\n        - " + std::string(entry) + "\n");
}

constexpr std::string_view kGnssReceiverEntry =
    "{mount_pos: [0.25, -0.5, 0.75], rate_divider: 2, sigma_h: 1.5, sigma_v: 2.5, sigma_vel: 0.125, "
    "bias_tau_s: 1, sigma_bias: 0.5}";

}  // namespace

TEST(ScenarioFile, LoadsWhatTheDocumentSays) {
    const spade::Result<LoadedScenario> loaded =
        load_scratch_scenario("minimal", minimal_scenario_text());
    ASSERT_OK(loaded);
    const ScenarioData& data = *loaded->data;

    EXPECT_EQ(data.name, "scratch");
    EXPECT_EQ(loaded->scenario.name, "scratch");
    EXPECT_EQ(data.dt_ns, 1'000'000u);
    EXPECT_EQ(data.substeps, 1u);
    EXPECT_EQ(data.steps, 10u);
    EXPECT_EQ(loaded->scenario.dt_ns, data.dt_ns);
    EXPECT_EQ(loaded->scenario.substeps, data.substeps);
    EXPECT_EQ(loaded->scenario.steps, data.steps);

    ASSERT_EQ(data.worlds.worlds.size(), 1u);
    const WorldInstanceDesc& instance = data.worlds.worlds[0];
    EXPECT_EQ(instance.seed, 7u);
    EXPECT_EQ(instance.world.name, "ground");  // the world file, resolved
    EXPECT_EQ(instance.contacts.restitution_e, 0.5f);
    EXPECT_EQ(instance.contacts.friction_mu, 0.25f);
    EXPECT_EQ(instance.grid.cell_size, 0.5f);
    // The level shorthand is the FACTORY, not a transcription of its output.
    const spade::DrydenParams light = spade::dryden_params(TurbulenceLevel::light);
    EXPECT_EQ(0, std::memcmp(&instance.turbulence, &light, sizeof(spade::DrydenParams)));

    ASSERT_EQ(data.spawns.size(), 1u);
    EXPECT_EQ(data.spawns[0].kind, ScenarioSpawn::Kind::body);
    EXPECT_EQ(data.spawns[0].world, 0u);
    EXPECT_EQ(data.spawns[0].local_body_slot, 0u);
    EXPECT_EQ(data.spawns[0].body.pos, glm::vec3(0.0f, 2.0f, 0.0f));
    EXPECT_EQ(data.spawns[0].body.mass, 1.0f);
    EXPECT_TRUE(data.spawns[0].drag_elements.empty());
    EXPECT_TRUE(data.spawns[0].gnss_receivers.empty());  // absent means none

    ASSERT_EQ(data.inputs.size(), 1u);
    EXPECT_EQ(data.inputs[0].tick, 3u);
    EXPECT_EQ(data.inputs[0].spawn_index, 0u);
    EXPECT_EQ(data.inputs[0].force, glm::vec3(1.0f, 0.0f, 0.0f));

    // And the three closures really are wired: build() hands back the set,
    // setup() spawns, input() drives.
    const spade::Result<uint64_t> digest = spade::testing::run_scenario(loaded->scenario);
    ASSERT_OK(digest);
}

// parse_turbulence()'s OTHER branch (scenario_file.hpp): LoadsWhatTheDocumentSays
// above only exercises the level shorthand ("light" -> dryden_params()'s
// factory output). This scenario spells the mapping of all seven DrydenParams
// fields directly, which nothing in this file's positive coverage has run
// before (S6 hygiene, T7-S5 untested branch).
TEST(ScenarioFile, AcceptsAnExplicitDrydenParamsMappingForTurbulence) {
    const std::string text = replaced(
        minimal_scenario_text(), "turbulence: light",
        "turbulence: {scale_u: 150, scale_v: 175, scale_w: 40, sigma_u: 1.5, sigma_v: 2, "
        "sigma_w: 0.75, reference_airspeed: 12}");
    const spade::Result<LoadedScenario> loaded = load_scratch_scenario("explicit_dryden", text);
    ASSERT_OK(loaded);

    const spade::DrydenParams& t = loaded->data->worlds.worlds[0].turbulence;
    EXPECT_EQ(t.scale_u, 150.0f);
    EXPECT_EQ(t.scale_v, 175.0f);
    EXPECT_EQ(t.scale_w, 40.0f);
    EXPECT_EQ(t.sigma_u, 1.5f);
    EXPECT_EQ(t.sigma_v, 2.0f);
    EXPECT_EQ(t.sigma_w, 0.75f);
    EXPECT_EQ(t.reference_airspeed, 12.0f);
    EXPECT_EQ(t._reserved0, 0.0f);  // the file never carries it -- must stay 0

    const spade::Result<uint64_t> digest = spade::testing::run_scenario(loaded->scenario);
    ASSERT_OK(digest);
}

// parse_drag_elements()'s "componentwise" branch (scenario_file.hpp):
// LoadsWhatTheDocumentSays' spawn carries no `drag_elements` key at all
// (data.spawns[0].drag_elements.empty() there), so nothing in this file's
// positive coverage has ever populated one from a scenario document (S6
// hygiene, T7-S5 untested branch).
TEST(ScenarioFile, AcceptsComponentwiseDragElementsOnABodySpawn) {
    const std::string text = replaced(
        minimal_scenario_text(), "      inv_inertia_diag: [100, 100, 100]\n",
        "      inv_inertia_diag: [100, 100, 100]\n"
        "      drag_elements:\n"
        "        - {mode: componentwise, area: 0, coeffs: [0.02, 0.03, 0.04], "
        "local_pos: [0.1, 0, -0.1], local_orient: [1, 0, 0, 0]}\n");
    const spade::Result<LoadedScenario> loaded = load_scratch_scenario("componentwise_drag", text);
    ASSERT_OK(loaded);

    ASSERT_EQ(loaded->data->spawns.size(), 1u);
    ASSERT_EQ(loaded->data->spawns[0].drag_elements.size(), 1u);
    const spade::DragElementSpawn& drag = loaded->data->spawns[0].drag_elements[0];
    EXPECT_EQ(drag.mode, spade::physics::drag_mode::componentwise);
    EXPECT_EQ(drag.area, 0.0f);
    EXPECT_EQ(drag.coeffs, glm::vec3(0.02f, 0.03f, 0.04f));
    EXPECT_EQ(drag.local_pos, glm::vec3(0.1f, 0.0f, -0.1f));
    EXPECT_EQ(drag.local_orient.w, 1.0f);
    EXPECT_EQ(drag.local_orient.x, 0.0f);
    EXPECT_EQ(drag.local_orient.y, 0.0f);
    EXPECT_EQ(drag.local_orient.z, 0.0f);

    // And the whole pipeline actually runs with the element wired in, not
    // just parsed.
    const spade::Result<uint64_t> digest = spade::testing::run_scenario(loaded->scenario);
    ASSERT_OK(digest);
}

// parse_gnss_receivers() (PHY-6): every field read exactly, and the receiver
// actually ATTACHED by setup -- live, carrying those values, and emitting on
// its own clock (10 substeps at rate_divider 2 is five fixes).
TEST(ScenarioFile, AttachesGnssReceiversToABodySpawn) {
    const spade::Result<LoadedScenario> loaded =
        load_scratch_scenario("gnss_receiver", with_gnss_receiver(kGnssReceiverEntry));
    ASSERT_OK(loaded);

    ASSERT_EQ(loaded->data->spawns.size(), 1u);
    ASSERT_EQ(loaded->data->spawns[0].gnss_receivers.size(), 1u);
    const spade::GnssSensorSpawn& parsed = loaded->data->spawns[0].gnss_receivers[0];
    EXPECT_EQ(parsed.mount_pos, glm::vec3(0.25f, -0.5f, 0.75f));
    EXPECT_EQ(parsed.rate_divider, 2u);
    EXPECT_EQ(parsed.sigma_h, 1.5f);
    EXPECT_EQ(parsed.sigma_v, 2.5f);
    EXPECT_EQ(parsed.sigma_vel, 0.125f);
    EXPECT_EQ(parsed.bias_tau_s, 1.0f);
    EXPECT_EQ(parsed.sigma_bias, 0.5f);

    spade::Result<Simulation> sim = spade::testing::start_scenario(loaded->scenario);
    ASSERT_OK(sim);
    ASSERT_OK(spade::testing::advance_scenario(loaded->scenario, *sim, loaded->scenario.steps));
    const spade::Result<uint32_t> live = sim->live_gnss_sensor_count(0);
    ASSERT_OK(live);
    ASSERT_EQ(*live, 1u);

    const auto rows = sim->arenas().world_slice(sim->gnss_sensors_array(), 0);
    ASSERT_OK(rows);
    const auto row = std::find_if(rows->begin(), rows->end(),
                                  [](const spade::sensors::GnssSensorRow& r) { return r.kind != 0u; });
    ASSERT_NE(row, rows->end());
    EXPECT_EQ(row->mount_pos, parsed.mount_pos);
    EXPECT_EQ(row->rate_divider, 2u);
    EXPECT_EQ(row->sigma_h, 1.5f);
    EXPECT_EQ(row->sigma_v, 2.5f);
    EXPECT_EQ(row->sigma_vel, 0.125f);
    EXPECT_EQ(row->bias_tau_s, 1.0f);
    EXPECT_EQ(row->sigma_bias, 0.5f);
    EXPECT_EQ(row->last_index, 5u) << "10 substeps at rate_divider 2";
}

// A receiver whose SHAPE is right but whose values the engine refuses loads,
// and is then refused by add_gnss_sensor() when setup attaches it -- never
// dropped. The loader deliberately does not re-check the values (one site per
// invariant), so this is where that refusal is shown to arrive.
TEST(ScenarioFile, AGnssReceiverTheEngineRefusesFailsSetupRatherThanVanishing) {
    const spade::Result<LoadedScenario> loaded = load_scratch_scenario(
        "gnss_refused", with_gnss_receiver(replaced(std::string(kGnssReceiverEntry), "rate_divider: 2",
                                                    "rate_divider: 0")));
    ASSERT_OK(loaded);
    ASSERT_EQ(loaded->data->spawns[0].gnss_receivers.size(), 1u);

    const spade::Result<Simulation> sim = spade::testing::start_scenario(loaded->scenario);
    ASSERT_FALSE(sim.has_value()) << "setup attached a receiver with rate_divider 0";
    EXPECT_EQ(code_of(sim), code(spade::Code::invalid_argument)) << sim.error().context;
}

// EVERY REJECTION: one document, one defect each.
TEST(ScenarioFile, RejectsEveryMalformedDocument) {
    const std::string base = minimal_scenario_text();

    struct Case {
        const char* what;
        std::string text;
        spade::Code expected_code;
        const char* names;  // a substring the diagnostic must carry
    };
    const Case cases[] = {
        // A v2 file is answered BY VERSION, not by the first key a v1 reader
        // does not recognize -- the one failure a caller may answer by
        // upgrading rather than by editing.
        {"a future schema version", replaced(base, "scenario_version: 1", "scenario_version: 2"),
         spade::Code::schema_mismatch, "version 2"},
        {"an unknown top-level key", base + "extra_key: 3\n", spade::Code::invalid_argument,
         "unknown key 'extra_key'"},
        // ...and blames THIS file's schema: the shared check_map
        // (world/detail/yaml_text.hpp) is told which schema by its caller.
        {"an unknown key names the scenario schema", base + "extra_key: 3\n",
         spade::Code::invalid_argument, "scenario schema v1 does not define it"},
        {"an unknown nested key",
         replaced(base, "grid: {cell_size: 0.5}", "grid: {cell_size: 0.5, cell_pad: 1}"),
         spade::Code::invalid_argument, "unknown key 'cell_pad'"},
        // check_map()'s OTHER rejection (scenario_file.hpp's own comment:
        // "Unknown keys and duplicate keys are both rejected") -- exercised
        // nowhere else in this file, which tested only the unknown-key branch
        // above.
        {"a duplicate key", replaced(base, "grid: {cell_size: 0.5}", "grid: {cell_size: 0.5, cell_size: 0.6}"),
         spade::Code::invalid_argument, "duplicate key 'cell_size'"},
        {"a missing required key", replaced(base, "    grid: {cell_size: 0.5}\n", ""),
         spade::Code::invalid_argument, "missing required key 'grid'"},
        {"an unknown turbulence level", replaced(base, "turbulence: light", "turbulence: gentle"),
         spade::Code::invalid_argument, "unknown turbulence level 'gentle'"},
        {"a malformed float", replaced(base, "cell_size: 0.5}", "cell_size: 0.5f}"),
         spade::Code::invalid_argument, "is not a decimal number"},
        {"a non-finite float", replaced(base, "cell_size: 0.5}", "cell_size: inf}"),
         spade::Code::invalid_argument, "is not finite"},
        {"a spawn naming a world that is not in the set",
         replaced(base, "  - world: 0", "  - world: 4"), spade::Code::invalid_argument,
         "outside this scenario's 1 worlds"},
        {"a spawn carrying neither a body nor a vehicle",
         replaced(base,
                  "    body:\n"
                  "      pos: [0, 2, 0]\n"
                  "      orient: [1, 0, 0, 0]\n"
                  "      vel: [0, 0, 0]\n"
                  "      omega_body: [0, 0, 0]\n"
                  "      mass: 1\n"
                  "      inv_inertia_diag: [100, 100, 100]\n",
                  ""),
         spade::Code::invalid_argument, "exactly one of 'body'"},
        {"an input past the end of the run", replaced(base, "{tick: 3,", "{tick: 10,"),
         spade::Code::invalid_argument, "would never be applied"},
        {"an input addressing a spawn that is not there",
         replaced(base, "wrench: {body: 0,", "wrench: {body: 2,"), spade::Code::invalid_argument,
         "outside this scenario's 1 spawns"},
        {"rotor commands aimed at a bare body",
         replaced(base, "wrench: {body: 0, force: [1, 0, 0], torque: [0, 0, 0]}",
                  "rotor_commands: {vehicle: 0, omega: [1, 1, 1, 1]}"),
         spade::Code::invalid_argument, "is a bare body"},
        {"a vehicle spawn with no model to name",
         replaced(base,
                  "    body:\n"
                  "      pos: [0, 2, 0]\n"
                  "      orient: [1, 0, 0, 0]\n"
                  "      vel: [0, 0, 0]\n"
                  "      omega_body: [0, 0, 0]\n"
                  "      mass: 1\n"
                  "      inv_inertia_diag: [100, 100, 100]\n",
                  "    vehicle:\n"
                  "      model: 0\n"
                  "      pos: [0, 2, 0]\n"
                  "      orient: [1, 0, 0, 0]\n"
                  "      vel: [0, 0, 0]\n"
                  "      omega_body: [0, 0, 0]\n"
                  "      rotor_omega: hover\n"),
         spade::Code::invalid_argument, "0 declared models"},
        // gnss_receivers (PHY-6): optional, but strict once present.
        {"a GNSS receiver with an unknown key",
         with_gnss_receiver(replaced(std::string(kGnssReceiverEntry), "sigma_bias: 0.5}",
                                     "sigma_bias: 0.5, sigma_clock: 1}")),
         spade::Code::invalid_argument, "unknown key 'sigma_clock'"},
        {"a GNSS receiver missing a key",
         with_gnss_receiver(replaced(std::string(kGnssReceiverEntry), ", sigma_bias: 0.5", "")),
         spade::Code::invalid_argument, "missing required key 'sigma_bias'"},
        {"GNSS receivers that are not a sequence",
         replaced(with_gnss_receiver(kGnssReceiverEntry), "      gnss_receivers:\n        - ",
                  "      gnss_receivers: "),
         spade::Code::invalid_argument, "gnss_receivers must be a sequence"},
        {"a GNSS receiver with a negative rate divider",
         with_gnss_receiver(replaced(std::string(kGnssReceiverEntry), "rate_divider: 2", "rate_divider: -2")),
         spade::Code::invalid_argument, "is not a non-negative decimal or 0x-hex integer"},
        {"a GNSS receiver with a non-finite sigma",
         with_gnss_receiver(replaced(std::string(kGnssReceiverEntry), "sigma_h: 1.5", "sigma_h: nan")),
         spade::Code::invalid_argument, "is not finite"},
        // Body spawns only: a vehicle's sensors come from its model type.
        {"GNSS receivers on a vehicle spawn",
         replaced(base,
                  "    body:\n"
                  "      pos: [0, 2, 0]\n"
                  "      orient: [1, 0, 0, 0]\n"
                  "      vel: [0, 0, 0]\n"
                  "      omega_body: [0, 0, 0]\n"
                  "      mass: 1\n"
                  "      inv_inertia_diag: [100, 100, 100]\n",
                  "    vehicle:\n"
                  "      model: 0\n"
                  "      pos: [0, 2, 0]\n"
                  "      orient: [1, 0, 0, 0]\n"
                  "      vel: [0, 0, 0]\n"
                  "      omega_body: [0, 0, 0]\n"
                  "      rotor_omega: hover\n"
                  "      gnss_receivers: []\n"),
         spade::Code::invalid_argument, "unknown key 'gnss_receivers'"},
        {"a world file that is not there",
         replaced(base, "bounce.world.yaml", "no_such_world.world.yaml"), spade::Code::io_error,
         "no_such_world.world.yaml"},
        {"a top level that is not a mapping", "- 1\n- 2\n", spade::Code::invalid_argument,
         "must be a mapping"},
        {"YAML that does not parse", "scenario_version: [1\n", spade::Code::invalid_argument,
         "yaml-cpp"},
    };

    for (const Case& c : cases) {
        const spade::Result<LoadedScenario> loaded = load_scratch_scenario("malformed", c.text);
        ASSERT_FALSE(loaded.has_value()) << c.what << ": the loader accepted it";
        EXPECT_EQ(code_of(loaded), code(c.expected_code)) << c.what << ": " << why_scenario(loaded);
        EXPECT_NE(why_scenario(loaded).find(c.names), std::string::npos)
            << c.what << ": the diagnostic does not name what is wrong: " << why_scenario(loaded);
    }
}

TEST(ScenarioFile, AMissingFileIsAnIoErrorCarryingThePath) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "spade_test_no_such.scenario.yaml";
    std::filesystem::remove(path);
    const spade::Result<LoadedScenario> loaded = spade::testing::scenario_from_yaml(path);
    EXPECT_EQ(code_of(loaded), code(spade::Code::io_error));
    EXPECT_NE(why_scenario(loaded).find("spade_test_no_such"), std::string::npos)
        << why_scenario(loaded);
}

// The corpus's `world` values are relative, and this is what that means:
// relative to the SCENARIO FILE, not to the working directory. That the four
// migrated scenarios resolve at all is already proven by the corpus tests; this
// pins the RULE, so a loader that quietly started resolving against the CWD
// would fail here rather than only on someone else's machine.
TEST(ScenarioFile, TheWorldPathIsRelativeToTheScenarioFile) {
    const spade::Result<LoadedScenario> loaded = load_scenario("bounce");
    ASSERT_OK(loaded);
    EXPECT_EQ(loaded->data->world_path.lexically_normal(),
              (std::filesystem::path(SPADE_GOLDEN_DIR) / "worlds" / "bounce.world.yaml")
                  .lexically_normal());
    EXPECT_EQ(loaded->data->worlds.worlds[0].world.name, "ground");
}

// ===========================================================================
// 13. quad_hover -- the vehicle scenario, and the two tickets it closes
// ===========================================================================

// ---------------------------------------------------------------------------
// TICKET T18-M3: TWO-WORLD VEHICLE ISOLATION.
//
// Batching invariance was proven for BODIES at S1-S4 (section 5 above). A
// vehicle carries more: rotor rows and sensor rows in their own arenas, each
// naming its body by a WORLD-LOCAL slot, each read by a pass that iterates the
// whole array. A rotor row addressed by global slot where a world-local one was
// meant, or a sensor ring window computed from the wrong partition, would leak
// between worlds -- and nothing in the body-only corpus would notice.
//
// The shape is the batching-invariance one: each world of the two-world run,
// against that same world run ALONE. Both worlds, not just one, because a leak
// is directional (world 0 reading world 1's rows is a different bug from the
// reverse) and because the two carry different seeds.
// ---------------------------------------------------------------------------
TEST(QuadHover, AWorldsVehicleDoesNotDependOnWhatElseIsInTheSet) {
    const spade::Result<LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_OK(loaded);
    ASSERT_EQ(loaded->data->worlds.worlds.size(), 2u);

    spade::Result<Simulation> both = spade::testing::start_scenario(loaded->scenario);
    ASSERT_OK(both);
    ASSERT_OK(spade::testing::advance_scenario(loaded->scenario, *both, loaded->scenario.steps));

    for (uint32_t w = 0; w < 2; ++w) {
        const spade::Result<Simulation> alone = run_world_alone(*loaded->data, w);
        ASSERT_OK(alone) << "world " << w;
        EXPECT_EQ(spade::testing::world_digest(*alone, 0), spade::testing::world_digest(*both, w))
            << "world " << w << ": a quadrotor's trajectory depended on what else was in the set";
    }

    // The comparison is only worth anything if the two worlds actually differ
    // -- two identical worlds would make a cross-world leak invisible.
    EXPECT_NE(spade::testing::world_digest(*both, 0), spade::testing::world_digest(*both, 1))
        << "the two quad_hover worlds produced identical states, so this test cannot see a leak";
}

// ---------------------------------------------------------------------------
// TICKET T18-M2: vehicle_ref_at() THROUGH A RESTORE. This is the test the new
// API exists for.
//
// The replay guarantee for a VEHICLE is strictly harder than for a body,
// because the input script does not merely read state -- it COMMANDS actuators,
// through a VehicleRef whose rotor slots it has to have. A ref captured at
// setup is stale after a restore (the restore rewinds the generation counters),
// so a script that held one would either be rejected or, worse, address slots
// that now belong to something else. The scenario's script therefore re-derives
// its ref from the state at every command, and this is the proof that doing so
// lands exactly where the uninterrupted run did.
//
// TICK 450 IS MID-SCRIPT ON PURPOSE: between the +10% collective step at 300
// and the -10% one at 600, so the resumed run must apply a command the original
// run applied AFTER the snapshot, to a vehicle whose ref it did not inherit.
// ---------------------------------------------------------------------------
TEST(QuadHover, ResumesMidScriptInAFreshSimulationThroughVehicleRefAt) {
    const spade::Result<LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_OK(loaded);
    const Scenario& scenario = loaded->scenario;
    constexpr uint64_t kResumeTick = 450;
    ASSERT_LT(kResumeTick, scenario.steps);

    // The uninterrupted reference run.
    spade::Result<Simulation> reference = spade::testing::start_scenario(scenario);
    ASSERT_OK(reference);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *reference, scenario.steps));
    const uint64_t expected = spade::testing::state_digest(*reference);

    // The interrupted run: stop mid-script, snapshot.
    spade::Result<Simulation> interrupted = spade::testing::start_scenario(scenario);
    ASSERT_OK(interrupted);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *interrupted, kResumeTick));
    const spade::Result<SnapshotBlob> blob = interrupted->snapshot();
    ASSERT_OK(blob);
    EXPECT_EQ(blob->tick().value, kResumeTick);

    // A FRESH Simulation. start_scenario() re-registers the model -- the
    // registry is configuration and a restore does not rebuild it -- and
    // re-spawns, so the refs it mints are its own.
    spade::Result<Simulation> resumed = spade::testing::start_scenario(scenario);
    ASSERT_OK(resumed);

    // THE REF THE SPAWN HANDED BACK IS STALE THE MOMENT THE RESTORE LANDS, and
    // the generation is where that shows. Captured before, compared after.
    const spade::Result<spade::VehicleRef> before = resumed->vehicle_ref_at(0, 0);
    ASSERT_OK(before);
    ASSERT_OK(resumed->restore(*blob));
    EXPECT_EQ(resumed->tick().value, kResumeTick);

    const spade::Result<spade::VehicleRef> after = resumed->vehicle_ref_at(0, 0);
    ASSERT_OK(after);
    EXPECT_EQ(after->body.slot, before->body.slot) << "the vehicle did not move slots";
    EXPECT_EQ(after->rotor_count, 4u);
    EXPECT_EQ(after->imu_count, 1u);

    // And the finish lands on the uninterrupted run's digest -- which it can
    // only do if every rotor command from tick 450 onward reached the right
    // four rows.
    ASSERT_OK(spade::testing::advance_scenario(scenario, *resumed, scenario.steps));
    EXPECT_EQ(spade::testing::state_digest(*resumed), expected)
        << "a vehicle scenario resumed from a mid-script snapshot diverged";
}

// ---------------------------------------------------------------------------
// THE SCRIPT DOES WHAT ITS NAME SAYS. A digest pins a trajectory perfectly
// whether or not the rotor commands ever reached a rotor: a scenario whose
// set_rotor_commands() calls were all quietly dropped would still be
// deterministic, still reproduce, and still be worthless.
//
// MEASURED AGAINST A COUNTERFACTUAL rather than against a threshold, because
// each world's absolute vertical velocity is dominated by the gust bias it drew
// (see the sanity test's note) and any absolute bound would be a number chosen
// to fit today's trajectory. The counterfactual is the SAME scenario with its
// input script removed -- which is not a different experiment but the same one
// with the commands taken out, because VehicleSpawn::rotor_omega is both the
// shaft speed a rotor spawns at AND the command it spawns holding, and a rotor
// command is PERSISTENT. So the held run flies trim for all 900 steps through
// the same worlds, the same seeds and the same gusts, and every difference
// between the two IS the script.
// ---------------------------------------------------------------------------
TEST(QuadHover, TheCollectiveStepsAreWhatMovesTheQuadrotor) {
    const spade::Result<LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_OK(loaded);
    const Scenario& scripted = loaded->scenario;

    Scenario held = scripted;
    held.input = {};  // no commands at all -- see above

    spade::Result<Simulation> a = spade::testing::start_scenario(scripted);
    ASSERT_OK(a);
    spade::Result<Simulation> b = spade::testing::start_scenario(held);
    ASSERT_OK(b);

    // Sampled at the two phase boundaries the script steps on.
    float vy_a[2][2] = {};
    float vy_b[2][2] = {};
    const uint64_t marks[2] = {600, 900};
    for (std::size_t phase = 0; phase < 2; ++phase) {
        ASSERT_OK(spade::testing::advance_scenario(scripted, *a, marks[phase]));
        ASSERT_OK(spade::testing::advance_scenario(held, *b, marks[phase]));
        for (uint32_t w = 0; w < 2; ++w) {
            const spade::Result<std::span<const spade::BodyState>> bodies_a = a->world_bodies(w);
            ASSERT_OK(bodies_a);
            vy_a[w][phase] = (*bodies_a)[0].vel.y;
            const spade::Result<std::span<const spade::BodyState>> bodies_b = b->world_bodies(w);
            ASSERT_OK(bodies_b);
            vy_b[w][phase] = (*bodies_b)[0].vel.y;
        }
    }

    for (uint32_t w = 0; w < 2; ++w) {
        // 0.6 s at +10% collective is +21% thrust, about +2 m/s^2, so the
        // scripted run should be roughly 1.2 m/s faster upward at tick 600 than
        // the one that held trim. Asserted at half of that -- far above any
        // second-order term, and far from the zero a dropped command gives.
        EXPECT_GT(vy_a[w][0], vy_b[w][0] + 0.6f)
            << "world " << w << ": the +10% collective at tick 300 did not lift the quadrotor "
            << "relative to the run that held trim (scripted vy " << vy_a[w][0] << ", held "
            << vy_b[w][0] << ")";
        // And the -10% step at 600 turns that around: over ticks 600..900 the
        // scripted run must LOSE vertical speed relative to the held one.
        EXPECT_LT(vy_a[w][1] - vy_a[w][0], vy_b[w][1] - vy_b[w][0] - 0.6f)
            << "world " << w << ": the -10% collective at tick 600 did not decelerate the "
            << "quadrotor relative to the run that held trim";
    }

    // The script changed the run at all -- the floor under both comparisons.
    EXPECT_NE(spade::testing::state_digest(*a), spade::testing::state_digest(*b))
        << "removing the input script changed nothing: the rotor commands never reached a rotor";
}

// The file says `hover` and the tick-0 command says a number; this is the
// assertion that they are the same number, bit for bit. Without it the
// scenario's "hold trim, then step the collective" narrative would be a claim
// about a value nobody checked -- and a retuned airframe would silently start
// its script with a step it does not describe.
TEST(QuadHover, TheSpawnTrimAndTheTickZeroCommandAreTheSameShaftSpeed) {
    const spade::Result<LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_OK(loaded);
    const ScenarioData& data = *loaded->data;

    ASSERT_EQ(data.spawns.size(), 2u);
    ASSERT_EQ(data.spawns[0].kind, ScenarioSpawn::Kind::vehicle);
    ASSERT_FALSE(data.inputs.empty());
    ASSERT_EQ(data.inputs[0].tick, 0u);
    ASSERT_EQ(data.inputs[0].omega.size(), 4u);

    const float trim = data.spawns[0].vehicle.rotor_omega;
    EXPECT_GT(trim, 0.0f) << "`rotor_omega: hover` resolved to nothing";
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(data.inputs[0].omega[i], trim) << "rotor " << i;
    }
    // Both vehicles spawn at the same trim: they are the same airframe.
    EXPECT_EQ(data.spawns[1].vehicle.rotor_omega, trim);
}

// ===========================================================================
// 14. vehicle_ref_at -- the API itself (ticket T18-M2)
//
// quad_hover proves it works across a restore, which is what it is FOR. These
// pin what it means, which quad_hover cannot: what the ordinal counts, what a
// bare body is to it, and that the ref it re-derives is the ref spawn() issued.
// ===========================================================================

TEST(VehicleRefAt, RederivesExactlyTheRefTheSpawnIssued) {
    const spade::Result<LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_OK(loaded);

    spade::Result<Simulation> sim = Simulation::create(loaded->data->worlds, loaded->data->dt_ns,
                                                       loaded->data->substeps);
    ASSERT_OK(sim);
    const spade::Result<spade::ModelTypeId> model = sim->register_model(loaded->data->models[0]);
    ASSERT_OK(model);

    const spade::Result<spade::VehicleRef> spawned =
        sim->spawn(1, *model, loaded->data->spawns[1].vehicle);
    ASSERT_OK(spawned);
    ASSERT_OK(sim->flush_structural());

    const spade::Result<spade::VehicleRef> rederived = sim->vehicle_ref_at(1, 0);
    ASSERT_OK(rederived);
    EXPECT_EQ(rederived->body, spawned->body);
    ASSERT_EQ(rederived->rotor_count, spawned->rotor_count);
    for (uint32_t i = 0; i < spawned->rotor_count; ++i) {
        EXPECT_EQ(rederived->rotor_slots[i], spawned->rotor_slots[i]) << "rotor " << i;
    }
    ASSERT_EQ(rederived->imu_count, spawned->imu_count);
    for (uint32_t i = 0; i < spawned->imu_count; ++i) {
        EXPECT_EQ(rederived->imu_sensors[i], spawned->imu_sensors[i]) << "sensor " << i;
    }

    // `model` IS THE ONE FIELD IT CANNOT KNOW, and that is documented rather
    // than papered over: no ModelTypeId appears anywhere in the state, so there
    // is nothing to read it back from. Pinned so the null is a decision.
    EXPECT_TRUE(rederived->model.is_null())
        << "vehicle_ref_at guessed a model id; the registry is configuration, not state";

    // The re-derived ref is USABLE, which is the only thing the null costs
    // nothing on: every entry point taking a VehicleRef validates `body` and
    // indexes the slots.
    const std::array<float, 4> hold{500.0f, 500.0f, 500.0f, 500.0f};
    EXPECT_OK(sim->set_rotor_commands(*rederived, hold));
    const spade::Result<const spade::vehicles::RotorRow*> row = sim->rotor(*rederived, 2);
    ASSERT_OK(row);
    EXPECT_EQ((*row)->omega_cmd, 500.0f);
}

TEST(VehicleRefAt, CountsLiveVehiclesInSlotOrderAndIgnoresBareBodies) {
    // A world with room for a quadrotor AND a couple of bare bodies, so the two
    // populations can be interleaved.
    const spade::Result<LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_OK(loaded);
    WorldSetDesc set = loaded->data->worlds;
    set.worlds.resize(1);
    set.worlds[0].world.capacities.bodies = 4;
    set.worlds[0].world.capacities.force_elements = 11;  // two quads (5 each) + one drag element
    set.worlds[0].world.capacities.sensors = 2;          // one IMU per quadrotor

    spade::Result<Simulation> sim = Simulation::create(set, 2'000'000, 2);
    ASSERT_OK(sim);
    const spade::Result<spade::ModelTypeId> model = sim->register_model(loaded->data->models[0]);
    ASSERT_OK(model);

    spade::VehicleSpawn where = loaded->data->spawns[0].vehicle;

    // Slot 0: a bare body. Slot 1: a vehicle. Slot 2: a bare body WITH a drag
    // element (so "owns a force element" is not what makes a vehicle). Slot 3:
    // a second vehicle.
    BodySpawn bare;
    bare.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    bare.mass = 1.0f;
    ASSERT_OK(sim->spawn(0, bare));
    const spade::Result<spade::VehicleRef> first = sim->spawn(0, *model, where);
    ASSERT_OK(first);
    const spade::Result<BodyRef> dragged = sim->spawn(0, bare);
    ASSERT_OK(dragged);
    DragElementSpawn drag;
    drag.mode = spade::physics::drag_mode::componentwise;
    drag.coeffs = glm::vec3(0.1f);
    ASSERT_OK(sim->add_drag_element(*dragged, drag));
    where.pos = glm::vec3(5.0f, 10.0f, 0.0f);
    const spade::Result<spade::VehicleRef> second = sim->spawn(0, *model, where);
    ASSERT_OK(second);
    ASSERT_OK(sim->flush_structural());

    // Two vehicles, in ascending body-slot order -- which is creation order on
    // an unfragmented partition.
    const spade::Result<spade::VehicleRef> zero = sim->vehicle_ref_at(0, 0);
    ASSERT_OK(zero);
    EXPECT_EQ(zero->body, first->body);
    const spade::Result<spade::VehicleRef> one = sim->vehicle_ref_at(0, 1);
    ASSERT_OK(one);
    EXPECT_EQ(one->body, second->body);

    // The two bare bodies -- including the one carrying a force element -- are
    // not vehicles and do not consume an ordinal.
    EXPECT_EQ(code_of(sim->vehicle_ref_at(0, 2)), code(spade::Code::not_found));
    EXPECT_EQ(code_of(sim->vehicle_ref_at(0, 7)), code(spade::Code::not_found));
    EXPECT_EQ(code_of(sim->vehicle_ref_at(1, 0)), code(spade::Code::invalid_argument))
        << "a world outside the set is not a lookup miss";

    // AND A DESPAWN RENUMBERS, which is the honest cost of an ordinal being a
    // census rather than an address (see the header). The generation parity
    // makes the first vehicle dead the moment despawn() returns, so the second
    // becomes vehicle 0 immediately -- before the step boundary, not after.
    ASSERT_OK(sim->despawn(first->body));
    const spade::Result<spade::VehicleRef> renumbered = sim->vehicle_ref_at(0, 0);
    ASSERT_OK(renumbered);
    EXPECT_EQ(renumbered->body, second->body);
    EXPECT_EQ(code_of(sim->vehicle_ref_at(0, 1)), code(spade::Code::not_found));
}

TEST(VehicleRefAt, FindsAVehicleWhoseSpawnHasNotBeenFlushedYet) {
    // The same posture body_ref_at has, and for the same reason: the row's
    // identity is written at RESERVATION time, so the vehicle is discoverable
    // immediately, and the "not yet flushed" verdict is left to the call the
    // caller then makes -- which reports it by name instead of leaving the
    // caller to wonder why the lookup failed.
    const spade::Result<LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_OK(loaded);
    spade::Result<Simulation> sim = Simulation::create(loaded->data->worlds, loaded->data->dt_ns,
                                                       loaded->data->substeps);
    ASSERT_OK(sim);
    const spade::Result<spade::ModelTypeId> model = sim->register_model(loaded->data->models[0]);
    ASSERT_OK(model);
    ASSERT_OK(sim->spawn(0, *model, loaded->data->spawns[0].vehicle));

    const spade::Result<spade::VehicleRef> pending = sim->vehicle_ref_at(0, 0);
    ASSERT_OK(pending);
    EXPECT_EQ(pending->rotor_count, 4u);
    const std::array<float, 4> hold{400.0f, 400.0f, 400.0f, 400.0f};
    EXPECT_EQ(code_of(sim->set_rotor_commands(*pending, hold)), code(spade::Code::not_found));

    ASSERT_OK(sim->flush_structural());
    EXPECT_OK(sim->set_rotor_commands(*pending, hold));
}

// ===========================================================================
// 15. The runner respects Task 3's replay_config
//
// A data scenario carries its own step block, and Simulation::create() folds
// (dt_ns, substeps) into the replay_config row that rides every blob. So a blob
// taken from a scenario run cannot be restored into a Simulation running that
// same world set under a DIFFERENT step decomposition -- which is precisely the
// mistake a data corpus makes newly easy to make, because the step block is now
// a field in a file that somebody can edit.
//
// The rejection itself is Task 3's and is pinned in section 11 above; what this
// adds is that the SCENARIO PATH goes through it, with the scenario's own
// worlds on both sides so nothing else can be what rejects.
// ===========================================================================

TEST(RestoreConfigCheck, RejectsAScenarioBlobRestoredUnderADifferentStepBlock) {
    const spade::Result<LoadedScenario> loaded = load_scenario("two_world_isolation");
    ASSERT_OK(loaded);
    const ScenarioData& data = *loaded->data;
    ASSERT_EQ(data.dt_ns, 2'000'000u);
    ASSERT_EQ(data.substeps, 2u);

    spade::Result<Simulation> source = spade::testing::start_scenario(loaded->scenario);
    ASSERT_OK(source);
    ASSERT_OK(spade::testing::advance_scenario(loaded->scenario, *source, 20));
    const spade::Result<SnapshotBlob> blob = source->snapshot();
    ASSERT_OK(blob);

    struct Case {
        const char* what;
        uint64_t dt_ns;
        uint32_t substeps;
        const char* names;  // the one field the message must name
    };
    const Case cases[] = {
        {"a doubled step duration", data.dt_ns * 2, data.substeps, "dt_ns"},
        {"a halved substep count", data.dt_ns, data.substeps / 2, "substeps"},
    };
    const char* const all_fields[] = {"dt_ns", "substeps", "config_hash"};

    for (const Case& c : cases) {
        // THE SCENARIO'S OWN WORLDS on both sides -- same geometry, same seeds,
        // same materials -- so the config_hash agrees and only the step block
        // can be what rejects.
        spade::Result<Simulation> target = Simulation::create(data.worlds, c.dt_ns, c.substeps);
        ASSERT_OK(target) << c.what;
        ASSERT_EQ(spade::schema_hash(target->arenas().registry()), blob->schema_hash())
            << c.what << ": the two runs must be schema-identical, or this case is vacuous";

        const uint64_t before = spade::testing::state_digest(*target);
        const spade::Result<void> refused = target->restore(*blob);
        ASSERT_FALSE(refused.has_value()) << c.what;
        EXPECT_EQ(code_of(refused), code(spade::Code::invalid_argument)) << c.what;

        const std::string& context = refused.error().context;
        const std::size_t at = context.find("Diverged: ");
        ASSERT_NE(at, std::string::npos) << c.what << ": got: " << context;
        const std::string diverged = context.substr(at);
        for (const char* field : all_fields) {
            const bool named = diverged.find(field) != std::string::npos;
            EXPECT_EQ(named, std::string(field) == c.names)
                << c.what << ": '" << field << "' named=" << named << " in: " << diverged;
        }
        EXPECT_EQ(spade::testing::state_digest(*target), before)
            << c.what << ": a rejected restore changed registered state";
    }

    // AND THE MATCHING PAIRING STILL WORKS -- otherwise this test would pass
    // just as happily against a restore() that refused everything.
    spade::Result<Simulation> twin =
        Simulation::create(data.worlds, data.dt_ns, data.substeps);
    ASSERT_OK(twin);
    EXPECT_OK(twin->restore(*blob));
}


