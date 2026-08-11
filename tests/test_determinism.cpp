#include <gtest/gtest.h>

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <sstream>
#include <string>
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
// Plus the corpus itself: four scenarios whose digests are committed to
// tests/golden/*.digest and re-asserted here, so that an unintended change to
// ANY pinned op order, rng construction, layout offset or schedule position
// fails a test in this file rather than surfacing as a parity mystery at S6.
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

// Refs captured by a scenario's setup and read by its input script. Shared so
// the two lambdas -- which are copied into std::functions -- see one table.
// Cleared by setup(), so re-running a Scenario object is safe.
using RefTable = std::shared_ptr<std::vector<std::vector<BodyRef>>>;

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
// Scenario 1 -- ballistic. One body, no geometry at all, but the full force
// path: moderate turbulence (so the Dryden filter advances and is sampled), a
// quadratic drag element reading the medium, and a scripted wrench every tick.
// The purest determinism test: nothing here can be stabilized by a contact
// clamping a divergence away.
// ---------------------------------------------------------------------------
[[nodiscard]] Scenario ballistic_scenario() {
    RefTable refs = std::make_shared<std::vector<std::vector<BodyRef>>>();

    Scenario s;
    s.name = "ballistic";
    s.dt_ns = 5'000'000;  // 5 ms step
    s.substeps = 5;       // 1 ms substep
    s.steps = 200;        // one second

    s.build = []() -> spade::Result<WorldSetDesc> {
        // No SDF nodes at all: an empty program evaluates to kSdfEmptyDistance,
        // so CollisionStatic finds nothing and the trajectory is pure force.
        const spade::Result<spade::WorldDesc> world =
            WorldBuilder().name("void").environment(default_environment()).capacities(capacities(4, 4)).build();
        if (!world) return std::unexpected(world.error());

        WorldInstanceDesc instance;
        instance.world = *world;
        instance.seed = 0xB0117571C0FFEEULL;
        instance.turbulence = spade::dryden_params(TurbulenceLevel::moderate);
        instance.contacts = contacts(0.0f, 0.0f, 0.1f);
        instance.grid = grid(0.5f);
        return WorldSetDesc{{instance}};
    };

    s.setup = [refs](Simulation& sim) -> spade::Result<void> {
        refs->assign(sim.world_count(), {});
        BodySpawn body;
        body.pos = glm::vec3(0.0f, 100.0f, 0.0f);
        body.vel = glm::vec3(3.0f, 0.0f, -2.0f);
        body.omega_body = glm::vec3(0.3f, -0.1f, 0.2f);
        body.mass = 0.9f;
        body.inv_inertia_diag = glm::vec3(120.0f, 90.0f, 110.0f);
        const spade::Result<BodyRef> ref = sim.spawn(0, body);
        if (!ref) return std::unexpected(ref.error());
        (*refs)[0].push_back(*ref);

        DragElementSpawn drag;
        drag.mode = spade::physics::drag_mode::quadratic;
        drag.area = 0.05f;
        drag.coeffs = glm::vec3(1.1f, 0.0f, 0.0f);
        drag.local_pos = glm::vec3(0.0f, 0.02f, 0.0f);
        const spade::Result<spade::DragElementRef> elem = sim.add_drag_element(*ref, drag);
        if (!elem) return std::unexpected(elem.error());
        return {};
    };

    s.input = [refs](Simulation& sim, Tick tick) -> spade::Result<void> {
        for (uint32_t w = 0; w < sim.world_count(); ++w) {
            for (uint32_t i = 0; i < (*refs)[w].size(); ++i) {
                if (spade::Result<void> r =
                        sim.apply_wrench((*refs)[w][i], scripted_force(tick.value, i), glm::vec3(0.0f));
                    !r) {
                    return r;
                }
            }
        }
        return {};
    };

    return s;
}

// The body every bounce world drops. Spelled once so the two-forms tripwire
// below reproduces bounce's setup EXACTLY rather than by transcription -- a
// transcription would drift the day someone retunes the drop, and the tripwire
// would then be comparing two different experiments and passing anyway.
[[nodiscard]] BodySpawn bounce_drop_body() {
    BodySpawn body;
    body.pos = glm::vec3(0.0f, 2.0f, 0.0f);
    body.vel = glm::vec3(0.5f, 0.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(100.0f);
    return body;
}

// ---------------------------------------------------------------------------
// Scenario 2 -- bounce. A restitution ladder: four worlds, identical geometry
// and identical drops, restitution 0.0 / 0.3 / 0.6 / 0.9.
//
// It is the HETEROGENEOUS-MATERIAL scenario on purpose: because the four worlds
// disagree about ContactParams, WorldSetLayout::uniform_dynamic_params is false
// and the CollisionDynamic pass takes its per-world form. Every other scenario
// takes the batched form, so between them the corpus pins both.
// ---------------------------------------------------------------------------
[[nodiscard]] Scenario bounce_scenario() {
    Scenario s;
    s.name = "bounce";
    s.dt_ns = 1'000'000;  // 1 ms step
    s.substeps = 1;
    s.steps = 900;

    s.build = []() -> spade::Result<WorldSetDesc> {
        // Solid half-space y <= 0: the ground.
        const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                          .name("ground")
                                                          .environment(default_environment())
                                                          .capacities(capacities(4, 2))
                                                          .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                          .build();
        if (!world) return std::unexpected(world.error());

        WorldSetDesc set;
        const float ladder[4] = {0.0f, 0.3f, 0.6f, 0.9f};
        for (uint32_t i = 0; i < 4; ++i) {
            WorldInstanceDesc instance;
            instance.world = *world;
            instance.seed = 0x5EEDU + i;
            instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
            instance.contacts = contacts(ladder[i], 0.4f, 0.1f);
            instance.grid = grid(0.2f);
            set.worlds.push_back(instance);
        }
        return set;
    };

    s.setup = [](Simulation& sim) -> spade::Result<void> {
        for (uint32_t w = 0; w < sim.world_count(); ++w) {
            const spade::Result<BodyRef> ref = sim.spawn(w, bounce_drop_body());
            if (!ref) return std::unexpected(ref.error());
        }
        return {};
    };

    return s;
}

// ---------------------------------------------------------------------------
// Scenario 3 -- shower. 100 spheres dropped into an SDF bowl (a spherical shell
// with its cavity offset upward), so that CollisionStatic and CollisionDynamic
// both run hot and a pile forms. This is the scenario that would expose an
// ordering dependence in the sorted-grid sweep: a hundred bodies in mutual
// contact resolve Gauss-Seidel, so any instability in the sort order or the
// neighbour walk changes the pile.
// ---------------------------------------------------------------------------
[[nodiscard]] Scenario shower_scenario() {
    Scenario s;
    s.name = "shower";
    s.dt_ns = 2'000'000;  // 2 ms step
    s.substeps = 2;       // 1 ms substep
    s.steps = 400;

    s.build = []() -> spade::Result<WorldSetDesc> {
        // subtract() removes the operand added SECOND: a solid sphere of radius
        // 4 minus a sphere of radius 3.8 centred 0.4 m higher. The remaining
        // shell is thick at the bottom and open-ish at the top -- a bowl.
        spade::SdfPose cavity;
        cavity.position = glm::vec3(0.0f, 0.4f, 0.0f);
        const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                          .name("bowl")
                                                          .environment(default_environment())
                                                          .capacities(capacities(128, 4))
                                                          .sphere(4.0f)
                                                          .sphere(3.8f, cavity)
                                                          .subtract()
                                                          .build();
        if (!world) return std::unexpected(world.error());

        WorldInstanceDesc instance;
        instance.world = *world;
        instance.seed = 0xA11CE0F5E6DULL;
        instance.turbulence = spade::dryden_params(TurbulenceLevel::light);
        instance.contacts = contacts(0.2f, 0.35f, 0.15f);
        instance.grid = grid(0.3f);
        return WorldSetDesc{{instance}};
    };

    s.setup = [](Simulation& sim) -> spade::Result<void> {
        for (uint32_t i = 0; i < 100; ++i) {
            BodySpawn body;
            body.pos = glm::vec3(-1.2f + 0.3f * static_cast<float>(i % 5u),
                                 0.35f * static_cast<float>(i / 25u),
                                 -1.2f + 0.3f * static_cast<float>((i / 5u) % 5u));
            body.mass = 0.2f;
            body.inv_inertia_diag = glm::vec3(500.0f);
            const spade::Result<BodyRef> ref = sim.spawn(0, body);
            if (!ref) return std::unexpected(ref.error());
        }
        return {};
    };

    return s;
}

// ---------------------------------------------------------------------------
// Scenario 4 -- two-world isolation. Two worlds at OVERLAPPING COORDINATES:
// world 0 holds a dense eight-body cluster, world 1 holds two bodies at
// coordinates inside that cluster. If the broad phase leaked across worlds --
// a hash collision, a range check off by one -- world 1's pair would be shoved
// by neighbours it cannot see. The uniform-material path, so the sweep really
// is one batched call over both worlds.
// ---------------------------------------------------------------------------
[[nodiscard]] Scenario two_world_isolation_scenario() {
    Scenario s;
    s.name = "two_world_isolation";
    s.dt_ns = 2'000'000;
    s.substeps = 2;
    s.steps = 300;

    s.build = []() -> spade::Result<WorldSetDesc> {
        const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                          .name("ground")
                                                          .environment(default_environment())
                                                          .capacities(capacities(16, 4))
                                                          .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                          .build();
        if (!world) return std::unexpected(world.error());

        WorldSetDesc set;
        for (uint32_t i = 0; i < 2; ++i) {
            WorldInstanceDesc instance;
            instance.world = *world;
            instance.seed = 0x1501A7E0ULL + i;
            instance.turbulence = spade::dryden_params(TurbulenceLevel::moderate);
            instance.contacts = contacts(0.4f, 0.2f, 0.12f);
            instance.grid = grid(0.24f);
            set.worlds.push_back(instance);
        }
        return set;
    };

    s.setup = [](Simulation& sim) -> spade::Result<void> {
        // World 0: a tight 2x2x2 cluster around (0, 1, 0).
        for (uint32_t i = 0; i < 8; ++i) {
            BodySpawn body;
            body.pos = glm::vec3(0.2f * static_cast<float>(i % 2u), 1.0f + 0.2f * static_cast<float>(i / 4u),
                                 0.2f * static_cast<float>((i / 2u) % 2u));
            body.mass = 0.5f;
            body.inv_inertia_diag = glm::vec3(200.0f);
            const spade::Result<BodyRef> ref = sim.spawn(0, body);
            if (!ref) return std::unexpected(ref.error());
        }
        // World 1: two bodies AT THE SAME COORDINATES as two of the cluster.
        for (uint32_t i = 0; i < 2; ++i) {
            BodySpawn body;
            body.pos = glm::vec3(0.2f * static_cast<float>(i), 1.0f, 0.0f);
            body.mass = 0.5f;
            body.inv_inertia_diag = glm::vec3(200.0f);
            const spade::Result<BodyRef> ref = sim.spawn(1, body);
            if (!ref) return std::unexpected(ref.error());
        }
        return {};
    };

    return s;
}

// ---------------------------------------------------------------------------
// Scenario 5 -- structural churn. Not part of the committed corpus (its job is
// the queue, not a golden), but a full scenario so it can be replayed from a
// resume point.
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

[[nodiscard]] std::vector<Scenario> corpus() {
    return {ballistic_scenario(), bounce_scenario(), shower_scenario(),
            two_world_isolation_scenario()};
}

// ---------------------------------------------------------------------------
// Golden files: tests/golden/<name>.digest. Located by SPADE_GOLDEN_DIR, an
// absolute path baked in at configure time, so nothing here depends on the
// working directory.
//
// FORMAT: '#' comment lines carrying provenance, then one line holding the
// 64-bit digest in hex. Deliberately text: a digest that has to be readable in
// a code review, a bisect and a failure message is worth four bytes of parsing.
// ---------------------------------------------------------------------------
[[nodiscard]] std::string golden_path(const std::string& name) {
    return std::string(SPADE_GOLDEN_DIR) + "/" + name + ".digest";
}

struct GoldenFile {
    bool found = false;
    uint64_t digest = 0;
};

[[nodiscard]] GoldenFile read_golden(const std::string& name) {
    std::ifstream in(golden_path(name));
    if (!in) return {};
    std::string line;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::istringstream parse(line);
        uint64_t value = 0;
        parse >> std::hex >> value;
        if (parse.fail()) return {};
        return GoldenFile{true, value};
    }
    return {};
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

TEST(Schedule, IsExactlySpecSectionThreeInOrder) {
    const std::span<const spade::physics::Pass> schedule = spade::physics::substep_schedule();
    ASSERT_EQ(schedule.size(), spade::physics::kSubstepPassCount);

    // The eight names, in §3's order, transcribed independently of
    // schedule.cpp. If a pass moves, is dropped, or is inserted, this fails --
    // which is the entire reason the schedule is data.
    const char* expected[] = {"MediumUpdate",   "ForceElements",    "Gravity",         "CollisionStatic",
                              "CollisionDynamic", "Integrate",      "SensorSynthesis", "Publish"};
    for (std::size_t i = 0; i < schedule.size(); ++i) {
        EXPECT_EQ(schedule[i].name, expected[i]) << "pass " << i;
        EXPECT_NE(schedule[i].run, nullptr) << "pass " << i;
    }
}

// The Gravity pass is REPRESENTED BUT INERT: integrate_bodies() applies gravity
// itself, so a Gravity pass that also accumulated m*g would double it. A body
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
    // The failure this test exists for: a Gravity pass that also accumulated
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

TEST(Determinism, SameScenarioTwiceProducesTheSameDigest) {
    for (const Scenario& scenario : corpus()) {
        const spade::Result<uint64_t> first = spade::testing::run_scenario(scenario);
        ASSERT_OK(first) << scenario.name;
        const spade::Result<uint64_t> second = spade::testing::run_scenario(scenario);
        ASSERT_OK(second) << scenario.name;
        EXPECT_EQ(*first, *second) << scenario.name;
    }
}

TEST(Determinism, SameScenarioTwiceProducesByteIdenticalSnapshots) {
    for (const Scenario& scenario : corpus()) {
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
        EXPECT_EQ(std::memcmp(bytes[0].data(), bytes[1].data(), bytes[0].size()), 0) << scenario.name;
    }
}

// ===========================================================================
// 4. THE REPLAY GUARANTEE (charter P2/P4)
// ===========================================================================

TEST(Determinism, SnapshotRestoreIntoAFreshSimulationResumesIdentically) {
    std::vector<Scenario> scenarios = corpus();
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
    const Scenario scenario = two_world_isolation_scenario();
    spade::Result<Simulation> both = spade::testing::start_scenario(scenario);
    ASSERT_OK(both);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *both, scenario.steps));

    const spade::Result<WorldSetDesc> full = scenario.build();
    ASSERT_OK(full);
    WorldSetDesc solo;
    solo.worlds.push_back(full->worlds[1]);
    spade::Result<Simulation> alone = Simulation::create(solo, scenario.dt_ns, scenario.substeps);
    ASSERT_OK(alone);
    for (uint32_t i = 0; i < 2; ++i) {
        BodySpawn body;
        body.pos = glm::vec3(0.2f * static_cast<float>(i), 1.0f, 0.0f);
        body.mass = 0.5f;
        body.inv_inertia_diag = glm::vec3(200.0f);
        ASSERT_OK(alone->spawn(0, body));
    }
    ASSERT_OK(alone->step(scenario.steps));

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
    const Scenario scenario = bounce_scenario();

    spade::Result<Simulation> four = spade::testing::start_scenario(scenario);
    ASSERT_OK(four);
    // bounce's restitution ladder is what makes the set heterogeneous.
    ASSERT_FALSE(four->layout().uniform_dynamic_params);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *four, scenario.steps));

    const spade::Result<WorldSetDesc> full = scenario.build();
    ASSERT_OK(full);
    WorldSetDesc solo;
    solo.worlds.push_back(full->worlds[1]);  // same instance, same capacities

    spade::Result<Simulation> alone = Simulation::create(solo, scenario.dt_ns, scenario.substeps);
    ASSERT_OK(alone);
    ASSERT_TRUE(alone->layout().uniform_dynamic_params);
    // bounce's own setup body, not a transcription of it.
    ASSERT_OK(alone->spawn(0, bounce_drop_body()));
    ASSERT_OK(alone->step(scenario.steps));

    EXPECT_EQ(spade::testing::world_digest(*alone, 0), spade::testing::world_digest(*four, 1));
}

// ===========================================================================
// 6. The committed golden corpus
// ===========================================================================

TEST(Determinism, DigestsMatchTheCommittedGoldenCorpus) {
    for (const Scenario& scenario : corpus()) {
        const spade::Result<uint64_t> digest = spade::testing::run_scenario(scenario);
        ASSERT_OK(digest) << scenario.name;

        const GoldenFile golden = read_golden(scenario.name);
        if (!golden.found) {
            ADD_FAILURE() << "missing or unreadable golden " << golden_path(scenario.name)
                          << "\nTo (re)generate it, write exactly:\n"
                          << "# spade determinism corpus v0 -- scenario: " << scenario.name << "\n"
                          << hex64(*digest) << "\n";
            continue;
        }
        EXPECT_EQ(*digest, golden.digest)
            << "scenario '" << scenario.name << "' digest changed: got " << hex64(*digest)
            << ", golden " << hex64(golden.digest)
            << "\nThis is a PINNED op order / layout / rng / schedule change. If it is intended, "
               "the golden is updated deliberately, with the reason recorded in the file's "
               "provenance header -- never as a drive-by.";
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
    const auto every_live_body_is_sane = [](Simulation& sim, const char* what) {
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
        const Scenario s = ballistic_scenario();
        spade::Result<Simulation> sim = spade::testing::start_scenario(s);
        ASSERT_OK(sim);
        ASSERT_OK(spade::testing::advance_scenario(s, *sim, s.steps));
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
        const Scenario s = bounce_scenario();
        spade::Result<Simulation> sim = spade::testing::start_scenario(s);
        ASSERT_OK(sim);
        ASSERT_OK(spade::testing::advance_scenario(s, *sim, s.steps));
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
        const Scenario s = shower_scenario();
        spade::Result<Simulation> sim = spade::testing::start_scenario(s);
        ASSERT_OK(sim);
        ASSERT_OK(spade::testing::advance_scenario(s, *sim, s.steps));
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
        const Scenario s = two_world_isolation_scenario();
        spade::Result<Simulation> sim = spade::testing::start_scenario(s);
        ASSERT_OK(sim);
        ASSERT_OK(spade::testing::advance_scenario(s, *sim, s.steps));
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
// the innocent row, after which the queued init_drag would write a LIVE row
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
    const Scenario scenario = bounce_scenario();
    spade::Result<Simulation> sim = spade::testing::start_scenario(scenario);
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
