// test_objects_determinism.cpp -- Plan A Task 5 (24th spec SL3/SL18).
//
// THIS IS THE TEST THE WHOLE PLAN RESTS ON. SL3's claim is that the object
// graph is composition and identity, never registered state, and therefore
// costs the determinism estate nothing: same digest, byte-identical snapshot,
// unchanged `kSnapshotVersion`. If that is false, the object model is not free
// and the architecture is wrong -- so this file must be able to DETECT a
// perturbation, not merely fail to observe one.
//
// TWO CHOICES HERE THAT THE PLAN DID NOT SPECIFY, BOTH TO MAKE IT FALSIFIABLE
// ---------------------------------------------------------------------------
// 1. THE GRAPH IS CHURNED *BETWEEN* STEPS, not built once before the run. A
//    graph constructed before stepping and left alone proves almost nothing:
//    ObjectGraph and Simulation share no member, so of course a std::vector
//    filled beforehand does not change a float computed afterwards. The
//    failure worth excluding is a COUPLING THROUGH SOMETHING GLOBAL -- an
//    allocator sequence, a shared RNG, a static cache -- and that only shows
//    up if graph mutation is interleaved with the ticks whose results are
//    being compared. Interleaving costs nothing and is the version of this
//    test that could actually go red.
//
// 2. IT RUNS THE COMMITTED SCENARIO CORPUS, not a two-body scenario written
//    here. The plan said to lift test_determinism.cpp's construction verbatim
//    if no reusable helper existed. One does: scenario_from_yaml() +
//    start_scenario()/advance_scenario() are the same entry points the golden
//    digests are produced through, so this test compares runs of the SAME
//    experiments the corpus already pins, and cannot drift from them the way
//    a transcribed scenario would.
//
// Both comparisons are made: the 64-bit whole-set digest (what the goldens
// speak in) AND the full snapshot blob, memcmp'd. The digest is the readable
// signal; the blob is the one that cannot hide a difference in a byte the
// digest happens not to reach.
//
// WHAT THIS FILE STRUCTURALLY CANNOT SEE, stated because a green result here
// should not be read as more than it is. Both runs happen IN ONE PROCESS, so
// any byte that is indeterminate rather than wrong -- uninitialized padding
// inside a snapshotted struct, say -- holds the SAME garbage in both and
// compares equal. A single-process comparison can prove that adding the object
// graph changed nothing; it can never prove the bytes were defined to begin
// with. (A peer program hit exactly this in 2026-09: a 4-byte padding hole
// memcpy'd into a snapshot gave six processes six different digests while
// every in-process determinism test in that estate stayed green, and one
// conformance golden was true only because of the uninitialized bytes.)
//
// WHAT COVERS IT HERE, so the gap is closed rather than merely noted:
//   * state/layout.hpp static_asserts that BodyState's NAMED fields sum to
//     sizeof(BodyState) -- there is no implicit padding to be indeterminate.
//   * WorldParams is the one struct in that header with tail padding, it is
//     documented as such, ArenaSet zero-fills it on construction and on free,
//     and StateLayout.WorldParamsTailPaddingIsZeroInAnArena pins it.
//   * The corpus's `expected_digest` values are COMMITTED CONSTANTS, produced
//     by a different process on a different machine, and .github/workflows/
//     spade.yml re-runs `ctest -L spade` on another platform. That comparison
//     is cross-process by construction, which is the property this file's own
//     two runs lack.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "objects/graph.hpp"
#include "sim/simulation.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"
#include "testing/scenario_file.hpp"

namespace {

using spade::Simulation;
using spade::SnapshotBlob;
using spade::objects::BehaviorComponent;
using spade::objects::BodyComponent;
using spade::objects::CameraComponent;
using spade::objects::ColliderComponent;
using spade::objects::FluidComponent;
using spade::objects::ForceElementComponent;
using spade::objects::MaterialComponent;
using spade::objects::MeshComponent;
using spade::objects::ObjectGraph;
using spade::objects::ObjectId;
using spade::objects::SensorComponent;
using spade::objects::TransformComponent;
using spade::testing::LoadedScenario;
using spade::testing::Scenario;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code)
                                       << "] " << r.error().context;
}

#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)

[[nodiscard]] std::filesystem::path scenario_dir() {
    return std::filesystem::path(SPADE_GOLDEN_DIR) / "scenarios";
}

// Loaded in filename order, because directory_iterator's order is unspecified
// and a failure message should name a stable sequence.
//
// The empty-corpus failure below is not defensive noise: without it, a broken
// SPADE_GOLDEN_DIR would make every `for (scenario : corpus())` case in this
// file pass over an empty range -- asserting nothing, cheerfully, while
// reporting that SL3 holds. That is the exact vacuity this program keeps
// finding, and it is worse here than anywhere else in the tree, because this
// file's green is what the plan cites to justify the whole object model.
[[nodiscard]] std::vector<LoadedScenario> corpus() {
    std::vector<std::filesystem::path> paths;
    std::error_code ec;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(scenario_dir(), ec)) {
        if (entry.is_regular_file() &&
            entry.path().filename().string().ends_with(".scenario.yaml")) {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());

    std::vector<LoadedScenario> loaded;
    loaded.reserve(paths.size());
    for (const std::filesystem::path& path : paths) {
        spade::Result<LoadedScenario> scenario = spade::testing::scenario_from_yaml(path);
        if (!scenario) {
            ADD_FAILURE() << "failed to load " << path.string() << ": ["
                          << static_cast<int>(scenario.error().code) << "] "
                          << scenario.error().context;
            continue;
        }
        loaded.push_back(std::move(*scenario));
    }
    if (loaded.empty()) {
        ADD_FAILURE() << "the scenario corpus is empty: " << scenario_dir().string();
    }
    return loaded;
}

// Every component type, so that if ANY of them perturbs anything the
// comparison catches it -- including the two whose payloads Plan B and Task 8
// have yet to fill in.
void attach_a_full_complement(ObjectGraph& g, ObjectId id, uint32_t i) {
    g.attach<TransformComponent>(id, TransformComponent{});
    g.attach<BodyComponent>(id, BodyComponent{.world_index = 0, .body_slot = i});
    g.attach<MeshComponent>(id, MeshComponent{.draw_item = i});
    g.attach<MaterialComponent>(id, MaterialComponent{.material_index = i});
    g.attach<ColliderComponent>(id, ColliderComponent{.sphere_radius = 0.25f});
    g.attach<SensorComponent>(id, SensorComponent{.sensor_slot = i});
    g.attach<ForceElementComponent>(id, ForceElementComponent{.element_slot = i});
    g.attach<CameraComponent>(id, CameraComponent{});
    g.attach<BehaviorComponent>(id, BehaviorComponent{.behavior_index = i});
    g.attach<FluidComponent>(id, FluidComponent{});
}

// Allocation churn with recycling, so the free list and slot reuse are
// exercised between ticks rather than only at the start.
void churn(ObjectGraph& g, uint64_t round) {
    std::vector<ObjectId> made;
    for (uint32_t i = 0; i < 8; ++i) {
        const ObjectId id = g.create("churn-" + std::to_string(round) + "-" + std::to_string(i));
        attach_a_full_complement(g, id, i);
        made.push_back(id);
    }
    // Free every other one, so the next round allocates from a non-empty free
    // list in LIFO order rather than always appending.
    for (std::size_t i = 0; i < made.size(); i += 2) {
        g.destroy(made[i]);
    }
}

struct RunResult {
    uint64_t digest = 0;
    std::vector<std::byte> blob;
};

// Runs one corpus scenario to completion. With `with_graph`, an ObjectGraph is
// built and churned between step batches; without it, nothing else differs.
[[nodiscard]] spade::Result<RunResult> run(const Scenario& scenario, bool with_graph) {
    spade::Result<Simulation> sim = spade::testing::start_scenario(scenario);
    if (!sim) return std::unexpected(sim.error());

    ObjectGraph graph;

    // Four segments, so graph mutation lands between ticks rather than only
    // before the first one. Integer division can leave a remainder; the final
    // advance to scenario.steps collects it, so both runs always reach the
    // same tick.
    constexpr uint64_t kSegments = 4;
    for (uint64_t seg = 1; seg <= kSegments; ++seg) {
        const uint64_t until = (seg == kSegments) ? scenario.steps
                                                  : (scenario.steps * seg) / kSegments;
        if (with_graph) churn(graph, seg);
        if (spade::Result<void> r = spade::testing::advance_scenario(scenario, *sim, until); !r) {
            return std::unexpected(r.error());
        }
    }

    spade::Result<SnapshotBlob> blob = sim->snapshot();
    if (!blob) return std::unexpected(blob.error());

    RunResult out;
    out.digest = spade::testing::state_digest(*sim);
    out.blob.assign(blob->bytes().begin(), blob->bytes().end());
    return out;
}

}  // namespace

// SL3's headline claim, stated as an experiment: the same scenario, run with
// and without a live object graph being built and churned alongside it, must
// produce the same digest and the same snapshot bytes.
TEST(ObjectGraphDeterminism, BuildingAGraphDoesNotPerturbTheSimulation) {
    for (const LoadedScenario& loaded : corpus()) {
        const Scenario& scenario = loaded.scenario;
        const spade::Result<RunResult> without = run(scenario, false);
        ASSERT_OK(without) << scenario.name;
        const spade::Result<RunResult> with = run(scenario, true);
        ASSERT_OK(with) << scenario.name;

        EXPECT_EQ(without->digest, with->digest)
            << "SL3 violated: the object graph changed " << scenario.name << "'s result";
        ASSERT_EQ(without->blob.size(), with->blob.size()) << scenario.name;
        EXPECT_EQ(std::memcmp(without->blob.data(), with->blob.data(), without->blob.size()), 0)
            << "SL3 violated: the object graph changed " << scenario.name << "'s snapshot bytes";
    }
}

// The other half of "not registered state": a blob written by a Simulation
// that never heard of this module restores into one sitting beside a populated
// graph, because `kSnapshotVersion` and the registry walk do not know the
// object graph exists. If this ever fails, something put the graph inside the
// snapshot.
TEST(ObjectGraphDeterminism, SnapshotTakenWithoutAGraphRestoresWithOnePresent) {
    for (const LoadedScenario& loaded : corpus()) {
        const Scenario& scenario = loaded.scenario;
        const uint64_t half = scenario.steps / 2;

        spade::Result<Simulation> a = spade::testing::start_scenario(scenario);
        ASSERT_OK(a) << scenario.name;
        ASSERT_OK(spade::testing::advance_scenario(scenario, *a, half)) << scenario.name;
        const spade::Result<SnapshotBlob> blob = a->snapshot();
        ASSERT_OK(blob) << scenario.name;

        spade::Result<Simulation> b = spade::testing::start_scenario(scenario);
        ASSERT_OK(b) << scenario.name;
        ObjectGraph graph;
        churn(graph, 1);

        // Before the restore the two differ -- which is what makes the
        // equality below evidence that the restore DID something, rather than
        // evidence that two freshly-started simulations agree. Without this
        // line the test would still pass if restore() became a no-op.
        ASSERT_NE(spade::testing::state_digest(*a), spade::testing::state_digest(*b))
            << scenario.name << ": half its steps left the state unchanged, so the "
                                "restore below would prove nothing";

        ASSERT_OK(b->restore(*blob)) << scenario.name;
        EXPECT_EQ(spade::testing::state_digest(*a), spade::testing::state_digest(*b))
            << scenario.name;

        // And it keeps matching once both resume, which is the property a
        // version bump would break rather than the restore call itself.
        ASSERT_OK(spade::testing::advance_scenario(scenario, *a, scenario.steps)) << scenario.name;
        churn(graph, 2);
        ASSERT_OK(spade::testing::advance_scenario(scenario, *b, scenario.steps)) << scenario.name;
        EXPECT_EQ(spade::testing::state_digest(*a), spade::testing::state_digest(*b))
            << scenario.name;
    }
}
