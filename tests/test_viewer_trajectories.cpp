// ---------------------------------------------------------------------------
// test_viewer_trajectories.cpp -- the guard over tests/golden/viewer/ (INT-4).
//
// SL14b: each viewer scene's successor must reproduce its trajectory bit for
// bit. These goldens were captured from spade_viewer's own setup, so this
// replays every scene from spade_viewer_scenes (v1-free, built in every
// configuration) and asserts the committed checkpoints. An engine change that
// moves a viewer scene therefore fails here, the way a moved golden does
// (TD-1). Regenerate only as tests/golden/viewer/README.md says.
//
// One case per scene, so a failure names its scene. Each case checks every
// checkpoint up to its own N (the running chain makes any prefix checkable),
// chosen so the case stays under 5 s on the debug preset. With
// SPADE_FULL_VIEWER_TRAJECTORIES=1 every case runs to the golden's last tick;
// the Docker leg sets it, so all 3000 ticks are checked on gcc too.
// ---------------------------------------------------------------------------
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "bridge.hpp"
#include "setup.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"

namespace {

struct Checkpoint {
    uint64_t chain = 0;
    uint64_t state = 0;
    std::vector<uint64_t> worlds;
};

struct Golden {
    uint64_t ticks = 0;
    uint64_t dt_ns = 0;
    uint32_t substeps = 0;
    std::map<uint64_t, Checkpoint> checkpoints;
};

// getenv is read once per case, before any thread starts, so the C4996
// thread-safety warning does not apply here (as in test_fp32_math.cpp).
[[nodiscard]] bool env_gate_is_set(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name) != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// Lines, not bytes: a Windows checkout has CRLF, and the blob has LF.
[[nodiscard]] Golden load_golden(const std::string& scene) {
    Golden g;
    std::ifstream in(std::string(SPADE_GOLDEN_DIR) + "/viewer/" + scene + ".trajectory.txt", std::ios::binary);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        std::istringstream fields(line);
        std::string key;
        fields >> key;
        if (key == "ticks") {
            fields >> g.ticks;
        } else if (key == "dt_ns") {
            fields >> g.dt_ns;
        } else if (key == "substeps") {
            fields >> g.substeps;
        } else if (key == "checkpoint") {
            uint64_t tick = 0;
            std::string chain;
            std::string state;
            fields >> tick >> chain >> state;
            Checkpoint c{std::stoull(chain, nullptr, 16), std::stoull(state, nullptr, 16), {}};
            std::string world;
            while (fields >> world) {
                c.worlds.push_back(std::stoull(world, nullptr, 16));
            }
            g.checkpoints[tick] = c;
        }
    }
    return g;
}

struct Case {
    const char* scene;
    uint64_t ticks;  // checked by default; a multiple of 25 so it ends on a checkpoint
};

// A failure names the scene and its N, not the struct's bytes.
void PrintTo(const Case& c, std::ostream* os) {
    *os << c.scene << " to tick " << c.ticks;
}

class ViewerTrajectory : public ::testing::TestWithParam<Case> {};

TEST_P(ViewerTrajectory, MatchesItsGolden) {
    const Case c = GetParam();
    const Golden golden = load_golden(c.scene);
    ASSERT_FALSE(golden.checkpoints.empty())
        << "no golden for '" << c.scene << "' in tests/golden/viewer/. Capture it with "
        << "engine/tools/viewer/capture-v1-baselines.ps1 -Part trajectories.";
    ASSERT_EQ(golden.dt_ns, spade::viewer::kStepDtNs) << "the golden was captured at another step";
    ASSERT_EQ(golden.substeps, spade::viewer::kSubsteps) << "the golden was captured at another substep count";

    const uint64_t n = env_gate_is_set("SPADE_FULL_VIEWER_TRAJECTORIES") ? golden.ticks : c.ticks;
    ASSERT_LE(n, golden.ticks) << "the case checks past the golden's last tick";
    ASSERT_TRUE(golden.checkpoints.contains(n)) << "the case's N is not a checkpoint tick";

    std::optional<spade::viewer::Scene> scene = spade::viewer::make_scene(c.scene);
    ASSERT_TRUE(scene.has_value()) << "spade_viewer_scenes has no scene '" << c.scene << "'";
    spade::viewer::SceneRun run = spade::viewer::make_simulation(*scene, spade::compute::BackendDesc{});
    const uint32_t world_count = run.sim.layout().world_count;

    // The same per-tick order the golden was written with (trajectory.cpp):
    // fold this tick's digest, check, then the hook, then exactly one step.
    uint64_t chain = spade::kFnv1a64Offset;
    for (uint64_t t = 0;; ++t) {
        const uint64_t state = spade::testing::state_digest(run.sim);
        chain = spade::fnv1a64(std::as_bytes(std::span<const uint64_t, 1>(&state, 1)), chain);
        if (const auto it = golden.checkpoints.find(t); it != golden.checkpoints.end()) {
            const Checkpoint& want = it->second;
            ASSERT_EQ(chain, want.chain) << c.scene << ": the trajectory diverges at or before tick " << t;
            ASSERT_EQ(state, want.state) << c.scene << ": the state differs at tick " << t;
            ASSERT_EQ(want.worlds.size(), world_count) << c.scene << ": the golden has another world count";
            for (uint32_t w = 0; w < world_count; ++w) {
                ASSERT_EQ(spade::testing::world_digest(run.sim, w), want.worlds[w])
                    << c.scene << ": world " << w << " differs at tick " << t;
            }
        }
        if (t == n) {
            break;
        }
        if (scene->command_hook != nullptr) {
            scene->command_hook(run.sim, run.sim.tick().value, run.vehicle_refs);
        }
        const spade::Result<void> stepped = run.sim.step(1);
        ASSERT_TRUE(stepped.has_value()) << c.scene << ": step failed at tick " << t << ": "
                                         << stepped.error().context;
    }
}

// N per scene, from a debug measurement on 2026-10-03, under the lead's cap of
// 5 s per case and 40 s in all. Debug step cost is about 0.3-0.9 ms per tick,
// so each case takes 1.7 s or less, and all eight about 9 s; runs varied by up
// to 2x, which the margin covers. shower steps 1000 bodies at about 280 ms per
// debug tick, so even its first checkpoint (25 ticks, 7 s) is over the cap. By
// default it checks only tick 0 -- the scene, the spawns and the setup -- and
// its stepping is checked in full by SPADE_FULL_VIEWER_TRAJECTORIES=1.
INSTANTIATE_TEST_SUITE_P(Viewer, ViewerTrajectory,
                         ::testing::Values(Case{"drop", 2000}, Case{"bounce", 2500}, Case{"shower", 0},
                                           Case{"gate", 2500}, Case{"hover", 3000}, Case{"wind", 3000},
                                           Case{"flight", 3000}, Case{"swarm", 1500}),
                         [](const ::testing::TestParamInfo<Case>& info) { return std::string(info.param.scene); });

}  // namespace
