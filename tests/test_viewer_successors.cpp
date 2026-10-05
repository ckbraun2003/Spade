// ---------------------------------------------------------------------------
// test_viewer_successors.cpp -- the viewer scenes as world and scene files
// (the scene composer's task 5; the lead's ruling, 2026-10-05).
//
// assets/worlds/ and assets/scenes/ hold each spade_viewer scene as content: a
// world file and a scene file that spade::scene::compose_file() and
// instantiate() turn into a run. This test ties each one to the viewer: it
// runs the viewer's own setup (make_scene + make_simulation, which the
// trajectory goldens in golden/viewer/ pin) beside the successor, and asserts
// before every step that every row the viewer holds is the successor's row,
// byte for byte.
//
// WHY ROWS AND NOT THE GOLDEN DIGESTS. The digests cannot match by
// construction, and no golden moves:
//   - compose() sizes each capacity as the world's own count plus what the
//     scene spawns plus spare (SCN-007), and a world file's counts must be
//     > 0, so a successor always has at least one more slot than the viewer,
//     which declares exactly what it uses; the digests fold the capacities.
//   - drop, bounce, shower and gate spawn bare bodies, which a scene file has
//     no kind for. Their successors spawn body-only models (a body and nothing
//     else), which write the same body rows but add a model to the registry.
//   - bounce and swarm are four-world runs; a scene names one world. Each lane
//     is its own successor, compared with the viewer's world `lane`.
// So the comparison is over the viewer's rows: every row of every array the
// viewer registers, in world `lane`, against the same row of the successor's
// world 0. The viewer's capacities are exactly what it uses, so every
// compared row is a live one, and the successor's extra rows are the padding
// SCN-007 adds. Excluded, each with its reason, in kExcluded below.
//
// What the files cannot hold stays in code, here: the per-world physics
// records (turbulence, contacts, grid), which world file v2 does not carry
// and instantiate() takes as its WorldInstanceDesc; and flight's command hook.
//
// A MISSING FILE is written from the viewer scene into the build tree (never
// the source tree), and the case fails naming it: review it, then copy it
// into assets/. assets/scenes/README.md says the same.
// ---------------------------------------------------------------------------
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "bridge.hpp"
#include "scene/compose.hpp"
#include "scene/scene_file.hpp"
#include "setup.hpp"
#include "sim/simulation.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"
#include "world/world_file.hpp"

namespace {

namespace fs = std::filesystem;
using spade::RegisteredArray;

// What is not a world's state, and so is not compared. Each is named with why.
//   replay_config: the whole set's identity (world count, capacities, world
//   names, the model registry). It differs by construction (above), and
//   world_digest() leaves it out for the same reason.
const std::vector<std::string_view> kExcluded = {spade::kReplayConfigArray};

// Fields within a compared row that are not a world's state, each with why.
struct ExcludedField {
    std::string_view array;
    std::size_t offset;
    std::size_t size;
};
//   world_params.body_capacity: the body partition's size, which SCN-007 pads.
//   The row's body_count, its live bodies, is compared.
const std::vector<ExcludedField> kExcludedFields = {
    {"world_params", offsetof(spade::WorldParams, body_capacity), sizeof(spade::WorldParams::body_capacity)},
};

// Arrays that hold a world's id: every "<partition>.slot_to_world", one
// uint32_t per slot naming the world the slot belongs to. A lane's successor
// is world 0 of its own run, so each row must read `lane` in the viewer and 0
// in the successor: the same world, numbered by its run. Not skipped,
// translated.
constexpr std::string_view kSlotToWorld = ".slot_to_world";

[[nodiscard]] bool holds_world_ids(const RegisteredArray& a) {
    return a.name.size() > kSlotToWorld.size() &&
           std::string_view(a.name).substr(a.name.size() - kSlotToWorld.size()) == kSlotToWorld;
}

// Set means present, as in test_viewer_trajectories.cpp, so one variable gates
// both tests the same way.
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

// The scene's file stem: the viewer's name, or "<name>_lane_<i>" for a lane of
// a many-world scene.
[[nodiscard]] std::string stem_of(std::string_view scene, uint32_t lane, uint32_t lanes) {
    return lanes == 1u ? std::string(scene) : std::string(scene) + "_lane_" + std::to_string(lane);
}

struct Successor {
    spade::WorldDesc world;
    spade::scene::SceneDesc scene;
};

// The viewer scene's lane `lane`, as a world and a scene. Used only to write a
// missing file; the comparison always runs the files on disk.
[[nodiscard]] Successor successor_of(const spade::viewer::Scene& v, uint32_t lane, const std::string& stem) {
    Successor s;
    s.world = v.worlds.worlds.at(lane).world;
    // compose() adds what the scene spawns to the world's own counts (SCN-007),
    // and a world's counts must be > 0: the world keeps one of each for itself.
    // Contacts are not spawned, so the world's count is the run's.
    s.world.capacities.bodies = 1;
    s.world.capacities.force_elements = 1;
    s.world.capacities.sensors = 1;

    s.scene.name = stem;
    s.scene.world.file = "../worlds/" + stem + ".world.yaml";
    s.scene.world.hash = spade::scene::world_hash(s.world).value();

    // Bare bodies become vehicles of a body-only model, one model per distinct
    // body template. The model carries the inertia; the body row carries its
    // inverse, so the reciprocal must survive the round trip exactly.
    for (const spade::viewer::BodyPlacement& b : v.bodies) {
        if (b.world_index != lane) continue;
        const glm::vec3 inertia = 1.0f / b.spawn.inv_inertia_diag;
        EXPECT_EQ(1.0f / inertia, b.spawn.inv_inertia_diag) << stem << ": an inverse inertia does not round-trip";
        std::string model;
        for (const spade::vehicles::ModelType& m : s.scene.models) {
            if (m.body.mass == b.spawn.mass && m.body.inertia_diag == inertia) model = m.name;
        }
        if (model.empty()) {
            spade::vehicles::ModelType m;
            m.name = stem + "_body" + (s.scene.models.empty() ? "" : "_" + std::to_string(s.scene.models.size()));
            m.body.mass = b.spawn.mass;
            m.body.inertia_diag = inertia;
            s.scene.models.push_back(m);
            model = m.name;
        }
        spade::VehicleSpawn start;
        start.pos = b.spawn.pos;
        start.orient = b.spawn.orient;
        start.vel = b.spawn.vel;
        start.omega_body = b.spawn.omega_body;
        s.scene.vehicles.push_back({stem + "_" + std::to_string(s.scene.vehicles.size()), model, start});
    }

    // The viewer's models, all of them and in order (ids follow it), and its
    // vehicles in this lane.
    for (const spade::vehicles::ModelType& m : v.models) {
        s.scene.models.push_back(m);
    }
    for (const spade::viewer::VehiclePlacement& p : v.vehicles) {
        if (p.world_index != lane) continue;
        const std::string& model = v.models.at(p.model_index).name;
        s.scene.vehicles.push_back({model + "_" + std::to_string(s.scene.vehicles.size()), model, p.spawn});
    }
    return s;
}

[[nodiscard]] bool write_text(const fs::path& path, const std::string& text) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    return static_cast<bool>(out);
}

// One array of the viewer's, paired with the successor's array of that name:
// the viewer's world `lane` partition against the successor's world 0.
struct Pair {
    std::string name;
    const std::byte* viewer = nullptr;
    const std::byte* successor = nullptr;
    std::size_t stride = 0;
    uint32_t rows = 0;          // the viewer's capacity: every row it holds
    std::vector<bool> skip;     // per byte of a row; empty when every byte is compared
    bool world_ids = false;     // a *.slot_to_world array
};

// Pairs every compared array once. Capacities are fixed at create() and the
// arenas never move, so the pairs stay valid for the whole run; the test
// checks that again at its last tick. Returns the pairs, or why they cannot
// be made.
[[nodiscard]] spade::Result<std::vector<Pair>> pair_arrays(const spade::Simulation& viewer, uint32_t lane,
                                                           const spade::Simulation& successor) {
    std::map<std::string, RegisteredArray> theirs;
    successor.arenas().registry().for_each_array(
        [&theirs](const RegisteredArray& a) { theirs.emplace(a.name, a); });
    std::vector<Pair> pairs;
    std::string why;
    viewer.arenas().registry().for_each_array([&](const RegisteredArray& mine) {
        if (!why.empty()) return;
        for (const std::string_view x : kExcluded) {
            if (mine.name == x) return;
        }
        // A set-wide array (one partition in a many-world run) is compared on
        // lane 0 only, as world_digest() folds it only for world 0.
        if (lane >= mine.world_count) return;
        const auto it = theirs.find(mine.name);
        if (it == theirs.end()) {
            why = "array '" + mine.name + "' is not in the successor";
            return;
        }
        const RegisteredArray& other = it->second;
        if (other.elem_size != mine.elem_size) {
            why = "array '" + mine.name + "' has element size " + std::to_string(other.elem_size) +
                  ", the viewer's is " + std::to_string(mine.elem_size);
            return;
        }
        if (other.capacity_per_world < mine.capacity_per_world) {
            why = "array '" + mine.name + "' holds " + std::to_string(other.capacity_per_world) +
                  " rows, the viewer uses " + std::to_string(mine.capacity_per_world);
            return;
        }
        Pair p;
        p.name = mine.name;
        p.stride = mine.elem_size;
        p.rows = mine.capacity_per_world;
        p.viewer = mine.data + static_cast<std::size_t>(lane) * mine.capacity_per_world * p.stride;
        p.successor = other.data;
        for (const ExcludedField& f : kExcludedFields) {
            if (f.array != mine.name) continue;
            p.skip.resize(p.stride, false);
            for (std::size_t k = f.offset; k < f.offset + f.size && k < p.stride; ++k) p.skip[k] = true;
        }
        if (holds_world_ids(mine)) {
            if (p.stride != sizeof(uint32_t)) {
                why = "array '" + mine.name + "' holds world ids but its element is " + std::to_string(p.stride) +
                      " bytes, not a uint32_t";
                return;
            }
            p.world_ids = true;
        }
        pairs.push_back(std::move(p));
    });
    if (!why.empty()) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, why});
    }
    return pairs;
}

// True when every pair still points at its run's arrays.
[[nodiscard]] bool pairs_still_valid(const std::vector<Pair>& pairs, const spade::Simulation& viewer, uint32_t lane,
                                     const spade::Simulation& successor) {
    const auto again = pair_arrays(viewer, lane, successor);
    if (!again || again->size() != pairs.size()) return false;
    for (std::size_t i = 0; i < pairs.size(); ++i) {
        if ((*again)[i].viewer != pairs[i].viewer || (*again)[i].successor != pairs[i].successor) return false;
    }
    return true;
}

// The first row where the pairs differ, or "" when every viewer row matches.
// A plain array is one memcmp; the row-by-row walk runs only for the masked
// and translated arrays, or to name the row once a memcmp has failed.
[[nodiscard]] std::string first_difference(const std::vector<Pair>& pairs, uint32_t lane) {
    for (const Pair& p : pairs) {
        if (!p.world_ids && p.skip.empty() &&
            std::memcmp(p.viewer, p.successor, static_cast<std::size_t>(p.rows) * p.stride) == 0) {
            continue;
        }
        for (uint32_t row = 0; row < p.rows; ++row) {
            const std::byte* a = p.viewer + static_cast<std::size_t>(row) * p.stride;
            const std::byte* b = p.successor + static_cast<std::size_t>(row) * p.stride;
            if (p.world_ids) {
                uint32_t in_viewer = 0;
                uint32_t in_successor = 0;
                std::memcpy(&in_viewer, a, sizeof in_viewer);
                std::memcpy(&in_successor, b, sizeof in_successor);
                // UINT32_MAX is a slot no world owns; it must be unowned in both.
                const bool unowned = in_viewer == UINT32_MAX && in_successor == UINT32_MAX;
                if (!unowned && (in_viewer != lane || in_successor != 0u)) {
                    return "array '" + p.name + "' row " + std::to_string(row) + " names world " +
                           std::to_string(in_viewer) + " in the viewer and " + std::to_string(in_successor) +
                           " in the successor; lane " + std::to_string(lane) + " should map to 0";
                }
                continue;
            }
            for (std::size_t byte = 0; byte < p.stride; ++byte) {
                if ((p.skip.empty() || !p.skip[byte]) && a[byte] != b[byte]) {
                    return "array '" + p.name + "' row " + std::to_string(row) + " differs at byte " +
                           std::to_string(byte) + " of " + std::to_string(p.stride);
                }
            }
        }
    }
    return {};
}

struct Case {
    const char* scene;
    uint32_t lanes;
    uint64_t ticks;       // checked by default
    uint64_t full_ticks;  // with SPADE_FULL_VIEWER_TRAJECTORIES=1 (the Docker leg)
};

void PrintTo(const Case& c, std::ostream* os) {
    *os << c.scene << " (" << c.lanes << (c.lanes == 1u ? " lane" : " lanes") << ") to tick " << c.ticks;
}

class ViewerSuccessor : public ::testing::TestWithParam<Case> {};

TEST_P(ViewerSuccessor, EveryViewerRowIsTheSuccessorsEveryTick) {
    const Case c = GetParam();
    std::optional<spade::viewer::Scene> scene = spade::viewer::make_scene(c.scene);
    ASSERT_TRUE(scene.has_value()) << "spade_viewer_scenes has no scene '" << c.scene << "'";
    ASSERT_EQ(scene->worlds.worlds.size(), c.lanes) << c.scene << ": the viewer scene has another world count";
    ASSERT_TRUE(scene->bodies.empty() || scene->vehicles.empty())
        << c.scene << ": the viewer spawns bodies before it registers models, and a scene registers models "
                      "first, so a scene with both would spawn in another order";
    ASSERT_TRUE(scene->command_hook == nullptr || c.lanes == 1u) << c.scene << ": a hook drives all lanes at once";

    spade::viewer::SceneRun viewer = spade::viewer::make_simulation(*scene, spade::compute::BackendDesc{});

    const fs::path assets(SPADE_ASSETS_DIR);
    const fs::path drafts = fs::path(SPADE_TEST_OUTPUT_DIR) / "viewer-successors";
    std::vector<spade::scene::SceneRun> successors;
    std::vector<std::string> missing;
    for (uint32_t lane = 0; lane < c.lanes; ++lane) {
        const std::string stem = stem_of(c.scene, lane, c.lanes);
        const fs::path world_file = assets / "worlds" / (stem + ".world.yaml");
        const fs::path scene_file = assets / "scenes" / (stem + ".scene.yaml");
        if (!fs::exists(world_file) || !fs::exists(scene_file)) {
            const Successor s = successor_of(*scene, lane, stem);
            const auto world_text = spade::world_to_yaml(s.world);
            const auto scene_text = spade::scene::scene_to_yaml(s.scene);
            ASSERT_TRUE(world_text.has_value()) << stem << ": " << world_text.error().context;
            ASSERT_TRUE(scene_text.has_value()) << stem << ": " << scene_text.error().context;
            ASSERT_TRUE(write_text(drafts / "worlds" / world_file.filename(), *world_text));
            ASSERT_TRUE(write_text(drafts / "scenes" / scene_file.filename(), *scene_text));
            missing.push_back(stem);
            continue;
        }
        const auto composed = spade::scene::compose_file(scene_file);
        ASSERT_TRUE(composed.has_value()) << stem << ": " << composed.error().context;
        // The physics records world file v2 does not hold, from the viewer.
        auto run = spade::scene::instantiate(*composed, scene->worlds.worlds[lane], spade::viewer::kStepDtNs,
                                             spade::viewer::kSubsteps);
        ASSERT_TRUE(run.has_value()) << stem << ": " << run.error().context;
        successors.push_back(std::move(*run));
    }
    ASSERT_TRUE(missing.empty()) << c.scene << ": no world and scene files for " << ::testing::PrintToString(missing)
                                 << ". Drafts from the viewer scene are in " << drafts.string()
                                 << "; review them and copy them into " << assets.string() << ".";

    std::vector<std::vector<Pair>> pairs;
    for (uint32_t lane = 0; lane < c.lanes; ++lane) {
        auto made = pair_arrays(viewer.sim, lane, successors[lane].sim);
        ASSERT_TRUE(made.has_value()) << stem_of(c.scene, lane, c.lanes) << ": " << made.error().context;
        pairs.push_back(std::move(*made));
    }

    const char* gate = "SPADE_FULL_VIEWER_TRAJECTORIES";
    const uint64_t n = env_gate_is_set(gate) ? c.full_ticks : c.ticks;
    for (uint64_t t = 0;; ++t) {
        for (uint32_t lane = 0; lane < c.lanes; ++lane) {
            const std::string diff = first_difference(pairs[lane], lane);
            ASSERT_TRUE(diff.empty()) << stem_of(c.scene, lane, c.lanes) << " at tick " << t << ": " << diff;
        }
        if (t == n) {
            for (uint32_t lane = 0; lane < c.lanes; ++lane) {
                EXPECT_TRUE(pairs_still_valid(pairs[lane], viewer.sim, lane, successors[lane].sim))
                    << stem_of(c.scene, lane, c.lanes) << ": an array moved during the run, so the pairs went stale";
            }
            break;
        }
        if (scene->command_hook != nullptr) {
            scene->command_hook(viewer.sim, viewer.sim.tick().value, viewer.vehicle_refs);
            scene->command_hook(successors[0].sim, successors[0].sim.tick().value, successors[0].vehicles);
        }
        const spade::Result<void> stepped = viewer.sim.step(1);
        ASSERT_TRUE(stepped.has_value()) << c.scene << ": the viewer's step failed at tick " << t;
        for (spade::scene::SceneRun& s : successors) {
            const spade::Result<void> s_stepped = s.sim.step(1);
            ASSERT_TRUE(s_stepped.has_value()) << c.scene << ": a successor's step failed at tick " << t << ": "
                                               << s_stepped.error().context;
        }
    }
}

// N per scene, under the same cap as test_viewer_trajectories.cpp's (5 s a case
// on the debug preset), halved because each case steps two runs. shower's 1000
// bodies make even its first checkpoint too slow there, so by default it
// checks tick 0 (the files, the spawns and the setup) and steps under the gate.
INSTANTIATE_TEST_SUITE_P(Viewer, ViewerSuccessor,
                         ::testing::Values(Case{"drop", 1, 1000, 3000}, Case{"bounce", 4, 1250, 3000},
                                           Case{"shower", 1, 0, 250}, Case{"gate", 1, 1250, 3000},
                                           Case{"hover", 1, 1500, 3000}, Case{"wind", 1, 1500, 3000},
                                           Case{"flight", 1, 2100, 3000}, Case{"swarm", 4, 750, 3000}),
                         [](const ::testing::TestParamInfo<Case>& info) { return std::string(info.param.scene); });

}  // namespace
