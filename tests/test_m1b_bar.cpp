#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"

// ---------------------------------------------------------------------------
// Task 20 -- the M1B charter bar, as executable asserts.
//
// The brief's six bullets, and what "executable" means for each one here:
//
//   1. FIXED-STEP (no wall-clock symbol in the engine). A SOURCE-LEVEL SCAN,
//      not a link-level dumpbin audit -- see "THE CANARY MECHANISM, AND ITS
//      HONESTY TRADEOFF" below for why that choice was made and what it does
//      and does not prove.
//   2. HEADLESS (Simulation runs with SPADE_BUILD_V1=OFF). The SAME scanning
//      mechanism, pointed at a different forbidden pattern (a v1 #include),
//      PLUS an actual end-to-end vehicle+sensor run in this very process --
//      see bullet 5's test, which is exactly that run, and which succeeds in
//      a process whose link closure never pulled in `Spade` (tests/
//      CMakeLists.txt's target_link_libraries for spade_tests names
//      spade::core/world/state/physics/vehicles/sim and GTest, never
//      `Spade` -- a fact this test's own OWN scan re-verifies against the
//      CURRENT source tree rather than trusting the CMakeLists.txt prose).
//   3. SEEDED DETERMINISM (corpus green). A real determinism-replay run
//      (spade::testing -- the SAME mechanism test_determinism.cpp's corpus
//      uses) at SMOKE SCALE: one small scenario, not the four-scenario
//      corpus (which already gates in test_determinism.cpp -- rerunning it
//      here would test nothing this suite does not already test, at ten
//      times the cost).
//   4. SNAPSHOT (round-trip green). The SAME replay-guarantee shape
//      (snapshot mid-run, restore into a FRESH Simulation, resume, compare
//      against the uninterrupted digest) test_determinism.cpp's corpus-wide
//      test performs, at smoke scale and on its own small scenario.
//   5. 6-DOF + QUADROTOR + IMU (suites green). A short, real quadrotor
//      flight: spawn in trim, hold hover, step, poll the IMU, check both the
//      body's 6-DOF state and the sensor's output are physically sane --
//      NOT a rerun of test_quadrotor.cpp's or test_imu.cpp's own exhaustive
//      suites (which already gate), just enough of the same mechanism to
//      prove the three systems compose in one process.
//   6. THE SUMMARY TABLE. Prints PASS/FAIL for all six rows by calling the
//      SAME helper functions the five tests above call -- so the printed
//      table cannot silently drift from what was actually checked.
//
// ---------------------------------------------------------------------------
// THE CANARY MECHANISM, AND ITS HONESTY TRADEOFF (bullets 1 and 2)
//
// The brief's own words: "if a true link-level canary is not achievable
// cleanly with MSVC+Ninja, implement the strongest honest alternative (e.g.
// a dumpbin symbol audit wired as a test, or an include-level static check)
// and report the tradeoff." This file takes the SECOND option -- a
// SOURCE-LEVEL scan of every .hpp/.cpp under spade/engine/ (skipping
// tools/, the viewer's documented wall-clock exemption) for a short list of
// forbidden substrings -- and the reasoning is worth stating rather than
// just asserting:
//
//   * PORTABILITY. spade/CMakeLists.txt's own comment records that Task 4's
//     Linux CI configures SPADE_BUILD_V1=OFF specifically to build/test the
//     v2 engine tree on its own -- i.e. this suite is expected to run on a
//     platform where dumpbin.exe does not exist at all. A dumpbin-based
//     canary would SKIP unconditionally there, providing zero coverage on
//     the one CI job the "headless" bullet is actually about. A source scan
//     runs identically everywhere ctest runs.
//   * WHAT IT DOES NOT PROVE, stated rather than hidden. This is a TEXT
//     search, not a linker's closure: it cannot see a wall-clock call
//     reached only through an indirect function pointer whose target is
//     never named in the scanned text, and it cannot see one introduced by
//     a vendored dependency's own translation unit. A true link-level
//     symbol audit (e.g. dumpbin /symbols on each engine .lib, restricted to
//     the .lib files themselves and NOT the spade_tests executable --
//     GTest's own runtime legitimately reads a wall clock for test timing,
//     which would poison a whole-executable scan) would close that gap on
//     this box; it was not added here because a Windows-only bonus check
//     that provides no coverage on the platform the "headless" bullet cares
//     about was judged not worth the added complexity for THIS task. If a
//     future task wants the stronger guarantee, the .lib targets to point
//     it at are spade_core/spade_state/spade_world/spade_physics/
//     spade_sim/spade_vehicles (engine/CMakeLists.txt).
//   * WHAT IT DOES PROVE. Every wall-clock API this codebase's own
//     documentation names (world/medium.hpp, core/rng.hpp, this task's own
//     context) as the thing to avoid, and every literal v1 #include, in
//     every file this build actually compiles into the engine and its
//     tests -- checked baseline-clean (grep, by hand) before this list was
//     written, so the patterns are calibrated against a KNOWN-good tree
//     rather than guessed.
// ---------------------------------------------------------------------------

namespace {

namespace fs = std::filesystem;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}

#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// ---------------------------------------------------------------------------
// Source scanning -- the shared mechanism behind bullets 1 and 2.
// ---------------------------------------------------------------------------

// Every .hpp/.cpp under `root`, at any depth, EXCLUDING any path whose
// component list contains one of `skip_dirs` -- e.g. "tools", the viewer's
// documented wall-clock and v1 exemption (global-constraints.md; this
// task's own context).
[[nodiscard]] std::vector<fs::path> collect_sources(const fs::path& root,
                                                     std::initializer_list<std::string_view> skip_dirs) {
    std::vector<fs::path> out;
    if (!fs::exists(root)) return out;
    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(root)) {
        if (!entry.is_regular_file()) continue;
        const fs::path& p = entry.path();
        const std::string ext = p.extension().string();
        if (ext != ".hpp" && ext != ".cpp") continue;

        bool skip = false;
        for (const fs::path& part : p) {
            for (const std::string_view skip_name : skip_dirs) {
                if (part.string() == skip_name) {
                    skip = true;
                    break;
                }
            }
            if (skip) break;
        }
        if (!skip) out.push_back(p);
    }
    std::sort(out.begin(), out.end());  // deterministic report order
    return out;
}

// Every line of `file`, 1-based line numbers implied by index + 1.
[[nodiscard]] std::vector<std::string> read_lines(const fs::path& file) {
    std::ifstream in(file);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) lines.push_back(std::move(line));
    return lines;
}

// Scans `files` for any of `forbidden` (plain substring match -- these are
// C++/Win32 identifiers, not regexes, so a substring match is exact and
// cannot under-match). Returns every "path:line: matched substring" hit, so
// a failure names exactly what to go fix rather than just "something is
// wrong somewhere".
[[nodiscard]] std::vector<std::string> scan_for_forbidden(const std::vector<fs::path>& files,
                                                           std::initializer_list<std::string_view> forbidden) {
    std::vector<std::string> hits;
    for (const fs::path& file : files) {
        const std::vector<std::string> lines = read_lines(file);
        for (std::size_t i = 0; i < lines.size(); ++i) {
            for (const std::string_view pattern : forbidden) {
                if (lines[i].find(pattern) != std::string::npos) {
                    std::ostringstream hit;
                    hit << file.string() << ":" << (i + 1) << ": matched \"" << pattern << "\"";
                    hits.push_back(hit.str());
                }
            }
        }
    }
    return hits;
}

// True iff `line` is a GENUINE #include DIRECTIVE naming something under the
// v1 `Spade/` tree -- i.e. the line, after trimming leading whitespace,
// syntactically STARTS WITH "#include". A plain substring search for
// "#include <Spade" would also match that very string appearing as DATA --
// which is exactly what happened the first time this ran: it flagged this
// file's OWN pattern-list literal (scan_for_forbidden's caller, below,
// necessarily writes the text "#include <Spade" somewhere so there is
// something to search FOR). Requiring the match to be the line's own leading
// token is what tells a real preprocessor directive apart from a string
// holding one, and it is the more precise check on its own merits, not just
// a workaround.
[[nodiscard]] bool is_v1_include_directive(const std::string& line) {
    std::size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
    const std::string_view rest(line.data() + i, line.size() - i);
    if (rest.substr(0, 8) != "#include") return false;
    return rest.find("<Spade") != std::string_view::npos || rest.find("\"Spade") != std::string_view::npos;
}

[[nodiscard]] std::vector<std::string> scan_for_v1_includes(const std::vector<fs::path>& files) {
    std::vector<std::string> hits;
    for (const fs::path& file : files) {
        const std::vector<std::string> lines = read_lines(file);
        for (std::size_t i = 0; i < lines.size(); ++i) {
            if (is_v1_include_directive(lines[i])) {
                std::ostringstream hit;
                hit << file.string() << ":" << (i + 1) << ": " << lines[i];
                hits.push_back(hit.str());
            }
        }
    }
    return hits;
}

// SPADE_ENGINE_DIR / SPADE_TESTS_DIR: absolute paths baked in at configure
// time (tests/CMakeLists.txt), the same discipline SPADE_GOLDEN_DIR already
// uses -- see that compile definition's own comment for why (the kat
// testing spec's no-CWD rule: a data location a test MUST find cannot be
// CWD-relative).

// Bullet 1 -- FIXED-STEP: no wall-clock symbol anywhere under spade/engine/
// except tools/ (the viewer's documented pacing exemption).
[[nodiscard]] bool no_wallclock_symbols_in_engine_source(std::string& detail) {
    const std::vector<fs::path> files = collect_sources(fs::path(SPADE_ENGINE_DIR), {"tools"});
    const std::vector<std::string> hits = scan_for_forbidden(
        files, {"chrono", "QueryPerformanceCounter", "QueryPerformanceFrequency", "GetSystemTime",
                "GetLocalTime", "GetTickCount", "timeGetTime", "std::time(", "::time(", "_ftime",
                "glfwGetTime"});
    if (hits.empty()) {
        std::ostringstream ok;
        ok << "0 wall-clock references in " << files.size() << " source files under engine/ (tools/ excluded)";
        detail = ok.str();
        return true;
    }
    std::ostringstream bad;
    bad << hits.size() << " wall-clock reference(s): ";
    for (std::size_t i = 0; i < hits.size() && i < 5; ++i) bad << (i == 0 ? "" : "; ") << hits[i];
    detail = bad.str();
    return false;
}

// Bullet 2's source-level half -- HEADLESS: no engine or test source file
// includes v1 (Spade/Spade.hpp or anything else under the `Spade/` tree)
// outside tools/. See this file's header comment for the functional half.
[[nodiscard]] bool no_v1_include_in_engine_or_test_source(std::string& detail) {
    std::vector<fs::path> files = collect_sources(fs::path(SPADE_ENGINE_DIR), {"tools"});
    const std::vector<fs::path> test_files = collect_sources(fs::path(SPADE_TESTS_DIR), {});
    files.insert(files.end(), test_files.begin(), test_files.end());

    const std::vector<std::string> hits = scan_for_v1_includes(files);
    if (hits.empty()) {
        std::ostringstream ok;
        ok << "0 v1 (Spade/*) includes in " << files.size()
           << " source files under engine/ (tools/ excluded) + tests/";
        detail = ok.str();
        return true;
    }
    std::ostringstream bad;
    bad << hits.size() << " v1 include(s): ";
    for (std::size_t i = 0; i < hits.size() && i < 5; ++i) bad << (i == 0 ? "" : "; ") << hits[i];
    detail = bad.str();
    return false;
}

// ---------------------------------------------------------------------------
// Bullet 3 -- SEEDED DETERMINISM (corpus green), smoke scale.
//
// One small scenario -- a body under a scripted drag+wrench, no geometry --
// run twice via spade::testing::run_scenario(), the EXACT mechanism
// test_determinism.cpp's four-scenario corpus uses. 40 steps, one body: this
// is a SMOKE test of the mechanism, not a rerun of the corpus (which stays
// exactly where it is, gating test_determinism.cpp).
// ---------------------------------------------------------------------------
[[nodiscard]] spade::testing::Scenario m1b_smoke_scenario() {
    spade::testing::Scenario s;
    s.name = "m1b_smoke";
    s.dt_ns = 2'000'000;  // 2 ms step
    s.substeps = 2;       // 1 ms substep
    s.steps = 40;

    s.build = []() -> spade::Result<spade::WorldSetDesc> {
        const spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                          .name("m1b_void")
                                                          .environment(spade::Environment{})
                                                          .capacities(spade::Capacities{2, 2, 1, 1})
                                                          .build();
        if (!world) return std::unexpected(world.error());

        spade::WorldInstanceDesc instance;
        instance.world = *world;
        instance.seed = 0x5EEDED7ULL;
        instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::light);
        instance.contacts.restitution_e = 0.0f;
        instance.contacts.friction_mu = 0.0f;
        instance.contacts.proxy_radius = 0.1f;
        instance.grid.cell_size = 0.5f;
        return spade::WorldSetDesc{{instance}};
    };

    s.setup = [](spade::Simulation& sim) -> spade::Result<void> {
        spade::BodySpawn body;
        body.pos = glm::vec3(0.0f, 50.0f, 0.0f);
        body.vel = glm::vec3(1.0f, 0.0f, -0.5f);
        body.mass = 0.8f;
        body.inv_inertia_diag = glm::vec3(100.0f);
        const spade::Result<spade::BodyRef> ref = sim.spawn(0, body);
        if (!ref) return std::unexpected(ref.error());

        spade::DragElementSpawn drag;
        drag.mode = spade::physics::drag_mode::quadratic;
        drag.area = 0.03f;
        drag.coeffs = glm::vec3(0.8f, 0.0f, 0.0f);
        const spade::Result<spade::DragElementRef> elem = sim.add_drag_element(*ref, drag);
        if (!elem) return std::unexpected(elem.error());
        return {};
    };

    s.input = [](spade::Simulation& sim, spade::Tick tick) -> spade::Result<void> {
        const spade::Result<spade::BodyRef> ref = sim.body_ref_at(0, 0);
        if (!ref) return {};  // not yet flushed on tick 0; nothing to command
        const float a = static_cast<float>(tick.value % 5u) - 2.0f;
        return sim.apply_wrench(*ref, glm::vec3(0.02f * a, 0.0f, 0.0f), glm::vec3(0.0f));
    };

    return s;
}

[[nodiscard]] bool seeded_determinism_smoke(std::string& detail) {
    const spade::testing::Scenario scenario = m1b_smoke_scenario();
    const spade::Result<uint64_t> first = spade::testing::run_scenario(scenario);
    if (!first) {
        detail = std::string("first run failed: ") + first.error().context;
        return false;
    }
    const spade::Result<uint64_t> second = spade::testing::run_scenario(scenario);
    if (!second) {
        detail = std::string("second run failed: ") + second.error().context;
        return false;
    }
    std::ostringstream out;
    out << "two independent runs of a " << scenario.steps << "-step smoke scenario both digest to 0x" << std::hex
        << *first;
    detail = out.str();
    return *first == *second;
}

// ---------------------------------------------------------------------------
// Bullet 4 -- SNAPSHOT (round-trip green), smoke scale.
//
// The SAME replay-guarantee shape test_determinism.cpp's
// SnapshotRestoreIntoAFreshSimulationResumesIdentically performs: run
// uninterrupted to `steps` for a reference digest, then run again but stop
// at steps/2, snapshot, restore the blob into a BRAND NEW Simulation (never
// the one that produced it -- the part that actually proves the blob, not
// the live object, carries the state), resume to `steps`, and compare.
// ---------------------------------------------------------------------------
[[nodiscard]] bool snapshot_roundtrip_smoke(std::string& detail) {
    const spade::testing::Scenario scenario = m1b_smoke_scenario();
    const uint64_t k = scenario.steps / 2;

    spade::Result<spade::Simulation> reference = spade::testing::start_scenario(scenario);
    if (!reference) {
        detail = std::string("reference start failed: ") + reference.error().context;
        return false;
    }
    if (spade::Result<void> r = spade::testing::advance_scenario(scenario, *reference, scenario.steps); !r) {
        detail = std::string("reference advance failed: ") + r.error().context;
        return false;
    }
    const uint64_t expected = spade::testing::state_digest(*reference);

    spade::Result<spade::Simulation> interrupted = spade::testing::start_scenario(scenario);
    if (!interrupted) {
        detail = std::string("interrupted start failed: ") + interrupted.error().context;
        return false;
    }
    if (spade::Result<void> r = spade::testing::advance_scenario(scenario, *interrupted, k); !r) {
        detail = std::string("interrupted advance failed: ") + r.error().context;
        return false;
    }
    const spade::Result<spade::SnapshotBlob> blob = interrupted->snapshot();
    if (!blob) {
        detail = std::string("snapshot failed: ") + blob.error().context;
        return false;
    }
    if (blob->tick().value != k) {
        detail = "snapshot tick mismatch";
        return false;
    }

    spade::Result<spade::Simulation> resumed = spade::testing::start_scenario(scenario);  // a FRESH Simulation
    if (!resumed) {
        detail = std::string("resumed start failed: ") + resumed.error().context;
        return false;
    }
    if (spade::Result<void> r = resumed->restore(*blob); !r) {
        detail = std::string("restore failed: ") + r.error().context;
        return false;
    }
    if (spade::Result<void> r = spade::testing::advance_scenario(scenario, *resumed, scenario.steps); !r) {
        detail = std::string("resumed advance failed: ") + r.error().context;
        return false;
    }
    const uint64_t actual = spade::testing::state_digest(*resumed);

    std::ostringstream out;
    out << "snapshot at tick " << k << ", restored into a fresh Simulation, resumed to " << scenario.steps
        << " -- digest 0x" << std::hex << actual << (actual == expected ? " == " : " != ") << "uninterrupted 0x"
        << expected;
    detail = out.str();
    return actual == expected;
}

// ---------------------------------------------------------------------------
// Bullet 5 -- 6-DOF + QUADROTOR + IMU (suites green), smoke scale.
//
// One quadrotor, spawned in trim (VehicleSpawn::rotor_omega ==
// hover_command(), sim/simulation.hpp's "avoids a rotor-spin-up transient"),
// held for half a second at 1 kHz, IMU polled. Checks 6-DOF (the body barely
// moves -- a hovering rigid body's full 6-DOF state staying near its spawn
// pose is only true if translation AND rotation both integrated correctly),
// quadrotor (the hold itself -- vehicles/quadrotor.hpp's whole derivation),
// and IMU (a hovering accelerometer reads +g opposite gravity -- sensors/
// imu.hpp section 1 -- which is only true if specific-force capture, the
// mount transform and the ring write all composed correctly).
// ---------------------------------------------------------------------------
[[nodiscard]] bool quadrotor_imu_smoke(std::string& detail) {
    constexpr float kG = 9.80665f;

    const spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                      .name("m1b_quad_void")
                                                      .environment(spade::Environment{})
                                                      .capacities(spade::Capacities{1, 5, 1, 1})
                                                      .build();
    if (!world) {
        detail = std::string("world build failed: ") + world.error().context;
        return false;
    }
    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0xB17;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.15f;
    instance.grid.cell_size = 0.4f;

    spade::Result<spade::Simulation> sim = spade::Simulation::create(spade::WorldSetDesc{{instance}}, 1'000'000, 1);
    if (!sim) {
        detail = std::string("Simulation::create failed: ") + sim.error().context;
        return false;
    }

    spade::vehicles::QuadrotorParams params;
    params.name = "m1b_quad";
    params.mass = 1.0f;
    params.inertia_diag = glm::vec3(0.02f, 0.03f, 0.025f);
    params.arm_length = 0.15f;
    params.rotor_height = 0.0f;
    for (spade::vehicles::RotorParams& rotor : params.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.12f;
        rotor.thrust_coeff = 1.0e-5f;
        rotor.torque_coeff = 1.6e-7f;
    }
    params.drag.mode = spade::physics::drag_mode::quadratic;
    params.imu.rate_divider = 1;  // ideal sensor, every substep

    const spade::Result<spade::vehicles::ModelType> model = spade::vehicles::make_quadrotor(params);
    if (!model) {
        detail = std::string("make_quadrotor failed: ") + model.error().context;
        return false;
    }
    const spade::Result<spade::ModelTypeId> id = sim->register_model(*model);
    if (!id) {
        detail = std::string("register_model failed: ") + id.error().context;
        return false;
    }

    const float hover = spade::vehicles::hover_command(params);
    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where.rotor_omega = hover;  // IN TRIM
    const spade::Result<spade::VehicleRef> vehicle = sim->spawn(0, *id, where);
    if (!vehicle) {
        detail = std::string("vehicle spawn failed: ") + vehicle.error().context;
        return false;
    }
    if (spade::Result<void> r = sim->flush_structural(); !r) {
        detail = std::string("flush_structural failed: ") + r.error().context;
        return false;
    }

    constexpr uint64_t kSubsteps = 500;  // 0.5 s at 1 kHz
    if (spade::Result<void> r = sim->step(kSubsteps); !r) {
        detail = std::string("step failed: ") + r.error().context;
        return false;
    }

    const spade::Result<const spade::BodyState*> body = sim->body(vehicle->body);
    if (!body) {
        detail = std::string("body read failed: ") + body.error().context;
        return false;
    }
    const float position_drift = glm::length((*body)->pos - where.pos);
    const float attitude_w = (*body)->orient.w;

    if (vehicle->imu_count == 0) {
        detail = "vehicle spawned with no IMU sensor";
        return false;
    }
    std::vector<spade::sensors::ImuSample> samples(spade::sensors::kRingDepth);
    const spade::Result<spade::ImuPoll> poll = sim->poll_imu(vehicle->imu_sensors[0], 0, samples);
    if (!poll) {
        detail = std::string("poll_imu failed: ") + poll.error().context;
        return false;
    }
    if (poll->samples.empty()) {
        detail = "IMU produced no samples over 500 substeps";
        return false;
    }
    const spade::sensors::ImuSample& latest = poll->samples.back();

    std::ostringstream out;
    out << kSubsteps << " substeps hovering: position drift " << position_drift << " m, orient.w " << attitude_w
        << ", " << poll->samples.size() << " IMU samples, latest accel.y " << latest.accel.y << " (+g = " << kG
        << "), latest gyro " << glm::length(latest.gyro) << " rad/s";
    detail = out.str();

    // THE CONTRACT, deliberately loose (this is a smoke test, not
    // test_quadrotor.cpp's ten-thousand-substep hold): position drift under
    // 5 cm, attitude within a hair of level, accel.y within 5% of +g
    // (sensors/imu.hpp section 1: level hover reads +g OPPOSITE gravity, not
    // ~0 -- imu.hpp section 5's "resting on the ground reads ~0" caveat does
    // NOT apply here; this vehicle is airborne under thrust the whole time),
    // gyro near zero (level, not rotating).
    if (position_drift >= 0.05f) return false;
    if (std::fabs(attitude_w) <= 0.999f) return false;
    if (latest.accel.y <= 0.95f * kG || latest.accel.y >= 1.05f * kG) return false;
    if (glm::length(latest.gyro) >= 0.05f) return false;
    return true;
}

}  // namespace

// ===========================================================================
// The six tests -- one clear responsibility each, per this task's own
// self-review contract.
// ===========================================================================

TEST(M1B, FixedStepNoWallClockSymbolsInEngineSource) {
    std::string detail;
    const bool ok = no_wallclock_symbols_in_engine_source(detail);
    EXPECT_TRUE(ok) << detail;
}

TEST(M1B, HeadlessNoV1IncludeInEngineOrTestSource) {
    std::string detail;
    const bool ok = no_v1_include_in_engine_or_test_source(detail);
    EXPECT_TRUE(ok) << detail;
}

TEST(M1B, SeededDeterminismCorpusMechanismGreenAtSmokeScale) {
    std::string detail;
    const bool ok = seeded_determinism_smoke(detail);
    EXPECT_TRUE(ok) << detail;
}

TEST(M1B, SnapshotRoundTripMechanismGreenAtSmokeScale) {
    std::string detail;
    const bool ok = snapshot_roundtrip_smoke(detail);
    EXPECT_TRUE(ok) << detail;
}

TEST(M1B, SixDofQuadrotorImuSuitesMechanismGreenAtSmokeScale) {
    std::string detail;
    const bool ok = quadrotor_imu_smoke(detail);
    EXPECT_TRUE(ok) << detail;
}

// ---------------------------------------------------------------------------
// The summary bar table. Re-derives all five booleans above through the SAME
// helper functions the five tests each call -- so this table can never
// report a row it did not actually (re-)check, and a failure here is not a
// sixth independent risk, it is one of the five above surfacing a second
// time with the full table for context.
//
// VISIBILITY NOTE, stated rather than assumed: ctest (test.ps1's own
// invocation) suppresses a passing test's stdout unless run with
// --verbose/-V or --output-on-failure (which only shows it for a FAILING
// test). This table therefore prints unconditionally into THIS test's own
// stdout either way; seeing it on a green run means passing -V to ctest, the
// same as any other diagnostic print in this suite -- there is no special
// exemption from that for M1B.
// ---------------------------------------------------------------------------
TEST(M1B, SummaryBarTable) {
    struct Row {
        const char* charter_item;
        bool ok;
        std::string detail;
    };
    std::vector<Row> rows;
    {
        std::string detail;
        const bool ok = no_wallclock_symbols_in_engine_source(detail);
        rows.push_back({"fixed-step   (no wall-clock symbol in engine source)", ok, detail});
    }
    {
        std::string detail;
        const bool ok = no_v1_include_in_engine_or_test_source(detail);
        rows.push_back({"headless     (no v1 include in engine/test source)", ok, detail});
    }
    {
        std::string detail;
        const bool ok = seeded_determinism_smoke(detail);
        rows.push_back({"determinism  (seeded, corpus mechanism, smoke)", ok, detail});
    }
    {
        std::string detail;
        const bool ok = snapshot_roundtrip_smoke(detail);
        rows.push_back({"snapshot     (round-trip, smoke)", ok, detail});
    }
    {
        std::string detail;
        const bool ok = quadrotor_imu_smoke(detail);
        rows.push_back({"6-dof+quad+imu (suites mechanism, smoke)", ok, detail});
    }

    std::printf("\n");
    std::printf("=================== Spade v2 M1B charter bar ===================\n");
    for (const Row& row : rows) {
        std::printf("[%s] %s\n", row.ok ? "PASS" : "FAIL", row.charter_item);
        std::printf("       %s\n", row.detail.c_str());
    }
    std::printf("==================================================================\n\n");
    std::fflush(stdout);

    for (const Row& row : rows) {
        EXPECT_TRUE(row.ok) << row.charter_item << ": " << row.detail;
    }
}

// ===========================================================================
// Addendum A3 -- "reset-preserves-roster round-trip" conformance case
// (Task 21 close-out; kat-open-agendas.md's T21 section carries the OTHER
// A3 case, reseed determinism, as a follow-up ticket instead).
//
// RULING, from this task's brief: do not build new engine surface during
// close-out. Addendum A3 names two conformance cases. reseed determinism
// needs Simulation::reseed(seed), which does not exist -- WorldParams::seed
// is fixed at create() from WorldSetDesc and never rewritten after (sim/
// simulation.hpp's WorldConfig carries "NO `seed` MEMBER, deliberately"),
// so that case is a follow-up ticket, not a test. THIS case needs no new
// surface: restore()'s own doc comment says a restore overwrites EVERY
// arena byte -- including each array's derived free lists and live counts
// -- and rewinds the tick to match, all in one call, which is exactly what
// "reset to an earlier point" means. register_model() is NOT part of the
// blob (restore() doc comment, again), so restoring into the SAME
// Simulation object -- one that never forgot its model registrations -- is
// what makes this a RESET rather than the resume-into-a-fresh-object shape
// test_quadrotor.cpp's AFlyingVehicleSurvivesASnapshotIntoASimulationThat
// NeverSpawnedIt already covers.
// ===========================================================================
TEST(M1B, ResetPreservesRosterRoundTrip) {
    const spade::Result<spade::WorldDesc> world =
        spade::WorldBuilder()
            .name("m1b_roster_void")
            .environment(spade::Environment{})
            .capacities(spade::Capacities{3, 15, 3, 1})  // headroom for 3 vehicles' worth transiently
            .build();
    ASSERT_OK(world);

    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0xA3;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.15f;
    instance.grid.cell_size = 0.4f;

    spade::Result<spade::Simulation> sim = spade::Simulation::create(spade::WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(sim);

    spade::vehicles::QuadrotorParams params;
    params.name = "m1b_roster_quad";
    params.mass = 1.0f;
    params.inertia_diag = glm::vec3(0.02f, 0.03f, 0.025f);
    params.arm_length = 0.15f;
    for (spade::vehicles::RotorParams& rotor : params.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.12f;
        rotor.thrust_coeff = 1.0e-5f;
        rotor.torque_coeff = 1.6e-7f;
    }
    params.drag.mode = spade::physics::drag_mode::quadratic;
    params.imu.rate_divider = 1;  // ideal sensor, every substep

    const spade::Result<spade::vehicles::ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);
    const spade::Result<spade::ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    const float hover = spade::vehicles::hover_command(params);

    // THE ROSTER: two vehicles, A and B, in trim.
    spade::VehicleSpawn where_a;
    where_a.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where_a.rotor_omega = hover;
    ASSERT_OK(sim->spawn(0, *id, where_a));

    spade::VehicleSpawn where_b;
    where_b.pos = glm::vec3(2.0f, 12.0f, -1.0f);
    where_b.rotor_omega = hover;
    ASSERT_OK(sim->spawn(0, *id, where_b));

    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->step(30));

    // THE SNAPSHOT -- the point a "reset" below will return to.
    const spade::Result<uint32_t> baseline_bodies = sim->live_body_count(0);
    ASSERT_OK(baseline_bodies);
    const spade::Result<uint32_t> baseline_rotors = sim->live_rotor_count(0);
    ASSERT_OK(baseline_rotors);
    const spade::Result<uint32_t> baseline_sensors = sim->live_imu_sensor_count(0);
    ASSERT_OK(baseline_sensors);
    const spade::Tick baseline_tick = sim->tick();
    const spade::Result<spade::SnapshotBlob> roster_blob = sim->snapshot();
    ASSERT_OK(roster_blob);
    const uint64_t baseline_digest = spade::testing::state_digest(*sim);

    // MUTATE THE ROSTER past the snapshot point: despawn A, spawn a THIRD
    // vehicle C (landing on A's just-freed slot -- lowest-free-first, same
    // as test_quadrotor.cpp's despawn-cascade test relies on), step further.
    // If restore() left any of A's, B's or C's rows behind, or the free
    // list pointed at the wrong slot afterward, the round trip below would
    // not come out exact.
    const spade::Result<spade::BodyRef> body_a = sim->body_ref_at(0, 0);
    ASSERT_OK(body_a);
    ASSERT_OK(sim->despawn(*body_a));
    spade::VehicleSpawn where_c;
    where_c.pos = glm::vec3(-3.0f, 8.0f, 4.0f);
    where_c.rotor_omega = hover * 0.7f;
    ASSERT_OK(sim->spawn(0, *id, where_c));
    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->step(20));

    ASSERT_NE(spade::testing::state_digest(*sim), baseline_digest) << "the mutation step above must actually "
                                                                       "change state, or the round-trip check "
                                                                       "below proves nothing";

    // THE RESET: restore the earlier snapshot into the SAME Simulation.
    ASSERT_OK(sim->restore(*roster_blob));

    // THE ROSTER IS BACK. Not just "same counts" -- the exact same
    // registered state the pre-mutation snapshot captured, byte for byte.
    EXPECT_EQ(sim->tick().value, baseline_tick.value);
    const spade::Result<uint32_t> restored_bodies = sim->live_body_count(0);
    ASSERT_OK(restored_bodies);
    EXPECT_EQ(*restored_bodies, *baseline_bodies);
    const spade::Result<uint32_t> restored_rotors = sim->live_rotor_count(0);
    ASSERT_OK(restored_rotors);
    EXPECT_EQ(*restored_rotors, *baseline_rotors);
    const spade::Result<uint32_t> restored_sensors = sim->live_imu_sensor_count(0);
    ASSERT_OK(restored_sensors);
    EXPECT_EQ(*restored_sensors, *baseline_sensors);
    EXPECT_EQ(spade::testing::state_digest(*sim), baseline_digest);

    // THE ROSTER IS USABLE, not just byte-identical dead arena bytes: refs
    // captured before a restore are stale (restore rewinds generation
    // counters too, per body_ref_at()'s own doc comment), so re-deriving
    // from the state the restore actually produced is the honest way to
    // confirm vehicle A -- gone a moment ago -- is genuinely flying again.
    const spade::Result<spade::BodyRef> restored_a = sim->body_ref_at(0, 0);
    ASSERT_OK(restored_a);
    const spade::Result<const spade::BodyState*> restored_a_body = sim->body(*restored_a);
    ASSERT_OK(restored_a_body);
    EXPECT_TRUE(std::isfinite((*restored_a_body)->pos.y));
    EXPECT_GT((*restored_a_body)->pos.y, 5.0f) << "vehicle A should be back near its hover altitude, not fallen";
}
