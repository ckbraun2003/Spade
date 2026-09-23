// ===========================================================================
// test_gpu_parity.cpp (S6 Task 6) -- THE FIRST LIVE CPU<->GPU PARITY RESULTS.
//
// Runs a scenario twice -- once through the CPU schedule, once through the
// Vulkan backend's ported kernels -- from the same world set, the same spawns
// and the same input script, and compares every quantity of every body against
// a table of measured-then-pinned tolerance bands (engine/testing/parity.hpp).
//
// ---------------------------------------------------------------------------
// WHY THIS IS BANDED AND NOT A DIGEST COMPARISON. engine/testing/parity.hpp's
// header carries the full argument; the short form is that the ported kernels
// use division and square root, Vulkan specifies both to <= 2.5 ulp rather than
// correctly rounded, and the global constraints allow that precisely BECAUSE
// the comparison is banded. A digest is a bit-equality instrument and would
// answer "different" to a one-ulp divide, which is the expected outcome, not a
// defect.
//
// ---------------------------------------------------------------------------
// THE ALLOWLIST IS GONE (S6 Task 8). EVERY CORPUS SCENARIO COMPARES, VERBATIM.
//
// Through Task 7 this file carried a table of VERDICTS -- each corpus scenario
// marked IN or OUT, an excluded one naming the pass that excluded it and the
// task that would port it -- because only some of the schedule's passes
// had kernels. A scenario whose CPU run exercised an un-ported pass was not a
// parity candidate at all: the two runs would have been running different
// physics, and any band wide enough to hide that would be wide enough to hide
// anything.
//
// TWO SCENARIOS WERE ALSO ADAPTED rather than merely excluded. `ballistic`
// (authored `turbulence: moderate`) and `shower` (`turbulence: light`) ran as
// `ballistic_calm` and `shower_calm`, with the one instance field the un-ported
// MediumUpdate pass owned overridden to none on the BUILT world set. Both
// adaptations are deleted here: the corpus files are consumed as filed, gusts
// and all.
//
// WHAT REPLACES THE TABLE is a COMPLETENESS assertion with no exclusion branch
// (ParityCorpus.EveryCorpusScenarioIsInTheParitySet): the five corpus names are
// checked against the scenarios directory in both directions, so adding a
// scenario file fails this suite until it gets a parity test, and deleting one
// fails it too. What the new form can no longer express is "this scenario is
// excluded" -- because after wave C there is no pass an exclusion could be
// claimed on behalf of.
//
// ---------------------------------------------------------------------------
// WHAT IS COMPARED, AND WHY IT IS NO LONGER JUST `bodies`.
//
// Wave C ports the three passes that write the OTHER registered arrays, so the
// band tables now cover them too: `dryden` (the gust filter's five normalized
// states AND its rng stream), `rotors` (shaft speed, the one field the rotor
// pass writes), `imu_sensors` (phase, both bias walks, the noise stream, the
// write cursor) and `imu_ring` (the samples themselves).
//
// Comparing only `bodies` would have been vacuous for exactly the passes this
// wave adds: a sensor that produced NO SAMPLES AT ALL, or a gust filter frozen
// at its initial state, changes no trajectory in a scenario whose bodies do not
// read them -- so every band in this file would have passed while two of the
// three new kernels did nothing.
//
// THE INTEGER LANES ARE COMPARED FOR BIT EQUALITY, not within a band
// (QuantityKind::bits, S6 Task 8). A stream position, a Box-Muller cache flag,
// a rate-divider phase, a monotonic sample index and a tick stamp are integers,
// and "approximately the same stream position" is not a thing: one draw out of
// step is a completely different -- and perfectly plausible -- sequence from
// then on.
// ===========================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "compute/backend.hpp"
#include "compute/grid_entry.hpp"
#include "compute/step_params.hpp"
#include "compute/vulkan/context.hpp"
#include "core/rng.hpp"
#include "physics/grid.hpp"
#include "physics/integrator.hpp"
#include "sensors/imu.hpp"
#include "sensors/rings.hpp"
#include "sim/simulation.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "vehicles/rotor.hpp"
#include "world/medium.hpp"
#include "sim/world_set.hpp"
#include "state/layout.hpp"
#include "testing/parity.hpp"
#include "testing/replay.hpp"
#include "sensors/gnss.hpp"
#include "testing/scenario_file.hpp"
#include "world/world_ref.hpp"

namespace {

using spade::BodySpawn;
using spade::Result;
using spade::Simulation;
using spade::WorldInstanceDesc;
using spade::WorldRef;
using spade::WorldSetDesc;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::vulkan_available;
using spade::testing::BandEntry;
using spade::testing::compare_arrays;
using spade::testing::ParityReport;
using spade::testing::QuantityKind;
using spade::testing::Scenario;
using spade::testing::ToleranceBand;

// A Vulkan validation message during a GPU test is a test FAILURE (this
// program's standing rule) -- the same sink tests/test_gpu_state_mirror.cpp
// installs, for the same reason.
void fail_on_validation_message(std::string_view message) noexcept {
    ADD_FAILURE() << "Vulkan validation message during a GpuParity test: " << message;
}

class GpuParityTest : public ::testing::Test {
protected:
    void SetUp() override { spade::compute::set_error_sink(&fail_on_validation_message); }
    void TearDown() override { spade::compute::set_error_sink(nullptr); }
};

// ---------------------------------------------------------------------------
// The band table for a body-only parity run.
//
// EVERY MUTABLE FIELD OF BodyState IS COVERED, and the offsets come from
// offsetof() rather than from typed numbers -- state/layout.hpp is the
// declaring side and this table follows it by construction.
//
// force_acc and torque_acc carry ZERO bands DELIBERATELY, and that is an
// assertion rather than an oversight: Integrate's step 7 clears both
// accumulators at the end of every substep on both paths, so after any
// completed step they are exactly (0,0,0) on both sides. A non-zero difference
// there would mean a kernel left a wrench behind -- a structural bug, not a
// rounding one -- and no band should hide it.
//
// mass, inv_inertia_diag and flags are NOT listed: no wave-A kernel writes
// them, and tests/test_gpu_state_mirror.cpp's VulkanStepTouchesOnlyTheBodies-
// Array already proves nothing outside `bodies` moves. `proxy_radius`
// (S6 Task 2 / D-S6-2; the field this comment used to name `_p0`, before the
// T2 merge's rename) is likewise unlisted -- collision_static.slang reads it
// live (the per-body sphere-proxy override, physics::effective_proxy_radius()'s
// sentinel semantics mirrored character-for-character), but no scenario in
// this file spawns a vehicle (the only writer, Simulation::spawn(world,
// ModelTypeId, VehicleSpawn)), so every body here stays at the zero-fill
// sentinel throughout, unwritten and identical on both legs.
[[nodiscard]] std::vector<BandEntry> body_bands(const ToleranceBand& pos, const ToleranceBand& vel,
                                                 const ToleranceBand& orient, const ToleranceBand& omega,
                                                 const ToleranceBand& specific_force) {
    return {
        {"bodies", "pos", offsetof(spade::BodyState, pos), 3, QuantityKind::components, pos},
        {"bodies", "vel", offsetof(spade::BodyState, vel), 3, QuantityKind::components, vel},
        {"bodies", "orient", offsetof(spade::BodyState, orient), 4, QuantityKind::quaternion, orient},
        {"bodies", "omega_body", offsetof(spade::BodyState, omega_body), 3, QuantityKind::components, omega},
        {"bodies", "specific_force", offsetof(spade::BodyState, specific_force), 3, QuantityKind::components,
         specific_force},
        {"bodies", "force_acc", offsetof(spade::BodyState, force_acc), 3, QuantityKind::components,
         ToleranceBand{0.0f, 0.0f}},
        {"bodies", "torque_acc", offsetof(spade::BodyState, torque_acc), 3, QuantityKind::components,
         ToleranceBand{0.0f, 0.0f}},
    };
}

// ---------------------------------------------------------------------------
// THE THREE ARRAYS WAVE C ADDED TO THE COMPARISON (S6 Task 8), each as a
// helper a scenario appends when it exercises the pass that writes it.
//
// APPENDED RATHER THAN ALWAYS PRESENT, deliberately. compare_arrays() reports
// `elements_compared` per row and run_parity() requires it to be non-zero, so a
// row naming an array a scenario never populates would still compare its
// zero-filled slots and pass vacuously. A scenario adds the rows for the passes
// it actually runs; which rows a scenario carries is therefore itself a
// statement about what it covers, readable at the call site.
// ---------------------------------------------------------------------------

// `dryden` -- one row per world, direct-indexed. EVERY scenario gets these,
// because MediumUpdate runs unconditionally on every world of every world set
// (five draws per substep whatever the sigmas are -- world/medium.hpp's
// determinism note), so there is no such thing as a scenario that does not
// exercise it.
//
// THE STREAM IS BIT-EXACT AND THE FILTER IS BANDED, and the split is the whole
// point of the row layout. `stream.state` is splitmix64's 64-bit position,
// advanced by engine/shaders/u64.slang's exact integer arithmetic -- if it ever
// differs, the two runs have drawn different NUMBERS of values and every
// comparison after it is meaningless. `has_cached` is the Box-Muller parity
// flag, likewise integer. `cached_gauss` and the five normalized states are
// floats that inherit Box-Muller's one banded operation (the sqrt).
// ONE SHARED BAND SET, NOT ONE PER SCENARIO, and that is a claim about the
// filter rather than a shortcut. Every other quantity in this file gets
// per-scenario bands because its divergence is amplified by the TRAJECTORY --
// "the divergence is a property of the RUN, not of the quantity"
// (engine/testing/parity.hpp). The Dryden filter is not in that loop at all: it
// is a fixed linear recurrence driven only by its own rng stream and its own
// world's constant parameters, and NOTHING about a body's state feeds back into
// it. Its arithmetic is therefore the same arithmetic in every scenario, and a
// per-scenario band would be ten copies of one measurement.
[[nodiscard]] std::vector<BandEntry> medium_bands() {
    const ToleranceBand filter = spade::testing::bands::medium::kFilterState;
    const ToleranceBand cached = spade::testing::bands::medium::kCachedGauss;
    return {
        {"dryden", "stream.state", offsetof(spade::DrydenState, stream) + offsetof(spade::rng::Stream, state),
         2, QuantityKind::bits, ToleranceBand{0.0f, 0.0f}},
        {"dryden", "stream.cached",
         offsetof(spade::DrydenState, stream) + offsetof(spade::rng::Stream, cached_gauss), 1,
         QuantityKind::components, cached},
        {"dryden", "stream.flag",
         offsetof(spade::DrydenState, stream) + offsetof(spade::rng::Stream, has_cached), 1,
         QuantityKind::bits, ToleranceBand{0.0f, 0.0f}},
        // u, v0, v1, w0, w1 -- five consecutive floats, compared as one row
        // because they are one filter state and share a band by construction.
        {"dryden", "filter_state", offsetof(spade::DrydenState, u), 5, QuantityKind::components, filter},
    };
}

// `rotors` -- `omega` is the ONE field the rotor pass writes (vehicles/rotor.hpp:
// "the only state this pass writes into its own row, and the reason `rotors` is
// mutable"). Everything else in the row is configuration the pass must leave
// alone, and tests/test_gpu_state_mirror.cpp's byte comparison is what catches a
// kernel scribbling on it.
[[nodiscard]] std::vector<BandEntry> rotor_bands(const ToleranceBand& omega) {
    return {
        {"rotors", "omega", offsetof(spade::vehicles::RotorRow, omega), 1, QuantityKind::components, omega},
    };
}

// `imu_sensors` + `imu_ring` -- everything SensorSynthesis writes, split by
// whether it is an integer or a float.
//
// `phase`, `noise.state`, `noise.flag` and `last_index` are BIT-EXACT claims and
// each is load-bearing in a different way: phase proves the two runs emitted on
// the same substeps, the stream proves they drew the same twelve normals per
// sample, and last_index proves they emitted the same NUMBER of samples. Get any
// of those wrong and the sample values would still be plausible.
//
// `index`/`tick` on the ring are the same claim at the sample level, and they
// are what makes the ring comparison meaningful at all: a sample compared at the
// wrong ring slot would be comparing two different instants.
[[nodiscard]] std::vector<BandEntry> imu_bands(const ToleranceBand& bias, const ToleranceBand& cached,
                                                const ToleranceBand& accel, const ToleranceBand& gyro) {
    using spade::sensors::ImuSample;
    using spade::sensors::ImuSensorRow;
    return {
        {"imu_sensors", "phase", offsetof(ImuSensorRow, phase), 1, QuantityKind::bits,
         ToleranceBand{0.0f, 0.0f}},
        {"imu_sensors", "bias_a", offsetof(ImuSensorRow, bias_a), 3, QuantityKind::components, bias},
        {"imu_sensors", "bias_g", offsetof(ImuSensorRow, bias_g), 3, QuantityKind::components, bias},
        {"imu_sensors", "noise.state", offsetof(ImuSensorRow, noise) + offsetof(spade::rng::Stream, state), 2,
         QuantityKind::bits, ToleranceBand{0.0f, 0.0f}},
        {"imu_sensors", "noise.cached",
         offsetof(ImuSensorRow, noise) + offsetof(spade::rng::Stream, cached_gauss), 1,
         QuantityKind::components, cached},
        {"imu_sensors", "noise.flag",
         offsetof(ImuSensorRow, noise) + offsetof(spade::rng::Stream, has_cached), 1, QuantityKind::bits,
         ToleranceBand{0.0f, 0.0f}},
        {"imu_sensors", "last_index", offsetof(ImuSensorRow, last_index), 2, QuantityKind::bits,
         ToleranceBand{0.0f, 0.0f}},
        {"imu_ring", "accel", offsetof(ImuSample, accel), 3, QuantityKind::components, accel},
        {"imu_ring", "gyro", offsetof(ImuSample, gyro), 3, QuantityKind::components, gyro},
        {"imu_ring", "index", offsetof(ImuSample, index), 2, QuantityKind::bits, ToleranceBand{0.0f, 0.0f}},
        {"imu_ring", "tick", offsetof(ImuSample, tick), 2, QuantityKind::bits, ToleranceBand{0.0f, 0.0f}},
    };
}


// ---------------------------------------------------------------------------
// `gnss_sensors` + `gnss_ring` -- everything the GNSS half of SensorSynthesis
// writes, split the same way imu_bands() splits: integers are BIT-EXACT claims,
// floats get a measured band.
//
// ⭐ WHY THIS TABLE EXISTS AT ALL, AND IT IS NOT ROUTINE COVERAGE:
// sensor_gnss.slang has been compiled, bound at 23/24/25 and dispatched every
// step since acec7f7f, and until this test it had NEVER EXECUTED OVER A LIVE
// ROW -- on any path. Every corpus scenario's receivers are inert (kind ==
// none) because no scenario can spawn one, so both backends' synthesis returned
// immediately and the parity band proved nothing about GNSS. A bug in that
// kernel was undetectable by anything in the tree.
//
// ⭐⭐ AND THE HARNESS WAS ALREADY RIGHT. run_parity() requires
// elements_compared > 0 per row, so this table CANNOT be added without a
// scenario that populates the arrays -- "compared nothing" is a failure, not a
// green. WHAT WAS MISSING WAS NEVER A GUARD. IT WAS A SCENARIO.
//
// The bit-exact claims are load-bearing in the same way the IMU's are, one
// layer over: `phase` proves both runs emitted on the same substeps, the stream
// proves they drew the same three gaussians per fix, and `last_index` proves
// they emitted the same NUMBER of fixes. Get any of those wrong and the
// positions would still be plausible.
//
// `sigma_h`/`sigma_v` on the FIX are copied from the row by the kernel and
// never computed, so they are bit-exact and cheap -- and they are the one pair
// that proves the kernel read the row it was dispatched for rather than a
// neighbouring one.
// ---------------------------------------------------------------------------
[[nodiscard]] std::vector<BandEntry> gnss_bands(const ToleranceBand& bias, const ToleranceBand& cached,
                                                const ToleranceBand& position,
                                                const ToleranceBand& velocity) {
    using spade::sensors::GnssFix;
    using spade::sensors::GnssSensorRow;
    return {
        {"gnss_sensors", "phase", offsetof(GnssSensorRow, phase), 1, QuantityKind::bits,
         ToleranceBand{0.0f, 0.0f}},
        {"gnss_sensors", "bias", offsetof(GnssSensorRow, bias), 3, QuantityKind::components, bias},
        {"gnss_sensors", "noise.state",
         offsetof(GnssSensorRow, noise) + offsetof(spade::rng::Stream, state), 2, QuantityKind::bits,
         ToleranceBand{0.0f, 0.0f}},
        {"gnss_sensors", "noise.cached",
         offsetof(GnssSensorRow, noise) + offsetof(spade::rng::Stream, cached_gauss), 1,
         QuantityKind::components, cached},
        {"gnss_sensors", "noise.flag",
         offsetof(GnssSensorRow, noise) + offsetof(spade::rng::Stream, has_cached), 1,
         QuantityKind::bits, ToleranceBand{0.0f, 0.0f}},
        {"gnss_sensors", "last_index", offsetof(GnssSensorRow, last_index), 2, QuantityKind::bits,
         ToleranceBand{0.0f, 0.0f}},
        {"gnss_ring", "position", offsetof(GnssFix, position), 3, QuantityKind::components, position},
        {"gnss_ring", "velocity", offsetof(GnssFix, velocity), 3, QuantityKind::components, velocity},
        {"gnss_ring", "sigma_h", offsetof(GnssFix, sigma_h), 1, QuantityKind::bits,
         ToleranceBand{0.0f, 0.0f}},
        {"gnss_ring", "sigma_v", offsetof(GnssFix, sigma_v), 1, QuantityKind::bits,
         ToleranceBand{0.0f, 0.0f}},
        {"gnss_ring", "index", offsetof(GnssFix, index), 2, QuantityKind::bits, ToleranceBand{0.0f, 0.0f}},
        {"gnss_ring", "tick", offsetof(GnssFix, tick), 2, QuantityKind::bits, ToleranceBand{0.0f, 0.0f}},
    };
}

// ---------------------------------------------------------------------------
// A HAND-BUILT Scenario, not a corpus one -- and that is the finding that made
// this leg cheap. run_parity() takes a `Scenario` (testing/replay.hpp), which is
// a plain struct of three std::function members, NOT a YAML-loaded one. So a
// receiver can be spawned in `setup` with no scenario-format change, no
// VehicleSpawn field, no corpus file and NO DIGEST MOVEMENT.
//
// It is also invisible to ParityCorpus.EveryCorpusScenarioIsInTheParitySet,
// which censuses on_disk against kCorpusScenarios in both directions -- a
// census over the CORPUS TABLE, not over parity tests, exactly as
// StructuralOpsAreCoveredTransitively is already outside it.
//
// THE BODY MUST MOVE AND ROTATE, or the comparison is over a constant. Gravity
// supplies the translation; a non-zero omega_body supplies the lever-arm term,
// which is the only path by which mount_pos reaches the reported position --
// without it the antenna offset is a fixed addition both backends would get
// right by doing nothing.
// ---------------------------------------------------------------------------
[[nodiscard]] Scenario gnss_receiver_scenario() {
    Scenario s;
    s.name = "gnss_receiver";
    s.dt_ns = 1'000'000;
    s.substeps = 1;
    s.steps = 200;

    s.build = []() -> Result<WorldSetDesc> {
        spade::Environment env;
        env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
        env.wind = glm::vec3(0.0f);
        env.air_density = 1.225f;

        spade::Capacities caps;
        caps.bodies = 2;
        caps.force_elements = 1;
        caps.sensors = 1;
        caps.contacts = 1;

        const Result<spade::WorldDesc> world =
            spade::WorldBuilder().name("gnss_void").environment(env).capacities(caps).build();
        if (!world) return std::unexpected(world.error());

        spade::physics::ContactParams contacts;
        contacts.restitution_e = 0.0f;
        contacts.friction_mu = 0.0f;
        contacts.proxy_radius = 0.0f;

        spade::physics::GridParams grid;
        grid.cell_size = 1.0f;

        spade::WorldInstanceDesc instance;
        instance.world = *world;
        instance.seed = 0x6E55ull;
        instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
        instance.contacts = contacts;
        instance.grid = grid;
        return WorldSetDesc{{instance}};
    };

    s.setup = [](Simulation& sim) -> Result<void> {
        spade::BodySpawn body;
        body.pos = glm::vec3(0.0f, 50.0f, 0.0f);
        body.vel = glm::vec3(3.0f, 0.0f, -1.5f);
        body.omega_body = glm::vec3(0.3f, -0.7f, 0.4f);  // the lever arm must rotate
        body.mass = 1.0f;
        body.inv_inertia_diag = glm::vec3(1.0f);
        const Result<spade::BodyRef> ref = sim.spawn(0, body);
        if (!ref) return std::unexpected(ref.error());

        spade::GnssSensorSpawn gnss;
        gnss.mount_pos = glm::vec3(0.25f, -0.5f, 0.75f);
        gnss.rate_divider = 1;  // emit every substep, so 200 steps give 200 fixes
        gnss.sigma_h = 1.5f;
        gnss.sigma_v = 2.5f;
        gnss.sigma_vel = 0.125f;
        gnss.bias_tau_s = 60.0f;
        gnss.sigma_bias = 0.8f;
        const Result<spade::GnssSensorRef> sensor = sim.add_gnss_sensor(*ref, gnss);
        if (!sensor) return std::unexpected(sensor.error());
        return {};
    };

    s.input = [](Simulation&, spade::Tick) -> Result<void> { return {}; };
    return s;
}

// Table concatenation, so a scenario reads as `join(body_bands(...),
// medium_bands(...))` rather than as a sequence of insert() calls.
[[nodiscard]] std::vector<BandEntry> join(std::vector<BandEntry> a, const std::vector<BandEntry>& b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

// ---------------------------------------------------------------------------
// Runs `scenario` end to end on BOTH backends and compares. The two legs differ only
// in the BackendDesc handed to Simulation::create() -- same world set builder,
// same setup closure, same per-tick input closure, same step count -- so a
// difference in the report is a difference in the PASSES and cannot be a
// difference in the experiment.
//
// Both Simulations are alive at once (that is what lets compare_arrays() take
// two live ArenaSets rather than two snapshots), and the CPU leg runs FIRST so
// that a failure in the far cheaper path is reported before any device work.
// ---------------------------------------------------------------------------
// An optional extra assertion run on BOTH finished Simulations, while they are
// still alive (S6 Task 7). Its one use today is the shower cases'
// NON-VACUITY check -- proof that the CollisionDynamic pass actually resolved
// pairs, which no band can tell you: a kernel that dispatched nothing at all
// would leave both legs agreeing perfectly, and the report would print a
// table of zeroes and call it parity.
using ParityCheck = void (*)(const Simulation& cpu, const Simulation& gpu, std::string_view label);

void run_parity(const Scenario& scenario, std::string_view label, const std::vector<BandEntry>& table,
                ParityCheck extra = nullptr) {
    Result<Simulation> cpu = spade::testing::start_scenario(scenario, BackendDesc{.kind = BackendKind::cpu});
    ASSERT_TRUE(cpu.has_value()) << "cpu leg: " << cpu.error().context;
    Result<Simulation> gpu =
        spade::testing::start_scenario(scenario, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(gpu.has_value()) << "gpu leg: " << gpu.error().context;

    // The two runs start from IDENTICAL state -- asserted, not assumed. Both
    // legs were built by the same closures, but a create()-time divergence
    // (a different seed derivation, a spawn the vulkan path rejected) would
    // otherwise show up later as a "parity failure" that had nothing to do
    // with the kernels.
    EXPECT_EQ(spade::testing::state_digest(*cpu), spade::testing::state_digest(*gpu))
        << label << ": the two legs did not start from identical state";

    ASSERT_TRUE(spade::testing::advance_scenario(scenario, *cpu, scenario.steps).has_value());
    ASSERT_TRUE(spade::testing::advance_scenario(scenario, *gpu, scenario.steps).has_value());
    ASSERT_EQ(cpu->tick().value, gpu->tick().value);
    ASSERT_EQ(cpu->tick().value, scenario.steps);

    const Result<ParityReport> report = compare_arrays(cpu->arenas(), gpu->arenas(), table);
    ASSERT_TRUE(report.has_value()) << report.error().context;

    // PRINTED ALWAYS, passing or failing: the measured-versus-pinned table is
    // this task's actual deliverable (the user checkpoint's centrepiece), and
    // a table printed only on failure is a table nobody reads. `ctest -V` is
    // what surfaces it.
    report->print(label);

    for (const spade::testing::QuantityReport& q : report->quantities) {
        EXPECT_TRUE(q.within_band())
            << label << ": '" << q.quantity << "' is outside its pinned band -- measured max |abs| "
            << q.max_abs << ", max rel " << q.max_rel << ", pinned (" << q.band.abs << ", " << q.band.rel
            << "); " << q.elements_outside_band << " of " << q.elements_compared << " elements";
        // The band table must not pass VACUOUSLY: a quantity nothing wrote
        // would report zero error and sail through. Every row here names a
        // field a wave-A kernel writes on every substep, so at least one
        // element must have been compared.
        EXPECT_GT(q.elements_compared, std::size_t{0}) << label << ": '" << q.quantity << "' compared nothing";
        if (q.kind_is_quaternion) {
            // integrate_orientation renormalizes on the way out on BOTH paths
            // (math_ops.cpp:47, integrate.slang's glm_quat_normalize), so a
            // drifting norm is a different defect from a component band being
            // exceeded -- see parity.hpp's header.
            EXPECT_LT(q.max_unit_norm_error, 1.0e-5f)
                << label << ": a quaternion drifted off the unit sphere by " << q.max_unit_norm_error;
        }
    }

    if (extra != nullptr) extra(*cpu, *gpu, label);
}

// ---------------------------------------------------------------------------
// The corpus.
// ---------------------------------------------------------------------------

[[nodiscard]] std::filesystem::path golden_dir() { return std::filesystem::path(SPADE_GOLDEN_DIR); }

[[nodiscard]] Result<spade::testing::LoadedScenario> load_scenario(std::string_view name) {
    return spade::testing::scenario_from_yaml(golden_dir() / "scenarios" /
                                              (std::string(name) + ".scenario.yaml"));
}

// Every scenario in tests/golden/scenarios/, and what each one covers. NO
// VERDICT COLUMN AS OF S6 TASK 8: there is no longer an un-ported pass a
// scenario could be excluded on behalf of, so every name here has a parity test
// below and the table's only job is COMPLETENESS in both directions.
//
// The `covers` string is what a reviewer reads to see the corpus as a set
// rather than as five files -- and what makes an accidental duplicate obvious.
struct CorpusScenario {
    const char* scenario;
    const char* covers;
};

constexpr CorpusScenario kCorpusScenarios[] = {
    {"ballistic",
     "VERBATIM as of S6 Task 8 (it ran as `ballistic_calm`, turbulence overridden to none, through "
     "Task 7). One body, quadratic drag, a 200-tick wrench script, an empty world SDF -- and "
     "`turbulence: moderate`, which makes it the corpus's smallest scenario in which the gust "
     "actually REACHES a body: a drag element is the only path from the medium to force_acc, and "
     "this is the only corpus scenario that has both."},
    {"bounce",
     "VERBATIM: four worlds, a four-rung restitution ladder, one body each, no turbulence, no "
     "rotors, no sensors. The contact torture case, and the one that was already bit-identical."},
    {"quad_hover",
     "VERBATIM as of S6 Task 8, and the scenario wave C exists for: a registered ModelType spawning "
     "one body + four rotor rows + a drag body + an IMU mount per world, two worlds, `turbulence: "
     "moderate`, a three-step collective script over 900 steps x 2 substeps. It is the ONLY corpus "
     "scenario that exercises the RotorElement pass or SensorSynthesis at all, and it exercises "
     "both together with the gust the rotors' inflow reads."},
    {"shower",
     "VERBATIM as of S6 Task 8 (it ran as `shower_calm` through Task 7). 100 spheres in an SDF "
     "bowl, `turbulence: light`: the CollisionDynamic torture case -- a hundred bodies in mutual "
     "contact resolving Gauss-Seidel in a pile."},
    {"two_world_isolation",
     "VERBATIM as of S6 Task 8. Two worlds at OVERLAPPING COORDINATES with `turbulence: moderate`, "
     "so it is simultaneously the broad phase's cross-world leak test and a two-world gust test: "
     "each world draws from its own seed, and a kernel that indexed `dryden` or `dryden_params` by "
     "anything but the world id would give both worlds one world's weather."},
};

}  // namespace

// ===========================================================================
// 0. The parity set is the WHOLE corpus -- ADDING A SCENARIO FORCES A TEST.
//
// Without this, a scenario added to the corpus would simply never appear in
// the parity set, and nothing would say so. Host-only: it reads a directory
// and a table, and never touches a device, so it runs on a CI machine with no
// GPU exactly as it runs here.
//
// SUITE NAME `ParityCorpus`, NOT `GpuParityCorpus` (S6 Task 6 review round 1,
// finding I4). tests/AppendSpadeLabels.cmake attaches the ctest "gpu" label by
// a NAME-PREFIX RULE -- `^Gpu[^.]*\.` -- and that label means "device-
// executing" (the set a caller drops with `ctest -LE gpu` on a machine with no
// GPU). A host-only test carrying it would be excluded by exactly the
// invocation it is meant to survive. The device-executing cases in this file
// keep the Gpu prefix; the two that never touch a device do not.
// ===========================================================================


// ---------------------------------------------------------------------------
// THE FIRST EXECUTION OF sensor_gnss.slang OVER A LIVE ROW, ON EITHER BACKEND.
//
// ⛔ THE BANDS WERE MEASURED, NOT GUESSED, AND THE FIRST PASS WAS PRE-REGISTERED
// TO FAIL. Every band was set to ZERO and the run was made expecting a red: a
// tolerance band must be measured, because too wide passes anything, too narrow
// is flaky, AND NEITHER FAILURE ANNOUNCES WHICH IT IS. A zero band forces the
// harness to print the delta for every quantity that is genuinely band-legal,
// so THE FAILURE REPORT WAS THE MEASUREMENT. See parity.hpp's gnss_receiver
// namespace for the three numbers and their derivation.
//
// ⭐ AND THE SECOND THING THAT PASS BOUGHT CANNOT BE GUESSED EITHER: NINE OF
// TWELVE QUANTITIES CAME BACK EXACT UNDER A ZERO BAND, so they keep the zero on
// their merits rather than by assumption. A band pinned over a quantity that is
// actually bit-exact is a guard that has been switched off without anyone
// deciding to.
// ---------------------------------------------------------------------------
TEST_F(GpuParityTest, GnssReceiverMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    const Scenario scenario = gnss_receiver_scenario();

    using namespace spade::testing::bands::gnss_receiver;
    run_parity(scenario, "gnss_receiver (1 receiver x 200 substeps at rate_divider 1)",
               gnss_bands(kGnssBias, /*cached=*/ToleranceBand{0.0f, 0.0f}, kGnssPosition,
                          kGnssVelocity),
               [](const Simulation& cpu, const Simulation& gpu, std::string_view label) {
                   // THE PREMISE, ASSERTED BEFORE THE BANDS MEAN ANYTHING: the
                   // receiver is LIVE on both legs. run_parity already refuses a
                   // row that compared nothing, but that fires per band entry;
                   // this says the same thing about the thing itself, so a
                   // failure reads as "no receiver" rather than as twelve
                   // separate "compared nothing" lines.
                   const Result<uint32_t> cpu_live = cpu.live_gnss_sensor_count(0);
                   const Result<uint32_t> gpu_live = gpu.live_gnss_sensor_count(0);
                   ASSERT_TRUE(cpu_live.has_value() && gpu_live.has_value());
                   EXPECT_EQ(*cpu_live, 1u) << label << ": the cpu leg has no live receiver";
                   EXPECT_EQ(*gpu_live, 1u) << label << ": the gpu leg has no live receiver";
               });
}

TEST(ParityCorpus, EveryCorpusScenarioIsInTheParitySet) {
    std::vector<std::string> on_disk;
    std::error_code ec;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(golden_dir() / "scenarios", ec)) {
        const std::string filename = entry.path().filename().string();
        if (entry.is_regular_file() && filename.ends_with(".scenario.yaml")) {
            on_disk.push_back(filename.substr(0, filename.size() - std::string(".scenario.yaml").size()));
        }
    }
    std::sort(on_disk.begin(), on_disk.end());
    ASSERT_FALSE(on_disk.empty()) << "the scenario corpus is empty: " << (golden_dir() / "scenarios").string();

    for (const std::string& name : on_disk) {
        const auto found = std::find_if(std::begin(kCorpusScenarios), std::end(kCorpusScenarios),
                                        [&](const CorpusScenario& v) { return name == v.scenario; });
        EXPECT_NE(found, std::end(kCorpusScenarios))
            << "scenario '" << name << "' has no entry in this file's kCorpusScenarios table, and "
            << "therefore no parity test. Since S6 Task 8 every corpus scenario compares -- there is no "
            << "un-ported pass left to exclude one on behalf of -- so adding a scenario file means "
            << "adding a row here AND a TEST_F below.";
    }
    for (const CorpusScenario& scenario : kCorpusScenarios) {
        EXPECT_NE(std::find(on_disk.begin(), on_disk.end(), std::string(scenario.scenario)), on_disk.end())
            << "kCorpusScenarios names '" << scenario.scenario << "', which is not in the corpus any more";
    }

    // The set's size, pinned. It is now simply the corpus's size -- the two
    // numbers are the same number as of S6 Task 8, which is the whole claim
    // this suite exists to make -- so a scenario silently leaving the parity set
    // has nowhere to hide.
    EXPECT_EQ(std::size(kCorpusScenarios), on_disk.size());
    EXPECT_EQ(std::size(kCorpusScenarios), std::size_t{5})
        << "the corpus is ballistic, bounce, quad_hover, shower and two_world_isolation, and ALL FIVE "
        << "are in the parity set as of S6 Task 8";
}

// ===========================================================================
// 1. ballistic -- THE CORPUS FILE, VERBATIM, TURBULENCE AND ALL (S6 Task 8).
//    MediumUpdate + the drag half of ForceElements + Integrate, 200 steps x
//    5 substeps, an empty world SDF and a scripted wrench on every tick.
//
// WHAT CHANGED AND WHY IT MATTERS. Through Task 7 this ran as `ballistic_calm`,
// with the instance's authored `turbulence: moderate` overridden to none on the
// built world set, because MediumUpdate was a stub. That adaptation is gone, and
// removing it is not cosmetic: this is the corpus's ONLY scenario in which a
// gust actually reaches a body. The medium reaches force_acc through a DRAG
// ELEMENT and nothing else (physics/forces.cpp samples it per element), `shower`
// and `two_world_isolation` spawn none, and `quad_hover`'s gust reaches its body
// through a rotor as well as a drag body. So this scenario is what makes the
// Dryden port's OUTPUT -- not merely its stream position -- a measured claim.
// ===========================================================================

TEST_F(GpuParityTest, BallisticMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<spade::testing::LoadedScenario> loaded = load_scenario("ballistic");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;

    // THE SCENARIO'S OWN TURBULENCE IS LIVE, asserted rather than assumed: if a
    // future edit to the corpus file calmed this world, the comparison would
    // silently stop covering the gust path and every band below would still
    // pass.
    {
        const Result<WorldSetDesc> desc = loaded->scenario.build();
        ASSERT_TRUE(desc.has_value()) << desc.error().context;
        ASSERT_FALSE(desc->worlds.empty());
        EXPECT_NE(desc->worlds[0].turbulence.sigma_w, 0.0f)
            << "ballistic.scenario.yaml is authored `turbulence: moderate`, and this test's whole "
            << "reason for being the gust case is that a drag element reads that gust every substep";
    }

    using namespace spade::testing::bands::ballistic;
    run_parity(loaded->scenario, "ballistic (200 steps x 5 substeps, gust + drag + integrate)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()));
}

// ===========================================================================
// 2. gate_fleet -- CollisionStatic against a four-primitive SDF union, four
//    worlds, 900 steps. THE SDF EVALUATOR'S PARITY TEST.
//
// gate.world.yaml is `union(union(union(plane, torus), box), box)`: a plane
// (analytic gradient), a torus (the pinned CENTRAL-DIFFERENCE gradient -- six
// primitive evaluations, each with its own sqrt, which is the longest div/sqrt
// chain anywhere in wave A), and two posts (the box's analytic exterior/interior
// branches). The four spawns below are placed so that each of those branches is
// actually taken rather than merely compiled.
// ===========================================================================

TEST_F(GpuParityTest, GateFleetMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    Scenario gate;
    gate.name = "gate_fleet";
    gate.dt_ns = 1'000'000;  // 1 ms step
    gate.substeps = 1;       // 1 kHz substep, matching `bounce`
    gate.steps = 900;

    gate.build = []() -> Result<WorldSetDesc> {
        WorldInstanceDesc prototype;
        // Uniform across the fleet, deliberately: `bounce` below is the
        // heterogeneous case (four different restitutions through the
        // per-world contact_params buffer), so this one holds that variable
        // still and varies the GEOMETRY the bodies meet instead.
        prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
        prototype.contacts.restitution_e = 0.2f;
        prototype.contacts.friction_mu = 0.3f;
        prototype.contacts.baumgarte_beta = 0.2f;
        prototype.contacts.slop = 1.0e-3f;
        prototype.contacts.proxy_radius = 0.1f;
        prototype.grid.cell_size = 0.5f;
        return spade::world_set_from(WorldRef{golden_dir() / "worlds" / "gate.world.yaml"}, 4,
                                     0x6A7E0F1EE7ULL, prototype);
    };

    gate.setup = [](Simulation& sim) -> Result<void> {
        // ONE body per world. That was originally what kept this scenario
        // clear of the then-unported CollisionDynamic pass; since S6 Task 7
        // ported it (and Task 8 deleted the gate entirely) it is simply what
        // makes gate_fleet a pure CollisionStatic case -- the body-body solver
        // has no pair to find, so every number in its table comes from the SDF
        // evaluator and the integrator.
        //
        // The gate world (tests/golden/worlds/gate.world.yaml) is a ground
        // plane at y = 0, a torus of major radius 1.5 / tube 0.15 centred at
        // (0, 1.8, 0) with its hole axis along +Z, and two 0.3 x 3.0 x 0.3
        // posts centred at (-1.8, 1.5, 0) and (+1.8, 1.5, 0). Each spawn below
        // is aimed at a different branch of the SDF:
        //   world 0  straight down onto the TORUS crown (central-difference
        //            gradient), then off it and onto the plane
        //   world 1  straight down onto a POST's top face (box EXTERIOR
        //            branch), then off it
        //   world 2  down beside the other post, drifting into its SIDE (box
        //            exterior again, a different face) and onto the plane
        //   world 3  a low horizontal shot INTO a post, then the plane
        //
        // ASYMMETRIC INERTIA ON EVERY SPAWN, deliberately. BodySpawn's default
        // inv_inertia_diag is (1, 1, 1), i.e. a SPHERICAL inertia -- and
        // gyroscopic_torque() is identically zero for a spherical body
        // (I*omega is then parallel to omega, so their cross product vanishes,
        // as core/math_ops.hpp says outright). A spherical spawn would leave
        // omega_body exactly constant for the whole run and make its parity row
        // a comparison of two unchanging numbers. Distinct, non-unit,
        // per-axis-different inertias make Integrate's step 4 -- the reciprocal
        // 1/inv_I, the gyroscopic cross product, and the componentwise
        // I^-1 * tau -- genuinely live in this scenario.
        const std::array<BodySpawn, 4> spawns = {{
            [] { BodySpawn b; b.pos = {0.0f, 5.0f, 0.0f};   b.vel = {0.4f, 0.0f, 0.0f};  b.omega_body = {0.2f, -0.1f, 0.15f};  b.mass = 0.85f; b.inv_inertia_diag = {110.0f, 95.0f, 130.0f}; return b; }(),
            [] { BodySpawn b; b.pos = {1.8f, 5.0f, 0.0f};   b.vel = {-0.3f, 0.0f, 0.1f}; b.omega_body = {0.0f, 0.3f, -0.2f};   b.mass = 1.15f; b.inv_inertia_diag = {90.0f, 140.0f, 105.0f}; return b; }(),
            [] { BodySpawn b; b.pos = {-1.9f, 4.5f, 0.35f}; b.vel = {0.25f, 0.0f, -0.1f};b.omega_body = {-0.1f, 0.25f, 0.2f};  b.mass = 0.7f;  b.inv_inertia_diag = {125.0f, 80.0f, 115.0f}; return b; }(),
            [] { BodySpawn b; b.pos = {3.2f, 2.6f, 0.0f};   b.vel = {-2.0f, 0.0f, 0.0f}; b.omega_body = {0.35f, -0.15f, 0.1f}; b.mass = 1.4f;  b.inv_inertia_diag = {100.0f, 120.0f, 85.0f};  return b; }(),
        }};
        for (uint32_t w = 0; w < 4; ++w) {
            const Result<spade::BodyRef> ref = sim.spawn(w, spawns[w]);
            if (!ref) return std::unexpected(ref.error());
        }
        return {};
    };

    using namespace spade::testing::bands::gate_fleet;
    run_parity(gate, "gate_fleet (4 worlds x 900 steps, SDF union: plane + torus + 2 boxes)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()));
}

// ===========================================================================
// 3. bounce -- the golden corpus scenario VERBATIM. The restitution ladder,
//    i.e. the contact torture case AND the heterogeneous-material case (four
//    worlds, four different ContactParams, which is what the per-world
//    contact_params buffer exists for).
// ===========================================================================

TEST_F(GpuParityTest, BounceMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<spade::testing::LoadedScenario> loaded = load_scenario("bounce");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;

    using namespace spade::testing::bands::bounce;
    run_parity(loaded->scenario, "bounce (4-rung restitution ladder x 900 steps, contacts)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()));
}

namespace {

// ---------------------------------------------------------------------------
// RAW ARENA READS, for the NON-VACUITY checks the two corpus scenarios below
// need. Through the registry walk rather than through new Simulation
// accessors, for the reason first_rotor_omega() states further down: adding
// public API for a test assertion is a change made for a test's convenience.
// ---------------------------------------------------------------------------
[[nodiscard]] const spade::RegisteredArray* find_registered(const Simulation& sim, std::string_view name) {
    const spade::RegisteredArray* found = nullptr;
    sim.arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
        if (found == nullptr && array.name == name) found = &array;
    });
    return found;
}

template <class Row>
[[nodiscard]] Row read_row(const Simulation& sim, std::string_view array_name, std::size_t index) {
    Row row{};
    const spade::RegisteredArray* array = find_registered(sim, array_name);
    if (array != nullptr && array->byte_size() >= (index + 1) * sizeof(Row)) {
        std::memcpy(&row, array->data + index * sizeof(Row), sizeof(Row));
    }
    return row;
}

// ---------------------------------------------------------------------------
// two_world_isolation's NON-VACUITY CHECK: the two worlds really did draw
// DIFFERENT weather, on both legs.
//
// WHY A BAND CANNOT SAY THIS. The parity table compares world w's `dryden` row
// on the CPU against world w's on the GPU. A kernel that served BOTH worlds
// world 0's row -- the off-by-one this scenario exists to catch -- would make
// both legs identically wrong and every band would pass. The property that
// distinguishes right from wrong is a WITHIN-RUN one: world 0's filter state
// must differ from world 1's, because their seeds differ.
//
// It is checked on the GPU leg AND the CPU leg, so a scenario whose two worlds
// somehow converged would be reported as a bad instrument rather than as a
// device failure.
// ---------------------------------------------------------------------------
void expect_two_worlds_drew_different_weather(const Simulation& cpu, const Simulation& gpu,
                                              std::string_view label) {
    const auto worlds_differ = [&](const Simulation& sim, const char* leg) {
        const spade::DrydenState w0 = read_row<spade::DrydenState>(sim, "dryden", 0);
        const spade::DrydenState w1 = read_row<spade::DrydenState>(sim, "dryden", 1);
        EXPECT_NE(w0.stream.state, w1.stream.state)
            << label << " (" << leg << "): the two worlds' gust streams are at the same position, so "
            << "they were seeded identically or one row is serving both worlds";
        EXPECT_NE(w0.u, w1.u) << label << " (" << leg << "): the two worlds' filter states agree";
        std::printf("    %s dryden: world0.u=%.9e world1.u=%.9e\n", leg, static_cast<double>(w0.u),
                    static_cast<double>(w1.u));
    };
    worlds_differ(cpu, "cpu");
    worlds_differ(gpu, "gpu");
}

// ---------------------------------------------------------------------------
// quad_hover's NON-VACUITY CHECK: the vehicle actually FLEW under rotor thrust
// and the sensor actually EMITTED, on both legs.
//
// WHY A BAND CANNOT SAY THIS EITHER, and it is the sharpest instance of the
// problem in this file. quad_hover spawns IN HOVER TRIM with all four sigmas
// zero, so a `rotors` kernel that did nothing at all would leave omega at its
// spawned value on BOTH legs -- agreeing perfectly -- and a `sensor_imu` kernel
// that dispatched nothing would leave every ring slot zero on both legs, also
// agreeing perfectly. The table would print a column of zeroes and call it
// parity. So: the shaft speeds must be non-zero and the sample cursor must have
// advanced, measured on each leg independently.
//
// `omega > 0` ALONE IS NOT THAT CHECK (S6 hygiene: T8 review M5, strengthened
// here). quad_hover's spawn IS hover trim -- a specific, already-positive
// omega (`hover_command()`'s output for this airframe) -- so a completely
// DEAD RotorElement kernel that dispatched nothing at all would leave omega
// frozen at that spawned value, which is still > 0. The check below adds a
// SECOND, physically independent assertion a dead kernel cannot satisfy by
// construction: `BodyState::specific_force` (integrator.cpp:76,
// sensors/imu.hpp's convention) is the body's proper acceleration EXCLUDING
// gravity -- what an accelerometer reads. In true hover, thrust balances
// gravity, so specific_force's magnitude sits near standard gravity
// (`vehicles::kStandardGravity`, 9.80665). If the rotor pass produced no
// thrust, the body is in FREE FALL instead, and specific_force reads near
// ZERO (gravity itself is excluded from the quantity by definition) --
// a dead kernel therefore fails a `specific_force ~ g` check even though it
// would have passed `omega > 0`. The two together are strictly stronger than
// either alone: `omega > 0` still guards against a rotor row going inert
// mid-run (specific_force could plausibly still read near g for one substep
// off residual momentum), and `specific_force ~ g` is what a dead kernel
// cannot fake from spawn state alone.
// ---------------------------------------------------------------------------
void expect_quad_hover_flew_and_sensed(const Simulation& cpu, const Simulation& gpu,
                                       std::string_view label) {
    const auto check = [&](const Simulation& sim, const char* leg) {
        const spade::vehicles::RotorRow rotor = read_row<spade::vehicles::RotorRow>(sim, "rotors", 0);
        EXPECT_NE(rotor.enabled, 0u) << label << " (" << leg << "): rotor slot 0 is not live";
        EXPECT_GT(rotor.omega, 0.0f)
            << label << " (" << leg << "): rotor slot 0's shaft speed is not positive -- the "
            << "RotorElement pass produced no thrust, and this scenario's parity table would be a "
            << "comparison of two unchanging numbers";

        // See the header note above: this is the assertion a dead rotor
        // kernel cannot pass merely by leaving omega at its (already
        // positive) hover-trim spawn value.
        const spade::BodyState body0 = read_row<spade::BodyState>(sim, "bodies", 0);
        const float specific_force_mag = glm::length(body0.specific_force);
        EXPECT_GT(specific_force_mag, spade::vehicles::kStandardGravity * 0.5f)
            << label << " (" << leg << "): body 0's sensed specific_force magnitude is "
            << specific_force_mag << " m/s^2, less than half of standard gravity ("
            << spade::vehicles::kStandardGravity << ") -- a RotorElement pass that produced no "
            << "thrust would leave the body in free fall, where an accelerometer reads ~0 specific "
            << "force rather than the ~g a hovering vehicle's does";

        const spade::sensors::ImuSensorRow sensor =
            read_row<spade::sensors::ImuSensorRow>(sim, "imu_sensors", 0);
        EXPECT_EQ(sensor.kind, spade::sensors::sensor_kind::imu)
            << label << " (" << leg << "): sensor slot 0 is not a live IMU";
        EXPECT_GT(sensor.last_index, spade::sensors::SampleIndex{0})
            << label << " (" << leg << "): the sensor emitted NO samples, so every `imu_ring` row "
            << "compared above was the zero-fill on both sides";

        std::printf("    %s quad_hover: rotor0.omega=%.9e specific_force_mag=%.9e sensor0.last_index=%llu\n",
                    leg, static_cast<double>(rotor.omega), static_cast<double>(specific_force_mag),
                    static_cast<unsigned long long>(sensor.last_index));
    };
    check(cpu, "cpu");
    check(gpu, "gpu");
}

}  // namespace

// ===========================================================================
// 3b. two_world_isolation -- the golden corpus scenario VERBATIM (S6 Task 8;
//     it was EXCLUDED from the parity set through Task 7 because its instances
//     carry `turbulence: moderate` and MediumUpdate was a stub).
//
// TWO PROPERTIES AT ONCE, which is what makes it worth its own case rather than
// a fourth contact scenario:
//
//   1. CROSS-WORLD ISOLATION IN THE BROAD PHASE. Its two worlds hold bodies at
//      OVERLAPPING COORDINATES -- world 1's pair sits inside world 0's
//      eight-body cluster -- so a sweep that leaked across worlds would shove
//      world 1's bodies with neighbours they cannot see. On the device that
//      leak would be a bug in the sorted key's leading world field or in
//      collision_dynamic.slang's [lo, hi) binary search, neither of which any
//      single-world scenario can exercise.
//   2. PER-WORLD WEATHER. Both worlds are turbulent with DIFFERENT seeds, so
//      the `dryden` rows must diverge from each other while each matches its
//      CPU twin. A kernel that indexed `dryden` or `dryden_params` by anything
//      but the world id -- the classic off-by-one that gives every world world
//      0's weather -- fails here and nowhere else in this file, since every
//      other turbulent scenario has one world or identical instances.
// ===========================================================================

TEST_F(GpuParityTest, TwoWorldIsolationMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<spade::testing::LoadedScenario> loaded = load_scenario("two_world_isolation");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;

    // THE PREMISE, asserted rather than assumed: two worlds, both turbulent,
    // with DIFFERENT seeds. If the corpus file ever calmed them or gave them
    // one seed, property 2 above would silently stop being tested while every
    // band still passed.
    {
        const Result<WorldSetDesc> desc = loaded->scenario.build();
        ASSERT_TRUE(desc.has_value()) << desc.error().context;
        ASSERT_EQ(desc->worlds.size(), std::size_t{2});
        EXPECT_NE(desc->worlds[0].turbulence.sigma_w, 0.0f);
        EXPECT_NE(desc->worlds[1].turbulence.sigma_w, 0.0f);
        EXPECT_NE(desc->worlds[0].seed, desc->worlds[1].seed)
            << "the two worlds share a seed, so their gust filters would agree by construction and "
            << "a per-world indexing bug would be invisible";
    }

    using namespace spade::testing::bands::two_world_isolation;
    run_parity(loaded->scenario,
               "two_world_isolation (2 overlapping worlds x 300 steps x 2 substeps, per-world gusts)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()),
               &expect_two_worlds_drew_different_weather);
}

// ===========================================================================
// 3c. quad_hover -- THE SCENARIO WAVE C EXISTS FOR (S6 Task 8), the golden
//     corpus file VERBATIM.
//
// It was excluded through Task 7 on THREE separate counts at once, and all
// three are now ported: its rotors need the RotorElement half of ForceElements,
// its IMU mounts need SensorSynthesis, and its instances carry turbulence
// (MediumUpdate). It is the only corpus scenario that exercises any of them.
//
// WHAT IT COVERS THAT NOTHING ELSE IN THIS FILE DOES:
//   * THE WHOLE VEHICLE PATH -- a registered ModelType composing one body, four
//     rotor rows, a drag body and an IMU mount, spawned in exact hover trim,
//     flying a three-step collective script over 900 steps x 2 substeps.
//   * THE ROTOR PASS'S OWN STATE. `rotors.omega` is the only field that pass
//     writes, and the RPM lag makes it a per-substep recurrence: 1800 chained
//     applications of alpha = 1 - exp(-h/tau), which is the exp32 port under
//     continuous load.
//   * SENSOR SYNTHESIS END TO END: phase counting, twelve draws per emitted
//     sample, the bias walk, the mount rotation, and 1800 ring writes per
//     sensor into a 64-deep ring (so the ring has wrapped 28 times over by the
//     final tick, and the comparison is of the last 64 samples).
//   * THE THREE FORCE SOURCES INTERACTING. The gust feeds BOTH the rotors'
//     axial-inflow term and the drag body, in one substep, through one shared
//     medium sample -- which is exactly the coupling forces_drag.slang and
//     rotors.slang share dryden.slang's dryden_medium_sample() to guarantee.
//
// THE SIGMAS ARE ZERO ON THIS SENSOR (the corpus file authors sigma_a, sigma_g,
// sigma_ba and sigma_bg all 0), and that does NOT make the noise path untested:
// the twelve draws happen regardless (imu.hpp section 4's pinned draw count), so
// `noise.state` is a bit-exact claim about 21600 gaussian draws per sensor even
// though every one of them is multiplied by zero. What it does mean is that the
// SAMPLE VALUES are pure signal, which makes their band a clean measurement of
// the mount rotation and the centripetal term rather than of accumulated noise.
// ===========================================================================

TEST_F(GpuParityTest, QuadHoverMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<spade::testing::LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;

    using namespace spade::testing::bands::quad_hover;
    run_parity(loaded->scenario,
               "quad_hover (2 vehicles x 900 steps x 2 substeps, rotors + IMU + gusts)",
               join(join(join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()),
                         rotor_bands(kRotorOmega)),
                    imu_bands(kBias, kCachedGauss, kAccel, kGyro)),
               &expect_quad_hover_flew_and_sensed);
}

// ===========================================================================
// 4. THE StepParams ROUND TRIP, CLOSED (S6 Task 5 review round 1, finding C2).
//
// Task 5 proved the HOST half -- the per-submit write happens with the right
// value -- and deferred the device half: "wiring it into bindings.slang and
// proving a kernel can read it is Task 6's job, the first task with a real
// consumer". This is that proof.
//
// WHAT IT ASSERTS, AND WHY EACH HALF IS LOAD-BEARING:
//   * the WITNESS TICK equals the last submitted step's tick. Only a fresh
//     per-dispatch READ can produce that: the command buffer is recorded ONCE
//     and resubmitted n times unchanged, so a value baked in at record time
//     would be frozen at whatever the first submit carried (which is exactly
//     the failure mode that made a push constant unusable for the tick and
//     forced this buffer to exist at all).
//   * the WITNESS SUBSTEP equals substeps - 1. That comes from PassParams --
//     the push constant baked at record time -- so it proves the OTHER half of
//     the parameter path is live in the same kernel, and it pins the recorded
//     chain's ordering: the LAST substep's Integrate dispatch is the one that
//     wrote last.
//
// See compute/step_params.hpp's StepWitness doc comment for why wave A
// publishes a witness rather than inventing a physical dependency on a tick
// that none of its four passes needs.
// ===========================================================================

TEST_F(GpuParityTest, IntegrateKernelReadsThePerStepTickBuffer) {
    if (!vulkan_available()) GTEST_SKIP();

    WorldInstanceDesc prototype;
    prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    prototype.contacts.proxy_radius = 0.1f;
    prototype.grid.cell_size = 0.5f;
    const Result<WorldSetDesc> set = spade::world_set_from(
        WorldRef{golden_dir() / "worlds" / "gate.world.yaml"}, 2, 0x57E9C10CULL, prototype);
    ASSERT_TRUE(set.has_value()) << set.error().context;

    constexpr uint32_t kSubsteps = 4;
    Result<Simulation> sim =
        Simulation::create(*set, 4'000'000, kSubsteps, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    // Before any step the witness is the zero the mirror filled it with at
    // create() -- so the assertions below cannot pass on stale device memory.
    const Result<spade::compute::StepWitness> initial = sim->vulkan_step_witness();
    ASSERT_TRUE(initial.has_value()) << initial.error().context;
    EXPECT_EQ(spade::compute::step_witness_tick(*initial), 0u);
    EXPECT_EQ(initial->substep, 0u);

    // Seven steps from tick 0, so the last submit carries tick 6. Seven rather
    // than one because a single step cannot distinguish "read the buffer" from
    // "happened to hold the right value".
    ASSERT_TRUE(sim->step(7).has_value());
    ASSERT_EQ(sim->tick().value, 7u);

    const Result<spade::compute::StepWitness> witness = sim->vulkan_step_witness();
    ASSERT_TRUE(witness.has_value()) << witness.error().context;
    EXPECT_EQ(spade::compute::step_witness_tick(*witness), 6u)
        << "the Integrate kernel did not read the tick the host wrote before the LAST submit -- the "
        << "per-step buffer is not reaching the device, or the kernel is reading a stale copy";
    EXPECT_EQ(witness->substep, kSubsteps - 1u)
        << "the witness was written by a dispatch other than the last substep's, so either the recorded "
        << "chain's ordering or PassParams::substep is wrong";

    // And it MOVES with the tick: another three steps take it to 9. A witness
    // frozen at 6 would still satisfy every assertion above.
    ASSERT_TRUE(sim->step(3).has_value());
    const Result<spade::compute::StepWitness> later = sim->vulkan_step_witness();
    ASSERT_TRUE(later.has_value()) << later.error().context;
    EXPECT_EQ(spade::compute::step_witness_tick(*later), 9u);
}

// ---------------------------------------------------------------------------
// 5. THE UNPORTED-PASS GATE IS GONE (S6 Task 8), AND SO IS THE TEST THAT
//    PINNED IT.
//
// This slot held ParityGate.TurbulentWorldIsRefusedOnTheVulkanBackend from
// Task 6 through Task 7: a turbulent world set had to be REFUSED at create()
// with Code::unavailable naming MediumUpdate, because that pass was a stub and
// stepping such a world would have produced a wrong trajectory with nothing to
// say why. It was HOST-ONLY (the refusal happened before a device was ever
// requested), which is why it carried no `Gpu` prefix.
//
// Wave C ports MediumUpdate, the RotorElement half of ForceElements and
// SensorSynthesis -- every schedule slot now has a kernel or is inert by design
// on both backends -- so sim/simulation.cpp's two check_vulkan_unported_*
// functions are deleted, and a test asserting the refusal would now assert the
// opposite of the truth.
//
// IT IS REPLACED, NOT MERELY REMOVED, AND BY A STRICTLY STRONGER STATEMENT --
// the same posture Task 7 took when it deleted the two-active-bodies gate.
// `ballistic` (section 1) and `two_world_isolation` (section 3b) now STEP
// turbulent world sets on the vulkan backend and match the CPU within measured
// bands, gust filter and rng stream included; `quad_hover` (section 3c) does
// the same for rotors and IMU sensors. A test that only checked "no longer
// refused" would pass just as happily against a MediumUpdate kernel that
// dispatched nothing at all, which is precisely the silent absence the old gate
// existed to prevent.
//
// ONE CONSEQUENCE WORTH RECORDING because it is easy to lose: this file's
// HOST-ONLY case count drops from four back to three (ParityCorpus,
// ParityGeometry, ParityChaos). tests/CMakeLists.txt names them.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 5b. THE GATE THAT WAS DELETED (S6 Task 7), AND ITS REPLACEMENT.
//
// Through Task 6 this slot held SecondActiveBodyInAWorldIsRefusedOnTheVulkan-
// Backend: two active bodies in a world were refused outright, because
// CollisionDynamic was a stub and any contact between them would have been
// silently absent. That refusal is gone -- the pass is ported -- so the test
// that pinned it would now assert the opposite of the truth.
//
// It is replaced rather than merely removed, and the replacement is the
// STRONGER statement: the same two bodies now STEP, and the trajectory they
// produce matches the CPU's. A test that only checked "no longer refused"
// would pass just as happily against a CollisionDynamic kernel that dispatched
// nothing at all -- which is precisely the silent absence the old gate
// existed to prevent. The two spawns below are 0.15 m apart with a proxy
// radius of 0.1 m each, i.e. contact_dist = 0.2 m > 0.15 m, so they are
// OVERLAPPING at t = 0 and the pass has real work on its very first substep.
// ---------------------------------------------------------------------------

TEST_F(GpuParityTest, TwoActiveBodiesInAWorldStepAndMatchTheCpu) {
    if (!vulkan_available()) GTEST_SKIP();

    Scenario pair;
    pair.name = "contact_pair";
    pair.dt_ns = 1'000'000;  // 1 ms step
    pair.substeps = 1;       // 1 kHz substep
    pair.steps = 600;

    pair.build = []() -> Result<WorldSetDesc> {
        WorldInstanceDesc prototype;
        prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
        prototype.contacts.restitution_e = 0.4f;
        prototype.contacts.friction_mu = 0.3f;
        prototype.contacts.baumgarte_beta = 0.2f;
        prototype.contacts.slop = 1.0e-3f;
        prototype.contacts.proxy_radius = 0.1f;
        prototype.grid.cell_size = 0.5f;  // >= 2 * proxy_radius (grid.hpp's footgun note)
        return spade::world_set_from(WorldRef{golden_dir() / "worlds" / "gate.world.yaml"}, 1,
                                     0xB0D1E5ULL, prototype);
    };

    pair.setup = [](Simulation& sim) -> Result<void> {
        // UNEQUAL MASSES, deliberately: the mass weights w_a = mb/(ma+mb) and
        // w_b = ma/(ma+mb) are the one structural difference between
        // resolve_pair() and the static pass's impulse, and with equal masses
        // they are both exactly 0.5 -- a transcription that dropped them, or
        // swapped them, would be invisible. 0.6 and 1.4 make them 0.7 and 0.3.
        BodySpawn a{};
        a.pos = {0.0f, 5.0f, 0.0f};
        a.vel = {0.25f, 0.0f, 0.0f};
        a.omega_body = {0.2f, -0.1f, 0.15f};
        a.mass = 0.6f;
        a.inv_inertia_diag = {110.0f, 95.0f, 130.0f};
        if (const Result<spade::BodyRef> ref = sim.spawn(0, a); !ref) return std::unexpected(ref.error());

        BodySpawn b{};
        // 0.15 m from `a` against a contact distance of 0.2 m: OVERLAPPING at
        // t = 0, so the pass resolves a pair on its first substep rather than
        // waiting for the fall to bring them together.
        b.pos = {0.15f, 5.0f, 0.0f};
        b.vel = {-0.1f, 0.0f, 0.05f};
        b.omega_body = {-0.1f, 0.25f, 0.2f};
        b.mass = 1.4f;
        b.inv_inertia_diag = {125.0f, 80.0f, 115.0f};
        if (const Result<spade::BodyRef> ref = sim.spawn(0, b); !ref) return std::unexpected(ref.error());
        return {};
    };

    using namespace spade::testing::bands::contact_pair;
    run_parity(pair, "contact_pair (2 overlapping bodies, unequal masses, 600 steps)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()));
}

// ===========================================================================
// 6. COMPONENTWISE DRAG (S6 Task 6 review round 1, finding I1).
//
// WHAT WAS UNCOVERED. `ballistic` -- the only corpus scenario in the parity set
// with a drag element at all -- uses drag_mode::quadratic, and `bounce` and
// `gate_fleet` spawn no drag elements whatsoever. So the COMPONENTWISE branch
// of forces_drag.slang had zero parity coverage: the branch that rotates v_rel
// into the body frame with the orientation's conjugate, applies the per-axis
// law F_i = -c_i*|v_i|*v_i, and rotates the RESULT back out to world frame.
// That round trip is the most rotation-sensitive arithmetic in the drag kernel
// (two quaternion rotations per element per substep, each a pair of cross
// products), and it was compiled but never executed against the CPU.
//
// This scenario is synthetic and hand-built -- no corpus file is added, and
// none is needed: the void world (ballistic.world.yaml, no geometry at all) is
// exactly the right stage, because the point is the force law and not the
// contact solver.
//
// THE SPAWN IS CHOSEN SO NOTHING IS DEGENERATE:
//   * a TILTED, non-identity orientation -- with an identity quaternion the
//     conjugate-rotate-law-rotate round trip is the identity twice over and the
//     test would pass without exercising anything;
//   * non-zero omega AND asymmetric inertia, so the attitude keeps CHANGING
//     across the run and each substep rotates by a different amount;
//   * three DIFFERENT per-axis coefficients, so a kernel that read coeffs.x for
//     all three axes (the obvious transcription slip) fails;
//   * a non-zero, off-axis local_pos, so the body-frame torque cross product is
//     live rather than identically zero;
//   * a velocity with all three components non-zero and no symmetry.
// ===========================================================================

TEST_F(GpuParityTest, ComponentwiseDragMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    Scenario drag;
    drag.name = "drag_componentwise";
    drag.dt_ns = 2'000'000;  // 2 ms step
    drag.substeps = 2;       // 1 kHz substep
    drag.steps = 400;

    drag.build = []() -> Result<WorldSetDesc> {
        WorldInstanceDesc prototype;
        prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
        // No geometry in this world, so the contact record is inert; spelled
        // explicitly rather than left default so the intent is on the page.
        prototype.contacts.proxy_radius = 0.1f;
        prototype.grid.cell_size = 0.5f;
        return spade::world_set_from(WorldRef{golden_dir() / "worlds" / "ballistic.world.yaml"}, 1,
                                     0xD9A67C0DEULL, prototype);
    };

    drag.setup = [](Simulation& sim) -> Result<void> {
        BodySpawn body{};
        body.pos = {0.0f, 60.0f, 0.0f};
        body.vel = {4.5f, -1.25f, -3.0f};        // all three components live, no symmetry
        body.omega_body = {0.6f, -0.35f, 0.8f};  // attitude keeps moving
        body.mass = 0.75f;
        body.inv_inertia_diag = {115.0f, 85.0f, 140.0f};  // asymmetric: gyroscopic term is live
        // (w, x, y, z) at construction; spawn() normalizes. Deliberately far
        // from identity -- see the header note.
        body.orient = glm::quat(0.8f, 0.35f, -0.25f, 0.4f);
        const Result<spade::BodyRef> ref = sim.spawn(0, body);
        if (!ref) return std::unexpected(ref.error());

        spade::DragElementSpawn elem{};
        elem.mode = spade::physics::drag_mode::componentwise;
        elem.area = 0.0f;  // componentwise mode does not read it (physics/forces.hpp)
        elem.coeffs = {0.055f, 0.09f, 0.07f};        // three DIFFERENT per-axis coefficients
        elem.local_pos = {0.021f, -0.014f, 0.033f};  // off-axis: the torque cross product is live
        const Result<spade::DragElementRef> attached = sim.add_drag_element(*ref, elem);
        if (!attached) return std::unexpected(attached.error());
        return {};
    };

    using namespace spade::testing::bands::drag_componentwise;
    run_parity(drag, "drag_componentwise (400 steps x 2 substeps, per-axis body-frame drag law)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()));
}

// ===========================================================================
// 7. THE DIRTY-FLAG CENSUS (S6 Task 6 review round 1, findings C1 and I2).
//
// The vulkan path uploads `arenas_` only when it believes them stale, so every
// public entry point that can change the arenas between two step() calls owes a
// `mark_vulkan_dirty()`. sim/simulation.hpp carries the census in prose; this
// suite is its enforcement, one case per row, and the instrument in every case
// is the same: `vulkan_upload_count()` must advance across the next step().
//
// WHY A COUNTER AND NOT THE FLAG. `vulkan_dirty_` is private and, more to the
// point, is not the property that matters -- what matters is that the device
// actually receives the new bytes. The counter advances only on a SUCCESSFUL
// upload (compute/vulkan/backend.hpp), so it measures the outcome rather than
// the intent.
//
// Each case runs a CONTROL first: a step with nothing changed in between must
// NOT re-upload. Without that, a test would pass just as happily against an
// implementation that uploaded unconditionally on every step -- which would
// hide the very bug the flag exists to prevent from ever being detectable.
// ===========================================================================

namespace {

// A one-world, one-body void world (no geometry, no turbulence) -- the smallest
// thing that can legally be stepped on the vulkan backend. Every dirty-tracking
// case below wants a Simulation, not a scenario, so this returns the desc and
// the caller creates whichever backend it needs.
// Slot 0 of the `rotors` array's shaft speed, read straight out of the arena.
//
// THROUGH THE REGISTRY WALK, NOT THROUGH A NEW ACCESSOR. Simulation exposes
// world_bodies() but no world_rotors(), and adding one for a single test
// assertion would be a public API change made for a test's convenience --
// exactly what S6 Task 6b's brief ruled out ("no new convenience API this
// task"). The walk is already public (arenas().registry()), is what
// engine/testing/parity.hpp itself reads, and costs four lines here.
[[nodiscard]] float first_rotor_omega(const Simulation& sim) {
    float omega = std::numeric_limits<float>::quiet_NaN();
    sim.arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
        if (array.name != "rotors" || array.byte_size() < sizeof(spade::vehicles::RotorRow)) return;
        std::memcpy(&omega, array.data + offsetof(spade::vehicles::RotorRow, omega), sizeof(float));
    });
    return omega;
}

[[nodiscard]] Result<WorldSetDesc> dirty_test_world_set(uint32_t count, uint64_t seed) {
    WorldInstanceDesc prototype;
    prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    prototype.contacts.proxy_radius = 0.1f;
    prototype.grid.cell_size = 0.5f;
    return spade::world_set_from(WorldRef{golden_dir() / "worlds" / "ballistic.world.yaml"}, count, seed,
                                 prototype);
}

}  // namespace

TEST_F(GpuParityTest, ApplyWrenchForcesReupload) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = dirty_test_world_set(1, 0xA99117E0ULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    BodySpawn body{};
    body.pos = {0.0f, 10.0f, 0.0f};
    const Result<spade::BodyRef> ref = sim->spawn(0, body);
    ASSERT_TRUE(ref.has_value()) << ref.error().context;

    ASSERT_TRUE(sim->step(1).has_value());
    const uint64_t after_first = sim->vulkan_upload_count();
    EXPECT_EQ(after_first, 1u) << "the first step always uploads (vulkan_dirty_ starts true)";

    // CONTROL: nothing changed, so nothing re-uploads.
    ASSERT_TRUE(sim->step(1).has_value());
    ASSERT_EQ(sim->vulkan_upload_count(), after_first)
        << "no arena mutation between steps; upload_count must not move";

    // apply_wrench writes force_acc/torque_acc straight into the arena with no
    // structural queue involved -- before this task's fix, that write was never
    // uploaded and the input silently did nothing.
    ASSERT_TRUE(sim->apply_wrench(*ref, glm::vec3(0.5f, 0.0f, -0.25f), glm::vec3(0.0f, 0.01f, 0.0f))
                    .has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    EXPECT_EQ(sim->vulkan_upload_count(), after_first + 1)
        << "apply_wrench() between two steps must force exactly one re-upload";
}

TEST_F(GpuParityTest, RotorCommandsForceReupload) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = dirty_test_world_set(1, 0x0C0FFEE1ULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    // A MODEL WITH A REAL ROTOR AS OF S6 TASK 8 -- which is what the Task 6
    // version of this case said should happen "when Task 8 lands the rotor
    // pass", and it is a genuine strengthening rather than bookkeeping.
    //
    // Through Task 7 this test had to use a ZERO-ROTOR ModelType, because
    // check_vulkan_unported_state() refused to step any world set holding a
    // live rotor: a rotor-bearing vehicle could not be stepped on this backend
    // at all, so it could not be used to observe an upload counter, and the
    // most the case could reach was set_rotor_commands(ref, {}) running its
    // body over empty spans. So it proved the MARK was reached and nothing about
    // whether the command survived to the device.
    //
    // Now the rotor is real, the command is real and non-zero, and the step
    // actually runs the rotor kernel with it -- so the counter assertion is
    // backed by a re-upload that carries a value the device will USE. The final
    // read-back check below is what makes that difference visible.
    spade::vehicles::ModelType model{};
    model.name = "one_rotor_probe";
    model.body.mass = 1.0f;
    model.body.inertia_diag = {0.01f, 0.01f, 0.02f};
    model.proxy_radius = 0.0f;
    spade::vehicles::RotorDesc rotor{};
    rotor.local_pos = {0.0f, 0.02f, 0.0f};
    rotor.spin_dir = 1.0f;
    rotor.tau = 0.02f;
    rotor.radius = 0.12f;
    rotor.thrust_coeff = 1.0e-5f;
    rotor.torque_coeff = 1.6e-7f;
    model.rotors.push_back(rotor);
    const Result<spade::ModelTypeId> id = sim->register_model(model);
    ASSERT_TRUE(id.has_value()) << id.error().context;

    spade::VehicleSpawn where{};
    where.pos = {0.0f, 10.0f, 0.0f};
    const Result<spade::VehicleRef> vehicle = sim->spawn(0, *id, where);
    ASSERT_TRUE(vehicle.has_value()) << vehicle.error().context;
    ASSERT_EQ(vehicle->rotor_count, 1u);

    ASSERT_TRUE(sim->step(1).has_value());
    const uint64_t after_first = sim->vulkan_upload_count();
    EXPECT_EQ(after_first, 1u);

    // CONTROL.
    ASSERT_TRUE(sim->step(1).has_value());
    ASSERT_EQ(sim->vulkan_upload_count(), after_first);

    const float commanded = 500.0f;
    ASSERT_TRUE(sim->set_rotor_commands(*vehicle, std::span<const float>(&commanded, 1)).has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    EXPECT_EQ(sim->vulkan_upload_count(), after_first + 1)
        << "set_rotor_commands() between two steps must force exactly one re-upload";

    // AND THE COMMAND REACHED THE DEVICE AND WAS ACTED ON. The rotor started at
    // omega = 0 (VehicleSpawn's default) and the lag pulls it toward the
    // command by 1 - exp(-h/tau) per substep, so after one step it must be
    // strictly between 0 and the command. A re-upload that carried the old
    // omega_cmd, or a rotor kernel that did not run, would leave it at exactly
    // 0 -- and the counter assertion above would still have passed.
    const float omega_after = first_rotor_omega(*sim);
    EXPECT_GT(omega_after, 0.0f)
        << "the rotor's shaft speed did not move after a command + a step: either the re-upload did "
        << "not carry omega_cmd, or the rotors kernel did not run";
    EXPECT_LT(omega_after, commanded) << "the RPM lag must approach the command, not jump to it";
}

TEST_F(GpuParityTest, ReseedForcesReupload) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = dirty_test_world_set(2, 0x5EED5EEDULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    BodySpawn body{};
    body.pos = {0.0f, 10.0f, 0.0f};
    ASSERT_TRUE(sim->spawn(0, body).has_value());

    ASSERT_TRUE(sim->step(1).has_value());
    const uint64_t after_first = sim->vulkan_upload_count();
    EXPECT_EQ(after_first, 1u);

    // CONTROL.
    ASSERT_TRUE(sim->step(1).has_value());
    ASSERT_EQ(sim->vulkan_upload_count(), after_first);

    // reseed() rewrites WorldParams::seed in a REGISTERED array. Unmarked, the
    // next step would submit the old seeds and the readback would put them
    // straight back -- a call that appeared to succeed and changed nothing.
    const Result<const spade::WorldParams*> before = sim->world_params(0);
    ASSERT_TRUE(before.has_value()) << before.error().context;
    const uint64_t seed_before = (*before)->seed;

    ASSERT_TRUE(sim->reseed(0xFEEDFACEULL).has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    EXPECT_EQ(sim->vulkan_upload_count(), after_first + 1)
        << "reseed() between two steps must force exactly one re-upload";

    // AND THE NEW SEED SURVIVED THE ROUND TRIP. The counter proves an upload
    // happened; this proves the upload carried the reseed rather than the
    // readback erasing it -- which is the actual failure mode C1 named.
    const Result<const spade::WorldParams*> after = sim->world_params(0);
    ASSERT_TRUE(after.has_value()) << after.error().context;
    EXPECT_NE((*after)->seed, seed_before)
        << "the reseed was erased by the readback: WorldParams::seed came back as it was before";
}

TEST_F(GpuParityTest, StructuralOpsAreCoveredTransitively) {
    if (!vulkan_available()) GTEST_SKIP();

    // The census (simulation.hpp) claims spawn/despawn/add_drag_element/
    // add_imu_sensor need no mark of their own because each ALWAYS queues an op
    // and flush_structural() marks on a non-empty queue. That is an argument;
    // this is the measurement.
    const Result<WorldSetDesc> set = dirty_test_world_set(1, 0x57D0C7ULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    BodySpawn body{};
    body.pos = {0.0f, 10.0f, 0.0f};
    const Result<spade::BodyRef> ref = sim->spawn(0, body);
    ASSERT_TRUE(ref.has_value()) << ref.error().context;
    ASSERT_TRUE(sim->step(1).has_value());
    uint64_t expected = 1;
    ASSERT_EQ(sim->vulkan_upload_count(), expected) << "spawn + first step";

    // CONTROL, so nothing below can pass by uploading unconditionally.
    ASSERT_TRUE(sim->step(1).has_value());
    ASSERT_EQ(sim->vulkan_upload_count(), expected);

    spade::DragElementSpawn elem{};
    elem.mode = spade::physics::drag_mode::quadratic;
    elem.area = 0.02f;
    elem.coeffs = {1.0f, 0.0f, 0.0f};
    ASSERT_TRUE(sim->add_drag_element(*ref, elem).has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    ++expected;
    EXPECT_EQ(sim->vulkan_upload_count(), expected) << "add_drag_element";

    ASSERT_TRUE(sim->despawn(*ref).has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    ++expected;
    EXPECT_EQ(sim->vulkan_upload_count(), expected) << "despawn (and its element cascade)";

    // add_imu_sensor, MEASURED THE SAME WAY AS THE OTHERS AS OF S6 TASK 8.
    //
    // Through Task 7 this row was proved a different (and, at the time,
    // stronger) way: a live IMU sensor was REFUSED on this backend, and the
    // refusal ran ONLY inside step()'s `if (vulkan_dirty_)` branch -- so
    // getting the refusal at all was proof the queued sensor had marked the
    // mirror stale, and a missing mark would have shown up as a step that
    // cheerfully succeeded. That instrument is gone with the gate: a live IMU
    // sensor now steps.
    //
    // What replaces it is the counter every other row in this test uses, plus
    // something the refusal could never show -- that the sensor actually
    // PRODUCED a sample. A mark that never happened would leave the device
    // mirror without the sensor row at all, so SensorSynthesis would find
    // kind == none and emit nothing, and last_index would stay 0.
    const Result<spade::BodyRef> host = sim->spawn(0, body);
    ASSERT_TRUE(host.has_value()) << host.error().context;
    ASSERT_TRUE(sim->step(1).has_value());
    ++expected;
    ASSERT_EQ(sim->vulkan_upload_count(), expected) << "respawn";

    spade::ImuSensorSpawn imu{};
    imu.rate_divider = 1;  // emit every substep, so one step is enough to observe
    const Result<spade::ImuSensorRef> sensor = sim->add_imu_sensor(*host, imu);
    ASSERT_TRUE(sensor.has_value()) << sensor.error().context;
    ASSERT_TRUE(sim->step(1).has_value());
    ++expected;
    EXPECT_EQ(sim->vulkan_upload_count(), expected) << "add_imu_sensor";

    std::array<spade::sensors::ImuSample, spade::sensors::kRingDepth> out{};
    const Result<spade::ImuPoll> poll = sim->poll_imu(*sensor, 0, std::span<spade::sensors::ImuSample>(out));
    ASSERT_TRUE(poll.has_value()) << poll.error().context;
    EXPECT_GT(poll->samples.size(), std::size_t{0})
        << "a live IMU sensor stepped on the vulkan backend produced no samples -- either the queued "
        << "spawn never marked the device mirror stale (so the device never saw the sensor row) or "
        << "the SensorSynthesis dispatch did nothing";
}

// ===========================================================================
// 8. RESTORE ON THE VULKAN PATH (S6 Task 6 review round 1, finding C1 -- the
//    substantive half).
//
// THE BUG THIS PINS. `restore()` rewrites every registered array and is not a
// structural-queue op, so before the fix it left `vulkan_dirty_` false. A
// restore followed by a step on the vulkan path would therefore submit the
// STALE PRE-RESTORE state to the device, and the readback would overwrite the
// freshly restored arenas with the result -- the restore silently discarded,
// the run continuing from a state the caller had explicitly replaced.
//
// THE SHAPE OF THE TEST IS WHAT MAKES IT CATCH THAT. The vulkan leg
// deliberately STEPS TWICE BEFORE RESTORING: the first step clears the dirty
// flag and the second confirms it stays clear, so an unmarked restore would be
// invisible to every step after it. Restoring into a FRESHLY CREATED
// Simulation would NOT catch the bug, because create() starts the flag true and
// the first step would upload the restored state by accident.
//
// The comparison is the real one: the restored-and-resumed vulkan run against
// an uninterrupted CPU run of the same sequence, within pinned bands. A test
// that only checked the upload counter would prove an upload happened, not that
// it carried the right bytes.
// ===========================================================================

TEST_F(GpuParityTest, RestoredRunResumesAndMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    constexpr uint64_t kSnapshotTick = 300;
    constexpr uint64_t kFinalTick = 700;

    Scenario resume;
    resume.name = "restore_resume";
    resume.dt_ns = 1'000'000;
    resume.substeps = 1;
    resume.steps = kFinalTick;
    resume.build = []() -> Result<WorldSetDesc> {
        WorldInstanceDesc prototype;
        prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
        prototype.contacts.restitution_e = 0.35f;
        prototype.contacts.friction_mu = 0.3f;
        prototype.contacts.baumgarte_beta = 0.2f;
        prototype.contacts.slop = 1.0e-3f;
        prototype.contacts.proxy_radius = 0.1f;
        prototype.grid.cell_size = 0.5f;
        return spade::world_set_from(WorldRef{golden_dir() / "worlds" / "gate.world.yaml"}, 2, 0x2E570BEULL,
                                     prototype);
    };
    resume.setup = [](Simulation& sim) -> Result<void> {
        const std::array<BodySpawn, 2> spawns = {{
            [] { BodySpawn b; b.pos = {0.0f, 4.5f, 0.0f}; b.vel = {0.35f, 0.0f, 0.0f}; b.omega_body = {0.2f, -0.1f, 0.15f}; b.mass = 0.9f; b.inv_inertia_diag = {110.0f, 95.0f, 130.0f}; return b; }(),
            [] { BodySpawn b; b.pos = {1.8f, 4.5f, 0.0f}; b.vel = {-0.2f, 0.0f, 0.1f}; b.omega_body = {0.1f, 0.3f, -0.2f}; b.mass = 1.1f; b.inv_inertia_diag = {90.0f, 140.0f, 105.0f}; return b; }(),
        }};
        for (uint32_t w = 0; w < 2; ++w) {
            const Result<spade::BodyRef> ref = sim.spawn(w, spawns[w]);
            if (!ref) return std::unexpected(ref.error());
        }
        return {};
    };

    // --- the CPU reference: one uninterrupted run to kFinalTick -------------
    Result<Simulation> cpu = spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::cpu});
    ASSERT_TRUE(cpu.has_value()) << cpu.error().context;
    ASSERT_TRUE(spade::testing::advance_scenario(resume, *cpu, kFinalTick).has_value());

    // --- the blob: a second CPU run, snapshotted at kSnapshotTick -----------
    // Taken from its own Simulation rather than from the reference above, which
    // has already run past that tick. Same builder, same setup, so it is the
    // same experiment.
    Result<Simulation> source = spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::cpu});
    ASSERT_TRUE(source.has_value()) << source.error().context;
    ASSERT_TRUE(spade::testing::advance_scenario(resume, *source, kSnapshotTick).has_value());
    const Result<spade::SnapshotBlob> blob = source->snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    ASSERT_EQ(blob->tick().value, kSnapshotTick);

    // --- the vulkan leg: step first (clearing the dirty flag), THEN restore --
    Result<Simulation> gpu = spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    ASSERT_TRUE(gpu->step(1).has_value());
    const uint64_t uploads_before_restore = gpu->vulkan_upload_count();
    ASSERT_EQ(uploads_before_restore, 1u);
    // CONTROL: the mirror is clean now, so a bare step does not re-upload --
    // which is precisely the state in which an unmarked restore is invisible.
    ASSERT_TRUE(gpu->step(1).has_value());
    ASSERT_EQ(gpu->vulkan_upload_count(), uploads_before_restore);

    ASSERT_TRUE(gpu->restore(*blob).has_value());
    ASSERT_EQ(gpu->tick().value, kSnapshotTick) << "restore must also reset the clock";

    ASSERT_TRUE(spade::testing::advance_scenario(resume, *gpu, kFinalTick).has_value());
    EXPECT_EQ(gpu->vulkan_upload_count(), uploads_before_restore + 1)
        << "restore() must force exactly one re-upload -- and exactly one: the run after it queues "
        << "nothing, so a second would mean something else is marking the mirror stale too";
    ASSERT_EQ(gpu->tick().value, kFinalTick);
    ASSERT_EQ(cpu->tick().value, kFinalTick);

    using namespace spade::testing::bands::restore_resume;
    const std::vector<BandEntry> table =
        join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands());
    const Result<ParityReport> report = compare_arrays(cpu->arenas(), gpu->arenas(), table);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    report->print("restore_resume (vulkan restored at tick 300, resumed to 700, vs uninterrupted cpu)");

    for (const spade::testing::QuantityReport& q : report->quantities) {
        EXPECT_TRUE(q.within_band())
            << "restore_resume: '" << q.quantity << "' is outside its pinned band -- measured max |abs| "
            << q.max_abs << ", max rel " << q.max_rel << ", pinned (" << q.band.abs << ", " << q.band.rel
            << "); " << q.elements_outside_band << " of " << q.elements_compared << " elements. A GROSS "
            << "divergence here means the restore was discarded and the device stepped stale state.";
        EXPECT_GT(q.elements_compared, std::size_t{0});
    }
}

// ===========================================================================
// 9. HETEROGENEOUS-WORLDS PROVISION (S6 Task 6b, checkpoint-1 ruling): a
// two-world set built DIRECTLY as a vector<WorldInstanceDesc> -- no
// world_set_from()/replicate() convenience call (this task's brief,
// verbatim: "no new convenience API this task") -- from TWO DIFFERENT
// corpus world files: `ballistic` (empty SDF, no geometry at all) and `gate`
// (the plane+torus+two-box union GateFleetMatchesTheCpuWithinBands above
// already exercises). Both consumed AS-IS; neither .world.yaml is edited.
//
// WHAT THIS CLOSES THAT NOTHING ELSE IN THIS FILE DOES. `bounce` varies
// ContactParams across four worlds, but all four still share one (empty) SDF
// program -- a material-only difference. Nothing anywhere builds a set whose
// worlds carry genuinely different SDF PROGRAMS (different node counts,
// different transform counts). That is exactly the shape the per-world GPU
// geometry buffers (sdf_world_ranges/sdf_nodes/sdf_transforms, contact_params,
// and now grid_params -- S6 Task 6b's own binding 20) exist to serve, and it
// is untested until this task. Per-world CAPACITY heterogeneity is a
// different claim this test does NOT make (the two worlds' filed capacities
// differ -- see the override note below -- but both are overridden to the
// same value before either run, precisely so it is not what the comparison
// is sensitive to).
//
// ONE COMMON CAPACITY, OVERRIDDEN ON THE BUILT DESC, NEVER ON THE FILE.
// ballistic.world.yaml declares {bodies:4, elements:4, sensors:1, contacts:1};
// gate.world.yaml declares {bodies:5, elements:1, sensors:1, contacts:1} --
// different on purpose, each sized for its own corpus scenario. Left AS
// FILED, this set's WorldSetLayout would uniformize to the MAX over the set
// (body_capacity 5, element_capacity 4), which a SOLO run of either world
// alone would NOT reproduce (a solo ballistic run alone uniformizes to
// body_capacity 4, not 5) -- and sim/world_set.hpp's own note says a world's
// digest FOOTPRINT is a function of the SET's maximum capacity, not of that
// world's own, so a solo-vs-embedded digest comparison over differently-
// shaped sets would answer a shape question, not a physics one. Both
// instances are therefore given the SAME explicit capacity here --
// {1,1,1,1}, exactly what one spawned body needs -- so a solo 1-world set
// and this 2-world set uniformize to the identical shape either way. This
// overrides the BUILT WorldDesc's `capacities` field only; the same
// technique BallisticCalmMatchesTheCpuWithinBands (above) uses for
// `turbulence`.
// ===========================================================================

namespace {

[[nodiscard]] Result<WorldInstanceDesc> heterogeneous_instance(const std::filesystem::path& world_path,
                                                                uint64_t seed) {
    const Result<spade::WorldDesc> world = spade::resolve_world(WorldRef{world_path});
    if (!world) return std::unexpected(world.error());

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.world.capacities = spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1};
    instance.seed = seed;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.proxy_radius = 0.1f;
    instance.grid.cell_size = 0.5f;
    return instance;
}

constexpr uint64_t kHeteroBallisticSeed = 0x6E7EA05EEDULL;
constexpr uint64_t kHeteroGateSeed = 0x9A7EC7A7EULL;
constexpr uint64_t kHeteroDtNs = 1'000'000;  // 1 ms step, matching gate_fleet's proven timing
constexpr uint32_t kHeteroSubsteps = 1;
constexpr uint64_t kHeteroSteps = 900;

[[nodiscard]] Result<WorldInstanceDesc> heterogeneous_ballistic_instance() {
    return heterogeneous_instance(golden_dir() / "worlds" / "ballistic.world.yaml", kHeteroBallisticSeed);
}

[[nodiscard]] Result<WorldInstanceDesc> heterogeneous_gate_instance() {
    return heterogeneous_instance(golden_dir() / "worlds" / "gate.world.yaml", kHeteroGateSeed);
}

// World 0 = ballistic (empty SDF), world 1 = gate. World index is world
// identity (sim/world_set.hpp), so this order is a contract the spawns and
// the solo-run comparisons below both depend on.
[[nodiscard]] Result<WorldSetDesc> heterogeneous_world_set() {
    Result<WorldInstanceDesc> w0 = heterogeneous_ballistic_instance();
    if (!w0) return std::unexpected(w0.error());
    Result<WorldInstanceDesc> w1 = heterogeneous_gate_instance();
    if (!w1) return std::unexpected(w1.error());

    WorldSetDesc set;
    set.worlds = {std::move(*w0), std::move(*w1)};
    return set;
}

[[nodiscard]] BodySpawn heterogeneous_ballistic_spawn() {
    // No geometry in this world, no drag, no wrench: a plain free-falling,
    // tumbling body -- Integrate's full rotational chain (the inertia
    // reciprocal, the exp-map, the quaternion renormalization) is live,
    // CollisionStatic's dispatch runs and finds nothing (kSdfEmptyDistance).
    //
    // WHY pos.y = 2.0, SPECIFICALLY (fix round 1, finding I1) -- this is what
    // makes the lock BIDIRECTIONAL rather than only catching "world 1 loses
    // its own geometry". Over 900 x 1 ms with no drag, gravity alone drops the
    // body by ~3.976 m. Under the CORRECT (empty) SDF, world 0 free-falls
    // unimpeded to y ~ 2.0 - 3.976 = -1.98 m, in both the solo run and this
    // world's slice of the combined set -- so the CPU digest comparison is
    // unaffected by this choice. If world 0 were instead served world 1's
    // gate SDF (the "last world wins" / off-by-one bug class this whole task
    // exists to close), the ground plane at y = 0 would arrest it once
    // d < proxy_radius = 0.1, i.e. at y ~ 0.1 m -- a ~2.1 m divergence from
    // -1.98 m that both the pinned `pos` band (of order 1e-6) and the digest
    // EXPECT_EQ would catch instantly. At the ORIGINAL pos.y = 10.0, the body
    // ended at y ~ 6.02 m, never within 2.5 m of the gate's topmost solid
    // point (y = 3.45 m) -- so a phantom-gate-SDF bug on world 0 was
    // structurally undetectable: CollisionStatic would evaluate the gate
    // program, find d ~ 2.6, write nothing, and world 0's trajectory would be
    // bit-identical either way. y = 2.0 closes that blind spot while world 0
    // stays an empty-SDF world (the file is still consumed as-is; only the
    // spawn moved).
    BodySpawn b{};
    b.pos = {0.0f, 2.0f, 0.0f};
    b.vel = {0.3f, 0.0f, -0.2f};
    b.omega_body = {0.15f, 0.2f, -0.1f};
    b.mass = 1.0f;
    b.inv_inertia_diag = {100.0f, 120.0f, 90.0f};
    return b;
}

[[nodiscard]] BodySpawn heterogeneous_gate_spawn() {
    // Identical to gate_fleet's world-0 spawn (GateFleetMatchesTheCpuWithinBands,
    // above): already proven non-degenerate against exactly this geometry over
    // exactly this 900-step/1ms timing (it is what reaches the torus crown).
    BodySpawn b{};
    b.pos = {0.0f, 5.0f, 0.0f};
    b.vel = {0.4f, 0.0f, 0.0f};
    b.omega_body = {0.2f, -0.1f, 0.15f};
    b.mass = 0.85f;
    b.inv_inertia_diag = {110.0f, 95.0f, 130.0f};
    return b;
}

}  // namespace

// ---------------------------------------------------------------------------
// 9a. THE SOLO-RUN DIGEST LOCK -- HOST ONLY, no device: proves a world's
// trajectory does not depend on what else shares its world set, over a set
// whose worlds carry genuinely different GEOMETRY (not merely different
// material constants, which `bounce` already covers). Each world is stepped
// both ALONE (a 1-world set) and embedded in the 2-world heterogeneous set,
// and replay.hpp's world_digest() -- which normalizes away the two things a
// world's OWN state does not encode (its slot->world liveness and the
// whole-set replay_config identity) -- must agree between the two runs.
//
// HOST-ONLY, NOT Gpu*-PREFIXED (section 0's labelling rule): this is a pure
// CPU-backend comparison, so it runs on every CI runner with no device.
// ---------------------------------------------------------------------------

TEST(ParityGeometry, HeterogeneousGeometrySetMatchesSoloRuns) {
    const Result<WorldSetDesc> combined = heterogeneous_world_set();
    ASSERT_TRUE(combined.has_value()) << combined.error().context;

    Result<Simulation> combined_sim = Simulation::create(*combined, kHeteroDtNs, kHeteroSubsteps);
    ASSERT_TRUE(combined_sim.has_value()) << combined_sim.error().context;
    ASSERT_TRUE(combined_sim->spawn(0, heterogeneous_ballistic_spawn()).has_value());
    ASSERT_TRUE(combined_sim->spawn(1, heterogeneous_gate_spawn()).has_value());
    ASSERT_TRUE(combined_sim->step(kHeteroSteps).has_value());
    ASSERT_EQ(combined_sim->tick().value, kHeteroSteps);

    // World 0 (ballistic) alone -- the SAME instance (config, seed) and the
    // SAME spawn as its slice of the combined set above.
    {
        const Result<WorldInstanceDesc> w0 = heterogeneous_ballistic_instance();
        ASSERT_TRUE(w0.has_value()) << w0.error().context;
        WorldSetDesc solo;
        solo.worlds = {*w0};
        Result<Simulation> solo_sim = Simulation::create(solo, kHeteroDtNs, kHeteroSubsteps);
        ASSERT_TRUE(solo_sim.has_value()) << solo_sim.error().context;
        ASSERT_TRUE(solo_sim->spawn(0, heterogeneous_ballistic_spawn()).has_value());
        ASSERT_TRUE(solo_sim->step(kHeteroSteps).has_value());

        EXPECT_EQ(spade::testing::world_digest(*combined_sim, 0), spade::testing::world_digest(*solo_sim, 0))
            << "world 0 (ballistic, empty SDF) diverged between its solo run and its slice of the "
            << "heterogeneous 2-world set -- a world's trajectory must not depend on what else shares "
            << "its world set";
    }

    // World 1 (gate) alone -- same instance and spawn as its slice above; note
    // it is world index 0 of ITS OWN solo set (world_digest does not depend on
    // the world's global index, only its own content -- replay.hpp's header).
    {
        const Result<WorldInstanceDesc> w1 = heterogeneous_gate_instance();
        ASSERT_TRUE(w1.has_value()) << w1.error().context;
        WorldSetDesc solo;
        solo.worlds = {*w1};
        Result<Simulation> solo_sim = Simulation::create(solo, kHeteroDtNs, kHeteroSubsteps);
        ASSERT_TRUE(solo_sim.has_value()) << solo_sim.error().context;
        ASSERT_TRUE(solo_sim->spawn(0, heterogeneous_gate_spawn()).has_value());
        ASSERT_TRUE(solo_sim->step(kHeteroSteps).has_value());

        EXPECT_EQ(spade::testing::world_digest(*combined_sim, 1), spade::testing::world_digest(*solo_sim, 0))
            << "world 1 (gate geometry) diverged between its solo run and its slice of the heterogeneous "
            << "2-world set -- a world's trajectory must not depend on what else shares its world set";
    }
}

// ---------------------------------------------------------------------------
// 9b. THE GPU LEG: the SAME two-world set stepped on the vulkan backend,
// compared against the CPU within measured-then-pinned bands -- reusing
// run_parity() exactly like every scenario in section 1-3 above, which is
// what gives this test the Task-2 seam guard and the start-of-run digest
// check for free.
// ---------------------------------------------------------------------------

TEST_F(GpuParityTest, HeterogeneousGeometrySetMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    Scenario hetero;
    hetero.name = "heterogeneous_geometry_set";
    hetero.dt_ns = kHeteroDtNs;
    hetero.substeps = kHeteroSubsteps;
    hetero.steps = kHeteroSteps;
    hetero.build = []() -> Result<WorldSetDesc> { return heterogeneous_world_set(); };
    hetero.setup = [](Simulation& sim) -> Result<void> {
        if (const Result<spade::BodyRef> b0 = sim.spawn(0, heterogeneous_ballistic_spawn()); !b0) {
            return std::unexpected(b0.error());
        }
        if (const Result<spade::BodyRef> b1 = sim.spawn(1, heterogeneous_gate_spawn()); !b1) {
            return std::unexpected(b1.error());
        }
        return {};
    };

    using namespace spade::testing::bands::heterogeneous_geometry_set;
    run_parity(hetero, "heterogeneous_geometry_set (2 worlds: empty-SDF ballistic + gate geometry, 900 steps)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()));
}

// ===========================================================================
// 10. THE SHOWER -- CollisionDynamic's PARITY CASE, ON BOTH BATCHING SHAPES
//     (S6 Task 7).
//
// tests/golden/scenarios/shower.scenario.yaml is the corpus's contact torture
// case and says so itself: "100 spheres dropped into an SDF bowl ...
// CollisionStatic and CollisionDynamic both run hot and a pile forms. This is
// the scenario that would expose an ordering dependence in the sorted-grid
// sweep: a hundred bodies in mutual contact resolve Gauss-Seidel, so any
// instability in the sort order or the neighbour walk changes the pile."
//
// THE ONE ADAPTATION, and it is `ballistic`'s exactly (see this file's
// header): the instance is authored `turbulence: light` and MediumUpdate is
// ported at the time, so the built world set's turbulence was overridden to
// none. THE CORPUS TEST NO LONGER NEEDS THAT (S6 Task 8 ports MediumUpdate and
// GpuParityTest.ShowerMatchesTheCpuWithinBands runs the file verbatim), but
// shower_ladder is HAND-BUILT rather than loaded -- it has to be, since its
// whole point is a second world with a different restitution -- and
// shower_instance() below still constructs both worlds calm. That is now a
// CHOICE rather than a constraint, and the reason to keep it is that this case
// exists to isolate ONE variable, the per-world dispatch shape, against
// `shower`: the two scenarios differ in exactly that and nothing else.
//
// ---------------------------------------------------------------------------
// TWO TESTS, BECAUSE `batch_dynamic_collision` HAS TWO VALUES AND BOTH ARE
// DISPATCH SHAPES THIS PORT IMPLEMENTS.
//
// physics/schedule.cpp:59-73 runs the CPU pass either as ONE batched sweep
// over every world (when WorldSetLayout::uniform_dynamic_params holds) or as
// one sweep per world. compute/grid_entry.hpp's grid_domain_of() mirrors that
// choice on the device as two PARTITIONS of the key array -- one power-of-two
// segment covering every world's slots, or one segment per world -- and
// nothing else: both read `contact_params[world]`/`grid_params[world]` per
// world (checkpoint-1 ruling).
//
//   ShowerCalm...      1 world  -> uniform_dynamic_params TRUE  -> BATCHED
//   ShowerLadder...    2 worlds, world 1's restitution changed
//                               -> uniform_dynamic_params FALSE -> PER-WORLD
//
// Each test ASSERTS the layout flag it means to exercise, so a change in
// world_set.cpp's uniformity rule that silently collapsed the two shapes into
// one would fail here rather than quietly halving the coverage.
// ===========================================================================

namespace {

// shower.scenario.yaml's own lattice, from its header comment:
//     pos = (-1.2 + 0.3*(i%5), 0.35*(i/25), -1.2 + 0.3*((i/5)%5))
// evaluated in fp32. Used ONLY by the ladder scenario below, which is
// hand-built; the `shower_calm` test loads the corpus file and gets the
// tabulated 9-digit values from it.
[[nodiscard]] BodySpawn shower_lattice_spawn(uint32_t i) {
    BodySpawn b{};
    b.pos = {-1.2f + 0.3f * static_cast<float>(i % 5u), 0.35f * static_cast<float>(i / 25u),
             -1.2f + 0.3f * static_cast<float>((i / 5u) % 5u)};
    b.mass = 0.2f;
    b.inv_inertia_diag = {500.0f, 500.0f, 500.0f};
    return b;
}

// THE NON-VACUITY INSTRUMENT for the shower cases, and the reason it exists at
// all: a tolerance band cannot distinguish "the two runs agree because the
// port is right" from "the two runs agree because neither of them resolved a
// single pair". A CollisionDynamic kernel that dispatched nothing would print
// a table of zeroes and pass every band in this file.
//
// So this counts, at the FINAL tick, the pairs of active bodies in the same
// world whose separation is inside the contact distance -- i.e. pairs the
// sweep must have been resolving on the last substep -- and requires a
// non-zero count on BOTH legs. It uses the same predicate the pass does
// (contact_dist = the sum of the two effective proxy radii, physics/
// contacts.hpp's effective_proxy_radius) rather than a proxy for it.
//
// It is a LOWER bound on the pass's activity, deliberately: a pair that was
// resolved earlier in the run and has since separated does not count. A
// non-zero answer is proof; a zero answer would be grounds to ask whether the
// scenario reaches contact at all, which is exactly the question worth failing
// on.
[[nodiscard]] std::size_t contacting_pair_count(const Simulation& sim, float proxy_radius) {
    std::size_t pairs = 0;
    for (uint32_t w = 0; w < sim.layout().world_count; ++w) {
        const Result<std::span<const spade::BodyState>> bodies = sim.world_bodies(w);
        EXPECT_TRUE(bodies.has_value());
        if (!bodies) return 0;
        for (std::size_t i = 0; i < bodies->size(); ++i) {
            const spade::BodyState& a = (*bodies)[i];
            if ((a.flags & spade::physics::body_flags::active) == 0u) continue;
            for (std::size_t k = i + 1; k < bodies->size(); ++k) {
                const spade::BodyState& b = (*bodies)[k];
                if ((b.flags & spade::physics::body_flags::active) == 0u) continue;
                const float ra = spade::physics::effective_proxy_radius(a, proxy_radius);
                const float rb = spade::physics::effective_proxy_radius(b, proxy_radius);
                const glm::vec3 d = b.pos - a.pos;
                const float dist2 = glm::dot(d, d);
                const float contact_dist = ra + rb;
                if (dist2 > 0.0f && dist2 < contact_dist * contact_dist) ++pairs;
            }
        }
    }
    return pairs;
}

void expect_shower_resolved_pairs(const Simulation& cpu, const Simulation& gpu, std::string_view label) {
    constexpr float kShowerProxyRadius = 0.150000006f;  // the instance both shower cases build
    const std::size_t cpu_pairs = contacting_pair_count(cpu, kShowerProxyRadius);
    const std::size_t gpu_pairs = contacting_pair_count(gpu, kShowerProxyRadius);
    EXPECT_GT(cpu_pairs, std::size_t{0})
        << label << ": the CPU leg ended with NO pair inside contact distance -- this scenario never "
        << "exercised body-body contact, so its parity table proves nothing about CollisionDynamic";
    EXPECT_GT(gpu_pairs, std::size_t{0})
        << label << ": the GPU leg ended with NO pair inside contact distance -- the CollisionDynamic "
        << "chain may be dispatching nothing at all, which every band in this file would tolerate";
    std::printf("    contacting pairs at the final tick: cpu=%zu gpu=%zu\n", cpu_pairs, gpu_pairs);
}

[[nodiscard]] WorldInstanceDesc shower_instance(const spade::WorldDesc& world, uint64_t seed,
                                                 float restitution) {
    WorldInstanceDesc instance;
    instance.world = world;
    instance.seed = seed;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = restitution;
    instance.contacts.friction_mu = 0.349999994f;
    instance.contacts.baumgarte_beta = 0.200000003f;
    instance.contacts.slop = 0.00100000005f;
    instance.contacts.proxy_radius = 0.150000006f;
    instance.grid.cell_size = 0.300000012f;  // exactly 2 * proxy_radius, the cheapest correct setting
    return instance;
}

}  // namespace

// ---------------------------------------------------------------------------
// 10a. THE CHAOS CONTROL -- HOST ONLY, no device (S6 Task 7).
//
// WHY IT IS HERE, AND WHY IT COMES BEFORE THE TWO PARITY CASES. A settled
// 100-body pile resolved Gauss-Seidel is a CHAOTIC system, and this task
// measured how chaotic: at 400 steps the CPU and GPU legs of `shower` agree to
// ~4e-06 m/s, and at 800 steps -- once the pile has actually formed and every
// body is in permanent mutual contact -- they diverge to ~5e-01 m/s. That is a
// five-order-of-magnitude change from doubling the run, which is either
// amplification or a defect, and a parity table alone cannot tell you which.
//
// THIS TEST TELLS YOU WHICH, and it does so without a GPU in the room: it runs
// the SAME scenario twice ON THE CPU, identical in every respect except that
// ONE body's initial x is moved by ONE ULP (std::nextafter), and reports how
// far apart the two runs are at 400 and at 800 steps. If a single ulp of
// initial condition produces the same order of divergence the CPU<->GPU
// comparison sees, then what the parity table measured at 800 steps is the
// SCENARIO's Lyapunov amplification of last-bit differences -- which every
// implementation of this pass has, including two CPU runs -- and not a
// property of the port.
//
// It also fixes the band policy for the two cases below: they are pinned at
// the 400-step regime, where the divergence is still rounding-scale, and the
// 800-step measurement is REPORTED rather than banded over (the standing rule:
// "bands are NEVER widened to make a failure pass").
//
// HOST-ONLY, so NOT `Gpu`-prefixed (this file's section 0 labelling rule): two
// CPU runs, no device, runs on every CI machine.
// ---------------------------------------------------------------------------

TEST(ParityChaos, ShowerPileAmplifiesOneUlpOnTheCpuAlone) {
    const Result<spade::WorldDesc> world =
        spade::resolve_world(WorldRef{golden_dir() / "worlds" / "shower.world.yaml"});
    ASSERT_TRUE(world.has_value()) << world.error().context;

    // One world, shower's own material (or the ladder's bouncier one), shower's
    // own lattice. Perturbation applied to body 0's x only.
    const auto run = [&](bool perturb, uint64_t steps, float restitution) -> Result<std::vector<glm::vec3>> {
        WorldSetDesc set;
        set.worlds = {shower_instance(*world, 0xA11CE0F5E6DULL, restitution)};
        Result<Simulation> sim = Simulation::create(set, 2'000'000, 2);
        if (!sim) return std::unexpected(sim.error());
        for (uint32_t i = 0; i < 100; ++i) {
            BodySpawn b = shower_lattice_spawn(i);
            if (perturb && i == 0) {
                // ONE ULP. Not a "small" perturbation -- the smallest one that
                // exists at this magnitude, i.e. exactly the size of the
                // difference a <= 2.5-ulp GPU divide can introduce.
                b.pos.x = std::nextafter(b.pos.x, 1.0f);
            }
            if (const Result<spade::BodyRef> ref = sim->spawn(0, b); !ref) {
                return std::unexpected(ref.error());
            }
        }
        if (const Result<void> stepped = sim->step(steps); !stepped) {
            return std::unexpected(stepped.error());
        }
        const Result<std::span<const spade::BodyState>> bodies = sim->world_bodies(0);
        if (!bodies) return std::unexpected(bodies.error());
        std::vector<glm::vec3> out;
        out.reserve(bodies->size());
        for (const spade::BodyState& body : *bodies) out.push_back(body.pos);
        return out;
    };

    const auto worst_gap = [](const std::vector<glm::vec3>& a, const std::vector<glm::vec3>& b) {
        float worst = 0.0f;
        for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
            worst = std::max(worst, std::fabs(a[i].x - b[i].x));
            worst = std::max(worst, std::fabs(a[i].y - b[i].y));
            worst = std::max(worst, std::fabs(a[i].z - b[i].z));
        }
        return worst;
    };

    // FOUR MEASUREMENTS: two horizons x two restitutions. The second
    // restitution (0.8) is `shower_ladder`'s world 1 -- a far bouncier pile,
    // and the one whose CPU<->GPU figure at 800 steps is the largest this task
    // measured. Reporting both is what lets the report attribute that figure to
    // the MATERIAL rather than to the port.
    const auto gap_at = [&](uint64_t steps, float restitution) -> float {
        const Result<std::vector<glm::vec3>> base = run(false, steps, restitution);
        EXPECT_TRUE(base.has_value());
        const Result<std::vector<glm::vec3>> ulp = run(true, steps, restitution);
        EXPECT_TRUE(ulp.has_value());
        if (!base || !ulp) return 0.0f;
        return worst_gap(*base, *ulp);
    };

    const float gap_400_e02 = gap_at(400, 0.200000003f);
    const float gap_800_e02 = gap_at(800, 0.200000003f);
    const float gap_400_e08 = gap_at(400, 0.800000012f);
    const float gap_800_e08 = gap_at(800, 0.800000012f);

    std::printf("\n=== shower pile: CPU-vs-CPU worst |pos| gap from ONE ULP of initial x ===\n");
    std::printf("    e = 0.2   400 steps: %.9e m     800 steps: %.9e m\n",
                static_cast<double>(gap_400_e02), static_cast<double>(gap_800_e02));
    std::printf("    e = 0.8   400 steps: %.9e m     800 steps: %.9e m\n",
                static_cast<double>(gap_400_e08), static_cast<double>(gap_800_e08));

    // THE CLAIMS, IN THE ORDER THAT MATTERS.
    //
    // 1. The perturbation is REAL and does something -- otherwise everything
    //    below would be satisfied by a scenario that ignores it.
    EXPECT_GT(gap_400_e02, 0.0f) << "one ulp of initial condition changed nothing at all in 400 steps; "
                                 << "this control cannot say anything about amplification";

    // 2. IT AMPLIFIES WITH THE HORIZON. This is the mechanism, and it is what
    //    makes a band pinned at 400 steps the honest one and a band pinned at
    //    800 steps a band that would be absorbing amplification rather than
    //    bounding rounding.
    EXPECT_GT(gap_800_e02, gap_400_e02)
        << "the pile did not amplify a single ulp between the 400- and 800-step horizons on the CPU "
        << "alone; if that is true, the CPU<->GPU growth this task measured over the same interval "
        << "needs a different explanation and must be escalated rather than attributed to the "
        << "scenario";

    // 3. AND IT AMPLIFIES HARDER AT HIGHER RESTITUTION. `shower_ladder`'s
    //    world 1 is e = 0.8, and its 800-step CPU<->GPU figure is by far the
    //    largest this task measured; this is the control that says the
    //    MATERIAL, not the port, is what distinguishes it.
    EXPECT_GT(gap_800_e08, gap_800_e02)
        << "a bouncier pile did not amplify a single ulp further than a dead one over the same "
        << "horizon -- which would leave `shower_ladder`'s world-1 divergence unexplained";
}

// THE CORPUS FILE, VERBATIM, TURBULENCE INCLUDED (S6 Task 8). It ran as
// `shower_calm` through Task 7, with the authored `turbulence: light` overridden
// to none because MediumUpdate was a stub. The override is gone. Note what that
// does and does not change: shower spawns NO drag element and NO rotor, so a
// gust still reaches no body here -- what the restored turbulence adds is a
// live, non-zero-sigma Dryden filter running on the device for 800 substeps,
// compared against the CPU's by the `dryden` rows.
TEST_F(GpuParityTest, ShowerMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<spade::testing::LoadedScenario> loaded = load_scenario("shower");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;

    // THE SHAPE THIS TEST MEANS TO EXERCISE, asserted rather than assumed.
    {
        const Result<WorldSetDesc> desc = loaded->scenario.build();
        ASSERT_TRUE(desc.has_value()) << desc.error().context;
        const Result<spade::WorldSetLayout> layout = spade::validate_world_set(*desc);
        ASSERT_TRUE(layout.has_value()) << layout.error().context;
        ASSERT_TRUE(layout->uniform_dynamic_params)
            << "shower is a single-world set, so the BATCHED CollisionDynamic shape is what this test "
            << "covers -- if this flag is false the two shower tests cover the same shape twice";
    }

    using namespace spade::testing::bands::shower;
    run_parity(loaded->scenario, "shower (100 bodies in a bowl, 400 steps x 2 substeps, BATCHED grid)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()),
               &expect_shower_resolved_pairs);
}

TEST_F(GpuParityTest, ShowerLadderMatchesTheCpuWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    Scenario ladder;
    ladder.name = "shower_ladder";
    // shower's OWN step decomposition, verbatim, so the two shower tests differ
    // in exactly one thing -- the dispatch shape -- and their measured numbers
    // are directly comparable. Affordable because worlds are the axis the
    // device DOES parallelize: the sweep is sequential within a world but the
    // two worlds run concurrently, so the GPU leg costs about what
    // `shower_calm`'s does; only the CPU leg doubles.
    //
    // 400 STEPS AND NOT MORE, AND THE REASON IS A MEASUREMENT RATHER THAN A
    // BUDGET (S6 Task 7). This task ran both shower cases to 800 steps too, and
    // at that horizon the CPU<->GPU figures blow up -- to 4.6e-06 m of position
    // for the one-world case and 1.02e-02 m for this two-world one.
    // ParityChaos.ShowerPileAmplifiesOneUlpOnTheCpuAlone above is what explains
    // that without a device in the room: a SINGLE ULP of initial condition,
    // CPU-versus-CPU, amplifies to 1.0175e-02 m over the same 800 steps at
    // e = 0.8 -- within one percent of the CPU<->GPU figure. Past ~400 steps
    // this pile is simply not reproducible to rounding by ANY two
    // implementations that differ in a last bit, including two CPU runs, so a
    // band pinned there would be measuring the scenario's Lyapunov exponent
    // rather than the port's fidelity. The tripwire is therefore reported, not
    // banded over (engine/testing/parity.hpp's shower blocks carry the full
    // table).
    ladder.dt_ns = 2'000'000;
    ladder.substeps = 2;
    ladder.steps = 400;

    ladder.build = []() -> Result<WorldSetDesc> {
        const Result<spade::WorldDesc> world =
            spade::resolve_world(WorldRef{golden_dir() / "worlds" / "shower.world.yaml"});
        if (!world) return std::unexpected(world.error());

        // THE NON-UNIFORM TWIN: world 0 carries shower's own restitution
        // (0.2), world 1 carries a different one. That single byte is what
        // makes WorldSetLayout::uniform_dynamic_params false and sends both
        // the CPU pass and the device chain down the PER-WORLD shape -- and it
        // is also a real physics difference the parity comparison will see, so
        // a kernel that read world 0's material for both worlds would fail
        // here rather than merely being untested.
        WorldSetDesc set;
        set.worlds = {shower_instance(*world, 0xA11CE0F5E6DULL, 0.200000003f),
                      shower_instance(*world, 0x5EC0D5E0F5E6DULL, 0.800000012f)};
        return set;
    };

    ladder.setup = [](Simulation& sim) -> Result<void> {
        for (uint32_t w = 0; w < 2; ++w) {
            for (uint32_t i = 0; i < 100; ++i) {
                if (const Result<spade::BodyRef> ref = sim.spawn(w, shower_lattice_spawn(i)); !ref) {
                    return std::unexpected(ref.error());
                }
            }
        }
        return {};
    };

    // THE SHAPE THIS TEST MEANS TO EXERCISE, asserted rather than assumed --
    // the other half of the pair.
    {
        const Result<WorldSetDesc> desc = ladder.build();
        ASSERT_TRUE(desc.has_value()) << desc.error().context;
        const Result<spade::WorldSetLayout> layout = spade::validate_world_set(*desc);
        ASSERT_TRUE(layout.has_value()) << layout.error().context;
        ASSERT_FALSE(layout->uniform_dynamic_params)
            << "the two worlds' restitutions differ, so this set must take the PER-WORLD "
            << "CollisionDynamic shape -- if it is uniform, both shower tests cover the batched one";
    }

    using namespace spade::testing::bands::shower_ladder;
    run_parity(ladder,
               "shower_ladder (2 worlds x 100 bodies, differing restitution, 400 steps x 2 substeps, "
               "PER-WORLD grid)",
               join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands()),
               &expect_shower_resolved_pairs);
}

// ===========================================================================
// 11. THE SORT, IN ISOLATION (S6 Task 7) -- `GpuGridSort.*`.
//
// WHY THIS EXISTS WHEN THE SHOWER TESTS ALREADY PASS. The sorted key array is
// the one part of the ported pass whose correctness a tolerance band cannot
// see. The sweep is Gauss-Seidel, so the ORDER of the keys decides which pair
// is resolved against which velocities -- but a sort defect that merely
// permuted two entries of the same cell would move the answer by an amount
// indistinguishable from the div/sqrt rounding the bands are sized for. So the
// keys are read back and compared to std::sort's own output ELEMENT FOR
// ELEMENT, with the CPU's own comparator: an EQUALITY assertion, not a banded
// one. That is also what pins the TIE-BREAK rule, which is otherwise invisible:
// physics/grid.cpp:334's `return a.slot < b.slot`.
//
// ---------------------------------------------------------------------------
// HOW THE ADVERSARIAL KEY SETS ARE CONSTRUCTED, AND WHY THIS RUNS THE REAL
// RECORDED CHAIN RATHER THAN A SYNTHETIC PROBE DISPATCH.
//
// The brief's suggested instrument was probe_runner over a hand-built key
// buffer. What is done instead is stronger and needs no second binding layout:
// the keys are produced by the ACTUAL grid_build kernel from ACTUAL body
// positions, sorted by the ACTUAL recorded bitonic chain, inside an ACTUAL
// step of an ACTUAL vulkan Simulation -- and the positions are chosen so that
// the resulting key set is exactly the adversarial one each case names.
//
// THE KEYS ARE THE AUTHORED ONES, EXACTLY, and this is the property that makes
// the cases deterministic rather than approximately so. CollisionDynamic runs
// BEFORE Integrate within a substep (physics/schedule.cpp's kSchedule), and
// these worlds are built from ballistic.world.yaml, whose SDF is EMPTY -- so
// with one step of one substep, nothing has moved any body by the time
// grid_build reads `pos`: no static contact fires, no drag element exists, and
// Integrate has not run yet. The key array the readback returns is therefore
// build(spawn positions), bit for bit.
//
// The four cases, each named by what it does to the SORT rather than to the
// physics:
//   AlreadySorted   cells ascend with the slot index: the identity permutation
//   Reversed        cells descend with the slot index: the full reversal
//   AllEqualCells   every body in ONE cell, so the key differs only in `slot`
//                   -- the case that tests NOTHING BUT the tie-break
//   OneWorldEmpty   two worlds, the second with no active body at all
// ===========================================================================

namespace {

// The GpuGridSort fixture. A separate one from GpuParityTest purely so the
// SUITE is named `GpuGridSort` (this task's brief, verbatim) -- which also
// keeps the ctest "gpu" label, since tests/AppendSpadeLabels.cmake attaches it
// by the `^Gpu[^.]*\.` name-prefix rule. Same validation-message sink, for the
// same reason: a Vulkan validation message during a GPU test is a failure.
class GpuGridSort : public ::testing::Test {
protected:
    void SetUp() override { spade::compute::set_error_sink(&fail_on_validation_message); }
    void TearDown() override { spade::compute::set_error_sink(nullptr); }
};

// The sentinel this file reads the device array with is the CPU's own
// kInvalidWorld. compute/grid_entry.hpp restates that constant rather than
// including state/arenas.hpp (it is on layout_check.cpp's include path); this
// is where the restatement is pinned so it cannot drift.
static_assert(spade::compute::kGridInvalidWorld == spade::kInvalidWorld,
              "compute/grid_entry.hpp's sentinel must be state/arenas.hpp's kInvalidWorld");

// The CPU's own answer, built with the CPU's own functions: physics::
// grid_cell_of() for the key and physics::grid_entry_less() for the order.
// Nothing here restates either rule -- that is the point, since a test that
// re-transcribed the comparator would be checking its own transcription.
[[nodiscard]] std::vector<spade::physics::GridEntry> cpu_expected_entries(const Simulation& sim,
                                                                          float cell_size) {
    std::vector<spade::physics::GridEntry> entries;
    const uint32_t capacity = sim.layout().body_capacity;
    for (uint32_t w = 0; w < sim.layout().world_count; ++w) {
        const Result<std::span<const spade::BodyState>> bodies = sim.world_bodies(w);
        EXPECT_TRUE(bodies.has_value());
        if (!bodies) return {};
        for (uint32_t i = 0; i < bodies->size(); ++i) {
            const spade::BodyState& body = (*bodies)[i];
            if ((body.flags & spade::physics::body_flags::active) == 0u) continue;
            spade::physics::GridCell cell{};
            if (!spade::physics::grid_cell_of(body.pos, cell_size, cell)) continue;
            entries.push_back(spade::physics::GridEntry{w, cell, w * capacity + i});
        }
    }
    std::sort(entries.begin(), entries.end(), spade::physics::grid_entry_less);
    return entries;
}

// Compares the device's key array against `expected`. Three assertions, and
// the second and third are what a weaker check would skip:
//
//   1. the LIVE keys match std::sort's output element for element, INCLUDING
//      the `slot` tiebreak;
//   2. the live keys occupy a PREFIX of the array (every sentinel is after
//      every live key), so the sweep's [lo, hi) search cannot be reading a
//      sentinel as a body;
//   3. the array's LENGTH is the padded domain compute/grid_entry.hpp sizes,
//      so a readback that silently returned a shorter buffer could not pass
//      by having fewer keys to disagree about.
//
// `segments` is 1 for every case below, because all four use a set whose
// worlds share their contact/grid records -- i.e. the BATCHED shape, one
// segment over the whole slot space. It is a parameter rather than a constant
// so a future per-world-shape case has somewhere to say so.
void expect_sorted_keys_match(const std::vector<spade::compute::GridEntryRow>& device,
                              const std::vector<spade::physics::GridEntry>& expected, uint32_t segments,
                              std::size_t expected_domain, std::string_view label) {
    EXPECT_EQ(device.size(), expected_domain)
        << label << ": the device key array is " << device.size() << " entries, the padded domain is "
        << expected_domain;
    ASSERT_EQ(segments, 1u) << label << ": this helper's prefix check assumes a single segment";

    std::vector<spade::compute::GridEntryRow> live;
    bool sentinel_seen = false;
    bool live_after_sentinel = false;
    for (const spade::compute::GridEntryRow& row : device) {
        if (row.world == spade::compute::kGridInvalidWorld) {
            sentinel_seen = true;
        } else {
            if (sentinel_seen) live_after_sentinel = true;
            live.push_back(row);
        }
    }
    EXPECT_FALSE(live_after_sentinel)
        << label << ": a LIVE key follows a sentinel -- the sentinel (world 0xFFFFFFFF) must sort "
        << "after every real key, and the sweep's [lo, hi) search relies on exactly that";

    ASSERT_EQ(live.size(), expected.size())
        << label << ": the device produced " << live.size() << " live keys, the CPU build produced "
        << expected.size();

    for (std::size_t i = 0; i < live.size(); ++i) {
        EXPECT_EQ(live[i].world, expected[i].world) << label << ": key " << i << " world";
        EXPECT_EQ(live[i].cell_x, expected[i].cell.x) << label << ": key " << i << " cell.x";
        EXPECT_EQ(live[i].cell_y, expected[i].cell.y) << label << ": key " << i << " cell.y";
        EXPECT_EQ(live[i].cell_z, expected[i].cell.z) << label << ": key " << i << " cell.z";
        EXPECT_EQ(live[i].slot, expected[i].slot)
            << label << ": key " << i << " slot -- this is the TIE-BREAK (physics/grid.cpp:334); two "
            << "entries of the same cell in the wrong order change the Gauss-Seidel sweep order and "
            << "therefore the answer";
    }
}

// Steps a one-substep vulkan Simulation exactly once and returns the device
// key array. One step is all it takes (see this section's header for why the
// keys are then exactly build(spawn positions)), and it is what makes these
// cases cheap enough to run four of.
//
// A TIMING SUBTLETY EVERY CALLER OF THIS FUNCTION SHOULD KNOW (S6 hygiene:
// T7 review M2, stated rather than left implicit). `cpu_expected_entries()`
// below is called AFTER `sim.step(1)` returns, i.e. AFTER Integrate has
// already moved every body -- so it reads POST-step positions. The device
// keys THIS function returns reflect PRE-Integrate positions instead:
// CollisionDynamic (grid_build) runs BEFORE Integrate within the same
// substep (physics/schedule.cpp's kSchedule), so by the time step(1) returns
// the device's key array was already built from the positions bodies had at
// the START of this step. Every test in this section spawns bodies AT REST
// (BodySpawn{} defaults vel to zero), so the only motion between the two
// reads is one substep of pure gravity from rest: displacement =
// 0.5*g*dt^2 ~= 0.5 * 9.80665 * (1e-3)^2 ~= 4.9e-6 m, against this section's
// kSortCellSize of 1.0 m -- about FIVE ORDERS OF MAGNITUDE below the cell
// size, and every spawned position here sits comfortably clear of its own
// cell's boundary (e.g. the "already-sorted" case's `y: 10.5f`, mid-cell).
// grid_cell_of()'s floor() is therefore provably unaffected by which of the
// two reads a test compares against: this mismatch is real, timing-wise, and
// harmless, numerically, for every case this file constructs.
[[nodiscard]] Result<std::vector<spade::compute::GridEntryRow>> stepped_grid_entries(Simulation& sim) {
    if (Result<void> stepped = sim.step(1); !stepped) return std::unexpected(stepped.error());
    return sim.vulkan_grid_entries();
}

// A world set of `count` void worlds (ballistic.world.yaml: no geometry at
// all), cell size 1 m, proxy radius 0.1 m. The EMPTY SDF is what keeps
// CollisionStatic from moving anything before grid_build reads the positions,
// which is what makes the key set exactly the authored one.
[[nodiscard]] Result<WorldSetDesc> sort_test_world_set(uint32_t count, uint64_t seed) {
    WorldInstanceDesc prototype;
    prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    prototype.contacts.proxy_radius = 0.1f;
    prototype.grid.cell_size = 1.0f;
    return spade::world_set_from(WorldRef{golden_dir() / "worlds" / "ballistic.world.yaml"}, count, seed,
                                 prototype);
}

constexpr uint32_t kSortBodies = 4;      // ballistic.world.yaml files bodies: 4
constexpr float kSortCellSize = 1.0f;    // the prototype above
constexpr std::size_t kSortDomain1 = 4;  // next_pow2(1 world * 4 slots), the batched shape
constexpr std::size_t kSortDomain2 = 8;  // next_pow2(2 worlds * 4 slots)

}  // namespace

TEST_F(GpuGridSort, AlreadySortedKeySetMatchesTheCpu) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = sort_test_world_set(1, 0x507E0ULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    // Cells ascend with the slot index along x, so the key order IS the slot
    // order and the network's correct answer is the identity permutation --
    // the input a broken network is most likely to get right by accident, and
    // therefore the control rather than the interesting case.
    for (uint32_t i = 0; i < kSortBodies; ++i) {
        BodySpawn b{};
        b.pos = {0.5f + static_cast<float>(i), 10.5f, 0.5f};
        ASSERT_TRUE(sim->spawn(0, b).has_value());
    }

    const Result<std::vector<spade::compute::GridEntryRow>> device = stepped_grid_entries(*sim);
    ASSERT_TRUE(device.has_value()) << device.error().context;
    expect_sorted_keys_match(*device, cpu_expected_entries(*sim, kSortCellSize), 1u, kSortDomain1,
                             "already-sorted");
}

TEST_F(GpuGridSort, ReversedKeySetMatchesTheCpu) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = sort_test_world_set(1, 0x5EE5EDULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    // Cells DESCEND with the slot index: the correct answer is the full
    // reversal, which is the input that exercises every compare-exchange of
    // every stage of the network.
    for (uint32_t i = 0; i < kSortBodies; ++i) {
        BodySpawn b{};
        b.pos = {0.5f - static_cast<float>(i), 10.5f, 0.5f};
        ASSERT_TRUE(sim->spawn(0, b).has_value());
    }

    const Result<std::vector<spade::compute::GridEntryRow>> device = stepped_grid_entries(*sim);
    ASSERT_TRUE(device.has_value()) << device.error().context;
    expect_sorted_keys_match(*device, cpu_expected_entries(*sim, kSortCellSize), 1u, kSortDomain1,
                             "reversed");
}

TEST_F(GpuGridSort, AllEqualCellsFallsBackToTheSlotTiebreak) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = sort_test_world_set(1, 0xE00A11ULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    // EVERY body in the SAME cell (cell size 1 m, all four inside [10, 11) on
    // every axis), so all four keys agree on world and on all three cell
    // integers and the order is decided ENTIRELY by grid.cpp:334's
    // `a.slot < b.slot`. Spawned in ASCENDING slot order but at DESCENDING x
    // within the cell, so a network that (wrongly) ordered by position rather
    // than by the key would produce the reverse of the right answer instead of
    // accidentally agreeing with it.
    for (uint32_t i = 0; i < kSortBodies; ++i) {
        BodySpawn b{};
        b.pos = {10.9f - 0.1f * static_cast<float>(i), 10.5f, 10.5f};
        ASSERT_TRUE(sim->spawn(0, b).has_value());
    }

    const Result<std::vector<spade::compute::GridEntryRow>> device = stepped_grid_entries(*sim);
    ASSERT_TRUE(device.has_value()) << device.error().context;
    const std::vector<spade::physics::GridEntry> expected = cpu_expected_entries(*sim, kSortCellSize);

    // The premise of this case, asserted rather than assumed: if the four
    // bodies did NOT land in one cell the test would silently degrade into a
    // second copy of the already-sorted case.
    ASSERT_EQ(expected.size(), std::size_t{kSortBodies});
    for (const spade::physics::GridEntry& e : expected) {
        EXPECT_EQ(e.cell, expected.front().cell) << "the all-equal-cells premise does not hold";
    }
    // And the tiebreak is genuinely load-bearing here: the expected order must
    // be the SLOT order, ascending, because nothing else distinguishes these
    // four keys.
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(expected[i].slot, static_cast<uint32_t>(i));
    }

    expect_sorted_keys_match(*device, expected, 1u, kSortDomain1, "all-equal-cells");
}

TEST_F(GpuGridSort, OneWorldEmptyMatchesTheCpu) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = sort_test_world_set(2, 0xE377E0ULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    // World 0 populated, world 1 EMPTY. Empty worlds are legal (this program's
    // own constraint says so), and this is the case where a segment holds
    // nothing but sentinels -- which is also where an off-by-one in the sweep's
    // [lo, hi) search would read a sentinel as a body.
    for (uint32_t i = 0; i < kSortBodies; ++i) {
        BodySpawn b{};
        b.pos = {0.5f - static_cast<float>(i), 10.5f, 0.5f};
        ASSERT_TRUE(sim->spawn(0, b).has_value());
    }

    const Result<std::vector<spade::compute::GridEntryRow>> device = stepped_grid_entries(*sim);
    ASSERT_TRUE(device.has_value()) << device.error().context;
    const std::vector<spade::physics::GridEntry> expected = cpu_expected_entries(*sim, kSortCellSize);
    ASSERT_EQ(expected.size(), std::size_t{kSortBodies}) << "world 1 must contribute no keys";
    for (const spade::physics::GridEntry& e : expected) {
        EXPECT_EQ(e.world, 0u) << "world 1 is empty; no key may carry its id";
    }
    expect_sorted_keys_match(*device, expected, 1u, kSortDomain2, "one-world-empty");
}

// ===========================================================================
// 12. GPU-vs-GPU DETERMINISM (S6 Task 8) -- `state_digest` over the READ-BACK
//     arenas, bit-identical across two runs of the same scenario on the same
//     device and driver.
//
// A DIFFERENT CLAIM FROM EVERY OTHER TEST IN THIS FILE, and the one the banded
// comparisons structurally cannot make. CPU<->GPU is banded because Vulkan's
// division and square root are <= 2.5 ulp rather than correctly rounded. That
// allowance says nothing about REPRODUCIBILITY: the same device running the
// same kernels over the same inputs must produce the same bits every time, or
// the parity bands themselves are measuring noise and the whole calibration is
// meaningless. So this leg is an EQUALITY assertion, not a banded one.
//
// WHAT WOULD BREAK IT, which is why it is worth running rather than assuming: a
// race between two dispatches with a missing barrier, a kernel whose result
// depends on which lane reached a shared location first, a reduction whose
// order varies with scheduling, or an uninitialized read that happens to see
// different garbage. Every one of those is a real hazard this port designed
// against -- per-item writes, fixed-order serial folds, no groupshared anything
// -- and none of them is visible in a comparison against the CPU, because a
// single GPU run of a racy kernel still lands somewhere inside a band.
//
// THE DIGEST MACHINERY IS USED UNCHANGED, ON READ-BACK ARENAS, AND THAT IS
// ASSERTED RATHER THAN ASSUMED. engine/testing/replay.hpp's state_digest() is an
// FNV-1a fold over the registry walk -- it reads `arenas_`, and after a vulkan
// step() `arenas_` holds exactly what the device produced (sim/simulation.cpp
// reads back into it before returning). So the same function that pins the CPU
// determinism corpus applies here with no adaptation at all. The two EXPECTs
// before the equality say so directly: a NON-ZERO digest (a zero would mean the
// walk found nothing to fold) that MOVED across the run (proving the run
// changed state at all).
//
// TWO SCENARIOS, chosen so the two hardest dispatch shapes are both covered:
// quad_hover is the rotors/IMU/gust one (three kernels writing three arrays,
// plus the per-sensor and per-world grids), and shower is the 100-body
// Gauss-Seidel pile -- the scenario ParityChaos proves is CHAOTIC, so if
// anything in this engine were run-to-run unstable, amplification would make it
// visible there first.
// ===========================================================================

namespace {

// One complete GPU run of a scenario: the whole-set digest before any step and
// after the last one.
struct GpuRunDigests {
    uint64_t before = 0;
    uint64_t after = 0;
};

[[nodiscard]] Result<GpuRunDigests> gpu_run_digests(const Scenario& scenario) {
    Result<Simulation> sim =
        spade::testing::start_scenario(scenario, BackendDesc{.kind = BackendKind::vulkan});
    if (!sim) return std::unexpected(sim.error());
    GpuRunDigests out;
    out.before = spade::testing::state_digest(*sim);
    if (Result<void> advanced = spade::testing::advance_scenario(scenario, *sim, scenario.steps);
        !advanced) {
        return std::unexpected(advanced.error());
    }
    out.after = spade::testing::state_digest(*sim);
    return out;
}

void expect_gpu_run_to_run_determinism(const Scenario& scenario, std::string_view label) {
    const Result<GpuRunDigests> first = gpu_run_digests(scenario);
    ASSERT_TRUE(first.has_value()) << label << " (run 1): " << first.error().context;
    const Result<GpuRunDigests> second = gpu_run_digests(scenario);
    ASSERT_TRUE(second.has_value()) << label << " (run 2): " << second.error().context;

    std::printf("\n=== GPU determinism: %.*s ===\n", static_cast<int>(label.size()), label.data());
    std::printf("    run 1  before=0x%016llx  after=0x%016llx\n",
                static_cast<unsigned long long>(first->before),
                static_cast<unsigned long long>(first->after));
    std::printf("    run 2  before=0x%016llx  after=0x%016llx\n",
                static_cast<unsigned long long>(second->before),
                static_cast<unsigned long long>(second->after));

    // THE DIGEST MACHINERY WORKS ON READ-BACK ARENAS -- the explicit assertion
    // this task's brief asks for, made in the only way that can fail
    // informatively: a non-zero fold that MOVED across the run.
    EXPECT_NE(first->after, 0u)
        << label << ": state_digest() over the read-back arenas folded to zero -- the registry walk "
        << "found nothing, so the equality below would be vacuous";
    EXPECT_NE(first->after, first->before)
        << label << ": the digest did not move across the run, so a frozen device would satisfy the "
        << "run-to-run equality below without stepping anything";

    // The claim itself.
    EXPECT_EQ(first->after, second->after)
        << label << ": two runs of the same scenario on the same device produced DIFFERENT state. "
        << "The CPU<->GPU bands elsewhere in this file are calibrated against a device assumed to be "
        << "reproducible; if it is not, they are measuring noise. Look for a missing barrier, a "
        << "lane-order-dependent write, or an uninitialized read.";
    EXPECT_EQ(first->before, second->before)
        << label << ": the two runs did not even START from the same state";
}

}  // namespace

TEST_F(GpuParityTest, QuadHoverIsBitIdenticalAcrossTwoGpuRuns) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<spade::testing::LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    expect_gpu_run_to_run_determinism(loaded->scenario, "quad_hover (rotors + IMU + gusts)");
}

TEST_F(GpuParityTest, ShowerIsBitIdenticalAcrossTwoGpuRuns) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<spade::testing::LoadedScenario> loaded = load_scenario("shower");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    expect_gpu_run_to_run_determinism(loaded->scenario, "shower (100-body Gauss-Seidel pile)");
}

// ===========================================================================
// 13. THE RING READBACK LANDS WHERE ring_poll() LOOKS (S6 Task 8).
//
// WHAT THIS ADDS OVER THE `imu_ring` BAND ROWS, which is a real distinction and
// not belt-and-braces. Those rows compare the ring ARRAY slot by slot: they
// prove the device wrote the right samples into the right slots. They say
// nothing about whether a CONSUMER can read them, because a consumer does not
// index the array -- it calls poll_imu(), which computes a window from the
// sensor's `last_index`, resolves the wrap, reports drops and hands back a span
// in index order (sensors/rings.hpp's TA5 convention).
//
// So this test polls BOTH backends' sensors through that real path and requires
// the SAME COUNT, the SAME INDICES, the SAME TICK STAMPS and the same drop
// report -- exactly, integers, no band -- with the sample VALUES inside the
// bands the array comparison pinned. A GPU ring written one slot out of phase,
// or a `last_index` advancing at the wrong rate, would satisfy the value bands
// and fail here on the stamps.
//
// THE RING IS IN ITS MOST DEMANDING STATE: quad_hover writes 1800 samples per
// sensor into a 64-deep ring, so it has wrapped 28 times and `since_index` 0
// resolves to the resident TAIL rather than to everything ever produced -- the
// case that actually exercises ring_oldest_resident() and the wrap arithmetic.
// ===========================================================================

TEST_F(GpuParityTest, ImuRingPollAfterGpuStepsMatchesTheCpu) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<spade::testing::LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    const Scenario& scenario = loaded->scenario;

    Result<Simulation> cpu = spade::testing::start_scenario(scenario, BackendDesc{.kind = BackendKind::cpu});
    ASSERT_TRUE(cpu.has_value()) << cpu.error().context;
    Result<Simulation> gpu =
        spade::testing::start_scenario(scenario, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;

    ASSERT_TRUE(spade::testing::advance_scenario(scenario, *cpu, scenario.steps).has_value());
    ASSERT_TRUE(spade::testing::advance_scenario(scenario, *gpu, scenario.steps).has_value());

    using namespace spade::testing::bands::quad_hover;

    std::size_t sensors_polled = 0;
    for (uint32_t world = 0; world < cpu->layout().world_count; ++world) {
        const Result<spade::VehicleRef> cpu_vehicle = cpu->vehicle_ref_at(world, 0);
        ASSERT_TRUE(cpu_vehicle.has_value()) << cpu_vehicle.error().context;
        const Result<spade::VehicleRef> gpu_vehicle = gpu->vehicle_ref_at(world, 0);
        ASSERT_TRUE(gpu_vehicle.has_value()) << gpu_vehicle.error().context;
        ASSERT_EQ(cpu_vehicle->imu_count, gpu_vehicle->imu_count);
        ASSERT_GT(cpu_vehicle->imu_count, 0u) << "quad_hover's model declares one IMU mount per vehicle";

        for (uint32_t m = 0; m < cpu_vehicle->imu_count; ++m) {
            // `since_index` 0 means "everything you have" (rings.hpp's TA5
            // convention), which for a ring that has wrapped is the resident
            // tail -- the case that exercises the wrap arithmetic.
            std::array<spade::sensors::ImuSample, spade::sensors::kRingDepth> cpu_out{};
            std::array<spade::sensors::ImuSample, spade::sensors::kRingDepth> gpu_out{};
            const Result<spade::ImuPoll> cpu_poll = cpu->poll_imu(
                cpu_vehicle->imu_sensors[m], 0, std::span<spade::sensors::ImuSample>(cpu_out));
            ASSERT_TRUE(cpu_poll.has_value()) << cpu_poll.error().context;
            const Result<spade::ImuPoll> gpu_poll = gpu->poll_imu(
                gpu_vehicle->imu_sensors[m], 0, std::span<spade::sensors::ImuSample>(gpu_out));
            ASSERT_TRUE(gpu_poll.has_value()) << gpu_poll.error().context;

            // NON-VACUITY FIRST: a poll that returned nothing would make every
            // comparison below trivially true.
            ASSERT_GT(cpu_poll->samples.size(), std::size_t{0})
                << "world " << world << " sensor " << m << ": the CPU leg produced no samples, so this "
                << "comparison proves nothing about the GPU's ring";

            EXPECT_EQ(gpu_poll->samples.size(), cpu_poll->samples.size())
                << "world " << world << " sensor " << m << ": the two backends' polls returned "
                << "different sample COUNTS -- the device's sensor emitted on different substeps";
            EXPECT_EQ(gpu_poll->next_since, cpu_poll->next_since)
                << "world " << world << " sensor " << m << ": different write cursors";
            EXPECT_EQ(gpu_poll->dropped, cpu_poll->dropped)
                << "world " << world << " sensor " << m << ": different drop counts";

            const std::size_t n = std::min(cpu_poll->samples.size(), gpu_poll->samples.size());
            float worst_accel = 0.0f;
            float worst_gyro = 0.0f;
            for (std::size_t i = 0; i < n; ++i) {
                const spade::sensors::ImuSample& a = cpu_poll->samples[i];
                const spade::sensors::ImuSample& b = gpu_poll->samples[i];
                // EXACT, both of them: an index or a tick is an integer, and a
                // sample compared at the wrong instant is not a comparison.
                ASSERT_EQ(b.index, a.index) << "world " << world << " sensor " << m << " sample " << i
                                            << ": the polls are not aligned on the same sample index";
                EXPECT_EQ(b.tick, a.tick) << "world " << world << " sensor " << m << " sample " << i
                                          << ": same index, different TICK STAMP";
                for (int c = 0; c < 3; ++c) {
                    worst_accel = std::max(worst_accel, std::fabs(a.accel[c] - b.accel[c]));
                    worst_gyro = std::max(worst_gyro, std::fabs(a.gyro[c] - b.gyro[c]));
                }
            }

            std::printf("    ring poll world %u sensor %u: n=%zu dropped=%llu worst|accel|=%.9e "
                        "worst|gyro|=%.9e\n",
                        world, m, n, static_cast<unsigned long long>(cpu_poll->dropped),
                        static_cast<double>(worst_accel), static_cast<double>(worst_gyro));

            EXPECT_LE(worst_accel, kAccel.abs)
                << "world " << world << " sensor " << m << ": polled accel outside the band the "
                << "`imu_ring` array comparison pinned";
            EXPECT_LE(worst_gyro, kGyro.abs)
                << "world " << world << " sensor " << m << ": polled gyro outside the band the "
                << "`imu_ring` array comparison pinned";
            ++sensors_polled;
        }
    }
    EXPECT_EQ(sensors_polled, std::size_t{2}) << "quad_hover has two worlds, one IMU each";
}
