// ===========================================================================
// test_gpu_invariance.cpp (S6 Task 9) -- THE INVARIANCE BATTERY.
//
// Every earlier GPU task proved the ported kernels agree with the CPU twin
// (test_gpu_parity.cpp, banded) and agree with THEMSELVES run to run
// (QuadHoverIsBitIdenticalAcrossTwoGpuRuns / ShowerIsBitIdenticalAcrossTwoGpuRuns,
// bit-exact). This task proves the four invariances the ARCHITECTURE promises
// but nothing yet exercises:
//
//   1. A7 KNOB INVARIANCE       -- the same device, the workgroup_size knob
//                                  varied, must produce bit-identical state.
//   2. GPU BATCHING INVARIANCE  -- a world's trajectory must not depend on
//                                  what else shares its world set, on the GPU
//                                  (the CPU claim is ParityGeometry.
//                                  HeterogeneousGeometrySetMatchesSoloRuns;
//                                  nothing proves it device-side).
//   3. BACKEND-SEAM DIGEST RE-PROOF -- the cpu path's five committed corpus
//                                  digests are unchanged AT THIS TASK'S TIP
//                                  (A7's cpu leg, rerun rather than assumed).
//   4. RESTORE-CROSS-BACKEND CONTRACT -- a snapshot taken on one backend
//                                  restores into a Simulation running the
//                                  OTHER backend, in both directions, and
//                                  continuation stays within the pinned
//                                  parity bands.
//
// PLUS the DISPATCH-MUST-CARRY addendum (T7 review finding I1, coordinator-
// ruled into this task): GpuGridSort (test_gpu_parity.cpp) only ever sorts an
// 8-stage-or-shallower network. A workgroup-size sweep over a network that
// shallow is not discriminating -- so this file adds a 128-key, ONE-SEGMENT,
// 28-stage case and a genuinely SEGMENTED (per-world) case, both verified
// element-for-element against std::sort + physics::grid_entry_less exactly as
// GpuGridSort's four cases are.
//
// ===========================================================================
// `BackendDesc::workgroup_size` IS LIVE, AND THIS FILE'S SWEEP DISCRIMINATES
// (S6 Task 9b).
//
// WHAT THIS PARAGRAPH REPLACES, because the history is the point. Through Task
// 9 this banner carried a HEADLINE FINDING: the knob reached nothing. All nine
// dispatched kernels baked a literal `[numthreads(64,1,1)]` into their SPIR-V
// at slangc time, cmake/SpadeSlang.cmake compiled each kernel exactly once, and
// step_recorder.cpp divided every dispatch grid by a file-scope
// `constexpr uint32_t kWorkgroupSize = 64`. The three legs of the sweep below
// therefore built three IDENTICAL pipelines and compared them to each other --
// plumbing-proof, not physics-proof, and this file said so rather than passing
// it off as satisfying A7.
//
// THE COORDINATOR RULED THAT CLOSED, and Task 9b closed it. The nine kernels
// now spell `[numthreads(SPADE_WG,1,1)]`; cmake/SpadeSlang.cmake's
// spade_slang_kernel_variants() compiles each of them once per size in
// compute/backend.hpp's kSupportedWorkgroupSizes {32, 64, 128};
// VulkanBackend::create() rejects anything outside that set BEFORE it touches a
// device; and StepRecorder::create() selects the matching compiled variant for
// every pipeline AND divides all four dispatch grids by the SAME value it
// selected with (a mismatched pair is a Vulkan valid-usage violation whose
// symptom is wrong physics rather than an error, which is why one field feeds
// both halves).
//
// SO WHAT THE SWEEP NOW PROVES, stated as precisely as its predecessor stated
// its own limits: three fresh Simulations run the corpus through kernels whose
// LOCAL SIZE GENUINELY DIFFERS -- 32, 64 and 128 threads per workgroup, three
// separately-compiled SPIR-V modules, three different dispatch grids over the
// same extents -- and their read-back state_digest()s must be bit-identical.
// That is A7's actual claim, and it is what the global constraints' "no
// workgroup-shared reduction whose result order depends on local size" kernel
// design rule exists to make true. A failure here is a reduction-order bug in a
// kernel, to be fixed, NEVER banded.
//
// AND IT IS PROVEN DISCRIMINATING RATHER THAN ASSUMED SO. Task 9b's mutation
// kill put a workgroup-shared partial sum, folded in local-index order, into
// one kernel's parity path; rebuilt; and watched this sweep FAIL across all
// three sizes. The mutation was then reverted and the sweep went green again.
// Both transcripts are in that task's report. A sweep whose failure mode has
// never been observed is a sweep nobody should trust.
//
// device_index, the OTHER A7 knob, IS live (VulkanContext::create() reads it
// to select a physical device) but is not independently invariance-tested
// here: this box enumerates exactly one Vulkan device, so device_index=1 is
// not a second value to compare against, only an out-of-range rejection --
// a different claim from A7's "same device, knob varied, identical result".
//
// ===========================================================================
// SUITE NAMING (tests/AppendSpadeLabels.cmake's `^Gpu[^.]*\.` rule, this
// file's own convention exactly as test_gpu_parity.cpp's section 0 states
// it): `GpuInvarianceTest` and `GpuGridSortDeepTest` are device-executing
// fixtures and carry the ctest "gpu" label (skip-not-fail, TIMEOUT 180)
// automatically by name prefix. `BackendSeamReproof` is host-only -- it never
// touches a device -- and is deliberately NOT Gpu*-prefixed, matching
// test_gpu_state_mirror.cpp's BackendKnobInvariance suite one file over.
// ===========================================================================

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "compute/backend.hpp"
#include "compute/grid_entry.hpp"
#include "compute/vulkan/context.hpp"
#include "core/rng.hpp"
#include "physics/grid.hpp"
#include "physics/integrator.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/layout.hpp"
#include "testing/parity.hpp"
#include "testing/replay.hpp"
#include "testing/scenario_file.hpp"
#include "world/medium.hpp"
#include "world/world_ref.hpp"

namespace {

using spade::BodyRef;
using spade::Code;
using spade::BodySpawn;
using spade::Capacities;
using spade::Result;
using spade::Simulation;
using spade::WorldDesc;
using spade::WorldInstanceDesc;
using spade::WorldRef;
using spade::WorldSetDesc;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::vulkan_available;
using spade::dryden_params;
using spade::resolve_world;
using spade::TurbulenceLevel;
using spade::testing::BandEntry;
using spade::testing::compare_arrays;
using spade::testing::LoadedScenario;
using spade::testing::ParityReport;
using spade::testing::QuantityKind;
using spade::testing::Scenario;
using spade::testing::ToleranceBand;

// Same sink, same reason, as every other GPU test file in this tree: a
// Vulkan validation message during a GPU test is a FAILURE, not a warning.
void fail_on_validation_message(std::string_view message) noexcept {
    ADD_FAILURE() << "Vulkan validation message during a GpuInvariance test: " << message;
}

class GpuInvarianceTest : public ::testing::Test {
protected:
    void SetUp() override { spade::compute::set_error_sink(&fail_on_validation_message); }
    void TearDown() override { spade::compute::set_error_sink(nullptr); }
};

[[nodiscard]] std::filesystem::path golden_dir() { return std::filesystem::path(SPADE_GOLDEN_DIR); }

[[nodiscard]] Result<LoadedScenario> load_scenario(std::string_view name) {
    return spade::testing::scenario_from_yaml(golden_dir() / "scenarios" / (std::string(name) + ".scenario.yaml"));
}

// ---------------------------------------------------------------------------
// The two band tables this file's cross-backend restore battery needs.
// DUPLICATED from test_gpu_parity.cpp rather than shared, deliberately: every
// GPU test TU in this tree is self-contained (test_gpu_state_mirror.cpp
// re-derives its own `corpus()` rather than reaching into test_gpu_parity.cpp
// for kCorpusScenarios, for the identical reason) -- these are ~30 lines of
// offsetof() table, not logic, so the duplication carries no drift risk a
// shared header would meaningfully reduce, and it keeps this file readable
// standalone.
// ---------------------------------------------------------------------------
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
        {"dryden", "filter_state", offsetof(spade::DrydenState, u), 5, QuantityKind::components, filter},
    };
}

[[nodiscard]] std::vector<BandEntry> join(std::vector<BandEntry> a, const std::vector<BandEntry>& b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

}  // namespace

// ===========================================================================
// BATTERY 1 -- A7 WORKGROUP_SIZE KNOB INVARIANCE.
//
// Three FRESH Simulations (fresh WorldSetDesc build, fresh Simulation::create,
// fresh backend -- never a knob flipped on a live object), one per
// workgroup_size in {32, 64, 128}, over the full corpus PLUS T6b's
// heterogeneous_geometry_set (coordinator addition: heterogeneous geometry
// proven knob-invariant too, not just uniform SDF-per-world scenarios).
//
// THE PREDICATE: state_digest() over the READ-BACK arenas, bit-identical
// across all three sizes -- the same instrument and the same non-vacuity
// posture as QuadHoverIsBitIdenticalAcrossTwoGpuRuns /
// ShowerIsBitIdenticalAcrossTwoGpuRuns (test_gpu_parity.cpp), generalized from
// two runs to three. See this file's header for what this predicate can and
// cannot currently prove.
// ===========================================================================

namespace {

constexpr std::array<uint32_t, 3> kWorkgroupSizes = {32u, 64u, 128u};

// One scenario, stepped once per workgroup_size, with the digest table
// printed unconditionally (passing or failing -- this program's standing
// "the measured table is the deliverable" posture).
void expect_workgroup_size_invariance(const Scenario& scenario, std::string_view label) {
    std::array<uint64_t, kWorkgroupSizes.size()> before{};
    std::array<uint64_t, kWorkgroupSizes.size()> after{};
    std::array<uint64_t, kWorkgroupSizes.size()> ticks{};

    for (std::size_t i = 0; i < kWorkgroupSizes.size(); ++i) {
        const BackendDesc backend{.kind = BackendKind::vulkan, .workgroup_size = kWorkgroupSizes[i]};
        Result<Simulation> sim = spade::testing::start_scenario(scenario, backend);
        ASSERT_TRUE(sim.has_value())
            << label << " (workgroup_size=" << kWorkgroupSizes[i] << "): " << sim.error().context;
        before[i] = spade::testing::state_digest(*sim);
        ASSERT_TRUE(spade::testing::advance_scenario(scenario, *sim, scenario.steps).has_value())
            << label << " (workgroup_size=" << kWorkgroupSizes[i] << ")";
        after[i] = spade::testing::state_digest(*sim);
        ticks[i] = sim->tick().value;
    }

    std::printf("\n=== A7 workgroup_size invariance: %.*s ===\n", static_cast<int>(label.size()), label.data());
    for (std::size_t i = 0; i < kWorkgroupSizes.size(); ++i) {
        std::printf("    workgroup_size=%-4u  tick=%-6llu  before=0x%016llx  after=0x%016llx\n",
                    kWorkgroupSizes[i], static_cast<unsigned long long>(ticks[i]),
                    static_cast<unsigned long long>(before[i]), static_cast<unsigned long long>(after[i]));
    }

    // VACUITY DEFENSE 1: every leg actually stepped to completion.
    for (std::size_t i = 0; i < kWorkgroupSizes.size(); ++i) {
        EXPECT_EQ(ticks[i], scenario.steps)
            << label << " (workgroup_size=" << kWorkgroupSizes[i] << "): did not reach the scenario's own "
            << "step count";
    }
    // VACUITY DEFENSE 2: the digest folded to something and it MOVED -- a
    // frozen or empty run would satisfy bit-identity trivially.
    for (std::size_t i = 0; i < kWorkgroupSizes.size(); ++i) {
        EXPECT_NE(after[i], 0u) << label << " (workgroup_size=" << kWorkgroupSizes[i] << "): state_digest() "
                                << "folded to zero -- the registry walk found nothing";
        EXPECT_NE(after[i], before[i])
            << label << " (workgroup_size=" << kWorkgroupSizes[i] << "): the digest did not move across "
            << "the run -- a frozen device would satisfy bit-identity below vacuously";
    }
    // Construction does not depend on the knob: the three legs must have
    // started from IDENTICAL state, or a divergence in `after` would not be
    // isolable to the step loop.
    EXPECT_EQ(before[0], before[1]) << label << ": workgroup_size=32 and 64 did not even START identically";
    EXPECT_EQ(before[1], before[2]) << label << ": workgroup_size=64 and 128 did not even START identically";

    // THE CLAIM.
    EXPECT_EQ(after[0], after[1])
        << label << ": workgroup_size=32 vs 64 diverged after " << scenario.steps << " steps -- see this "
        << "file's header for what that would mean (a reduction-order bug, never to be banded)";
    EXPECT_EQ(after[1], after[2])
        << label << ": workgroup_size=64 vs 128 diverged after " << scenario.steps << " steps";
}

}  // namespace

TEST_F(GpuInvarianceTest, BallisticBitIdenticalAcrossWorkgroupSizes) {
    if (!vulkan_available()) GTEST_SKIP();
    const Result<LoadedScenario> loaded = load_scenario("ballistic");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    expect_workgroup_size_invariance(loaded->scenario, "ballistic");
}

TEST_F(GpuInvarianceTest, BounceBitIdenticalAcrossWorkgroupSizes) {
    if (!vulkan_available()) GTEST_SKIP();
    const Result<LoadedScenario> loaded = load_scenario("bounce");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    expect_workgroup_size_invariance(loaded->scenario, "bounce");
}

TEST_F(GpuInvarianceTest, QuadHoverBitIdenticalAcrossWorkgroupSizes) {
    if (!vulkan_available()) GTEST_SKIP();
    const Result<LoadedScenario> loaded = load_scenario("quad_hover");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    expect_workgroup_size_invariance(loaded->scenario, "quad_hover");
}

TEST_F(GpuInvarianceTest, ShowerBitIdenticalAcrossWorkgroupSizes) {
    if (!vulkan_available()) GTEST_SKIP();
    const Result<LoadedScenario> loaded = load_scenario("shower");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    expect_workgroup_size_invariance(loaded->scenario, "shower");
}

// PHY-6: the corpus's GNSS scenario, so sensor_gnss.slang over live rows is
// held to the same workgroup-size bit-identity as every other kernel.
TEST_F(GpuInvarianceTest, GnssTumbleBitIdenticalAcrossWorkgroupSizes) {
    if (!vulkan_available()) GTEST_SKIP();
    const Result<LoadedScenario> loaded = load_scenario("gnss_tumble");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    expect_workgroup_size_invariance(loaded->scenario, "gnss_tumble");
}

// ===========================================================================
// THE KNOB'S ALLOWED SET, AND WHAT HAPPENS OUTSIDE IT (S6 Task 9b).
//
// A knob that is real has values it REFUSES; an "unsupported" size that quietly
// ran at 64 anyway would put the sweep above back where Task 9 found it, only
// less visibly. Two claims, and note that NEITHER needs a device:
//
//   1. Every size compute/backend.hpp admits was actually compiled. The two
//      lists (kSupportedWorkgroupSizes and cmake/SpadeSlang.cmake's
//      SPADE_SLANG_WORKGROUP_SIZES) are separate files in separate languages,
//      so their agreement is a real thing to assert -- and this file's own
//      kWorkgroupSizes sweep array is a THIRD statement of it, pinned here too.
//   2. A size outside the set is a NAMED Code::invalid_argument that says what
//      the legal values are.
//
// HOST-ONLY, DELIBERATELY, and the reason is a design decision rather than a
// convenience: VulkanBackend::create() validates workgroup_size BEFORE it
// builds a context, so the answer does not depend on whether a device is
// present. Diagnosing a bad argument as "no Vulkan device" on every CI runner
// would be the wrong answer to the caller's actual mistake. Hence no Gpu*
// prefix and no skip guard -- matching BackendSeamReproof one section down.
// ===========================================================================

TEST(WorkgroupSizeContract, AllowedSetMatchesCompiledVariants) {
    // The sweep array above must BE the allowed set -- not a subset of it, or
    // this file would silently stop testing a size the backend still accepts.
    ASSERT_EQ(kWorkgroupSizes.size(), spade::compute::kSupportedWorkgroupSizes.size())
        << "this file's sweep array and compute/backend.hpp's allowed set have different sizes";
    for (std::size_t i = 0; i < kWorkgroupSizes.size(); ++i) {
        EXPECT_EQ(kWorkgroupSizes[i], spade::compute::kSupportedWorkgroupSizes[i])
            << "sweep entry " << i << " is not the allowed-set entry at the same index";
    }

    for (const uint32_t size : spade::compute::kSupportedWorkgroupSizes) {
        EXPECT_TRUE(spade::compute::workgroup_size_supported(size))
            << size << " is in kSupportedWorkgroupSizes but the predicate rejects it";
    }

    // The message a rejection produces names the set from its single
    // definition, so it can never go stale against the array above.
    const std::string text = spade::compute::supported_workgroup_sizes_text();
    for (const uint32_t size : spade::compute::kSupportedWorkgroupSizes) {
        EXPECT_NE(text.find(std::to_string(size)), std::string::npos)
            << "the allowed-set message '" << text << "' does not name " << size;
    }
}

TEST(WorkgroupSizeContract, UnsupportedSizeIsRejectedByName) {
    // Four ways to be wrong, each a different KIND of wrong: zero (a divisor
    // that would divide by zero), one below the smallest compiled size, a
    // plausible-looking power of two nobody compiled, and a value past every
    // device's maxComputeWorkGroupInvocations.
    constexpr std::array<uint32_t, 4> kRejected = {0u, 16u, 256u, 4096u};

    const Result<LoadedScenario> loaded = load_scenario("ballistic");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;

    for (const uint32_t size : kRejected) {
        ASSERT_FALSE(spade::compute::workgroup_size_supported(size))
            << size << " must not be in the allowed set for this test to mean anything";

        const BackendDesc backend{.kind = BackendKind::vulkan, .workgroup_size = size};
        const Result<Simulation> sim = spade::testing::start_scenario(loaded->scenario, backend);

        ASSERT_FALSE(sim.has_value())
            << "workgroup_size=" << size << " was ACCEPTED; the knob's allowed set is not enforced";
        EXPECT_EQ(sim.error().code, Code::invalid_argument)
            << "workgroup_size=" << size << ": expected invalid_argument (the caller asked for "
            << "something this build cannot provide), got: " << sim.error().context;
        // EXACT TOKENS, NOT LOOSE SUBSTRING SEARCHES (Task 9b review, minor M4).
        // The first spelling of these looked for the field name, the rejected
        // value, and each allowed value as three INDEPENDENT substrings. For
        // single-digit values that is close to vacuous -- `find("0")` is
        // satisfied by any zero character anywhere in the sentence -- and for
        // the allowed set it was worse than vacuous: `find("32")` succeeds on
        // the "32" inside "128", so the check could pass on a message that
        // never named 32 at all.
        //
        // Both are now single CONTIGUOUS tokens. The field name must sit
        // directly against the rejected value, and the allowed set must appear
        // as the one rendered string compute/backend.hpp produces -- which also
        // means this assertion cannot drift from the message the code emits,
        // because both come from supported_workgroup_sizes_text().
        const std::string value_token = "workgroup_size " + std::to_string(size);
        EXPECT_NE(sim.error().context.find(value_token), std::string::npos)
            << "the rejection must name the offending field against its value ('" << value_token
            << "'): " << sim.error().context;

        const std::string allowed_token = spade::compute::supported_workgroup_sizes_text();
        EXPECT_NE(sim.error().context.find(allowed_token), std::string::npos)
            << "the rejection must name the allowed set verbatim ('" << allowed_token
            << "'): " << sim.error().context;
        std::printf("    workgroup_size=%-5u rejected: %s\n", size, sim.error().context.c_str());
    }
}

TEST_F(GpuInvarianceTest, TwoWorldIsolationBitIdenticalAcrossWorkgroupSizes) {
    if (!vulkan_available()) GTEST_SKIP();
    const Result<LoadedScenario> loaded = load_scenario("two_world_isolation");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    expect_workgroup_size_invariance(loaded->scenario, "two_world_isolation");
}

// ---------------------------------------------------------------------------
// heterogeneous_geometry_set (S6 Task 6b), REPLICATED here so it can be run
// as an ordinary Scenario -- it is test-constructed (WorldInstanceDesc built
// by hand from two DIFFERENT world files), not a corpus yaml, so there is
// nothing for load_scenario() to load. Every field below is copied verbatim
// from test_gpu_parity.cpp's own heterogeneous_* helpers (same world files,
// same seeds, same spawns, same capacity override) rather than re-derived, to
// keep transcription risk at zero: this is the coordinator's own named
// scenario, not a new one.
// ---------------------------------------------------------------------------

namespace {

constexpr uint64_t kHeteroBallisticSeed = 0x6E7EA05EEDULL;
constexpr uint64_t kHeteroGateSeed = 0x9A7EC7A7EULL;
constexpr uint64_t kHeteroDtNs = 1'000'000;
constexpr uint32_t kHeteroSubsteps = 1;
constexpr uint64_t kHeteroSteps = 900;

[[nodiscard]] Result<WorldInstanceDesc> hetero_instance(const std::filesystem::path& world_path, uint64_t seed) {
    const Result<WorldDesc> world = resolve_world(WorldRef{world_path});
    if (!world) return std::unexpected(world.error());

    WorldInstanceDesc instance;
    instance.world = *world;
    // The two world files declare different capacities; overridden to the
    // same {1,1,1,1} on the BUILT desc so the solo and 2-world shapes
    // uniformize identically -- see test_gpu_parity.cpp's own note on this.
    instance.world.capacities = Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1};
    instance.seed = seed;
    instance.turbulence = dryden_params(TurbulenceLevel::none);
    instance.contacts.proxy_radius = 0.1f;
    instance.grid.cell_size = 0.5f;
    return instance;
}

[[nodiscard]] Result<WorldSetDesc> hetero_world_set() {
    Result<WorldInstanceDesc> w0 = hetero_instance(golden_dir() / "worlds" / "ballistic.world.yaml", kHeteroBallisticSeed);
    if (!w0) return std::unexpected(w0.error());
    Result<WorldInstanceDesc> w1 = hetero_instance(golden_dir() / "worlds" / "gate.world.yaml", kHeteroGateSeed);
    if (!w1) return std::unexpected(w1.error());
    WorldSetDesc set;
    set.worlds = {std::move(*w0), std::move(*w1)};
    return set;
}

[[nodiscard]] BodySpawn hetero_ballistic_spawn() {
    BodySpawn b{};
    b.pos = {0.0f, 2.0f, 0.0f};
    b.vel = {0.3f, 0.0f, -0.2f};
    b.omega_body = {0.15f, 0.2f, -0.1f};
    b.mass = 1.0f;
    b.inv_inertia_diag = {100.0f, 120.0f, 90.0f};
    return b;
}

[[nodiscard]] BodySpawn hetero_gate_spawn() {
    BodySpawn b{};
    b.pos = {0.0f, 5.0f, 0.0f};
    b.vel = {0.4f, 0.0f, 0.0f};
    b.omega_body = {0.2f, -0.1f, 0.15f};
    b.mass = 0.85f;
    b.inv_inertia_diag = {110.0f, 95.0f, 130.0f};
    return b;
}

[[nodiscard]] Scenario hetero_scenario() {
    Scenario s;
    s.name = "heterogeneous_geometry_set";
    s.dt_ns = kHeteroDtNs;
    s.substeps = kHeteroSubsteps;
    s.steps = kHeteroSteps;
    s.build = []() -> Result<WorldSetDesc> { return hetero_world_set(); };
    s.setup = [](Simulation& sim) -> Result<void> {
        if (const Result<BodyRef> b0 = sim.spawn(0, hetero_ballistic_spawn()); !b0) {
            return std::unexpected(b0.error());
        }
        if (const Result<BodyRef> b1 = sim.spawn(1, hetero_gate_spawn()); !b1) {
            return std::unexpected(b1.error());
        }
        return {};
    };
    return s;
}

}  // namespace

TEST_F(GpuInvarianceTest, HeterogeneousGeometrySetBitIdenticalAcrossWorkgroupSizes) {
    if (!vulkan_available()) GTEST_SKIP();
    expect_workgroup_size_invariance(hetero_scenario(), "heterogeneous_geometry_set (2 worlds, differing SDF)");
}

// ===========================================================================
// BATTERY 2 -- GPU BATCHING INVARIANCE.
//
// two_world_isolation's two worlds, stepped ALONE (a 1-world set) and
// EMBEDDED in the corpus's 2-world set, on the VULKAN backend, must produce
// bit-identical world_digest() per world. This is the device-side twin of
// ParityGeometry.HeterogeneousGeometrySetMatchesSoloRuns (test_gpu_parity.cpp,
// CPU-only) -- nothing today proves this property survives the device.
//
// two_world_isolation is chosen over building a fresh set for this because it
// is the corpus's own cross-world-leak scenario (overlapping coordinates,
// per-world turbulent seeds -- see that file's header) and already has a
// committed `expected_digest`, which is what lets this test PROVE its own
// hand-rolled replica is transcribed correctly before trusting anything else
// it measures: see the CPU-backend cross-check below.
//
// WHY A HAND-ROLLED REPLICA RATHER THAN load_scenario("two_world_isolation").
// The loaded Scenario's `setup` closure spawns BOTH worlds in one pass and is
// opaque (a std::function) -- there is no way to ask it for "just world 1's
// spawns" to build a solo set. The instances and spawns below are transcribed
// verbatim from tests/golden/scenarios/two_world_isolation.scenario.yaml.
// ===========================================================================

namespace {

constexpr uint64_t kTwoWorldSeed0 = 0x1501A7E0ULL;
constexpr uint64_t kTwoWorldSeed1 = 0x1501A7E1ULL;
constexpr uint64_t kTwoWorldDtNs = 2'000'000;
constexpr uint32_t kTwoWorldSubsteps = 2;
constexpr uint64_t kTwoWorldSteps = 300;
// The corpus file's own committed golden (two_world_isolation.scenario.yaml's
// `expected_digest`) -- used ONLY to cross-check that this file's hand-rolled
// replica of the scenario is transcribed correctly, never as this test's own
// claim (that is TwoWorldIsolationMatchesTheCpuWithinBands's job, elsewhere).
//
// REGENERATED WITH THE CORPUS TWICE, and the two reasons were different:
// 0xaab013d47c0eba42 -> 0xe43eedcf57f740dc when the GNSS arena JOINED the
// state walk (four new entries), then -> 0x69c6467bf2df206c when the GNSS
// ROW GREW 96 -> 112 bytes (state_digest folds elem_size, so a row that
// changes size moves every digest even though the walk did not change). That is correct for THIS constant
// and would be wrong for the four in test_determinism.cpp's GoldenCorpus table:
// this one is a MIRROR of the yaml, whose only job is to catch a transcription
// slip in the replica beside it, so it must track the yaml. Those four are the
// retired builder's own output, and not moving them is the entire independence
// claim. Two hardcoded digests, opposite regeneration rules, and the difference
// is what each number is a copy OF.
constexpr uint64_t kTwoWorldIsolationExpectedDigest = 0x69c6467bf2df206cULL;

[[nodiscard]] Result<WorldInstanceDesc> two_world_isolation_instance(uint64_t seed) {
    const Result<WorldDesc> world =
        resolve_world(WorldRef{golden_dir() / "worlds" / "two_world_isolation.world.yaml"});
    if (!world) return std::unexpected(world.error());

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = seed;
    instance.turbulence = dryden_params(TurbulenceLevel::moderate);
    instance.contacts.restitution_e = 0.400000006f;
    instance.contacts.friction_mu = 0.200000003f;
    instance.contacts.baumgarte_beta = 0.200000003f;
    instance.contacts.slop = 0.00100000005f;
    instance.contacts.proxy_radius = 0.119999997f;
    instance.grid.cell_size = 0.239999995f;
    return instance;
}

// World 0's 8-body 2x2x2 cluster around (0, 1, 0), verbatim from the yaml.
[[nodiscard]] std::vector<BodySpawn> two_world_isolation_world0_spawns() {
    std::vector<BodySpawn> out;
    for (float dy : {0.0f, 0.200000005f}) {
        for (float dz : {0.0f, 0.200000003f}) {
            for (float dx : {0.0f, 0.200000003f}) {
                BodySpawn b{};
                b.pos = {dx, 1.0f + dy, dz};
                b.mass = 0.5f;
                b.inv_inertia_diag = {200.0f, 200.0f, 200.0f};
                out.push_back(b);
            }
        }
    }
    return out;
}

// World 1's pair, AT THE SAME COORDINATES as two of world 0's cluster.
[[nodiscard]] std::vector<BodySpawn> two_world_isolation_world1_spawns() {
    std::vector<BodySpawn> out;
    for (float dx : {0.0f, 0.200000003f}) {
        BodySpawn b{};
        b.pos = {dx, 1.0f, 0.0f};
        b.mass = 0.5f;
        b.inv_inertia_diag = {200.0f, 200.0f, 200.0f};
        out.push_back(b);
    }
    return out;
}

}  // namespace

TEST_F(GpuInvarianceTest, TwoWorldIsolationWorldsMatchSoloRunsOnGpu) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldInstanceDesc> w0 = two_world_isolation_instance(kTwoWorldSeed0);
    ASSERT_TRUE(w0.has_value()) << w0.error().context;
    const Result<WorldInstanceDesc> w1 = two_world_isolation_instance(kTwoWorldSeed1);
    ASSERT_TRUE(w1.has_value()) << w1.error().context;

    WorldSetDesc combined_desc;
    combined_desc.worlds = {*w0, *w1};

    // TRANSCRIPTION CROSS-CHECK: this replica, run on the CPU backend, must
    // reproduce the corpus file's own committed digest exactly. If it does
    // not, every claim this test makes below is about the WRONG scenario, and
    // this assertion is what catches that rather than a silent false pass.
    {
        Result<Simulation> cpu_check = Simulation::create(combined_desc, kTwoWorldDtNs, kTwoWorldSubsteps);
        ASSERT_TRUE(cpu_check.has_value()) << cpu_check.error().context;
        for (const BodySpawn& b : two_world_isolation_world0_spawns()) {
            ASSERT_TRUE(cpu_check->spawn(0, b).has_value());
        }
        for (const BodySpawn& b : two_world_isolation_world1_spawns()) {
            ASSERT_TRUE(cpu_check->spawn(1, b).has_value());
        }
        ASSERT_TRUE(cpu_check->step(kTwoWorldSteps).has_value());
        const uint64_t digest = spade::testing::state_digest(*cpu_check);
        ASSERT_EQ(digest, kTwoWorldIsolationExpectedDigest)
            << "this file's hand-rolled replica of two_world_isolation does not reproduce the corpus's own "
            << "committed digest (0x" << std::hex << kTwoWorldIsolationExpectedDigest << ") -- got 0x"
            << digest << std::dec << ". A transcription error here would make everything below meaningless.";
    }

    // --- the embedded (2-world) GPU run -------------------------------------
    Result<Simulation> combined = Simulation::create(combined_desc, kTwoWorldDtNs, kTwoWorldSubsteps,
                                                      BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(combined.has_value()) << combined.error().context;
    for (const BodySpawn& b : two_world_isolation_world0_spawns()) {
        ASSERT_TRUE(combined->spawn(0, b).has_value());
    }
    for (const BodySpawn& b : two_world_isolation_world1_spawns()) {
        ASSERT_TRUE(combined->spawn(1, b).has_value());
    }
    ASSERT_TRUE(combined->flush_structural().has_value());
    const uint64_t combined_before = spade::testing::state_digest(*combined);
    ASSERT_TRUE(combined->step(kTwoWorldSteps).has_value());
    ASSERT_EQ(combined->tick().value, kTwoWorldSteps);
    const uint64_t combined_after = spade::testing::state_digest(*combined);

    // VACUITY DEFENSE: the embedded run actually stepped and moved.
    EXPECT_NE(combined_after, 0u) << "embedded run: state_digest() folded to zero";
    EXPECT_NE(combined_after, combined_before) << "embedded run: the digest did not move across 300 steps";

    // --- world 0 alone -------------------------------------------------------
    {
        WorldSetDesc solo;
        solo.worlds = {*w0};
        Result<Simulation> solo_sim =
            Simulation::create(solo, kTwoWorldDtNs, kTwoWorldSubsteps, BackendDesc{.kind = BackendKind::vulkan});
        ASSERT_TRUE(solo_sim.has_value()) << solo_sim.error().context;
        for (const BodySpawn& b : two_world_isolation_world0_spawns()) {
            ASSERT_TRUE(solo_sim->spawn(0, b).has_value());
        }
        ASSERT_TRUE(solo_sim->step(kTwoWorldSteps).has_value());
        ASSERT_EQ(solo_sim->tick().value, kTwoWorldSteps);

        const uint64_t combined_digest = spade::testing::world_digest(*combined, 0);
        const uint64_t solo_digest = spade::testing::world_digest(*solo_sim, 0);
        std::printf("\n=== GPU batching invariance: two_world_isolation world 0 ===\n");
        std::printf("    embedded=0x%016llx  solo=0x%016llx\n", static_cast<unsigned long long>(combined_digest),
                    static_cast<unsigned long long>(solo_digest));
        EXPECT_NE(combined_digest, 0u) << "world 0: world_digest() folded to zero on the embedded leg";
        EXPECT_EQ(combined_digest, solo_digest)
            << "world 0 (the 8-body cluster) diverged between its slice of the embedded 2-world GPU set and "
            << "its solo GPU run -- a world's trajectory must not depend on what else shares its world set, "
            << "and this is the property no CPU-only test can observe";
    }

    // --- world 1 alone -------------------------------------------------------
    {
        WorldSetDesc solo;
        solo.worlds = {*w1};
        Result<Simulation> solo_sim =
            Simulation::create(solo, kTwoWorldDtNs, kTwoWorldSubsteps, BackendDesc{.kind = BackendKind::vulkan});
        ASSERT_TRUE(solo_sim.has_value()) << solo_sim.error().context;
        for (const BodySpawn& b : two_world_isolation_world1_spawns()) {
            ASSERT_TRUE(solo_sim->spawn(0, b).has_value());
        }
        ASSERT_TRUE(solo_sim->step(kTwoWorldSteps).has_value());
        ASSERT_EQ(solo_sim->tick().value, kTwoWorldSteps);

        const uint64_t combined_digest = spade::testing::world_digest(*combined, 1);
        const uint64_t solo_digest = spade::testing::world_digest(*solo_sim, 0);
        std::printf("=== GPU batching invariance: two_world_isolation world 1 ===\n");
        std::printf("    embedded=0x%016llx  solo=0x%016llx\n", static_cast<unsigned long long>(combined_digest),
                    static_cast<unsigned long long>(solo_digest));
        EXPECT_NE(combined_digest, 0u) << "world 1: world_digest() folded to zero on the embedded leg";
        EXPECT_EQ(combined_digest, solo_digest)
            << "world 1 (the two overlapping bodies) diverged between its slice of the embedded 2-world GPU "
            << "set and its solo GPU run -- the exact leak this scenario exists to catch (world 1's bodies "
            << "sit at world 0's own coordinates, so a cross-world broad-phase leak would move them)";
    }
}

// ===========================================================================
// BATTERY 3 -- BACKEND-SEAM DIGEST RE-PROOF AT TIP (A7 leg 1, RERUN).
//
// test_gpu_state_mirror.cpp's BackendKnobInvariance.
// CpuBackendReproducesTodaysDigestsAcrossTheFullCorpus already proves this at
// S6 Task 5's tip (backend={} vs an explicit BackendDesc{cpu} agree, and both
// agree with the corpus's committed goldens). This task's brief asks for the
// SAME cpu-path claim reconfirmed HERE, at Task 9's tip, after Tasks 6-8's
// kernel work landed: the cpu path is untouched by any of it (it is the host
// schedule, unchanged since before S6), but "untouched" is a claim this test
// makes rather than assumes.
//
// HOST-ONLY -- deliberately not Gpu*-prefixed (this file's header note).
// ===========================================================================

TEST(BackendSeamReproof, CpuPathCorpusDigestsUnchangedAtTip) {
    std::vector<std::filesystem::path> paths;
    std::error_code ec;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(golden_dir() / "scenarios", ec)) {
        if (entry.is_regular_file() && entry.path().filename().string().ends_with(".scenario.yaml")) {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    ASSERT_EQ(paths.size(), std::size_t{6}) << "the golden corpus is expected to carry six scenarios";

    for (const std::filesystem::path& path : paths) {
        const Result<LoadedScenario> loaded = spade::testing::scenario_from_yaml(path);
        ASSERT_TRUE(loaded.has_value()) << path.string() << ": " << loaded.error().context;

        const Result<uint64_t> digest = spade::testing::run_scenario(loaded->scenario);
        ASSERT_TRUE(digest.has_value()) << loaded->data->name << ": " << digest.error().context;

        std::printf("    %-20s cpu digest=0x%016llx  golden=0x%016llx  %s\n", loaded->data->name.c_str(),
                    static_cast<unsigned long long>(*digest),
                    static_cast<unsigned long long>(loaded->data->expected_digest),
                    *digest == loaded->data->expected_digest ? "match" : "MISMATCH");
        EXPECT_EQ(*digest, loaded->data->expected_digest)
            << loaded->data->name << ": the cpu path's digest moved from the committed golden -- at Task 9's "
            << "tip, after every S6 kernel wave, the cpu schedule must still be untouched";
    }
}

// ===========================================================================
// BATTERY 4 -- RESTORE-CROSS-BACKEND CONTRACT.
//
// A snapshot carries no notion of which backend produced it (ReplayConfig,
// sim/simulation.hpp, pins dt_ns/substeps/config_hash -- never BackendKind),
// so restore()'s own schema/config check has no reason to refuse a blob taken
// on the OTHER backend. This proves that directly, both directions, over the
// same resumed-run shape test_gpu_parity.cpp's
// RestoredRunResumesAndMatchesTheCpuWithinBands already proves for one
// direction (cpu source -> vulkan target): a CPU snapshot restores into a
// vulkan Simulation, AND a vulkan snapshot restores into a cpu Simulation,
// and continuation from the restored tick stays within the pinned
// restore_resume parity bands against the UNINTERRUPTED run of the OTHER
// backend.
//
// THE SCENARIO is the identical shape to test_gpu_parity.cpp's private
// `resume` scenario (same world file, same seed, same two spawns, same
// dt_ns/substeps, same tick 300 -> 700 resume window) -- reused deliberately
// so `spade::testing::bands::restore_resume`'s already-measured-and-pinned
// bands genuinely apply rather than being reused across a different
// experiment. If either direction's MEASURED table below lands outside that
// band, that is a Task 9 finding for the coordinator (bands are never
// widened to make a failure pass), not something silently absorbed.
// ===========================================================================

namespace {

constexpr uint64_t kCrossSnapshotTick = 300;
constexpr uint64_t kCrossFinalTick = 700;

[[nodiscard]] Scenario cross_backend_resume_scenario() {
    Scenario resume;
    resume.name = "cross_backend_restore";
    resume.dt_ns = 1'000'000;
    resume.substeps = 1;
    resume.steps = kCrossFinalTick;
    resume.build = []() -> Result<WorldSetDesc> {
        WorldInstanceDesc prototype;
        prototype.turbulence = dryden_params(TurbulenceLevel::none);
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
            [] {
                BodySpawn b;
                b.pos = {0.0f, 4.5f, 0.0f};
                b.vel = {0.35f, 0.0f, 0.0f};
                b.omega_body = {0.2f, -0.1f, 0.15f};
                b.mass = 0.9f;
                b.inv_inertia_diag = {110.0f, 95.0f, 130.0f};
                return b;
            }(),
            [] {
                BodySpawn b;
                b.pos = {1.8f, 4.5f, 0.0f};
                b.vel = {-0.2f, 0.0f, 0.1f};
                b.omega_body = {0.1f, 0.3f, -0.2f};
                b.mass = 1.1f;
                b.inv_inertia_diag = {90.0f, 140.0f, 105.0f};
                return b;
            }(),
        }};
        for (uint32_t w = 0; w < 2; ++w) {
            const Result<BodyRef> ref = sim.spawn(w, spawns[w]);
            if (!ref) return std::unexpected(ref.error());
        }
        return {};
    };
    return resume;
}

}  // namespace

TEST_F(GpuInvarianceTest, CpuSnapshotRestoresIntoVulkanAndContinuesWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    const Scenario resume = cross_backend_resume_scenario();

    // --- the uninterrupted CPU reference to kCrossFinalTick -----------------
    Result<Simulation> cpu_ref = spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::cpu});
    ASSERT_TRUE(cpu_ref.has_value()) << cpu_ref.error().context;
    ASSERT_TRUE(spade::testing::advance_scenario(resume, *cpu_ref, kCrossFinalTick).has_value());

    // --- the CPU source, snapshotted at kCrossSnapshotTick ------------------
    Result<Simulation> cpu_source = spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::cpu});
    ASSERT_TRUE(cpu_source.has_value()) << cpu_source.error().context;
    ASSERT_TRUE(spade::testing::advance_scenario(resume, *cpu_source, kCrossSnapshotTick).has_value());
    // GENUINELY-DIFFERENT-BACKEND EVIDENCE (vacuity discipline): a cpu-backend
    // Simulation's upload counter is defined to stay 0 for its whole life
    // (Simulation::vulkan_upload_count(), simulation.cpp) -- this is a
    // runtime-observable fact about `cpu_source`, not merely the enum value
    // it was constructed with.
    EXPECT_EQ(cpu_source->vulkan_upload_count(), 0u) << "source claims the cpu backend but has a nonzero "
                                                     << "device-upload count";
    const Result<spade::SnapshotBlob> blob = cpu_source->snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    ASSERT_EQ(blob->tick().value, kCrossSnapshotTick);

    // --- the vulkan target: step first (clean mirror), THEN restore ---------
    Result<Simulation> vulkan_target =
        spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(vulkan_target.has_value()) << vulkan_target.error().context;
    ASSERT_TRUE(vulkan_target->step(1).has_value());
    const uint64_t uploads_before_restore = vulkan_target->vulkan_upload_count();
    EXPECT_GE(uploads_before_restore, 1u) << "target claims the vulkan backend but never uploaded a device "
                                          << "mirror";

    // THE CONTRACT: restore() ACCEPTS a same-schema/same-config blob taken on
    // the OTHER backend. BackendKind is not part of ReplayConfig (see the
    // documentation added to sim/simulation.hpp by this task), so there is no
    // reason for this to be refused -- and this assertion is what turns that
    // reasoning into a checked fact.
    const Result<void> restored = vulkan_target->restore(*blob);
    ASSERT_TRUE(restored.has_value())
        << "restore() must ACCEPT a same-schema/same-config blob from the OTHER backend: "
        << restored.error().context;
    ASSERT_EQ(vulkan_target->tick().value, kCrossSnapshotTick) << "restore must also reset the clock";

    ASSERT_TRUE(spade::testing::advance_scenario(resume, *vulkan_target, kCrossFinalTick).has_value());
    EXPECT_GT(vulkan_target->vulkan_upload_count(), uploads_before_restore)
        << "restore() must force exactly one re-upload -- otherwise the device continued stepping stale "
        << "pre-restore state";
    ASSERT_EQ(vulkan_target->tick().value, kCrossFinalTick);

    using namespace spade::testing::bands::restore_resume;
    const std::vector<BandEntry> table = join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands());
    const Result<ParityReport> report = compare_arrays(cpu_ref->arenas(), vulkan_target->arenas(), table);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    report->print(
        "cross-backend restore: CPU snapshot @ tick 300 -> vulkan target, resumed to 700, vs uninterrupted "
        "CPU reference (band: restore_resume, reused -- same scenario/timing as its measurement)");

    for (const spade::testing::QuantityReport& q : report->quantities) {
        EXPECT_TRUE(q.within_band())
            << "cpu->vulkan restore: '" << q.quantity << "' is outside its pinned band -- measured max |abs| "
            << q.max_abs << ", max rel " << q.max_rel << ", pinned (" << q.band.abs << ", " << q.band.rel
            << "); " << q.elements_outside_band << " of " << q.elements_compared << " elements";
        EXPECT_GT(q.elements_compared, std::size_t{0});
    }
}

TEST_F(GpuInvarianceTest, VulkanSnapshotRestoresIntoCpuAndContinuesWithinBands) {
    if (!vulkan_available()) GTEST_SKIP();

    const Scenario resume = cross_backend_resume_scenario();

    // --- the uninterrupted VULKAN reference to kCrossFinalTick --------------
    Result<Simulation> vulkan_ref =
        spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(vulkan_ref.has_value()) << vulkan_ref.error().context;
    ASSERT_TRUE(spade::testing::advance_scenario(resume, *vulkan_ref, kCrossFinalTick).has_value());

    // --- the VULKAN source, snapshotted at kCrossSnapshotTick ----------------
    // snapshot() needs no pre-restore dirty choreography here (that concern is
    // the RECEIVING device mirror's; the SOURCE is simply read back after its
    // last step, exactly as every vulkan-path snapshot is -- see replay.hpp's
    // "the digest machinery works on read-back arenas" note, which applies
    // equally to a snapshot's byte image).
    Result<Simulation> vulkan_source =
        spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(vulkan_source.has_value()) << vulkan_source.error().context;
    ASSERT_TRUE(spade::testing::advance_scenario(resume, *vulkan_source, kCrossSnapshotTick).has_value());
    EXPECT_GE(vulkan_source->vulkan_upload_count(), 1u) << "source claims the vulkan backend but never "
                                                        << "uploaded a device mirror";
    const Result<spade::SnapshotBlob> blob = vulkan_source->snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    ASSERT_EQ(blob->tick().value, kCrossSnapshotTick);

    // --- the CPU target -------------------------------------------------------
    Result<Simulation> cpu_target = spade::testing::start_scenario(resume, BackendDesc{.kind = BackendKind::cpu});
    ASSERT_TRUE(cpu_target.has_value()) << cpu_target.error().context;
    // GENUINELY-DIFFERENT-BACKEND EVIDENCE, the other direction: the target
    // never touches a device mirror at all, before or after restore.
    EXPECT_EQ(cpu_target->vulkan_upload_count(), 0u);

    const Result<void> restored = cpu_target->restore(*blob);
    ASSERT_TRUE(restored.has_value())
        << "restore() must ACCEPT a same-schema/same-config blob from the OTHER backend: "
        << restored.error().context;
    ASSERT_EQ(cpu_target->tick().value, kCrossSnapshotTick) << "restore must also reset the clock";

    ASSERT_TRUE(spade::testing::advance_scenario(resume, *cpu_target, kCrossFinalTick).has_value());
    ASSERT_EQ(cpu_target->tick().value, kCrossFinalTick);
    EXPECT_EQ(cpu_target->vulkan_upload_count(), 0u) << "the cpu path must never touch a device mirror, "
                                                     << "restore included";

    using namespace spade::testing::bands::restore_resume;
    const std::vector<BandEntry> table = join(body_bands(kPos, kVel, kOrient, kOmega, kSpecificForce), medium_bands());
    const Result<ParityReport> report = compare_arrays(vulkan_ref->arenas(), cpu_target->arenas(), table);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    report->print(
        "cross-backend restore: vulkan snapshot @ tick 300 -> cpu target, resumed to 700, vs uninterrupted "
        "vulkan reference (band: restore_resume, reused -- same scenario/timing as its measurement)");

    for (const spade::testing::QuantityReport& q : report->quantities) {
        EXPECT_TRUE(q.within_band())
            << "vulkan->cpu restore: '" << q.quantity << "' is outside its pinned band -- measured max |abs| "
            << q.max_abs << ", max rel " << q.max_rel << ", pinned (" << q.band.abs << ", " << q.band.rel
            << "); " << q.elements_outside_band << " of " << q.elements_compared << " elements";
        EXPECT_GT(q.elements_compared, std::size_t{0});
    }
}

// ===========================================================================
// BATTERY 5 (DISPATCH-MUST-CARRY, T7 review finding I1) -- A DEEP SORT
// NETWORK, PLUS A GENUINELY SEGMENTED CASE.
//
// GpuGridSort's four cases (test_gpu_parity.cpp) all sort a domain of 4 or 8
// entries -- a 2- or 3-stage bitonic network. A workgroup-size sweep needs a
// deeper network to be discriminating, and this task's cross-backend/knob
// work needs a genuinely SEGMENTED (per-world) case, which nothing before
// this task exercises in isolation (only the two `shower*` PARITY tests touch
// the per-world shape, and only through 900 steps of full physics, not a
// direct key comparison).
//
// Both cases use the SAME instrument GpuGridSort does: the ACTUAL grid_build
// kernel over ACTUAL spawn positions, inside ONE step of an ACTUAL vulkan
// Simulation (ballistic.world.yaml's empty SDF means nothing has moved a body
// by the time grid_build reads `pos` -- see that suite's own header for the
// full argument), read back via Simulation::vulkan_grid_entries() and
// compared ELEMENT FOR ELEMENT against std::sort + physics::grid_entry_less.
// ===========================================================================

namespace {

class GpuGridSortDeepTest : public ::testing::Test {
protected:
    void SetUp() override { spade::compute::set_error_sink(&fail_on_validation_message); }
    void TearDown() override { spade::compute::set_error_sink(nullptr); }
};

[[nodiscard]] Result<std::vector<spade::compute::GridEntryRow>> stepped_grid_entries(Simulation& sim) {
    if (Result<void> stepped = sim.step(1); !stepped) return std::unexpected(stepped.error());
    return sim.vulkan_grid_entries();
}

// The CPU's own answer, built with the CPU's own functions -- identical in
// method to GpuGridSort's cpu_expected_entries(), duplicated for the same
// self-containment reason as this file's band tables above.
[[nodiscard]] std::vector<spade::physics::GridEntry> cpu_expected_entries(const Simulation& sim, float cell_size) {
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

// Verifies ONE segment (a contiguous slice of the device key array, already
// isolated by the caller) against its expected live keys, in order. The three
// checks GpuGridSort's own expect_sorted_keys_match() makes, generalized to
// operate on a pre-sliced segment rather than the whole domain, so the same
// function serves both the single-segment (batched) and multi-segment
// (per-world) cases below.
void expect_one_segment_sorted(std::span<const spade::compute::GridEntryRow> segment,
                               const std::vector<spade::physics::GridEntry>& expected, std::string_view label) {
    std::vector<spade::compute::GridEntryRow> live;
    bool sentinel_seen = false;
    bool live_after_sentinel = false;
    for (const spade::compute::GridEntryRow& row : segment) {
        if (row.world == spade::compute::kGridInvalidWorld) {
            sentinel_seen = true;
        } else {
            if (sentinel_seen) live_after_sentinel = true;
            live.push_back(row);
        }
    }
    EXPECT_FALSE(live_after_sentinel)
        << label << ": a LIVE key follows a sentinel within this segment";

    ASSERT_EQ(live.size(), expected.size())
        << label << ": the device produced " << live.size() << " live keys in this segment, the CPU build "
        << "produced " << expected.size();

    for (std::size_t i = 0; i < live.size(); ++i) {
        EXPECT_EQ(live[i].world, expected[i].world) << label << ": key " << i << " world";
        EXPECT_EQ(live[i].cell_x, expected[i].cell.x) << label << ": key " << i << " cell.x";
        EXPECT_EQ(live[i].cell_y, expected[i].cell.y) << label << ": key " << i << " cell.y";
        EXPECT_EQ(live[i].cell_z, expected[i].cell.z) << label << ": key " << i << " cell.z";
        EXPECT_EQ(live[i].slot, expected[i].slot)
            << label << ": key " << i << " slot (the tie-break, physics/grid.cpp:334)";
    }
}

// ballistic.world.yaml: no geometry at all, 4 bodies declared. Same prototype
// shape as GpuGridSort's own sort_test_world_set(), duplicated for this file's
// self-containment.
[[nodiscard]] Result<WorldSetDesc> deep_sort_world_set(uint32_t count, uint64_t seed) {
    WorldInstanceDesc prototype;
    prototype.turbulence = dryden_params(TurbulenceLevel::none);
    prototype.contacts.proxy_radius = 0.1f;
    prototype.grid.cell_size = 1.0f;
    return spade::world_set_from(WorldRef{golden_dir() / "worlds" / "ballistic.world.yaml"}, count, seed, prototype);
}

constexpr uint32_t kDeepSortBodiesPerWorld = 4;  // ballistic.world.yaml files bodies: 4
constexpr float kDeepSortCellSize = 1.0f;
// 32 worlds * 4 bodies/world = 128 = 2^7 slots, exactly a power of two: ONE
// batched segment, no sentinel padding at all, and a 7*(7+1)/2 = 28-STAGE
// bitonic network -- the deep case GpuGridSort's shallow 4/8-entry cases
// cannot exercise.
constexpr uint32_t kDeepSortWorldCount = 32;
constexpr std::size_t kDeepSortDomain = 128;

// Reversed within EVERY world (the same adversarial pattern GpuGridSort's own
// ReversedKeySetMatchesTheCpu uses for one world): cells descend with the
// local body index, so the correct per-world sub-order is a full local
// reversal. World id is already the outer, correctly-ascending sort key by
// construction (bodies are spawned into world w in loop order w=0..31), so
// this input still gives the bitonic network genuine, non-trivial work at
// every one of its 28 stages -- long-stride comparator pairs span DIFFERENT
// worlds (decided instantly by the leading world field) while short-stride
// pairs within a world's own 4-slot block require a real swap -- and the
// comparison below is against std::sort's OWN output, element for element, so
// it does not matter how "hard" the input happens to be: any wrong swap, at
// any stage, at any stride, fails it.
void spawn_deep_sort_bodies(Simulation& sim) {
    for (uint32_t w = 0; w < kDeepSortWorldCount; ++w) {
        for (uint32_t i = 0; i < kDeepSortBodiesPerWorld; ++i) {
            BodySpawn b{};
            b.pos = {0.5f - static_cast<float>(i), 10.5f, 0.5f};
            ASSERT_TRUE(sim.spawn(w, b).has_value());
        }
    }
}

}  // namespace

TEST_F(GpuGridSortDeepTest, Deep128KeyBatchedNetworkMatchesTheCpu) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = deep_sort_world_set(kDeepSortWorldCount, 0xD33D5027ULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;

    // THE SHAPE THIS TEST MEANS TO EXERCISE, asserted rather than assumed
    // (this file's and test_gpu_parity.cpp's shared convention): one
    // prototype replicated across every world, so the set is uniform and the
    // dynamic sweep -- and this domain -- take the BATCHED, single-segment
    // form.
    {
        const Result<spade::WorldSetLayout> layout = spade::validate_world_set(*set);
        ASSERT_TRUE(layout.has_value()) << layout.error().context;
        ASSERT_TRUE(layout->uniform_dynamic_params)
            << "deep_sort_world_set must be uniform -- if this is false, this case covers the SEGMENTED "
            << "shape SegmentedPerWorldPartitionMatchesTheCpu below already covers, not the deep batched one";
    }

    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    spawn_deep_sort_bodies(*sim);

    const Result<std::vector<spade::compute::GridEntryRow>> device = stepped_grid_entries(*sim);
    ASSERT_TRUE(device.has_value()) << device.error().context;
    const std::vector<spade::physics::GridEntry> expected = cpu_expected_entries(*sim, kDeepSortCellSize);

    ASSERT_EQ(expected.size(), kDeepSortDomain) << "the premise (every one of 32*4 bodies landed in a "
                                                << "representable cell) does not hold";
    ASSERT_EQ(device->size(), kDeepSortDomain)
        << "the device key array is not the expected 128-entry padded domain -- 128 is already a power of "
        << "two, so this domain carries NO sentinel padding at all, unlike every GpuGridSort case";

    expect_one_segment_sorted(*device, expected, "deep-128-key-batched (28-stage network)");
}

TEST_F(GpuGridSortDeepTest, SegmentedPerWorldPartitionMatchesTheCpu) {
    if (!vulkan_available()) GTEST_SKIP();

    // Hand-built (not world_set_from(), which replicates ONE prototype and so
    // can only ever produce a UNIFORM set): two worlds from the SAME file,
    // differing in exactly one ContactParams byte (restitution), which is
    // what world_set.cpp's uniform_dynamic_params check keys on -- and
    // therefore what sends this set's CollisionDynamic (and grid_domain_of()'s
    // key-array partition) down the PER-WORLD, SEGMENTED shape.
    const Result<WorldDesc> world = resolve_world(WorldRef{golden_dir() / "worlds" / "ballistic.world.yaml"});
    ASSERT_TRUE(world.has_value()) << world.error().context;

    WorldInstanceDesc w0;
    w0.world = *world;
    w0.seed = 0x5E90000ULL;
    w0.turbulence = dryden_params(TurbulenceLevel::none);
    w0.contacts.proxy_radius = 0.1f;
    w0.contacts.restitution_e = 0.1f;
    w0.grid.cell_size = 1.0f;

    WorldInstanceDesc w1 = w0;
    w1.seed = 0x5E90001ULL;
    w1.contacts.restitution_e = 0.9f;  // the one differing byte

    WorldSetDesc set;
    set.worlds = {w0, w1};

    // THE SHAPE THIS TEST MEANS TO EXERCISE -- the other half of the pair.
    {
        const Result<spade::WorldSetLayout> layout = spade::validate_world_set(set);
        ASSERT_TRUE(layout.has_value()) << layout.error().context;
        ASSERT_FALSE(layout->uniform_dynamic_params)
            << "the two worlds' restitutions differ, so this set must take the PER-WORLD, SEGMENTED shape -- "
            << "if it is uniform, this case covers the same batched shape the deep case above already does";
    }

    Result<Simulation> sim = Simulation::create(set, 1'000'000, 1, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    for (uint32_t w = 0; w < 2; ++w) {
        for (uint32_t i = 0; i < kDeepSortBodiesPerWorld; ++i) {
            // Same reversed-within-world adversarial pattern as the deep case.
            BodySpawn b{};
            b.pos = {0.5f - static_cast<float>(i), 10.5f, 0.5f};
            ASSERT_TRUE(sim->spawn(w, b).has_value());
        }
    }

    const Result<std::vector<spade::compute::GridEntryRow>> device = stepped_grid_entries(*sim);
    ASSERT_TRUE(device.has_value()) << device.error().context;
    const std::vector<spade::physics::GridEntry> expected_all = cpu_expected_entries(*sim, 1.0f);

    ASSERT_EQ(expected_all.size(), std::size_t{2 * kDeepSortBodiesPerWorld});
    // segment = next_pow2(4) = 4, segments = 2 worlds -> entries = 8, exactly
    // packed (no sentinel padding within either world's own segment either).
    constexpr std::size_t kSegment = 4;
    constexpr std::size_t kSegmentedDomain = 2 * kSegment;
    ASSERT_EQ(device->size(), kSegmentedDomain)
        << "the device key array is not the expected 2-segment, 8-entry PER-WORLD domain";

    for (uint32_t w = 0; w < 2; ++w) {
        // grid_entry_less sorts by (world, cell.z, cell.y, cell.x, slot):
        // filtering a globally-sorted sequence down to one constant world
        // preserves the RELATIVE order of the retained entries, which is
        // exactly the per-world sub-order the device's own segment holds.
        std::vector<spade::physics::GridEntry> expected_w;
        for (const spade::physics::GridEntry& e : expected_all) {
            if (e.world == w) expected_w.push_back(e);
        }
        const std::span<const spade::compute::GridEntryRow> segment(device->data() + w * kSegment, kSegment);
        expect_one_segment_sorted(segment, expected_w,
                                  "segmented-per-world (2 worlds x 4 bodies, differing restitution), world " +
                                      std::to_string(w));
    }
}
