#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "compute/vulkan/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/error.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"
#include "testing/replay.hpp"
#include "testing/scenario_file.hpp"
#include "world/medium.hpp"
#include "world/world_ref.hpp"

// ---------------------------------------------------------------------------
// test_gpu_state_mirror.cpp -- S6 Task 5: the Vulkan state mirror, the step
// command recorder, and the VulkanBackend seam Simulation owns.
//
// FIVE THINGS WERE UNDER TEST AT S6 TASK 5; FOUR STILL ARE. The fifth (the
// no-op stub's byte preservation) retired with the stub itself at Task 8 -- see
// item 5 below, kept as the record of what that test was and what replaced it.
//
//
//   1. THE ROUND TRIP. Upload a 4-world gate.world.yaml set to the device
//      WITHOUT stepping, read it back, and every registered array (the full
//      18-entry walk, not merely the 14 that carry a shader binding) must be
//      byte-identical to what was uploaded.
//
//   2. DIRTY TRACKING. A structural op (spawn) between two step(1) calls on
//      the vulkan path forces a re-upload -- checked via
//      Simulation::vulkan_upload_count(), an instrumentation counter, not via
//      a GPU-side physics comparison. At Task 5 that was because the vulkan
//      path was all stub and made no physics claim at all; it stays a counter
//      because a counter measures the OUTCOME the flag exists for (bytes
//      reaching the device) rather than the flag's own value.
//      tests/test_gpu_parity.cpp is where the physics is compared.
//
//   3. FAULT PATHS. An absurd StepShape forces a real Vulkan allocation
//      failure at VulkanBackend::create() time, and it is reported as a
//      spade::Error rather than a crash -- the one fault path this box can
//      exercise for real without actually losing the device (VK_ERROR_-
//      DEVICE_LOST itself is written into every allocation/submit call site
//      this task added, spade::compute::detail's map_vk_error helpers in
//      state_mirror.cpp/step_recorder.cpp, but is not independently
//      triggerable from a test without inducing a genuine device loss on
//      the box other lanes are concurrently using -- documented here, not
//      exercised).
//
//   4. THE PER-STEP TICK WRITE (S6 Task 5 review round 1, coordinator ruling
//      on finding C2). StepRecorder now records its command buffer ONCE per
//      shape and resubmits it verbatim (finding C1's fix), which means a
//      push constant can no longer carry a value that varies per submit --
//      the tick moved to a small, persistently mapped, HOST_VISIBLE|
//      HOST_COHERENT buffer that the host writes fresh before every
//      vkQueueSubmit. This test proves that HOST-SIDE half only:
//      VulkanBackend::last_written_tick() reflects exactly the value the
//      most recent submit carried. GPU-VISIBLE CONSUMPTION IS DELIBERATELY
//      NOT TESTED HERE -- the buffer is not bound to any descriptor and no
//      kernel reads it yet (the ruling's own words: "Task 6's first real
//      kernel reads it and proves the far side"); proving THAT is Task 6's
//      job, not this fix's.
//
//   5. RETIRED AT S6 TASK 8, WITH THE THING IT TESTED. It was "THE NO-OP STUB
//      IS BYTE-PRESERVING" (Task 5 review round 2, finding I2): round 1 had
//      respelled the stub kernel as an arithmetic identity that is exact for
//      every FINITE input but does not preserve a SIGNED ZERO through IEEE-754
//      addition, and round 2 respelled it again as a pure bit copy. Task 8
//      ports the last two schedule passes, so no slot binds a stub and
//      pipeline_smoke.slang is deleted -- there is no byte-preserving kernel
//      left to make the claim about.
//
//      WHAT SURVIVES IS THE TEST BODY, rescoped: section 4 still steps the
//      vulkan path once with a live body carrying a -0.0f velocity component
//      and memcmps the read-back arrays against the pre-step upload. It now
//      asserts which arrays a REAL chain may write rather than that none of
//      them may, which is the same instrument pointed at a stronger claim --
//      and the -0.0f precondition is kept verbatim, because "the spawn path
//      delivers exactly this bit pattern" is what stops the comparison being
//      vacuous either way.
//
// PLUS the A7 cpu-leg backend-knob invariance test, which is DELIBERATELY
// NOT Gpu*-prefixed: BackendKind::cpu never touches Vulkan (Simulation::
// step()'s cpu branch, unchanged by this task), so it runs on every CI
// runner with no device, exactly like every other host-only suite in this
// tree.
// ---------------------------------------------------------------------------

namespace {

using spade::BodySpawn;
using spade::Result;
using spade::Simulation;
using spade::WorldInstanceDesc;
using spade::WorldRef;
using spade::WorldSetDesc;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::StepShape;
using spade::compute::VulkanBackend;
using spade::compute::vulkan_available;

// ---------------------------------------------------------------------------
// The gpu test fixture (this task's brief): wires compute::set_error_sink()
// to ADD_FAILURE() so a validation-layer message during any test in THIS
// file is a hard failure, and clears it again in TearDown so a later,
// unrelated test elsewhere in the same spade_tests process never inherits a
// stale callback. Every TEST_F below (device-executing or not) gets this --
// host-only tests never trigger it (nothing calls into Vulkan), so it costs
// them nothing.
// ---------------------------------------------------------------------------

void fail_on_validation_message(std::string_view message) noexcept {
    ADD_FAILURE() << "Vulkan validation message during a GpuStateMirror test: " << message;
}

class GpuStateMirrorTest : public ::testing::Test {
protected:
    void SetUp() override { spade::compute::set_error_sink(&fail_on_validation_message); }
    void TearDown() override { spade::compute::set_error_sink(nullptr); }
};

// ---------------------------------------------------------------------------
// A 4-world set from gate.world.yaml (T6-S5 API: sim/world_set.hpp's
// world_set_from()), the SAME construction test_world_file.cpp's
// WorldSetFrom.FourWorldFleetFromGateWorldCreatesAndStepsASimulation already
// exercises on the cpu path -- the prototype below matches its
// fleet_prototype() helper.
// ---------------------------------------------------------------------------
[[nodiscard]] Result<WorldSetDesc> gate_world_set(uint32_t count, uint64_t scene_seed) {
    const std::filesystem::path path = std::filesystem::path(SPADE_GOLDEN_DIR) / "worlds" / "gate.world.yaml";
    WorldInstanceDesc prototype;
    // TurbulenceLevel::none, and it STAYS none after S6 Task 8 -- deliberately,
    // not by omission.
    //
    // Task 6 set it to none (from ::light) because the vulkan backend then
    // refused a turbulent world outright: MediumUpdate was a stub and the gusts
    // would have been silently absent. Task 8 ports that pass and deletes the
    // refusal, so ::light is legal here again and the obvious move would be to
    // restore it. It is not restored, for a reason. Nothing in THIS file
    // asserts anything about turbulence -- these are mirror and recorder tests
    // -- while section 4's byte-comparison test DOES depend on knowing exactly
    // which arrays a step writes. A calm world keeps the OUTPUT gust exactly
    // zero (sigma * state) and every body's trajectory trivially reproducible,
    // while `dryden` still moves every substep (the five draws are
    // unconditional), so section 4 still proves MediumUpdate ran. The scenarios
    // that exercise a REAL gust are tests/test_gpu_parity.cpp's, which compare
    // it against the CPU instead of merely tolerating it.
    prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    prototype.contacts.restitution_e = 0.3f;
    prototype.contacts.friction_mu = 0.25f;
    prototype.contacts.proxy_radius = 0.1f;
    prototype.grid.cell_size = 0.5f;
    return spade::world_set_from(WorldRef{path}, count, scene_seed, prototype);
}

// NO RunParams ARGUMENT AT ANY VulkanBackend::create() CALL IN THIS FILE, as
// of S6 Task 6b: PassParams no longer carries a per-batch ContactParams/
// GridParams (RunParams's sole purpose), so create() no longer takes one --
// see compute/backend.hpp's doc comment on RunParams's removal. These tests
// exercise the MIRROR and the RECORDER, never a contact material, so this
// costs them nothing: the world set's own per-world contacts/grid (set via
// `prototype.contacts`/`prototype.grid` in gate_world_set() above) still
// reach the device through the per-world contact_params/grid_params buffers
// exactly as before. Leaving StepShape's two sdf_* counts at their 0
// defaults is the same reasoning as ever: these tests never upload an SDF
// program, and a zero-node program is exactly the empty world sdf.cpp's own
// eval() answers kSdfEmptyDistance for.
[[nodiscard]] StepShape shape_of(const Simulation& sim) {
    StepShape shape{};
    shape.world_count = sim.layout().world_count;
    shape.body_capacity = sim.layout().body_capacity;
    shape.element_capacity = sim.layout().element_capacity;
    shape.sensor_capacity = sim.layout().sensor_capacity;
    shape.substeps = sim.substeps();
    shape.h = sim.substep_h();
    shape.batch_dynamic_collision = sim.layout().uniform_dynamic_params;
    return shape;
}

}  // namespace

// ===========================================================================
// 1. Round trip: upload, readback, WITHOUT stepping.
// ===========================================================================

TEST_F(GpuStateMirrorTest, RoundTripUploadReadbackWithoutSteppingIsByteIdentical) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = gate_world_set(4, 0xA11CE5EEDULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;

    // A plain cpu-backend Simulation -- this test's only use for it is a
    // correctly-shaped, correctly-seeded ArenaSet to round-trip; the actual
    // upload()/readback() under test is driven directly against a
    // separately created VulkanBackend, not through Simulation::step().
    Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    Result<std::unique_ptr<VulkanBackend>> backend =
        VulkanBackend::create(BackendDesc{.kind = BackendKind::vulkan}, shape_of(*sim));
    ASSERT_TRUE(backend.has_value()) << backend.error().context;

    // "Every registered array" (18-entry walk: state/arenas.hpp's
    // ArenaSet::registry()) -- snapshot every byte BEFORE upload, per array.
    std::vector<std::pair<std::string, std::vector<std::byte>>> before;
    sim->arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
        before.emplace_back(array.name, std::vector<std::byte>(array.data, array.data + array.byte_size()));
    });
    ASSERT_EQ(before.size(), std::size_t{18}) << "registered walk drifted from the pinned 18 entries";

    ASSERT_TRUE((*backend)->upload(sim->arenas()).has_value());

    // Corrupt the LIVE cpu-side arenas so a trivial "readback wrote nothing,
    // bytes already matched" pass is impossible -- only a working
    // device->staging->host copy can restore them to `before`.
    //
    // const_cast justified: `sim` is a non-const local Simulation, so the
    // ArenaSet it owns is genuinely mutable; Simulation::arenas() (the only
    // accessor) deliberately returns a const view for every OTHER caller in
    // the tree, and this test needs to reach through that view to mutate
    // bytes for a reason no other caller has (proving a round trip, not
    // reading state). RegisteredArray::data is itself a mutable
    // std::byte* by design either way (state/registry.hpp's own note).
    spade::ArenaSet& mutable_arenas = const_cast<spade::ArenaSet&>(sim->arenas());  // NOLINT
    mutable_arenas.registry().for_each_array(
        [](const spade::RegisteredArray& array) { std::memset(array.data, 0xCD, array.byte_size()); });

    ASSERT_TRUE((*backend)->readback(mutable_arenas).has_value());

    std::size_t checked = 0;
    sim->arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
        const auto found =
            std::find_if(before.begin(), before.end(), [&](const auto& p) { return p.first == array.name; });
        ASSERT_NE(found, before.end()) << array.name;
        EXPECT_EQ(std::memcmp(array.data, found->second.data(), array.byte_size()), 0)
            << "round-trip mismatch for '" << array.name << "'";
        ++checked;
    });
    EXPECT_EQ(checked, std::size_t{18});
}

// ===========================================================================
// 2. Dirty tracking: a structural op between two step(1) calls forces a
//    re-upload.
// ===========================================================================

TEST_F(GpuStateMirrorTest, StructuralOpBetweenStepsForcesReupload) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = gate_world_set(4, 0xD1279101ULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;

    Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    EXPECT_EQ(sim->vulkan_upload_count(), 0u) << "no step() has run yet";

    ASSERT_TRUE(sim->step(1).has_value());
    const uint64_t after_first_step = sim->vulkan_upload_count();
    // The FIRST step always uploads (vulkan_dirty_ starts true -- create()'s
    // seeded arenas have never reached the device before this call).
    EXPECT_EQ(after_first_step, 1u);

    // A second step with NO structural op queued in between must NOT
    // re-upload -- the mirror already reflects arenas_ (this task's brief
    // asks specifically about a structural op forcing it; this is the
    // control case proving the counter is not simply incrementing every
    // step regardless).
    ASSERT_TRUE(sim->step(1).has_value());
    EXPECT_EQ(sim->vulkan_upload_count(), after_first_step) << "no structural op queued; upload_count must not move";

    // Now queue a spawn and step again -- flush_structural() applies it (a
    // real mutation to arenas_), which must mark the mirror dirty and force
    // the NEXT step()'s upload.
    ASSERT_TRUE(sim->spawn(0, BodySpawn{}).has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    EXPECT_EQ(sim->vulkan_upload_count(), after_first_step + 1)
        << "spawn between two step(1) calls must force exactly one re-upload";
}

// ===========================================================================
// 3. The per-step tick write (review round 1, C2): the host-side half of the
//    fix -- last_written_tick() reflects exactly the value the most recent
//    submit carried. GPU-visible consumption is deliberately out of scope
//    here; see this file's header note and StepParams's own doc comment
//    (compute/vulkan/step_recorder.hpp).
// ===========================================================================

TEST_F(GpuStateMirrorTest, StepParamsTickWrittenPerSubmit) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = gate_world_set(4, 0x71C4574EULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;

    Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    Result<std::unique_ptr<VulkanBackend>> backend =
        VulkanBackend::create(BackendDesc{.kind = BackendKind::vulkan}, shape_of(*sim));
    ASSERT_TRUE(backend.has_value()) << backend.error().context;

    EXPECT_EQ((*backend)->last_written_tick(), 0u) << "before the first step(), the buffer is still its zeroed init";

    // Two single-step submits with deliberately non-adjacent tick values --
    // proves the write is a genuine per-submit event carrying THIS call's
    // value, not a value baked in once at record/create() time (exactly the
    // failure mode a push constant baked into a record-once command buffer
    // would have produced, which is why this mechanism exists at all).
    ASSERT_TRUE((*backend)->step(1, 100).has_value());
    EXPECT_EQ((*backend)->last_written_tick(), 100u);

    ASSERT_TRUE((*backend)->step(1, 777).has_value());
    EXPECT_EQ((*backend)->last_written_tick(), 777u);

    // n > 1: submit()'s internal loop writes first_tick + step_index before
    // EACH of the n submits and fence-waits between them (StepRecorder::submit()'s
    // own comment), so by the time step() returns, exactly the LAST
    // iteration's value (first_tick + n - 1) is the one left resident --
    // this checks the loop lands on precisely that value for a multi-step
    // call, not merely that a single-step call works.
    ASSERT_TRUE((*backend)->step(3, 1000).has_value());
    EXPECT_EQ((*backend)->last_written_tick(), 1002u) << "expected first_tick(1000) + n(3) - 1";
}

// ===========================================================================
// 4. WHAT A VULKAN STEP MAY AND MAY NOT TOUCH (S6 Task 5 review round 2's
//    finding I2, rescoped by Task 6 and again by TASK 8 -- read this note
//    before concluding the test got weaker).
//
//    THE ASSERTION HAS ALWAYS BEEN "EXACTLY THE ARRAYS THE PORTED PASSES
//    WRITE, AND NOTHING ELSE"; what moves each wave is WHICH arrays those are.
//    Round 2 could demand byte identity of ALL EIGHTEEN walk entries, because
//    every schedule slot then bound the same no-op stub. Task 6 made `bodies`
//    the one exception (drag, CollisionStatic and Integrate write it). Task 8
//    adds a SECOND: `dryden`.
//
//    WHY `dryden` CHANGES EVEN THOUGH THIS WORLD SET IS CALM -- the part worth
//    stating rather than patching around. MediumUpdate advances the gust filter
//    once per world per substep UNCONDITIONALLY, five gaussians drawn whatever
//    the sigmas are, because world/medium.hpp requires the stream POSITION to
//    be a function of the substep count alone and not of the turbulence level.
//    So even at TurbulenceLevel::none the row's rng stream and its five
//    normalized states move every substep; only the OUTPUT gust, which is
//    sigma * state, stays exactly zero. A test expecting `dryden` to be
//    untouched here would be expecting the port to skip those draws, which is
//    precisely the divergence that discipline exists to prevent.
//
//    WHAT IT STILL PROVES, AND WHY THIS IS THE SHARPEST FORM AVAILABLE: sixteen
//    walk entries must be byte-identical after a step, so a mis-indexed write
//    into `drag_bodies`, `body_generation`, `imu_ring` or `replay_config` lands
//    here and nowhere else; the two permanently-inert slots (Gravity, Publish)
//    are shown to record nothing that touches state; and BOTH expected arrays
//    are asserted to have CHANGED, so the test cannot pass by the port silently
//    doing nothing. `rotors`, `imu_sensors` and `imu_ring` stay unchanged here
//    only because this world set spawns neither a rotor nor a sensor -- the
//    scenarios that do are tests/test_gpu_parity.cpp's, which compare those
//    arrays against the CPU rather than merely watching them move.
//
//    THE -0.0f PRECONDITION IS KEPT VERBATIM: it is the check that the spawn
//    path delivers exactly the bit pattern this test intends into the arena,
//    the same non-vacuity discipline every other assertion in this file
//    carries.
TEST_F(GpuStateMirrorTest, VulkanStepTouchesOnlyTheExpectedArrays) {
    if (!vulkan_available()) GTEST_SKIP();

    const Result<WorldSetDesc> set = gate_world_set(1, 0x0B17E5EEULL);
    ASSERT_TRUE(set.has_value()) << set.error().context;

    Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2, BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(sim.has_value()) << sim.error().context;

    // -0.0f on x and z (matching the axes five of the six golden worlds
    // give exactly-zero gravity -- the concrete scenario round 2's finding
    // names), and a NaN-free maximal-finite-magnitude value on y (FLT_MAX)
    // as a "far corner of the bit space" check that costs nothing extra to
    // include. Deliberately NOT an actual NaN encoding: a NaN's bit pattern
    // is not itself under test here (this kernel never performs floating-
    // point arithmetic on the value at all, so there is no computation that
    // could canonicalize a NaN payload the way IEEE-754 addition canonicalizes
    // a signed zero), and driving one through GTest's own floating-point-
    // aware assertion machinery elsewhere in this process is not worth the
    // risk for a property std::memcmp below already proves at the byte level.
    BodySpawn spawn{};
    spawn.vel = glm::vec3(-0.0f, std::numeric_limits<float>::max(), -0.0f);
    ASSERT_TRUE(sim->spawn(0, spawn).has_value());
    ASSERT_TRUE(sim->flush_structural().has_value());

    // Ground truth: every registered array's bytes AFTER the flush (the
    // spawn, including its -0.0f components, is now live) but BEFORE the
    // vulkan step() below ever runs the recorded chain for real.
    std::vector<std::pair<std::string, std::vector<std::byte>>> before;
    sim->arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
        before.emplace_back(array.name, std::vector<std::byte>(array.data, array.data + array.byte_size()));
    });
    ASSERT_EQ(before.size(), std::size_t{18});

    // PRECONDITION (S6 Task 5 review round 3): prove the -0.0f pattern this
    // test's whole premise rests on actually reached the arena, rather than
    // assuming the spawn path preserves it. Without this, a future spawn-path
    // refactor that canonicalizes velocity (e.g. a whole-object assignment
    // somewhere that happens to normalize signed zero) would silently
    // degrade this test to comparing +0.0f against +0.0f -- a vacuous pass
    // that proves nothing, exactly the failure mode this review loop has
    // been closing everywhere else (SlangSpirv.FloatControlsPinned's own
    // non-vacuity checks, the round-trip test's deliberate corruption step).
    // Slot 0 (world 0, local slot 0 -- the only body this test spawns) is
    // global arena slot 0, so its BodyState sits at byte 0 of the "bodies"
    // entry's raw bytes; reinterpreting is the same pattern
    // state/arenas.hpp's own array<T>() accessor uses internally.
    {
        const auto bodies_before =
            std::find_if(before.begin(), before.end(), [](const auto& p) { return p.first == "bodies"; });
        ASSERT_NE(bodies_before, before.end());
        ASSERT_GE(bodies_before->second.size(), sizeof(spade::BodyState));
        spade::BodyState body0{};
        std::memcpy(&body0, bodies_before->second.data(), sizeof(spade::BodyState));

        constexpr uint32_t kNegativeZeroBits = std::bit_cast<uint32_t>(-0.0f);
        static_assert(kNegativeZeroBits == 0x80000000u);
        ASSERT_EQ(std::bit_cast<uint32_t>(body0.vel.x), kNegativeZeroBits)
            << "test precondition failed: slot 0's vel.x did not arrive as -0.0f -- the spawn "
            << "path canonicalized it before this test ever reached the vulkan step, which "
            << "would make the byte-preservation check below vacuous (comparing +0 to +0)";
        ASSERT_EQ(std::bit_cast<uint32_t>(body0.vel.z), kNegativeZeroBits)
            << "test precondition failed: slot 0's vel.z did not arrive as -0.0f (see vel.x's "
            << "message above for why that matters)";
    }

    // ONE real step through the vulkan path: flush (no-op, queue already
    // drained above) -> upload (dirty from the flush above) -> the WHOLE
    // recorded per-substep chain runs for real, all eight schedule slots,
    // including the live body slot carrying -0.0f -> readback.
    ASSERT_TRUE(sim->step(1).has_value());

    // The arrays a step over THIS world set (no rotors, no sensors) is expected
    // to write. Both must actually move; see the header for why `dryden` is
    // among them even at TurbulenceLevel::none.
    const auto expected_to_change = [](const std::string& name) {
        return name == "bodies" || name == "dryden";
    };

    std::size_t unchanged = 0;
    std::size_t changed = 0;
    sim->arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
        const auto found =
            std::find_if(before.begin(), before.end(), [&](const auto& p) { return p.first == array.name; });
        ASSERT_NE(found, before.end()) << array.name;
        const bool same = std::memcmp(array.data, found->second.data(), array.byte_size()) == 0;
        if (expected_to_change(array.name)) {
            EXPECT_FALSE(same)
                << "a vulkan step left '" << array.name << "' byte-identical. For 'bodies' that means "
                << "Integrate did not run or the readback did not land; for 'dryden' it means "
                << "MediumUpdate skipped its five unconditional draws, which would put the two "
                << "backends' rng streams at different positions from the first substep on";
            ++changed;
            return;
        }
        EXPECT_TRUE(same)
            << "the ported kernels write 'bodies' and 'dryden' and nothing else for a world set with "
            << "no rotors and no sensors, but '" << array.name
            << "' changed across a vulkan step -- either an inert slot stopped being inert, or a "
            << "ported kernel wrote outside its own array";
        ++unchanged;
    });
    EXPECT_EQ(changed, std::size_t{2});
    EXPECT_EQ(unchanged, std::size_t{16});
    EXPECT_EQ(changed + unchanged, std::size_t{18}) << "registered walk drifted from the pinned 18 entries";
}

// ===========================================================================
// 5. Fault path: an absurd StepShape forces a real Vulkan allocation
//    failure, reported as a spade::Error, never a crash.
// ===========================================================================

TEST_F(GpuStateMirrorTest, AbsurdShapeAllocationFailureIsReportedNotCrashed) {
    if (!vulkan_available()) GTEST_SKIP();

    // world_count=1, body_capacity=UINT32_MAX: the `bodies` buffer alone
    // would be sizeof(BodyState) (128) * ~4.29e9 slots =~ 549 GB -- far past
    // any device-local (or host-visible) heap on this box or any real one,
    // without overflowing VkDeviceSize (uint64_t) the way world_count AND
    // body_capacity BOTH at UINT32_MAX would (documented, deliberately
    // avoided: an overflowed size could wrap to something SMALL and
    // spuriously succeed instead of failing the way this test needs it to).
    StepShape shape{};
    shape.world_count = 1;
    shape.body_capacity = 0xFFFFFFFFu;
    shape.element_capacity = 1;
    shape.sensor_capacity = 1;
    shape.substeps = 1;
    shape.h = 0.001f;
    shape.batch_dynamic_collision = false;

    const Result<std::unique_ptr<VulkanBackend>> backend =
        VulkanBackend::create(BackendDesc{.kind = BackendKind::vulkan}, shape);
    ASSERT_FALSE(backend.has_value()) << "an absurd shape must not silently succeed";
    // capacity_exceeded (VK_ERROR_OUT_OF_DEVICE_MEMORY/_HOST_MEMORY) is the
    // expected mapping (state_mirror.cpp's map_vk_error); `internal` is
    // accepted too -- both are Code values distinct from a crash, which is
    // the property this test actually needs, and a driver is free to report
    // a huge allocation's failure through either VkResult.
    EXPECT_TRUE(backend.error().code == spade::Code::capacity_exceeded ||
                backend.error().code == spade::Code::internal)
        << "unexpected code " << static_cast<int>(backend.error().code) << ": " << backend.error().context;
    std::cout << "[GpuStateMirror] absurd-shape failure: code=" << static_cast<int>(backend.error().code)
              << " context=" << backend.error().context << std::endl;
}

// ===========================================================================
// A7: BackendKind::cpu backend-knob invariance -- the full corpus, under
// backend={} (every pre-existing caller) vs an explicit
// BackendDesc{.kind = cpu}, byte-identical. Host-only, and the suite name
// deliberately does NOT start with "Gpu": this never touches Vulkan (the cpu
// path, unchanged by this task) and must not carry AppendSpadeLabels.cmake's
// "gpu" label or be mistaken for a device-executing/skippable test -- the
// same posture test_slang_layouts.cpp's own header documents for itself.
// ===========================================================================

namespace {

[[nodiscard]] std::filesystem::path scenario_dir() { return std::filesystem::path(SPADE_GOLDEN_DIR) / "scenarios"; }

[[nodiscard]] std::vector<spade::testing::LoadedScenario> corpus() {
    std::vector<std::filesystem::path> paths;
    std::error_code ec;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(scenario_dir(), ec)) {
        if (entry.is_regular_file() && entry.path().filename().string().ends_with(".scenario.yaml")) {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());

    std::vector<spade::testing::LoadedScenario> loaded;
    loaded.reserve(paths.size());
    for (const std::filesystem::path& path : paths) {
        const Result<spade::testing::LoadedScenario> scenario = spade::testing::scenario_from_yaml(path);
        if (!scenario) {
            ADD_FAILURE() << "failed to load " << path.string() << ": " << scenario.error().context;
            continue;
        }
        loaded.push_back(*scenario);
    }
    if (loaded.empty()) {
        ADD_FAILURE() << "the scenario corpus is empty: " << scenario_dir().string();
    }
    return loaded;
}

}  // namespace

TEST(BackendKnobInvariance, CpuBackendReproducesTodaysDigestsAcrossTheFullCorpus) {
    const std::vector<spade::testing::LoadedScenario> loaded = corpus();
    ASSERT_EQ(loaded.size(), std::size_t{5}) << "the golden corpus is expected to carry five scenarios";

    for (const spade::testing::LoadedScenario& entry : loaded) {
        const Result<uint64_t> implicit_digest = spade::testing::run_scenario(entry.scenario);
        ASSERT_TRUE(implicit_digest.has_value()) << entry.data->name << ": " << implicit_digest.error().context;

        const Result<uint64_t> explicit_cpu_digest =
            spade::testing::run_scenario(entry.scenario, BackendDesc{.kind = BackendKind::cpu});
        ASSERT_TRUE(explicit_cpu_digest.has_value())
            << entry.data->name << ": " << explicit_cpu_digest.error().context;

        EXPECT_EQ(*implicit_digest, *explicit_cpu_digest)
            << entry.data->name << ": backend={} and explicit BackendDesc{cpu} must share the exact code path";
        EXPECT_EQ(*implicit_digest, entry.data->expected_digest)
            << entry.data->name << ": this task must not move any committed golden digest";
    }
}
