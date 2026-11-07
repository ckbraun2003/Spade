// test_gpu_cadence.cpp -- the recorder's half of the cadence (module API stage 7, Task B2; design 4.2): a pass with
// PassDecl::every_steps = j > 1 lives in one of TWO recordings, both recorded once at create(), and submit() picks one per
// step by (tick mod j) -- a function of the tick alone, so a restore lands on the same cadence. The CPU half and the rule are
// test_cadence.cpp. The test module is probe_module_support.hpp's: one pass with its own recipe, dispatching a kernel that
// does nothing, so the recordings, their barriers and their timing are what is observed. Every test skips with its reason
// when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "compute/backend.hpp"
#include "gpu_skip.hpp"
#include "probe_module_support.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace {

using spade::Simulation;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::RecordedChain;
using spade::test::floor_world_set;
using spade::test::ProbeModule;

constexpr uint32_t kSubsteps = 2;
constexpr uint32_t kJ = 2;

[[nodiscard]] Simulation make(const spade::modules::ModuleSet& set) {
    const spade::Result<spade::WorldSetDesc> world = floor_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<Simulation> sim =
        Simulation::create(*world, 1'000'000, kSubsteps, BackendDesc{.kind = BackendKind::vulkan}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

[[nodiscard]] bool holds(const RecordedChain& chain, const std::string& pass) {
    for (const std::string& p : chain.passes) {
        if (p == pass) return true;
    }
    return false;
}

}  // namespace

// A PIN, green at red: a set that never uses a cadence has one recording and today's dispatch count.
TEST(GpuCadence, TheStandardSetHasOneRecordingAndTheSameDispatchCount) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation sim = make(spade::modules::standard_modules());
    const auto chains = sim.vulkan_recorded_chains();
    ASSERT_TRUE(chains.has_value()) << chains.error().context;
    ASSERT_EQ(chains->size(), 1u);
    EXPECT_EQ((*chains)[0].dispatches, kSubsteps * (11u + (*chains)[0].sort_stages));
}

// Two recordings: the step every pass is due in, and the step the slow pass is not. The slow pass runs in substep 0 only,
// so the due recording holds exactly one dispatch more.
TEST(GpuCadence, ACadenceHasTwoRecordingsAndTheSlowPassIsOnlyInTheFirst) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ProbeModule probe(kJ);
    Simulation sim = make(probe.set);
    const auto chains = sim.vulkan_recorded_chains();
    ASSERT_TRUE(chains.has_value()) << chains.error().context;
    ASSERT_EQ(chains->size(), 2u);
    EXPECT_TRUE(holds((*chains)[0], "probe.step"));
    EXPECT_FALSE(holds((*chains)[1], "probe.step"));
    EXPECT_EQ((*chains)[0].dispatches, (*chains)[1].dispatches + 1u) << "substep 0 only";
}

// CORE-2 on both recordings: a barrier between every adjacent pair of dispatches, none after the last.
TEST(GpuCadence, TheCore2BarrierTestPassesOnBothRecordings) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ProbeModule probe(kJ);
    Simulation sim = make(probe.set);
    const auto chains = sim.vulkan_recorded_chains();
    ASSERT_TRUE(chains.has_value()) << chains.error().context;
    ASSERT_EQ(chains->size(), 2u);
    for (std::size_t i = 0; i < chains->size(); ++i) {
        EXPECT_EQ((*chains)[i].barriers, (*chains)[i].dispatches - 1u) << "recording " << i;
    }
}

// The recorded order on both recordings is the schedule's, with the slow pass absent from the second.
TEST(GpuCadence, TheRecordedOrderHoldsOnBothRecordings) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ProbeModule probe(kJ);
    Simulation sim = make(probe.set);
    const auto chains = sim.vulkan_recorded_chains();
    ASSERT_TRUE(chains.has_value()) << chains.error().context;
    ASSERT_EQ(chains->size(), 2u);
    std::vector<std::string> due = (*chains)[0].passes;
    std::erase(due, "probe.step");
    EXPECT_EQ(due, (*chains)[1].passes) << "the second recording is the first without the slow pass, in the same order";
}

// Timing names the recording that ran and the tick: step 0 ran recording 0, step 1 recording 1.
TEST(GpuCadence, TimingNamesTheRecordingThatRan) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ProbeModule probe(kJ);
    Simulation sim = make(probe.set);

    ASSERT_TRUE(sim.step(1).has_value());  // tick 0: every pass is due
    auto first = sim.vulkan_pass_durations_ns();
    ASSERT_TRUE(first.has_value()) << first.error().context;
    if (!first->supported) GTEST_SKIP() << "this device cannot time compute work";
    EXPECT_EQ(first->recording, 0u);
    EXPECT_EQ(first->tick, 0u);
    EXPECT_TRUE(first->find("probe.step").has_value());

    ASSERT_TRUE(sim.step(1).has_value());  // tick 1: the slow pass is not due
    auto second = sim.vulkan_pass_durations_ns();
    ASSERT_TRUE(second.has_value()) << second.error().context;
    EXPECT_EQ(second->recording, 1u);
    EXPECT_EQ(second->tick, 1u);
}

// A pass the recording that ran does not hold has no entry: it is never read as a measured zero.
TEST(GpuCadence, ASkippedStepHasNoFluidTiming) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ProbeModule probe(kJ);
    Simulation sim = make(probe.set);
    ASSERT_TRUE(sim.step(2).has_value());  // ticks 0 and 1: the last step is the one without the slow pass
    auto d = sim.vulkan_pass_durations_ns();
    ASSERT_TRUE(d.has_value()) << d.error().context;
    if (!d->supported) GTEST_SKIP() << "this device cannot time compute work";
    EXPECT_FALSE(d->find("probe.step").has_value()) << "an absent pass must not read as a zero duration";
    EXPECT_TRUE(d->find("integrate.integrate").has_value()) << "the passes the recording holds are still timed";
}

// Review Focus 3, on the device: a snapshot between slow steps (tick odd) restores onto the same recording.
TEST(GpuCadence, ASnapshotBetweenFluidStepsRestoresOntoTheSameRecording) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ProbeModule probe(kJ);
    Simulation first = make(probe.set);
    ASSERT_TRUE(first.step(3).has_value());  // ticks 0, 1, 2: next is tick 3, odd, the recording without the slow pass
    const auto blob = first.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    Simulation second = make(probe.set);
    ASSERT_TRUE(second.restore(*blob).has_value());
    ASSERT_TRUE(second.step(1).has_value());  // tick 3
    auto d = second.vulkan_pass_durations_ns();
    ASSERT_TRUE(d.has_value()) << d.error().context;
    if (!d->supported) GTEST_SKIP() << "this device cannot time compute work";
    EXPECT_EQ(d->tick, 3u);
    EXPECT_EQ(d->recording, 1u) << "the choice is a function of the tick alone";
}
