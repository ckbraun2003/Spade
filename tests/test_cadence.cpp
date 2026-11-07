// test_cadence.cpp -- the cadence: PassDecl::every_steps, due(), SubstepContext::substep (module API stage 7, Task B2;
// docs/design/core/plans/2026-11-06-module-api-stage7-plan.md; design 4.2).
//
// A pass with every_steps = j > 1 runs in substep 0 of every j-th step. This file is the CPU half: the one rule, the
// compile-time refusal of two different j, the scheduler skipping by it, and a snapshot between fluid steps restoring
// onto the same cadence (Review Focus 3). The recorder's half (two recordings) is test_gpu_cadence.cpp.
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "physics/schedule.hpp"
#include "set_body_state_support.hpp"
#include "sim/cadence.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace {

using spade::modules::Access;
using spade::modules::compile_schedule;
using spade::modules::due;
using spade::modules::ModuleDesc;
using spade::modules::ModuleSet;
using spade::modules::PassDecl;
using spade::modules::Phase;
using spade::modules::QuantityAccess;
using spade::test::floor_world_set;

struct Seen {
    uint64_t tick = 0;
    uint32_t substep = 0;
    bool operator==(const Seen&) const = default;
};

// The passes record what they were called with. The scheduler calls a pass once per substep on one thread, so plain
// vectors do; a test clears them before it steps.
std::vector<Seen> g_each;  // every_steps = 1
std::vector<Seen> g_slow;  // every_steps = j

void each_pass(const spade::physics::SubstepContext& ctx) noexcept { g_each.push_back({ctx.tick.value, ctx.substep}); }
void slow_pass(const spade::physics::SubstepContext& ctx) noexcept { g_slow.push_back({ctx.tick.value, ctx.substep}); }

constexpr QuantityAccess kAccess[] = {{"body.pose", Access::read}};

// A module of two stateless passes, "each" (every substep) and "slow" (every `j`-th step), plus an optional third
// ("slower") with its own j: for the two-different-j refusal. Built in place, never copied: the spans point into it.
struct Fixture {
    std::array<PassDecl, 3> passes;
    ModuleSet set;

    Fixture(uint32_t j, uint32_t other_j = 0) {
        passes[0] = {.name = "each", .phase = Phase::sensors, .access = kAccess, .cpu = &each_pass, .every_steps = 1};
        passes[1] = {.name = "slow", .phase = Phase::sensors, .access = kAccess, .cpu = &slow_pass, .every_steps = j};
        passes[2] = {.name = "slower", .phase = Phase::sensors, .access = kAccess, .cpu = &slow_pass,
                     .every_steps = other_j};
        set = spade::modules::standard_modules();
        set.push_back(ModuleDesc{.name = "cadence_probe",
                                 .passes = std::span<const PassDecl>(passes.data(), other_j == 0 ? 2u : 3u)});
    }
    Fixture(const Fixture&) = delete;
    Fixture& operator=(const Fixture&) = delete;
};

[[nodiscard]] spade::Simulation make(const Fixture& f, uint32_t substeps) {
    const spade::Result<spade::WorldSetDesc> world = floor_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<spade::Simulation> sim =
        spade::Simulation::create(*world, 1'200'000, substeps, spade::compute::BackendDesc{}, f.set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

}  // namespace

// The one rule, as a table: j = 1 is always due; j > 1 is due in substep 0 of the steps whose tick is a multiple of j.
TEST(Cadence, DueIsTheOneRule) {
    struct Row {
        uint64_t tick;
        uint32_t substep;
        uint32_t j;
        bool expected;
    };
    constexpr Row kRows[] = {
        {0, 0, 1, true}, {0, 1, 1, true}, {7, 3, 1, true},                                            // j = 1
        {0, 0, 2, true}, {0, 1, 2, false}, {1, 0, 2, false}, {1, 1, 2, false}, {2, 0, 2, true},       // j = 2
        {3, 1, 2, false}, {4, 0, 2, true}, {5, 0, 2, false},
        {0, 0, 3, true}, {1, 0, 3, false}, {2, 0, 3, false}, {3, 0, 3, true}, {3, 1, 3, false},       // j = 3
        {6, 0, 3, true}, {7, 0, 3, false},
        {0xFFFFFFFFFFFFFFFEull, 0, 2, true}, {0xFFFFFFFFFFFFFFFFull, 0, 2, false},                      // the far end of a tick
    };
    for (const Row& r : kRows) {
        EXPECT_EQ(due(r.tick, r.substep, r.j), r.expected)
            << "tick " << r.tick << " substep " << r.substep << " every_steps " << r.j;
    }
}

TEST(Cadence, AScheduleWithTwoDifferentEveryStepsIsRefusedNamingBoth) {
    const Fixture f(2, 3);
    const auto compiled = compile_schedule(f.set);
    ASSERT_FALSE(compiled.has_value()) << "two different every_steps would need a third recording";
    const std::string& why = compiled.error().context;
    EXPECT_NE(why.find("every_steps"), std::string::npos) << why;
    EXPECT_NE(why.find("cadence_probe.slow"), std::string::npos) << why;
    EXPECT_NE(why.find("cadence_probe.slower"), std::string::npos) << why;
    EXPECT_NE(why.find('2'), std::string::npos) << why;
    EXPECT_NE(why.find('3'), std::string::npos) << why;
}

// A PIN, green at red (the refusal above is the only thing that can fail it): two passes sharing one j are one cadence.
TEST(Cadence, TwoPassesWithTheSameEveryStepsAreAccepted) {
    const Fixture f(2, 2);
    const auto compiled = compile_schedule(f.set);
    EXPECT_TRUE(compiled.has_value()) << compiled.error().context;
}

TEST(Cadence, AZeroEveryStepsIsRefused) {
    const Fixture f(0);
    const auto compiled = compile_schedule(f.set);
    ASSERT_FALSE(compiled.has_value());
    EXPECT_NE(compiled.error().context.find("every_steps"), std::string::npos) << compiled.error().context;
}

// The cadence is a different schedule: every_steps = 2 is not every_steps = 1, and a set that never uses it is the set it
// always was (the standard identity is pinned elsewhere; here the default and an explicit 1 agree, and 2 differs).
TEST(Cadence, EveryStepsEntersTheIdentityOnlyWhenItIsNotOne) {
    const Fixture one(1);
    const Fixture two(2);
    const auto a = compile_schedule(one.set);
    const auto b = compile_schedule(two.set);
    ASSERT_TRUE(a.has_value()) << a.error().context;
    ASSERT_TRUE(b.has_value()) << b.error().context;
    EXPECT_NE(a->identity, b->identity);
}

TEST(Cadence, TheCpuSkipsAPassOffItsTick) {
    const Fixture f(2);
    spade::Simulation sim = make(f, 2);
    g_each.clear();
    g_slow.clear();
    ASSERT_TRUE(sim.step(4).has_value());
    EXPECT_EQ(g_each.size(), 8u) << "the every-step pass runs in both substeps of all four steps";
    ASSERT_EQ(g_slow.size(), 2u) << "steps 0 and 2 only";
    EXPECT_EQ(g_slow[0].tick, 0u);
    EXPECT_EQ(g_slow[1].tick, 2u);
}

TEST(Cadence, TheCpuRunsItInSubstepZeroOnly) {
    const Fixture f(2);
    spade::Simulation sim = make(f, 3);
    g_each.clear();
    g_slow.clear();
    ASSERT_TRUE(sim.step(4).has_value());
    ASSERT_FALSE(g_slow.empty());
    for (const Seen& s : g_slow) EXPECT_EQ(s.substep, 0u) << "tick " << s.tick;
    // The ordinary pass sees the substep index it is run in, 0, 1, 2, 0, 1, 2, ...
    ASSERT_EQ(g_each.size(), 12u);
    for (std::size_t i = 0; i < g_each.size(); ++i) EXPECT_EQ(g_each[i].substep, i % 3) << i;
}

// Review Focus 3: a snapshot taken between fluid steps (j = 2, tick odd) restores onto the same cadence. The run that
// goes straight through and the run that stops at tick 3, snapshots, restores into a fresh Simulation and goes on must
// call the slow pass at the same ticks.
TEST(Cadence, ASnapshotBetweenFluidStepsRestoresOntoTheSameCadence) {
    const Fixture f(2);

    spade::Simulation straight = make(f, 2);
    g_slow.clear();
    ASSERT_TRUE(straight.step(6).has_value());
    const std::vector<Seen> expected = g_slow;
    ASSERT_EQ(expected.size(), 3u);  // ticks 0, 2, 4

    spade::Simulation first = make(f, 2);
    g_slow.clear();
    ASSERT_TRUE(first.step(3).has_value());  // ticks 0, 1, 2: the next step is tick 3, odd
    const auto blob = first.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    spade::Simulation second = make(f, 2);
    ASSERT_TRUE(second.restore(*blob).has_value());
    ASSERT_EQ(second.tick().value, 3u);
    ASSERT_TRUE(second.step(3).has_value());  // ticks 3, 4, 5
    EXPECT_EQ(g_slow, expected);
}
