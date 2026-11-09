// test_rebuild.cpp -- the rebuild hook, the host half (module API stage 7, Task C3; the stage 7 plan, C3).
//
// ModuleDesc::rebuild_cpu rebuilds what is DERIVED from a module's configuration and rows (not state, so a snapshot does not
// carry it) at create() and at restore(), before the next step. The device half is test_gpu_rebuild.cpp.
//
// Green at red (pins, named): a module with no hook is unchanged, and a hook is behaviour, not schedule, so it leaves the
// identity alone.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "rebuild_module_support.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"

namespace {

using spade::test::DeriveModule;
using spade::test::g_rebuild_calls;
using spade::test::g_rebuild_derived;
using spade::test::kDeriveConfig;

[[nodiscard]] spade::Simulation make(const spade::modules::ModuleSet& set) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::two_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<spade::Simulation> sim = spade::Simulation::create(*world, 1'000'000, 2, spade::compute::BackendDesc{}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

}  // namespace

TEST(Rebuild, RunsAtCreateAndAtRestoreBeforeTheNextStep) {
    const DeriveModule m;
    g_rebuild_calls = 0;
    spade::Simulation a = make(m.set);
    EXPECT_EQ(g_rebuild_calls, 1) << "once at create(), one call for the module whatever the world count";
    EXPECT_EQ(g_rebuild_derived[0], kDeriveConfig.k) << "region 0 at create: derived = 0 * 2 + k";

    ASSERT_TRUE(a.step(2).has_value());  // two steps x two substeps: region 4
    const auto blob = a.snapshot();
    ASSERT_TRUE(blob.has_value());

    spade::Simulation b = make(m.set);
    g_rebuild_calls = 0;
    ASSERT_TRUE(b.restore(*blob).has_value());
    EXPECT_EQ(g_rebuild_calls, 1) << "once at restore(), before any step";
    EXPECT_EQ(g_rebuild_derived[0], 4u * 2u + kDeriveConfig.k) << "the hook saw the RESTORED region row";
}

TEST(Rebuild, ARestoreReplacesTheRegionRowAndTheDerivedBufferFollows) {
    const DeriveModule m;
    spade::Simulation a = make(m.set);
    ASSERT_TRUE(a.step(1).has_value());  // region 2
    const auto blob = a.snapshot();
    ASSERT_TRUE(blob.has_value());

    spade::Simulation b = make(m.set);
    ASSERT_TRUE(b.step(3).has_value());  // region 6
    ASSERT_TRUE(b.restore(*blob).has_value());
    EXPECT_EQ(g_rebuild_derived[0], 2u * 2u + kDeriveConfig.k) << "derived follows the replaced row, not b's old one";
    EXPECT_EQ(g_rebuild_derived[1], 2u * 2u + kDeriveConfig.k);
}

TEST(Rebuild, ARefusedRestoreRunsNoRebuild) {
    const DeriveModule m;
    spade::Simulation a = make(m.set);
    ASSERT_TRUE(a.step(1).has_value());
    auto blob = a.snapshot();
    ASSERT_TRUE(blob.has_value());
    spade::Simulation other = make(spade::modules::standard_modules());  // another schedule identity
    g_rebuild_calls = 0;
    EXPECT_FALSE(other.restore(*blob).has_value());
    EXPECT_EQ(g_rebuild_calls, 0);
}

// PIN, green at red: a module with no hook is unchanged, and the standard set has none.
TEST(Rebuild, AModuleWithNoHookIsUnchanged) {
    const auto standard = spade::modules::compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(standard.has_value());
    EXPECT_EQ(standard->identity, 0x3303cfc86821f502ULL);

    const DeriveModule hooked(true);
    const DeriveModule plain(false);
    EXPECT_EQ(spade::modules::compile_schedule(hooked.set)->identity, spade::modules::compile_schedule(plain.set)->identity)
        << "a hook is behaviour, not schedule: the identity spells names, versions, order and images";
    g_rebuild_calls = 0;
    spade::Simulation sim = make(plain.set);
    ASSERT_TRUE(sim.step(2).has_value());
    EXPECT_EQ(g_rebuild_calls, 0);
}

TEST(Rebuild, ARebuildAccessNamingNothingIsRefusedNamingTheModule) {
    static const spade::modules::QuantityAccess missing[] = {{"derive.no_such_array", spade::modules::Access::read}};
    DeriveModule m;
    m.set.back().rebuild_access = missing;
    const auto refused = spade::modules::compile_schedule(m.set);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("derive"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("no_such_array"), std::string::npos) << refused.error().context;
}

TEST(Rebuild, AHookNeedsItsAccessListAndTheOtherWayRound) {
    DeriveModule m;
    m.set.back().rebuild_access = {};  // a hook that sees nothing
    EXPECT_FALSE(spade::modules::compile_schedule(m.set).has_value()) << "rebuild_cpu with no declared access";

    DeriveModule n(false);
    n.set.back().rebuild_access = spade::test::kDeriveRebuildAccess;  // an access list with no hook
    const auto refused = spade::modules::compile_schedule(n.set);
    ASSERT_FALSE(refused.has_value()) << "rebuild_access with neither hook";
    EXPECT_NE(refused.error().context.find("derive"), std::string::npos) << refused.error().context;
}
