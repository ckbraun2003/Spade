// test_module_schedule.cpp -- the module API's schedule compiler
// (docs/design/core/plans/2026-10-02-module-api-design.md, section 4).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "compute/backend.hpp"
#include "core/rng.hpp"
#include "physics/schedule.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace {

using spade::modules::Access;
using spade::modules::CompiledSchedule;
using spade::modules::compile_schedule;
using spade::modules::ModuleDesc;
using spade::modules::PassDecl;
using spade::modules::Phase;
using spade::modules::Placement;
using spade::modules::QuantityAccess;

void noop(const spade::physics::SubstepContext&) noexcept {}

// An attached array's init that writes nothing: for fixtures that test an
// array's shape, where only its presence matters.
void noop_row(const spade::modules::RowInit&) noexcept {}

using Names = std::vector<std::string>;

[[nodiscard]] Names names(const CompiledSchedule& s) {
    Names out;
    for (const auto& p : s.passes) out.push_back(std::string(p.module) + "." + std::string(p.pass));
    return out;
}

[[nodiscard]] spade::Code code_of(const spade::Result<CompiledSchedule>& r) {
    return r.has_value() ? spade::Code::internal : r.error().code;
}

}  // namespace

TEST(ModuleSchedule, PhasesRunInOrderWhateverTheSetOrder) {
    static constexpr PassDecl late[] = {{.name = "late", .phase = Phase::sensors, .cpu = &noop}};
    static constexpr PassDecl early[] = {{.name = "early", .phase = Phase::fields, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = late}, {.name = "b", .passes = early}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"b.early", "a.late"}));
}

TEST(ModuleSchedule, AReaderRunsAfterAWriterOfTheSameQuantity) {
    static constexpr QuantityAccess reads[] = {{"w.x", Access::read}};
    static constexpr QuantityAccess writes[] = {{"w.x", Access::write}};
    static constexpr PassDecl reader[] = {{.name = "read", .phase = Phase::forces, .access = reads, .cpu = &noop}};
    static constexpr PassDecl writer[] = {{.name = "write", .phase = Phase::forces, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "r", .passes = reader}, {.name = "w", .passes = writer}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"w.write", "r.read"}));
}

TEST(ModuleSchedule, AccumulatorsNeedNoEdgeAndFollowSetOrder) {
    static constexpr QuantityAccess adds[] = {{"body.wrench", Access::accumulate}};
    static constexpr PassDecl p[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    static constexpr PassDecl q[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "p", .passes = p}, {.name = "q", .passes = q}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"p.add", "q.add"}));
}

TEST(ModuleSchedule, AnEdgeOrdersAccumulatorsAgainstSetOrder) {
    static constexpr QuantityAccess adds[] = {{"body.wrench", Access::accumulate}};
    static constexpr std::string_view after_q[] = {"q.add"};
    static constexpr PassDecl p[] = {
        {.name = "add", .phase = Phase::forces, .access = adds, .after = after_q, .cpu = &noop}};
    static constexpr PassDecl q[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "p", .passes = p}, {.name = "q", .passes = q}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"q.add", "p.add"}));
}

TEST(ModuleSchedule, TwoWritersWithoutAnEdgeAreRefused) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr PassDecl a[] = {{.name = "fix", .phase = Phase::constraints, .access = writes, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "fix", .phase = Phase::constraints, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, AWriterAndAnAccumulatorWithoutAnEdgeAreRefused) {
    static constexpr QuantityAccess writes[] = {{"body.wrench", Access::write}};
    static constexpr QuantityAccess adds[] = {{"body.wrench", Access::accumulate}};
    static constexpr PassDecl a[] = {{.name = "set", .phase = Phase::forces, .access = writes, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, TwoWritersWithAnEdgeCompileInEdgeOrder) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr std::string_view after_b[] = {"b.fix"};
    static constexpr PassDecl a[] = {
        {.name = "fix", .phase = Phase::constraints, .access = writes, .after = after_b, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "fix", .phase = Phase::constraints, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"b.fix", "a.fix"}));
}

TEST(ModuleSchedule, ACycleIsRefused) {
    static constexpr std::string_view after_b[] = {"b.two"};
    static constexpr std::string_view after_a[] = {"a.one"};
    static constexpr PassDecl a[] = {{.name = "one", .phase = Phase::forces, .after = after_b, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "two", .phase = Phase::forces, .after = after_a, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, EdgesMustNameAPassInThisOrAnEarlierPhase) {
    static constexpr std::string_view to_nobody[] = {"nobody.here"};
    static constexpr std::string_view to_late[] = {"b.late"};
    static constexpr std::string_view to_early[] = {"b.early"};
    static constexpr PassDecl dangling[] = {{.name = "x", .phase = Phase::forces, .after = to_nobody, .cpu = &noop}};
    static constexpr PassDecl into_later[] = {{.name = "x", .phase = Phase::forces, .after = to_late, .cpu = &noop}};
    static constexpr PassDecl into_earlier[] = {{.name = "x", .phase = Phase::forces, .after = to_early, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "late", .phase = Phase::sensors, .cpu = &noop},
                                     {.name = "early", .phase = Phase::fields, .cpu = &noop}};
    const ModuleDesc s1[] = {{.name = "a", .passes = dangling}, {.name = "b", .passes = b}};
    const ModuleDesc s2[] = {{.name = "a", .passes = into_later}, {.name = "b", .passes = b}};
    const ModuleDesc s3[] = {{.name = "a", .passes = into_earlier}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(s1)), spade::Code::invalid_argument);
    EXPECT_EQ(code_of(compile_schedule(s2)), spade::Code::invalid_argument);
    EXPECT_TRUE(compile_schedule(s3).has_value()) << "an edge to an earlier phase is already satisfied";
}

TEST(ModuleSchedule, FirstAndLastPlacementBracketTheOrderedPasses) {
    static constexpr PassDecl m[] = {
        {.name = "mid", .phase = Phase::forces, .cpu = &noop},
        {.name = "end", .phase = Phase::forces, .placement = Placement::last, .cpu = &noop},
        {.name = "top", .phase = Phase::forces, .placement = Placement::first, .cpu = &noop},
    };
    const ModuleDesc set[] = {{.name = "m", .passes = m}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"m.top", "m.mid", "m.end"}));
}

TEST(ModuleSchedule, TwoConflictingPassesOfOnePlacementAreRefused) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr PassDecl a[] = {
        {.name = "pin", .phase = Phase::fields, .placement = Placement::first, .access = writes, .cpu = &noop}};
    static constexpr PassDecl b[] = {
        {.name = "pin", .phase = Phase::fields, .placement = Placement::first, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, PlacementOrdersAPlacedWriterBeforeAnOrderedWriter) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr PassDecl ordered[] = {{.name = "move", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    static constexpr PassDecl placed[] = {
        {.name = "pin", .phase = Phase::fields, .placement = Placement::first, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "o", .passes = ordered}, {.name = "p", .passes = placed}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"p.pin", "o.move"}));
}

// Placement orders a pass against every pass of another placement in its
// phase, so a last-placed WRITER (a force behavior that sets force_acc) after
// ordered ACCUMULATORS needs no edge.
TEST(ModuleSchedule, ALastPlacedWriterFollowsOrderedAccumulators) {
    static constexpr QuantityAccess adds[] = {{"body.wrench", Access::accumulate}};
    static constexpr QuantityAccess sets[] = {{"body.wrench", Access::write}};
    static constexpr PassDecl acc[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    static constexpr PassDecl last[] = {
        {.name = "set", .phase = Phase::forces, .placement = Placement::last, .access = sets, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "l", .passes = last}, {.name = "a", .passes = acc}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"a.add", "l.set"}));
}

TEST(ModuleSchedule, APlacementThatContradictsAHazardIsRefused) {
    // A first-placed READER and an ordered WRITER of one quantity: the hazard
    // wants the writer first, the placement wants the reader first.
    static constexpr QuantityAccess reads[] = {{"body.pose", Access::read}};
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr PassDecl r[] = {
        {.name = "look", .phase = Phase::fields, .placement = Placement::first, .access = reads, .cpu = &noop}};
    static constexpr PassDecl w[] = {{.name = "move", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "r", .passes = r}, {.name = "w", .passes = w}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, UnknownQuantitiesAreRefused) {
    static constexpr QuantityAccess misspelt[] = {{"body.wrnch", Access::accumulate}};
    static constexpr QuantityAccess no_owner[] = {{"ghost.x", Access::read}};
    static constexpr PassDecl a[] = {{.name = "x", .phase = Phase::forces, .access = misspelt, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "x", .phase = Phase::forces, .access = no_owner, .cpu = &noop}};
    const ModuleDesc s1[] = {{.name = "a", .passes = a}};
    const ModuleDesc s2[] = {{.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(s1)), spade::Code::invalid_argument);
    EXPECT_EQ(code_of(compile_schedule(s2)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, MalformedSetsAreRefused) {
    static constexpr PassDecl ok[] = {{.name = "x", .phase = Phase::forces, .cpu = &noop}};
    static constexpr PassDecl no_cpu[] = {{.name = "x", .phase = Phase::forces}};
    const ModuleDesc twice[] = {{.name = "a", .passes = ok}, {.name = "a", .passes = ok}};
    const ModuleDesc dotted[] = {{.name = "a.b", .passes = ok}};
    const ModuleDesc null_fn[] = {{.name = "a", .passes = no_cpu}};
    EXPECT_EQ(code_of(compile_schedule(twice)), spade::Code::invalid_argument);
    EXPECT_EQ(code_of(compile_schedule(dotted)), spade::Code::invalid_argument);
    EXPECT_EQ(code_of(compile_schedule(null_fn)), spade::Code::invalid_argument);
}

// The identity is spelt byte by byte (module name, 0x00, version as 4 bytes
// little-endian; then 0x01; then per compiled pass: module, '.', pass, 0x00,
// phase byte), FNV-1a 64 over the result. The constant below was computed by
// an independent Python implementation of that spelling, so a platform- or
// container-dependent compiler fails here.
TEST(ModuleSchedule, IdentityIsSpeltOutAndChangesWithVersionOrOrder) {
    static constexpr PassDecl one[] = {{.name = "one", .phase = Phase::forces, .cpu = &noop}};
    static constexpr PassDecl two[] = {{.name = "two", .phase = Phase::fields, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "alpha", .version = 1, .passes = one},
                              {.name = "beta", .version = 2, .passes = two}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(s->identity, 0x2a62d4b78a25f25aULL);

    const ModuleDesc bumped[] = {{.name = "alpha", .version = 1, .passes = one},
                                 {.name = "beta", .version = 3, .passes = two}};
    const ModuleDesc swapped[] = {{.name = "beta", .version = 2, .passes = two},
                                  {.name = "alpha", .version = 1, .passes = one}};
    EXPECT_NE(compile_schedule(bumped)->identity, s->identity);
    EXPECT_NE(compile_schedule(swapped)->identity, s->identity);
}

// Spec section 4's table: the standard set compiles to today's order, with the
// two documented changes (kinematic behaviors before Dryden; the empty
// Gravity and Publish passes gone).
TEST(StandardModules, CompileToTodaysOrder) {
    const spade::modules::ModuleSet set = spade::modules::standard_modules();
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"behaviors.kinematic", "dryden.advance", "dryden.sample", "environment.sample",
                                "rotor.forces", "drag.forces", "behaviors.force", "static_contact.resolve",
                                "dynamic_contact.resolve", "integrate.integrate", "imu.synthesize",
                                "gnss.synthesize"}));
}

// Without drag's edge, set order (drag before rotor: the golden walk's order)
// would run drag first and move every golden with a rotor. The edge is
// load-bearing, so dropping it must change both the order and the identity.
TEST(StandardModules, DroppingDragsEdgeChangesTheOrderAndTheIdentity) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    const auto with_edge = compile_schedule(set);
    ASSERT_TRUE(with_edge.has_value()) << with_edge.error().context;

    static constexpr QuantityAccess drag_access[] = {{"body.pose", Access::read},
                                                     {"body.wrench", Access::accumulate},
                                                     {"dryden.dryden", Access::read}};
    static constexpr PassDecl drag_no_edge[] = {
        {.name = "forces", .phase = Phase::forces, .access = drag_access, .cpu = &spade::physics::pass_drag}};
    ASSERT_EQ(set[0].name, "drag");
    set[0].passes = drag_no_edge;

    const auto without = compile_schedule(set);
    ASSERT_TRUE(without.has_value()) << without.error().context;
    // After the four Fields passes (behaviors.kinematic, dryden.advance,
    // dryden.sample, environment.sample).
    EXPECT_EQ(names(*without)[4], "drag.forces");
    EXPECT_EQ(names(*without)[5], "rotor.forces");
    EXPECT_NE(without->identity, with_edge->identity);
}

namespace {

// A developer module: one pass that holds body 0 of every world up against
// gravity by adding m*(-g) to force_acc. Mass 2 (a power of two) makes the
// cancellation in Integrate exact.
void hover_pass(const spade::physics::SubstepContext& ctx) noexcept {
    for (const spade::physics::WorldSubstepView& w : ctx.worlds) {
        spade::BodyState& b = w.bodies[0];
        b.force_acc += -b.mass * w.params->gravity;
    }
}
constexpr QuantityAccess kHoverAccess[] = {{"body.wrench", Access::accumulate}};
constexpr PassDecl kHoverPasses[] = {
    {.name = "lift", .phase = Phase::forces, .access = kHoverAccess, .cpu = &hover_pass}};

[[nodiscard]] spade::WorldSetDesc one_body_world() {
    auto world = spade::WorldBuilder()
                     .name("m")
                     .environment(spade::Environment{})
                     .capacities(spade::Capacities{1, 1, 1, 1})
                     .build();
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::WorldSetDesc{{inst}};
}

}  // namespace

TEST(ModuleSimulation, TheDefaultSetIsTheStandardSet) {
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto standard = compile_schedule(spade::modules::standard_modules());
    EXPECT_EQ(sim->schedule().identity, standard->identity);
}

TEST(ModuleSimulation, ADeveloperModulePlugsInOnTheCpu) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "hover", .passes = kHoverPasses});
    auto held = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    auto falling = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(held.has_value()) << held.error().context;
    ASSERT_TRUE(falling.has_value()) << falling.error().context;
    spade::BodySpawn body;
    body.mass = 2.0f;
    const auto held_ref = held->spawn(0, body);
    const auto falling_ref = falling->spawn(0, body);
    ASSERT_TRUE(held_ref.has_value() && falling_ref.has_value());
    ASSERT_TRUE(held->step(10).has_value());
    ASSERT_TRUE(falling->step(10).has_value());
    const auto h = held->body(*held_ref);
    const auto f = falling->body(*falling_ref);
    ASSERT_TRUE(h.has_value() && f.has_value());
    EXPECT_EQ((*h)->vel, glm::vec3(0.0f)) << "the module's lift cancels gravity exactly";
    EXPECT_LT((*f)->vel.y, 0.0f) << "without the module the body falls";
}

TEST(ModuleSimulation, ScheduleNamesOutliveTheStringsThatNamedThem) {
    std::string name = "hover";
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = name, .passes = kHoverPasses});
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    name.assign("xxxxx");  // a view into `name` would now read "xxxxx"
    bool found = false;
    for (const auto& pass : sim->schedule().passes) found = found || pass.module == "hover";
    EXPECT_TRUE(found) << "the compiled schedule must own its module names";
}

// The identity hashes names, versions and order, not function pointers, so a
// set that keeps the standard names but swaps in another rotor function has the
// standard identity. The GPU must not run the stock rotor kernel in its place
// (L6): a pass without a recipe has no kernel, and the recipe can only be
// claimed with the built-in function. Found by the stage-1 whole-branch review.
TEST(ModuleSimulation, AStandardNamedSetWithAnotherFunctionIsRefusedOnVulkan) {
    static constexpr QuantityAccess rotor_access[] = {{"body.pose", Access::read},
                                                      {"body.wrench", Access::accumulate},
                                                      {"rotor.rotors", Access::write},
                                                      {"dryden.dryden", Access::read}};
    static constexpr PassDecl my_rotor[] = {
        {.name = "forces", .phase = Phase::forces, .access = rotor_access, .cpu = &hover_pass}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    for (spade::modules::ModuleDesc& m : set) {
        if (m.name == "rotor") m.passes = my_rotor;
    }
    ASSERT_EQ(compile_schedule(set)->identity, compile_schedule(spade::modules::standard_modules())->identity)
        << "the precondition: the identity cannot see the substituted function";

    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2,
                                               spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan},
                                               set);
    ASSERT_FALSE(sim.has_value()) << "a substituted pass must not run the stock GPU kernel silently";
    EXPECT_EQ(sim.error().code, spade::Code::unavailable);
    EXPECT_NE(sim.error().context.find("module set"), std::string::npos) << sim.error().context;
}

TEST(ModuleSimulation, AnInvalidSetIsRefusedAtCreate) {
    static constexpr QuantityAccess misspelt[] = {{"body.wrnch", Access::accumulate}};
    static constexpr PassDecl bad[] = {{.name = "x", .phase = Phase::forces, .access = misspelt, .cpu = &noop}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "bad", .passes = bad});
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_FALSE(sim.has_value());
    EXPECT_EQ(sim.error().code, spade::Code::invalid_argument);
}

TEST(ModuleSnapshot, ABlobCarriesTheScheduleIdentity) {
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto blob = sim->snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    EXPECT_EQ(blob->configuration_identity(), sim->schedule().identity);
}

TEST(ModuleSnapshot, RestoreIntoAnotherModuleSetIsRefused) {
    spade::modules::ModuleSet with_hover = spade::modules::standard_modules();
    with_hover.push_back({.name = "hover", .passes = kHoverPasses});
    auto source = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, with_hover);
    auto target = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(source.has_value() && target.has_value());
    const auto blob = source->snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    const auto restored = target->restore(*blob);
    ASSERT_FALSE(restored.has_value());
    EXPECT_EQ(restored.error().code, spade::Code::invalid_argument);
    EXPECT_NE(restored.error().context.find("module set"), std::string::npos) << restored.error().context;
    // Both identities in hex, as every other 64-bit identity in this tree is
    // written (stage 4, Task 2; the stage-1 review's minor).
    for (const uint64_t id : {blob->configuration_identity(), target->schedule().identity}) {
        std::array<char, 24> hex{};
        std::snprintf(hex.data(), hex.size(), "0x%016llx", static_cast<unsigned long long>(id));
        EXPECT_NE(restored.error().context.find(hex.data()), std::string::npos)
            << hex.data() << " in: " << restored.error().context;
    }
}

TEST(ModuleSnapshot, RestoreUnderAnotherModuleVersionIsRefused) {
    spade::modules::ModuleSet a = spade::modules::standard_modules();
    spade::modules::ModuleSet b = spade::modules::standard_modules();
    a.push_back({.name = "hover", .version = 1, .passes = kHoverPasses});
    b.push_back({.name = "hover", .version = 2, .passes = kHoverPasses});
    auto source = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, a);
    auto target = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, b);
    ASSERT_TRUE(source.has_value() && target.has_value());
    const auto blob = source->snapshot();
    ASSERT_TRUE(blob.has_value());
    EXPECT_FALSE(target->restore(*blob).has_value());
}

TEST(ModuleSnapshot, RestoreIntoTheSameSetSucceeds) {
    auto source = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    auto target = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(source.has_value() && target.has_value());
    ASSERT_TRUE(source->step(3).has_value());
    const auto blob = source->snapshot();
    ASSERT_TRUE(blob.has_value());
    EXPECT_TRUE(target->restore(*blob).has_value());
}

// Stage 2: a pass names the built-in GPU kernel (its recipe) that does on the
// GPU what its CPU function does. The recipe is honest only with that function.
TEST(StandardModules, EveryPassNamesARecipePairedWithItsOwnFunction) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    for (const auto& p : s->passes) {
        EXPECT_NE(p.gpu, spade::compute::GpuRecipe::none) << p.module << "." << p.pass;
        EXPECT_EQ(p.cpu, spade::modules::builtin_cpu_for(p.gpu)) << p.module << "." << p.pass;
    }
}

TEST(ModuleSchedule, ARecipeWithAnotherCpuFunctionIsRefused) {
    static constexpr PassDecl liar[] = {{.name = "lift", .phase = Phase::forces, .access = kHoverAccess,
                                         .cpu = &hover_pass, .gpu = spade::compute::GpuRecipe::rotors}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "hover", .passes = liar});
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::invalid_argument);
    EXPECT_NE(s.error().context.find("hover.lift"), std::string::npos) << s.error().context;
}

// The refusal happens in create() before any device is opened, so this runs
// on a machine without a GPU.
TEST(ModuleSimulation, ADeveloperPassWithNoRecipeIsRefusedOnVulkanByName) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "hover", .passes = kHoverPasses});
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2,
                                               spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan},
                                               set);
    ASSERT_FALSE(sim.has_value());
    EXPECT_EQ(sim.error().code, spade::Code::unavailable);
    EXPECT_NE(sim.error().context.find("hover.lift"), std::string::npos) << sim.error().context;
}

TEST(ModuleSchedule, GpuPassesFollowTheSchedule) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    const auto g = spade::modules::gpu_passes(*s);
    ASSERT_EQ(g.size(), s->passes.size());
    for (size_t i = 0; i < g.size(); ++i) {
        EXPECT_EQ(g[i].name, s->passes[i].module + "." + s->passes[i].pass);
        EXPECT_EQ(g[i].recipe, s->passes[i].gpu);
    }
}

// Stage 3: modules declare the fields they provide (scalar, vec3 or a fixed
// band array); a read of `field.<name>` resolves against those declarations.
TEST(ModuleFields, ABuiltinFieldDeclaredWithAnotherKindIsRefused) {
    static constexpr spade::modules::FieldDecl wind[] = {{.name = "wind", .kind = spade::modules::FieldKind::scalar, .unit = "m/s"}};
    static constexpr QuantityAccess writes[] = {{"field.wind", Access::write}};
    static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "gusts", .passes = p, .fields = wind}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleFields, AReadOfAFieldNoModuleProvidesIsRefused) {
    static constexpr QuantityAccess reads[] = {{"field.salinity", Access::read}};
    static constexpr PassDecl p[] = {{.name = "probe", .phase = Phase::forces, .access = reads, .cpu = &noop}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "probe", .passes = p});
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("field.salinity"), std::string::npos) << s.error().context;
}

TEST(ModuleFields, TwoProvidersOfOneFieldAreRefused) {
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::scalar, .unit = "dB"}};
    static constexpr QuantityAccess writes[] = {{"field.hum", Access::write}};
    static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = p, .fields = hum}, {.name = "b", .passes = p, .fields = hum}};
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("hum"), std::string::npos) << s.error().context;
}

TEST(ModuleFields, ABandFieldTakesItsCountAfterTheBuiltins) {
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::bands, .bands = 8, .unit = "dB"}};
    static constexpr QuantityAccess writes[] = {{"field.hum", Access::write}};
    static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "acoustic", .passes = p, .fields = hum});
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(s->fields.back().name, "hum");
    EXPECT_EQ(s->fields.back().offset, spade::modules::kFieldBuiltinFloats);
    EXPECT_EQ(s->fields.back().count, 8u);
    EXPECT_EQ(s->field_stride, spade::modules::kFieldBuiltinFloats + 8u);
}

TEST(ModuleFields, ABandCountOutsideOneToTheMaximumIsRefused) {
    for (const uint32_t n : {0u, spade::modules::kMaxFieldBands + 1u}) {
        const spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::bands, .bands = n, .unit = "dB"}};
        static constexpr QuantityAccess writes[] = {{"field.hum", Access::write}};
        static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::fields, .access = writes, .cpu = &noop}};
        spade::modules::ModuleSet set = spade::modules::standard_modules();
        set.push_back({.name = "acoustic", .passes = p, .fields = hum});
        EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument) << "bands = " << n;
    }
}

// A provider writes in Fields, before any reader (spec section 6). A write in a
// later phase would leave an earlier-phase reader the previous substep's row --
// zeros on the first step, another timeline's after restore() -- so it is
// refused at compile, naming the field and the pass. (Stage-3 review.)
TEST(ModuleFields, AFieldWrittenOutsideTheFieldsPhaseIsRefused) {
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::scalar, .unit = "dB"}};
    static constexpr QuantityAccess writes[] = {{"field.hum", Access::write}};
    static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::forces, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "acoustic", .passes = p, .fields = hum}};
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::invalid_argument);
    EXPECT_NE(s.error().context.find("field.hum"), std::string::npos) << s.error().context;
    EXPECT_NE(s.error().context.find("acoustic.sample"), std::string::npos) << s.error().context;
}

TEST(ModuleFields, AProviderModuleWithNoPassWritingItsFieldIsRefused) {
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::scalar, .unit = "dB"}};
    static constexpr PassDecl p[] = {{.name = "idle", .phase = Phase::fields, .cpu = &noop}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "acoustic", .passes = p, .fields = hum});
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("field.hum"), std::string::npos) << s.error().context;
}

// test_module_schedule.cpp, after the hover helpers. Pass functions reach their field through a
// file-static offset, set from the compiled schedule before create().
namespace {
uint32_t g_offset = 0;
float g_seen_height = -1.0f;
std::array<float, 8> g_seen_bands{};

[[nodiscard]] uint32_t offset_of(const spade::modules::CompiledSchedule& s, std::string_view name) {
    for (const auto& f : s.fields) if (f.name == name) return f.offset;
    ADD_FAILURE() << "no field " << name;
    return 0;
}
void lift_to_42(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds) w.bodies[0].pos.y = 42.0f;
}
void sample_height(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds) w.fields[g_offset] = w.bodies[0].pos.y;
}
void read_height(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds) g_seen_height = w.fields[g_offset];
}
void sample_bands(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds)
        for (uint32_t i = 0; i < 8; ++i) w.fields[g_offset + i] = 0.5f * static_cast<float>(i);
}
void read_bands(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds)
        for (uint32_t i = 0; i < 8; ++i) g_seen_bands[i] = w.fields[g_offset + i];
}
}  // namespace

// The mover (an ordered Fields writer of body.pose) and the provider (a Fields reader of body.pose)
// are ordered by their hazard, so with ONE substep the provider must see 42, not the spawn's 0.
TEST(ModuleFields, AProviderSeesThePoseWrittenEarlierInFieldsTheSameSubstep) {
    static constexpr QuantityAccess move[] = {{"body.pose", Access::write}};
    static constexpr QuantityAccess sample[] = {{"body.pose", Access::read}, {"field.height", Access::write}};
    static constexpr QuantityAccess read[] = {{"field.height", Access::read}};
    static constexpr spade::modules::FieldDecl height[] = {{.name = "height", .kind = spade::modules::FieldKind::scalar, .unit = "m"}};
    static constexpr PassDecl mover[] = {{.name = "lift", .phase = Phase::fields, .access = move, .cpu = &lift_to_42}};
    static constexpr PassDecl probe[] = {
        {.name = "sample", .phase = Phase::fields, .access = sample, .cpu = &sample_height},
        {.name = "read", .phase = Phase::forces, .access = read, .cpu = &read_height}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "probe", .passes = probe, .fields = height});   // declared BEFORE the mover in set order
    set.push_back({.name = "mover", .passes = mover});
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    g_offset = offset_of(*s, "height");
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 1, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->spawn(0, spade::BodySpawn{}).has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    EXPECT_EQ(g_seen_height, 42.0f);
}

TEST(ModuleFields, ABandFieldRoundTripsOnTheCpu) {
    static constexpr QuantityAccess sample[] = {{"field.hum", Access::write}};
    static constexpr QuantityAccess read[] = {{"field.hum", Access::read}};
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::bands, .bands = 8, .unit = "dB"}};
    static constexpr PassDecl acoustic[] = {
        {.name = "sample", .phase = Phase::fields, .access = sample, .cpu = &sample_bands},
        {.name = "read", .phase = Phase::forces, .access = read, .cpu = &read_bands}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "acoustic", .passes = acoustic, .fields = hum});
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    g_offset = offset_of(*s, "hum");
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 1, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->step(1).has_value());
    for (uint32_t i = 0; i < 8; ++i) EXPECT_EQ(g_seen_bands[i], 0.5f * static_cast<float>(i)) << "band " << i;
}

TEST(ModuleFields, ABandFieldIsRefusedOnVulkanByName) {
    static constexpr QuantityAccess sample[] = {{"field.hum", Access::write}};
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::bands, .bands = 8, .unit = "dB"}};
    static constexpr PassDecl acoustic[] = {{.name = "sample", .phase = Phase::fields, .access = sample, .cpu = &sample_bands}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "acoustic", .passes = acoustic, .fields = hum});
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 1,
                                               spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan}, set);
    ASSERT_FALSE(sim.has_value());
    EXPECT_NE(sim.error().context.find("acoustic.sample"), std::string::npos) << sim.error().context;
}

TEST(StandardModules, ProvideGravityDensityAndWindAtFixedOffsets) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    ASSERT_EQ(s->fields.size(), 3u);
    EXPECT_EQ(s->fields[0].name, "gravity");
    EXPECT_EQ(s->fields[0].offset, spade::modules::kFieldGravityOffset);
    EXPECT_EQ(s->fields[1].name, "density");
    EXPECT_EQ(s->fields[1].offset, spade::modules::kFieldDensityOffset);
    EXPECT_EQ(s->fields[2].name, "wind");
    EXPECT_EQ(s->fields[2].offset, spade::modules::kFieldWindOffset);
    EXPECT_EQ(s->field_stride, spade::modules::kFieldBuiltinFloats);
}

// Stage 4: modules declare the arrays they own. compile_schedule tables them in
// walk order; the standard set's table spells today's walk, and the declarations
// leave the compiled order and the configuration identity alone.
namespace {

struct TallyRow {
    uint32_t count;
    uint32_t _p[3];
};
constexpr uint32_t kTallySize = spade::modules::row_size<TallyRow>();
constexpr spade::modules::ArrayDecl kTallyArrays[] = {{.name = "tally_counts", .elem_size = kTallySize}};

// Master d7db700's values, printed by a scratch test before any stage-4 change
// (plan Task 1, step 0). They are master's, not regenerated ones.
constexpr uint64_t kStandardIdentity = 0x3303cfc86821f502ULL;
// schema_hash() of one_body_world()'s registry at 2 ms and 2 substeps. Task 2
// pins the registered walk with it.
constexpr uint64_t kOneBodySchema = 0x1f60a6fef22e254aULL;

[[nodiscard]] spade::Result<CompiledSchedule> standard_plus(const ModuleDesc& extra) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(extra);
    return compile_schedule(set);
}

// Refused with invalid_argument, by a message that names `what`: a compile, or
// a Simulation::create().
template <class T>
[[nodiscard]] testing::AssertionResult refused_naming(const spade::Result<T>& r, std::string_view what) {
    if (r.has_value()) return testing::AssertionFailure() << "accepted; expected a refusal naming '" << what << "'";
    if (r.error().code != spade::Code::invalid_argument) {
        return testing::AssertionFailure() << "code " << static_cast<int>(r.error().code) << ": " << r.error().context;
    }
    if (r.error().context.find(what) == std::string::npos) {
        return testing::AssertionFailure() << "'" << what << "' is not named in: " << r.error().context;
    }
    return testing::AssertionSuccess();
}

}  // namespace

TEST(StandardModules, DeclareTodaysArraysInTodaysWalkOrder) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(spade::modules::walk_order(*s),
              (Names{"world_params", "bodies", "body_generation", "drag_bodies", "dryden", "imu_sensors", "imu_ring",
                     "rotors", "replay_config", "gnss_sensors", "gnss_ring"}));
}

TEST(StandardModules, StateDeclarationsLeaveTheOrderAndTheIdentityAlone) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(s->identity, kStandardIdentity) << "master's value; CompileToTodaysOrder pins the order";
}

TEST(ModuleState, AStatefulModulesQuantitiesNameItsArrays) {
    static constexpr QuantityAccess wrong[] = {{"tally.count", Access::write}};
    static constexpr QuantityAccess right[] = {{"tally.tally_counts", Access::write}};
    static constexpr PassDecl a[] = {{.name = "count", .phase = Phase::forces, .access = wrong, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "count", .phase = Phase::forces, .access = right, .cpu = &noop}};
    const ModuleDesc bad[] = {{.name = "tally", .passes = a, .state = kTallyArrays}};
    const ModuleDesc good[] = {{.name = "tally", .passes = b, .state = kTallyArrays}};
    EXPECT_EQ(code_of(compile_schedule(bad)), spade::Code::invalid_argument);
    EXPECT_TRUE(compile_schedule(good).has_value());
}

TEST(ModuleState, AnArrayNameIsDeclaredOnceAndIsNotACoreArray) {
    using spade::modules::ArrayDecl;
    static constexpr ArrayDecl twice[] = {{.name = "tally_counts", .elem_size = kTallySize},
                                          {.name = "tally_counts", .elem_size = kTallySize}};
    static constexpr ArrayDecl builtin[] = {{.name = "rotors", .elem_size = kTallySize}};
    static constexpr ArrayDecl core[] = {{.name = "bodies", .elem_size = kTallySize}};
    static constexpr ArrayDecl dotted[] = {{.name = "a.b", .elem_size = kTallySize}};
    static constexpr ArrayDecl sizeless[] = {{.name = "tally_counts", .elem_size = 0}};
    ASSERT_TRUE(standard_plus({.name = "tally", .state = kTallyArrays}).has_value()) << "the control";
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = twice}), "tally_counts"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = builtin}), "rotors"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = core}), "bodies"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = dotted}), "a.b"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = sizeless}), "tally_counts"));
}

TEST(ModuleState, APerRowArrayNeedsAnOwnerThatHoldsRows) {
    using spade::modules::ArrayDecl;
    using spade::modules::Extent;
    static constexpr ArrayDecl ownerless[] = {
        {.name = "tally_ring", .elem_size = kTallySize, .extent = Extent::per_row, .owner = "nobody", .depth = 4}};
    static constexpr ArrayDecl world_owner[] = {
        {.name = "tally_ring", .elem_size = kTallySize, .extent = Extent::per_row, .owner = "dryden", .depth = 4}};
    static constexpr ArrayDecl row_owner[] = {
        {.name = "tally_ring", .elem_size = kTallySize, .extent = Extent::per_row, .owner = "imu_ring", .depth = 4}};
    static constexpr ArrayDecl no_depth[] = {
        {.name = "tally_ring", .elem_size = kTallySize, .extent = Extent::per_row, .owner = "imu_sensors", .depth = 0}};
    static constexpr ArrayDecl owned_body[] = {
        {.name = "tally_ring", .elem_size = kTallySize, .extent = Extent::per_body, .owner = "imu_sensors"}};
    static constexpr ArrayDecl deep_body[] = {
        {.name = "tally_ring", .elem_size = kTallySize, .extent = Extent::per_body, .depth = 4}};
    // Physics' shape: one row per rotor slot, owned by another module's array.
    static constexpr ArrayDecl per_rotor[] = {
        {.name = "tally_ring", .elem_size = kTallySize, .extent = Extent::per_row, .owner = "rotors", .depth = 1}};
    ASSERT_TRUE(standard_plus({.name = "tally", .state = per_rotor}).has_value()) << "the control";
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = ownerless}), "nobody"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = world_owner}), "dryden"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = row_owner}), "imu_ring"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = no_depth}), "tally_ring"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = owned_body}), "tally_ring"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = deep_body}), "tally_ring"))
        << "a depth is not silently dropped";
}

TEST(ModuleState, OnlyTheLegacyArraysMayCarryTheLegacyMarker) {
    EXPECT_TRUE(refused_naming(standard_plus({.name = "tally", .state = kTallyArrays, .legacy_walk = true}),
                               "tally_counts"));
}

TEST(ModuleSchedule, APassNameWithADotOrAnUnknownPhaseIsRefused) {
    static constexpr PassDecl dotted[] = {{.name = "a.b", .phase = Phase::forces, .cpu = &noop}};
    static constexpr PassDecl unphased[] = {{.name = "x", .phase = static_cast<Phase>(6), .cpu = &noop}};
    EXPECT_TRUE(refused_naming(standard_plus({.name = "probe", .passes = dotted}), "probe.a.b"));
    EXPECT_TRUE(refused_naming(standard_plus({.name = "probe", .passes = unphased}), "probe.x"));
}

// Stage 4, Task 2: create() registers every module array from the declarations.
// The core registers its four itself; the legacy modules' arrays go before
// replay_config in set order, and every other module array after gnss_ring.
namespace {

[[nodiscard]] std::vector<std::string> walk_names(const spade::Simulation& sim) {
    std::vector<std::string> out;
    sim.arenas().registry().for_each_array([&](const spade::RegisteredArray& a) { out.push_back(a.name); });
    return out;
}

// The standard set with one module's arrays replaced.
[[nodiscard]] spade::modules::ModuleSet standard_with_state(std::string_view module,
                                                            std::span<const spade::modules::ArrayDecl> state) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    for (ModuleDesc& m : set) {
        if (m.name == module) m.state = state;
    }
    return set;
}

// A copy of one standard module's declarations, for a fixture that changes
// exactly one thing about them (Task 4 gave an attached array its spawn size,
// init and validate, which a hand-written copy would also have to spell).
[[nodiscard]] std::vector<spade::modules::ArrayDecl> standard_state(std::string_view module) {
    for (const ModuleDesc& m : spade::modules::standard_modules()) {
        if (m.name == module) return {m.state.begin(), m.state.end()};
    }
    return {};
}

}  // namespace

TEST(ModuleState, TheStandardWalkIsTodaysTwentyTwoEntries) {
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    std::vector<std::string> expected;
    for (const std::string& a : spade::modules::walk_order(sim->schedule())) {
        expected.push_back(a);
        expected.push_back(a + std::string(spade::kSlotToWorldSuffix));
    }
    const std::vector<std::string> walk = walk_names(*sim);
    EXPECT_EQ(walk, expected);
    ASSERT_EQ(walk.size(), 22u) << "today's walk: eleven arrays and their maps";
    EXPECT_EQ(walk[16], "replay_config") << "ReplayConfig.OccupiesItsPinnedWalkPosition's index";
    EXPECT_EQ(spade::schema_hash(sim->arenas().registry()), kOneBodySchema) << "master's value";
}

TEST(ModuleState, ADevelopersArrayIsAppendedAfterTheStandardWalk) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "tally", .state = kTallyArrays});
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto walk = walk_names(*sim);
    ASSERT_EQ(walk.size(), 24u);
    EXPECT_EQ(walk[16], "replay_config");
    EXPECT_EQ(walk[22], "tally_counts");
    EXPECT_EQ(walk[23], "tally_counts.slot_to_world");

    // Found by name, one zero-filled per_world row, typed only as its own size.
    const auto index = sim->module_array("tally_counts");
    ASSERT_TRUE(index.has_value()) << index.error().context;
    const auto bytes = sim->arenas().bytes(*index);
    ASSERT_TRUE(bytes.has_value()) << bytes.error().context;
    EXPECT_EQ(bytes->size(), sizeof(TallyRow)) << "one world, one row";
    EXPECT_TRUE(std::ranges::all_of(*bytes, [](std::byte b) { return b == std::byte{0}; }));
    EXPECT_TRUE(sim->arenas().typed<TallyRow>(*index).has_value());
    const auto wrong = sim->arenas().typed<uint64_t>(*index);
    ASSERT_FALSE(wrong.has_value());
    EXPECT_EQ(wrong.error().code, spade::Code::invalid_argument);
    const auto missing = sim->module_array("tally_count");
    ASSERT_FALSE(missing.has_value());
    EXPECT_EQ(missing.error().code, spade::Code::not_found);
    EXPECT_NE(missing.error().context.find("tally_count"), std::string::npos) << missing.error().context;
}

TEST(ModuleState, LegacyModulesRegisterBeforeReplayConfigInSetOrder) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    ASSERT_EQ(set[2].name, "imu");
    ASSERT_EQ(set[3].name, "rotor");
    std::swap(set[2], set[3]);
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto walk = walk_names(*sim);
    ASSERT_EQ(walk.size(), 22u);
    EXPECT_EQ(walk[10], "rotors");
    EXPECT_EQ(walk[12], "imu_sensors");
    EXPECT_EQ(walk[14], "imu_ring");
    EXPECT_EQ(walk[16], "replay_config");
}

// The engine's typed calls (add_gnss_sensor, set_rotor_commands, ...) use the
// seven built-in arrays, so a set must declare each exactly as the standard set
// does: its row size, extent, owner and depth (open question 1, approved), and
// (Task 4) the spawn size its typed front hands attach_row.
TEST(ModuleState, ASetWithoutABuiltinArrayOrWithAnotherRowSizeIsRefused) {
    using spade::modules::Extent;
    const auto create = [](const spade::modules::ModuleSet& set) {
        return spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    };

    spade::modules::ModuleSet no_gnss = spade::modules::standard_modules();
    std::erase_if(no_gnss, [](const ModuleDesc& m) { return m.name == "gnss"; });
    EXPECT_TRUE(refused_naming(create(no_gnss), "gnss_sensors"));
    spade::modules::ModuleSet no_drag = spade::modules::standard_modules();
    std::erase_if(no_drag, [](const ModuleDesc& m) { return m.name == "drag"; });
    EXPECT_TRUE(refused_naming(create(no_drag), "drag_bodies"));

    // Each fixture is the standard declaration with exactly one thing changed.
    auto small_rotors = standard_state("rotor");
    ASSERT_EQ(small_rotors.size(), 1u);
    small_rotors[0].elem_size = 16;
    EXPECT_TRUE(refused_naming(create(standard_with_state("rotor", small_rotors)), "rotors"));

    // Beyond the row size: another extent, depth or owner is another array.
    auto body_dryden = standard_state("dryden");
    ASSERT_EQ(body_dryden.size(), 1u);
    body_dryden[0].extent = Extent::per_body;
    body_dryden[0].init = &noop_row;  // a per_body array is attached, so it needs one to compile
    auto shallow_imu = standard_state("imu");
    ASSERT_EQ(shallow_imu.size(), 2u);
    shallow_imu[1].depth = 1;
    auto gnss_ring_on_imu = standard_state("gnss");
    ASSERT_EQ(gnss_ring_on_imu.size(), 2u);
    gnss_ring_on_imu[1].owner = "imu_sensors";
    EXPECT_TRUE(refused_naming(create(standard_with_state("dryden", body_dryden)), "dryden"));
    EXPECT_TRUE(refused_naming(create(standard_with_state("imu", shallow_imu)), "imu_ring"));
    EXPECT_TRUE(refused_naming(create(standard_with_state("gnss", gnss_ring_on_imu)), "gnss_ring"))
        << "the same shape, owned by another array";

    // add_gnss_sensor hands attach_row a GnssSensorSpawn; another spawn size
    // would refuse every receiver at the call, so it is refused here, by name.
    auto other_spawn = standard_state("gnss");
    other_spawn[0].spawn_size += 4;
    EXPECT_TRUE(refused_naming(create(standard_with_state("gnss", other_spawn)), "gnss_sensors"));
}

// The converse of OnlyTheLegacyArraysMayCarryTheLegacyMarker: a legacy array
// declared by a module without the marker would register after replay_config
// and move the walk, so it is refused by name.
TEST(ModuleState, ALegacyArrayIsDeclaredOnlyUnderTheLegacyMarker) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    for (ModuleDesc& m : set) {
        if (m.name == "rotor") m.legacy_walk = false;
    }
    const auto r = compile_schedule(set);
    EXPECT_TRUE(refused_naming(r, "array 'rotors'"));
    EXPECT_TRUE(refused_naming(r, "module 'rotor'"));
    EXPECT_TRUE(standard_plus({.name = "probe", .legacy_walk = true}).has_value())
        << "the control: the marker on a module with no arrays registers nothing";
}

// Stage 4, Task 3: the GPU mirror is sized from the declarations.
// state_array_shapes() needs no device, so it is checked against the registry
// here, on every box; tests/test_gpu_state_mirror.cpp runs it on the device.
namespace {

// Two worlds whose capacities all differ (and differ from a ring's), so an
// array sized by the wrong extent cannot match the registry by accident.
[[nodiscard]] spade::WorldSetDesc two_shaped_worlds() {
    auto world = spade::WorldBuilder()
                     .name("shaped")
                     .environment(spade::Environment{})
                     .capacities(spade::Capacities{3, 5, 2, 1})
                     .build();
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::WorldSetDesc{{inst, inst}};
}

[[nodiscard]] spade::compute::StepShape capacities_of(const spade::Simulation& sim) {
    spade::compute::StepShape shape{};
    shape.world_count = sim.layout().world_count;
    shape.body_capacity = sim.layout().body_capacity;
    shape.element_capacity = sim.layout().element_capacity;
    shape.sensor_capacity = sim.layout().sensor_capacity;
    return shape;
}

[[nodiscard]] std::string shape_text(const std::string& name, uint32_t elem_size, uint32_t capacity_per_world) {
    return name + ": " + std::to_string(elem_size) + " x " + std::to_string(capacity_per_world);
}

// A developer module with a per_world and a per_body array, so both of the
// extents the standard set leaves to the core are sized from a declaration.
// (A per_body array is attached, so it carries an init: Task 4's rule.)
constexpr spade::modules::ArrayDecl kTallyAndBodyArrays[] = {
    {.name = "tally_counts", .elem_size = kTallySize},
    {.name = "tally_bodies", .elem_size = kTallySize, .extent = spade::modules::Extent::per_body, .init = &noop_row}};

}  // namespace

TEST(ModuleState, TheMirrorsShapesAreTheRegistryWalkEntryForEntry) {
    spade::modules::ModuleSet with_tally = spade::modules::standard_modules();
    with_tally.push_back({.name = "tally", .state = kTallyAndBodyArrays});
    for (const spade::modules::ModuleSet& set : {spade::modules::standard_modules(), with_tally}) {
        const bool tally = set.size() == with_tally.size();
        SCOPED_TRACE(tally ? "standard + tally" : "standard");
        auto sim = spade::Simulation::create(two_shaped_worlds(), 2'000'000, 2, {}, set);
        ASSERT_TRUE(sim.has_value()) << sim.error().context;
        const auto shapes = spade::state_array_shapes(sim->schedule(), capacities_of(*sim));
        ASSERT_TRUE(shapes.has_value()) << shapes.error().context;

        Names registry;
        sim->arenas().registry().for_each_array([&](const spade::RegisteredArray& a) {
            EXPECT_EQ(a.world_count, 2u) << a.name;
            registry.push_back(shape_text(a.name, a.elem_size, a.capacity_per_world));
        });
        Names mirror;
        for (const spade::compute::StateArrayShape& a : *shapes) {
            mirror.push_back(shape_text(a.name, a.elem_size, a.capacity_per_world));
        }
        EXPECT_EQ(mirror, registry);
        EXPECT_EQ(mirror.size(), tally ? 26u : 22u);
    }
}

// A per_row capacity is its owner's rows times its depth, taken in 64 bits.
// 2^26 sensors with 64-deep rings is 2^32 ring rows per world, which a uint32
// product wraps to 0 -- as the mirror's hand list, sensor_capacity *
// kRingDepth, did before stage 4.
TEST(ModuleState, AStateArrayShapeThatWouldWrapIsRefusedByName) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    spade::compute::StepShape shape{};
    shape.world_count = 1;
    shape.body_capacity = 1;
    shape.element_capacity = 1;

    shape.sensor_capacity = (1u << 26) - 1u;
    const auto fits = spade::state_array_shapes(*s, shape);
    ASSERT_TRUE(fits.has_value()) << fits.error().context;
    const auto ring = std::ranges::find(*fits, std::string("imu_ring"), &spade::compute::StateArrayShape::name);
    ASSERT_NE(ring, fits->end()) << "the control";
    EXPECT_EQ(ring->capacity_per_world, 0xFFFF'FFC0u) << "the control: (2^26 - 1) * 64";

    shape.sensor_capacity = 1u << 26;
    const auto wraps = spade::state_array_shapes(*s, shape);
    ASSERT_FALSE(wraps.has_value()) << "2^26 * 64 = 2^32 ring rows would wrap to 0";
    EXPECT_EQ(wraps.error().code, spade::Code::capacity_exceeded);
    EXPECT_NE(wraps.error().context.find("'imu_ring'"), std::string::npos) << wraps.error().context;
}

// Stage 4, Task 4: attached rows go through one queued init and one declared
// cascade. The first two tests below pin the cascade's END STATE against
// today's hand cascade (drag, imu, gnss, rotors). The declared one runs in walk
// order (drag, imu, rotors, gnss), and each free touches only its own array and
// free list, so nothing may move.
namespace {

[[nodiscard]] spade::vehicles::ModelType test_quad() {
    spade::vehicles::QuadrotorParams p;
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.18f;
    p.rotor_height = 0.02f;
    for (spade::vehicles::RotorParams& rotor : p.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.13f;
        rotor.thrust_coeff = 1.2e-5f;
        rotor.torque_coeff = 1.9e-7f;
    }
    p.drag.mode = spade::physics::drag_mode::quadratic;
    p.drag.area = 0.05f;
    p.drag.coeffs = glm::vec3(1.6f, 0.0f, 0.0f);
    p.imu.rate_divider = 1;
    p.imu.sigma_a = 0.05f;
    p.imu.sigma_g = 0.01f;
    return spade::vehicles::make_quadrotor(p).value();
}

[[nodiscard]] spade::WorldSetDesc world_with(spade::Capacities capacities) {
    auto world = spade::WorldBuilder()
                     .name("rows")
                     .environment(spade::Environment{})
                     .capacities(capacities)
                     .build();
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::WorldSetDesc{{inst}};
}

[[nodiscard]] spade::DragElementSpawn test_drag() {
    spade::DragElementSpawn d;
    d.mode = spade::physics::drag_mode::quadratic;
    d.area = 0.02f;
    d.coeffs = glm::vec3(1.1f, 0.0f, 0.0f);
    d.local_pos = glm::vec3(0.0f, 0.1f, 0.0f);
    return d;
}

[[nodiscard]] spade::GnssSensorSpawn test_gnss() {
    spade::GnssSensorSpawn g;
    g.mount_pos = glm::vec3(0.0f, 0.05f, 0.0f);
    g.rate_divider = 1;
    g.sigma_h = 0.5f;
    g.sigma_v = 0.8f;
    g.sigma_vel = 0.1f;
    g.bias_tau_s = 2.0f;
    g.sigma_bias = 0.3f;
    return g;
}

[[nodiscard]] spade::ImuSensorSpawn test_imu() {
    spade::ImuSensorSpawn s;
    s.mount_pos = glm::vec3(0.02f, 0.0f, 0.0f);
    s.sigma_a = 0.02f;
    s.sigma_bg = 0.001f;
    return s;
}

[[nodiscard]] spade::VehicleSpawn at_x(float x) {
    spade::VehicleSpawn where;
    where.pos = glm::vec3(x, 10.0f, 0.0f);
    return where;
}

[[nodiscard]] spade::BodySpawn body_at_x(float x) {
    spade::BodySpawn b;
    b.pos = glm::vec3(x, 10.0f, 0.0f);
    b.vel = glm::vec3(0.5f, 0.0f, 0.0f);
    return b;
}

// Every byte and map entry of one module array, by name; empty if it is not
// found.
struct ArrayImage {
    std::vector<std::byte> bytes;
    std::vector<uint32_t> map;
};
[[nodiscard]] ArrayImage image_of(const spade::Simulation& sim, std::string_view array) {
    const auto index = sim.module_array(array);
    if (!index) return {};
    const auto bytes = sim.arenas().bytes(*index);
    const auto map = sim.arenas().slot_to_world(*index);
    if (!bytes || !map) return {};
    return {std::vector<std::byte>(bytes->begin(), bytes->end()), std::vector<uint32_t>(map->begin(), map->end())};
}

constexpr std::string_view kAttachedArrays[] = {"drag_bodies", "imu_sensors", "imu_ring",
                                                "rotors",      "gnss_sensors", "gnss_ring"};

}  // namespace

// Review focus 2: despawn frees exactly what the hand cascade freed. Green on
// the hand cascade too: it pins the end state, not the mechanism.
TEST(ModuleRows, DespawnLeavesEveryAttachedArrayAsAFreshSimulationHasIt) {
    const spade::WorldSetDesc desc = world_with(spade::Capacities{1, 6, 1, 1});
    auto fresh = spade::Simulation::create(desc, 2'000'000, 2);
    auto sim = spade::Simulation::create(desc, 2'000'000, 2);
    ASSERT_TRUE(fresh.has_value() && sim.has_value());
    const auto model = sim->register_model(test_quad());
    ASSERT_TRUE(model.has_value()) << model.error().context;
    const auto quad = sim->spawn(0, *model, at_x(0.0f));
    ASSERT_TRUE(quad.has_value()) << quad.error().context;
    ASSERT_TRUE(sim->add_drag_element(quad->body, test_drag()).has_value());
    ASSERT_TRUE(sim->add_gnss_sensor(quad->body, test_gnss()).has_value());
    ASSERT_TRUE(sim->step(3).has_value());

    // Every row and ring holds something before the despawn, or the test
    // would pass on a cascade that freed nothing.
    for (const std::string_view array : kAttachedArrays) {
        const ArrayImage live = image_of(*sim, array);
        ASSERT_FALSE(live.bytes.empty()) << array;
        EXPECT_TRUE(std::ranges::any_of(live.bytes, [](std::byte b) { return b != std::byte{0}; }))
            << array << " holds nothing to free";
    }

    ASSERT_TRUE(sim->despawn(quad->body).has_value());
    ASSERT_TRUE(sim->flush_structural().has_value());
    for (const std::string_view array : kAttachedArrays) {
        const ArrayImage after = image_of(*sim, array);
        const ArrayImage want = image_of(*fresh, array);
        ASSERT_FALSE(want.bytes.empty()) << array;
        EXPECT_TRUE(after.bytes == want.bytes) << array << ": a byte survived the despawn";
        EXPECT_TRUE(std::ranges::all_of(after.bytes, [](std::byte b) { return b == std::byte{0}; })) << array;
        EXPECT_TRUE(after.map == want.map) << array << ": a slot is still allocated";
        EXPECT_TRUE(std::ranges::all_of(after.map, [](uint32_t w) { return w == spade::kInvalidWorld; })) << array;
    }
}

// The plan's "Bytes" argument, pinned (the lead's request). The declared
// cascade frees in walk order, rotors before gnss_sensors; today's hand cascade
// freed gnss_sensors before rotors. Spawns and despawns interleave -- a vehicle
// reserved while a despawn is queued, and two bodies freed in one flush, one of
// them carrying rotors AND a receiver -- and every reservation's slot, and the
// whole walk's digest at three boundaries, must be what today's order produced.
// The digests were printed on a346cda's hand cascade before any Task 4 engine
// change: they are its values, not regenerated ones.
TEST(ModuleRows, InterleavedChurnLeavesTodaysSlotsAndBytes) {
    auto sim = spade::Simulation::create(world_with(spade::Capacities{4, 16, 4, 1}), 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto model = sim->register_model(test_quad());
    ASSERT_TRUE(model.has_value()) << model.error().context;
    const auto quad = [&](float x) {
        auto v = sim->spawn(0, *model, at_x(x));
        EXPECT_TRUE(v.has_value()) << v.error().context;
        return v.value_or(spade::VehicleRef{});
    };
    const auto rotors = [](const spade::VehicleRef& v) {
        return std::vector<uint32_t>(v.rotor_slots.begin(), v.rotor_slots.begin() + v.rotor_count);
    };
    const auto gnss = [&](spade::BodyRef body) {
        const auto r = sim->add_gnss_sensor(body, test_gnss());
        EXPECT_TRUE(r.has_value()) << r.error().context;
        return r.has_value() ? r->slot : spade::kInvalidWorld;
    };
    const auto imu = [&](spade::BodyRef body) {
        const auto r = sim->add_imu_sensor(body, test_imu());
        EXPECT_TRUE(r.has_value()) << r.error().context;
        return r.has_value() ? r->slot : spade::kInvalidWorld;
    };
    const auto drag = [&](spade::BodyRef body) {
        const auto r = sim->add_drag_element(body, test_drag());
        EXPECT_TRUE(r.has_value()) << r.error().context;
        return r.has_value() ? r->slot : spade::kInvalidWorld;
    };
    const auto digest = [&] { return spade::testing::state_digest(*sim); };

    // Body 0 a quad (rotors 0-3, drag 0, imu 0), body 1 a bare body with all
    // three attachments, body 2 a quad.
    const spade::VehicleRef a = quad(0.0f);
    EXPECT_EQ(a.body.slot, 0u);
    EXPECT_EQ(rotors(a), (std::vector<uint32_t>{0, 1, 2, 3}));
    EXPECT_EQ(a.imu_sensors[0].slot, 0u);
    const auto b = sim->spawn(0, body_at_x(3.0f));
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(b->slot, 1u);
    EXPECT_EQ(gnss(*b), 0u);
    EXPECT_EQ(imu(*b), 1u);
    EXPECT_EQ(drag(*b), 1u);
    EXPECT_EQ(gnss(a.body), 1u);
    const spade::VehicleRef c = quad(6.0f);
    EXPECT_EQ(c.body.slot, 2u);
    EXPECT_EQ(rotors(c), (std::vector<uint32_t>{4, 5, 6, 7}));
    EXPECT_EQ(c.imu_sensors[0].slot, 2u);
    EXPECT_EQ(gnss(c.body), 2u);
    ASSERT_TRUE(sim->step(3).has_value());

    // a's despawn is queued when d reserves, so d cannot take a's slots.
    ASSERT_TRUE(sim->despawn(a.body).has_value());
    const spade::VehicleRef d = quad(9.0f);
    EXPECT_EQ(d.body.slot, 3u);
    EXPECT_EQ(rotors(d), (std::vector<uint32_t>{8, 9, 10, 11}));
    EXPECT_EQ(d.imu_sensors[0].slot, 3u);
    EXPECT_EQ(gnss(d.body), 3u);
    ASSERT_TRUE(sim->flush_structural().has_value());
    const uint64_t after_a = digest();
    ASSERT_TRUE(sim->step(2).has_value());

    // e takes a's freed slots, lowest first. Then b and c go in one flush: c
    // carries rotors and a receiver, the pair whose order changed.
    const spade::VehicleRef e = quad(12.0f);
    EXPECT_EQ(e.body.slot, 0u);
    EXPECT_EQ(rotors(e), (std::vector<uint32_t>{0, 1, 2, 3}));
    EXPECT_EQ(e.imu_sensors[0].slot, 0u);
    EXPECT_EQ(gnss(e.body), 1u);
    ASSERT_TRUE(sim->despawn(*b).has_value());
    ASSERT_TRUE(sim->despawn(c.body).has_value());
    ASSERT_TRUE(sim->flush_structural().has_value());
    const uint64_t after_bc = digest();
    ASSERT_TRUE(sim->step(2).has_value());

    // f and g take b's and c's freed slots.
    const auto f = sim->spawn(0, body_at_x(15.0f));
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->slot, 1u);
    EXPECT_EQ(drag(*f), 1u);
    EXPECT_EQ(imu(*f), 1u);
    EXPECT_EQ(gnss(*f), 0u);
    const spade::VehicleRef g = quad(18.0f);
    EXPECT_EQ(g.body.slot, 2u);
    EXPECT_EQ(rotors(g), (std::vector<uint32_t>{4, 5, 6, 7}));
    EXPECT_EQ(g.imu_sensors[0].slot, 2u);
    EXPECT_EQ(gnss(g.body), 2u);
    ASSERT_TRUE(sim->step(2).has_value());
    const uint64_t after_fg = digest();

    EXPECT_EQ(after_a, 0x4223c0e1cd3b5bc4ULL) << "a freed while d was reserved";
    EXPECT_EQ(after_bc, 0xc9c283214c6c1020ULL) << "b and c freed in one flush";
    EXPECT_EQ(after_fg, 0x09c9140ec8ca9496ULL) << "f and g in b's and c's slots, two steps on";
}

// A developer's attached arrays, through the same door as the built-ins: a
// per_sensor tag with a four-deep per_row ring, and a per_body mark with a
// two-deep per_row trail.
namespace {

using spade::modules::Extent;

struct TagRow {
    uint32_t body_slot;
    uint32_t live;
    float value;
    float _p;
};
struct TagSpawn {
    float value;
};
void init_tag(const spade::modules::RowInit& in) noexcept {
    TagRow& row = spade::modules::row_as<TagRow>(in.row);
    row.value = spade::modules::spawn_as<TagSpawn>(in.spawn).value;
    row.live = 1u;  // last, like every built-in's liveness flag
}
spade::Result<void> validate_tag(std::span<const std::byte> spawn) {
    if (!(spade::modules::spawn_as<TagSpawn>(spawn).value >= 0.0f)) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "value must be >= 0"});
    }
    return {};
}
constexpr spade::modules::ArrayDecl kTagArrays[] = {
    {.name = "tag_rows",
     .elem_size = spade::modules::attached_row_size<TagRow>(),
     .extent = Extent::per_sensor,
     .spawn_size = sizeof(TagSpawn),
     .init = &init_tag,
     .validate = &validate_tag},
    {.name = "tag_ring",
     .elem_size = spade::modules::row_size<TagRow>(),
     .extent = Extent::per_row,
     .owner = "tag_rows",
     .depth = 4}};

[[nodiscard]] spade::modules::ModuleSet standard_plus_tags() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "tags", .state = kTagArrays});
    return set;
}

struct MarkRow {
    float value;
    uint32_t live;
    uint32_t local_body;
    uint32_t _p;
};
struct MarkSpawn {
    float value;
};
void init_mark(const spade::modules::RowInit& in) noexcept {
    MarkRow& row = spade::modules::row_as<MarkRow>(in.row);
    row.value = spade::modules::spawn_as<MarkSpawn>(in.spawn).value;
    row.local_body = in.local_body;
    row.live = 1u;
}
constexpr spade::modules::ArrayDecl kMarkArrays[] = {
    {.name = "mark_rows",
     .elem_size = spade::modules::row_size<MarkRow>(),
     .extent = Extent::per_body,
     .spawn_size = sizeof(MarkSpawn),
     .init = &init_mark},
    {.name = "mark_trail",
     .elem_size = spade::modules::row_size<MarkRow>(),
     .extent = Extent::per_row,
     .owner = "mark_rows",
     .depth = 2}};

// Every byte of a registered array set to `value` through the registry, as a
// pass would have written it.
void fill(const spade::Simulation& sim, std::string_view array, unsigned char value) {
    const spade::RegisteredArray* a = sim.arenas().registry().find(array);
    ASSERT_NE(a, nullptr) << array;
    std::memset(a->data, value, a->byte_size());
}

[[nodiscard]] bool all_bytes(std::span<const std::byte> bytes, unsigned char value) {
    return std::ranges::all_of(bytes, [value](std::byte b) { return b == std::byte{value}; });
}

}  // namespace

TEST(ModuleRows, AnAttachedRowIsInitializedAtTheBoundaryAndFreedWithItsBody) {
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, standard_plus_tags());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto body = sim->spawn(0, spade::BodySpawn{});
    ASSERT_TRUE(body.has_value());
    const auto row = sim->attach_row(*body, "tag_rows", TagSpawn{2.5f});
    ASSERT_TRUE(row.has_value()) << row.error().context;
    EXPECT_EQ(*row, (spade::RowRef{0, 0}));
    EXPECT_EQ((*sim->module_rows<TagRow>("tag_rows", 0))[0].live, 0u) << "inert until the boundary";
    EXPECT_EQ(sim->pending_structural_ops(), 2u) << "the body's init and the row's";
    ASSERT_TRUE(sim->step(0).has_value());
    const TagRow live = (*sim->module_rows<TagRow>("tag_rows", 0))[0];
    EXPECT_EQ(live.value, 2.5f);
    EXPECT_EQ(live.live, 1u);
    EXPECT_EQ(live.body_slot, 0u);
    ASSERT_TRUE(sim->despawn(*body).has_value() && sim->step(0).has_value());
    const TagRow freed = (*sim->module_rows<TagRow>("tag_rows", 0))[0];
    EXPECT_EQ(freed.live, 0u);
    EXPECT_EQ(freed.value, 0.0f);
    const auto index = sim->module_array("tag_rows");
    ASSERT_TRUE(index.has_value());
    EXPECT_EQ((*sim->arenas().slot_to_world(*index))[0], spade::kInvalidWorld) << "the slot is free again";
}

TEST(ModuleRows, AChildRowIsClearedWithItsOwner) {
    auto sim = spade::Simulation::create(world_with(spade::Capacities{2, 1, 2, 1}), 2'000'000, 2, {},
                                         standard_plus_tags());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto first = sim->spawn(0, body_at_x(0.0f));
    const auto second = sim->spawn(0, body_at_x(3.0f));
    ASSERT_TRUE(first.has_value() && second.has_value());
    const auto first_tag = sim->attach_row(*first, "tag_rows", TagSpawn{1.0f});
    const auto second_tag = sim->attach_row(*second, "tag_rows", TagSpawn{2.0f});
    ASSERT_TRUE(first_tag.has_value()) << first_tag.error().context;
    ASSERT_TRUE(second_tag.has_value()) << second_tag.error().context;
    EXPECT_EQ(first_tag->slot, 0u);
    EXPECT_EQ(second_tag->slot, 1u);
    ASSERT_TRUE(sim->step(0).has_value());
    fill(*sim, "tag_ring", 0xAB);

    ASSERT_TRUE(sim->despawn(*first).has_value() && sim->flush_structural().has_value());
    const auto ring = sim->arenas().bytes(sim->module_array("tag_ring").value());
    ASSERT_TRUE(ring.has_value());
    ASSERT_EQ(ring->size(), 8 * sizeof(TagRow)) << "two sensors, four rows each";
    EXPECT_TRUE(all_bytes(ring->first(4 * sizeof(TagRow)), 0x00)) << "the freed row's window";
    EXPECT_TRUE(all_bytes(ring->last(4 * sizeof(TagRow)), 0xAB)) << "the other body's window is untouched";
    EXPECT_EQ((*sim->module_rows<TagRow>("tag_rows", 0))[1].value, 2.0f);
}

TEST(ModuleRows, APerBodyRowIsAttachedAtTheBodysSlotAndClearedWithIt) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "marks", .state = kMarkArrays});
    auto sim = spade::Simulation::create(world_with(spade::Capacities{2, 1, 1, 1}), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto first = sim->spawn(0, body_at_x(0.0f));
    const auto second = sim->spawn(0, body_at_x(3.0f));
    ASSERT_TRUE(first.has_value() && second.has_value());
    const auto mark = sim->attach_row(*second, "mark_rows", MarkSpawn{7.0f});
    ASSERT_TRUE(mark.has_value()) << mark.error().context;
    EXPECT_EQ(*mark, (spade::RowRef{0, second->slot})) << "the body's own slot; nothing is reserved";
    ASSERT_TRUE(sim->attach_row(*first, "mark_rows", MarkSpawn{5.0f}).has_value());
    EXPECT_EQ((*sim->module_rows<MarkRow>("mark_rows", 0))[1].live, 0u) << "inert until the boundary";
    ASSERT_TRUE(sim->step(0).has_value());
    {
        const auto rows = *sim->module_rows<MarkRow>("mark_rows", 0);
        EXPECT_EQ(rows[1].value, 7.0f);
        EXPECT_EQ(rows[1].local_body, 1u);
        EXPECT_EQ(rows[1].live, 1u);
        EXPECT_EQ(rows[0].value, 5.0f);
    }
    fill(*sim, "mark_trail", 0xCD);

    ASSERT_TRUE(sim->despawn(*second).has_value() && sim->flush_structural().has_value());
    const auto rows = *sim->module_rows<MarkRow>("mark_rows", 0);
    const auto bytes = sim->arenas().bytes(sim->module_array("mark_rows").value());
    ASSERT_TRUE(bytes.has_value());
    EXPECT_TRUE(all_bytes(bytes->last(sizeof(MarkRow)), 0x00)) << "the despawned body's row";
    EXPECT_EQ(rows[0].value, 5.0f) << "the other body's row is untouched";
    EXPECT_EQ(rows[0].live, 1u);
    const auto trail = sim->arenas().bytes(sim->module_array("mark_trail").value());
    ASSERT_TRUE(trail.has_value());
    EXPECT_TRUE(all_bytes(trail->last(2 * sizeof(MarkRow)), 0x00)) << "its two trail rows";
    EXPECT_TRUE(all_bytes(trail->first(2 * sizeof(MarkRow)), 0xCD)) << "the other body's trail is untouched";
}

TEST(ModuleRows, AttachRowRefusesWhatItCannotAttach) {
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, standard_plus_tags());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto body = sim->spawn(0, spade::BodySpawn{});
    ASSERT_TRUE(body.has_value());
    const auto code = [](const spade::Result<spade::RowRef>& r) {
        return r.has_value() ? spade::Code::internal : r.error().code;
    };
    const auto unknown = sim->attach_row(*body, "tag_rowz", TagSpawn{1.0f});
    EXPECT_EQ(code(unknown), spade::Code::not_found);
    EXPECT_TRUE(!unknown.has_value() && unknown.error().context.find("tag_rowz") != std::string::npos);

    EXPECT_TRUE(refused_naming(sim->attach_row(*body, "dryden", TagSpawn{1.0f}), "dryden"))
        << "a per_world row is not attached to a body";
    EXPECT_TRUE(refused_naming(sim->attach_row(*body, "tag_ring", TagSpawn{1.0f}), "tag_ring"))
        << "a per_row row comes with its owner";
    EXPECT_TRUE(refused_naming(sim->attach_row(*body, "tag_rows", uint64_t{1}), "tag_rows"))
        << "an 8-byte record for a 4-byte spawn";
    EXPECT_TRUE(refused_naming(sim->attach_row(*body, "tag_rows", TagSpawn{-1.0f}), "attach_row: value must be >= 0"))
        << "the module's validate, prefixed by the call";

    // Rotors arrive only with a vehicle, whatever record is offered.
    const auto rotors = std::ranges::find(sim->schedule().arrays, std::string("rotors"),
                                          &spade::modules::CompiledArray::name);
    ASSERT_NE(rotors, sim->schedule().arrays.end());
    const std::vector<std::byte> rotor_record(rotors->spawn_size);
    EXPECT_TRUE(refused_naming(sim->attach_row(*body, "rotors", rotor_record), "vehicle"));

    // The typed front and the door give one message after their prefixes.
    spade::ImuSensorSpawn no_rate;
    no_rate.rate_divider = 0;
    const auto front = sim->add_imu_sensor(*body, no_rate);
    const auto door = sim->attach_row(*body, "imu_sensors", no_rate);
    ASSERT_FALSE(front.has_value());
    ASSERT_TRUE(refused_naming(door, "attach_row: "));
    const std::string_view front_text = front.error().context;
    ASSERT_TRUE(front_text.starts_with("add_imu_sensor: ")) << front_text;
    EXPECT_EQ(front_text.substr(std::string_view("add_imu_sensor: ").size()),
              std::string_view(door.error().context).substr(std::string_view("attach_row: ").size()));

    // Nothing above reserved a row or queued an op.
    EXPECT_EQ(sim->pending_structural_ops(), 1u) << "the body's own init only";

    // sensors = 1: the second tag is past the world's declared capacity, which
    // a per_sensor array counts alone (the IMU and GNSS arrays hold none).
    ASSERT_TRUE(sim->attach_row(*body, "tag_rows", TagSpawn{1.0f}).has_value());
    const auto second = sim->attach_row(*body, "tag_rows", TagSpawn{1.0f});
    EXPECT_EQ(code(second), spade::Code::capacity_exceeded);
    EXPECT_TRUE(sim->add_imu_sensor(*body, spade::ImuSensorSpawn{}).has_value()) << "the IMU array counts alone";

    // A dead body attaches nothing.
    ASSERT_TRUE(sim->despawn(*body).has_value());
    EXPECT_EQ(code(sim->attach_row(*body, "tag_rows", TagSpawn{1.0f})), spade::Code::not_found);
}

TEST(ModuleState, AnAttachedArrayNeedsAnInitAndASpawnThatFits) {
    using spade::modules::ArrayDecl;
    using spade::modules::kMaxSpawnBytes;
    const auto with = [](std::span<const ArrayDecl> state) { return standard_plus({.name = "tags", .state = state}); };
    ASSERT_TRUE(with(kTagArrays).has_value()) << "the control";

    static constexpr ArrayDecl no_init[] = {
        {.name = "tag_rows", .elem_size = kTallySize, .extent = Extent::per_sensor, .spawn_size = 4}};
    static constexpr ArrayDecl body_no_init[] = {{.name = "tag_rows", .elem_size = kTallySize, .extent = Extent::per_body}};
    static constexpr ArrayDecl fits[] = {
        {.name = "tag_rows", .elem_size = kTallySize, .extent = Extent::per_element, .spawn_size = kMaxSpawnBytes,
         .init = &noop_row}};
    static constexpr ArrayDecl too_big[] = {
        {.name = "tag_rows", .elem_size = kTallySize, .extent = Extent::per_element, .spawn_size = kMaxSpawnBytes + 1,
         .init = &noop_row}};
    static constexpr ArrayDecl no_body_slot[] = {
        {.name = "tag_rows", .elem_size = 2, .extent = Extent::per_sensor, .init = &noop_row}};
    static constexpr ArrayDecl world_init[] = {{.name = "tag_rows", .elem_size = kTallySize, .init = &noop_row}};
    static constexpr ArrayDecl world_spawn[] = {{.name = "tag_rows", .elem_size = kTallySize, .spawn_size = 4}};
    // The plan's Task 7 initializes rows owned by rotors from a vehicle hook,
    // so a per_row array may carry an init and a spawn size.
    static constexpr ArrayDecl ring_init[] = {
        {.name = "tag_ring", .elem_size = kTallySize, .extent = Extent::per_row, .owner = "rotors", .depth = 1,
         .spawn_size = 4, .init = &noop_row}};
    static constexpr ArrayDecl ring_too_big[] = {
        {.name = "tag_ring", .elem_size = kTallySize, .extent = Extent::per_row, .owner = "rotors", .depth = 1,
         .spawn_size = kMaxSpawnBytes + 1, .init = &noop_row}};
    ASSERT_TRUE(with(fits).has_value()) << "the control: exactly kMaxSpawnBytes";
    EXPECT_TRUE(with(ring_init).has_value()) << "a per_row row a vehicle hook initializes";
    EXPECT_TRUE(refused_naming(with(no_init), "tag_rows"));
    EXPECT_TRUE(refused_naming(with(body_no_init), "tag_rows")) << "a per_body row is attached too";
    EXPECT_TRUE(refused_naming(with(too_big), "tag_rows"));
    EXPECT_TRUE(refused_naming(with(no_body_slot), "tag_rows")) << "a slot-allocated row starts with its body_slot";
    EXPECT_TRUE(refused_naming(with(world_init), "tag_rows")) << "an init that would never run";
    EXPECT_TRUE(refused_naming(with(world_spawn), "tag_rows")) << "a spawn size that would never be offered";
    EXPECT_TRUE(refused_naming(with(ring_too_big), "tag_ring"));
}

// Stage 4, Task 5: seeded streams are declared, and create() and reseed() walk
// the declarations. A developer's module holds a per_world stream and a
// per_sensor one, each derived as the built-ins derive theirs: from the world's
// registered seed, a tag of its own and the row's world-local slot. Each row
// type has ONE derivation, which its init and its reseed share (TD-9).
namespace {

using spade::modules::StreamDecl;

constexpr std::string_view kWeatherTag = "test.weather";
constexpr std::string_view kNoisyTag = "test.noisy";

struct WeatherRow {
    spade::rng::Stream noise;
};
struct NoisyRow {
    uint32_t body_slot;
    uint32_t live;
    uint32_t _p[2];
    spade::rng::Stream noise;
};

void reseed_weather(std::span<std::byte> row, const spade::WorldParams& params, uint32_t local_slot) noexcept {
    spade::modules::row_as<WeatherRow>(row).noise = spade::rng::make_stream(params.seed, kWeatherTag, local_slot);
}
void reseed_noisy(std::span<std::byte> row, const spade::WorldParams& params, uint32_t local_slot) noexcept {
    spade::modules::row_as<NoisyRow>(row).noise = spade::rng::make_stream(params.seed, kNoisyTag, local_slot);
}
void init_noisy(const spade::modules::RowInit& in) noexcept {
    NoisyRow& row = spade::modules::row_as<NoisyRow>(in.row);
    row.body_slot = in.local_body;
    row._p[0] = 0u;
    row._p[1] = 0u;
    reseed_noisy(in.row, *in.params, in.local_slot);
    row.live = 1u;
}

constexpr spade::modules::ArrayDecl kNoisyArrays[] = {
    {.name = "noisy_weather", .elem_size = spade::modules::row_size<WeatherRow>()},
    {.name = "noisy_rows",
     .elem_size = spade::modules::attached_row_size<NoisyRow>(),
     .extent = Extent::per_sensor,
     .init = &init_noisy},
    {.name = "noisy_ring",
     .elem_size = spade::modules::row_size<NoisyRow>(),
     .extent = Extent::per_row,
     .owner = "noisy_rows",
     .depth = 2}};
constexpr StreamDecl kNoisyStreams[] = {{.tag = kWeatherTag, .array = "noisy_weather", .reseed = &reseed_weather},
                                        {.tag = kNoisyTag, .array = "noisy_rows", .reseed = &reseed_noisy}};

[[nodiscard]] ModuleDesc noisy_module(std::span<const StreamDecl> streams = kNoisyStreams) {
    return {.name = "noisy", .state = kNoisyArrays, .streams = streams};
}

[[nodiscard]] spade::modules::ModuleSet standard_plus_noisy() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(noisy_module());
    return set;
}

[[nodiscard]] bool same_stream(const spade::rng::Stream& a, const spade::rng::Stream& b) {
    return std::memcmp(&a, &b, sizeof(spade::rng::Stream)) == 0;
}

[[nodiscard]] spade::Result<spade::RowRef> attach_noisy(spade::Simulation& sim, spade::BodyRef body) {
    return sim.attach_row(body, "noisy_rows", std::span<const std::byte>{});
}

// World `w`'s rows of `array`, byte for byte the same in `a` and `b`, and not
// all zero (or the comparison would prove nothing).
template <class Row>
void expect_same_rows(const spade::Simulation& a, const spade::Simulation& b, std::string_view array, uint32_t w) {
    const auto rows_a = a.module_rows<Row>(array, w);
    const auto rows_b = b.module_rows<Row>(array, w);
    ASSERT_TRUE(rows_a.has_value() && rows_b.has_value()) << array;
    ASSERT_EQ(rows_a->size(), rows_b->size()) << array;
    EXPECT_EQ(std::memcmp(rows_a->data(), rows_b->data(), rows_b->size_bytes()), 0) << array << ", world " << w;
    EXPECT_TRUE(std::ranges::any_of(std::as_bytes(*rows_b), [](std::byte x) { return x != std::byte{0}; }))
        << array << ", world " << w << ": nothing was derived";
}

// The `noise` stream of every row of `array` in world `w`, the same in `a` and
// `b`; the first `live` rows hold one.
template <class Row>
void expect_same_noise(const spade::Simulation& a, const spade::Simulation& b, std::string_view array, uint32_t w,
                       std::size_t live) {
    const auto rows_a = a.module_rows<Row>(array, w);
    const auto rows_b = b.module_rows<Row>(array, w);
    ASSERT_TRUE(rows_a.has_value() && rows_b.has_value()) << array;
    ASSERT_EQ(rows_a->size(), rows_b->size()) << array;
    for (std::size_t i = 0; i < rows_b->size(); ++i) {
        EXPECT_TRUE(same_stream((*rows_a)[i].noise, (*rows_b)[i].noise)) << array << ", world " << w << ", slot " << i;
        if (i < live) {
            EXPECT_FALSE(same_stream((*rows_b)[i].noise, spade::rng::Stream{}))
                << array << ", world " << w << ", slot " << i << ": no stream was derived";
        }
    }
}

}  // namespace

// Review focus 3, the 2026-10-02 GNSS defect closed structurally: two modules
// cannot share a tag, or the second would draw the first's numbers; and "world"
// is the tag every world's root is derived under (kWorldSeedDomainTag).
TEST(ModuleStreams, ATagIsDeclaredOnceAndIsNotTheWorldTag) {
    static_assert(spade::kWorldSeedDomainTag == "world");
    static constexpr StreamDecl imus[] = {{.tag = "sensor.imu", .array = "noisy_rows", .reseed = &reseed_noisy}};
    static constexpr StreamDecl twice[] = {{.tag = kNoisyTag, .array = "noisy_rows", .reseed = &reseed_noisy},
                                           {.tag = kNoisyTag, .array = "noisy_weather", .reseed = &reseed_weather}};
    static constexpr StreamDecl world[] = {
        {.tag = spade::kWorldSeedDomainTag, .array = "noisy_weather", .reseed = &reseed_weather}};
    static constexpr StreamDecl untagged[] = {{.tag = "", .array = "noisy_weather", .reseed = &reseed_weather}};
    ASSERT_TRUE(standard_plus(noisy_module()).has_value()) << "the control";

    const auto shared = standard_plus(noisy_module(imus));
    EXPECT_TRUE(refused_naming(shared, "sensor.imu"));
    EXPECT_TRUE(refused_naming(shared, "module 'imu'")) << "the module that declared it first";
    EXPECT_TRUE(refused_naming(shared, "module 'noisy'"));
    EXPECT_TRUE(refused_naming(standard_plus(noisy_module(twice)), kNoisyTag)) << "twice in one module";
    EXPECT_TRUE(refused_naming(standard_plus(noisy_module(world)), "'world'"));
    EXPECT_TRUE(refused_naming(standard_plus(noisy_module(untagged)), "noisy_weather")) << "a stream needs a tag";
}

// A stream's liveness is the arena's: a per_world row is always live, and a
// slot-allocated (per_element, per_sensor) row is live while its map names the
// world. A per_body or per_row row has no map of its own, so it holds none.
TEST(ModuleStreams, AStreamLivesInItsModulesSlotAllocatedOrPerWorldArray) {
    static constexpr StreamDecl foreign[] = {{.tag = kNoisyTag, .array = "imu_sensors", .reseed = &reseed_noisy}};
    static constexpr StreamDecl unknown[] = {{.tag = kNoisyTag, .array = "noisy_rowz", .reseed = &reseed_noisy}};
    static constexpr StreamDecl ring[] = {{.tag = kNoisyTag, .array = "noisy_ring", .reseed = &reseed_noisy}};
    static constexpr StreamDecl headless[] = {{.tag = kNoisyTag, .array = "noisy_rows"}};
    static constexpr StreamDecl per_body[] = {{.tag = kNoisyTag, .array = "mark_rows", .reseed = &reseed_noisy}};
    EXPECT_TRUE(refused_naming(standard_plus(noisy_module(foreign)), "imu_sensors")) << "another module's array";
    EXPECT_TRUE(refused_naming(standard_plus(noisy_module(unknown)), "noisy_rowz"));
    EXPECT_TRUE(refused_naming(standard_plus(noisy_module(ring)), "noisy_ring"));
    EXPECT_TRUE(refused_naming(standard_plus(noisy_module(headless)), kNoisyTag)) << "a stream nothing re-derives";
    EXPECT_TRUE(refused_naming(standard_plus({.name = "marks", .state = kMarkArrays, .streams = per_body}),
                               "mark_rows"));
}

// reseed()'s order before stage 4 -- per world, dryden, then the IMU rows, then
// the GNSS rows -- is the declarations' order: set order, then declaration
// order. Each tag is the one its derivation draws under.
TEST(StandardModules, DeclareTheDrydenImuAndGnssStreams) {
    static_assert(spade::kDrydenDomainTag == "dryden");
    static_assert(spade::sensors::kImuNoiseDomainTag == "sensor.imu");
    static_assert(spade::sensors::kGnssNoiseDomainTag == "sensor.gnss");
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    Names tags;
    Names modules;
    Names arrays;
    for (const spade::modules::CompiledStream& stream : s->streams) {
        tags.push_back(stream.tag);
        modules.push_back(stream.module);
        arrays.push_back(stream.array < s->arrays.size() ? s->arrays[stream.array].name : "?");
        EXPECT_NE(stream.reseed, nullptr) << stream.tag;
    }
    EXPECT_EQ(tags, (Names{"dryden", "sensor.imu", "sensor.gnss"}));
    EXPECT_EQ(modules, (Names{"dryden", "imu", "gnss"}));
    EXPECT_EQ(arrays, (Names{"dryden", "imu_sensors", "gnss_sensors"}));
}

// Review focus 3: reseed() reaches a developer's streams with no edit to
// Simulation -- the per_world one create() derived, and every live row's --
// and writes nothing into a free row.
TEST(ModuleStreams, ReseedRederivesADevelopersStreamAndLeavesFreeRowsZero) {
    auto sim = spade::Simulation::create(world_with(spade::Capacities{1, 1, 2, 1}), 2'000'000, 2, {},
                                         standard_plus_noisy());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto body = sim->spawn(0, spade::BodySpawn{});
    ASSERT_TRUE(body.has_value());
    const auto row = attach_noisy(*sim, *body);
    ASSERT_TRUE(row.has_value()) << row.error().context;
    ASSERT_TRUE(sim->flush_structural().has_value());

    const auto params = sim->world_params(0);
    const auto weather = sim->module_rows<WeatherRow>("noisy_weather", 0);
    const auto rows = sim->module_rows<NoisyRow>("noisy_rows", 0);
    ASSERT_TRUE(params.has_value() && weather.has_value() && rows.has_value());
    ASSERT_EQ(rows->size(), 2u);
    const uint64_t old_seed = (*params)->seed;
    EXPECT_TRUE(same_stream((*weather)[0].noise, spade::rng::make_stream(old_seed, kWeatherTag, 0)))
        << "create() derives a per_world stream from the declarations";
    EXPECT_TRUE(same_stream((*rows)[0].noise, spade::rng::make_stream(old_seed, kNoisyTag, 0)));

    ASSERT_TRUE(sim->reseed(0xBEEF).has_value());
    const uint64_t new_seed = (*params)->seed;
    ASSERT_NE(new_seed, old_seed);
    EXPECT_TRUE(same_stream((*weather)[0].noise, spade::rng::make_stream(new_seed, kWeatherTag, 0)))
        << "the per_world row follows the new seed";
    EXPECT_TRUE(same_stream((*rows)[0].noise, spade::rng::make_stream(new_seed, kNoisyTag, 0)))
        << "the live row follows the new seed";
    EXPECT_EQ((*rows)[0].live, 1u) << "only the stream is rewritten";
    const std::array<std::byte, sizeof(NoisyRow)> zero{};
    EXPECT_EQ(std::memcmp(&(*rows)[1], zero.data(), zero.size()), 0) << "a free row stays zero";
}

// The lead's pin (TD-9): the init and the reseed derive each stream with ONE
// function, so a Simulation reseeded to S holds exactly the streams a
// Simulation created at S holds -- every built-in stream and a developer's, in
// two worlds, after the reseeded run's streams have advanced.
TEST(ModuleStreams, AReseededSimulationHoldsTheStreamsOneCreatedAtTheNewSeedHolds) {
    constexpr uint32_t kWorlds = 2;
    constexpr uint64_t kOldScene = 0x5EED'0001ULL;
    constexpr uint64_t kNewScene = 0x5EED'0002ULL;
    const spade::WorldInstanceDesc prototype = world_with(spade::Capacities{2, 1, 2, 1}).worlds[0];
    // Per world: body 0 carries an IMU, a receiver and a noisy row; body 1 an
    // IMU and a noisy row. So every streamed array holds a live row at local
    // slot 0, and the IMU and noisy arrays one at slot 1.
    const auto build = [&](uint64_t scene_seed) -> spade::Result<spade::Simulation> {
        auto sim = spade::Simulation::create(spade::replicate(prototype, kWorlds, scene_seed), 2'000'000, 2, {},
                                             standard_plus_noisy());
        if (!sim) return sim;
        for (uint32_t w = 0; w < kWorlds; ++w) {
            const auto first = sim->spawn(w, body_at_x(0.0f));
            const auto second = sim->spawn(w, body_at_x(3.0f));
            if (!first || !second) return std::unexpected(spade::Error{spade::Code::internal, "spawn"});
            if (!sim->add_imu_sensor(*first, test_imu()) || !sim->add_gnss_sensor(*first, test_gnss()) ||
                !attach_noisy(*sim, *first) || !sim->add_imu_sensor(*second, test_imu()) ||
                !attach_noisy(*sim, *second)) {
                return std::unexpected(spade::Error{spade::Code::internal, "attach"});
            }
        }
        if (!sim->flush_structural()) return std::unexpected(spade::Error{spade::Code::internal, "flush"});
        return sim;
    };
    auto reseeded = build(kOldScene);
    auto fresh = build(kNewScene);
    ASSERT_TRUE(reseeded.has_value()) << reseeded.error().context;
    ASSERT_TRUE(fresh.has_value()) << fresh.error().context;

    // The control: before the reseed the two hold different streams.
    {
        const auto a = reseeded->module_rows<spade::sensors::ImuSensorRow>("imu_sensors", 0);
        const auto b = fresh->module_rows<spade::sensors::ImuSensorRow>("imu_sensors", 0);
        ASSERT_TRUE(a.has_value() && b.has_value());
        ASSERT_FALSE(same_stream((*a)[0].noise, (*b)[0].noise));
    }
    ASSERT_TRUE(reseeded->step(3).has_value()) << "advance the old streams, so a re-derivation is visible";
    ASSERT_TRUE(reseeded->reseed(kNewScene).has_value());

    for (uint32_t w = 0; w < kWorlds; ++w) {
        const auto seed_a = reseeded->world_params(w);
        const auto seed_b = fresh->world_params(w);
        ASSERT_TRUE(seed_a.has_value() && seed_b.has_value());
        EXPECT_EQ((*seed_a)->seed, (*seed_b)->seed) << "world " << w;
        expect_same_rows<spade::DrydenState>(*reseeded, *fresh, "dryden", w);  // dryden_init writes the whole row
        expect_same_rows<WeatherRow>(*reseeded, *fresh, "noisy_weather", w);
        expect_same_noise<spade::sensors::ImuSensorRow>(*reseeded, *fresh, "imu_sensors", w, 2);
        expect_same_noise<spade::sensors::GnssSensorRow>(*reseeded, *fresh, "gnss_sensors", w, 1);
        expect_same_noise<NoisyRow>(*reseeded, *fresh, "noisy_rows", w, 2);
    }
}

// L6: a streamed built-in array (dryden, imu_sensors, gnss_sensors) whose
// module dropped its stream, or declared it under another tag, would hold noise
// reseed() never reaches. create() refuses it, as it refuses a reshaped one,
// naming the module, the array and the standard tag.
TEST(ModuleStreams, ABuiltinStreamedArrayWithoutItsStreamIsRefused) {
    const auto create = [](const spade::modules::ModuleSet& set) {
        return spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    };
    const auto with_imu_streams = [](std::span<const StreamDecl> streams) {
        spade::modules::ModuleSet set = spade::modules::standard_modules();
        for (ModuleDesc& m : set) {
            if (m.name == "imu") m.streams = streams;
        }
        return set;
    };
    std::vector<StreamDecl> renamed;
    for (const ModuleDesc& m : spade::modules::standard_modules()) {
        if (m.name == "imu") renamed.assign(m.streams.begin(), m.streams.end());
    }
    ASSERT_EQ(renamed.size(), 1u);
    renamed[0].tag = "sensor.imu.v2";
    ASSERT_TRUE(create(spade::modules::standard_modules()).has_value()) << "the control";
    ASSERT_TRUE(create(standard_plus_noisy()).has_value()) << "the control: a developer's streams beside them";

    const auto silent = create(with_imu_streams({}));
    EXPECT_TRUE(refused_naming(silent, "module 'imu'"));
    EXPECT_TRUE(refused_naming(silent, "'imu_sensors'"));
    EXPECT_TRUE(refused_naming(silent, "'sensor.imu'"));
    const auto retagged = create(with_imu_streams(renamed));
    EXPECT_TRUE(refused_naming(retagged, "'imu_sensors'"));
    EXPECT_TRUE(refused_naming(retagged, "'sensor.imu'")) << "the standard tag, not the one declared";
}

// Stage 4, Task 6 (review focus 4): a pass sees the state it declares, and
// nothing else, through SubstepContext::state -- one view per declared access,
// in its own declaration order. An optional read of an absent module binds an
// absent view and orders nothing; a `before` edge orders like a mirrored
// `after`. Each developer pass below checks the size of ctx.state first, so a
// missing binding shows as a wrong count rather than an out-of-range read.
namespace {

using spade::modules::BindingKind;

// `tally`: one per_world counter, incremented by its pass once per substep in
// every world. It declares one access, so it sees one view.
std::size_t g_tally_views = 0;      // ctx.state.size() at the last tally pass
spade::StateView g_tally_view{};    // ctx.state[0] at the last tally pass
void tally_pass(const spade::physics::SubstepContext& ctx) noexcept {
    g_tally_views = ctx.state.size();
    if (ctx.state.size() != 1) return;
    g_tally_view = ctx.state[0];
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<TallyRow> rows = spade::world_rows<TallyRow>(ctx.state[0], w);
        if (!rows.empty()) ++rows[0].count;
    }
}
constexpr QuantityAccess kTallyAccess[] = {{"tally.tally_counts", Access::write}};
constexpr PassDecl kTallyPasses[] = {{.name = "count", .phase = Phase::forces, .access = kTallyAccess, .cpu = &tally_pass}};

[[nodiscard]] ModuleDesc tally_module() { return {.name = "tally", .passes = kTallyPasses, .state = kTallyArrays}; }

[[nodiscard]] spade::WorldSetDesc two_world_set() {
    const spade::WorldInstanceDesc one = one_body_world().worlds[0];
    return spade::WorldSetDesc{{one, one}};
}

// `copier`: copies tally's count into a per_world row of its own. It declares a
// core quantity, tally's array and its own array, so it sees three views: the
// first absent, whatever the set holds.
struct CopyRow {
    uint32_t seen;           // tally's count this substep, or kNothing
    uint32_t tally_present;  // ctx.state[1].present()
    uint32_t core_present;   // ctx.state[0].present()
    uint32_t views;          // ctx.state.size()
};
constexpr uint32_t kNothing = 0xFFFF'FFFFu;
void copy_pass(const spade::physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 3) return;
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<CopyRow> out = spade::world_rows<CopyRow>(ctx.state[2], w);
        if (out.empty()) continue;
        const std::span<const TallyRow> in = spade::world_rows<const TallyRow>(ctx.state[1], w);
        out[0].seen = in.empty() ? kNothing : in[0].count;
        out[0].tally_present = ctx.state[1].present() ? 1u : 0u;
        out[0].core_present = ctx.state[0].present() ? 1u : 0u;
        out[0].views = static_cast<uint32_t>(ctx.state.size());
    }
}
constexpr spade::modules::ArrayDecl kCopierArrays[] = {
    {.name = "copier_rows", .elem_size = spade::modules::row_size<CopyRow>()}};
constexpr QuantityAccess kCopyAccess[] = {
    {"world.params", Access::read}, {"tally.tally_counts", Access::read}, {"copier.copier_rows", Access::write}};
constexpr QuantityAccess kCopyOptionalAccess[] = {{"world.params", Access::read},
                                                  {.quantity = "tally.tally_counts", .access = Access::read, .optional = true},
                                                  {"copier.copier_rows", Access::write}};
constexpr QuantityAccess kCopyNothingAccess[] = {{"world.params", Access::read}, {"copier.copier_rows", Access::write}};
constexpr PassDecl kCopyPasses[] = {{.name = "copy", .phase = Phase::forces, .access = kCopyAccess, .cpu = &copy_pass}};
constexpr PassDecl kCopyOptionalPasses[] = {
    {.name = "copy", .phase = Phase::forces, .access = kCopyOptionalAccess, .cpu = &copy_pass}};
constexpr PassDecl kCopyNothingPasses[] = {
    {.name = "copy", .phase = Phase::forces, .access = kCopyNothingAccess, .cpu = &copy_pass}};

[[nodiscard]] ModuleDesc copier_module(std::span<const PassDecl> passes) {
    return {.name = "copier", .passes = passes, .state = kCopierArrays};
}

[[nodiscard]] std::ptrdiff_t position_of(const Names& order, std::string_view pass) {
    const auto it = std::ranges::find(order, std::string(pass));
    return it == order.end() ? -1 : it - order.begin();
}

[[nodiscard]] const spade::modules::CompiledPass* compiled_pass(const CompiledSchedule& s, std::string_view module,
                                                                std::string_view pass) {
    for (const spade::modules::CompiledPass& p : s.passes) {
        if (p.module == module && p.pass == pass) return &p;
    }
    return nullptr;
}

[[nodiscard]] uint32_t array_index(const CompiledSchedule& s, std::string_view name) {
    const auto it = std::ranges::find(s.arrays, std::string(name), &spade::modules::CompiledArray::name);
    return it == s.arrays.end() ? spade::modules::kNoArray : static_cast<uint32_t>(it - s.arrays.begin());
}

}  // namespace

TEST(ModuleViews, APassSeesItsOwnArrayPerWorld) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(tally_module());
    auto sim = spade::Simulation::create(two_world_set(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    g_tally_views = 0;
    ASSERT_TRUE(sim->step(3).has_value());
    EXPECT_EQ(g_tally_views, 1u) << "one view per declared access";
    EXPECT_EQ(g_tally_view.elem_size, sizeof(TallyRow));
    EXPECT_EQ(g_tally_view.world_count, 2u);
    EXPECT_EQ(g_tally_view.capacity_per_world, 1u) << "per_world";
    for (uint32_t w = 0; w < 2; ++w) {
        const auto rows = sim->module_rows<TallyRow>("tally_counts", w);
        ASSERT_TRUE(rows.has_value()) << rows.error().context;
        EXPECT_EQ((*rows)[0].count, 6u) << "3 steps x 2 substeps, world " << w;
    }
    EXPECT_TRUE(spade::world_rows<TallyRow>(g_tally_view, 2).empty()) << "a world outside the set";
    EXPECT_TRUE(spade::world_rows<uint64_t>(g_tally_view, 0).empty()) << "another row size";
    EXPECT_TRUE(spade::world_rows<TallyRow>(spade::StateView{}, 0).empty()) << "an absent view";
}

TEST(ModuleViews, ADevelopersStateSnapshotsAndRestoresBitwise) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(tally_module());
    auto source = spade::Simulation::create(two_world_set(), 2'000'000, 2, {}, set);
    auto twin = spade::Simulation::create(two_world_set(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(source.has_value()) << source.error().context;
    ASSERT_TRUE(twin.has_value()) << twin.error().context;
    ASSERT_TRUE(source->step(3).has_value());
    const auto blob = source->snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    ASSERT_TRUE(twin->restore(*blob).has_value());
    ASSERT_TRUE(source->step(2).has_value());
    ASSERT_TRUE(twin->step(2).has_value());
    EXPECT_EQ(spade::testing::state_digest(*twin), spade::testing::state_digest(*source));
    for (uint32_t w = 0; w < 2; ++w) {
        EXPECT_EQ((*source->module_rows<TallyRow>("tally_counts", w))[0].count, 10u) << "world " << w;
        EXPECT_EQ((*twin->module_rows<TallyRow>("tally_counts", w))[0].count, 10u) << "world " << w;
    }
}

TEST(ModuleViews, AReaderOfAnotherModulesArrayRunsAfterItsWriter) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(copier_module(kCopyPasses));  // before tally in the set
    set.push_back(tally_module());
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const Names order = names(sim->schedule());
    EXPECT_LT(position_of(order, "tally.count"), position_of(order, "copier.copy")) << "the read orders it";
    ASSERT_TRUE(sim->step(3).has_value());
    const auto rows = sim->module_rows<CopyRow>("copier_rows", 0);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    EXPECT_EQ((*rows)[0].views, 3u) << "one view per declared access";
    EXPECT_EQ((*rows)[0].seen, 6u) << "this substep's count, not the last one's";
    EXPECT_EQ((*rows)[0].tally_present, 1u);
    EXPECT_EQ((*rows)[0].core_present, 0u) << "a core quantity binds an absent view";
}

TEST(ModuleViews, AnOptionalReadOfAnAbsentModuleBindsNothing) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(copier_module(kCopyOptionalPasses));
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;

    // It orders nothing: the schedule (and so the identity) is the one a copier
    // that never named tally compiles to, which is the copier-free order plus
    // copier.copy.
    spade::modules::ModuleSet unnamed = spade::modules::standard_modules();
    unnamed.push_back(copier_module(kCopyNothingPasses));
    const auto plain = compile_schedule(unnamed);
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    EXPECT_EQ(names(*s), names(*plain));
    EXPECT_EQ(s->identity, plain->identity);
    Names without = names(*s);
    std::erase(without, std::string("copier.copy"));
    EXPECT_EQ(without, names(*compile_schedule(spade::modules::standard_modules())));
    const spade::modules::CompiledPass* copy = compiled_pass(*s, "copier", "copy");
    ASSERT_NE(copy, nullptr);
    ASSERT_EQ(copy->state.size(), 3u);
    EXPECT_EQ(copy->state[1].kind, BindingKind::absent);
    EXPECT_EQ(copy->state[2].kind, BindingKind::array);

    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->step(1).has_value());
    const auto rows = sim->module_rows<CopyRow>("copier_rows", 0);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    EXPECT_EQ((*rows)[0].views, 3u) << "the absent read still has its slot";
    EXPECT_EQ((*rows)[0].tally_present, 0u);
    EXPECT_EQ((*rows)[0].seen, kNothing) << "an absent view has no rows";
}

TEST(ModuleSchedule, AnOptionalAccessMayOnlyReadAnArrayThatExists) {
    using spade::modules::ArrayDecl;
    static constexpr QuantityAccess read_ok[] = {
        {.quantity = "tally.tally_counts", .access = Access::read, .optional = true}};
    static constexpr QuantityAccess write[] = {
        {.quantity = "tally.tally_counts", .access = Access::write, .optional = true}};
    static constexpr QuantityAccess add[] = {
        {.quantity = "tally.tally_counts", .access = Access::accumulate, .optional = true}};
    static constexpr QuantityAccess absent_write[] = {{.quantity = "ghost.x", .access = Access::write, .optional = true}};
    static constexpr QuantityAccess nope[] = {{.quantity = "tally.nope", .access = Access::read, .optional = true}};
    static constexpr QuantityAccess core[] = {{.quantity = "body.pose", .access = Access::read, .optional = true}};
    static constexpr QuantityAccess field[] = {{.quantity = "field.wind", .access = Access::read, .optional = true}};
    static constexpr QuantityAccess token[] = {
        {.quantity = "integrate.anything", .access = Access::read, .optional = true}};
    static constexpr QuantityAccess bare[] = {{.quantity = "ghost", .access = Access::read, .optional = true}};
    const auto with = [](std::span<const QuantityAccess> access) {
        const PassDecl probe[] = {{.name = "probe", .phase = Phase::sensors, .access = access, .cpu = &noop}};
        spade::modules::ModuleSet set = spade::modules::standard_modules();
        set.push_back({.name = "tally", .state = kTallyArrays});
        set.push_back({.name = "probe", .passes = probe});
        return compile_schedule(set);
    };
    const auto control = with(read_ok);
    ASSERT_TRUE(control.has_value()) << control.error().context;
    const spade::modules::CompiledPass* probe = compiled_pass(*control, "probe", "probe");
    ASSERT_NE(probe, nullptr);
    EXPECT_EQ(probe->state.size(), 1u);
    if (probe->state.size() == 1u) {
        EXPECT_EQ(probe->state[0].kind, BindingKind::array) << "present: it binds like a plain read";
        EXPECT_EQ(probe->state[0].index, array_index(*control, "tally_counts"));
    }

    EXPECT_TRUE(refused_naming(with(write), "tally.tally_counts")) << "an optional write";
    EXPECT_TRUE(refused_naming(with(add), "tally.tally_counts")) << "an optional accumulate";
    EXPECT_TRUE(refused_naming(with(absent_write), "ghost.x")) << "an optional write, module absent";
    EXPECT_TRUE(refused_naming(with(nope), "tally.nope")) << "present, but not one of its arrays";
    EXPECT_TRUE(refused_naming(with(core), "body.pose")) << "a core quantity is never absent";
    EXPECT_TRUE(refused_naming(with(field), "field.wind")) << "a field is not <module>.<array>";
    EXPECT_TRUE(refused_naming(with(token), "integrate.anything")) << "present and stateless: no array to bind";
    EXPECT_TRUE(refused_naming(with(bare), "ghost")) << "not <module>.<name>";
}

TEST(ModuleSchedule, ABeforeEdgeOrdersTwoWriters) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr std::string_view before_a[] = {"a.fix"};
    static constexpr PassDecl a[] = {{.name = "fix", .phase = Phase::constraints, .access = writes, .cpu = &noop}};
    static constexpr PassDecl b[] = {
        {.name = "fix", .phase = Phase::constraints, .access = writes, .before = before_a, .cpu = &noop}};
    static constexpr PassDecl b_no_edge[] = {
        {.name = "fix", .phase = Phase::constraints, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    const auto s = compile_schedule(set);
    EXPECT_TRUE(s.has_value()) << s.error().context;
    if (s) EXPECT_EQ(names(*s), (Names{"b.fix", "a.fix"})) << "the second in set order runs first";
    const ModuleDesc no_edge[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b_no_edge}};
    EXPECT_TRUE(refused_naming(compile_schedule(no_edge), "body.pose"));

    // An edge that closes a cycle is refused, naming the passes in it: two
    // `before` edges, and a `before` against an `after`.
    static constexpr std::string_view to_one[] = {"a.one"};
    static constexpr std::string_view to_two[] = {"b.two"};
    static constexpr PassDecl one_before_two[] = {{.name = "one", .phase = Phase::forces, .before = to_two, .cpu = &noop}};
    static constexpr PassDecl two_before_one[] = {{.name = "two", .phase = Phase::forces, .before = to_one, .cpu = &noop}};
    static constexpr PassDecl one_both[] = {
        {.name = "one", .phase = Phase::forces, .after = to_two, .before = to_two, .cpu = &noop}};
    static constexpr PassDecl two[] = {{.name = "two", .phase = Phase::forces, .cpu = &noop}};
    const ModuleDesc befores[] = {{.name = "a", .passes = one_before_two}, {.name = "b", .passes = two_before_one}};
    const ModuleDesc mixed[] = {{.name = "a", .passes = one_both}, {.name = "b", .passes = two}};
    for (const auto& cycle : {compile_schedule(befores), compile_schedule(mixed)}) {
        EXPECT_TRUE(refused_naming(cycle, "cycle"));
        EXPECT_TRUE(refused_naming(cycle, "a.one"));
        EXPECT_TRUE(refused_naming(cycle, "b.two"));
    }
}

TEST(ModuleSchedule, ABeforeEdgeMustNameAPassInThisOrALaterPhase) {
    static constexpr std::string_view to_nobody[] = {"nobody.here"};
    static constexpr std::string_view to_late[] = {"b.late"};
    static constexpr std::string_view to_early[] = {"b.early"};
    static constexpr PassDecl dangling[] = {{.name = "x", .phase = Phase::forces, .before = to_nobody, .cpu = &noop}};
    static constexpr PassDecl into_later[] = {{.name = "x", .phase = Phase::forces, .before = to_late, .cpu = &noop}};
    static constexpr PassDecl into_earlier[] = {{.name = "x", .phase = Phase::forces, .before = to_early, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "late", .phase = Phase::sensors, .cpu = &noop},
                                     {.name = "early", .phase = Phase::fields, .cpu = &noop}};
    const ModuleDesc s1[] = {{.name = "a", .passes = dangling}, {.name = "b", .passes = b}};
    const ModuleDesc s2[] = {{.name = "a", .passes = into_earlier}, {.name = "b", .passes = b}};
    const ModuleDesc s3[] = {{.name = "a", .passes = into_later}, {.name = "b", .passes = b}};
    EXPECT_TRUE(refused_naming(compile_schedule(s1), "nobody.here"));
    EXPECT_TRUE(refused_naming(compile_schedule(s2), "b.early"));
    EXPECT_TRUE(compile_schedule(s3).has_value()) << "an edge into a later phase is already satisfied";
}

// Physics' input 5 (propulsion-rows plan section 4), declared as the stage-4
// plan writes it: rotor.forces reads propulsion.motors optionally, and
// propulsion.drive, which also writes rotor.rotors, runs before rotor.forces.
// The one change is the initializer: C++ cannot mix positional and designated
// clauses, so the optional read is spelt with designators throughout.
TEST(ModuleSchedule, PhysicsInputFiveDeclaresAsWritten) {
    struct MotorRow {
        float duty;
        float _p[3];
    };
    static constexpr spade::modules::ArrayDecl motors[] = {{.name = "motors",
                                                            .elem_size = spade::modules::row_size<MotorRow>(),
                                                            .extent = Extent::per_row,
                                                            .owner = "rotors",
                                                            .depth = 1}};
    static constexpr QuantityAccess drive_access[] = {{"propulsion.motors", Access::write},
                                                      {"rotor.rotors", Access::write}};
    static constexpr std::string_view before_rotor[] = {"rotor.forces"};
    static constexpr PassDecl drive[] = {
        {.name = "drive", .phase = Phase::forces, .access = drive_access, .before = before_rotor, .cpu = &noop}};
    static constexpr PassDecl drive_no_edge[] = {
        {.name = "drive", .phase = Phase::forces, .access = drive_access, .cpu = &noop}};

    // The standard rotor pass, with the optional read appended to its access.
    spade::modules::ModuleSet standard = spade::modules::standard_modules();
    const auto rotor = std::ranges::find(standard, std::string_view("rotor"), &ModuleDesc::name);
    ASSERT_NE(rotor, standard.end());
    ASSERT_EQ(rotor->passes.size(), 1u);
    std::vector<QuantityAccess> rotor_access(rotor->passes[0].access.begin(), rotor->passes[0].access.end());
    const std::size_t optional_slot = rotor_access.size();
    rotor_access.push_back({.quantity = "propulsion.motors", .access = Access::read, .optional = true});
    std::array<PassDecl, 1> rotor_pass{rotor->passes[0]};
    rotor_pass[0].access = rotor_access;
    rotor->passes = rotor_pass;

    // Without propulsion: today's order and today's identity.
    const auto alone = compile_schedule(standard);
    ASSERT_TRUE(alone.has_value()) << alone.error().context;
    EXPECT_EQ(names(*alone), names(*compile_schedule(spade::modules::standard_modules())));
    EXPECT_EQ(alone->identity, kStandardIdentity);
    const spade::modules::CompiledPass* forces = compiled_pass(*alone, "rotor", "forces");
    ASSERT_NE(forces, nullptr);
    ASSERT_EQ(forces->state.size(), rotor_access.size());
    EXPECT_EQ(forces->state[optional_slot].kind, BindingKind::absent);

    // With it: drive runs immediately before rotor.forces, and the motors bind.
    spade::modules::ModuleSet with = standard;
    with.push_back({.name = "propulsion", .passes = drive, .state = motors});
    const auto s = compile_schedule(with);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    const Names order = names(*s);
    const std::ptrdiff_t at = position_of(order, "rotor.forces");
    ASSERT_GT(at, 0);
    EXPECT_EQ(order[static_cast<std::size_t>(at) - 1], "propulsion.drive");
    forces = compiled_pass(*s, "rotor", "forces");
    ASSERT_NE(forces, nullptr);
    ASSERT_EQ(forces->state.size(), rotor_access.size());
    EXPECT_EQ(forces->state[optional_slot].kind, BindingKind::array);
    EXPECT_EQ(forces->state[optional_slot].index, array_index(*s, "motors"));
    const spade::modules::CompiledPass* driver = compiled_pass(*s, "propulsion", "drive");
    ASSERT_NE(driver, nullptr);
    ASSERT_EQ(driver->state.size(), 2u);
    EXPECT_EQ(driver->state[1].index, array_index(*s, "rotors")) << "another module's array";

    // Without the edge the two writers of rotor.rotors are refused.
    spade::modules::ModuleSet unordered = standard;
    unordered.push_back({.name = "propulsion", .passes = drive_no_edge, .state = motors});
    EXPECT_TRUE(refused_naming(compile_schedule(unordered), "rotor.rotors"));
}
