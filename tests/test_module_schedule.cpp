// test_module_schedule.cpp -- the module API's schedule compiler
// (docs/design/core/plans/2026-10-02-module-api-design.md, section 4).

#include <gtest/gtest.h>

#include <string>
#include <string_view>
#include <vector>

#include "compute/backend.hpp"
#include "physics/schedule.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/rotor.hpp"  // RotorRow complete: schedule.hpp only forward-declares it
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
    EXPECT_EQ(names(*s), (Names{"behaviors.kinematic", "dryden.advance", "rotor.forces", "drag.forces",
                                "behaviors.force", "static_contact.resolve", "dynamic_contact.resolve",
                                "integrate.integrate", "imu.synthesize", "gnss.synthesize"}));
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
                                                     {"dryden.state", Access::read}};
    static constexpr PassDecl drag_no_edge[] = {
        {.name = "forces", .phase = Phase::forces, .access = drag_access, .cpu = &spade::physics::pass_drag}};
    ASSERT_EQ(set[0].name, "drag");
    set[0].passes = drag_no_edge;

    const auto without = compile_schedule(set);
    ASSERT_TRUE(without.has_value()) << without.error().context;
    EXPECT_EQ(names(*without)[2], "drag.forces");
    EXPECT_EQ(names(*without)[3], "rotor.forces");
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

TEST(ModuleSimulation, ANonStandardSetOnVulkanIsRefusedUntilStage2) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "hover", .passes = kHoverPasses});
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2,
                                               spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan},
                                               set);
    ASSERT_FALSE(sim.has_value());
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
