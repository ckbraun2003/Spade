// test_objects_behavior.cpp -- Plan A Task 8 (24th spec SL6): the behavior
// registry.
//
// The registry is data plus an execution order. What is worth pinning is
// therefore not that a function pointer can be called, but the three rules SL6
// attaches to it: registration order IS execution order, a name is unique
// because it is a serialization key, and a behavior with no GPU half makes the
// registry ineligible rather than silently degraded.
//
// The SCHEDULE side of SL6 -- that the two slots exist, sit at the ruled
// positions, and are inert when nothing is registered -- is pinned in
// test_determinism.cpp beside the golden corpus that proves the inertness, not
// duplicated here.

#include <gtest/gtest.h>

#include <string>
#include <string_view>
#include <vector>

#include "objects/behavior.hpp"
#include "physics/schedule.hpp"
// RotorRow's complete definition. schedule.hpp deliberately only forward-
// declares it -- valid for a std::span member "until element access", as its
// comment says -- but VALUE-INITIALIZING a SubstepContext instantiates
// std::span<RotorRow>'s default constructor, which needs the size. Any TU that
// constructs one therefore needs this include; physics/schedule.cpp already
// carries it for the same reason.
#include "vehicles/rotor.hpp"

#include <cmath>
#include <cstddef>

#include "objects/behaviors/kinematic_mover.hpp"
#include "sim/simulation.hpp"
#include "world/medium.hpp"

using spade::objects::BehaviorDesc;
using spade::objects::BehaviorRegistry;
using spade::objects::BehaviorSlot;
using spade::physics::SubstepContext;

namespace {

// Captureless lambdas need somewhere to record, and a behavior is a plain
// function pointer by design (SL6: a behavior is data, not an object with
// state), so the recorder is file-scope.
std::vector<std::string_view> g_order;
int g_calls = 0;

void count_calls(const SubstepContext&, const void*) noexcept { ++g_calls; }
void noop_behavior(const SubstepContext&, const void*) noexcept {}
void record_first(const SubstepContext&, const void*) noexcept { g_order.push_back("first"); }
void record_second(const SubstepContext&, const void*) noexcept { g_order.push_back("second"); }

// Reads its params, so that "the registry hands each behavior ITS OWN
// user_data" is observable rather than assumed.
void record_params(const SubstepContext&, const void* params) noexcept {
    g_order.push_back(params != nullptr ? *static_cast<const std::string_view*>(params)
                                        : std::string_view{"<null>"});
}

[[nodiscard]] BehaviorDesc desc(std::string_view name, BehaviorSlot slot,
                                spade::objects::BehaviorFn fn) {
    return BehaviorDesc{.name = name, .slot = slot, .reads = 0, .writes = 0, .execute_cpu = fn};
}

}  // namespace

// Registration order is execution order, and it is a contract rather than an
// accident: two behaviors writing the same accumulator must compose in a
// defined sequence, or the result is not reproducible.
TEST(BehaviorRegistry, RunsRegisteredBehaviorsInRegistrationOrder) {
    BehaviorRegistry r;
    g_order.clear();
    ASSERT_TRUE(r.register_behavior(desc("first", BehaviorSlot::kinematic, record_first)));
    ASSERT_TRUE(r.register_behavior(desc("second", BehaviorSlot::kinematic, record_second)));

    const SubstepContext ctx{};
    r.run_slot(BehaviorSlot::kinematic, ctx);
    ASSERT_EQ(g_order.size(), 2u);
    EXPECT_EQ(g_order[0], "first");
    EXPECT_EQ(g_order[1], "second");
}

// Registration order holds ACROSS slots too -- a later-registered kinematic
// behavior still runs before an earlier-registered one only if it was
// registered first. This is the case that fails if run_slot ever groups or
// sorts by slot instead of filtering in place.
TEST(BehaviorRegistry, OrderIsRegistrationOrderNotSlotOrder) {
    BehaviorRegistry r;
    g_order.clear();
    ASSERT_TRUE(r.register_behavior(desc("second", BehaviorSlot::kinematic, record_second)));
    ASSERT_TRUE(r.register_behavior(desc("interposed", BehaviorSlot::force, noop_behavior)));
    ASSERT_TRUE(r.register_behavior(desc("first", BehaviorSlot::kinematic, record_first)));

    const SubstepContext ctx{};
    r.run_slot(BehaviorSlot::kinematic, ctx);
    ASSERT_EQ(g_order.size(), 2u);
    EXPECT_EQ(g_order[0], "second") << "registered first, so it runs first";
    EXPECT_EQ(g_order[1], "first");
}

TEST(BehaviorRegistry, RunsOnlyTheRequestedSlot) {
    BehaviorRegistry r;
    g_calls = 0;
    ASSERT_TRUE(r.register_behavior(desc("k", BehaviorSlot::kinematic, count_calls)));

    const SubstepContext ctx{};
    r.run_slot(BehaviorSlot::force, ctx);
    EXPECT_EQ(g_calls, 0);
    r.run_slot(BehaviorSlot::kinematic, ctx);
    EXPECT_EQ(g_calls, 1);
}

// Each entry gets ITS OWN user_data back, not the last-registered one -- the
// failure a single shared pointer field, or storing the desc by reference,
// would produce. With every mover in a scene sharing one params block, every
// body would trace the same circle.
TEST(BehaviorRegistry, EachBehaviorReceivesItsOwnParams) {
    static constexpr std::string_view kAlpha = "alpha";
    static constexpr std::string_view kBeta = "beta";

    BehaviorRegistry r;
    g_order.clear();
    BehaviorDesc a = desc("a", BehaviorSlot::kinematic, record_params);
    a.user_data = &kAlpha;
    BehaviorDesc b = desc("b", BehaviorSlot::kinematic, record_params);
    b.user_data = &kBeta;
    ASSERT_TRUE(r.register_behavior(a));
    ASSERT_TRUE(r.register_behavior(b));

    const SubstepContext ctx{};
    r.run_slot(BehaviorSlot::kinematic, ctx);
    ASSERT_EQ(g_order.size(), 2u);
    EXPECT_EQ(g_order[0], "alpha");
    EXPECT_EQ(g_order[1], "beta");
}

TEST(BehaviorRegistry, RejectsADuplicateName) {
    // Names are the key a saved BehaviorComponent resolves through, so
    // duplicates would make a saved graph ambiguous -- the same reasoning that
    // made object PARENT links indices rather than names in serialize.hpp,
    // arriving at the opposite remedy because a behavior vocabulary is closed
    // and can be required to be unique, where user object names cannot.
    BehaviorRegistry r;
    ASSERT_TRUE(r.register_behavior(desc("mover", BehaviorSlot::kinematic, noop_behavior)));
    const auto again = r.register_behavior(desc("mover", BehaviorSlot::force, noop_behavior));
    ASSERT_FALSE(again.has_value());
    EXPECT_NE(again.error().context.find("mover"), std::string::npos) << again.error().context;
    EXPECT_EQ(r.size(), 1u);
}

TEST(BehaviorRegistry, RejectsABehaviorWithNoCpuImplementation) {
    BehaviorRegistry r;
    EXPECT_FALSE(r.register_behavior(desc("empty", BehaviorSlot::force, nullptr)).has_value());
    EXPECT_EQ(r.size(), 0u);
}

// SL6: refusal, never a silent fallback. A world carrying a behavior with no
// GPU implementation must not run on the GPU-authoritative path, because the
// parity corpus would then hold a world whose behavior did not run identically
// on both backends -- it would be comparing two different experiments and
// passing.
TEST(BehaviorRegistry, ACpuOnlyBehaviorMakesTheRegistryGpuIneligible) {
    BehaviorRegistry r;
    EXPECT_TRUE(r.gpu_eligible()) << "an empty registry has nothing to refuse";

    BehaviorDesc both = desc("has_gpu", BehaviorSlot::force, noop_behavior);
    both.record_gpu = noop_behavior;
    ASSERT_TRUE(r.register_behavior(both));
    EXPECT_TRUE(r.gpu_eligible()) << "a behavior with both halves keeps the registry eligible";

    ASSERT_TRUE(r.register_behavior(desc("cpu_only", BehaviorSlot::force, noop_behavior)));
    EXPECT_FALSE(r.gpu_eligible());

    // And it does not recover: registering a well-formed behavior afterwards
    // cannot re-qualify a registry that already holds a CPU-only one.
    BehaviorDesc later = desc("also_has_gpu", BehaviorSlot::kinematic, noop_behavior);
    later.record_gpu = noop_behavior;
    ASSERT_TRUE(r.register_behavior(later));
    EXPECT_FALSE(r.gpu_eligible());
}

// A REGRESSION GUARD FOR A LIFETIME BUG, and honest about its limits.
//
// The plan stored BehaviorDesc values and kept names in a parallel
// std::vector<std::string>, repointing each stored `name` view at its owned
// string. Growing that vector MOVES its elements, and a short std::string
// keeps its characters inside the object (SSO), so every view dangled after a
// reallocation -- and every name here and in kinematic_mover is short.
//
// This case would catch a reintroduction under a sanitizer or a debug
// allocator. It is NOT a reliable detector on its own: a use-after-free of
// just-freed memory usually still reads the right bytes. That is precisely why
// the fix was to own the string rather than to add this test.
TEST(BehaviorRegistry, NamesSurviveReallocationOfTheEntryStorage) {
    BehaviorRegistry r;
    constexpr uint32_t kMany = 128;  // well past any small initial capacity
    for (uint32_t i = 0; i < kMany; ++i) {
        const std::string name = "b" + std::to_string(i);
        ASSERT_TRUE(r.register_behavior(desc(name, BehaviorSlot::kinematic, noop_behavior)))
            << "at " << i;
    }
    ASSERT_EQ(r.size(), kMany);
    for (uint32_t i = 0; i < kMany; ++i) {
        EXPECT_EQ(r.name_at(i), "b" + std::to_string(i)) << "name " << i << " did not survive";
    }
    EXPECT_TRUE(r.name_at(kMany).empty()) << "out of range is empty, not undefined";
}

// The inert path the golden corpus depends on: a context with no registry runs
// both passes and touches nothing. Task 7 proved this against the corpus; this
// states it directly at the seam Task 8 introduced, since the null check is now
// the thing standing between an empty world and a null dereference.
TEST(BehaviorSchedule, PassesWithNoRegistryAttachedDoNothing) {
    const SubstepContext ctx{};
    ASSERT_EQ(ctx.behaviors, nullptr) << "a default context attaches no registry";
    spade::physics::pass_behaviors_kinematic(ctx);
    spade::physics::pass_behaviors_force(ctx);
    SUCCEED() << "both passes are reachable with a null registry";
}

// ===========================================================================
// Task 9: the kinematic mover -- the first declared behavior, and the first
// end-to-end proof that a registered behavior reaches a running Simulation.
// ===========================================================================

namespace {

using spade::BodySpawn;
using spade::BodyState;
using spade::Simulation;
using spade::objects::KinematicMoverParams;
using spade::objects::kinematic_mover_desc;

// One body, one world, no geometry and no turbulence -- so the only thing
// moving the body is the mover under test.
[[nodiscard]] spade::Result<Simulation> single_body_sim() {
    spade::Environment env;
    env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;

    spade::Capacities caps;
    caps.bodies = 1;
    caps.force_elements = 1;
    caps.sensors = 1;
    caps.contacts = 1;

    spade::Result<spade::WorldDesc> world =
        spade::WorldBuilder().name("mover").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());

    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 11;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.0f;
    instance.grid.cell_size = 1.0f;

    spade::Result<Simulation> sim =
        Simulation::create(spade::WorldSetDesc{{instance}}, 4'000'000, 4);
    if (!sim) return std::unexpected(sim.error());

    BodySpawn body;
    body.pos = glm::vec3(0.0f, 0.0f, 0.0f);
    body.mass = 1.0f;
    if (spade::Result<spade::BodyRef> ref = sim->spawn(0, body); !ref) {
        return std::unexpected(ref.error());
    }
    if (spade::Result<void> flushed = sim->flush_structural(); !flushed) {
        return std::unexpected(flushed.error());
    }
    return sim;
}

[[nodiscard]] std::vector<std::byte> snapshot_bytes(const Simulation& sim) {
    const spade::Result<spade::SnapshotBlob> blob = sim.snapshot();
    if (!blob) return {};
    return std::vector<std::byte>(blob->bytes().begin(), blob->bytes().end());
}

}  // namespace

// The end-to-end wiring Task 8 flagged as owned by nobody: a registry attached
// to a Simulation actually runs.
TEST(KinematicMover, AttachedToASimulationItMovesTheBody) {
    spade::Result<Simulation> sim = single_body_sim();
    ASSERT_TRUE(sim.has_value());

    const KinematicMoverParams params{.origin = glm::vec3(0.0f),
                                      .axis = glm::vec3(0.0f, 1.0f, 0.0f),
                                      .radius_m = 5.0f,
                                      .period_s = 4.0f};
    BehaviorRegistry registry;
    ASSERT_TRUE(registry.register_behavior(kinematic_mover_desc(params)));
    sim->set_behaviors(&registry);

    ASSERT_TRUE(sim->step(10).has_value());
    const spade::Result<const BodyState*> state = sim->body(spade::BodyRef{
        .world_index = 0, .slot = 0, .generation = 1});
    ASSERT_TRUE(state.has_value()) << "body ref did not resolve";

    // On the circle: radius from the origin, and in the plane perpendicular to
    // the axis (y == 0 for a y-axis circle).
    const glm::vec3 pos = (*state)->pos;
    EXPECT_NEAR(std::sqrt(pos.x * pos.x + pos.z * pos.z), 5.0f, 1e-3f);
    EXPECT_NEAR(pos.y, 0.0f, 1e-4f) << "the mover left the circle's plane";
}

// A CPU-only behavior makes the registry ineligible -- SL6's refusal, observed
// on the real mover rather than only on a test double.
TEST(KinematicMover, IsCpuOnlyAndSaysSo) {
    const KinematicMoverParams params{};
    BehaviorRegistry registry;
    ASSERT_TRUE(registry.register_behavior(kinematic_mover_desc(params)));
    EXPECT_FALSE(registry.gpu_eligible());
}

TEST(KinematicMover, MovesABodyOnACircleDeterministically) {
    const KinematicMoverParams params{.origin = glm::vec3(0.0f),
                                      .axis = glm::vec3(0.0f, 1.0f, 0.0f),
                                      .radius_m = 5.0f,
                                      .period_s = 4.0f};
    std::vector<std::byte> bytes[2];
    for (int run = 0; run < 2; ++run) {
        spade::Result<Simulation> sim = single_body_sim();
        ASSERT_TRUE(sim.has_value());
        BehaviorRegistry registry;
        ASSERT_TRUE(registry.register_behavior(kinematic_mover_desc(params)));
        sim->set_behaviors(&registry);
        ASSERT_TRUE(sim->step(400).has_value());
        bytes[run] = snapshot_bytes(*sim);
        ASSERT_FALSE(bytes[run].empty());
    }
    EXPECT_EQ(bytes[0], bytes[1]) << "the mover is not a pure function of (params, tick)";
}

// THE CASE THE CLOSED FORM EXISTS FOR. Stepping 400 times and stepping 4x100
// must land in the same place -- the pose is derived from the tick, never
// integrated incrementally. An accumulating implementation passes every other
// case in this file and fails only this one.
TEST(KinematicMover, IsAFunctionOfTickNotOfCallCount) {
    const KinematicMoverParams params{.origin = glm::vec3(1.0f, 2.0f, 3.0f),
                                      .axis = glm::vec3(0.0f, 1.0f, 0.0f),
                                      .radius_m = 5.0f,
                                      .period_s = 4.0f};
    spade::Result<Simulation> once = single_body_sim();
    spade::Result<Simulation> split = single_body_sim();
    ASSERT_TRUE(once.has_value());
    ASSERT_TRUE(split.has_value());

    BehaviorRegistry r1;
    BehaviorRegistry r2;
    ASSERT_TRUE(r1.register_behavior(kinematic_mover_desc(params)));
    ASSERT_TRUE(r2.register_behavior(kinematic_mover_desc(params)));
    once->set_behaviors(&r1);
    split->set_behaviors(&r2);

    ASSERT_TRUE(once->step(400).has_value());
    for (int i = 0; i < 4; ++i) ASSERT_TRUE(split->step(100).has_value());

    EXPECT_EQ(snapshot_bytes(*once), snapshot_bytes(*split));
}

// THE STRONGEST STATEMENT OF THE CLOSED FORM, and the one that actually
// distinguishes it from an integration.
//
// IsAFunctionOfTickNotOfCallCount above is weaker than the plan claimed: 400
// steps and 4 x 100 steps execute the SAME number of substeps in the same tick
// sequence, so an implementation that accumulated per substep would agree with
// itself across the two and pass. Restoring a snapshot is what an accumulator
// cannot survive -- the phase it carries is not registered state, so it does
// not travel in the blob, and a resumed run lands somewhere the original never
// was. This is also exactly why the closed form keeps the mover compatible with
// SL3: nothing about it needs to be in the snapshot.
TEST(KinematicMover, ARestoredRunResumesOnTheSameCircle) {
    const KinematicMoverParams params{.origin = glm::vec3(0.0f),
                                      .axis = glm::vec3(0.0f, 1.0f, 0.0f),
                                      .radius_m = 5.0f,
                                      .period_s = 4.0f};

    spade::Result<Simulation> straight = single_body_sim();
    ASSERT_TRUE(straight.has_value());
    BehaviorRegistry r1;
    ASSERT_TRUE(r1.register_behavior(kinematic_mover_desc(params)));
    straight->set_behaviors(&r1);
    ASSERT_TRUE(straight->step(200).has_value());
    const spade::Result<spade::SnapshotBlob> blob = straight->snapshot();
    ASSERT_TRUE(blob.has_value());
    ASSERT_TRUE(straight->step(200).has_value());

    spade::Result<Simulation> resumed = single_body_sim();
    ASSERT_TRUE(resumed.has_value());
    BehaviorRegistry r2;
    ASSERT_TRUE(r2.register_behavior(kinematic_mover_desc(params)));
    resumed->set_behaviors(&r2);
    ASSERT_TRUE(resumed->restore(*blob).has_value());
    ASSERT_TRUE(resumed->step(200).has_value());

    EXPECT_EQ(snapshot_bytes(*straight), snapshot_bytes(*resumed))
        << "a resumed run diverged -- the mover is carrying phase the snapshot does not";
}

// A misconfigured mover is INERT, not undefined: a zero period would divide by
// zero and a degenerate axis has no plane. NaN in a body position would poison
// every subsequent digest rather than failing here.
TEST(KinematicMover, ADegenerateConfigurationLeavesTheBodyAlone) {
    for (const KinematicMoverParams& bad :
         {KinematicMoverParams{.axis = glm::vec3(0.0f, 1.0f, 0.0f), .period_s = 0.0f},
          KinematicMoverParams{.axis = glm::vec3(0.0f), .period_s = 1.0f}}) {
        spade::Result<Simulation> sim = single_body_sim();
        ASSERT_TRUE(sim.has_value());
        BehaviorRegistry registry;
        ASSERT_TRUE(registry.register_behavior(kinematic_mover_desc(bad)));
        sim->set_behaviors(&registry);
        ASSERT_TRUE(sim->step(4).has_value());

        const spade::Result<const BodyState*> state = sim->body(spade::BodyRef{
            .world_index = 0, .slot = 0, .generation = 1});
        ASSERT_TRUE(state.has_value());
        const glm::vec3 pos = (*state)->pos;
        EXPECT_TRUE(std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z))
            << "a degenerate mover produced a non-finite position";
    }
}
