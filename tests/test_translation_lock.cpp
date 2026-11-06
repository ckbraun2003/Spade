// test_translation_lock.cpp -- the translation lock (module API stage 6;
// docs/design/core/plans/2026-11-05-module-api-stage6-plan.md; spec section 9).
//
// Task 1: the lock module, its rows and its typed calls (this part).
// Task 2: Integrate honours the lock on the CPU. Task 3: the held-quad golden.
// Task 5: the same on Vulkan (GPU cases skip when no device is admitted).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <bit>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "compute/optional_bindings.hpp"
#include "contact_set_fakes.hpp"
#include "physics/lock.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/layout.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"

namespace {

using spade::BodyRef;
using spade::BodyState;
using spade::Result;
using spade::Simulation;
using spade::physics::kLockAllAxes;
using spade::physics::kLockAxisX;
using spade::physics::kLockAxisY;
using spade::physics::kLockAxisZ;
using spade::physics::LockRow;
using spade::test::bare_body;
using spade::test::floor_world_set;
using spade::test::row_of;

constexpr uint64_t kDt = 1'000'000;
constexpr uint32_t kSubsteps = 2;
// The standard set's pinned identity (test_module_schedule.cpp's kStandardIdentity): a lock set must not move it.
constexpr uint64_t kStandardIdentity = 0x3303cfc86821f502ULL;

[[nodiscard]] spade::modules::ModuleSet standard_plus_lock() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::lock_module());
    return set;
}

[[nodiscard]] Simulation make_sim(const spade::modules::ModuleSet& set, uint32_t bodies = 4) {
    const Result<spade::WorldSetDesc> world = floor_world_set(bodies);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    return Simulation::create(*world, kDt, kSubsteps, {}, set).value();
}

// A body spawned and flushed, so it is live and its row is initialized.
[[nodiscard]] BodyRef spawn_flushed(Simulation& sim, glm::vec3 pos = glm::vec3(0.0f, 3.0f, 0.0f)) {
    const BodyRef ref = sim.spawn(0, bare_body(pos)).value();
    EXPECT_TRUE(sim.flush_structural().has_value());
    return ref;
}

[[nodiscard]] std::array<uint32_t, 3> bits(const glm::vec3& v) {
    std::array<uint32_t, 3> out{};
    std::memcpy(out.data(), &v, sizeof(v));
    return out;
}

[[nodiscard]] LockRow lock_row_of(const Simulation& sim, BodyRef ref) {
    const auto rows = sim.module_rows<LockRow>("lock_rows", 0);
    EXPECT_TRUE(rows.has_value()) << rows.error().context;
    return rows ? (*rows)[ref.slot] : LockRow{};
}

// A pose well away from the floor and from the spawn: not buried, in tolerance.
struct Pose {
    glm::vec3 pos{-1.0f, 2.5f, 0.75f};
    glm::quat orient = glm::normalize(glm::quat(0.9f, 0.1f, 0.3f, -0.2f));
    glm::vec3 vel{0.5f, 1.0f, -0.25f};
    glm::vec3 omega{0.2f, -0.3f, 0.4f};
};
[[nodiscard]] Result<void> set_to(Simulation& sim, BodyRef ref, const Pose& p = Pose{}) {
    return sim.set_body_state(ref, p.pos, p.orient, p.vel, p.omega);
}

}  // namespace

// ---------------------------------------------------------------------------
// Task 1: the module, its rows, the calls
// ---------------------------------------------------------------------------

TEST(TranslationLock, ACallIsRefusedWhenTheSetHasNoLockModule) {
    Simulation sim = make_sim(spade::modules::standard_modules());
    const BodyRef ref = spawn_flushed(sim);
    for (const auto& r : {sim.lock_translation(ref, kLockAllAxes), sim.unlock(ref)}) {
        ASSERT_FALSE(r.has_value());
        EXPECT_EQ(r.error().code, spade::Code::unavailable) << r.error().context;
        EXPECT_NE(r.error().context.find("'lock'"), std::string::npos) << r.error().context;
    }
    EXPECT_EQ(sim.pending_structural_ops(), 0u) << "a refused call queues nothing";
}

TEST(TranslationLock, ALockWritesItsRowAtTheNextBoundaryAndUnlockClearsIt) {
    Simulation sim = make_sim(standard_plus_lock());
    const glm::vec3 where(0.25f, 3.0f, -0.5f);
    const BodyRef ref = spawn_flushed(sim, where);

    ASSERT_TRUE(sim.lock_translation(ref, kLockAxisX | kLockAxisZ).has_value());
    EXPECT_EQ(sim.pending_structural_ops(), 1u);
    EXPECT_EQ(lock_row_of(sim, ref).axes, 0u) << "the row is written at the boundary, not at the call";
    ASSERT_TRUE(sim.flush_structural().has_value());
    EXPECT_EQ(lock_row_of(sim, ref).axes, kLockAxisX | kLockAxisZ);
    EXPECT_EQ(bits(lock_row_of(sim, ref).anchor), bits(where)) << "the anchor is the body's position at the call";

    ASSERT_TRUE(sim.unlock(ref).has_value());
    ASSERT_TRUE(sim.flush_structural().has_value());
    EXPECT_EQ(lock_row_of(sim, ref).axes, 0u);
    EXPECT_EQ(bits(lock_row_of(sim, ref).anchor), bits(glm::vec3(0.0f))) << "an unlock leaves a zero row";
}

TEST(TranslationLock, ARefusalNamesTheCallAndQueuesNothing) {
    Simulation sim = make_sim(standard_plus_lock());
    const BodyRef live = spawn_flushed(sim);
    for (const uint32_t axes : {0u, 8u, 0xFFFFFFFFu}) {
        const auto r = sim.lock_translation(live, axes);
        ASSERT_FALSE(r.has_value()) << "axes " << axes;
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << r.error().context;
        EXPECT_NE(r.error().context.find("lock_translation"), std::string::npos) << r.error().context;
    }
    // Spawned this boundary, not flushed: its position is not final and a queued row init would not reach it.
    const BodyRef fresh = sim.spawn(0, bare_body(glm::vec3(1.0f, 3.0f, 0.0f))).value();
    for (const auto& r : {sim.lock_translation(fresh, kLockAllAxes), sim.unlock(fresh)}) {
        ASSERT_FALSE(r.has_value());
        EXPECT_EQ(r.error().code, spade::Code::not_found) << r.error().context;
        EXPECT_NE(r.error().context.find("not yet flushed"), std::string::npos) << r.error().context;
    }
    ASSERT_TRUE(sim.flush_structural().has_value());
    // A ref whose despawn is queued is dead the moment despawn() returns.
    ASSERT_TRUE(sim.despawn(live).has_value());
    const std::size_t after_despawn = sim.pending_structural_ops();
    for (const auto& r : {sim.lock_translation(live, kLockAllAxes), sim.unlock(live)}) {
        ASSERT_FALSE(r.has_value());
        EXPECT_EQ(r.error().code, spade::Code::not_found) << r.error().context;
    }
    EXPECT_EQ(sim.pending_structural_ops(), after_despawn) << "a refused call queues nothing";
}

TEST(TranslationLock, ADespawnFreesTheRowAndARespawnIsFree) {
    Simulation sim = make_sim(standard_plus_lock());
    const BodyRef first = spawn_flushed(sim);
    ASSERT_TRUE(sim.lock_translation(first, kLockAllAxes).has_value());
    ASSERT_TRUE(sim.flush_structural().has_value());
    ASSERT_EQ(lock_row_of(sim, first).axes, kLockAllAxes);

    ASSERT_TRUE(sim.despawn(first).has_value());
    ASSERT_TRUE(sim.flush_structural().has_value());
    EXPECT_EQ(lock_row_of(sim, first).axes, 0u) << "the despawn cascade zeroes a freed per_body row";

    const BodyRef second = spawn_flushed(sim, glm::vec3(2.0f, 3.0f, 0.0f));
    EXPECT_EQ(lock_row_of(sim, second).axes, 0u) << "a respawned body is free";
}

TEST(TranslationLock, TheStandardSetKeepsItsIdentityAndItsWalk) {
    Simulation sim = make_sim(spade::modules::standard_modules());
    EXPECT_EQ(sim.schedule().identity, kStandardIdentity);
    std::size_t arrays = 0;
    sim.arenas().registry().for_each_array([&arrays](const spade::RegisteredArray&) { ++arrays; });
    EXPECT_EQ(arrays, 22u) << "the standard walk is 22 entries";
}

TEST(TranslationLock, ASetWithLockHasItsOwnIdentityAndWalk) {
    Simulation sim = make_sim(standard_plus_lock());
    EXPECT_NE(sim.schedule().identity, kStandardIdentity) << "the lock module's name is spelt into the identity";
    std::size_t arrays = 0;
    bool has_rows = false;
    sim.arenas().registry().for_each_array([&](const spade::RegisteredArray& a) {
        ++arrays;
        has_rows = has_rows || std::string(a.name) == "lock_rows";
    });
    EXPECT_TRUE(has_rows);
    EXPECT_EQ(arrays, 23u) << "the standard walk and lock_rows (a per_body array has no map)";
}

TEST(TranslationLock, ALockedStateSurvivesASnapshotAndRestore) {
    Simulation source = make_sim(standard_plus_lock());
    const BodyRef ref = spawn_flushed(source, glm::vec3(0.5f, 3.0f, 0.25f));
    ASSERT_TRUE(source.lock_translation(ref, kLockAxisY | kLockAxisZ).has_value());
    ASSERT_TRUE(source.flush_structural().has_value());
    const auto blob = source.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    Simulation twin = make_sim(standard_plus_lock());
    ASSERT_TRUE(twin.restore(*blob).has_value());
    EXPECT_EQ(lock_row_of(twin, ref).axes, kLockAxisY | kLockAxisZ);
    EXPECT_EQ(bits(lock_row_of(twin, ref).anchor), bits(lock_row_of(source, ref).anchor));
    EXPECT_EQ(spade::testing::state_digest(twin), spade::testing::state_digest(source));
}

// A PIN, green at red: another module set (here, a lock the target lacks) is refused at restore, with the identity's
// message, as for another role choice (stage 5 Task 6). The lock module's name is spelt into the identity.
TEST(TranslationLock, RestoreIntoASetWithoutLockIsRefused) {
    Simulation source = make_sim(standard_plus_lock());
    (void)spawn_flushed(source);
    const auto blob = source.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    Simulation target = make_sim(spade::modules::standard_modules());
    const auto restored = target.restore(*blob);
    ASSERT_FALSE(restored.has_value());
    EXPECT_EQ(restored.error().code, spade::Code::invalid_argument);
    EXPECT_NE(restored.error().context.find("module set"), std::string::npos) << restored.error().context;
}

// ---------------------------------------------------------------------------
// set_body_state and the lock (plan decision 6; the lead's addition 1)
// ---------------------------------------------------------------------------

TEST(TranslationLock, SetBodyStateRefusesALockedBody) {
    Simulation sim = make_sim(standard_plus_lock());
    const BodyRef ref = spawn_flushed(sim);
    ASSERT_TRUE(sim.lock_translation(ref, kLockAxisX).has_value());
    ASSERT_TRUE(sim.flush_structural().has_value());
    const BodyState before = row_of(sim, ref);

    const auto r = set_to(sim, ref);
    ASSERT_FALSE(r.has_value()) << "Integrate would snap the pose back, so the call is refused, never skipped";
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << r.error().context;
    EXPECT_NE(r.error().context.find("locked on axes"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("next boundary"), std::string::npos)
        << "the message says an unlock applies at the next boundary: " << r.error().context;
    const BodyState after = row_of(sim, ref);
    EXPECT_EQ(std::memcmp(&before, &after, sizeof(BodyState)), 0) << "a refusal writes nothing";
}

TEST(TranslationLock, SetBodyStateRefusesWhileALockIsPending) {
    Simulation sim = make_sim(standard_plus_lock());
    const glm::vec3 where(0.25f, 3.0f, -0.5f);
    const BodyRef ref = spawn_flushed(sim, where);
    ASSERT_TRUE(sim.lock_translation(ref, kLockAllAxes).has_value());
    const BodyState before = row_of(sim, ref);

    // The anchor was taken at the call, and the row init cannot see BodyState: a re-pose before the flush would be
    // followed by the boundary writing the OLD anchor, and Integrate would snap the body back.
    const auto r = set_to(sim, ref);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << r.error().context;
    EXPECT_NE(r.error().context.find("pending"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("flush"), std::string::npos) << r.error().context;
    const BodyState after = row_of(sim, ref);
    EXPECT_EQ(std::memcmp(&before, &after, sizeof(BodyState)), 0) << "a refusal writes nothing";

    ASSERT_TRUE(sim.flush_structural().has_value()) << "the refused call did not disturb the queued lock";
    EXPECT_EQ(lock_row_of(sim, ref).axes, kLockAllAxes);
    EXPECT_EQ(bits(lock_row_of(sim, ref).anchor), bits(where)) << "the flush holds the ORIGINAL anchor";
}

TEST(TranslationLock, SetBodyStateRefusesWhileAnUnlockIsPending) {
    Simulation sim = make_sim(standard_plus_lock());
    const BodyRef ref = spawn_flushed(sim);
    ASSERT_TRUE(sim.lock_translation(ref, kLockAllAxes).has_value());
    ASSERT_TRUE(sim.flush_structural().has_value());
    ASSERT_TRUE(sim.unlock(ref).has_value());

    const auto r = set_to(sim, ref);
    ASSERT_FALSE(r.has_value()) << "the row is still locked until the boundary";
    EXPECT_NE(r.error().context.find("pending"), std::string::npos) << r.error().context;
    ASSERT_TRUE(sim.flush_structural().has_value());
    EXPECT_EQ(lock_row_of(sim, ref).axes, 0u);
    EXPECT_TRUE(set_to(sim, ref).has_value()) << "after the boundary the body is free to re-pose";
}

// A PIN, green at red: a body that never had a lock, in a set that holds the module, re-poses as before.
TEST(TranslationLock, SetBodyStateOnAnUnlockedBodyInALockSetIsUnchanged) {
    Simulation sim = make_sim(standard_plus_lock());
    const BodyRef ref = spawn_flushed(sim);
    EXPECT_TRUE(set_to(sim, ref).has_value());
}

// ---------------------------------------------------------------------------
// Task 2: Integrate honours the lock on the CPU
// ---------------------------------------------------------------------------
namespace {

[[nodiscard]] Simulation make_sim_substeps(const spade::modules::ModuleSet& set, uint32_t substeps) {
    const Result<spade::WorldSetDesc> world = floor_world_set(4);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    return Simulation::create(*world, kDt, substeps, {}, set).value();
}

[[nodiscard]] spade::BodySpawn moving_body(glm::vec3 pos, glm::vec3 vel) {
    spade::BodySpawn body = bare_body(pos);  // mass 0.8: not a power of two
    body.vel = vel;
    return body;
}

[[nodiscard]] BodyRef spawn_body(Simulation& sim, const spade::BodySpawn& body) {
    const BodyRef ref = sim.spawn(0, body).value();
    EXPECT_TRUE(sim.flush_structural().has_value());
    return ref;
}

const glm::vec3 kPlusZero(0.0f);

}  // namespace

TEST(TranslationLock, AFullLockHoldsPositionAndVelocityBitwise) {
    Simulation sim = make_sim(standard_plus_lock());
    const glm::vec3 anchor(0.0f, 3.0f, 0.0f);
    const BodyRef ref = spawn_body(sim, moving_body(anchor, glm::vec3(0.5f, -1.0f, 0.25f)));
    ASSERT_TRUE(sim.lock_translation(ref, kLockAllAxes).has_value());
    for (int s = 0; s < 200; ++s) {
        ASSERT_TRUE(sim.step(1).has_value());
        const BodyState b = row_of(sim, ref);
        ASSERT_EQ(bits(b.pos), bits(anchor)) << "step " << s;
        ASSERT_EQ(bits(b.vel), bits(kPlusZero)) << "step " << s << ": +0.0f on every axis";
    }
}

TEST(TranslationLock, AHeldBodysImuReadsMinusRTgFromAnyMass) {
    // One substep per step, so the orientation the specific force is taken from is the one before the step.
    Simulation sim = make_sim_substeps(standard_plus_lock(), 1);
    spade::BodySpawn spawn = bare_body(glm::vec3(0.0f, 3.0f, 0.0f));
    spawn.orient = glm::normalize(glm::quat(0.9f, 0.1f, 0.3f, -0.2f));  // not axis-aligned
    const BodyRef ref = spawn_body(sim, spawn);
    ASSERT_TRUE(sim.lock_translation(ref, kLockAllAxes).has_value());
    const glm::vec3 gravity(0.0f, -9.80665f, 0.0f);  // the floor world's
    for (int s = 0; s < 5; ++s) {
        const glm::quat before = row_of(sim, ref).orient;
        ASSERT_TRUE(sim.step(1).has_value());
        const glm::vec3 expected = glm::conjugate(before) * (-gravity);
        EXPECT_EQ(bits(row_of(sim, ref).specific_force), bits(expected)) << "step " << s;
    }
}

TEST(TranslationLock, TheYFreeRailClimbsLikeAFreeBodyAndHoldsXAndZ) {
    Simulation sim = make_sim(standard_plus_lock());
    const glm::vec3 a0(0.0f, 2.0f, 0.0f);
    const glm::vec3 b0(5.0f, 2.0f, 0.0f);
    const BodyRef railed = spawn_body(sim, moving_body(a0, glm::vec3(0.4f, 0.1f, -0.3f)));
    const BodyRef free_twin = spawn_body(sim, moving_body(b0, glm::vec3(0.4f, 0.1f, -0.3f)));
    ASSERT_TRUE(sim.lock_translation(railed, kLockAxisX | kLockAxisZ).has_value());
    for (int s = 0; s < 100; ++s) {
        for (const BodyRef r : {railed, free_twin}) {
            ASSERT_TRUE(sim.apply_wrench(r, glm::vec3(0.0f, 12.0f, 0.0f), glm::vec3(0.0f)).has_value());
        }
        ASSERT_TRUE(sim.step(1).has_value());
        const BodyState a = row_of(sim, railed);
        const BodyState b = row_of(sim, free_twin);
        ASSERT_EQ(a.pos.y, b.pos.y) << "the free axis is today's arithmetic, step " << s;
        ASSERT_EQ(a.vel.y, b.vel.y) << "step " << s;
        ASSERT_EQ(a.pos.x, a0.x) << "step " << s;
        ASSERT_EQ(a.pos.z, a0.z) << "step " << s;
        ASSERT_EQ(bits(glm::vec3(a.vel.x, 0.0f, a.vel.z)), bits(kPlusZero)) << "step " << s;
    }
    EXPECT_GT(row_of(sim, railed).pos.y, a0.y) << "the rail climbed: a 12 N lift on 0.8 kg beats gravity";
}

// A PIN, green at red: rotation is the same arithmetic as a free body's, so it holds before and after the lock.
TEST(TranslationLock, RotationStaysFree) {
    Simulation sim = make_sim(standard_plus_lock());
    spade::BodySpawn spin = moving_body(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f));
    spin.omega_body = glm::vec3(0.3f, -0.2f, 0.5f);
    const BodyRef held = spawn_body(sim, spin);
    spin.pos = glm::vec3(5.0f, 2.0f, 0.0f);
    const BodyRef free_twin = spawn_body(sim, spin);
    ASSERT_TRUE(sim.lock_translation(held, kLockAllAxes).has_value());
    for (int s = 0; s < 100; ++s) {
        for (const BodyRef r : {held, free_twin}) {
            ASSERT_TRUE(sim.apply_wrench(r, glm::vec3(0.0f), glm::vec3(0.02f, 0.01f, -0.015f)).has_value());
        }
        ASSERT_TRUE(sim.step(1).has_value());
        const BodyState a = row_of(sim, held);
        const BodyState b = row_of(sim, free_twin);
        ASSERT_EQ(bits(a.omega_body), bits(b.omega_body)) << "step " << s;
        ASSERT_EQ(std::memcmp(&a.orient, &b.orient, sizeof(a.orient)), 0) << "step " << s;
    }
    EXPECT_NE(row_of(sim, held).omega_body, glm::vec3(0.3f, -0.2f, 0.5f)) << "the torque did rotate it";
}

// THE LIMIT (plan decision 5, spec section 9): the contact solve treats a locked body as free, with its finite mass,
// and the lock then discards that body's response. A free body that hits a held one gets neither a wall's response
// nor a free body's: it gets the response it would get from an UNLOCKED body, which is what is pinned here. The free
// body starts overlapping the other, so contact acts in the first substep, when the two runs' states are identical.
TEST(TranslationLock, ContactAgainstALockedBodyIsTodaysBehavior) {
    struct Run {
        BodyState held;
        BodyState hitter;
    };
    const auto run = [](bool locked) {
        Simulation sim = make_sim_substeps(standard_plus_lock(), 1);
        const BodyRef held = spawn_body(sim, moving_body(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f)));
        const BodyRef hitter = spawn_body(sim, moving_body(glm::vec3(0.0f, 1.15f, 0.0f), glm::vec3(0.0f, -2.0f, 0.0f)));
        if (locked) {
            EXPECT_TRUE(sim.lock_translation(held, kLockAllAxes).has_value());
        }
        EXPECT_TRUE(sim.step(1).has_value());
        return Run{row_of(sim, held), row_of(sim, hitter)};
    };
    const Run locked = run(true);
    const Run unlocked = run(false);
    EXPECT_NE(bits(unlocked.held.vel), bits(kPlusZero)) << "contact acted on the free body: the premise of this test";
    EXPECT_EQ(bits(locked.held.pos), bits(glm::vec3(0.0f, 1.0f, 0.0f))) << "the lock discarded the held body's response";
    EXPECT_EQ(bits(locked.held.vel), bits(kPlusZero));
    EXPECT_EQ(bits(locked.hitter.vel), bits(unlocked.hitter.vel)) << "the solve does not know about the lock";
    EXPECT_EQ(bits(locked.hitter.pos), bits(unlocked.hitter.pos));
}

TEST(TranslationLock, AHeldBodyOnTheGroundIsOverwrittenOnItsLockedAxes) {
    Simulation sim = make_sim(standard_plus_lock());
    const glm::vec3 anchor(0.0f, 0.05f, 0.0f);  // inside the floor's proxy radius: static contact pushes it up
    const BodyRef ref = spawn_body(sim, moving_body(anchor, glm::vec3(0.0f)));
    ASSERT_TRUE(sim.lock_translation(ref, kLockAllAxes).has_value());
    for (int s = 0; s < 50; ++s) {
        ASSERT_TRUE(sim.step(1).has_value());
        const BodyState b = row_of(sim, ref);
        ASSERT_EQ(bits(b.pos), bits(anchor)) << "contact's correction never survives a substep, step " << s;
        ASSERT_EQ(bits(b.vel), bits(kPlusZero)) << "step " << s;
    }
}

// A PIN, green at red. A body that never had a lock, in a set that holds the module, computes today's bits: its rows are those of the same
// run in the standard set. (The walk differs by lock_rows, so the comparison is body row by body row.)
TEST(TranslationLock, AnUnlockedBodyInALockSetIsBitIdenticalToTheStandardSet) {
    const auto run = [](const spade::modules::ModuleSet& set) {
        Simulation sim = make_sim(set);
        const BodyRef a = spawn_body(sim, moving_body(glm::vec3(0.0f, 0.3f, 0.0f), glm::vec3(0.4f, -2.0f, 0.1f)));
        const BodyRef b = spawn_body(sim, moving_body(glm::vec3(0.5f, 0.12f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
        std::vector<BodyState> rows;
        for (int s = 0; s < 150; ++s) {
            EXPECT_TRUE(sim.apply_wrench(a, glm::vec3(0.3f, 1.0f, -0.2f), glm::vec3(0.01f, 0.0f, 0.02f)).has_value());
            EXPECT_TRUE(sim.step(1).has_value());
            rows.push_back(row_of(sim, a));
            rows.push_back(row_of(sim, b));
        }
        return rows;
    };
    const std::vector<BodyState> standard = run(spade::modules::standard_modules());
    const std::vector<BodyState> with_lock = run(standard_plus_lock());
    ASSERT_EQ(standard.size(), with_lock.size());
    for (std::size_t i = 0; i < standard.size(); ++i) {
        ASSERT_EQ(std::memcmp(&standard[i], &with_lock[i], sizeof(BodyState)), 0) << "row " << i;
    }
}

TEST(TranslationLock, UnlockReleasesFromTheAnchorAtRest) {
    Simulation held_sim = make_sim(standard_plus_lock());
    const glm::vec3 anchor(0.0f, 3.0f, 0.0f);
    const BodyRef held = spawn_body(held_sim, moving_body(anchor, glm::vec3(0.5f, -1.0f, 0.25f)));
    ASSERT_TRUE(held_sim.lock_translation(held, kLockAllAxes).has_value());
    ASSERT_TRUE(held_sim.step(10).has_value());
    ASSERT_TRUE(held_sim.unlock(held).has_value());
    ASSERT_TRUE(held_sim.step(5).has_value());  // the first step's flush writes the free row

    Simulation twin_sim = make_sim(standard_plus_lock());
    const BodyRef twin = spawn_body(twin_sim, moving_body(anchor, glm::vec3(0.0f)));
    ASSERT_TRUE(twin_sim.step(5).has_value());

    const BodyState a = row_of(held_sim, held);
    const BodyState b = row_of(twin_sim, twin);
    EXPECT_EQ(bits(a.pos), bits(b.pos)) << "free fall from the anchor with vel 0";
    EXPECT_EQ(bits(a.vel), bits(b.vel));
}

// ---------------------------------------------------------------------------
// Task 3: the held-quad golden (CPU reference). ADDED, never regenerated.
//
// WHAT IT COVERS: a quadrotor of a mass that is not a power of two, held on all three axes at its spawn position,
// flying a collective-and-attitude script (four rotor-command changes and a body-torque pulse, so its attitude moves
// and its rotors spin up and down) for 450 steps of 2 ms at 2 substeps. Under the lock the body's position is bitwise
// its anchor and its velocity +0.0f at every step, whatever the rotors do; the final state digest (the whole registry
// walk, testing/replay.hpp's state_digest) is the golden. A mid-run snapshot at tick 225, restored into a fresh lock
// set, finishes on the same digest.
//
// THE DIGEST ACROSS COMPILERS. It is a literal, pinned from the MSVC run (both presets, twice). The corpus goldens hold
// under gcc too (TD-1), and the two MSVC presets do not show that: THE HELD-QUAD DIGEST IS CHECKED UNDER GCC BY THE
// BATCH GATE'S -NoSeed LEG, AND BY NOTHING EARLIER. A mismatch there stops the merge and goes to the lead; it is never
// re-pinned to gcc's number.
// ---------------------------------------------------------------------------
namespace {

// PINNED AT THE SLOT (stage 6 Task 3): 0 until the first run prints it; the failure message carries the number.
constexpr uint64_t kHeldQuadDigest = 0;
constexpr int kHeldQuadSteps = 450;
constexpr uint64_t kQuadDt = 2'000'000;

struct HeldQuad {
    Simulation sim;
    spade::VehicleRef ref;
    glm::vec3 anchor;
};

// A fresh lock set with the quad's model registered; the quad is spawned and held only when `spawn` is true (a restore
// target registers the model and spawns nothing).
[[nodiscard]] HeldQuad held_quad(uint32_t substeps, bool spawn = true) {
    Simulation sim = Simulation::create(contact_set_fakes::world(false), kQuadDt, substeps, {}, standard_plus_lock())
                         .value();
    const spade::ModelTypeId model = sim.register_model(contact_set_fakes::quad_of_mass(1.37f)).value();
    spade::VehicleRef ref{};
    glm::vec3 anchor(0.0f);
    if (spawn) {
        spade::VehicleSpawn where;
        where.pos = glm::vec3(0.0f, 5.0f, 0.0f);
        where.rotor_omega = 550.0f;
        ref = sim.spawn(0, model, where).value();
        EXPECT_TRUE(sim.flush_structural().has_value());
        anchor = row_of(sim, ref.body).pos;
        EXPECT_TRUE(sim.lock_translation(ref.body, kLockAllAxes).has_value());
    }
    return HeldQuad{std::move(sim), ref, anchor};
}

// The script, applied before step `t` (0-based): a collective-and-attitude sequence the lock must be indifferent to.
void fly(Simulation& sim, const spade::VehicleRef& ref, int t) {
    static constexpr std::array<std::array<float, 4>, 4> kCommands = {{{550.0f, 550.0f, 550.0f, 550.0f},
                                                                       {640.0f, 640.0f, 640.0f, 640.0f},
                                                                       {620.0f, 560.0f, 620.0f, 560.0f},
                                                                       {500.0f, 600.0f, 500.0f, 600.0f}}};
    if (t % 100 == 0) {
        EXPECT_TRUE(sim.set_rotor_commands(ref, kCommands[(t / 100) % 4]).has_value());
    }
    if (t >= 150 && t < 170) {
        EXPECT_TRUE(sim.apply_wrench(ref.body, glm::vec3(0.0f), glm::vec3(0.05f, 0.02f, 0.03f)).has_value());
    }
}

[[nodiscard]] std::string hex64(uint64_t v) {
    char text[32];
    std::snprintf(text, sizeof(text), "0x%016llx", static_cast<unsigned long long>(v));
    return text;
}

}  // namespace

TEST(TranslationLockGolden, TheHeldQuadHoldsItsAnchorAndMatchesTheGolden) {
    HeldQuad q = held_quad(2);
    const glm::quat start = row_of(q.sim, q.ref.body).orient;
    for (int t = 0; t < kHeldQuadSteps; ++t) {
        fly(q.sim, q.ref, t);
        ASSERT_TRUE(q.sim.step(1).has_value());
        const BodyState b = row_of(q.sim, q.ref.body);
        ASSERT_EQ(bits(b.pos), bits(q.anchor)) << "step " << t;
        ASSERT_EQ(bits(b.vel), bits(glm::vec3(0.0f))) << "step " << t;
    }
    const glm::quat end = row_of(q.sim, q.ref.body).orient;
    EXPECT_NE(std::memcmp(&start, &end, sizeof(start)), 0)
        << "the script rotated the quad: rotation is free under the lock";
    const uint64_t digest = spade::testing::state_digest(q.sim);
    EXPECT_EQ(digest, kHeldQuadDigest) << "pin kHeldQuadDigest = " << hex64(digest);
}

// A PIN, green at red once Task 2 lands: the IMU reads +1 g up from the held quad, whatever its rotors do. One
// substep per step, so the orientation the specific force is taken from is the one before the step.
TEST(TranslationLockGolden, TheHeldQuadsSpecificForceIsMinusRTgEveryStep) {
    HeldQuad q = held_quad(1);
    const glm::vec3 gravity(0.0f, -9.80665f, 0.0f);  // spade::Environment's default
    for (int t = 0; t < kHeldQuadSteps; ++t) {
        fly(q.sim, q.ref, t);
        const glm::quat before = row_of(q.sim, q.ref.body).orient;
        ASSERT_TRUE(q.sim.step(1).has_value());
        const glm::vec3 expected = glm::conjugate(before) * (-gravity);
        ASSERT_EQ(bits(row_of(q.sim, q.ref.body).specific_force), bits(expected)) << "step " << t;
    }
}

TEST(TranslationLockGolden, AMidRunSnapshotFinishesOnTheUninterruptedDigest) {
    HeldQuad whole = held_quad(2);
    for (int t = 0; t < kHeldQuadSteps; ++t) {
        fly(whole.sim, whole.ref, t);
        ASSERT_TRUE(whole.sim.step(1).has_value());
    }
    const uint64_t uninterrupted = spade::testing::state_digest(whole.sim);

    HeldQuad first = held_quad(2);
    for (int t = 0; t < 225; ++t) {
        fly(first.sim, first.ref, t);
        ASSERT_TRUE(first.sim.step(1).has_value());
    }
    const auto blob = first.sim.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    HeldQuad resumed = held_quad(2, /*spawn=*/false);
    ASSERT_TRUE(resumed.sim.restore(*blob).has_value());
    // The vehicle's ref is re-derived from the restored state, as the quad_hover scenario's script does.
    const auto ref = resumed.sim.vehicle_ref_at(0, 0);
    ASSERT_TRUE(ref.has_value()) << ref.error().context;
    for (int t = 225; t < kHeldQuadSteps; ++t) {
        fly(resumed.sim, *ref, t);
        ASSERT_TRUE(resumed.sim.step(1).has_value());
    }
    EXPECT_EQ(spade::testing::state_digest(resumed.sim), uninterrupted);
}

// ---------------------------------------------------------------------------
// Task 4: the fixed binding's switch (CPU side; the mirror's half is in test_gpu_state_mirror.cpp)
// ---------------------------------------------------------------------------

TEST(OptionalBindings, ASetHasTheLockSwitchOnlyWhenItHoldsTheLockArray) {
    const auto standard = spade::modules::compile_schedule(spade::modules::standard_modules());
    const auto with_lock = spade::modules::compile_schedule(standard_plus_lock());
    ASSERT_TRUE(standard.has_value() && with_lock.has_value());
    EXPECT_EQ(spade::optional_switches(*standard), 0u) << "the standard set computes today's bits: no switch";
    EXPECT_EQ(spade::optional_switches(*with_lock), spade::compute::kSwitchLockRows);
}

// A PIN, green at red: the table's own shape. Every row names an array, owns exactly one switch bit, and no two share
// one; the dummy is at least a word.
TEST(OptionalBindings, EveryRowOwnsOneSwitchBitAndAnArrayName) {
    uint32_t seen = 0;
    for (const spade::compute::OptionalBinding& row : spade::compute::kOptionalBindings) {
        EXPECT_FALSE(row.name.empty());
        EXPECT_EQ(std::popcount(row.switch_bit), 1) << row.name;
        EXPECT_EQ(seen & row.switch_bit, 0u) << row.name << " shares a switch bit";
        seen |= row.switch_bit;
        EXPECT_GE(row.dummy_bytes, 4u) << row.name;
    }
}

// ---------------------------------------------------------------------------
// Task 5: the lock's grade (host side; the device half is test_gpu_translation_lock.cpp)
// ---------------------------------------------------------------------------

// A PIN, green at red: the lock declares CPU reference and Vulkan banded (plan decision 8; Physics' PHY-2 table is
// confirmed in the same task), and a set holding it keeps the standard set's grades.
TEST(TranslationLock, ALockSetsGradeIsReferenceOnTheCpuAndBandedOnVulkan) {
    const auto schedule = spade::modules::compile_schedule(standard_plus_lock());
    ASSERT_TRUE(schedule.has_value()) << schedule.error().context;
    const auto lock = std::ranges::find(schedule->modules, "lock", &spade::modules::CompiledModule::name);
    ASSERT_NE(lock, schedule->modules.end());
    EXPECT_EQ(lock->grade.cpu, spade::Grade::reference);
    EXPECT_EQ(lock->grade.vulkan, spade::Grade::banded);
    EXPECT_EQ(spade::modules::set_grade(*schedule, spade::compute::BackendKind::cpu).grade, spade::Grade::reference);
    EXPECT_EQ(spade::modules::set_grade(*schedule, spade::compute::BackendKind::vulkan).grade, spade::Grade::banded);
}

// ---------------------------------------------------------------------------
// The held body's IMU at TWO substeps (the sandbox drone box runs kSubsteps = 2; the lead's review of the plan).
//
// WHAT HOLDS. integrate writes the specific force once per SUBSTEP, from that substep's pre-update orientation, and
// the IMU samples the last one. At two substeps that is conj(q after substep 1) * (-g): a unit-quaternion rotation of
// -g, so its NORM is g (to fp32 rounding) at any substep count, and its DIRECTION is the one-substep-old orientation's
// -- it lags the step-start orientation by the rotation of ONE substep, not the whole step. At 1 substep the lag is
// zero and the reading is bit-equal to conj(q at step start) * (-g) (the test above).
// WHAT IT MEANS FOR A STAND. A level stand (identity orientation, no spin) reads (0, +g, 0) at any substep count,
// exactly: nothing rotates, so there is nothing to lag. That is the premise of the sandbox's
// TheStandMountedImuReadsPlusG (1e-4).
// ---------------------------------------------------------------------------

[[nodiscard]] float angle_between(const glm::vec3& a, const glm::vec3& b) {
    return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
}

TEST(TranslationLock, ALevelHeldBodyReadsPlusGExactlyAtTwoSubsteps) {
    Simulation sim = make_sim_substeps(standard_plus_lock(), 2);
    const BodyRef ref = spawn_body(sim, bare_body(glm::vec3(0.0f, 3.0f, 0.0f)));  // identity orientation, no spin
    ASSERT_TRUE(sim.lock_translation(ref, kLockAllAxes).has_value());
    for (int s = 0; s < 10; ++s) {
        ASSERT_TRUE(sim.step(1).has_value());
        const glm::vec3 f = row_of(sim, ref).specific_force;
        EXPECT_EQ(f.x, 0.0f) << "step " << s;
        EXPECT_EQ(f.y, 9.80665f) << "step " << s;
        EXPECT_EQ(f.z, 0.0f) << "step " << s;
    }
}

TEST(TranslationLock, ASpinningHeldBodyReadsGWithinOneSubstepOfRotationAtTwoSubsteps) {
    Simulation sim = make_sim_substeps(standard_plus_lock(), kSubsteps);
    spade::BodySpawn spawn = bare_body(glm::vec3(0.0f, 3.0f, 0.0f));
    spawn.orient = glm::normalize(glm::quat(0.9f, 0.1f, 0.3f, -0.2f));
    spawn.omega_body = glm::vec3(0.3f, -0.2f, 0.5f);
    const BodyRef ref = spawn_body(sim, spawn);
    ASSERT_TRUE(sim.lock_translation(ref, kLockAllAxes).has_value());
    const glm::vec3 gravity(0.0f, -9.80665f, 0.0f);
    const float h_sub = static_cast<float>(kDt) * 1e-9f / static_cast<float>(kSubsteps);
    for (int s = 0; s < 20; ++s) {
        const BodyState start = row_of(sim, ref);
        ASSERT_TRUE(sim.step(1).has_value());
        const glm::vec3 f = row_of(sim, ref).specific_force;
        EXPECT_NEAR(glm::length(f), 9.80665f, 2e-5f) << "the norm is g at any substep count, step " << s;
        const glm::vec3 at_start = glm::conjugate(start.orient) * (-gravity);
        // One substep of rotation, with a margin for omega changing under the gyroscopic term.
        EXPECT_LE(angle_between(f, at_start), glm::length(start.omega_body) * h_sub * 1.1f + 1e-5f) << "step " << s;
    }
}
