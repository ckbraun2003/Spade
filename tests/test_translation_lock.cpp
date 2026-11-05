// test_translation_lock.cpp -- the translation lock (module API stage 6;
// docs/design/core/plans/2026-11-05-module-api-stage6-plan.md; spec section 9).
//
// Task 1: the lock module, its rows and its typed calls (this part).
// Task 2: Integrate honours the lock on the CPU. Task 3: the held-quad golden.
// Task 5: the same on Vulkan (GPU cases skip when no device is admitted).
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

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
