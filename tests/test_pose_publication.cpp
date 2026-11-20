// test_pose_publication.cpp -- Simulation::published_poses() and keep_previous_poses() (Phase 3.2 of rendering/camera-sensor-design,
// section 11 item 5): the host-side, read-only view of the body poses at a tick boundary, with the previous tick's poses for
// camera-motion blur. The contract is sim/simulation.hpp's PublishedPoses, which is compute/vulkan/pose_view.hpp's.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include <glm/vec3.hpp>

#include "set_body_state_support.hpp"
#include "sim/simulation.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "testing/replay.hpp"

namespace {

using spade::BodyRef;
using spade::BodyState;
using spade::PublishedPoses;
using spade::Result;
using spade::Simulation;
using spade::test::bare_body;

constexpr uint64_t kDtNs = 1'000'000;
constexpr uint32_t kSubsteps = 2;

// One floor world with a falling body, so every tick moves it.
struct Rig {
    Simulation sim;
    BodyRef body;
};

[[nodiscard]] Rig make_rig() {
    const spade::Result<spade::WorldSetDesc> set = spade::test::floor_world_set(4);
    EXPECT_TRUE(set.has_value()) << set.error().context;
    Result<Simulation> sim = Simulation::create(*set, kDtNs, kSubsteps);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    const Result<BodyRef> body = sim->spawn(0, bare_body(glm::vec3(0.0f, 3.0f, 0.0f)));
    EXPECT_TRUE(body.has_value()) << body.error().context;
    EXPECT_TRUE(sim->flush_structural().has_value());
    return Rig{std::move(*sim), *body};
}

// The whole bodies array as the arena holds it, copied.
[[nodiscard]] std::vector<BodyState> bodies_copy(const Simulation& sim) {
    const Result<std::span<const BodyState>> all = sim.arenas().array(sim.bodies_array());
    EXPECT_TRUE(all.has_value());
    return std::vector<BodyState>(all->begin(), all->end());
}

[[nodiscard]] bool same_bytes(std::span<const BodyState> a, std::span<const BodyState> b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size_bytes()) == 0;
}

}  // namespace

TEST(PosePublication, AFreshWorldPublishesItsPosesAsNotYetStepped) {
    Rig rig = make_rig();
    const Result<PublishedPoses> view = rig.sim.published_poses();
    ASSERT_TRUE(view.has_value()) << view.error().context;
    EXPECT_FALSE(view->stepped);
    EXPECT_EQ(view->world_count, 1u);
    EXPECT_EQ(view->current.size(), static_cast<std::size_t>(view->world_count) * view->capacity_per_world);
    EXPECT_EQ(view->slot_to_world.size(), view->current.size());
    EXPECT_TRUE(view->previous.empty());
    EXPECT_TRUE(same_bytes(view->current, bodies_copy(rig.sim)));
}

TEST(PosePublication, PreviousIsEmptyUnlessKeptAndNeverAllocatedByDefault) {
    Rig rig = make_rig();
    ASSERT_TRUE(rig.sim.step(3).has_value());
    const Result<PublishedPoses> view = rig.sim.published_poses();
    ASSERT_TRUE(view.has_value()) << view.error().context;
    EXPECT_TRUE(view->stepped);
    EXPECT_EQ(view->tick, 2u) << "the last tick run, 0-based";
    EXPECT_TRUE(view->previous.empty()) << "a world that never asks for the history pays nothing for it";
    EXPECT_TRUE(same_bytes(view->current, bodies_copy(rig.sim)));
}

TEST(PosePublication, PreviousIsTheStateTheLastTickStartedFrom) {
    Rig rig = make_rig();
    ASSERT_TRUE(rig.sim.keep_previous_poses(true).has_value());
    const std::vector<BodyState> upload = bodies_copy(rig.sim);
    ASSERT_TRUE(rig.sim.step(1).has_value());
    {
        const Result<PublishedPoses> view = rig.sim.published_poses();
        ASSERT_TRUE(view.has_value()) << view.error().context;
        EXPECT_EQ(view->tick, 0u);
        EXPECT_TRUE(same_bytes(view->previous, upload)) << "for the first tick: the state the world was created with";
        EXPECT_TRUE(same_bytes(view->current, bodies_copy(rig.sim)));
        EXPECT_FALSE(same_bytes(view->previous, view->current)) << "the falling body moved";
    }
    const std::vector<BodyState> after_one = bodies_copy(rig.sim);
    ASSERT_TRUE(rig.sim.step(1).has_value());
    const Result<PublishedPoses> view = rig.sim.published_poses();
    ASSERT_TRUE(view.has_value()) << view.error().context;
    EXPECT_EQ(view->tick, 1u);
    EXPECT_TRUE(same_bytes(view->previous, after_one));
}

TEST(PosePublication, AStepOfSeveralTicksKeepsOneTickOfHistory) {
    Rig stepped_by_one = make_rig();
    Rig stepped_by_four = make_rig();
    ASSERT_TRUE(stepped_by_four.sim.keep_previous_poses(true).has_value());
    ASSERT_TRUE(stepped_by_one.sim.step(3).has_value());
    const std::vector<BodyState> end_of_tick_2 = bodies_copy(stepped_by_one.sim);
    ASSERT_TRUE(stepped_by_four.sim.step(4).has_value());
    const Result<PublishedPoses> view = stepped_by_four.sim.published_poses();
    ASSERT_TRUE(view.has_value()) << view.error().context;
    EXPECT_EQ(view->tick, 3u);
    EXPECT_TRUE(same_bytes(view->previous, end_of_tick_2)) << "the end of tick 2, not of tick -1";
}

TEST(PosePublication, KeepingThePreviousPosesChangesNoResult) {
    Rig plain = make_rig();
    Rig kept = make_rig();
    ASSERT_TRUE(kept.sim.keep_previous_poses(true).has_value());
    for (int i = 0; i < 12; ++i) {
        ASSERT_TRUE(plain.sim.step(1).has_value());
        ASSERT_TRUE(kept.sim.step(1).has_value());
        const Result<PublishedPoses> view = kept.sim.published_poses();  // reading it every tick is part of the claim
        ASSERT_TRUE(view.has_value());
    }
    EXPECT_EQ(spade::testing::state_digest(plain.sim), spade::testing::state_digest(kept.sim));
    const Result<spade::SnapshotBlob> a = plain.sim.snapshot();
    const Result<spade::SnapshotBlob> b = kept.sim.snapshot();
    ASSERT_TRUE(a.has_value() && b.has_value());
    ASSERT_EQ(a->size(), b->size());
    EXPECT_EQ(std::memcmp(a->bytes().data(), b->bytes().data(), a->size()), 0) << "the history is not in the snapshot";
}

TEST(PosePublication, ARestoreForgetsTheHistoryUntilTheNextTick) {
    Rig rig = make_rig();
    ASSERT_TRUE(rig.sim.keep_previous_poses(true).has_value());
    ASSERT_TRUE(rig.sim.step(2).has_value());
    const Result<spade::SnapshotBlob> blob = rig.sim.snapshot();
    ASSERT_TRUE(blob.has_value());
    ASSERT_TRUE(rig.sim.step(2).has_value());
    ASSERT_TRUE(rig.sim.restore(*blob).has_value());
    const Result<PublishedPoses> after_restore = rig.sim.published_poses();
    ASSERT_TRUE(after_restore.has_value()) << after_restore.error().context;
    EXPECT_FALSE(after_restore->stepped);
    EXPECT_TRUE(after_restore->previous.empty()) << "the pre-restore history belongs to a timeline that no longer exists";
    const std::vector<BodyState> restored = bodies_copy(rig.sim);
    ASSERT_TRUE(rig.sim.step(1).has_value());
    const Result<PublishedPoses> view = rig.sim.published_poses();
    ASSERT_TRUE(view.has_value());
    EXPECT_TRUE(view->stepped);
    EXPECT_TRUE(same_bytes(view->previous, restored)) << "the restored state is what the next tick started from";
}

TEST(PosePublication, TurningItOffDropsTheHistory) {
    Rig rig = make_rig();
    ASSERT_TRUE(rig.sim.keep_previous_poses(true).has_value());
    ASSERT_TRUE(rig.sim.step(2).has_value());
    ASSERT_TRUE(rig.sim.keep_previous_poses(false).has_value());
    const Result<PublishedPoses> view = rig.sim.published_poses();
    ASSERT_TRUE(view.has_value());
    EXPECT_TRUE(view->previous.empty());
}
