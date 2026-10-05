// What the live smoke's scene main functions check, asserted with no window.
//
// The tour runs the built-in scenes (assets/scenes/) and checks that each
// shows what it is there to show: bounce's lanes rebound in the order of their
// restitution, gate's lobs pass through the ring. The measurements behind
// those checks are display-free (sandbox/scene_checks.hpp) and pinned here
// with hand-written trajectories.

#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "../sandbox/scene_checks.hpp"

namespace checks = spade::sandbox::checks;

// ===========================================================================
// bounce: the rebound after first contact
// ===========================================================================

TEST(SandboxSceneChecks, TheReboundIsTheHighestPointAfterFirstContact) {
    // Falls to the ground (contact at y <= 0.4), rebounds to 2.1, falls again.
    const std::vector<float> h = {6.0f, 4.0f, 1.0f, 0.36f, 1.5f, 2.1f, 1.8f, 0.37f, 0.9f};
    const std::optional<float> r = checks::rebound_height(h, 0.4f);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 2.1f);
}

// The start height is before contact, so it never counts as a rebound.
TEST(SandboxSceneChecks, TheDropHeightIsNotARebound) {
    const std::vector<float> h = {6.0f, 3.0f, 0.35f, 0.35f, 0.35f};
    const std::optional<float> r = checks::rebound_height(h, 0.4f);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 0.35f);  // an inelastic lane stays down
}

TEST(SandboxSceneChecks, ABodyThatNeverLandsHasNoRebound) {
    const std::vector<float> h = {6.0f, 5.0f, 4.0f};
    EXPECT_FALSE(checks::rebound_height(h, 0.4f).has_value());
    EXPECT_FALSE(checks::rebound_height({}, 0.4f).has_value());
}

TEST(SandboxSceneChecks, StrictlyIncreasingMeansEveryStepRises) {
    EXPECT_TRUE(checks::strictly_increasing(std::vector<float>{0.35f, 0.9f, 1.6f, 2.8f}));
    EXPECT_FALSE(checks::strictly_increasing(std::vector<float>{0.35f, 0.9f, 0.9f, 2.8f}));
    EXPECT_FALSE(checks::strictly_increasing(std::vector<float>{0.35f, 1.6f, 0.9f}));
    EXPECT_TRUE(checks::strictly_increasing(std::vector<float>{1.0f}));
}

// ===========================================================================
// gate: where a lob crosses the gate's plane, and whether that is in the ring
// ===========================================================================

TEST(SandboxSceneChecks, ACrossingIsInterpolatedAtZeroZ) {
    // z goes -0.5 -> +0.5, so the crossing is halfway: x 0.1 -> 0.3, y 2.0 -> 1.6.
    const auto c = checks::crossing_z0(glm::vec3(0.1f, 2.0f, -0.5f), glm::vec3(0.3f, 1.6f, 0.5f));
    ASSERT_TRUE(c.has_value());
    EXPECT_FLOAT_EQ(c->x, 0.2f);
    EXPECT_FLOAT_EQ(c->y, 1.8f);
}

TEST(SandboxSceneChecks, NoCrossingWithoutChangingSides) {
    EXPECT_FALSE(checks::crossing_z0(glm::vec3(0.0f, 2.0f, -1.0f), glm::vec3(0.0f, 2.0f, -0.2f)).has_value());
    EXPECT_FALSE(checks::crossing_z0(glm::vec3(0.0f, 2.0f, 0.2f), glm::vec3(0.0f, 2.0f, 1.0f)).has_value());
    // Backwards through the plane is not the lob the scene throws.
    EXPECT_FALSE(checks::crossing_z0(glm::vec3(0.0f, 2.0f, 0.5f), glm::vec3(0.0f, 2.0f, -0.5f)).has_value());
}

TEST(SandboxSceneChecks, ThroughTheRingMeansInsideItsClearRadius) {
    EXPECT_TRUE(checks::through_ring({0.0f, 1.8f}, 1.8f, 1.35f));
    EXPECT_TRUE(checks::through_ring({0.3f, 2.9f}, 1.8f, 1.35f));
    EXPECT_FALSE(checks::through_ring({0.0f, 3.2f}, 1.8f, 1.35f));  // over the top
    EXPECT_FALSE(checks::through_ring({1.4f, 1.8f}, 1.8f, 1.35f));  // into a post
}
