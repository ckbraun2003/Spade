// L87 (KAT's interception work): a body flagged BodySpawn::no_body_contact takes no part in dynamic (body-body) contact. The
// CPU side: the grid does not list it, so no pair ever has it as either member. Static contact with the world is untouched.
// The device twin is tests/test_gpu_no_body_contact.cpp.

#include <gtest/gtest.h>

#include "no_body_contact_case.hpp"

namespace {

using spade::compute::BackendKind;

TEST(NoBodyContact, WithoutTheFlagAnInterceptorMeetsTheTargetAndLosesItsSpeed) {
    const auto r = spade::testing::fly_through(BackendKind::cpu, false);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_LT(r->interceptor_vel.x, 5.0f) << "the unflagged pair collides: the interceptor gives up most of its 10 m/s";
    EXPECT_GT(r->target_vel.x, 0.005f) << "and the heavy target is pushed";
}

TEST(NoBodyContact, AFlaggedTargetIsFlownThroughAndNotPushed) {
    const auto r = spade::testing::fly_through(BackendKind::cpu, true);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_NEAR(r->interceptor_vel.x, 10.0f, 1.0e-4f) << "the interceptor keeps its speed";
    EXPECT_GT(r->interceptor_pos.x, 1.9f) << "and is 2 m past the target after 0.4 s";
    EXPECT_EQ(r->target_vel.x, 0.0f) << "the target takes no push";
    EXPECT_EQ(r->target_pos.x, 0.0f);
}

}  // namespace
