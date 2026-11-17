// L87: the device side of tests/test_no_body_contact.cpp. grid_build.slang skips a flagged body as grid.cpp does, so the same
// pass-through and the same collision without the flag happen on the device, and the pair of runs agree with the CPU's.

#include <gtest/gtest.h>

#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"
#include "no_body_contact_case.hpp"

namespace {

using spade::compute::BackendKind;

void fail_on_validation_message(std::string_view message) noexcept {
    ADD_FAILURE() << "Vulkan validation message during a GpuNoBodyContact test: " << message;
}

class GpuNoBodyContact : public ::testing::Test {
  protected:
    void SetUp() override { spade::compute::set_error_sink(&fail_on_validation_message); }
    void TearDown() override { spade::compute::set_error_sink(nullptr); }
};

TEST_F(GpuNoBodyContact, WithoutTheFlagAnInterceptorMeetsTheTargetAndLosesItsSpeed) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const auto r = spade::testing::fly_through(BackendKind::vulkan, false);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_LT(r->interceptor_vel.x, 5.0f);
    EXPECT_GT(r->target_vel.x, 0.005f);
}

TEST_F(GpuNoBodyContact, AFlaggedTargetIsFlownThroughAndNotPushed) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const auto r = spade::testing::fly_through(BackendKind::vulkan, true);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_NEAR(r->interceptor_vel.x, 10.0f, 1.0e-4f);
    EXPECT_GT(r->interceptor_pos.x, 1.9f);
    EXPECT_EQ(r->target_vel.x, 0.0f);
    EXPECT_EQ(r->target_pos.x, 0.0f);
}

TEST_F(GpuNoBodyContact, TheDeviceAndTheCpuAgreeOnBothRuns) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    for (const bool flag : {false, true}) {
        const auto c = spade::testing::fly_through(BackendKind::cpu, flag);
        const auto g = spade::testing::fly_through(BackendKind::vulkan, flag);
        ASSERT_TRUE(c.has_value()) << c.error().context;
        ASSERT_TRUE(g.has_value()) << g.error().context;
        EXPECT_NEAR(c->interceptor_vel.x, g->interceptor_vel.x, 1.0e-2f) << "flag " << flag;
        EXPECT_NEAR(c->target_vel.x, g->target_vel.x, 1.0e-3f) << "flag " << flag;
    }
}

}  // namespace
