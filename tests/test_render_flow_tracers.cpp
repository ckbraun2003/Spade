// render/flow_lines.hpp's tracers: RK4 in time through the published field,
// emitted on a fixed schedule, retired by reason, capacity fixed. The
// volumes hold fields f16 stores exactly (dyadic values at the centres),
// and the times are binary fractions, so the schedule is exact too.
// Test-only double and std::cos/sin; this file feeds no golden.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "flow_synthetic.hpp"
#include "render/flow_lines.hpp"

namespace {

using spade::render::advance_tracers;
using spade::render::TracerOptions;
using spade::render::TracerSource;
using spade::render::TracerState;

[[nodiscard]] spade::FlowVolume uniform_box(glm::vec3 u) {
    return spade::test::make_flow_volume(glm::vec3(-4.0f), 1.0f, 8, 8, 8, spade::test::uniform_flow(u));
}

void run(TracerState& state, const spade::FlowVolume& volume, const std::vector<TracerSource>& sources, float dt,
         int steps, const TracerOptions& options = {}) {
    for (int i = 0; i < steps; ++i) {
        const auto r = advance_tracers(state, volume, sources, dt, options);
        ASSERT_TRUE(r.has_value()) << r.error().context;
    }
}

// Uniform flow carries a tracer emitted at te to source + u (T - te).
TEST(RenderFlowTracers, UniformFlowCarriesEachTracerToItsExactPlace) {
    const glm::vec3 u(0.75f, 0.5f, 0.0f);
    const spade::FlowVolume volume = uniform_box(u);
    const std::vector<TracerSource> sources{{glm::vec3(-3.0f, 0.0f, 0.0f), 0.25f}};
    TracerState state;
    run(state, volume, sources, 0.125f, 4);  // T = 0.5: emissions at 0 and 0.25
    ASSERT_EQ(state.positions.size(), 2u);
    EXPECT_EQ(state.clock_s, 0.5);
    const float emitted[2] = {0.0f, 0.25f};
    for (size_t i = 0; i < 2; ++i) {
        const float age = 0.5f - emitted[i];
        EXPECT_LT(glm::length(state.positions[i] - (sources[0].position + u * age)), 1e-5f) << "tracer " << i;
        EXPECT_FLOAT_EQ(state.ages[i], age);
        EXPECT_EQ(state.sources[i], 0u);
    }
}

// The schedule is k * interval, however the run is cut into steps.
TEST(RenderFlowTracers, TheScheduleDoesNotDependOnTheSteps) {
    const spade::FlowVolume volume = uniform_box(glm::vec3(0.5f, 0.0f, 0.25f));
    const std::vector<TracerSource> sources{{glm::vec3(-3.0f, 0.0f, 0.0f), 0.125f},
                                            {glm::vec3(0.0f, -2.0f, 0.0f), 0.25f}};
    TracerState fine, coarse;
    run(fine, volume, sources, 0.0625f, 8);
    run(coarse, volume, sources, 0.5f, 1);
    ASSERT_EQ(fine.positions.size(), 6u) << "4 from the first source, 2 from the second";
    ASSERT_EQ(coarse.positions.size(), fine.positions.size());
    for (size_t i = 0; i < fine.positions.size(); ++i) {
        EXPECT_LT(glm::length(fine.positions[i] - coarse.positions[i]), 1e-5f) << "tracer " << i;
        EXPECT_FLOAT_EQ(fine.ages[i], coarse.ages[i]);
    }
}

// Solid-body rotation about z, Omega = 1: a tracer from (2, 0) is at
// 2 (cos T, sin T) after T. RK4 in time at dt = 1/32.
TEST(RenderFlowTracers, SolidBodyRotationCarriesATracerRoundItsCircle) {
    const spade::FlowVolume volume =
        spade::test::make_flow_volume(glm::vec3(-4.0f), 1.0f, 8, 8, 8, [](glm::vec3 p) {
            spade::test::FlowChannels c{};
            c[spade::kFlowU] = -p.y;
            c[spade::kFlowV] = p.x;
            return c;
        });
    const std::vector<TracerSource> sources{{glm::vec3(2.0f, 0.0f, 0.5f), 1000.0f}};  // one tracer, at t = 0
    TracerState state;
    run(state, volume, sources, 0.03125f, 50);
    ASSERT_EQ(state.positions.size(), 1u);
    const double t = 50 * 0.03125;
    const glm::vec3 want(static_cast<float>(2.0 * std::cos(t)), static_cast<float>(2.0 * std::sin(t)), 0.5f);
    EXPECT_LT(glm::length(state.positions[0] - want), 1e-5f);
}

TEST(RenderFlowTracers, CapacityDropsAndCountsNeverGrows) {
    const spade::FlowVolume volume = uniform_box(glm::vec3(0.25f, 0.0f, 0.0f));
    const std::vector<TracerSource> sources{{glm::vec3(-3.0f, 0.0f, 0.0f), 0.0625f}};
    TracerOptions options;
    options.capacity = 3;
    TracerState state;
    run(state, volume, sources, 0.5f, 1, options);  // 8 emissions
    EXPECT_EQ(state.positions.size(), 3u);
    EXPECT_EQ(state.dropped, 5u);
}

// L6: each way a tracer retires is counted, and no retired one stays.
TEST(RenderFlowTracers, RetiredTracersAreCountedByReason) {
    {
        const spade::FlowVolume volume = uniform_box(glm::vec3(2.0f, 0.0f, 0.0f));
        const std::vector<TracerSource> sources{{glm::vec3(2.0f, 0.0f, 0.0f), 0.25f}};
        TracerState state;
        run(state, volume, sources, 0.25f, 12);
        EXPECT_GT(state.retired_left_box, 0u);
        for (const glm::vec3& p : state.positions) EXPECT_LE(p.x, 4.0f);
    }
    {
        const spade::FlowVolume volume =
            spade::test::make_flow_volume(glm::vec3(-4.0f), 1.0f, 8, 8, 8, [](glm::vec3 p) {
                spade::test::FlowChannels c{};
                c[spade::kFlowU] = p.x > 1.0f ? std::numeric_limits<float>::quiet_NaN() : 1.0f;
                return c;
            });
        const std::vector<TracerSource> sources{{glm::vec3(-2.0f, 0.0f, 0.0f), 0.25f}};
        TracerState state;
        run(state, volume, sources, 0.25f, 16);
        EXPECT_GT(state.retired_no_data, 0u);
        for (const glm::vec3& p : state.positions) EXPECT_LT(p.x, 1.0f);
    }
    {
        const spade::FlowVolume volume = uniform_box(glm::vec3(0.0625f, 0.0f, 0.0f));
        const std::vector<TracerSource> sources{{glm::vec3(0.0f), 0.125f}};
        TracerOptions options;
        options.max_age_s = 0.375f;
        TracerState state;
        run(state, volume, sources, 0.125f, 8, options);
        EXPECT_GT(state.retired_aged, 0u);
        for (const float age : state.ages) EXPECT_LE(age, 0.375f);
    }
}

TEST(RenderFlowTracers, BadInputIsRefusedBeforeAnyChange) {
    const spade::FlowVolume volume = uniform_box(glm::vec3(1.0f, 0.0f, 0.0f));
    const std::vector<TracerSource> good{{glm::vec3(0.0f), 0.25f}};
    const auto refused = [&](TracerState state, const std::vector<TracerSource>& sources, float dt,
                             const TracerOptions& options, const char* what) {
        const TracerState before = state;
        const auto r = advance_tracers(state, volume, sources, dt, options);
        ASSERT_FALSE(r.has_value()) << what << " must be refused";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << what;
        EXPECT_EQ(state.positions, before.positions) << what;
        EXPECT_EQ(state.clock_s, before.clock_s) << what;
    };
    refused({}, good, 0.0f, {}, "a zero dt");
    refused({}, good, std::numeric_limits<float>::quiet_NaN(), {}, "a NaN dt");
    TracerOptions o;
    o.capacity = 0;
    refused({}, good, 0.1f, o, "zero capacity");
    o = {};
    o.max_age_s = 0.0f;
    refused({}, good, 0.1f, o, "a zero max age");
    refused({}, {{glm::vec3(0.0f), 0.0f}}, 0.1f, {}, "a zero interval");
    refused({}, {{glm::vec3(std::numeric_limits<float>::infinity()), 0.25f}}, 0.1f, {}, "an infinite source");
    refused({}, {{glm::vec3(0.0f), 1e-9f}}, 1.0f, {}, "a source emitting past 2^20 in one step");
    TracerState torn;
    torn.positions = {glm::vec3(0.0f)};
    refused(torn, good, 0.1f, {}, "a state whose arrays disagree");
    TracerState full;
    full.positions.assign(3, glm::vec3(0.0f));
    full.ages.assign(3, 0.0f);
    full.sources.assign(3, 0u);
    o = {};
    o.capacity = 2;
    refused(full, good, 0.1f, o, "a state past capacity");
}

}  // namespace
