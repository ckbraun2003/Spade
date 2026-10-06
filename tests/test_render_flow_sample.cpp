// render/flow_sample.hpp: cell-centred trilinear sampling of a flow volume.
// Exact on a linear field (trilinear reproduces any linear function), held
// constant past the outermost centres, and no-data propagating only through
// corners that carry weight.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <glm/glm.hpp>

#include "flow_synthetic.hpp"
#include "render/flow_sample.hpp"

namespace {

using spade::render::FlowQuantity;
using spade::render::make_flow_sampler;
using spade::render::sample_channel;
using spade::render::sample_quantity;
using spade::render::sample_velocity;

const glm::vec3 kOrigin(1.0f, -2.0f, 0.5f);
constexpr float kCell = 0.5f;

// The world point at continuous cell index g (cell centres at integers).
[[nodiscard]] glm::vec3 at_index(glm::vec3 g) { return kOrigin + (g + glm::vec3(0.5f)) * kCell; }

// 2 gi + 3 gj - gk + 10: integers at every centre, so f16 holds them exactly.
[[nodiscard]] float linear(glm::vec3 g) { return 2.0f * g.x + 3.0f * g.y - g.z + 10.0f; }

[[nodiscard]] spade::FlowVolume linear_volume(glm::vec3 nan_cell = glm::vec3(-1.0f)) {
    return spade::test::make_flow_volume(kOrigin, kCell, 5, 4, 3, [nan_cell](glm::vec3 p) {
        const glm::vec3 g = (p - kOrigin) / kCell - glm::vec3(0.5f);
        spade::test::FlowChannels c{};
        const bool bad = glm::all(glm::lessThan(glm::abs(g - nan_cell), glm::vec3(1e-3f)));
        c[spade::kFlowPressure] = bad ? std::numeric_limits<float>::quiet_NaN() : linear(g);
        return c;
    });
}

TEST(RenderFlowSample, TrilinearIsExactOnALinearField) {
    const spade::FlowVolume volume = linear_volume();
    ASSERT_TRUE(spade::validate_flow_volume(volume).has_value());
    const auto sampler = make_flow_sampler(volume);
    for (const glm::vec3 g : {glm::vec3(0.0f), glm::vec3(1.25f, 0.7f, 1.1f), glm::vec3(3.9f, 2.5f, 0.3f),
                              glm::vec3(4.0f, 3.0f, 2.0f), glm::vec3(2.0f, 1.0f, 1.0f), glm::vec3(0.01f, 2.99f, 1.5f)}) {
        const auto s = sample_channel(sampler, spade::kFlowPressure, at_index(g));
        EXPECT_FALSE(s.no_data);
        EXPECT_NEAR(s.value, linear(g), 1e-5f * 20.0f) << g.x << ", " << g.y << ", " << g.z;
    }
}

// Past the outermost centres the field is held, never extrapolated.
TEST(RenderFlowSample, PastTheOutermostCentresTheFieldIsHeld) {
    const spade::FlowVolume volume = linear_volume();
    const auto sampler = make_flow_sampler(volume);
    const auto value = [&](glm::vec3 g) { return sample_channel(sampler, spade::kFlowPressure, at_index(g)).value; };
    EXPECT_FLOAT_EQ(value({-0.4f, 1.5f, 1.0f}), value({0.0f, 1.5f, 1.0f}));
    EXPECT_FLOAT_EQ(value({4.3f, 1.5f, 1.0f}), value({4.0f, 1.5f, 1.0f}));
    EXPECT_FLOAT_EQ(value({2.0f, -7.0f, 9.0f}), value({2.0f, 0.0f, 2.0f}));
    EXPECT_FLOAT_EQ(value({2.0f, -7.0f, 9.0f}), linear({2.0f, 0.0f, 2.0f}));
}

// L6: a no-data corner that carries weight makes the sample no-data; one
// with zero weight is not read.
TEST(RenderFlowSample, NoDataPropagatesThroughWeightedCornersOnly) {
    const spade::FlowVolume volume = linear_volume(glm::vec3(2.0f, 1.0f, 1.0f));
    ASSERT_EQ(volume.header.no_data, 1u);
    const auto sampler = make_flow_sampler(volume);
    const auto at = [&](glm::vec3 g) { return sample_channel(sampler, spade::kFlowPressure, at_index(g)); };
    EXPECT_TRUE(at({1.5f, 1.0f, 1.0f}).no_data);
    EXPECT_TRUE(at({2.0f, 1.0f, 1.0f}).no_data);
    EXPECT_TRUE(at({2.9f, 1.9f, 0.1f}).no_data);
    EXPECT_EQ(at({2.9f, 1.9f, 0.1f}).value, 0.0f);
    const auto beside = at({1.0f, 1.0f, 1.0f});
    EXPECT_FALSE(beside.no_data) << "the no-data neighbour has zero weight at this centre";
    EXPECT_FLOAT_EQ(beside.value, linear({1.0f, 1.0f, 1.0f}));
    EXPECT_FALSE(at({0.5f, 0.5f, 0.5f}).no_data);
}

TEST(RenderFlowSample, SpeedIsTheNormOfTheInterpolatedVelocity) {
    const spade::FlowVolume volume =
        spade::test::make_flow_volume(kOrigin, kCell, 3, 3, 3, spade::test::uniform_flow({3.0f, 4.0f, 0.0f}));
    const auto sampler = make_flow_sampler(volume);
    const glm::vec3 p = at_index({1.3f, 0.2f, 1.9f});
    EXPECT_FLOAT_EQ(sample_quantity(sampler, FlowQuantity::speed, p).value, 5.0f);
    EXPECT_FLOAT_EQ(sample_quantity(sampler, FlowQuantity::u, p).value, 3.0f);
    EXPECT_FLOAT_EQ(sample_quantity(sampler, FlowQuantity::v, p).value, 4.0f);
    const auto vel = sample_velocity(sampler, p);
    EXPECT_FALSE(vel.no_data);
    EXPECT_EQ(vel.u, glm::vec3(3.0f, 4.0f, 0.0f));
}

// Opposite velocities either side of a face: speed of the interpolated
// velocity is 0 there, where an interpolated speed would read |u|.
TEST(RenderFlowSample, SpeedDoesNotInterpolateMagnitudes) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(kOrigin, kCell, 2, 1, 1, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowU] = p.x < kOrigin.x + kCell ? 2.0f : -2.0f;
        return c;
    });
    const auto sampler = make_flow_sampler(volume);
    EXPECT_FLOAT_EQ(sample_quantity(sampler, FlowQuantity::speed, at_index({0.5f, 0.0f, 0.0f})).value, 0.0f);
    EXPECT_FLOAT_EQ(sample_quantity(sampler, FlowQuantity::speed, at_index({0.0f, 0.0f, 0.0f})).value, 2.0f);
}

}  // namespace
