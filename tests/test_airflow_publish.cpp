// The airflow solver core's render-volume publish (airflow core plan,
// Task 8b; the lead, 2026-10-12): spade::FlowVolume filled from the MAC grid
// at cell centres, with |omega| and q-hat from staggered derivatives, the f16
// no-data rule, and the first layer of solid cells extrapolated from the
// fluid side so a surface reads its wall pressure (the demonstration plan, P6).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "physics/airflow/publish.hpp"

namespace {

using namespace spade::physics::airflow;
using spade::FlowVolume;

[[nodiscard]] float channel(const FlowVolume& vol, uint32_t i, uint32_t j, uint32_t k, spade::FlowChannel c) {
    return spade::fp16::to_f32(spade::flow_half(vol, spade::flow_cell_index(vol.header, i, j, k), c));
}

TEST(AirflowPublish, AUniformFlowPublishesItsVelocityAndNoVorticity) {
    MacGrid g = *make_grid({4, 4, 4, 0.25f}, DomainBc{});
    for (float& x : g.u) x = 1.0f;
    for (float& x : g.v) x = 2.0f;
    for (float& x : g.w) x = -0.5f;
    for (float& x : g.p) x = 3.0f;
    FlowVolume vol;
    const auto counts = publish_render_volume(g, {.origin = glm::vec3(1.0f, 2.0f, 3.0f)}, vol);
    ASSERT_TRUE(counts.has_value()) << counts.error().context;
    ASSERT_TRUE(spade::validate_flow_volume(vol).has_value());
    EXPECT_EQ(vol.header.nx, 4u);
    EXPECT_EQ(vol.header.cell, 0.25f);
    EXPECT_EQ(vol.header.origin, glm::vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(vol.header.no_data, 0u);
    EXPECT_EQ(counts->extrapolated, 0u);
    EXPECT_EQ(vol.header.extrapolated_cells, 0u);
    for (uint32_t k = 0; k < 4; ++k) {
        for (uint32_t j = 0; j < 4; ++j) {
            for (uint32_t i = 0; i < 4; ++i) {
                EXPECT_EQ(channel(vol, i, j, k, spade::kFlowU), 1.0f);
                EXPECT_EQ(channel(vol, i, j, k, spade::kFlowV), 2.0f);
                EXPECT_EQ(channel(vol, i, j, k, spade::kFlowW), -0.5f);
                EXPECT_EQ(channel(vol, i, j, k, spade::kFlowPressure), 3.0f);
                EXPECT_EQ(channel(vol, i, j, k, spade::kFlowVorticity), 0.0f);
                EXPECT_EQ(channel(vol, i, j, k, spade::kFlowQHat), 0.0f);
                EXPECT_EQ(channel(vol, i, j, k, spade::kFlowReserved), 0.0f);
            }
        }
    }
}

// Solid-body rotation u = -W (y - c), v = W (x - c): omega_z = 2W everywhere,
// no strain, so Q = |omega|^2 / 4 = W^2 and q-hat = W. The fields are linear,
// so the differences are exact at the edges too.
TEST(AirflowPublish, ASolidBodyRotationHasVorticityTwiceItsRateAndQHatItsRate) {
    const float W = 3.0f;
    MacGrid g = *make_grid({6, 6, 2, 0.5f}, DomainBc{});
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = -W * (x.y - 1.5f); });
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) { g.v[idx] = W * (x.x - 1.5f); });
    FlowVolume vol;
    ASSERT_TRUE(publish_render_volume(g, {}, vol).has_value());
    for (uint32_t j = 0; j < 6; ++j) {
        for (uint32_t i = 0; i < 6; ++i) {
            EXPECT_NEAR(channel(vol, i, j, 1, spade::kFlowVorticity), 2.0f * W, 2.0e-3f * W) << i << "," << j;
            EXPECT_NEAR(channel(vol, i, j, 1, spade::kFlowQHat), W, 2.0e-3f * W) << i << "," << j;
        }
    }
}

// Pure strain u = a x, v = -a y: no vorticity, Q = -a^2, q-hat = -a.
TEST(AirflowPublish, APureStrainHasNoVorticityAndANegativeQHat) {
    const float a = 2.0f;
    MacGrid g = *make_grid({4, 4, 2, 0.5f}, DomainBc{});
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = a * x.x; });
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) { g.v[idx] = -a * x.y; });
    FlowVolume vol;
    ASSERT_TRUE(publish_render_volume(g, {}, vol).has_value());
    EXPECT_EQ(channel(vol, 2, 2, 1, spade::kFlowVorticity), 0.0f);
    EXPECT_NEAR(channel(vol, 2, 2, 1, spade::kFlowQHat), -a, 2.0e-3f * a);
}

// A non-finite sample, and a finite one past f16's range (|x| >= 65520), are
// no-data: counted in the header, left out of the channel's range, never
// saturated (L6; the spec's §5).
TEST(AirflowPublish, ANonFiniteOrOutOfRangeSampleIsNoDataCountedAndOutOfTheRange) {
    MacGrid g = *make_grid({4, 4, 4, 0.25f}, DomainBc{});
    for (float& x : g.p) x = 10.0f;
    g.p[cell_index(g.shape, 1, 1, 1)] = std::numeric_limits<float>::quiet_NaN();
    g.p[cell_index(g.shape, 2, 2, 2)] = 70000.0f;
    FlowVolume vol;
    ASSERT_TRUE(publish_render_volume(g, {}, vol).has_value());
    ASSERT_TRUE(spade::validate_flow_volume(vol).has_value());
    EXPECT_EQ(vol.header.no_data, 2u);
    EXPECT_EQ(spade::flow_half(vol, spade::flow_cell_index(vol.header, 2, 2, 2), spade::kFlowPressure),
              spade::fp16::kNoDataBits);
    EXPECT_EQ(vol.header.channel_min[spade::kFlowPressure], 10.0f);
    EXPECT_EQ(vol.header.channel_max[spade::kFlowPressure], 10.0f);
}

// The demonstration plan's P6 (Rendering's ask): the first layer of solid
// cells -- any solid cell with a fluid face-neighbour -- takes the mean of its
// fluid face-neighbours, so C_p sampled at a surface reads the wall's
// pressure; deeper solid cells are no-data.
TEST(AirflowPublish, TheFirstSolidLayerTakesTheFluidSideAndDeeperCellsAreNoData) {
    MacGrid g = *make_grid({5, 3, 3, 0.5f}, DomainBc{});
    for (uint32_t k = 0; k < 3; ++k) {
        for (uint32_t j = 0; j < 3; ++j) {
            for (uint32_t i = 0; i < 5; ++i) g.p[cell_index(g.shape, i, j, k)] = static_cast<float>(i + 1);
        }
    }
    std::vector<uint8_t> solid(g.p.size(), 0);
    for (uint32_t k = 0; k < 3; ++k) {
        for (uint32_t j = 0; j < 3; ++j) {
            solid[cell_index(g.shape, 3, j, k)] = 1;  // the first solid layer
            solid[cell_index(g.shape, 4, j, k)] = 1;  // deeper
        }
    }
    FlowVolume vol;
    const auto counts = publish_render_volume(g, {.solid = solid}, vol);
    ASSERT_TRUE(counts.has_value()) << counts.error().context;
    ASSERT_TRUE(spade::validate_flow_volume(vol).has_value());
    EXPECT_EQ(counts->extrapolated, 9u);
    EXPECT_EQ(vol.header.extrapolated_cells, 9u) << "the header carries the count (Core's field)";
    for (uint32_t k = 0; k < 3; ++k) {
        for (uint32_t j = 0; j < 3; ++j) {
            EXPECT_EQ(channel(vol, 3, j, k, spade::kFlowPressure), 3.0f) << "the fluid side's pressure, cell i = 2";
            EXPECT_EQ(spade::flow_half(vol, spade::flow_cell_index(vol.header, 4, j, k), spade::kFlowPressure),
                      spade::fp16::kNoDataBits);
        }
    }
    EXPECT_EQ(vol.header.channel_max[spade::kFlowPressure], 3.0f) << "the deeper cells' pressure (5) is not data";
}

TEST(AirflowPublish, AMismatchedSolidMaskIsRefused) {
    const MacGrid g = *make_grid({4, 4, 4, 0.25f}, DomainBc{});
    const std::vector<uint8_t> solid(7, 0);
    FlowVolume vol;
    const auto counts = publish_render_volume(g, {.solid = solid}, vol);
    ASSERT_FALSE(counts.has_value());
    EXPECT_EQ(counts.error().code, spade::Code::invalid_argument);
}

}  // namespace
