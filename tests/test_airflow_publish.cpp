// The airflow solver core's render-volume publish (airflow core plan,
// Task 8b; the lead, 2026-10-12): spade::FlowVolume filled from the MAC grid
// at cell centres, with |omega| and q-hat from staggered derivatives, the f16
// no-data rule, and the first layer of solid cells extrapolated from the
// fluid side so a surface reads its wall pressure (the demonstration plan, P6).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
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
                EXPECT_EQ(channel(vol, i, j, k, spade::kFlowLambda2Hat), 0.0f);
            }
        }
    }
}

// Solid-body rotation u = -W (y - c), v = W (x - c): omega_z = 2W everywhere,
// no strain, so Q = |omega|^2 / 4 = W^2 and q-hat = W; S^2 + Omega^2 =
// diag(-W^2, -W^2, 0), so lambda2 = -W^2 and its hat -W (a vortex core). The
// fields are linear, so the differences are exact at the edges too.
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
            EXPECT_NEAR(channel(vol, i, j, 1, spade::kFlowLambda2Hat), -W, 2.0e-3f * W) << i << "," << j;
        }
    }
}

// Pure strain u = a x, v = -a y: no vorticity, Q = -a^2, q-hat = -a; S^2 =
// diag(a^2, a^2, 0), so lambda2 = a^2 and its hat a (no vortex).
TEST(AirflowPublish, APureStrainHasNoVorticityAndANegativeQHat) {
    const float a = 2.0f;
    MacGrid g = *make_grid({4, 4, 2, 0.5f}, DomainBc{});
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = a * x.x; });
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) { g.v[idx] = -a * x.y; });
    FlowVolume vol;
    ASSERT_TRUE(publish_render_volume(g, {}, vol).has_value());
    EXPECT_EQ(channel(vol, 2, 2, 1, spade::kFlowVorticity), 0.0f);
    EXPECT_NEAR(channel(vol, 2, 2, 1, spade::kFlowQHat), -a, 2.0e-3f * a);
    EXPECT_NEAR(channel(vol, 2, 2, 1, spade::kFlowLambda2Hat), a, 2.0e-3f * a);
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
            EXPECT_EQ(spade::flow_half(vol, spade::flow_cell_index(vol.header, 4, j, k), spade::kFlowLambda2Hat),
                      spade::fp16::kNoDataBits)
                << "lambda2 is data now: a deeper solid cell has none";
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


// Rendering's finding (2026-10-12): on a periodic axis the edge cells'
// differences must wrap, not go one-sided. u = sin(k y) in a box periodic in
// y: omega_z = -du/dy, and the central difference at every cell -- the edges
// included -- is k cos(k y) sin(k dx) / (k dx).
TEST(AirflowPublish, APeriodicAxisWrapsItsEdgeDifferences) {
    const uint32_t n = 16;
    const double L = 2.0;
    const double k = 2.0 * std::numbers::pi / L;
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid({4, n, 2, static_cast<float>(L / n)}, bc);
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) {
        g.u[idx] = static_cast<float>(std::sin(k * static_cast<double>(x.y)));
    });
    FlowVolume vol;
    ASSERT_TRUE(publish_render_volume(g, {.periodic = {true, true, true}}, vol).has_value());
    const double dx = L / n;
    const double factor = std::sin(k * dx) / (k * dx);
    for (uint32_t j : {0u, 1u, n / 2u, n - 1u}) {
        const double y = (j + 0.5) * dx;
        const double expected = std::fabs(k * std::cos(k * y) * factor);
        EXPECT_NEAR(channel(vol, 1, j, 1, spade::kFlowVorticity), expected, 2.0e-3 * k + 1.0e-3 * expected) << j;
    }
}


// The final review's M3 (2026-10-12): on a periodic axis the solid layer's
// fluid neighbour across the wrap counts. A wall of solid cells at i = 0 in a
// box periodic in x: its only fluid face-neighbours are at i = 1 and, across
// the wrap, i = n - 1. Pressure 1 at i = 1 and 3 at i = n - 1: the wall reads
// their mean, 2, not 1.
TEST(AirflowPublish, APeriodicAxisWrapsTheSolidLayersFluidNeighbours) {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid({6, 2, 2, 0.5f}, bc);
    std::vector<uint8_t> solid(g.p.size(), 0);
    for (uint32_t k = 0; k < 2; ++k) {
        for (uint32_t j = 0; j < 2; ++j) {
            solid[cell_index(g.shape, 0, j, k)] = 1;
            for (uint32_t i = 0; i < 6; ++i) g.p[cell_index(g.shape, i, j, k)] = i == 5 ? 3.0f : 1.0f;
        }
    }
    FlowVolume vol;
    const auto counts = publish_render_volume(g, {.solid = solid, .periodic = {true, true, true}}, vol);
    ASSERT_TRUE(counts.has_value());
    EXPECT_EQ(counts->extrapolated, 4u);
    for (uint32_t k = 0; k < 2; ++k) {
        for (uint32_t j = 0; j < 2; ++j) EXPECT_EQ(channel(vol, 0, j, k, spade::kFlowPressure), 2.0f) << j << "," << k;
    }
}


// A strained vortex, where lambda2 and -Q differ (Rendering, 2026-10-13: the
// cases above have lambda2 = -Q, so they could not tell this channel from
// -q-hat). Rotation W about z plus axisymmetric strain alpha:
// u = -W y - (alpha/2) x, v = W x - (alpha/2) y, w = alpha z. Linear, so the
// differences are exact. S^2 + Omega^2 = diag(alpha^2/4 - W^2 (twice),
// alpha^2): lambda2 = alpha^2/4 - W^2 = -8 (hat -sqrt 8), while
// Q = W^2 - 3 alpha^2/4 = 6 (so -q-hat = -sqrt 6).
TEST(AirflowPublish, AStrainedVortexsLambda2IsNotMinusQ) {
    const float W = 3.0f;
    const float alpha = 2.0f;
    MacGrid g = *make_grid({6, 6, 6, 0.5f}, DomainBc{});
    const float c = 1.5f;
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = -W * (x.y - c) - 0.5f * alpha * (x.x - c); });
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) { g.v[idx] = W * (x.x - c) - 0.5f * alpha * (x.y - c); });
    for_each_sample(g.shape, 2, [&](std::size_t idx, glm::vec3 x) { g.w[idx] = alpha * (x.z - c); });
    FlowVolume vol;
    ASSERT_TRUE(publish_render_volume(g, {}, vol).has_value());
    const float lambda2_hat = -std::sqrt(8.0f);
    const float q_hat = std::sqrt(6.0f);
    for (uint32_t k = 0; k < 6; ++k) {
        for (uint32_t j = 0; j < 6; ++j) {
            for (uint32_t i = 0; i < 6; ++i) {
                EXPECT_NEAR(channel(vol, i, j, k, spade::kFlowLambda2Hat), lambda2_hat, 2.0e-3f * 3.0f)
                    << i << "," << j << "," << k;
                EXPECT_NEAR(channel(vol, i, j, k, spade::kFlowQHat), q_hat, 2.0e-3f * 3.0f) << i << "," << j << "," << k;
            }
        }
    }
}

// Each publish names its content anew (core/flow_volume.hpp), so a reader's
// cache never draws an older frame's halves; a refused publish leaves 0, as
// it may have written part of the volume.
TEST(AirflowPublish, EachPublishNamesItsContentAnew) {
    MacGrid g = *make_grid({4, 4, 4, 0.25f}, DomainBc{});
    FlowVolume vol;
    ASSERT_TRUE(publish_render_volume(g, {}, vol).has_value());
    const uint64_t first = vol.generation;
    EXPECT_NE(first, 0u);
    for (float& x : g.u) x = 1.0f;
    ASSERT_TRUE(publish_render_volume(g, {}, vol).has_value());
    EXPECT_NE(vol.generation, 0u);
    EXPECT_NE(vol.generation, first);
    const std::vector<uint8_t> solid(7, 0);
    ASSERT_FALSE(publish_render_volume(g, {.solid = solid}, vol).has_value());
    EXPECT_EQ(vol.generation, 0u);
}

}  // namespace
