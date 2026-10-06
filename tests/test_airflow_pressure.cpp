// The airflow solver core's pressure operator, multigrid and projection
// (airflow core plan, Tasks 3-5).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/pressure.hpp"

namespace {

using namespace spade::physics::airflow;

// A fixed pseudo-random sequence in [-1, 1), so every run sees the same field.
struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] std::array<FaceKind, 6> all(FaceKind k) {
    std::array<FaceKind, 6> out{};
    out.fill(k);
    return out;
}

[[nodiscard]] float max_abs(const std::vector<float>& v) {
    float m = 0.0f;
    for (float x : v) m = std::max(m, std::fabs(x));
    return m;
}

TEST(AirflowPressure, AUniformFlowHasExactlyZeroDivergence) {
    MacGrid g = *make_grid({4, 4, 4, 0.1f}, DomainBc{});
    std::fill(g.u.begin(), g.u.end(), 1.5f);
    std::fill(g.v.begin(), g.v.end(), -0.25f);
    std::fill(g.w.begin(), g.w.end(), 3.0f);
    std::vector<float> div(g.p.size(), 7.0f);
    divergence(g, div);
    for (float d : div) EXPECT_EQ(d, 0.0f);
}

TEST(AirflowPressure, ALinearFlowsDivergenceIsItsSlope) {
    MacGrid g = *make_grid({4, 4, 4, 0.5f}, DomainBc{});
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = 2.0f * x.x; });
    std::vector<float> div(g.p.size(), 0.0f);
    divergence(g, div);
    for (float d : div) EXPECT_NEAR(d, 2.0f, 1.0e-6f);
}

// b set from a known p through the same arithmetic, so the residual is
// exactly 0: the expression is deterministic, which the GPU twin relies on.
TEST(AirflowPressure, AManufacturedSolutionLeavesAZeroResidual) {
    PoissonLevel L = make_level(4, 4, 4, all(FaceKind::pressure_outlet));
    Lcg rng;
    for (float& x : L.p) x = rng.next();
    std::fill(L.b.begin(), L.b.end(), 0.0f);
    residual(L);  // r = 0 - (sum - diag p)
    for (std::size_t c = 0; c < L.b.size(); ++c) L.b[c] = -L.r[c];
    residual(L);
    for (float r : L.r) EXPECT_EQ(r, 0.0f);
}

TEST(AirflowPressure, SmoothingReducesTheResidual) {
    PoissonLevel L = make_level(16, 16, 16, all(FaceKind::pressure_outlet));
    Lcg rng;
    for (float& x : L.b) x = rng.next();
    residual(L);
    const float r0 = max_abs(L.r);
    smooth_red_black(L, 10);
    residual(L);
    EXPECT_LT(max_abs(L.r), 0.5f * r0);
}

// Red-black is order-free within a colour: sweeping the cells in reverse order
// gives the same bits. That is the property that lets the GPU update every
// cell of a colour at once.
TEST(AirflowPressure, AColourSweepDoesNotDependOnItsCellOrder) {
    std::array<FaceKind, 6> kinds = all(FaceKind::periodic);
    kinds[kXMinus] = kinds[kXPlus] = FaceKind::pressure_outlet;
    PoissonLevel forward = make_level(8, 6, 4, kinds);
    Lcg rng;
    for (float& x : forward.b) x = rng.next();
    for (float& x : forward.p) x = rng.next();
    PoissonLevel reverse = forward;
    smooth_red_black(forward, 3);
    for (uint32_t sweep = 0; sweep < 3; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            for (uint32_t k = reverse.nz; k-- > 0;) {
                for (uint32_t j = reverse.ny; j-- > 0;) {
                    for (uint32_t i = reverse.nx; i-- > 0;) {
                        if (((i + j + k) & 1u) == colour) smooth_one_cell(reverse, i, j, k);
                    }
                }
            }
        }
    }
    EXPECT_EQ(forward.p, reverse.p);
}

}  // namespace
