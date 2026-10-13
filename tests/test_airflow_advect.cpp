// The airflow solver core's advection (airflow core plan, Task 6). Not a
// golden-feeding file: it computes reference values in double with std::sin.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

#include "physics/airflow/advect.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/viscous.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] DomainBc periodic_box() {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    return bc;
}

// dx a power of two, so x * (1 / dx) is exact and a sample at a stored
// position reads exactly the stored value.
TEST(AirflowAdvect, SamplingReadsAStoredValueAtItsOwnPosition) {
    const DomainBc bc = periodic_box();
    MacGrid g = *make_grid({4, 4, 4, 0.25f}, bc);
    Lcg rng;
    for (float& x : g.v) x = rng.next();
    apply_velocity_boundaries(g, bc);  // face ny is face 0 on a periodic axis
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) {
        EXPECT_EQ(sample_component(g.shape, g.v, 1, x, bc).value, g.v[idx]) << idx;
    });
}

TEST(AirflowAdvect, SamplingWrapsAPeriodicAxisAndClampsAnOtherAxis) {
    DomainBc bc = periodic_box();
    bc.face[kYMinus] = bc.face[kYPlus] = FaceBc::wall;
    MacGrid g = *make_grid({4, 4, 1, 0.25f}, bc);
    Lcg rng;
    for (float& x : g.u) x = rng.next();
    for (uint32_t j = 0; j < 4; ++j) g.u[u_index(g.shape, 4, j, 0)] = g.u[u_index(g.shape, 0, j, 0)];
    const glm::vec3 inside(0.25f, 0.375f, 0.125f);
    EXPECT_EQ(sample_component(g.shape, g.u, 0, inside + glm::vec3(1.0f, 0.0f, 0.0f), bc).value,
              sample_component(g.shape, g.u, 0, inside, bc).value)
        << "x is periodic with period nx dx = 1";
    EXPECT_EQ(sample_component(g.shape, g.u, 0, glm::vec3(0.25f, -5.0f, 0.125f), bc).value,
              sample_component(g.shape, g.u, 0, glm::vec3(0.25f, 0.125f, 0.125f), bc).value)
        << "y clamps to the first row of samples";
}

TEST(AirflowAdvect, AUniformFlowIsReproducedBitForBit) {
    const DomainBc bc = periodic_box();
    MacGrid in = *make_grid({8, 8, 8, 0.125f}, bc);
    std::fill(in.u.begin(), in.u.end(), 1.0f);
    std::fill(in.v.begin(), in.v.end(), 2.0f);
    std::fill(in.w.begin(), in.w.end(), -3.0f);
    MacGrid out = in;
    AdvectScratch s = make_advect_scratch(in.shape);
    advect_velocity(in, out, bc, 0.01f, s);
    EXPECT_EQ(out.u, in.u);
    EXPECT_EQ(out.v, in.v);
    EXPECT_EQ(out.w, in.w);
}

TEST(AirflowAdvect, TheClampMakesNoNewExtrema) {
    const DomainBc bc = periodic_box();
    MacGrid in = *make_grid({8, 8, 8, 0.125f}, bc);
    Lcg rng;
    for (float& x : in.u) x = rng.next();
    for (float& x : in.v) x = rng.next();
    for (float& x : in.w) x = rng.next();
    MacGrid out = in;
    AdvectScratch s = make_advect_scratch(in.shape);
    advect_velocity(in, out, bc, 0.05f, s);
    const auto range = [](const std::vector<float>& v) { return std::minmax_element(v.begin(), v.end()); };
    for (const auto& [a, b] : {std::pair{&in.u, &out.u}, std::pair{&in.v, &out.v}, std::pair{&in.w, &out.w}}) {
        EXPECT_GE(*range(*b).first, *range(*a).first);
        EXPECT_LE(*range(*b).second, *range(*a).second);
    }
}

// A sine in v carried once across a periodic box by a uniform u of 1 m/s at
// CFL 0.5 comes back to within 5% of its amplitude.
TEST(AirflowAdvect, ASmoothWaveCrossesThePeriodicBoxAndReturns) {
    const DomainBc bc = periodic_box();
    MacGrid g = *make_grid({64, 4, 4, 1.0f / 64.0f}, bc);
    std::fill(g.u.begin(), g.u.end(), 1.0f);
    const double amplitude = 1.0e-3;
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) {
        g.v[idx] = static_cast<float>(amplitude * std::sin(2.0 * std::numbers::pi * x.x));
    });
    const std::vector<float> start = g.v;
    MacGrid out = g;
    AdvectScratch s = make_advect_scratch(g.shape);
    const float h = 0.5f / 64.0f;
    for (int step = 0; step < 128; ++step) {
        advect_velocity(g, out, bc, h, s);
        std::swap(g.u, out.u);
        std::swap(g.v, out.v);
        std::swap(g.w, out.w);
    }
    float worst = 0.0f;
    for (std::size_t i = 0; i < start.size(); ++i) worst = std::max(worst, std::fabs(g.v[i] - start[i]));
    std::printf("one crossing at CFL 0.5, 64 cells per wavelength: max error %.3e of amplitude %.1e\n",
                static_cast<double>(worst), amplitude);
    EXPECT_LT(worst, 0.05f * static_cast<float>(amplitude));
}

// Review Focus 1.
TEST(AirflowAdvect, AVelocityFarPastTheGridStaysFinite) {
    for (bool periodic : {true, false}) {
        DomainBc bc = periodic ? periodic_box() : DomainBc{};
        MacGrid in = *make_grid({8, 8, 8, 0.125f}, bc);
        Lcg rng;
        for (float& x : in.u) x = 1.0e6f * rng.next();
        for (float& x : in.v) x = 1.0e6f * rng.next();
        for (float& x : in.w) x = 1.0e6f * rng.next();
        MacGrid out = in;
        AdvectScratch s = make_advect_scratch(in.shape);
        advect_velocity(in, out, bc, 0.01f, s);
        for (float x : out.u) ASSERT_TRUE(std::isfinite(x)) << (periodic ? "periodic" : "clamped");
        for (float x : out.v) ASSERT_TRUE(std::isfinite(x));
        for (float x : out.w) ASSERT_TRUE(std::isfinite(x));
    }
}

// The final review's M8: the sampler's index arithmetic meets a NaN, an
// infinity and a huge coordinate directly (a velocity that is itself not
// finite). Each lands on a valid sample: the value and its range come from
// the field's own values, never from outside the array.
TEST(AirflowAdvect, ANonFiniteOrHugeCoordinateSamplesInsideTheField) {
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (bool periodic : {true, false}) {
        const DomainBc bc = periodic ? periodic_box() : DomainBc{};
        MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
        for (std::size_t i = 0; i < g.u.size(); ++i) g.u[i] = 1.0f + static_cast<float>(i % 13u);
        for (const float c : {nan, inf, -inf, 3.0e38f, -3.0e38f}) {
            for (const glm::vec3 x : {glm::vec3(c, 0.3f, 0.3f), glm::vec3(0.3f, c, 0.3f), glm::vec3(0.3f, 0.3f, c),
                                      glm::vec3(c)}) {
                const Sample s = sample_component(g.shape, g.u, 0, x, bc);
                EXPECT_TRUE(std::isfinite(s.value)) << (periodic ? "periodic " : "clamped ") << c;
                EXPECT_GE(s.lo, 1.0f);
                EXPECT_LE(s.hi, 13.0f);
                EXPECT_GE(s.value, s.lo);
                EXPECT_LE(s.value, s.hi);
            }
        }
    }
}

[[nodiscard]] std::vector<float> rhs_for(const MacGrid& g) {
    return std::vector<float>(std::max({g.u.size(), g.v.size(), g.w.size()}), 0.0f);
}

TEST(AirflowViscous, WithoutViscosityNothingChanges) {
    const DomainBc bc = periodic_box();
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
    Lcg rng;
    for (float& x : g.u) x = rng.next();
    const MacGrid before = g;
    std::vector<float> rhs = rhs_for(g);
    diffuse(g, bc, 0.0f, 0.01f, 8, rhs);
    EXPECT_EQ(g.u, before.u);
}

// Plane Couette flow: a wall at rest at y = 0, a wall moving at 1 m/s along x
// at y = 1. The discrete steady state with the no-slip ghost is exactly
// linear, u = y.
TEST(AirflowViscous, CouetteFlowSettlesToALinearProfile) {
    DomainBc bc = periodic_box();
    bc.face[kYMinus] = bc.face[kYPlus] = FaceBc::wall;
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    MacGrid g = *make_grid({4, 16, 1, 1.0f / 16.0f}, bc);
    std::vector<float> rhs = rhs_for(g);
    for (int step = 0; step < 200; ++step) diffuse(g, bc, 1.0f, 1.0f, 50, rhs);
    for (uint32_t j = 0; j < 16; ++j) {
        const float expected = (static_cast<float>(j) + 0.5f) / 16.0f;
        EXPECT_NEAR(g.u[u_index(g.shape, 1, j, 0)], expected, 1.0e-4f) << j;
    }
}

// One converged implicit step damps a sine mode by exactly the discrete
// backward-Euler factor 1 / (1 + 4 alpha sin^2(pi dx / L)).
TEST(AirflowViscous, ASineModeDecaysAtTheImplicitRate) {
    const DomainBc bc = periodic_box();
    const uint32_t n = 32;
    MacGrid g = *make_grid({4, n, 1, 1.0f / static_cast<float>(n)}, bc);
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) {
        g.u[idx] = static_cast<float>(std::sin(2.0 * std::numbers::pi * x.y));
    });
    const std::vector<float> start = g.u;
    const float nu = 0.5f;
    const float h = 1.0f / static_cast<float>(n * n);  // alpha = nu h / dx^2 = 0.5
    std::vector<float> rhs = rhs_for(g);
    diffuse(g, bc, nu, h, 400, rhs);
    const double s = std::sin(std::numbers::pi / n);
    const double factor = 1.0 / (1.0 + 4.0 * 0.5 * s * s);
    for (std::size_t i = 0; i < start.size(); ++i) {
        EXPECT_NEAR(g.u[i], static_cast<float>(factor * start[i]), 1.0e-5f) << i;
    }
}

TEST(AirflowViscous, AMovingWallDragsTheFluidBesideIt) {
    DomainBc bc = periodic_box();
    bc.face[kYMinus] = bc.face[kYPlus] = FaceBc::wall;
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    MacGrid g = *make_grid({4, 8, 1, 0.125f}, bc);
    std::vector<float> rhs = rhs_for(g);
    diffuse(g, bc, 0.01f, 0.01f, 8, rhs);
    EXPECT_GT(g.u[u_index(g.shape, 1, 7, 0)], 0.0f) << "the row beside the lid";
    EXPECT_GT(g.u[u_index(g.shape, 1, 7, 0)], g.u[u_index(g.shape, 1, 6, 0)]);
}

}  // namespace
