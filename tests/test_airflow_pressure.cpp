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
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pressure.hpp"
#include "physics/airflow/projection.hpp"
#include "physics/airflow/reduce.hpp"

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

TEST(AirflowMultigrid, HalvesEveryParticipatingAxisUntilOneIsTooSmall) {
    const Multigrid slab = make_multigrid(128, 128, 1);
    ASSERT_EQ(slab.levels.size(), 7u);
    EXPECT_EQ(slab.levels.back().nx, 2u);
    EXPECT_EQ(slab.levels.back().nz, 1u) << "a size-1 axis takes no part";
    const Multigrid box = make_multigrid(64, 64, 128);
    ASSERT_EQ(box.levels.size(), 6u);
    EXPECT_EQ(box.levels.back().nx, 2u);
    EXPECT_EQ(box.levels.back().nz, 4u);
    EXPECT_EQ(make_multigrid(6, 6, 6).levels.size(), 2u) << "6 -> 3, and 3 is odd";
}

TEST(AirflowMultigrid, OneVCycleCutsTheResidualAtLeastThreefold) {
    for (const std::array<uint32_t, 3>& n : {std::array<uint32_t, 3>{32, 32, 32}, std::array<uint32_t, 3>{64, 64, 1}}) {
        Multigrid mg = make_multigrid(n[0], n[1], n[2]);
        std::array<FaceKind, 6> kinds = all(FaceKind::pressure_outlet);
        kinds[kYMinus] = kinds[kYPlus] = FaceKind::fixed_velocity;
        if (n[2] == 1u) kinds[kZMinus] = kinds[kZPlus] = FaceKind::periodic;
        set_kinds(mg, kinds);
        Lcg rng;
        for (float& x : mg.levels[0].b) x = rng.next();
        residual(mg.levels[0]);
        const float r0 = max_abs(mg.levels[0].r);
        v_cycle(mg);
        residual(mg.levels[0]);
        const float r1 = max_abs(mg.levels[0].r);
        std::printf("V(2,2) on %ux%ux%u: residual %.3e -> %.3e, factor %.1f\n", n[0], n[1], n[2],
                    static_cast<double>(r0), static_cast<double>(r1), static_cast<double>(r0 / r1));
        EXPECT_LT(r1, r0 / 3.0f) << n[0] << "x" << n[1] << "x" << n[2];
    }
}

TEST(AirflowMultigrid, TwoRunsAreBitIdentical) {
    const auto run = [] {
        Multigrid mg = make_multigrid(16, 16, 16);
        set_kinds(mg, all(FaceKind::pressure_outlet));
        Lcg rng;
        for (float& x : mg.levels[0].b) x = rng.next();
        for (int n = 0; n < 3; ++n) v_cycle(mg);
        return mg.levels[0].p;
    };
    EXPECT_EQ(run(), run());
}

[[nodiscard]] float max_divergence(const MacGrid& g) {
    std::vector<float> div(g.p.size(), 0.0f);
    divergence(g, div);
    return max_abs(div);
}

void randomise(MacGrid& g, uint32_t seed) {
    Lcg rng{seed};
    for (float& x : g.u) x = rng.next();
    for (float& x : g.v) x = rng.next();
    for (float& x : g.w) x = rng.next();
}

TEST(AirflowProjection, AUniformFlowPassesThroughUntouched) {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid({8, 8, 8, 0.1f}, bc);
    std::fill(g.u.begin(), g.u.end(), 1.5f);
    std::fill(g.v.begin(), g.v.end(), -0.5f);
    const MacGrid before = g;
    ProjectionScratch s = make_projection_scratch(g.shape);
    project(g, bc, 1.225f, 0.004f, 2, s);
    EXPECT_EQ(g.u, before.u);
    EXPECT_EQ(g.v, before.v);
    EXPECT_EQ(g.w, before.w);
    for (float p : g.p) EXPECT_EQ(p, 0.0f);
}

TEST(AirflowProjection, MoreCyclesLeaveLessDivergence) {
    DomainBc bc;  // still air: every face an outlet
    float previous = 0.0f;
    for (uint32_t n_v : {1u, 2u, 4u, 8u}) {
        MacGrid g = *make_grid({32, 32, 32, 0.05f}, bc);
        randomise(g, 7u);
        apply_velocity_boundaries(g, bc);
        const float before = max_divergence(g);
        ProjectionScratch s = make_projection_scratch(g.shape);
        project(g, bc, 1.225f, 0.004f, n_v, s);
        const float after = max_divergence(g);
        std::printf("n_v %u: max |div| %.3e -> %.3e\n", n_v, static_cast<double>(before), static_cast<double>(after));
        if (n_v > 1u) {
            EXPECT_LT(after, previous) << n_v;
        }
        previous = after;
        if (n_v == 8u) {
            EXPECT_LT(after, 1.0e-4f * before);
        }
    }
}

// Review Focus 3: Neumann on every face, so p is fixed only up to a constant.
TEST(AirflowProjection, AClosedBoxStaysFiniteAndItsDivergenceFalls) {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    MacGrid g = *make_grid({16, 16, 16, 0.05f}, bc);
    randomise(g, 11u);
    apply_velocity_boundaries(g, bc);
    const float before = max_divergence(g);
    ProjectionScratch s = make_projection_scratch(g.shape);
    for (int n = 0; n < 20; ++n) project(g, bc, 1.225f, 0.004f, 2, s);
    for (float x : g.p) ASSERT_TRUE(std::isfinite(x));
    EXPECT_LT(max_divergence(g), 1.0e-3f * before);
}

// Review Focus 4.
TEST(AirflowProjection, AFaceFollowsTheAmbientEachStep) {
    DomainBc bc;
    bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);  // x- is an inflow, fixed
    MacGrid g = *make_grid({8, 4, 4, 0.1f}, bc);
    randomise(g, 3u);
    apply_velocity_boundaries(g, bc);
    ProjectionScratch s = make_projection_scratch(g.shape);
    project(g, bc, 1.225f, 0.004f, 2, s);
    EXPECT_EQ(g.u[u_index(g.shape, 0, 2, 2)], 1.0f) << "an inflow face is fixed";
    bc.ambient = glm::vec3(-1.0f, 0.0f, 0.0f);  // now x- is an outlet
    g.u[u_index(g.shape, 0, 2, 2)] = 0.0f;
    project(g, bc, 1.225f, 0.004f, 2, s);
    EXPECT_NE(g.u[u_index(g.shape, 0, 2, 2)], 0.0f) << "an outlet face is corrected by the projection";
}

// The lead's ask (2026-10-06): with no outlet face (every face a wall, or
// every axis periodic) p is fixed only up to a constant. The projection
// removes the mean of the right side and of p in a fixed order, so the
// pressure is repeatable and its mean is zero, never drifting.
TEST(AirflowProjection, AClosedBoxsPressureIsRepeatableWithZeroMean) {
    for (const FaceBc kind : {FaceBc::wall, FaceBc::periodic}) {
        const auto run = [kind] {
            DomainBc bc;
            bc.face.fill(kind);
            MacGrid g = *make_grid({16, 16, 16, 0.05f}, bc);
            randomise(g, 11u);
            apply_velocity_boundaries(g, bc);
            ProjectionScratch s = make_projection_scratch(g.shape);
            for (int n = 0; n < 5; ++n) project(g, bc, 1.225f, 0.004f, 2, s);
            return g.p;
        };
        const std::vector<float> p = run();
        EXPECT_EQ(p, run());
        double sum = 0.0;
        for (float x : p) sum += x;
        const double mean = sum / static_cast<double>(p.size());
        EXPECT_LE(std::fabs(mean), 1.0e-6 * static_cast<double>(max_abs(p)))
            << (kind == FaceBc::wall ? "walls" : "periodic") << ": mean " << mean;
    }
}


// THE FIXED-ORDER REDUCTION (spec §4.5 as amended 2026-10-12; Core's stage 7
// design §5.5): 4x4x4 tiles, each summed in (k, j, i) order from 0, then the
// tiles in (tk, tj, ti) order from 0, all in fp32. This is its definition,
// written out independently of reduce.cpp; the GPU twin sums the same way.
[[nodiscard]] float tile_sum_by_definition(const std::vector<float>& x, uint32_t nx, uint32_t ny, uint32_t nz) {
    float total = 0.0f;
    for (uint32_t tk = 0; tk < nz; tk += 4u) {
        for (uint32_t tj = 0; tj < ny; tj += 4u) {
            for (uint32_t ti = 0; ti < nx; ti += 4u) {
                float tile = 0.0f;
                for (uint32_t k = tk; k < std::min(tk + 4u, nz); ++k) {
                    for (uint32_t j = tj; j < std::min(tj + 4u, ny); ++j) {
                        for (uint32_t i = ti; i < std::min(ti + 4u, nx); ++i) {
                            tile += x[i + std::size_t{nx} * (j + std::size_t{ny} * k)];
                        }
                    }
                }
                total += tile;
            }
        }
    }
    return total;
}

TEST(AirflowReduce, TheFixedTileSumIsItsDefinitionBitForBit) {
    for (const std::array<uint32_t, 3>& n :
         {std::array<uint32_t, 3>{8, 8, 8}, std::array<uint32_t, 3>{6, 5, 3}, std::array<uint32_t, 3>{16, 16, 1}}) {
        std::vector<float> x(std::size_t{n[0]} * n[1] * n[2]);
        Lcg rng;
        for (float& v : x) v = 1000.0f * rng.next();
        EXPECT_EQ(fixed_tile_sum(x, n[0], n[1], n[2]), tile_sum_by_definition(x, n[0], n[1], n[2]))
            << n[0] << "x" << n[1] << "x" << n[2];
    }
}

// The closed-domain mean (Task 5) is the fixed-tile sum times the host's
// reciprocal of the count, subtracted in array order: bit for bit.
TEST(AirflowReduce, RemovingTheMeanSubtractsTheFixedTileMean) {
    const uint32_t nx = 6;
    const uint32_t ny = 5;
    const uint32_t nz = 3;
    std::vector<float> x(std::size_t{nx} * ny * nz);
    Lcg rng;
    for (float& v : x) v = 3.0f + rng.next();
    const auto inv_count = static_cast<float>(1.0 / static_cast<double>(x.size()));
    const float mean = tile_sum_by_definition(x, nx, ny, nz) * inv_count;
    std::vector<float> expected = x;
    for (float& v : expected) v -= mean;
    remove_mean(x, nx, ny, nz);
    EXPECT_EQ(x, expected);
}

}  // namespace
