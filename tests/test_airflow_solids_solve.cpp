// The pressure solve with solids (the solids design, section 3; spec 1.5 as
// amended 2026-10-13): face weights, their coarse means, the symmetric
// V-cycle, CG, and the fluid regions (P2 Tasks 3-5).
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pressure.hpp"
#include "physics/airflow/solids.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 2463534242u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] std::array<std::vector<float>, 3> ones(uint32_t nx, uint32_t ny, uint32_t nz) {
    return {std::vector<float>(std::size_t{nx + 1u} * ny * nz, 1.0f),
            std::vector<float>(std::size_t{nx} * (ny + 1u) * nz, 1.0f),
            std::vector<float>(std::size_t{nx} * ny * (nz + 1u), 1.0f)};
}

[[nodiscard]] bool same_bits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

// Weights of exactly 1 are plan A's stencil bit for bit: 1.0f * p is p, and
// the six weights (and an outlet's 2 * 1) sum as plan A's integers do. Every
// face kind is present: x periodic, y outlets, z fixed.
TEST(AirflowSolidsSolve, WeightsOfOneAreBitIdentical) {
    const std::array<FaceKind, 6> kinds{FaceKind::periodic,        FaceKind::periodic,       FaceKind::pressure_outlet,
                                        FaceKind::pressure_outlet, FaceKind::fixed_velocity, FaceKind::fixed_velocity};
    PoissonLevel plain = make_level(8, 6, 5, kinds);
    Lcg rng;
    for (float& v : plain.p) v = rng.next();
    for (float& v : plain.b) v = rng.next();
    PoissonLevel weighted = plain;
    weighted.w = ones(8, 6, 5);
    smooth_red_black(plain, 3);
    smooth_red_black(weighted, 3);
    EXPECT_TRUE(same_bits(plain.p, weighted.p));
    residual(plain);
    residual(weighted);
    EXPECT_TRUE(same_bits(plain.r, weighted.r));
    std::vector<float> out_plain(plain.p.size());
    std::vector<float> out_weighted(plain.p.size());
    apply_operator(plain, plain.p, out_plain);
    apply_operator(weighted, weighted.p, out_weighted);
    EXPECT_TRUE(same_bits(out_plain, out_weighted));
    for (std::size_t c = 0; c < plain.r.size(); ++c) {
        EXPECT_EQ(plain.r[c], plain.b[c] - out_plain[c]) << "apply_operator is the residual's operator";
    }
}

// A closed face (weight 0) decouples the two cells on either side: with p = 0
// on the left half and 1 on the right, the operator is 0 everywhere but at the
// open faces between the halves, and there are none.
TEST(AirflowSolidsSolve, AClosedFaceDecouplesItsCells) {
    const std::array<FaceKind, 6> walls{FaceKind::fixed_velocity, FaceKind::fixed_velocity, FaceKind::fixed_velocity,
                                        FaceKind::fixed_velocity, FaceKind::fixed_velocity, FaceKind::fixed_velocity};
    PoissonLevel L = make_level(4, 2, 2, walls);
    L.w = ones(4, 2, 2);
    for (uint32_t k = 0; k < 2; ++k) {
        for (uint32_t j = 0; j < 2; ++j) L.w[0][2u + 5u * (j + 2u * k)] = 0.0f;  // the u faces at i = 2
    }
    for (uint32_t k = 0; k < 2; ++k) {
        for (uint32_t j = 0; j < 2; ++j) {
            for (uint32_t i = 0; i < 4; ++i) L.p[i + 4u * (j + 2u * k)] = i < 2 ? 0.0f : 1.0f;
        }
    }
    std::vector<float> out(L.p.size());
    apply_operator(L, L.p, out);
    for (const float v : out) EXPECT_EQ(v, 0.0f);
}

// A coarse face's weight is the mean of the fine faces on its plane that it
// covers: 2 x 2 of them.
TEST(AirflowSolidsSolve, CoarseWeightsAreTheMeanOfTheFacesTheyCover) {
    Multigrid mg = make_multigrid(8, 8, 8);
    std::array<std::vector<float>, 3> fine = ones(8, 8, 8);
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t q = 0; q < fine[c].size(); ++q) fine[c][q] = static_cast<float>((q * 7u + c) % 5u) * 0.25f;
    }
    set_weights(mg, {fine[0], fine[1], fine[2]});
    ASSERT_GE(mg.levels.size(), 2u);
    const PoissonLevel& coarse = mg.levels[1];
    ASSERT_EQ(coarse.w[0].size(), std::size_t{5} * 4 * 4);
    for (uint32_t K = 0; K < 4; ++K) {
        for (uint32_t J = 0; J < 4; ++J) {
            for (uint32_t I = 0; I <= 4; ++I) {
                const auto f = [&](uint32_t i, uint32_t j, uint32_t k) { return fine[0][i + 9u * (j + 8u * k)]; };
                float sum = 0.0f;
                sum += f(2u * I, 2u * J, 2u * K);
                sum += f(2u * I, 2u * J + 1u, 2u * K);
                sum += f(2u * I, 2u * J, 2u * K + 1u);
                sum += f(2u * I, 2u * J + 1u, 2u * K + 1u);
                EXPECT_EQ(coarse.w[0][I + 5u * (J + 4u * K)], sum * 0.25f) << I << "," << J << "," << K;
            }
        }
    }
    clear_weights(mg);
    for (const PoissonLevel& L : mg.levels) EXPECT_TRUE(L.w[0].empty());
}

}  // namespace
