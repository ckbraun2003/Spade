// The pressure solve with solids (the solids design, section 3; spec 1.5 as
// amended 2026-10-13): face weights, their coarse means, the symmetric
// V-cycle, CG, and the fluid regions (P2 Tasks 3-5).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pcg.hpp"
#include "physics/airflow/pressure.hpp"
#include "physics/airflow/reduce.hpp"
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

// --- The symmetric V-cycle and CG (P2 Task 4) -------------------------------

// S3's channel: 32 x 8 x 8 cells, every axis periodic, dx 1, and a sheet
// 0.25 dx thick across the whole section, its mid-plane on fine face plane
// `plane` (16: every level sees it; 17: no coarse level does).
[[nodiscard]] std::array<std::vector<float>, 3> sheet_weights(uint32_t plane) {
    const GridShape shape{32, 8, 8, 1.0f};
    airflow_test::Scene scene;
    scene.shapes.push_back(airflow_test::Shape{.name = "plate", .kind = airflow_test::Kind::box,
                                               .centre = {static_cast<float>(plane), 4.0f, 4.0f},
                                               .half = {0.125f, 20.0f, 20.0f}, .min_feature = 0.25f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].box_min = glm::vec3(static_cast<float>(plane) - 0.125f, 0.0f, 0.0f);
    parts[0].box_max = glm::vec3(static_cast<float>(plane) + 0.125f, 8.0f, 8.0f);
    parts[0].extent = glm::vec3(0.25f, 8.0f, 8.0f);
    SolidField f = *make_solid_field(shape);
    EXPECT_TRUE(build_solids(f, shape, {true, true, true}, parts, scene.sampler()).has_value());
    return f.theta;
}

[[nodiscard]] Multigrid sheet_multigrid(uint32_t plane) {
    const std::array<bool, 3> periodic{true, true, true};
    Multigrid mg = make_multigrid(32, 8, 8, periodic);
    set_kinds(mg, {FaceKind::periodic, FaceKind::periodic, FaceKind::periodic, FaceKind::periodic,
                   FaceKind::periodic, FaceKind::periodic});
    const std::array<std::vector<float>, 3> w = sheet_weights(plane);
    set_weights(mg, {w[0], w[1], w[2]});
    return mg;
}

void remove_mean_gauge(void* ctx, std::span<float> cells) noexcept {
    (void)ctx;
    remove_mean(cells, 32, 8, 8);
}

[[nodiscard]] std::vector<float> apply_m(Multigrid& mg, const std::vector<float>& r) {
    std::copy(r.begin(), r.end(), mg.levels[0].b.begin());
    precondition(mg);
    return mg.levels[0].p;
}

[[nodiscard]] double dot64(const std::vector<float>& a, const std::vector<float>& b) {
    double s = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) s += static_cast<double>(a[i]) * static_cast<double>(b[i]);
    return s;
}

// Design test 10 (the lead's condition): CG needs a symmetric preconditioner,
// <M r1, r2> = <r1, M r2>, to fp32 rounding, on S3's grid with its sheet on
// an odd plane. Mutation: plan A's restriction (the children's sum) in the
// cycle, which the port measured at 1e-1 of asymmetry.
TEST(AirflowSolidsSolve, ThePreconditionerIsSymmetric) {
    Multigrid mg = sheet_multigrid(17);
    Lcg rng;
    std::vector<float> r1(32u * 8u * 8u);
    std::vector<float> r2(r1.size());
    for (float& v : r1) v = rng.next();
    for (float& v : r2) v = rng.next();
    remove_mean(r1, 32, 8, 8);
    remove_mean(r2, 32, 8, 8);
    const std::vector<float> m1 = apply_m(mg, r1);
    const std::vector<float> m2 = apply_m(mg, r2);
    const double a = dot64(m1, r2);
    const double b = dot64(r1, m2);
    EXPECT_NEAR(a, b, 1.0e-4 * std::fabs(a)) << a << " vs " << b;
    EXPECT_LT(dot64(m1, r1), 0.0) << "negative definite on mean-free vectors, as the operator (sum - diag p) is";
}

// The right side of one projection of a uniform u = 1 blocked by the sheet:
// the aperture divergence theta_{i+1} - theta_i, exact integers.
[[nodiscard]] std::vector<float> blocked_flow_rhs(const std::array<std::vector<float>, 3>& w) {
    std::vector<float> b(32u * 8u * 8u);
    for (uint32_t k = 0; k < 8; ++k) {
        for (uint32_t j = 0; j < 8; ++j) {
            for (uint32_t i = 0; i < 32; ++i) {
                b[i + 32u * (j + 8u * k)] = w[0][(i + 1u) + 33u * (j + 8u * k)] - w[0][i + 33u * (j + 8u * k)];
            }
        }
    }
    return b;
}

// Design test 3's convergence check (the lead's finding 2): from a cold start,
// on both parities, CG preconditioned by the symmetric V-cycle brings the
// relative residual to 1e-3 within 8 iterations. An implementation band
// (TD-2), not a Test/Docs tolerance: the port (Physics' ledger,
// .superpowers/sdd/airflow-solids/mg_sym_port.py) measured 1-2e-3 at 4
// iterations and 1e-8 at 8, in fp64; pinned at 8. Measured here (MSVC release,
// 2026-10-19): 2.3e-5 on plane 16, 1.5e-5 on plane 17, fp32's floor. Mutation:
// plan A's plain V-cycles on the odd plane, which the port leaves at 0.6 after 8.
TEST(AirflowSolidsSolve, ASheetOnEitherParityConvergesInEightIterations) {
    for (const uint32_t plane : {16u, 17u}) {
        Multigrid mg = sheet_multigrid(plane);
        const std::array<std::vector<float>, 3> w = sheet_weights(plane);
        PcgScratch s = make_pcg_scratch(32u * 8u * 8u);
        s.b = blocked_flow_rhs(w);
        std::fill(s.x.begin(), s.x.end(), 0.0f);
        const float rel = solve_pcg(mg, s, 8, PcgGauge{&remove_mean_gauge, nullptr, nullptr});
        EXPECT_LE(rel, 1.0e-3f) << "plane " << plane;
        std::printf("plane %u: relative residual after 8 = %.3e\n", plane, static_cast<double>(rel));
    }
}

TEST(AirflowSolidsSolve, TheFixedTileDotIsTheTileSumOfProducts) {
    std::vector<float> a(9u * 7u * 5u);
    std::vector<float> b(a.size());
    std::vector<float> ab(a.size());
    Lcg rng;
    for (std::size_t i = 0; i < a.size(); ++i) {
        a[i] = rng.next();
        b[i] = rng.next();
        ab[i] = a[i] * b[i];
    }
    const float d = fixed_tile_dot(a, b, 9, 7, 5);
    const float s = fixed_tile_sum(ab, 9, 7, 5);
    EXPECT_EQ(std::memcmp(&d, &s, sizeof d), 0) << d << " vs " << s;
}

}  // namespace
