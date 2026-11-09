// test_lbm_flow.cpp -- L1's flow step on a periodic grid: RR3 + WALE in tau
// (design: docs/design/physics/plans/2026-11-04-l1-lbm-design.md, sections 4
// and 6), and the stability map's point runner.
//
//   LbmFlow      fast checks: a quiet box stays quiet; mass and momentum are
//                conserved; a shear wave decays at nu k^2 and the same rate on
//                a moving lattice (Galilean invariance); WALE adds nothing to
//                a laminar wave.
//   LbmThreads   the step split over a WorkerPool is the same bits as the serial step, on any thread count and
//                with the chunks run out of order (ItemOrder::reversed: an order-dependent split fails here by
//                construction, not by the luck of a race), periodic and in a box with a moving lid.
//   LbmFlush     the reference step under the flush scope: a field of subnormal populations is a quiet field
//                (DAZ), serial and split, and the caller's MXCSR comes back.
//   LbmMap       env-gated (SPADE_LBM_VALIDATION=1, SPADE_LBM_MAP="<u_lat>:<nu_lat>"):
//                one point of the stability map, C0b's field at 64^3, printed
//                as the record's measures plus MLUPS. Report only.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include <xmmintrin.h>

#include "lbm_support.hpp"
#include "physics/lbm/lbm_flow.hpp"

namespace {

using namespace spade::physics::lbm;

constexpr double kTwoPi = 2.0 * std::numbers::pi;

const char* env_text(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

TEST(LbmFlow, AQuietBoxStaysQuietExactly) {
    LbmFlow fl = make_flow(8, 8, 8, 0.01f, 0.5f);
    for (uint32_t z = 0; z < 8; ++z) {
        for (uint32_t y = 0; y < 8; ++y) {
            for (uint32_t x = 0; x < 8; ++x) set_node_equilibrium(fl, x, y, z, 1.0f, 0.0f, 0.0f, 0.0f);
        }
    }
    for (int t = 0; t < 20; ++t) spade::testing::lbm_support::step_ok(fl);
    for (const float v : fl.grid.f) EXPECT_EQ(v, 0.0f);
}

// A shear wave u_y = a sin(2 pi x / N) on a lattice moving at u0 along x. Returns the decay rate of the wave's
// amplitude per step, read from the projection on sin and cos (the wave is carried by u0, so its phase moves).
double shear_decay_rate(float u0, float nu, float cw, int steps, uint32_t n = 32) {
    LbmFlow fl = make_flow(n, 4, 4, nu, cw);
    const double a = 0.01;
    for (uint32_t z = 0; z < 4; ++z) {
        for (uint32_t y = 0; y < 4; ++y) {
            for (uint32_t x = 0; x < n; ++x) {
                const double uy = a * std::sin(kTwoPi * x / n);
                set_node_equilibrium(fl, x, y, z, 1.0f, u0, static_cast<float>(uy), 0.0f);
            }
        }
    }
    const auto amplitude = [&]() {
        compute_velocity(fl);
        double s = 0.0;
        double c = 0.0;
        for (uint32_t x = 0; x < n; ++x) {
            const double uy = fl.u_at(1, fl.grid.cell(x, 0, 0));
            s += uy * std::sin(kTwoPi * x / n);
            c += uy * std::cos(kTwoPi * x / n);
        }
        return std::hypot(s, c) * 2.0 / n;
    };
    const double a0 = amplitude();
    for (int t = 0; t < steps; ++t) spade::testing::lbm_support::step_ok(fl);
    return -std::log(amplitude() / a0) / steps;
}

// The lattice is second order: the wave's decay rate differs from nu k^2 by at most (k dx)^2 (coefficient 1, the
// bound Test/Docs' C0a takes: 2 (2 pi / N)^2 for a wave along two axes, one here). Halving dx while keeping nu and
// quartering nu k^2's time scale (4x the steps) must cut the error by 4: an observed order of 2, asked here at 1.
TEST(LbmFlow, AShearWaveDecaysAtNuKSquaredToSecondOrderAndTheSameOnAMovingLattice) {
    const float nu = 0.01f;
    const auto rate_error = [&](float u0, uint32_t n, int steps) {
        const double k = kTwoPi / n;
        return shear_decay_rate(u0, nu, 0.0f, steps, n) / (static_cast<double>(nu) * k * k) - 1.0;
    };
    const double e32 = rate_error(0.0f, 32, 300);
    const double e64 = rate_error(0.0f, 64, 1200);
    const double k32 = kTwoPi / 32.0;
    EXPECT_LE(std::abs(e32), k32 * k32);
    EXPECT_LE(std::abs(e64), 0.5 * std::abs(e32)) << "e32 " << e32 << " e64 " << e64;
    const double moving = rate_error(0.2f, 32, 300);
    EXPECT_NEAR(moving, e32, 0.005) << "Galilean invariance to O(Ma^2): u0 0.2 is Ma 0.35";
}

TEST(LbmFlow, WaleAddsAnEddyViscosityOfAtMostALaminarWavesSize) {
    const float nu = 0.01f;
    const double off = shear_decay_rate(0.0f, nu, 0.0f, 300);
    const double on = shear_decay_rate(0.0f, nu, 0.5f, 300);
    EXPECT_NEAR(on / off, 1.0, 0.01) << "a smooth shear wave carries no WALE viscosity worth 1%";
}

TEST(LbmFlow, TheTaylorGreenStartConservesMassAndMomentum) {
    constexpr uint32_t n = 16;
    LbmFlow fl = make_flow(n, n, n, 0.002f, 0.5f);
    const double u0 = 0.1;
    for (uint32_t z = 0; z < n; ++z) {
        for (uint32_t y = 0; y < n; ++y) {
            for (uint32_t x = 0; x < n; ++x) {
                const double X = kTwoPi * x / n;
                const double Y = kTwoPi * y / n;
                const double Z = kTwoPi * z / n;
                const double p = u0 * u0 / 16.0 * (std::cos(2 * X) + std::cos(2 * Y)) * (std::cos(2 * Z) + 2.0);
                set_node_equilibrium(fl, x, y, z, static_cast<float>(1.0 + 3.0 * p),
                                     static_cast<float>(u0 * std::sin(X) * std::cos(Y) * std::cos(Z)),
                                     static_cast<float>(-u0 * std::cos(X) * std::sin(Y) * std::cos(Z)), 0.0f);
            }
        }
    }
    const FlowSums before = flow_sums(fl);
    for (int t = 0; t < 60; ++t) spade::testing::lbm_support::step_ok(fl);
    const FlowSums after = flow_sums(fl);
    EXPECT_TRUE(after.finite);
    EXPECT_NEAR(after.mass_m1, before.mass_m1, 1.0e-7);
    for (int a = 0; a < 3; ++a) EXPECT_NEAR(after.momentum[a], before.momentum[a], 1.0e-7) << a;
    EXPECT_LT(after.kinetic, before.kinetic) << "dissipating";
}

// A closed box (walls on every face, none moving) holds its mass: halfway bounce-back returns every population that
// reaches a wall. Momentum is not conserved (the walls take it), and the kinetic energy only falls.
TEST(LbmWalls, AClosedBoxConservesMassAndLosesEnergy) {
    constexpr uint32_t n = 8;
    LbmFlow fl = make_flow(n, n, n, 0.02f, 0.5f);
    fl.walls.periodic = {false, false, false};
    uint32_t seed = 7u;
    const auto rnd = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>(seed >> 8) / 16777216.0f - 0.5f;
    };
    for (uint32_t z = 0; z < n; ++z) {
        for (uint32_t y = 0; y < n; ++y) {
            for (uint32_t x = 0; x < n; ++x) {
                set_node_equilibrium(fl, x, y, z, 1.0f + 0.01f * rnd(), 0.05f * rnd(), 0.05f * rnd(), 0.05f * rnd());
            }
        }
    }
    const FlowSums before = flow_sums(fl);
    for (int t = 0; t < 200; ++t) spade::testing::lbm_support::step_ok(fl);
    const FlowSums after = flow_sums(fl);
    EXPECT_TRUE(after.finite);
    EXPECT_NEAR(after.mass_m1, before.mass_m1, 5.0e-7);
    EXPECT_LT(after.kinetic, before.kinetic);
}

// Couette flow between a still floor and a lid moving at U: halfway bounce-back puts the walls half a cell past the
// first and last nodes, so the steady profile is exactly u_j = U (j + 1/2) / N. This also reads the moving-wall term's
// sign: a wrong sign drives the flow against the lid.
TEST(LbmWalls, CouetteFlowBetweenAStillFloorAndAMovingLidIsLinear) {
    constexpr uint32_t n = 16;
    const float lid = 0.05f;
    LbmFlow fl = make_flow(4, n, 4, 0.1f, 0.0f);
    fl.walls.periodic = {true, false, true};
    fl.walls.wall_u[1][1] = {lid, 0.0f, 0.0f};  // the high-y face is the lid
    for (uint32_t z = 0; z < 4; ++z) {
        for (uint32_t y = 0; y < n; ++y) {
            for (uint32_t x = 0; x < 4; ++x) set_node_equilibrium(fl, x, y, z, 1.0f, 0.0f, 0.0f, 0.0f);
        }
    }
    for (int t = 0; t < 4000; ++t) spade::testing::lbm_support::step_ok(fl);
    compute_velocity(fl);
    for (uint32_t y = 0; y < n; ++y) {
        const double want = static_cast<double>(lid) * (y + 0.5) / n;
        EXPECT_NEAR(fl.u_at(0, fl.grid.cell(1, y, 2)), want, 0.01 * lid) << "row " << y;
        EXPECT_NEAR(fl.u_at(1, fl.grid.cell(1, y, 2)), 0.0, 1.0e-5) << "row " << y;
    }
}

// One map point. SPADE_LBM_MAP="<u_lat>:<nu_lat>", e.g. "0.25:1e-6".
TEST(LbmMap, OnePointOfTheStabilityMap) {
    const char* spec = env_text("SPADE_LBM_MAP");
    if (env_text("SPADE_LBM_VALIDATION") == nullptr || spec == nullptr) {
        GTEST_SKIP() << "set SPADE_LBM_VALIDATION=1 and SPADE_LBM_MAP=<u_lat>:<nu_lat> to run a map point";
    }
    char* colon = nullptr;
    const double u_lat = std::strtod(spec, &colon);
    ASSERT_TRUE(colon != spec && *colon == ':') << spec;
    const double nu_lat = std::strtod(colon + 1, nullptr);
    constexpr uint32_t n = 64;
    const double l_lat = n / kTwoPi;
    const int horizon = static_cast<int>(std::lround(20.0 * l_lat / u_lat));
    LbmFlow fl = spade::testing::lbm_support::taylor_green_field(n, u_lat, nu_lat);
    namespace support = spade::testing::lbm_support;
    const double k0 = support::case_sums(fl).kinetic;
    int held = 0;
    double last_ratio = 1.0;
    bool alive = true;
    const auto t_start = std::chrono::steady_clock::now();
    for (int t = 1; t <= horizon && alive; ++t) {
        support::case_step(fl);
        if (t % 5 == 0) {
            const FlowSums s = support::case_sums(fl);
            if (!s.finite || !(s.kinetic <= 1.05 * k0)) {
                alive = false;
            } else {
                last_ratio = s.kinetic / k0;
                held = t;
            }
        } else {
            held = t;
        }
    }
    if (!alive) held = (held / 5) * 5;  // the last sample that held
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    const double mlups = static_cast<double>(fl.grid.step) * static_cast<double>(n) * n * n / secs / 1.0e6;
    const float nu_f = static_cast<float>(nu_lat);
    std::printf("MAP u_lat %.3f (Ma %.3f) nu_lat %g (tau-1/2 %.9g): held %d of %d, K*/K*(0) at last sample %.4f, "
                "%u thread(s) %.3f MLUPS\n",
                u_lat, u_lat * std::sqrt(3.0), nu_lat, 3.0 * static_cast<double>(nu_f), held, horizon, last_ratio,
                support::case_threads(), mlups);
    support::print_digest("MAP", fl);
    SUCCEED();
}

// ---- the thread split -------------------------------------------------------------------------------------------

namespace airflow = spade::physics::airflow;

// The field the split is tested on (tests/lbm_support.hpp's box_field: shared with the GPU twin's bit tests).
LbmFlow thread_field(bool lid) { return spade::testing::lbm_support::box_field(lid); }

bool same_bits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

// Twelve steps (both parities, several times over), then the populations, u and the diagnostics against the serial run.
void expect_pool_matches_serial(bool lid, airflow::WorkerPool& pool, const char* what) {
    LbmFlow serial = thread_field(lid);
    LbmFlow split = thread_field(lid);
    const std::vector<float> start = serial.grid.f;
    for (int t = 0; t < 12; ++t) {
        spade::testing::lbm_support::step_ok(serial);
        spade::testing::lbm_support::step_ok(split, &pool);
    }
    EXPECT_FALSE(same_bits(serial.grid.f, start)) << "the field moved, so the comparison means something";
    EXPECT_TRUE(same_bits(split.grid.f, serial.grid.f)) << what << ": populations";
    const FlowSums a = flow_sums(serial);
    const FlowSums b = flow_sums(split, &pool);
    EXPECT_TRUE(same_bits(split.u, serial.u)) << what << ": u";
    EXPECT_TRUE(spade::testing::lbm_support::same_sums(a, b)) << what << ": diagnostics";
    EXPECT_EQ(split.grid.step, serial.grid.step);
}

TEST(LbmThreads, ThePeriodicStepIsTheSameBitsOnAnyThreadCountAndInAnyChunkOrder) {
    airflow::WorkerPool one(1);
    airflow::WorkerPool two(2);
    airflow::WorkerPool four(4);
    airflow::WorkerPool reversed(3, airflow::ItemOrder::reversed);
    expect_pool_matches_serial(false, one, "1 thread");
    expect_pool_matches_serial(false, two, "2 threads");
    expect_pool_matches_serial(false, four, "4 threads");
    expect_pool_matches_serial(false, reversed, "chunks last to first");
}

TEST(LbmThreads, TheBoxWithAMovingLidIsTheSameBitsOnAnyThreadCountAndInAnyChunkOrder) {
    airflow::WorkerPool two(2);
    airflow::WorkerPool four(4);
    airflow::WorkerPool reversed(3, airflow::ItemOrder::reversed);
    expect_pool_matches_serial(true, two, "2 threads");
    expect_pool_matches_serial(true, four, "4 threads");
    expect_pool_matches_serial(true, reversed, "chunks last to first");
}

// ---- the flush scope ---------------------------------------------------------------------------------------------

LbmFlow subnormal_field(bool lid) {
    LbmFlow fl = make_flow(6, 6, 6, 0.004f, 0.5f);
    if (lid) {
        fl.walls.periodic[1] = false;
        fl.walls.wall_u[1][1] = {0.0f, 0.0f, 0.0f};  // a wall at rest: the field stays quiet
    }
    fl.grid.f.assign(fl.grid.f.size(), 1.0e-40f);  // below 2^-126 = 1.18e-38: every population is subnormal
    return fl;
}

// Under FTZ|DAZ a subnormal is read as zero, so a field of subnormals is a quiet field and stays exactly zero (the
// quiet-box test's claim); outside the scope the step keeps the subnormals and moves them.
TEST(LbmFlush, AFieldOfSubnormalPopulationsIsAQuietFieldUnderTheScopeAndNotWithout) {
    for (const bool lid : {false, true}) {
        LbmFlow gradual = subnormal_field(lid);
        spade::testing::lbm_support::step_ok(gradual);
        bool moved = false;
        for (const float v : gradual.grid.f) moved = moved || v != 0.0f;
        EXPECT_TRUE(moved) << "the unscoped step keeps subnormals, so this test can tell the scope from its absence";

        LbmFlow flushed = subnormal_field(lid);
        for (int t = 0; t < 3; ++t) spade::testing::lbm_support::step_ok_flushed(flushed);
        for (const float v : flushed.grid.f) EXPECT_EQ(v, 0.0f);
        const FlowSums sums = flow_sums_flushed(flushed);
        EXPECT_EQ(sums.kinetic, 0.0);
        EXPECT_TRUE(sums.finite);
    }
}

TEST(LbmFlush, TheSplitStepFlushesOnEveryThreadAndTheCallersWordComesBack) {
    airflow::WorkerPool four(4);
    airflow::WorkerPool reversed(3, airflow::ItemOrder::reversed);
    for (airflow::WorkerPool* pool : {&four, &reversed}) {
        LbmFlow fl = subnormal_field(false);
        const unsigned int before = _mm_getcsr();
        spade::testing::lbm_support::step_ok_flushed(fl, pool);
        spade::testing::lbm_support::step_ok_flushed(fl, pool);
        EXPECT_EQ(_mm_getcsr(), before) << "the scope restores the caller's MXCSR whole";
        for (const float v : fl.grid.f) {
            EXPECT_EQ(v, 0.0f) << "a worker that ran without the caller's flush leaves subnormals";
        }
    }
}

// ---- LbmWalls: a halfway bounce-back wall moves along itself, never into or out of the box ----------------

TEST(LbmWalls, ANormalVelocityComponentIsRefusedByNameOnEveryWall) {
    namespace support = spade::testing::lbm_support;
    const char* const axis[3] = {"x", "y", "z"};
    const char* const side[2] = {"low", "high"};
    for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t sd = 0; sd < 2; ++sd) {
            LbmFlow fl = support::box_field(false);
            fl.walls.periodic[a] = false;
            fl.walls.wall_u[a][sd][a] = 0.05f;
            const std::vector<float> before = fl.grid.f;
            for (const bool flushed : {false, true}) {
                const spade::Result<void> r = flushed ? step_flow_flushed(fl) : step_flow(fl);
                ASSERT_FALSE(r.has_value()) << axis[a] << "-" << side[sd] << (flushed ? " (flushed)" : "");
                EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
                const std::string& why = r.error().context;
                EXPECT_NE(why.find(std::string(axis[a]) + "-" + side[sd] + " wall"), std::string::npos) << why;
                EXPECT_NE(why.find(std::string("component ") + axis[a]), std::string::npos) << why;
                EXPECT_NE(why.find("along its own normal"), std::string::npos) << why;
                EXPECT_NE(why.find("mass source or sink"), std::string::npos) << why;
                EXPECT_EQ(fl.grid.step, uint64_t{0}) << "a refused step takes no step";
                EXPECT_TRUE(fl.grid.f == before) << "a refused step touches nothing";
            }
        }
    }
}

TEST(LbmWalls, AWallThatMovesAlongItselfIsAccepted) {
    namespace support = spade::testing::lbm_support;
    LbmFlow lid = support::box_field(true);  // the high-y face moves along x
    EXPECT_TRUE(step_flow(lid).has_value());
    LbmFlow six = support::six_wall_field(0.01f);  // a different tangential vector on each face
    EXPECT_TRUE(step_flow_flushed(six).has_value());
    EXPECT_TRUE(check_walls(six.walls).has_value());
}

TEST(LbmWalls, ANormalComponentOnAPeriodicAxisIsNotAWall) {
    LbmFlow fl = spade::testing::lbm_support::box_field(false);  // all periodic: wall_u is never read
    fl.walls.wall_u[0][0][0] = 0.5f;
    fl.walls.wall_u[1][1][1] = 0.5f;
    EXPECT_TRUE(step_flow(fl).has_value());
}

TEST(LbmWalls, ANonFiniteWallVelocityIsRefused) {
    for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()}) {
        LbmFlow fl = spade::testing::lbm_support::box_field(true);
        fl.walls.wall_u[1][1][0] = bad;  // the lid's tangential component
        const spade::Result<void> r = step_flow(fl);
        ASSERT_FALSE(r.has_value());
        EXPECT_NE(r.error().context.find("is not finite"), std::string::npos) << r.error().context;
    }
}

TEST(LbmWalls, AClosedBoxKeepsItsMass) {
    namespace support = spade::testing::lbm_support;
    for (const char* what : {"lid box", "six-wall box"}) {
        LbmFlow fl = std::string(what) == "lid box" ? support::box_field(true) : support::six_wall_field(0.01f);
        for (int t = 1; t <= 400; ++t) {
            support::step_ok_flushed(fl);
            if (t % 100 == 0) support::expect_mass_steady(fl, what);
        }
    }
}

// Why the bound exists: the box this file used to test with had a normal component on every wall, and its mean
// density fell by 9.4e-3 a step to NaN at step 28. Stepped unchecked it still does, and the mass bound sees it at once.
TEST(LbmWalls, TheMassBoundCatchesTheBoxWithNormalWallVelocities) {
    namespace support = spade::testing::lbm_support;
    LbmFlow fl = support::box_field(true);
    for (std::size_t a = 0; a < 3; ++a) {
        fl.walls.periodic[a] = false;
        for (std::size_t sd = 0; sd < 2; ++sd) {
            for (std::size_t k = 0; k < 3; ++k) {
                fl.walls.wall_u[a][sd][k] = 0.01f * static_cast<float>(1 + a * 6 + sd * 3 + k) - 0.05f;
            }
        }
    }
    EXPECT_FALSE(check_walls(fl.walls).has_value());
    for (int t = 0; t < 10; ++t) step_flow_unchecked(fl);
    const FlowSums s = flow_sums(fl);
    EXPECT_GT(std::abs(s.mass_m1), 1.0e3 * support::kMassBound) << s.mass_m1;
}

}  // namespace
