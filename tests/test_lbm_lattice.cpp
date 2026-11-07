// test_lbm_lattice.cpp -- L1's lattice core: the D3Q27 tables and the in-place
// (esoteric pull) streaming, on a fully periodic grid (design: docs/design/
// physics/plans/2026-11-04-l1-lbm-design.md, sections 2 and 6).
//
//   LbmLattice   the weights and velocities: their moments to fourth order,
//                derived in double; the opposite table.
//   LbmStreaming one population travels along its own direction, in all 26
//                directions and for both parities; the in-place scheme equals
//                a plain two-copy streaming, bit for bit, through a nonlinear
//                collision, so a wrong slot shows.
//
// Mutations (each fails a test below): a swapped read and write slot in the
// odd step (the travel test and the two-copy test); the opposite table off by
// one (the opposite test).

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <vector>

#include "physics/lbm/lattice.hpp"
#include "physics/lbm/lbm_grid.hpp"
#include "physics/lbm/lbm_step.hpp"

namespace {

using namespace spade::physics::lbm;

double c_of(uint32_t i, int axis) {
    const Vel c = kC[i];
    return axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
}

TEST(LbmLattice, TheWeightsSumToOneAndCarryNoMomentum) {
    double sum = 0.0;
    double mom[3] = {0, 0, 0};
    for (uint32_t i = 0; i < kQ; ++i) {
        sum += static_cast<double>(kW[i]);
        for (int a = 0; a < 3; ++a) mom[a] += static_cast<double>(kW[i]) * c_of(i, a);
    }
    EXPECT_NEAR(sum, 1.0, 1.0e-7);
    for (int a = 0; a < 3; ++a) EXPECT_EQ(mom[a], 0.0) << a;
}

// The moments the collision's equilibrium and regularization rely on: second
// order c_s^2 I, third order zero, fourth order c_s^4 (d d + d d + d d), with
// c_s^2 = 1/3. On D3Q27 the fourth-order tensor is isotropic, xxxx included
// (sum w c^4 = sum w c^2 = 1/3 = 3 c_s^4).
TEST(LbmLattice, TheMomentsAreIsotropicToFourthOrder) {
    const double cs2 = 1.0 / 3.0;
    const auto d = [](int a, int b) { return a == b ? 1.0 : 0.0; };
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            double m2 = 0.0;
            for (uint32_t i = 0; i < kQ; ++i) m2 += static_cast<double>(kW[i]) * c_of(i, a) * c_of(i, b);
            EXPECT_NEAR(m2, cs2 * d(a, b), 1.0e-7) << a << b;
            for (int c = 0; c < 3; ++c) {
                double m3 = 0.0;
                for (uint32_t i = 0; i < kQ; ++i) {
                    m3 += static_cast<double>(kW[i]) * c_of(i, a) * c_of(i, b) * c_of(i, c);
                }
                EXPECT_NEAR(m3, 0.0, 1.0e-7) << a << b << c;
                for (int e = 0; e < 3; ++e) {
                    double m4 = 0.0;
                    for (uint32_t i = 0; i < kQ; ++i) {
                        m4 += static_cast<double>(kW[i]) * c_of(i, a) * c_of(i, b) * c_of(i, c) * c_of(i, e);
                    }
                    const double want = cs2 * cs2 * (d(a, b) * d(c, e) + d(a, c) * d(b, e) + d(a, e) * d(b, c));
                    EXPECT_NEAR(m4, want, 1.0e-7) << a << b << c << e;
                }
            }
        }
    }
}

// Hermite's third-order polynomial vanishes on the diagonal, which is why the
// collision's third-order terms hold only the 7 off-diagonal components.
TEST(LbmLattice, TheThirdOrderHermiteDiagonalVanishesOnTheLattice) {
    for (uint32_t i = 0; i < kQ; ++i) {
        for (int a = 0; a < 3; ++a) {
            const double c = c_of(i, a);
            EXPECT_EQ(c * c * c - 3.0 * (1.0 / 3.0) * c, 0.0) << i << a;
        }
    }
}

TEST(LbmLattice, EveryDirectionHasItsOppositeAndTheyArePairedNextToEachOther) {
    EXPECT_EQ(opposite(0), 0u);
    for (uint32_t i = 1; i < kQ; ++i) {
        const uint32_t o = opposite(i);
        EXPECT_NE(o, i);
        EXPECT_EQ(opposite(o), i);
        EXPECT_EQ(kC[o].x, -kC[i].x) << i;
        EXPECT_EQ(kC[o].y, -kC[i].y) << i;
        EXPECT_EQ(kC[o].z, -kC[i].z) << i;
        EXPECT_EQ(o > i ? o - i : i - o, 1u) << "the adjacent pairs of the design";
        EXPECT_EQ(kW[o], kW[i]);
    }
}

// One population, shifted storage with a distinct value, in a box that is not
// a cube; the collision does nothing. It must travel one cell a step along its
// own direction, through every wrap, for both parities, and nothing else may
// hold anything.
TEST(LbmStreaming, OnePopulationTravelsAlongItsDirectionInAllTwentySixDirections) {
    for (uint32_t dir = 1; dir < kQ; ++dir) {
        LbmGrid g = make_lbm_grid(5, 4, 3);
        const uint32_t x0 = 1;
        const uint32_t y0 = 2;
        const uint32_t z0 = 0;
        g.f[slot_of(g, dir, g.cell(x0, y0, z0))] = 1.0f;  // step 0 is even: incoming is the slot itself
        const Vel c = kC[dir];
        for (uint32_t t = 0; t < 14; ++t) {
            const auto ex = static_cast<uint32_t>(((static_cast<int>(x0) + c.x * static_cast<int>(t)) % 5 + 5) % 5);
            const auto ey = static_cast<uint32_t>(((static_cast<int>(y0) + c.y * static_cast<int>(t)) % 4 + 4) % 4);
            const auto ez = static_cast<uint32_t>(((static_cast<int>(z0) + c.z * static_cast<int>(t)) % 3 + 3) % 3);
            double total = 0.0;
            for (uint32_t k = 0; k < g.nz; ++k) {
                for (uint32_t j = 0; j < g.ny; ++j) {
                    for (uint32_t i = 0; i < g.nx; ++i) {
                        for (uint32_t d = 0; d < kQ; ++d) {
                            const float v = incoming(g, d, i, j, k);
                            total += static_cast<double>(v);
                            const bool here = d == dir && i == ex && j == ey && k == ez;
                            EXPECT_EQ(v, here ? 1.0f : 0.0f) << "dir " << dir << " step " << t << " d " << d;
                        }
                    }
                }
            }
            EXPECT_EQ(total, 1.0);
            step_periodic(g, [](std::array<float, kQ>&) {});
        }
    }
}

// A deterministic nonlinear collision of one node's 27 values (every output
// depends on all of them), so a population read from or written to the wrong
// slot changes the bits.
void mix(std::array<float, kQ>& f) {
    float s = 0.0f;
    for (uint32_t i = 0; i < kQ; ++i) s += f[i];
    const std::array<float, kQ> in = f;
    for (uint32_t i = 0; i < kQ; ++i) {
        f[i] = in[i] + 0.3f * (s * (1.0f / 27.0f) - in[i]) + 0.02f * in[i] * in[opposite(i)] +
               0.001f * static_cast<float>(i);
    }
}

struct Lcg {
    uint32_t state = 2463534242u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (1.0f / 16777216.0f) - 0.5f;
    }
};

// The plain scheme: two arrays of the incoming populations; each node collides
// its own incoming, and the result is pushed along c into the other array.
struct TwoCopy {
    uint32_t nx, ny, nz;
    std::vector<float> a;  // incoming populations, direction-major
    std::vector<float> b;
    [[nodiscard]] std::size_t cells() const { return std::size_t{nx} * ny * nz; }
    void step() {
        LbmGrid shape = make_lbm_grid(nx, ny, nz);
        for (uint32_t z = 0; z < nz; ++z) {
            for (uint32_t y = 0; y < ny; ++y) {
                for (uint32_t x = 0; x < nx; ++x) {
                    std::array<float, kQ> f{};
                    for (uint32_t i = 0; i < kQ; ++i) f[i] = a[i * cells() + shape.cell(x, y, z)];
                    mix(f);
                    for (uint32_t i = 0; i < kQ; ++i) b[i * cells() + neighbour_cell(shape, x, y, z, kC[i])] = f[i];
                }
            }
        }
        a.swap(b);
    }
};

TEST(LbmStreaming, TheInPlaceSchemeEqualsATwoCopyStreamingBitForBit) {
    constexpr uint32_t nx = 6;
    constexpr uint32_t ny = 5;
    constexpr uint32_t nz = 7;
    LbmGrid g = make_lbm_grid(nx, ny, nz);
    TwoCopy ref{nx, ny, nz, std::vector<float>(kQ * g.cells()), std::vector<float>(kQ * g.cells())};
    Lcg rng;
    for (float& v : g.f) v = rng.next();
    ref.a = g.f;  // step 0 is even: the slots hold the incoming populations
    for (uint32_t t = 0; t < 40; ++t) {
        for (uint32_t z = 0; z < nz; ++z) {
            for (uint32_t y = 0; y < ny; ++y) {
                for (uint32_t x = 0; x < nx; ++x) {
                    for (uint32_t i = 0; i < kQ; ++i) {
                        const float want = ref.a[i * g.cells() + g.cell(x, y, z)];
                        const float got = incoming(g, i, x, y, z);
                        ASSERT_EQ(std::bit_cast<uint32_t>(got), std::bit_cast<uint32_t>(want))
                            << "step " << t << " dir " << i << " node " << x << "," << y << "," << z;
                    }
                }
            }
        }
        step_periodic(g, mix);
        ref.step();
    }
}

}  // namespace
