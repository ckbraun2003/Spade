// test_lbm_cases.cpp -- L1's validation cases C0a, C0b and C1 on the LBM tier
// (bands and references: docs/design/test-docs/2026-11-03-l1-lbm-tolerances.md).
// One env-gated test runs one case and prints the record's measures:
//
//   SPADE_LBM_VALIDATION=1 SPADE_LBM_CASE=c0a:<N>:<u_lat>   the Taylor-Green mode's decay (N x N x 1, nu 0.01)
//   SPADE_LBM_VALIDATION=1 SPADE_LBM_CASE=c0b:<N>           the vortex at Re 1600 (N^3, u_lat 0.1, WALE on)
//   SPADE_LBM_VALIDATION=1 SPADE_LBM_CASE=c1:<Re>:<u_lat>   the cavity against Ghia (128 x 128 x 1, WALE on)
//
// The measures are printed, not asserted: the verdict against the fixed band is
// the record's, written beside the commit. Every line carries MLUPS (one CPU
// thread).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "data/airflow/ghia1982_cavity.hpp"
#include "data/airflow/tgv1600_spectral.hpp"
#include "lbm_support.hpp"
#include "physics/lbm/lbm_flow.hpp"

namespace {

using namespace spade::physics::lbm;
namespace ref = spade::testing::airflow_reference;
using namespace spade::testing::lbm_support;

constexpr double kTwoPi = 2.0 * std::numbers::pi;

using Clock = std::chrono::steady_clock;

double mlups_of(const LbmFlow& fl, Clock::time_point since) {
    const double secs = std::chrono::duration<double>(Clock::now() - since).count();
    return static_cast<double>(fl.grid.step) * static_cast<double>(fl.grid.cells()) / secs / 1.0e6;
}

// C0a: the two-dimensional mode in a box of side 2 pi L (L = N / 2 pi lattice units).
void run_c0a(uint32_t n, double u_lat) {
    const double nu = 0.01;
    const double l_lat = n / kTwoPi;
    LbmFlow fl = make_flow(n, n, 1, static_cast<float>(nu), 0.0f);
    for (uint32_t y = 0; y < n; ++y) {
        for (uint32_t x = 0; x < n; ++x) {
            const double X = kTwoPi * x / n;
            const double Y = kTwoPi * y / n;
            const double p = 0.25 * u_lat * u_lat * (std::cos(2 * X) + std::cos(2 * Y));
            set_node_equilibrium(fl, x, y, 0, static_cast<float>(1.0 + 3.0 * p),
                                 static_cast<float>(u_lat * std::sin(X) * std::cos(Y)),
                                 static_cast<float>(-u_lat * std::cos(X) * std::sin(Y)), 0.0f);
        }
    }
    const int steps = static_cast<int>(std::lround(l_lat * l_lat / (4.0 * nu)));
    const double k0 = case_sums(fl).kinetic;
    const auto t0 = Clock::now();
    for (int t = 0; t < steps; ++t) case_step(fl);
    const double k1 = case_sums(fl).kinetic;
    const double nu_eff = -l_lat * l_lat * std::log(k1 / k0) / (4.0 * steps);
    std::printf("C0A N %u u_lat %.3f (Ma %.3f) nu_lat %g: steps %d, nu_eff/nu - 1 = %+.5f, %u thread(s) %.3f MLUPS\n",
                n, u_lat, u_lat * std::sqrt(3.0), nu, steps, nu_eff / nu - 1.0, case_threads(), mlups_of(fl, t0));
    print_digest("C0A", fl);
}

// C0b: the three-dimensional vortex at Re 1600, WALE on at C_w 0.5.
void run_c0b(uint32_t n) {
    const double u_lat = 0.1;
    const double l_lat = n / kTwoPi;
    const double nu = u_lat * l_lat / 1600.0;
    LbmFlow fl = make_flow(n, n, n, static_cast<float>(nu), 0.5f);
    for (uint32_t z = 0; z < n; ++z) {
        for (uint32_t y = 0; y < n; ++y) {
            for (uint32_t x = 0; x < n; ++x) {
                const double X = kTwoPi * x / n;
                const double Y = kTwoPi * y / n;
                const double Z = kTwoPi * z / n;
                const double p = u_lat * u_lat / 16.0 * (std::cos(2 * X) + std::cos(2 * Y)) * (std::cos(2 * Z) + 2.0);
                set_node_equilibrium(fl, x, y, z, static_cast<float>(1.0 + 3.0 * p),
                                     static_cast<float>(u_lat * std::sin(X) * std::cos(Y) * std::cos(Z)),
                                     static_cast<float>(-u_lat * std::cos(X) * std::sin(Y) * std::cos(Z)), 0.0f);
            }
        }
    }
    const int steps = static_cast<int>(std::lround(20.0 * l_lat / u_lat));
    const int every = static_cast<int>(std::floor(0.05 * l_lat / u_lat));  // sample at most 0.05 t* apart
    const double dt_star = every * u_lat / l_lat;
    std::vector<double> kstar;  // K* at steps 0, every, 2 every, ...
    kstar.push_back(case_sums(fl).kinetic / (u_lat * u_lat));
    const auto t0 = Clock::now();
    for (int t = 1; t <= steps; ++t) {
        case_step(fl);
        if (t % every == 0) kstar.push_back(case_sums(fl).kinetic / (u_lat * u_lat));
    }
    const double mlups = mlups_of(fl, t0);
    // eps* by central differences of the samples; the peak is the largest, unsmoothed.
    std::vector<double> eps(kstar.size(), 0.0);
    double peak = 0.0;
    double peak_t = 0.0;
    for (std::size_t s = 1; s + 1 < kstar.size(); ++s) {
        eps[s] = -(kstar[s + 1] - kstar[s - 1]) / (2.0 * dt_star);
        if (eps[s] > peak) {
            peak = eps[s];
            peak_t = static_cast<double>(s) * dt_star;
        }
    }
    double dns_peak = 0.0;
    double dns_peak_t = 0.0;
    for (int i = 0; i < ref::kTgv1600Samples; ++i) {
        if (ref::kTgv1600Dissipation[i] > dns_peak) {
            dns_peak = ref::kTgv1600Dissipation[i];
            dns_peak_t = i * ref::kTgv1600Dt;
        }
    }
    std::printf("C0B N %u nu_lat %.4g steps %d, sampled every %d steps (dt* %.4f): peak eps* %.6f at t* %.3f; "
                "DNS %.6f at t* %.2f; peak ratio %.4f, t ratio %.4f, %u thread(s) %.3f MLUPS\n",
                n, nu, steps, every, dt_star, peak, peak_t, dns_peak, dns_peak_t, peak / dns_peak, peak_t / dns_peak_t,
                case_threads(), mlups);
    print_digest("C0B", fl);
    for (int ts = 1; ts <= 19; ++ts) {
        const std::size_t s = static_cast<std::size_t>(std::lround(ts / dt_star));
        if (s + 1 >= kstar.size()) break;
        std::printf("C0B series t* %2d: eps* %.6f, DNS %.6f\n", ts, eps[s],
                    ref::kTgv1600Dissipation[static_cast<std::size_t>(ts / ref::kTgv1600Dt + 0.5)]);
    }
}

// Piecewise-linear value at `at` over points (t_k, v_k): the wall's value at 0 and 1, the nodes' between.
double sample_line(const std::vector<double>& node, double wall_lo, double wall_hi, double at) {
    const double n = static_cast<double>(node.size());
    const double s = at * n - 0.5;  // node index coordinate
    if (s <= -0.5 + 1e-12) return wall_lo;
    if (s >= n - 0.5 - 1e-12) return wall_hi;
    if (s < 0.0) return wall_lo + (node.front() - wall_lo) * (s + 0.5) / 0.5;
    if (s > n - 1.0) return node.back() + (wall_hi - node.back()) * (s - (n - 1.0)) / 0.5;
    const auto j = static_cast<std::size_t>(s);
    const double f = s - static_cast<double>(j);
    return node[j] + f * (node[std::min(j + 1, node.size() - 1)] - node[j]);
}

// C1: the cavity. u along x = 0.5 (the mean of the two middle columns) and v along y = 0.5.
std::vector<double> cavity_samples(LbmFlow& fl, double u_lat, bool skip_point_117) {
    compute_velocity(fl);
    const uint32_t n = fl.grid.nx;
    std::vector<double> col_u(n);
    std::vector<double> row_v(n);
    for (uint32_t j = 0; j < n; ++j) {
        col_u[j] = 0.5 * (fl.u_at(0, fl.grid.cell(n / 2 - 1, j, 0)) + fl.u_at(0, fl.grid.cell(n / 2, j, 0)));
        row_v[j] = 0.5 * (fl.u_at(1, fl.grid.cell(j, n / 2 - 1, 0)) + fl.u_at(1, fl.grid.cell(j, n / 2, 0)));
    }
    std::vector<double> out;
    for (int q = 0; q < 17; ++q) out.push_back(sample_line(col_u, 0.0, u_lat, ref::kGhiaY[q]) / u_lat);
    for (int q = 0; q < 17; ++q) {
        if (skip_point_117 && q == ref::kGhiaTableIIPoint117) continue;
        out.push_back(sample_line(row_v, 0.0, 0.0, ref::kGhiaX[q]) / u_lat);
    }
    return out;
}

void run_c1(int re, double u_lat) {
    constexpr uint32_t n = 128;
    const double nu = u_lat * n / re;
    LbmFlow fl = make_flow(n, n, 1, static_cast<float>(nu), 0.5f);
    fl.walls.periodic = {false, false, true};
    fl.walls.wall_u[1][1] = {static_cast<float>(u_lat), 0.0f, 0.0f};
    for (uint32_t y = 0; y < n; ++y) {
        for (uint32_t x = 0; x < n; ++x) set_node_equilibrium(fl, x, y, 0, 1.0f, 0.0f, 0.0f, 0.0f);
    }
    const double t_lu = re == 100 ? 50.0 : 190.0;  // T in L/U
    const int per_lu = static_cast<int>(std::lround(n / u_lat));
    const int steps = static_cast<int>(t_lu) * per_lu;
    const int every = static_cast<int>(std::lround(0.05 * n / u_lat));
    const bool skip = re == 400;
    std::vector<std::vector<double>> window;  // samples over the last 1.0 L/U
    const auto t0 = Clock::now();
    for (int t = 1; t <= steps; ++t) {
        case_step(fl);
        if (t % every == 0 && t > steps - per_lu) window.push_back(cavity_samples(fl, u_lat, skip));
    }
    const double mlups = mlups_of(fl, t0);
    const std::vector<double> last = cavity_samples(fl, u_lat, skip);
    const std::size_t nu_count = 17;
    double eu = 0.0;
    double ev = 0.0;
    for (std::size_t q = 0; q < nu_count; ++q) {
        eu = std::max(eu, std::abs(last[q] - (re == 100 ? ref::kGhiaU100[q] : ref::kGhiaU400[q])));
    }
    std::size_t at = nu_count;
    for (int q = 0; q < 17; ++q) {
        if (skip && q == ref::kGhiaTableIIPoint117) continue;
        ev = std::max(ev, std::abs(last[at++] - (re == 100 ? ref::kGhiaV100[q] : ref::kGhiaV400[q])));
    }
    double steady = 0.0;
    for (const auto& w : window) {
        for (std::size_t i = 0; i < w.size(); ++i) steady = std::max(steady, std::abs(w[i] - last[i]));
    }
    double nut_max = 0.0;
    for (uint32_t y = 0; y < n; ++y) {
        for (uint32_t x = 0; x < n; ++x) {
            const float nt = wale_nu_t(velocity_gradient(fl, x, y, 0), 0.5f);
            nut_max = std::max(nut_max, static_cast<double>(nt));
        }
    }
    std::printf("C1 Re %d u_lat %.2f nu_lat %.4g steps %d: max|u-uG| %.5f, max|v-vG| %.5f, steadiness %.5f over %zu "
                "samples, peak nu_t/nu %.5f, %u thread(s) %.3f MLUPS\n",
                re, u_lat, nu, steps, eu, ev, steady, window.size(), nut_max / nu, case_threads(), mlups);
    print_digest("C1", fl);
}

TEST(LbmCase, OneValidationCase) {
    const char* spec = env_text("SPADE_LBM_CASE");
    if (env_text("SPADE_LBM_VALIDATION") == nullptr || spec == nullptr) {
        GTEST_SKIP() << "set SPADE_LBM_VALIDATION=1 and SPADE_LBM_CASE=c0a:<N>:<u_lat> | c0b:<N> | c1:<Re>:<u_lat>";
    }
    const std::string s = spec;
    const auto field = [&s](std::size_t index) {
        std::size_t at = 0;
        for (std::size_t k = 0; k < index; ++k) at = s.find(':', at) + 1;
        return std::strtod(s.c_str() + at, nullptr);
    };
    if (s.rfind("c0a:", 0) == 0) {
        run_c0a(static_cast<uint32_t>(field(1)), field(2));
    } else if (s.rfind("c0b:", 0) == 0) {
        run_c0b(static_cast<uint32_t>(field(1)));
    } else if (s.rfind("c1:", 0) == 0) {
        run_c1(static_cast<int>(field(1)), field(2));
    } else {
        FAIL() << "unknown case " << s;
    }
}

}  // namespace
