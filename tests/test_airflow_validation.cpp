// V1 of the airflow spec's validation (§11.6): C1, C0 and C8 (airflow core
// plan, Task 11). Not golden-feeding: reference values are computed in double
// with std:: functions.
//
// THE RECORDS. Each case prints one record block in the shape Test/Docs fixed
// (docs/design/test-docs/01-verification.md): an opening
// "=== validation record: <case> ===" line, then one "key: value" per line --
// path, config, then for each measure in order: measure, reference,
// tolerance, verdict. A measure line is "measure: <quantity> = <value>", with
// a unit after the value where there is one. The verdict is computed from the
// same comparison the EXPECT uses, so the record cannot disagree with the
// test. Whoever pastes a block into tests/data/airflow/records/ adds
// "commit:" and "date:" after its opening line.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "data/airflow/ghia1982_cavity.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"

namespace {

using namespace spade::physics::airflow;
namespace ref = spade::testing::airflow_reference;

[[nodiscard]] bool validation_enabled() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_AIRFLOW_VALIDATION") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// "CPU reference", the compiler and the preset.
[[nodiscard]] std::string path_line() {
#if defined(NDEBUG)
    const char* preset = "release";
#else
    const char* preset = "debug";
#endif
#if defined(_MSC_VER)
    return "CPU reference, MSVC " + std::to_string(_MSC_VER) + ", " + preset;
#else
    return std::string("CPU reference, ") + __VERSION__ + ", " + preset;
#endif
}

[[nodiscard]] const char* verdict(bool within) { return within ? "within" : "MISS"; }

// u on the line x = 0.5 at height y. The u faces at i = n/2 sit at
// y_j = (j + 1/2) / n; below the first, u falls linearly to the floor's 0;
// above the last, it rises linearly to the lid's 1.
[[nodiscard]] double centreline_u(const MacGrid& g, double y) {
    const uint32_t n = g.shape.ny;
    const uint32_t i = g.shape.nx / 2u;
    const auto at = [&](uint32_t j) { return static_cast<double>(g.u[u_index(g.shape, i, j, 0)]); };
    const double pos = y * n - 0.5;
    if (pos <= 0.0) return at(0) * (y * n / 0.5);
    if (pos >= n - 1.0) return at(n - 1u) + (pos - (n - 1.0)) / 0.5 * (1.0 - at(n - 1u));
    const auto j0 = static_cast<uint32_t>(pos);
    return at(j0) + (pos - j0) * (at(j0 + 1u) - at(j0));
}

// v on the line y = 0.5 at x: v faces at j = n/2, x_i = (i + 1/2) / n; the
// side walls hold v = 0.
[[nodiscard]] double centreline_v(const MacGrid& g, double x) {
    const uint32_t n = g.shape.nx;
    const uint32_t j = g.shape.ny / 2u;
    const auto at = [&](uint32_t i) { return static_cast<double>(g.v[v_index(g.shape, i, j, 0)]); };
    const double pos = x * n - 0.5;
    if (pos <= 0.0) return at(0) * (x * n / 0.5);
    if (pos >= n - 1.0) return at(n - 1u) * (1.0 - (pos - (n - 1.0)) / 0.5);
    const auto i0 = static_cast<uint32_t>(pos);
    return at(i0) + (pos - i0) * (at(i0 + 1u) - at(i0));
}

struct CavityResult {
    double u_error = 0.0;
    double v_error = 0.0;
    bool u_within = false;  // the comparison the record's verdict and the EXPECT both use
    bool v_within = false;
};

// The larger of the running maximum and e; a NaN, once in, stays (std::max
// would drop it, and a faulted run's record would read "within").
void take_max(double& worst, double e) noexcept {
    if (std::isnan(e) || e > worst) worst = e;
}

// The cavity at `re` on an n x n slab to time t_end; the two measures against
// Ghia's tables, each checked against `tolerance`. With `skip_v_point_117`,
// Table II's point 117 is left out of the v maximum (Re 400: printed
// -0.23827, a suspected typo in the original; Task 10's note).
// Where each band comes from: 0.02 is the spec's (section 11.2); 0.06 is plan
// A Task 11's own gate band at 32^2 (Test/Docs' review, 2026-10-12).
constexpr const char* kSpecOrigin = "plan A Task 11, spec \xc2\xa7" "11.2";
constexpr const char* kGateOrigin = "plan A Task 11's gate band";

[[nodiscard]] CavityResult run_cavity(uint32_t n, double re, double t_end, const double* u_ref, const double* v_ref,
                                      double tolerance, const char* origin, bool skip_v_point_117) {
    DomainBc bc;
    bc.face = {FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::periodic, FaceBc::periodic};
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    MacGrid g = *make_grid({n, n, 1, 1.0f / static_cast<float>(n)}, bc);
    const FluidConfig cfg{.rho = 1.0f, .nu = static_cast<float>(1.0 / re), .h = 0.5f / static_cast<float>(n),
                          .n_v = 2, .visc_sweeps = 8};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const auto steps = static_cast<uint32_t>(t_end / static_cast<double>(cfg.h));
    for (uint32_t k = 0; k < steps; ++k) {
        const FluidHealth h = fluid_step(g, bc, cfg, s);
        EXPECT_EQ(h.fault, FluidFault::none) << describe(h, g.shape);
        if (h.fault != FluidFault::none) break;
    }
    CavityResult r;
    for (int q = 0; q < 17; ++q) {
        take_max(r.u_error, std::fabs(centreline_u(g, ref::kGhiaY[q]) - u_ref[q]));
        if (skip_v_point_117 && q == ref::kGhiaTableIIPoint117) continue;
        take_max(r.v_error, std::fabs(centreline_v(g, ref::kGhiaX[q]) - v_ref[q]));
    }
    r.u_within = r.u_error <= tolerance;  // false for a NaN
    r.v_within = r.v_error <= tolerance;
    const std::string v_quantity = skip_v_point_117
                                       ? "max |v - v_Ghia| on y = 0.5, Table II point 117 left out (printed -0.23827, a "
                                         "suspected typo in the original)"
                                       : "max |v - v_Ghia| on y = 0.5";
    std::printf("=== validation record: C1 lid-driven cavity, Re %.0f, %ux%u ===\n"
                "path: %s\n"
                "config: %ux%u (a quasi-2D slab, z periodic), Re %.0f, T %.0f (%u steps of h = %.6f), n_v 2, "
                "visc_sweeps 8, rho 1, U 1\n"
                "measure: max |u - u_Ghia| on x = 0.5 = %.4f\n"
                "reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)\n"
                "tolerance: <= %.2f (%s, fixed before the first run)\n"
                "verdict: %s\n"
                "measure: %s = %.4f\n"
                "reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)\n"
                "tolerance: <= %.2f (%s, fixed before the first run)\n"
                "verdict: %s\n",
                re, n, n, path_line().c_str(), n, n, re, t_end, steps, static_cast<double>(cfg.h), r.u_error,
                tolerance, origin, verdict(r.u_within), v_quantity.c_str(), r.v_error, tolerance, origin,
                verdict(r.v_within));
    return r;
}

TEST(AirflowValidation, C1CavityAtRe100On32CellsIsWithinTheGateBand) {
    const CavityResult r = run_cavity(32, 100.0, 20.0, ref::kGhiaU100, ref::kGhiaV100, 0.06, kGateOrigin, false);
    EXPECT_TRUE(r.u_within) << r.u_error;
    EXPECT_TRUE(r.v_within) << r.v_error;
}

TEST(AirflowValidation, C1CavityAtRe100On128Cells) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the 128x128 cavity at Re 100";
    const CavityResult r = run_cavity(128, 100.0, 40.0, ref::kGhiaU100, ref::kGhiaV100, 0.02, kSpecOrigin, false);
    EXPECT_TRUE(r.u_within) << r.u_error;
    EXPECT_TRUE(r.v_within) << r.v_error;
}

TEST(AirflowValidation, C1CavityAtRe400On128Cells) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the 128x128 cavity at Re 400";
    const CavityResult r = run_cavity(128, 400.0, 60.0, ref::kGhiaU400, ref::kGhiaV400, 0.02, kSpecOrigin, true);
    EXPECT_TRUE(r.u_within) << r.u_error;
    EXPECT_TRUE(r.v_within) << r.v_error;
}

// The kinetic energy per unit mass, from the faces, the periodic alias faces left out.
[[nodiscard]] double kinetic_energy(const MacGrid& g) {
    const GridShape& s = g.shape;
    double sum[3] = {0.0, 0.0, 0.0};
    for (uint32_t c = 0; c < 3; ++c) {
        const std::vector<float>& f = c == 0 ? g.u : c == 1 ? g.v : g.w;
        for (uint32_t k = 0; k < s.nz; ++k) {
            for (uint32_t j = 0; j < s.ny; ++j) {
                for (uint32_t i = 0; i < s.nx; ++i) {
                    const std::size_t idx =
                        c == 0 ? u_index(s, i, j, k) : c == 1 ? v_index(s, i, j, k) : w_index(s, i, j, k);
                    sum[c] += static_cast<double>(f[idx]) * f[idx];
                }
            }
        }
    }
    const double cells = static_cast<double>(s.nx) * s.ny * s.nz;
    return 0.5 * (sum[0] + sum[1] + sum[2]) / cells;
}

TEST(AirflowValidation, C0TaylorGreenDissipationIsRecorded) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the 64^3 Taylor-Green vortex";
    const uint32_t n = 64;
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid({n, n, n, static_cast<float>(2.0 * std::numbers::pi / n)}, bc);
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) {
        g.u[idx] = static_cast<float>(std::sin(double(x.x)) * std::cos(double(x.y)) * std::cos(double(x.z)));
    });
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) {
        g.v[idx] = static_cast<float>(-std::cos(double(x.x)) * std::sin(double(x.y)) * std::cos(double(x.z)));
    });
    const FluidConfig cfg{.rho = 1.0f, .nu = 0.0f, .h = 0.5f * g.shape.dx, .n_v = 2};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const double ke0 = kinetic_energy(g);
    std::printf("=== validation record: C0 Taylor-Green vortex, 64^3 ===\n"
                "path: %s\n"
                "config: 64^3 periodic 2 pi box, inviscid (the numerics' own dissipation), h = %.6f, n_v 2\n",
                path_line().c_str(), static_cast<double>(cfg.h));
    // Every 0.5 (plan A's C0 line), at the first step at or past each mark,
    // labelled with that step's actual time (Test/Docs' review, 2026-10-12).
    const auto report = [](double t, double mark, double ke) {
        std::printf("measure: kinetic energy per unit mass at t = %.3f (the first step at or past %.1f) = %.6f\n"
                    "reference: none (report only; DNS data enter at V3, spec \xc2\xa7" "11.6)\n"
                    "tolerance: none (report only)\n"
                    "verdict: report only\n",
                    t, mark, ke);
    };
    report(0.0, 0.0, ke0);
    double ke = ke0;
    double mark = 0.5;
    for (uint64_t step = 1; mark <= 10.0; ++step) {
        const FluidHealth h = fluid_step(g, bc, cfg, s);
        ASSERT_EQ(h.fault, FluidFault::none) << describe(h, g.shape);
        const double t = static_cast<double>(step) * static_cast<double>(cfg.h);
        if (t >= mark) {
            ke = kinetic_energy(g);
            ASSERT_TRUE(std::isfinite(ke));
            report(t, mark, ke);
            mark += 0.5;
        }
    }
    EXPECT_LT(ke, ke0);
}

// ---- C8, the actuator disc's far wake -----------------------------------------
// The far wake is chaotic past its horizon T_p (step 104 at the probe, under
// the banded-parity plan's rule), so one run is one trajectory, and a measure
// read from it moves with a last bit. On the axis 2R below, C8's first
// measure, 3 of 16 one-ulp-nudged runs missed the 10%: the disc's 48 points
// leave a hub within 2.1 cells of the axis unloaded (the inner ring at 0.408R,
// Peskin's reach 2 cells), and the axis sits in that hub's meandering wake.
// The measure is the mean downwash over the 80 v faces within 0.5R of the
// axis, 2R below (the end-goal checks' probe disc), over steps 385 to 768. The
// verdict is an ensemble's: eight runs that differ only by a nudge in this
// test, 1 ulp up on 110 of the disc's increments (B4's nudge,
// tests/test_gpu_airflow_actuator.cpp) from each run's own seed. Measured and
// signed off by Physics before this run:
// docs/design/test-docs/2026-10-22-c8-spread.md.
//
// Physics' note on the bias: the disc mean reads about 6% above momentum
// theory's uniform disc. spread_actuator's 48 points load the disc unevenly:
// an unloaded hub, and 1.34x the uniform thrust per area inside 0.5R.
// Annular momentum theory gives up to sqrt 1.34 = 1.16x there, and mixing
// reduces it. The bias belongs to the disc's point loading, not the solver.

constexpr uint32_t kC8Steps = 768;
constexpr std::size_t kC8Nudged = 110;
constexpr uint32_t kC8Threads = 8;

// The probes, all of the downwash -v: C8's four-face mean on the axis at R,
// 2R, 3R and 4R below; the mean over the faces within 0.5R and within R of
// the axis, 2R below.
enum C8Probe : std::size_t { kAxisR, kAxis2R, kAxis3R, kAxis4R, kDiscHalfR, kDiscR, kC8Probes };

// splitmix64, as B4's: a fixed-seed stream, the same on every compiler.
struct SplitMix {
    uint64_t state = 0;
    uint64_t next() noexcept {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
};

// C8's case as plan A's Task 11 fixed it, with the disc's increments from
// zero (packed u, v, w, as B4 packs them) and the probes' faces.
struct C8Case {
    GridShape shape{64, 128, 64, 0.01f};
    DomainBc bc;  // open faces, still air: every face an outlet
    float rho = 1.225f;
    float vh = 4.0f;
    ActuatorDisc disc;
    FluidConfig cfg;
    std::array<std::size_t, 3> faces{};
    std::vector<float> spread;
    std::vector<std::size_t> touched;
    std::vector<std::size_t> disc_half_r;
    std::vector<std::size_t> disc_r;
};

[[nodiscard]] C8Case make_c8() {
    C8Case c;
    const float radius = 0.1f;
    const float area = static_cast<float>(std::numbers::pi) * radius * radius;
    c.disc = ActuatorDisc{.centre = {0.32f, 0.96f, 0.32f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = radius,
                          .thrust = 2.0f * c.rho * area * c.vh * c.vh};
    c.cfg = FluidConfig{.rho = c.rho, .nu = 0.0f, .h = c.shape.dx / 8.0f, .n_v = 2};
    const GridShape& s = c.shape;
    c.faces = {std::size_t{s.nx + 1u} * s.ny * s.nz, std::size_t{s.nx} * (s.ny + 1u) * s.nz,
               std::size_t{s.nx} * s.ny * (s.nz + 1u)};
    std::vector<float> du(c.faces[0], 0.0f), dv(c.faces[1], 0.0f), dw(c.faces[2], 0.0f);
    spread_actuator(c.disc, s, c.rho, c.cfg.h, du, dv, dw);
    c.spread.insert(c.spread.end(), du.begin(), du.end());
    c.spread.insert(c.spread.end(), dv.begin(), dv.end());
    c.spread.insert(c.spread.end(), dw.begin(), dw.end());
    for (std::size_t i = 0; i < c.spread.size(); ++i) {
        if (c.spread[i] != 0.0f) c.touched.push_back(i);
    }
    // The v faces 2R below (j = 76) within r cells of the axis (x = z = 0.32 m,
    // the corner of cells 31 and 32), by their centres.
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t i = 0; i < s.nx; ++i) {
            const int64_t a = 2 * int64_t{i} + 1 - 64;
            const int64_t b = 2 * int64_t{k} + 1 - 64;
            if (a * a + b * b <= 4 * 5 * 5) c.disc_half_r.push_back(v_index(s, i, 76, k));
            if (a * a + b * b <= 4 * 10 * 10) c.disc_r.push_back(v_index(s, i, 76, k));
        }
    }
    return c;
}

// One run from still air, the probes after every step. Seed 0: the run as it
// is, spread_actuator() each step. Any other seed: the disc's increments with
// kC8Nudged picks from the touched faces put 1 ulp up, copied in each step.
// Copying the unnudged increments in reproduces the run to the bit (B4).
struct C8Run {
    std::vector<std::array<double, kC8Probes>> probes;
    std::optional<uint32_t> fault_at;
};

[[nodiscard]] C8Run c8_run(const C8Case& c, uint64_t seed, uint32_t steps) {
    std::vector<float> fixed;
    if (seed != 0) {
        fixed = c.spread;
        SplitMix rng{seed};
        for (std::size_t m = 0; m < kC8Nudged; ++m) {
            const std::size_t i = c.touched[rng.next() % c.touched.size()];
            fixed[i] = std::nextafter(fixed[i], std::numeric_limits<float>::infinity());
        }
    }
    const auto nu = static_cast<std::ptrdiff_t>(c.faces[0]);
    const auto nv = static_cast<std::ptrdiff_t>(c.faces[1]);
    MacGrid g = *make_grid(c.shape, c.bc);
    FluidScratch s = *make_fluid_scratch(g, c.bc, c.cfg);
    const auto axis = [&](uint32_t j) {
        const auto v = [&](uint32_t i, uint32_t k) { return static_cast<double>(g.v[v_index(c.shape, i, j, k)]); };
        return -0.25 * (v(31, 31) + v(32, 31) + v(31, 32) + v(32, 32));
    };
    const auto mean = [&](const std::vector<std::size_t>& faces) {
        double sum = 0.0;
        for (const std::size_t f : faces) sum += static_cast<double>(g.v[f]);
        return -sum / static_cast<double>(faces.size());
    };
    C8Run out;
    out.probes.reserve(steps);
    for (uint32_t k = 0; k < steps; ++k) {
        if (seed == 0) {
            spread_actuator(c.disc, c.shape, c.rho, c.cfg.h, s.du, s.dv, s.dw);
        } else {
            std::copy(fixed.begin(), fixed.begin() + nu, s.du.begin());
            std::copy(fixed.begin() + nu, fixed.begin() + nu + nv, s.dv.begin());
            std::copy(fixed.begin() + nu + nv, fixed.end(), s.dw.begin());
        }
        if (fluid_step(g, c.bc, c.cfg, s).fault != FluidFault::none) {
            out.fault_at = k + 1;
            break;
        }
        out.probes.push_back({axis(86), axis(76), axis(66), axis(56), mean(c.disc_half_r), mean(c.disc_r)});
    }
    return out;
}

// The runs for `seeds`, kC8Threads at a time. Each run is single-threaded, so
// its bits are the same however many run beside it.
[[nodiscard]] std::vector<C8Run> c8_runs(const C8Case& c, const std::vector<uint64_t>& seeds, uint32_t steps) {
    std::vector<C8Run> runs(seeds.size());
    for (std::size_t base = 0; base < seeds.size(); base += kC8Threads) {
        std::vector<std::thread> pool;
        for (std::size_t m = base; m < std::min<std::size_t>(seeds.size(), base + kC8Threads); ++m) {
            pool.emplace_back([&, m] { runs[m] = c8_run(c, seeds[m], steps); });
        }
        for (std::thread& t : pool) t.join();
    }
    return runs;
}

// A probe's mean over steps first to last, counted from 1.
[[nodiscard]] double window_mean(const C8Run& r, C8Probe p, uint32_t first, uint32_t last) {
    double sum = 0.0;
    for (uint32_t k = first; k <= last; ++k) sum += r.probes[k - 1][p];
    return sum / static_cast<double>(last - first + 1);
}

TEST(AirflowValidation, C8ActuatorDiscFarWakeAgainstMomentumTheory) {
    if (!validation_enabled()) {
        GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the actuator-disc wake, eight runs (~11 min)";
    }
    const C8Case c = make_c8();
    ASSERT_TRUE(check_disc(c.disc, c.shape).has_value());
    ASSERT_EQ(c.disc_half_r.size(), 80u);
    // Fixed before this run: run 0 as it is, and seven seeds the note's
    // measurement did not use.
    const std::vector<uint64_t> seeds = {0, 0x1C8, 0x1C9, 0x1CA, 0x1CB, 0x1CC, 0x1CD, 0x1CE};
    const std::vector<C8Run> runs = c8_runs(c, seeds, kC8Steps);
    std::vector<double> disc;
    std::vector<double> axis;
    for (std::size_t m = 0; m < runs.size(); ++m) {
        ASSERT_FALSE(runs[m].fault_at.has_value()) << "run " << m << " faulted at step " << *runs[m].fault_at;
        disc.push_back(window_mean(runs[m], kDiscHalfR, 385, kC8Steps));
        axis.push_back(window_mean(runs[m], kAxis2R, 257, 384));
    }
    const double expected = c.vh * (1.0 + 2.0 / std::sqrt(5.0));
    double mean = 0.0;
    for (const double d : disc) mean += d;
    mean /= static_cast<double>(disc.size());
    std::size_t worst = 0;
    double deviation = 0.0;
    for (std::size_t m = 0; m < disc.size(); ++m) {
        if (std::fabs(disc[m] - expected) > std::fabs(disc[worst] - expected)) worst = m;
        deviation = std::max(deviation, std::fabs(disc[m] - mean));
    }
    const bool mean_within = std::fabs(mean - expected) <= 0.1 * expected;
    const bool worst_within = std::fabs(disc[worst] - expected) <= 0.1 * expected;
    std::string seed_list;
    for (std::size_t m = 1; m < seeds.size(); ++m) {
        char hex[16];
        std::snprintf(hex, sizeof hex, "%s0x%llX", m == 1 ? "" : ", ", static_cast<unsigned long long>(seeds[m]));
        seed_list += hex;
    }
    const auto [axis_lo, axis_hi] = std::minmax_element(axis.begin(), axis.end());
    std::printf("=== validation record: C8 actuator-disc far wake ===\n"
                "path: %s\n"
                "config: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T %.4f N, v_h 4 m/s, still air, inviscid, "
                "h = %.6f, n_v 2, %u steps; eight runs that differ only by a nudge in the test, 1 ulp up on %zu "
                "increments, seed s: run 0 as it is, runs 1 to 7 seeds %s. So the spread is the measure's "
                "sensitivity to rounding, not a physical uncertainty\n"
                "measure: the eight runs' mean of the mean downwash over the 80 v faces within 0.5R of the axis, 2R "
                "below the disc, steps 385 to %u = %.4f m/s\n"
                "reference: momentum theory, v_h (1 + 2/sqrt 5) = %.4f m/s (tests/data/airflow/README.md, C8); the "
                "vortex-cylinder model's mean over the same faces is 7.5909 m/s\n"
                "tolerance: within 10%% (plan A Task 11, spec \xc2\xa7" "11.2, fixed before the first run; the "
                "measure and the eight runs fixed before this run, docs/design/test-docs/2026-10-22-c8-spread.md)\n"
                "verdict: %s\n"
                "measure: the worst of the eight runs, run %zu, the same mean = %.4f m/s\n"
                "reference: momentum theory, %.4f m/s, as above\n"
                "tolerance: within 10%% (as above: the verdict holds across the spread)\n"
                "verdict: %s\n"
                "measure: the eight runs' largest deviation from their mean, the same mean = %.4f m/s\n"
                "reference: the spread's size: past T_p the GPU's band is 4x it (test-docs/01-verification.md, a "
                "case judged past its horizon)\n"
                "tolerance: none\n"
                "verdict: report only\n"
                "measure: C8's first measure, the axis 2R below over steps 257 to 384, run 0 (the eight runs %.4f "
                "to %.4f) = %.4f m/s\n"
                "reference: momentum theory, %.4f m/s, as above\n"
                "tolerance: none (report only since 2026-10-22: the axis sits in the disc's unloaded hub)\n"
                "verdict: report only\n",
                path_line().c_str(), static_cast<double>(c.disc.thrust), static_cast<double>(c.cfg.h), kC8Steps,
                kC8Nudged, seed_list.c_str(), kC8Steps, mean, expected, verdict(mean_within), worst, disc[worst],
                expected, verdict(worst_within), deviation, *axis_lo, *axis_hi, axis[0], expected);
    EXPECT_TRUE(mean_within) << mean << " against " << expected;
    EXPECT_TRUE(worst_within) << "run " << worst << ": " << disc[worst] << " against " << expected;
}

// The measurement behind C8's measure: every probe after every step, opt-in.
// SPADE_C8_ENSEMBLE=<first>:<count>[:<steps>] runs runs first to
// first + count - 1, each <steps> long (768 if left out): run 0 as it is, run
// m > 0 from seed 0xC8 + m - 1, so runs 1 to 3 are B4's three. It repeats the
// note's 16 runs to the bit (docs/design/test-docs/2026-10-22-c8-spread.md).
// A measuring tool: it prints no record, and asserts only healthy runs.
[[nodiscard]] std::optional<std::array<uint32_t, 3>> c8_ensemble_from_env() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* text = std::getenv("SPADE_C8_ENSEMBLE");
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (text == nullptr) return std::nullopt;
    char* end = nullptr;
    const auto first = static_cast<uint32_t>(std::strtoul(text, &end, 10));
    if (*end != ':') return std::nullopt;
    const auto count = static_cast<uint32_t>(std::strtoul(end + 1, &end, 10));
    uint32_t steps = kC8Steps;
    if (*end == ':') steps = static_cast<uint32_t>(std::strtoul(end + 1, &end, 10));
    if (*end != '\0' || count == 0 || steps < 384) return std::nullopt;
    return std::array<uint32_t, 3>{first, count, steps};
}

TEST(AirflowValidation, C8OneUlpEnsembleProbes) {
    const auto ens = c8_ensemble_from_env();
    if (!ens) GTEST_SKIP() << "set SPADE_C8_ENSEMBLE=<first>:<count>[:<steps>] to run C8's one-ulp ensemble";
    const auto [first, count, steps] = *ens;
    const C8Case c = make_c8();
    ASSERT_TRUE(check_disc(c.disc, c.shape).has_value());
    std::vector<uint64_t> seeds;
    for (uint32_t m = first; m < first + count; ++m) seeds.push_back(m == 0 ? 0 : 0xC8ull + m - 1);
    const std::vector<C8Run> runs = c8_runs(c, seeds, steps);
    std::printf("c8spread: C8's case, %u steps; run 0 as it is, run m > 0 puts %zu of the disc's %zu increments "
                "1 ulp up from seed 0xC8 + m - 1; disc 0.5R holds %zu faces, disc R %zu\n",
                steps, kC8Nudged, c.touched.size(), c.disc_half_r.size(), c.disc_r.size());
    std::printf("c8spread: member step axisR axis2R axis3R axis4R disc05R disc1R\n");
    for (uint32_t m = 0; m < count; ++m) {
        const C8Run& r = runs[m];
        EXPECT_FALSE(r.fault_at.has_value()) << "run " << first + m << " faulted at step " << *r.fault_at;
        for (std::size_t k = 0; k < r.probes.size(); ++k) {
            const auto& p = r.probes[k];
            std::printf("c8spread: %u %zu %.7f %.7f %.7f %.7f %.7f %.7f\n", first + m, k + 1, p[kAxisR], p[kAxis2R],
                        p[kAxis3R], p[kAxis4R], p[kDiscHalfR], p[kDiscR]);
        }
        if (r.probes.size() >= 384) {
            std::printf("c8spread-member: %u seed 0x%llX C8's first measure (axis 2R, steps 257 to 384) %.6f m/s\n",
                        first + m, static_cast<unsigned long long>(seeds[m]), window_mean(r, kAxis2R, 257, 384));
        }
    }
}

}  // namespace
