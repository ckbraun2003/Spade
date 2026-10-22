// C8's spread under the CPU's own last bits (the lead, 2026-10-22, after
// Rendering's B4 finding at e965df2; C8's record,
// tests/data/airflow/records/c8-actuator-disc.md). C8's far wake is chaotic
// past about step 140, so its measure, the mean of the last 128 of 384 steps
// on the axis 2R below, moves with a last bit. This runs an ensemble of C8
// runs, each with 110 of the disc's increments put 1 ulp up, the same every
// step: Rendering's nudge (TEST(GpuAirflowActuator, C8GpuGatherInTheCpuStep)),
// so seeds 0xC8, 0xC9 and 0xCA repeat B4's three runs. 110 is how many
// increments the GPU gather differs on.
//
// A measuring harness, not a verdict: it prints every probe after every step,
// for choosing a measure whose verdict holds across the spread, and no record.
// It asserts only that every run stays healthy.
//
// The probes, all of the downwash -v:
//   axis R, 2R, 3R, 4R   C8's four-face mean on the axis, R to 4R below
//                        (2R is C8's own probe)
//   disc 0.5R, disc R    the mean over the faces within 0.5R and within R of
//                        the axis, 2R below
//
// Opt-in: SPADE_C8_ENSEMBLE=<first>:<count>[:<steps>] runs members first to
// first + count - 1, each <steps> long (768 if left out). Member 0 is C8
// unnudged; member m > 0 takes seed 0xC8 + m - 1. Up to four members run at
// once, one thread each; each run is single-threaded, so its bits are the
// same however many run beside it. About 2.3 minutes per 384 steps alone, so
// run it from the test binary with --gtest_filter, as C8 is.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <thread>
#include <vector>

#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

namespace af = spade::physics::airflow;

constexpr std::size_t kNudged = 110;
constexpr uint64_t kFirstSeed = 0xC8;
constexpr uint32_t kMaxThreads = 4;
constexpr uint32_t kProbes = 6;

struct Ensemble {
    uint32_t first = 0;
    uint32_t count = 0;
    uint32_t steps = 768;
};

[[nodiscard]] std::optional<Ensemble> ensemble_from_env() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* text = std::getenv("SPADE_C8_ENSEMBLE");
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (text == nullptr) return std::nullopt;
    Ensemble e;
    char* end = nullptr;
    e.first = static_cast<uint32_t>(std::strtoul(text, &end, 10));
    if (*end != ':') return std::nullopt;
    e.count = static_cast<uint32_t>(std::strtoul(end + 1, &end, 10));
    if (*end == ':') e.steps = static_cast<uint32_t>(std::strtoul(end + 1, &end, 10));
    if (*end != '\0' || e.count == 0 || e.steps < 384) return std::nullopt;
    return e;
}

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

// One member's run: the probes after every step, or the step it faulted at.
struct MemberRun {
    std::vector<std::array<double, kProbes>> probes;
    std::optional<uint32_t> fault_at;
    double seconds = 0.0;
};

TEST(AirflowC8Spread, EnsembleOfOneUlpNudges) {
    const std::optional<Ensemble> ens = ensemble_from_env();
    if (!ens) GTEST_SKIP() << "set SPADE_C8_ENSEMBLE=<first>:<count>[:<steps>] to run C8's 1-ulp ensemble";

    // C8's case, as test_airflow_validation.cpp builds it.
    const af::GridShape shape{64, 128, 64, 0.01f};
    const af::DomainBc bc;  // open faces, still air: every face an outlet
    const float rho = 1.225f;
    const float vh = 4.0f;
    const float radius = 0.1f;
    const float area = static_cast<float>(std::numbers::pi) * radius * radius;
    const af::ActuatorDisc disc{.centre = {0.32f, 0.96f, 0.32f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = radius,
                                .thrust = 2.0f * rho * area * vh * vh};
    ASSERT_TRUE(af::check_disc(disc, shape).has_value());
    const af::FluidConfig cfg{.rho = rho, .nu = 0.0f, .h = shape.dx / 8.0f, .n_v = 2};

    // The disc's increments from zero, packed u, v, w, as B4 packs them.
    const std::size_t nu = std::size_t{shape.nx + 1u} * shape.ny * shape.nz;
    const std::size_t nv = std::size_t{shape.nx} * (shape.ny + 1u) * shape.nz;
    const std::size_t nw = std::size_t{shape.nx} * shape.ny * (shape.nz + 1u);
    std::vector<float> du(nu, 0.0f), dv(nv, 0.0f), dw(nw, 0.0f);
    af::spread_actuator(disc, shape, rho, cfg.h, du, dv, dw);
    std::vector<float> spread;
    spread.reserve(nu + nv + nw);
    spread.insert(spread.end(), du.begin(), du.end());
    spread.insert(spread.end(), dv.begin(), dv.end());
    spread.insert(spread.end(), dw.begin(), dw.end());
    std::vector<std::size_t> touched;
    for (std::size_t i = 0; i < spread.size(); ++i) {
        if (spread[i] != 0.0f) touched.push_back(i);
    }
    ASSERT_FALSE(touched.empty());

    // B4's nudge: `kNudged` picks from the touched faces, each 1 ulp up.
    const auto nudged = [&](uint64_t seed) {
        std::vector<float> out = spread;
        SplitMix rng{seed};
        for (std::size_t m = 0; m < kNudged; ++m) {
            const std::size_t i = touched[rng.next() % touched.size()];
            out[i] = std::nextafter(out[i], std::numeric_limits<float>::infinity());
        }
        return out;
    };

    // The v faces 2R below within `r` cells of the axis (x = z = 0.32 m, the
    // corner of cells 31 and 32), by their centres.
    const auto disc_faces = [&](uint32_t r) {
        std::vector<std::size_t> faces;
        for (uint32_t k = 0; k < shape.nz; ++k) {
            for (uint32_t i = 0; i < shape.nx; ++i) {
                const int64_t a = 2 * int64_t{i} + 1 - 64;
                const int64_t b = 2 * int64_t{k} + 1 - 64;
                if (a * a + b * b <= 4 * int64_t{r} * r) faces.push_back(af::v_index(shape, i, 76, k));
            }
        }
        return faces;
    };
    const std::vector<std::size_t> half = disc_faces(5);
    const std::vector<std::size_t> full = disc_faces(10);

    const auto run = [&](uint32_t member) {
        MemberRun out;
        const auto start = std::chrono::steady_clock::now();
        const std::vector<float> fixed = member == 0 ? std::vector<float>{} : nudged(kFirstSeed + member - 1);
        af::MacGrid g = *af::make_grid(shape, bc);
        af::FluidScratch s = *af::make_fluid_scratch(g, bc, cfg);
        const auto axis = [&](uint32_t j) {
            const auto v = [&](uint32_t i, uint32_t k) { return static_cast<double>(g.v[af::v_index(shape, i, j, k)]); };
            return -0.25 * (v(31, 31) + v(32, 31) + v(31, 32) + v(32, 32));
        };
        const auto mean = [&](const std::vector<std::size_t>& faces) {
            double sum = 0.0;
            for (const std::size_t f : faces) sum += static_cast<double>(g.v[f]);
            return -sum / static_cast<double>(faces.size());
        };
        out.probes.reserve(ens->steps);
        for (uint32_t k = 0; k < ens->steps; ++k) {
            if (member == 0) {
                af::spread_actuator(disc, shape, rho, cfg.h, s.du, s.dv, s.dw);
            } else {
                std::copy(fixed.begin(), fixed.begin() + static_cast<std::ptrdiff_t>(nu), s.du.begin());
                std::copy(fixed.begin() + static_cast<std::ptrdiff_t>(nu),
                          fixed.begin() + static_cast<std::ptrdiff_t>(nu + nv), s.dv.begin());
                std::copy(fixed.begin() + static_cast<std::ptrdiff_t>(nu + nv), fixed.end(), s.dw.begin());
            }
            if (af::fluid_step(g, bc, cfg, s).fault != af::FluidFault::none) {
                out.fault_at = k + 1;
                break;
            }
            out.probes.push_back({axis(86), axis(76), axis(66), axis(56), mean(half), mean(full)});
        }
        out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        return out;
    };

    std::printf("c8spread: C8's case (64x128x64 at dx 1 cm, R 0.1 m, T %.4f N, v_h 4 m/s, inviscid, h = %.6f, "
                "n_v 2), %u steps; member 0 unnudged, member m > 0 puts %zu of the disc's %zu increments 1 ulp up "
                "from seed 0x%llX + m - 1; disc 0.5R holds %zu faces, disc R %zu\n",
                static_cast<double>(disc.thrust), static_cast<double>(cfg.h), ens->steps, kNudged, touched.size(),
                static_cast<unsigned long long>(kFirstSeed), half.size(), full.size());
    std::printf("c8spread: member step axisR axis2R axis3R axis4R disc05R disc1R\n");
    std::vector<MemberRun> runs(ens->count);
    for (uint32_t base = 0; base < ens->count; base += kMaxThreads) {
        std::vector<std::thread> pool;
        for (uint32_t m = base; m < std::min(ens->count, base + kMaxThreads); ++m) {
            pool.emplace_back([&, m] { runs[m] = run(ens->first + m); });
        }
        for (std::thread& t : pool) t.join();
    }
    for (uint32_t m = 0; m < ens->count; ++m) {
        const uint32_t member = ens->first + m;
        const MemberRun& r = runs[m];
        EXPECT_FALSE(r.fault_at.has_value()) << "member " << member << " faulted at step " << *r.fault_at;
        for (std::size_t k = 0; k < r.probes.size(); ++k) {
            const auto& p = r.probes[k];
            std::printf("c8spread: %u %zu %.7f %.7f %.7f %.7f %.7f %.7f\n", member, k + 1, p[0], p[1], p[2], p[3],
                        p[4], p[5]);
        }
        double late = 0.0;
        for (std::size_t k = 256; k < std::min<std::size_t>(384, r.probes.size()); ++k) late += r.probes[k][1];
        std::printf("c8spread-member: %u seed 0x%llX C8's measure (axis 2R, steps 257 to 384) %.6f m/s, %.0f s\n",
                    member, static_cast<unsigned long long>(member == 0 ? 0 : kFirstSeed + member - 1), late / 128.0,
                    r.seconds);
    }
}

}  // namespace
