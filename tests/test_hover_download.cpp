// test_hover_download.cpp -- the held reference quad's hover download at Run air's Detailed level, on the CPU.
// One env-gated measurement, report only (Test/Docs names the headless Loads test's bound from its numbers):
//
//   SPADE_HOVER_DOWNLOAD=<seconds>   e.g. 3.5: the quad held at its hover throttle in still air, its ten rigid parts
//                                    the grid's solids (P2), the discs on, at live_grid()'s cell and the Detailed
//                                    level's step (live_level_step_s(), CFL <= 2). After every fluid step the wall
//                                    load (wall_load(), the pressure force the air puts on the solids, the grid's
//                                    axes, Y up) is read. The first 0.5 s is start-up and is dropped; the rest is cut
//                                    into windows of 0.5 s, and the line prints each window's mean and the spread.
//
// The load is the pressure force on the solids as the step left it; F_y below zero is a download.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "d1_case.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/wall_load.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::physics::airflow;

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

// The CPU's fluid behind LiveRun's seam, with the solids attached and the wall load read after each step.
struct LoadedFluid final : an::LiveFluid {
    af::MacGrid g;
    af::DomainBc bc;
    af::FluidConfig cfg;
    af::FluidScratch s;
    std::vector<glm::vec3> load;  // the wall load's force after each step, N

    spade::Result<af::FluidHealth> step(std::span<const af::ActuatorDisc> discs) override {
        for (const af::ActuatorDisc& d : discs) af::spread_actuator(d, g.shape, cfg.rho, cfg.h, s.du, s.dv, s.dw);
        const af::FluidHealth h = af::fluid_step(g, bc, cfg, s);
        load.push_back(af::wall_load(g, s.solids, s.regions, glm::vec3(0.0f)).force);
        return h;
    }
    spade::Result<void> publish(const af::PublishInput&, spade::FlowVolume&) override { return {}; }
    spade::Result<void> read(af::MacGrid& out) const override {
        out = g;
        return {};
    }
};

struct Moments {
    double mean = 0.0;
    double sd = 0.0;  // sample standard deviation
};

Moments moments(const std::vector<double>& v) {
    Moments m;
    if (v.empty()) return m;
    for (const double x : v) m.mean += x;
    m.mean /= static_cast<double>(v.size());
    if (v.size() > 1) {
        double ss = 0.0;
        for (const double x : v) ss += (x - m.mean) * (x - m.mean);
        m.sd = std::sqrt(ss / static_cast<double>(v.size() - 1));
    }
    return m;
}

TEST(HoverDownload, TheHeldQuadsPressureLoadAtTheDetailedLevel) {
    const char* spec = env_text("SPADE_HOVER_DOWNLOAD");
    if (spec == nullptr) GTEST_SKIP() << "set SPADE_HOVER_DOWNLOAD=<seconds> (e.g. 3.5) to measure";
    const double seconds = std::strtod(spec, nullptr);
    ASSERT_GT(seconds, 1.0);

    auto q = spade::testing::powered_library_quad(spade::testing::PoweredQuadSetup{
        .seconds = seconds, .parts_in_flow = true, .hover_only = true});
    ASSERT_TRUE(q.has_value()) << q.error().context;
    const auto grid = an::live_grid(q->run_model);
    ASSERT_TRUE(grid.has_value()) << grid.error().context;
    q->c.flight.kind = an::FlightKind::hover;  // still air: the case's wind is not the question
    q->c.flight.speed = 0.0f;
    q->c.grid.spacing = grid->dx;
    q->c.grid.box = grid->dx * static_cast<float>(grid->n);
    q->c.time.step = an::live_level_step_s(q->c, q->run_model, grid->dx);
    const auto resolved = an::resolve(q->c, q->run_model, an::ThinWalls::drawn_by_the_live_run);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().front().field << ": " << resolved.error().front().message;

    LoadedFluid* kept = nullptr;
    const an::LiveFluidMaker maker = [&kept](const an::LiveFluidStart& st)
        -> spade::Result<std::unique_ptr<an::LiveFluid>> {
        auto f = std::make_unique<LoadedFluid>();
        f->g = st.grid;
        f->bc = st.domain;
        f->cfg = st.config;
        auto s = af::make_fluid_scratch(f->g, f->bc, f->cfg);
        if (!s) return std::unexpected(s.error());
        f->s = std::move(*s);
        if (st.solids != nullptr) {
            if (auto a = af::attach_solid_field(f->s, f->g, f->bc, *st.solids); !a) return std::unexpected(a.error());
        }
        kept = f.get();
        return std::unique_ptr<an::LiveFluid>(std::move(f));
    };
    spade::Result<an::LiveRun> run = an::LiveRun::start(*resolved, q->run_model, maker);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    ASSERT_NE(kept, nullptr);
    ASSERT_TRUE(kept->s.solids.any) << "the parts are in the air: " << run->run_label();

    const double h = run->step_s();
    const int steps = static_cast<int>(std::lround(seconds / h));
    const int per_tick = std::max(1, static_cast<int>(std::lround(0.1 / h)));
    std::printf("HOVERDL start: %ux%ux%u cells at dx %.4f m, h %.6f s, %d steps for %.2f s, a progress line every %d "
                "steps (0.1 s)\n",
                kept->g.shape.nx, kept->g.shape.ny, kept->g.shape.nz, static_cast<double>(kept->g.shape.dx), h, steps,
                seconds, per_tick);
    std::fflush(stdout);
    const auto t_start = std::chrono::steady_clock::now();
    const an::Controls hover{.armed = true, .throttle = q->trim.hover};
    for (int t = 0; t < steps; ++t) {
        const auto r = run->step(hover);
        ASSERT_TRUE(r.has_value()) << "step " << t + 1 << ": " << r.error().context;
        if ((t + 1) % per_tick == 0) {  // flushed: a killed run still shows where it got
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
            std::printf("HOVERDL progress: %d of %d steps, t %.2f s, wall %.1f s, %.3f steps/s, F_y %.5f N\n", t + 1,
                        steps, static_cast<double>(t + 1) * h, wall, static_cast<double>(t + 1) / wall,
                        static_cast<double>(kept->load.back().y));
            std::fflush(stdout);
        }
    }
    ASSERT_EQ(kept->load.size(), static_cast<std::size_t>(steps));

    const int skip = static_cast<int>(std::lround(0.5 / h));
    const int per_window = static_cast<int>(std::lround(0.5 / h));
    const int windows = (steps - skip) / per_window;
    ASSERT_GE(windows, 1) << "a window is 0.5 s after the first 0.5 s: ask for at least 1.0 s";
    std::vector<double> window_mean_y;
    std::vector<double> within_sd_y;
    std::vector<double> all_y;
    for (int w = 0; w < windows; ++w) {
        std::vector<double> fy;
        for (int t = skip + w * per_window; t < skip + (w + 1) * per_window; ++t) {
            fy.push_back(static_cast<double>(kept->load[static_cast<std::size_t>(t)].y));
        }
        const Moments m = moments(fy);
        window_mean_y.push_back(m.mean);
        within_sd_y.push_back(m.sd);
        all_y.insert(all_y.end(), fy.begin(), fy.end());
    }
    const Moments of_means = moments(window_mean_y);
    const Moments step_to_step = moments(all_y);
    double fx = 0.0;
    double fz = 0.0;
    for (int t = skip; t < skip + windows * per_window; ++t) {
        fx += static_cast<double>(kept->load[static_cast<std::size_t>(t)].x);
        fz += static_cast<double>(kept->load[static_cast<std::size_t>(t)].z);
    }
    const double n_used = static_cast<double>(windows * per_window);
    std::printf("HOVERDL config: %s; CPU reference, %u thread(s); %u^3 cells at dx %.4f m, h %.6f s (Detailed "
                "level, CFL <= 2), %d steps over %.2f s, the first 0.5 s dropped, %d windows of 0.5 s (%d steps "
                "each)\n",
                run->run_label().c_str(), kept->cfg.threads, grid->n, static_cast<double>(grid->dx), h, steps, seconds,
                windows, per_window);
    for (int w = 0; w < windows; ++w) {
        std::printf("HOVERDL window %d: mean F_y %.5f N, step-to-step sd %.5f N\n", w + 1,
                    window_mean_y[static_cast<std::size_t>(w)], within_sd_y[static_cast<std::size_t>(w)]);
    }
    std::printf("HOVERDL result: mean of the window means F_y %.5f N, sd of the window means %.5f N, "
                "step-to-step sd %.5f N (all steps), min %.5f, max %.5f; mean F_x %.5f N, mean F_z %.5f N; "
                "F_y below zero is a download\n",
                of_means.mean, of_means.sd, step_to_step.sd, *std::min_element(all_y.begin(), all_y.end()),
                *std::max_element(all_y.begin(), all_y.end()), fx / n_used, fz / n_used);
    EXPECT_TRUE(std::isfinite(of_means.mean));
}

}  // namespace
