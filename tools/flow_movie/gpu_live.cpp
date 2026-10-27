#include "gpu_live.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/flow_volume.hpp"
#include "frame_check.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "spike/airflow/step_chain.hpp"
#include "vehicles/rotor_discs.hpp"

namespace flow_movie {
namespace an = spade::analysis;
namespace af = spade::physics::airflow;
namespace fs = std::filesystem;

namespace {

constexpr uint64_t kNsPerSecond = 1'000'000'000;  // live_run.cpp's

constexpr const char* kChannelNames[] = {"u", "v", "w", "dye", "pressure", "|w|", "q-hat", "lambda2-hat"};

[[nodiscard]] bool same(const an::Controls& a, const an::Controls& b) {
    return a.armed == b.armed && a.throttle == b.throttle && a.pitch_deg == b.pitch_deg && a.roll_deg == b.roll_deg &&
           a.yaw_deg == b.yaw_deg;
}

}  // namespace

spade::Result<void> simulate_gpu(const LiveCase& lc, uint32_t frames, const std::string& stem, const fs::path& cache,
                                 const CacheMark& mark) {
    const auto failed = [&stem](const spade::Error& e) {
        return std::unexpected(spade::Error{e.code, stem + ": " + e.context});
    };
    const auto refused = [&stem](const std::string& why) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, stem + ": " + why});
    };

    // The program held: the discs are formed once.
    const an::Controls held = an::controls_at(lc.program, 0.0f);
    for (const an::ControlPoint& p : lc.program.points) {
        if (!same(p.controls, held)) {
            char at[32];
            std::snprintf(at, sizeof at, "%.3f", static_cast<double>(p.t_s));
            return refused("--gpu holds the rotors at the program's first controls, and " + lc.program_name +
                           " changes them at t = " + at + " s");
        }
    }
    auto resolved = resolve_live(lc);
    if (!resolved) return failed(resolved.error());
    const af::GridShape& shape = resolved->base;
    const af::DomainBc& bc = resolved->domain;
    const glm::vec3 origin = -0.5f * shape.dx *
                             glm::vec3(static_cast<float>(shape.nx), static_cast<float>(shape.ny),
                                       static_cast<float>(shape.nz));

    // The rotors' rows and the held body, after LiveRun's first step.
    std::vector<spade::vehicles::RotorRow> rows;
    spade::BodyState body{};
    std::string label;
    {
        auto run = an::LiveRun::start(*resolved, lc.run_model);
        if (!run) return failed(run.error());
        if (run->solids().any) {
            return refused("--gpu films the flow without the model's parts until GpuFluid takes solids (B7's G3), "
                           "and this run model has them in the flow (" + run->solids().run_line + ")");
        }
        if (spade::Result<void> s = run->step(held); !s) return failed(s.error());
        rows.assign(run->rotor_rows().begin(), run->rotor_rows().end());
        body = run->body();
        label = run->run_label();
    }
    std::vector<double> live{body.pos.x, body.pos.y, body.pos.z, body.orient.w, body.orient.x, body.orient.y,
                             body.orient.z};
    for (spade::vehicles::RotorRow& r : rows) {
        r.omega = r.omega_cmd;
        live.push_back(r.omega);
    }
    std::vector<af::ActuatorDisc> discs(rows.size());
    if (!rows.empty()) {
        if (spade::Result<void> d = spade::vehicles::rotor_discs(rows, body, origin, shape, discs); !d) {
            return failed(d.error());
        }
    }
    char held_at[96];
    std::snprintf(held_at, sizeof held_at, " · fluid on the GPU (GpuFluid), no parts; rotors held at %.0f rad/s",
                  rows.empty() ? 0.0 : static_cast<double>(rows.front().omega));
    label += held_at;

    // LiveRun's fluid without solids, from rest.
    const an::AnalysisCase& c = resolved->spec;
    const auto h_ns =
        static_cast<uint64_t>(static_cast<double>(*c.time.step) * static_cast<double>(kNsPerSecond) + 0.5);
    af::FluidConfig cfg;
    cfg.rho = c.fluid.density;
    cfg.nu = c.fluid.kinematic_viscosity;
    cfg.h = static_cast<float>(static_cast<double>(h_ns) / static_cast<double>(kNsPerSecond));
    spade::Result<af::MacGrid> grid = af::make_grid(shape, bc);
    if (!grid) return failed(grid.error());
    auto ctx = spade::compute::VulkanContext::create(
        spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!ctx) return failed(ctx.error());
    auto fluid = spade::airflow_gpu::GpuFluid::create(**ctx, *grid, bc, cfg);
    if (!fluid) return failed(fluid.error());

    // The flow's bound (the lead, 2026-10-27): ten times the case's expected
    // top speed, the speed colours' top (the wind plus momentum theory's
    // 2 v_h), or twice the wind for a case without one; at least 10 m/s.
    // Past it the flow has blown up, finite or not.
    const float expected = lc.speed_max > 0.0f ? lc.speed_max : 2.0f * lc.c.flight.speed;
    const float bound = 10.0f * std::max(expected, 1.0f);

    std::error_code ec;
    fs::create_directories(cache, ec);
    mark_cache(cache, mark);
    std::ofstream lines(cache / "run.txt", std::ios::binary | std::ios::trunc);
    spade::FlowVolume volume;
    uint64_t steps = 0;
    double stepping_s = 0.0;
    using Clock = std::chrono::steady_clock;
    const Clock::time_point start = Clock::now();
    for (uint32_t k = 0; k < frames; ++k) {
        // advance_frame()'s clock: frame k + 1 ends at round((k + 1) / fps) s, in whole ns.
        const uint64_t end_ns = ((k + 1u) * kNsPerSecond + an::kFramesPerSecond / 2u) / an::kFramesPerSecond;
        const Clock::time_point t0 = Clock::now();
        while (steps * h_ns < end_ns) {
            if (auto s = (*fluid)->step(discs); !s) return failed(s.error());
            ++steps;
        }
        stepping_s += std::chrono::duration<double>(Clock::now() - t0).count();
        if (spade::Result<void> p = (*fluid)->publish(origin, volume); !p) return failed(p.error());
        const double t_s = static_cast<double>(steps * h_ns) / static_cast<double>(kNsPerSecond);
        if (const std::optional<NonFinite> bad = first_non_finite(volume)) {
            char why[320];
            std::snprintf(why, sizeof why,
                          "frame %u/%u, after step %llu (t %.6f s), holds %zu non-finite samples, the first %s at "
                          "cell (%u, %u, %u), half 0x%04x: the fluid went NaN, infinite or past binary16's range at "
                          "or before step %llu; the frame is not cached and the film stops",
                          k + 1u, frames, static_cast<unsigned long long>(steps), t_s, bad->count,
                          kChannelNames[bad->channel], bad->i, bad->j, bad->k, static_cast<unsigned>(bad->bits),
                          static_cast<unsigned long long>(steps));
            return std::unexpected(spade::Error{spade::Code::internal, stem + ": " + why});
        }
        const FastestCell fast = fastest_cell(volume);
        if (fast.speed > bound) {
            char why[320];
            std::snprintf(why, sizeof why,
                          "frame %u/%u, after step %llu (t %.6f s): max |v| %.1f m/s at cell (%u, %u, %u), past the "
                          "bound %.1f m/s (10 x the case's expected top speed, %.2f m/s): the flow blew up at or "
                          "before step %llu; the frame is not cached and the film stops",
                          k + 1u, frames, static_cast<unsigned long long>(steps), t_s,
                          static_cast<double>(fast.speed), fast.i, fast.j, fast.k, static_cast<double>(bound),
                          static_cast<double>(expected), static_cast<unsigned long long>(steps));
            return std::unexpected(spade::Error{spade::Code::internal, stem + ": " + why});
        }
        if (auto wrote = write_frame(cache, k, volume, FrameLog{steps, t_s, live}); !wrote) {
            return failed(wrote.error());
        }
        lines << steps << '\t' << label << '\n';
        if (k % 10u == 0u || k + 1u == frames) {
            const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
            std::printf("[ %s ] frame %u/%u, t %.3f s, step %llu, max |v| %.2f m/s; %.1f s\n", stem.c_str(), k + 1u,
                        frames, t_s, static_cast<unsigned long long>(steps), static_cast<double>(fast.speed),
                        elapsed);
            std::fflush(stdout);
        }
    }
    if (!lines) return std::unexpected(spade::Error{spade::Code::io_error, "writing " + (cache / "run.txt").string()});
    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    std::printf("[ %s ] %llu steps on the GPU in %.1f s, %.2f ms a step; the frames' publish, check and cache %.1f s\n",
                stem.c_str(), static_cast<unsigned long long>(steps), stepping_s,
                steps > 0u ? 1e3 * stepping_s / static_cast<double>(steps) : 0.0, elapsed - stepping_s);
    return {};
}

}  // namespace flow_movie
