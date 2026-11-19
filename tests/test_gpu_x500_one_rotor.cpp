// ---------------------------------------------------------------------------
// test_gpu_x500_one_rotor.cpp -- L105 (the lead, 2026-11-19): does the lines' thrust at a fixed rotor speed converge once the blade tip moves less than a
// cell a step? ONE of the kit's rotors (the 1045 at its balance speed) in an open box, on the device, the same case as the CPU sweep
// (test_x500_case.cpp, AirflowX500.TheLinesThrustAgainstTheStepInOneRotorOnTheCpu): a 0.40 x 0.55 x 0.40 m box, the rotor 0.20 m under the top.
//
//   GpuX500OneRotor.TheLinesThrustAgainstCellsAPerStep   opt-in: SPADE_L105_GPU=1 (GPU, minutes)
//
// Knobs: SPADE_L105_DXMM (the cell in mm, default 5; the box in cells follows), SPADE_L105_CELLS (the tip's cells a step, comma list, default
// "1.45,0.72,0.36,0.18": h = cells dx / (omega R)), SPADE_L105_T (the last time, s, default 0.1), SPADE_L105_NSUB (8), SPADE_L105_NV (the projection's
// V-cycles, the config's default), SPADE_L105_CS (the Smagorinsky constant, 0; the film's is 0.17), SPADE_L105_NU (the air's viscosity, m^2/s, 0; the film's is 1.5e-5), SPADE_L105_DEPTH (the box's height in m, 0.55: 0.35 m under the rotor,
// which the wake front crosses at about 0.065 s; a later time wants a deeper box), SPADE_L105_EVERY (a step count: every that many steps print the
// breakdown's evidence -- the disc's downflow at three planes, the box's downward momentum against the impulse put in, the extreme v and where, the
// pressure's mean and largest, the projection's health since the last print, and the host azimuth against step x h x omega). Reports the thrust at 0.01, 0.02, 0.05, 0.075 and 0.1 s (up to T) as the
// step's own and as the mean over the last revolution, and the disc's mean downflow at 0.05 s and T against momentum theory.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"
#include "spike/airflow/step_chain.hpp"
#include "x500_model.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
namespace x5 = spade::testing::x500;

[[nodiscard]] const char* l105g_env(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] double l105g_number(const char* name, double fallback) {
    const char* v = l105g_env(name);
    return v != nullptr ? std::strtod(v, nullptr) : fallback;
}

[[nodiscard]] std::vector<double> l105g_cells() {
    const char* v = l105g_env("SPADE_L105_CELLS");
    std::stringstream in(v != nullptr ? v : "1.45,0.72,0.36,0.18");
    std::vector<double> out;
    std::string item;
    while (std::getline(in, item, ',')) out.push_back(std::strtod(item.c_str(), nullptr));
    return out;
}

TEST(GpuX500OneRotor, TheLinesThrustAgainstCellsAPerStep) {
    if (l105g_env("SPADE_L105_GPU") == nullptr) GTEST_SKIP() << "opt-in: SPADE_L105_GPU=1 (GPU, minutes)";
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const auto part = x5::prop_part();
    ASSERT_TRUE(part.has_value()) << part.error().context;
    const auto prop = spade::scene::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const float target = static_cast<float>(x5::kMass * 9.80665 / 4.0);
    const auto omega = x5::balance_omega(prop->blade, prop->polar, target);
    ASSERT_TRUE(omega.has_value()) << omega.error().context;
    constexpr float kRho = 1.225f;
    constexpr double kPi = 3.14159265358979;
    const double radius = static_cast<double>(prop->blade.radius);
    const double area = kPi * radius * radius;
    const double tip_speed = static_cast<double>(*omega) * radius;
    const double rev = 2.0 * kPi / static_cast<double>(*omega);  // s
    const float dx = static_cast<float>(0.001 * l105g_number("SPADE_L105_DXMM", 5.0));
    const uint32_t n_sub = static_cast<uint32_t>(l105g_number("SPADE_L105_NSUB", 8.0));
    const double t_end = l105g_number("SPADE_L105_T", 0.1);
    const double sample_t[] = {0.01, 0.02, 0.05, 0.075, 0.1};
    std::printf("[ L105g   ] omega %.1f rad/s, R %.4f m, tip speed %.2f m/s, one revolution %.2f ms, dx %.2f mm, n_sub %u, W/4 %.4f N\n",
                static_cast<double>(*omega), radius, tip_speed, rev * 1000.0, static_cast<double>(dx) * 1000.0, n_sub, static_cast<double>(target));
    std::fflush(stdout);

    auto ctx = spade::compute::VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;

    for (const double cells : l105g_cells()) {
        const float h = static_cast<float>(cells * static_cast<double>(dx) / tip_speed);
        const af::GridShape shape{static_cast<uint32_t>(std::lround(0.40 / static_cast<double>(dx))),
                                  static_cast<uint32_t>(std::lround(l105g_number("SPADE_L105_DEPTH", 0.55) / static_cast<double>(dx))),
                                  static_cast<uint32_t>(std::lround(0.40 / static_cast<double>(dx))), dx};
        af::DomainBc bc;
        auto grid = af::make_grid(shape, bc);
        ASSERT_TRUE(grid.has_value()) << grid.error().context;
        af::FluidConfig cfg;
        cfg.rho = kRho;
        cfg.h = h;
        if (const char* nv = l105g_env("SPADE_L105_NV"); nv != nullptr) cfg.n_v = static_cast<uint32_t>(std::strtoul(nv, nullptr, 10));
        cfg.sgs_cs = static_cast<float>(l105g_number("SPADE_L105_CS", 0.0));
        cfg.nu = static_cast<float>(l105g_number("SPADE_L105_NU", 0.0));
        af::LineRotor rotor;
        rotor.centre = glm::vec3(0.5f * static_cast<float>(shape.nx) * dx, static_cast<float>(shape.ny) * dx - 0.20f,
                                 0.5f * static_cast<float>(shape.nz) * dx);
        rotor.axis = glm::vec3(0.0f, 1.0f, 0.0f);
        rotor.omega = *omega;
        rotor.blade = &prop->blade;
        rotor.polar = &prop->polar;
        rotor.tip_correction = true;
        const uint32_t elements = static_cast<uint32_t>(af::line_elements(rotor, dx, h, n_sub).size());
        const ag::LineSetup line_setup{.polar = &prop->polar, .elements = elements, .n_sub = n_sub, .tip_correction = true, .binned = true, .rotors = 1u};
        auto gpu = ag::GpuFluid::create(**ctx, *grid, bc, cfg, nullptr, 64u, &line_setup, nullptr);
        ASSERT_TRUE(gpu.has_value()) << gpu.error().context;

        const uint32_t total = static_cast<uint32_t>(std::lround(t_end / static_cast<double>(h)));
        std::vector<double> thrust;
        thrust.reserve(total);
        std::vector<af::LineRotor> lines(1, rotor);
        const uint32_t row = static_cast<uint32_t>(std::lround(rotor.centre.y / dx));
        const uint32_t row_below = row - static_cast<uint32_t>(std::lround(0.05f / dx));
        // The mean downward air over the cells inside the disc's radius, on the v faces of row j (the grid read back from the device).
        const auto disc_mean = [&](uint32_t j) {
            double sum = 0.0;
            uint32_t n = 0;
            for (uint32_t k = 0; k < shape.nz; ++k) {
                for (uint32_t i = 0; i < shape.nx; ++i) {
                    const float x = (static_cast<float>(i) + 0.5f) * dx - rotor.centre.x;
                    const float z = (static_cast<float>(k) + 0.5f) * dx - rotor.centre.z;
                    if (x * x + z * z > static_cast<float>(radius * radius)) continue;
                    sum -= static_cast<double>(grid->v[af::v_index(shape, i, j, k)]);
                    ++n;
                }
            }
            return n != 0u ? sum / static_cast<double>(n) : 0.0;
        };
        std::printf("[ L105g   ] %.2f cells a step: h %.4f ms, %u steps to %.3f s, box %ux%ux%u, %u line elements, n_v %u\n", cells,
                    static_cast<double>(h) * 1000.0, total, t_end, shape.nx, shape.ny, shape.nz, elements, cfg.n_v);
        std::fflush(stdout);
        double psi = 0.0;
        double wall_steps = 0.0;
        const uint32_t every = static_cast<uint32_t>(l105g_number("SPADE_L105_EVERY", 0.0));
        double impulse = 0.0;
        double res_max = 0.0;
        uint64_t iters = 0;
        uint32_t health_steps = 0, capped = 0, unconverged = 0, stalled = 0;
        const auto t0 = std::chrono::steady_clock::now();
        const uint32_t rev_steps = static_cast<uint32_t>(std::lround(rev / static_cast<double>(h)));
        for (uint32_t step = 1; step <= total; ++step) {
            lines[0].psi = static_cast<float>(psi);
            const auto ts = std::chrono::steady_clock::now();
            const auto health = (*gpu)->step({}, lines);
            wall_steps += std::chrono::duration<double>(std::chrono::steady_clock::now() - ts).count();
            ASSERT_TRUE(health.has_value()) << "step " << step << ": " << health.error().context;
            ASSERT_EQ(health->fault, af::FluidFault::none) << "step " << step;
            thrust.push_back(static_cast<double>((*gpu)->line_reports().at(0).thrust));
            impulse += thrust.back() * static_cast<double>(h);
            res_max = std::max(res_max, static_cast<double>(health->pressure_residual));
            iters += health->pressure_iterations;
            ++health_steps;
            capped += health->capped ? 1u : 0u;
            unconverged += health->unconverged ? 1u : 0u;
            stalled += health->stalled ? 1u : 0u;
            if (every != 0u && step % every == 0u) {
                const auto read = (*gpu)->read(*grid);
                ASSERT_TRUE(read.has_value()) << read.error().context;
                double down = 0.0, p_sum = 0.0, p_max = 0.0, v_min = 1.0e30, v_max = -1.0e30;
                std::size_t v_min_at = 0, v_max_at = 0;
                uint32_t nans = 0;
                for (std::size_t c = 0; c < grid->v.size(); ++c) {
                    const float v = grid->v[c];
                    if (std::isnan(v)) { ++nans; continue; }
                    down -= static_cast<double>(v);
                    if (v < v_min) { v_min = v; v_min_at = c; }
                    if (v > v_max) { v_max = v; v_max_at = c; }
                }
                for (const float pv : grid->p) {
                    p_sum += static_cast<double>(pv);
                    p_max = std::max(p_max, std::fabs(static_cast<double>(pv)));
                }
                const double dv = static_cast<double>(dx) * dx * dx * static_cast<double>(kRho);
                std::printf("[ L105g   ] step %u (t %.4f s; host psi %.4f rad against %.4f from step x h x omega): downflow in the plane %.3f, 0.05 m under %.3f, "
                            "0.15 m under %.3f m/s; box momentum %.5f against the impulse %.5f (ratio %.4f); v from %.2f (cell %zu) to %.2f (cell %zu) m/s, "
                            "NaN %u; p mean %.4f, largest %.2f Pa; projection since the last print: %u steps, largest residual %.2e, %.1f iterations a step, "
                            "capped %u, unconverged %u, stalled %u\n",
                            step, static_cast<double>(step) * static_cast<double>(h), psi,
                            std::fmod(static_cast<double>(step) * static_cast<double>(h) * static_cast<double>(*omega), 2.0 * kPi), disc_mean(row),
                            disc_mean(row_below), disc_mean(row - static_cast<uint32_t>(std::lround(0.15f / dx))), down * dv, impulse,
                            impulse > 0.0 ? down * dv / impulse : 0.0, static_cast<double>(v_min), v_min_at, static_cast<double>(v_max), v_max_at, nans,
                            p_sum / static_cast<double>(grid->p.size()), p_max, health_steps, res_max,
                            health_steps != 0u ? static_cast<double>(iters) / static_cast<double>(health_steps) : 0.0, capped, unconverged, stalled);
                std::fflush(stdout);
                res_max = 0.0;
                iters = 0;
                health_steps = capped = unconverged = stalled = 0;
            }
            psi = std::fmod(psi + static_cast<double>(*omega) * static_cast<double>(h), 2.0 * kPi);
            for (const double t : sample_t) {
                if (t > t_end + 1.0e-9 || step != static_cast<uint32_t>(std::lround(t / static_cast<double>(h)))) continue;
                const uint32_t from = step > rev_steps ? step - rev_steps : 0u;
                const double mean = std::accumulate(thrust.begin() + from, thrust.end(), 0.0) / static_cast<double>(thrust.size() - from);
                char text[420];
                int len = std::snprintf(text, sizeof text, "[ L105g   ]   %.3f s (step %u, wall %.0f s, %.1f ms a step): thrust %.3f N, mean over the last revolution %.3f N", t,
                                        step, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
                                        wall_steps / static_cast<double>(step) * 1000.0, thrust.back(), mean);
                if (t == 0.05 || step == total) {
                    const auto read = (*gpu)->read(*grid);
                    ASSERT_TRUE(read.has_value()) << read.error().context;
                    const double v_mt = std::sqrt(mean / (2.0 * static_cast<double>(kRho) * area));
                    const double in_plane = disc_mean(row);
                    std::snprintf(text + len, sizeof text - static_cast<std::size_t>(len),
                                  "; disc downflow %.3f m/s in its plane (%.3f of the mean thrust's momentum-theory %.3f m/s), %.3f m/s 0.05 m under it", in_plane,
                                  in_plane / v_mt, v_mt, disc_mean(row_below));
                }
                std::printf("%s\n", text);
                std::fflush(stdout);
            }
        }
        std::printf("[ L105g   ] %.2f cells a step done: wall %.0f s, %.1f ms a step\n", cells,
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), wall_steps / static_cast<double>(total) * 1000.0);
        std::fflush(stdout);
    }
}

}  // namespace
