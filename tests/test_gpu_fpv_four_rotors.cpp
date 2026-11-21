// ---------------------------------------------------------------------------
// test_gpu_fpv_four_rotors.cpp -- the OpenFrame-5F's thrust gate with all FOUR rotors in the film box (the lead, 2026-11-21): the front and rear rotors of
// the stretched-X frame are 12.1 mm apart (hubs +-69.534 mm along, +-89.000 mm across, R 0.0635 m), so does one rotor's wake change the next's thrust?
// Four actuator lines in open air on kD1FpvGrid (252 x 237 x 284 at 1.25 mm), the rotors' plane 1.5 R under the top face, no frame and no solids (the
// rotors alone: what the frame adds is the kit's own check). Each rotor's thrust is reported as the mean over the last revolution against W/4, the
// front/rear difference, and the disc's mean downflow in its plane against momentum theory.
//
//   GpuFpvFourRotors.TheFourRotorsThrustAgainstTheWeightInTheFilmBox   opt-in: SPADE_FPV4_GPU=1 (GPU, about 10 min)
//
// Knobs: SPADE_FPV4_PROP (a library prop part, default uiuc_mit_5x4), SPADE_FPV4_OMEGA (rad/s, default 914.8 = 8,735 rpm), SPADE_FPV4_MASS (kg, the
// airframe whose weight W/4 is taken against, default 0.5), SPADE_FPV4_T (the last time, s, default 0.05), SPADE_FPV4_STEP_US (the step, us, default 25),
// SPADE_FPV4_NSUB (8), SPADE_FPV4_CS (the Smagorinsky constant, 0.17 the film's), SPADE_FPV4_NU (1.4607e-5 m^2/s), SPADE_FPV4_EVERY (a step count: print the
// four thrusts every that many steps).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numeric>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "d1_case.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"
#include "spike/airflow/step_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;

[[nodiscard]] const char* fpv4_env(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] double fpv4_number(const char* name, double fallback) {
    const char* v = fpv4_env(name);
    return v != nullptr ? std::strtod(v, nullptr) : fallback;
}

// The OpenFrame-5F's motor axes (Interface, from Arm.step): along (the kit's +X, the grid's x) and across (+Z), m. Front rotors first, then rear;
// spins alternate on the diagonals (a quad's X layout: the same spin on a diagonal pair).
struct Hub {
    float along;
    float across;
    float spin;
    const char* name;
};
constexpr std::array<Hub, 4> kHubs{{{+0.069534f, +0.089000f, +1.0f, "front right"},
                                    {+0.069534f, -0.089000f, -1.0f, "front left"},
                                    {-0.069534f, -0.089000f, +1.0f, "rear left"},
                                    {-0.069534f, +0.089000f, -1.0f, "rear right"}}};

TEST(GpuFpvFourRotors, TheFourRotorsThrustAgainstTheWeightInTheFilmBox) {
    if (fpv4_env("SPADE_FPV4_GPU") == nullptr) GTEST_SKIP() << "opt-in: SPADE_FPV4_GPU=1 (GPU, about 10 min)";
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const std::string prop_name = fpv4_env("SPADE_FPV4_PROP") != nullptr ? fpv4_env("SPADE_FPV4_PROP") : "uiuc_mit_5x4";
    const std::filesystem::path prop_file = std::filesystem::path(SPADE_ASSETS_DIR) / "library/parts" / (prop_name + ".prop.yaml");
    std::ifstream in(prop_file, std::ios::binary);
    ASSERT_TRUE(in.good()) << "cannot read " << prop_file.string();
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto part = spade::scene::prop_from_yaml(text);
    ASSERT_TRUE(part.has_value()) << part.error().context;
    const auto prop = spade::scene::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;

    constexpr float kRho = 1.225f;
    constexpr double kPi = 3.14159265358979;
    const af::GridShape shape = spade::testing::kD1FpvGrid;
    const float dx = shape.dx;
    const double radius = static_cast<double>(prop->blade.radius);
    const double area = kPi * radius * radius;
    const float omega = static_cast<float>(fpv4_number("SPADE_FPV4_OMEGA", 914.8));
    const double mass = fpv4_number("SPADE_FPV4_MASS", 0.5);
    const double w4 = mass * 9.80665 / 4.0;
    const double tip_speed = static_cast<double>(omega) * radius;
    const double rev = 2.0 * kPi / static_cast<double>(omega);
    const float h = static_cast<float>(1.0e-6 * fpv4_number("SPADE_FPV4_STEP_US", 25.0));
    const uint32_t n_sub = static_cast<uint32_t>(fpv4_number("SPADE_FPV4_NSUB", 8.0));
    const double t_end = fpv4_number("SPADE_FPV4_T", 0.05);
    const uint32_t every = static_cast<uint32_t>(fpv4_number("SPADE_FPV4_EVERY", 0.0));
    std::printf("[ fpv4    ] %s: omega %.1f rad/s (%.0f rpm), R %.4f m, tip speed %.2f m/s, one revolution %.2f ms, h %.1f us (%.2f tip cells), n_sub %u, "
                "W/4 %.4f N for %.3f kg; box %ux%ux%u at %.2f mm\n",
                prop_name.c_str(), static_cast<double>(omega), static_cast<double>(omega) * 60.0 / (2.0 * kPi), radius, tip_speed, rev * 1000.0,
                static_cast<double>(h) * 1.0e6, tip_speed * static_cast<double>(h) / static_cast<double>(dx), n_sub, w4, mass, shape.nx, shape.ny,
                shape.nz, static_cast<double>(dx) * 1000.0);
    std::fflush(stdout);

    auto ctx = spade::compute::VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    af::DomainBc bc;
    auto grid = af::make_grid(shape, bc);
    ASSERT_TRUE(grid.has_value()) << grid.error().context;
    af::FluidConfig cfg;
    cfg.rho = kRho;
    cfg.h = h;
    cfg.sgs_cs = static_cast<float>(fpv4_number("SPADE_FPV4_CS", 0.17));
    cfg.nu = static_cast<float>(fpv4_number("SPADE_FPV4_NU", 1.4607e-5));
    // The rotors' plane 1.5 R under the top face, on a cell row (d1_fpv_hold's rule with the hubs at the plane).
    const float plane_y = static_cast<float>(shape.ny) * dx - 1.5f * static_cast<float>(radius);
    std::vector<af::LineRotor> lines(kHubs.size());
    for (std::size_t k = 0; k < kHubs.size(); ++k) {
        af::LineRotor& r = lines[k];
        r.centre = glm::vec3(0.5f * static_cast<float>(shape.nx) * dx + kHubs[k].along, plane_y,
                             0.5f * static_cast<float>(shape.nz) * dx + kHubs[k].across);
        r.axis = glm::vec3(0.0f, 1.0f, 0.0f);
        r.spin = kHubs[k].spin;
        r.omega = omega;
        r.blade = &prop->blade;
        r.polar = &prop->polar;
        r.tip_correction = true;
    }
    uint32_t elements = 0;
    for (const af::LineRotor& r : lines) elements += static_cast<uint32_t>(af::line_elements(r, dx, h, n_sub).size());
    const ag::LineSetup line_setup{.polar = &prop->polar, .elements = elements, .n_sub = n_sub, .tip_correction = true, .binned = true,
                                   .rotors = static_cast<uint32_t>(lines.size())};
    auto gpu = ag::GpuFluid::create(**ctx, *grid, bc, cfg, nullptr, 64u, &line_setup, nullptr);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    const double usage_mib = static_cast<double>((*ctx)->memory_budget().usage) / (1024.0 * 1024.0);
    const uint32_t total = static_cast<uint32_t>(std::lround(t_end / static_cast<double>(h)));
    const uint32_t rev_steps = static_cast<uint32_t>(std::lround(rev / static_cast<double>(h)));
    std::printf("[ fpv4    ] %u line elements, %u steps to %.3f s, a revolution is %u steps; device memory in use after create %.0f MiB\n", elements, total,
                t_end, rev_steps, usage_mib);
    std::fflush(stdout);

    std::array<std::vector<double>, 4> thrust;
    for (auto& t : thrust) t.reserve(total);
    // The mean downward air over the v faces inside a disc's radius, in the row of the rotors' plane (the grid read back from the device).
    const uint32_t row = static_cast<uint32_t>(std::lround(plane_y / dx));
    const auto disc_mean = [&](const af::LineRotor& r, uint32_t j) {
        double sum = 0.0;
        uint32_t n = 0;
        for (uint32_t k = 0; k < shape.nz; ++k) {
            for (uint32_t i = 0; i < shape.nx; ++i) {
                const float x = (static_cast<float>(i) + 0.5f) * dx - r.centre.x;
                const float z = (static_cast<float>(k) + 0.5f) * dx - r.centre.z;
                if (x * x + z * z > static_cast<float>(radius * radius)) continue;
                sum -= static_cast<double>(grid->v[af::v_index(shape, i, j, k)]);
                ++n;
            }
        }
        return n != 0u ? sum / static_cast<double>(n) : 0.0;
    };
    const auto mean_last_rev = [&](const std::vector<double>& t) {
        const std::size_t from = t.size() > rev_steps ? t.size() - rev_steps : 0u;
        return std::accumulate(t.begin() + static_cast<std::ptrdiff_t>(from), t.end(), 0.0) / static_cast<double>(t.size() - from);
    };
    const auto report = [&](const char* tag, uint32_t step, bool downflow) {
        std::array<double, 4> m{};
        for (std::size_t k = 0; k < 4; ++k) m[k] = mean_last_rev(thrust[k]);
        const double front = 0.5 * (m[0] + m[1]);
        const double rear = 0.5 * (m[2] + m[3]);
        const double all = 0.25 * (m[0] + m[1] + m[2] + m[3]);
        std::printf("[ fpv4    ] %s step %u (t %.4f s): last-revolution thrust N, FR %.4f FL %.4f RL %.4f RR %.4f (of W/4: %.3f %.3f %.3f %.3f); mean %.4f (%.3f of W/4); "
                    "front %.4f rear %.4f, rear - front %+.4f N (%+.1f%%)\n",
                    tag, step, static_cast<double>(step) * static_cast<double>(h), m[0], m[1], m[2], m[3], m[0] / w4, m[1] / w4, m[2] / w4, m[3] / w4, all,
                    all / w4, front, rear, rear - front, 100.0 * (rear - front) / front);
        if (downflow) {
            const auto read = (*gpu)->read(*grid);
            ASSERT_TRUE(read.has_value()) << read.error().context;
            for (std::size_t k = 0; k < 4; ++k) {
                const double v_mt = std::sqrt(m[k] / (2.0 * static_cast<double>(kRho) * area));
                const double in_plane = disc_mean(lines[k], row);
                std::printf("[ fpv4    ]   %-11s disc downflow %.3f m/s in its plane (%.3f of its thrust's momentum-theory %.3f m/s), %.3f m/s 0.05 m under it\n",
                            kHubs[k].name, in_plane, in_plane / v_mt, v_mt, disc_mean(lines[k], row - static_cast<uint32_t>(std::lround(0.05f / dx))));
            }
        }
        std::fflush(stdout);
    };

    double psi = 0.0;
    double wall_steps = 0.0;
    uint32_t capped = 0, unconverged = 0, stalled = 0;
    double res_max = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    const double sample_t[] = {0.02, 0.035, 0.05, 0.075, 0.1};
    for (uint32_t step = 1; step <= total; ++step) {
        for (af::LineRotor& r : lines) r.psi = static_cast<float>(psi);
        const auto ts = std::chrono::steady_clock::now();
        const auto health = (*gpu)->step({}, lines);
        wall_steps += std::chrono::duration<double>(std::chrono::steady_clock::now() - ts).count();
        ASSERT_TRUE(health.has_value()) << "step " << step << ": " << health.error().context;
        ASSERT_EQ(health->fault, af::FluidFault::none) << "step " << step;
        const auto& reports = (*gpu)->line_reports();
        ASSERT_EQ(reports.size(), 4u);
        for (std::size_t k = 0; k < 4; ++k) thrust[k].push_back(static_cast<double>(reports[k].thrust));
        res_max = std::max(res_max, static_cast<double>(health->pressure_residual));
        capped += health->capped ? 1u : 0u;
        unconverged += health->unconverged ? 1u : 0u;
        stalled += health->stalled ? 1u : 0u;
        psi = std::fmod(psi + static_cast<double>(omega) * static_cast<double>(h), 2.0 * kPi);
        if (every != 0u && step % every == 0u) report("every", step, false);
        for (const double t : sample_t) {
            if (t > t_end + 1.0e-9 || step != static_cast<uint32_t>(std::lround(t / static_cast<double>(h)))) continue;
            report("sample", step, step == total || t == 0.05);
            std::printf("[ fpv4    ]   wall %.0f s, %.1f ms a step; projection since the start: largest residual %.2e, capped %u, unconverged %u, stalled %u\n",
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
                        wall_steps / static_cast<double>(step) * 1000.0, res_max, capped, unconverged, stalled);
            std::fflush(stdout);
        }
    }
    std::printf("[ fpv4    ] done: wall %.0f s, %.1f ms a step; device memory in use now %.0f MiB\n",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), wall_steps / static_cast<double>(total) * 1000.0,
                static_cast<double>((*ctx)->memory_budget().usage) / (1024.0 * 1024.0));
    std::fflush(stdout);
}

}  // namespace
