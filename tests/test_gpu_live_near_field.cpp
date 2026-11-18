// ---------------------------------------------------------------------------
// test_gpu_live_near_field.cpp -- L86i, option A (the lead, 2026-11-17): the camera quadcopter on a UNIFORM 2 mm CROPPED NEAR FIELD,
// tests/d1_case.hpp's kD1NearGrid (272 x 232 x 272 = 17.2 M cells, 0.544 x 0.464 x 0.544 m: the drone 0.177 m below the top face and
// 0.29 m of wake below it), its rotors as actuator lines and the dye at the blade tips, on the device through LiveRun.
//
//   AirflowNearField.TheTwoMillimetreGridIsWithinTheCapWithItsSolids   check_grid(kD1NearGrid, the case's faces, with solids): measured
//                                                                      bytes against kGridByteCap, printed; kGridByteCap is spec law
//   GpuLiveNearField.ThePhantomFliesOnTheTwoMillimetreNearField        opt-in SPADE_NEAR_FIELD=1 (GPU, minutes): LiveRun::start() on the
//                                                                      grid with lines and dye, N steps (SPADE_NEAR_STEPS, 10): no
//                                                                      fault, the air moves, the dye is in the frame; the device's
//                                                                      memory and the step's ms printed
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#include "analysis/live_run.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/flow_volume.hpp"
#include "d1_case.hpp"
#include "flow_still.hpp"
#include "gpu_skip.hpp"
#include "phantom_case.hpp"
#include "x500_model.hpp"
#include "physics/airflow/dye.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"
#include "spike/airflow/live_fluid.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
using flow_still::env_cstr;

constexpr double kGiB = 1024.0 * 1024.0 * 1024.0;

TEST(AirflowNearField, TheTwoMillimetreGridIsWithinTheCapWithItsSolids) {
    const af::GridShape grid = spade::testing::kD1NearGrid;
    const auto made = spade::testing::phantom_case(grid);
    ASSERT_TRUE(made.has_value()) << made.error().context;
    const af::DomainBc& bc = made->resolved.domain;
    const uint64_t fluid = af::fluid_bytes(grid, bc);
    const uint64_t solids = af::solid_bytes(grid, bc);
    std::printf("[ near    ] %ux%ux%u at %.0f mm: %.1f M cells; fluid_bytes %.3f GiB (%.0f B a cell), solid_bytes %.3f GiB, together "
                "%.3f GiB against kGridByteCap %.3f GiB\n",
                grid.nx, grid.ny, grid.nz, static_cast<double>(grid.dx) * 1000.0,
                static_cast<double>(grid.nx) * grid.ny * grid.nz / 1.0e6, static_cast<double>(fluid) / kGiB,
                static_cast<double>(fluid) / (static_cast<double>(grid.nx) * grid.ny * grid.nz), static_cast<double>(solids) / kGiB,
                static_cast<double>(fluid + solids) / kGiB, static_cast<double>(af::kGridByteCap) / kGiB);
    const auto ok = af::check_grid(grid, bc, true);
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
    EXPECT_EQ(af::kGridByteCap, uint64_t{4} << 30) << "the cap is spec law (the lead, 2026-11-17): a test does not move it";
}

TEST(GpuLiveNearField, ThePhantomFliesOnTheTwoMillimetreNearField) {
    if (env_cstr("SPADE_NEAR_FIELD") == nullptr) GTEST_SKIP() << "opt-in: SPADE_NEAR_FIELD=1 (GPU, minutes)";
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    // SPADE_NEAR_DRONE=x500 (L86k): the X500 on its 2.5 mm grid with the kit's 10-inch prop; else the phantom on the 2 mm near field.
    const bool x500 = env_cstr("SPADE_NEAR_DRONE") != nullptr && std::string(env_cstr("SPADE_NEAR_DRONE")) == "x500";
    const af::GridShape grid = x500 ? spade::testing::kD1X500Grid : spade::testing::kD1NearGrid;
    const auto made = x500 ? spade::testing::still_air_case(spade::testing::x500::data_dir() / spade::testing::x500::kModelFile, "x500", grid, 0.0002f,
                                                                spade::testing::d1_x500_hold())
                           : spade::testing::phantom_case(grid);
    ASSERT_TRUE(made.has_value()) << made.error().context;

    const std::filesystem::path prop_file = x500 ? spade::testing::x500::data_dir() / spade::testing::x500::kPropFile
                                                 : std::filesystem::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_apce_10x5.prop.yaml";
    std::ifstream in(prop_file, std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto part = spade::scene::prop_from_yaml(text);
    ASSERT_TRUE(part.has_value()) << part.error().context;
    const auto prop = spade::scene::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    an::LiveLines lines;
    lines.blade = prop->blade;
    lines.polar = prop->polar;
    lines.n_sub = 8;

    auto ctx = spade::compute::VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    const double before_mib = static_cast<double>((*ctx)->memory_budget().usage) / (1024.0 * 1024.0);
    const ag::DyeSetup dye{af::DyeRelease{.rate = 0.0f}, af::TipRelease{.gain = 1.5f}};
    const auto t_start = std::chrono::steady_clock::now();
    spade::Result<an::LiveRun> run =
        an::LiveRun::start(made->resolved, made->model, ag::gpu_live_fluid(**ctx, 64u, 0.17f, dye), &lines);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const double start_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    const double after_mib = static_cast<double>((*ctx)->memory_budget().usage) / (1024.0 * 1024.0);
    std::printf("[ near    ] start() %.1f s; device memory %.0f MiB before, %.0f MiB after (+%.2f GiB); %s\n", start_s, before_mib,
                after_mib, (after_mib - before_mib) / 1024.0, run->run_label().c_str());
    std::fflush(stdout);

    const uint32_t n = env_cstr("SPADE_NEAR_STEPS") != nullptr ? static_cast<uint32_t>(std::strtoul(env_cstr("SPADE_NEAR_STEPS"), nullptr, 10)) : 10u;
    const an::Controls armed{true, made->trim.hover, 0.0f, 0.0f, 0.0f};
    const auto t_steps = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < n; ++i) {
        const auto s = run->step(armed);
        ASSERT_TRUE(s.has_value()) << "step " << i + 1 << ": " << s.error().context;
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_steps).count() / n;
    spade::FlowVolume out;
    const auto frame = run->advance_steps(armed, 1, out);
    ASSERT_TRUE(frame.has_value()) << frame.error().context;
    float fastest = 0.0f;
    for (int c = 0; c < 3; ++c) {
        fastest = std::max({fastest, std::fabs(out.header.channel_min[c]), std::fabs(out.header.channel_max[c])});
    }
    std::printf("[ near    ] %u steps: %.0f ms a step; fastest %.2f m/s, dye max %.3f; device memory now %.0f MiB\n", n, ms,
                static_cast<double>(fastest), static_cast<double>(out.header.channel_max[spade::kFlowDye]),
                static_cast<double>((*ctx)->memory_budget().usage) / (1024.0 * 1024.0));
    EXPECT_GT(fastest, 0.0f) << "the lines move the air";
    EXPECT_GT(out.header.channel_max[spade::kFlowDye], 0.0f) << "the tips' dye is in the frame";
}

}  // namespace
