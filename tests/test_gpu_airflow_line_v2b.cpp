// ---------------------------------------------------------------------------
// test_gpu_airflow_line_v2b.cpp -- L61/L72c's V2b: the actuator line on the GPU hovering the UIUC 10x5 in a grid box, against UIUC's
// MEASURED static point (C_T 0.09636, C_Q 0.005870 at 5000 rpm; tests/library_props.gen.hpp). The kernel's width is a PHYSICAL size
// (mm), the resolution R/dx a separate choice: the tune set of the width-choice rule
// (.superpowers/sdd/2026-11-10-wake/line-width-choice-rule.md: the smallest width with C_Q and C_T both within 10% of the measured).
// A tune run is not a band: it prints the record's numbers (signed errors against UIUC, the window drift) and asserts only that the
// run settled. The held-out checks (the same prop at other rpm, a second prop, V3, V4) are other runs with the width fixed.
//
// Opt-in, GPU only (the CPU's box at R/dx 32 is about 80 minutes):
//   SPADE_V2B_RDX=<R/dx>  (32 is the rule's: dx 3.97 mm)        SPADE_V2B_EPS_MM10=<width in tenths of a mm> (80 is 8.0 mm; the width in
//   cells, mm / dx, must be in [1, 3])    SPADE_V2B_NSUB (sub-azimuths, default 8)    SPADE_V2B_RPM (default 5000)
//   SPADE_V2B_STEPS (a short run for timing: no settled check)
// The box is the V2 box in units of R: 5.25 R across, the rotor 1.7 R under the top, 6.6 R of air below it; every face open, still air;
// the step h = 0.38 ms * dx / 4 mm (D1's, CFL held). The window: the mean over 10 R/v_h starting at 20 R/v_h, v_h from the UIUC static
// thrust; UNSETTLED when the window after the verdict window differs (thrust 2.5%, power 3.75%).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <numbers>
#include <string>
#include <vector>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"
#include "library_props.gen.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/line.hpp"
#include "ke_share.hpp"
#include "spike/airflow/line_chain.hpp"
#include "spike/airflow/step_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
namespace lp = spade::test::library_props;

constexpr double kInch = 0.0254;
constexpr float kRho = 1.225f;

uint32_t env_u32(const char* name, uint32_t fallback) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* v = std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    return v == nullptr ? fallback : static_cast<uint32_t>(std::strtoul(v, nullptr, 10));
}

double env_double(const char* name, double fallback) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* v = std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    return v == nullptr ? fallback : std::strtod(v, nullptr);
}

// L84's smoothness check: the grid-scale energy under the rotor and a slice still of speed and of the out-of-plane vorticity
// (SPADE_V2B_CS100 sets the subgrid Cs times 100; SPADE_V2B_SLICE=<path prefix> writes <prefix>-speed.ppm and -vort.ppm).
const char* env_cstr(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// A cell-centre velocity from the faces.
struct CellVel {
    float u, v, w;
};

CellVel cell_velocity(const af::MacGrid& g, uint32_t i, uint32_t j, uint32_t k) {
    const af::GridShape& s = g.shape;
    return {0.5f * (g.u[af::u_index(s, i, j, k)] + g.u[af::u_index(s, i + 1u, j, k)]),
            0.5f * (g.v[af::v_index(s, i, j, k)] + g.v[af::v_index(s, i, j + 1u, k)]),
            0.5f * (g.w[af::w_index(s, i, j, k)] + g.w[af::w_index(s, i, j, k + 1u)])};
}

// A blue-to-yellow ramp over t in [0, 1].
void ramp(float t, unsigned char* rgb) {
    t = std::min(1.0f, std::max(0.0f, t));
    rgb[0] = static_cast<unsigned char>(255.0f * std::min(1.0f, 1.6f * t));
    rgb[1] = static_cast<unsigned char>(255.0f * std::min(1.0f, 1.2f * t));
    rgb[2] = static_cast<unsigned char>(255.0f * std::min(1.0f, std::max(0.0f, 0.55f + 0.9f * t - 1.4f * t * t)));
}

// The vertical plane through the rotor's axis (z = k), x across and y up, magnified `zoom` times: speed over 0..vmax and
// |omega_z| over 0..vort_max, two binary PPMs.
void write_slices(const af::MacGrid& g, uint32_t k, float vmax, float vort_max, uint32_t zoom,
                  const std::string& prefix) {
    const af::GridShape& s = g.shape;
    const uint32_t wpx = s.nx * zoom, hpx = s.ny * zoom;
    std::vector<unsigned char> speed(std::size_t{wpx} * hpx * 3u), vort(std::size_t{wpx} * hpx * 3u);
    for (uint32_t py = 0; py < hpx; ++py) {
        const uint32_t j = s.ny - 1u - py / zoom;  // y up
        for (uint32_t px = 0; px < wpx; ++px) {
            const uint32_t i = px / zoom;
            const CellVel c = cell_velocity(g, i, j, k);
            float vz = 0.0f;
            if (i > 0u && i + 1u < s.nx && j > 0u && j + 1u < s.ny) {
                const CellVel xp = cell_velocity(g, i + 1u, j, k), xm = cell_velocity(g, i - 1u, j, k);
                const CellVel yp = cell_velocity(g, i, j + 1u, k), ym = cell_velocity(g, i, j - 1u, k);
                vz = ((xp.v - xm.v) - (yp.u - ym.u)) / (2.0f * s.dx);
            }
            const std::size_t at = (std::size_t{py} * wpx + px) * 3u;
            ramp(std::sqrt(c.u * c.u + c.v * c.v + c.w * c.w) / vmax, &speed[at]);
            ramp(std::abs(vz) / vort_max, &vort[at]);
        }
    }
    const auto save = [&](const std::string& name, const std::vector<unsigned char>& pixels) {
        std::ofstream f(name, std::ios::binary);
        f << "P6" << '\n' << wpx << ' ' << hpx << '\n' << "255" << '\n';
        f.write(reinterpret_cast<const char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    };
    save(prefix + "-speed.ppm", speed);
    save(prefix + "-vort.ppm", vort);
}

// The library's props the runs use: the tune prop (SPADE_V2B_PROP unset or "apce": uiuc_apce_10x5, D 10 in), the transfer prop
// ("mit": uiuc_mit_5x4, D 5 in, 3 blades, measured at 6057 rpm), V3's stand-in ("v3": the 10x5 at D 9.4 in) and D1's blade
// stand-in ("d1": triblade_6inch_scaled at D 0.1524 m; its reference is the chain's k_T w^2, given by SPADE_V2B_CT_E6).
struct Chosen {
    const lp::Prop* prop = nullptr;
    double diameter = 0.0;
};

Chosen chosen_prop() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* v = std::getenv("SPADE_V2B_PROP");
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    const std::string want = v == nullptr ? "apce" : v;
    for (const lp::Prop& p : lp::kProps) {
        if (want == "apce" && std::string(p.id) == "uiuc_apce_10x5") return {&p, 10.0 * kInch};
        if (want == "mit" && std::string(p.id) == "uiuc_mit_5x4") return {&p, 5.0 * kInch};
        if (want == "v3" && std::string(p.id) == "uiuc_apce_10x5") return {&p, 9.4 * kInch};
        if (want == "d1" && std::string(p.id) == "triblade_6inch_scaled") return {&p, 0.1524};
        // L72k's B1, the polar fits: the other props with a static point (the run's D is UIUC's, the geometry is per R)
        if (want == "apce9" && std::string(p.id) == "uiuc_apce_9x4.5") return {&p, 9.0 * kInch};
        if (want == "cf" && std::string(p.id) == "uiuc_cfnq_45t1") return {&p, 0.04699};
    }
    return {};
}

af::BladeShape blade_of(const lp::Prop& p, double diameter) {
    af::BladeShape b;
    b.blades = static_cast<uint32_t>(p.blades);
    b.radius = static_cast<float>(0.5 * diameter);
    for (std::size_t i = 0; i < p.r_over_r.size(); ++i) {
        b.stations.push_back({static_cast<float>(p.r_over_r[i] * 0.5 * diameter),
                              static_cast<float>(p.c_over_r[i] * 0.5 * diameter), static_cast<float>(p.beta_deg[i])});
    }
    return b;
}

// The blade's chord at 0.75 R, linear between stations: "the mean chord" the same-fraction width is a share of.
double chord_at_three_quarters(const af::BladeShape& b) {
    const double r = 0.75 * static_cast<double>(b.radius);
    for (std::size_t i = 1; i < b.stations.size(); ++i) {
        if (r <= static_cast<double>(b.stations[i].r)) {
            const double t = (r - b.stations[i - 1].r) / (static_cast<double>(b.stations[i].r) - b.stations[i - 1].r);
            return b.stations[i - 1].chord + t * (static_cast<double>(b.stations[i].chord) - b.stations[i - 1].chord);
        }
    }
    return b.stations.back().chord;
}

double static_value(std::span<const std::array<double, 2>> table) {
    for (const auto& row : table) {
        if (std::abs(row[0]) < 1e-9) return row[1];
    }
    return 0.0;
}

TEST(GpuAirflowLineHover, V2bTheUiuc10x5OnTheGpu) {
    const uint32_t rdx = env_u32("SPADE_V2B_RDX", 0);
    if (rdx == 0) {
        GTEST_SKIP() << "opt-in: SPADE_V2B_RDX=32 and SPADE_V2B_EPS_MM10=<width in tenths of a mm> (GPU, minutes)";
    }
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const uint32_t n_sub = env_u32("SPADE_V2B_NSUB", 8);
    const uint32_t steps_override = env_u32("SPADE_V2B_STEPS", 0);
    const Chosen pick = chosen_prop();
    ASSERT_NE(pick.prop, nullptr) << "SPADE_V2B_PROP is apce, mit, v3, d1, apce9 or cf";
    const lp::Prop& prop = *pick.prop;
    const double kD = pick.diameter;
    const double rpm = static_cast<double>(env_u32("SPADE_V2B_RPM", static_cast<uint32_t>(prop.run_rpm)));
    const af::BladeShape blade = blade_of(prop, kD);
    ASSERT_TRUE(af::check_blade(blade).has_value());
    // The width: in mm, or as a share of the chord at 0.75 R (SPADE_V2B_EPS_CHORD100: 60 is 0.60 of it).
    const double chord_mm = 1000.0 * chord_at_three_quarters(blade);
    const double chord_share = static_cast<double>(env_u32("SPADE_V2B_EPS_CHORD100", 0)) / 100.0;
    const double eps_mm = chord_share > 0.0 ? chord_share * chord_mm
                                            : static_cast<double>(env_u32("SPADE_V2B_EPS_MM10", 0)) / 10.0;
    ASSERT_GT(eps_mm, 0.0) << "set SPADE_V2B_EPS_MM10 (80 is 8.0 mm) or SPADE_V2B_EPS_CHORD100";
    // L72k's B1: the estimated polar's two free parameters, SPADE_V2B_ALPHA0 (degrees, default -3) and SPADE_V2B_CDSCALE (1).
    const af::EstimatedPolarShape shape_fit{static_cast<float>(env_double("SPADE_V2B_ALPHA0", -3.0)),
                                            static_cast<float>(env_double("SPADE_V2B_CDSCALE", 1.0))};
    const af::PolarTable polar = af::estimated_polar(shape_fit);
    std::printf("V2b polar alpha_0 %.4f deg, cd_scale %.4f\n", static_cast<double>(shape_fit.alpha0_deg),
                static_cast<double>(shape_fit.cd_scale));
    const double radius = 0.5 * kD;
    const double n_rev = rpm / 60.0;
    const float omega = static_cast<float>(2.0 * std::numbers::pi * n_rev);
    // The reference: the library's static point, or the one a band note gives (SPADE_V2B_CT_E6, SPADE_V2B_CQ_E7).
    const uint32_t ct_env = env_u32("SPADE_V2B_CT_E6", 0);
    const uint32_t cq_env = env_u32("SPADE_V2B_CQ_E7", 0);
    const double ct_uiuc = ct_env != 0u ? static_cast<double>(ct_env) * 1.0e-6 : static_value(prop.ct_table);
    const double cq_uiuc = cq_env != 0u ? static_cast<double>(cq_env) * 1.0e-7 : static_value(prop.cq_table);
    const double dim_t = static_cast<double>(kRho) * n_rev * n_rev * std::pow(kD, 4);  // T = C_T * dim_t
    const double dim_q = dim_t * kD;                                                   // Q = C_Q * dim_q
    const double area = std::numbers::pi * radius * radius;
    const double v_h = std::sqrt(ct_uiuc * dim_t / (2.0 * static_cast<double>(kRho) * area));
    const double t_rv = radius / v_h;

    const float dx = static_cast<float>(radius) / static_cast<float>(rdx);
    const double eps_cells = eps_mm * 1.0e-3 / static_cast<double>(dx);
    ASSERT_TRUE(eps_cells >= 1.0 && eps_cells <= 3.0)
        << "the width " << eps_mm << " mm is " << eps_cells << " cells at this dx: the kernel allows 1 to 3";
    const auto cells = [&](double m) { return static_cast<uint32_t>(std::ceil(m / static_cast<double>(dx))); };
    const af::GridShape shape{cells(5.25 * radius), cells(8.3 * radius), cells(5.25 * radius), dx};
    const af::DomainBc bc;
    auto grid = af::make_grid(shape, bc);
    ASSERT_TRUE(grid.has_value()) << grid.error().context;
    af::FluidConfig cfg;
    cfg.rho = kRho;
    cfg.sgs_cs = static_cast<float>(env_u32("SPADE_V2B_CS100", 17)) / 100.0f;  // L84: the subgrid viscosity; L105a: 0 is refused with a line (the film's 0.17)
    cfg.h = 0.00038f * dx / 0.004f;

    af::LineRotor rotor;
    rotor.centre = glm::vec3(0.5f * static_cast<float>(shape.nx) * dx,
                             static_cast<float>(shape.ny) * dx - static_cast<float>(1.7 * radius),
                             0.5f * static_cast<float>(shape.nz) * dx);
    rotor.axis = glm::vec3(0.0f, 1.0f, 0.0f);
    rotor.omega = omega;
    rotor.blade = &blade;
    rotor.polar = &polar;
    rotor.eps_cells = static_cast<float>(eps_cells);
    rotor.tip_correction = env_u32("SPADE_V2B_TIP", 1) != 0;  // L72k's A1, the tip/smearing correction
    const std::vector<af::LineRotor> probe{rotor};
    const auto records = ag::line_records(probe, shape, kRho, cfg.h, n_sub);
    ASSERT_TRUE(records.has_value()) << records.error().context;

    auto ctx = spade::compute::VulkanContext::create(
        spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value());
    const ag::LineSetup setup{.polar = &polar,
                              .elements = records->per_rotor[0],
                              .n_sub = n_sub,
                              .tip_correction = rotor.tip_correction};
    auto gpu = ag::GpuFluid::create(**ctx, *grid, bc, cfg, nullptr, 64u, &setup);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;

    const uint32_t total = steps_override != 0u
                               ? steps_override
                               : static_cast<uint32_t>(std::ceil(40.0 * t_rv / static_cast<double>(cfg.h)));
    const auto step_of = [&](double t) {
        return static_cast<uint32_t>(std::ceil(t * t_rv / static_cast<double>(cfg.h)));
    };
    const std::array<uint32_t, 4> edge{step_of(20.0), step_of(30.0), step_of(40.0), total};
    std::printf("V2b prop %s, D %.3f m, %u blades, chord at 0.75 R %.2f mm (eps/chord %.2f)\n",
                std::string(prop.id).c_str(), kD, blade.blades, chord_mm, eps_mm / chord_mm);
    std::printf("V2b setup: R/dx %u (dx %.3f mm), box %ux%ux%u = %.2f M cells, h %.3f ms, n_sub %u, %u elements, "
                "%u steps (40 R/v_h = %.3f s); %.0f rpm; kernel eps %.2f mm = %.2f cells; UIUC static C_T %.5f "
                "C_Q %.6f (T %.4f N), v_h %.3f m/s; tip correction %s\n",
                rdx, static_cast<double>(dx) * 1000.0, shape.nx, shape.ny, shape.nz,
                static_cast<double>(shape.nx) * shape.ny * shape.nz / 1.0e6, static_cast<double>(cfg.h) * 1000.0, n_sub,
                setup.elements, total, 40.0 * t_rv, rpm, eps_mm, eps_cells, ct_uiuc, cq_uiuc, ct_uiuc * dim_t, v_h,
                rotor.tip_correction ? "ON" : "off");

    std::array<double, 2> thrust_sum{}, torque_sum{};
    std::array<uint32_t, 2> count{};
    double wall = 0.0;
    double psi = 0.0;
    for (uint32_t step = 0; step < total; ++step) {
        const auto t0 = std::chrono::steady_clock::now();
        rotor.psi = static_cast<float>(psi);
        const std::vector<af::LineRotor> rotors{rotor};
        const auto h = (*gpu)->step({}, rotors);
        ASSERT_TRUE(h.has_value()) << h.error().context;
        ASSERT_EQ(h->fault, af::FluidFault::none);
        const af::LineReport& rep = (*gpu)->line_reports().front();
        psi = std::fmod(psi + static_cast<double>(omega) * cfg.h, 2.0 * std::numbers::pi);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        wall += ms;
        if (step < 3 || (step + 1) % 200 == 0) {
            std::printf("V2b step %u: %.0f ms (mean %.0f ms), line thrust %.4f N torque %.5f N m\n", step + 1, ms,
                        wall / (step + 1), static_cast<double>(rep.thrust), static_cast<double>(rep.torque));
            std::fflush(stdout);
        }
        for (uint32_t w = 0; w < 2; ++w) {
            if (step >= edge[w] && step < edge[w + 1]) {
                thrust_sum[w] += rep.thrust;
                torque_sum[w] += rep.torque;
                ++count[w];
            }
        }
    }
    std::printf("V2b wall: %.1f s over %u steps (%.0f ms a step)\n", wall / 1000.0, total, wall / total);
    if (cfg.sgs_cs > 0.0f || env_u32("SPADE_V2B_KEHI", 0) != 0u) {
        // The grid-scale energy under the rotor at the end of the run: the block 2.4 R across, from 0.4 R under the disc down
        // 2.2 R, the cell-centre velocity (test_airflow_sgs.cpp's ke_hi, tests/ke_share.hpp)
        af::MacGrid read = *grid;
        ASSERT_TRUE((*gpu)->read(read).has_value());
        const auto half = static_cast<uint32_t>(std::lround(1.2 * rdx));
        const auto nyb = static_cast<uint32_t>(std::lround(2.2 * rdx));
        const auto ci = static_cast<uint32_t>(rotor.centre.x / dx), ck = static_cast<uint32_t>(rotor.centre.z / dx);
        const auto top = static_cast<uint32_t>((rotor.centre.y - 0.4f * static_cast<float>(radius)) / dx);
        ASSERT_GE(ci, half);
        ASSERT_GE(top, nyb);
        std::vector<float> block(std::size_t{2u * half} * nyb * 2u * half * 3u);
        float peak = 0.0f;
        for (uint32_t z = 0; z < 2u * half; ++z) {
            for (uint32_t y = 0; y < nyb; ++y) {
                for (uint32_t x = 0; x < 2u * half; ++x) {
                    const CellVel c = cell_velocity(read, ci - half + x, top - nyb + y, ck - half + z);
                    const std::size_t at = (std::size_t{z} * nyb + y) * (2u * half) + x;
                    block[at * 3u] = c.u;
                    block[at * 3u + 1u] = c.v;
                    block[at * 3u + 2u] = c.w;
                    peak = std::max(peak, std::sqrt(c.u * c.u + c.v * c.v + c.w * c.w));
                }
            }
        }
        const double ke_hi = spade::test::ke_share_above(block, 2u * half, nyb, 2u * half, static_cast<double>(dx));
        std::printf("V2b smooth: Cs %.2f, ke_hi %.4f on the block %ux%ux%u cells under the rotor; its fastest "
                    "cell %.2f m/s\n",
                    static_cast<double>(cfg.sgs_cs), ke_hi, 2u * half, nyb, 2u * half, static_cast<double>(peak));
        const char* slice = env_cstr("SPADE_V2B_SLICE");
        if (slice != nullptr) {
            write_slices(read, ck, 1.4f * static_cast<float>(v_h),
                         12.0f * static_cast<float>(v_h) / static_cast<float>(radius), 4u, slice);
            std::printf("V2b slices written: %s-speed.ppm, %s-vort.ppm (%ux%u, speed 0..%.1f m/s, "
                        "|omega_z| 0..%.0f 1/s)\n",
                        slice, slice, read.shape.nx * 4u, read.shape.ny * 4u, 1.4 * v_h, 12.0 * v_h / radius);
        }
    }
    if (steps_override != 0u || count[0] == 0 || count[1] == 0) return;

    const std::array<double, 2> thrust{thrust_sum[0] / count[0], thrust_sum[1] / count[1]};
    const std::array<double, 2> torque{torque_sum[0] / count[0], torque_sum[1] / count[1]};
    // L72d's note: the verdict is the mean over 10 R/v_h from 20 R/v_h; the run is settled when the window after it
    // has the same thrust to 2.5% and the same power (torque at one omega) to 3.75%.
    const double drift = std::abs(thrust[1] - thrust[0]) / thrust[0];
    const double power_drift = std::abs(torque[1] - torque[0]) / torque[0];
    const double ct = thrust[0] / dim_t;
    const double cq = torque[0] / dim_q;
    const double power = torque[0] * static_cast<double>(omega);
    const double fm = std::pow(thrust[0], 1.5) / (std::sqrt(2.0 * static_cast<double>(kRho) * area) * power);
    const bool settled = drift <= 0.025 && power_drift <= 0.0375;
    std::printf("V2b R/dx %u eps %.1f mm (%.2f cells) %.0f rpm: windows [20,30] (verdict) and [30,40] R/v_h thrust "
                "%.4f, %.4f N (differ %.2f%%), torque %.5f, %.5f N m (differ %.2f%%): %s\n",
                rdx, eps_mm, eps_cells, rpm, thrust[0], thrust[1], 100.0 * drift, torque[0], torque[1],
                100.0 * power_drift, settled ? "settled" : "UNSETTLED");
    std::printf("V2b C_T %.5f vs UIUC %.5f (%+.1f%%); C_Q %.6f vs %.6f (%+.1f%%); FM %.3f; both within 10%%: %s\n",
                ct, ct_uiuc,
                100.0 * (ct / ct_uiuc - 1.0), cq, cq_uiuc, 100.0 * (cq / cq_uiuc - 1.0), fm,
                (std::abs(ct / ct_uiuc - 1.0) <= 0.10 && std::abs(cq / cq_uiuc - 1.0) <= 0.10) ? "yes" : "no");
    EXPECT_TRUE(settled) << "unsettled: run longer";
}

}  // namespace
