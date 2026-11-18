// ---------------------------------------------------------------------------
// test_gpu_airflow_tip_dye.cpp -- L86i: the dye released at a line's blade tips on the device (DyeSetup::tip; the kernel
// airflow_dye_tip) against its CPU twin, tip_points() and emit_tip_dye() (physics/airflow/dye.hpp), in the order the GPU runs them: the
// step's line and discs, the step (the dye advected at its end), the discs' release, then the tips'.
//
//   GpuAirflowTipDye.TheTipDyeIsTheCpusBitForBit    a turning rotor and a disc in noise, 6 steps, then 2 steps without lines (no tip release,
//                                                   the points of the last line step are not released again): the dye equal after every step
//   GpuAirflowTipDye.TheTipDyeFollowsTheBladeAround dye of the tip release alone from clear air: its mass sits at the tips' arc, which
//                                                   turns with psi (the centroid of each blade's dye is near the blade's tip path)
//   GpuAirflowTipDye.RefusesATipDyeItCannotMake     a tip release without a LineSetup, and a release the check refuses
//
// Every test starts with the vulkan_skip_reason() skip; the bit comparisons are made on the device of record.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_exact_device.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/dye.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "spike/airflow/line_chain.hpp"
#include "spike/airflow/step_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
using spade::compute::VulkanContext;

constexpr float kDx = 0.004f;
constexpr float kRho = 1.225f;
constexpr float kH = 0.0005f;
constexpr uint32_t kN = 52;  // 0.208 m: reach R + 6 dx = 0.1002 m about a rotor near the middle
constexpr uint32_t kNSub = 4;

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

af::BladeShape blade_two() {
    af::BladeShape b;
    b.blades = 2;
    b.radius = 0.0762f;
    b.stations = {{0.0114f, 0.0085f, 30.0f}, {0.0457f, 0.0148f, 22.0f}, {0.0762f, 0.0010f, 14.0f}};
    return b;
}

[[nodiscard]] std::size_t differing(const std::vector<float>& a, const std::vector<float>& b) {
    std::size_t d = a.size() == b.size() ? 0u : std::max(a.size(), b.size());
    for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
        d += std::bit_cast<uint32_t>(a[i]) != std::bit_cast<uint32_t>(b[i]) ? 1u : 0u;
    }
    return d;
}

TEST(GpuAirflowTipDye, TheTipDyeIsTheCpusBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    const af::BladeShape blade = blade_two();
    const af::PolarTable polar = af::estimated_polar();
    const af::GridShape shape{kN, kN, kN, kDx};
    const af::DomainBc bc{};
    const af::FluidConfig cfg{.rho = kRho, .nu = 1.5e-5f, .h = kH, .n_v = 2, .visc_sweeps = 8};
    af::LineRotor rotor;
    rotor.centre = glm::vec3(0.104f, 0.104f, 0.104f);
    rotor.blade = &blade;
    rotor.polar = &polar;
    rotor.omega = 400.0f;
    rotor.psi = 0.3f;
    rotor.tip_correction = false;
    const std::vector<af::ActuatorDisc> discs{af::ActuatorDisc{
        .centre = {0.07f, 0.104f, 0.104f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = 0.03f, .thrust = 0.2f, .torque = 0.001f}};
    const af::DyeRelease release{400.0f, 0.0f, 1.0f};
    const af::TipRelease tip{.gain = 6.0f, .tip_frac = 0.97f};

    af::MacGrid start = *af::make_grid(shape, bc);
    uint32_t state = 0xC0FFEEu;
    for (std::vector<float>* comp : {&start.u, &start.v, &start.w}) {
        for (float& x : *comp) {
            state = state * 1664525u + 1013904223u;
            x = (static_cast<float>(state >> 8) / 16777216.0f - 0.5f) * 1.0f;
        }
    }
    auto scratch = af::make_fluid_scratch(start, bc, cfg);
    ASSERT_TRUE(scratch.has_value()) << scratch.error().context;
    ASSERT_TRUE(af::enable_dye(*scratch, start).has_value());
    af::MacGrid cpu = start;

    const std::vector<af::LineRotor> first{rotor};
    const auto probe = ag::line_records(first, shape, kRho, kH, kNSub);
    ASSERT_TRUE(probe.has_value()) << probe.error().context;
    const ag::LineSetup setup{.polar = &polar, .elements = probe->per_rotor[0], .n_sub = kNSub, .tip_correction = false};
    const ag::DyeSetup dye_setup{release, tip};
    auto gpu = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, &setup, &dye_setup);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;

    std::vector<float> got(scratch->dye.size());
    std::size_t total = 0;
    float dye_max = 0.0f;
    for (int step = 0; step < 8; ++step) {
        const bool with_lines = step < 6;
        if (with_lines) {
            const auto rep = af::spread_line(rotor, cpu, kRho, kH, kNSub, scratch->du, scratch->dv, scratch->dw);
            ASSERT_TRUE(rep.has_value()) << rep.error().context;
        }
        for (const af::ActuatorDisc& d : discs) {
            af::spread_actuator(d, shape, kRho, kH, scratch->du, scratch->dv, scratch->dw);
        }
        ASSERT_EQ(af::fluid_step(cpu, bc, cfg, *scratch).fault, af::FluidFault::none);
        af::emit_dye(scratch->dye, shape, discs, kH, release);
        if (with_lines) {
            const std::vector<af::LineRotor> rotors{rotor};
            af::emit_tip_dye(scratch->dye, shape, af::tip_points(rotors, kDx, kH, kNSub, tip.tip_frac), tip.gain);
        }
        const std::vector<af::LineRotor> rotors{rotor};
        const auto h = with_lines ? (*gpu)->step(discs, rotors) : (*gpu)->step(discs);
        ASSERT_TRUE(h.has_value()) << h.error().context;
        ASSERT_TRUE((*gpu)->read_dye(got).has_value());
        const std::size_t d = differing(got, scratch->dye);
        std::printf("[ tip dye ] step %d%s: %zu of %zu dye cells differ\n", step + 1, with_lines ? "" : " (no lines)", d, got.size());
        total += d;
        dye_max = std::max(dye_max, *std::max_element(scratch->dye.begin(), scratch->dye.end()));
        rotor.psi = std::fmod(rotor.psi + rotor.omega * kH, 6.28318530718f);
    }
    EXPECT_GT(dye_max, 0.3f) << "the run is not vacuous: the tips release";
    if (bit_equal) {
        EXPECT_EQ(total, 0u) << "the GPU's dye is the CPU's, bit for bit";
    }
}

// The tips alone from clear air, a fresh fluid at rest for each psi: the dye of one step sits on the tips' ring, in the rotor's plane,
// and its angular phase (the argument of the sum of d exp(2 i phi), two blades) is the blades' own: it turns by 2 dpsi with psi.
TEST(GpuAirflowTipDye, TheTipDyeFollowsTheBladeAround) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::BladeShape blade = blade_two();
    const af::PolarTable polar = af::estimated_polar();
    const af::GridShape shape{kN, kN, kN, kDx};
    const af::DomainBc bc{};
    const af::FluidConfig cfg{.rho = kRho, .nu = 1.5e-5f, .h = kH, .n_v = 2, .visc_sweeps = 8};
    const af::MacGrid start = *af::make_grid(shape, bc);
    double phase[2] = {0.0, 0.0};
    const float psis[2] = {0.0f, 0.4f};
    for (int run = 0; run < 2; ++run) {
        af::LineRotor rotor;
        rotor.centre = glm::vec3(0.104f, 0.104f, 0.104f);
        rotor.blade = &blade;
        rotor.polar = &polar;
        rotor.omega = 100.0f;  // slow: the line's load barely moves the air in one step
        rotor.tip_correction = false;
        rotor.psi = psis[run];
        const std::vector<af::LineRotor> rotors{rotor};
        const auto probe = ag::line_records(rotors, shape, kRho, kH, kNSub);
        ASSERT_TRUE(probe.has_value()) << probe.error().context;
        const ag::LineSetup setup{.polar = &polar, .elements = probe->per_rotor[0], .n_sub = kNSub, .tip_correction = false};
        const ag::DyeSetup dye_setup{af::DyeRelease{}, af::TipRelease{.gain = 6.0f, .tip_frac = 1.0f}};
        auto gpu = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, &setup, &dye_setup);
        ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
        ASSERT_TRUE((*gpu)->step({}, rotors).has_value());
        std::vector<float> dye(std::size_t{kN} * kN * kN);
        ASSERT_TRUE((*gpu)->read_dye(dye).has_value());
        double mass = 0.0;
        glm::dvec2 sum{0.0};
        for (uint32_t k = 0; k < kN; ++k) {
            for (uint32_t j = 0; j < kN; ++j) {
                for (uint32_t i = 0; i < kN; ++i) {
                    const double d = dye[i + kN * (j + std::size_t{kN} * k)];
                    if (d <= 0.0) continue;
                    const double x = (i + 0.5) * kDx - rotor.centre.x, y = (j + 0.5) * kDx - rotor.centre.y,
                                 z = (k + 0.5) * kDx - rotor.centre.z;
                    EXPECT_LT(std::fabs(std::sqrt(x * x + z * z) - 0.0762), 3.5 * kDx) << "on the tips' ring: psi " << psis[run];
                    EXPECT_LT(std::fabs(y), 2.5 * kDx) << "in the rotor's plane";
                    const double phi = std::atan2(-z, x);  // spin +1, ref +x, axis +y: phi grows from +x toward -z
                    mass += d;
                    sum += d * glm::dvec2(std::cos(2.0 * phi), std::sin(2.0 * phi));
                }
            }
        }
        ASSERT_GT(mass, 1.0) << "a tip releases: psi " << psis[run];
        EXPECT_GT(glm::length(sum) / mass, 0.2) << "the dye is at the blades, not round the ring";
        phase[run] = std::atan2(sum.y, sum.x);
    }
    double turned = phase[1] - phase[0];
    if (turned < -3.14159265) turned += 6.28318531;
    if (turned > 3.14159265) turned -= 6.28318531;
    EXPECT_NEAR(turned, 0.8, 0.05) << "the release turned with the blades (2 x 0.4 rad)";
}

TEST(GpuAirflowTipDye, RefusesATipDyeItCannotMake) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::GridShape shape{kN, kN, kN, kDx};
    const af::DomainBc bc{};
    const af::FluidConfig cfg{.rho = kRho, .nu = 1.5e-5f, .h = kH, .n_v = 2, .visc_sweeps = 8};
    const af::MacGrid start = *af::make_grid(shape, bc);
    const ag::DyeSetup no_line{af::DyeRelease{}, af::TipRelease{}};
    const auto a = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, nullptr, &no_line);
    ASSERT_FALSE(a.has_value());
    EXPECT_NE(a.error().context.find("tip"), std::string::npos) << a.error().context;
    EXPECT_NE(a.error().context.find("LineSetup"), std::string::npos) << a.error().context;
    const af::PolarTable polar = af::estimated_polar();
    const ag::LineSetup setup{.polar = &polar, .elements = 10, .n_sub = kNSub, .tip_correction = false};
    const ag::DyeSetup bad{af::DyeRelease{}, af::TipRelease{.gain = 0.0f}};
    const auto b = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, &setup, &bad);
    ASSERT_FALSE(b.has_value());
    EXPECT_NE(b.error().context.find("gain"), std::string::npos) << b.error().context;
}

}  // namespace
