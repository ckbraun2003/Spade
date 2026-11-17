// ---------------------------------------------------------------------------
// test_gpu_airflow_line_binned.cpp -- L86i: the actuator line's spread by rotor boxes (LineSetup::binned; the kernel
// airflow_line_spread_binned), against the CPU's spread_line() and against the unbinned kernel, bit for bit.
//
//   GpuAirflowLineBinned.TheRotorBoxesHoldEveryStencilAndAreTight   line_ranges(): each rotor's first element and count, its boxes hold every
//                                                                   element's stencil on each component and some element touches each face
//   GpuAirflowLineBinned.TheBinnedStepIsTheCpusAndTheUnbinnedsBitForBit
//                                                                   two overlapping rotors (one tilted), noise, 3 steps: u, v, w, p and the
//                                                                   reports equal to the CPU's, the binned fluid's to the unbinned one's
//   GpuAirflowLineBinned.RefusesABinnedSetupItCannotMake            binned with no rotors, and a step with another rotor count
//
// Every GPU test starts with the vulkan_skip_reason() skip; the bit comparisons are made on the device of record.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_exact_device.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/blade.hpp"
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
constexpr uint32_t kN = 52;
constexpr uint32_t kNSub = 2;

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

af::BladeShape blade_of(uint32_t blades, float radius) {
    af::BladeShape b;
    b.blades = blades;
    b.radius = radius;
    b.stations = {{0.15f * radius, 0.0085f, 30.0f}, {0.6f * radius, 0.0148f, 22.0f}, {radius, 0.0010f, 14.0f}};
    return b;
}

struct Rig {
    af::BladeShape big = blade_of(2, 0.0762f);
    af::BladeShape small = blade_of(3, 0.05f);
    af::PolarTable polar = af::estimated_polar();
    std::vector<af::LineRotor> rotors;

    Rig() {
        af::LineRotor a;
        a.centre = glm::vec3(0.104f, 0.104f, 0.104f);
        a.blade = &big;
        a.polar = &polar;
        a.omega = 400.0f;
        a.psi = 0.3f;
        a.tip_correction = false;
        af::LineRotor b;  // a left hand, its axis along +x, overlapping the first
        b.centre = glm::vec3(0.12f, 0.09f, 0.104f);
        b.axis = {1.0f, 0.0f, 0.0f};
        b.ref = {0.0f, 1.0f, 0.0f};
        b.spin = -1.0f;
        b.blade = &small;
        b.polar = &polar;
        b.omega = 500.0f;
        b.psi = 1.1f;
        b.tip_correction = false;
        rotors = {a, b};
    }
};

[[nodiscard]] std::size_t differing(const std::vector<float>& a, const std::vector<float>& b) {
    std::size_t d = a.size() == b.size() ? 0u : std::max(a.size(), b.size());
    for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
        d += std::bit_cast<uint32_t>(a[i]) != std::bit_cast<uint32_t>(b[i]) ? 1u : 0u;
    }
    return d;
}

TEST(GpuAirflowLineBinned, TheRotorBoxesHoldEveryStencilAndAreTight) {
    Rig rig;
    const af::GridShape shape{kN, kN, kN, kDx};
    const auto records = ag::line_records(rig.rotors, shape, kRho, kH, kNSub);
    ASSERT_TRUE(records.has_value()) << records.error().context;
    const std::vector<float> ranges = ag::line_ranges(*records);
    ASSERT_EQ(ranges.size(), std::size_t{ag::kLineRangeWords} * 2u);
    uint32_t first = 0;
    for (std::size_t r = 0; r < 2; ++r) {
        const float* rec = ranges.data() + r * ag::kLineRangeWords;
        EXPECT_EQ(rec[0], static_cast<float>(first)) << "rotor " << r << "'s first element";
        EXPECT_EQ(rec[1], static_cast<float>(records->per_rotor[r])) << "rotor " << r << "'s count";
        for (uint32_t c = 0; c < 3; ++c) {
            float lo[3] = {1.0e9f, 1.0e9f, 1.0e9f}, hi[3] = {-1.0e9f, -1.0e9f, -1.0e9f};
            for (uint32_t e = first; e < first + records->per_rotor[r]; ++e) {
                const float* comp = records->elems.data() + std::size_t{e} * ag::kLineStride + ag::kLineHeaderWords +
                                    std::size_t{c} * ag::kLineComponentWords;
                for (int a = 0; a < 3; ++a) {
                    lo[a] = std::min(lo[a], comp[a]);
                    hi[a] = std::max(hi[a], comp[a] + comp[3 + a]);
                }
            }
            for (int a = 0; a < 3; ++a) {
                EXPECT_EQ(rec[2 + 6 * c + a], lo[a]) << "rotor " << r << " component " << c << " lo axis " << a;
                EXPECT_EQ(rec[2 + 6 * c + 3 + a], hi[a]) << "rotor " << r << " component " << c << " hi axis " << a;
            }
        }
        first += records->per_rotor[r];
    }
}

TEST(GpuAirflowLineBinned, TheBinnedStepIsTheCpusAndTheUnbinnedsBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    Rig rig;
    const af::GridShape shape{kN, kN, kN, kDx};
    const af::DomainBc bc{};
    const af::FluidConfig cfg{.rho = kRho, .nu = 1.5e-5f, .h = kH, .n_v = 2, .visc_sweeps = 8};
    af::MacGrid start = *af::make_grid(shape, bc);
    uint32_t state = 0xBEEF01u;
    for (std::vector<float>* comp : {&start.u, &start.v, &start.w}) {
        for (float& x : *comp) {
            state = state * 1664525u + 1013904223u;
            x = (static_cast<float>(state >> 8) / 16777216.0f - 0.5f) * 1.0f;
        }
    }
    auto scratch = af::make_fluid_scratch(start, bc, cfg);
    ASSERT_TRUE(scratch.has_value()) << scratch.error().context;
    af::MacGrid cpu = start;

    const auto probe = ag::line_records(rig.rotors, shape, kRho, kH, kNSub);
    ASSERT_TRUE(probe.has_value()) << probe.error().context;
    const uint32_t elements = probe->per_rotor[0] + probe->per_rotor[1];
    ag::LineSetup plain{.polar = &rig.polar, .elements = elements, .n_sub = kNSub, .tip_correction = false};
    ag::LineSetup binned = plain;
    binned.binned = true;
    binned.rotors = 2;
    auto gpu_plain = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, &plain);
    ASSERT_TRUE(gpu_plain.has_value()) << gpu_plain.error().context;
    auto gpu_binned = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, &binned);
    ASSERT_TRUE(gpu_binned.has_value()) << gpu_binned.error().context;

    std::size_t total = 0;
    for (int step = 0; step < 3; ++step) {
        std::vector<af::LineReport> want;
        for (const af::LineRotor& r : rig.rotors) {
            const auto rep = af::spread_line(r, cpu, kRho, kH, kNSub, scratch->du, scratch->dv, scratch->dw);
            ASSERT_TRUE(rep.has_value()) << rep.error().context;
            want.push_back(*rep);
        }
        ASSERT_EQ(af::fluid_step(cpu, bc, cfg, *scratch).fault, af::FluidFault::none);
        ASSERT_TRUE((*gpu_plain)->step({}, rig.rotors).has_value());
        const auto got = (*gpu_binned)->step({}, rig.rotors);
        ASSERT_TRUE(got.has_value()) << got.error().context;
        af::MacGrid a = start, b = start;
        ASSERT_TRUE((*gpu_plain)->read(a).has_value());
        ASSERT_TRUE((*gpu_binned)->read(b).has_value());
        const std::size_t vs_cpu = differing(b.u, cpu.u) + differing(b.v, cpu.v) + differing(b.w, cpu.w) + differing(b.p, cpu.p);
        const std::size_t vs_plain = differing(b.u, a.u) + differing(b.v, a.v) + differing(b.w, a.w) + differing(b.p, a.p);
        std::printf("[ binned  ] step %d: %zu samples differ from the CPU, %zu from the unbinned kernel's\n", step + 1, vs_cpu, vs_plain);
        total += vs_cpu + vs_plain;
        ASSERT_EQ((*gpu_binned)->line_reports().size(), 2u);
        for (std::size_t r = 0; r < 2; ++r) {
            if (bit_equal) {
                EXPECT_EQ(std::bit_cast<uint32_t>((*gpu_binned)->line_reports()[r].thrust), std::bit_cast<uint32_t>(want[r].thrust))
                    << "rotor " << r << " step " << step + 1;
            }
        }
        for (af::LineRotor& r : rig.rotors) r.psi = std::fmod(r.psi + r.omega * kH, 6.28318530718f);
    }
    if (bit_equal) EXPECT_EQ(total, 0u);
}

TEST(GpuAirflowLineBinned, RefusesABinnedSetupItCannotMake) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    Rig rig;
    const af::GridShape shape{kN, kN, kN, kDx};
    const af::DomainBc bc{};
    const af::FluidConfig cfg{.rho = kRho, .nu = 1.5e-5f, .h = kH, .n_v = 2, .visc_sweeps = 8};
    const af::MacGrid start = *af::make_grid(shape, bc);
    const auto probe = ag::line_records(rig.rotors, shape, kRho, kH, kNSub);
    ASSERT_TRUE(probe.has_value());
    const uint32_t elements = probe->per_rotor[0] + probe->per_rotor[1];
    ag::LineSetup none{.polar = &rig.polar, .elements = elements, .n_sub = kNSub, .tip_correction = false, .binned = true, .rotors = 0};
    const auto a = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, &none);
    ASSERT_FALSE(a.has_value());
    EXPECT_NE(a.error().context.find("rotors"), std::string::npos) << a.error().context;
    ag::LineSetup one = none;
    one.rotors = 1;  // the steps below give two
    auto gpu = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, &one);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    const auto b = (*gpu)->step({}, rig.rotors);
    ASSERT_FALSE(b.has_value());
    EXPECT_NE(b.error().context.find("rotors"), std::string::npos) << b.error().context;
}

}  // namespace
