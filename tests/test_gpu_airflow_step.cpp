// ---------------------------------------------------------------------------
// test_gpu_airflow_step.cpp -- plan B, B7's G1: GpuFluid
// (spike/airflow/step_chain.hpp), the whole fluid step on the device, against
// fluid_step() (engine/physics/airflow/fluid_step.cpp) in LiveRun's order:
// each step's discs spread into du (spread_actuator()), then the step.
//
//   GpuAirflowStep   bit for bit on the device of record
//                    (airflow_exact_device.hpp; elsewhere reported only),
//                    u, v, w and p after every step: an open box in a wind
//                    with two discs and the air's viscosity; a periodic
//                    channel with one disc; a closed box under a moving lid,
//                    inviscid and with no disc (the closed domain's means);
//                    the same bits at workgroup sizes 32, 64 and 128; the
//                    publish equal to publish_render_volume()'s; the
//                    refusals; with SPADE_AIRFLOW_TIMINGS=1, a step's time in
//                    the D1 box.
//
// The fields start as fixed-seed noise with a few subnormals mixed in, so
// the flush is exercised too. Every Gpu* test starts with the
// vulkan_skip_reason() skip (gpu_skip.hpp).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_exact_device.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/flow_volume.hpp"
#include "d1_case.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "spike/airflow/step_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
using spade::compute::VulkanContext;

constexpr uint32_t kSteps = 6;
constexpr float kNu = 1.4607e-5f;  // the air's, m^2/s (D1's)

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// splitmix64: a fixed-seed stream, the same on every compiler.
struct SplitMix {
    uint64_t state = 0;
    uint64_t next() {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    float unit() { return static_cast<float>(next() >> 40) * 0x1p-24f * 2.0f - 1.0f; }  // [-1, 1)
};

// One configuration: the box, its faces, the step and the discs.
struct Case {
    const char* name;
    af::GridShape shape;
    af::DomainBc bc;
    af::FluidConfig cfg;
    std::vector<af::ActuatorDisc> discs;
};

// The ambient plus noise of `scale` on every face, one face in 61 a subnormal
// of either sign; p noise of a tenth of it, the projection's warm start.
[[nodiscard]] af::MacGrid noisy(const Case& c, uint64_t seed, float scale) {
    af::MacGrid g = *af::make_grid(c.shape, c.bc);
    SplitMix rng{seed};
    std::size_t n = 0;
    const std::array<std::vector<float>*, 3> comps{&g.u, &g.v, &g.w};
    for (uint32_t k = 0; k < 3; ++k) {
        for (float& x : *comps[k]) {
            x = (++n % 61u == 0u) ? (rng.unit() < 0.0f ? -1.0e-40f : 3.0e-41f)
                                  : c.bc.ambient[static_cast<glm::length_t>(k)] + scale * rng.unit();
        }
    }
    for (float& x : g.p) x = 0.1f * scale * rng.unit();
    return g;
}

[[nodiscard]] af::FluidConfig config(float nu, float h) {
    return af::FluidConfig{.rho = 1.225f, .nu = nu, .h = h, .n_v = 2, .visc_sweeps = 8};
}

// A rotor disc pushing the air down (-y), its torque swirling it.
[[nodiscard]] af::ActuatorDisc disc(glm::vec3 centre, float thrust, float torque) {
    return af::ActuatorDisc{.centre = centre, .axis = {0.0f, -1.0f, 0.0f}, .radius = 0.05f, .thrust = thrust,
                            .torque = torque};
}

// An open box in a 3 m/s wind along -x (an inflow at +x, outlets elsewhere),
// the air's viscosity, two discs; D1's domain in small.
[[nodiscard]] Case open_box() {
    Case c{"open box, wind, 2 discs", {24, 20, 18, 0.02f}, {}, config(kNu, 0.002f), {}};
    c.bc.ambient = {-3.0f, 0.0f, 0.0f};
    c.discs = {disc({0.20f, 0.26f, 0.13f}, 1.5f, 0.02f), disc({0.30f, 0.26f, 0.23f}, 1.2f, -0.02f)};
    return c;
}

// Periodic along x and z, walls along y: the advection's wrap and the viscous
// solve's alias faces.
[[nodiscard]] Case periodic_channel() {
    Case c{"periodic channel, 1 disc", {20, 16, 12, 0.025f}, {}, config(kNu, 0.002f), {}};
    c.bc.face = {af::FaceBc::periodic, af::FaceBc::periodic, af::FaceBc::wall,
                 af::FaceBc::wall,     af::FaceBc::periodic, af::FaceBc::periodic};
    c.bc.ambient = {1.0f, 0.0f, 0.0f};
    c.discs = {disc({0.25f, 0.2f, 0.15f}, 1.0f, 0.01f)};
    return c;
}

// Walls all round, the top one sliding along +x; inviscid, no disc: the
// closed domain's means (tile_sum, remove_mean) and the chain without a
// gather or the viscous solve.
[[nodiscard]] Case lid_box() {
    Case c{"closed box, lid, inviscid", {16, 16, 16, 0.025f}, {}, config(0.0f, 0.002f), {}};
    c.bc.face.fill(af::FaceBc::wall);
    c.bc.wall_velocity[af::kYPlus] = {1.0f, 0.0f, 0.0f};
    return c;
}

// LiveRun's step on the CPU: the discs into du, then fluid_step().
struct CpuFluid {
    af::MacGrid g;
    af::FluidScratch s;

    af::FluidHealth step(const Case& c) {
        for (const af::ActuatorDisc& d : c.discs) {
            af::spread_actuator(d, g.shape, c.cfg.rho, c.cfg.h, s.du, s.dv, s.dw);
        }
        return af::fluid_step(g, c.bc, c.cfg, s);
    }
};

[[nodiscard]] std::optional<CpuFluid> cpu_fluid(const Case& c, const af::MacGrid& start) {
    auto s = af::make_fluid_scratch(start, c.bc, c.cfg);
    if (!s) {
        ADD_FAILURE() << s.error().context;
        return std::nullopt;
    }
    return CpuFluid{start, std::move(*s)};
}

[[nodiscard]] std::size_t differing(const std::vector<float>& a, const std::vector<float>& b) {
    std::size_t d = a.size() == b.size() ? 0u : std::max(a.size(), b.size());
    for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
        d += std::bit_cast<uint32_t>(a[i]) != std::bit_cast<uint32_t>(b[i]) ? 1u : 0u;
    }
    return d;
}

[[nodiscard]] std::size_t differing(const af::MacGrid& a, const af::MacGrid& b) {
    return differing(a.u, b.u) + differing(a.v, b.v) + differing(a.w, b.w) + differing(a.p, b.p);
}

[[nodiscard]] std::size_t samples_of(const af::MacGrid& g) { return g.u.size() + g.v.size() + g.w.size() + g.p.size(); }

// Steps `c` kSteps times on both sides from the same noise, comparing after
// every step; the count of differing samples summed over the steps, or
// nullopt on a refusal.
[[nodiscard]] std::optional<std::size_t> run_both(VulkanContext& ctx, const Case& c, uint32_t workgroup_size,
                                                  af::MacGrid* gpu_end = nullptr) {
    const af::MacGrid start = noisy(c, 0x57E9ull, 1.0f);
    auto cpu = cpu_fluid(c, start);
    if (!cpu) return std::nullopt;
    auto gpu = ag::GpuFluid::create(ctx, start, c.bc, c.cfg, nullptr, workgroup_size);
    if (!gpu) {
        ADD_FAILURE() << gpu.error().context;
        return std::nullopt;
    }
    af::MacGrid read = start;
    std::size_t total = 0;
    for (uint32_t step = 0; step < kSteps; ++step) {
        const af::FluidHealth h = cpu->step(c);
        if (h.fault != af::FluidFault::none) {
            ADD_FAILURE() << "the CPU step faulted: " << af::describe(h, c.shape);
            return std::nullopt;
        }
        const auto g = (*gpu)->step(c.discs);
        if (!g) {
            ADD_FAILURE() << g.error().context;
            return std::nullopt;
        }
        EXPECT_EQ(g->fault, af::FluidFault::none);
        EXPECT_EQ(g->pressure_iterations, c.cfg.n_v);
        if (const auto ok = (*gpu)->read(read); !ok) {
            ADD_FAILURE() << ok.error().context;
            return std::nullopt;
        }
        const std::size_t d = differing(read, cpu->g);
        std::printf("[ step    ] %-28s wg %3u step %u: %zu of %zu samples differ\n", c.name, workgroup_size, step + 1,
                    d, samples_of(read));
        total += d;
    }
    if (gpu_end != nullptr) *gpu_end = read;
    return total;
}

void expect_bit_equal(const Case& c) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    const auto d = run_both(*ctx, c, 64u);
    ASSERT_TRUE(d.has_value());
    if (bit_equal) {
        EXPECT_EQ(*d, 0u);
    }
}

}  // namespace

// The discs reach the flow: the CPU's run with them differs from its run
// without, so the step tests below compare a gathered step.
TEST(GpuAirflowStep, TheDiscsMoveTheFlowOnTheCpu) {
    Case with = open_box();
    Case without = with;
    without.discs.clear();
    const af::MacGrid start = noisy(with, 0x57E9ull, 1.0f);
    auto a = cpu_fluid(with, start);
    auto b = cpu_fluid(without, start);
    ASSERT_TRUE(a && b);
    ASSERT_EQ(a->step(with).fault, af::FluidFault::none);
    ASSERT_EQ(b->step(without).fault, af::FluidFault::none);
    EXPECT_GT(differing(a->g, b->g), 0u);
}

TEST(GpuAirflowStep, OpenBoxWithDiscsMatchesFluidStepBitForBit) { expect_bit_equal(open_box()); }

TEST(GpuAirflowStep, PeriodicChannelMatchesFluidStepBitForBit) { expect_bit_equal(periodic_channel()); }

TEST(GpuAirflowStep, ClosedLidBoxMatchesFluidStepBitForBit) { expect_bit_equal(lid_box()); }

// No invocation reads another's result within a dispatch, and the closed
// domain's sums run in reduce.hpp's fixed blocks: the same bits at every
// supported workgroup size, on any device.
TEST(GpuAirflowStep, SameBitsAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    for (const Case& c : {open_box(), lid_box()}) {
        SCOPED_TRACE(c.name);
        std::optional<af::MacGrid> first;
        for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
            af::MacGrid end;
            ASSERT_TRUE(run_both(*ctx, c, wg, &end).has_value());
            if (!first) {
                first = end;
            } else {
                EXPECT_EQ(differing(end, *first), 0u) << "wg " << wg;
            }
        }
    }
}

// The publish after the steps is publish_render_volume()'s of the CPU's flow:
// the same halves and header, on the device of record.
TEST(GpuAirflowStep, PublishMatchesTheCpuVolume) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    const Case c = periodic_channel();
    const af::MacGrid start = noisy(c, 0x9B1Bull, 1.0f);
    auto cpu = cpu_fluid(c, start);
    ASSERT_TRUE(cpu.has_value());
    auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    for (uint32_t step = 0; step < 2; ++step) {
        ASSERT_EQ(cpu->step(c).fault, af::FluidFault::none);
        ASSERT_TRUE((*gpu)->step(c.discs).has_value());
    }
    const glm::vec3 origin{-0.25f, 0.0f, 1.5f};
    spade::FlowVolume want;
    const af::PublishInput in{.origin = origin, .periodic = af::periodic_axes(c.bc)};
    ASSERT_TRUE(af::publish_render_volume(cpu->g, in, want).has_value());
    spade::FlowVolume got;
    const auto ok = (*gpu)->publish(origin, got);
    ASSERT_TRUE(ok.has_value()) << ok.error().context;
    ASSERT_EQ(got.v.size(), want.v.size());
    ASSERT_EQ(got.s.size(), want.s.size());
    std::size_t d = 0;
    for (std::size_t i = 0; i < want.v.size(); ++i) d += got.v[i] != want.v[i] ? 1u : 0u;
    for (std::size_t i = 0; i < want.s.size(); ++i) d += got.s[i] != want.s[i] ? 1u : 0u;
    std::printf("[ step    ] publish: %zu of %zu halves differ\n", d, want.v.size() + want.s.size());
    EXPECT_EQ(got.header.origin, want.header.origin);
    EXPECT_EQ(got.header.cell, want.header.cell);
    EXPECT_EQ(got.header.nx, want.header.nx);
    EXPECT_EQ(got.header.extrapolated_cells, 0u);
    if (bit_equal) {
        EXPECT_EQ(d, 0u);
        EXPECT_EQ(got.header.channel_min, want.header.channel_min);
        EXPECT_EQ(got.header.channel_max, want.header.channel_max);
        EXPECT_EQ(got.header.no_data, want.header.no_data);
    }
}

// What GpuFluid refuses: make_fluid_scratch()'s config refusals, a
// SolidField with parts (G3), more than kMaxDiscs discs.
TEST(GpuAirflowStep, RefusesWhatItCannotStep) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Case c = open_box();
    const af::MacGrid start = noisy(c, 1u, 1.0f);
    af::FluidConfig no_cycles = c.cfg;
    no_cycles.n_v = 0;
    const auto a = ag::GpuFluid::create(*ctx, start, c.bc, no_cycles);
    ASSERT_FALSE(a.has_value());
    EXPECT_NE(a.error().context.find("n_v"), std::string::npos) << a.error().context;
    af::SolidField parts;
    parts.any = true;
    const auto b = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, &parts);
    ASSERT_FALSE(b.has_value());
    EXPECT_NE(b.error().context.find("solids"), std::string::npos) << b.error().context;
    auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    const std::vector<af::ActuatorDisc> many(ag::kMaxDiscs + 1u, c.discs.front());
    const auto s = (*gpu)->step(many);
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::capacity_exceeded);
}

// A step's time in the D1 box (240 x 177 x 168 at 4 mm, the air's nu, h
// 0.38 ms) with four discs about its centre, no read-back: the mean of 20
// steps after 2 to warm up. Opt-in with SPADE_AIRFLOW_TIMINGS, as the
// kernels' D1 times are.
TEST(GpuAirflowStep, StepTimeInTheD1Box) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const bool timings = std::getenv("SPADE_AIRFLOW_TIMINGS") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (!timings) GTEST_SKIP() << "opt-in: SPADE_AIRFLOW_TIMINGS=1";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::GridShape s = spade::testing::kD1Grid;
    Case c{"D1 box", s, {}, config(kNu, spade::testing::kD1Step), {}};
    c.bc.ambient = {-3.0f, 0.0f, 0.0f};
    const glm::vec3 mid = 0.5f * s.dx * glm::vec3(static_cast<float>(s.nx), static_cast<float>(s.ny),
                                                  static_cast<float>(s.nz));
    for (const glm::vec2 xz : {glm::vec2(-0.1f, -0.1f), glm::vec2(0.1f, -0.1f), glm::vec2(-0.1f, 0.1f),
                               glm::vec2(0.1f, 0.1f)}) {
        c.discs.push_back(disc(mid + glm::vec3(xz.x, 0.0f, xz.y), 1.5f, 0.02f));
    }
    const af::MacGrid start = noisy(c, 0xD1ull, 3.0f);
    auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    for (int warm = 0; warm < 2; ++warm) ASSERT_TRUE((*gpu)->step(c.discs).has_value());
    constexpr int kTimed = 20;
    const auto t0 = std::chrono::steady_clock::now();
    for (int step = 0; step < kTimed; ++step) ASSERT_TRUE((*gpu)->step(c.discs).has_value());
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / kTimed;
    std::printf("measure: GpuFluid step in the D1 box, 240x177x168 at 4 mm, 4 discs, n_v 2, 8 sweeps: %.3f ms\n", ms);
}
