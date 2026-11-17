// ---------------------------------------------------------------------------
// test_gpu_airflow_dye.cpp -- P10: the passive dye on the device (spike/airflow/step_chain.hpp's DyeSetup; the kernels
// airflow_dye_{advect,source,publish}) against its CPU twin: advect_scalar() inside fluid_step() on FluidScratch::dye, and
// emit_dye() after the step (engine/physics/airflow/dye.hpp), in LiveRun's order.
//
//   GpuAirflowDye.TheDyeIsTheCpusBitForBit              the dye after every step, in the open box, the periodic channel and the
//                                                       box with a sphere (solids): bit for bit on the device of record
//   GpuAirflowDye.TheDyeIsTheCpusAtEveryWorkgroupSize   the open box at 32, 64 and 128
//   GpuAirflowDye.ThePublishCarriesTheDyeInChannelThree publish_render_volume()'s halves with the dye, solids included (the
//                                                       first solid layer's mean of the fluid neighbours' dye)
//   GpuAirflowDye.TheFlowIsTheSameBitsWithTheDye        u, v, w and p of a fluid with the dye and one without, step by step
//   GpuAirflowDye.RefusesWhatItCannotCarry              a release the check refuses; read_dye() on a fluid with no dye
//
// The fields start as fixed-seed noise, a few subnormals mixed in, as test_gpu_airflow_step.cpp's. Every test starts with the
// vulkan_skip_reason() skip (gpu_skip.hpp); the bit comparisons are made on the device of record (airflow_exact_device.hpp),
// elsewhere the differences are reported only.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
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
#include "airflow_solid_shapes.hpp"
#include "b7_harness.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "d1_case.hpp"
#include "core/flow_volume.hpp"
#include "gpu_skip.hpp"
#include "perf_line.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/dye.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/regions.hpp"
#include "spike/airflow/step_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
using spade::compute::VulkanContext;

constexpr uint32_t kSteps = 8;
constexpr float kNu = 1.4607e-5f;

[[nodiscard]] const char* env_cstr(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

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

struct Case {
    const char* name;
    af::GridShape shape;
    af::DomainBc bc;
    af::FluidConfig cfg;
    std::vector<af::ActuatorDisc> discs;
    af::DyeRelease release;
    std::vector<airflow_test::Shape> shapes{};  // set_solids()' parts; none: no solids
};

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

[[nodiscard]] af::ActuatorDisc disc(glm::vec3 centre, float thrust, float torque) {
    return af::ActuatorDisc{.centre = centre, .axis = {0.0f, -1.0f, 0.0f}, .radius = 0.05f, .thrust = thrust,
                            .torque = torque};
}

// The release is strong (rate 400 at h 0.002 fills a peak cell by 0.8 a step) so the dye is everywhere the discs reach.
[[nodiscard]] Case open_box() {
    Case c{"open box, wind, 2 discs", {24, 20, 18, 0.02f}, {}, config(kNu, 0.002f), {}, {400.0f, 0.0f, 1.0f}};
    c.bc.ambient = {-3.0f, 0.0f, 0.0f};
    c.discs = {disc({0.20f, 0.26f, 0.13f}, 1.5f, 0.02f), disc({0.30f, 0.26f, 0.23f}, 1.2f, -0.02f)};
    return c;
}

// Periodic along x and z, the tip ring of a disc tilted off the axes (the release's cube on a tilted axis).
[[nodiscard]] Case periodic_channel() {
    Case c{"periodic channel, tilted ring", {20, 16, 12, 0.025f}, {}, config(kNu, 0.002f), {}, {400.0f, 0.5f, 1.2f}};
    c.bc.face = {af::FaceBc::periodic, af::FaceBc::periodic, af::FaceBc::wall,
                 af::FaceBc::wall,     af::FaceBc::periodic, af::FaceBc::periodic};
    c.bc.ambient = {1.0f, 0.0f, 0.0f};
    af::ActuatorDisc d = disc({0.25f, 0.2f, 0.15f}, 1.0f, 0.01f);
    d.axis = glm::normalize(glm::vec3(0.3f, -1.0f, 0.2f));
    d.radius = 0.08f;
    c.discs = {d};
    return c;
}

[[nodiscard]] af::FluidConfig solids_config(float nu, float h) {
    af::FluidConfig cfg = config(nu, h);
    cfg.n_v = af::kSolidsCgIterations;
    return cfg;
}

// Walls all round, a sphere spinning about y below a disc: solid cells in the field, the publish's neighbour mean.
[[nodiscard]] Case sphere_box() {
    Case c{"closed box, spinning sphere", {20, 24, 16, 0.02f}, {}, solids_config(kNu, 0.002f), {}, {400.0f, 0.0f, 1.0f}};
    c.bc.face.fill(af::FaceBc::wall);
    c.discs = {disc({0.20f, 0.38f, 0.16f}, 1.0f, 0.01f)};
    const glm::vec3 about{0.20f, 0.18f, 0.16f};
    c.shapes = {{.name = "sphere", .kind = airflow_test::Kind::sphere, .centre = about, .radius = 0.08f,
                 .omega = {0.0f, 15.0f, 0.0f}, .min_feature = 0.16f}};
    return c;
}

// LiveRun's step on the CPU: the discs into du, the step (the dye advected at its end), then the release.
struct CpuFluid {
    af::MacGrid g;
    af::FluidScratch s;

    af::FluidHealth step(const Case& c) {
        for (const af::ActuatorDisc& d : c.discs) {
            af::spread_actuator(d, g.shape, c.cfg.rho, c.cfg.h, s.du, s.dv, s.dw);
        }
        const af::FluidHealth h = af::fluid_step(g, c.bc, c.cfg, s);
        if (!s.dye.empty()) af::emit_dye(s.dye, g.shape, c.discs, c.cfg.h, c.release);
        return h;
    }
};

[[nodiscard]] std::optional<CpuFluid> cpu_fluid(const Case& c, const af::MacGrid& start, bool with_dye) {
    auto s = af::make_fluid_scratch(start, c.bc, c.cfg);
    if (!s) {
        ADD_FAILURE() << s.error().context;
        return std::nullopt;
    }
    if (!c.shapes.empty()) {
        const airflow_test::Scene scene{c.shapes};
        if (const auto ok = af::set_solids(*s, start, c.bc, scene.parts(), scene.sampler()); !ok || !s->solids.any) {
            ADD_FAILURE() << c.name << ": the parts left no solid";
            return std::nullopt;
        }
    }
    CpuFluid cpu{start, std::move(*s)};
    if (with_dye) {
        if (const auto ok = af::enable_dye(cpu.s, cpu.g); !ok) {
            ADD_FAILURE() << ok.error().context;
            return std::nullopt;
        }
    }
    return cpu;
}

[[nodiscard]] const af::SolidField* solids_of(const CpuFluid& cpu) { return cpu.s.solids.any ? &cpu.s.solids : nullptr; }

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

// Steps `c` on both sides from the same noise; after every step the dye's differing cells (and the flow's differing samples),
// summed. `reach` gets the CPU dye's most cells with a value above 0 and its largest value: a case whose dye never moves
// proves nothing.
[[nodiscard]] std::optional<std::size_t> run_both(VulkanContext& ctx, const Case& c, uint32_t workgroup_size,
                                                  std::size_t* cells_with_dye = nullptr, float* dye_max = nullptr) {
    const af::MacGrid start = noisy(c, 0xD1E5ull, 1.0f);
    auto cpu = cpu_fluid(c, start, true);
    if (!cpu) return std::nullopt;
    const ag::DyeSetup setup{c.release};
    auto gpu = ag::GpuFluid::create(ctx, start, c.bc, c.cfg, solids_of(*cpu), workgroup_size, nullptr, &setup);
    if (!gpu) {
        ADD_FAILURE() << gpu.error().context;
        return std::nullopt;
    }
    std::vector<float> got(cpu->s.dye.size());
    af::MacGrid read = start;
    std::size_t total = 0;
    for (uint32_t step = 0; step < kSteps; ++step) {
        if (cpu->step(c).fault != af::FluidFault::none) {
            ADD_FAILURE() << "the CPU step faulted";
            return std::nullopt;
        }
        const auto h = (*gpu)->step(c.discs);
        if (!h) {
            ADD_FAILURE() << h.error().context;
            return std::nullopt;
        }
        if (const auto ok = (*gpu)->read_dye(got); !ok) {
            ADD_FAILURE() << ok.error().context;
            return std::nullopt;
        }
        if (const auto ok = (*gpu)->read(read); !ok) {
            ADD_FAILURE() << ok.error().context;
            return std::nullopt;
        }
        const std::size_t dd = differing(got, cpu->s.dye);
        const std::size_t df = differing(read, cpu->g);
        std::printf("[ dye     ] %-30s wg %3u step %u: %zu of %zu dye cells differ, %zu flow samples\n", c.name, workgroup_size,
                    step + 1, dd, got.size(), df);
        total += dd + df;
    }
    if (cells_with_dye != nullptr) {
        *cells_with_dye = static_cast<std::size_t>(std::count_if(cpu->s.dye.begin(), cpu->s.dye.end(), [](float x) { return x > 0.0f; }));
    }
    if (dye_max != nullptr) *dye_max = *std::max_element(cpu->s.dye.begin(), cpu->s.dye.end());
    return total;
}

TEST(GpuAirflowDye, TheDyeIsTheCpusBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const Case& c : {open_box(), periodic_channel(), sphere_box()}) {
        SCOPED_TRACE(c.name);
        std::size_t with_dye = 0;
        float dye_max = 0.0f;
        const auto d = run_both(*ctx, c, 64u, &with_dye, &dye_max);
        ASSERT_TRUE(d.has_value());
        const std::size_t cells = std::size_t{c.shape.nx} * c.shape.ny * c.shape.nz;
        EXPECT_GT(with_dye, cells / 50u) << "the dye reached too few cells to prove the advection";
        EXPECT_GT(dye_max, 0.5f);
        EXPECT_LE(dye_max, 1.0f);
        if (bit_equal) {
            EXPECT_EQ(*d, 0u);
        }
    }
}

TEST(GpuAirflowDye, TheDyeIsTheCpusAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const uint32_t wg : {32u, 128u}) {
        const auto d = run_both(*ctx, open_box(), wg);
        ASSERT_TRUE(d.has_value());
        if (bit_equal) {
            EXPECT_EQ(*d, 0u) << "workgroup " << wg;
        }
    }
}

// The publish after the steps is publish_render_volume()'s of the CPU's flow and dye: V's channel 3 is the dye, and with solids
// the first solid layer's channel is the mean of its fluid neighbours' dye.
TEST(GpuAirflowDye, ThePublishCarriesTheDyeInChannelThree) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const Case& c : {periodic_channel(), sphere_box()}) {
        SCOPED_TRACE(c.name);
        const af::MacGrid start = noisy(c, 0x9B1Bull, 1.0f);
        auto cpu = cpu_fluid(c, start, true);
        ASSERT_TRUE(cpu.has_value());
        const ag::DyeSetup setup{c.release};
        auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, solids_of(*cpu), 64u, nullptr, &setup);
        ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
        for (uint32_t step = 0; step < 4; ++step) {
            ASSERT_EQ(cpu->step(c).fault, af::FluidFault::none);
            ASSERT_TRUE((*gpu)->step(c.discs).has_value());
        }
        const glm::vec3 origin{-0.25f, 0.0f, 1.5f};
        spade::FlowVolume want;
        af::PublishInput in{.origin = origin, .periodic = af::periodic_axes(c.bc)};
        in.dye = cpu->s.dye;
        if (cpu->s.solids.any) {
            in.solid = cpu->s.regions.solid;
            for (uint32_t a = 0; a < 3; ++a) in.theta[a] = cpu->s.solids.theta[a];
        }
        ASSERT_TRUE(af::publish_render_volume(cpu->g, in, want).has_value());
        spade::FlowVolume got;
        const auto ok = (*gpu)->publish(origin, got);
        ASSERT_TRUE(ok.has_value()) << ok.error().context;
        ASSERT_EQ(got.v.size(), want.v.size());
        std::size_t d = 0;
        for (std::size_t i = 0; i < want.v.size(); ++i) d += got.v[i] != want.v[i] ? 1u : 0u;
        for (std::size_t i = 0; i < want.s.size(); ++i) d += got.s[i] != want.s[i] ? 1u : 0u;
        std::printf("[ dye     ] publish, %s: %zu of %zu halves differ; dye range [%g, %g]; %u cells extrapolated\n", c.name, d,
                    want.v.size() + want.s.size(), static_cast<double>(want.header.channel_min[spade::kFlowDye]),
                    static_cast<double>(want.header.channel_max[spade::kFlowDye]), want.header.extrapolated_cells);
        EXPECT_GT(want.header.channel_max[spade::kFlowDye], 0.5f) << "the published dye is not vacuous";
        if (cpu->s.solids.any) {
            EXPECT_GT(want.header.extrapolated_cells, 0u);
        }
        if (bit_equal) {
            EXPECT_EQ(d, 0u);
            EXPECT_EQ(got.header.channel_min, want.header.channel_min);
            EXPECT_EQ(got.header.channel_max, want.header.channel_max);
        }
    }
}

// Passive: a fluid created with the dye and one without hold the same u, v, w and p after every step (a second run on the same
// device, so the proof holds wherever it runs).
TEST(GpuAirflowDye, TheFlowIsTheSameBitsWithTheDye) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    for (const Case& c : {open_box(), sphere_box()}) {
        SCOPED_TRACE(c.name);
        const af::MacGrid start = noisy(c, 0xFACEull, 1.0f);
        auto cpu = cpu_fluid(c, start, false);
        ASSERT_TRUE(cpu.has_value());
        const ag::DyeSetup setup{c.release};
        auto with = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, solids_of(*cpu), 64u, nullptr, &setup);
        auto without = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, solids_of(*cpu), 64u);
        ASSERT_TRUE(with.has_value() && without.has_value());
        EXPECT_TRUE((*with)->dye_enabled());
        EXPECT_FALSE((*without)->dye_enabled());
        af::MacGrid a = start, b = start;
        for (uint32_t step = 0; step < kSteps; ++step) {
            ASSERT_TRUE((*with)->step(c.discs).has_value());
            ASSERT_TRUE((*without)->step(c.discs).has_value());
            ASSERT_TRUE((*with)->read(a).has_value());
            ASSERT_TRUE((*without)->read(b).has_value());
            EXPECT_EQ(differing(a, b), 0u) << "step " << step + 1;
        }
    }
}

TEST(GpuAirflowDye, RefusesWhatItCannotCarry) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Case c = open_box();
    const af::MacGrid start = noisy(c, 7u, 1.0f);
    const ag::DyeSetup bad{af::DyeRelease{-1.0f, 0.0f, 1.0f}};
    const auto a = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, nullptr, 64u, nullptr, &bad);
    ASSERT_FALSE(a.has_value());
    EXPECT_EQ(a.error().code, spade::Code::invalid_argument);
    EXPECT_NE(a.error().context.find("rate"), std::string::npos) << a.error().context;
    auto none = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg);
    ASSERT_TRUE(none.has_value()) << none.error().context;
    std::vector<float> out(std::size_t{c.shape.nx} * c.shape.ny * c.shape.nz);
    const auto r = (*none)->read_dye(out);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
    const ag::DyeSetup ok{};
    auto with = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, nullptr, 64u, nullptr, &ok);
    ASSERT_TRUE(with.has_value()) << with.error().context;
    std::vector<float> small(3);
    EXPECT_FALSE((*with)->read_dye(small).has_value());
    ASSERT_TRUE((*with)->read_dye(out).has_value());
    EXPECT_EQ(*std::max_element(out.begin(), out.end()), 0.0f) << "a new dye is clear air";
}

// Opt-in (SPADE_DYE_COST=1): what the dye costs a step in the web clips' deep box (kD1DeepGrid, 15.8 M cells, the quad's solids,
// h 0.2 ms, Cs 0.17, the tip ring released at the quad's four discs): the same fluid stepped without and with the dye, one after the
// other (the two do not fit the device together), the mean wall time of SPADE_DYE_COST_STEPS (default 30) steps after two warm ones,
// and the dye chain's own phase time. PERF lines for the record.
TEST(GpuAirflowDye, TheDyeCostsAStepInTheDeepBox) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (env_cstr("SPADE_DYE_COST") == nullptr) GTEST_SKIP() << "opt-in: SPADE_DYE_COST=1";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const char* steps_env = env_cstr("SPADE_DYE_COST_STEPS");
    const uint32_t steps = steps_env != nullptr ? static_cast<uint32_t>(std::strtoul(steps_env, nullptr, 10)) : 30u;
    constexpr uint32_t kWarm = 2;
    double ms_of[2] = {0.0, 0.0};
    for (int with_dye = 0; with_dye < 2; ++with_dye) {
        spade::Result<b7::B7Box> box = b7::d1_box(b7::BoxSolids::with, spade::testing::kD1DeepGrid);
        ASSERT_TRUE(box.has_value()) << box.error().context;
        box->cfg.h = 0.0002f;
        box->cfg.sgs_cs = 0.17f;
        const ag::DyeSetup setup{af::DyeRelease{}};
        auto gpu = ag::GpuFluid::create(*ctx, box->grid, box->bc, box->cfg, &box->solids, 64u, nullptr,
                                        with_dye != 0 ? &setup : nullptr);
        ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
        double timed = 0.0;
        for (uint32_t step = 0; step < steps + kWarm; ++step) {
            const auto t0 = std::chrono::steady_clock::now();
            const auto h = (*gpu)->step(box->discs);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            ASSERT_TRUE(h.has_value()) << h.error().context;
            ASSERT_EQ(h->fault, af::FluidFault::none);
            if (step >= kWarm) timed += ms;
        }
        ms_of[with_dye] = timed / steps;
        spade::test::perf_line(with_dye != 0 ? "physics.diag_dye_step_deep_with" : "physics.diag_dye_step_deep_without",
                               ms_of[with_dye], "ms");
        spade::test::perf_line(with_dye != 0 ? "physics.diag_dye_mem_deep_with" : "physics.diag_dye_mem_deep_without",
                               static_cast<double>(ctx->memory_budget().usage) / (1024.0 * 1024.0), "MiB");
        if (with_dye != 0) {
            (*gpu)->set_phase_timing(true);
            double dye_ms = 0.0;
            for (uint32_t step = 0; step < 10; ++step) {
                ASSERT_TRUE((*gpu)->step(box->discs).has_value());
                for (const auto& p : (*gpu)->phase_times()) {
                    if (p.name == "dye") dye_ms += p.ms;
                }
            }
            (*gpu)->set_phase_timing(false);
            spade::test::perf_line("physics.diag_dye_phase_deep", dye_ms / 10.0, "ms");
            std::vector<float> dye(std::size_t{box->grid.shape.nx} * box->grid.shape.ny * box->grid.shape.nz);
            ASSERT_TRUE((*gpu)->read_dye(dye).has_value());
            EXPECT_GT(*std::max_element(dye.begin(), dye.end()), 0.0f) << "the deep box's dye is not vacuous";
        }
    }
    std::printf("[ dye     ] deep box: %.2f ms a step without the dye, %.2f with (%+.2f ms, %+.1f%%)\n", ms_of[0], ms_of[1],
                ms_of[1] - ms_of[0], 100.0 * (ms_of[1] - ms_of[0]) / ms_of[0]);
}

}  // namespace
