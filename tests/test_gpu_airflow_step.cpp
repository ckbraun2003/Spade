// ---------------------------------------------------------------------------
// test_gpu_airflow_step.cpp -- plan B, B7's G1 and G3: GpuFluid
// (spike/airflow/step_chain.hpp), the whole fluid step on the device, against
// fluid_step() (engine/physics/airflow/fluid_step.cpp) in LiveRun's order:
// each step's discs spread into du (spread_actuator()), then the step.
//
//   GpuAirflowStep   bit for bit on the device of record
//                    (airflow_exact_device.hpp; elsewhere reported only),
//                    u, v, w and p after every step: an open box in a wind
//                    with two discs and the air's viscosity; a periodic
//                    channel with one disc; a closed box under a moving lid,
//                    inviscid and with no disc (the closed domain's means).
//                    With solid parts (G3), each step's CG report and wall
//                    load too: a spinning sphere in a closed box; a turned
//                    box in the open box's wind; a cylinder with the
//                    free-slip interior, inviscid; a hollow box whose cavity
//                    is a closed region; the periodic channel with a sphere;
//                    a block over the inflow face, viscous; a region that
//                    can't balance, faulted as on the CPU. The
//                    same bits at workgroup sizes 32, 64 and 128; the publish
//                    equal to publish_render_volume()'s, with solids too; the
//                    refusals; with SPADE_AIRFLOW_TIMINGS=1, a step's time in
//                    the D1 box, the GPU's beside the CPU's, and with its
//                    solids, the GPU's with its CG iterations.
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
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_exact_device.hpp"
#include "airflow_solid_shapes.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "b7_harness.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/flow_volume.hpp"
#include "d1_case.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/regions.hpp"
#include "physics/airflow/wall_load.hpp"
#include "spike/airflow/step_chain.hpp"
#include "vehicles/rotor_discs.hpp"

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

// One configuration: the box, its faces, the step and the discs; with G3,
// its solid parts and the wall load's moment point.
struct Case {
    const char* name;
    af::GridShape shape;
    af::DomainBc bc;
    af::FluidConfig cfg;
    std::vector<af::ActuatorDisc> discs;
    std::vector<airflow_test::Shape> shapes{};  // set_solids()' parts; none: plan A's paths
    glm::vec3 about{0.0f};                      // the wall load's moment point, the grid's frame
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

// The open box on odd sizes, as D1's 177 cells along y are: the multigrid's
// odd axes (Physics' mg-odd-axes rule) under the whole step. D1's --gpu film
// went non-finite on every grid with an odd ny and on none without (G2,
// 2026-10-27).
[[nodiscard]] Case odd_open_box() {
    Case c = open_box();
    c.name = "open box, odd sizes 25x21x19";
    c.shape = {25, 21, 19, 0.02f};
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

// --- G3: the boxes with solid parts (the CG floor kSolidsCgIterations) ------

[[nodiscard]] af::FluidConfig solids_config(float nu, float h) {
    af::FluidConfig cfg = config(nu, h);
    cfg.n_v = af::kSolidsCgIterations;
    return cfg;
}

// Walls all round, a sphere spinning about y below a disc: the rigid
// interior's moving walls, B3's viscous wall links, one region.
[[nodiscard]] Case sphere_box() {
    Case c{"closed box, spinning sphere", {20, 24, 16, 0.02f}, {}, solids_config(kNu, 0.002f), {}};
    c.bc.face.fill(af::FaceBc::wall);
    c.discs = {disc({0.20f, 0.38f, 0.16f}, 1.0f, 0.01f)};
    c.about = {0.20f, 0.18f, 0.16f};
    c.shapes = {{.name = "sphere", .kind = airflow_test::Kind::sphere, .centre = c.about, .radius = 0.08f,
                 .omega = {0.0f, 15.0f, 0.0f}, .min_feature = 0.16f}};
    return c;
}

// The open box's wind and discs with a box turned 30 degrees about y beneath
// them: cut faces at every fraction.
[[nodiscard]] Case part_open_box() {
    Case c = open_box();
    c.name = "open box, turned box part";
    c.cfg = solids_config(kNu, 0.002f);
    c.about = {0.24f, 0.12f, 0.18f};
    const float co = 0.8660254f;  // cos 30 degrees
    const float si = 0.5f;
    c.shapes = {{.name = "box", .kind = airflow_test::Kind::box, .centre = c.about, .half = {0.06f, 0.04f, 0.05f},
                 .to_local = glm::mat3(glm::vec3(co, 0.0f, si), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(-si, 0.0f, co)),
                 .min_feature = 0.08f}};
    return c;
}

// Inviscid, the free-slip interior (SolidInterior::extrapolate, its two
// fill layers): the wind past a cylinder along y, through the open faces.
[[nodiscard]] Case slip_cylinder() {
    Case c = open_box();
    c.name = "open box, cylinder, free slip";
    c.cfg = solids_config(0.0f, 0.002f);
    c.cfg.interior = af::SolidInterior::extrapolate;
    c.discs = {disc({0.12f, 0.26f, 0.13f}, 1.2f, 0.02f)};
    c.about = {0.32f, 0.20f, 0.18f};
    c.shapes = {{.name = "cylinder", .kind = airflow_test::Kind::cylinder_y, .centre = c.about, .radius = 0.06f,
                 .min_feature = 0.12f}};
    return c;
}

// A hollow box in the wind: its cavity is a closed region, so the CG's gauge
// removes that region's own mean (the region table, airflow_region_mean).
[[nodiscard]] Case cavity_box() {
    Case c = open_box();
    c.name = "open box, sealed cavity";
    c.cfg = solids_config(kNu, 0.002f);
    c.about = {0.24f, 0.12f, 0.18f};
    c.shapes = {{.name = "shell", .kind = airflow_test::Kind::box_shell, .centre = c.about,
                 .half = {0.10f, 0.08f, 0.10f}, .radius = 0.04f, .min_feature = 0.04f}};
    return c;
}

// The periodic channel with a sphere in the disc's wake: the solids' faces
// and regions with the advection's wrap and the viscous solve's alias faces.
// The sphere stays 2 dx clear of both seams (set_solids' rule).
[[nodiscard]] Case part_channel() {
    Case c = periodic_channel();
    c.name = "periodic channel, sphere";
    c.cfg = solids_config(kNu, 0.002f);
    c.about = {0.30f, 0.11f, 0.15f};
    c.shapes = {{.name = "sphere", .kind = airflow_test::Kind::sphere, .centre = c.about, .radius = 0.06f,
                 .min_feature = 0.12f}};
    return c;
}

// A still box over the open -x face, the wind entering there, viscous: after
// the viscous solve's boundaries the faces it covers take the box's 0 again,
// not the inflow (fluid_step's third apply_solid_velocities; the CPU's
// AirflowSolidsStep.ADomainFaceUnderASolidKeepsTheSolidsVelocity).
[[nodiscard]] Case covered_inflow_box() {
    Case c{"open box, a block over the inflow", {16, 8, 8, 0.1f}, {}, solids_config(kNu, 0.005f), {}};
    c.bc.ambient = {1.0f, 0.0f, 0.0f};
    c.about = {0.0f, 0.4f, 0.4f};
    c.shapes = {{.name = "block", .kind = airflow_test::Kind::box, .centre = c.about, .half = {0.25f, 0.2f, 0.2f},
                 .min_feature = 0.4f}};
    return c;
}

// Walls but an outlet at -x, a plate across the whole box at mid x, and the
// +x wall pushing in at 1 m/s: the domain has its outlet (check_domain()
// passes), but the plate seals the +x half into a closed region that takes
// the wall's flux, so check_regions() faults the step.
[[nodiscard]] Case ill_posed_box() {
    Case c{"walled box, a plate, a wall pushing in", {20, 24, 16, 0.02f}, {}, solids_config(kNu, 0.002f), {}};
    c.bc.face.fill(af::FaceBc::wall);
    c.bc.face[af::kXMinus] = af::FaceBc::open;
    c.bc.wall_velocity[af::kXPlus] = {-1.0f, 0.0f, 0.0f};
    c.about = {0.20f, 0.24f, 0.16f};
    c.shapes = {{.name = "plate", .kind = airflow_test::Kind::box, .centre = c.about, .half = {0.03f, 0.30f, 0.22f},
                 .min_feature = 0.06f}};
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

// With the case's parts, its solids set as LiveRun sets them (set_solids());
// GpuFluid takes the field they leave.
[[nodiscard]] std::optional<CpuFluid> cpu_fluid(const Case& c, const af::MacGrid& start) {
    auto s = af::make_fluid_scratch(start, c.bc, c.cfg);
    if (!s) {
        ADD_FAILURE() << s.error().context;
        return std::nullopt;
    }
    if (!c.shapes.empty()) {
        const airflow_test::Scene scene{c.shapes};
        if (const auto ok = af::set_solids(*s, start, c.bc, scene.parts(), scene.sampler()); !ok) {
            ADD_FAILURE() << ok.error().context;
            return std::nullopt;
        }
        if (!s->solids.any) {
            ADD_FAILURE() << c.name << ": the parts left no solid";
            return std::nullopt;
        }
    }
    return CpuFluid{start, std::move(*s)};
}

// The field GpuFluid::create() takes for the CPU's run: its solids, or none.
[[nodiscard]] const af::SolidField* solids_of(const CpuFluid& cpu) {
    return cpu.s.solids.any ? &cpu.s.solids : nullptr;
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

[[nodiscard]] std::size_t non_finite(const af::MacGrid& g) {
    std::size_t n = 0;
    for (const std::vector<float>* a : {&g.u, &g.v, &g.w, &g.p}) {
        n += static_cast<std::size_t>(std::count_if(a->begin(), a->end(), [](float x) { return !std::isfinite(x); }));
    }
    return n;
}

[[nodiscard]] std::size_t differing(float a, float b) {
    return std::bit_cast<uint32_t>(a) != std::bit_cast<uint32_t>(b) ? 1u : 0u;
}

[[nodiscard]] std::size_t differing(glm::vec3 a, glm::vec3 b) {
    return differing(a.x, b.x) + differing(a.y, b.y) + differing(a.z, b.z);
}

// The health fields fluid_step() fills from the projection's report (the CG's with solids, plan A's
// residual without, 0fcb99f9): how many of the five differ.
[[nodiscard]] std::size_t health_differing(const af::FluidHealth& cpu, const af::FluidHealth& gpu) {
    return differing(cpu.pressure_residual, gpu.pressure_residual) +
           (cpu.pressure_iterations != gpu.pressure_iterations ? 1u : 0u) +
           (cpu.unconverged != gpu.unconverged ? 1u : 0u) + (cpu.capped != gpu.capped ? 1u : 0u) +
           (cpu.stalled != gpu.stalled ? 1u : 0u);
}

// Steps `c` kSteps times on both sides from the same noise, comparing after
// every step: u, v, w and p; with solids, the health's CG report and the
// wall load about c.about too. The count of differing values summed over the
// steps, or nullopt on a refusal.
[[nodiscard]] std::optional<std::size_t> run_both(VulkanContext& ctx, const Case& c, uint32_t workgroup_size,
                                                  af::MacGrid* gpu_end = nullptr) {
    const af::MacGrid start = noisy(c, 0x57E9ull, 1.0f);
    auto cpu = cpu_fluid(c, start);
    if (!cpu) return std::nullopt;
    auto gpu = ag::GpuFluid::create(ctx, start, c.bc, c.cfg, solids_of(*cpu), workgroup_size);
    if (!gpu) {
        ADD_FAILURE() << gpu.error().context;
        return std::nullopt;
    }
    const bool solids = !c.shapes.empty();
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
        if (!solids) {
            // Plan A's report (0fcb99f9): the device's residual after its cycles is the CPU's, bit for
            // bit, and the CPU's is not zero, so a device that says 0 is wrong.
            EXPECT_EQ(g->pressure_iterations, c.cfg.n_v);
            EXPECT_GT(h.pressure_residual, 0.0f) << c.name << " step " << step + 1;
            const std::size_t dh = health_differing(h, *g);
            std::printf("[ step    ] %-28s   plan A residual %.6g (CPU %.6g); %zu of 5 health values differ\n", "",
                        static_cast<double>(g->pressure_residual), static_cast<double>(h.pressure_residual), dh);
            total += dh;
        }
        if (const auto ok = (*gpu)->read(read); !ok) {
            ADD_FAILURE() << ok.error().context;
            return std::nullopt;
        }
        const std::size_t d = differing(read, cpu->g);
        std::printf("[ step    ] %-28s wg %3u step %u: %zu of %zu samples differ; non-finite: CPU %zu, GPU %zu\n",
                    c.name, workgroup_size, step + 1, d, samples_of(read), non_finite(cpu->g), non_finite(read));
        total += d;
        if (solids) {
            const af::WallLoad want = af::wall_load(cpu->g, cpu->s.solids, cpu->s.regions, c.about);
            const auto got = (*gpu)->wall_load(c.about);
            if (!got) {
                ADD_FAILURE() << got.error().context;
                return std::nullopt;
            }
            const std::size_t dh = health_differing(h, *g);
            const std::size_t dl = differing(want.force, got->force) + differing(want.moment, got->moment);
            std::printf("[ step    ] %-28s   CG %u iterations, residual %.3g (CPU %u, %.3g); %zu of 5 health and %zu "
                        "of 6 wall-load values differ; force (%.4g, %.4g, %.4g) N\n",
                        "", g->pressure_iterations, static_cast<double>(g->pressure_residual), h.pressure_iterations,
                        static_cast<double>(h.pressure_residual), dh, dl, static_cast<double>(want.force.x),
                        static_cast<double>(want.force.y), static_cast<double>(want.force.z));
            total += dh + dl;
        }
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

TEST(GpuAirflowStep, OddSizedOpenBoxMatchesFluidStepBitForBit) { expect_bit_equal(odd_open_box()); }

TEST(GpuAirflowStep, PeriodicChannelMatchesFluidStepBitForBit) { expect_bit_equal(periodic_channel()); }

// The step makes ONE fence wait (0fcb99f9): the plan A health word comes from the chain's own readback, not a second
// submit. The first step also uploads the discs' records (its own waits); the second, with the same discs, is one.
TEST(GpuAirflowStep, AStepWithoutSolidsMakesOneFenceWait) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Case c = open_box();
    const af::MacGrid start = noisy(c, 0x57E9ull, 1.0f);
    auto cpu = cpu_fluid(c, start);
    ASSERT_TRUE(cpu.has_value());
    auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, solids_of(*cpu), 64u);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    ASSERT_TRUE((*gpu)->step(c.discs).has_value());
    for (int step = 0; step < 3; ++step) {
        const uint64_t before = (*gpu)->waits();
        const auto h = (*gpu)->step(c.discs);
        ASSERT_TRUE(h.has_value()) << h.error().context;
        EXPECT_EQ((*gpu)->waits() - before, 1u) << "step " << step + 2;
        EXPECT_GT(h->pressure_residual, 0.0f);
    }
}

// Plan A's health (0fcb99f9) at every cycle count a run may set: the open, odd, periodic and closed (remove_mean)
// domains, residual and announce equal to the CPU's bit for bit. run_both() compares them every step.
TEST(GpuAirflowStep, PlanAHealthMatchesFluidStepAtEveryVCycleCount) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const Case& base : {open_box(), odd_open_box(), periodic_channel(), lid_box()}) {
        for (uint32_t n_v = 1; n_v <= 4; ++n_v) {
            Case c = base;
            c.cfg.n_v = n_v;
            const auto d = run_both(*ctx, c, 64u);
            ASSERT_TRUE(d.has_value()) << c.name << " n_v " << n_v;
            if (bit_equal) {
                EXPECT_EQ(*d, 0u) << c.name << " n_v " << n_v;
            }
        }
    }
}

TEST(GpuAirflowStep, ClosedLidBoxMatchesFluidStepBitForBit) { expect_bit_equal(lid_box()); }

// G3: the step with solid parts, each step's CG report and wall load too.
TEST(GpuAirflowStep, SpinningSphereInAClosedBoxMatchesFluidStepBitForBit) { expect_bit_equal(sphere_box()); }

TEST(GpuAirflowStep, TurnedPartInTheOpenBoxMatchesFluidStepBitForBit) { expect_bit_equal(part_open_box()); }

TEST(GpuAirflowStep, FreeSlipCylinderMatchesFluidStepBitForBit) { expect_bit_equal(slip_cylinder()); }

TEST(GpuAirflowStep, PeriodicChannelWithASphereMatchesFluidStepBitForBit) { expect_bit_equal(part_channel()); }

TEST(GpuAirflowStep, ABlockOverTheInflowMatchesFluidStepBitForBit) { expect_bit_equal(covered_inflow_box()); }

// The cavity is a closed region of its own (else the case proves nothing of
// the gauge), and the step matches with it.
TEST(GpuAirflowStep, SealedCavityMatchesFluidStepBitForBit) {
    const Case c = cavity_box();
    auto cpu = cpu_fluid(c, noisy(c, 1u, 1.0f));
    ASSERT_TRUE(cpu.has_value());
    ASSERT_EQ(cpu->step(c).fault, af::FluidFault::none);
    const auto closed = std::count_if(cpu->s.regions.closed.begin(), cpu->s.regions.closed.end(),
                                      [](uint8_t x) { return x != 0u; });
    std::printf("[ step    ] %s: %u regions, %td closed\n", c.name, cpu->s.regions.count, closed);
    ASSERT_GE(closed, 1);
    expect_bit_equal(c);
}

// A region that can't balance: the CPU's step faults without stepping, and
// GpuFluid's every step returns the same fault, its faces and cell, leaving
// the flow as it was.
TEST(GpuAirflowStep, AnIllPosedRegionFaultsAsOnTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Case c = ill_posed_box();
    const af::MacGrid start = noisy(c, 7u, 1.0f);
    auto cpu = cpu_fluid(c, start);
    ASSERT_TRUE(cpu.has_value());
    const af::FluidHealth want = cpu->step(c);
    ASSERT_EQ(want.fault, af::FluidFault::ill_posed_region);
    EXPECT_EQ(want.faces, 1u << af::kXPlus);  // the pushing wall, beyond the plate
    auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, solids_of(*cpu));
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    for (uint32_t step = 0; step < 2; ++step) {
        const auto got = (*gpu)->step(c.discs);
        ASSERT_TRUE(got.has_value()) << got.error().context;
        EXPECT_EQ(got->fault, want.fault);
        EXPECT_EQ(got->faces, want.faces);
        EXPECT_EQ(got->index, want.index);
    }
    af::MacGrid read = start;
    ASSERT_TRUE((*gpu)->read(read).has_value());
    EXPECT_EQ(differing(read, start), 0u);
}

// One NaN in an open u face above the spinning sphere: the CPU's step scans
// it (non_finite), and GpuFluid's faults on the CG's r . z, which the NaN
// reaches through the divergence -- naming the pressure, with no sample
// (Physics' review of G3, 2026-10-28). The report alone would read healthy:
// without the r . z check the device's residual is 0 after its 8 iterations,
// its max-norms passing over the NaN.
TEST(GpuAirflowStep, ANanWithSolidsFaultsAsOnTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Case c = sphere_box();
    af::MacGrid start = noisy(c, 7u, 1.0f);
    start.u[af::u_index(c.shape, c.shape.nx / 2, c.shape.ny - 4, c.shape.nz / 2)] =
        std::numeric_limits<float>::quiet_NaN();
    auto cpu = cpu_fluid(c, start);
    ASSERT_TRUE(cpu.has_value());
    const af::FluidHealth want = cpu->step(c);
    ASSERT_EQ(want.fault, af::FluidFault::non_finite);
    auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, solids_of(*cpu));
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    const auto got = (*gpu)->step(c.discs);
    ASSERT_TRUE(got.has_value()) << got.error().context;
    std::printf("[ step    ] a NaN with solids: CPU fault %d (quantity %u, index %zu); GPU fault %d (quantity %u)\n",
                static_cast<int>(want.fault), static_cast<unsigned>(want.quantity), want.index,
                static_cast<int>(got->fault), static_cast<unsigned>(got->quantity));
    EXPECT_EQ(got->fault, af::FluidFault::non_finite);
    EXPECT_EQ(got->quantity, 3u);  // the pressure: the CG's r . z is what the device reads back
    EXPECT_EQ(got->index, af::kNoSample);  // and no sample: the device scans none
    const std::string why = af::describe(*got, c.shape);
    EXPECT_NE(why.find("found on the device"), std::string::npos) << why;

    // The fault is kept, as the CPU's is: the next step returns it without
    // stepping, the fields as the faulted step left them.
    af::MacGrid faulted = start;
    ASSERT_TRUE((*gpu)->read(faulted).has_value());
    const auto again = (*gpu)->step(c.discs);
    ASSERT_TRUE(again.has_value()) << again.error().context;
    EXPECT_EQ(again->fault, af::FluidFault::non_finite);
    EXPECT_EQ(again->quantity, 3u);
    EXPECT_EQ(cpu->step(c).fault, af::FluidFault::non_finite);
    af::MacGrid after = start;
    ASSERT_TRUE((*gpu)->read(after).has_value());
    EXPECT_EQ(differing(after, faulted), 0u);
}

// No invocation reads another's result within a dispatch, and the closed
// domain's sums run in reduce.hpp's fixed blocks: the same bits at every
// supported workgroup size, on any device, with solids too (the CG's dots,
// max-norm and region means in fixed blocks).
TEST(GpuAirflowStep, SameBitsAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    for (const Case& c : {open_box(), lid_box(), sphere_box(), cavity_box()}) {
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
// the same halves and header, on the device of record. With solids, LiveRun's
// publish input: the solid mask (the regions' solid cells) and the apertures,
// so the first solid layer is extrapolated and a difference never crosses a
// closed face.
TEST(GpuAirflowStep, PublishMatchesTheCpuVolume) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const Case& c : {periodic_channel(), sphere_box()}) {
        SCOPED_TRACE(c.name);
        const af::MacGrid start = noisy(c, 0x9B1Bull, 1.0f);
        auto cpu = cpu_fluid(c, start);
        ASSERT_TRUE(cpu.has_value());
        auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg, solids_of(*cpu));
        ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
        for (uint32_t step = 0; step < 2; ++step) {
            ASSERT_EQ(cpu->step(c).fault, af::FluidFault::none);
            ASSERT_TRUE((*gpu)->step(c.discs).has_value());
        }
        const glm::vec3 origin{-0.25f, 0.0f, 1.5f};
        spade::FlowVolume want;
        af::PublishInput in{.origin = origin, .periodic = af::periodic_axes(c.bc)};
        if (cpu->s.solids.any) {
            in.solid = cpu->s.regions.solid;
            for (uint32_t a = 0; a < 3; ++a) in.theta[a] = cpu->s.solids.theta[a];
        }
        ASSERT_TRUE(af::publish_render_volume(cpu->g, in, want).has_value());
        spade::FlowVolume got;
        const auto ok = (*gpu)->publish(origin, got);
        ASSERT_TRUE(ok.has_value()) << ok.error().context;
        ASSERT_EQ(got.v.size(), want.v.size());
        ASSERT_EQ(got.s.size(), want.s.size());
        std::size_t d = 0;
        for (std::size_t i = 0; i < want.v.size(); ++i) d += got.v[i] != want.v[i] ? 1u : 0u;
        for (std::size_t i = 0; i < want.s.size(); ++i) d += got.s[i] != want.s[i] ? 1u : 0u;
        std::printf("[ step    ] publish, %s: %zu of %zu halves differ; %u cells extrapolated\n", c.name, d,
                    want.v.size() + want.s.size(), want.header.extrapolated_cells);
        EXPECT_EQ(got.header.origin, want.header.origin);
        EXPECT_EQ(got.header.cell, want.header.cell);
        EXPECT_EQ(got.header.nx, want.header.nx);
        EXPECT_EQ(got.header.extrapolated_cells, want.header.extrapolated_cells);
        if (cpu->s.solids.any) {
            EXPECT_GT(want.header.extrapolated_cells, 0u);
        }
        if (bit_equal) {
            EXPECT_EQ(d, 0u);
            EXPECT_EQ(got.header.channel_min, want.header.channel_min);
            EXPECT_EQ(got.header.channel_max, want.header.channel_max);
            EXPECT_EQ(got.header.no_data, want.header.no_data);
        }
    }
}

// What GpuFluid refuses: make_fluid_scratch()'s config refusals, a
// SolidField not made for the grid's shape (attach_solid_field()), more than
// kMaxDiscs discs.
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
    EXPECT_EQ(b.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(b.error().context, "airflow solids: the solid field was not made for this grid's shape");
    auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    const std::vector<af::ActuatorDisc> many(ag::kMaxDiscs + 1u, c.discs.front());
    const auto s = (*gpu)->step(many);
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::capacity_exceeded);
}

// A step's time in the D1 box (240 x 177 x 168 at 4 mm, the air's nu, h
// 0.38 ms) with four discs about its centre, no read-back: the mean of 20
// steps after 2 to warm up. Beside it, the CPU's step on the same box from
// the same start (CpuFluid: spread_actuator() and fluid_step(), plan A with
// no parts, LiveRun's step without solids), the mean of 3 after 1, so the
// speed-up compares like with like (the lead, 2026-10-27). Opt-in with
// SPADE_AIRFLOW_TIMINGS, as the kernels' D1 times are.
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

    auto cpu = cpu_fluid(c, start);
    ASSERT_TRUE(cpu.has_value());
    ASSERT_EQ(cpu->step(c).fault, af::FluidFault::none);
    constexpr int kCpuTimed = 3;
    const auto c0 = std::chrono::steady_clock::now();
    for (int step = 0; step < kCpuTimed; ++step) ASSERT_EQ(cpu->step(c).fault, af::FluidFault::none);
    const double cpu_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count() / kCpuTimed;
    std::printf("measure: fluid_step on the CPU (plan A, no parts) in the same box: %.1f ms; the GPU's %.0fx faster\n",
                cpu_ms, cpu_ms / ms);
}

// B7a's first full-GPU D1 step (G3): the D1 box with the reference quad's
// rigid parts as solids (b7_harness.hpp's d1_box(): LiveRun's own solids and
// discs, the CG floor kSolidsCgIterations), from rest. Each step's time with
// its CG iterations and residual, the host loop's reads included; the mean of
// 10 after 2 to warm up. GPU only: the CPU's step there is about 23 s, and
// B7's harness compares the two (test_gpu_airflow_d1_device.cpp, opt-in).
// Opt-in with SPADE_AIRFLOW_TIMINGS.
TEST(GpuAirflowStep, StepTimeInTheD1BoxWithItsSolids) {
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
    const spade::Result<b7::B7Box> box = b7::d1_box(b7::BoxSolids::with);
    ASSERT_TRUE(box.has_value()) << box.error().context;
    auto gpu = ag::GpuFluid::create(*ctx, box->grid, box->bc, box->cfg, &box->solids);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    constexpr int kWarm = 2;
    constexpr int kTimed = 10;
    double timed_ms = 0.0;
    std::string iterations;
    for (int step = 0; step < kWarm + kTimed; ++step) {
        const auto t0 = std::chrono::steady_clock::now();
        const auto h = (*gpu)->step(box->discs);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        ASSERT_TRUE(h.has_value()) << h.error().context;
        ASSERT_EQ(h->fault, af::FluidFault::none);
        if (step >= kWarm) timed_ms += ms;
        iterations += (iterations.empty() ? "" : " ") + std::to_string(h->pressure_iterations);
        std::printf("[ d1      ] step %d: %.1f ms, CG %u iterations, residual %.3g%s%s\n", step + 1, ms,
                    h->pressure_iterations, static_cast<double>(h->pressure_residual), h->capped ? ", capped" : "",
                    h->stalled ? ", stalled" : "");
    }
    std::printf("measure: GpuFluid step in the D1 box with its solids (%s), CG floor %u cap %u: %.1f ms a step (the "
                "mean of %d after %d); CG iterations by step: %s\n",
                box->solids.run_line.c_str(), box->cfg.n_v, box->cfg.cg_cap, timed_ms / kTimed, kTimed, kWarm,
                iterations.c_str());
}

// The --gpu film's case (flow_movie's gpu_live.cpp, B7's G2) against the CPU,
// step by step: D1's case without its parts on a 112 x 81 x 112 box at 4 mm,
// its discs formed as gpu_live forms them (LiveRun's rows after one step at
// the program's controls, each at its command), from rest; each GPU step read
// back beside spread_actuator() and fluid_step(): the same bits while both
// are finite, and both non-finite from the same step if either goes. Whether
// the case blows up is reported, not asserted: it does, on the CPU first
// (2026-10-27: bit for bit through step 67, both non-finite at 68, the CPU's
// own fault; solid-free D1 under plan A, Physics' triage), so D1's --gpu film
// went non-finite on every grid with an odd ny. Opt-in with
// SPADE_AIRFLOW_VALIDATION: the CPU's steps take minutes.
TEST(GpuAirflowStep, D1NoPartsCaseStepByStepAgainstTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const bool validation = std::getenv("SPADE_AIRFLOW_VALIDATION") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (!validation) GTEST_SKIP() << "opt-in: SPADE_AIRFLOW_VALIDATION=1";
    namespace an = spade::analysis;
    auto q = spade::testing::d1_case(0.1, af::GridShape{112, 81, 112, 0.004f}, false);
    ASSERT_TRUE(q.has_value()) << q.error().context;
    const auto resolved = spade::testing::resolve_with_grid(q->c, q->run_model, q->grid);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    const af::GridShape& s = resolved->base;
    const glm::vec3 origin = -0.5f * s.dx * glm::vec3(static_cast<float>(s.nx), static_cast<float>(s.ny),
                                                      static_cast<float>(s.nz));
    std::vector<spade::vehicles::RotorRow> rows;
    spade::BodyState body{};
    {
        auto run = an::LiveRun::start(*resolved, q->run_model);
        ASSERT_TRUE(run.has_value()) << run.error().context;
        ASSERT_FALSE(run->solids().any);
        ASSERT_TRUE(run->step(an::controls_at(q->program, 0.0f)).has_value());
        rows.assign(run->rotor_rows().begin(), run->rotor_rows().end());
        body = run->body();
    }
    for (spade::vehicles::RotorRow& r : rows) r.omega = r.omega_cmd;
    Case c{"D1 no parts 112x81x112", s, resolved->domain, {}, std::vector<af::ActuatorDisc>(rows.size())};
    ASSERT_TRUE(spade::vehicles::rotor_discs(rows, body, origin, s, c.discs).has_value());
    const auto h_ns = static_cast<uint64_t>(static_cast<double>(*resolved->spec.time.step) * 1e9 + 0.5);
    c.cfg.rho = resolved->spec.fluid.density;
    c.cfg.nu = resolved->spec.fluid.kinematic_viscosity;
    c.cfg.h = static_cast<float>(static_cast<double>(h_ns) / 1e9);
    const af::MacGrid start = *af::make_grid(s, c.bc);

    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    auto cpu = cpu_fluid(c, start);
    ASSERT_TRUE(cpu.has_value());
    auto gpu = ag::GpuFluid::create(*ctx, start, c.bc, c.cfg);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    af::MacGrid read = start;
    uint32_t first_differing = 0, cpu_bad = 0, gpu_bad = 0;
    // Until either side goes non-finite: a NaN's payload is not compared.
    for (uint32_t step = 1; step <= 100u && cpu_bad == 0u && gpu_bad == 0u; ++step) {
        const af::FluidHealth h = cpu->step(c);
        ASSERT_TRUE((*gpu)->step(c.discs).has_value());
        ASSERT_TRUE((*gpu)->read(read).has_value());
        const std::size_t nc = non_finite(cpu->g), ng = non_finite(read);
        const std::size_t d = nc == 0u && ng == 0u ? differing(read, cpu->g) : 0u;
        if (d > 0u && first_differing == 0u) first_differing = step;
        if (nc > 0u) cpu_bad = step;
        if (ng > 0u) gpu_bad = step;
        if (step % 10u == 0u || (d > 0u && first_differing == step) || nc > 0u || ng > 0u) {
            std::printf("[ d1 gpu  ] step %u: %zu of %zu samples differ; non-finite: CPU %zu (fault %d), GPU %zu\n",
                        step, d, samples_of(read), nc, static_cast<int>(h.fault), ng);
            std::fflush(stdout);
        }
    }
    std::printf("[ d1 gpu  ] first differing step %u; first non-finite: CPU %u, GPU %u (0: none in 100 steps)\n",
                first_differing, cpu_bad, gpu_bad);
    EXPECT_EQ(gpu_bad, cpu_bad) << "the GPU goes non-finite when, and only when, the CPU does";
    if (spade::testing::airflow_bit_equal_device(*ctx)) {
        EXPECT_EQ(first_differing, 0u);
    }
}
