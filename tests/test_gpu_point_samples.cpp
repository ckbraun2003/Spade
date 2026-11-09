// test_gpu_point_samples.cpp -- point samples on the device (module API stage 7, Task D1; the stage 7 plan, batch D).
//
// A set with a position-dependent provider records the points.positions kernel and drag.forces' SECOND variant
// (forces_drag_points, L40), and its airflow_points array is bound at set 1 (names are the contract). A standard set records
// the standard drag kernel, binds no set 1, and is untouched. The device rows are held to the CPU's: positions and the module's
// own rows exactly (the positions pass is a copy, the provider's arithmetic is exact in fp32), the drag force within the band.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "compute/backend.hpp"
#include "gpu_skip.hpp"
#include "point_samples_gpu_support.hpp"
#include "points_proof_support.hpp"
#include "probe_module_support.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace {

using spade::compute::BackendDesc;
using spade::Simulation;
using spade::compute::BackendKind;
using spade::modules::PointRow;
using spade::test::PointsGpuModule;

constexpr uint32_t kBodies = 4;
constexpr uint32_t kSensors = 2;
constexpr uint32_t kRowsPerWorld = kBodies + 4u * spade::test::kPointsK + kSensors + spade::test::kPointsHostRows;

[[nodiscard]] Simulation make(const spade::modules::ModuleSet& set, BackendKind kind, uint32_t substeps = 1) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(kBodies);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<Simulation> sim = Simulation::create(*world, 1'000'000, substeps, BackendDesc{.kind = kind}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

}  // namespace

// PIN: a standard set on the device is what it was: no points kernels, no second variant, one descriptor set.
TEST(GpuPointSamples, AStandardSetRecordsNoPointsKernelAndBindsNoSetOne) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation sim = make(spade::modules::standard_modules(), BackendKind::vulkan);
    ASSERT_TRUE(sim.vulkan_descriptor_set_count().has_value());
    EXPECT_EQ(*sim.vulkan_descriptor_set_count(), 1u);
    ASSERT_TRUE(sim.vulkan_pipeline_count().has_value());
    EXPECT_EQ(*sim.vulkan_pipeline_count(), 12u);
    const auto chain = sim.vulkan_recorded_chain();
    ASSERT_TRUE(chain.has_value());
    EXPECT_EQ(std::ranges::count_if(chain->passes, [](const std::string& n) { return n.starts_with("points."); }), 0);
}

// The chain of a set with a provider: points.positions after behaviors.kinematic, the provider's pass, drag.forces after both;
// the pipelines are the standard twelve with drag's variant swapped (not added), plus positions and the provider's: 14.
// A set-1 descriptor set is built (airflow_points).
TEST(GpuPointSamples, ASetWithAProviderRecordsThePositionsKernelAndTheSecondDragVariant) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const PointsGpuModule m;
    Simulation sim = make(m.set, BackendKind::vulkan);
    const auto chain = sim.vulkan_recorded_chain();
    ASSERT_TRUE(chain.has_value()) << chain.error().context;
    const auto at = [&](const std::string& name) { return std::ranges::find(chain->passes, name) - chain->passes.begin(); };
    ASSERT_NE(std::ranges::find(chain->passes, "points.positions"), chain->passes.end());
    EXPECT_LT(at("behaviors.kinematic"), at("points.positions"));
    EXPECT_LT(at("points.positions"), at("gusts.sample"));
    EXPECT_LT(at("gusts.sample"), at("drag.forces"));
    ASSERT_TRUE(sim.vulkan_pipeline_count().has_value());
    EXPECT_EQ(*sim.vulkan_pipeline_count(), 14u) << "12 standard (drag's variant swapped in, not added) + positions + the provider's";
    ASSERT_TRUE(sim.vulkan_descriptor_set_count().has_value());
    EXPECT_EQ(*sim.vulkan_descriptor_set_count(), 2u) << "set 0 and the set-1 set airflow_points binds in";
}

// The reader kernel reads the per-point row behind the variant (L40): the same bodies, the same drag element, but the rows'
// wind differs from the world's -- on the device and on the CPU alike the drag force follows the ROW, and the device equals the
// CPU within the band; and the device's rows equal the CPU's exactly.
TEST(GpuPointSamples, TheReaderKernelReadsThePerPointRowBehindTheVariant) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const PointsGpuModule m;
    struct Run {
        std::vector<PointRow> rows;
        glm::vec3 force{0.0f};
    };
    const auto run = [&](BackendKind kind) {
        Simulation sim = make(m.set, kind);
        const auto body = sim.spawn(0, spade::test::bare_body(glm::vec3(1.0f, 2.0f, 3.0f)));
        EXPECT_TRUE(body.has_value());
        spade::DragElementSpawn drag;
        drag.mode = spade::physics::drag_mode::quadratic;
        drag.area = 0.5f;
        drag.coeffs = glm::vec3(1.0f, 0.0f, 0.0f);
        EXPECT_TRUE(sim.add_drag_element(*body, drag).has_value());
        EXPECT_TRUE(sim.step(1).has_value());
        const auto rows = sim.module_rows<PointRow>("airflow_points", 0);
        EXPECT_TRUE(rows.has_value());
        Run out;
        out.rows.assign(rows->begin(), rows->end());
        out.force = spade::test::row_of(sim, *body).vel;  // the drag impulse shows in the velocity after one step
        return out;
    };
    const Run cpu = run(BackendKind::cpu);
    const Run gpu = run(BackendKind::vulkan);
    ASSERT_EQ(cpu.rows.size(), kRowsPerWorld);
    ASSERT_EQ(gpu.rows.size(), kRowsPerWorld);
    for (uint32_t i = 0; i < kRowsPerWorld; ++i) {
        for (int k = 0; k < 3; ++k) EXPECT_EQ(gpu.rows[i].pos[k], cpu.rows[i].pos[k]) << "row " << i << " pos " << k;
        EXPECT_EQ(gpu.rows[i].wind[0], cpu.rows[i].wind[0]) << "row " << i;
        EXPECT_EQ(gpu.rows[i].density, cpu.rows[i].density) << "row " << i;
    }
    // The row says wind.x = 2 * pos.y = 4 m/s and density = pos.x = 1: nothing like the world's calm 1.225 kg/m^3 air, so a
    // reader that fell back to the world sample would give another velocity.
    EXPECT_EQ(cpu.rows[0].wind[0], 4.0f);
    EXPECT_NEAR(gpu.force.x, cpu.force.x, 1e-5f * std::max(1.0f, std::abs(cpu.force.x)));
    EXPECT_NEAR(gpu.force.y, cpu.force.y, 1e-5f * std::max(1.0f, std::abs(cpu.force.y)));
}

// REVIEW FOCUS 2, the device: the points variant fed the world's own sample in every row steps a drag body to the standard
// kernel's bits (the same expressions on the same operands). Twenty steps, so a drag force that differed in one ulp would show.
TEST(GpuPointSamples, ThePointsVariantFedTheWorldSampleReproducesTheStandardBitsOnTheDevice) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::test::ProofModule m(&spade::test::kProbeRecipe);
    const auto standard = spade::test::proof_run(spade::modules::standard_modules(), BackendKind::vulkan, false, 20);
    const auto points = spade::test::proof_run(m.set, BackendKind::vulkan, true, 20);
    ASSERT_EQ(standard.size(), points.size());
    EXPECT_EQ(std::memcmp(standard.data(), points.data(), standard.size()), 0) << "the body row differs bytewise";
}
