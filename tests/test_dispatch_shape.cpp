// test_dispatch_shape.cpp -- 3D dispatch arithmetic and the kernels' pipeline layouts (module API stage 7, Task B1;
// docs/design/core/plans/2026-11-06-module-api-stage7-plan.md; design 7.5 and the plan's decision 1).
//
// Host-only. The device half -- a kernel seeing every global id once, the three sizes bit-identical, the recorder's
// recording functions and pipelines on demand -- is test_gpu_dispatch_3d.cpp.
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/admit.hpp"
#include "compute/device_limits.hpp"
#include "compute/dispatch_shape.hpp"
#include "compute/layout_classes.hpp"

namespace {

using spade::compute::AdmitShape;
using spade::compute::DeviceLimits;
using spade::compute::GridExtent;
using spade::compute::grid_groups;
using spade::compute::kernel_layout;
using spade::compute::local_shape;
using spade::compute::LocalShape;
using spade::compute::world_of_group_z;
using spade::compute::WorldZ;
using Groups = std::array<uint32_t, 3>;

}  // namespace

TEST(Dispatch3D, TheWorkgroupSizeKnobMapsTo3DShapes) {
    EXPECT_EQ(local_shape(32), (LocalShape{4, 4, 2}));
    EXPECT_EQ(local_shape(64), (LocalShape{4, 4, 4}));
    EXPECT_EQ(local_shape(128), (LocalShape{8, 4, 4}));
    for (const uint32_t size : {32u, 64u, 128u}) {
        const LocalShape s = local_shape(size);
        EXPECT_EQ(s.x * s.y * s.z, size) << "the shape holds exactly workgroup_size invocations";
    }
    EXPECT_EQ(local_shape(48), (LocalShape{0, 0, 0})) << "a size the build does not compile has no shape";
    EXPECT_EQ(local_shape(0), (LocalShape{0, 0, 0}));
}

TEST(Dispatch3D, TheGroupsCoverTheExtentWhateverItsRemainder) {
    EXPECT_EQ(grid_groups({16, 16, 16}, 64, 1), (Groups{4, 4, 4}));
    EXPECT_EQ(grid_groups({16, 16, 16}, 32, 1), (Groups{4, 4, 8}));
    EXPECT_EQ(grid_groups({16, 16, 16}, 128, 1), (Groups{2, 4, 4}));
    // 33 x 17 x 9 is a multiple of no local shape: the groups round up, and the kernel guards its edge.
    EXPECT_EQ(grid_groups({33, 17, 9}, 32, 1), (Groups{9, 5, 5}));
    EXPECT_EQ(grid_groups({33, 17, 9}, 64, 1), (Groups{9, 5, 3}));
    EXPECT_EQ(grid_groups({33, 17, 9}, 128, 1), (Groups{5, 5, 3}));
}

// The reason 3D exists: a 128^3 cell grid at workgroup 32 was 65,536 groups on x. It is now 32 x 32 x 64, and its face
// grid 33 x 32 x 64, on every axis inside the guaranteed 65,535.
TEST(Dispatch3D, A128CubedGridFitsTheGuaranteedLimitAtEveryWorkgroupSize) {
    for (const uint32_t size : {32u, 64u, 128u}) {
        for (const GridExtent e : {GridExtent{128, 128, 128}, GridExtent{129, 128, 128}}) {
            const Groups g = grid_groups(e, size, 1);
            for (int axis = 0; axis < 3; ++axis) EXPECT_LE(g[axis], 65535u) << "size " << size << " axis " << axis;
            EXPECT_GT(g[0], 0u);
        }
    }
    EXPECT_EQ(grid_groups({128, 128, 128}, 32, 1), (Groups{32, 32, 64}));
    EXPECT_EQ(grid_groups({129, 128, 128}, 32, 1), (Groups{33, 32, 64}));
}

TEST(Dispatch3D, WorldsFoldIntoTheZGroupsEachWorldContiguous) {
    // Two worlds of 33 x 17 x 9 at workgroup 32: each world is 5 z groups, so 10.
    EXPECT_EQ(grid_groups({33, 17, 9}, 32, 2), (Groups{9, 5, 10}));
    EXPECT_EQ(grid_groups({33, 17, 9}, 32, 3), (Groups{9, 5, 15}));
    // World w owns z groups [5w, 5w + 5): the first group of a world starts at its multiple.
    EXPECT_EQ(world_of_group_z(0, 5), (WorldZ{0, 0}));
    EXPECT_EQ(world_of_group_z(4, 5), (WorldZ{0, 4}));
    EXPECT_EQ(world_of_group_z(5, 5), (WorldZ{1, 0}));
    EXPECT_EQ(world_of_group_z(14, 5), (WorldZ{2, 4}));
    // Every group maps to exactly one (world, z) and the map is onto: 3 worlds x 5 groups.
    std::array<std::array<int, 5>, 3> seen{};
    for (uint32_t gz = 0; gz < 15; ++gz) {
        const WorldZ w = world_of_group_z(gz, 5);
        ASSERT_LT(w.world, 3u);
        ASSERT_LT(w.z, 5u);
        ++seen[w.world][w.z];
    }
    for (const auto& row : seen) {
        for (const int count : row) EXPECT_EQ(count, 1);
    }
    EXPECT_EQ(world_of_group_z(7, 0), (WorldZ{0, 7})) << "no z groups per world: no division by zero";
}

TEST(Dispatch3D, AnEmptyShapeIsAnEmptyDispatchAndAHugeOneSaturates) {
    EXPECT_EQ(grid_groups({0, 8, 8}, 64, 1), (Groups{0, 0, 0}));
    EXPECT_EQ(grid_groups({8, 8, 8}, 64, 0), (Groups{0, 0, 0})) << "no worlds";
    EXPECT_EQ(grid_groups({8, 8, 8}, 48, 1), (Groups{0, 0, 0})) << "an unsupported size";
    // 2^32-1 z cells at lz 2 is 2^31 groups per world; four worlds is past a uint32: it saturates, so admit() refuses it by
    // name (check_groups) instead of a wrapped count passing.
    EXPECT_EQ(grid_groups({1, 1, 0xFFFFFFFFu}, 32, 4)[2], 0xFFFFFFFFu);
}

// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] DeviceLimits limits_with_per_stage(uint32_t per_stage) {
    DeviceLimits l;
    l.max_storage_buffer_range = uint64_t{1} << 32;
    l.max_memory_allocation_size = uint64_t{1} << 32;
    l.max_per_stage_storage_buffers = per_stage;
    l.max_set_storage_buffers = per_stage;
    l.max_push_constants_size = 256;
    l.max_group_count = {65535, 65535, 65535};
    l.max_group_size = {1024, 1024, 64};
    l.max_group_invocations = 1024;
    l.allocation_size_reported = true;
    return l;
}

}  // namespace

TEST(LayoutClasses, AModuleKernelCountsSetZeroPlusItsSetOneSubset) {
    const std::array<uint32_t, 3> subset = {0, 3, 5};
    const auto layout = kernel_layout("airflow_step", subset);
    ASSERT_TRUE(layout.has_value()) << layout.error().context;
    EXPECT_EQ(layout->name, "airflow_step");
    EXPECT_EQ(layout->bindings_per_set, (std::vector<uint32_t>{spade::compute::gen::kBindingCount_state, 3}));

    AdmitShape shape;
    shape.layouts = {*layout};
    // 30 + 3 = 33 storage buffers in the layout: one under the limit is refused, the limit itself is admitted.
    const auto refused = spade::compute::admit(shape, limits_with_per_stage(32), "Test GPU");
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("33"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("airflow_step"), std::string::npos) << refused.error().context;
    EXPECT_TRUE(spade::compute::admit(shape, limits_with_per_stage(33), "Test GPU").has_value());
}

TEST(LayoutClasses, AStandardKernelWithoutASetOneUserIsStillThirty) {
    const auto layout = kernel_layout("integrate", {});
    ASSERT_TRUE(layout.has_value()) << layout.error().context;
    EXPECT_EQ(layout->bindings_per_set, (std::vector<uint32_t>{spade::compute::gen::kBindingCount_state}))
        << "one set, the thirty bindings of today";
}

// BATCH D's FIRST RED (lead, 2026-11-09): an undeclared set-1 number INSIDE the 0..47 space is refused by name when the layout is
// made, not later. Declared today: 0..27 (airflow), 28 (points), 33 (config), 34 (centred), 40 and 41 (lbm). 29 to 32 are
// named in the plan but not yet declared; 35 to 39 are spare; 42 to 47 are reserved.
TEST(LayoutClasses, AnUndeclaredSetOneNumberInsideTheSlotsIsRefusedByName) {
    for (const uint32_t undeclared : {29u, 30u, 31u, 32u, 35u, 39u, 42u, 47u}) {
        const std::array<uint32_t, 1> named = {undeclared};
        const auto refused = kernel_layout("airflow_step", named);
        ASSERT_FALSE(refused.has_value()) << "binding " << undeclared << " is declared nowhere in bindings.slang";
        EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
        EXPECT_NE(refused.error().context.find("airflow_step"), std::string::npos) << refused.error().context;
        EXPECT_NE(refused.error().context.find(std::to_string(undeclared)), std::string::npos) << refused.error().context;
        EXPECT_NE(refused.error().context.find("undeclared"), std::string::npos) << refused.error().context;
    }
    for (const uint32_t declared : {0u, 27u, 28u, 33u, 34u, 40u, 41u}) {
        const std::array<uint32_t, 1> named = {declared};
        EXPECT_TRUE(kernel_layout("airflow_step", named).has_value()) << "binding " << declared << " is declared";
    }
}

TEST(LayoutClasses, ABindingOutOfRangeOrNamedTwiceIsRefusedNamingTheKernel) {
    const std::array<uint32_t, 2> twice = {4, 4};
    const auto dup = kernel_layout("airflow_step", twice);
    ASSERT_FALSE(dup.has_value());
    EXPECT_EQ(dup.error().code, spade::Code::invalid_argument);
    EXPECT_NE(dup.error().context.find("airflow_step"), std::string::npos) << dup.error().context;
    EXPECT_NE(dup.error().context.find('4'), std::string::npos) << dup.error().context;

    // Set 1's binding space is 0..47 (the plan: 28 in use plus six new, six spare, eight for the LBM twin); the declared
    // count (kBindingCount_airflow) is not the bound, the numbers are sparse.
    const std::array<uint32_t, 1> past = {spade::compute::kSet1BindingSlots};
    const auto out = kernel_layout("airflow_step", past);
    ASSERT_FALSE(out.has_value());
    EXPECT_NE(out.error().context.find("airflow_step"), std::string::npos) << out.error().context;
    EXPECT_NE(out.error().context.find("48"), std::string::npos) << out.error().context;
}
