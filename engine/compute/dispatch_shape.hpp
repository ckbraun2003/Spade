// dispatch_shape.hpp -- how a grid kernel's work is cut into workgroups (module API stage 7, Task B1;
// docs/design/core/plans/2026-10-12-module-api-stage7-design.md 7.5).
//
// Every dispatch was 1D, and a 128^3 cell grid at workgroup 32 needs 65,536 groups on x, one past the guaranteed
// maxComputeWorkGroupCount of 65,535. A grid kernel is dispatched in 3D: the `workgroup_size` knob (32, 64 or 128) maps to
// a local SHAPE, the grid's extent is cut by it, and the worlds fold into the z groups, each world contiguous. Vulkan-free
// and constexpr, so the recorder, the tests and admit() read the same arithmetic.
//
// RED STUBS (stage 7 Task B1): every function answers zeros. The mapping arrives with the GREEN commit.
#pragma once

#include <array>
#include <cstdint>

namespace spade::compute {

struct LocalShape {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;
    bool operator==(const LocalShape&) const = default;
};

// The local shape for a workgroup_size of 32 (4x4x2), 64 (4x4x4) or 128 (8x4x4); {0, 0, 0} for any other size. x*y*z is
// the size.
[[nodiscard]] constexpr LocalShape local_shape(uint32_t /*workgroup_size*/) noexcept { return {}; }

struct GridExtent {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;
};

// The group counts for `worlds` grids of `extent` cells at `workgroup_size`: {ceil(x/lx), ceil(y/ly), worlds * ceil(z/lz)}.
// An empty extent, no worlds or an unsupported size is {0, 0, 0}, a legal dispatch that runs nothing. A count past 2^32-1
// saturates there, so admit() refuses it by name instead of the arithmetic wrapping.
[[nodiscard]] constexpr std::array<uint32_t, 3> grid_groups(GridExtent /*extent*/, uint32_t /*workgroup_size*/,
                                                           uint32_t /*worlds*/) noexcept {
    return {0, 0, 0};
}

struct WorldZ {
    uint32_t world = 0;
    uint32_t z = 0;  // the group's z index within its world
    bool operator==(const WorldZ&) const = default;
};

// Which world a z group belongs to and where it sits inside it, given each world's z groups (ceil(z/lz)). World w owns
// groups [w*z_per_world, (w+1)*z_per_world). z_per_world 0 answers {0, group_z}.
[[nodiscard]] constexpr WorldZ world_of_group_z(uint32_t /*group_z*/, uint32_t /*z_groups_per_world*/) noexcept {
    return {};
}

}  // namespace spade::compute
