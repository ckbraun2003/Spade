#pragma once

// world/detail/weld.hpp -- the exact weld and the compaction every reader
// shares, and the declared-unit check the readers of unitless formats share
// (TD-9). Internal to spade::mesh_import, whose sources are in
// world/mesh_import/; world/detail/ is never installed.
//
// Two vertices merge when their positions are equal by value, component by
// component (so -0 equals +0, and a NaN merges with nothing). No tolerance.
// Survivors are numbered in first-use order: walking `indices` in order, a
// vertex gets the next number the first time a triangle names it (or names
// a duplicate of it). A vertex no index names is dropped. The sort is under
// a total order, ties broken by index, so the result depends on the input
// alone.
//
// Precondition: every index is < positions.size() (the readers check it,
// naming the element, before welding).

#include <cstdint>
#include <span>
#include <string>

#include <glm/vec3.hpp>

#include "world/mesh.hpp"

namespace spade::detail {

[[nodiscard]] MeshInput weld_exact(std::span<const glm::dvec3> positions, std::span<const uint32_t> indices,
                                   MeshUnit unit);

// Why `unit` can't be the unit a caller declares for a file that carries none
// (STL, OBJ): it is unspecified, or not a MeshUnit. Empty when it can.
[[nodiscard]] std::string unit_problem(MeshUnit unit);

}  // namespace spade::detail
