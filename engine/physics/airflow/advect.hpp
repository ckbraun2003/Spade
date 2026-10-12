#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// ADVECTION (spec §1.5, step 4): semi-Lagrangian, an RK2 backtrace, and the
// MacCormack correction clamped to the forward stencil's range.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct Sample {
    float value = 0.0f;
    float lo = 0.0f;  // the smallest of the eight values interpolated
    float hi = 0.0f;  // the largest
};

// Trilinear interpolation of one velocity component's array at a point
// (metres, in the grid's frame), in the lerp form a + t (b - a), x then y
// then z. A periodic axis wraps; any other clamps to the samples' extent.
[[nodiscard]] Sample sample_component(const GridShape& s, std::span<const float> field, uint32_t component,
                                      glm::vec3 x, const DomainBc& bc) noexcept;
[[nodiscard]] glm::vec3 velocity_at(const MacGrid& g, glm::vec3 x, const DomainBc& bc) noexcept;

struct AdvectScratch {
    std::array<std::vector<float>, 3> hat;  // the forward estimate, per component
    std::array<std::vector<float>, 3> lo;   // its stencil's range, for the clamp
    std::array<std::vector<float>, 3> hi;
};

[[nodiscard]] AdvectScratch make_advect_scratch(const GridShape& shape);

// Writes every face of `out` (shaped like `in`); reads only `in`.
void advect_velocity(const MacGrid& in, MacGrid& out, const DomainBc& bc, float h, AdvectScratch& s) noexcept;

}  // namespace spade::physics::airflow
