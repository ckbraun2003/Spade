#pragma once

// ---------------------------------------------------------------------------
// flow_lines -- streamlines through a flow volume (airflow visualization
// plan section 4), the CPU reference.
//
// CLASSIC RK4 IN ARC LENGTH, dx/ds = u / |u|, at a fixed step: a quarter
// cell by default (the lead's Q4). Never adaptive, so every backend takes
// the same steps; adaptive stepping can come later as an opt-in. Points
// are evenly spaced along the line, whatever the speed. Velocity is
// sampled by render/flow_sample.hpp's one rule (cell-centred trilinear).
//
// Each seed traces backward and forward. Each direction ENDS FOR A REASON
// IT RECORDS (L6): leaving the volume's box, stagnation (|u| under the
// floor at any RK4 stage), a no-data sample at any stage, or the step
// budget. A seed outside the box is a line of one point that left the box
// both ways.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"

namespace spade::render {

enum class LineEnd : uint8_t { left_box, stagnation, no_data, step_budget };

struct StreamlineOptions {
    float step_cells = 0.25f;          // the RK4 step, in cells (0, 4]
    uint32_t max_steps = 4096;         // per direction, 1 to 2^20
    float stagnation_speed = 1.0e-4f;  // m/s, >= 0
    bool forward = true;
    bool backward = true;
};

struct Streamline {
    std::vector<glm::vec3> points;  // the backward end first, through the seed, to the forward end
    std::vector<float> speeds;      // |u| at each point
    std::size_t seed_point = 0;     // the seed's index in points
    LineEnd backward_end = LineEnd::step_budget;
    LineEnd forward_end = LineEnd::step_budget;
};

// invalid_argument, by name: a volume that fails validate_flow_volume(), a
// seed that is not finite, a step outside (0, 4] cells, a step budget
// outside 1 to 2^20, a negative or non-finite stagnation floor, or neither
// direction asked for.
[[nodiscard]] Result<std::vector<Streamline>> trace_streamlines(const FlowVolume& volume,
                                                                std::span<const glm::vec3> seeds,
                                                                const StreamlineOptions& options = {});

// Seeds. A rake: n points evenly from a to b (n = 1 is the midpoint).
[[nodiscard]] std::vector<glm::vec3> seed_rake(glm::vec3 a, glm::vec3 b, uint32_t n);

// A ring: n points evenly around `axis` (unit) at `radius` from `centre`,
// in the plane through centre normal to the axis. sin32/cos32 (TD-3).
[[nodiscard]] std::vector<glm::vec3> seed_ring(glm::vec3 centre, glm::vec3 axis, float radius, uint32_t n);

}  // namespace spade::render
