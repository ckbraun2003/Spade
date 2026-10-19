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
#include "render/transfer_function.hpp"

namespace spade::render {

enum class LineEnd : uint8_t { left_box, stagnation, no_data, step_budget };

struct StreamlineOptions {
    float step_cells = 0.25f;          // the RK4 step, in cells (0, 4]
    uint32_t max_steps = 4096;         // per direction, 1 to 2^20
    float stagnation_speed = 1.0e-4f;  // m/s, >= 0
    bool forward = true;
    bool backward = true;
    // A cap on each direction's length, in the volume's box diagonals (finite,
    // >= 0; 0 is no cap). Whichever of it and max_steps is fewer steps ends
    // the line, as step_budget. The default has none: the films and goldens.
    float max_length_diagonals = 0.0f;

    // For a live view (the editor, the tour): one box diagonal each way, so a
    // line on a closed orbit costs one crossing of the box, not its whole step
    // budget (the lead, 2026-10-19); the rest as the default.
    [[nodiscard]] static StreamlineOptions live() noexcept;

    [[nodiscard]] bool operator==(const StreamlineOptions&) const = default;
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

// ---------------------------------------------------------------------------
// TRACERS (streaklines, the smoke the eye knows): particles the published
// field carries, dx/dt = u, by classic RK4 in time at the caller's dt.
// Each source emits one tracer at every multiple of its interval of
// simulated time, k * interval for k = 0, 1, ...: a fixed schedule, no RNG.
// A tracer emitted inside a step is carried for the rest of that step, so
// how a run is cut into steps does not change when tracers appear.
//
// A tracer RETIRES, counted by reason (L6), when a step ends outside the
// box, when any stage meets no-data, or when it passes max_age_s. The
// state is the caller's, passed in and out, so the technique stays a pure
// function of its inputs. Capacity is fixed: an emission past it is
// dropped and counted, never a silent grow.
// ---------------------------------------------------------------------------

struct TracerSource {
    glm::vec3 position{0.0f};
    float interval_s = 0.1f;  // one tracer per interval, > 0
};

struct TracerOptions {
    uint32_t capacity = 4096;  // live tracers, 1 to 2^24
    float max_age_s = 10.0f;   // > 0
};

struct TracerState {
    std::vector<glm::vec3> positions;
    std::vector<float> ages;        // seconds since emission
    std::vector<uint32_t> sources;  // the emitting source's index
    double clock_s = 0.0;           // simulated time advanced so far
    uint64_t dropped = 0;           // emissions refused at capacity
    uint64_t retired_left_box = 0;
    uint64_t retired_no_data = 0;
    uint64_t retired_aged = 0;
};

// Advances every live tracer by dt, retires those that must, then emits
// what the schedule holds in [clock, clock + dt), and advances the clock.
// invalid_argument, by name, before any change: a volume that fails
// validate_flow_volume(), dt not finite and > 0, a capacity outside 1 to
// 2^24, max_age_s not finite and > 0, a source not finite or whose interval
// is not finite and > 0, a source that would emit more than 2^20 tracers in
// one step, or a state whose arrays disagree in length or exceed capacity.
[[nodiscard]] Result<void> advance_tracers(TracerState& state, const FlowVolume& volume,
                                           std::span<const TracerSource> sources, float dt,
                                           const TracerOptions& options = {});

// ---------------------------------------------------------------------------
// DRAWING. A streamline becomes a polyline with a colour per point; tracers
// become points. The raster modes draw both after the flow layers and
// before the overlays, depth-tested and unlit. A segment takes the mean of
// its two points' colours; a point is a square of size_px pixels. GL
// refuses them until its pass lands (plan step 7).
// ---------------------------------------------------------------------------

struct FlowPolyline {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colours;  // one per point, in [0, 1]
};

struct FlowPoints {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> colours;  // one per position, in [0, 1]
    uint32_t size_px = 3;            // 1 to 16
};

// invalid_argument, by name: not one colour per point, a point that is not
// finite, a colour outside [0, 1], or (points) a size outside 1 to 16.
[[nodiscard]] Result<void> validate_flow_polyline(const FlowPolyline& line);
[[nodiscard]] Result<void> validate_flow_points(const FlowPoints& points);

// A streamline coloured by its speeds through `by_speed`'s colours.
[[nodiscard]] FlowPolyline colour_streamline(const Streamline& line, const PreparedTransfer& by_speed);

// Live tracers as points of one colour (smoke).
[[nodiscard]] FlowPoints tracer_points(const TracerState& state, glm::vec3 colour, uint32_t size_px = 3);

}  // namespace spade::render
