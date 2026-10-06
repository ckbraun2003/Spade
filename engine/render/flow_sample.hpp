#pragma once

// ---------------------------------------------------------------------------
// flow_sample -- reading a flow volume at a world point (airflow
// visualization plan section 1). The one sampling rule every flow view uses,
// on the CPU reference and, ported, on GL:
//
// CELL-CENTRED TRILINEAR, CLAMPED AT THE EDGES. Cell (i, j, k) holds its
// value at origin + (i + 1/2, j + 1/2, k + 1/2) cell. A point's continuous
// index g = (p - origin) / cell - 1/2 is clamped to [0, n - 1] per axis, so
// past the outermost centres the field is held constant, never extrapolated.
// Halves decode exactly (core/fp16.hpp).
//
// NO-DATA PROPAGATES (L6): a sample is no-data when any corner that carries
// weight is no-data. A corner with zero weight is not read, so a point at a
// valid cell's centre is that cell's value even beside a no-data cell.
//
// Speed is |(u, v, w)| of the interpolated velocity, not an interpolated
// speed.
// ---------------------------------------------------------------------------

#include <cstdint>

#include <glm/vec3.hpp>

#include "core/flow_volume.hpp"
#include "render/transfer_function.hpp"

namespace spade::render {

// A validated volume with its per-call constants hoisted.
struct FlowSampler {
    const FlowVolume* volume = nullptr;
    glm::vec3 origin{0.0f};
    float inv_cell = 0.0f;
};

// Precondition: validate_flow_volume(volume) passed. The sampler keeps a
// pointer: the volume outlives it.
[[nodiscard]] FlowSampler make_flow_sampler(const FlowVolume& volume);

struct FlowValue {
    float value = 0.0f;
    bool no_data = false;  // value is 0 then, and never drawn
};

[[nodiscard]] FlowValue sample_channel(const FlowSampler& sampler, uint32_t channel, glm::vec3 p);

[[nodiscard]] FlowValue sample_quantity(const FlowSampler& sampler, FlowQuantity quantity, glm::vec3 p);

struct FlowVelocity {
    glm::vec3 u{0.0f};
    bool no_data = false;
};

[[nodiscard]] FlowVelocity sample_velocity(const FlowSampler& sampler, glm::vec3 p);

}  // namespace spade::render
