#pragma once

// ---------------------------------------------------------------------------
// flow_volume -- the airflow render volume: what Physics' Publish writes and
// Rendering's views read (physics/plans/2026-10-05-airflow-design.md section
// 5; rendering/plans/2026-10-05-airflow-visualization-plan.md section 1).
// In core/ because physics/airflow/ and render/ may both include core/ and
// neither may include the other.
//
// A PUBLISH OUTPUT. It is not in the walk, the digest or the snapshot:
// nothing steps from it, and a view reading it cannot change stepping (L5).
// Its storage is sized when the airflow module is created, never allocated
// in a step.
//
// Cell-centred, i fastest, then j, then k. Two textures of four binary16
// channels each, stored as raw halves (core/fp16.hpp's rule):
//   V = (u, v, w) in m/s, dye (dimensionless);
//   S = gauge pressure in Pa, vorticity magnitude |w| in 1/s,
//       q-hat = sign(Q) sqrt(|Q|) in 1/s for the Q-criterion, and a channel
//       reserved as zero.
// A sample that is NaN, infinite or past binary16's range is no-data
// (fp16::kNoDataBits), counted in the header, and drawn as clear (L6).
//
// The CPU twin is the reference; plan C's GPU writer is banded against it.
// Vulkan's own f32 -> f16 rounding mode is implementation-defined unless the
// device reports shaderRoundingModeRTEFloat16, so plan C either checks that
// or packs in the shader with integer operations, as fp16.hpp does.
// ---------------------------------------------------------------------------

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/fp16.hpp"

namespace spade {

// Channel ids: 0-3 are V's, 4-7 are S's, each in its texture's order.
enum FlowChannel : uint32_t {
    kFlowU = 0,
    kFlowV = 1,
    kFlowW = 2,
    kFlowDye = 3,
    kFlowPressure = 4,
    kFlowVorticity = 5,
    kFlowQHat = 6,
    kFlowReserved = 7,
};
inline constexpr uint32_t kFlowChannelCount = 8;
inline constexpr uint32_t kFlowHalvesPerCell = 4;  // per texture

struct FlowVolumeHeader {
    glm::vec3 origin{0.0f};  // world position of the box's minimum corner, metres
    float cell = 0.0f;       // the cubic cell's edge, metres
    uint32_t nx = 0, ny = 0, nz = 0;
    // The finite minimum and maximum of each channel over its valid samples,
    // for transfer functions that ask for the frame's own range. Both are 0
    // for a channel with no valid sample.
    std::array<float, kFlowChannelCount> channel_min{};
    std::array<float, kFlowChannelCount> channel_max{};
    uint32_t no_data = 0;  // samples written as fp16::kNoDataBits, both textures together
};

struct FlowVolume {
    FlowVolumeHeader header;
    std::vector<uint16_t> v;  // nx ny nz x 4 halves: u, v, w, dye
    std::vector<uint16_t> s;  // nx ny nz x 4 halves: pressure, |w|, q-hat, reserved
};

[[nodiscard]] inline std::size_t flow_cell_index(const FlowVolumeHeader& h, uint32_t i, uint32_t j,
                                                 uint32_t k) noexcept {
    return i + std::size_t{h.nx} * (j + std::size_t{h.ny} * k);
}

// The raw half of `channel` at cell `cell`. Precondition: a validated volume.
[[nodiscard]] inline uint16_t flow_half(const FlowVolume& volume, std::size_t cell, uint32_t channel) noexcept {
    const std::vector<uint16_t>& texture = channel < kFlowHalvesPerCell ? volume.v : volume.s;
    return texture[cell * kFlowHalvesPerCell + (channel % kFlowHalvesPerCell)];
}

// L6: a reader refuses a malformed volume rather than drawing part of it.
// invalid_argument for a zero dimension, a cell that is not finite and
// positive, an origin that is not finite, a texture of the wrong size, or a
// channel range that is not finite with min <= max.
// RED: not yet implemented.
[[nodiscard]] inline Result<void> validate_flow_volume(const FlowVolume&) { return {}; }

}  // namespace spade
