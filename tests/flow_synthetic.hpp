#pragma once

// Test-side synthetic flow volumes: analytic fields written through the
// publish format (core/flow_volume.hpp, core/fp16.hpp), so a view's test
// pins rendering alone and never moves with the solver (airflow spec
// section 5). Each cell holds its field's value at the cell's centre.

#include <algorithm>
#include <array>
#include <cstdint>

#include <glm/vec3.hpp>

#include "core/flow_volume.hpp"
#include "core/fp16.hpp"

namespace spade::test {

using FlowChannels = std::array<float, kFlowChannelCount>;

// `field(p)` gives every channel at world point p. Ranges are taken over the
// decoded halves, as the volume holds them; no-data samples are counted and
// left out of the ranges.
template <class Field>
[[nodiscard]] FlowVolume make_flow_volume(glm::vec3 origin, float cell, uint32_t nx, uint32_t ny, uint32_t nz,
                                          Field&& field) {
    FlowVolume volume;
    FlowVolumeHeader& h = volume.header;
    h.origin = origin;
    h.cell = cell;
    h.nx = nx;
    h.ny = ny;
    h.nz = nz;
    const std::size_t cells = std::size_t{nx} * ny * nz;
    volume.v.resize(cells * kFlowHalvesPerCell);
    volume.s.resize(cells * kFlowHalvesPerCell);
    std::array<bool, kFlowChannelCount> seen{};
    for (uint32_t k = 0; k < nz; ++k) {
        for (uint32_t j = 0; j < ny; ++j) {
            for (uint32_t i = 0; i < nx; ++i) {
                const glm::vec3 p = origin + (glm::vec3(static_cast<float>(i), static_cast<float>(j),
                                                        static_cast<float>(k)) +
                                              glm::vec3(0.5f)) *
                                                 cell;
                const FlowChannels c = field(p);
                const std::size_t base = flow_cell_index(h, i, j, k) * kFlowHalvesPerCell;
                for (uint32_t ch = 0; ch < kFlowChannelCount; ++ch) {
                    const fp16::Half half = fp16::from_f32(c[ch]);
                    (ch < kFlowHalvesPerCell ? volume.v : volume.s)[base + ch % kFlowHalvesPerCell] = half.bits;
                    if (half.no_data) {
                        ++h.no_data;
                        continue;
                    }
                    const float x = fp16::to_f32(half.bits);
                    h.channel_min[ch] = seen[ch] ? std::min(h.channel_min[ch], x) : x;
                    h.channel_max[ch] = seen[ch] ? std::max(h.channel_max[ch], x) : x;
                    seen[ch] = true;
                }
            }
        }
    }
    return volume;
}

// Uniform flow (u, v, w), everything else zero.
[[nodiscard]] inline auto uniform_flow(glm::vec3 velocity) {
    return [velocity](glm::vec3) {
        FlowChannels c{};
        c[kFlowU] = velocity.x;
        c[kFlowV] = velocity.y;
        c[kFlowW] = velocity.z;
        return c;
    };
}

}  // namespace spade::test
