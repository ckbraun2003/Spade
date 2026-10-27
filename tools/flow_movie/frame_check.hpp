// A published frame checked on the host before a film keeps it (B7's G2:
// GpuFluid has no scan for a NaN or an infinity yet, so the film checks what
// it publishes, and a bad frame never becomes a film; the lead, 2026-10-27).
// The publish writes a sample that is NaN, infinite or past binary16's range
// as fp16::kNoDataBits (core/flow_volume.hpp), and a frame with no parts has
// no other no-data sample, so every half with binary16's all-ones exponent is
// one of those. A flow can blow up and stay finite (solid-free D1 under plan
// A, 2026-10-27: 8.8e23 m/s at step 32), so the frame's fastest cell is
// checked too, against a bound the film sets.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "core/flow_volume.hpp"
#include "core/fp16.hpp"

namespace flow_movie {

struct NonFinite {
    uint32_t channel = 0;          // spade::FlowChannel: 0-3 the V texture's, 4-7 the S texture's
    uint32_t i = 0, j = 0, k = 0;  // its cell
    uint16_t bits = 0;             // the half as published
    std::size_t count = 0;         // the frame's non-finite halves in all
};

// The frame's first non-finite half, V's then S's, each in cell order (i
// fastest), with the count of them all; nullopt for a frame without one.
[[nodiscard]] inline std::optional<NonFinite> first_non_finite(const spade::FlowVolume& volume) {
    const std::size_t nx = volume.header.nx, ny = volume.header.ny;
    std::optional<NonFinite> first;
    std::size_t count = 0;
    const auto scan = [&](const std::vector<uint16_t>& halves, uint32_t first_channel) {
        for (std::size_t at = 0; at < halves.size(); ++at) {
            if ((halves[at] & 0x7C00u) != 0x7C00u) continue;
            ++count;
            if (first) continue;
            const std::size_t cell = at / spade::kFlowHalvesPerCell;
            first = NonFinite{.channel = first_channel + static_cast<uint32_t>(at % spade::kFlowHalvesPerCell),
                              .i = static_cast<uint32_t>(nx > 0 ? cell % nx : 0),
                              .j = static_cast<uint32_t>(nx * ny > 0 ? cell / nx % ny : 0),
                              .k = static_cast<uint32_t>(nx * ny > 0 ? cell / (nx * ny) : 0),
                              .bits = halves[at]};
        }
    };
    scan(volume.v, 0u);
    scan(volume.s, spade::kFlowHalvesPerCell);
    if (first) first->count = count;
    return first;
}

struct FastestCell {
    float speed = 0.0f;            // |(u, v, w)| as published, m/s
    uint32_t i = 0, j = 0, k = 0;  // its cell
};

// The frame's fastest cell by its published u, v and w (V's first three
// channels), the first of equals; a cell with a non-finite one is
// first_non_finite()'s. Speed 0 for a frame without a finite cell.
[[nodiscard]] inline FastestCell fastest_cell(const spade::FlowVolume& volume) {
    const std::size_t nx = volume.header.nx, ny = volume.header.ny;
    const std::size_t cells = volume.v.size() / spade::kFlowHalvesPerCell;
    float best = -1.0f;
    std::size_t at = 0;
    for (std::size_t c = 0; c < cells; ++c) {
        const uint16_t* h = volume.v.data() + c * spade::kFlowHalvesPerCell;
        const float u = spade::fp16::to_f32(h[spade::kFlowU]);
        const float v = spade::fp16::to_f32(h[spade::kFlowV]);
        const float w = spade::fp16::to_f32(h[spade::kFlowW]);
        const float s2 = u * u + v * v + w * w;
        if (std::isfinite(s2) && s2 > best) {
            best = s2;
            at = c;
        }
    }
    if (best < 0.0f) return {};
    return FastestCell{.speed = std::sqrt(best),
                       .i = static_cast<uint32_t>(nx > 0 ? at % nx : 0),
                       .j = static_cast<uint32_t>(nx * ny > 0 ? at / nx % ny : 0),
                       .k = static_cast<uint32_t>(nx * ny > 0 ? at / (nx * ny) : 0)};
}

}  // namespace flow_movie
