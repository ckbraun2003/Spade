// A published frame checked on the host before a film keeps it: every
// film's frames pass through write_frame() (cache.hpp), which caches none
// that fails (the lead, 2026-10-27; first for G2's GpuFluid, which has no
// scan for a NaN or an infinity yet, then for every film, after Test/Docs
// found the CPU's D1 with its parts growing a disc-plane face ~4 m/s a step).
// The publish writes a sample that is NaN, infinite or past binary16's range
// as fp16::kNoDataBits (core/flow_volume.hpp), so every half with binary16's
// all-ones exponent is one of those, or a solid cell's past the parts' first
// layer: those are no-data in all eight channels (publish.hpp), and a frame
// may hold that many. A flow can blow up and stay finite (solid-free D1
// under plan A, 2026-10-27: 8.8e23 m/s at step 32), so the frame's fastest
// cell is checked too, against ten times the case's expected top speed.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "core/flow_volume.hpp"
#include "core/fp16.hpp"

namespace flow_movie {

inline constexpr const char* kChannelNames[] = {"u", "v", "w", "dye", "pressure", "|w|", "q-hat", "lambda2-hat"};

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

// What a film's frames are held to.
struct FrameBound {
    float expected_speed = 0.0f;  // the case's expected top speed, in its own units (m/s, or the lid's)
    uint32_t solid_cells = 0;     // the run's cells inside its parts (LiveRun::solids()); 0 without
};

// The fastest cell's bound: ten times the expected speed, and at least ten.
[[nodiscard]] inline float speed_bound(const FrameBound& bound) {
    return 10.0f * std::max(bound.expected_speed, 1.0f);
}

// Why `volume` can't be kept, or "" when it can: a bound without an expected
// speed; non-finite halves past those its parts' interior publishes, eight a
// solid cell past the first layer (the solid cells less the frame's
// extrapolated ones), the first named when the run has no parts; or a
// fastest cell past speed_bound().
[[nodiscard]] inline std::string frame_fault(const spade::FlowVolume& volume, const FrameBound& bound) {
    char why[320];
    if (!(bound.expected_speed > 0.0f)) return "no expected top speed to bound its cells by";
    const std::size_t extrapolated = volume.header.extrapolated_cells;
    const std::size_t interior = bound.solid_cells > extrapolated ? bound.solid_cells - extrapolated : 0u;
    const std::size_t allowed = interior * spade::kFlowChannelCount;
    if (const std::optional<NonFinite> bad = first_non_finite(volume); bad && bad->count > allowed) {
        if (bound.solid_cells == 0u) {
            std::snprintf(why, sizeof why,
                          "%zu non-finite sample%s, the first %s at cell (%u, %u, %u), half 0x%04x: the fluid went "
                          "NaN, infinite or past binary16's range",
                          bad->count, bad->count == 1u ? "" : "s", kChannelNames[bad->channel], bad->i, bad->j, bad->k,
                          static_cast<unsigned>(bad->bits));
        } else {
            std::snprintf(why, sizeof why,
                          "%zu non-finite samples, %zu past the %zu its parts' interior publishes as no data: the "
                          "fluid went NaN, infinite or past binary16's range",
                          bad->count, bad->count - allowed, allowed);
        }
        return why;
    }
    const FastestCell fast = fastest_cell(volume);
    if (const float top = speed_bound(bound); fast.speed > top) {
        std::snprintf(why, sizeof why,
                      "max |v| %.1f at cell (%u, %u, %u), past the bound %.1f (10 x the case's expected top speed, "
                      "%.2f): the flow blew up",
                      static_cast<double>(fast.speed), fast.i, fast.j, fast.k, static_cast<double>(top),
                      static_cast<double>(bound.expected_speed));
        return why;
    }
    return {};
}

}  // namespace flow_movie
