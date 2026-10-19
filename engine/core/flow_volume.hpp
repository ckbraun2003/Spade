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
//       q-hat = sign(Q) sqrt(|Q|) in 1/s for the Q-criterion, and
//       lambda2-hat = sign(lambda2) sqrt(|lambda2|) in 1/s for the lambda2
//       criterion (core/lambda2.hpp; below zero in a vortex core).
// A sample that is NaN, infinite or past binary16's range is no-data
// (fp16::kNoDataBits), counted in the header, and drawn as clear (L6).
//
// The CPU twin is the reference; plan C's GPU writer is banded against it.
// Vulkan's own f32 -> f16 rounding mode is implementation-defined unless the
// device reports shaderRoundingModeRTEFloat16, so plan C either checks that
// or packs in the shader with integer operations, as fp16.hpp does.
// ---------------------------------------------------------------------------

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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
    kFlowLambda2Hat = 7,
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
    // CELLS (not halves) inside a solid that were written from their fluid
    // side: the first solid layer takes the mean of its fluid face-neighbours,
    // so a surface pressure coefficient reads the wall pressure. They are
    // extrapolated, not solved; a reader may say so. At most nx ny nz.
    uint32_t extrapolated_cells = 0;
};

struct FlowVolume {
    FlowVolumeHeader header;
    std::vector<uint16_t> v;  // nx ny nz x 4 halves: u, v, w, dye
    std::vector<uint16_t> s;  // nx ny nz x 4 halves: pressure, |w|, q-hat, lambda2-hat
    // Names this content, so a reader may cache what it made of it (GL keeps
    // the halves it uploaded): a value from next_flow_generation(), unique in
    // the process, or 0 for unknown, which nothing caches. A copy keeps it,
    // and with it the content it names. ANY write to v, s or the header --
    // a publish, an edit in place, an edit of a copy -- sets it again, to
    // next_flow_generation() or to 0. A write that keeps it leaves a cache
    // drawing the old halves: silent stale pixels (L6), which only a debug
    // build's check catches (GlRenderer::CreateOptions::verify_cached_volumes).
    // Not part of the content: not published, cached, digested or walked.
    uint64_t generation = 0;
};

// A generation no volume in this process has had: never 0, increasing.
[[nodiscard]] inline uint64_t next_flow_generation() noexcept {
    static std::atomic<uint64_t> last{0};
    return last.fetch_add(1u, std::memory_order_relaxed) + 1u;
}

[[nodiscard]] inline std::size_t flow_cell_index(const FlowVolumeHeader& h, uint32_t i, uint32_t j,
                                                 uint32_t k) noexcept {
    return i + std::size_t{h.nx} * (j + std::size_t{h.ny} * k);
}

// The raw half of `channel` at cell `cell`. Precondition: a validated volume
// and cell < nx ny nz. The channel is a FlowChannel, so an integer past the
// eight channels is a type error rather than a read of the wrong lane.
[[nodiscard]] inline uint16_t flow_half(const FlowVolume& volume, std::size_t cell, FlowChannel channel) noexcept {
    const std::vector<uint16_t>& texture = channel < kFlowHalvesPerCell ? volume.v : volume.s;
    return texture[cell * kFlowHalvesPerCell + (channel % kFlowHalvesPerCell)];
}

// L6: a reader refuses a malformed volume rather than drawing part of it.
// invalid_argument for a zero dimension, dimensions whose half count
// overflows, a cell that is not finite and positive, an origin that is not
// finite, a texture of the wrong size, a no-data count past the volume's
// halves, an extrapolated count past its cells, or a channel range that is
// not finite with min <= max.
[[nodiscard]] inline Result<void> validate_flow_volume(const FlowVolume& volume) {
    const FlowVolumeHeader& h = volume.header;
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "flow volume: " + why});
    };
    if (h.nx == 0u || h.ny == 0u || h.nz == 0u) {
        return refuse("a dimension is zero");
    }
    if (!(std::isfinite(h.cell) && h.cell > 0.0f)) {
        return refuse("the cell size is not finite and positive");
    }
    if (!(std::isfinite(h.origin.x) && std::isfinite(h.origin.y) && std::isfinite(h.origin.z))) {
        return refuse("the origin is not finite");
    }
    // Each multiply checked: an unchecked product wraps (a 2^22 cube is 2^68
    // halves, 0 modulo 2^64) and would pass empty textures to unchecked reads.
    constexpr uint64_t kMax = std::numeric_limits<std::size_t>::max();
    uint64_t halves = kFlowHalvesPerCell;
    for (const uint32_t n : {h.nx, h.ny, h.nz}) {
        if (halves > kMax / n) {
            return refuse("its " + std::to_string(h.nx) + " x " + std::to_string(h.ny) + " x " +
                          std::to_string(h.nz) + " cells overflow a texture's size");
        }
        halves *= n;
    }
    if (volume.v.size() != halves || volume.s.size() != halves) {
        return refuse("the textures hold " + std::to_string(volume.v.size()) + " and " +
                      std::to_string(volume.s.size()) + " halves, not " + std::to_string(halves));
    }
    if (h.no_data > 2u * halves) {
        return refuse("its no-data count " + std::to_string(h.no_data) + " is past its " +
                      std::to_string(2u * halves) + " halves");
    }
    if (const uint64_t cells = halves / kFlowHalvesPerCell; h.extrapolated_cells > cells) {
        return refuse("its extrapolated count " + std::to_string(h.extrapolated_cells) + " is past its " +
                      std::to_string(cells) + " cells");
    }
    for (uint32_t c = 0; c < kFlowChannelCount; ++c) {
        const float lo = h.channel_min[c];
        const float hi = h.channel_max[c];
        if (!(std::isfinite(lo) && std::isfinite(hi) && lo <= hi)) {
            return refuse("channel " + std::to_string(c) + "'s range is not finite with min <= max");
        }
    }
    return {};
}

}  // namespace spade
