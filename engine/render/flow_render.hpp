#pragma once

// ---------------------------------------------------------------------------
// flow_render -- the flow volume's emission-absorption march (airflow
// visualization plan section 2), the CPU reference. raster_cpu draws a
// RenderScene's flow layers through march_flow_ray(); GL ports it.
//
// THE MARCH. Along a ray clipped to the volume's box, to the near and far
// planes and to the opaque depth already in the frame, front to back in
// fixed steps of half a cell. Step i spans [t_start + i h, its end]; the
// last step takes its true length, so a frame does not depend on where the
// box starts. Each step samples at its midpoint and composites:
//   tau = exp32(-sigma dt)              (Beer-Lambert, core/fp32_math)
//   C  += T (1 - tau) c
//   T  *= tau
// stopping early once T < 1/256. The pixel becomes C + T C_behind.
//
// A LAYER'S TRANSFER FUNCTIONS MARCH TOGETHER (speed with smoke, say):
// their extinctions add, and their colours mix by extinction,
// c = sum(sigma_i c_i) / sum(sigma_i).
//
// NO-DATA (L6): a step whose sample is no-data for any of the layer's
// quantities contributes nothing (it is clear), and the ray reports that it
// crossed no-data, so the frame's report counts the pixel.
//
// Colours are display values, composited in that space, as the field
// layer's are. Volumes write no depth. Overlays draw after them, on top.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/flow_sample.hpp"
#include "render/target.hpp"
#include "render/transfer_function.hpp"

namespace spade::render {

inline constexpr std::size_t kMaxFlowTransfers = 4;
inline constexpr float kFlowEarlyExitTransmittance = 1.0f / 256.0f;

// One published volume and the transfer functions that draw it. The volume
// is the caller's (the airflow module's publish output) and outlives every
// render() that reads it.
struct FlowLayer {
    const FlowVolume* volume = nullptr;
    std::vector<TransferFunction> transfers;  // 1 to kMaxFlowTransfers
};

// invalid_argument, by name: no volume, a volume that fails
// validate_flow_volume(), no transfer function or more than four, or a
// transfer function that fails validate_transfer_function().
[[nodiscard]] Result<void> validate_flow_layer(const FlowLayer& layer);

// A validated layer with its per-frame constants hoisted.
struct PreparedFlowLayer {
    FlowSampler sampler;
    std::vector<PreparedTransfer> transfers;
    Aabb box;
    float step = 0.0f;  // half a cell
    float inv_step = 0.0f;
};

[[nodiscard]] PreparedFlowLayer prepare_flow_layer(const FlowLayer& layer);

struct FlowRayResult {
    glm::vec3 colour{0.0f};      // C: emitted light reaching the eye
    float transmittance = 1.0f;  // T: what lets C_behind through
    bool crossed_no_data = false;
};

// The march over [t_start, t_end] of origin + t dir; dir is unit length.
// An empty or reversed interval leaves the pixel as it was (C 0, T 1).
[[nodiscard]] FlowRayResult march_flow_ray(const PreparedFlowLayer& layer, glm::vec3 origin, glm::vec3 dir,
                                           float t_start, float t_end);

// What a frame drew with, for a caller to show beside it (a legend, a HUD).
struct FlowLayerReport {
    std::vector<ResolvedRange> ranges;     // per transfer function; from_header ones are announced
    uint32_t volume_no_data = 0;           // the volume header's count
    uint64_t pixels_crossing_no_data = 0;  // pixels whose ray met no-data and drew it clear
};

struct FlowFrameReport {
    std::vector<FlowLayerReport> layers;  // one per RenderScene::flow_layers entry, in order
};

}  // namespace spade::render
