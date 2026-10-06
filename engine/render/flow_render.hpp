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
// ISO-SURFACES (plan section 3) are ray-cast in the same march, not meshed.
// A sign change of (value - iso) between two steps' samples is refined by
// four bisections on the trilinear field; the hit's normal is the field's
// gradient (central differences one cell apart), turned toward the eye and
// lit by the scene's sun through shade_vertex_color(). Its colour is fixed,
// or another quantity at the hit through a transfer function's colours (the
// standard "Q coloured by speed"). An opaque hit ends the ray and writes
// depth; a translucent one composites and the ray goes on. The volume adds
// light up to the step whose sample passes the hit.
//
// Colours are display values, composited in that space, as the field
// layer's are. A volume writes no depth; an opaque iso-surface does.
// Overlays draw after them, on top.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/flow_sample.hpp"
#include "render/target.hpp"
#include "render/transfer_function.hpp"

namespace spade::render {

struct Lighting;  // render/scene.hpp, which includes this header

inline constexpr std::size_t kMaxFlowTransfers = 4;
inline constexpr std::size_t kMaxIsoSurfaces = 4;
inline constexpr uint32_t kIsoBisections = 4;

// One level set of one quantity. For the Q-criterion, iso on q_hat at
// sqrt(Q0): no division, no transcendental.
struct IsoSurface {
    FlowQuantity quantity = FlowQuantity::q_hat;
    float value = 0.0f;              // the level, in the quantity's units
    glm::vec3 colour{0.8f};          // used unless colour_by is set
    std::optional<TransferFunction> colour_by;  // only its quantity, colours and range are used
    float opacity = 1.0f;            // (0, 1]; 1 ends the ray
};

inline constexpr float kFlowEarlyExitTransmittance = 1.0f / 256.0f;

// One published volume and the transfer functions and iso-surfaces that
// draw it. The volume is the caller's (the airflow module's publish output)
// and outlives every render() that reads it.
struct FlowLayer {
    const FlowVolume* volume = nullptr;
    std::vector<TransferFunction> transfers;  // 0 to kMaxFlowTransfers
    std::vector<IsoSurface> isos;             // 0 to kMaxIsoSurfaces; one of the two is non-empty
};

// invalid_argument, by name: no volume, a volume that fails
// validate_flow_volume(), more than four transfer functions or iso-surfaces,
// neither, a transfer function that fails validate_transfer_function(), or
// an iso-surface whose level is not finite, whose opacity is outside
// (0, 1], whose colour is outside [0, 1], or whose colour_by fails
// validation.
[[nodiscard]] Result<void> validate_flow_layer(const FlowLayer& layer);

// A validated layer with its per-frame constants hoisted.
struct PreparedIso {
    IsoSurface iso;
    std::optional<PreparedTransfer> colour_by;
};

struct PreparedFlowLayer {
    FlowSampler sampler;
    std::vector<PreparedTransfer> transfers;
    std::vector<PreparedIso> isos;
    const Lighting* lighting = nullptr;  // the scene's sun for iso-surfaces; null draws them unlit
    Aabb box;
    float step = 0.0f;  // half a cell
    float inv_step = 0.0f;
};

// `lighting` (the scene's) outlives the prepared layer.
[[nodiscard]] PreparedFlowLayer prepare_flow_layer(const FlowLayer& layer, const Lighting* lighting = nullptr);

struct FlowRayResult {
    glm::vec3 colour{0.0f};      // C: emitted light reaching the eye
    float transmittance = 1.0f;  // T: what lets C_behind through
    bool crossed_no_data = false;
    float opaque_t = std::numeric_limits<float>::infinity();  // an opaque iso-surface's hit, else infinity
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
