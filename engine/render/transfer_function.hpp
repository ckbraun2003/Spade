#pragma once

// ---------------------------------------------------------------------------
// transfer_function -- how a flow-volume value becomes colour and extinction
// (rendering/plans/2026-10-05-airflow-visualization-plan.md section 2).
//
// COLOUR MAPS are published, perceptually uniform tables of 256 entries,
// committed with their provenance (colour_maps.cpp): viridis for speed,
// Moreland's cool-warm diverging map for pressure, inferno for vorticity and
// dye. Colours are display values in [0, 1], as the renderer's other colours
// are, and compositing happens in that space, as the field layer's does.
//
// EXTINCTION sigma(value), in 1/m, is piecewise linear in 2 to 8 points of
// the quantity's own units, held constant past the end points. So a band of
// interest can be made dense and the rest clear.
//
// THE RANGE (the lead's Q5): fixed per scene by default, because comparing
// frames or runs needs one mapping. auto_range is an explicit opt-in that
// takes the volume header's range and ANNOUNCES the range it chose
// (ResolvedRange, L6). A diverging map's auto range is symmetric about zero,
// so zero gauge pressure stays the map's neutral centre.
// ---------------------------------------------------------------------------

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"

namespace spade::render {

enum class ColourMapId : uint32_t {
    viridis,    // sequential: speed
    inferno,    // sequential: vorticity, dye
    cool_warm,  // diverging, neutral at the centre: pressure
};
inline constexpr std::size_t kColourMapEntries = 256;

[[nodiscard]] const std::array<glm::vec3, kColourMapEntries>& colour_map_table(ColourMapId id);

// The map at t in [0, 1], linear between entries; t outside is clamped.
[[nodiscard]] glm::vec3 colour_map_at(ColourMapId id, float t);

// What a layer shows. speed is |(u, v, w)|; the rest are a channel each.
// lambda2_hat is sign(lambda2) sqrt(|lambda2|) (core/lambda2.hpp), below
// zero in a vortex core. Quantity n > 0 reads FlowChannel n - 1; the GL
// shaders rely on that order.
enum class FlowQuantity : uint32_t { speed, u, v, w, dye, pressure, vorticity, q_hat, lambda2_hat };

struct ExtinctionPoint {
    float value = 0.0f;  // in the quantity's units
    float sigma = 0.0f;  // 1/m, >= 0
};
inline constexpr std::size_t kMaxExtinctionPoints = 8;

struct TransferFunction {
    FlowQuantity quantity = FlowQuantity::speed;
    ColourMapId colours = ColourMapId::viridis;
    // Smoke: one constant in-scatter colour in place of the map.
    std::optional<glm::vec3> constant_colour;
    float range_min = 0.0f;  // the map's bottom, used unless auto_range
    float range_max = 1.0f;  // the map's top
    bool auto_range = false;
    std::vector<ExtinctionPoint> extinction{{0.0f, 0.0f}, {1.0f, 1.0f}};  // values strictly ascending
};

// invalid_argument, by name, for: a fixed range that is not finite with
// min < max; fewer than 2 or more than 8 extinction points; values not
// finite and strictly ascending; a sigma that is not finite and >= 0; a
// constant colour outside [0, 1].
[[nodiscard]] Result<void> validate_transfer_function(const TransferFunction& tf);

// The range a layer draws with, and where it came from.
struct ResolvedRange {
    float min = 0.0f;
    float max = 1.0f;
    bool from_header = false;  // true: auto_range chose it, and a caller announces it
};

// The fixed range, or with auto_range the header's: the channel's own min
// and max; for speed, 0 to the largest |(u, v, w)| the per-channel ranges
// allow; for a diverging map, symmetric about zero. Precondition: a valid
// transfer function and a validated volume.
[[nodiscard]] ResolvedRange resolve_range(const TransferFunction& tf, const FlowVolumeHeader& header);

// A transfer function with its per-call constants hoisted (reciprocals on
// the host, as the solver's are).
struct PreparedTransfer {
    TransferFunction tf;
    ResolvedRange range;
    float inv_span = 0.0f;  // 1 / (max - min), or 0 for an empty span (every value maps to t = 0)
    std::array<float, kMaxExtinctionPoints - 1> extinction_inv_width{};  // 1 / each segment's width
};

[[nodiscard]] PreparedTransfer prepare_transfer(const TransferFunction& tf, const FlowVolumeHeader& header);

struct TransferSample {
    glm::vec3 colour{0.0f};
    float sigma = 0.0f;  // 1/m
};

// The colour and extinction of `value`. Precondition: `value` is finite (a
// no-data sample never reaches a transfer function).
[[nodiscard]] TransferSample apply_transfer(const PreparedTransfer& prepared, float value);

}  // namespace spade::render
