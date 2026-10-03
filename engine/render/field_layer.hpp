// A field layer: a registered field drawn as a camera channel
// (docs/design/rendering/plans/2026-10-03-field-channel-plan.md).
//
// The caller samples the field and hands the values in, so the renderer
// never reads the simulation (L5). The layer is data, not appearance: every
// pixel is exactly its bin's palette colour, with no lighting, shadow or
// atmospheric term (SR-17a), and it draws from both sides.
//
// The palette and binning came from the drone sim box's heatmap
// (sandbox/drone_view.hpp) with their arithmetic unchanged, so its
// exact-pixel tests keep their numbers.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"

namespace spade::render {

// Control colours spaced evenly over [0, 1], blended piecewise linearly.
// Stops are in byte units (0..255), divided after blending, because the
// renderer has no transfer function: these are the bytes that reach the
// screen.
struct FieldPalette {
    std::vector<glm::vec3> stops;
};

// Viridis, over its nine standard control points
// (#440154 #482878 #3E4A89 #31688E #26828E #1F9E89 #35B779 #6DCD59 #FDE725).
[[nodiscard]] inline FieldPalette viridis_palette() {
    return FieldPalette{{
        {68.0f, 1.0f, 84.0f}, {72.0f, 40.0f, 120.0f}, {62.0f, 74.0f, 137.0f},
        {49.0f, 104.0f, 142.0f}, {38.0f, 130.0f, 142.0f}, {31.0f, 158.0f, 137.0f},
        {53.0f, 183.0f, 121.0f}, {109.0f, 205.0f, 89.0f}, {253.0f, 231.0f, 37.0f},
    }};
}

// Linear colour at t in [0, 1]; t outside is clamped. Needs two stops or more
// (validate_field_layer() checks).
[[nodiscard]] inline glm::vec3 palette_colour(const FieldPalette& palette, float t) {
    const std::vector<glm::vec3>& s = palette.stops;
    const float x = std::clamp(t, 0.0f, 1.0f) * static_cast<float>(s.size() - 1);
    const auto i = std::min(static_cast<std::size_t>(x), s.size() - 2);
    const float u = x - static_cast<float>(i);
    const glm::vec3& a = s[i];
    const glm::vec3& b = s[i + 1];
    return glm::vec3(a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u, a.z + (b.z - a.z) * u) / 255.0f;
}

struct FieldColourMap {
    FieldPalette palette = viridis_palette();
    uint32_t bins = 32;
    float range_min = 0.0f;  // the bottom of bin 0
    float range_max = 0.0f;  // the top of the last bin; at or below range_min, the layer's own maximum
};

// A world-space rectangle of cells_u x cells_v cells, `width` metres along
// `right` and `height` along `up`, centred on `center`. `right` and `up` are
// unit vectors. values[j * cells_u + i] belongs to cell (i, j), counted from
// the (-right, -up) corner, sampled at field_cell_center().
struct FieldLayer {
    glm::vec3 center{0.0f};
    glm::vec3 right{1.0f, 0.0f, 0.0f};
    glm::vec3 up{0.0f, 1.0f, 0.0f};
    float width = 1.0f;
    float height = 1.0f;
    uint32_t cells_u = 1;
    uint32_t cells_v = 1;
    std::vector<float> values;
    FieldColourMap colour_map;
};

// Where cell (i, j) is sampled: its centre.
[[nodiscard]] inline glm::vec3 field_cell_center(const FieldLayer& layer, uint32_t i, uint32_t j) {
    const float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(layer.cells_u) - 0.5f;
    const float v = (static_cast<float>(j) + 0.5f) / static_cast<float>(layer.cells_v) - 0.5f;
    return layer.center + layer.right * (u * layer.width) + layer.up * (v * layer.height);
}

// Lattice corner (i, j), for i in 0..cells_u and j in 0..cells_v. Neighbouring
// cells share these points exactly, so the layer has no cracks. GL's field
// shader computes the same expression.
[[nodiscard]] inline glm::vec3 field_cell_corner(const FieldLayer& layer, uint32_t i, uint32_t j) {
    const float u = static_cast<float>(i) / static_cast<float>(layer.cells_u) - 0.5f;
    const float v = static_cast<float>(j) / static_cast<float>(layer.cells_v) - 0.5f;
    return layer.center + layer.right * (u * layer.width) + layer.up * (v * layer.height);
}

// The top of the range: range_max when it is above range_min, else the
// largest finite value (never below range_min).
[[nodiscard]] inline float resolved_range_max(const FieldLayer& layer) {
    const FieldColourMap& map = layer.colour_map;
    if (map.range_max > map.range_min) return map.range_max;
    float top = map.range_min;
    for (const float v : layer.values) {
        if (std::isfinite(v) && v > top) top = v;
    }
    return top;
}

// 0..bins-1, and total: a non-finite value, a value at or below the bottom,
// or an empty or non-finite range is bin 0, never a division by zero.
[[nodiscard]] inline uint32_t field_bin(const FieldColourMap& map, float value, float range_max) {
    const float span = range_max - map.range_min;
    const float offset = value - map.range_min;
    if (!std::isfinite(value) || !std::isfinite(span) || !(span > 0.0f) || !(offset > 0.0f)) return 0u;
    const float x = offset / span * static_cast<float>(map.bins);
    if (!(x < static_cast<float>(map.bins - 1u))) return map.bins - 1u;
    return static_cast<uint32_t>(x);
}

// The colour a bin draws in: the palette at the centre of the bin's span.
[[nodiscard]] inline glm::vec3 field_bin_colour(const FieldColourMap& map, uint32_t bin) {
    return palette_colour(map.palette, (static_cast<float>(bin) + 0.5f) / static_cast<float>(map.bins));
}

// A malformed layer is refused before anything draws (L6), never drawn in part.
[[nodiscard]] inline Result<void> validate_field_layer(const FieldLayer& layer) {
    const auto refuse = [](std::string why) -> Result<void> {
        return std::unexpected(Error{Code::invalid_argument, "field layer: " + std::move(why)});
    };
    if (layer.cells_u == 0u || layer.cells_v == 0u) return refuse("it has no cells");
    const std::size_t cells = static_cast<std::size_t>(layer.cells_u) * layer.cells_v;
    if (layer.values.size() != cells) {
        return refuse(std::to_string(layer.values.size()) + " values for " + std::to_string(layer.cells_u) + "x" +
                      std::to_string(layer.cells_v) + " cells");
    }
    if (!(layer.width > 0.0f) || !(layer.height > 0.0f)) return refuse("its width and height must be positive");
    if (layer.colour_map.bins == 0u) return refuse("its colour map has no bins");
    if (layer.colour_map.palette.stops.size() < 2u) return refuse("its palette needs two stops or more");
    return {};
}

}  // namespace spade::render
