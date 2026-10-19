#include "render/flow_look.hpp"

#include <cmath>
#include <numbers>
#include <utility>

#include <glm/glm.hpp>

#include "core/fp32_math.hpp"

namespace spade::render {

TransferFunction volume_transfer(FlowQuantity quantity, ColourMapId map, float range_max, float onset, float sigma) {
    TransferFunction tf;
    tf.quantity = quantity;
    tf.colours = map;
    tf.range_min = 0.0f;
    tf.range_max = range_max;
    tf.extinction = {{onset * range_max, 0.0f}, {range_max, sigma}};
    return tf;
}

TransferFunction colours_over(FlowQuantity quantity, ColourMapId map, float range_min, float range_max) {
    TransferFunction tf;
    tf.quantity = quantity;
    tf.colours = map;
    tf.range_min = range_min;
    tf.range_max = range_max;
    return tf;
}

Result<void> add_streamlines(RenderScene& scene, const FlowVolume& volume, std::span<const glm::vec3> seeds,
                             const StreamlineOptions& options, const TransferFunction& by_speed,
                             const glm::vec3* constant) {
    auto lines = trace_streamlines(volume, seeds, options);
    if (!lines) return std::unexpected(lines.error());
    const PreparedTransfer prepared = prepare_transfer(by_speed, volume.header);
    for (const Streamline& line : *lines) {
        if (line.points.size() < 2u) continue;
        FlowPolyline poly = colour_streamline(line, prepared);
        if (constant != nullptr) poly.colours.assign(poly.points.size(), *constant);
        scene.flow_polylines.push_back(std::move(poly));
    }
    return {};
}

Result<void> add_slice(RenderScene& scene, const FlowVolume& volume, FlowQuantity quantity, FieldLayer geometry,
                       ColourMapId map, float range_min, float range_max) {
    auto slice = slice_flow_volume(volume, quantity, geometry);
    if (!slice) return std::unexpected(slice.error());
    slice->colour_map.palette = field_palette_of(map);
    slice->colour_map.bins = 128;
    slice->colour_map.range_min = range_min;
    slice->colour_map.range_max = range_max;
    scene.field_layers.push_back(std::move(*slice));
    return {};
}

FlowPolyline circle(glm::vec3 centre, glm::vec3 axis, float radius, glm::vec3 colour, uint32_t segments) {
    const glm::vec3 n = glm::normalize(axis);
    const glm::vec3 helper = std::abs(n.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 e1 = glm::normalize(glm::cross(n, helper));
    const glm::vec3 e2 = glm::cross(n, e1);
    FlowPolyline line;
    line.points.reserve(segments + 1u);
    line.colours.reserve(segments + 1u);
    for (uint32_t s = 0; s <= segments; ++s) {
        // The last point is the first, exactly: a closed rim.
        const uint32_t k = s == segments ? 0u : s;
        const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(k) / static_cast<float>(segments);
        line.points.push_back(centre + radius * (math::cos32(a) * e1 + math::sin32(a) * e2));
        line.colours.push_back(colour);
    }
    return line;
}

}  // namespace spade::render
