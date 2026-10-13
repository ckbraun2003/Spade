#include "views.hpp"

#include <cmath>
#include <numbers>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace flow_movie {

namespace render = spade::render;

render::RenderScene dark_scene(const spade::FlowVolume& volume) {
    render::RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.035f, 0.04f, 0.05f);
    scene.lighting.sky_horizon = glm::vec3(0.075f, 0.08f, 0.095f);
    const glm::vec3 lo = volume.header.origin;
    scene.bounds = render::Aabb{.min = lo, .max = lo + volume_size(volume)};
    return scene;
}

glm::vec3 volume_size(const spade::FlowVolume& volume) {
    const spade::FlowVolumeHeader& h = volume.header;
    return glm::vec3(static_cast<float>(h.nx), static_cast<float>(h.ny), static_cast<float>(h.nz)) * h.cell;
}

glm::vec3 volume_centre(const spade::FlowVolume& volume) {
    return volume.header.origin + 0.5f * volume_size(volume);
}

render::Camera orbit(glm::vec3 target, float distance, float azimuth_deg, float elevation_deg) {
    const float az = glm::radians(azimuth_deg), el = glm::radians(elevation_deg);
    const glm::vec3 from(std::cos(el) * std::sin(az), std::sin(el), std::cos(el) * std::cos(az));
    render::Camera camera;
    camera.position = target + distance * from;
    camera.orientation = glm::quatLookAt(-from, glm::vec3(0.0f, 1.0f, 0.0f));
    camera.near_plane = 0.01f * distance;
    camera.far_plane = 100.0f * distance;
    return camera;
}

render::TransferFunction volume_transfer(render::FlowQuantity quantity, render::ColourMapId map, float range_max,
                                         float onset, float sigma) {
    render::TransferFunction tf;
    tf.quantity = quantity;
    tf.colours = map;
    tf.range_min = 0.0f;
    tf.range_max = range_max;
    tf.extinction = {{onset * range_max, 0.0f}, {range_max, sigma}};
    return tf;
}

render::TransferFunction colours_over(render::FlowQuantity quantity, render::ColourMapId map, float range_min,
                                      float range_max) {
    render::TransferFunction tf;
    tf.quantity = quantity;
    tf.colours = map;
    tf.range_min = range_min;
    tf.range_max = range_max;
    return tf;
}

spade::Result<void> add_streamlines(render::RenderScene& scene, const spade::FlowVolume& volume,
                                    std::span<const glm::vec3> seeds, const render::StreamlineOptions& options,
                                    const render::TransferFunction& by_speed, const glm::vec3* constant) {
    auto lines = render::trace_streamlines(volume, seeds, options);
    if (!lines) return std::unexpected(lines.error());
    const render::PreparedTransfer prepared = render::prepare_transfer(by_speed, volume.header);
    for (const render::Streamline& line : *lines) {
        if (line.points.size() < 2u) continue;
        render::FlowPolyline poly = render::colour_streamline(line, prepared);
        if (constant != nullptr) poly.colours.assign(poly.points.size(), *constant);
        scene.flow_polylines.push_back(std::move(poly));
    }
    return {};
}

spade::Result<void> add_slice(render::RenderScene& scene, const spade::FlowVolume& volume,
                              render::FlowQuantity quantity, render::FieldLayer geometry, render::ColourMapId map,
                              float range_min, float range_max) {
    auto slice = render::slice_flow_volume(volume, quantity, geometry);
    if (!slice) return std::unexpected(slice.error());
    slice->colour_map.palette = render::field_palette_of(map);
    slice->colour_map.bins = 128;
    slice->colour_map.range_min = range_min;
    slice->colour_map.range_max = range_max;
    scene.field_layers.push_back(std::move(*slice));
    return {};
}

render::FlowPolyline circle(glm::vec3 centre, glm::vec3 axis, float radius, glm::vec3 colour, uint32_t segments) {
    const glm::vec3 n = glm::normalize(axis);
    const glm::vec3 helper = std::abs(n.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 e1 = glm::normalize(glm::cross(n, helper));
    const glm::vec3 e2 = glm::cross(n, e1);
    render::FlowPolyline line;
    for (uint32_t s = 0; s <= segments; ++s) {
        const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(s) / static_cast<float>(segments);
        line.points.push_back(centre + radius * (std::cos(a) * e1 + std::sin(a) * e2));
        line.colours.push_back(colour);
    }
    return line;
}

}  // namespace flow_movie
