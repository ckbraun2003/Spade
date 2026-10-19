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

}  // namespace flow_movie
