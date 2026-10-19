// A screen pixel as a ray into the world: what a click in a view picks with.
// Display-free (SL15b). Moved from builder_scene.hpp when the builder mode
// left the sandbox (INT-9); the editor's pick and gizmo use it.

#pragma once

#include <cmath>
#include <cstdint>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/target.hpp"

namespace spade::sandbox {

struct Ray {
    glm::vec3 origin{0.0f};
    glm::vec3 direction{0.0f, 0.0f, -1.0f};   // unit
};

// Screen pixel -> world ray.
//
// THE CONVENTION IS THE ENGINE'S AND IS NOT NEGOTIABLE HERE: camera space is
// right-handed with FORWARD = local -Z (raster_cpu.cpp's own derivation), and
// Camera::orientation maps local -> world. `px`/`py` are framebuffer pixels
// with the ORIGIN AT TOP-LEFT and +y DOWN, which is what every mouse API
// reports and the opposite of what NDC uses -- hence the flip on y and not on
// x. Getting that flip wrong produces a picker that is correct along one axis
// and mirrored along the other, which reads as "picking is flaky".
[[nodiscard]] inline Ray ray_from_screen(const render::Camera& camera, float px, float py,
                                         uint32_t width, uint32_t height) noexcept {
    const float w = static_cast<float>(width == 0u ? 1u : width);
    const float h = static_cast<float>(height == 0u ? 1u : height);
    const float tan_half = std::tan(camera.fov_y_radians * 0.5f);
    const float aspect = w / h;
    const float ndc_x = (2.0f * (px + 0.5f) / w - 1.0f) * aspect * tan_half;
    const float ndc_y = (1.0f - 2.0f * (py + 0.5f) / h) * tan_half;
    const glm::vec3 dir_cam = glm::normalize(glm::vec3(ndc_x, ndc_y, -1.0f));
    Ray r;
    r.origin = camera.position;
    r.direction = glm::normalize(camera.orientation * dir_cam);
    return r;
}

}  // namespace spade::sandbox
