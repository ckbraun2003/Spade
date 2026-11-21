#include "render/camera_shade.hpp"

#include "core/srgb.hpp"  // spade::srgb_from_linear: the exact round-to-nearest sRGB byte, no libm

namespace spade::render {

glm::vec3 shade_ray(const RenderScene& scene, const RayScene& rays, const Ray& ray, const CameraShading& shading) {
    const glm::vec3 dir(ray.direction);
    const RayHit hit = rays.trace(ray);
    const glm::vec3 sky = sky_gradient_color(scene.lighting, dir);
    if (hit.kind == RayHitKind::none) return sky;
    const Material fallback{};
    const Material& material = hit.material < scene.materials.size() ? scene.materials[hit.material] : fallback;
    const glm::vec3 surface = shade_vertex_color(material, scene.lighting, hit.normal).combined;
    return horizon_blend(surface, sky, static_cast<float>(hit.t), shading.atmosphere_onset, shading.atmosphere_strength);
}

uint8_t srgb_byte(float linear) { return spade::srgb_from_linear(static_cast<double>(linear)); }

uint32_t channel_count(const CameraShading& shading) { return shading.grey ? 1u : 3u; }

uint32_t encode_pixel(const glm::vec3& linear, const CameraShading& shading, uint8_t* out) {
    const glm::vec3 exposed = linear * shading.exposure;
    if (shading.grey) {
        out[0] = srgb_byte(0.2126f * exposed.x + 0.7152f * exposed.y + 0.0722f * exposed.z);
        return 1u;
    }
    out[0] = srgb_byte(exposed.x);
    out[1] = srgb_byte(exposed.y);
    out[2] = srgb_byte(exposed.z);
    return 3u;
}

}  // namespace spade::render
