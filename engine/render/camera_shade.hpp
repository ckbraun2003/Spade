#pragma once

// ---------------------------------------------------------------------------
// camera_shade -- the camera's shading: one ray to a linear colour, and a linear colour to bytes (camera sensor design, section 5, "one pixel"; Phase 1, P1.3).
//
// THE ONE DEFINITION OF A LIT PIXEL, PORTED NOT REIMPLEMENTED (design section 5; 01-techniques-and-channels.md). A ray that hits shades with the scene's own
// shade_vertex_color (Lambert + ambient; unlit and emissive echo the base colour) on the normal that FACES the ray (RayHit::normal), a ray that misses is the scene's
// sky_gradient_color by its elevation, and the atmosphere (SR-17a) blends the surface toward the sky at range by the scene's own horizon_blend (strength 0, the default,
// is exact). Shadows are off in v1 (design section 5). Textures and the ground grid are not in this phase.
//
// THE PIXEL. The linear colour takes the exposure gain (linear, BEFORE the clamp, so a later exposure jitter is a seeded gain, not a new pipeline), then the sensor mode:
// colour keeps three channels; grey is one, the luminance Y = 0.2126 R + 0.7152 G + 0.0722 B of the exposed linear colour. Then the sRGB transfer curve to a byte rounded
// to nearest (v1 is sRGB only, section 11a). The engine has no pow: the byte is the exact round-to-nearest of the curve by the engine's own threshold table
// (core/srgb.hpp's threshold table: the code of a linear value is the number of thresholds at or below it), which is exact, so the design's 1,024-entry
// interpolated table (an error to state and pin) is not needed on the CPU; the GPU port in Phase 2 may still want a table and is then banded against this.
// No libm: + - * / and sqrt only (SR-14).
// ---------------------------------------------------------------------------

#include <cstdint>

#include <glm/glm.hpp>

#include "render/ray_scene.hpp"
#include "render/scene.hpp"

namespace spade::render {

struct CameraShading {
    float exposure = 1.0f;             // a linear gain on the shaded colour, before the clamp
    bool grey = false;                 // one channel, the luminance of the exposed linear colour
    float atmosphere_strength = 0.0f;  // SR-17a: 0 is exactly off
    float atmosphere_onset = 45.0f;    // metres: the range at which the sky is half way to its strength
};

// The linear colour of one ray (before the exposure): the nearest hit shaded with the scene's lighting and its material, or the sky on a miss.
[[nodiscard]] glm::vec3 shade_ray(const RenderScene& scene, const RayScene& rays, const Ray& ray, const CameraShading& shading = {});

// The sRGB byte of a linear channel: clamped to [0, 1], the exact round-to-nearest of the curve. A NaN is 0.
[[nodiscard]] uint8_t srgb_byte(float linear);

// Bytes a pixel: 3 for colour, 1 for grey.
[[nodiscard]] uint32_t channel_count(const CameraShading& shading);

// The exposure gain, the sensor mode and the curve: writes channel_count(shading) bytes at `out` (R, G, B, or the grey) and returns that count.
uint32_t encode_pixel(const glm::vec3& linear, const CameraShading& shading, uint8_t* out);

}  // namespace spade::render
