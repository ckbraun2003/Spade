#pragma once

// ---------------------------------------------------------------------------
// A planar texture (SP-4, branch 2; docs/design/rendering/plans/2026-11-17-geo-terrain-design.md): an RGB8 image laid over the draw items
// that ask for it, mapped by WORLD position, so it needs no UVs and is perspective-correct by construction (the raster recovers each pixel's
// world position, as the shadow and the haze already do). The image's outer edges sit at world x in [x_min, x_max] and z in [z_min, z_max];
// row 0 is at z_min (north, z = -north) and column 0 at x_min (west). A sample is bilinear between texel centres and clamped at the edge.
//
// The colour is the surface's ALBEDO: the item is lit as if its material were white and the lighting (ambient, the sun's N.L, the shadow)
// multiplies the sampled colour, so a constant image equals the same item drawn with that colour as its material.
//
// STUB (RED): nothing here is written yet.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"

namespace spade::render {

struct PlanarTexture {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint8_t> rgb;  // width x height x 3, row 0 first
    float x_min = 0.0f, x_max = 0.0f;
    float z_min = 0.0f, z_max = 0.0f;
};

// invalid_argument, by name: a zero side, a byte count that is not width x height x 3, an extent that is not finite or not wider than 0 on
// either axis.
[[nodiscard]] inline Result<void> validate_planar_texture(const PlanarTexture&) { return {}; }

// The image's colour at world (x, z), each channel in [0, 1].
[[nodiscard]] inline glm::vec3 sample_planar_texture(const PlanarTexture&, float, float) { return glm::vec3(0.0f); }

}  // namespace spade::render
