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
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
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
[[nodiscard]] inline Result<void> validate_planar_texture(const PlanarTexture& t) {
    const auto refuse = [](const std::string& why) { return std::unexpected(Error{Code::invalid_argument, "planar texture: " + why}); };
    if (t.width == 0u) return refuse("its width is 0");
    if (t.height == 0u) return refuse("its height is 0");
    const std::size_t want = std::size_t{t.width} * t.height * 3u;
    if (t.rgb.size() != want) {
        return refuse("it holds " + std::to_string(t.rgb.size()) + " bytes, not " + std::to_string(t.width) + " x " + std::to_string(t.height) + " x 3 = " + std::to_string(want));
    }
    if (!(std::isfinite(t.x_min) && std::isfinite(t.x_max) && t.x_max > t.x_min)) return refuse("its x extent is not finite and wider than 0");
    if (!(std::isfinite(t.z_min) && std::isfinite(t.z_max) && t.z_max > t.z_min)) return refuse("its z extent is not finite and wider than 0");
    return {};
}

// The image's colour at world (x, z), each channel in [0, 1].
[[nodiscard]] inline glm::vec3 sample_planar_texture(const PlanarTexture& t, float x, float z) {
    // Texel centres sit half a texel in from the extent's edges; the position is taken to texel units in double, the two neighbours blended, and an index past either end clamped.
    const double u = (static_cast<double>(x) - t.x_min) / (static_cast<double>(t.x_max) - t.x_min) * t.width - 0.5;
    const double v = (static_cast<double>(z) - t.z_min) / (static_cast<double>(t.z_max) - t.z_min) * t.height - 0.5;
    const double fu = std::floor(u), fv = std::floor(v);
    const double tx = u - fu, ty = v - fv;
    const auto at = [&](double i, double j, int c) {
        const std::size_t ci = static_cast<std::size_t>(std::clamp(i, 0.0, static_cast<double>(t.width - 1u)));
        const std::size_t cj = static_cast<std::size_t>(std::clamp(j, 0.0, static_cast<double>(t.height - 1u)));
        return static_cast<double>(t.rgb[(cj * t.width + ci) * 3u + static_cast<std::size_t>(c)]) / 255.0;
    };
    glm::vec3 out(0.0f);
    for (int c = 0; c < 3; ++c) {
        const double top = at(fu, fv, c) * (1.0 - tx) + at(fu + 1.0, fv, c) * tx;
        const double bottom = at(fu, fv + 1.0, c) * (1.0 - tx) + at(fu + 1.0, fv + 1.0, c) * tx;
        out[c] = static_cast<float>(top * (1.0 - ty) + bottom * ty);
    }
    return out;
}

}  // namespace spade::render
