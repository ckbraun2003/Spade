#include "render/agreement.hpp"

#include <cstdlib>
#include <string>
#include <utility>

namespace spade::render {
namespace {

// Packs 4 bytes in the SAME order render()/render_raymarch() write a BGRX8
// pixel (px[0]=B, px[1]=G, px[2]=R, px[3]=X -- raster_cpu.cpp's
// setPixelIfCloser()/raymarch.cpp's own final write, both cited in this
// file's own header comment) into one uint32_t, B in the low byte. Never
// exposed: a caller only ever needs the PACKED value (pack_sky_reference_bgrx
// below), never this function's own byte order convention.
[[nodiscard]] uint32_t pack_bgrx(uint8_t b, uint8_t g, uint8_t r, uint8_t x) {
    return static_cast<uint32_t>(b) | (static_cast<uint32_t>(g) << 8) | (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(x) << 24);
}

[[nodiscard]] uint32_t pixel_at(const RenderTarget& target, uint32_t x, uint32_t y) {
    const std::size_t idx = static_cast<std::size_t>(y) * target.stride + static_cast<std::size_t>(x) * 4;
    return pack_bgrx(target.pixels[idx], target.pixels[idx + 1], target.pixels[idx + 2], target.pixels[idx + 3]);
}

}  // namespace

AgreementResult compare_silhouettes(const RenderTarget& fast, const RenderTarget& raymarched,
                                     uint32_t sky_reference_rgb) {
    AgreementResult result;
    if (fast.width != raymarched.width || fast.height != raymarched.height) {
        // A caller error (comparing two differently-sized frames is
        // meaningless), not a geometry disagreement to measure -- total
        // disagreement, matching AgreementResult's own documented
        // all-covered-and-nothing-agrees 1.0 default, never a crash and
        // never a silent partial scan over whichever dimensions are smaller.
        return result;
    }

    const uint32_t width = fast.width, height = fast.height;
    uint64_t disagreeing_pixels = 0;
    // Fixed row-by-row, column-by-column scan order (determinism contract,
    // this file's own header comment) -- not that summation order could
    // change an integer count, but the SAME discipline every other pass in
    // this program follows unconditionally, not only where it would matter.
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const bool a_covered = pixel_at(fast, x, y) != sky_reference_rgb;
            const bool b_covered = pixel_at(raymarched, x, y) != sky_reference_rgb;
            if (a_covered) {
                ++result.covered_a;
            }
            if (b_covered) {
                ++result.covered_b;
            }
            if (a_covered != b_covered) {
                ++disagreeing_pixels;
            }
        }
    }

    const uint64_t total_pixels = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
    result.disagreement_fraction =
        total_pixels > 0 ? static_cast<double>(disagreeing_pixels) / static_cast<double>(total_pixels) : 0.0;
    return result;
}

uint32_t pack_sky_reference_bgrx(const Lighting& lighting) {
    return pack_bgrx(to_byte(lighting.sky_zenith.b), to_byte(lighting.sky_zenith.g), to_byte(lighting.sky_zenith.r),
                      0xFFu);
}

Result<void> assert_no_material_matches_sky(std::span<const spade::MaterialDesc> materials,
                                             uint32_t sky_reference_rgb, uint8_t tolerance) {
    const int sky_b = static_cast<int>(sky_reference_rgb & 0xFFu);
    const int sky_g = static_cast<int>((sky_reference_rgb >> 8) & 0xFFu);
    const int sky_r = static_cast<int>((sky_reference_rgb >> 16) & 0xFFu);

    for (std::size_t i = 0; i < materials.size(); ++i) {
        const spade::MaterialDesc& material = materials[i];
        const int b = static_cast<int>(to_byte(material.base_color.b));
        const int g = static_cast<int>(to_byte(material.base_color.g));
        const int r = static_cast<int>(to_byte(material.base_color.r));
        const int db = std::abs(b - sky_b);
        const int dg = std::abs(g - sky_g);
        const int dr = std::abs(r - sky_r);
        if (db <= tolerance && dg <= tolerance && dr <= tolerance) {
            return std::unexpected(Error{
                Code::invalid_argument,
                "material '" + material.name + "' (index " + std::to_string(i) + ") quantizes to BGR(" +
                    std::to_string(b) + "," + std::to_string(g) + "," + std::to_string(r) +
                    "), within tolerance " + std::to_string(static_cast<int>(tolerance)) +
                    " of the sky reference colour BGR(" + std::to_string(sky_b) + "," + std::to_string(sky_g) + "," +
                    std::to_string(sky_r) +
                    ") -- compare_silhouettes()'s colour-based silhouette classifier cannot tell this material's "
                    "coverage apart from a miss (S7a Task R9, Step 1c guard)"});
        }
    }
    return {};
}

}  // namespace spade::render
