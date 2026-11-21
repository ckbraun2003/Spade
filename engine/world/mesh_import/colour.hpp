#pragma once

// world/mesh_import/colour.hpp -- the colour an object carries, for the readers' collecting mode (L88t). Internal to spade::mesh_import.
//
// The colour is sRGB 0..255. An object may carry several; the one covering the most surface area is the object's colour, the lowest (r, g, b)
// on a tie, and the count of distinct colours goes beside it. Surface with no colour counts for nothing.
//
// glTF's baseColorFactor is LINEAR; 3MF's and STEP's colours are sRGB. The linear to sRGB conversion is core/srgb.hpp's (the engine has no libm:
// BitPortability), shared with the camera's shading so there is one conversion.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string_view>

#include <glm/glm.hpp>

#include "core/srgb.hpp"
#include "world/mesh_import.hpp"

namespace spade::detail {





// "#RRGGBB" or "#RRGGBBAA" (3MF), alpha ignored.
[[nodiscard]] inline std::optional<std::array<uint8_t, 3>> parse_hex_colour(std::string_view text) {
    if (text.size() != 7 && text.size() != 9) return std::nullopt;
    if (text[0] != '#') return std::nullopt;
    const auto nibble = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    std::array<uint8_t, 3> rgb{};
    for (std::size_t k = 0; k < 3; ++k) {
        const int hi = nibble(text[1 + 2 * k]), lo = nibble(text[2 + 2 * k]);
        if (hi < 0 || lo < 0) return std::nullopt;
        rgb[k] = static_cast<uint8_t>(hi * 16 + lo);
    }
    if (text.size() == 9 && (nibble(text[7]) < 0 || nibble(text[8]) < 0)) return std::nullopt;
    return rgb;
}

[[nodiscard]] inline double triangle_area(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) {
    return 0.5 * std::sqrt(glm::dot(glm::cross(b - a, c - a), glm::cross(b - a, c - a)));
}

// The colours of an object's triangles, by area.
class ColourTally {
public:
    void add(const std::array<uint8_t, 3>& rgb, double area) {
        const uint32_t key = (static_cast<uint32_t>(rgb[0]) << 16) | (static_cast<uint32_t>(rgb[1]) << 8) | rgb[2];
        area_[key] += area;
    }
    [[nodiscard]] std::optional<SurfaceColour> result() const {
        if (area_.empty()) return std::nullopt;
        uint32_t best = area_.begin()->first;  // the map is ordered, so the first of equal areas is the lowest (r, g, b)
        double best_area = area_.begin()->second;
        for (const auto& [key, area] : area_) {
            if (area > best_area) {
                best = key;
                best_area = area;
            }
        }
        SurfaceColour out;
        out.rgb = {static_cast<uint8_t>(best >> 16), static_cast<uint8_t>((best >> 8) & 0xFFu), static_cast<uint8_t>(best & 0xFFu)};
        out.colours = static_cast<uint32_t>(area_.size());
        return out;
    }

private:
    std::map<uint32_t, double> area_;
};

}  // namespace spade::detail
