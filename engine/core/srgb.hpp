#pragma once

// core/srgb.hpp -- RED stub: the public home of the linear to sRGB byte (moved from world/mesh_import/colour.hpp in the GREEN commit). Answers 0 for now.

#include <array>
#include <cstdint>

namespace spade {

inline constexpr std::array<double, 255> kSrgbThreshold{};

[[nodiscard]] inline uint8_t srgb_from_linear(double linear) {
    (void)linear;
    return 0;
}

}  // namespace spade
