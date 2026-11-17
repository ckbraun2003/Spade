// The particle look's seeds (Cameron, 2026-11-17: the airflow should surround the blades and feel like particles): where air
// is drawn in above and round each rotor, and a far-field box round the drone. A deterministic hash, no RNG: the same inputs
// give the same points. Used by the particle preview (tracers carried through one frozen frame) and by the real particle
// system after it.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <vector>

#include <glm/glm.hpp>

namespace flow_movie {

struct PreviewRotor {
    glm::vec3 hub;
    glm::vec3 axis;  // the way thrust points: "above" is along it
    float radius;
};

struct PreviewBox {
    glm::vec3 lo, hi;
};

namespace particles_detail {

// splitmix64 to [0, 1): a stateless hash of an index and a stream, so a seed never depends on how many came before it.
[[nodiscard]] inline float unit(uint64_t index, uint64_t stream) noexcept {
    uint64_t z = index * 0x9E3779B97F4A7C15ull + stream * 0xBF58476D1CE4E5B9ull + 0x94D049BB133111EBull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return static_cast<float>(z >> 40) * (1.0f / 16777216.0f);
}

}  // namespace particles_detail

// `per_rotor` seeds in the shell above each rotor (0.02 to 0.35 m along its axis, thicker near the disc, within two radii of the
// axis), then `far` seeds filling `box`. Rotor r's seeds come first, in order.
[[nodiscard]] inline std::vector<glm::vec3> preview_seeds(const std::vector<PreviewRotor>& rotors, const PreviewBox& box,
                                                          std::size_t per_rotor, std::size_t far) {
    using particles_detail::unit;
    std::vector<glm::vec3> out;
    out.reserve(rotors.size() * per_rotor + far);
    for (std::size_t r = 0; r < rotors.size(); ++r) {
        const PreviewRotor& rotor = rotors[r];
        const glm::vec3 axis = glm::normalize(rotor.axis);
        const glm::vec3 helper = std::abs(axis.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 e1 = glm::normalize(glm::cross(axis, helper));
        const glm::vec3 e2 = glm::cross(axis, e1);
        for (std::size_t i = 0; i < per_rotor; ++i) {
            const uint64_t k = r * 1000003ull + i;
            const float theta = 2.0f * std::numbers::pi_v<float> * unit(k, 1);
            const float radial = rotor.radius * (0.05f + 1.95f * std::sqrt(unit(k, 2)));  // an even fill of the disc of radius 2R
            const float height = 0.02f + 0.33f * unit(k, 3) * unit(k, 3);                  // thicker near the disc
            out.push_back(rotor.hub + axis * height + (e1 * std::cos(theta) + e2 * std::sin(theta)) * radial);
        }
    }
    for (std::size_t i = 0; i < far; ++i) {
        const uint64_t k = 7000000ull + i;
        out.emplace_back(box.lo.x + (box.hi.x - box.lo.x) * unit(k, 4), box.lo.y + (box.hi.y - box.lo.y) * unit(k, 5),
                         box.lo.z + (box.hi.z - box.lo.z) * unit(k, 6));
    }
    return out;
}

}  // namespace flow_movie
