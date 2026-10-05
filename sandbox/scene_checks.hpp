// What the live smoke's scene main functions measure, display-free (SL15b).
//
// Each built-in scene is in the tour to show something, and its step checks
// that it did: bounce's lanes rebound in the order of their restitution, and
// gate's lobs pass through the ring. The measurements are here, asserted by
// tests/test_sandbox_scene_checks.cpp; live_tour.cpp feeds them the run.

#pragma once

#include <cmath>
#include <optional>
#include <span>

#include <glm/glm.hpp>

namespace spade::sandbox::checks {

// The highest a body rose after it first came down to `contact_y` (its centre
// one contact radius above the ground, plus a margin), or none if it never
// landed. The height it was dropped from is before contact, so it never
// counts.
[[nodiscard]] inline std::optional<float> rebound_height(std::span<const float> heights, float contact_y) {
    std::optional<float> peak;
    for (const float h : heights) {
        if (!peak) {
            if (h <= contact_y) peak = h;
            continue;
        }
        if (h > *peak) peak = h;
    }
    return peak;
}

[[nodiscard]] inline bool strictly_increasing(std::span<const float> v) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        if (!(v[i] > v[i - 1])) return false;
    }
    return true;
}

// Where a body crossed the plane z = 0 between two samples, moving towards +z
// (the way the gate scene throws), by linear interpolation; none otherwise.
struct GateCrossing {
    float x = 0.0f;
    float y = 0.0f;
};

[[nodiscard]] inline std::optional<GateCrossing> crossing_z0(glm::vec3 before, glm::vec3 after) {
    if (!(before.z < 0.0f && after.z >= 0.0f)) return std::nullopt;
    const float t = -before.z / (after.z - before.z);
    return GateCrossing{before.x + t * (after.x - before.x), before.y + t * (after.y - before.y)};
}

// True when a crossing passes inside a ring of clear radius `clear_radius`
// centred at (0, centre_y) in the z = 0 plane.
[[nodiscard]] inline bool through_ring(GateCrossing c, float centre_y, float clear_radius) {
    return std::hypot(c.x, c.y - centre_y) < clear_radius;
}

}  // namespace spade::sandbox::checks
