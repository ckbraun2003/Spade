// A drawn wake's soft edge (the lead, 2026-11-17): where the volume is made to fade out before the picture shows a cut. Shared by
// look.hpp (the fade itself and its caption sentence) and web_masters.hpp (each drone's own numbers).
#pragma once

namespace flow_movie {

// side_m: the velocity fades to nothing over the last side_m metres before each of the box's four side faces (x and z); 0 none.
// floor_from_y, floor_to_y: world heights (the drone's centre of mass is y = 0): full above floor_from_y, nothing at or below
// floor_to_y, a smoothstep between; the fade is on when floor_to_y is below floor_from_y. ceiling_from_y, ceiling_to_y: the same
// upward: full below ceiling_from_y, nothing at or above ceiling_to_y; on when ceiling_to_y is above ceiling_from_y.
struct WakeFade {
    float side_m = 0.0f;
    float floor_from_y = 0.0f;
    float floor_to_y = 0.0f;
    float ceiling_from_y = 0.0f;
    float ceiling_to_y = 0.0f;

    [[nodiscard]] bool operator==(const WakeFade&) const noexcept = default;
    [[nodiscard]] bool on() const noexcept {
        return side_m > 0.0f || floor_to_y < floor_from_y || ceiling_to_y > ceiling_from_y;
    }
};

}  // namespace flow_movie
