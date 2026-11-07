#pragma once

#include <array>
#include <cstdint>

// ---------------------------------------------------------------------------
// physics/lbm/lattice.hpp -- the LBM tier's D3Q27 lattice (L1; the design:
// docs/design/physics/plans/2026-11-04-l1-lbm-design.md, section 2).
//
// Lattice units throughout: dx = dt = 1, the sound speed c_s = 1/sqrt(3), so
// c_s^2 = 1/3 and the Hermite constants of the collision are 3, 4.5 and 4.5,
// all exact in fp32.
//
// THE ORDER. Direction 0 is the rest population. Directions 1..26 come in
// opposite pairs that are ADJACENT: (1, 2), (3, 4), ... , (25, 26), so
// opposite(i) is i + 1 for an odd i and i - 1 for an even one. The six faces
// come first, then the twelve edges, then the eight corners. The GPU twin
// holds the same tables, verbatim.
//
// THE WEIGHTS are fp32 literals (the nearest float to 8/27, 2/27, 1/54 and
// 1/216, each rounded once), so both twins hold the same bits.
// ---------------------------------------------------------------------------

namespace spade::physics::lbm {

inline constexpr uint32_t kQ = 27;

struct Vel {
    int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;
    friend constexpr bool operator==(const Vel&, const Vel&) = default;
};

inline constexpr std::array<Vel, kQ> kC = {{
    {0, 0, 0},
    // faces
    {1, 0, 0},  {-1, 0, 0},  {0, 1, 0},  {0, -1, 0},  {0, 0, 1},  {0, 0, -1},
    // edges
    {1, 1, 0},  {-1, -1, 0}, {1, -1, 0}, {-1, 1, 0},  {1, 0, 1},  {-1, 0, -1},
    {1, 0, -1}, {-1, 0, 1},  {0, 1, 1},  {0, -1, -1}, {0, 1, -1}, {0, -1, 1},
    // corners
    {1, 1, 1},  {-1, -1, -1}, {1, 1, -1}, {-1, -1, 1}, {1, -1, 1}, {-1, 1, -1}, {-1, 1, 1}, {1, -1, -1},
}};

inline constexpr float kWRest = 0.2962963f;     // 8 / 27
inline constexpr float kWFace = 0.074074075f;   // 2 / 27
inline constexpr float kWEdge = 0.018518519f;   // 1 / 54
inline constexpr float kWCorner = 0.0046296297f;  // 1 / 216

[[nodiscard]] constexpr float weight_of(Vel c) noexcept {
    const int n = c.x * c.x + c.y * c.y + c.z * c.z;
    return n == 0 ? kWRest : (n == 1 ? kWFace : (n == 2 ? kWEdge : kWCorner));
}

[[nodiscard]] constexpr std::array<float, kQ> make_weights() noexcept {
    std::array<float, kQ> w{};
    for (uint32_t i = 0; i < kQ; ++i) w[i] = weight_of(kC[i]);
    return w;
}

inline constexpr std::array<float, kQ> kW = make_weights();

[[nodiscard]] constexpr uint32_t opposite(uint32_t i) noexcept {
    return i == 0u ? 0u : ((i & 1u) != 0u ? i + 1u : i - 1u);
}

namespace detail {
[[nodiscard]] constexpr bool opposites_are_opposite() noexcept {
    for (uint32_t i = 0; i < kQ; ++i) {
        const Vel a = kC[i];
        const Vel b = kC[opposite(i)];
        if (a.x != -b.x || a.y != -b.y || a.z != -b.z) return false;
    }
    return true;
}
[[nodiscard]] constexpr bool directions_are_distinct() noexcept {
    for (uint32_t i = 0; i < kQ; ++i) {
        for (uint32_t j = i + 1; j < kQ; ++j) {
            if (kC[i] == kC[j]) return false;
        }
    }
    return true;
}
}  // namespace detail

static_assert(detail::opposites_are_opposite(), "opposite() must negate every direction");
static_assert(detail::directions_are_distinct(), "D3Q27 holds each of its 27 velocities once");

}  // namespace spade::physics::lbm
