#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// AN ACTUATOR DISC (spec §3.1): one step's impulse thrust * h along `axis`,
// carried by n points on rings and spread with Peskin's 4-point kernel onto
// the velocity increments a fluid step adds. A gather, so the GPU twin needs
// no atomics.
//
// THE DENSITY (the lead, 2026-10-27): the points follow R/dx, about 2 dx
// apart at most, so no face takes more than a few m/s a step (triage 2,
// 2026-10-27: 3 x 16 at R/dx 19 drove a 150 m/s jet in D1; 10 x 64 keeps it
// near 20) and a coarse grid pays for no more points than it resolves
// (R/dx 3: 3 x 16, the editor's Run air at its old speed). disc_density():
// rings = clamp(ceil(R / 2dx), 3, kDiscRings), angles the smallest of 16, 32
// and 64 at or above pi R/dx (64 if none is), n = rings x angles. Above
// R/dx 20 the cap leaves the points over 2 dx apart; disc_spacing_note()
// names that for the run line.
//
// THE SWIRL (the demonstration plan's P1): `torque` goes to the air as a
// tangential force at each point, torque / (n r) on its ring of radius r,
// along axis x (point - centre). That is constant circulation, the ideal
// rotor under the disc's uniform thrust loading, and the points' sum of
// r |f| is the torque exactly. The kernel keeps first moments, so the air's
// angular momentum about the centre gains h * torque along `axis` and the
// swirl adds no net force. A torque of 0 spreads exactly what the thrust-only
// disc did, bit for bit.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct ActuatorDisc {
    glm::vec3 centre{0.0f};             // metres, in the grid's frame
    glm::vec3 axis{0.0f, -1.0f, 0.0f};  // unit: the direction the disc pushes the air
    float radius = 0.0f;                // metres
    float thrust = 0.0f;                // newtons: the force the air takes along `axis`
    // newton-metres: the torque the air takes about `axis`, by the right-hand
    // rule. The air swirls with the blades, so a rotor spinning s (+1 or -1)
    // about its thrust axis -axis gives the air -s Q here (the coupler's rule).
    float torque = 0.0f;
};

// The most rings, angles and points a disc takes: kDiscPoints is the room in
// DiscSpread and in the GPU's disc record, whatever a disc's own count.
inline constexpr uint32_t kDiscRings = 10;
inline constexpr uint32_t kDiscAngles = 64;
inline constexpr uint32_t kDiscPoints = kDiscRings * kDiscAngles;

// A disc's points by its R/dx (THE DENSITY above).
struct DiscDensity {
    uint32_t rings = 0;
    uint32_t angles = 0;  // 16, 32 or 64: a power of 2, so kDiscAngles' table serves each
    uint32_t count = 0;   // rings x angles, at most kDiscPoints
};

[[nodiscard]] DiscDensity disc_density(float radius, float dx) noexcept;

// The run line's note on discs whose points the cap leaves over 2 dx apart:
// empty when none is. A disc's spacing is the larger of its outer ring's arc,
// 2 pi r_outer / angles, and its rings' radial pitch, R / rings, in dx. Discs
// of one radius are named together ("discs 0-3 at R/dx 25.0 space their
// points 2.4 dx apart (over 2 dx)").
[[nodiscard]] std::string disc_spacing_note(std::span<const float> radii, float dx);

// Peskin's 4-point kernel, in cells.
[[nodiscard]] float peskin4(float r) noexcept;

// Refused (invalid_argument): a radius not positive and finite; a thrust or a
// torque not finite; an axis not of unit length; a kernel support (radius +
// 2 dx about the centre) that leaves the grid.
[[nodiscard]] Result<void> check_disc(const ActuatorDisc& disc, const GridShape& shape);

// One disc's spread, formed on the host: what spread_actuator() adds from.
// Its second consumer is the GPU's actuator_gather (plan B, B4), whose upload
// packs it, so the two spread by one rule (TD-9).
// The density is formed here once and carried: the GPU's record holds
// `count` in its header, so both sides spread over the same points.
struct DiscSpread {
    std::array<glm::vec3, kDiscPoints> points{};   // grid frame, m: `rings` rings of `angles`; past `count`, 0
    std::array<glm::vec3, kDiscPoints> tangent{};  // each point's swirl direction over its radius; 0 without swirl
    float scale = 0.0f;                            // h thrust / (count rho dx^3)
    float swirl_scale = 0.0f;                      // h torque / (count rho dx^3)
    float reach = 0.0f;                            // radius + 2 dx: no point's weight reaches past it
    bool swirl = false;                            // torque != 0
    uint32_t rings = 0;                            // disc_density(radius, dx)'s
    uint32_t angles = 0;
    uint32_t count = 0;                            // the points spread: rings x angles
};

[[nodiscard]] DiscSpread disc_spread(const ActuatorDisc& disc, const GridShape& shape, float rho, float h) noexcept;

// Adds one step's increments for a checked disc, with subnormals flushed as
// fluid_step() flushes them (fluid_step.hpp).
void spread_actuator(const ActuatorDisc& disc, const GridShape& shape, float rho, float h, std::span<float> du,
                     std::span<float> dv, std::span<float> dw) noexcept;

}  // namespace spade::physics::airflow
