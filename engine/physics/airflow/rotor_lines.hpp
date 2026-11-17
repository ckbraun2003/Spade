#pragma once

#include <span>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/rotor_discs.hpp"

// ---------------------------------------------------------------------------
// ROTORS TO ACTUATOR LINES (L86i; the one rule, TD-9, as rotor_discs.hpp is for the discs). A rotor's LineRotor in the airflow
// grid's frame, from its hub, thrust axis and spin in the body frame, the body's pose in the world, the grid's place in the world
// (world-aligned, minimum corner at grid_origin_world), the shaft's speed and the blade azimuth:
//
//   centre  com + R_body hub - grid_origin_world                        (the disc's, exactly)
//   axis    +(R_body (orient_body +Y)): the direction the rotor pushes the DRONE; the disc's axis is its negative
//   ref     R_body (orient_body +X): azimuth zero, the part's own +X (the drawn blade 0 at psi 0, the authored pose)
//   spin    spin_dir, +1 or -1 (a line has no "no swirl": the blades turn)
//   omega   the shaft's speed; psi the blade azimuth (physics/blade_azimuth.hpp's P11)
//
// THE DRAWN BLADE AND THE LINE COINCIDE. posed_parts() turns a rotor's parts by R_y(s psi) about the local +Y (blade_azimuth.hpp);
// the line puts blade b at phi = s psi + 2 pi b / B, d = ref cos phi + (axis x ref) sin phi (line.hpp), and axis x ref = -(local +Z)
// for axis +Y and ref +X, so d = R_y(phi) +X: the same direction. A test holds it.
//
// Nothing here reads the air; spread_line() (line.hpp) does, with the rotor this returns. check_line() (the kernel's reach inside
// the grid, n_sub, rho, h) is the step's, not this rule's.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct LineRotorSpec {
    glm::vec3 hub_body{0.0f};                       // m, body frame, from the COM
    glm::quat orient_body{1.0f, 0.0f, 0.0f, 0.0f};  // local -> body; the thrust axis is orient_body * +Y, azimuth zero * +X
    float spin_dir = 1.0f;                          // +1 or -1 about the thrust axis
    float omega = 0.0f;                             // rad/s, >= 0
    float psi = 0.0f;                               // rad, the blade angle
    const BladeShape* blade = nullptr;
    const PolarTable* polar = nullptr;
};

// One rotor's line. Refused (invalid_argument), naming the field: a body pose or a rotor field not finite; an orientation, the
// body's or the rotor's, whose squared length is not within 1e-5 of 1; a spin other than -1 or +1; omega negative; no blade or no
// polar.
[[nodiscard]] Result<LineRotor> line_from_rotor(const LineRotorSpec& rotor, const DiscBody& body, glm::vec3 grid_origin_world,
                                                const GridShape& grid);

}  // namespace spade::physics::airflow
