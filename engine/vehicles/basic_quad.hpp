// ---------------------------------------------------------------------------
// vehicles/basic_quad.hpp -- the basic parametric quad (Task L3 of docs/
// design/core/plans/2026-10-21-default-drone-library-plan.md).
//
// One function turns a handful of numbers into an X-layout quad's
// AirframeSpec: a centre box, four arm tubes, a hub at each arm's end, and
// the motor, propeller, ESC and battery blocks. compile_airframe() makes the
// model of it, unchanged (vehicles/airframe_compile.hpp). The library's
// basic_quad_6inch is these defaults, and basic_quad_competition is Kat's
// competition-700 fit (Task L5).
//
// THE FRAME is the airframe's design frame: Y up and nose +X, so the
// layout's lateral axis is +Z, to the right. Hub k sits at
// (x_k, hub_height, z_k), in the order front-right (+x, +z), front-left
// (+x, -z), rear-left (-x, -z) and rear-right (-x, +z), with
// x = half_span_x and z = half_span_z. Spins are +1, -1, +1, -1, so the
// diagonals turn alike. Every rotor is a tractor, thrust along +Y.
//
// THE PARTS, in record order:
//   body      a box of body_size and body_mass, centred at the origin
//   arm[k]    a tube from the origin to hub k's (x, z) in the y = 0 plane:
//             radius arm_radius, length L = sqrt(x^2 + z^2), mass
//             arm_mass_per_metre * L, centred at its midpoint and turned from
//             +Y onto the arm with sqrt only (TD-3)
//   remainder only with a fit target (THE FIT, below)
// The blocks follow as the compile gathers them: a motor beside each hub, a
// propeller on it, the ESC and the battery at their own mounts. One IMU sits
// at the origin. proxy_radius is L plus the propeller's radius. Drag is left
// empty, so the compile estimates it from the parts (best-effort; there is no
// mesh record).
//
// THE FIT. Each target is optional, and none is set by default.
//   mass     The total, met by a `remainder` record of (mass - the parts'
//            mass) at the parts' centre of mass. It is mass only (geometry
//            source none): no model part and no solid.
//   inertia  The total's diagonal about its centre of mass, in design axes
//            (x roll, y yaw, z pitch). The remainder carries the tensor
//            diag(inertia) - the parts' composite inertia. That composite is
//            compile_airframe()'s own, so the rule lives in one place (TD-9).
//            It needs a mass target, because a massless remainder is no body.
//   kappa    m, the torque-to-thrust ratio. It sets cq_static to
//            kappa * ct_static / diameter, and is refused for a propeller with
//            tables, which the compile reads instead of cq_static.
// Met "exactly" means the compiled mass and principal moments round to the
// targets' floats. The remainder is exact to double rounding: a few ulps from
// the order of the compile's sums, and its place at the centre of mass the
// compile reports in float. So only a target within about 1e-15 of a float
// rounding midpoint could round the other way.
// A remainder that is not a physical body is refused by name
// (invalid_argument), and so is a target the parts already exceed:
//   "mass"               a mass target that is not above the parts' mass
//   "inertia.x/.y/.z"    an inertia target below the parts' own about that
//                        axis
//   "inertia"            a remainder tensor the composite's own checks
//                        refuse (one that breaks the triangle inequality)
// Its error names the field, the target and the parts' value.
//
// DETERMINISM. Double arithmetic, + - * / and sqrt only (TD-3), in a fixed
// order. The same params give a bit-identical spec on every conforming
// platform.
// ---------------------------------------------------------------------------
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "vehicles/airframe_compile.hpp"

namespace spade::vehicles {

// Every default is the 6-inch basic quad's, and every one is assumed
// (basic_quad_assumptions()) until Task L5 cites a part for it.
struct BasicQuadParams {
    std::string name = "basic_quad";

    // The layout (THE FRAME). Kat's competition-700 layout, +-0.075 x +-0.10 m.
    double half_span_x = 0.075;  // m, along the nose
    double half_span_z = 0.10;   // m, lateral
    double hub_height = 0.022;   // m: each propeller's plane, above the arms

    // The centre box: frame plates, flight controller and wiring.
    glm::dvec3 body_size{0.09, 0.035, 0.06};  // m, full extents
    double body_mass = 0.12;                  // kg

    // The arm tubes.
    double arm_radius = 0.006;          // m: a 12 mm tube, 3 cells at 4 mm
    double arm_mass_per_metre = 0.15;   // kg/m

    // The blocks. Their defaults stand for a 2306 1700 KV motor, a 6x3.5x3
    // propeller, a 45 A 4-in-1 ESC and a 6S 1300 mAh pack.
    MotorBlock motor = default_motor();
    PropBlock prop = default_prop();
    EscBlock esc = default_esc();
    BatteryBlock battery = default_battery();

    // The fit targets (THE FIT).
    std::optional<double> mass{};             // kg
    std::optional<glm::dvec3> inertia{};      // kg m^2, design axes, about the centre of mass
    std::optional<double> kappa{};            // m

    [[nodiscard]] static MotorBlock default_motor();
    [[nodiscard]] static PropBlock default_prop();
    [[nodiscard]] static EscBlock default_esc();
    [[nodiscard]] static BatteryBlock default_battery();
};

// The spec. invalid_argument, naming the field: a half span not finite and
// above 0; a non-finite fit target; a kappa not above 0, beside a propeller
// with tables or a diameter not above 0; an inertia target without a mass
// target; and the refusals of THE FIT. With a mass target, the parts are
// compiled once to measure them, so the compile's own refusal comes back
// unchanged. Otherwise the compile's rules are left to compile_airframe().
[[nodiscard]] Result<AirframeSpec> basic_quad_airframe(const BasicQuadParams& params);

// One default the basic quad assumes: the field's path in BasicQuadParams
// ("half_span_x", "motor.kv") and why the value is plausible.
struct BasicQuadAssumption {
    std::string_view field;
    std::string_view note;
};

// Every numeric default of BasicQuadParams, in declaration order, so the UI
// can list what a default-built quad assumes. The name and the identity
// strings (stator, protocol) are not numbers and are not listed, and neither
// are the fit targets, which have no default.
[[nodiscard]] std::vector<BasicQuadAssumption> basic_quad_assumptions();

}  // namespace spade::vehicles
