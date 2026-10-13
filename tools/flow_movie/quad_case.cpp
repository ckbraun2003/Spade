// tools/flow_movie/quad_case.cpp -- the quad-wake case (owner: Physics).
//
// THE ROTORS are the COMPILED reference quad: compile_airframe(reference_quad())
// of tests/test_airframe_compile.cpp, a 0.526 kg plus-layout quad (Y up, nose
// +X), printed at master d98da3d. ModelType::rotors[k].local_pos (from the COM),
// radius and spin_dir; the hover fit (AirframeFit): hover_thrust = weight / 4 =
// 1.28957453 N per rotor at hover_omega = 801.26 rad/s, k_T = 2.00861849e-6 and
// k_Q = 2.04075653e-8, so the hover torque Q = k_Q omega^2 = T k_Q / k_T =
// 0.0131021 N m. Adjacent hubs are 0.1556 m apart against 2R = 0.1524 m.
//
// THE SWIRL: each disc pushes the air along -Y (its axis); the air takes the
// torque -spin Q about that axis (actuator.hpp's rule): the air turns with
// the blades, a rotor of spin s turning about +Y.
//
// THE GRID is plan A's validated C8 box (64x128x64 at dx 1 cm, open faces,
// still air, inviscid, n_v 2) widened to 80 so four discs keep 19 cm to the
// side faces. h = 0.5 ms keeps CFL near 0.6: momentum theory gives v_h =
// sqrt(T / (2 rho pi R^2)) = 5.37 m/s, about 10.7 m/s in the far wake, plus up
// to about 4 m/s of swirl at the inner ring. t_end = 0.6 s: the starting
// vortex leaves the bottom face in about 0.25 s and the column is close to
// steady by 0.5 s.
#include "cases.hpp"

#include <array>
#include <string>

#include "physics/airflow/actuator.hpp"

namespace flow_movie {

spade::Result<AirflowCase> quad_hover_case() {
    constexpr float kThrust = 1.28957453f;  // N per rotor, the hover fit
    constexpr float kTorque = 0.0131021f;   // N m per rotor, T k_Q / k_T
    constexpr float kRadius = 0.0762f;      // m, the compiled rotors' radius
    struct Rotor {
        glm::vec3 local_pos;  // from the COM, body = world (level hover), m
        int spin;
    };
    const std::array<Rotor, 4> rotors{{
        {glm::vec3(0.11f, 0.0241444856f, 0.0f), +1},
        {glm::vec3(0.0f, 0.0241444856f, 0.11f), -1},
        {glm::vec3(-0.11f, 0.0241444856f, 0.0f), +1},
        {glm::vec3(0.0f, 0.0241444856f, -0.11f), -1},
    }};
    // The hub plane at y = 0.96 m of the grid (C8's disc height), centred across.
    const glm::vec3 com(0.40f, 0.96f - 0.0241444856f, 0.40f);

    AirflowCase c;
    c.name = "Quadrotor in hover: four swirling actuator discs";
    c.settings = "reference quad 0.526 kg: R 0.0762 m, T 1.290 N and Q 0.0131 N m per rotor at 801 rad/s, "
                 "spins + - + - (+x, +z, -x, -z); 80x128x80 at dx 1 cm, h 0.5 ms, inviscid, still air";
    c.shape = airflow::GridShape{80, 128, 80, 0.01f};
    c.bc = airflow::DomainBc{};  // every face open, ambient 0: still air
    c.fluid = airflow::FluidConfig{.rho = 1.225f, .nu = 0.0f, .h = 0.0005f, .n_v = 2};
    c.origin = -com;  // the COM at the world origin
    for (const Rotor& r : rotors) {
        const airflow::ActuatorDisc d{.centre = com + r.local_pos,
                                      .axis = glm::vec3(0.0f, -1.0f, 0.0f),
                                      .radius = kRadius,
                                      .thrust = kThrust,
                                      .torque = -static_cast<float>(r.spin) * kTorque};
        if (spade::Result<void> ok = airflow::check_disc(d, c.shape); !ok) return std::unexpected(ok.error());
        c.discs.push_back(d);
        c.spins.push_back(r.spin);
    }
    c.t_end = 0.6f;
    return c;
}

}  // namespace flow_movie
