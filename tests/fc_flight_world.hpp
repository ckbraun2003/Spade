#pragma once

// ---------------------------------------------------------------------------
// fc_flight_world.hpp -- the world the flight film's autopilot flies (docs/design/interface/plans/2026-11-21-fc-links.md section 13): Physics' X500 kit
// model (tools/flow_movie/data/x500/x500.model.yaml, the one source every X500 test reads) standing on its own feet (tests/x500_contact_hull.hpp, the
// multi-point contact of test_x500_ground_contact.cpp) on a ground plane under gravity, with the sensors the link sends bound in the order the
// exchange expects: IMU 0, barometer 1, magnetometer 2, GNSS 3. dt 500 us, one substep; the IMU at 4 ms (a divider of 8), the others at the seam
// test's rates. Nothing in it is PX4's: the autopilot only sees the sensors.
// ---------------------------------------------------------------------------

#include <cmath>
#include <optional>
#include <vector>

#include "fc_host/lockstep.hpp"
#include "fc_host_test_rig.hpp"
#include "scene/model_file.hpp"
#include "sim/simulation.hpp"

namespace fc_flight_test {

struct FlightWorld {
    spade::Simulation sim;
    spade::VehicleRef vehicle{};
    spade::scene::ModelFile model;                          // the X500's file: the propulsion chain the link converts duties through
    std::vector<spade::fc_host::SensorBinding> bindings;    // IMU 0, barometer 1, magnetometer 2, GNSS 3
    double rest_com_y = 0.0;                                // the centre of mass' height above the ground when resting on the feet, m

    // The centre of mass' height above where it rests on the ground: 0 sitting on the feet, 2.0 two metres up.
    [[nodiscard]] double altitude_m() const { return 0.0; }  // RED
};

[[nodiscard]] inline std::optional<FlightWorld> make_flight_world() { return std::nullopt; }  // RED: no world yet

}  // namespace fc_flight_test
