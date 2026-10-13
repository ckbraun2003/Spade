#pragma once

// The recorder's airflow cases (tools/flow_movie): each one a solver set-up
// in the solver's own terms, and the simulated time to film from rest.
// Physics owns the quad case (quad_case.cpp); Rendering owns the validation
// reel's cases (validation_cases.cpp), each run as its V1 record ran it.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace flow_movie {

namespace airflow = spade::physics::airflow;

struct AirflowCase {
    std::string name;      // the caption's first line
    std::string settings;  // the caption's second line: the rotor or flow settings
    airflow::GridShape shape;
    airflow::DomainBc bc;
    airflow::FluidConfig fluid;  // fluid.h is the step
    glm::vec3 origin{0.0f};      // the grid's minimum corner in the world, m
    std::vector<airflow::ActuatorDisc> discs;  // grid frame; torque = -spin Q (actuator.hpp's rule)
    std::vector<int> spins;                    // +1 or -1 per disc, for the captions
    float t_end = 0.0f;                        // simulated time filmed, from rest
    std::function<void(airflow::MacGrid&)> init;  // the initial field; empty: at rest
};

// Physics' (quad_case.cpp): the reference quad in hover, four swirling
// actuator discs in an open box.
[[nodiscard]] spade::Result<AirflowCase> quad_hover_case();

// The validation reel's cases (validation_cases.cpp), as
// tests/test_airflow_validation.cpp runs them for the V1 records.
[[nodiscard]] AirflowCase cavity_re400_case();  // C1: 128 x 128 slab, Re 400, T 60
[[nodiscard]] AirflowCase taylor_green_case();  // C0: 64^3 periodic, inviscid, t 10
[[nodiscard]] AirflowCase disc_wake_case();     // C8: 64 x 128 x 64 at 1 cm, 384 steps

// A small open box with one disc, for the smoke test.
[[nodiscard]] AirflowCase smoke_case();

}  // namespace flow_movie
