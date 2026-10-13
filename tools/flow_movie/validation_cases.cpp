// The validation reel's cases, each set up as tests/test_airflow_validation.cpp
// sets it up for its V1 record (tests/data/airflow/records/), so the reel
// films the run its record measured.

#include <cmath>
#include <cstddef>
#include <numbers>

#include "cases.hpp"

namespace flow_movie {

AirflowCase cavity_re400_case() {
    constexpr uint32_t n = 128;
    AirflowCase c;
    c.name = "C1 lid-driven cavity, Re 400";
    c.settings = "lid speed 1, rho 1, nu 1/400; 128 x 128 quasi-2D slab (z periodic), dx 1/128";
    c.bc.face = {airflow::FaceBc::wall, airflow::FaceBc::wall, airflow::FaceBc::wall,
                 airflow::FaceBc::wall, airflow::FaceBc::periodic, airflow::FaceBc::periodic};
    c.bc.wall_velocity[airflow::kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    c.shape = {n, n, 1, 1.0f / static_cast<float>(n)};
    c.fluid = {.rho = 1.0f, .nu = 1.0f / 400.0f, .h = 0.5f / static_cast<float>(n), .n_v = 2, .visc_sweeps = 8};
    c.t_end = 60.0f;
    return c;
}

AirflowCase taylor_green_case() {
    constexpr uint32_t n = 64;
    AirflowCase c;
    c.name = "C0 Taylor-Green vortex";
    c.settings = "u = sin x cos y cos z, v = -cos x sin y cos z; inviscid; 64^3 periodic 2 pi box";
    c.bc.face.fill(airflow::FaceBc::periodic);
    c.shape = {n, n, n, static_cast<float>(2.0 * std::numbers::pi / n)};
    c.fluid = {.rho = 1.0f, .nu = 0.0f, .h = 0.5f * c.shape.dx, .n_v = 2};
    c.t_end = 10.0f;
    c.init = [](airflow::MacGrid& g) {
        airflow::for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) {
            g.u[idx] = static_cast<float>(std::sin(double(x.x)) * std::cos(double(x.y)) * std::cos(double(x.z)));
        });
        airflow::for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) {
            g.v[idx] = static_cast<float>(-std::cos(double(x.x)) * std::sin(double(x.y)) * std::cos(double(x.z)));
        });
    };
    return c;
}

AirflowCase disc_wake_case() {
    AirflowCase c;
    const float rho = 1.225f, vh = 4.0f, radius = 0.1f;
    const float area = static_cast<float>(std::numbers::pi) * radius * radius;
    c.name = "C8 actuator-disc far wake";
    c.settings = "R 0.1 m, T 1.2315 N (v_h 4 m/s), still air, inviscid; 64 x 128 x 64, dx 1 cm";
    c.shape = {64, 128, 64, 0.01f};  // c.bc: every face open
    c.fluid = {.rho = rho, .nu = 0.0f, .h = c.shape.dx / 8.0f, .n_v = 2};
    c.discs = {airflow::ActuatorDisc{.centre = {0.32f, 0.96f, 0.32f}, .axis = {0.0f, -1.0f, 0.0f},
                                     .radius = radius, .thrust = 2.0f * rho * area * vh * vh}};
    c.spins = {0};
    c.t_end = 384.0f * c.fluid.h;  // the record's 384 steps
    return c;
}

AirflowCase smoke_case() {
    AirflowCase c;
    c.name = "smoke: one actuator disc";
    c.settings = "R 0.2 m, T 1.2 N; 24^3 open box, dx 5 cm (the step-6 feed test's)";
    c.shape = {24, 24, 24, 0.05f};
    c.fluid = {.rho = 1.225f, .nu = 1.5e-5f, .h = 0.005f};
    c.discs = {airflow::ActuatorDisc{.centre = {0.6f, 0.9f, 0.6f}, .axis = {0.0f, -1.0f, 0.0f},
                                     .radius = 0.2f, .thrust = 1.2f}};
    c.spins = {0};
    c.t_end = 60.0f * c.fluid.h;
    return c;
}

}  // namespace flow_movie
