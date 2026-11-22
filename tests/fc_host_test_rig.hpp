#pragma once

// fc_host_test_rig.hpp -- the world, the vehicle and the sensors the fc_host tests share: a still world (no gravity), a quadrotor with an IMU, a
// barometer and a magnetometer, dt 500 us with one substep by default.

#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "fc_host/lockstep.hpp"
#include "sensors/baro.hpp"
#include "sensors/imu.hpp"
#include "sensors/mag.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"

namespace fc_host_test {


using spade::BodyRef;
using spade::Capacities;
using spade::Environment;
using spade::RowRef;
using spade::Simulation;
using spade::TurbulenceLevel;
using spade::VehicleRef;
using spade::VehicleSpawn;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::fc_host::HostConfig;
using spade::fc_host::LockstepHost;
using spade::fc_host::Sample;
using spade::fc_host::SensorBinding;
using spade::fc_host::SensorKind;
using spade::vehicles::QuadrotorParams;
using spade::vehicles::RotorParams;

#ifndef ASSERT_OK
#define ASSERT_OK(expr)                                                                         \
    do {                                                                                        \
        const auto& r_ = (expr);                                                                \
        ASSERT_TRUE(r_.has_value()) << #expr << " failed: " << (r_ ? "" : r_.error().context); \
    } while (false)
#endif

constexpr uint64_t kDt = 500'000;  // 500 us, one substep: the phase 1 default (4 ms is 8 ticks, 2.5 ms is 5)

[[nodiscard]] inline spade::modules::ModuleSet modules_with_baro_mag() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::baro_module());
    set.push_back(spade::modules::mag_module());
    return set;
}

// No gravity: a commanded rotor shows as acceleration and nothing else.
[[nodiscard]] inline spade::Result<WorldSetDesc> still_world() {
    Environment env;
    env.gravity = glm::vec3(0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    Capacities caps;
    caps.bodies = 4;
    caps.force_elements = 8;
    caps.sensors = 4;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> world = WorldBuilder().name("still").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0x10C57E9ULL;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.1f;
    instance.grid.cell_size = 0.5f;
    return WorldSetDesc{{instance}};
}

struct Rig {
    Simulation sim;
    VehicleRef vehicle;
    RowRef baro;
    RowRef mag;
};

// A quadrotor with an IMU at `imu_divider`, a barometer at `baro_divider`, a magnetometer at `mag_divider`; dt 500 us, one substep.
[[nodiscard]] inline std::optional<Rig> make_rig(uint32_t imu_divider, uint32_t baro_divider = 40, uint32_t mag_divider = 20, uint64_t dt_ns = kDt,
                                          uint32_t substeps = 1) {
    const spade::Result<WorldSetDesc> set = still_world();
    if (!set) return std::nullopt;
    spade::Result<Simulation> sim = Simulation::create(*set, dt_ns, substeps, {}, modules_with_baro_mag());
    if (!sim) return std::nullopt;

    QuadrotorParams params;
    params.name = "host_quad";
    params.param_schema_id = 7;
    params.visual_ref = "meshes/host_quad";
    params.mass = 1.0f;
    params.inertia_diag = glm::vec3(0.012f, 0.021f, 0.016f);
    params.arm_length = 0.18f;
    params.rotor_height = 0.02f;
    params.proxy_radius = 0.18f;
    for (std::size_t i = 0; i < spade::vehicles::kQuadrotorRotorCount; ++i) {
        RotorParams& rotor = params.rotors[i];
        rotor.tau = 0.0f;
        rotor.radius = 0.12f;
        rotor.thrust_coeff = 1.0e-5f;
        rotor.torque_coeff = 1.6e-7f;
    }
    params.drag.mode = spade::physics::drag_mode::quadratic;
    params.drag.area = 0.02f;
    params.drag.coeffs = glm::vec3(0.0f);
    params.imu.rate_divider = imu_divider;
    const spade::Result<spade::vehicles::ModelType> model = spade::vehicles::make_quadrotor(params);
    if (!model) return std::nullopt;
    const spade::Result<spade::ModelTypeId> id = sim->register_model(*model);
    if (!id) return std::nullopt;
    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where.rotor_omega = 0.0f;
    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    if (!vehicle) return std::nullopt;

    spade::sensors::BaroSensorSpawn baro;
    baro.rate_divider = baro_divider;
    const spade::Result<RowRef> baro_row = sim->attach_row(vehicle->body, "baro_sensors", baro);
    spade::sensors::MagSensorSpawn mag;
    mag.rate_divider = mag_divider;
    mag.field_world_ut = glm::vec3(20.0f, -40.0f, 5.0f);
    const spade::Result<RowRef> mag_row = sim->attach_row(vehicle->body, "mag_sensors", mag);
    if (!baro_row || !mag_row) return std::nullopt;
    if (!sim->flush_structural()) return std::nullopt;
    return Rig{std::move(*sim), *vehicle, *baro_row, *mag_row};
}

[[nodiscard]] inline std::vector<SensorBinding> all_bindings(const Rig& rig) {
    return {SensorBinding::of(rig.vehicle.imu_sensors[0]), SensorBinding::baro(rig.baro), SensorBinding::mag(rig.mag)};
}

[[nodiscard]] inline std::size_t count_of(const std::vector<Sample>& samples, SensorKind kind) {
    std::size_t n = 0;
    for (const Sample& s : samples) n += s.kind == kind ? 1u : 0u;
    return n;
}


}  // namespace fc_host_test
