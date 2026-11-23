#pragma once

// ---------------------------------------------------------------------------
// fc_flight_world.hpp -- the world the flight film's autopilot flies (docs/design/interface/plans/2026-11-21-fc-links.md section 13): Physics' X500 kit
// model (tools/flow_movie/data/x500/x500.model.yaml, the one source every X500 test reads) standing on its own feet (tests/x500_contact_hull.hpp, the
// multi-point contact of test_x500_ground_contact.cpp) on a ground plane under gravity, with the sensors the link sends bound in the order the
// exchange expects: IMU 0, barometer 1, magnetometer 2, GNSS 3. dt 500 us, one substep; the IMU at 4 ms (a divider of 8), the others at the seam
// test's rates. Nothing in it is PX4's: the autopilot only sees the sensors.
// ---------------------------------------------------------------------------

#include <cmath>
#include <filesystem>
#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "compute/backend.hpp"
#include "fc_host/lockstep.hpp"
#include "fc_host_test_rig.hpp"
#include "physics/rigid_contact.hpp"
#include "scene/model_file.hpp"
#include "sensors/baro.hpp"
#include "sensors/gnss.hpp"
#include "sensors/mag.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "x500_contact_hull.hpp"

namespace fc_flight_test {

struct FlightWorld {
    spade::Simulation sim;
    spade::VehicleRef vehicle{};
    spade::scene::ModelFile model;                          // the X500's file: the propulsion chain the link converts duties through
    std::vector<spade::fc_host::SensorBinding> bindings;    // IMU 0, barometer 1, magnetometer 2, GNSS 3
    double rest_com_y = 0.0;                                // the centre of mass' height above the ground when resting on the feet, m

    // The centre of mass' height above where it rests on the ground: 0 sitting on the feet, 2.0 two metres up.
    [[nodiscard]] double altitude_m() const {
        const auto b = sim.body(vehicle.body);
        return b ? static_cast<double>((*b)->pos.y) - rest_com_y : 0.0;
    }
};

// dt 500 us (the autopilot link's tick); the IMU every 8 ticks (4 ms), the barometer every 40 (20 ms), the magnetometer every 20 (10 ms), the GNSS every
// 400 (200 ms). The sensors' noise is the engine's default for each (the IMU's is the model file's); the film's claims do not rest on it.
inline constexpr uint64_t kDtNs = 500'000;
inline constexpr uint32_t kImuDivider = 8, kBaroDivider = 40, kMagDivider = 20, kGnssDivider = 400;
// The centre of mass over the plane at rest on the feet: the design frame's centre of mass height minus the pads' lowest point (test_x500_ground_contact.cpp).
inline constexpr double kRestComY = -0.0254301 - (-0.2153);
inline constexpr double kRestDesignOriginY = 0.2153;
// The ground plane (world y = 0) is this high above mean sea level, ISA datum: the barometer and the GNSS (the link's geo frame) must both say so.
inline constexpr double kGroundAmslM = 488.0;
// Where on Earth the world's origin is (PX4 SITL's default home, Zurich): the autopilot's geodetic home. These and kGroundAmslM are the ONLY place the
// run's code-made geography lives, so the run record's world_file_hash (this header) covers them.
inline constexpr double kHomeLat = 47.397742, kHomeLon = 8.545594;
// Where each sensor sits in FlightWorld::bindings, for AutopilotLink's ExchangeConfig (the IMU is 0). RED: not set yet.
inline constexpr uint32_t kBaroBinding = 0, kMagBinding = 0, kGnssBinding = 0;

[[nodiscard]] inline std::optional<FlightWorld> make_flight_world() {
    namespace x5 = spade::test::x500;
    auto loaded = spade::scene::load_model_file(std::filesystem::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/x500.model.yaml");
    if (!loaded || loaded->model.imu_mounts.empty()) return std::nullopt;
    spade::scene::ModelFile file = std::move(*loaded);
    spade::vehicles::ModelType model = x5::with_feet(file.model);
    model.imu_mounts[0].rate_divider = kImuDivider;

    spade::Environment env;
    env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    spade::WorldBuilder b;
    b.name("flight").environment(env).capacities(spade::Capacities{4, 16, 8, 1});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    const auto built = b.build();
    if (!built) return std::nullopt;
    spade::WorldInstanceDesc inst;
    inst.world = *built;
    inst.seed = 0xF11647ULL;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    inst.contacts.restitution_e = 0.0f;
    inst.contacts.friction_mu = 0.0f;
    inst.contacts.proxy_radius = 0.0f;

    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::rigid_contact_module());
    set.push_back(spade::modules::baro_module());
    set.push_back(spade::modules::mag_module());
    auto sim = spade::Simulation::create(spade::WorldSetDesc{{inst}}, kDtNs, 1, spade::compute::BackendDesc{.kind = spade::compute::BackendKind::cpu}, set);
    if (!sim) return std::nullopt;
    const auto id = sim->register_model(model);
    if (!id) return std::nullopt;
    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, static_cast<float>(kRestDesignOriginY) + 0.0005f, 0.0f);  // the design frame's origin, a half millimetre above rest
    where.rotor_omega = 0.0f;
    const auto vehicle = sim->spawn(0, *id, where);
    if (!vehicle) return std::nullopt;

    spade::sensors::BaroSensorSpawn baro;
    baro.rate_divider = kBaroDivider;
    baro.world_origin_altitude_m = static_cast<float>(kGroundAmslM);  // the ground is 488 m above the ISA datum, as the GNSS says
    spade::sensors::MagSensorSpawn mag;
    mag.rate_divider = kMagDivider;
    mag.field_world_ut = glm::vec3(20.0f, -40.0f, 5.0f);
    spade::GnssSensorSpawn gnss;
    gnss.rate_divider = kGnssDivider;
    const auto baro_row = sim->attach_row(vehicle->body, "baro_sensors", baro);
    const auto mag_row = sim->attach_row(vehicle->body, "mag_sensors", mag);
    const auto gnss_ref = sim->add_gnss_sensor(vehicle->body, gnss);
    if (!baro_row || !mag_row || !gnss_ref || !sim->flush_structural()) return std::nullopt;

    using spade::fc_host::SensorBinding;
    std::vector<SensorBinding> bindings{SensorBinding::of(vehicle->imu_sensors[0]), SensorBinding::baro(*baro_row), SensorBinding::mag(*mag_row),
                                        SensorBinding::of(*gnss_ref)};
    return FlightWorld{std::move(*sim), *vehicle, std::move(file), std::move(bindings), kRestComY};
}

}  // namespace fc_flight_test
