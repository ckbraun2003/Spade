// sim/builtin_state.hpp -- the built-in modules' attached rows (module-API
// stage 4): how drag, imu, gnss and rotor write a row from its spawn record
// at the step boundary, and check a record when it is offered.
//
// sim/standard_modules.cpp wires these into the declarations (ArrayDecl's
// init and validate), and Simulation runs them only through the declarations:
// attach_row() calls `validate`, and the structural queue's init_row calls
// `init`. They live in sim/, above the row types (physics/forces.hpp,
// sensors/, vehicles/rotor.hpp) and beside the spawn records
// (sim/simulation.hpp), so no lower layer sees a module type.
//
// Every validate message carries no call prefix: the caller adds
// "add_imu_sensor: " or "attach_row: ".
//
// The three seeded streams (dryden, sensor.imu, sensor.gnss) are declared with
// their reseed functions, below. create() and reseed() reach them only through
// the declarations (ModuleDesc::streams), and an init that derives a stream
// calls its row's reseed function to do it: ONE derivation per row type, so
// the init and the reseed cannot drift apart (TD-9).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "core/error.hpp"
#include "sim/module.hpp"
#include "vehicles/model_type.hpp"

namespace spade::modules::builtin {

// A rotor's spawn record. Rotors arrive only with a vehicle: spawn(world,
// model, where) queues one per rotor the model declares, holding the shaft
// speed the vehicle spawns at (VehicleSpawn::rotor_omega).
struct RotorSpawn {
    vehicles::RotorDesc desc{};
    float omega = 0.0f;  // rad/s; the row's omega AND omega_cmd
};

// drag_bodies, from a DragElementSpawn.
void init_drag_row(const RowInit& in) noexcept;
[[nodiscard]] Result<void> validate_drag_spawn(std::span<const std::byte> spawn);

// dryden, one row per world, which takes no init: its whole row is the
// stream's. dryden_init() derives the stream under "dryden" at index 0 and
// places the filter on its stationary distribution (world/medium.hpp), so the
// world starts gusty -- at create() and again at every reseed().
void reseed_dryden_row(std::span<std::byte> row, const WorldParams& params, uint32_t local_slot) noexcept;

// imu_sensors, from an ImuSensorSpawn. reseed_imu_row() writes only `noise`
// (sensors::imu_noise_stream), and init_imu_row() calls it.
void init_imu_row(const RowInit& in) noexcept;
[[nodiscard]] Result<void> validate_imu_spawn(std::span<const std::byte> spawn);
void reseed_imu_row(std::span<std::byte> row, const WorldParams& params, uint32_t local_slot) noexcept;

// gnss_sensors, from a GnssSensorSpawn. reseed_gnss_row() writes only `noise`
// (sensors::gnss_noise_stream), and init_gnss_row() calls it.
void init_gnss_row(const RowInit& in) noexcept;
[[nodiscard]] Result<void> validate_gnss_spawn(std::span<const std::byte> spawn);
void reseed_gnss_row(std::span<std::byte> row, const WorldParams& params, uint32_t local_slot) noexcept;

// rotors, from a RotorSpawn. refuse_direct_rotor() refuses every record:
// attach_row() cannot add a rotor to a body (a rotor is part of a vehicle).
void init_rotor_row(const RowInit& in) noexcept;
[[nodiscard]] Result<void> refuse_direct_rotor(std::span<const std::byte> spawn);

}  // namespace spade::modules::builtin
