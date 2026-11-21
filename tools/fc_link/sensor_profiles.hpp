#pragma once

#include <cstdint>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "sensors/baro.hpp"
#include "sensors/mag.hpp"
#include "sim/simulation.hpp"

// ===========================================================================
// fc_link/sensor_profiles.hpp -- THE RED STUB (O140 phase 1: docs/design/physics/plans/2026-11-21-fc-physics.md section 1.4). Returns default spawn records
// so the tests fail on values. The real header replaces this.
// ===========================================================================

namespace spade::fc_link {

enum class SensorProfile { reference, kit };

struct SensorSet {
    ImuSensorSpawn imu;
    sensors::BaroSensorSpawn baro;
    sensors::MagSensorSpawn mag;
    GnssSensorSpawn gnss;
};

[[nodiscard]] inline Result<SensorSet> sensor_set(SensorProfile, uint64_t, const glm::vec3&, float = 0.0f) { return SensorSet{}; }

}  // namespace spade::fc_link
