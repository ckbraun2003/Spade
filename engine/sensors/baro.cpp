// sensors/baro.cpp -- RED stub: the barometer's arithmetic (filled in by the GREEN commit).
#include "sensors/baro.hpp"

namespace spade::sensors {

BaroTruth baro_truth(float, float, float, float) noexcept { return {}; }
float baro_pressure_altitude(float, float, float, float) noexcept { return 0.0f; }
void synthesize_baro(std::span<const BodyState>, std::span<BaroSensorRow>, std::span<BaroSample>, uint64_t) noexcept {}

}  // namespace spade::sensors
