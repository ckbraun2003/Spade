// sensors/mag.cpp -- RED stub: the magnetometer's arithmetic (filled in by the GREEN commit).
#include "sensors/mag.hpp"

namespace spade::sensors {

glm::vec3 mag_field_world(float, float, float, MagWorldFrame) noexcept { return glm::vec3(0.0f); }
glm::vec3 mag_truth(const glm::quat&, const glm::quat&, const glm::vec3&) noexcept { return glm::vec3(0.0f); }
void synthesize_mag(std::span<const BodyState>, std::span<MagSensorRow>, std::span<MagSample>, uint64_t) noexcept {}

}  // namespace spade::sensors
