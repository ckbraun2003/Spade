// sensors/mag.cpp -- the magnetometer's arithmetic (sensors/mag.hpp): the field helper, the truth rotation, and the synthesis pass's work.
#include "sensors/mag.hpp"

#include <glm/geometric.hpp>

#include "core/fp32_math.hpp"
#include "physics/integrator.hpp"  // body_flags::active

namespace spade::sensors {

namespace {

// Three standard normals as a vector in a pinned order (x, then y, then z): a function, not an initialiser list, because
// argument evaluation order is unspecified (gnss.cpp's note).
[[nodiscard]] glm::vec3 draw_gauss3(rng::Stream& stream) noexcept {
    const float x = stream.next_gauss();
    const float y = stream.next_gauss();
    const float z = stream.next_gauss();
    return glm::vec3(x, y, z);
}

}  // namespace

glm::vec3 mag_field_world(float declination_rad, float inclination_rad, float strength_ut, MagWorldFrame frame) noexcept {
    const float horizontal = strength_ut * math::cos32(inclination_rad);
    const float north = horizontal * math::cos32(declination_rad);
    const float east = horizontal * math::sin32(declination_rad);
    const float down = strength_ut * math::sin32(inclination_rad);
    if (frame == MagWorldFrame::anchored_world) return glm::vec3(east, -down, -north);  // x east, y up, z south
    return glm::vec3(north, -down, east);                                                // x north, y up, z east
}

glm::vec3 mag_truth(const glm::quat& body_orient, const glm::quat& mount_orient, const glm::vec3& field_world_ut) noexcept {
    return glm::conjugate(body_orient * mount_orient) * field_world_ut;
}

void synthesize_mag(std::span<const BodyState> bodies, std::span<MagSensorRow> sensors, std::span<MagSample> rings,
                    uint64_t tick) noexcept {
    for (std::size_t s = 0; s < sensors.size(); ++s) {
        MagSensorRow& row = sensors[s];
        if (row.kind != sensor_kind::mag) continue;

        const BodyState& body = bodies[row.body_slot];
        if ((body.flags & physics::body_flags::active) == 0u) continue;

        row.phase += 1u;
        if (row.phase < row.rate_divider) continue;
        row.phase = 0u;

        // SIX DRAWS, in this order, unconditionally: the bias step (x, y, z), then the white (x, y, z).
        const glm::vec3 bias_step = draw_gauss3(row.noise);
        const glm::vec3 white = draw_gauss3(row.noise);

        row.bias = row.bias * row.bias_retention + row.bias_drive * bias_step;

        const glm::vec3 field = mag_truth(body.orient, row.mount_orient, row.field_world_ut);
        const glm::mat3 soft(row.soft_iron_c0, row.soft_iron_c1, row.soft_iron_c2);

        // Left to right: the soft-iron product, the hard iron, the bias, the white noise.
        MagSample sample{};
        sample.field_body_ut = soft * field + row.hard_iron_ut + row.bias + row.sigma_white_ut * white;
        row.last_index += 1u;
        sample.index = row.last_index;
        sample.tick = tick;
        ring_write(rings.subspan(s * kRingDepth, kRingDepth), sample.index, sample);
    }
}

}  // namespace spade::sensors
