// sensors/baro.cpp -- the barometer's arithmetic (sensors/baro.hpp): the ISA truth, its inverse, and the synthesis pass's work.
#include "sensors/baro.hpp"

#include <glm/geometric.hpp>

#include "core/fp32_math.hpp"
#include "physics/integrator.hpp"  // body_flags::active

namespace spade::sensors {

namespace {

// The exponent of the pressure law, g M / (R L): 5.25588 at the ISA's 0.0065 K/m.
[[nodiscard]] float isa_exponent(float lapse_k_per_m) noexcept { return kIsaGMOverR / lapse_k_per_m; }

}  // namespace

BaroTruth baro_truth(float height_m, float sea_level_pressure_pa, float sea_level_temperature_k,
                     float lapse_k_per_m) noexcept {
    float temperature = sea_level_temperature_k - lapse_k_per_m * height_m;
    if (!(temperature >= 1.0f)) temperature = 1.0f;  // far above any troposphere: keep the logarithm defined
    const float ratio = temperature / sea_level_temperature_k;
    const float pressure = sea_level_pressure_pa * math::exp32(isa_exponent(lapse_k_per_m) * math::log32(ratio));
    return {temperature, pressure};
}

float baro_pressure_altitude(float pressure_pa, float sea_level_pressure_pa, float sea_level_temperature_k,
                             float lapse_k_per_m) noexcept {
    const float p = pressure_pa >= 1.0f ? pressure_pa : 1.0f;
    const float ratio = math::exp32(math::log32(p / sea_level_pressure_pa) / isa_exponent(lapse_k_per_m));
    return sea_level_temperature_k * (1.0f - ratio) / lapse_k_per_m;
}

void synthesize_baro(std::span<const BodyState> bodies, std::span<BaroSensorRow> sensors, std::span<BaroSample> rings,
                     uint64_t tick) noexcept {
    for (std::size_t s = 0; s < sensors.size(); ++s) {
        BaroSensorRow& row = sensors[s];
        if (row.kind != sensor_kind::baro) continue;  // free, or its init still queued

        const BodyState& body = bodies[row.body_slot];
        if ((body.flags & physics::body_flags::active) == 0u) continue;

        // The rate boundary, spelled as the IMU's and the GNSS's: a divider of 0 degrades to every substep.
        row.phase += 1u;
        if (row.phase < row.rate_divider) continue;
        row.phase = 0u;

        // THE DRAWS: three, in this order, unconditionally (three named locals: argument order is unspecified).
        const float n_bias = row.noise.next_gauss();
        const float n_p = row.noise.next_gauss();
        const float n_t = row.noise.next_gauss();

        // The Gauss-Markov advance on the sensor's own clock: two multiplies and an add, no exp (the coefficients are row constants).
        row.bias = row.bias * row.bias_retention + row.bias_drive * n_bias;

        const glm::vec3 position = body.pos + body.orient * row.mount_pos;
        const float height = row.world_origin_altitude_m + position.y;
        const BaroTruth truth =
            baro_truth(height, row.sea_level_pressure_pa, row.sea_level_temperature_k, row.lapse_k_per_m);

        // Left to right: the truth, the bias, the temperature term, the white noise.
        BaroSample sample{};
        sample.pressure_pa = truth.pressure_pa + row.bias +
                             row.temp_coeff_pa_per_k * (truth.temperature_k - row.sea_level_temperature_k) +
                             row.sigma_p_pa * n_p;
        sample.temperature_c = (truth.temperature_k - kKelvinAtZeroCelsius) + row.sigma_t_k * n_t;
        sample.pressure_alt_m = baro_pressure_altitude(sample.pressure_pa, row.sea_level_pressure_pa,
                                                       row.sea_level_temperature_k, row.lapse_k_per_m);
        row.last_index += 1u;
        sample.index = row.last_index;
        sample.tick = tick;
        ring_write(rings.subspan(s * kRingDepth, kRingDepth), sample.index, sample);
    }
}

}  // namespace spade::sensors
