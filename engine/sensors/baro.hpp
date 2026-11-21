#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <type_traits>

#include <glm/glm.hpp>

#include "core/rng.hpp"
#include "sensors/kinds.hpp"
#include "sensors/rings.hpp"
#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// sensors/baro.hpp -- a barometer: static pressure, temperature and the pressure altitude, at a rate, with a slowly
// varying error. The fourth sensor kind (sensor_kind::baro), and an OPT-IN MODULE: `baro_module()` (sim/module.hpp) is
// not in standard_modules(), so a set that does not add it keeps every digest and configuration hash it has.
//
// THE MODEL, in the order the pass applies it (Physics owns the constants and any change to a formula; Core owns the
// plumbing; the truth function below is the one place the physics is written):
//   h       = world_origin_altitude_m + (body.pos + body.orient * mount_pos).y        the sensor's height, ISA datum, m
//   T       = T0 - L * h                                                               ISA troposphere, K
//   p_true  = p0 * (T / T0)^(g M / (R L))                                              exp32/log32, no libm
//   p_meas  = p_true + bias + temp_coeff * (T - T0) + sigma_p * n_p                    Pa
//   t_meas  = (T - 273.15) + sigma_t * n_t                                             degrees Celsius
//   alt     = the ISA altitude of p_meas (the inverse of the line above; HIL_SENSOR carries it)
// `bias` is a first-order Gauss-Markov process on the sensor's own clock (one step per EMITTED sample, as the GNSS's);
// its turn-on value is a seeded stationary draw, sigma_bias * n, taken at spawn and again at every reseed, so each
// episode has its own offset. The ISA is the troposphere's (valid to 11 km); the spawn refuses a mount height outside
// -1000..11000 m, and above that the formula simply continues (no stratosphere layer), unvalidated.
//
// UNITS: pascal, kelvin (settings) and degrees Celsius (the reading), metres. FRAME: the world's y is up.
//
// DRAWS: three standard normals per emitted sample, always, in this order -- the bias step, the pressure white, the
// temperature white -- whatever the sigmas are, so the stream position is a function of the emitted count alone.
// One more at spawn and at every reseed: the turn-on bias, taken before any sample.
//
// CPU ONLY. The module grades `reference` on the CPU and `absent` on Vulkan: a Vulkan world that holds it is refused
// at create() (L6). There is no GPU twin until a consumer wants one.
// ---------------------------------------------------------------------------

namespace spade::sensors {

// The stream a barometer draws its noise from: PINNED. Changing it re-seeds every barometer.
inline constexpr std::string_view kBaroNoiseDomainTag = "sensor.baro";

// The ISA constants (ISO 2533 / the U.S. Standard Atmosphere 1976), the defaults of a spawn.
inline constexpr float kIsaSeaLevelPressurePa = 101325.0f;
inline constexpr float kIsaSeaLevelTemperatureK = 288.15f;
inline constexpr float kIsaLapseKPerM = 0.0065f;
inline constexpr float kKelvinAtZeroCelsius = 273.15f;
// g0 * M / R, K/m: the exponent of the pressure law is this over the lapse rate (5.25588 at 0.0065 K/m).
inline constexpr float kIsaGMOverR = 0.034163195f;
// The troposphere, m: a spawn's mount height (origin altitude plus the mount's y) must be inside.
inline constexpr float kBaroMinHeightM = -1000.0f;
inline constexpr float kBaroMaxHeightM = 11000.0f;

// One reading. 32 bytes, every byte named.
struct alignas(kStd430StructAlignment) BaroSample {
    float pressure_pa;     // the measured static pressure, Pa
    float temperature_c;   // the measured temperature, degrees Celsius
    float pressure_alt_m;  // the ISA altitude of the measured pressure, m
    float _p0;             // std430 pad
    SampleIndex index;     // monotonically increasing, per sensor, starts at 1
    uint64_t tick;         // the step this sample was produced in
};

static_assert(std::is_standard_layout_v<BaroSample>);
static_assert(std::is_trivially_copyable_v<BaroSample>);
static_assert(std::is_trivially_destructible_v<BaroSample>);
static_assert(alignof(BaroSample) == 16 && sizeof(BaroSample) == 32);
static_assert(offsetof(BaroSample, pressure_pa) == 0);
static_assert(offsetof(BaroSample, temperature_c) == 4);
static_assert(offsetof(BaroSample, pressure_alt_m) == 8);
static_assert(offsetof(BaroSample, index) == 16);
static_assert(offsetof(BaroSample, tick) == 24);
static_assert(3 * sizeof(float) + sizeof(float) + sizeof(SampleIndex) + sizeof(uint64_t) == sizeof(BaroSample),
              "BaroSample has implicit padding");

// What a caller states about a barometer (the attach_row record of the array "baro_sensors"). A sigma is a per-sample
// standard deviation, not a spectral density; bias_tau_s <= 0 disables the correlation (the bias is then redrawn white).
struct BaroSensorSpawn {
    glm::vec3 mount_pos{0.0f};                                // body-frame offset from the COM, m
    uint32_t rate_divider = 1;                                // emit one sample every N substeps
    float world_origin_altitude_m = 0.0f;                     // the height of world y = 0 above the ISA datum, m
    float sea_level_pressure_pa = kIsaSeaLevelPressurePa;     // p0
    float sea_level_temperature_k = kIsaSeaLevelTemperatureK; // T0
    float lapse_k_per_m = kIsaLapseKPerM;                     // L, above 0
    float temp_coeff_pa_per_k = 0.0f;                         // pressure error per kelvin of (T - T0)
    float sigma_p_pa = 0.0f;                                  // white noise, Pa
    float sigma_t_k = 0.0f;                                   // white noise, K
    float sigma_bias_pa = 0.0f;                               // stationary stddev of the Gauss-Markov bias, Pa
    float bias_tau_s = 0.0f;                                  // its correlation time, s; <= 0 disables
};

// One barometer's row: an element of the per-sensor array "baro_sensors", 112 bytes, std430, every byte named.
struct alignas(kStd430StructAlignment) BaroSensorRow {
    uint32_t body_slot;      // WORLD-LOCAL index into the world's bodies span
    uint32_t kind;           // sensor_kind::; 0 == none == inert (active-high liveness)
    uint32_t rate_divider;   // emit one sample every N substeps
    uint32_t phase;          // substeps elapsed in the current sample period
    glm::vec3 mount_pos;     // body-frame offset from the COM, m
    float world_origin_altitude_m;
    float sea_level_pressure_pa;
    float sea_level_temperature_k;
    float lapse_k_per_m;
    float temp_coeff_pa_per_k;
    float sigma_p_pa;
    float sigma_t_k;
    float sigma_bias_pa;
    float bias_tau_s;
    float bias;              // the Gauss-Markov bias state, Pa
    float bias_retention;    // exp(-fix_dt / tau), derived at spawn from the SAMPLE period; 0 == no correlation
    float bias_drive;        // sigma_bias * sqrt(1 - exp(-2 fix_dt / tau))
    float _p0;               // std430 pad
    rng::Stream noise;       // this barometer's own stream
    SampleIndex last_index;  // index of the newest sample written; 0 == none yet
    uint64_t _reserved0;     // must stay 0
};

static_assert(std::is_standard_layout_v<BaroSensorRow>);
static_assert(std::is_trivially_copyable_v<BaroSensorRow>);
static_assert(std::is_trivially_destructible_v<BaroSensorRow>);
static_assert(alignof(BaroSensorRow) == 16 && sizeof(BaroSensorRow) == 112);
static_assert(offsetof(BaroSensorRow, body_slot) == 0);
static_assert(offsetof(BaroSensorRow, kind) == 4);
static_assert(offsetof(BaroSensorRow, rate_divider) == 8);
static_assert(offsetof(BaroSensorRow, phase) == 12);
static_assert(offsetof(BaroSensorRow, mount_pos) == 16);
static_assert(offsetof(BaroSensorRow, world_origin_altitude_m) == 28);
static_assert(offsetof(BaroSensorRow, sea_level_pressure_pa) == 32);
static_assert(offsetof(BaroSensorRow, sea_level_temperature_k) == 36);
static_assert(offsetof(BaroSensorRow, lapse_k_per_m) == 40);
static_assert(offsetof(BaroSensorRow, temp_coeff_pa_per_k) == 44);
static_assert(offsetof(BaroSensorRow, sigma_p_pa) == 48);
static_assert(offsetof(BaroSensorRow, sigma_t_k) == 52);
static_assert(offsetof(BaroSensorRow, sigma_bias_pa) == 56);
static_assert(offsetof(BaroSensorRow, bias_tau_s) == 60);
static_assert(offsetof(BaroSensorRow, bias) == 64);
static_assert(offsetof(BaroSensorRow, bias_retention) == 68);
static_assert(offsetof(BaroSensorRow, bias_drive) == 72);
static_assert(offsetof(BaroSensorRow, _p0) == 76);
static_assert(offsetof(BaroSensorRow, noise) == 80);
static_assert(offsetof(BaroSensorRow, last_index) == 96);
static_assert(offsetof(BaroSensorRow, _reserved0) == 104);
static_assert(4 * sizeof(uint32_t) + sizeof(glm::vec3) + 13 * sizeof(float) + sizeof(rng::Stream) + sizeof(SampleIndex) +
                      sizeof(uint64_t) ==
                  sizeof(BaroSensorRow),
              "BaroSensorRow has implicit padding");

// The stream one barometer draws from, from the WORLD-LOCAL slot (so a world's randomness does not depend on where it
// sits in the set).
[[nodiscard]] constexpr rng::Stream baro_noise_stream(uint64_t world_seed, uint32_t local_slot) noexcept {
    return rng::make_stream(world_seed, kBaroNoiseDomainTag, local_slot);
}

// The ISA at a height: the true temperature (K) and pressure (Pa) for the given sea-level pressure, sea-level
// temperature and lapse rate. No libm (exp32/log32); total: the temperature is floored at 1 K so the logarithm is
// defined, which matters only far above any troposphere.
struct BaroTruth {
    float temperature_k;
    float pressure_pa;
};
[[nodiscard]] BaroTruth baro_truth(float height_m, float sea_level_pressure_pa, float sea_level_temperature_k,
                                   float lapse_k_per_m) noexcept;

// The ISA altitude (m) of a pressure: the inverse of baro_truth's pressure. A pressure at or below 1 Pa reads as 1 Pa.
[[nodiscard]] float baro_pressure_altitude(float pressure_pa, float sea_level_pressure_pa,
                                           float sea_level_temperature_k, float lapse_k_per_m) noexcept;

// The baro.synthesize pass's work for ONE world's spans (the pass in sim/ calls it). The precondition is the IMU's:
// every row of kind baro has a body_slot that indexes `bodies`. `rings` holds sensors.size() * kRingDepth samples.
void synthesize_baro(std::span<const BodyState> bodies, std::span<BaroSensorRow> sensors, std::span<BaroSample> rings,
                     uint64_t tick) noexcept;

}  // namespace spade::sensors
