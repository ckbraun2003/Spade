#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <type_traits>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/rng.hpp"
#include "sensors/kinds.hpp"
#include "sensors/rings.hpp"
#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// sensors/mag.hpp -- a magnetometer: the magnetic field in the sensor's own axes, at a rate, with hard-iron and
// soft-iron distortion and a slowly varying error. The fifth sensor kind (sensor_kind::mag), and an OPT-IN MODULE:
// `mag_module()` (sim/module.hpp) is not in standard_modules(), so a set that does not add it keeps every digest and
// configuration hash it has.
//
// THE WORLD FRAME OF THE FIELD is the world's own, and a world has one of two conventions (Interface's FrameMap has both):
//   compass_x_north  x NORTH, y UP, z EAST (North is Spade +X, East +Z, Down -Y): the default, a yaw of 90 degrees from the world file's
//                    usual reading, chosen so the autopilot link's NED conversion needs no fix-up;
//   anchored_world   x EAST, y UP, z SOUTH: the world file's reading for a world with an anchor (a geodetic origin).
// The SENSOR is frame-blind: `field_world_ut` is the field in whatever frame the world uses, in microtesla (1 gauss = 100 uT), and the pass only
// rotates it into the sensor's axes. The convention enters in one place, the helper below that builds the vector, so no engine state records it.
//
// THE MODEL, in the order the pass applies it (Physics owns the constants and any change to a formula):
//   B_mount = (body.orient * mount_orient)^-1 * field_world      the world field in the sensor's axes
//   B_meas  = soft_iron * B_mount + hard_iron + bias + sigma_white * n
// `bias` is a first-order Gauss-Markov process per axis on the sensor's own clock (one step per EMITTED sample); its
// turn-on value is a seeded stationary draw, sigma_bias * n per axis, taken at spawn and again at every reseed.
//
// DRAWS: six standard normals per emitted sample, always, in this order -- the bias step (x, y, z), the white (x, y, z);
// three more at spawn and at every reseed (the turn-on bias), taken before any sample.
//
// CPU ONLY, as the barometer: grade `reference` on the CPU, `absent` on Vulkan (L6), no GPU twin.
// ---------------------------------------------------------------------------

namespace spade::sensors {

// The stream a magnetometer draws its noise from: PINNED.
inline constexpr std::string_view kMagNoiseDomainTag = "sensor.mag";

// The world's frame convention, for the helper below.
enum class MagWorldFrame : uint32_t { compass_x_north = 0, anchored_world = 1 };

// The field of a place: declination (east of true north) and inclination (positive down) in radians and a total strength in microtesla, as
// (N, E, D) = B (cos i cos d, cos i sin d, sin i), written in the world's frame: compass_x_north (N, -D, E), anchored_world (E, -D, -N).
// sin32/cos32, no libm.
[[nodiscard]] glm::vec3 mag_field_world(float declination_rad, float inclination_rad, float strength_ut,
                                        MagWorldFrame frame = MagWorldFrame::compass_x_north) noexcept;

// One reading. 32 bytes, every byte named.
struct alignas(kStd430StructAlignment) MagSample {
    glm::vec3 field_body_ut;  // the measured field in the sensor's axes, uT
    float _p0;                // std430 pad
    SampleIndex index;        // monotonically increasing, per sensor, starts at 1
    uint64_t tick;            // the step this sample was produced in
};

static_assert(std::is_standard_layout_v<MagSample>);
static_assert(std::is_trivially_copyable_v<MagSample>);
static_assert(std::is_trivially_destructible_v<MagSample>);
static_assert(alignof(MagSample) == 16 && sizeof(MagSample) == 32);
static_assert(offsetof(MagSample, field_body_ut) == 0);
static_assert(offsetof(MagSample, index) == 16);
static_assert(offsetof(MagSample, tick) == 24);
static_assert(sizeof(glm::vec3) + sizeof(float) + sizeof(SampleIndex) + sizeof(uint64_t) == sizeof(MagSample),
              "MagSample has implicit padding");

// What a caller states about a magnetometer (the attach_row record of the array "mag_sensors"). A sigma is a
// per-sample, per-axis standard deviation; bias_tau_s <= 0 disables the correlation.
struct MagSensorSpawn {
    glm::quat mount_orient{1.0f, 0.0f, 0.0f, 0.0f};  // mount -> body, as the IMU's
    uint32_t rate_divider = 1;                       // emit one sample every N substeps
    glm::vec3 field_world_ut{0.0f};                  // the world field, x north, y up, z east, uT
    glm::vec3 hard_iron_ut{0.0f};                    // an additive offset in the sensor's axes, uT
    glm::mat3 soft_iron{1.0f};                       // a matrix in the sensor's axes (columns), identity by default
    float sigma_white_ut = 0.0f;                     // white noise per axis, uT
    float sigma_bias_ut = 0.0f;                      // stationary stddev of the Gauss-Markov bias per axis, uT
    float bias_tau_s = 0.0f;                         // its correlation time, s; <= 0 disables
};
static_assert(sizeof(MagSensorSpawn) <= 128, "a spawn record is at most kMaxSpawnBytes");

// One magnetometer's row: an element of the per-sensor array "mag_sensors", 176 bytes, std430, every byte named.
struct alignas(kStd430StructAlignment) MagSensorRow {
    uint32_t body_slot;       // WORLD-LOCAL index into the world's bodies span
    uint32_t kind;            // sensor_kind::; 0 == none == inert
    uint32_t rate_divider;    // emit one sample every N substeps
    uint32_t phase;           // substeps elapsed in the current sample period
    glm::quat mount_orient;   // mount -> body
    glm::vec3 field_world_ut;
    float _p0;
    glm::vec3 hard_iron_ut;
    float _p1;
    glm::vec3 soft_iron_c0;   // the soft-iron matrix's columns, each with a std430 pad
    float _p2;
    glm::vec3 soft_iron_c1;
    float _p3;
    glm::vec3 soft_iron_c2;
    float _p4;
    float sigma_white_ut;
    float sigma_bias_ut;
    float bias_tau_s;
    float bias_retention;     // exp(-fix_dt / tau), from the SAMPLE period; 0 == no correlation
    glm::vec3 bias;           // the Gauss-Markov bias state, uT
    float bias_drive;         // sigma_bias * sqrt(1 - exp(-2 fix_dt / tau))
    rng::Stream noise;        // this magnetometer's own stream
    SampleIndex last_index;   // index of the newest sample written; 0 == none yet
    uint64_t _reserved0;      // must stay 0
};

static_assert(std::is_standard_layout_v<MagSensorRow>);
static_assert(std::is_trivially_copyable_v<MagSensorRow>);
static_assert(std::is_trivially_destructible_v<MagSensorRow>);
static_assert(alignof(MagSensorRow) == 16 && sizeof(MagSensorRow) == 176);
static_assert(offsetof(MagSensorRow, body_slot) == 0);
static_assert(offsetof(MagSensorRow, kind) == 4);
static_assert(offsetof(MagSensorRow, rate_divider) == 8);
static_assert(offsetof(MagSensorRow, phase) == 12);
static_assert(offsetof(MagSensorRow, mount_orient) == 16);
static_assert(offsetof(MagSensorRow, field_world_ut) == 32);
static_assert(offsetof(MagSensorRow, hard_iron_ut) == 48);
static_assert(offsetof(MagSensorRow, soft_iron_c0) == 64);
static_assert(offsetof(MagSensorRow, soft_iron_c1) == 80);
static_assert(offsetof(MagSensorRow, soft_iron_c2) == 96);
static_assert(offsetof(MagSensorRow, sigma_white_ut) == 112);
static_assert(offsetof(MagSensorRow, sigma_bias_ut) == 116);
static_assert(offsetof(MagSensorRow, bias_tau_s) == 120);
static_assert(offsetof(MagSensorRow, bias_retention) == 124);
static_assert(offsetof(MagSensorRow, bias) == 128);
static_assert(offsetof(MagSensorRow, bias_drive) == 140);
static_assert(offsetof(MagSensorRow, noise) == 144);
static_assert(offsetof(MagSensorRow, last_index) == 160);
static_assert(offsetof(MagSensorRow, _reserved0) == 168);
static_assert(4 * sizeof(uint32_t) + sizeof(glm::quat) + 5 * (sizeof(glm::vec3) + sizeof(float)) + 4 * sizeof(float) +
                      sizeof(glm::vec3) + sizeof(float) + sizeof(rng::Stream) + sizeof(SampleIndex) + sizeof(uint64_t) ==
                  sizeof(MagSensorRow),
              "MagSensorRow has implicit padding");

[[nodiscard]] constexpr rng::Stream mag_noise_stream(uint64_t world_seed, uint32_t local_slot) noexcept {
    return rng::make_stream(world_seed, kMagNoiseDomainTag, local_slot);
}

// The true field in the sensor's axes for a body pose and a mount: (body.orient * mount_orient)^-1 * field_world.
[[nodiscard]] glm::vec3 mag_truth(const glm::quat& body_orient, const glm::quat& mount_orient,
                                  const glm::vec3& field_world_ut) noexcept;

// The mag.synthesize pass's work for ONE world's spans (the pass in sim/ calls it). The precondition is the IMU's.
void synthesize_mag(std::span<const BodyState> bodies, std::span<MagSensorRow> sensors, std::span<MagSample> rings,
                    uint64_t tick) noexcept;

}  // namespace spade::sensors
