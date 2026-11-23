// sim/sensor_modules.cpp -- the opt-in barometer and magnetometer modules (Phase 1 of the flight-computer work, O144; sensors/baro.hpp,
// sensors/mag.hpp). Neither is in standard_modules(): a set that adds them owns two per_sensor arrays and two rings more, and a set that does not keeps
// every digest and configuration hash it has. Each is a CPU pass in the Sensors phase over its own rows, graded `reference` on the CPU and `absent` on
// Vulkan (no GPU twin: a Vulkan world that holds either is refused at create(), L6).

#include <cmath>

#include <glm/geometric.hpp>

#include "physics/schedule.hpp"
#include "sensors/baro.hpp"
#include "sensors/camera.hpp"
#include "sensors/gnss.hpp"  // gnss_bias_retention / gnss_bias_drive: the one Gauss-Markov coefficient pair, whoever's sensor
#include "sensors/mag.hpp"
#include "sim/module.hpp"
#include "state/state_view.hpp"

namespace spade::modules {

namespace {

[[nodiscard]] Result<void> invalid(const char* why) { return std::unexpected(Error{Code::invalid_argument, why}); }

[[nodiscard]] bool finite(float v) noexcept { return std::isfinite(v); }
[[nodiscard]] bool finite(const glm::vec3& v) noexcept { return finite(v.x) && finite(v.y) && finite(v.z); }
[[nodiscard]] bool non_negative(float v) noexcept { return finite(v) && v >= 0.0f; }  // spelled so a NaN is refused

// ---------------------------------------------------------------------------
// baro
// ---------------------------------------------------------------------------

// The streams' reseed: re-derive the sensor's stream and take the TURN-ON bias from it (one normal), as init does. A reseed is a new random life
// (an episode), so the offset a run starts with follows the seed; the phase and the ring are history and are left alone. tau <= 0 disables the bias.
void reseed_baro_row(std::span<std::byte> bytes, const WorldParams& params, uint32_t local_slot) noexcept {
    sensors::BaroSensorRow& row = row_as<sensors::BaroSensorRow>(bytes);
    row.noise = sensors::baro_noise_stream(params.seed, local_slot);
    const float turn_on = row.noise.next_gauss();
    row.bias = row.bias_tau_s > 0.0f ? row.sigma_bias_pa * turn_on : 0.0f;
}

void init_baro_row(const RowInit& in) noexcept {
    sensors::BaroSensorRow& row = row_as<sensors::BaroSensorRow>(in.row);
    const sensors::BaroSensorSpawn spawn = spawn_as<sensors::BaroSensorSpawn>(in.spawn);

    // The bias advances once per EMITTED sample, so its coefficients are on the sample period (rate_divider substeps), not the substep.
    const float sample_dt = static_cast<float>(spawn.rate_divider) * in.h;

    // Field-wise, never a whole-object assignment (a temporary's padding must not reach the arena); `kind` last.
    row.body_slot = in.local_body;
    row.rate_divider = spawn.rate_divider;
    row.phase = 0u;
    row.mount_pos = spawn.mount_pos;
    row.world_origin_altitude_m = spawn.world_origin_altitude_m;
    row.sea_level_pressure_pa = spawn.sea_level_pressure_pa;
    row.sea_level_temperature_k = spawn.sea_level_temperature_k;
    row.lapse_k_per_m = spawn.lapse_k_per_m;
    row.temp_coeff_pa_per_k = spawn.temp_coeff_pa_per_k;
    row.sigma_p_pa = spawn.sigma_p_pa;
    row.sigma_t_k = spawn.sigma_t_k;
    row.sigma_bias_pa = spawn.sigma_bias_pa;
    row.bias_tau_s = spawn.bias_tau_s;
    row.bias = 0.0f;
    row.bias_retention = sensors::gnss_bias_retention(sample_dt, spawn.bias_tau_s);
    row.bias_drive = sensors::gnss_bias_drive(sample_dt, spawn.bias_tau_s, spawn.sigma_bias_pa);
    row._p0 = 0.0f;
    row.last_index = 0;
    row._reserved0 = 0;
    reseed_baro_row(in.row, *in.params, in.local_slot);  // the stream and the turn-on bias
    row.kind = sensors::sensor_kind::baro;
}

[[nodiscard]] Result<void> validate_baro_spawn(std::span<const std::byte> bytes) {
    const sensors::BaroSensorSpawn s = spawn_as<sensors::BaroSensorSpawn>(bytes);
    if (s.rate_divider == 0) return invalid("rate_divider must be >= 1");
    if (!finite(s.mount_pos)) return invalid("mount_pos must be finite");
    if (!finite(s.world_origin_altitude_m) || !finite(s.temp_coeff_pa_per_k)) {
        return invalid("world_origin_altitude_m and temp_coeff_pa_per_k must be finite");
    }
    if (!(s.sea_level_pressure_pa > 0.0f) || !finite(s.sea_level_pressure_pa)) {
        return invalid("sea_level_pressure_pa must be finite and > 0");
    }
    if (!(s.sea_level_temperature_k > 0.0f) || !finite(s.sea_level_temperature_k)) {
        return invalid("sea_level_temperature_k must be finite and > 0");
    }
    if (!(s.lapse_k_per_m > 0.0f) || !finite(s.lapse_k_per_m)) return invalid("lapse_k_per_m must be finite and > 0");
    if (!non_negative(s.sigma_p_pa) || !non_negative(s.sigma_t_k) || !non_negative(s.sigma_bias_pa) || !non_negative(s.bias_tau_s)) {
        return invalid("every sigma and bias_tau_s must be finite and >= 0");
    }
    // The ISA is the troposphere's: refuse a mount height outside it rather than quietly clamp (the body's own height is not known here).
    const float mount_height = s.world_origin_altitude_m + s.mount_pos.y;
    if (!(mount_height >= sensors::kBaroMinHeightM && mount_height <= sensors::kBaroMaxHeightM)) {
        return invalid("world_origin_altitude_m + mount_pos.y must be within -1000..11000 m (the ISA troposphere)");
    }
    return {};
}

void pass_baro(const physics::SubstepContext& ctx) noexcept {
    for (std::size_t w = 0; w < ctx.worlds.size(); ++w) {
        const uint32_t world = static_cast<uint32_t>(w);
        sensors::synthesize_baro(ctx.worlds[w].bodies, world_rows<sensors::BaroSensorRow>(ctx.state[1], world),
                                 world_rows<sensors::BaroSample>(ctx.state[2], world), ctx.tick.value);
    }
}

constexpr ArrayDecl kBaroArrays[] = {
    {.name = "baro_sensors",
     .elem_size = attached_row_size<sensors::BaroSensorRow>(),
     .extent = Extent::per_sensor,
     .spawn_size = sizeof(sensors::BaroSensorSpawn),
     .init = &init_baro_row,
     .validate = &validate_baro_spawn},
    {.name = "baro_ring",
     .elem_size = row_size<sensors::BaroSample>(),
     .extent = Extent::per_row,
     .owner = "baro_sensors",
     .depth = sensors::kRingDepth}};
constexpr StreamDecl kBaroStreams[] = {
    {.tag = sensors::kBaroNoiseDomainTag, .array = "baro_sensors", .reseed = &reseed_baro_row}};
constexpr QuantityAccess kBaroAccess[] = {
    {"body.pose", Access::read}, {"baro.baro_sensors", Access::write}, {"baro.baro_ring", Access::write}};
constexpr PassDecl kBaroPasses[] = {{.name = "synthesize", .phase = Phase::sensors, .access = kBaroAccess, .cpu = &pass_baro}};

// ---------------------------------------------------------------------------
// camera (Phase 3.2: the row, the schedule and the due-frame ring; the pixels are Rendering's)
// ---------------------------------------------------------------------------

// The stream is re-derived from the world-local slot, as every sensor's is. A camera draws nothing in Core (its noise is Rendering's, phase 6), so the
// stream is only positioned, ready for whoever draws from it.
void reseed_camera_row(std::span<std::byte> bytes, const WorldParams& params, uint32_t local_slot) noexcept {
    sensors::CameraSensorRow& row = row_as<sensors::CameraSensorRow>(bytes);
    row.noise = sensors::camera_noise_stream(params.seed, local_slot);
}

void init_camera_row(const RowInit& in) noexcept {
    sensors::CameraSensorRow& row = row_as<sensors::CameraSensorRow>(in.row);
    const sensors::CameraSensorSpawn spawn = spawn_as<sensors::CameraSensorSpawn>(in.spawn);

    // Field-wise, never a whole-object assignment (a temporary's padding must not reach the arena); `kind` last.
    row.body_slot = in.local_body;
    row.rate_divider = spawn.rate_divider;
    row.phase = 0u;
    row.mount_pos = spawn.mount_pos;
    row.shutter_fraction = spawn.shutter_fraction;
    row.mount_orient = spawn.mount_orient;
    row.lens_kind = spawn.lens_kind;
    row.width = spawn.width;
    row.height = spawn.height;
    row.mode = spawn.mode;
    row.fx = spawn.fx;
    row.fy = spawn.fy;
    row.cx = spawn.cx;
    row.cy = spawn.cy;
    row.k1 = spawn.k1;
    row.k2 = spawn.k2;
    row.k3 = spawn.k3;
    row.k4 = spawn.k4;
    row.blur_samples = spawn.blur_samples;
    row.aa_samples = spawn.aa_samples;
    row.exposure_gain = spawn.exposure_gain;
    row.cull_mask = spawn.cull_mask;
    row.channels = spawn.channels;
    row._p0 = 0u;
    row.last_index = 0;
    row._reserved0 = 0;
    row._reserved1 = 0;
    reseed_camera_row(in.row, *in.params, in.local_slot);
    row.kind = sensors::sensor_kind::camera;
}

// The row's own rules, on a row built from the spawn. The one rule that needs the world (keypoints need a declared keypoint_groups) is the world's to
// enforce where the capacities are known; here a placeholder of 1 lets every other rule speak.
[[nodiscard]] Result<void> validate_camera_spawn(std::span<const std::byte> bytes) {
    const sensors::CameraSensorSpawn s = spawn_as<sensors::CameraSensorSpawn>(bytes);
    sensors::CameraSensorRow row{};
    row.kind = sensors::sensor_kind::camera;
    row.rate_divider = s.rate_divider;
    row.mount_pos = s.mount_pos;
    row.shutter_fraction = s.shutter_fraction;
    row.mount_orient = s.mount_orient;
    row.lens_kind = s.lens_kind;
    row.width = s.width;
    row.height = s.height;
    row.mode = s.mode;
    row.fx = s.fx;
    row.fy = s.fy;
    row.cx = s.cx;
    row.cy = s.cy;
    row.k1 = s.k1;
    row.k2 = s.k2;
    row.k3 = s.k3;
    row.k4 = s.k4;
    row.blur_samples = s.blur_samples;
    row.aa_samples = s.aa_samples;
    row.exposure_gain = s.exposure_gain;
    row.cull_mask = s.cull_mask;
    row.channels = s.channels;
    const std::string issue = sensors::camera_row_issue(row, 1u);
    if (!issue.empty()) return std::unexpected(Error{Code::invalid_argument, issue});
    return {};
}

void pass_camera(const physics::SubstepContext& ctx) noexcept {
    for (std::size_t w = 0; w < ctx.worlds.size(); ++w) {
        const uint32_t world = static_cast<uint32_t>(w);
        sensors::schedule_cameras(ctx.worlds[w].bodies, world_rows<sensors::CameraSensorRow>(ctx.state[1], world),
                                  world_rows<sensors::CameraFrameDue>(ctx.state[2], world), ctx.tick.value);
    }
}

constexpr ArrayDecl kCameraArrays[] = {
    {.name = "camera_sensors",
     .elem_size = attached_row_size<sensors::CameraSensorRow>(),
     .extent = Extent::per_sensor,
     .spawn_size = sizeof(sensors::CameraSensorSpawn),
     .init = &init_camera_row,
     .validate = &validate_camera_spawn},
    {.name = "camera_frames",
     .elem_size = row_size<sensors::CameraFrameDue>(),
     .extent = Extent::per_row,
     .owner = "camera_sensors",
     .depth = sensors::kRingDepth}};
constexpr StreamDecl kCameraStreams[] = {
    {.tag = sensors::kCameraNoiseDomainTag, .array = "camera_sensors", .reseed = &reseed_camera_row}};
constexpr QuantityAccess kCameraAccess[] = {
    {"body.pose", Access::read}, {"camera.camera_sensors", Access::write}, {"camera.camera_frames", Access::write}};
constexpr PassDecl kCameraPasses[] = {{.name = "schedule", .phase = Phase::sensors, .access = kCameraAccess, .cpu = &pass_camera}};

// ---------------------------------------------------------------------------
// mag

// ---------------------------------------------------------------------------

void reseed_mag_row(std::span<std::byte> bytes, const WorldParams& params, uint32_t local_slot) noexcept {
    sensors::MagSensorRow& row = row_as<sensors::MagSensorRow>(bytes);
    row.noise = sensors::mag_noise_stream(params.seed, local_slot);
    // The turn-on bias: three normals, x then y then z, named locals (argument order is unspecified).
    const float nx = row.noise.next_gauss();
    const float ny = row.noise.next_gauss();
    const float nz = row.noise.next_gauss();
    row.bias = row.bias_tau_s > 0.0f ? row.sigma_bias_ut * glm::vec3(nx, ny, nz) : glm::vec3(0.0f);
}

void init_mag_row(const RowInit& in) noexcept {
    sensors::MagSensorRow& row = row_as<sensors::MagSensorRow>(in.row);
    const sensors::MagSensorSpawn spawn = spawn_as<sensors::MagSensorSpawn>(in.spawn);
    const float sample_dt = static_cast<float>(spawn.rate_divider) * in.h;

    row.body_slot = in.local_body;
    row.rate_divider = spawn.rate_divider;
    row.phase = 0u;
    row.mount_orient = glm::normalize(spawn.mount_orient);
    row.field_world_ut = spawn.field_world_ut;
    row._p0 = 0.0f;
    row.hard_iron_ut = spawn.hard_iron_ut;
    row._p1 = 0.0f;
    row.soft_iron_c0 = spawn.soft_iron[0];
    row._p2 = 0.0f;
    row.soft_iron_c1 = spawn.soft_iron[1];
    row._p3 = 0.0f;
    row.soft_iron_c2 = spawn.soft_iron[2];
    row._p4 = 0.0f;
    row.sigma_white_ut = spawn.sigma_white_ut;
    row.sigma_bias_ut = spawn.sigma_bias_ut;
    row.bias_tau_s = spawn.bias_tau_s;
    row.bias_retention = sensors::gnss_bias_retention(sample_dt, spawn.bias_tau_s);
    row.bias = glm::vec3(0.0f);
    row.bias_drive = sensors::gnss_bias_drive(sample_dt, spawn.bias_tau_s, spawn.sigma_bias_ut);
    row.last_index = 0;
    row._reserved0 = 0;
    reseed_mag_row(in.row, *in.params, in.local_slot);
    row.kind = sensors::sensor_kind::mag;
}

[[nodiscard]] Result<void> validate_mag_spawn(std::span<const std::byte> bytes) {
    const sensors::MagSensorSpawn s = spawn_as<sensors::MagSensorSpawn>(bytes);
    if (s.rate_divider == 0) return invalid("rate_divider must be >= 1");
    if (!finite(s.field_world_ut) || !finite(s.hard_iron_ut)) return invalid("field_world_ut and hard_iron_ut must be finite");
    for (int c = 0; c < 3; ++c) {
        if (!finite(s.soft_iron[c])) return invalid("soft_iron must be finite");
    }
    const float length_sq = s.mount_orient.w * s.mount_orient.w + s.mount_orient.x * s.mount_orient.x +
                            s.mount_orient.y * s.mount_orient.y + s.mount_orient.z * s.mount_orient.z;
    if (!(std::fabs(length_sq - 1.0f) < 2.0e-3f)) return invalid("mount_orient must be a finite unit quaternion");
    if (!non_negative(s.sigma_white_ut) || !non_negative(s.sigma_bias_ut) || !non_negative(s.bias_tau_s)) {
        return invalid("every sigma and bias_tau_s must be finite and >= 0");
    }
    return {};
}

void pass_mag(const physics::SubstepContext& ctx) noexcept {
    for (std::size_t w = 0; w < ctx.worlds.size(); ++w) {
        const uint32_t world = static_cast<uint32_t>(w);
        sensors::synthesize_mag(ctx.worlds[w].bodies, world_rows<sensors::MagSensorRow>(ctx.state[1], world),
                                world_rows<sensors::MagSample>(ctx.state[2], world), ctx.tick.value);
    }
}

constexpr ArrayDecl kMagArrays[] = {
    {.name = "mag_sensors",
     .elem_size = attached_row_size<sensors::MagSensorRow>(),
     .extent = Extent::per_sensor,
     .spawn_size = sizeof(sensors::MagSensorSpawn),
     .init = &init_mag_row,
     .validate = &validate_mag_spawn},
    {.name = "mag_ring",
     .elem_size = row_size<sensors::MagSample>(),
     .extent = Extent::per_row,
     .owner = "mag_sensors",
     .depth = sensors::kRingDepth}};
constexpr StreamDecl kMagStreams[] = {
    {.tag = sensors::kMagNoiseDomainTag, .array = "mag_sensors", .reseed = &reseed_mag_row}};
constexpr QuantityAccess kMagAccess[] = {
    {"body.pose", Access::read}, {"mag.mag_sensors", Access::write}, {"mag.mag_ring", Access::write}};
constexpr PassDecl kMagPasses[] = {{.name = "synthesize", .phase = Phase::sensors, .access = kMagAccess, .cpu = &pass_mag}};

constexpr GradeDecl kCpuOnlyGrade{.cpu = Grade::reference, .vulkan = Grade::absent};

}  // namespace

ModuleDesc camera_module() {
    return {.name = "camera", .passes = kCameraPasses, .state = kCameraArrays, .streams = kCameraStreams, .grade = kCpuOnlyGrade};
}

ModuleDesc baro_module() {
    return {.name = "baro", .passes = kBaroPasses, .state = kBaroArrays, .streams = kBaroStreams, .grade = kCpuOnlyGrade};
}

ModuleDesc mag_module() {
    return {.name = "mag", .passes = kMagPasses, .state = kMagArrays, .streams = kMagStreams, .grade = kCpuOnlyGrade};
}

}  // namespace spade::modules
