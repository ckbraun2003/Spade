// The built-in modules' row functions (sim/builtin_state.hpp). Each init is
// the structural queue's old per-kind case (init_drag, init_imu, init_gnss,
// init_rotor in Simulation::apply_op), moved here with its comments: the same
// field-wise writes in the same order, the same fp32 expressions, so the bytes
// it leaves are the bytes it left. Each validate is the old add_* call's
// parameter check, with the same messages less their "<call>: " prefix.
#include "sim/builtin_state.hpp"

#include <string>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/validate.hpp"
#include "physics/forces.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sim/simulation.hpp"
#include "vehicles/rotor.hpp"

namespace spade::modules::builtin {
namespace {

[[nodiscard]] std::unexpected<Error> invalid(std::string context) {
    return std::unexpected(Error{Code::invalid_argument, std::move(context)});
}

}  // namespace

// ---------------------------------------------------------------------------
// drag_bodies
// ---------------------------------------------------------------------------

void init_drag_row(const RowInit& in) noexcept {
    physics::DragBodyRow& row = row_as<physics::DragBodyRow>(in.row);
    const DragElementSpawn drag = spawn_as<DragElementSpawn>(in.spawn);

    // WORLD-LOCAL, not global: physics/forces.hpp documents body_slot as an
    // index into the world's body slice, which is what apply_drag() is handed.
    // Already written at reservation time; rewritten here so this function
    // remains the complete statement of the row's contents.
    row.body_slot = in.local_body;
    row.mode = drag.mode;
    row.area = drag.area;
    row.local_pos = drag.local_pos;
    row.local_orient = glm::normalize(drag.local_orient);
    row.coeffs = drag.coeffs;
    // LAST, like every built-in's liveness flag: until `enabled` is set the
    // row is inert to apply_drag(). (The old init_drag case set it second; the
    // flush writes the row in one go, so the order changes no byte.)
    row.enabled = 1u;
}

Result<void> validate_drag_spawn(std::span<const std::byte> spawn) {
    const DragElementSpawn elem = spawn_as<DragElementSpawn>(spawn);
    if (!finite(elem.area) || !finite(elem.coeffs) || !finite(elem.local_pos) || !finite(elem.local_orient)) {
        return invalid("parameters must all be finite");
    }
    if (const char* why = physics::check_drag_law(elem.mode, elem.area, elem.coeffs).first()) {
        return invalid(why);
    }
    if (!(glm::dot(elem.local_orient, elem.local_orient) > 0.0f)) {
        return invalid("local_orient must have non-zero length");
    }
    return {};
}

// ---------------------------------------------------------------------------
// imu_sensors
// ---------------------------------------------------------------------------

void init_imu_row(const RowInit& in) noexcept {
    sensors::ImuSensorRow& row = row_as<sensors::ImuSensorRow>(in.row);
    const ImuSensorSpawn imu = spawn_as<ImuSensorSpawn>(in.spawn);

    // FIELD-WISE, NOT WHOLE-OBJECT -- see apply_op's init_body. Every field is
    // written even where the arena's zero-fill would already have done
    // it: the row's contents are this function's complete statement,
    // not a coincidence of how the slot came to be free.
    row.body_slot = in.local_body;
    row.rate_divider = imu.rate_divider;
    row.phase = 0u;
    row.mount_pos = imu.mount_pos;
    row._p0 = 0.0f;
    row.mount_orient = glm::normalize(imu.mount_orient);
    row.sigma_a = imu.sigma_a;
    row.sigma_g = imu.sigma_g;
    row.sigma_ba = imu.sigma_ba;
    row.sigma_bg = imu.sigma_bg;
    row.bias_a = glm::vec3(0.0f);
    row._p1 = 0.0f;
    row.bias_g = glm::vec3(0.0f);
    row._p2 = 0.0f;
    // Seeded from the REGISTERED WorldParams row -- the world's rng
    // authority (see WorldConfig's "NO `seed` MEMBER" note) -- and from
    // the WORLD-LOCAL slot, so a world's noise does not depend on where
    // that world sits in the set.
    row.noise = sensors::imu_noise_stream(in.params->seed, in.local_slot);
    row.last_index = 0;
    row._reserved0 = 0;
    // LAST, like body_flags::active in init_body: until `kind` is set
    // the row is inert to the sensor passes, so a partially
    // written row can never be sampled.
    row.kind = sensors::sensor_kind::imu;
}

Result<void> validate_imu_spawn(std::span<const std::byte> spawn) {
    const ImuSensorSpawn sensor = spawn_as<ImuSensorSpawn>(spawn);
    if (sensor.rate_divider == 0) {
        // 0 would mean "no rate at all". synthesize_imu() degrades it to 1
        // rather than dividing by zero, but a caller who wrote 0 meant
        // something, and it was not "every substep".
        return invalid("rate_divider must be >= 1");
    }
    if (!finite(sensor.mount_pos) || !finite(sensor.mount_orient)) {
        return invalid("mount pose must be finite");
    }
    if (!(glm::dot(sensor.mount_orient, sensor.mount_orient) > 0.0f)) {
        return invalid("mount_orient must have non-zero length");
    }
    if (!(sensor.sigma_a >= 0.0f) || !(sensor.sigma_g >= 0.0f) || !(sensor.sigma_ba >= 0.0f) ||
        !(sensor.sigma_bg >= 0.0f) || !finite(sensor.sigma_a) || !finite(sensor.sigma_g) ||
        !finite(sensor.sigma_ba) || !finite(sensor.sigma_bg)) {
        // Spelled `!(x >= 0)` so a NaN rejects rather than comparing false on
        // both sides -- the same discipline world_set.cpp's in_range() uses.
        return invalid("every sigma must be finite and >= 0");
    }
    return {};
}

// ---------------------------------------------------------------------------
// gnss_sensors
// ---------------------------------------------------------------------------

void init_gnss_row(const RowInit& in) noexcept {
    sensors::GnssSensorRow& row = row_as<sensors::GnssSensorRow>(in.row);
    const GnssSensorSpawn gnss = spawn_as<GnssSensorSpawn>(in.spawn);

    // ⛔⛔ THE FIX PERIOD, NOT THE SUBSTEP. The Gauss-Markov bias
    // advances ONCE PER EMITTED FIX -- it is the receiver's own error
    // process and runs on the receiver's own clock -- so `dt` is
    // rate_divider substeps, not one. Computed HERE rather than in
    // add_gnss_sensor() because this is where the row is written and
    // where the arena hands it over; the substep `h` is the run's,
    // fixed at create().
    //
    //   A CONSTANT COMPUTED FROM THE WRONG CLOCK IS STILL
    //   DETERMINISTIC, AND DETERMINISM IS WHAT THIS SUITE CHECKS.
    //
    // A receiver at rate_divider = 200 on the substep clock decays 200x
    // too slowly, both backends still agree bit for bit, every digest
    // stays self-consistent, and the corpus pins it wrong forever. The
    // only thing standing between that and this line is this comment.
    const float fix_dt = static_cast<float>(gnss.rate_divider) * in.h;

    // FIELD-WISE, NOT WHOLE-OBJECT -- see apply_op's init_body and init_imu_row. The
    // row's contents are this function's complete statement.
    row.body_slot = in.local_body;
    row.rate_divider = gnss.rate_divider;
    row.phase = 0u;
    row.mount_pos = gnss.mount_pos;
    row._p0 = 0.0f;
    row.bias = glm::vec3(0.0f);
    row._p1 = 0.0f;
    row.sigma_h = gnss.sigma_h;
    row.sigma_v = gnss.sigma_v;
    row.sigma_vel = gnss.sigma_vel;
    row.bias_tau_s = gnss.bias_tau_s;
    row.noise = sensors::gnss_noise_stream(in.params->seed, in.local_slot);
    row.last_index = 0;
    row.bias_retention = sensors::gnss_bias_retention(fix_dt, gnss.bias_tau_s);
    row.bias_drive = sensors::gnss_bias_drive(fix_dt, gnss.bias_tau_s, gnss.sigma_bias);
    row.sigma_bias = gnss.sigma_bias;
    row._p2 = 0.0f;
    row._reserved0 = 0;
    // LAST, like init_imu_row's: until `kind` is set the row is inert to
    // the sensor passes, so a partially written row can never be
    // sampled.
    row.kind = sensors::sensor_kind::gnss;
}

Result<void> validate_gnss_spawn(std::span<const std::byte> spawn) {
    const GnssSensorSpawn sensor = spawn_as<GnssSensorSpawn>(spawn);
    if (sensor.rate_divider == 0) {
        return invalid("rate_divider must be >= 1");
    }
    if (!finite(sensor.mount_pos)) {
        return invalid("mount_pos must be finite");
    }
    // Spelled `!(x >= 0)` so a NaN REJECTS rather than comparing false on both
    // sides -- validate_imu_spawn()'s discipline, and world_set.cpp's
    // in_range(). bias_tau_s joins the sigmas here: <= 0 legitimately DISABLES
    // the bias (gnss_bias_retention returns 0), but a NEGATIVE tau is a caller
    // error and a NaN one would propagate into every fix through a
    // finite-looking row.
    if (!(sensor.sigma_h >= 0.0f) || !(sensor.sigma_v >= 0.0f) || !(sensor.sigma_vel >= 0.0f) ||
        !(sensor.sigma_bias >= 0.0f) || !(sensor.bias_tau_s >= 0.0f) || !finite(sensor.sigma_h) ||
        !finite(sensor.sigma_v) || !finite(sensor.sigma_vel) || !finite(sensor.sigma_bias) ||
        !finite(sensor.bias_tau_s)) {
        return invalid("every sigma and bias_tau_s must be finite and >= 0");
    }
    return {};
}

// ---------------------------------------------------------------------------
// rotors
// ---------------------------------------------------------------------------

void init_rotor_row(const RowInit& in) noexcept {
    vehicles::RotorRow& row = row_as<vehicles::RotorRow>(in.row);
    const RotorSpawn rotor = spawn_as<RotorSpawn>(in.spawn);

    // FIELD-WISE, NOT WHOLE-OBJECT -- see apply_op's init_body. Every field is
    // written, including the four reserved lanes: rotor.hpp requires
    // them to stay 0, and stating that here rather than relying on the
    // arena's zero-fill makes this function the row's complete
    // definition.
    row.body_slot = in.local_body;
    row.tau = rotor.desc.tau;
    row.radius = rotor.desc.radius;
    row.local_pos = rotor.desc.local_pos;
    row.spin_dir = rotor.desc.spin_dir;
    row.local_orient = glm::normalize(rotor.desc.local_orient);
    // SPAWNED IN TRIM: omega and omega_cmd both take the spawn's
    // rotor_omega, so a vehicle inserted mid-flight holds its speed
    // instead of spinning up from rest. See VehicleSpawn.
    row.omega = rotor.omega;
    row.omega_cmd = rotor.omega;
    row.thrust_coeff = rotor.desc.thrust_coeff;
    row.torque_coeff = rotor.desc.torque_coeff;
    row._r0 = 0.0f;
    row._r1 = 0.0f;
    row._r2 = 0.0f;
    row._r3 = 0.0f;
    // LAST, like body_flags::active in init_body and `kind` in
    // init_imu_row: until `enabled` is set the row is inert to
    // apply_rotors(), so a partially written row can never be stepped.
    row.enabled = 1u;
}

// A rotor is part of a vehicle: its model's descriptor, its slot in the
// VehicleRef and its command all come from spawn(world, model, where). One
// bolted onto a body by hand would have no VehicleRef to command it through.
Result<void> refuse_direct_rotor(std::span<const std::byte>) {
    return invalid("rotors arrive only with a vehicle; spawn(world, model, where) attaches them");
}

}  // namespace spade::modules::builtin
