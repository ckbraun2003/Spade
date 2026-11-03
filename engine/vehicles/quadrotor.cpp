#include "vehicles/quadrotor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

// ---------------------------------------------------------------------------
// The quadrotor, assembled. Read quadrotor.hpp's sections 1-3 first: the
// geometry, the control mapping and the scope boundaries live there, and this
// file is only their transcription.
// ---------------------------------------------------------------------------

namespace spade::vehicles {

glm::vec3 quadrotor_arm_offset(const QuadrotorParams& params, std::size_t index) noexcept {
    const float l = params.arm_length;
    const float h = params.rotor_height;
    // quadrotor.hpp section 1's table, spelled once. A switch rather than a
    // trigonometric parameterization: four exact offsets, with no sin/cos
    // rounding to make the +X arm and the -X arm disagree about |L|, and no
    // way for a future reader to wonder which azimuth convention was meant.
    switch (index) {
        case 0: return glm::vec3(l, h, 0.0f);
        case 1: return glm::vec3(0.0f, h, l);
        case 2: return glm::vec3(-l, h, 0.0f);
        case 3: return glm::vec3(0.0f, h, -l);
        default: return glm::vec3(0.0f);
    }
}

Result<ModelType> make_quadrotor(const QuadrotorParams& params) {
    // The arm length is checked HERE rather than in ModelType::validate(),
    // because a ModelType has no concept of an arm -- it has four independent
    // hub offsets, and (0,0,0) is a perfectly legal one for a coaxial layout.
    // A quadrotor with a zero arm is a quadrotor with no roll or pitch
    // authority at all (M_x = M_z = 0 identically, whatever the four
    // commands), which is exactly the kind of plausible-looking degenerate
    // this layer exists to refuse.
    if (!(params.arm_length > 0.0f) || !std::isfinite(params.arm_length)) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "make_quadrotor: arm_length must be finite and > 0 (a zero arm "
                                     "leaves the airframe with no roll or pitch authority)"});
    }
    if (!std::isfinite(params.rotor_height)) {
        return std::unexpected(
            Error{Code::invalid_argument, "make_quadrotor: rotor_height must be finite"});
    }

    ModelType model;
    model.name = params.name;
    model.param_schema_id = params.param_schema_id;
    model.visual_ref = params.visual_ref;
    model.body.mass = params.mass;
    model.body.inertia_diag = params.inertia_diag;
    model.proxy_radius = params.proxy_radius;

    model.rotors.reserve(kQuadrotorRotorCount);
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
        const RotorParams& calibration = params.rotors[i];
        RotorDesc rotor;
        rotor.local_pos = quadrotor_arm_offset(params, i);
        // IDENTITY, deliberately: rotor.hpp's kRotorLocalThrustAxis is local
        // +Y and Spade is Y-up, so an identity mount is a rotor thrusting
        // straight up out of a level airframe. A tilted-rotor airframe is a
        // different model type, not a QuadrotorParams field.
        rotor.local_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        rotor.spin_dir = params.spin_dirs[i];
        rotor.tau = calibration.tau;
        rotor.radius = calibration.radius;
        rotor.thrust_coeff = calibration.thrust_coeff;
        rotor.torque_coeff = calibration.torque_coeff;
        model.rotors.push_back(rotor);
    }

    model.drag_bodies.push_back(params.drag);
    model.imu_mounts.push_back(params.imu);

    // Everything else -- mass, inertia, the four radii and coefficients, the
    // spin layout's entries, the drag mode, the IMU's sigmas -- is checked by
    // the layer that owns the checking, and its message is returned verbatim
    // so a caller sees "model type 'x': rotor 2: radius must be ..." rather
    // than a re-worded copy that can drift.
    if (Result<void> valid = model.validate(); !valid) {
        return std::unexpected(valid.error());
    }

    // Two discs in the one rotor plane may not overlap: their blades would
    // meet (the lead, 2026-10-13; the old defaults did, adjacent hubs L sqrt2
    // = 0.212 m apart against 2 R = 0.24 m). Tips that touch are allowed.
    // After validate(), so every radius is finite and positive here; in
    // double, so a pair a hair apart is decided by the geometry, not a float's
    // rounding. A pair's hubs lie L times a fixed spacing apart (sqrt2 for
    // neighbours, 2 across), so the arm that clears it is L reach / apart.
    double clearing_arm = 0.0;
    std::size_t first = kQuadrotorRotorCount;  // the first overlapping pair, in (i, j) order
    std::size_t second = kQuadrotorRotorCount;
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
        for (std::size_t j = i + 1; j < kQuadrotorRotorCount; ++j) {
            const glm::dvec3 d = glm::dvec3(model.rotors[i].local_pos) - glm::dvec3(model.rotors[j].local_pos);
            const double apart = std::sqrt(glm::dot(d, d));
            const double reach =
                static_cast<double>(model.rotors[i].radius) + static_cast<double>(model.rotors[j].radius);
            clearing_arm = std::max(clearing_arm, static_cast<double>(params.arm_length) * reach / apart);
            if (apart < reach && first == kQuadrotorRotorCount) {
                first = i;
                second = j;
            }
        }
    }
    if (first != kQuadrotorRotorCount) {
        const glm::dvec3 d =
            glm::dvec3(model.rotors[first].local_pos) - glm::dvec3(model.rotors[second].local_pos);
        const double reach =
            static_cast<double>(model.rotors[first].radius) + static_cast<double>(model.rotors[second].radius);
        char why[256];
        std::snprintf(why, sizeof why,
                      "make_quadrotor: rotors %zu and %zu overlap: their hubs are %.4f m apart and their radii sum to "
                      "%.4f m, so their blades would meet; an arm_length of at least %.4f m clears every pair",
                      first, second, std::sqrt(glm::dot(d, d)), reach, std::ceil(clearing_arm * 1.0e4) / 1.0e4);
        return std::unexpected(Error{Code::invalid_argument, std::string(why)});
    }
    return model;
}

float hover_command(const QuadrotorParams& params, float gravity_magnitude) noexcept {
    // Every guard spelled !(x > 0) so a NaN takes the degenerate exit instead
    // of reaching the square root -- rotor.cpp's posture, for the same reason.
    if (!(params.mass > 0.0f) || !(gravity_magnitude > 0.0f)) {
        return 0.0f;
    }

    // sum_i k_T,i, in index order. Summed rather than assumed uniform: the
    // airframe's four calibrations are independent, and for the symmetric case
    // this is exactly 4 * k_T with no rounding difference from writing it that
    // way (four adds of the same float).
    float thrust_coeff_sum = 0.0f;
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
        thrust_coeff_sum += params.rotors[i].thrust_coeff;
    }
    if (!(thrust_coeff_sum > 0.0f)) {
        return 0.0f;
    }

    const float weight = params.mass * gravity_magnitude;
    return std::sqrt(weight / thrust_coeff_sum);
}

}  // namespace spade::vehicles
