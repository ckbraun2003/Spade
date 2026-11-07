// ---------------------------------------------------------------------------
// editor_flight_readout.hpp -- Run air's flight readout, display-free (the lead's approval, 2026-11-07).
//
// What a user reads while the air runs: the airspeed and the angles the air meets the drone at, each rotor's speed
// and thrust against the weight, and the attitude and rates the run holds. Every number is from what LiveRun already
// holds (the attitude, the rates, the rotors' speeds and k_T) and the case's air; nothing is asked of the solver.
//
// HONESTY. The thrust is the model's static curve, k_T w^2: it is not the thrust the air's state gives the rotor.
// The angle of attack and the sideslip are against the CASE's flow direction (the run holds the drone and the air
// moves past it), not a measurement in the solved flow. The aero loads on the body are not shown: they wait on
// Core's stage 7 batch E (T12). The panel says all three.
// ---------------------------------------------------------------------------
#pragma once

#include <algorithm>
#include <cmath>
#include <format>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "drone_sim.hpp"  // attitude_from_quat: the readouts' yaw, pitch and roll; vehicles::kStandardGravity

namespace spade::sandbox::editor {

struct FlightInput {
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};  // design axes -> world
    glm::vec3 omega_body{0.0f};                // rad/s, design axes
    glm::vec3 ambient{0.0f};                   // the case's air velocity past the drone, world axes (ResolvedCase::domain.ambient)
    float mass = 0.0f;                         // kg
    std::span<const float> rotor_omega;        // rad/s, the model's rotor order
    std::span<const float> thrust_coeff;       // k_T, N s^2, the same order
};

struct RotorReadout {
    float rpm = 0.0f;
    float thrust_n = 0.0f;  // k_T w^2: the model's static curve
};

struct FlightReadout {
    bool flow = false;  // the air moves past the drone: false in a calm hover, where the angles have no meaning
    float airspeed_mps = 0.0f;
    float aoa_deg = 0.0f;      // + the air meets the underside: the nose-up pitch against a head-on flow
    float sideslip_deg = 0.0f;  // + the air comes from the right of the nose
    std::vector<RotorReadout> rotors;
    float thrust_n = 0.0f;
    float weight_n = 0.0f;
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    glm::vec3 rates_deg_s{0.0f};
};

[[nodiscard]] inline FlightReadout flight_readout(const FlightInput& in) {
    constexpr float kRadToDeg = 57.29577951f;
    constexpr float kRadPerSToRpm = 9.54929659f;  // 60 / 2 pi
    FlightReadout r;
    // The vehicle moves through the air at -ambient; in the drone's axes (x the nose, y up, z the right):
    // AoA = atan2(-v_y, v_x), + when the air meets the underside; sideslip = asin(v_z / V), + from the right.
    const float airspeed = glm::length(in.ambient);
    r.airspeed_mps = airspeed;
    r.flow = airspeed > 0.0f;
    if (r.flow) {
        const glm::vec3 v = glm::conjugate(in.orient) * (-in.ambient);
        r.aoa_deg = std::atan2(-v.y, v.x) * kRadToDeg;
        r.sideslip_deg = std::asin(std::clamp(v.z / airspeed, -1.0f, 1.0f)) * kRadToDeg;
    }
    for (std::size_t i = 0; i < in.rotor_omega.size(); ++i) {
        RotorReadout rr;
        rr.rpm = in.rotor_omega[i] * kRadPerSToRpm;
        if (i < in.thrust_coeff.size()) rr.thrust_n = in.thrust_coeff[i] * in.rotor_omega[i] * in.rotor_omega[i];
        r.thrust_n += rr.thrust_n;
        r.rotors.push_back(rr);
    }
    r.weight_n = in.mass * vehicles::kStandardGravity;
    const AttitudeTarget a = attitude_from_quat(in.orient);
    r.yaw_deg = a.yaw * kRadToDeg;
    r.pitch_deg = a.pitch * kRadToDeg;
    r.roll_deg = a.roll * kRadToDeg;
    r.rates_deg_s = in.omega_body * kRadToDeg;
    return r;
}

struct FlightRow {
    std::string label;
    std::string value;
};

// What the panel shows, in order, and the sentence that names what it does not.
// "{:.1f}" without the sign of a zero: a held drone's yaw is a hair under 0, and "-0.0" reads as a fault. `plus` puts
// a + on a value at or above zero.
[[nodiscard]] inline std::string tenths(float v, bool plus = false) {
    std::string s = std::format("{:.1f}", v);
    if (s == "-0.0") s = "0.0";
    if (plus && s.front() != '-') s.insert(s.begin(), '+');
    return s;
}

[[nodiscard]] inline std::vector<FlightRow> flight_rows(const FlightReadout& r) {
    constexpr const char* kDeg = "\xC2\xB0";
    constexpr const char* kAgainst = " (against the case's flow)";
    std::vector<FlightRow> rows;
    rows.push_back({"Airspeed", std::format("{:.1f} m/s", r.airspeed_mps)});
    if (r.flow) {
        rows.push_back({std::string("Angle of attack") + kAgainst, tenths(r.aoa_deg, true) + kDeg});
        rows.push_back({std::string("Sideslip") + kAgainst, tenths(r.sideslip_deg, true) + kDeg});
    } else {
        rows.push_back({"Angle of attack", "none: no relative wind"});
        rows.push_back({"Sideslip", "none: no relative wind"});
    }
    for (std::size_t i = 0; i < r.rotors.size(); ++i) {
        rows.push_back({std::format("Rotor {}, static curve", i + 1),
                        std::format("{:.0f} rpm, {:.2f} N", r.rotors[i].rpm, r.rotors[i].thrust_n)});
    }
    rows.push_back({"Thrust, the model's static curve (k_T w^2)",
                    std::format("{:.2f} of {:.2f} N weight", r.thrust_n, r.weight_n)});
    rows.push_back({std::string("Yaw, pitch, roll (") + kDeg + ")",
                    tenths(r.yaw_deg) + ", " + tenths(r.pitch_deg) + ", " + tenths(r.roll_deg)});
    rows.push_back({std::string("Rates x, y, z (") + kDeg + "/s)",
                    tenths(r.rates_deg_s.x) + ", " + tenths(r.rates_deg_s.y) + ", " + tenths(r.rates_deg_s.z)});
    return rows;
}

// The note under the rows, wrapped: what the numbers are, and what is not shown.
inline constexpr const char* kFlightNotShown =
    "Thrust is the model's static curve (k_T w^2), not what the air's state gives the rotor; the angles are against "
    "the case's flow direction, not measured in the solved flow. Not shown: the aero loads on the body, which wait on "
    "Core's stage 7 (held-load rows).";

}  // namespace spade::sandbox::editor
