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

#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace spade::editor {

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

[[nodiscard]] inline FlightReadout flight_readout(const FlightInput&) { return {}; }

struct FlightRow {
    std::string label;
    std::string value;
};

// What the panel shows, in order, and the sentence that names what it does not.
[[nodiscard]] inline std::vector<FlightRow> flight_rows(const FlightReadout&) { return {}; }
inline constexpr const char* kFlightNotShown = "";

}  // namespace spade::editor
