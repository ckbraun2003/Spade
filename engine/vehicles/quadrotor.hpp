#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "vehicles/model_type.hpp"

// ===========================================================================
// THE QUADROTOR -- v2's first vehicle class (engine design D4: "model-type
// layer + Quadrotor"; spec S6).
//
// make_quadrotor() is a CONSTRUCTOR FOR A ModelType, nothing more: it turns
// the dozen numbers a quadrotor is actually specified by (mass, inertia, arm
// length, four motor/prop calibrations) into the general element descriptions
// the model-type layer already understands, and validates the result. There is
// no Quadrotor class, no quadrotor-shaped code path in the Simulation, and no
// quadrotor branch in any pass. If this file were deleted, the engine would
// still fly quadrotors -- a caller would just have to spell out the four
// RotorDescs itself.
//
// ---------------------------------------------------------------------------
// 1. THE GEOMETRY: A PLUS CONFIGURATION IN A Y-UP FRAME
//
// Spade is Y-up (world/builder.hpp), so an airframe's rotor plane lies in the
// body XZ plane and its thrust points along body +Y. The four arms lie ALONG
// THE BODY AXES -- a "plus" airframe, not an "X" one:
//
//                          +Z
//                           |
//                        [1] (-)
//                           |
//              (+) [2] ---- o ---- [0] (+)     -> +X
//                           |
//                        [3] (-)
//                           |
//
//     index   arm offset (body frame)      spin_dir
//       0     (+L, hz,  0)                   +1
//       1     ( 0, hz, +L)                   -1
//       2     (-L, hz,  0)                   +1
//       3     ( 0, hz, -L)                   -1
//
// L is `arm_length` and hz is `rotor_height` -- the rotor plane's offset ABOVE
// the COM, which is what a real airframe has and what makes the thrust line
// not pass through the COM. (It does not affect the moments of a level hover:
// cross(r, T*+Y) is (-rz*T, 0, rx*T), in which ry does not appear. It matters
// the moment the vehicle is not level, which is every moment that matters.)
//
// WHY PLUS AND NOT X. Two reasons, one practical and one honest. Practically,
// a plus airframe makes each control differential act on EXACTLY ONE body axis
// (section 2), which is what makes the axis assignment checkable rather than
// merely plausible. Honestly: an X airframe is this same model rotated 45
// degrees about +Y, so supporting both would be one more parameter and one
// more thing to get wrong, and nothing in v2 needs it. A caller who wants an X
// quad today builds the ModelType directly -- that door is open by
// construction, because make_quadrotor() has no privileges.
//
// ---------------------------------------------------------------------------
// 2. THE CONTROL MAPPING, DERIVED RATHER THAN ASSERTED
//
// With the airframe level, rotor i produces thrust T_i along body +Y at
// r_i, and a reaction torque -spin_i * Q_i along body +Y (rotor.hpp section
// 6). vehicles/rotor.cpp accumulates exactly
//
//     torque_body += cross(r_i, T_i * +Y) - spin_i * Q_i * (+Y)
//                  = (-r_iz * T_i,  -spin_i * Q_i,  r_ix * T_i)
//
// so, summing the four rows of the table above,
//
//     M_x = L * (T_3 - T_1)                      <- the 1/3 pair alone
//     M_y = -(Q_0 - Q_1 + Q_2 - Q_3)             <- the signed reaction sum
//     M_z = L * (T_0 - T_2)                      <- the 0/2 pair alone
//
// Three consequences, and each is a test in test_quadrotor.cpp:
//
//   * COLLECTIVE. All four equal: M_x = M_z = 0 exactly (equal terms cancel
//     bit-for-bit in fp32, not approximately), and M_y = 0 because the +-+-
//     layout pairs equal Q's with opposite signs. Total thrust 4 k_T w^2 along
//     body +Y. That is `hover_command`'s entire premise.
//   * ROLL / PITCH. A differential inside the 1/3 pair moves M_x ALONE; a
//     differential inside the 0/2 pair moves M_z ALONE. With the nose along
//     body +X those are roll and pitch respectively.
//   * YAW comes from Q, NOT FROM T. Raising the +1 diagonal (0 and 2) and
//     lowering the -1 diagonal (1 and 3) leaves M_x = M_z = 0 -- each pair is
//     still internally matched -- and drives M_y NEGATIVE, because Q_0 and Q_2
//     have grown while Q_1 and Q_3 have shrunk and the layout's signs are
//     (+, -, +, -). That is the +-+- layout's whole purpose.
//
// THE YAW AUTHORITY IS UNCORRECTED, AND THAT IS THE MODEL, NOT A BUG.
// rotor.hpp is explicit that f_inflow and f_ground scale the THRUST ONLY, so
// Q = k_Q w^2 exactly, whatever the flight state and however close the ground.
// A yaw prediction from this model is therefore an exact function of the shaft
// speeds alone -- which is why test_quadrotor.cpp can pin M_y analytically
// while it can only pin M_x/M_z's SIGN and axis. It also means: DO NOT read
// this vehicle's T and Q as a consistent shaft-power budget (rotor.hpp's
// second scope boundary says the same thing one layer down).
//
// ---------------------------------------------------------------------------
// 3. WHAT THIS FILE DOES NOT MODEL
//
// No motor electrical model (k_T and k_Q are the whole propulsion
// calibration), no battery sag, no rotor gyroscopic reaction (rotor.hpp
// section 6 puts it on the roadmap with its reserved lane), no airframe
// flexibility, no propeller-airframe interaction, and no mixer: a quadrotor
// here takes FOUR SHAFT SPEED COMMANDS, and turning (collective, roll, pitch,
// yaw) into four numbers is the controller's job, one layer up. The two
// helpers this file does export -- hover_command() and the arm offsets --
// exist because they are properties of the AIRFRAME, computable from
// QuadrotorParams alone, and because a test or a bring-up scenario that had to
// re-derive them would be re-deriving them wrong.
// ===========================================================================

namespace spade::vehicles {

// Every quadrotor has four rotors. Spelled once so a loop bound and an array
// extent cannot disagree.
inline constexpr std::size_t kQuadrotorRotorCount = 4;

// The +-+- spin layout, in index order (= the order the arms are visited going
// around the ring, see the diagram). Diagonally OPPOSITE rotors therefore
// co-rotate and ADJACENT ones counter-rotate, which is what makes a quadrotor
// yaw-neutral in the collective and yaw-authoritative in the diagonal
// differential.
//
// PINNED, and exported so a test can compare the built model against this
// array rather than against a transcription of it.
inline constexpr std::array<float, kQuadrotorRotorCount> kQuadrotorSpinLayout{1.0f, -1.0f, 1.0f, -1.0f};

// Standard gravity, m/s^2 -- the same constant world/builder.hpp's Environment
// defaults to. hover_command()'s default argument, so a caller flying in a
// non-standard field passes its own magnitude rather than getting a silently
// mistrimmed vehicle.
inline constexpr float kStandardGravity = 9.80665f;

// ---------------------------------------------------------------------------
// One rotor's propulsion calibration. The geometry (where the rotor is, which
// way it spins) comes from the airframe below; this is the motor and the prop.
// Units are RotorDesc's, which are RotorRow's.
// ---------------------------------------------------------------------------
struct RotorParams {
    float tau = 0.02f;           // RPM-lag time constant, s; 0 means no lag
    float radius = 0.12f;        // disc radius R, m; MUST be > 0
    float thrust_coeff = 0.0f;   // k_T in T_static = k_T w^2, N s^2; > 0
    float torque_coeff = 0.0f;   // k_Q in Q = k_Q w^2, N m s^2; >= 0
};

// ---------------------------------------------------------------------------
// The airframe. Everything make_quadrotor() needs and nothing it does not.
// ---------------------------------------------------------------------------
struct QuadrotorParams {
    std::string name = "quadrotor";
    uint32_t param_schema_id = 0;
    std::string visual_ref;

    float mass = 1.0f;                    // kg, > 0
    glm::vec3 inertia_diag{1.0f};         // body-frame PRINCIPAL moments, kg m^2, componentwise > 0
    float arm_length = 0.15f;             // L: COM to rotor hub, in the body XZ plane, m, > 0
    float rotor_height = 0.0f;            // hz: rotor plane above the COM, body +Y, m (any sign)
    float proxy_radius = 0.0f;            // m, >= 0; CONSUMED as of D-S6-2 (model_type.hpp)

    std::array<RotorParams, kQuadrotorRotorCount> rotors{};

    // The spin layout, defaulted to +-+-. A FIELD rather than a constant
    // because the brief names it as one and because a mis-wired airframe is a
    // thing a bring-up scenario legitimately wants to reproduce; validated to
    // +1 / -1 / 0 per entry by ModelType::validate(). The default is the only
    // layout that cancels in the collective -- nothing here enforces that, so
    // a caller who changes it owns the consequence.
    std::array<float, kQuadrotorRotorCount> spin_dirs = kQuadrotorSpinLayout;

    // The airframe's bluff-body drag. A quadrotor carries exactly one, at the
    // COM: rotor.hpp is explicit that the disc's own drag is NOT the rotor
    // element's job ("gross body drag belongs to DragBody ... double-counting
    // it here would make the element's own regime boundaries depend on how the
    // vehicle was assembled"), so this element is where it lives.
    DragBodyDesc drag{};

    // The one IMU, at its mount. Defaults to an ideal sensor at the COM
    // sampling every substep (sensors/imu.hpp's default posture).
    ImuMountDesc imu{};
};

// ---------------------------------------------------------------------------
// The body-frame hub offset of rotor `index`, from the airframe's arm length
// and rotor height. Section 1's table, as a function.
//
// Returns (0, 0, 0) for an index at or above kQuadrotorRotorCount -- total,
// like everything else in this layer, and unreachable for a caller iterating
// the four rotors it declared.
// ---------------------------------------------------------------------------
[[nodiscard]] glm::vec3 quadrotor_arm_offset(const QuadrotorParams& params, std::size_t index) noexcept;

// ---------------------------------------------------------------------------
// make_quadrotor -- the airframe, as a ModelType.
//
// Builds four RotorDescs from the arm layout and the per-rotor calibrations,
// the one drag body, the one IMU mount, and the body template (inverting
// nothing -- BodyTemplate carries the inertia, and the spawn path inverts it),
// then runs ModelType::validate() and returns its error verbatim on failure.
//
// So a ModelType that comes out of this function is valid BY CONSTRUCTION, and
// the only failures a caller sees are its own parameters: a non-positive mass,
// inertia, arm length or rotor radius; a spin_dirs entry that is not +1/-1/0;
// a non-finite anything. Codes: invalid_argument, always.
// ---------------------------------------------------------------------------
[[nodiscard]] Result<ModelType> make_quadrotor(const QuadrotorParams& params);

// ---------------------------------------------------------------------------
// hover_command -- the common shaft speed w_h at which this airframe's four
// rotors produce exactly its own weight.
//
//     sum_i k_T,i * w_h^2 = m * g     =>     w_h = sqrt(m * g / sum_i k_T,i)
//
// EXACT AT HOVER, BY CONSTRUCTION AND NOT BY APPROXIMATION. rotor.hpp's inflow
// closure is normalized so that lambda(0) = 1 -- "the thrust curve T(w) is
// used as exactly what it is measured as, a static/hover thrust curve" -- and
// Cheeseman-Bennett's f_ground is exactly 1 out of ground effect. So a
// stationary, level quadrotor far from geometry, in still air, holding this
// command, is in exact vertical trim; the only residual is the rounding of the
// square root itself (test_quadrotor.cpp derives the resulting drift bound and
// measures against it).
//
// THE TWO THINGS IT DOES NOT KNOW, stated because both are silent:
//   * IN GROUND EFFECT IT UNDER-COMMANDS. f_ground reaches 1.25 near the
//     floor, so this speed lifts a hovering quadrotor that gets close to it.
//     That is the physics, not an error in the formula.
//   * FOR AN ASYMMETRIC AIRFRAME IT IS A COLLECTIVE, NOT A TRIM. The sum makes
//     the TOTAL thrust right, but four unequal k_T at one common speed do not
//     cancel in M_x/M_z, so such a vehicle lifts and rolls. Symmetric
//     airframes -- what this helper is for -- have no such term.
//
// TOTAL, like every scalar helper in this layer: returns 0 ("no hover speed
// exists") for a non-positive or non-finite mass, gravity or thrust-coefficient
// sum, none of which a validated QuadrotorParams can present.
// ---------------------------------------------------------------------------
[[nodiscard]] float hover_command(const QuadrotorParams& params,
                                  float gravity_magnitude = kStandardGravity) noexcept;

}  // namespace spade::vehicles
