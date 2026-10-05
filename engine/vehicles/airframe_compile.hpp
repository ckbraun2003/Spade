#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "physics/forces.hpp"  // drag_mode
#include "vehicles/composite_inertia.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_steady.hpp"

// ===========================================================================
// airframe_compile.hpp -- a part-built airframe to a ModelType (drone-builder
// physics DBP-44; docs/design/physics/plans/2026-10-03-airframe-compile-plan.md).
//
// GRADE: BEST-EFFORT. The rotor in the compiled model is today's momentum-
// theory RotorElement with a first-order lag, FITTED to the motor, ESC,
// battery and propeller chain at hover:
//
//   k_T = C_T(0) rho D^4 / (4 pi^2)      k_Q = C_Q(0) rho D^5 / (4 pi^2)
//   tau = the chain's small-signal time constant at hover
//         (steady_state_time_constant)
//
// It matches the chain at hover and for small changes about it. Away from
// hover it departs: it has no current limit, no battery sag, no inductance
// droop and no inflow dependence of torque. It is an APPROXIMATION that
// stands in until the stateful motor and battery rows land
// (2026-10-03-propulsion-rows-plan.md); the mass, inertia and mounts are not
// approximate.
//
// THE FRAMES. Everything in an AirframeSpec is in the airframe's DESIGN frame.
// The compiled ModelType is in its principal BODY frame, with
// design_to_principal (q_bd) and com_offset (c) saying how the two relate
// (spec section 6). Every mount is converted here, and nowhere else:
//
//   r_b = R_bd (p_d - c)          q_b = q_bd * q_d
//
// So an IMU authored with the identity orientation reads in design axes.
//
// DETERMINISM. Double arithmetic in a fixed order, sqrt the only
// non-arithmetic call (IEEE 754 rounds it correctly), fixed iteration counts,
// and one float rounding per output: the same spec gives a bit-identical
// ModelType on every conforming platform (Kat's question 1).
//
// INPUT. Spade's own struct, filled by the caller: the part blocks carry the
// field names and units of the spec's section 14.2. Spade never reads a
// builder's files.
//
// REFUSALS. check_airframe() returns every problem the compile's own checks
// and the parts' checks find, as a list, one issue per fault. The model's own
// rules (name, version, proxy radius, spin, drag elements, IMUs and every
// pose) belong to ModelType::validate, which reports its first problem as one
// issue of kind "model" (TD-9: one owner per rule). compile_airframe() runs
// the same checks and, on any problem, returns them joined in its error.
// ===========================================================================

namespace spade::vehicles {

// Where a derived quantity came from (Kat's question 2).
enum class Provenance : uint8_t {
    from_parts,  // exact from the parts: mass, inertia, centre of mass, mounts
    fitted,      // fitted from the chain: the momentum rotor's k_T, k_Q and tau
    estimated,   // a best-effort estimate: drag from parts, a propeller's inertia from its mass
    given,       // supplied by the caller as is
};

// A pose in the design frame: local -> design.
struct MountPose {
    glm::dvec3 position{0.0};                    // m
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};  // normalized here
};

// An optional shape for a block's mass, in the block's local frame. Without
// one, a block is a point mass, except a propeller, which is a thin disc.
struct BlockShape {
    PartShape shape = PartShape::point;
    glm::dvec3 size{0.0};
    glm::dmat3 tensor{0.0};
};

// The spec's section 14.2 blocks. SI units, except `kv` in the datasheet's
// rpm/V. Each has its mass; the motor and propeller sit at every rotor's hub,
// the ESC and battery at their own mounts.
struct MotorBlock {
    double kv = 0.0;               // rpm/V
    double resistance = 0.0;       // ohm, line to line
    double no_load_current = 0.0;  // A; held constant (the model has no speed-dependent loss)
    double no_load_voltage = 0.0;  // V, recorded only
    double current_max = 0.0;      // A, continuous
    double pole_pairs = 0.0;       // 0 drops the inductance term
    double inductance = 0.0;       // H, line to line; 0 drops the inductance term
    double rotor_inertia = 0.0;    // kg m^2, bell and shaft
    std::string stator;            // identity only
    double mass = 0.0;             // kg, per motor; 0 means "counted elsewhere"
    std::optional<BlockShape> shape;
};

struct PropBlock {
    double diameter = 0.0;   // m
    double pitch = 0.0;      // m, identity only
    uint32_t blades = 0;     // identity only
    double inertia = 0.0;    // kg m^2 about the shaft; 0 -> estimated as m D^2 / 12
    double ct_static = 0.0;  // C_T at J = 0; used when the tables are empty
    double cq_static = 0.0;  // C_Q at J = 0; used when the tables are empty
    std::vector<std::array<double, 2>> ct_table;  // [J, C_T], UIUC convention
    std::vector<std::array<double, 2>> cq_table;  // [J, C_Q]
    double mass = 0.0;       // kg, per propeller
    std::optional<BlockShape> shape;
};

struct EscBlock {
    double current_continuous = 0.0;  // A per channel; recorded (the model has no thermal state)
    double current_burst = 0.0;       // A per channel; the motor's current clamp is the smaller rating
    double current_total = 0.0;       // A, this board's total drive current (DBP-26); 0 for none
    uint32_t channels = 1;            // >= 1; with a total, it must equal the rotor count (one board for all)
    double on_resistance = 0.0;       // ohm, adds to the motor's resistance
    bool braking = false;             // active braking: current may reverse
    std::string protocol;             // identity only
    double mass = 0.0;                // kg
    MountPose mount;
    std::optional<BlockShape> shape;
};

struct BatteryBlock {
    uint32_t cells_series = 0;
    uint32_t cells_parallel = 0;
    double cell_capacity = 0.0;            // Ah
    double cell_resistance = 0.0;          // ohm
    double cell_voltage_nominal = 0.0;     // V, identity only
    double cell_voltage_full = 0.0;        // V, identity only
    double cell_voltage_cutoff = 0.0;      // V
    std::vector<std::array<double, 2>> ocv_table;  // [SoC, V] per cell
    double c_rating = 0.0;                 // 1/h, > 0; the pack's current limit is c_rating x capacity
    double polarization_resistance = 0.0;  // ohm, pack R1
    double polarization_capacitance = 0.0; // F, pack C1 (the stateful rows use it; the fit does not)
    double mass = 0.0;                     // kg
    MountPose mount;
    std::optional<BlockShape> shape;
};

// One rotor: its hub, thrust along local +Y, and its spin (+1, -1 or 0, as
// RotorDesc::spin_dir). Declaration order is rotor order.
struct RotorSpec {
    MountPose hub;
    double spin_dir = 1.0;
};

// A drag element, as DragBodyDesc but in the design frame. Componentwise
// coefficients run along the MOUNT's axes; the drag law applies them along
// body axes, so the compile maps them there: exactly when the mount's axes
// land on body axes (an axis-aligned mount in an untilted principal frame),
// and otherwise by the on-axis projection c_i = sum_j |M_ij|^3 c_j, which drops
// the coupling between axes and makes the drag provenance `estimated`.
struct DragSpec {
    uint32_t mode = physics::drag_mode::componentwise;
    double area = 0.0;
    glm::dvec3 coeffs{0.0};
    MountPose mount;
};

// An IMU, as ImuMountDesc but in the design frame.
struct ImuSpec {
    MountPose mount;
    uint32_t rate_divider = 1;
    double sigma_a = 0.0;
    double sigma_g = 0.0;
    double sigma_ba = 0.0;
    double sigma_bg = 0.0;
};

struct AirframeSpec {
    std::string name;               // non-empty
    uint32_t param_schema_id = 0;
    std::string visual_ref;
    uint32_t version = 1;           // >= 1

    std::vector<PartInertia> parts; // frame, arms, avionics: every mass not in a block below
    MotorBlock motor;               // one motor type, at every hub
    PropBlock prop;                 // one propeller type, at every hub
    EscBlock esc;
    BatteryBlock battery;
    std::vector<RotorSpec> rotors;  // at least one
    std::vector<DragSpec> drag;     // empty: estimated from the parts (best-effort)
    std::vector<ImuSpec> imus;
    double proxy_radius = 0.0;      // m, >= 0

    // The fit's operating point: hover in still air.
    double air_density = 1.225;     // kg/m^3
    double gravity = 9.80665;       // m/s^2
    double state_of_charge = 1.0;   // [0, 1]
};

// One problem: what kind of entry ("airframe", "parts", "motor", "prop",
// "esc", "battery", "rotors", "drag", "model"), its index within that kind (0
// for the single blocks), the field and a message. A block's shape is
// "<block>[0].shape"; a bad pose at a rotor's hub is "rotors[k].hub", once.
struct AirframeIssue {
    Code code = Code::invalid_argument;
    std::string kind;
    std::size_t index = 0;
    std::string field;
    std::string message;
};

struct AirframeFit {
    double hover_duty = 0.0;
    double hover_omega = 0.0;       // rad/s
    double hover_thrust = 0.0;      // N per rotor (weight / rotor count)
    double thrust_coeff = 0.0;      // k_T, N s^2
    double torque_coeff = 0.0;      // k_Q, N m s^2
    double tau = 0.0;               // s
    double rotor_inertia = 0.0;     // J_r, kg m^2
    uint32_t solver_flags = 0;      // propulsion_flags at hover
    Provenance mass_and_inertia = Provenance::from_parts;
    Provenance mounts = Provenance::from_parts;
    Provenance rotor = Provenance::fitted;
    Provenance rotor_inertia_source = Provenance::given;
    Provenance drag = Provenance::given;
    Provenance proxy_radius = Provenance::given;
    Provenance sensors = Provenance::given;
};

struct CompiledAirframe {
    ModelType model;
    CompositeInertia inertia;
    AirframeFit fit;
};

// The propulsion chain the blocks describe, as the steady-state solver and
// the fit use it: SI conversion, tables resampled (33 points), the motor's
// resistance plus the ESC's, the smaller current rating, braking, the pack's
// limits. invalid_argument if a block is unusable; check_airframe() lists
// each field.
[[nodiscard]] Result<PropulsionChain> airframe_propulsion_chain(const AirframeSpec& spec);

// Every problem with the spec, as a list; empty when compile_airframe() will
// succeed.
[[nodiscard]] std::vector<AirframeIssue> check_airframe(const AirframeSpec& spec);

// The compile. On any problem, invalid_argument with every issue, joined.
[[nodiscard]] Result<CompiledAirframe> compile_airframe(const AirframeSpec& spec);

}  // namespace spade::vehicles
