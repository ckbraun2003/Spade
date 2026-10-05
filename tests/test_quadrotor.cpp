#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "physics/integrator.hpp"
#include "sensors/imu.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "vehicles/rotor.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// Task 18 -- the model-type layer and the Quadrotor.
//
// FIVE BURDENS OF PROOF, and the shape of each is chosen so that the expected
// answer is DERIVED rather than recorded:
//
//   * THE AIRFRAME IS THE AIRFRAME IT CLAIMS TO BE. make_quadrotor()'s four
//     RotorDescs are compared, one at a time, against quadrotor.hpp section
//     1's table -- position AND spin, per index, ASYMMETRICALLY. A layout test
//     that only checked "two +1s and two -1s" would pass a model whose arms
//     were permuted, which is exactly the bug that makes a controller's roll
//     command yaw the vehicle.
//
//   * THE MOMENTS LAND ON THE RIGHT AXES, WITH THE RIGHT SIGNS AND THE RIGHT
//     MAGNITUDES. Every moment test measures ONE substep from EXACT REST in an
//     EMPTY world, which is the configuration in which the whole rotor chain
//     collapses to arithmetic a test can redo in three lines: at v = 0 the
//     inflow factor is exactly 1 (rotor.hpp: lambda(0) = 1 by construction),
//     an empty SDF program reports kSdfEmptyDistance so the ground factor is
//     exactly 1, and a rotor with tau = 0 tracks its command within the
//     substep. So the expected angular velocity is a closed form, not a
//     tolerance around a recorded number.
//
//   * HOVER IS ACTUALLY HOVER. 10,000 substeps at 1 kHz -- ten seconds, during
//     which an unpowered body falls 490 m -- inside 5 cm and half a degree.
//     The residual is derived below rather than discovered.
//
//   * THE INFLOW CORRECTION IS REALLY IN THE LOOP. The collective-step test
//     tracks the measured climb acceleration against a reference built from
//     Task 17's OWN exported functions (rotor_hover_induced_velocity,
//     rotor_inflow_factor, rotor_ground_factor) fed the vehicle's actual
//     state, and asserts that the correction has moved off 1 by a margin
//     bigger than the tolerance -- otherwise the "reference" would be checking
//     nothing but multiplication by one.
//
//   * A VEHICLE IS STATE, NOT AN OBJECT. Spawn/despawn round trips are
//     digest-identical run to run, a despawn releases every rotor, drag and
//     sensor slot it took, and a snapshot restored into a FRESH Simulation --
//     one that has registered the same model (configuration, SCN-006) but
//     never spawned anything -- keeps flying on the restored shaft speeds and
//     commands.
//
// WHY THE MOMENT TESTS USE tau = 0 AND THE HOVER TESTS DO NOT. tau = 0 is
// rotor.hpp's documented "no lag" degenerate (alpha = 1), and it is what turns
// a one-substep measurement into a closed form. It is used ONLY where the
// quantity under test is a moment. The hover and climb tests carry a realistic
// 20 ms time constant, because there the lag is part of what is being trusted.
// ---------------------------------------------------------------------------

namespace {

using spade::BodyRef;
using spade::Capacities;
using spade::Environment;
using spade::ModelTypeId;
using spade::Simulation;
using spade::SnapshotBlob;
using spade::TurbulenceLevel;
using spade::VehicleRef;
using spade::VehicleSpawn;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::sensors::ImuSample;
using spade::sensors::kRingDepth;
using spade::vehicles::ModelType;
using spade::vehicles::QuadrotorParams;
using spade::vehicles::RotorParams;
using spade::vehicles::kQuadrotorRotorCount;

// GoogleTest plumbing for spade::Result -- prints the Error's context on
// failure. Same shape as test_determinism.cpp's and test_imu.cpp's; macros are
// per-TU, so the definitions do not collide.
template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code)
                                       << "] " << r.error().context;
}

#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// The error code of a Result that is EXPECTED to have failed, with -1 standing
// for "it succeeded" -- calling .error() on a Result holding a value is UB.
template <class T>
[[nodiscard]] int code_of(const spade::Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}

[[nodiscard]] constexpr int code(spade::Code c) { return static_cast<int>(c); }

// The engine is Y-UP and Environment defaults gravity to (0, -9.80665, 0).
constexpr float kG = 9.80665f;

// ===========================================================================
// The test airframe.
//
// A 1 kg quadrotor with 12 cm props on 15 cm arms -- roughly a 5-inch racing
// frame, and deliberately a REAL set of numbers rather than round ones, so
// that no assertion below can accidentally hold because two quantities were
// both 1.
//
//   k_T = 1e-5 N s^2   =>  hover shaft speed sqrt(mg / 4k_T) = 495.1 rad/s
//                          (4728 rpm), per-rotor thrust 2.4517 N
//   k_Q = 1.6e-7 N m s^2 => hover reaction torque 0.0392 N m per rotor,
//                          i.e. a torque/thrust ratio of 16 mm -- the right
//                          order for a prop of this size.
//
// THE THREE PRINCIPAL MOMENTS ARE DELIBERATELY DISTINCT (and Iy is the largest,
// as it is for a flat airframe). Equal moments would let an X/Z axis swap pass
// every moment test in this file.
// ===========================================================================

constexpr float kMass = 1.0f;
constexpr float kIx = 0.012f;
constexpr float kIy = 0.021f;
constexpr float kIz = 0.016f;
constexpr float kArm = 0.15f;
constexpr float kRotorHeight = 0.02f;
constexpr float kRadius = 0.12f;
constexpr float kThrustCoeff = 1.0e-5f;
constexpr float kTorqueCoeff = 1.6e-7f;
constexpr float kAirDensity = 1.225f;

// `lag`: the rotor time constant. 0 means "no lag" -- rotor.hpp section 5's
// documented degenerate, which is what makes a one-substep moment measurement
// a closed form. `drag_cd`: 0 leaves the drag element present but inert, so a
// climb measurement is the rotor chain alone.
[[nodiscard]] QuadrotorParams test_quad(float lag, float drag_cd) {
    QuadrotorParams params;
    params.name = "test_quad";
    params.param_schema_id = 7;
    params.visual_ref = "meshes/test_quad";
    params.mass = kMass;
    params.inertia_diag = glm::vec3(kIx, kIy, kIz);
    params.arm_length = kArm;
    params.rotor_height = kRotorHeight;
    params.proxy_radius = 0.18f;
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
        RotorParams& rotor = params.rotors[i];
        rotor.tau = lag;
        rotor.radius = kRadius;
        rotor.thrust_coeff = kThrustCoeff;
        rotor.torque_coeff = kTorqueCoeff;
    }
    params.drag.mode = spade::physics::drag_mode::quadratic;
    params.drag.area = 0.02f;
    params.drag.coeffs = glm::vec3(drag_cd, 0.0f, 0.0f);
    params.imu.rate_divider = 1;  // every substep, ideal sensor
    return params;
}

// ---------------------------------------------------------------------------
// World construction. EVERY world here has NO SDF NODES AT ALL: an empty
// program evaluates to kSdfEmptyDistance (world/sdf.hpp), so CollisionStatic
// finds nothing and -- the part every derivation below leans on -- the
// ground-effect factor is exactly 1. That claim is not assumed; the first test
// in this file computes rotor_ground_factor(kSdfEmptyDistance, R) and pins it.
// ---------------------------------------------------------------------------

[[nodiscard]] Environment default_environment() {
    Environment env;
    env.gravity = glm::vec3(0.0f, -kG, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = kAirDensity;
    return env;
}

[[nodiscard]] Capacities capacities(uint32_t bodies, uint32_t elements, uint32_t sensors) {
    Capacities caps;
    caps.bodies = bodies;
    caps.force_elements = elements;
    caps.sensors = sensors;
    caps.contacts = 1;  // declared, unused: contacts are not an arena array in v1
    return caps;
}

[[nodiscard]] spade::Result<WorldSetDesc> void_world(uint32_t bodies = 4, uint32_t elements = 8,
                                                     uint32_t sensors = 2) {
    const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                      .name("void")
                                                      .environment(default_environment())
                                                      .capacities(capacities(bodies, elements, sensors))
                                                      .build();
    if (!world) return std::unexpected(world.error());

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0x0DAD4070ULL;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.1f;
    instance.grid.cell_size = 0.5f;
    return WorldSetDesc{{instance}};
}

// A one-world Simulation on the void world, at 1 kHz with one substep per step
// -- so one step() IS one substep, which is what lets a test read a body's
// state between substeps.
[[nodiscard]] spade::Result<Simulation> void_sim(uint32_t bodies = 4, uint32_t elements = 8,
                                                 uint32_t sensors = 2) {
    const spade::Result<WorldSetDesc> desc = void_world(bodies, elements, sensors);
    if (!desc) return std::unexpected(desc.error());
    return Simulation::create(*desc, 1'000'000, 1);
}

// ---------------------------------------------------------------------------
// D-S6-2 fix-loop (I2+M3): a world WITH geometry, unlike void_world(), which
// deliberately has none. A single ground plane, normal +Y, offset 0, so
// phi(p) == p.y (world/sdf.cpp's plane eval is dot(p,n) - offset; test_
// contacts.cpp's GroundPlane() documents the same primitive on the Z axis --
// this file is Y-up, hence +Y here). `default_radius` and `cell_size` are the
// two knobs the vehicle-spawn precondition tests below vary.
// ---------------------------------------------------------------------------
[[nodiscard]] spade::Result<Simulation> ground_plane_sim(float default_radius, float cell_size) {
    const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                      .name("ground_plane")
                                                      .environment(default_environment())
                                                      .capacities(capacities(4, 8, 2))
                                                      .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                      .build();
    if (!world) return std::unexpected(world.error());

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0xC0FFEE2020ULL;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = default_radius;
    instance.grid.cell_size = cell_size;
    const WorldSetDesc desc{{instance}};
    return Simulation::create(desc, 1'000'000, 1);
}

// ---------------------------------------------------------------------------
// The analytic reference for ONE substep from EXACT REST in an empty world,
// re-derived here from quadrotor.hpp section 2 rather than transcribed from
// the implementation.
//
//   T_i = k_T w_i^2                    (f_inflow = 1 at v = 0; f_ground = 1)
//   Q_i = k_Q w_i^2                    (NEVER corrected -- rotor.hpp is
//                                       explicit that the inflow and ground
//                                       factors scale the THRUST only)
//   M  = sum_i ( -r_iz T_i,  -spin_i Q_i,  r_ix T_i )
//   dw_body = (M_x/I_x, M_y/I_y, M_z/I_z) * h        (omega starts at 0, so
//                                       the gyroscopic term is identically 0)
//
// Written with `1/I` rather than `M/I` to match integrate_bodies() step 4c,
// which multiplies by the stored inverse inertia.
// ---------------------------------------------------------------------------
[[nodiscard]] glm::vec3 expected_first_substep_omega(const QuadrotorParams& params,
                                                     std::span<const float> commands, float h) {
    glm::vec3 moment(0.0f);
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
        const float w_sq = commands[i] * commands[i];
        const float thrust = params.rotors[i].thrust_coeff * w_sq;
        const float torque = params.rotors[i].torque_coeff * w_sq;
        const glm::vec3 r = spade::vehicles::quadrotor_arm_offset(params, i);
        moment += glm::vec3(-r.z * thrust, -params.spin_dirs[i] * torque, r.x * thrust);
    }
    return glm::vec3(moment.x / params.inertia_diag.x, moment.y / params.inertia_diag.y,
                     moment.z / params.inertia_diag.z) *
           h;
}

// Spawns the airframe at rest in a fresh void world, applies `commands`, and
// steps exactly one substep. Returns the resulting body-frame angular
// velocity, which -- starting from omega = 0 -- IS the substep's dw_body.
[[nodiscard]] spade::Result<glm::vec3> measure_first_substep_omega(const QuadrotorParams& params,
                                                                   std::span<const float> commands) {
    spade::Result<Simulation> sim = void_sim();
    if (!sim) return std::unexpected(sim.error());
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
    if (!model) return std::unexpected(model.error());
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    if (!id) return std::unexpected(id.error());

    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 50.0f, 0.0f);
    where.rotor_omega = 0.0f;  // at rest: v = 0 and omega = 0 at the measured substep
    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    if (!vehicle) return std::unexpected(vehicle.error());
    if (spade::Result<void> r = sim->flush_structural(); !r) return std::unexpected(r.error());
    if (spade::Result<void> r = sim->set_rotor_commands(*vehicle, commands); !r) {
        return std::unexpected(r.error());
    }
    if (spade::Result<void> r = sim->step(1); !r) return std::unexpected(r.error());

    const spade::Result<const spade::BodyState*> body = sim->body(vehicle->body);
    if (!body) return std::unexpected(body.error());
    return (*body)->omega_body;
}

// "These two angular velocities agree", with an absolute floor so a component
// the derivation says is EXACTLY zero is compared against zero rather than
// against a relative tolerance of zero. 1e-9 rad/s is about six orders below
// the smallest non-zero component any test here produces.
void ExpectOmegaNear(const glm::vec3& measured, const glm::vec3& expected, float rel_tol,
                     const char* what) {
    for (int axis = 0; axis < 3; ++axis) {
        const float tol = std::max(1.0e-9f, rel_tol * std::fabs(expected[axis]));
        EXPECT_NEAR(measured[axis], expected[axis], tol)
            << what << ": axis " << axis << " (0=X, 1=Y, 2=Z)";
    }
}

// The angle of a rotation, radians -- the attitude-drift metric.
[[nodiscard]] float rotation_angle(const glm::quat& q) {
    // 2*acos(|w|), clamped: a unit quaternion's w can round a hair past 1.
    const float w = std::min(1.0f, std::fabs(q.w));
    return 2.0f * std::acos(w);
}

}  // namespace

// ===========================================================================
// 0. The two premises every derivation in this file rests on
// ===========================================================================

// Both aero corrections are EXACTLY 1 in the configuration the moment tests
// use -- an empty world, at rest. Pinned rather than assumed, because if either
// were merely CLOSE to 1 every "closed form" below would be a fit.
TEST(Quadrotor, InflowAndGroundFactorsAreExactlyOneAtRestInAnEmptyWorld) {
    // An empty SDF program reports kSdfEmptyDistance everywhere, and
    // Cheeseman-Bennett's (R/4z)^2 underflows fp32 to exactly 0 there.
    EXPECT_EQ(spade::vehicles::rotor_ground_factor(spade::kSdfEmptyDistance, kRadius), 1.0f);

    // lambda(0) = 1 is the closure's normalization: "the thrust curve T(w) is
    // used as exactly what it is measured as -- a static/hover thrust curve"
    // (rotor.hpp section 3).
    const float hover_speed = std::sqrt(kMass * kG / (4.0f * kThrustCoeff));
    const float v_hover = spade::vehicles::rotor_hover_induced_velocity(
        kThrustCoeff * hover_speed * hover_speed, kAirDensity, kRadius);
    EXPECT_GT(v_hover, 0.0f);
    EXPECT_EQ(spade::vehicles::rotor_inflow_factor(0.0f, v_hover), 1.0f);

    // A rotor with tau = 0 reaches its command within the substep, which is
    // what makes a one-substep measurement a closed form.
    EXPECT_EQ(spade::vehicles::rotor_lag_alpha(0.001f, 0.0f), 1.0f);

    // Every moment test below spells its substep as the literal 1.0e-3f. That
    // is only legitimate because the engine's own ns -> s conversion lands on
    // the same float; pinned here so it is a checked assumption, not one.
    const spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);
    EXPECT_EQ(sim->substep_h(), 1.0e-3f);
    EXPECT_EQ(sim->substeps(), 1u);
}

// ===========================================================================
// 1. make_quadrotor builds the airframe it documents
// ===========================================================================

// THE ASYMMETRIC PIN. Each index is checked against quadrotor.hpp section 1's
// table individually -- arm position and spin sign together -- so a permutation
// of the four rotors fails here rather than surfacing as a control-axis bug.
TEST(Quadrotor, MakeQuadrotorProducesThePlusLayoutWithThePlusMinusPlusMinusSpins) {
    const QuadrotorParams params = test_quad(0.02f, 0.9f);
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);

    ASSERT_EQ(model->rotors.size(), kQuadrotorRotorCount);
    ASSERT_EQ(model->drag_bodies.size(), 1u);
    ASSERT_EQ(model->imu_mounts.size(), 1u);
    EXPECT_EQ(model->name, "test_quad");
    EXPECT_EQ(model->param_schema_id, 7u);
    EXPECT_EQ(model->visual_ref, "meshes/test_quad");
    EXPECT_EQ(model->body.mass, kMass);
    EXPECT_EQ(model->body.inertia_diag, glm::vec3(kIx, kIy, kIz));
    EXPECT_EQ(model->proxy_radius, 0.18f);
    // Rotors and drag bodies share one budget: 4 + 1.
    EXPECT_EQ(model->force_element_count(), std::size_t{5});

    const glm::vec3 expected_pos[kQuadrotorRotorCount] = {
        glm::vec3(kArm, kRotorHeight, 0.0f),   // 0: +X arm
        glm::vec3(0.0f, kRotorHeight, kArm),   // 1: +Z arm
        glm::vec3(-kArm, kRotorHeight, 0.0f),  // 2: -X arm
        glm::vec3(0.0f, kRotorHeight, -kArm),  // 3: -Z arm
    };
    const float expected_spin[kQuadrotorRotorCount] = {1.0f, -1.0f, 1.0f, -1.0f};

    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
        EXPECT_EQ(model->rotors[i].local_pos, expected_pos[i]) << "rotor " << i << " arm position";
        EXPECT_EQ(model->rotors[i].spin_dir, expected_spin[i]) << "rotor " << i << " spin direction";
        // Identity mount: rotor.hpp's local thrust axis is +Y and Spade is
        // Y-up, so a level airframe thrusts straight up.
        EXPECT_EQ(model->rotors[i].local_orient, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) << "rotor " << i;
        EXPECT_EQ(model->rotors[i].radius, kRadius) << "rotor " << i;
        EXPECT_EQ(model->rotors[i].thrust_coeff, kThrustCoeff) << "rotor " << i;
        EXPECT_EQ(model->rotors[i].torque_coeff, kTorqueCoeff) << "rotor " << i;
    }

    // The exported layout constant and the built model must be the same thing.
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
        EXPECT_EQ(model->rotors[i].spin_dir, spade::vehicles::kQuadrotorSpinLayout[i]);
    }
    // Diagonals co-rotate, neighbours counter-rotate -- the property the
    // per-index table above encodes, stated once as the invariant it serves.
    EXPECT_EQ(model->rotors[0].spin_dir, model->rotors[2].spin_dir);
    EXPECT_EQ(model->rotors[1].spin_dir, model->rotors[3].spin_dir);
    EXPECT_NE(model->rotors[0].spin_dir, model->rotors[1].spin_dir);
}

TEST(Quadrotor, HoverCommandSolvesTotalStaticThrustEqualsWeight) {
    const QuadrotorParams params = test_quad(0.02f, 0.9f);
    const float hover = spade::vehicles::hover_command(params);

    // The closed form, spelled independently of quadrotor.cpp.
    const float expected = std::sqrt(kMass * kG / (4.0f * kThrustCoeff));
    EXPECT_NEAR(hover, expected, 1.0e-3f);

    // ... and what it MEANS: four rotors at this speed lift exactly the
    // airframe's weight, to within the rounding of the square root.
    const float total_thrust = 4.0f * kThrustCoeff * hover * hover;
    EXPECT_NEAR(total_thrust, kMass * kG, 1.0e-4f);

    // A non-standard gravity re-trims: the ratio is sqrt(g'/g).
    const float lunar = spade::vehicles::hover_command(params, 1.62f);
    EXPECT_NEAR(lunar / hover, std::sqrt(1.62f / kG), 1.0e-5f);

    // Total, like every scalar helper in this layer.
    QuadrotorParams degenerate = params;
    degenerate.mass = 0.0f;
    EXPECT_EQ(spade::vehicles::hover_command(degenerate), 0.0f);
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) degenerate.rotors[i].thrust_coeff = 0.0f;
    degenerate.mass = kMass;
    EXPECT_EQ(spade::vehicles::hover_command(degenerate), 0.0f);
}

// ===========================================================================
// 2. Parameter validation -- including the one rotor.hpp says fails QUIETLY
// ===========================================================================

// rotor.hpp's apply_rotors() preconditions name this task by number: "radius
// == 0 makes v_h == 0 (no disc area) so f_inflow returns its neutral 1, AND
// leaves no image source so f_ground returns 1 as well ... a plausible-looking
// vehicle with both aero corrections silently switched off. Task 18's parameter
// validation should reject a non-positive radius on an enabled row."
TEST(Quadrotor, NonPositiveRotorRadiusIsRejectedRatherThanSilentlyDisablingBothCorrections) {
    for (const float bad_radius : {0.0f, -0.12f}) {
        QuadrotorParams params = test_quad(0.02f, 0.9f);
        params.rotors[2].radius = bad_radius;
        const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
        EXPECT_EQ(code_of(model), code(spade::Code::invalid_argument)) << "radius " << bad_radius;
        if (!model) {
            // The message must name the rotor, or a four-rotor model's error is
            // a scavenger hunt.
            EXPECT_NE(model.error().context.find("rotor 2"), std::string::npos)
                << model.error().context;
        }
    }

    // And the quiet failure this is guarding against is real: with radius 0
    // BOTH corrections return their neutral value, at every flight state.
    EXPECT_EQ(spade::vehicles::rotor_hover_induced_velocity(2.45f, kAirDensity, 0.0f), 0.0f);
    EXPECT_EQ(spade::vehicles::rotor_inflow_factor(-30.0f, 0.0f), 1.0f);  // deep descent
    EXPECT_EQ(spade::vehicles::rotor_ground_factor(0.001f, 0.0f), 1.0f);  // 1 mm off the floor
}

TEST(Quadrotor, DegenerateAirframeParametersAreRejected) {
    const auto rejected = [](const QuadrotorParams& params, const char* what) {
        const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
        EXPECT_EQ(code_of(model), code(spade::Code::invalid_argument)) << what;
    };

    {
        // mass is DIVIDED by in integrate_bodies() step 1 and is not guarded
        // there; the model layer is where it is refused.
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.mass = 0.0f;
        rejected(p, "zero mass");
        p.mass = -1.0f;
        rejected(p, "negative mass");
        p.mass = std::numeric_limits<float>::quiet_NaN();
        rejected(p, "NaN mass");
    }
    {
        // A zero moment would be inverted into the integrator's
        // "rotation-locked axis", which is a different statement from what the
        // author wrote.
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.inertia_diag = glm::vec3(kIx, 0.0f, kIz);
        rejected(p, "zero moment about Y");
        p.inertia_diag = glm::vec3(kIx, -kIy, kIz);
        rejected(p, "negative moment");
    }
    {
        // No arm, no roll or pitch authority -- and make_quadrotor, not
        // ModelType, is the layer that knows a quadrotor has arms.
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.arm_length = 0.0f;
        rejected(p, "zero arm length");
    }
    {
        // spin_dir is a DIRECTION, not a scale (rotor.hpp): 2 would double the
        // yaw torque without doubling the thrust.
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.spin_dirs[1] = 2.0f;
        rejected(p, "spin_dir of 2");
        p.spin_dirs[1] = -0.5f;
        rejected(p, "fractional spin_dir");
    }
    {
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.rotors[0].thrust_coeff = 0.0f;
        rejected(p, "zero thrust coefficient");
    }
    {
        // Q is a NON-NEGATIVE magnitude; spin_dir carries the sign.
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.rotors[3].torque_coeff = -1.0e-7f;
        rejected(p, "negative torque coefficient");
    }
    {
        // A negative time constant is a typo; 0 is the documented "no lag".
        QuadrotorParams p = test_quad(-0.01f, 0.9f);
        rejected(p, "negative tau");
    }
    {
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.imu.rate_divider = 0;
        rejected(p, "zero rate divider");
    }
    {
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.drag.mode = 99u;
        rejected(p, "unknown drag mode");
    }
    {
        // Drag must never add energy: a negative coefficient or area is
        // refused (physics::check_drag_law, the rule both drag doors share).
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.drag.coeffs.x = -0.01f;
        rejected(p, "negative drag coefficient");
    }
    {
        QuadrotorParams p = test_quad(0.02f, 0.9f);
        p.drag.area = -1.0f;
        rejected(p, "negative drag area");
    }

    // tau == 0 is LEGAL -- it is the "no lag" degenerate every moment test in
    // this file relies on.
    EXPECT_OK(spade::vehicles::make_quadrotor(test_quad(0.0f, 0.0f)));
}

// ===========================================================================
// 3. The model registry
// ===========================================================================

TEST(Quadrotor, ModelRegistryMintsOneBasedIdsAndRefusesInvalidModels) {
    spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);
    EXPECT_EQ(sim->model_count(), 0u);

    // A null id names nothing, before anything is registered and after.
    EXPECT_EQ(code_of(sim->model(ModelTypeId{})), code(spade::Code::not_found));

    const spade::Result<ModelType> quad = spade::vehicles::make_quadrotor(test_quad(0.02f, 0.9f));
    ASSERT_OK(quad);
    const spade::Result<ModelTypeId> first = sim->register_model(*quad);
    ASSERT_OK(first);
    EXPECT_EQ(first->value, 1u);  // one-based: 0 is the null id
    EXPECT_EQ(sim->model_count(), 1u);

    const spade::Result<ModelTypeId> second = sim->register_model(*quad);
    ASSERT_OK(second);
    EXPECT_EQ(second->value, 2u);
    EXPECT_EQ(sim->model_count(), 2u);

    const spade::Result<const ModelType*> read_back = sim->model(*first);
    ASSERT_OK(read_back);
    EXPECT_EQ((*read_back)->name, "test_quad");

    // Out of range, and the validator runs at REGISTRATION rather than at the
    // first spawn.
    EXPECT_EQ(code_of(sim->model(ModelTypeId{3})), code(spade::Code::not_found));
    ModelType broken = *quad;
    broken.rotors[1].radius = 0.0f;
    EXPECT_EQ(code_of(sim->register_model(broken)), code(spade::Code::invalid_argument));
    EXPECT_EQ(sim->model_count(), 2u) << "a rejected model must not enter the registry";

    // Spawning against an id from nowhere is not_found, not a crash.
    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    EXPECT_EQ(code_of(sim->spawn(0, ModelTypeId{99}, where)), code(spade::Code::not_found));
    EXPECT_EQ(code_of(sim->spawn(0, ModelTypeId{}, where)), code(spade::Code::not_found));
}

// ===========================================================================
// 4. Spawn: one description in, one composed vehicle out
// ===========================================================================

TEST(Quadrotor, SpawnBuildsTheBodyEveryRotorAndTheSensorAndSeedsTheRotorsInTrim) {
    spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);
    const QuadrotorParams params = test_quad(0.02f, 0.9f);
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    const float hover = spade::vehicles::hover_command(params);
    VehicleSpawn where;
    where.pos = glm::vec3(1.0f, 10.0f, -2.0f);
    where.vel = glm::vec3(0.5f, 0.0f, 0.25f);
    where.rotor_omega = hover;

    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    ASSERT_OK(vehicle);
    EXPECT_EQ(vehicle->rotor_count, 4u);
    EXPECT_EQ(vehicle->imu_count, 1u);
    EXPECT_EQ(vehicle->model, *id);
    EXPECT_FALSE(vehicle->is_null());

    // Before the boundary the vehicle exists as reservations only: the rotor
    // rows are inert, so a command would be overwritten by the pending init
    // and is REPORTED instead of lost (apply_wrench's contract, applied here).
    const std::array<float, 4> commands{hover, hover, hover, hover};
    EXPECT_EQ(code_of(sim->set_rotor_commands(*vehicle, commands)), code(spade::Code::not_found));
    EXPECT_EQ(code_of(sim->rotor(*vehicle, 0)), code(spade::Code::not_found));

    ASSERT_OK(sim->flush_structural());

    const spade::Result<const spade::BodyState*> body = sim->body(vehicle->body);
    ASSERT_OK(body);
    EXPECT_EQ((*body)->pos, where.pos);
    EXPECT_EQ((*body)->vel, where.vel);
    EXPECT_EQ((*body)->mass, kMass);
    // The MODEL carries the inertia; BodyState stores its inverse, and the
    // spawn path is the one place that inverts it.
    EXPECT_NEAR((*body)->inv_inertia_diag.x, 1.0f / kIx, 1e-6f);
    EXPECT_NEAR((*body)->inv_inertia_diag.y, 1.0f / kIy, 1e-6f);
    EXPECT_NEAR((*body)->inv_inertia_diag.z, 1.0f / kIz, 1e-6f);
    EXPECT_NE((*body)->flags & spade::physics::body_flags::active, 0u);
    // D-S6-2: the MODEL owns its bodies' contact-proxy radius, written at
    // spawn from ModelType::proxy_radius (0.18f here) -- NOT the world's
    // ContactParams::proxy_radius (0.1f, void_world()), which is the
    // assertion that would pass by coincidence if spawn() had wired the two
    // up backwards.
    EXPECT_EQ((*body)->proxy_radius, params.proxy_radius);
    EXPECT_NE((*body)->proxy_radius, 0.1f);

    // Four rotors, in DECLARATION order, each spawned holding its speed --
    // omega AND omega_cmd, which is what "spawns in trim" means.
    for (uint32_t i = 0; i < 4; ++i) {
        const spade::Result<const spade::vehicles::RotorRow*> row = sim->rotor(*vehicle, i);
        ASSERT_OK(row) << "rotor " << i;
        EXPECT_EQ((*row)->omega, hover) << "rotor " << i;
        EXPECT_EQ((*row)->omega_cmd, hover) << "rotor " << i;
        EXPECT_EQ((*row)->spin_dir, model->rotors[i].spin_dir) << "rotor " << i;
        EXPECT_EQ((*row)->local_pos, model->rotors[i].local_pos) << "rotor " << i;
        EXPECT_EQ((*row)->enabled, 1u) << "rotor " << i;
        // The reserved lanes rotor.hpp holds for the gyroscopic/BEMT opt-ins
        // must stay 0.
        EXPECT_EQ((*row)->_r0, 0.0f);
        EXPECT_EQ((*row)->_r1, 0.0f);
        EXPECT_EQ((*row)->_r2, 0.0f);
        EXPECT_EQ((*row)->_r3, 0.0f);
    }
    EXPECT_EQ(code_of(sim->rotor(*vehicle, 4)), code(spade::Code::invalid_argument));

    const spade::Result<uint32_t> live_rotors = sim->live_rotor_count(0);
    ASSERT_OK(live_rotors);
    EXPECT_EQ(*live_rotors, 4u);
    const spade::Result<uint32_t> live_sensors = sim->live_imu_sensor_count(0);
    ASSERT_OK(live_sensors);
    EXPECT_EQ(*live_sensors, 1u);
}

// ---------------------------------------------------------------------------
// D-S6-2's other half of the contract: a BARE body (Simulation::spawn(world,
// BodySpawn), the non-vehicle entry point -- no model, no per-body radius
// concept at authoring time) never gets a proxy_radius override. Its slot
// stays at the 0 sentinel, which is what makes a plain-body world's digest
// impossible to move by D-S6-2: the fallback to the world's default IS the
// only behaviour a bare spawn() ever had.
// ---------------------------------------------------------------------------
TEST(Quadrotor, PlainBodySpawnLeavesProxyRadiusAtTheSentinel) {
    spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);

    spade::BodySpawn plain;
    plain.pos = glm::vec3(3.0f, 10.0f, -1.0f);
    plain.mass = 2.0f;
    const spade::Result<BodyRef> ref = sim->spawn(0, plain);
    ASSERT_OK(ref);
    ASSERT_OK(sim->flush_structural());

    const spade::Result<const spade::BodyState*> body = sim->body(*ref);
    ASSERT_OK(body);
    EXPECT_EQ((*body)->proxy_radius, 0.0f);
}

// ---------------------------------------------------------------------------
// D-S6-2 fix-loop, guard 1 (I2, coordinator review): the vehicle-spawn site
// must reject a model whose own proxy_radius overrides the world default with
// something the world's GRID cannot afford (physics/grid.hpp's cell_size >=
// 2*radius footgun). sim/world_set.cpp's validate_grid_against_contacts()
// only ever checks the world's DEFAULT at world-creation time, before any
// vehicle model exists to register -- this is the gap that check cannot see.
// ---------------------------------------------------------------------------
TEST(Quadrotor, SpawnRejectsAModelRadiusTheWorldsGridCannotAfford) {
    // void_world()'s grid.cell_size is 0.5 m, which affords radii up to
    // 0.25 m (2*0.25 == 0.5). A model overriding the world's default (0.1 m)
    // with 0.3 m (2*0.3 == 0.6 > 0.5) must be rejected at spawn.
    {
        spade::Result<Simulation> sim = void_sim();
        ASSERT_OK(sim);
        QuadrotorParams oversize = test_quad(0.02f, 0.9f);
        oversize.proxy_radius = 0.3f;
        const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(oversize);
        ASSERT_OK(model);
        const spade::Result<ModelTypeId> id = sim->register_model(*model);
        ASSERT_OK(id);

        VehicleSpawn where;
        where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
        EXPECT_EQ(code_of(sim->spawn(0, *id, where)), code(spade::Code::invalid_argument));
    }

    // Sanity, in a FRESH world (own Simulation, so this vehicle's force-
    // element budget never collides with the rejected spawn above, which
    // never got as far as reserving one): the SAME grid does not reject a
    // model radius it CAN afford (0.2 m: 2*0.2 == 0.4 <= 0.5) -- the guard is
    // a genuine bound, not a blanket rejection of every override.
    {
        spade::Result<Simulation> sim = void_sim();
        ASSERT_OK(sim);
        QuadrotorParams fits = test_quad(0.02f, 0.9f);
        fits.proxy_radius = 0.2f;
        const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(fits);
        ASSERT_OK(model);
        const spade::Result<ModelTypeId> id = sim->register_model(*model);
        ASSERT_OK(id);

        VehicleSpawn where;
        where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
        EXPECT_OK(sim->spawn(0, *id, where));
    }
}

// ---------------------------------------------------------------------------
// D-S6-2 fix-loop, guard 2 (M3, coordinator review): the vehicle-spawn site's
// SDF-depth precondition ("not buried more than one proxy radius") must test
// the vehicle's own EFFECTIVE radius, not unconditionally the world default --
// a world WITH geometry (quad_hover deliberately has none, so it cannot
// exercise this at all).
// ---------------------------------------------------------------------------
TEST(Quadrotor, SpawnSdfDepthPreconditionUsesTheModelsEffectiveRadiusNotAlwaysTheWorldDefault) {
    // 0.15 m INSIDE the solid (phi == -0.15), so admission depends entirely on
    // which radius the precondition reads (phi >= -radius).
    const glm::vec3 buried_pos(0.0f, -0.15f, 0.0f);
    constexpr float kCellSize = 1.0f;  // comfortably affords every radius below

    // Direction 1: the model's own (LARGER) radius ADMITS a spawn the world
    // default would have REJECTED. World default 0.1 m: -0.15 >= -0.1 is
    // false (would reject). Model override 0.2 m: -0.15 >= -0.2 is true.
    {
        spade::Result<Simulation> sim = ground_plane_sim(/*default_radius=*/0.1f, kCellSize);
        ASSERT_OK(sim);
        QuadrotorParams params = test_quad(0.02f, 0.9f);
        params.proxy_radius = 0.2f;
        const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
        ASSERT_OK(model);
        const spade::Result<ModelTypeId> id = sim->register_model(*model);
        ASSERT_OK(id);

        VehicleSpawn where;
        where.pos = buried_pos;
        EXPECT_OK(sim->spawn(0, *id, where))
            << "the model's own (larger) radius should have admitted this spawn";
    }

    // Direction 2 (vice versa): the model's own (SMALLER) radius REJECTS a
    // spawn the world default would have ADMITTED. World default 0.3 m:
    // -0.15 >= -0.3 is true (would admit). Model override 0.1 m: -0.15 >=
    // -0.1 is false.
    {
        spade::Result<Simulation> sim = ground_plane_sim(/*default_radius=*/0.3f, kCellSize);
        ASSERT_OK(sim);
        QuadrotorParams params = test_quad(0.02f, 0.9f);
        params.proxy_radius = 0.1f;
        const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
        ASSERT_OK(model);
        const spade::Result<ModelTypeId> id = sim->register_model(*model);
        ASSERT_OK(id);

        VehicleSpawn where;
        where.pos = buried_pos;
        EXPECT_EQ(code_of(sim->spawn(0, *id, where)), code(spade::Code::invalid_argument))
            << "the model's own (smaller) radius should have rejected this spawn";
    }
}

TEST(Quadrotor, SpawnValidatesItsPoseAndRotorSpeed) {
    spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(test_quad(0.02f, 0.9f));
    ASSERT_OK(model);
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    const auto rejected = [&](const VehicleSpawn& where, const char* what) {
        EXPECT_EQ(code_of(sim->spawn(0, *id, where)), code(spade::Code::invalid_argument)) << what;
    };

    VehicleSpawn bad;
    bad.pos = glm::vec3(0.0f, std::numeric_limits<float>::infinity(), 0.0f);
    rejected(bad, "non-finite position");

    bad = VehicleSpawn{};
    bad.orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
    rejected(bad, "zero-length orientation");

    bad = VehicleSpawn{};
    // Both rotor curves are EVEN in omega, so a negative shaft speed would
    // produce the thrust of its magnitude while the lag drove it further
    // negative. spin_dir, not the sign of omega, says which way a rotor turns.
    bad.rotor_omega = -100.0f;
    rejected(bad, "negative rotor speed");

    bad = VehicleSpawn{};
    bad.rotor_omega = std::numeric_limits<float>::quiet_NaN();
    rejected(bad, "NaN rotor speed");

    // World index is checked too.
    VehicleSpawn ok;
    EXPECT_EQ(code_of(sim->spawn(1, *id, ok)), code(spade::Code::invalid_argument));
}

// The force-element budget is SHARED: a quadrotor's 4 rotors and 1 drag body
// are 5 force elements, whatever array each row lives in.
TEST(Quadrotor, RotorsAndDragBodiesShareTheWorldsDeclaredForceElementBudget) {
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(test_quad(0.02f, 0.9f));
    ASSERT_OK(model);
    ASSERT_EQ(model->force_element_count(), std::size_t{5});

    {   // Four force elements is one short, and the spawn is refused whole.
        spade::Result<Simulation> sim = void_sim(4, 4, 2);
        ASSERT_OK(sim);
        const spade::Result<ModelTypeId> id = sim->register_model(*model);
        ASSERT_OK(id);
        VehicleSpawn where;
        where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
        const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
        EXPECT_EQ(code_of(vehicle), code(spade::Code::capacity_exceeded));
        // NOTHING LEAKED: the capacity checks precede the first reservation, so
        // a refused spawn leaves every arena exactly as it was.
        const spade::Result<uint32_t> rotors = sim->live_rotor_count(0);
        ASSERT_OK(rotors);
        EXPECT_EQ(*rotors, 0u);
        const spade::Result<uint32_t> bodies = sim->live_body_count(0);
        ASSERT_OK(bodies);
        EXPECT_EQ(*bodies, 0u);
        const spade::Result<uint32_t> sensors = sim->live_imu_sensor_count(0);
        ASSERT_OK(sensors);
        EXPECT_EQ(*sensors, 0u);
    }
    {   // Five fits exactly -- and then the world is full for BOTH kinds.
        spade::Result<Simulation> sim = void_sim(4, 5, 2);
        ASSERT_OK(sim);
        const spade::Result<ModelTypeId> id = sim->register_model(*model);
        ASSERT_OK(id);
        VehicleSpawn where;
        where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
        const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
        ASSERT_OK(vehicle);
        ASSERT_OK(sim->flush_structural());

        // A sixth force element does not fit, even though the drag ARRAY has
        // four unused slots -- the budget is what the world declared.
        spade::DragElementSpawn extra;
        extra.mode = spade::physics::drag_mode::componentwise;
        EXPECT_EQ(code_of(sim->add_drag_element(vehicle->body, extra)),
                  code(spade::Code::capacity_exceeded));
    }
}

// The second door for drag. add_drag_element applies the same law check as
// ModelType::validate (physics::check_drag_law), so what a model may not
// carry, a body may not be given either.
TEST(Quadrotor, AddDragElementRefusesWhatTheDragLawCannotTake) {
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(test_quad(0.02f, 0.9f));
    ASSERT_OK(model);
    spade::Result<Simulation> sim = void_sim(4, 8, 2);
    ASSERT_OK(sim);
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);
    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    ASSERT_OK(vehicle);
    ASSERT_OK(sim->flush_structural());

    spade::DragElementSpawn negative_coeff;
    negative_coeff.mode = spade::physics::drag_mode::componentwise;
    negative_coeff.coeffs = glm::vec3(-0.01f, 0.02f, 0.02f);
    EXPECT_EQ(code_of(sim->add_drag_element(vehicle->body, negative_coeff)), code(spade::Code::invalid_argument))
        << "negative coefficient";

    spade::DragElementSpawn negative_area;
    negative_area.mode = spade::physics::drag_mode::quadratic;
    negative_area.area = -1.0f;
    negative_area.coeffs = glm::vec3(1.0f, 0.0f, 0.0f);
    EXPECT_EQ(code_of(sim->add_drag_element(vehicle->body, negative_area)), code(spade::Code::invalid_argument))
        << "negative area";

    spade::DragElementSpawn fine;
    fine.mode = spade::physics::drag_mode::componentwise;
    fine.coeffs = glm::vec3(0.01f, 0.02f, 0.02f);
    EXPECT_OK(sim->add_drag_element(vehicle->body, fine));
}

// ===========================================================================
// 5. The control mapping: index -> arm -> axis -> sign
// ===========================================================================

// ONE ROTOR AT A TIME. This is the strongest form of the layout assertion: for
// each index in turn, the moment it produces alone identifies BOTH which arm it
// sits on (which of M_x / M_z is non-zero, and with what sign) AND which way it
// spins (the sign of M_y). Four independent, asymmetric facts.
TEST(Quadrotor, EachRotorAloneProducesTheMomentItsArmAndSpinPredict) {
    const QuadrotorParams params = test_quad(0.0f, 0.0f);  // no lag: omega == command
    const float w = 400.0f;
    const float h = 1.0e-3f;  // 1 kHz substep -- Simulation::substep_h() for this world

    // The three quantities the derivation needs, computed here rather than read
    // out of the engine.
    const float thrust = kThrustCoeff * w * w;   // 1.6 N
    const float reaction = kTorqueCoeff * w * w; // 0.0256 N m

    struct Expectation {
        glm::vec3 moment;
        const char* what;
    };
    const Expectation expectations[kQuadrotorRotorCount] = {
        {glm::vec3(0.0f, -reaction, kArm * thrust), "rotor 0: +X arm, spin +1"},
        {glm::vec3(-kArm * thrust, reaction, 0.0f), "rotor 1: +Z arm, spin -1"},
        {glm::vec3(0.0f, -reaction, -kArm * thrust), "rotor 2: -X arm, spin +1"},
        {glm::vec3(kArm * thrust, reaction, 0.0f), "rotor 3: -Z arm, spin -1"},
    };

    for (std::size_t j = 0; j < kQuadrotorRotorCount; ++j) {
        std::array<float, 4> commands{0.0f, 0.0f, 0.0f, 0.0f};
        commands[j] = w;

        const spade::Result<glm::vec3> measured = measure_first_substep_omega(params, commands);
        ASSERT_OK(measured) << expectations[j].what;

        const glm::vec3 expected = glm::vec3(expectations[j].moment.x / kIx,
                                             expectations[j].moment.y / kIy,
                                             expectations[j].moment.z / kIz) *
                                   h;
        ExpectOmegaNear(*measured, expected, 1.0e-5f, expectations[j].what);

        // Cross-checked against the general reference too, so the table above
        // and the formula cannot both be wrong in the same way.
        ExpectOmegaNear(*measured, expected_first_substep_omega(params, commands, h), 1.0e-5f,
                        expectations[j].what);
    }
}

// THE BRIEF'S YAW CHECKBOX. Raising the +1 diagonal (0 and 2) and lowering the
// -1 diagonal (1 and 3) must yaw in the direction the SIGNED REACTION TORQUE
// dictates -- and, because rotor.hpp corrects the THRUST but never Q, the
// magnitude is an exact closed form rather than a bound.
TEST(Quadrotor, DifferentialBetweenTheSpinPairsYawsInTheDirectionTheLayoutDictates) {
    const QuadrotorParams params = test_quad(0.0f, 0.0f);
    const float hover = spade::vehicles::hover_command(params);
    const float h = 1.0e-3f;
    const float fast = hover * 1.1f;
    const float slow = hover * 0.9f;

    // Spin up the +1 diagonal, spin down the -1 diagonal.
    const std::array<float, 4> plus_up{fast, slow, fast, slow};
    const spade::Result<glm::vec3> measured = measure_first_substep_omega(params, plus_up);
    ASSERT_OK(measured);

    // M_y = -(Q_0 - Q_1 + Q_2 - Q_3) = -2 k_Q (fast^2 - slow^2) < 0.
    const float moment_y = -2.0f * kTorqueCoeff * (fast * fast - slow * slow);
    EXPECT_LT(moment_y, 0.0f) << "the derivation itself must predict a negative yaw moment";
    const glm::vec3 expected(0.0f, moment_y / kIy * h, 0.0f);
    ExpectOmegaNear(*measured, expected, 1.0e-5f, "spin-up of the +1 diagonal");

    // PURE YAW: each diagonal is internally matched, so M_x and M_z cancel
    // exactly -- the +-+- layout's whole point.
    EXPECT_NEAR(measured->x, 0.0f, 1.0e-9f) << "a yaw differential must not roll";
    EXPECT_NEAR(measured->z, 0.0f, 1.0e-9f) << "a yaw differential must not pitch";
    EXPECT_LT(measured->y, 0.0f);

    // The mirrored differential yaws the other way, by the same magnitude.
    const std::array<float, 4> minus_up{slow, fast, slow, fast};
    const spade::Result<glm::vec3> mirrored = measure_first_substep_omega(params, minus_up);
    ASSERT_OK(mirrored);
    EXPECT_GT(mirrored->y, 0.0f);
    EXPECT_NEAR(mirrored->y, -measured->y, 1.0e-6f);

    // And the sign really is the SPIN's, not the position's: flipping the
    // layout to -+-+ with the identical commands reverses the yaw.
    QuadrotorParams flipped = params;
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) flipped.spin_dirs[i] = -params.spin_dirs[i];
    const spade::Result<glm::vec3> reversed = measure_first_substep_omega(flipped, plus_up);
    ASSERT_OK(reversed);
    EXPECT_GT(reversed->y, 0.0f);
    EXPECT_NEAR(reversed->y, -measured->y, 1.0e-6f);
}

// THE BRIEF'S ROLL-AXIS CHECKBOX. A differential inside ONE arm pair moves
// exactly one of M_x / M_z and leaves the other EXACTLY zero.
TEST(Quadrotor, ARollOrPitchDifferentialMovesOneBodyAxisAndLeavesTheOtherExactlyZero) {
    const QuadrotorParams params = test_quad(0.0f, 0.0f);
    const float hover = spade::vehicles::hover_command(params);
    const float h = 1.0e-3f;
    const float fast = hover * 1.1f;
    const float slow = hover * 0.9f;

    // ROLL: the 1/3 pair sits on the +-Z arms, so M_x = L (T_3 - T_1) and
    // M_z = L (T_0 - T_2) = 0. With the nose along body +X this is roll.
    {
        const std::array<float, 4> roll{hover, fast, hover, slow};
        const spade::Result<glm::vec3> measured = measure_first_substep_omega(params, roll);
        ASSERT_OK(measured);

        const float thrust_fast = kThrustCoeff * fast * fast;
        const float thrust_slow = kThrustCoeff * slow * slow;
        const float moment_x = kArm * (thrust_slow - thrust_fast);  // T_3 - T_1 < 0
        EXPECT_LT(moment_x, 0.0f);
        EXPECT_NEAR(measured->x, moment_x / kIx * h, 1.0e-5f * std::fabs(moment_x / kIx * h));
        EXPECT_NEAR(measured->z, 0.0f, 1.0e-9f) << "a roll differential must not pitch";

        // THE YAW COUPLING IS REAL AND IS NOT ZERO, because Q is quadratic in
        // omega: raising one rotor by 10% and lowering the other by 10% raises
        // the PAIR's total reaction torque by (1.21 + 0.81 - 2)/2 = 1% of one
        // rotor's. Derived from the IMPLEMENTED Q -- which rotor.hpp never
        // corrects -- rather than assumed away.
        const float moment_y = -kTorqueCoeff * (2.0f * hover * hover - fast * fast - slow * slow);
        EXPECT_GT(moment_y, 0.0f);
        EXPECT_NEAR(measured->y, moment_y / kIy * h, 1.0e-4f * std::fabs(moment_y / kIy * h));

        ExpectOmegaNear(*measured, expected_first_substep_omega(params, roll, h), 1.0e-4f, "roll");
    }

    // PITCH: the 0/2 pair sits on the +-X arms, so M_z moves and M_x is zero.
    {
        const std::array<float, 4> pitch{fast, hover, slow, hover};
        const spade::Result<glm::vec3> measured = measure_first_substep_omega(params, pitch);
        ASSERT_OK(measured);

        const float moment_z = kArm * (kThrustCoeff * fast * fast - kThrustCoeff * slow * slow);
        EXPECT_GT(moment_z, 0.0f);
        EXPECT_NEAR(measured->z, moment_z / kIz * h, 1.0e-5f * std::fabs(moment_z / kIz * h));
        EXPECT_NEAR(measured->x, 0.0f, 1.0e-9f) << "a pitch differential must not roll";

        ExpectOmegaNear(*measured, expected_first_substep_omega(params, pitch, h), 1.0e-4f, "pitch");
    }

    // COLLECTIVE: all four equal is pure thrust -- no moment on any axis at
    // all, exactly (equal terms cancel bit for bit, and the +-+- layout pairs
    // equal Q's with opposite signs).
    {
        const std::array<float, 4> collective{hover, hover, hover, hover};
        const spade::Result<glm::vec3> measured = measure_first_substep_omega(params, collective);
        ASSERT_OK(measured);
        EXPECT_EQ(measured->x, 0.0f);
        EXPECT_EQ(measured->y, 0.0f);
        EXPECT_EQ(measured->z, 0.0f);
    }
}

// ===========================================================================
// 6. HOVER -- the brief's headline bar
// ===========================================================================
//
// WHERE THE 5 cm AND 0.5 deg GO, derived rather than measured, so that a
// regression that merely stays inside the bar is still a visible change:
//
// ATTITUDE. With four identical rotors at one speed the four moment
// contributions are computed from BIT-IDENTICAL inputs -- same k_T, same omega,
// same inflow factor (v_axial is the same for all four, since omega_body is
// exactly 0 and every rotor samples the same wind), same ground factor (an
// empty SDF reports the same distance everywhere) -- and rotor.cpp accumulates
// them in slot order, which pairs +L*T against -L*T and +Q against -Q. Each
// cancellation is x + (-x) of the SAME float, i.e. exact. So the accumulated
// torque is EXACTLY zero, omega_body stays exactly zero, and
// integrate_orientation of a zero rate returns the quaternion unchanged. The
// predicted attitude drift is therefore not "small" -- it is ZERO -- and the
// test asserts a bound five orders below the contract to say so.
//
// POSITION. The only source of residual is the rounding of the square root in
// hover_command(): omega_h carries at most half an ulp of relative error, so
// T_total = 4 k_T omega_h^2 misses m*g by of order 2^-23 relative, i.e. an
// acceleration |a_0| <~ 1.2e-6 m/s^2. That residual is not free to integrate
// twice, either: the inflow factor is lambda(v / v_h) ~= 1 - v/(2 v_h), so a
// vertical velocity v produces a restoring acceleration of about -g v/(2 v_h)
// -- a first-order lag with time constant 2 v_h / g = 0.96 s for this airframe.
// The velocity therefore saturates around |a_0| * 0.96 ~ 1.2e-6 m/s and the
// ten-second displacement is bounded by about 1.2e-5 m. TWELVE MICRONS, against
// a 5 cm contract, and against the 490 m an unpowered body falls in the same
// ten seconds -- which is what makes even the loose bar a real assertion.
//
// WHAT IS ACTUALLY OBSERVED IS TIGHTER STILL, AND IS NOT ASSERTED. For THIS
// airframe's constants the fp32 chain happens to close exactly: 4 k_T omega_h^2
// divided by this mass lands on precisely the float gravity was given, so
// (accel_ext + gravity) is exactly zero every substep, the velocity never
// leaves zero, and the measured drift is 0 in position and 0 in attitude after
// all 10,000 substeps. That is a COINCIDENCE OF THE CONSTANTS, not a property
// of the engine -- a different mass or k_T would leave the ulp-level residual
// the paragraph above bounds -- so the test deliberately asserts the DERIVED
// bound rather than the coincidence. Asserting exact zero here would make this
// test fail on the first airframe retune for no engineering reason.
//
// So: the CONTRACT (5 cm, 0.5 deg) and, separately, the DERIVED bound with
// three orders of slack on the analysis. If the derived assertion ever fails
// while the contract holds, something changed in the trim path and this comment
// is where to start.
// ===========================================================================

TEST(Quadrotor, SymmetricHoverHoldsPositionAndAttitudeForTenThousandSubsteps) {
    const QuadrotorParams params = test_quad(0.02f, 0.9f);
    spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);
    ASSERT_EQ(sim->substeps(), 1u);
    ASSERT_EQ(sim->substep_dt_ns(), 1'000'000u);  // 1 kHz

    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    const float hover = spade::vehicles::hover_command(params);
    const glm::vec3 start(0.0f, 10.0f, 0.0f);
    VehicleSpawn where;
    where.pos = start;
    where.rotor_omega = hover;  // spawned IN TRIM -- see VehicleSpawn
    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    ASSERT_OK(vehicle);
    ASSERT_OK(sim->flush_structural());

    constexpr uint64_t kSubsteps = 10'000;  // ten seconds at 1 kHz
    ASSERT_OK(sim->step(kSubsteps));

    const spade::Result<const spade::BodyState*> body = sim->body(vehicle->body);
    ASSERT_OK(body);

    const float position_drift = glm::length((*body)->pos - start);
    const float attitude_drift = rotation_angle((*body)->orient);

    // THE CONTRACT.
    EXPECT_LT(position_drift, 0.05f) << "position drift over 10k substeps, m";
    EXPECT_LT(attitude_drift, 0.5f * 3.14159265f / 180.0f) << "attitude drift over 10k substeps, rad";

    // THE DERIVATION (see the block comment): the predicted displacement bound
    // is about 12 microns -- and this airframe's constants close exactly, so
    // what is actually observed is 0. The bound, not the coincidence, is what
    // is asserted.
    EXPECT_LT(position_drift, 1.0e-2f) << "position drift is three orders above the derived bound";
    EXPECT_LT(attitude_drift, 1.0e-6f) << "attitude drift is not the exactly-zero the symmetry predicts";

    // The trim is HELD, not decayed into: with omega == omega_cmd the lag
    // update is a no-op every substep, forever.
    for (uint32_t i = 0; i < 4; ++i) {
        const spade::Result<const spade::vehicles::RotorRow*> row = sim->rotor(*vehicle, i);
        ASSERT_OK(row) << "rotor " << i;
        EXPECT_EQ((*row)->omega, hover) << "rotor " << i << " shaft speed drifted";
    }

    // AND IT REALLY IS FLYING, not parked: a hovering accelerometer reads +g
    // OPPOSITE gravity (sensors/imu.hpp section 1). This is the reading that
    // distinguishes a working rotor chain from a body that never moved --
    // imu.hpp section 5 warns that a body RESTING on the ground SDF reads ~0
    // instead, because contact impulses bypass force_acc, so a hover test is
    // the only place this assertion means what it says.
    ASSERT_EQ(vehicle->imu_count, 1u);
    std::vector<ImuSample> samples(kRingDepth);
    const spade::Result<spade::ImuPoll> poll = sim->poll_imu(vehicle->imu_sensors[0], 0, samples);
    ASSERT_OK(poll);
    ASSERT_FALSE(poll->samples.empty());
    const ImuSample& newest = poll->samples.back();
    EXPECT_NEAR(newest.accel.x, 0.0f, 1.0e-4f);
    EXPECT_NEAR(newest.accel.y, kG, 1.0e-3f) << "a hovering IMU must read +g opposite gravity";
    EXPECT_NEAR(newest.accel.z, 0.0f, 1.0e-4f);
    EXPECT_NEAR(glm::length(newest.gyro), 0.0f, 1.0e-6f);
}

// The counterfactual: the SAME airframe with its rotors at rest falls, and
// falls by the amount one g explains. Without this, the hover assertion above
// would also pass on a build where the rotor pass never ran and the body was
// somehow frozen.
TEST(Quadrotor, TheSameAirframeWithItsRotorsStoppedFallsUnderGravity) {
    const QuadrotorParams params = test_quad(0.02f, 0.0f);
    spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 1000.0f, 0.0f);
    where.rotor_omega = 0.0f;
    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    ASSERT_OK(vehicle);
    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->step(1000));  // one second

    const spade::Result<const spade::BodyState*> body = sim->body(vehicle->body);
    ASSERT_OK(body);
    // Symplectic Euler over n substeps of h: y = y0 - g h^2 n(n+1)/2. With zero
    // drag coefficients that is exact.
    const float h = sim->substep_h();
    const float expected = 1000.0f - kG * h * h * (1000.0f * 1001.0f * 0.5f);
    EXPECT_NEAR((*body)->pos.y, expected, 1.0e-2f);
    EXPECT_LT((*body)->pos.y, 996.0f) << "the airframe did not fall";
}

// ===========================================================================
// 7. COLLECTIVE STEP -- climb acceleration against the Task 17 reference
// ===========================================================================
//
// The brief's form is  a = (dT * f_inflow) / m,  and it is checked twice:
//
//   * AT THE STEP, from rest, where f_inflow is exactly 1 and dT is exactly
//     (1.1^2 - 1) m g = 0.21 m g, so the expected climb acceleration is
//     0.21 g = 2.059 m/s^2 with no modelling left in it at all;
//   * AFTER 300 ms OF CLIMB, where the vehicle is doing about 0.6 m/s and
//     f_inflow has fallen several percent below 1. That second check is the one
//     that puts Task 17's curve in the loop, so the test REFUSES TO PASS unless
//     the correction has actually moved -- see the EXPECT_LT on f_inflow.
//
// The reference is built from rotor.hpp's own exported functions, fed the
// vehicle's ACTUAL state (the shaft speed read out of the rotor row, the
// vertical velocity read out of the body), not from a re-implementation of
// them. The model carries ZERO drag coefficients so the measured acceleration
// is the rotor chain alone; the drag element is still present and still
// evaluated, it simply contributes nothing.
// ===========================================================================

TEST(Quadrotor, CollectiveStepClimbAccelerationTracksTheRotorInflowReference) {
    const QuadrotorParams params = test_quad(0.0f, 0.0f);  // no lag: the step is a step
    spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    const float hover = spade::vehicles::hover_command(params);
    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 50.0f, 0.0f);
    where.rotor_omega = hover;
    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    ASSERT_OK(vehicle);
    ASSERT_OK(sim->flush_structural());

    const float h = sim->substep_h();
    const spade::Result<const spade::WorldParams*> world = sim->world_params(0);
    ASSERT_OK(world);
    const float density = (*world)->air_density;

    // The reference, from Task 17's exported curve. `omega` and `climb_rate`
    // come from the engine's own state; everything else is this test's.
    const auto reference_accel = [&](float omega, float climb_rate) {
        const float thrust_static = kThrustCoeff * omega * omega;
        const float v_hover =
            spade::vehicles::rotor_hover_induced_velocity(thrust_static, density, kRadius);
        const float f_inflow = spade::vehicles::rotor_inflow_factor(climb_rate, v_hover);
        const float f_ground = spade::vehicles::rotor_ground_factor(spade::kSdfEmptyDistance, kRadius);
        return 4.0f * thrust_static * f_inflow * f_ground / kMass - kG;
    };
    const auto inflow_factor = [&](float omega, float climb_rate) {
        const float thrust_static = kThrustCoeff * omega * omega;
        return spade::vehicles::rotor_inflow_factor(
            climb_rate, spade::vehicles::rotor_hover_induced_velocity(thrust_static, density, kRadius));
    };

    // --- trim, as the baseline -------------------------------------------
    ASSERT_OK(sim->step(1));
    const spade::Result<const spade::BodyState*> trimmed = sim->body(vehicle->body);
    ASSERT_OK(trimmed);
    EXPECT_NEAR((*trimmed)->vel.y, 0.0f, 1.0e-5f) << "the vehicle did not spawn in trim";

    // --- the +10% collective step ----------------------------------------
    const float stepped = hover * 1.1f;
    const std::array<float, 4> commands{stepped, stepped, stepped, stepped};
    ASSERT_OK(sim->set_rotor_commands(*vehicle, commands));

    const float v_before = (*trimmed)->vel.y;
    ASSERT_OK(sim->step(1));
    const spade::Result<const spade::BodyState*> after = sim->body(vehicle->body);
    ASSERT_OK(after);
    const float measured = ((*after)->vel.y - v_before) / h;

    // f_inflow is exactly 1 here, so the closed form is 0.21 g exactly.
    const float closed_form = (1.1f * 1.1f - 1.0f) * kG;
    EXPECT_NEAR(measured, closed_form, 0.05f * closed_form)
        << "climb acceleration at the collective step";
    EXPECT_NEAR(measured, reference_accel(stepped, v_before), 0.05f * closed_form);

    // --- one second later, with the inflow correction genuinely engaged ---
    //
    // The climb is a first-order approach to a terminal rate: the correction
    // lambda(v/v_h) ~= 1 - v/(2 v_h) is itself the damping. For this airframe
    // v_h is 5.17 m/s at the stepped speed and the terminal rate is about
    // 2.0 m/s with a time constant near one second, so a full second of climb
    // puts the vehicle around 1.3 m/s -- where f_inflow is roughly 0.88, i.e.
    // a correction more than twice the 5% tolerance. That margin is what makes
    // the comparison below a test of Task 17's curve rather than of
    // multiplication by one, and it is asserted rather than assumed.
    ASSERT_OK(sim->step(999));
    const spade::Result<const spade::BodyState*> climbing = sim->body(vehicle->body);
    ASSERT_OK(climbing);
    const spade::Result<const spade::vehicles::RotorRow*> row = sim->rotor(*vehicle, 0);
    ASSERT_OK(row);

    // Captured BEFORE the measured substep: both of these point into the arena
    // and would otherwise read the post-step values.
    const float climb_rate = (*climbing)->vel.y;
    const float omega = (*row)->omega;
    EXPECT_EQ(omega, stepped) << "with tau = 0 the shaft speed is the command, exactly";
    EXPECT_GT(climb_rate, 0.5f) << "the vehicle is not climbing, so nothing below means anything";

    const float engaged = inflow_factor(omega, climb_rate);
    EXPECT_LT(engaged, 0.95f) << "f_inflow = " << engaged
                              << " has not moved far enough off 1 for this test to be a test";

    ASSERT_OK(sim->step(1));
    const spade::Result<const spade::BodyState*> next = sim->body(vehicle->body);
    ASSERT_OK(next);
    const float measured_late = ((*next)->vel.y - climb_rate) / h;
    const float expected_late = reference_accel(omega, climb_rate);

    EXPECT_NEAR(measured_late, expected_late, 0.05f * std::fabs(expected_late))
        << "climb acceleration in the inflow-corrected regime (f_inflow = " << engaged << ")";
    // And the correction really is COSTING thrust: the corrected acceleration
    // must be measurably below the uncorrected one.
    const float uncorrected = 4.0f * kThrustCoeff * omega * omega / kMass - kG;
    EXPECT_LT(measured_late, uncorrected * 0.95f);
}

// ===========================================================================
// 8. A vehicle is state: spawn/despawn round trips, and the snapshot
// ===========================================================================

// THE BRIEF'S ROUND-TRIP CHECKBOX. Two runs of the same spawn/despawn/respawn
// script must land on the same digest -- which covers every registered byte,
// including the rotor rows, the free lists' effect on future slot assignment,
// and the sensor rings.
TEST(Quadrotor, SpawnDespawnRespawnIsDigestIdenticalRunToRun) {
    const QuadrotorParams params = test_quad(0.02f, 0.9f);
    const float hover = spade::vehicles::hover_command(params);

    const auto run = [&]() -> spade::Result<uint64_t> {
        spade::Result<Simulation> sim = void_sim(4, 12, 4);
        if (!sim) return std::unexpected(sim.error());
        const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
        if (!model) return std::unexpected(model.error());
        const spade::Result<ModelTypeId> id = sim->register_model(*model);
        if (!id) return std::unexpected(id.error());

        VehicleSpawn first;
        first.pos = glm::vec3(0.0f, 10.0f, 0.0f);
        first.rotor_omega = hover;
        const spade::Result<VehicleRef> a = sim->spawn(0, *id, first);
        if (!a) return std::unexpected(a.error());
        if (spade::Result<void> r = sim->step(50); !r) return std::unexpected(r.error());

        if (spade::Result<void> r = sim->despawn(a->body); !r) return std::unexpected(r.error());
        if (spade::Result<void> r = sim->step(5); !r) return std::unexpected(r.error());

        VehicleSpawn second;
        second.pos = glm::vec3(1.0f, 12.0f, -1.0f);
        second.rotor_omega = hover * 0.5f;
        const spade::Result<VehicleRef> b = sim->spawn(0, *id, second);
        if (!b) return std::unexpected(b.error());
        if (spade::Result<void> r = sim->step(1); !r) return std::unexpected(r.error());

        const std::array<float, 4> commands{hover, hover * 1.05f, hover, hover * 0.95f};
        if (spade::Result<void> r = sim->set_rotor_commands(*b, commands); !r) {
            return std::unexpected(r.error());
        }
        if (spade::Result<void> r = sim->step(50); !r) return std::unexpected(r.error());
        return spade::testing::state_digest(*sim);
    };

    const spade::Result<uint64_t> first = run();
    ASSERT_OK(first);
    const spade::Result<uint64_t> second = run();
    ASSERT_OK(second);
    EXPECT_EQ(*first, *second);
}

// The despawn CASCADE. Every rotor, drag body and sensor a vehicle took comes
// back, its rows read as zeroes, and the next vehicle gets the same slots --
// which is the property that keeps a long spawn/despawn run from exhausting a
// world's capacity while appearing to work.
TEST(Quadrotor, DespawningAVehicleReleasesEveryRotorDragAndSensorSlotItTook) {
    spade::Result<Simulation> sim = void_sim(4, 12, 4);
    ASSERT_OK(sim);
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(test_quad(0.02f, 0.9f));
    ASSERT_OK(model);
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where.rotor_omega = 500.0f;
    const spade::Result<VehicleRef> first = sim->spawn(0, *id, where);
    ASSERT_OK(first);
    ASSERT_OK(sim->step(3));

    {
        const spade::Result<uint32_t> rotors = sim->live_rotor_count(0);
        ASSERT_OK(rotors);
        EXPECT_EQ(*rotors, 4u);
        const spade::Result<uint32_t> sensors = sim->live_imu_sensor_count(0);
        ASSERT_OK(sensors);
        EXPECT_EQ(*sensors, 1u);
    }

    // Remember the slots so the reuse assertion below is exact.
    const std::array<uint32_t, spade::vehicles::kMaxModelRotors> taken = first->rotor_slots;

    ASSERT_OK(sim->despawn(first->body));
    // The ref is dead the moment despawn() returns, for rotors as for bodies.
    const std::array<float, 4> commands{100.0f, 100.0f, 100.0f, 100.0f};
    EXPECT_EQ(code_of(sim->set_rotor_commands(*first, commands)), code(spade::Code::not_found));
    ASSERT_OK(sim->flush_structural());

    {
        const spade::Result<uint32_t> rotors = sim->live_rotor_count(0);
        ASSERT_OK(rotors);
        EXPECT_EQ(*rotors, 0u) << "the despawn cascade did not reach the rotors";
        const spade::Result<uint32_t> sensors = sim->live_imu_sensor_count(0);
        ASSERT_OK(sensors);
        EXPECT_EQ(*sensors, 0u);
        const spade::Result<uint32_t> bodies = sim->live_body_count(0);
        ASSERT_OK(bodies);
        EXPECT_EQ(*bodies, 0u);
    }

    // The freed rows read as zeroes, like every other freed arena slot -- so a
    // disabled rotor cannot keep thrusting and cannot appear in a snapshot.
    const spade::Result<std::span<const spade::vehicles::RotorRow>> rows =
        sim->arenas().array(sim->rotors_array());
    ASSERT_OK(rows);
    for (const spade::vehicles::RotorRow& row : *rows) {
        EXPECT_EQ(row.enabled, 0u);
        EXPECT_EQ(row.omega, 0.0f);
        EXPECT_EQ(row.omega_cmd, 0.0f);
        EXPECT_EQ(row.thrust_coeff, 0.0f);
    }

    // Lowest-free-first: the next vehicle lands on exactly the released slots.
    const spade::Result<VehicleRef> second = sim->spawn(0, *id, where);
    ASSERT_OK(second);
    for (uint32_t i = 0; i < 4; ++i) {
        EXPECT_EQ(second->rotor_slots[i], taken[i]) << "rotor " << i;
    }
    ASSERT_OK(sim->flush_structural());
    const spade::Result<uint32_t> rotors = sim->live_rotor_count(0);
    ASSERT_OK(rotors);
    EXPECT_EQ(*rotors, 4u);
}

// THE REPLAY GUARANTEE, for a flying vehicle. The resumed Simulation is FRESH:
// it has registered the same model -- the model registry is configuration, and
// restore() refuses a blob taken under another (snapshot format v3, SCN-006) --
// but it has never spawned anything and never issues a rotor command. If the
// shaft speeds or the commands in force were not registered state, the
// restored vehicle would spin down and fall out of the digest immediately.
TEST(Quadrotor, AFlyingVehicleSurvivesASnapshotIntoASimulationThatNeverSpawnedIt) {
    const QuadrotorParams params = test_quad(0.02f, 0.9f);
    const float hover = spade::vehicles::hover_command(params);
    const spade::Result<WorldSetDesc> desc = void_world();
    ASSERT_OK(desc);
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);

    // The uninterrupted reference.
    const auto build_and_fly = [&](Simulation& sim) -> spade::Result<void> {
        const spade::Result<ModelTypeId> id = sim.register_model(*model);
        if (!id) return std::unexpected(id.error());
        VehicleSpawn where;
        where.pos = glm::vec3(0.0f, 20.0f, 0.0f);
        where.rotor_omega = hover;
        const spade::Result<VehicleRef> vehicle = sim.spawn(0, *id, where);
        if (!vehicle) return std::unexpected(vehicle.error());
        if (spade::Result<void> r = sim.flush_structural(); !r) return r;
        // An ASYMMETRIC command, so the vehicle is genuinely manoeuvring
        // across the resume point rather than sitting in a fixed point.
        const std::array<float, 4> commands{hover * 1.02f, hover * 0.98f, hover * 1.03f,
                                            hover * 0.97f};
        return sim.set_rotor_commands(*vehicle, commands);
    };

    spade::Result<Simulation> reference = Simulation::create(*desc, 1'000'000, 1);
    ASSERT_OK(reference);
    ASSERT_OK(build_and_fly(*reference));
    ASSERT_OK(reference->step(400));
    const uint64_t expected = spade::testing::state_digest(*reference);

    spade::Result<Simulation> interrupted = Simulation::create(*desc, 1'000'000, 1);
    ASSERT_OK(interrupted);
    ASSERT_OK(build_and_fly(*interrupted));
    ASSERT_OK(interrupted->step(200));
    const spade::Result<SnapshotBlob> blob = interrupted->snapshot();
    ASSERT_OK(blob);

    // A FRESH Simulation: the same model registered (configuration), but no
    // spawn and no command. Everything the vehicle needs must come out of the
    // blob.
    spade::Result<Simulation> resumed = Simulation::create(*desc, 1'000'000, 1);
    ASSERT_OK(resumed);
    ASSERT_OK(resumed->register_model(*model));
    EXPECT_EQ(resumed->model_count(), 1u);
    const spade::Result<uint32_t> spawned = resumed->live_body_count(0);
    ASSERT_OK(spawned);
    EXPECT_EQ(*spawned, 0u) << "nothing spawned before the restore";
    ASSERT_OK(resumed->restore(*blob));
    EXPECT_EQ(resumed->tick().value, 200u);
    ASSERT_OK(resumed->step(200));

    EXPECT_EQ(spade::testing::state_digest(*resumed), expected);

    // ... and the restored vehicle really was still flying, not frozen or
    // fallen: an unpowered body would be 19.2 m by now, and a vehicle whose
    // rotors had spun down would be on its way there.
    const spade::Result<BodyRef> ref = resumed->body_ref_at(0, 0);
    ASSERT_OK(ref);
    const spade::Result<const spade::BodyState*> body = resumed->body(*ref);
    ASSERT_OK(body);
    EXPECT_GT((*body)->pos.y, 19.5f);
    EXPECT_LT((*body)->pos.y, 20.5f);
}

// ===========================================================================
// 9. The command API's own contract
// ===========================================================================

TEST(Quadrotor, SetRotorCommandsIsAllOrNothingAndRefusesWhatItCannotMean) {
    spade::Result<Simulation> sim = void_sim();
    ASSERT_OK(sim);
    const QuadrotorParams params = test_quad(0.02f, 0.9f);
    const spade::Result<ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);
    const spade::Result<ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    const float hover = spade::vehicles::hover_command(params);
    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where.rotor_omega = hover;
    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    ASSERT_OK(vehicle);
    ASSERT_OK(sim->flush_structural());

    const std::array<float, 3> too_few{hover, hover, hover};
    EXPECT_EQ(code_of(sim->set_rotor_commands(*vehicle, too_few)),
              code(spade::Code::invalid_argument));

    const std::array<float, 4> negative{hover, -1.0f, hover, hover};
    EXPECT_EQ(code_of(sim->set_rotor_commands(*vehicle, negative)),
              code(spade::Code::invalid_argument));

    const std::array<float, 4> nan_command{hover, hover,
                                           std::numeric_limits<float>::quiet_NaN(), hover};
    EXPECT_EQ(code_of(sim->set_rotor_commands(*vehicle, nan_command)),
              code(spade::Code::invalid_argument));

    // ALL OR NOTHING: none of the three rejected calls may have written a
    // single row, or a mixer bug becomes a vehicle that yaws for no reason.
    for (uint32_t i = 0; i < 4; ++i) {
        const spade::Result<const spade::vehicles::RotorRow*> row = sim->rotor(*vehicle, i);
        ASSERT_OK(row) << "rotor " << i;
        EXPECT_EQ((*row)->omega_cmd, hover) << "rotor " << i << ": a rejected call wrote a row";
    }

    // A good call writes every row, in declaration order.
    const std::array<float, 4> good{10.0f, 20.0f, 30.0f, 40.0f};
    ASSERT_OK(sim->set_rotor_commands(*vehicle, good));
    for (uint32_t i = 0; i < 4; ++i) {
        const spade::Result<const spade::vehicles::RotorRow*> row = sim->rotor(*vehicle, i);
        ASSERT_OK(row) << "rotor " << i;
        EXPECT_EQ((*row)->omega_cmd, good[i]) << "rotor " << i;
    }

    // PERSISTENT, unlike apply_wrench(): the command survives stepping,
    // because it is a row field rather than an accumulator.
    ASSERT_OK(sim->step(3));
    const spade::Result<const spade::vehicles::RotorRow*> row = sim->rotor(*vehicle, 2);
    ASSERT_OK(row);
    EXPECT_EQ((*row)->omega_cmd, 30.0f);
    // ... and the shaft speed is LAGGING toward it, not jumping to it, with the
    // 20 ms time constant this airframe declares.
    EXPECT_LT((*row)->omega, hover);
    EXPECT_GT((*row)->omega, 30.0f);
}
