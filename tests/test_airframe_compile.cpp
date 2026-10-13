#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include "state/layout.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/battery.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/motor.hpp"
#include "vehicles/part_pose.hpp"
#include "vehicles/propeller.hpp"
#include "vehicles/propulsion_flags.hpp"
#include "world/sdf.hpp"

// ===========================================================================
// The airframe compile (vehicles/airframe_compile.hpp, DBP-44). Expectations
// are closed forms or independent measurements, never the compile's own
// arithmetic read back (TD-4).
// ===========================================================================

using namespace spade::vehicles;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kArm = 0.11;

[[nodiscard]] PartInertia make_part(PartShape shape, double mass, glm::dvec3 pos, glm::dvec3 size,
                                    glm::dquat orient = glm::dquat(1.0, 0.0, 0.0, 0.0)) {
    PartInertia p;
    p.shape = shape;
    p.mass = mass;
    p.position = pos;
    p.size = size;
    p.orientation = orient;
    return p;
}

// A 5-inch plus-layout quad (Y up, nose +X), 0.526 kg, in its design frame.
[[nodiscard]] AirframeSpec reference_quad() {
    AirframeSpec s;
    s.name = "ref_quad";
    s.param_schema_id = 1;
    s.visual_ref = "meshes/ref_quad";
    s.parts.push_back(make_part(PartShape::box, 0.12, glm::dvec3(0.0), glm::dvec3(0.12, 0.03, 0.12)));
    // 90-degree turns as exact literals, not glm::angleAxis: its libm cos and
    // sin would put a platform's rounding into the pinned identity's input.
    const double h = 0.7071067811865476;  // sqrt(1/2)
    const glm::dquat along_x(h, 0.0, 0.0, h);  // about +Z: the tube's +Y onto -X
    const glm::dquat along_z(h, h, 0.0, 0.0);  // about +X: the tube's +Y onto +Z
    const glm::dvec3 tube(0.006, kArm, 0.0);
    s.parts.push_back(make_part(PartShape::tube, 0.015, glm::dvec3(+kArm / 2, 0.0, 0.0), tube, along_x));
    s.parts.push_back(make_part(PartShape::tube, 0.015, glm::dvec3(0.0, 0.0, +kArm / 2), tube, along_z));
    s.parts.push_back(make_part(PartShape::tube, 0.015, glm::dvec3(-kArm / 2, 0.0, 0.0), tube, along_x));
    s.parts.push_back(make_part(PartShape::tube, 0.015, glm::dvec3(0.0, 0.0, -kArm / 2), tube, along_z));

    s.motor.kv = 2450.0;
    s.motor.resistance = 0.07;
    s.motor.no_load_current = 1.2;
    s.motor.no_load_voltage = 10.0;
    s.motor.current_max = 45.0;
    s.motor.pole_pairs = 7.0;
    s.motor.rotor_inertia = 1.2e-5;
    s.motor.stator = "2207";
    s.motor.mass = 0.033;
    s.motor.shape = BlockShape{PartShape::cylinder, glm::dvec3(0.014, 0.018, 0.0), glm::dmat3(0.0)};

    s.prop.diameter = 0.1524;
    s.prop.pitch = 0.1016;
    s.prop.blades = 3;
    s.prop.ct_table = {{0.0, 0.12}, {0.8, 0.04}};
    s.prop.cq_table = {{0.0, 0.008}, {0.8, 0.004}};
    s.prop.mass = 0.006;

    s.esc.current_burst = 55.0;
    s.esc.current_total = 160.0;
    s.esc.channels = 4;
    s.esc.on_resistance = 0.005;
    s.esc.mass = 0.01;
    s.esc.mount.position = glm::dvec3(0.0, 0.01, 0.0);

    s.battery.cells_series = 4;
    s.battery.cells_parallel = 1;
    s.battery.cell_capacity = 1.5;
    s.battery.cell_resistance = 0.005;
    s.battery.cell_voltage_cutoff = 3.0;
    s.battery.ocv_table = {{0.0, 3.3}, {0.2, 3.6}, {0.5, 3.75}, {0.8, 3.95}, {1.0, 4.2}};
    s.battery.c_rating = 100.0;
    s.battery.mass = 0.18;
    s.battery.mount.position = glm::dvec3(0.0, -0.03, 0.0);
    s.battery.shape = BlockShape{PartShape::box, glm::dvec3(0.07, 0.035, 0.035), glm::dmat3(0.0)};

    const double spins[4] = {1.0, -1.0, 1.0, -1.0};
    const glm::dvec3 hubs[4] = {glm::dvec3(kArm, 0.02, 0.0), glm::dvec3(0.0, 0.02, kArm),
                                glm::dvec3(-kArm, 0.02, 0.0), glm::dvec3(0.0, 0.02, -kArm)};
    for (int k = 0; k < 4; ++k) {
        RotorSpec r;
        r.hub.position = hubs[k];
        r.spin_dir = spins[k];
        s.rotors.push_back(r);
    }
    ImuSpec imu;
    imu.mount.position = glm::dvec3(0.0, 0.005, 0.0);
    s.imus.push_back(imu);
    s.proxy_radius = 0.15;
    return s;
}

// The same quad with its battery offset and turned: no longer symmetric.
[[nodiscard]] AirframeSpec asymmetric_quad() {
    AirframeSpec s = reference_quad();
    s.battery.mount.position = glm::dvec3(0.02, -0.03, 0.01);
    // 15 degrees about +Y, as an exact literal: (cos 7.5, 0, sin 7.5, 0).
    s.battery.mount.orientation = glm::dquat(0.9914448613738104, 0.0, 0.13052619222005157, 0.0);
    return s;
}

[[nodiscard]] bool has_issue(const std::vector<AirframeIssue>& issues, const std::string& kind, std::size_t index,
                             const std::string& field) {
    for (const AirframeIssue& i : issues) {
        if (i.kind == kind && i.index == index && i.field == field) return true;
    }
    return false;
}

// A double rounded to float once: how the compile rounds every solid's size.
[[nodiscard]] float f(double x) { return static_cast<float>(x); }

// A part's solid is one primitive at the identity, in the part's frame.
void expect_solid(const PartDesc& p, spade::SdfPrim kind, glm::vec4 params, float min_feature) {
    ASSERT_EQ(p.shape.nodes.size(), 1u) << p.name;
    ASSERT_EQ(p.shape.transforms.size(), 1u) << p.name;
    EXPECT_TRUE(p.shape.node_materials.empty()) << p.name;
    EXPECT_EQ(p.shape.nodes[0].kind, static_cast<uint32_t>(kind)) << p.name;
    EXPECT_EQ(p.shape.nodes[0].op, static_cast<uint32_t>(spade::SdfOp::none)) << p.name;
    EXPECT_EQ(p.shape.nodes[0].transform, 0u) << p.name;
    EXPECT_TRUE(p.shape.nodes[0].params == params) << p.name;
    EXPECT_TRUE(p.shape.transforms[0].world_to_local == glm::mat4(1.0f)) << p.name;
    EXPECT_EQ(p.shape.transforms[0].scale, 1.0f) << p.name;
    EXPECT_EQ(p.min_feature, min_feature) << p.name;
}

// Every float lane of a glm vector or quaternion, bit for bit.
template <class V>
[[nodiscard]] bool same_bits(const V& a, const V& b) {
    for (glm::length_t i = 0; i < V::length(); ++i) {
        if (std::bit_cast<uint32_t>(a[i]) != std::bit_cast<uint32_t>(b[i])) return false;
    }
    return true;
}

}  // namespace

TEST(AirframeCompile, CompilesAReferenceQuad) {
    const auto c = compile_airframe(reference_quad());
    ASSERT_TRUE(c.has_value()) << c.error().context;
    EXPECT_TRUE(c->model.validate().has_value());
    EXPECT_EQ(c->model.rotors.size(), 4u);
    EXPECT_NEAR(c->model.body.mass, 0.526, 1e-6);
    EXPECT_GT(c->fit.hover_duty, 0.0);
    EXPECT_LT(c->fit.hover_duty, 1.0);
    EXPECT_GT(c->fit.tau, 0.0);
    EXPECT_EQ(c->fit.mass_and_inertia, Provenance::from_parts);
    EXPECT_EQ(c->fit.rotor, Provenance::fitted);
    EXPECT_EQ(c->fit.rotor_inertia_source, Provenance::estimated) << "the propeller's inertia came from its mass";
    EXPECT_EQ(c->fit.drag, Provenance::estimated);
    EXPECT_TRUE(check_airframe(reference_quad()).empty());
}

TEST(AirframeCompile, ASymmetricQuadKeepsItsAxesAndMovesMountsByTheCentreOfMass) {
    const AirframeSpec s = reference_quad();
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const glm::quat q = c->model.design_to_principal;
    EXPECT_EQ(q.w, 1.0f);
    EXPECT_EQ(q.x, 0.0f);
    EXPECT_EQ(q.y, 0.0f);
    EXPECT_EQ(q.z, 0.0f);
    const glm::dvec3 com(c->model.com_offset);
    // Symmetric in x and z to rounding only: +x and -x masses are summed with
    // other parts between them, so they need not cancel to the last bit.
    EXPECT_NEAR(c->model.com_offset.x, 0.0f, 1e-9f);
    EXPECT_NEAR(c->model.com_offset.z, 0.0f, 1e-9f);
    for (std::size_t k = 0; k < 4; ++k) {
        const glm::dvec3 expected = s.rotors[k].hub.position - com;
        EXPECT_EQ(c->model.rotors[k].local_pos.x, static_cast<float>(expected.x)) << "rotor " << k;
        EXPECT_EQ(c->model.rotors[k].local_pos.y, static_cast<float>(expected.y)) << "rotor " << k;
        EXPECT_EQ(c->model.rotors[k].local_pos.z, static_cast<float>(expected.z)) << "rotor " << k;
        EXPECT_EQ(c->model.rotors[k].spin_dir, static_cast<float>(s.rotors[k].spin_dir));
    }
}

TEST(AirframeCompile, AnAsymmetricQuadsMountsRoundTripToTheDesignFrame) {
    const AirframeSpec s = asymmetric_quad();
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const glm::quat q = c->model.design_to_principal;
    ASSERT_FALSE(q.w == 1.0f && q.x == 0.0f && q.y == 0.0f && q.z == 0.0f) << "the offset battery must tilt the axes";
    const glm::dmat3 r_bd = glm::mat3_cast(glm::dquat(q.w, q.x, q.y, q.z));
    const glm::dvec3 com(c->model.com_offset);
    for (std::size_t k = 0; k < 4; ++k) {
        const glm::dvec3 back = glm::transpose(r_bd) * glm::dvec3(c->model.rotors[k].local_pos) + com;
        EXPECT_NEAR(back.x, s.rotors[k].hub.position.x, 1e-6) << "rotor " << k;
        EXPECT_NEAR(back.y, s.rotors[k].hub.position.y, 1e-6) << "rotor " << k;
        EXPECT_NEAR(back.z, s.rotors[k].hub.position.z, 1e-6) << "rotor " << k;
    }
}

TEST(AirframeCompile, AnImuAuthoredInDesignAxesReadsInDesignAxes) {
    const auto c = compile_airframe(asymmetric_quad());
    ASSERT_TRUE(c.has_value()) << c.error().context;
    ASSERT_EQ(c->model.imu_mounts.size(), 1u);
    // mount -> body is q_bd: a vector the IMU reports is a design-frame vector.
    const glm::quat m = c->model.imu_mounts[0].mount_orient;
    const glm::quat q = c->model.design_to_principal;
    EXPECT_EQ(m.w, q.w);
    EXPECT_EQ(m.x, q.x);
    EXPECT_EQ(m.y, q.y);
    EXPECT_EQ(m.z, q.z);
}

TEST(AirframeCompile, TheFittedRotorMatchesTheTableAndHoversTheWeight) {
    const AirframeSpec s = reference_quad();
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const double rho = s.air_density;
    const double d = s.prop.diameter;
    const double k_t = double{0.12f} * rho * std::pow(d, 4.0) / (4.0 * kPi * kPi);
    const double k_q = double{0.008f} * rho * std::pow(d, 5.0) / (4.0 * kPi * kPi);
    EXPECT_NEAR(c->fit.thrust_coeff, k_t, 1e-12 * k_t);
    EXPECT_NEAR(c->fit.torque_coeff, k_q, 1e-12 * k_q);
    EXPECT_EQ(c->model.rotors[0].thrust_coeff, static_cast<float>(c->fit.thrust_coeff));
    EXPECT_EQ(c->model.rotors[0].radius, static_cast<float>(0.5 * d));
    // At the hover speed, the fitted static thrust is the weight per rotor.
    const double weight = double{c->model.body.mass} * s.gravity / 4.0;
    EXPECT_NEAR(c->fit.thrust_coeff * c->fit.hover_omega * c->fit.hover_omega, weight, 1e-6 * weight);
}

// The fitted tau, against a measurement: step the duty 1% from hover through
// the float step functions and time the 63% rise. Small-signal, so the
// response is a first-order lag with the slope's time constant.
TEST(AirframeCompile, TheFittedTimeConstantMatchesAStepResponse) {
    const AirframeSpec s = reference_quad();
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const auto chain = airframe_propulsion_chain(s);
    ASSERT_TRUE(chain.has_value()) << chain.error().context;

    const float kv = static_cast<float>(chain->kv);
    const float r = static_cast<float>(chain->resistance);
    const float i0 = static_cast<float>(chain->no_load_current);
    const float dia = static_cast<float>(chain->diameter);
    const float rho = static_cast<float>(s.air_density);
    const float inertia = static_cast<float>(c->fit.rotor_inertia);
    const float v_source = 4.0f * battery_ocv(static_cast<float>(s.state_of_charge), std::span<const float>(chain->ocv_table));
    const float r0 = battery_pack_resistance(4u, 1u, static_cast<float>(chain->cell_resistance));
    const float h = 1e-4f;

    float omega = static_cast<float>(c->fit.hover_omega);
    const auto run = [&](float duty, int steps, std::vector<float>* trace) {
        for (int k = 0; k < steps; ++k) {
            uint32_t flags = 0;
            const float r_eff = motor_effective_resistance(r, 7.0f, 0.0f, omega);
            std::array<BusMotor, 4> motors;
            motors.fill(BusMotor{duty, omega / kv, r_eff, 0.0f, static_cast<float>(chain->current_max)});
            std::array<float, 4> currents{};
            const BusResult bus = bus_solve(v_source, r0, static_cast<float>(chain->pack_current_max), 12.0f,
                                            static_cast<float>(chain->esc_current_total_max), motors, currents);
            const float j = propeller_advance_ratio(0.0f, omega, dia);
            const float cq = propeller_coefficient(chain->table.cq, chain->table.j_min, chain->table.j_max, j, flags);
            const float load = propeller_torque(cq, rho, omega, dia);
            const float w_inf = motor_speed_target(duty * bus.duty_scale, bus.bus_voltage, kv, r_eff, i0, load);
            omega = motor_speed_step(omega, w_inf, motor_alpha(h, motor_time_constant(inertia, r_eff, kv)), flags);
            if (trace != nullptr) trace->push_back(omega);
        }
    };
    const float duty = static_cast<float>(c->fit.hover_duty);
    run(duty, 5000, nullptr);  // settle the float chain at its own hover
    const float w0 = omega;
    std::vector<float> trace;
    run(duty * 1.01f, 20000, &trace);
    const float wf = trace.back();
    ASSERT_GT(wf - w0, 1.0f) << "the step must move the shaft";
    const float target = w0 + 0.632120559f * (wf - w0);
    double t63 = -1.0;
    for (std::size_t k = 1; k < trace.size(); ++k) {
        if (trace[k - 1] < target && trace[k] >= target) {
            const double f = (target - trace[k - 1]) / (trace[k] - trace[k - 1]);
            t63 = (static_cast<double>(k) + f) * h;
            break;
        }
    }
    ASSERT_GT(t63, 0.0);
    EXPECT_NEAR(t63, c->fit.tau, 0.05 * c->fit.tau);
}

TEST(AirframeCompile, AnAirframeTheChainCannotLiftIsRefused) {
    AirframeSpec s = reference_quad();
    s.battery.mass = 20.0;
    const std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "airframe", 0, "propulsion"));
    const auto c = compile_airframe(s);
    ASSERT_FALSE(c.has_value());
    EXPECT_NE(c.error().context.find("cannot hover"), std::string::npos) << c.error().context;
}

TEST(AirframeCompile, CheckAirframeListsEveryProblem) {
    AirframeSpec s = reference_quad();
    s.motor.kv = 0.0;
    s.battery.cells_series = 0;
    s.rotors[1].spin_dir = 2.0;
    s.parts[0].mass = -1.0;
    s.imus[0].rate_divider = 0;
    s.name.clear();
    const std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "motor", 0, "kv"));
    EXPECT_TRUE(has_issue(issues, "battery", 0, "cells_series"));
    EXPECT_TRUE(has_issue(issues, "rotors", 1, "spin_dir"));
    EXPECT_TRUE(has_issue(issues, "parts", 0, "mass"));
    EXPECT_TRUE(has_issue(issues, "imus", 0, "rate_divider"));
    EXPECT_TRUE(has_issue(issues, "airframe", 0, "name"));
    const auto c = compile_airframe(s);
    ASSERT_FALSE(c.has_value());
    EXPECT_NE(c.error().context.find("motor[0].kv"), std::string::npos) << c.error().context;
    EXPECT_NE(c.error().context.find("rotors[1].spin_dir"), std::string::npos) << c.error().context;
    EXPECT_NE(c.error().context.find("parts[0].mass"), std::string::npos) << c.error().context;
    EXPECT_FALSE(airframe_propulsion_chain(s).has_value()) << "an unusable motor is no chain";
}

// TD-9: a rule has one owner. The model's own rules (spin_dir here) come
// from ModelType's validator, not a copy, and are listed once, at the
// element that breaks them.
TEST(AirframeCompile, AModelRuleIsListedOnceAtTheElementThatBreaksIt) {
    AirframeSpec s = reference_quad();
    s.rotors[1].spin_dir = 2.0;
    const std::vector<AirframeIssue> issues = check_airframe(s);
    ASSERT_EQ(issues.size(), 1u);
    EXPECT_TRUE(has_issue(issues, "rotors", 1, "spin_dir"));
    EXPECT_NE(issues[0].message.find("spin_dir"), std::string::npos) << issues[0].message;
}

// The whole list in one call, even when the fit cannot run: an airframe the
// chain cannot lift still lists its model-rule problems (Kat's question 2).
TEST(AirframeCompile, ModelRulesAreListedEvenWhenTheFitFails) {
    AirframeSpec s = reference_quad();
    s.battery.mass = 20.0;
    s.rotors[3].spin_dir = 0.5;
    const std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "rotors", 3, "spin_dir"));
}

// A zero or non-finite orientation is refused where it was written, before
// anything normalizes it: glm::normalize turns a zero quaternion into the
// identity, which would pass every later check.
TEST(AirframeCompile, AZeroOrNonFiniteMountOrientationIsRefusedWhereWritten) {
    const glm::dquat zero(0.0, 0.0, 0.0, 0.0);
    const double nan = std::numeric_limits<double>::quiet_NaN();

    AirframeSpec s = reference_quad();
    s.imus[0].mount.orientation = zero;
    EXPECT_TRUE(has_issue(check_airframe(s), "imus", 0, "mount")) << "IMU mount";

    s = reference_quad();
    DragSpec componentwise;
    componentwise.coeffs = glm::dvec3(0.02);
    componentwise.mount.orientation = zero;
    s.drag.push_back(componentwise);
    EXPECT_TRUE(has_issue(check_airframe(s), "drag", 0, "mount")) << "componentwise drag mount";

    s = reference_quad();
    DragSpec quadratic;
    quadratic.mode = spade::physics::drag_mode::quadratic;
    quadratic.area = 0.01;
    quadratic.coeffs = glm::dvec3(1.0, 0.0, 0.0);
    quadratic.mount.orientation = glm::dquat(nan, 0.0, 0.0, 1.0);
    s.drag.push_back(quadratic);
    EXPECT_TRUE(has_issue(check_airframe(s), "drag", 0, "mount")) << "quadratic drag mount";

    s = reference_quad();
    s.motor.mass = 0.0;
    s.prop.mass = 0.0;
    s.rotors[2].hub.orientation = zero;
    EXPECT_TRUE(has_issue(check_airframe(s), "rotors", 2, "hub")) << "a hub no part sits at";
}

// One bad hub is one issue: the motor and the propeller parts at it are not
// reported as well.
TEST(AirframeCompile, ABadHubIsReportedOnce) {
    AirframeSpec s = reference_quad();
    s.rotors[2].hub.position.x = std::numeric_limits<double>::quiet_NaN();
    const std::vector<AirframeIssue> issues = check_airframe(s);
    ASSERT_EQ(issues.size(), 1u);
    EXPECT_TRUE(has_issue(issues, "rotors", 2, "hub"));
}

// A block's shape is the block's: one issue, index 0, field "shape", however
// many hubs carry it, and checked even when the block has no mass.
TEST(AirframeCompile, ABadBlockShapeIsOneIssueOnTheBlock) {
    AirframeSpec s = reference_quad();
    s.motor.shape = BlockShape{PartShape::box, glm::dvec3(0.01, 0.0, 0.01), glm::dmat3(0.0)};
    std::vector<AirframeIssue> issues = check_airframe(s);
    ASSERT_EQ(issues.size(), 1u);
    EXPECT_TRUE(has_issue(issues, "motor", 0, "shape"));

    s = reference_quad();
    s.esc.mass = 0.0;
    s.esc.shape = BlockShape{PartShape::sphere, glm::dvec3(-1.0, 0.0, 0.0), glm::dmat3(0.0)};
    issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "esc", 0, "shape")) << "a shape is checked even with no mass";
}

// Kat's convention (2026-10-05): one EscBlock is one physical board, and its
// total caps that board's channels. Until an airframe carries several boards,
// a total is accepted only when the board drives every rotor.
TEST(AirframeCompile, AnEscTotalNeedsTheBoardToDriveEveryRotor) {
    AirframeSpec s = reference_quad();
    EXPECT_TRUE(check_airframe(s).empty()) << "4 rotors on a 4-channel board";
    s.esc.channels = 8;
    EXPECT_TRUE(has_issue(check_airframe(s), "esc", 0, "current_total"));
    s.esc.current_total = 0.0;
    EXPECT_TRUE(check_airframe(s).empty()) << "no total, nothing to misapply";
    s.esc.channels = 0;
    EXPECT_TRUE(has_issue(check_airframe(s), "esc", 0, "channels"));
}

// Silent passes, now refused: recorded-only fields still have to be numbers
// that make sense, c_rating 0 is not "unlimited", and an airframe with no drag
// given and nothing to estimate it from says so.
TEST(AirframeCompile, NothingPassesSilently) {
    AirframeSpec s = reference_quad();
    s.esc.current_continuous = std::numeric_limits<double>::quiet_NaN();
    s.motor.no_load_voltage = -1.0;
    s.battery.polarization_capacitance = -1.0;
    s.battery.c_rating = 0.0;
    std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "esc", 0, "current_continuous"));
    EXPECT_TRUE(has_issue(issues, "motor", 0, "no_load_voltage"));
    EXPECT_TRUE(has_issue(issues, "battery", 0, "polarization_capacitance"));
    EXPECT_TRUE(has_issue(issues, "battery", 0, "c_rating"));

    s = reference_quad();
    for (PartInertia& p : s.parts) p.shape = PartShape::point;
    s.motor.shape.reset();
    s.battery.shape.reset();
    issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "drag", 0, "")) << "no drag given and no shape to estimate it from";
}

TEST(AirframeCompile, DragIsEstimatedFromThePartsOrTakenAsGiven) {
    // Only the frame plate has a shape that sees the flow: the motors are
    // points, the propellers are left out, the battery is a point.
    AirframeSpec s = reference_quad();
    s.parts.resize(1);
    s.motor.shape.reset();
    s.battery.shape.reset();
    s.battery.mount.position = glm::dvec3(0.0);
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    ASSERT_EQ(c->model.drag_bodies.size(), 1u);
    const double half_rho = 0.5 * s.air_density;
    const glm::dvec3 expected = half_rho * glm::dvec3(0.03 * 0.12, 0.12 * 0.12, 0.12 * 0.03);  // C_d = 1
    EXPECT_NEAR(c->model.drag_bodies[0].coeffs.x, expected.x, 1e-6 * expected.x);
    EXPECT_NEAR(c->model.drag_bodies[0].coeffs.y, expected.y, 1e-6 * expected.y);
    EXPECT_NEAR(c->model.drag_bodies[0].coeffs.z, expected.z, 1e-6 * expected.z);
    EXPECT_EQ(c->fit.drag, Provenance::estimated);

    DragSpec given;
    given.coeffs = glm::dvec3(0.028);
    given.mount.position = glm::dvec3(0.01, 0.0, 0.0);
    s.drag.push_back(given);
    const auto g = compile_airframe(s);
    ASSERT_TRUE(g.has_value()) << g.error().context;
    ASSERT_EQ(g->model.drag_bodies.size(), 1u);
    EXPECT_EQ(g->model.drag_bodies[0].coeffs.x, 0.028f);
    EXPECT_EQ(g->model.drag_bodies[0].local_pos.x, static_cast<float>(0.01 - double{g->model.com_offset.x}));
    EXPECT_EQ(g->fit.drag, Provenance::given);
}

// Supplied componentwise drag goes into BODY axes: the drag law applies its
// coefficients along the body's axes and never reads local_orient
// (physics/forces.cpp). A mount turned 90 degrees about +Y maps its local
// axes onto body axes exactly, so the coefficients permute and stay "given".
TEST(AirframeCompile, SuppliedDragOnATurnedMountPermutesIntoBodyAxes) {
    AirframeSpec s = reference_quad();
    DragSpec d;
    d.coeffs = glm::dvec3(0.01, 0.02, 0.03);
    const double h = 0.70710678118654752;  // sqrt(1/2): a 90-degree turn about +Y
    d.mount.orientation = glm::dquat(h, 0.0, h, 0.0);
    s.drag.push_back(d);
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    ASSERT_EQ(c->model.drag_bodies.size(), 1u);
    // R_y(90): local +X lies along body -Z and local +Z along body +X.
    EXPECT_EQ(c->model.drag_bodies[0].coeffs.x, 0.03f);
    EXPECT_EQ(c->model.drag_bodies[0].coeffs.y, 0.02f);
    EXPECT_EQ(c->model.drag_bodies[0].coeffs.z, 0.01f);
    EXPECT_EQ(c->fit.drag, Provenance::given);
}

// With a tilted principal frame no permutation exists, so each body axis gets
// the coefficient that gives the exact force for motion along it:
// c_i = sum_j |M_ij|^3 c_j, M the mount -> body rotation. Cross-axis coupling
// is lost, so the provenance drops to estimated.
TEST(AirframeCompile, SuppliedDragInATiltedFrameIsProjectedOntoBodyAxes) {
    AirframeSpec s = asymmetric_quad();
    DragSpec d;
    d.coeffs = glm::dvec3(0.01, 0.02, 0.03);
    s.drag.push_back(d);
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    ASSERT_EQ(c->model.drag_bodies.size(), 1u);
    const glm::quat q = c->model.design_to_principal;
    const glm::dmat3 m = glm::mat3_cast(glm::dquat(q.w, q.x, q.y, q.z));  // mount = design axes
    for (int i = 0; i < 3; ++i) {
        double expected = 0.0;
        for (int j = 0; j < 3; ++j) {
            const double mij = std::fabs(m[j][i]);  // glm is column-major: m[j][i] is row i, column j
            expected += mij * mij * mij * d.coeffs[j];
        }
        EXPECT_NEAR(c->model.drag_bodies[0].coeffs[i], expected, 1e-6 * expected) << "body axis " << i;
    }
    EXPECT_EQ(c->fit.drag, Provenance::estimated);
}

// The drag estimate in a tilted principal frame. Each body axis sees the
// plate's projected area along it, |n_x| A_yz + |n_y| A_xz + |n_z| A_xy for
// the axis n in design coordinates (the projection of a box), with n taken
// from the model's own design_to_principal.
TEST(AirframeCompile, TheDragEstimateProjectsThePartsOntoTiltedBodyAxes) {
    AirframeSpec s = reference_quad();
    s.parts.resize(1);  // the 0.12 x 0.03 x 0.12 m plate: the one shape that sees the flow
    s.motor.shape.reset();
    s.battery.shape.reset();
    s.battery.mount.position = glm::dvec3(0.03, -0.03, 0.02);  // off-centre: tilts the principal axes
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const glm::quat q = c->model.design_to_principal;
    ASSERT_FALSE(q.w == 1.0f && q.x == 0.0f && q.y == 0.0f && q.z == 0.0f) << "the battery must tilt the axes";
    ASSERT_EQ(c->model.drag_bodies.size(), 1u);
    const glm::dmat3 r = glm::mat3_cast(glm::dquat(q.w, q.x, q.y, q.z));  // design -> body
    const double a_yz = 0.03 * 0.12;
    const double a_xz = 0.12 * 0.12;
    const double a_xy = 0.12 * 0.03;
    for (int i = 0; i < 3; ++i) {
        // Body axis i in design coordinates is row i of r (glm: r[j][i]).
        const double area = std::fabs(r[0][i]) * a_yz + std::fabs(r[1][i]) * a_xz + std::fabs(r[2][i]) * a_xy;
        const double expected = 0.5 * s.air_density * area;  // C_d = 1
        EXPECT_NEAR(c->model.drag_bodies[0].coeffs[i], expected, 1e-6 * expected) << "body axis " << i;
    }
    EXPECT_EQ(c->fit.drag, Provenance::estimated);
}

// A compile whose propulsion reaches the ESC total, with braking on. Counting
// |I| toward the total let the braking motors drag the full-duty thrust from
// 18.0 N to 16.7 N per rotor (a Python port of the chain), so an airframe
// hovering at 17.4 N per rotor was refused as unable to hover. Counting drive
// current, it compiles, and full duty is held at the board's total.
TEST(AirframeCompile, AnAirframeThatReachesTheEscTotalWithBrakingCompiles) {
    AirframeSpec s = reference_quad();
    s.prop.cq_table = {{0.0, 0.0049}, {0.8, 0.003}};
    s.esc.braking = true;
    s.esc.current_total = 120.0;
    const double hover_thrust = 17.4;  // N per rotor
    s.battery.mass = 4.0 * hover_thrust / s.gravity - (0.526 - 0.18);  // the battery carries the rest
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    EXPECT_NEAR(c->fit.hover_thrust, hover_thrust, 1e-4);
    const auto chain = airframe_propulsion_chain(s);
    ASSERT_TRUE(chain.has_value()) << chain.error().context;
    EXPECT_EQ(chain->esc_current_total_max, 120.0);
    EXPECT_LT(chain->current_min, 0.0) << "braking";
    const SteadyPoint full = steady_state_at_duty(*chain, 1.0, s.air_density, 0.0, s.state_of_charge);
    EXPECT_NE(full.flags & propulsion_flags::esc_total_limited, 0u) << "full duty reaches the board's total";
    EXPECT_GT(full.thrust, hover_thrust);
}

// DETERMINISM (Kat's question 1). The reference quad's compiled model has one
// identity, here and on every platform. FINAL under TD-12's discipline: the
// Docker gcc leg reproduced the MSVC value at 3e4b589.
TEST(AirframeCompile, TheReferenceQuadHasAPinnedIdentity) {
    const auto a = compile_airframe(reference_quad());
    const auto b = compile_airframe(reference_quad());
    ASSERT_TRUE(a.has_value() && b.has_value());
    EXPECT_EQ(model_identity(a->model), model_identity(b->model)) << "the same spec, compiled twice";
    // msvc-ninja-release at 1b61a41; gcc-13 leg at 3e4b589 (2026-10-04).
    constexpr uint64_t kPinned = 0x9be262bdf825ba41ull;
    EXPECT_EQ(model_identity(a->model), kPinned) << std::hex << model_identity(a->model);
}

// ===========================================================================
// PARTS (CORE-7; the parts and meshes plan, Task A4). One record gives both
// the inertia and the model part (TD-9). Each sphere, box, cylinder or tube
// record, and each shaped motor, ESC or battery block, becomes a body-frame
// wall part: one primitive at the identity in the record's frame, its size
// rounded to float once, and its mount the record's pose, converted as every
// other mount is. min_feature is the solid's smallest dimension: a sphere's
// diameter, a box's smallest extent, a cylinder's or a tube's min(2r, length).
// ===========================================================================

TEST(AirframeCompile, ShapedPartsBecomeWallPartsInTheBodyFrame) {
    // The reference quad: a box frame, four tube arms, a cylinder motor at
    // each hub, a box battery, and an ESC with no shape.
    const AirframeSpec s = reference_quad();
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const ModelType& m = c->model;
    ASSERT_EQ(m.parts.size(), 10u) << "frame, four arms, four motors, the battery; no ESC, never a propeller";
    EXPECT_TRUE(m.issues().empty()) << "the compiled parts pass the model's rules";

    const std::array<const char*, 10> names = {"parts[0]", "parts[1]", "parts[2]", "parts[3]", "parts[4]",
                                               "motor[0]", "motor[1]", "motor[2]", "motor[3]", "battery"};
    const std::array<const char*, 10> roles = {"", "", "", "", "", "motor", "motor", "motor", "motor", "battery"};
    for (std::size_t i = 0; i < m.parts.size(); ++i) {
        const PartDesc& p = m.parts[i];
        EXPECT_EQ(p.name, names[i]);
        EXPECT_EQ(p.role, roles[i]) << p.name;
        EXPECT_EQ(p.attach, PartAttach::body) << p.name << ": rigid, a motor's included";
        EXPECT_EQ(p.rotor, 0u) << p.name;
        EXPECT_EQ(p.surface.kind, PartSurface::Kind::wall) << p.name;
        EXPECT_EQ(p.surface.inv_permeability, 0.0f) << p.name;
        EXPECT_EQ(p.surface.inertial_coeff, 0.0f) << p.name;
    }

    // The frame, a 0.12 x 0.03 x 0.12 m box: its half extents, and its
    // smallest extent.
    expect_solid(m.parts[0], spade::SdfPrim::box, glm::vec4(f(0.06), f(0.015), f(0.06), 0.0f), f(0.03));
    // An arm, a tube of radius 0.006 and length 0.11: the solid cylinder of
    // its outer wall. 2r is the smaller.
    for (std::size_t i = 1; i <= 4; ++i) {
        expect_solid(m.parts[i], spade::SdfPrim::cylinder, glm::vec4(f(0.006), f(0.055), 0.0f, 0.0f), f(0.012));
    }
    // A motor, a cylinder of radius 0.014 and length 0.018: the length is the
    // smaller.
    for (std::size_t i = 5; i <= 8; ++i) {
        expect_solid(m.parts[i], spade::SdfPrim::cylinder, glm::vec4(f(0.014), f(0.009), 0.0f, 0.0f), f(0.018));
    }
    // The battery, a 0.07 x 0.035 x 0.035 m box.
    expect_solid(m.parts[9], spade::SdfPrim::box, glm::vec4(f(0.035), f(0.0175), f(0.0175), 0.0f), f(0.035));

    // Mounts in the body frame by the rule every mount uses, r_b = R_bd (p_d - c).
    // The reference quad's principal frame is its design frame
    // (ASymmetricQuadKeepsItsAxesAndMovesMountsByTheCentreOfMass), so r_b = p_d - c.
    const glm::dvec3 com(m.com_offset);
    const auto expect_at = [&](const PartDesc& p, const glm::dvec3& design) {
        const glm::dvec3 e = design - com;
        EXPECT_EQ(p.mount_pos.x, f(e.x)) << p.name;
        EXPECT_EQ(p.mount_pos.y, f(e.y)) << p.name;
        EXPECT_EQ(p.mount_pos.z, f(e.z)) << p.name;
    };
    for (std::size_t k = 0; k < 5; ++k) expect_at(m.parts[k], s.parts[k].position);
    for (std::size_t k = 0; k < 4; ++k) {
        expect_at(m.parts[5 + k], s.rotors[k].hub.position);
        // A motor sits at its rotor's hub, posed as the rotor is.
        EXPECT_TRUE(m.parts[5 + k].mount_pos == m.rotors[k].local_pos) << "motor " << k;
        EXPECT_TRUE(m.parts[5 + k].mount_orient == m.rotors[k].local_orient) << "motor " << k;
    }
    expect_at(m.parts[9], s.battery.mount.position);
    // An unturned record keeps the identity. An arm's mount turns its +Y, the
    // cylinder's axis, onto the arm's line: -X, +Z, -X, +Z.
    EXPECT_TRUE(m.parts[0].mount_orient == glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    const std::array<glm::vec3, 4> lines = {glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f),
                                            glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)};
    for (std::size_t k = 0; k < 4; ++k) {
        const glm::vec3 axis = m.parts[1 + k].mount_orient * glm::vec3(0.0f, 1.0f, 0.0f);
        EXPECT_NEAR(axis.x, lines[k].x, 1e-6f) << "arm " << k;
        EXPECT_NEAR(axis.y, lines[k].y, 1e-6f) << "arm " << k;
        EXPECT_NEAR(axis.z, lines[k].z, 1e-6f) << "arm " << k;
    }
}

// The table's other rows. A sphere gives its solid, min_feature its diameter;
// a point, a tensor or a cylinder of length 0 (a disc, which the mass
// accepts, with no volume) gives no part, and neither does a record marked
// none. A record's name and role are its part's.
TEST(AirframeCompile, ASphereGivesItsSolidAndNoneAPointATensorOrADiscGivesNoPart) {
    AirframeSpec s = reference_quad();
    s.parts[0].geometry = PartGeometrySpec{.source = GeometrySource::inertia_shape, .name = "frame", .role = "body"};
    s.parts[2].geometry.source = GeometrySource::none;  // an arm, mass only
    PartInertia gps = make_part(PartShape::sphere, 0.008, glm::dvec3(0.0, 0.03, 0.0), glm::dvec3(0.012, 0.0, 0.0));
    gps.geometry = PartGeometrySpec{.source = GeometrySource::inertia_shape, .name = "gps", .role = "sensor"};
    s.parts.push_back(gps);  // record 5
    s.parts.push_back(make_part(PartShape::point, 0.004, glm::dvec3(0.0, 0.01, 0.0), glm::dvec3(0.0)));  // record 6
    PartInertia receiver = make_part(PartShape::tensor, 0.004, glm::dvec3(0.0, -0.01, 0.0), glm::dvec3(0.0));
    receiver.tensor = glm::dmat3(1e-7);
    s.parts.push_back(receiver);  // record 7
    s.parts.push_back(make_part(PartShape::cylinder, 0.002, glm::dvec3(0.0, 0.016, 0.0),
                                glm::dvec3(0.01, 0.0, 0.0)));  // record 8, a disc
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const ModelType& m = c->model;

    const std::array<const char*, 10> names = {"frame", "parts[1]", "parts[3]", "parts[4]", "gps",
                                               "motor[0]", "motor[1]", "motor[2]", "motor[3]", "battery"};
    ASSERT_EQ(m.parts.size(), names.size()) << "no part for the arm marked none, the point, the tensor or the disc";
    for (std::size_t i = 0; i < names.size(); ++i) EXPECT_EQ(m.parts[i].name, names[i]);
    EXPECT_EQ(m.parts[0].role, "body");
    EXPECT_EQ(m.parts[4].role, "sensor");
    expect_solid(m.parts[4], spade::SdfPrim::sphere, glm::vec4(f(0.012), 0.0f, 0.0f, 0.0f), f(0.024));
    EXPECT_EQ(m.parts[4].surface.kind, PartSurface::Kind::wall);
}

// The propeller never becomes a part: its disc is mass only, and the rotor is
// an actuator disc. A shaped propeller still gives its mass. A shaped ESC
// becomes "esc", at its mount.
TEST(AirframeCompile, APropellerIsNeverAGeometryPart) {
    AirframeSpec s = reference_quad();
    s.prop.shape = BlockShape{.shape = PartShape::cylinder, .size = glm::dvec3(0.0762, 0.004, 0.0),
                              .tensor = glm::dmat3(0.0)};
    s.esc.shape = BlockShape{.shape = PartShape::box, .size = glm::dvec3(0.03, 0.006, 0.03),
                             .tensor = glm::dmat3(0.0)};
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const ModelType& m = c->model;
    ASSERT_EQ(m.parts.size(), 11u) << "the shaped ESC joins; the shaped propellers do not";
    for (const PartDesc& p : m.parts) {
        EXPECT_EQ(p.name.find("prop"), std::string::npos) << p.name;
        EXPECT_NE(p.role, "prop") << p.name;
    }
    EXPECT_EQ(m.parts[9].name, "esc");
    EXPECT_EQ(m.parts[9].role, "esc");
    expect_solid(m.parts[9], spade::SdfPrim::box, glm::vec4(f(0.015), f(0.003), f(0.015), 0.0f), f(0.006));
    const glm::dvec3 at = s.esc.mount.position - glm::dvec3(m.com_offset);
    EXPECT_NEAR(m.parts[9].mount_pos.x, at.x, 1e-6);
    EXPECT_NEAR(m.parts[9].mount_pos.y, at.y, 1e-6);
    EXPECT_NEAR(m.parts[9].mount_pos.z, at.z, 1e-6);
    EXPECT_EQ(m.parts[10].name, "battery");
    EXPECT_NEAR(m.body.mass, 0.526, 1e-6) << "every propeller's mass is still the body's";
}

// A part's problem is listed at its record, in the one call that lists
// everything (Kat's question 2): a name the model's rules refuse at
// parts[k].geometry.name, an unknown source at parts[k].geometry.source. A
// block's part has a fixed name, so when it repeats a record's, the record is
// at fault. Neither stops the fit, which still lists its own problem.
TEST(AirframeCompile, APartProblemIsListedAtItsRecord) {
    AirframeSpec s = reference_quad();
    s.parts[1].geometry.name = "arm";
    s.parts[3].geometry.name = "arm";      // repeats parts[1]'s
    s.parts[0].geometry.name = "battery";  // the battery block's part repeats it
    s.parts[2].geometry.source = static_cast<GeometrySource>(7);
    s.battery.mass = 20.0;  // and the chain cannot lift it
    const std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "parts", 3, "geometry.name")) << "the later of two equal names";
    EXPECT_FALSE(has_issue(issues, "parts", 1, "geometry.name"));
    EXPECT_TRUE(has_issue(issues, "parts", 0, "geometry.name")) << "a block's part's name is fixed";
    EXPECT_FALSE(has_issue(issues, "battery", 0, "name"));
    EXPECT_TRUE(has_issue(issues, "parts", 2, "geometry.source"));
    EXPECT_TRUE(has_issue(issues, "airframe", 0, "propulsion")) << "the fit still runs, and lists its own";

    s.battery.mass = reference_quad().battery.mass;
    const auto c = compile_airframe(s);
    ASSERT_FALSE(c.has_value());
    EXPECT_NE(c.error().context.find("parts[3].geometry.name"), std::string::npos) << c.error().context;
    EXPECT_NE(c.error().context.find("parts[0].geometry.name"), std::string::npos) << c.error().context;
    EXPECT_NE(c.error().context.find("parts[2].geometry.source"), std::string::npos) << c.error().context;
}

// TD-9: no mass or drag rule reads a part's geometry. The reference quad as
// is, and with every record's geometry none, compile to the same composite,
// fit and model, bit for bit, but for the parts; model_identity, which folds
// every field but visual_ref and the parts, agrees. Physics' answer: a part
// does not touch a drag element.
TEST(AirframeCompile, PartsChangeNeitherTheMassNorTheDrag) {
    AirframeSpec bare = reference_quad();
    for (PartInertia& p : bare.parts) p.geometry.source = GeometrySource::none;
    const auto a = compile_airframe(reference_quad());
    const auto b = compile_airframe(bare);
    ASSERT_TRUE(a.has_value()) << a.error().context;
    ASSERT_TRUE(b.has_value()) << b.error().context;
    ASSERT_EQ(a->model.parts.size(), 10u);
    ASSERT_EQ(b->model.parts.size(), 5u) << "the shaped blocks still give theirs";

    // The composite.
    EXPECT_EQ(std::bit_cast<uint32_t>(a->inertia.mass), std::bit_cast<uint32_t>(b->inertia.mass));
    EXPECT_TRUE(same_bits(a->inertia.center_of_mass, b->inertia.center_of_mass));
    EXPECT_TRUE(same_bits(a->inertia.principal_moments, b->inertia.principal_moments));
    EXPECT_TRUE(same_bits(a->inertia.design_to_body, b->inertia.design_to_body));
    EXPECT_TRUE(a->inertia.inertia_design == b->inertia.inertia_design);

    // The model: its body, its design frame, its rotors and its drag.
    const ModelType& ma = a->model;
    const ModelType& mb = b->model;
    EXPECT_EQ(std::bit_cast<uint32_t>(ma.body.mass), std::bit_cast<uint32_t>(mb.body.mass));
    EXPECT_TRUE(same_bits(ma.body.inertia_diag, mb.body.inertia_diag));
    EXPECT_TRUE(same_bits(ma.design_to_principal, mb.design_to_principal));
    EXPECT_TRUE(same_bits(ma.com_offset, mb.com_offset));
    ASSERT_EQ(ma.rotors.size(), mb.rotors.size());
    for (std::size_t k = 0; k < ma.rotors.size(); ++k) {
        EXPECT_TRUE(same_bits(ma.rotors[k].local_pos, mb.rotors[k].local_pos)) << "rotor " << k;
        EXPECT_TRUE(same_bits(ma.rotors[k].local_orient, mb.rotors[k].local_orient)) << "rotor " << k;
        EXPECT_EQ(std::bit_cast<uint32_t>(ma.rotors[k].tau), std::bit_cast<uint32_t>(mb.rotors[k].tau));
        EXPECT_EQ(std::bit_cast<uint32_t>(ma.rotors[k].thrust_coeff),
                  std::bit_cast<uint32_t>(mb.rotors[k].thrust_coeff));
        EXPECT_EQ(std::bit_cast<uint32_t>(ma.rotors[k].torque_coeff),
                  std::bit_cast<uint32_t>(mb.rotors[k].torque_coeff));
    }
    ASSERT_EQ(ma.drag_bodies.size(), mb.drag_bodies.size());
    for (std::size_t k = 0; k < ma.drag_bodies.size(); ++k) {
        EXPECT_EQ(ma.drag_bodies[k].mode, mb.drag_bodies[k].mode) << "drag " << k;
        EXPECT_EQ(std::bit_cast<uint32_t>(ma.drag_bodies[k].area), std::bit_cast<uint32_t>(mb.drag_bodies[k].area));
        EXPECT_TRUE(same_bits(ma.drag_bodies[k].coeffs, mb.drag_bodies[k].coeffs)) << "drag " << k;
        EXPECT_TRUE(same_bits(ma.drag_bodies[k].local_pos, mb.drag_bodies[k].local_pos)) << "drag " << k;
    }
    EXPECT_EQ(model_identity(ma), model_identity(mb)) << "every folded field, bit for bit";

    // The fit.
    EXPECT_EQ(a->fit.hover_duty, b->fit.hover_duty);
    EXPECT_EQ(a->fit.hover_omega, b->fit.hover_omega);
    EXPECT_EQ(a->fit.hover_thrust, b->fit.hover_thrust);
    EXPECT_EQ(a->fit.thrust_coeff, b->fit.thrust_coeff);
    EXPECT_EQ(a->fit.torque_coeff, b->fit.torque_coeff);
    EXPECT_EQ(a->fit.tau, b->fit.tau);
    EXPECT_EQ(a->fit.rotor_inertia, b->fit.rotor_inertia);
    EXPECT_EQ(a->fit.solver_flags, b->fit.solver_flags);
    EXPECT_EQ(a->fit.drag, b->fit.drag);
}

// The compiled parts are a model's parts like any other (vehicles/
// part_pose.hpp). Posed with the body, the wall reads inside at the frame's
// centre, a motor's and the battery's, and outside above and below the
// airframe, in posed_program() and sample_parts() alike. Distances are worked
// by hand from the solids; posing is rigid, so a turned and moved body reads
// the same.
TEST(AirframeCompile, ThePosedReferenceQuadIsSolidWhereItsPartsAre) {
    const auto c = compile_airframe(reference_quad());
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const ModelType& m = c->model;
    ASSERT_EQ(m.parts.size(), 10u);

    spade::BodyState b{};
    b.pos = glm::vec3(0.5f, 1.0f, -0.25f);
    b.orient = glm::normalize(glm::quat(0.70710678f, 0.0f, 0.70710678f, 0.0f));  // a quarter turn about +Y
    const auto posed = posed_parts(m, b, {}, {}, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    const auto wall = posed_program(m, *posed);
    ASSERT_TRUE(wall.has_value()) << wall.error().context;

    // A design point in the world: x = pos + orient (p_d - c), the principal
    // frame being the design frame here.
    const glm::vec3 com = m.com_offset;
    const auto world = [&](glm::vec3 design) { return b.pos + b.orient * (design - com); };
    struct Probe {
        glm::vec3 design;
        float distance;
        uint32_t owner;
        const char* where;
    };
    const std::array<Probe, 5> probes = {{
        {.design = glm::vec3(0.0f), .distance = -0.015f, .owner = 0u,
         .where = "the frame's centre: 0.015 m inside its top and bottom"},
        {.design = glm::vec3(0.11f, 0.02f, 0.0f), .distance = -0.009f, .owner = 5u,
         .where = "motor 0's centre: 0.009 m inside its caps"},
        {.design = glm::vec3(0.0f, -0.03f, 0.0f), .distance = -0.0175f, .owner = 9u,
         .where = "the battery's centre: 0.0175 m inside four faces"},
        {.design = glm::vec3(0.0f, 0.2f, 0.0f), .distance = 0.185f, .owner = 0u,
         .where = "above the frame's top, at y = 0.015"},
        {.design = glm::vec3(0.0f, -0.1f, 0.0f), .distance = 0.0525f, .owner = 9u,
         .where = "below the battery's bottom, at y = -0.0475"},
    }};
    for (const Probe& p : probes) {
        const glm::vec3 x = world(p.design);
        EXPECT_NEAR(spade::eval(*wall, x), p.distance, 1e-6f) << p.where;
        const PartSample at = sample_parts(m, *posed, x);
        EXPECT_NEAR(at.distance, p.distance, 1e-6f) << p.where;
        EXPECT_EQ(at.owner, p.owner) << p.where;
        EXPECT_EQ(at.surface, PartSurface::Kind::wall) << p.where;
        EXPECT_TRUE(at.velocity == glm::vec3(0.0f)) << p.where << ": the body is at rest";
    }
}

// model_identity never folds parts (CORE-7, agreed with Interface 2026-10-12):
// the reference quad's pin above holds whatever parts it carries, and only
// parts_identity() moves. A CFD case pins the pair.
TEST(AirframeCompile, PartsMoveOnlyThePartsIdentity) {
    const auto c = compile_airframe(reference_quad());
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const ModelType& m = c->model;
    ASSERT_EQ(m.parts.size(), 10u) << "the compile emits parts (A4)";
    constexpr uint64_t kPinned = 0x9be262bdf825ba41ull;  // the pin above
    EXPECT_EQ(model_identity(m), kPinned) << std::hex << model_identity(m);

    // The compiled quad's parts_identity: PINNED (msvc-ninja-release on
    // core/parts-identity, 2026-10-12; the gcc leg reproduces it). It is
    // the compile's own output read back, a regression pin; the independent
    // check is PartsIdentity.AFixedModelHasAPinnedValue (test_model_parts.cpp).
    // The coming motor-offset task moves the motors' mounts, so it moves this
    // value: a named regeneration, never a silent one.
    constexpr uint64_t kPartsPinned = 0x2578117f1aa83070ull;
    EXPECT_EQ(parts_identity(m), kPartsPinned) << std::hex << parts_identity(m);
    EXPECT_EQ(parts_identity(m), parts_identity(compile_airframe(reference_quad())->model)) << "compiled twice";

    // Without its parts: the constant, and the same model_identity.
    ModelType bare = m;
    bare.parts.clear();
    EXPECT_EQ(model_identity(bare), kPinned);
    EXPECT_EQ(parts_identity(bare), kNoPartsIdentity) << std::hex << parts_identity(bare);

    // A part changed (an arm's mount, one ulp; the battery's min_feature) or
    // added (a canopy): parts_identity moves, model_identity does not.
    ModelType nudged = m;
    nudged.parts[1].mount_pos.x = std::nextafter(nudged.parts[1].mount_pos.x, 1.0f);
    ModelType finer = m;
    finer.parts[9].min_feature = 0.03f;
    spade::SdfNode canopy_box{};
    canopy_box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    canopy_box.op = static_cast<uint32_t>(spade::SdfOp::none);
    canopy_box.params = glm::vec4(0.04f, 0.01f, 0.04f, 0.0f);
    PartDesc canopy{};
    canopy.name = "canopy";
    canopy.role = "body";
    canopy.mount_pos = glm::vec3(0.0f, 0.03f, 0.0f);
    canopy.shape.transforms.push_back(spade::SdfTransform{});
    canopy.shape.nodes.push_back(canopy_box);
    canopy.min_feature = 0.02f;
    ModelType dressed = m;
    dressed.parts.push_back(canopy);
    ASSERT_TRUE(dressed.validate().has_value());
    for (const ModelType* changed : {&nudged, &finer, &dressed}) {
        EXPECT_EQ(model_identity(*changed), kPinned) << "parts never enter model_identity";
        EXPECT_NE(parts_identity(*changed), kPartsPinned);
        EXPECT_NE(parts_identity(*changed), kNoPartsIdentity);
    }
}
