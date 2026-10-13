#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
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
#include "world/mesh.hpp"
#include "world/mesh_import.hpp"
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

// RotorSpec::pusher (the lead's ruling, 2026-10-12): a rotor is a tractor
// unless it says otherwise, and an airframe of pushers is a valid airframe.
TEST(AirframeCompile, ARotorIsATractorUnlessItIsAPusher) {
    EXPECT_FALSE(RotorSpec{}.pusher) << "the usual top-mounted multirotor prop";
    AirframeSpec s = reference_quad();
    for (const RotorSpec& r : s.rotors) EXPECT_FALSE(r.pusher);
    for (RotorSpec& r : s.rotors) r.pusher = true;
    EXPECT_TRUE(check_airframe(s).empty());
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    EXPECT_EQ(c->model.rotors.size(), 4u);
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

// A zero or non-finite hub orientation is the hub's one issue, whichever side
// its motor is on. The motor, moved off its hub along the hub's +Y (THE
// MOTOR'S PLACE, below), stays on a hub with no usable orientation, so it
// lists nothing of its own and makes no NaN position (Physics' guard,
// 2026-10-12). It passed before the motor moved too, when the motor sat on
// its hub; it pins the guard against a regression.
TEST(AirframeCompile, ABadHubOrientationIsReportedOnceWithItsMotor) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const std::array<glm::dquat, 2> orientations = {glm::dquat(0.0, 0.0, 0.0, 0.0), glm::dquat(nan, 0.0, 0.0, 1.0)};
    for (const glm::dquat& q : orientations) {
        for (const bool pusher : {false, true}) {
            AirframeSpec s = reference_quad();
            s.rotors[2].hub.orientation = q;
            s.rotors[2].pusher = pusher;
            const std::vector<AirframeIssue> issues = check_airframe(s);
            ASSERT_EQ(issues.size(), 1u) << (pusher ? "a pusher" : "a tractor");
            EXPECT_TRUE(has_issue(issues, "rotors", 2, "hub"));
            for (const AirframeIssue& i : issues) EXPECT_NE(i.kind, "motor") << i.field << ": " << i.message;
        }
    }
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
// identity, here and on every platform. FINAL under TD-12's discipline: a
// fresh (unseeded) Docker gcc leg reproduced the regenerated value below at
// 397248a.
TEST(AirframeCompile, TheReferenceQuadHasAPinnedIdentity) {
    const auto a = compile_airframe(reference_quad());
    const auto b = compile_airframe(reference_quad());
    ASSERT_TRUE(a.has_value() && b.has_value());
    EXPECT_EQ(model_identity(a->model), model_identity(b->model)) << "the same spec, compiled twice";
    // REGENERATED 2026-10-13 (TD-1). Cause: the motor record offset half its
    // extent off the rotor plane, 2026-10-12, the lead's ruling
    // (RotorSpec::pusher). The reference quad's motors are tractors, so each
    // motor's mass and solid moved 0.009 m below its hub; the centre of mass,
    // the inertia and every body-frame mount moved with them. Its drag is the
    // estimate: one element at the centre of mass, so it follows the moved
    // centre; its coefficients (projected areas) do not change. Was
    // 0x9be262bdf825ba41 (msvc-ninja-release at 1b61a41; gcc-13 leg at
    // 3e4b589, 2026-10-04). Now msvc-ninja-release at 68720be; gcc-13 fresh
    // leg at 397248a (2026-10-13).
    constexpr uint64_t kPinned = 0x78fdc769a0503ff6ull;
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
    // The reference quad: a box frame, four tube arms, a cylinder motor under
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
        // A motor sits under its propeller, half its length (0.009) below its
        // rotor's hub (a tractor, RotorSpec::pusher false), posed as the rotor
        // is. THE MOTOR'S PLACE, below, works the rule through.
        expect_at(m.parts[5 + k], s.rotors[k].hub.position - glm::dvec3(0.0, 0.5 * s.motor.shape->size.y, 0.0));
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
        {.design = glm::vec3(0.11f, 0.011f, 0.0f), .distance = -0.009f, .owner = 5u,
         .where = "motor 0's centre, 0.009 m under its hub: 0.009 m inside its caps"},
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
    constexpr uint64_t kPinned = 0x78fdc769a0503ff6ull;  // the pin above
    EXPECT_EQ(model_identity(m), kPinned) << std::hex << model_identity(m);

    // The compiled quad's parts_identity: PINNED. It is the compile's own
    // output read back, a regression pin; the independent check is
    // PartsIdentity.AFixedModelHasAPinnedValue (test_model_parts.cpp).
    // REGENERATED 2026-10-13 (TD-1). Cause: the motor record offset half its
    // extent off the rotor plane, 2026-10-12, the lead's ruling
    // (RotorSpec::pusher): the motors' mounts moved, and so did the centre of
    // mass every mount is measured from. Was 0x2578117f1aa83070
    // (msvc-ninja-release on core/parts-identity, 2026-10-12; the gcc leg
    // reproduced it). Now msvc-ninja-release at 68720be; gcc-13 fresh leg at
    // 397248a (2026-10-13), final under TD-12.
    constexpr uint64_t kPartsPinned = 0x1f0c49d9fd04ed42ull;
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

// ===========================================================================
// THE MOTOR'S PLACE (the lead's ruling, 2026-10-12; RotorSpec::pusher). A
// motor's one record, its mass and its solid together (TD-9), sits beside its
// propeller rather than across the rotor plane: half its extent along the
// thrust axis (the hub's +Y) behind the hub for a tractor, ahead of it for a
// pusher. Half its extent is a cylinder's or a tube's length / 2, a box's y
// extent / 2, a sphere's radius; a point or a tensor stays on the hub. The
// propeller, the rotor and the fit stay as they were. Expectations are worked
// by hand from the spec's numbers (TD-4): the composite's centre of mass is
// sum(m p) / sum(m) over every record, and a motor's solid is read back in
// the design frame, p_d = R_bd^T r_b + c.
// ===========================================================================

namespace {

// The design-frame centre of mass of every record the compile gathers: the
// spec's parts, a motor at motors[k] and a propeller on the hub of each rotor,
// the ESC and the battery.
[[nodiscard]] glm::dvec3 centre_of_mass(const AirframeSpec& s, const std::vector<glm::dvec3>& motors) {
    double mass = 0.0;
    glm::dvec3 moment(0.0);
    const auto add = [&](double m, const glm::dvec3& p) {
        mass += m;
        moment += m * p;
    };
    for (const PartInertia& p : s.parts) add(p.mass, p.position);
    for (std::size_t k = 0; k < s.rotors.size(); ++k) {
        add(s.motor.mass, motors[k]);
        add(s.prop.mass, s.rotors[k].hub.position);
    }
    add(s.esc.mass, s.esc.mount.position);
    add(s.battery.mass, s.battery.mount.position);
    return moment / mass;
}

// A body-frame point of a compiled model, back in its design frame.
[[nodiscard]] glm::dvec3 to_design(const ModelType& m, const glm::vec3& body) {
    const glm::quat q = m.design_to_principal;
    const glm::dmat3 r_bd = glm::mat3_cast(glm::dquat(q.w, q.x, q.y, q.z));
    return glm::transpose(r_bd) * glm::dvec3(body) + glm::dvec3(m.com_offset);
}

void expect_near(const glm::dvec3& got, const glm::dvec3& want, double tolerance, const std::string& what) {
    EXPECT_NEAR(got.x, want.x, tolerance) << what;
    EXPECT_NEAR(got.y, want.y, tolerance) << what;
    EXPECT_NEAR(got.z, want.z, tolerance) << what;
}

// Compiles `s`, whose motors the caller expects at `motors` (design frame),
// and checks the motors' mass through the composite's centre of mass, their
// solids (one per rotor when `solid`, else none), and that every rotor, the
// propeller's plane, still sits on its hub.
void expect_motors_at(const AirframeSpec& s, const std::vector<glm::dvec3>& motors, bool solid,
                      const std::string& what) {
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << what << ": " << c.error().context;
    const ModelType& m = c->model;
    // The mass. The compile rounds the centre of mass to float once.
    expect_near(glm::dvec3(m.com_offset), centre_of_mass(s, motors), 1e-7, what + ": the centre of mass");
    for (std::size_t k = 0; k < s.rotors.size(); ++k) {
        const std::string rotor = what + ", rotor " + std::to_string(k);
        expect_near(to_design(m, m.rotors[k].local_pos), s.rotors[k].hub.position, 1e-6, rotor + ": its hub");
        // The solid: where the mass is, posed as the rotor is.
        const std::string name = "motor[" + std::to_string(k) + "]";
        int found = 0;
        for (const PartDesc& p : m.parts) {
            if (p.name != name) continue;
            ++found;
            expect_near(to_design(m, p.mount_pos), motors[k], 1e-6, rotor + ": the motor's solid");
            EXPECT_TRUE(p.mount_orient == m.rotors[k].local_orient) << rotor;
        }
        EXPECT_EQ(found, solid ? 1 : 0) << rotor << ": the motor's part";
    }
}

}  // namespace

// The reference quad's motor is a cylinder 0.018 long, on hubs 0.02 above the
// frame with thrust along +Y. A tractor's motor hangs under its propeller,
// centred 0.009 below the hub (y 0.002 to 0.020); a pusher's stands over it,
// centred 0.009 above (y 0.020 to 0.038). Each rotor says which it is. The
// rotor and the fit are the same either way: the propeller does not move, and
// neither does the mass.
TEST(AirframeCompile, AMotorSitsHalfItsLengthBehindItsPropeller) {
    const double half = 0.009;
    const std::array<std::array<bool, 4>, 3> layouts = {{
        {{false, false, false, false}},
        {{true, true, true, true}},
        {{false, true, false, true}},
    }};
    const std::array<const char*, 3> names = {"every rotor a tractor", "every rotor a pusher",
                                              "rotors 1 and 3 pushers"};
    for (std::size_t i = 0; i < layouts.size(); ++i) {
        AirframeSpec s = reference_quad();
        std::vector<glm::dvec3> motors;
        for (std::size_t k = 0; k < 4; ++k) {
            s.rotors[k].pusher = layouts[i][k];
            motors.push_back(s.rotors[k].hub.position + glm::dvec3(0.0, layouts[i][k] ? half : -half, 0.0));
        }
        expect_motors_at(s, motors, true, names[i]);
    }

    AirframeSpec pushers = reference_quad();
    for (RotorSpec& r : pushers.rotors) r.pusher = true;
    const auto a = compile_airframe(reference_quad());
    const auto b = compile_airframe(pushers);
    ASSERT_TRUE(a.has_value()) << a.error().context;
    ASSERT_TRUE(b.has_value()) << b.error().context;
    EXPECT_EQ(a->model.body.mass, b->model.body.mass);
    EXPECT_EQ(a->fit.hover_duty, b->fit.hover_duty);
    EXPECT_EQ(a->fit.hover_omega, b->fit.hover_omega);
    EXPECT_EQ(a->fit.thrust_coeff, b->fit.thrust_coeff);
    EXPECT_EQ(a->fit.torque_coeff, b->fit.torque_coeff);
    EXPECT_EQ(a->fit.tau, b->fit.tau);
    for (std::size_t k = 0; k < 4; ++k) {
        const RotorDesc& ra = a->model.rotors[k];
        const RotorDesc& rb = b->model.rotors[k];
        EXPECT_EQ(ra.spin_dir, rb.spin_dir) << "rotor " << k << ": spin is the torque's, not the motor's side";
        EXPECT_EQ(ra.radius, rb.radius) << "rotor " << k;
        EXPECT_EQ(ra.thrust_coeff, rb.thrust_coeff) << "rotor " << k;
        EXPECT_EQ(ra.torque_coeff, rb.torque_coeff) << "rotor " << k;
        EXPECT_EQ(ra.tau, rb.tau) << "rotor " << k;
    }
}

// Half its extent along the thrust axis, local +Y, for each inertia shape: a
// cylinder's or a tube's length / 2, a box's y extent / 2, a sphere's radius.
// A point mass (a motor with no shape) or a tensor has no extent, so it stays
// on the hub and gives no part.
TEST(AirframeCompile, AMotorMovesHalfItsExtentAlongTheThrustAxis) {
    struct Case {
        std::optional<BlockShape> shape;
        double half;
        bool solid;
        const char* what;
    };
    const std::array<Case, 6> cases = {{
        {BlockShape{PartShape::cylinder, glm::dvec3(0.014, 0.018, 0.0), glm::dmat3(0.0)}, 0.009, true, "a cylinder"},
        {BlockShape{PartShape::tube, glm::dvec3(0.014, 0.022, 0.0), glm::dmat3(0.0)}, 0.011, true, "a tube"},
        {BlockShape{PartShape::box, glm::dvec3(0.028, 0.016, 0.026), glm::dmat3(0.0)}, 0.008, true, "a box"},
        {BlockShape{PartShape::sphere, glm::dvec3(0.012, 0.0, 0.0), glm::dmat3(0.0)}, 0.012, true, "a sphere"},
        {std::nullopt, 0.0, false, "no shape, a point mass"},
        {BlockShape{PartShape::tensor, glm::dvec3(0.0), glm::dmat3(2e-6)}, 0.0, false, "a tensor"},
    }};
    for (const Case& c : cases) {
        for (const bool pusher : {false, true}) {
            AirframeSpec s = reference_quad();
            s.motor.shape = c.shape;
            std::vector<glm::dvec3> motors;
            for (RotorSpec& r : s.rotors) {
                r.pusher = pusher;
                motors.push_back(r.hub.position + glm::dvec3(0.0, pusher ? c.half : -c.half, 0.0));
            }
            expect_motors_at(s, motors, c.solid, std::string(c.what) + (pusher ? ", a pusher" : ", a tractor"));
        }
    }
}

// The thrust axis is the hub's +Y, wherever the hub points. Every hub turned
// a quarter about +Z, as an exact literal: its +Y onto design -X. A tractor's
// motor then sits 0.009 toward +X of its hub, behind the propeller, and a
// pusher's 0.009 toward -X.
TEST(AirframeCompile, AMotorMovesAlongItsHubsThrustAxis) {
    const double h = 0.7071067811865476;  // sqrt(1/2)
    for (const bool pusher : {false, true}) {
        AirframeSpec s = reference_quad();
        std::vector<glm::dvec3> motors;
        for (RotorSpec& r : s.rotors) {
            r.hub.orientation = glm::dquat(h, 0.0, 0.0, h);
            r.pusher = pusher;
            motors.push_back(r.hub.position + glm::dvec3(pusher ? -0.009 : 0.009, 0.0, 0.0));
        }
        expect_motors_at(s, motors, true, pusher ? "turned hubs, pushers" : "turned hubs, tractors");
    }
}

// ===========================================================================
// MASS FROM A MESH (the mass-from-mesh plan, 2026-10-13). A record whose
// geometry source is a mesh takes its mass, centre of mass and inertia from
// its mesh's properties times its density, and its part from the same mesh
// (TD-9: one record). Its pose is the mesh's PLACEMENT, mesh local -> design;
// a mesh's origin is not its centroid. Task M1: the record's fields and its
// refusals, each by record index, name and field (L6), in the one call that
// lists everything.
// ===========================================================================

namespace {

using MeshPtr = std::shared_ptr<const spade::TriangleMesh>;

// A box's twelve triangles, counter-clockwise seen from outside, its corners
// numbered x + 2y + 4z (tests/test_mesh_mass.cpp's).
constexpr uint32_t kBoxTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3},  // z = lo, normal -z
    {4, 5, 6}, {5, 7, 6},  // z = hi, normal +z
    {0, 1, 4}, {1, 5, 4},  // y = lo, normal -y
    {2, 6, 3}, {3, 6, 7},  // y = hi, normal +y
    {0, 4, 2}, {2, 4, 6},  // x = lo, normal -x
    {1, 3, 5}, {3, 7, 5},  // x = hi, normal +x
};

// The box [lo, lo + size], in metres: corner k is lo + (k & 1, k >> 1 & 1,
// k >> 2 & 1) * size.
[[nodiscard]] spade::MeshInput box_input(glm::dvec3 lo, glm::dvec3 size) {
    spade::MeshInput m;
    m.unit = spade::MeshUnit::metre;
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::dvec3 bit(static_cast<double>(v & 1u), static_cast<double>((v >> 1) & 1u),
                             static_cast<double>((v >> 2) & 1u));
        m.positions.push_back(lo + bit * size);
    }
    for (const auto& t : kBoxTriangles) m.indices.insert(m.indices.end(), {t[0], t[1], t[2]});
    return m;
}

[[nodiscard]] MeshPtr mesh_of(const spade::MeshInput& in) {
    auto m = spade::make_mesh(in, "fixture");
    EXPECT_TRUE(m.has_value()) << (m.has_value() ? std::string() : m.error().context);
    return m.has_value() ? *m : nullptr;
}

// The canopy: a 62.5 x 15.625 x 31.25 mm box with its ORIGIN AT A CORNER, so
// its centroid is half its sides from its origin. Dyadic sides: fp32 holds
// every corner, and its volume, 2^-15 m^3, is exact.
const glm::dvec3 kCanopySides(0.0625, 0.015625, 0.03125);
constexpr double kCanopyDensity = 1250.0;  // kg/m^3, a printed plastic

[[nodiscard]] MeshPtr canopy_mesh() { return mesh_of(box_input(glm::dvec3(0.0), kCanopySides)); }

// A mesh record as an importer writes it: no mass, shape, size or tensor of
// its own (TD-9), its mesh placed at `position`, turned by `orientation`.
[[nodiscard]] PartInertia mesh_record(MeshPtr mesh, double density, glm::dvec3 position,
                                      glm::dquat orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                                      std::string name = "canopy") {
    PartInertia p;
    p.position = position;
    p.orientation = orientation;
    p.geometry = PartGeometrySpec{.source = GeometrySource::mesh,
                                  .name = std::move(name),
                                  .role = "body",
                                  .mesh = std::move(mesh),
                                  .density = density,
                                  .min_feature = 0.004f};
    return p;
}

// The spec with one drag element given outright, zero coefficients for none.
// An airframe with a mesh record needs one: the drag estimate cannot see a
// mesh's area (Physics' drag rule, below).
[[nodiscard]] AirframeSpec with_drag(AirframeSpec s) {
    s.drag.push_back(DragSpec{});
    return s;
}

// The reference quad with the canopy on top as record 5: its corner at
// (-1/32, 1/64, -1/64), so it spans x and z about the frame's centre and sits
// on the frame's top (y = 0.015). Its drag is given.
const glm::dvec3 kCanopyPlacement(-0.03125, 0.015625, -0.015625);

[[nodiscard]] AirframeSpec quad_with_canopy() {
    AirframeSpec s = with_drag(reference_quad());
    s.parts.push_back(mesh_record(canopy_mesh(), kCanopyDensity, kCanopyPlacement));
    return s;
}

[[nodiscard]] bool mentions(const std::string& text, const std::string& what) {
    return text.find(what) != std::string::npos;
}

}  // namespace

// Each fault of a mesh record is ONE issue, at the record's index and the
// field at fault, and its message names the record. A hand-entered mass,
// shape, size or tensor beside a mesh source is a TD-9 conflict, refused by
// name, never silently overridden. There is no default density.
TEST(AirframeCompile, AMeshRecordsFaultIsOneIssueAtItsField) {
    using D = std::numeric_limits<double>;
    using F = std::numeric_limits<float>;
    struct Case {
        const char* what;
        void (*fault)(PartInertia&);
        const char* field;
        bool td9;
    };
    const std::array<Case, 15> cases = {{
        {"no mesh", [](PartInertia& p) { p.geometry.mesh = nullptr; }, "geometry.mesh", false},
        {"no density: 0 is not given", [](PartInertia& p) { p.geometry.density = 0.0; }, "geometry.density",
         false},
        {"a negative density", [](PartInertia& p) { p.geometry.density = -kCanopyDensity; }, "geometry.density",
         false},
        {"a NaN density", [](PartInertia& p) { p.geometry.density = D::quiet_NaN(); }, "geometry.density", false},
        {"an infinite density", [](PartInertia& p) { p.geometry.density = D::infinity(); }, "geometry.density",
         false},
        {"a density whose mass is not finite",
         [](PartInertia& p) {
             // A 2 m cube, 8 m^3: the largest double times 8 overflows.
             p.geometry.mesh = mesh_of(box_input(glm::dvec3(0.0), glm::dvec3(2.0)));
             p.geometry.density = D::max();
         },
         "geometry.density", false},
        {"no min_feature", [](PartInertia& p) { p.geometry.min_feature = 0.0f; }, "geometry.min_feature", false},
        {"a negative min_feature", [](PartInertia& p) { p.geometry.min_feature = -0.004f; },
         "geometry.min_feature", false},
        {"a NaN min_feature", [](PartInertia& p) { p.geometry.min_feature = F::quiet_NaN(); },
         "geometry.min_feature", false},
        {"an infinite min_feature", [](PartInertia& p) { p.geometry.min_feature = F::infinity(); },
         "geometry.min_feature", false},
        {"a hand-entered mass", [](PartInertia& p) { p.mass = 0.038; }, "mass", true},
        {"a hand-entered NaN mass", [](PartInertia& p) { p.mass = D::quiet_NaN(); }, "mass", true},
        {"a hand-entered shape", [](PartInertia& p) { p.shape = PartShape::box; }, "shape", true},
        {"a hand-entered size", [](PartInertia& p) { p.size = kCanopySides; }, "size", true},
        {"a hand-entered tensor", [](PartInertia& p) { p.tensor = glm::dmat3(1e-6); }, "tensor", true},
    }};
    for (const Case& c : cases) {
        AirframeSpec s = quad_with_canopy();
        c.fault(s.parts[5]);
        const std::vector<AirframeIssue> issues = check_airframe(s);
        ASSERT_EQ(issues.size(), 1u) << c.what << ": " << (issues.empty() ? "" : issues[0].field);
        EXPECT_TRUE(has_issue(issues, "parts", 5, c.field)) << c.what << ": " << issues[0].field;
        EXPECT_TRUE(mentions(issues[0].message, "'canopy'")) << c.what << ": " << issues[0].message;
        if (c.td9) {
            EXPECT_TRUE(mentions(issues[0].message, "TD-9")) << c.what << ": " << issues[0].message;
        }
        const auto compiled = compile_airframe(s);
        ASSERT_FALSE(compiled.has_value()) << c.what;
        EXPECT_TRUE(mentions(compiled.error().context, std::string("parts[5].") + c.field))
            << c.what << ": " << compiled.error().context;
    }
}

// Every fault at once, in the one call (Kat's question 2).
TEST(AirframeCompile, EveryFaultOfAMeshRecordIsListedInOneCall) {
    AirframeSpec s = quad_with_canopy();
    PartInertia& canopy = s.parts[5];
    canopy.geometry.mesh = nullptr;
    canopy.geometry.density = 0.0;
    canopy.geometry.min_feature = 0.0f;
    canopy.mass = 0.038;
    canopy.tensor = glm::dmat3(1e-6);
    const std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_EQ(issues.size(), 5u);
    for (const char* field : {"geometry.mesh", "geometry.density", "geometry.min_feature", "mass", "tensor"}) {
        EXPECT_TRUE(has_issue(issues, "parts", 5, field)) << field;
    }
}

// A mesh record whose mass cannot be taken from its mesh stops the composite
// and the fit: no fit runs on a mass that is not known. (A part's other
// problems, a name, do not stop the fit: APartProblemIsListedAtItsRecord.)
TEST(AirframeCompile, AMeshRecordWithoutItsMassStopsTheFit) {
    AirframeSpec s = quad_with_canopy();
    s.parts[5].geometry.density = 0.0;
    s.battery.mass = 20.0;  // the chain cannot lift it, which only a fit would find
    const std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "parts", 5, "geometry.density"));
    EXPECT_FALSE(has_issue(issues, "airframe", 0, "propulsion")) << "no fit ran";
}

// TD-9 the other way round: a mesh, a density or a min_feature beside a
// source that is not mesh would be read by nothing, so it is refused at its
// field rather than silently dropped. The mass is the record's own, so the
// fit still runs.
TEST(AirframeCompile, MeshFieldsBesideAnotherSourceAreRefused) {
    AirframeSpec s = reference_quad();
    s.parts[0].geometry.mesh = canopy_mesh();  // the frame's box, an inertia shape
    s.parts[1].geometry.density = kCanopyDensity;
    s.parts[2].geometry.source = GeometrySource::none;
    s.parts[2].geometry.min_feature = 0.004f;
    s.battery.mass = 20.0;
    const std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_EQ(issues.size(), 4u);
    EXPECT_TRUE(has_issue(issues, "parts", 0, "geometry.mesh"));
    EXPECT_TRUE(has_issue(issues, "parts", 1, "geometry.density"));
    EXPECT_TRUE(has_issue(issues, "parts", 2, "geometry.min_feature"));
    EXPECT_TRUE(has_issue(issues, "airframe", 0, "propulsion")) << "the fit still runs, and lists its own";
}

// A volume that is not positive, an inside-out mesh, never reaches the
// compile: make_mesh() refuses it (world/mesh.hpp), and so B6's reader, which
// reverses a mirrored node's winding, hands over none. The compile refuses
// one at geometry.mesh all the same.
TEST(AirframeCompile, AnInsideOutMeshIsRefusedBeforeTheCompile) {
    spade::MeshInput in = box_input(glm::dvec3(0.0), kCanopySides);
    for (std::size_t t = 0; t < in.indices.size(); t += 3) std::swap(in.indices[t + 1], in.indices[t + 2]);
    const auto m = spade::make_mesh(in, "inside out");
    ASSERT_FALSE(m.has_value());
    EXPECT_TRUE(mentions(m.error().context, "inside out")) << m.error().context;
}

// ===========================================================================
// MASS FROM A MESH, Task M2. The compile RESOLVES a mesh record, before
// composite_inertia() runs, into an explicit record: mass rho V; centre of
// mass the record's pose applied to the mesh's centroid, placement + R c;
// PartShape::tensor rho I_centroid in the record's local frame; orientation
// the record's. Its part sits at the record's PLACEMENT, never at its centre
// of mass (Physics' frame trap), carrying the mesh and the declared
// min_feature. The expectations are A4's analytic box records of the same
// mass and sides, and closed forms (TD-4).
// ===========================================================================

namespace {

const glm::dquat kIdentity(1.0, 0.0, 0.0, 0.0);

// The analytic twin of the box mesh [0, sides] placed at (position,
// orientation): a box record of mass rho V, centred where that mesh's
// centroid lands, position + R sides / 2, and turned as the mesh is.
[[nodiscard]] PartInertia box_twin(glm::dvec3 sides, double density, glm::dvec3 position,
                                   glm::dquat orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                                   std::string name = "canopy") {
    const glm::dvec3 centre = position + glm::mat3_cast(glm::normalize(orientation)) * (0.5 * sides);
    PartInertia p = make_part(PartShape::box, density * (sides.x * sides.y * sides.z), centre, sides, orientation);
    p.geometry = PartGeometrySpec{.source = GeometrySource::inertia_shape, .name = std::move(name), .role = "body"};
    return p;
}

// B3's bound (tests/test_mesh_mass.cpp, kRel): a mesh's tensor is within
// 1e-15 of its trace of the exact one. Summed into the composite, the
// difference is within 1e-15 of the composite's trace.
constexpr double kMeshRel = 1e-15;

// Two compiles' bodies agree. The composite's double tensor (about the centre
// of mass, design frame) within kMeshRel of its trace; and its float outputs,
// each rounded once from double, bit for bit when `bit_exact`, else within a
// few float ulps.
void expect_same_body(const CompiledAirframe& got, const CompiledAirframe& want, bool bit_exact,
                      const std::string& what) {
    const CompositeInertia& a = got.inertia;
    const CompositeInertia& b = want.inertia;
    const double trace = b.inertia_design[0][0] + b.inertia_design[1][1] + b.inertia_design[2][2];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(a.inertia_design[i][j], b.inertia_design[i][j], kMeshRel * trace)
                << what << ": inertia [" << i << "][" << j << "]";
        }
    }
    if (bit_exact) {
        EXPECT_EQ(std::bit_cast<uint32_t>(a.mass), std::bit_cast<uint32_t>(b.mass)) << what;
        EXPECT_TRUE(same_bits(a.center_of_mass, b.center_of_mass)) << what;
        EXPECT_TRUE(same_bits(a.principal_moments, b.principal_moments)) << what;
        EXPECT_TRUE(same_bits(a.design_to_body, b.design_to_body)) << what;
        EXPECT_TRUE(same_bits(got.model.body.inertia_diag, want.model.body.inertia_diag)) << what;
        EXPECT_TRUE(same_bits(got.model.com_offset, want.model.com_offset)) << what;
        return;
    }
    EXPECT_NEAR(a.mass, b.mass, 1e-6f * b.mass) << what;
    for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(a.center_of_mass[k], b.center_of_mass[k], 1e-7f) << what << ": centre of mass " << k;
        EXPECT_NEAR(a.principal_moments[k], b.principal_moments[k], 1e-6f * b.principal_moments[k])
            << what << ": moment " << k;
    }
    // q and -q are one rotation.
    EXPECT_NEAR(std::abs(glm::dot(a.design_to_body, b.design_to_body)), 1.0f, 1e-6f) << what;
}

// A design-frame point of a compiled model, in the world, the body posed at b.
[[nodiscard]] glm::vec3 world_of(const ModelType& m, const spade::BodyState& b, const glm::dvec3& design) {
    const glm::quat q = m.design_to_principal;
    const glm::dmat3 r_bd = glm::mat3_cast(glm::dquat(q.w, q.x, q.y, q.z));
    const glm::dvec3 body = r_bd * (design - glm::dvec3(m.com_offset));
    return b.pos + b.orient * glm::vec3(body);
}

// Every issue's place and message, for a failure's note.
[[nodiscard]] std::string listed(const std::vector<AirframeIssue>& issues) {
    std::string out;
    for (const AirframeIssue& i : issues) {
        out += i.kind + "[" + std::to_string(i.index) + "]." + i.field + ": " + i.message + "; ";
    }
    return out;
}

// Every triangle reversed: as a shell inside another, a cavity.
[[nodiscard]] spade::MeshInput flipped(spade::MeshInput m) {
    for (std::size_t t = 0; t < m.indices.size(); t += 3) std::swap(m.indices[t + 1], m.indices[t + 2]);
    return m;
}

// `a` then `b` as one input. The fixtures joined here share no position.
[[nodiscard]] spade::MeshInput joined(const spade::MeshInput& a, const spade::MeshInput& b) {
    spade::MeshInput m = a;
    const uint32_t base = static_cast<uint32_t>(a.positions.size());
    m.positions.insert(m.positions.end(), b.positions.begin(), b.positions.end());
    for (const uint32_t i : b.indices) m.indices.push_back(base + i);
    return m;
}

// The reference quad's own records (the canopy record carries no mass, so
// it adds nothing): their mass, 0.526 kg, and their moment, sum m p, in the
// design frame. Its motors are tractors, 0.009 below their hubs.
constexpr double kQuadMass = 0.526;

[[nodiscard]] glm::dvec3 quad_moment() {
    const AirframeSpec s = reference_quad();
    std::vector<glm::dvec3> motors;
    for (const RotorSpec& r : s.rotors) motors.push_back(r.hub.position - glm::dvec3(0.0, 0.009, 0.0));
    return kQuadMass * centre_of_mass(s, motors);
}

// A glTF 2.0 document, as text: the unit cube [0, 1]^3, one mesh, under one
// node with this translation and scale, its buffer a base64 data: URI. B6
// bakes the node's transform into the positions: T + S p.
[[nodiscard]] std::string scaled_cube_gltf(glm::dvec3 translation, glm::dvec3 scale) {
    std::vector<std::byte> bin;
    const auto put = [&bin](uint32_t v, int bytes) {
        for (int i = 0; i < bytes; ++i) bin.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFu));
    };
    for (uint32_t v = 0; v < 8; ++v) {
        for (const uint32_t bit : {v & 1u, (v >> 1) & 1u, (v >> 2) & 1u}) {
            put(std::bit_cast<uint32_t>(static_cast<float>(bit)), 4);
        }
    }
    for (const auto& t : kBoxTriangles) {
        for (const uint32_t i : t) put(i, 2);
    }
    static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string base64;
    for (std::size_t i = 0; i < bin.size(); i += 3) {
        uint32_t n = std::to_integer<uint32_t>(bin[i]) << 16;
        if (i + 1 < bin.size()) n |= std::to_integer<uint32_t>(bin[i + 1]) << 8;
        if (i + 2 < bin.size()) n |= std::to_integer<uint32_t>(bin[i + 2]);
        base64 += kAlphabet[(n >> 18) & 63u];
        base64 += kAlphabet[(n >> 12) & 63u];
        base64 += i + 1 < bin.size() ? kAlphabet[(n >> 6) & 63u] : '=';
        base64 += i + 2 < bin.size() ? kAlphabet[n & 63u] : '=';
    }
    const auto number = [](double x) {
        char buf[32];
        const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), x);
        return std::string(buf, r.ptr);
    };
    const auto triple = [&number](glm::dvec3 v) {
        return "[" + number(v.x) + "," + number(v.y) + "," + number(v.z) + "]";
    };
    return R"({"asset":{"version":"2.0"},)"
           R"("buffers":[{"byteLength":168,"uri":"data:application/octet-stream;base64,)" +
           base64 +
           R"("}],)"
           R"("bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":96},)"
           R"({"buffer":0,"byteOffset":96,"byteLength":72}],)"
           R"("accessors":[{"bufferView":0,"componentType":5126,"count":8,"type":"VEC3"},)"
           R"({"bufferView":1,"componentType":5123,"count":36,"type":"SCALAR"}],)"
           R"("meshes":[{"name":"cube","primitives":[{"attributes":{"POSITION":0},"indices":1}]}],)"
           R"("nodes":[{"name":"canopy","mesh":0,"translation":)" +
           triple(translation) + R"(,"scale":)" + triple(scale) +
           R"(}],)"
           R"("scenes":[{"nodes":[0]}],"scene":0})";
}

}  // namespace

// The canopy, 2^-15 m^3 of 1250 kg/m^3, as a mesh record and as the analytic
// box record of the same mass and sides. Its sides are dyadic, so the mesh's
// volume and centroid are exact, and so is the twin's mass. Eberly divides
// the second moments by 60 and the box formula by 12, each rounding once, so
// the two tensors agree within B3's bound, not bit for bit; the composite's
// float outputs, each rounded once from double, agree bit for bit.
TEST(AirframeCompile, AMeshBoxWeighsAsTheAnalyticBox) {
    const auto mesh = compile_airframe(quad_with_canopy());
    AirframeSpec twin_spec = with_drag(reference_quad());
    twin_spec.parts.push_back(box_twin(kCanopySides, kCanopyDensity, kCanopyPlacement));
    const auto twin = compile_airframe(twin_spec);
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;
    ASSERT_TRUE(twin.has_value()) << twin.error().context;
    const double v = kCanopySides.x * kCanopySides.y * kCanopySides.z;
    ASSERT_EQ(v, 0x1p-15);
    EXPECT_NEAR(mesh->model.body.mass, kQuadMass + kCanopyDensity * v, 1e-6);
    expect_same_body(*mesh, *twin, true, "the canopy");
    EXPECT_EQ(mesh->fit.mass_and_inertia, Provenance::from_parts);

    // The part, and on its mesh the volume P2 reads: computed once, exact here.
    const ModelType& m = mesh->model;
    EXPECT_TRUE(m.issues().empty());
    ASSERT_EQ(m.parts.size(), 11u) << "the frame, four arms, the canopy, four motors, the battery";
    const PartDesc& canopy = m.parts[5];
    EXPECT_EQ(canopy.name, "canopy");
    EXPECT_EQ(canopy.role, "body");
    ASSERT_NE(canopy.mesh, nullptr);
    EXPECT_EQ(canopy.mesh->mass_properties().volume, v);
}

// THE FRAME TRAP (Physics). The canopy's origin is a corner, so its centroid
// is R sides / 2 from its placement. Turned off every axis and moved, the
// mesh record weighs as its twin centred on that centroid, and its PART sits
// at the placement, carrying the mesh, so the posed part's surface is the
// twin's box: sample_parts() agrees at the centre and two millimetres off
// each face and a corner.
TEST(AirframeCompile, AMeshPartSitsAtItsPlacementAndWeighsAtItsCentroid) {
    const glm::dquat turn = glm::normalize(glm::dquat(0.9, 0.3, -0.2, 0.25));
    const glm::dvec3 at(-0.02, 0.04, 0.01);
    AirframeSpec s = with_drag(reference_quad());
    s.parts.push_back(mesh_record(canopy_mesh(), kCanopyDensity, at, turn));
    AirframeSpec t = with_drag(reference_quad());
    t.parts.push_back(box_twin(kCanopySides, kCanopyDensity, at, turn));
    const auto mesh = compile_airframe(s);
    const auto twin = compile_airframe(t);
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;
    ASSERT_TRUE(twin.has_value()) << twin.error().context;
    expect_same_body(*mesh, *twin, false, "a turned canopy");

    // The part: at the PLACEMENT, turned as the record, never at the centre
    // of mass, R sides / 2 (about 36 mm) away.
    const ModelType& m = mesh->model;
    ASSERT_EQ(m.parts.size(), 11u);
    const PartDesc& part = m.parts[5];
    EXPECT_EQ(part.name, "canopy");
    EXPECT_EQ(part.mesh, s.parts[5].geometry.mesh) << "the record's mesh, shared";
    EXPECT_TRUE(part.shape.nodes.empty()) << "a mesh part has no SdfProgram";
    EXPECT_EQ(part.attach, PartAttach::body);
    EXPECT_EQ(part.surface.kind, PartSurface::Kind::wall);
    EXPECT_EQ(part.min_feature, 0.004f) << "declared, not measured";
    expect_near(to_design(m, part.mount_pos), at, 1e-7, "the canopy part's origin");
    const glm::quat q_bd = m.design_to_principal;
    const glm::quat want_orient = q_bd * glm::quat(static_cast<float>(turn.w), static_cast<float>(turn.x),
                                                   static_cast<float>(turn.y), static_cast<float>(turn.z));
    EXPECT_NEAR(std::abs(glm::dot(part.mount_orient, want_orient)), 1.0f, 1e-6f);
    const glm::dmat3 r = glm::mat3_cast(turn);
    const glm::dvec3 half = 0.5 * kCanopySides;
    const glm::dvec3 centre = at + r * half;
    EXPECT_GT(glm::length(to_design(m, part.mount_pos) - centre), 0.03) << "the trap: not at the centre of mass";

    // Posed, the mesh part is the twin's box.
    spade::BodyState b{};
    b.pos = glm::vec3(0.5f, 1.0f, -0.25f);
    b.orient = glm::normalize(glm::quat(0.70710678f, 0.0f, 0.70710678f, 0.0f));
    const auto posed = posed_parts(m, b, {}, {}, PartFilter::all);
    const auto posed_twin = posed_parts(twin->model, b, {}, {}, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    ASSERT_TRUE(posed_twin.has_value()) << posed_twin.error().context;
    const double d = 0.002;
    const std::array<glm::dvec3, 8> probes = {
        glm::dvec3(0.0),
        glm::dvec3(half.x + d, 0.0, 0.0),
        glm::dvec3(-half.x - d, 0.0, 0.0),
        glm::dvec3(0.0, half.y + d, 0.0),
        glm::dvec3(0.0, -half.y - d, 0.0),
        glm::dvec3(0.0, 0.0, half.z + d),
        glm::dvec3(0.0, 0.0, -half.z - d),
        half + glm::dvec3(d),
    };
    for (std::size_t i = 0; i < probes.size(); ++i) {
        const glm::vec3 x = world_of(m, b, centre + r * probes[i]);
        const PartSample got = sample_parts(m, *posed, x);
        const PartSample want = sample_parts(twin->model, *posed_twin, x);
        EXPECT_NEAR(got.distance, want.distance, 2e-6f) << "probe " << i;
        EXPECT_EQ(got.owner, 5u) << "probe " << i << ": the canopy";
        EXPECT_EQ(want.owner, 5u) << "probe " << i << ": the twin";
        EXPECT_EQ(got.surface, PartSurface::Kind::wall) << "probe " << i;
    }
    EXPECT_NEAR(sample_parts(m, *posed, world_of(m, b, centre)).distance, -half.y, 2e-6)
        << "the centre, the smallest half side inside";
    EXPECT_NEAR(sample_parts(m, *posed, world_of(m, b, centre + r * probes[7])).distance, std::sqrt(3.0) * d, 2e-6)
        << "a corner, 2 mm out along each axis";
}

// The centre of mass moves with the record's pose, by the closed form: the
// reference quad's moment plus rho V at placement + R c, c = sides / 2 here,
// over the total mass. Taking the placement for the centre of mass (the
// trap) would be off by about 2.4 mm.
TEST(AirframeCompile, TheCentreOfMassMovesWithTheMeshRecordsPose) {
    const double h = 0.7071067811865476;  // sqrt(1/2)
    const double mass = kCanopyDensity * kCanopySides.x * kCanopySides.y * kCanopySides.z;
    const glm::dvec3 c = 0.5 * kCanopySides;
    struct Pose {
        glm::dvec3 at;
        glm::dquat q;
        const char* what;
    };
    const std::array<Pose, 4> poses = {{
        {kCanopyPlacement, kIdentity, "on the frame"},
        {kCanopyPlacement + glm::dvec3(0.01, 0.02, -0.03), kIdentity, "moved"},
        {kCanopyPlacement, glm::dquat(h, 0.0, h, 0.0), "a quarter turn about +Y"},
        {glm::dvec3(-0.02, 0.04, 0.01), glm::normalize(glm::dquat(0.9, 0.3, -0.2, 0.25)), "moved and turned"},
    }};
    const glm::dvec3 moment = quad_moment();
    for (const Pose& p : poses) {
        AirframeSpec s = with_drag(reference_quad());
        s.parts.push_back(mesh_record(canopy_mesh(), kCanopyDensity, p.at, p.q));
        const auto compiled = compile_airframe(s);
        ASSERT_TRUE(compiled.has_value()) << p.what << ": " << compiled.error().context;
        const glm::dvec3 com = (moment + mass * (p.at + glm::mat3_cast(glm::normalize(p.q)) * c)) / (kQuadMass + mass);
        expect_near(glm::dvec3(compiled->model.com_offset), com, 1e-7, p.what);
        const glm::dvec3 trap = (moment + mass * p.at) / (kQuadMass + mass);
        EXPECT_GT(glm::length(com - trap), 2e-3) << p.what << ": the test tells the two apart";
    }
}

// A cavity subtracts. The canopy with a box cavity off its centre, an inward
// shell: its mass, centre of mass and inertia are the outer box's less the
// cavity's, by the closed form about the composite's centre of mass (the
// reference quad's tensor moved there by the parallel-axis rule, the outer
// box's added, the cavity's taken off). The closed form sums in another
// order than the compile, so the tensor agrees within 1e-12 of its trace;
// the cavity's own share is about 1e-6 kg m^2, so a cavity added rather than
// taken off fails by far.
TEST(AirframeCompile, AMeshCavitySubtracts) {
    const glm::dvec3 cavity_lo(0.03125, 0.00390625, 0.0078125);
    const glm::dvec3 cavity_sides(0.015625, 0.0078125, 0.015625);
    const MeshPtr hollow = mesh_of(joined(box_input(glm::dvec3(0.0), kCanopySides),
                                          flipped(box_input(cavity_lo, cavity_sides))));
    ASSERT_NE(hollow, nullptr);
    ASSERT_EQ(hollow->shell_count(), 2u);
    AirframeSpec s = with_drag(reference_quad());
    s.parts.push_back(mesh_record(hollow, kCanopyDensity, kCanopyPlacement));
    const auto compiled = compile_airframe(s);
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    const auto quad = compile_airframe(reference_quad());
    ASSERT_TRUE(quad.has_value()) << quad.error().context;

    const auto box_tensor = [](double mass, glm::dvec3 sides) {
        glm::dmat3 i(0.0);
        i[0][0] = mass * (sides.y * sides.y + sides.z * sides.z) / 12.0;
        i[1][1] = mass * (sides.x * sides.x + sides.z * sides.z) / 12.0;
        i[2][2] = mass * (sides.x * sides.x + sides.y * sides.y) / 12.0;
        return i;
    };
    const auto moved = [](const glm::dmat3& about_own_centre, double mass, glm::dvec3 d) {
        return about_own_centre + mass * (glm::dot(d, d) * glm::dmat3(1.0) - glm::outerProduct(d, d));
    };
    const double m_out = kCanopyDensity * (kCanopySides.x * kCanopySides.y * kCanopySides.z);
    const double m_in = kCanopyDensity * (cavity_sides.x * cavity_sides.y * cavity_sides.z);
    const glm::dvec3 c_out = kCanopyPlacement + 0.5 * kCanopySides;
    const glm::dvec3 c_in = kCanopyPlacement + cavity_lo + 0.5 * cavity_sides;
    const double total = kQuadMass + m_out - m_in;
    const glm::dvec3 com = (quad_moment() + m_out * c_out - m_in * c_in) / total;
    const glm::dvec3 quad_com = quad_moment() / kQuadMass;
    const glm::dmat3 want = moved(quad->inertia.inertia_design, kQuadMass, quad_com - com) +
                            moved(box_tensor(m_out, kCanopySides), m_out, c_out - com) -
                            moved(box_tensor(m_in, cavity_sides), m_in, c_in - com);

    EXPECT_NEAR(compiled->model.body.mass, total, 1e-6);
    expect_near(glm::dvec3(compiled->model.com_offset), com, 1e-7, "the hollow canopy's centre of mass");
    const glm::dmat3& got = compiled->inertia.inertia_design;
    const double trace = want[0][0] + want[1][1] + want[2][2];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(got[i][j], want[i][j], 1e-12 * trace) << "[" << i << "][" << j << "]";
        }
    }
    EXPECT_EQ(hollow->mass_properties().volume,
              kCanopySides.x * kCanopySides.y * kCanopySides.z - cavity_sides.x * cavity_sides.y * cavity_sides.z)
        << "the volume P2 reads is the solid's, the cavity taken off";
}

// Two mesh parts add: the canopy and a camera, each against its analytic
// twin, two parts in record order, each at its own placement.
TEST(AirframeCompile, TwoMeshPartsAdd) {
    const glm::dvec3 camera_sides(0.015625, 0.015625, 0.0234375);
    const glm::dvec3 camera_at(0.0625, -0.0078125, -0.01171875);
    const double camera_density = 2700.0;
    AirframeSpec s = quad_with_canopy();
    s.parts.push_back(mesh_record(mesh_of(box_input(glm::dvec3(0.0), camera_sides)), camera_density, camera_at,
                                  kIdentity, "camera"));
    AirframeSpec t = with_drag(reference_quad());
    t.parts.push_back(box_twin(kCanopySides, kCanopyDensity, kCanopyPlacement));
    t.parts.push_back(box_twin(camera_sides, camera_density, camera_at, kIdentity, "camera"));
    const auto mesh = compile_airframe(s);
    const auto twin = compile_airframe(t);
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;
    ASSERT_TRUE(twin.has_value()) << twin.error().context;
    expect_same_body(*mesh, *twin, true, "the canopy and the camera");

    const ModelType& m = mesh->model;
    ASSERT_EQ(m.parts.size(), 12u);
    EXPECT_EQ(m.parts[5].name, "canopy");
    EXPECT_EQ(m.parts[6].name, "camera");
    expect_near(to_design(m, m.parts[5].mount_pos), kCanopyPlacement, 1e-7, "the canopy's origin");
    expect_near(to_design(m, m.parts[6].mount_pos), camera_at, 1e-7, "the camera's origin");
    EXPECT_EQ(m.parts[6].mesh, s.parts[6].geometry.mesh);
}

// Scale (B6). The record and the part pose carry none: B6 bakes every node
// transform, scale included, into the mesh's positions, so a scaled glTF
// node weighs as its baked solid. The unit cube under a node scaled to the
// canopy's sides and moved to its placement weighs as the canopy (38 g),
// not as a unit cube of 1250 kg; its record is placed at the identity, the
// node's transform being in the mesh.
TEST(AirframeCompile, AScaledGltfNodeWeighsAsItsBakedGeometry) {
    const std::string text = scaled_cube_gltf(kCanopyPlacement, kCanopySides);
    std::vector<std::byte> bytes(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) bytes[i] = static_cast<std::byte>(text[i]);
    const auto objects = spade::read_gltf_bytes(bytes, {});
    ASSERT_TRUE(objects.has_value()) << objects.error().context;
    ASSERT_EQ(objects->size(), 1u);
    const MeshPtr baked = (*objects)[0].mesh;
    ASSERT_NE(baked, nullptr);
    const double v = kCanopySides.x * kCanopySides.y * kCanopySides.z;
    EXPECT_EQ(baked->mass_properties().volume, v) << "the scaled solid's volume, exact (dyadic)";

    AirframeSpec s = with_drag(reference_quad());
    s.parts.push_back(mesh_record(baked, kCanopyDensity, glm::dvec3(0.0)));
    AirframeSpec t = with_drag(reference_quad());
    t.parts.push_back(box_twin(kCanopySides, kCanopyDensity, kCanopyPlacement));
    const auto mesh = compile_airframe(s);
    const auto twin = compile_airframe(t);
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;
    ASSERT_TRUE(twin.has_value()) << twin.error().context;
    EXPECT_NEAR(mesh->model.body.mass, kQuadMass + kCanopyDensity * v, 1e-6);
    expect_same_body(*mesh, *twin, true, "the baked canopy");
}

// A mesh record's part takes the model's rules like any part: a name the
// battery block's part repeats is listed at the record.
TEST(AirframeCompile, AMeshRecordsPartTakesTheModelsRules) {
    AirframeSpec s = quad_with_canopy();
    s.parts[5].geometry.name = "battery";
    const std::vector<AirframeIssue> issues = check_airframe(s);
    ASSERT_EQ(issues.size(), 1u) << listed(issues);
    EXPECT_TRUE(has_issue(issues, "parts", 5, "geometry.name"));
}

// A mesh record's bad pose is one issue, at the record's own field: a zero
// orientation leaves its centre of mass at its placement (no NaN is made
// from it), and a non-finite placement is the position's.
TEST(AirframeCompile, AMeshRecordsBadPoseIsOneIssue) {
    AirframeSpec s = quad_with_canopy();
    s.parts[5].orientation = glm::dquat(0.0, 0.0, 0.0, 0.0);
    std::vector<AirframeIssue> issues = check_airframe(s);
    ASSERT_EQ(issues.size(), 1u) << listed(issues);
    EXPECT_TRUE(has_issue(issues, "parts", 5, "orientation"));

    s = quad_with_canopy();
    s.parts[5].position.x = std::numeric_limits<double>::quiet_NaN();
    issues = check_airframe(s);
    ASSERT_EQ(issues.size(), 1u) << listed(issues);
    EXPECT_TRUE(has_issue(issues, "parts", 5, "position"));
}

// ===========================================================================
// PHYSICS' DRAG RULE (2026-10-13). With no DragSpec the drag is estimated
// from the records' inertia shapes, and a mesh record has none; yet its
// solid is real external area, unlike a point or a tensor record. So with no
// DragSpec, any mesh record among the drag sources refuses the estimate
// (L6): one issue at drag, naming the first mesh record, "(and N more)" for
// the rest. A DragSpec clears it.
// ===========================================================================

// A mixed airframe, the reference quad's analytic frame, arms, motors and
// battery plus the mesh canopy, with no DragSpec: refused at drag, naming
// the canopy. The same airframe with a DragSpec compiles.
TEST(AirframeCompile, AMixedAirframeWithoutADragSpecIsRefusedAtDrag) {
    AirframeSpec s = quad_with_canopy();
    s.drag.clear();
    std::vector<AirframeIssue> issues = check_airframe(s);
    ASSERT_EQ(issues.size(), 1u) << listed(issues);
    EXPECT_TRUE(has_issue(issues, "drag", 0, ""));
    EXPECT_TRUE(mentions(issues[0].message, "none given, and part 'canopy' is a mesh, whose area the estimate "
                                            "cannot see: give a DragSpec (zero coefficients for none)"))
        << issues[0].message;
    EXPECT_FALSE(mentions(issues[0].message, "more)")) << "one mesh record";
    const auto refused = compile_airframe(s);
    ASSERT_FALSE(refused.has_value());
    EXPECT_TRUE(mentions(refused.error().context, "drag[0]: none given, and part 'canopy' is a mesh"))
        << refused.error().context;

    // In the one call that lists everything, beside the record's own fault.
    s.parts[5].geometry.density = 0.0;
    issues = check_airframe(s);
    EXPECT_EQ(issues.size(), 2u) << listed(issues);
    EXPECT_TRUE(has_issue(issues, "drag", 0, ""));
    EXPECT_TRUE(has_issue(issues, "parts", 5, "geometry.density"));

    const auto given = compile_airframe(quad_with_canopy());
    ASSERT_TRUE(given.has_value()) << given.error().context;
    EXPECT_EQ(given->fit.drag, Provenance::given);
}

// An all-mesh airframe: three mesh records, the blocks unshaped. The same
// rule, one issue, naming the first mesh record and counting the rest; with
// a DragSpec it compiles.
TEST(AirframeCompile, AnAllMeshAirframeWithoutADragSpecNamesTheFirstMesh) {
    AirframeSpec s = reference_quad();
    s.parts.clear();
    s.motor.shape.reset();
    s.battery.shape.reset();
    s.parts.push_back(mesh_record(canopy_mesh(), kCanopyDensity, glm::dvec3(-0.03125, -0.0078125, -0.015625),
                                  kIdentity, "frame"));
    s.parts.push_back(mesh_record(canopy_mesh(), kCanopyDensity, glm::dvec3(0.03125, -0.0078125, -0.015625),
                                  kIdentity, "arm"));
    s.parts.push_back(mesh_record(canopy_mesh(), kCanopyDensity, kCanopyPlacement, kIdentity, "lid"));
    const std::vector<AirframeIssue> issues = check_airframe(s);
    ASSERT_EQ(issues.size(), 1u) << listed(issues);
    EXPECT_TRUE(has_issue(issues, "drag", 0, ""));
    EXPECT_TRUE(mentions(issues[0].message, "none given, and part 'frame' is a mesh (and 2 more), whose area "
                                            "the estimate cannot see: give a DragSpec (zero coefficients for none)"))
        << issues[0].message;

    s.drag.push_back(DragSpec{});
    EXPECT_TRUE(check_airframe(s).empty()) << listed(check_airframe(s));
    const auto c = compile_airframe(s);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    EXPECT_EQ(c->model.parts.size(), 3u) << "three mesh parts; the blocks are unshaped";
}

// THE TENSOR'S CONVENTION, END TO END (Physics' ask, 2026-10-13). A box's
// tensor is diagonal in its own axes; a corner tetrahedron's is not. The
// tetrahedron (0, s e_x, s e_y, s e_z), s = 1/16, has volume s^3 / 6 and
// centroid s/4 (1, 1, 1), and about the centroid, per unit density, s^5/80
// on the diagonal and +s^5/480 off it: minus the integral of xy, which is
// -s^5/480 there (tests/test_mesh_mass.cpp's ACornerTetrahedronIsShiftedTo
// ItsCentroid works it by hand). As a mesh record it compiles to the body of
// a hand-entered PartShape::tensor record of the same mass, centre and
// textbook tensor times rho, so a product of the wrong sign, or a tensor
// read in the wrong frame, fails the compile here, not only in mesh_mass.
// Unturned, the composite's float outputs agree bit for bit; turned off
// every axis, within a few float ulps; the double tensor within B3's bound
// either way.
TEST(AirframeCompile, AMeshTensorWithProductsCompilesAsTheTextbookTensor) {
    const double s = 0.0625;
    spade::MeshInput in;
    in.unit = spade::MeshUnit::metre;
    in.positions = {glm::dvec3(0.0), glm::dvec3(s, 0.0, 0.0), glm::dvec3(0.0, s, 0.0), glm::dvec3(0.0, 0.0, s)};
    in.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    const MeshPtr tetrahedron = mesh_of(in);
    ASSERT_NE(tetrahedron, nullptr);
    const double s5 = s * s * s * s * s;
    glm::dmat3 textbook(0.0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) textbook[i][j] = i == j ? s5 / 80.0 : s5 / 480.0;
    }

    const glm::dvec3 at(0.03125, 0.015625, -0.03125);
    struct Case {
        glm::dquat q;
        bool bit_exact;
        const char* what;
    };
    const std::array<Case, 2> cases = {{
        {kIdentity, true, "unturned"},
        {glm::normalize(glm::dquat(0.9, 0.3, -0.2, 0.25)), false, "turned off every axis"},
    }};
    for (const Case& c : cases) {
        AirframeSpec mesh_spec = with_drag(reference_quad());
        mesh_spec.parts.push_back(mesh_record(tetrahedron, kCanopyDensity, at, c.q, "wedge"));
        PartInertia hand;
        hand.mass = kCanopyDensity * (s * s * s / 6.0);
        hand.position = at + glm::mat3_cast(glm::normalize(c.q)) * glm::dvec3(s / 4.0);
        hand.orientation = c.q;
        hand.shape = PartShape::tensor;
        hand.tensor = kCanopyDensity * textbook;
        hand.geometry = PartGeometrySpec{.source = GeometrySource::none, .name = "wedge", .role = {}};
        AirframeSpec hand_spec = with_drag(reference_quad());
        hand_spec.parts.push_back(hand);

        const auto mesh = compile_airframe(mesh_spec);
        const auto want = compile_airframe(hand_spec);
        ASSERT_TRUE(mesh.has_value()) << c.what << ": " << mesh.error().context;
        ASSERT_TRUE(want.has_value()) << c.what << ": " << want.error().context;
        // The wedge gives the composite products of inertia (the reference
        // quad alone has none); its own, about 2.5e-6 kg m^2, are far above
        // the tolerance, so their sign is pinned.
        ASSERT_GT(std::abs(want->inertia.inertia_design[0][1]), 1e-8) << c.what;
        expect_same_body(*mesh, *want, c.bit_exact, c.what);
    }
}
