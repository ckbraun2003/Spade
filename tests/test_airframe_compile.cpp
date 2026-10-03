#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include "vehicles/airframe_compile.hpp"
#include "vehicles/battery.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/motor.hpp"
#include "vehicles/propeller.hpp"
#include "vehicles/propulsion_flags.hpp"

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
    const glm::dquat along_x = glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 0.0, 1.0));
    const glm::dquat along_z = glm::angleAxis(glm::half_pi<double>(), glm::dvec3(1.0, 0.0, 0.0));
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
    s.battery.mount.orientation = glm::angleAxis(glm::radians(15.0), glm::dvec3(0.0, 1.0, 0.0));
    return s;
}

[[nodiscard]] bool has_issue(const std::vector<AirframeIssue>& issues, const std::string& kind, std::size_t index,
                             const std::string& field) {
    for (const AirframeIssue& i : issues) {
        if (i.kind == kind && i.index == index && i.field == field) return true;
    }
    return false;
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
    const std::vector<AirframeIssue> issues = check_airframe(s);
    EXPECT_TRUE(has_issue(issues, "motor", 0, "kv"));
    EXPECT_TRUE(has_issue(issues, "battery", 0, "cells_series"));
    EXPECT_TRUE(has_issue(issues, "rotors", 1, "spin_dir"));
    EXPECT_TRUE(has_issue(issues, "parts", 0, "mass"));
    const auto c = compile_airframe(s);
    ASSERT_FALSE(c.has_value());
    EXPECT_NE(c.error().context.find("motor[0].kv"), std::string::npos) << c.error().context;
    EXPECT_NE(c.error().context.find("rotors[1].spin_dir"), std::string::npos) << c.error().context;
    EXPECT_FALSE(airframe_propulsion_chain(s).has_value()) << "an unusable motor is no chain";
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

// DETERMINISM (Kat's question 1). The reference quad's compiled model has one
// identity, here and on every platform. PROVISIONAL until the Docker gcc leg
// reproduces it (TD-12's discipline, applied to a compiled model).
TEST(AirframeCompile, TheReferenceQuadHasAPinnedIdentity) {
    const auto a = compile_airframe(reference_quad());
    const auto b = compile_airframe(reference_quad());
    ASSERT_TRUE(a.has_value() && b.has_value());
    EXPECT_EQ(model_identity(a->model), model_identity(b->model)) << "the same spec, compiled twice";
    constexpr uint64_t kPinned = 0x0000000000000000ull;  // measured on msvc-ninja-release; see the commit
    EXPECT_EQ(model_identity(a->model), kPinned) << std::hex << model_identity(a->model);
}
