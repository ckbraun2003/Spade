// N17's mixer (sandbox/flight_mixer.hpp; the end-goal design's ruling 2: a
// pure function Physics reviews, moving to vehicles/ when it leaves the
// sandbox). From any model's rotor layout, the effectiveness matrix: per-rotor
// thrust T_i along each axis n_i to the wrench [collective along body +Y,
// moment about body x (roll), y (yaw), z (pitch)], the moment by the
// simulation's own law (vehicles/rotor.cpp: r x f - spin k_Q w^2 n, with
// k_Q w^2 = (k_Q / k_T) T). Allocation is minimum-norm within each rotor's
// [T_min, T_max], giving up yaw first, then roll and pitch, keeping the
// collective. A layout that can't reach an axis is refused naming it.

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/flight_mixer.hpp"
#include "flight_test_support.hpp"
#include "vehicles/model_type.hpp"

namespace {

namespace fl = spade::sandbox::flight;
namespace vh = spade::vehicles;

constexpr float kKt = 1.0e-5f;  // N s^2
constexpr float kKq = 2.0e-7f;  // N m s^2: k_Q / k_T = 0.02 m
constexpr float kArm = 0.2f;    // m

[[nodiscard]] vh::RotorDesc rotor_at(glm::vec3 pos, float spin, glm::quat orient = glm::quat(1, 0, 0, 0)) {
    vh::RotorDesc r;
    r.local_pos = pos;
    r.local_orient = orient;
    r.spin_dir = spin;
    r.radius = 0.06f;
    r.thrust_coeff = kKt;
    r.torque_coeff = kKq;
    return r;
}

// Rotors 0..3 at +X, +Z, -X, -Z, spinning +1, -1, +1, -1 (drone_sim.hpp's plus).
[[nodiscard]] vh::ModelType plus_quad() {
    vh::ModelType m;
    m.name = "plus";
    m.rotors = {rotor_at({kArm, 0, 0}, 1.0f), rotor_at({0, 0, kArm}, -1.0f), rotor_at({-kArm, 0, 0}, 1.0f),
                rotor_at({0, 0, -kArm}, -1.0f)};
    return m;
}

[[nodiscard]] vh::ModelType x_quad() {
    const float a = kArm / std::numbers::sqrt2_v<float>;
    vh::ModelType m;
    m.name = "x";
    m.rotors = {rotor_at({a, 0, a}, 1.0f), rotor_at({-a, 0, a}, -1.0f), rotor_at({-a, 0, -a}, 1.0f),
                rotor_at({a, 0, -a}, -1.0f)};
    return m;
}

// Six on a circle, 60 degrees apart, alternating spin, each axis tilted 10
// degrees about its arm (a real hex's tilt, so the axes aren't all +Y).
[[nodiscard]] vh::ModelType hex() {
    vh::ModelType m;
    m.name = "hex";
    for (int i = 0; i < 6; ++i) {
        const float angle = static_cast<float>(i) * std::numbers::pi_v<float> / 3.0f;
        const glm::vec3 arm(std::cos(angle), 0.0f, std::sin(angle));
        const float tilt = (i % 2 == 0 ? 1.0f : -1.0f) * glm::radians(10.0f);
        m.rotors.push_back(rotor_at(kArm * arm, i % 2 == 0 ? 1.0f : -1.0f, glm::angleAxis(tilt, arm)));
    }
    return m;
}

// The wrench of `thrust` by the simulation's law, written out independently
// of the mixer.
[[nodiscard]] fl::Wrench law(const vh::ModelType& m, const std::vector<float>& thrust) {
    fl::Wrench w;
    for (std::size_t i = 0; i < m.rotors.size(); ++i) {
        const vh::RotorDesc& r = m.rotors[i];
        const glm::vec3 n = r.local_orient * glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 f = thrust[i] * n;
        w.collective += f.y;
        w.moment += glm::cross(r.local_pos, f) - (r.spin_dir * (r.torque_coeff / r.thrust_coeff) * thrust[i]) * n;
    }
    return w;
}

void expect_near(const fl::Wrench& got, const fl::Wrench& want, float tol, const std::string& what) {
    EXPECT_NEAR(got.collective, want.collective, tol) << what;
    EXPECT_NEAR(got.moment.x, want.moment.x, tol) << what << " (roll)";
    EXPECT_NEAR(got.moment.y, want.moment.y, tol) << what << " (yaw)";
    EXPECT_NEAR(got.moment.z, want.moment.z, tol) << what << " (pitch)";
}

[[nodiscard]] fl::Mixer mixer_of(const vh::ModelType& m) {
    const spade::Result<fl::Mixer> mixer = fl::make_mixer(m);
    EXPECT_TRUE(mixer.has_value()) << m.name << ": " << (mixer ? "" : mixer.error().context);
    return mixer ? *mixer : fl::Mixer{};
}

constexpr float kBig = 1.0e6f;  // a bound nothing reaches

}  // namespace

// The matrix is the simulation's force and torque law, for any layout.
TEST(FlightMixer, TheMatrixIsTheRotorsForceAndTorqueLaw) {
    for (const vh::ModelType& m : {plus_quad(), x_quad(), hex()}) {
        const fl::Mixer mixer = mixer_of(m);
        std::vector<float> thrust;
        for (std::size_t i = 0; i < m.rotors.size(); ++i) thrust.push_back(1.0f + 0.37f * static_cast<float>(i));
        expect_near(fl::wrench_of(mixer, thrust), law(m, thrust), 1e-5f, m.name);
    }
}

// Within reach, the allocation gives exactly the wrench asked for.
TEST(FlightMixer, AllocationGivesAnyWrenchWithinReach) {
    const fl::Wrench want{10.0f, glm::vec3(0.10f, 0.05f, -0.08f)};
    for (const vh::ModelType& m : {plus_quad(), x_quad(), hex()}) {
        const fl::Allocation a = fl::allocate(mixer_of(m), want, 0.0f, kBig);
        ASSERT_EQ(a.thrust.size(), m.rotors.size()) << m.name;
        expect_near(law(m, a.thrust), want, 1e-4f, m.name);
        EXPECT_EQ(a.yaw_kept, 1.0f) << m.name;
        EXPECT_EQ(a.attitude_kept, 1.0f) << m.name;
        EXPECT_FALSE(a.collective_clamped) << m.name;
    }
}

// A symmetric layout shares a pure collective evenly.
TEST(FlightMixer, ThePureCollectiveSplitsEvenlyOnASymmetricLayout) {
    for (const vh::ModelType& m : {plus_quad(), x_quad()}) {
        const fl::Allocation a = fl::allocate(mixer_of(m), fl::Wrench{8.0f, glm::vec3(0.0f)}, 0.0f, kBig);
        for (const float t : a.thrust) EXPECT_NEAR(t, 2.0f, 1e-5f) << m.name;
    }
}

// On the plus quad: rotor 0 (+X) and 2 (-X) carry pitch (about +Z), rotor 1
// (+Z) and 3 (-Z) roll (about +X), and yaw is by spin: r x n for a rotor at
// +Z is (-L, 0, 0), at +X (0, 0, +L); spin +1 pushes yaw negative.
TEST(FlightMixer, AUnitMomentMovesTheRightRotorsTheRightWay) {
    const fl::Mixer mixer = mixer_of(plus_quad());
    const fl::Wrench hover{8.0f, glm::vec3(0.0f)};
    const fl::Allocation base = fl::allocate(mixer, hover, 0.0f, kBig);
    const auto delta = [&](glm::vec3 moment) {
        const fl::Allocation a = fl::allocate(mixer, fl::Wrench{8.0f, moment}, 0.0f, kBig);
        std::vector<float> d;
        for (std::size_t i = 0; i < a.thrust.size(); ++i) d.push_back(a.thrust[i] - base.thrust[i]);
        return d;
    };
    const std::vector<float> roll = delta({0.1f, 0.0f, 0.0f});
    EXPECT_GT(roll[3], 0.0f);
    EXPECT_LT(roll[1], 0.0f);
    EXPECT_NEAR(roll[0], 0.0f, 1e-6f);
    EXPECT_NEAR(roll[2], 0.0f, 1e-6f);
    const std::vector<float> pitch = delta({0.0f, 0.0f, 0.1f});
    EXPECT_GT(pitch[0], 0.0f);
    EXPECT_LT(pitch[2], 0.0f);
    EXPECT_NEAR(pitch[1], 0.0f, 1e-6f);
    EXPECT_NEAR(pitch[3], 0.0f, 1e-6f);
    const std::vector<float> yaw = delta({0.0f, 0.01f, 0.0f});
    EXPECT_GT(yaw[1], 0.0f);
    EXPECT_GT(yaw[3], 0.0f);
    EXPECT_LT(yaw[0], 0.0f);
    EXPECT_LT(yaw[2], 0.0f);
}

// Near the top of the rotors' range, a big yaw is given up first: the
// collective and the roll and pitch are kept, and no rotor leaves its range.
TEST(FlightMixer, SaturationKeepsTheCollectiveAndGivesUpYawFirst) {
    const vh::ModelType m = plus_quad();
    const fl::Mixer mixer = mixer_of(m);
    const float t_max = 3.0f;
    const fl::Wrench want{11.0f, glm::vec3(0.02f, 0.5f, -0.02f)};
    const fl::Allocation a = fl::allocate(mixer, want, 0.5f, t_max);
    for (const float t : a.thrust) {
        EXPECT_GE(t, 0.5f - 1e-6f);
        EXPECT_LE(t, t_max + 1e-6f);
    }
    EXPECT_LT(a.yaw_kept, 1.0f);
    EXPECT_EQ(a.attitude_kept, 1.0f);
    EXPECT_FALSE(a.collective_clamped);
    const fl::Wrench got = law(m, a.thrust);
    EXPECT_NEAR(got.collective, want.collective, 1e-4f);
    EXPECT_NEAR(got.moment.x, want.moment.x, 1e-4f);
    EXPECT_NEAR(got.moment.z, want.moment.z, 1e-4f);
    EXPECT_NEAR(got.moment.y, a.yaw_kept * want.moment.y, 1e-4f) << "yaw is the asked yaw, scaled";
}

// When roll and pitch can't all fit either, yaw goes to zero and roll and
// pitch scale together, so the tilt keeps its direction; the collective stays.
TEST(FlightMixer, WhenRollAndPitchCantFitTheyScaleTogetherAndTheCollectiveStays) {
    const vh::ModelType m = plus_quad();
    const fl::Mixer mixer = mixer_of(m);
    const fl::Wrench want{8.0f, glm::vec3(2.0f, 0.1f, 1.0f)};
    const fl::Allocation a = fl::allocate(mixer, want, 0.0f, 4.0f);
    EXPECT_EQ(a.yaw_kept, 0.0f);
    EXPECT_GT(a.attitude_kept, 0.0f);
    EXPECT_LT(a.attitude_kept, 1.0f);
    const fl::Wrench got = law(m, a.thrust);
    EXPECT_NEAR(got.collective, want.collective, 1e-4f);
    EXPECT_NEAR(got.moment.x, a.attitude_kept * want.moment.x, 1e-4f);
    EXPECT_NEAR(got.moment.z, a.attitude_kept * want.moment.z, 1e-4f);
    EXPECT_NEAR(got.moment.y, 0.0f, 1e-4f);
}

// A collective beyond every rotor's range is clamped, and says so.
TEST(FlightMixer, ACollectiveOutOfRangeIsClampedAndSaysSo) {
    const fl::Allocation a = fl::allocate(mixer_of(plus_quad()), fl::Wrench{100.0f, glm::vec3(0.0f)}, 0.0f, 3.0f);
    EXPECT_TRUE(a.collective_clamped);
    for (const float t : a.thrust) EXPECT_NEAR(t, 3.0f, 1e-6f);
}

// A layout that can't reach an axis is refused, naming the axis; so is one
// with no rotors, or a rotor without thrust.
TEST(FlightMixer, ALayoutThatCantReachAnAxisIsRefusedNamingIt) {
    vh::ModelType same_spin = plus_quad();
    for (vh::RotorDesc& r : same_spin.rotors) r.spin_dir = 1.0f;
    const spade::Result<fl::Mixer> no_yaw = fl::make_mixer(same_spin);
    ASSERT_FALSE(no_yaw.has_value());
    EXPECT_NE(no_yaw.error().context.find("no yaw authority"), std::string::npos) << no_yaw.error().context;
    EXPECT_NE(no_yaw.error().context.find("'plus'"), std::string::npos) << no_yaw.error().context;

    vh::ModelType coaxial;
    coaxial.name = "coaxial";
    coaxial.rotors = {rotor_at({0, 0, 0}, 1.0f), rotor_at({0, 0.05f, 0}, -1.0f)};
    const spade::Result<fl::Mixer> no_roll = fl::make_mixer(coaxial);
    ASSERT_FALSE(no_roll.has_value());
    EXPECT_NE(no_roll.error().context.find("no roll authority"), std::string::npos) << no_roll.error().context;

    vh::ModelType none;
    none.name = "glider";
    const spade::Result<fl::Mixer> empty = fl::make_mixer(none);
    ASSERT_FALSE(empty.has_value());
    EXPECT_NE(empty.error().context.find("no rotors"), std::string::npos) << empty.error().context;

    vh::ModelType dead = plus_quad();
    dead.rotors[2].thrust_coeff = 0.0f;
    const spade::Result<fl::Mixer> no_thrust = fl::make_mixer(dead);
    ASSERT_FALSE(no_thrust.has_value());
    EXPECT_NE(no_thrust.error().context.find("rotor 2"), std::string::npos) << no_thrust.error().context;
    EXPECT_NE(no_thrust.error().context.find("k_T"), std::string::npos) << no_thrust.error().context;
}

// Physics' review (2026-10-22): B T = w checks the mixer against itself, so
// its moments must also turn the SIMULATED reference quad the right way. At
// its hover collective in open air, a small moment about each body axis in
// turn, through the mixer, spins the body that way about that axis, and
// mostly about it, within 0.1 s.
TEST(FlightMixer, ItsMomentsTurnTheSimulatedReferenceQuadTheRightWay) {
    namespace ft = spade::sandbox::flight::test;
    const char* const kAxisName[3] = {"roll (body x)", "yaw (body y)", "pitch (body z)"};
    for (int axis = 0; axis < 3; ++axis) {
        SCOPED_TRACE(kAxisName[axis]);
        std::optional<ft::FlightRig> rig = ft::reference_quad_rig(glm::vec3(0.0f, 10.0f, 0.0f));
        ASSERT_TRUE(rig.has_value());
        const fl::Mixer mixer = mixer_of(rig->model);
        glm::vec3 moment(0.0f);
        moment[axis] = 0.02f;  // N m
        const float weight = rig->model.body.mass * 9.80665f;
        const fl::Allocation a = fl::allocate(mixer, fl::Wrench{weight, moment}, 0.0f, kBig);
        std::vector<float> omega;
        for (std::size_t i = 0; i < a.thrust.size(); ++i) {
            omega.push_back(std::sqrt(std::max(a.thrust[i], 0.0f) / rig->model.rotors[i].thrust_coeff));
        }
        ASSERT_TRUE(rig->sim.set_rotor_commands(rig->ref, omega).has_value());
        ASSERT_TRUE(rig->sim.step(25).has_value());
        const spade::Result<const spade::BodyState*> body = rig->sim.body(rig->ref.body);
        ASSERT_TRUE(body.has_value()) << body.error().context;
        const glm::vec3 w = (*body)->omega_body;
        EXPECT_GT(w[axis], 0.0f) << "omega_body " << w.x << ", " << w.y << ", " << w.z;
        for (int other = 0; other < 3; ++other) {
            if (other != axis) {
                EXPECT_LT(std::abs(w[other]), 0.5f * std::abs(w[axis])) << "about axis " << other;
            }
        }
    }
}
