// N17's flight controller (sandbox/flight_controller.hpp): the pilot's
// analysis::Controls {armed, throttle, pitch, roll, yaw} to each rotor's
// commanded speed, THROUGH THE CHAIN, every 4 ms step.
// - Disarmed, every rotor 0. Armed, the collective is the arm rule's duty
//   (idle + (1 - idle) throttle) as thrust, over the tilt (floored at 0.5);
//   no rotor goes below idle.
// - Attitude: a PD on the principal inertia, the error split into tilt and
//   yaw (q_err = q_tilt * q_yaw), omega_n = min(12, 0.6 / tau_max) for the
//   tilt and a third of it for yaw, zeta 0.9 (Physics' review, 2026-10-22:
//   faster rings against the rotors' lag).
// - Each rotor's thrust (flight_mixer.hpp) to its duty and speed through the
//   chain, so its idle and limits apply. A model without a chain is refused.
// Judged by Test/Docs' editor.fly predicates (end_goal_checks.hpp: armed,
// climbs, follows-attitude), headless on the default library's reference
// quad, before the editor and the tour fly it.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/end_goal_checks.hpp"
#include "../sandbox/flight_controller.hpp"
#include "analysis/case.hpp"
#include "flight_test_support.hpp"
#include "vehicles/propulsion_steady.hpp"
#include "world/world_file.hpp"

namespace {

namespace fl = spade::sandbox::flight;
namespace ft = spade::sandbox::flight::test;
namespace eg = spade::sandbox::end_goal;
namespace vh = spade::vehicles;
using spade::analysis::Controls;

constexpr float kGravity = 9.80665f;

// The air both rigs fly in (Environment's default; the hover world's too).
const float kDensity = spade::Environment{}.air_density;

// The throttle whose thrust is `weights` times the weight (1: a hover), by
// the arm rule and the chain.
[[nodiscard]] float throttle_for(const vh::ModelType& m, float density, double weights = 1.0) {
    const double per_rotor = weights * m.body.mass * kGravity / static_cast<double>(m.rotors.size());
    const vh::SteadyPoint p = vh::steady_state_for_thrust(*m.propulsion, per_rotor, density, 0.0,
                                                         spade::analysis::kDefaultStateOfCharge);
    const double idle = spade::analysis::kDefaultIdleDuty;
    return static_cast<float>((p.duty - idle) / (1.0 - idle));
}

// One frame as editor.fly records it: the design frame's attitude (+X nose).
[[nodiscard]] eg::FlightFrame frame_of(const ft::FlightRig& rig, double t) {
    eg::FlightFrame f;
    f.t_s = t;
    const spade::Result<const spade::BodyState*> body = rig.sim.body(rig.ref.body);
    EXPECT_TRUE(body.has_value());
    if (!body) return f;
    f.height = (*body)->pos.y;
    f.velocity = (*body)->vel;
    f.attitude = (*body)->orient * rig.model.design_to_principal;
    std::vector<eg::RotorProbe> probes;
    for (uint32_t i = 0; i < rig.ref.rotor_count; ++i) {
        const spade::Result<const vh::RotorRow*> row = rig.sim.rotor(rig.ref, i);
        EXPECT_TRUE(row.has_value());
        if (!row) continue;
        probes.push_back(eg::probe_rotor(**row, **body));
        f.rotor_omega.push_back((*row)->omega);
    }
    f.thrust_up = eg::vertical_thrust(probes);
    return f;
}

// Flies the rig from t0 for `seconds`, the controller commanding before each
// step with what `at(t)` asks, one frame recorded after each step.
void fly(ft::FlightRig& rig, const fl::FlightController& fc, double& t, double seconds,
         const std::function<Controls(double)>& at, std::vector<eg::FlightFrame>& frames) {
    const int steps = static_cast<int>(std::lround(seconds / ft::kStepS));
    std::vector<float> omega(rig.ref.rotor_count, 0.0f);
    for (int s = 0; s < steps; ++s) {
        const spade::Result<const spade::BodyState*> body = rig.sim.body(rig.ref.body);
        ASSERT_TRUE(body.has_value());
        const spade::Result<void> commanded = fc.command(at(t), **body, omega);
        ASSERT_TRUE(commanded.has_value()) << commanded.error().context;
        ASSERT_TRUE(rig.sim.set_rotor_commands(rig.ref, omega).has_value());
        ASSERT_TRUE(rig.sim.step(1).has_value());
        t += ft::kStepS;
        frames.push_back(frame_of(rig, t));
    }
}

// The hover world: a ground to rest on, with room for the quad's elements
// (a scene's composition would size it; scene/compose.cpp).
[[nodiscard]] std::optional<ft::FlightRig> on_the_ground() {
    spade::Result<spade::WorldDesc> world =
        spade::load_world_file(std::filesystem::path(SPADE_ASSETS_DIR) / "worlds" / "hover.world.yaml");
    EXPECT_TRUE(world.has_value()) << (world ? "" : world.error().context);
    if (!world) return std::nullopt;
    world->capacities = spade::Capacities{4, 64, 16, 8};
    return ft::reference_quad_rig(glm::vec3(0.0f, 0.5f, 0.0f), 0.0f, *world);
}

}  // namespace

// Physics' rule, printed so the review can read it off the run.
TEST(FlightController, ItsGainsFollowTheModelsSlowestRotor) {
    std::optional<ft::FlightRig> rig = ft::reference_quad_rig(glm::vec3(0.0f, 10.0f, 0.0f));
    ASSERT_TRUE(rig.has_value());
    float tau_max = 0.0f;
    for (const vh::RotorDesc& r : rig->model.rotors) tau_max = std::max(tau_max, r.tau);
    const float want = tau_max > 0.0f ? std::min(12.0f, 0.6f / tau_max) : 12.0f;
    std::printf("[ reference quad ] tau_max %.4f s -> omega_n %.3f rad/s (yaw %.3f), zeta 0.9\n", tau_max, want,
                want / 3.0f);
    const spade::Result<fl::FlightController> fc = fl::FlightController::make(rig->model, kDensity);
    ASSERT_TRUE(fc.has_value()) << fc.error().context;
    EXPECT_FLOAT_EQ(fc->gains().tau_max, tau_max);
    EXPECT_FLOAT_EQ(fc->gains().omega_tilt, want);
    EXPECT_FLOAT_EQ(fc->gains().omega_yaw, want / 3.0f);
    EXPECT_FLOAT_EQ(fc->gains().zeta, 0.9f);
}

TEST(FlightController, DisarmedEveryRotorIsCommandedZero) {
    std::optional<ft::FlightRig> rig = ft::reference_quad_rig(glm::vec3(0.0f, 10.0f, 0.0f));
    ASSERT_TRUE(rig.has_value());
    const spade::Result<fl::FlightController> fc = fl::FlightController::make(rig->model, kDensity);
    ASSERT_TRUE(fc.has_value()) << fc.error().context;
    const spade::Result<const spade::BodyState*> body = rig->sim.body(rig->ref.body);
    ASSERT_TRUE(body.has_value());
    Controls c;
    c.armed = false;
    c.throttle = 0.8f;
    c.pitch_deg = 10.0f;
    std::vector<float> omega(rig->ref.rotor_count, 7.0f);
    ASSERT_TRUE(fc->command(c, **body, omega).has_value());
    for (const float w : omega) EXPECT_EQ(w, 0.0f);
}

// On the ground: still before arming, every rotor at idle within 0.5 s of
// arming, and the drone stays down (Test/Docs' armed). Then throttle up from
// where it rests, after the ground contact's pop, never the spawn height
// (the lead, 2026-10-22), to 1.3 times its weight, inside the band climbs was
// simulated for (1.1 to 1.4 m g; full throttle climbs so fast that inflow
// takes more thrust than its lower bound allows): it climbs as its thrust
// predicts (climbs).
TEST(FlightController, ArmedItIdlesOnTheGroundAndThrottleUpItClimbs) {
    std::optional<ft::FlightRig> rig = on_the_ground();
    ASSERT_TRUE(rig.has_value());
    const float density = kDensity;
    const spade::Result<fl::FlightController> fc = fl::FlightController::make(rig->model, density);
    ASSERT_TRUE(fc.has_value()) << fc.error().context;
    std::vector<eg::FlightFrame> frames;
    double t = 0.0;
    fly(*rig, *fc, t, 1.5, [](double) { return Controls{}; }, frames);  // disarmed: it settles
    const double arm_s = t;
    Controls armed;
    armed.armed = true;
    fly(*rig, *fc, t, 1.5, [&](double) { return armed; }, frames);
    const double throttle_up_s = t;
    Controls up = armed;
    up.throttle = throttle_for(rig->model, density, 1.3);
    fly(*rig, *fc, t, 3.0, [&](double) { return up; }, frames);

    std::vector<float> idle;
    for (uint32_t i = 0; i < rig->ref.rotor_count; ++i) {
        const spade::Result<vh::DutyCommand> d = vh::rotor_omega_for_duty(
            *rig->model.propulsion, spade::analysis::kDefaultIdleDuty, density, spade::analysis::kDefaultStateOfCharge);
        ASSERT_TRUE(d.has_value()) << d.error().context;
        idle.push_back(d->omega_cmd);
    }
    const auto up_at = static_cast<std::ptrdiff_t>(std::lround(throttle_up_s / ft::kStepS));
    const std::vector<eg::FlightFrame> until_up(frames.begin(), frames.begin() + up_at);
    const eg::Verdict is_armed = eg::armed(until_up, arm_s, idle);
    EXPECT_TRUE(is_armed.ok) << is_armed.detail;
    std::printf("[ reference quad ] %s\n", is_armed.detail.c_str());
    const eg::Verdict climbed =
        eg::climbs(frames, throttle_up_s, rig->model.body.mass, kGravity, rig->model.rotors.front().radius);
    EXPECT_TRUE(climbed.ok) << climbed.detail;
    std::printf("[ reference quad ] %s\n", climbed.detail.c_str());
}

// In open air at the hover throttle, 15 degrees nose down: reached within 3
// degrees in 1.0 s, held through 1.5 s, roll held level, and the drone gains
// speed along its nose (follows-attitude).
TEST(FlightController, APitchCommandIsFollowedAndTheDroneGainsSpeedAlongItsNose) {
    std::optional<ft::FlightRig> rig = ft::reference_quad_rig(glm::vec3(0.0f, 10.0f, 0.0f));
    ASSERT_TRUE(rig.has_value());
    const float density = kDensity;
    const spade::Result<fl::FlightController> fc = fl::FlightController::make(rig->model, density);
    ASSERT_TRUE(fc.has_value()) << fc.error().context;
    Controls hover;
    hover.armed = true;
    hover.throttle = throttle_for(rig->model, density);
    std::vector<eg::FlightFrame> frames;
    double t = 0.0;
    fly(*rig, *fc, t, 0.5, [&](double) { return hover; }, frames);
    const double command_s = t;
    Controls nose_down = hover;
    nose_down.pitch_deg = -15.0f;
    fly(*rig, *fc, t, 2.0, [&](double) { return nose_down; }, frames);
    const eg::Verdict followed = eg::follows_attitude(frames, command_s, glm::radians(-15.0f));
    EXPECT_TRUE(followed.ok) << followed.detail;
    std::printf("[ reference quad ] %s\n", followed.detail.c_str());
}

// Roll and yaw, whose signs follows-attitude doesn't judge: 10 degrees of
// roll (right side down) is reached within 3 degrees in 1.0 s with the pitch
// held level; 30 degrees of yaw turns the nose that way (about +Y, nose
// toward -Z), within 5 degrees by 4 s at its slower gain.
TEST(FlightController, RollAndYawCommandsTurnTheDroneTheRightWay) {
    std::optional<ft::FlightRig> rig = ft::reference_quad_rig(glm::vec3(0.0f, 10.0f, 0.0f));
    ASSERT_TRUE(rig.has_value());
    const float density = kDensity;
    const spade::Result<fl::FlightController> fc = fl::FlightController::make(rig->model, density);
    ASSERT_TRUE(fc.has_value()) << fc.error().context;
    Controls c;
    c.armed = true;
    c.throttle = throttle_for(rig->model, density);
    c.roll_deg = 10.0f;
    std::vector<eg::FlightFrame> frames;
    double t = 0.0;
    fly(*rig, *fc, t, 1.0, [&](double) { return c; }, frames);
    ASSERT_FALSE(frames.empty());
    EXPECT_NEAR(glm::degrees(eg::roll_of(frames.back().attitude)), 10.0f, 3.0f);
    EXPECT_NEAR(glm::degrees(eg::pitch_of(frames.back().attitude)), 0.0f, 3.0f);

    c.roll_deg = 0.0f;
    c.yaw_deg = 30.0f;
    fly(*rig, *fc, t, 4.0, [&](double) { return c; }, frames);
    const glm::vec3 nose = frames.back().attitude * glm::vec3(1.0f, 0.0f, 0.0f);
    EXPECT_NEAR(glm::degrees(std::atan2(-nose.z, nose.x)), 30.0f, 5.0f);
}

// A model with rotors and no chain is refused by name, as LiveRun refuses it.
TEST(FlightController, AModelWithoutAChainIsRefusedByName) {
    std::optional<ft::FlightRig> rig = ft::reference_quad_rig(glm::vec3(0.0f, 10.0f, 0.0f));
    ASSERT_TRUE(rig.has_value());
    vh::ModelType bare = rig->model;
    bare.propulsion.reset();
    const spade::Result<fl::FlightController> c = fl::FlightController::make(bare, kDensity);
    ASSERT_FALSE(c.has_value());
    EXPECT_NE(c.error().context.find("'" + bare.name + "'"), std::string::npos) << c.error().context;
    EXPECT_NE(c.error().context.find("chain"), std::string::npos) << c.error().context;
}
