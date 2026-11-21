// ---------------------------------------------------------------------------
// test_autopilot_map.cpp -- O140 phase 1, item 3 (docs/design/physics/plans/2026-11-21-fc-physics.md section 3): fc_link/autopilot_map.hpp, the order and
// the values a flight controller's motor outputs mean to Spade's rotors.
//
//   AutopilotMap.TheOrderTableIsPinnedToTheModelFilesGeometry      PX4's CA_ROTOR0..3 (4001_gz_x500: FRD positions and the KM signs) and ArduPilot's Quad X
//                                                                  place motor i at a corner and a spin; the table must send each to the Spade rotor
//                                                                  of the X500 and OpenFrame-5F model files at that corner with that spin
//   AutopilotMap.Px4OutputsAreTheChainsDuty                        0..1 maps to itself, a hair outside is clamped, further out or not finite is refused
//   AutopilotMap.ArdupilotPulsesAreTheChainsDuty                   1000 us 0, 2000 us 1, linear between, 0 = no pulse = 0, a bad range refused
//   AutopilotMap.DutiesGoThroughTheChainInSpadesOrder              four different duties come out as the chain's speeds, in the permuted order
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>

#include "scene/model_file.hpp"
#include "fc_link/autopilot_map.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace {

namespace veh = spade::vehicles;
namespace fcl = spade::fc_link;

struct Px4Motor {
    double px;  // forward, m (FRD)
    double py;  // right, m (FRD)
    double km;  // yaw moment coefficient: positive is counter-clockwise (the x500 Gazebo model's turningDirection pairs with it)
};

// 4001_gz_x500, read 2026-11-21: CA_ROTOR0..3 _PX, _PY, _KM.
constexpr std::array<Px4Motor, 4> kPx4X500{{{0.174, 0.174, 0.05}, {-0.174, -0.174, 0.05}, {0.174, -0.174, -0.05}, {-0.174, 0.174, -0.05}}};

[[nodiscard]] std::filesystem::path repo(const char* rel) { return std::filesystem::path(SPADE_REPO_DIR) / rel; }

// Spade's design frame is +X forward, +Z right (the frames section of the design record), so an FRD (forward, right) corner is Spade's (x, z).
void expect_table_matches(const spade::vehicles::ModelType& model, const char* what) {
    ASSERT_EQ(model.rotors.size(), 4u) << what;
    for (std::size_t i = 0; i < kPx4X500.size(); ++i) {
        const uint32_t rotor = fcl::kAutopilotQuadXToSpade[i];
        ASSERT_LT(rotor, model.rotors.size());
        const auto& r = model.rotors[rotor];
        EXPECT_EQ(r.local_pos.x > 0.0f, kPx4X500[i].px > 0.0) << what << ": autopilot motor " << i << " is on the wrong side forward and back";
        EXPECT_EQ(r.local_pos.z > 0.0f, kPx4X500[i].py > 0.0) << what << ": autopilot motor " << i << " is on the wrong side left and right";
        EXPECT_EQ(r.spin_dir, kPx4X500[i].km > 0.0 ? 1.0f : -1.0f) << what << ": autopilot motor " << i << " spins the wrong way";
    }
    // A permutation: every Spade rotor is driven exactly once.
    std::array<int, 4> used{};
    for (uint32_t t : fcl::kAutopilotQuadXToSpade) ++used[t];
    for (int u : used) EXPECT_EQ(u, 1) << what;
}

TEST(AutopilotMap, TheOrderTableIsPinnedToTheModelFilesGeometry) {
    const auto x500 = spade::scene::load_model_file(repo("tools/flow_movie/data/x500/x500.model.yaml"));
    ASSERT_TRUE(x500.has_value()) << x500.error().context;
    expect_table_matches(x500->model, "x500.model.yaml");
    const auto kit = spade::scene::load_model_file(repo("tools/flow_movie/data/openframe5f/openframe5f.model.yaml"));
    ASSERT_TRUE(kit.has_value()) << kit.error().context;
    expect_table_matches(kit->model, "openframe5f.model.yaml");
}

TEST(AutopilotMap, Px4OutputsAreTheChainsDuty) {
    for (const float o : {0.0f, 0.25f, 0.6f, 1.0f}) {
        const auto d = fcl::px4_output_to_duty(o);
        ASSERT_TRUE(d.has_value()) << o << ": " << d.error().context;
        EXPECT_DOUBLE_EQ(*d, static_cast<double>(o));
    }
    const auto low = fcl::px4_output_to_duty(-0.0005f);
    ASSERT_TRUE(low.has_value());
    EXPECT_EQ(*low, 0.0);
    const auto high = fcl::px4_output_to_duty(1.0005f);
    ASSERT_TRUE(high.has_value());
    EXPECT_EQ(*high, 1.0);
    for (const float bad : {-0.5f, 1.2f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        const auto d = fcl::px4_output_to_duty(bad);
        ASSERT_FALSE(d.has_value()) << bad;
        EXPECT_EQ(d.error().code, spade::Code::invalid_argument);
        EXPECT_NE(d.error().context.find("px4_output_to_duty"), std::string::npos) << d.error().context;
    }
}

TEST(AutopilotMap, ArdupilotPulsesAreTheChainsDuty) {
    const auto at = [](uint16_t pwm, uint16_t lo = 1000, uint16_t hi = 2000) { return fcl::ardupilot_pwm_to_duty(pwm, lo, hi); };
    ASSERT_TRUE(at(1000).has_value());
    EXPECT_EQ(*at(1000), 0.0);
    EXPECT_DOUBLE_EQ(*at(1500), 0.5);
    EXPECT_EQ(*at(2000), 1.0);
    EXPECT_EQ(*at(0), 0.0);      // no pulse: disarmed
    EXPECT_EQ(*at(900), 0.0);    // below the minimum
    EXPECT_EQ(*at(2100), 1.0);   // above the maximum
    EXPECT_DOUBLE_EQ(*at(1500, 1100, 1900), 0.5);
    EXPECT_DOUBLE_EQ(*at(1300, 1100, 1900), 0.25);
    const auto bad = at(1500, 2000, 1000);
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error().code, spade::Code::invalid_argument);
    EXPECT_FALSE(at(1500, 1500, 1500).has_value());
}

TEST(AutopilotMap, DutiesGoThroughTheChainInSpadesOrder) {
    const auto x500 = spade::scene::load_model_file(repo("tools/flow_movie/data/x500/x500.model.yaml"));
    ASSERT_TRUE(x500.has_value()) << x500.error().context;
    ASSERT_TRUE(x500->model.propulsion.has_value());
    const veh::PropulsionChain& chain = *x500->model.propulsion;
    const std::array<double, 4> duties{0.45, 0.55, 0.65, 0.75};  // in the AUTOPILOT's order: motors 1..4
    const auto cmd = fcl::autopilot_duties_to_rotor_commands(chain, duties, 1.225, 1.0);
    ASSERT_TRUE(cmd.has_value()) << cmd.error().context;
    for (std::size_t i = 0; i < duties.size(); ++i) {
        const auto direct = veh::rotor_omega_for_duty(chain, duties[i], 1.225, 1.0);
        ASSERT_TRUE(direct.has_value()) << direct.error().context;
        EXPECT_FLOAT_EQ((*cmd)[fcl::kAutopilotQuadXToSpade[i]], direct->omega_cmd) << "autopilot motor " << i;
    }
    // The four are different, so a wrong order cannot pass by symmetry: motor 2 (index 1) drives Spade rotor 2, motor 3 drives rotor 1.
    EXPECT_LT((*cmd)[0], (*cmd)[2]);
    EXPECT_LT((*cmd)[2], (*cmd)[1]);
    EXPECT_LT((*cmd)[1], (*cmd)[3]);
    // A duty outside [0, 1] is the chain function's refusal, with the motor named.
    const std::array<double, 4> bad{0.5, 1.5, 0.5, 0.5};
    const auto refused = fcl::autopilot_duties_to_rotor_commands(chain, bad, 1.225, 1.0);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("motor 1"), std::string::npos) << refused.error().context;
    const std::array<double, 3> short_list{0.5, 0.5, 0.5};
    EXPECT_FALSE(fcl::autopilot_duties_to_rotor_commands(chain, short_list, 1.225, 1.0).has_value());
}

}  // namespace
