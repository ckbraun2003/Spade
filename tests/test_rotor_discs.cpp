// Rotors to actuator discs (P7a; physics/airflow/rotor_discs.hpp, and the
// rotor rows' adapter vehicles/rotor_discs.hpp): the one rule from a body's
// rotors to the airflow's discs. Expectations are worked by hand from the
// geometry (TD-4); the quad case's are flow_movie's own mapping, which now
// comes through the rule.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/airflow/rotor_discs.hpp"
#include "state/layout.hpp"
#include "vehicles/rotor.hpp"
#include "vehicles/rotor_discs.hpp"

namespace {

using namespace spade::physics::airflow;
using spade::Result;

constexpr float kHalfPi = 1.57079632679f;

void expect_near(glm::vec3 got, glm::vec3 want, const char* what) {
    for (int a = 0; a < 3; ++a) EXPECT_NEAR(got[a], want[a], 1.0e-6f) << what << " [" << a << "]";
}

// A body at (1, 2, 3) m turned a quarter turn about world +z (body x -> world
// y, body y -> world -x), in a 1.6 m grid whose minimum corner is at
// (0.5, 1.5, 2.5) m. Rotor 0: hub (0.1, 0.02, 0) in the body, thrust along
// body +y, spin +1. Rotor 1: hub (0, 0, 0.1), tipped a quarter turn about its
// x so it thrusts along body +z, spin -1.
TEST(AirflowRotorDiscs, ATurnedBodysRotorsGiveTheirDiscsByHand) {
    const GridShape grid{32, 32, 32, 0.05f};
    const DiscBody body{.com_world = {1.0f, 2.0f, 3.0f},
                        .orient_world = glm::angleAxis(kHalfPi, glm::vec3(0.0f, 0.0f, 1.0f))};
    const std::array<DiscRotor, 2> rotors{{
        {.hub_body = {0.1f, 0.02f, 0.0f}, .radius = 0.05f, .spin_dir = 1.0f, .thrust = 2.0f, .torque = 0.03f},
        {.hub_body = {0.0f, 0.0f, 0.1f},
         .orient_body = glm::angleAxis(kHalfPi, glm::vec3(1.0f, 0.0f, 0.0f)),
         .radius = 0.05f,
         .spin_dir = -1.0f,
         .thrust = 1.5f,
         .torque = 0.02f},
    }};
    std::array<ActuatorDisc, 2> out{};
    const Result<void> ok = discs_from_rotors(rotors, body, {0.5f, 1.5f, 2.5f}, grid, out);
    ASSERT_TRUE(ok.has_value()) << ok.error().context;

    // Rotor 0: the hub turns to (-0.02, 0.1, 0); the thrust axis, body +y,
    // turns to world -x, so the disc pushes the air along +x.
    expect_near(out[0].centre, {0.48f, 0.6f, 0.5f}, "rotor 0 centre");
    expect_near(out[0].axis, {1.0f, 0.0f, 0.0f}, "rotor 0 axis");
    EXPECT_EQ(out[0].radius, 0.05f);
    EXPECT_EQ(out[0].thrust, 2.0f);
    EXPECT_EQ(out[0].torque, -0.03f) << "spin +1: the air takes -Q about the disc's axis";

    // Rotor 1: the hub (0, 0, 0.1) is on the turn's axis; local +y tips to
    // body +z, which the body's turn leaves alone, so the disc pushes along -z.
    expect_near(out[1].centre, {0.5f, 0.5f, 0.6f}, "rotor 1 centre");
    expect_near(out[1].axis, {0.0f, 0.0f, -1.0f}, "rotor 1 axis");
    EXPECT_EQ(out[1].thrust, 1.5f);
    EXPECT_EQ(out[1].torque, 0.02f) << "spin -1: the air takes +Q";
}

// The swirl's sense is the spin's: +1 gives -Q, -1 gives +Q, 0 gives none;
// any other spin is not a direction and is refused.
TEST(AirflowRotorDiscs, TheSpinSetsTheSwirlsSense) {
    const GridShape grid{16, 16, 16, 0.05f};
    const DiscBody body{.com_world = {0.4f, 0.4f, 0.4f}};
    for (const auto& [spin, want] : {std::pair{1.0f, -0.01f}, std::pair{-1.0f, 0.01f}, std::pair{0.0f, 0.0f}}) {
        const std::array<DiscRotor, 1> r{{{.radius = 0.05f, .spin_dir = spin, .thrust = 1.0f, .torque = 0.01f}}};
        std::array<ActuatorDisc, 1> out{};
        ASSERT_TRUE(discs_from_rotors(r, body, glm::vec3(0.0f), grid, out).has_value()) << spin;
        EXPECT_EQ(out[0].torque, want) << spin;
    }
    const std::array<DiscRotor, 1> half{{{.radius = 0.05f, .spin_dir = 0.5f, .thrust = 1.0f, .torque = 0.01f}}};
    std::array<ActuatorDisc, 1> out{};
    const Result<void> r = discs_from_rotors(half, body, glm::vec3(0.0f), grid, out);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("rotor 0"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("spin"), std::string::npos) << r.error().context;
}

// A refusal names its rotor: rotor 2's disc reaches past the grid. A body
// orientation not of unit length, and an output of the wrong size, are refused.
TEST(AirflowRotorDiscs, ARefusalNamesItsRotor) {
    const GridShape grid{16, 16, 16, 0.05f};  // 0.8 m
    const DiscBody body{.com_world = {0.4f, 0.4f, 0.4f}};
    std::array<DiscRotor, 3> r{};
    for (DiscRotor& d : r) d = {.radius = 0.05f, .spin_dir = 1.0f, .thrust = 1.0f, .torque = 0.01f};
    r[2].hub_body = glm::vec3(0.3f, 0.0f, 0.0f);  // support to 0.4 + 0.3 + 0.05 + 0.1 = 0.85 m > 0.8 m
    std::array<ActuatorDisc, 3> out{};
    const Result<void> e = discs_from_rotors(r, body, glm::vec3(0.0f), grid, out);
    ASSERT_FALSE(e.has_value());
    EXPECT_NE(e.error().context.find("rotor 2"), std::string::npos) << e.error().context;

    r[2].hub_body = glm::vec3(0.0f);
    DiscBody skewed = body;
    skewed.orient_world = glm::quat(1.1f, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(discs_from_rotors(r, skewed, glm::vec3(0.0f), grid, out).has_value());
    std::array<ActuatorDisc, 2> short_out{};
    EXPECT_FALSE(discs_from_rotors(r, body, glm::vec3(0.0f), grid, short_out).has_value());
    EXPECT_TRUE(discs_from_rotors(r, body, glm::vec3(0.0f), grid, out).has_value());
}

// flow_movie's quad case (tools/flow_movie/quad_case.cpp) builds its discs
// through the rule: the compiled reference quad level at the world origin,
// the grid's corner at -com. The rule gives the case's original hand mapping
// bit for bit -- centre com + hub, axis -y, torque -spin Q -- so the films
// already cached are the rule's.
TEST(AirflowRotorDiscs, TheQuadCasesDiscsAreItsHandMappingBitForBit) {
    constexpr float kThrust = 1.28957453f;
    constexpr float kTorque = 0.0131021f;
    constexpr float kRadius = 0.0762f;
    const glm::vec3 hubs[4] = {{0.11f, 0.0241444856f, 0.0f},
                               {0.0f, 0.0241444856f, 0.11f},
                               {-0.11f, 0.0241444856f, 0.0f},
                               {0.0f, 0.0241444856f, -0.11f}};
    const float spins[4] = {1.0f, -1.0f, 1.0f, -1.0f};
    const glm::vec3 com(0.40f, 0.96f - 0.0241444856f, 0.40f);
    const GridShape grid{80, 128, 80, 0.01f};
    std::array<DiscRotor, 4> r{};
    for (int i = 0; i < 4; ++i) {
        r[i] = {.hub_body = hubs[i], .radius = kRadius, .spin_dir = spins[i], .thrust = kThrust, .torque = kTorque};
    }
    std::array<ActuatorDisc, 4> out{};
    ASSERT_TRUE(discs_from_rotors(r, DiscBody{}, -com, grid, out).has_value());
    for (int i = 0; i < 4; ++i) {
        const glm::vec3 centre = com + hubs[i];
        EXPECT_EQ(out[i].centre.x, centre.x) << i;
        EXPECT_EQ(out[i].centre.y, centre.y) << i;
        EXPECT_EQ(out[i].centre.z, centre.z) << i;
        EXPECT_EQ(out[i].axis.x, 0.0f) << i;
        EXPECT_EQ(out[i].axis.y, -1.0f) << i;
        EXPECT_EQ(out[i].axis.z, 0.0f) << i;
        EXPECT_EQ(out[i].radius, kRadius) << i;
        EXPECT_EQ(out[i].thrust, kThrust) << i;
        EXPECT_EQ(out[i].torque, -spins[i] * kTorque) << i;
    }
}

// The rotor rows' adapter: static thrust k_T w^2 and Q = k_Q w^2 from one
// w^2, as apply_rotors() forms them; a disabled row is a zero disc in its
// place; the body row's pose; a refusal names the row.
TEST(VehicleRotorDiscs, RowsGiveTheirStaticThrustAndTorque) {
    using spade::vehicles::RotorRow;
    EXPECT_EQ(spade::vehicles::kRotorLocalThrustAxis, glm::vec3(0.0f, 1.0f, 0.0f)) << "the rule's thrust axis";
    const GridShape grid{64, 64, 64, 0.01f};  // 0.64 m
    spade::BodyState body{};
    body.pos = glm::vec3(1.2f, 0.2f, -0.3f);
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    std::array<RotorRow, 2> rows{};
    for (RotorRow& row : rows) {
        row.enabled = 1u;
        row.radius = 0.0762f;
        row.local_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        row.omega = 801.25f;
        row.thrust_coeff = 2.0087e-6f;
        row.torque_coeff = 2.04075653e-8f;
    }
    rows[0].local_pos = glm::vec3(0.11f, 0.024f, 0.0f);
    rows[0].spin_dir = 1.0f;
    rows[1].local_pos = glm::vec3(-0.11f, 0.024f, 0.0f);
    rows[1].spin_dir = -1.0f;
    rows[1].enabled = 0u;
    const glm::vec3 origin(0.9f, 0.0f, -0.5f);  // the body at (0.3, 0.2, 0.2) m of the grid
    std::array<ActuatorDisc, 2> out{};
    const Result<void> ok = spade::vehicles::rotor_discs(rows, body, origin, grid, out);
    ASSERT_TRUE(ok.has_value()) << ok.error().context;
    const float w2 = rows[0].omega * rows[0].omega;
    EXPECT_EQ(out[0].thrust, rows[0].thrust_coeff * w2);
    EXPECT_EQ(out[0].torque, -(rows[0].torque_coeff * w2));
    expect_near(out[0].centre, body.pos + rows[0].local_pos - origin, "row 0 centre");
    EXPECT_EQ(out[1].thrust, 0.0f) << "a disabled row";
    EXPECT_EQ(out[1].torque, 0.0f);
    expect_near(out[1].centre, body.pos + rows[1].local_pos - origin, "row 1 keeps its place");

    rows[1].enabled = 1u;
    rows[1].local_pos = glm::vec3(0.3f, 0.0f, 0.0f);  // support to 0.6 + 0.0962 m, past 0.64 m
    const Result<void> e = spade::vehicles::rotor_discs(rows, body, origin, grid, out);
    ASSERT_FALSE(e.has_value());
    EXPECT_NE(e.error().context.find("rotor 1"), std::string::npos) << e.error().context;
}

}  // namespace
