// tests/test_fc_link_frames.cpp -- Spade's frames against the autopilots' NED and FRD (tools/fc_link/frames.hpp).
//
// The eleven vectors of docs/design/physics/plans/2026-11-21-fc-physics.md section 2 ("Test vectors") are the tests, checked to 1e-6, taken
// from Physics' table as written; then the rules the table implies (round trips over a seeded stream, the determinant, the sign of w),
// the anchored world's layout (x east, z south) and the latitude, longitude and height conversion (core/geodesy.hpp's exact one).
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "fc_link/frames.hpp"

namespace {

using namespace spade::fc_link::frames;

constexpr double kTol = 1e-6;

void expect_vec(const glm::dvec3& got, const glm::dvec3& want, const char* what) {
    EXPECT_NEAR(got.x, want.x, kTol) << what << " x";
    EXPECT_NEAR(got.y, want.y, kTol) << what << " y";
    EXPECT_NEAR(got.z, want.z, kTol) << what << " z";
}
// A quaternion is the same rotation as its negative; the converter returns the w >= 0 one, and Physics' table is written with w >= 0.
void expect_quat(const glm::dquat& got, double w, double x, double y, double z, const char* what) {
    EXPECT_NEAR(got.w, w, kTol) << what << " w";
    EXPECT_NEAR(got.x, x, kTol) << what << " x";
    EXPECT_NEAR(got.y, y, kTol) << what << " y";
    EXPECT_NEAR(got.z, z, kTol) << what << " z";
}

}  // namespace

// ---- Physics' eleven vectors (fc-physics section 2) ----

TEST(FcLinkFrames, V01_LevelAtRestTheAccelerometerReadsMinusGDownInFrd) {
    expect_vec(body_to_frd(compass_x_north(), {0, 9.80665, 0}), {0, 0, -9.80665}, "level at rest");
}
TEST(FcLinkFrames, V02_FreeFallReadsZero) { expect_vec(body_to_frd(compass_x_north(), {0, 0, 0}), {0, 0, 0}, "free fall"); }
TEST(FcLinkFrames, V03_RollRateAboutTheForwardAxis) { expect_vec(body_to_frd(compass_x_north(), {0.1, 0, 0}), {0.1, 0, 0}, "roll rate"); }
TEST(FcLinkFrames, V04_YawRateTurningLeftIsNegativeDown) { expect_vec(body_to_frd(compass_x_north(), {0, 0.2, 0}), {0, 0, -0.2}, "yaw rate left"); }
TEST(FcLinkFrames, V05_PitchRateNoseUpAboutPlusZ) { expect_vec(body_to_frd(compass_x_north(), {0, 0, 0.3}), {0, 0.3, 0}, "pitch rate nose up"); }
TEST(FcLinkFrames, V06_PositionTenForwardFiveUpThreeLeft) {
    expect_vec(position_to_ned(compass_x_north(), {10, 5, -3}, {0, 0, 0}), {10, -3, -5}, "N 10, E -3, D -5");
}
TEST(FcLinkFrames, V07_VelocityOneTwoThree) { expect_vec(world_to_ned(compass_x_north(), {1, 2, 3}), {1, 3, -2}, "climbing at 2 m/s"); }
TEST(FcLinkFrames, V08_FacingWest) {
    expect_quat(attitude_to_ned_frd(compass_x_north(), glm::dquat(0.70710678, 0, 0.70710678, 0)), 0.70710678, 0, 0, -0.70710678, "facing west");
}
TEST(FcLinkFrames, V09_PitchUpThirtyDegrees) {
    expect_quat(attitude_to_ned_frd(compass_x_north(), glm::dquat(0.96592583, 0, 0, 0.25881905)), 0.96592583, 0, 0.25881905, 0, "pitch up 30");
}
TEST(FcLinkFrames, V10_RollRightTwentyDegrees) {
    const double c = std::cos(glm::radians(10.0)), s = std::sin(glm::radians(10.0));
    expect_quat(attitude_to_ned_frd(compass_x_north(), glm::dquat(c, s, 0, 0)), c, s, 0, 0, "roll right 20");
}
TEST(FcLinkFrames, V11_AFieldOnABodyFacingWest) {
    // field (N, E, D) = (0.2, 0, 0.4): in Spade's world (0.2, -0.4, 0); a body facing west sees it as FRD (0, 0.2, 0.4)
    const FrameMap m = compass_x_north();
    const glm::dvec3 ned = world_to_ned(m, {0.2, -0.4, 0});
    expect_vec(ned, {0.2, 0, 0.4}, "the field in NED");
    const glm::dquat q = attitude_to_ned_frd(m, glm::dquat(0.70710678, 0, 0.70710678, 0));
    expect_vec(glm::inverse(q) * ned, {0, 0.2, 0.4}, "the field in the body's FRD");
}

// ---- what the table implies ----

TEST(FcLinkFrames, TheMapIsAProperRotationSoAxialVectorsAndQuaternionsFollow) {
    for (const FrameMap& m : {compass_x_north(), anchored_world()}) {
        EXPECT_NEAR(glm::determinant(m.world), 1.0, 1e-12);
        EXPECT_NEAR(glm::determinant(m.body), 1.0, 1e-12);
    }
}

TEST(FcLinkFrames, TheInverseMapsReturnWhatWentIn) {
    std::mt19937 rng(20261121);
    std::uniform_real_distribution<double> u(-100.0, 100.0);
    for (const FrameMap& m : {compass_x_north(), anchored_world()}) {
        for (int i = 0; i < 200; ++i) {
            const glm::dvec3 v(u(rng), u(rng), u(rng));
            expect_vec(ned_to_world(m, world_to_ned(m, v)), v, "world round trip");
            expect_vec(frd_to_body(m, body_to_frd(m, v)), v, "body round trip");
            const glm::dquat q = glm::normalize(glm::dquat(u(rng), u(rng), u(rng), u(rng)));
            const glm::dquat n = attitude_to_ned_frd(m, q);
            EXPECT_GE(n.w, 0.0) << "w >= 0";
            EXPECT_NEAR(glm::length(n), 1.0, 1e-12) << "still a unit quaternion";
            const glm::dquat back = attitude_from_ned_frd(m, n);
            // the same rotation: compare up to sign
            const double dot = std::abs(glm::dot(back, q));
            EXPECT_NEAR(dot, 1.0, 1e-9) << "attitude round trip";
        }
    }
}

TEST(FcLinkFrames, TheAttitudeAgreesWithTheVectorMapOnARotatedVector) {
    // R_ned,frd must carry a body vector to the same NED vector the two vector maps give: world(R_spade v) == R' (body(v)).
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (const FrameMap& m : {compass_x_north(), anchored_world()}) {
        for (int i = 0; i < 100; ++i) {
            const glm::dquat q = glm::normalize(glm::dquat(u(rng), u(rng), u(rng), u(rng)));
            const glm::dvec3 v(u(rng), u(rng), u(rng));
            expect_vec(attitude_to_ned_frd(m, q) * body_to_frd(m, v), world_to_ned(m, q * v), "R' body(v) == world(R v)");
        }
    }
}

TEST(FcLinkFrames, GravityIsOneGOnBothSides) {
    // Spade's gravity (0, -9.80665, 0) is Down 9.80665 in NED; PX4's CONSTANTS_ONE_G is 9.80665 m/s^2.
    expect_vec(world_to_ned(compass_x_north(), {0, -9.80665, 0}), {0, 0, 9.80665}, "gravity in NED");
}

// ---- the anchored world (x east, y up, z south) ----

TEST(FcLinkFrames, AnAnchoredWorldHasEastOnXAndNorthOnMinusZ) {
    const FrameMap m = anchored_world();
    expect_vec(position_to_ned(m, {10, 5, -20}, {0, 0, 0}), {20, 10, -5}, "10 east, 20 north, 5 up");
    expect_vec(world_to_ned(m, {1, 2, 3}), {-3, 1, -2}, "velocity");
}

TEST(FcLinkFrames, ADroneAtIdentityInAnAnchoredWorldFacesEast) {
    // identity orientation: the body's +X (forward) is the world's +X, which is East; level, so heading +90 degrees about Down.
    expect_quat(attitude_to_ned_frd(anchored_world(), glm::dquat(1, 0, 0, 0)), 0.70710678, 0, 0, 0.70710678, "facing east");
    // the same drone turned to face north: yaw -90 degrees about +Y would take +X to... +Z south, so +90 takes +X to -Z, north
    expect_quat(attitude_to_ned_frd(anchored_world(), glm::dquat(0.70710678, 0, 0.70710678, 0)), 1, 0, 0, 0, "facing north");
}

TEST(FcLinkFrames, AHomeShiftsThePosition) {
    expect_vec(position_to_ned(compass_x_north(), {110, 25, 7}, {100, 20, 4}), {10, 3, -5}, "about a home");
}

// ---- latitude, longitude and height ----

TEST(FcLinkFrames, NedGoesToGeodeticByTheExactConversionAndBackToAMillimetre) {
    const spade::geo::Geodetic anchor{47.397742, 8.545594, 488.0};
    const spade::geo::LocalFrame frame = spade::geo::make_local_frame(anchor);
    const spade::geo::Geodetic at_home = ned_to_geodetic(frame, {0, 0, 0});
    EXPECT_NEAR(at_home.lat_deg, anchor.lat_deg, 1e-9);
    EXPECT_NEAR(at_home.lon_deg, anchor.lon_deg, 1e-9);
    EXPECT_NEAR(at_home.height_m, anchor.height_m, 1e-6);

    const spade::geo::Geodetic north = ned_to_geodetic(frame, {1000, 0, 0});
    EXPECT_GT(north.lat_deg, anchor.lat_deg) << "north is up in latitude";
    EXPECT_NEAR(north.lon_deg, anchor.lon_deg, 1e-6);
    const spade::geo::Geodetic east = ned_to_geodetic(frame, {0, 1000, 0});
    EXPECT_GT(east.lon_deg, anchor.lon_deg) << "east is up in longitude";
    const spade::geo::Geodetic up = ned_to_geodetic(frame, {0, 0, -50});
    EXPECT_GT(up.height_m, anchor.height_m + 49.0) << "Down is negative height";

    std::mt19937 rng(1121);
    std::uniform_real_distribution<double> u(-1000.0, 1000.0);
    for (int i = 0; i < 200; ++i) {
        const glm::dvec3 ned(u(rng), u(rng), u(rng) * 0.1);
        expect_vec(geodetic_to_ned(frame, ned_to_geodetic(frame, ned)), ned, "round trip over 1 km");
        const glm::dvec3 back = geodetic_to_ned(frame, ned_to_geodetic(frame, ned));
        EXPECT_LT(glm::length(back - ned), 1e-3) << "within a millimetre";
    }
}
