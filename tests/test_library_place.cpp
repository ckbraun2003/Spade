// compose_placement (L88l; scene/library_place.hpp; the library contract's section 4).
//
// What is pinned here:
//   - THE CHAIN: pose = parent . install_point . adjust . part_to_mount, against closed-form hand products (a yawed parent turns the
//     install point's offset; the adjustment acts in the install point's own frame); an empty item is the identity.
//   - THE ANGLES: Rz(yaw) . Ry(pitch) . Rx(roll), intrinsic; a positive pitch tips +X toward -Z; exact at the multiples of 90.
//   - A HAND-COMPOSED CASE: frame -> motor -> prop on the motor's prop_hub, and a camera tilted on its mount.
//   - THE SPELLING: the result is normalised and has w >= 0 (and, at w == 0, a positive first non-zero axis).
//   - THE RANGE: the frame part's adjust_range per axis, bounds inclusive; an absent adjust is zero and is checked; no range, no check.
//   - A BAD ITEM does not hide the others: no pose, its own refusal, the results in request order with their keys.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "scene/library_place.hpp"

namespace {

namespace sc = spade::scene;
using Vec = std::array<double, 3>;

constexpr double kS = 0.7071067811865476;  // sin 45 = cos 45

// An independent rotation of v by the unit quaternion q = [w, x, y, z] (the test's own, not the engine's).
Vec rotate(const std::array<double, 4>& q, const Vec& v) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double m[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                            {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                            {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
    Vec out{};
    for (int i = 0; i < 3; ++i) out[static_cast<std::size_t>(i)] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
    return out;
}

void expect_vec(const Vec& got, const Vec& want, double tol, const char* what) {
    for (std::size_t i = 0; i < 3; ++i) EXPECT_NEAR(got[i], want[i], tol) << what << " [" << i << "]";
}

sc::PlacementItem item(std::string key) {
    sc::PlacementItem i;
    i.key = std::move(key);
    return i;
}

sc::SetPose pose(Vec p, std::array<double, 4> q = {1, 0, 0, 0}) {
    sc::SetPose s;
    s.pos = p;
    s.quat = q;
    return s;
}

sc::PlacedItem one(const sc::PlacementItem& i) {
    const std::vector<sc::PlacedItem> r = sc::compose_placement({i});
    EXPECT_EQ(r.size(), 1u);
    return r.empty() ? sc::PlacedItem{} : r[0];
}

TEST(LibraryPlace, AnItemWithNothingButAnInstallPointIsThatPoint) {
    sc::PlacementItem i = item("a");
    i.install_point = pose({0.09, -0.09, 0.02}, {kS, 0, 0, kS});
    const sc::PlacedItem r = one(i);
    ASSERT_TRUE(r.pose) << r.refusal->message;
    EXPECT_EQ(r.key, "a");
    EXPECT_FALSE(r.refusal);
    expect_vec(r.pose->pos, {0.09, -0.09, 0.02}, 1e-15, "pos");
    EXPECT_NEAR(r.pose->quat[0], kS, 1e-15);
    EXPECT_NEAR(r.pose->quat[3], kS, 1e-15);
    // all absent and an identity install point: the identity
    const sc::PlacedItem id = one(item("id"));
    ASSERT_TRUE(id.pose);
    EXPECT_EQ(id.pose->pos, (std::array<double, 3>{0, 0, 0}));
    EXPECT_EQ(id.pose->quat, (std::array<double, 4>{1, 0, 0, 0}));
}

TEST(LibraryPlace, AYawedParentTurnsTheInstallPointsOffsetAndKeepsItsOwnOrientation) {
    sc::PlacementItem i = item("motor_fl");
    i.parent = pose({1, 2, 3}, {kS, 0, 0, kS});  // yaw 90: x -> y, y -> -x
    i.install_point = pose({0.09, 0.09, 0});
    const sc::PlacedItem r = one(i);
    ASSERT_TRUE(r.pose);
    expect_vec(r.pose->pos, {0.91, 2.09, 3}, 1e-15, "pos");
    expect_vec(rotate(r.pose->quat, {1, 0, 0}), {0, 1, 0}, 1e-15, "x axis");
}

TEST(LibraryPlace, TheAdjustmentActsInTheInstallPointsOwnFrameAfterItsPose) {
    sc::PlacementItem i = item("cam");
    i.install_point = pose({0.09, 0.09, 0}, {kS, 0, 0, kS});  // the mount is turned 90 about z
    sc::SetAdjust a;
    a.pos = {0.01, 0, 0};  // along the MOUNT's x, which is the drone's +y
    i.adjust = a;
    const sc::PlacedItem r = one(i);
    ASSERT_TRUE(r.pose);
    expect_vec(r.pose->pos, {0.09, 0.10, 0}, 1e-15, "pos");
}

TEST(LibraryPlace, PartToMountComesLastInThePartsOwnFrame) {
    sc::PlacementItem i = item("p");
    i.install_point = pose({0, 0, 0}, {kS, 0, 0, kS});
    i.part_to_mount = pose({0, 0.5, 0});  // the part's own +y, which the 90-degree yaw makes -x
    const sc::PlacedItem r = one(i);
    ASSERT_TRUE(r.pose);
    expect_vec(r.pose->pos, {-0.5, 0, 0}, 1e-15, "pos");
}

TEST(LibraryPlace, TheAnglesAreRzRyRxIntrinsicAndAPositivePitchTipsXTowardMinusZ) {
    sc::PlacementItem pitch = item("pitch");
    sc::SetAdjust p;
    p.pitch_deg = 90;
    pitch.adjust = p;
    const sc::PlacedItem r = one(pitch);
    ASSERT_TRUE(r.pose);
    expect_vec(rotate(r.pose->quat, {1, 0, 0}), {0, 0, -1}, 1e-15, "+x under pitch 90");

    // yaw 90 then pitch 90, each about the axes made so far: Rz(90) . Ry(90) takes +x to -z (the other order would give +y)
    sc::PlacementItem both = item("both");
    sc::SetAdjust b;
    b.yaw_deg = 90;
    b.pitch_deg = 90;
    both.adjust = b;
    const sc::PlacedItem rb = one(both);
    ASSERT_TRUE(rb.pose);
    expect_vec(rotate(rb.pose->quat, {1, 0, 0}), {0, 0, -1}, 1e-15, "x");
    expect_vec(rotate(rb.pose->quat, {0, 1, 0}), {-1, 0, 0}, 1e-15, "y");

    // roll 90 about +x takes +y to +z
    sc::PlacementItem roll = item("roll");
    sc::SetAdjust c;
    c.roll_deg = 90;
    roll.adjust = c;
    const sc::PlacedItem rr = one(roll);
    ASSERT_TRUE(rr.pose);
    expect_vec(rotate(rr.pose->quat, {0, 1, 0}), {0, 0, 1}, 1e-15, "y under roll 90");

    // yaw + pitch + roll together: Rz(30) Ry(20) Rx(10) against the test's own matrix product
    sc::PlacementItem all = item("all");
    sc::SetAdjust d;
    d.yaw_deg = 30;
    d.pitch_deg = 20;
    d.roll_deg = 10;
    all.adjust = d;
    const sc::PlacedItem ra = one(all);
    ASSERT_TRUE(ra.pose);
    const double pi = 3.14159265358979323846;
    const double cy = std::cos(30 * pi / 180), sy = std::sin(30 * pi / 180);
    const double cp = std::cos(20 * pi / 180), sp = std::sin(20 * pi / 180);
    const double cr = std::cos(10 * pi / 180), sr = std::sin(10 * pi / 180);
    // R = Rz Ry Rx; its first column is R e_x, the second R e_y, the third R e_z
    expect_vec(rotate(ra.pose->quat, {1, 0, 0}), {cy * cp, sy * cp, -sp}, 1e-14, "R ex");
    expect_vec(rotate(ra.pose->quat, {0, 1, 0}), {cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr}, 1e-14, "R ey");
    expect_vec(rotate(ra.pose->quat, {0, 0, 1}), {cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr}, 1e-14, "R ez");
}

TEST(LibraryPlace, AFrameMotorPropChainAndATiltedCameraMatchTheHandProduct) {
    const sc::SetPose frame = pose({1, 2, 3}, {kS, 0, 0, kS});  // the drone's frame part, yawed 90 in the world
    // motor_fl on the frame at (0.09, 0.09, 0)
    sc::PlacementItem motor = item("motor_fl");
    motor.parent = frame;
    motor.install_point = pose({0.09, 0.09, 0});
    // prop on the MOTOR's prop_hub at (0, 0, 0.03): the parent is the motor's own pose, as the caller composed it
    // camera on the frame's mount at (0.12, 0, 0.02), tilted down 10 degrees (a negative pitch lifts +x toward +z)
    sc::PlacementItem camera = item("camera");
    camera.parent = frame;
    camera.install_point = pose({0.12, 0, 0.02});
    sc::SetAdjust tilt;
    tilt.pitch_deg = -10;
    camera.adjust = tilt;
    const std::vector<sc::PlacedItem> first = sc::compose_placement({motor, camera});
    ASSERT_EQ(first.size(), 2u);
    ASSERT_TRUE(first[0].pose);
    ASSERT_TRUE(first[1].pose);
    expect_vec(first[0].pose->pos, {0.91, 2.09, 3}, 1e-15, "motor pos");
    sc::PlacementItem prop = item("prop_fl");
    prop.parent = *first[0].pose;
    prop.install_point = pose({0, 0, 0.03});
    const sc::PlacedItem rp = one(prop);
    ASSERT_TRUE(rp.pose);
    expect_vec(rp.pose->pos, {0.91, 2.09, 3.03}, 1e-15, "prop pos");
    expect_vec(first[1].pose->pos, {1, 2.12, 3.02}, 1e-15, "camera pos");
    const double c10 = std::cos(10 * 3.14159265358979323846 / 180), s10 = std::sin(10 * 3.14159265358979323846 / 180);
    // the camera's forward axis: the frame's yaw takes (cos10, 0, sin10) to (0, cos10, sin10)
    expect_vec(rotate(first[1].pose->quat, {1, 0, 0}), {0, c10, s10}, 1e-14, "camera forward");
}

TEST(LibraryPlace, TheResultIsNormalisedWithANonNegativeW) {
    sc::PlacementItem i = item("n");
    i.install_point = pose({0, 0, 0}, {-kS, 0, 0, -kS});  // the same rotation spelt with w < 0
    const sc::PlacedItem r = one(i);
    ASSERT_TRUE(r.pose);
    EXPECT_GE(r.pose->quat[0], 0.0);
    EXPECT_NEAR(r.pose->quat[0], kS, 1e-15);
    EXPECT_NEAR(r.pose->quat[3], kS, 1e-15);
    // a product of two 90-degree yaws is a 180-degree yaw: w is 0 and the first non-zero axis (z) is positive
    sc::PlacementItem half = item("half");
    half.parent = pose({0, 0, 0}, {kS, 0, 0, kS});
    half.install_point = pose({0, 0, 0}, {kS, 0, 0, kS});
    const sc::PlacedItem h = one(half);
    ASSERT_TRUE(h.pose);
    EXPECT_NEAR(h.pose->quat[0], 0.0, 1e-15);
    EXPECT_NEAR(h.pose->quat[3], 1.0, 1e-15);
    const std::array<double, 4>& q = h.pose->quat;
    EXPECT_NEAR(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1.0, 1e-15);
    // an input a little off unit length (within 1e-6) comes out unit
    sc::PlacementItem off = item("off");
    off.install_point = pose({0, 0, 0}, {1.0000004, 0, 0, 0});
    const sc::PlacedItem o = one(off);
    ASSERT_TRUE(o.pose);
    EXPECT_EQ(o.pose->quat, (std::array<double, 4>{1, 0, 0, 0}));
}

sc::PlacementItem ranged(std::string key) {
    sc::PlacementItem i = item(std::move(key));
    i.install_point = pose({0, 0, 0});
    sc::AdjustRange r;
    r.pos_min = {-0.01, -0.01, -0.01};
    r.pos_max = {0.01, 0.01, 0.01};
    r.yaw_deg = {-5, 5};
    r.pitch_deg = {-10, 60};
    r.roll_deg = {0, 0};
    i.adjust_range = r;
    return i;
}

TEST(LibraryPlace, TheAdjustRangeIsCheckedPerAxisWithInclusiveBounds) {
    sc::PlacementItem at_bounds = ranged("bounds");
    sc::SetAdjust a;
    a.pos = {0.01, -0.01, 0.01};
    a.yaw_deg = 5;
    a.pitch_deg = -10;
    a.roll_deg = 0;
    at_bounds.adjust = a;
    const sc::PlacedItem ok = one(at_bounds);
    EXPECT_TRUE(ok.pose) << (ok.refusal ? ok.refusal->message : "");

    const auto refused = [](sc::PlacementItem i, const sc::SetAdjust& adj) {
        i.adjust = adj;
        return one(i);
    };
    sc::SetAdjust pitch;
    pitch.pitch_deg = -10.0001;
    sc::PlacedItem r = refused(ranged("k"), pitch);
    EXPECT_FALSE(r.pose);
    ASSERT_TRUE(r.refusal);
    EXPECT_EQ(r.refusal->code, "adjust_out_of_range");
    EXPECT_EQ(r.refusal->field, "adjust.pitch_deg");
    EXPECT_EQ(r.key, "k");
    EXPECT_FALSE(r.refusal->message.empty());

    sc::SetAdjust roll;
    roll.roll_deg = 0.1;
    r = refused(ranged("k"), roll);
    ASSERT_TRUE(r.refusal);
    EXPECT_EQ(r.refusal->field, "adjust.roll_deg");

    sc::SetAdjust z;
    z.pos = {0, 0, 0.011};
    r = refused(ranged("k"), z);
    ASSERT_TRUE(r.refusal);
    EXPECT_EQ(r.refusal->code, "adjust_out_of_range");
    EXPECT_EQ(r.refusal->field, "adjust.pos[2]");

    // two axes out: the first in the order pos, yaw, pitch, roll is named
    sc::SetAdjust two;
    two.yaw_deg = 6;
    two.pitch_deg = 61;
    r = refused(ranged("k"), two);
    ASSERT_TRUE(r.refusal);
    EXPECT_EQ(r.refusal->field, "adjust.yaw_deg");
}

TEST(LibraryPlace, AnAbsentAdjustIsZeroAndIsCheckedAgainstARangeThatIsThere) {
    EXPECT_TRUE(one(ranged("zero")).pose) << "0 is inside every axis of the range";
    sc::PlacementItem excluded = ranged("excl");
    excluded.adjust_range->yaw_deg = {1, 5};
    const sc::PlacedItem r = one(excluded);
    EXPECT_FALSE(r.pose);
    ASSERT_TRUE(r.refusal);
    EXPECT_EQ(r.refusal->code, "adjust_out_of_range");
    EXPECT_EQ(r.refusal->field, "adjust.yaw_deg");
    // no range: nothing is checked, however large the adjustment
    sc::PlacementItem free = item("free");
    sc::SetAdjust big;
    big.pitch_deg = 170;
    big.pos = {5, 5, 5};
    free.adjust = big;
    EXPECT_TRUE(one(free).pose);
}

TEST(LibraryPlace, ABadItemHasNoPoseAndItsOwnRefusalAndTheOthersAreStillComposed) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    sc::PlacementItem good1 = item("g1");
    good1.install_point = pose({1, 0, 0});
    sc::PlacementItem nan_pos = item("nan");
    nan_pos.install_point = pose({nan, 0, 0});
    sc::PlacementItem zero_quat = item("zero");
    zero_quat.install_point = pose({0, 0, 0}, {0, 0, 0, 0});
    sc::PlacementItem not_unit = item("notunit");
    not_unit.parent = pose({0, 0, 0}, {2, 0, 0, 0});
    sc::PlacementItem inverted = ranged("inverted");
    inverted.adjust_range->yaw_deg = {5, -5};
    sc::PlacementItem inf_angle = item("inf");
    sc::SetAdjust bad;
    bad.yaw_deg = std::numeric_limits<double>::infinity();
    inf_angle.adjust = bad;
    sc::PlacementItem good2 = item("g2");
    good2.install_point = pose({2, 0, 0});

    const std::vector<sc::PlacedItem> r = sc::compose_placement({good1, nan_pos, zero_quat, not_unit, inverted, inf_angle, good2});
    ASSERT_EQ(r.size(), 7u);
    const std::array<const char*, 7> keys = {"g1", "nan", "zero", "notunit", "inverted", "inf", "g2"};
    for (std::size_t i = 0; i < r.size(); ++i) EXPECT_EQ(r[i].key, keys[i]) << "request order";
    EXPECT_TRUE(r[0].pose);
    EXPECT_TRUE(r[6].pose);
    expect_vec(r[6].pose->pos, {2, 0, 0}, 0, "g2");
    const std::array<const char*, 7> fields = {"", "install_point.pos", "install_point.quat", "parent.quat", "adjust_range.yaw_deg", "adjust.yaw_deg", ""};
    for (std::size_t i = 1; i < 6; ++i) {
        EXPECT_FALSE(r[i].pose) << keys[i];
        ASSERT_TRUE(r[i].refusal) << keys[i];
        EXPECT_EQ(r[i].refusal->code, "invalid_request") << keys[i];
        EXPECT_EQ(r[i].refusal->field, fields[i]) << keys[i];
    }
    EXPECT_TRUE(sc::compose_placement({}).empty());
}

TEST(LibraryPlace, TheSameRequestGivesTheSameBitsEveryTime) {
    sc::PlacementItem i = item("d");
    i.parent = pose({0.1, -0.2, 0.3}, {0.9238795325112867, 0, 0.3826834323650898, 0});
    i.install_point = pose({0.07, 0.03, -0.01}, {kS, kS, 0, 0});
    sc::SetAdjust a;
    a.pos = {0.001, 0.002, -0.003};
    a.yaw_deg = 3.3;
    a.pitch_deg = -7.7;
    a.roll_deg = 1.1;
    i.adjust = a;
    i.part_to_mount = pose({0, 0.01, 0}, {0.8660254037844386, 0, 0, 0.5});
    const sc::PlacedItem x = one(i);
    const sc::PlacedItem y = one(i);
    ASSERT_TRUE(x.pose);
    ASSERT_TRUE(y.pose);
    EXPECT_EQ(x.pose->pos, y.pose->pos);
    EXPECT_EQ(x.pose->quat, y.pose->quat);
}

}  // namespace
