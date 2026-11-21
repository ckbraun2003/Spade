// test_camera_mount.cpp -- a camera's pose from its row and its body, and the row's frame request (camera sensor Phase 1, P1.5; design sections 5, 11a and open item 1).
//
// WHAT IS PINNED. The row's mount_pos is a BODY-frame offset from the centre of mass (metres) and mount_orient is the mount -> body rotation of the CAMERA frame (x right,
// y down, z forward); the camera's pose at capture is the stamp's (CameraStamp::position, ::orientation): position = body position + body orientation * mount_pos and
// camera -> world = body orientation * mount_orient. KAT's frames (answer 1): the body is x forward, y up, z right; at tilt 0 the camera's z is body +x, its x is body +z
// and its y is body -y; a tilt +theta turns the optical axis UP toward body +y (+theta about body +z). camera_mount_from_tilt(theta_deg) is that mount_orient (no libm).
// camera_frame_desc turns a row, its body's pose and the host's channel tables into the frame request; it changes the mount transform only, never the lens.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/camera_frame.hpp"
#include "render/camera_mount.hpp"
#include "sensors/camera.hpp"

namespace rd = spade::render;
namespace sn = spade::sensors;

namespace {

[[nodiscard]] sn::CameraSensorRow row() {
    sn::CameraSensorRow r{};
    r.kind = 1;
    r.rate_divider = 1;
    r.mount_pos = glm::vec3(0.0f);
    r.mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    r.lens_kind = sn::lens_kind::pinhole;
    r.width = 640;
    r.height = 360;
    r.mode = sn::camera_mode::colour;
    r.fx = r.fy = 320.0f;
    r.cx = 320.0f;
    r.cy = 180.0f;
    r.blur_samples = 1;
    r.aa_samples = 1;
    r.exposure_gain = 1.0f;
    r.cull_mask = 0xFFu;
    r.channels = sn::camera_channel::image;
    return r;
}

[[nodiscard]] glm::dmat3 as_matrix(const glm::quat& q) { return glm::dmat3(glm::mat3_cast(q)); }

}  // namespace

// The render library's channel bits are Core's, so the two cannot drift.
static_assert(rd::kChannelImage == sn::camera_channel::image);
static_assert(rd::kChannelInstanceId == sn::camera_channel::instance_id);
static_assert(rd::kChannelKeypoints == sn::camera_channel::keypoints);
static_assert(rd::kChannelKnown == sn::camera_channel::known);

// The pose at capture: the mount offset is in the body frame, the camera's rotation is the body's times the mount's.
TEST(CameraMount, ThePoseIsTheBodyPoseTimesTheMount) {
    sn::CameraSensorRow r = row();
    r.mount_pos = glm::vec3(0.1f, 0.0f, 0.05f);
    const rd::CameraPose still = rd::camera_world_pose(r, glm::dvec3(1.0, 2.0, 3.0), glm::dquat(1.0, 0.0, 0.0, 0.0));
    EXPECT_NEAR(still.position.x, 1.1, 1e-6);
    EXPECT_NEAR(still.position.y, 2.0, 1e-6);
    EXPECT_NEAR(still.position.z, 3.05, 1e-6);

    // the body yawed 90 degrees about +y (its +x goes to world -z): the offset (0.1, 0, 0.05) goes to (0.05, 0, -0.1)
    const glm::dquat yaw = glm::angleAxis(glm::radians(90.0), glm::dvec3(0.0, 1.0, 0.0));
    const rd::CameraPose turned = rd::camera_world_pose(r, glm::dvec3(1.0, 2.0, 3.0), yaw);
    EXPECT_NEAR(turned.position.x, 1.05, 1e-6);
    EXPECT_NEAR(turned.position.y, 2.0, 1e-6);
    EXPECT_NEAR(turned.position.z, 2.9, 1e-6);
    // camera -> world is the body's rotation times the mount's: with an identity mount the camera's z is the body's z, which the yaw turns to world +x
    const glm::dvec3 z = turned.camera_to_world * glm::dvec3(0.0, 0.0, 1.0);
    EXPECT_NEAR(z.x, 1.0, 1e-6);
    EXPECT_NEAR(z.y, 0.0, 1e-6);
    EXPECT_NEAR(z.z, 0.0, 1e-6);
    EXPECT_NEAR(glm::determinant(turned.camera_to_world), 1.0, 1e-9) << "a proper rotation";
}

// KAT's mount: at tilt 0 the camera's z is body +x, its x is body +z, its y is body -y; a tilt +theta about body +z turns the optical axis up toward body +y.
TEST(CameraMount, KatsTiltIsTheMountOrientation) {
    const glm::dmat3 flat = as_matrix(rd::camera_mount_from_tilt(0.0));
    auto near = [](const glm::dvec3& a, const glm::dvec3& b) { return glm::length(a - b) < 1e-6; };
    EXPECT_TRUE(near(flat * glm::dvec3(0.0, 0.0, 1.0), glm::dvec3(1.0, 0.0, 0.0))) << "camera z is body +x";
    EXPECT_TRUE(near(flat * glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0))) << "camera x is body +z";
    EXPECT_TRUE(near(flat * glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0, -1.0, 0.0))) << "camera y is body -y";

    for (double deg : {13.27, 22.0, -13.27, 90.0}) {
        const glm::dmat3 tilted = as_matrix(rd::camera_mount_from_tilt(deg));
        const double c = std::cos(glm::radians(deg)), s = std::sin(glm::radians(deg));
        EXPECT_TRUE(near(tilted * glm::dvec3(0.0, 0.0, 1.0), glm::dvec3(c, s, 0.0))) << "the optical axis turns up toward body +y by " << deg << " degrees";
        EXPECT_TRUE(near(tilted * glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0))) << "camera x stays body +z (the tilt is about body +z)";
    }
}

// The frame request carries the row's lens, mode, exposure, channels and the pose; a point straight ahead of a tilt-0 camera is the principal point, and a tilt of
// +10 degrees puts it BELOW the centre by fy tan(10 degrees); a point to the body's right (+z) is to the right of the centre.
TEST(CameraMount, TheFrameRequestCarriesTheRowAndAPointAheadIsAtThePrincipalPoint) {
    sn::CameraSensorRow r = row();
    r.mount_orient = rd::camera_mount_from_tilt(0.0);
    r.mode = sn::camera_mode::grey;
    r.exposure_gain = 1.25f;
    r.channels = sn::camera_channel::image | sn::camera_channel::keypoints;
    r.lens_kind = sn::lens_kind::kb4;
    r.k1 = 0.01f;
    r.k2 = -0.002f;
    const rd::CameraFrameDesc d = rd::camera_frame_desc(r, glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0));
    EXPECT_EQ(d.lens.kind, rd::kLensKb4);
    EXPECT_EQ(d.lens.width, 640u);
    EXPECT_EQ(d.lens.height, 360u);
    EXPECT_DOUBLE_EQ(d.lens.fx, 320.0);
    EXPECT_DOUBLE_EQ(d.lens.cy, 180.0);
    EXPECT_NEAR(d.lens.k[0], 0.01, 1e-9);
    EXPECT_NEAR(d.lens.k[1], -0.002, 1e-9);
    EXPECT_TRUE(d.shading.grey);
    EXPECT_FLOAT_EQ(d.shading.exposure, 1.25f);
    EXPECT_EQ(d.channels, r.channels);

    // end to end through render_camera_frame: pinhole, ahead of the body (+x), then to its right (+z), at tilt 0 and tilt +10
    r.lens_kind = sn::lens_kind::pinhole;
    r.k1 = r.k2 = 0.0f;
    r.mode = sn::camera_mode::colour;
    rd::RenderScene scene;
    spade::Result<rd::RayScene> rays = rd::RayScene::build(scene);
    ASSERT_TRUE(rays.has_value());
    auto project = [&](double tilt_deg, const glm::dvec3& body_point) {
        r.mount_orient = rd::camera_mount_from_tilt(tilt_deg);
        rd::CameraFrameDesc dd = rd::camera_frame_desc(r, glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0));
        dd.channels = rd::kChannelKeypoints;
        dd.keypoint_groups = {{1, {body_point}}};
        auto f = rd::render_camera_frame(scene, *rays, dd);
        EXPECT_TRUE(f.has_value()) << (f ? "" : f.error().context);
        return f ? glm::dvec3(f->keypoint_uv[0], f->keypoint_uv[1], f->keypoint_uv[2]) : glm::dvec3(0.0);
    };
    const glm::dvec3 ahead = project(0.0, {5.0, 0.0, 0.0});
    EXPECT_NEAR(ahead.x, 320.0, 1e-3);
    EXPECT_NEAR(ahead.y, 180.0, 1e-3);
    EXPECT_EQ(ahead.z, 2.0);
    const glm::dvec3 right = project(0.0, {5.0, 0.0, 1.0});
    EXPECT_NEAR(right.x, 320.0 + 320.0 * 1.0 / 5.0, 1e-3) << "the body's +z is the image's right";
    EXPECT_NEAR(right.y, 180.0, 1e-3);
    const glm::dvec3 up_tilt = project(10.0, {5.0, 0.0, 0.0});
    EXPECT_NEAR(up_tilt.x, 320.0, 1e-3);
    EXPECT_NEAR(up_tilt.y, 180.0 + 320.0 * std::tan(glm::radians(10.0)), 1e-2) << "the axis is tilted up, so the point that was ahead is below the centre";
}
