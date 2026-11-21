#pragma once

// ---------------------------------------------------------------------------
// camera_mount -- a camera's pose from its row and its body, KAT's tilt as the mount, and the row's frame request (camera sensor design sections 5 and 11a, open item 1;
// Phase 1, P1.5).
//
// THE POSE. The row's mount_pos is a BODY-frame offset from the centre of mass (metres) and mount_orient the mount -> body rotation of the CAMERA frame (x right, y down,
// z forward), the IMU's convention. The camera's pose at capture is what the stamp records (CameraStamp::position, ::orientation, camera -> world):
//   position = body position + body orientation * mount_pos;   camera -> world = body orientation * mount_orient.
// The body position is the centre of mass's, as the row's offset is from it. TIME: the pose is the body's at the capture tick (Core's stamp); with motion blur (Phase 6) it is
// the exposure midpoint's, and this function takes whichever body pose the host passes.
//
// KAT'S MOUNT (answer 1). The KAT body is x forward, y up, z right. At tilt 0 the camera's z is body +x, its x is body +z and its y is body -y; a tilt +theta turns the optical
// axis UP toward body +y, that is +theta about body +z. camera_mount_from_tilt(theta_deg) is that mount_orient. It uses the engine's own sin and cos (no libm, SR-14).
// The tilt changes the mount transform only; the lens and its ray table are untouched.
// ---------------------------------------------------------------------------

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/camera_frame.hpp"
#include "sensors/camera.hpp"

namespace spade::render {

struct CameraPose {
    glm::dvec3 position{0.0};         // world
    glm::dmat3 camera_to_world{1.0};  // columns: the camera's x (right), y (down), z (forward) in the world
};

[[nodiscard]] CameraPose camera_world_pose(const sensors::CameraSensorRow& row, const glm::dvec3& body_position, const glm::dquat& body_orientation);

// KAT's mount: the camera's z at body +x, x at body +z, y at body -y, then turned +tilt_deg about body +z.
[[nodiscard]] glm::quat camera_mount_from_tilt(double tilt_deg);

// The frame request of one row at one body pose: the row's lens, mode (grey or colour), exposure gain, channel bits and the pose. The host adds the label table and the
// keypoint groups. Precondition: the row is an active camera that camera_row_issue() accepts.
[[nodiscard]] CameraFrameDesc camera_frame_desc(const sensors::CameraSensorRow& row, const glm::dvec3& body_position, const glm::dquat& body_orientation);

}  // namespace spade::render
