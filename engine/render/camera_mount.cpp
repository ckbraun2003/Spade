#include "render/camera_mount.hpp"

#include "core/detail/math_ops_f64.hpp"  // spade::math::sincos_deg_arith: sin and cos in degrees from + - * / only

namespace spade::render {

CameraPose camera_world_pose(const sensors::CameraSensorRow& row, const glm::dvec3& body_position, const glm::dquat& body_orientation) {
    const glm::dquat mount(static_cast<double>(row.mount_orient.w), static_cast<double>(row.mount_orient.x), static_cast<double>(row.mount_orient.y),
                           static_cast<double>(row.mount_orient.z));
    CameraPose pose;
    pose.position = body_position + body_orientation * glm::dvec3(row.mount_pos);
    pose.camera_to_world = glm::mat3_cast(glm::normalize(body_orientation) * glm::normalize(mount));
    return pose;
}

glm::quat camera_mount_from_tilt(double tilt_deg) {
    double s = 0.0, c = 1.0;
    math::sincos_deg_arith(tilt_deg, s, c);
    // columns, in the body frame: camera x = body +z, camera y = body -y, camera z = body +x; then Rz(tilt) on the left
    const glm::dmat3 flat(glm::dvec3(0.0, 0.0, 1.0), glm::dvec3(0.0, -1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0));
    const glm::dmat3 about_z(glm::dvec3(c, s, 0.0), glm::dvec3(-s, c, 0.0), glm::dvec3(0.0, 0.0, 1.0));  // columns
    return glm::quat(glm::normalize(glm::quat_cast(about_z * flat)));
}

CameraFrameDesc camera_frame_desc(const sensors::CameraSensorRow& row, const glm::dvec3& body_position, const glm::dquat& body_orientation) {
    const CameraPose pose = camera_world_pose(row, body_position, body_orientation);
    CameraFrameDesc d;
    d.lens.kind = row.lens_kind;
    d.lens.width = row.width;
    d.lens.height = row.height;
    d.lens.fx = static_cast<double>(row.fx);
    d.lens.fy = static_cast<double>(row.fy);
    d.lens.cx = static_cast<double>(row.cx);
    d.lens.cy = static_cast<double>(row.cy);
    d.lens.k[0] = static_cast<double>(row.k1);
    d.lens.k[1] = static_cast<double>(row.k2);
    d.lens.k[2] = static_cast<double>(row.k3);
    d.lens.k[3] = static_cast<double>(row.k4);
    d.position = pose.position;
    d.camera_to_world = pose.camera_to_world;
    d.channels = row.channels;
    d.shading.exposure = row.exposure_gain;
    d.shading.grey = row.mode == sensors::camera_mode::grey;
    return d;
}

}  // namespace spade::render
