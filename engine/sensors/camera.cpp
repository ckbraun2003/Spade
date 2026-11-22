#include "sensors/camera.hpp"

namespace spade::sensors {

bool camera_advance(CameraSensorRow& row) noexcept {
    row.phase += 1u;
    if (row.phase < row.rate_divider) return false;  // a divider of 0 or 1 is every substep
    row.phase = 0u;
    return true;
}

// RED stub: the pass does nothing yet.
void schedule_cameras(std::span<const BodyState>, std::span<CameraSensorRow>, std::span<CameraFrameDue>, uint64_t) noexcept {}

namespace {

bool finite(float v) noexcept { return std::isfinite(v); }

std::string bad(const char* field, const char* why) { return std::string(field) + ": " + why; }

}  // namespace

std::string camera_row_issue(const CameraSensorRow& row, uint32_t keypoint_groups) {
    if (row.kind == sensor_kind::none) return {};  // inert: a freed or reserved slot
    if (row.kind != sensor_kind::camera) return bad("kind", "not a camera row");
    if (row.rate_divider == 0u) return bad("rate_divider", "must be at least 1");
    if (row.phase >= row.rate_divider) return bad("phase", "must be below rate_divider");
    if (row.lens_kind >= lens_kind::count) return bad("lens_kind", "an unknown lens kind (0 pinhole, 1 KB4)");
    if (row.width == 0u) return bad("width", "must be above 0");
    if (row.height == 0u) return bad("height", "must be above 0");
    if (row.mode >= camera_mode::count) return bad("mode", "an unknown mode (0 colour, 1 grey)");
    if (!finite(row.fx) || !(row.fx > 0.0f)) return bad("fx", "must be finite and above 0");
    if (!finite(row.fy) || !(row.fy > 0.0f)) return bad("fy", "must be finite and above 0");
    if (!finite(row.cx)) return bad("cx", "must be finite");
    if (!finite(row.cy)) return bad("cy", "must be finite");
    const float k[4] = {row.k1, row.k2, row.k3, row.k4};
    const char* const kname[4] = {"k1", "k2", "k3", "k4"};
    for (int i = 0; i < 4; ++i) {
        if (!finite(k[i])) return bad(kname[i], "must be finite");
        if (row.lens_kind == lens_kind::pinhole && k[i] != 0.0f) return bad(kname[i], "must be 0 for a pinhole");
    }
    if (!finite(row.shutter_fraction) || row.shutter_fraction < 0.0f || row.shutter_fraction > 1.0f) {
        return bad("shutter_fraction", "must be in [0, 1]");
    }
    if (row.blur_samples == 0u) return bad("blur_samples", "must be at least 1 (1 is off)");
    if (row.aa_samples == 0u) return bad("aa_samples", "must be at least 1 (1 is off)");
    if (!finite(row.exposure_gain) || !(row.exposure_gain > 0.0f)) return bad("exposure_gain", "must be finite and above 0");
    if (row.cull_mask > 0xFFu) return bad("cull_mask", "is an 8-bit mask");
    if (row.channels == 0u) return bad("channels", "an active camera needs at least one channel (image, instance_id, keypoints)");
    if ((row.channels & ~camera_channel::known) != 0u) {
        return bad("channels", "has a bit outside the known set (image, instance_id, keypoints); depth and above are reserved");
    }
    if ((row.channels & camera_channel::keypoints) != 0u && keypoint_groups == 0u) {
        return bad("channels", "asks for keypoints, but the world declares no keypoint_groups (capacities.keypoint_groups is 0)");
    }
    if (!finite(row.mount_pos.x) || !finite(row.mount_pos.y) || !finite(row.mount_pos.z)) return bad("mount_pos", "must be finite");
    const float q2 = row.mount_orient.w * row.mount_orient.w + row.mount_orient.x * row.mount_orient.x + row.mount_orient.y * row.mount_orient.y +
                     row.mount_orient.z * row.mount_orient.z;
    if (!finite(q2) || std::abs(q2 - 1.0f) > 1e-3f) return bad("mount_orient", "must be a unit quaternion");
    if (row._p0 != 0u) return bad("_p0", "must stay 0");
    if (row._reserved0 != 0u) return bad("_reserved0", "reserved for versioned growth, must stay 0");
    if (row._reserved1 != 0u) return bad("_reserved1", "reserved for versioned growth, must stay 0");
    return {};
}

}  // namespace spade::sensors
