#include "render/camera_frame.hpp"

#include <cmath>
#include <string>

namespace spade::render {

namespace {

[[nodiscard]] std::unexpected<Error> refuse(const std::string& why) {
    return std::unexpected(Error{Code::invalid_argument, "render_camera_frame: " + why});
}

// The state of one keypoint, 0 absent, 1 occluded, 2 visible, and its pixel position (0, 0 when absent).
[[nodiscard]] float keypoint_state(const RayScene& rays, const CameraFrameDesc& desc, const glm::dvec3& point, float* u, float* v) {
    *u = 0.0f;
    *v = 0.0f;
    const glm::dvec3 offset = point - desc.position;
    const glm::dvec3 in_camera = glm::transpose(desc.camera_to_world) * offset;  // the rotation's inverse
    const std::optional<glm::dvec2> pixel = lens_project(desc.lens, in_camera);
    if (!pixel) return 0.0f;
    if (!(pixel->x >= 0.0 && pixel->x < static_cast<double>(desc.lens.width) && pixel->y >= 0.0 && pixel->y < static_cast<double>(desc.lens.height))) return 0.0f;
    *u = static_cast<float>(pixel->x);
    *v = static_cast<float>(pixel->y);
    const double range = glm::length(offset);
    const double eps = 2.0e-3 + 1.0e-4 * range;  // occlusion_epsilon: a corner on the aperture's own surface is not its own occluder
    if (!(range > eps)) return 2.0f;
    Ray ray;
    ray.origin = desc.position;
    ray.direction = offset / range;
    ray.t_max = range - eps;
    return rays.trace(ray).kind == RayHitKind::none ? 2.0f : 1.0f;
}

}  // namespace

Result<CameraFrame> render_camera_frame(const RenderScene& scene, const RayScene& rays, const CameraFrameDesc& desc) {
    if ((desc.channels & ~kChannelKnown) != 0u) return refuse("channels has a bit outside the known mask 0x7 (depth and above are reserved in v1)");
    if (desc.channels == 0u) return refuse("channels is 0: an active camera would write nothing");
    if (const std::string why = lens_issue(desc.lens); !why.empty()) return refuse("lens: " + why);
    if ((desc.channels & kChannelInstanceId) != 0u && desc.label_ids.size() < scene.labels.size()) {
        return refuse("label_ids has " + std::to_string(desc.label_ids.size()) + " entries and the scene has " + std::to_string(scene.labels.size()) + " labels");
    }
    std::size_t points = 0;
    if ((desc.channels & kChannelKeypoints) != 0u) {
        if (desc.keypoint_groups.empty()) return refuse("keypoint_groups is empty and channels asks for keypoints");
        points = desc.keypoint_groups.front().points.size();
        for (const KeypointGroup& g : desc.keypoint_groups) {
            if (g.points.size() != points) return refuse("keypoint_groups: every group has the same number of points");
        }
    }

    const uint32_t w = desc.lens.width, h = desc.lens.height;
    const std::size_t pixels = static_cast<std::size_t>(w) * h;
    const bool want_image = (desc.channels & kChannelImage) != 0u;
    const bool want_ids = (desc.channels & kChannelInstanceId) != 0u;
    CameraFrame out;
    const uint32_t channels = channel_count(desc.shading);
    if (want_image) out.image.assign(pixels * channels, 0u);
    if (want_ids) out.instance_id.assign(pixels, 0u);

    if (want_image || want_ids) {
        for (uint32_t j = 0; j < h; ++j) {
            for (uint32_t i = 0; i < w; ++i) {
                Ray ray;
                ray.origin = desc.position;
                ray.direction = glm::normalize(desc.camera_to_world * lens_ray(desc.lens, i + 0.5, j + 0.5));
                const std::size_t at = static_cast<std::size_t>(j) * w + i;
                if (want_image) {
                    uint8_t bytes[3] = {};
                    encode_pixel(shade_ray(scene, rays, ray, desc.shading), desc.shading, bytes);
                    for (uint32_t c = 0; c < channels; ++c) out.image[c * pixels + at] = bytes[c];
                }
                if (want_ids) {
                    const RayHit hit = rays.trace(ray);
                    if (hit.kind != RayHitKind::none && hit.label != kNoLabel && hit.label < desc.label_ids.size()) out.instance_id[at] = desc.label_ids[hit.label];
                }
            }
        }
    }

    if ((desc.channels & kChannelKeypoints) != 0u) {
        const std::size_t groups = desc.keypoint_groups.size();
        out.keypoint_uv.assign(groups * points * 3u, 0.0f);
        out.keypoint_labels.assign(groups, 0u);
        for (std::size_t g = 0; g < groups; ++g) {
            out.keypoint_labels[g] = desc.keypoint_groups[g].label;
            for (std::size_t p = 0; p < points; ++p) {
                float* slot = &out.keypoint_uv[(g * points + p) * 3u];
                slot[2] = keypoint_state(rays, desc, desc.keypoint_groups[g].points[p], &slot[0], &slot[1]);
            }
        }
    }
    return out;
}

}  // namespace spade::render
