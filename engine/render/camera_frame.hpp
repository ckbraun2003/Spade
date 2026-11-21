#pragma once

// ---------------------------------------------------------------------------
// camera_frame -- one camera's frame on the CPU reference: the image, instance-id and keypoint channels (camera sensor design sections 5, 6 and 11a; Phase 1, P1.4).
//
// A ray a pixel through the camera's own lens (camera_lens.hpp: pixel centres at i + 0.5, origin at the top-left corner, x right, y down, z forward), traced on the
// RayScene and shaded by the camera shading (camera_shade.hpp). The channels written are the ones the row's bit set names (design 11a); the bit values are Core's
// `camera_channel` (engine/sensors/camera.hpp) and are repeated here as the render library's own constants because render does not include sensors/.
//
//   image        uint8, planar C x H x W (C = 3 colour, 1 grey), sRGB, exposed (P1.3).
//   instance_id  uint16, H x W. The NEAREST hit's host label: scene object_label (an index into RenderScene::labels) mapped through `label_ids` (the host's table,
//                one uint16 per scene label); 0 for sky, an unlabelled item or a label the table maps to 0 (the ground is whatever the table maps its plane's label to: 0 unless the host labels it). Two labels with one id are one object. A CSG hit
//                takes the label of its subtree's item (L118a: Core's per-node SDF labels will replace this when core/world-v4 lands).
//   keypoints    G groups x P points x (u, v, state) float32, plus the group's label uint16. u, v are the lens's own projection in pixels (the distorted pixel for a
//                KB4), continuous, in the image's own frame. State 0 = absent (behind the image plane, or off the image): u and v written 0; 1 = in frame but occluded;
//                2 = visible. Occlusion is a DEPTH TEST along the point's own ray, never an id lookup: occluded if a surface is nearer than the point by more than
//                eps = 2 mm + 1e-4 x range; a ray reaching the point's own surface (the point on it) is visible (the self-hit rule). Points are in the order given, groups
//                are stable slots, and a group out of view keeps its slot with its label written. The points are WORLD positions: the host poses the object's local
//                keypoints (same pose as the image, at the capture time).
//
// Every byte of every asked channel is written. A channel not asked for is an empty vector. No libm: + - * / and sqrt only (SR-14).
// ---------------------------------------------------------------------------

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "render/camera_lens.hpp"
#include "render/camera_shade.hpp"
#include "render/ray_scene.hpp"
#include "render/scene.hpp"

namespace spade::render {

// The channel bits (Core's camera_channel, design 11a). Depth (bit 3) and above are reserved and refused in v1.
inline constexpr uint32_t kChannelImage = 1u << 0;
inline constexpr uint32_t kChannelInstanceId = 1u << 1;
inline constexpr uint32_t kChannelKeypoints = 1u << 2;
inline constexpr uint32_t kChannelKnown = kChannelImage | kChannelInstanceId | kChannelKeypoints;

struct KeypointGroup {
    uint16_t label = 0;                // the group's object label (0 = empty slot)
    std::vector<glm::dvec3> points;    // world positions, in the declared order; every group of a frame has the same count
};

struct CameraFrameDesc {
    LensSpec lens;
    glm::dvec3 position{0.0};                // the camera's world position
    glm::dmat3 camera_to_world{1.0};         // columns: the camera's x (right), y (down), z (forward) in the world
    uint32_t channels = 0;                   // kChannel* bits
    CameraShading shading;
    std::vector<uint16_t> label_ids;         // indexed by RenderScene label; needed (and as long as scene.labels) for instance_id
    std::vector<KeypointGroup> keypoint_groups;
};

struct CameraFrame {
    std::vector<uint8_t> image;              // C x H x W
    std::vector<uint16_t> instance_id;       // H x W
    std::vector<float> keypoint_uv;          // G x P x 3: u, v, state
    std::vector<uint16_t> keypoint_labels;   // G
};

// Refuses (invalid_argument, naming the field) a bit outside the known mask, no channel, keypoints with no groups or a ragged group, an instance_id request whose
// label_ids does not cover the scene's labels, and a lens lens_issue() refuses.
[[nodiscard]] Result<CameraFrame> render_camera_frame(const RenderScene& scene, const RayScene& rays, const CameraFrameDesc& desc);

}  // namespace spade::render
