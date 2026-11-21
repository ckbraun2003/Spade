// test_camera_frame.cpp -- one camera's frame: the image, instance-id and keypoint channels (camera sensor Phase 1, P1.4; design sections 5, 6 and 11a).
//
// WHAT IS PINNED. render_camera_frame traces one ray a pixel through the camera's own lens (pixel centres at i + 0.5, origin at the top-left corner, x right, y down, z
// forward in the camera frame), shades it with the camera shading (P1.3) and writes the channels the row's bit set names (11a): `image` uint8 planar C x H x W (C = 3
// colour, 1 grey), `instance_id` uint16 H x W (the nearest hit's host label: 0 for sky, ground and unlabelled; two labels with one id are one object), `keypoints`
// G groups x P points x (u, v, state) float32 plus the group's label uint16 (state 0 absent or off-image with u, v written 0; 1 in frame but occluded; 2 visible; the
// occlusion is a DEPTH TEST along the point's own ray, never an id lookup, with the self-hit rule: a ray reaching the point's own surface counts as visible). (u, v)
// is the lens's own projection, a continuous position in pixels. A bit outside the known mask, an empty bit set, a keypoint bit with no groups, or a bad lens is
// refused naming the field.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "render/camera_frame.hpp"
#include "render/camera_lens.hpp"
#include "render/camera_shade.hpp"
#include "render/ray_scene.hpp"
#include "render/scene.hpp"
#include "world/builder.hpp"

namespace rd = spade::render;

namespace {

constexpr uint32_t kW = 16, kH = 12;

[[nodiscard]] rd::LensSpec lens() {
    rd::LensSpec l;
    l.kind = rd::kLensPinhole;
    l.width = kW;
    l.height = kH;
    l.fx = l.fy = 16.0;  // half a field of view of 45 degrees across
    l.cx = 8.0;
    l.cy = 6.0;
    return l;
}

// A quad in the plane z = `z`, |x| <= half, |y| <= half, facing -z (toward a camera at the origin looking +z), one item with the given label name.
void add_wall(rd::RenderScene& scene, float z, float half, const std::string& label) {
    rd::MeshData m;
    m.positions = {{-half, -half, z}, {half, -half, z}, {half, half, z}, {-half, half, z}};
    m.normals = {{0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}};
    m.indices = {0, 2, 1, 0, 3, 2};
    m.submesh_first_index = {0};
    m.submesh_index_count = {6};
    m.submesh_material = {0};
    const uint32_t mesh = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(m);
    rd::DrawItem d;
    d.mesh_index = mesh;
    if (!label.empty()) {
        scene.labels.push_back(label);
        d.object_label = static_cast<uint32_t>(scene.labels.size() - 1);
    }
    scene.statics.push_back(d);
}

struct Fixture {
    rd::RenderScene scene;
    Fixture() {
        scene.materials.resize(1);
        scene.materials[0].base_color = glm::vec4(0.5f, 0.25f, 0.125f, 1.0f);
        scene.lighting.sun_direction = glm::vec3(0.0f, 0.0f, 1.0f);
        scene.lighting.sun_color = glm::vec3(1.0f);
        scene.lighting.sun_intensity = 0.6f;
        scene.lighting.ambient_color = glm::vec3(0.1f);
    }
    [[nodiscard]] rd::RayScene rays() const {
        spade::Result<rd::RayScene> r = rd::RayScene::build(scene);
        EXPECT_TRUE(r.has_value());
        return r ? std::move(*r) : rd::RayScene{};
    }
};

[[nodiscard]] rd::CameraFrameDesc desc(uint32_t channels) {
    rd::CameraFrameDesc d;
    d.lens = lens();
    d.position = glm::dvec3(0.0);
    d.camera_to_world = glm::dmat3(1.0);
    d.channels = channels;
    return d;
}

}  // namespace

// The three channels' shapes: image planar C x H x W (3, or 1 for grey), instance id H x W, the keypoints' two arrays; a channel not asked for is empty. Each image
// byte is the camera shading of the ray through that pixel's centre.
TEST(CameraFrame, TheChannelsHaveTheirShapesAndTheImageIsTheShadedRay) {
    Fixture f;
    add_wall(f.scene, 5.0f, 1.0f, "gate");
    const rd::RayScene rays = f.rays();

    rd::CameraFrameDesc d = desc(rd::kChannelImage | rd::kChannelInstanceId);
    d.label_ids = {7};
    auto frame = rd::render_camera_frame(f.scene, rays, d);
    ASSERT_TRUE(frame.has_value()) << (frame ? "" : frame.error().context);
    EXPECT_EQ(frame->image.size(), 3u * kW * kH);
    EXPECT_EQ(frame->instance_id.size(), static_cast<std::size_t>(kW) * kH);
    EXPECT_TRUE(frame->keypoint_uv.empty());
    EXPECT_TRUE(frame->keypoint_labels.empty());

    // pixel (i, j) = (7, 5) is on the wall; pixel (0, 0) is the sky: both are the shaded ray, encoded
    for (const std::pair<uint32_t, uint32_t>& p : {std::pair<uint32_t, uint32_t>{7, 5}, std::pair<uint32_t, uint32_t>{0, 0}}) {
        const glm::dvec3 cam = rd::lens_ray(d.lens, p.first + 0.5, p.second + 0.5);
        rd::Ray r;
        r.origin = d.position;
        r.direction = glm::normalize(d.camera_to_world * cam);
        uint8_t want[3] = {};
        rd::encode_pixel(rd::shade_ray(f.scene, rays, r, d.shading), d.shading, want);
        for (uint32_t c = 0; c < 3; ++c) EXPECT_EQ(frame->image[(c * kH + p.second) * kW + p.first], want[c]) << "pixel " << p.first << "," << p.second << " channel " << c;
    }

    d.shading.grey = true;
    d.channels = rd::kChannelImage;
    auto grey = rd::render_camera_frame(f.scene, rays, d);
    ASSERT_TRUE(grey.has_value());
    EXPECT_EQ(grey->image.size(), static_cast<std::size_t>(kW) * kH) << "grey is one channel";
    EXPECT_TRUE(grey->instance_id.empty());
}

// The instance id is the nearest hit's host label; sky, ground and unlabelled are 0; two labels with one id are one object; a nearer unlabelled surface hides the
// label behind it.
TEST(CameraFrame, TheInstanceIdIsTheNearestHitsHostLabel) {
    Fixture f;
    add_wall(f.scene, 5.0f, 1.0f, "gate_a");   // label index 0 -> id 7
    add_wall(f.scene, 8.0f, 4.0f, "gate_b");   // label index 1 -> id 7 (one object, behind and around the first)
    add_wall(f.scene, 2.0f, 0.2f, "");         // an unlabelled small wall in front of everything at the centre
    const rd::RayScene rays = f.rays();

    rd::CameraFrameDesc d = desc(rd::kChannelInstanceId);
    d.label_ids = {7, 7};
    auto frame = rd::render_camera_frame(f.scene, rays, d);
    ASSERT_TRUE(frame.has_value()) << (frame ? "" : frame.error().context);
    auto at = [&](uint32_t i, uint32_t j) { return frame->instance_id[j * kW + i]; };
    EXPECT_EQ(at(8, 6), 0u) << "the unlabelled wall is nearest at the centre: 0, not the gate behind it";
    EXPECT_EQ(at(5, 6), 7u) << "gate_a, off the small wall";
    EXPECT_EQ(at(0, 0), 7u) << "gate_b around the first (one id, one object)";
    d.label_ids = {7, 9};
    auto two = rd::render_camera_frame(f.scene, rays, d);
    ASSERT_TRUE(two.has_value());
    EXPECT_EQ(two->instance_id[6 * kW + 5], 7u);
    EXPECT_EQ(two->instance_id[0], 9u) << "distinct labels keep distinct ids";

    Fixture sky;
    add_wall(sky.scene, 5.0f, 0.5f, "gate");
    const rd::RayScene sky_rays = sky.rays();
    d.label_ids = {3};
    auto small = rd::render_camera_frame(sky.scene, sky_rays, d);
    ASSERT_TRUE(small.has_value());
    EXPECT_EQ(small->instance_id[0], 0u) << "sky is 0";
}

// Keypoints: state 2 visible (in frame, nothing nearer), 1 in frame but occluded (u, v written), 0 absent (behind the camera or off the image: u, v written 0). The
// self-hit rule: a point ON the surface a ray reaches counts as visible. Order and slots are the declared order; the group's label is written even out of view.
TEST(CameraFrame, KeypointsHaveStateZeroOneOrTwoByADepthTestAlongTheirOwnRay) {
    Fixture f;
    add_wall(f.scene, 5.0f, 1.0f, "gate");
    const rd::RayScene rays = f.rays();

    rd::CameraFrameDesc d = desc(rd::kChannelKeypoints);
    d.keypoint_groups = {
        {11, {{0.0, 0.0, 2.5}, {0.0, 0.0, 8.0}, {0.5, 0.5, 5.0}, {0.0, 0.0, -1.0}}},
        {12, {{100.0, 0.0, 5.0}, {0.0, 0.0, 3.0}, {0.0, 0.0, 3.0}, {0.0, 0.0, 3.0}}},
    };
    auto frame = rd::render_camera_frame(f.scene, rays, d);
    ASSERT_TRUE(frame.has_value()) << (frame ? "" : frame.error().context);
    ASSERT_EQ(frame->keypoint_uv.size(), 2u * 4u * 3u);
    ASSERT_EQ(frame->keypoint_labels.size(), 2u);
    EXPECT_TRUE(frame->image.empty());
    auto uv = [&](uint32_t g, uint32_t p, uint32_t k) { return frame->keypoint_uv[((g * 4 + p) * 3) + k]; };

    EXPECT_NEAR(uv(0, 0, 0), 8.0f, 1e-4f);
    EXPECT_NEAR(uv(0, 0, 1), 6.0f, 1e-4f);
    EXPECT_EQ(uv(0, 0, 2), 2.0f) << "in the air in front of the gate: visible";
    EXPECT_NEAR(uv(0, 1, 0), 8.0f, 1e-4f);
    EXPECT_EQ(uv(0, 1, 2), 1.0f) << "behind the gate: occluded, u and v written";
    EXPECT_NEAR(uv(0, 2, 0), 8.0f + 16.0f * 0.5f / 5.0f, 1e-4f);
    EXPECT_NEAR(uv(0, 2, 1), 6.0f + 16.0f * 0.5f / 5.0f, 1e-4f);
    EXPECT_EQ(uv(0, 2, 2), 2.0f) << "on the gate's own surface: the self-hit rule, visible";
    EXPECT_EQ(uv(0, 3, 0), 0.0f);
    EXPECT_EQ(uv(0, 3, 1), 0.0f);
    EXPECT_EQ(uv(0, 3, 2), 0.0f) << "behind the camera: absent, u and v zeroed";
    EXPECT_EQ(uv(1, 0, 0), 0.0f);
    EXPECT_EQ(uv(1, 0, 1), 0.0f);
    EXPECT_EQ(uv(1, 0, 2), 0.0f) << "off the image: absent, zeroed";
    EXPECT_EQ(frame->keypoint_labels[0], 11u);
    EXPECT_EQ(frame->keypoint_labels[1], 12u) << "a group's label is written even when the group is out of view";
}

// The pixel convention: a point on the ray through pixel (3, 2)'s centre projects to (3.5, 2.5); the camera's pose (a position and a camera-to-world rotation) moves the
// keypoints with the image.
TEST(CameraFrame, TheKeypointIsTheLensProjectionInPixelsAndFollowsThePose) {
    Fixture f;
    const rd::RayScene rays = f.rays();
    rd::CameraFrameDesc d = desc(rd::kChannelKeypoints);
    const glm::dvec3 ray = rd::lens_ray(d.lens, 3.5, 2.5);
    d.keypoint_groups = {{1, {ray * 4.0, {1.0, 0.0, 2.5}, {2.5, 0.0, 0.0}, {0.0, 0.0, 2.5}}}};
    auto a = rd::render_camera_frame(f.scene, rays, d);
    ASSERT_TRUE(a.has_value()) << (a ? "" : a.error().context);
    EXPECT_NEAR(a->keypoint_uv[0], 3.5f, 1e-4f);
    EXPECT_NEAR(a->keypoint_uv[1], 2.5f, 1e-4f);
    EXPECT_EQ(a->keypoint_uv[2], 2.0f);
    EXPECT_EQ(a->keypoint_uv[3 * 1 + 2], 2.0f);
    EXPECT_NEAR(a->keypoint_uv[3 * 1 + 0], 8.0f + 16.0f * 1.0f / 2.5f, 1e-4f);
    EXPECT_EQ(a->keypoint_uv[3 * 2 + 2], 0.0f) << "a point to the side at z = 0 is not in front of the image plane";

    // the camera at (1, 0, 0): the point (1, 0, 2.5) is on the axis
    d.position = glm::dvec3(1.0, 0.0, 0.0);
    auto b = rd::render_camera_frame(f.scene, rays, d);
    ASSERT_TRUE(b.has_value());
    EXPECT_NEAR(b->keypoint_uv[3 * 1 + 0], 8.0f, 1e-4f);
    EXPECT_NEAR(b->keypoint_uv[3 * 1 + 1], 6.0f, 1e-4f);

    // yawed 90 degrees about +y: camera x -> world -z, camera z (forward) -> world +x; the point (2.5, 0, 0) is on the axis
    d.position = glm::dvec3(0.0);
    d.camera_to_world = glm::dmat3(glm::dvec3(0.0, 0.0, -1.0), glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0));
    auto c = rd::render_camera_frame(f.scene, rays, d);
    ASSERT_TRUE(c.has_value());
    EXPECT_NEAR(c->keypoint_uv[3 * 2 + 0], 8.0f, 1e-4f);
    EXPECT_NEAR(c->keypoint_uv[3 * 2 + 1], 6.0f, 1e-4f);
    EXPECT_EQ(c->keypoint_uv[3 * 2 + 2], 2.0f);
}

// Refusals name the field: a bit outside the known mask (depth is reserved), an empty bit set, keypoints with no groups, a ragged group, a bad lens, a label table
// shorter than the scene's labels.
TEST(CameraFrame, ABadRequestIsRefusedNamingTheField) {
    Fixture f;
    add_wall(f.scene, 5.0f, 1.0f, "gate");
    const rd::RayScene rays = f.rays();
    auto issue = [&](const rd::CameraFrameDesc& d) {
        auto r = rd::render_camera_frame(f.scene, rays, d);
        return r ? std::string() : r.error().context;
    };
    rd::CameraFrameDesc d = desc(rd::kChannelImage);
    EXPECT_EQ(issue(d), "");
    d.channels = rd::kChannelImage | 8u;
    EXPECT_NE(issue(d).find("channels"), std::string::npos) << "depth (bit 3) is reserved and refused in v1";
    d.channels = 0;
    EXPECT_NE(issue(d).find("channels"), std::string::npos) << "an active camera with no channel would write nothing";
    d.channels = rd::kChannelKeypoints;
    EXPECT_NE(issue(d).find("keypoint_groups"), std::string::npos);
    d.keypoint_groups = {{1, {{0.0, 0.0, 2.0}}}, {2, {{0.0, 0.0, 2.0}, {0.0, 0.0, 3.0}}}};
    EXPECT_NE(issue(d).find("keypoint_groups"), std::string::npos) << "every group has the same number of points";
    d.keypoint_groups.clear();
    d.channels = rd::kChannelInstanceId;
    d.label_ids = {};
    EXPECT_NE(issue(d).find("label_ids"), std::string::npos) << "the scene has a label the table does not cover";
    d.channels = rd::kChannelImage;
    d.lens.fx = 0.0;
    EXPECT_NE(issue(d).find("fx"), std::string::npos) << "the lens's own refusal";
}

// An SDF hit's instance id is the host label its LEAF carries (node_labels, resolved by sdf_hit_label), read directly: the same ball (label 7) with a box (label 8)
// cut into its +x side seen from +x with the camera at (3, 0, 0): the middle pixel looks down the notch at the cut face (8), a pixel above it sees the ball (7), a pixel off
// the ball is 0. The host's label table is not consulted for a labelled leaf.
TEST(CameraFrame, ASdfHitsInstanceIdIsItsLeafsHostLabel) {
    spade::WorldBuilder b;
    b.name("camera-frame-label-test").capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    b.sphere(1.0f).label_for_last_node(7);
    b.box(glm::vec3(0.5f, 0.3f, 0.3f), spade::SdfPose{.position = glm::vec3(0.9f, 0.0f, 0.0f)}).label_for_last_node(8);
    b.subtract();
    const spade::Result<spade::WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << (world ? "" : world.error().context);
    spade::Result<rd::RenderScene> scene = rd::scene_from_world(*world, {});
    ASSERT_TRUE(scene.has_value()) << (scene ? "" : scene.error().context);
    spade::Result<rd::RayScene> rays = rd::RayScene::build(*scene);
    ASSERT_TRUE(rays.has_value());

    rd::CameraFrameDesc d;
    d.lens.kind = rd::kLensPinhole;
    d.lens.width = d.lens.height = 11;
    d.lens.fx = d.lens.fy = 15.0;
    d.lens.cx = d.lens.cy = 5.5;
    d.position = glm::dvec3(3.0, 0.0, 0.0);
    d.camera_to_world = glm::dmat3(glm::dvec3(0.0, 0.0, -1.0), glm::dvec3(0.0, -1.0, 0.0), glm::dvec3(-1.0, 0.0, 0.0));  // looking down -x, y up
    d.channels = rd::kChannelInstanceId;
    d.label_ids.assign(scene->labels.size(), 99);
    auto frame = rd::render_camera_frame(*scene, *rays, d);
    ASSERT_TRUE(frame.has_value()) << (frame ? "" : frame.error().context);
    auto at = [&](uint32_t i, uint32_t j) { return frame->instance_id[j * 11u + i]; };
    EXPECT_EQ(at(5, 5), 8u) << "down the notch: the cut face is the box's";
    EXPECT_EQ(at(5, 1), 7u) << "above the notch: the ball's surface";
    EXPECT_EQ(at(0, 0), 0u) << "the corner pixel, 0.47 off the axis, is past the ball's 0.35 (the ball seen from 3 m: tan of 19.5 degrees)";
}
