// test_camera_golden.cpp -- the CPU reference camera's goldens (camera sensor Phase 1, P1.6; design sections 8 and 9: "CPU goldens on two platforms").
//
// WHAT IS PINNED. One small fixed scene (two lit quads, one slanted, with host labels, and a sky) seen by a 32 x 24 pinhole colour camera through render_camera_frame: the
// image bytes, the instance-id plane and the keypoints' (u, v, state) and labels are each hashed (FNV-1a 64 over their bytes: the image and ids little-endian as stored,
// the keypoint floats by their bits) and the hashes are the goldens. The CPU reference is the oracle the GPU camera is banded against and a GPU frame is never a golden (L4),
// so these hashes must be the SAME on every platform: MSVC and the Linux gcc leg build the same constants (no libm, + - * / and sqrt only, SR-14; the scene's lighting is
// float and the pinned lens is double). A few pixels are also checked against the scene's own shading functions, independently of the hash, so a wrong golden cannot be
// pinned by accident: the sky pixel, a wall pixel and the instance ids.
//
// To re-pin after a deliberate change, run the test: a mismatch prints the actual hashes (in hex).

#include <gtest/gtest.h>

#include <bit>
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

namespace rd = spade::render;

namespace {

constexpr uint32_t kW = 32, kH = 24;

// FNV-1a 64 over a byte range
[[nodiscard]] uint64_t fnv(uint64_t h, const uint8_t* p, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 1099511628211ull;
    return h;
}
constexpr uint64_t kBasis = 14695981039346656037ull;

[[nodiscard]] std::string hex(uint64_t v) {
    static const char* d = "0123456789abcdef";
    std::string s = "0x";
    for (int i = 60; i >= 0; i -= 4) s += d[(v >> i) & 0xF];
    return s;
}

void add_quad(rd::RenderScene& scene, const glm::vec3 (&p)[4], const glm::vec3& n, const std::string& label, uint32_t material) {
    rd::MeshData m;
    m.positions = {p[0], p[1], p[2], p[3]};
    m.normals = {n, n, n, n};
    m.indices = {0, 1, 2, 0, 2, 3};
    m.submesh_first_index = {0};
    m.submesh_index_count = {6};
    m.submesh_material = {material};
    const uint32_t mesh = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(m);
    rd::DrawItem d;
    d.mesh_index = mesh;
    scene.labels.push_back(label);
    d.object_label = static_cast<uint32_t>(scene.labels.size() - 1);
    scene.statics.push_back(d);
}

struct Scene {
    rd::RenderScene scene;
    Scene() {
        scene.materials.resize(2);
        scene.materials[0].base_color = glm::vec4(0.5f, 0.25f, 0.125f, 1.0f);
        scene.materials[1].base_color = glm::vec4(0.125f, 0.5f, 0.375f, 1.0f);
        scene.lighting.sun_direction = glm::normalize(glm::vec3(0.3f, 0.4f, -0.8f));
        scene.lighting.sun_color = glm::vec3(1.0f);
        scene.lighting.sun_intensity = 0.7f;
        scene.lighting.ambient_color = glm::vec3(0.12f);
        // wall A: z = 5, facing the camera
        const glm::vec3 a[4] = {{-1.5f, -1.5f, 5.0f}, {1.5f, -1.5f, 5.0f}, {1.5f, 1.5f, 5.0f}, {-1.5f, 1.5f, 5.0f}};
        add_quad(scene, a, glm::vec3(0.0f, 0.0f, -1.0f), "a", 0);
        // wall B: slanted, z from 3.0 at x = -3 to 4.5 at x = -0.4, nearer than A where they overlap in the image
        const glm::vec3 b[4] = {{-3.0f, -1.0f, 3.0f}, {-0.4f, -1.0f, 4.5f}, {-0.4f, 1.0f, 4.5f}, {-3.0f, 1.0f, 3.0f}};
        add_quad(scene, b, glm::normalize(glm::vec3(1.5f, 0.0f, -2.6f)), "b", 1);
    }
};

[[nodiscard]] rd::CameraFrameDesc desc() {
    rd::CameraFrameDesc d;
    d.lens.kind = rd::kLensPinhole;
    d.lens.width = kW;
    d.lens.height = kH;
    d.lens.fx = d.lens.fy = 32.0;
    d.lens.cx = 16.0;
    d.lens.cy = 12.0;
    d.position = glm::dvec3(0.0);
    d.camera_to_world = glm::dmat3(1.0);
    d.channels = rd::kChannelImage | rd::kChannelInstanceId | rd::kChannelKeypoints;
    d.label_ids = {3, 9};
    d.keypoint_groups = {{21, {{0.0, 0.0, 5.0}, {-1.0, 0.0, 3.5}, {-1.5, 0.0, 4.5}, {2.0, 3.0, 6.0}}}};
    return d;
}

}  // namespace

// The goldens. The hashes are the CPU reference's frame; they must be identical on MSVC and on the gcc leg.
TEST(CameraGolden, TheReferenceFrameHashesAreFixedOnEveryPlatform) {
    Scene s;
    spade::Result<rd::RayScene> rays = rd::RayScene::build(s.scene);
    ASSERT_TRUE(rays.has_value());
    const rd::CameraFrameDesc d = desc();
    auto frame = rd::render_camera_frame(s.scene, *rays, d);
    ASSERT_TRUE(frame.has_value()) << (frame ? "" : frame.error().context);
    ASSERT_EQ(frame->image.size(), 3u * kW * kH);

    const uint64_t image = fnv(kBasis, frame->image.data(), frame->image.size());
    uint64_t ids = kBasis;
    for (uint16_t v : frame->instance_id) {
        const uint8_t b[2] = {static_cast<uint8_t>(v & 0xFFu), static_cast<uint8_t>(v >> 8)};
        ids = fnv(ids, b, 2);
    }
    uint64_t kp = kBasis;
    for (float f : frame->keypoint_uv) {
        const uint32_t bits = std::bit_cast<uint32_t>(f);
        const uint8_t b[4] = {static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 8), static_cast<uint8_t>(bits >> 16), static_cast<uint8_t>(bits >> 24)};
        kp = fnv(kp, b, 4);
    }
    for (uint16_t v : frame->keypoint_labels) {
        const uint8_t b[2] = {static_cast<uint8_t>(v & 0xFFu), static_cast<uint8_t>(v >> 8)};
        kp = fnv(kp, b, 2);
    }
    EXPECT_EQ(image, 0ull) << "image hash is " << hex(image);
    EXPECT_EQ(ids, 0ull) << "instance-id hash is " << hex(ids);
    EXPECT_EQ(kp, 0ull) << "keypoint hash is " << hex(kp);
}

// Independent of the hashes: the sky pixel and a wall pixel are the scene's own shading, the ids are the labels, the keypoints are where the geometry puts them.
TEST(CameraGolden, SomePixelsAreTheScenesOwnShadingIndependentlyOfTheHash) {
    Scene s;
    spade::Result<rd::RayScene> rays = rd::RayScene::build(s.scene);
    ASSERT_TRUE(rays.has_value());
    const rd::CameraFrameDesc d = desc();
    auto frame = rd::render_camera_frame(s.scene, *rays, d);
    ASSERT_TRUE(frame.has_value());
    auto pixel = [&](uint32_t i, uint32_t j, uint32_t c) { return frame->image[(c * kH + j) * kW + i]; };

    // the sky at pixel (31, 0): top right, off both walls
    const glm::dvec3 cam = rd::lens_ray(d.lens, 31.5, 0.5);
    const glm::vec3 sky = rd::sky_gradient_color(s.scene.lighting, glm::vec3(glm::normalize(cam)));
    uint8_t want[3] = {};
    rd::encode_pixel(sky, d.shading, want);
    for (uint32_t c = 0; c < 3; ++c) EXPECT_EQ(pixel(31, 0, c), want[c]) << "sky, channel " << c;
    EXPECT_EQ(frame->instance_id[0 * kW + 31], 0u);

    // the centre pixel (16, 12) is wall A, shaded on its own normal
    const glm::vec3 lit = rd::shade_vertex_color(s.scene.materials[0], s.scene.lighting, glm::vec3(0.0f, 0.0f, -1.0f)).combined;
    rd::encode_pixel(lit, d.shading, want);
    for (uint32_t c = 0; c < 3; ++c) EXPECT_EQ(pixel(16, 12, c), want[c]) << "wall A, channel " << c;
    EXPECT_EQ(frame->instance_id[12 * kW + 16], 3u);

    // wall B (to the left, nearer): label 9
    EXPECT_EQ(frame->instance_id[12 * kW + 5], 9u);

    // the keypoints: on wall A (visible), in front of B (visible), behind B (occluded), below the frame (absent)
    auto state = [&](uint32_t p) { return frame->keypoint_uv[p * 3 + 2]; };
    EXPECT_EQ(state(0), 2.0f);
    EXPECT_EQ(state(1), 2.0f);
    EXPECT_EQ(state(2), 1.0f);
    EXPECT_EQ(state(3), 0.0f);
    EXPECT_EQ(frame->keypoint_labels[0], 21u);
}
