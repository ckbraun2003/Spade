#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <span>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/tessellate.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// render() -- S7a Task R3, the deterministic CPU rasterizer.
//
// scene_from_world() now wires real geometry into every SDF-derived mesh
// slot (Task R5) -- but every test below still builds its own RenderScene/
// MeshData directly rather than going through scene_from_world(), exactly as
// task-R3-brief.md's context note says to: this file's golden frames are
// pinned to hand-authored scenes so a later, unrelated change to
// scene_from_world()'s own wiring (node-to-mesh mapping, split_program()'s
// grouping, world_bounds_of()'s heuristics) can never move a rasterizer
// golden that has nothing to do with it.
//
// Sections:
//   0. A tiny, self-contained SHA-256 -- the golden manifest's fingerprint
//      (test-only; verified against the two textbook vectors before anything
//      else here trusts it -- same posture as test_render_tessellate.cpp).
//   1. Fixtures/helpers shared by every test below.
//   2. Step 1: buffer-fully-written / opaque-X (MN-14) / geometry-visible /
//      call-twice-byte-identical / purity.
//   3. SR-13 (controller ruling): DrawMode::shaded culls back faces,
//      DrawMode::wireframe does not -- the test that ties Task R2's
//      tessellation-winding fix to a pixel-level observable.
//   4. SR-11 submesh contract: empty submesh arrays == one implicit submesh
//      at material 0; explicit submeshes each keep their own material;
//      DrawItem::material_override replaces every submesh's material.
//   5. RenderOptions::overlays (PA-4): ground grid, world bounds, spawn
//      marker, body marker -- each checked present when enabled, absent when
//      not.
//   6. DrawMode::raymarch is out of scope (Tasks R8/R9) and reported as an
//      error rather than silently rendered some other way; validate_target()
//      failures propagate.
//   7. Byte-exact frame goldens (Step 4).
// ---------------------------------------------------------------------------

namespace {

using spade::Code;
using spade::Error;
using spade::Result;
using spade::SdfPrim;
using spade::render::Aabb;
using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::DrawMode;
using spade::render::kNoMaterial;
using spade::render::kNoMesh;
using spade::render::kTessellationDefaults;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::PixelFormat;
using spade::render::render;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::tessellate_primitive;
using spade::render::validate_target;

// ===========================================================================
// 0. A tiny, self-contained SHA-256 (FIPS 180-4). Test-only, duplicated
// (rather than shared) from test_render_tessellate.cpp's identical block --
// each is a private, anonymous-namespace-scoped helper of its own file, the
// established convention this corpus already uses. Verified below
// (RasterGoldenSha256.MatchesTheStandardTestVectors) before anything in this
// file trusts it.
// ===========================================================================

constexpr std::array<uint32_t, 64> kSha256RoundConstants = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

[[nodiscard]] constexpr uint32_t rotr32(uint32_t x, uint32_t n) noexcept {
    return (x >> n) | (x << (32u - n));
}

void sha256_process_block(std::array<uint32_t, 8>& h, const uint8_t block[64]) {
    std::array<uint32_t, 64> w{};
    for (uint32_t i = 0; i < 16; ++i) {
        w[i] = (static_cast<uint32_t>(block[4 * i]) << 24) | (static_cast<uint32_t>(block[4 * i + 1]) << 16) |
               (static_cast<uint32_t>(block[4 * i + 2]) << 8) | static_cast<uint32_t>(block[4 * i + 3]);
    }
    for (uint32_t i = 16; i < 64; ++i) {
        const uint32_t s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const uint32_t s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (uint32_t i = 0; i < 64; ++i) {
        const uint32_t s1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
        const uint32_t ch = (e & f) ^ ((~e) & g);
        const uint32_t temp1 = hh + s1 + ch + kSha256RoundConstants[i] + w[i];
        const uint32_t s0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
        const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const uint32_t temp2 = s0 + maj;
        hh = g;
        g = f;
        f = e;
        e = d + temp1;
        d = c;
        c = b;
        b = a;
        a = temp1 + temp2;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
}

[[nodiscard]] std::string sha256_hex(std::span<const std::byte> data) {
    std::array<uint32_t, 8> h = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                                  0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};

    const uint64_t bit_length = static_cast<uint64_t>(data.size()) * 8u;
    std::vector<uint8_t> message(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        message[i] = std::to_integer<uint8_t>(data[i]);
    }
    message.push_back(0x80u);
    while (message.size() % 64u != 56u) {
        message.push_back(0u);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
        message.push_back(static_cast<uint8_t>((bit_length >> shift) & 0xffu));
    }

    for (size_t offset = 0; offset < message.size(); offset += 64u) {
        sha256_process_block(h, &message[offset]);
    }

    std::ostringstream out;
    for (const uint32_t word : h) {
        out << std::hex << std::setw(8) << std::setfill('0') << word;
    }
    return out.str();
}

[[nodiscard]] std::string sha256_hex_of(const std::vector<uint8_t>& bytes) {
    return sha256_hex(std::as_bytes(std::span<const uint8_t>(bytes)));
}

// ===========================================================================
// 1. Fixtures/helpers.
// ===========================================================================

// Mirrors test_render_target.cpp's well_formed_target(): storage is the
// caller's, target is a non-owning view over it (PA-1). Pre-filled with a
// sentinel byte (0xAA, never a value render() would legitimately write to
// every byte) so "every byte was actually overwritten" is a real assertion,
// not a coincidence of a zero-initialized vector.
[[nodiscard]] RenderTarget make_target(std::vector<uint8_t>& storage, uint32_t width, uint32_t height) {
    storage.assign(static_cast<size_t>(width) * 4 * height, 0xAAu);
    return RenderTarget{
        .pixels = std::span<uint8_t>(storage),
        .width = width,
        .height = height,
        .stride = width * 4,
        .format = PixelFormat::bgrx8,
    };
}

void render_or_fail(const RenderScene& scene, const Camera& camera, const RenderOptions& options,
                     RenderTarget& target) {
    const Result<void> result = render(scene, camera, options, target);
    if (!result) {
        ADD_FAILURE() << "render() failed: " << result.error().context;
    }
}

// A camera at `position` with identity orientation -- looks toward world -Z
// (Camera's own default orientation, target.hpp).
[[nodiscard]] Camera camera_looking_down_neg_z(glm::vec3 position) {
    Camera camera;
    camera.position = position;
    return camera;
}

// A camera at `position`, pitched -90 degrees about local X -- looks straight
// down world -Y. Numerically identical to the wireframe rasterizer's own
// default CameraPose orientation (dronesim/spade/raster.h): (cos(-45deg),
// sin(-45deg), 0, 0) = (0.70710678, -0.70710678, 0, 0).
//
// Spelled as literal float32 components (review finding, Task R3 fix round):
// this camera feeds RasterGolden.CylinderStaticAndDynamicBoxTopDownMatches-
// CommittedManifest's committed sha256, and glm::angleAxis computes sin/cos
// through libm -- exactly the cross-platform-ulp hazard render/raster_cpu.cpp
// (its tan32 comment) and render/tessellate.cpp already document, now closed
// on the test side too. 0x1.6a09e6p-1f is the nearest float32 to cos(45deg) =
// sin(45deg) = sqrt(2)/2, the same fp32_math.hpp "hex float literal for an
// irrational constant" discipline this engine already follows.
[[nodiscard]] Camera camera_top_down(glm::vec3 position) {
    Camera camera;
    camera.position = position;
    camera.orientation = glm::quat(0x1.6a09e6p-1f, -0x1.6a09e6p-1f, 0.0f, 0.0f);
    return camera;
}

// The background pixel (BGR only -- position (0,0), a corner render()
// unconditionally clears and no test scene below ever draws over) of an
// otherwise-empty scene. Used as a live reference rather than hard-coding
// the wireframe rasterizer's background constant a second time in this file.
[[nodiscard]] std::array<uint8_t, 3> background_pixel(uint32_t width, uint32_t height) {
    const RenderScene scene;  // no meshes, no statics/dynamics, no spawns
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 1000.0f, 1000.0f));  // looks at nothing
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, width, height);
    render_or_fail(scene, camera, options, target);
    return {storage[0], storage[1], storage[2]};
}

[[nodiscard]] bool region_contains_bgr(const std::vector<uint8_t>& storage, uint32_t width, uint32_t x0, uint32_t x1,
                                        uint32_t y0, uint32_t y1, std::array<uint8_t, 3> bgr) {
    for (uint32_t y = y0; y < y1; ++y) {
        for (uint32_t x = x0; x < x1; ++x) {
            const size_t idx = (static_cast<size_t>(y) * width + x) * 4;
            if (storage[idx] == bgr[0] && storage[idx + 1] == bgr[1] && storage[idx + 2] == bgr[2]) {
                return true;
            }
        }
    }
    return false;
}

// Stronger than region_contains_bgr: EVERY pixel in the region must match --
// needed where "never drew anything at all" (not merely "never drew some
// OTHER specific colour") is the claim, e.g. a malformed submesh that must
// be skipped entirely rather than merely mis-coloured.
[[nodiscard]] bool region_is_entirely_bgr(const std::vector<uint8_t>& storage, uint32_t width, uint32_t x0,
                                           uint32_t x1, uint32_t y0, uint32_t y1, std::array<uint8_t, 3> bgr) {
    for (uint32_t y = y0; y < y1; ++y) {
        for (uint32_t x = x0; x < x1; ++x) {
            const size_t idx = (static_cast<size_t>(y) * width + x) * 4;
            if (storage[idx] != bgr[0] || storage[idx + 1] != bgr[1] || storage[idx + 2] != bgr[2]) {
                return false;
            }
        }
    }
    return true;
}

[[nodiscard]] size_t count_non_matching_bgr(const std::vector<uint8_t>& storage, std::array<uint8_t, 3> bgr) {
    size_t count = 0;
    for (size_t idx = 0; idx + 4 <= storage.size(); idx += 4) {
        if (storage[idx] != bgr[0] || storage[idx + 1] != bgr[1] || storage[idx + 2] != bgr[2]) {
            ++count;
        }
    }
    return count;
}

// Re-derives render()'s own float-channel -> byte conversion independently
// (same "second source" posture as test_render_tessellate.cpp's
// expected_vertex_count/expected_index_count), so a test asserting an exact
// material colour is not just calling back into the code under test.
[[nodiscard]] uint8_t channel_to_byte(float channel) {
    const float clamped = std::clamp(channel, 0.0f, 1.0f);
    return static_cast<uint8_t>(std::lround(clamped * 255.0f));
}

// R, G, B in that order (matching how kBackgroundR/kGridR/etc. are declared
// in raster_cpu.cpp) -- returned as the {B, G, R} triple BGRX8 actually
// stores, so callers never have to reorder by hand.
[[nodiscard]] std::array<uint8_t, 3> bgr(uint8_t r, uint8_t g, uint8_t b) { return {b, g, r}; }

[[nodiscard]] std::array<uint8_t, 3> expected_bgr(const Material& material) {
    return bgr(channel_to_byte(material.base_color.r), channel_to_byte(material.base_color.g),
               channel_to_byte(material.base_color.b));
}

// A hand-built, CCW-outward-wound box -- the SAME face/corner convention as
// render/tessellate.cpp's tessellate_box() (SR-13's back-face cull depends on
// it), built directly here rather than through tessellate_primitive() (or
// scene_from_world(), which now wires tessellate_primitive() in for real --
// Task R5) so this test's own golden hash stays isolated from either one
// (task-R3-brief.md's own note: this task's tests build MeshData directly).
[[nodiscard]] MeshData make_box_mesh(float half_extent) {
    MeshData mesh;
    const float h = half_extent;
    struct Face {
        glm::vec3 normal;
        glm::vec3 corners[4];  // CCW as seen from outside the box
    };
    const Face faces[6] = {
        {{1.0f, 0.0f, 0.0f}, {{h, -h, -h}, {h, h, -h}, {h, h, h}, {h, -h, h}}},
        {{-1.0f, 0.0f, 0.0f}, {{-h, -h, h}, {-h, h, h}, {-h, h, -h}, {-h, -h, -h}}},
        {{0.0f, 1.0f, 0.0f}, {{-h, h, h}, {h, h, h}, {h, h, -h}, {-h, h, -h}}},
        {{0.0f, -1.0f, 0.0f}, {{-h, -h, -h}, {h, -h, -h}, {h, -h, h}, {-h, -h, h}}},
        {{0.0f, 0.0f, 1.0f}, {{h, -h, h}, {h, h, h}, {-h, h, h}, {-h, -h, h}}},
        {{0.0f, 0.0f, -1.0f}, {{-h, -h, -h}, {-h, h, -h}, {h, h, -h}, {h, -h, -h}}},
    };
    for (const Face& f : faces) {
        const uint32_t base = static_cast<uint32_t>(mesh.positions.size());
        for (const glm::vec3& c : f.corners) {
            mesh.positions.push_back(c);
            mesh.normals.push_back(f.normal);
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return mesh;
}

// A single triangle at z=+1, outward normal +Z -- CCW-outward as seen from a
// camera on the +Z side looking toward -Z (SR-13's canonical "front face"
// case). `reversed` swaps the last two indices, producing the SAME triangle
// geometrically but the opposite winding, with nothing else different.
[[nodiscard]] MeshData make_single_triangle(bool reversed) {
    MeshData mesh;
    mesh.positions = {glm::vec3(1.0f, -1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(-1.0f, 1.0f, 1.0f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = reversed ? std::vector<uint32_t>{0, 2, 1} : std::vector<uint32_t>{0, 1, 2};
    return mesh;
}

// Two separate, non-overlapping, correctly outward-wound (z=+1) triangles --
// a "left" one (x roughly in [-2,-1]) and a "right" one (x roughly in [1,2])
// -- so a test can sample the left/right halves of a centered frame
// independently to prove a per-submesh material choice.
[[nodiscard]] MeshData make_two_triangle_mesh() {
    MeshData mesh;
    mesh.positions = {
        glm::vec3(-1.0f, -1.0f, 1.0f), glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3(-2.0f, 1.0f, 1.0f),  // left: 0,1,2
        glm::vec3(2.0f, -1.0f, 1.0f), glm::vec3(2.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f),      // right: 3,4,5
    };
    mesh.normals.assign(6, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2, 3, 4, 5};
    return mesh;
}

// One static DrawItem at `local_to_world`, default material (index 0, the
// struct default grey). Callers overwrite `.materials` afterward when a test
// cares about a specific colour.
[[nodiscard]] RenderScene make_scene(MeshData mesh, glm::mat4 local_to_world = glm::mat4(1.0f)) {
    RenderScene scene;
    scene.meshes.push_back(std::move(mesh));
    scene.materials = {Material{}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = local_to_world, .material_override = kNoMaterial});
    scene.bounds = Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    return scene;
}

// A scene with no meshes at all -- overlay-only fixtures (section 5) start
// from this and add spawn/dynamics data as each test needs.
[[nodiscard]] RenderScene make_empty_overlay_scene() {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.bounds = Aabb{.min = glm::vec3(-3.0f, -1.0f, -3.0f), .max = glm::vec3(3.0f, 2.0f, 3.0f)};
    return scene;
}

constexpr uint32_t kSmallWidth = 64, kSmallHeight = 48;

}  // namespace

// ===========================================================================
// 0. The golden manifest's own hash function is trustworthy.
// ===========================================================================

TEST(RasterGoldenSha256, MatchesTheStandardTestVectors) {
    const std::string empty;
    EXPECT_EQ(sha256_hex(std::as_bytes(std::span<const char>(empty.data(), empty.size()))),
              "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    const std::string abc = "abc";
    EXPECT_EQ(sha256_hex(std::as_bytes(std::span<const char>(abc.data(), abc.size()))),
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

// ===========================================================================
// 2. Step 1: the buffer is fully written, opaque, shows geometry, and
//    render() is deterministic and pure.
// ===========================================================================

TEST(Render, WritesEveryPixelOpaqueBgrx8AndShowsGeometry) {
    const RenderScene scene = make_scene(make_box_mesh(1.0f));
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;  // isolate "does the box itself show up"

    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    ASSERT_EQ(storage.size(), static_cast<size_t>(kSmallWidth) * kSmallHeight * 4);
    for (size_t i = 3; i < storage.size(); i += 4) {
        ASSERT_EQ(storage[i], 0xFFu) << "pixel " << (i / 4) << "'s X byte is not opaque (MN-14)";
    }

    const auto bg = background_pixel(kSmallWidth, kSmallHeight);
    EXPECT_GT(count_non_matching_bgr(storage, bg), 0u) << "the box should be visible against the background";
}

TEST(Render, CallingTwiceProducesByteIdenticalBuffers) {
    const RenderScene scene = make_scene(make_box_mesh(1.0f));
    // An off-center camera -- exercises more of the near/far + projection
    // paths than a perfectly centered view would.
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.5f, -0.3f, 6.0f));
    const RenderOptions options;  // default: shaded, overlays=true

    std::vector<uint8_t> storage_a, storage_b;
    RenderTarget target_a = make_target(storage_a, kSmallWidth, kSmallHeight);
    RenderTarget target_b = make_target(storage_b, kSmallWidth, kSmallHeight);

    render_or_fail(scene, camera, options, target_a);
    render_or_fail(scene, camera, options, target_b);

    EXPECT_EQ(storage_a, storage_b);
}

TEST(Render, PurityConstSceneUnchangedAndOutputDeterministic) {
    const RenderScene scene = make_scene(make_box_mesh(1.0f));
    // render() takes `const RenderScene&`; a snapshot copy taken before and
    // compared after is belt-and-suspenders against a future refactor
    // sneaking in a const_cast or mutable member, not merely trusting the
    // type system today.
    const RenderScene before = scene;
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    const RenderOptions options;

    std::vector<uint8_t> storage_a, storage_b;
    RenderTarget target_a = make_target(storage_a, kSmallWidth, kSmallHeight);
    RenderTarget target_b = make_target(storage_b, kSmallWidth, kSmallHeight);

    render_or_fail(scene, camera, options, target_a);
    render_or_fail(scene, camera, options, target_b);

    EXPECT_EQ(storage_a, storage_b);

    ASSERT_EQ(scene.meshes.size(), before.meshes.size());
    EXPECT_EQ(scene.meshes[0].positions, before.meshes[0].positions);
    EXPECT_EQ(scene.meshes[0].normals, before.meshes[0].normals);
    EXPECT_EQ(scene.meshes[0].indices, before.meshes[0].indices);
    ASSERT_EQ(scene.statics.size(), before.statics.size());
    EXPECT_EQ(scene.statics[0].mesh_index, before.statics[0].mesh_index);
    EXPECT_EQ(scene.statics[0].material_override, before.statics[0].material_override);
    ASSERT_EQ(scene.materials.size(), before.materials.size());
}

// ===========================================================================
// 3. SR-13 -- DrawMode::shaded culls back faces; DrawMode::wireframe does
//    not. This is what makes Task R2's tessellation-winding fix (four of
//    seven primitives had backward triangle order) observable at the pixel
//    level rather than only at the vertex-order level.
// ===========================================================================

TEST(RasterCpu, ShadedModeRendersOutwardFacingTriangleButCullsReversedOne) {
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    const auto bg = background_pixel(kSmallWidth, kSmallHeight);
    RenderOptions shaded;
    shaded.mode = DrawMode::shaded;
    shaded.overlays = false;

    {
        const RenderScene scene = make_scene(make_single_triangle(/*reversed=*/false));
        std::vector<uint8_t> storage;
        RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
        render_or_fail(scene, camera, shaded, target);
        EXPECT_GT(count_non_matching_bgr(storage, bg), 0u)
            << "a correctly-wound, outward-facing triangle must be visible in shaded mode";
    }
    {
        const RenderScene scene = make_scene(make_single_triangle(/*reversed=*/true));
        std::vector<uint8_t> storage;
        RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
        render_or_fail(scene, camera, shaded, target);
        EXPECT_EQ(count_non_matching_bgr(storage, bg), 0u)
            << "a reversed-winding triangle must be back-face culled in shaded mode (SR-13) -- "
               "this is Task R2's winding fix made observable at the pixel level";
    }
}

TEST(RasterCpu, WireframeModeDrawsBothWindingsIdentically) {
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions wireframe;
    wireframe.mode = DrawMode::wireframe;
    wireframe.overlays = false;

    const RenderScene forward_scene = make_scene(make_single_triangle(false));
    const RenderScene reversed_scene = make_scene(make_single_triangle(true));

    std::vector<uint8_t> storage_forward, storage_reversed;
    RenderTarget target_forward = make_target(storage_forward, kSmallWidth, kSmallHeight);
    RenderTarget target_reversed = make_target(storage_reversed, kSmallWidth, kSmallHeight);

    render_or_fail(forward_scene, camera, wireframe, target_forward);
    render_or_fail(reversed_scene, camera, wireframe, target_reversed);

    // SR-13: wireframe never culls -- a triangle's 3 edges are the same set
    // of segments regardless of traversal direction, so a merely-reversed
    // index order produces byte-identical output.
    EXPECT_EQ(storage_forward, storage_reversed);

    const auto bg = background_pixel(kSmallWidth, kSmallHeight);
    EXPECT_GT(count_non_matching_bgr(storage_forward, bg), 0u)
        << "wireframe mode must still draw something for a back-facing triangle";
}

// ===========================================================================
// 4. SR-11 submesh contract.
// ===========================================================================

TEST(RasterCpu, EmptySubmeshArraysUseTheDefaultMaterialAcrossTheWholeMesh) {
    RenderScene scene = make_scene(make_two_triangle_mesh());
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)};
    scene.materials = {red};  // submesh_first_index/index_count/material all stay empty (SR-11)

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    const auto expected = expected_bgr(red);
    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected))
        << "the left triangle should render in the single implicit submesh's material";
    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight, expected))
        << "the right triangle should render in the SAME material -- one implicit submesh spans the whole mesh";
}

TEST(RasterCpu, ExplicitSubmeshesEachKeepTheirOwnMaterial) {
    MeshData mesh = make_two_triangle_mesh();
    mesh.submesh_first_index = {0, 3};
    mesh.submesh_index_count = {3, 3};
    mesh.submesh_material = {0, 1};

    RenderScene scene = make_scene(std::move(mesh));
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)};
    const Material blue{.base_color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)};
    scene.materials = {red, blue};

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected_bgr(red)))
        << "the left triangle (submesh 0) should be red";
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected_bgr(blue)))
        << "the left triangle should never render blue";
    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight,
                                     expected_bgr(blue)))
        << "the right triangle (submesh 1) should be blue";
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight,
                                      expected_bgr(red)))
        << "the right triangle should never render red";
}

TEST(RasterCpu, DrawItemMaterialOverrideReplacesEverySubmeshsMaterial) {
    MeshData mesh = make_two_triangle_mesh();
    mesh.submesh_first_index = {0, 3};
    mesh.submesh_index_count = {3, 3};
    mesh.submesh_material = {0, 1};  // would normally be red/blue

    RenderScene scene;
    scene.meshes.push_back(std::move(mesh));
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)};
    const Material blue{.base_color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)};
    const Material green{.base_color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)};
    scene.materials = {red, blue, green};
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = 2});
    scene.bounds = Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth, 0, kSmallHeight, expected_bgr(green)))
        << "material_override should replace every submesh's own material";
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth, 0, kSmallHeight, expected_bgr(red)));
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth, 0, kSmallHeight, expected_bgr(blue)));
}

// ---------------------------------------------------------------------------
// Review fix round (2 Important findings): draw_mesh_item must never trust
// unvalidated MeshData -- scene.hpp's own MESH INDEX SPACE note says
// resolving a world visual reference to real geometry (e.g. Task R4's glTF
// loader) is the CALLER's job, so a malformed or truncated file's data
// reaches this function without this module ever having checked it. Each
// case below corrupts exactly one thing a real loader could get wrong and
// asserts render() neither crashes (a debug build's _CrtIsValidHeapPointer /
// vector::at-style abort, or plain UB in release) nor draws garbage -- the
// well-formed part of the mesh still renders, only the malformed part is
// silently skipped.
// ---------------------------------------------------------------------------

TEST(RasterCpu, MismatchedSubmeshArrayLengthsDoNotCrashAndDrawOnlyTheShortestPrefix) {
    MeshData mesh = make_two_triangle_mesh();
    // submesh_first_index (3 entries) outruns submesh_index_count/material (1
    // and 2 entries) -- SR-11 says these three arrays are parallel, but
    // nothing upstream enforces it for file-derived data.
    mesh.submesh_first_index = {0, 3, 999};
    mesh.submesh_index_count = {3};
    mesh.submesh_material = {0, 1};

    RenderScene scene = make_scene(std::move(mesh));
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)};
    scene.materials = {red};

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);  // must not crash / read out of bounds

    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected_bgr(red)))
        << "the one fully-specified submesh (index 0, the shortest array's length) should still draw";
    // The right triangle's submesh (index 1) is beyond the shortest array's
    // length -- submesh_count clamps to 1, so it must never be drawn at all,
    // in any colour.
    const auto bg = background_pixel(kSmallWidth, kSmallHeight);
    EXPECT_TRUE(region_is_entirely_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight, bg))
        << "the right triangle's out-of-range submesh entry must never be drawn -- that half stays background";
}

TEST(RasterCpu, SubmeshRangeExceedingTheIndexBufferIsSkippedNotTheWholeMesh) {
    MeshData mesh = make_two_triangle_mesh();  // 6 indices total
    mesh.submesh_first_index = {0, 3};
    mesh.submesh_index_count = {3, 999};  // second submesh's range runs past indices.size()
    mesh.submesh_material = {0, 1};

    RenderScene scene = make_scene(std::move(mesh));
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)};
    const Material blue{.base_color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)};
    scene.materials = {red, blue};

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);  // must not read past mesh.indices

    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected_bgr(red)))
        << "the well-formed left submesh should still draw";
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight,
                                      expected_bgr(blue)))
        << "the out-of-range right submesh must be skipped, not read out of bounds";
}

TEST(RasterCpu, IndexExceedingTheVertexBufferSkipsOnlyThatTriangle) {
    MeshData mesh = make_two_triangle_mesh();  // indices = {0,1,2, 3,4,5}; positions.size() == 6
    mesh.indices[3] = 9999;  // the right triangle's first index now points past positions.size()

    RenderScene scene = make_scene(std::move(mesh));
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)};
    scene.materials = {red};  // single implicit submesh (SR-11) -- both triangles would otherwise be red

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);  // must not dereference positions[9999]

    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected_bgr(red)))
        << "the left triangle's indices are untouched and should still draw";
    const auto bg = background_pixel(kSmallWidth, kSmallHeight);
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight,
                                      expected_bgr(red)))
        << "the right triangle has an out-of-range index and must be skipped, not drawn";
    EXPECT_GT(count_non_matching_bgr(storage, bg), 0u) << "the left triangle should still be visible somewhere";
}

TEST(RasterCpu, EmptyMaterialsListSkipsDrawingRatherThanReadingMaterialsZero) {
    RenderScene scene;  // scene.materials left default-empty, deliberately
    scene.meshes.push_back(make_box_mesh(1.0f));
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});
    scene.bounds = Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    ASSERT_TRUE(scene.materials.empty());

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);  // must not index scene.materials[0] on an empty vector

    const auto bg = background_pixel(kSmallWidth, kSmallHeight);
    EXPECT_EQ(count_non_matching_bgr(storage, bg), 0u)
        << "with no material at all to shade with, the item must be skipped, not drawn with an untrusted fallback";
}

// ===========================================================================
// 5. RenderOptions::overlays (PA-4) -- ground grid, world bounds, spawn
//    marker, body marker. Each is checked present with overlays on and
//    absent with overlays off, using the wireframe rasterizer's own
//    documented, unchanged colour constants (raster_cpu.cpp).
// ===========================================================================

constexpr uint32_t kOverlayWidth = 96, kOverlayHeight = 96;

TEST(RasterCpu, OverlaysDrawGroundGridWhenEnabled) {
    const RenderScene scene = make_empty_overlay_scene();  // has_ground=false -> grid defaults to y=0
    // Same camera as the wireframe rasterizer's own default CameraPose.
    const Camera camera = camera_top_down(glm::vec3(0.0f, 10.0f, 0.0001f));
    const auto grid = bgr(90, 90, 90);

    RenderOptions with_overlays;
    with_overlays.overlays = true;
    std::vector<uint8_t> storage_on;
    RenderTarget target_on = make_target(storage_on, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, with_overlays, target_on);
    EXPECT_TRUE(region_contains_bgr(storage_on, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, grid));

    RenderOptions without_overlays;
    without_overlays.overlays = false;
    std::vector<uint8_t> storage_off;
    RenderTarget target_off = make_target(storage_off, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, without_overlays, target_off);
    EXPECT_FALSE(region_contains_bgr(storage_off, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, grid));
}

TEST(RasterCpu, OverlaysDrawWorldBoundsBoxWhenEnabled) {
    const RenderScene scene = make_empty_overlay_scene();  // bounds = [-3,-1,-3]..[3,2,3]
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.5f, 12.0f));
    const auto bounds = bgr(90, 140, 200);

    RenderOptions with_overlays;
    with_overlays.overlays = true;
    std::vector<uint8_t> storage_on;
    RenderTarget target_on = make_target(storage_on, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, with_overlays, target_on);
    EXPECT_TRUE(region_contains_bgr(storage_on, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, bounds));

    RenderOptions without_overlays;
    without_overlays.overlays = false;
    std::vector<uint8_t> storage_off;
    RenderTarget target_off = make_target(storage_off, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, without_overlays, target_off);
    EXPECT_FALSE(region_contains_bgr(storage_off, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, bounds));
}

TEST(RasterCpu, OverlaysDrawSpawnMarkerWhenEnabled) {
    RenderScene scene = make_empty_overlay_scene();
    scene.spawn_positions = {glm::vec3(0.0f, 0.0f, 0.0f)};
    scene.spawn_orientations = {glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    const Camera camera = camera_top_down(glm::vec3(0.0f, 1.2f, 0.0001f));  // close: the marker's radius is 0.3
    const auto spawn = bgr(190, 90, 170);

    RenderOptions with_overlays;
    with_overlays.overlays = true;
    std::vector<uint8_t> storage_on;
    RenderTarget target_on = make_target(storage_on, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, with_overlays, target_on);
    EXPECT_TRUE(region_contains_bgr(storage_on, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, spawn));

    RenderOptions without_overlays;
    without_overlays.overlays = false;
    std::vector<uint8_t> storage_off;
    RenderTarget target_off = make_target(storage_off, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, without_overlays, target_off);
    EXPECT_FALSE(region_contains_bgr(storage_off, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, spawn));
}

TEST(RasterCpu, OverlaysDrawBodyMarkerForDynamicItemEvenWithoutAResolvedMesh) {
    RenderScene scene = make_empty_overlay_scene();
    // kNoMesh: this body has no resolved visual mesh yet -- the marker
    // overlay is an orientation aid independent of that, and must still draw.
    scene.dynamics.push_back(
        DrawItem{.mesh_index = kNoMesh, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});
    const Camera camera = camera_top_down(glm::vec3(0.0f, 1.2f, 0.0001f));
    const auto drone = bgr(124, 147, 255);  // #7C93FF

    RenderOptions with_overlays;
    with_overlays.overlays = true;
    std::vector<uint8_t> storage_on;
    RenderTarget target_on = make_target(storage_on, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, with_overlays, target_on);
    EXPECT_TRUE(region_contains_bgr(storage_on, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, drone));

    RenderOptions without_overlays;
    without_overlays.overlays = false;
    std::vector<uint8_t> storage_off;
    RenderTarget target_off = make_target(storage_off, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, without_overlays, target_off);
    EXPECT_FALSE(region_contains_bgr(storage_off, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, drone));
}

// ===========================================================================
// 6. DrawMode::raymarch is out of this task's scope; validate_target()
//    failures propagate.
// ===========================================================================

TEST(RasterCpu, RaymarchModeIsNotImplementedAndReturnsAnError) {
    const RenderScene scene = make_scene(make_box_mesh(1.0f));
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.mode = DrawMode::raymarch;

    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);

    const Result<void> result = render(scene, camera, options, target);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, Code::invalid_argument);
}

TEST(RasterCpu, PropagatesValidateTargetFailure) {
    const RenderScene scene = make_scene(make_box_mesh(1.0f));
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    const RenderOptions options;

    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, 32, 24);
    target.stride = 0;  // now invalid: stride != width * 4

    const Result<void> result = render(scene, camera, options, target);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, Code::invalid_argument);
}

// ===========================================================================
// 7. Byte-exact frame goldens (Step 4). Each fixture builds a fully-populated
// RenderScene by hand -- statics/dynamics, materials, overlay data -- using
// tessellate_primitive() (Task R2) for realistic mesh geometry, so the
// tessellation_limits_version recorded in the manifest pins WHICH
// kTessellationDefaults these hashes were captured against (tests/golden/
// render/tessellation/manifest.json's own convention). See that manifest's
// "_regeneration_note" analogue below: R6 (lighting) and R7 (shadows) are
// EXPECTED to move these hashes; a move alongside a matching lighting/shadow
// commit is intentional, a move with no such commit is rot.
// ===========================================================================

namespace {

constexpr uint32_t kGoldenWidth = 160, kGoldenHeight = 120;
const Aabb kGoldenTessellationBounds{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};

[[nodiscard]] MeshData tessellate_or_fail(SdfPrim kind, glm::vec4 params) {
    const Result<MeshData> mesh = tessellate_primitive(kind, params, kGoldenTessellationBounds, kTessellationDefaults);
    if (!mesh) {
        ADD_FAILURE() << "tessellate_primitive failed: " << mesh.error().context;
        return MeshData{};
    }
    return *mesh;
}

[[nodiscard]] glm::mat4 pose_at(glm::vec3 position, glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    glm::mat4 m = glm::mat4_cast(orientation);
    m[3] = glm::vec4(position, 1.0f);
    return m;
}

// Golden A: a single tessellated box, shaded, sitting on the ground plane,
// with every overlay category also present (grid + bounds + spawn + a
// mesh-less dynamic body marker).
[[nodiscard]] RenderScene golden_scene_box_shaded_with_overlays() {
    RenderScene scene;
    scene.meshes.push_back(tessellate_or_fail(SdfPrim::box, glm::vec4(1.0f, 0.8f, 1.2f, 0.0f)));
    scene.materials = {Material{.base_color = glm::vec4(0.85f, 0.35f, 0.2f, 1.0f)}};
    scene.statics.push_back(DrawItem{
        .mesh_index = 0, .local_to_world = pose_at(glm::vec3(0.0f, 0.8f, 0.0f)), .material_override = kNoMaterial});
    // A body with no resolved visual mesh yet -- exercises the body-marker
    // overlay's independence from real dynamic geometry (draw_mesh_item skips
    // kNoMesh; draw_body_markers does not).
    scene.dynamics.push_back(DrawItem{
        .mesh_index = kNoMesh, .local_to_world = pose_at(glm::vec3(-1.0f, 0.15f, 0.5f)), .material_override = kNoMaterial});
    scene.has_ground = true;
    scene.ground_y = 0.0f;
    scene.bounds = Aabb{.min = glm::vec3(-2.0f, -0.2f, -2.0f), .max = glm::vec3(2.0f, 2.0f, 2.0f)};
    scene.spawn_positions = {glm::vec3(1.2f, 0.0f, 1.2f)};
    scene.spawn_orientations = {glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    return scene;
}

// Golden B: a single tessellated sphere, wireframe, no overlays -- isolates
// the "old look" edges-only draw path from the ground/bounds/spawn/body
// overlay layer entirely.
[[nodiscard]] RenderScene golden_scene_sphere_wireframe_no_overlays() {
    RenderScene scene;
    scene.meshes.push_back(tessellate_or_fail(SdfPrim::sphere, glm::vec4(1.3f, 0.0f, 0.0f, 0.0f)));
    scene.materials = {Material{.base_color = glm::vec4(0.2f, 0.5f, 0.9f, 1.0f)}};
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});
    scene.bounds = Aabb{.min = glm::vec3(-3.0f), .max = glm::vec3(3.0f)};  // unused: overlays are off
    return scene;
}

// Golden C: a static cylinder plus a dynamic (rotated) box, shaded, viewed
// top-down, every overlay category on, and DrawItem::material_override in
// play on the dynamic item -- the broadest single golden, exercising
// statics+dynamics+override+overlays together.
[[nodiscard]] RenderScene golden_scene_cylinder_static_dynamic_box_top_down() {
    RenderScene scene;
    scene.meshes.push_back(tessellate_or_fail(SdfPrim::cylinder, glm::vec4(0.8f, 1.0f, 0.0f, 0.0f)));  // [0]
    scene.meshes.push_back(tessellate_or_fail(SdfPrim::box, glm::vec4(0.3f, 0.3f, 0.3f, 0.0f)));       // [1]
    scene.materials = {
        Material{.base_color = glm::vec4(0.15f, 0.55f, 0.5f, 1.0f)},   // teal -- the cylinder's own (submesh 0)
        Material{.base_color = glm::vec4(0.7f, 0.2f, 0.6f, 1.0f)},     // magenta -- the box's override
    };
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});
    // 30 deg about Y, spelled as literal float32 half-angle (15 deg)
    // components rather than glm::angleAxis (review finding -- see
    // camera_top_down's identical comment above: this quaternion feeds a
    // committed golden hash, and angleAxis's sin/cos go through libm).
    // 0x1.ee8dd4p-1f/0x1.0907dcp-2f are the nearest float32 values to
    // cos(15deg)/sin(15deg).
    const glm::quat box_spin(0x1.ee8dd4p-1f, 0.0f, 0x1.0907dcp-2f, 0.0f);
    scene.dynamics.push_back(DrawItem{
        .mesh_index = 1, .local_to_world = pose_at(glm::vec3(1.5f, 0.3f, 0.5f), box_spin), .material_override = 1});
    scene.has_ground = true;
    scene.ground_y = 0.0f;
    scene.bounds = Aabb{.min = glm::vec3(-2.0f, -0.5f, -2.0f), .max = glm::vec3(2.0f, 2.0f, 2.0f)};
    scene.spawn_positions = {glm::vec3(-1.2f, 0.0f, -1.2f)};
    // 45 deg about Y, half-angle 22.5 deg -- same literal-quaternion fix as
    // box_spin above; 0x1.d906bcp-1f/0x1.87de2ap-2f are the nearest float32
    // values to cos(22.5deg)/sin(22.5deg).
    scene.spawn_orientations = {glm::quat(0x1.d906bcp-1f, 0.0f, 0x1.87de2ap-2f, 0.0f)};
    return scene;
}

[[nodiscard]] std::vector<uint8_t> render_golden(const RenderScene& scene, const Camera& camera,
                                                  const RenderOptions& options) {
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kGoldenWidth, kGoldenHeight);
    render_or_fail(scene, camera, options, target);
    return storage;
}

[[nodiscard]] std::filesystem::path golden_manifest_path() {
    return std::filesystem::path(SPADE_GOLDEN_DIR) / "render" / "frames" / "manifest.json";
}

// Bumped only alongside a deliberate kTessellationDefaults change, together
// with tests/golden/render/frames/manifest.json's own field -- mirrors
// tests/golden/render/tessellation/manifest.json's identical convention.
constexpr const char* kExpectedTessellationLimitsVersion = "kTessellationDefaults@1";

void check_against_manifest(const std::string& frame_name, const std::vector<uint8_t>& pixels) {
    const std::filesystem::path path = golden_manifest_path();
    ASSERT_TRUE(std::filesystem::exists(path)) << path.string();

    YAML::Node root = YAML::LoadFile(path.string());
    ASSERT_TRUE(root["tessellation_limits_version"]) << "manifest missing tessellation_limits_version";
    EXPECT_EQ(root["tessellation_limits_version"].as<std::string>(), kExpectedTessellationLimitsVersion)
        << "the manifest's tessellation_limits_version no longer matches this test's own constant -- if "
           "kTessellationDefaults changed deliberately, update BOTH together (and regenerate these goldens)";

    const YAML::Node frames = root["frames"];
    ASSERT_TRUE(frames) << "manifest missing frames";
    const YAML::Node entry = frames[frame_name];
    ASSERT_TRUE(entry) << "manifest missing entry for " << frame_name;

    EXPECT_EQ(sha256_hex_of(pixels), entry["sha256"].as<std::string>())
        << frame_name << ": rendered frame no longer matches the committed golden";
}

}  // namespace

TEST(RasterGolden, BoxShadedWithOverlaysMatchesCommittedManifest) {
    const RenderScene scene = golden_scene_box_shaded_with_overlays();
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 1.5f, 6.0f));
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = true;

    const std::vector<uint8_t> pixels = render_golden(scene, camera, options);
    const auto bg = background_pixel(kGoldenWidth, kGoldenHeight);
    ASSERT_GT(count_non_matching_bgr(pixels, bg), 0u)
        << "sanity floor: this scene/camera must actually show something before its hash means anything";
    check_against_manifest("box_shaded_with_overlays", pixels);
}

TEST(RasterGolden, SphereWireframeNoOverlaysMatchesCommittedManifest) {
    const RenderScene scene = golden_scene_sphere_wireframe_no_overlays();
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.mode = DrawMode::wireframe;
    options.overlays = false;

    const std::vector<uint8_t> pixels = render_golden(scene, camera, options);
    const auto bg = background_pixel(kGoldenWidth, kGoldenHeight);
    ASSERT_GT(count_non_matching_bgr(pixels, bg), 0u)
        << "sanity floor: this scene/camera must actually show something before its hash means anything";
    check_against_manifest("sphere_wireframe_no_overlays", pixels);
}

TEST(RasterGolden, CylinderStaticAndDynamicBoxTopDownMatchesCommittedManifest) {
    const RenderScene scene = golden_scene_cylinder_static_dynamic_box_top_down();
    const Camera camera = camera_top_down(glm::vec3(0.0f, 4.0f, 0.0001f));
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = true;

    const std::vector<uint8_t> pixels = render_golden(scene, camera, options);
    const auto bg = background_pixel(kGoldenWidth, kGoldenHeight);
    ASSERT_GT(count_non_matching_bgr(pixels, bg), 0u)
        << "sanity floor: this scene/camera must actually show something before its hash means anything";
    check_against_manifest("cylinder_static_dynamic_box_top_down", pixels);
}
