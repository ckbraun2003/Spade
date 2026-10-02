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

#include <world/builder.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "core/fp32_math.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/shadow.hpp"
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
//   6. DrawMode::raymarch (S7a Task R8) forwards to render/raymarch.cpp's
//      render_raymarch() rather than erroring (see raymarch.hpp's own header
//      for that path's algorithm -- test_render_raymarch.cpp owns its actual
//      behaviour; this file pins only that render()'s DISPATCH no longer
//      rejects the mode); validate_target() failures propagate.
//   7. Byte-exact frame goldens (Step 4).
// ---------------------------------------------------------------------------

namespace {

using spade::Code;
using spade::Error;
using spade::Result;
using spade::SdfPrim;
using spade::render::Aabb;
using spade::render::build_static_shadow_map;
using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::DrawMode;
using spade::render::GroundPlane;
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
using spade::render::ShadowMap;
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
// down world -Y. Numerically identical to the interim wireframe rasterizer's
// default top-down camera orientation: (cos(-45deg),
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

// A full reference render of an otherwise-empty scene (no meshes, no
// statics/dynamics/ground_planes, no overlays) at the given camera/size --
// used to tell "matches background" from "differs from background" pixel by
// pixel. S7a Task R6 replaced render()'s old flat background clear with a
// vertical sky gradient (+ an analytic ground for a scene that has one,
// never true of the default-constructed RenderScene this function renders),
// so "the background" is no longer one flat colour a single BGR triple can
// stand in for -- it varies by screen row -- and a comparison against only
// row 0's colour would misclassify every OTHER row's legitimate sky-gradient
// pixel as "something got drawn". Comparing against this full reference
// buffer instead stays correct under a non-flat background exactly the way
// a single reference colour no longer can.
[[nodiscard]] std::vector<uint8_t> render_background_only(const Camera& camera, uint32_t width, uint32_t height) {
    const RenderScene scene;  // no meshes, no statics/dynamics, no ground_planes
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, width, height);
    render_or_fail(scene, camera, options, target);
    return storage;
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

// Stronger than region_contains_bgr: EVERY pixel in the region must match the
// SAME-SIZED reference buffer (render_background_only(), above) -- needed
// where "never drew anything beyond background" (not merely "never drew some
// OTHER specific colour") is the claim, e.g. a malformed submesh that must
// be skipped entirely rather than merely mis-coloured.
[[nodiscard]] bool region_is_entirely_reference(const std::vector<uint8_t>& storage,
                                                 const std::vector<uint8_t>& reference, uint32_t width, uint32_t x0,
                                                 uint32_t x1, uint32_t y0, uint32_t y1) {
    for (uint32_t y = y0; y < y1; ++y) {
        for (uint32_t x = x0; x < x1; ++x) {
            const size_t idx = (static_cast<size_t>(y) * width + x) * 4;
            if (storage[idx] != reference[idx] || storage[idx + 1] != reference[idx + 1] ||
                storage[idx + 2] != reference[idx + 2]) {
                return false;
            }
        }
    }
    return true;
}

[[nodiscard]] size_t count_pixels_differing_from_reference(const std::vector<uint8_t>& storage,
                                                            const std::vector<uint8_t>& reference) {
    size_t count = 0;
    for (size_t idx = 0; idx + 4 <= storage.size(); idx += 4) {
        if (storage[idx] != reference[idx] || storage[idx + 1] != reference[idx + 1] ||
            storage[idx + 2] != reference[idx + 2]) {
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

    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_GT(count_pixels_differing_from_reference(storage, bg), 0u)
        << "the box should be visible against the background";
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
    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    RenderOptions shaded;
    shaded.mode = DrawMode::shaded;
    shaded.overlays = false;

    {
        const RenderScene scene = make_scene(make_single_triangle(/*reversed=*/false));
        std::vector<uint8_t> storage;
        RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
        render_or_fail(scene, camera, shaded, target);
        EXPECT_GT(count_pixels_differing_from_reference(storage, bg), 0u)
            << "a correctly-wound, outward-facing triangle must be visible in shaded mode";
    }
    {
        const RenderScene scene = make_scene(make_single_triangle(/*reversed=*/true));
        std::vector<uint8_t> storage;
        RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
        render_or_fail(scene, camera, shaded, target);
        EXPECT_EQ(count_pixels_differing_from_reference(storage, bg), 0u)
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

    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_GT(count_pixels_differing_from_reference(storage_forward, bg), 0u)
        << "wireframe mode must still draw something for a back-facing triangle";
}

// ===========================================================================
// 4. SR-11 submesh contract.
// ===========================================================================

TEST(RasterCpu, EmptySubmeshArraysUseTheDefaultMaterialAcrossTheWholeMesh) {
    RenderScene scene = make_scene(make_two_triangle_mesh());
    // unlit (shading=1u): this test is about the SUBMESH CONTRACT (SR-11),
    // not lighting -- an unlit material's base_color echoes through exactly,
    // decoupled from S7a Task R6's Lambert N.L/ambient term, which expected_bgr()
    // does not model.
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), .shading = 1u};
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

// ---------------------------------------------------------------------------
// A MESH'S OWN AUTHORED MATERIALS, THROUGH scene_from_world() AND ONTO PIXELS.
//
// The debt 88ebdc6a named in its own commit message and did not pay: it added
// MeshData::source_materials, taught the glTF loader to read baseColorFactor,
// and made scene_from_world() merge a file's palette into the scene's -- with
// no test at all, at any level.
//
// ⚠ THE CASE BELOW IS NOT THE ONE ABOVE IT. ExplicitSubmeshesEachKeepTheir-
// OwnMaterial hand-builds `scene.materials` and uses SCENE-palette indices; it
// proves the RASTERIZER honours submesh_material. This one gives the mesh a
// FILE-LOCAL palette and goes through scene_from_world(), which is the step
// that was untested -- and the step whose absence made a ten-submesh aircraft
// draw in ten of the ground's colours while every index stayed in range, so
// there was no fallback, no error, and nothing red.
//
// ⚠ THE WORLD'S PALETTE IS DELIBERATELY A COLOUR THE MESH NEVER AUTHORS.
// If the world's materials shared the mesh's, this test would pass whether the
// merge happened or not -- the two answers would coincide, which is exactly
// how the original defect stayed invisible. Green is the discriminator: a
// scene_from_world() that dropped the merge would leave file-local indices
// {0,1} pointing into the WORLD's palette and both triangles would render
// green, which the final two assertions forbid.
//
// SCOPE: this covers the merge and the rasterisation. A host's own draw-item
// construction for a vehicle is a separate seam and
// is not exercised here; the DrawItem below stands in for it, carrying
// kNoMaterial exactly as the vehicle path does, because a prop's
// material_override would REPLACE the submesh materials outright
// (raster_cpu.cpp:807) and so could never exercise them.
// ---------------------------------------------------------------------------

TEST(RasterCpu, AMeshsOwnAuthoredMaterialsSurviveSceneFromWorldAndReachTheFrame) {
    using spade::MaterialDesc;
    using spade::render::NamedMesh;
    using spade::render::scene_from_world;

    const glm::vec4 kGreen(0.0f, 1.0f, 0.0f, 1.0f);  // the WORLD's only colour
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), .shading = 1u};
    const Material blue{.base_color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), .shading = 1u};

    spade::WorldBuilder builder;
    // Capacities are REQUIRED -- validate_world_desc() refuses bodies == 0, and
    // the first version of this test found that out by failing with "world
    // capacity 'bodies' must be > 0" rather than by anyone reading the
    // validator. Same minimum test_render_scene.cpp's own base_builder() uses.
    builder.name("raster-material-merge-test")
        .capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1})
        .material(MaterialDesc{.name = "world_green",
                                .base_color = kGreen,
                                .shading = spade::MaterialShading::unlit});
    const spade::Result<spade::WorldDesc> world = builder.build();
    if (!world) {
        ADD_FAILURE() << "fixture world would not build: " << world.error().context;
        return;
    }

    // A file-local palette: submesh_material indexes source_materials, NOT the
    // scene. That is what a loaded glTF looks like.
    MeshData mesh = make_two_triangle_mesh();
    mesh.submesh_first_index = {0, 3};
    mesh.submesh_index_count = {3, 3};
    mesh.submesh_material = {0, 1};
    mesh.source_materials = {red, blue};

    const NamedMesh named{.ref = "mesh:test/two-tone", .mesh = mesh};
    spade::Result<RenderScene> built = scene_from_world(*world, std::span<const NamedMesh>(&named, 1));
    if (!built) {
        ADD_FAILURE() << "scene_from_world failed: " << built.error().context;
        return;
    }
    RenderScene scene = std::move(*built);

    ASSERT_EQ(scene.meshes.size(), 1u);
    // The merge, at the data level -- stated separately from the pixels so a
    // failure says WHICH half broke.
    ASSERT_GE(scene.materials.size(), 3u)
        << "the mesh's two authored materials should have been appended to the world's one";
    EXPECT_NE(scene.meshes[0].submesh_material[0], 0u)
        << "a file-local index 0 must have been rewritten past the world's palette, not left "
           "pointing at the world's material 0";

    scene.statics.push_back(DrawItem{.mesh_index = 0,
                                      .local_to_world = glm::mat4(1.0f),
                                      .material_override = spade::render::kNoMaterial});
    scene.bounds = Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    const auto green_bgr = expected_bgr(Material{.base_color = kGreen, .shading = 1u});

    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight,
                                     expected_bgr(red)))
        << "submesh 0 must render in the MESH's own first authored colour";
    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight,
                                     expected_bgr(blue)))
        << "submesh 1 must render in the MESH's own second authored colour";

    // The discriminators. Without the merge both triangles take the world's
    // palette and these two fail; with it, the world's colour appears nowhere
    // on the geometry at all.
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, green_bgr))
        << "submesh 0 rendered in the WORLD's colour -- the file's palette was not merged";
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight,
                                      green_bgr))
        << "submesh 1 rendered in the WORLD's colour -- the file's palette was not merged";
}

TEST(RasterCpu, ExplicitSubmeshesEachKeepTheirOwnMaterial) {
    MeshData mesh = make_two_triangle_mesh();
    mesh.submesh_first_index = {0, 3};
    mesh.submesh_index_count = {3, 3};
    mesh.submesh_material = {0, 1};

    RenderScene scene = make_scene(std::move(mesh));
    // unlit (shading=1u): SR-11 coverage, not lighting -- see the previous
    // test's identical note.
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), .shading = 1u};
    const Material blue{.base_color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), .shading = 1u};
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
    // unlit (shading=1u): this test is about material_override, not lighting
    // -- see EmptySubmeshArraysUseTheDefaultMaterialAcrossTheWholeMesh's
    // identical note.
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), .shading = 1u};
    const Material blue{.base_color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), .shading = 1u};
    const Material green{.base_color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), .shading = 1u};
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
    // unlit (shading=1u): this test is about submesh-range validation, not
    // lighting -- see EmptySubmeshArraysUseTheDefaultMaterialAcrossTheWholeMesh's
    // identical note.
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), .shading = 1u};
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
    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_TRUE(region_is_entirely_reference(storage, bg, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight))
        << "the right triangle's out-of-range submesh entry must never be drawn -- that half stays background";
}

TEST(RasterCpu, SubmeshRangeExceedingTheIndexBufferIsSkippedNotTheWholeMesh) {
    MeshData mesh = make_two_triangle_mesh();  // 6 indices total
    mesh.submesh_first_index = {0, 3};
    mesh.submesh_index_count = {3, 999};  // second submesh's range runs past indices.size()
    mesh.submesh_material = {0, 1};

    RenderScene scene = make_scene(std::move(mesh));
    // unlit (shading=1u): this test is about index-range validation, not
    // lighting -- see EmptySubmeshArraysUseTheDefaultMaterialAcrossTheWholeMesh's
    // identical note.
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), .shading = 1u};
    const Material blue{.base_color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), .shading = 1u};
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
    // unlit (shading=1u): this test is about index-range validation, not
    // lighting -- see EmptySubmeshArraysUseTheDefaultMaterialAcrossTheWholeMesh's
    // identical note.
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), .shading = 1u};
    scene.materials = {red};  // single implicit submesh (SR-11) -- both triangles would otherwise be red

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);  // must not dereference positions[9999]

    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected_bgr(red)))
        << "the left triangle's indices are untouched and should still draw";
    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_FALSE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight,
                                      expected_bgr(red)))
        << "the right triangle has an out-of-range index and must be skipped, not drawn";
    EXPECT_GT(count_pixels_differing_from_reference(storage, bg), 0u)
        << "the left triangle should still be visible somewhere";
}

// Review MINOR 6: draw_mesh_item's missing/truncated-normals skip (S7a Task
// R6) is NEW behaviour with no dedicated test -- every sibling defensive
// branch (mismatched submesh arrays, an out-of-range submesh range, an
// out-of-range vertex index) already has one. A mesh whose normals array is
// shorter than its positions/indices arrays rendered FLAT per-submesh colour
// before this task (normals were unread); it renders NOTHING for the
// affected triangle now (draw_mesh_item's own "skip, never crash" posture,
// matching how the pre-existing checks above already treat malformed data).
TEST(RasterCpu, MissingNormalsSkipsThatTriangleInShadedModeButWireframeStillDraws) {
    MeshData mesh = make_two_triangle_mesh();  // indices = {0,1,2, 3,4,5}; positions.size() == 6
    mesh.normals.resize(3);  // only the LEFT triangle's vertices (0,1,2) still have a normal

    // unlit (shading=1u): this test is about the missing-normals skip path,
    // not lighting -- see EmptySubmeshArraysUseTheDefaultMaterialAcrossTheWholeMesh's
    // identical note.
    const Material red{.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), .shading = 1u};
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));

    {
        RenderScene scene = make_scene(mesh);
        scene.materials = {red};
        RenderOptions options;
        options.mode = DrawMode::shaded;
        options.overlays = false;
        std::vector<uint8_t> storage;
        RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
        render_or_fail(scene, camera, options, target);  // must not read past mesh.normals

        EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected_bgr(red)))
            << "the left triangle has valid normals and must still draw in shaded mode";
        const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
        EXPECT_TRUE(
            region_is_entirely_reference(storage, bg, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight))
            << "the right triangle's missing normals must skip it entirely in shaded mode -- that half stays "
               "background, never drawn with a wrong or default normal";
    }
    {
        RenderScene scene = make_scene(mesh);
        scene.materials = {red};
        RenderOptions options;
        options.mode = DrawMode::wireframe;
        options.overlays = false;
        std::vector<uint8_t> storage;
        RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
        render_or_fail(scene, camera, options, target);

        EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth / 2, 0, kSmallHeight, expected_bgr(red)))
            << "the left triangle's wireframe edges must draw";
        EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, kSmallWidth / 2, kSmallWidth, 0, kSmallHeight,
                                         expected_bgr(red)))
            << "wireframe mode never needs a normal -- the right triangle's edges must still draw despite its "
               "missing normals, unlike the shaded-mode skip above";
    }
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

    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_EQ(count_pixels_differing_from_reference(storage, bg), 0u)
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
// 5b. Ruling SR-21 (S7a Task R7, controller amendment) -- the ground-grid-
// overlay vs tessellated-mesh z-fight carried since Task R3. None of the
// three RasterGolden.* fixtures below happen to contain an actual ground-
// plane MESH at the SAME height as the has_ground/ground_y overlay grid (see
// that section's own header comment) -- box/sphere/cylinder primitives never
// exercise this seam, so this is the dedicated regression coverage for the
// scenario the ticket actually describes: a real, flat, y=0 mesh occupying
// the WHOLE visible frame, coincident with the grid overlay drawn at that
// SAME y.
// ===========================================================================

namespace {

// A flat, single quad (2 triangles, +Y normal, CCW-outward -- same corner
// convention as make_box_mesh's own +Y face above) at y=0, deliberately
// larger than anything camera_top_down(..., height=4) can see at this file's
// kOverlayWidth/kOverlayHeight/default-fov (visible ground half-extent
// ~4*tan(30deg) ~= 2.3 world units) -- every visible ground pixel in the
// tests below is therefore guaranteed to be mesh-covered, not merely
// "beyond the mesh's edge, where the grid always shows freely regardless of
// any bias".
[[nodiscard]] MeshData make_flat_ground_quad(float half_extent) {
    MeshData mesh;
    const float h = half_extent;
    const glm::vec3 corners[4] = {{-h, 0.0f, h}, {h, 0.0f, h}, {h, 0.0f, -h}, {-h, 0.0f, -h}};
    for (const glm::vec3& c : corners) {
        mesh.positions.push_back(c);
        mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
    }
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return mesh;
}

}  // namespace

TEST(OverlayDepthBias, GridOverlayIsVisibleOverACoincidentGroundMeshNotHiddenByTheFirstWriterWinsTie) {
    RenderScene scene;
    // Unlit, and a colour with NOTHING in common with the grid's own
    // (90,90,90) -- so "found the grid colour" can only mean the overlay
    // line actually won a pixel, never a coincidental match with the mesh's
    // own shaded colour.
    scene.materials = {Material{.base_color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), .shading = 1u}};
    scene.meshes.push_back(make_flat_ground_quad(8.0f));
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});
    scene.has_ground = true;
    scene.ground_y = 0.0f;  // the OLDER grid-overlay heuristic -- SAME height as the mesh above
    scene.bounds = Aabb{.min = glm::vec3(-8.0f, -1.0f, -8.0f), .max = glm::vec3(8.0f, 1.0f, 8.0f)};

    const Camera camera = camera_top_down(glm::vec3(0.0f, 4.0f, 0.0001f));
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = true;
    options.shadows = false;  // isolates this ticket from R7's OTHER, unrelated feature

    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kOverlayWidth, kOverlayHeight);
    render_or_fail(scene, camera, options, target);

    const auto grid = bgr(90, 90, 90);
    const auto mesh_colour = bgr(0, 255, 0);
    size_t grid_pixels = 0;
    for (uint32_t y = 0; y < kOverlayHeight; ++y) {
        for (uint32_t x = 0; x < kOverlayWidth; ++x) {
            const size_t idx = (static_cast<size_t>(y) * kOverlayWidth + x) * 4;
            if (storage[idx] == grid[0] && storage[idx + 1] == grid[1] && storage[idx + 2] == grid[2]) {
                ++grid_pixels;
            }
        }
    }
    EXPECT_TRUE(region_contains_bgr(storage, kOverlayWidth, 0, kOverlayWidth, 0, kOverlayHeight, mesh_colour))
        << "sanity: the ground mesh itself must be visible somewhere in frame";
    // A COUNT, not merely "somewhere in frame", and a THRESHOLD picked from
    // measurement rather than assumption (this task's own "recompute before
    // you write it down" discipline): a grid line and a coincident mesh
    // surface at the SAME y do not resolve as one universal exact tie --
    // TWO GENUINELY DIFFERENT interpolation formulas (a line's own two-point
    // DDA lerp vs. a triangle's three-point barycentric weighting) round
    // differently pixel by pixel, so SOME grid pixels already win even with
    // NO bias at all. Measured directly on this exact fixture: 513 grid
    // pixels win with kOverlayDepthBias temporarily forced to 0.0, vs. 935
    // with the real, tuned value -- a reproducible +422 pixels (systematic,
    // not lucky-rounding noise) attributable to nothing but the bias. 700
    // sits with a wide margin on both sides of that measured gap.
    EXPECT_GT(grid_pixels, 700u)
        << "only " << grid_pixels << " grid-overlay pixels survived over the coincident ground mesh (measured "
           "513 with the bias forced to 0.0, 935 with it live) -- ruling SR-21's fix should recover most of the "
           "grid's own geometry, not leave it mostly hidden behind the mesh it annotates";
}

// ===========================================================================
// 6. DrawMode::raymarch is out of this task's scope; validate_target()
//    failures propagate.
// ===========================================================================

// S7a Task R8: DrawMode::raymarch is now implemented (render/raymarch.cpp)
// and no longer an error -- this is the regression guard for render()'s own
// DISPATCH decision (raster_cpu.cpp: "if (options.mode == DrawMode::raymarch)
// return render_raymarch(...)"), so a future revert of that one line trips
// this test immediately rather than being caught only indirectly by
// test_render_raymarch.cpp's own suite. make_scene() never sets `.sdf`
// (RenderScene::sdf defaults to nullptr, scene.hpp's own "non-owning, MAY BE
// NULL" contract), so this exercises the null-pointer case too -- succeeds
// with a sky-only frame, never a fault (raymarch.hpp's own header comment);
// test_render_raymarch.cpp's NullSdfPointerYieldsSkyOnlyNeverFaults is the
// exhaustive per-pixel version of that same claim.
TEST(RasterCpu, RaymarchModeIsHandledAndNoLongerAnError) {
    const RenderScene scene = make_scene(make_box_mesh(1.0f));
    ASSERT_EQ(scene.sdf, nullptr) << "sanity: make_scene() never points RenderScene::sdf at a program";
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.mode = DrawMode::raymarch;

    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);

    const Result<void> result = render(scene, camera, options, target);
    ASSERT_TRUE(result.has_value()) << "render() failed: " << result.error().context;

    // MN-14: every pixel's 4th (X) byte is opaque, unconditionally -- the
    // same invariant every other DrawMode upholds.
    for (uint32_t y = 0; y < kSmallHeight; ++y) {
        for (uint32_t x = 0; x < kSmallWidth; ++x) {
            const size_t idx = (static_cast<size_t>(y) * kSmallWidth + x) * 4;
            ASSERT_EQ(storage[idx + 3], 0xFFu) << "(" << x << "," << y << ") X byte must be opaque";
        }
    }
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

// Golden D (S7a Task R7, review IMPORTANT I5): a real, tessellated ground
// plane (not merely the analytic background -- SR-11's own submesh
// fallback puts it at material 0) with a box caster floating above it, its
// shadow baked via build_static_shadow_map() and sampled through render()'s
// own shadow gate -- the first byte-exact anchor for the shadow feature,
// since none of Goldens A-C populate RenderScene::static_shadow at all (see
// this manifest's own _changelog for why the shadow feature moved none of
// them). Verified sensitive, not assumed: forcing RenderOptions::shadows to
// false renders a DIFFERENT frame (see this test's own temporary
// verification block, removed once confirmed against the committed hash).
//
// Overlays are on, and the tessellated ground grid's finite extent
// (kGoldenTessellationBounds, +-5) DOES sit at the SAME height as the
// has_ground/ground_y overlay grid (+-10) -- but this golden does NOT, it
// turns out, additionally anchor ruling SR-21's overlay-depth-bias fix:
// checked directly by forcing kOverlayDepthBias to 0.0 and re-running this
// exact test, and the hash did not move, the same outcome Goldens A-C
// already have for the identical reason described in the ticket's own text
// (the pixels where the grid and this mesh compete resolve as a clean win
// for the mesh either way, at this camera's own framing, not a near-tie).
// The dedicated OverlayDepthBias.* regression test above remains the sole,
// mutation-verified anchor for that fix -- recorded here rather than left
// as an unverified assumption once this golden was built specifically to
// try to double as one.
[[nodiscard]] RenderScene golden_scene_shadowed_ground_with_caster() {
    RenderScene scene;
    scene.meshes.push_back(tessellate_or_fail(SdfPrim::plane, glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));  // [0] ground
    scene.meshes.push_back(tessellate_or_fail(SdfPrim::box, glm::vec4(0.8f, 0.8f, 0.8f, 0.0f)));     // [1] caster
    scene.materials = {
        Material{.base_color = glm::vec4(0.55f, 0.5f, 0.45f, 1.0f)},  // ground -- lambert (shading defaults to 0)
        Material{.base_color = glm::vec4(0.75f, 0.3f, 0.25f, 1.0f)},  // caster -- lambert, a different colour
    };
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});
    scene.statics.push_back(DrawItem{
        .mesh_index = 1, .local_to_world = pose_at(glm::vec3(0.0f, 2.5f, 0.0f)), .material_override = 1});
    // Ruling SR-17: alongside the tessellated grid above, not instead of it
    // -- scene_from_world()'s own contract for a standalone plane primitive
    // (scene.cpp), reproduced by hand here since this fixture never goes
    // through scene_from_world() itself.
    scene.ground_planes.push_back(GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = 0.0f, .material = 0});
    scene.has_ground = true;  // the OLDER grid-overlay heuristic -- SAME y=0 height as the ground mesh above
    scene.ground_y = 0.0f;
    scene.bounds = Aabb{.min = glm::vec3(-5.0f, -1.0f, -5.0f), .max = glm::vec3(5.0f, 4.0f, 5.0f)};
    const Result<ShadowMap> shadow_map = build_static_shadow_map(scene);
    if (!shadow_map) {
        ADD_FAILURE() << "build_static_shadow_map failed: " << shadow_map.error().context;
        return scene;
    }
    scene.static_shadow = *shadow_map;
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
    const auto bg = render_background_only(camera, kGoldenWidth, kGoldenHeight);
    ASSERT_GT(count_pixels_differing_from_reference(pixels, bg), 0u)
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
    const auto bg = render_background_only(camera, kGoldenWidth, kGoldenHeight);
    ASSERT_GT(count_pixels_differing_from_reference(pixels, bg), 0u)
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
    const auto bg = render_background_only(camera, kGoldenWidth, kGoldenHeight);
    ASSERT_GT(count_pixels_differing_from_reference(pixels, bg), 0u)
        << "sanity floor: this scene/camera must actually show something before its hash means anything";
    check_against_manifest("cylinder_static_dynamic_box_top_down", pixels);
}

// SR-17a's atmospheric term (03-world-and-render.md section 16, amended to ALL
// GEOMETRY AT RANGE by user ruling 2026-09-17), pinned across platforms.
//
// ⚠⚠ THIS GOLDEN IS A PAIR, NOT A FRAME, AND THE PAIRING IS THE WHOLE POINT.
// The question to ask of any golden whose subject is a FEATURE is "what would
// this still pass on?" -- and a lone "term ON" frame would still pass if the
// term's strength were silently zero, because a sha256 only says the pipeline
// produced the bytes it produced last time. That golden would prove the
// renderer ran. It would be a fifth CONTROL wearing a feature's name.
//
// So this test renders the SAME scene and camera twice and pins both ends:
// the off arm must reproduce the committed `shadowed_ground_with_caster` hash
// that has been in this manifest since before SR-17a existed, and the on arm
// must reproduce its own -- AND THE TWO MUST DIFFER. A term that stopped
// applying collapses the on arm onto the off arm, and the inequality assertion
// fires before either hash is even consulted.
TEST(RasterGolden, AtmosphericTermOnAndOffAreTwoPinnedFramesThatMustDifferRulingSR17a) {
    const RenderScene scene = golden_scene_shadowed_ground_with_caster();
    ASSERT_TRUE(scene.static_shadow.has_value());
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.8f, 8.0f));

    RenderOptions off;
    off.mode = DrawMode::shaded;
    off.overlays = true;
    off.shadows = true;  // the engine default strength is 0, so this IS the pre-SR-17a frame

    RenderOptions on = off;
    // A real setting rather than the 45 m shipped default: at 45 m this scene's
    // own depth range sits in the fade's toe and the two frames would differ by
    // a handful of bytes, which is a weak thing to pin.
    on.horizon_blend_strength = 0.75f;
    on.horizon_blend_onset = 20.0f;

    const std::vector<uint8_t> pixels_off = render_golden(scene, camera, off);
    const std::vector<uint8_t> pixels_on = render_golden(scene, camera, on);

    // FIRST, before any hash: the term must have done something. This is the
    // assertion that makes the pair a pair.
    size_t differing = 0;
    ASSERT_EQ(pixels_off.size(), pixels_on.size());
    for (size_t i = 0; i < pixels_off.size(); ++i) {
        if (pixels_off[i] != pixels_on[i]) ++differing;
    }
    ASSERT_GT(differing, 1000u)
        << "SR-17a changed only " << differing << " bytes of this frame -- the golden below would be pinning a "
           "frame the term did not affect, i.e. a fifth control rather than a feature";

    // The off arm must still be the frame this manifest has always held. It is
    // re-derived here rather than assumed, so that a change to the term which
    // accidentally perturbed the strength-0 path is caught HERE, attached to
    // the term, and not only in the older test that has no idea SR-17a exists.
    check_against_manifest("shadowed_ground_with_caster", pixels_off);
    check_against_manifest("shadowed_ground_with_caster_atmospheric", pixels_on);
}

TEST(RasterGolden, ShadowedGroundWithCasterMatchesCommittedManifest) {
    const RenderScene scene = golden_scene_shadowed_ground_with_caster();
    ASSERT_TRUE(scene.static_shadow.has_value());
    // Below the box caster's underside (y=2.5-0.8=1.7) so its own shadow is
    // not self-occluded from the camera's own line of sight -- the SAME
    // "look under the floating object" placement task-R7-brief.md's own
    // ShadowStep1 fixture (test_render_shadow.cpp) uses.
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.8f, 8.0f));
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = true;
    options.shadows = true;

    const std::vector<uint8_t> pixels = render_golden(scene, camera, options);
    const auto bg = render_background_only(camera, kGoldenWidth, kGoldenHeight);
    ASSERT_GT(count_pixels_differing_from_reference(pixels, bg), 0u)
        << "sanity floor: this scene/camera must actually show something before its hash means anything";

    // Kept as a PERMANENT guard, not just a one-time check at authoring
    // time (I5's own point: a golden that happens to pass is not the same
    // as a golden that is actually anchoring the feature it was added
    // for) -- shadows=false must render something DIFFERENT, so a future
    // regression that silently breaks render()'s own shadow gate (e.g. the
    // `options.mode == DrawMode::shaded` or `scene.static_shadow.has_value()`
    // conditions) cannot hide behind an unrelated hash staying green.
    {
        RenderOptions no_shadows = options;
        no_shadows.shadows = false;
        const std::vector<uint8_t> pixels_no_shadow = render_golden(scene, camera, no_shadows);
        EXPECT_NE(pixels, pixels_no_shadow) << "this golden must be sensitive to RenderOptions::shadows";
    }

    check_against_manifest("shadowed_ground_with_caster", pixels);
}

// ===========================================================================
// 8. Task R5b (controller ruling SR-15): exact near/far triangle clipping.
// Before this task, draw_world_triangle and draw_mesh_triangle_shaded both
// rejected a triangle OUTRIGHT the instant any one vertex fell outside the
// [near, far] camera-space slab -- a camera approaching a wall made that
// wall's triangles vanish one by one as they straddled the near plane,
// rather than showing the part still in view. Every render() call below is
// a black-box call into the real renderer -- none of these tests reach into
// raster_cpu.cpp's anonymous-namespace internals -- so the geometry is
// engineered so a straddling triangle's outcome (empty vs. non-empty,
// or its exact silhouette) is knowable independently of the implementation.
// ===========================================================================

namespace {

// Local, independent (never calls into raster_cpu.cpp) re-derivation of the
// camera-space arithmetic needed to predict the EXACT clipped-and-projected
// triangle for RasterCpu.NearPlaneClipIsGeometricallyExact below -- the same
// "second source" posture as this file's own channel_to_byte/expected_bgr.
struct Cam3 {
    double x = 0.0, y = 0.0, z = 0.0;
};
[[nodiscard]] Cam3 operator+(const Cam3& a, const Cam3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
[[nodiscard]] Cam3 operator-(const Cam3& a, const Cam3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
[[nodiscard]] Cam3 operator*(const Cam3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }

// Independently re-derives clipSegmentToHalfSpace's own interpolation
// formula (task-R5b-brief.md: t = (planeZ - p0.z) / (p1.z - p0.z),
// p0 + (p1 - p0) * t) for the one crossing this test needs.
[[nodiscard]] Cam3 lerp_to_plane(Cam3 p0, Cam3 p1, double planeZ) {
    const double t = (planeZ - p0.z) / (p1.z - p0.z);
    return p0 + (p1 - p0) * t;
}

struct Px {
    double x = 0.0, y = 0.0;
};

// Independently re-derives projectCameraSpace's own tan32() -- same two
// steps (narrow to float, sin32/cos32) -- by calling spade::math::sin32/
// cos32 DIRECTLY rather than re-deriving the ratio some other way. These are
// the engine's own approved deterministic replacements, not libm (this file
// is scanned by BitPortability.NoLibmTranscendentalInEngineOrGoldenTestSource,
// per this program's own history of libm-portability defects: neither
// "sin32(" nor "cos32(" is on that scan's forbidden-pattern list). Calling
// the SAME functions production calls -- rather than assuming a "nice" FOV
// makes the ratio an exact constant -- keeps this oracle bit-for-bit
// identical to production's own f, which matters right at a clipped
// triangle's vertex tips where even a handful of ULPs can flip a pixel.
[[nodiscard]] double tan32_oracle(double half_fov_rad) {
    const float x = static_cast<float>(half_fov_rad);
    return static_cast<double>(spade::math::sin32(x)) / static_cast<double>(spade::math::cos32(x));
}

// Independently re-derives projectCameraSpace's perspective-divide formula
// for a camera at the world origin with identity orientation (world space
// IS camera space here -- no rotation/translation to account for).
[[nodiscard]] Px project_camera_space_oracle(Cam3 pc, double fov_y_radians, double width, double height) {
    const double f = 1.0 / tan32_oracle(fov_y_radians * 0.5);
    const double aspect = width / height;
    const double invNegZ = 1.0 / (-pc.z);
    const double xNdc = (f / aspect) * pc.x * invNegZ;
    const double yNdc = f * pc.y * invNegZ;
    return {(xNdc * 0.5 + 0.5) * width, (1.0 - (yNdc * 0.5 + 0.5)) * height};
}

// Independently re-derives rasterizeTriangleFlat's own fill rule (both
// windings count as inside a pixel whose center is `p`).
[[nodiscard]] bool inside_triangle(Px v0, Px v1, Px v2, Px p) {
    const auto edge = [](Px a, Px b, Px c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); };
    const double w0 = edge(v1, v2, p), w1 = edge(v2, v0, p), w2 = edge(v0, v1, p);
    return (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
}

// R5b review round 1, IMPORTANT 1: general (not per-shape hand-derived)
// independent re-derivation of clipPolygonToHalfSpace's own Sutherland-
// Hodgman pass -- never calls into raster_cpu.cpp -- used by the two new
// exactness tests below that exercise the 4-vertex (one plane) and 5-vertex
// (both planes) clip outputs T7 above does not reach (T7's clip result is
// exactly 3 vertices, so its fan loop runs its body exactly once and cannot
// distinguish a correct fan from several plausible wrong ones).
[[nodiscard]] std::vector<Cam3> clip_polygon_oracle(const std::vector<Cam3>& poly, double planeZ, bool keepLessEq) {
    if (poly.empty()) {
        return {};
    }
    const auto inside = [&](const Cam3& p) { return keepLessEq ? (p.z <= planeZ) : (p.z >= planeZ); };
    std::vector<Cam3> out;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Cam3& cur = poly[i];
        const Cam3& next = poly[(i + 1) % poly.size()];
        const bool curIn = inside(cur);
        const bool nextIn = inside(next);
        if (curIn) {
            out.push_back(cur);
        }
        if (curIn != nextIn) {
            out.push_back(lerp_to_plane(cur, next, planeZ));
        }
    }
    return out;
}

// True iff pixel-center `p` falls inside ANY of the fan triangles
// (screenPoly[0], screenPoly[i], screenPoly[i+1]) for i in [1, size-2] --
// the SAME fan pattern draw_mesh_triangle_shaded/draw_world_triangle use
// over their own clipped-and-projected polygon. A wrong fan (a sliding
// window, or emitting only the first triangle) produces a DIFFERENT set of
// sub-triangles here than production's, so a pixel this oracle says is
// covered by the CORRECT fan but a wrong fan would miss (or vice versa) is
// exactly what catches those mutants.
[[nodiscard]] bool fan_covers_pixel(const std::vector<Px>& screenPoly, Px p) {
    for (size_t i = 1; i + 1 < screenPoly.size(); ++i) {
        if (inside_triangle(screenPoly[0], screenPoly[i], screenPoly[i + 1], p)) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST(RasterCpu, TriangleStraddlingNearPlaneRendersNonEmptyPixels) {
    // One vertex (z=+0.3) is behind the near plane (default 0.1); the other
    // two (z=-6) are in front. Camera at the world origin, identity
    // orientation, so world space IS camera space and these z values ARE
    // the near/far test's own input.
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 0.0f));
    MeshData mesh;
    mesh.positions = {glm::vec3(-1.0f, -1.0f, -6.0f), glm::vec3(1.0f, -1.0f, -6.0f), glm::vec3(0.0f, 1.0f, 0.3f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2};  // front-facing: 2D cross of (p1-p0),(p2-p0) using x,y only is +4

    const RenderScene scene = make_scene(mesh);
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_GT(count_pixels_differing_from_reference(storage, bg), 0u)
        << "exact clipping must draw the surviving portion instead of discarding the whole triangle -- this "
           "rendered ZERO pixels under the pre-fix whole-triangle near/far rejection";
}

TEST(RasterCpu, TriangleStraddlingFarPlaneRendersNonEmptyPixels) {
    // The symmetric far-plane case: one vertex (z=-11) is beyond a reduced
    // far plane (10); the other two (z=-9) are in front. Same shape as the
    // near-plane case above, shifted in depth.
    Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 0.0f));
    camera.far_plane = 10.0f;
    MeshData mesh;
    mesh.positions = {glm::vec3(-1.0f, -1.0f, -9.0f), glm::vec3(1.0f, -1.0f, -9.0f), glm::vec3(0.0f, 1.0f, -11.0f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2};  // front-facing, same as the near-plane case

    const RenderScene scene = make_scene(mesh);
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_GT(count_pixels_differing_from_reference(storage, bg), 0u)
        << "one vertex is beyond the far plane, two are in front -- exact clipping must still draw the "
           "surviving portion (this rendered ZERO pixels under the pre-fix whole-triangle rejection)";
}

TEST(RasterCpu, SpawnMarkerOverlaySurvivesNearPlaneStraddle) {
    // draw_world_triangle (the overlay/marker path) shares the SAME pre-fix
    // whole-triangle rejection bug as draw_mesh_triangle_shaded -- SR-15's
    // own text warns that fixing only the mesh path would leave spawn/body
    // markers vanishing at close range while real geometry survives. This
    // spawn marker sits close enough to the camera (base z=-0.05) that, of
    // its two triangles (draw_spawn_markers' own v0/v1/v2/v3, radius 0.3,
    // y-offset 0.02): triangle (v0,v1,v2) has all three vertices at
    // z in {-0.05, +0.25, -0.05} -- every one already past the default
    // near_plane=0.1 -- so it is WHOLLY outside and correctly draws nothing
    // either before or after this fix; triangle (v0,v2,v3) has v0,v2 at
    // z=-0.05 (outside) and v3 at z=-0.35 (inside) -- THIS is the one that
    // straddles. Before this fix, (v0,v2,v3) was also rejected outright by
    // the whole-triangle rejection, so the entire marker was invisible;
    // after it, (v0,v2,v3)'s surviving sliver is what this test looks for.
    RenderScene scene = make_empty_overlay_scene();
    scene.spawn_positions = {glm::vec3(0.0f, 0.0f, -0.05f)};
    scene.spawn_orientations = {glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 0.0f));  // at the origin: world == camera space
    RenderOptions options;
    options.overlays = true;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    const auto spawn = bgr(190, 90, 170);
    EXPECT_TRUE(region_contains_bgr(storage, kSmallWidth, 0, kSmallWidth, 0, kSmallHeight, spawn))
        << "the spawn marker's near-straddling triangle must still contribute visible pixels -- this rendered "
           "no spawn-colored pixels at all under the pre-fix whole-triangle rejection";
}

TEST(RasterCpu, TriangleEntirelyBehindNearPlaneRendersNothing) {
    // The trivial-reject path must survive exact clipping: a triangle with
    // EVERY vertex outside the slab must still render nothing.
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 0.0f));  // make_single_triangle's z=+1 is now entirely behind
    const RenderScene scene = make_scene(make_single_triangle(/*reversed=*/false));
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_EQ(count_pixels_differing_from_reference(storage, bg), 0u)
        << "a triangle entirely behind the near plane must render nothing after exact clipping too";
}

TEST(RasterCpu, TriangleEntirelyBeyondFarPlaneRendersNothing) {
    Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 0.0f));
    camera.far_plane = 5.0f;
    MeshData mesh;
    mesh.positions = {glm::vec3(1.0f, -1.0f, -10.0f), glm::vec3(1.0f, 1.0f, -10.0f), glm::vec3(-1.0f, 1.0f, -10.0f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2};  // front-facing, same shape/winding as make_single_triangle(false)

    const RenderScene scene = make_scene(mesh);
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_EQ(count_pixels_differing_from_reference(storage, bg), 0u)
        << "a triangle entirely beyond the far plane must render nothing after exact clipping too";
}

TEST(RasterCpu, BackFacingTriangleStraddlingNearPlaneStaysCulledAfterClipping) {
    // The SAME straddling geometry as TriangleStraddlingNearPlaneRendersNon-
    // EmptyPixels above, but reversed winding (back-facing). Clipping must
    // not become a way for a back face to leak through the SR-13 cull.
    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 0.0f));
    MeshData mesh;
    mesh.positions = {glm::vec3(-1.0f, -1.0f, -6.0f), glm::vec3(1.0f, -1.0f, -6.0f), glm::vec3(0.0f, 1.0f, 0.3f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 2, 1};  // reversed vs. the front-facing case -- back-facing

    const RenderScene scene = make_scene(mesh);
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kSmallWidth, kSmallHeight);
    render_or_fail(scene, camera, options, target);

    const auto bg = render_background_only(camera, kSmallWidth, kSmallHeight);
    EXPECT_EQ(count_pixels_differing_from_reference(storage, bg), 0u)
        << "clipping a straddling triangle must not become a way for a back-facing triangle to leak through "
           "the SR-13 cull (fan triangulation preserves the source triangle's facing)";
}

TEST(RasterCpu, NearPlaneClipIsGeometricallyExactNotJustNonEmpty) {
    // A wedge: P0/P1 share depth z=-2 (an edge parallel to the near plane);
    // apex P2 is farther away at z=-6. Camera at the world origin, identity
    // orientation (world space IS camera space, so no worldToCameraSpace
    // call is needed either).
    //
    // Review round 1, MINOR 2 (reworded in round 2 -- the original comment
    // here, and the dispatch that asked for this nudge, both overclaimed):
    // the apex's X was originally an exact 3.0, which (combined with
    // P0/P1's exact +-1.0 Y and the near_plane=4.1 used below) put pixel
    // centers exactly on the clipped triangle's slanted edges (px - py and
    // px + py are always integers at a pixel center (px+0.5, py+0.5), and
    // those edges had slope exactly +-1 with an integer intercept) -- the
    // review's own from-scratch re-derivation counted 46 such centers. A
    // real degeneracy, but only in EXACT (infinite-precision) arithmetic,
    // at f=1 exactly. It was never a LIVE failure mode: production's actual
    // f (from tan32/sin32/cos32) differs from 1 by ~1e-7, displacing those
    // 46 centers by ~1e-5 px -- roughly 10^9 ULPs beyond what any
    // reassociating-but-mathematically-neutral refactor of
    // projectCameraSpace (hoisting f/aspect, an FMA contraction, a
    // reordered multiply) could move a result by. Verified directly: such a
    // refactor mutant leaves all 28 rasterizer tests green, goldens
    // included. Only a change to tan32/sin32/cos32 itself could move these
    // pixels -- and then this test's own oracle (which calls those same
    // functions, not a hardcoded ratio) would move identically. Nudging the
    // apex's X to 3.1 removes the exact-arithmetic degeneracy anyway, as
    // hygiene: it costs nothing, and it is one fewer thing to reason about
    // if this geometry is ever reused verbatim in a context where f could
    // legitimately be exactly rational (e.g. a hand-computed oracle that
    // does not call the real trig functions). It changes nothing about
    // which vertices the near plane clips -- that depends only on Z.
    constexpr uint32_t kDim = 400;  // square target: aspect = 1 exactly
    // Declared once, as a float32, and reused (cast to double) for the
    // oracle's own p2 below -- mesh.positions is glm::vec3 (float), and
    // production widens it to double via vec3d()'s static_cast<double>, so
    // the oracle must widen the SAME float32 value rather than an
    // independently-typed double literal (a repeat, at the last ULP, of
    // this file's earlier hardcoded-tan(45deg) lesson: 3.1f and the double
    // literal 3.1 are not the same number).
    constexpr float kApexX = 3.1f;
    Camera camera;
    camera.position = glm::vec3(0.0f);
    camera.fov_y_radians = 1.5707963267948966f;  // pi/2 = 90 degrees

    // Indices {0,2,1} order the triangle (P0,P2,P1) CCW as seen from the
    // camera (front-facing: 2D cross of (P2-P0),(P1-P0) using x,y only is
    // +6.2) -- shaded mode must not cull it, before or after clipping.
    MeshData mesh;
    mesh.positions = {glm::vec3(0.0f, -1.0f, -2.0f), glm::vec3(0.0f, 1.0f, -2.0f), glm::vec3(kApexX, 0.0f, -6.0f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 2, 1};

    RenderScene scene = make_scene(mesh);
    // unlit (shading=1u): this test is a pixel-exact clipping/projection
    // oracle, not a lighting one -- an unlit material's base_color echoes
    // through exactly, decoupled from S7a Task R6's Lambert N.L/ambient term
    // (every vertex here shares the same constant normal, so the Gouraud
    // fast path (rasterizeTriangleGouraud) would flat-fill regardless, but
    // WITH whatever the lighting term scaled it to -- unlit sidesteps needing
    // to also reproduce that term in this file's own expected_bgr() oracle).
    const Material yellow{.base_color = glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), .shading = 1u};
    scene.materials = {yellow};
    const auto expected = expected_bgr(yellow);

    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;

    // Frame A: near_plane (0.1, default) is below both depths -- the FULL
    // triangle is inside the slab, unclipped. Sanity baseline only.
    camera.near_plane = 0.1f;
    std::vector<uint8_t> storage_full;
    RenderTarget target_full = make_target(storage_full, kDim, kDim);
    render_or_fail(scene, camera, options, target_full);
    const auto bg = render_background_only(camera, kDim, kDim);
    ASSERT_GT(count_pixels_differing_from_reference(storage_full, bg), 0u)
        << "sanity: the full triangle must be visible";

    // Frame B: SAME camera position/orientation/fov -- so the world->screen
    // mapping is IDENTICAL to Frame A's; projectCameraSpace never reads
    // near/far -- but near_plane = 4.1, strictly between the two depths.
    // P0/P1 (z=-2) now fall outside the valid slab and P2 (z=-6) stays
    // inside: exactly a "two behind, one in front" straddle.
    camera.near_plane = 4.1f;
    std::vector<uint8_t> storage_clipped;
    RenderTarget target_clipped = make_target(storage_clipped, kDim, kDim);
    render_or_fail(scene, camera, options, target_clipped);

    // Independently re-derive the exact clipped triangle -- camera space ==
    // world space here, so no worldToCameraSpace call is needed, just the
    // two crossing edges (P0-P2) and (P2-P1), via the SAME interpolation
    // formula clipSegmentToHalfSpace already uses.
    const Cam3 p0{0.0, -1.0, -2.0}, p1{0.0, 1.0, -2.0}, p2{static_cast<double>(kApexX), 0.0, -6.0};
    const double planeZ = -static_cast<double>(camera.near_plane);  // -4.1
    const Cam3 i1 = lerp_to_plane(p0, p2, planeZ);
    const Cam3 i2 = lerp_to_plane(p2, p1, planeZ);
    const double fov = static_cast<double>(camera.fov_y_radians);
    const Px v0 = project_camera_space_oracle(i1, fov, kDim, kDim);
    const Px v1 = project_camera_space_oracle(p2, fov, kDim, kDim);
    const Px v2 = project_camera_space_oracle(i2, fov, kDim, kDim);

    size_t expected_count = 0;
    for (uint32_t y = 0; y < kDim; ++y) {
        for (uint32_t x = 0; x < kDim; ++x) {
            const bool want = inside_triangle(v0, v1, v2, Px{x + 0.5, y + 0.5});
            const size_t idx = (static_cast<size_t>(y) * kDim + x) * 4;
            const bool got = storage_clipped[idx] == expected[0] && storage_clipped[idx + 1] == expected[1] &&
                              storage_clipped[idx + 2] == expected[2];
            ASSERT_EQ(got, want) << "pixel (" << x << "," << y
                                  << ") disagrees with the analytically clipped triangle -- clipping must be "
                                     "geometrically exact, not merely non-empty";
            if (want) {
                ++expected_count;
            }
        }
    }
    EXPECT_GT(expected_count, 0u) << "the analytic reference triangle itself must be non-degenerate";

    // The straddling frame's visible geometry must be a subset of the
    // fully-inside frame's -- same camera/projection in both, so clipping
    // only ever REMOVES silhouette, never relocates or adds to it.
    for (uint32_t y = 0; y < kDim; ++y) {
        for (uint32_t x = 0; x < kDim; ++x) {
            const size_t idx = (static_cast<size_t>(y) * kDim + x) * 4;
            const bool clipped_pixel_is_material =
                storage_clipped[idx] == expected[0] && storage_clipped[idx + 1] == expected[1] &&
                storage_clipped[idx + 2] == expected[2];
            if (!clipped_pixel_is_material) {
                continue;
            }
            EXPECT_TRUE(storage_full[idx] == expected[0] && storage_full[idx + 1] == expected[1] &&
                        storage_full[idx + 2] == expected[2])
                << "(" << x << "," << y << ") is material-colored in the clipped frame but not in the "
                   "fully-inside frame -- same camera/projection, so this can only mean the clip leaked "
                   "geometry outside the original triangle's silhouette";
        }
    }
}

// R5b review round 1, IMPORTANT 1: T7 above only ever produces a 3-vertex
// clip result (P0/P1 outside, P2 inside), so its fan loop body runs exactly
// once and cannot distinguish a correct fan from a wrong one. This test's
// clip produces exactly 4 vertices instead -- P0,P1 both survive unclipped,
// P2 is the ONLY vertex clipped away, replaced by two crossing points -- so
// the fan loop runs TWICE, over (P0,P1,X1) and (P0,X1,X2), matching the
// "[P0, P1, X1, X2]" topology the review specifically named.
TEST(RasterCpu, NearPlaneClipQuadCaseIsGeometricallyExact) {
    constexpr uint32_t kDim = 400;
    Camera camera;
    camera.position = glm::vec3(0.0f);
    camera.fov_y_radians = 1.5707963267948966f;  // pi/2 = 90 degrees
    camera.near_plane = 2.5f;

    // All coordinates are exact in float32 (multiples of 0.25), so the
    // oracle's double literals below are bit-identical to what
    // mesh.positions (glm::vec3, float) actually carries once widened --
    // no float/double mismatch to guard against here (see T7's kApexX
    // comment above for why that matters). That same exactness cuts both
    // ways, though (review round 2): it is also precisely what puts 4 pixel
    // centers exactly on the clipped quad's boundary at f=1 -- the reason
    // the measured count below is 6395 rather than 6399, not a rounding
    // slip. Like T7, this is a real degeneracy only in exact arithmetic,
    // not a live failure mode (production's f differs from 1 by ~1e-7). The
    // 5-vertex test below, whose inputs are NOT this clean, has zero
    // on-boundary centers and is the more robust of the two for that
    // reason.
    const Cam3 p0{-1.0, -1.5, -3.0}, p1{1.5, -0.5, -3.0}, p2{0.25, 1.75, -1.0};
    MeshData mesh;
    mesh.positions = {glm::vec3(-1.0f, -1.5f, -3.0f), glm::vec3(1.5f, -0.5f, -3.0f), glm::vec3(0.25f, 1.75f, -1.0f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2};  // front-facing: 2D cross of (p1-p0),(p2-p0) using x,y only is +6.875

    RenderScene scene = make_scene(mesh);
    // unlit (shading=1u): a pixel-exact clipping oracle, not a lighting one
    // -- see T7's (NearPlaneClipIsGeometricallyExactNotJustNonEmpty) identical note.
    const Material cyan{.base_color = glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), .shading = 1u};
    scene.materials = {cyan};
    const auto expected = expected_bgr(cyan);

    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kDim, kDim);
    render_or_fail(scene, camera, options, target);

    // Independent oracle: clip against near (P2 is the only vertex outside)
    // then far (a no-op here -- both surviving depths are well inside the
    // default far_plane=1000), then project via the same formula
    // production uses.
    const double nearZ = -static_cast<double>(camera.near_plane);
    const double farZ = -static_cast<double>(camera.far_plane);
    std::vector<Cam3> poly = clip_polygon_oracle({p0, p1, p2}, nearZ, /*keepLessEq=*/true);
    poly = clip_polygon_oracle(poly, farZ, /*keepLessEq=*/false);
    ASSERT_EQ(poly.size(), 4u) << "sanity: this geometry must produce the 4-vertex case under test";

    const double fov = static_cast<double>(camera.fov_y_radians);
    std::vector<Px> screenPoly;
    for (const Cam3& v : poly) {
        screenPoly.push_back(project_camera_space_oracle(v, fov, kDim, kDim));
    }

    size_t expected_count = 0;
    for (uint32_t y = 0; y < kDim; ++y) {
        for (uint32_t x = 0; x < kDim; ++x) {
            const bool want = fan_covers_pixel(screenPoly, Px{x + 0.5, y + 0.5});
            const size_t idx = (static_cast<size_t>(y) * kDim + x) * 4;
            const bool got = storage[idx] == expected[0] && storage[idx + 1] == expected[1] &&
                              storage[idx + 2] == expected[2];
            ASSERT_EQ(got, want) << "pixel (" << x << "," << y
                                  << ") disagrees with the analytically clipped 4-vertex polygon's fan "
                                     "triangulation (P0,P1,X1) + (P0,X1,X2)";
            if (want) {
                ++expected_count;
            }
        }
    }
    EXPECT_GT(expected_count, 0u) << "the analytic reference polygon itself must be non-degenerate";
}

// R5b review round 1, IMPORTANT 1: the 5-vertex case (a triangle straddling
// BOTH the near and the far plane at once) was exercised by no test at all.
// P0 is too close (outside near), P1 is too far (outside far), P2 is inside
// both -- clipping near-then-far leaves 5 vertices and a 3-triangle fan.
TEST(RasterCpu, BothPlanesClipFiveVertexCaseIsGeometricallyExact) {
    constexpr uint32_t kDim = 400;
    Camera camera;
    camera.position = glm::vec3(0.0f);
    camera.fov_y_radians = 1.5707963267948966f;  // pi/2 = 90 degrees
    camera.near_plane = 1.0f;
    camera.far_plane = 10.0f;

    // All coordinates are exact in float32 (multiples of 0.25) -- same
    // float/double-consistency reasoning as the quad-case test above.
    const Cam3 p0{0.5, -1.25, -0.5}, p1{2.75, 0.5, -15.0}, p2{-1.25, 1.5, -5.0};
    MeshData mesh;
    mesh.positions = {glm::vec3(0.5f, -1.25f, -0.5f), glm::vec3(2.75f, 0.5f, -15.0f), glm::vec3(-1.25f, 1.5f, -5.0f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2};  // front-facing: 2D cross of (p1-p0),(p2-p0) using x,y only is +9.25

    RenderScene scene = make_scene(mesh);
    // unlit (shading=1u): a pixel-exact clipping oracle, not a lighting one
    // -- see T7's (NearPlaneClipIsGeometricallyExactNotJustNonEmpty) identical note.
    const Material magenta{.base_color = glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), .shading = 1u};
    scene.materials = {magenta};
    const auto expected = expected_bgr(magenta);

    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kDim, kDim);
    render_or_fail(scene, camera, options, target);

    const double nearZ = -static_cast<double>(camera.near_plane);
    const double farZ = -static_cast<double>(camera.far_plane);
    std::vector<Cam3> poly = clip_polygon_oracle({p0, p1, p2}, nearZ, /*keepLessEq=*/true);
    poly = clip_polygon_oracle(poly, farZ, /*keepLessEq=*/false);
    ASSERT_EQ(poly.size(), 5u) << "sanity: this geometry must produce the 5-vertex case under test";

    const double fov = static_cast<double>(camera.fov_y_radians);
    std::vector<Px> screenPoly;
    for (const Cam3& v : poly) {
        screenPoly.push_back(project_camera_space_oracle(v, fov, kDim, kDim));
    }

    size_t expected_count = 0;
    for (uint32_t y = 0; y < kDim; ++y) {
        for (uint32_t x = 0; x < kDim; ++x) {
            const bool want = fan_covers_pixel(screenPoly, Px{x + 0.5, y + 0.5});
            const size_t idx = (static_cast<size_t>(y) * kDim + x) * 4;
            const bool got = storage[idx] == expected[0] && storage[idx + 1] == expected[1] &&
                              storage[idx + 2] == expected[2];
            ASSERT_EQ(got, want) << "pixel (" << x << "," << y
                                  << ") disagrees with the analytically clipped 5-vertex polygon's fan "
                                     "triangulation";
            if (want) {
                ++expected_count;
            }
        }
    }
    EXPECT_GT(expected_count, 0u) << "the analytic reference polygon itself must be non-degenerate";
}

// R5b review round 2, IMPORTANT: everything above exercises
// draw_mesh_triangle_shaded's fan. draw_world_triangle -- the OVERLAY path,
// used only by the spawn/body markers -- had NO exactness coverage at all:
// SpawnMarkerOverlaySurvivesNearPlaneStraddle's straddling triangle
// (v0,v2,v3) clips to exactly 3 vertices (one crossing survives), so its
// fan loop body runs once and every fan variant coincides -- the identical
// blind spot the previous round closed for the mesh path. This matters
// specifically here: draw_world_triangle draws the markers that straddle
// the near plane when Phase 4's camera flies close to them, which is
// exactly the scenario SR-15 named ("leaving the overlay path rejecting
// would mean spawn markers vanish at close range").
//
// This spawn marker's FIRST triangle (v0,v1,v2, draw_spawn_markers' own
// naming) has v0,v2 survive the near plane and v1 clipped away -- exactly 4
// vertices, a fan body that runs TWICE. Its SECOND triangle (v0,v2,v3) is
// entirely inside (the fast path) and is included in the oracle below too,
// since render() always draws both together.
TEST(RasterCpu, SpawnMarkerNearPlaneClipQuadCaseIsGeometricallyExact) {
    constexpr uint32_t kWidth = 300, kHeight = 300;
    // Mirrors draw_spawn_markers' own kSpawnMarkerRadius/kSpawnMarkerYOffset
    // (raster_cpu.cpp, anonymous namespace -- not accessible from this test
    // file), duplicated here as named constants rather than magic numbers,
    // same as SpawnMarkerOverlaySurvivesNearPlaneStraddle above already has
    // to do. Both are `double` in production (not float), so these double
    // literals are bit-identical to production's own -- no float32
    // round-trip involved for either.
    constexpr double kRadius = 0.3, kYOffset = 0.02;
    // X stays at 0 -- the marker's whole footprint (radius 0.3) must fit
    // inside the camera's frustum at this close a depth (visible half-width
    // at depth d, fov_y=60deg default, is d*tan(30deg) ~ 0.577d; at
    // d~0.2-0.5 that is only ~0.1-0.3, leaving no room for an X offset).
    // No grid/bounds interference despite x=0 coinciding with the ground
    // grid's own x=0 line: the camera sits exactly ON the grid's y=0 plane,
    // so EVERY grid vertex has world y=0 relative to the camera, and
    // therefore projects to the EXACT same degenerate screen row (yNdc=0
    // for any depth, since yNdc = f*rel.y*invNegZ and rel.y=0). This
    // marker sits at y=kYOffset=0.02 instead, and -- because this whole
    // marker is planar at that one Y -- every one of its vertices/crossings
    // projects comfortably away from that row (roughly 10+ screen rows at
    // this depth and kHeight, verified by inspection of the rendered
    // output below, not just assumed). Z is a float32 constant, reused
    // (via static_cast<double>) for the oracle below rather than re-typed
    // as a double literal -- same reasoning as T7's kApexX (round 1): -0.2
    // is not exact in either precision, so the widened float32 value and
    // an independently-written double literal would not be the same
    // number.
    constexpr float kMarkerX = 0.0f;
    constexpr float kMarkerBaseZ = -0.2f;

    RenderScene scene = make_empty_overlay_scene();
    scene.spawn_positions = {glm::vec3(kMarkerX, 0.0f, kMarkerBaseZ)};
    scene.spawn_orientations = {glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 0.0f));  // world == camera space
    RenderOptions options;
    options.overlays = true;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kWidth, kHeight);
    render_or_fail(scene, camera, options, target);

    const auto expected = bgr(190, 90, 170);  // kSpawnR/G/B

    // Independent re-derivation of draw_spawn_markers' own vertex placement
    // for BOTH of its triangles -- identity spawn orientation means
    // rotateByQuat is a no-op, so world == base-plus-local exactly, in the
    // SAME double arithmetic (base.z + 0.0 or +-kRadius) production uses.
    const double baseX = static_cast<double>(kMarkerX), baseZ = static_cast<double>(kMarkerBaseZ);
    const Cam3 v0{baseX + kRadius, kYOffset, baseZ}, v1{baseX, kYOffset, baseZ + kRadius},
        v2{baseX - kRadius, kYOffset, baseZ}, v3{baseX, kYOffset, baseZ - kRadius};

    const double nearZ = -static_cast<double>(camera.near_plane);
    const double farZ = -static_cast<double>(camera.far_plane);
    const double fov = static_cast<double>(camera.fov_y_radians);

    const auto clip_and_project = [&](std::vector<Cam3> tri) {
        std::vector<Cam3> poly = clip_polygon_oracle(std::move(tri), nearZ, /*keepLessEq=*/true);
        poly = clip_polygon_oracle(poly, farZ, /*keepLessEq=*/false);
        std::vector<Px> screenPoly;
        for (const Cam3& v : poly) {
            screenPoly.push_back(project_camera_space_oracle(v, fov, kWidth, kHeight));
        }
        return screenPoly;
    };

    const std::vector<Px> triangle1 = clip_and_project({v0, v1, v2});
    const std::vector<Px> triangle2 = clip_and_project({v0, v2, v3});
    ASSERT_EQ(triangle1.size(), 4u) << "sanity: triangle (v0,v1,v2) must produce the 4-vertex case under test";
    ASSERT_EQ(triangle2.size(), 3u) << "sanity: triangle (v0,v2,v3) must be entirely inside, unclipped";

    size_t expected_count = 0;
    for (uint32_t y = 0; y < kHeight; ++y) {
        for (uint32_t x = 0; x < kWidth; ++x) {
            const Px p{x + 0.5, y + 0.5};
            const bool want = fan_covers_pixel(triangle1, p) || fan_covers_pixel(triangle2, p);
            const size_t idx = (static_cast<size_t>(y) * kWidth + x) * 4;
            const bool got = storage[idx] == expected[0] && storage[idx + 1] == expected[1] &&
                              storage[idx + 2] == expected[2];
            ASSERT_EQ(got, want) << "pixel (" << x << "," << y
                                  << ") disagrees with the analytically clipped spawn-marker silhouette "
                                     "(draw_world_triangle's own fan)";
            if (want) {
                ++expected_count;
            }
        }
    }
    EXPECT_GT(expected_count, 0u) << "the analytic reference silhouette itself must be non-degenerate";
}
