// GlRenderer (engine/render_gl) on a real GL context. The first automated
// test of the OpenGL path: before it, GL was checked only by eye in the
// sandbox.
//
// The suite name starts with `Gpu`, so AppendSpadeLabels.cmake gives every
// case the `gpu` label (TD-13): it runs on the development box and the Docker
// leg excludes it. With no display or no GL 4.3 core context, each case skips
// and says why (L6), never passes on nothing.
//
// Rendering goes to an offscreen framebuffer, not the hidden window's own:
// pixels of a window that is not shown can fail the pixel-ownership test, so
// reading them back is undefined.
//
// Frames are RGBA8 with the BOTTOM row first, as glReadPixels returns them.
// cpu_rgba() converts raster_cpu's BGRX top-row-first frame to that layout,
// so a GL-to-CPU comparison is pixel for pixel.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render_gl/gl_renderer.hpp"

namespace {

using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::DrawMode;
using spade::render::GroundPlane;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render_gl::GlRenderer;

constexpr int kWidth = 160;
constexpr int kHeight = 120;
constexpr size_t kPixels = static_cast<size_t>(kWidth) * kHeight;

std::string g_last_glfw_error;

void record_glfw_error(int code, const char* description) {
    g_last_glfw_error = "GLFW error " + std::to_string(code) + ": " + (description != nullptr ? description : "");
}

// The same triangle as test_render_raster.cpp's make_single_triangle(): at
// z=+1 with normal +Z, so it faces a camera on the +Z side when wound CCW.
// `reversed` swaps the last two indices and changes nothing else.
[[nodiscard]] MeshData make_single_triangle(bool reversed) {
    MeshData mesh;
    mesh.positions = {glm::vec3(1.0f, -1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(-1.0f, 1.0f, 1.0f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = reversed ? std::vector<uint32_t>{0, 2, 1} : std::vector<uint32_t>{0, 1, 2};
    return mesh;
}

// One static item, unlit white, under a black sky. The coverage cases count
// pixels that are not black, so the sky must be black and lighting must not
// decide whether a pixel lights (the default sun is RND-5's subject).
[[nodiscard]] RenderScene make_scene(MeshData mesh) {
    RenderScene scene;
    scene.meshes.push_back(std::move(mesh));
    scene.materials = {Material{.base_color = glm::vec4(1.0f), .shading = 1u}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)});
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    return scene;
}

// No meshes; one lambert plane at y = 0 under the default gradient sky.
[[nodiscard]] RenderScene make_ground_scene() {
    RenderScene scene;
    scene.materials = {Material{.base_color = glm::vec4(0.45f, 0.5f, 0.42f, 1.0f)}};
    scene.ground_planes = {GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = 0.0f, .material = 0}};
    scene.lighting.sun_direction = glm::normalize(glm::vec3(0.4f, 0.8f, 0.6f));
    return scene;
}

// At +Z looking down -Z (the default orientation), as in the CPU test.
[[nodiscard]] Camera camera_on_plus_z() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    return camera;
}

// Above the ground, pitched down 12 degrees: sky, horizon and ground in one frame.
[[nodiscard]] Camera camera_over_ground() {
    Camera camera;
    camera.position = glm::vec3(0.3f, 1.7f, 6.0f);
    camera.orientation = glm::angleAxis(glm::radians(-12.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    return camera;
}

// Options for a GL-to-CPU comparison: the two features GL does not draw are off.
[[nodiscard]] RenderOptions comparable_options() {
    RenderOptions options;
    options.shadows = false;
    options.overlays = false;
    return options;
}

[[nodiscard]] bool is_black(const std::vector<uint8_t>& rgba, size_t pixel) {
    return rgba[pixel * 4u] == 0u && rgba[pixel * 4u + 1u] == 0u && rgba[pixel * 4u + 2u] == 0u;
}

[[nodiscard]] size_t count_non_black(const std::vector<uint8_t>& rgba) {
    size_t n = 0;
    for (size_t p = 0; p < rgba.size() / 4u; ++p) {
        if (!is_black(rgba, p)) ++n;
    }
    return n;
}

[[nodiscard]] bool same_pixel(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, size_t pixel) {
    return a[pixel * 4u] == b[pixel * 4u] && a[pixel * 4u + 1u] == b[pixel * 4u + 1u] &&
           a[pixel * 4u + 2u] == b[pixel * 4u + 2u];
}

// A count, not EXPECT_EQ on the frames: a failing EXPECT_EQ prints both frames.
[[nodiscard]] size_t count_differing(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    if (a.size() != b.size()) return kPixels;
    size_t n = 0;
    for (size_t p = 0; p < a.size() / 4u; ++p) {
        if (!same_pixel(a, b, p)) ++n;
    }
    return n;
}

// raster_cpu's frame in draw_and_read()'s layout: RGBA8, bottom row first.
[[nodiscard]] std::vector<uint8_t> cpu_rgba(const RenderScene& scene, const Camera& camera,
                                            const RenderOptions& options) {
    std::vector<uint8_t> bgrx(kPixels * 4u);
    RenderTarget target{.pixels = std::span<uint8_t>(bgrx),
                        .width = static_cast<uint32_t>(kWidth),
                        .height = static_cast<uint32_t>(kHeight),
                        .stride = static_cast<uint32_t>(kWidth) * 4u};
    if (auto drew = spade::render::render(scene, camera, options, target); !drew) {
        ADD_FAILURE() << drew.error().context;
        return {};
    }
    std::vector<uint8_t> rgba(bgrx.size());
    for (size_t y = 0; y < static_cast<size_t>(kHeight); ++y) {
        for (size_t x = 0; x < static_cast<size_t>(kWidth); ++x) {
            const size_t src = (y * kWidth + x) * 4u;
            const size_t dst = ((kHeight - 1u - y) * kWidth + x) * 4u;
            rgba[dst] = bgrx[src + 2u];
            rgba[dst + 1u] = bgrx[src + 1u];
            rgba[dst + 2u] = bgrx[src];
            rgba[dst + 3u] = 255u;
        }
    }
    return rgba;
}

class GpuGlRenderer : public ::testing::Test {
  protected:
    void SetUp() override {
        glfwSetErrorCallback(record_glfw_error);
        g_last_glfw_error.clear();
        if (glfwInit() != GLFW_TRUE) {
            GTEST_SKIP() << "GLFW could not initialise; no display is available to this process. "
                         << g_last_glfw_error;
        }
        glfw_initialised_ = true;

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(kWidth, kHeight, "spade GpuGlRenderer", nullptr, nullptr);
        if (window_ == nullptr) {
            GTEST_SKIP() << "no OpenGL 4.3 core context on this machine. " << g_last_glfw_error;
        }
        glfwMakeContextCurrent(window_);

        auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress));
        if (!made) {
            // unavailable is the driver saying no (the version check); any
            // other code is GlRenderer itself failing, which is a real failure.
            if (made.error().code == spade::Code::unavailable) {
                GTEST_SKIP() << made.error().context;
            }
            FAIL() << made.error().context;
        }
        renderer_ = std::move(*made);

        glGenFramebuffers(1, &fbo_);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glGenRenderbuffers(1, &colour_);
        glBindRenderbuffer(GL_RENDERBUFFER, colour_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, kWidth, kHeight);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour_);
        glGenRenderbuffers(1, &depth_);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, kWidth, kHeight);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
        ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));
    }

    void TearDown() override {
        // GL objects die with the context, so the context must still be current.
        renderer_.reset();
        if (fbo_ != 0) glDeleteFramebuffers(1, &fbo_);
        if (colour_ != 0) glDeleteRenderbuffers(1, &colour_);
        if (depth_ != 0) glDeleteRenderbuffers(1, &depth_);
        if (window_ != nullptr) glfwDestroyWindow(window_);
        if (glfw_initialised_) glfwTerminate();
    }

    // Clears to black, draws `scene`, and reads back RGBA8, bottom row first.
    // Empty if the renderer refused.
    [[nodiscard]] std::vector<uint8_t> draw_and_read(const RenderScene& scene,
                                                     const RenderOptions& options = RenderOptions{},
                                                     const Camera& camera = camera_on_plus_z()) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        if (auto up = renderer_->upload_scene(scene); !up) {
            ADD_FAILURE() << up.error().context;
            return {};
        }
        if (auto drew = renderer_->draw(scene, camera, options, kWidth, kHeight); !drew) {
            ADD_FAILURE() << drew.error().context;
            return {};
        }
        std::vector<uint8_t> rgba(kPixels * 4u);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        EXPECT_EQ(glGetError(), static_cast<GLenum>(GL_NO_ERROR));
        return rgba;
    }

    // How many pixels `scene` turns from black.
    [[nodiscard]] size_t covered_pixels(const RenderScene& scene, const RenderOptions& options = RenderOptions{}) {
        return count_non_black(draw_and_read(scene, options));
    }

    std::unique_ptr<GlRenderer> renderer_;

  private:
    bool glfw_initialised_ = false;
    GLFWwindow* window_ = nullptr;
    GLuint fbo_ = 0, colour_ = 0, depth_ = 0;
};

// The control: without it, the culling case below would pass on a camera
// that sees nothing.
TEST_F(GpuGlRenderer, DrawsAnOutwardFacingTriangle) {
    const size_t covered = covered_pixels(make_scene(make_single_triangle(/*reversed=*/false)));
    EXPECT_GT(covered, 0u) << "a correctly wound, outward-facing triangle must be visible on "
                           << renderer_->renderer_name();
    EXPECT_EQ(renderer_->last_draw_calls(), 1u);
}

// SR-13: shaded mode culls back faces, as raster_cpu does
// (RasterCpu.ShadedModeRendersOutwardFacingTriangleButCullsReversedOne).
TEST_F(GpuGlRenderer, CullsAReversedTriangle) {
    const size_t covered = covered_pixels(make_scene(make_single_triangle(/*reversed=*/true)));
    EXPECT_EQ(covered, 0u) << "a reversed-winding triangle must be back-face culled (SR-13)";
    EXPECT_EQ(renderer_->last_draw_calls(), 1u) << "culled by GL, not skipped by the draw loop";
}

// Normals take the inverse-transpose (render/scene.hpp's transform_normal()).
// A triangle in the plane x + z = 0, scaled 2x along x, faces (1, 0, 2); mat3(m)
// would say (2, 0, 1). The sun lies in the true surface, so the true N.L is 0
// and every covered pixel is the ambient term alone: 0.2 of white, 51. The old
// normal puts N.L at 0.6, which reads as 204.
TEST_F(GpuGlRenderer, LightsANonUniformlyScaledSurfaceByItsTrueNormal) {
    MeshData mesh;
    mesh.positions = {glm::vec3(-1.0f, -1.0f, 1.0f), glm::vec3(1.0f, -1.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f)};
    mesh.normals.assign(3, glm::normalize(glm::vec3(1.0f, 0.0f, 1.0f)));
    mesh.indices = {0, 1, 2};
    RenderScene scene;
    scene.meshes.push_back(std::move(mesh));
    scene.materials = {Material{.base_color = glm::vec4(1.0f), .shading = 0u}};
    glm::mat4 stretch(1.0f);
    stretch[0][0] = 2.0f;
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = stretch});
    scene.lighting.sun_direction = glm::normalize(glm::vec3(2.0f, 0.0f, -1.0f));
    scene.lighting.sun_color = glm::vec3(1.0f);
    scene.lighting.sun_intensity = 1.0f;
    scene.lighting.ambient_color = glm::vec3(0.2f);
    // A black sky keeps "not black" meaning "covered" once GL draws the sky.
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);

    const std::vector<uint8_t> rgba = draw_and_read(scene);
    size_t covered = 0;
    for (size_t i = 0; i < rgba.size(); i += 4u) {
        if (rgba[i] == 0u && rgba[i + 1u] == 0u && rgba[i + 2u] == 0u) continue;
        ++covered;
        for (size_t ch = 0; ch < 3u; ++ch) {
            ASSERT_NEAR(static_cast<int>(rgba[i + ch]), 51, 2)
                << "pixel " << i / 4u << " channel " << ch << ": lit by a normal that is off its surface";
        }
    }
    EXPECT_GT(covered, 0u) << "the scaled triangle must be visible";
}

// SR-13's other clause: wireframe culls nothing, so both windings give the
// same edges (RasterCpu.WireframeModeDrawsBothWindingsIdentically). Fewer
// pixels than the filled triangle proves GL drew lines, not a fill.
TEST_F(GpuGlRenderer, WireframeDrawsBothWindingsIdentically) {
    RenderOptions wireframe;
    wireframe.mode = DrawMode::wireframe;
    const std::vector<uint8_t> forward = draw_and_read(make_scene(make_single_triangle(false)), wireframe);
    const std::vector<uint8_t> reversed = draw_and_read(make_scene(make_single_triangle(true)), wireframe);
    const size_t filled = covered_pixels(make_scene(make_single_triangle(false)));
    const size_t edges = count_non_black(forward);
    EXPECT_GT(edges, 0u) << "wireframe must draw a back-facing triangle's edges too";
    EXPECT_LT(edges, filled) << "wireframe drew as many pixels as the fill: it is not drawing lines";
    EXPECT_EQ(count_differing(forward, reversed), 0u) << "wireframe must not cull (SR-13)";
}

// SR-22: the analytic ground draws in shaded mode only; the sky in every mode.
TEST_F(GpuGlRenderer, DrawsTheAnalyticGroundInShadedModeOnly) {
    const RenderScene with_ground = make_ground_scene();
    RenderScene without_ground = with_ground;
    without_ground.ground_planes.clear();
    for (const DrawMode mode : {DrawMode::shaded, DrawMode::wireframe, DrawMode::velocity}) {
        RenderOptions options = comparable_options();
        options.mode = mode;
        const std::vector<uint8_t> a = draw_and_read(with_ground, options, camera_over_ground());
        const std::vector<uint8_t> b = draw_and_read(without_ground, options, camera_over_ground());
        if (mode == DrawMode::shaded) {
            EXPECT_GT(count_differing(a, b), 0u) << "shaded mode must draw the analytic ground";
        } else {
            EXPECT_EQ(count_differing(a, b), 0u)
                << "mode " << static_cast<uint32_t>(mode) << " must not draw the analytic ground";
        }
    }
}

// GL's background is a port of raster_cpu's draw_sky_and_ground_background():
// sky, analytic ground, grid and horizon term. GL is fp32 and the CPU is
// fp64, so the frames agree to a band, measured and then pinned
// (03-verification). It guards against regressions on a best-effort
// technique (RND-3); it is not a grade.
//
// Two numbers. Edges dominate a single maximum: a grid line or the horizon
// can land one pixel apart on the two paths. So the second maximum covers
// "interior" pixels only, whose 3x3 neighbourhood on the CPU frame holds no
// grid-line pixel and no sky/ground change.
TEST_F(GpuGlRenderer, BackgroundMatchesTheCpuWithinItsMeasuredBand) {
    // Measured 2026-10-03 at 087c3c3, 160x120, on Intel Iris Plus Graphics
    // (driver 31.0.101.2125): at most 1 level over all 19200 pixels, and 1
    // over the 15821 interior ones. Pinned at the measurement. Another device
    // may differ; re-measure there before widening (03-verification).
    constexpr int kBandAll = 1;
    constexpr int kBandInterior = 1;

    const RenderScene scene = make_ground_scene();
    RenderScene sky_only = scene;
    sky_only.ground_planes.clear();
    RenderOptions options = comparable_options();
    options.ground_grid = true;
    options.horizon_blend_strength = 0.6f;
    options.horizon_blend_onset = 20.0f;
    RenderOptions no_grid = options;
    no_grid.ground_grid = false;
    const Camera camera = camera_over_ground();

    const std::vector<uint8_t> gl = draw_and_read(scene, options, camera);
    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, options);
    const std::vector<uint8_t> cpu_no_grid = cpu_rgba(scene, camera, no_grid);
    const std::vector<uint8_t> cpu_sky = cpu_rgba(sky_only, camera, options);
    ASSERT_EQ(gl.size(), kPixels * 4u);
    ASSERT_EQ(cpu.size(), kPixels * 4u);

    std::vector<uint8_t> is_ground(kPixels), is_grid(kPixels);
    size_t ground = 0, grid = 0;
    for (size_t p = 0; p < kPixels; ++p) {
        is_ground[p] = static_cast<uint8_t>(same_pixel(cpu, cpu_sky, p) ? 0 : 1);
        is_grid[p] = static_cast<uint8_t>(same_pixel(cpu, cpu_no_grid, p) ? 0 : 1);
        ground += is_ground[p];
        grid += is_grid[p];
    }
    ASSERT_GT(ground, kPixels / 4u) << "the frame must hold ground";
    ASSERT_LT(ground, kPixels * 3u / 4u) << "the frame must hold sky";
    ASSERT_GT(grid, 0u) << "the frame must hold grid lines";

    int max_all = 0, max_interior = 0;
    size_t interior = 0;
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const size_t p = static_cast<size_t>(y) * kWidth + static_cast<size_t>(x);
            int diff = 0;
            for (size_t ch = 0; ch < 3u; ++ch) {
                diff = std::max(diff, std::abs(static_cast<int>(gl[p * 4u + ch]) - static_cast<int>(cpu[p * 4u + ch])));
            }
            max_all = std::max(max_all, diff);
            bool is_interior = true;
            for (int dy = -1; dy <= 1 && is_interior; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = std::clamp(x + dx, 0, kWidth - 1);
                    const int ny = std::clamp(y + dy, 0, kHeight - 1);
                    const size_t q = static_cast<size_t>(ny) * kWidth + static_cast<size_t>(nx);
                    if (is_grid[q] != 0u || is_ground[q] != is_ground[p]) {
                        is_interior = false;
                        break;
                    }
                }
            }
            if (is_interior) {
                max_interior = std::max(max_interior, diff);
                ++interior;
            }
        }
    }
    ASSERT_GT(interior, kPixels / 2u) << "most of the frame must be interior, or the second band says little";
    RecordProperty("max_all", max_all);
    RecordProperty("max_interior", max_interior);
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    EXPECT_LE(max_interior, kBandInterior)
        << "interior pixels (no grid line or horizon within 1 px) differ from the CPU by up to " << max_interior
        << " levels; " << interior << " interior pixels, on " << device;
    EXPECT_LE(max_all, kBandAll) << "some pixel differs from the CPU by " << max_all << " levels, on " << device;
}

// SR-17a on mesh fragments: every shaded surface blends toward the sky by
// view distance, and at strength 0 the term is exact.
TEST_F(GpuGlRenderer, AtmosphericTermBlendsMeshPixelsAndIsExactAtZero) {
    RenderScene scene = make_scene(make_single_triangle(false));
    scene.materials[0].base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);  // unlit red
    scene.lighting.sky_zenith = glm::vec3(0.0f, 0.0f, 1.0f);           // flat blue sky
    scene.lighting.sky_horizon = glm::vec3(0.0f, 0.0f, 1.0f);
    RenderScene sky_only = scene;
    sky_only.statics.clear();

    RenderOptions off = comparable_options();
    off.horizon_blend_onset = 2.0f;  // the triangle is about 4 m away: a strong blend
    RenderOptions on = off;
    on.horizon_blend_strength = 1.0f;

    const std::vector<uint8_t> sky = draw_and_read(sky_only, off);
    const std::vector<uint8_t> frame_off = draw_and_read(scene, off);
    const std::vector<uint8_t> frame_on = draw_and_read(scene, on);
    ASSERT_EQ(frame_off.size(), kPixels * 4u);
    ASSERT_EQ(frame_on.size(), kPixels * 4u);
    // Counts, not one EXPECT per pixel, so a failure prints three lines.
    size_t mesh = 0, inexact_at_zero = 0, unblended = 0;
    for (size_t p = 0; p < kPixels; ++p) {
        if (same_pixel(frame_off, sky, p)) continue;
        ++mesh;
        if (frame_off[p * 4u] != 255u || frame_off[p * 4u + 1u] != 0u || frame_off[p * 4u + 2u] != 0u) {
            ++inexact_at_zero;
        }
        if (!(frame_on[p * 4u] < 200u && frame_on[p * 4u + 2u] > 55u)) ++unblended;
    }
    EXPECT_GT(mesh, 0u) << "the triangle must be visible";
    EXPECT_EQ(inexact_at_zero, 0u) << "of " << mesh << " mesh pixels: strength 0 must leave unlit red exact";
    EXPECT_EQ(unblended, 0u) << "of " << mesh << " mesh pixels: strength 1 must blend red toward the blue sky";
}

// raster_cpu's velocity_ramp(): blue at rest, red at velocity_scale_mps and
// above. Unlit white, so each pixel is the ramp colour itself.
TEST_F(GpuGlRenderer, VelocityModeColoursEachInstanceBySpeed) {
    RenderScene scene = make_scene(make_single_triangle(false));
    scene.statics = {
        DrawItem{.mesh_index = 0, .local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(-1.3f, 0.0f, 0.0f)),
                 .speed_mps = 0.0f},
        DrawItem{.mesh_index = 0, .local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(1.3f, 0.0f, 0.0f)),
                 .speed_mps = 25.0f},
    };
    RenderOptions options = comparable_options();
    options.mode = DrawMode::velocity;
    options.velocity_scale_mps = 20.0f;
    const std::vector<uint8_t> rgba = draw_and_read(scene, options);
    ASSERT_EQ(rgba.size(), kPixels * 4u);
    size_t blue = 0, red = 0, wrong = 0;
    for (size_t p = 0; p < kPixels; ++p) {
        if (is_black(rgba, p)) continue;
        const bool left = (p % kWidth) < static_cast<size_t>(kWidth) / 2u;
        const uint8_t r = rgba[p * 4u], g = rgba[p * 4u + 1u], b = rgba[p * 4u + 2u];
        if (left && r == 0u && g == 0u && b == 255u) {
            ++blue;
        } else if (!left && r == 255u && g == 0u && b == 0u) {
            ++red;
        } else {
            ++wrong;
        }
    }
    EXPECT_GT(blue, 0u) << "the instance at rest must draw blue";
    EXPECT_GT(red, 0u) << "the instance above the scale must draw red";
    EXPECT_EQ(wrong, 0u) << "pixels that are not their instance's ramp colour";
    EXPECT_EQ(renderer_->last_draw_calls(), 1u) << "two instances of one mesh are still one draw";
}

// FC-4: GL draws the same field layer as raster_cpu, within a band measured
// and then pinned. The atmospheric term is on, and neither path may apply it
// to the layer (FC-2). As with the background band, cell borders can land a
// pixel apart, so the second maximum covers interior pixels only: those whose
// 3x3 neighbourhood on the CPU frame is one colour.
TEST_F(GpuGlRenderer, FieldLayerMatchesTheCpuWithinItsMeasuredBand) {
    // Measured 2026-10-03 at a89d8ca, 160x120, on Intel Iris Plus Graphics
    // (GL 4.3, driver 31.0.101.2125): at most 1 level over all 19200 pixels,
    // and 1 over the 15960 interior ones; no border pixel changed cell.
    // Pinned at the measurement. Re-measure on another device before widening.
    constexpr int kBandAll = 1;
    constexpr int kBandInterior = 1;

    RenderScene scene;
    scene.materials = {Material{}};
    // A flat sky, so sky pixels are interior too: a gradient changes every row.
    scene.lighting.sky_zenith = glm::vec3(0.3f, 0.4f, 0.6f);
    scene.lighting.sky_horizon = glm::vec3(0.3f, 0.4f, 0.6f);
    spade::render::FieldLayer layer;  // about 60% of the frame from 5 m
    layer.width = 6.0f;
    layer.height = 4.5f;
    layer.cells_u = 8;
    layer.cells_v = 6;
    for (uint32_t j = 0; j < layer.cells_v; ++j) {
        for (uint32_t i = 0; i < layer.cells_u; ++i) {
            layer.values.push_back(static_cast<float>(i * 3u + j * 5u));
        }
    }
    layer.colour_map.bins = 16;  // range_max 0: the layer's own maximum
    RenderScene no_layer = scene;
    scene.field_layers = {layer};
    RenderOptions options = comparable_options();
    options.horizon_blend_strength = 1.0f;
    options.horizon_blend_onset = 0.5f;

    const std::vector<uint8_t> gl = draw_and_read(scene, options);
    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera_on_plus_z(), options);
    const std::vector<uint8_t> cpu_bare = cpu_rgba(no_layer, camera_on_plus_z(), options);
    ASSERT_EQ(gl.size(), kPixels * 4u);
    ASSERT_EQ(cpu.size(), kPixels * 4u);

    size_t covered = 0;
    for (size_t p = 0; p < kPixels; ++p) {
        if (!same_pixel(cpu, cpu_bare, p)) ++covered;
    }
    ASSERT_GT(covered, kPixels / 4u) << "the layer must cover a good part of the frame";

    int max_all = 0, max_interior = 0;
    size_t interior = 0;
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const size_t p = static_cast<size_t>(y) * kWidth + static_cast<size_t>(x);
            int diff = 0;
            for (size_t ch = 0; ch < 3u; ++ch) {
                diff = std::max(diff, std::abs(static_cast<int>(gl[p * 4u + ch]) - static_cast<int>(cpu[p * 4u + ch])));
            }
            max_all = std::max(max_all, diff);
            bool is_interior = true;
            for (int dy = -1; dy <= 1 && is_interior; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = std::clamp(x + dx, 0, kWidth - 1);
                    const int ny = std::clamp(y + dy, 0, kHeight - 1);
                    const size_t q = static_cast<size_t>(ny) * kWidth + static_cast<size_t>(nx);
                    if (cpu[q * 4u] != cpu[p * 4u] || cpu[q * 4u + 1u] != cpu[p * 4u + 1u] ||
                        cpu[q * 4u + 2u] != cpu[p * 4u + 2u]) {
                        is_interior = false;
                        break;
                    }
                }
            }
            if (is_interior) {
                max_interior = std::max(max_interior, diff);
                ++interior;
            }
        }
    }
    ASSERT_GT(interior, kPixels / 2u) << "most of the frame must be interior, or the second band says little";
    RecordProperty("max_all", max_all);
    RecordProperty("max_interior", max_interior);
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    EXPECT_LE(max_interior, kBandInterior) << "interior pixels differ from the CPU by up to " << max_interior
                                           << " levels; " << interior << " interior pixels, on " << device;
    EXPECT_LE(max_all, kBandAll) << "some pixel differs from the CPU by " << max_all << " levels, on " << device;
}

// The no-data colour on GL too: a NaN or infinite sample is grey (0.5),
// never bin 0's colour. Black sky, so "not black" means "layer".
TEST_F(GpuGlRenderer, FieldLayerShowsNonFiniteSamplesAsNoData) {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    spade::render::FieldLayer layer;
    layer.width = 2.0f;
    layer.height = 2.0f;
    layer.cells_u = 2;
    layer.values = {std::nanf(""), -std::numeric_limits<float>::infinity()};
    scene.field_layers = {layer};
    const std::vector<uint8_t> rgba = draw_and_read(scene, comparable_options());
    ASSERT_EQ(rgba.size(), kPixels * 4u);
    size_t covered = 0, not_grey = 0;
    for (size_t p = 0; p < kPixels; ++p) {
        if (is_black(rgba, p)) continue;
        ++covered;
        for (size_t ch = 0; ch < 3u; ++ch) {
            if (std::abs(static_cast<int>(rgba[p * 4u + ch]) - 128) > 1) {
                ++not_grey;
                break;
            }
        }
    }
    EXPECT_GT(covered, 0u) << "the layer must be visible";
    EXPECT_EQ(not_grey, 0u) << "layer pixels that are not the no-data grey";
}

// Ray-marching is a different technique (render/raymarch) that GL does not
// have, so draw() refuses it rather than drawing something else (L6).
TEST_F(GpuGlRenderer, RefusesRaymarch) {
    const RenderScene scene = make_scene(make_single_triangle(false));
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
    RenderOptions options;
    options.mode = DrawMode::raymarch;
    const spade::Result<void> drew = renderer_->draw(scene, camera_on_plus_z(), options, kWidth, kHeight);
    ASSERT_FALSE(drew.has_value()) << "GL must not draw a raymarch frame";
    EXPECT_EQ(drew.error().code, spade::Code::unavailable);
}

// L6: what GL does not draw is announced by name, so a caller can show it or
// refuse GL. Host-only: no context needed, so this suite has no gpu label.
TEST(GlRendererOptions, UnhonouredNamesWhatGlDoesNotDraw) {
    using Names = std::vector<std::string_view>;
    EXPECT_EQ(GlRenderer::unhonoured(RenderOptions{}), (Names{"shadows", "overlays"}))
        << "the defaults ask for shadows and overlays, which GL does not draw";

    RenderOptions drawn = comparable_options();
    drawn.ground_grid = true;
    drawn.horizon_blend_strength = 0.5f;
    EXPECT_EQ(GlRenderer::unhonoured(drawn), Names{}) << "grid and atmosphere are drawn";

    RenderOptions wireframe;
    wireframe.mode = DrawMode::wireframe;
    wireframe.overlays = false;
    EXPECT_EQ(GlRenderer::unhonoured(wireframe), Names{}) << "shadows draw in shaded mode only, on the CPU too";

    RenderOptions raymarch;
    raymarch.mode = DrawMode::raymarch;
    EXPECT_EQ(GlRenderer::unhonoured(raymarch), (Names{"mode"})) << "raymarch is refused, and named";
}

}  // namespace
