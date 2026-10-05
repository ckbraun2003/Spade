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
#include "render/shadow.hpp"
#include "render/target.hpp"
#include "render/tessellate.hpp"
#include "world/sdf.hpp"
#include "world/builder.hpp"
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

// The fixture's default options, and a GL-to-CPU comparison's: shadows and
// overlays off, so a case sees only what it is about. The shadow and overlay
// cases turn theirs on.
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
        // This test calls GL itself (framebuffer, readback), through glad.
        // GlRenderer has its own private table and does not load glad for us.
        if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) == 0) {
            GTEST_SKIP() << "glad could not load GL entry points for the test's own calls";
        }

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
                                                     const RenderOptions& options = comparable_options(),
                                                     const Camera& camera = camera_on_plus_z()) {
        if (auto up = renderer_->upload_scene(scene); !up) {
            ADD_FAILURE() << up.error().context;
            return {};
        }
        return draw_uploaded_and_read(scene, options, camera);
    }

    // As draw_and_read(), with whatever upload_scene() last accepted.
    [[nodiscard]] std::vector<uint8_t> draw_uploaded_and_read(const RenderScene& scene,
                                                              const RenderOptions& options = comparable_options(),
                                                              const Camera& camera = camera_on_plus_z()) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);
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
    [[nodiscard]] size_t covered_pixels(const RenderScene& scene, const RenderOptions& options = comparable_options()) {
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

// GL draws the overlays from the CPU's own lists (render::overlay_geometry()),
// with the CPU's inverse-depth bias. Lines rasterize by different rules on
// the two paths, so a pixel may sit one pixel apart; the colours are flat
// and must match exactly. A miss is an overlay pixel on one path with no
// overlay pixel of the same colour within 1 px on the other. Both counts are
// measured and pinned (03-verification): a regression guard on a
// best-effort technique (RND-3), not a grade.
TEST_F(GpuGlRenderer, OverlaysMatchTheCpuWithinTheirBand) {
    // Measured 2026-10-05 on rendering/gl-overlays, 160x120, on an NVIDIA
    // GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83): 4 of the CPU's 3277
    // overlay pixels miss, and 0 of GL's. The 4 are where a bounds edge
    // crosses grid lines on screen: GL's bounds line covers a pixel that is
    // grid on the CPU (3), and one grid pixel lands on bare ground in GL.
    // Pinned at the measurement. Another device may differ; re-measure there
    // before widening (03-verification). Known difference, within the band:
    // GL biases each vertex before clipping, and a vertex behind the eye gets
    // no bias, so a grid line crossing the eye plane bends by about 0.2 px
    // against the CPU, which biases after clipping.
    constexpr size_t kBandCpuMisses = 4;
    constexpr size_t kBandGlMisses = 0;

    RenderScene scene = make_ground_scene();
    // Off the grid lines: a bounds edge on a grid line would tie with it, and
    // which colour wins a tie is rounding, not a property of either path.
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-2.5f, 0.25f, -2.5f), .max = glm::vec3(2.5f, 2.0f, 2.5f)};
    scene.spawn_positions = {glm::vec3(1.0f, 0.0f, 1.0f), glm::vec3(-1.5f, 0.0f, -1.0f)};
    scene.spawn_orientations = {glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 1.0f, 0.0f))};
    scene.dynamics.push_back(DrawItem{.mesh_index = spade::render::kNoMesh,
                                      .local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.0f, 0.0f))});
    RenderOptions with = comparable_options();
    with.overlays = true;
    with.spawn_markers = true;
    const RenderOptions without = comparable_options();
    const Camera camera = camera_over_ground();

    const std::vector<uint8_t> cpu_on = cpu_rgba(scene, camera, with);
    const std::vector<uint8_t> cpu_off = cpu_rgba(scene, camera, without);
    const std::vector<uint8_t> gl_on = draw_and_read(scene, with, camera);
    const std::vector<uint8_t> gl_off = draw_and_read(scene, without, camera);
    ASSERT_EQ(gl_on.size(), kPixels * 4u);
    ASSERT_EQ(gl_off.size(), kPixels * 4u);

    std::vector<uint8_t> cpu_overlay(kPixels), gl_overlay(kPixels);
    size_t cpu_count = 0, gl_count = 0;
    for (size_t p = 0; p < kPixels; ++p) {
        cpu_overlay[p] = static_cast<uint8_t>(same_pixel(cpu_on, cpu_off, p) ? 0 : 1);
        gl_overlay[p] = static_cast<uint8_t>(same_pixel(gl_on, gl_off, p) ? 0 : 1);
        cpu_count += cpu_overlay[p];
        gl_count += gl_overlay[p];
    }
    ASSERT_GT(cpu_count, 200u) << "the CPU frame must hold overlays";

    // Overlay pixels in `a` with no same-coloured overlay pixel within 1 px in `b`.
    const auto misses = [](const std::vector<uint8_t>& a, const std::vector<uint8_t>& a_mask,
                           const std::vector<uint8_t>& b, const std::vector<uint8_t>& b_mask) {
        size_t n = 0;
        for (int y = 0; y < kHeight; ++y) {
            for (int x = 0; x < kWidth; ++x) {
                const size_t p = static_cast<size_t>(y) * kWidth + static_cast<size_t>(x);
                if (a_mask[p] == 0u) continue;
                bool found = false;
                for (int dy = -1; dy <= 1 && !found; ++dy) {
                    for (int dx = -1; dx <= 1 && !found; ++dx) {
                        const int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= kWidth || ny >= kHeight) continue;
                        const size_t q = static_cast<size_t>(ny) * kWidth + static_cast<size_t>(nx);
                        found = b_mask[q] != 0u && a[p * 4u] == b[q * 4u] && a[p * 4u + 1u] == b[q * 4u + 1u] &&
                                a[p * 4u + 2u] == b[q * 4u + 2u];
                    }
                }
                if (!found) ++n;
            }
        }
        return n;
    };
    const size_t cpu_misses = misses(cpu_on, cpu_overlay, gl_on, gl_overlay);
    const size_t gl_misses = misses(gl_on, gl_overlay, cpu_on, cpu_overlay);
    RecordProperty("cpu_overlay_pixels", static_cast<int>(cpu_count));
    RecordProperty("gl_overlay_pixels", static_cast<int>(gl_count));
    RecordProperty("cpu_misses", static_cast<int>(cpu_misses));
    RecordProperty("gl_misses", static_cast<int>(gl_misses));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    EXPECT_LE(cpu_misses, kBandCpuMisses) << cpu_misses << " of " << cpu_count
                                          << " CPU overlay pixels have no match in GL within 1 px, on " << device;
    EXPECT_LE(gl_misses, kBandGlMisses) << gl_misses << " of " << gl_count
                                        << " GL overlay pixels have no match on the CPU within 1 px, on " << device;
}

// SR-21 on GL: the grid lies exactly on a tessellated ground mesh at y = 0,
// and the overlay depth bias (ported in the vertex shader) must win that tie,
// as it does on the CPU. Without the bias the two are at one depth and
// GL_LESS hides most of the grid.
TEST_F(GpuGlRenderer, OverlayBiasKeepsTheGridOnACoincidentGroundMesh) {
    MeshData ground;
    ground.positions = {glm::vec3(-8.0f, 0.0f, -8.0f), glm::vec3(-8.0f, 0.0f, 8.0f), glm::vec3(8.0f, 0.0f, 8.0f),
                        glm::vec3(8.0f, 0.0f, -8.0f)};
    ground.normals.assign(4, glm::vec3(0.0f, 1.0f, 0.0f));
    ground.indices = {0, 1, 2, 0, 2, 3};  // counter-clockwise seen from +Y
    RenderScene scene = make_scene(std::move(ground));
    scene.materials[0] = Material{.base_color = glm::vec4(0.2f, 0.6f, 0.3f, 1.0f)};  // lambert, not grid grey
    Camera camera;
    camera.position = glm::vec3(0.3f, 4.0f, 0.2f);
    camera.orientation = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));  // looking down
    RenderOptions with = comparable_options();
    with.overlays = true;

    const auto grid_pixels = [](const std::vector<uint8_t>& rgba) {
        size_t n = 0;
        for (size_t p = 0; p < kPixels; ++p) {
            n += (rgba[p * 4u] == 90u && rgba[p * 4u + 1u] == 90u && rgba[p * 4u + 2u] == 90u) ? 1u : 0u;
        }
        return n;
    };
    const size_t cpu = grid_pixels(cpu_rgba(scene, camera, with));
    const std::vector<uint8_t> gl_frame = draw_and_read(scene, with, camera);
    ASSERT_EQ(gl_frame.size(), kPixels * 4u);
    const size_t gl = grid_pixels(gl_frame);
    ASSERT_GT(cpu, 200u) << "the CPU frame must show the grid on the ground mesh";
    EXPECT_GE(gl * 10u, cpu * 9u) << "GL shows " << gl << " grid pixels on the ground mesh, the CPU " << cpu;
}

// A surface nearer than an overlay hides it: from above, a quad at y = 1 hides
// the ground grid at y = 0, which still shows around it.
TEST_F(GpuGlRenderer, OverlaysLoseToNearerGeometry) {
    MeshData quad;
    quad.positions = {glm::vec3(-1.0f, 1.0f, -1.0f), glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f),
                      glm::vec3(1.0f, 1.0f, -1.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 1.0f, 0.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};  // counter-clockwise seen from +Y
    const RenderScene scene = make_scene(std::move(quad));
    Camera camera;
    camera.position = glm::vec3(0.0f, 5.0f, 0.0f);
    camera.orientation = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));  // looking down
    RenderOptions with = comparable_options();
    with.overlays = true;
    const RenderOptions without = comparable_options();

    const std::vector<uint8_t> quad_mask = cpu_rgba(scene, camera, without);
    const std::vector<uint8_t> gl_on = draw_and_read(scene, with, camera);
    const std::vector<uint8_t> gl_off = draw_and_read(scene, without, camera);
    ASSERT_EQ(gl_on.size(), kPixels * 4u);

    size_t interior = 0, shows_through = 0, grid_outside = 0;
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const size_t p = static_cast<size_t>(y) * kWidth + static_cast<size_t>(x);
            bool is_interior = true;
            for (int dy = -1; dy <= 1 && is_interior; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = std::clamp(x + dx, 0, kWidth - 1), ny = std::clamp(y + dy, 0, kHeight - 1);
                    if (is_black(quad_mask, static_cast<size_t>(ny) * kWidth + static_cast<size_t>(nx))) {
                        is_interior = false;
                        break;
                    }
                }
            }
            const bool changed = !same_pixel(gl_on, gl_off, p);
            if (is_interior) {
                ++interior;
                shows_through += changed ? 1u : 0u;
            } else if (is_black(quad_mask, p) && changed) {
                ++grid_outside;
            }
        }
    }
    ASSERT_GT(interior, kPixels / 10u) << "the quad must fill part of the frame";
    EXPECT_EQ(shows_through, 0u) << "the grid shows through the quad above it";
    EXPECT_GT(grid_outside, 0u) << "the grid must show around the quad";
}

// A lambert ground (the analytic plane, plus a tessellated plane mesh within
// +-5 m, as scene_from_world() gives a plane) under a box at y = 2.5, with
// the default sun. The box is static, or dynamic: then the static shadow map
// holds only the ground, and each frame adds the box (render/shadow.hpp).
[[nodiscard]] RenderScene make_shadow_scene(bool dynamic_caster) {
    const spade::render::Aabb tess_bounds{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    const auto mesh = [&](spade::SdfPrim kind, glm::vec4 params) {
        auto made = spade::render::tessellate_primitive(kind, params, tess_bounds, spade::render::kTessellationDefaults);
        if (!made) {
            ADD_FAILURE() << made.error().context;
            return MeshData{};
        }
        return std::move(*made);
    };
    RenderScene scene;
    scene.meshes.push_back(mesh(spade::SdfPrim::plane, glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));
    scene.meshes.push_back(mesh(spade::SdfPrim::box, glm::vec4(0.8f, 0.8f, 0.8f, 0.0f)));
    scene.materials = {Material{.base_color = glm::vec4(0.55f, 0.5f, 0.45f, 1.0f)},
                       Material{.base_color = glm::vec4(0.75f, 0.3f, 0.25f, 1.0f)}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)});
    const DrawItem caster{.mesh_index = 1,
                          .local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 2.5f, 0.0f)),
                          .material_override = 1};
    (dynamic_caster ? scene.dynamics : scene.statics).push_back(caster);
    scene.ground_planes = {GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = 0.0f, .material = 0}};
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f, -1.0f, -5.0f), .max = glm::vec3(5.0f, 4.0f, 5.0f)};
    auto map = spade::render::build_static_shadow_map(scene);
    if (!map) {
        ADD_FAILURE() << map.error().context;
        return scene;
    }
    scene.static_shadow = std::move(*map);
    return scene;
}

// Pitched down at the box's shadow (the default sun casts it toward -x, -z):
// the box, its shadow, and ground past the shadow map's footprint near the
// top of the frame, which stays lit (SR-17 clause 6).
[[nodiscard]] Camera camera_over_shadow() {
    Camera camera;
    camera.position = glm::vec3(-1.0f, 4.5f, 3.0f);
    camera.orientation = glm::angleAxis(glm::radians(-40.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    return camera;
}

// Shadowed pixels are those a frame changes when shadows turn on. A miss is
// a shadowed pixel on one path with no shadowed pixel within 1 px on the
// other: a texel edge can land a pixel apart, since GL rasterizes casters by
// its own rules. Inside the shadow (3x3 shadowed on both paths) the colours
// are compared as the background band compares them.
struct ShadowMatch {
    size_t cpu_shadowed = 0, gl_shadowed = 0;
    size_t cpu_misses = 0, gl_misses = 0;
    size_t interior = 0;
    int max_interior = 0;
};

[[nodiscard]] ShadowMatch match_shadows(const std::vector<uint8_t>& cpu_on, const std::vector<uint8_t>& cpu_off,
                                        const std::vector<uint8_t>& gl_on, const std::vector<uint8_t>& gl_off) {
    ShadowMatch m;
    std::vector<uint8_t> cpu_mask(kPixels), gl_mask(kPixels);
    for (size_t p = 0; p < kPixels; ++p) {
        cpu_mask[p] = static_cast<uint8_t>(same_pixel(cpu_on, cpu_off, p) ? 0 : 1);
        gl_mask[p] = static_cast<uint8_t>(same_pixel(gl_on, gl_off, p) ? 0 : 1);
        m.cpu_shadowed += cpu_mask[p];
        m.gl_shadowed += gl_mask[p];
    }
    const auto count_round = [](const std::vector<uint8_t>& mask, int x, int y) {
        int n = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int nx = std::clamp(x + dx, 0, kWidth - 1), ny = std::clamp(y + dy, 0, kHeight - 1);
                n += mask[static_cast<size_t>(ny) * kWidth + static_cast<size_t>(nx)];
            }
        }
        return n;
    };
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const size_t p = static_cast<size_t>(y) * kWidth + static_cast<size_t>(x);
            if (cpu_mask[p] != 0u && count_round(gl_mask, x, y) == 0) ++m.cpu_misses;
            if (gl_mask[p] != 0u && count_round(cpu_mask, x, y) == 0) ++m.gl_misses;
            if (count_round(cpu_mask, x, y) == 9 && count_round(gl_mask, x, y) == 9) {
                ++m.interior;
                for (size_t ch = 0; ch < 3u; ++ch) {
                    m.max_interior = std::max(
                        m.max_interior, std::abs(static_cast<int>(gl_on[p * 4u + ch]) - static_cast<int>(cpu_on[p * 4u + ch])));
                }
            }
        }
    }
    return m;
}

// GL samples the CPU's own static shadow map, uploaded as a texture, with a
// port of sample_shadow(). Pinned like the other GL bands.
TEST_F(GpuGlRenderer, StaticShadowsMatchTheCpuWithinTheirBand) {
    // Measured 2026-10-05 on rendering/gl-shadows, 160x120, on an NVIDIA
    // GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83): 776 shadowed pixels
    // on each path, 0 misses either way, and 0 levels over the 648 interior
    // ones. The map is the CPU's own data. Pinned at the measurement; another
    // device may differ, so re-measure there before widening (03-verification).
    constexpr size_t kBandMisses = 0;
    constexpr int kBandInterior = 0;

    const RenderScene scene = make_shadow_scene(/*dynamic_caster=*/false);
    RenderOptions on = comparable_options();
    on.shadows = true;
    const RenderOptions off = comparable_options();
    const Camera camera = camera_over_shadow();
    const ShadowMatch m = match_shadows(cpu_rgba(scene, camera, on), cpu_rgba(scene, camera, off),
                                        draw_and_read(scene, on, camera), draw_and_read(scene, off, camera));
    ASSERT_GT(m.cpu_shadowed, kPixels / 40u) << "the CPU frame must hold a shadow";
    RecordProperty("cpu_shadowed", static_cast<int>(m.cpu_shadowed));
    RecordProperty("gl_shadowed", static_cast<int>(m.gl_shadowed));
    RecordProperty("max_interior", m.max_interior);
    RecordProperty("interior", static_cast<int>(m.interior));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    EXPECT_LE(m.cpu_misses, kBandMisses) << m.cpu_misses << " of " << m.cpu_shadowed
                                         << " CPU shadow pixels have no GL shadow within 1 px, on " << device;
    EXPECT_LE(m.gl_misses, kBandMisses) << m.gl_misses << " of " << m.gl_shadowed
                                        << " GL shadow pixels have no CPU shadow within 1 px, on " << device;
    EXPECT_LE(m.max_interior, kBandInterior) << "inside the shadow (" << m.interior
                                             << " pixels) GL differs from the CPU by up to " << m.max_interior
                                             << " levels, on " << device;
}

// A dynamic caster: GL copies the static map and rasterizes the box into the
// copy, keeping the larger light-space z, as the CPU does each frame.
TEST_F(GpuGlRenderer, DynamicCasterShadowsMatchTheCpuWithinTheirBand) {
    // Measured 2026-10-05 as above, same device: 776 shadowed pixels on each
    // path, 0 misses either way, 0 levels over 648 interior pixels. GL's own
    // caster texels land where the CPU's do at this map size. Pinned at the
    // measurement.
    constexpr size_t kBandMisses = 0;
    constexpr int kBandInterior = 0;

    const RenderScene scene = make_shadow_scene(/*dynamic_caster=*/true);
    RenderOptions on = comparable_options();
    on.shadows = true;
    const RenderOptions off = comparable_options();
    const Camera camera = camera_over_shadow();
    const ShadowMatch m = match_shadows(cpu_rgba(scene, camera, on), cpu_rgba(scene, camera, off),
                                        draw_and_read(scene, on, camera), draw_and_read(scene, off, camera));
    ASSERT_GT(m.cpu_shadowed, kPixels / 40u) << "the CPU frame must hold the dynamic box's shadow";
    RecordProperty("cpu_shadowed", static_cast<int>(m.cpu_shadowed));
    RecordProperty("gl_shadowed", static_cast<int>(m.gl_shadowed));
    RecordProperty("max_interior", m.max_interior);
    RecordProperty("interior", static_cast<int>(m.interior));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    EXPECT_LE(m.cpu_misses, kBandMisses) << m.cpu_misses << " of " << m.cpu_shadowed
                                         << " CPU shadow pixels have no GL shadow within 1 px, on " << device;
    EXPECT_LE(m.gl_misses, kBandMisses) << m.gl_misses << " of " << m.gl_shadowed
                                        << " GL shadow pixels have no CPU shadow within 1 px, on " << device;
    EXPECT_LE(m.max_interior, kBandInterior) << "inside the shadow (" << m.interior
                                             << " pixels) GL differs from the CPU by up to " << m.max_interior
                                             << " levels, on " << device;
}

// ---------------------------------------------------------------------------
// CSG subtrees, ray-marched (RS3, replacement signed 2026-10-05;
// rendering/plans/2026-10-04-b2-raymarched-csg-plan.md, step 3). GL marches
// each subtree within its box in shaded and velocity modes, as raster_cpu
// does, and draws its mesh only in wireframe. Scenes come through
// scene_from_world(), the only path that makes subtrees, under a black sky,
// so "not black" means "covered".
// ---------------------------------------------------------------------------

// Rotates a Y-axis cylinder onto Z.
const glm::quat kCsgYToZ(0.70710678f, 0.70710678f, 0.0f, 0.0f);

[[nodiscard]] spade::WorldBuilder csg_builder() {
    spade::WorldBuilder b;
    b.name("gpu-gl-csg-test").capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    return b;
}

[[nodiscard]] RenderScene csg_scene_or_fail(spade::WorldBuilder& b) {
    const spade::Result<spade::WorldDesc> world = b.build();
    if (!world) {
        ADD_FAILURE() << "build failed: " << world.error().context;
        return RenderScene{};
    }
    // The scene keeps no pointer into `world`: each subtree's program is its own copy.
    spade::Result<RenderScene> scene = spade::render::scene_from_world(*world, {});
    if (!scene) {
        ADD_FAILURE() << "scene_from_world failed: " << scene.error().context;
        return RenderScene{};
    }
    scene->lighting.sky_zenith = glm::vec3(0.0f);
    scene->lighting.sky_horizon = glm::vec3(0.0f);
    return std::move(*scene);
}

// A red slab with a round hole: one subtree, every node one material.
void add_slab_with_hole(spade::WorldBuilder& b, uint32_t material, glm::vec3 centre) {
    b.box(glm::vec3(1.0f, 1.0f, 0.1f), spade::SdfPose{.position = centre}).material_for_last_node(material);
    b.cylinder(0.4f, 1.0f, spade::SdfPose{.position = centre, .rotation = kCsgYToZ}).material_for_last_node(material);
    b.subtract().material_for_last_node(material);
}

// What each frame covers, and how the two agree: a covered pixel misses
// when the other frame covers nothing within 1 px. Where both cover the
// whole 3x3 round a pixel, the colours are compared, as the background band
// compares them; `interior_over_2` counts those more than 2 levels apart.
struct SurfaceMatch {
    size_t cpu_covered = 0, gl_covered = 0;
    size_t cpu_misses = 0, gl_misses = 0;
    size_t interior = 0, interior_over_2 = 0;
    int max_interior = 0;
};

[[nodiscard]] SurfaceMatch match_surfaces(const std::vector<uint8_t>& cpu, const std::vector<uint8_t>& gl) {
    SurfaceMatch m;
    if (cpu.size() != kPixels * 4u || gl.size() != kPixels * 4u) {
        ADD_FAILURE() << "a frame is missing";
        return m;
    }
    std::vector<uint8_t> cpu_mask(kPixels), gl_mask(kPixels);
    for (size_t p = 0; p < kPixels; ++p) {
        cpu_mask[p] = static_cast<uint8_t>(is_black(cpu, p) ? 0 : 1);
        gl_mask[p] = static_cast<uint8_t>(is_black(gl, p) ? 0 : 1);
        m.cpu_covered += cpu_mask[p];
        m.gl_covered += gl_mask[p];
    }
    const auto count_round = [](const std::vector<uint8_t>& mask, int x, int y) {
        int n = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int nx = std::clamp(x + dx, 0, kWidth - 1), ny = std::clamp(y + dy, 0, kHeight - 1);
                n += mask[static_cast<size_t>(ny) * kWidth + static_cast<size_t>(nx)];
            }
        }
        return n;
    };
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const size_t p = static_cast<size_t>(y) * kWidth + static_cast<size_t>(x);
            if (cpu_mask[p] != 0u && count_round(gl_mask, x, y) == 0) ++m.cpu_misses;
            if (gl_mask[p] != 0u && count_round(cpu_mask, x, y) == 0) ++m.gl_misses;
            if (count_round(cpu_mask, x, y) == 9 && count_round(gl_mask, x, y) == 9) {
                ++m.interior;
                int worst = 0;
                for (size_t ch = 0; ch < 3u; ++ch) {
                    worst = std::max(worst, std::abs(static_cast<int>(gl[p * 4u + ch]) - static_cast<int>(cpu[p * 4u + ch])));
                }
                m.max_interior = std::max(m.max_interior, worst);
                m.interior_over_2 += worst > 2 ? 1u : 0u;
            }
        }
    }
    return m;
}

// RED before step 3. Shaded and velocity frames draw a subtree without its
// mesh: with the mesh emptied, GL still shows the slab where the CPU does.
TEST_F(GpuGlRenderer, CsgSubtreesDrawWithoutTheirMesh) {
    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{.name = "red", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}});
    add_slab_with_hole(b, 0, glm::vec3(0.0f));
    RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    scene.meshes[scene.statics[scene.csg_subtrees[0].draw_item].mesh_index].indices.clear();
    Camera camera;
    camera.position = glm::vec3(0.3f, 0.2f, 3.0f);

    for (const DrawMode mode : {DrawMode::shaded, DrawMode::velocity}) {
        SCOPED_TRACE(mode == DrawMode::shaded ? "shaded" : "velocity");
        RenderOptions options = comparable_options();
        options.mode = mode;
        const SurfaceMatch m = match_surfaces(cpu_rgba(scene, camera, options), draw_and_read(scene, options, camera));
        ASSERT_GT(m.cpu_covered, kPixels / 10u) << "the CPU frame must show the slab";
        EXPECT_LE(m.cpu_misses, kPixels / 1000u) << "GL shows " << m.gl_covered << " of the CPU's " << m.cpu_covered
                                                 << " slab pixels with the mesh emptied";
        EXPECT_LE(m.gl_misses, kPixels / 1000u);
    }
}

// RED before step 3. A concentric shell whose 0.02 m wall is under one mesh
// cell: the mesh loses it, and from inside GL shows holes. A march does not.
TEST_F(GpuGlRenderer, AThinCsgWallDrawsWithoutHolesFromInside) {
    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{
        .name = "white", .base_color = {1.0f, 1.0f, 1.0f, 1.0f}, .shading = spade::MaterialShading::unlit});
    b.sphere(4.0f).material_for_last_node(0).sphere(3.98f).material_for_last_node(0).subtract().material_for_last_node(0);
    const RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    const Camera camera;  // at the centre: every ray meets the inner wall

    const size_t cpu = count_non_black(cpu_rgba(scene, camera, comparable_options()));
    ASSERT_EQ(cpu, kPixels) << "the CPU must see the wall in every pixel";
    const size_t gl = count_non_black(draw_and_read(scene, comparable_options(), camera));
    EXPECT_EQ(gl, kPixels) << "GL shows " << (kPixels - gl) << " holes in a 0.02 m wall";
}

// Wireframe has no surface to outline in a march, so it keeps the mesh (RS3).
TEST_F(GpuGlRenderer, WireframeStillDrawsTheCsgMesh) {
    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{.name = "red", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}});
    add_slab_with_hole(b, 0, glm::vec3(0.0f));
    RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    Camera camera;
    camera.position = glm::vec3(0.3f, 0.2f, 3.0f);
    RenderOptions wireframe = comparable_options();
    wireframe.mode = DrawMode::wireframe;

    EXPECT_GT(count_non_black(draw_and_read(scene, wireframe, camera)), 0u) << "wireframe draws the mesh's edges";
    scene.meshes[scene.statics[scene.csg_subtrees[0].draw_item].mesh_index].indices.clear();
    EXPECT_EQ(count_non_black(draw_and_read(scene, wireframe, camera)), 0u) << "and only the mesh";
}

// GL against the CPU on a lit scene with shadows: the slab, a smooth-union
// blob, a tessellated ball in front of the slab casting on it, and a ball
// behind it seen through the hole. Depth both ways, shading, the shadow
// lookup on a marched surface, and the SR-17a blend.
//
// The balls are unlit. A lit mesh is Gouraud-shaded on the CPU (SR-18) and
// per pixel on GL, which the GL options plan accepted (RND-3). Near the
// terminator that alone moves a lit ball's pixels by up to 11 levels here,
// and it would hide what this band is for. Both paths shade a marched
// surface per pixel.
TEST_F(GpuGlRenderer, CsgMatchesTheCpuWithinItsBand) {
    // Measured 2026-10-05 on rendering/b2-gl, 160x120, on an NVIDIA GeForce
    // RTX 3060 Ti (OpenGL 4.3.0, driver 572.83): 2709 covered pixels on each
    // path, 0 misses either way, and at most 1 level apart over the 2335
    // interior ones. Pinned at the measurement; another device may differ,
    // so re-measure there before widening (03-verification).
    constexpr size_t kBandMisses = 0;
    constexpr int kBandInterior = 1;

    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{.name = "red", .base_color = {0.8f, 0.3f, 0.2f, 1.0f}});
    b.material(spade::MaterialDesc{.name = "blue", .base_color = {0.2f, 0.35f, 0.8f, 1.0f}});
    b.material(spade::MaterialDesc{
        .name = "green", .base_color = {0.25f, 0.7f, 0.3f, 1.0f}, .shading = spade::MaterialShading::unlit});
    add_slab_with_hole(b, 0, glm::vec3(-0.6f, 0.8f, 0.0f));
    b.sphere(0.45f, spade::SdfPose{.position = {1.2f, 0.6f, 0.2f}}).material_for_last_node(1);
    b.sphere(0.3f, spade::SdfPose{.position = {1.6f, 1.0f, 0.2f}}).material_for_last_node(1);
    b.smooth_union(0.25f).material_for_last_node(1);
    b.union_();
    b.sphere(0.3f, spade::SdfPose{.position = {-0.1f, 0.3f, 0.9f}}).material_for_last_node(2);
    b.union_();
    b.sphere(0.5f, spade::SdfPose{.position = {-0.6f, 0.8f, -1.5f}}).material_for_last_node(2);
    b.union_();
    const RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.csg_subtrees.size(), 2u);
    ASSERT_TRUE(scene.static_shadow.has_value());
    Camera camera;
    camera.position = glm::vec3(0.2f, 0.9f, 4.5f);
    RenderOptions options = comparable_options();
    options.shadows = true;

    const SurfaceMatch m = match_surfaces(cpu_rgba(scene, camera, options), draw_and_read(scene, options, camera));
    RecordProperty("cpu_covered", static_cast<int>(m.cpu_covered));
    RecordProperty("gl_covered", static_cast<int>(m.gl_covered));
    RecordProperty("cpu_misses", static_cast<int>(m.cpu_misses));
    RecordProperty("gl_misses", static_cast<int>(m.gl_misses));
    RecordProperty("interior", static_cast<int>(m.interior));
    RecordProperty("interior_over_2", static_cast<int>(m.interior_over_2));
    RecordProperty("max_interior", m.max_interior);
    ASSERT_GT(m.cpu_covered, kPixels / 10u) << "the CPU frame must show the scene";
    EXPECT_LE(m.cpu_misses, kBandMisses) << m.cpu_misses << " of the CPU's " << m.cpu_covered
                                         << " covered pixels have no GL surface within 1 px, on "
                                         << renderer_->renderer_name();
    EXPECT_LE(m.gl_misses, kBandMisses) << m.gl_misses << " of GL's " << m.gl_covered
                                        << " covered pixels have no CPU surface within 1 px";
    EXPECT_LE(m.max_interior, kBandInterior)
        << "interior pixels differ by up to " << m.max_interior << " levels; " << m.interior_over_2 << " of "
        << m.interior << " by more than 2";
}

// Pixels whose green channel dominates: the unlit green ball, here.
[[nodiscard]] size_t count_green(const std::vector<uint8_t>& rgba) {
    size_t n = 0;
    for (size_t p = 0; p < rgba.size() / 4u; ++p) {
        n += (rgba[p * 4u + 1u] > rgba[p * 4u] && rgba[p * 4u + 1u] > rgba[p * 4u + 2u]) ? 1u : 0u;
    }
    return n;
}

// A mesh partly inside a subtree's box but behind its surface stays hidden:
// the march writes the surface's depth, not the proxy's. Were the proxy's
// back face the depth, the ball's front (inside the box) would be nearer and
// show through the slab. The other CSG cases keep every mesh wholly in front
// of or behind a box, so they cannot see this.
TEST_F(GpuGlRenderer, AMeshInsideASubtreesBoxStaysBehindItsSurface) {
    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{.name = "red", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}});
    b.material(spade::MaterialDesc{
        .name = "green", .base_color = {0.1f, 0.9f, 0.1f, 1.0f}, .shading = spade::MaterialShading::unlit});
    add_slab_with_hole(b, 0, glm::vec3(0.0f));
    // Behind the slab (front face z = +0.1), clear of the hole, its front at z = -0.05.
    b.sphere(0.25f, spade::SdfPose{.position = {0.5f, 0.55f, -0.3f}}).material_for_last_node(1);
    b.union_();
    const RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    ASSERT_LT(scene.csg_subtrees[0].bounds.min.z, -0.05f) << "the ball's front must lie inside the slab's box";
    Camera camera;
    camera.position = glm::vec3(0.3f, 0.2f, 3.0f);

    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, comparable_options());
    ASSERT_EQ(count_green(cpu), 0u) << "the CPU must hide the ball behind the slab";
    ASSERT_GT(count_non_black(cpu), kPixels / 10u);
    EXPECT_EQ(count_green(draw_and_read(scene, comparable_options(), camera)), 0u)
        << "GL shows the ball through the slab: the march's depth is not the surface's";
}

// The proxy is the box's back faces, and depth clamp keeps the parts of them
// past the far plane. With far between the slab and its box's back, GL must
// still show the slab where the CPU does.
TEST_F(GpuGlRenderer, ASubtreeDrawsWhereItsBoxCrossesTheFarPlane) {
    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{.name = "red", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}});
    add_slab_with_hole(b, 0, glm::vec3(0.0f));
    const RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    Camera camera;
    camera.position = glm::vec3(0.3f, 0.2f, 3.0f);
    camera.far_plane = 3.0f;  // the slab's front is 2.9 m away, its box's back more than 3.1 m
    ASSERT_GT(camera.position.z - scene.csg_subtrees[0].bounds.min.z, camera.far_plane);

    const SurfaceMatch m =
        match_surfaces(cpu_rgba(scene, camera, comparable_options()), draw_and_read(scene, comparable_options(), camera));
    ASSERT_GT(m.cpu_covered, kPixels / 10u) << "the CPU frame must show the slab";
    EXPECT_LE(m.cpu_misses, kPixels / 1000u) << "GL shows " << m.gl_covered << " of the CPU's " << m.cpu_covered
                                             << " slab pixels with the far plane through the box";
    EXPECT_LE(m.gl_misses, kPixels / 1000u);
}

// A one-subtree program by hand: `leaves` spheres, then unions. Pushed all
// first, the stack peaks at `leaves`; alternated, at 2.
[[nodiscard]] spade::SdfProgram sphere_chain(uint32_t leaves, bool all_first) {
    spade::SdfProgram program;
    program.transforms = {spade::SdfTransform{}};
    const spade::SdfNode sphere{.kind = static_cast<uint32_t>(spade::SdfPrim::sphere), .params = {0.5f, 0.0f, 0.0f, 0.0f}};
    const spade::SdfNode unite{.op = static_cast<uint32_t>(spade::SdfOp::union_)};
    if (all_first) {
        program.nodes.assign(leaves, sphere);
        program.nodes.insert(program.nodes.end(), leaves - 1u, unite);
    } else {
        program.nodes.push_back(sphere);
        for (uint32_t i = 1; i < leaves; ++i) {
            program.nodes.push_back(sphere);
            program.nodes.push_back(unite);
        }
    }
    return program;
}

[[nodiscard]] RenderScene slab_scene() {
    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{.name = "red", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}});
    add_slab_with_hole(b, 0, glm::vec3(0.0f));
    return csg_scene_or_fail(b);
}

// L6: a subtree with no program, or pointing past the statics, would vanish
// from the frame in silence. upload_scene() refuses both.
TEST_F(GpuGlRenderer, RefusesAnEmptyOrUnattachedCsgSubtree) {
    const RenderScene good = slab_scene();
    ASSERT_EQ(good.csg_subtrees.size(), 1u);
    {
        RenderScene scene = good;
        scene.csg_subtrees[0].program = spade::SdfProgram{};
        const spade::Result<void> up = renderer_->upload_scene(scene);
        ASSERT_FALSE(up.has_value()) << "an empty subtree program must be refused";
        EXPECT_EQ(up.error().code, spade::Code::invalid_argument);
    }
    {
        RenderScene scene = good;
        scene.csg_subtrees[0].draw_item = static_cast<uint32_t>(scene.statics.size());
        const spade::Result<void> up = renderer_->upload_scene(scene);
        ASSERT_FALSE(up.has_value()) << "a subtree whose draw item is past the statics must be refused";
        EXPECT_EQ(up.error().code, spade::Code::invalid_argument);
    }
}

// validate()'s own code comes through: a program needing more stack than
// kMaxSdfDepth is capacity_exceeded, not invalid_argument.
TEST_F(GpuGlRenderer, PassesThroughAProgramsCapacityRefusal) {
    RenderScene scene = slab_scene();
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    scene.csg_subtrees[0].program = sphere_chain(spade::kMaxSdfDepth + 1u, /*all_first=*/true);
    const spade::Result<void> up = renderer_->upload_scene(scene);
    ASSERT_FALSE(up.has_value());
    EXPECT_EQ(up.error().code, spade::Code::capacity_exceeded) << up.error().context;
}

// A subtree past GL's node cap is refused, at the cap it is not. The cap
// bounds one pixel's march so a frame stays inside the driver's timeout.
TEST_F(GpuGlRenderer, RefusesASubtreePastTheNodeCap) {
    RenderScene scene = slab_scene();
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    constexpr uint32_t kCap = GlRenderer::kMaxCsgSubtreeNodes;
    static_assert(kCap % 2u == 0u, "a sphere chain has an odd node count");
    scene.csg_subtrees[0].program = sphere_chain(kCap / 2u, /*all_first=*/false);  // kCap - 1 nodes
    EXPECT_TRUE(renderer_->upload_scene(scene).has_value()) << "under the cap";
    scene.csg_subtrees[0].program = sphere_chain(kCap / 2u + 1u, /*all_first=*/false);  // kCap + 1 nodes
    const spade::Result<void> up = renderer_->upload_scene(scene);
    ASSERT_FALSE(up.has_value()) << "past the cap";
    EXPECT_EQ(up.error().code, spade::Code::capacity_exceeded) << up.error().context;
}

// A refused upload changes nothing: the scene uploaded before it still draws
// the same frame. Refusing after replacing the meshes, or with the subtree
// table half filled, would leave GL drawing a mix of the two scenes.
TEST_F(GpuGlRenderer, ARefusedUploadLeavesThePreviousSceneDrawing) {
    const RenderScene good = slab_scene();
    Camera camera;
    camera.position = glm::vec3(0.3f, 0.2f, 3.0f);
    const std::vector<uint8_t> before = draw_and_read(good, comparable_options(), camera);
    ASSERT_GT(count_non_black(before), kPixels / 10u);

    RenderScene bad = make_scene(make_single_triangle(false));  // other meshes and materials
    bad.csg_subtrees.push_back(spade::render::CsgSubtree{});  // no program
    ASSERT_FALSE(renderer_->upload_scene(bad).has_value());

    const std::vector<uint8_t> after = draw_uploaded_and_read(good, comparable_options(), camera);
    ASSERT_EQ(after.size(), before.size()) << "the scene uploaded before the refusal no longer draws";
    EXPECT_EQ(count_differing(before, after), 0u);
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

// L6: a static shadow map whose depth array is not size x size cannot be
// sampled (the CPU would read past it), so upload_scene() refuses it rather
// than drawing the scene unshadowed in silence.
TEST_F(GpuGlRenderer, RefusesAMalformedShadowMap) {
    RenderScene scene = make_scene(make_single_triangle(false));
    spade::render::ShadowMap map;
    map.size = 4;
    map.depth.assign(3, spade::render::kNoOccluder);  // 16 texels needed
    scene.static_shadow = map;
    const spade::Result<void> up = renderer_->upload_scene(scene);
    ASSERT_FALSE(up.has_value()) << "a 3-texel map for a 4 x 4 size must be refused";
    EXPECT_EQ(up.error().code, spade::Code::invalid_argument);
}

// draw() sets every state it depends on: blending and depth clamp left on by
// the caller (or by a GL that does not reset them) change nothing.
TEST_F(GpuGlRenderer, TheCallersBlendAndDepthClampStateDoNotReachTheFrame) {
    const RenderScene scene = make_ground_scene();
    RenderOptions options = comparable_options();
    options.overlays = true;
    const std::vector<uint8_t> clean = draw_and_read(scene, options, camera_over_ground());

    glEnable(GL_BLEND);
    glBlendFunc(GL_ZERO, GL_ZERO);  // would turn every fragment black
    glEnable(GL_DEPTH_CLAMP);
    const std::vector<uint8_t> dirty = draw_and_read(scene, options, camera_over_ground());
    glDisable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ZERO);
    glDisable(GL_DEPTH_CLAMP);
    ASSERT_EQ(dirty.size(), clean.size());
    EXPECT_EQ(count_differing(clean, dirty), 0u) << "the caller's blend or depth-clamp state reached the frame";
}

// L6: what GL does not draw is announced by name, so a caller can show it or
// refuse GL. Host-only: no context needed, so this suite has no gpu label.
// A loader that finds nothing is refused with unavailable, naming the first
// missing entry point, before any GL call: with a table of null pointers, a
// call would crash. Needs no context, so an out-of-tree consumer can check
// that render_gl links without opening a window.
TEST(GlRendererCreate, ANullLoaderIsRefusedBeforeAnyGlCall) {
    const auto finds_nothing = [](const char*) -> void* { return nullptr; };
    const auto made = GlRenderer::create(finds_nothing);
    ASSERT_FALSE(made.has_value());
    EXPECT_EQ(made.error().code, spade::Code::unavailable);
    EXPECT_NE(made.error().context.find("glGetString"), std::string::npos)
        << "the refusal must name the missing entry point: " << made.error().context;
}

TEST(GlRendererOptions, UnhonouredNamesWhatGlDoesNotDraw) {
    using Names = std::vector<std::string_view>;
    EXPECT_EQ(GlRenderer::unhonoured(RenderOptions{}), Names{}) << "GL draws the defaults' shadows and overlays";

    RenderOptions overlays_only = comparable_options();
    overlays_only.overlays = true;
    overlays_only.spawn_markers = true;
    EXPECT_EQ(GlRenderer::unhonoured(overlays_only), Names{}) << "GL draws every overlay";

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
