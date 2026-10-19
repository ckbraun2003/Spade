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
#include <chrono>
#include <thread>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
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
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/publish.hpp"

#include "cp_sphere.hpp"
#include "flow_synthetic.hpp"
#include "view_mode_cases.hpp"

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

        // Every reuse of the flow textures checked, as a debug build does.
        auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress),
                                       GlRenderer::CreateOptions{.verify_cached_volumes = true});
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

    // The fixture's framebuffer, bound and cleared to black, for a renderer
    // other than renderer_.
    void bind_cleared_target() {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);
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
    // Re-measured 2026-10-06 on rendering/line-gaps, 160x120, on an NVIDIA
    // GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83), after the CPU's
    // rasterizeLine() stopped skipping columns (named cause, TD-2): 2 of the
    // CPU's 3284 overlay pixels miss, at (9, 45) and (28, 55), and 1 of GL's
    // 3282, at (17, 49). All three are grid pixels (grey 90) that the other
    // path draws a pixel further off. Before the fix (2026-10-05, on
    // rendering/gl-overlays): 4 of the CPU's 3277 and 0 of GL's, where a
    // bounds edge crossed grid lines. Gap-free CPU lines agree with GL on
    // those, and add one grid pixel GL places elsewhere.
    // Pinned at the measurement. Another device may differ; re-measure there
    // before widening (03-verification). Known difference, within the band:
    // GL biases each vertex before clipping, and a vertex behind the eye gets
    // no bias, so a grid line crossing the eye plane bends by about 0.2 px
    // against the CPU, which biases after clipping.
    constexpr size_t kBandCpuMisses = 2;
    constexpr size_t kBandGlMisses = 1;

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

// L6: GL marches one flow layer per frame. The demo's layer carries all its
// transfers and iso-surfaces, so it needs only one. More is refused, never
// drawn as fewer.
TEST_F(GpuGlRenderer, RefusesMoreThanOneFlowLayer) {
    RenderScene scene = slab_scene();
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 0.25f, 8, 8, 8,
                                                                   spade::test::uniform_flow(glm::vec3(1.0f, 0.0f, 0.0f)));
    spade::render::TransferFunction speed;
    speed.extinction = {{0.0f, 1.0f}, {2.0f, 1.0f}};
    scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {speed}, {}});
    scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {speed}, {}});
    const spade::Result<void> drew =
        renderer_->draw(scene, camera_on_plus_z(), spade::render::RenderOptions{}, kWidth, kHeight);
    ASSERT_FALSE(drew.has_value()) << "a second flow layer GL cannot draw must be refused";
    EXPECT_EQ(drew.error().code, spade::Code::unavailable);
}

// Per channel, between two frames: the largest difference in levels, how
// many pixels differ at all, and how many by more than two levels.
struct ChannelDiff {
    int worst = 0;
    size_t differing = 0;
    size_t over_two = 0;
};
[[nodiscard]] ChannelDiff channel_diff(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    ChannelDiff d;
    for (size_t p = 0; p < kPixels; ++p) {
        int m = 0;
        for (size_t c = 0; c < 3; ++c) m = std::max(m, std::abs(int{a[p * 4u + c]} - int{b[p * 4u + c]}));
        d.worst = std::max(d.worst, m);
        d.differing += m > 0 ? 1u : 0u;
        d.over_two += m > 2 ? 1u : 0u;
    }
    return d;
}

// The flow goldens' camera, 2.2 m out on +z, and an opaque quad over the left
// half at z = -0.2, through the ring, where the march must stop.
[[nodiscard]] Camera camera_on_ring() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 2.2f);
    return camera;
}
[[nodiscard]] RenderScene ring_cut_scene() {
    RenderScene scene;
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-1.2f, -1.2f, -0.2f), glm::vec3(0.0f, -1.2f, -0.2f), glm::vec3(0.0f, 1.2f, -0.2f),
                      glm::vec3(-1.2f, 1.2f, -0.2f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    scene.meshes.push_back(quad);
    scene.materials = {Material{.base_color = glm::vec4(0.3f, 0.35f, 0.4f, 1.0f)}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)});
    return scene;
}

// Airflow visualization plan step 7c: GL marches a flow layer as the CPU
// does. The flow_vortex_ring golden's layer: vorticity through inferno with a
// ramped extinction, and the dye as smoke, 32^3, through the cut quad.
TEST_F(GpuGlRenderer, FlowVolumeMatchesTheCpuWithinItsBand) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 32.0f, 32, 32, 32,
                                                                   spade::test::vortex_ring_with_dye());
    spade::render::TransferFunction vorticity;
    vorticity.quantity = spade::render::FlowQuantity::vorticity;
    vorticity.colours = spade::render::ColourMapId::inferno;
    vorticity.range_min = 0.0f;
    vorticity.range_max = 22.0f;
    vorticity.extinction = {{2.0f, 0.0f}, {22.0f, 12.0f}};
    spade::render::TransferFunction smoke;
    smoke.quantity = spade::render::FlowQuantity::dye;
    smoke.constant_colour = glm::vec3(0.95f, 0.95f, 0.95f);
    smoke.extinction = {{0.0f, 0.0f}, {1.0f, 15.0f}};
    RenderScene scene = ring_cut_scene();
    scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {vorticity, smoke}, {}});
    RenderScene bare = scene;
    bare.flow_layers.clear();
    const Camera camera = camera_on_ring();

    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, comparable_options());
    const std::vector<uint8_t> cpu_bare = cpu_rgba(bare, camera, comparable_options());
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    ASSERT_EQ(gl.size(), kPixels * 4u) << "GL must draw flow layers now";
    const size_t marched = count_differing(cpu, cpu_bare);
    ASSERT_GT(marched, kPixels / 10u) << "the volume must reach a good part of the frame";
    const ChannelDiff d = channel_diff(cpu, gl);
    RecordProperty("marched_pixels", static_cast<int>(marched));
    RecordProperty("worst_levels", d.worst);
    RecordProperty("differing_pixels", static_cast<int>(d.differing));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ flow volume ] %zu px marched; worst %d levels, %zu pixels differ; on %s\n", marched, d.worst,
                d.differing, device.c_str());
    // Measured 2026-10-12, 160x120, on an NVIDIA GeForce RTX 3060 Ti (OpenGL
    // 4.3.0, driver 572.83): of 3986 marched pixels, 3134 differ, by at most
    // two levels. GLSL's exp() for exp32(), float order in the sums, and the
    // framebuffer's blend in place of the CPU's to_byte() rounding.
    constexpr int kBandWorstLevels = 2;
    constexpr size_t kBandDifferingPixels = 3134;
    EXPECT_LE(d.worst, kBandWorstLevels) << device;
    EXPECT_LE(d.differing, kBandDifferingPixels) << device;
}

// The flow_vortex_ring_q golden's layer: the Q iso at 30% of its peak,
// coloured by speed and lit, 64^3, through the cut quad. A streamline along x
// through the tube's cores tests the iso's depth: the lines pass is
// depth-tested against it on both paths.
TEST_F(GpuGlRenderer, FlowIsoMatchesTheCpuWithinItsBand) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 64.0f, 64, 64, 64,
                                                                   spade::test::vortex_ring_with_dye());
    const float q_hat_peak = volume.header.channel_max[spade::kFlowQHat];
    ASSERT_GT(q_hat_peak, 0.0f);
    spade::render::TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 0.9f;
    spade::render::IsoSurface q_iso;
    q_iso.quantity = spade::render::FlowQuantity::q_hat;
    q_iso.value = 0.5477226f * q_hat_peak;  // sqrt(0.3): Q at 30% of its peak
    q_iso.colour_by = speed;
    RenderScene scene = ring_cut_scene();
    scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {}, {q_iso}});
    spade::render::FlowPolyline line;
    for (int i = 0; i <= 30; ++i) {
        line.points.push_back(glm::vec3(-1.5f + 0.1f * static_cast<float>(i), 0.0f, 0.0f));
        line.colours.push_back(glm::vec3(1.0f));
    }
    scene.flow_polylines.push_back(line);
    RenderScene bare = scene;
    bare.flow_layers.clear();
    const Camera camera = camera_on_ring();

    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, comparable_options());
    const std::vector<uint8_t> cpu_bare = cpu_rgba(bare, camera, comparable_options());
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    ASSERT_EQ(gl.size(), kPixels * 4u) << "GL must draw iso-surfaces now";
    const size_t marched = count_differing(cpu, cpu_bare);
    ASSERT_GT(marched, kPixels / 20u) << "the iso-surface must reach the frame";
    const ChannelDiff d = channel_diff(cpu, gl);
    RecordProperty("marched_pixels", static_cast<int>(marched));
    RecordProperty("worst_levels", d.worst);
    RecordProperty("differing_pixels", static_cast<int>(d.differing));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ flow iso ] %zu px marched; worst %d levels, %zu over two; %zu pixels differ; on %s\n", marched,
                d.worst, d.over_two, d.differing, device.c_str());
    // Measured 2026-10-12, 160x120, on an NVIDIA GeForce RTX 3060 Ti (OpenGL
    // 4.3.0, driver 572.83), on master ae57288 (gap-free CPU lines): 674
    // pixels differ, every one by a single level, and none by more than two.
    // The line hides behind the iso on both. (Before line-gaps, 19 more
    // differed: the CPU's gaps along the streamline.)
    constexpr size_t kBandOverTwoPixels = 0;
    constexpr size_t kBandDifferingPixels = 674;
    EXPECT_LE(d.over_two, kBandOverTwoPixels) << device;
    EXPECT_LE(d.differing, kBandDifferingPixels) << device;
}

// Whether an opt-in gate is set, as test_core_fp16.cpp asks it.
[[nodiscard]] bool env_gate_is_set(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name) != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// Plan step 7c's cost: a flow layer at 128^3, drawn at 1920x1080, against
// the airflow spec section 5 budget of 2 to 4 ms. A measurement, so opt-in
// (SPADE_GL_FLOW_COST=1; about 17 s, most of it building the volume). Each
// frame's time (draw() to glFinish(), median of 8 after 2 warm-ups) includes
// uploading the volume's 32 MB of halves, as a live run does every frame.
// Measured 2026-10-12 on the RTX 3060 Ti (572.83), unskipped, across runs
// under different box load: volume 84 to 165 ms, iso 41 to 83 ms. About 5 ms
// of that is the upload, the depth copy and the rest of the frame (march
// skipped, measured). The march is 20 to 40 times the budget; the lead has
// it (RND-7). The ceiling only catches a pass gone pathological.
TEST_F(GpuGlRenderer, FlowLayerCostAt128CubedAnd1080p) {
    if (!env_gate_is_set("SPADE_GL_FLOW_COST")) {
        GTEST_SKIP() << "a measurement: set SPADE_GL_FLOW_COST=1 to run it";
    }
    constexpr int kW = 1920, kH = 1080;
    GLuint fbo = 0, colour = 0, depth = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenRenderbuffers(1, &colour);
    glBindRenderbuffer(GL_RENDERBUFFER, colour);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, kW, kH);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, kW, kH);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
    ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));
    glViewport(0, 0, kW, kH);

    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 128.0f, 128, 128, 128,
                                                                   spade::test::vortex_ring_with_dye());
    spade::render::TransferFunction vorticity;
    vorticity.quantity = spade::render::FlowQuantity::vorticity;
    vorticity.colours = spade::render::ColourMapId::inferno;
    vorticity.range_min = 0.0f;
    vorticity.range_max = 22.0f;
    vorticity.extinction = {{2.0f, 0.0f}, {22.0f, 12.0f}};
    spade::render::TransferFunction smoke;
    smoke.quantity = spade::render::FlowQuantity::dye;
    smoke.constant_colour = glm::vec3(0.95f, 0.95f, 0.95f);
    smoke.extinction = {{0.0f, 0.0f}, {1.0f, 15.0f}};
    spade::render::TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 0.9f;
    spade::render::IsoSurface q_iso;
    q_iso.quantity = spade::render::FlowQuantity::q_hat;
    q_iso.value = 0.5477226f * volume.header.channel_max[spade::kFlowQHat];
    q_iso.colour_by = speed;

    RenderScene volume_scene = ring_cut_scene();
    volume_scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {vorticity, smoke}, {}});
    RenderScene iso_scene = ring_cut_scene();
    iso_scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {}, {q_iso}});
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    const auto median_ms = [&](const RenderScene& scene, bool skip,
                               GlRenderer::FlowScale scale = GlRenderer::FlowScale::full) {
        renderer_->set_flow_quality(GlRenderer::FlowQuality{.skip_empty_space = skip, .scale = scale});
        EXPECT_TRUE(renderer_->upload_scene(scene).has_value());
        std::vector<double> ms;
        for (int i = 0; i < 10; ++i) {
            glFinish();
            const auto t0 = std::chrono::steady_clock::now();
            const spade::Result<void> drew =
                renderer_->draw(scene, camera_on_ring(), comparable_options(), static_cast<uint32_t>(kW),
                                static_cast<uint32_t>(kH));
            glFinish();
            const auto t1 = std::chrono::steady_clock::now();
            EXPECT_TRUE(drew.has_value());
            if (i >= 2) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        std::sort(ms.begin(), ms.end());
        return 0.5 * (ms[3] + ms[4]);
    };
    const double volume_full_ms = median_ms(volume_scene, false);
    const double iso_full_ms = median_ms(iso_scene, false);
    const double volume_ms = median_ms(volume_scene, true);
    const double iso_ms = median_ms(iso_scene, true);
    const double volume_half_ms = median_ms(volume_scene, true, GlRenderer::FlowScale::half);
    const double iso_half_ms = median_ms(iso_scene, true, GlRenderer::FlowScale::half);
    RecordProperty("volume_half_ms_x100", static_cast<int>(volume_half_ms * 100.0));
    RecordProperty("iso_half_ms_x100", static_cast<int>(iso_half_ms * 100.0));
    std::printf("[ flow cost ] at half scale: volume %.2f ms, iso %.2f ms\n", volume_half_ms, iso_half_ms);
    RecordProperty("volume_ms_x100", static_cast<int>(volume_ms * 100.0));
    RecordProperty("iso_ms_x100", static_cast<int>(iso_ms * 100.0));
    RecordProperty("volume_unskipped_ms_x100", static_cast<int>(volume_full_ms * 100.0));
    RecordProperty("iso_unskipped_ms_x100", static_cast<int>(iso_full_ms * 100.0));
    std::printf("[ flow cost ] 128^3 at 1920x1080: volume %.2f ms (%.2f unskipped), iso %.2f ms (%.2f unskipped); "
                "on %s\n",
                volume_ms, volume_full_ms, iso_ms, iso_full_ms, device.c_str());
    EXPECT_LT(volume_full_ms, 1000.0) << device;
    EXPECT_LT(iso_full_ms, 1000.0) << device;
    // Plan steps 7e and 7f, each against the slower march in the same run.
    // Measured 2026-10-12 with the box quiet: skipping took the volume to
    // 0.6 of unskipped and the iso to 0.35. The volume skips less because its
    // smoke cannot be skipped exactly: the dye blob's Gaussian tail stays
    // above fp16's smallest subnormal out to 0.82 m, so a third of the box
    // still absorbs a little. Half scale took both to about 0.47 of skipped
    // full. With other realms building, the ratios drift (skipping 0.57 to
    // 0.71, half up to 0.56), so these bounds only hold each saving to be
    // clearly real; the numbers themselves are in the run's properties.
    EXPECT_LT(volume_ms, 0.9 * volume_full_ms) << device;
    EXPECT_LT(iso_ms, 0.9 * iso_full_ms) << device;
    EXPECT_LT(volume_half_ms, 0.75 * volume_ms) << device;
    EXPECT_LT(iso_half_ms, 0.75 * iso_ms) << device;
    renderer_->set_flow_quality({});

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);
}

// lambda2's iso on GL. The ring's cores are straight Lamb-Oseen vortices
// locally, a plane flow where lambda2 = -Q, so the ring carries lambda2-hat =
// -q-hat (flow_synthetic.hpp). The lambda2-hat iso at -L must then draw
// GL's Q iso at +L bit for bit: negation is exact through the halves, the
// lerps and the crossing search, and the normal is turned to the eye. So
// FlowIsoMatchesTheCpuWithinItsBand's band holds for lambda2 as it is.
TEST_F(GpuGlRenderer, FlowLambda2IsoDrawsTheQIsoOfAStraightCore) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 64.0f, 64, 64, 64,
                                                                   spade::test::vortex_ring_with_dye());
    const float q_hat_peak = volume.header.channel_max[spade::kFlowQHat];
    ASSERT_GT(q_hat_peak, 0.0f);
    spade::render::TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 0.9f;
    spade::render::IsoSurface q_iso;
    q_iso.quantity = spade::render::FlowQuantity::q_hat;
    q_iso.value = 0.5477226f * q_hat_peak;
    q_iso.colour_by = speed;
    spade::render::IsoSurface l2_iso = q_iso;
    l2_iso.quantity = spade::render::FlowQuantity::lambda2_hat;
    l2_iso.value = -q_iso.value;
    RenderScene by_q = ring_cut_scene();
    by_q.flow_layers.push_back(spade::render::FlowLayer{&volume, {}, {q_iso}});
    RenderScene by_lambda2 = ring_cut_scene();
    by_lambda2.flow_layers.push_back(spade::render::FlowLayer{&volume, {}, {l2_iso}});
    RenderScene bare = ring_cut_scene();
    const Camera camera = camera_on_ring();
    const std::vector<uint8_t> want = draw_and_read(by_q, comparable_options(), camera);
    const std::vector<uint8_t> got = draw_and_read(by_lambda2, comparable_options(), camera);
    const std::vector<uint8_t> nothing = draw_and_read(bare, comparable_options(), camera);
    ASSERT_EQ(got.size(), kPixels * 4u);
    EXPECT_GT(count_differing(got, nothing), kPixels / 20u) << "the lambda2 iso must reach the frame";
    EXPECT_EQ(count_differing(got, want), 0u) << "lambda2's iso must draw exactly as Q's on a plane flow";
}

// --- RND-8 on GL: flow walls, and the value read off the wall ---------------

// A pressure field that jumps across z = 0 like a sheet's: -1 below, +1
// above, between the cell centres at z = -0.125 and 0.125 m.
[[nodiscard]] spade::FlowVolume sheet_pressure() {
    return spade::test::make_flow_volume(glm::vec3(-2.0f, -2.0f, -1.0f), 0.25f, 16, 16, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowPressure] = p.z > 0.0f ? 1.0f : -1.0f;
        return c;
    });
}

// Walls only, one cell off the wall, cool-warm over the sheet's [-1, 1].
[[nodiscard]] spade::render::FlowSurfaceColouring walls_colouring(const spade::FlowVolume& volume) {
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::pressure;
    colours.colours = spade::render::ColourMapId::cool_warm;
    colours.range_min = -1.0f;
    colours.range_max = 1.0f;
    spade::render::FlowSurfaceColouring colouring{&volume, colours};
    colouring.walls_only = true;
    colouring.offset_cells = 1.0f;
    return colouring;
}

// A frame with a flow wall in its left half and a surface that is not in its
// right, coloured by walls_colouring(): GL against its own bare frame, which
// needs no CPU, and against the CPU.
struct FlowWalls {
    size_t coloured = 0;        // the CPU's coloured pixels
    size_t right_coloured = 0;  // GL's in the right half: 0 when only walls take the colour
    size_t row_wall = 0;        // GL's coloured pixels on the middle row
    size_t row_off = 0;         // of those, more than 2 levels from the seen side's +1
    ChannelDiff band;           // GL against the CPU
};

[[nodiscard]] FlowWalls flow_walls(const std::vector<uint8_t>& gl, const std::vector<uint8_t>& gl_bare,
                                   const std::vector<uint8_t>& cpu, const std::vector<uint8_t>& cpu_bare) {
    FlowWalls w;
    if (gl.size() != kPixels * 4u || gl_bare.size() != kPixels * 4u) {
        ADD_FAILURE() << "a GL frame is missing";
        return w;
    }
    w.coloured = count_differing(cpu, cpu_bare);
    const glm::vec3 plus_one = 255.0f * spade::render::colour_map_at(spade::render::ColourMapId::cool_warm, 1.0f);
    for (size_t y = 0; y < static_cast<size_t>(kHeight); ++y) {
        for (size_t x = 0; x < static_cast<size_t>(kWidth); ++x) {
            const size_t p = y * kWidth + x;
            if (same_pixel(gl, gl_bare, p)) continue;
            if (x >= static_cast<size_t>(kWidth) / 2u) {
                ++w.right_coloured;
            } else if (y == static_cast<size_t>(kHeight) / 2u) {
                const glm::vec3 got(gl[p * 4u], gl[p * 4u + 1u], gl[p * 4u + 2u]);
                const glm::vec3 diff = glm::abs(got - plus_one);
                ++w.row_wall;
                w.row_off += std::max({diff.x, diff.y, diff.z}) > 2.0f ? 1u : 0u;
            }
        }
    }
    w.band = channel_diff(cpu, gl);
    return w;
}

// FlowSurfaceWallsAndOffsetMatchTheCpu's band. Measured 2026-10-13, 160x120,
// on an NVIDIA GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83), the same in
// four runs: of 3080 coloured pixels, 566 differ, each by one level, as in
// step 5's band (GL's interpolated point against the CPU's from depth). All
// 55 wall pixels on the middle row read +1.
constexpr int kWallsWorst = 1;
constexpr size_t kWallsOverTwo = 0;
constexpr size_t kWallsDiffering = 566;

// Two quads at z = 0, the left a flow wall, across the sheet, seen from
// z = 3: the wall reads the seen side's +1 and the other quad keeps its
// shading. GL's face normal comes from derivatives, raster_cpu's from the
// triangle; on these flat quads they agree.
TEST_F(GpuGlRenderer, FlowSurfaceWallsAndOffsetMatchTheCpu) {
    const spade::FlowVolume volume = sheet_pressure();
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-0.8f, -0.8f, 0.0f), glm::vec3(0.8f, -0.8f, 0.0f), glm::vec3(0.8f, 0.8f, 0.0f),
                      glm::vec3(-0.8f, 0.8f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    RenderScene scene;
    scene.meshes.push_back(quad);
    scene.materials = {spade::render::Material{}};
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    for (const float x : {-1.0f, 1.0f}) {
        spade::render::DrawItem item{.mesh_index = 0,
                                     .local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(x, 0.0f, 0.0f)),
                                     .material_override = spade::render::kNoMaterial};
        item.flow_wall = x < 0.0f;
        scene.statics.push_back(item);
    }
    scene.flow_surface_colouring = walls_colouring(volume);
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 3.0f);
    RenderScene bare = scene;
    bare.flow_surface_colouring.reset();
    const FlowWalls w =
        flow_walls(draw_and_read(scene, comparable_options(), camera), draw_and_read(bare, comparable_options(), camera),
                   cpu_rgba(scene, camera, comparable_options()), cpu_rgba(bare, camera, comparable_options()));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ flow walls ] %zu px coloured; row %zu, %zu off; worst %d levels, %zu over two; %zu differ; on %s\n",
                w.coloured, w.row_wall, w.row_off, w.band.worst, w.band.over_two, w.band.differing, device.c_str());
    ASSERT_GT(w.coloured, kPixels / 20u) << "the wall must take the colour";
    EXPECT_EQ(w.right_coloured, 0u) << "only the flow wall takes the colour";
    EXPECT_GE(w.row_wall, 10u);
    EXPECT_EQ(w.row_off, 0u) << "the wall reads the seen side's +1";
    EXPECT_LE(w.band.worst, kWallsWorst) << device;
    EXPECT_LE(w.band.over_two, kWallsOverTwo);
    EXPECT_LE(w.band.differing, kWallsDiffering);
}

// FlowSurfaceOnCsgWallsMatchesTheCpu's band, measured as the one above: of
// 3053 coloured pixels none differ, and all 56 wall pixels on the middle row
// read +1.
constexpr size_t kCsgWallsRowOff = 0;
constexpr int kCsgWallsWorst = 0;
constexpr size_t kCsgWallsOverTwo = 0;
constexpr size_t kCsgWallsDiffering = 0;

// The same on GL's CSG pass: two 2 cm slabs with holes, marched, the left a
// flow wall. The offset follows the distance gradient, on both paths.
TEST_F(GpuGlRenderer, FlowSurfaceOnCsgWallsMatchesTheCpu) {
    spade::WorldBuilder b = csg_builder();
    for (const float x : {-1.0f, 1.0f}) {
        b.box(glm::vec3(0.8f, 0.8f, 0.01f), spade::SdfPose{.position = glm::vec3(x, 0.0f, 0.0f)});
        b.sphere(0.15f, spade::SdfPose{.position = glm::vec3(x + 0.4f, 0.4f, 0.0f)});
        b.subtract();
    }
    b.union_();  // the program reduces to one value; each slab stays a subtree
    RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.csg_subtrees.size(), 2u);
    for (const spade::render::CsgSubtree& sub : scene.csg_subtrees) {
        scene.statics[sub.draw_item].flow_wall = sub.bounds.max.x < 0.0f;
    }
    const spade::FlowVolume volume = sheet_pressure();
    scene.flow_surface_colouring = walls_colouring(volume);
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 3.0f);
    RenderScene bare = scene;
    bare.flow_surface_colouring.reset();
    const FlowWalls w =
        flow_walls(draw_and_read(scene, comparable_options(), camera), draw_and_read(bare, comparable_options(), camera),
                   cpu_rgba(scene, camera, comparable_options()), cpu_rgba(bare, camera, comparable_options()));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ csg walls ] %zu px coloured; row %zu, %zu off; worst %d levels, %zu over two; %zu differ; on %s\n",
                w.coloured, w.row_wall, w.row_off, w.band.worst, w.band.over_two, w.band.differing, device.c_str());
    ASSERT_GT(w.coloured, kPixels / 20u) << "the wall must take the colour";
    EXPECT_EQ(w.right_coloured, 0u) << "only the flow wall takes the colour";
    EXPECT_GE(w.row_wall, 10u);
    EXPECT_LE(w.row_off, kCsgWallsRowOff) << "the wall reads the seen side's +1";
    EXPECT_LE(w.band.worst, kCsgWallsWorst) << device;
    EXPECT_LE(w.band.over_two, kCsgWallsOverTwo);
    EXPECT_LE(w.band.differing, kCsgWallsDiffering);
}

// CpOffASphereMatchesTheCpu's bands. Measured 2026-10-13, 160x120, on an
// NVIDIA GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83), the same in
// three runs: GL's worst |dC_p| 0.202 over 1672 pixels, none uncoloured, as
// the CPU's (0.202); 952 pixels differ from the CPU's frame, each by one
// level.
constexpr double kSphereGlWorstCp = 0.21;
constexpr int kSphereWorst = 1;
constexpr size_t kSphereOverTwo = 0;
constexpr size_t kSphereDiffering = 952;

// RND-8's C_p view on GL: the potential-flow sphere at 8 cells per radius,
// read one cell off the wall and extrapolated to it. GL's frame is judged on
// its own against 1 - 9/4 sin^2(theta), by the CPU test's rule
// (tests/cp_sphere.hpp), and banded against the CPU's frame.
TEST_F(GpuGlRenderer, CpOffASphereMatchesTheCpu) {
    const float cp_min = -1.25f, cp_max = 1.0f;
    const auto colours = spade::render::cp_colours(0.5f, 0.0f, cp_min, cp_max);
    ASSERT_TRUE(colours.has_value());
    const spade::FlowVolume volume = spade::test::potential_sphere_volume(8);
    RenderScene scene;
    scene.meshes.push_back(spade::test::potential_sphere_mesh());
    scene.materials = {Material{}};
    DrawItem sphere{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)};
    sphere.flow_wall = true;
    scene.statics.push_back(sphere);
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    spade::render::FlowSurfaceColouring colouring{&volume, *colours};
    colouring.walls_only = true;
    colouring.offset_cells = 1.0f;
    colouring.extrapolate = true;
    scene.flow_surface_colouring = colouring;
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 2.2f);
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, comparable_options());
    ASSERT_EQ(gl.size(), kPixels * 4u);
    const auto judge = [&](const std::vector<uint8_t>& rgba) {
        return spade::test::potential_sphere_cp_error(
            camera, static_cast<uint32_t>(kWidth), static_cast<uint32_t>(kHeight), cp_min, cp_max,
            [&](uint32_t x, uint32_t y) {
                const size_t i = ((static_cast<size_t>(kHeight) - 1u - y) * kWidth + x) * 4u;  // bottom row first
                return glm::vec3(rgba[i], rgba[i + 1u], rgba[i + 2u]);
            });
    };
    const spade::test::CpError on_gl = judge(gl), on_cpu = judge(cpu);
    const ChannelDiff band = channel_diff(cpu, gl);
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ sphere C_p ] GL worst %.3f, mean %.4f over %zu px, %zu unmapped; CPU worst %.3f; GL-CPU worst %d "
                "levels, %zu over two, %zu differ; on %s\n",
                on_gl.worst, on_gl.mean, on_gl.count, on_gl.unmapped, on_cpu.worst, band.worst, band.over_two,
                band.differing, device.c_str());
    EXPECT_GE(on_gl.count, 1000u);
    EXPECT_EQ(on_gl.unmapped, 0u) << "every pixel on the sphere takes a C_p colour";
    EXPECT_LE(on_gl.worst, kSphereGlWorstCp);
    EXPECT_LE(band.worst, kSphereWorst) << device;
    EXPECT_LE(band.over_two, kSphereOverTwo);
    EXPECT_LE(band.differing, kSphereDiffering);
}

// Plan step 7g: a renderer asked to compile its flow programs in the
// background says when they are ready. Until then it refuses a flow layer
// as compiling (L6); once ready, within two minutes even on a cold driver
// cache, it draws the frame a renderer that waited at create() draws.
TEST_F(GpuGlRenderer, FlowProgramsCompileInTheBackgroundAndSaySo) {
    auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress),
                                   GlRenderer::CreateOptions{.flow_programs_in_background = true});
    ASSERT_TRUE(made.has_value()) << made.error().context;
    GlRenderer& background = **made;
    const spade::FlowVolume ring = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 32.0f, 32, 32, 32,
                                                                 spade::test::vortex_ring_with_dye());
    spade::render::TransferFunction vorticity;
    vorticity.quantity = spade::render::FlowQuantity::vorticity;
    vorticity.extinction = {{2.0f, 0.0f}, {22.0f, 12.0f}};
    RenderScene scene = ring_cut_scene();
    scene.flow_layers.push_back(spade::render::FlowLayer{&ring, {vorticity}, {}});
    ASSERT_TRUE(background.upload_scene(scene).has_value());
    const auto draw_with = [&](GlRenderer& r) {
        bind_cleared_target();
        const spade::Result<void> drew = r.draw(scene, camera_on_ring(), comparable_options(), kWidth, kHeight);
        std::vector<uint8_t> rgba;
        if (drew) {
            rgba.resize(kPixels * 4u);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        }
        return std::pair{drew, rgba};
    };
    // Before it says ready, a draw is refused as compiling, or (the driver
    // having finished meanwhile) drawn.
    const auto start = std::chrono::steady_clock::now();
    const bool ready_at_once = background.flow_ready();
    bool refused = false;
    if (!ready_at_once) {
        const auto [drew, rgba] = draw_with(background);
        if (!drew) {
            refused = true;
            EXPECT_EQ(drew.error().code, spade::Code::unavailable);
            EXPECT_NE(drew.error().context.find("compiling"), std::string::npos) << drew.error().context;
        }
    }
    const auto deadline = start + std::chrono::seconds(120);
    while (!background.flow_ready() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(background.flow_ready()) << "the flow programs must finish compiling";
    const double waited_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    std::printf("[ flow ready ] ready at create: %s; a draw before ready refused: %s; ready after %.0f ms\n",
                ready_at_once ? "yes" : "no", refused ? "yes" : "no", waited_ms);
    const auto [drew, got] = draw_with(background);
    ASSERT_TRUE(drew.has_value()) << drew.error().context;
    ASSERT_TRUE(renderer_->flow_ready()) << "a renderer that waited at create() is ready at once";
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
    const auto [drew_waited, want] = draw_with(*renderer_);
    ASSERT_TRUE(drew_waited.has_value());
    EXPECT_EQ(count_differing(got, want), 0u) << "the same programs draw the same frame";
}

// 7g's measurement, from cold. In the fixture the waiting renderer has
// already compiled the same programs, so the driver hands them back at
// once. Here the background renderer is the process's first. Opt-in
// (SPADE_GL_FLOW_COLD=1), and cold only when the driver's shader cache is
// empty: __GL_SHADER_DISK_CACHE=0 on Linux; on Windows, NVIDIA's GLCache
// folder cleared by whoever runs it. Measured 2026-10-12 on the RTX 3060 Ti
// with the cache warm: create() 42 ms, ready at 55 ms. It reports the times
// and holds the contract: a flow layer drawn before ready is refused as
// compiling, and the programs finish.
TEST(GpuGlFlowCold, CreateReturnsBeforeTheFlowProgramsCompile) {
    if (!env_gate_is_set("SPADE_GL_FLOW_COLD")) {
        GTEST_SKIP() << "a measurement: set SPADE_GL_FLOW_COLD=1 with the driver's shader cache empty, and run it "
                        "alone";
    }
    glfwSetErrorCallback(record_glfw_error);
    if (glfwInit() != GLFW_TRUE) GTEST_SKIP() << "GLFW could not initialise. " << g_last_glfw_error;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(kWidth, kHeight, "spade GpuGlFlowCold", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        GTEST_SKIP() << "no OpenGL 4.3 core context on this machine. " << g_last_glfw_error;
    }
    glfwMakeContextCurrent(window);
    {
        const auto t0 = std::chrono::steady_clock::now();
        auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress),
                                       GlRenderer::CreateOptions{.flow_programs_in_background = true});
        const double create_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        ASSERT_TRUE(made.has_value()) << made.error().context;
        GlRenderer& r = **made;
        const bool ready_at_once = r.flow_ready();
        const spade::FlowVolume ring = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 32.0f, 32, 32, 32,
                                                                     spade::test::vortex_ring_with_dye());
        spade::render::TransferFunction vorticity;
        vorticity.quantity = spade::render::FlowQuantity::vorticity;
        vorticity.extinction = {{2.0f, 0.0f}, {22.0f, 12.0f}};
        RenderScene scene = ring_cut_scene();
        scene.flow_layers.push_back(spade::render::FlowLayer{&ring, {vorticity}, {}});
        ASSERT_TRUE(r.upload_scene(scene).has_value());
        const spade::Result<void> early = r.draw(scene, camera_on_ring(), comparable_options(), kWidth, kHeight);
        const auto deadline = t0 + std::chrono::seconds(120);
        while (!r.flow_ready() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const double ready_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        RecordProperty("create_ms", static_cast<int>(create_ms));
        RecordProperty("ready_ms", static_cast<int>(ready_ms));
        std::printf("[ flow cold ] create() %.0f ms; flow programs ready after %.0f ms; ready at create: %s; "
                    "early draw: %s\n",
                    create_ms, ready_ms, ready_at_once ? "yes" : "no",
                    early ? "drawn" : early.error().context.c_str());
        if (!ready_at_once && !early) {
            EXPECT_EQ(early.error().code, spade::Code::unavailable);
            EXPECT_NE(early.error().context.find("compiling"), std::string::npos);
        }
        EXPECT_FALSE(ready_at_once && !early) << "a renderer that says ready must draw";
        EXPECT_TRUE(r.flow_ready()) << "the flow programs must finish compiling";
    }
    glfwDestroyWindow(window);
    glfwTerminate();
}

// Plan step 7f: the live-only reduced march. Half marches a quarter of the
// rays and says so through last_flow_scale(); a frame with no flow layer
// says 0. The half frame is held to the full one by a measured band: a live
// view, not the CPU's frame.
TEST_F(GpuGlRenderer, FlowHalfScaleMarchesAtHalfAndSaysSo) {
    const spade::FlowVolume ring = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 32.0f, 32, 32, 32,
                                                                 spade::test::vortex_ring_with_dye());
    spade::render::TransferFunction vorticity;
    vorticity.quantity = spade::render::FlowQuantity::vorticity;
    vorticity.colours = spade::render::ColourMapId::inferno;
    vorticity.range_min = 0.0f;
    vorticity.range_max = 22.0f;
    vorticity.extinction = {{2.0f, 0.0f}, {22.0f, 12.0f}};
    RenderScene scene = ring_cut_scene();
    scene.flow_layers.push_back(spade::render::FlowLayer{&ring, {vorticity}, {}});

    const std::vector<uint8_t> full = draw_and_read(scene, comparable_options(), camera_on_ring());
    EXPECT_EQ(renderer_->last_flow_scale(), 1.0f);
    renderer_->set_flow_quality(GlRenderer::FlowQuality{.scale = GlRenderer::FlowScale::half});
    const std::vector<uint8_t> half = draw_and_read(scene, comparable_options(), camera_on_ring());
    EXPECT_EQ(renderer_->last_flow_scale(), 0.5f) << "a half-resolution march must say so";
    RenderScene bare = ring_cut_scene();
    (void)draw_and_read(bare, comparable_options(), camera_on_ring());
    EXPECT_EQ(renderer_->last_flow_scale(), 0.0f) << "a frame without a flow layer says 0";
    renderer_->set_flow_quality({});

    ASSERT_EQ(half.size(), kPixels * 4u);
    const ChannelDiff d = channel_diff(full, half);
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ flow half ] against full: worst %d levels, %zu over two, %zu differ; on %s\n", d.worst,
                d.over_two, d.differing, device.c_str());
    // Measured 2026-10-12, 160x120 (so an 80x60 march), on an NVIDIA GeForce
    // RTX 3060 Ti (OpenGL 4.3.0, driver 572.83): 3186 pixels differ from the
    // full frame, 2349 by more than two levels, the worst by 32. Looked at:
    // the ring is the same ring, softer; the large differences sit on its
    // edges and along the quad's edge, where one half pixel's march spans the
    // opaque edge. A live view's band, announced as such, not the CPU's.
    constexpr int kBandWorstLevels = 32;
    constexpr size_t kBandOverTwoPixels = 2349;
    constexpr size_t kBandDifferingPixels = 3186;
    EXPECT_GT(d.differing, 0u) << "a half-resolution march is not the full frame";
    EXPECT_LE(d.worst, kBandWorstLevels) << device;
    EXPECT_LE(d.over_two, kBandOverTwoPixels) << device;
    EXPECT_LE(d.differing, kBandDifferingPixels) << device;
}

// Automatic picks half when the march misses its budget, and full when it
// holds it, judged from the frames before.
TEST_F(GpuGlRenderer, FlowAutomaticScaleFollowsTheBudget) {
    const spade::FlowVolume ring = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 32.0f, 32, 32, 32,
                                                                 spade::test::vortex_ring_with_dye());
    spade::render::TransferFunction vorticity;
    vorticity.quantity = spade::render::FlowQuantity::vorticity;
    vorticity.extinction = {{2.0f, 0.0f}, {22.0f, 12.0f}};
    RenderScene scene = ring_cut_scene();
    scene.flow_layers.push_back(spade::render::FlowLayer{&ring, {vorticity}, {}});
    const auto settle = [&](float budget_ms) {
        renderer_->set_flow_quality(
            GlRenderer::FlowQuality{.scale = GlRenderer::FlowScale::automatic, .march_budget_ms = budget_ms});
        for (int i = 0; i < 4; ++i) (void)draw_and_read(scene, comparable_options(), camera_on_ring());
        return renderer_->last_flow_scale();
    };
    EXPECT_EQ(settle(1e-6f), 0.5f) << "a march over its budget drops to half";
    EXPECT_EQ(settle(1e6f), 1.0f) << "a march well within its budget returns to full";
    renderer_->set_flow_quality({});
}

// RND-7's ask: 7e's skipping measured on a real rotor wake, not only the
// sparse synthetic ring. Plan A's actuator disc (Task 9) at 32^3 for 80
// steps, as RenderFlowFeed's test drives it, published, and marched at
// 1920x1080 by speed and by Q, skipped and not. Opt-in with the cost test
// (SPADE_GL_FLOW_COST=1). Skipping must change no pixel here too.
TEST_F(GpuGlRenderer, FlowSkippingOnTheActuatorDiscsWake) {
    if (!env_gate_is_set("SPADE_GL_FLOW_COST")) {
        GTEST_SKIP() << "a measurement: set SPADE_GL_FLOW_COST=1 to run it";
    }
    namespace air = spade::physics::airflow;
    const air::DomainBc bc;
    air::MacGrid g = *air::make_grid({32, 32, 32, 0.05f}, bc);
    const air::ActuatorDisc disc{.centre = glm::vec3(0.8f, 1.2f, 0.8f),
                                 .axis = glm::vec3(0.0f, -1.0f, 0.0f),
                                 .radius = 0.2f,
                                 .thrust = 1.2f};
    ASSERT_TRUE(air::check_disc(disc, g.shape).has_value());
    const air::FluidConfig cfg{.nu = 1.5e-5f, .h = 0.005f};
    air::FluidScratch scratch = *air::make_fluid_scratch(g, bc, cfg);
    for (int step = 0; step < 80; ++step) {
        air::spread_actuator(disc, g.shape, cfg.rho, cfg.h, scratch.du, scratch.dv, scratch.dw);
        ASSERT_EQ(air::fluid_step(g, bc, cfg, scratch).fault, air::FluidFault::none);
    }
    spade::FlowVolume wake;
    ASSERT_TRUE(air::publish_render_volume(g, air::PublishInput{}, wake).has_value());
    const float v_h = std::sqrt(disc.thrust / (2.0f * cfg.rho * 3.14159265f * disc.radius * disc.radius));
    spade::render::TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 2.0f * v_h;
    speed.extinction = {{0.25f * v_h, 0.0f}, {2.0f * v_h, 6.0f}};
    spade::render::IsoSurface q_iso;
    q_iso.quantity = spade::render::FlowQuantity::q_hat;
    q_iso.value = 0.3f * wake.header.channel_max[spade::kFlowQHat];
    q_iso.colour_by = speed;
    RenderScene volume_scene;
    volume_scene.flow_layers.push_back(spade::render::FlowLayer{&wake, {speed}, {}});
    RenderScene iso_scene;
    iso_scene.flow_layers.push_back(spade::render::FlowLayer{&wake, {}, {q_iso}});
    Camera camera;
    camera.position = glm::vec3(0.8f, 0.8f, 3.4f);

    // Identical frames at the fixture's size first.
    for (const RenderScene* scene : {&volume_scene, &iso_scene}) {
        renderer_->set_flow_quality(GlRenderer::FlowQuality{.skip_empty_space = false});
        const std::vector<uint8_t> full = draw_and_read(*scene, comparable_options(), camera);
        renderer_->set_flow_quality(GlRenderer::FlowQuality{.skip_empty_space = true});
        const std::vector<uint8_t> skipped = draw_and_read(*scene, comparable_options(), camera);
        ASSERT_EQ(full.size(), kPixels * 4u);
        EXPECT_EQ(count_differing(full, skipped), 0u) << "skipping must change no pixel of the wake";
    }

    constexpr int kW = 1920, kH = 1080;
    GLuint fbo = 0, colour = 0, depth = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenRenderbuffers(1, &colour);
    glBindRenderbuffer(GL_RENDERBUFFER, colour);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, kW, kH);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, kW, kH);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
    ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));
    glViewport(0, 0, kW, kH);
    const auto median_ms = [&](const RenderScene& scene, bool skip) {
        renderer_->set_flow_quality(GlRenderer::FlowQuality{.skip_empty_space = skip});
        std::vector<double> ms;
        for (int i = 0; i < 10; ++i) {
            glFinish();
            const auto t0 = std::chrono::steady_clock::now();
            EXPECT_TRUE(renderer_->draw(scene, camera, comparable_options(), static_cast<uint32_t>(kW),
                                        static_cast<uint32_t>(kH))
                            .has_value());
            glFinish();
            const auto t1 = std::chrono::steady_clock::now();
            if (i >= 2) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        std::sort(ms.begin(), ms.end());
        return 0.5 * (ms[3] + ms[4]);
    };
    const double volume_full = median_ms(volume_scene, false);
    const double volume_skip = median_ms(volume_scene, true);
    const double iso_full = median_ms(iso_scene, false);
    const double iso_skip = median_ms(iso_scene, true);
    renderer_->set_flow_quality({});
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    RecordProperty("volume_ms_x100", static_cast<int>(volume_skip * 100.0));
    RecordProperty("volume_unskipped_ms_x100", static_cast<int>(volume_full * 100.0));
    RecordProperty("iso_ms_x100", static_cast<int>(iso_skip * 100.0));
    RecordProperty("iso_unskipped_ms_x100", static_cast<int>(iso_full * 100.0));
    std::printf("[ wake cost ] disc wake 32^3 at 1920x1080: volume %.2f ms (%.2f unskipped), iso %.2f ms (%.2f "
                "unskipped); on %s\n",
                volume_skip, volume_full, iso_skip, iso_full, device.c_str());
    EXPECT_LT(volume_skip, 1000.0) << device;
    EXPECT_LT(iso_skip, 1000.0) << device;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);
}

// Plan step 7e: skipping empty space changes no pixel. The flow goldens'
// volume and iso layers, through the cut quad, skipped and not: identical
// frames, byte for byte. And a one-cell sheet of vorticity at x index 8, the
// first cell of the second brick: the first brick's cells are all 0, yet the
// rays down the last half cell of that brick read the sheet. Only the brick
// pass's one-cell apron keeps that brick from being skipped there.
TEST_F(GpuGlRenderer, FlowSkippingChangesNoPixel) {
    const spade::FlowVolume sheet =
        spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 32.0f, 32, 32, 32, [](glm::vec3 p) {
            spade::test::FlowChannels c{};
            const long i = std::lround((p.x + 1.0f) * 16.0f - 0.5f);  // the cell's x index
            c[spade::kFlowVorticity] = i == 8 ? 20.0f : 0.0f;
            return c;
        });
    const spade::FlowVolume ring32 = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 32.0f, 32, 32, 32,
                                                                   spade::test::vortex_ring_with_dye());
    const spade::FlowVolume ring64 = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 64.0f, 64, 64, 64,
                                                                   spade::test::vortex_ring_with_dye());
    spade::render::TransferFunction vorticity;
    vorticity.quantity = spade::render::FlowQuantity::vorticity;
    vorticity.colours = spade::render::ColourMapId::inferno;
    vorticity.range_min = 0.0f;
    vorticity.range_max = 22.0f;
    vorticity.extinction = {{2.0f, 0.0f}, {22.0f, 12.0f}};
    spade::render::TransferFunction smoke;
    smoke.quantity = spade::render::FlowQuantity::dye;
    smoke.constant_colour = glm::vec3(0.95f, 0.95f, 0.95f);
    smoke.extinction = {{0.0f, 0.0f}, {1.0f, 15.0f}};
    spade::render::TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 0.9f;
    spade::render::IsoSurface q_iso;
    q_iso.quantity = spade::render::FlowQuantity::q_hat;
    q_iso.value = 0.5477226f * ring64.header.channel_max[spade::kFlowQHat];
    q_iso.colour_by = speed;
    RenderScene volume_scene = ring_cut_scene();
    volume_scene.flow_layers.push_back(spade::render::FlowLayer{&ring32, {vorticity, smoke}, {}});
    RenderScene iso_scene = ring_cut_scene();
    iso_scene.flow_layers.push_back(spade::render::FlowLayer{&ring64, {}, {q_iso}});
    RenderScene sheet_scene;
    sheet_scene.flow_layers.push_back(spade::render::FlowLayer{&sheet, {vorticity}, {}});

    for (const RenderScene* scene : {&volume_scene, &iso_scene, &sheet_scene}) {
        renderer_->set_flow_quality(GlRenderer::FlowQuality{.skip_empty_space = false});
        const std::vector<uint8_t> full = draw_and_read(*scene, comparable_options(), camera_on_ring());
        renderer_->set_flow_quality(GlRenderer::FlowQuality{.skip_empty_space = true});
        const std::vector<uint8_t> skipped = draw_and_read(*scene, comparable_options(), camera_on_ring());
        ASSERT_EQ(full.size(), kPixels * 4u);
        EXPECT_EQ(count_differing(full, skipped), 0u) << "skipping empty space must change no pixel";
    }
    renderer_->set_flow_quality({});
}

// Airflow plan step 7a: GL draws streamlines and tracers as the CPU does.
// Lines as GL_LINES, each segment in the mean of its ends' colours; points as
// squares of size_px. Depth-tested against the ground. Lines rasterize by
// different rules on the two paths, so a pixel may sit one pixel apart; the
// colours are flat and must match exactly. A miss is a flow pixel on one path
// with no flow pixel of the same colour within 1 px on the other: the
// overlay band's measure. Both counts are measured and pinned (03-verification).
TEST_F(GpuGlRenderer, FlowLinesAndPointsMatchTheCpuWithinTheirBand) {
    RenderScene scene = make_ground_scene();
    // Five helical streamlines over the ground, coloured along their length,
    // and tracers along a rake; some dip below the ground and are hidden.
    for (int k = 0; k < 5; ++k) {
        spade::render::FlowPolyline line;
        for (int i = 0; i <= 40; ++i) {
            const float s = static_cast<float>(i) / 40.0f;
            const float a = 6.0f * s + 1.3f * static_cast<float>(k);
            line.points.push_back(glm::vec3(-2.0f + 4.0f * s, 0.6f + 0.4f * static_cast<float>(k) * 0.3f + 0.5f * std::sin(a),
                                            -1.0f + 0.5f * std::cos(a) + 0.3f * static_cast<float>(k)));
            line.colours.push_back(spade::render::colour_map_at(spade::render::ColourMapId::viridis, s));
        }
        scene.flow_polylines.push_back(std::move(line));
    }
    for (int i = 0; i < 9; ++i) {
        scene.flow_points.positions.push_back(glm::vec3(-1.6f + 0.4f * static_cast<float>(i), 1.4f, 0.4f));
        scene.flow_points.colours.push_back(glm::vec3(0.95f));
    }
    scene.flow_points.size_px = 3;
    RenderScene bare = scene;
    bare.flow_polylines.clear();
    bare.flow_points = {};
    const Camera camera = camera_over_ground();

    const std::vector<uint8_t> cpu_on = cpu_rgba(scene, camera, comparable_options());
    const std::vector<uint8_t> cpu_off = cpu_rgba(bare, camera, comparable_options());
    const std::vector<uint8_t> gl_on = draw_and_read(scene, comparable_options(), camera);
    const std::vector<uint8_t> gl_off = draw_and_read(bare, comparable_options(), camera);
    ASSERT_EQ(gl_on.size(), kPixels * 4u) << "GL must draw streamlines and tracers now";
    ASSERT_EQ(gl_off.size(), kPixels * 4u);

    std::vector<uint8_t> cpu_flow(kPixels), gl_flow(kPixels);
    size_t cpu_count = 0, gl_count = 0;
    for (size_t p = 0; p < kPixels; ++p) {
        cpu_flow[p] = static_cast<uint8_t>(same_pixel(cpu_on, cpu_off, p) ? 0 : 1);
        gl_flow[p] = static_cast<uint8_t>(same_pixel(gl_on, gl_off, p) ? 0 : 1);
        cpu_count += cpu_flow[p];
        gl_count += gl_flow[p];
    }
    ASSERT_GT(cpu_count, 200u) << "the CPU frame must hold streamlines and tracers";

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
    const size_t cpu_misses = misses(cpu_on, cpu_flow, gl_on, gl_flow);
    const size_t gl_misses = misses(gl_on, gl_flow, cpu_on, cpu_flow);
    RecordProperty("cpu_flow_pixels", static_cast<int>(cpu_count));
    RecordProperty("gl_flow_pixels", static_cast<int>(gl_count));
    RecordProperty("cpu_misses", static_cast<int>(cpu_misses));
    RecordProperty("gl_misses", static_cast<int>(gl_misses));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ flow lines ] CPU %zu px, %zu miss; GL %zu px, %zu miss; on %s\n", cpu_count, cpu_misses, gl_count,
                gl_misses, device.c_str());
    // Measured 2026-10-12 on master ae57288 (gap-free CPU lines), 160x120, on
    // an NVIDIA GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83): 11 of the
    // CPU's 409 flow pixels miss, and 4 of GL's 374. Located: all 15 are
    // streamline pixels in rows 68 to 77, where the helices are densest, and
    // each has a flow pixel within 1 px on the other path, of another colour.
    // So coverage agrees; the two paths give a pixel at a segment joint or a
    // crossing to different segments. The tracers match everywhere. (Before
    // line-gaps: 10 of 279 and 59 of 374, GL's misses being the CPU's gaps.)
    // Colours go to GL as the CPU's own bytes; without that, about 7% of
    // pixels split one level apart in rounding.
    constexpr size_t kBandCpuMisses = 11;
    constexpr size_t kBandGlMisses = 4;
    EXPECT_LE(cpu_misses, kBandCpuMisses) << device;
    EXPECT_LE(gl_misses, kBandGlMisses) << device;
}

// L6: nor surface colouring by a flow quantity (plan step 7).
TEST_F(GpuGlRenderer, SurfaceColouringMatchesTheCpuWithinItsBand) {
    // Airflow plan step 7b: the C_p view on GL. A quad facing the camera at
    // z = 0 inside a volume holding p = 2x - y + 0.5z (dyadic: f16 holds it
    // exactly), coloured by pressure through cool-warm over [-2, 2]. GL colours
    // each fragment at its own world point; the CPU rebuilds the point from
    // depth. Compared per channel over the frame. A corner of the volume is
    // no-data (NaN), where both keep the shaded colour: GL tests the half's
    // bit pattern, not the driver's NaN.
    const spade::FlowVolume volume =
        spade::test::make_flow_volume(glm::vec3(-1.0f, -2.0f, -1.0f), 0.25f, 8, 16, 8, [](glm::vec3 p) {
            spade::test::FlowChannels c{};
            c[spade::kFlowPressure] = p.x > 0.5f && p.y > 0.5f ? std::numeric_limits<float>::quiet_NaN()
                                                                : 2.0f * p.x - p.y + 0.5f * p.z;
            return c;
        });
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::pressure;
    colours.colours = spade::render::ColourMapId::cool_warm;
    colours.range_min = -2.0f;
    colours.range_max = 2.0f;
    RenderScene scene;
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-1.5f, -1.5f, 0.0f), glm::vec3(1.5f, -1.5f, 0.0f), glm::vec3(1.5f, 1.5f, 0.0f),
                      glm::vec3(-1.5f, 1.5f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    scene.meshes.push_back(quad);
    scene.materials = {Material{}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)});
    RenderScene coloured = scene;
    coloured.flow_surface_colouring = spade::render::FlowSurfaceColouring{&volume, colours};

    const std::vector<uint8_t> cpu = cpu_rgba(coloured, camera_on_plus_z(), comparable_options());
    const std::vector<uint8_t> cpu_plain = cpu_rgba(scene, camera_on_plus_z(), comparable_options());
    const std::vector<uint8_t> gl = draw_and_read(coloured);
    ASSERT_EQ(gl.size(), kPixels * 4u) << "GL must draw surface colouring now";
    const size_t coloured_pixels = count_differing(cpu, cpu_plain);
    ASSERT_GT(coloured_pixels, kPixels / 10u) << "the colouring must change a good part of the frame";
    int worst = 0;
    size_t differing = 0;
    for (size_t p = 0; p < kPixels; ++p) {
        int d = 0;
        for (size_t c = 0; c < 3; ++c) d = std::max(d, std::abs(int{cpu[p * 4u + c]} - int{gl[p * 4u + c]}));
        worst = std::max(worst, d);
        differing += d > 0 ? 1u : 0u;
    }
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    RecordProperty("coloured_pixels", static_cast<int>(coloured_pixels));
    RecordProperty("worst_levels", worst);
    RecordProperty("differing_pixels", static_cast<int>(differing));
    std::printf("[ surface colouring ] %zu px coloured; worst %d levels, %zu pixels differ; on %s\n", coloured_pixels,
                worst, differing, device.c_str());
    // Measured 2026-10-12, 160x120, on an NVIDIA GeForce RTX 3060 Ti (OpenGL
    // 4.3.0, driver 572.83): of 2305 coloured pixels, 861 differ, each by one
    // level. GL reaches the point by the fragment's interpolated world
    // position; the CPU rebuilds it from depth. Before the no-data corner, a
    // probe that rounded GL's colour as to_byte() does left 681 of 874, so
    // the point, not the byte conversion, is most of it. One level is
    // invisible in a C_p view.
    constexpr int kBandWorstLevels = 1;
    constexpr size_t kBandDifferingPixels = 861;
    EXPECT_LE(worst, kBandWorstLevels) << device;
    EXPECT_LE(differing, kBandDifferingPixels) << device;
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

// render/view_mode.hpp, the lead's condition: for every case, supports() says
// drawable exactly when apply_view() and GlRenderer::draw() draw it, and names
// the reason they refuse with (the CPU's twin is in test_render_view_mode.cpp).
// The renderer here waited at create(), so its flow programs are ready.
TEST_F(GpuGlRenderer, EveryViewDrawsOnGlExactlyWhenSupportsSaysSo) {
    ASSERT_TRUE(renderer_->flow_ready());
    spade::test::expect_view_parity(spade::render::ViewBackend::gl,
                                    [&](const RenderScene& scene, const RenderOptions& options) -> spade::Result<void> {
                                        if (auto up = renderer_->upload_scene(scene); !up) return up;
                                        bind_cleared_target();
                                        return renderer_->draw(scene, camera_on_plus_z(), options, kWidth, kHeight);
                                    });
}

// And while the flow programs compile, supports() names GL's refusal of a
// flow layer. A driver that finishes at once leaves nothing to check.
TEST_F(GpuGlRenderer, AViewWaitsForTheFlowProgramsAsGlDoes) {
    auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress),
                                   GlRenderer::CreateOptions{.flow_programs_in_background = true});
    ASSERT_TRUE(made.has_value()) << made.error().context;
    GlRenderer& background = **made;
    const spade::FlowVolume volume = spade::test::view_mode_volume();
    const spade::render::ViewSpec spec{spade::render::ViewBase::shaded,
                                       {spade::test::layer(spade::render::FlowView::vortex_iso)}};
    spade::render::ViewInputs inputs{.has_volume = true, .has_walls = true, .has_sdf = false, .gl_flow_ready = false};
    RenderScene scene = spade::test::view_mode_scene(true, nullptr);
    RenderOptions options;
    ASSERT_TRUE(spade::render::apply_view(spec, &volume, scene, options).has_value());
    ASSERT_TRUE(background.upload_scene(scene).has_value());
    if (background.flow_ready()) GTEST_SKIP() << "the driver compiled the flow programs at once";
    bind_cleared_target();
    const spade::Result<void> drew = background.draw(scene, camera_on_plus_z(), options, kWidth, kHeight);
    if (drew) GTEST_SKIP() << "the driver finished compiling before the draw";
    const spade::render::ViewSupport said = spade::render::supports(spec, spade::render::ViewBackend::gl, inputs);
    EXPECT_FALSE(said.drawable);
    EXPECT_NE(drew.error().context.find(said.why_not), std::string::npos)
        << "supports() named \"" << said.why_not << "\"; GL refused: " << drew.error().context;
    while (!background.flow_ready()) std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

// The editor's views of one frame (the lead, 2026-10-19): a named volume is
// uploaded once and reused by every draw of its generation, and the reuse
// draws what an upload draws. Generation 0 is uploaded by each reader, here
// the C_p colouring and the march; a new generation is uploaded again.
TEST_F(GpuGlRenderer, ViewsOfOneVolumeUploadItOnce) {
    spade::FlowVolume volume = spade::test::view_mode_volume();
    const spade::render::ViewSpec spec{spade::render::ViewBase::shaded,
                                       {spade::test::layer(spade::render::FlowView::vortex_iso),
                                        spade::test::layer(spade::render::FlowView::surface_cp)}};
    RenderScene scene = spade::test::view_mode_scene(true, nullptr);
    RenderOptions options = comparable_options();
    ASSERT_TRUE(spade::render::apply_view(spec, &volume, scene, options).has_value());
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());

    const uint64_t at_start = renderer_->flow_volume_uploads();
    const std::vector<uint8_t> unnamed = draw_and_read(scene, options);
    ASSERT_FALSE(unnamed.empty());
    EXPECT_EQ(renderer_->flow_volume_uploads() - at_start, 2u) << "generation 0: each reader uploads it";

    volume.generation = spade::next_flow_generation();
    const uint64_t named_at = renderer_->flow_volume_uploads();
    for (int view = 0; view < 4; ++view) {
        EXPECT_EQ(draw_and_read(scene, options), unnamed) << "view " << view << ": a reuse draws what an upload draws";
    }
    EXPECT_EQ(renderer_->flow_volume_uploads() - named_at, 1u) << "four views of one generation upload it once";

    volume.generation = spade::next_flow_generation();
    EXPECT_EQ(draw_and_read(scene, options), unnamed);
    EXPECT_EQ(renderer_->flow_volume_uploads() - named_at, 2u) << "a new generation is uploaded again";
}

// The L6 guard (the lead's): a copy edited in place that keeps its generation
// would draw the old halves, silently. With verify_cached_volumes, on in debug
// builds and in this fixture, the reuse is refused, naming the generation;
// named anew, the copy draws.
TEST_F(GpuGlRenderer, AnEditThatKeepsItsGenerationIsRefused) {
    spade::FlowVolume original = spade::test::view_mode_volume();
    original.generation = spade::next_flow_generation();
    const spade::render::ViewSpec spec{spade::render::ViewBase::shaded,
                                       {spade::test::layer(spade::render::FlowView::vortex_iso)}};
    RenderScene scene = spade::test::view_mode_scene(true, nullptr);
    RenderOptions options = comparable_options();
    ASSERT_TRUE(spade::render::apply_view(spec, &original, scene, options).has_value());
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
    ASSERT_FALSE(draw_and_read(scene, options).empty());

    spade::FlowVolume copy = original;
    copy.s[1] ^= 0x0001u;  // edited in place; the generation kept
    ASSERT_TRUE(spade::render::apply_view(spec, &copy, scene, options).has_value());
    bind_cleared_target();
    const spade::Result<void> stale = renderer_->draw(scene, camera_on_plus_z(), options, kWidth, kHeight);
    ASSERT_FALSE(stale.has_value()) << "a stale reuse is refused, not drawn";
    EXPECT_NE(stale.error().context.find("generation " + std::to_string(original.generation)), std::string::npos)
        << stale.error().context;

    copy.generation = spade::next_flow_generation();
    bind_cleared_target();
    const spade::Result<void> named = renderer_->draw(scene, camera_on_plus_z(), options, kWidth, kHeight);
    EXPECT_TRUE(named.has_value()) << named.error().context;
}

// The lead's before-and-after (2026-10-19): a frame of 1 or 4 flow views of
// one volume, at the tour's 32^3 and at 128^3, uncached (generation 0: every
// view uploads, as every draw did before) and cached (each frame a new
// generation, as a live run publishes: its first view uploads, the rest
// reuse). Each view is a 960x540 draw (four tile a 1080p editor) of the ring's
// Q-hat iso coloured by speed; a frame is draw() to glFinish() over its views,
// the median of 8 after 2 warm-ups, on a renderer that does not verify (a
// release build's). A measurement, so opt-in (SPADE_GL_VIEW_COST=1).
// Measured 2026-10-19 on the RTX 3060 Ti (572.83), three runs, other realms
// working: at 128^3, 4 views 42 to 55 ms uncached, 26 to 36 ms cached (1 view
// 15 to 24 ms), so each upload skipped saves about 5 to 8 ms; at 32^3 the
// upload is 0.5 MB and the load's noise (1 view 3 to 11 ms, 4 views 9 to 17
// ms either way) hides it. Only the 128^3 saving is asserted.
TEST_F(GpuGlRenderer, FlowViewsCostCachedAndUncached) {
    if (!env_gate_is_set("SPADE_GL_VIEW_COST")) {
        GTEST_SKIP() << "a measurement: set SPADE_GL_VIEW_COST=1 to run it";
    }
    auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress),
                                   GlRenderer::CreateOptions{.verify_cached_volumes = false});
    ASSERT_TRUE(made.has_value()) << made.error().context;
    GlRenderer& r = **made;
    constexpr int kW = 960, kH = 540;
    GLuint fbo = 0, colour = 0, depth = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenRenderbuffers(1, &colour);
    glBindRenderbuffer(GL_RENDERBUFFER, colour);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, kW, kH);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, kW, kH);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
    ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));
    glViewport(0, 0, kW, kH);
    const std::string device = r.renderer_name() + " (" + r.version_string() + ")";

    for (const uint32_t n : {32u, 128u}) {
        spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / static_cast<float>(n), n, n,
                                                                 n, spade::test::vortex_ring_with_dye());
        spade::render::TransferFunction speed;
        speed.range_min = 0.0f;
        speed.range_max = 0.9f;
        spade::render::IsoSurface q_iso;
        q_iso.quantity = spade::render::FlowQuantity::q_hat;
        q_iso.value = 0.5477226f * volume.header.channel_max[spade::kFlowQHat];
        q_iso.colour_by = speed;
        RenderScene scene = ring_cut_scene();
        scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {}, {q_iso}});
        ASSERT_TRUE(r.upload_scene(scene).has_value());
        // ms per frame of `views` views; `cached` names each frame anew.
        const auto frame_ms = [&](int views, bool cached) {
            std::vector<double> ms;
            const uint64_t uploads_at = r.flow_volume_uploads();
            for (int i = 0; i < 10; ++i) {
                volume.generation = cached ? spade::next_flow_generation() : 0u;
                glFinish();
                const auto t0 = std::chrono::steady_clock::now();
                for (int v = 0; v < views; ++v) {
                    EXPECT_TRUE(r.draw(scene, camera_on_ring(), comparable_options(), kW, kH).has_value());
                }
                glFinish();
                const auto t1 = std::chrono::steady_clock::now();
                if (i >= 2) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            }
            EXPECT_EQ(r.flow_volume_uploads() - uploads_at, cached ? 10u : 10u * static_cast<uint64_t>(views))
                << "uploads per frame: one cached, one per view uncached";
            std::sort(ms.begin(), ms.end());
            return 0.5 * (ms[3] + ms[4]);
        };
        const double one = frame_ms(1, false);
        const double four_uncached = frame_ms(4, false);
        const double four_cached = frame_ms(4, true);
        const std::string tag = std::to_string(n);
        RecordProperty("one_view_ms_x100_" + tag, static_cast<int>(one * 100.0));
        RecordProperty("four_views_uncached_ms_x100_" + tag, static_cast<int>(four_uncached * 100.0));
        RecordProperty("four_views_cached_ms_x100_" + tag, static_cast<int>(four_cached * 100.0));
        std::printf("[ view cost ] %u^3, 960x540 views: 1 view %.2f ms; 4 views %.2f ms uncached, %.2f ms cached; "
                    "on %s\n",
                    n, one, four_uncached, four_cached, device.c_str());
        if (n == 128u) {
            EXPECT_LT(four_cached, four_uncached) << device;
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);
}

// The editor's cost table (the lead, 2026-10-19), for Interface to pick a
// layout against: a frame of every view in a layout, each view one mix built
// by apply_view() from a new generation of the volume, as a live run
// publishes one. Layouts: the whole 1920x1080 window (1 view; 2 at
// 960x1080; 4 at 960x540) and the editor's central region between its docked
// panels (Interface's sizes: 1 at 1280x960, 2 at 640x960, 4 at 640x480).
// Mixes (Interface's four): vortices (the lambda2-hat iso by speed, and 16
// streamlines), an absorbing |w| volume, a pressure slice with C_p on the
// wall, and streamlines alone. Lines are the editor's: the live preset (one
// box diagonal each way) through one ViewCache kept across views and frames,
// so each frame traces once. Volumes:
// the tour's 32^3, and 64^3 for headroom. Per frame, the median of 8 after 2
// warm-ups: the whole frame (apply_view() and draw() for every view, to
// glFinish()), and apply_view()'s CPU share of it. A measurement, so opt-in
// (SPADE_GL_LAYOUT_COST=1).
// Measured 2026-10-19 on the RTX 3060 Ti (572.83), other realms working, a
// frame over the six layouts: at 32^3, vortices 13 to 18 ms, |w| volume 5 to
// 11, slice + C_p 2 to 9, lines 8 to 13; at 64^3, 23 to 32, 8 to 17, 3 to 10
// and 14 to 26. The lines' trace is about 7 ms a frame at 32^3 (13 at 64^3),
// once however many views. Before the cache and the live preset (default
// options, each view tracing), the line mixes took 130 to 665 ms.
TEST_F(GpuGlRenderer, EditorLayoutsCost) {
    if (!env_gate_is_set("SPADE_GL_LAYOUT_COST")) {
        GTEST_SKIP() << "a measurement: set SPADE_GL_LAYOUT_COST=1 to run it";
    }
    auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress),
                                   GlRenderer::CreateOptions{.verify_cached_volumes = false});
    ASSERT_TRUE(made.has_value()) << made.error().context;
    GlRenderer& r = **made;
    constexpr int kW = 1920, kH = 1080;
    GLuint fbo = 0, colour = 0, depth = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenRenderbuffers(1, &colour);
    glBindRenderbuffer(GL_RENDERBUFFER, colour);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, kW, kH);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, kW, kH);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
    ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));
    const std::string device = r.renderer_name() + " (" + r.version_string() + ")";

    struct Layout {
        const char* name;
        int views, width, height;
    };
    const Layout layouts[] = {{"1 x 1920x1080", 1, 1920, 1080}, {"2 x 960x1080", 2, 960, 1080},
                              {"4 x 960x540", 4, 960, 540},     {"1 x 1280x960", 1, 1280, 960},
                              {"2 x 640x960", 2, 640, 960},     {"4 x 640x480", 4, 640, 480}};
    using spade::render::FlowView;
    using spade::render::FlowViewSpec;

    for (const uint32_t n : {32u, 64u}) {
        spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / static_cast<float>(n), n, n,
                                                                 n, spade::test::vortex_ring_with_dye());
        const float q_max = volume.header.channel_max[spade::kFlowQHat];
        FlowViewSpec iso;
        iso.kind = FlowView::vortex_iso;
        iso.quantity = spade::render::FlowQuantity::lambda2_hat;
        iso.iso_value = -0.5477226f * q_max;  // the ring's lambda2-hat is -Q-hat
        iso.range_max = 0.9f;
        FlowViewSpec lines;
        lines.kind = FlowView::streamlines;
        lines.range_max = 0.9f;
        lines.lines = spade::render::StreamlineOptions::live();  // the editor's: one box diagonal each way
        for (int i = 0; i < 16; ++i) {
            const float a = 6.2831853f * static_cast<float>(i) / 16.0f;
            lines.seeds.push_back(glm::vec3(0.45f * std::cos(a), 0.45f * std::sin(a), -0.6f));
        }
        FlowViewSpec vorticity;
        vorticity.kind = FlowView::volume;
        vorticity.quantity = spade::render::FlowQuantity::vorticity;
        vorticity.colours = spade::render::ColourMapId::inferno;
        vorticity.range_max = 22.0f;
        vorticity.onset = 2.0f / 22.0f;
        vorticity.sigma = 12.0f;
        FlowViewSpec slice;
        slice.kind = FlowView::slice;
        slice.quantity = spade::render::FlowQuantity::pressure;
        slice.plane.width = 2.0f;
        slice.plane.height = 2.0f;
        slice.plane.cells_u = 128;
        slice.plane.cells_v = 128;
        slice.range_min = volume.header.channel_min[spade::kFlowPressure];
        slice.range_max = volume.header.channel_max[spade::kFlowPressure] + 1e-3f;
        FlowViewSpec cp;
        cp.kind = FlowView::surface_cp;
        cp.q_ref = 0.5f;
        cp.range_min = -1.0f;
        cp.range_max = 1.0f;
        struct Mix {
            const char* name;
            spade::render::ViewSpec spec;
        };
        const Mix mixes[] = {{"vortices", {spade::render::ViewBase::shaded, {iso, lines}}},
                             {"|w| volume", {spade::render::ViewBase::shaded, {vorticity}}},
                             {"slice + C_p", {spade::render::ViewBase::shaded, {slice, cp}}},
                             {"lines", {spade::render::ViewBase::shaded, {lines}}}};

        RenderScene scene = ring_cut_scene();
        scene.statics[0].flow_wall = true;
        ASSERT_TRUE(r.upload_scene(scene).has_value());
        for (const Mix& mix : mixes) {
            for (const Layout& layout : layouts) {
                std::vector<double> frame_ms, apply_ms;
                spade::render::ViewCache cache;  // the editor's: one across its views, frame to frame
                for (int i = 0; i < 10; ++i) {
                    volume.generation = spade::next_flow_generation();
                    glFinish();
                    const auto t0 = std::chrono::steady_clock::now();
                    double applying = 0.0;
                    for (int v = 0; v < layout.views; ++v) {
                        RenderOptions options = comparable_options();
                        const auto a0 = std::chrono::steady_clock::now();
                        const auto applied = spade::render::apply_view(mix.spec, &volume, scene, options, cache);
                        applying += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a0)
                                        .count();
                        ASSERT_TRUE(applied.has_value()) << mix.name << ": " << applied.error().context;
                        const spade::Result<void> drew = r.draw(scene, camera_on_ring(), options,
                                                                static_cast<uint32_t>(layout.width),
                                                                static_cast<uint32_t>(layout.height));
                        ASSERT_TRUE(drew.has_value()) << mix.name << ": " << drew.error().context;
                    }
                    glFinish();
                    const auto t1 = std::chrono::steady_clock::now();
                    if (i >= 2) {
                        frame_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
                        apply_ms.push_back(applying);
                    }
                }
                std::sort(frame_ms.begin(), frame_ms.end());
                std::sort(apply_ms.begin(), apply_ms.end());
                const double frame = 0.5 * (frame_ms[3] + frame_ms[4]);
                const double apply = 0.5 * (apply_ms[3] + apply_ms[4]);
                std::printf("[ layout cost ] %u^3 | %-11s | %-13s | frame %7.2f ms | apply_view %6.2f ms\n", n,
                            mix.name, layout.name, frame, apply);
            }
        }
    }
    std::printf("[ layout cost ] on %s\n", device.c_str());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);
}

}  // namespace
