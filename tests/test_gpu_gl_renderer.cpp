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
#include <array>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <set>
#include <functional>
#include <cstring>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "render/gltf.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/sprite_kit.hpp"
#include "render/planar_texture.hpp"
#include "render/shadow.hpp"
#include "render/target.hpp"
#include "render/terrain.hpp"
#include "world/terrain_input.hpp"
#include "render/tessellate.hpp"
#include "world/sdf.hpp"
#include "world/builder.hpp"
#include "render_gl/gl_renderer.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/publish.hpp"

#include "cp_sphere.hpp"
#include "flow_synthetic.hpp"
#include "solid_drone_rig.hpp"
#include "view_mode_cases.hpp"

// The tour's airflow chapter (sandbox/airflow_live.hpp), where the sandbox is
// built (its directory is then on the include path): its poster frame on GL.
#if __has_include("airflow_live.hpp")
#define SPADE_GL_TEST_HAS_TOUR 1
#include <filesystem>

#include "airflow_live.hpp"
#include "orbit_camera.hpp"
#include "render_parts/render_parts.hpp"
#include "vehicle_draw.hpp"
#endif

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

// Kat's sawtooth (2026-10-28) on both paths: a thin gate standing on the
// ground and a drone-sized box resting beside it, both red, under VQ1's sun
// (the default), in a shadow map as wide as VQ1's world (~172 m, ~15 cm
// texels along its long axis). Their lit faces must not shadow themselves on
// either path -- compared at their own depth, they did in bands on both --
// and GL's shadows must land where the CPU's do.
//
// Two runs (rendering/soft-shadows, RND-10). The HARD run is filter 0, the single texel, which is exactly the frame
// this test pinned before shadows went soft: its numbers are unchanged. The SOFT run is the default, a filter one
// texel wide, whose penumbra spreads a real shadow's edge by up to the filter's reach, so "a red pixel changes when
// shadows turn on" is no longer the test of acne (the gate shades its own reveals: the lintel's shadow on the left
// reveal's top, the right post's on the bottom reveal's end, slivers that fell between pixel centres at filter 0).
// Soft acne is a red pixel changed FARTHER than the filter's reach from every red pixel a hard 4096-texel reference
// shadows -- a pixel that is not beside any real shadow edge.
TEST_F(GpuGlRenderer, AGateInAWorldAsWideAsVq1DoesNotShadowItselfOnEitherPath) {
    // HARD run. Measured 2026-11-03 on rendering/shadow-acne, 160x120, on an NVIDIA
    // GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83): 3 of 2896 red pixels
    // shadowed on each path -- the hole's +x-facing inner wall at its top
    // corner, which the hole's top truly shades -- where comparing each
    // point's own depth shadowed 406 on each; 171 shadowed pixels on each
    // path, 0 misses either way, 0 levels over the 24 interior ones. Pinned at
    // the measurement; another device may differ, so re-measure there before
    // widening (03-verification).
    constexpr size_t kBandRedShadowed = 3;
    constexpr size_t kBandMisses = 0;
    constexpr int kBandInterior = 0;
    // SOFT run. A "miss" is a shadowed pixel (one a frame changes when shadows turn on) on one path with no shadowed
    // pixel within 1 px of it on the other. A penumbra pixel's sun term is scaled by a fractional weight, formed in
    // float on the CPU and in the driver's on GL, so a pixel darkened by a fraction of one level can round to a change
    // on one path and to none on the other, an isolated shadowed pixel with no partner: one such miss is the rounding
    // floor of fractional weights, and for the same reason an interior pixel can differ by one level. Measured
    // 2026-11-09, same device: 1 level, 0 CPU misses, 1 GL miss. At filter 0 the weights are 0 or 1 and need none.
    constexpr size_t kSoftBandMisses = 1;
    constexpr int kSoftBandInterior = 1;
    // Soft acne: a red pixel changed by MORE than that one-level rounding floor, farther than the filter's whole reach
    // from any pixel the hard reference shadows (a pixel GL darkens by one level and the CPU not at all, as measured at
    // (92,44), is that floor, not a shadow). The reach is 1.5
    // texels -- the point's own texel's centre is within 0.5 of it, a tap up to a texel beyond -- computed below in
    // pixels from the map's texel (~0.15 m, so 1.5 texels = 0.23 m) and this view's scale (the gate's face fills about
    // 31 px a metre at 160 px wide), which is 7 px. Not tuned to the measurement.
    constexpr size_t kBandSoftAcne = 0;

    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{.name = "ground", .base_color = {0.55f, 0.5f, 0.45f, 1.0f}});
    b.material(spade::MaterialDesc{.name = "gate", .base_color = {0.85f, 0.15f, 0.1f, 1.0f}});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).material_for_last_node(0);
    const spade::SdfPose gate_at{.position = {0.0f, 1.0f, 0.0f}};
    b.box(glm::vec3(1.0f, 1.0f, 0.05f), gate_at).material_for_last_node(1);
    b.box(glm::vec3(0.7f, 0.7f, 0.2f), gate_at).material_for_last_node(1);
    b.subtract().material_for_last_node(1);
    b.union_();
    // A lone box is drawn as a mesh, so its lit faces check the mesh path.
    b.box(glm::vec3(0.3f, 0.15f, 0.3f), spade::SdfPose{.position = {1.7f, 0.15f, 0.7f}}).material_for_last_node(1);
    b.union_();
    RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-80.0f, -1.0f, -10.0f), .max = glm::vec3(92.0f, 30.0f, 10.0f)};
    auto map = spade::render::build_static_shadow_map(scene);
    ASSERT_TRUE(map.has_value()) << map.error().context;
    scene.static_shadow = std::move(*map);
    Camera camera;  // in front of the gate's lit face, the box to its right
    camera.position = glm::vec3(0.6f, 1.3f, 3.2f);
    camera.orientation = glm::angleAxis(glm::radians(-12.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    RenderOptions on = comparable_options();  // soft: the default filter
    on.shadows = true;
    RenderOptions hard = on;
    hard.shadow_filter_texels = 0.0f;
    const RenderOptions off = comparable_options();
    const std::vector<uint8_t> cpu_on = cpu_rgba(scene, camera, on);
    const std::vector<uint8_t> cpu_hard = cpu_rgba(scene, camera, hard);
    const std::vector<uint8_t> cpu_off = cpu_rgba(scene, camera, off);
    const std::vector<uint8_t> gl_on = draw_and_read(scene, on, camera);
    const std::vector<uint8_t> gl_hard = draw_and_read(scene, hard, camera);
    const std::vector<uint8_t> gl_off = draw_and_read(scene, off, camera);
    ASSERT_EQ(gl_on.size(), kPixels * 4u);
    ASSERT_EQ(gl_hard.size(), kPixels * 4u);
    ASSERT_EQ(gl_off.size(), kPixels * 4u);

    // The gate's and the box's pixels are those red well over green with
    // shadows off; a shadowed one changes when they turn on.
    const auto is_red = [](const std::vector<uint8_t>& frame, size_t p) {
        return static_cast<int>(frame[p * 4u]) > static_cast<int>(frame[p * 4u + 1u]) + 50;
    };
    const auto gate_shadowed = [&](const std::vector<uint8_t>& with, const std::vector<uint8_t>& without,
                                   size_t& gate) {
        size_t shadowed = 0;
        for (size_t p = 0; p < kPixels; ++p) {
            if (!is_red(without, p)) continue;
            ++gate;
            shadowed += same_pixel(with, without, p) ? 0u : 1u;
        }
        return shadowed;
    };

    // --- the hard run: today's frame, today's numbers --------------------------------------------------------------
    size_t cpu_gate = 0, gl_gate = 0;
    const size_t cpu_acne = gate_shadowed(cpu_hard, cpu_off, cpu_gate);
    const size_t gl_acne = gate_shadowed(gl_hard, gl_off, gl_gate);
    const ShadowMatch m = match_shadows(cpu_hard, cpu_off, gl_hard, gl_off);
    std::printf("[ acne    ] gate and box, VQ1-wide map, filter 0: red pixels shadowed CPU %zu of %zu, GL %zu of %zu; "
                "shadowed CPU %zu GL %zu, misses %zu and %zu, interior %zu up to %d levels\n",
                cpu_acne, cpu_gate, gl_acne, gl_gate, m.cpu_shadowed, m.gl_shadowed, m.cpu_misses, m.gl_misses,
                m.interior, m.max_interior);
    ASSERT_GT(cpu_gate, kPixels / 20u) << "the gate must fill a twentieth of the frame";
    ASSERT_GT(gl_gate, kPixels / 20u) << "the gate must fill a twentieth of GL's frame";
    EXPECT_LE(cpu_acne, kBandRedShadowed) << "a lit red face shadows itself on the CPU";
    EXPECT_LE(gl_acne, kBandRedShadowed) << "a lit red face shadows itself on GL, on " << renderer_->renderer_name();
    EXPECT_LE(m.max_interior, kBandInterior) << "inside the shadow (" << m.interior
                                             << " pixels) GL differs from the CPU by up to " << m.max_interior
                                             << " levels";
    EXPECT_LE(m.cpu_misses, kBandMisses) << m.cpu_misses << " of " << m.cpu_shadowed
                                         << " CPU shadow pixels have no GL shadow within 1 px";
    EXPECT_LE(m.gl_misses, kBandMisses) << m.gl_misses << " of " << m.gl_shadowed
                                        << " GL shadow pixels have no CPU shadow within 1 px";

    // --- the soft run: no red pixel changed beside no real shadow --------------------------------------------------
    RenderScene reference = scene;  // a 4096-texel map, a hard lookup: the shadow to within a few millimetres
    auto fine = spade::render::build_static_shadow_map(reference, 4096);
    ASSERT_TRUE(fine.has_value()) << fine.error().context;
    reference.static_shadow = std::move(*fine);
    const std::vector<uint8_t> truth = cpu_rgba(reference, camera, hard);
    std::vector<std::pair<int, int>> truth_pixels;  // (x, y) of each red pixel the reference shadows
    for (size_t p = 0; p < kPixels; ++p) {
        if (is_red(cpu_off, p) && !same_pixel(truth, cpu_off, p)) {
            truth_pixels.emplace_back(static_cast<int>(p % kWidth), static_cast<int>(p / kWidth));
        }
    }
    ASSERT_GT(truth_pixels.size(), 0u) << "the gate's own shadow edges must show in the reference";
    const float texel_m = spade::render::shadow_texel_metres(*scene.static_shadow);
    const float face_distance_m = camera.position.z - 0.05f;  // to the gate's lit face
    const float px_per_m = static_cast<float>(kWidth) /
                           (2.0f * face_distance_m * std::tan(camera.fov_y_radians * 0.5f) *
                            (static_cast<float>(kWidth) / static_cast<float>(kHeight)));
    const int reach_px = static_cast<int>(std::lround(1.5f * texel_m * px_per_m));
    const auto soft_acne = [&](const std::vector<uint8_t>& with, int& worst_x, int& worst_y, int& worst_dist,
                               int& worst_levels) {
        size_t far = 0;
        worst_dist = 0;
        worst_levels = 0;
        for (size_t p = 0; p < kPixels; ++p) {
            if (!is_red(cpu_off, p) || same_pixel(with, cpu_off, p)) continue;
            int levels = 0;
            for (size_t c = 0; c < 3u; ++c) {
                levels = std::max(levels, std::abs(static_cast<int>(with[p * 4u + c]) - static_cast<int>(cpu_off[p * 4u + c])));
            }
            if (levels <= kSoftBandInterior) continue;  // within the rounding floor of fractional weights (above)
            const int x = static_cast<int>(p % kWidth), y = static_cast<int>(p / kWidth);
            int nearest = std::numeric_limits<int>::max();
            for (const auto& [tx, ty] : truth_pixels) nearest = std::min(nearest, std::max(std::abs(tx - x), std::abs(ty - y)));
            if (nearest > reach_px) ++far;
            if (nearest > worst_dist) {
                worst_dist = nearest;
                worst_x = x;
                worst_y = y;
                worst_levels = levels;
            }
        }
        return far;
    };
    int cx = -1, cy = -1, cd = 0, cl = 0, gx = -1, gy = -1, gd = 0, gl_levels = 0;
    const size_t cpu_far = soft_acne(cpu_on, cx, cy, cd, cl);
    const size_t gl_far = soft_acne(gl_on, gx, gy, gd, gl_levels);
    size_t soft_cpu_gate = 0, soft_gl_gate = 0;
    const size_t soft_cpu = gate_shadowed(cpu_on, cpu_off, soft_cpu_gate);
    const size_t soft_gl = gate_shadowed(gl_on, gl_off, soft_gl_gate);
    const ShadowMatch ms = match_shadows(cpu_on, cpu_off, gl_on, gl_off);
    std::printf("[ acne    ] gate and box, VQ1-wide map, filter 1: red pixels changed CPU %zu GL %zu, of which farther than "
                "%d px (1.5 texels of %.3f m at %.1f px/m) from the reference shadow: CPU %zu (worst %d px at %d,%d, "
                "%d levels), GL %zu (worst %d px at %d,%d, %d levels); shadowed CPU %zu GL %zu, misses %zu and %zu, "
                "interior %zu up to %d levels\n",
                soft_cpu, soft_gl, reach_px, texel_m, px_per_m, cpu_far, cd, cx, cy, cl, gl_far, gd, gx, gy, gl_levels, ms.cpu_shadowed,
                ms.gl_shadowed, ms.cpu_misses, ms.gl_misses, ms.interior, ms.max_interior);
    EXPECT_LE(cpu_far, kBandSoftAcne) << cpu_far << " red pixels changed on the CPU farther than " << reach_px
                                      << " px from any pixel the reference shadows (worst: " << cd << " px, at x=" << cx
                                      << " y=" << cy << ", rows bottom-up)";
    EXPECT_LE(gl_far, kBandSoftAcne) << gl_far << " red pixels changed on GL farther than " << reach_px
                                     << " px from any pixel the reference shadows (worst: " << gd << " px, at x=" << gx
                                     << " y=" << gy << ", rows bottom-up), on " << renderer_->renderer_name();
    EXPECT_LE(ms.max_interior, kSoftBandInterior) << "inside the soft shadow (" << ms.interior
                                                  << " pixels) GL differs from the CPU by up to " << ms.max_interior
                                                  << " levels";
    EXPECT_LE(ms.cpu_misses, kSoftBandMisses) << ms.cpu_misses << " of " << ms.cpu_shadowed
                                              << " CPU soft-shadow pixels have no GL shadow within 1 px";
    EXPECT_LE(ms.gl_misses, kSoftBandMisses) << ms.gl_misses << " of " << ms.gl_shadowed
                                             << " GL soft-shadow pixels have no CPU shadow within 1 px";
}

// The ground alone, in a world as long as VQ1 but laid out along the sun's
// horizontal (400 m along z), so the ground's slope runs along the map's
// coarse axis. Nothing but the ground casts, so with shadows on no pixel may
// change, on either path; compared at its own depth, the ground shadowed
// itself in bands.
TEST_F(GpuGlRenderer, TheGroundInAWorldLongAlongTheSunDoesNotShadowItselfOnEitherPath) {
    spade::WorldBuilder b = csg_builder();
    b.material(spade::MaterialDesc{.name = "ground", .base_color = {0.55f, 0.5f, 0.45f, 1.0f}});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).material_for_last_node(0);
    RenderScene scene = csg_scene_or_fail(b);
    ASSERT_EQ(scene.ground_planes.size(), 1u);
    scene.bounds =
        spade::render::Aabb{.min = glm::vec3(-10.0f, -1.0f, -200.0f), .max = glm::vec3(10.0f, 10.0f, 200.0f)};
    auto map = spade::render::build_static_shadow_map(scene);
    ASSERT_TRUE(map.has_value()) << map.error().context;
    scene.static_shadow = std::move(*map);
    Camera camera;  // over the ground, pitched down at it
    camera.position = glm::vec3(0.0f, 3.0f, 6.0f);
    camera.orientation = glm::angleAxis(glm::radians(-30.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    RenderOptions on = comparable_options();
    on.shadows = true;
    const RenderOptions off = comparable_options();
    const std::vector<uint8_t> cpu_on = cpu_rgba(scene, camera, on);
    const std::vector<uint8_t> cpu_off = cpu_rgba(scene, camera, off);
    const std::vector<uint8_t> gl_on = draw_and_read(scene, on, camera);
    const std::vector<uint8_t> gl_off = draw_and_read(scene, off, camera);
    ASSERT_EQ(gl_on.size(), kPixels * 4u);
    ASSERT_EQ(gl_off.size(), kPixels * 4u);
    const size_t ground = count_non_black(cpu_off);
    const size_t cpu_shadowed = count_differing(cpu_on, cpu_off);
    const size_t gl_shadowed = count_differing(gl_on, gl_off);
    std::printf("[ acne    ] ground, map long along the sun: shadowed CPU %zu, GL %zu, of %zu ground pixels\n",
                cpu_shadowed, gl_shadowed, ground);
    ASSERT_GT(ground, kPixels / 3u) << "the ground must fill a third of the frame";
    EXPECT_EQ(cpu_shadowed, 0u) << "the ground shadows itself on the CPU";
    EXPECT_EQ(gl_shadowed, 0u) << "the ground shadows itself on GL, on " << renderer_->renderer_name();
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

// L6: GL marches a subtree from the program and transforms upload_scene()
// copied to the GPU, but takes its box from the scene at draw. A subtree moved
// or edited since its upload -- as render_parts re-binds a moved part, posed
// into its program -- would march at its old pose inside its new box: clipped,
// or gone. draw() refuses it, naming it, and an upload of the scene clears it.
// The move keeps the node count, the only thing draw() compared before.
TEST_F(GpuGlRenderer, RefusesACsgSubtreeMovedOrEditedSinceItsUpload) {
    const RenderScene good = slab_scene();
    ASSERT_EQ(good.csg_subtrees.size(), 1u);
    ASSERT_TRUE(renderer_->upload_scene(good).has_value());
    const auto draw = [this](const RenderScene& scene) {
        return renderer_->draw(scene, camera_on_plus_z(), spade::render::RenderOptions{}, kWidth, kHeight);
    };
    ASSERT_TRUE(draw(good).has_value()) << "the uploaded scene draws";

    RenderScene moved = good;
    for (spade::SdfTransform& t : moved.csg_subtrees[0].program.transforms) {
        t.world_to_local = t.world_to_local * glm::translate(glm::mat4(1.0f), glm::vec3(-0.5f, 0.0f, 0.0f));
    }
    moved.csg_subtrees[0].bounds.min.x += 0.5f;
    moved.csg_subtrees[0].bounds.max.x += 0.5f;
    RenderScene edited = good;
    edited.csg_subtrees[0].program.nodes.front().params.x *= 1.5f;
    RenderScene boxed = good;
    boxed.csg_subtrees[0].bounds.max.y += 0.25f;

    for (const auto& [what, scene] : {std::pair<const char*, const RenderScene*>{"moved", &moved},
                                      {"a parameter edited", &edited},
                                      {"its box changed", &boxed}}) {
        SCOPED_TRACE(what);
        const spade::Result<void> drew = draw(*scene);
        ASSERT_FALSE(drew.has_value()) << "a subtree " << what << " since its upload must be refused, never marched stale";
        EXPECT_EQ(drew.error().code, spade::Code::invalid_argument);
        EXPECT_NE(drew.error().context.find("CSG subtree 0"), std::string::npos) << drew.error().context;
        EXPECT_NE(drew.error().context.find("upload_scene"), std::string::npos) << drew.error().context;
    }

    // A transform the subtree does not march is not its concern.
    RenderScene unrelated = good;
    unrelated.csg_subtrees[0].program.transforms.push_back(spade::SdfTransform{});
    const spade::Result<void> drew_unrelated = draw(unrelated);
    EXPECT_TRUE(drew_unrelated.has_value()) << drew_unrelated.error().context;

    ASSERT_TRUE(renderer_->upload_scene(moved).has_value());
    const spade::Result<void> drew_moved = draw(moved);
    EXPECT_TRUE(drew_moved.has_value()) << "uploaded again, the moved subtree draws: " << drew_moved.error().context;
}

// The hosting contract (docs/design/rendering/render-gl-hosting.md): draw()
// SETS the state it relies on. A host that left scissor, stencil, rasterizer
// discard, sRGB encoding, a colour mask, a depth range or an unpack buffer
// behind -- an immediate-mode UI does, routinely -- gets the very bytes a
// clean draw gives. The target is sRGB, so encoding left on would show, and
// has stencil, so a failing stencil test would; the flow volume's generation
// is 0, so it uploads on every draw and an unpack buffer left bound would
// feed it.
TEST_F(GpuGlRenderer, DrawSetsTheStateItReliesOn) {
    ASSERT_TRUE(renderer_->flow_ready());
    GLuint fbo = 0, colour = 0, depth = 0, pbo = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenRenderbuffers(1, &colour);
    glBindRenderbuffer(GL_RENDERBUFFER, colour);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_SRGB8_ALPHA8, kWidth, kHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, kWidth, kHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth);
    ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));
    glGenBuffers(1, &pbo);

    RenderScene scene = slab_scene();
    spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 0.25f, 8, 8, 8,
                                                             spade::test::uniform_flow(glm::vec3(1.0f, 0.0f, 0.0f)));
    volume.generation = 0;
    spade::render::TransferFunction speed;
    speed.extinction = {{0.0f, 0.5f}, {2.0f, 0.5f}};
    scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {speed}, {}});
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());

    const auto frame = [&]() -> std::vector<uint8_t> {
        if (auto drew = renderer_->draw(scene, camera_on_plus_z(), comparable_options(), kWidth, kHeight); !drew) {
            ADD_FAILURE() << drew.error().context;
            return {};
        }
        std::vector<uint8_t> rgba(kPixels * 4u);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        EXPECT_EQ(glGetError(), static_cast<GLenum>(GL_NO_ERROR));
        return rgba;
    };

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    const std::vector<uint8_t> clean = frame();

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_SCISSOR_TEST);
    glScissor(0, 0, 8, 8);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_NEVER, 0, 0xFF);
    glEnable(GL_FRAMEBUFFER_SRGB);
    glColorMask(GL_TRUE, GL_FALSE, GL_TRUE, GL_TRUE);
    glDepthRange(0.5, 1.0);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, 64, nullptr, GL_STREAM_DRAW);
    glEnable(GL_RASTERIZER_DISCARD);
    const std::vector<uint8_t> dirty = frame();

    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_RASTERIZER_DISCARD);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthRange(0.0, 1.0);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteBuffers(1, &pbo);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);

    ASSERT_EQ(clean.size(), dirty.size());
    ASSERT_FALSE(clean.empty());
    size_t differ = 0;
    for (size_t i = 0; i < clean.size(); ++i) differ += clean[i] != dirty[i] ? 1u : 0u;
    EXPECT_EQ(differ, 0u) << "a host's leftover state changed " << differ << " bytes of the frame";
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

// An environment variable's text, "" when it is not set (the same C4996 as below).
[[nodiscard]] std::string env_text(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* v = std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    return v != nullptr ? std::string(v) : std::string();
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

// L74's first metric (the lead, 2026-11-11; Cameron: "drastically speed up ... training/physics/rendering"): what the
// editor's live training view waits on. KAT's view draws the run's drones as render-only puppets (a pose each frame, no
// physics) through this renderer, so a frame is the poses refreshed (update_dynamics(), which the host calls each render),
// the draw submitted, the GPU's time, and the read back. The scene has static geometry (the ground, six gate-sized boxes) and
// so a static shadow map, as a world's does: without one GL treats every pixel as unshadowed and the dynamic casters' pass
// never runs.
// A measurement, so opt-in: SPADE_GL_PUPPET_COST=1 (about 10 s). SPADE_PUPPET_GLTF=<file> draws that mesh for every puppet
// (KAT's content/meshes/drone/competition-700.gltf: 15 submeshes, 14,100 triangles); without it a UV sphere of 14,000
// triangles stands in, the same triangle count and one submesh. The puppets are DYNAMIC items (a pose per frame), as the
// host's are, over the analytic ground, 1280 x 720, the default sun with its shadows ON and then OFF; the frame's
// shadow cost is the difference (the dynamic casters drawn into the sun's map, plus their sampling). Each figure is the
// median of 20 frames after 3 warm-ups: `pose` the CPU filling the BodyPoses and update_dynamics() (and the clear), `submit` draw()
// returning (the instance buffers refreshed, the shadow pass and the draws issued), `wait` the CPU blocked in glFinish()
// after it, `gpu` a GL_TIME_ELAPSED query around draw(), `read` glReadPixels of the frame into memory.
// The ledger's metric (Test/Docs, PERF.md) is the in-repo workload, the UV sphere: with it each row also prints one
// `PERF rendering.puppet_frame_<N>_shadows_<on|off> <median ms> ms` line (the median of the whole frame, pose to read-back)
// and `PERF rendering.diag_<phase>_<N>_shadows_<on|off> <ms> ms` for each phase. With SPADE_PUPPET_GLTF (a one-off check
// against KAT's mesh) the PERF lines are not printed, so no ledger row rests on a file outside the repo.
TEST_F(GpuGlRenderer, PuppetFrameCostAt128And1024Drones) {
    if (!env_gate_is_set("SPADE_GL_PUPPET_COST")) {
        GTEST_SKIP() << "a measurement: set SPADE_GL_PUPPET_COST=1 to run it (SPADE_PUPPET_GLTF=<a drone .gltf> for a real mesh)";
    }
    constexpr int kW = 1280, kH = 720;
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

    // The puppet's mesh.
    MeshData drone;
    std::string source = "a UV sphere, 14,000 triangles";
    const std::string gltf_path = env_text("SPADE_PUPPET_GLTF");
    if (!gltf_path.empty()) {
        auto loaded = spade::render::load_gltf(gltf_path);
        ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
        drone = std::move(*loaded);
        drone.source_materials.clear();  // the scene's one material draws every submesh
        std::fill(drone.submesh_material.begin(), drone.submesh_material.end(), 0u);
        source = gltf_path + ", " + std::to_string(drone.indices.size() / 3u) + " triangles, " +
                 std::to_string(drone.submesh_first_index.size()) + " submeshes";
    } else {
        constexpr uint32_t kLat = 70, kLon = 100;  // 2 * 70 * 100 = 14,000 triangles
        for (uint32_t i = 0; i <= kLat; ++i) {
            const float a = glm::pi<float>() * static_cast<float>(i) / static_cast<float>(kLat);
            for (uint32_t j = 0; j <= kLon; ++j) {
                const float b = 2.0f * glm::pi<float>() * static_cast<float>(j) / static_cast<float>(kLon);
                const glm::vec3 n(std::sin(a) * std::cos(b), std::cos(a), std::sin(a) * std::sin(b));
                drone.positions.push_back(0.16f * n);
                drone.normals.push_back(n);
            }
        }
        for (uint32_t i = 0; i < kLat; ++i) {
            for (uint32_t j = 0; j < kLon; ++j) {
                const uint32_t p = i * (kLon + 1u) + j, q = p + kLon + 1u;
                for (const uint32_t v : {p, q, p + 1u, p + 1u, q, q + 1u}) drone.indices.push_back(v);
            }
        }
        drone.submesh_first_index = {0u};
        drone.submesh_index_count = {static_cast<uint32_t>(drone.indices.size())};
        drone.submesh_material = {0u};
    }
    const double triangles = static_cast<double>(drone.indices.size() / 3u);

    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    const auto median = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        return 0.5 * (v[(v.size() - 1u) / 2u] + v[v.size() / 2u]);
    };
    const auto ms_between = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    std::printf("[ puppets ] %dx%d, mesh %s; on %s\n", kW, kH, source.c_str(), device.c_str());
    for (const uint32_t n : {1u, 128u, 1024u}) {
        const uint32_t cols = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(n))));
        const float spacing = 1.0f;  // a training run's drones are spread about its world; a metre apart here
        const float extent = std::max(spacing * static_cast<float>(cols), 4.0f);
        const float reach = 0.5f * extent + 6.0f;
        const spade::render::Aabb tess_bounds{.min = glm::vec3(-reach), .max = glm::vec3(reach)};
        const auto tessellated = [&](spade::SdfPrim kind, glm::vec4 params) {
            auto made = spade::render::tessellate_primitive(kind, params, tess_bounds, spade::render::kTessellationDefaults);
            EXPECT_TRUE(made.has_value());
            return made ? std::move(*made) : MeshData{};
        };
        RenderScene scene;
        scene.meshes.push_back(drone);                                                        // 0: the puppets
        scene.meshes.push_back(tessellated(spade::SdfPrim::plane, glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));  // 1: the ground
        scene.meshes.push_back(tessellated(spade::SdfPrim::box, glm::vec4(1.0f, 1.0f, 0.1f, 0.0f)));    // 2: a gate-sized box
        scene.materials = {Material{.base_color = glm::vec4(0.7f, 0.72f, 0.75f, 1.0f)}};
        scene.ground_planes = {GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = 0.0f, .material = 0}};
        scene.statics.push_back(DrawItem{.mesh_index = 1, .local_to_world = glm::mat4(1.0f)});
        for (int g = 0; g < 6; ++g) {
            const float a = 2.0f * glm::pi<float>() * static_cast<float>(g) / 6.0f;
            scene.statics.push_back(DrawItem{
                .mesh_index = 2,
                .local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(0.45f * extent * std::cos(a), 1.0f, 0.45f * extent * std::sin(a)))});
        }
        scene.bounds = spade::render::Aabb{.min = glm::vec3(-reach, -1.0f, -reach), .max = glm::vec3(reach, 6.0f, reach)};
        const auto shadow_start = std::chrono::steady_clock::now();
        auto static_map = spade::render::build_static_shadow_map(scene);
        ASSERT_TRUE(static_map.has_value()) << static_map.error().context;
        const double shadow_build_ms = ms_between(shadow_start, std::chrono::steady_clock::now());
        scene.static_shadow = std::move(*static_map);
        std::vector<spade::render::BodyPose> bodies(n);
        const auto pose = [&](float t) {
            for (uint32_t i = 0; i < n; ++i) {
                bodies[i].position = glm::vec3(spacing * static_cast<float>(i % cols) - 0.5f * extent,
                                               1.0f + 0.2f * std::sin(t + 0.3f * static_cast<float>(i)),
                                               spacing * static_cast<float>(i / cols) - 0.5f * extent);
                bodies[i].orientation = glm::angleAxis(0.5f * t + 0.1f * static_cast<float>(i), glm::vec3(0.0f, 1.0f, 0.0f));
                bodies[i].velocity = glm::vec3(0.0f, 0.2f, 0.0f);
                bodies[i].mesh_index = 0;
            }
            spade::render::update_dynamics(scene, bodies);
        };
        pose(0.0f);
        // The camera: the whole formation in view, from above and to one side, as the free camera sits.
        Camera camera;
        camera.position = glm::vec3(0.0f, 0.55f * extent + 1.5f, 0.95f * extent + 2.5f);
        const glm::mat4 view = glm::lookAt(camera.position, glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        camera.orientation = glm::quat_cast(glm::inverse(view));
        const auto up_start = std::chrono::steady_clock::now();
        ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
        glFinish();
        const double upload_ms = ms_between(up_start, std::chrono::steady_clock::now());
        std::printf("[ puppets ] %u drones (%.1f M triangles): upload_scene (the mesh set and static map, once) %.2f ms; "
                    "build_static_shadow_map (once) %.2f ms\n", n, 1e-6 * triangles * n, upload_ms, shadow_build_ms);
        GLuint query = 0;
        glGenQueries(1, &query);
        struct Row {
            double pose = 0, submit = 0, wait = 0, gpu = 0, read = 0;
            uint32_t draws = 0, instances = 0;
            double frame = 0;
        };
        Row rows[2];
        std::vector<uint8_t> rgba(static_cast<size_t>(kW) * kH * 4u);
        for (int shadows = 1; shadows >= 0; --shadows) {
            RenderOptions options = comparable_options();
            options.shadows = shadows == 1;
            std::vector<double> pose_ms, submit_ms, wait_ms, gpu_ms, read_ms, frame_ms;
            for (int frame = 0; frame < 23; ++frame) {
                const auto t_pose = std::chrono::steady_clock::now();
                pose(0.05f * static_cast<float>(frame));
                glBindFramebuffer(GL_FRAMEBUFFER, fbo);
                glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glFinish();
                const auto t0 = std::chrono::steady_clock::now();
                glBeginQuery(GL_TIME_ELAPSED, query);
                const spade::Result<void> drew = renderer_->draw(scene, camera, options, kW, kH);
                glEndQuery(GL_TIME_ELAPSED);
                const auto t1 = std::chrono::steady_clock::now();
                glFinish();
                const auto t2 = std::chrono::steady_clock::now();
                glPixelStorei(GL_PACK_ALIGNMENT, 1);
                glReadPixels(0, 0, kW, kH, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
                const auto t3 = std::chrono::steady_clock::now();
                ASSERT_TRUE(drew.has_value()) << drew.error().context;
                GLuint64 ns = 0;
                glGetQueryObjectui64v(query, GL_QUERY_RESULT, &ns);
                if (frame < 3) continue;
                pose_ms.push_back(ms_between(t_pose, t0));
                submit_ms.push_back(ms_between(t0, t1));
                wait_ms.push_back(ms_between(t1, t2));
                gpu_ms.push_back(1e-6 * static_cast<double>(ns));
                read_ms.push_back(ms_between(t2, t3));
                frame_ms.push_back(ms_between(t_pose, t3));
            }
            Row& r = rows[shadows];
            r = Row{median(pose_ms), median(submit_ms), median(wait_ms), median(gpu_ms), median(read_ms),
                    renderer_->last_draw_calls(), renderer_->last_instances(), median(frame_ms)};
            if (gltf_path.empty()) {
                const std::string tag = std::to_string(n) + "_shadows_" + (shadows == 1 ? "on" : "off");
                std::printf("PERF rendering.puppet_frame_%s %.3f ms\n", tag.c_str(), r.frame);
                std::printf("PERF rendering.diag_pose_%s %.3f ms\nPERF rendering.diag_submit_%s %.3f ms\n"
                            "PERF rendering.diag_wait_%s %.3f ms\nPERF rendering.diag_gpu_%s %.3f ms\n"
                            "PERF rendering.diag_read_%s %.3f ms\n",
                            tag.c_str(), r.pose, tag.c_str(), r.submit, tag.c_str(), r.wait, tag.c_str(), r.gpu,
                            tag.c_str(), r.read);
            }
            std::printf("[ puppets ] %4u drones, shadows %-3s: pose+clear %6.2f ms, submit %6.2f ms, wait %6.2f ms, "
                        "gpu %6.2f ms, read %5.2f ms; frame %6.2f ms; %u draw calls, %u instances\n",
                        n, shadows == 1 ? "on" : "off", r.pose, r.submit, r.wait, r.gpu, r.read,
                        r.pose + r.submit + r.wait + r.read, r.draws, r.instances);
            RecordProperty(("gpu_ms_x100_" + std::to_string(n) + (shadows == 1 ? "_shadows" : "_noshadows")).c_str(),
                           static_cast<int>(r.gpu * 100.0));
        }
        std::printf("[ puppets ] %4u drones: the shadows cost submit %+.2f ms, gpu %+.2f ms\n", n,
                    rows[1].submit - rows[0].submit, rows[1].gpu - rows[0].gpu);
        glDeleteQueries(1, &query);
        EXPECT_LT(rows[1].gpu, 5000.0) << device;
        EXPECT_GE(rows[1].instances, n) << "the puppets are drawn";
    }
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

// SolidDroneCpMatchesTheCpu's band (D1's drawing side). Measured 2026-10-21,
// 160x120, on an NVIDIA GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83): of
// 859 coloured pixels 138 differ (16 %), all but one by at most two levels,
// as in the walls' band above (GL's interpolated point against the CPU's from
// depth). The one pixel more than two levels off (58; 34 when read at the
// wall) is taken to be a marched part's edge, which the two paths cover
// differently and CsgMatchesTheCpuWithinItsBand leaves out as a silhouette;
// it was not located.
// Its parts are CSG, so the offset follows the distance gradient on both.
constexpr size_t kSolidDroneOverTwo = 2;
constexpr size_t kSolidDroneDifferingPercent = 20;

// The C_p view over the reference quad's airframe as solids in the flow
// (tests/solid_drone_rig.hpp): one apply_view, drawn on GL and the CPU.
TEST_F(GpuGlRenderer, SolidDroneCpMatchesTheCpu) {
    const spade::test::SolidDrone d = spade::test::solid_drone();
    ASSERT_TRUE(d.error.empty()) << d.error;
    spade::render::FlowViewSpec cp;
    cp.kind = spade::render::FlowView::surface_cp;
    cp.colours = spade::render::ColourMapId::cool_warm;
    cp.range_min = -1.5f;
    cp.range_max = 1.0f;
    cp.q_ref = d.q_ref;
    RenderScene scene = d.scene;
    RenderOptions options = comparable_options();
    const auto legends =
        spade::render::apply_view(spade::render::ViewSpec{spade::render::ViewBase::shaded, {cp}}, &d.volume, scene,
                                  options);
    ASSERT_TRUE(legends.has_value()) << legends.error().context;
    Camera camera;
    camera.position = glm::vec3(-0.25f, 0.2f, 0.3f);
    camera.orientation = glm::quatLookAt(glm::normalize(-camera.position), glm::vec3(0.0f, 1.0f, 0.0f));
    camera.near_plane = 0.01f;
    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, options);
    const std::vector<uint8_t> cpu_bare = cpu_rgba(d.scene, camera, comparable_options());
    const std::vector<uint8_t> gl = draw_and_read(scene, options, camera);
    ASSERT_EQ(gl.size(), cpu.size());
    const size_t coloured = count_differing(cpu, cpu_bare);
    const ChannelDiff band = channel_diff(cpu, gl);
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ solid drone ] %zu px coloured; worst %d levels, %zu over two; %zu differ; on %s\n", coloured,
                band.worst, band.over_two, band.differing, device.c_str());
    ASSERT_GT(coloured, kPixels / 50u) << "the airframe takes the colour";
    EXPECT_LE(band.over_two, kSolidDroneOverTwo) << device;
    EXPECT_LE(band.differing * 100u, coloured * kSolidDroneDifferingPercent);
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

// The pressure film's read past a solid (O124), on GL as on the CPU: a drawn quad inside a slab of no-data cells takes the seen side's pressure from the first
// fluid sample outward along its normal (search_cells), where one cell off it the read is no-data. GL's frame is the CPU's, within one level.
TEST_F(GpuGlRenderer, ADrawnPointInsideASolidReadsThePastTheSolidLikeTheCpu) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-2.0f, -2.0f, -1.0f), 0.25f, 16, 16, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowPressure] = std::fabs(p.z) < 0.3f ? std::numeric_limits<float>::quiet_NaN() : (p.z > 0.0f ? 1.0f : -1.0f);
        return c;
    });
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::pressure;
    colours.colours = spade::render::ColourMapId::cool_warm;
    colours.range_min = -1.0f;
    colours.range_max = 1.0f;
    RenderScene scene;
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-1.5f, -1.5f, 0.0f), glm::vec3(1.5f, -1.5f, 0.0f), glm::vec3(1.5f, 1.5f, 0.0f), glm::vec3(-1.5f, 1.5f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    scene.meshes.push_back(quad);
    scene.materials = {Material{}};
    DrawItem item{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)};
    item.flow_wall = true;
    scene.statics.push_back(item);
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    const std::vector<uint8_t> shaded = cpu_rgba(scene, camera, comparable_options());
    spade::render::FlowSurfaceColouring colouring{&volume, colours};
    colouring.walls_only = true;
    colouring.offset_cells = 1.0f;
    colouring.search_cells = 2.0f;
    scene.flow_surface_colouring = colouring;
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, comparable_options());
    ASSERT_EQ(gl.size(), kPixels * 4u);
    const ChannelDiff painted = channel_diff(shaded, cpu);
    EXPECT_GT(painted.differing, kPixels / 50u) << "the CPU paints the quad from past the solid";
    const ChannelDiff band = channel_diff(cpu, gl);
    EXPECT_LE(band.worst, 1) << renderer_->renderer_name();
    EXPECT_EQ(band.over_two, 0u);
}

// The shaded surface colouring (O124) on GL as on the CPU: hue from the quantity, brightness from the sun's N.L over a floor; GL's frame is the CPU's within one level.
TEST_F(GpuGlRenderer, AShadedSurfaceColouringMatchesTheCpu) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-2.0f, -2.0f, -1.0f), 0.25f, 16, 16, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowPressure] = p.z > 0.0f ? 1.0f : -1.0f;
        return c;
    });
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::pressure;
    colours.colours = spade::render::ColourMapId::cool_warm;
    colours.range_min = -1.0f;
    colours.range_max = 1.0f;
    RenderScene scene;
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-1.5f, -1.5f, 0.0f), glm::vec3(1.5f, -1.5f, 0.0f), glm::vec3(1.5f, 1.5f, 0.0f), glm::vec3(-1.5f, 1.5f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    scene.meshes.push_back(quad);
    scene.materials = {Material{}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)});
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    scene.lighting.sun_direction = glm::normalize(glm::vec3(0.0f, 0.6f, 0.8f));
    spade::render::FlowSurfaceColouring colouring{&volume, colours};
    colouring.offset_cells = 1.0f;
    colouring.shade = true;
    scene.flow_surface_colouring = colouring;
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, comparable_options());
    ASSERT_EQ(gl.size(), kPixels * 4u);
    scene.flow_surface_colouring->shade = false;
    const std::vector<uint8_t> flat = cpu_rgba(scene, camera, comparable_options());
    EXPECT_GT(channel_diff(flat, cpu).differing, kPixels / 50u) << "the shade darkens the quad";
    const ChannelDiff band = channel_diff(cpu, gl);
    EXPECT_LE(band.worst, 1) << renderer_->renderer_name();
    EXPECT_EQ(band.over_two, 0u);
}

// DrawItem::flow_gain on GL as on the CPU (O128, the pressure film's blades): the wall's value times its gain before the map. A quad over a pressure ramp read at
// gain 1 and at gain 2 differs on the CPU, and GL's frame at gain 2 is the CPU's within one level.
TEST_F(GpuGlRenderer, AWallItemsFlowGainMatchesTheCpu) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-2.0f, -2.0f, -1.0f), 0.25f, 16, 16, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowPressure] = 0.4f * p.x;
        return c;
    });
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::pressure;
    colours.colours = spade::render::ColourMapId::cool_warm;
    colours.range_min = -1.0f;
    colours.range_max = 1.0f;
    RenderScene scene;
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-1.5f, -1.5f, 0.0f), glm::vec3(1.5f, -1.5f, 0.0f), glm::vec3(1.5f, 1.5f, 0.0f), glm::vec3(-1.5f, 1.5f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    scene.meshes.push_back(quad);
    scene.materials = {Material{}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)});
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    spade::render::FlowSurfaceColouring colouring{&volume, colours};
    colouring.offset_cells = 1.0f;
    colouring.walls_only = true;
    scene.flow_surface_colouring = colouring;
    scene.statics[0].flow_wall = true;
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    scene.statics[0].flow_gain = 1.0f;
    const std::vector<uint8_t> cpu1 = cpu_rgba(scene, camera, comparable_options());
    scene.statics[0].flow_gain = 2.0f;
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    const std::vector<uint8_t> cpu2 = cpu_rgba(scene, camera, comparable_options());
    ASSERT_EQ(gl.size(), kPixels * 4u);
    EXPECT_GT(channel_diff(cpu1, cpu2).differing, kPixels / 50u) << "the gain changes the colours";
    const ChannelDiff band = channel_diff(cpu2, gl);
    EXPECT_LE(band.worst, 1) << renderer_->renderer_name();
    EXPECT_EQ(band.over_two, 0u);
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

// What create() spends, program by program (Spade Host, 2026-10-21: an 11 s
// first view where the driver's shader cache missed; the lead: measure
// first). Cold is a fresh source_salt, which no cache has seen; warm is the
// same salt again in the same process. Every program is waited for here
// (no background compile), so each one's time is its own. A measurement,
// not a test: set SPADE_GL_CREATE_COST=1, and run it alone.
TEST(GpuGlCreateCost, EachProgramColdAndWarm) {
    if (!env_gate_is_set("SPADE_GL_CREATE_COST")) {
        GTEST_SKIP() << "a measurement: set SPADE_GL_CREATE_COST=1 and run it alone";
    }
    glfwSetErrorCallback(record_glfw_error);
    if (glfwInit() != GLFW_TRUE) GTEST_SKIP() << "GLFW could not initialise. " << g_last_glfw_error;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(kWidth, kHeight, "spade GpuGlCreateCost", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        GTEST_SKIP() << "no OpenGL 4.3 core context on this machine. " << g_last_glfw_error;
    }
    glfwMakeContextCurrent(window);
    using Clock = std::chrono::steady_clock;
    struct Timed {
        std::vector<std::pair<std::string, double>> programs;  // name, ms since the one before
        double total_ms = 0.0;
    };
    const auto timed_create = [](const std::string& salt) {
        Timed t;
        Clock::time_point last = Clock::now();
        const Clock::time_point start = last;
        GlRenderer::CreateProbe probe;
        probe.source_salt = salt;
        probe.on_program = [&](std::string_view name) {
            const Clock::time_point now = Clock::now();
            t.programs.emplace_back(std::string(name), std::chrono::duration<double, std::milli>(now - last).count());
            last = now;
        };
        auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress),
                                       GlRenderer::CreateOptions{}, probe);
        t.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        EXPECT_TRUE(made.has_value()) << (made ? "" : made.error().context);
        return t;
    };
    const std::string salt =
        "cold_" + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
    const Timed cold = timed_create(salt);
    const Timed warm = timed_create(salt);
    const Timed usual = timed_create("");  // the shipped sources, as the disk cache has them from earlier runs
    ASSERT_EQ(cold.programs.size(), warm.programs.size());
    ASSERT_EQ(cold.programs.size(), usual.programs.size());
    ASSERT_EQ(cold.programs.size(), 9u) << "six programs, then the three flow programs";
    std::printf("[ create cost ] per program, ms, cold (salt %s) / warm (the same salt again) / usual (no salt):\n",
                salt.c_str());
    for (size_t i = 0; i < cold.programs.size(); ++i) {
        std::printf("[ create cost ]   %-16s %9.1f / %7.1f / %7.1f\n", cold.programs[i].first.c_str(),
                    cold.programs[i].second, warm.programs[i].second, usual.programs[i].second);
        RecordProperty(cold.programs[i].first + "_cold_ms", static_cast<int>(cold.programs[i].second));
    }
    std::printf("[ create cost ]   create() total   %9.1f / %7.1f / %7.1f\n", cold.total_ms, warm.total_ms,
                usual.total_ms);
    RecordProperty("create_cold_ms", static_cast<int>(cold.total_ms));
    RecordProperty("create_warm_ms", static_cast<int>(warm.total_ms));

    // As a host that asks for the flow programs in the background sees it,
    // cold and then warm: create() returns, then flow_ready() turns true.
    for (const char* run : {"cold", "warm"}) {
        GlRenderer::CreateProbe probe;
        probe.source_salt = salt + "_background";
        const Clock::time_point start = Clock::now();
        auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress),
                                       GlRenderer::CreateOptions{.flow_programs_in_background = true}, probe);
        const double create_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        ASSERT_TRUE(made.has_value()) << made.error().context;
        const Clock::time_point deadline = start + std::chrono::seconds(120);
        while (!(*made)->flow_ready() && Clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const double ready_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        std::printf("[ create cost ]   in the background, %s: create() %.1f ms, flow ready at %.1f ms\n", run,
                    create_ms, ready_ms);
        RecordProperty(std::string("create_background_") + run + "_ms", static_cast<int>(create_ms));
        RecordProperty(std::string("flow_ready_background_") + run + "_ms", static_cast<int>(ready_ms));
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

// STYLED STREAMLINES (FlowLineStyle; the lead, 2026-11-10): the same five helices as capsules 3 px wide with a
// 4 px glow at 0.4, on both paths. GL's flow-line program computes raster_cpu's draw_styled_flow_lines() per
// fragment (the distance to the segment from the pixel's centre, the coverage, the glow, the depth at the nearest
// point), so the two differ by float32 against the CPU's mixed float and double, and by the one-pixel boundary
// cases. The measure is the whole frame against the CPU's (the ground's own band is in it; the same frame without
// the lines is printed beside it). Measured 2026-11-10, 160x120, on an NVIDIA GeForce RTX 3060 Ti (OpenGL 4.3.0,
// driver 572.83), at a3e85246's code: 2107 pixels styled; worst 3 levels, one pixel over two, 1014 pixels differ
// (the ground alone: worst 1, 608 differ), with one 8 px and one 1 px helix among the 3 px ones. Pinned at
// measured + 1.
TEST_F(GpuGlRenderer, StyledFlowLinesMatchTheCpuWithinTheirBand) {
    RenderScene scene = make_ground_scene();
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
    scene.flow_line_style = spade::render::FlowLineStyle{3.0f, 4.0f, 0.4f};
    // A mixed-width scene (the lead, 2026-11-10, L49): the second helix is 8 px wide of its own, the others inherit
    // the scene's 3, and the fourth is 1 px: the one capsule path, three widths.
    scene.flow_polylines[1].width_px = 8.0f;
    scene.flow_polylines[3].width_px = 1.0f;
    RenderScene bare = scene;
    bare.flow_polylines.clear();
    const Camera camera = camera_over_ground();

    const std::vector<uint8_t> cpu_on = cpu_rgba(scene, camera, comparable_options());
    const std::vector<uint8_t> cpu_off = cpu_rgba(bare, camera, comparable_options());
    const std::vector<uint8_t> gl_on = draw_and_read(scene, comparable_options(), camera);
    const std::vector<uint8_t> gl_off = draw_and_read(bare, comparable_options(), camera);
    ASSERT_EQ(gl_on.size(), kPixels * 4u) << "GL must draw styled streamlines";
    ASSERT_EQ(gl_off.size(), kPixels * 4u);
    const size_t styled = count_differing(cpu_on, cpu_off);
    ASSERT_GT(styled, 1000u) << "the capsules and their halos must reach the frame";
    ASSERT_GT(count_differing(gl_on, gl_off), 1000u) << "and GL's";

    const ChannelDiff base = channel_diff(cpu_off, gl_off);
    const ChannelDiff d = channel_diff(cpu_on, gl_on);
    RecordProperty("styled_pixels", static_cast<int>(styled));
    RecordProperty("worst_levels", d.worst);
    RecordProperty("differing_pixels", static_cast<int>(d.differing));
    RecordProperty("ground_worst_levels", base.worst);
    RecordProperty("ground_differing_pixels", static_cast<int>(base.differing));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ styled lines ] %zu px styled; worst %d levels (the ground alone %d), %zu over two; %zu pixels differ "
                "(the ground alone %zu); on %s\n",
                styled, d.worst, base.worst, d.over_two, d.differing, base.differing, device.c_str());
    constexpr int kBandWorstLevels = 4;           // measured 3
    constexpr size_t kBandOverTwoPixels = 2;      // measured 1
    constexpr size_t kBandDifferingPixels = 1015;  // measured 1014
    EXPECT_LE(d.worst, kBandWorstLevels) << device;
    EXPECT_LE(d.over_two, kBandOverTwoPixels) << device;
    EXPECT_LE(d.differing, kBandDifferingPixels) << device;
}

// RND-11 (L47, the lead 2026-11-10): see-through draw items. Three overlapping discs facing the camera, at alpha
// 0.25, 0.4 and 0.6, over the shadow scene's lambert ground and its shadow, listed in both orders. A see-through item
// is drawn after the opaques, farther first, blended src-over with its material's alpha, and the two backends draw
// the same frame to a band: the interior is 1 level for the shading plus one more for each blend layer, the
// coverage edges as the existing bands. Measured 2026-11-10 on an NVIDIA GeForce RTX 3060 Ti (OpenGL 4.3.0, driver
// 572.83), 160x120: worst 1 level (the same discs opaque: 1), 0 pixels over two, 290 pixels differ (opaque 99).
// Pinned at measured + 1.
[[nodiscard]] MeshData make_disc_mesh(float radius, int segments) {
    MeshData disc;
    disc.positions.push_back(glm::vec3(0.0f));
    disc.normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
    for (int i = 0; i < segments; ++i) {
        const float a = 6.2831853f * static_cast<float>(i) / static_cast<float>(segments);
        disc.positions.push_back(glm::vec3(radius * std::cos(a), radius * std::sin(a), 0.0f));
        disc.normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
    }
    for (int i = 0; i < segments; ++i) {
        disc.indices.insert(disc.indices.end(), {0u, 1u + static_cast<uint32_t>(i),
                                                 1u + static_cast<uint32_t>((i + 1) % segments)});
    }
    disc.submesh_first_index = {0};
    disc.submesh_index_count = {static_cast<uint32_t>(disc.indices.size())};
    disc.submesh_material = {0};
    return disc;
}

TEST_F(GpuGlRenderer, SeeThroughDiscsMatchTheCpuWithinTheirBandInEitherListingOrder) {
    const Camera camera = camera_over_shadow();
    const glm::vec3 forward = camera.orientation * glm::vec3(0.0f, 0.0f, -1.0f);
    const glm::vec3 right = camera.orientation * glm::vec3(1.0f, 0.0f, 0.0f);
    const float alphas[3] = {0.25f, 0.4f, 0.6f};
    const glm::vec3 colours[3] = {glm::vec3(1.0f, 0.3f, 0.2f), glm::vec3(0.2f, 0.9f, 0.3f), glm::vec3(0.3f, 0.4f, 1.0f)};
    const float distance[3] = {3.0f, 3.5f, 4.0f};
    const float lateral[3] = {-0.5f, 0.0f, 0.5f};
    const auto build = [&](bool reversed, bool flagged) {
        RenderScene scene = make_shadow_scene(false);
        const auto disc_mesh = static_cast<uint32_t>(scene.meshes.size());
        scene.meshes.push_back(make_disc_mesh(0.8f, 24));
        const auto first_material = static_cast<uint32_t>(scene.materials.size());
        for (int k = 0; k < 3; ++k) scene.materials.push_back(Material{.base_color = glm::vec4(colours[k], alphas[k])});
        const int order[2][3] = {{0, 1, 2}, {2, 1, 0}};
        for (const int k : order[reversed ? 1 : 0]) {
            const glm::vec3 at = camera.position + forward * distance[k] + right * lateral[k];
            scene.statics.push_back(DrawItem{
                .mesh_index = disc_mesh,
                .local_to_world = glm::translate(glm::mat4(1.0f), at) * glm::mat4_cast(camera.orientation),
                .material_override = first_material + static_cast<uint32_t>(k),
                .see_through = flagged});
        }
        return scene;
    };
    RenderOptions options = comparable_options();
    options.shadows = true;
    const RenderScene flagged = build(false, true), reversed = build(true, true), opaque = build(false, false);

    const std::vector<uint8_t> cpu_flagged = cpu_rgba(flagged, camera, options);
    const std::vector<uint8_t> cpu_reversed = cpu_rgba(reversed, camera, options);
    const std::vector<uint8_t> cpu_opaque = cpu_rgba(opaque, camera, options);
    const std::vector<uint8_t> gl_flagged = draw_and_read(flagged, options, camera);
    const std::vector<uint8_t> gl_reversed = draw_and_read(reversed, options, camera);
    const std::vector<uint8_t> gl_opaque = draw_and_read(opaque, options, camera);
    ASSERT_EQ(gl_flagged.size(), kPixels * 4u);

    // They blend: the same discs unflagged are opaque and the frames differ by hundreds of pixels, on both paths.
    EXPECT_GT(count_differing(cpu_flagged, cpu_opaque), 300u) << "the CPU blends the discs";
    EXPECT_GT(count_differing(gl_flagged, gl_opaque), 300u) << "GL blends the discs";
    // Order independence: the listing order changes nothing, on either path.
    EXPECT_EQ(count_differing(cpu_flagged, cpu_reversed), 0u);
    EXPECT_EQ(count_differing(gl_flagged, gl_reversed), 0u);

    const ChannelDiff base = channel_diff(cpu_opaque, gl_opaque);
    const ChannelDiff d = channel_diff(cpu_flagged, gl_flagged);
    RecordProperty("worst_levels", d.worst);
    RecordProperty("differing_pixels", static_cast<int>(d.differing));
    RecordProperty("opaque_worst_levels", base.worst);
    RecordProperty("opaque_differing_pixels", static_cast<int>(base.differing));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ see-through ] worst %d levels (the same discs opaque %d), %zu over two; %zu pixels differ (opaque %zu); "
                "on %s\n",
                d.worst, base.worst, d.over_two, d.differing, base.differing, device.c_str());
    constexpr int kBandWorstLevels = 2;            // measured 1
    constexpr size_t kBandOverTwoPixels = 1;       // measured 0
    constexpr size_t kBandDifferingPixels = 291;   // measured 290
    EXPECT_LE(d.worst, kBandWorstLevels) << device;
    EXPECT_LE(d.over_two, kBandOverTwoPixels) << device;
    EXPECT_LE(d.differing, kBandDifferingPixels) << device;
}

// A flagged item a CSG subtree marches is refused at upload, by name, as the CPU refuses it.
TEST_F(GpuGlRenderer, RefusesASeeThroughItemACsgSubtreeMarches) {
    RenderScene scene = slab_scene();
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    scene.statics[scene.csg_subtrees[0].draw_item].see_through = true;
    const spade::Result<void> up = renderer_->upload_scene(scene);
    ASSERT_FALSE(up.has_value());
    EXPECT_EQ(up.error().code, spade::Code::invalid_argument);
    EXPECT_NE(up.error().context.find("see-through"), std::string::npos) << up.error().context;
    EXPECT_NE(up.error().context.find("CSG subtree 0"), std::string::npos) << up.error().context;
}

// What 24 see-through discs cost a 1080p frame (the lead's request, L47): opt-in, SPADE_SEE_THROUGH_COST=1, since
// it measures and asserts nothing about the machine. Prop-sized discs (about 0.3 m across at 4 m, a rotor's size) and
// full-screen ones (each covers the frame), against the same frame with none and with the same discs opaque.
TEST_F(GpuGlRenderer, TwentyFourSeeThroughDiscsCostOnA1080pFrameIsMeasured) {
    if (!env_gate_is_set("SPADE_SEE_THROUGH_COST")) GTEST_SKIP() << "opt-in: set SPADE_SEE_THROUGH_COST=1";
    const Camera camera = camera_over_shadow();
    const glm::vec3 forward = camera.orientation * glm::vec3(0.0f, 0.0f, -1.0f);
    const glm::vec3 right = camera.orientation * glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 up = camera.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
    const auto build = [&](float radius, int count, bool flagged) {
        RenderScene scene = make_shadow_scene(false);
        const auto disc_mesh = static_cast<uint32_t>(scene.meshes.size());
        scene.meshes.push_back(make_disc_mesh(radius, 24));
        const auto material = static_cast<uint32_t>(scene.materials.size());
        scene.materials.push_back(Material{.base_color = glm::vec4(0.9f, 0.4f, 0.3f, 0.3f)});
        for (int k = 0; k < count; ++k) {
            const float fk = static_cast<float>(k);
            const glm::vec3 at = camera.position + forward * (3.0f + 0.05f * fk) +
                                 right * (-0.6f + 0.1f * std::fmod(fk, 12.0f)) + up * (-0.3f + 0.2f * std::floor(fk / 12.0f));
            scene.statics.push_back(DrawItem{
                .mesh_index = disc_mesh,
                .local_to_world = glm::translate(glm::mat4(1.0f), at) * glm::mat4_cast(camera.orientation),
                .material_override = material,
                .see_through = flagged});
        }
        return scene;
    };
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    // A real 1080p target: draw() fills the bound framebuffer, and the fixture's own is 160 x 120.
    GLuint fbo = 0, colour = 0, depth = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenRenderbuffers(1, &colour);
    glBindRenderbuffer(GL_RENDERBUFFER, colour);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, 1920, 1080);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 1920, 1080);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
    ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));
    const auto centre_pixel = [&](const RenderScene& scene) {
        EXPECT_TRUE(renderer_->upload_scene(scene).has_value());
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        EXPECT_TRUE(renderer_->draw(scene, camera, comparable_options(), 1920u, 1080u).has_value());
        uint8_t rgba[4] = {0, 0, 0, 0};
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(960, 540, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
        return glm::ivec3(rgba[0], rgba[1], rgba[2]);
    };
    const auto median_ms = [&](const RenderScene& scene) {
        EXPECT_TRUE(renderer_->upload_scene(scene).has_value());
        std::vector<double> ms;
        for (int i = 0; i < 12; ++i) {
            glBindFramebuffer(GL_FRAMEBUFFER, fbo);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glFinish();
            const auto t0 = std::chrono::steady_clock::now();
            const spade::Result<void> drew =
                renderer_->draw(scene, camera, comparable_options(), 1920u, 1080u);
            glFinish();
            const auto t1 = std::chrono::steady_clock::now();
            EXPECT_TRUE(drew.has_value());
            if (i >= 2) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        std::sort(ms.begin(), ms.end());
        return 0.5 * (ms[4] + ms[5]);
    };
    // The discs reach the 1080p frame: the full-screen ones change the centre pixel the bare scene leaves.
    EXPECT_NE(centre_pixel(build(40.0f, 24, true)), centre_pixel(build(40.0f, 0, true)));
    const double none = median_ms(build(0.15f, 0, true));
    const double small_blend = median_ms(build(0.15f, 24, true));
    const double small_opaque = median_ms(build(0.15f, 24, false));
    const double big_blend = median_ms(build(40.0f, 24, true));
    const double big_opaque = median_ms(build(40.0f, 24, false));
    std::printf("[ see-through cost ] 1080p, 24 discs: none %.3f ms; prop-sized %.3f ms blended, %.3f opaque; "
                "full-screen %.3f ms blended, %.3f opaque; on %s\n",
                none, small_blend, small_opaque, big_blend, big_opaque, device.c_str());
    RecordProperty("none_ms_x1000", static_cast<int>(none * 1000.0));
    RecordProperty("small_blend_ms_x1000", static_cast<int>(small_blend * 1000.0));
    RecordProperty("big_blend_ms_x1000", static_cast<int>(big_blend * 1000.0));
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);
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

// create()'s measurement hooks (GpuGlCreateCost): each program reports as it
// links, in create()'s order, the flow programs last; a salt that is not a
// macro name is refused by name. The sources are the fixture's, unsalted, so
// the driver hands them back at once.
TEST_F(GpuGlRenderer, CreateReportsEachProgramAndRefusesABadSalt) {
    const auto loader = reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress);
    std::vector<std::string> names;
    GlRenderer::CreateProbe probe;
    probe.on_program = [&names](std::string_view name) { names.emplace_back(name); };
    const auto made = GlRenderer::create(loader, GlRenderer::CreateOptions{}, probe);
    ASSERT_TRUE(made.has_value()) << made.error().context;
    const std::vector<std::string> want = {"mesh",        "background",  "field",       "overlay",
                                           "flow lines",  "sprites",       "shadow caster", "csg",
                                           "flow volume", "flow bricks",   "flow composite"};
    EXPECT_EQ(names, want);

    GlRenderer::CreateProbe bad;
    bad.source_salt = "not a name";
    const auto refused = GlRenderer::create(loader, GlRenderer::CreateOptions{}, bad);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("source_salt"), std::string::npos) << refused.error().context;
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

// The sensor modes (N13; plans/2026-10-20-sensor-views-design.md) on GL as
// on the CPU: a CSG slab marched, a tilted mesh wall and the analytic ground,
// seen over the horizon. Both draw the same planar depth through the same
// colour table, and the same world normals; they may differ only where the
// two rasterisers' edge rules do.
TEST_F(GpuGlRenderer, SensorFramesMatchTheCpu) {
    RenderScene scene = slab_scene();
    scene.ground_planes.push_back(spade::render::GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = -1.0f});
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.5f, -0.5f, 0.0f), glm::vec3(0.5f, 0.5f, 0.0f),
                      glm::vec3(-0.5f, 0.5f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    spade::render::DrawItem wall;
    wall.mesh_index = static_cast<uint32_t>(scene.meshes.size());
    wall.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(1.4f, -0.3f, 0.5f)) *
                          glm::rotate(glm::mat4(1.0f), -0.6f, glm::vec3(0.0f, 1.0f, 0.0f));
    scene.meshes.push_back(quad);
    scene.statics.push_back(wall);
    const Camera camera = camera_over_ground();

    for (const DrawMode mode : {DrawMode::depth, DrawMode::normals}) {
        SCOPED_TRACE(mode == DrawMode::depth ? "depth" : "normals");
        RenderOptions options = comparable_options();
        options.mode = mode;
        options.depth = spade::render::DepthRange{.near_m = 0.5f, .far_m = 12.0f};
        const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, options);
        const std::vector<uint8_t> gl = draw_and_read(scene, options, camera);
        ASSERT_EQ(cpu.size(), gl.size());
        size_t covered = 0, differ = 0;
        for (size_t p = 0; p < kPixels; ++p) {
            covered += is_black(cpu, p) ? 0u : 1u;
            int worst = 0;
            for (size_t k = 0; k < 3; ++k) worst = std::max(worst, std::abs(int(cpu[p * 4 + k]) - int(gl[p * 4 + k])));
            differ += worst > 6 ? 1u : 0u;
        }
        EXPECT_GT(covered, kPixels / 3u) << "the frame must show the slab, the wall and the ground";
        EXPECT_LE(differ, kPixels / 40u) << differ << " of " << kPixels << " pixels differ by more than 6";
    }
}

// Segmentation on GL as on the CPU, byte for byte inside each surface: a
// labelled CSG slab, a labelled wall, a labelled ground and an unlabelled
// wall, by objects and by parts. Both draw the colour segment_colour() gives
// a name, so only the rasterisers' edge rules may differ.
TEST_F(GpuGlRenderer, SegmentationMatchesTheCpuInsideEachSurface) {
    RenderScene scene = slab_scene();
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    scene.statics[scene.csg_subtrees[0].draw_item].object_label = spade::render::intern_label(scene, "slab");
    scene.ground_planes.push_back(spade::render::GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = -1.0f});
    scene.ground_planes.back().label = spade::render::intern_label(scene, "ground");
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.5f, -0.5f, 0.0f), glm::vec3(0.5f, 0.5f, 0.0f),
                      glm::vec3(-0.5f, 0.5f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    const uint32_t quad_mesh = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(quad);
    for (const float x : {1.4f, -1.6f}) {
        spade::render::DrawItem wall;
        wall.mesh_index = quad_mesh;
        wall.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(x, -0.3f, 0.5f));
        scene.statics.push_back(wall);
    }
    scene.statics[scene.statics.size() - 2].object_label = spade::render::intern_label(scene, "wall");
    scene.statics[scene.statics.size() - 2].part_label = spade::render::intern_label(scene, "wall/face");
    const Camera camera = camera_over_ground();

    for (const spade::render::SegmentBy by : {spade::render::SegmentBy::objects, spade::render::SegmentBy::parts}) {
        SCOPED_TRACE(by == spade::render::SegmentBy::objects ? "objects" : "parts");
        RenderOptions options = comparable_options();
        options.mode = DrawMode::segmentation;
        options.segment_by = by;
        const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, options);
        const std::vector<uint8_t> gl = draw_and_read(scene, options, camera);
        ASSERT_EQ(cpu.size(), gl.size());
        std::set<std::array<uint8_t, 3>> colours;
        size_t differ = 0;
        for (size_t p = 0; p < kPixels; ++p) {
            colours.insert({cpu[p * 4], cpu[p * 4 + 1], cpu[p * 4 + 2]});
            differ += (cpu[p * 4] != gl[p * 4] || cpu[p * 4 + 1] != gl[p * 4 + 1] || cpu[p * 4 + 2] != gl[p * 4 + 2])
                          ? 1u
                          : 0u;
        }
        EXPECT_GE(colours.size(), 5u) << "slab, wall, ground, unlabelled grey and no surface";
        EXPECT_LE(differ, kPixels / 40u) << differ << " of " << kPixels << " pixels differ";
    }
}

// A multisampled target would blend a sensor frame's values at every edge
// into values no surface has, so GL refuses a sensor mode there, by name.
// Shaded draws into the same target.
TEST_F(GpuGlRenderer, RefusesASensorModeOnAMultisampledTarget) {
    GLuint fbo = 0, colour = 0, depth = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenRenderbuffers(1, &colour);
    glBindRenderbuffer(GL_RENDERBUFFER, colour);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_RGBA8, kWidth, kHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT24, kWidth, kHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
    ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));

    const RenderScene scene = slab_scene();
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    for (const DrawMode mode : {DrawMode::depth, DrawMode::normals, DrawMode::segmentation}) {
        RenderOptions options = comparable_options();
        options.mode = mode;
        const spade::Result<void> drew = renderer_->draw(scene, camera_on_plus_z(), options, kWidth, kHeight);
        ASSERT_FALSE(drew.has_value()) << "a sensor mode on a multisampled target must be refused";
        EXPECT_EQ(drew.error().code, spade::Code::invalid_argument);
        EXPECT_NE(drew.error().context.find("multisampled"), std::string::npos) << drew.error().context;
        EXPECT_NE(drew.error().context.find("GlRenderer::draw: " +
                                            std::string(spade::render::sensor_mode_name(mode)) + " draws data"),
                  std::string::npos)
            << "the refusal names its own mode: " << drew.error().context;
    }
    const spade::Result<void> shaded = renderer_->draw(scene, camera_on_plus_z(), comparable_options(), kWidth, kHeight);
    EXPECT_TRUE(shaded.has_value()) << shaded.error().context;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);
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

// What reading a frame back costs a host (Spade Host's question for Kat's
// viewport, 2026-10-20; docs/design/rendering/render-gl-hosting.md): the
// render thread's wall time per frame at 1280x720 for draw() alone, draw()
// plus a synchronous glReadPixels into a BGRX buffer flipped to top row
// first (the CPU target's layout), and draw() plus a ring of two pixel-pack
// buffers, each frame reading into one and copying out the other, a frame
// late. Medians of 40 frames after 5 warm-up frames, on a CSG slab over the
// analytic ground. A measurement, not a test: set SPADE_GL_READBACK_COST=1.
TEST_F(GpuGlRenderer, ReadbackCostSyncAndPboRing) {
    if (!env_gate_is_set("SPADE_GL_READBACK_COST")) {
        GTEST_SKIP() << "a measurement: set SPADE_GL_READBACK_COST=1 to run it";
    }
    constexpr int kW = 1280, kH = 720;
    constexpr size_t kBytes = static_cast<size_t>(kW) * kH * 4u;
    constexpr size_t kRow = static_cast<size_t>(kW) * 4u;
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
    GLuint pbo[2] = {0, 0};
    glGenBuffers(2, pbo);
    for (const GLuint b : pbo) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER, b);
        glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(kBytes), nullptr, GL_STREAM_READ);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    RenderScene scene = slab_scene();
    scene.ground_planes.push_back(spade::render::GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = -1.0f});
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
    RenderOptions options = comparable_options();
    options.shadows = true;
    const Camera camera = camera_over_ground();
    std::vector<uint8_t> bgrx(kBytes);

    // Flips GL's bottom-up rows into the BGRX buffer, top row first.
    const auto flip_into = [&](const uint8_t* src) {
        for (int y = 0; y < kH; ++y) {
            std::memcpy(bgrx.data() + static_cast<size_t>(y) * kRow, src + static_cast<size_t>(kH - 1 - y) * kRow, kRow);
        }
    };
    const auto draw = [&]() {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        EXPECT_TRUE(renderer_->draw(scene, camera, options, kW, kH).has_value());
    };
    // Median ms of 40 frames of `one` after 5 warm-up frames.
    const auto median_ms = [&](const std::function<void(int)>& one) {
        std::vector<double> ms;
        for (int i = 0; i < 45; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            one(i);
            const auto t1 = std::chrono::steady_clock::now();
            if (i >= 5) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        std::sort(ms.begin(), ms.end());
        return 0.5 * (ms[19] + ms[20]);
    };

    std::vector<uint8_t> staging(kBytes);
    const double draw_only = median_ms([&](int) {
        draw();
        glFinish();
    });
    const double sync = median_ms([&](int) {
        draw();
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
        glReadPixels(0, 0, kW, kH, GL_BGRA, GL_UNSIGNED_BYTE, staging.data());
        flip_into(staging.data());
    });
    const double ring = median_ms([&](int i) {
        draw();
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo[i % 2]);
        glReadPixels(0, 0, kW, kH, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);
        if (i > 0) {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo[(i + 1) % 2]);
            const auto* mapped = static_cast<const uint8_t*>(
                glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, static_cast<GLsizeiptr>(kBytes), GL_MAP_READ_BIT));
            if (mapped != nullptr) {
                flip_into(mapped);
                glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
            }
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    });
    EXPECT_EQ(glGetError(), static_cast<GLenum>(GL_NO_ERROR));

    RecordProperty("draw_only_ms_x100", static_cast<int>(draw_only * 100.0));
    RecordProperty("draw_sync_readback_ms_x100", static_cast<int>(sync * 100.0));
    RecordProperty("draw_pbo_ring_ms_x100", static_cast<int>(ring * 100.0));
    std::printf("[ readback cost ] 1280x720, render thread per frame (median of 40): draw alone %.2f ms; draw + sync "
                "glReadPixels to BGRX, flipped %.2f ms; draw + 2-PBO ring, a frame late %.2f ms\n",
                draw_only, sync, ring);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteBuffers(2, pbo);
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



// The flow layer leaves the frame's alpha as the target held it (the tour's
// near-black cores, 2026-10-21). The editor draws its 3D view into an RGBA8
// texture and lays it over the window with alpha blending (ImGui's
// AddImage). The march blended C + T * behind into alpha as well, T (1 +
// behind): 0 wherever an iso was opaque, so the window's dark showed through
// every core, flat. Readbacks of RGB never saw it. A frame cleared opaque,
// the ring's Q iso drawn over it at full and at half scale, stays opaque.
TEST_F(GpuGlRenderer, FlowLayerLeavesTheFramesAlpha) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-1.0f), 2.0f / 64.0f, 64, 64, 64,
                                                                   spade::test::vortex_ring_with_dye());
    spade::render::TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 0.9f;
    spade::render::IsoSurface q_iso;
    q_iso.quantity = spade::render::FlowQuantity::q_hat;
    q_iso.value = 0.5477226f * volume.header.channel_max[spade::kFlowQHat];
    q_iso.colour_by = speed;
    RenderScene scene = ring_cut_scene();
    scene.flow_layers.push_back(spade::render::FlowLayer{&volume, {}, {q_iso}});
    const Camera camera = camera_on_ring();
    ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
    for (const GlRenderer::FlowScale scale : {GlRenderer::FlowScale::full, GlRenderer::FlowScale::half}) {
        GlRenderer::FlowQuality quality;
        quality.scale = scale;
        renderer_->set_flow_quality(quality);
        bind_cleared_target();
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        const auto drew = renderer_->draw(scene, camera, comparable_options(), kWidth, kHeight);
        ASSERT_TRUE(drew.has_value()) << drew.error().context;
        std::vector<uint8_t> rgba(kPixels * 4u);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        size_t lit = 0, see_through = 0;
        for (size_t i = 0; i < kPixels; ++i) {
            if (rgba[i * 4u] != 0u || rgba[i * 4u + 1u] != 0u || rgba[i * 4u + 2u] != 0u) ++lit;
            if (rgba[i * 4u + 3u] != 255u) ++see_through;
        }
        const char* name = scale == GlRenderer::FlowScale::full ? "full" : "half";
        std::printf("[ flow alpha ] %s scale: %zu px drawn, %zu not opaque\n", name, lit, see_through);
        ASSERT_GT(lit, kPixels / 20u) << name << ": the ring must reach the frame";
        EXPECT_EQ(see_through, 0u) << name << " scale: the march changed the frame's alpha";
    }
    renderer_->set_flow_quality(GlRenderer::FlowQuality{});
}

#if defined(SPADE_GL_TEST_HAS_TOUR)
// The tour's near-black vortex cores (Interface, 2026-10-21): the airflow
// chapter's poster frame (armed + 0.9 s), built as AirflowSession builds it
// (Interface's ThePosterFrameMeasured: the drone's parts, shadows and
// overlays off, airflow_view() applied), drawn on GL and the CPU. The CPU
// draws its cores in viridis teal and yellow; GL drew them near-black. A core
// pixel is one the iso layer changes on the CPU; its mean luminance on GL
// must be the CPU's within kPosterCoreLuminance. Variants without the lines
// and without the drone are printed, to say what the GL frame depends on.
// Measured 2026-10-21 (RTX 3060 Ti, 572.83): GL's RGB was the CPU's to 0.1 of
// a level in all four, before the fix too -- the editor's black was the
// frame's alpha (FlowLayerLeavesTheFramesAlpha), which this readback leaves
// out. Kept: the poster frame on both backends, the same.
// Opt-in (SPADE_AIRFLOW_POSTER=1): the flight to the frame is about 40 s.
constexpr double kPosterCoreLuminance = 0.15;  // of the CPU's mean, either way

[[nodiscard]] double luminance(const std::vector<uint8_t>& rgba, size_t pixel) {
    return 0.2126 * rgba[pixel * 4u] + 0.7152 * rgba[pixel * 4u + 1u] + 0.0722 * rgba[pixel * 4u + 2u];
}

TEST_F(GpuGlRenderer, AirflowPosterCoresMatchTheCpu) {
    if (!env_gate_is_set("SPADE_AIRFLOW_POSTER")) {
        GTEST_SKIP() << "the tour's poster frame: set SPADE_AIRFLOW_POSTER=1 to fly it (about 40 s)";
    }
    namespace af = spade::sandbox::airflow;
    namespace an = spade::analysis;
    using spade::render::FlowView;
    using spade::render::ViewSpec;
    const auto quad =
        af::live_quad(std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml");
    ASSERT_TRUE(quad.has_value()) << quad.error().context;
    auto run = an::LiveRun::start(quad->resolved, quad->run_model);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const double poster = af::armed_at(quad->program).value_or(0.0) + 0.9;
    spade::FlowVolume volume;
    while (run->state().t_s < poster) {
        const an::Controls c = an::controls_at(quad->program, static_cast<float>(run->state().t_s));
        const auto f = run->advance_frame(c, volume);
        ASSERT_TRUE(f.has_value()) << f.error().context;
    }
    const float top = af::speed_top(quad->run_model, quad->resolved.spec.fluid.density, 9.80665f);
    const ViewSpec spec = af::airflow_view(volume, af::probes_of(*run), top);
    const auto without = [&spec](FlowView kind) {
        ViewSpec v = spec;
        std::erase_if(v.layers, [kind](const spade::render::FlowViewSpec& l) { return l.kind == kind; });
        return v;
    };
    ASSERT_NE(without(FlowView::vortex_iso).layers.size(), spec.layers.size()) << "no cores at the poster frame";

    const auto prepared = spade::render_parts::prepare_parts(quad->draw_model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    const auto scene_with = [&](bool parts) {
        RenderScene scene;
        if (!parts) return scene;
        spade::render_parts::PartsLook look;
        look.body = static_cast<uint32_t>(scene.materials.size());
        scene.materials.push_back(spade::sandbox::detail::lambert(0.35f, 0.37f, 0.40f));
        look.rotor = static_cast<uint32_t>(scene.materials.size());
        scene.materials.push_back(spade::sandbox::detail::lambert(0.12f, 0.12f, 0.13f));
        look.porous = look.body;
        look.object_name = quad->draw_model.name;
        const auto bound = spade::sandbox::append_vehicle_parts(scene, *prepared, quad->draw_model, run->body(), look);
        EXPECT_TRUE(bound.has_value()) << bound.error().context;
        return scene;
    };
    spade::sandbox::OrbitCamera orbit;
    orbit.target = glm::vec3(0.0f, -0.12f * af::kBoxM, 0.0f);
    orbit.yaw = 0.52f;
    orbit.pitch = 0.35f;
    orbit.distance = 1.5f * af::kBoxM;
    const Camera camera = orbit.to_render_camera();

    // One frame of `view` on both, with the CPU's frame of the same view
    // without its cores, which says which pixels are cores.
    struct Means {
        size_t cores = 0;
        double cpu = 0.0;
        double gl = 0.0;
    };
    const auto measure = [&](const ViewSpec& view, bool parts) {
        RenderScene scene = scene_with(parts);
        RenderScene bare = scene;
        RenderOptions options = comparable_options();
        RenderOptions bare_options = options;
        spade::render::ViewCache cache;
        const auto legends = spade::render::apply_view(view, &volume, scene, options, cache);
        EXPECT_TRUE(legends.has_value()) << legends.error().context;
        ViewSpec no_cores = view;
        std::erase_if(no_cores.layers,
                      [](const spade::render::FlowViewSpec& l) { return l.kind == FlowView::vortex_iso; });
        const auto bare_legends = spade::render::apply_view(no_cores, &volume, bare, bare_options, cache);
        EXPECT_TRUE(bare_legends.has_value()) << bare_legends.error().context;
        const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, options);
        const std::vector<uint8_t> cpu_bare = cpu_rgba(bare, camera, bare_options);
        const std::vector<uint8_t> gl = draw_and_read(scene, options, camera);
        Means m;
        for (size_t i = 0; i < kPixels; ++i) {
            if (std::memcmp(&cpu[i * 4u], &cpu_bare[i * 4u], 3) == 0) continue;
            ++m.cores;
            m.cpu += luminance(cpu, i);
            m.gl += luminance(gl, i);
        }
        if (m.cores > 0u) {
            m.cpu /= static_cast<double>(m.cores);
            m.gl /= static_cast<double>(m.cores);
        }
        return m;
    };
    const Means editor = measure(spec, true);
    const Means no_lines = measure(without(FlowView::streamlines), true);
    const Means no_drone = measure(spec, false);
    const Means alone = measure(without(FlowView::streamlines), false);
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    const auto line = [](const char* what, const Means& m) {
        std::printf("[ poster cores ] %-26s %4zu core px, mean luminance CPU %6.1f, GL %6.1f\n", what, m.cores, m.cpu,
                    m.gl);
    };
    line("as the editor draws it:", editor);
    line("without the lines:", no_lines);
    line("without the drone:", no_drone);
    line("the cores alone:", alone);
    std::printf("[ poster cores ] on %s\n", device.c_str());
    ASSERT_GT(editor.cores, kPixels / 400u) << "the cores show at the poster frame (68 px measured)";
    EXPECT_NEAR(editor.gl, editor.cpu, kPosterCoreLuminance * editor.cpu) << device;
}
#endif

// ---------------------------------------------------------------------------
// The shadow cost and picture record (rendering/soft-shadows, the lead's one table for Cameron). DISABLED_: a
// measuring tool, run on purpose with --gtest_also_run_disabled_tests --gtest_filter=*ShadowRecord*, not a check.
// It times GlRenderer's frame (draw plus glFinish) at 1920x1080 over the four shadow scenes of bench_render.cpp's
// BM_RenderShadows, at four option sets, and writes raw RGBA frames (GL's and the CPU's) of the bench scenes and of a
// mesh drone over a ~170 m world to $SPADE_SHADOW_RECORD_DIR when that is set.
// ---------------------------------------------------------------------------

struct RecordOption {
    const char* name;
    float filter;
    uint32_t focus;
};
constexpr RecordOption kRecordOptions[4] = {
    {"A_today", 0.0f, 0u}, {"B_defaults", 1.0f, 1024u}, {"C_focus512", 1.0f, 512u}, {"D_filter0.5", 0.5f, 1024u}};

[[nodiscard]] MeshData record_mesh(spade::SdfPrim kind, glm::vec4 params, const spade::render::Aabb& bounds) {
    auto made = spade::render::tessellate_primitive(kind, params, bounds, spade::render::kTessellationDefaults);
    if (!made) {
        ADD_FAILURE() << made.error().context;
        return MeshData{};
    }
    return std::move(*made);
}

[[nodiscard]] glm::mat4 record_at(glm::vec3 t, glm::mat4 rotate = glm::mat4(1.0f)) {
    return glm::translate(glm::mat4(1.0f), t) * rotate;
}

// A 172 m world: bounds as VQ1's, the ground mesh over them (it casts, so ~15 cm texels), the analytic ground, a
// static map. variant: 0 none, 1 a 2 m static box, 2 a 0.5 m dynamic box, 3 both (bench_render.cpp's scenes).
[[nodiscard]] RenderScene record_world(int variant) {
    const spade::render::Aabb world{.min = glm::vec3(-80.0f, -1.0f, -10.0f), .max = glm::vec3(92.0f, 30.0f, 10.0f)};
    const spade::render::Aabb small{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    RenderScene scene;
    scene.bounds = world;
    scene.materials = {Material{.base_color = glm::vec4(0.55f, 0.5f, 0.45f, 1.0f)},
                       Material{.base_color = glm::vec4(0.75f, 0.3f, 0.25f, 1.0f)}};
    scene.meshes.push_back(record_mesh(spade::SdfPrim::plane, glm::vec4(0.0f, 1.0f, 0.0f, 0.0f), world));  // 0
    scene.meshes.push_back(record_mesh(spade::SdfPrim::box, glm::vec4(1.0f, 1.0f, 1.0f, 0.0f), small));    // 1
    scene.meshes.push_back(record_mesh(spade::SdfPrim::box, glm::vec4(0.25f, 0.25f, 0.25f, 0.0f), small)); // 2
    scene.ground_planes = {GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = 0.0f, .material = 0}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)});
    if (variant == 1 || variant == 3) {
        scene.statics.push_back(DrawItem{.mesh_index = 1, .local_to_world = record_at({0.0f, 1.0f, 0.0f}), .material_override = 1});
    }
    if (variant == 2 || variant == 3) {
        scene.dynamics.push_back(DrawItem{.mesh_index = 2, .local_to_world = record_at({1.5f, 1.0f, 1.0f}), .material_override = 1});
    }
    auto map = spade::render::build_static_shadow_map(scene);
    if (!map) {
        ADD_FAILURE() << map.error().context;
        return scene;
    }
    scene.static_shadow = std::move(*map);
    return scene;
}

// The same world with a mesh drone 1.2 m over the ground: a body, four arms, four motors, four rotor discs, a camera
// pod and four feet, every part its own dynamic draw item of the tessellated primitives, ~30 cm across.
[[nodiscard]] RenderScene record_drone_world() {
    RenderScene scene = record_world(0);
    const spade::render::Aabb small{.min = glm::vec3(-1.0f), .max = glm::vec3(1.0f)};
    scene.materials.push_back(Material{.base_color = glm::vec4(0.12f, 0.12f, 0.14f, 1.0f)});  // 2: body, motors, rotors
    scene.materials.push_back(Material{.base_color = glm::vec4(0.8f, 0.8f, 0.82f, 1.0f)});    // 3: arms, feet
    const auto box = [&](glm::vec3 half) {
        scene.meshes.push_back(record_mesh(spade::SdfPrim::box, glm::vec4(half, 0.0f), small));
        return static_cast<uint32_t>(scene.meshes.size() - 1u);
    };
    const auto cylinder = [&](float radius, float half_height) {
        scene.meshes.push_back(record_mesh(spade::SdfPrim::cylinder, glm::vec4(radius, half_height, 0.0f, 0.0f), small));
        return static_cast<uint32_t>(scene.meshes.size() - 1u);
    };
    const glm::vec3 at(0.0f, 1.2f, 0.0f);
    const auto part = [&](uint32_t mesh, glm::vec3 offset, uint32_t material) {
        scene.dynamics.push_back(DrawItem{.mesh_index = mesh, .local_to_world = record_at(at + offset), .material_override = material});
    };
    const uint32_t body = box({0.06f, 0.02f, 0.06f});
    const uint32_t arm_x = box({0.14f, 0.008f, 0.012f});
    const uint32_t arm_z = box({0.012f, 0.008f, 0.14f});
    const uint32_t motor = cylinder(0.02f, 0.015f);
    const uint32_t rotor = cylinder(0.09f, 0.002f);
    const uint32_t pod = box({0.02f, 0.015f, 0.02f});
    const uint32_t foot = box({0.01f, 0.03f, 0.01f});
    part(body, {0.0f, 0.0f, 0.0f}, 2);
    part(arm_x, {0.0f, 0.0f, 0.0f}, 3);
    part(arm_z, {0.0f, 0.0f, 0.0f}, 3);
    for (const glm::vec3 m : {glm::vec3(0.14f, 0.02f, 0.0f), glm::vec3(-0.14f, 0.02f, 0.0f), glm::vec3(0.0f, 0.02f, 0.14f), glm::vec3(0.0f, 0.02f, -0.14f)}) {
        part(motor, m, 2);
        part(rotor, m + glm::vec3(0.0f, 0.03f, 0.0f), 2);
    }
    part(pod, {0.0f, -0.0f, 0.075f}, 2);
    for (const glm::vec3 f : {glm::vec3(0.05f, -0.05f, 0.05f), glm::vec3(-0.05f, -0.05f, 0.05f), glm::vec3(0.05f, -0.05f, -0.05f), glm::vec3(-0.05f, -0.05f, -0.05f)}) {
        part(foot, f, 3);
    }
    return scene;
}

[[nodiscard]] Camera record_camera(glm::vec3 from, glm::vec3 to) {
    Camera camera;
    camera.position = from;
    camera.orientation = glm::quatLookAt(glm::normalize(to - from), glm::vec3(0.0f, 1.0f, 0.0f));
    return camera;
}

// raster_cpu's 1080p frame in GL's layout (RGBA8, bottom row first).
[[nodiscard]] std::vector<uint8_t> record_cpu_frame(const RenderScene& scene, const Camera& camera,
                                                    const RenderOptions& options, uint32_t w, uint32_t h) {
    std::vector<uint8_t> bgrx(static_cast<size_t>(w) * h * 4u);
    RenderTarget target{.pixels = std::span<uint8_t>(bgrx), .width = w, .height = h, .stride = w * 4u};
    if (auto drew = spade::render::render(scene, camera, options, target); !drew) {
        ADD_FAILURE() << drew.error().context;
        return {};
    }
    std::vector<uint8_t> rgba(bgrx.size());
    for (size_t y = 0; y < h; ++y) {
        for (size_t x = 0; x < w; ++x) {
            const size_t src = (y * w + x) * 4u;
            const size_t dst = ((h - 1u - y) * w + x) * 4u;
            rgba[dst] = bgrx[src + 2u];
            rgba[dst + 1u] = bgrx[src + 1u];
            rgba[dst + 2u] = bgrx[src];
            rgba[dst + 3u] = 255u;
        }
    }
    return rgba;
}

TEST_F(GpuGlRenderer, DISABLED_ShadowRecord) {
    constexpr uint32_t kW = 1920, kH = 1080;
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

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* record_dir_env = std::getenv("SPADE_SHADOW_RECORD_DIR");
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    const std::string record_dir = record_dir_env != nullptr ? record_dir_env : "";
    const auto write_frame = [&](const std::string& name, const std::vector<uint8_t>& rgba) {
        if (record_dir.empty() || rgba.empty()) return;
        std::ofstream(record_dir + "/" + name + ".rgba", std::ios::binary)
            .write(reinterpret_cast<const char*>(rgba.data()), static_cast<std::streamsize>(rgba.size()));
    };
    const auto options_for = [](const RecordOption& o) {
        RenderOptions options = comparable_options();
        options.shadows = true;
        options.shadow_filter_texels = o.filter;
        options.dynamic_shadow_size = o.focus;
        return options;
    };
    const auto gl_frame = [&](const RenderScene& scene, const Camera& camera, const RenderOptions& options) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (auto drew = renderer_->draw(scene, camera, options, kW, kH); !drew) {
            ADD_FAILURE() << drew.error().context;
            return std::vector<uint8_t>{};
        }
        std::vector<uint8_t> rgba(static_cast<size_t>(kW) * kH * 4u);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, kW, kH, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        return rgba;
    };

    // --- GL frame times: draw + glFinish, the median of 25 after 5 warm-ups -------------------------------------------
    static const char* const kScene[4] = {"no_edges", "static_box", "dynamic_box", "both"};
    const Camera bench_camera = record_camera({5.0f, 2.5f, 7.0f}, {0.0f, 0.8f, 0.0f});
    const Camera close_camera = record_camera({2.6f, 1.0f, 2.8f}, {1.0f, 0.0f, 0.3f});
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ shadow record ] GL frame times, 1920x1080, draw + glFinish, median of 25, on %s\n", device.c_str());
    for (int variant = 0; variant < 4; ++variant) {
        const RenderScene scene = record_world(variant);
        ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
        for (const RecordOption& o : kRecordOptions) {
            const RenderOptions options = options_for(o);
            std::vector<double> ms;
            for (int i = 0; i < 30; ++i) {
                glBindFramebuffer(GL_FRAMEBUFFER, fbo);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                const auto t0 = std::chrono::steady_clock::now();
                ASSERT_TRUE(renderer_->draw(scene, bench_camera, options, kW, kH).has_value());
                glFinish();
                const auto t1 = std::chrono::steady_clock::now();
                if (i >= 5) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            }
            std::sort(ms.begin(), ms.end());
            std::printf("[ shadow record ] GL %-12s %-12s %7.3f ms (min %7.3f, max %7.3f)\n", kScene[variant], o.name,
                        ms[ms.size() / 2], ms.front(), ms.back());
        }
    }

    // --- frames: the bench scenes' shadow edges (GL), and the mesh drone (GL and CPU) -----------------------------------
    for (const int variant : {1, 2}) {
        const RenderScene scene = record_world(variant);
        ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
        for (const RecordOption& o : kRecordOptions) {
            write_frame(std::string("scene") + kScene[variant] + "_wide_gl_" + o.name, gl_frame(scene, bench_camera, options_for(o)));
            write_frame(std::string("scene") + kScene[variant] + "_close_gl_" + o.name, gl_frame(scene, close_camera, options_for(o)));
        }
    }
    {
        const RenderScene scene = record_drone_world();
        ASSERT_TRUE(renderer_->upload_scene(scene).has_value());
        const Camera drone_camera = record_camera({0.9f, 1.15f, 1.1f}, {-0.1f, 0.55f, 0.05f});
        const Camera top_camera = record_camera({0.25f, 1.9f, 0.55f}, {0.0f, 0.9f, 0.0f});
        for (const RecordOption& o : kRecordOptions) {
            const RenderOptions options = options_for(o);
            write_frame(std::string("drone_gl_") + o.name, gl_frame(scene, drone_camera, options));
            write_frame(std::string("drone_cpu_") + o.name, record_cpu_frame(scene, drone_camera, options, kW, kH));
            write_frame(std::string("dronetop_gl_") + o.name, gl_frame(scene, top_camera, options));
            write_frame(std::string("dronetop_cpu_") + o.name, record_cpu_frame(scene, top_camera, options, kW, kH));
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);
}

// PER-POINT ALPHA AND ADDITIVE LIGHT on the capsule path (the lead, 2026-11-17; Cameron: "particle-y", "no dark strands"). GL's
// flow-line program takes an alpha at each end of a segment (FlowPolyline::alphas, interpolated along it) and, with
// FlowLineStyle::additive, adds the capsules as light: out = behind + colour a, depth-tested, no depth write. The CPU renderer
// refuses additive (test_render_flow_line_style.cpp; it draws alphas since D182 S1, test_render_polyline_s1.cpp), so these are not a parity band: each asserts the stated formula on a black
// frame, where a pixel's level is the light's. 160 x 120, the camera 5 m back on +Z, capsules in the z = 0 plane.
[[nodiscard]] RenderScene black_frame_scene() {
    RenderScene scene = make_scene(make_single_triangle(false));
    scene.statics.clear();
    return scene;
}

[[nodiscard]] int level_at(const std::vector<uint8_t>& rgba, int x, int y_from_bottom) {
    return rgba[(static_cast<size_t>(y_from_bottom) * kWidth + static_cast<size_t>(x)) * 4u];
}

[[nodiscard]] spade::render::FlowPolyline light_line(glm::vec3 a, glm::vec3 b, float grey, float alpha0 = -1.0f, float alpha1 = -1.0f) {
    spade::render::FlowPolyline line{{a, b}, {glm::vec3(grey), glm::vec3(grey)}};
    if (alpha0 >= 0.0f) line.alphas = {alpha0, alpha1};
    return line;
}

TEST_F(GpuGlRenderer, AdditiveCapsulesAddTheirLightWhereTheyCrossAndOverBlendDoesNot) {
    RenderScene scene = black_frame_scene();
    scene.flow_polylines.push_back(light_line({-1.5f, 0.0f, 0.0f}, {1.5f, 0.0f, 0.0f}, 0.4f));
    scene.flow_polylines.push_back(light_line({0.0f, -1.5f, 0.0f}, {0.0f, 1.5f, 0.0f}, 0.4f));
    scene.flow_line_style = spade::render::FlowLineStyle{6.0f, 0.0f, 0.35f};
    const int one = 102;  // 0.4 as a byte
    {
        scene.flow_line_style.additive = true;
        const std::vector<uint8_t> frame = draw_and_read(scene);
        ASSERT_EQ(frame.size(), kPixels * 4u);
        EXPECT_NEAR(level_at(frame, 95, 60), one, 4) << "one capsule: its own light";
        EXPECT_NEAR(level_at(frame, 80, 75), one, 4);
        EXPECT_NEAR(level_at(frame, 80, 60), 2 * one, 6) << "two capsules: the sum";
    }
    {
        scene.flow_line_style.additive = false;
        const std::vector<uint8_t> frame = draw_and_read(scene);
        ASSERT_EQ(frame.size(), kPixels * 4u);
        EXPECT_NEAR(level_at(frame, 80, 60), one, 4) << "over-blend: the nearer capsule covers the other, no sum";
    }
}

TEST_F(GpuGlRenderer, AlphasFadeACapsuleFromItsTailToItsHeadByTransparency) {
    RenderScene scene = black_frame_scene();
    scene.flow_polylines.push_back(light_line({-1.5f, 0.0f, 0.0f}, {1.5f, 0.0f, 0.0f}, 1.0f, 0.0f, 1.0f));
    scene.flow_line_style = spade::render::FlowLineStyle{6.0f, 0.0f, 0.35f};
    const std::vector<uint8_t> frame = draw_and_read(scene);
    ASSERT_EQ(frame.size(), kPixels * 4u);
    int lo = -1, hi = -1;
    for (int x = 0; x < kWidth; ++x) {
        if (level_at(frame, x, 60) > 8) {
            if (lo < 0) lo = x;
            hi = x;
        }
    }
    ASSERT_GT(hi - lo, 20) << "the capsule reaches the frame";
    // Never dimmer toward the head, up to the round cap: the last 4 columns are the cap's own edge (3 px radius), where the
    // coverage falls to nothing whatever the alpha is.
    for (int x = lo + 1; x <= hi - 4; ++x) {
        EXPECT_GE(level_at(frame, x, 60) + 2, level_at(frame, x - 1, 60)) << "never dimmer toward the head: column " << x;
    }
    EXPECT_NEAR(level_at(frame, (lo + hi) / 2, 60), 128, 28) << "half-way along, about half";
    EXPECT_GE(level_at(frame, hi - 2, 60), 200) << "the head is the full colour";
    EXPECT_LE(level_at(frame, lo + 2, 60), 60) << "the tail is nearly gone, not black-on-black: there is nothing behind it to hide in";
}

TEST_F(GpuGlRenderer, AdditiveLightBehindASurfaceIsHiddenAndAnAlphaScalesIt) {
    RenderScene scene = make_scene(make_single_triangle(false));  // z = 1: nearer the camera than the capsule at z = 0
    scene.materials[0].base_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);
    scene.flow_polylines.push_back(light_line({-1.5f, 0.0f, 0.0f}, {1.5f, 0.0f, 0.0f}, 0.4f, 1.0f, 1.0f));
    scene.flow_line_style = spade::render::FlowLineStyle{6.0f, 0.0f, 0.35f};
    scene.flow_line_style.additive = true;
    const std::vector<uint8_t> frame = draw_and_read(scene);
    ASSERT_EQ(frame.size(), kPixels * 4u);
    EXPECT_NEAR(level_at(frame, 70, 60), 102, 4) << "left of the triangle: the capsule's light on black";
    EXPECT_NEAR(level_at(frame, 90, 60), 128, 4) << "behind the triangle: the triangle alone, the light does not show through";
}

TEST_F(GpuGlRenderer, AnAlphaScalesAnAdditiveCapsulesLight) {
    RenderScene scene = black_frame_scene();
    scene.flow_polylines.push_back(light_line({-1.5f, 0.0f, 0.0f}, {1.5f, 0.0f, 0.0f}, 0.8f, 0.5f, 0.5f));
    scene.flow_line_style = spade::render::FlowLineStyle{6.0f, 0.0f, 0.35f};
    scene.flow_line_style.additive = true;
    const std::vector<uint8_t> frame = draw_and_read(scene);
    ASSERT_EQ(frame.size(), kPixels * 4u);
    EXPECT_NEAR(level_at(frame, 80, 60), 102, 4) << "0.8 at alpha 0.5 is 0.4";
}

// D182 S1 (KAT's drawing spec, P1; the lead approved the header, 2026-11-18) on GL: a polyline's shading (gradient, flat_mean), caps (flat), depth_test, aa
// and slot mask, and RenderOptions' slot and ui_scale, with the CPU's numbers (test_render_polyline_s1.cpp). Each asserts the stated rule on a black frame
// (160 x 120, the camera 5 m back on +Z, capsules in the z = 0 plane), then one frame holds every look and is held to the CPU's.
[[nodiscard]] int channel_at(const std::vector<uint8_t>& rgba, int x, int y_from_bottom, int channel) {
    return rgba[(static_cast<size_t>(y_from_bottom) * kWidth + static_cast<size_t>(x)) * 4u + static_cast<size_t>(channel)];
}

// The first and last column of row 60 whose red or blue level is over 8.
[[nodiscard]] std::pair<int, int> lit_columns(const std::vector<uint8_t>& rgba, int row = 60) {
    int lo = -1, hi = -1;
    for (int x = 0; x < static_cast<int>(kWidth); ++x) {
        if (channel_at(rgba, x, row, 0) > 8 || channel_at(rgba, x, row, 2) > 8) {
            if (lo < 0) lo = x;
            hi = x;
        }
    }
    return {lo, hi};
}

[[nodiscard]] spade::render::FlowPolyline s1_line(glm::vec3 c0, glm::vec3 c1, float z = 0.0f) {
    return spade::render::FlowPolyline{{glm::vec3(-1.5f, 0.0f, z), glm::vec3(1.5f, 0.0f, z)}, {c0, c1}};
}

[[nodiscard]] RenderScene s1_scene(spade::render::FlowPolyline line, float width = 6.0f) {
    RenderScene scene = black_frame_scene();
    scene.flow_polylines.push_back(std::move(line));
    scene.flow_line_style = spade::render::FlowLineStyle{width, 0.0f, 0.35f};
    return scene;
}

TEST_F(GpuGlRenderer, AGradientRunsFromOneEndsColourToTheOthersAndFlatMeanIsOneWholeByteMean) {
    spade::render::FlowPolyline line = s1_line(glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    line.shading = spade::render::LineShading::gradient;
    const std::vector<uint8_t> grad = draw_and_read(s1_scene(line));
    ASSERT_EQ(grad.size(), kPixels * 4u);
    const auto [lo, hi] = lit_columns(grad);
    ASSERT_GT(hi - lo, 30) << "the capsule reaches the frame";
    const int mid = (lo + hi) / 2;
    EXPECT_NEAR(channel_at(grad, mid, 60, 0), 128, 24) << "half-way: half red";
    EXPECT_NEAR(channel_at(grad, mid, 60, 2), 128, 24) << "and half blue";
    EXPECT_GT(channel_at(grad, lo + 4, 60, 0), channel_at(grad, lo + 4, 60, 2) + 80) << "near the first end: red";
    EXPECT_GT(channel_at(grad, hi - 4, 60, 2), channel_at(grad, hi - 4, 60, 0) + 80) << "near the last: blue";
    // as_before is the mean of the ends all along; flat_mean the same in whole 8-bit channels, (255 + 0 + 1) / 2 = 128
    line.shading = spade::render::LineShading::as_before;
    const std::vector<uint8_t> before = draw_and_read(s1_scene(line));
    EXPECT_EQ(channel_at(before, lo + 4, 60, 0), channel_at(before, hi - 4, 60, 0)) << "the same all along the segment";
    line.shading = spade::render::LineShading::flat_mean;
    const std::vector<uint8_t> flat = draw_and_read(s1_scene(line));
    for (const int x : {lo + 4, mid, hi - 4}) {
        EXPECT_NEAR(channel_at(flat, x, 60, 0), 128, 1) << "column " << x;
        EXPECT_NEAR(channel_at(flat, x, 60, 2), 128, 1) << "column " << x;
        EXPECT_EQ(channel_at(flat, x, 60, 1), 0);
    }
}

TEST_F(GpuGlRenderer, AFlatCapEndsSquareAtTheEndPointAndARoundOneReachesHalfAWidthPast) {
    spade::render::FlowPolyline round = s1_line(glm::vec3(1.0f), glm::vec3(1.0f));
    spade::render::FlowPolyline flat = round;
    flat.cap = spade::render::LineCap::flat;
    const std::vector<uint8_t> r = draw_and_read(s1_scene(round, 10.0f));
    const std::vector<uint8_t> f = draw_and_read(s1_scene(flat, 10.0f));
    const auto [rlo, rhi] = lit_columns(r);
    const auto [flo, fhi] = lit_columns(f);
    ASSERT_GT(rhi - rlo, 30);
    EXPECT_GE(flo - rlo, 4) << "the flat end is half a width (5 px) inside the round one's";
    EXPECT_LE(flo - rlo, 6);
    EXPECT_GE(rhi - fhi, 4);
    EXPECT_LE(rhi - fhi, 6);
    // square: three pixels off the centre line the first lit column is the same as on it, within a pixel; round it curves away
    EXPECT_LE(std::abs(lit_columns(f, 63).first - flo), 1);
    EXPECT_GE(lit_columns(r, 63).first - rlo, 2);
}

TEST_F(GpuGlRenderer, DepthTestFalseDrawsOverANearerSurfaceAndAHardEdgeHasNoPartialCoverage) {
    RenderScene scene = make_scene(make_single_triangle(false));  // z = 1: nearer the camera than the capsule at z = 0
    scene.materials[0].base_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);
    spade::render::FlowPolyline line = light_line({-1.5f, 0.0f, 0.0f}, {1.5f, 0.0f, 0.0f}, 0.4f, 1.0f, 1.0f);
    scene.flow_polylines.push_back(line);
    scene.flow_line_style = spade::render::FlowLineStyle{6.0f, 0.0f, 0.35f};
    EXPECT_NEAR(level_at(draw_and_read(scene), 90, 60), 128, 4) << "tested: behind the triangle, the triangle alone";
    scene.flow_polylines[0].depth_test = false;
    const std::vector<uint8_t> over = draw_and_read(scene);
    EXPECT_NEAR(level_at(over, 90, 60), 102, 4) << "untested: drawn over the triangle";
    EXPECT_NEAR(level_at(over, 70, 60), 102, 4);
    // aa false: every pixel of a black frame is 0 or the colour, nothing between; aa on has partial pixels
    RenderScene hard = black_frame_scene();
    hard.flow_polylines.push_back(light_line({-1.5f, 0.2f, 0.0f}, {1.4f, -0.3f, 0.0f}, 0.4f));
    hard.flow_line_style = spade::render::FlowLineStyle{5.0f, 0.0f, 0.35f};
    const auto partial = [](const std::vector<uint8_t>& rgba) {
        size_t n = 0;
        for (size_t p = 0; p < rgba.size() / 4u; ++p) {
            const int v = rgba[p * 4u];
            if (v > 1 && std::abs(v - 102) > 1) ++n;
        }
        return n;
    };
    EXPECT_GT(partial(draw_and_read(hard)), 10u) << "anti-aliased: partial coverage along the edge";
    hard.flow_polylines[0].aa = false;
    const std::vector<uint8_t> edge = draw_and_read(hard);
    ASSERT_EQ(edge.size(), kPixels * 4u);
    EXPECT_EQ(partial(edge), 0u) << "a hard edge: coverage 1 or nothing";
    EXPECT_GT(count_non_black(edge), 100u);
}

TEST_F(GpuGlRenderer, ASlotMaskAndUiScaleChooseWhatARenderDrawsAndOutOfRangeValuesAreRefused) {
    spade::render::FlowPolyline line = s1_line(glm::vec3(1.0f), glm::vec3(1.0f));
    line.slots = 0b10u;
    RenderOptions slot0 = comparable_options();
    RenderOptions slot1 = comparable_options();
    slot1.slot = 1u;
    EXPECT_EQ(count_non_black(draw_and_read(s1_scene(line), slot0)), 0u) << "slot 0 is not in the mask";
    EXPECT_GT(count_non_black(draw_and_read(s1_scene(line), slot1)), 100u);
    spade::render::FlowPolyline thin = line;  // the one-pixel line has a mask too
    RenderScene plain = black_frame_scene();
    plain.flow_polylines.push_back(thin);
    EXPECT_EQ(count_non_black(draw_and_read(plain, slot0)), 0u);
    EXPECT_GT(count_non_black(draw_and_read(plain, slot1)), 10u);
    // ui_scale: width 3 at 2 is exactly width 6 at 1, the glow's pixels too
    spade::render::FlowPolyline all = s1_line(glm::vec3(1.0f), glm::vec3(1.0f));
    RenderScene narrow = s1_scene(all, 3.0f);
    narrow.flow_line_style.glow_px = 2.0f;
    RenderScene wide = s1_scene(all, 6.0f);
    wide.flow_line_style.glow_px = 4.0f;
    RenderOptions doubled = comparable_options();
    doubled.ui_scale = 2.0f;
    EXPECT_EQ(count_differing(draw_and_read(narrow, doubled), draw_and_read(wide)), 0u);
    // refusals, by name, before any pixel
    ASSERT_TRUE(renderer_->upload_scene(wide).has_value());
    RenderOptions bad = comparable_options();
    bad.slot = 32u;
    auto drew = renderer_->draw(wide, camera_on_plus_z(), bad, kWidth, kHeight);
    ASSERT_FALSE(drew.has_value());
    EXPECT_NE(drew.error().context.find("slot"), std::string::npos) << drew.error().context;
    bad = comparable_options();
    bad.ui_scale = 9.0f;
    drew = renderer_->draw(wide, camera_on_plus_z(), bad, kWidth, kHeight);
    ASSERT_FALSE(drew.has_value());
    EXPECT_NE(drew.error().context.find("ui_scale"), std::string::npos) << drew.error().context;
    RenderScene big = black_frame_scene();
    spade::render::FlowPolyline huge;
    huge.points.assign(spade::render::kMaxFlowLineVertices + 1u, glm::vec3(0.0f));
    huge.colours.assign(spade::render::kMaxFlowLineVertices + 1u, glm::vec3(1.0f));
    big.flow_polylines.push_back(std::move(huge));
    big.flow_line_style = spade::render::FlowLineStyle{3.0f, 0.0f, 0.35f};
    drew = renderer_->draw(big, camera_on_plus_z(), comparable_options(), kWidth, kHeight);
    ASSERT_FALSE(drew.has_value());
    EXPECT_NE(drew.error().context.find("262144"), std::string::npos) << drew.error().context;
}

// One frame with every look at once, GL against the CPU: a gradient with alphas, a gradient, flat_mean, a flat cap, a hard edge, an untested line over the
// ground, a polyline masked out of the slot, at a ui_scale of 1.5. Full-coverage pixels agree to a few levels, edge pixels to the existing styled-line band.
TEST_F(GpuGlRenderer, EveryS1LookMatchesTheCpuWithinItsBand) {
    RenderScene scene = make_ground_scene();
    const auto helix = [](int k) {
        spade::render::FlowPolyline line;
        for (int i = 0; i <= 40; ++i) {
            const float s = static_cast<float>(i) / 40.0f;
            const float a = 6.0f * s + 1.3f * static_cast<float>(k);
            line.points.push_back(glm::vec3(-2.0f + 4.0f * s, 0.6f + 0.4f * static_cast<float>(k) * 0.3f + 0.5f * std::sin(a),
                                            -1.0f + 0.5f * std::cos(a) + 0.3f * static_cast<float>(k)));
            line.colours.push_back(spade::render::colour_map_at(spade::render::ColourMapId::viridis, s));
        }
        return line;
    };
    for (int k = 0; k < 7; ++k) scene.flow_polylines.push_back(helix(k));
    scene.flow_polylines[0].shading = spade::render::LineShading::gradient;
    scene.flow_polylines[0].alphas.assign(41u, 1.0f);
    for (size_t i = 0; i < 41u; ++i) scene.flow_polylines[0].alphas[i] = 0.15f + 0.85f * static_cast<float>(i) / 40.0f;
    scene.flow_polylines[1].shading = spade::render::LineShading::gradient;
    scene.flow_polylines[2].shading = spade::render::LineShading::flat_mean;
    scene.flow_polylines[3].cap = spade::render::LineCap::flat;
    scene.flow_polylines[4].aa = false;
    scene.flow_polylines[5].depth_test = false;
    scene.flow_polylines[6].slots = 0b10u;  // not in slot 0
    scene.flow_line_style = spade::render::FlowLineStyle{3.0f, 4.0f, 0.4f};
    RenderScene bare = scene;
    bare.flow_polylines.clear();
    RenderOptions options = comparable_options();
    options.ui_scale = 1.5f;
    const Camera camera = camera_over_ground();
    const std::vector<uint8_t> cpu_on = cpu_rgba(scene, camera, options);
    const std::vector<uint8_t> cpu_off = cpu_rgba(bare, camera, options);
    const std::vector<uint8_t> gl_on = draw_and_read(scene, options, camera);
    const std::vector<uint8_t> gl_off = draw_and_read(bare, options, camera);
    ASSERT_EQ(gl_on.size(), kPixels * 4u);
    const size_t styled = count_differing(cpu_on, cpu_off);
    ASSERT_GT(styled, 1000u) << "the looks reach the frame";
    ASSERT_GT(count_differing(gl_on, gl_off), 1000u) << "and GL's";
    const ChannelDiff base = channel_diff(cpu_off, gl_off);
    const ChannelDiff d = channel_diff(cpu_on, gl_on);
    RecordProperty("styled_pixels", static_cast<int>(styled));
    RecordProperty("worst_levels", d.worst);
    RecordProperty("differing_pixels", static_cast<int>(d.differing));
    RecordProperty("over_two_pixels", static_cast<int>(d.over_two));
    RecordProperty("ground_worst_levels", base.worst);
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ S1 looks ] %zu px styled; worst %d levels (the ground alone %d), %zu over two; %zu pixels differ (the ground alone %zu); on %s\n", styled,
                d.worst, base.worst, d.over_two, d.differing, base.differing, device.c_str());
    // Measured 2026-11-19, 160x120, on an NVIDIA GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83), at df325400's code: 2656 pixels styled; worst 6 levels
    // (the ground alone 1), 2 pixels over two levels, 1276 pixels differ (the ground alone 608), with seven looks at once at a ui_scale of 1.5. The
    // one-look tests above are tighter on a black frame (a gradient's mid colour within 24 levels, a flat colour within 1). Which look carries the worst
    // pixel was not attributed: the CPU's mixed float and double and GL's float32 can fall either side of a coverage or the 0.5 threshold. Pinned at
    // measured + 1.
    constexpr int kBandWorstLevels = 7;        // measured 6
    constexpr size_t kBandOverTwoPixels = 3;   // measured 2
    constexpr size_t kBandDifferingPixels = 1277;  // measured 1276
    EXPECT_LE(d.worst, kBandWorstLevels) << device;
    EXPECT_LE(d.over_two, kBandOverTwoPixels) << device;
    EXPECT_LE(d.differing, kBandDifferingPixels) << device;
}

// D182 S3: the sprites, GL against the CPU. The GL renderer projects and rasterises them with the CPU reference's own code (render/sprites.cpp), uploads
// the premultiplied overlay and composites it over its frame, so the sprites agree with the reference up to the ground underneath and one level of
// rounding where a translucent shape covers a pixel. Drone discs, the three marks, the reticle in its three states, a hard-edged and a translucent shape,
// a sprite on another slot and one partly off the frame, at a ui_scale of 1.5.
TEST_F(GpuGlRenderer, SpritesMatchTheCpuReferenceWithinOneLevelOfRounding) {
    using namespace spade::render;
    RenderScene scene = make_ground_scene();
    const SpriteColour fill{0xDC, 0xE5, 0xEC, 255}, border{0x0B, 0x0F, 0x14, 255}, crash{0xE2, 0x6E, 0x73, 255}, lap{0x5B, 0xBB, 0x86, 255},
        limit{0xE8, 0xA3, 0x3D, 255}, dim{0x82, 0x8F, 0x9C, 255}, accent{0x7C, 0x93, 0xFF, 255};
    float x = -2.0f;
    const auto next = [&x]() {
        const glm::dvec3 p(x, 0.6, -0.4);
        x += 0.45f;
        return p;
    };
    scene.sprites.push_back(make_drone_disc(next(), fill, border));
    scene.sprites.push_back(make_mark(MarkKind::crash, next(), crash));
    scene.sprites.push_back(make_mark(MarkKind::lap, next(), lap));
    scene.sprites.push_back(make_mark(MarkKind::time_limit, next(), limit));
    scene.sprites.push_back(make_spawn_reticle(next(), ReticleState::normal, dim, accent, lap));
    scene.sprites.push_back(make_spawn_reticle(next(), ReticleState::hover, accent, accent, lap));
    scene.sprites.push_back(make_spawn_reticle(next(), ReticleState::occupied, dim, accent, lap));
    {
        SpriteShape glass;  // translucent, hard-edged, over the marks
        glass.kind = SpriteShapeKind::circle;
        glass.x0 = -3.0f;
        glass.r = 14.0f;
        glass.fill = SpriteColour{200, 100, 50, 128};
        glass.aa = false;
        WorldSprite sprite;
        sprite.anchor = glm::dvec3(-1.1, 0.6, -0.4);
        sprite.shapes = {glass};
        scene.sprites.push_back(sprite);
    }
    {
        WorldSprite other = make_mark(MarkKind::crash, glm::dvec3(0.0, 1.2, -0.4), crash);
        other.slots = 0b10u;  // not slot 0
        scene.sprites.push_back(other);
        WorldSprite edge = make_mark(MarkKind::lap, glm::dvec3(2.6, 0.6, -0.4), lap);
        scene.sprites.push_back(edge);
    }
    RenderScene bare = scene;
    bare.sprites.clear();
    RenderOptions options = comparable_options();
    options.ui_scale = 1.5f;
    const Camera camera = camera_over_ground();
    const std::vector<uint8_t> cpu_on = cpu_rgba(scene, camera, options);
    const std::vector<uint8_t> cpu_off = cpu_rgba(bare, camera, options);
    const std::vector<uint8_t> gl_on = draw_and_read(scene, options, camera);
    const std::vector<uint8_t> gl_off = draw_and_read(bare, options, camera);
    ASSERT_EQ(gl_on.size(), kPixels * 4u);
    const size_t drawn = count_differing(cpu_on, cpu_off);
    ASSERT_GT(drawn, 400u) << "the sprites reach the frame";
    ASSERT_GT(count_differing(gl_on, gl_off), 400u) << "and GL's";
    const ChannelDiff base = channel_diff(cpu_off, gl_off);
    const ChannelDiff d = channel_diff(cpu_on, gl_on);
    RecordProperty("sprite_pixels", static_cast<int>(drawn));
    RecordProperty("worst_levels", d.worst);
    RecordProperty("differing_pixels", static_cast<int>(d.differing));
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ S3 sprites ] %zu px drawn; worst %d levels (the ground alone %d), %zu over two; %zu pixels differ (the ground alone %zu); on %s\n", drawn,
                d.worst, base.worst, d.over_two, d.differing, base.differing, device.c_str());
    // Measured 2026-11-19, 160x120, on an NVIDIA GeForce RTX 3060 Ti (OpenGL 4.3.0, driver 572.83), the sprites drawn from tiles (Option A): 2635 pixels drawn;
    // worst 2 levels (the ground alone 1), none over two, 1483 pixels differ (the ground alone 608). Before Option A (a frame-sized overlay built by the same
    // rasteriser the CPU frame draws with) it was worst 1 and 1280 differing. The EXTRA level is named: the CPU frame rounds to 8 bits on the frame once for each
    // shape that covers a pixel (fill, then stroke), the tile rounds the same shapes to 8 bits premultiplied on a transparent base and GL's blend then rounds
    // the composite onto the frame, one more rounding than the CPU's; a pixel under both a fill and a stroke can land a level apart. The bound stays what it was
    // (the ground's own worst plus one level, nothing over two); only the count of pixels that differ by a level rose, and is pinned at measured + 1.
    EXPECT_LE(d.worst, base.worst + 1) << device;
    EXPECT_EQ(d.over_two, base.over_two) << device;
    constexpr size_t kBandDifferingPixels = 1484;  // measured 1483
    EXPECT_LE(d.differing, kBandDifferingPixels) << device;
}

TEST_F(GpuGlRenderer, ASpriteThatCannotBeDrawnIsRefusedByName) {
    using namespace spade::render;
    RenderScene scene = make_ground_scene();
    scene.sprites.push_back(WorldSprite{});  // no shapes
    const Camera camera = camera_over_ground();
    const spade::Result<void> refused = renderer_->draw(scene, camera, comparable_options(), kWidth, kHeight);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("sprites[0]"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("shapes"), std::string::npos) << refused.error().context;
}

// Option A: the sprites are drawn from per-sprite tiles with one upload and one instanced draw; nothing of a frame is kept for the next. Three frames on ONE
// renderer: sprites at A, the same sprites moved to B, then none. Each matches the CPU reference within the S3 band, and the sprite-free frame is exactly the
// frame without sprites (no tile left behind).
TEST_F(GpuGlRenderer, SpritesFollowAMovingSceneAndLeaveNothingBehind) {
    using namespace spade::render;
    const SpriteColour fill{0xDC, 0xE5, 0xEC, 255}, border{0x0B, 0x0F, 0x14, 255}, crash{0xE2, 0x6E, 0x73, 255}, dim{0x82, 0x8F, 0x9C, 255},
        accent{0x7C, 0x93, 0xFF, 255}, lap{0x5B, 0xBB, 0x86, 255};
    const auto at = [&](double shift) {
        RenderScene scene = make_ground_scene();
        for (int k = 0; k < 6; ++k) {
            scene.sprites.push_back(make_drone_disc(glm::dvec3(-1.6 + 0.6 * static_cast<double>(k) + shift, 0.5, -0.4), fill, border));
        }
        scene.sprites.push_back(make_mark(MarkKind::crash, glm::dvec3(-0.5 + shift, 0.9, -0.4), crash));
        scene.sprites.push_back(make_spawn_reticle(glm::dvec3(0.3 + shift, 0.2, -0.4), ReticleState::hover, dim, accent, lap));
        return scene;
    };
    RenderScene bare = make_ground_scene();
    RenderOptions options = comparable_options();
    options.ui_scale = 1.25f;
    const Camera camera = camera_over_ground();
    const RenderScene frame_a = at(0.0), frame_b = at(0.37);
    const std::vector<uint8_t> gl_a = draw_and_read(frame_a, options, camera);
    const std::vector<uint8_t> gl_b = draw_and_read(frame_b, options, camera);
    const std::vector<uint8_t> gl_c = draw_and_read(bare, options, camera);
    const std::vector<uint8_t> gl_bare = draw_and_read(bare, options, camera);
    const std::vector<uint8_t> cpu_bare = cpu_rgba(bare, camera, options);
    const ChannelDiff base = channel_diff(cpu_bare, gl_bare);
    for (const auto& pair : {std::pair<const RenderScene*, const std::vector<uint8_t>*>{&frame_a, &gl_a}, std::pair<const RenderScene*, const std::vector<uint8_t>*>{&frame_b, &gl_b}}) {
        const std::vector<uint8_t> cpu = cpu_rgba(*pair.first, camera, options);
        ASSERT_GT(count_differing(cpu, cpu_bare), 300u) << "the sprites reach the frame";
        const ChannelDiff d = channel_diff(cpu, *pair.second);
        EXPECT_LE(d.worst, base.worst + 1);
        EXPECT_EQ(d.over_two, base.over_two);
    }
    EXPECT_EQ(count_differing(gl_c, gl_bare), 0u) << "after frames with sprites, a frame without leaves no sprite behind";
    EXPECT_GT(count_differing(gl_a, gl_b), 100u) << "and the moved sprites are not the old ones";
}

// D182 S3's cost (KAT's host draws a disc per drone, a few marks and a reticle each frame): on GL every frame that has a sprite rasterises it on the CPU into a
// frame-sized premultiplied overlay, uploads it and composites it. This times the whole draw with sprites against the same frame without them, at 1280 x 720
// and 1920 x 1080, with 32 and 128 drone discs plus one reticle and four marks, and the CPU rasterisation alone (rasterize_sprite_overlay). Medians of 30
// frames after two warm-ups, the draw alone (the scene is uploaded first). Opt-in, as the other cost tests: set SPADE_SPRITE_COST=1.
TEST_F(GpuGlRenderer, SpriteCostAt32And128DiscsAt720pAnd1080p) {
    if (!env_gate_is_set("SPADE_SPRITE_COST")) GTEST_SKIP() << "opt-in: set SPADE_SPRITE_COST=1";
    using namespace spade::render;
    const Camera camera = camera_over_ground();
    const SpriteColour fill{0xDC, 0xE5, 0xEC, 255}, border{0x0B, 0x0F, 0x14, 255}, crash{0xE2, 0x6E, 0x73, 255}, lap{0x5B, 0xBB, 0x86, 255},
        limit{0xE8, 0xA3, 0x3D, 255}, dim{0x82, 0x8F, 0x9C, 255}, accent{0x7C, 0x93, 0xFF, 255};
    const auto build = [&](int discs, bool with_sprites) {
        RenderScene scene = make_ground_scene();
        if (!with_sprites) return scene;
        for (int k = 0; k < discs; ++k) {
            WorldSprite disc = make_drone_disc(glm::dvec3(-2.2 + 4.4 * static_cast<double>(k % 16) / 15.0 + 0.14 * static_cast<double>(k / 128), 0.3 + 0.25 * static_cast<double>((k % 128) / 16), -0.4), fill, border);
            disc.visible_max_span_px = 0.0f;  // always drawn: the disc is the cost being measured
            scene.sprites.push_back(std::move(disc));
        }
        scene.sprites.push_back(make_spawn_reticle(glm::dvec3(0.0, 0.2, 0.3), ReticleState::hover, dim, accent, lap));
        scene.sprites.push_back(make_mark(MarkKind::crash, glm::dvec3(-1.0, 0.9, 0.0), crash));
        scene.sprites.push_back(make_mark(MarkKind::lap, glm::dvec3(0.0, 0.9, 0.0), lap));
        scene.sprites.push_back(make_mark(MarkKind::time_limit, glm::dvec3(1.0, 0.9, 0.0), limit));
        scene.sprites.push_back(make_mark(MarkKind::crash, glm::dvec3(2.0, 0.9, 0.0), crash));
        return scene;
    };
    const auto median = [](std::vector<double> ms) {
        std::sort(ms.begin(), ms.end());
        return 0.5 * (ms[ms.size() / 2 - 1] + ms[ms.size() / 2]);
    };
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    for (const auto& size : {std::pair<uint32_t, uint32_t>{1280u, 720u}, std::pair<uint32_t, uint32_t>{1920u, 1080u}}) {
        const uint32_t w = size.first, h = size.second;
        GLuint fbo = 0, colour = 0, depth = 0;
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glGenRenderbuffers(1, &colour);
        glBindRenderbuffer(GL_RENDERBUFFER, colour);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, static_cast<GLsizei>(w), static_cast<GLsizei>(h));
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
        glGenRenderbuffers(1, &depth);
        glBindRenderbuffer(GL_RENDERBUFFER, depth);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, static_cast<GLsizei>(w), static_cast<GLsizei>(h));
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
        ASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER), static_cast<GLenum>(GL_FRAMEBUFFER_COMPLETE));
        const auto gl_ms = [&](const RenderScene& scene) {
            EXPECT_TRUE(renderer_->upload_scene(scene).has_value());
            std::vector<double> ms;
            for (int i = 0; i < 32; ++i) {
                glBindFramebuffer(GL_FRAMEBUFFER, fbo);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glFinish();
                const auto t0 = std::chrono::steady_clock::now();
                const spade::Result<void> drew = renderer_->draw(scene, camera, comparable_options(), w, h);
                glFinish();
                const auto t1 = std::chrono::steady_clock::now();
                EXPECT_TRUE(drew.has_value());
                if (i >= 2) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            }
            return median(ms);
        };
        const auto raster_ms = [&](const RenderScene& scene) {
            SpriteTiles tiles;
            std::vector<double> ms;
            for (int i = 0; i < 32; ++i) {
                const auto t0 = std::chrono::steady_clock::now();
                const size_t drawn = rasterize_sprite_tiles(scene, camera, comparable_options(), w, h, tiles);
                const auto t1 = std::chrono::steady_clock::now();
                EXPECT_GT(drawn, 0u);
                if (i >= 2) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            }
            return median(ms);
        };
        const double base = gl_ms(build(0, false));
        for (const int discs : {32, 128, 256}) {
            const RenderScene scene = build(discs, true);
            const double with = gl_ms(scene);
            const double raster = raster_ms(scene);
            std::printf("[ sprite cost ] %ux%u, %3d discs + a reticle + 4 marks: draw %.3f ms against %.3f ms without sprites (+%.3f ms); of that the CPU rasterisation alone %.3f ms; on %s\n",
                        w, h, discs, with, base, with - base, raster, device.c_str());
            RecordProperty("added_ms_x1000_" + std::to_string(w) + "_" + std::to_string(discs), static_cast<int>((with - base) * 1000.0));
            RecordProperty("raster_ms_x1000_" + std::to_string(w) + "_" + std::to_string(discs), static_cast<int>(raster * 1000.0));
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteFramebuffers(1, &fbo);
        glDeleteRenderbuffers(1, &colour);
        glDeleteRenderbuffers(1, &depth);
    }
}

// D182 S1's cost (KAT's host draws a trail per drone): 128 drones, one polyline each of 64, 200 and 400 points, 3 px wide with a 4 px glow, on a 1280 x 720
// frame, GL and the CPU, the default look (as_before, no alphas: the work the capsule pass did before S1) and every look at once (gradient, per-point
// alpha, flat cap, a hard edge on every fourth, depth_test false on every eighth). Medians of ten frames after two warm-ups, the draw alone (the scene is
// uploaded first); the CPU's is render() into a 1280 x 720 buffer. Opt-in, as the other cost tests: set SPADE_POLYLINE_COST=1.
TEST_F(GpuGlRenderer, PolylineCostAt128DronesAnd64And200And400Points) {
    if (!env_gate_is_set("SPADE_POLYLINE_COST")) GTEST_SKIP() << "opt-in: set SPADE_POLYLINE_COST=1";
    constexpr uint32_t kW = 1280u, kH = 720u;
    const Camera camera = camera_over_ground();
    const auto build = [&](int points, bool looks) {
        RenderScene scene = make_ground_scene();
        for (int k = 0; k < 128; ++k) {
            spade::render::FlowPolyline line;
            const float fx = -2.0f + 0.25f * static_cast<float>(k % 16);
            const float fz = -1.5f + 0.4f * static_cast<float>(k / 16);
            for (int i = 0; i < points; ++i) {
                const float s = static_cast<float>(i) / static_cast<float>(points - 1);
                line.points.push_back(glm::vec3(fx + 0.5f * s, 0.6f + 0.15f * std::sin(9.0f * s + static_cast<float>(k)), fz + 0.1f * std::cos(7.0f * s)));
                line.colours.push_back(spade::render::colour_map_at(spade::render::ColourMapId::viridis, s));
            }
            if (looks) {
                line.shading = spade::render::LineShading::gradient;
                line.alphas.resize(static_cast<size_t>(points));
                for (int i = 0; i < points; ++i) line.alphas[static_cast<size_t>(i)] = 0.08f + 0.92f * static_cast<float>(i) / static_cast<float>(points - 1);
                line.cap = spade::render::LineCap::flat;
                line.aa = (k % 4) != 0;
                line.depth_test = (k % 8) != 0;
            }
            scene.flow_polylines.push_back(std::move(line));
        }
        scene.flow_line_style = spade::render::FlowLineStyle{3.0f, 4.0f, 0.4f};
        return scene;
    };
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
    const auto median = [](std::vector<double> ms) {
        std::sort(ms.begin(), ms.end());
        return 0.5 * (ms[ms.size() / 2 - 1] + ms[ms.size() / 2]);
    };
    const auto gl_ms = [&](const RenderScene& scene) {
        EXPECT_TRUE(renderer_->upload_scene(scene).has_value());
        std::vector<double> ms;
        for (int i = 0; i < 12; ++i) {
            glBindFramebuffer(GL_FRAMEBUFFER, fbo);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glFinish();
            const auto t0 = std::chrono::steady_clock::now();
            const spade::Result<void> drew = renderer_->draw(scene, camera, comparable_options(), kW, kH);
            glFinish();
            const auto t1 = std::chrono::steady_clock::now();
            EXPECT_TRUE(drew.has_value());
            if (i >= 2) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        return median(ms);
    };
    const auto cpu_ms = [&](const RenderScene& scene) {
        std::vector<uint8_t> pixels(static_cast<size_t>(kW) * kH * 4u);
        spade::render::RenderTarget target{.pixels = std::span<uint8_t>(pixels), .width = kW, .height = kH, .stride = kW * 4u};
        std::vector<double> ms;
        for (int i = 0; i < 12; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            const spade::Result<void> r = spade::render::render(scene, camera, comparable_options(), target);
            const auto t1 = std::chrono::steady_clock::now();
            EXPECT_TRUE(r.has_value());
            if (i >= 2) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        return median(ms);
    };
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    const RenderScene ground_only = [&] {
        RenderScene s = build(64, false);
        s.flow_polylines.clear();
        return s;
    }();
    const double gl_base = gl_ms(ground_only);
    const double cpu_base = cpu_ms(ground_only);
    std::printf("[ polyline cost ] 1280x720, 128 drones, 3 px + 4 px glow; the frame without lines: GL %.3f ms, CPU %.2f ms; on %s\n", gl_base, cpu_base, device.c_str());
    for (const int points : {64, 200, 400}) {
        const double gl_default = gl_ms(build(points, false));
        const double gl_looks = gl_ms(build(points, true));
        const double cpu_default = cpu_ms(build(points, false));
        const double cpu_looks = cpu_ms(build(points, true));
        std::printf("[ polyline cost ] %3d points: GL default %.3f ms, every look %.3f ms; CPU default %.2f ms, every look %.2f ms\n", points, gl_default,
                    gl_looks, cpu_default, cpu_looks);
        RecordProperty("gl_default_ms_x1000_" + std::to_string(points), static_cast<int>(gl_default * 1000.0));
        RecordProperty("gl_looks_ms_x1000_" + std::to_string(points), static_cast<int>(gl_looks * 1000.0));
        RecordProperty("cpu_default_ms_x100_" + std::to_string(points), static_cast<int>(cpu_default * 100.0));
        RecordProperty("cpu_looks_ms_x100_" + std::to_string(points), static_cast<int>(cpu_looks * 100.0));
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &colour);
    glDeleteRenderbuffers(1, &depth);
}

// GlRenderer::update_csg_subtrees (D180, "run smoother"; the lead approved the header, 2026-11-19). A CSG vehicle that moves changes only its program's
// transforms and its box, yet upload_scene() redoes every mesh, the materials, the shadow map and the whole CSG tables, and draw() refuses a frame whose
// subtree is not byte for byte the uploaded one. update_csg_subtrees(scene) re-uploads only what changed: it packs each subtree as upload_scene() does,
// compares with the HOST copy of what was last uploaded (the one draw() compares against; never a GPU readback), and writes only the changed element ranges.
[[nodiscard]] RenderScene csg_moved_by(const RenderScene& good, float dx) {
    RenderScene moved = good;
    for (spade::SdfTransform& t : moved.csg_subtrees[0].program.transforms) {
        t.world_to_local = t.world_to_local * glm::translate(glm::mat4(1.0f), glm::vec3(-dx, 0.0f, 0.0f));
    }
    moved.csg_subtrees[0].bounds.min.x += dx;
    moved.csg_subtrees[0].bounds.max.x += dx;
    return moved;
}

TEST_F(GpuGlRenderer, AnUpdateAfterAMoveDrawsExactlyWhatAFreshUploadDraws) {
    const RenderScene good = slab_scene();
    ASSERT_EQ(good.csg_subtrees.size(), 1u);
    const std::vector<uint8_t> before = draw_and_read(good);
    ASSERT_EQ(before.size(), kPixels * 4u);
    const RenderScene moved = csg_moved_by(good, 0.5f);
    const auto report = renderer_->update_csg_subtrees(moved);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    EXPECT_GT(report->transforms_written, 0u) << "the moved leaf transforms are written";
    EXPECT_EQ(report->nodes_written, 0u) << "the nodes did not change";
    EXPECT_EQ(report->boxes_changed, 1u);
    EXPECT_TRUE(report->any());
    const std::vector<uint8_t> updated = draw_uploaded_and_read(moved);  // draw() passes its byte compare with the updated upload
    ASSERT_EQ(updated.size(), kPixels * 4u);
    ASSERT_TRUE(renderer_->upload_scene(moved).has_value());
    const std::vector<uint8_t> fresh = draw_uploaded_and_read(moved);
    EXPECT_EQ(count_differing(updated, fresh), 0u) << "an update is a fresh upload's frame, byte for byte";
    EXPECT_GT(count_differing(before, updated), 50u) << "and the move is real: the frame is not the unmoved one";
}

TEST_F(GpuGlRenderer, AnUnchangedSceneWritesNothingAndTheUpdateNeverReadsTheDeviceBack) {
    const RenderScene good = slab_scene();
    ASSERT_TRUE(renderer_->upload_scene(good).has_value());
    const auto report = renderer_->update_csg_subtrees(good);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    EXPECT_FALSE(report->any()) << "nothing changed: no node, no transform, no box";
    EXPECT_EQ(report->nodes_written + report->transforms_written + report->boxes_changed, 0u);
    EXPECT_TRUE(renderer_->draw(good, camera_on_plus_z(), spade::render::RenderOptions{}, kWidth, kHeight).has_value());
    // The compare is against the host copy of what was uploaded, never a read-back of the buffers: the renderer's source names no GetBufferSubData
    // and maps no buffer.
    std::ifstream in(std::string(SPADE_REPO_DIR) + "/engine/render_gl/gl_renderer.cpp", std::ios::binary);
    ASSERT_TRUE(in.good());
    const std::string source((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_EQ(source.find("GetBufferSubData"), std::string::npos);
    EXPECT_EQ(source.find("MapBuffer"), std::string::npos);
    EXPECT_NE(source.find("update_csg_subtrees"), std::string::npos) << "the source scanned is the renderer's";
}

// A parked vehicle under Run wobbles at about 1e-17 in x: its transform's bytes change, nothing else, and the update writes that one transform.
TEST_F(GpuGlRenderer, ANudgeOfOneUlpWritesThatOneTransformAndNothingElse) {
    const RenderScene good = slab_scene();
    ASSERT_TRUE(renderer_->upload_scene(good).has_value());
    RenderScene nudged = good;
    ASSERT_FALSE(nudged.csg_subtrees[0].program.transforms.empty());
    glm::mat4& m = nudged.csg_subtrees[0].program.transforms[0].world_to_local;
    m[3][0] = std::nextafter(m[3][0], 1.0e9f);
    ASSERT_NE(std::memcmp(&m, &good.csg_subtrees[0].program.transforms[0].world_to_local, sizeof(glm::mat4)), 0);
    const auto report = renderer_->update_csg_subtrees(nudged);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    EXPECT_EQ(report->transforms_written, 1u) << "the one transform whose bytes changed";
    EXPECT_EQ(report->nodes_written, 0u);
    EXPECT_EQ(report->boxes_changed, 0u);
    EXPECT_TRUE(renderer_->draw(nudged, camera_on_plus_z(), spade::render::RenderOptions{}, kWidth, kHeight).has_value());
    // and the same scene again is now unchanged
    const auto again = renderer_->update_csg_subtrees(nudged);
    ASSERT_TRUE(again.has_value());
    EXPECT_FALSE(again->any());
}

TEST_F(GpuGlRenderer, AnUpdateThatCannotBeAnUpdateIsRefusedAndTheOldUploadStillDraws) {
    const RenderScene good = slab_scene();
    ASSERT_TRUE(renderer_->upload_scene(good).has_value());
    const auto draw = [this](const RenderScene& scene) {
        return renderer_->draw(scene, camera_on_plus_z(), spade::render::RenderOptions{}, kWidth, kHeight);
    };
    RenderScene fewer = good;
    fewer.csg_subtrees.clear();
    RenderScene more_meshes = good;
    more_meshes.meshes.push_back(more_meshes.meshes.front());
    RenderScene bigger = good;  // a node more: the table layout moves, so it is an upload, not an update
    bigger.csg_subtrees[0].program.nodes.push_back(bigger.csg_subtrees[0].program.nodes.front());
    RenderScene empty_program = good;
    empty_program.csg_subtrees[0].program.nodes.clear();
    RenderScene past_statics = good;
    past_statics.csg_subtrees[0].draw_item = static_cast<uint32_t>(good.statics.size());
    for (const auto& [what, scene] : {std::pair<const char*, const RenderScene*>{"a different subtree count", &fewer},
                                      {"a different mesh count", &more_meshes},
                                      {"a different node count", &bigger},
                                      {"a subtree with no program", &empty_program},
                                      {"a draw item past the statics", &past_statics}}) {
        SCOPED_TRACE(what);
        const auto refused = renderer_->update_csg_subtrees(*scene);
        ASSERT_FALSE(refused.has_value()) << what;
        EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
        EXPECT_NE(refused.error().context.find("update_csg_subtrees"), std::string::npos) << refused.error().context;
        EXPECT_TRUE(draw(good).has_value()) << "a refusal changes nothing: the previous upload still draws";
    }
    // a draw of a refused scene is refused by draw(), as today
    EXPECT_FALSE(draw(fewer).has_value());
    // the node-count refusal says to upload instead
    const auto refused = renderer_->update_csg_subtrees(bigger);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("upload_scene"), std::string::npos) << refused.error().context;
}

// An update does not touch the meshes: a scene whose mesh was edited after its upload, updated, still draws the uploaded geometry (wireframe draws a
// subtree's mesh), where a fresh upload shows the edit.
TEST_F(GpuGlRenderer, AnUpdateLeavesTheUploadedMeshesAlone) {
    const RenderScene good = slab_scene();
    spade::render::RenderOptions wire = comparable_options();
    wire.mode = spade::render::DrawMode::wireframe;
    ASSERT_TRUE(renderer_->upload_scene(good).has_value());
    const std::vector<uint8_t> original = draw_uploaded_and_read(good, wire);
    ASSERT_GT(count_non_black(original), 50u);
    RenderScene edited = good;
    for (spade::render::MeshData& mesh : edited.meshes) {
        for (glm::vec3& p : mesh.positions) p.x += 0.4f;
    }
    const auto report = renderer_->update_csg_subtrees(edited);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    EXPECT_FALSE(report->any()) << "the CSG did not change";
    const std::vector<uint8_t> after_update = draw_uploaded_and_read(edited, wire);
    EXPECT_EQ(count_differing(original, after_update), 0u) << "the uploaded meshes are the ones drawn";
    ASSERT_TRUE(renderer_->upload_scene(edited).has_value());
    EXPECT_GT(count_differing(original, draw_uploaded_and_read(edited, wire)), 20u) << "a fresh upload shows the edit";
}

// The cost, for Interface's before and after (opt-in, SPADE_CSG_UPDATE_COST=1): the host time of upload_scene() against update_csg_subtrees() on a scene with
// the CSG slab and 64 more meshes, the slab moved a little each time. Median of 30 calls, glFinish after each.
TEST_F(GpuGlRenderer, AnUpdateCostsAFractionOfAFullUploadForAMovingSubtree) {
    if (!env_gate_is_set("SPADE_CSG_UPDATE_COST")) GTEST_SKIP() << "opt-in: set SPADE_CSG_UPDATE_COST=1";
    RenderScene base = slab_scene();
    for (int k = 0; k < 64; ++k) {
        spade::render::MeshData mesh = base.meshes.front();
        for (glm::vec3& p : mesh.positions) p += glm::vec3(0.01f * static_cast<float>(k), 0.0f, 0.0f);
        base.meshes.push_back(std::move(mesh));
    }
    ASSERT_TRUE(renderer_->upload_scene(base).has_value());
    const auto median = [](std::vector<double> ms) {
        std::sort(ms.begin(), ms.end());
        return 0.5 * (ms[ms.size() / 2 - 1] + ms[ms.size() / 2]);
    };
    std::vector<double> upload_ms, update_ms;
    for (int i = 0; i < 32; ++i) {
        const RenderScene scene = csg_moved_by(base, 0.001f * static_cast<float>(i + 1));
        const RenderScene next = csg_moved_by(base, 0.001f * static_cast<float>(i + 2));  // both built outside the timing: a scene copy is not the call's cost
        glFinish();
        const auto t0 = std::chrono::steady_clock::now();
        const spade::Result<void> up = renderer_->upload_scene(scene);
        glFinish();
        const auto t1 = std::chrono::steady_clock::now();
        const auto report = renderer_->update_csg_subtrees(next);
        glFinish();
        const auto t2 = std::chrono::steady_clock::now();
        EXPECT_TRUE(up.has_value());
        EXPECT_TRUE(report.has_value());
        if (i >= 2) {
            upload_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            update_ms.push_back(std::chrono::duration<double, std::milli>(t2 - t1).count());
        }
    }
    const double upload = median(upload_ms);
    const double update = median(update_ms);
    const std::string device = renderer_->renderer_name() + " (" + renderer_->version_string() + ")";
    std::printf("[ csg update cost ] 65 meshes and one CSG subtree: upload_scene %.3f ms, update_csg_subtrees %.3f ms (%.1fx); on %s\n", upload, update,
                update > 0.0 ? upload / update : 0.0, device.c_str());
    RecordProperty("upload_ms_x1000", static_cast<int>(upload * 1000.0));
    RecordProperty("update_ms_x1000", static_cast<int>(update * 1000.0));
    EXPECT_LT(update, upload) << device;
}

// A search then an extrapolation (O158), on GL as on the CPU: the same slab as the search test with p = 0.8 z beyond it; the wall's value is the line through the first fluid read and the
// one an offset further out. GL's frame is the CPU's within one level.
TEST_F(GpuGlRenderer, ASearchedThenExtrapolatedSurfaceColouringMatchesTheCpu) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-2.0f, -2.0f, -1.0f), 0.25f, 16, 16, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowPressure] = std::fabs(p.z) < 0.3f ? std::numeric_limits<float>::quiet_NaN() : 0.8f * p.z;
        return c;
    });
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::pressure;
    colours.colours = spade::render::ColourMapId::cool_warm;
    colours.range_min = -1.0f;
    colours.range_max = 1.0f;
    RenderScene scene;
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-1.5f, -1.5f, 0.0f), glm::vec3(1.5f, -1.5f, 0.0f), glm::vec3(1.5f, 1.5f, 0.0f), glm::vec3(-1.5f, 1.5f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    scene.meshes.push_back(quad);
    scene.materials = {Material{}};
    DrawItem item{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)};
    item.flow_wall = true;
    scene.statics.push_back(item);
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    spade::render::FlowSurfaceColouring colouring{&volume, colours};
    colouring.walls_only = true;
    colouring.offset_cells = 1.0f;
    colouring.search_cells = 2.0f;
    scene.flow_surface_colouring = colouring;
    const std::vector<uint8_t> searched = cpu_rgba(scene, camera, comparable_options());
    scene.flow_surface_colouring->extrapolate = true;
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, comparable_options());
    ASSERT_EQ(gl.size(), kPixels * 4u);
    EXPECT_GT(channel_diff(searched, cpu).differing, kPixels / 50u) << "the extrapolation moves the CPU's colour off the search's";
    const ChannelDiff band = channel_diff(cpu, gl);
    EXPECT_LE(band.worst, 1) << renderer_->renderer_name();
    EXPECT_EQ(band.over_two, 0u);
}

// The planar texture (SP-4 branch 3), on GL as on the CPU (tests/test_render_planar_texture.cpp is the CPU's): an image over the items that ask for it, mapped by
// world position and lit as a white material. These are the GL-to-CPU band cases and the two the CPU side shares: nothing asking changes nothing, and a constant
// image is the plain material's frame.
namespace planar {

[[nodiscard]] spade::render::PlanarTexture checker(uint32_t n, float half) {
    spade::render::PlanarTexture t;
    t.width = t.height = n;
    for (uint32_t j = 0; j < n; ++j) {
        for (uint32_t i = 0; i < n; ++i) {
            const bool a = ((i + j) & 1u) == 0u;
            t.rgb.push_back(a ? 230 : 30);
            t.rgb.push_back(40);
            t.rgb.push_back(a ? 40 : 230);
        }
    }
    t.x_min = t.z_min = -half;
    t.x_max = t.z_max = half;
    return t;
}

// A quad at y = 0 over [-half, half]^2 facing up, as the item `textured` or not.
[[nodiscard]] RenderScene quad_scene(float half, glm::vec4 base, bool textured) {
    RenderScene scene;
    scene.materials = {Material{.base_color = base}};
    scene.lighting.sun_direction = glm::normalize(glm::vec3(0.4f, 0.8f, 0.6f));
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-half, 0.0f, -half), glm::vec3(-half, 0.0f, half), glm::vec3(half, 0.0f, -half), glm::vec3(half, 0.0f, half)};
    quad.normals.assign(4, glm::vec3(0.0f, 1.0f, 0.0f));
    quad.indices = {0, 1, 2, 2, 1, 3};
    scene.meshes.push_back(quad);
    DrawItem item{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)};
    item.planar_textured = textured;
    scene.statics.push_back(item);
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-500.0f, -5.0f, -500.0f), .max = glm::vec3(500.0f, 50.0f, 500.0f)};
    return scene;
}

[[nodiscard]] Camera oblique() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 3.0f, 60.0f);
    camera.orientation = glm::angleAxis(-0.18f, glm::vec3(1.0f, 0.0f, 0.0f));
    return camera;
}

}  // namespace planar

TEST_F(GpuGlRenderer, APlanarTexturedQuadSeenObliquelyMatchesTheCpuWithinItsBand) {
    RenderScene scene = planar::quad_scene(100.0f, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f), true);
    scene.planar_texture = planar::checker(16, 100.0f);
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), planar::oblique());
    const std::vector<uint8_t> cpu = cpu_rgba(scene, planar::oblique(), comparable_options());
    ASSERT_EQ(gl.size(), kPixels * 4u);
    RenderScene flat = planar::quad_scene(100.0f, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f), false);
    EXPECT_GT(channel_diff(cpu, cpu_rgba(flat, planar::oblique(), comparable_options())).differing, kPixels / 10u) << "the texture is in the CPU frame at all";
    const ChannelDiff band = channel_diff(cpu, gl);
    EXPECT_LE(band.worst, 2) << renderer_->renderer_name();
    EXPECT_LE(band.over_two, kPixels / 200u) << "the quad's edge pixels only";
}

TEST_F(GpuGlRenderer, AConstantPlanarTextureDrawsThePlainMaterialsFrameOnGl) {
    RenderScene plain = planar::quad_scene(10.0f, glm::vec4(0.8f, 0.5f, 0.2f, 1.0f), false);
    RenderScene textured = planar::quad_scene(10.0f, glm::vec4(0.1f, 0.9f, 0.9f, 1.0f), true);  // its own colour is not drawn
    spade::render::PlanarTexture t;
    t.width = t.height = 4;
    for (int i = 0; i < 16; ++i) {
        t.rgb.insert(t.rgb.end(), {204, 128, 51});
    }
    t.x_min = t.z_min = -10.0f;
    t.x_max = t.z_max = 10.0f;
    textured.planar_texture = t;
    Camera camera;
    camera.position = glm::vec3(0.0f, 20.0f, 0.0f);
    camera.orientation = glm::angleAxis(-1.5707963267948966f, glm::vec3(1.0f, 0.0f, 0.0f));
    const ChannelDiff band = channel_diff(draw_and_read(plain, comparable_options(), camera), draw_and_read(textured, comparable_options(), camera));
    EXPECT_LE(band.worst, 1) << renderer_->renderer_name();
}

TEST_F(GpuGlRenderer, ATexturePresentThatNoItemAsksForChangesNothingOnGl) {
    RenderScene without = planar::quad_scene(10.0f, glm::vec4(0.8f, 0.5f, 0.2f, 1.0f), false);
    RenderScene with = without;
    with.planar_texture = planar::checker(8, 10.0f);
    Camera camera;
    camera.position = glm::vec3(0.0f, 20.0f, 0.0f);
    camera.orientation = glm::angleAxis(-1.5707963267948966f, glm::vec3(1.0f, 0.0f, 0.0f));
    EXPECT_EQ(count_differing(draw_and_read(without, comparable_options(), camera), draw_and_read(with, comparable_options(), camera)), 0u);
}

// attach_terrain with the image, on GL: the quadrants' colours land with north up (GL's frame is bottom row first).
TEST_F(GpuGlRenderer, ATexturedTerrainDrawsItsImageNorthUpOnGl) {
    spade::world::TerrainInput map;
    map.nx = 20;
    map.ny = 20;
    map.extent = {-10.0, 10.0, -10.0, 10.0};
    map.heights.assign(20u * 20u, 0.0f);
    map.image.width = 2;
    map.image.height = 2;
    map.image.rgb = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0};  // north row: red, green; south row: blue, yellow
    RenderScene scene;
    scene.materials = {Material{}};
    scene.lighting.sun_direction = glm::normalize(glm::vec3(0.4f, 0.8f, 0.6f));
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-60.0f, -5.0f, -60.0f), .max = glm::vec3(60.0f, 5.0f, 60.0f)};
    spade::render::TerrainSpec spec;
    spec.textured = true;
    ASSERT_TRUE(spade::render::attach_terrain(scene, map, spec).has_value());
    Camera camera;
    camera.position = glm::vec3(0.0f, 20.0f, 0.0f);
    camera.orientation = glm::angleAxis(-1.5707963267948966f, glm::vec3(1.0f, 0.0f, 0.0f));
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    ASSERT_EQ(gl.size(), kPixels * 4u);
    const auto at = [&](int px, int py_from_top) {
        const size_t i = (static_cast<size_t>(kHeight - 1 - py_from_top) * kWidth + static_cast<size_t>(px)) * 4u;
        return std::array<int, 3>{gl[i], gl[i + 1], gl[i + 2]};
    };
    const auto nw = at(54, 34), ne = at(106, 34), sw = at(54, 86), se = at(106, 86);
    EXPECT_TRUE(nw[0] > 120 && nw[1] < 40 && nw[2] < 40) << "north-west: red";
    EXPECT_TRUE(ne[1] > 120 && ne[0] < 40 && ne[2] < 40) << "north-east: green";
    EXPECT_TRUE(sw[2] > 120 && sw[0] < 40 && sw[1] < 40) << "south-west: blue";
    EXPECT_TRUE(se[0] > 120 && se[1] > 120 && se[2] < 40) << "south-east: yellow";
}

// A flat-coloured terrain with a slope under it (heights vary, so the shading does), used by both pins below. `set_flat` asks for the flat colour wherever the spec has the flag (a spec before the
// planar texture has none and is flat already), so the same text pins the frame before and after the texture was added.
namespace flat_pin {

template <typename Spec>
void set_flat(Spec& spec) {
    if constexpr (requires { spec.textured; }) spec.textured = false;
}

[[nodiscard]] spade::world::TerrainInput map() {
    spade::world::TerrainInput m;
    m.nx = 24;
    m.ny = 24;
    m.extent = {-30.0, 30.0, -30.0, 30.0};
    m.heights.resize(24u * 24u);
    for (uint32_t j = 0; j < 24u; ++j) {
        for (uint32_t i = 0; i < 24u; ++i) {
            const double east = -30.0 + (i + 0.5) * 60.0 / 24.0;
            const double north = 30.0 - (j + 0.5) * 60.0 / 24.0;
            m.heights[static_cast<size_t>(j) * 24u + i] = static_cast<float>(0.08 * east + 0.05 * north + 1.5 * std::sin(0.3 * east) * std::cos(0.2 * north));
        }
    }
    m.image.width = 2;
    m.image.height = 2;
    m.image.rgb.assign(12u, 200u);
    return m;
}

[[nodiscard]] spade::render::RenderScene scene() {
    spade::render::RenderScene s;
    s.materials = {spade::render::Material{}};
    s.bounds = {glm::vec3(-120.0f, -20.0f, -120.0f), glm::vec3(120.0f, 40.0f, 120.0f)};
    s.lighting.sun_direction = glm::normalize(glm::vec3(0.4f, 0.8f, 0.6f));
    spade::render::TerrainSpec spec;
    spec.base_color = glm::vec3(0.55f, 0.62f, 0.35f);
    set_flat(spec);
    const auto attached = spade::render::attach_terrain(s, map(), spec);
    if (!attached) ADD_FAILURE() << attached.error().context;
    return s;
}

[[nodiscard]] spade::render::Camera camera() {
    spade::render::Camera c;
    c.position = glm::vec3(4.0f, 14.0f, 40.0f);
    c.orientation = glm::angleAxis(-0.42f, glm::vec3(1.0f, 0.0f, 0.0f));
    return c;
}

// FNV-1a over the frame's bytes.
[[nodiscard]] uint64_t digest(const std::vector<uint8_t>& bytes) {
    uint64_t h = 1469598103934665603ull;
    for (const uint8_t b : bytes) h = (h ^ b) * 1099511628211ull;
    return h;
}

}  // namespace flat_pin

// The same flat terrain on GL, pinned the same way: the digest of GL's frame on the card the gate runs on, taken from the build WITHOUT the texture. A different driver may round differently; the
// CPU pin is the portable one, this is the check that the GL shader's new branch leaves a flat terrain alone.
TEST_F(GpuGlRenderer, TheFlatTerrainsGlFrameIsTheOneBeforeTheTexture) {
    const RenderScene scene = flat_pin::scene();
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), flat_pin::camera());
    ASSERT_EQ(gl.size(), kPixels * 4u);
    EXPECT_GT(count_non_black(gl), kPixels / 5u) << "the terrain is in the frame";
    EXPECT_EQ(flat_pin::digest(gl), UINT64_C(16726099002277015038)) << renderer_->renderer_name() << ": the flat terrain's GL frame changed";
}

// The no-search extrapolation on GL as on the CPU (O158): p = 0.8 z, no solid, the wall's value 0; GL's frame is the CPU's within one level.
TEST_F(GpuGlRenderer, AnExtrapolatedSurfaceColouringWithNoSearchMatchesTheCpu) {
    const spade::FlowVolume volume = spade::test::make_flow_volume(glm::vec3(-2.0f, -2.0f, -1.0f), 0.25f, 16, 16, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowPressure] = 0.8f * p.z;
        return c;
    });
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::pressure;
    colours.colours = spade::render::ColourMapId::cool_warm;
    colours.range_min = -1.0f;
    colours.range_max = 1.0f;
    RenderScene scene;
    spade::render::MeshData quad;
    quad.positions = {glm::vec3(-1.5f, -1.5f, 0.0f), glm::vec3(1.5f, -1.5f, 0.0f), glm::vec3(1.5f, 1.5f, 0.0f), glm::vec3(-1.5f, 1.5f, 0.0f)};
    quad.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    quad.indices = {0, 1, 2, 0, 2, 3};
    scene.meshes.push_back(quad);
    scene.materials = {Material{}};
    DrawItem item{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)};
    item.flow_wall = true;
    scene.statics.push_back(item);
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    spade::render::FlowSurfaceColouring colouring{&volume, colours};
    colouring.walls_only = true;
    colouring.offset_cells = 1.0f;
    colouring.extrapolate = true;
    scene.flow_surface_colouring = colouring;
    const std::vector<uint8_t> gl = draw_and_read(scene, comparable_options(), camera);
    const std::vector<uint8_t> cpu = cpu_rgba(scene, camera, comparable_options());
    ASSERT_EQ(gl.size(), kPixels * 4u);
    const ChannelDiff band = channel_diff(cpu, gl);
    EXPECT_LE(band.worst, 1) << renderer_->renderer_name();
    EXPECT_EQ(band.over_two, 0u);
}

}  // namespace
