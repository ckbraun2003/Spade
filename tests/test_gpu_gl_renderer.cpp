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

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render_gl/gl_renderer.hpp"

namespace {

using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render_gl::GlRenderer;

constexpr int kWidth = 64;
constexpr int kHeight = 64;

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

// One static item, unlit white: the test is about coverage, so lighting
// (and the default sun, RND-5) cannot decide whether a pixel lights.
[[nodiscard]] RenderScene make_scene(MeshData mesh) {
    RenderScene scene;
    scene.meshes.push_back(std::move(mesh));
    scene.materials = {Material{.base_color = glm::vec4(1.0f), .shading = 1u}};
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)});
    return scene;
}

// At +Z looking down -Z (the default orientation), as in the CPU test.
[[nodiscard]] Camera camera_on_plus_z() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    return camera;
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

    // Clears to black, draws `scene` with default options, and counts the
    // pixels that are no longer black.
    [[nodiscard]] size_t covered_pixels(const RenderScene& scene) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        if (auto up = renderer_->upload_scene(scene); !up) {
            ADD_FAILURE() << up.error().context;
            return 0;
        }
        if (auto drew = renderer_->draw(scene, camera_on_plus_z(), RenderOptions{}, kWidth, kHeight); !drew) {
            ADD_FAILURE() << drew.error().context;
            return 0;
        }
        std::vector<uint8_t> rgba(static_cast<size_t>(kWidth) * kHeight * 4u);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        EXPECT_EQ(glGetError(), static_cast<GLenum>(GL_NO_ERROR));
        size_t covered = 0;
        for (size_t i = 0; i < rgba.size(); i += 4u) {
            if (rgba[i] != 0u || rgba[i + 1u] != 0u || rgba[i + 2u] != 0u) ++covered;
        }
        return covered;
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

}  // namespace
