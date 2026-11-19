// What the editor's per-frame scene upload cost, and what CsgUploadGate saves (L88v). Rendering is GL on a hidden window, so it carries the
// `gpu` label (the Gpu suite prefix) and skips without a display.
//
// The scene is the editor's way of drawing: a base palette, then each vehicle's parts bound where it is (append_vehicle_parts()): one vehicle
// with a CSG ring part and three plain reference quads. For each case the frame is timed with the old policy (upload_scene() every frame, as
// EditorSession did when any model had a CSG piece) and with the gate (upload only when a CSG subtree changed), by the wall clock around the
// upload (when there is one), the draw and a glFinish(). It is the renderer's cost per frame at 1280 x 720, not the whole editor frame (the
// panels, the sim and the present are not here). The numbers are printed, never asserted; the upload COUNTS are asserted.
//
//   still:  the scene is the same every frame (also what a run with the CSG vehicle parked gives the renderer: the sim steps, the pose does not)
//   flying: the CSG vehicle moves every frame, so its program differs every frame and the gate uploads every frame

#include <gtest/gtest.h>

#if __has_include("vehicle_draw.hpp")

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "reference_airframe.hpp"
#include "render/scene.hpp"
#include "render_gl/gl_renderer.hpp"
#include "render_parts/render_parts.hpp"
#include "vehicle_draw.hpp"
#include "vehicles/airframe_compile.hpp"
#include "world/sdf.hpp"

namespace {

namespace vh = spade::vehicles;
using spade::render::Camera;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render_gl::GlRenderer;
using Clock = std::chrono::steady_clock;

constexpr int kWidth = 1280;
constexpr int kHeight = 720;
constexpr int kWarm = 20;
constexpr int kFrames = 150;

vh::ModelType reference_model() {
    return vh::compile_airframe(vh::fixtures::reference_quad()).value().model;
}

// The reference quad with one CSG part (a box minus a sphere) on its body.
vh::ModelType quad_with_ring() {
    vh::ModelType model = reference_model();
    spade::SdfProgram ring;
    ring.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode box{};
    box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    box.params = glm::vec4(0.06f, 0.03f, 0.06f, 0.0f);
    spade::SdfNode ball{};
    ball.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    ball.params = glm::vec4(0.045f, 0.0f, 0.0f, 0.0f);
    spade::SdfNode subtract{};
    subtract.op = static_cast<uint32_t>(spade::SdfOp::subtract);
    ring.nodes = {box, ball, subtract};
    model.parts.push_back(vh::PartDesc{.name = "ring",
                                       .role = "test",
                                       .attach = vh::PartAttach::body,
                                       .rotor = 0,
                                       .mount_pos = glm::vec3(0.02f, 0.01f, -0.03f),
                                       .mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                       .shape = std::move(ring),
                                       .surface = vh::PartSurface{.kind = vh::PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
                                       .min_feature = 0.001f});
    return model;
}

spade::render_parts::PartsLook look() {
    spade::render_parts::PartsLook l;
    l.body = 0;
    l.rotor = 1;
    l.porous = 2;
    return l;
}

class GpuCsgUploadCost : public ::testing::Test {
  protected:
    void SetUp() override {
        if (glfwInit() != GLFW_TRUE) GTEST_SKIP() << "GLFW could not initialise; no display is available to this process";
        glfw_ = true;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(kWidth, kHeight, "spade GpuCsgUploadCost", nullptr, nullptr);
        if (window_ == nullptr) GTEST_SKIP() << "no OpenGL 4.3 core context on this machine";
        glfwMakeContextCurrent(window_);
        if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) == 0) GTEST_SKIP() << "glad could not load GL";
        auto made = GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress), GlRenderer::CreateOptions{});
        if (!made) {
            if (made.error().code == spade::Code::unavailable) GTEST_SKIP() << made.error().context;
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
        renderer_.reset();
        if (fbo_ != 0) glDeleteFramebuffers(1, &fbo_);
        if (colour_ != 0) glDeleteRenderbuffers(1, &colour_);
        if (depth_ != 0) glDeleteRenderbuffers(1, &depth_);
        if (window_ != nullptr) glfwDestroyWindow(window_);
        if (glfw_) glfwTerminate();
    }

    // The scene as the editor draws it at frame `frame`: the CSG vehicle at x = `speed * frame`, three plain quads beside it.
    static RenderScene scene_at(const std::vector<std::pair<vh::ModelType, glm::vec3>>& vehicles) {
        RenderScene scene;
        scene.materials.resize(3);
        for (const auto& [model, pos] : vehicles) {
            const auto prepared = spade::render_parts::prepare_parts(model);
            EXPECT_TRUE(prepared.has_value());
            if (!prepared.has_value()) continue;
            EXPECT_TRUE(spade::sandbox::append_vehicle_parts(scene, *prepared, model, pos, glm::quat(1.0f, 0.0f, 0.0f, 0.0f), look()).has_value());
        }
        return scene;
    }

    // Mean ms per frame over kFrames, and how many uploads were made; `gated` uploads only when the gate says a subtree changed.
    struct Cost {
        double ms = 0.0;
        int uploads = 0;
    };
    Cost run(bool gated, float speed) {
        const vh::ModelType ring = quad_with_ring();
        const vh::ModelType plain = reference_model();
        Camera camera;
        camera.position = glm::vec3(0.0f, 0.0f, 3.0f);
        RenderOptions options;
        options.shadows = false;
        options.overlays = false;
        spade::sandbox::CsgUploadGate gate;
        Cost cost;
        double total = 0.0;
        for (int frame = 0; frame < kWarm + kFrames; ++frame) {
            const float x = speed * static_cast<float>(frame);
            const RenderScene scene = scene_at({{ring, glm::vec3(x, 0.0f, 0.0f)},
                                                {plain, glm::vec3(-0.6f, 0.0f, 0.0f)},
                                                {plain, glm::vec3(0.0f, 0.4f, 0.0f)},
                                                {plain, glm::vec3(0.0f, -0.4f, 0.0f)}});
            const auto t0 = Clock::now();
            if (!gated || gate.needs_upload(scene)) {
                const auto up = renderer_->upload_scene(scene);
                EXPECT_TRUE(up.has_value()) << (up.has_value() ? "" : up.error().context);
                gate.note_uploaded(scene);
                if (frame >= kWarm) ++cost.uploads;
            }
            glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            const auto drew = renderer_->draw(scene, camera, options, kWidth, kHeight);
            EXPECT_TRUE(drew.has_value()) << (drew.has_value() ? "" : drew.error().context);
            glFinish();
            if (frame >= kWarm) total += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        }
        cost.ms = total / kFrames;
        return cost;
    }

    GLFWwindow* window_ = nullptr;
    bool glfw_ = false;
    std::unique_ptr<GlRenderer> renderer_;
    GLuint fbo_ = 0, colour_ = 0, depth_ = 0;
};

TEST_F(GpuCsgUploadCost, AStillSceneUploadsOnceWithTheGateAndEveryFrameWithout) {
    const Cost every = run(false, 0.0f);
    const Cost gated = run(true, 0.0f);
    std::cout << "[csg-upload] still:  every-frame upload " << every.ms << " ms/frame (" << every.uploads << " uploads over " << kFrames
              << "), gated " << gated.ms << " ms/frame (" << gated.uploads << " uploads)\n";
    EXPECT_EQ(every.uploads, kFrames);
    EXPECT_EQ(gated.uploads, 0) << "the warm-up frames uploaded it; the timed ones found it unchanged";
}

TEST_F(GpuCsgUploadCost, AFlyingCsgVehicleUploadsEveryFrameEitherWay) {
    const Cost every = run(false, 0.004f);
    const Cost gated = run(true, 0.004f);
    std::cout << "[csg-upload] flying: every-frame upload " << every.ms << " ms/frame (" << every.uploads << " uploads over " << kFrames
              << "), gated " << gated.ms << " ms/frame (" << gated.uploads << " uploads)\n";
    EXPECT_EQ(every.uploads, kFrames);
    EXPECT_EQ(gated.uploads, kFrames) << "its program changes every frame, so the gate cannot skip it";
}

}  // namespace

#else

TEST(GpuCsgUploadCost, NeedsTheSandboxHeaders) {
    GTEST_SKIP() << "built without the sandbox directory on the include path";
}

#endif
