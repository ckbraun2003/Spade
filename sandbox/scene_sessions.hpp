// The sandbox's two scenes as sessions: everything a scene needs to draw a
// frame into the window, and the one function that draws it.
//
// ⭐ ONE FRAME, ONE STATEMENT. The interactive loops, the builder's --smoke and
// the live smoke (live_tour.hpp) all drive a scene through its session's
// frame(), so a smoke exercises the path a user does. A smoke with a private
// frame loop proves only that the smoke works; this file is where main.cpp's
// frame bodies moved so that a second caller could share them.
//
// Display-free logic stays in builder_scene.hpp, drone_sim.hpp and
// drone_view.hpp (SL15b); a session is wiring.

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

#if SPADE_SANDBOX_HAS_GPU
#include "render_gl/gl_renderer.hpp"  // v2's GPU render backend -- the PRIMARY path
#endif

#include "builder_scene.hpp"   // the builder's object model and its whole interaction
#include "drone_view.hpp"      // the drone sim box: stand, controller, air field, heatmap (+ drone_sim.hpp)
#include "gl_target_sink.hpp"  // Plan C task C2 -- the window
#include "orbit_camera.hpp"    // Plan C task C2 -- input, testable with no display
#include "target_sink.hpp"     // Plan C task C1 -- the seam SL11 names

namespace spade::sandbox {

#if SPADE_SANDBOX_HAS_GPU
using GpuRenderer = spade::render_gl::GlRenderer;
#else
// ⚠ BUILT WITH SPADE_RENDER_GL=OFF: THERE IS NO GPU RENDERER, AND THIS TYPE
// DOES NOT PRETEND TO BE ONE. create() refuses with the cause, so start_gpu()
// announces the CPU path exactly as it does when a driver refuses (a missing
// capability is announced, never silent), and every `if (gpu)` below is false
// for the whole run. The other members exist only so those branches compile:
// the constructor is private and create() never succeeds, so none of them can
// run. One guard here instead of one at every GPU call.
class GpuRenderer {
  public:
    [[nodiscard]] static spade::Result<std::unique_ptr<GpuRenderer>> create(void* (*)(const char*)) {
        return std::unexpected(off());
    }
    [[nodiscard]] spade::Result<void> upload_scene(const spade::render::RenderScene&) {
        return std::unexpected(off());
    }
    [[nodiscard]] spade::Result<void> draw(const spade::render::RenderScene&, const spade::render::Camera&,
                                           const spade::render::RenderOptions&, uint32_t, uint32_t) {
        return std::unexpected(off());
    }
    [[nodiscard]] uint32_t last_draw_calls() const noexcept { return 0u; }
    [[nodiscard]] uint32_t last_instances() const noexcept { return 0u; }
    [[nodiscard]] const std::string& renderer_name() const noexcept { return none_; }
    [[nodiscard]] const std::string& version_string() const noexcept { return none_; }
    [[nodiscard]] static std::vector<std::string_view> unhonoured(const spade::render::RenderOptions&) {
        return {};
    }

  private:
    GpuRenderer() = default;
    [[nodiscard]] static spade::Error off() {
        return spade::Error{spade::Code::unavailable,
                            "this sandbox was built with SPADE_RENDER_GL=OFF, so it has no GPU renderer"};
    }
    std::string none_;
};
#endif

// C0's built-in scene, the builder's ground: a ground plane and nothing else.
// The world is an out-parameter because the scene borrows its SDF: the
// returned scene must not outlive `world_out`.
[[nodiscard]] bool build_builtin_scene(spade::render::RenderScene& out, spade::WorldDesc& world_out);

// The drone sim box's empty world (no ground): the drone hangs in air. The
// world is an out-parameter for the same reason.
[[nodiscard]] bool build_drone_scene(spade::render::RenderScene& out, spade::WorldDesc& world_out);

// Renders on the CPU and hands the target to a TargetSink (C1).
[[nodiscard]] bool render_frame(const spade::render::RenderScene& scene, const spade::render::Camera& camera,
                                uint32_t width, uint32_t height, const spade::render::RenderOptions& options,
                                std::vector<uint8_t>& pixels, TargetSink& sink);

// The drone frame's draw list and options, shared by the window and headless.
[[nodiscard]] spade::render::RenderOptions drone_frame(spade::render::RenderScene& scene,
                                                       const DroneDrawBinding& binding, const DroneSim& drone,
                                                       const glm::quat& orientation,
                                                       const spade::render::Camera& camera, bool heatmap,
                                                       float heatmap_max, float blur, float* observed_max);

// GPU first, CPU fallback: the GPU renderer when it can be created and take
// the scene, otherwise nullptr, with the reason printed.
[[nodiscard]] std::unique_ptr<GpuRenderer> start_gpu(const spade::render::RenderScene& scene);

// Opens the window, or explains why not and returns nullptr.
[[nodiscard]] std::unique_ptr<GlTargetSink> open_window(uint32_t width, uint32_t height, bool vsync);

// ---------------------------------------------------------------------------
// The drone sim box in a window.
// ---------------------------------------------------------------------------
class DroneSession {
  public:
    // Builds the stand and the scene. Needs no window, so a build that fails
    // refuses before one opens.
    [[nodiscard]] static spade::Result<std::unique_ptr<DroneSession>> create(float blur);

    // Starts the GPU renderer in `sink`'s context, or announces the CPU
    // fallback, and attaches the panel. `sink` must outlive this. Call once,
    // before the first frame.
    void attach(GlTargetSink& sink);

    ~DroneSession();
    DroneSession(const DroneSession&) = delete;
    DroneSession& operator=(const DroneSession&) = delete;

    // One frame: the orbit and attitude keys, the view toggle and the panel's
    // edits (debounced inside), `dt` seconds of stepping, then a frame drawn
    // and presented. A hard failure returns its cause.
    [[nodiscard]] spade::Result<void> frame(const FrameInput& in, float dt);

    [[nodiscard]] DroneSim& drone() noexcept { return *drone_; }
    [[nodiscard]] DronePanelModel& panel() noexcept { return panel_; }
    [[nodiscard]] OrbitCamera& camera() noexcept { return camera_; }
    [[nodiscard]] bool gpu_active() const noexcept { return gpu_ != nullptr; }
    // Rebuilds the panel's edits have caused so far.
    [[nodiscard]] uint32_t rebuilds() const noexcept { return rebuilds_; }

  private:
    DroneSession() = default;

    GlTargetSink* sink_ = nullptr;
    float blur_ = 0.0f;
    std::optional<DroneSim> drone_;
    // Declared before the scene, so it is destroyed after it: the scene
    // borrows its SDF.
    spade::WorldDesc world_;
    spade::render::RenderScene scene_;
    DroneDrawBinding binding_{};
    std::unique_ptr<GpuRenderer> gpu_;
    DronePanelModel panel_;
    OrbitCamera camera_;
    std::vector<uint8_t> pixels_;
    uint32_t rebuilds_ = 0;
};

// ---------------------------------------------------------------------------
// The builder in a window.
// ---------------------------------------------------------------------------
class BuilderSession {
  public:
    // Builds the ground scene and the builder's meshes and palette. Needs no
    // window, so a build that fails refuses before one opens.
    [[nodiscard]] static spade::Result<std::unique_ptr<BuilderSession>> create(bool grid, float blur);

    // Starts the GPU renderer in `sink`'s context, or announces the CPU
    // fallback, and attaches the builder panel. `sink` must outlive this.
    // Call once, before the first frame.
    void attach(GlTargetSink& sink);

    ~BuilderSession();
    BuilderSession(const BuilderSession&) = delete;
    BuilderSession& operator=(const BuilderSession&) = delete;

    // One frame: the camera, everything the mouse means to the builder, the
    // uploads a changed set needs, then a frame drawn and presented.
    [[nodiscard]] spade::Result<void> frame(const FrameInput& in);

    [[nodiscard]] BuilderScene& builder() noexcept { return builder_; }
    [[nodiscard]] spade::render::RenderScene& scene() noexcept { return scene_; }
    [[nodiscard]] OrbitCamera& camera() noexcept { return camera_; }
    [[nodiscard]] GpuRenderer* gpu() noexcept { return gpu_.get(); }
    // Where the builder's meshes and materials start in scene(): object i
    // draws with material binding().material_base + i.
    [[nodiscard]] const BuilderBinding& binding() const noexcept { return bind_; }

  private:
    BuilderSession() = default;

    GlTargetSink* sink_ = nullptr;
    spade::WorldDesc world_;  // before the scene: the scene borrows its SDF
    spade::render::RenderScene scene_;
    spade::render::RenderOptions options_;
    BuilderScene builder_;
    BuilderBinding bind_{};
    std::unique_ptr<GpuRenderer> gpu_;
    OrbitCamera camera_;
    std::vector<uint8_t> pixels_;
};

}  // namespace spade::sandbox
