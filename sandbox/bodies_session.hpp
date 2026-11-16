// The tour's chapter 4 in a window: sandbox/bodies_sim.hpp (L68's contact tumbling, four shapes in a row) drawn on a
// ground, a frame of the sim per frame of the window. Like the drone box and the airflow chapter it is a session the
// tour drives with no input of its own: create() needs no window, attach() starts the renderer, frame() steps and draws.

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "bodies_sim.hpp"
#include "core/error.hpp"
#include "gl_target_sink.hpp"
#include "orbit_camera.hpp"
#include "render/scene.hpp"
#include "scene_sessions.hpp"  // GpuRenderer, start_gpu(), render_frame(): the helpers every session shares
#include "target_sink.hpp"
#include "world/builder.hpp"

namespace spade::sandbox {

// The camera starts wide, with the sphere rolling in shot, and the tour dollies in to kBodiesEndDistance while the shapes
// tumble, so each covers a good part of the picture by the time it lands.
inline constexpr float kBodiesStartDistance = 3.4f;
inline constexpr float kBodiesEndDistance = 2.2f;

class BodiesSession {
  public:
    // The sim and the scene, no window: a build that fails refuses before one opens.
    [[nodiscard]] static Result<std::unique_ptr<BodiesSession>> create();

    // Starts the GPU renderer in `sink`'s context (the CPU rasteriser when there is none). `sink` must outlive this.
    // Call once, before the first frame.
    void attach(GlTargetSink& sink);

    ~BodiesSession();
    BodiesSession(const BodiesSession&) = delete;
    BodiesSession& operator=(const BodiesSession&) = delete;

    // `dt` seconds of the sim (whole milliseconds, the rest carried), then a frame drawn and presented.
    [[nodiscard]] Result<void> frame(float dt);

    [[nodiscard]] bodies::BodiesSim& sim() noexcept { return *sim_; }
    [[nodiscard]] OrbitCamera& camera() noexcept { return camera_; }
    [[nodiscard]] bool gpu_active() const noexcept { return gpu_ != nullptr; }

  private:
    BodiesSession() = default;
    void draw_items();

    GlTargetSink* sink_ = nullptr;
    std::unique_ptr<bodies::BodiesSim> sim_;
    // Declared before the scene, so it is destroyed after it: the scene borrows the world's SDF.
    WorldDesc world_;
    render::RenderScene scene_;
    std::unique_ptr<GpuRenderer> gpu_;
    OrbitCamera camera_;
    std::vector<uint8_t> pixels_;

    // Indices into scene_, bound once.
    uint32_t box_mesh_ = 0, sphere_mesh_ = 0, cylinder_mesh_ = 0;
    uint32_t sphere_mat_ = 0, cube_mat_ = 0, capsule_mat_ = 0, hull_mat_ = 0, rotor_mat_ = 0;
};

}  // namespace spade::sandbox
