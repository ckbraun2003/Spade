// A scene file in a window: the built-in scenes of assets/scenes/, run.
//
// Each lane is one scene file, composed with its world (compose_file()) and
// instantiated with the physics records the files cannot hold yet
// (builtin_records.hpp), at the step the files assume. A many-world viewer
// scene (bounce) is its lane files side by side: one run per lane, drawn
// together, as the viewer shows them. The lanes' worlds share their ground,
// so the first lane's world is the one drawn.
//
// The live smoke's scene main functions drive it (live_tour.cpp); the
// measurements their checks make are display-free (scene_checks.hpp).

#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "builtin_records.hpp"
#include "core/error.hpp"
#include "orbit_camera.hpp"
#include "render/scene.hpp"
#include "scene/compose.hpp"
#include "scene_sessions.hpp"

namespace spade::sandbox {

class GlTargetSink;

class SceneFileSession {
  public:
    struct Lane {
        std::filesystem::path scene_file;
        BuiltinRecords records;
    };

    // Composes and instantiates every lane, and binds the meshes that draw
    // them. Needs no window, so a scene that will not compose refuses before
    // one opens.
    [[nodiscard]] static spade::Result<std::unique_ptr<SceneFileSession>> create(std::vector<Lane> lanes);

    // Starts the GPU renderer in `sink`'s context, or announces the CPU
    // fallback. `sink` must outlive this. Call once, before the first frame.
    void attach(GlTargetSink& sink);

    ~SceneFileSession();
    SceneFileSession(const SceneFileSession&) = delete;
    SceneFileSession& operator=(const SceneFileSession&) = delete;

    // One frame: the camera, `dt` seconds of stepping in whole built-in steps
    // (the remainder carries to the next frame), then a frame drawn and
    // presented.
    [[nodiscard]] spade::Result<void> frame(const FrameInput& in, float dt);

    [[nodiscard]] std::size_t lane_count() const noexcept { return runs_.size(); }
    [[nodiscard]] std::size_t vehicle_count(std::size_t lane) const noexcept { return runs_[lane].run.vehicles.size(); }
    // A vehicle's centre of mass and orientation, world frame.
    [[nodiscard]] spade::Result<glm::vec3> position(std::size_t lane, std::size_t vehicle) const;
    [[nodiscard]] spade::Result<glm::quat> orientation(std::size_t lane, std::size_t vehicle) const;
    [[nodiscard]] const BuiltinRecords& records(std::size_t lane) const noexcept { return runs_[lane].records; }
    [[nodiscard]] uint64_t steps() const noexcept { return steps_; }
    [[nodiscard]] OrbitCamera& camera() noexcept { return camera_; }
    [[nodiscard]] bool gpu_active() const noexcept { return gpu_ != nullptr; }

  private:
    SceneFileSession() = default;
    void draw_items();

    struct Run {
        spade::scene::ComposedScene composed;
        spade::scene::SceneRun run;
        BuiltinRecords records;
    };

    GlTargetSink* sink_ = nullptr;
    // Declared before the scene, so they are destroyed after it: the scene
    // borrows the first lane's composed world's SDF.
    std::vector<Run> runs_;
    spade::render::RenderScene scene_;
    spade::render::RenderOptions options_;
    uint32_t box_mesh_ = 0;
    uint32_t sphere_mesh_ = 0;
    uint32_t disc_mesh_ = 0;
    uint32_t body_material_ = 0;    // a drone's body and arms
    uint32_t nose_material_ = 0;    // its front arm and rotor
    uint32_t rotor_material_ = 0;
    uint32_t ball_material_ = 0;    // first of one per lane
    std::unique_ptr<GpuRenderer> gpu_;
    OrbitCamera camera_;
    std::vector<uint8_t> pixels_;
    uint64_t carry_ns_ = 0;
    uint64_t steps_ = 0;
};

}  // namespace spade::sandbox
