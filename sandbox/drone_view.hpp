// The drone sim box's view: the drone as parts, and the panel and keys that fly
// it. (The analytic air-speed heatmap that was drawn here is retired, INT-9:
// it drew a synthetic field that could be read as flow data; the real airflow
// is LiveRun's, drawn through Rendering's flow views.)
//
// WHY THE DRONE'S PARTS ARE MESHES AT THEIR REAL SIZE. Placed with rotation
// and translation only, every lit part has a conformal frame. Since 1cac77a,
// transform_normal() (render/scene.hpp) and the GL vertex shader use the
// cofactor for a non-conformal frame, so a scaled lit part would also shade
// correctly; conformal frames keep the arithmetic the frame goldens pin.
//
// Display-free like drone_sim.hpp, so tests/test_sandbox_drone_view.cpp
// asserts it with no window (SL15b).

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "vehicles/quadrotor.hpp"
#include "vehicles/rotor.hpp"

#include "unit_meshes.hpp"  // make_box_mesh / make_cylinder_mesh
#include "drone_sim.hpp"
#include "orbit_camera.hpp"   // FrameInput

namespace spade::sandbox {

// ---------------------------------------------------------------------------
// Binding into a RenderScene
// ---------------------------------------------------------------------------

// Indices into a RenderScene, appended once by bind_drone_scene.
struct DroneDrawBinding {
    uint32_t body_mesh = 0, arm_mesh = 0, rotor_mesh = 0;
    uint32_t body_material = 0, arm_material = 0, nose_material = 0, rotor_material = 0;
};

// Real-size part dimensions, metres. The airframe's own numbers (arm length,
// rotor radius) come from QuadrotorParams; these are the visual ones it has
// no field for.
inline constexpr glm::vec3 kDroneBodyHalf{0.06f, 0.025f, 0.06f};
inline constexpr float kDroneArmHalfSection = 0.01f;
inline constexpr float kDroneRotorHalfThickness = 0.005f;
inline constexpr float kDroneRotorLift = kDroneArmHalfSection + kDroneRotorHalfThickness;  // disc sits on the arm

namespace detail {

[[nodiscard]] inline render::Material lambert(float r, float g, float b) {
    render::Material m;
    m.base_color = glm::vec4(r, g, b, 1.0f);
    m.shading = 0u;
    return m;
}

}  // namespace detail

// Appends the drone's meshes (at `params`' real dimensions) and materials.
// Call ONCE per scene.
[[nodiscard]] inline DroneDrawBinding bind_drone_scene(render::RenderScene& scene,
                                                       const vehicles::QuadrotorParams& params) {
    DroneDrawBinding b;
    b.body_mesh = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(make_box_mesh(kDroneBodyHalf));
    b.arm_mesh = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(
        make_box_mesh(glm::vec3(0.5f * params.arm_length, kDroneArmHalfSection, kDroneArmHalfSection)));
    b.rotor_mesh = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(make_cylinder_mesh(24u, params.rotors[0].radius, kDroneRotorHalfThickness));

    b.body_material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(detail::lambert(0.18f, 0.18f, 0.20f));
    b.arm_material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(detail::lambert(0.55f, 0.55f, 0.58f));
    b.nose_material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(detail::lambert(0.85f, 0.20f, 0.15f));
    b.rotor_material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(detail::lambert(0.15f, 0.55f, 0.85f));
    return b;
}

// The drone as nine draw items: body, four arms and four rotor discs.
// Rotation and translation only. The front (+X, rotor 0) arm AND disc take the
// nose colour: adjacent hubs are 0.255 m apart and the discs 0.24 m across, so
// from above the discs all but hide the arms, and an arm alone cannot carry
// the heading.
inline void append_drone_items(const DroneDrawBinding& b, const vehicles::QuadrotorParams& params,
                               const glm::vec3& pos, const glm::quat& orient,
                               std::vector<render::DrawItem>& out) {
    const glm::mat4 pose = glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(orient);

    render::DrawItem body;
    body.mesh_index = b.body_mesh;
    body.local_to_world = pose;
    body.material_override = b.body_material;
    out.push_back(body);

    for (std::size_t i = 0; i < vehicles::kQuadrotorRotorCount; ++i) {
        const glm::vec3 hub = vehicles::quadrotor_arm_offset(params, i);
        const glm::vec3 flat(hub.x, 0.0f, hub.z);
        // The arm mesh lies along +X; turn it about +Y onto this arm.
        // Rotating +X by a about +Y gives (cos a, 0, -sin a).
        const float a = std::atan2(-flat.z, flat.x);
        const glm::mat4 arm_frame = pose * glm::translate(glm::mat4(1.0f), 0.5f * flat) *
                                    glm::rotate(glm::mat4(1.0f), a, glm::vec3(0.0f, 1.0f, 0.0f));

        render::DrawItem arm;
        arm.mesh_index = b.arm_mesh;
        arm.local_to_world = arm_frame;
        arm.material_override = i == 0 ? b.nose_material : b.arm_material;
        out.push_back(arm);

        render::DrawItem disc;
        disc.mesh_index = b.rotor_mesh;
        disc.local_to_world = pose * glm::translate(glm::mat4(1.0f), hub + glm::vec3(0.0f, kDroneRotorLift, 0.0f));
        disc.material_override = i == 0 ? b.nose_material : b.rotor_material;
        out.push_back(disc);
    }
}

// ---------------------------------------------------------------------------
// The panel and the keys -- the logic the window would otherwise own
// ---------------------------------------------------------------------------

// What the physics panel shows and edits. The window draws it and writes
// `edited` and `ui_item_active`; everything that decides what those edits DO
// is in the functions below.
struct DronePanelModel {
    DronePhysicsOptions edited{};  // the widgets' values; the simulation's are drone.options()
    DroneReadouts readouts{};
    AttitudeTarget target{};     // what the keys asked for, beside what the drone did
    std::string status;          // shown in a warning colour when non-empty
    bool ui_item_active = false; // a widget is being dragged or typed into this frame
    const char* render_path = "";  // "GPU (OpenGL)" or "CPU raster"
};

inline constexpr float kAttitudeRateRadPerS = 1.5f;

// The attitude keys nudge a target that holds when released. Pitch and roll
// are clamped HERE, at the target, so holding a key past the limit does not
// wind up a target the controller then has to unwind; yaw wraps.
inline void nudge_attitude(AttitudeTarget& t, const FrameInput& in, float dt_seconds) noexcept {
    if (in.ui_captured_keyboard) return;
    const float step = kAttitudeRateRadPerS * std::max(dt_seconds, 0.0f);
    t.pitch = std::clamp(t.pitch + in.attitude_pitch * step, -kMaxTiltRad, kMaxTiltRad);
    t.roll = std::clamp(t.roll + in.attitude_roll * step, -kMaxTiltRad, kMaxTiltRad);
    constexpr float kPi = 3.14159265358979323846f;
    t.yaw += in.attitude_yaw * step;
    if (t.yaw > kPi) t.yaw -= 2.0f * kPi;
    if (t.yaw < -kPi) t.yaw += 2.0f * kPi;
    if (in.level_pressed) {
        t.pitch = 0.0f;
        t.roll = 0.0f;
    }
}

// Applies the panel's physics edits, DEBOUNCED: nothing happens while a widget
// is active (a dragged slider would otherwise rebuild the Simulation every
// frame), and nothing happens when the edits match what is running. A refused
// rebuild -- Vulkan today -- keeps the running simulation, puts the reason in
// `status`, and snaps the widgets back to what is actually running so the
// refusal is not retried every frame. Returns true when it rebuilt.
inline bool apply_panel_edits(DroneSim& drone, DronePanelModel& panel) {
    if (panel.ui_item_active || panel.edited == drone.options()) return false;
    if (auto r = drone.apply_options(panel.edited); !r) {
        panel.status = r.error().context;
        panel.edited = drone.options();
        return false;
    }
    panel.status.clear();
    return true;
}

}  // namespace spade::sandbox
