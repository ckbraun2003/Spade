// The drone sim box's view: the air around the drone as a field, the field as
// a heatmap slice, and the drone itself as parts.
//
// THE FIELD IS ANALYTIC AND READ ONLY FOR DRAWING; STEPPING NEVER READS IT.
//     v(p) = medium(p) + sum over rotors of wake_i(p)
// medium(p) is the engine's own sample (Simulation::sample_medium: wind plus
// the current Dryden gust), and wake_i is the actuator-disc slipstream in
// vehicles/rotor_wake.hpp, fed from each rotor's live row. There is no
// obstruction by the frame -- honest for what the engine models.
//
// WHY THE DRONE'S PARTS ARE MESHES AT THEIR REAL SIZE. Placed with rotation
// and translation only, every lit part has a conformal frame. Since 1cac77a,
// transform_normal() (render/scene.hpp) and the GL vertex shader use the
// cofactor for a non-conformal frame, so a scaled lit part would also shade
// correctly; conformal frames keep the arithmetic the frame goldens pin. Only
// the heatmap cells, which are unlit, are scaled.
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
#include "render/field_layer.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "vehicles/quadrotor.hpp"
#include "vehicles/rotor.hpp"
#include "vehicles/rotor_wake.hpp"
#include "world/medium.hpp"

#include "builder_scene.hpp"  // make_box_mesh / make_cylinder_mesh
#include "drone_sim.hpp"
#include "orbit_camera.hpp"   // FrameInput

namespace spade::sandbox {

// ---------------------------------------------------------------------------
// The air field
// ---------------------------------------------------------------------------

struct AirField {
    MediumSample medium{};  // position-independent today (Dryden is a point model)
    std::array<vehicles::RotorWakeInput, vehicles::kQuadrotorRotorCount> rotors{};

    [[nodiscard]] glm::vec3 velocity(glm::vec3 p) const noexcept {
        glm::vec3 v = medium.wind;
        for (const auto& r : rotors) v += vehicles::rotor_wake_velocity(r, p);
        return v;
    }
};

[[nodiscard]] inline Result<AirField> air_field_from(const DroneSim& drone) {
    const Simulation& sim = drone.sim();
    auto body = sim.body(drone.vehicle().body);
    if (!body) return std::unexpected(body.error());
    const glm::vec3 pos = (*body)->pos;
    const glm::quat orient = (*body)->orient;

    AirField f;
    auto medium = sim.sample_medium(0, pos);
    if (!medium) return std::unexpected(medium.error());
    f.medium = *medium;

    for (uint32_t i = 0; i < vehicles::kQuadrotorRotorCount; ++i) {
        auto row = sim.rotor(drone.vehicle(), i);
        if (!row) return std::unexpected(row.error());
        vehicles::RotorWakeInput& in = f.rotors[i];
        in.hub_world = pos + orient * (*row)->local_pos;
        in.thrust_axis_world = orient * ((*row)->local_orient * vehicles::kRotorLocalThrustAxis);
        in.radius = (*row)->radius;
        in.thrust_coeff = (*row)->thrust_coeff;
        in.omega = (*row)->omega;
        in.density = f.medium.density;
        in.freestream = f.medium.wind;
    }
    return f;
}

// ---------------------------------------------------------------------------
// The colour scale
// ---------------------------------------------------------------------------

// The palette and the binning live in render/field_layer.hpp now (the field
// channel, Rendering's plan step 1). They moved from here with their
// arithmetic unchanged. This scene only chooses the map: viridis in 32 bins
// from 0 m/s, topped by the manual range or, when there is none, by this
// frame's fastest cell.
inline constexpr uint32_t kHeatmapBins = 32;
inline constexpr uint32_t kSliceCells = 64;

[[nodiscard]] inline render::FieldColourMap heatmap_colour_map(float manual_max) {
    render::FieldColourMap map;
    map.palette = render::viridis_palette();
    map.bins = kHeatmapBins;
    map.range_min = 0.0f;
    map.range_max = manual_max > 0.0f ? manual_max : 0.0f;  // 0: the layer's own maximum
    return map;
}

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
// Call ONCE per scene. The heatmap needs neither: it is a field layer.
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
// The heatmap slice
// ---------------------------------------------------------------------------

// A plane through the drone ALIGNED WITH THE BODY: it holds the thrust axis
// and one arm axis, so a rotor pair and both their plumes lie in it at any
// attitude and from any camera azimuth. (A vertical camera-facing plane
// contains the hubs only near an arm's axis: at 45 degrees it misses them by
// 1.06 R while the far wake contracts to 0.71 R, and the downwash vanished
// every 90 degrees of orbit -- Physics' review; the lead's ruling.) Of the two
// pairs, the one whose plane faces the camera more squarely is shown. 2 m
// across and 2.5 m along the thrust axis, reaching 0.75 m above the drone and
// 1.75 m below it, because the downwash is what there is to see.
//
// THE ONE VIEW IT CANNOT SERVE: looking straight down the thrust axis, every
// plane that contains it is edge-on, so the slice thins to a line. That is
// inherent to showing the plumes in-plane, not a defect; orbit off the axis.
struct SliceSpec {
    glm::vec3 center{0.0f};
    glm::vec3 right{1.0f, 0.0f, 0.0f};  // an arm axis
    glm::vec3 up{0.0f, 1.0f, 0.0f};     // the thrust axis
    float width = 2.0f;
    float height = 2.5f;
};

[[nodiscard]] inline SliceSpec camera_facing_slice(const render::Camera& camera, const glm::vec3& drone_pos,
                                                   const glm::quat& drone_orient) {
    SliceSpec s;
    const glm::vec3 thrust = drone_orient * glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 arm_x = drone_orient * glm::vec3(1.0f, 0.0f, 0.0f);  // rotors 0 and 2
    const glm::vec3 arm_z = drone_orient * glm::vec3(0.0f, 0.0f, 1.0f);  // rotors 1 and 3
    glm::vec3 to_cam = camera.position - drone_pos;
    to_cam = glm::length(to_cam) > 1e-6f ? glm::normalize(to_cam) : thrust;
    // The 0/2 plane's normal is thrust x arm_x = -arm_z, and the 1/3 plane's
    // is arm_x, so "faces the camera more squarely" compares these two.
    s.right = std::fabs(glm::dot(arm_z, to_cam)) >= std::fabs(glm::dot(arm_x, to_cam)) ? arm_x : arm_z;
    s.up = thrust;
    // Seen from the camera's side, not the mirror image: the drawn normal
    // cross(right, up) points at the camera.
    if (glm::dot(glm::cross(s.right, s.up), to_cam) < 0.0f) s.right = -s.right;
    s.center = drone_pos + s.up * -0.5f;
    return s;
}

// The slice as a field layer of kSliceCells^2 air speeds, each sampled at
// render::field_cell_center(), so a cell's value is the air where the
// renderer draws that cell. The engine draws it (RenderScene::field_layers):
// unlit, double-sided and depth-tested. The samples are still this scene's
// own sum, because the slipstream is not a registered field (option (b) of
// Rendering's plan). `manual_max > 0` fixes the range; otherwise it is this
// frame's fastest cell. `observed` receives that fastest speed, the legend's
// top in auto mode.
[[nodiscard]] inline render::FieldLayer slice_layer(const SliceSpec& s, const AirField& field, float manual_max,
                                                    float* observed = nullptr) {
    render::FieldLayer layer;
    layer.center = s.center;
    layer.right = s.right;
    layer.up = s.up;
    layer.width = s.width;
    layer.height = s.height;
    layer.cells_u = kSliceCells;
    layer.cells_v = kSliceCells;
    layer.colour_map = heatmap_colour_map(manual_max);
    layer.values.resize(static_cast<std::size_t>(kSliceCells) * kSliceCells);
    float fastest = 0.0f;
    for (uint32_t j = 0; j < kSliceCells; ++j) {
        for (uint32_t i = 0; i < kSliceCells; ++i) {
            const float v = glm::length(field.velocity(render::field_cell_center(layer, i, j)));
            layer.values[static_cast<std::size_t>(j) * kSliceCells + i] = v;
            if (v > fastest) fastest = v;
        }
    }
    if (observed != nullptr) *observed = fastest;
    return layer;
}

// ---------------------------------------------------------------------------
// The panel and the keys -- the logic the window would otherwise own
// ---------------------------------------------------------------------------

// What the physics panel shows and edits. The window draws it and writes
// `edited`, `view_heatmap`, `heatmap_max` and `ui_item_active`; everything that
// decides what those edits DO is in the functions below.
struct DronePanelModel {
    DronePhysicsOptions edited{};  // the widgets' values; the simulation's are drone.options()
    bool view_heatmap = false;
    float heatmap_max = 0.0f;   // m/s; 0 = auto
    float observed_max = 0.0f;  // the last slice's maximum, for the legend
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
