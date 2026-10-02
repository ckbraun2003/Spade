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
// WHY THE DRONE'S PARTS ARE MESHES AT THEIR REAL SIZE. Both shading paths
// transform normals by mat3(model) with no inverse-transpose, so a lit part
// must be placed with rotation and translation only. Only the heatmap cells,
// which are unlit, are scaled.
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
#include "vehicles/rotor_wake.hpp"
#include "world/medium.hpp"

#include "builder_scene.hpp"  // make_box_mesh / make_cylinder_mesh / detail::push_tri
#include "drone_sim.hpp"

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

inline constexpr uint32_t kHeatmapBins = 32;
inline constexpr uint32_t kSliceCells = 64;

// Viridis, piecewise-linear over its nine standard control points
// (#440154 #482878 #3E4A89 #31688E #26828E #1F9E89 #35B779 #6DCD59 #FDE725).
// sRGB bytes / 255, used as linear colour: the renderer has no transfer
// function, so these are the bytes that reach the screen.
[[nodiscard]] inline glm::vec3 viridis(float t) {
    static constexpr std::array<std::array<float, 3>, 9> kStops{{
        {68.0f, 1.0f, 84.0f}, {72.0f, 40.0f, 120.0f}, {62.0f, 74.0f, 137.0f},
        {49.0f, 104.0f, 142.0f}, {38.0f, 130.0f, 142.0f}, {31.0f, 158.0f, 137.0f},
        {53.0f, 183.0f, 121.0f}, {109.0f, 205.0f, 89.0f}, {253.0f, 231.0f, 37.0f},
    }};
    const float x = std::clamp(t, 0.0f, 1.0f) * static_cast<float>(kStops.size() - 1);
    const auto i = std::min(static_cast<std::size_t>(x), kStops.size() - 2);
    const float u = x - static_cast<float>(i);
    const auto& a = kStops[i];
    const auto& b = kStops[i + 1];
    return glm::vec3(a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u) / 255.0f;
}

// 0..kHeatmapBins-1. Total: a non-positive or non-finite maximum (rotors
// stopped in still air, the first frame) is bin 0, never a division by zero.
[[nodiscard]] inline uint32_t speed_bin(float speed, float max_speed) noexcept {
    if (!(max_speed > 0.0f) || !(speed > 0.0f)) return 0u;
    const float x = speed / max_speed * static_cast<float>(kHeatmapBins);
    if (!(x < static_cast<float>(kHeatmapBins - 1))) return kHeatmapBins - 1;
    return static_cast<uint32_t>(x);
}

// The colour bin b is drawn in: the centre of its span.
[[nodiscard]] inline glm::vec3 heatmap_color(uint32_t bin) {
    return viridis((static_cast<float>(bin) + 0.5f) / static_cast<float>(kHeatmapBins));
}

// ---------------------------------------------------------------------------
// Binding into a RenderScene
// ---------------------------------------------------------------------------

// Indices into a RenderScene, appended once by bind_drone_scene.
struct DroneDrawBinding {
    uint32_t body_mesh = 0, arm_mesh = 0, rotor_mesh = 0, quad_mesh = 0;
    uint32_t body_material = 0, arm_material = 0, nose_material = 0, rotor_material = 0;
    uint32_t heatmap_base = 0;  // first of kHeatmapBins unlit materials
};

// Real-size part dimensions, metres. The airframe's own numbers (arm length,
// rotor radius) come from QuadrotorParams; these are the visual ones it has
// no field for.
inline constexpr glm::vec3 kDroneBodyHalf{0.06f, 0.025f, 0.06f};
inline constexpr float kDroneArmHalfSection = 0.01f;
inline constexpr float kDroneRotorHalfThickness = 0.005f;
inline constexpr float kDroneRotorLift = kDroneArmHalfSection + kDroneRotorHalfThickness;  // disc sits on the arm

namespace detail {

// A unit quad in the XY plane, facing both ways: the CPU shaded path culls
// back faces, and the slice is seen from whichever side the camera is on.
[[nodiscard]] inline render::MeshData make_double_sided_quad() {
    render::MeshData m;
    const glm::vec3 a(-0.5f, -0.5f, 0.0f), b(0.5f, -0.5f, 0.0f), c(0.5f, 0.5f, 0.0f), d(-0.5f, 0.5f, 0.0f);
    push_tri(m, a, b, c);
    push_tri(m, a, c, d);
    push_tri(m, a, c, b);
    push_tri(m, a, d, c);
    finish(m);
    return m;
}

[[nodiscard]] inline render::Material lambert(float r, float g, float b) {
    render::Material m;
    m.base_color = glm::vec4(r, g, b, 1.0f);
    m.shading = 0u;
    return m;
}

}  // namespace detail

// Appends the drone's meshes (at `params`' real dimensions) and materials, and
// the kHeatmapBins unlit palette materials. Call ONCE per scene.
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
    b.quad_mesh = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(detail::make_double_sided_quad());

    b.body_material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(detail::lambert(0.18f, 0.18f, 0.20f));
    b.arm_material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(detail::lambert(0.55f, 0.55f, 0.58f));
    b.nose_material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(detail::lambert(0.85f, 0.20f, 0.15f));
    b.rotor_material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(detail::lambert(0.15f, 0.55f, 0.85f));

    b.heatmap_base = static_cast<uint32_t>(scene.materials.size());
    for (uint32_t i = 0; i < kHeatmapBins; ++i) {
        render::Material m;
        m.base_color = glm::vec4(heatmap_color(i), 1.0f);
        m.shading = 1u;  // unlit: the colour IS the datum
        scene.materials.push_back(m);
    }
    return b;
}

// The drone as nine draw items: body, four arms (the +X nose arm in its own
// colour) and four rotor discs. Rotation and translation only.
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
        disc.material_override = b.rotor_material;
        out.push_back(disc);
    }
}

// ---------------------------------------------------------------------------
// The heatmap slice
// ---------------------------------------------------------------------------

// A vertical plane through the drone that turns to face the camera
// horizontally. 2 m wide and 2.5 m tall, reaching 0.75 m above the drone and
// 1.75 m below it, because the downwash is what there is to see.
struct SliceSpec {
    glm::vec3 center{0.0f};
    glm::vec3 right{1.0f, 0.0f, 0.0f};
    glm::vec3 up{0.0f, 1.0f, 0.0f};
    float width = 2.0f;
    float height = 2.5f;
};

[[nodiscard]] inline SliceSpec camera_facing_slice(const render::Camera& camera, const glm::vec3& drone_pos) {
    SliceSpec s;
    glm::vec3 to_cam = camera.position - drone_pos;
    to_cam.y = 0.0f;
    if (!(glm::length(to_cam) > 1e-6f)) to_cam = glm::vec3(0.0f, 0.0f, 1.0f);  // looking straight down
    to_cam = glm::normalize(to_cam);
    s.up = glm::vec3(0.0f, 1.0f, 0.0f);
    s.right = glm::normalize(glm::cross(s.up, to_cam));
    s.center = drone_pos + glm::vec3(0.0f, -0.5f, 0.0f);
    return s;
}

// The centre of cell (i, j), i across and j up, in world space.
[[nodiscard]] inline glm::vec3 slice_cell_center(const SliceSpec& s, uint32_t i, uint32_t j) {
    const float n = static_cast<float>(kSliceCells);
    const float u = (static_cast<float>(i) + 0.5f) / n - 0.5f;
    const float v = (static_cast<float>(j) + 0.5f) / n - 0.5f;
    return s.center + s.right * (u * s.width) + s.up * (v * s.height);
}

// Appends kSliceCells^2 cells. Speeds are sampled first, then binned, so the
// automatic range is THIS frame's maximum. `manual_max > 0` fixes the range
// instead. Returns the largest speed sampled (the legend's top in auto mode).
inline float append_slice_items(const DroneDrawBinding& b, const SliceSpec& s, const AirField& field,
                                float manual_max, std::vector<render::DrawItem>& out) {
    std::array<float, kSliceCells * kSliceCells> speed{};
    float observed = 0.0f;
    for (uint32_t j = 0; j < kSliceCells; ++j) {
        for (uint32_t i = 0; i < kSliceCells; ++i) {
            const float v = glm::length(field.velocity(slice_cell_center(s, i, j)));
            speed[j * kSliceCells + i] = v;
            if (v > observed) observed = v;
        }
    }
    const float range = manual_max > 0.0f ? manual_max : observed;

    const glm::vec3 normal = glm::cross(s.right, s.up);
    const glm::mat4 basis(glm::vec4(s.right, 0.0f), glm::vec4(s.up, 0.0f), glm::vec4(normal, 0.0f),
                          glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    const glm::mat4 cell_scale = glm::scale(glm::mat4(1.0f), glm::vec3(s.width / static_cast<float>(kSliceCells),
                                                                       s.height / static_cast<float>(kSliceCells), 1.0f));
    out.reserve(out.size() + kSliceCells * kSliceCells);
    for (uint32_t j = 0; j < kSliceCells; ++j) {
        for (uint32_t i = 0; i < kSliceCells; ++i) {
            render::DrawItem cell;
            cell.mesh_index = b.quad_mesh;
            cell.local_to_world = glm::translate(glm::mat4(1.0f), slice_cell_center(s, i, j)) * basis * cell_scale;
            cell.material_override = b.heatmap_base + speed_bin(speed[j * kSliceCells + i], range);
            out.push_back(cell);
        }
    }
    return observed;
}

}  // namespace spade::sandbox
