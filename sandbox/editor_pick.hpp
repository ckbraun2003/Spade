// What a click in the view selects. Display-free (SL15b).
//
// The click is a ray from the camera (the caller builds it from the cursor,
// measured from the view's corner). It meets two kinds of thing:
// - the composed world's SDF: the world's terrain and every asset's collider.
//   A hit belongs to the asset whose posed collider is at the hit point; a hit
//   on the world's own terrain belongs to nothing, so clicking the ground
//   clears the selection;
// - each vehicle, as a sphere of its reach: its proxy radius, or its rotors'
//   reach (hub distance plus disc radius) when that is larger, so a click on
//   an arm picks the drone. The plan named the proxy radius alone, which is
//   often 0 ("use the world's default") and never covers the rotors.
// The nearer of the two wins. While the scene runs, a vehicle is where the
// run has it; otherwise it is at its start.
//
// docs/design/interface/plans/2026-10-05-editor-plan.md Task 10.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "editor_run.hpp"
#include "scene/compose.hpp"
#include "scene/scene_file.hpp"
#include "vehicles/model_type.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace spade::sandbox::editor {

inline constexpr float kPickMaxDistance = 1000.0f;  // m: past this, the sky
inline constexpr int kPickMaxSteps = 512;
inline constexpr float kPickDefaultReach = 0.25f;   // m, for a vehicle with no proxy and no rotors

namespace detail {

// The nearest distance along the ray at which it meets the program's surface,
// or none. Each step is 0.9 of the distance: a heightfield or a smooth union
// is a bound, not an exact distance, and overstepping one tunnels through.
[[nodiscard]] inline std::optional<float> march(const SdfProgram& program, glm::vec3 origin, glm::vec3 dir) {
    if (program.empty()) return std::nullopt;
    float t = 0.0f;
    for (int i = 0; i < kPickMaxSteps && t < kPickMaxDistance; ++i) {
        const float d = eval(program, origin + dir * t);
        if (!std::isfinite(d)) return std::nullopt;
        if (d < 1e-3f * std::max(1.0f, t)) return t;
        t += 0.9f * d;
    }
    return std::nullopt;
}

// The asset's collider placed by its pose, as compose() places it.
[[nodiscard]] inline std::optional<SdfProgram> posed_collider(const scene::SceneAsset& asset) {
    if (asset.collider.empty()) return std::nullopt;
    const Result<SdfTransform> pose = transform_of(asset.pose);
    if (!pose) return std::nullopt;
    SdfProgram p;
    p.nodes = asset.collider.nodes;
    p.transforms.reserve(asset.collider.transforms.size());
    for (const SdfTransform& t : asset.collider.transforms) p.transforms.push_back(scene::compose_transform(t, *pose));
    return p;
}

[[nodiscard]] inline float reach_of(const vehicles::ModelType& model) {
    const float r = vehicles::model_reach(model);  // Core's one rule (TD-9)
    return r > 0.0f ? r : kPickDefaultReach;
}

// The nearest t >= 0 at which the ray enters (or, from inside, leaves) the sphere.
[[nodiscard]] inline std::optional<float> ray_sphere(glm::vec3 origin, glm::vec3 dir, glm::vec3 centre, float r) {
    const glm::vec3 oc = origin - centre;
    const float b = glm::dot(oc, dir);
    const float disc = b * b - (glm::dot(oc, oc) - r * r);
    if (disc < 0.0f) return std::nullopt;
    const float s = std::sqrt(disc);
    float t = -b - s;
    if (t < 0.0f) t = -b + s;
    if (t < 0.0f) return std::nullopt;
    return t;
}

}  // namespace detail

// The name of the asset or vehicle under the ray, or none. `run` is the
// running scene, or null when it is not running. `direction` need not be unit
// length; a zero or non-finite ray picks nothing.
[[nodiscard]] inline std::optional<std::string> pick(const scene::SceneDesc& scene,
                                                     const scene::ComposedScene& composed, const EditorRun* run,
                                                     glm::vec3 origin, glm::vec3 direction) {
    const float length = glm::length(direction);
    if (!(length > 0.0f) || !std::isfinite(length) || !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
        !std::isfinite(origin.z)) {
        return std::nullopt;
    }
    const glm::vec3 dir = direction / length;

    std::optional<std::string> best;
    float best_t = std::numeric_limits<float>::infinity();
    for (const scene::SceneVehicle& v : scene.vehicles) {
        const auto model = std::find_if(scene.models.begin(), scene.models.end(),
                                        [&](const vehicles::ModelType& m) { return m.name == v.model; });
        if (model == scene.models.end()) continue;
        glm::vec3 at = v.start.pos;
        if (run != nullptr) {
            if (const std::optional<FrameState> now = run->state_of(v.name)) at = now->pos;
        }
        if (const std::optional<float> t = detail::ray_sphere(origin, dir, at, detail::reach_of(*model));
            t && *t < best_t) {
            best = v.name;
            best_t = *t;
        }
    }

    if (const std::optional<float> t = detail::march(composed.world.sdf, origin, dir); t && *t < best_t) {
        const glm::vec3 hit = origin + dir * *t;
        float nearest = std::max(0.02f, 2e-3f * *t);  // how close a collider must be to own the hit
        std::optional<std::string> owner;
        for (const scene::SceneAsset& a : scene.assets) {
            const std::optional<SdfProgram> posed = detail::posed_collider(a);
            if (!posed) continue;
            const float d = std::abs(eval(*posed, hit));
            if (d <= nearest) {
                nearest = d;
                owner = a.name;
            }
        }
        return owner;  // none: the world's own terrain
    }
    return best;
}

}  // namespace spade::sandbox::editor
