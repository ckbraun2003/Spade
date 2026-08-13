#pragma once

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace spade {

// ---------------------------------------------------------------------------
// finite() -- componentwise std::isfinite, one overload set for every scalar/
// vector/quaternion/matrix type the engine validates on entry (WorldDesc,
// ModelType::validate(), SdfProgram parameter checks, snapshot load, ...).
// Consolidates five previously file-local, semantically-identical copies
// (world/sdf.cpp, world/builder.cpp, sim/world_set.cpp, sim/simulation.cpp,
// vehicles/model_type.cpp) into one place.
//
// VALIDATION-PATH ONLY -- these run once per registration/load, never on the
// per-substep hot path, so they favor a single obvious per-component form
// over anything branch-clever.
// ---------------------------------------------------------------------------

[[nodiscard]] inline bool finite(float v) noexcept { return std::isfinite(v); }

[[nodiscard]] inline bool finite(const glm::vec3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

[[nodiscard]] inline bool finite(const glm::vec4& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) && std::isfinite(v.w);
}

[[nodiscard]] inline bool finite(const glm::quat& q) noexcept {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

[[nodiscard]] inline bool finite(const glm::mat4& m) noexcept {
    return finite(m[0]) && finite(m[1]) && finite(m[2]) && finite(m[3]);
}

}  // namespace spade
