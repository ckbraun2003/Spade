// kinematic_mover.cpp -- see kinematic_mover.hpp for why the pose is a closed
// form of (params, tick) and never an integration.

#include "objects/behaviors/kinematic_mover.hpp"

#include <cmath>

#include "core/fp32_math.hpp"
#include "objects/component.hpp"
#include "physics/schedule.hpp"
#include "state/layout.hpp"

namespace spade::objects {
namespace {

constexpr float kTwoPi = 6.2831853071795864769f;

// An orthonormal pair spanning the plane the circle lives in.
//
// The reference vector is chosen by which component of `axis` is SMALLEST, so
// the cross product is never near-degenerate -- picking a fixed reference would
// collapse to a zero-length vector for an axis parallel to it, and picking by
// the largest component would choose the near-parallel one. Deterministic: the
// choice is a comparison of the same floats on every platform, not a tolerance.
[[nodiscard]] bool basis_for(const glm::vec3& axis, glm::vec3& u, glm::vec3& v) noexcept {
    const float len2 = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
    if (!(len2 > 0.0f)) return false;  // also rejects NaN, which `<= 0` would not
    const glm::vec3 n = axis / std::sqrt(len2);

    const float ax = std::fabs(n.x);
    const float ay = std::fabs(n.y);
    const float az = std::fabs(n.z);
    glm::vec3 reference(0.0f, 0.0f, 1.0f);
    if (ax <= ay && ax <= az) {
        reference = glm::vec3(1.0f, 0.0f, 0.0f);
    } else if (ay <= az) {
        reference = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    const glm::vec3 w = glm::cross(n, reference);
    const float w_len2 = w.x * w.x + w.y * w.y + w.z * w.z;
    if (!(w_len2 > 0.0f)) return false;
    u = w / std::sqrt(w_len2);
    v = glm::cross(n, u);
    return true;
}

void kinematic_mover_execute(const physics::SubstepContext& ctx, const void* params) noexcept {
    if (params == nullptr) return;
    const auto& p = *static_cast<const KinematicMoverParams*>(params);

    // A zero or negative period would divide by zero; a degenerate axis has no
    // plane. Both are inert rather than undefined -- a misconfigured mover
    // leaves its body alone, which is visible in a scene and harmless in a
    // parity run, where NaN would poison every subsequent digest.
    if (!(p.period_s > 0.0f)) return;
    glm::vec3 u{};
    glm::vec3 v{};
    if (!basis_for(p.axis, u, v)) return;

    if (p.world_index >= ctx.worlds.size()) return;
    const physics::WorldSubstepView& world = ctx.worlds[p.world_index];
    if (p.body_slot >= world.bodies.size()) return;

    // Tick counts STEPS, so this is the step's time -- dt, never the substep h.
    const float t = static_cast<float>(ctx.tick.value) * ctx.dt_s;
    const float theta = kTwoPi * (t / p.period_s);
    const float c = math::cos32(theta);
    const float s = math::sin32(theta);

    BodyState& body = world.bodies[p.body_slot];
    body.pos = p.origin + p.radius_m * (c * u + s * v);
    // A kinematic body's motion is PRESCRIBED, not integrated. Leaving stale
    // velocity would let Integrate fight the mover: it would advance the body
    // away from the circle every substep and the mover would drag it back,
    // producing a body that is neither where the mover says nor where physics
    // says.
    body.vel = glm::vec3(0.0f);
}

}  // namespace

BehaviorDesc kinematic_mover_desc(const KinematicMoverParams& params) noexcept {
    BehaviorDesc desc;
    desc.name = "kinematic_mover";
    desc.slot = BehaviorSlot::kinematic;
    // Declared sets, in ComponentTypeId bits: it reads its own configuration
    // and writes the body it names. Nothing consumes these yet -- SL6 declares
    // them so a future scheduler can check for conflicts without re-deriving
    // them from the code.
    desc.reads = 1u << static_cast<uint32_t>(ComponentTypeId::transform);
    desc.writes = 1u << static_cast<uint32_t>(ComponentTypeId::body);
    desc.execute_cpu = &kinematic_mover_execute;
    // No record_gpu: this is CPU-only for now, so a registry holding it is
    // GPU-ineligible by SL6 -- a refusal, deliberately, rather than a world
    // that would run differently on the two backends.
    desc.record_gpu = nullptr;
    desc.user_data = &params;
    return desc;
}

}  // namespace spade::objects
