// kinematic_mover.hpp -- the first declared behavior (24th spec SL6, Plan A
// Task 9).
//
// Moves one body around a circle. It is the mechanism the demo-mission program
// and the `tracking-moving-target` training structure are both blocked on, and
// it supplies the ENGINE CAPABILITY only -- it does not adopt DM3's
// mover/Target schema, which stays that addendum's to define.
//
// THE POSE IS A CLOSED-FORM FUNCTION OF (params, tick). Never an incremental
// integration, and the distinction is the whole design:
//
//   * a restored snapshot lands exactly where the original run was, because
//     the pose is recomputed from the tick rather than carried in accumulated
//     state the snapshot would have to know about -- which is what keeps this
//     behavior compatible with SL3's "the object graph is not registered
//     state";
//   * stepping 400 times and stepping 4 x 100 give bit-identical results,
//     because nothing accumulates across calls;
//   * it is idempotent within a step. Tick counts STEPS, so every substep of
//     step k computes the same pose and writes it again. Collision, which runs
//     after this slot, therefore sees a stable pose for the whole step.
//
// sin32/cos32, NOT std::sin/std::cos: core/fp32_math.hpp is the engine's
// bit-portability contract, and std::sin's result is not portable across
// platforms or libm versions. A behavior that used it would move parity from
// "measured" to "hoped for".

#pragma once

#include <cstdint>

#include <glm/glm.hpp>

#include "objects/behavior.hpp"

namespace spade::objects {

struct KinematicMoverParams {
    glm::vec3 origin{0.0f};              // circle centre, world frame
    glm::vec3 axis{0.0f, 1.0f, 0.0f};    // circle's normal; need not be unit
    float radius_m = 1.0f;
    float period_s = 1.0f;               // seconds for one revolution

    // WHICH BODY. `body_slot` is an index into the WORLD'S body slice, not a
    // global arena slot -- the same convention DragBodyRow::body_slot uses, so
    // the two cannot disagree about what a slot number means.
    //
    // The binding lives in the params rather than coming from the object graph
    // because the user ruled the object model "designed fully, built
    // minimally". Reaching a body through BehaviorComponent + BodyComponent is
    // the designed path; it needs the ObjectGraph in SubstepContext, which
    // nothing yet requires. One registration moves one body; several movers
    // are several registrations, which registration-order execution already
    // makes deterministic.
    uint32_t world_index = 0;
    uint32_t body_slot = 0;
};

// The desc binds `params` BY POINTER (BehaviorDesc::user_data), so the caller
// keeps it alive for as long as the registry is attached, and must not mutate
// it during a step.
//
// Registered as BehaviorSlot::kinematic -- before ForceElements and both
// collision passes, which is what makes a moved pose visible to collision on
// the same substep rather than one late.
[[nodiscard]] BehaviorDesc kinematic_mover_desc(const KinematicMoverParams& params) noexcept;

}  // namespace spade::objects
