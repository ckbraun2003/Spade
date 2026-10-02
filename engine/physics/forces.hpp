#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "state/layout.hpp"
#include "world/medium.hpp"

// ---------------------------------------------------------------------------
// Force elements -- the ForceElements pass (engine design spec S3 "the step
// model": MediumUpdate -> ForceElements -> Gravity -> CollisionStatic ->
// CollisionDynamic -> Integrate -> SensorSynthesis -> Publish; see
// physics/integrator.hpp's header for why Gravity does not itself touch
// force_acc). This header is the first of what will be several element KINDS
// -- rotors are Task 17; contacts live in physics/contacts.* per the
// coordinator's split of this same target's source list. DragBody is the
// only kind this task ships (YAGNI).
//
// ELEMENT TABLES ARE REGISTERED, WORLD-PARTITIONED STATE, the same shape as
// the bodies array itself: one ArenaSet array per element kind
// (`arenas.register_array<DragBodyRow>("drag_bodies", capacity)`), so it is
// snapshot-covered automatically (state/registry.hpp's "no unregistered
// state" invariant) and so a GPU dispatch will one day be able to bound a
// world's element partition the same way it bounds a world's body partition
// (engine design spec S4 "Many-worlds", D8).
//
// Each row carries a COMMON header -- which body it acts on, its offset from
// that body's COM, whether it currently participates -- plus its
// kind-specific parameters, per the brief and the coordinator's resolution.
//
// FRAMES, following layout.hpp's BodyState ruling exactly (see that header's
// "THE TWO ACCUMULATORS DO NOT SHARE A FRAME" note): a force-element pass
// must hand Integrate a WORLD-frame force_acc contribution and a BODY-frame
// torque_acc contribution. `local_pos` -- the offset r used for the torque --
// is therefore BODY frame, matching every other body-relative quantity in
// BodyState.
// ---------------------------------------------------------------------------

namespace spade::physics {

// DragBodyRow::mode values. A plain uint32_t rather than an enum class member
// -- the same choice physics/integrator.hpp makes for BodyState::flags --
// because the field's BYTES are what a snapshot blob and (from S6) a Slang
// buffer see; the enum-like names here are sugar for call sites, not the
// wire type.
namespace drag_mode {

// F = -1/2 * rho * Cd * A * |v_rel| * v_rel, WORLD frame. Isotropic: the law
// scales by the RELATIVE-VELOCITY MAGNITUDE, not a per-axis magnitude, so a
// single Cd (DragBodyRow::coeffs.x) is all it uses.
inline constexpr uint32_t quadratic = 0u;

// F_i = -c_i * |v_rel_i| * v_rel_i per BODY axis, i.e. DragBodyRow::coeffs
// applied componentwise to the body-frame relative velocity: per-axis
// quadratic drag, the usual small-multirotor form, whose axes do not couple.
// See apply_drag()'s comment in forces.cpp for the frame round trip.
inline constexpr uint32_t componentwise = 1u;

}  // namespace drag_mode

// ---------------------------------------------------------------------------
// DragBodyRow -- one drag element. One row of the world-partitioned
// "drag_bodies" array registered via ArenaSet::register_array<DragBodyRow>.
//
// Four 16-byte rows, following state/layout.hpp's std430 discipline. This
// struct does NOT live in layout.hpp -- that file is frozen by Task 6's
// static_asserts, per the coordinator's resolution for this task -- but the
// same contract applies here: it is a POD state row a future S6 Slang buffer
// must agree with byte-for-byte, so its size/alignment/offsets are
// static_asserted the same way layout.hpp asserts BodyState/WorldParams.
//
//   row 0  body_slot | enabled | mode | area
//   row 1  local_pos | _p0
//   row 2  local_orient
//   row 3  coeffs | _p1
//
// `body_slot` indexes into the SAME world's `bodies` span apply_drag() is
// handed -- i.e. it is local to that per-world slice (what
// ArenaSet::world_slice returns for the bodies array), NOT a global arena
// slot. Both the bodies array and this array are world-partitioned the same
// way (one partition per world, same world count), so a caller that builds
// both spans from the same world id keeps the indices in agreement;
// apply_drag() itself never consults ArenaSet or a world id, matching
// physics/integrator.hpp's integrate_bodies() taking a bare body span.
//
// `local_pos` is r in the torque formula `torque_acc += cross(r, F_body)`
// (BODY frame, m, offset from the body's COM). `local_orient` (local frame ->
// body frame) rides along as a genuinely common force-element field -- a
// future rotor element (Task 17) needs an orientation to know which way its
// thrust points -- but DragBody's own force law does not consult it: drag is
// evaluated directly against the body's own axes (componentwise mode) or
// against world-frame relative velocity (quadratic mode), never against a
// further-rotated local frame. Every test not specifically exercising torque
// leaves it identity.
//
// `enabled` mirrors physics/integrator.hpp's body_flags::active convention:
// active-high (0 == inert), a plain uint32_t rather than a bool so a
// zero-filled (freed or never-allocated) arena slot is inert by construction
// -- the same zero-fill guarantee state/arenas.hpp gives every registered
// array.
//
// `mode`/`coeffs`/`area` are the drag law's own parameters -- see
// apply_drag()'s doc comment in forces.cpp for the formulas and frames.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) DragBodyRow {
    uint32_t body_slot;      // index into the world's `bodies` span, see above
    uint32_t enabled;        // active-high; 0 = inert (skipped, same posture as a freed slot)
    uint32_t mode;           // drag_mode::quadratic or drag_mode::componentwise
    float area;              // m^2, frontal/reference area -- quadratic mode only
    glm::vec3 local_pos;     // r, BODY-frame offset from the body's COM, m
    float _p0;                // std430 pad -- keeps `local_orient` on row 2
    glm::quat local_orient;  // local->body rotation; unused by DragBody's own force law (see above)
    glm::vec3 coeffs;        // quadratic: coeffs.x is Cd (dimensionless), y/z unused (see drag_mode::quadratic)
                              // componentwise: per-body-axis coefficient, kg/m (see drag_mode::componentwise)
    float _p1;                // std430 pad
};

static_assert(std::is_standard_layout_v<DragBodyRow>, "DragBodyRow must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<DragBodyRow>, "DragBodyRow must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<DragBodyRow>, "arena slots are never individually destroyed");
static_assert(alignof(DragBodyRow) == 16, "std430 base alignment");
static_assert(sizeof(DragBodyRow) == 64, "std430 array stride");

static_assert(offsetof(DragBodyRow, body_slot) == 0);
static_assert(offsetof(DragBodyRow, enabled) == 4);
static_assert(offsetof(DragBodyRow, mode) == 8);
static_assert(offsetof(DragBodyRow, area) == 12);
static_assert(offsetof(DragBodyRow, local_pos) == 16);
static_assert(offsetof(DragBodyRow, _p0) == 28);
static_assert(offsetof(DragBodyRow, local_orient) == 32);
static_assert(offsetof(DragBodyRow, coeffs) == 48);
static_assert(offsetof(DragBodyRow, _p1) == 60);

// Every vec3/quat starts a 16-byte row.
static_assert(offsetof(DragBodyRow, local_pos) % 16 == 0);
static_assert(offsetof(DragBodyRow, local_orient) % 16 == 0);
static_assert(offsetof(DragBodyRow, coeffs) % 16 == 0);

// Named fields account for every byte: no implicit padding, same discipline
// as layout.hpp's BodyState/WorldParams asserts.
static_assert(sizeof(DragBodyRow::body_slot) + sizeof(DragBodyRow::enabled) + sizeof(DragBodyRow::mode) +
                  sizeof(DragBodyRow::area) + sizeof(DragBodyRow::local_pos) + sizeof(DragBodyRow::_p0) +
                  sizeof(DragBodyRow::local_orient) + sizeof(DragBodyRow::coeffs) + sizeof(DragBodyRow::_p1) ==
              sizeof(DragBodyRow),
              "DragBodyRow has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// apply_drag() -- the ForceElements pass's DragBody contribution. For every
// enabled element whose body is active, accumulates a drag force and torque
// into that body's force_acc (WORLD frame) / torque_acc (BODY frame).
//
// `bodies` and `elems` must be spans over the SAME world's partitions (see
// DragBodyRow::body_slot above); `medium` and `params` are that world's
// Medium and WorldParams row, sampled once per element at the body's current
// position (engine design D6: Medium is the one seam every aerodynamic force
// reads the air through).
//
// `h` is UNUSED: this is pure per-substep force accumulation, and neither
// drag law depends on the substep length. It is kept in the signature for
// uniformity with the ForceElements pass's other element kinds (per the
// coordinator's resolution) -- a future kind with its own internal dynamics
// (e.g. motor spin-up lag) will need it even though DragBody does not.
//
// Skips disabled elements (`enabled == 0`) and elements whose body is not
// body_flags::active (physics/integrator.hpp), same posture as
// integrate_bodies() skipping inactive bodies. Allocates nothing; safe to
// call every substep from a hot loop.
//
// PRECONDITION: every `elems[i].body_slot` is a valid index into `bodies`
// (same posture as integrate_bodies()'s undocumented-but-required
// preconditions on mass/orient -- not runtime-checked, so a caller that
// builds both spans from the same world's partitions satisfies it for free).
// ---------------------------------------------------------------------------
void apply_drag(std::span<BodyState> bodies, std::span<const DragBodyRow> elems, const Medium& medium,
                 const WorldParams& params, float h) noexcept;

}  // namespace spade::physics
