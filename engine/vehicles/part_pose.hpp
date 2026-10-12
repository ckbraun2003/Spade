#pragma once

// ---------------------------------------------------------------------------
// vehicles/part_pose.hpp -- the one CPU posing rule for a model's parts
// (CORE-7; the parts and meshes plan, Task A2). Stage 7's GPU raster mirrors
// it (CORE-8 section 5.2; TD-9), so this is the CPU reference it is checked
// against.
//
// THE POSE. B = (BodyState::pos, BodyState::orient). Each quaternion (the
// body's, a hub's, a mount's) is normalized first, with glm::normalize, as
// the spawn path normalizes a rotor row's.
//   a body part:       X = B . M
//   a part on rotor k: X = B . H_k . R_y(s_k psi_k) . M
// M = (mount_pos, mount_orient), H_k = (RotorDesc::local_pos, local_orient),
// s_k = spin_dir, and R_y(a) = (cos32(a / 2), 0, sin32(a / 2), 0): a turn
// about the hub's +Y, its thrust axis (core/fp32_math.hpp; no libm, TD-3).
// world_to_part is transform_of(SdfPose{X}), so a part is posed bit for bit
// as WorldBuilder poses a primitive at X.
//
// THE WALL VELOCITY at world x, for a posed part:
//   vel + omega x (x - com) + spin x (x - hub)
// with omega = orient * omega_body (BodyState::omega_body is body-frame), and,
// on rotor k, spin = s_k Omega_k a_k (a_k the world thrust axis, Omega_k
// RotorRow::omega) and hub = h_k, the world hub. Both are 0 on a body part.
//
// TWO FORMS, ONE RULE.
//   posed_program(): the WALL parts as one world-frame SdfProgram, the form a
//     solver takes for the static world SDF.
//   sample_parts(): every posed part, wall or porous, each evaluated in its
//     own frame (part_sample, vehicles/model_part.hpp). The smallest
//     distance wins, and A TIE GOES TO THE LOWER PART INDEX (CORE-8 section
//     5.2): the parts are visited in declaration order and a later part
//     replaces the winner only when its distance is strictly smaller. It
//     reports the winner's index, surface kind and wall velocity. A mesh part
//     (Task B4) has no SdfProgram form, so sample_parts is the interface a
//     solver reads every part through.
//   The two agree within rounding on the wall parts (tests/test_part_pose.cpp).
//
// PSI is the blade azimuth, in radians, any finite value: the caller wraps it
// (sin32 is within 1 ulp for |psi| <= 100). Physics owns it (P11); this rule
// takes it as an input and stores nothing.
//
// HOST CALLS, never in step(). posed_parts() and posed_program() allocate
// their results; part_wall_velocity(), part_sample() and sample_parts()
// never allocate.
//
// PRECONDITIONS: `model` has passed ModelType::validate() (register_model()'s
// rule), and a PosedParts was posed from that same model.
//
// LAYERING: vehicles/ includes world/ and state/, never scene/ or sim/.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "state/layout.hpp"         // BodyState
#include "vehicles/model_part.hpp"  // PartDesc, part_sample: a part's own geometry, in its part frame
#include "vehicles/model_type.hpp"  // ModelType, PartSurface, kNoPart
#include "world/sdf.hpp"            // SdfProgram, SdfSample, SdfTransform

namespace spade::vehicles {

// Which parts to pose. rigid_only drops the parts on a rotor: the actuator
// disc and actuator line tiers model the rotor themselves.
enum class PartFilter : uint32_t { all = 0, rigid_only = 1 };

// One posed part.
struct PosedPart {
    uint32_t part = 0;                         // index into ModelType::parts
    SdfTransform world_to_part{};              // transform_of(the part frame's world pose)
    glm::vec3 origin{0.0f};                    // the part frame's origin, world, m
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};  // part -> world (a product of unit quaternions)
    glm::vec3 hub{0.0f};                       // a rotor part: its hub, world, m; 0 for a body part
    glm::vec3 spin{0.0f};                      // a rotor part: s_k * Omega_k * the world thrust axis, rad/s; 0 for a body part
};

// A model's parts, posed for one body state.
struct PosedParts {
    glm::vec3 com{0.0f};           // BodyState::pos, m
    glm::vec3 vel{0.0f};           // BodyState::vel, m/s
    glm::vec3 omega{0.0f};         // orient * omega_body: the world angular velocity, rad/s
    std::vector<PosedPart> parts;  // declaration order, filtered
};

// Poses `model`'s parts for `body`. rotor_omega and psi: one entry per model
// rotor (RotorRow::omega, rad/s; the azimuth, rad), or empty when no rotor
// part is posed.
//
// invalid_argument, naming what is wrong: a body orientation that is not
// finite and non-zero; a non-finite body pos, vel or omega_body; a filter
// that is neither all nor rigid_only; a span neither empty nor
// rotors.size() long; a non-finite entry (naming the span and the rotor); a
// span empty while a rotor part is posed (naming the part); a rotor part whose
// rotor is out of range (a model that does not validate). Whatever
// transform_of() refuses, prefixed with the part.
[[nodiscard]] Result<PosedParts> posed_parts(const ModelType& model, const BodyState& body,
                                             std::span<const float> rotor_omega, std::span<const float> psi,
                                             PartFilter filter);

// The union of the posed WALL parts as one world-frame program. transforms[0]
// is the identity; each part's transforms are composed with
// compose_transform(t, world_to_part), its nodes appended with their
// primitives' transform indices offset, and one union_ follows each part but
// the first (compose()'s join, without materials). No wall part: no nodes.
//
// capacity_exceeded past kMaxSdfDepth (naming the part that crosses it), or
// past 2^32 - 1 transforms. invalid_argument for a posed part that is not in
// `model`, or a shape that does not validate (a model that does not validate).
[[nodiscard]] Result<SdfProgram> posed_program(const ModelType& model, const PosedParts& posed);

// vel + omega x (x - com) + spin x (x - hub), for posed.parts[i], in that
// order. i < posed.parts.size().
[[nodiscard]] glm::vec3 part_wall_velocity(const PosedParts& posed, std::size_t i, glm::vec3 x) noexcept;

// What sample_parts() reports at a world point. surface and velocity are the
// owner's, and mean nothing while owner is kNoPart.
struct PartSample {
    float distance = kSdfEmptyDistance;                   // m, negative inside
    glm::vec3 gradient{0.0f};                             // world frame
    uint32_t owner = kNoPart;                             // the ModelType part index
    PartSurface::Kind surface = PartSurface::Kind::wall;  // the owner's surface kind
    glm::vec3 velocity{0.0f};                             // the owner's wall velocity at x, world, m/s
};

// Every posed part, wall or porous: part_sample at world_to_part * x, its
// distance scaled and its gradient rotated to world as eval() does through a
// transform. The smallest distance wins; a tie goes to the lower part index
// (CORE-8 section 5.2). No posed part, or no finite distance: the defaults.
[[nodiscard]] PartSample sample_parts(const ModelType& model, const PosedParts& posed, glm::vec3 x) noexcept;

}  // namespace spade::vehicles
