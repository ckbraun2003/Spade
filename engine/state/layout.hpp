#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// ---------------------------------------------------------------------------
// Shared state PODs -- the engine's layout contract.
//
// THE FIELD ORDER AND PADDING IN THIS HEADER *ARE* THE std430 CONTRACT. Every
// `_pN` member below is load-bearing: it exists so that each vec3 sits on a
// 16-byte boundary, which is what std430 (and therefore the S6 GPU mirror of
// these arrays) requires of a vec3 member. Do not reorder fields, do not
// "clean up" the padding, and do not add fields anywhere except where a
// reserved slot already exists -- any of those silently changes the byte
// image that Task 7's snapshot blobs and the S6 device buffers agree on.
//
// The static_asserts below are the enforcement, not decoration: sizeof,
// alignof, and a per-field offsetof for EVERY field, plus a
// "named bytes account for the whole struct" sum that catches padding a
// compiler inserts that we did not ask for. A layout change that a human
// would have missed becomes a build error here.
//
// Provenance and lifetime of this file (engine design spec D9, §9
// "Single-source layouts"): from S6 these structs are authored once as Slang
// modules under engine/shaders/shared/ and this header is GENERATED from
// slangc reflection. The asserts survive that handover unchanged -- they are
// what makes the generated header's agreement with the hand-written
// expectations checkable. Until then this hand-written header is the single
// source, and duplicating any of these structs elsewhere in the tree is the
// exact v1 defect (a 9-file struct duplication) the spec calls out.
//
// GLM CAVEAT, also called out by the spec ("the GLM quaternion-order
// coincidence"): with glm 1.0.1 and no GLM_FORCE_QUAT_DATA_WXYZ define,
// glm::quat's CONSTRUCTOR takes (w, x, y, z) but its MEMORY order is
// x, y, z, w. Nothing in this header can static_assert that (the components
// live in an anonymous union), so test_state.cpp pins it at runtime instead:
// see StateLayout.GlmQuatMemoryOrderIsXyzw. If a glm bump ever flips that
// default, that test fails rather than the GPU quietly seeing a rotated
// world.
// ---------------------------------------------------------------------------

namespace spade {

// Base alignment std430 gives a struct whose largest member alignment is a
// vec3/vec4 -- i.e. all of the structs in this header. Also the alignment
// ArenaSet allocates its arenas to (see arenas.hpp).
inline constexpr std::size_t kStd430StructAlignment = 16;

// The engine's fp32 scalar. Spelled once so a stray double in a layout
// struct is a compile error at the asserts rather than a silent 8-byte
// field. Global constraints: fp32 only in engine state and math.
static_assert(sizeof(float) == 4, "engine state assumes 32-bit float");
static_assert(sizeof(glm::vec3) == 12, "glm::vec3 must be 3 packed floats (no GLM_FORCE_ALIGNED_GENTYPES)");
static_assert(sizeof(glm::quat) == 16, "glm::quat must be 4 packed floats");
static_assert(alignof(glm::vec3) == 4, "glm::vec3 must be 4-byte aligned; the _pN fields do the 16-byte work");
static_assert(alignof(glm::quat) == 4, "glm::quat must be 4-byte aligned; the _pN fields do the 16-byte work");

// ---------------------------------------------------------------------------
// BodyState -- one rigid body's authoritative state (engine design spec §4
// "State layout", §5 "Rigid bodies"). One element of the world-partitioned
// bodies arena.
//
// Eight 16-byte rows:
//   row 0  pos                | _p0
//   row 1  orient (quat)
//   row 2  vel                | mass
//   row 3  omega_body         | _p1
//   row 4  inv_inertia_diag   | flags
//   row 5  force_acc          | _p2
//   row 6  torque_acc         | _p3
//   row 7  specific_force     | _p4
//
// Frames: `pos`, `vel` and `force_acc` are world-frame; `omega_body`,
// `inv_inertia_diag`, `torque_acc` and `specific_force` are body-frame
// (`specific_force` is what an IMU accelerometer reads, hence body-frame by
// definition).
//
// THE TWO ACCUMULATORS DO NOT SHARE A FRAME, and that asymmetry is deliberate
// rather than an oversight -- it is the ruling this layout is single-sourcing
// for the S6 Slang generator, so it is stated here in full. The Integrate
// pass (physics/integrator.hpp) consumes `force_acc` as F/m in world frame,
// alongside world-frame gravity and velocity. It consumes `torque_acc` in the
// BODY frame, because that is the frame of the two quantities it is summed
// and scaled with: math::gyroscopic_torque()'s body-frame result and the
// body-frame principal `inv_inertia_diag`. Angular dynamics are cheapest in
// the frame the inertia tensor is diagonal in -- the body frame -- while
// linear dynamics have no such preference. Torque producers (force elements,
// contacts) therefore owe BODY-frame torques and WORLD-frame forces.
//
// `flags` is a bitfield reserved for per-body predicates (asleep, kinematic,
// ...); no bits are assigned yet, so it is 0 in every state this task
// produces -- assigning bits is the owning pass's business, not the layout's.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) BodyState {
    glm::vec3 pos;               // world-frame position, m
    float _p0;                   // std430 pad -- keeps `orient` on row 1
    glm::quat orient;            // body->world rotation, unit quaternion
    glm::vec3 vel;               // world-frame linear velocity, m/s
    float mass;                  // kg (the row-2 slot the pad would occupy)
    glm::vec3 omega_body;        // body-frame angular velocity, rad/s
    float _p1;                   // std430 pad
    glm::vec3 inv_inertia_diag;  // body-frame 1/I diagonal, 1/(kg m^2)
    uint32_t flags;              // per-body predicate bits (none assigned yet)
    glm::vec3 force_acc;         // world-frame accumulated force, N
    float _p2;                   // std430 pad
    glm::vec3 torque_acc;        // BODY-frame accumulated torque, N m (see frames note above)
    float _p3;                   // std430 pad
    glm::vec3 specific_force;    // body-frame specific force (IMU), m/s^2
    float _p4;                   // std430 pad
};

static_assert(std::is_standard_layout_v<BodyState>, "BodyState must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<BodyState>, "BodyState must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<BodyState>, "arena slots are never individually destroyed");
static_assert(alignof(BodyState) == 16, "std430 base alignment");
static_assert(sizeof(BodyState) == 128, "std430 array stride");

static_assert(offsetof(BodyState, pos) == 0);
static_assert(offsetof(BodyState, _p0) == 12);
static_assert(offsetof(BodyState, orient) == 16);
static_assert(offsetof(BodyState, vel) == 32);
static_assert(offsetof(BodyState, mass) == 44);
static_assert(offsetof(BodyState, omega_body) == 48);
static_assert(offsetof(BodyState, _p1) == 60);
static_assert(offsetof(BodyState, inv_inertia_diag) == 64);
static_assert(offsetof(BodyState, flags) == 76);
static_assert(offsetof(BodyState, force_acc) == 80);
static_assert(offsetof(BodyState, _p2) == 92);
static_assert(offsetof(BodyState, torque_acc) == 96);
static_assert(offsetof(BodyState, _p3) == 108);
static_assert(offsetof(BodyState, specific_force) == 112);
static_assert(offsetof(BodyState, _p4) == 124);

// Every vec3 starts a 16-byte row, which is the whole point of the pads.
static_assert(offsetof(BodyState, pos) % 16 == 0);
static_assert(offsetof(BodyState, orient) % 16 == 0);
static_assert(offsetof(BodyState, vel) % 16 == 0);
static_assert(offsetof(BodyState, omega_body) % 16 == 0);
static_assert(offsetof(BodyState, inv_inertia_diag) % 16 == 0);
static_assert(offsetof(BodyState, force_acc) % 16 == 0);
static_assert(offsetof(BodyState, torque_acc) % 16 == 0);
static_assert(offsetof(BodyState, specific_force) % 16 == 0);

// Named fields account for every byte: no IMPLICIT padding anywhere in
// BodyState. This matters beyond tidiness -- implicit padding is the one
// part of a state array a byte-wise snapshot comparison cannot reason about,
// so BodyState has none by construction.
static_assert(sizeof(BodyState::pos) + sizeof(BodyState::_p0) + sizeof(BodyState::orient) +
                  sizeof(BodyState::vel) + sizeof(BodyState::mass) + sizeof(BodyState::omega_body) +
                  sizeof(BodyState::_p1) + sizeof(BodyState::inv_inertia_diag) + sizeof(BodyState::flags) +
                  sizeof(BodyState::force_acc) + sizeof(BodyState::_p2) + sizeof(BodyState::torque_acc) +
                  sizeof(BodyState::_p3) + sizeof(BodyState::specific_force) + sizeof(BodyState::_p4) ==
              sizeof(BodyState),
              "BodyState has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// WorldParams -- one world's per-world parameter record (engine design spec
// §4: "Per-world parameters (gravity, medium, capacities, seed) live in an
// indexed param buffer"). One element of the world_params arena, indexed by
// world id, so a single dispatch covering N worlds reads its world's row.
//
//   row 0  gravity | air_density
//   row 1  wind    | _p
//   row 2  body_capacity | body_count | seed(lo,hi)
//   row 3  _reserved0 | <8 bytes tail padding>
//
// `body_count` MIRRORS the arena's live population for this world; ArenaSet
// is type-agnostic and does not write it. The pass that owns spawn/despawn
// publishes it (a later task) -- ArenaSet::live_count() is the authority.
//
// TAIL PADDING IS EXPLICIT AND INTENTIONAL: named fields end at byte 56 and
// alignas(16) rounds sizeof to 64, which is exactly std430's array stride
// for a struct with 16-byte base alignment. Bytes 56..63 are therefore real,
// addressable, and NOT covered by a named field -- the only such bytes in
// this header. ArenaSet zero-fills arena storage on construction and again
// on free, so those bytes are deterministically zero in every snapshot blob
// rather than heap garbage (test_state.cpp
// StateLayout.WorldParamsTailPaddingIsZeroInAnArena pins that). Growth
// consumes `_reserved0` first, then bytes 56..63, without moving any
// existing field.
// ---------------------------------------------------------------------------
#if defined(_MSC_VER)
#pragma warning(push)
// C4324: "structure was padded due to alignment specifier". That padding is
// the std430 array stride and is asserted below; the warning is telling us
// the thing we asked for happened.
#pragma warning(disable : 4324)
#endif
struct alignas(kStd430StructAlignment) WorldParams {
    glm::vec3 gravity;       // world-frame gravity, m/s^2
    float air_density;       // kg/m^3
    glm::vec3 wind;          // world-frame mean wind, m/s
    float _p;                // std430 pad -- keeps row 2 aligned
    uint32_t body_capacity;  // fixed slots in this world's body partition
    uint32_t body_count;     // live bodies (mirror; see note above)
    uint64_t seed;           // this world's rng stream seed (Task 8)
    uint64_t _reserved0;     // reserved for versioned growth; must stay 0
};
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

static_assert(std::is_standard_layout_v<WorldParams>);
static_assert(std::is_trivially_copyable_v<WorldParams>);
static_assert(std::is_trivially_destructible_v<WorldParams>);
static_assert(alignof(WorldParams) == 16, "std430 base alignment");
static_assert(sizeof(WorldParams) == 64, "std430 array stride (56 named bytes rounded up to the 16-byte base alignment)");

static_assert(offsetof(WorldParams, gravity) == 0);
static_assert(offsetof(WorldParams, air_density) == 12);
static_assert(offsetof(WorldParams, wind) == 16);
static_assert(offsetof(WorldParams, _p) == 28);
static_assert(offsetof(WorldParams, body_capacity) == 32);
static_assert(offsetof(WorldParams, body_count) == 36);
static_assert(offsetof(WorldParams, seed) == 40);
static_assert(offsetof(WorldParams, _reserved0) == 48);

static_assert(offsetof(WorldParams, gravity) % 16 == 0);
static_assert(offsetof(WorldParams, wind) % 16 == 0);
static_assert(offsetof(WorldParams, seed) % 8 == 0, "uint64 needs 8-byte alignment in std430 too");

// Named fields cover bytes 0..55; 56..63 is the documented tail pad. Pinning
// the named total separately from sizeof means inserting a field silently
// into the tail pad still trips one of these two asserts.
static_assert(sizeof(WorldParams::gravity) + sizeof(WorldParams::air_density) + sizeof(WorldParams::wind) +
                  sizeof(WorldParams::_p) + sizeof(WorldParams::body_capacity) + sizeof(WorldParams::body_count) +
                  sizeof(WorldParams::seed) + sizeof(WorldParams::_reserved0) ==
              56,
              "WorldParams named fields must occupy exactly bytes 0..55");

// Offset of the first tail-pad byte. Task 7's snapshot writer and any future
// field added to WorldParams both key on this number.
inline constexpr std::size_t kWorldParamsNamedBytes = 56;

}  // namespace spade
