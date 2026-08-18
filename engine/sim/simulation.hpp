#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <type_traits>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "compute/grid_entry.hpp"
#include "compute/step_params.hpp"
#include "core/error.hpp"
#include "core/time.hpp"
#include "physics/forces.hpp"
#include "physics/grid.hpp"
#include "physics/schedule.hpp"
#include "sensors/imu.hpp"
#include "sensors/rings.hpp"
#include "sim/world_set.hpp"
#include "state/arenas.hpp"
#include "state/snapshot.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/rotor.hpp"

// VulkanBackend (S6 Task 5) -- forward-declared, deliberately never fully
// #included here. Simulation holds one behind a unique_ptr (private member,
// below) and its destructor/move special members are DECLARED here but
// DEFINED out-of-line in simulation.cpp, where
// compute/vulkan/backend.hpp -- the Vulkan-FREE seam header a unique_ptr's
// implicit special members can be instantiated against -- is actually
// included. This is what lets sim/simulation.hpp itself stay exactly as
// Vulkan-free as it always was: nothing here names a Vulkan type, only this
// incomplete class.
namespace spade::compute {
class VulkanBackend;
}  // namespace spade::compute

// ---------------------------------------------------------------------------
// Simulation -- the engine's integration point (engine design spec §3 "The step
// model": "v1 has no concept of a step -- the substep loop lives in the
// sandbox. v2 makes the step the engine's central object: deterministic,
// declared, inspectable").
//
// It owns exactly four things and nothing else:
//
//   1. the ArenaSet -- ALL authoritative state, and therefore the whole
//      snapshot (state/registry.hpp's "no unregistered state" invariant);
//   2. the per-world CONFIGURATION the passes read (SDF program, material,
//      grid and turbulence records) -- values, not state, copied from the
//      WorldSetDesc at create();
//   3. the tick counter and the fixed dt/substep decomposition;
//   4. the structural queue.
//
// Everything else -- the pass order, the physics -- lives below it, and the
// step loop here is deliberately trivial: flush the structural queue, run the
// declared schedule `substeps` times, tick. If this file ever grows physics,
// something has gone wrong.
//
// ---------------------------------------------------------------------------
// DETERMINISM, WHICH IS THE POINT (charter P2/P4, spec §3)
//
// A Simulation's entire future is a function of (WorldSetDesc, dt_ns,
// substeps, the sequence of API calls). Concretely, the properties this class
// is responsible for:
//
//   * NO WALL CLOCK. Nothing under spade/engine/ reads one; Tick is the only
//     notion of time and it advances by exactly one per step().
//   * NO UNORDERED CONTAINERS anywhere in the step path or the structural
//     queue. The queue is a std::vector applied front to back; worlds are
//     iterated by index; slot allocation is ArenaSet's lowest-free-first.
//   * ALL RANDOMNESS through spade::rng streams derived from the per-world
//     seed. The only stochastic system in v1 is Dryden, whose stream lives in
//     a registered array and therefore rides every snapshot.
//   * FIXED CAPACITY. Nothing reallocates mid-run; a spawn past capacity is
//     capacity_exceeded, never a silent grow (which would move state and
//     change nothing observable today but everything at S6).
//
// THE REPLAY GUARANTEE, stated as the property the corpus tests:
//
//     snapshot at tick k, restore into a FRESH Simulation created from the
//     same (WorldSetDesc, dt_ns, substeps), replay the same API calls from
//     tick k -- and every registered byte at tick N matches the uninterrupted
//     run's, exactly.
//
// "Created from the same WorldSetDesc" is load-bearing and is the one part of
// the state that the blob does NOT carry: material records, grid cell sizes,
// turbulence parameters and the SDF program are CONFIGURATION. The schema hash
// catches a shape mismatch (different arrays, different capacities, different
// world count); it cannot catch a restore into a set with a different
// restitution, and that would replay the same state forward under different
// physics. Callers own that pairing.
//
// ---------------------------------------------------------------------------
// THREADING. Externally synchronized to one caller thread, like the rest of the
// engine. No pass is threaded and none may become threaded without answering
// the Gauss-Seidel sweep-order question physics/grid.hpp raises.
// ---------------------------------------------------------------------------

namespace spade {

// ---------------------------------------------------------------------------
// A reference to one spawned body.
//
// `slot` is the GLOBAL arena slot -- an index into the whole bodies array, not
// into a world's partition -- which is what makes it directly usable with
// ArenaSet::array() and with the slot->world map.
//
// `world_index` is REDUNDANT (partitions are contiguous and equal sized, so
// world == slot / body_capacity) and is carried anyway for two reasons: call
// sites read better, and validate() cross-checks the two, which catches a
// hand-built or foreign ref before it indexes anything.
//
// `generation` is the guard. It counts SLOT LIFECYCLE EVENTS: bumped on spawn
// and bumped again on despawn, so a ref is live only between them, a
// double-despawn is rejected rather than silently freeing a reused slot, and a
// ref held across a despawn/respawn of the same slot -- which is the LIKELY
// case, since allocation is lowest-free-first -- does not silently address the
// new body. Generation 0 means "never issued", matching core/ids.hpp's
// convention, so a default-constructed BodyRef is null and matches nothing.
//
// ITS PARITY IS LOAD-BEARING: **ODD MEANS LIVE, EVEN MEANS DEAD.** A free slot
// starts at 0 (even), spawn takes it to odd, despawn takes it back to even.
// That is what covers the window a lone "bump on despawn" does not: between
// despawn() and the step boundary the slot is still ALLOCATED (its release is
// queued), so the slot->world map still says "live" and only the parity knows
// better. Both validate_ref() and body_ref_at() honour it, which is what makes
// despawn()'s "the ref is dead the moment this returns" true for every way of
// obtaining a ref rather than only for the one the caller already held --
// without it, body_ref_at() would re-mint the reference despawn just killed.
//
// The bumps preserve parity across the (unreachable) uint32 wrap in both
// directions; see the notes at the two call sites.
//
// Generations live in a REGISTERED array ("body_generation"), not in a plain
// member, so that they survive a snapshot/restore round trip like every other
// piece of engine state. A generation counter outside the registry walk would
// be exactly the "state the snapshot misses" the state layer exists to prevent.
// ---------------------------------------------------------------------------
struct BodyRef {
    uint32_t world_index = 0;
    uint32_t slot = 0;
    uint32_t generation = 0;

    [[nodiscard]] constexpr bool is_null() const noexcept { return generation == 0; }

    friend constexpr bool operator==(const BodyRef&, const BodyRef&) noexcept = default;
};

// A reference to one drag element. No generation: elements are not addressable
// state (nothing reads one back), and their lifetime is strictly contained in
// their body's -- despawning a body frees every element attached to it. A ref
// held past that point names a free slot and is rejected by nothing; it is
// returned for completeness and for a caller that wants to count them.
struct DragElementRef {
    uint32_t world_index = 0;
    uint32_t slot = 0;  // global slot in the drag-element array

    friend constexpr bool operator==(const DragElementRef&, const DragElementRef&) noexcept = default;
};

// ---------------------------------------------------------------------------
// A reference to one IMU sensor. Same shape as DragElementRef and, like it, NO
// GENERATION -- but the reasoning has to be different, so it is spelled out.
//
// DragElementRef's argument is "nothing reads one back". That is false here:
// poll_imu() reads a sensor back, which is the sensor's entire purpose. What
// makes a generation unnecessary anyway is the LIFETIME: a sensor is attached
// to a body at setup and lives exactly as long as that body (despawning a body
// frees its sensors, and freeing a sensor CLEARS ITS RING -- see
// Simulation::despawn), so a ref outlives its sensor only if the caller keeps
// using it after despawning the body it was bolted to. poll_imu() reports
// not_found for a freed slot; what it cannot detect is a ref used after that
// slot was recycled by a LATER add_imu_sensor(), which would read the new
// sensor's samples.
//
// That is the same contract DragElementRef carries, stated honestly rather than
// argued away. If sensors ever outlive or migrate between bodies, the answer is
// a registered generation array and `body_generation` is the template.
// ---------------------------------------------------------------------------
struct ImuSensorRef {
    uint32_t world_index = 0;
    uint32_t slot = 0;  // global slot in the sensor array

    friend constexpr bool operator==(const ImuSensorRef&, const ImuSensorRef&) noexcept = default;
};

// What a poll_imu() call returns: samples in the caller's buffer, the index to
// resume from, and how many were lost to the ring's wrap. See
// sensors/rings.hpp for the full TA5 semantics.
using ImuPoll = sensors::PollResult<sensors::ImuSample>;

// ---------------------------------------------------------------------------
// A registered ModelType's identity (Task 18).
//
// ONE-BASED, so that a default-constructed id is null and names nothing --
// core/ids.hpp's convention for generation 0, applied to an index. There is no
// generation counter because there is no lifecycle to track: models are
// registered and never removed (see Simulation::register_model), so an id,
// once minted, is valid for the life of the Simulation and can never name a
// different model than it did when it was issued.
//
// NOT PORTABLE ACROSS Simulations, and not part of any snapshot. It is an
// index into THIS object's registration order, which is configuration --
// exactly like the world index is an index into THIS object's WorldSetDesc.
// ---------------------------------------------------------------------------
struct ModelTypeId {
    uint32_t value = 0;

    [[nodiscard]] constexpr bool is_null() const noexcept { return value == 0; }

    friend constexpr bool operator==(const ModelTypeId&, const ModelTypeId&) noexcept = default;
};

// ---------------------------------------------------------------------------
// A reference to one spawned VEHICLE: its body, plus the slots of every
// element and sensor the model type asked for.
//
// WHY THE SLOTS RIDE INLINE RATHER THAN BEING LOOKED UP. The obvious
// alternative -- carry only the BodyRef and find a vehicle's rotors by
// scanning the world's rotor partition for rows whose body_slot matches --
// works, but it makes ROTOR INDEX mean "position in ascending slot order",
// which is only the model's declaration order while the arena happens to be
// unfragmented. After any despawn/respawn churn the free list can hand a
// vehicle's four rotors slots {0, 3, 7, 8}, and the mapping from
// `model.rotors[i]` to a row would silently depend on allocation history.
// Carrying the slots makes rotor i rotor i, forever, by construction -- and
// makes set_rotor_commands() an O(1) array write.
//
// THE SIZE THAT COSTS: kMaxModelRotors + kMaxModelImuMounts slots, inline, so
// a VehicleRef is around eighty bytes and is passed by const reference. That
// is the price of the bound; see vehicles/model_type.hpp for why the bounds
// exist and what raising one costs.
//
// STALENESS IS THE BodyRef's. The generation guard, the parity rule and the
// "a ref captured before a snapshot is stale after a restore" caveat are all
// BodyRef's (see above) and are inherited unchanged: every entry point that
// takes a VehicleRef validates `body` first, so a dead vehicle is rejected
// exactly where a dead body would be.
// ---------------------------------------------------------------------------
struct VehicleRef {
    BodyRef body{};
    ModelTypeId model{};

    // GLOBAL arena slots, in the model's DECLARATION order: rotor_slots[i] is
    // the row built from `model.rotors[i]`.
    uint32_t rotor_count = 0;
    std::array<uint32_t, vehicles::kMaxModelRotors> rotor_slots{};

    // The sensors, likewise in declaration order. Whole refs rather than bare
    // slots because a caller passes these straight to poll_imu().
    uint32_t imu_count = 0;
    std::array<ImuSensorRef, vehicles::kMaxModelImuMounts> imu_sensors{};

    [[nodiscard]] constexpr bool is_null() const noexcept { return body.is_null(); }
};

// ---------------------------------------------------------------------------
// Where and how a vehicle enters the world.
//
// The same shape as BodySpawn minus everything the ModelType already says
// (mass, inertia, elements, sensors) and plus the one piece of instance state
// a model type cannot know:
//
// `rotor_omega` IS THE SHAFT SPEED EVERY ROTOR SPAWNS AT *AND* THE COMMAND IT
// SPAWNS HOLDING. Both, and that is the point: a vehicle inserted mid-flight
// starts IN TRIM rather than spinning its rotors up from rest under gravity.
// The alternative -- spawn at zero and let the caller command hover -- costs
// several rotor time constants of unrecoverable descent (hover thrust cancels
// gravity, it does not cancel the velocity already accumulated), which is not
// a state any hover, hold or trim test can start from.
//
// ONE SCALAR FOR EVERY ROTOR, not one per rotor. A symmetric airframe's trim
// IS a single common speed (vehicles/quadrotor.hpp's hover_command()), and an
// asymmetric one has no common-speed trim to spawn into anyway. A caller that
// wants unequal initial speeds spawns at rest and commands them, paying the
// lag.
// ---------------------------------------------------------------------------
struct VehicleSpawn {
    glm::vec3 pos{0.0f};
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};  // (w, x, y, z) at construction; normalized by spawn
    glm::vec3 vel{0.0f};                       // world frame, m/s
    glm::vec3 omega_body{0.0f};                // body frame, rad/s
    float rotor_omega = 0.0f;                  // rad/s, >= 0; see above
};

// ---------------------------------------------------------------------------
// What Simulation::spawn() needs to fully initialize a BodyState.
//
// DELIBERATELY BODY-ONLY (no force elements, no sensors, no model type), and
// it STAYS that way now that the model-type layer exists (Task 18): a bare
// body is what a projectile, a prop or a test fixture is, and the composed
// object -- body plus elements plus sensors, manufactured from one description
// -- is spawn(world, ModelTypeId, VehicleSpawn). Two entry points, two honest
// shapes; widening this one would have built half the model-type layer here.
//
// UNITS AND FRAMES follow state/layout.hpp's BodyState exactly: pos/vel are
// world frame, omega_body and inv_inertia_diag are body frame, mass is kg.
// `orient` is (w, x, y, z) at construction (the GLM constructor order; see
// layout.hpp's quaternion caveat) and is normalized by spawn().
// ---------------------------------------------------------------------------
struct BodySpawn {
    glm::vec3 pos{0.0f};
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 vel{0.0f};
    glm::vec3 omega_body{0.0f};
    float mass = 1.0f;
    glm::vec3 inv_inertia_diag{1.0f};
};

// One drag element's parameters. `body_slot` is not here -- it comes from the
// BodyRef add_drag_element() is given, which is the only way to guarantee the
// world-local index physics/forces.hpp requires.
struct DragElementSpawn {
    uint32_t mode = physics::drag_mode::quadratic;
    float area = 0.0f;                                 // m^2, quadratic mode only
    glm::vec3 coeffs{0.0f};                            // see drag_mode:: for the per-mode meaning
    glm::vec3 local_pos{0.0f};                         // body-frame offset from COM, m
    glm::quat local_orient{1.0f, 0.0f, 0.0f, 0.0f};    // local -> body
};

// ---------------------------------------------------------------------------
// One IMU's parameters. Same shape and same reasoning as DragElementSpawn: the
// body it attaches to comes from the BodyRef, not from a field here.
//
// The DEFAULT IS AN IDEAL SENSOR at the body's COM sampling every substep --
// all four sigmas zero, identity mount pose, rate_divider 1. That default is
// deliberately the one a test or a bring-up scenario wants; a realistic sensor
// is a fully specified one.
//
// UNITS AND FRAMES follow sensors/imu.hpp exactly. In particular the four
// sigmas are PER-SAMPLE standard deviations, NOT spectral densities -- see that
// header's section 4 for the datasheet conversion, which is the caller's job.
// ---------------------------------------------------------------------------
struct ImuSensorSpawn {
    glm::vec3 mount_pos{0.0f};                       // body-frame offset from COM, m
    glm::quat mount_orient{1.0f, 0.0f, 0.0f, 0.0f};  // mount -> body
    uint32_t rate_divider = 1;                       // emit one sample every N substeps
    float sigma_a = 0.0f;                            // accel white noise, m/s^2 per sample
    float sigma_g = 0.0f;                            // gyro white noise, rad/s per sample
    float sigma_ba = 0.0f;                           // accel bias walk step, m/s^2 per sample
    float sigma_bg = 0.0f;                           // gyro bias walk step, rad/s per sample
};

// ---------------------------------------------------------------------------
// ReplayConfig -- the run's identity, as registered state (ticket M-1).
//
// WHAT PROBLEM THIS SOLVES. A snapshot blob is a registry walk: it carries
// STATE and, deliberately, no configuration at all. So until this record
// existed, `sim.restore(blob)` accepted any blob whose ARRAY SHAPES matched --
// and two Simulations built from world sets that disagree about restitution,
// about turbulence, about the SDF geometry, or running at a different substep
// h, have byte-identical shapes. The blob loaded, the state was correct, and
// every subsequent step ran different physics. restore()'s own doc comment used
// to name that hole and hand it to the caller as a contract ("pairing a blob
// with the (WorldSetDesc, dt_ns, substeps) it came from is the caller's
// contract"). This record is that contract made ENFORCEABLE: the three numbers
// below ride in the walk, so they are in every blob, and restore() compares
// them before it writes a byte.
//
// IT IS CONFIGURATION LIVING IN STATE, WHICH IS THE POINT. Other registered
// arrays hold per-element parameters too (physics::DragBodyRow's coefficients,
// vehicles::RotorRow's curves), but those belong to a spawned thing and change
// with its lifecycle; this one is a WHOLE-RUN CONSTANT, and it is the only
// array in the walk that is. It is written EXACTLY ONCE, by create(), and
// thereafter read-only to the whole engine; no pass touches it, and the only
// other write in its life is restore() copying back bytes it has already proven
// identical. So it is a constant in every run, which is precisely what makes it
// a usable identity: it is the same in the blob and in the target, or the
// pairing is wrong.
//
// 32 BYTES, std430-safe, laid out with the same battery discipline as
// physics::ContactParams: every byte belongs to a named field, so the record
// hashes and compares byte-wise with nothing left to reason about. `_pad` and
// `_reserved0` are that discipline, not decoration -- `_pad` keeps
// `config_hash` on its natural 8-byte boundary, and `_reserved0` is the growth
// slot a version-2 field takes without moving anything.
//
// WHY THE HASH AND NOT THE DESC ITSELF. Storing the WorldSetDesc would mean
// serializing SDF programs into every blob -- unbounded, and a format to
// version. A 64-bit identity answers the only question restore() asks ("is this
// the same configuration?") in eight bytes. It cannot say WHAT differs, which
// is why the error message says which of the three FIELDS diverged and leaves
// the diffing of two world sets to the caller who has both.
// ---------------------------------------------------------------------------
struct ReplayConfig {
    uint64_t dt_ns;        // Simulation::dt_ns() -- the STEP duration, ns
    uint32_t substeps;     // Simulation::substeps() -- substeps per step
    uint32_t _pad;         // keeps config_hash 8-byte aligned; must stay 0
    uint64_t config_hash;  // config_hash(WorldSetDesc) -- see sim/world_set.hpp
    uint64_t _reserved0;   // reserved for versioned growth; must stay 0
};

// The layout battery, in physics/contacts.hpp's style and for its stated
// reason: these asserts are the enforcement, not decoration. This record is
// registered state, so its byte image is part of every snapshot blob and every
// committed golden digest -- a layout change a human would have missed becomes
// a build error here rather than a corpus-wide digest move nobody can explain.
static_assert(std::is_standard_layout_v<ReplayConfig>,
              "ReplayConfig must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<ReplayConfig>,
              "ReplayConfig must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<ReplayConfig>,
              "registered array slots are never individually destroyed");
static_assert(sizeof(ReplayConfig) == 32, "ReplayConfig layout is a snapshot contract");
// std430 gives a struct the alignment of its largest member -- 8 here, since
// nothing in it is a vec3/vec4. Asserted rather than assumed because ArenaSet
// requires alignof(T) <= 16 and because a stray glm member would silently
// change both this and the stride.
static_assert(alignof(ReplayConfig) == 8, "ReplayConfig layout is a snapshot contract");
static_assert(offsetof(ReplayConfig, dt_ns) == 0);
static_assert(offsetof(ReplayConfig, substeps) == 8);
static_assert(offsetof(ReplayConfig, _pad) == 12);
static_assert(offsetof(ReplayConfig, config_hash) == 16);
static_assert(offsetof(ReplayConfig, _reserved0) == 24);
static_assert(offsetof(ReplayConfig, dt_ns) % 8 == 0, "uint64 needs 8-byte alignment in std430 too");
static_assert(offsetof(ReplayConfig, config_hash) % 8 == 0, "uint64 needs 8-byte alignment in std430 too");
// Named fields account for every byte: no IMPLICIT padding anywhere in
// ReplayConfig. Without this the two `_` lanes would be indistinguishable from
// compiler padding, and a byte-wise comparison of two rows -- which is what a
// snapshot restore performs -- could differ on bytes no field owns.
static_assert(sizeof(ReplayConfig::dt_ns) + sizeof(ReplayConfig::substeps) + sizeof(ReplayConfig::_pad) +
                      sizeof(ReplayConfig::config_hash) + sizeof(ReplayConfig::_reserved0) ==
                  sizeof(ReplayConfig),
              "ReplayConfig has implicit padding: every byte must belong to a named field");

// The registered name of the ReplayConfig array. PUBLIC, unlike the other eight
// array names (which are private constants in sim/simulation.cpp), because two
// things outside this file key on it by name: a caller inspecting a blob's
// configuration without restoring it (find_section(), state/snapshot.hpp), and
// the per-world replay digest, which must SKIP it -- see the normalization note
// in engine/testing/replay.hpp.
inline constexpr std::string_view kReplayConfigArray = "replay_config";

class Simulation {
public:
    // ---------------------------------------------------------------------
    // create -- validate the world set, allocate every arena, seed every
    // world, and pin the step decomposition.
    //
    // FIXED dt, IMMUTABLE AT CREATION (spec C5/§3). `dt_ns` is the STEP
    // duration in nanoseconds and `substeps` the per-step substep count; the
    // effective substep duration is dt_ns / substeps. Nanoseconds because the
    // step rate is an integer fact that must not accumulate float error over a
    // million steps, and because it is what the host-side rate contracts (25 Hz
    // vision, 200 Hz supervisor) are expressed in.
    //
    // THE ns -> s CONVERSION IS EXACT WHERE IT CAN BE AND CORRECTLY ROUNDED
    // WHERE IT CANNOT, and the distinction matters enough to spell out:
    //
    //   * dt_ns must be divisible by `substeps` (invalid_argument otherwise).
    //     So the substep duration in NANOSECONDS, h_ns = dt_ns / substeps, is
    //     an exact integer -- no truncation, no drift, no "1000000/3" silently
    //     becoming 333333.
    //   * substep_h() is then static_cast<float>(h_ns) / 1.0e9f. Both operands
    //     are EXACTLY representable in fp32 for every rate this engine runs at
    //     -- h_ns below 2^24 ns (16.7 ms, i.e. every substep rate at or above
    //     60 Hz) converts exactly, and 1e9 is exactly representable because
    //     1e9 / 2^9 = 1953125 fits in 21 bits. A single IEEE-754 division of
    //     two exact operands is correctly rounded, so every conformant build
    //     computes the SAME float. That -- reproducibility -- is what
    //     determinism needs.
    //   * What is NOT true, and is worth saying because it is the natural thing
    //     to assume: the RESULT is generally not exact. 1 ms is not
    //     representable in binary fp32; h for a 1 kHz substep is the nearest
    //     float to 0.001 (0x3A83126F, about 1.00000005e-3). The engine's
    //     substep duration is that float, identically, everywhere -- which is
    //     the property the goldens pin. A caller who needs the exact rate reads
    //     dt_ns(), not substep_h().
    //
    // Codes: invalid_argument (the world set does not validate -- see
    // validate_world_set(); dt_ns == 0; substeps == 0; dt_ns not divisible by
    // substeps), capacity_exceeded (the arena allocation does not fit), plus
    // (S6 Task 5) `backend`'s own failure taxonomy when
    // backend.kind == compute::BackendKind::vulkan (compute/vulkan/backend.hpp's
    // VulkanBackend::create() doc comment: unavailable, invalid_argument,
    // capacity_exceeded, internal).
    //
    // `backend` (S6 Task 5) -- DEFAULTED TRAILING PARAMETER, so every call
    // site that predates this task compiles UNCHANGED and gets today's
    // behaviour: BackendKind::cpu, the existing host-side schedule, exactly
    // as before. Passing BackendDesc{.kind = vulkan} instead makes this
    // Simulation own a VulkanBackend (see vulkan_backend_ below) and routes
    // step() through it -- see step()'s own doc comment for the split.
    // ---------------------------------------------------------------------
    [[nodiscard]] static Result<Simulation> create(const WorldSetDesc& desc, uint64_t dt_ns,
                                                   uint32_t substeps,
                                                   const compute::BackendDesc& backend = {});

    // Declared here, DEFINED in simulation.cpp (not `= default` inline):
    // vulkan_backend_ is a unique_ptr<compute::VulkanBackend> and
    // VulkanBackend is only forward-declared above this class -- an
    // implicitly-instantiated destructor needs the complete type, which
    // simulation.cpp has (it includes compute/vulkan/backend.hpp) and this
    // header deliberately does not. The four special members below all
    // touch that member, so all four move out-of-line together.
    ~Simulation();

    // Move-only, for the same reason ArenaSet is: copying would have to deep
    // copy every arena. Moving is cheap and safe -- every pointer the registry
    // caches lives in a heap block the arenas only reference, so it survives
    // the move (state/arenas.hpp's move note).
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    // Deliberately NOT spelled `noexcept` here either (same reasoning the
    // pre-Task-5 comment gave for the inline `= default` this replaces): the
    // exception specification is whatever ArenaSet's, the vectors', and now
    // unique_ptr<VulkanBackend>'s all turn out to be, and simulation.cpp's
    // `Simulation::Simulation(Simulation&&) = default;` derives it exactly
    // the same way an inline `= default` would have -- moving the
    // DEFINITION out-of-line does not change what gets derived, only where.
    Simulation(Simulation&&);
    Simulation& operator=(Simulation&&);

    // --- shape and time ---------------------------------------------------
    [[nodiscard]] uint32_t world_count() const noexcept { return layout_.world_count; }
    [[nodiscard]] Tick tick() const noexcept { return tick_; }
    [[nodiscard]] uint64_t dt_ns() const noexcept { return dt_ns_; }
    [[nodiscard]] uint32_t substeps() const noexcept { return substeps_; }
    [[nodiscard]] uint64_t substep_dt_ns() const noexcept { return dt_ns_ / substeps_; }
    [[nodiscard]] float substep_h() const noexcept { return h_; }
    [[nodiscard]] const WorldSetLayout& layout() const noexcept { return layout_; }

    // S6 Task 5 instrumentation: how many times step() has uploaded the
    // vulkan-path device mirror (compute::VulkanBackend::upload_count()),
    // zero on the cpu path or before this Simulation's first step(). This is
    // what tests/test_gpu_state_mirror.cpp's dirty-tracking test reads --
    // "a structural op between steps forces re-upload... assert only the
    // re-upload happened via an instrumentation counter" (this task's
    // brief) -- rather than a test reaching past Simulation's private
    // vulkan_backend_ member. DEFINED in simulation.cpp, not inline here,
    // for the same incomplete-VulkanBackend reason the destructor/move
    // members are: this class body only forward-declares the type.
    [[nodiscard]] uint64_t vulkan_upload_count() const noexcept;

    // ---------------------------------------------------------------------
    // S6 Task 6: the StepWitness the Integrate kernel published on its most
    // recent dispatch -- the tick it READ out of the per-step buffer and the
    // PassParams::substep it was dispatched with. This is what closes S6 Task
    // 5's deliberately-deferred half ("proving a kernel can read it is Task
    // 6's job"); see compute/step_params.hpp for what each field proves and
    // why wave A publishes a witness rather than inventing a physical
    // dependency on a tick none of its four passes needs.
    //
    // Costs a device->host copy and a queue submit, so it is a diagnostic
    // call and not a step-path one. `unavailable` on the cpu path (there is
    // no device to ask), which is a fact about the backend rather than a
    // failure.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<compute::StepWitness> vulkan_step_witness() const;

    // ---------------------------------------------------------------------
    // S6 Task 7: the CollisionDynamic key array, as the most recent
    // CollisionDynamic pass left it -- i.e. SORTED by physics::
    // grid_entry_less()'s exact key, since the sweep is the chain's last
    // dispatch and never reorders it.
    //
    // WHY THIS IS EXPOSED AT ALL. The sorted key array is the one part of the
    // ported pass whose correctness is NOT visible in the trajectory: a sort
    // defect that merely permutes two entries of the same cell changes the
    // Gauss-Seidel sweep order and therefore the answer, but by an amount no
    // tolerance band can distinguish from rounding. Reading the keys out is
    // what lets tests/test_gpu_parity.cpp's GpuGridSort cases compare them
    // ELEMENT FOR ELEMENT against std::sort with the CPU's own comparator --
    // an equality assertion, not a banded one.
    //
    // Same posture as vulkan_step_witness() above: a DIAGNOSTIC (one
    // device->host copy and one queue submit, never on the step path), and
    // `unavailable` on the cpu path, which is a fact about the backend rather
    // than a failure. The vector covers the whole POWER-OF-TWO-PADDED domain,
    // sentinels included (compute/grid_entry.hpp) -- the caller decides where
    // the live prefix ends, because that boundary is part of what the test is
    // checking.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<std::vector<compute::GridEntryRow>> vulkan_grid_entries() const;

    // ---------------------------------------------------------------------
    // S6 Task 10: per-pass GPU timing for the most recently completed step()
    // call (compute/backend.hpp's PassDurationsNs -- see that struct's own
    // doc comment for why it lives in the Vulkan-free compute/backend.hpp
    // rather than compute/step_params.hpp beside StepWitness). Same
    // diagnostic posture as vulkan_step_witness()/vulkan_grid_entries()
    // immediately above: `unavailable` on the cpu backend, never called from
    // step() itself. Unlike those two, this reads no device memory of its
    // own -- PassDurationsNs is already a host-side reduction by the time it
    // reaches here -- so this call costs no extra device round trip beyond
    // whatever the timing query pool's own vkGetQueryPoolResults did.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<compute::PassDurationsNs> vulkan_pass_durations_ns() const;

    // ---------------------------------------------------------------------
    // step -- advance `n` steps.
    //
    // CPU PATH (backend.kind == cpu at create(), the default -- unchanged
    // from before S6 Task 5): flush the structural queue (the step
    // BOUNDARY), then run the declared schedule `substeps` times, then
    // ++tick. n == 0 is a no-op that still flushes -- "advance zero steps"
    // is a boundary too.
    //
    // VULKAN PATH (S6 Task 5, vulkan_backend_ != nullptr): n == 0 takes the
    // identical early flush-only return the cpu path does (see step()'s
    // definition) -- no GPU work for "advance zero steps" either. For n > 0:
    // flush the structural queue ONCE (not once per step the way the cpu
    // path's loop does -- sound today because nothing queues a structural op
    // from INSIDE a step yet, the same "no reentrant hook exists" fact
    // flush_structural()'s own doc comment already relies on), upload the
    // device mirror if vulkan_dirty_, submit all `n` steps as ONE
    // VulkanBackend::step() call, read the result back into arenas_, and
    // advance tick_ by `n`. Errors are whatever the flush/upload/step/
    // readback stage that failed reports -- see VulkanBackend's own doc
    // comments (compute/vulkan/backend.hpp) for that taxonomy.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<void> step(uint64_t n = 1);

    // ---------------------------------------------------------------------
    // flush_structural -- apply every pending structural change NOW.
    //
    // step() calls this itself, so a caller never has to. It is public for the
    // one case where it matters: making a just-spawned body addressable
    // (apply_wrench, body state reads) before the first step, and making a
    // snapshot legal -- snapshot() refuses a non-empty queue rather than
    // silently dropping it.
    //
    // This does NOT weaken §3's "structural changes are queued and applied only
    // at step boundaries". A caller can only reach this between steps, and
    // between two steps IS a step boundary. The queue's reason to exist is
    // calls made from INSIDE a step (pass callbacks, controller hooks), which
    // do not exist yet and which this design is ready for.
    //
    // S6 Task 5: on the vulkan path (vulkan_backend_ != nullptr), applying a
    // non-empty queue also sets vulkan_dirty_ -- the device mirror now
    // disagrees with arenas_ and the next step() (or an explicit re-upload)
    // must refresh it before trusting a readback. An empty queue changes
    // nothing and leaves the flag as it was.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<void> flush_structural();

    [[nodiscard]] std::size_t pending_structural_ops() const noexcept { return queue_.size(); }

    // --- structural API (queued; applied at the next boundary) ------------

    // ---------------------------------------------------------------------
    // spawn -- reserve a body slot in `world_index` and queue its
    // initialization.
    //
    // TWO PHASES, AND THE SPLIT IS DELIBERATE:
    //
    //   * SLOT RESERVATION IS IMMEDIATE, in call order, lowest-free-first --
    //     which is exactly spec §4's "Spawn/despawn (C5 calls) allocate/free
    //     slots deterministically in call order within capacity". That is what
    //     lets this return a usable BodyRef instead of a promise.
    //   * STATE INITIALIZATION AND ACTIVATION ARE QUEUED and applied at the
    //     next step boundary. Until then the reserved slot holds the zeroes
    //     ArenaSet guarantees, so flags == 0 and every pass skips it
    //     (physics/integrator.hpp's active-high, inert-when-zero contract). A
    //     body therefore never appears mid-step.
    //
    // VALIDATION (all invalid_argument):
    //   * world_index in range;
    //   * pos, vel, omega_body, inv_inertia_diag all finite; mass finite and
    //     > 0 (integrate_bodies divides by it; 0 means degenerate, not
    //     infinite); inv_inertia_diag componentwise >= 0 (0 legitimately means
    //     "this axis cannot be angularly accelerated");
    //   * orient finite with non-zero length (it is normalized here, so the
    //     integrator's unit-quaternion precondition holds from substep 1);
    //   * THE SPAWN IS NOT BURIED IN THE WORLD. phi = eval(world_sdf, pos)
    //     must satisfy phi >= -contacts.proxy_radius -- i.e. the body origin is
    //     no deeper than one proxy radius inside the solid. This is not
    //     fastidiousness: the Baumgarte positional correction is a FRACTION of
    //     the excess penetration per substep and is uncapped, so a body spawned
    //     deep inside geometry is ejected with a correction proportional to its
    //     depth, which reads as a teleport and can tunnel it through the far
    //     side. The predicate is spelled `!(phi >= -r)` so a NaN field value
    //     rejects rather than slipping through.
    //
    // capacity_exceeded when the world already holds its declared
    // `capacities.bodies` (which is the ENFORCED limit even when the arena
    // partition is larger -- see WorldSetLayout). Note that a QUEUED DESPAWN
    // has not yet released its slot, so at a full world a despawn/spawn pair
    // issued in the same window is refused even though the net population is
    // unchanged; flush_structural() between the two makes it succeed. That is
    // deterministic and conservative -- the alternative, reasoning about the
    // queue's net effect on capacity, would make the answer depend on where in
    // a batch the call sits.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<BodyRef> spawn(uint32_t world_index, const BodySpawn& body);

    // ---------------------------------------------------------------------
    // despawn -- queue the release of a body's slot.
    //
    // The generation is bumped IMMEDIATELY, so `ref` is dead the moment this
    // returns: a second despawn is not_found rather than a double free, and
    // nothing can address the body during the window before the boundary. The
    // slot itself is released at the boundary, in queue order.
    //
    // CASCADES TO THE BODY'S FORCE ELEMENTS AND SENSORS. Every drag element and
    // every IMU sensor in the same world whose body_slot names this body is
    // freed too, in ascending slot order. Without the cascade an element would
    // outlive its body, be inert (apply_drag and synthesize_imu both skip rows
    // whose body is not active) and leak its slot -- so a long run with
    // spawn/despawn churn would exhaust capacity while appearing to work.
    //
    // FREEING A SENSOR ALSO ZEROES ITS RING WINDOW. ArenaSet zero-fills a freed
    // slot's own bytes, but a sensor's samples live in a SECOND array that is
    // direct-indexed rather than slot-allocated, so nothing else would clear
    // them. Leaving them would break the engine-wide "a freed slot reads as
    // zeroes" invariant, put dead samples in every subsequent snapshot, and
    // hand the next sensor allocated into that slot a ring full of another
    // sensor's history.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<void> despawn(BodyRef ref);

    // Reserve a drag-element slot and queue its initialization. Same two-phase
    // shape as spawn(). The element's DragBodyRow::body_slot is derived from
    // `ref` as the WORLD-LOCAL index physics/forces.hpp requires.
    [[nodiscard]] Result<DragElementRef> add_drag_element(BodyRef ref, const DragElementSpawn& elem);

    // ---------------------------------------------------------------------
    // add_imu_sensor -- reserve a sensor slot on `ref`'s body and queue its
    // initialization. Same two-phase shape as spawn() and add_drag_element():
    // the slot is reserved immediately (in call order, lowest-free-first), the
    // row is written at the next step boundary, and until then `kind` is
    // sensor_kind::none so the SensorSynthesis pass skips it.
    //
    // THE SENSOR'S rng STREAM IS SEEDED AT THE BOUNDARY, from the world's
    // registered WorldParams::seed and the sensor's WORLD-LOCAL slot
    // (sensors::imu_noise_stream). World-local, not global, so a world's noise
    // does not depend on where that world sits in the set -- the same property
    // DragBodyRow::body_slot's world-locality protects.
    //
    // VALIDATION (all invalid_argument): rate_divider >= 1; every sigma finite
    // and >= 0; mount_pos finite; mount_orient finite with non-zero length (it
    // is normalized here, so the pass's unit-quaternion precondition holds from
    // the first substep). not_found for a dead or stale BodyRef;
    // capacity_exceeded when the world already holds its declared
    // `capacities.sensors`.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<ImuSensorRef> add_imu_sensor(BodyRef ref, const ImuSensorSpawn& sensor);

    // --- model types (Task 18) --------------------------------------------

    // ---------------------------------------------------------------------
    // register_model -- validate a ModelType and add it to this Simulation's
    // registry, returning the id spawn() takes.
    //
    // THE REGISTRY IS CONFIGURATION, NOT STATE, and that has consequences a
    // caller must know:
    //
    //   * IT IS NOT IN THE SNAPSHOT. restore() does not rebuild it, exactly as
    //     it does not rebuild the SDF programs or the material records. A
    //     replay into a fresh Simulation must register the same models, in the
    //     same order, to obtain the same ids -- one more item on
    //     Simulation::restore()'s "what this does not restore" list.
    //   * THERE IS NO UNREGISTER. Models are append-only, so an id can never
    //     name a different model than it did when it was issued, which is what
    //     lets ModelTypeId be a bare one-based index with no generation.
    //   * IT IS NOT A STRUCTURAL CHANGE. Nothing about the arenas moves, so
    //     this is not queued and may be called at any time between steps.
    //
    // Codes: invalid_argument (the model does not validate -- reported
    // verbatim from ModelType::validate()), capacity_exceeded (2^32-1 models,
    // which is not a thing that happens).
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<ModelTypeId> register_model(vehicles::ModelType model);

    [[nodiscard]] uint32_t model_count() const noexcept {
        return static_cast<uint32_t>(models_.size());
    }

    // The registered model behind an id. The pointer is invalidated by a later
    // register_model(), like StateRegistry::find()'s and for the same reason.
    [[nodiscard]] Result<const vehicles::ModelType*> model(ModelTypeId id) const;

    // ---------------------------------------------------------------------
    // spawn -- manufacture one instance of a registered model.
    //
    // The composed counterpart of spawn(world, BodySpawn): one body from the
    // model's BodyTemplate at `where`'s pose, plus one arena row for every
    // rotor, drag body and IMU mount the model declares, all attached to that
    // body and all released together when it is despawned.
    //
    // SAME TWO PHASES AS EVERY OTHER STRUCTURAL CALL: every slot is RESERVED
    // immediately, in call order, lowest-free-first -- so the returned
    // VehicleRef is usable straight away -- and every row is WRITTEN at the
    // next step boundary. Until then the body is inactive and every element
    // and sensor row is inert (`enabled`/`kind` are 0), so a half-built
    // vehicle can never be stepped.
    //
    // ALL OR NOTHING. The capacity checks below happen BEFORE the first
    // reservation, so a spawn that starts reserving cannot run out; if one
    // somehow failed anyway, every slot already taken is released and the
    // arenas are left exactly as they were (a reserve/release pair leaves the
    // free SET unchanged, which is all alloc_slot() consults -- see
    // state/arenas.hpp's resync note). A failed spawn never leaks a slot and
    // never leaves a half-attached vehicle.
    //
    // THE FORCE-ELEMENT BUDGET IS SHARED. Rotors and drag bodies live in
    // separate arena arrays but count against ONE declared capacity, the
    // world's Capacities::force_elements, because that is what spec §3 calls
    // them both ("ForceElements // rotors -> drag -> lift surfaces"). A world
    // that declares 4 force elements fits one quadrotor's 4 rotors OR 4 drag
    // bodies OR any mix, never 4 of each.
    //
    // VALIDATION (all invalid_argument): world_index in range; a live model
    // id; pos/vel/omega_body finite; orient finite with non-zero length;
    // rotor_omega finite and >= 0 (rotor.hpp: both curves are even in omega,
    // so a negative shaft speed would produce the thrust of its magnitude
    // while the lag drove it further negative); and the body's spawn position
    // not buried in the world SDF, exactly as spawn(world, BodySpawn) requires
    // and for the same Baumgarte reason.
    //
    // not_found for a null or out-of-range ModelTypeId. capacity_exceeded when
    // the world cannot fit the body, the force elements or the sensors.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<VehicleRef> spawn(uint32_t world_index, ModelTypeId model,
                                           const VehicleSpawn& where);

    // --- inputs -----------------------------------------------------------

    // ---------------------------------------------------------------------
    // set_rotor_commands -- the vehicle input door (spec S6's "command API").
    //
    // Writes `omega_cmd[i]` into the row behind `ref.rotor_slots[i]`, i.e.
    // into the rotor the model declared at index i. Commands are SHAFT SPEEDS
    // IN RAD/S, not normalized throttles and not RPM: rotor.hpp keeps the
    // engine SI throughout, and turning a controller's (collective, roll,
    // pitch, yaw) into four shaft speeds is a mixer's job, one layer up.
    //
    // PERSISTENT, UNLIKE apply_wrench(). A command stays in force until the
    // next call -- it is a row field, not an accumulator -- which is what
    // makes a vehicle hold trim without being re-commanded every step, and
    // what makes it ride every snapshot (rotor.hpp: "omega_cmd rides in the
    // row ... a replay that dropped it would spin every rotor down").
    //
    // Requires the vehicle to be LIVE AND FLUSHED, and reports (not_found)
    // rather than silently losing the command otherwise -- the queued
    // initialization would overwrite it, exactly as apply_wrench()'s would be
    // zeroed. Call step() or flush_structural() first.
    //
    // `omega_cmd.size()` must equal `ref.rotor_count` (invalid_argument
    // otherwise): a partial command is far more likely to be a mixer bug than
    // an intention. Every entry must be finite and >= 0.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<void> set_rotor_commands(const VehicleRef& ref, std::span<const float> omega_cmd);

    // ---------------------------------------------------------------------
    // apply_wrench -- accumulate an external force and torque onto a body.
    //
    // The v1 input door: the replay corpus's "input script" applies forces
    // through this, and it is what a controller will use until the RotorElement
    // command path (Task 17) exists.
    //
    // FRAMES, per state/layout.hpp's accumulator ruling: `world_force` is WORLD
    // frame, `body_torque` is BODY frame. They do not share a frame and that
    // asymmetry is intentional.
    //
    // CONSUMED BY THE NEXT SUBSTEP ONLY. Integrate clears both accumulators at
    // the end of every substep, so a wrench applied between steps acts for
    // substep 0 of the next step and then is gone. That is the honest reading
    // of an accumulator, and a caller wanting a sustained force applies it each
    // step (or, from Task 17, models it as a force element).
    //
    // Requires the body to be LIVE AND ACTIVE -- i.e. its spawn has been
    // flushed. A wrench on a still-queued body would be written into the slot
    // and then zeroed by the pending initialization, so it is reported
    // (not_found) rather than silently lost.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<void> apply_wrench(BodyRef ref, glm::vec3 world_force, glm::vec3 body_torque);

    // --- snapshot / restore -----------------------------------------------

    // ---------------------------------------------------------------------
    // snapshot -- the registry walk, at the current tick.
    //
    // Refuses (invalid_argument) while the structural queue is non-empty: the
    // queue is not part of the walk (spec §4 makes restore FLUSH it), so
    // snapshotting over pending ops would silently drop them and leave the
    // restored set holding reserved-but-never-initialized slots. Call step() or
    // flush_structural() first.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<SnapshotBlob> snapshot() const;

    // ---------------------------------------------------------------------
    // restore -- overwrite all state from a blob, and reset the clock to it.
    //
    // Uses the ArenaSet overload of spade::restore(), never the registry-level
    // one: the registry-level primitive leaves each arena's derived free lists
    // and live counts describing the PRE-restore population, so the next
    // allocation silently diverges from the uninterrupted run
    // (state/snapshot.hpp says so, and test_snapshot.cpp pins the divergence).
    //
    // Also, per spec §4's "Restore = reverse + structural-queue flush", the
    // pending queue is DISCARDED: it describes changes to a state that is being
    // replaced.
    //
    // THE ARENA RESTORE IS ALL-OR-NOTHING: every check -- schema hash, section
    // table, every declared length, every section against its registry entry --
    // happens before the first byte is written, so a rejected blob leaves the
    // arenas, the tick and the queue exactly as they were. The bookkeeping that
    // FOLLOWS a successful restore (tick, queue, scratch, view rebuild) is not
    // itself transactional; the view rebuild is the only part that can report an
    // error at all, and only by way of an array lookup that cannot fail on a set
    // this object registered itself. Stated rather than claimed away.
    //
    // ---------------------------------------------------------------------
    // BACKEND IS NOT PART OF REPLAY IDENTITY (S6 Task 9).
    //
    // ReplayConfig (above) pins exactly three numbers into every blob --
    // dt_ns, substeps, config_hash -- and compute::BackendKind is deliberately
    // NOT a fourth. A blob taken from a cpu-backend Simulation restores into a
    // vulkan-backend one, and a blob taken from a vulkan-backend Simulation
    // restores into a cpu-backend one, exactly as freely as either restores
    // into a same-backend Simulation with a matching (dt_ns, substeps,
    // config_hash) -- restore() has no code path that even reads which
    // backend produced a blob, let alone rejects a mismatch.
    //
    // WHY THAT IS THE RIGHT CONTRACT AND NOT A GAP. Registered state (what a
    // blob carries) is backend-independent by construction: BodyState, the
    // rng streams, the sensor rings and every other array in the walk mean
    // the same thing regardless of which schedule wrote them. BackendKind
    // selects an EXECUTION STRATEGY for the steps that follow a restore, not
    // a fact about the state itself -- the same distinction WorldConfig (this
    // class's own private struct, just below) draws between configuration and
    // state, one level down: nothing about "which engine will step this next"
    // belongs in the snapshot any more than the SDF program does.
    //
    // WHAT GOVERNS CONTINUATION ACROSS THE SWITCH, since it is worth being
    // precise about what restore()'s ACCEPTANCE does and does not promise. A
    // same-backend resume is bit-identical to the uninterrupted run (the
    // determinism guarantee this class's own header states). A CROSS-backend
    // resume is not, and is not supposed to be: continuing on the other
    // backend from that point onward is subject to the SAME CPU<->GPU parity
    // bands (engine/testing/parity.hpp) that govern a live comparison of the
    // two backends over the same scenario -- the divergence is the identical
    // <= 2.5-ulp div/sqrt source parity.hpp's header explains, not a new one
    // restore() introduces. tests/test_gpu_invariance.cpp's
    // CpuSnapshotRestoresIntoVulkanAndContinuesWithinBands and
    // VulkanSnapshotRestoresIntoCpuAndContinuesWithinBands are the two
    // directions of that claim, checked.
    //
    // ---------------------------------------------------------------------
    // WHAT THIS DOES NOT RESTORE -- AND WHAT IT NOW DETECTS INSTEAD.
    //
    // CONFIGURATION IS STILL NOT RESTORED, and cannot be: the SDF programs, the
    // material, grid and turbulence records, the environment and the step
    // decomposition all come from the (WorldSetDesc, dt_ns, substeps) THIS
    // Simulation was created with, and a blob carries none of them. Neither is
    // the MODEL REGISTRY (register_model(); the ids a replay uses must come
    // from re-registering the same models in the same order -- that one is
    // still an unchecked caller obligation, because model types are not
    // registered state and have no identity in the walk).
    //
    // WHAT CHANGED (ticket M-1) IS THAT A MISMATCH IS NO LONGER SILENT. The
    // `replay_config` array (see ReplayConfig above) carries this run's
    // (dt_ns, substeps, config_hash) as registered state, so it is in every
    // blob. Before a single arena byte is written, this function locates that
    // section in the blob and compares its WHOLE PAYLOAD, byte for byte,
    // against the LIVE registered array -- every world's row and every reserved
    // lane, not merely the three named fields, because the restore would
    // overwrite all of them. Any difference is invalid_argument, naming which
    // of the three diverged (or saying plainly when the difference lies outside
    // them). The old contract -- "pairing a blob with the (WorldSetDesc,
    // dt_ns, substeps) it came from is the caller's contract" -- has therefore
    // shrunk to "pair the blob with a COMPATIBLE world set", and that is now
    // enforced rather than merely documented.
    //
    // THE THREE LAYERS OF CHECK, and why none of them subsumes another:
    //   * WORLD COUNT and the schema hash catch a SHAPE mismatch -- different
    //     arrays, element sizes, capacities or world count. They say nothing
    //     about VALUES, so two sets differing only in restitution pass them
    //     both, byte for byte.
    //   * The config check catches exactly that: SAME SHAPE, DIFFERENT
    //     PHYSICS. It is a hash comparison, so it names the field, not the
    //     difference. It DEFERS to the shape verdict -- when the schema hash
    //     already disagrees it stands down, because "array 'bodies': shape 2x5
    //     in the blob, 2x4 in the registry" is a more useful sentence than
    //     "you paired the wrong blob". So the only blob it can reject is one
    //     the state layer would have accepted.
    //   * The blob parse and the section-by-section match catch a corrupt or
    //     foreign blob. All three run before anything is written.
    //
    // WHAT IT STILL CANNOT CATCH, stated rather than claimed away: a
    // Simulation created from a desc that hashes the same is accepted, which is
    // what a 64-bit identity means (see world_set.hpp's "not cryptographic"
    // note); and the model registry, above.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<void> restore(const SnapshotBlob& blob);

    // --- reseed -------------------------------------------------------------

    // ---------------------------------------------------------------------
    // reseed -- rewrite every world's rng root from a new SCENE seed, in
    // place, and re-derive everything that had already cached a stream from
    // the old one (engine design Addendum A3's second conformance case).
    //
    // This is what a training loop calls between episodes: same world set,
    // same arenas, same roster, same model registry, same refs -- a fresh
    // draw of every stochastic system. The alternative, destroying the
    // Simulation and create()ing another one from a WorldSetDesc carrying new
    // seeds, reallocates every arena, forgets every registered model and
    // invalidates every BodyRef, which is a great deal of ceremony for "give
    // me different weather".
    //
    // THE DERIVATION IS replicate()'s, VERBATIM:
    //
    //     WorldParams::seed = splitmix64(scene_seed ^ fnv1a64("world") ^ w)
    //
    // -- the same expression sim/world_set.cpp's replicate() computes, under
    // the same pinned kWorldSeedDomainTag, over the same world INDEX. The
    // equality is the point rather than a
    // coincidence: a set reseeded to S is seed-for-seed the set
    // `replicate(prototype, N, S)` would have built, so "reseed this set" and
    // "rebuild it with a new scene seed" cannot drift apart. It also inherits
    // errata-R4's domain separation unchanged -- adding a fifth world does not
    // perturb the first four's roots.
    //
    // ---------------------------------------------------------------------
    // WHAT IT REWRITES, AND WHY THE LIST IS EXACTLY THESE THREE THINGS
    //
    // The world seed is NOT read by the step loop. It is read ONCE by each
    // system that derives a stream from it, and from then on the DERIVED
    // STREAM is the live state -- which makes rewriting the seed only half a
    // reseed. Every cached derivation has to be re-derived too, and the
    // complete inventory of them is:
    //
    //   1. WorldParams::seed itself (state/layout.hpp) -- THE AUTHORITY, and
    //      the only place a seed is stored. WorldConfig deliberately holds no
    //      copy (see its "NO `seed` MEMBER" note below), which is exactly what
    //      makes this a single write rather than a two-place update that could
    //      half-fail. create() is the only other writer.
    //   2. DrydenState, one row per world -- derived by dryden_init() under
    //      the "dryden" tag at index 0 (world/medium.cpp). Re-run per world
    //      AFTER the seed write, in create()'s order and for create()'s
    //      reason: dryden_init both derives the stream and places the filter
    //      on its stationary distribution, so the world starts gusty instead
    //      of burning off a spin-up transient.
    //   3. Every LIVE ImuSensorRow::noise -- derived at OpKind::init_imu from
    //      (WorldParams::seed, WORLD-LOCAL sensor slot) via
    //      sensors::imu_noise_stream(). Re-derived for every ALLOCATED sensor
    //      row in the world, so a sensor that predates the reseed draws from
    //      the new stream exactly like one added after it. World-local, like
    //      the spawn path's, so a world's noise still does not depend on where
    //      that world sits in the set.
    //
    // AND NOTHING ELSE -- a checkable claim, not a hope. WorldParams::seed is
    // read at exactly three sites in the engine (create()'s row write,
    // dryden_init(), and imu_noise_stream() at init_imu), and an rng::Stream
    // is only ever constructed from a world seed at the latter two; every
    // other stochastic value in the engine is a draw from one of those two
    // streams, so re-deriving them re-randomizes everything downstream by
    // construction. A future system that derives a THIRD stream from the world
    // seed belongs in this list, and the A3 conformance test in
    // tests/test_m1b_bar.cpp is what will catch its absence: a system left
    // behind keeps its old stream, so a reseeded run would still agree with
    // the un-reseeded one on that system's contribution.
    //
    // WHAT IT DOES NOT TOUCH: THE PAST. Ring contents, ring cursors
    // (last_index), rate-divider phase and the accumulated bias states all
    // stay exactly as they are. A RESEED CHANGES THE FUTURE DRAWS, NOT THE
    // HISTORY: unpolled samples are readings that genuinely happened and a
    // caller is still entitled to them, and rewinding a bias state would
    // invent a different past for a sensor that is physically the same device.
    // Body state, rotor state, the tick, the model registry and the arenas'
    // shape are untouched for the same reason -- this call is about
    // randomness and nothing else.
    //
    // Refuses (invalid_argument) while the structural queue is non-empty,
    // mirroring snapshot() and for a related reason: a queued op carries its
    // own seeding (init_imu derives its stream from the WorldParams row AT
    // FLUSH TIME) while a queued despawn's rows are still allocated, so the
    // outcome of a reseed issued over a pending queue would depend on how the
    // two interleave rather than on the state alone. Requiring an empty queue
    // makes a reseed a function of the FLUSHED state and the scene seed, full
    // stop. Call step() or flush_structural() first.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<void> reseed(uint64_t scene_seed);

    // --- state inspection --------------------------------------------------

    // The whole state, for snapshots, digests and tests. Const protects the
    // arena DESCRIPTORS, not the bytes (state/registry.hpp).
    [[nodiscard]] const ArenaSet& arenas() const noexcept { return arenas_; }

    // Handy typed accessors for the two arrays a caller is likely to read back.
    [[nodiscard]] Result<std::span<const BodyState>> world_bodies(uint32_t world_index) const;
    [[nodiscard]] Result<const BodyState*> body(BodyRef ref) const;
    [[nodiscard]] Result<uint32_t> live_body_count(uint32_t world_index) const;

    // ---------------------------------------------------------------------
    // The live BodyRef occupying a world-local body slot, or not_found if that
    // slot is free.
    //
    // The state-first door to a ref, as opposed to spawn()'s ref-first one. It
    // exists because a ref captured before a snapshot is STALE after a restore
    // -- the restore rewinds the generation counters along with everything else
    // -- so anything that has to work across a replay resume point (an input
    // script, a tool, an editor selection) must be able to re-derive its refs
    // from the state it is looking at rather than from a variable it kept.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<BodyRef> body_ref_at(uint32_t world_index, uint32_t local_slot) const;

    // ---------------------------------------------------------------------
    // vehicle_ref_at -- the VEHICLE analog of body_ref_at(): a live VehicleRef
    // re-derived from the state alone, with no memory of the spawn that
    // produced it (ticket T18-M2).
    //
    // WHY IT HAS TO EXIST. A VehicleRef captured before a snapshot is STALE
    // after a restore, for exactly body_ref_at's reason -- the restore rewinds
    // the generation counters with everything else -- and a vehicle's ref
    // carries MORE than a body's: the rotor and sensor slots its commands are
    // addressed through. So an input script that commands rotors across a
    // replay resume point cannot hold a ref; it has to be able to ask the
    // state which vehicle is which. That is this function.
    //
    // ---------------------------------------------------------------------
    // WHAT `vehicle_ordinal` COUNTS, precisely, because it is NOT a slot.
    //
    // A VEHICLE, IN THE STATE, IS A LIVE BODY THAT OWNS AT LEAST ONE ROTOR
    // ROW. There is no "is a vehicle" flag anywhere -- a model type is
    // configuration, not registered state (see register_model), so the rotor
    // rows naming a body are the only evidence in the walk that the body was
    // manufactured from one. A rotorless model is therefore INVISIBLE here and
    // deliberately so: nothing distinguishes its instance from a bare body
    // with a drag element bolted on, and inventing a distinction would mean
    // inventing state.
    //
    // THE ORDINAL IS THE POSITION IN ASCENDING WORLD-LOCAL BODY SLOT ORDER
    // among those vehicles, 0-based. Slots are allocated lowest-free-first in
    // call order (spawn()), so on an UNFRAGMENTED partition -- one that has
    // seen no despawn -- that is exactly CREATION ORDER, which is the sense in
    // which "vehicle 1" means "the second vehicle this scenario spawned into
    // this world" and survives a restore.
    //
    // WHERE THAT EQUIVALENCE ENDS, stated rather than left to be discovered:
    // a despawn renumbers. The vehicle whose despawn is queued stops being
    // live the moment despawn() returns (the generation parity says so, and
    // this function honours it exactly as body_ref_at does), so every vehicle
    // after it in the world moves down one ordinal, and a later respawn into
    // the freed slot lands wherever that slot sits rather than at the end.
    // body_ref_at's addressing does not renumber, because a SLOT is an
    // address; an ordinal is a census. A caller that despawns vehicles and
    // needs stable addressing wants the body slot, not this.
    //
    // ROTOR AND SENSOR ORDER IS ASCENDING SLOT ORDER, which is the model's
    // DECLARATION order under the same unfragmented-arena condition and for
    // the same reason (spawn() reserves a model's rotors in declaration
    // order). VehicleRef's own header note explains why the spawn path carries
    // the slots inline rather than re-deriving them this way: after churn the
    // free list can hand a vehicle's four rotors non-contiguous slots, and
    // then "ascending slot order" and "declaration order" part company. That
    // caveat is inherited here in full -- this function is the resume door,
    // not a replacement for the ref spawn() returned.
    //
    // `model` IS LEFT NULL, and that is not an oversight. The model registry
    // is configuration: it is not in any blob, restore() does not rebuild it,
    // and no ModelTypeId appears anywhere in the state -- so there is nothing
    // to read it back from, and guessing (by matching live rotor rows against
    // registered models) would hand back a plausible answer that is sometimes
    // wrong. Nothing in the engine reads VehicleRef::model today; every entry
    // point that takes a VehicleRef validates `body` and indexes the slots.
    //
    // COST: O(bodies * elements) in the world's declared capacities. It is a
    // query -- a resume, a tool, an editor selection -- never a step-path
    // call, and it is spelled for clarity accordingly.
    //
    // Codes: invalid_argument for a world index outside the set; not_found
    // when the world holds fewer than `vehicle_ordinal + 1` live vehicles;
    // internal if a body somehow owns more rotors or sensors than a model may
    // declare (vehicles::kMaxModelRotors / kMaxModelImuMounts), which the
    // spawn path makes unreachable.
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<VehicleRef> vehicle_ref_at(uint32_t world_index,
                                                    uint32_t vehicle_ordinal) const;

    // This world's runtime parameter row (gravity, wind, density, seed, and the
    // body_count mirror). Read-only: WorldParams is written by create() and by
    // the structural-queue application, nowhere else.
    [[nodiscard]] Result<const WorldParams*> world_params(uint32_t world_index) const;

    // --- sensors ----------------------------------------------------------

    // ---------------------------------------------------------------------
    // poll_imu -- the samples this sensor has produced since `since_index`
    // that are still resident in its fixed-depth ring (editor tech spec TA5;
    // sensors/rings.hpp for the full semantics).
    //
    // Copies into `out` and returns a span over the filled prefix, oldest
    // first, together with the index to pass back next time and how many
    // samples were LOST to the ring's wrap. Pass `since_index == 0` to mean
    // "everything you have" -- sample indices start at 1.
    //
    // `out` should hold at least sensors::kRingDepth entries to drain the ring
    // in one call; a shorter buffer returns the oldest samples that fit and
    // stops `next_since` there, so nothing is skipped.
    //
    // not_found for a slot that is not allocated. A sensor whose spawn has not
    // been flushed yet is legal and simply has nothing to report.
    //
    // NOTE that a SampleIndex is meaningful only within one uninterrupted run:
    // a snapshot restore rewinds the rings along with everything else, so a
    // caller resuming across one re-polls from 0 (or snapshots its own cursor
    // alongside the blob).
    // ---------------------------------------------------------------------
    [[nodiscard]] Result<ImuPoll> poll_imu(ImuSensorRef ref, sensors::SampleIndex since_index,
                                           std::span<sensors::ImuSample> out) const;

    // This sensor's table row -- its mount pose, its configured sigmas, and its
    // live bias/phase/stream/cursor state. Read-only: the row is written by the
    // structural queue and by the SensorSynthesis pass, nowhere else.
    [[nodiscard]] Result<const sensors::ImuSensorRow*> imu_sensor(ImuSensorRef ref) const;

    [[nodiscard]] Result<uint32_t> live_imu_sensor_count(uint32_t world_index) const;

    // --- rotors -----------------------------------------------------------

    // One of a vehicle's rotor rows, by the model's declaration index --
    // its live shaft speed, the command in force, and the parameters the model
    // gave it. Read-only: a rotor row is written by the structural queue, by
    // set_rotor_commands() and by the RotorElement pass, nowhere else.
    //
    // not_found for a dead vehicle or a slot whose spawn is not yet flushed;
    // invalid_argument for an index at or above ref.rotor_count.
    [[nodiscard]] Result<const vehicles::RotorRow*> rotor(const VehicleRef& ref, uint32_t index) const;

    [[nodiscard]] Result<uint32_t> live_rotor_count(uint32_t world_index) const;

    // The array ids, for a caller walking the arenas directly (the replay
    // digest does). Stable for the lifetime of the Simulation.
    [[nodiscard]] ArrayId<BodyState> bodies_array() const noexcept { return bodies_id_; }
    [[nodiscard]] ArrayId<physics::DragBodyRow> drag_elements_array() const noexcept { return drag_id_; }
    [[nodiscard]] ArrayId<sensors::ImuSensorRow> imu_sensors_array() const noexcept { return imu_id_; }
    [[nodiscard]] ArrayId<sensors::ImuSample> imu_ring_array() const noexcept { return imu_ring_id_; }
    [[nodiscard]] ArrayId<vehicles::RotorRow> rotors_array() const noexcept { return rotors_id_; }
    [[nodiscard]] ArrayId<ReplayConfig> replay_config_array() const noexcept { return replay_config_id_; }

private:
    // What one world's passes read that is not per-body state. Copied from the
    // WorldInstanceDesc at create() and immutable thereafter -- configuration,
    // not state, which is precisely why none of it is in the snapshot.
    struct WorldConfig {
        SdfProgram sdf;
        DrydenParams turbulence{};
        physics::ContactParams contacts{};
        physics::GridParams grid{};
        uint32_t declared_body_capacity = 0;     // the world's OWN capacity, enforced at spawn
        uint32_t declared_element_capacity = 0;  // ditto, for force elements
        uint32_t declared_sensor_capacity = 0;   // ditto, for sensors

        // NO `seed` MEMBER, deliberately. The world's rng root lives in the
        // registered WorldParams row and nowhere else. A copy here would be
        // written once at create() and then silently disagree with the
        // authority the moment restore() overwrote WorldParams from a blob --
        // a second source of truth for the one value every stochastic system
        // in the world derives from. The declared capacities above have no
        // such problem: they are config with no registered counterpart
        // (WorldParams::body_capacity is the ARENA PARTITION size, which is
        // the set's maximum and deliberately a different number).
    };

    // One queued structural change. A plain tagged record in a std::vector:
    // FIFO by construction, with no hashing, no pointer ordering and no
    // container whose iteration order is unspecified anywhere in sight.
    enum class OpKind : uint32_t {
        init_body,   // write the reserved body slot and set body_flags::active
        free_body,   // release a body slot (and cascade to its elements and sensors)
        init_drag,   // write the reserved drag-element slot
        init_imu,    // write the reserved sensor slot and seed its rng stream
        init_rotor,  // write the reserved rotor slot and seed its shaft speed
    };

    struct StructuralOp {
        OpKind kind = OpKind::init_body;
        uint32_t world_index = 0;
        uint32_t slot = 0;        // the reserved global slot this op targets
        uint32_t body_slot = 0;   // init_drag/init_imu/init_rotor: the global body slot it attaches to
        BodySpawn body{};         // init_body
        // init_body: the per-body contact-proxy radius to write into the new
        // slot's BodyState::proxy_radius (D-S6-2), 0 = sentinel ("use the
        // world's ContactParams::proxy_radius"). NOT a BodySpawn field --
        // BodySpawn is the public, bare-body entry point and a plain body
        // never gets a custom radius (see state/layout.hpp's BodyState note),
        // so this stays 0.0f for every op a plain spawn() enqueues; only
        // Simulation::spawn(world, ModelTypeId, VehicleSpawn) sets it, from
        // the model's own ModelType::proxy_radius.
        float body_proxy_radius = 0.0f;
        DragElementSpawn drag{};  // init_drag
        ImuSensorSpawn imu{};     // init_imu
        vehicles::RotorDesc rotor{};  // init_rotor
        float rotor_omega = 0.0f;     // init_rotor: the shaft speed AND command it spawns holding
    };

    Simulation(ArenaSet arenas, WorldSetLayout layout, std::vector<WorldConfig> configs,
               uint64_t dt_ns, uint32_t substeps, float h);

    // Rebuilds the per-world views from the arenas. Called at the top of every
    // step rather than cached across steps: the cost is O(worlds) and it makes
    // a stale view -- the classic way a restore silently keeps pointing at the
    // wrong thing -- structurally impossible.
    [[nodiscard]] Result<void> rebuild_views();

    // The pre-apply half of restore(): "was this blob produced under the same
    // (dt_ns, substeps, config_hash) this Simulation runs?". Reads the blob and
    // this object's own registered row; writes nothing, so a failure leaves the
    // whole tree untouched. See its definition for why it can only ever ADD a
    // rejection, never mask one of the state layer's.
    [[nodiscard]] Result<void> check_replay_config(const SnapshotBlob& blob) const;

    // ---------------------------------------------------------------------
    // THE UNPORTED-PASS GATE IS GONE (S6 Task 8), AND ITS REMOVAL IS THE
    // HEADLINE RATHER THAN A SIDE EFFECT.
    //
    // From S6 Task 6 through Task 7 this class carried a two-halved refusal --
    // check_vulkan_unported_config() (turbulence, known at create()) and
    // check_vulkan_unported_state() (a live rotor or a live IMU, which arrive
    // with spawns and so were scanned at the step boundary). Its purpose was to
    // make a MISSING PASS loud: a world set that exercised an un-ported pass
    // would otherwise have stepped on the vulkan backend with that pass
    // silently absent, which is a wrong trajectory rather than a last-bit
    // divergence, and nothing in a digest or a band would have named the cause.
    // Each wave deleted its own clause as it landed (Task 7 removed the
    // CollisionDynamic one, a "more than one active body in a world" scan).
    //
    // Task 8 ports MediumUpdate, the RotorElement half of ForceElements and
    // SensorSynthesis -- ALL EIGHT schedule slots now have a kernel or are
    // inert by design on both backends -- so there is nothing left to be
    // silently absent and the refusal would now reject correct configurations.
    // What REPLACED it is stronger than what it forbade:
    // tests/test_gpu_parity.cpp runs the FULL committed corpus, `quad_hover`
    // (rotors + IMU + turbulence) and `two_world_isolation` (turbulence)
    // included, against the CPU within measured bands, and compares the
    // `dryden`, `rotors`, `imu_sensors` and `imu_ring` arrays rather than only
    // `bodies`. It measures the passes instead of forbidding the state that
    // would exercise them.
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    // mark_vulkan_dirty -- "the arenas just changed under the device mirror".
    // A no-op on the cpu path.
    //
    // THE CENSUS THIS FUNCTION EXISTS TO MAKE CHECKABLE (S6 Task 6 review
    // round 1, finding C1). The vulkan path uploads `arenas_` only when it
    // believes them stale, so EVERY way the arenas can change between two
    // step() calls owes a call to this. Round 1 shipped with two of the four
    // non-queue writers marked and two not -- and the two that were missed,
    // restore() and reseed(), are the two that rewrite the MOST state. The
    // complete inventory of this class's public, non-const entry points, and
    // how each is covered, is therefore written down HERE, once, rather than
    // rediscovered at each site:
    //
    //   step()                      owns the upload itself; not a caller.
    //   flush_structural()          MARKS, on a non-empty queue (the original
    //                               and, before this fix, the only site).
    //   spawn(world, BodySpawn)     COVERED TRANSITIVELY. Both overloads
    //   spawn(world, model, where)  reserve slots immediately (alloc_slot
    //   despawn()                   writes the slot->world map; despawn also
    //   add_drag_element()          bumps a generation) AND ALWAYS queue a
    //   add_imu_sensor()            matching op, so the next boundary's
    //                               flush_structural() sees a non-empty queue
    //                               and marks. There is no reserve-without-
    //                               queue path: a failed vehicle spawn
    //                               releases every slot it took, and a
    //                               reserve/release pair restores the map
    //                               bytes it touched (a free slot's row reads
    //                               as zeroes either way). Verified by the
    //                               census test, not merely argued here.
    //   apply_wrench()              MARKS (round 1). Writes force_acc/
    //                               torque_acc straight into the arena with
    //                               no queue involved.
    //   set_rotor_commands()        MARKS (round 1). Same shape.
    //   restore()                   MARKS (round 1 REVIEW, C1). Rewrites
    //                               EVERY registered array.
    //   reseed()                    MARKS (round 1 REVIEW, C1). Rewrites
    //                               WorldParams::seed, every DrydenState and
    //                               every live ImuSensorRow::noise.
    //   register_model()            touches no arena at all -- the model
    //                               registry is configuration, held in a
    //                               plain vector outside the walk.
    //
    // A future public method that writes an arena outside step() belongs in
    // that list and owes this call; tests/test_gpu_parity.cpp's
    // VulkanDirtyTracking suite is the enforcement, one case per row.
    // ---------------------------------------------------------------------
    void mark_vulkan_dirty() noexcept;

    [[nodiscard]] Result<void> apply_op(const StructuralOp& op);
    void free_drag_elements_of(uint32_t world_index, uint32_t body_slot);
    void free_imu_sensors_of(uint32_t world_index, uint32_t body_slot);
    void free_rotors_of(uint32_t world_index, uint32_t body_slot);
    // Zeroes one sensor's ring window. `slot` is the GLOBAL sensor slot; the
    // window is [slot * kRingDepth, (slot + 1) * kRingDepth), which lands in
    // the right world's ring partition by construction (global sensor slot
    // w * sensor_capacity + local times kRingDepth is exactly that world's
    // partition offset plus the local sensor's window).
    void clear_imu_ring(uint32_t slot);
    void publish_body_counts();

    [[nodiscard]] Result<uint32_t> checked_world(uint32_t world_index) const;
    [[nodiscard]] Result<void> validate_ref(BodyRef ref) const;
    [[nodiscard]] Result<void> validate_imu_ref(ImuSensorRef ref) const;

    // How many force-element slots (rotors + drag bodies) world `w` currently
    // holds. The shared budget spawn() and add_drag_element() both check
    // against Capacities::force_elements -- see spawn()'s doc comment.
    [[nodiscard]] Result<uint32_t> live_force_elements(uint32_t world_index) const;

    ArenaSet arenas_;
    WorldSetLayout layout_{};
    std::vector<WorldConfig> configs_;

    // The model-type registry. CONFIGURATION, append-only, not in the snapshot
    // -- see register_model(). A plain vector: ModelTypeId is a one-based
    // index into it, and there is no removal, so nothing can reorder it.
    std::vector<vehicles::ModelType> models_;

    ArrayId<WorldParams> world_params_id_{};
    ArrayId<BodyState> bodies_id_{};
    ArrayId<uint32_t> body_gen_id_{};
    ArrayId<physics::DragBodyRow> drag_id_{};
    ArrayId<DrydenState> dryden_id_{};
    ArrayId<sensors::ImuSensorRow> imu_id_{};
    ArrayId<sensors::ImuSample> imu_ring_id_{};
    ArrayId<vehicles::RotorRow> rotors_id_{};
    // REGISTERED LAST, and that is load-bearing rather than incidental -- see
    // the APPEND-ONLY note over create()'s registration block.
    ArrayId<ReplayConfig> replay_config_id_{};

    Tick tick_{};
    uint64_t dt_ns_ = 0;
    uint32_t substeps_ = 1;
    float h_ = 0.0f;

    std::vector<StructuralOp> queue_;
    std::vector<physics::WorldSubstepView> views_;
    physics::GridScratch scratch_;

    // ---------------------------------------------------------------------
    // S6 Task 5: the vulkan-path backend. Null iff this Simulation was
    // created with backend.kind == compute::BackendKind::cpu (every call
    // site that does not pass `backend` at all, and every existing one that
    // predates this task) -- step() branches on `vulkan_backend_ != nullptr`
    // rather than caching BackendKind separately, so there is exactly one
    // source of truth for which path a Simulation runs.
    //
    // vulkan_dirty_ starts true (not false) so the FIRST step() after
    // create() always uploads once, seeding the device mirror from create()'s
    // freshly seeded arenas, even when nothing has been spawned yet and the
    // structural queue is empty (flush_structural() only ever SETS this
    // true, on a non-empty queue -- see its doc comment -- so something else
    // has to account for "never uploaded at all").
    // ---------------------------------------------------------------------
    std::unique_ptr<compute::VulkanBackend> vulkan_backend_;
    bool vulkan_dirty_ = true;
};

}  // namespace spade
