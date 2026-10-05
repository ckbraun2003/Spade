// sim/module.hpp -- the module API (docs/design/core/plans/2026-10-02-module-api-design.md).
//
// A MODULE is a descriptor: a name, a version and the passes it contributes.
// create() compiles a module set into ONE ordered pass list with
// compile_schedule(). Stages 1-3 of the plan: passes, each naming the built-in
// GPU kernel that matches its CPU function, and the fields a module provides.
// Stage 4: the arrays a module owns (its state). Grades and roles join the
// descriptor in later stages.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "compute/backend.hpp"
#include "core/error.hpp"
#include "physics/field_row.hpp"
#include "state/layout.hpp"  // kStd430StructAlignment

namespace spade::physics {
struct SubstepContext;
}  // namespace spade::physics

namespace spade::modules {

// A pass's CPU half: the same function-pointer type as physics::PassFn,
// spelt here so this header stays light and does not pull in
// physics/schedule.hpp and everything it includes.
using PassFn = void (*)(const physics::SubstepContext&) noexcept;

// The engine model's six phases, run in this order every substep.
enum class Phase : uint8_t { fields = 0, forces = 1, constraints = 2, integrate = 3, sensors = 4, publish = 5 };
inline constexpr std::size_t kPhaseCount = 6;

// How a pass touches a quantity. ACCUMULATE adds to it and never clears it
// (rotors and drag on body.wrench); two accumulators of one quantity need no
// edge between them, two writers do.
enum class Access : uint8_t { read, write, accumulate };

// Where a pass sits inside its phase beyond what hazards and edges decide.
// Behaviors use it: kinematic behaviors first in Fields, force behaviors last
// in Forces. A placed pass still declares its access: placement orders it
// against passes of another placement, and two passes of the SAME placement
// that conflict need an edge like any others.
enum class Placement : uint8_t { first = 0, ordered = 1, last = 2 };

// A quantity is a core quantity (kCoreQuantities), "field.<name>" for a field
// a module in the set declares, or "<module>.<name>" for a module in the set.
// Anything else is refused, so a misspelling cannot drop a hazard.
inline constexpr std::string_view kCoreQuantities[] = {"body.pose", "body.wrench", "body.specific_force",
                                                       "world.params"};

struct QuantityAccess {
    std::string_view quantity;
    Access access = Access::read;
};

struct PassDecl {
    std::string_view name;
    Phase phase = Phase::fields;
    Placement placement = Placement::ordered;
    std::span<const QuantityAccess> access{};
    std::span<const std::string_view> after{};  // "<module>.<pass>"
    PassFn cpu = nullptr;
    // The built-in kernel that does what `cpu` does, on the GPU; `none` runs
    // only on the CPU, and Vulkan refuses the set at create().
    compute::GpuRecipe gpu = compute::GpuRecipe::none;
};

// A FIELD is a value a provider writes once per world per substep, in the
// Fields phase, and readers read in later phases (the module-API spec, section
// 6). The module that declares a field is its provider: one of its passes
// writes "field.<name>", and nothing else may.
//
// Its value is one float (scalar), three (vec3), or a fixed array of `bands`
// floats -- the acoustic and RF fields' frequency bands (drone-builder design,
// section 4).
enum class FieldKind : uint8_t { scalar = 0, vec3 = 1, bands = 2 };
inline constexpr uint32_t kMaxFieldBands = 32;

struct FieldDecl {
    std::string_view name;  // no '.'; unique in the set
    FieldKind kind = FieldKind::scalar;
    uint32_t bands = 0;     // FieldKind::bands only: 1 .. kMaxFieldBands
    std::string_view unit;  // SI, spelt in ASCII ("m/s^2")
};

// The built-in fields sit at fixed places at the front of every world's sample
// row, whether or not the set provides them, so the GPU row's layout and a
// developer field's offset never depend on which built-ins are present. A
// module that declares one must use exactly its kind and unit:
//   gravity  vec3    m/s^2   floats 0..2
//   density  scalar  kg/m^3  float  3
//   wind     vec3    m/s     floats 4..6   (float 7 is padding)
// (The values live in physics/field_row.hpp, beside the passes that use them.)
inline constexpr uint32_t kFieldGravityOffset = physics::kFieldGravityOffset;
inline constexpr uint32_t kFieldDensityOffset = physics::kFieldDensityOffset;
inline constexpr uint32_t kFieldWindOffset = physics::kFieldWindOffset;
inline constexpr uint32_t kFieldBuiltinFloats = physics::kFieldBuiltinFloats;

// STATE (stage 4; spec section 3). A module declares the arrays it owns. Each
// becomes one registered, world-partitioned array -- walked, digested,
// snapshotted and mirrored like every other -- and its slot->world map.
//
// An array's EXTENT sets its per-world capacity:
//   per_world    1
//   per_body     the world's body capacity, indexed by body slot
//   per_element  the force-element capacity (one budget, shared by every
//                per_element array, as drag and rotors share it)
//   per_sensor   the sensor capacity
//   per_row      depth rows for each row of `owner`, which holds rows
//                (per_body, per_element or per_sensor); row g of the owner
//                owns rows [g * depth, (g + 1) * depth)
enum class Extent : uint8_t { per_world = 0, per_body = 1, per_element = 2, per_sensor = 3, per_row = 4 };

struct ArrayDecl {
    std::string_view name;     // the registered name: unique in the set, no '.', not in kCoreArrays
    uint32_t elem_size = 0;    // attached_row_size<T>() for per_element and per_sensor; row_size<T>() otherwise
    Extent extent = Extent::per_world;
    std::string_view owner{};  // per_row only: the owning array (per_body, per_element or per_sensor; any module)
    uint32_t depth = 1;        // per_row only: rows per owner row, >= 1
};

// A row is raw arena bytes, never constructed or destroyed one at a time, and
// copied byte-wise by snapshots: ArenaSet::register_array's own conditions.
template <class T>
[[nodiscard]] consteval uint32_t row_size() noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "a row is copied byte-wise by snapshots");
    static_assert(std::is_trivially_destructible_v<T>, "arena slots are never individually destroyed");
    static_assert(alignof(T) <= kStd430StructAlignment, "the arena allocates at 16-byte alignment");
    return static_cast<uint32_t>(sizeof(T));
}

// A row attached to a body (per_element, per_sensor) starts with the
// WORLD-LOCAL slot of its body, as every built-in attached row does.
template <class T>
[[nodiscard]] consteval uint32_t attached_row_size() noexcept {
    static_assert(std::is_standard_layout_v<T>, "an attached row is standard-layout, so its body_slot has an offset");
    static_assert(std::is_same_v<decltype(T::body_slot), uint32_t>, "an attached row has a uint32_t body_slot");
    static_assert(offsetof(T, body_slot) == 0, "an attached row's body_slot is its first member");
    return row_size<T>();
}

// The arrays the core registers itself. No module may declare one.
inline constexpr std::string_view kCoreArrays[] = {"world_params", "bodies", "body_generation", "replay_config"};

// The arrays that predate replay_config in the walk. The four modules that
// declare them (drag, dryden, imu, rotor) carry the LEGACY MARKER, which
// registers their arrays before replay_config, in set order; the goldens'
// walk depends on it. No other module may carry it (spec section 3). A future
// snapshot-format version may drop it.
inline constexpr std::string_view kLegacyWalkArrays[] = {"drag_bodies", "dryden", "imu_sensors", "imu_ring",
                                                         "rotors"};

struct ModuleDesc {
    std::string_view name;  // no '.'; not "field", which names the field quantities
    uint32_t version = 1;
    std::span<const PassDecl> passes{};
    std::span<const FieldDecl> fields{};    // the fields this module provides
    std::span<const ArrayDecl> state{};     // the arrays this module owns
    bool legacy_walk = false;               // only on a module whose every array is in kLegacyWalkArrays
};

using ModuleSet = std::vector<ModuleDesc>;

// Owned names: a Simulation keeps its CompiledSchedule for life, and the set
// it was compiled from may have been built from temporary strings.
struct CompiledPass {
    std::string module;
    std::string pass;
    Phase phase = Phase::fields;
    PassFn cpu = nullptr;
    compute::GpuRecipe gpu = compute::GpuRecipe::none;
};

// One field of the compiled registry: where its `count` floats sit in a world's
// sample row.
struct CompiledField {
    std::string name;
    FieldKind kind = FieldKind::scalar;
    uint32_t count = 0;   // 1, 3, or its bands
    uint32_t offset = 0;  // in floats, from the start of the row
    std::string unit;
};

// One module array of the compiled table.
inline constexpr uint32_t kNoArray = 0xFFFF'FFFFu;
struct CompiledArray {
    std::string module;
    std::string name;
    uint32_t elem_size = 0;
    Extent extent = Extent::per_world;
    uint32_t owner = kNoArray;  // per_row only: the owner's index in CompiledSchedule::arrays
    uint32_t depth = 1;
    bool legacy_walk = false;   // registered before replay_config
};

struct CompiledSchedule {
    std::vector<CompiledPass> passes;
    uint64_t identity = 0;  // FNV-1a 64 over the set and the compiled order; see compile_schedule()
    // The field registry: the built-ins first (gravity, density, wind, those the
    // set provides), then every other field in set order. Not part of the
    // identity: fields are declarations, like access.
    std::vector<CompiledField> fields{};
    uint32_t field_stride = kFieldBuiltinFloats;  // floats per world's sample row
    // The module arrays in registration order: the legacy modules' arrays in
    // set order, then every other module's in set order; each module's in
    // declaration order. Not part of the identity: arrays are declarations,
    // like access and fields (the schema hash covers them).
    std::vector<CompiledArray> arrays{};
};

// Orders every pass of `modules`:
//   1. phases in Phase order;
//   2. inside a phase: placement groups (first, ordered, last); a reader after
//      every writer and accumulator of what it reads; `after` edges; two
//      writers, or a writer and an accumulator, of one placement need an edge;
//   3. ties by module-set order, then declaration order.
// invalid_argument for: a module name that is empty, contains '.', or repeats;
// a pass with no name, a '.' in its name, no CPU function, or an unknown
// phase, or declared twice; a GPU recipe
// paired with any CPU function but builtin_cpu_for(recipe); a field with no
// name, a '.' in its name, a bad kind or band count, or declared twice; a
// built-in field with another kind or unit; a module named "field"; a provider
// module with no pass that writes its field; a write of "field.<name>" outside
// its provider, or outside the Fields phase; an unknown
// quantity, including "<module>.<name>" where the module declares arrays and
// <name> is none of them; an array with no name, a '.' in its name, a core
// array's name, elem_size 0 or an unknown extent, or declared twice in the
// set; a per_row array with depth 0, or whose owner no module declares or is
// per_world or per_row; an owner or a depth other than 1 on any other array;
// the legacy marker on a module with an array outside kLegacyWalkArrays; an
// edge to a pass no module declares or to a later phase; two
// writers, or a writer and an accumulator, of one quantity with no edge
// between them; a cycle.
//
// IDENTITY, spelt byte by byte so it is the same on every platform: for each
// module in set order, its name, 0x00 and its version as 4 bytes little-endian;
// then 0x01; then for each compiled pass, module, '.', pass, 0x00 and the phase
// as one byte. FNV-1a 64 (core/rng.hpp's constants) over those bytes. State
// declarations are not spelt: the schema hash refuses a blob whose arrays
// differ, and a changed row rides the module's version.
[[nodiscard]] Result<CompiledSchedule> compile_schedule(std::span<const ModuleDesc> modules);

// Today's engine as modules. The default module set of Simulation::create().
[[nodiscard]] ModuleSet standard_modules();

// The CPU function a GPU recipe stands for; nullptr for `none`. A pass that
// names a recipe must carry exactly this function (compile_schedule refuses
// anything else), so the two backends run one experiment.
[[nodiscard]] PassFn builtin_cpu_for(compute::GpuRecipe recipe) noexcept;

// Every compiled pass as "<module>.<pass>" and its recipe, in schedule order --
// the chain the Vulkan step records. Passes with no recipe are included; the
// caller refuses them.
[[nodiscard]] std::vector<compute::GpuPass> gpu_passes(const CompiledSchedule& schedule);

// Every registered array's name in registration order, the core's included:
// world_params, bodies, body_generation, the legacy arrays, replay_config,
// then every other module array. Each name is also followed in the walk by its
// "<name>.slot_to_world" map, which this list does not spell.
[[nodiscard]] std::vector<std::string> walk_order(const CompiledSchedule& schedule);

}  // namespace spade::modules
