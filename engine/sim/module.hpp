// sim/module.hpp -- the module API (docs/design/core/plans/2026-10-02-module-api-design.md).
//
// A MODULE is a descriptor: a name, a version and the passes it contributes.
// create() compiles a module set into ONE ordered pass list with
// compile_schedule(). Stages 1-3 of the plan: passes, each naming the built-in
// GPU kernel that matches its CPU function, and the fields a module provides.
// Stage 4: the arrays a module owns (its state), the rows it attaches to
// bodies, the seeded streams those rows hold, and the scratch its passes share
// within a substep. Grades and roles join the
// descriptor in later stages.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "compute/backend.hpp"
#include "core/error.hpp"
#include "physics/field_row.hpp"
#include "state/layout.hpp"         // kStd430StructAlignment, WorldParams
#include "vehicles/model_type.hpp"  // complete: a span of an incomplete type breaks MSVC (physics/schedule.hpp)

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
                                                       "body.contact_dv", "world.params"};

// An OPTIONAL access (stage 4, Task 6) reads "<module>.<array>" of a module the
// set may not hold. When the module is in the set it is an ordinary read: the
// name must be one of that module's arrays, it orders the pass after the
// array's writers, and the pass's view binds the array. When the module is
// absent it is no hazard and orders nothing, and the view is absent. Only a
// read may be optional: an optional write or accumulation would let a pass
// change state that its ordering never accounted for.
struct QuantityAccess {
    std::string_view quantity;
    Access access = Access::read;
    bool optional = false;
};

// EDGES. `after` names passes this one runs after; `before` names passes it
// runs before -- the mirror, for a module that must slot in ahead of a pass it
// does not own (Physics' propulsion.drive before rotor.forces). An edge to a
// pass of the same phase orders the pair. Phases already order passes of
// different phases, so an edge that agrees with them (after an earlier phase,
// before a later one) is satisfied and adds nothing, and one that contradicts
// them is refused.
struct PassDecl {
    std::string_view name;
    Phase phase = Phase::fields;
    Placement placement = Placement::ordered;
    std::span<const QuantityAccess> access{};
    std::span<const std::string_view> after{};   // "<module>.<pass>", in this or an earlier phase
    std::span<const std::string_view> before{};  // "<module>.<pass>", in this or a later phase
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

// ATTACHED ROWS (stage 4; spec section 3). A per_body, per_element or
// per_sensor array is ATTACHED to bodies. Simulation::attach_row() -- which the
// typed add_* calls front -- checks the module's spawn record with `validate`,
// reserves a row for the body, writes the row's body_slot at once, and queues
// the module's `init` for the next step boundary. Until then the row is the
// arena's zeroes, so it is inert. A per_body row needs no reservation: it is
// the body's own slot.
//
// FREEING IS THE CORE'S, not the module's. When a body is despawned, the core
// walks the table in walk order: it frees every slot-allocated row whose
// body_slot is the body's (in ascending slot order), zeroes the per_body row at
// the body's slot, and zeroes every per_row child of each. A freed row reads as
// zeroes, like every free slot. There is no module `free` function until a
// module needs more than a zero-fill (the stage-4 plan's open question 2).
//
// The SPAWN RECORD is the module's own trivially copyable struct of
// `spawn_size` bytes, at most kMaxSpawnBytes; the structural queue carries it
// by value.
inline constexpr uint32_t kMaxSpawnBytes = 128;

struct RowInit {
    std::span<std::byte> row;             // the row; an attached row's body_slot is already written
    std::span<const std::byte> spawn;     // spawn_size bytes
    const WorldParams* params = nullptr;  // the world's registered row (its seed)
    uint32_t local_slot = 0;              // the row's world-local slot
    uint32_t local_body = 0;              // the world-local body slot it belongs to
    float h = 0.0f;                       // the substep, s
};
// Writes every field of the row field-wise -- never a whole-object assignment,
// which would copy a temporary's padding into the arena -- and the row's
// liveness flag last.
using RowInitFn = void (*)(const RowInit&) noexcept;
// invalid_argument on a bad record, with a message that carries no call
// prefix: the caller adds "add_imu_sensor: " or "attach_row: ".
using RowValidateFn = Result<void> (*)(std::span<const std::byte> spawn);

struct ArrayDecl {
    std::string_view name;     // the registered name: unique in the set, no '.', not in kCoreArrays
    uint32_t elem_size = 0;    // attached_row_size<T>() for per_element and per_sensor; row_size<T>() otherwise
    Extent extent = Extent::per_world;
    std::string_view owner{};  // per_row only: the owning array (per_body, per_element or per_sensor; any module)
    uint32_t depth = 1;        // per_row only: rows per owner row, >= 1
    // An attached array (per_body, per_element, per_sensor) needs `init`;
    // `validate` is optional. A per_row array may carry an init (rows a
    // vehicle hook initializes); a per_world array takes none of the three.
    uint32_t spawn_size = 0;          // bytes of the spawn record, <= kMaxSpawnBytes
    RowInitFn init = nullptr;         // writes the row at the step boundary
    RowValidateFn validate = nullptr; // checks a record when it is offered, before anything is reserved
};

// The row inside RowInit::row, typed. Row is the array's own row type, so
// sizeof(Row) is the declared elem_size.
template <class Row>
[[nodiscard]] Row& row_as(std::span<std::byte> row) noexcept {
    static_assert(std::is_trivially_copyable_v<Row>, "a row is copied byte-wise by snapshots");
    return *reinterpret_cast<Row*>(row.data());
}

// The spawn record, copied out: a record's bytes carry no alignment. A record
// shorter than Spawn leaves the rest value-initialized; attach_row() refuses a
// record whose size is not the declared spawn_size.
template <class Spawn>
[[nodiscard]] Spawn spawn_as(std::span<const std::byte> spawn) noexcept {
    static_assert(std::is_trivially_copyable_v<Spawn>, "a spawn record is carried by value through the queue");
    static_assert(sizeof(Spawn) <= kMaxSpawnBytes, "a spawn record is at most kMaxSpawnBytes");
    Spawn out{};
    std::memcpy(&out, spawn.data(), std::min(spawn.size(), sizeof(Spawn)));
    return out;
}

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
// walk depends on it. No other module may carry it, and a module that declares
// one of these arrays must (spec section 3). A future snapshot-format version
// may drop it.
inline constexpr std::string_view kLegacyWalkArrays[] = {"drag_bodies", "dryden", "imu_sensors", "imu_ring",
                                                         "rotors"};

// SEEDED STREAMS (stage 4; spec section 3). A module that keeps an rng::Stream
// in its rows declares it: the domain tag it derives under, the array whose
// rows hold it, and `reseed`, which writes one row's stream from the world's
// registered seed and the row's world-local slot -- that stream and nothing
// else, so a row's history (bias states, ring cursors, phases) is untouched.
//
// Simulation walks the declarations, in set order and then declaration order:
// create() derives every per_world stream once the world's seed is written, and
// reseed() rewrites the seed and then re-derives every stream of every live
// row. So a module's streams are reseeded with no edit to Simulation, and a
// stream that is not declared is a stream reseed() cannot reach (the 2026-10-02
// GNSS defect, closed by construction).
//
// LIVENESS IS THE ARENA'S, which is why a stream lives in a per_world array (one
// row per world, always live) or a slot-allocated one (per_element or
// per_sensor; a row is live while its slot->world map names the world). A free
// row stays zero.
//
// ONE DERIVATION PER ROW TYPE (TD-9). An attached row's init derives its stream
// too, at the step boundary: it must call this same function, so a row attached
// after a reseed and one reseeded in place hold the same stream. A per_world row
// takes no init; create() derives it through `reseed`.
//
// A tag is unique in the set, so two modules can never draw the same numbers,
// and is never kWorldSeedDomainTag ("world", sim/world_set.hpp), the tag every
// world's own root is derived under. The configuration identity does not spell
// tags: a changed tag rides the module's version, like a changed row.
using RowReseedFn = void (*)(std::span<std::byte> row, const WorldParams& params, uint32_t local_slot) noexcept;

struct StreamDecl {
    std::string_view tag;          // the rng domain tag: unique in the set, never "world"
    std::string_view array;        // this module's per_world, per_element or per_sensor array
    RowReseedFn reseed = nullptr;  // re-derives one live row's stream; history untouched
};

// CONFIGURATION TABLES (stage 4, Task 7; Physics' propulsion-rows plan, section
// 4). A table is a flat list of floats built from the MODEL REGISTRY: model
// data a pass reads, such as a propeller's coefficient table resampled from
// every registered model. Simulation builds each table once at create(), from
// no models, and register_model() rebuilds every table from all the registered
// models, in registration order. Nothing else writes one, and it never changes
// inside a step, so a step allocates nothing for it.
//
// A table is CONFIGURATION, NOT STATE: it is not registered, so it is not in
// the walk, the digest or the snapshot, and restore() does not rebuild it.
// A pass reads it as a bound view: "<module>.<table>" in its access list binds
// it (BindingKind::table), as one row of floats -- world_count 1 and elem_size
// 4, read with world_rows<float>(view, 0) -- because one table serves every
// world. A pass may only READ a table: compile_schedule refuses a write or an
// accumulation, since a table changes only in register_model(). Another module
// may read it, plainly or optionally, by the same rules as an array. Table names
// share the array namespace: unique among every array and table in the set,
// with no '.', and never a core array's name.
//
// THE IDENTITY RULE. A table is a pure function of the registered models (and
// its build function, whose changes ride the module's version, like a pass's).
// So snapshot v3's model-registry identity covers it -- a blob restores only
// into a Simulation whose registry spells the same identity, and so builds the
// same tables -- ONLY IF EVERY ModelType FIELD A BUILD READS IS FOLDED INTO
// vehicles::model_identity() (vehicles/model_identity.hpp). A build that reads
// a field the identity does not fold (visual_ref, or a field added to
// ModelType without joining the fold) would let two registries with one
// identity hold different tables, and restore() would accept a blob under the
// wrong ones. Every new build function, and every new ModelType field, is
// reviewed against that list. (The standard set declares no table.)
//
// The GPU has no copy yet: a table's device mirror waits for stage 6's fixed
// bindings. Until then only the CPU reads one. A developer's pass has no GPU
// kernel, so Vulkan refuses its set by name (stage 2's rule), and no built-in
// kernel or pass function reads a table.
//
// `build` receives the registered models in registration order and an EMPTY
// `out`, and appends the table's floats.
using ConfigBuildFn = void (*)(std::span<const vehicles::ModelType> models, std::vector<float>& out);

struct ConfigTableDecl {
    std::string_view name;          // shares the array namespace: unique in the set, no '.', not a core array
    ConfigBuildFn build = nullptr;  // the table from every registered model, in registration order
};

// THE VEHICLE-SPAWN HOOK (stage 4, Task 7). spawn(world, model, where) calls
// each module's `vehicle_rows`, in set order, once the vehicle's slots are
// reserved, with the vehicle as VehicleRows. The hook appends one
// RowInitRequest per row it wants initialized: rows of ITS OWN module only, and
// only the vehicle's own -- a per_body row at the vehicle's body slot, or a
// per_row row owned by `rotors` whose slot lies in one of the vehicle's rotor
// windows ([rotor_slot * depth, (rotor_slot + 1) * depth)). Each record must be
// the array's spawn_size, and the array must have an init.
//
// spawn() checks every request before anything is committed. One that breaks a
// rule refuses the whole spawn (invalid_argument, naming the module and the
// array) and releases every slot it reserved, so a refused spawn leaves the
// arenas and the queue as they were. The checked requests are queued as
// init_row ops after the vehicle's built-in rows, so each init runs at the next
// step boundary; the despawn cascade then frees the rows with their body or
// their rotor (ATTACHED ROWS above).
//
// A model the hook has nothing for gets no request: its rows stay the arena's
// zeroes, which a module's rows read as "not driven".
//
// The hook sees the whole registry -- what every table's build was handed --
// and the model's place in it, so a row can name that model's entries in a
// table a build laid out model by model (Physics' MotorRow::table_offset into
// propeller_tables). Every span is valid only during the call.
struct VehicleRows {
    const vehicles::ModelType* model = nullptr;     // the registered model being spawned: models[model_index]
    std::span<const vehicles::ModelType> models{};  // every registered model, in registration order
    uint32_t model_index = 0;                       // the model's place in `models` (its ModelTypeId - 1)
    uint32_t world_index = 0;
    uint32_t body_slot = 0;                         // GLOBAL
    std::span<const uint32_t> rotor_slots;          // GLOBAL, in the model's rotor order
};

struct RowInitRequest {
    std::string_view array;  // this module's per_body array, or a per_row array owned by rotors
    uint32_t slot = 0;       // the row's GLOBAL slot
    std::array<std::byte, kMaxSpawnBytes> spawn{};
    uint32_t spawn_size = 0;  // the array's spawn_size
};

using VehicleRowsFn = void (*)(const VehicleRows& vehicle, std::vector<RowInitRequest>& out);

// SCRATCH (stage 4, Task 7b; agreed with Physics 2026-10-05 for the IMU contact
// plan's contact_dv, option A). Rows a module's passes hand each other WITHIN a
// substep and do not keep across it -- the contact passes' velocity change,
// which integrate reads and clears in the same substep. NOT STATE:
//
//   NOT WALKED, DIGESTED OR SNAPSHOTTED. A scratch is never registered, so it
//   is in no walk entry, schema hash, state_digest or blob, and declaring one
//   moves none of them, nor the configuration identity.
//
//   ZERO-FILLED AT create(), AND ONLY THERE. After that, keeping every row zero
//   at every substep boundary is THE MODULE'S INVARIANT, not the engine's:
//   contact_dv's integrate zeroes every slot it reads. The engine does not clear
//   a scratch per step or per substep, so a row a module leaves non-zero at a
//   boundary is still there in the next substep, on either backend.
//
//   RESTORE NEEDS NOTHING, given that invariant. A blob is taken at a step
//   boundary, where every row is zero, so restore() neither carries a scratch
//   nor clears one, and the run resumes as the snapshotted one would have.
//
// A scratch has a NAME in its module's namespace, which its arrays and tables
// share (unique in the set, no '.', never a core array's); a ROW SIZE by
// row_size<T>()'s rules, which sizes its CPU rows AND its device buffer -- one
// size for both, so a GPU row needs no layout of its own (contact_dv is xyz
// and a pad, 16 bytes); and an EXTENT, per_body (the body capacity, indexed by
// body slot) or per_world (one row per world). No other extent: nothing
// attaches, frees or owns a scratch row.
//
// A PASS REACHES IT AS IT REACHES AN ARRAY: "<module>.<name>" in its access
// list binds it (BindingKind::scratch), every world's rows world-contiguous,
// through SubstepContext::state, sliced with world_rows<T>(). The access rules
// are the array rules: its own module or any other may read, write or
// accumulate it as a required access, the hazard graph orders those accesses as
// it orders an array's, an optional read of an absent module's scratch binds an
// absent view, and an optional write is refused. Its rows are allocated at
// create(), so a step allocates nothing.
//
// ON THE GPU kernels bind statically, so a scratch a kernel reaches names its
// fixed BINDING, a gen::kBinding_* constant generated from
// shaders/shared/bindings.slang. The state mirror creates, sizes (a 64-bit
// count), zero-fills and binds every bound scratch from its declaration, as it
// does its hand-listed derived buffers, and never uploads or reads one back.
// So on the Vulkan path the device rows are the scratch, and the CPU rows stay
// as create() left them. A scratch with kNoBinding is CPU-only and gets no
// device buffer; a Vulkan set in which a pass declares one is refused at
// create() (unavailable, naming the pass and the scratch). The mirror refuses,
// by name, a binding outside the generated range, or one a walk entry, a
// hand-listed derived buffer or another scratch already holds; compile_schedule
// refuses two scratches with one binding on every backend. (bindings.gen.hpp
// exists only in a SPADE_VULKAN build, so a declaration in a TU that must also
// build without it spells kNoBinding there, where no backend could bind it.)
inline constexpr uint32_t kNoBinding = compute::kNoBinding;

struct ScratchDecl {
    std::string_view name;              // shares the array namespace: unique in the set, no '.', not a core array
    uint32_t elem_size = 0;             // row_size<T>(): the CPU rows' size and the device buffer's stride
    Extent extent = Extent::per_world;  // per_body or per_world
    uint32_t binding = kNoBinding;      // a gen::kBinding_* constant (bindings.slang), or kNoBinding: CPU-only
};

// One request, its record copied in by value: what a hook appends.
template <class Spawn>
[[nodiscard]] RowInitRequest init_request(std::string_view array, uint32_t slot, const Spawn& spawn) noexcept {
    static_assert(std::is_trivially_copyable_v<Spawn>, "a spawn record is carried by value through the queue");
    static_assert(sizeof(Spawn) <= kMaxSpawnBytes, "a spawn record is at most kMaxSpawnBytes");
    RowInitRequest out{.array = array, .slot = slot, .spawn_size = static_cast<uint32_t>(sizeof(Spawn))};
    std::memcpy(out.spawn.data(), &spawn, sizeof(Spawn));
    return out;
}

struct ModuleDesc {
    std::string_view name;  // no '.'; not "field", which names the field quantities
    uint32_t version = 1;
    std::span<const PassDecl> passes{};
    std::span<const FieldDecl> fields{};          // the fields this module provides
    std::span<const ArrayDecl> state{};           // the arrays this module owns
    bool legacy_walk = false;                     // only on a module whose every array is in kLegacyWalkArrays
    std::span<const StreamDecl> streams{};        // the seeded streams this module's rows hold
    std::span<const ConfigTableDecl> tables{};    // the configuration tables this module builds
    VehicleRowsFn vehicle_rows = nullptr;         // the rows this module initializes when a vehicle spawns
    std::span<const ScratchDecl> scratch{};       // the scratch this module owns (Task 7b)
};

using ModuleSet = std::vector<ModuleDesc>;

inline constexpr uint32_t kNoArray = 0xFFFF'FFFFu;

// What one declared access of a compiled pass binds (stage 4, Task 6): the
// view Simulation hands the pass as SubstepContext::state[i] for its i-th
// access. "<module>.<array>" binds that array -- its index in
// CompiledSchedule::arrays, whichever module owns it -- "<module>.<table>"
// that configuration table (Task 7), its index in CompiledSchedule::tables, and
// "<module>.<scratch>" that scratch (Task 7b), its index in
// CompiledSchedule::scratch. A core quantity, a field, a stateless module's
// token, and an optional read of an absent module bind nothing: an absent view,
// so every access keeps its slot.
enum class BindingKind : uint8_t { absent = 0, array = 1, table = 2, scratch = 3 };
struct CompiledBinding {
    BindingKind kind = BindingKind::absent;
    uint32_t index = kNoArray;  // its index in CompiledSchedule::arrays, ::tables or ::scratch, by kind
};

// Owned names: a Simulation keeps its CompiledSchedule for life, and the set
// it was compiled from may have been built from temporary strings.
struct CompiledPass {
    std::string module;
    std::string pass;
    Phase phase = Phase::fields;
    PassFn cpu = nullptr;
    compute::GpuRecipe gpu = compute::GpuRecipe::none;
    std::vector<CompiledBinding> state{};  // one per declared access, in declaration order
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
struct CompiledArray {
    std::string module;
    std::string name;
    uint32_t elem_size = 0;
    Extent extent = Extent::per_world;
    uint32_t owner = kNoArray;  // per_row only: the owner's index in CompiledSchedule::arrays
    uint32_t depth = 1;
    bool legacy_walk = false;   // registered before replay_config
    uint32_t spawn_size = 0;    // attached arrays only, as declared
    RowInitFn init = nullptr;
    RowValidateFn validate = nullptr;
};

// One declared stream of the compiled table.
struct CompiledStream {
    std::string module;
    std::string tag;
    uint32_t array = kNoArray;  // its index in CompiledSchedule::arrays
    RowReseedFn reseed = nullptr;
};

// One declared configuration table.
struct CompiledTable {
    std::string module;
    std::string name;
    ConfigBuildFn build = nullptr;
};

// One module's vehicle-spawn hook.
struct CompiledVehicleRows {
    std::string module;
    VehicleRowsFn rows = nullptr;
};

// One declared scratch (Task 7b), as declared.
struct CompiledScratch {
    std::string module;
    std::string name;
    uint32_t elem_size = 0;
    Extent extent = Extent::per_world;  // per_body or per_world
    uint32_t binding = kNoBinding;      // kNoBinding: CPU-only
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
    // The declared streams in set order, then each module's declaration order:
    // the order create() and reseed() derive them in. Not part of the identity.
    std::vector<CompiledStream> streams{};
    // The configuration tables in set order, then each module's declaration
    // order. Not part of the identity: a table's contents are the model
    // registry's (ConfigTableDecl's identity rule), and its build rides the
    // module's version.
    std::vector<CompiledTable> tables{};
    // The vehicle-spawn hooks, in set order: the order spawn() calls them in.
    std::vector<CompiledVehicleRows> vehicle_rows{};
    // The declared scratch in set order, then each module's declaration order
    // (Task 7b). Not part of the identity: a scratch is not state, and a
    // changed row rides the module's version, like a changed array's.
    std::vector<CompiledScratch> scratch{};
};

// Orders every pass of `modules`:
//   1. phases in Phase order;
//   2. inside a phase: placement groups (first, ordered, last); a reader after
//      every writer and accumulator of what it reads; `after` and `before`
//      edges; two writers, or a writer and an accumulator, of one placement
//      need an edge (either kind, or a chain of them); an optional read of an
//      absent module orders nothing;
//   3. ties by module-set order, then declaration order.
// and binds each pass's declared accesses (CompiledPass::state).
// invalid_argument for: a module name that is empty, contains '.', or repeats;
// a pass with no name, a '.' in its name, no CPU function, or an unknown
// phase, or declared twice; a GPU recipe
// paired with any CPU function but builtin_cpu_for(recipe); a field with no
// name, a '.' in its name, a bad kind or band count, or declared twice; a
// built-in field with another kind or unit; a module named "field"; a provider
// module with no pass that writes its field; a write of "field.<name>" outside
// its provider, or outside the Fields phase; an unknown
// quantity, including "<module>.<name>" where the module declares arrays,
// tables or scratch and <name> is none of them; a write or an accumulation of a
// table; a scratch with no name, a '.' in its name, a core array's name, the
// name of an array, a table or another scratch in the set, elem_size 0, an
// extent other than per_body or per_world, or the binding of another scratch; a
// table with no name, a '.' in its name, a core array's name, the name of an
// array or another table in the set, or no build function; an array with no
// name, a '.' in its name, a core
// array's name, elem_size 0 or an unknown extent, or declared twice in the
// set; a per_row array with depth 0, or whose owner no module declares or is
// per_world or per_row; an owner or a depth other than 1 on any other array;
// a spawn_size above kMaxSpawnBytes; an attached array (per_body, per_element
// or per_sensor) with no init, or a slot-allocated one whose row cannot hold
// its uint32_t body_slot; a spawn size, init or validate on a per_world array;
// the legacy marker on a module with an array outside kLegacyWalkArrays, and an
// array in kLegacyWalkArrays declared by a module without it; a stream with an
// empty tag, the tag kWorldSeedDomainTag, a tag declared twice in the set, no
// reseed function, or an array that is not one of its module's per_world,
// per_element or per_sensor arrays; an
// optional access that is not a read, that names a core quantity, a field or
// anything but "<module>.<name>", or whose module is in the set without an
// array, table or scratch of that name; an `after` edge to a pass no module declares or
// to a later phase; a `before` edge to a pass no module declares or to an earlier
// phase; two
// writers, or a writer and an accumulator, of one quantity with no edge
// between them; a cycle, naming the passes left unordered.
//
// IDENTITY, spelt byte by byte so it is the same on every platform: for each
// module in set order, its name, 0x00 and its version as 4 bytes little-endian;
// then 0x01; then for each compiled pass, module, '.', pass, 0x00 and the phase
// as one byte. FNV-1a 64 (core/rng.hpp's constants) over those bytes. State,
// stream, table and scratch declarations and vehicle hooks are not spelt: the
// schema hash refuses a blob whose arrays differ, a table's contents are the
// model registry's (whose identity the blob carries), a scratch is in no blob,
// and a changed row, tag, build or hook rides the module's version. Edges,
// optional reads and bindings are not spelt either: what they decide is the
// compiled order, which is.
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
