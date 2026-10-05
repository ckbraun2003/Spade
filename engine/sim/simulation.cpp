#include "sim/simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include <glm/geometric.hpp>

#include "compute/sdf_program.hpp"
#include "compute/step_params.hpp"
#include "compute/vulkan/backend.hpp"
#include "core/rng.hpp"
#include "core/validate.hpp"
#include "objects/behavior.hpp"
#include "physics/integrator.hpp"
#include "sim/builtin_state.hpp"
#include "vehicles/model_identity.hpp"
#include "world/medium.hpp"
#include "world/sdf.hpp"

namespace spade {
namespace {

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

[[nodiscard]] Error missing(std::string context) {
    return Error{Code::not_found, std::move(context)};
}

[[nodiscard]] Error internal(std::string context) {
    return Error{Code::internal, std::move(context)};
}

// Where an EMPTY configuration table's view points (rebuild_views): the view
// has zero rows, so nothing ever reads or writes these bytes. They exist so
// that the view's data is not null, which would read as an absent module.
alignas(float) std::byte g_no_table_rows[sizeof(float)]{};

// A 64-bit identity, for an error message. Hex because that is how config
// hashes and digests are written everywhere else in this tree
// (the golden corpus's expected_digest, the schema hash) -- a decimal one would
// be ungreppable against them. Hand-rolled rather than via <format>/ostringstream:
// its callers are restore()'s error paths, and neither of those belongs in an
// engine TU that otherwise allocates nothing but the message itself.
[[nodiscard]] std::string hex64(uint64_t value) {
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string out = "0x";
    out.reserve(18);
    for (int shift = 60; shift >= 0; shift -= 4) {
        out += kDigits[(value >> shift) & 0xFu];
    }
    return out;
}

// The array names are the snapshot blob's stable identities (a blob keys
// entries by name, never by walk position), so changing one invalidates every
// recorded blob and every committed digest. The core's are spelled here, except
// "replay_config", which is spelled in sim/simulation.hpp (kReplayConfigArray)
// because callers outside this file key on it -- see the note at that constant.
// A module's arrays are spelled by its declarations (module-API stage 4; the
// built-ins' in sim/standard_modules.cpp).
constexpr const char* kWorldParamsArray = "world_params";
constexpr const char* kBodiesArray = "bodies";
constexpr const char* kBodyGenerationArray = "body_generation";


// ---------------------------------------------------------------------------
// THE SENSOR-FAMILY TEMPLATES -- one implementation per operation, instantiated
// once per sensor family.
//
// WHY TEMPLATES AND NOT A KIND-TAGGED ARENA. The audit that named this
// duplication (a0f81cde) proposed tagging ONE arena so poll() dispatches on the
// tag. That amendment is RETIRED; design-specs/spade/sensor-arena-dedup.md has
// the measurement. The short form: the arena is three things, and the audit was
// written about one of them.
//
//   A CPU CONTAINER   what the audit saw
//   A GPU BINDING     RWStructuredBuffer<ImuSensorRow> @5, <GnssSensorRow> @23
//   A DIGEST SOURCE   state_digest folds elem_size AND the raw bytes
//
// sizeof(ImuSensorRow) is 128 and sizeof(GnssSensorRow) is 112, so one arena
// means one union row: a 128-byte stride with 16 dead bytes per receiver, a
// VARIANT in layouts.slang instead of a concrete struct, and -- disqualifying --
//
//   WITH ONE ARENA, A CHANGE TO THE IMU ROW MOVES EVERY GNSS DIGEST AND VICE
//   VERSA. TWO SENSORS THAT SHARE NOTHING PHYSICALLY WOULD SHARE A CORPUS
//   REGENERATION.
//
// THE SEPARATENESS OF THE ARENAS IS NOT AN ACCIDENT OF HOW THEY WERE BUILT; IT
// IS WHAT KEEPS ONE SENSOR'S LAYOUT CHURN OUT OF THE OTHER'S REPRODUCIBILITY
// RECORD. A runtime tag is the right answer when the set of kinds is OPEN and
// the storage is SHARED; here the storage is deliberately not shared and the
// kinds are a closed compile-time set, which is the shape a template fits.
//
// NOT HERE: adding a sensor and freeing one. Those were never identical -- the
// validation is sensor-specific (mount_orient and four sigmas versus
// bias_tau_s and sigma_bias) and the row's init differs -- and since module-API
// stage 4 neither is a sensor-family operation at all. Each sensor module
// DECLARES its validate and init (sim/builtin_state.cpp), add_imu_sensor() and
// add_gnss_sensor() are thin fronts over the one attach_row() door, and the
// despawn cascade frees every attached row, and its ring, by declaration
// (free_rows_of). What stays here is what is still identical: the ref check,
// the poll and the row read.
// ---------------------------------------------------------------------------

template <class Row>
[[nodiscard]] Result<void> validate_sensor_ref(const ArenaSet& arenas, ArrayId<Row> rows_id,
                                               const WorldSetLayout& layout, uint32_t world_index,
                                               uint32_t slot, std::string_view what) {
    if (world_index >= layout.world_count) {
        return std::unexpected(missing(std::string(what) + " ref names a world outside this set"));
    }
    // The same redundancy check validate_ref() applies to a BodyRef: the world
    // index must agree with the partition the slot falls in, which catches a
    // hand-built ref or one from a differently-shaped set before it indexes.
    if (layout.sensor_capacity == 0 || slot / layout.sensor_capacity != world_index) {
        return std::unexpected(
            missing(std::string(what) + " ref's slot does not lie in its world's partition"));
    }
    const Result<std::span<const uint32_t>> map = arenas.slot_to_world(rows_id);
    if (!map) return std::unexpected(map.error());
    if (slot >= map->size() || (*map)[slot] != world_index) {
        return std::unexpected(missing(std::string(what) + " ref names a slot that is not allocated"));
    }
    return {};
}

template <class Row, class Sample>
[[nodiscard]] Result<sensors::PollResult<Sample>> poll_sensor(
    const ArenaSet& arenas, ArrayId<Row> rows_id, ArrayId<Sample> ring_id, uint32_t slot,
    sensors::SampleIndex since_index, std::span<Sample> out, std::string_view what) {
    const Result<std::span<const Row>> rows = arenas.array(rows_id);
    if (!rows) return std::unexpected(rows.error());
    const Result<std::span<const Sample>> ring = arenas.array(ring_id);
    if (!ring) return std::unexpected(ring.error());

    // The window is the sensor's global slot times the depth -- the implicit
    // ring reference sensors/imu.hpp describes.
    const std::size_t begin = static_cast<std::size_t>(slot) * sensors::kRingDepth;
    if (begin + sensors::kRingDepth > ring->size()) {
        return std::unexpected(internal(std::string(what) + ": ring window is outside the ring array"));
    }

    // A row whose init is still queued has kind == none and last_index == 0, so
    // this reports "nothing yet" rather than an error -- polling a sensor you
    // just added, before the next step, is a legal thing to do, and
    // sensor_kind::poll_permits() is the statement of why.
    return sensors::ring_poll<Sample>(ring->subspan(begin, sensors::kRingDepth),
                                      (*rows)[slot].last_index, since_index, out);
}

template <class Row>
[[nodiscard]] Result<const Row*> sensor_row(const ArenaSet& arenas, ArrayId<Row> rows_id,
                                            uint32_t slot) {
    const Result<std::span<const Row>> rows = arenas.array(rows_id);
    if (!rows) return std::unexpected(rows.error());
    return &(*rows)[slot];
}

// "elem_size 80, per_element, spawn_size 52" or "elem_size 32, per_row: 64 per
// row of 'imu_sensors'".
[[nodiscard]] std::string describe_array(uint32_t elem_size, modules::Extent extent, std::string_view owner,
                                         uint32_t depth, uint32_t spawn_size) {
    std::string out = "elem_size " + std::to_string(elem_size) + ", ";
    const std::string spawn = ", spawn_size " + std::to_string(spawn_size);
    switch (extent) {
        case modules::Extent::per_world: return out + "per_world";
        case modules::Extent::per_body: return out + "per_body" + spawn;
        case modules::Extent::per_element: return out + "per_element" + spawn;
        case modules::Extent::per_sensor: return out + "per_sensor" + spawn;
        case modules::Extent::per_row:
            return out + "per_row: " + std::to_string(depth) + " per row of '" + std::string(owner) + "'";
    }
    return out + "an unknown extent";
}

// THE BUILT-IN ARRAYS (module-API stage 4; open question 1, approved). The
// engine's own calls -- spawn()'s rotors, add_drag_element(), add_imu_sensor(),
// add_gnss_sensor(), the polls, the step's views -- use the
// seven arrays the standard set declares, through typed ids minted from the
// table. So a set must declare each of them exactly as the standard set does:
// the same row size, extent, owner and depth, and the same spawn size, the
// record its typed front hands attach_row(). One that omits or reshapes one is
// refused here, before anything is registered, naming the array. (Where it sits
// in the walk is compile_schedule's: only the legacy marker registers before
// replay_config, and only the legacy arrays carry it.)
//
// THE BUILT-IN STREAMS (L6). A streamed built-in array (dryden, imu_sensors,
// gnss_sensors) must also keep its stream: declared on that array under the
// standard set's tag (compile_schedule already requires a reseed function).
// One whose module dropped it, or retagged it, would hold noise reseed() never
// reaches -- silently -- so it is refused too, naming the module, the array and
// the tag.
[[nodiscard]] Result<void> check_builtin_arrays(const modules::CompiledSchedule& schedule) {
    for (const modules::ModuleDesc& standard : modules::standard_modules()) {
        for (const modules::ArrayDecl& want : standard.state) {
            const auto have = std::ranges::find(schedule.arrays, want.name, &modules::CompiledArray::name);
            if (have == schedule.arrays.end()) {
                return std::unexpected(invalid("module set: no module declares the built-in array '" +
                                               std::string(want.name) + "' (the standard set's module '" +
                                               std::string(standard.name) +
                                               "' does); the engine's own calls use it, so a set must keep it"));
            }
            const std::string_view owner = have->owner == modules::kNoArray
                                               ? std::string_view{}
                                               : std::string_view(schedule.arrays[have->owner].name);
            if (have->elem_size != want.elem_size || have->extent != want.extent || have->depth != want.depth ||
                owner != want.owner || have->spawn_size != want.spawn_size) {
                return std::unexpected(invalid(
                    "module set: the built-in array '" + have->name + "' (module '" + have->module +
                    "') is declared as " +
                    describe_array(have->elem_size, have->extent, owner, have->depth, have->spawn_size) +
                    "; the engine's is " +
                    describe_array(want.elem_size, want.extent, want.owner, want.depth, want.spawn_size)));
            }
        }
    }
    for (const modules::ModuleDesc& standard : modules::standard_modules()) {
        for (const modules::StreamDecl& want : standard.streams) {
            const bool kept = std::ranges::any_of(schedule.streams, [&](const modules::CompiledStream& s) {
                return s.tag == want.tag && s.reseed != nullptr && schedule.arrays[s.array].name == want.array;
            });
            if (kept) continue;
            // The loop above found every built-in array, so `have` is real.
            const auto have = std::ranges::find(schedule.arrays, want.array, &modules::CompiledArray::name);
            return std::unexpected(invalid(
                "module set: the built-in array '" + std::string(want.array) + "' (module '" + have->module +
                "') has no stream declared under the tag '" + std::string(want.tag) +
                "'; its rows hold noise that reseed() would never re-derive, so a set must keep the stream"));
        }
    }
    return {};
}

// The per-world rows the three capacity extents resolve to: a WorldSetLayout's
// when create() registers an array, a StepShape's when state_array_shapes()
// sizes its device buffer -- the same three numbers, read from either.
struct ExtentRows {
    uint32_t body = 0;
    uint32_t element = 0;
    uint32_t sensor = 0;
};

// A module array's per-world capacity: its extent's formula, as create()
// sized the built-ins by hand before stage 4 (sim/module.hpp's Extent). A
// per_row array's owner holds rows itself (never per_world or per_row;
// compile_schedule checked), and the product is taken in 64 bits so an
// overflow is refused rather than wrapped.
[[nodiscard]] Result<uint32_t> array_capacity(const modules::CompiledSchedule& schedule, std::size_t index,
                                              const ExtentRows& rows) {
    const auto extent_rows = [&rows](modules::Extent extent) -> uint32_t {
        switch (extent) {
            case modules::Extent::per_world: return 1u;
            case modules::Extent::per_body: return rows.body;
            case modules::Extent::per_element: return rows.element;
            case modules::Extent::per_sensor: return rows.sensor;
            case modules::Extent::per_row: break;
        }
        return 0u;  // unreachable: compile_schedule refused an unknown extent, and per_row is handled below
    };
    const modules::CompiledArray& a = schedule.arrays[index];
    if (a.extent != modules::Extent::per_row) return extent_rows(a.extent);
    const uint64_t owned = uint64_t{extent_rows(schedule.arrays[a.owner].extent)} * a.depth;
    if (owned > std::numeric_limits<uint32_t>::max()) {
        return std::unexpected(Error{Code::capacity_exceeded, "array '" + a.name + "' (module '" + a.module +
                                                                  "'): its owner's rows times its depth exceed "
                                                                  "2^32-1 per world"});
    }
    return static_cast<uint32_t>(owned);
}

}  // namespace

// The walk create() registers, as data for the GPU mirror. walk_order() is the
// registration order. The core's four arrays are sized as create() registers
// them by hand, and every module array by array_capacity(), the function
// create() registers it with; ModuleState.TheMirrorsShapesAreTheRegistryWalk
// EntryForEntry holds this list to the registry on every box.
Result<std::vector<compute::StateArrayShape>> state_array_shapes(const modules::CompiledSchedule& schedule,
                                                                 const compute::StepShape& shape) {
    const ExtentRows rows{shape.body_capacity, shape.element_capacity, shape.sensor_capacity};
    std::vector<compute::StateArrayShape> out;
    out.reserve(2 * (std::size(modules::kCoreArrays) + schedule.arrays.size()));
    const auto add = [&out](const std::string& name, uint32_t elem_size, uint32_t capacity_per_world) {
        out.push_back({name, elem_size, capacity_per_world});
        out.push_back({name + std::string(kSlotToWorldSuffix), static_cast<uint32_t>(sizeof(uint32_t)),
                       capacity_per_world});
    };
    for (const std::string& name : modules::walk_order(schedule)) {
        if (name == kWorldParamsArray) {
            add(name, static_cast<uint32_t>(sizeof(WorldParams)), 1u);
        } else if (name == kBodiesArray) {
            add(name, static_cast<uint32_t>(sizeof(BodyState)), shape.body_capacity);
        } else if (name == kBodyGenerationArray) {
            add(name, static_cast<uint32_t>(sizeof(uint32_t)), shape.body_capacity);
        } else if (name == kReplayConfigArray) {
            add(name, static_cast<uint32_t>(sizeof(ReplayConfig)), 1u);
        } else {
            const auto it = std::ranges::find(schedule.arrays, name, &modules::CompiledArray::name);
            if (it == schedule.arrays.end()) {
                // walk_order() spells only the core's four and the table's names.
                return std::unexpected(internal("state_array_shapes: '" + name + "' is in the walk but not the table"));
            }
            const auto index = static_cast<std::size_t>(it - schedule.arrays.begin());
            const Result<uint32_t> capacity = array_capacity(schedule, index, rows);
            if (!capacity) return std::unexpected(capacity.error());
            add(name, it->elem_size, *capacity);
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// create
// ---------------------------------------------------------------------------

Simulation::Simulation(ArenaSet arenas, WorldSetLayout layout, std::vector<WorldConfig> configs,
                       uint64_t dt_ns, uint32_t substeps, float h)
    : arenas_(std::move(arenas)),
      layout_(layout),
      configs_(std::move(configs)),
      dt_ns_(dt_ns),
      substeps_(substeps),
      h_(h) {}

// S6 Task 5: declared (not `= default`) in the header because
// unique_ptr<compute::VulkanBackend>'s implicit special members need
// VulkanBackend COMPLETE, and simulation.hpp only forward-declares it.
// compute/vulkan/backend.hpp, included above, makes it complete HERE, which
// is what lets `= default` be used at last -- these four definitions are
// exactly what an inline `= default` would have generated, just placed
// where the type they need is actually visible.
Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) = default;
Simulation& Simulation::operator=(Simulation&&) = default;

Result<Simulation> Simulation::create(const WorldSetDesc& desc, uint64_t dt_ns, uint32_t substeps,
                                      const compute::BackendDesc& backend, const modules::ModuleSet& module_set) {
    if (dt_ns == 0) {
        return std::unexpected(invalid("Simulation::create: dt_ns must be > 0"));
    }
    if (substeps == 0) {
        return std::unexpected(invalid("Simulation::create: substeps must be > 0"));
    }
    if (dt_ns % substeps != 0) {
        // Rejected rather than truncated: a 1e6 ns step over 3 substeps would
        // silently become 333333 ns each and lose a nanosecond per step, which
        // is a drift no digest could distinguish from a physics change.
        return std::unexpected(invalid("Simulation::create: dt_ns must be divisible by substeps "
                                       "(the substep duration must be an exact integer of ns)"));
    }

    Result<modules::CompiledSchedule> compiled = modules::compile_schedule(module_set);
    if (!compiled) return std::unexpected(compiled.error());
    if (Result<void> builtins = check_builtin_arrays(*compiled); !builtins) return std::unexpected(builtins.error());
    if (backend.kind == compute::BackendKind::vulkan) {
        // A pass with no GPU kernel would be missing from the GPU chain: the
        // GPU would run a different experiment from the CPU (L6). Refuse it by
        // name. (A recipe can only be named with its own CPU function --
        // compile_schedule checked that.)
        for (const modules::CompiledPass& pass : compiled->passes) {
            if (pass.gpu == compute::GpuRecipe::none) {
                return std::unexpected(Error{Code::unavailable, "module set: pass '" + pass.module + "." +
                                                                    pass.pass +
                                                                    "' has no GPU kernel; it runs only on the CPU"});
            }
        }
        // Every other pass has a kernel, and the GPU records the schedule's own
        // order (VulkanBackend::create() below is handed gpu_passes()), so any
        // set whose passes all name a recipe runs the same experiment on both
        // backends.
    }

    const Result<WorldSetLayout> layout = validate_world_set(desc);
    if (!layout) {
        return std::unexpected(layout.error());
    }

    // ns -> s. See the create() doc comment for the exactness argument: h_ns is
    // an exact integer, both operands of the division are exactly representable
    // in fp32 for every rate this engine runs at, and a single IEEE-754
    // division of exact operands is correctly rounded -- hence bit-identical on
    // every conformant build, which is what determinism requires.
    const uint64_t substep_ns = dt_ns / substeps;
    const float h = static_cast<float>(substep_ns) / 1.0e9f;
    if (!(h > 0.0f) || !std::isfinite(h)) {
        return std::unexpected(invalid("Simulation::create: substep duration underflows fp32"));
    }

    std::vector<WorldConfig> configs;
    configs.reserve(desc.worlds.size());
    for (const WorldInstanceDesc& instance : desc.worlds) {
        WorldConfig config;
        config.sdf = instance.world.sdf;
        config.turbulence = instance.turbulence;
        config.contacts = instance.contacts;
        config.grid = instance.grid;
        // instance.seed is NOT copied here -- it goes straight into the
        // registered WorldParams row below and lives only there. See WorldConfig.
        config.declared_body_capacity = instance.world.capacities.bodies;
        config.declared_element_capacity = instance.world.capacities.force_elements;
        config.declared_sensor_capacity = instance.world.capacities.sensors;
        configs.push_back(std::move(config));
    }

    Simulation sim(ArenaSet(layout->world_count), *layout, std::move(configs), dt_ns, substeps, h);
    sim.schedule_ = std::move(*compiled);

    // -----------------------------------------------------------------------
    // REGISTRATION ORDER IS THE WALK ORDER IS THE SCHEMA. schema_hash() folds
    // each array's name, element size and extents in registration order, and
    // restore() rejects a blob whose hash differs -- so reordering the calls
    // below, or renaming an array, invalidates every recorded snapshot and
    // every committed digest. That is the intended cost of a layout change; it
    // is not a thing to do casually.
    //
    // Each call contributes TWO registry entries (the elements and the
    // slot->world map), atomically.
    //
    // APPEND-ONLY. A new array belongs AFTER every call below it, never
    // between two existing ones: FOUR of the five golden scenario headers
    // (tests/golden/scenarios/*.scenario.yaml) carry an
    // `old + suffix = actual, match=YES` line, and a mid-list insertion would
    // silently invalidate that argument for every one of them at once rather
    // than merely adding a new digest to check. (quad_hover carries no such
    // line: it is the fifth scenario, generated after that regeneration
    // rather than continued through it -- which is the "not merely add a
    // fifth to check" the test's own comment refers to.)
    //
    // AND IT IS ENFORCED BY A TEST, NOT BY THIS COMMENT -- which is what this
    // paragraph used to claim. test_determinism.cpp's
    // ReplayConfig.OccupiesItsPinnedWalkPosition asserts replay_config's
    // elements sit at walk INDEX 16 and its slot->world map at 17. Read that
    // test before appending: it pins the POSITION and deliberately NOT the
    // last-ness, because an assertion that replay_config is LAST would fail on
    // the very move that is sanctioned (appending), and its failure message
    // would then recommend registering the new array BEFORE it -- the exact
    // mid-list insertion the rule forbids, arrived at by obeying the test.
    //
    // Appending is therefore SILENT here: the index does not move, the test
    // stays green, and the cost is regenerating the corpus for the new suffix.
    //
    // HISTORICAL NOTE, because this comment carried it wrongly until 2026-09-21:
    // the structural argument it quoted was "rotors registered LAST -> pure
    // walk suffix". That premise was superseded at S5 Task 3, when
    // replay_config joined the walk after rotors; no golden has argued it
    // since. The rule the paragraph states was never wrong -- only its
    // citation and its claim to be the sole guard.
    // -----------------------------------------------------------------------
    Result<ArrayId<WorldParams>> world_params_id =
        sim.arenas_.register_array<WorldParams>(kWorldParamsArray, 1);
    if (!world_params_id) return std::unexpected(world_params_id.error());
    sim.world_params_id_ = *world_params_id;

    Result<ArrayId<BodyState>> bodies_id =
        sim.arenas_.register_array<BodyState>(kBodiesArray, layout->body_capacity);
    if (!bodies_id) return std::unexpected(bodies_id.error());
    sim.bodies_id_ = *bodies_id;

    // Per-slot generation counters. DIRECT-INDEXED, NEVER SLOT-ALLOCATED --
    // see the note above rebuild_views() for why, and why its slot->world map
    // is uniformly "free".
    Result<ArrayId<uint32_t>> body_gen_id =
        sim.arenas_.register_array<uint32_t>(kBodyGenerationArray, layout->body_capacity);
    if (!body_gen_id) return std::unexpected(body_gen_id.error());
    sim.body_gen_id_ = *body_gen_id;

    // THE MODULE ARRAYS (module-API stage 4) are registered from the set's
    // declarations. compile_schedule tabled them in registration order
    // (schedule_.arrays), and each registers exactly what its hand-written
    // register_array<T>(name, capacity) call registered before stage 4: the
    // elements and the map, the same element size, the same per-world capacity
    // (array_capacity() above), zero-filled. Each built-in array's own
    // rationale -- why it is registered, why a ring is direct-indexed, why
    // rotors share the force-element budget -- sits beside its declaration in
    // sim/standard_modules.cpp.
    //
    // THE LEGACY MARKER is what keeps the goldens' walk. Five module arrays
    // predate replay_config: drag_bodies, dryden, imu_sensors, imu_ring and
    // rotors. The four modules that declare them (drag, dryden, imu, rotor)
    // carry the marker, so their arrays register HERE, before replay_config,
    // in set order -- and the standard set's order is the order these arrays
    // were registered in by hand. compile_schedule lets no other array carry
    // the marker and no legacy array go without it, so nothing else can
    // register at or before replay_config's position.
    sim.module_array_ids_.assign(sim.schedule_.arrays.size(), ArrayIndex{});
    const ExtentRows rows{layout->body_capacity, layout->element_capacity, layout->sensor_capacity};
    const auto register_module_arrays = [&sim, &rows](bool legacy) -> Result<void> {
        for (std::size_t i = 0; i < sim.schedule_.arrays.size(); ++i) {
            const modules::CompiledArray& array = sim.schedule_.arrays[i];
            if (array.legacy_walk != legacy) continue;
            const Result<uint32_t> capacity = array_capacity(sim.schedule_, i, rows);
            if (!capacity) return std::unexpected(capacity.error());
            const Result<ArrayIndex> index = sim.arenas_.register_bytes(array.name, array.elem_size, *capacity);
            if (!index) return std::unexpected(index.error());
            sim.module_array_ids_[i] = *index;
        }
        return {};
    };
    if (Result<void> legacy = register_module_arrays(true); !legacy) return std::unexpected(legacy.error());

    // THE RUN'S IDENTITY (ticket M-1): (dt_ns, substeps, config_hash), as
    // registered state so that it rides every blob and restore() can refuse a
    // blob produced under a different one. See ReplayConfig in
    // sim/simulation.hpp for what problem that solves; see restore() below for
    // the check itself.
    //
    // REGISTERED HERE, AT WALK POSITION 16, AND THAT POSITION IS WHAT IS
    // PINNED. The committed goldens' provenance blocks argue that this array
    // joined the walk as a pure SUFFIX -- every earlier array kept its position,
    // so every earlier digest is the new one's prefix -- and that argument
    // survives exactly as long as nothing is registered at or before this call.
    //
    // The rule is therefore about POSITION, not about being LAST. A tenth array
    // APPENDED BELOW this one is the sanctioned move: every existing entry keeps
    // its index, the corpus regenerates once for the new suffix, and this
    // paragraph stays true. Registering it ABOVE this one -- or anywhere among
    // the nine -- shifts every later entry and invalidates the continuation
    // argument for all four digests at once, which is a different and much more
    // expensive act. ReplayConfig.OccupiesItsPinnedWalkPosition
    // (tests/test_determinism.cpp) enforces precisely that and nothing more: it
    // asserts this array's INDEX, so appending below is silent and only an
    // insertion at or before it can fail.
    //
    // ONE ROW PER WORLD, holding the same set-wide record N times, and that
    // deserves a word because it is a real cost (32 bytes and one map entry per
    // world) paid for a real reason. ArenaSet is the ONLY door to registered
    // storage (state/arenas.hpp), and it partitions every array it registers by
    // world -- there is no "one global row" shape. The alternative is a
    // non-arena registration of a Simulation MEMBER, which the state registry
    // does support (state/registry.hpp) and which is unusable here: the
    // registry would cache a pointer INTO this object, and this object is moved
    // out of create() by value. Duplicating a constant is the cheap, safe
    // version of that trade; every row is written identically below, a test
    // pins that they stay identical, and restore()'s cross-check compares the
    // WHOLE array's bytes rather than row 0 -- so the duplication is checked,
    // not merely assumed.
    Result<ArrayId<ReplayConfig>> replay_config_id =
        sim.arenas_.register_array<ReplayConfig>(std::string(kReplayConfigArray), 1);
    if (!replay_config_id) return std::unexpected(replay_config_id.error());
    sim.replay_config_id_ = *replay_config_id;

    // EVERY OTHER MODULE ARRAY, APPENDED BELOW replay_config: the move the
    // paragraph above sanctions, in set order. In the standard set that is
    // gnss_sensors and gnss_ring, the first time the append was taken: every
    // earlier entry keeps its index, replay_config stays at 16, and each
    // golden's continuation argument extends by a new suffix instead of being
    // invalidated. GoldenCorpus.TheDataScenariosReproduceTheRetiredBuilderCorpus
    // checks that claim over precisely those entries. A module appended to the
    // standard set registers after them, so the standard walk's 22 entries
    // keep their indices too.
    if (Result<void> appended = register_module_arrays(false); !appended) {
        return std::unexpected(appended.error());
    }

    // THE BUILT-INS' TYPED IDS, minted from the table by name.
    // check_builtin_arrays() above refused a set that omits one of these or
    // declares it with another shape, so each lookup finds its array with its
    // row's size; a misspelt name here fails every create(), never silently.
    const auto mint = [&sim]<class Row>(ArrayId<Row>& id, std::string_view name) -> Result<void> {
        const Result<ArrayIndex> index = sim.module_array(name);
        if (!index) return std::unexpected(index.error());
        const Result<ArrayId<Row>> typed = sim.arenas_.typed<Row>(*index);
        if (!typed) return std::unexpected(typed.error());
        id = *typed;
        return {};
    };
    for (const Result<void>& minted :
         {mint(sim.drag_id_, "drag_bodies"), mint(sim.dryden_id_, "dryden"), mint(sim.imu_id_, "imu_sensors"),
          mint(sim.imu_ring_id_, "imu_ring"), mint(sim.rotors_id_, "rotors"), mint(sim.gnss_id_, "gnss_sensors"),
          mint(sim.gnss_ring_id_, "gnss_ring")}) {
        if (!minted) return std::unexpected(minted.error());
    }

    // -----------------------------------------------------------------------
    // Seed the per-world rows.
    // -----------------------------------------------------------------------
    Result<std::span<WorldParams>> params = sim.arenas_.array(sim.world_params_id_);
    if (!params) return std::unexpected(params.error());

    for (uint32_t w = 0; w < layout->world_count; ++w) {
        const Environment& env = desc.worlds[w].world.environment;
        WorldParams& row = (*params)[w];

        // FIELD-WISE, NOT WHOLE-OBJECT. WorldParams has eight bytes of real,
        // addressable TAIL PADDING (layout.hpp: named fields end at 56, the
        // std430 stride is 64). ArenaSet zero-fills arena storage, and
        // test_state.cpp pins that those bytes read as zero in a snapshot;
        // assigning a stack temporary over the row would copy that temporary's
        // indeterminate padding instead, and two otherwise identical runs could
        // then differ byte-wise. Every write to an arena row in this file is
        // field-wise for that reason.
        row.gravity = env.gravity;
        row.air_density = env.air_density;
        row.wind = env.wind;
        row._p = 0.0f;
        // The ARENA PARTITION size, which is the set's maximum -- this is the
        // bound a dispatch range-checks against. The world's own declared
        // capacity is the smaller number spawn() enforces; see WorldSetLayout.
        row.body_capacity = layout->body_capacity;
        row.body_count = 0;
        row.seed = desc.worlds[w].seed;
        row._reserved0 = 0;

        // THE DECLARED STREAMS (sim/module.hpp's SEEDED STREAMS), derived from
        // row.seed, so this must come AFTER the seed is written. No attached
        // row is live yet, so this derives the per_world ones: in the standard
        // set, dryden's, whose dryden_init also places the filter on its
        // stationary distribution so the world starts gusty rather than
        // burning off a spin-up transient. reseed() runs the same walk.
        if (Result<void> derived = sim.derive_streams(w); !derived) return std::unexpected(derived.error());
    }

    // -----------------------------------------------------------------------
    // The run's identity, written ONCE and never again by anything in this
    // engine. Read the ReplayConfig doc comment in the header for why a
    // constant lives in the state arenas at all.
    //
    // config_hash() is computed from the DESC, here, at the only moment the
    // desc is in scope: Simulation does not retain it (WorldConfig is a lossy
    // projection of it, by design), so this is the one and only chance to take
    // its identity. Which also means the hash covers the CREATING desc for the
    // life of the object -- reseed() rewrites WorldParams::seed rows without
    // touching the desc, so it deliberately does not touch this row either.
    // See sim/world_set.hpp's fold-order contract.
    // -----------------------------------------------------------------------
    Result<std::span<ReplayConfig>> replay_config = sim.arenas_.array(sim.replay_config_id_);
    if (!replay_config) return std::unexpected(replay_config.error());
    const uint64_t hash = config_hash(desc);
    for (uint32_t w = 0; w < layout->world_count; ++w) {
        // FIELD-WISE, like every other arena write in this file. ReplayConfig
        // has no implicit padding (the battery in the header pins that), so a
        // whole-object assignment would in fact be safe here -- but the
        // discipline is uniform on purpose: the day a field is added, the safe
        // form is already the form in use.
        ReplayConfig& row = (*replay_config)[w];
        row.dt_ns = dt_ns;
        row.substeps = substeps;
        row._pad = 0;
        row.config_hash = hash;
        row._reserved0 = 0;
    }

    sim.views_.resize(layout->world_count);
    // Every pass's declared-state views (module-API stage 4, Task 6), sized
    // once; rebuild_views() below fills them.
    sim.pass_view_begin_.reserve(sim.schedule_.passes.size() + 1);
    sim.pass_view_begin_.push_back(0);
    for (const modules::CompiledPass& pass : sim.schedule_.passes) {
        sim.pass_view_begin_.push_back(sim.pass_view_begin_.back() + static_cast<uint32_t>(pass.state.size()));
    }
    sim.pass_views_.assign(sim.pass_view_begin_.back(), StateView{});
    // The field sample rows, sized once and zero-filled (module-API stage 3).
    sim.field_rows_.assign(static_cast<std::size_t>(layout->world_count) * sim.schedule_.field_stride, 0.0f);
    // The configuration tables (module-API stage 4, Task 7), built here from
    // no models -- so a pass that reads one before any model is registered
    // sees an empty table -- and rebuilt by register_model().
    sim.config_tables_.assign(sim.schedule_.tables.size(), std::vector<float>{});
    sim.rebuild_config_tables();
    // One-shot sizing so the broad phase never allocates in the steady state
    // (physics/grid.hpp's GridScratch note). Worst case is one entry and one
    // run per body slot in the whole set.
    sim.scratch_.reserve(static_cast<std::size_t>(layout->world_count) *
                         static_cast<std::size_t>(layout->body_capacity));
    // The contact scratch, sized and zero-filled once (PHY-7): the step never
    // allocates it, and it is zero between steps.
    sim.contact_dv_.assign(static_cast<std::size_t>(layout->world_count) *
                               static_cast<std::size_t>(layout->body_capacity),
                           glm::vec3(0.0f));

    if (Result<void> views = sim.rebuild_views(); !views) {
        return std::unexpected(views.error());
    }

    // -----------------------------------------------------------------------
    // S6 Task 5: the vulkan-path backend. StepShape mirrors the VALIDATED
    // layout this function already derived above (`*layout`) plus the two
    // scalars (substeps, h) and the config flag this function also already
    // has in scope -- see compute/backend.hpp's StepShape doc comment for
    // why it carries exactly these seven fields and no others.
    // -----------------------------------------------------------------------
    if (backend.kind == compute::BackendKind::vulkan) {
        // NO UNPORTED-PASS GATE HERE ANY MORE (S6 Task 8). Through Task 7 this
        // was where check_vulkan_unported_config() refused a TURBULENT world
        // set, because MediumUpdate was a stub and its gusts would have been
        // silently absent from the trajectory. All eight schedule slots now
        // have a kernel or are inert by design on both backends, so there is
        // nothing left to refuse -- see simulation.hpp's note where that
        // function and its state-time twin used to be declared.

        // The SDF extents ARE shape (compute/backend.hpp's StepShape note):
        // they size two device buffers and are fixed for this Simulation's
        // life, because the per-world SdfProgram is configuration.
        uint32_t sdf_node_count = 0;
        uint32_t sdf_transform_count = 0;
        for (const WorldConfig& config : sim.configs_) {
            sdf_node_count += static_cast<uint32_t>(config.sdf.nodes.size());
            sdf_transform_count += static_cast<uint32_t>(config.sdf.transforms.size());
        }

        compute::StepShape shape{};
        shape.world_count = layout->world_count;
        shape.body_capacity = layout->body_capacity;
        shape.element_capacity = layout->element_capacity;
        shape.sensor_capacity = layout->sensor_capacity;
        shape.substeps = substeps;
        shape.h = h;
        shape.batch_dynamic_collision = layout->uniform_dynamic_params;
        shape.sdf_node_count = sdf_node_count;
        shape.sdf_transform_count = sdf_transform_count;

        // The registered walk, as data (module-API stage 4): the mirror makes
        // one device buffer per entry, so every module array -- a developer's
        // included -- reaches the device from its declaration. Registration
        // above already refused any capacity this refuses.
        Result<std::vector<compute::StateArrayShape>> arrays = state_array_shapes(sim.schedule_, shape);
        if (!arrays) return std::unexpected(arrays.error());
        shape.arrays = std::move(*arrays);

        // NO RunParams ARGUMENT AS OF S6 TASK 6b (checkpoint-1 ruling):
        // PassParams no longer carries a per-batch ContactParams/GridParams
        // (compute/vulkan/step_recorder.hpp's PassParams doc comment has the
        // ruling), so VulkanBackend::create() takes the shape and the GPU chain
        // -- the compiled schedule's passes, in order (module-API plan, stage
        // 2). Per-world material still reaches the device -- see the
        // contact_params and grid_params uploads a few lines further down.
        const std::vector<compute::GpuPass> gpu_chain = modules::gpu_passes(sim.schedule_);
        Result<std::unique_ptr<compute::VulkanBackend>> vulkan_backend =
            compute::VulkanBackend::create(backend, shape, gpu_chain);
        if (!vulkan_backend) return std::unexpected(vulkan_backend.error());
        sim.vulkan_backend_ = std::move(*vulkan_backend);

        // dryden_params (binding 13): uploaded exactly ONCE, here, not on
        // every step() -- it is per-world CONFIGURATION
        // (WorldConfig::turbulence, immutable for a Simulation's lifetime),
        // not registered state, so it never appears in the ArenaSet
        // upload()/readback() exchange step() drives. `sim.configs_` is
        // used (not the local `configs` above, already moved-from into
        // `sim` by this point) and copied into a CONTIGUOUS buffer first:
        // WorldConfig interleaves turbulence with sdf/contacts/grid, so a
        // span directly over sim.configs_ would have the wrong stride.
        std::vector<DrydenParams> dryden_params_upload;
        dryden_params_upload.reserve(layout->world_count);
        for (uint32_t w = 0; w < layout->world_count; ++w) {
            dryden_params_upload.push_back(sim.configs_[w].turbulence);
        }
        const std::span<const std::byte> dryden_bytes =
            std::as_bytes(std::span<const DrydenParams>(dryden_params_upload));
        if (Result<void> uploaded = sim.vulkan_backend_->upload_dryden_params(
                dryden_bytes, static_cast<uint32_t>(sizeof(DrydenParams)), layout->world_count);
            !uploaded) {
            return std::unexpected(uploaded.error());
        }

        // -----------------------------------------------------------------
        // S6 Task 6: the same once-only, config-not-state upload, for the two
        // records CollisionStatic reads.
        //
        // THE SDF PROGRAMS (bindings 14-16). Flattened -- every world's nodes
        // and transforms concatenated, plus a per-world range record -- by
        // compute/sdf_program.hpp, which is also where the "why three buffers"
        // and "why the node's transform index stays world-local" reasoning
        // lives. A pointer vector is built first because flatten_sdf_programs()
        // takes a span of programs and WorldConfig interleaves the SdfProgram
        // with three other members, so a span over configs_ would have the
        // wrong stride -- the identical reason the dryden upload above copies
        // into a contiguous buffer.
        // -----------------------------------------------------------------
        std::vector<const SdfProgram*> programs;
        programs.reserve(layout->world_count);
        for (uint32_t w = 0; w < layout->world_count; ++w) {
            programs.push_back(&sim.configs_[w].sdf);
        }
        const compute::SdfUpload sdf =
            compute::flatten_sdf_programs(std::span<const SdfProgram* const>(programs));
        if (Result<void> uploaded = sim.vulkan_backend_->upload_sdf_program(
                std::as_bytes(std::span<const compute::SdfNodeRow>(sdf.nodes)),
                std::as_bytes(std::span<const compute::SdfTransformRow>(sdf.transforms)),
                std::as_bytes(std::span<const compute::SdfWorldRange>(sdf.ranges)));
            !uploaded) {
            return std::unexpected(uploaded.error());
        }

        // THE PER-WORLD CONTACT MATERIAL (binding 19). Per world, not per run:
        // tests/golden/scenarios/bounce.scenario.yaml is a restitution ladder
        // whose four worlds carry four different records, and CollisionStatic
        // reads its own world's row exactly as the cpu pass reads `w.contacts`.
        std::vector<physics::ContactParams> contacts_upload;
        contacts_upload.reserve(layout->world_count);
        for (uint32_t w = 0; w < layout->world_count; ++w) {
            contacts_upload.push_back(sim.configs_[w].contacts);
        }
        if (Result<void> uploaded = sim.vulkan_backend_->upload_contact_params(
                std::as_bytes(std::span<const physics::ContactParams>(contacts_upload)),
                static_cast<uint32_t>(sizeof(physics::ContactParams)), layout->world_count);
            !uploaded) {
            return std::unexpected(uploaded.error());
        }

        // THE PER-WORLD GRID CONFIG (binding 20, S6 Task 6b -- checkpoint-1
        // ruling). Same per-world reasoning as contact_params immediately
        // above, and the direct closure of that task: a world set may be
        // heterogeneous, so a single per-dispatch cell_size cannot serve
        // every world. No kernel reads this buffer yet (T7's CollisionDynamic
        // is the first consumer -- see bindings.slang binding 20's comment),
        // but it is uploaded now, unconditionally, so a world set built today
        // is already correct once that kernel lands.
        std::vector<physics::GridParams> grid_upload;
        grid_upload.reserve(layout->world_count);
        for (uint32_t w = 0; w < layout->world_count; ++w) {
            grid_upload.push_back(sim.configs_[w].grid);
        }
        if (Result<void> uploaded = sim.vulkan_backend_->upload_grid_params(
                std::as_bytes(std::span<const physics::GridParams>(grid_upload)),
                static_cast<uint32_t>(sizeof(physics::GridParams)), layout->world_count);
            !uploaded) {
            return std::unexpected(uploaded.error());
        }
    }

    return sim;
}

// A linear scan in set order: the table is a handful of rows, and a lookup
// happens at create() and in host calls, never in a step.
Result<ArrayIndex> Simulation::module_array(std::string_view name) const {
    for (std::size_t i = 0; i < schedule_.arrays.size(); ++i) {
        if (schedule_.arrays[i].name == name) return module_array_ids_[i];
    }
    return std::unexpected(
        missing("module_array: no module in this set declares an array '" + std::string(name) + "'"));
}

// A linear scan, like module_array(): a host call, never in a step.
Result<std::span<const float>> Simulation::config_table(std::string_view name) const {
    for (std::size_t i = 0; i < schedule_.tables.size(); ++i) {
        if (schedule_.tables[i].name == name) return std::span<const float>(config_tables_[i]);
    }
    return std::unexpected(
        missing("config_table: no module in this set declares a table '" + std::string(name) + "'"));
}

void Simulation::rebuild_config_tables() {
    for (std::size_t i = 0; i < schedule_.tables.size(); ++i) {
        // EMPTY, then built: ConfigBuildFn's contract. clear() keeps the
        // capacity, so a table that does not grow is rebuilt in place.
        config_tables_[i].clear();
        schedule_.tables[i].build(std::span<const vehicles::ModelType>(models_), config_tables_[i]);
    }
}

// ---------------------------------------------------------------------------
// views
// ---------------------------------------------------------------------------
//
// SIX ARRAYS ARE DIRECT-INDEXED RATHER THAN SLOT-ALLOCATED: world_params,
// dryden and replay_config (one row per world, indexed by world id -- they have
// no lifecycle, so alloc_slot/free_slot would be ceremony around a constant),
// body_generation (one row per BODY slot, indexed by that slot -- it must
// SURVIVE its slot being freed, and free_slot zero-fills, which would erase the
// very counter it exists to preserve), and imu_ring and gnss_ring (kRingDepth
// rows per SENSOR slot, indexed by that slot -- a ring's lifetime IS its
// sensor's, so a second alloc/free lifecycle would only be a thing to keep in
// step; the despawn cascade, free_rows_of(), zeroes a sensor's window when it
// frees the sensor's row). Every per_world, per_body and per_row module array
// is direct-indexed the same way.
//
// The consequence is that those arrays' slot->world maps stay uniformly
// kInvalidWorld. They are still carried by the registry walk, still snapshotted
// and still resynced on restore -- an all-free map resyncs to an all-free set,
// which is consistent -- and they cost four bytes a slot for the uniformity of
// having exactly one way to obtain arena storage.
// ---------------------------------------------------------------------------

Result<void> Simulation::rebuild_views() {
    Result<std::span<BodyState>> bodies = arenas_.array(bodies_id_);
    if (!bodies) return std::unexpected(bodies.error());
    Result<std::span<const uint32_t>> body_map = arenas_.slot_to_world(bodies_id_);
    if (!body_map) return std::unexpected(body_map.error());
    Result<std::span<physics::DragBodyRow>> drag = arenas_.array(drag_id_);
    if (!drag) return std::unexpected(drag.error());
    Result<std::span<WorldParams>> params = arenas_.array(world_params_id_);
    if (!params) return std::unexpected(params.error());
    Result<std::span<DrydenState>> dryden = arenas_.array(dryden_id_);
    if (!dryden) return std::unexpected(dryden.error());
    Result<std::span<sensors::ImuSensorRow>> imu = arenas_.array(imu_id_);
    if (!imu) return std::unexpected(imu.error());
    Result<std::span<sensors::ImuSample>> imu_ring = arenas_.array(imu_ring_id_);
    if (!imu_ring) return std::unexpected(imu_ring.error());
    Result<std::span<sensors::GnssSensorRow>> gnss = arenas_.array(gnss_id_);
    if (!gnss) return std::unexpected(gnss.error());
    Result<std::span<sensors::GnssFix>> gnss_ring = arenas_.array(gnss_ring_id_);
    if (!gnss_ring) return std::unexpected(gnss_ring.error());
    Result<std::span<vehicles::RotorRow>> rotors = arenas_.array(rotors_id_);
    if (!rotors) return std::unexpected(rotors.error());

    const std::size_t ring_per_world =
        static_cast<std::size_t>(layout_.sensor_capacity) * sensors::kRingDepth;

    for (uint32_t w = 0; w < layout_.world_count; ++w) {
        const std::size_t body_begin = static_cast<std::size_t>(w) * layout_.body_capacity;
        const std::size_t elem_begin = static_cast<std::size_t>(w) * layout_.element_capacity;
        const std::size_t sensor_begin = static_cast<std::size_t>(w) * layout_.sensor_capacity;

        // THE PAIRING POINT. Every per-world binding a pass reads -- the param
        // row, the body/element partitions, the turbulence filter state AND its
        // parameters -- is subscripted by the SAME `w` here, and this is the
        // only place any of them is bound. That is the whole guarantee that
        // dryden.sample writes world w's field row from world w's filter
        // alongside world w's WorldParams row: one world's air with another's
        // gusts is not expressible, rather than being a runtime assert away.
        //
        // A cheap runtime cross-check on WorldParams::seed was considered and
        // is not possible: DrydenState's stream is derived from the seed but
        // advances with every draw, so after the first substep the seed is not
        // recoverable from the row. The property is covered instead by the
        // batching-invariance test -- a crossed pairing would make world w
        // inside a four-world set diverge from the same world run alone, which
        // is precisely what that test asserts cannot happen.
        physics::WorldSubstepView& view = views_[w];
        view.params = &(*params)[w];
        view.bodies = bodies->subspan(body_begin, layout_.body_capacity);
        view.drag_elements =
            std::span<const physics::DragBodyRow>(drag->subspan(elem_begin, layout_.element_capacity));
        view.body_slot_to_world = body_map->subspan(body_begin, layout_.body_capacity);
        view.contact_dv = std::span<glm::vec3>(contact_dv_).subspan(body_begin, layout_.body_capacity);
        view.dryden = &(*dryden)[w];
        view.dryden_params = &configs_[w].turbulence;
        view.fields = std::span<float>(field_rows_).subspan(static_cast<std::size_t>(w) * schedule_.field_stride,
                                                            schedule_.field_stride);
        view.sdf = &configs_[w].sdf;
        view.contacts = configs_[w].contacts;
        view.grid = configs_[w].grid;
        view.imu_sensors = imu->subspan(sensor_begin, layout_.sensor_capacity);
        // Subscripted by the SAME `w` as everything above -- the pairing point
        // again. A world's sensors and its ring storage are bound together
        // here and nowhere else, which is what makes
        // "imu_ring.subspan(i * kRingDepth, kRingDepth) is sensor i's window"
        // true inside the pass without the pass knowing a world id.
        view.imu_ring = imu_ring->subspan(static_cast<std::size_t>(w) * ring_per_world, ring_per_world);
        // The second sensor kind, bound by the SAME `w` and the SAME
        // sensor_capacity -- so a receiver's world-local body_slot agrees with
        // the `bodies` span above exactly as an IMU's does, and its ring window
        // arithmetic is the same expression. The two arenas are separate but
        // their partitioning is not: one sensor_capacity sizes both.
        view.gnss_sensors = gnss->subspan(sensor_begin, layout_.sensor_capacity);
        view.gnss_ring =
            gnss_ring->subspan(static_cast<std::size_t>(w) * ring_per_world, ring_per_world);
        // Subscripted by the SAME `w` and, crucially, by the SAME
        // element_capacity as `drag_elements` above -- which is what makes
        // RotorRow::body_slot (a WORLD-LOCAL body index, exactly like
        // DragBodyRow's) agree with the `bodies` span bound three lines up.
        view.rotors = rotors->subspan(elem_begin, layout_.element_capacity);
    }

    // THE PASSES' DECLARED STATE (module-API stage 4, Task 6), refilled in
    // place: each declared access's view is the array its binding names --
    // all worlds' rows, world-contiguous, which world_rows() slices by the
    // same `w` the views above use -- or absent. The arenas never move, so
    // this rewrites the same values every step; refilling rather than caching
    // keeps the one rule this function exists for.
    //
    // A CONFIGURATION TABLE (Task 7) is one row of floats that every world
    // shares: world_count 1, elem_size 4, as many rows as it has floats. Its
    // storage moves only in register_model(), which never runs inside a step,
    // and this runs at the top of every step. An empty table -- no model
    // registered yet -- still reads as present(), since its module is in the
    // set: its view points at g_no_table_rows and has no rows.
    for (std::size_t p = 0; p < schedule_.passes.size(); ++p) {
        const std::vector<modules::CompiledBinding>& bindings = schedule_.passes[p].state;
        for (std::size_t k = 0; k < bindings.size(); ++k) {
            StateView& view = pass_views_[pass_view_begin_[p] + k];
            view = StateView{};
            if (bindings[k].kind == modules::BindingKind::table) {
                std::vector<float>& table = config_tables_[bindings[k].index];
                view.data = table.empty() ? g_no_table_rows : reinterpret_cast<std::byte*>(table.data());
                view.elem_size = static_cast<uint32_t>(sizeof(float));
                view.world_count = 1;
                view.capacity_per_world = static_cast<uint32_t>(table.size());
                continue;
            }
            if (bindings[k].kind != modules::BindingKind::array) continue;
            const ArrayIndex id = module_array_ids_[bindings[k].index];
            const Result<std::span<std::byte>> bytes = arenas_.bytes(id);
            if (!bytes) return std::unexpected(bytes.error());
            const Result<WorldRange> world0 = arenas_.range(id, 0);  // its count is the per-world capacity
            if (!world0) return std::unexpected(world0.error());
            view.data = bytes->data();
            view.elem_size = schedule_.arrays[bindings[k].index].elem_size;
            view.world_count = layout_.world_count;
            view.capacity_per_world = world0->count;
        }
    }
    return {};
}

// ---------------------------------------------------------------------------
// step
// ---------------------------------------------------------------------------

Result<void> Simulation::step(uint64_t n) {
    if (n == 0) {
        // "Advance zero steps" is still a step boundary, so the queue is still
        // applied. Anything else would make step(0) observably different from
        // step(1) minus the physics. Shared by both paths: the vulkan branch
        // below never runs for n == 0 either.
        return flush_structural();
    }

    // -------------------------------------------------------------------
    // S6 Task 5: the vulkan path. Flush ONCE (not per-step the way the cpu
    // loop below does -- see step()'s header doc comment for why that is
    // sound today), upload the device mirror if the flush (or create()'s
    // initial vulkan_dirty_ == true) made it stale, submit all `n` steps in
    // one VulkanBackend::step() call, read the result back, and advance the
    // tick by `n`. The cpu schedule (schedule_, the compiled modules) never runs on
    // this path -- the GPU is authoritative for these `n` steps.
    // -------------------------------------------------------------------
    if (vulkan_backend_) {
        // Behaviors run only in the cpu schedule; their GPU recipes record
        // nothing (CORE-1). So an attached registry is refused here, before
        // anything moves, rather than skipped (SL6: a refusal, never a silent
        // fallback). Any non-empty registry counts, including one whose
        // behaviors all declare a record_gpu half, because nothing calls that
        // half yet. step(0) above is unaffected: it runs no behaviors.
        if (behaviors_ != nullptr && behaviors_->size() > 0) {
            return std::unexpected(Error{Code::unavailable,
                                         "behaviors are CPU-only today; the Vulkan step cannot run "
                                         "them and refuses rather than skipping them (SL6)"});
        }
        if (Result<void> flushed = flush_structural(); !flushed) {
            return flushed;
        }
        if (vulkan_dirty_) {
            // NO STATE-TIME GATE HERE ANY MORE (S6 Task 8). Through Task 7 this
            // was where check_vulkan_unported_state() scanned for a live rotor
            // or a live IMU sensor and refused the step, because the
            // RotorElement half of ForceElements and SensorSynthesis were
            // stubs. Both are ported; see simulation.hpp for the full note.
            if (Result<void> uploaded = vulkan_backend_->upload(arenas_); !uploaded) {
                return uploaded;
            }
            vulkan_dirty_ = false;
        }
        if (Result<void> stepped = vulkan_backend_->step(n, tick_.value); !stepped) {
            return stepped;
        }
        if (Result<void> read_back = vulkan_backend_->readback(arenas_); !read_back) {
            return read_back;
        }
        tick_ = Tick{tick_.value + n};
        return {};
    }

    Result<std::span<BodyState>> all_bodies = arenas_.array(bodies_id_);
    if (!all_bodies) return std::unexpected(all_bodies.error());
    Result<std::span<const uint32_t>> all_map = arenas_.slot_to_world(bodies_id_);
    if (!all_map) return std::unexpected(all_map.error());

    for (uint64_t i = 0; i < n; ++i) {
        // THE STEP BOUNDARY. Structural changes land here and nowhere else.
        if (Result<void> flushed = flush_structural(); !flushed) {
            return flushed;
        }
        // Rebuilt every step rather than cached: O(worlds), and it makes a view
        // that outlived a restore impossible to hold.
        if (Result<void> views = rebuild_views(); !views) {
            return views;
        }

        physics::SubstepContext ctx;
        ctx.worlds = std::span<const physics::WorldSubstepView>(views_);
        ctx.all_bodies = *all_bodies;
        ctx.all_slot_to_world = *all_map;
        ctx.batch_dynamic_collision = layout_.uniform_dynamic_params;
        ctx.dynamic_contacts = configs_[0].contacts;
        ctx.dynamic_grid = configs_[0].grid;
        ctx.scratch = &scratch_;
        ctx.all_contact_dv = contact_dv_;
        ctx.h = h_;
        // dt, not h: Tick counts STEPS, and a behavior deriving a pose from the
        // tick needs what one tick is worth. Same ns -> s conversion as
        // substep_h(), so the two agree by construction.
        ctx.dt_s = static_cast<float>(dt_ns_) / 1.0e9f;
        ctx.behaviors = behaviors_;  // nullptr unless set_behaviors() was called
        // The step being executed -- what SensorSynthesis stamps samples with.
        // Taken BEFORE the increment below, so every substep of step k stamps
        // k (Tick counts steps, not substeps).
        ctx.tick = tick_;

        // The compiled module schedule (sim/module.hpp), in order, every
        // substep. The order is the parity contract; create() fixed it. Each
        // pass sees its own declared state, and only that (Task 6).
        const std::span<const StateView> pass_views(pass_views_);
        for (uint32_t s = 0; s < substeps_; ++s) {
            for (std::size_t p = 0; p < schedule_.passes.size(); ++p) {
                ctx.state = pass_views.subspan(pass_view_begin_[p], pass_view_begin_[p + 1] - pass_view_begin_[p]);
                schedule_.passes[p].cpu(ctx);
            }
        }

        // Tick counts STEPS, not substeps (engine A9). One increment, after the
        // last substep.
        ++tick_;
    }

    return {};
}

// ---------------------------------------------------------------------------
// structural queue
// ---------------------------------------------------------------------------

uint64_t Simulation::vulkan_upload_count() const noexcept {
    return vulkan_backend_ ? vulkan_backend_->upload_count() : 0;
}

// See simulation.hpp for the CENSUS this function exists to make checkable --
// every public entry point that can change `arenas_` between two step() calls,
// and how each one is covered.
void Simulation::mark_vulkan_dirty() noexcept {
    if (vulkan_backend_) {
        vulkan_dirty_ = true;
    }
}

Result<compute::StepWitness> Simulation::vulkan_step_witness() const {
    if (!vulkan_backend_) {
        return std::unexpected(Error{Code::unavailable,
                                     "vulkan_step_witness: this Simulation runs on the cpu backend"});
    }
    compute::StepWitness witness{};
    std::byte bytes[sizeof(compute::StepWitness)];
    if (Result<void> read = vulkan_backend_->read_step_witness(std::span<std::byte>(bytes, sizeof(bytes)));
        !read) {
        return std::unexpected(read.error());
    }
    std::memcpy(&witness, bytes, sizeof(witness));
    return witness;
}

Result<std::vector<compute::GridEntryRow>> Simulation::vulkan_grid_entries() const {
    if (!vulkan_backend_) {
        return std::unexpected(Error{Code::unavailable,
                                     "vulkan_grid_entries: this Simulation runs on the cpu backend"});
    }
    const std::size_t bytes = vulkan_backend_->grid_entries_byte_size();
    std::vector<compute::GridEntryRow> rows(bytes / sizeof(compute::GridEntryRow));
    if (Result<void> read = vulkan_backend_->read_grid_entries(
            std::span<std::byte>(reinterpret_cast<std::byte*>(rows.data()), bytes));
        !read) {
        return std::unexpected(read.error());
    }
    return rows;
}

Result<compute::PassDurationsNs> Simulation::vulkan_pass_durations_ns() const {
    if (!vulkan_backend_) {
        return std::unexpected(Error{Code::unavailable,
                                     "vulkan_pass_durations_ns: this Simulation runs on the cpu backend"});
    }
    return vulkan_backend_->read_pass_durations_ns();
}

Result<compute::RecordedChain> Simulation::vulkan_recorded_chain() const {
    if (!vulkan_backend_) {
        return std::unexpected(Error{Code::unavailable,
                                     "vulkan_recorded_chain: this Simulation runs on the cpu backend"});
    }
    return vulkan_backend_->recorded_chain();
}

Result<compute::DeviceReport> Simulation::vulkan_device_report() const {
    if (!vulkan_backend_) {
        return std::unexpected(Error{Code::unavailable,
                                     "vulkan_device_report: this Simulation runs on the cpu backend"});
    }
    return vulkan_backend_->device_report();
}

Result<void> Simulation::flush_structural() {
    if (queue_.empty()) {
        return {};
    }

    // S6 Task 5: a non-empty queue is about to mutate arenas_, which makes
    // the vulkan-path device mirror stale the moment it does. Set
    // unconditionally before applying (not after) -- apply_op() below can
    // fail partway, and a partially-applied queue is still a mutation the
    // mirror does not yet reflect.
    mark_vulkan_dirty();

    // FIFO over a std::vector: insertion order, front to back, no hashing, no
    // pointer ordering, nothing whose iteration order is unspecified. This loop
    // IS the "deterministic slot order" the brief asks for -- slot order follows
    // from call order because ArenaSet hands out the lowest free slot and the
    // reservations happened in call order.
    for (const StructuralOp& op : queue_) {
        if (Result<void> applied = apply_op(op); !applied) {
            // Every op was validated when it was queued and every slot it
            // touches was reserved then, so reaching here means an invariant of
            // this class is broken rather than a caller error. The queue is
            // cleared regardless: leaving half-applied ops queued would make the
            // next step apply them twice.
            queue_.clear();
            publish_body_counts();
            return applied;
        }
    }
    queue_.clear();
    publish_body_counts();
    return {};
}

Result<void> Simulation::apply_op(const StructuralOp& op) {
    switch (op.kind) {
        case OpKind::init_body: {
            Result<std::span<BodyState>> bodies = arenas_.array(bodies_id_);
            if (!bodies) return std::unexpected(bodies.error());
            if (op.slot >= bodies->size()) {
                return std::unexpected(internal("structural queue: body slot out of range"));
            }
            BodyState& b = (*bodies)[op.slot];

            // FIELD-WISE, NOT WHOLE-OBJECT -- same reason as create(): the
            // slot is zero-filled (ArenaSet zeroes at registration and again on
            // free), so writing fields one by one leaves BodyState's std430
            // pads at zero, whereas assigning a stack temporary over the row
            // would copy that temporary's padding instead.
            //
            // The three accumulators and specific_force are written explicitly
            // even though the slot is already zero. They are not redundant: the
            // spawn contract is "initializes BodyState FULLY", and a body must
            // start a step with no inherited wrench regardless of how its slot
            // came to be free. Relying on the zero-fill for state that has a
            // meaning would make this correct by coincidence. `proxy_radius`
            // (D-S6-2) is the same discipline: op.body_proxy_radius is 0.0f for
            // a plain spawn() and the model's own value for a vehicle spawn, and
            // writing it explicitly here -- rather than trusting the zero-fill
            // for the common (plain-body) case -- is what keeps the field's
            // value a stated fact of every spawn rather than an accident of two
            // of its callers happening to agree.
            b.pos = op.body.pos;
            b.proxy_radius = op.body_proxy_radius;  // D-S6-2; see StructuralOp's field comment
            b.orient = glm::normalize(op.body.orient);
            b.vel = op.body.vel;
            b.mass = op.body.mass;
            b.omega_body = op.body.omega_body;
            b.inv_inertia_diag = op.body.inv_inertia_diag;
            b.force_acc = glm::vec3(0.0f);
            b.torque_acc = glm::vec3(0.0f);
            b.specific_force = glm::vec3(0.0f);
            // WITHOUT THIS THE BODY NEVER MOVES. body_flags::active is
            // active-high precisely so a zeroed slot is inert, which makes
            // setting it the spawn path's non-negotiable job
            // (physics/integrator.hpp).
            b.flags = physics::body_flags::active;
            return {};
        }

        case OpKind::free_body: {
            // Cascade first: the attached rows are addressed relative to the
            // body that is about to stop existing.
            free_rows_of(op.world_index, op.slot);
            if (Result<void> freed = arenas_.free_slot(bodies_id_, op.slot); !freed) {
                return std::unexpected(internal("structural queue: freeing a body slot failed: " +
                                                freed.error().context));
            }
            return {};
        }

        case OpKind::init_row: {
            if (op.array >= schedule_.arrays.size()) {
                return std::unexpected(internal("structural queue: init_row names no module array"));
            }
            const modules::CompiledArray& array = schedule_.arrays[op.array];
            const ArrayIndex id = module_array_ids_[op.array];
            Result<std::span<std::byte>> bytes = arenas_.bytes(id);
            if (!bytes) return std::unexpected(bytes.error());
            const std::size_t row_bytes = array.elem_size;
            if (std::size_t{op.slot} >= bytes->size() / row_bytes) {
                return std::unexpected(internal("structural queue: row slot out of range in array '" + array.name +
                                                "'"));
            }
            // DEFENCE IN DEPTH: if this row's slot -- for a per_body row, its
            // body's -- was released earlier in this same flush, its body no
            // longer exists and there is nothing to initialize. With `body_slot`
            // written at reservation time (see attach_row_impl) the cascade
            // cannot reach a row belonging to a DIFFERENT body, and a row
            // belonging to the SAME body is always initialized before that
            // body's free is applied (a ref for a body whose despawn is queued
            // is rejected, so the row can only have been reserved first). So
            // this is unreachable today -- and it is cheap insurance that a
            // future queue reordering degrades to a no-op rather than to a live
            // row in a free slot.
            //
            // WHOSE MAP SAYS A ROW IS LIVE. A slot-allocated row's own; a
            // per_body row's BODY's, at the same slot; and a per_row row's
            // OWNER's, at row slot / depth (Task 7), read by the owner's own
            // rule -- the body map for a per_body owner, else the owner's map.
            // A per_row array's own map is never written (it is direct-indexed,
            // like the rings), so reading it would skip every such row in
            // silence: Task 4's check did, for every row a rotor owns.
            uint32_t live_array = op.array;
            uint32_t live_slot = op.slot;
            if (array.extent == modules::Extent::per_row) {
                live_array = array.owner;
                live_slot = op.slot / array.depth;
            }
            const ArrayIndex liveness = schedule_.arrays[live_array].extent == modules::Extent::per_body
                                            ? ArrayIndex(bodies_id_)
                                            : module_array_ids_[live_array];
            const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(liveness);
            if (!map) return std::unexpected(map.error());
            if (live_slot >= map->size() || (*map)[live_slot] != op.world_index) return {};

            const Result<std::span<const WorldParams>> params = arenas_.array(world_params_id_);
            if (!params) return std::unexpected(params.error());
            const Result<WorldRange> range = arenas_.range(id, op.world_index);
            if (!range) return std::unexpected(range.error());
            // The module's init writes the row (sim/builtin_state.cpp has the
            // built-ins'): field-wise, every field, its liveness flag last.
            // Both slots it is handed are WORLD-LOCAL, so a world's rows -- and
            // the noise streams seeded from them -- do not depend on where that
            // world sits in the set.
            array.init(modules::RowInit{
                .row = bytes->subspan(std::size_t{op.slot} * row_bytes, row_bytes),
                .spawn = std::span<const std::byte>(op.spawn.data(), op.spawn_size),
                .params = &(*params)[op.world_index],
                .local_slot = op.slot - range->begin,
                .local_body = op.body_slot - op.world_index * layout_.body_capacity,
                .h = h_,
            });
            return {};
        }
    }
    return std::unexpected(internal("structural queue: unknown op kind"));
}

void Simulation::free_rows_of(uint32_t world_index, uint32_t body_slot) {
    const uint32_t local_body = body_slot - world_index * layout_.body_capacity;
    // WALK ORDER (schedule_.arrays' order), so the cascade is a function of the
    // declarations alone. It frees rotors before gnss_sensors, where the hand
    // cascade it replaced freed them after; the bytes and every later
    // allocation are the same, because each free touches only its own array's
    // bytes, map and free list, and alloc_slot() consults only that array's
    // free set (state/arenas.hpp). ModuleRows.InterleavedChurnLeavesTodaysSlots
    // AndBytes pins it against the hand cascade's digests.
    for (uint32_t i = 0; i < schedule_.arrays.size(); ++i) {
        const modules::CompiledArray& array = schedule_.arrays[i];
        const ArrayIndex id = module_array_ids_[i];
        if (array.extent == modules::Extent::per_body) {
            // The body's own row, direct-indexed by body slot: nothing to
            // release, only to zero, as free_slot zeroes a released row.
            Result<std::span<std::byte>> bytes = arenas_.bytes(id);
            if (!bytes) continue;
            const std::size_t begin = std::size_t{body_slot} * array.elem_size;
            if (begin + array.elem_size > bytes->size()) continue;
            std::memset(bytes->data() + begin, 0, array.elem_size);
            clear_children(i, body_slot);
            continue;
        }
        if (array.extent != modules::Extent::per_element && array.extent != modules::Extent::per_sensor) continue;

        const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(id);
        const Result<std::span<std::byte>> bytes = arenas_.bytes(id);
        const Result<WorldRange> range = arenas_.range(id, world_index);
        if (!map || !bytes || !range) continue;

        // Ascending slot order, so the free sequence -- and therefore every
        // future allocation out of the rebuilt free list -- is a function of
        // the slot indices alone. Liveness comes from the slot->world map and
        // NOT from the row's contents: a freed row is zero-filled, and
        // body_slot 0 is a perfectly legitimate world-local index, so trusting
        // the row would free live rows attached to body 0.
        //
        // `map` and `bytes` stay valid across the free_slot() calls below: they
        // span storage that is sized once at registration and never moves, so
        // only the VALUES change (to kInvalidWorld and zeroes, for slots this
        // loop has already passed).
        for (uint32_t slot = range->begin; slot < range->begin + range->count; ++slot) {
            if ((*map)[slot] != world_index) continue;
            // An attached row's body_slot is its first four bytes
            // (modules::attached_row_size), copied out rather than read through
            // a cast: the loop knows the row only as bytes.
            uint32_t owner = 0;
            std::memcpy(&owner, bytes->data() + std::size_t{slot} * array.elem_size, sizeof(owner));
            if (owner != local_body) continue;
            if (Result<void> freed = arenas_.free_slot(id, slot); freed) {
                // free_slot zeroes the ROW; a ring's samples live in a second,
                // direct-indexed array that nothing else would clear. See
                // Simulation::despawn's contract.
                clear_children(i, slot);
            }
        }
    }
}

void Simulation::clear_children(uint32_t array, uint32_t slot) {
    for (uint32_t j = 0; j < schedule_.arrays.size(); ++j) {
        const modules::CompiledArray& child = schedule_.arrays[j];
        if (child.extent != modules::Extent::per_row || child.owner != array) continue;
        Result<std::span<std::byte>> bytes = arenas_.bytes(module_array_ids_[j]);
        if (!bytes) continue;
        const std::size_t window = std::size_t{child.depth} * child.elem_size;
        const std::size_t begin = std::size_t{slot} * window;
        if (begin + window > bytes->size()) continue;
        // EVERY BYTE, BY memset. A freed slot reads as zeroes, padding
        // included, because snapshot digests fold raw bytes; a value-initialized
        // assignment (`row = Row{}`) would zero every MEMBER and leave padding
        // unspecified. For the built-in rings this writes exactly the bytes the
        // field-wise clears it replaced wrote: ImuSample and GnssFix have no
        // implicit padding (sensors/imu.hpp and sensors/gnss.hpp assert it), so
        // naming all six fields of each zeroed every byte.
        std::memset(bytes->data() + begin, 0, window);
    }
}

void Simulation::publish_body_counts() {
    Result<std::span<WorldParams>> params = arenas_.array(world_params_id_);
    if (!params) return;
    for (uint32_t w = 0; w < layout_.world_count; ++w) {
        // ArenaSet::live_count() is the authority; WorldParams::body_count is
        // the mirror the param buffer publishes (layout.hpp says exactly that).
        // Republished for every world rather than only for the ones the queue
        // touched: it is O(worlds), and "the mirror is correct after every
        // flush" is a much easier invariant to keep than "the mirror is correct
        // for the worlds we remembered to update".
        const Result<uint32_t> live = arenas_.live_count(bodies_id_, w);
        if (live) (*params)[w].body_count = *live;
    }
}

// ---------------------------------------------------------------------------
// structural API
// ---------------------------------------------------------------------------

Result<uint32_t> Simulation::checked_world(uint32_t world_index) const {
    if (world_index >= layout_.world_count) {
        return std::unexpected(invalid("world index " + std::to_string(world_index) +
                                       " is out of range (world_count = " +
                                       std::to_string(layout_.world_count) + ")"));
    }
    return world_index;
}

Result<void> Simulation::validate_ref(BodyRef ref) const {
    if (ref.is_null()) {
        return std::unexpected(missing("body ref is null"));
    }
    if (ref.world_index >= layout_.world_count) {
        return std::unexpected(missing("body ref names a world outside this set"));
    }
    // The redundancy check: `world_index` must agree with the partition `slot`
    // falls in. Catches a hand-built ref, and a ref from a differently-shaped
    // world set, before it indexes anything.
    if (layout_.body_capacity == 0 || ref.slot / layout_.body_capacity != ref.world_index) {
        return std::unexpected(missing("body ref's slot does not lie in its world's partition"));
    }

    const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(bodies_id_);
    if (!map) return std::unexpected(map.error());
    if (ref.slot >= map->size() || (*map)[ref.slot] != ref.world_index) {
        return std::unexpected(missing("body ref names a slot that is not allocated"));
    }

    const Result<std::span<const uint32_t>> generations = arenas_.array(body_gen_id_);
    if (!generations) return std::unexpected(generations.error());
    if ((*generations)[ref.slot] != ref.generation) {
        return std::unexpected(missing("body ref is stale: its slot has been recycled"));
    }
    // Parity: an EVEN generation means the slot is allocated but its release is
    // already queued. spawn() only ever hands out odd generations, so a ref that
    // gets this far with an even one came from body_ref_at() reading a slot
    // between despawn() and the step boundary. Rejecting it here is what makes
    // despawn()'s "the ref is dead the moment this returns" true for EVERY way
    // of obtaining a ref, not just for the one the caller already held.
    if ((ref.generation & 1u) == 0u) {
        return std::unexpected(missing("body ref names a body whose despawn is already queued"));
    }
    return {};
}

Result<BodyRef> Simulation::spawn(uint32_t world_index, const BodySpawn& body) {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    const WorldConfig& config = configs_[world_index];

    if (!finite(body.pos) || !finite(body.vel) || !finite(body.omega_body) ||
        !finite(body.inv_inertia_diag)) {
        return std::unexpected(invalid("spawn: pos/vel/omega/inv_inertia must all be finite"));
    }
    if (!(body.mass > 0.0f) || !finite(body.mass)) {
        // mass is stored directly, not as an inverse, so 0 means "degenerate",
        // never "infinite" -- integrate_bodies divides by it.
        return std::unexpected(invalid("spawn: mass must be finite and > 0"));
    }
    if (!(body.inv_inertia_diag.x >= 0.0f) || !(body.inv_inertia_diag.y >= 0.0f) ||
        !(body.inv_inertia_diag.z >= 0.0f)) {
        return std::unexpected(invalid("spawn: inv_inertia_diag must be componentwise >= 0"));
    }
    if (!finite(body.orient)) {
        return std::unexpected(invalid("spawn: orientation must be finite"));
    }
    if (!(glm::dot(body.orient, body.orient) > 0.0f)) {
        return std::unexpected(invalid("spawn: orientation must have non-zero length"));
    }

    // NOT BURIED IN THE WORLD. The Baumgarte correction is a fraction of the
    // excess penetration per substep and is UNCAPPED, so a body spawned deep
    // inside geometry is ejected proportionally to its depth -- a teleport, and
    // possibly a tunnel through the far side. Spelled `!(phi >= -r)` so a NaN
    // field value rejects rather than comparing false on both sides.
    const float phi = eval(config.sdf, body.pos);
    if (!(phi >= -config.contacts.proxy_radius)) {
        return std::unexpected(invalid("spawn: position is deeper than one proxy radius inside the "
                                       "world SDF (uncapped positional correction would eject it)"));
    }

    // The world's OWN declared capacity, not the arena partition's -- see
    // WorldSetLayout. live_count() already includes slots reserved by earlier
    // spawns in this same window, which is what makes the check conservative and
    // correct without consulting the queue.
    const Result<uint32_t> live = arenas_.live_count(bodies_id_, world_index);
    if (!live) return std::unexpected(live.error());
    if (*live >= config.declared_body_capacity) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "spawn: world " + std::to_string(world_index) +
                                         " is at its declared body capacity (" +
                                         std::to_string(config.declared_body_capacity) + ")"});
    }

    const Result<uint32_t> slot = arenas_.alloc_slot(bodies_id_, world_index);
    if (!slot) return std::unexpected(slot.error());

    // Takes the counter from even (free / never issued) to ODD (live) -- see
    // the parity note on BodyRef. The wrap guard preserves that: a free slot's
    // counter is even, so `++` can only reach 0 from 0xFFFFFFFF, which is odd
    // and therefore not a free slot; if it somehow did, 1 is odd and live.
    Result<std::span<uint32_t>> generations = arenas_.array(body_gen_id_);
    if (!generations) return std::unexpected(generations.error());
    uint32_t& generation = (*generations)[*slot];
    ++generation;
    if (generation == 0) generation = 1;  // 0 means "never issued" (core/ids.hpp)

    StructuralOp op;
    op.kind = OpKind::init_body;
    op.world_index = world_index;
    op.slot = *slot;
    op.body = body;
    queue_.push_back(op);

    return BodyRef{world_index, *slot, generation};
}

Result<void> Simulation::despawn(BodyRef ref) {
    if (Result<void> valid = validate_ref(ref); !valid) {
        return valid;
    }

    // Bumped NOW, not at the boundary: `ref` must be dead the moment this
    // returns, so a second despawn is rejected instead of double-freeing a slot
    // that a spawn in the same window may already have re-reserved.
    //
    // The bump takes the counter from odd to EVEN, which is what marks the slot
    // "allocated but pending release" -- see the parity note on BodyRef. No
    // wrap guard here, deliberately: `++` reaching 0 lands on an even value,
    // which is exactly the dead state, so the parity invariant survives the
    // (unreachable) overflow on its own. A guard forcing 1 would mark a dead
    // slot LIVE.
    Result<std::span<uint32_t>> generations = arenas_.array(body_gen_id_);
    if (!generations) return std::unexpected(generations.error());
    ++(*generations)[ref.slot];

    StructuralOp op;
    op.kind = OpKind::free_body;
    op.world_index = ref.world_index;
    op.slot = ref.slot;
    queue_.push_back(op);
    return {};
}

// ---------------------------------------------------------------------------
// attached rows: the one door, and its typed fronts
// ---------------------------------------------------------------------------

Result<RowRef> Simulation::attach_row(BodyRef body, std::string_view array, std::span<const std::byte> spawn) {
    for (uint32_t i = 0; i < schedule_.arrays.size(); ++i) {
        if (schedule_.arrays[i].name == array) return attach_row_impl(body, i, spawn, "attach_row");
    }
    return std::unexpected(
        missing("attach_row: no module in this set declares an array '" + std::string(array) + "'"));
}

Result<RowRef> Simulation::attach_row_impl(BodyRef body, uint32_t array, std::span<const std::byte> spawn,
                                           std::string_view what) {
    if (Result<void> valid = validate_ref(body); !valid) {
        return std::unexpected(valid.error());
    }
    const std::string prefix = std::string(what) + ": ";
    if (array >= schedule_.arrays.size()) {
        // The typed fronts pass the arrays create() minted, which always exist.
        return std::unexpected(internal(prefix + "the array is not in this set's table"));
    }
    const modules::CompiledArray& decl = schedule_.arrays[array];
    const bool slot_allocated =
        decl.extent == modules::Extent::per_element || decl.extent == modules::Extent::per_sensor;
    if (!slot_allocated && decl.extent != modules::Extent::per_body) {
        return std::unexpected(invalid(prefix + "array '" + decl.name + "' (module '" + decl.module +
                                       "') is not attached to a body; only a per_body, per_element or "
                                       "per_sensor array is"));
    }
    if (spawn.size() != decl.spawn_size) {
        return std::unexpected(invalid(prefix + "array '" + decl.name + "' takes a " +
                                       std::to_string(decl.spawn_size) + "-byte spawn record, not " +
                                       std::to_string(spawn.size())));
    }
    if (decl.validate != nullptr) {
        if (Result<void> checked = decl.validate(spawn); !checked) {
            return std::unexpected(Error{checked.error().code, prefix + checked.error().context});
        }
    }

    const WorldConfig& config = configs_[body.world_index];
    const ArrayIndex id = module_array_ids_[array];
    uint32_t slot = body.slot;  // per_body: the body's own row
    if (slot_allocated) {
        if (decl.extent == modules::Extent::per_element) {
            // THE SHARED FORCE-ELEMENT BUDGET (Task 18): rotors and drag
            // bodies -- and every other per_element array -- live in separate
            // arena arrays but count against ONE declared capacity, because
            // spec §3 calls them all force elements and a world file declares
            // how many force elements a world holds, not how many of each kind.
            // A world with rotors in it therefore has fewer drag slots
            // available than it has drag STORAGE, which is the honest reading
            // of its own declaration.
            const Result<uint32_t> live = live_force_elements(body.world_index);
            if (!live) return std::unexpected(live.error());
            if (*live >= config.declared_element_capacity) {
                return std::unexpected(Error{Code::capacity_exceeded,
                                             prefix + "world " + std::to_string(body.world_index) +
                                                 " is at its declared force-element capacity (" +
                                                 std::to_string(config.declared_element_capacity) +
                                                 ", shared between drag bodies and rotors)"});
            }
        } else {
            // A per_sensor array counts ALONE: the declared sensor capacity
            // bounds each sensor array's live rows, never their sum, so a world
            // may hold `sensors` IMUs AND `sensors` receivers.
            const Result<uint32_t> live = arenas_.live_count(id, body.world_index);
            if (!live) return std::unexpected(live.error());
            if (*live >= config.declared_sensor_capacity) {
                return std::unexpected(Error{Code::capacity_exceeded,
                                             prefix + "world " + std::to_string(body.world_index) +
                                                 " is at its declared sensor capacity (" +
                                                 std::to_string(config.declared_sensor_capacity) + ")"});
            }
        }

        const Result<uint32_t> reserved = arenas_.alloc_slot(id, body.world_index);
        if (!reserved) return std::unexpected(reserved.error());
        slot = *reserved;

        // -------------------------------------------------------------------
        // THE ROW'S IDENTITY IS WRITTEN AT RESERVATION TIME; ONLY ITS
        // PARAMETERS AND ITS LIVENESS FLAG WAIT FOR THE BOUNDARY. This is not
        // an optimization, it closes an aliasing hole:
        //
        // a reserved row is otherwise the arena's zeroes, i.e. `body_slot == 0`
        // -- and 0 is a perfectly legitimate world-local body index. The
        // despawn cascade (free_rows_of) identifies a body's rows by exactly
        // that field, so a row reserved while a despawn of world-local body 0
        // was already queued would be mistaken for one of body 0's rows and
        // freed, after which the queued init_row would write a live row into a
        // slot the arena considers free -- breaking the "a freed slot reads as
        // zeroes" invariant, under-counting live_count, letting a later
        // reservation alias the same row, and leaving an orphan element that
        // applies drag (say) to whatever body next occupies that body slot.
        // Every one of those is silent.
        //
        // Writing `body_slot` here makes the reserved row TRUTHFUL about which
        // body it belongs to from the instant it exists, so the cascade's test
        // is exact. The liveness flag stays 0 (the arena's zero-fill), so the
        // row is still inert to its pass until the boundary -- the same
        // "reserved but not yet live" posture spawn() gives a body slot via
        // body_flags::active.
        // -------------------------------------------------------------------
        Result<std::span<std::byte>> bytes = arenas_.bytes(id);
        if (!bytes) return std::unexpected(bytes.error());
        const uint32_t local_body = body.slot - body.world_index * layout_.body_capacity;
        std::memcpy(bytes->data() + std::size_t{slot} * decl.elem_size, &local_body, sizeof(local_body));
    }

    queue_init_row(array, body.world_index, slot, body.slot, spawn);
    return RowRef{body.world_index, slot};
}

void Simulation::queue_init_row(uint32_t array, uint32_t world_index, uint32_t slot, uint32_t body_slot,
                                std::span<const std::byte> spawn) {
    StructuralOp op;
    op.kind = OpKind::init_row;
    op.world_index = world_index;
    op.slot = slot;
    op.body_slot = body_slot;
    op.array = array;
    // compile_schedule bounded every declared spawn_size by kMaxSpawnBytes,
    // and attach_row_impl and spawn(vehicle) hand over exactly that many.
    op.spawn_size = static_cast<uint32_t>(std::min<std::size_t>(spawn.size(), op.spawn.size()));
    std::memcpy(op.spawn.data(), spawn.data(), op.spawn_size);
    queue_.push_back(op);
}

// THE RULES A VEHICLE HOOK'S REQUEST KEEPS (sim/module.hpp's VEHICLE-SPAWN
// HOOK). Each is what makes the queued init safe without a reservation of its
// own: the row is one this vehicle owns outright (its body's, or one in a
// rotor window it just reserved), so no other body's row can be written, and
// the despawn cascade frees it with that body or rotor.
Result<uint32_t> Simulation::check_row_request(std::string_view module, const modules::VehicleRows& vehicle,
                                               const modules::RowInitRequest& request) const {
    const std::string who = "spawn(vehicle): module '" + std::string(module) + "' asks for a row of '" +
                            std::string(request.array) + "'";
    const auto it = std::ranges::find(schedule_.arrays, request.array, &modules::CompiledArray::name);
    if (it == schedule_.arrays.end()) {
        return std::unexpected(invalid(who + ", which no module in this set declares"));
    }
    const modules::CompiledArray& array = *it;
    if (array.module != module) {
        return std::unexpected(invalid(who + ", an array of module '" + array.module +
                                       "'; a vehicle hook initializes only its own module's rows"));
    }
    if (array.init == nullptr) {
        return std::unexpected(invalid(who + ", which declares no init to run"));
    }
    if (request.spawn_size != array.spawn_size) {
        return std::unexpected(invalid(who + " with a " + std::to_string(request.spawn_size) +
                                       "-byte record; the array takes " + std::to_string(array.spawn_size)));
    }
    if (array.extent == modules::Extent::per_body) {
        if (request.slot != vehicle.body_slot) {
            return std::unexpected(invalid(who + " at slot " + std::to_string(request.slot) +
                                           "; the vehicle's per_body row is its body's, slot " +
                                           std::to_string(vehicle.body_slot)));
        }
    } else if (array.extent == modules::Extent::per_row) {
        if (array.owner != table_index(rotors_id_)) {
            return std::unexpected(invalid(who + ", whose rows are owned by '" + schedule_.arrays[array.owner].name +
                                           "'; a vehicle's per_row rows are its rotors'"));
        }
        // Row g of the owner owns rows [g * depth, (g + 1) * depth).
        if (std::ranges::find(vehicle.rotor_slots, request.slot / array.depth) == vehicle.rotor_slots.end()) {
            return std::unexpected(invalid(who + " at slot " + std::to_string(request.slot) +
                                           ", which lies in none of the vehicle's rotor windows"));
        }
    } else {
        return std::unexpected(invalid(who + ", which is neither per_body nor owned by rotors; a vehicle hook "
                                             "initializes those, and attach_row() reserves the rest"));
    }
    return static_cast<uint32_t>(it - schedule_.arrays.begin());
}

uint32_t Simulation::table_index(ArrayIndex array) const noexcept {
    for (uint32_t i = 0; i < module_array_ids_.size(); ++i) {
        if (module_array_ids_[i] == array) return i;
    }
    return modules::kNoArray;
}

Result<DragElementRef> Simulation::add_drag_element(BodyRef ref, const DragElementSpawn& elem) {
    const Result<RowRef> row = attach_row_impl(ref, table_index(drag_id_), std::as_bytes(std::span(&elem, 1)),
                                               "add_drag_element");
    if (!row) return std::unexpected(row.error());
    return DragElementRef{row->world_index, row->slot};
}

Result<GnssSensorRef> Simulation::add_gnss_sensor(BodyRef ref, const GnssSensorSpawn& sensor) {
    const Result<RowRef> row = attach_row_impl(ref, table_index(gnss_id_), std::as_bytes(std::span(&sensor, 1)),
                                               "add_gnss_sensor");
    if (!row) return std::unexpected(row.error());
    return GnssSensorRef{row->world_index, row->slot};
}

Result<ImuSensorRef> Simulation::add_imu_sensor(BodyRef ref, const ImuSensorSpawn& sensor) {
    const Result<RowRef> row = attach_row_impl(ref, table_index(imu_id_), std::as_bytes(std::span(&sensor, 1)),
                                               "add_imu_sensor");
    if (!row) return std::unexpected(row.error());
    return ImuSensorRef{row->world_index, row->slot};
}

Result<uint32_t> Simulation::live_force_elements(uint32_t world_index) const {
    uint32_t live = 0;
    for (uint32_t i = 0; i < schedule_.arrays.size(); ++i) {
        if (schedule_.arrays[i].extent != modules::Extent::per_element) continue;
        const Result<uint32_t> rows = arenas_.live_count(module_array_ids_[i], world_index);
        if (!rows) return std::unexpected(rows.error());
        live += *rows;
    }
    return live;
}

// ---------------------------------------------------------------------------
// model types
// ---------------------------------------------------------------------------

Result<ModelTypeId> Simulation::register_model(vehicles::ModelType model) {
    // Validated HERE, once, at registration -- so a bad model is an error the
    // moment it is offered rather than at the first spawn, or (worse) as a
    // plausible-looking wrong trajectory. ModelType::validate()'s message is
    // returned verbatim.
    if (Result<void> valid = model.validate(); !valid) {
        return std::unexpected(valid.error());
    }
    // q and -q are one rotation; stored normalized with a fixed sign, two models
    // that differ only there spawn identical rows. Not on the step path.
    model.design_to_principal = vehicles::canonical_design_rotation(model.design_to_principal);
    if (models_.size() >= std::numeric_limits<uint32_t>::max() - 1u) {
        return std::unexpected(Error{Code::capacity_exceeded, "register_model: model id space is full"});
    }
    models_.push_back(std::move(model));
    // Every configuration table is a function of the whole registry, in
    // registration order, so all of them are rebuilt -- here and only here,
    // never in a step (sim/module.hpp's ConfigTableDecl).
    rebuild_config_tables();
    // ONE-BASED: id 0 is the null id (see ModelTypeId), so the first model
    // registered is 1.
    return ModelTypeId{static_cast<uint32_t>(models_.size())};
}

Result<const vehicles::ModelType*> Simulation::model(ModelTypeId id) const {
    if (id.is_null() || id.value > models_.size()) {
        return std::unexpected(missing("model id " + std::to_string(id.value) +
                                       " is not registered with this simulation"));
    }
    return &models_[id.value - 1u];
}

// ---------------------------------------------------------------------------
// vehicle spawn
// ---------------------------------------------------------------------------
//
// THE UNWIND. Everything below reserves slots across FOUR arenas before a
// single queue op is pushed, so the failure story has to be stated: the
// capacity checks all happen before the first reservation, which makes a
// mid-reservation failure unreachable, and if one happened anyway every slot
// already taken is released here. ONE UNWIND IS REACHABLE (module-API stage 4,
// Task 7): a module's vehicle hook asks for its rows only once the slots are
// reserved, and a request that breaks a rule is refused here, before the
// generation bump. A reserve/release pair leaves the arena's
// free SET exactly as it was (state/arenas.hpp: the bump-cursor/free-list pair
// is canonical-equivalent, and alloc_slot only ever consults the free set), so
// an unwound spawn is invisible to every future allocation -- which is what
// keeps a failed spawn from perturbing a determinism replay.
// ModuleVehicleRows.ARequestOutsideTheVehicleIsRefusedAndUnwinds pins it.
// ---------------------------------------------------------------------------

Result<VehicleRef> Simulation::spawn(uint32_t world_index, ModelTypeId model_id,
                                     const VehicleSpawn& where) {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    const Result<const vehicles::ModelType*> found = this->model(model_id);
    if (!found) return std::unexpected(found.error());
    const vehicles::ModelType& model = **found;
    const WorldConfig& config = configs_[world_index];

    // --- validation, before anything is reserved ----------------------------
    if (!finite(where.pos) || !finite(where.vel) || !finite(where.omega_body)) {
        return std::unexpected(invalid("spawn(vehicle): pos/vel/omega must all be finite"));
    }
    if (!finite(where.orient)) {
        return std::unexpected(invalid("spawn(vehicle): orientation must be finite"));
    }
    if (!(glm::dot(where.orient, where.orient) > 0.0f)) {
        return std::unexpected(invalid("spawn(vehicle): orientation must have non-zero length"));
    }
    if (!(where.rotor_omega >= 0.0f) || !finite(where.rotor_omega)) {
        // Both rotor curves are EVEN in omega, so a negative shaft speed would
        // produce the thrust and torque of its magnitude while the lag drove
        // it further negative -- not a configuration this model has meaning
        // for (vehicles/rotor.hpp's preconditions). `spin_dir`, not the sign
        // of omega, is what says which way a rotor turns.
        return std::unexpected(invalid("spawn(vehicle): rotor_omega must be finite and >= 0"));
    }
    // D-S6-2 fix-loop (I2+M3, coordinator review): both preconditions below
    // test the VEHICLE's own EFFECTIVE radius -- the model's own proxy_radius
    // when it overrides the default, else the world's contacts.proxy_radius
    // -- rather than unconditionally the world default. A model whose own
    // radius disagrees with the world's (either direction) would otherwise be
    // validated against the wrong number, the same bug the collision passes
    // themselves had before this task.
    const float effective_radius =
        physics::effective_proxy_radius(model.proxy_radius, config.contacts.proxy_radius);

    // MIRRORS sim/world_set.cpp's validate_grid_against_contacts() -- same
    // invariant (physics/grid.hpp's cell_size >= 2*radius footgun: the
    // resolve step only gathers the 27 cells around a body, so a contact
    // diameter larger than one cell silently loses pairs that straddle a
    // gap), same failure mode, but checked against THIS MODEL's own declared
    // radius rather than the world's default -- exactly the gap the
    // world-level check cannot see, since it runs at world-set creation,
    // before any vehicle model even exists to register. Only checked when the
    // model overrides the default (proxy_radius != 0); the sentinel case
    // (0, i.e. "use the world's default") is already covered by the
    // world-level check that ran when this world was built.
    if (model.proxy_radius != 0.0f && config.grid.cell_size < 2.0f * model.proxy_radius) {
        return std::unexpected(invalid("spawn(vehicle): grid.cell_size must be >= 2 * the model's "
                                       "proxy_radius (smaller silently misses contacts across cell "
                                       "boundaries)"));
    }

    // NOT BURIED IN THE WORLD -- the same check, for the same uncapped-
    // Baumgarte reason, that spawn(world, BodySpawn) applies. Spelled
    // `!(phi >= -r)` so a NaN field value rejects. `r` is `effective_radius`
    // above, not unconditionally the world default -- see the D-S6-2 note.
    const float phi = eval(config.sdf, where.pos);
    if (!(phi >= -effective_radius)) {
        return std::unexpected(invalid("spawn(vehicle): position is deeper than one proxy radius "
                                       "inside the world SDF (uncapped positional correction would "
                                       "eject it)"));
    }

    // --- capacity, also before anything is reserved -------------------------
    const Result<uint32_t> live_bodies = arenas_.live_count(bodies_id_, world_index);
    if (!live_bodies) return std::unexpected(live_bodies.error());
    if (*live_bodies >= config.declared_body_capacity) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "spawn(vehicle): world " + std::to_string(world_index) +
                                         " is at its declared body capacity (" +
                                         std::to_string(config.declared_body_capacity) + ")"});
    }
    const Result<uint32_t> live_elements = live_force_elements(world_index);
    if (!live_elements) return std::unexpected(live_elements.error());
    if (*live_elements + model.force_element_count() > config.declared_element_capacity) {
        return std::unexpected(
            Error{Code::capacity_exceeded,
                  "spawn(vehicle): world " + std::to_string(world_index) + " cannot fit the model's " +
                      std::to_string(model.force_element_count()) +
                      " force elements in its declared capacity (" +
                      std::to_string(config.declared_element_capacity) +
                      ", shared between drag bodies and rotors)"});
    }
    const Result<uint32_t> live_sensors = arenas_.live_count(imu_id_, world_index);
    if (!live_sensors) return std::unexpected(live_sensors.error());
    if (*live_sensors + model.imu_mounts.size() > config.declared_sensor_capacity) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "spawn(vehicle): world " + std::to_string(world_index) +
                                         " cannot fit the model's " +
                                         std::to_string(model.imu_mounts.size()) +
                                         " sensors in its declared capacity (" +
                                         std::to_string(config.declared_sensor_capacity) + ")"});
    }

    // --- reservation --------------------------------------------------------
    const Result<uint32_t> body_slot = arenas_.alloc_slot(bodies_id_, world_index);
    if (!body_slot) return std::unexpected(body_slot.error());

    std::array<uint32_t, vehicles::kMaxModelRotors> rotor_slots{};
    std::array<uint32_t, vehicles::kMaxModelDragBodies> drag_slots{};
    std::array<uint32_t, vehicles::kMaxModelImuMounts> sensor_slots{};
    std::size_t rotors_taken = 0;
    std::size_t drags_taken = 0;
    std::size_t sensors_taken = 0;

    // Releases everything taken so far, in the reverse order it was taken, and
    // returns `error`. Reached by a refused vehicle-hook request; otherwise
    // unreachable given the checks above. See the unwind note.
    const auto unwind = [&](Error error) -> std::unexpected<Error> {
        for (std::size_t i = sensors_taken; i-- > 0;) (void)arenas_.free_slot(imu_id_, sensor_slots[i]);
        for (std::size_t i = drags_taken; i-- > 0;) (void)arenas_.free_slot(drag_id_, drag_slots[i]);
        for (std::size_t i = rotors_taken; i-- > 0;) (void)arenas_.free_slot(rotors_id_, rotor_slots[i]);
        (void)arenas_.free_slot(bodies_id_, *body_slot);
        return std::unexpected(std::move(error));
    };

    for (std::size_t i = 0; i < model.rotors.size(); ++i) {
        const Result<uint32_t> slot = arenas_.alloc_slot(rotors_id_, world_index);
        if (!slot) return unwind(slot.error());
        rotor_slots[i] = *slot;
        ++rotors_taken;
    }
    for (std::size_t i = 0; i < model.drag_bodies.size(); ++i) {
        const Result<uint32_t> slot = arenas_.alloc_slot(drag_id_, world_index);
        if (!slot) return unwind(slot.error());
        drag_slots[i] = *slot;
        ++drags_taken;
    }
    for (std::size_t i = 0; i < model.imu_mounts.size(); ++i) {
        const Result<uint32_t> slot = arenas_.alloc_slot(imu_id_, world_index);
        if (!slot) return unwind(slot.error());
        sensor_slots[i] = *slot;
        ++sensors_taken;
    }

    // --- the modules' vehicle rows (module-API stage 4, Task 7) -------------
    //
    // Each module's hook, in set order, asks for rows of its own module. Every
    // request is checked HERE, before the generation bump, so one that breaks
    // a rule unwinds exactly like a failed reservation: nothing is committed
    // and every slot taken above is released. Each hook fills a vector of its
    // own, so it sees -- and can disturb -- only its own requests. A set with
    // no hook allocates nothing here.
    const modules::VehicleRows vehicle{.model = &model,
                                       .models = std::span<const vehicles::ModelType>(models_),
                                       .model_index = model_id.value - 1u,
                                       .world_index = world_index,
                                       .body_slot = *body_slot,
                                       .rotor_slots = std::span<const uint32_t>(rotor_slots.data(), rotors_taken)};
    std::vector<modules::RowInitRequest> requests;
    std::vector<uint32_t> request_arrays;  // parallel to `requests`: each one's index in schedule_.arrays
    std::vector<modules::RowInitRequest> asked;
    for (const modules::CompiledVehicleRows& hook : schedule_.vehicle_rows) {
        asked.clear();
        hook.rows(vehicle, asked);
        for (const modules::RowInitRequest& request : asked) {
            const Result<uint32_t> array = check_row_request(hook.module, vehicle, request);
            if (!array) return unwind(array.error());
            requests.push_back(request);
            request_arrays.push_back(*array);
        }
    }

    // Every reservation succeeded; from here nothing can fail, so the
    // generation bump and the queue pushes are safe to commit.
    Result<std::span<uint32_t>> generations = arenas_.array(body_gen_id_);
    if (!generations) return unwind(generations.error());
    uint32_t& generation = (*generations)[*body_slot];
    // Even (free) -> ODD (live), with the same unreachable-wrap guard spawn()
    // uses; see the parity note on BodyRef.
    ++generation;
    if (generation == 0) generation = 1;

    // -----------------------------------------------------------------------
    // THE ROWS' IDENTITY IS WRITTEN AT RESERVATION TIME -- the Task 13 lesson,
    // spelled out at length in attach_row_impl(). A reserved row is otherwise
    // the arena's zeroes, i.e. `body_slot == 0`, and 0 is a perfectly
    // legitimate world-local body index; the despawn cascade identifies a
    // body's rows by exactly that field, so a row reserved while a despawn of
    // world-local body 0 was already queued would be mistaken for one of body
    // 0's and freed out from under its own pending init. `enabled`/`kind` stay
    // 0, so every row is still inert until the boundary.
    // -----------------------------------------------------------------------
    const uint32_t local_body = *body_slot - world_index * layout_.body_capacity;
    Result<std::span<vehicles::RotorRow>> rotor_rows = arenas_.array(rotors_id_);
    if (!rotor_rows) return unwind(rotor_rows.error());
    Result<std::span<physics::DragBodyRow>> drag_rows = arenas_.array(drag_id_);
    if (!drag_rows) return unwind(drag_rows.error());
    Result<std::span<sensors::ImuSensorRow>> sensor_rows = arenas_.array(imu_id_);
    if (!sensor_rows) return unwind(sensor_rows.error());
    for (std::size_t i = 0; i < rotors_taken; ++i) (*rotor_rows)[rotor_slots[i]].body_slot = local_body;
    for (std::size_t i = 0; i < drags_taken; ++i) (*drag_rows)[drag_slots[i]].body_slot = local_body;
    for (std::size_t i = 0; i < sensors_taken; ++i) (*sensor_rows)[sensor_slots[i]].body_slot = local_body;

    // --- queue --------------------------------------------------------------
    //
    // BODY FIRST, THEN ELEMENTS, THEN SENSORS, in declaration order within
    // each kind. The order does not matter to correctness (each op writes its
    // own reserved slot), and it is fixed anyway because the queue IS the
    // determinism contract: a flush applies ops front to back, so the sequence
    // a spawn contributes must be a function of the model alone.
    // THE DESIGN FRAME (DBP-46): `where` is the design origin's state; the body
    // row holds the centre of mass in principal axes. A model whose frames
    // coincide passes `where` through untouched (sim/design_frame.hpp).
    const FrameState start = to_body_state(FrameState{where.pos, where.orient, where.vel, where.omega_body},
                                           model.design_to_principal, model.com_offset);
    StructuralOp body_op;
    body_op.kind = OpKind::init_body;
    body_op.world_index = world_index;
    body_op.slot = *body_slot;
    body_op.body.pos = start.pos;
    // D-S6-2: the model OWNS its bodies' contact-proxy radius. Left at the
    // model's own 0.0f default, this writes 0 -- the sentinel that defers to
    // the world's ContactParams::proxy_radius, i.e. exactly what a model that
    // never customized its proxy produced before this field was consumed.
    body_op.body_proxy_radius = model.proxy_radius;
    body_op.body.orient = start.orient;
    body_op.body.vel = start.vel;
    body_op.body.omega_body = start.omega;
    body_op.body.mass = model.body.mass;
    // The model carries the INERTIA; BodyState stores its inverse. Inverted
    // here, in one place. Validated componentwise > 0 by ModelType::validate(),
    // so no division guard is needed and none is written -- a guard here would
    // silently admit the model the validator exists to refuse.
    body_op.body.inv_inertia_diag = glm::vec3(1.0f / model.body.inertia_diag.x,
                                              1.0f / model.body.inertia_diag.y,
                                              1.0f / model.body.inertia_diag.z);
    queue_.push_back(body_op);

    // Each row is an init_row of its module's array, with that module's spawn
    // record -- the records attach_row() takes -- and no validate: the model
    // was validated once, by register_model(), and the rows were reserved
    // above. The body_slot of each is already written.
    const uint32_t rotor_table = table_index(rotors_id_);
    const uint32_t drag_table = table_index(drag_id_);
    const uint32_t imu_table = table_index(imu_id_);
    for (std::size_t i = 0; i < rotors_taken; ++i) {
        const modules::builtin::RotorSpawn spawn{model.rotors[i], where.rotor_omega};
        queue_init_row(rotor_table, world_index, rotor_slots[i], *body_slot, std::as_bytes(std::span(&spawn, 1)));
    }
    for (std::size_t i = 0; i < drags_taken; ++i) {
        // The desc -> spawn-record mapping vehicles/model_type.hpp promises
        // lives in exactly one place. This is it, for drag bodies.
        const vehicles::DragBodyDesc& desc = model.drag_bodies[i];
        DragElementSpawn spawn;
        spawn.mode = desc.mode;
        spawn.area = desc.area;
        spawn.coeffs = desc.coeffs;
        spawn.local_pos = desc.local_pos;
        spawn.local_orient = desc.local_orient;
        queue_init_row(drag_table, world_index, drag_slots[i], *body_slot, std::as_bytes(std::span(&spawn, 1)));
    }
    for (std::size_t i = 0; i < sensors_taken; ++i) {
        // ... and this is it for IMU mounts.
        const vehicles::ImuMountDesc& desc = model.imu_mounts[i];
        ImuSensorSpawn spawn;
        spawn.mount_pos = desc.mount_pos;
        spawn.mount_orient = desc.mount_orient;
        spawn.rate_divider = desc.rate_divider;
        spawn.sigma_a = desc.sigma_a;
        spawn.sigma_g = desc.sigma_g;
        spawn.sigma_ba = desc.sigma_ba;
        spawn.sigma_bg = desc.sigma_bg;
        queue_init_row(imu_table, world_index, sensor_slots[i], *body_slot, std::as_bytes(std::span(&spawn, 1)));
    }
    // THEN THE MODULES' ROWS, checked above: hook order, then each hook's
    // request order, so this too is a function of the model and the set.
    for (std::size_t r = 0; r < requests.size(); ++r) {
        queue_init_row(request_arrays[r], world_index, requests[r].slot, *body_slot,
                       std::span<const std::byte>(requests[r].spawn.data(), requests[r].spawn_size));
    }

    VehicleRef ref;
    ref.body = BodyRef{world_index, *body_slot, generation};
    ref.model = model_id;
    ref.rotor_count = static_cast<uint32_t>(rotors_taken);
    ref.rotor_slots = rotor_slots;
    ref.imu_count = static_cast<uint32_t>(sensors_taken);
    for (std::size_t i = 0; i < sensors_taken; ++i) {
        ref.imu_sensors[i] = ImuSensorRef{world_index, sensor_slots[i]};
    }
    return ref;
}

Result<void> Simulation::set_rotor_commands(const VehicleRef& ref, std::span<const float> omega_cmd) {
    if (Result<void> valid = validate_ref(ref.body); !valid) {
        return valid;
    }
    if (omega_cmd.size() != ref.rotor_count) {
        return std::unexpected(invalid("set_rotor_commands: expected " +
                                       std::to_string(ref.rotor_count) + " commands, got " +
                                       std::to_string(omega_cmd.size())));
    }
    for (const float w : omega_cmd) {
        if (!(w >= 0.0f) || !finite(w)) {
            // Spelled `!(w >= 0)` so a NaN rejects. See spawn(vehicle) for why
            // a negative shaft speed is not a configuration this model has a
            // meaning for.
            return std::unexpected(invalid("set_rotor_commands: every command must be finite and >= 0"));
        }
    }

    Result<std::span<vehicles::RotorRow>> rows = arenas_.array(rotors_id_);
    if (!rows) return std::unexpected(rows.error());
    const uint32_t local_body = ref.body.slot - ref.body.world_index * layout_.body_capacity;

    // Two passes: check every row is writable, THEN write. A partially applied
    // command is a mixer output nobody asked for -- three rotors at the new
    // setpoint and one at the old is a vehicle that yaws for no reason -- so
    // this call is all-or-nothing like every other structural-adjacent API
    // here.
    for (uint32_t i = 0; i < ref.rotor_count; ++i) {
        const uint32_t slot = ref.rotor_slots[i];
        if (slot >= rows->size()) {
            return std::unexpected(internal("set_rotor_commands: rotor slot is outside the array"));
        }
        const vehicles::RotorRow& row = (*rows)[slot];
        if (row.enabled == 0u) {
            // The slot is reserved but its initialization is still queued, and
            // that initialization writes omega_cmd from the spawn -- so the
            // command would be silently discarded. Reported instead, exactly
            // as apply_wrench() reports a wrench on an unflushed body.
            return std::unexpected(missing("set_rotor_commands: the vehicle is spawned but not yet "
                                           "flushed (call step() or flush_structural() first)"));
        }
        if (row.body_slot != local_body) {
            // The ref names slots that now belong to someone else. Reachable
            // only by using a VehicleRef whose body was despawned and whose
            // slots were recycled -- which validate_ref() already rejects via
            // the generation -- so this is the belt to that braces.
            return std::unexpected(missing("set_rotor_commands: rotor slot no longer belongs to this "
                                           "vehicle (stale VehicleRef)"));
        }
    }
    for (uint32_t i = 0; i < ref.rotor_count; ++i) {
        (*rows)[ref.rotor_slots[i]].omega_cmd = omega_cmd[i];
    }

    // The device mirror is now stale -- the identical statement, for the
    // identical reason, as apply_wrench()'s (see the long note there). A rotor
    // command is a between-steps arena write that no structural queue knows
    // about. It was written by Task 6 in advance of being REACHABLE on the
    // vulkan path -- a live rotor was refused outright until Task 8 ported the
    // pass -- and Task 8 is what makes it live: tests/test_gpu_parity.cpp's
    // RotorCommandsForceReupload now issues a real command to a real rotor and
    // watches the upload counter, where through Task 7 it could only reach this
    // line with a rotorless model.
    mark_vulkan_dirty();
    return {};
}

Result<void> Simulation::apply_wrench(BodyRef ref, glm::vec3 world_force, glm::vec3 body_torque) {
    if (Result<void> valid = validate_ref(ref); !valid) {
        return valid;
    }
    if (!finite(world_force) || !finite(body_torque)) {
        return std::unexpected(invalid("apply_wrench: force and torque must be finite"));
    }

    Result<std::span<BodyState>> bodies = arenas_.array(bodies_id_);
    if (!bodies) return std::unexpected(bodies.error());
    BodyState& b = (*bodies)[ref.slot];
    if ((b.flags & physics::body_flags::active) == 0u) {
        // The slot is reserved but its initialization is still queued, and that
        // initialization zeroes the accumulators -- so the wrench would be
        // silently discarded. Reported instead. flush_structural() first.
        return std::unexpected(missing("apply_wrench: body is spawned but not yet flushed "
                                       "(call step() or flush_structural() first)"));
    }

    b.force_acc += world_force;
    b.torque_acc += body_torque;

    // ---------------------------------------------------------------------
    // S6 Task 6: THE DEVICE MIRROR IS NOW STALE, and saying so here closes a
    // real hole rather than being belt-and-braces.
    //
    // Before this line, `vulkan_dirty_` was set by exactly one thing:
    // flush_structural() applying a NON-EMPTY queue. apply_wrench() is not a
    // structural op -- it writes straight into the arena, between steps, with
    // no queue involved -- so a wrench applied on the vulkan path landed in
    // arenas_ and was then never uploaded, because the next step() found the
    // mirror "clean" and skipped the upload. The GPU kept its own force_acc
    // (which its own Integrate had just zeroed) and the input silently did
    // nothing.
    //
    // That is not hypothetical: the golden corpus's `ballistic` scenario
    // applies a wrench on EVERY ONE of its 200 ticks, so a parity run of it
    // would have compared a CPU trajectory driven by 200 inputs against a GPU
    // trajectory driven by none. Found while building this task's parity
    // harness; the fix is the same statement flush_structural() already makes
    // about its own mutation, at the other place state is mutated between
    // steps.
    // ---------------------------------------------------------------------
    mark_vulkan_dirty();
    return {};
}

// ---------------------------------------------------------------------------
// snapshot / restore
// ---------------------------------------------------------------------------

Result<SnapshotBlob> Simulation::snapshot() const {
    if (!queue_.empty()) {
        return std::unexpected(invalid("snapshot: the structural queue is not empty; pending "
                                       "spawn/despawn ops are not part of the registry walk. Call "
                                       "step() or flush_structural() first."));
    }
    return save(arenas_, tick_, schedule_.identity, vehicles::model_registry_identity(models_));
}

// ---------------------------------------------------------------------------
// The pre-apply configuration cross-check (ticket M-1).
//
// RUNS BEFORE ANY ARENA BYTE IS WRITTEN, which is what keeps restore()
// all-or-nothing: it reads the blob and this object's own registered row, and
// writes nothing. On rejection the tree is untouched, and a digest taken before
// the failed call equals one taken after it (test_determinism.cpp pins exactly
// that).
//
// IT ONLY EVER ADDS A REJECTION, AND NEVER MASKS ONE. Every way this function
// can fail to find something to compare -- a schema that already disagrees, no
// such section, a wrong element size, no rows -- is a blob spade::restore() is
// about to reject anyway on schema grounds (the schema hash covers array names,
// element sizes and extents), so all of those FALL THROUGH deliberately rather
// than inventing an error of their own. That keeps the state layer's own, more
// precise diagnostics reachable instead of burying them under a config message,
// and it means the ONLY blob this function can reject is one that the state
// layer would have accepted.
//
// THE COMPARISON SOURCE IS THE LIVE REGISTERED ROW, not dt_ns_/substeps_. The
// row is the only home config_hash has -- Simulation does not retain the desc
// -- so taking two of the three values from one place and the third from
// another would be exactly the second source of truth WorldConfig's "NO seed
// MEMBER" note forbids. That the row still agrees with dt_ns_/substeps_ is an
// induction, not a hope: create() writes both from the same two arguments, and
// the only other write to the row is the restore this function has just proven
// carries identical values. The internal check below is that induction's
// tripwire -- if it ever fires, something wrote arena bytes behind Simulation's
// back (the registry-level spade::restore() overload takes a CONST registry and
// can be called on arenas().registry() by anyone), and continuing would compare
// a corrupted authority against a blob.
//
// THE TRIPWIRE REACHES TWO OF THE THREE FIELDS, AND THAT IS ALL IT CAN REACH.
// dt_ns and substeps have a second, independent copy in this object to be
// checked against; config_hash does not -- the row IS its only home, by design
// (there is no desc to recompute it from). So a bypass that corrupted ONLY
// config_hash is undetectable here, and the check would then compare a blob
// against a corrupted authority and reject a sound pairing. That is the
// residual cost of having one source of truth, stated rather than papered over:
// the alternative -- caching the hash in a member -- would trade an
// undetectable corruption for a second value that can silently disagree with
// the registered one on every restore.
// ---------------------------------------------------------------------------
Result<void> Simulation::check_replay_config(const SnapshotBlob& blob) const {
    const Result<std::span<const ReplayConfig>> live = arenas_.array(replay_config_id_);
    if (!live) return std::unexpected(live.error());
    if (live->empty()) {
        return std::unexpected(internal("restore: this simulation has no replay_config row"));
    }
    const ReplayConfig& mine = (*live)[0];

    if (mine.dt_ns != dt_ns_ || mine.substeps != substeps_) {
        return std::unexpected(internal(
            "restore: the registered replay_config row (dt_ns " + std::to_string(mine.dt_ns) + ", substeps " +
            std::to_string(mine.substeps) + ") disagrees with this simulation's step decomposition (dt_ns " +
            std::to_string(dt_ns_) + ", substeps " + std::to_string(substeps_) +
            "); arena bytes were written behind Simulation's back"));
    }

    // A SHAPE MISMATCH IS NOT THIS CHECK'S TO REPORT. A blob from a set with a
    // different capacity has a different schema hash AND a different
    // config_hash (capacities are folded into both), and of the two verdicts
    // the state layer's is the more specific one -- "array 'bodies': shape 2x5
    // in the blob, 2x4 in the registry" tells a caller what to fix; "you paired
    // the wrong blob" does not. So whenever the schema already disagrees, this
    // function stands down and lets spade::restore() speak. What is left is
    // exactly the case this check exists for and the schema hash is blind to:
    // SAME SHAPE, DIFFERENT PHYSICS.
    if (blob.schema_hash() != schema_hash(arenas_.registry())) {
        return {};
    }

    const Result<BlobSection> section = find_section(blob, kReplayConfigArray);
    if (!section) {
        // not_found (no such section) or io_error (the blob does not parse).
        // Both are spade::restore()'s to report, in its own words.
        return {};
    }
    if (section->elem_size != sizeof(ReplayConfig) || section->payload.size() < sizeof(ReplayConfig)) {
        return {};  // a shape spade::restore() will reject; see above.
    }

    // ---------------------------------------------------------------------
    // THE ACCEPT TEST IS THE WHOLE PAYLOAD, BYTE FOR BYTE -- not the three
    // named fields, and not row 0 alone.
    //
    // The three fields are 24 of the 32 bytes of ONE row, and restore() is
    // about to overwrite ALL of them, in every world. Accepting on a
    // field-wise comparison would mean restoring the remainder on trust: the
    // two reserved lanes today, a fourth field the day `_reserved0` becomes
    // real, and every world's row above index 0. Comparing the bytes closes
    // all three at once and needs no maintenance when a field is added -- the
    // opposite of a comment promising that a future author will remember to
    // grow a loop here.
    //
    // The length equality is checked first rather than assumed from the schema
    // hash agreeing: a hash is never trusted for correctness in this tree
    // (state/snapshot.cpp's match_registry says so), and a length mismatch is a
    // shape fault that belongs to spade::restore().
    // ---------------------------------------------------------------------
    const std::span<const std::byte> mine_bytes = std::as_bytes(*live);
    if (section->payload.size() != mine_bytes.size()) {
        return {};  // a shape spade::restore() will reject; see above.
    }
    if (std::memcmp(section->payload.data(), mine_bytes.data(), mine_bytes.size()) == 0) {
        return {};
    }

    // Past here the payloads DIFFER and the blob is refused. Everything below
    // exists only to say WHY in the caller's terms, so it reads row 0's three
    // named fields -- the ones a caller can act on. memcpy, never a
    // reinterpret_cast: blob sections are tightly packed, so a payload can
    // start at any offset (state/snapshot.hpp).
    ReplayConfig theirs{};
    std::memcpy(&theirs, section->payload.data(), sizeof(ReplayConfig));

    std::string diverged;
    const auto note = [&diverged](const std::string& text) {
        if (!diverged.empty()) diverged += ", ";
        diverged += text;
    };
    if (theirs.dt_ns != mine.dt_ns) {
        note("dt_ns (blob " + std::to_string(theirs.dt_ns) + ", this simulation " + std::to_string(mine.dt_ns) + ")");
    }
    if (theirs.substeps != mine.substeps) {
        note("substeps (blob " + std::to_string(theirs.substeps) + ", this simulation " +
             std::to_string(mine.substeps) + ")");
    }
    if (theirs.config_hash != mine.config_hash) {
        note("config_hash (blob " + hex64(theirs.config_hash) + ", this simulation " + hex64(mine.config_hash) + ")");
    }
    // The bytes differ somewhere the three named fields do not reach: a
    // reserved lane, or a row above index 0. Refused all the same -- the whole
    // record is the identity -- but named honestly rather than reported as a
    // field divergence that a caller would go looking for and not find.
    if (diverged.empty()) {
        diverged = "the replay_config bytes differ outside (dt_ns, substeps, config_hash) -- a "
                   "reserved lane or a row above world 0";
    }

    // "COULD", not "would": for the one pairing whose only difference is the
    // per-world seeds, the replay would in fact be identical (seeds are
    // registered state and arrive with the blob) -- config_hash covers the
    // creating desc's seeds, so that pairing is refused fail-closed. The
    // diagnostic has to stay true of every case it prints on.
    return std::unexpected(invalid("restore: blob was produced under a different (dt_ns, substeps, "
                                   "config_hash) -- restoring it here could replay different physics. "
                                   "Diverged: " +
                                   diverged));
}

Result<void> Simulation::restore(const SnapshotBlob& blob) {
    if (blob.world_count() != layout_.world_count) {
        return std::unexpected(Error{Code::schema_mismatch,
                                     "restore: blob describes " + std::to_string(blob.world_count()) +
                                         " worlds, this simulation has " +
                                         std::to_string(layout_.world_count)});
    }

    // BEFORE the restore, never after: see check_replay_config() above. This is
    // the only path from a Simulation into spade::restore(), so there is no
    // second door a blob could come through unchecked.
    // L2: a blob taken under another module set, module version or schedule
    // would replay different physics here. The identity rides in the snapshot
    // header (module-API plan Ruling 1), not in the digested replay_config row.
    if (blob.configuration_identity() != schedule_.identity) {
        return std::unexpected(invalid("restore: blob was taken under a different module set or schedule "
                                       "(identity " + hex64(blob.configuration_identity()) +
                                       ", this simulation runs " + hex64(schedule_.identity) +
                                       "); restoring it here could replay different physics"));
    }
    // Snapshot format v3: the model registry, in registration order (L2). The
    // rows a vehicle spawned carry its model's values; the registry is what a
    // later spawn, vehicle_state() or a viewer consults, so a blob is only
    // meaningful beside the registry it was taken under.
    if (const uint64_t registry = vehicles::model_registry_identity(models_); blob.model_registry_identity() != registry) {
        return std::unexpected(invalid("restore: blob was taken under a different model registry (identity " +
                                       std::to_string(blob.model_registry_identity()) +
                                       ", this simulation's is " + std::to_string(registry) +
                                       "); register the same models, in the same order, before restoring"));
    }
    if (Result<void> config = check_replay_config(blob); !config) {
        return config;
    }

    // THE ArenaSet OVERLOAD, never the registry-level one: the latter leaves the
    // free lists describing the pre-restore population, which is a silent
    // divergence on the very next allocation (state/snapshot.hpp; test_snapshot
    // pins it). All-or-nothing, so nothing below runs on failure.
    if (Result<void> restored = spade::restore(arenas_, blob); !restored) {
        return restored;
    }

    // -----------------------------------------------------------------------
    // THE DEVICE MIRROR NOW DISAGREES WITH EVERY REGISTERED BYTE (S6 Task 6
    // review round 1, finding C1). This is the largest arena mutation in the
    // whole API -- spade::restore() above rewrote all nine arrays and both
    // their free lists -- and it is not a structural-queue op, so
    // flush_structural() never sees it. Without this call, a restore followed
    // by a step() on the vulkan path would submit the STALE PRE-RESTORE state
    // to the device and then have the readback overwrite the freshly restored
    // arenas with the result: the restore would be silently discarded, and the
    // run would continue from a state the caller had explicitly replaced.
    //
    // Marked AFTER the restore succeeds, not before, and the ordering is a
    // deliberate match to this function's own all-or-nothing contract: a
    // REJECTED blob leaves the arenas untouched (every check runs before the
    // first byte is written), so it leaves the mirror valid too and must not
    // force a pointless re-upload. flush_structural() marks BEFORE applying
    // for the opposite reason -- apply_op() can fail partway, and a
    // partially-applied queue IS a mutation.
    // -----------------------------------------------------------------------
    mark_vulkan_dirty();

    tick_ = blob.tick();

    // Spec §4: "Restore = reverse + structural-queue flush." The queue describes
    // changes to a state that has just been replaced -- including reservations
    // of slots the restored free lists now consider free -- so it is discarded,
    // not replayed.
    queue_.clear();

    // Transient by construction (rebuilt from body positions every substep), but
    // cleared so a restore cannot be observed through leftover scratch.
    scratch_.entries.clear();
    scratch_.runs.clear();

    return rebuild_views();
}

// ---------------------------------------------------------------------------
// reseed
// ---------------------------------------------------------------------------
//
// See the doc comment in simulation.hpp for the full argument. What it
// re-derives is no longer a list here: it is every stream the module set
// declares (sim/module.hpp's SEEDED STREAMS), walked by derive_streams().
// ---------------------------------------------------------------------------

Result<void> Simulation::reseed(uint64_t scene_seed) {
    if (!queue_.empty()) {
        return std::unexpected(invalid("reseed: the structural queue is not empty; a pending op "
                                       "carries its own seeding, so the result would depend on how "
                                       "the two interleave. Call step() or flush_structural() "
                                       "first."));
    }

    // RESOLVE FIRST, WRITE SECOND, as the hand-written body did: every array
    // the walk touches is resolved before any seed is written, so a refusal
    // leaves every seed and every stream as it was.
    Result<std::span<WorldParams>> params = arenas_.array(world_params_id_);
    if (!params) return std::unexpected(params.error());
    for (const modules::CompiledStream& stream : schedule_.streams) {
        const ArrayIndex id = module_array_ids_[stream.array];
        if (const Result<std::span<std::byte>> bytes = arenas_.bytes(id); !bytes) {
            return std::unexpected(bytes.error());
        }
        if (const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(id); !map) {
            return std::unexpected(map.error());
        }
    }

    // Worlds in INDEX order; within a world, derive_streams() walks the
    // streams in declaration order and each array's rows in ascending slot
    // order. Nothing here depends on the iteration order (each write is a pure
    // function of the new world seed and the slot index), but the engine's
    // determinism posture is that an ordered walk is the only kind there is.
    // For the standard set the order is the hand-written body's: per world,
    // dryden, then the live IMU rows, then the live GNSS rows.
    for (uint32_t w = 0; w < layout_.world_count; ++w) {
        WorldParams& row = (*params)[w];

        // replicate()'s formula, verbatim (sim/world_set.cpp's replicate()).
        // Written field-wise into the registered row, which is the world's
        // sole rng authority -- there is no cached copy anywhere to keep in
        // step.
        row.seed = rng::splitmix64(scene_seed ^ rng::fnv1a64(kWorldSeedDomainTag) ^ uint64_t{w});

        // AFTER the seed write, exactly as create() does it. Every array was
        // resolved above, so this cannot fail part-way.
        if (Result<void> derived = derive_streams(w); !derived) return std::unexpected(derived.error());
    }

    // The device mirror is stale (S6 Task 6 review round 1, finding C1). Every
    // write above lands in a REGISTERED array -- WorldParams and every declared
    // stream's array are part of the walk the mirror uploads (a developer's
    // too: the mirror holds every module array, bound or not) -- and none of
    // them goes through the structural queue, so
    // nothing else would say so. Unmarked, a reseed on the vulkan path would
    // be a call that appeared to succeed and changed nothing about the run:
    // the next step() would submit the OLD seeds and the readback would put
    // them straight back into `arenas_`, erasing the reseed.
    //
    // It was already true when Task 6 wrote it, even though the two systems the
    // new seeds feed (Dryden gusts, IMU noise) were then REFUSED outright on
    // this backend: WorldParams::seed is uploaded and read back regardless of
    // whether any pass consumes it, so the erasure would have been observable
    // in a snapshot or a digest immediately. S6 Task 8 ports both systems, so
    // the mark is now load-bearing for the trajectory as well as for the bytes.
    mark_vulkan_dirty();

    return {};
}

Result<void> Simulation::derive_streams(uint32_t world_index) {
    Result<std::span<WorldParams>> params = arenas_.array(world_params_id_);
    if (!params) return std::unexpected(params.error());
    const WorldParams& world = (*params)[world_index];
    for (const modules::CompiledStream& stream : schedule_.streams) {
        const modules::CompiledArray& array = schedule_.arrays[stream.array];
        const ArrayIndex id = module_array_ids_[stream.array];
        const Result<std::span<std::byte>> bytes = arenas_.bytes(id);
        if (!bytes) return std::unexpected(bytes.error());
        const Result<WorldRange> range = arenas_.range(id, world_index);
        if (!range) return std::unexpected(range.error());
        const std::size_t row_bytes = array.elem_size;
        const auto row = [&bytes, row_bytes](uint32_t slot) {
            return bytes->subspan(std::size_t{slot} * row_bytes, row_bytes);
        };

        if (array.extent == modules::Extent::per_world) {
            // The world's one row, always live (compile_schedule allows a
            // stream only here or in a slot-allocated array).
            stream.reseed(row(range->begin), world, 0);
            continue;
        }

        // LIVENESS COMES FROM THE SLOT->WORLD MAP, never from the row's
        // contents -- the same discipline free_rows_of() states and for the
        // same reason: a free row is zero-filled, and writing a fresh stream
        // into one would break the engine-wide "a freed slot reads as zeroes"
        // invariant and put non-zero bytes into every later snapshot of a slot
        // nothing owns. The slot handed over is WORLD-LOCAL, as init_row's is.
        const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(id);
        if (!map) return std::unexpected(map.error());
        for (uint32_t slot = range->begin; slot < range->begin + range->count; ++slot) {
            if ((*map)[slot] != world_index) continue;
            stream.reseed(row(slot), world, slot - range->begin);
        }
    }
    return {};
}

// ---------------------------------------------------------------------------
// inspection
// ---------------------------------------------------------------------------

Result<std::span<const BodyState>> Simulation::world_bodies(uint32_t world_index) const {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    return arenas_.world_slice(bodies_id_, world_index);
}

Result<FrameState> Simulation::vehicle_state(const VehicleRef& vehicle) const {
    const Result<const vehicles::ModelType*> found = model(vehicle.model);
    if (!found) return std::unexpected(found.error());
    const Result<const BodyState*> row = body(vehicle.body);
    if (!row) return std::unexpected(row.error());
    const FrameState state{(*row)->pos, (*row)->orient, (*row)->vel, (*row)->omega_body};
    return to_design_state(state, (*found)->design_to_principal, (*found)->com_offset);
}

Result<const BodyState*> Simulation::body(BodyRef ref) const {
    if (Result<void> valid = validate_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    const Result<std::span<const BodyState>> bodies = arenas_.array(bodies_id_);
    if (!bodies) return std::unexpected(bodies.error());
    return &(*bodies)[ref.slot];
}

Result<uint32_t> Simulation::live_body_count(uint32_t world_index) const {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    return arenas_.live_count(bodies_id_, world_index);
}

Result<uint32_t> Simulation::body_capacity(uint32_t world_index) const {
    if (world_index >= configs_.size()) {
        return std::unexpected(Error{Code::not_found, "world index " + std::to_string(world_index) +
                                                          " is out of range"});
    }
    return configs_[world_index].declared_body_capacity;
}

Result<BodyRef> Simulation::body_ref_at(uint32_t world_index, uint32_t local_slot) const {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    if (local_slot >= layout_.body_capacity) {
        return std::unexpected(invalid("body_ref_at: local slot is outside the world's partition"));
    }
    const uint32_t slot = world_index * layout_.body_capacity + local_slot;

    const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(bodies_id_);
    if (!map) return std::unexpected(map.error());
    if ((*map)[slot] != world_index) {
        return std::unexpected(missing("body_ref_at: slot is not allocated"));
    }

    const Result<std::span<const uint32_t>> generations = arenas_.array(body_gen_id_);
    if (!generations) return std::unexpected(generations.error());
    const uint32_t generation = (*generations)[slot];

    // The slot->world map says "allocated", but an EVEN generation says the
    // release is already queued (see the parity note on BodyRef). Handing back a
    // ref here would re-mint exactly the reference despawn() just killed, and
    // every check in validate_ref() would pass it: a second despawn would queue
    // a second free of the same slot, an add_drag_element would attach an
    // element to a body that stops existing at the same boundary, and an
    // apply_wrench would be zeroed by free_slot's memset without a word.
    if ((generation & 1u) == 0u) {
        return std::unexpected(missing("body_ref_at: slot's despawn is already queued"));
    }
    return BodyRef{world_index, slot, generation};
}

Result<VehicleRef> Simulation::vehicle_ref_at(uint32_t world_index, uint32_t vehicle_ordinal) const {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());

    const Result<std::span<const uint32_t>> body_map = arenas_.slot_to_world(bodies_id_);
    if (!body_map) return std::unexpected(body_map.error());
    const Result<std::span<const uint32_t>> generations = arenas_.array(body_gen_id_);
    if (!generations) return std::unexpected(generations.error());
    const Result<std::span<const uint32_t>> rotor_map = arenas_.slot_to_world(rotors_id_);
    if (!rotor_map) return std::unexpected(rotor_map.error());
    const Result<std::span<const vehicles::RotorRow>> rotor_rows = arenas_.array(rotors_id_);
    if (!rotor_rows) return std::unexpected(rotor_rows.error());
    const Result<std::span<const uint32_t>> sensor_map = arenas_.slot_to_world(imu_id_);
    if (!sensor_map) return std::unexpected(sensor_map.error());
    const Result<std::span<const sensors::ImuSensorRow>> sensor_rows = arenas_.array(imu_id_);
    if (!sensor_rows) return std::unexpected(sensor_rows.error());

    const uint32_t element_begin = world_index * layout_.element_capacity;
    const uint32_t sensor_begin = world_index * layout_.sensor_capacity;

    uint32_t seen = 0;
    for (uint32_t local = 0; local < layout_.body_capacity; ++local) {
        const uint32_t body_slot = world_index * layout_.body_capacity + local;
        if ((*body_map)[body_slot] != world_index) continue;
        const uint32_t generation = (*generations)[body_slot];
        // Parity, exactly as body_ref_at applies it: an even generation means
        // the release is queued, so this vehicle is already dead and is not
        // counted. See the renumbering note in the header.
        if ((generation & 1u) == 0u) continue;

        // The rotor rows this body owns, in ascending slot order. A row's
        // body_slot is written at RESERVATION time (see spawn(vehicle)), so a
        // spawned-but-unflushed vehicle is discoverable here -- which mirrors
        // body_ref_at handing back a ref for a body whose init is still
        // queued, and leaves the "not yet flushed" verdict to the call the
        // caller then makes (set_rotor_commands reports it by name).
        VehicleRef ref;
        uint32_t rotor_count = 0;
        for (uint32_t i = 0; i < layout_.element_capacity; ++i) {
            const uint32_t slot = element_begin + i;
            if ((*rotor_map)[slot] != world_index) continue;
            if ((*rotor_rows)[slot].body_slot != local) continue;
            if (rotor_count >= vehicles::kMaxModelRotors) {
                return std::unexpected(internal(
                    "vehicle_ref_at: body owns more rotors than a model type may declare"));
            }
            ref.rotor_slots[rotor_count] = slot;
            ++rotor_count;
        }
        // No rotors, no vehicle. See the header: a rotor row naming this body
        // is the only evidence in the STATE that it was manufactured from a
        // model type.
        if (rotor_count == 0) continue;
        if (seen != vehicle_ordinal) {
            ++seen;
            continue;
        }

        uint32_t imu_count = 0;
        for (uint32_t i = 0; i < layout_.sensor_capacity; ++i) {
            const uint32_t slot = sensor_begin + i;
            if ((*sensor_map)[slot] != world_index) continue;
            if ((*sensor_rows)[slot].body_slot != local) continue;
            if (imu_count >= vehicles::kMaxModelImuMounts) {
                return std::unexpected(internal(
                    "vehicle_ref_at: body owns more IMU sensors than a model type may declare"));
            }
            ref.imu_sensors[imu_count] = ImuSensorRef{world_index, slot};
            ++imu_count;
        }

        ref.body = BodyRef{world_index, body_slot, generation};
        // ref.model stays null -- there is no ModelTypeId anywhere in the
        // state to read it back from. See the header.
        ref.rotor_count = rotor_count;
        ref.imu_count = imu_count;
        return ref;
    }

    return std::unexpected(missing("vehicle_ref_at: world " + std::to_string(world_index) +
                                   " holds fewer than " + std::to_string(vehicle_ordinal + 1) +
                                   " live vehicles"));
}

// ---------------------------------------------------------------------------
// sensors
// ---------------------------------------------------------------------------

Result<void> Simulation::validate_gnss_ref(GnssSensorRef ref) const {
    return validate_sensor_ref(arenas_, gnss_id_, layout_, ref.world_index, ref.slot,
                               "gnss sensor");
}

Result<GnssPoll> Simulation::poll_gnss(GnssSensorRef ref, sensors::SampleIndex since_index,
                                       std::span<sensors::GnssFix> out) const {
    if (Result<void> valid = validate_gnss_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    return poll_sensor(arenas_, gnss_id_, gnss_ring_id_, ref.slot, since_index, out, "poll_gnss");
}

Result<const sensors::GnssSensorRow*> Simulation::gnss_sensor(GnssSensorRef ref) const {
    if (Result<void> valid = validate_gnss_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    return sensor_row(arenas_, gnss_id_, ref.slot);
}

Result<uint32_t> Simulation::live_gnss_sensor_count(uint32_t world_index) const {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    return arenas_.live_count(gnss_id_, world_index);
}

Result<void> Simulation::validate_imu_ref(ImuSensorRef ref) const {
    return validate_sensor_ref(arenas_, imu_id_, layout_, ref.world_index, ref.slot, "imu sensor");
}

Result<ImuPoll> Simulation::poll_imu(ImuSensorRef ref, sensors::SampleIndex since_index,
                                     std::span<sensors::ImuSample> out) const {
    if (Result<void> valid = validate_imu_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    return poll_sensor(arenas_, imu_id_, imu_ring_id_, ref.slot, since_index, out, "poll_imu");
}

Result<const sensors::ImuSensorRow*> Simulation::imu_sensor(ImuSensorRef ref) const {
    if (Result<void> valid = validate_imu_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    return sensor_row(arenas_, imu_id_, ref.slot);
}

Result<uint32_t> Simulation::live_imu_sensor_count(uint32_t world_index) const {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    return arenas_.live_count(imu_id_, world_index);
}

// ---------------------------------------------------------------------------
// rotors
// ---------------------------------------------------------------------------

Result<const vehicles::RotorRow*> Simulation::rotor(const VehicleRef& ref, uint32_t index) const {
    if (Result<void> valid = validate_ref(ref.body); !valid) {
        return std::unexpected(valid.error());
    }
    if (index >= ref.rotor_count) {
        return std::unexpected(invalid("rotor: index " + std::to_string(index) +
                                       " is at or above the vehicle's rotor count (" +
                                       std::to_string(ref.rotor_count) + ")"));
    }
    const Result<std::span<const vehicles::RotorRow>> rows = arenas_.array(rotors_id_);
    if (!rows) return std::unexpected(rows.error());
    const uint32_t slot = ref.rotor_slots[index];
    if (slot >= rows->size()) {
        return std::unexpected(internal("rotor: slot is outside the array"));
    }
    if ((*rows)[slot].enabled == 0u) {
        return std::unexpected(missing("rotor: the vehicle is spawned but not yet flushed "
                                       "(call step() or flush_structural() first)"));
    }
    return &(*rows)[slot];
}

Result<uint32_t> Simulation::live_rotor_count(uint32_t world_index) const {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    return arenas_.live_count(rotors_id_, world_index);
}

Result<const WorldParams*> Simulation::world_params(uint32_t world_index) const {
    const Result<uint32_t> world = checked_world(world_index);
    if (!world) return std::unexpected(world.error());
    const Result<std::span<const WorldParams>> params = arenas_.array(world_params_id_);
    if (!params) return std::unexpected(params.error());
    return &(*params)[world_index];
}

Result<MediumSample> Simulation::sample_medium(uint32_t world_index, glm::vec3 pos) const {
    const Result<const WorldParams*> params = world_params(world_index);
    if (!params) return std::unexpected(params.error());
    const Result<std::span<const DrydenState>> dryden = arenas_.array(dryden_id_);
    if (!dryden) return std::unexpected(dryden.error());
    // Paired by the same world index rebuild_views() uses, so this is the
    // expression dryden.sample writes into this world's wind field.
    const DrydenMedium medium((*dryden)[world_index], configs_[world_index].turbulence);
    return medium.sample(**params, pos);
}

Result<std::vector<float>> Simulation::field_samples(uint32_t world_index) const {
    if (world_index >= layout_.world_count) {
        return std::unexpected(invalid("field_samples: world " + std::to_string(world_index) +
                                       " is outside a set of " + std::to_string(layout_.world_count)));
    }
    if (vulkan_backend_) {
        // The device rows hold the built-in prefix only (no developer field
        // can run on Vulkan: its provider has no recipe), one FieldSampleRow
        // per world. A diagnostic read: one device->host copy.
        const std::size_t bytes = vulkan_backend_->field_samples_byte_size();
        std::vector<physics::FieldSampleRow> rows(bytes / sizeof(physics::FieldSampleRow));
        if (Result<void> read = vulkan_backend_->read_field_samples(
                std::span<std::byte>(reinterpret_cast<std::byte*>(rows.data()), bytes));
            !read) {
            return std::unexpected(read.error());
        }
        std::vector<float> out(physics::kFieldBuiltinFloats);
        std::memcpy(out.data(), &rows[world_index], sizeof(physics::FieldSampleRow));
        return out;
    }
    const std::size_t stride = schedule_.field_stride;
    const auto begin = field_rows_.begin() + static_cast<std::ptrdiff_t>(world_index * stride);
    return std::vector<float>(begin, begin + static_cast<std::ptrdiff_t>(stride));
}

}  // namespace spade
