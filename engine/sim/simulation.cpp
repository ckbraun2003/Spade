#include "sim/simulation.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include <glm/geometric.hpp>

#include "core/rng.hpp"
#include "core/validate.hpp"
#include "physics/integrator.hpp"
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

// A 64-bit identity, for an error message. Hex because that is how config
// hashes and digests are written everywhere else in this tree
// (the golden corpus's expected_digest, the schema hash) -- a decimal one would
// be ungreppable against them. Hand-rolled rather than via <format>/ostringstream:
// this is one call site on an error path, and neither of those belongs in an
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
// entries by name, never by walk position), so they are spelled once, here, and
// changing one invalidates every recorded blob and every committed digest. The
// ninth, "replay_config", is spelled in sim/simulation.hpp instead
// (kReplayConfigArray) because callers outside this file key on it -- see the
// note at that constant.
constexpr const char* kWorldParamsArray = "world_params";
constexpr const char* kBodiesArray = "bodies";
constexpr const char* kBodyGenerationArray = "body_generation";
constexpr const char* kDragElementsArray = "drag_bodies";
constexpr const char* kDrydenArray = "dryden";
constexpr const char* kImuSensorsArray = "imu_sensors";
constexpr const char* kImuRingArray = "imu_ring";
constexpr const char* kRotorsArray = "rotors";

}  // namespace

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

Result<Simulation> Simulation::create(const WorldSetDesc& desc, uint64_t dt_ns, uint32_t substeps) {
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
    // APPEND-ONLY, ENFORCED BY CONVENTION HERE (no compile-time or runtime
    // check forbids inserting mid-list -- this comment is the guard). A new
    // array belongs AFTER every call below it, never between two existing
    // ones: the golden scenarios' own provenance headers
    // (tests/golden/scenarios/*.scenario.yaml) carry a structural argument
    // ("rotors registered LAST -> pure walk suffix") that a future reader
    // can re-verify by reading THIS function's call order directly --
    // inserting a registration mid-list would silently invalidate that
    // argument for every digest in the corpus at once, not just add a new
    // one to check.
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

    Result<ArrayId<physics::DragBodyRow>> drag_id =
        sim.arenas_.register_array<physics::DragBodyRow>(kDragElementsArray, layout->element_capacity);
    if (!drag_id) return std::unexpected(drag_id.error());
    sim.drag_id_ = *drag_id;

    // One Dryden filter row per world. This is the FIRST production registration
    // of the turbulence state (Task 16 shipped the model and its tests; nothing
    // in the engine had registered the array yet), and registering it here is
    // what makes "snapshot the world, restore it, resume the identical gust
    // sequence" true rather than aspirational.
    Result<ArrayId<DrydenState>> dryden_id = sim.arenas_.register_array<DrydenState>(kDrydenArray, 1);
    if (!dryden_id) return std::unexpected(dryden_id.error());
    sim.dryden_id_ = *dryden_id;

    // The sensor table and its output rings (Task 19). REGISTERED, which is
    // what makes "snapshot mid-flight, restore, and the IMU resumes the
    // identical noise sequence AND the identical unread samples" true: the
    // rows carry the bias random walk, the rate-divider phase, the rng stream
    // and the ring write cursor; the ring array carries the samples themselves.
    // None of it lives in a Simulation member, deliberately.
    //
    // APPENDED AFTER the five that came before, not inserted among them: the
    // walk order is the schema, so appending keeps every earlier array at its
    // existing position in the blob.
    Result<ArrayId<sensors::ImuSensorRow>> imu_id =
        sim.arenas_.register_array<sensors::ImuSensorRow>(kImuSensorsArray, layout->sensor_capacity);
    if (!imu_id) return std::unexpected(imu_id.error());
    sim.imu_id_ = *imu_id;

    // kRingDepth samples PER SENSOR, laid out so that global sensor slot g owns
    // ring slots [g * kRingDepth, (g+1) * kRingDepth) -- see clear_imu_ring().
    // DIRECT-INDEXED, never slot-allocated: a ring window's lifetime is its
    // sensor's, so an independent alloc/free would be a second lifecycle to
    // keep in step with the first.
    Result<ArrayId<sensors::ImuSample>> imu_ring_id = sim.arenas_.register_array<sensors::ImuSample>(
        kImuRingArray, layout->sensor_capacity * sensors::kRingDepth);
    if (!imu_ring_id) return std::unexpected(imu_ring_id.error());
    sim.imu_ring_id_ = *imu_ring_id;

    // The rotor table (Task 18). REGISTERED, and that is NOT optional the way
    // it arguably is for the parameter-only drag rows: a RotorRow carries
    // `omega` -- genuine dynamic state with its own time constant -- and
    // `omega_cmd`, the command in force at the snapshot instant. A snapshot
    // that missed them would restore a vehicle whose rotors are at the wrong
    // speed, or spinning down toward zero, and whose next second of flight
    // differs (vehicles/rotor.hpp says exactly this).
    //
    // SIZED BY THE FORCE-ELEMENT CAPACITY, which the drag table also uses.
    // Rotors and drag bodies are both force elements (spec §3) and share ONE
    // declared budget per world, so a world declaring N force elements can
    // hold at most N rotors -- but the two arrays are separately allocated,
    // which costs a world that never spawns a vehicle N unused RotorRows
    // (80 bytes each; 320 bytes for the corpus's largest world). Accepted
    // deliberately: the alternative is either a per-kind capacity in the world
    // FILE (S5's format, for a distinction a world author should not have to
    // predict) or a conditional registration, which would make the state
    // layer's shape depend on its contents.
    //
    // APPENDED LAST, like the sensor arrays before it: the walk order is the
    // schema, so appending keeps every earlier array at its existing position
    // in the blob.
    Result<ArrayId<vehicles::RotorRow>> rotors_id =
        sim.arenas_.register_array<vehicles::RotorRow>(kRotorsArray, layout->element_capacity);
    if (!rotors_id) return std::unexpected(rotors_id.error());
    sim.rotors_id_ = *rotors_id;

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

    // -----------------------------------------------------------------------
    // Seed the per-world rows.
    // -----------------------------------------------------------------------
    Result<std::span<WorldParams>> params = sim.arenas_.array(sim.world_params_id_);
    if (!params) return std::unexpected(params.error());
    Result<std::span<DrydenState>> dryden = sim.arenas_.array(sim.dryden_id_);
    if (!dryden) return std::unexpected(dryden.error());

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

        // Places the filter on its stationary distribution using row.seed, so
        // the world starts gusty rather than burning off a spin-up transient.
        // Must come AFTER the seed is written.
        dryden_init((*dryden)[w], row);
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
    // One-shot sizing so the broad phase never allocates in the steady state
    // (physics/grid.hpp's GridScratch note). Worst case is one entry and one
    // run per body slot in the whole set.
    sim.scratch_.reserve(static_cast<std::size_t>(layout->world_count) *
                         static_cast<std::size_t>(layout->body_capacity));

    if (Result<void> views = sim.rebuild_views(); !views) {
        return std::unexpected(views.error());
    }

    return sim;
}

// ---------------------------------------------------------------------------
// views
// ---------------------------------------------------------------------------
//
// FIVE ARRAYS ARE DIRECT-INDEXED RATHER THAN SLOT-ALLOCATED: world_params,
// dryden and replay_config (one row per world, indexed by world id -- they have
// no lifecycle, so alloc_slot/free_slot would be ceremony around a constant),
// body_generation (one row per BODY slot, indexed by that slot -- it must
// SURVIVE its slot being freed, and free_slot zero-fills, which would erase the
// very counter it exists to preserve), and imu_ring (kRingDepth rows per SENSOR
// slot, indexed by that slot -- its lifetime IS its sensor's, so a second
// alloc/free lifecycle would only be a thing to keep in step; clear_imu_ring()
// does the zeroing the sensor's own free_slot does for the row).
//
// The consequence is that those five arrays' slot->world maps stay uniformly
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
        // ForceElements samples a DrydenMedium built from world w's filter
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
        view.dryden = &(*dryden)[w];
        view.dryden_params = &configs_[w].turbulence;
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
        // Subscripted by the SAME `w` and, crucially, by the SAME
        // element_capacity as `drag_elements` above -- which is what makes
        // RotorRow::body_slot (a WORLD-LOCAL body index, exactly like
        // DragBodyRow's) agree with the `bodies` span bound three lines up.
        view.rotors = rotors->subspan(elem_begin, layout_.element_capacity);
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
        // step(1) minus the physics.
        return flush_structural();
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
        ctx.h = h_;
        // The step being executed -- what SensorSynthesis stamps samples with.
        // Taken BEFORE the increment below, so every substep of step k stamps
        // k (Tick counts steps, not substeps).
        ctx.tick = tick_;

        for (uint32_t s = 0; s < substeps_; ++s) {
            physics::run_substep(ctx);
        }

        // Tick counts STEPS, not substeps -- see pass_publish()'s note on §3's
        // shorthand. One increment, after the last substep.
        ++tick_;
    }

    return {};
}

// ---------------------------------------------------------------------------
// structural queue
// ---------------------------------------------------------------------------

Result<void> Simulation::flush_structural() {
    if (queue_.empty()) {
        return {};
    }

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
            // meaning would make this correct by coincidence.
            b.pos = op.body.pos;
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
            // Cascade first: the elements and sensors are addressed relative to
            // the body that is about to stop existing.
            free_drag_elements_of(op.world_index, op.slot);
            free_imu_sensors_of(op.world_index, op.slot);
            free_rotors_of(op.world_index, op.slot);
            if (Result<void> freed = arenas_.free_slot(bodies_id_, op.slot); !freed) {
                return std::unexpected(internal("structural queue: freeing a body slot failed: " +
                                                freed.error().context));
            }
            return {};
        }

        case OpKind::init_drag: {
            Result<std::span<physics::DragBodyRow>> rows = arenas_.array(drag_id_);
            if (!rows) return std::unexpected(rows.error());
            if (op.slot >= rows->size()) {
                return std::unexpected(internal("structural queue: element slot out of range"));
            }
            // DEFENCE IN DEPTH: if this element's slot was released earlier in
            // this same flush, its body no longer exists and there is nothing to
            // initialize. With `body_slot` written at reservation time (see
            // add_drag_element) the cascade cannot reach a row belonging to a
            // DIFFERENT body, and a row belonging to the SAME body is always
            // initialized before that body's free is applied (a ref for a body
            // whose despawn is queued is rejected, so the element can only have
            // been reserved first). So this is unreachable today -- and it is
            // cheap insurance that a future queue reordering degrades to a
            // no-op rather than to a live row in a free slot.
            const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(drag_id_);
            if (!map) return std::unexpected(map.error());
            if ((*map)[op.slot] != op.world_index) return {};

            physics::DragBodyRow& row = (*rows)[op.slot];

            // WORLD-LOCAL, not global: physics/forces.hpp documents body_slot as
            // an index into the world's body slice, which is what apply_drag()
            // is handed. Already written at reservation time; rewritten here so
            // this function remains the complete statement of the row's contents.
            row.body_slot = op.body_slot - op.world_index * layout_.body_capacity;
            row.enabled = 1u;
            row.mode = op.drag.mode;
            row.area = op.drag.area;
            row.local_pos = op.drag.local_pos;
            row.local_orient = glm::normalize(op.drag.local_orient);
            row.coeffs = op.drag.coeffs;
            return {};
        }

        case OpKind::init_imu: {
            Result<std::span<sensors::ImuSensorRow>> rows = arenas_.array(imu_id_);
            if (!rows) return std::unexpected(rows.error());
            if (op.slot >= rows->size()) {
                return std::unexpected(internal("structural queue: sensor slot out of range"));
            }
            // Same defence in depth as init_drag, for the same reason: if this
            // slot was released earlier in this same flush its body no longer
            // exists, so there is nothing to initialize. Unreachable today (the
            // cascade's test is exact because `body_slot` is written at
            // reservation time, below), cheap insurance against a future queue
            // reordering.
            const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(imu_id_);
            if (!map) return std::unexpected(map.error());
            if ((*map)[op.slot] != op.world_index) return {};

            const Result<std::span<const WorldParams>> params = arenas_.array(world_params_id_);
            if (!params) return std::unexpected(params.error());

            sensors::ImuSensorRow& row = (*rows)[op.slot];
            const uint32_t local_slot = op.slot - op.world_index * layout_.sensor_capacity;

            // FIELD-WISE, NOT WHOLE-OBJECT -- see init_body. Every field is
            // written even where the arena's zero-fill would already have done
            // it: the row's contents are this function's complete statement,
            // not a coincidence of how the slot came to be free.
            row.body_slot = op.body_slot - op.world_index * layout_.body_capacity;
            row.rate_divider = op.imu.rate_divider;
            row.phase = 0u;
            row.mount_pos = op.imu.mount_pos;
            row._p0 = 0.0f;
            row.mount_orient = glm::normalize(op.imu.mount_orient);
            row.sigma_a = op.imu.sigma_a;
            row.sigma_g = op.imu.sigma_g;
            row.sigma_ba = op.imu.sigma_ba;
            row.sigma_bg = op.imu.sigma_bg;
            row.bias_a = glm::vec3(0.0f);
            row._p1 = 0.0f;
            row.bias_g = glm::vec3(0.0f);
            row._p2 = 0.0f;
            // Seeded from the REGISTERED WorldParams row -- the world's rng
            // authority (see WorldConfig's "NO `seed` MEMBER" note) -- and from
            // the WORLD-LOCAL slot, so a world's noise does not depend on where
            // that world sits in the set.
            row.noise = sensors::imu_noise_stream((*params)[op.world_index].seed, local_slot);
            row.last_index = 0;
            row._reserved0 = 0;
            // LAST, like body_flags::active in init_body: until `kind` is set
            // the row is inert to the SensorSynthesis pass, so a partially
            // written row can never be sampled.
            row.kind = sensors::sensor_kind::imu;
            return {};
        }

        case OpKind::init_rotor: {
            Result<std::span<vehicles::RotorRow>> rows = arenas_.array(rotors_id_);
            if (!rows) return std::unexpected(rows.error());
            if (op.slot >= rows->size()) {
                return std::unexpected(internal("structural queue: rotor slot out of range"));
            }
            // Same defence in depth as init_drag and init_imu, and for the
            // same reason: a slot released earlier in this same flush belongs
            // to a body that no longer exists.
            const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(rotors_id_);
            if (!map) return std::unexpected(map.error());
            if ((*map)[op.slot] != op.world_index) return {};

            vehicles::RotorRow& row = (*rows)[op.slot];

            // FIELD-WISE, NOT WHOLE-OBJECT -- see init_body. Every field is
            // written, including the four reserved lanes: rotor.hpp requires
            // them to stay 0, and stating that here rather than relying on the
            // arena's zero-fill makes this function the row's complete
            // definition.
            row.body_slot = op.body_slot - op.world_index * layout_.body_capacity;
            row.tau = op.rotor.tau;
            row.radius = op.rotor.radius;
            row.local_pos = op.rotor.local_pos;
            row.spin_dir = op.rotor.spin_dir;
            row.local_orient = glm::normalize(op.rotor.local_orient);
            // SPAWNED IN TRIM: omega and omega_cmd both take the spawn's
            // rotor_omega, so a vehicle inserted mid-flight holds its speed
            // instead of spinning up from rest. See VehicleSpawn.
            row.omega = op.rotor_omega;
            row.omega_cmd = op.rotor_omega;
            row.thrust_coeff = op.rotor.thrust_coeff;
            row.torque_coeff = op.rotor.torque_coeff;
            row._r0 = 0.0f;
            row._r1 = 0.0f;
            row._r2 = 0.0f;
            row._r3 = 0.0f;
            // LAST, like body_flags::active in init_body and `kind` in
            // init_imu: until `enabled` is set the row is inert to
            // apply_rotors(), so a partially written row can never be stepped.
            row.enabled = 1u;
            return {};
        }
    }
    return std::unexpected(internal("structural queue: unknown op kind"));
}

void Simulation::free_drag_elements_of(uint32_t world_index, uint32_t body_slot) {
    Result<std::span<const uint32_t>> map = arenas_.slot_to_world(drag_id_);
    Result<std::span<physics::DragBodyRow>> rows = arenas_.array(drag_id_);
    if (!map || !rows) return;

    const uint32_t local_body = body_slot - world_index * layout_.body_capacity;
    const uint32_t begin = world_index * layout_.element_capacity;
    const uint32_t end = begin + layout_.element_capacity;

    // Ascending slot order, so the free sequence -- and therefore every future
    // allocation out of the rebuilt free list -- is a function of the slot
    // indices alone. Liveness comes from the slot->world map and NOT from the
    // row's contents: a freed row is zero-filled, and body_slot 0 is a perfectly
    // legitimate world-local index, so trusting the row would free live elements
    // attached to body 0.
    //
    // `map` stays valid across the free_slot() calls below: it spans a vector
    // that is sized once at registration and never resized, so only the VALUES
    // change (to kInvalidWorld, for slots this loop has already passed).
    for (uint32_t slot = begin; slot < end; ++slot) {
        if ((*map)[slot] != world_index) continue;
        if ((*rows)[slot].body_slot != local_body) continue;
        (void)arenas_.free_slot(drag_id_, slot);
    }
}

void Simulation::free_imu_sensors_of(uint32_t world_index, uint32_t body_slot) {
    Result<std::span<const uint32_t>> map = arenas_.slot_to_world(imu_id_);
    Result<std::span<sensors::ImuSensorRow>> rows = arenas_.array(imu_id_);
    if (!map || !rows) return;

    const uint32_t local_body = body_slot - world_index * layout_.body_capacity;
    const uint32_t begin = world_index * layout_.sensor_capacity;
    const uint32_t end = begin + layout_.sensor_capacity;

    // Ascending slot order and liveness-from-the-map, exactly as
    // free_drag_elements_of does and for exactly the same reasons (a
    // deterministic free sequence; a zero-filled freed row's body_slot 0 is a
    // legitimate world-local index, so trusting the row alone would free live
    // sensors attached to body 0).
    for (uint32_t slot = begin; slot < end; ++slot) {
        if ((*map)[slot] != world_index) continue;
        if ((*rows)[slot].body_slot != local_body) continue;
        if (Result<void> freed = arenas_.free_slot(imu_id_, slot); freed) {
            // free_slot zeroes the ROW; the samples live in a second,
            // direct-indexed array that nothing else would clear. See
            // Simulation::despawn's contract.
            clear_imu_ring(slot);
        }
    }
}

void Simulation::free_rotors_of(uint32_t world_index, uint32_t body_slot) {
    Result<std::span<const uint32_t>> map = arenas_.slot_to_world(rotors_id_);
    Result<std::span<vehicles::RotorRow>> rows = arenas_.array(rotors_id_);
    if (!map || !rows) return;

    const uint32_t local_body = body_slot - world_index * layout_.body_capacity;
    // The ROTOR partition is sized by element_capacity, the same number the
    // drag table uses -- see create()'s registration note on the shared
    // force-element budget.
    const uint32_t begin = world_index * layout_.element_capacity;
    const uint32_t end = begin + layout_.element_capacity;

    // Ascending slot order and liveness-from-the-map, exactly as
    // free_drag_elements_of does and for exactly the same two reasons (a
    // deterministic free sequence; a zero-filled freed row's body_slot 0 is a
    // legitimate world-local index, so trusting the row alone would free live
    // rotors attached to body 0).
    for (uint32_t slot = begin; slot < end; ++slot) {
        if ((*map)[slot] != world_index) continue;
        if ((*rows)[slot].body_slot != local_body) continue;
        (void)arenas_.free_slot(rotors_id_, slot);
    }
}

void Simulation::clear_imu_ring(uint32_t slot) {
    Result<std::span<sensors::ImuSample>> ring = arenas_.array(imu_ring_id_);
    if (!ring) return;
    const std::size_t begin = static_cast<std::size_t>(slot) * sensors::kRingDepth;
    if (begin + sensors::kRingDepth > ring->size()) return;

    for (std::size_t i = 0; i < sensors::kRingDepth; ++i) {
        // FIELD-WISE, like every other arena write in this file. ImuSample has
        // no implicit padding (sensors/imu.hpp asserts it), so naming all six
        // fields zeroes every byte -- which is what makes a freed sensor's ring
        // read as zeroes in a snapshot, the same as any other freed slot.
        sensors::ImuSample& sample = (*ring)[begin + i];
        sample.accel = glm::vec3(0.0f);
        sample._p0 = 0.0f;
        sample.gyro = glm::vec3(0.0f);
        sample._p1 = 0.0f;
        sample.index = 0;
        sample.tick = 0;
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

Result<DragElementRef> Simulation::add_drag_element(BodyRef ref, const DragElementSpawn& elem) {
    if (Result<void> valid = validate_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    if (!finite(elem.area) || !finite(elem.coeffs) || !finite(elem.local_pos) ||
        !finite(elem.local_orient)) {
        return std::unexpected(invalid("add_drag_element: parameters must all be finite"));
    }
    if (elem.mode != physics::drag_mode::quadratic && elem.mode != physics::drag_mode::componentwise) {
        return std::unexpected(invalid("add_drag_element: unknown drag mode"));
    }
    if (!(glm::dot(elem.local_orient, elem.local_orient) > 0.0f)) {
        return std::unexpected(invalid("add_drag_element: local_orient must have non-zero length"));
    }

    const WorldConfig& config = configs_[ref.world_index];
    // THE SHARED FORCE-ELEMENT BUDGET (Task 18): rotors and drag bodies live
    // in separate arena arrays but count against ONE declared capacity,
    // because spec §3 calls them both force elements and a world file declares
    // how many force elements a world holds, not how many of each kind. A
    // world with rotors in it therefore has fewer drag slots available than it
    // has drag STORAGE, which is the honest reading of its own declaration.
    const Result<uint32_t> live = live_force_elements(ref.world_index);
    if (!live) return std::unexpected(live.error());
    if (*live >= config.declared_element_capacity) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "add_drag_element: world " + std::to_string(ref.world_index) +
                                         " is at its declared force-element capacity (" +
                                         std::to_string(config.declared_element_capacity) +
                                         ", shared between drag bodies and rotors)"});
    }

    const Result<uint32_t> slot = arenas_.alloc_slot(drag_id_, ref.world_index);
    if (!slot) return std::unexpected(slot.error());

    // -----------------------------------------------------------------------
    // THE ROW'S IDENTITY IS WRITTEN AT RESERVATION TIME; ONLY ITS PARAMETERS
    // AND `enabled` WAIT FOR THE BOUNDARY. This is not an optimization, it
    // closes an aliasing hole:
    //
    // a reserved row is otherwise the arena's zeroes, i.e. `body_slot == 0` --
    // and 0 is a perfectly legitimate world-local body index. The despawn
    // cascade (free_drag_elements_of) identifies a body's elements by exactly
    // that field, so a row reserved while a despawn of world-local body 0 was
    // already queued would be mistaken for one of body 0's elements and freed,
    // after which the queued init_drag would write a live row into a slot the
    // arena considers free -- breaking the "a freed slot reads as zeroes"
    // invariant, under-counting live_count, letting a later reservation alias
    // the same row, and leaving an orphan element that applies drag to whatever
    // body next occupies that body slot. Every one of those is silent.
    //
    // Writing `body_slot` here makes the reserved row TRUTHFUL about which body
    // it belongs to from the instant it exists, so the cascade's test is exact.
    // `enabled` stays 0 (the arena's zero-fill), so the row is still inert to
    // apply_drag until the boundary -- the same "reserved but not yet live"
    // posture spawn() gives a body slot via body_flags::active.
    // -----------------------------------------------------------------------
    Result<std::span<physics::DragBodyRow>> rows = arenas_.array(drag_id_);
    if (!rows) return std::unexpected(rows.error());
    (*rows)[*slot].body_slot = ref.slot - ref.world_index * layout_.body_capacity;

    StructuralOp op;
    op.kind = OpKind::init_drag;
    op.world_index = ref.world_index;
    op.slot = *slot;
    op.body_slot = ref.slot;
    op.drag = elem;
    queue_.push_back(op);

    return DragElementRef{ref.world_index, *slot};
}

Result<ImuSensorRef> Simulation::add_imu_sensor(BodyRef ref, const ImuSensorSpawn& sensor) {
    if (Result<void> valid = validate_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    if (sensor.rate_divider == 0) {
        // 0 would mean "no rate at all". synthesize_imu() degrades it to 1
        // rather than dividing by zero, but a caller who wrote 0 meant
        // something, and it was not "every substep".
        return std::unexpected(invalid("add_imu_sensor: rate_divider must be >= 1"));
    }
    if (!finite(sensor.mount_pos) || !finite(sensor.mount_orient)) {
        return std::unexpected(invalid("add_imu_sensor: mount pose must be finite"));
    }
    if (!(glm::dot(sensor.mount_orient, sensor.mount_orient) > 0.0f)) {
        return std::unexpected(invalid("add_imu_sensor: mount_orient must have non-zero length"));
    }
    if (!(sensor.sigma_a >= 0.0f) || !(sensor.sigma_g >= 0.0f) || !(sensor.sigma_ba >= 0.0f) ||
        !(sensor.sigma_bg >= 0.0f) || !finite(sensor.sigma_a) || !finite(sensor.sigma_g) ||
        !finite(sensor.sigma_ba) || !finite(sensor.sigma_bg)) {
        // Spelled `!(x >= 0)` so a NaN rejects rather than comparing false on
        // both sides -- the same discipline world_set.cpp's in_range() uses.
        return std::unexpected(invalid("add_imu_sensor: every sigma must be finite and >= 0"));
    }

    const WorldConfig& config = configs_[ref.world_index];
    const Result<uint32_t> live = arenas_.live_count(imu_id_, ref.world_index);
    if (!live) return std::unexpected(live.error());
    if (*live >= config.declared_sensor_capacity) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "add_imu_sensor: world " + std::to_string(ref.world_index) +
                                         " is at its declared sensor capacity (" +
                                         std::to_string(config.declared_sensor_capacity) + ")"});
    }

    const Result<uint32_t> slot = arenas_.alloc_slot(imu_id_, ref.world_index);
    if (!slot) return std::unexpected(slot.error());

    // THE ROW'S IDENTITY IS WRITTEN AT RESERVATION TIME -- see the long note in
    // add_drag_element() for the aliasing hole this closes. It is the same hole:
    // a reserved row's zeroed `body_slot` is a legitimate world-local index
    // (body 0), and free_imu_sensors_of() identifies a body's sensors by exactly
    // that field. `kind` stays 0 (the arena's zero-fill), so the row is still
    // inert to the SensorSynthesis pass until the boundary.
    Result<std::span<sensors::ImuSensorRow>> rows = arenas_.array(imu_id_);
    if (!rows) return std::unexpected(rows.error());
    (*rows)[*slot].body_slot = ref.slot - ref.world_index * layout_.body_capacity;

    StructuralOp op;
    op.kind = OpKind::init_imu;
    op.world_index = ref.world_index;
    op.slot = *slot;
    op.body_slot = ref.slot;
    op.imu = sensor;
    queue_.push_back(op);

    return ImuSensorRef{ref.world_index, *slot};
}

Result<uint32_t> Simulation::live_force_elements(uint32_t world_index) const {
    const Result<uint32_t> drag = arenas_.live_count(drag_id_, world_index);
    if (!drag) return std::unexpected(drag.error());
    const Result<uint32_t> rotors = arenas_.live_count(rotors_id_, world_index);
    if (!rotors) return std::unexpected(rotors.error());
    return *drag + *rotors;
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
    if (models_.size() >= std::numeric_limits<uint32_t>::max() - 1u) {
        return std::unexpected(Error{Code::capacity_exceeded, "register_model: model id space is full"});
    }
    models_.push_back(std::move(model));
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
// already taken is released here. A reserve/release pair leaves the arena's
// free SET exactly as it was (state/arenas.hpp: the bump-cursor/free-list pair
// is canonical-equivalent, and alloc_slot only ever consults the free set), so
// an unwound spawn is invisible to every future allocation -- which is what
// keeps a failed spawn from perturbing a determinism replay.
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
    // NOT BURIED IN THE WORLD -- the same check, for the same uncapped-
    // Baumgarte reason, that spawn(world, BodySpawn) applies. Spelled
    // `!(phi >= -r)` so a NaN field value rejects.
    const float phi = eval(config.sdf, where.pos);
    if (!(phi >= -config.contacts.proxy_radius)) {
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
    // returns `error`. Unreachable given the checks above; see the unwind note.
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
    // spelled out at length in add_drag_element(). A reserved row is otherwise
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
    StructuralOp body_op;
    body_op.kind = OpKind::init_body;
    body_op.world_index = world_index;
    body_op.slot = *body_slot;
    body_op.body.pos = where.pos;
    body_op.body.orient = where.orient;
    body_op.body.vel = where.vel;
    body_op.body.omega_body = where.omega_body;
    body_op.body.mass = model.body.mass;
    // The model carries the INERTIA; BodyState stores its inverse. Inverted
    // here, in one place. Validated componentwise > 0 by ModelType::validate(),
    // so no division guard is needed and none is written -- a guard here would
    // silently admit the model the validator exists to refuse.
    body_op.body.inv_inertia_diag = glm::vec3(1.0f / model.body.inertia_diag.x,
                                              1.0f / model.body.inertia_diag.y,
                                              1.0f / model.body.inertia_diag.z);
    queue_.push_back(body_op);

    for (std::size_t i = 0; i < rotors_taken; ++i) {
        StructuralOp op;
        op.kind = OpKind::init_rotor;
        op.world_index = world_index;
        op.slot = rotor_slots[i];
        op.body_slot = *body_slot;
        op.rotor = model.rotors[i];
        op.rotor_omega = where.rotor_omega;
        queue_.push_back(op);
    }
    for (std::size_t i = 0; i < drags_taken; ++i) {
        // The desc -> spawn-record mapping vehicles/model_type.hpp promises
        // lives in exactly one place. This is it, for drag bodies.
        const vehicles::DragBodyDesc& desc = model.drag_bodies[i];
        StructuralOp op;
        op.kind = OpKind::init_drag;
        op.world_index = world_index;
        op.slot = drag_slots[i];
        op.body_slot = *body_slot;
        op.drag.mode = desc.mode;
        op.drag.area = desc.area;
        op.drag.coeffs = desc.coeffs;
        op.drag.local_pos = desc.local_pos;
        op.drag.local_orient = desc.local_orient;
        queue_.push_back(op);
    }
    for (std::size_t i = 0; i < sensors_taken; ++i) {
        // ... and this is it for IMU mounts.
        const vehicles::ImuMountDesc& desc = model.imu_mounts[i];
        StructuralOp op;
        op.kind = OpKind::init_imu;
        op.world_index = world_index;
        op.slot = sensor_slots[i];
        op.body_slot = *body_slot;
        op.imu.mount_pos = desc.mount_pos;
        op.imu.mount_orient = desc.mount_orient;
        op.imu.rate_divider = desc.rate_divider;
        op.imu.sigma_a = desc.sigma_a;
        op.imu.sigma_g = desc.sigma_g;
        op.imu.sigma_ba = desc.sigma_ba;
        op.imu.sigma_bg = desc.sigma_bg;
        queue_.push_back(op);
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
    return save(arenas_, tick_);
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
// See the doc comment in simulation.hpp for the full argument -- in particular
// for WHY the three writes below are the complete list. The short form: the
// world seed is read at exactly three sites in the engine, and this function is
// the mirror image of all three.
// ---------------------------------------------------------------------------

Result<void> Simulation::reseed(uint64_t scene_seed) {
    if (!queue_.empty()) {
        return std::unexpected(invalid("reseed: the structural queue is not empty; a pending op "
                                       "carries its own seeding, so the result would depend on how "
                                       "the two interleave. Call step() or flush_structural() "
                                       "first."));
    }

    Result<std::span<WorldParams>> params = arenas_.array(world_params_id_);
    if (!params) return std::unexpected(params.error());
    Result<std::span<DrydenState>> dryden = arenas_.array(dryden_id_);
    if (!dryden) return std::unexpected(dryden.error());
    Result<std::span<sensors::ImuSensorRow>> sensors_rows = arenas_.array(imu_id_);
    if (!sensors_rows) return std::unexpected(sensors_rows.error());
    // LIVENESS COMES FROM THE SLOT->WORLD MAP, never from the row's contents --
    // the same discipline free_imu_sensors_of() states and for the same reason:
    // a free row is zero-filled, and writing a fresh stream into one would break
    // the engine-wide "a freed slot reads as zeroes" invariant and put sixteen
    // non-zero bytes into every subsequent snapshot of a slot nothing owns.
    Result<std::span<const uint32_t>> sensor_map = arenas_.slot_to_world(imu_id_);
    if (!sensor_map) return std::unexpected(sensor_map.error());

    // Worlds in INDEX order, and each world's sensors in ascending slot order.
    // Nothing here depends on the iteration order (each write is a pure function
    // of the new world seed and the slot index), but the engine's determinism
    // posture is that an ordered walk is the only kind there is.
    for (uint32_t w = 0; w < layout_.world_count; ++w) {
        WorldParams& row = (*params)[w];

        // replicate()'s formula, verbatim (sim/world_set.cpp's replicate()).
        // Written field-wise into the registered row, which is the world's
        // sole rng authority -- there is no cached copy anywhere to keep in
        // step.
        row.seed = rng::splitmix64(scene_seed ^ rng::fnv1a64(kWorldSeedDomainTag) ^ uint64_t{w});

        // AFTER the seed write, exactly as create() does it: dryden_init reads
        // row.seed and re-places the filter on its stationary distribution.
        dryden_init((*dryden)[w], row);

        const uint32_t begin = w * layout_.sensor_capacity;
        const uint32_t end = begin + layout_.sensor_capacity;
        for (uint32_t slot = begin; slot < end; ++slot) {
            if ((*sensor_map)[slot] != w) continue;
            // ONLY `noise`. The bias states, the divider phase, the ring cursor
            // and the ring itself are HISTORY and stay exactly as they are --
            // see the header: a reseed changes the future draws, not the past.
            (*sensors_rows)[slot].noise = sensors::imu_noise_stream(row.seed, slot - begin);
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

Result<void> Simulation::validate_imu_ref(ImuSensorRef ref) const {
    if (ref.world_index >= layout_.world_count) {
        return std::unexpected(missing("imu sensor ref names a world outside this set"));
    }
    // The same redundancy check validate_ref() applies to a BodyRef: the world
    // index must agree with the partition the slot falls in, which catches a
    // hand-built ref or one from a differently-shaped set before it indexes.
    if (layout_.sensor_capacity == 0 || ref.slot / layout_.sensor_capacity != ref.world_index) {
        return std::unexpected(missing("imu sensor ref's slot does not lie in its world's partition"));
    }
    const Result<std::span<const uint32_t>> map = arenas_.slot_to_world(imu_id_);
    if (!map) return std::unexpected(map.error());
    if (ref.slot >= map->size() || (*map)[ref.slot] != ref.world_index) {
        return std::unexpected(missing("imu sensor ref names a slot that is not allocated"));
    }
    return {};
}

Result<ImuPoll> Simulation::poll_imu(ImuSensorRef ref, sensors::SampleIndex since_index,
                                     std::span<sensors::ImuSample> out) const {
    if (Result<void> valid = validate_imu_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    const Result<std::span<const sensors::ImuSensorRow>> rows = arenas_.array(imu_id_);
    if (!rows) return std::unexpected(rows.error());
    const Result<std::span<const sensors::ImuSample>> ring = arenas_.array(imu_ring_id_);
    if (!ring) return std::unexpected(ring.error());

    // The window is the sensor's global slot times the depth -- the implicit
    // ring reference sensors/imu.hpp describes.
    const std::size_t begin = static_cast<std::size_t>(ref.slot) * sensors::kRingDepth;
    if (begin + sensors::kRingDepth > ring->size()) {
        return std::unexpected(internal("poll_imu: ring window is outside the ring array"));
    }

    // A row whose init is still queued has kind == none and last_index == 0, so
    // this reports "nothing yet" rather than an error -- polling a sensor you
    // just added, before the next step, is a legal thing to do.
    return sensors::ring_poll<sensors::ImuSample>(ring->subspan(begin, sensors::kRingDepth),
                                                  (*rows)[ref.slot].last_index, since_index, out);
}

Result<const sensors::ImuSensorRow*> Simulation::imu_sensor(ImuSensorRef ref) const {
    if (Result<void> valid = validate_imu_ref(ref); !valid) {
        return std::unexpected(valid.error());
    }
    const Result<std::span<const sensors::ImuSensorRow>> rows = arenas_.array(imu_id_);
    if (!rows) return std::unexpected(rows.error());
    return &(*rows)[ref.slot];
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

}  // namespace spade
