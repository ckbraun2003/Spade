#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "physics/contacts.hpp"
#include "physics/grid.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"
#include "world/world_ref.hpp"

// ---------------------------------------------------------------------------
// The world set -- what a Simulation is created from (engine design spec §3
// "A Simulation owns a WorldSet and steps it", §4 "Many-worlds": "A WorldSet
// template ('N worlds from this world file, capacities X') is the
// training-fleet constructor; heterogeneous sets are allowed but uniform sets
// get the tightest layouts").
//
// THIS HEADER IS DESCRIPTION ONLY -- no state, no arenas, no I/O. It is the
// authoring product (WorldDesc, from world/builder.hpp) plus the four runtime
// parameter records the passes need, one per world, and the validation that
// turns "a vector of those" into a shape a Simulation can allocate against.
// The Simulation owns the state; this owns the recipe.
//
// WHY THE PARAMETER RECORDS RIDE HERE RATHER THAN IN WorldDesc. WorldDesc is
// the world FILE's product (S5 serializes exactly it): geometry, spawn points,
// environment, capacities. Material and solver parameters -- restitution, the
// grid cell size, the turbulence intensity -- are properties of the RUN, not
// of the geometry: the same race course is legitimately instantiated at four
// turbulence levels in one training fleet. Splitting them here is what lets
// replicate() below build that fleet from one WorldDesc without mutating it.
// ---------------------------------------------------------------------------

namespace spade {

// ---------------------------------------------------------------------------
// One world instance: the geometry plus everything the per-substep passes read
// that is not per-body state.
//
// SEED. `seed` is THE authority for this instance's rng root -- it is what
// lands in WorldParams::seed and therefore what every stochastic system in the
// world derives its stream from (core/rng.hpp's make_stream(world_seed, tag,
// index)). WorldDesc::environment.seed is the AUTHORING default that a world
// file carries; Simulation ignores it, because the whole point of a world set
// is N instances of one world file at N different seeds. A loader that wants
// the file's seed copies it into this field.
//
// DrydenParams IS STORED HERE, NOT IN WorldParams, and that is a ruling rather
// than an accident: WorldParams is a std430 wire contract with exactly eight
// reserved bytes (state/layout.hpp), and DrydenParams is 32. It is
// configuration, not state -- the per-world FILTER STATE (DrydenState) is a
// registered array and rides every snapshot; these parameters do not, so a
// blob restored into a Simulation built from a DIFFERENT WorldSetDesc replays
// the same state forward under different physics. See Simulation::restore().
//
// ---------------------------------------------------------------------------
// READ THIS BEFORE CONFIGURING `turbulence`: AT THE DEFAULT REFERENCE AIRSPEED,
// A GUST IS A PER-EPISODE CONSTANT BIAS, NOT A BUFFET.
//
// The Dryden model is a FROZEN-TURBULENCE mapping (world/medium.hpp §1): a
// spatial scale length L becomes a temporal correlation time
//
//     tau = L / V,   V = DrydenParams::reference_airspeed
//
// and V is a CONFIGURED CONSTANT, defaulting to 5 m/s, not the vehicle's actual
// airspeed. With the default L_u = L_v = 200 m that gives
//
//     tau_u = tau_v = 40 s        horizontal
//     tau_w = 10 s                vertical (L_w = 50 m)
//
// A training episode is seconds to tens of seconds long. Over 5 s the
// horizontal gust decorrelates by exp(-5/40) = 12%: for practical purposes each
// episode draws ONE horizontal gust vector at spawn and flies in it for the
// whole episode. Per-axis one-sigma values are 1.23 / 2.46 / 3.69 m/s at
// light / moderate / severe, so a severe world hands a typical episode a steady
// horizontal wind bias anywhere in roughly +-7 m/s (two sigma) -- comparable to
// the vehicle's own airspeed, and constant for the flight.
//
// THAT IS THE MODEL BEHAVING CORRECTLY, not a bug, and it has two practical
// consequences worth stating where the knob is:
//
//   * A POLICY TRAINED ON ONE SEED SEES ONE WIND. Turbulence robustness comes
//     from varying the world SEED across episodes (replicate() below is exactly
//     that constructor), not from waiting for the gust to change within one.
//   * RAISING `reference_airspeed` SHORTENS tau. It is the only knob that
//     converts the bias into a buffet: V = 40 m/s gives tau_u = 5 s. Lowering
//     it lengthens the correlation further. A per-body airspeed-driven mapping
//     and a spatially correlated field are both S8 roadmap, not v1.
// ---------------------------------------------------------------------------
#if defined(_MSC_VER)
#pragma warning(push)
// C4324: "structure was padded due to alignment specifier". ContactParams and
// GridParams are alignas(16) because they are std430 records the S6 device
// buffers mirror, so ANY host aggregate that holds one inherits their alignment
// and gains tail padding. That is the intended cost of single-sourcing the
// layout, and it costs nothing here -- this struct is a description, never
// uploaded, never hashed and never snapshotted. Same disable, same reason, as
// state/layout.hpp's WorldParams.
#pragma warning(disable : 4324)
#endif
struct WorldInstanceDesc {
    WorldDesc world;                   // geometry, spawn points, environment, capacities
    uint64_t seed = 0;                 // this instance's rng root -> WorldParams::seed
    DrydenParams turbulence{};         // per-world turbulence config (see above)
    physics::ContactParams contacts{}; // per-world material/solver record
    physics::GridParams grid{};        // broad-phase cell size (see the note on uniformity below)
};
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

// ---------------------------------------------------------------------------
// The world set itself. A plain vector: world INDEX is world IDENTITY
// everywhere downstream (the arena's world partitions, WorldParams' row index,
// the slot->world map's values), so the order of this vector is a contract, not
// a detail.
// ---------------------------------------------------------------------------
struct WorldSetDesc {
    std::vector<WorldInstanceDesc> worlds;
};

// ---------------------------------------------------------------------------
// The training-fleet constructor (spec §4): N copies of one world instance,
// each with its own rng root derived from a single scene seed.
//
// The derivation is core/rng.hpp's, verbatim -- splitmix64(scene_seed ^
// fnv1a64("world") ^ index) -- which is the errata-R4 discipline applied one
// level up: the domain tag keeps world 0's root from colliding with the bare
// scene seed, and every per-world system then derives ITS stream from the world
// root under its own tag. Adding a fifth world therefore does not perturb the
// first four.
//
// `count` == 0 yields an empty set, which validate_world_set() rejects; that is
// deliberate (an empty fleet is a caller bug, not a degenerate case worth
// supporting).
// ---------------------------------------------------------------------------
[[nodiscard]] WorldSetDesc replicate(const WorldInstanceDesc& prototype, uint32_t count,
                                     uint64_t scene_seed);

// The rng domain tag replicate() derives world roots under. Pinned: changing it
// re-seeds every world in every replicated set, and therefore every digest in
// the determinism corpus.
inline constexpr std::string_view kWorldSeedDomainTag = "world";

// ---------------------------------------------------------------------------
// C5: "N worlds from this world file, capacities X" -- the training-fleet
// constructor, spelled over a WorldRef (world/world_ref.hpp) so a caller can
// name the world by FILE PATH or hand over an already-built WorldDesc.
//
// Resolves `ref` EXACTLY ONCE (a path is loaded and validated a single time,
// never once per replicated instance), stamps the resolved WorldDesc into a
// COPY of `instance_prototype`, and delegates to replicate() above so every
// world's rng root derives by the ONE formula that function owns -- this
// function must never re-derive or duplicate replicate()'s expression.
//
// THE CAPACITIES-OVERRIDE SPLIT, stated because it is observable: stamping
// the resolved WorldDesc onto the prototype replaces the prototype's ENTIRE
// `.world` field -- geometry, spawns, environment, and `capacities` all come
// from the FILE (or the handed-over desc), never from instance_prototype. The
// prototype's OTHER fields -- `turbulence`, `contacts`, `grid` -- are left
// exactly as the caller set them, because those are RUN parameters, not world
// geometry (see WorldInstanceDesc's own note above on why they ride
// separately from WorldDesc): the FILE owns the world, the PROTOTYPE owns
// instance physics.
//
// Codes: invalid_argument if `count == 0` -- checked here, before resolving,
// so a zero-count caller never pays for a file read or parse it is about to
// discard (replicate() itself has no way to reject this: it returns a plain
// WorldSetDesc, not a Result, so the empty-fleet-is-a-caller-bug judgment it
// documents for `count == 0` has to be enforced one level up, here);
// otherwise resolve_world()'s error, verbatim (code and context, including
// any file path it already carries).
// ---------------------------------------------------------------------------
[[nodiscard]] Result<WorldSetDesc> world_set_from(const WorldRef& ref, uint32_t count,
                                                  uint64_t scene_seed,
                                                  const WorldInstanceDesc& instance_prototype);

// ---------------------------------------------------------------------------
// The allocation shape a validated world set implies -- what Simulation sizes
// its arenas from.
//
// CAPACITIES ARE UNIFORMIZED TO THE MAXIMUM, and that needs stating because it
// is observable. ArenaSet gives every world the SAME per-world capacity for a
// given array (one global allocation, world-contiguous partitions of equal
// size), so a set whose worlds declare different capacities gets partitions
// sized by the largest. Each world still ENFORCES ITS OWN declared capacity at
// spawn time -- a world that asked for 4 bodies is refused a 5th even though
// its partition has room -- so the declared number remains the honest contract.
//
// The consequence, stated once here and referenced by the digest helpers: a
// world's snapshot/digest FOOTPRINT is a function of the SET's maximum
// capacity, not of that world's own. Comparing one world's digest across two
// differently-shaped sets is therefore only meaningful when the two sets agree
// on these maxima -- which they do for the uniform sets replicate() builds, and
// which the batching-invariance test relies on.
// ---------------------------------------------------------------------------
struct WorldSetLayout {
    uint32_t world_count = 0;
    uint32_t body_capacity = 0;     // per-world body slots: max over the set
    uint32_t element_capacity = 0;  // per-world force-element slots: max over the set --
                                     // the SHARED rotor+drag budget since Task 18 (see
                                     // sim/simulation.cpp: rotors and drag bodies both
                                     // size off this one field, not one each)
    uint32_t sensor_capacity = 0;   // per-world sensor slots: max over the set

    // NOTE that the sensor capacity costs more storage than the others: each
    // sensor also owns sensors::kRingDepth output samples in the ring array
    // (sensors/rings.hpp), so a world's ring partition is
    // sensor_capacity * kRingDepth elements. validate_world_set() range-checks
    // that product against the uint32 slot index like every other array.

    // True iff every world's ContactParams AND GridParams are byte-identical.
    //
    // This is what lets the CollisionDynamic pass take its BATCHED form -- one
    // sorted-grid sweep over every world's bodies at once, which is the shape
    // D8 exists for and the one S6 mirrors. resolve_dynamic_contacts() takes a
    // SINGLE ContactParams/GridParams pair for the whole call, so a set whose
    // worlds disagree about restitution or cell size cannot be swept in one
    // call without silently applying one world's material to another's bodies.
    // Simulation therefore falls back to one sweep per world in that case.
    //
    // Both forms produce byte-identical per-world results (the grid keys on
    // world id and pairs never cross worlds, so a world's sorted sub-sequence
    // -- and hence its Gauss-Seidel sweep order -- is the same either way), so
    // this flag is a performance/shape decision, never a numerical one. It is
    // derived from CONFIG at create() and never from state, so it cannot
    // introduce a data-dependent branch into the step loop.
    bool uniform_dynamic_params = false;
};

// ---------------------------------------------------------------------------
// Validates a world set and derives its allocation shape.
//
// Codes:
//   invalid_argument  -- empty set; a world whose SdfProgram does not validate
//                        (reported verbatim from SdfProgram::validate); a zero
//                        capacity; a non-finite environment; a contact, grid or
//                        turbulence parameter outside its documented domain.
//   capacity_exceeded -- the set's world count times a per-world capacity does
//                        not fit a uint32 (ArenaSet's addressing limit), or the
//                        SDF program is deeper than kMaxSdfDepth.
//
// The parameter-domain checks are deliberately stricter than the physics passes
// themselves, which document their preconditions and do not check them (they
// are the inner loop). THIS is the layer that owns the checking, so that a
// misconfigured world is an error at create() rather than a NaN forty substeps
// later.
// ---------------------------------------------------------------------------
[[nodiscard]] Result<WorldSetLayout> validate_world_set(const WorldSetDesc& desc);

// ---------------------------------------------------------------------------
// THE CONFIG IDENTITY OF A WORLD SET: FNV-1a 64 over a canonical serialization
// of `desc` (ticket M-1).
//
// WHY IT EXISTS. A snapshot blob carries STATE, never CONFIGURATION -- the SDF
// programs, the material and grid records, the turbulence parameters and the
// environment all come from the WorldSetDesc the receiving Simulation was
// created with (see WorldInstanceDesc's DrydenParams note above, and
// Simulation::restore). Until this hash existed, restoring a blob into a
// Simulation built from a DIFFERENT desc was a silent, undetectable way to
// replay the same state forward under different physics: the snapshot schema
// hash covers array names, element sizes and extents, and every one of those is
// identical between two sets that disagree only about restitution. This number
// is what closes that hole -- it rides in the registered `replay_config` row
// (sim/simulation.hpp), so it is in every blob, and Simulation::restore()
// compares it before writing a byte.
//
// ---------------------------------------------------------------------------
// THE FOLD ORDER IS A CONTRACT. Changing it changes every hash, which
// invalidates every recorded blob's config check and moves every golden digest
// in the determinism corpus. It is pinned here, in full, so that a future
// reader can re-derive a hash by hand from this list:
//
//   seed = kFnv1a64Offset                          (state/snapshot.hpp's basis)
//   fold u64  world count
//   fold u32  max over the set of Capacities::bodies
//   fold u32  max over the set of Capacities::force_elements
//   fold u32  max over the set of Capacities::sensors
//   fold u32  max over the set of Capacities::contacts
//   for each world, IN INDEX ORDER (world index is world identity):
//       fold u64   world.name.size(), then the name bytes
//       fold u64   world.sdf.nodes.size(),      then the nodes' raw bytes
//       fold u64   world.sdf.transforms.size(), then the transforms' raw bytes
//       fold, in Environment's DECLARATION order:
//            f32 gravity.x, gravity.y, gravity.z
//            f32 wind.x, wind.y, wind.z
//            f32 air_density
//            f32 temperature_k
//            u64 environment.seed
//       fold u32   capacities.bodies, force_elements, sensors, contacts
//       fold u64   instance seed (WorldInstanceDesc::seed)
//       fold       DrydenParams raw bytes  (32)
//       fold       ContactParams raw bytes (32)
//       fold       GridParams raw bytes    (16)
//
// EVERY VARIABLE-LENGTH RUN IS LENGTH-PREFIXED -- the name and both SDF vectors
// -- for the reason state/snapshot.hpp's schema hash gives for its own names:
// without the prefix, two different (count, content) splits can present the
// same byte stream, and a hash that cannot tell them apart is a hash with a
// designed-in collision.
//
// THE THREE PARAMETER RECORDS FOLD AS RAW BYTES, which is only sound because
// each one static_asserts that every byte of it belongs to a NAMED field (see
// the batteries in world/medium.hpp, physics/contacts.hpp and physics/grid.hpp,
// and the re-assertion at this function's definition). Implicit padding is the
// one thing a byte-wise fold cannot reason about; none of the three has any.
// The same argument, and the same re-assertion, covers SdfNode and
// SdfTransform: both carry EXPLICIT `_pad` lanes that are named fields with
// zero initializers, so their byte images are fully determined by their fields.
//
// WHAT IT DELIBERATELY DOES NOT COVER, and why:
//
//   * SPAWN POINTS. They are authoring anchors a caller reads to place a body;
//     the placed body's state is in the snapshot. Two descs differing only in
//     their spawn tables replay a restored blob identically, so covering them
//     would reject pairings that are in fact sound.
//   * dt_ns AND substeps. They are not properties of the world set at all --
//     they are Simulation::create()'s other two arguments, and they ride in
//     `replay_config` as their own fields, checked separately and reported by
//     name.
//   * VISUAL_REFS (WorldDesc, T5). RENDER-ONLY -- physics never reads them, no
//     pass, no digest, no snapshot touches them (see WorldDesc's own comment
//     on the field). Two descs differing only in which meshes/materials a
//     presentation layer resolves for the same physical world replay a
//     restored blob identically, so covering them here would reject pairings
//     that are in fact sound, the same argument as spawn points above.
//
// AND ONE THING IT COVERS THAT THE ENGINE IGNORES, stated so the strictness is
// not mistaken for a bug: Environment::seed is the world FILE's authoring
// default, which Simulation deliberately ignores in favour of
// WorldInstanceDesc::seed (see the SEED note above). Folding it makes this hash
// STRICTER than the physics requires -- two descs differing only there would
// replay identically but hash differently. That direction is the safe one: a
// false rejection costs a caller an explicit error, a false acceptance costs
// them a silently wrong replay.
//
// IT IS THE HASH OF A DESC, NOT OF LIVE STATE. Simulation::reseed() rewrites
// every world's WorldParams::seed row without touching the desc it was created
// from, so a reseeded Simulation's `replay_config` still carries the hash of
// its CREATING desc -- which is exactly right: the new seeds are STATE, they
// ride in the blob, and restoring that blob into a twin built from the original
// desc must (and does) pass this check and then adopt the new seeds.
//
// NOT CRYPTOGRAPHIC. Nothing here defends against a crafted collision, for the
// same reason state/snapshot.hpp's schema hash does not: the adversary is a
// tired engineer pairing the wrong blob with the wrong world set, not an
// attacker.
// ---------------------------------------------------------------------------
[[nodiscard]] uint64_t config_hash(const WorldSetDesc& desc) noexcept;

}  // namespace spade
