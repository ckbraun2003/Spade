#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "physics/contacts.hpp"
#include "physics/grid.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

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

}  // namespace spade
