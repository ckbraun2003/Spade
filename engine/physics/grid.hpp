#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

#include <glm/glm.hpp>

#include "physics/contacts.hpp"  // ContactParams (shared material record)
#include "state/arenas.hpp"      // kInvalidWorld
#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// The dynamic_contact.resolve pass (engine design spec D3 "sorted-grid for
// dynamic-dynamic", D8 "the collision grid hashes (world_id, cell); sorted
// domains are global; range checks make cross-world interaction structurally
// impossible", section 5 "Collision"). Every active body is a SPHERE PROXY of
// the shared ContactParams::proxy_radius, and every overlapping pair gets one
// mass-weighted impulse (restitution + Coulomb friction) plus a Baumgarte-style
// positional split.
//
// WHERE THIS SITS IN THE SCHEDULE. The module schedule's six phases, per substep:
//
//     Fields -> Forces -> Constraints/Contacts (static, then dynamic) ->
//     Integrate -> Sensors -> Publish (sim/standard_modules.cpp)
//
// so this runs AFTER resolve_static_contacts() and BEFORE integrate_bodies(),
// on the velocity the previous substep's Integrate produced. Running after the
// static pass is what makes a body pinched between the floor and another body
// settle rather than be pushed back into the floor: the last velocity write
// before Integrate is the body-body one, and the next substep's static pass
// sees (and re-corrects) whatever penetration that left.
//
// ---------------------------------------------------------------------------
// THE V1 PORT, AND THE FOUR THINGS THAT CHANGED
//
// This is a CPU port of v1's atomics-free sorted-grid pipeline -- "the engine's
// best idea" in the design spec's words -- kept in the same five-stage shape so
// the S6 GPU mirror is a transliteration rather than a redesign:
//
//     1. BUILD    one key per active body: (world_id, cell), cell =
//                 int3(floor(pos / cell_size)).            [v1: GridBuild.comp]
//     2. SORT     by a TOTAL order on (world, z, y, x, slot).
//                                                       [v1: BitonicSort.comp]
//     3. OFFSETS  one run record per distinct (world, cell), covering that
//                 cell's contiguous span of the sorted array.
//                                                      [v1: GridOffsets.comp]
//     4. RESOLVE  per body, gather the 27 cells of its own + neighbouring
//                 cells and pair-test the candidates.  [v1: GridCollision.comp]
//     (v1's GridReorder/GridScatter stages have no CPU analogue: they exist to
//     make the GPU's memory access coalesced by permuting the state arrays into
//     sorted order and back. On the CPU the entry's `slot` indexes the
//     unpermuted array directly, which is also why nothing here has to worry
//     about the sorted/unsorted index confusion that made v1's GridCollision
//     read `entityBounds` through a two-level indirection.)
//
// FIX 1 -- EXACT CELL COMPARE, NOT HASH-BUCKET EQUALITY. This is the fixed v1
// bug, and it is worth being precise about what the bug WAS, because "distant
// cells collide" undersells it. v1 keyed the sorted array on
//
//     GetHash(cell) = ((x*73856093) ^ (y*19349663) ^ (z*83492791)) % 2^21
//                                        [assets/shaders/[SYSTEM]GridBuild.comp:29-36]
//
// and its neighbour scan terminated a run with
//
//     if (gridPairs[k].cellID != neighborHash) break;
//                                    [assets/shaders/[SYSTEM]GridCollision.comp:157]
//
// -- a comparison of BUCKET INDICES. Two consequences, one merely wasteful and
// one an actual wrong answer:
//
//   * false candidates. Bodies from an unrelated far-away cell that happens to
//     hash into the same bucket land in the same run and get pair-tested. The
//     sphere-sphere distance test then rejects them, so this costs work but not
//     correctness.
//   * DOUBLE RESOLUTION -- the real defect. The scan visits 27 neighbour CELLS;
//     under bucket equality two DISTINCT neighbours of the same cell can map to
//     the same bucket, and then the run behind that bucket is scanned TWICE, so
//     every genuine contact inside it is resolved twice in one substep. Such a
//     pair exists among cells a test can write down: the neighbours (-1,-1,1)
//     and (-1,1,-1) of cell (0,0,0) both hash to bucket 1592181. Doubling the
//     positional correction (and, at other restitutions, the impulse) is a
//     silent physics error, not a performance note.
//
// Here the key IS the cell -- world id and all three signed cell integers,
// compared exactly, never hashed -- so both consequences are structurally
// impossible rather than merely unlikely. test_grid.cpp reconstructs v1's hash
// from the shader above and pins both cases.
//
// FIX 2 -- THE "CONFIGURABLE TABLE SIZE" KNOB IS GONE, NOT MOVED. The design
// spec asks for v1's `1 << 21` (spelled FOUR separate times in
// src/Core/Engine.cpp: lines 197, 279, 340, 399) to become "hash size
// configurable per world set". In this sorted-key formulation THERE IS NO HASH
// TABLE: sorting the exact keys is what groups a cell's bodies contiguously, so
// the array is exactly as large as the live population and there is no bucket
// count to size, no load factor to tune, and no way to under-size it. The knob
// became moot rather than configurable -- GridParams carries the one geometric
// parameter that survives (`cell_size`) and nothing else. THIS IS THE
// FORMULATION S6 MIRRORS: a GPU bitonic sort over the same keys reproduces it
// directly, and it is precisely the property that a configurable table size
// would have preserved only statistically.
//
// FIX 3 -- BOUNDS-SAFE TAIL, IN BOTH SENSES. v1 padded its power-of-two sort
// buffer with 0xFFFFFFFF sentinels and then relied on scattered `if (index >=
// numInstances) return;` guards to keep the tail from being interpreted as a
// body. There is no padding here -- std::sort takes a size, not a power of two
// -- so the class of bug does not exist. The other bounds hazard v1 answered
// badly is out-of-range positions: it CLAMPED every cell into
// [0, gridDim) (GridBuild.comp:50), which collapses every far-away body into
// the boundary cells and makes them all mutual neighbours -- a second aliasing
// source on top of the hash. Here cells are signed and unbounded, and a body
// whose cell coordinate would not fit an int32 (or whose position is NaN or
// infinite) is SKIPPED rather than clamped: it takes no part in dynamic
// collision at all, which is the honest answer for a body 2e9 cells from the
// origin. See grid_cell_of().
//
// FIX 4 -- MOMENTUM CONSERVATION BY CONSTRUCTION. v1 accumulated every contact
// a body found and then applied the AVERAGE (GridCollision.comp:229-236), which
// is not a physical operation: the two halves of one pair see different
// neighbour counts, so they receive impulses that are not equal and opposite
// and the total momentum of a closed cloud drifts. Here each pair is resolved
// ONCE, in place, with equal-and-opposite applications (a Gauss-Seidel sweep in
// sorted order), so momentum is conserved to fp32 rounding -- exactly when the
// two masses are equal. That is the property the 1k-sphere shower test asserts.
// v1's `if (abs(velAlongNormal) < 0.5) restitution = 0.0;` resting hack is also
// dropped: it is a magic constant with no physical reading, and the approach
// guard plus the slop band already do its job.
//
// ---------------------------------------------------------------------------
// PURE FUNCTION OF (state, params) -- WITH ONE NAMED EXCEPTION. Like the
// integrator and the static-contact pass, this holds no state between calls,
// reads no clock, draws no randomness, and visits bodies in a fully determined
// order, so two runs on equal inputs produce byte-equal outputs (test_grid.cpp
// pins that with memcmp). The exception is `scratch`: the key and run arrays
// have to live somewhere, and threading them through the caller is what keeps
// the steady state allocation-free (see GridScratch). Bodies without
// body_flags::active are left byte-for-byte untouched, and never appear as
// either half of a pair.
//
// PARITY. The numbered op order in grid.cpp is the CPU<->GPU parity contract
// (P1/P2, D11), and for this pass the SWEEP ORDER is part of it: the response
// is Gauss-Seidel (each pair sees the velocities the earlier pairs left), so
// the sorted entry order, the fixed dz/dy/dx neighbour order, and the
// lower-slot-resolves rule together determine the answer. A GPU mirror that
// resolves pairs concurrently is a DIFFERENT (Jacobi) solver and will not
// reproduce these numbers; making S6 bit-identical means reproducing this
// sweep, which the sorted key array is exactly the right structure for.
//
// ---------------------------------------------------------------------------
// SCOPE, STATED AS LOUDLY AS contacts.hpp STATES ITS OWN
//
//  1. THE RESPONSE IS LINEAR-ONLY, exactly as in contacts.hpp: impulses change
//     `vel` and nothing else, `omega_body`/`torque_acc` are never touched, and
//     no contact torque r x J is generated. Two spheres exchanging a glancing
//     blow do not start spinning. Revisited at the VEHICLE LAYER (D4), which is
//     what introduces per-body contact geometry.
//  2. ONE PROXY RADIUS FOR THE WHOLE SPAN, BY DEFAULT -- WITH A PER-BODY
//     OVERRIDE (D-S6-2). ContactParams::proxy_radius is the same record the
//     static pass uses -- the two passes cannot disagree about a world's
//     DEFAULT -- but each body's own BodyState::proxy_radius (state/
//     layout.hpp) now overrides it when nonzero, exactly as contacts.cpp's
//     effective_proxy_radius() reads it. Contact distance for a pair is
//     therefore the SUM of the two bodies' own effective radii, not a single
//     shared diameter -- see resolve_pair() in grid.cpp for where that sum is
//     computed.
//  3. NO CONTINUOUS COLLISION DETECTION and no manifold: the test samples
//     current positions only. Same envelope as the static pass, halved by
//     relative motion -- |v_rel| * h < 2 * proxy_radius.
//  4. NO SLEEPING, NO ISLANDS, NO CAPSULES. Deliberately out of scope; a
//     capsule proxy is D4's business and islands/sleeping only pay off at a
//     body count this pass is not aimed at.
// ---------------------------------------------------------------------------

namespace spade::physics {

// ---------------------------------------------------------------------------
// The grid's one geometric parameter. PER WORLD (WorldConfig::grid,
// sim/simulation.hpp; the S6 device mirror uploads one GridParams per world
// at bindings.slang binding 20, exactly like ContactParams at binding 19) --
// a heterogeneous world set may legitimately want different cell sizes for
// different worlds' geometry scales, the same reasoning ContactParams already
// documents for restitution. It is a separate record from ContactParams
// rather than four more lanes inside it because the two vary independently:
// a world's material and its broad-phase cell size are unrelated knobs.
//
// WHY THIS STRUCT IS NOT IN state/layout.hpp, AND WHY IT CARRIES LAYOUT.HPP'S
// ASSERT BATTERY ANYWAY: the coordinator ruling recorded on ContactParams
// applies unchanged -- layout.hpp holds memory-resident, snapshot-walked engine
// STATE; PASS-PARAMETER VALUE STRUCTS stay in their pass's header but carry the
// full layout discipline here so the S6 Slang mirror has something pinned to
// mirror. GRADUATION PATH, likewise unchanged: when the broad phase becomes a
// GPU dispatch this becomes a uniform/param-buffer row, moves to layout.hpp or
// the Slang shared module, and these asserts move with it.
//
// THE LAYOUT: four floats in one 16-byte std430 row.
//
//   row 0  cell_size | _r0 | _r1 | _r2
//
// alignas(16) gives std430's base alignment for a struct in an array, and 16
// bytes is already a whole row, so sizeof IS the array stride and no implicit
// tail padding exists (asserted below both ways). The three reserved lanes are
// named, default-initialized, and reserved for versioned growth -- a
// per-axis cell size, or a neighbourhood radius above 1, are the obvious next
// fields -- so growth consumes them without moving `cell_size` or changing the
// stride.
//
// UNITS: cell_size is metres.
//
// PRECONDITIONS, not validated at runtime (this is the physics inner loop; the
// world author or the config layer owns them):
//
//   * cell_size > 0. Neither degenerate case is diagnosed, and they fail
//     differently, so both are stated. A ZERO cell_size divides every position
//     to +-inf (or, at the origin, NaN), which grid_cell_of() rejects, so the
//     pass degrades to "no contacts" rather than to UB -- that is
//     bounds-safety, not a supported configuration. A NEGATIVE cell_size still
//     divides to finite values, so nothing rejects it; it partitions space into
//     slabs of width |cell_size| indexed in reverse, which happens to leave the
//     neighbour relation intact. That is an accident of the arithmetic and is
//     not tested or supported.
//   * cell_size >= 2 * ContactParams::proxy_radius. THIS ONE IS A FOOTGUN AND
//     DESERVES ITS OWN PARAGRAPH. The resolve step gathers only the 27 cells
//     around a body, so it can only find partners whose cell differs by at most
//     one per axis -- which covers everything within `cell_size` of the body.
//     A contact distance LARGER than one cell is therefore silently MISSED for
//     any pair that happens to straddle two cells with a gap. Nothing here
//     detects that; test_grid.cpp pins the envelope
//     (CellSizeBelowContactDiameterMissesContacts) so the limit is recorded
//     behaviour rather than folklore. The default is deliberately 1 m against a
//     defaulted proxy_radius of 0, which satisfies it. D-S6-2 WIDENS THE
//     HAZARD, NOT ITS KIND: a per-body override (BodyState::proxy_radius) can
//     make one PAIR's contact_dist exceed cell_size even when
//     ContactParams::proxy_radius alone would not. THE INVARIANT IS ENFORCED
//     AT BOTH AUTHORING SITES, not left as pure folklore: sim/world_set.cpp's
//     validate_grid_against_contacts() checks cell_size against the world's
//     DEFAULT (ContactParams::proxy_radius) when a world is built, and
//     Simulation::spawn(world, ModelTypeId, VehicleSpawn) (sim/simulation.cpp)
//     checks it AGAIN against the spawning MODEL's own proxy_radius when that
//     model overrides the default -- the gap the world-level check cannot
//     see, since it runs before any vehicle model exists to register. Each
//     check independently guarantees `2 * r <= cell_size` for the body it
//     admits (r = the world default or the model's own override,
//     respectively), i.e. `r <= cell_size / 2` for EVERY body in the world at
//     the moment it is spawned -- which is what closes this for PAIRS too,
//     algebraically, without a separate per-pair check: for any two bodies a
//     and b, ra <= cell_size/2 and rb <= cell_size/2 together give
//     contact_dist = ra + rb <= cell_size, so two individually-compliant
//     bodies cannot produce a pair whose contact_dist exceeds the cell. Config
//     (config.grid.cell_size, config.contacts.proxy_radius) is fixed for a
//     world's lifetime at create() and there is no reconfigure API, so neither
//     check needs to be, or is, revisited after the fact.
//
// The default describes a valid but inert grid: with ContactParams' defaulted
// zero proxy radius no pair is ever in contact, so a default-constructed pair
// of records cannot silently inject energy into a world whose author forgot to
// configure it. Every lane has an initializer, so the 16-byte image of a
// defaulted record is fully determined and the record is hashable and
// uploadable byte-wise.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) GridParams {
    // Broad-phase cell edge length, metres. Must be >= 2 * proxy_radius (see
    // above). Larger costs candidates quadratically in the per-cell population;
    // smaller costs nothing but silently loses contacts once it drops below the
    // contact diameter. "Exactly the contact diameter" is the cheapest correct
    // setting and the one the tests use.
    float cell_size = 1.0f;

    // Reserved for versioned growth; must stay 0. Named rather than left as
    // implicit tail padding so that EVERY byte of this record belongs to a
    // field -- the property the "named bytes" assert below pins, and the one
    // thing a byte-wise comparison or hash of the record cannot reason about
    // otherwise.
    float _r0 = 0.0f;
    float _r1 = 0.0f;
    float _r2 = 0.0f;
};

// ---------------------------------------------------------------------------
// The layout battery, in state/layout.hpp's style and for its stated reason:
// these asserts are the enforcement, not decoration.
// ---------------------------------------------------------------------------
static_assert(std::is_standard_layout_v<GridParams>,
              "GridParams must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<GridParams>,
              "GridParams must be memcpy-able: it uploads to a device buffer verbatim at S6");
static_assert(std::is_trivially_destructible_v<GridParams>,
              "GridParams is a value record, never individually destroyed");
static_assert(alignof(GridParams) == 16, "std430 base alignment");
static_assert(sizeof(GridParams) == 16, "std430 array stride (one 16-byte row, no tail pad)");

static_assert(offsetof(GridParams, cell_size) == 0);
static_assert(offsetof(GridParams, _r0) == 4);
static_assert(offsetof(GridParams, _r1) == 8);
static_assert(offsetof(GridParams, _r2) == 12);

// The row starts on a 16-byte boundary -- trivially true for an all-scalar
// struct, asserted anyway so that adding a vec3 lane later cannot silently
// straddle a row the way std430 forbids.
static_assert(offsetof(GridParams, cell_size) % 16 == 0);

// Named fields account for every byte: no IMPLICIT padding anywhere in
// GridParams.
static_assert(sizeof(GridParams::cell_size) + sizeof(GridParams::_r0) + sizeof(GridParams::_r1) +
                  sizeof(GridParams::_r2) ==
                  sizeof(GridParams),
              "GridParams has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// A broad-phase cell coordinate: floor(pos / cell_size), component-wise.
//
// SIGNED AND UNBOUNDED, unlike v1's clamped [0, gridDim) index -- see FIX 3 in
// the file header. The origin cell is (0,0,0) and there is no global-bounds
// offset to agree on, which is one fewer parameter for the CPU and GPU paths to
// disagree about.
// ---------------------------------------------------------------------------
struct GridCell {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    friend constexpr bool operator==(const GridCell&, const GridCell&) noexcept = default;
};

// ---------------------------------------------------------------------------
// One sorted key: which world, which cell, which body. v1's GridPair
// {cellID, instanceID} with the hash replaced by the exact cell and the world
// id added.
//
// `slot` is an index into the `bodies` span handed to
// resolve_dynamic_contacts(). When that span is the whole bodies array -- the
// normal all-worlds call -- it is also the global arena slot, which is what
// makes it a stable, unique final tiebreak for the sort.
// ---------------------------------------------------------------------------
struct GridEntry {
    uint32_t world = kInvalidWorld;
    GridCell cell{};
    uint32_t slot = 0;
};

// ---------------------------------------------------------------------------
// One run of sorted entries sharing an exact (world, cell) key: v1's gridHead
// offset, made explicit and exact. `begin`/`count` index GridScratch::entries.
//
// Runs appear in the same order as the entries they cover, so the run array is
// itself sorted by the key comparator -- which is what lets a neighbour lookup
// be a binary search that ends in an EXACT key comparison.
// ---------------------------------------------------------------------------
struct GridCellRun {
    uint32_t world = kInvalidWorld;
    GridCell cell{};
    uint32_t begin = 0;
    uint32_t count = 0;
};

// ---------------------------------------------------------------------------
// The pass's working storage, owned by the caller.
//
// WHY IT IS A PARAMETER RATHER THAN A LOCAL. The build/sort/offset stages need
// O(live bodies) storage; allocating it per substep would put a heap round-trip
// at 1 kHz in the middle of the hot loop. Threading it through the caller means
// the vectors reach their high-water capacity during the first few substeps and
// never allocate again: resolve_dynamic_contacts() clear()s them, which keeps
// capacity, and only push_back past capacity allocates. Call reserve() once at
// world load to skip even the warmup.
//
// THE CONSEQUENCE, STATED HONESTLY: resolve_dynamic_contacts() is noexcept and
// can allocate, so an out-of-memory during warmup terminates rather than
// unwinding. That is the same posture as the rest of the engine (no exceptions
// across module boundaries) and the reason reserve() exists.
//
// THE CONTENTS ARE OBSERVABLE AFTER THE CALL and are deliberately public: they
// are the pass's entire intermediate state, and being able to look at the exact
// keys is what lets test_grid.cpp assert the no-aliasing property structurally
// instead of inferring it from behaviour. They carry no meaning ACROSS calls --
// every call rebuilds both arrays from scratch -- so a caller may share one
// scratch between several worlds' passes, or keep one per thread.
//
// NOT REGISTERED STATE. Nothing here is snapshot-walked: it is a pure function
// of the body positions at the moment of the call, so a restored snapshot
// rebuilds it on the next substep. That is why it lives in a plain std::vector
// rather than an arena.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// The five fields resolve_dynamic_contacts_jacobi() reads off a body, split
// out so that pos/vel come from the START-OF-ITERATION SNAPSHOT while mass and
// the proxy radius do not need to be snapshotted at all.
//
// THAT SPLIT IS EXACT RATHER THAN AN OPTIMIZATION, and it is the reason this
// record is two vectors' worth of floats and not a second BodyState array: the
// dynamic_contact.resolve pass writes `pos` and `vel` AND NOTHING ELSE, so `mass`,
// `proxy_radius` and `flags` cannot change during it and a shadow copy of them
// would be a copy of something already immutable. The GPU mirror therefore
// needs a shadow buffer of two float3s per body rather than a second
// 128-byte BodyState row (shaders/shared/layouts.slang).
//
// `radius` is ALREADY effective_proxy_radius() -- the 0 sentinel is resolved
// once per body when the snapshot is taken, instead of twice per pair
// examination as the Gauss-Seidel sweep does it.
// ---------------------------------------------------------------------------
// FIELD ORDER IS THE std430 ROW'S, NOT A READING ORDER. `mass` sits between
// `pos` and `vel` because a float3 aligns to 16 bytes in std430, so the fourth
// lane after `pos` exists whether or not anything occupies it -- putting a
// scalar there is free, and putting it anywhere else costs 16 bytes of pad.
// The layout is therefore
//
//     pos 0..11   mass 12..15   vel 16..27   radius 28..31      32 bytes
//
// which is exactly shaders/shared/layouts.slang's GatherBodyRow, and the
// generated static_assert in layout_check.gen.hpp is what holds the two to it
// -- gen_layout_check.py fails the build for a mirror it cannot check.
// BodyState uses the same trick one row up (`float3 pos; float proxy_radius;`).
//
// alignas IS NOT DECORATION AND I DID NOT PREDICT NEEDING IT. The offsets came
// out right first time; the ALIGNMENT did not. glm::vec3 is three floats and
// aligns to 4, so this struct's natural alignment is 4 while std430 requires
// 16 for any struct containing a vector -- the size was already 32, so nothing
// about reading the fields would have looked wrong, and an array of these
// would have been laid out at a stride the device does not agree with. The
// generated check named it exactly: "C++ alignment does not satisfy the Slang
// std430 alignment (16)". Same alignas every other mirrored row carries.
struct alignas(kStd430StructAlignment) GatherBody {
    glm::vec3 pos{0.0f};
    float mass = 0.0f;
    glm::vec3 vel{0.0f};
    float radius = 0.0f;
};

struct GridScratch {
    // One entry per ACTIVE, in-range body. Sorted by grid_entry_less() when the
    // call returns.
    std::vector<GridEntry> entries;

    // One run per distinct (world, cell) present in `entries`, in the same
    // order.
    std::vector<GridCellRun> runs;

    // JACOBI ONLY -- one row per ENTRY (not per body, and not per slot),
    // holding the start-of-iteration pos/vel plus the two scalars the pair
    // math needs. Indexed by entry index, which is what lets the inner loop
    // reach a candidate as snapshot[k] with no indirection through `slot`, and
    // what makes the self-exclusion a single `k == i` compare.
    //
    // LEFT EMPTY BY resolve_dynamic_contacts(): the Gauss-Seidel path neither
    // fills nor reads it, so its cost is unchanged by this field's existence.
    // Same clear()-keeps-capacity discipline as the two arrays above.
    std::vector<GatherBody> snapshot;

    // Sizes all three arrays' capacity for `body_count` bodies -- the worst
    // case for `runs` too, since every body could occupy its own cell.
    // Idempotent and never shrinks (std::vector::reserve semantics).
    void reserve(std::size_t body_count) {
        entries.reserve(body_count);
        runs.reserve(body_count);
        snapshot.reserve(body_count);
    }
};

// ---------------------------------------------------------------------------
// The largest cell coordinate magnitude this pass will produce.
//
// Two jobs, which is why it is 2e9 rather than INT32_MAX:
//
//   1. It keeps the float -> int32 conversion in grid_cell_of() DEFINED. A
//      float outside int32's range converts to an unspecified value (UB before
//      C++20's saturating rules, still not what anyone wants), so the range is
//      checked before the cast rather than after.
//   2. It leaves headroom for the +-1 neighbour offsets the resolve step adds:
//      2e9 + 1 is still comfortably inside int32, so the neighbour loop needs
//      no overflow guard of its own.
//
// 2e9 is exactly representable in fp32 (15625000 * 2^7), so the comparison
// against it has no rounding subtlety. A body beyond it is 2e9 cells from the
// origin; skipping it is the honest answer (see FIX 3 in the file header).
// ---------------------------------------------------------------------------
inline constexpr float kMaxCellCoord = 2.0e9f;

// ---------------------------------------------------------------------------
// pos -> cell, the BUILD stage's per-body kernel. Returns false and leaves
// `out` untouched when the position has no representable cell: a coordinate
// beyond kMaxCellCoord, an infinity, or a NaN (including the +-inf/NaN a
// non-positive cell_size produces).
//
// The range test is spelled `!(lo <= v && v <= hi)` so a NaN FAILS it rather
// than comparing false on both sides and falling through into the cast.
//
// Exposed rather than file-static because it is the definition of "which cell",
// and test_grid.cpp needs to state cell membership in the same terms the pass
// does -- notably in the v1-hash tests, which must place bodies in named cells.
// bool-plus-out-parameter rather than std::optional because that is the shape
// the S6 shader mirror takes.
// ---------------------------------------------------------------------------
[[nodiscard]] bool grid_cell_of(const glm::vec3& pos, float cell_size, GridCell& out) noexcept;

// ---------------------------------------------------------------------------
// The SORT stage's comparator: a strict TOTAL order on
// (world, cell.z, cell.y, cell.x, slot).
//
// WHY THIS ORDER. The z/y/x nesting makes cells that are adjacent along x
// contiguous in the sorted array, which mirrors the row-major cell
// linearization a GPU grid uses, so the S6 port sorts the same sequence into
// the same layout. `slot` last is what makes the order TOTAL rather than merely
// weak: slots are unique within a call, so no two entries ever compare
// equivalent, the sorted permutation is UNIQUE, and std::sort's unspecified
// choice of algorithm (and its unstable partitioning) cannot affect the result.
// That is the whole determinism argument for this pass -- there is nothing left
// for an implementation to choose.
//
// Strict weak ordering is immediate: it is lexicographic over a fixed-length
// tuple of totally-ordered scalars, hence irreflexive, transitive, and
// asymmetric, with incomparability (equality of all five fields) trivially
// transitive.
//
// Exposed so tests can pin the ordering itself rather than only its
// consequences.
// ---------------------------------------------------------------------------
[[nodiscard]] bool grid_entry_less(const GridEntry& a, const GridEntry& b) noexcept;

// ---------------------------------------------------------------------------
// Resolves every overlapping pair of ACTIVE bodies in `bodies`.
//
// CONTACT TEST: for bodies a and b, d = pos_b - pos_a; contact_dist =
// effective_proxy_radius(a, params.proxy_radius) + effective_proxy_radius(b,
// params.proxy_radius) (D-S6-2: each body's own override, or the world's
// default -- physics/contacts.hpp); a contact exists iff 0 < |d| < contact_
// dist, with depth = contact_dist - |d| and normal n = d / |d| pointing from a
// to b. Coincident bodies (|d| == 0) have no defined normal and are skipped,
// exactly as the static pass skips a zero SDF gradient.
//
// THE TWO SPANS ARE PARALLEL: index i of `bodies` and index i of
// `slot_to_world` describe the same slot. `slot_to_world` is
// ArenaSet::slot_to_world(bodies_array) verbatim.
//
// PASS THE WHOLE ARRAY, NOT A WORLD SLICE. This is the one pass that is
// deliberately all-worlds: keying on world id is what makes a single sweep
// cover N worlds with cross-world interaction structurally impossible (D8), so
// the natural call site hands it every slot of every world at once. Passing a
// single world's slice (with the matching subspan of the map) also works and
// costs nothing -- every entry simply shares one world id -- but batching N
// worlds into one call is the shape this exists for, and the one whose result
// is asserted to be independent of the batching.
//
// SKIPPED BODIES, and there are three kinds, all left byte-for-byte untouched:
//   * flags without body_flags::active -- the same inert-slot contract the
//     integrator and the static pass use, and the reason a zero-filled arena
//     slot costs one bit test;
//   * slot_to_world == kInvalidWorld -- a freed slot. Redundant with the flag
//     test for anything ArenaSet produced (it zero-fills on free), and kept
//     because a body the liveness map says is gone must not participate even if
//     something left its flag set;
//   * a position with no representable cell -- see grid_cell_of().
// If the two spans differ in length only their common prefix participates; the
// tail is bounds-safe rather than undefined.
//
// `params` is the SAME ContactParams record the static pass takes, so the two
// cannot disagree about a world's DEFAULT size or its material. `proxy_radius`
// is now a per-body FALLBACK rather than the pair's whole answer -- see the
// CONTACT TEST above and effective_proxy_radius() -- while `restitution_e`,
// `friction_mu`, `baumgarte_beta` and `slop` are unchanged, per-world values;
// the reserved lanes are not read.
//
// NO `h` PARAMETER. Unlike resolve_static_contacts(), which keeps an unused `h`
// so its shape matches the uniform pass signature, this one already departs
// from that shape by taking `scratch`, and no term of the pinned model is
// dimensionally dependent on dt. Adding it back would be decoration.
//
// COMPLEXITY: O(n log n) for the sort plus O(sum over cells of (27-neighbour
// population) * (cell population)) for the resolve -- linear in n at a bounded
// density, quadratic in the population of a single overloaded cell. There is no
// per-cell candidate cap: dropping contacts to bound worst-case cost is a
// policy this pass deliberately does not have.
// ---------------------------------------------------------------------------
void resolve_dynamic_contacts(std::span<BodyState> bodies, std::span<const uint32_t> slot_to_world,
                              const GridParams& grid, const ContactParams& params,
                              GridScratch& scratch, std::span<glm::vec3> contact_dv = {}) noexcept;

// ---------------------------------------------------------------------------
// THE SAME PASS AS A JACOBI GATHER -- one iteration, no in-place coupling.
//
// NOTHING IN THE ENGINE CALLS THIS YET. It is built, tested and measured
// alongside resolve_dynamic_contacts() rather than replacing it, because
// switching solvers moves every pinned number in the dynamic-collision corpus
// and that is a decision with its own gate. Today its only callers are tests.
//
// WHAT IS THE SAME: the broad phase, exactly -- build, sort and offsets are
// the SHARED code both functions call, so the two see byte-identical
// `entries` and `runs` for the same input. The per-pair arithmetic is
// resolve_pair()'s, op for op, grouping for grouping.
//
// WHAT IS DIFFERENT, AND IT IS ONE THING: every body computes its contacts
// from the START-OF-ITERATION state and writes only itself. So
//
//   * THE RESULT DOES NOT DEPEND ON THE ORDER BODIES ARE VISITED IN. That is
//     the whole point: on the GPU the outer loop becomes the thread index, and
//     the answer must not depend on how many lanes are running. The
//     Gauss-Seidel sweep cannot offer this -- its answer IS its sweep order.
//   * EACH PAIR IS COMPUTED TWICE, once in each endpoint's gather, and the
//     `ea.slot < eb.slot` rule that made the sweep visit it once is GONE
//     rather than relaxed. A gather must see all of a body's partners, not
//     only its higher-slot ones.
//   * IT IS A DIFFERENT SOLVER AND GIVES DIFFERENT NUMBERS on anything with a
//     body in two simultaneous contacts. With ONE pair in flight the two agree
//     BIT FOR BIT -- there is no earlier pair for Gauss-Seidel to have seen --
//     and the tests pin exactly that, because it separates an arithmetic
//     mistake from the intended change in coupling.
//
// MOMENTUM, STATED PRECISELY RATHER THAN CLAIMED. Per pair the two bodies'
// mass-weighted changes still cancel identically (ma*w_a == mb*w_b == m_eff),
// and the approach guard fires identically from both sides because v_rel_n is
// sign-identical under the exchange. What does NOT hold exactly is the sum:
// the pair's two halves land in two different fp32 accumulations, in different
// orders, so the cancellation is to those sums' rounding. THAT IS NOT v1's
// DEFECT (FIX 4 above), which was an AVERAGE over each body's own neighbour
// count -- a systematic, O(1) error that grew with contact count. This one is
// O(eps) and unbiased, the same class as the sweep's own accumulation residue.
//
// CONVERGENCE IS NOT CLAIMED HERE. One Jacobi iteration propagates a contact
// one body per substep where a Gauss-Seidel sweep can carry it through a whole
// stack in one pass, so deep stacks are softer under this function until the
// iteration count is raised. No number for that appears in this comment because
// none has been measured.
// ---------------------------------------------------------------------------
void resolve_dynamic_contacts_jacobi(std::span<BodyState> bodies,
                                     std::span<const uint32_t> slot_to_world,
                                     const GridParams& grid, const ContactParams& params,
                                     GridScratch& scratch) noexcept;

}  // namespace spade::physics
