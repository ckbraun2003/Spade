#pragma once

// ---------------------------------------------------------------------------
// solids_chain.hpp (plan B, task B5) -- the GPU solids projection's host
// side, for the chain runner: the solid field packed as bindings 13-17 hold
// it, the words of airflow_params B5 writes (word 16's bits, word 28's pcg
// block), the table of where the CG's vectors and scalars live, and the
// dispatches of B5's reductions. The CPU twins, and the authority, are
// engine/physics/airflow/projection.cpp's project() with solids and pcg.cpp's
// solve_pcg_to().
//
// ONE HOME (TD-9; the lead, 2026-10-22 and 2026-10-26): pack_solid_field()
// is the one host packer of a SolidField (B3's test-local copy retires with
// it), and PcgVector and the kSlot* constants are the host's twin of
// engine/shaders/airflow_pcg.slang's tables. Every host placement and read of
// a CG vector or slot goes through them.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "compute/vulkan/dispatch_chain.hpp"
#include "core/error.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/regions.hpp"
#include "physics/airflow/solids.hpp"
#include "physics/airflow/wall_load.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/airflow/projection_chain.hpp"

namespace spade::airflow_gpu {

// --- airflow_params' words (airflow_params_layout.slang) ----------------------

inline constexpr uint32_t kParamsFlags = 16;       // the flags word
inline constexpr uint32_t kParamsFlagSolids = 1;   // bit 0: has_solids()
inline constexpr uint32_t kParamsFlagTheta = 2;    // bit 1: has_apertures()
inline constexpr uint32_t kParamsPcgBlock = 28;    // off_pcg_block: the pcg block's offset, in words
inline constexpr uint32_t kPcgBlockWords = 6;      // {closed regions, announce, gain, stall iterations, floor, cap}

// --- airflow_pcg and airflow_scalars (airflow_pcg.slang's tables) -------------

// airflow_pcg's vectors, each the grid's cells, vector v at v * cells.
enum PcgVector : uint32_t {
    kPcgX = 0,     // the unknown
    kPcgB = 1,     // the right side
    kPcgR = 2,     // the residual
    kPcgZ = 3,     // the preconditioned residual
    kPcgD = 4,     // the search direction
    kPcgQ = 5,     // A d, or A x when measuring
    kPcgBest = 6,  // the x that measured least (P2d)
};
inline constexpr uint32_t kPcgVectors = 7;  // pcg_bytes()' seven floats a cell

// Where vector v starts in airflow_pcg, in floats.
[[nodiscard]] uint64_t pcg_offset(PcgVector v, const physics::airflow::GridShape& shape) noexcept;

// airflow_scalars' slots: 0 is remove_mean()'s (projection_chain.hpp's
// kMeanSlot); B5's are 1-31.
inline constexpr uint32_t kSlotRz = 1;        // r . z
inline constexpr uint32_t kSlotDq = 2;        // d . q
inline constexpr uint32_t kSlotRzNext = 3;    // r . z after an iteration's preconditioner
inline constexpr uint32_t kSlotAlpha = 4;     // rz / dq
inline constexpr uint32_t kSlotBeta = 5;      // rz_next / rz
inline constexpr uint32_t kSlotRMax = 6;      // max |b - q| over the cells in a region
inline constexpr uint32_t kSlotBMax = 7;      // max |b| over the same cells
inline constexpr uint32_t kSlotResidual = 8;  // b_max > 0 ? r_max / b_max : 0
inline constexpr uint32_t kScalarSlots = 32;  // airflow_scalars' floats: slot 0 and B5's

// --- the solid field (bindings 13-17) -----------------------------------------

// A SolidField as the device holds it, by the rule Physics confirmed to
// Rendering on 2026-10-22:
//   apertures, wall_velocity, phi: theta, the wall velocity and phi, one float
//     per face, u then v then w, component c's faces from aperture_base(c),
//     each in mac_grid.hpp's index;
//   face_tags: words 0 .. F - 1 (F = F_u + F_v + F_w, every face) the face's
//     wall-link index, link[c][idx] or kNoWallLinks; then the free-slip
//     layer, one byte a face, little-endian, four a word, face f in byte
//     f % 4 of word F + f / 4;
//   wall_links: 12 words a link, inv_f[0..5] then wall[0..5], as floats' bits.
// An array with nothing in it (no wall link) is empty: its binding is bound
// as a placeholder and never read.
struct PackedSolidField {
    std::vector<float> apertures;      // binding 13, airflow_apertures
    std::vector<float> wall_velocity;  // binding 14, airflow_wall_velocity
    std::vector<float> phi;            // binding 15, airflow_phi
    std::vector<uint32_t> face_tags;   // binding 16, airflow_face_tags
    std::vector<uint32_t> wall_links;  // binding 17, airflow_wall_links
    bool has_solids = false;           // SolidField::any: has_solids() and has_apertures()
};

inline constexpr uint32_t kWallLinkWords = 12;

// Where component c's faces start in apertures, wall_velocity, phi and
// face_tags: 0 for u, F_u for v, F_u + F_v for w (airflow_pcg.slang's
// aperture_base()).
[[nodiscard]] uint64_t aperture_base(const physics::airflow::GridShape& shape, uint32_t c) noexcept;

// The packing above. A host call: it allocates.
[[nodiscard]] PackedSolidField pack_solid_field(const physics::airflow::SolidField& f);

// airflow_params word 16's solid bits: has_solids() and has_apertures() iff
// the field has a part (SolidField::any).
[[nodiscard]] uint32_t solids_flags(const PackedSolidField& packed) noexcept;

// Writes word 28 (= offset) and the pcg block at `offset`: the closed-region
// count, then pcg.hpp's kResidualAnnounce, kStallGain and kStallIterations,
// then the solve's floor and cap. `params` holds offset + kPcgBlockWords.
void write_pcg_block(std::span<uint32_t> params, uint32_t offset, uint32_t closed_regions, uint32_t floor,
                     uint32_t cap) noexcept;

// --- the reductions' dispatches -----------------------------------------------

// B5's kernels' pipelines in a runner (RunnerDesc::kernels); a chain names
// only those it dispatches.
struct SolidsPipelines {
    uint32_t tile_sum = 0;  // airflow_tile_sum: the dots' passes 1 and 2
    uint32_t tile_dot = 0;
    uint32_t measure = 0;
    uint32_t scalars = 0;
};

// airflow_pcg_scalars' ops.
enum class ScalarsOp : uint32_t { alpha = 0, beta = 1, residual = 2 };

// fixed_tile_dot(a, b) into airflow_scalars[slot]: airflow_tile_dot (pass 0),
// then airflow_tile_sum's pass 1 and pass 2 with reduce.hpp's
// kReduceBlockTiles. airflow_partials holds a float a tile.
void append_tile_dot(std::vector<compute::ChainCommand>& out, const physics::airflow::GridShape& shape,
                     const SolidsPipelines& p, PcgVector a, PcgVector b, uint32_t slot, uint32_t workgroup_size);

// The max-norm's r_max and b_max over b and q into kSlotRMax and kSlotBMax:
// airflow_pcg_measure's three passes, with `block` tiles a block (the
// solver's is kReduceBlockTiles; any block gives the same bits).
// airflow_partials holds two floats a tile.
void append_measure(std::vector<compute::ChainCommand>& out, const physics::airflow::GridShape& shape,
                    const SolidsPipelines& p, uint32_t workgroup_size, uint32_t block);

// One airflow_pcg_scalars op.
[[nodiscard]] compute::ChainCommand scalars_dispatch(const SolidsPipelines& p, ScalarsOp op);

// ===========================================================================
// B5.2b (Physics, 2026-10-27; a block of its own, before B5.3's and B5.5's):
// the symmetric V(2,2) preconditioner on the device -- the full-weighting
// restriction (airflow_mg_restrict_full) and multigrid.cpp's cycle_sym() as a
// chain over B2's smoother, residual and prolongation -- and init_solids(),
// which puts a grid's solids on the device for a runner (the GPU step's host
// side, B7).
// ===========================================================================

// --- the preconditioner -------------------------------------------------------

// The preconditioner's pipelines: B2's smoother, residual and prolongation
// (mg's other members are not dispatched), and the full weighting.
struct PreconditionPipelines {
    ProjectionPipelines mg;
    uint32_t restrict_full = 0;  // airflow_mg_restrict_full
};

// multigrid.cpp's restrict_full_weighting() of level `level` onto level + 1,
// with the coarse p set to 0 (cycle_sym()'s std::fill): one invocation per
// coarse cell, push {level, 0, coarse cells, 0}.
[[nodiscard]] compute::ChainCommand restrict_full_dispatch(const MgLayout& layout, const PreconditionPipelines& p,
                                                           uint32_t level, uint32_t workgroup_size);

// One symmetric V(2,2) cycle from level `level` down, multigrid.cpp's
// cycle_sym() dispatch for dispatch: kPreSweeps sweeps red first, the
// residual, the full weighting, the coarser cycle, the prolongation, then
// kPostSweeps sweeps BLACK first; on the coarsest level kCoarsestSymSweeps
// sweeps red first, then as many black first. From level 0 it is
// precondition()'s cycle: the input is level 0's b (airflow_rhs; the CG's r)
// with level 0's p (airflow_pressure) at 0, as precondition() sets it, and
// the output level 0's p. Every coarser level's p, b and r, and level 0's r,
// are written before they are read. The face weights are the levels'
// (init_solids()).
void append_symmetric_v_cycle(std::vector<compute::ChainCommand>& out, const MgLayout& layout,
                              const PreconditionPipelines& p, uint32_t level, uint32_t workgroup_size);

// --- init_solids(): a grid's solids on the device -----------------------------

// What init_solids() puts on the device, from the CPU's own scratch after
// set_solids() or attach_solid_field() (fluid_step.hpp): the field and every
// level's face weights are the CPU's bits, never recomputed on the device.
struct SolidsUpload {
    PackedSolidField field;        // bindings 13-17, pack_solid_field()
    MgLayout layout;               // mg_layout() after set_weights(): the level table for airflow_params
                                   // (write_projection_params()), the weights for airflow_mg_weights (12)
    std::vector<uint32_t> labels;  // airflow_region_labels (18): the solid cells, the CG's mask
    // B5.3 (the closed regions) adds the region table, airflow_region_table (19), here.
};

// The upload for the scratch's solids. Refused as mg_layout() refuses. A
// host call: it allocates.
[[nodiscard]] Result<SolidsUpload> solids_upload(const physics::airflow::FluidScratch& s);

// The bytes airflow-set binding `binding` needs for the upload, at least 4
// (an empty array binds as a placeholder); 0 for a binding init_solids() does
// not write.
[[nodiscard]] uint64_t solids_binding_bytes(const SolidsUpload& u, uint32_t binding) noexcept;

// Puts the upload on `runner`, whose buffers are the airflow set in binding
// order (kAirflowSetBuffers, binding_base 0), those it writes sized by
// solids_binding_bytes() or more. Into `params`, which holds the projection's
// words written from u.layout (write_projection_params()) and the other
// blocks, it writes word 16's solid bits and the pcg block at `pcg_block`
// (write_pcg_block(): the closed-region count, B5.3's, then pcg.hpp's
// constants, the floor and the cap); then it uploads airflow_params whole and
// bindings 12 to 18. Refused as ChainRunner::upload() refuses.
[[nodiscard]] Result<void> init_solids(ChainRunner& runner, std::span<uint32_t> params, const SolidsUpload& u,
                                       uint32_t pcg_block, uint32_t closed_regions, uint32_t floor, uint32_t cap);

}  // namespace spade::airflow_gpu

// ===========================================================================
// B5.3 AND B5.5 (Physics, 2026-10-27; appended as a block of its own): the
// closed regions' means (the CG's gauge), what a closed face holds, and the
// wall load. Their kernels are airflow_region_mean, airflow_solid_velocities
// and airflow_wall_load; their CPU twins regions.cpp's remove_region_means(),
// solids.cpp's apply_solid_velocities() and wall_load.cpp's wall_load(). The
// table constants below are airflow_pcg.slang's of the same names (TD-9).
// ===========================================================================

namespace spade::airflow_gpu {

// --- the region table (airflow_region_table, binding 19) ----------------------

// Closed region j's record, kRegionWords words from kRegionWords * j; after
// the C records, a word per region id: its closed index, or kNoRegion when it
// is pinned.
inline constexpr uint32_t kRegionWords = 4;
inline constexpr uint32_t kRegionId = 0;          // its region id
inline constexpr uint32_t kRegionCells = 1;       // its cells
inline constexpr uint32_t kRegionReciprocal = 2;  // the host's 1 / cells (float bits), remove_region_means()'
inline constexpr uint32_t kRegionMean = 3;        // its mean (float bits), written by the device

struct RegionTable {
    std::vector<uint32_t> words;  // binding 19's words
    uint32_t closed = 0;          // C: the pcg block's closed-region count (write_pcg_block())
};

// The table for the regions as the last check_regions() left them: each
// closed region in id order, then the id-to-index words. A host call: it
// allocates.
[[nodiscard]] RegionTable pack_region_table(const physics::airflow::FluidRegions& r);

// The floats airflow_partials needs for the means of `closed` regions: a sum
// and a met flag per closed region a tile.
[[nodiscard]] uint64_t region_partials_floats(const physics::airflow::GridShape& shape, uint32_t closed) noexcept;

// Each closed region's mean of airflow_pcg's vector v into the table:
// airflow_region_mean's passes 0-2, with reduce.hpp's kReduceBlockTiles.
// `closed` is the table's C; with none, nothing is appended.
void append_region_means(std::vector<compute::ChainCommand>& out, const physics::airflow::GridShape& shape,
                         uint32_t pipeline, PcgVector v, uint32_t closed, uint32_t workgroup_size);

// remove_region_means(v), the CG's gauge (projection.cpp's gauge_regions()):
// the means, then pass 3, each closed region's cells less its mean.
void append_remove_region_means(std::vector<compute::ChainCommand>& out, const physics::airflow::GridShape& shape,
                                uint32_t pipeline, PcgVector v, uint32_t closed, uint32_t workgroup_size);

// --- the interior --------------------------------------------------------------

// apply_solid_velocities(g, f, mode) on airflow_u, _v and _w: airflow_solid_
// velocities' pass 0 (rigid), or passes 1 and 2 (extrapolate: layer 2 after
// layer 1), each over every face.
void append_solid_velocities(std::vector<compute::ChainCommand>& out, const physics::airflow::GridShape& shape,
                             uint32_t pipeline, physics::airflow::SolidInterior mode, uint32_t workgroup_size);

// --- the wall load (airflow_wall_load, binding 27) -------------------------------

inline constexpr uint32_t kWallLoadForce = 0;   // F's x, y, z
inline constexpr uint32_t kWallLoadMoment = 3;  // M's x, y, z
inline constexpr uint32_t kWallLoadAbout = 6;   // the point M is taken about: the host's input
inline constexpr uint32_t kWallLoadFloats = 9;
inline constexpr uint32_t kWallLoadTerms = 6;   // airflow_partials' floats a tile

// Binding 27's floats before the chain: the load 0, the point `about` (the
// grid's frame, as wall_load() takes it).
[[nodiscard]] std::array<float, kWallLoadFloats> wall_load_words(glm::vec3 about) noexcept;

// The load in binding 27's floats after the chain.
[[nodiscard]] physics::airflow::WallLoad wall_load_of(std::span<const float, kWallLoadFloats> words) noexcept;

// wall_load(g, f, regions, about) from airflow_pressure into binding 27:
// airflow_wall_load's three passes, with reduce.hpp's kReduceBlockTiles.
void append_wall_load(std::vector<compute::ChainCommand>& out, const physics::airflow::GridShape& shape,
                      uint32_t pipeline, uint32_t workgroup_size);

}  // namespace spade::airflow_gpu
