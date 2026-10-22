#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"

// ---------------------------------------------------------------------------
// FLUID REGIONS (the solids design, section 3; the lead's finding 3). A sheet
// or a solid can cut the fluid into separate regions, each closed one with
// its own null space:
//   * a cell is fluid when any of its six faces is open (theta > 0; a domain
//     face counts), and in no region (solid) otherwise;
//   * regions are the connected sets of fluid cells through the open faces
//     between them, across a periodic seam too, labelled by a flood fill from
//     each unlabelled fluid cell in array order, faces -x, +x, -y, +y, -z, +z:
//     deterministic;
//   * each step, a region an open pressure-outlet face reaches is pinned; any
//     other is closed, keeps its own mean (remove_region_means), and must take
//     no net prescribed flux through the domain faces it touches -- walls and
//     inflows, theta-weighted, summed in double over their samples, within
//     1e-6 of their magnitudes (check_domain()'s rule, per region). Immersed
//     rigid walls need no check: a rigid motion's flux through any closed
//     surface of cell faces is exactly 0 (the face-centre rule is exact for a
//     linear field), and the gauge takes the rounding.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

inline constexpr uint32_t kNoRegion = std::numeric_limits<uint32_t>::max();

struct FluidRegions {
    std::vector<uint32_t> label;   // per cell: its region, or kNoRegion (solid)
    std::vector<uint8_t> solid;    // per cell: 1 where label is kNoRegion (CG's mask)
    uint32_t count = 0;
    std::vector<uint32_t> cells;   // per region: its cell count
    std::vector<uint32_t> first;   // per region: its first cell, array order
    std::vector<uint8_t> closed;   // per region, this step: no open outlet reaches it
    std::vector<uint8_t> faces;    // per region, this step: a bit per domain side with a prescribed flux into it
    std::vector<double> flux;      // per region, this step: the net outward prescribed flux, m^3/s
    std::vector<double> magnitude;
    std::vector<float> total;      // per region: the masked sums, then the means
    std::vector<float> block;      // per region: the current block's partial sum
    std::vector<uint8_t> in_block; // per region: met in the current block
    std::vector<float> tile;       // per region: one tile's partial sum
    std::vector<uint8_t> mark;     // per region: met in the current tile
    std::vector<uint32_t> touched; // the regions the current tile met, in first-met order
};

// The bytes label_regions() allocates for the shape: the label and solid
// mask per cell, the flood fill's queue (a cell index per cell, held while it
// labels) and the tile list. Not counted: the per-region arrays, which grow
// with the region count.
[[nodiscard]] uint64_t regions_bytes(const GridShape& s) noexcept;

// Labels the fluid cells (a host call: it allocates).
void label_regions(FluidRegions& r, const GridShape& s, std::array<bool, 3> periodic, const SolidField& f);

struct RegionFault {
    uint32_t region = kNoRegion;  // kNoRegion: every closed region balances
    uint32_t cell = 0;            // the region's first cell
    uint8_t faces = 0;            // a bit per domain side whose prescribed flux enters it
};

// This step's status, allocation-free: closed[], and the first closed region
// whose prescribed flux does not balance.
[[nodiscard]] RegionFault check_regions(FluidRegions& r, const GridShape& s, const DomainBc& bc,
                                        const SolidField& f) noexcept;

// x -= each closed region's mean over its own cells. A region's sum is
// reduce.hpp's two-level order restricted to its cells: each tile's cells of
// that region in (k, j, i) order from 0; each block of kReduceBlockTiles
// consecutive tiles (the grid's blocks, whatever the region) adds the tile
// partials it has of the region from 0 in tile order; the block sums the
// region has are added from 0 in block order; all in fp32. A tile or block
// without the region's cells adds nothing to it. The mean is the sum times
// the host's reciprocal of its cell count, subtracted in array order. Pinned
// regions are left alone.
void remove_region_means(std::span<float> x, FluidRegions& r, const GridShape& s) noexcept;

// "fluid regions: N (K with an outlet, M closed)", from the last check_regions().
[[nodiscard]] std::string describe_regions(const FluidRegions& r);

}  // namespace spade::physics::airflow
