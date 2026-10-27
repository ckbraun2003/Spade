// ---------------------------------------------------------------------------
// airflow_pcg.slang (plan B, task B5; Physics, 2026-10-26) -- WHERE B5's
// STATE LIVES on the GPU: pcg.hpp's PcgScratch vectors in airflow_pcg (set 1,
// binding 20), B5's scalars in airflow_scalars (binding 21), and the solid
// field's faces as spike/airflow/solids_chain.hpp's pack_solid_field() packs
// them (bindings 13-15). Every B5 kernel reads them through this module, never
// by an index of its own (TD-9).
//
// THE HOST'S TWIN of the two tables below is solids_chain.hpp's PcgVector and
// kSlot* constants, from which the host places and reads every vector and
// slot; B5's tests do the same, so a table here that differs from the host's
// reads the wrong data and goes red.
//
// airflow_pcg: seven vectors of the grid's cells (nx ny nz, airflow_params'
// dims), each in cell order (i innermost), vector v at v * cells:
//    0  x     the unknown: the grid's p in, the solution out
//    1  b     the right side
//    2  r     the residual
//    3  z     the preconditioned residual
//    4  d     the search direction
//    5  q     A d, and A x when the residual is measured
//    6  best  the x that measured least past the floor (P2d)
// Seven floats a cell, as pcg_bytes() counts.
//
// airflow_scalars: slot 0 is remove_mean()'s sum (projection_chain.hpp's
// kMeanSlot); slots 1-31 are B5's. APPEND-ONLY, as airflow_params' words:
//    1  rz        r . z (a tile dot)
//    2  dq        d . q (a tile dot)
//    3  rz_next   r . z after an iteration's preconditioner (a tile dot)
//    4  alpha     rz / dq            (airflow_pcg_scalars, div32_ftz)
//    5  beta      rz_next / rz       (airflow_pcg_scalars, div32_ftz)
//    6  r_max     max |b - q| over the cells in a region (airflow_pcg_measure)
//    7  b_max     max |b| over the same cells (airflow_pcg_measure)
//    8  residual  b_max > 0 ? r_max / b_max : 0 (airflow_pcg_scalars, div32_ftz)
//    9-31 free (B5.4's decisions)
//
// THE SOLID FIELD (bindings 13-15, pack_solid_field()'s rule, confirmed by
// Physics to Rendering on 2026-10-22): theta, the wall velocity and phi as
// floats per face, u then v then w, component c's faces from aperture_base(c),
// each in mac_grid.hpp's u, v or w index. (face_tags and wall_links, bindings
// 16 and 17, are the viscous solve's; airflow_viscous.slang reads them.)
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

// --- airflow_pcg's vectors -----------------------------------------------------

static const uint kPcgX = 0u;
static const uint kPcgB = 1u;
static const uint kPcgR = 2u;
static const uint kPcgZ = 3u;
static const uint kPcgD = 4u;
static const uint kPcgQ = 5u;
static const uint kPcgBest = 6u;
static const uint kPcgVectors = 7u;

uint pcg_cells() {
    const uint3 n = grid_dims();
    return n.x * n.y * n.z;
}

// Where vector v starts in airflow_pcg, in floats.
uint pcg_offset(uint v) { return v * pcg_cells(); }

float pcg_get(uint v, uint c) { return airflow_pcg[pcg_offset(v) + c]; }

void pcg_set(uint v, uint c, float value) { airflow_pcg[pcg_offset(v) + c] = value; }

// --- airflow_scalars' slots ----------------------------------------------------

static const uint kSlotRz = 1u;
static const uint kSlotDq = 2u;
static const uint kSlotRzNext = 3u;
static const uint kSlotAlpha = 4u;
static const uint kSlotBeta = 5u;
static const uint kSlotRMax = 6u;
static const uint kSlotBMax = 7u;
static const uint kSlotResidual = 8u;

// --- the regions ---------------------------------------------------------------

// regions.hpp's kNoRegion: airflow_region_labels' word for a solid cell, in no
// fluid region (the CG's mask).
static const uint kNoRegion = 0xFFFFFFFFu;

bool solid_cell(uint c) { return airflow_region_labels[c] == kNoRegion; }

// --- the solid field -------------------------------------------------------------

// Where component c's faces start in airflow_apertures, airflow_wall_velocity
// and airflow_phi (u, then v, then w).
uint aperture_base(uint c) {
    const uint3 n = grid_dims();
    const uint faces_u = (n.x + 1u) * n.y * n.z;
    const uint faces_v = n.x * (n.y + 1u) * n.z;
    return c == 0u ? 0u : (c == 1u ? faces_u : faces_u + faces_v);
}

float face_theta(uint c, uint idx) { return airflow_apertures[aperture_base(c) + idx]; }

float face_wall(uint c, uint idx) { return airflow_wall_velocity[aperture_base(c) + idx]; }

// solids.hpp's closed(): theta exactly 0.
bool face_closed(uint c, uint idx) { return face_theta(c, idx) == 0.0; }
