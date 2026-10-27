// ---------------------------------------------------------------------------
// airflow_params_layout.slang -- THE LAYOUT OF airflow_params (set 1, binding
// 0; bindings.slang section F), frozen for every airflow kernel (plan B, B1;
// Physics, 2026-10-21). A kernel imports this module and reads the params
// through the accessors below, never by a word index of its own.
//
// uint32 words. A float word holds the float's bits (asfloat).
//
//    0- 3  dims            {nx, ny, nz, 0}
//    4- 7  grid            {dx, inv_dx, h, 0}                      floats
//    8-11  periodic        {px, py, pz, 0}                         0 or 1 per axis
//   12-15  offsets_advect  {hat, lo, hi, faces_total}              in floats, into airflow_advect
//   16     flags           bit 0 has solids, bit 1 has theta (airflow_apertures)
//   17     the multigrid's level count
//   18-23  face kinds      -x, +x, -y, +y, -z, +z                  FaceKind values (below)
//   24     b_coef          rho * dx * dx / h                       float
//   25     g_coef          h / (rho * dx)                          float
//   26     inv_count       remove_mean's 1 / count                 float
//   27     off_face_block  in words, into airflow_params: the face block below
//   28     off_pcg_block   in words, into airflow_params: the pcg block below
//   29-31  free
//   32-    the level table: level l at 32 + 12 l,
//          {nx, ny, nz, off_p, off_b, off_r, off_w0, off_w1, off_w2, weighted, 0, 0}
//   then   the face block, at word 27's offset, 22 floats (B3's viscosity and
//          velocity conditions; Rendering, 2026-10-22, Physics confirmed):
//          {alpha, face_velocity[3 side + c] for side -x, +x, -y, +y, -z, +z
//          and c = x, y, z, ambient[c]}. alpha is diffuse()'s nu h / (dx dx);
//          a side's face velocity is its wall's wall_velocity for a wall and
//          the ambient otherwise (an inflow's ghost; an outlet's is unread),
//          so fixed_normal_velocity(side) is face_velocity(side, axis).
//   then   the pcg block, at word 28's offset, after the face block, 6 words
//          (B5's solids CG; Physics, 2026-10-26): {the closed-region count,
//          kResidualAnnounce (float), kStallGain (float), kStallIterations,
//          the floor, the cap}, the constants from physics/airflow/pcg.hpp and
//          the solve's floor and cap as solve_pcg_to() takes them, all
//          written by the host (spike/airflow/solids_chain.hpp's
//          write_pcg_block()). kResidualAnnounce is the solve's mark, as
//          projection.cpp passes it to solve_pcg_to().
//
// AND airflow_scalars' slots (binding 21; B5 owns 1-31; Physics, 2026-10-27),
// their table airflow_pcg.slang's: 0 remove_mean()'s sum; 1-3 the CG's dots
// (rz, dq, rz_next); 4-5 alpha and beta; 6-7 r_max and b_max; 8 the residual;
// 9-23 B5.4's decisions (pcg.cpp's solve_pcg_to()), with 9-12 the 16-byte
// decision word the host reads {stop, restart, iterations, stalled}, then
// residual_at, least, since, restarted, best, best_at, measured, exact,
// keep_best, take_best and capped; 24-31 free.
//
// APPEND-ONLY. A word, once given a meaning, keeps it: words 0-16 are already
// coded against (Rendering's B3 and B6). A new scalar takes a free word in
// 29-31; past those, the table moves only by a new layout with its own name.
//
// THE FLOAT WORDS ARE THE HOST'S (TD-9): inv_dx is 1.0f / dx, and b_coef,
// g_coef, inv_count and alpha are the CPU's own expressions (projection.cpp's
// b_coef and g_coef, remove_mean's inv_count, viscous.cpp's alpha, computed
// under the step's flush as the CPU computes it), computed once on the host
// and uploaded.
// No kernel recomputes them, so a kernel and its CPU twin multiply by the same
// bits.
//
// THE PUSH CONVENTION that goes with it: an airflow kernel's push block is 16
// bytes, four uint words, and word 2 is always the count of the elements the
// dispatch covers (faces or cells), the one a dispatch's guard compares with.
// ---------------------------------------------------------------------------

import bindings;

// FaceKind (physics/airflow/boundaries.hpp), as words 18-23 hold it.
static const uint kFaceFixedVelocity = 0u;
static const uint kFacePressureOutlet = 1u;
static const uint kFacePeriodic = 2u;

// Word 16's bits.
static const uint kParamsFlagSolids = 1u;
static const uint kParamsFlagTheta = 2u;

// The level table.
static const uint kParamsLevelBase = 32u;
static const uint kParamsLevelWords = 12u;

// --- words 0-15 --------------------------------------------------------------

uint3 grid_dims() { return uint3(airflow_params[0], airflow_params[1], airflow_params[2]); }
float grid_dx() { return asfloat(airflow_params[4]); }
float grid_inv_dx() { return asfloat(airflow_params[5]); }
float step_h() { return asfloat(airflow_params[6]); }
bool axis_periodic(uint a) { return airflow_params[8 + a] != 0u; }
uint offset_hat() { return airflow_params[12]; }
uint offset_lo() { return airflow_params[13]; }
uint offset_hi() { return airflow_params[14]; }
uint advect_faces_total() { return airflow_params[15]; }

// --- words 16-26 -------------------------------------------------------------

uint params_flags() { return airflow_params[16]; }
bool has_solids() { return (airflow_params[16] & kParamsFlagSolids) != 0u; }
bool has_apertures() { return (airflow_params[16] & kParamsFlagTheta) != 0u; }
uint mg_level_count() { return airflow_params[17]; }
// side: 0 -x, 1 +x, 2 -y, 3 +y, 4 -z, 5 +z.
uint face_kind(uint side) { return airflow_params[18 + side]; }
float b_coef() { return asfloat(airflow_params[24]); }
float g_coef() { return asfloat(airflow_params[25]); }
float mean_inv_count() { return asfloat(airflow_params[26]); }

// --- word 27 and the face block ------------------------------------------------

uint offset_face_block() { return airflow_params[27]; }
float viscous_alpha() { return asfloat(airflow_params[airflow_params[27]]); }
// side: 0 -x, 1 +x, 2 -y, 3 +y, 4 -z, 5 +z; c: 0 x, 1 y, 2 z.
float face_velocity(uint side, uint c) { return asfloat(airflow_params[airflow_params[27] + 1u + 3u * side + c]); }
float ambient_velocity(uint c) { return asfloat(airflow_params[airflow_params[27] + 19u + c]); }

// --- word 28 and the pcg block --------------------------------------------------

uint offset_pcg_block() { return airflow_params[28]; }
uint pcg_closed_regions() { return airflow_params[airflow_params[28]]; }
float pcg_residual_announce() { return asfloat(airflow_params[airflow_params[28] + 1u]); }
float pcg_stall_gain() { return asfloat(airflow_params[airflow_params[28] + 2u]); }
uint pcg_stall_iterations() { return airflow_params[airflow_params[28] + 3u]; }
uint pcg_floor() { return airflow_params[airflow_params[28] + 4u]; }
uint pcg_cap() { return airflow_params[airflow_params[28] + 5u]; }

// --- the level table ---------------------------------------------------------

struct MgLevel {
    uint3 dims;
    uint off_p;
    uint off_b;
    uint off_r;
    uint3 off_w;    // the three axes' face weights
    bool weighted;
};

MgLevel mg_level(uint l) {
    const uint base = kParamsLevelBase + kParamsLevelWords * l;
    MgLevel out;
    out.dims = uint3(airflow_params[base + 0u], airflow_params[base + 1u], airflow_params[base + 2u]);
    out.off_p = airflow_params[base + 3u];
    out.off_b = airflow_params[base + 4u];
    out.off_r = airflow_params[base + 5u];
    out.off_w = uint3(airflow_params[base + 6u], airflow_params[base + 7u], airflow_params[base + 8u]);
    out.weighted = airflow_params[base + 9u] != 0u;
    return out;
}
