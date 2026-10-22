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
//   27-31  free
//   32-    the level table: level l at 32 + 12 l,
//          {nx, ny, nz, off_p, off_b, off_r, off_w0, off_w1, off_w2, weighted, 0, 0}
//
// APPEND-ONLY. A word, once given a meaning, keeps it: words 0-16 are already
// coded against (Rendering's B3 and B6). A new scalar takes a free word in
// 27-31; past those, the table moves only by a new layout with its own name.
//
// THE FLOAT WORDS ARE THE HOST'S (TD-9): inv_dx is 1.0f / dx, and b_coef,
// g_coef and inv_count are the CPU's own expressions (projection.cpp's b_coef
// and g_coef, remove_mean's inv_count), computed once on the host and uploaded.
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
