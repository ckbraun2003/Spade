// ---------------------------------------------------------------------------
// lbm_lattice.slang -- the LBM tier's D3Q27 lattice on the device (L1, the GPU
// twin: docs/design/physics/plans/2026-11-06-l1-gpu-twin-plan.md, T1). The CPU
// twin is engine/physics/lbm/lattice.hpp and the tables are ITS, verbatim:
//
//   * the direction order: 0 is the rest population; 1..26 come in ADJACENT
//     opposite pairs (1,2), (3,4), ... (25,26), faces first, then the twelve
//     edges, then the eight corners (lattice.hpp lines 30-47);
//   * the weights are the same fp32 literals (lattice.hpp lines 49-52), so a
//     decimal that rounds to the same float gives the same bits on both sides;
//   * opposite(i) is i + 1 for an odd i and i - 1 for an even one, 0 for 0
//     (lattice.hpp lines 63-65).
//
// tests/test_gpu_lbm.cpp's GpuLbmLattice.TheTablesAreTheCpusBitForBit dumps
// these through lbm_lattice_dump.slang and compares every entry to the CPU's.
// ---------------------------------------------------------------------------

static const uint kLbmQ = 27u;

// lattice.hpp's kC, one component per array.
static const int kLbmCx[27] = {0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, -1, 1};
static const int kLbmCy[27] = {0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, 1, -1};
static const int kLbmCz[27] = {0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1};

// lattice.hpp's kWRest, kWFace, kWEdge, kWCorner (the nearest floats to 8/27, 2/27, 1/54 and 1/216).
static const float kLbmWRest = 0.2962963f;
static const float kLbmWFace = 0.074074075f;
static const float kLbmWEdge = 0.018518519f;
static const float kLbmWCorner = 0.0046296297f;

// lattice.hpp's weight_of(): the squared length picks the class.
float lbm_weight(uint i) {
    const int n = kLbmCx[i] * kLbmCx[i] + kLbmCy[i] * kLbmCy[i] + kLbmCz[i] * kLbmCz[i];
    return n == 0 ? kLbmWRest : (n == 1 ? kLbmWFace : (n == 2 ? kLbmWEdge : kLbmWCorner));
}

// lattice.hpp's opposite().
uint lbm_opposite(uint i) {
    return i == 0u ? 0u : ((i & 1u) != 0u ? i + 1u : i - 1u);
}
