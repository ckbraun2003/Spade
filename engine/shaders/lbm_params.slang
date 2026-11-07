// ---------------------------------------------------------------------------
// lbm_params.slang -- the LBM kernels' push-constant block (L1, the GPU twin,
// T1; docs/design/physics/plans/2026-11-06-l1-gpu-twin-plan.md, "Push
// constants"): 25 words, 100 bytes, under Vulkan's guaranteed 128. Its C++ twin
// is spade::lbm_gpu::LbmPush (spike/lbm/lbm_chain.hpp), field for field, and
// tests/test_gpu_lbm.cpp's GpuLbmLattice.ThePushBlockReadsBackWordForWord
// has the dump kernel echo it so the two layouts cannot drift.
//
//   nx, ny, nz     the grid, in nodes
//   parity         the step counter's low bit (0 even, 1 odd): the esoteric rule
//   nu, cw         LbmFlow::nu and ::cw (the molecular viscosity, WALE's constant)
//   periodic_mask  bit a set = axis a is periodic (BoxWalls::periodic)
//   wall_u[18]     BoxWalls::wall_u[axis][side][component] flattened, index
//                  (axis * 2 + side) * 3 + component
// ---------------------------------------------------------------------------

struct LbmPush {
    uint nx;
    uint ny;
    uint nz;
    uint parity;
    float nu;
    float cw;
    uint periodic_mask;
    float wall_u[18];
};

static const uint kLbmPushWords = 25u;
