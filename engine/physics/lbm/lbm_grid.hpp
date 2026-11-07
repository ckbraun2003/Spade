#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "physics/lbm/lattice.hpp"

// ---------------------------------------------------------------------------
// physics/lbm/lbm_grid.hpp -- the LBM tier's state: one copy of the 27
// populations per node, STORED SHIFTED, streamed in place (L1; design
// sections 2 and 4).
//
// SHIFTED STORAGE. A slot holds f'_i = f_i - w_i (rho0 = 1), so the rest state
// never enters a sum or a load: rho - 1 = sum f', rho u = sum c f' (the rest
// state has no momentum), and f - f_eq is unchanged by the shift. The shift is
// exact algebra; both bounce-backs keep their coefficient sums at 1.
//
// THE LAYOUT. Direction-major: the slot of direction i at node (x, y, z) is
// f[i * N + x + nx * (y + ny * z)], N = nx ny nz, so each direction is one
// contiguous plane of memory (the device's coalescing).
//
// ESOTERIC PULL (Lehmann 2022; Geier and Schonherr 2017), one copy, every slot
// read and written by exactly one node a step. With opposite(i) = i-bar and a
// step counted from 0:
//   EVEN step, node x:  incoming f_i = slot [i][x];
//                       outgoing f*_i -> slot [i-bar][x].
//   ODD step, node x:   incoming f_i = slot [i-bar][x - c_i];
//                       outgoing f*_j -> slot [j][x + c_j].
// The slots an odd node reads are exactly the slots it writes, so the order
// the nodes run in does not matter, and a node needs no scratch beyond its 27
// values. incoming() is the logical population at the START of the step about
// to run (the parity rule above), so a reader (the macroscopic fields, the
// tests) never sees the twist.
// ---------------------------------------------------------------------------

namespace spade::physics::lbm {

struct LbmGrid {
    uint32_t nx = 0;
    uint32_t ny = 0;
    uint32_t nz = 0;
    uint64_t step = 0;     // steps taken; its parity says which streaming rule runs next
    std::vector<float> f;  // kQ * N shifted populations, direction-major

    [[nodiscard]] std::size_t cells() const noexcept { return std::size_t{nx} * ny * nz; }
    [[nodiscard]] std::size_t cell(uint32_t x, uint32_t y, uint32_t z) const noexcept {
        return x + std::size_t{nx} * (y + std::size_t{ny} * z);
    }
    [[nodiscard]] bool odd() const noexcept { return (step & 1u) != 0u; }
};

[[nodiscard]] inline LbmGrid make_lbm_grid(uint32_t nx, uint32_t ny, uint32_t nz) {
    LbmGrid g;
    g.nx = nx;
    g.ny = ny;
    g.nz = nz;
    g.f.assign(std::size_t{kQ} * g.cells(), 0.0f);
    return g;
}

// The coordinate `v + d` wrapped into [0, n) (d in {-1, 0, 1}, n >= 1).
[[nodiscard]] inline uint32_t wrapped(uint32_t v, int d, uint32_t n) noexcept {
    if (d < 0) return v == 0u ? n - 1u : v - 1u;
    if (d > 0) return v + 1u == n ? 0u : v + 1u;
    return v;
}

// The slot of direction `dir` at the node.
[[nodiscard]] inline std::size_t slot_of(const LbmGrid& g, uint32_t dir, std::size_t cell) noexcept {
    return std::size_t{dir} * g.cells() + cell;
}

// The cell one lattice step along c from (x, y, z), periodic on all faces.
[[nodiscard]] inline std::size_t neighbour_cell(const LbmGrid& g, uint32_t x, uint32_t y, uint32_t z, Vel c) noexcept {
    return g.cell(wrapped(x, c.x, g.nx), wrapped(y, c.y, g.ny), wrapped(z, c.z, g.nz));
}

// The slot incoming population `dir` of node (x, y, z) is read from in the
// step about to run.
[[nodiscard]] inline std::size_t incoming_slot(const LbmGrid& g, uint32_t dir, uint32_t x, uint32_t y,
                                               uint32_t z) noexcept {
    if (!g.odd()) return slot_of(g, dir, g.cell(x, y, z));
    const Vel c = kC[dir];
    const Vel back{static_cast<int8_t>(-c.x), static_cast<int8_t>(-c.y), static_cast<int8_t>(-c.z)};
    return slot_of(g, opposite(dir), neighbour_cell(g, x, y, z, back));
}

// The logical (shifted) population `dir` at the node at the start of the step about to run.
[[nodiscard]] inline float incoming(const LbmGrid& g, uint32_t dir, uint32_t x, uint32_t y, uint32_t z) noexcept {
    return g.f[incoming_slot(g, dir, x, y, z)];
}

}  // namespace spade::physics::lbm
