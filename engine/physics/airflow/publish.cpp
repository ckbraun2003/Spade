#include "physics/airflow/publish.hpp"

#include <array>
#include <cmath>
#include <string>

#include "core/fp16.hpp"

namespace spade::physics::airflow {

namespace {

// The per-cell helpers are forced inline: called out of line they cost most
// of the publish (the airflow CPU-performance task, 2026-10-12). The code is
// unchanged, so the published bits are too.
#if defined(_MSC_VER)
#define SPADE_AIRFLOW_HOT __forceinline
#else
#define SPADE_AIRFLOW_HOT inline __attribute__((always_inline))
#endif

using Channels = std::array<float, kFlowChannelCount>;

struct Context {
    const MacGrid& g;
    float inv_dx;
    std::array<bool, 3> periodic;
};

// The velocity component c averaged to the centre of cell (i, j, k).
[[nodiscard]] SPADE_AIRFLOW_HOT float centred(const Context& x, uint32_t c, uint32_t i, uint32_t j, uint32_t k) noexcept {
    const GridShape& s = x.g.shape;
    switch (c) {
        case 0:
            return 0.5f * (x.g.u[u_index(s, i, j, k)] + x.g.u[u_index(s, i + 1u, j, k)]);
        case 1:
            return 0.5f * (x.g.v[v_index(s, i, j, k)] + x.g.v[v_index(s, i, j + 1u, k)]);
        default:
            return 0.5f * (x.g.w[w_index(s, i, j, k)] + x.g.w[w_index(s, i, j, k + 1u)]);
    }
}

// G[c][a] = d u_c / d x_a at the centre of cell (i, j, k).
[[nodiscard]] SPADE_AIRFLOW_HOT std::array<std::array<float, 3>, 3> gradient(const Context& x, uint32_t i, uint32_t j,
                                                           uint32_t k) noexcept {
    const GridShape& s = x.g.shape;
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    std::array<std::array<float, 3>, 3> G{};
    G[0][0] = (x.g.u[u_index(s, i + 1u, j, k)] - x.g.u[u_index(s, i, j, k)]) * x.inv_dx;
    G[1][1] = (x.g.v[v_index(s, i, j + 1u, k)] - x.g.v[v_index(s, i, j, k)]) * x.inv_dx;
    G[2][2] = (x.g.w[w_index(s, i, j, k + 1u)] - x.g.w[w_index(s, i, j, k)]) * x.inv_dx;
    for (uint32_t a = 0; a < 3; ++a) {
        const uint32_t at[3] = {i, j, k};
        if (n[a] == 1u) continue;  // an axis of one cell: no difference along it
        // A periodic axis wraps at its edges (a central difference everywhere);
        // any other goes one-sided there.
        const bool wrap = x.periodic[a];
        const uint32_t lo_q = at[a] == 0u ? (wrap ? n[a] - 1u : 0u) : at[a] - 1u;
        const uint32_t hi_q = at[a] + 1u == n[a] ? (wrap ? 0u : at[a]) : at[a] + 1u;
        const bool central = wrap || (lo_q + 2u == hi_q);
        const float scale = central ? 0.5f * x.inv_dx : x.inv_dx;
        uint32_t lo[3] = {i, j, k};
        uint32_t hi[3] = {i, j, k};
        lo[a] = lo_q;
        hi[a] = hi_q;
        for (uint32_t c = 0; c < 3; ++c) {
            if (c == a) continue;
            G[c][a] = (centred(x, c, hi[0], hi[1], hi[2]) - centred(x, c, lo[0], lo[1], lo[2])) * scale;
        }
    }
    return G;
}

// The fluid cell's channels: u, v, w, dye, p, |omega|, q-hat, reserved.
[[nodiscard]] SPADE_AIRFLOW_HOT Channels fluid_channels(const Context& x, uint32_t i, uint32_t j, uint32_t k) noexcept {
    const auto G = gradient(x, i, j, k);
    const float wx = G[2][1] - G[1][2];
    const float wy = G[0][2] - G[2][0];
    const float wz = G[1][0] - G[0][1];
    const float w2 = (wx * wx + wy * wy) + wz * wz;
    float s2 = 0.0f;
    for (uint32_t r = 0; r < 3; ++r) {
        for (uint32_t c = 0; c < 3; ++c) {
            const float sym = 0.5f * (G[r][c] + G[c][r]);
            s2 += sym * sym;
        }
    }
    const float Q = 0.25f * w2 - 0.5f * s2;
    const float q_hat = Q >= 0.0f ? std::sqrt(Q) : -std::sqrt(-Q);
    return {centred(x, 0, i, j, k), centred(x, 1, i, j, k), centred(x, 2, i, j, k), 0.0f,
            x.g.p[cell_index(x.g.shape, i, j, k)], std::sqrt(w2), q_hat, 0.0f};
}

}  // namespace

Result<PublishCounts> publish_render_volume(const MacGrid& g, const PublishInput& in, FlowVolume& out) {
    const GridShape& s = g.shape;
    const std::size_t cells = std::size_t{s.nx} * s.ny * s.nz;
    if (g.p.size() != cells || g.u.size() != std::size_t{s.nx + 1u} * s.ny * s.nz ||
        g.v.size() != std::size_t{s.nx} * (s.ny + 1u) * s.nz || g.w.size() != std::size_t{s.nx} * s.ny * (s.nz + 1u)) {
        return std::unexpected(Error{Code::invalid_argument, "airflow publish: the grid's arrays do not match its shape"});
    }
    if (!in.solid.empty() && in.solid.size() != cells) {
        return std::unexpected(Error{Code::invalid_argument, "airflow publish: the solid mask holds " +
                                                                 std::to_string(in.solid.size()) + " cells, not " +
                                                                 std::to_string(cells)});
    }
    const Context x{g, 1.0f / s.dx, in.periodic};
    const auto is_solid = [&](uint32_t i, uint32_t j, uint32_t k) {
        return !in.solid.empty() && in.solid[cell_index(s, i, j, k)] != 0u;
    };

    FlowVolumeHeader& h = out.header;
    h = FlowVolumeHeader{};
    h.origin = in.origin;
    h.cell = s.dx;
    h.nx = s.nx;
    h.ny = s.ny;
    h.nz = s.nz;
    out.v.assign(cells * kFlowHalvesPerCell, 0);
    out.s.assign(cells * kFlowHalvesPerCell, 0);
    std::array<bool, kFlowChannelCount> seen{};
    PublishCounts counts;

    const auto write = [&](std::size_t cell, uint32_t c, float value) {
        std::vector<uint16_t>& texture = c < kFlowHalvesPerCell ? out.v : out.s;
        const fp16::Half half = fp16::from_f32(value);
        texture[cell * kFlowHalvesPerCell + (c % kFlowHalvesPerCell)] = half.bits;
        if (half.no_data) {
            ++h.no_data;
            return;
        }
        const float stored = fp16::to_f32(half.bits);
        if (!seen[c]) {
            h.channel_min[c] = stored;
            h.channel_max[c] = stored;
            seen[c] = true;
        } else {
            h.channel_min[c] = stored < h.channel_min[c] ? stored : h.channel_min[c];
            h.channel_max[c] = stored > h.channel_max[c] ? stored : h.channel_max[c];
        }
    };

    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const std::size_t cell = flow_cell_index(h, i, j, k);
                Channels ch{};
                if (!is_solid(i, j, k)) {
                    ch = fluid_channels(x, i, j, k);
                } else {
                    // The fluid face-neighbours, -x, +x, -y, +y, -z, +z.
                    const uint32_t n[3] = {s.nx, s.ny, s.nz};
                    const uint32_t at[3] = {i, j, k};
                    Channels sum{};
                    uint32_t fluid = 0;
                    for (uint32_t a = 0; a < 3; ++a) {
                        for (uint32_t dir = 0; dir < 2; ++dir) {
                            if (dir == 0 ? at[a] == 0u : at[a] + 1u == n[a]) continue;
                            uint32_t nb[3] = {i, j, k};
                            nb[a] = dir == 0 ? at[a] - 1u : at[a] + 1u;
                            if (is_solid(nb[0], nb[1], nb[2])) continue;
                            const Channels f = fluid_channels(x, nb[0], nb[1], nb[2]);
                            for (uint32_t c = 0; c < kFlowChannelCount; ++c) sum[c] += f[c];
                            ++fluid;
                        }
                    }
                    if (fluid == 0u) {  // deeper than the first layer: no data
                        for (uint32_t c = 0; c < kFlowChannelCount; ++c) {
                            if (c == kFlowReserved) {
                                write(cell, c, 0.0f);
                                continue;
                            }
                            (c < kFlowHalvesPerCell ? out.v : out.s)[cell * kFlowHalvesPerCell +
                                                                     (c % kFlowHalvesPerCell)] = fp16::kNoDataBits;
                            ++h.no_data;
                        }
                        continue;
                    }
                    const float inv = 1.0f / static_cast<float>(fluid);
                    for (uint32_t c = 0; c < kFlowChannelCount; ++c) ch[c] = sum[c] * inv;
                    ch[kFlowReserved] = 0.0f;
                    ++counts.extrapolated;
                }
                for (uint32_t c = 0; c < kFlowChannelCount; ++c) write(cell, c, ch[c]);
            }
        }
    }
    h.extrapolated_cells = counts.extrapolated;
    return counts;
}

}  // namespace spade::physics::airflow
