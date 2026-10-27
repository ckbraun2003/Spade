#include "physics/airflow/publish.hpp"

#include <array>
#include <cmath>
#include <string>

#include "core/detail/flush_subnormals.hpp"
#include "core/fp16.hpp"
#include "core/lambda2.hpp"

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
    std::array<std::span<const float>, 3> theta;  // empty: every face open
};

// Whether the face of component a at coordinate f along a, in the line
// through cell (i, j, k), is closed.
[[nodiscard]] SPADE_AIRFLOW_HOT bool closed_face(const Context& x, uint32_t a, uint32_t i, uint32_t j, uint32_t k,
                                                 uint32_t f) noexcept {
    if (x.theta[a].empty()) return false;
    const GridShape& s = x.g.shape;
    const std::size_t idx = a == 0 ? u_index(s, f, j, k) : a == 1 ? v_index(s, i, f, k) : w_index(s, i, j, f);
    return x.theta[a][idx] == 0.0f;
}

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
        // A neighbour is used when it exists (a periodic axis wraps at its
        // edges) and the face between is open: both, a central difference;
        // one, one-sided from the cell; none, 0.
        const bool wrap = x.periodic[a];
        const bool lo_ok = (at[a] != 0u || wrap) && !closed_face(x, a, i, j, k, at[a]);
        const bool hi_ok = (at[a] + 1u != n[a] || wrap) && !closed_face(x, a, i, j, k, at[a] + 1u);
        const uint32_t lo_q = !lo_ok ? at[a] : at[a] == 0u ? n[a] - 1u : at[a] - 1u;
        const uint32_t hi_q = !hi_ok ? at[a] : at[a] + 1u == n[a] ? 0u : at[a] + 1u;
        const float scale = lo_ok && hi_ok ? 0.5f * x.inv_dx : lo_ok || hi_ok ? x.inv_dx : 0.0f;
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

// The fluid cell's channels: u, v, w, dye, p, |omega|, q-hat, lambda2-hat
// (core/lambda2.hpp, from the same gradient).
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
            x.g.p[cell_index(x.g.shape, i, j, k)], std::sqrt(w2), q_hat, lambda2_hat(G)};
}

// publish_render_volume()'s body, run under its wrapper's scope (the wrapper
// rule, core/detail/flush_subnormals.hpp); Core's lambda2_hat() runs inside it.
SPADE_NOINLINE Result<PublishCounts> publish_flushed(const MacGrid& g, const PublishInput& in, FlowVolume& out) {
    out.generation = 0;  // unnamed until it is whole: a refusal may leave it part-written
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
    for (uint32_t c = 0; c < 3; ++c) {
        const std::size_t faces = std::size_t{c == 0 ? s.nx + 1u : s.nx} * (c == 1 ? s.ny + 1u : s.ny) *
                                  (c == 2 ? s.nz + 1u : s.nz);
        if (!in.theta[c].empty() && in.theta[c].size() != faces) {
            return std::unexpected(Error{Code::invalid_argument,
                                         std::string("airflow publish: the ") + "uvw"[c] + " apertures hold " +
                                             std::to_string(in.theta[c].size()) + " faces, not " +
                                             std::to_string(faces)});
        }
    }
    const Context x{g, 1.0f / s.dx, in.periodic, in.theta};
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
                    // The fluid face-neighbours, -x, +x, -y, +y, -z, +z; across the wrap
                    // on a periodic axis of more than one cell (the final review's M3).
                    const uint32_t n[3] = {s.nx, s.ny, s.nz};
                    const uint32_t at[3] = {i, j, k};
                    Channels sum{};
                    uint32_t fluid = 0;
                    for (uint32_t a = 0; a < 3; ++a) {
                        for (uint32_t dir = 0; dir < 2; ++dir) {
                            const bool edge = dir == 0 ? at[a] == 0u : at[a] + 1u == n[a];
                            if (edge && !(in.periodic[a] && n[a] > 1u)) continue;
                            uint32_t nb[3] = {i, j, k};
                            if (edge) {
                                nb[a] = dir == 0 ? n[a] - 1u : 0u;
                            } else {
                                nb[a] = dir == 0 ? at[a] - 1u : at[a] + 1u;
                            }
                            if (is_solid(nb[0], nb[1], nb[2])) continue;
                            const Channels f = fluid_channels(x, nb[0], nb[1], nb[2]);
                            for (uint32_t c = 0; c < kFlowChannelCount; ++c) sum[c] += f[c];
                            ++fluid;
                        }
                    }
                    if (fluid == 0u) {  // deeper than the first layer: no data
                        for (uint32_t c = 0; c < kFlowChannelCount; ++c) {
                            (c < kFlowHalvesPerCell ? out.v : out.s)[cell * kFlowHalvesPerCell +
                                                                     (c % kFlowHalvesPerCell)] = fp16::kNoDataBits;
                            ++h.no_data;
                        }
                        continue;
                    }
                    const float inv = 1.0f / static_cast<float>(fluid);
                    for (uint32_t c = 0; c < kFlowChannelCount; ++c) ch[c] = sum[c] * inv;
                    ++counts.extrapolated;
                }
                for (uint32_t c = 0; c < kFlowChannelCount; ++c) write(cell, c, ch[c]);
            }
        }
    }
    h.extrapolated_cells = counts.extrapolated;
    out.generation = next_flow_generation();  // a new content, for a reader's cache
    return counts;
}

}  // namespace

// Subnormals flushed as fluid_step() flushes them (spec section 4.5 as amended
// 2026-10-22; the GPU's twin is B6's publish_fields). An f16 texel cannot
// show the flush; B6's fp32 comparison can.
Result<PublishCounts> publish_render_volume(const MacGrid& g, const PublishInput& in, FlowVolume& out) {
    const spade::math::ScopedFlushSubnormals flush;
    return publish_flushed(g, in, out);
}

}  // namespace spade::physics::airflow
