#include "spike/airflow/projection_chain.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <limits>
#include <string>

#include "physics/airflow/reduce.hpp"

namespace spade::airflow_gpu {

namespace af = physics::airflow;

namespace {

[[nodiscard]] uint64_t cells_of(const std::array<uint32_t, kParamsLevelStride>& level) noexcept {
    return uint64_t{level[0]} * level[1] * level[2];
}

}  // namespace

Result<MgLayout> mg_layout(const af::Multigrid& mg) {
    MgLayout out;
    uint64_t at = 0;      // floats into airflow_multigrid
    uint64_t w_at = 0;    // floats into airflow_mg_weights
    const auto fits = [](uint64_t v) { return v <= std::numeric_limits<uint32_t>::max(); };
    for (std::size_t l = 0; l < mg.levels.size(); ++l) {
        const af::PoissonLevel& L = mg.levels[l];
        const uint64_t cells = uint64_t{L.nx} * L.ny * L.nz;
        std::array<uint32_t, kParamsLevelStride> e{L.nx, L.ny, L.nz};
        if (l > 0) {
            if (!fits(at + 3u * cells)) {
                return std::unexpected(Error{Code::capacity_exceeded, "the multigrid's levels pass 2^32 floats"});
            }
            e[3] = static_cast<uint32_t>(at);
            e[4] = static_cast<uint32_t>(at + cells);
            e[5] = static_cast<uint32_t>(at + 2u * cells);
            at += 3u * cells;
        }
        if (!L.w[0].empty()) {
            e[9] = 1u;
            for (uint32_t a = 0; a < 3; ++a) {
                if (!fits(w_at + L.w[a].size())) {
                    return std::unexpected(Error{Code::capacity_exceeded, "the multigrid's weights pass 2^32 floats"});
                }
                e[6 + a] = static_cast<uint32_t>(w_at);
                out.weights.insert(out.weights.end(), L.w[a].begin(), L.w[a].end());
                w_at += L.w[a].size();
            }
        }
        out.levels.push_back(e);
    }
    out.multigrid_floats = at;
    return out;
}

std::size_t projection_params_words(const MgLayout& layout) noexcept {
    return kParamsLevelTable + kParamsLevelStride * layout.levels.size();
}

void write_projection_params(std::span<uint32_t> params, const MgLayout& layout,
                             const std::array<af::FaceKind, 6>& kinds, const af::ProjectionFloats& floats) {
    params[kParamsInvDx] = std::bit_cast<uint32_t>(floats.inv_dx);
    params[kParamsLevelCount] = static_cast<uint32_t>(layout.levels.size());
    for (uint32_t s = 0; s < 6; ++s) params[kParamsFaceKinds + s] = static_cast<uint32_t>(kinds[s]);
    params[kParamsBCoef] = std::bit_cast<uint32_t>(floats.b_coef);
    params[kParamsGCoef] = std::bit_cast<uint32_t>(floats.g_coef);
    params[kParamsInvCount] = std::bit_cast<uint32_t>(floats.inv_count);
    for (std::size_t l = 0; l < layout.levels.size(); ++l) {
        std::copy(layout.levels[l].begin(), layout.levels[l].end(),
                  params.begin() + static_cast<std::ptrdiff_t>(kParamsLevelTable + kParamsLevelStride * l));
    }
}

compute::ChainCommand projection_dispatch(uint32_t pipeline, const ProjectionPush& push, uint32_t workgroup_size) {
    compute::ChainCommand c{.is_dispatch = true, .pipeline = pipeline};
    std::memcpy(c.push.data(), &push, sizeof push);
    c.groups = {(push.count + workgroup_size - 1u) / workgroup_size, 1u, 1u};
    return c;
}

void append_sweeps(std::vector<compute::ChainCommand>& out, const MgLayout& layout, const ProjectionPipelines& p,
                   uint32_t level, uint32_t sweeps, uint32_t workgroup_size) {
    const auto& L = layout.levels[level];
    // Each colour's slots: (nx + 1) / 2 a row (airflow_mg_smooth.slang).
    const auto slots = static_cast<uint32_t>(uint64_t{(L[0] + 1u) / 2u} * L[1] * L[2]);
    for (uint32_t s = 0; s < sweeps; ++s) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            out.push_back(projection_dispatch(p.smooth, {.a = level, .b = colour, .count = slots}, workgroup_size));
        }
    }
}

void append_v_cycle(std::vector<compute::ChainCommand>& out, const MgLayout& layout, const ProjectionPipelines& p,
                    uint32_t level, uint32_t workgroup_size) {
    if (level + 1u == layout.levels.size()) {
        append_sweeps(out, layout, p, level, af::kCoarsestSweeps, workgroup_size);
        return;
    }
    const auto fine = static_cast<uint32_t>(cells_of(layout.levels[level]));
    const auto coarse = static_cast<uint32_t>(cells_of(layout.levels[level + 1u]));
    append_sweeps(out, layout, p, level, af::kPreSweeps, workgroup_size);
    out.push_back(projection_dispatch(p.residual, {.a = level, .count = fine}, workgroup_size));
    // restrict_residual(), and the coarse p set to 0 (cycle()'s std::fill).
    out.push_back(projection_dispatch(p.restrict_residual, {.a = level, .count = coarse}, workgroup_size));
    append_v_cycle(out, layout, p, level + 1u, workgroup_size);
    out.push_back(projection_dispatch(p.prolong, {.a = level, .count = fine}, workgroup_size));
    append_sweeps(out, layout, p, level, af::kPostSweeps, workgroup_size);
}

uint32_t tile_sum_tiles(const af::GridShape& shape) noexcept {
    const auto tiles = [](uint32_t n) { return (n + af::kReduceTile - 1u) / af::kReduceTile; };
    return tiles(shape.nx) * tiles(shape.ny) * tiles(shape.nz);
}

namespace {

// remove_mean(field): airflow_tile_sum's three passes (one thread per tile,
// one per block of kReduceBlockTiles tiles, then one over the blocks) into
// airflow_scalars[kMeanSlot], then the subtraction. The pushes are {field,
// tile_sum_pass_word(pass), cells, slot} and {field, 0, cells, slot}.
void append_remove_mean(std::vector<compute::ChainCommand>& out, const af::GridShape& shape,
                        const ProjectionPipelines& p, uint32_t field, uint32_t workgroup_size) {
    const auto cells = static_cast<uint32_t>(uint64_t{shape.nx} * shape.ny * shape.nz);
    const uint32_t tile_count = tile_sum_tiles(shape);
    const uint32_t block_count = (tile_count + af::kReduceBlockTiles - 1u) / af::kReduceBlockTiles;
    const std::array<uint32_t, 3> threads{tile_count, block_count, 1u};
    for (uint32_t pass = 0; pass < 3u; ++pass) {
        compute::ChainCommand c = projection_dispatch(
            p.tile_sum, {.a = field, .b = tile_sum_pass_word(pass), .count = cells, .d = kMeanSlot}, workgroup_size);
        c.groups = {(threads[pass] + workgroup_size - 1u) / workgroup_size, 1u, 1u};
        out.push_back(c);
    }
    out.push_back(projection_dispatch(p.remove_mean, {.a = field, .count = cells, .d = kMeanSlot}, workgroup_size));
}

// airflow_tile_sum's and airflow_remove_mean's field selectors.
constexpr uint32_t kFieldPressure = 0;
constexpr uint32_t kFieldRhs = 1;

}  // namespace

std::vector<compute::ChainCommand> projection_chain(const MgLayout& layout, const af::GridShape& shape,
                                                    const std::array<af::FaceKind, 6>& kinds, uint32_t n_v,
                                                    const ProjectionPipelines& p, uint32_t workgroup_size) {
    const bool closed =
        std::none_of(kinds.begin(), kinds.end(), [](af::FaceKind k) { return k == af::FaceKind::pressure_outlet; });
    std::vector<compute::ChainCommand> out;
    const auto cells = static_cast<uint32_t>(uint64_t{shape.nx} * shape.ny * shape.nz);
    out.push_back(projection_dispatch(p.divergence, {.count = cells}, workgroup_size));
    if (closed) append_remove_mean(out, shape, p, kFieldRhs, workgroup_size);
    for (uint32_t n = 0; n < n_v; ++n) {
        append_v_cycle(out, layout, p, 0, workgroup_size);
        if (closed) append_remove_mean(out, shape, p, kFieldPressure, workgroup_size);
    }
    const uint32_t faces[3] = {(shape.nx + 1u) * shape.ny * shape.nz, shape.nx * (shape.ny + 1u) * shape.nz,
                               shape.nx * shape.ny * (shape.nz + 1u)};
    for (uint32_t c = 0; c < 3; ++c) {
        out.push_back(projection_dispatch(p.gradient, {.a = c, .count = faces[c]}, workgroup_size));
    }
    return out;
}

}  // namespace spade::airflow_gpu
