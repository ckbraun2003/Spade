#include "spike/airflow/solids_chain.hpp"

#include <algorithm>
#include <bit>
#include <span>
#include <utility>

#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pcg.hpp"
#include "physics/airflow/reduce.hpp"
#include "spike/airflow/projection_chain.hpp"

namespace spade::airflow_gpu {

namespace af = physics::airflow;

uint64_t pcg_offset(PcgVector v, const af::GridShape& shape) noexcept {
    return static_cast<uint64_t>(v) * shape.nx * shape.ny * shape.nz;
}

uint64_t aperture_base(const af::GridShape& shape, uint32_t c) noexcept {
    const uint64_t faces_u = uint64_t{shape.nx + 1u} * shape.ny * shape.nz;
    const uint64_t faces_v = uint64_t{shape.nx} * (shape.ny + 1u) * shape.nz;
    return c == 0 ? 0u : (c == 1 ? faces_u : faces_u + faces_v);
}

PackedSolidField pack_solid_field(const af::SolidField& f) {
    PackedSolidField out;
    for (uint32_t c = 0; c < 3; ++c) {
        out.apertures.insert(out.apertures.end(), f.theta[c].begin(), f.theta[c].end());
        out.wall_velocity.insert(out.wall_velocity.end(), f.wall[c].begin(), f.wall[c].end());
        out.phi.insert(out.phi.end(), f.phi[c].begin(), f.phi[c].end());
        out.face_tags.insert(out.face_tags.end(), f.link[c].begin(), f.link[c].end());
    }
    // The free-slip layer: a byte a face from word F, four a word, little-endian.
    const std::size_t faces = out.face_tags.size();
    out.face_tags.resize(faces + (faces + 3u) / 4u, 0u);
    std::size_t face = 0;
    for (uint32_t c = 0; c < 3; ++c) {
        for (const uint8_t layer : f.layer[c]) {
            out.face_tags[faces + face / 4u] |= uint32_t{layer} << (8u * (face % 4u));
            ++face;
        }
    }
    out.wall_links.reserve(f.links.size() * kWallLinkWords);
    for (const af::WallLinks& l : f.links) {
        for (const float x : l.inv_f) out.wall_links.push_back(std::bit_cast<uint32_t>(x));
        for (const float x : l.wall) out.wall_links.push_back(std::bit_cast<uint32_t>(x));
    }
    out.has_solids = f.any;
    return out;
}

uint32_t solids_flags(const PackedSolidField& packed) noexcept {
    return packed.has_solids ? kParamsFlagSolids | kParamsFlagTheta : 0u;
}

void write_pcg_block(std::span<uint32_t> params, uint32_t offset, uint32_t closed_regions, uint32_t floor,
                     uint32_t cap) noexcept {
    params[kParamsPcgBlock] = offset;
    params[offset + 0u] = closed_regions;
    params[offset + 1u] = std::bit_cast<uint32_t>(af::kResidualAnnounce);
    params[offset + 2u] = std::bit_cast<uint32_t>(af::kStallGain);
    params[offset + 3u] = af::kStallIterations;
    params[offset + 4u] = floor;
    params[offset + 5u] = cap;
}

namespace {

[[nodiscard]] uint32_t groups_for(uint64_t threads, uint32_t workgroup_size) noexcept {
    return static_cast<uint32_t>((threads + workgroup_size - 1u) / workgroup_size);
}

[[nodiscard]] uint32_t cells_of(const af::GridShape& shape) noexcept { return shape.nx * shape.ny * shape.nz; }

}  // namespace

void append_tile_dot(std::vector<compute::ChainCommand>& out, const af::GridShape& shape, const SolidsPipelines& p,
                     PcgVector a, PcgVector b, uint32_t slot, uint32_t workgroup_size) {
    const uint32_t cells = cells_of(shape);
    const uint32_t tiles = tile_sum_tiles(shape);
    const uint32_t blocks = (tiles + af::kReduceBlockTiles - 1u) / af::kReduceBlockTiles;
    const uint32_t vectors = static_cast<uint32_t>(a) | (static_cast<uint32_t>(b) << 8u);
    compute::ChainCommand dot = projection_dispatch(p.tile_dot, {.a = vectors, .count = cells}, workgroup_size);
    dot.groups = {groups_for(tiles, workgroup_size), 1u, 1u};
    out.push_back(dot);
    const std::array<uint32_t, 2> threads{blocks, 1u};
    for (uint32_t pass = 1; pass <= 2u; ++pass) {
        compute::ChainCommand c = projection_dispatch(
            p.tile_sum, {.b = tile_sum_pass_word(pass), .count = cells, .d = slot}, workgroup_size);
        c.groups = {groups_for(threads[pass - 1u], workgroup_size), 1u, 1u};
        out.push_back(c);
    }
}

void append_measure(std::vector<compute::ChainCommand>& out, const af::GridShape& shape, const SolidsPipelines& p,
                    uint32_t workgroup_size, uint32_t block) {
    const uint32_t cells = cells_of(shape);
    const uint32_t tiles = tile_sum_tiles(shape);
    const std::array<uint32_t, 3> threads{tiles, (tiles + block - 1u) / block, 1u};
    for (uint32_t pass = 0; pass < 3u; ++pass) {
        compute::ChainCommand c =
            projection_dispatch(p.measure, {.b = tile_sum_pass_word(pass, block), .count = cells}, workgroup_size);
        c.groups = {groups_for(threads[pass], workgroup_size), 1u, 1u};
        out.push_back(c);
    }
}

compute::ChainCommand scalars_dispatch(const SolidsPipelines& p, ScalarsOp op) {
    return projection_dispatch(p.scalars, {.a = static_cast<uint32_t>(op), .count = 1u}, 1u);
}

// ===========================================================================
// B5.2b (Physics, 2026-10-27): the preconditioner's chain and init_solids()
// ===========================================================================

namespace {

[[nodiscard]] uint32_t level_cells(const MgLayout& layout, uint32_t level) noexcept {
    const auto& L = layout.levels[level];
    return static_cast<uint32_t>(uint64_t{L[0]} * L[1] * L[2]);
}

}  // namespace

compute::ChainCommand restrict_full_dispatch(const MgLayout& layout, const PreconditionPipelines& p, uint32_t level,
                                             uint32_t workgroup_size) {
    return projection_dispatch(p.restrict_full, {.a = level, .count = level_cells(layout, level + 1u)},
                               workgroup_size);
}

void append_symmetric_v_cycle(std::vector<compute::ChainCommand>& out, const MgLayout& layout,
                              const PreconditionPipelines& p, uint32_t level, uint32_t workgroup_size) {
    if (level + 1u == layout.levels.size()) {
        append_sweeps(out, layout, p.mg, level, af::kCoarsestSymSweeps, workgroup_size, false);
        append_sweeps(out, layout, p.mg, level, af::kCoarsestSymSweeps, workgroup_size, true);
        return;
    }
    append_sweeps(out, layout, p.mg, level, af::kPreSweeps, workgroup_size, false);
    out.push_back(projection_dispatch(p.mg.residual, {.a = level, .count = level_cells(layout, level)},
                                      workgroup_size));
    out.push_back(restrict_full_dispatch(layout, p, level, workgroup_size));
    append_symmetric_v_cycle(out, layout, p, level + 1u, workgroup_size);
    out.push_back(projection_dispatch(p.mg.prolong, {.a = level, .count = level_cells(layout, level)},
                                      workgroup_size));
    append_sweeps(out, layout, p.mg, level, af::kPostSweeps, workgroup_size, true);
}

Result<SolidsUpload> solids_upload(const af::FluidScratch& s) {
    Result<MgLayout> layout = mg_layout(s.projection.mg);
    if (!layout) return std::unexpected(layout.error());
    return SolidsUpload{.field = pack_solid_field(s.solids), .layout = std::move(*layout), .labels = s.regions.label};
}

uint64_t solids_binding_bytes(const SolidsUpload& u, uint32_t binding) noexcept {
    const auto bytes = [](const auto& v) { return std::max<uint64_t>(4u, uint64_t{v.size()} * sizeof(v[0])); };
    switch (binding) {
        case compute::gen::kBinding_airflow_mg_weights: return bytes(u.layout.weights);
        case compute::gen::kBinding_airflow_apertures: return bytes(u.field.apertures);
        case compute::gen::kBinding_airflow_wall_velocity: return bytes(u.field.wall_velocity);
        case compute::gen::kBinding_airflow_phi: return bytes(u.field.phi);
        case compute::gen::kBinding_airflow_face_tags: return bytes(u.field.face_tags);
        case compute::gen::kBinding_airflow_wall_links: return bytes(u.field.wall_links);
        case compute::gen::kBinding_airflow_region_labels: return bytes(u.labels);
        default: return 0u;
    }
}

Result<void> init_solids(ChainRunner& runner, std::span<uint32_t> params, const SolidsUpload& u, uint32_t pcg_block,
                         uint32_t closed_regions, uint32_t floor, uint32_t cap) {
    params[kParamsFlags] = (params[kParamsFlags] & ~(kParamsFlagSolids | kParamsFlagTheta)) | solids_flags(u.field);
    write_pcg_block(params, pcg_block, closed_regions, floor, cap);
    // An empty array is bound as a placeholder and never read: nothing to upload.
    const auto upload = [&](uint32_t binding, const auto& v) -> Result<void> {
        if (v.empty()) return {};
        return runner.upload(binding, std::as_bytes(std::span(v)));
    };
    namespace gen = compute::gen;
    if (Result<void> ok = upload(gen::kBinding_airflow_params, params); !ok) return ok;
    if (Result<void> ok = upload(gen::kBinding_airflow_mg_weights, u.layout.weights); !ok) return ok;
    if (Result<void> ok = upload(gen::kBinding_airflow_apertures, u.field.apertures); !ok) return ok;
    if (Result<void> ok = upload(gen::kBinding_airflow_wall_velocity, u.field.wall_velocity); !ok) return ok;
    if (Result<void> ok = upload(gen::kBinding_airflow_phi, u.field.phi); !ok) return ok;
    if (Result<void> ok = upload(gen::kBinding_airflow_face_tags, u.field.face_tags); !ok) return ok;
    if (Result<void> ok = upload(gen::kBinding_airflow_wall_links, u.field.wall_links); !ok) return ok;
    if (Result<void> ok = upload(gen::kBinding_airflow_region_labels, u.labels); !ok) return ok;
    // B5.3 (the closed regions): the region table's upload, binding 19, goes here.
    return {};
}

}  // namespace spade::airflow_gpu
