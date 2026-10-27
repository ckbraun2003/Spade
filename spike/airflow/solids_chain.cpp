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
    return SolidsUpload{.field = pack_solid_field(s.solids),
                        .layout = std::move(*layout),
                        .labels = s.regions.label,
                        .regions = pack_region_table(s.regions)};
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
        case compute::gen::kBinding_airflow_region_table: return bytes(u.regions.words);
        default: return 0u;
    }
}

Result<void> init_solids(ChainRunner& runner, std::span<uint32_t> params, const SolidsUpload& u, uint32_t pcg_block,
                         uint32_t floor, uint32_t cap) {
    params[kParamsFlags] = (params[kParamsFlags] & ~(kParamsFlagSolids | kParamsFlagTheta)) | solids_flags(u.field);
    write_pcg_block(params, pcg_block, u.regions.closed, floor, cap);
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
    return upload(gen::kBinding_airflow_region_table, u.regions.words);
}

}  // namespace spade::airflow_gpu

// ===========================================================================
// B5.3 AND B5.5 (Physics, 2026-10-27; appended as a block of its own): the
// region table and the means, the interior and the wall load.
// ===========================================================================

namespace spade::airflow_gpu {

namespace {

// airflow_region_mean's and airflow_wall_load's tiles in blocks of reduce.hpp's
// kReduceBlockTiles.
[[nodiscard]] uint32_t blocks_of(uint32_t tiles) noexcept {
    return (tiles + af::kReduceBlockTiles - 1u) / af::kReduceBlockTiles;
}

// airflow_solid_velocities' passes (its push word 0).
inline constexpr uint32_t kInteriorRigid = 0;
inline constexpr uint32_t kInteriorLayer1 = 1;
inline constexpr uint32_t kInteriorLayer2 = 2;

// airflow_region_mean's pass 3, the subtract.
inline constexpr uint32_t kRegionSubtractPass = 3;

}  // namespace

RegionTable pack_region_table(const af::FluidRegions& r) {
    RegionTable out;
    std::vector<uint32_t> index(r.count, af::kNoRegion);
    for (uint32_t id = 0; id < r.count; ++id) {
        if (r.closed[id] == 0u) continue;
        index[id] = out.closed++;
        out.words.push_back(id);
        out.words.push_back(r.cells[id]);
        // remove_region_means()' reciprocal (regions.cpp): the host's, 1 / cells rounded once from double.
        out.words.push_back(std::bit_cast<uint32_t>(static_cast<float>(1.0 / static_cast<double>(r.cells[id]))));
        out.words.push_back(0u);  // the mean: the device's
    }
    out.words.insert(out.words.end(), index.begin(), index.end());
    return out;
}

uint64_t region_partials_floats(const af::GridShape& shape, uint32_t closed) noexcept {
    return 2ull * closed * tile_sum_tiles(shape);
}

void append_region_means(std::vector<compute::ChainCommand>& out, const af::GridShape& shape, uint32_t pipeline,
                         PcgVector v, uint32_t closed, uint32_t workgroup_size) {
    if (closed == 0u) return;
    const uint32_t cells = cells_of(shape);
    const uint32_t tiles = tile_sum_tiles(shape);
    const std::array<uint64_t, 3> threads{uint64_t{closed} * tiles, uint64_t{closed} * blocks_of(tiles), closed};
    for (uint32_t pass = 0; pass < 3u; ++pass) {
        compute::ChainCommand c =
            projection_dispatch(pipeline, {.a = v, .b = tile_sum_pass_word(pass), .count = cells}, workgroup_size);
        c.groups = {groups_for(threads[pass], workgroup_size), 1u, 1u};
        out.push_back(c);
    }
}

void append_remove_region_means(std::vector<compute::ChainCommand>& out, const af::GridShape& shape,
                                uint32_t pipeline, PcgVector v, uint32_t closed, uint32_t workgroup_size) {
    if (closed == 0u) return;
    append_region_means(out, shape, pipeline, v, closed, workgroup_size);
    out.push_back(projection_dispatch(
        pipeline, {.a = v, .b = tile_sum_pass_word(kRegionSubtractPass), .count = cells_of(shape)}, workgroup_size));
}

void append_solid_velocities(std::vector<compute::ChainCommand>& out, const af::GridShape& shape, uint32_t pipeline,
                             af::SolidInterior mode, uint32_t workgroup_size) {
    const auto faces = static_cast<uint32_t>(aperture_base(shape, 2u) + uint64_t{shape.nx} * shape.ny * (shape.nz + 1u));
    if (mode == af::SolidInterior::rigid) {
        out.push_back(projection_dispatch(pipeline, {.a = kInteriorRigid, .count = faces}, workgroup_size));
        return;
    }
    out.push_back(projection_dispatch(pipeline, {.a = kInteriorLayer1, .count = faces}, workgroup_size));
    out.push_back(projection_dispatch(pipeline, {.a = kInteriorLayer2, .count = faces}, workgroup_size));
}

std::array<float, kWallLoadFloats> wall_load_words(glm::vec3 about) noexcept {
    std::array<float, kWallLoadFloats> out{};
    out[kWallLoadAbout + 0u] = about.x;
    out[kWallLoadAbout + 1u] = about.y;
    out[kWallLoadAbout + 2u] = about.z;
    return out;
}

af::WallLoad wall_load_of(std::span<const float, kWallLoadFloats> words) noexcept {
    af::WallLoad out;
    out.force = glm::vec3(words[kWallLoadForce], words[kWallLoadForce + 1u], words[kWallLoadForce + 2u]);
    out.moment = glm::vec3(words[kWallLoadMoment], words[kWallLoadMoment + 1u], words[kWallLoadMoment + 2u]);
    return out;
}

void append_wall_load(std::vector<compute::ChainCommand>& out, const af::GridShape& shape, uint32_t pipeline,
                      uint32_t workgroup_size) {
    const uint32_t cells = cells_of(shape);
    const uint32_t tiles = tile_sum_tiles(shape);
    const std::array<uint64_t, 3> threads{tiles, uint64_t{kWallLoadTerms} * blocks_of(tiles), kWallLoadTerms};
    for (uint32_t pass = 0; pass < 3u; ++pass) {
        compute::ChainCommand c =
            projection_dispatch(pipeline, {.b = tile_sum_pass_word(pass), .count = cells}, workgroup_size);
        c.groups = {groups_for(threads[pass], workgroup_size), 1u, 1u};
        out.push_back(c);
    }
}

}  // namespace spade::airflow_gpu

// ===========================================================================
// B5.4 (Physics, 2026-10-27; appended as a block of its own): the solids CG's
// five chains and the host loop.
// ===========================================================================

namespace spade::airflow_gpu {

namespace {

// One grid's CG: each member appends pcg.cpp's ops in its order.
struct CgChain {
    const af::GridShape& shape;
    const MgLayout& layout;
    const SolvePipelines& p;
    uint32_t closed;
    uint32_t workgroup_size;

    [[nodiscard]] uint32_t cells() const noexcept { return cells_of(shape); }

    void update(std::vector<compute::ChainCommand>& out, UpdateOp op) const {
        out.push_back(projection_dispatch(p.update, {.a = static_cast<uint32_t>(op), .count = cells()},
                                          workgroup_size));
    }

    void scalars(std::vector<compute::ChainCommand>& out, ScalarsOp op) const {
        out.push_back(scalars_dispatch(p.solids, op));
    }

    // q = A v (apply_operator()).
    void apply(std::vector<compute::ChainCommand>& out, PcgVector v) const {
        out.push_back(projection_dispatch(p.apply, {.a = v, .count = cells()}, workgroup_size));
    }

    void dot(std::vector<compute::ChainCommand>& out, PcgVector a, PcgVector b, uint32_t slot) const {
        append_tile_dot(out, shape, p.solids, a, b, slot, workgroup_size);
    }

    // gauge(v): remove_region_means() (projection.cpp's gauge_regions()).
    void gauge(std::vector<compute::ChainCommand>& out, PcgVector v) const {
        append_remove_region_means(out, shape, p.region_mean, v, closed, workgroup_size);
    }

    // precond() (pcg.cpp:39-45): level 0's b <- r and its p <- 0
    // (precondition()), the symmetric V(2,2), z <- level 0's p masked, gauge(z).
    void precond(std::vector<compute::ChainCommand>& out) const {
        update(out, UpdateOp::to_level0);
        append_symmetric_v_cycle(out, layout, p.precondition, 0u, workgroup_size);
        update(out, UpdateOp::z_from_level0);
        gauge(out, kPcgZ);
    }

    // start() (pcg.cpp:61-69): r = b - A x masked, gauge(r), precond(),
    // d <- z, rz = r . z.
    void start(std::vector<compute::ChainCommand>& out) const {
        apply(out, kPcgX);
        update(out, UpdateOp::residual);
        gauge(out, kPcgR);
        precond(out);
        update(out, UpdateOp::d_from_z);
        dot(out, kPcgR, kPcgZ, kSlotRz);
    }
};

}  // namespace

SolveChains solve_chains(const af::GridShape& shape, const MgLayout& layout, const SolvePipelines& p,
                         uint32_t closed, uint32_t workgroup_size) {
    const CgChain cg{shape, layout, p, closed, workgroup_size};
    SolveChains c;
    // setup: the scalars' start (:73-80), x <- p masked (:70), start() (:71).
    cg.scalars(c.setup, ScalarsOp::start);
    cg.update(c.setup, UpdateOp::x_from_p);
    cg.start(c.setup);
    // iteration (:111-128).
    cg.apply(c.iteration, kPcgD);
    cg.dot(c.iteration, kPcgD, kPcgQ, kSlotDq);
    cg.scalars(c.iteration, ScalarsOp::step);
    cg.update(c.iteration, UpdateOp::xr);
    cg.gauge(c.iteration, kPcgR);
    cg.precond(c.iteration);
    cg.dot(c.iteration, kPcgR, kPcgZ, kSlotRzNext);
    cg.scalars(c.iteration, ScalarsOp::next);
    cg.update(c.iteration, UpdateOp::direction);
    // measure (:83-109): gauge(x), its subtract behind the gate (a solve
    // stopped and measured leaves x alone), A x, the max-norm, the decision,
    // best <- x.
    if (closed > 0u) {
        append_region_means(c.measure, shape, p.region_mean, kPcgX, closed, workgroup_size);
        cg.scalars(c.measure, ScalarsOp::gate);
        c.measure.push_back(projection_dispatch(
            p.region_mean, {.a = kPcgX, .b = tile_sum_pass_word(kRegionSubtractPass), .count = cg.cells()},
            workgroup_size));
    }
    cg.apply(c.measure, kPcgX);
    append_measure(c.measure, shape, p.solids, workgroup_size, af::kReduceBlockTiles);
    cg.scalars(c.measure, ScalarsOp::decide);
    cg.update(c.measure, UpdateOp::keep_best);
    // restart (:108).
    cg.start(c.restart);
    // finish (:130-137).
    cg.scalars(c.finish, ScalarsOp::finish);
    cg.update(c.finish, UpdateOp::take_best);
    return c;
}

uint64_t solve_partials_floats(const af::GridShape& shape, uint32_t closed) noexcept {
    return std::max<uint64_t>(2ull * tile_sum_tiles(shape), region_partials_floats(shape, closed));
}

Result<SolveChainIds> record_solve(ChainRunner& runner, const SolveChains& chains) {
    SolveChainIds ids;
    const std::pair<const std::vector<compute::ChainCommand>*, uint32_t*> all[] = {
        {&chains.setup, &ids.setup},     {&chains.iteration, &ids.iteration}, {&chains.measure, &ids.measure},
        {&chains.restart, &ids.restart}, {&chains.finish, &ids.finish}};
    for (const auto& [chain, id] : all) {
        Result<uint32_t> recorded = runner.record(*chain);
        if (!recorded) return std::unexpected(recorded.error());
        *id = *recorded;
    }
    return ids;
}

Result<GpuPcgResult> run_solve(ChainRunner& runner, const SolveChainIds& ids, uint32_t floor, uint32_t cap) {
    const uint32_t scalars = compute::gen::kBinding_airflow_scalars;
    if (Result<void> ok = runner.submit(ids.setup, 1); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = runner.submit(ids.iteration, floor); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = runner.submit(ids.measure, 1); !ok) return std::unexpected(ok.error());
    // The device measures at the floor and after each iteration past it, and
    // stops at the cap: max(floor, cap) - floor + 1 measures at the most.
    const uint32_t measures = std::max(floor, cap) - floor + 1u;
    std::array<float, kSlotDecision + kDecisionSlots> word{};
    for (uint32_t read = 1;; ++read) {
        if (Result<void> ok = runner.read(scalars, std::as_writable_bytes(std::span(word))); !ok) {
            return std::unexpected(ok.error());
        }
        if (word[kSlotStop] != 0.0f) break;
        if (read >= measures) {
            return std::unexpected(Error{Code::internal, "airflow solids CG: the device did not stop after " +
                                                             std::to_string(read) + " measures (its cap allows " +
                                                             std::to_string(measures) + ")"});
        }
        if (word[kSlotRestart] != 0.0f) {
            if (Result<void> ok = runner.submit(ids.restart, 1); !ok) return std::unexpected(ok.error());
        }
        if (Result<void> ok = runner.submit(ids.iteration, 1); !ok) return std::unexpected(ok.error());
        if (Result<void> ok = runner.submit(ids.measure, 1); !ok) return std::unexpected(ok.error());
    }
    if (Result<void> ok = runner.submit(ids.finish, 1); !ok) return std::unexpected(ok.error());
    std::array<float, kScalarSlots> s{};
    if (Result<void> ok = runner.read(scalars, std::as_writable_bytes(std::span(s))); !ok) {
        return std::unexpected(ok.error());
    }
    return GpuPcgResult{.residual = s[kSlotResidual],
                        .iterations = static_cast<uint32_t>(s[kSlotIterations]),
                        .stalled = s[kSlotStalled] != 0.0f,
                        .residual_at = static_cast<uint32_t>(s[kSlotResidualAt]),
                        .capped = s[kSlotCapped] != 0.0f};
}

}  // namespace spade::airflow_gpu
