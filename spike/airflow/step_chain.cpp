#include "spike/airflow/step_chain.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <span>
#include <string>
#include <utility>

#include "airflow_actuator_gather.spv.gen.hpp"
#include "airflow_add_increments.spv.gen.hpp"
#include "airflow_advection.spv.gen.hpp"
#include "airflow_divergence.spv.gen.hpp"
#include "airflow_gradient_subtract.spv.gen.hpp"
#include "airflow_mg_prolong.spv.gen.hpp"
#include "airflow_mg_residual.spv.gen.hpp"
#include "airflow_mg_restrict.spv.gen.hpp"
#include "airflow_mg_restrict_full.spv.gen.hpp"
#include "airflow_mg_smooth.spv.gen.hpp"
#include "airflow_pack_f16.spv.gen.hpp"
#include "airflow_pcg_apply.spv.gen.hpp"
#include "airflow_pcg_measure.spv.gen.hpp"
#include "airflow_pcg_scalars.spv.gen.hpp"
#include "airflow_pcg_update.spv.gen.hpp"
#include "airflow_plan_a_measure.spv.gen.hpp"
#include "airflow_publish_fields.spv.gen.hpp"
#include "airflow_region_mean.spv.gen.hpp"
#include "airflow_remove_mean.spv.gen.hpp"
#include "airflow_sgs.spv.gen.hpp"
#include "airflow_solid_divergence.spv.gen.hpp"
#include "airflow_solid_gradient.spv.gen.hpp"
#include "airflow_solid_velocities.spv.gen.hpp"
#include "airflow_tile_dot.spv.gen.hpp"
#include "airflow_tile_sum.spv.gen.hpp"
#include "airflow_velocity_bc.spv.gen.hpp"
#include "airflow_viscous.spv.gen.hpp"
#include "airflow_wall_load.spv.gen.hpp"
#include "bindings.gen.hpp"
#include "core/detail/flush_subnormals.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/projection.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/regions.hpp"
#include "airflow_line_load.spv.gen.hpp"
#include "airflow_line_sample.spv.gen.hpp"
#include "airflow_line_spread.spv.gen.hpp"
#include "airflow_line_tipcorr.spv.gen.hpp"
#include "airflow_dye_advect.spv.gen.hpp"
#include "airflow_dye_publish.spv.gen.hpp"
#include "airflow_dye_source.spv.gen.hpp"
#include "airflow_dye_tip.spv.gen.hpp"
#include "spike/airflow/publish_chain.hpp"
#include "spike/airflow/solids_chain.hpp"

namespace spade::airflow_gpu {

namespace af = physics::airflow;
namespace gen = compute::gen;
using compute::ChainCommand;

namespace {

// RunnerDesc::kernels' order.
enum Pipeline : uint32_t {
    kVelocityBc = 0,
    kAdvection,
    kGather,
    kAddIncrements,
    kViscous,
    kSgs,
    kDivergence,
    kSmooth,
    kResidual,
    kRestrict,
    kProlong,
    kGradient,
    kTileSum,
    kRemoveMean,
    kPublishFields,
    kPackF16,
    // Plan A's health (0fcb99f9): the residual's max-norm and the quotient (B5's op 2).
    kPlanAMeasure,
    kPcgScalars,
    // With solids only (B5's): the runner without them makes none of these.
    kTileDot,
    kPcgMeasure,
    kPcgApply,
    kRegionMean,
    kRestrictFull,
    kPcgUpdate,
    kSolidDivergence,
    kSolidGradient,
    kSolidVelocities,
    kWallLoad,
};

constexpr ProjectionPipelines kProjection{.divergence = kDivergence,
                                          .smooth = kSmooth,
                                          .residual = kResidual,
                                          .restrict_residual = kRestrict,
                                          .prolong = kProlong,
                                          .gradient = kGradient,
                                          .tile_sum = kTileSum,
                                          .remove_mean = kRemoveMean};
constexpr PublishPipelines kPublish{.fields = kPublishFields, .pack = kPackF16};
constexpr ProjectionSolidsPipelines kProjectionSolids{
    .solve = {.solids = {.tile_sum = kTileSum, .tile_dot = kTileDot, .measure = kPcgMeasure, .scalars = kPcgScalars},
              .precondition = {.mg = kProjection, .restrict_full = kRestrictFull},
              .apply = kPcgApply,
              .region_mean = kRegionMean,
              .update = kPcgUpdate},
    .solid_divergence = kSolidDivergence,
    .solid_gradient = kSolidGradient};

// The kernels in Pipeline's order: the step's and the publish's, then B5's.
constexpr std::array<const compute::SpirvVariantSet*, kPcgScalars + 1> kStepKernels = {
    &gen::kSpvVariants_airflow_velocity_bc,     &gen::kSpvVariants_airflow_advection,
    &gen::kSpvVariants_airflow_actuator_gather, &gen::kSpvVariants_airflow_add_increments,
    &gen::kSpvVariants_airflow_viscous,         &gen::kSpvVariants_airflow_sgs,
    &gen::kSpvVariants_airflow_divergence,
    &gen::kSpvVariants_airflow_mg_smooth,       &gen::kSpvVariants_airflow_mg_residual,
    &gen::kSpvVariants_airflow_mg_restrict,     &gen::kSpvVariants_airflow_mg_prolong,
    &gen::kSpvVariants_airflow_gradient_subtract, &gen::kSpvVariants_airflow_tile_sum,
    &gen::kSpvVariants_airflow_remove_mean,     &gen::kSpvVariants_airflow_publish_fields,
    &gen::kSpvVariants_airflow_pack_f16,        &gen::kSpvVariants_airflow_plan_a_measure,
    &gen::kSpvVariants_airflow_pcg_scalars};
constexpr std::array<const compute::SpirvVariantSet*, kWallLoad - kPcgScalars> kSolidsKernels = {
    &gen::kSpvVariants_airflow_tile_dot,         &gen::kSpvVariants_airflow_pcg_measure,
    &gen::kSpvVariants_airflow_pcg_apply,
    &gen::kSpvVariants_airflow_region_mean,      &gen::kSpvVariants_airflow_mg_restrict_full,
    &gen::kSpvVariants_airflow_pcg_update,       &gen::kSpvVariants_airflow_solid_divergence,
    &gen::kSpvVariants_airflow_solid_gradient,   &gen::kSpvVariants_airflow_solid_velocities,
    &gen::kSpvVariants_airflow_wall_load};

// The face block's words (airflow_params_layout.slang): alpha, a velocity per
// side, the ambient.
constexpr uint32_t kFaceBlockWords = 22;

// The subgrid kernel's (airflow_sgs.slang): its three params words, the scalars' three report words, and the
// invocations of its reduction.
constexpr uint32_t kParamsSgsK = 29;
constexpr uint32_t kParamsSgsC2 = 30;
constexpr uint32_t kParamsSgsCap = 31;
constexpr uint32_t kSlotSgsCapped = 28;
constexpr uint32_t kSlotSgsLimited = 29;
constexpr uint32_t kSlotSgsPeak = 30;
constexpr uint32_t kSgsReduceThreads = 1024;

[[nodiscard]] Error invalid(std::string what) {
    return Error{Code::invalid_argument, "GpuFluid: " + std::move(what)};
}

[[nodiscard]] std::array<uint32_t, 3> face_counts(const af::GridShape& s) noexcept {
    return {(s.nx + 1u) * s.ny * s.nz, s.nx * (s.ny + 1u) * s.nz, s.nx * s.ny * (s.nz + 1u)};
}

// One dispatch by the airflow push convention: four words, the count in 2.
[[nodiscard]] ChainCommand dispatch(uint32_t pipeline, uint32_t a, uint32_t b, uint32_t count, uint32_t d,
                                    uint32_t wg) {
    return projection_dispatch(pipeline, ProjectionPush{a, b, count, d}, wg);
}

// sgs_diffuse(): nu_t per cell, each component's change, each component's add, then the report's two-stage
// reduction (airflow_sgs.slang's modes 0-4).
void append_sgs(std::vector<ChainCommand>& out, const af::GridShape& s, uint32_t wg) {
    const auto n = face_counts(s);
    out.push_back(dispatch(kSgs, 0u, 0u, s.nx * s.ny * s.nz, 0u, wg));
    for (uint32_t c = 0; c < 3; ++c) out.push_back(dispatch(kSgs, c, 1u, n[c], 0u, wg));
    for (uint32_t c = 0; c < 3; ++c) out.push_back(dispatch(kSgs, c, 2u, n[c], 0u, wg));
    out.push_back(dispatch(kSgs, 0u, 3u, kSgsReduceThreads, 0u, wg));
    out.push_back(dispatch(kSgs, 0u, 4u, 1u, 0u, wg));
}

// apply_velocity_boundaries(): apply_axis() per axis, one invocation per line.
void append_velocity_bc(std::vector<ChainCommand>& out, const af::GridShape& s, uint32_t wg) {
    const uint32_t lines[3] = {s.ny * s.nz, s.nx * s.nz, s.nx * s.ny};
    for (uint32_t axis = 0; axis < 3; ++axis) out.push_back(dispatch(kVelocityBc, axis, 0u, lines[axis], 0u, wg));
}

// diffuse(): per component, prepare, then colours 0 and 1 `sweeps` times,
// then the alias face across a periodic axis.
void append_viscous(std::vector<ChainCommand>& out, const af::GridShape& s, const std::array<bool, 3>& periodic,
                    uint32_t sweeps, uint32_t wg) {
    for (uint32_t c = 0; c < 3; ++c) {
        const uint32_t cnt[3] = {s.nx + (c == 0u ? 1u : 0u), s.ny + (c == 1u ? 1u : 0u), s.nz + (c == 2u ? 1u : 0u)};
        const uint32_t faces = cnt[0] * cnt[1] * cnt[2];
        out.push_back(dispatch(kViscous, c, 0u, faces, 0u, wg));
        for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
            for (const uint32_t colour : {0u, 1u}) {
                out.push_back(dispatch(kViscous, c, 1u, ((cnt[0] + 1u) / 2u) * cnt[1] * cnt[2], colour, wg));
            }
        }
        if (periodic[c]) out.push_back(dispatch(kViscous, c, 2u, faces / cnt[c], 0u, wg));
    }
}

// The subgrid kernel's report off the scalars: the counts as uint bits, the peak a float. 0 with Cs 0 (the kernel
// never ran, the words are never read).
void add_sgs_health(af::FluidHealth& h, const std::array<float, kScalarSlots>& scalars, bool on) {
    if (!on) return;
    h.sgs_capped_cells = std::bit_cast<uint32_t>(scalars[kSlotSgsCapped]);
    h.sgs_limited_faces = std::bit_cast<uint32_t>(scalars[kSlotSgsLimited]);
    h.sgs_nu_t_max = scalars[kSlotSgsPeak];
}

}  // namespace

std::vector<float> disc_records(std::span<const af::ActuatorDisc> discs, const af::GridShape& shape, float rho,
                                float h) {
    const spade::math::ScopedFlushSubnormals flush;
    std::vector<float> out(std::max<std::size_t>(discs.size(), 1u) * kDiscStride, 0.0f);
    for (std::size_t d = 0; d < discs.size(); ++d) {
        const af::DiscSpread sp = af::disc_spread(discs[d], shape, rho, h);
        float* r = out.data() + d * kDiscStride;
        for (uint32_t a = 0; a < 3; ++a) {
            r[a] = discs[d].centre[static_cast<glm::length_t>(a)];
            r[4 + a] = discs[d].axis[static_cast<glm::length_t>(a)];
        }
        r[3] = sp.reach;
        r[11] = discs[d].radius;  // the dye's release (airflow_dye_source.slang) reads it; the gather kernel does not
        r[7] = sp.scale;
        r[8] = sp.swirl_scale;
        r[9] = sp.swirl ? 1.0f : 0.0f;
        r[10] = static_cast<float>(sp.count);
        for (uint32_t i = 0; i < af::kDiscPoints; ++i) {
            for (uint32_t a = 0; a < 3; ++a) {
                r[kDiscPointsAt + 3 * i + a] = sp.points[i][static_cast<glm::length_t>(a)];
                r[kDiscTangentsAt + 3 * i + a] = sp.tangent[i][static_cast<glm::length_t>(a)];
            }
        }
    }
    return out;
}

Result<std::unique_ptr<GpuFluid>> GpuFluid::create(compute::VulkanContext& ctx, const af::MacGrid& g,
                                                   const af::DomainBc& bc, const af::FluidConfig& cfg,
                                                   const af::SolidField* solids, uint32_t workgroup_size,
                                                   const LineSetup* line, const DyeSetup* dye) {
    // A copy of make_fluid_scratch()'s refusals (fluid_step.cpp), the rule they
    // mirror, in its order and words: a change there is made here too, until
    // the two share one check.
    const auto positive_finite = [](float x) { return std::isfinite(x) && x > 0.0f; };
    if (!positive_finite(cfg.rho)) return std::unexpected(invalid("rho must be positive and finite"));
    if (!positive_finite(cfg.h)) return std::unexpected(invalid("h must be positive and finite"));
    if (!(cfg.nu >= 0.0f) || !std::isfinite(cfg.nu)) {
        return std::unexpected(invalid("nu must be non-negative and finite"));
    }
    if (cfg.n_v == 0u) return std::unexpected(invalid("n_v must be at least 1"));
    if (cfg.nu > 0.0f && cfg.visc_sweeps == 0u) {
        return std::unexpected(invalid("visc_sweeps must be at least 1 when nu > 0"));
    }
    if (!af::sgs_cs_valid(cfg.sgs_cs)) {
        return std::unexpected(invalid("sgs_cs must be 0 (off) or finite and at most kSgsCsMax"));
    }
    const bool with_solids = solids != nullptr && solids->any;
    if (Result<void> ok = af::check_grid(g.shape, bc, with_solids); !ok) return std::unexpected(ok.error());
    const af::GridShape& s = g.shape;
    const auto n = face_counts(s);
    const uint32_t total = n[0] + n[1] + n[2];
    const uint64_t cells = uint64_t{s.nx} * s.ny * s.nz;
    if (g.u.size() != n[0] || g.v.size() != n[1] || g.w.size() != n[2] || g.p.size() != cells) {
        return std::unexpected(invalid("the grid's arrays do not match its shape"));
    }

    std::unique_ptr<GpuFluid> self(new GpuFluid());
    self->shape_ = s;
    self->bc_ = bc;
    self->cfg_ = cfg;
    self->workgroup_size_ = workgroup_size;
    self->solids_ = with_solids;
    const std::array<af::FaceKind, 6> kinds = af::face_kinds(bc);

    // With solids, the CPU's own scratch for the field, as LiveRun's
    // set_solids() leaves it (attach_solid_field()): its levels' face weights,
    // its region labels, and the regions as check_regions() leaves them, which
    // the region table packs. The scratch goes once the upload is made.
    std::optional<SolidsUpload> upload;
    if (with_solids) {
        Result<af::FluidScratch> scratch = af::make_fluid_scratch(g, bc, cfg);
        if (!scratch) return std::unexpected(scratch.error());
        if (Result<void> ok = af::attach_solid_field(*scratch, g, bc, *solids); !ok) {
            return std::unexpected(ok.error());
        }
        if (const af::RegionFault rf = af::check_regions(scratch->regions, s, bc, scratch->solids);
            rf.region != af::kNoRegion) {
            self->fault_ = af::FluidHealth{.fault = af::FluidFault::ill_posed_region, .faces = rf.faces,
                                           .index = rf.cell};
        }
        Result<SolidsUpload> made = solids_upload(*scratch);
        if (!made) return std::unexpected(made.error());
        upload = std::move(*made);
        af::PublishInput mask;
        mask.solid = scratch->regions.solid;
        mask.periodic = af::periodic_axes(bc);
        Result<uint32_t> extrapolated = af::extrapolated_cells(s, mask);
        if (!extrapolated) return std::unexpected(extrapolated.error());
        self->extrapolated_ = *extrapolated;
        self->layout_ = upload->layout;
    } else {
        af::Multigrid mg = af::make_multigrid(s.nx, s.ny, s.nz, af::periodic_axes(bc));
        af::set_kinds(mg, kinds);
        auto layout = mg_layout(mg);
        if (!layout) return std::unexpected(layout.error());
        self->layout_ = std::move(*layout);
    }

    // airflow_params: the header, the level table, then the face block; with
    // solids, the pcg block after it (init_solids() writes it and word 16).
    // Its floats are the CPU's own expressions under the step's flush
    // (airflow_params_layout.slang).
    const std::size_t level_words = projection_params_words(self->layout_);
    const std::size_t pcg_block = level_words + kFaceBlockWords;
    std::vector<uint32_t> params(pcg_block + (with_solids ? kPcgBlockWords : 0u), 0u);
    {
        const spade::math::ScopedFlushSubnormals flush;
        const std::array<bool, 3> periodic = af::periodic_axes(bc);
        params[0] = s.nx;
        params[1] = s.ny;
        params[2] = s.nz;
        params[4] = std::bit_cast<uint32_t>(s.dx);
        params[5] = std::bit_cast<uint32_t>(1.0f / s.dx);
        params[6] = std::bit_cast<uint32_t>(cfg.h);
        for (uint32_t a = 0; a < 3; ++a) params[8 + a] = periodic[a] ? 1u : 0u;
        params[12] = 0u;
        params[13] = total;
        params[14] = 2u * total;
        params[15] = total;
        params[kParamsFlags] = 0u;
        if (cfg.sgs_cs > 0.0f) {
            // sgs_diffuse()'s own expressions (sgs.cpp), under the same flush.
            params[kParamsSgsK] = std::bit_cast<uint32_t>(cfg.h / (s.dx * s.dx));
            params[kParamsSgsC2] = std::bit_cast<uint32_t>(cfg.sgs_cs * cfg.sgs_cs * s.dx * s.dx);
            params[kParamsSgsCap] = std::bit_cast<uint32_t>(s.dx * s.dx / (8.0f * cfg.h));
        }
        write_projection_params(std::span<uint32_t>(params).first(level_words), self->layout_, kinds,
                                af::projection_floats(s, cfg.rho, cfg.h));
        params[27] = static_cast<uint32_t>(level_words);
        std::size_t w = level_words;
        params[w++] = std::bit_cast<uint32_t>(cfg.nu * cfg.h / (s.dx * s.dx));  // viscous.cpp's alpha
        for (uint32_t side = 0; side < 6; ++side) {
            // A wall's own velocity; otherwise the ambient (an inflow's ghost).
            const glm::vec3 v = bc.face[side] == af::FaceBc::wall ? bc.wall_velocity[side] : bc.ambient;
            for (uint32_t c = 0; c < 3; ++c) params[w++] = std::bit_cast<uint32_t>(v[static_cast<glm::length_t>(c)]);
        }
        for (uint32_t c = 0; c < 3; ++c) {
            params[w++] = std::bit_cast<uint32_t>(bc.ambient[static_cast<glm::length_t>(c)]);
        }
    }

    // Section F, binding by binding. Device-only where nothing crosses to the
    // host, airflow_advect too (only B6's P8 reads it back), so the staging
    // buffer is the largest buffer that does cross: publish_v.
    std::vector<RunnerBuffer> buffers(kAirflowSetBuffers.size());
    for (uint32_t b = 0; b < buffers.size(); ++b) {
        buffers[b] = {std::string(kAirflowSetBuffers[b].name), 4u, BufferKind::device_only};
    }
    const auto set = [&buffers](uint32_t binding, uint64_t floats, BufferKind kind) {
        buffers[binding].bytes = std::max<uint64_t>(floats, 1u) * 4u;
        buffers[binding].kind = kind;
    };
    set(gen::kBinding_airflow_params, params.size(), BufferKind::resident);
    set(gen::kBinding_airflow_u, n[0], BufferKind::resident);
    set(gen::kBinding_airflow_v, n[1], BufferKind::resident);
    set(gen::kBinding_airflow_w, n[2], BufferKind::resident);
    set(gen::kBinding_airflow_pressure, cells, BufferKind::resident);
    set(gen::kBinding_airflow_next, total, BufferKind::device_only);
    // The subgrid kernel's scratch: the change, its limited flags, the raw nu_t and the reduction's partials.
    const uint64_t sgs_floats = cfg.sgs_cs > 0.0f ? 2u * uint64_t{total} + cells + 3u * kSgsReduceThreads : 0u;
    set(gen::kBinding_airflow_advect, std::max({3u * uint64_t{total}, kFlowChannelCount * cells, sgs_floats}),
        BufferKind::device_only);
    set(gen::kBinding_airflow_increments, total, BufferKind::device_only);
    if (cfg.nu > 0.0f) set(gen::kBinding_airflow_viscous_rhs, total, BufferKind::device_only);
    set(gen::kBinding_airflow_rhs, cells, BufferKind::device_only);
    set(gen::kBinding_airflow_residual, cells, BufferKind::device_only);
    set(gen::kBinding_airflow_multigrid, self->layout_.multigrid_floats, BufferKind::device_only);
    set(gen::kBinding_airflow_mg_weights, self->layout_.weights.size(), BufferKind::resident);
    set(gen::kBinding_airflow_scalars, kScalarSlots, BufferKind::staged);  // plan A's health is read from it
    set(gen::kBinding_airflow_discs, uint64_t{kMaxDiscs} * kDiscStride, BufferKind::resident);
    set(gen::kBinding_airflow_partials, 2u * uint64_t{tile_sum_tiles(s)}, BufferKind::device_only);  // r_max, b_max a tile
    set(gen::kBinding_airflow_publish_v, 2u * cells, BufferKind::staged);
    set(gen::kBinding_airflow_publish_s, 2u * cells, BufferKind::staged);
    std::vector<const compute::SpirvVariantSet*> kernels(kStepKernels.begin(), kStepKernels.end());
    if (with_solids) {
        // The field, the labels and the table (bindings 12-19, init_solids()'s),
        // the CG's vectors, its scalars read by the host loop (run_solve()),
        // the reductions' partials and the wall load.
        for (uint32_t b = 0; b < buffers.size(); ++b) {
            if (const uint64_t bytes = solids_binding_bytes(*upload, b); bytes > 0u) {
                buffers[b].bytes = bytes;
                buffers[b].kind = BufferKind::resident;
            }
        }
        set(gen::kBinding_airflow_pcg, uint64_t{kPcgVectors} * cells, BufferKind::device_only);
        set(gen::kBinding_airflow_scalars, kScalarSlots, BufferKind::staged);
        set(gen::kBinding_airflow_partials,
            std::max({2u * uint64_t{tile_sum_tiles(s)}, solve_partials_floats(s, upload->regions),
                      uint64_t{kWallLoadTerms} * tile_sum_tiles(s)}),
            BufferKind::device_only);
        set(gen::kBinding_airflow_wall_load, kWallLoadFloats, BufferKind::staged);
        kernels.insert(kernels.end(), kSolidsKernels.begin(), kSolidsKernels.end());
    }
    // The actuator line: placeholders for the bindings below it, its four buffers, its kernels after all the others.
    uint64_t line_readback = 0;
    if (line != nullptr) {
        if (line->polar == nullptr || line->polar->key.size() < 2u ||
            line->polar->key.size() > kAirflowLinePolarCapacity || line->polar->cl.size() != line->polar->key.size() ||
            line->polar->cd.size() != line->polar->key.size()) {
            return std::unexpected(invalid("line: a polar of 2 to kAirflowLinePolarCapacity points is needed"));
        }
        if (line->binned) return std::unexpected(invalid("line: the binned spread is not written yet"));
        if (line->elements == 0u || line->elements > kAirflowLineMaxElements || line->n_sub == 0u) {
            return std::unexpected(invalid("line: elements must be 1 to kAirflowLineMaxElements and n_sub at least 1"));
        }
        for (uint32_t b = static_cast<uint32_t>(buffers.size()); b <= gen::kBinding_airflow_line_force; ++b) {
            buffers.push_back({"airflow_unused_" + std::to_string(b), 4u, BufferKind::device_only});
        }
        // With the tip correction the buffer holds, after the element records, the groups' (a group is at least an element).
        const uint64_t tip_words = line->tip_correction ? kTipHeaderWords + uint64_t{line->elements} * kTipStride : 0u;
        // With the dye's tip release, then the point count and the points (four floats each; a point is at least an element's worth).
        const uint64_t tip_dye_words = dye != nullptr && dye->tip ? 1u + 4u * uint64_t{line->elements} : 0u;
        set(gen::kBinding_airflow_line_elems, uint64_t{line->elements} * kLineStride + tip_words + tip_dye_words,
            BufferKind::resident);
        set(gen::kBinding_airflow_line_polar, 1u + 3u * uint64_t{line->polar->key.size()}, BufferKind::resident);
        set(gen::kBinding_airflow_line_vel, uint64_t{line->elements} * 4u, BufferKind::device_only);
        set(gen::kBinding_airflow_line_force, uint64_t{line->elements} * kLineForceStride, BufferKind::resident);
        self->line_pipeline_ = static_cast<uint32_t>(kernels.size());
        kernels.push_back(&gen::kSpvVariants_airflow_line_sample);
        kernels.push_back(&gen::kSpvVariants_airflow_line_load);
        kernels.push_back(&gen::kSpvVariants_airflow_line_spread);
        if (line->tip_correction) kernels.push_back(&gen::kSpvVariants_airflow_line_tipcorr);
        line_readback = (uint64_t{line->elements} * kLineForceStride * sizeof(float) + 15u) & ~uint64_t{15};
    }

    // The passive dye (P10): placeholders for the bindings below it, its one buffer, its kernels after all the others.
    if (dye != nullptr) {
        if (Result<void> ok = af::check_dye_release(dye->release); !ok) return std::unexpected(ok.error());
        if (dye->tip) {
            if (Result<void> ok = af::check_tip_release(*dye->tip); !ok) return std::unexpected(ok.error());
            if (line == nullptr) {
                return std::unexpected(invalid("dye: a tip release releases at the line's blade tips and needs a LineSetup"));
            }
            self->tip_on_ = true;
            self->tip_frac_ = dye->tip->tip_frac;
        }
        for (uint32_t b = static_cast<uint32_t>(buffers.size()); b < gen::kBinding_airflow_dye; ++b) {
            buffers.push_back({"airflow_unused_" + std::to_string(b), 4u, BufferKind::device_only});
        }
        buffers.push_back({std::string(kAirflowDyeBuffer.name), 4u, kAirflowDyeBuffer.kind});
        set(gen::kBinding_airflow_dye, kAirflowDyeHeaderFloats + 2u * cells, BufferKind::resident);
        self->dye_pipeline_ = static_cast<uint32_t>(kernels.size());
        kernels.push_back(&gen::kSpvVariants_airflow_dye_advect);
        kernels.push_back(&gen::kSpvVariants_airflow_dye_source);
        kernels.push_back(&gen::kSpvVariants_airflow_dye_publish);
        if (dye->tip) kernels.push_back(&gen::kSpvVariants_airflow_dye_tip);  // after the publish: no other pipeline moves
    }

    // Plan A's health word is read from each step chain's own readback of airflow_scalars: one 16-aligned
    // region a chain, one chain for each disc count.
    constexpr uint64_t kHealthRegion = (uint64_t{kScalarSlots} * sizeof(float) + 15u) & ~uint64_t{15};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                      .kernels = kernels,
                                                      .push_bytes = sizeof(ProjectionPush),
                                                      .workgroup_size = workgroup_size,
                                                      .readback_bytes =
                                                          (with_solids ? 0u : (kMaxDiscs + 1u) * kHealthRegion) +
                                                          line_readback});
    if (!runner) return std::unexpected(runner.error());
    self->runner_ = std::move(*runner);
    ChainRunner& r = *self->runner_;

    const auto bytes = [](const auto& v) { return std::as_bytes(std::span(v)); };
    if (with_solids) {
        // airflow_params whole, with word 16's solid bits and the pcg block,
        // and bindings 12-19.
        if (Result<void> ok = init_solids(r, params, *upload, static_cast<uint32_t>(pcg_block), cfg.n_v, cfg.cg_cap);
            !ok) {
            return std::unexpected(ok.error());
        }
        auto ids = record_solve(r, projection_chains(s, self->layout_, kProjectionSolids, upload->regions,
                                                     workgroup_size));
        if (!ids) return std::unexpected(ids.error());
        self->projection_ = *ids;
    } else {
        if (Result<void> ok = r.upload(gen::kBinding_airflow_params, bytes(params)); !ok) {
            return std::unexpected(ok.error());
        }
        if (Result<void> ok = r.upload(gen::kBinding_airflow_mg_weights, bytes(self->layout_.weights)); !ok) {
            return std::unexpected(ok.error());
        }
    }
    if (Result<void> ok = r.upload(gen::kBinding_airflow_u, bytes(g.u)); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = r.upload(gen::kBinding_airflow_v, bytes(g.v)); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = r.upload(gen::kBinding_airflow_w, bytes(g.w)); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = r.upload(gen::kBinding_airflow_pressure, bytes(g.p)); !ok) return std::unexpected(ok.error());
    // The increments start cleared, as make_fluid_scratch()'s du.
    const ChainCommand clear[] = {dispatch(kAddIncrements, 1u, 0u, total, 0u, workgroup_size)};
    auto cleared = r.record(clear);
    if (!cleared) return std::unexpected(cleared.error());
    if (Result<void> ok = r.submit(*cleared, 1); !ok) return std::unexpected(ok.error());
    if (dye != nullptr) {
        // Header (k = rate * h as emit_dye() forms it, r_in, r_out), then clear air in both halves.
        std::vector<float> image(kAirflowDyeHeaderFloats + 2u * cells, 0.0f);
        image[0] = dye->release.rate * cfg.h;
        image[1] = dye->release.r_in;
        image[2] = dye->release.r_out;
        image[3] = dye->tip ? dye->tip->gain : 0.0f;
        if (Result<void> ok = r.upload(gen::kBinding_airflow_dye, bytes(image)); !ok) return std::unexpected(ok.error());
        self->dye_on_ = true;
    }
    if (line != nullptr) {
        self->line_on_ = true;
        self->line_elements_ = line->elements;
        self->line_n_sub_ = line->n_sub;
        self->line_tip_ = line->tip_correction;
        self->line_polar_ = *line->polar;
        const std::vector<float> polar_floats = polar_records(*line->polar);
        if (Result<void> ok = r.upload(gen::kBinding_airflow_line_polar, bytes(polar_floats)); !ok) {
            return std::unexpected(ok.error());
        }
        const uint32_t e = line->elements;
        const uint32_t p = self->line_pipeline_;
        std::vector<ChainCommand> chain;
        chain.push_back(dispatch(p, e, 0u, 3u * e, 0u, workgroup_size));
        if (line->tip_correction) {
            // One pass is three dispatches over the elements (airflow_line_tipcorr.slang's phases): the first pass's
            // circulation saves the sampled air (phase 0), the others do not (phase 1).
            const int passes = af::LineTipParams{}.iterations;
            for (int pass = 0; pass < passes; ++pass) {
                chain.push_back(dispatch(p + 3u, e, pass == 0 ? 0u : 1u, e, 0u, workgroup_size));
                chain.push_back(dispatch(p + 3u, e, 2u, e, 0u, workgroup_size));
                chain.push_back(dispatch(p + 3u, e, 3u, e, 0u, workgroup_size));
            }
        }
        chain.push_back(dispatch(p + 1u, e, 0u, e, 0u, workgroup_size));
        for (uint32_t c = 0; c < 3; ++c) chain.push_back(dispatch(p + 2u, c, e, n[c], 0u, workgroup_size));
        const ChainReadback force{.buffer = gen::kBinding_airflow_line_force,
                                  .src_offset = 0,
                                  .bytes = uint64_t{e} * kLineForceStride * sizeof(float)};
        auto id = r.record(chain, std::span(&force, 1));
        if (!id) return std::unexpected(id.error());
        self->line_chain_ = *id;
    }
    return self;
}

void GpuFluid::set_phase_timing(bool on) {
    if (on != phase_timing_) {
        step_chains_.fill(std::nullopt);  // re-recorded with (or without) its marks
        dye_chains_.fill(std::nullopt);
        dye_tip_chains_.fill(std::nullopt);
    }
    phase_timing_ = on;
    phases_.clear();
}

Result<uint32_t> GpuFluid::step_chain_for(uint32_t disc_count) {
    if (step_chains_[disc_count]) return *step_chains_[disc_count];
    const af::GridShape& s = shape_;
    const uint32_t wg = workgroup_size_;
    const auto n = face_counts(s);
    std::vector<ChainCommand> chain;
    std::vector<std::string> names;  // the marks' names (phase_timing_): a mark after each phase
    const auto phase = [&](const char* name) {
        if (!phase_timing_) return;
        chain.push_back(ChainCommand{});
        names.emplace_back(name);
    };
    // apply_solid_velocities(g, solids, cfg.interior), after each
    // apply_velocity_boundaries() of the step.
    const auto solid_velocities = [&] {
        if (solids_) append_solid_velocities(chain, s, kSolidVelocities, cfg_.interior, wg);
    };
    append_velocity_bc(chain, s, wg);
    solid_velocities();
    phase("bc");
    for (const uint32_t pass : {0u, 1u}) {
        for (uint32_t c = 0; c < 3; ++c) chain.push_back(dispatch(kAdvection, c, pass, n[c], 0u, wg));
    }
    phase("advection");
    if (disc_count > 0u) {
        for (uint32_t c = 0; c < 3; ++c) chain.push_back(dispatch(kGather, c, 0u, n[c], disc_count, wg));
        phase("gather");
    }
    chain.push_back(dispatch(kAddIncrements, 0u, 0u, n[0] + n[1] + n[2], 0u, wg));
    append_velocity_bc(chain, s, wg);
    solid_velocities();
    phase("increments");
    if (cfg_.sgs_cs > 0.0f) {
        // fluid_step()'s hook: after the sources and the boundary passes, before the molecular diffusion.
        append_sgs(chain, s, wg);
        append_velocity_bc(chain, s, wg);
        solid_velocities();
        phase("sgs");
    }
    if (cfg_.nu > 0.0f) {
        append_viscous(chain, s, af::periodic_axes(bc_), cfg_.visc_sweeps, wg);
        append_velocity_bc(chain, s, wg);
        solid_velocities();
        phase("viscous");
    }
    // With solids the projection is project_with_solids()' own chains (step()).
    if (!solids_) {
        // project()'s health: the residual at level 0 after the last cycle, its max-norm against b's, and
        // their quotient (slot kSlotResidual), measured before the gradient as the CPU measures it.
        std::vector<ChainCommand> measure;
        measure.push_back(dispatch(kResidual, 0u, 0u, static_cast<uint32_t>(uint64_t{s.nx} * s.ny * s.nz), 0u, wg));
        append_measure(measure, s, SolidsPipelines{.measure = kPlanAMeasure}, wg, af::kReduceBlockTiles);
        measure.push_back(scalars_dispatch(SolidsPipelines{.scalars = kPcgScalars}, ScalarsOp::residual));
        const std::vector<ChainCommand> projection =
            projection_chain(layout_, s, af::face_kinds(bc_), cfg_.n_v, kProjection, wg, measure);
        chain.insert(chain.end(), projection.begin(), projection.end());
        phase("projection");
    }
    // Without solids the chain ends with a readback of the scalars (plan A's health word); with solids the CG
    // loop reads them by read().
    const ChainReadback health_word{.buffer = gen::kBinding_airflow_scalars,
                                    .src_offset = 0,
                                    .bytes = uint64_t{kScalarSlots} * sizeof(float)};
    auto id = solids_ ? runner_->record(chain) : runner_->record(chain, std::span(&health_word, 1));
    if (!id) return std::unexpected(id.error());
    step_chains_[disc_count] = *id;
    phase_names_[disc_count] = std::move(names);
    return *id;
}

Result<af::FluidHealth> GpuFluid::step(std::span<const af::ActuatorDisc> discs, std::span<const af::LineRotor> lines) {
    if (!line_on_) return std::unexpected(invalid("step: this GpuFluid was created without a LineSetup"));
    for (const af::LineRotor& rotor : lines) {
        if (rotor.polar == nullptr || rotor.polar->key != line_polar_.key || rotor.polar->cl != line_polar_.cl ||
            rotor.polar->cd != line_polar_.cd) {
            return std::unexpected(invalid("step: a rotor's polar is not the LineSetup's"));
        }
    }
    if (fault_.fault != af::FluidFault::none) return fault_;
    Result<LineRecords> records = line_records(lines, shape_, cfg_.rho, cfg_.h, line_n_sub_);
    if (!records) return std::unexpected(records.error());
    if (records->elems.size() != std::size_t{line_elements_} * kLineStride) {
        return std::unexpected(invalid("step: the rotors give " + std::to_string(records->elems.size() / kLineStride) +
                                       " line elements, the LineSetup says " + std::to_string(line_elements_)));
    }
    if (!line_tip_ && records->tip.size() > kTipHeaderWords) {
        return std::unexpected(invalid("step: a rotor's tip correction is on and the LineSetup was made without it"));
    }
    if (line_tip_) {  // the groups follow the element records in the one buffer
        records->elems.insert(records->elems.end(), records->tip.begin(), records->tip.end());
    }
    if (tip_on_) {  // then the dye's release points: the count, and x, y, z and a spare each (airflow_dye_tip.slang)
        const std::vector<glm::vec3> points = af::tip_points(lines, shape_.dx, cfg_.h, line_n_sub_, tip_frac_);
        records->elems.push_back(static_cast<float>(points.size()));
        for (const glm::vec3& p : points) {
            records->elems.insert(records->elems.end(), {p.x, p.y, p.z, 0.0f});
        }
        tip_live_ = true;
    }
    if (Result<void> ok = runner_->upload(gen::kBinding_airflow_line_elems, std::as_bytes(std::span(records->elems)));
        !ok) {
        return std::unexpected(ok.error());
    }
    if (line_tip_ || tip_on_) records->elems.resize(std::size_t{line_elements_} * kLineStride);  // line_reports() reads the elements
    if (Result<void> ok = runner_->submit(line_chain_, 1); !ok) return std::unexpected(ok.error());
    const std::span<const std::byte> force = runner_->readback(line_chain_, 0);
    if (force.size() != std::size_t{line_elements_} * kLineForceStride * sizeof(float)) {
        return std::unexpected(Error{Code::internal, "GpuFluid: the line chain's readback of the force is " +
                                                         std::to_string(force.size()) + " bytes"});
    }
    std::vector<float> force_floats(std::size_t{line_elements_} * kLineForceStride);
    std::memcpy(force_floats.data(), force.data(), force.size());
    line_reports_ = airflow_gpu::line_reports(*records, lines, force_floats);
    Result<af::FluidHealth> health = step(discs);
    tip_live_ = false;  // the points are this step's: a later step without lines releases none
    return health;
}

// The dye's chain for a step with `disc_count` discs: advect_scalar()'s two passes, then the release (which also copies the advected
// field to the cell array). Recorded once a disc count, with a mark after it when the phases are timed.
Result<uint32_t> GpuFluid::dye_chain_for(uint32_t disc_count, bool with_tip) {
    std::optional<uint32_t>& cached = with_tip ? dye_tip_chains_[disc_count] : dye_chains_[disc_count];
    if (cached) return *cached;
    const uint32_t cells = shape_.nx * shape_.ny * shape_.nz;
    const uint32_t wg = workgroup_size_;
    std::vector<ChainCommand> chain;
    chain.push_back(dispatch(dye_pipeline_, 0u, 0u, cells, 0u, wg));
    chain.push_back(dispatch(dye_pipeline_, 0u, 1u, cells, 0u, wg));
    chain.push_back(dispatch(dye_pipeline_ + 1u, disc_count, 0u, cells, 0u, wg));
    if (with_tip) chain.push_back(dispatch(dye_pipeline_ + 3u, line_elements_, line_tip_ ? 1u : 0u, cells, 0u, wg));
    if (phase_timing_) chain.push_back(ChainCommand{});
    auto id = runner_->record(chain);
    if (!id) return std::unexpected(id.error());
    cached = *id;
    return *id;
}

// The step, then the dye: the CPU advects the dye at the end of fluid_step() with its final velocity and the caller releases it
// after (dye.hpp), so the dye chain runs after the step's chain and, with solids, after its projection.
Result<af::FluidHealth> GpuFluid::step(std::span<const af::ActuatorDisc> discs) {
    Result<af::FluidHealth> health = step_flow(discs);
    if (!health || !dye_on_ || health->fault != af::FluidFault::none) return health;
    auto id = dye_chain_for(static_cast<uint32_t>(discs.size()), tip_on_ && tip_live_);
    if (!id) return std::unexpected(id.error());
    if (Result<void> ok = runner_->submit(*id, 1); !ok) return std::unexpected(ok.error());
    if (phase_timing_) {
        const Result<std::vector<uint64_t>> ns = runner_->durations_ns(*id);
        if (ns && ns->size() == 1u) phases_.push_back({"dye", static_cast<double>((*ns)[0]) * 1.0e-6, 1u});
    }
    return health;
}

Result<void> GpuFluid::read_dye(std::span<float> out) const {
    if (!dye_on_) return std::unexpected(invalid("read_dye(): this GpuFluid was created without a DyeSetup"));
    const std::size_t cells = std::size_t{shape_.nx} * shape_.ny * shape_.nz;
    if (out.size() != cells) return std::unexpected(invalid("read_dye() into " + std::to_string(out.size()) + " floats, the fluid has " +
                                                            std::to_string(cells) + " cells"));
    std::vector<float> image(kAirflowDyeHeaderFloats + 2u * cells);
    if (Result<void> ok = runner_->read(gen::kBinding_airflow_dye, std::as_writable_bytes(std::span(image))); !ok) return ok;
    std::copy_n(image.begin() + kAirflowDyeHeaderFloats, cells, out.begin());
    return {};
}

Result<af::FluidHealth> GpuFluid::step_flow(std::span<const af::ActuatorDisc> discs) {
    if (discs.size() > kMaxDiscs) {
        return std::unexpected(Error{Code::capacity_exceeded, "GpuFluid: " + std::to_string(discs.size()) +
                                                                  " discs, past kMaxDiscs (" +
                                                                  std::to_string(kMaxDiscs) + ")"});
    }
    if (!discs.empty()) {
        std::vector<float> records = disc_records(discs, shape_, cfg_.rho, cfg_.h);
        if (records != uploaded_discs_) {
            if (Result<void> ok = runner_->upload(gen::kBinding_airflow_discs, std::as_bytes(std::span(records)));
                !ok) {
                return std::unexpected(ok.error());
            }
            uploaded_discs_ = std::move(records);
        }
    }
    // A region that can't balance faults every step before it starts, as the
    // CPU's step_flushed() returns its fault.
    if (fault_.fault != af::FluidFault::none) return fault_;
    auto id = step_chain_for(static_cast<uint32_t>(discs.size()));
    if (!id) return std::unexpected(id.error());
    if (Result<void> ok = runner_->submit(*id, 1); !ok) return std::unexpected(ok.error());
    phases_.clear();
    if (phase_timing_) {
        // The marks' durations of the chain just submitted (empty if the device cannot time compute work).
        const Result<std::vector<uint64_t>> ns = runner_->durations_ns(*id);
        const std::vector<std::string>& names = phase_names_[discs.size()];
        if (ns && ns->size() == names.size()) {
            for (std::size_t i = 0; i < names.size(); ++i) {
                phases_.push_back({names[i], static_cast<double>((*ns)[i]) * 1.0e-6, 1u});
            }
        }
    }
    if (!solids_) {
        // Plan A's report as the CPU forms it (fluid_step.cpp): the residual after the cycles, announced
        // above its own mark. The chain copied the scalars into its readback region before its fence
        // signalled, so this reads host memory only: the step makes one wait. No scan on the device: a
        // non-finite step is found at the publish (LiveRun).
        std::array<float, kScalarSlots> scalars{};
        const std::span<const std::byte> word = runner_->readback(*id, 0);
        if (word.size() != sizeof(scalars)) {
            return std::unexpected(Error{Code::internal, "GpuFluid: the step chain's readback of airflow_scalars "
                                                         "is " + std::to_string(word.size()) + " bytes"});
        }
        std::memcpy(scalars.data(), word.data(), sizeof(scalars));
        af::FluidHealth h{.pressure_iterations = cfg_.n_v};
        h.pressure_residual = scalars[kSlotResidual];
        h.unconverged = h.pressure_residual > af::kPlanAResidualAnnounce;
        add_sgs_health(h, scalars, cfg_.sgs_cs > 0.0f);
        return h;
    }
    SolveTimes solve_times;
    Result<GpuProjectionReport> projected =
        project_with_solids(*runner_, projection_, cfg_.n_v, cfg_.cg_cap, phase_timing_ ? &solve_times : nullptr);
    if (!projected) return std::unexpected(projected.error());
    if (phase_timing_) {
        const std::pair<const char*, SolveTimes::Phase> cg[] = {
            {"cg_setup", solve_times.setup},     {"cg_iterations", solve_times.iterations},
            {"cg_measure", solve_times.measure}, {"cg_restart", solve_times.restart},
            {"cg_finish", solve_times.finish},   {"cg_reads", solve_times.reads}};
        for (const auto& [name, phase] : cg) phases_.push_back({name, phase.ms, phase.n});
    }
    // fluid_step()'s health from the CG's report, with solids.
    const af::ProjectionReport& rep = projected->report;
    // A non-finite flow, which the CPU's scan() names. The report can't show
    // one: its max-norms pass over a NaN, so a solve gone NaN reads converged
    // (residual 0). r . z is a sum over the region's cells, so it carries any
    // NaN or infinity the divergence took from u, v or w. The pressure is
    // named and no sample (kNoSample): the device scans none (Physics' review
    // of G3, 2026-10-28). Kept, as the CPU keeps its fault: every later step
    // returns it without stepping.
    std::array<float, kScalarSlots> scalars{};
    if (Result<void> ok = runner_->read(gen::kBinding_airflow_scalars, std::as_writable_bytes(std::span(scalars)));
        !ok) {
        return std::unexpected(ok.error());
    }
    if (!std::isfinite(rep.residual) || !std::isfinite(scalars[kSlotRz])) {
        fault_ = af::FluidHealth{.fault = af::FluidFault::non_finite, .quantity = 3, .index = af::kNoSample};
        return fault_;
    }
    af::FluidHealth h{};
    h.pressure_residual = rep.residual;
    h.pressure_iterations = rep.iterations;
    h.unconverged = rep.residual > af::kResidualAnnounce;
    h.capped = h.unconverged && !rep.stalled;
    h.stalled = h.unconverged && rep.stalled;
    add_sgs_health(h, scalars, cfg_.sgs_cs > 0.0f);
    return h;
}

Result<af::WallLoad> GpuFluid::wall_load(glm::vec3 about) {
    if (!solids_) return af::WallLoad{};
    if (!wall_load_chain_) {
        std::vector<ChainCommand> chain;
        append_wall_load(chain, shape_, kWallLoad, workgroup_size_);
        auto id = runner_->record(chain);
        if (!id) return std::unexpected(id.error());
        wall_load_chain_ = *id;
    }
    const std::array<float, kWallLoadFloats> words = wall_load_words(about);
    if (Result<void> ok = runner_->upload(gen::kBinding_airflow_wall_load, std::as_bytes(std::span(words))); !ok) {
        return std::unexpected(ok.error());
    }
    if (Result<void> ok = runner_->submit(*wall_load_chain_, 1); !ok) return std::unexpected(ok.error());
    std::array<float, kWallLoadFloats> out{};
    if (Result<void> ok = runner_->read(gen::kBinding_airflow_wall_load, std::as_writable_bytes(std::span(out))); !ok) {
        return std::unexpected(ok.error());
    }
    return wall_load_of(out);
}

Result<void> GpuFluid::read(af::MacGrid& g) const {
    const auto n = face_counts(shape_);
    const uint64_t cells = uint64_t{shape_.nx} * shape_.ny * shape_.nz;
    if (g.shape.nx != shape_.nx || g.shape.ny != shape_.ny || g.shape.nz != shape_.nz || g.u.size() != n[0] ||
        g.v.size() != n[1] || g.w.size() != n[2] || g.p.size() != cells) {
        return std::unexpected(invalid("read() into a grid of another shape"));
    }
    const auto bytes = [](auto& v) { return std::as_writable_bytes(std::span(v)); };
    if (Result<void> ok = runner_->read(gen::kBinding_airflow_u, bytes(g.u)); !ok) return ok;
    if (Result<void> ok = runner_->read(gen::kBinding_airflow_v, bytes(g.v)); !ok) return ok;
    if (Result<void> ok = runner_->read(gen::kBinding_airflow_w, bytes(g.w)); !ok) return ok;
    return runner_->read(gen::kBinding_airflow_pressure, bytes(g.p));
}

Result<void> GpuFluid::publish(glm::vec3 origin, FlowVolume& out) {
    if (!publish_chain_) {
        std::vector<ChainCommand> chain;
        PublishPipelines pipelines = kPublish;
        if (dye_on_) pipelines.dye = dye_pipeline_ + 2u;  // airflow_dye_publish: channel 3 from the dye
        append_publish(chain, pipelines, shape_.nx * shape_.ny * shape_.nz, workgroup_size_);
        auto id = runner_->record(chain);
        if (!id) return std::unexpected(id.error());
        publish_chain_ = *id;
    }
    if (Result<void> ok = runner_->submit(*publish_chain_, 1); !ok) return ok;
    // The solid cells written from their fluid side, counted once at create()
    // (extrapolated_cells(); 0 without solids).
    return read_flow_volume(*runner_, shape_, origin, extrapolated_, out);
}

}  // namespace spade::airflow_gpu
