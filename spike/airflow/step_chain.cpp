#include "spike/airflow/step_chain.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
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
#include "airflow_mg_smooth.spv.gen.hpp"
#include "airflow_pack_f16.spv.gen.hpp"
#include "airflow_publish_fields.spv.gen.hpp"
#include "airflow_remove_mean.spv.gen.hpp"
#include "airflow_tile_sum.spv.gen.hpp"
#include "airflow_velocity_bc.spv.gen.hpp"
#include "airflow_viscous.spv.gen.hpp"
#include "bindings.gen.hpp"
#include "core/detail/flush_subnormals.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/projection.hpp"
#include "physics/airflow/publish.hpp"
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

// The face block's words (airflow_params_layout.slang): alpha, a velocity per
// side, the ambient.
constexpr uint32_t kFaceBlockWords = 22;

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
                                                   const af::SolidField* solids, uint32_t workgroup_size) {
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
    if (Result<void> ok = af::check_grid(g.shape, bc, false); !ok) return std::unexpected(ok.error());
    if (solids != nullptr && solids->any) {
        return std::unexpected(invalid("a SolidField with parts needs Physics' solids hooks (B7's G3)"));
    }
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

    af::Multigrid mg = af::make_multigrid(s.nx, s.ny, s.nz, af::periodic_axes(bc));
    const std::array<af::FaceKind, 6> kinds = af::face_kinds(bc);
    af::set_kinds(mg, kinds);
    auto layout = mg_layout(mg);
    if (!layout) return std::unexpected(layout.error());
    self->layout_ = std::move(*layout);

    // airflow_params: the header, the level table, then the face block. Its
    // floats are the CPU's own expressions under the step's flush
    // (airflow_params_layout.slang).
    const std::size_t level_words = projection_params_words(self->layout_);
    std::vector<uint32_t> params(level_words + kFaceBlockWords, 0u);
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
    set(gen::kBinding_airflow_advect, std::max<uint64_t>(3u * uint64_t{total}, kFlowChannelCount * cells),
        BufferKind::device_only);
    set(gen::kBinding_airflow_increments, total, BufferKind::device_only);
    if (cfg.nu > 0.0f) set(gen::kBinding_airflow_viscous_rhs, total, BufferKind::device_only);
    set(gen::kBinding_airflow_rhs, cells, BufferKind::device_only);
    set(gen::kBinding_airflow_residual, cells, BufferKind::device_only);
    set(gen::kBinding_airflow_multigrid, self->layout_.multigrid_floats, BufferKind::device_only);
    set(gen::kBinding_airflow_mg_weights, self->layout_.weights.size(), BufferKind::resident);
    set(gen::kBinding_airflow_scalars, kScalarSlots, BufferKind::device_only);
    set(gen::kBinding_airflow_discs, uint64_t{kMaxDiscs} * kDiscStride, BufferKind::resident);
    set(gen::kBinding_airflow_partials, tile_sum_tiles(s), BufferKind::device_only);
    set(gen::kBinding_airflow_publish_v, 2u * cells, BufferKind::staged);
    set(gen::kBinding_airflow_publish_s, 2u * cells, BufferKind::staged);

    const compute::SpirvVariantSet* const kernels[] = {
        &gen::kSpvVariants_airflow_velocity_bc,     &gen::kSpvVariants_airflow_advection,
        &gen::kSpvVariants_airflow_actuator_gather, &gen::kSpvVariants_airflow_add_increments,
        &gen::kSpvVariants_airflow_viscous,         &gen::kSpvVariants_airflow_divergence,
        &gen::kSpvVariants_airflow_mg_smooth,       &gen::kSpvVariants_airflow_mg_residual,
        &gen::kSpvVariants_airflow_mg_restrict,     &gen::kSpvVariants_airflow_mg_prolong,
        &gen::kSpvVariants_airflow_gradient_subtract, &gen::kSpvVariants_airflow_tile_sum,
        &gen::kSpvVariants_airflow_remove_mean,     &gen::kSpvVariants_airflow_publish_fields,
        &gen::kSpvVariants_airflow_pack_f16};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                      .kernels = kernels,
                                                      .push_bytes = sizeof(ProjectionPush),
                                                      .workgroup_size = workgroup_size});
    if (!runner) return std::unexpected(runner.error());
    self->runner_ = std::move(*runner);
    ChainRunner& r = *self->runner_;

    const auto bytes = [](const auto& v) { return std::as_bytes(std::span(v)); };
    if (Result<void> ok = r.upload(gen::kBinding_airflow_params, bytes(params)); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = r.upload(gen::kBinding_airflow_u, bytes(g.u)); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = r.upload(gen::kBinding_airflow_v, bytes(g.v)); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = r.upload(gen::kBinding_airflow_w, bytes(g.w)); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = r.upload(gen::kBinding_airflow_pressure, bytes(g.p)); !ok) return std::unexpected(ok.error());
    if (Result<void> ok = r.upload(gen::kBinding_airflow_mg_weights, bytes(self->layout_.weights)); !ok) {
        return std::unexpected(ok.error());
    }
    // The increments start cleared, as make_fluid_scratch()'s du.
    const ChainCommand clear[] = {dispatch(kAddIncrements, 1u, 0u, total, 0u, workgroup_size)};
    auto cleared = r.record(clear);
    if (!cleared) return std::unexpected(cleared.error());
    if (Result<void> ok = r.submit(*cleared, 1); !ok) return std::unexpected(ok.error());
    return self;
}

Result<uint32_t> GpuFluid::step_chain_for(uint32_t disc_count) {
    if (step_chains_[disc_count]) return *step_chains_[disc_count];
    const af::GridShape& s = shape_;
    const uint32_t wg = workgroup_size_;
    const auto n = face_counts(s);
    std::vector<ChainCommand> chain;
    append_velocity_bc(chain, s, wg);
    for (const uint32_t pass : {0u, 1u}) {
        for (uint32_t c = 0; c < 3; ++c) chain.push_back(dispatch(kAdvection, c, pass, n[c], 0u, wg));
    }
    if (disc_count > 0u) {
        for (uint32_t c = 0; c < 3; ++c) chain.push_back(dispatch(kGather, c, 0u, n[c], disc_count, wg));
    }
    chain.push_back(dispatch(kAddIncrements, 0u, 0u, n[0] + n[1] + n[2], 0u, wg));
    append_velocity_bc(chain, s, wg);
    if (cfg_.nu > 0.0f) {
        append_viscous(chain, s, af::periodic_axes(bc_), cfg_.visc_sweeps, wg);
        append_velocity_bc(chain, s, wg);
    }
    const std::vector<ChainCommand> projection =
        projection_chain(layout_, s, af::face_kinds(bc_), cfg_.n_v, kProjection, wg);
    chain.insert(chain.end(), projection.begin(), projection.end());
    auto id = runner_->record(chain);
    if (!id) return std::unexpected(id.error());
    step_chains_[disc_count] = *id;
    return *id;
}

Result<af::FluidHealth> GpuFluid::step(std::span<const af::ActuatorDisc> discs) {
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
    auto id = step_chain_for(static_cast<uint32_t>(discs.size()));
    if (!id) return std::unexpected(id.error());
    if (Result<void> ok = runner_->submit(*id, 1); !ok) return std::unexpected(ok.error());
    return af::FluidHealth{.pressure_iterations = cfg_.n_v};
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
        append_publish(chain, kPublish, shape_.nx * shape_.ny * shape_.nz, workgroup_size_);
        auto id = runner_->record(chain);
        if (!id) return std::unexpected(id.error());
        publish_chain_ = *id;
    }
    if (Result<void> ok = runner_->submit(*publish_chain_, 1); !ok) return ok;
    // No solid cell, so none is extrapolated (extrapolated_cells()).
    return read_flow_volume(*runner_, shape_, origin, 0u, out);
}

}  // namespace spade::airflow_gpu
