// ---------------------------------------------------------------------------
// live_fluid.hpp -- LiveRun's fluid on B7's GpuFluid: the seam's device
// (Rendering's design note, docs/design/rendering/plans/
// 2026-10-27-run-air-on-gpu.md, slice 2; the seam is LiveRun's LiveFluid,
// analysis/live_run.hpp).
//
// gpu_live_fluid(ctx) is a LiveFluidMaker: LiveRun::start(resolved, model,
// gpu_live_fluid(ctx)) steps the run's fluid on the device, from the run's
// own first fields, faces, config and solids, the CPU's run to the bit on the
// device of record (TD-14). GpuFluid::create()'s refusals are the maker's,
// so start() refuses them by name; start_live_run() then starts again on
// the CPU and says why. The context outlives the run.
//
// Without parts GpuFluid has no scan for a NaN or an infinity (its health
// carries the pressure's iterations only); with parts its step faults on one
// (step_chain.hpp's HEALTH), but not on a finite flow past binary16's range.
// So each frame it publishes is checked on the host: a half with binary16's
// all-ones exponent (a NaN, an infinity or a value past its range,
// physics/airflow/publish.hpp) fails the publish, naming how many and the
// first one's cell, and every step after it, so the run ends there. With
// parts in the air (G3), the cells the publish's solid mask marks are left
// out of the scan: below their first layer they are no-data by publish's rule
// (fp16::kNoDataBits, a NaN), and the first layer is its fluid neighbours'
// mean, which the scan reads (the lead, 2026-10-28).
//
// Header only: its users link spade_airflow_runner and spade::analysis.
// ---------------------------------------------------------------------------
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "analysis/live_run.hpp"
#include "compute/vulkan/context.hpp"
#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "spike/airflow/step_chain.hpp"

namespace spade::airflow_gpu {

class GpuLiveFluid final : public analysis::LiveFluid {
  public:
    explicit GpuLiveFluid(std::unique_ptr<GpuFluid> fluid) noexcept : fluid_(std::move(fluid)) {}

    [[nodiscard]] Result<physics::airflow::FluidHealth> step(
        std::span<const physics::airflow::ActuatorDisc> discs) override {
        if (!dead_.empty()) return std::unexpected(Error{Code::internal, dead_});
        return fluid_->step(discs);
    }

    [[nodiscard]] Result<void> publish(const physics::airflow::PublishInput& in, FlowVolume& out) override {
        if (!dead_.empty()) return std::unexpected(Error{Code::internal, dead_});
        if (Result<void> p = fluid_->publish(in.origin, out); !p) return p;
        std::size_t bad = 0;
        std::size_t first = 0;
        const auto scan = [&](const std::vector<uint16_t>& halves) {
            for (std::size_t at = 0; at < halves.size(); ++at) {
                if ((halves[at] & 0x7C00u) != 0x7C00u) continue;
                const std::size_t cell = at / kFlowHalvesPerCell;
                // A cell the mask marks solid is no-data by publish's rule
                // (below its first layer); its first layer is the mean of its
                // fluid neighbours, which are scanned themselves.
                if (cell < in.solid.size() && in.solid[cell] != 0u) continue;
                if (bad++ == 0u) first = cell;
            }
        };
        scan(out.v);  // V and S lay their cells out alike: the first is V's, else S's
        scan(out.s);
        if (bad == 0u) return {};
        const std::size_t nx = out.header.nx, ny = out.header.ny;
        char why[240];
        std::snprintf(why, sizeof why,
                      "the published flow holds %zu non-finite samples, the first at cell (%zu, %zu, %zu): the "
                      "fluid went NaN, infinite or past binary16's range",
                      bad, first % nx, first / nx % ny, first / (nx * ny));
        dead_ = why;
        return std::unexpected(Error{Code::internal, dead_});
    }

    [[nodiscard]] Result<void> read(physics::airflow::MacGrid& g) const override { return fluid_->read(g); }

  private:
    std::unique_ptr<GpuFluid> fluid_;
    std::string dead_;  // why the run stopped, once a publish found it non-finite
};

// The maker LiveRun::start() takes: GpuFluid::create() on `ctx` from the
// run's start, at `workgroup_size`, its refusals passed on. `sgs_cs` above 0
// turns the subgrid eddy viscosity on in the config the device is made from
// (L52/L54: LiveRun's own config has none, so a host hands it here, as
// flow_movie does); 0, the default, leaves the run's config as it is.
[[nodiscard]] inline analysis::LiveFluidMaker gpu_live_fluid(compute::VulkanContext& ctx,
                                                             uint32_t workgroup_size = 64, float sgs_cs = 0.0f) {
    return [&ctx, workgroup_size, sgs_cs](
               const analysis::LiveFluidStart& s) -> Result<std::unique_ptr<analysis::LiveFluid>> {
        auto config = s.config;
        if (sgs_cs > 0.0f) config.sgs_cs = sgs_cs;
        Result<std::unique_ptr<GpuFluid>> fluid =
            GpuFluid::create(ctx, s.grid, s.domain, config, s.solids, workgroup_size);
        if (!fluid) return std::unexpected(fluid.error());
        return std::make_unique<GpuLiveFluid>(std::move(*fluid));
    };
}

// A run on the GPU, or on the CPU when the device refuses it.
struct LiveRunOn {
    analysis::LiveRun run;
    std::string cpu_because;  // "" on the GPU; else why it isn't: no device, or the device's refusal verbatim
};

// The case start_live_run() starts on the CPU when the device can't take
// its own; both null, the same case. The editor's Run air gives its discs
// case (Interface's item 1, granted by the lead 2026-11-03): a grid that
// resolves the body takes the CPU seconds a step.
struct CpuCase {
    const analysis::ResolvedCase* resolved = nullptr;
    const vehicles::ModelType* model = nullptr;
};

// LiveRun::start() with gpu_live_fluid(*ctx), started again without it when
// the device refuses (or `ctx` is null: "no Vulkan device"), the refusal
// kept, on `cpu`'s case when it gives one. A refusal of the case itself
// is start()'s, as without a device.
[[nodiscard]] inline Result<LiveRunOn> start_live_run(const analysis::ResolvedCase& resolved,
                                                      const vehicles::ModelType& model,
                                                      compute::VulkanContext* ctx,
                                                      uint32_t workgroup_size = 64,
                                                      CpuCase cpu = {}, float sgs_cs = 0.0f) {
    std::string because = "no Vulkan device";
    if (ctx != nullptr) {
        std::optional<Error> refused;
        const analysis::LiveFluidMaker gpu = gpu_live_fluid(*ctx, workgroup_size, sgs_cs);
        const analysis::LiveFluidMaker kept = [&](const analysis::LiveFluidStart& s) {
            Result<std::unique_ptr<analysis::LiveFluid>> device = gpu(s);
            if (!device) refused = device.error();
            return device;
        };
        Result<analysis::LiveRun> on_gpu = analysis::LiveRun::start(resolved, model, kept);
        if (on_gpu) return LiveRunOn{std::move(*on_gpu), {}};
        if (!refused) return std::unexpected(on_gpu.error());
        because = refused->context;
    }
    const bool given = cpu.resolved != nullptr && cpu.model != nullptr;
    Result<analysis::LiveRun> on_cpu =
        given ? analysis::LiveRun::start(*cpu.resolved, *cpu.model) : analysis::LiveRun::start(resolved, model);
    if (!on_cpu) return std::unexpected(on_cpu.error());
    return LiveRunOn{std::move(*on_cpu), std::move(because)};
}

}  // namespace spade::airflow_gpu
