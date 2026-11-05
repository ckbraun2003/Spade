#pragma once

// ---------------------------------------------------------------------------
// step_chain.hpp (plan B, B7's GPU side, G1; the lead's approval, 2026-10-27)
// -- GpuFluid: fluid_step() on the device. The airflow set's kernels run in
// the CPU's order, recorded once into one chain and resubmitted each step,
// with B6's publish as a second chain. Bit-equal to fluid_step() on the device
// of record (TD-14 as amended), in LiveRun's order: the step's discs are
// spread into du before the step (live_run.cpp), so the gather's increments
// are the CPU's du.
//
// THE STEP'S CHAIN (fluid_step.cpp's step_flushed; [solids] only with them):
//   velocity_bc, axes x, y, z
//   [solid_velocities, cfg.interior's passes]
//   advection, pass 0 then pass 1, each over u, v, w (into airflow_next)
//   actuator_gather over u, v, w (the discs, added into airflow_increments)
//   add_increments: u = next + du, du = 0 (the swap, then add_and_clear)
//   velocity_bc, axes x, y, z
//   [solid_velocities]
//   nu > 0: viscous per component (prepare, the sweeps, the periodic alias;
//           with solids, B3's wall links), then velocity_bc, axes x, y, z
//           [and solid_velocities]
//   without solids: projection_chain() (B2), n_v V-cycles, the gradient
//   subtracted, in the same chain
// A chain is recorded per disc count, since the gather's count is a push
// word; a step with no discs dispatches no gather.
//
// SOLIDS (G3, on Physics' hooks: spike/airflow/solids_chain.hpp, B5). create()
// builds the CPU's own scratch for the field (make_fluid_scratch(), then
// attach_solid_field()) and puts it on the device with init_solids(): the
// field, every level's face weights, the region labels and table, the CG's
// floor (cfg.n_v) and cap (cfg.cg_cap). The projection is then
// project_with_solids() (B5.6) after the step's chain: its five chains
// recorded once, the host following the device's decision word. The solids
// are static for a run, so the regions are checked once, at create(): a
// region that can't balance (check_regions()) is the fault every step()
// returns, as the CPU's step returns it, without stepping.
//
// HEALTH. Without solids, plan A's: the chain measures project()'s residual
// after the last cycle (level 0's residual, then the max-norm of it and of b
// by airflow_plan_a_measure, then their quotient by airflow_pcg_scalars' op
// 2), step() reads it and returns pressure_residual, pressure_iterations = n_v
// and unconverged (above kPlanAResidualAnnounce) as fluid_step() forms them,
// bit for bit: a max is exact and order-free. (0fcb99f9) With solids, the CG's report from the device's health word:
// pressure_residual, pressure_iterations, unconverged, capped and stalled, as
// fluid_step() forms them; and a non-finite flow faults the step
// (non_finite, naming the pressure but no sample) when the CG's r . z isn't
// finite, the NaN or infinity having reached the divergence, and every later
// step returns that fault without stepping, as the CPU's does. Without solids
// there is no scan on the device: a non-finite step is found at the frame's
// publish (LiveRun).
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "compute/vulkan/context.hpp"
#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"
#include "physics/airflow/wall_load.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/airflow/projection_chain.hpp"
#include "spike/airflow/solids_chain.hpp"

namespace spade::airflow_gpu {

// airflow_actuator_gather's disc record (airflow_discs, binding 22), in
// floats: [0-2] the centre in the grid's frame, [3] the reach, [4-6] the
// axis, [7] the scale, [8] the swirl's scale, [9] 1 with swirl, [10] the
// disc's point count (disc_spread()'s rings x angles, at most kDiscPoints;
// the slots past it stay 0), from kDiscPointsAt room for kDiscPoints points
// and from kDiscTangentsAt their tangents, three floats each. Derived from af::kDiscPoints, so a change of
// the spread moves the layout with it; the slang module's hand-written copy
// is pinned to this one by the GpuAirflowActuator parity tests.
inline constexpr uint32_t kDiscPointsAt = 16;
inline constexpr uint32_t kDiscTangentsAt = kDiscPointsAt + 3 * physics::airflow::kDiscPoints;
inline constexpr uint32_t kDiscStride = kDiscPointsAt + 6 * physics::airflow::kDiscPoints;

// The discs a step takes: airflow_discs' records.
inline constexpr uint32_t kMaxDiscs = 16;

// The discs' records, from disc_spread() under the step's flush as
// spread_actuator() forms them; one record of zeros for no disc.
[[nodiscard]] std::vector<float> disc_records(std::span<const physics::airflow::ActuatorDisc> discs,
                                              const physics::airflow::GridShape& shape, float rho, float h);

class GpuFluid {
public:
    // Uploads `g` (u, v, w and p), the step's constants and, with a SolidField
    // that has parts, the field (SOLIDS above). Refused, before the device is
    // touched: what make_fluid_scratch() refuses (the config's fields;
    // check_grid() on the domain, with the solids' bytes when there are
    // parts); what attach_solid_field() and solids_upload() refuse (a field
    // made for another shape); a workgroup size with no compiled variant. Then
    // as ChainRunner::create() refuses, the device's limits and the VRAM
    // budget.
    [[nodiscard]] static Result<std::unique_ptr<GpuFluid>> create(compute::VulkanContext& ctx,
                                                                  const physics::airflow::MacGrid& g,
                                                                  const physics::airflow::DomainBc& bc,
                                                                  const physics::airflow::FluidConfig& cfg,
                                                                  const physics::airflow::SolidField* solids = nullptr,
                                                                  uint32_t workgroup_size = 64);

    GpuFluid(const GpuFluid&) = delete;
    GpuFluid& operator=(const GpuFluid&) = delete;

    // One step with `discs` spread into it (at most kMaxDiscs). Their records
    // are uploaded when they differ from the last step's. With solids, the
    // step's chain, then project_with_solids(); the health as HEALTH above.
    [[nodiscard]] Result<physics::airflow::FluidHealth> step(std::span<const physics::airflow::ActuatorDisc> discs);

    // Reads u, v, w and p into `g`, whose shape must be the fluid's.
    [[nodiscard]] Result<void> read(physics::airflow::MacGrid& g) const;

    // The pressure's load on the solids about `about` (the grid's frame), as
    // wall_load() gives it from the current p: airflow_wall_load's three
    // passes (B5.5) into airflow_wall_load, read back. Without solids it is 0,
    // as wall_load() has it for a field with no part.
    [[nodiscard]] Result<physics::airflow::WallLoad> wall_load(glm::vec3 about);

    // B6's publish of the current flow into `out` (publish_chain.hpp's
    // read_flow_volume), the grid's minimum corner at `origin`.
    [[nodiscard]] Result<void> publish(glm::vec3 origin, FlowVolume& out);

    [[nodiscard]] const physics::airflow::GridShape& shape() const noexcept { return shape_; }

private:
    GpuFluid() = default;
    [[nodiscard]] Result<uint32_t> step_chain_for(uint32_t disc_count);

    physics::airflow::GridShape shape_{};
    physics::airflow::DomainBc bc_{};
    physics::airflow::FluidConfig cfg_{};
    uint32_t workgroup_size_ = 64;
    MgLayout layout_{};
    std::unique_ptr<ChainRunner> runner_;
    std::array<std::optional<uint32_t>, kMaxDiscs + 1> step_chains_{};  // by disc count
    std::optional<uint32_t> publish_chain_;
    std::vector<float> uploaded_discs_;  // the records on the device
    // With solids (SOLIDS above).
    bool solids_ = false;
    SolveChainIds projection_{};               // projection_chains()' five, recorded
    std::optional<uint32_t> wall_load_chain_;  // recorded on the first wall_load()
    physics::airflow::FluidHealth fault_{};    // check_regions()' at create(): returned by every step()
    uint32_t extrapolated_ = 0;                // the publish's extrapolated cells (extrapolated_cells())
};

}  // namespace spade::airflow_gpu
