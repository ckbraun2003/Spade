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
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "compute/vulkan/context.hpp"
#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/dye.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"
#include "physics/airflow/wall_load.hpp"
#include "physics/airflow/line.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/airflow/line_chain.hpp"
#include "spike/airflow/projection_chain.hpp"
#include "spike/airflow/solids_chain.hpp"

namespace spade::airflow_gpu {

// airflow_actuator_gather's disc record (airflow_discs, binding 22), in
// floats: [0-2] the centre in the grid's frame, [3] the reach, [4-6] the
// axis, [7] the scale, [8] the swirl's scale, [9] 1 with swirl, [10] the
// disc's point count (disc_spread()'s rings x angles, at most kDiscPoints;
// the slots past it stay 0), [11] the disc's radius (airflow_dye_source reads it; the gather kernel does not), from kDiscPointsAt room for kDiscPoints points
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

// THE ACTUATOR LINE (P4c; line_chain.hpp, the kernels airflow_line_{sample,load,spread}): what a GpuFluid that steps
// rotor lines is created with. The element count is fixed for the run (rotors x blades x elements x n_sub: it follows
// the grid's dx and the blades, not the flow); every step's rotors must give exactly `elements`.
struct LineSetup {
    const physics::airflow::PolarTable* polar = nullptr;  // every rotor's polar: this table (its values are compared)
    uint32_t elements = 0;                                // all rotors' elements a step, every sub-azimuth counted
    uint32_t n_sub = 1;
    // L72k's A1: the chain carries airflow_line_tipcorr (the tip/smearing correction, between the sample and the load) and the
    // elements buffer the groups' records. A rotor with tip_correction on is refused by a GpuFluid made without it.
    bool tip_correction = true;  // the rotors' default (LineRotor::tip_correction); a LineSetup must agree with them
};

// THE PASSIVE DYE (P10; physics/airflow/dye.hpp, the kernels airflow_dye_{advect,source,publish}): what a GpuFluid that carries a
// dye is created with. After every step (and after the solids' projection) the dye is advected with the step's final velocity and
// released at the step's discs, as fluid_step() advects the CPU's FluidScratch::dye and emit_dye() releases it; publish() then
// writes it into V's channel 3. Passive: with or without it, u, v, w and p are the same bits.
struct DyeSetup {
    physics::airflow::DyeRelease release{};
};

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
                                                                  uint32_t workgroup_size = 64,
                                                                  const LineSetup* line = nullptr,
                                                                  const DyeSetup* dye = nullptr);

    GpuFluid(const GpuFluid&) = delete;
    GpuFluid& operator=(const GpuFluid&) = delete;

    // One step with `discs` spread into it (at most kMaxDiscs). Their records
    // are uploaded when they differ from the last step's. With solids, the
    // step's chain, then project_with_solids(); the health as HEALTH above.
    [[nodiscard]] Result<physics::airflow::FluidHealth> step(std::span<const physics::airflow::ActuatorDisc> discs);

    // The step with `lines`' elements and `discs` both: the line's sample, load and spread run first, on the air as the
    // step begins (the CPU: spread_line() into the increments, then spread_actuator()'s, then fluid_step()); their
    // increments add to the discs'. Refused (invalid_argument): a fluid created without a LineSetup, a rotor whose
    // polar differs from the setup's or whose line spread_line() refuses, an element count other than the setup's.
    // line_reports() are the rotors' reports of this step.
    [[nodiscard]] Result<physics::airflow::FluidHealth> step(std::span<const physics::airflow::ActuatorDisc> discs,
                                                             std::span<const physics::airflow::LineRotor> lines);
    [[nodiscard]] const std::vector<physics::airflow::LineReport>& line_reports() const noexcept {
        return line_reports_;
    }

    // The dye, one float a cell in cell order, read into `out` (the fluid's cells, else invalid_argument); refused when the fluid was
    // made without a DyeSetup.
    [[nodiscard]] Result<void> read_dye(std::span<float> out) const;
    [[nodiscard]] bool dye_enabled() const noexcept { return dye_on_; }

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

    // The fence waits the runner has made (ChainRunner::waits()): a step without solids, with its discs
    // already on the device, makes one, the plan A health word coming from the chain's own readback.
    [[nodiscard]] uint64_t waits() const noexcept { return runner_ ? runner_->waits() : 0; }

    // OPT-IN DIAGNOSTIC (L74b): the time of each phase of the LAST step(). On, the step chain carries a timestamp
    // mark after each phase (a mark writes a device timestamp: no bit of the flow moves), and with solids the CG's
    // host loop is timed by the wall clock per submit and read. A phase is {name, ms, count}: count = how many
    // submits or reads it took (1 for a chain phase). The chain phases are, in order, those the step has ("bc",
    // "advection", "gather", "increments", "sgs", "viscous", and without solids "projection"); with solids the CG's:
    // "cg_setup", "cg_iterations", "cg_measure", "cg_restart", "cg_finish", and "cg_reads" (the host's waits on the
    // decision word). Off (the default) the step is exactly what it was. A device that cannot time compute work
    // leaves the chain phases out (the CG's wall times are still there).
    struct PhaseTime {
        std::string name;
        double ms = 0.0;
        uint32_t count = 0;
    };
    void set_phase_timing(bool on);
    [[nodiscard]] const std::vector<PhaseTime>& phase_times() const noexcept { return phases_; }

private:
    GpuFluid() = default;
    [[nodiscard]] Result<uint32_t> step_chain_for(uint32_t disc_count);
    [[nodiscard]] Result<uint32_t> dye_chain_for(uint32_t disc_count);
    [[nodiscard]] Result<physics::airflow::FluidHealth> step_flow(std::span<const physics::airflow::ActuatorDisc> discs);

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
    bool line_on_ = false;
    uint32_t line_pipeline_ = 0;  // airflow_line_sample's pipeline; load and spread follow it
    uint32_t line_elements_ = 0;
    uint32_t line_n_sub_ = 1;
    bool line_tip_ = false;
    uint32_t line_chain_ = 0;     // sample, load and the three spreads, the force read back
    physics::airflow::PolarTable line_polar_{};
    std::vector<physics::airflow::LineReport> line_reports_;
    bool dye_on_ = false;
    uint32_t dye_pipeline_ = 0;  // airflow_dye_advect's pipeline; source and publish follow it
    std::array<std::optional<uint32_t>, kMaxDiscs + 1> dye_chains_{};  // advect (two passes) and the release, by disc count
    bool phase_timing_ = false;
    std::vector<PhaseTime> phases_;            // the last step's, when phase_timing_
    std::array<std::vector<std::string>, kMaxDiscs + 1> phase_names_{};  // the marks' names, by disc count
};

}  // namespace spade::airflow_gpu
