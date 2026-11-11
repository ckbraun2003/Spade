// The subgrid viscosity's account of a LiveRun, from the other side: a LiveFluid that wraps another and keeps the
// FluidHealth it hands LiveRun each step, so a test can compare what LiveRun's sgs() says with the sums of what the
// fluid reported (the oracle is the fluid's own health, not LiveRun's code). It can also set the config's Cs for the
// fluid it wraps (as a host does, tools/flow_movie/main.cpp) and rewrite the sgs fields from a script, so the
// accounting is exact whatever the flow binds.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include "analysis/live_run.hpp"
#include "physics/airflow/fluid_step.hpp"

namespace spade::test {

struct SgsSums {
    uint64_t steps = 0;
    uint64_t capped_cell_steps = 0;
    uint64_t limited_face_steps = 0;
    uint64_t capped_steps = 0;
    uint64_t limited_steps = 0;
    float nu_t_max = 0.0f;
};

// One step's sgs account, as a script gives it.
struct ScriptedSgs {
    uint32_t capped_cells = 0;
    uint32_t limited_faces = 0;
    float nu_t_max = 0.0f;
};

class SgsProbe final : public analysis::LiveFluid {
  public:
    SgsProbe(std::unique_ptr<analysis::LiveFluid> inner, SgsSums& sums, std::vector<ScriptedSgs> script)
        : inner_(std::move(inner)), sums_(sums), script_(std::move(script)) {}

    [[nodiscard]] Result<physics::airflow::FluidHealth> step(std::span<const physics::airflow::ActuatorDisc> discs) override {
        Result<physics::airflow::FluidHealth> health = inner_->step(discs);
        if (!health) return health;
        if (!script_.empty()) {  // the script's step i, repeating
            const ScriptedSgs& s = script_[static_cast<std::size_t>(sums_.steps) % script_.size()];
            health->sgs_capped_cells = s.capped_cells;
            health->sgs_limited_faces = s.limited_faces;
            health->sgs_nu_t_max = s.nu_t_max;
        }
        ++sums_.steps;
        sums_.capped_cell_steps += health->sgs_capped_cells;
        sums_.limited_face_steps += health->sgs_limited_faces;
        if (health->sgs_capped_cells > 0) ++sums_.capped_steps;
        if (health->sgs_limited_faces > 0) ++sums_.limited_steps;
        if (health->sgs_nu_t_max > sums_.nu_t_max) sums_.nu_t_max = health->sgs_nu_t_max;
        return health;
    }

    [[nodiscard]] Result<void> publish(const physics::airflow::PublishInput& in, FlowVolume& out) override {
        return inner_->publish(in, out);
    }

    [[nodiscard]] Result<void> read(physics::airflow::MacGrid& g) const override { return inner_->read(g); }

  private:
    std::unique_ptr<analysis::LiveFluid> inner_;
    SgsSums& sums_;
    std::vector<ScriptedSgs> script_;
};

// `inner` with the subgrid viscosity at `cs` (0 leaves the run's config as it is), recorded into `sums`, the sgs
// fields rewritten by `script` when it has any.
[[nodiscard]] inline analysis::LiveFluidMaker sgs_probe(analysis::LiveFluidMaker inner, float cs, SgsSums& sums,
                                                        std::vector<ScriptedSgs> script = {}) {
    return [inner = std::move(inner), cs, &sums, script = std::move(script)](
               const analysis::LiveFluidStart& s) -> Result<std::unique_ptr<analysis::LiveFluid>> {
        physics::airflow::FluidConfig config = s.config;
        if (cs > 0.0f) config.sgs_cs = cs;
        Result<std::unique_ptr<analysis::LiveFluid>> fluid =
            inner(analysis::LiveFluidStart{s.grid, s.domain, config, s.solids});
        if (!fluid) return fluid;
        return std::unique_ptr<analysis::LiveFluid>(new SgsProbe(std::move(*fluid), sums, script));
    };
}

}  // namespace spade::test
