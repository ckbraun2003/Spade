#pragma once

// ---------------------------------------------------------------------------
// publish_chain.hpp (plan B, task B6) -- publish on the GPU, for the chain
// runner: publish_render_volume()'s halves packed on the device
// (airflow_publish_fields, then airflow_pack_f16) and read back as a whole
// spade::FlowVolume. The CPU twin, and the authority for every half and the
// header, is engine/physics/airflow/publish.cpp.
//
// P8'S RULE kept the device pack: at the D1 box the halves cross in about
// 13 ms against 348 ms for the fp32 channels read back and packed on the CPU
// (tests/test_gpu_airflow_publish.cpp, P8ReadbackCostAtTheD1Box).
//
// THE HEADER comes from the halves by publish's own rule
// (physics::airflow::flow_header_from_halves()), and the extrapolated cells
// from the solid mask by publish's own neighbour walk
// (physics::airflow::extrapolated_cells()), which the caller counts once per
// mask: the solids are static for a run (Physics, 2026-10-26), and a body
// that moves rebuilds its mask, and so the count, never per step.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "compute/vulkan/dispatch_chain.hpp"
#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "spike/airflow/chain_runner.hpp"

namespace spade::airflow_gpu {

// The publish kernels' pipelines in the runner (RunnerDesc::kernels).
struct PublishPipelines {
    uint32_t fields = 0;
    uint32_t pack = 0;
};

// publish_fields then pack_f16, one invocation a cell, both pushes {0, 0,
// cells, 0} by the airflow push convention. The fields write their fp32
// channels into airflow_advect, the advection's scratch, which is dead
// between the step's chain and this one.
void append_publish(std::vector<compute::ChainCommand>& out, const PublishPipelines& p, uint32_t cells,
                    uint32_t workgroup_size);

// The volume a submitted publish chain left on the device: the halves read
// from airflow_publish_v and _s (section F's bindings, the runner's
// binding_base 0), the header from them by flow_header_from_halves(), the
// box's dims and cell from `shape`, `origin` (the grid's minimum corner in
// the world), `extrapolated` (extrapolated_cells()), and a new generation.
// `out`'s storage is reused when its size fits; a refusal leaves its
// generation 0.
[[nodiscard]] Result<void> read_flow_volume(const ChainRunner& runner, const physics::airflow::GridShape& shape,
                                            glm::vec3 origin, uint32_t extrapolated, FlowVolume& out);

}  // namespace spade::airflow_gpu
