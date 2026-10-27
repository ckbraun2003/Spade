#include "spike/airflow/publish_chain.hpp"

#include <cstddef>
#include <cstring>
#include <span>

#include "bindings.gen.hpp"
#include "physics/airflow/publish.hpp"

namespace spade::airflow_gpu {

namespace af = physics::airflow;

namespace {

// The publish kernels' push: four words, the count in word 2.
struct PublishPush {
    uint32_t unused0 = 0;
    uint32_t unused1 = 0;
    uint32_t cell_count = 0;
    uint32_t unused2 = 0;
};

[[nodiscard]] compute::ChainCommand publish_dispatch(uint32_t pipeline, uint32_t cells, uint32_t workgroup_size) {
    compute::ChainCommand c{.is_dispatch = true, .pipeline = pipeline};
    const PublishPush push{.cell_count = cells};
    std::memcpy(c.push.data(), &push, sizeof push);
    c.groups = {(cells + workgroup_size - 1u) / workgroup_size, 1u, 1u};
    return c;
}

}  // namespace

void append_publish(std::vector<compute::ChainCommand>& out, const PublishPipelines& p, uint32_t cells,
                    uint32_t workgroup_size) {
    out.push_back(publish_dispatch(p.fields, cells, workgroup_size));
    out.push_back(publish_dispatch(p.pack, cells, workgroup_size));
}

Result<void> read_flow_volume(const ChainRunner& runner, const af::GridShape& shape, glm::vec3 origin,
                              uint32_t extrapolated, FlowVolume& out) {
    out.generation = 0;  // unnamed until it is whole
    const std::size_t halves = std::size_t{shape.nx} * shape.ny * shape.nz * kFlowHalvesPerCell;
    out.v.resize(halves);
    out.s.resize(halves);
    if (auto done = runner.read(compute::gen::kBinding_airflow_publish_v, std::as_writable_bytes(std::span(out.v)));
        !done) {
        return done;
    }
    if (auto done = runner.read(compute::gen::kBinding_airflow_publish_s, std::as_writable_bytes(std::span(out.s)));
        !done) {
        return done;
    }
    FlowVolumeHeader& h = out.header;
    h = FlowVolumeHeader{};
    h.origin = origin;
    h.cell = shape.dx;
    h.nx = shape.nx;
    h.ny = shape.ny;
    h.nz = shape.nz;
    if (auto done = af::flow_header_from_halves(out); !done) return done;
    h.extrapolated_cells = extrapolated;
    out.generation = next_flow_generation();  // a new content, for a reader's cache
    return {};
}

}  // namespace spade::airflow_gpu
