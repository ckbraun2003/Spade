#include "render/flow_look.hpp"

namespace spade::render {

// RED: the interface only; the tests fail until the green step.
TransferFunction volume_transfer(FlowQuantity, ColourMapId, float, float, float) { return TransferFunction{}; }

TransferFunction colours_over(FlowQuantity, ColourMapId, float, float) { return TransferFunction{}; }

Result<void> add_streamlines(RenderScene&, const FlowVolume&, std::span<const glm::vec3>, const StreamlineOptions&,
                             const TransferFunction&, const glm::vec3*) {
    return std::unexpected(Error{Code::unavailable, "add_streamlines: not built yet"});
}

Result<void> add_slice(RenderScene&, const FlowVolume&, FlowQuantity, FieldLayer, ColourMapId, float, float) {
    return std::unexpected(Error{Code::unavailable, "add_slice: not built yet"});
}

FlowPolyline circle(glm::vec3, glm::vec3, float, glm::vec3, uint32_t) { return FlowPolyline{}; }

}  // namespace spade::render
