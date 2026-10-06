#include "render/flow_render.hpp"

// RED: stubs.

namespace spade::render {

Result<void> validate_flow_layer(const FlowLayer&) { return {}; }

PreparedFlowLayer prepare_flow_layer(const FlowLayer&) { return {}; }

FlowRayResult march_flow_ray(const PreparedFlowLayer&, glm::vec3, glm::vec3, float, float) { return {}; }

}  // namespace spade::render
