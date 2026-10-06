#include "render/flow_sample.hpp"

// RED: stubs.

namespace spade::render {

FlowSampler make_flow_sampler(const FlowVolume& volume) { return FlowSampler{&volume, {}, 0.0f}; }

FlowValue sample_channel(const FlowSampler&, uint32_t, glm::vec3) { return {}; }

FlowValue sample_quantity(const FlowSampler&, FlowQuantity, glm::vec3) { return {}; }

FlowVelocity sample_velocity(const FlowSampler&, glm::vec3) { return {}; }

}  // namespace spade::render
