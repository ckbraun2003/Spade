#include "render/flow_lines.hpp"

// RED: stubs.

namespace spade::render {

Result<std::vector<Streamline>> trace_streamlines(const FlowVolume&, std::span<const glm::vec3>,
                                                  const StreamlineOptions&) {
    return std::vector<Streamline>{};
}

std::vector<glm::vec3> seed_rake(glm::vec3, glm::vec3, uint32_t) { return {}; }

std::vector<glm::vec3> seed_ring(glm::vec3, glm::vec3, float, uint32_t) { return {}; }

}  // namespace spade::render
