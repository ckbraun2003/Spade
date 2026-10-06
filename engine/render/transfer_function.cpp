#include "render/transfer_function.hpp"

// RED: stubs; the tables and the logic land in the green commit.

namespace spade::render {

const std::array<glm::vec3, kColourMapEntries>& colour_map_table(ColourMapId) {
    static const std::array<glm::vec3, kColourMapEntries> zeros{};
    return zeros;
}

glm::vec3 colour_map_at(ColourMapId, float) { return glm::vec3(0.0f); }

Result<void> validate_transfer_function(const TransferFunction&) { return {}; }

ResolvedRange resolve_range(const TransferFunction&, const FlowVolumeHeader&) { return {}; }

PreparedTransfer prepare_transfer(const TransferFunction& tf, const FlowVolumeHeader&) {
    return PreparedTransfer{tf, {}, 0.0f};
}

TransferSample apply_transfer(const PreparedTransfer&, float) { return {}; }

}  // namespace spade::render
