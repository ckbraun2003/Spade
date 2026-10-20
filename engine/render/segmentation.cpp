#include "render/segmentation.hpp"

namespace spade::render {

// RED: stubs until the green step.
Rgb8 segment_colour(std::string_view) noexcept { return kUnlabelledColour; }

uint32_t segment_label(const DrawItem&, SegmentBy) noexcept { return kNoLabel; }

Rgb8 item_segment_colour(const RenderScene&, const DrawItem&, SegmentBy) { return kUnlabelledColour; }

Result<void> validate_labels(const RenderScene&) { return {}; }

SegmentLegend segment_legend(const RenderScene&, SegmentBy) { return {}; }

}  // namespace spade::render
