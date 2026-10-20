// ---------------------------------------------------------------------------
// render/segmentation.hpp -- what DrawMode::segmentation colours an item
// (N13; docs/design/rendering/plans/2026-10-20-sensor-views-design.md).
//
// Each item draws flat in its label's colour, and the colour comes from the
// label's NAME (the lead, 2026-10-20): the same name is the same colour in
// every frame, run and backend, whatever the draw order and whatever else
// the scene holds, so adding an object recolours nothing. Unique names make
// instance segmentation, a shared name semantic: the producer's choice.
//
// Never silent: an item with no label draws grey (kUnlabelledColour, which
// no name's colour is), and two names whose colours are within
// kSegmentClashLevels are listed as a clash in the legend.
// ---------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "render/scene.hpp"

namespace spade::render {

using Rgb8 = std::array<uint8_t, 3>;

// A name's colour: never black ("no surface") and never grey (unlabelled).
[[nodiscard]] Rgb8 segment_colour(std::string_view name) noexcept;

inline constexpr Rgb8 kUnlabelledColour{128, 128, 128};

// Two names clash when no channel of their colours differs by more than this.
inline constexpr int kSegmentClashLevels = 8;

// The label an item draws by: its part's under SegmentBy::parts when it has
// one, else its object's; kNoLabel for none.
[[nodiscard]] uint32_t segment_label(const DrawItem& item, SegmentBy by) noexcept;

// The colour an item draws in: its label's, or kUnlabelledColour.
[[nodiscard]] Rgb8 item_segment_colour(const RenderScene& scene, const DrawItem& item, SegmentBy by);

// Every label index in `scene` (items, ground planes) names a label, or the
// scene is refused, naming the item.
[[nodiscard]] Result<void> validate_labels(const RenderScene& scene);

struct SegmentEntry {
    std::string name;
    Rgb8 colour{};
};

struct SegmentClash {
    std::string a, b;  // a < b
};

struct SegmentLegend {
    std::vector<SegmentEntry> entries;   // every label a drawn item or ground uses, by name
    uint32_t unlabelled_items = 0;       // items drawn grey
    std::vector<SegmentClash> clashes;   // pairs of entries whose colours clash
};

[[nodiscard]] SegmentLegend segment_legend(const RenderScene& scene, SegmentBy by);

}  // namespace spade::render
