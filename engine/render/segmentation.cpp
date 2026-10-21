#include "render/segmentation.hpp"

#include <algorithm>
#include <cstdlib>
#include <set>

namespace spade::render {

namespace {

// FNV-1a 64 over the name's bytes, then murmur3's finaliser: FNV mixes the
// last bytes weakly into the high bits, and "gate_1" / "gate_2" must not
// share a hue.
[[nodiscard]] uint64_t name_hash(std::string_view name) noexcept {
    uint64_t h = 14695981039346656037ull;
    for (const char c : name) {
        h ^= static_cast<uint8_t>(c);
        h *= 1099511628211ull;
    }
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccdull;
    h ^= h >> 33;
    h *= 0xc4ceb9fe1a85ec53ull;
    h ^= h >> 33;
    return h;
}

}  // namespace

Rgb8 segment_colour(std::string_view name) noexcept {
    const uint64_t h = name_hash(name);
    // A hue from 24 bits, and one of four saturation-value pairs from two
    // more. The least saturated, darkest pair keeps every channel at 27 levels
    // or more (never black) and two channels at least 98 apart (never grey).
    const float hue = static_cast<float>(h & 0xFFFFFFu) * (6.0f / 16777216.0f);  // [0, 6)
    const uint32_t step = static_cast<uint32_t>((h >> 24) & 3u);
    const float sat = (step & 1u) != 0u ? 0.55f : 0.85f;
    const float val = (step & 2u) != 0u ? 0.70f : 0.95f;
    const int sector = std::min(static_cast<int>(hue), 5);
    const float f = hue - static_cast<float>(sector);
    const float p = val * (1.0f - sat);
    const float q = val * (1.0f - sat * f);
    const float t = val * (1.0f - sat * (1.0f - f));
    glm::vec3 rgb;
    switch (sector) {
        case 0: rgb = {val, t, p}; break;
        case 1: rgb = {q, val, p}; break;
        case 2: rgb = {p, val, t}; break;
        case 3: rgb = {p, q, val}; break;
        case 4: rgb = {t, p, val}; break;
        default: rgb = {val, p, q}; break;
    }
    return {to_byte(rgb.r), to_byte(rgb.g), to_byte(rgb.b)};
}

uint32_t segment_label(const DrawItem& item, SegmentBy by) noexcept {
    return by == SegmentBy::parts && item.part_label != kNoLabel ? item.part_label : item.object_label;
}

Rgb8 item_segment_colour(const RenderScene& scene, const DrawItem& item, SegmentBy by) {
    const uint32_t label = segment_label(item, by);
    return label < scene.labels.size() ? segment_colour(scene.labels[label]) : kUnlabelledColour;
}

Result<void> validate_labels(const RenderScene& scene) {
    const auto check = [&scene](uint32_t label, const std::string& what) -> Result<void> {
        if (label == kNoLabel || label < scene.labels.size()) return {};
        return std::unexpected(Error{Code::invalid_argument, what + " label " + std::to_string(label) +
                                                                 " is past the scene's " +
                                                                 std::to_string(scene.labels.size()) + " labels"});
    };
    const auto items = [&check](const std::vector<DrawItem>& list, const char* kind) -> Result<void> {
        for (size_t i = 0; i < list.size(); ++i) {
            const std::string item = std::string(kind) + " item " + std::to_string(i) + "'s";
            if (auto ok = check(list[i].object_label, item + " object"); !ok) return ok;
            if (auto ok = check(list[i].part_label, item + " part"); !ok) return ok;
        }
        return {};
    };
    if (auto ok = items(scene.statics, "static"); !ok) return ok;
    if (auto ok = items(scene.dynamics, "dynamic"); !ok) return ok;
    for (size_t i = 0; i < scene.ground_planes.size(); ++i) {
        if (auto ok = check(scene.ground_planes[i].label, "ground plane " + std::to_string(i) + "'s"); !ok) return ok;
    }
    return {};
}

SegmentLegend segment_legend(const RenderScene& scene, SegmentBy by) {
    SegmentLegend legend;
    std::set<std::string> names;
    const auto use = [&](uint32_t label) {
        if (label < scene.labels.size()) {
            names.insert(scene.labels[label]);
        } else {
            ++legend.unlabelled_items;
        }
    };
    for (const DrawItem& item : scene.statics) use(segment_label(item, by));
    for (const DrawItem& item : scene.dynamics) use(segment_label(item, by));
    for (const GroundPlane& plane : scene.ground_planes) use(plane.label);
    for (const std::string& name : names) legend.entries.push_back(SegmentEntry{name, segment_colour(name)});
    for (size_t i = 0; i < legend.entries.size(); ++i) {
        for (size_t j = i + 1; j < legend.entries.size(); ++j) {
            const Rgb8& a = legend.entries[i].colour;
            const Rgb8& b = legend.entries[j].colour;
            int worst = 0;
            for (int k = 0; k < 3; ++k) worst = std::max(worst, std::abs(int(a[k]) - int(b[k])));
            if (worst <= kSegmentClashLevels) {
                legend.clashes.push_back(SegmentClash{legend.entries[i].name, legend.entries[j].name});
            }
        }
    }
    return legend;
}

}  // namespace spade::render
