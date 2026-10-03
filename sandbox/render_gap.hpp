// ---------------------------------------------------------------------------
// render_gap.hpp -- the HUD line that names what the GPU path does not draw.
//
// SL10: the CPU path is the reference, and a gap in the GPU path is shown, not
// papered over. GlRenderer::unhonoured(options) names the RenderOptions it
// will not draw for a frame; this turns that list into the one HUD line the
// window shows while GL is the active path. Display-free, so spade_tests
// checks the text with no window (SL15b).
// ---------------------------------------------------------------------------
#pragma once

#include <span>
#include <string>
#include <string_view>

namespace spade::sandbox {

// "GL does not draw: shadows, overlays", or "" when the list is empty, so a
// path with no gap draws no line at all.
[[nodiscard]] inline std::string render_gap_line(std::span<const std::string_view> unhonoured) {
    if (unhonoured.empty()) {
        return {};
    }
    std::string line = "GL does not draw: ";
    for (std::size_t i = 0; i < unhonoured.size(); ++i) {
        if (i != 0) {
            line += ", ";
        }
        line += unhonoured[i];
    }
    return line;
}

}  // namespace spade::sandbox
