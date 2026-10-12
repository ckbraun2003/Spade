// Where the 3D view goes: the dock's pass-through centre, in framebuffer
// pixels. Display-free, so the arithmetic is tested without a window.
//
// The panels dock at the sides (editor spec §13) and the scene renders only
// between them, as Unity's and Unreal's viewports do. ImGui reports the
// central node in display units (window coordinates); a HiDPI framebuffer has
// more pixels than that, so each edge is scaled by the display's framebuffer
// scale. Each edge rounds to the nearest pixel ON ITS OWN, not the width, so
// two neighbours split at a fractional boundary share it and no column is lost
// or drawn twice. Edges clamp to the framebuffer: the node can lag a resize by
// a frame.

#pragma once

#include <cmath>
#include <cstdint>

#include <glm/glm.hpp>

namespace spade::sandbox {

struct ViewportRect {
    uint32_t x = 0;  // framebuffer pixels, origin top-left, +y down
    uint32_t y = 0;
    uint32_t width = 0;
    uint32_t height = 0;

    [[nodiscard]] bool empty() const noexcept { return width == 0u || height == 0u; }

    // A point in framebuffer pixels, measured from the view's top-left corner.
    [[nodiscard]] glm::vec2 relative(glm::vec2 framebuffer_point) const noexcept {
        return framebuffer_point - glm::vec2(static_cast<float>(x), static_cast<float>(y));
    }

    // Whether a framebuffer point is over the view rather than a panel.
    [[nodiscard]] bool contains(glm::vec2 framebuffer_point) const noexcept {
        const glm::vec2 p = relative(framebuffer_point);
        return p.x >= 0.0f && p.y >= 0.0f && p.x < static_cast<float>(width) && p.y < static_cast<float>(height);
    }

    friend bool operator==(const ViewportRect&, const ViewportRect&) = default;
};

namespace detail {

// One edge in framebuffer pixels: scaled, rounded, clamped to [0, limit]. A
// NaN edge is 0, so a NaN node comes out empty.
[[nodiscard]] inline uint32_t framebuffer_edge(float display, float scale, uint32_t limit) noexcept {
    const float px = std::round(display * scale);
    if (!(px > 0.0f)) return 0u;
    if (px >= static_cast<float>(limit)) return limit;
    return static_cast<uint32_t>(px);
}

}  // namespace detail

// The node at `pos` with `size` (display units), on a framebuffer of
// fb_width x fb_height pixels whose scale per display unit is `scale`.
[[nodiscard]] inline ViewportRect viewport_in_framebuffer(glm::vec2 pos, glm::vec2 size, glm::vec2 scale,
                                                          uint32_t fb_width, uint32_t fb_height) noexcept {
    const uint32_t x0 = detail::framebuffer_edge(pos.x, scale.x, fb_width);
    const uint32_t y0 = detail::framebuffer_edge(pos.y, scale.y, fb_height);
    const uint32_t x1 = detail::framebuffer_edge(pos.x + size.x, scale.x, fb_width);
    const uint32_t y1 = detail::framebuffer_edge(pos.y + size.y, scale.y, fb_height);
    if (x1 <= x0 || y1 <= y0) return ViewportRect{x0, y0, 0u, 0u};
    return ViewportRect{x0, y0, x1 - x0, y1 - y0};
}

}  // namespace spade::sandbox
