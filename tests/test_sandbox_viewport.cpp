// The 3D view renders only between the panels (editor spec §13; plan Task 10).
//
// The dock's central node is in ImGui's display units (window coordinates).
// The 3D view needs it in framebuffer pixels, which differ on a HiDPI display,
// and the cursor relative to it, so a click picks what is under it.

#include <gtest/gtest.h>

#include <limits>

#include <glm/glm.hpp>

#include "../sandbox/viewport_rect.hpp"

namespace {

using spade::sandbox::ViewportRect;
using spade::sandbox::viewport_in_framebuffer;

}  // namespace

TEST(SandboxViewport, TheWholeDisplayIsTheWholeFramebuffer) {
    const ViewportRect r = viewport_in_framebuffer(glm::vec2(0.0f), glm::vec2(1280.0f, 720.0f), glm::vec2(1.0f), 1280u, 720u);
    EXPECT_EQ(r, (ViewportRect{0u, 0u, 1280u, 720u}));
}

TEST(SandboxViewport, AHiDpiScaleMultipliesEveryEdge) {
    const ViewportRect r =
        viewport_in_framebuffer(glm::vec2(256.0f, 24.0f), glm::vec2(768.0f, 600.0f), glm::vec2(2.0f), 2560u, 1440u);
    EXPECT_EQ(r, (ViewportRect{512u, 48u, 1536u, 1200u}));
}

// Each edge rounds on its own, so two neighbours split at a fractional
// boundary share it: no pixel column is lost or drawn twice.
TEST(SandboxViewport, NeighboursAtAFractionalScaleTileWithoutAGap) {
    const glm::vec2 scale(1.25f);
    const ViewportRect left = viewport_in_framebuffer(glm::vec2(0.0f), glm::vec2(301.0f, 400.0f), scale, 1000u, 500u);
    const ViewportRect right =
        viewport_in_framebuffer(glm::vec2(301.0f, 0.0f), glm::vec2(499.0f, 400.0f), scale, 1000u, 500u);
    EXPECT_EQ(left.x + left.width, right.x);
    EXPECT_EQ(right.x + right.width, 1000u);
}

// The node can outlast a resize by a frame; the view never leaves the framebuffer.
TEST(SandboxViewport, ARectPastTheFramebufferIsClamped) {
    const ViewportRect r =
        viewport_in_framebuffer(glm::vec2(200.0f, 50.0f), glm::vec2(2000.0f, 2000.0f), glm::vec2(1.0f), 1280u, 720u);
    EXPECT_EQ(r, (ViewportRect{200u, 50u, 1080u, 670u}));
    const ViewportRect off =
        viewport_in_framebuffer(glm::vec2(-50.0f, -10.0f), glm::vec2(100.0f, 100.0f), glm::vec2(1.0f), 1280u, 720u);
    EXPECT_EQ(off, (ViewportRect{0u, 0u, 50u, 90u}));
}

TEST(SandboxViewport, AnEmptyNodeIsAnEmptyView) {
    const ViewportRect r = viewport_in_framebuffer(glm::vec2(10.0f), glm::vec2(0.0f, 300.0f), glm::vec2(1.0f), 1280u, 720u);
    EXPECT_EQ(r.width, 0u);
    EXPECT_TRUE(r.empty());
    const ViewportRect nan = viewport_in_framebuffer(glm::vec2(10.0f), glm::vec2(std::numeric_limits<float>::quiet_NaN()),
                                                     glm::vec2(1.0f), 1280u, 720u);
    EXPECT_TRUE(nan.empty());
}

TEST(SandboxViewport, TheCursorIsMeasuredFromTheViewsCorner) {
    const ViewportRect r{512u, 48u, 1536u, 1200u};
    EXPECT_EQ(r.relative(glm::vec2(600.0f, 100.0f)), glm::vec2(88.0f, 52.0f));
    EXPECT_TRUE(r.contains(glm::vec2(600.0f, 100.0f)));
    EXPECT_FALSE(r.contains(glm::vec2(500.0f, 100.0f)));    // over the left panel
    EXPECT_FALSE(r.contains(glm::vec2(2048.0f, 100.0f)));   // the right edge is outside
}
