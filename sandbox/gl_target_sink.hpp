// GlTargetSink -- Plan C task C2: the window, and nothing else in it.
//
// ⛔ NO GLFW, NO GL, NO IMGUI TYPE APPEARS BELOW, and that is structural rather
// than tidy. main.cpp must compile on a machine with no GL headers at all, so
// every windowing type lives behind an opaque Impl in the .cpp. The Linux gate
// has no GL headers (measured 2026-09-18: GL_HEADERS_PRESENT=no,
// X11_DEV_HEADERS_PRESENT=no, EGL_HEADERS_PRESENT=no) and it still compiles
// this header.
//
// ⚠⚠ THE REFUSAL IS PART OF THE DELIVERABLE, NOT AN ERROR PATH. RULED.
// create() returns nullptr and writes a reason when this build has no
// windowing backend, or when the window cannot be opened. It NEVER returns a
// sink that quietly draws nowhere. A missing capability must refuse, never
// degrade -- a window that silently fails to appear is the worst outcome for
// the one person who is about to judge whether the GUI works.
//
// ⭐ AND `should_close()` IS HERE RATHER THAN ON `TargetSink`, which is C1's
// ruling and the reason the seam has exactly one method: HeadlessTargetSink has
// no honest answer to it (always-false = run forever, always-true = one frame),
// and an arbitrary value invented to satisfy an interface is how a loop asks
// the wrong object when to stop. Loop termination is the APPLICATION's concern,
// so it is asked of the concrete windowed type.

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "orbit_camera.hpp"
#include "target_sink.hpp"

namespace spade::sandbox {

class GlTargetSink final : public TargetSink {
  public:
    struct Options {
        uint32_t width = 1280;
        uint32_t height = 720;
        std::string title = "spade sandbox";
        bool vsync = true;
        bool show_help_overlay = true;
    };

    // Returns nullptr on refusal, with `why_not` set to a cause a human can
    // act on. Not a bool-out-param: the reason IS the product here, because
    // "no window appeared" without one is indistinguishable from a crash.
    [[nodiscard]] static std::unique_ptr<GlTargetSink> create(const Options& options,
                                                              std::string* why_not);

    // True when this build cannot open a window AT ALL -- reported separately
    // from create() failing, because "this binary was built without a backend"
    // and "the display refused us" need different answers from the person
    // reading the message.
    [[nodiscard]] static bool backend_available() noexcept;

    ~GlTargetSink() override;

    GlTargetSink(const GlTargetSink&) = delete;
    GlTargetSink& operator=(const GlTargetSink&) = delete;

    // THE SEAM. Uploads the frame as a texture, draws it, presents.
    // PA-1 holds: the target's memory is the caller's and is never retained
    // past this call.
    void accept(const spade::render::RenderTarget& target) override;

    // Pumps the OS event queue and reports what the user did since the last
    // call. Reports EVENTS, never camera state -- the shell does not know what
    // a camera is, which is what keeps the logic in OrbitCamera where it can
    // be tested without a display.
    [[nodiscard]] FrameInput poll();

    [[nodiscard]] bool should_close() const noexcept;

    // The framebuffer size in pixels, which is what the render target must
    // match. NOT the window size: on a HiDPI display they differ, and rendering
    // at window size then stretching is exactly the blocky-viewport defect this
    // estate already diagnosed once.
    [[nodiscard]] uint32_t framebuffer_width() const noexcept;
    [[nodiscard]] uint32_t framebuffer_height() const noexcept;

    // Seconds since the previous poll(), for frame-rate-independent movement.
    [[nodiscard]] float delta_seconds() const noexcept;

    // Frames this sink has presented, so the application can assert the loop
    // ran rather than infer it.
    [[nodiscard]] uint64_t presented() const noexcept;

  private:
    struct Impl;
    explicit GlTargetSink(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace spade::sandbox
