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

// Forward-declared rather than included: this header does not touch a single
// member of it, so including builder_scene.hpp here would pull the whole
// object model (and glm's matrix headers) into every translation unit that
// merely wants to open a window.
struct BuilderScene;
struct DronePanelModel;

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

    // ---------------------------------------------------------------------
    // WHERE THE FRAME WENT. Not a throwaway probe -- this is on the HUD.
    //
    // ⭐ A FRAME RATE IS A SUM AND A SUM IS NOT A DIAGNOSIS. "9 fps" is
    // consistent with a slow rasteriser, a per-frame texture reallocation, an
    // expensive overlay, and with a swap that is simply WAITING for the
    // display -- and those have opposite remedies. One of them is not even a
    // cost. So the window reports the SPLIT and not only the total, which
    // answers the question permanently rather than once.
    //
    // ⚠ swap_ms IS THE ONE TO READ FIRST, and it is the one that can mislead:
    // with vsync on, glfwSwapBuffers BLOCKS until the display is ready, so a
    // large swap_ms means the frame finished EARLY and waited. That is the
    // opposite of a problem, and it looks identical to a slow present.
    // --no-vsync is the control that tells them apart.
    // ---------------------------------------------------------------------
    struct Timings {
        // ⚠ physics_ms EXISTS BECAUSE ITS ABSENCE WAS NOT AN OMISSION, IT WAS
        // AN ABSENCE: until the sandbox could hold bodies, there was no
        // physics step in the frame to time. A split with no entry for the
        // thing a user complains about cannot attribute their complaint.
        float physics_ms = 0.0f;  // Simulation::step + pose sync, the application's
        float render_ms = 0.0f;   // spade::render -- the application's, handed in
        float convert_ms = 0.0f;  // BGRX -> RGBA, CPU, per pixel
        float upload_ms = 0.0f;   // glTexImage2D
        float ui_ms = 0.0f;       // clear + ImGui build and draw
        float swap_ms = 0.0f;     // glfwSwapBuffers (BLOCKS under vsync)
        float total_ms = 0.0f;
    };

    // The application owns the render call, so it hands its own cost in. Read
    // back on the NEXT accept(), which is why the HUD's render figure is one
    // frame behind everything else -- stated rather than hidden.
    void note_render_ms(float ms) noexcept;
    void note_physics_ms(float ms) noexcept;

    // Rolling average over the recent window, which is what the HUD shows: a
    // per-frame readout jitters far too much to read.
    [[nodiscard]] Timings average_timings() const noexcept;

    // Cumulative means over every frame this sink presented, for a summary
    // that does not depend on when someone happened to look.
    [[nodiscard]] Timings lifetime_timings() const noexcept;

    // WHICH GL IMPLEMENTATION THIS WINDOW GOT. Not cosmetic: a software
    // implementation makes the present a per-pixel CPU cost, which changes the
    // reading of every other number here. Empty strings if unavailable, never
    // a placeholder -- a plausible-looking wrong answer is worse than a blank.
    struct GlInfo {
        std::string renderer;
        std::string version;
        std::string vendor;
    };
    [[nodiscard]] GlInfo gl_info() const;

    // ---------------------------------------------------------------------
    // THE GPU PATH. accept() above is the CPU FALLBACK -- render into a buffer,
    // convert, upload as a texture. These three are the primary path: the GPU
    // renderer draws straight into this window's framebuffer and never makes a
    // pixel touch the CPU at all.
    //
    // ⭐ BOTH PATHS STAY LIVE AND THAT IS THE USER'S RULING, NOT A HEDGE:
    // "rendering should default to using the computer gpu... the cpu is the
    // fallback." A fallback that is never exercised is not a fallback.
    // ---------------------------------------------------------------------

    // GL entry-point loader for this window's context. Handed out so the
    // renderer can load GL without this sandbox linking it to GLFW, and
    // without the renderer knowing what a window is.
    [[nodiscard]] static void* (*proc_loader())(const char*);

    // Clear and size the framebuffer for a GPU frame. The renderer clears
    // depth itself; colour belongs to whoever owns the window.
    void begin_gpu_frame();

    // The overlay and the present, for a frame whose 3D was drawn by someone
    // else. Same ImGui pass and the same timing accounting as accept(), so the
    // HUD reads identically on both paths -- which is what makes them
    // comparable at all.
    //
    void present_overlay(float render_ms, float physics_ms);

    // The HUD line that names what the active path does not draw
    // (render_gap.hpp), or "" for none. Held until replaced, so the
    // application sets it whenever the path or the options can change.
    void set_gap_line(std::string line);

    // Attach the builder's model, or nullptr to detach. While attached, the
    // hierarchy and inspector are drawn by WHICHEVER path presents the frame
    // and written back into.
    //
    // ⭐ ATTACHED ONCE RATHER THAN PASSED PER PRESENT, SO BOTH PATHS GET IT.
    // The GPU path presents through present_overlay() and the CPU fallback
    // through accept(); a parameter on one of them gives the builder to one
    // path and silently withholds it from the other. That is the same defect
    // as two copies of the HUD -- which this file already refused once -- with
    // the copies replaced by an absence, which is harder to notice.
    //
    // WHY THE MODEL COMES HERE INSTEAD OF THE PANEL LIVING IN THE APPLICATION:
    // ImGui is this file's dependency and nobody else's. Letting main.cpp draw
    // its own widgets would put imgui.h in the application, which SL2b forbids
    // and which would spread the one dependency that has to stay contained.
    // BuilderScene is a pure struct with no UI in it, so it crosses this
    // header freely -- the model travels, the toolkit does not.
    void attach_builder(BuilderScene* model) noexcept;

    // The drone sim box's physics panel, attached the same way and for the
    // same reasons as the builder. The panel only writes the model; what an
    // edit DOES is decided by drone_view.hpp's apply_panel_edits.
    void attach_drone(DronePanelModel* model) noexcept;

    // Process working set in bytes, 0 when unavailable. The user asked for
    // this beside the fps.
    [[nodiscard]] static uint64_t working_set_bytes() noexcept;

    // Frames this sink has presented, so the application can assert the loop
    // ran rather than infer it.
    [[nodiscard]] uint64_t presented() const noexcept;

  private:
    // ONE STATEMENT OF THE HUD, called by BOTH paths. The CPU fallback and the
    // GPU primary must show the same panel or the two are not comparable --
    // and two copies of a panel is the same defect as two copies of a channel
    // reorder, which this file already refused once.
    void draw_overlay();

    // The builder's hierarchy + inspector. Separate from draw_overlay()
    // because the HUD must appear whether or not a builder model exists --
    // the headless and CPU-fallback paths have no builder and still need
    // their numbers.
    void draw_builder_panel(BuilderScene& model);

    void draw_drone_panel(DronePanelModel& model);

    struct Impl;
    explicit GlTargetSink(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace spade::sandbox
