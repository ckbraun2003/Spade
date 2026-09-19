// GlTargetSink -- Plan C task C2. See the header for why nothing windowing
// appears there.
//
// ⭐⭐ THIS FILE COMPILES ON EVERY PLATFORM, INCLUDING ONE WITH NO GL HEADERS.
// SPADE_SANDBOX_HAS_GL guards only the code that TOUCHES GLFW, GL or ImGui.
// The conversion, the frame budget and the refusal path are outside the guard
// on purpose: that is the mitigation the plan names for the windowed sink
// getting no Linux compile coverage. Linux keeps coverage of the logic and
// loses only the calls.
//
// ⚠ AND WHEN THE GUARD IS OFF, CMake LINKS NEITHER GLFW NOR IMGUI into this
// target. That is a decision, not an accident: ImGui's OpenGL3 backend needs GL
// symbols at LINK time even when no GL call of ours is reached, so a
// no-backend Linux link would fail for a window nobody asked it to open. The
// guard removes the references and the link stays clean.
//
// 🔬 HOW THIS REACHES GL, and the answer came from the library rather than from
// preference. ImGui's bundled imgui_impl_opengl3_loader.h resolves
// glGenTextures/glBindTexture/glTexImage2D/glTexParameteri/GL_RGBA/GL_LINEAR
// -- and NOT GL_BGRA, GL_RGB, GL_NEAREST or glTexSubImage2D, because it is
// filtered to what ImGui itself uses. Its header also says, in capitals, that
// it must not be included directly. So the sink uses the SYSTEM GL header, and
// every symbol it needs is GL 1.1, which Windows exports from opengl32 and
// every Linux GL implementation has had since 1997.
//
// Consequences, both measured rather than assumed:
//   * no GL_BGRA  -> the BGRX->RGBA swap is ours; it is bgrx_to_channels<4>,
//                    the SAME statement HeadlessTargetSink uses with <3>.
//   * no glTexSubImage2D in the loader -> glTexImage2D per frame. At sandbox
//                    resolutions the reallocation is free, and the system
//                    header does have glTexSubImage2D, but using the narrower
//                    call keeps this path identical to the one that was probed.

#include "gl_target_sink.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

// ⚠ OUTSIDE THE GL GUARD ON PURPOSE. The user asked for a memory figure, and
// reading it must not depend on whether this build can open a window --
// working_set_bytes() is declared unconditionally in the header, so it has to
// link unconditionally too.
//
// NOMINMAX and WIN32_LEAN_AND_MEAN are set target-scoped in CMake, for the
// reason spade/CMakeLists.txt already records against v1: windows.h's
// unguarded min/max macros shadow GLM's quaternion_exponential.inl.
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

#if SPADE_SANDBOX_HAS_GL
#include <GLFW/glfw3.h>  // GLFW_INCLUDE_NONE is set on this target, so no GL arrives with it
#include <GL/gl.h>

// ⚠ GL_CLAMP_TO_EDGE IS GL 1.2, AND WINDOWS SHIPS A GL 1.1 <GL/gl.h>. The
// constant is simply absent there, so this is a compile error on the platform
// the window is most likely to run on -- not a portability nicety. The value is
// the one from the GL registry and has never changed; defining it when the
// header does not is the standard way to use a post-1.1 enum against Microsoft's
// header without pulling in a loader.
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#endif  // SPADE_SANDBOX_HAS_GL

namespace spade::sandbox {
namespace {

using Clock = std::chrono::steady_clock;

// Timing accumulators. Defined HERE rather than beside the accessors that use
// them, because accept() calls them and sits earlier in the file -- C++ needs
// the declaration first, and the build is the only thing that would have told
// me. A reading pass caught it; the compiler would have, a slot later.
void accumulate(GlTargetSink::Timings& into, const GlTargetSink::Timings& t) {
    into.render_ms += t.render_ms;
    into.convert_ms += t.convert_ms;
    into.upload_ms += t.upload_ms;
    into.ui_ms += t.ui_ms;
    into.swap_ms += t.swap_ms;
    into.total_ms += t.total_ms;
}
[[nodiscard]] GlTargetSink::Timings scale(const GlTargetSink::Timings& t, float k) {
    GlTargetSink::Timings o;
    o.render_ms = t.render_ms * k;
    o.convert_ms = t.convert_ms * k;
    o.upload_ms = t.upload_ms * k;
    o.ui_ms = t.ui_ms * k;
    o.swap_ms = t.swap_ms * k;
    o.total_ms = t.total_ms * k;
    return o;
}

#if SPADE_SANDBOX_HAS_GL
// GLFW reports errors through a callback and otherwise returns a bare false,
// so without this a refusal says "could not create window" and nothing about
// why. The message is what the user acts on.
std::string g_last_glfw_error;

void glfw_error_callback(int code, const char* description) {
    g_last_glfw_error = "GLFW error " + std::to_string(code) + ": " +
                        (description != nullptr ? description : "(no description)");
}

// ImTextureID is void* in the vendored pin (v1.91.5-docking) and an integer
// handle in some builds. A C-style cast is the one spelling that is correct
// for both, which is why it is here rather than a static_cast.
ImTextureID as_texture_id(GLuint t) { return (ImTextureID)(std::uintptr_t)t; }
#endif

}  // namespace

// ---------------------------------------------------------------------------
// Impl -- every windowing type lives here and nowhere else.
// ---------------------------------------------------------------------------
struct GlTargetSink::Impl {
    GlTargetSink::Options options;

    // Outside the GL guard on purpose: the conversion buffer and the frame
    // clock are the logic that must keep compiling where there is no GL.
    std::vector<uint8_t> rgba;
    Clock::time_point last_frame = Clock::now();
    float dt = 0.0f;
    uint64_t presented = 0;
    uint32_t fb_width = 0;
    uint32_t fb_height = 0;

    // Outside the GL guard with the rest of the logic. A WINDOWED mean for the
    // HUD (a per-frame readout is unreadable) and a LIFETIME mean for the exit
    // summary, so the number does not depend on when somebody happened to look.
    float pending_render_ms = 0.0f;
    GlTargetSink::Timings win_sum;
    uint32_t win_count = 0;
    GlTargetSink::Timings win_avg;
    GlTargetSink::Timings life_sum;

#if SPADE_SANDBOX_HAS_GL
    GLFWwindow* window = nullptr;
    GLuint texture = 0;
    uint32_t texture_width = 0;
    uint32_t texture_height = 0;
    bool imgui_ready = false;
    bool dragging = false;
    double last_cursor_x = 0.0;
    double last_cursor_y = 0.0;
    float scroll_accum = 0.0f;
    bool show_help = true;
    bool f1_was_down = false;

    ~Impl() {
        // Teardown in creation-reverse order, and each step guarded by the flag
        // that records whether it actually happened. A create() that failed
        // halfway must not tear down what it never built.
        if (imgui_ready) {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
        }
        if (texture != 0) {
            glDeleteTextures(1, &texture);
        }
        if (window != nullptr) {
            glfwDestroyWindow(window);
        }
        glfwTerminate();
    }
#endif
};

bool GlTargetSink::backend_available() noexcept {
#if SPADE_SANDBOX_HAS_GL
    return true;
#else
    return false;
#endif
}

std::unique_ptr<GlTargetSink> GlTargetSink::create(const Options& options, std::string* why_not) {
    const auto refuse = [why_not](std::string reason) -> std::unique_ptr<GlTargetSink> {
        if (why_not != nullptr) {
            *why_not = std::move(reason);
        }
        return nullptr;
    };

#if !SPADE_SANDBOX_HAS_GL
    // ⚠ THE REFUSAL NAMES THE CAUSE AND THE FIX. "No window" on its own sends
    // the reader to look for a crash; this sends them to the configure line
    // that produced the binary they are holding.
    return refuse(
        "this build has no windowing backend. GLFW was configured without one because no X11 "
        "development headers were found at configure time (SPADE_GLFW_HAS_BACKEND=0), so a window "
        "cannot be opened by this binary at all. Use --headless, or reconfigure on a machine with "
        "X11 development packages present.");
#else
    if (options.width == 0u || options.height == 0u) {
        return refuse("window width and height must both be non-zero");
    }

    auto impl = std::make_unique<Impl>();
    impl->options = options;
    impl->show_help = options.show_help_overlay;

    glfwSetErrorCallback(glfw_error_callback);
    g_last_glfw_error.clear();
    if (glfwInit() != GLFW_TRUE) {
        // The most likely real-world instance: a backend was compiled in, but
        // there is no display to talk to (ssh with no X forwarding, a service
        // account, a CI runner). That is a DIFFERENT failure from having no
        // backend compiled in, and the two need different answers.
        return refuse("GLFW could not initialise, which usually means there is no display available "
                      "to this process. " +
                      (g_last_glfw_error.empty() ? std::string("(GLFW reported no detail.)")
                                                 : g_last_glfw_error));
    }

    // 3.3 core is what ImGui's GLSL 330 backend expects. Requesting it up front
    // makes an unsupported driver fail HERE with a clear message, rather than
    // later inside ImGui's shader compile where the error is a log line nobody
    // is reading.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    impl->window = glfwCreateWindow(static_cast<int>(options.width), static_cast<int>(options.height),
                                    options.title.c_str(), nullptr, nullptr);
    if (impl->window == nullptr) {
        return refuse("could not create a window. " +
                      (g_last_glfw_error.empty()
                           ? std::string("GLFW reported no detail; an OpenGL 3.3 core context may "
                                         "not be available on this driver.")
                           : g_last_glfw_error));
    }

    glfwMakeContextCurrent(impl->window);
    glfwSwapInterval(options.vsync ? 1 : 0);
    glfwSetWindowUserPointer(impl->window, impl.get());
    glfwSetScrollCallback(impl->window, [](GLFWwindow* w, double, double yoffset) {
        // Accumulated rather than read live: a scroll can arrive several times
        // between two polls, and sampling the last one loses the rest.
        auto* self = static_cast<Impl*>(glfwGetWindowUserPointer(w));
        if (self != nullptr) {
            self->scroll_accum += static_cast<float>(yoffset);
        }
    });

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    // ⛔ NO imgui.ini. ImGui persists window layout to the PROCESS'S WORKING
    // DIRECTORY by default, which for a developer tool launched from a repo
    // root means dropping an untracked file into a SHARED CHECKOUT -- it
    // appeared in every session's `git status` the first time this window ran,
    // which is how I found it.
    //
    // ⭐ Disabled rather than gitignored: a gitignore hides the symptom for
    // this repo and leaves the tool littering every other directory anyone
    // runs it from. And there is nothing here worth persisting -- SL15b's
    // "deliberately dumb shell" has exactly one overlay and no layout a user
    // could have arranged.
    ImGui::GetIO().IniFilename = nullptr;
    if (!ImGui_ImplGlfw_InitForOpenGL(impl->window, true)) {
        return refuse("ImGui's GLFW backend failed to initialise.");
    }
    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        return refuse("ImGui's OpenGL3 backend failed to initialise, which usually means the driver "
                      "did not provide an OpenGL 3.3 core context.");
    }
    impl->imgui_ready = true;

    glGenTextures(1, &impl->texture);
    glBindTexture(GL_TEXTURE_2D, impl->texture);
    // GL_LINEAR both ways: the frame is presented at or near 1:1, and GL_NEAREST
    // is not in the set the loader probe confirmed. Clamping stops the edge
    // texels wrapping when the framebuffer and texture disagree by a pixel
    // during a resize.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(impl->window, &fbw, &fbh);
    impl->fb_width = static_cast<uint32_t>(std::max(fbw, 1));
    impl->fb_height = static_cast<uint32_t>(std::max(fbh, 1));
    impl->last_frame = Clock::now();

    if (why_not != nullptr) {
        why_not->clear();
    }
    return std::unique_ptr<GlTargetSink>(new GlTargetSink(std::move(impl)));
#endif
}

GlTargetSink::GlTargetSink(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
GlTargetSink::~GlTargetSink() = default;

FrameInput GlTargetSink::poll() {
    FrameInput in;
#if SPADE_SANDBOX_HAS_GL
    const auto now = Clock::now();
    impl_->dt = std::chrono::duration<float>(now - impl_->last_frame).count();
    impl_->last_frame = now;
    // A first frame after a stall (a breakpoint, a swapped-out process) would
    // otherwise teleport the camera by dt * speed. Bounded, not averaged: an
    // average hides the stall, a bound just refuses to act on it.
    impl_->dt = std::clamp(impl_->dt, 0.0f, 0.1f);

    glfwPollEvents();

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(impl_->window, &fbw, &fbh);
    impl_->fb_width = static_cast<uint32_t>(std::max(fbw, 1));
    impl_->fb_height = static_cast<uint32_t>(std::max(fbh, 1));

    const ImGuiIO& io = ImGui::GetIO();

    // Mouse drag -> orbit, but only when ImGui does not want the mouse, so a
    // click on the overlay does not also spin the camera.
    double cx = 0.0, cy = 0.0;
    glfwGetCursorPos(impl_->window, &cx, &cy);
    const bool down = glfwGetMouseButton(impl_->window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    if (down && !io.WantCaptureMouse) {
        if (impl_->dragging) {
            in.orbit_dx = static_cast<float>(cx - impl_->last_cursor_x);
            in.orbit_dy = static_cast<float>(cy - impl_->last_cursor_y);
        }
        impl_->dragging = true;
    } else {
        impl_->dragging = false;
    }
    impl_->last_cursor_x = cx;
    impl_->last_cursor_y = cy;

    in.dolly = io.WantCaptureMouse ? 0.0f : impl_->scroll_accum;
    impl_->scroll_accum = 0.0f;

    if (!io.WantCaptureKeyboard) {
        const auto held = [this](int key) {
            return glfwGetKey(impl_->window, key) == GLFW_PRESS ? 1.0f : 0.0f;
        };
        in.move_forward = held(GLFW_KEY_W) - held(GLFW_KEY_S);
        in.move_right = held(GLFW_KEY_D) - held(GLFW_KEY_A);
        in.move_up = held(GLFW_KEY_E) - held(GLFW_KEY_Q);
        // EDGE-TRIGGERED, not level. glfwGetKey reports the key as HELD for
        // every frame it is down, so a level test would fire ~60 times per
        // press and the overlay could only ever end up in one state. This is
        // the same shape as a latched warning being re-raised per tick: the
        // input is an EVENT and the API reports a STATE.
        const bool f1 = glfwGetKey(impl_->window, GLFW_KEY_F1) == GLFW_PRESS;
        if (f1 && !impl_->f1_was_down) {
            impl_->show_help = !impl_->show_help;
        }
        impl_->f1_was_down = f1;
    }
    // Esc closes, because a tool you cannot leave with the key everyone
    // reaches for reads as hung.
    if (glfwGetKey(impl_->window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(impl_->window, GLFW_TRUE);
    }
    in.want_close = glfwWindowShouldClose(impl_->window) == GLFW_TRUE;
#endif
    return in;
}

void GlTargetSink::accept(const spade::render::RenderTarget& target) {
    // EVERY PHASE IS TIMED SEPARATELY, because a frame rate is a sum and a sum
    // is not a diagnosis. See the header for why swap_ms must be read first.
    Timings t;
    t.render_ms = impl_->pending_render_ms;  // handed in by the application
    const auto t0 = Clock::now();

    // OUTSIDE THE GUARD: the conversion is the logic, and it is the same
    // statement the headless sink runs. This is the line that keeps the Linux
    // gate's compile coverage meaningful.
    bgrx_to_channels<4u>(target, impl_->rgba);
    const auto t1 = Clock::now();

#if SPADE_SANDBOX_HAS_GL
    glBindTexture(GL_TEXTURE_2D, impl_->texture);
    // glTexImage2D per frame rather than glTexSubImage2D: the latter is absent
    // from the loader the probe measured. ⚠ THIS REALLOCATES THE TEXTURE EVERY
    // FRAME, which is a driver-side allocation per frame -- upload_ms exists
    // specifically so that cost is VISIBLE rather than assumed either way.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, static_cast<GLsizei>(target.width),
                 static_cast<GLsizei>(target.height), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 impl_->rgba.data());
    impl_->texture_width = target.width;
    impl_->texture_height = target.height;
    glBindTexture(GL_TEXTURE_2D, 0);
    const auto t2 = Clock::now();

    glViewport(0, 0, static_cast<GLsizei>(impl_->fb_width), static_cast<GLsizei>(impl_->fb_height));
    glClearColor(0.05f, 0.05f, 0.07f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // THE FRAME IS DRAWN BY IMGUI'S OWN DRAW LIST, not by a quad of ours.
    // ⭐ That is why this file needs no shader, no VAO and no vertex buffer:
    // ImGui already has a textured-quad pipeline and it is already verified.
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    ImGui::GetBackgroundDrawList()->AddImage(as_texture_id(impl_->texture), ImVec2(0.0f, 0.0f),
                                             display);

    if (impl_->show_help) {
        // NOT A PANEL. The exclusions ruled for this task are hierarchy,
        // inspector and scene picker. This is a controls legend plus the
        // instrumentation the user asked for, it takes no input, and F1
        // dismisses it.
        ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.65f);
        if (ImGui::Begin("spade sandbox", nullptr,
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                             ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav)) {
            const Timings a = impl_->win_avg;
            const double mb = static_cast<double>(working_set_bytes()) / (1024.0 * 1024.0);

            ImGui::Text("%ux%u    %.1f fps    %.1f ms", impl_->texture_width, impl_->texture_height,
                        a.total_ms > 0.0f ? 1000.0f / a.total_ms : 0.0f, a.total_ms);
            // The user's ask: memory beside the fps, same overlay, same cadence.
            ImGui::Text("memory  %.1f MB", mb);
            ImGui::Separator();
            // WHERE THE FRAME WENT. The point of showing all five rather than a
            // total: they have different remedies, and one of them is not a cost.
            ImGui::Text("render   %6.2f ms", a.render_ms);
            ImGui::Text("convert  %6.2f ms", a.convert_ms);
            ImGui::Text("upload   %6.2f ms", a.upload_ms);
            ImGui::Text("ui       %6.2f ms", a.ui_ms);
            ImGui::Text("swap     %6.2f ms%s", a.swap_ms,
                        impl_->options.vsync ? "   (vsync: waiting is normal)" : "");
            ImGui::Separator();
            ImGui::Text("drag orbit   scroll dolly   WASD pan   Q/E down/up");
            ImGui::Text("F1 hide      Esc quit");
        }
        ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    const auto t3 = Clock::now();

    glfwSwapBuffers(impl_->window);
    const auto t4 = Clock::now();

    const auto ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<float, std::milli>(b - a).count();
    };
    t.convert_ms = ms(t0, t1);
    t.upload_ms = ms(t1, t2);
    t.ui_ms = ms(t2, t3);
    t.swap_ms = ms(t3, t4);
#else
    t.convert_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
#endif
    // total is the SUM OF THE PARTS, deliberately, rather than a wall-clock
    // span across accept(): a wall-clock total would silently absorb anything
    // that is not one of the five, and then the parts would not add up and
    // nobody would know which was wrong.
    t.total_ms = t.render_ms + t.convert_ms + t.upload_ms + t.ui_ms + t.swap_ms;

    accumulate(impl_->win_sum, t);
    accumulate(impl_->life_sum, t);
    if (++impl_->win_count >= 30u) {
        impl_->win_avg = scale(impl_->win_sum, 1.0f / static_cast<float>(impl_->win_count));
        impl_->win_sum = Timings{};
        impl_->win_count = 0u;
    }
    ++impl_->presented;
}

bool GlTargetSink::should_close() const noexcept {
#if SPADE_SANDBOX_HAS_GL
    return glfwWindowShouldClose(impl_->window) == GLFW_TRUE;
#else
    return true;
#endif
}

void GlTargetSink::note_render_ms(float ms) noexcept { impl_->pending_render_ms = ms; }

GlTargetSink::Timings GlTargetSink::average_timings() const noexcept { return impl_->win_avg; }

GlTargetSink::Timings GlTargetSink::lifetime_timings() const noexcept {
    if (impl_->presented == 0u) {
        return Timings{};
    }
    return scale(impl_->life_sum, 1.0f / static_cast<float>(impl_->presented));
}

GlTargetSink::GlInfo GlTargetSink::gl_info() const {
    GlInfo info;
#if SPADE_SANDBOX_HAS_GL
    const auto str = [](GLenum name) -> std::string {
        const GLubyte* p = glGetString(name);
        return p != nullptr ? std::string(reinterpret_cast<const char*>(p)) : std::string{};
    };
    info.renderer = str(GL_RENDERER);
    info.version = str(GL_VERSION);
    info.vendor = str(GL_VENDOR);
#endif
    return info;
}

uint64_t GlTargetSink::working_set_bytes() noexcept {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        return static_cast<uint64_t>(pmc.WorkingSetSize);
    }
    return 0u;
#else
    // ⚠ REPORTS 0 RATHER THAN GUESSING. A fabricated figure on a HUD is worse
    // than a blank one: the reader cannot tell it apart from a real measurement.
    // The same argument that kept should_close() off the seam.
    return 0u;
#endif
}

uint32_t GlTargetSink::framebuffer_width() const noexcept { return impl_->fb_width; }
uint32_t GlTargetSink::framebuffer_height() const noexcept { return impl_->fb_height; }
float GlTargetSink::delta_seconds() const noexcept { return impl_->dt; }
uint64_t GlTargetSink::presented() const noexcept { return impl_->presented; }

}  // namespace spade::sandbox
