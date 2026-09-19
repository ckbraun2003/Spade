// spade_sandbox -- the engine's reference application (SL10-SL13, Plan C).
//
// ⚠ TASK C2 LANDED THE WINDOW. The banner below described C0 and is kept
// because its ARGUMENT is still the design -- headless first, so SL15b is
// satisfied BY CONSTRUCTION rather than by discipline. What changed is only
// that the window now exists and is the default mode; --headless remains the
// test surface and the only mode CI will ever run.
//
// THIS WAS TASK C0 AND IT WAS HEADLESS-ONLY, DELIBERATELY. There was no window
// here, no GLFW, no ImGui, and no TargetSink yet. SL15b rules that
// `--headless` is the TEST SURFACE and that the window is "a deliberately
// dumb shell -- any logic that migrates into it is a defect, because it
// becomes untestable by construction."
//
// Plan C takes that literally by building headless FIRST. A GUI built
// window-first satisfies SL15b only by discipline, and discipline is what
// fails; built headless-first it is satisfied BY CONSTRUCTION, because for
// the first three tasks there is no widget to hide logic in. Every later task
// states the headless command that exercises its logic before it states its
// UI.
//
// And it is the only mode CI will ever run: this suite's sole automated gate
// is .github/workflows/spade.yml (spade/tests/ is unreachable from every root
// build tree -- see tests/CMakeLists.txt), and that runner has no GPU AND NO
// DISPLAY.
//
// SL2b: this target may include only headers the engine INSTALLS and may link
// only spade:: targets. It reaches into no engine internal by relative path.
// The guard over that include graph acquires its first subject with this file.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

#include "gl_target_sink.hpp"  // Plan C task C2 -- the window
#include "orbit_camera.hpp"    // Plan C task C2 -- input, testable with no display
#include "target_sink.hpp"     // Plan C task C1 -- the seam SL11 names

namespace {

constexpr uint32_t kDefaultWidth = 320;
constexpr uint32_t kDefaultHeight = 180;

void print_usage() {
    std::puts("spade_sandbox -- the Spade engine's reference application.\n");
    std::puts("  (no mode flag)        open a window on the default scene -- THE DEFAULT");
    std::puts("  --window              the same, stated explicitly");
    std::puts("  --headless            render one frame with no window and exit (the CI surface)");
    std::puts("  --out <path>          write the frame as a binary PPM (default: no file, render only)");
    std::puts("  --width/--height <n>  frame size (headless 320x180, window 1280x720)");
    std::puts("  --no-vsync            do not wait for the display refresh");
    std::puts("  --no-grid             disable the infinite analytic ground grid");
    std::puts("  --horizon-blur <f>    SR-17a atmospheric strength, 0 = off (default 0)");
    std::puts("  --help                this text\n");
    std::puts("In the window: drag orbits, scroll dollies, WASD pans, Q/E lowers/raises, Esc quits.");
    std::puts("A scene picker, a hierarchy and an inspector arrive in Plan C tasks C3-C7.");
    std::puts("Run with --headless in CI; SPADE_BUILD_SANDBOX=OFF omits this target entirely.");
}

// C0's built-in scene: a ground plane and nothing else. Deliberately built in
// code rather than loaded from a file, so this task depends on NO content path
// at all -- the scene picker (C3) is what introduces file loading, behind its
// own task. Thirty agreement tests were red for days because a content path
// moved; C0 did not acquire one before it needed one.
//
// C2: SPLIT FROM THE RENDER. The world is built ONCE and rendered many times,
// which the single-frame version did not have to distinguish. Rebuilding the
// scene per frame would have worked and been a defect of the quiet kind: the
// window would run at a fraction of its frame rate for a reason no profile
// line would name.
[[nodiscard]] bool build_builtin_scene(spade::render::RenderScene& out) {
    const spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                      .name("sandbox_c0")
                                                      .environment(spade::Environment{})
                                                      .capacities(spade::Capacities{4, 4, 1, 1})
                                                      .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                      .build();
    if (!world) {
        std::fprintf(stderr, "spade_sandbox: WorldBuilder::build failed: %s\n",
                     world.error().context.c_str());
        return false;
    }
    const spade::Result<spade::render::RenderScene> scene =
        spade::render::scene_from_world(*world, {});
    if (!scene) {
        std::fprintf(stderr, "spade_sandbox: scene_from_world failed: %s\n",
                     scene.error().context.c_str());
        return false;
    }
    out = *scene;
    return true;
}

// C1: renders and hands the target to a TargetSink. The sink is a PARAMETER
// rather than something this function picks, which is the whole point of the
// seam -- C2's windowed sink substitutes here without touching a line below,
// and that is not a claim, it is what this function's signature enforces.
[[nodiscard]] bool render_frame(const spade::render::RenderScene& scene,
                                const spade::render::Camera& camera, uint32_t width,
                                uint32_t height, bool grid, float blur,
                                std::vector<uint8_t>& pixels, spade::sandbox::TargetSink& sink) {
    // PA-1: RenderTarget NEVER owns its pixel memory. The caller allocates and
    // hands it a span. This is the constraint the TargetSink seam (C1)
    // inherits -- a presenter is HANDED a buffer and must not allocate one, or
    // the sandbox grows a frame pool the engine deliberately does not have.
    //
    // `assign` rather than `resize`: on a window resize the old contents are
    // meaningless, and a partially-stale buffer is how a resize artefact gets
    // mistaken for a renderer bug.
    const size_t needed = static_cast<size_t>(width) * height * 4u;
    if (pixels.size() != needed) {
        pixels.assign(needed, 0u);
    }
    spade::render::RenderTarget target{
        .pixels = pixels,
        .width = width,
        .height = height,
        .stride = width * 4u,
        .format = spade::render::PixelFormat::bgrx8,
    };

    spade::render::RenderOptions options;
    options.ground_grid = grid;
    options.horizon_blend_strength = blur;
    const spade::Result<void> rendered = spade::render::render(scene, camera, options, target);
    if (!rendered) {
        std::fprintf(stderr, "spade_sandbox: render failed: %s\n", rendered.error().context.c_str());
        return false;
    }
    // THE SEAM. Everything above is the engine's; everything the sink does with
    // the frame is the application's. SL10 asserts these two halves agree, and
    // test_sandbox_target_sink.cpp drives this same call shape rather than a
    // re-creation of it.
    sink.accept(target);
    return true;
}

// NOTE: this file's own BGRX8 -> PPM writer MOVED to HeadlessTargetSink
// (target_sink.hpp) at task C1, and at C2 the channel reorder itself moved to
// bgrx_to_channels so the windowed sink's BGRX->RGBA and the headless sink's
// BGRX->RGB are ONE statement. A second copy of the conversion is exactly the
// "two expressions that happen to agree" shape SR-17 clause 4 exists to
// forbid, and SL10's test asserts against ONE of them.

}  // namespace

// ---------------------------------------------------------------------------
// THE WINDOW LOOP -- Plan C task C2.
//
// ⭐ IT IS THIS SHORT ON PURPOSE. render -> accept -> poll -> apply. There is
// no state here beyond the camera, because SL15b rules the window a
// "deliberately dumb shell" and every line that could have lived here instead
// lives in OrbitCamera, where it compiles and can be asserted on a machine
// with no display.
// ---------------------------------------------------------------------------
namespace {

int run_windowed(const spade::render::RenderScene& scene, uint32_t width, uint32_t height,
                 bool grid, float blur, bool vsync) {
    spade::sandbox::GlTargetSink::Options opts;
    opts.width = width;
    opts.height = height;
    opts.title = "spade sandbox";
    opts.vsync = vsync;

    std::string why_not;
    std::unique_ptr<spade::sandbox::GlTargetSink> sink =
        spade::sandbox::GlTargetSink::create(opts, &why_not);
    if (!sink) {
        // ⚠⚠ THE REFUSAL IS THE DELIVERABLE, NOT AN ERROR PATH. It names the
        // cause and it exits non-zero. A missing capability refuses, never
        // degrades -- a window that silently fails to appear is the worst
        // possible outcome for someone judging whether the GUI works.
        std::fprintf(stderr, "spade_sandbox: cannot open a window -- %s\n", why_not.c_str());
        std::fprintf(stderr, "spade_sandbox: --headless still works in this build.\n");
        return 3;
    }

    spade::sandbox::OrbitCamera camera;
    std::vector<uint8_t> pixels;

    while (!sink->should_close()) {
        const spade::sandbox::FrameInput in = sink->poll();
        if (in.want_close) {
            break;
        }
        camera.apply(in, sink->delta_seconds());

        // The FRAMEBUFFER size, never the window size: on a HiDPI display they
        // differ, and rendering at window size then letting GL stretch it is
        // precisely the blocky-viewport defect this estate has already
        // diagnosed once in the editor.
        const uint32_t fw = sink->framebuffer_width();
        const uint32_t fh = sink->framebuffer_height();
        if (fw == 0u || fh == 0u) {
            continue;  // minimised; there is nothing to render into
        }
        if (!render_frame(scene, camera.to_render_camera(), fw, fh, grid, blur, pixels, *sink)) {
            return 1;
        }
    }

    // Reported rather than inferred: "the window appeared" and "the loop ran"
    // are different claims, and only the second has a number.
    std::printf("spade_sandbox: window closed after %llu frames\n",
                static_cast<unsigned long long>(sink->presented()));
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    // ⚠ THE DEFAULT MODE CHANGED AT C2, AND THE OLD REFUSAL'S REASONING WENT
    // WITH IT. C0 refused to run without --headless because "no arguments"
    // could plausibly have meant "open a window", which did not exist. It
    // exists now and it is what this tool is for, so no-arguments opens it --
    // and the chosen mode is PRINTED, because the objection that produced the
    // old refusal was to picking a mode SILENTLY, not to having a default.
    bool headless = false;
    bool windowed = false;
    const char* out_path = nullptr;
    bool width_set = false, height_set = false;
    uint32_t width = 0, height = 0;
    // THE SANDBOX'S DEFAULT WORLD HAS THE GRID ON. The engine's RenderOptions
    // default is OFF (see target.hpp for the three tests that decided it); the
    // "default render world" the user asked for is THIS application's scene,
    // and it opts in here where the choice is visible.
    bool grid = true;
    bool vsync = true;
    float blur = 0.0f;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--help") == 0) {
            print_usage();
            return 0;
        } else if (std::strcmp(a, "--headless") == 0) {
            headless = true;
        } else if (std::strcmp(a, "--window") == 0) {
            windowed = true;
        } else if (std::strcmp(a, "--no-vsync") == 0) {
            vsync = false;
        } else if (std::strcmp(a, "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (std::strcmp(a, "--width") == 0 && i + 1 < argc) {
            width = static_cast<uint32_t>(std::atoi(argv[++i]));
            width_set = true;
        } else if (std::strcmp(a, "--no-grid") == 0) {
            grid = false;
        } else if (std::strcmp(a, "--horizon-blur") == 0 && i + 1 < argc) {
            blur = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(a, "--height") == 0 && i + 1 < argc) {
            height = static_cast<uint32_t>(std::atoi(argv[++i]));
            height_set = true;
        } else {
            std::fprintf(stderr, "spade_sandbox: unrecognised argument '%s'\n\n", a);
            print_usage();
            return 2;
        }
    }

    // REFUSES rather than guessing. Two modes named at once is not a
    // preference the tool can resolve, and picking one would make the other
    // flag silently inert.
    if (headless && windowed) {
        std::fprintf(stderr, "spade_sandbox: --headless and --window are mutually exclusive\n");
        return 2;
    }
    if (!headless) {
        windowed = true;
    }

    // The two modes have genuinely different natural sizes -- a CI frame is
    // small on purpose and a window is not -- so the default depends on the
    // mode while an EXPLICIT --width/--height always wins.
    if (!width_set) {
        width = windowed ? 1280u : kDefaultWidth;
    }
    if (!height_set) {
        height = windowed ? 720u : kDefaultHeight;
    }
    if (width == 0u || height == 0u) {
        std::fprintf(stderr, "spade_sandbox: --width and --height must both be non-zero\n");
        return 2;
    }
    if (windowed && out_path != nullptr) {
        // Not silently ignored: a flag that does nothing is worse than a flag
        // that refuses, because the user believes it worked.
        std::fprintf(stderr, "spade_sandbox: --out writes a single frame and applies to "
                             "--headless only\n");
        return 2;
    }

    spade::render::RenderScene scene;
    if (!build_builtin_scene(scene)) {
        return 1;
    }

    if (windowed) {
        std::printf("spade_sandbox: opening a %ux%u window (--headless for the CI surface)\n",
                    width, height);
        return run_windowed(scene, width, height, grid, blur, vsync);
    }

    // C1: the sink is chosen HERE, by the application, and handed down. An
    // empty path means render-and-discard -- the conversion still runs, so
    // --out changes only where the bytes go, never whether the work happens.
    spade::sandbox::HeadlessTargetSink sink{out_path != nullptr ? std::string(out_path)
                                                                : std::string{}};
    std::vector<uint8_t> pixels;
    spade::render::Camera camera;
    camera.position = glm::vec3(0.0f, 2.0f, 8.0f);
    if (!render_frame(scene, camera, width, height, grid, blur, pixels, sink)) {
        return 1;
    }
    if (sink.failed()) {
        std::fprintf(stderr, "spade_sandbox: could not write %s\n",
                     out_path != nullptr ? out_path : "(no path)");
        return 1;
    }

    std::printf("spade_sandbox: rendered %ux%u (%zu bytes)%s%s\n", width, height, pixels.size(),
                out_path != nullptr ? " -> " : "", out_path != nullptr ? out_path : "");
    return 0;
}
