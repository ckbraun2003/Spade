// spade_sandbox -- the engine's reference application (SL10-SL13, Plan C).
//
// THIS IS TASK C0 AND IT IS HEADLESS-ONLY, DELIBERATELY. There is no window
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
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

#include "target_sink.hpp"  // Plan C task C1 -- the seam SL11 names

namespace {

constexpr uint32_t kDefaultWidth = 320;
constexpr uint32_t kDefaultHeight = 180;

void print_usage() {
    std::puts("spade_sandbox -- the Spade engine's reference application.\n");
    std::puts("  --headless            render one frame with no window and exit (the only mode C0 has)");
    std::puts("  --out <path>          write the frame as a binary PPM (default: no file, render only)");
    std::puts("  --width/--height <n>  frame size (default 320x180)");
    std::puts("  --no-grid             disable the infinite analytic ground grid");
    std::puts("  --horizon-blur <f>    SR-17a atmospheric strength, 0 = off (default 0)");
    std::puts("  --help                this text\n");
    std::puts("A window, a scene picker, a hierarchy and an inspector arrive in Plan C tasks C2-C7.");
    std::puts("Run with --headless in CI; SPADE_BUILD_SANDBOX=OFF omits this target entirely.");
}

// C0's built-in scene: a ground plane and nothing else. Deliberately built in
// code rather than loaded from a file, so this task depends on NO content path
// at all -- the scene picker (C3) is what introduces file loading, behind its
// own task. Thirty agreement tests were red for days because a content path
// moved; C0 does not acquire one before it needs one.
// C1: renders and hands the target to a TargetSink. The sink is a PARAMETER
// rather than something this function picks, which is the whole point of the
// seam -- C2's windowed sink substitutes here without touching a line below.
[[nodiscard]] bool render_builtin_scene(uint32_t width, uint32_t height, bool grid, float blur,
                                         std::vector<uint8_t>& pixels,
                                         spade::sandbox::TargetSink& sink) {
    const spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                      .name("sandbox_c0")
                                                      .environment(spade::Environment{})
                                                      .capacities(spade::Capacities{4, 4, 1, 1})
                                                      .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                      .build();
    if (!world) {
        std::fprintf(stderr, "spade_sandbox: WorldBuilder::build failed: %s\n", world.error().context.c_str());
        return false;
    }

    const spade::Result<spade::render::RenderScene> scene = spade::render::scene_from_world(*world, {});
    if (!scene) {
        std::fprintf(stderr, "spade_sandbox: scene_from_world failed: %s\n", scene.error().context.c_str());
        return false;
    }

    spade::render::Camera camera;
    camera.position = glm::vec3(0.0f, 2.0f, 8.0f);

    // PA-1: RenderTarget NEVER owns its pixel memory. The caller allocates and
    // hands it a span. This is the constraint the TargetSink seam (C1) inherits
    // -- a presenter is HANDED a buffer and must not allocate one, or the
    // sandbox grows a frame pool the engine deliberately does not have.
    pixels.assign(static_cast<size_t>(width) * height * 4u, 0u);
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
    const spade::Result<void> rendered = spade::render::render(*scene, camera, options, target);
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
// (target_sink.hpp) at task C1. It is not duplicated here: a second copy of
// the conversion is exactly the "two expressions that happen to agree" shape
// SR-17 clause 4 exists to forbid, and SL10's test asserts against ONE of
// them.

}  // namespace

int main(int argc, char** argv) {
    bool headless = false;
    const char* out_path = nullptr;
    uint32_t width = kDefaultWidth;
    uint32_t height = kDefaultHeight;
    // THE SANDBOX'S DEFAULT WORLD HAS THE GRID ON. The engine's RenderOptions
    // default is OFF (see target.hpp for the three tests that decided it); the
    // "default render world" the user asked for is THIS application's scene,
    // and it opts in here where the choice is visible.
    bool grid = true;
    float blur = 0.0f;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--help") == 0) {
            print_usage();
            return 0;
        } else if (std::strcmp(a, "--headless") == 0) {
            headless = true;
        } else if (std::strcmp(a, "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (std::strcmp(a, "--width") == 0 && i + 1 < argc) {
            width = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (std::strcmp(a, "--no-grid") == 0) {
            grid = false;
        } else if (std::strcmp(a, "--horizon-blur") == 0 && i + 1 < argc) {
            blur = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(a, "--height") == 0 && i + 1 < argc) {
            height = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else {
            std::fprintf(stderr, "spade_sandbox: unrecognised argument '%s'\n\n", a);
            print_usage();
            return 2;
        }
    }

    // REFUSES rather than defaulting. C0 has exactly one mode, and running
    // with no arguments could plausibly mean "open a window" -- which does not
    // exist yet. A tool that quietly picks a mode when the caller named none
    // is how "it ran and did nothing" becomes a bug report.
    if (!headless) {
        std::fprintf(stderr, "spade_sandbox: --headless is required (C0 has no window yet)\n\n");
        print_usage();
        return 2;
    }
    if (width == 0u || height == 0u) {
        std::fprintf(stderr, "spade_sandbox: --width and --height must both be non-zero\n");
        return 2;
    }

    // C1: the sink is chosen HERE, by the application, and handed down. An
    // empty path means render-and-discard -- the conversion still runs, so
    // --out changes only where the bytes go, never whether the work happens.
    spade::sandbox::HeadlessTargetSink sink{out_path != nullptr ? std::string(out_path) : std::string{}};
    std::vector<uint8_t> pixels;
    if (!render_builtin_scene(width, height, grid, blur, pixels, sink)) {
        return 1;
    }
    if (sink.failed()) {
        std::fprintf(stderr, "spade_sandbox: could not write %s\n", out_path != nullptr ? out_path : "(no path)");
        return 1;
    }

    std::printf("spade_sandbox: rendered %ux%u (%zu bytes)%s%s\n", width, height, pixels.size(),
                out_path != nullptr ? " -> " : "", out_path != nullptr ? out_path : "");
    return 0;
}
