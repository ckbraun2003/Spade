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
// .github/workflows/spade.yml, which used to be this suite's sole automated
// gate and ran it exactly this way -- headless, no GPU, no display -- was
// deleted 2026-09-18. scripts/test.ps1 is this repo's gate now, but it does
// not run this suite; nothing currently runs spade_sandbox automatically
// (tests/ is unreachable from every root build tree -- see
// tests/CMakeLists.txt).
//
// SL2b: this target may include only headers the engine INSTALLS and may link
// only spade:: targets. It reaches into no engine internal by relative path.
// The guard over that include graph acquires its first subject with this file.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>  // std::move -- the WorldDesc moves into the caller's storage
#include <vector>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

#include "drone_view.hpp"      // the drone sim box: stand, controller, panel (+ drone_sim.hpp)
#include "editor_cli.hpp"      // --edit: the editor's command line, no window
#include "editor_session.hpp"  // --editor: the editor's window
#include "gl_target_sink.hpp"  // Plan C task C2 -- the window
#include "live_tour.hpp"       // --live-smoke: the recorded tour (scripts/live-smoke.ps1)
#include "orbit_camera.hpp"    // Plan C task C2 -- input, testable with no display
#include "scene_sessions.hpp"  // each scene's setup and its one frame function
#include "target_sink.hpp"     // Plan C task C1 -- the seam SL11 names

namespace {

using spade::sandbox::build_drone_scene;
using spade::sandbox::drone_frame;
using spade::sandbox::GpuRenderer;
using spade::sandbox::render_frame;

constexpr uint32_t kDefaultWidth = 320;
constexpr uint32_t kDefaultHeight = 180;

void print_usage() {
    std::puts("spade_sandbox -- the Spade engine's reference application.\n");
    std::puts("  (no mode flag)        open a window on the default scene -- THE DEFAULT");
    std::puts("  --window              the same, stated explicitly");
    std::puts("  --headless            render one frame with no window and exit (the CI surface)");
    std::puts("  --scene drone         the drone sim box, the only scene (the default)");
    std::puts("  --out <path>          write the frame as a binary PPM (default: no file, render only)");
    std::puts("  --width/--height <n>  frame size (headless 320x180, window 1280x720)");
    std::puts("  --no-vsync            do not wait for the display refresh");
    std::puts("  --live-smoke          tour the drone box, the airflow and the editor in the window,");
    std::puts("                        record tour.mp4, a PNG per chapter and journal.txt into");
    std::puts("                        --out <dir>; exit 0 PASS, 1 FAIL, 2 no verdict");
    std::puts("                        (scripts/live-smoke.ps1 runs it)");
    std::puts("  --label <text>        live smoke: the commit and date for the title cards");
    std::puts("  --assets <dir>        live smoke: the repository's assets/, for the editor's scene files");
    std::puts("  --inject <spec>       live smoke red runs: skip:<step> or anomaly:<step>/<name>");
    std::puts("  --horizon-blur <f>    SR-17a atmospheric strength, 0 = off (default 0)");
    std::puts("  --edit <scene.yaml>   FIRST ARGUMENT ONLY: the editor's command line, no window:");
    std::puts("                        [--apply <edits.txt>] [--save <scene.yaml>] [--save-world <world.yaml>]");
    std::puts("                        [--run <steps>]; exit 0 done, 1 refused, 2 bad arguments");
    std::puts("  --editor [scene.yaml] FIRST ARGUMENT ONLY: the editor's window on a scene file");
    std::puts("                        (default: assets/scenes/hover.scene.yaml from the working directory)");
    std::puts("  --help                this text\n");
    std::puts("In the window -- THE DRONE SIM BOX (default):");
    std::puts("  arrows         pitch and roll (held; the target holds when released)   Z/X yaw");
    std::puts("  R              level");
    std::puts("  A/D around     E/Q over/under     W/S nearer/further (0.5-6 m)     drag orbit");
    std::puts("  the panel      wind, turbulence, density, gravity, throttle, backend; readouts");
    std::puts("The scene builder is retired: the editor (--editor) places, moves, recolours,");
    std::puts("duplicates and deletes objects in a scene file.");
    std::puts("  F1 hides the HUD, Esc quits.");
    std::puts("Run with --headless in CI; SPADE_BUILD_SANDBOX=OFF omits this target entirely.");
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

// Reported rather than inferred: "the window appeared" and "the loop ran"
// are different claims, and only the second has a number.
// ⭐ AND THE SPLIT GOES TO STDOUT AS WELL AS THE HUD, so the answer to
// "why is it N fps" survives the window closing and can be captured from
// a script. A number only a human can read by looking at it is not a
// measurement anyone else can check.
void print_exit_summary(const spade::sandbox::GlTargetSink& sink, uint32_t width, uint32_t height, bool vsync) {
    const spade::sandbox::GlTargetSink::Timings life = sink.lifetime_timings();
    std::printf("spade_sandbox: window closed after %llu frames\n",
                static_cast<unsigned long long>(sink.presented()));
    std::printf("spade_sandbox: mean frame %.2f ms (%.1f fps) at %ux%u, vsync %s\n",
                life.total_ms, life.total_ms > 0.0f ? 1000.0f / life.total_ms : 0.0f, width,
                height, vsync ? "on" : "off");
    std::printf("spade_sandbox:   physics %.2f  render %.2f  convert %.2f  upload %.2f  ui %.2f  swap %.2f  ms\n",
                life.physics_ms, life.render_ms, life.convert_ms, life.upload_ms, life.ui_ms, life.swap_ms);
    std::printf("spade_sandbox:   working set %.1f MB\n",
                static_cast<double>(spade::sandbox::GlTargetSink::working_set_bytes()) /
                    (1024.0 * 1024.0));
}

// ---------------------------------------------------------------------------
// THE DRONE SIM BOX -- the default scene. A quadrotor on a test stand, flown
// in attitude through the engine's rotor model. Every decision is in
// drone_sim.hpp / drone_view.hpp and is
// asserted with no display; the wiring is DroneSession (scene_sessions.hpp).
// ---------------------------------------------------------------------------

int run_windowed_drone(uint32_t width, uint32_t height, float blur, bool vsync) {
    // Declared first, so it is destroyed last: the session detaches from it.
    // Opened second, so a stand that will not build refuses with no window.
    std::unique_ptr<spade::sandbox::GlTargetSink> sink;
    spade::Result<std::unique_ptr<spade::sandbox::DroneSession>> made = spade::sandbox::DroneSession::create(blur);
    if (!made) {
        std::fprintf(stderr, "spade_sandbox: %s\n", made.error().context.c_str());
        return 1;
    }
    sink = spade::sandbox::open_window(width, height, vsync);
    if (!sink) {
        return 3;
    }
    spade::sandbox::DroneSession& session = **made;
    session.attach(*sink);

    while (!sink->should_close()) {
        const spade::sandbox::FrameInput in = sink->poll();
        if (in.want_close) {
            break;
        }
        if (const spade::Result<void> drew = session.frame(in, sink->delta_seconds()); !drew) {
            std::fprintf(stderr, "spade_sandbox: %s\n", drew.error().context.c_str());
            return 1;
        }
    }
    print_exit_summary(*sink, width, height, vsync);
    return 0;
}

// --editor [scene.yaml]: the editor's window (editor plan Task 10). The
// scene is opened and composed before the window, so one that will not open
// refuses with no window.
int run_windowed_editor(const std::filesystem::path& scene_file, uint32_t width, uint32_t height, bool vsync) {
    std::unique_ptr<spade::sandbox::GlTargetSink> sink;  // destroyed last: the session detaches from it
    spade::Result<spade::sandbox::editor::OpenedScene> opened = spade::sandbox::editor::open_scene(scene_file);
    if (!opened) {
        std::fprintf(stderr, "spade_sandbox: --editor %s: %s\n", scene_file.string().c_str(),
                     opened.error().context.c_str());
        return 1;
    }
    spade::Result<std::unique_ptr<spade::sandbox::EditorSession>> made =
        spade::sandbox::EditorSession::create(std::move(*opened));
    if (!made) {
        std::fprintf(stderr, "spade_sandbox: --editor %s: %s\n", scene_file.string().c_str(),
                     made.error().context.c_str());
        return 1;
    }
    sink = spade::sandbox::open_window(width, height, vsync);
    if (!sink) {
        return 3;
    }
    spade::sandbox::EditorSession& session = **made;
    session.attach(*sink);
    while (!sink->should_close()) {
        const spade::sandbox::FrameInput in = sink->poll();
        if (in.want_close) {
            break;
        }
        if (const spade::Result<void> drew = session.frame(in, sink->delta_seconds()); !drew) {
            std::fprintf(stderr, "spade_sandbox: %s\n", drew.error().context.c_str());
            return 1;
        }
    }
    print_exit_summary(*sink, width, height, vsync);
    return 0;
}

// --headless --scene drone: deterministic -- 250 fixed steps (0.5 s) with the
// controller holding level, one CPU frame from a fixed camera.
int run_headless_drone(uint32_t width, uint32_t height, float blur, const char* out_path) {
    spade::Result<spade::sandbox::DroneSim> drone = spade::sandbox::DroneSim::create(spade::sandbox::DronePhysicsOptions{});
    if (!drone) {
        std::fprintf(stderr, "spade_sandbox: the drone stand would not build: %s\n", drone.error().context.c_str());
        return 1;
    }
    if (const spade::Result<void> stepped = drone->step_fixed(250); !stepped) {
        std::fprintf(stderr, "spade_sandbox: the drone step failed: %s\n", stepped.error().context.c_str());
        return 1;
    }
    spade::WorldDesc world;
    spade::render::RenderScene scene;
    if (!build_drone_scene(scene, world)) {
        return 1;
    }
    const spade::sandbox::DroneDrawBinding binding = spade::sandbox::bind_drone_scene(scene, drone->params());

    spade::render::Camera camera;
    camera.position = glm::vec3(1.2f, 0.6f, 1.6f);
    camera.orientation = glm::quatLookAt(glm::normalize(-camera.position), glm::vec3(0.0f, 1.0f, 0.0f));
    const spade::render::RenderOptions options =
        drone_frame(scene, binding, *drone, drone->readouts().orientation, blur);

    spade::sandbox::HeadlessTargetSink sink{out_path != nullptr ? std::string(out_path) : std::string{}};
    std::vector<uint8_t> pixels;
    if (!render_frame(scene, camera, width, height, options, pixels, sink)) {
        return 1;
    }
    if (sink.failed()) {
        std::fprintf(stderr, "spade_sandbox: could not write %s\n", out_path != nullptr ? out_path : "(no path)");
        return 1;
    }
    const spade::sandbox::DroneReadouts r = drone->readouts();
    std::printf("spade_sandbox: drone view, %ux%u after %llu steps%s%s\n", width, height,
                static_cast<unsigned long long>(r.tick), out_path != nullptr ? " -> " : "",
                out_path != nullptr ? out_path : "");
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    // --edit IS ITS OWN PROGRAM: the editor's command line (editor_cli.hpp)
    // takes every argument after it, parses them itself, and never opens a
    // window, so it is dispatched before any of the modes below.
    if (argc >= 2 && std::strcmp(argv[1], "--edit") == 0) {
        return spade::sandbox::editor::edit_cli(std::vector<std::string>(argv + 2, argv + argc), std::cout,
                                                std::cerr);
    }
    // --editor is the editor's window on one scene file, so it too takes the
    // arguments after it as its own.
    if (argc >= 2 && std::strcmp(argv[1], "--editor") == 0) {
        if (argc > 3) {
            std::fprintf(stderr, "spade_sandbox: --editor takes one scene file at most\n");
            return 2;
        }
        const std::filesystem::path scene = argc == 3 ? std::filesystem::path(argv[2])
                                                      : std::filesystem::path("assets/scenes/hover.scene.yaml");
        if (!std::filesystem::is_regular_file(scene)) {
            std::fprintf(stderr,
                         "spade_sandbox: --editor: no scene file at %s; give one: spade_sandbox --editor "
                         "<scene.yaml>\n",
                         scene.string().c_str());
            return 2;
        }
        return run_windowed_editor(scene, 1280u, 720u, true);
    }
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
    bool vsync = true;
    float blur = 0.0f;
    const char* scene_name = nullptr;  // "drone", the only scene (the default)
    bool live_smoke = false;
    spade::sandbox::live::TourOptions tour;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--help") == 0) {
            print_usage();
            return 0;
        } else if (std::strcmp(a, "--scene") == 0 && i + 1 < argc) {
            scene_name = argv[++i];
        } else if (std::strcmp(a, "--smoke") == 0) {
            // Named, not just unrecognised: an old command line says what replaced it.
            std::fprintf(stderr, "spade_sandbox: --smoke scripted the scene builder, which is retired (INT-9); "
                                 "--live-smoke tours the editor, which does what the builder did\n");
            return 2;
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
        } else if (std::strcmp(a, "--horizon-blur") == 0 && i + 1 < argc) {
            blur = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(a, "--height") == 0 && i + 1 < argc) {
            height = static_cast<uint32_t>(std::atoi(argv[++i]));
            height_set = true;
        } else if (std::strcmp(a, "--live-smoke") == 0) {
            live_smoke = true;
        } else if (std::strcmp(a, "--assets") == 0 && i + 1 < argc) {
            tour.assets_dir = argv[++i];
        } else if (std::strcmp(a, "--label") == 0 && i + 1 < argc) {
            tour.label = argv[++i];
        } else if (std::strcmp(a, "--inject") == 0 && i + 1 < argc) {
            const spade::Result<spade::sandbox::live::Injection> inj = spade::sandbox::live::parse_inject(argv[++i]);
            if (!inj) {
                std::fprintf(stderr, "spade_sandbox: %s\n", inj.error().context.c_str());
                return 2;
            }
            tour.injections.push_back(*inj);
        } else {
            std::fprintf(stderr, "spade_sandbox: unrecognised argument '%s'\n\n", a);
            print_usage();
            return 2;
        }
    }

    // --live-smoke tours the drone box, the airflow and the editor in one
    // window and records it; it picks its own scenes, and it writes a folder,
    // so it needs --out.
    if (live_smoke) {
        if (headless || scene_name != nullptr) {
            std::fprintf(stderr, "spade_sandbox: --live-smoke tours its chapters in a window; it takes no "
                                 "--headless or --scene\n");
            return 2;
        }
        if (out_path == nullptr) {
            std::fprintf(stderr, "spade_sandbox: --live-smoke needs --out <dir> for the recording and the "
                                 "journal (scripts/live-smoke.ps1 picks one)\n");
            return 2;
        }
        tour.out_dir = out_path;
        tour.width = width_set ? width : 1280u;
        tour.height = height_set ? height : 720u;
        tour.blur = blur;
        return spade::sandbox::live::run_live_smoke(tour);
    }
    if (!tour.label.empty() || !tour.injections.empty() || !tour.assets_dir.empty()) {
        std::fprintf(stderr, "spade_sandbox: --label, --inject and --assets apply to --live-smoke only\n");
        return 2;
    }
    // REFUSES rather than guessing. Two modes named at once is not a
    // preference the tool can resolve, and picking one would make the other
    // flag silently inert.
    if (headless && windowed) {
        std::fprintf(stderr, "spade_sandbox: --headless and --window are mutually exclusive\n");
        return 2;
    }
    // The drone sim box is the only scene: the scene builder is retired (INT-9).
    if (scene_name != nullptr && std::strcmp(scene_name, "drone") != 0) {
        std::fprintf(stderr, "spade_sandbox: --scene takes drone, the only scene, not '%s'%s\n", scene_name,
                     std::strcmp(scene_name, "builder") == 0
                         ? ": the scene builder is retired (INT-9), and the editor (--editor) does what it did"
                         : "");
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

    if (windowed) {
        std::printf("spade_sandbox: opening a %ux%u window on the drone sim box (--headless for the CI surface)\n",
                    width, height);
        return run_windowed_drone(width, height, blur, vsync);
    }
    return run_headless_drone(width, height, blur, out_path);
}
