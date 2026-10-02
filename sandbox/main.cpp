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
#include <memory>
#include <string>
#include <utility>  // std::move -- the WorldDesc moves into the caller's storage
#include <vector>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

#include "render_gl/gl_renderer.hpp"  // v2's GPU render backend -- the PRIMARY path

#include "builder_scene.hpp"   // the builder's object model and its whole interaction
#include "drone_view.hpp"      // the drone sim box: stand, controller, air field, heatmap (+ drone_sim.hpp)
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
    std::puts("  --scene <name>        drone (default): the drone sim box; builder: the scene builder");
    std::puts("  --view <name>         headless drone only: standard (default) or heatmap");
    std::puts("  --out <path>          write the frame as a binary PPM (default: no file, render only)");
    std::puts("  --width/--height <n>  frame size (headless 320x180, window 1280x720)");
    std::puts("  --no-vsync            do not wait for the display refresh");
    std::puts("  --smoke               drive the builder from a SCRIPT, assert, and exit");
    std::puts("                        non-zero if any check fails -- the re-runnable demo");
    std::puts("  --no-grid             disable the infinite analytic ground grid (builder)");
    std::puts("  --horizon-blur <f>    SR-17a atmospheric strength, 0 = off (default 0)");
    std::puts("  --help                this text\n");
    std::puts("In the window -- THE DRONE SIM BOX (default):");
    std::puts("  arrows         pitch and roll (held; the target holds when released)   Z/X yaw");
    std::puts("  R              level      V standard <-> air-speed heatmap");
    std::puts("  A/D around     E/Q over/under     W/S nearer/further (0.5-6 m)     drag orbit");
    std::puts("  the panel      wind, turbulence, density, gravity, throttle, backend; readouts");
    std::puts("In the window -- THE BUILDER (--scene builder):");
    std::puts("  LEFT click     place the selected primitive on the ground, or select an object");
    std::puts("  LEFT drag      move the selected object in the ground plane");
    std::puts("  RIGHT drag     orbit    scroll dolly    WASD pan    Q/E lower/raise");
    std::puts("  Del            delete the selection      Ctrl+D duplicate it");
    std::puts("  the panel      shape palette, hierarchy, per-object transform/colour/physics");
    std::puts("  F1 hides the HUD, Esc quits.");
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
// ⚠⚠ THE WORLD IS AN OUT-PARAMETER AND THAT IS A BUG FIX, NOT A STYLE CHOICE.
// scene_from_world() ends with `scene.sdf = &world.sdf` (scene.cpp:415) and
// its own header states the contract in one line: "the returned RenderScene
// must not outlive `world`." This function used to build the WorldDesc as a
// FUNCTION LOCAL and return the scene, so `out.sdf` DANGLED the moment it
// returned -- a use-after-free with a documented contract sitting above it.
//
// ⭐ IT WAS LATENT RATHER THAN HARMLESS, AND THE DIFFERENCE MATTERS: `sdf` is
// dereferenced only by the raymarch and agreement paths, and the sandbox
// renders shaded, so nothing touches it TODAY. The first person to add a
// --mode raymarch flag would have inherited a crash with no plausible
// connection to their change. ***A DEFECT THAT IS UNREACHABLE IS NOT A DEFECT
// THAT IS FIXED; IT IS ONE WHOSE BILL GOES TO SOMEBODY ELSE.***
//
// Found by reading this file while waiting for a build slot, not by a test --
// no test could have caught it, because nothing exercises the path.
[[nodiscard]] bool build_builtin_scene(spade::render::RenderScene& out,
                                       spade::WorldDesc& world_out) {
    spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
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
    // Moved into the CALLER's storage FIRST, so the scene below is built
    // against the object that will outlive this function -- not against a
    // temporary whose address the scene would then keep.
    world_out = std::move(*world);
    const spade::Result<spade::render::RenderScene> scene =
        spade::render::scene_from_world(world_out, {});
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
                                uint32_t height, const spade::render::RenderOptions& options,
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
// --smoke -- THE BUILDER, DRIVEN BY A SCRIPT INSTEAD OF BY A HAND.
//
// ⭐⭐⭐ WHY THIS EXISTS AT ALL. The evidence owed for the builder is "place a
// box, select it, drag it, recolour it, delete it" -- and the agent that built
// it cannot hold a mouse. The answer is NOT to ask a human: a human clicking
// produces "it looked right when I dragged it", which is not a claim anyone
// can re-run, and that is the exact sentence this builder's architecture was
// designed around. Feeding synthesised FrameInputs through the REAL window,
// the REAL GPU renderer and the REAL builder gives the same evidence and gives
// it every time anyone asks.
//
// ⚠⚠ AND IT MUST BE ABLE TO FAIL, OR IT IS A SCREENSHOT TOOL WITH OPINIONS.
// Every scripted action carries an assertion about what the model or the
// renderer should now report, each one prints its ACTUAL value beside its
// expectation, and the process exits non-zero if any of them did not hold.
// A smoke that always exits 0 measures that the smoke ran.
// ---------------------------------------------------------------------------
namespace {

struct SmokeTally {
    int checks = 0;
    int failed = 0;

    void check(bool ok, const char* what, const std::string& detail) {
        ++checks;
        if (!ok) {
            ++failed;
        }
        std::printf("  [%s] %-46s %s\n", ok ? "PASS" : "FAIL", what, detail.c_str());
    }
};

[[nodiscard]] std::string num(long long v) { return std::to_string(v); }

template <typename RenderOne>
[[nodiscard]] int run_smoke(spade::sandbox::GlTargetSink& sink,
                            spade::sandbox::BuilderScene& builder,
                            spade::render::RenderScene& scene,
                            spade::render_gl::GlRenderer* gpu, RenderOne&& render_one) {
    using spade::sandbox::FrameInput;
    SmokeTally t;

    // Pump N frames with a given input. The poll() is what keeps the window
    // responsive to the OS -- its INPUT is discarded, because the whole point
    // is that the script drives, not the mouse.
    const auto frames = [&](const FrameInput& in, int n) -> bool {
        for (int i = 0; i < n; ++i) {
            const FrameInput real = sink.poll();
            if (real.want_close) {
                return false;
            }
            if (!render_one(in)) {
                return false;
            }
        }
        return true;
    };
    const auto draws = [&]() -> uint32_t { return gpu != nullptr ? gpu->last_draw_calls() : 0u; };

    // Settle: let the window map and the first real frame happen before any
    // measurement. A first frame includes one-time GL work and is not
    // representative of anything.
    if (!frames(FrameInput{}, 4)) {
        std::fprintf(stderr, "spade_sandbox: smoke aborted -- window closed during settle\n");
        return 2;
    }

    const uint32_t fw = sink.framebuffer_width();
    const uint32_t fh = sink.framebuffer_height();
    std::printf("\nspade_sandbox: SMOKE -- scripted builder session at %ux%u\n", fw, fh);
    std::printf("  GPU renderer: %s\n", gpu != nullptr ? "ACTIVE" : "ABSENT (CPU fallback)");

    const uint32_t draws_empty = draws();

    // Three ground points, chosen in the lower half where the ground is.
    const float x1 = static_cast<float>(fw) * 0.50f, y1 = static_cast<float>(fh) * 0.72f;
    const float x2 = static_cast<float>(fw) * 0.38f, y2 = static_cast<float>(fh) * 0.66f;
    const float x3 = static_cast<float>(fw) * 0.62f, y3 = static_cast<float>(fh) * 0.66f;

    const auto click = [](float x, float y) {
        FrameInput in;
        in.mouse_x = x;
        in.mouse_y = y;
        in.left_click = true;
        in.left_down = true;
        return in;
    };

    // ----- PLACE ---------------------------------------------------------
    builder.placing = true;
    builder.pending_shape = spade::sandbox::Shape::box;
    if (!frames(click(x1, y1), 1)) return 2;
    t.check(builder.objects.size() == 1u, "click on empty ground places a box",
            "objects=" + num(static_cast<long long>(builder.objects.size())) + " expected 1");
    t.check(builder.selected == 0, "the new object is selected",
            "selected=" + num(builder.selected) + " expected 0");

    builder.pending_shape = spade::sandbox::Shape::sphere;
    if (!frames(click(x2, y2), 1)) return 2;
    builder.pending_shape = spade::sandbox::Shape::cylinder;
    if (!frames(click(x3, y3), 1)) return 2;
    t.check(builder.objects.size() == 3u, "a sphere and a cylinder place too",
            "objects=" + num(static_cast<long long>(builder.objects.size())) + " expected 3");
    t.check(scene.dynamics.size() == builder.objects.size(),
            "every object reached the render scene",
            "dynamics=" + num(static_cast<long long>(scene.dynamics.size())) + " objects=" +
                num(static_cast<long long>(builder.objects.size())));

    // ⚠ THE OBJECTS MUST REST ON THE GROUND, NOT IN IT. Asserted here against
    // the renderer's own ground_y rather than against 0, so a scene whose
    // ground moves does not silently make this vacuous.
    bool all_on_ground = true;
    for (const spade::sandbox::BuilderObject& o : builder.objects) {
        if (std::fabs(o.position.y - (scene.ground_y + o.scale.y * 0.5f)) > 1e-3f) {
            all_on_ground = false;
        }
    }
    t.check(all_on_ground, "placed objects rest ON the ground plane",
            "ground_y=" + std::to_string(scene.ground_y));

    const uint32_t draws_three = draws();

    // ⚠⚠ STOP HERE IF PLACEMENT DID NOT HAPPEN, AND STOP BY REPORTING RATHER
    // THAN BY CRASHING. Every step below indexes objects[0] and objects[2];
    // if a scripted click missed the ground, those are out-of-bounds reads on
    // an empty vector and this process dies with no output at all.
    // ⭐ A SMOKE THAT CRASHES INSTEAD OF REPORTING IS WORSE THAN USELESS: the
    // failure it was built to describe is exactly the failure that silences
    // it. Found by re-reading, not by running -- nothing has run yet.
    if (builder.objects.size() < 3u) {
        std::printf("\nspade_sandbox: SMOKE ABORTED -- placement produced %zu of 3 objects, so "
                    "the steps below have nothing to act on.\n",
                    builder.objects.size());
        std::printf("spade_sandbox: SMOKE FAILED -- %d checks, %d failed\n", t.checks, t.failed);
        return 1;
    }

    // ----- SELECT --------------------------------------------------------
    // Placement mode OFF, and a click on the FIRST object's own screen point.
    // That point is where its base was placed, so the ray that reached the
    // ground through it must pass through the object on the way.
    builder.placing = false;
    if (!frames(click(x1, y1), 1)) return 2;
    t.check(builder.selected == 0, "clicking an object selects it rather than placing",
            "selected=" + num(builder.selected) + " expected 0, objects=" +
                num(static_cast<long long>(builder.objects.size())) + " expected 3");
    t.check(builder.objects.size() == 3u, "selecting did NOT add an object",
            "objects=" + num(static_cast<long long>(builder.objects.size())));

    // ----- DRAG ----------------------------------------------------------
    const glm::vec3 before = builder.objects[0].position;
    FrameInput drag;
    drag.mouse_x = x1 + static_cast<float>(fw) * 0.12f;
    drag.mouse_y = y1;
    drag.left_down = true;
    if (!frames(drag, 2)) return 2;
    const glm::vec3 after = builder.objects[0].position;
    t.check(std::fabs(after.x - before.x) > 0.05f, "dragging moves the selection in X",
            "x " + std::to_string(before.x) + " -> " + std::to_string(after.x));
    t.check(std::fabs(after.y - before.y) < 1e-4f, "dragging leaves HEIGHT untouched",
            "y " + std::to_string(before.y) + " -> " + std::to_string(after.y));

    // ----- RECOLOUR ------------------------------------------------------
    builder.objects[0].color = glm::vec4(1.0f, 0.1f, 0.05f, 1.0f);
    builder.materials_dirty = true;
    if (!frames(FrameInput{}, 2)) return 2;
    t.check(scene.materials.size() >= builder.objects.size(),
            "a colour edit reaches the scene palette",
            "materials=" + num(static_cast<long long>(scene.materials.size())) + " objects=" +
                num(static_cast<long long>(builder.objects.size())));

    // ----- DELETE --------------------------------------------------------
    builder.selected = 2;
    builder.delete_request = true;
    if (!frames(FrameInput{}, 1)) return 2;
    t.check(builder.objects.size() == 2u, "delete removes the selection",
            "objects=" + num(static_cast<long long>(builder.objects.size())) + " expected 2");
    t.check(builder.selected == 1, "the selection clamps rather than clearing",
            "selected=" + num(builder.selected) + " expected 1");

    // ----- THE INSTANCING CLAIM, WHICH IS THE WHOLE ARGUMENT FOR THE GPU
    // PATH. One draw call per distinct mesh, NOT per object. Measured by
    // putting ten boxes in the scene and comparing against one.
    builder.objects.clear();
    builder.selected = -1;
    builder.objects.push_back(spade::sandbox::make_object_at(
        spade::sandbox::Shape::box, glm::vec3(0.0f, scene.ground_y, 0.0f), 0));
    builder.materials_dirty = true;
    if (!frames(FrameInput{}, 2)) return 2;
    const uint32_t draws_1box = draws();

    for (int i = 1; i < 10; ++i) {
        builder.objects.push_back(spade::sandbox::make_object_at(
            spade::sandbox::Shape::box,
            glm::vec3(static_cast<float>(i) * 1.5f - 7.0f, scene.ground_y, -2.0f), i));
    }
    builder.materials_dirty = true;
    if (!frames(FrameInput{}, 2)) return 2;
    const uint32_t draws_10box = draws();
    const uint32_t inst_10 = gpu != nullptr ? gpu->last_instances() : 0u;

    if (gpu != nullptr) {
        t.check(draws_10box == draws_1box,
                "10 BOXES COST THE SAME DRAW CALLS AS 1 BOX",
                "1 box=" + num(draws_1box) + "  10 boxes=" + num(draws_10box));
        t.check(inst_10 >= 10u, "and the renderer did issue 10 instances",
                "instances=" + num(inst_10));
        t.check(draws_three > draws_1box,
                "three DISTINCT shapes cost more draw calls than one shape",
                "1 shape=" + num(draws_1box) + "  3 shapes=" + num(draws_three));
    } else {
        std::printf("  [SKIP] draw-call checks -- no GPU renderer on this machine\n");
    }

    // ----- REPORT --------------------------------------------------------
    const spade::sandbox::GlTargetSink::Timings life = sink.lifetime_timings();
    const uint64_t tris = [&]() {
        uint64_t n = 0;
        for (const spade::render::DrawItem& d : scene.dynamics) {
            if (d.mesh_index < scene.meshes.size()) {
                n += scene.meshes[d.mesh_index].indices.size() / 3u;
            }
        }
        for (const spade::render::DrawItem& d : scene.statics) {
            if (d.mesh_index < scene.meshes.size()) {
                n += scene.meshes[d.mesh_index].indices.size() / 3u;
            }
        }
        return n;
    }();

    std::printf("\nspade_sandbox: SMOKE NUMBERS -- every figure with its population beside it\n");
    std::printf("  resolution        %ux%u\n", fw, fh);
    std::printf("  objects on screen %zu  (+ %zu static)\n", scene.dynamics.size(),
                scene.statics.size());
    std::printf("  triangles drawn   %llu\n", static_cast<unsigned long long>(tris));
    std::printf("  draw calls        empty=%u  1 box=%u  10 boxes=%u  3 shapes=%u\n", draws_empty,
                draws_1box, draws_10box, draws_three);
    std::printf("  frames presented  %llu\n", static_cast<unsigned long long>(sink.presented()));
    std::printf("  mean frame        %.3f ms  (%.1f fps)\n", life.total_ms,
                life.total_ms > 0.0f ? 1000.0f / life.total_ms : 0.0f);
    std::printf("    render %.3f  ui %.3f  swap %.3f ms\n", life.render_ms, life.ui_ms,
                life.swap_ms);
    std::printf("  scene composition %zu meshes  %zu materials  %zu ground_planes\n",
                scene.meshes.size(), scene.materials.size(), scene.ground_planes.size());
    std::printf("\nspade_sandbox: SMOKE %s -- %d checks, %d failed\n",
                t.failed == 0 ? "PASSED" : "FAILED", t.checks, t.failed);
    // ⚠ THE FPS ABOVE IS NOT COMPARABLE TO V1's. It is 10 boxes at this
    // resolution with vsync possibly on -- say the population or the number is
    // not a measurement.
    return t.failed == 0 ? 0 : 1;
}

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

// ⭐ GPU FIRST, CPU FALLBACK -- the user's ruling, implemented as a branch
// rather than as a preference. If the GPU renderer refuses, we say WHY and
// keep going on raster_cpu; we do not exit, because a fallback that aborts is
// not a fallback. Shared by both scenes so they cannot disagree about it.
[[nodiscard]] std::unique_ptr<spade::render_gl::GlRenderer> start_gpu(const spade::render::RenderScene& scene) {
    std::unique_ptr<spade::render_gl::GlRenderer> gpu;
    spade::Result<std::unique_ptr<spade::render_gl::GlRenderer>> made =
        spade::render_gl::GlRenderer::create(spade::sandbox::GlTargetSink::proc_loader());
    if (made) {
        gpu = std::move(*made);
        spade::Result<void> uploaded = gpu->upload_scene(scene);
        if (!uploaded) {
            std::fprintf(stderr, "spade_sandbox: GPU upload failed, falling back to the CPU "
                                 "rasteriser: %s\n",
                         uploaded.error().context.c_str());
            gpu.reset();
        }
    } else {
        std::fprintf(stderr, "spade_sandbox: GPU renderer unavailable, using the CPU "
                             "rasteriser: %s\n",
                     made.error().context.c_str());
    }
    if (gpu) {
        std::printf("spade_sandbox: GPU path ACTIVE -- %s, %s\n", gpu->renderer_name().c_str(),
                    gpu->version_string().c_str());
    } else {
        std::printf("spade_sandbox: CPU fallback path (no GPU renderer)\n");
    }
    return gpu;
}

// Opens the window, or explains why not. ⚠⚠ THE REFUSAL IS THE DELIVERABLE,
// NOT AN ERROR PATH: it names the cause and the caller exits non-zero. A
// missing capability refuses, never degrades -- a window that silently fails
// to appear is the worst possible outcome for someone judging whether the GUI
// works.
[[nodiscard]] std::unique_ptr<spade::sandbox::GlTargetSink> open_window(uint32_t width, uint32_t height,
                                                                        bool vsync) {
    spade::sandbox::GlTargetSink::Options opts;
    opts.width = width;
    opts.height = height;
    opts.title = "spade sandbox";
    opts.vsync = vsync;
    std::string why_not;
    std::unique_ptr<spade::sandbox::GlTargetSink> sink =
        spade::sandbox::GlTargetSink::create(opts, &why_not);
    if (!sink) {
        std::fprintf(stderr, "spade_sandbox: cannot open a window -- %s\n", why_not.c_str());
        std::fprintf(stderr, "spade_sandbox: --headless still works in this build.\n");
        return nullptr;
    }
    // ⚠⚠ WHICH GL IMPLEMENTATION ARE WE ACTUALLY ON. Printed unconditionally
    // rather than behind a flag, because it decides how every timing below is
    // read: on Microsoft's GDI GENERIC software GL, SwapBuffers is a CPU blit
    // of the framebuffer to the window, so it costs PER PIXEL rather than per
    // frame -- and a per-pixel swap is indistinguishable from "the rasteriser
    // is slow" in any aggregate number.
    const spade::sandbox::GlTargetSink::GlInfo gl = sink->gl_info();
    std::printf("spade_sandbox: GL_RENDERER %s\n", gl.renderer.c_str());
    std::printf("spade_sandbox: GL_VERSION  %s\n", gl.version.c_str());
    std::printf("spade_sandbox: GL_VENDOR   %s\n", gl.vendor.c_str());
    return sink;
}

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

int run_windowed(spade::render::RenderScene& scene, uint32_t width, uint32_t height,
                 bool grid, float blur, bool vsync, bool smoke) {
    spade::render::RenderOptions gpu_options;
    gpu_options.ground_grid = grid;
    gpu_options.horizon_blend_strength = blur;

    std::unique_ptr<spade::sandbox::GlTargetSink> sink = open_window(width, height, vsync);
    if (!sink) {
        return 3;
    }

    std::unique_ptr<spade::render_gl::GlRenderer> gpu = start_gpu(scene);

    spade::sandbox::OrbitCamera camera;
    std::vector<uint8_t> pixels;

    // THE BUILDER. The model is the application's; every decision it makes
    // lives in builder_scene.hpp and is exercised by tests/ with no
    // display. What remains here is the wiring, and it is deliberately dull.
    spade::sandbox::BuilderScene builder;
    builder.grid = grid;
    // Attached once, so BOTH present paths draw the panel -- the GPU path
    // through present_overlay() and the CPU fallback through accept().
    sink->attach_builder(&builder);
    const spade::sandbox::BuilderBinding bind = spade::sandbox::bind_builder_meshes(scene);
    spade::sandbox::sync_builder_materials(builder, bind, scene);
    // Cleared because the upload below IS the sync the flag was asking for.
    // Left set, the first frame would ask for a second identical upload --
    // harmless, and exactly the kind of "harmless" that gets copied.
    builder.materials_dirty = false;
    // The model starts from whatever the scene's own lighting is, so the
    // slider's initial position matches the picture rather than snapping it
    // the first time it is touched.
    builder.sun_intensity = scene.lighting.sun_intensity;
    builder.sun_direction = scene.lighting.sun_direction;
    // ⚠ THE BUILDER'S GEOMETRY MUST REACH THE GPU BEFORE THE FIRST DRAW. The
    // upload above happened before these three meshes existed, so without this
    // the first placed object draws from a mesh id the GPU has never seen.
    if (gpu) {
        const spade::Result<void> re = gpu->upload_scene(scene);
        if (!re) {
            std::fprintf(stderr, "spade_sandbox: builder mesh upload failed: %s\n",
                         re.error().context.c_str());
            return 1;
        }
    }

    // ⭐⭐⭐ ONE FRAME, ONE STATEMENT, CALLED BY BOTH THE INTERACTIVE LOOP AND
    // THE SMOKE. Extracted rather than copied, for the reason this file has
    // already refused twice: two copies of a render path are two things that
    // can disagree, and the smoke's whole value is that it exercises THE SAME
    // path a user does. A smoke with its own private frame loop proves that
    // the smoke works.
    //
    // Returns 0 on success and a non-zero exit code on a hard render failure.
    int frame_error = 0;
    const auto render_one = [&](const spade::sandbox::FrameInput& in) -> bool {
        camera.apply(in, sink->delta_seconds());

        // The FRAMEBUFFER size, never the window size: on a HiDPI display they
        // differ, and rendering at window size then letting GL stretch it is
        // precisely the blocky-viewport defect this estate has already
        // diagnosed once in the editor.
        const uint32_t fw = sink->framebuffer_width();
        const uint32_t fh = sink->framebuffer_height();
        if (fw == 0u || fh == 0u) {
            return true;  // minimised; there is nothing to render into
        }

        // ----- THE BUILDER, ONE CALL ------------------------------------
        // Everything the mouse means happens inside this function, against
        // the camera THIS frame is about to render from. Using the previous
        // frame's camera here would make picking lag the view by one frame,
        // which is invisible while the camera is still and wrong while it is
        // moving -- the hardest kind of bug to see in a viewport.
        const spade::render::Camera rc = camera.to_render_camera();
        const spade::sandbox::BuilderFrameResult bf =
            spade::sandbox::apply_builder_input(builder, in, rc, fw, fh, scene.ground_y);
        if (bf.needs_upload) {
            // ONLY WHEN THE SET CHANGED. A per-frame upload would work and
            // would silently throw away the entire reason the GPU path is
            // fast -- the defect would show up as a frame rate, never as a
            // wrong picture.
            spade::sandbox::sync_builder_materials(builder, bind, scene);
            if (gpu) {
                const spade::Result<void> re = gpu->upload_scene(scene);
                if (!re) {
                    std::fprintf(stderr, "spade_sandbox: re-upload failed: %s\n",
                                 re.error().context.c_str());
                    frame_error = 1;
                    return false;
                }
            }
        }
        spade::sandbox::rebuild_builder_dynamics(builder, bind, scene);

        // Scene settings the inspector owns, applied to what actually renders.
        gpu_options.ground_grid = builder.grid;
        scene.lighting.sun_intensity = builder.sun_intensity;
        // NORMALISED HERE RATHER THAN IN THE WIDGET. Both paths shade with
        // dot(n, sun_direction) and assume a unit vector; a dragged direction is
        // whatever the user left it at, and a non-unit one scales the whole
        // diffuse term -- which reads as the brightness slider being broken.
        if (glm::length(builder.sun_direction) > 1e-4f) {
            scene.lighting.sun_direction = glm::normalize(builder.sun_direction);
        }
        // The application owns the render call, so it times its own cost.
        // Both paths are timed the SAME WAY and feed the SAME HUD, which is
        // what makes the two comparable at all.
        const auto r0 = std::chrono::steady_clock::now();
        if (gpu) {
            sink->begin_gpu_frame();
            const spade::Result<void> drew = gpu->draw(scene, rc, gpu_options, fw, fh);
            if (!drew) {
                std::fprintf(stderr, "spade_sandbox: GPU draw failed: %s\n",
                             drew.error().context.c_str());
                frame_error = 1;
                return false;
            }
            const auto r1 = std::chrono::steady_clock::now();
            // physics_ms is 0 here and that is an ABSENCE, not an omission: this
            // scene is a static ground plane, so there is no physics step in
            // the frame to time. The entry exists so that when bodies arrive
            // the split can attribute them.
            sink->present_overlay(std::chrono::duration<float, std::milli>(r1 - r0).count(), 0.0f);
        } else {
            const bool ok = render_frame(scene, rc, fw, fh, gpu_options, pixels, *sink);
            const auto r1 = std::chrono::steady_clock::now();
            sink->note_render_ms(std::chrono::duration<float, std::milli>(r1 - r0).count());
            if (!ok) {
                frame_error = 1;
                return false;
            }
        }
        return true;
    };

    if (smoke) {
        const int rc_smoke = run_smoke(*sink, builder, scene, gpu.get(), render_one);
        if (rc_smoke != 0) {
            return rc_smoke;
        }
    } else {
        while (!sink->should_close()) {
            const spade::sandbox::FrameInput in = sink->poll();
            if (in.want_close) {
                break;
            }
            if (!render_one(in)) {
                return frame_error;
            }
        }
    }

    print_exit_summary(*sink, width, height, vsync);
    return 0;
}

// ---------------------------------------------------------------------------
// THE DRONE SIM BOX -- the default scene. A quadrotor on a test stand, flown
// in attitude through the engine's rotor model, with the air around it as an
// optional heatmap. Every decision is in drone_sim.hpp / drone_view.hpp and is
// asserted with no display; this is the wiring.
// ---------------------------------------------------------------------------

// An empty world (no ground): the drone hangs in air. The WorldDesc is an
// out-parameter for the same reason build_builtin_scene's is -- the scene
// borrows its sdf.
[[nodiscard]] bool build_drone_scene(spade::render::RenderScene& out, spade::WorldDesc& world_out) {
    spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                .name("drone_sim_box")
                                                .environment(spade::Environment{})
                                                .capacities(spade::Capacities{1, 5, 1, 1})
                                                .build();
    if (!world) {
        std::fprintf(stderr, "spade_sandbox: WorldBuilder::build failed: %s\n", world.error().context.c_str());
        return false;
    }
    world_out = std::move(*world);
    const spade::Result<spade::render::RenderScene> scene = spade::render::scene_from_world(world_out, {});
    if (!scene) {
        std::fprintf(stderr, "spade_sandbox: scene_from_world failed: %s\n", scene.error().context.c_str());
        return false;
    }
    out = *scene;
    return true;
}

// The drone frame's draw list and options, shared by the window and headless.
[[nodiscard]] spade::render::RenderOptions drone_frame(spade::render::RenderScene& scene,
                                                       const spade::sandbox::DroneDrawBinding& binding,
                                                       const spade::sandbox::DroneSim& drone,
                                                       const glm::quat& orientation,
                                                       const spade::render::Camera& camera, bool heatmap,
                                                       float heatmap_max, float blur, float* observed_max) {
    scene.dynamics.clear();
    spade::sandbox::append_drone_items(binding, drone.params(), glm::vec3(0.0f), orientation, scene.dynamics);
    if (heatmap) {
        if (const auto field = spade::sandbox::air_field_from(drone)) {
            const float seen = spade::sandbox::append_slice_items(
                binding, spade::sandbox::camera_facing_slice(camera, glm::vec3(0.0f), orientation), *field, heatmap_max,
                scene.dynamics);
            if (observed_max != nullptr) *observed_max = seen;
        }
    }
    spade::render::RenderOptions o;
    o.overlays = false;     // no ground, no bounds worth drawing
    o.ground_grid = false;  // and no ground for the grid to sit on
    o.shadows = !heatmap;   // the slice would shadow the drone and the drone the slice
    o.horizon_blend_strength = blur;
    return o;
}

int run_windowed_drone(uint32_t width, uint32_t height, float blur, bool vsync) {
    spade::Result<spade::sandbox::DroneSim> drone = spade::sandbox::DroneSim::create(spade::sandbox::DronePhysicsOptions{});
    if (!drone) {
        std::fprintf(stderr, "spade_sandbox: the drone stand would not build: %s\n", drone.error().context.c_str());
        return 1;
    }
    spade::WorldDesc world;
    spade::render::RenderScene scene;
    if (!build_drone_scene(scene, world)) {
        return 1;
    }
    const spade::sandbox::DroneDrawBinding binding = spade::sandbox::bind_drone_scene(scene, drone->params());

    std::unique_ptr<spade::sandbox::GlTargetSink> sink = open_window(width, height, vsync);
    if (!sink) {
        return 3;
    }
    // Uploaded once: the drone's meshes and the palette never change, only
    // the per-frame draw items do.
    std::unique_ptr<spade::render_gl::GlRenderer> gpu = start_gpu(scene);

    spade::sandbox::DronePanelModel panel;
    panel.edited = drone->options();
    panel.render_path = gpu ? "GPU (OpenGL)" : "CPU raster";
    sink->attach_drone(&panel);

    spade::sandbox::OrbitCamera camera;
    camera.target = glm::vec3(0.0f);
    camera.yaw = 0.6f;
    camera.pitch = 0.35f;
    camera.distance = 1.8f;
    std::vector<uint8_t> pixels;

    while (!sink->should_close()) {
        const spade::sandbox::FrameInput in = sink->poll();
        if (in.want_close) {
            break;
        }
        const float dt = sink->delta_seconds();
        spade::sandbox::apply_drone_orbit(camera, in, dt);
        spade::sandbox::nudge_attitude(drone->target, in, dt);
        if (in.toggle_view_pressed && !in.ui_captured_keyboard) {
            panel.view_heatmap = !panel.view_heatmap;
        }
        // Debounced inside: nothing happens while a slider is held.
        (void)spade::sandbox::apply_panel_edits(*drone, panel);

        const auto p0 = std::chrono::steady_clock::now();
        if (const spade::Result<void> stepped = drone->advance(dt); !stepped) {
            std::fprintf(stderr, "spade_sandbox: the drone step failed: %s\n", stepped.error().context.c_str());
            return 1;
        }
        const auto p1 = std::chrono::steady_clock::now();
        const float physics_ms = std::chrono::duration<float, std::milli>(p1 - p0).count();
        panel.readouts = drone->readouts();
        panel.target = drone->target;

        const uint32_t fw = sink->framebuffer_width();
        const uint32_t fh = sink->framebuffer_height();
        if (fw == 0u || fh == 0u) {
            continue;  // minimised: the stand keeps stepping, capped by the accumulator
        }
        const spade::render::Camera rc = camera.to_render_camera();
        const spade::render::RenderOptions options =
            drone_frame(scene, binding, *drone, panel.readouts.orientation, rc, panel.view_heatmap, panel.heatmap_max,
                        blur, &panel.observed_max);

        const auto r0 = std::chrono::steady_clock::now();
        if (gpu) {
            sink->begin_gpu_frame();
            if (const spade::Result<void> drew = gpu->draw(scene, rc, options, fw, fh); !drew) {
                std::fprintf(stderr, "spade_sandbox: GPU draw failed: %s\n", drew.error().context.c_str());
                return 1;
            }
            const auto r1 = std::chrono::steady_clock::now();
            sink->present_overlay(std::chrono::duration<float, std::milli>(r1 - r0).count(), physics_ms);
        } else {
            sink->note_physics_ms(physics_ms);
            const bool ok = render_frame(scene, rc, fw, fh, options, pixels, *sink);
            const auto r1 = std::chrono::steady_clock::now();
            sink->note_render_ms(std::chrono::duration<float, std::milli>(r1 - r0).count());
            if (!ok) {
                return 1;
            }
        }
    }
    print_exit_summary(*sink, width, height, vsync);
    return 0;
}

// --headless --scene drone: deterministic -- 250 fixed steps (0.5 s) with the
// controller holding level, one CPU frame from a fixed camera.
int run_headless_drone(uint32_t width, uint32_t height, float blur, bool heatmap, const char* out_path) {
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
    float observed = 0.0f;
    const spade::render::RenderOptions options =
        drone_frame(scene, binding, *drone, drone->readouts().orientation, camera, heatmap, 0.0f, blur, &observed);

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
    std::printf("spade_sandbox: drone %s view, %ux%u after %llu steps%s%s\n", heatmap ? "heatmap" : "standard",
                width, height, static_cast<unsigned long long>(r.tick), out_path != nullptr ? " -> " : "",
                out_path != nullptr ? out_path : "");
    if (heatmap) {
        std::printf("spade_sandbox:   air speed 0 .. %.2f m/s across the slice\n", static_cast<double>(observed));
    }
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
    bool smoke = false;
    float blur = 0.0f;
    const char* scene_name = nullptr;  // "drone" (the default) or "builder"
    const char* view_name = nullptr;   // drone only: "standard" (default) or "heatmap"

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--help") == 0) {
            print_usage();
            return 0;
        } else if (std::strcmp(a, "--scene") == 0 && i + 1 < argc) {
            scene_name = argv[++i];
        } else if (std::strcmp(a, "--view") == 0 && i + 1 < argc) {
            view_name = argv[++i];
        } else if (std::strcmp(a, "--headless") == 0) {
            headless = true;
        } else if (std::strcmp(a, "--window") == 0) {
            windowed = true;
        } else if (std::strcmp(a, "--no-vsync") == 0) {
            vsync = false;
        } else if (std::strcmp(a, "--smoke") == 0) {
            smoke = true;
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
    // REFUSES rather than silently ignoring. --smoke drives a window, a GPU
    // renderer and a builder; there is nothing for it to do headless, and a
    // flag that quietly does nothing is worse than one that says no.
    if (smoke && headless) {
        std::fprintf(stderr, "spade_sandbox: --smoke drives the window; it cannot run headless\n");
        return 2;
    }
    if (headless && windowed) {
        std::fprintf(stderr, "spade_sandbox: --headless and --window are mutually exclusive\n");
        return 2;
    }
    // THE DRONE SIM BOX IS THE DEFAULT SCENE; the builder stays one flag away,
    // and --smoke scripts the builder, so it takes the builder whatever the
    // default is -- but refuses an explicit request for anything else.
    if (scene_name != nullptr && std::strcmp(scene_name, "drone") != 0 && std::strcmp(scene_name, "builder") != 0) {
        std::fprintf(stderr, "spade_sandbox: --scene takes drone or builder, not '%s'\n", scene_name);
        return 2;
    }
    if (smoke && scene_name != nullptr && std::strcmp(scene_name, "builder") != 0) {
        std::fprintf(stderr, "spade_sandbox: --smoke scripts the builder scene; it cannot run --scene %s\n",
                     scene_name);
        return 2;
    }
    const bool drone_scene = !smoke && (scene_name == nullptr || std::strcmp(scene_name, "drone") == 0);
    if (view_name != nullptr && std::strcmp(view_name, "standard") != 0 && std::strcmp(view_name, "heatmap") != 0) {
        std::fprintf(stderr, "spade_sandbox: --view takes standard or heatmap, not '%s'\n", view_name);
        return 2;
    }
    if (view_name != nullptr && !drone_scene) {
        std::fprintf(stderr, "spade_sandbox: --view applies to the drone scene only\n");
        return 2;
    }
    if (view_name != nullptr && !headless) {
        // In the window V toggles the view; a flag that only sets the first
        // frame would read as broken the moment it was pressed.
        std::fprintf(stderr, "spade_sandbox: --view applies to --headless; in the window press V\n");
        return 2;
    }
    const bool heatmap = view_name != nullptr && std::strcmp(view_name, "heatmap") == 0;
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

    if (drone_scene) {
        if (windowed) {
            std::printf("spade_sandbox: opening a %ux%u window on the drone sim box "
                        "(--scene builder for the builder, --headless for the CI surface)\n",
                        width, height);
            return run_windowed_drone(width, height, blur, vsync);
        }
        return run_headless_drone(width, height, blur, heatmap, out_path);
    }

    // Declared HERE, before the scene, so it is destroyed AFTER it -- the
    // scene holds a non-owning pointer into this world (see
    // build_builtin_scene above), and reverse-declaration order is what makes
    // that safe for the whole of main().
    spade::WorldDesc world;
    spade::render::RenderScene scene;
    if (!build_builtin_scene(scene, world)) {
        return 1;
    }

    if (windowed) {
        std::printf("spade_sandbox: opening a %ux%u window (--headless for the CI surface)\n",
                    width, height);
        return run_windowed(scene, width, height, grid, blur, vsync, smoke);
    }

    // C1: the sink is chosen HERE, by the application, and handed down. An
    // empty path means render-and-discard -- the conversion still runs, so
    // --out changes only where the bytes go, never whether the work happens.
    spade::sandbox::HeadlessTargetSink sink{out_path != nullptr ? std::string(out_path)
                                                                : std::string{}};
    std::vector<uint8_t> pixels;
    spade::render::Camera camera;
    camera.position = glm::vec3(0.0f, 2.0f, 8.0f);
    spade::render::RenderOptions options;
    options.ground_grid = grid;
    options.horizon_blend_strength = blur;
    if (!render_frame(scene, camera, width, height, options, pixels, sink)) {
        return 1;
    }
    if (sink.failed()) {
        std::fprintf(stderr, "spade_sandbox: could not write %s\n",
                     out_path != nullptr ? out_path : "(no path)");
        return 1;
    }

    std::printf("spade_sandbox: rendered %ux%u (%zu bytes)%s%s\n", width, height, pixels.size(),
                out_path != nullptr ? " -> " : "", out_path != nullptr ? out_path : "");
    // ⭐ THE SCENE'S COMPOSITION, PRINTED RATHER THAN INFERRED. Added after an
    // evening of reasoning about what this scene CONTAINS from what the
    // builders ought to produce -- which is the same mistake as reading an
    // implementation instead of a behaviour, one layer up. `ground_planes` is
    // the analytic background candidate list and `statics` is real drawn
    // geometry; whether a plane yields one, the other or both decides what
    // ledger row L300 is even about, and until now nothing reported it.
    std::printf("spade_sandbox:   scene: %zu meshes  %zu statics  %zu dynamics  "
                "%zu materials  %zu ground_planes  has_ground=%d ground_y=%.2f\n",
                scene.meshes.size(), scene.statics.size(), scene.dynamics.size(),
                scene.materials.size(), scene.ground_planes.size(),
                scene.has_ground ? 1 : 0, static_cast<double>(scene.ground_y));
    return 0;
}
