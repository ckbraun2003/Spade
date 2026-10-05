// See scene_sessions.hpp. The helpers below moved here from main.cpp
// unchanged, comments included, when the live smoke became a second caller of
// each scene's frame.

#include "scene_sessions.hpp"

#include <chrono>
#include <cstdio>
#include <utility>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render_gap.hpp"  // the HUD line naming what the GPU path does not draw

namespace spade::sandbox {

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
bool build_builtin_scene(spade::render::RenderScene& out, spade::WorldDesc& world_out) {
    spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                .name("sandbox_c0")
                                                .environment(spade::Environment{})
                                                .capacities(spade::Capacities{4, 4, 1, 1})
                                                .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                .build();
    if (!world) {
        std::fprintf(stderr, "spade_sandbox: WorldBuilder::build failed: %s\n", world.error().context.c_str());
        return false;
    }
    // Moved into the CALLER's storage FIRST, so the scene below is built
    // against the object that will outlive this function -- not against a
    // temporary whose address the scene would then keep.
    world_out = std::move(*world);
    const spade::Result<spade::render::RenderScene> scene = spade::render::scene_from_world(world_out, {});
    if (!scene) {
        std::fprintf(stderr, "spade_sandbox: scene_from_world failed: %s\n", scene.error().context.c_str());
        return false;
    }
    out = *scene;
    return true;
}

// An empty world (no ground): the drone hangs in air. The WorldDesc is an
// out-parameter for the same reason build_builtin_scene's is -- the scene
// borrows its sdf.
bool build_drone_scene(spade::render::RenderScene& out, spade::WorldDesc& world_out) {
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

// C1: renders and hands the target to a TargetSink. The sink is a PARAMETER
// rather than something this function picks, which is the whole point of the
// seam -- C2's windowed sink substitutes here without touching a line below,
// and that is not a claim, it is what this function's signature enforces.
bool render_frame(const spade::render::RenderScene& scene, const spade::render::Camera& camera, uint32_t width,
                  uint32_t height, const spade::render::RenderOptions& options, std::vector<uint8_t>& pixels,
                  TargetSink& sink) {
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

// The drone frame's draw list and options, shared by the window and headless.
spade::render::RenderOptions drone_frame(spade::render::RenderScene& scene, const DroneDrawBinding& binding,
                                         const DroneSim& drone, const glm::quat& orientation,
                                         const spade::render::Camera& camera, bool heatmap, float heatmap_max,
                                         float blur, float* observed_max) {
    scene.dynamics.clear();
    scene.field_layers.clear();
    append_drone_items(binding, drone.params(), glm::vec3(0.0f), orientation, scene.dynamics);
    if (heatmap) {
        if (const auto field = air_field_from(drone)) {
            scene.field_layers.push_back(slice_layer(camera_facing_slice(camera, glm::vec3(0.0f), orientation),
                                                     *field, heatmap_max, observed_max));
        }
    }
    spade::render::RenderOptions o;
    o.overlays = false;     // no ground, no bounds worth drawing
    o.ground_grid = false;  // and no ground for the grid to sit on
    // The design's "shadows off in the heatmap view". A no-op in this scene
    // either way: the shadow map is baked from statics at world load, the
    // empty world has none, the drone's parts neither cast nor receive, and a
    // field layer takes no shadow.
    o.shadows = !heatmap;
    // The layer itself never takes the atmospheric term (field_layer.hpp,
    // SR-17a). The drone's parts drop it in the heatmap view too, so the
    // whole frame reads against a clean sky, as it did before the move to
    // field layers.
    o.horizon_blend_strength = heatmap ? 0.0f : blur;
    return o;
}

// ⭐ GPU FIRST, CPU FALLBACK -- the user's ruling, implemented as a branch
// rather than as a preference. If the GPU renderer refuses, we say WHY and
// keep going on raster_cpu; we do not exit, because a fallback that aborts is
// not a fallback. Shared by both scenes so they cannot disagree about it.
std::unique_ptr<GpuRenderer> start_gpu(const spade::render::RenderScene& scene) {
    std::unique_ptr<GpuRenderer> gpu;
    spade::Result<std::unique_ptr<GpuRenderer>> made = GpuRenderer::create(GlTargetSink::proc_loader());
    if (made) {
        gpu = std::move(*made);
        spade::Result<void> uploaded = gpu->upload_scene(scene);
        if (!uploaded) {
            std::fprintf(stderr,
                         "spade_sandbox: GPU upload failed, falling back to the CPU "
                         "rasteriser: %s\n",
                         uploaded.error().context.c_str());
            gpu.reset();
        }
    } else {
        std::fprintf(stderr,
                     "spade_sandbox: GPU renderer unavailable, using the CPU "
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
std::unique_ptr<GlTargetSink> open_window(uint32_t width, uint32_t height, bool vsync) {
    GlTargetSink::Options opts;
    opts.width = width;
    opts.height = height;
    opts.title = "spade sandbox";
    opts.vsync = vsync;
    std::string why_not;
    std::unique_ptr<GlTargetSink> sink = GlTargetSink::create(opts, &why_not);
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
    const GlTargetSink::GlInfo gl = sink->gl_info();
    std::printf("spade_sandbox: GL_RENDERER %s\n", gl.renderer.c_str());
    std::printf("spade_sandbox: GL_VERSION  %s\n", gl.version.c_str());
    std::printf("spade_sandbox: GL_VENDOR   %s\n", gl.vendor.c_str());
    return sink;
}

namespace {
[[nodiscard]] float ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

// ---------------------------------------------------------------------------
// THE DRONE SIM BOX -- the default scene. A quadrotor on a test stand, flown
// in attitude through the engine's rotor model, with the air around it as an
// optional heatmap. Every decision is in drone_sim.hpp / drone_view.hpp and is
// asserted with no display; this is the wiring.
// ---------------------------------------------------------------------------
spade::Result<std::unique_ptr<DroneSession>> DroneSession::create(GlTargetSink& sink, float blur) {
    std::unique_ptr<DroneSession> s(new DroneSession());
    s->sink_ = &sink;
    s->blur_ = blur;
    spade::Result<DroneSim> drone = DroneSim::create(DronePhysicsOptions{});
    if (!drone) {
        return std::unexpected(
            spade::Error{drone.error().code, "the drone stand would not build: " + drone.error().context});
    }
    s->drone_.emplace(std::move(*drone));
    if (!build_drone_scene(s->scene_, s->world_)) {
        return std::unexpected(spade::Error{spade::Code::internal, "the drone scene would not build (see above)"});
    }
    s->binding_ = bind_drone_scene(s->scene_, s->drone_->params());
    // Uploaded once: the drone's meshes and the palette never change, only
    // the per-frame draw items do.
    s->gpu_ = start_gpu(s->scene_);

    s->panel_.edited = s->drone_->options();
    s->panel_.render_path = s->gpu_ ? "GPU (OpenGL)" : "CPU raster";
    sink.attach_drone(&s->panel_);

    s->camera_.target = glm::vec3(0.0f);
    s->camera_.yaw = 0.6f;
    s->camera_.pitch = 0.35f;
    s->camera_.distance = 1.8f;
    return s;
}

DroneSession::~DroneSession() {
    if (sink_ != nullptr) {
        sink_->attach_drone(nullptr);
        sink_->set_gap_line({});
    }
}

spade::Result<void> DroneSession::frame(const FrameInput& in, float dt) {
    apply_drone_orbit(camera_, in, dt);
    nudge_attitude(drone_->target, in, dt);
    if (in.toggle_view_pressed && !in.ui_captured_keyboard) {
        panel_.view_heatmap = !panel_.view_heatmap;
    }
    // Debounced inside: nothing happens while a slider is held.
    if (apply_panel_edits(*drone_, panel_)) {
        ++rebuilds_;
    }

    const auto p0 = std::chrono::steady_clock::now();
    if (const spade::Result<void> stepped = drone_->advance(dt); !stepped) {
        return std::unexpected(
            spade::Error{stepped.error().code, "the drone step failed: " + stepped.error().context});
    }
    const float physics_ms = ms_since(p0);
    panel_.readouts = drone_->readouts();
    panel_.target = drone_->target;

    const uint32_t fw = sink_->framebuffer_width();
    const uint32_t fh = sink_->framebuffer_height();
    if (fw == 0u || fh == 0u) {
        return {};  // minimised: the stand keeps stepping, capped by the accumulator
    }
    const spade::render::Camera rc = camera_.to_render_camera();
    const spade::render::RenderOptions options =
        drone_frame(scene_, binding_, *drone_, panel_.readouts.orientation, rc, panel_.view_heatmap,
                    panel_.heatmap_max, blur_, &panel_.observed_max);

    // SL10: the HUD names what the active path does not draw.
    sink_->set_gap_line(gpu_ ? render_gap_line(GpuRenderer::unhonoured(options)) : std::string{});
    const auto r0 = std::chrono::steady_clock::now();
    if (gpu_) {
        sink_->begin_gpu_frame();
        if (const spade::Result<void> drew = gpu_->draw(scene_, rc, options, fw, fh); !drew) {
            return std::unexpected(spade::Error{drew.error().code, "GPU draw failed: " + drew.error().context});
        }
        sink_->present_overlay(ms_since(r0), physics_ms);
    } else {
        sink_->note_physics_ms(physics_ms);
        const bool ok = render_frame(scene_, rc, fw, fh, options, pixels_, *sink_);
        sink_->note_render_ms(ms_since(r0));
        if (!ok) {
            return std::unexpected(spade::Error{spade::Code::internal, "the CPU render failed (see above)"});
        }
    }
    return {};
}

// ---------------------------------------------------------------------------
// THE BUILDER. The model is the application's; every decision it makes lives
// in builder_scene.hpp and is exercised by tests/ with no display. What
// remains here is the wiring, and it is deliberately dull.
// ---------------------------------------------------------------------------
spade::Result<std::unique_ptr<BuilderSession>> BuilderSession::create(GlTargetSink& sink, bool grid, float blur) {
    std::unique_ptr<BuilderSession> s(new BuilderSession());
    s->sink_ = &sink;
    if (!build_builtin_scene(s->scene_, s->world_)) {
        return std::unexpected(spade::Error{spade::Code::internal, "the builder scene would not build (see above)"});
    }
    s->options_.ground_grid = grid;
    s->options_.horizon_blend_strength = blur;
    s->gpu_ = start_gpu(s->scene_);

    s->builder_.grid = grid;
    // Attached once, so BOTH present paths draw the panel -- the GPU path
    // through present_overlay() and the CPU fallback through accept().
    sink.attach_builder(&s->builder_);
    s->bind_ = bind_builder_meshes(s->scene_);
    sync_builder_materials(s->builder_, s->bind_, s->scene_);
    // Cleared because the upload below IS the sync the flag was asking for.
    // Left set, the first frame would ask for a second identical upload --
    // harmless, and exactly the kind of "harmless" that gets copied.
    s->builder_.materials_dirty = false;
    // The model starts from whatever the scene's own lighting is, so the
    // slider's initial position matches the picture rather than snapping it
    // the first time it is touched.
    s->builder_.sun_intensity = s->scene_.lighting.sun_intensity;
    s->builder_.sun_direction = s->scene_.lighting.sun_direction;
    // ⚠ THE BUILDER'S GEOMETRY MUST REACH THE GPU BEFORE THE FIRST DRAW. The
    // upload in start_gpu() happened before these three meshes existed, so
    // without this the first placed object draws from a mesh id the GPU has
    // never seen.
    if (s->gpu_) {
        if (const spade::Result<void> re = s->gpu_->upload_scene(s->scene_); !re) {
            return std::unexpected(
                spade::Error{re.error().code, "builder mesh upload failed: " + re.error().context});
        }
    }
    return s;
}

BuilderSession::~BuilderSession() {
    if (sink_ != nullptr) {
        sink_->attach_builder(nullptr);
        sink_->set_gap_line({});
    }
}

spade::Result<void> BuilderSession::frame(const FrameInput& in) {
    camera_.apply(in, sink_->delta_seconds());

    // The FRAMEBUFFER size, never the window size: on a HiDPI display they
    // differ, and rendering at window size then letting GL stretch it is
    // precisely the blocky-viewport defect this estate has already diagnosed
    // once in the editor.
    const uint32_t fw = sink_->framebuffer_width();
    const uint32_t fh = sink_->framebuffer_height();
    if (fw == 0u || fh == 0u) {
        return {};  // minimised; there is nothing to render into
    }

    // ----- THE BUILDER, ONE CALL ------------------------------------
    // Everything the mouse means happens inside this function, against the
    // camera THIS frame is about to render from. Using the previous frame's
    // camera here would make picking lag the view by one frame, which is
    // invisible while the camera is still and wrong while it is moving -- the
    // hardest kind of bug to see in a viewport.
    const spade::render::Camera rc = camera_.to_render_camera();
    const BuilderFrameResult bf = apply_builder_input(builder_, in, rc, fw, fh, scene_.ground_y);
    if (bf.needs_upload) {
        // ONLY WHEN THE SET CHANGED. A per-frame upload would work and would
        // silently throw away the entire reason the GPU path is fast -- the
        // defect would show up as a frame rate, never as a wrong picture.
        sync_builder_materials(builder_, bind_, scene_);
        if (gpu_) {
            if (const spade::Result<void> re = gpu_->upload_scene(scene_); !re) {
                return std::unexpected(spade::Error{re.error().code, "re-upload failed: " + re.error().context});
            }
        }
    }
    rebuild_builder_dynamics(builder_, bind_, scene_);

    // Scene settings the inspector owns, applied to what actually renders.
    options_.ground_grid = builder_.grid;
    scene_.lighting.sun_intensity = builder_.sun_intensity;
    // NORMALISED HERE RATHER THAN IN THE WIDGET. Both paths shade with
    // dot(n, sun_direction) and assume a unit vector; a dragged direction is
    // whatever the user left it at, and a non-unit one scales the whole
    // diffuse term -- which reads as the brightness slider being broken.
    if (glm::length(builder_.sun_direction) > 1e-4f) {
        scene_.lighting.sun_direction = glm::normalize(builder_.sun_direction);
    }
    // The application owns the render call, so it times its own cost. Both
    // paths are timed the SAME WAY and feed the SAME HUD, which is what makes
    // the two comparable at all.
    // SL10: the HUD names what the active path does not draw, every frame,
    // because the options and the path can both change.
    sink_->set_gap_line(gpu_ ? render_gap_line(GpuRenderer::unhonoured(options_)) : std::string{});
    const auto r0 = std::chrono::steady_clock::now();
    if (gpu_) {
        sink_->begin_gpu_frame();
        if (const spade::Result<void> drew = gpu_->draw(scene_, rc, options_, fw, fh); !drew) {
            return std::unexpected(spade::Error{drew.error().code, "GPU draw failed: " + drew.error().context});
        }
        // physics_ms is 0 here and that is an ABSENCE, not an omission: this
        // scene is a static ground plane, so there is no physics step in the
        // frame to time. The entry exists so that when bodies arrive the split
        // can attribute them.
        sink_->present_overlay(ms_since(r0), 0.0f);
    } else {
        const bool ok = render_frame(scene_, rc, fw, fh, options_, pixels_, *sink_);
        sink_->note_render_ms(ms_since(r0));
        if (!ok) {
            return std::unexpected(spade::Error{spade::Code::internal, "the CPU render failed (see above)"});
        }
    }
    return {};
}

}  // namespace spade::sandbox
