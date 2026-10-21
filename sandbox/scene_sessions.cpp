// See scene_sessions.hpp. The helpers below moved here from main.cpp
// unchanged, comments included, when the live smoke became a second caller of
// each scene's frame.

#include "scene_sessions.hpp"

#include <chrono>
#include <cstdio>
#include <utility>

#include <glm/glm.hpp>

#include "analysis/controls.hpp"
#include "render/raster_cpu.hpp"
#include "render_gap.hpp"     // the HUD line naming what the GPU path does not draw
#include "vehicle_draw.hpp"   // the drone drawn from its parts (N12)

namespace spade::sandbox {

// An empty world (no ground): the drone hangs in air.
//
// ⚠⚠ THE WORLD IS AN OUT-PARAMETER AND THAT IS A BUG FIX, NOT A STYLE CHOICE.
// scene_from_world() ends with `scene.sdf = &world.sdf` and its own header
// states the contract: "the returned RenderScene must not outlive `world`."
// The sandbox's first built-in scene once built its WorldDesc as a function
// local and returned the scene, so `out.sdf` dangled the moment it returned:
// a latent use-after-free, reachable the day anything ray-marched it.
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
                                         const DroneSim& drone, const glm::quat& orientation, float blur) {
    scene.dynamics.clear();
    append_drone_items(binding, drone.params(), glm::vec3(0.0f), orientation, scene.dynamics);
    spade::render::RenderOptions o;
    o.overlays = false;     // no ground, no bounds worth drawing
    o.ground_grid = false;  // and no ground for the grid to sit on
    o.horizon_blend_strength = blur;
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
// in attitude through the engine's rotor model. Every decision is in
// drone_sim.hpp / drone_view.hpp and is asserted with no display; this is the
// wiring.
// ---------------------------------------------------------------------------
spade::Result<std::unique_ptr<DroneSession>> DroneSession::create(float blur) {
    std::unique_ptr<DroneSession> s(new DroneSession());
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
    s->panel_.edited = s->drone_->options();

    s->camera_.target = glm::vec3(0.0f);
    s->camera_.yaw = 0.6f;
    s->camera_.pitch = 0.35f;
    s->camera_.distance = 1.8f;
    return s;
}

void DroneSession::attach(GlTargetSink& sink) {
    sink_ = &sink;
    // Uploaded once: the drone's meshes and the palette never change, only
    // the per-frame draw items do.
    gpu_ = start_gpu(scene_);
    panel_.render_path = gpu_ ? "GPU (OpenGL)" : "CPU raster";
    sink.attach_drone(&panel_);
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

    const uint32_t fw = sink_->viewport().width;
    const uint32_t fh = sink_->viewport().height;
    if (fw == 0u || fh == 0u) {
        return {};  // minimised: the stand keeps stepping, capped by the accumulator
    }
    const spade::render::Camera rc = camera_.to_render_camera();
    const spade::render::RenderOptions options =
        drone_frame(scene_, binding_, *drone_, panel_.readouts.orientation, blur_);

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
// THE AIRFLOW CHAPTER -- the library quad, live. Every decision is in
// airflow_live.hpp and is asserted with no display; this is the wiring.
// ---------------------------------------------------------------------------
namespace {
constexpr float kAirflowTurnRadPerS = 0.15f;  // the idle camera's slow turn about the drone
}  // namespace

spade::Result<std::unique_ptr<AirflowSession>> AirflowSession::create(const std::filesystem::path& model_file) {
    std::unique_ptr<AirflowSession> s(new AirflowSession());
    spade::Result<airflow::LiveQuad> quad = airflow::live_quad(model_file);
    if (!quad) return std::unexpected(quad.error());
    s->quad_ = std::move(*quad);
    spade::Result<spade::analysis::LiveRun> run =
        spade::analysis::LiveRun::start(s->quad_.resolved, s->quad_.run_model, s->quad_.propulsion);
    if (!run) {
        return std::unexpected(spade::Error{run.error().code, "the live run would not start: " + run.error().context});
    }
    s->run_.emplace(std::move(*run));
    spade::Result<spade::render_parts::PreparedParts> prepared =
        spade::render_parts::prepare_parts(s->quad_.draw_model);
    if (!prepared) {
        return std::unexpected(
            spade::Error{prepared.error().code, "the quad's parts would not prepare: " + prepared.error().context});
    }
    s->prepared_ = std::move(*prepared);
    s->look_.body = static_cast<uint32_t>(s->base_.materials.size());
    s->base_.materials.push_back(detail::lambert(0.35f, 0.37f, 0.40f));
    s->look_.rotor = static_cast<uint32_t>(s->base_.materials.size());
    s->base_.materials.push_back(detail::lambert(0.12f, 0.12f, 0.13f));
    s->look_.porous = s->look_.body;
    s->look_.object_name = s->quad_.draw_model.name;
    const spade::analysis::Fluid& fluid = s->quad_.resolved.spec.fluid;
    s->speed_top_ = airflow::speed_top(s->quad_.run_model, fluid.density, 9.80665f);

    // flow_movie's look: a little below the drone, so the wake is in frame.
    s->camera_.target = glm::vec3(0.0f, -0.12f * airflow::kBoxM, 0.0f);
    s->camera_.yaw = 0.52f;
    s->camera_.pitch = 0.35f;
    s->camera_.distance = 0.95f * airflow::kBoxM;
    return s;
}

void AirflowSession::attach(GlTargetSink& sink) {
    sink_ = &sink;
    gpu_ = start_gpu(base_);
}

AirflowSession::~AirflowSession() {
    if (sink_ != nullptr) sink_->set_gap_line({});
}

bool AirflowSession::finished() const noexcept {
    return static_cast<float>(run_->state().t_s) >= quad_.resolved.spec.time.duration;  // in float, as the case holds it
}

spade::Result<void> AirflowSession::frame(const FrameInput& in, float dt) {
    apply_drone_orbit(camera_, in, dt);
    camera_.yaw += kAirflowTurnRadPerS * dt;

    // The run is paced to the window at airflow::kPlayback: a run frame is due
    // once that much simulated time is owed (the first frame is owed at once,
    // so the volume is published before the first draw).
    sim_owed_s_ += static_cast<double>(dt) * static_cast<double>(airflow::kPlayback);
    const double frame_s = 1.0 / static_cast<double>(spade::analysis::kFramesPerSecond);
    const auto p0 = std::chrono::steady_clock::now();
    if (!finished() && sim_owed_s_ >= frame_s) {
        sim_owed_s_ -= frame_s;
        const spade::analysis::Controls controls =
            spade::analysis::controls_at(quad_.program, static_cast<float>(run_->state().t_s));
        if (const spade::Result<spade::analysis::LiveFrame> f = run_->advance_frame(controls, volume_); !f) {
            return std::unexpected(spade::Error{f.error().code, "the live run failed: " + f.error().context});
        }
        if (std::optional<end_goal::DownwashFrame> d = airflow::downwash_frame(volume_, *run_, quad_.program)) {
            frames_.push_back(std::move(*d));
        }
    }
    const float physics_ms = ms_since(p0);

    const uint32_t fw = sink_->viewport().width;
    const uint32_t fh = sink_->viewport().height;
    if (fw == 0u || fh == 0u) return {};

    scene_ = base_;
    if (const spade::Result<void> bound = append_vehicle_parts(scene_, prepared_, quad_.draw_model, run_->body(), look_);
        !bound) {
        return std::unexpected(spade::Error{bound.error().code, "the quad's parts would not bind: " + bound.error().context});
    }
    spade::render::RenderOptions options;
    const std::vector<end_goal::RotorProbe> probes = airflow::probes_of(*run_);
    const spade::render::ViewSpec spec = airflow::airflow_view(volume_, probes, speed_top_);
    spade::render::ViewInputs inputs;
    inputs.has_volume = true;
    inputs.gl_flow_ready = gpu_ ? gpu_->flow_ready() : true;
    const spade::render::ViewBackend backend = gpu_ ? spade::render::ViewBackend::gl : spade::render::ViewBackend::cpu;
    if (const spade::render::ViewSupport s = spade::render::supports(spec, backend, inputs); !s.drawable) {
        refusal_ = s.why_not;
        legends_.clear();
    } else if (spade::Result<std::vector<spade::render::ViewLegend>> legends =
                   spade::render::apply_view(spec, &volume_, scene_, options, cache_);
               !legends) {
        refusal_ = legends.error().context;
        legends_.clear();
    } else {
        refusal_.clear();
        legends_ = std::move(*legends);
    }

    const spade::render::Camera rc = camera_.to_render_camera();
    sink_->set_gap_line(gpu_ ? render_gap_line(GpuRenderer::unhonoured(options)) : std::string{});
    const auto r0 = std::chrono::steady_clock::now();
    if (gpu_) {
        // Uploaded every frame: the flow layers and the bound parts change
        // each frame, and flow views carry no meshes of their own.
        if (const spade::Result<void> up = gpu_->upload_scene(scene_); !up) {
            return std::unexpected(spade::Error{up.error().code, "GPU upload failed: " + up.error().context});
        }
        sink_->begin_gpu_frame();
        if (const spade::Result<void> drew = gpu_->draw(scene_, rc, options, fw, fh); !drew) {
            return std::unexpected(spade::Error{drew.error().code, "GPU draw failed: " + drew.error().context});
        }
        sink_->present_overlay(ms_since(r0), physics_ms);
    } else {
        sink_->note_physics_ms(physics_ms);
        const bool ok = render_frame(scene_, rc, fw, fh, options, pixels_, *sink_);
        sink_->note_render_ms(ms_since(r0));
        if (!ok) return std::unexpected(spade::Error{spade::Code::internal, "the CPU render failed (see above)"});
    }
    return {};
}

}  // namespace spade::sandbox
