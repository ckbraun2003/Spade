// flow_movie: films the CPU airflow solver through the engine's views.
//
// Each filmed case is simulated once, from rest, with every frame's
// published volume cached to disk at a fixed SIMULATED time; then each view
// is drawn on GL at 1920x1080 at full flow quality and piped to ffmpeg,
// with captions burned in. The video's clock is simulated time only.
//
//   flow_movie --video quad|validation|smoke|cavity|taylor-green|disc|case-quad|case-smoke
//   flow_movie --video case --case FILE.case.yaml [--controls FILE]
//              [--out DIR] [--ffmpeg EXE] [--seconds S] [--stills N]
//              [--resimulate] [--keep-cache] [--drop-cache] [--simulate-only]
//              [--budget-seconds S] [--frame-budget N] [--cpu] [--view NAME]
//   flow_movie --video d1-hover|d1-still|case ... --gpu
//
// --cpu draws stills on the CPU renderer, the GL views' reference; --view
// limits stills to one view.
//
// --gpu steps a LiveRun film's fluid on the device (B7's GpuFluid, in the
// Vulkan build), with no parts in the flow and the rotors held at the
// program's one set of controls (gpu_live.hpp); its cache is <stem>-gpu, and
// it never stops at a budget.
//
// The simulation keeps the solver's state every 20 frames, so a run stopped
// by --budget-seconds or --frame-budget (exit 3), or by the box, continues
// from there when run again. A cache from another case or build is refused;
// --resimulate starts again from rest.
//
// After its render, a film's cache goes, unless --keep-cache; a LiveRun
// film's (--case, case-*) stays unless --drop-cache, since it is the only
// copy of a run that may have taken hours (D1's lost its frames to its own
// render, 2026-10-22, before they were sampled).
//
// --stills N renders N evenly spaced frames of each view as PNGs instead of
// a video, to look at a shot. Exit 0 done, 1 failed, 77 no GL or no ffmpeg.
//
// Everything under --out (Desktop/Vantage/Videos/Footage/Spade/<date> by default) is working
// output, and none of it is posted. A film is POSTED only under
// Desktop/Vantage/Videos (under the user's profile), by Videos/CATEGORIES.md (Cameron,
// 2026-10-21). An airflow film is Research: 1920x1080 at 30 fps (as
// recorded), at most 120 s and 30 MB, with a title card. A renderer
// before-and-after is a Feature. To post one, encode it through the queue
// (a short test-class job), with the card prepended:
//   python %USERPROFILE%/Desktop/Vantage/Videos/check_video.py --encode SRC
//       Videos/Research/Spade/YYYY-MM-DD_HHMM_<name>.mp4 --card CARD.txt
// The card names the title, the tree, the commit, the date (UTC), the case,
// the grid, the simulated-on and drawn-on builds, and the limits that matter
// ("actuator discs, not blades"). Then write the caption .md beside it, with
// CATEGORIES.md's fields: Subject "airflow - <case, grid, builds>", and
// Reference "none (field film)" when nothing is measured. Send the lead its
// path; the lead posts the notification.

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iterator>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "arm_pressure.hpp"
#include "box_filter.hpp"
#include "cache.hpp"
#include "cases.hpp"
#include "line_export.hpp"
#ifdef SPADE_FLOW_MOVIE_GPU
#include "gpu_live.hpp"
#endif
#include "live_case.hpp"
#include "lines.hpp"
#include "render/gltf.hpp"
#include "look.hpp"
#include "part_materials.hpp"
#include "particles.hpp"
#include "q_look.hpp"
#include "build_parts.hpp"
#include "visual_mesh.hpp"
#include "stream_plan.hpp"
#include "recorder.hpp"
#include "sgs_note.hpp"
#include "subnormal_scan.hpp"
#include "views.hpp"
#include "web_masters.hpp"

#include "physics/airflow/publish.hpp"
#include "core/fp16.hpp"
#include "render/raster_cpu.hpp"
#include "render/view_mode.hpp"
#include "render_parts/render_parts.hpp"
#include "vehicles/part_pose.hpp"

#include "analysis/live_run.hpp"
#include "analysis/validation.hpp"

namespace {

namespace fs = std::filesystem;
namespace render = spade::render;
using namespace flow_movie;

constexpr int kExitUnavailable = 77;

#ifdef SPADE_FLOW_MOVIE_GPU
using DeviceHandle = DeviceFluid;  // gpu_live.hpp's
#else
struct DeviceHandle {
    spade::analysis::LiveFluidMaker maker;  // never filled: a device film is refused without the Vulkan build
};
#endif

// --- captions ---------------------------------------------------------------------------

// A case's own caption lines: under the settings at the top, and above the
// view's name at the bottom.
struct CaptionText {
    std::vector<std::string> top;
    std::vector<std::string> bottom;
};

// Splits `text` at spaces into lines of at most `width` characters (UTF-8
// bytes, which errs short for the few non-ASCII signs).
[[nodiscard]] std::vector<std::string> wrap(const std::string& text, std::size_t width) {
    std::vector<std::string> lines;
    std::string line;
    std::size_t at = 0;
    while (at < text.size()) {
        std::size_t end = text.find(' ', at);
        if (end == std::string::npos) end = text.size();
        const std::string word = text.substr(at, end - at);
        if (!line.empty() && line.size() + 1u + word.size() > width) {
            lines.push_back(line);
            line.clear();
        }
        line += (line.empty() ? "" : " ") + word;
        at = end + 1u;
    }
    if (!line.empty()) lines.push_back(line);
    return lines;
}

// A title and wrapped lines down from the top left; wrapped lines up from the
// bottom left, the first given highest.
// 118 characters of 30 px Segoe UI fill 1840 px; a sidebar takes about half.
constexpr std::size_t kFullWidth = 118;
constexpr std::size_t kSidebarWidth = 62;

[[nodiscard]] std::vector<Caption> layout(const std::string& title, const std::vector<std::string>& top,
                                          const std::vector<std::string>& bottom,
                                          std::size_t kBodyWidth = kFullWidth) {
    std::vector<Caption> out{{0, title, true}};
    int row = 1;
    for (const std::string& t : top) {
        for (std::string& line : wrap(t, kBodyWidth)) out.push_back({row++, std::move(line), false});
    }
    std::vector<std::string> lines;
    for (const std::string& b : bottom) {
        for (std::string& line : wrap(b, kBodyWidth)) lines.push_back(std::move(line));
    }
    for (std::size_t i = 0; i < lines.size(); ++i) {
        out.push_back({-static_cast<int>(lines.size() - i), std::move(lines[i]), false});
    }
    return out;
}

// --- what a film is -----------------------------------------------------------------

// One view of a film: a clip of its own, from the same cached frames.
struct View {
    using Shot = std::function<spade::Result<void>(const spade::FlowVolume&, uint32_t frame, uint32_t frames,
                                                   render::RenderScene&, render::Camera&)>;
    using OnParts = std::function<spade::Result<std::string>(const spade::FlowVolume&, render::RenderScene&)>;
    View(std::string name_, std::string label_, Shot shot_, OnParts on_parts_ = {})
        : name(std::move(name_)), label(std::move(label_)), shot(std::move(shot_)), on_parts(std::move(on_parts_)) {}

    std::string name;   // the clip's file name
    std::string label;  // the caption naming what it shows
    Shot shot;
    // What the view draws on the film's parts, after decorate() put them in
    // the scene (C_p on the walls), returning a caption line to show under
    // the label (the legend's note, verbatim), or "". Empty: nothing.
    OnParts on_parts;
};

// A streamed run (--stream; stream_plan.hpp): what simulate_live asks of the run per frame. `wanted(k)`: write frame k at all
// (a frame no draw reads is solved and dropped). `after(k, log)`: frame k is done (written if wanted): draw it, delete what
// is spent. The solve stops after `last_frame`.
struct StreamHooks {
    std::function<bool(uint32_t)> wanted;
    std::function<spade::Result<void>(uint32_t, const FrameLog&)> after;
    uint32_t last_frame = 0;
};

// The particle look's run-wide state: the driver the stream advances (made by run_web once the still frame is known) and the clip's
// first cache frame; the view reads it when it draws.
struct ParticleHold {
    std::shared_ptr<ParticleDriver> driver;
    StreakLook look;  // --particle-look: how the streaks are drawn
    uint32_t first = 0;
    float max_age_s = 0.5f;
};

struct Film {
    std::string stem;
    AirflowCase airflow;
    uint32_t frames = 300;
    std::string time_unit;      // " s", or "" for a case in its own units
    int time_decimals = 3;
    std::function<std::vector<double>(const airflow::MacGrid&)> live;  // the case's measures per frame
    std::function<CaptionText(const FrameLog&)> captions;              // the case's lines past the standard ones
    // The case's expected top speed, in its own units: every cached frame's
    // fastest cell is held to ten times it (frame_check.hpp). Each film sets it.
    float expected_speed = 0.0f;
    std::vector<std::string> record;  // its V1 record's lines, above the view's (the validation reel)
    // Where the volume's centre shows across the frame (-1 left, +1 right). A
    // film with a record puts the picture right and its captions in a
    // sidebar on the left; 0 centres both.
    float picture_x = 0.0f;
    std::vector<View> views;
    // A film flown through LiveRun (--case): its case, each frame's
    // run_label() by the frame's step (the caption's; filled from the cache's
    // run.txt once simulated), and what every view draws over its own shot --
    // the held drone's parts.
    std::shared_ptr<const LiveCase> run;
    std::shared_ptr<std::map<uint64_t, std::string>> run_labels;
    std::function<spade::Result<void>(const FrameLog&, render::RenderScene&)> decorate;
    // The particle look (--particles): the seed pool from a frame's log (the held drone's rotors), and the run's state.
    std::function<std::vector<glm::vec3>(const FrameLog&)> particle_pool;
    std::shared_ptr<ParticleHold> particle_hold;
    // The held body's pose a view that seeds at the rotors reads (build_parts.hpp's HeldPose). Set by a film that has such a view; the draw paths record it from
    // each frame's log before decorate(), which a film may replace (note_held_pose).
    std::shared_ptr<HeldPose> held_pose;
    std::shared_ptr<const spade::analysis::LiveLines> lines;  // --lines: the rotors as actuator lines (a device film)
    bool blade_azimuth = false;  // --prop-blur mode=blades: each frame's live record ends with the blades' azimuths (LiveRun::blade_azimuth), one a rotor, in radians
    bool hover_start = false;  // --hover-start: the rotors spawn at the speed the first controls command (no spool-up), as Physics' L106 harness does
    // A library drone's parts take their roles' colours (part_materials.hpp); off for the reference quad, whose frames stay.
    bool role_materials = false;
    bool gpu = false;  // --gpu: the run's fluid on GpuFluid, no parts (simulate_gpu())
    // The website masters (web_masters.hpp). The frames are spread over this simulated window from rest, frame k
    // ending at web::frame_end_step() (0, 0: the film's own spread over t_end); a LiveRun film then steps
    // advance_steps() between them, each frame carrying its true time.
    double window_from = 0.0, window_to = 0.0;
    // Nonzero: the window is whole steps, frame k ending (k + 1) * steps_per_frame steps past the window's start
    // (web::whole_frame_end_step); window_to then only names where the last frame ends. Zero: the frames are
    // spread evenly over [window_from, window_to] (--window).
    uint32_t steps_per_frame = 0;
    // A LiveRun film's fluid on GpuFluid WITH the parts in the flow (gpu_live.hpp: open_device_fluid()).
    bool device = false;
    std::shared_ptr<StreamHooks> stream;  // --stream: set by run_web alone; null for every other film
};

// Record the held body's pose from a frame's log into the film's, when the film has a view that reads it and the log holds a body.
void note_held_pose(const Film& film, const FrameLog& log) {
    if (!film.held_pose) return;
    const HeldPose pose = held_pose_from_live(log.live);
    if (pose.known) *film.held_pose = pose;
}

[[nodiscard]] std::string fixed(double x, int decimals) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.*f", decimals, x);
    return buffer;
}

[[nodiscard]] uint64_t total_steps(const AirflowCase& c) {
    return std::max<uint64_t>(1u, static_cast<uint64_t>(std::llround(static_cast<double>(c.t_end) / c.fluid.h)));
}

// --- simulate once -------------------------------------------------------------------

// The case a cache holds: the film, its frames and steps, and its grid.
// With the build (build_identity()), what a snapshot must match to be
// continued (Physics' and the lead's rule: never into another case or commit).
[[nodiscard]] std::string window_note(const Film& film) {
    if (film.window_to <= 0.0) return "";
    return ", window " + fixed(film.window_from, 4) + ".." + fixed(film.window_to, 4) + " of simulated time";
}

[[nodiscard]] std::string film_identity(const Film& film) {
    if (film.run) {
        return film.stem + ": " + std::to_string(film.frames) +
               (film.gpu      ? " frames of GpuFluid, no parts, the rotors held; "
                : film.device && film.run->sgs_cs > 0.0f
                    ? " frames of LiveRun on GpuFluid with the Smagorinsky term, the parts in the flow, the term "
                      "checked bit-equal to the CPU for 32 steps of D1 (0.01216 s of simulated time at h 0.38 ms) "
                      "only; "
                : film.device ? " frames of LiveRun on GpuFluid, the parts in the flow, checked bit-equal to the "
                                "CPU through about 0.18 s of simulated time only; "
                              : " frames of LiveRun; ") +
               film.run->identity + window_note(film);
    }
    const AirflowCase& c = film.airflow;
    return film.stem + ": " + std::to_string(film.frames) + " frames over " + std::to_string(total_steps(c)) +
           " steps of h " + fixed(c.fluid.h, 6) + " on " + std::to_string(c.shape.nx) + "x" +
           std::to_string(c.shape.ny) + "x" + std::to_string(c.shape.nz) + " at dx " + fixed(c.shape.dx, 6) +
           ", n_v " + std::to_string(c.fluid.n_v) + ", nu " + fixed(c.fluid.nu, 8) + ", rho " + fixed(c.fluid.rho, 4) +
           window_note(film);
}

// The solver step at which frame k of a windowed film ends, at step h.
[[nodiscard]] uint64_t film_frame_end_step(const Film& film, uint32_t k, double h) {
    if (film.steps_per_frame > 0u) {
        return web::whole_frame_end_step(k, static_cast<uint64_t>(std::llround(film.window_from / h)),
                                         film.steps_per_frame);
    }
    return web::frame_end_step(k, film.frames, film.window_from, film.window_to, h);
}

// A windowed film's frames each need a step of their own: more frames than the window has steps would show one
// state twice, and a clip meant to play smoothly would stutter. (A whole-step window gives each its own.)
[[nodiscard]] spade::Result<void> check_window(const Film& film, double h) {
    if (film.window_to <= 0.0 || film.steps_per_frame > 0u) return {};
    const uint64_t steps = web::frame_end_step(0, 1, 0.0, film.window_to, h) -
                           web::frame_end_step(0, 1, 0.0, film.window_from, h);
    if (steps < film.frames) {
        return std::unexpected(spade::Error{
            spade::Code::invalid_argument,
            film.stem + ": " + std::to_string(film.frames) + " frames over a window of " + std::to_string(steps) +
                " steps of h " + fixed(h, 6) + ": a frame needs a step of its own, or the clip stutters (fewer frames, "
                "or a finer grid)"});
    }
    return {};
}

constexpr uint32_t kStateEvery = 20;  // frames between solver snapshots

// Simulates the film's frames into `cache`, continuing from its solver state
// when it has one. Stops early, with the state written, past `budget_s` of
// wall time or `frame_budget` new frames (0: no limit). True when every frame
// is in.
[[nodiscard]] spade::Result<bool> simulate(const Film& film, const fs::path& cache, const CacheMark& mark,
                                           double budget_s, uint32_t frame_budget) {
    const AirflowCase& c = film.airflow;
    auto made = airflow::make_grid(c.shape, c.bc);
    if (!made) return std::unexpected(made.error());
    airflow::MacGrid& g = *made;
    if (c.init) c.init(g);
    auto scratch = airflow::make_fluid_scratch(g, c.bc, c.fluid);
    if (!scratch) return std::unexpected(scratch.error());
    for (const airflow::ActuatorDisc& d : c.discs) {
        if (auto ok = airflow::check_disc(d, c.shape); !ok) return std::unexpected(ok.error());
    }
    std::error_code ec;
    fs::create_directories(cache, ec);
    const uint64_t total = total_steps(c);
    if (auto ok = check_window(film, c.fluid.h); !ok) return std::unexpected(ok.error());
    airflow::PublishInput in;
    in.origin = c.origin;
    in.periodic = airflow::periodic_axes(c.bc);
    spade::FlowVolume volume;
    uint64_t step = 0;
    uint64_t sgs_capped = 0;   // L52: cells held at the nu_t cap and faces wall-limited, in cell- and face-steps
    uint64_t sgs_limited = 0;
    float sgs_nu_t_max = 0.0f;
    uint32_t first = 0;
    if (fs::exists(cache / "state.bin")) {
        auto at = read_state(cache, step, g);
        if (!at) return std::unexpected(at.error());
        first = *at + 1u;
        std::printf("[ %s ] continuing after frame %u (step %llu) from %s\n", film.stem.c_str(), *at + 1u,
                    static_cast<unsigned long long>(step), (cache / "state.bin").string().c_str());
    } else {
        mark_cache(cache, mark);
    }
    const auto start = std::chrono::steady_clock::now();
    for (uint32_t k = first; k < film.frames; ++k) {
        const uint64_t target =
            film.window_to > 0.0 ? film_frame_end_step(film, k, c.fluid.h)
            : film.frames <= 1u  ? total
                                 : static_cast<uint64_t>(std::llround(static_cast<double>(k) * total / (film.frames - 1u)));
        for (; step < target; ++step) {
            for (const airflow::ActuatorDisc& d : c.discs) {
                airflow::spread_actuator(d, c.shape, c.fluid.rho, c.fluid.h, scratch->du, scratch->dv, scratch->dw);
            }
            const airflow::FluidHealth health = airflow::fluid_step(g, c.bc, c.fluid, *scratch);
            sgs_capped += health.sgs_capped_cells;
            sgs_limited += health.sgs_limited_faces;
            sgs_nu_t_max = std::max(sgs_nu_t_max, health.sgs_nu_t_max);
            if (health.fault != airflow::FluidFault::none) {
                return std::unexpected(spade::Error{spade::Code::internal, film.stem + ": the solver faulted at step " +
                                                                                std::to_string(step) + ": " +
                                                                                airflow::describe(health, g.shape)});
            }
        }
        if (auto published = airflow::publish_render_volume(g, in, volume); !published) {
            return std::unexpected(published.error());
        }
        const spade::FlowVolumeHeader& h = volume.header;
        FrameLog log{step, static_cast<double>(step) * c.fluid.h, film.live ? film.live(g) : std::vector<double>{}};
        const FrameBound bound{.expected_speed = film.expected_speed};
        if (auto wrote = write_frame(cache, k, volume, log, bound); !wrote) {
            return std::unexpected(spade::Error{wrote.error().code, film.stem + ": " + wrote.error().context});
        }
        const double elapsed_now = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const bool over_budget = (budget_s > 0.0 && elapsed_now >= budget_s) ||
                                 (frame_budget > 0u && k + 1u - first >= frame_budget);
        if ((k + 1u) % kStateEvery == 0u || (over_budget && k + 1u < film.frames)) {
            if (auto kept = write_state(cache, k, step, g); !kept) return std::unexpected(kept.error());
        }
        if (over_budget && k + 1u < film.frames) {
            std::printf("[ %s ] stopped by the budget after frame %u/%u (step %llu); run again to continue\n",
                        film.stem.c_str(), k + 1u, film.frames, static_cast<unsigned long long>(step));
            return false;
        }
        if (k % 20u == 0u || k + 1u == film.frames) {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            const double eta = k == first ? 0.0 : elapsed / (k - first) * (film.frames - 1u - k);
            std::printf("[ %s ] frame %u/%u, step %llu/%llu, t %s; |w| max %.4g, Q-hat %.4g..%.4g, lambda2-hat %.4g..%.4g; "
                        "%.0f s, about %.0f s to go\n",
                        film.stem.c_str(), k + 1u, film.frames, static_cast<unsigned long long>(step),
                        static_cast<unsigned long long>(total), fixed(log.time, film.time_decimals).c_str(),
                        h.channel_max[spade::kFlowVorticity], h.channel_min[spade::kFlowQHat],
                        h.channel_max[spade::kFlowQHat], h.channel_min[spade::kFlowLambda2Hat],
                        h.channel_max[spade::kFlowLambda2Hat], elapsed, eta);
            if (c.fluid.sgs_cs > 0.0f) {
                std::printf("[ %s ] Smagorinsky Cs %.2f: nu_t max %.3g m2/s, %llu cell-steps held at the cap, "
                            "%llu face-steps wall-limited (since this run's start)\n",
                            film.stem.c_str(), static_cast<double>(c.fluid.sgs_cs), static_cast<double>(sgs_nu_t_max),
                            static_cast<unsigned long long>(sgs_capped), static_cast<unsigned long long>(sgs_limited));
            }
            std::fflush(stdout);
        }
    }
    fs::remove(cache / "state.bin", ec);
    return true;
}

// Flies a --case film through LiveRun, one frame per advance_frame(): the
// controls at the run's own time, then the frame published. Each frame's log
// keeps the held body (pos, then orient w x y z) and the rotors' speeds, so
// a view can pose the drone's parts where the run held them; run.txt keeps
// each frame's step and run_label(). LiveRun's save (live.save) is kept every
// kStateEvery frames and when a budget stops the run, and the next run
// restores it: a restored run gives exactly the frames an unbroken one does
// (LiveRun's save/restore). A finished film drops it.
[[nodiscard]] spade::Result<bool> simulate_live(const Film& film, const fs::path& cache, const CacheMark& mark,
                                                double budget_s, uint32_t frame_budget) {
    namespace an = spade::analysis;
    const LiveCase& lc = *film.run;
    auto resolved = resolve_live(lc);
    if (!resolved) return std::unexpected(spade::Error{resolved.error().code, film.stem + ": " + resolved.error().context});
    std::error_code ec;
    fs::create_directories(cache, ec);
    const fs::path save = cache / "live.save";
    // A device film keeps no save (it runs in minutes and a cache left unfinished starts again), so no budget.
    if (film.device && (budget_s > 0.0 || frame_budget > 0u)) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument,
                                            film.stem + ": a film on the device keeps no save, so it takes no budget"});
    }
    const bool resuming = !film.device && fs::exists(save, ec);
    DeviceHandle device;  // the run's context, which must outlive it
    if (film.device) {
#ifdef SPADE_FLOW_MOVIE_GPU
        auto opened = open_device_fluid();
        if (!opened) return std::unexpected(spade::Error{opened.error().code, film.stem + ": " + opened.error().context});
        device = std::move(*opened);
#else
        return std::unexpected(spade::Error{spade::Code::unavailable, film.stem + ": a device film needs the Vulkan build"});
#endif
    }
    // L52: LiveRun's config has no subgrid viscosity, so a device film's maker is handed the case's Cs on a copy of
    // the config (GpuFluid takes it since L54: its kernel is bit-equal to sgs.cpp).
    an::LiveFluidMaker maker = device.maker;
    if (film.device && lc.sgs_cs > 0.0f) {
        maker = [inner = device.maker, cs = lc.sgs_cs](const an::LiveFluidStart& s) {
            spade::physics::airflow::FluidConfig config = s.config;
            config.sgs_cs = cs;
            return inner(an::LiveFluidStart{s.grid, s.domain, config, s.solids, s.lines, s.line_elements, s.line_rotors});
        };
    }
    // --hover-start: the rotors spawn at the speed the film's first controls command, the same call the L106 harness makes (Physics' hover-start).
    const an::Controls first_controls = an::controls_at(lc.program, 0.0f);
    auto run = film.device ? an::LiveRun::start(*resolved, lc.run_model, maker, film.lines.get(), film.hover_start ? &first_controls : nullptr)
               : resuming  ? an::LiveRun::restore(save, *resolved, lc.run_model)
                           : an::LiveRun::start(*resolved, lc.run_model);
    if (!run) return std::unexpected(spade::Error{run.error().code, film.stem + ": " + run.error().context});
    if (auto ok = check_window(film, run->step_s()); !ok) return std::unexpected(ok.error());
    const auto first = static_cast<uint32_t>(run->state().frames);
    // run.txt: the saved frames' lines as they were (a kill after the save
    // may have left more), then this run's.
    std::vector<std::string> kept;
    if (resuming) {
        std::ifstream old(cache / "run.txt", std::ios::binary);
        for (std::string line; kept.size() < first && std::getline(old, line);) kept.push_back(line);
        if (kept.size() < first) {
            return std::unexpected(spade::Error{spade::Code::io_error, film.stem + ": " + (cache / "run.txt").string() +
                                                                           " holds fewer frames than its save"});
        }
        std::printf("[ %s ] continuing after frame %u (step %llu) from %s\n", film.stem.c_str(), first,
                    static_cast<unsigned long long>(run->state().steps), save.string().c_str());
    } else {
        mark_cache(cache, mark);
    }
    if (!lc.domain.empty()) {  // L86c: the resolved domain, in the log and beside the frames (a resume repeats the same line)
        std::printf("[ %s ] %s\n", film.stem.c_str(), lc.domain.c_str());
        std::ofstream named(cache / "domain.txt", std::ios::binary | std::ios::trunc);
        named << lc.domain << '\n';
    }
    std::ofstream lines(cache / "run.txt", std::ios::binary | std::ios::trunc);
    for (const std::string& line : kept) lines << line << '\n';
    // convergence.csv: after every frame, LiveRun's pressure convergence (D1's
    // record, 2026-10-22). LiveRun counts unconverged steps since its start or
    // restore (its save does not keep the count), so each line names the frame
    // its count runs from; a frame's own unconverged steps are the difference
    // from the line before within one count. Appended: a run killed after its
    // save repeats the frames after it, and the last line for a frame is the
    // one its cached frame came from.
    const bool new_log = !fs::exists(cache / "convergence.csv", ec);
    std::ofstream convergence(cache / "convergence.csv", std::ios::binary | std::ios::app);
    if (new_log) convergence << "frame,t_s,steps,counted_from_frame,unconverged_steps,last_residual\n";
    spade::FlowVolume volume;
    const auto start = std::chrono::steady_clock::now();
    for (uint32_t k = first; k < film.frames; ++k) {
        const an::Controls controls = an::controls_at(lc.program, static_cast<float>(run->state().t_s));
        // A window spreads the frames over its simulated time, a frame every few steps; else LiveRun's own 30 a second.
        const uint64_t target =
            film.window_to > 0.0 ? film_frame_end_step(film, k, run->step_s()) : 0u;
        if (film.window_to > 0.0 && target <= run->state().steps) {
            return std::unexpected(spade::Error{spade::Code::internal, film.stem + ": frame " + std::to_string(k) +
                                                                           " has no step of its own in its window"});
        }
        auto frame = film.window_to > 0.0 ? run->advance_steps(controls, target - run->state().steps, volume)
                                          : run->advance_frame(controls, volume);
        if (!frame) return std::unexpected(spade::Error{frame.error().code, film.stem + ": " + frame.error().context});
        const spade::BodyState& b = run->body();
        std::vector<double> live{b.pos.x, b.pos.y, b.pos.z, b.orient.w, b.orient.x, b.orient.y, b.orient.z};
        for (const float w : run->state().rotor_omega) live.push_back(w);
        // The lines' thrust in all, after this step, last: the 12th value of a 4-rotor record, as Physics' L106 harness writes it. It is not a speed
        // (rotor_speeds_of_live reads the speeds by the table's rotor count), so a thrustless record draws the same frame.
        if (film.lines) live.push_back(run->line_thrust());
        if (film.blade_azimuth) {  // the blades' azimuths, last: the simulation's psi at this frame's step (what the drawn blade is turned by)
            for (const float psi : run->blade_azimuth()) live.push_back(static_cast<double>(psi));
        }
        const FrameLog log{frame->steps, frame->t_s, std::move(live)};
        const FrameBound bound{.expected_speed = film.expected_speed, .solid_cells = run->solids().solid_cells};
        // A streamed run writes only the frames its plan needs: one no draw reads is solved and dropped.
        if (!film.stream || film.stream->wanted(k)) {
            if (auto wrote = write_frame(cache, k, volume, log, bound); !wrote) {
                return std::unexpected(spade::Error{wrote.error().code, film.stem + ": " + wrote.error().context});
            }
        }
        lines << frame->steps << '\t' << run->run_label() << '\n';
        char residual[32];
        std::snprintf(residual, sizeof residual, "%.3e", static_cast<double>(run->solids().pressure_residual));
        convergence << frame->frame << ',' << fixed(frame->t_s, 6) << ',' << frame->steps << ',' << first << ','
                    << run->solids().unconverged_steps << ',' << residual << '\n';
        convergence.flush();
        if (film.stream) {
            if (auto done = film.stream->after(k, log); !done) {
                return std::unexpected(spade::Error{done.error().code, film.stem + ": " + done.error().context});
            }
            if (k >= film.stream->last_frame) break;  // the plan's last frame: nothing after it is needed
        }
        const double elapsed_now = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const bool over_budget = (budget_s > 0.0 && elapsed_now >= budget_s) ||
                                 (frame_budget > 0u && k + 1u - first >= frame_budget);
        if (!film.device && ((k + 1u) % kStateEvery == 0u || (over_budget && k + 1u < film.frames))) {
            lines.flush();  // the save's frames are all in run.txt first
            if (auto kept_run = run->save(save); !kept_run) {
                return std::unexpected(spade::Error{kept_run.error().code, film.stem + ": " + kept_run.error().context});
            }
        }
        if (over_budget && k + 1u < film.frames) {
            std::printf("[ %s ] stopped by the budget after frame %u/%u (step %llu); run again to continue\n",
                        film.stem.c_str(), k + 1u, film.frames, static_cast<unsigned long long>(frame->steps));
            return false;
        }
        if (k % 10u == 0u || k + 1u == film.frames) {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            const std::string sgs_text = sgs_note(run->sgs());  // L59: the peak nu_t even when nothing bound
            std::printf("[ %s ] frame %u/%u, t %s s; held at (%.3f, %.3f, %.3f) in a box from (%.3f, %.3f, %.3f); "
                        "%.0f s%s%s\n",
                        film.stem.c_str(), k + 1u, film.frames, fixed(frame->t_s, 3).c_str(), b.pos.x, b.pos.y,
                        b.pos.z, volume.header.origin.x, volume.header.origin.y, volume.header.origin.z, elapsed,
                        sgs_text.empty() ? "" : "; ", sgs_text.c_str());
            std::fflush(stdout);
        }
    }
    if (!lines) return std::unexpected(spade::Error{spade::Code::io_error, "writing " + (cache / "run.txt").string()});
    // The run's health beside run.txt (whose lines must stay "step<TAB>label"): health.txt, the subgrid account and the
    // pressure solve's, each one line, counted since this run's start.
    {
        const std::string sgs_text = sgs_note(run->sgs());
        const std::string pressure_text = run->pressure_line();
        const std::string sgs_said = sgs_text.empty() ? "no subgrid viscosity (Cs 0)" : sgs_text;
        const std::string pressure_said = pressure_text.empty() ? "every step's pressure converged" : pressure_text;
        std::ofstream health_file(cache / "health.txt", std::ios::binary | std::ios::trunc);
        health_file << "subgrid: " << sgs_said << '\n' << "pressure: " << pressure_said << '\n';
        if (!health_file) {
            return std::unexpected(spade::Error{spade::Code::io_error, "writing " + (cache / "health.txt").string()});
        }
        std::printf("[ %s ] subgrid: %s\n[ %s ] pressure: %s (since this run's start; in health.txt)\n",
                    film.stem.c_str(), sgs_said.c_str(), film.stem.c_str(), pressure_said.c_str());
    }
    fs::remove(save, ec);
    return true;
}

// --step-time: starts the run (or fails naming why), prints its grid, its
// solids' line and run label, then times `steps` single steps, each with its
// pressure residual. The D1 run's go or no-go before its long run (the lead:
// over 30 s a step is reported first); nothing is cached.
[[nodiscard]] int step_time_report(const LiveCase& lc, uint32_t steps) {
    namespace an = spade::analysis;
    using Clock = std::chrono::steady_clock;
    auto resolved = resolve_live(lc);
    if (!resolved) {
        std::fprintf(stderr, "flow_movie: %s\n", resolved.error().context.c_str());
        return 1;
    }
    const spade::physics::airflow::GridShape& g = resolved->base;
    std::printf("[ step time ] %s: %u x %u x %u cells at %.4f m (%.2f M cells, %.0f MiB), h %.3f ms\n",
                lc.name.c_str(), g.nx, g.ny, g.nz, static_cast<double>(g.dx),
                static_cast<double>(g.nx) * g.ny * g.nz / 1e6, static_cast<double>(resolved->base_bytes) / 1048576.0,
                static_cast<double>(*lc.c.time.step) * 1e3);
    std::fflush(stdout);
    const Clock::time_point t0 = Clock::now();
    auto run = an::LiveRun::start(*resolved, lc.run_model);
    if (!run) {
        std::fprintf(stderr, "flow_movie: start: %s\n", run.error().context.c_str());
        return 1;
    }
    const double start_s = std::chrono::duration<double>(Clock::now() - t0).count();
    const an::LiveSolids& solids = run->solids();
    std::printf("[ step time ] start %.1f s; solids in the flow: %s, %u solid cells\n", start_s,
                solids.any ? "yes" : "no", solids.solid_cells);
    std::printf("[ step time ] run line: %s\n", solids.run_line.c_str());
    std::printf("[ step time ] run label: %s\n", run->run_label().c_str());
    std::fflush(stdout);
    double total = 0.0;
    for (uint32_t i = 0; i < steps; ++i) {
        const an::Controls controls = an::controls_at(lc.program, static_cast<float>(run->state().t_s));
        const Clock::time_point s0 = Clock::now();
        if (auto stepped = run->step(controls); !stepped) {
            std::fprintf(stderr, "flow_movie: step %u: %s\n", i + 1u, stepped.error().context.c_str());
            return 1;
        }
        const double dt = std::chrono::duration<double>(Clock::now() - s0).count();
        total += dt;
        std::printf("[ step time ] step %u: %.2f s; residual %.2e, unconverged so far %llu\n", i + 1u, dt,
                    static_cast<double>(run->solids().pressure_residual),
                    static_cast<unsigned long long>(run->solids().unconverged_steps));
        std::fflush(stdout);
    }
    if (steps > 0u) {
        std::printf("[ step time ] mean %.2f s a step over %u\n", total / steps, steps);
    }
    return 0;
}

// --- render many times -----------------------------------------------------------------

[[nodiscard]] std::vector<Caption> standard_captions(const Film& film, const View& view, const FrameLog& log,
                                                     const std::string& note = {}) {
    const AirflowCase& c = film.airflow;
    const CaptionText extra = film.captions ? film.captions(log) : CaptionText{};
    std::vector<std::string> top{c.settings};
    top.insert(top.end(), extra.top.begin(), extra.top.end());
    std::vector<std::string> bottom;
    if (film.picture_x != 0.0f) {
        // The sidebar: the record under the settings, its references left to
        // the card, which shows the record whole.
        for (const std::string& line : film.record) {
            if (line.rfind("reference: ", 0) != 0) top.push_back(line);
        }
    } else {
        bottom = film.record;
    }
    bottom.insert(bottom.end(), extra.bottom.begin(), extra.bottom.end());
    bottom.push_back(view.label);
    if (!note.empty()) bottom.push_back(note);
    bottom.push_back("t = " + fixed(log.time, film.time_decimals) + film.time_unit + "   (step " +
                     std::to_string(log.step) + " of " + std::to_string(total_steps(c)) + ")");
    return layout(c.name, top, bottom, film.picture_x != 0.0f ? kSidebarWidth : kFullWidth);
}

// Moves the camera sideways, keeping its aim, so the volume's centre shows
// at `x` of the frame's half-width from the middle (+ right).
void offset_picture(render::Camera& camera, const spade::FlowVolume& volume, float x, const Encoder& encoder) {
    if (x == 0.0f) return;
    const glm::vec3 right = camera.orientation * glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 forward = camera.orientation * glm::vec3(0.0f, 0.0f, -1.0f);
    const float depth = glm::dot(volume_centre(volume) - camera.position, forward);
    const float aspect = static_cast<float>(encoder.width) / static_cast<float>(encoder.height);
    camera.position -= right * (x * depth * std::tan(0.5f * camera.fov_y_radians) * aspect);
}

// One clip per view, from the cached frames.
[[nodiscard]] spade::Result<std::vector<fs::path>> render_film(const Film& film, const fs::path& cache, GlStage& stage,
                                                               const Encoder& encoder, const fs::path& dir) {
    std::vector<std::unique_ptr<Clip>> clips;
    for (const View& view : film.views) {
        auto clip = Clip::open(encoder, dir, film.stem + "-" + view.name);
        if (!clip) return std::unexpected(clip.error());
        clips.push_back(std::move(*clip));
    }
    spade::FlowVolume volume;
    FrameLog log;
    std::vector<uint8_t> rgba;
    const auto start = std::chrono::steady_clock::now();
    for (uint32_t k = 0; k < film.frames; ++k) {
        if (auto read = read_frame(cache, k, volume, log); !read) return std::unexpected(read.error());
        for (std::size_t v = 0; v < film.views.size(); ++v) {
            render::RenderScene scene = dark_scene(volume);
            render::Camera camera;
            if (auto shot = film.views[v].shot(volume, k, film.frames, scene, camera); !shot) {
                return std::unexpected(shot.error());
            }
            note_held_pose(film, log);
            if (film.decorate) {
                if (auto drawn = film.decorate(log, scene); !drawn) return std::unexpected(drawn.error());
            }
            std::string note;
            if (film.views[v].on_parts) {
                auto on = film.views[v].on_parts(volume, scene);
                if (!on) return std::unexpected(on.error());
                note = std::move(*on);
            }
            offset_picture(camera, volume, film.picture_x, encoder);
            if (auto drew = stage.draw(scene, camera, rgba); !drew) return std::unexpected(drew.error());
            if (auto added = clips[v]->add_frame(rgba, standard_captions(film, film.views[v], log, note)); !added) {
                return std::unexpected(added.error());
            }
        }
        if (k % 30u == 0u || k + 1u == film.frames) {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            std::printf("[ %s ] drew frame %u/%u of %zu views, %.0f s\n", film.stem.c_str(), k + 1u, film.frames,
                        film.views.size(), elapsed);
            std::fflush(stdout);
        }
    }
    std::vector<fs::path> out;
    for (auto& clip : clips) {
        auto done = clip->finish();
        if (!done) return std::unexpected(done.error());
        out.push_back(*done);
    }
    return out;
}

// N evenly spaced frames of each view, as PNGs, to look at the shots.
// One frame on the CPU renderer (raster_cpu), the GL views' reference, as
// RGBA8 with the top row first.
[[nodiscard]] spade::Result<void> draw_cpu(const render::RenderScene& scene, const render::Camera& camera,
                                           const Encoder& encoder, std::vector<uint8_t>& rgba) {
    std::vector<uint8_t> bgrx(static_cast<std::size_t>(encoder.width) * encoder.height * 4u);
    render::RenderTarget target{.pixels = std::span<uint8_t>(bgrx), .width = encoder.width,
                                .height = encoder.height, .stride = encoder.width * 4u};
    render::RenderOptions options;
    options.shadows = false;
    options.overlays = false;
    if (auto drew = render::render(scene, camera, options, target); !drew) return drew;
    rgba.resize(bgrx.size());
    for (std::size_t p = 0; p < bgrx.size(); p += 4u) {
        rgba[p] = bgrx[p + 2u];
        rgba[p + 1u] = bgrx[p + 1u];
        rgba[p + 2u] = bgrx[p];
        rgba[p + 3u] = 255u;
    }
    return {};
}

[[nodiscard]] spade::Result<void> render_stills(const Film& film, const fs::path& cache, GlStage& stage,
                                                const Encoder& encoder, const fs::path& dir, uint32_t stills,
                                                bool cpu, const std::string& only_view, uint32_t fixed_frame = ~0u) {
    spade::FlowVolume volume;
    FrameLog log;
    std::vector<uint8_t> rgba;
    std::error_code ec;
    fs::create_directories(dir, ec);
    for (uint32_t s = 0; s < stills; ++s) {
        const uint32_t k = fixed_frame != ~0u ? fixed_frame
                           : stills <= 1u    ? film.frames - 1u
                                             : s * (film.frames - 1u) / (stills - 1u);
        if (auto read = read_frame(cache, k, volume, log); !read) return read;
        for (const View& view : film.views) {
            if (!only_view.empty() && view.name != only_view) continue;
            render::RenderScene scene = dark_scene(volume);
            render::Camera camera;
            if (auto shot = view.shot(volume, k, film.frames, scene, camera); !shot) return shot;
            note_held_pose(film, log);
            if (film.decorate) {
                if (auto drawn = film.decorate(log, scene); !drawn) return drawn;
            }
            std::string note;
            if (view.on_parts) {
                auto on = view.on_parts(volume, scene);
                if (!on) return std::unexpected(on.error());
                note = std::move(*on);
            }
            offset_picture(camera, volume, film.picture_x, encoder);
            if (cpu) {
                if (auto drew = draw_cpu(scene, camera, encoder, rgba); !drew) return drew;
            } else if (auto drew = stage.draw(scene, camera, rgba); !drew) {
                return drew;
            }
            const fs::path png =
                dir / (film.stem + "-" + view.name + "-" + std::to_string(k) + (cpu ? "-cpu" : "") + ".png");
            if (auto saved = save_png(encoder, rgba, png); !saved) return saved;
            std::printf("still: %s (t %s)%s%s\n", png.string().c_str(), fixed(log.time, film.time_decimals).c_str(),
                        note.empty() ? "" : "; ", note.c_str());
        }
    }
    return {};
}

// The inflow's pressure: the mean gauge pressure over the inlet face's cell
// layer (x = 0, the wind's upstream face), no-data cells left out. What D1's
// C_p is taken against (Physics: "against the inflow's pressure").
[[nodiscard]] double inflow_pressure(const spade::FlowVolume& v) {
    const spade::FlowVolumeHeader& h = v.header;
    double sum = 0.0;
    uint64_t n = 0;
    for (uint32_t k = 0; k < h.nz; ++k) {
        for (uint32_t j = 0; j < h.ny; ++j) {
            const float p = spade::fp16::to_f32(v.s[spade::flow_cell_index(h, 0u, j, k) * spade::kFlowHalvesPerCell]);
            if (std::isfinite(p)) {
                sum += p;
                ++n;
            }
        }
    }
    return n > 0u ? sum / static_cast<double>(n) : 0.0;
}

// --arm-pressure: D1's arm-top samples (arm_pressure.hpp) over every cached
// frame, for Physics' record: a row per frame and rotor, and every sample, as
// CSV beside the cache (arm-pressure.csv, arm-samples.csv), the rows printed
// too. The inflow's pressure is inflow_pressure()'s, and C_p is over the C_p
// view's q, the discs' loading T/A.
[[nodiscard]] spade::Result<void> arm_pressure_report(const Film& film, const fs::path& cache) {
    if (!film.run) {
        return std::unexpected(
            spade::Error{spade::Code::invalid_argument, "--arm-pressure reads a LiveRun film's frames"});
    }
    const LiveCase& lc = *film.run;
    const float two_v_h = std::max(1.0f, lc.speed_max - lc.c.flight.speed);
    const double q = 0.5 * static_cast<double>(lc.c.fluid.density) * two_v_h * two_v_h;
    const fs::path rows_path = cache / "arm-pressure.csv", samples_path = cache / "arm-samples.csv";
    std::ofstream rows(rows_path, std::ios::binary | std::ios::trunc);
    std::ofstream each(samples_path, std::ios::binary | std::ios::trunc);
    rows << "frame,t_s,step,rotor,span_from_m,span_to_m,samples,mean_p_pa,p_inflow_pa,mean_dp_pa,cp\n";
    each << "frame,t_s,rotor,from_centre_m,x_m,y_m,z_m,p_pa\n";
    spade::FlowVolume volume;
    FrameLog log;
    const uint32_t frames = cached_frames(cache);
    for (uint32_t k = 0; k < frames; ++k) {
        if (auto read = read_frame(cache, k, volume, log); !read) return read;
        if (log.live.size() < 7u) {
            return std::unexpected(spade::Error{spade::Code::internal, "a LiveRun frame's log holds no held body"});
        }
        spade::BodyState b{};
        b.pos = glm::vec3(static_cast<float>(log.live[0]), static_cast<float>(log.live[1]),
                          static_cast<float>(log.live[2]));
        b.orient = glm::quat(static_cast<float>(log.live[3]), static_cast<float>(log.live[4]),
                             static_cast<float>(log.live[5]), static_cast<float>(log.live[6]));
        std::vector<ArmSample> samples;
        auto arms = arm_pressures(lc.run_model, b, volume, samples);
        if (!arms) return std::unexpected(arms.error());
        const double p_in = inflow_pressure(volume);
        char line[320];
        for (const ArmPressure& a : *arms) {
            const double dp = a.mean - p_in;
            std::snprintf(line, sizeof line, "%u,%.6f,%llu,%u,%.4f,%.4f,%u,%.4f,%.4f,%.4f,%.5f\n", k, log.time,
                          static_cast<unsigned long long>(log.step), a.rotor, static_cast<double>(a.span_from),
                          static_cast<double>(a.span_to), a.samples, a.mean, p_in, dp, dp / q);
            rows << line;
            std::printf("[ arm pressure ] %s", line);
        }
        for (const ArmSample& s : samples) {
            std::snprintf(line, sizeof line, "%u,%.6f,%u,%.4f,%.5f,%.5f,%.5f,%.4f\n", k, log.time, s.rotor,
                          static_cast<double>(s.from_centre), static_cast<double>(s.at.x),
                          static_cast<double>(s.at.y), static_cast<double>(s.at.z), s.pressure);
            each << line;
        }
    }
    if (!rows || !each) {
        return std::unexpected(spade::Error{spade::Code::io_error, "writing " + rows_path.string() + " or " +
                                                                       samples_path.string()});
    }
    std::printf("[ arm pressure ] %u frames, q %.2f Pa: %s, %s\n", frames, q, rows_path.string().c_str(),
                samples_path.string().c_str());
    return {};
}

// Physics' swirl check: the vertical vorticity w_y = du/dz - dw/dx, central
// differences on the published cell-centred velocity, averaged over the 3x3
// cells round each disc's axis, 0.5 R and 2 R below its centre.
[[nodiscard]] spade::Result<void> probe_swirl(const Film& film, const fs::path& cache, uint32_t frame) {
    spade::FlowVolume volume;
    FrameLog log;
    if (auto read = read_frame(cache, frame, volume, log); !read) return read;
    const spade::FlowVolumeHeader& h = volume.header;
    const auto vel = [&](int i, int j, int k, uint32_t c) {
        return spade::fp16::to_f32(volume.v[spade::flow_cell_index(h, static_cast<uint32_t>(i), static_cast<uint32_t>(j),
                                                                    static_cast<uint32_t>(k)) *
                                                spade::kFlowHalvesPerCell + c]);
    };
    std::printf("frame %u, t %s s: w_y (1/s), the mean over 3x3 cells on each disc's axis\n", frame,
                fixed(log.time, 3).c_str());
    for (std::size_t d = 0; d < film.airflow.discs.size(); ++d) {
        const airflow::ActuatorDisc& disc = film.airflow.discs[d];
        const glm::vec3 hub = disc.centre + film.airflow.origin;  // world
        for (const float below : {0.5f, 2.0f}) {
            const glm::vec3 at = hub - glm::vec3(0.0f, below * disc.radius, 0.0f);
            const glm::vec3 g = (at - h.origin) / h.cell;
            const int ci = static_cast<int>(std::floor(g.x)), cj = static_cast<int>(std::floor(g.y)),
                      ck = static_cast<int>(std::floor(g.z));
            double sum = 0.0;
            int n = 0;
            for (int dk = -1; dk <= 1; ++dk) {
                for (int di = -1; di <= 1; ++di) {
                    const int i = ci + di, k = ck + dk;
                    if (i < 1 || k < 1 || i + 1 >= static_cast<int>(h.nx) || k + 1 >= static_cast<int>(h.nz)) continue;
                    const double du_dz = (vel(i, cj, k + 1, 0) - vel(i, cj, k - 1, 0)) / (2.0 * h.cell);
                    const double dw_dx = (vel(i + 1, cj, k, 2) - vel(i - 1, cj, k, 2)) / (2.0 * h.cell);
                    sum += du_dz - dw_dx;
                    ++n;
                }
            }
            const int spin = d < film.airflow.spins.size() ? film.airflow.spins[d] : 0;
            std::printf("  disc %zu (spin %+d) at (%+.3f, %.3f, %+.3f), %.1f R below: w_y %+.2f\n", d, spin, hub.x,
                        hub.y, hub.z, static_cast<double>(below), n > 0 ? sum / n : 0.0);
        }
    }
    return {};
}

// --- the films ------------------------------------------------------------------------

[[nodiscard]] float induced_speed(const airflow::ActuatorDisc& d, float rho) {
    return std::sqrt(d.thrust / (2.0f * rho * std::numbers::pi_v<float> * d.radius * d.radius));
}

// The rims of a case's discs, orange for spin +1 and cyan for -1, white for a
// disc without swirl.
void add_rims(render::RenderScene& scene, const AirflowCase& c) {
    for (std::size_t i = 0; i < c.discs.size(); ++i) {
        const int spin = i < c.spins.size() ? c.spins[i] : 0;
        const glm::vec3 colour = spin > 0 ? glm::vec3(1.0f, 0.55f, 0.15f)
                                 : spin < 0 ? glm::vec3(0.2f, 0.85f, 1.0f)
                                            : glm::vec3(0.95f);
        scene.flow_polylines.push_back(circle(c.discs[i].centre + c.origin, c.discs[i].axis, c.discs[i].radius, colour));
    }
}

// Azimuth across a clip: a slow orbit of `sweep` degrees.
[[nodiscard]] float sweep_azimuth(float start, float sweep, uint32_t frame, uint32_t frames) {
    return start + sweep * (frames <= 1u ? 0.0f : static_cast<float>(frame) / static_cast<float>(frames - 1u));
}

[[nodiscard]] spade::Result<Film> quad_film(uint32_t frames) {
    auto c = quad_hover_case();
    if (!c) return std::unexpected(c.error());
    Film film;
    film.stem = "quad-wake";
    film.airflow = std::move(*c);
    film.frames = frames;
    film.time_unit = " s";
    film.captions = [](const FrameLog&) {
        return CaptionText{{"actuator discs: uniform thrust, constant-circulation swirl; no drone body yet "
                            "(D1 adds the solid drone)",
                            "inviscid: the shear layers break down to the grid's 1 cm scale"},
                           {}};
    };
    const AirflowCase& q = film.airflow;
    film.expected_speed = 2.0f * induced_speed(q.discs[0], q.fluid.rho);
    const float speed_max = 12.0f;  // the far wake: about 2 v_h = 10.7 m/s
    const glm::vec3 target(0.0f, -0.3f, 0.0f);
    const auto speed_colours = colours_over(render::FlowQuantity::speed, render::ColourMapId::viridis, 0.0f, speed_max);
    const auto orbit_at = [target](uint32_t k, uint32_t n) {
        return orbit(target, 1.4f, sweep_azimuth(25.0f, 90.0f, k, n), 18.0f);
    };
    film.views.push_back(View{
        "lambda2", "lambda2 vortex surfaces, coloured by speed |u|, 0 to 12 m/s",
        [q, speed_colours, orbit_at](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                                     render::Camera& camera) -> spade::Result<void> {
            render::IsoSurface iso;
            iso.quantity = render::FlowQuantity::lambda2_hat;
            iso.value = -120.0f;  // lambda2-hat in 1/s; the wake's cores reach about -600
            iso.colour_by = speed_colours;
            scene.flow_layers.push_back(render::FlowLayer{&v, {}, {iso}});
            add_rims(scene, q);
            camera = orbit_at(k, n);
            return {};
        }});
    film.views.push_back(View{
        "vorticity", "vorticity magnitude |w| as a volume, 0 to 400 1/s",
        [q, orbit_at](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                      render::Camera& camera) -> spade::Result<void> {
            scene.flow_layers.push_back(render::FlowLayer{
                &v, {volume_transfer(render::FlowQuantity::vorticity, render::ColourMapId::inferno, 400.0f, 0.15f, 30.0f)},
                {}});
            add_rims(scene, q);
            camera = orbit_at(k, n);
            return {};
        }});
    // The web master's volume (the lead, 2026-11-10): speed, not |w|. |w| is a derivative and non-negative, so grid-scale
    // ripple rectifies into a fog; the wake is a jet of fast air, which speed shows. Viridis, from the ramp's 15%.
    film.views.push_back(View{
        "speed-volume", "speed |u| as a volume, 0 to 12 m/s",
        [q, orbit_at](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                      render::Camera& camera) -> spade::Result<void> {
            scene.flow_layers.push_back(render::FlowLayer{
                &v, {volume_transfer(render::FlowQuantity::speed, render::ColourMapId::viridis, 12.0f, 0.15f, 30.0f)}, {}});
            add_rims(scene, q);
            camera = orbit_at(k, n);
            return {};
        }});
    film.views.push_back(View{
        "streamlines", "streamlines seeded above each rotor, coloured by speed |u|, 0 to 12 m/s",
        [q, speed_colours, orbit_at](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                                     render::Camera& camera) -> spade::Result<void> {
            std::vector<glm::vec3> seeds;
            for (const airflow::ActuatorDisc& d : q.discs) {
                const glm::vec3 hub = d.centre + q.origin;
                for (const float r : {0.35f, 0.75f}) {
                    for (const glm::vec3& s : render::seed_ring(hub - d.axis * 0.06f, d.axis, r * d.radius, 8)) seeds.push_back(s);
                }
            }
            render::StreamlineOptions options;
            options.max_steps = 1200;
            if (auto ok = add_streamlines(scene, v, seeds, options, speed_colours); !ok) return ok;
            add_rims(scene, q);
            camera = orbit_at(k, n);
            return {};
        }});
    film.views.push_back(View{
        "above", "the swirl from above: streamlines through each disc, coloured by speed |u|, 0 to 12 m/s; "
                 "rims orange for spin +1, cyan for spin -1",
        [q, speed_colours](const spade::FlowVolume& v, uint32_t, uint32_t, render::RenderScene& scene,
                           render::Camera& camera) -> spade::Result<void> {
            std::vector<glm::vec3> seeds;
            for (const airflow::ActuatorDisc& d : q.discs) {
                const glm::vec3 hub = d.centre + q.origin;
                for (const glm::vec3& s : render::seed_ring(hub - d.axis * 0.02f, d.axis, 0.7f * d.radius, 10)) {
                    seeds.push_back(s);
                }
            }
            render::StreamlineOptions options;
            options.backward = false;
            options.max_steps = 600;
            if (auto ok = add_streamlines(scene, v, seeds, options, speed_colours); !ok) return ok;
            add_rims(scene, q);
            // Straight down the axis but for 10 degrees: +x right, -z up.
            camera = orbit(glm::vec3(0.0f, 0.0f, 0.0f), 0.6f, 0.0f, 80.0f);
            return {};
        }});
    film.views.push_back(View{
        "slice", "speed |u| on the plane through rotors +x and -x, 0 to 12 m/s",
        [q, speed_max](const spade::FlowVolume& v, uint32_t, uint32_t, render::RenderScene& scene,
                       render::Camera& camera) -> spade::Result<void> {
            // From the box's floor to 13 cm above the hubs: the still air above
            // says little, and the top of the frame holds the captions.
            const glm::vec3 lo = v.header.origin, size = volume_size(v);
            const float top = q.discs[0].centre.y + q.origin.y + 0.13f;
            render::FieldLayer plane;
            plane.center = glm::vec3(lo.x + 0.5f * size.x, 0.5f * (lo.y + top), 0.0f);
            plane.width = size.x;
            plane.height = top - lo.y;
            plane.cells_u = 2u * v.header.nx;
            plane.cells_v = static_cast<uint32_t>(std::lround(plane.height / v.header.cell)) * 2u;
            if (auto ok = add_slice(scene, v, render::FlowQuantity::speed, plane, render::ColourMapId::viridis, 0.0f,
                                    speed_max);
                !ok) {
                return ok;
            }
            add_rims(scene, q);
            // Aimed 11 cm above the plane's centre, so the plane sits between
            // the caption bands.
            camera = orbit(plane.center + glm::vec3(0.0f, 0.11f, 0.0f), 1.55f, 0.0f, 0.0f);
            return {};
        }});
    return film;
}

[[nodiscard]] Film disc_film(uint32_t frames, AirflowCase c, std::string stem) {
    Film film;
    film.stem = std::move(stem);
    film.airflow = std::move(c);
    film.frames = frames;
    film.time_unit = " s";
    const AirflowCase& d = film.airflow;
    const float vh = induced_speed(d.discs[0], d.fluid.rho);
    const glm::vec3 hub = d.discs[0].centre + d.origin;
    const float r = d.discs[0].radius;
    // The record's measure: the axial speed on the axis 2R below the disc,
    // from the four v faces round it (C8's j = 76), shown live.
    const glm::vec3 hub_grid = d.discs[0].centre;
    film.live = [hub_grid, r](const airflow::MacGrid& g) {
        const airflow::GridShape& s = g.shape;
        const auto j = static_cast<uint32_t>(std::lround((hub_grid.y - 2.0f * r) / s.dx));
        const auto i = static_cast<uint32_t>(std::lround(hub_grid.x / s.dx));
        const auto kk = static_cast<uint32_t>(std::lround(hub_grid.z / s.dx));
        if (j >= s.ny || i < 1u || kk < 1u || i >= s.nx || kk >= s.nz) return std::vector<double>{0.0};
        const auto v = [&](uint32_t a, uint32_t b) { return static_cast<double>(g.v[airflow::v_index(s, a, j, b)]); };
        return std::vector<double>{-0.25 * (v(i - 1u, kk - 1u) + v(i, kk - 1u) + v(i - 1u, kk) + v(i, kk))};
    };
    film.captions = [](const FrameLog& log) {
        const double v = log.live.empty() ? 0.0 : log.live[0];
        return CaptionText{{}, {"this run, instantaneous at the red cross: " + fixed(v, 2) + " m/s"}};
    };
    const float speed_max = 2.0f * vh;
    film.expected_speed = speed_max;
    const glm::vec3 target = hub - glm::vec3(0.0f, 4.5f * r, 0.0f);
    film.views.push_back(View{
        "speed", "speed |u| as a volume, 0 to " + fixed(speed_max, 1) + " m/s; streamlines from above the disc",
        [d, speed_max, target, hub, r](const spade::FlowVolume& v, uint32_t k, uint32_t n,
                                           render::RenderScene& scene, render::Camera& camera) -> spade::Result<void> {
            const auto tf = volume_transfer(render::FlowQuantity::speed, render::ColourMapId::viridis, speed_max,
                                            0.15f, 4.0f / (2.0f * r));
            scene.flow_layers.push_back(render::FlowLayer{&v, {tf}, {}});
            const std::vector<glm::vec3> seeds = render::seed_ring(hub + glm::vec3(0.0f, 0.8f * r, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), 0.7f * r, 12);
            render::StreamlineOptions options;
            options.max_steps = 1500;
            const glm::vec3 white(0.92f);
            if (auto ok = add_streamlines(scene, v, seeds, options,
                                          colours_over(render::FlowQuantity::speed, render::ColourMapId::viridis, 0.0f, speed_max),
                                          &white);
                !ok) {
                return ok;
            }
            add_rims(scene, d);
            // The record's probe, 2R below the disc on its axis.
            const glm::vec3 probe = hub - glm::vec3(0.0f, 2.0f * r, 0.0f);
            const glm::vec3 red(1.0f, 0.25f, 0.25f);
            const float a = 0.25f * r;
            scene.flow_polylines.push_back(render::FlowPolyline{{probe - glm::vec3(a, 0, 0), probe + glm::vec3(a, 0, 0)}, {red, red}});
            scene.flow_polylines.push_back(render::FlowPolyline{{probe - glm::vec3(0, 0, a), probe + glm::vec3(0, 0, a)}, {red, red}});
            camera = orbit(target, 16.0f * r, sweep_azimuth(20.0f, 70.0f, k, n), 12.0f);
            return {};
        }});
    return film;
}

[[nodiscard]] Film cavity_film(uint32_t frames) {
    Film film;
    film.stem = "cavity-re400";
    film.airflow = cavity_re400_case();
    film.frames = frames;
    film.expected_speed = 1.0f;  // the lid's speed
    film.time_decimals = 1;
    film.captions = [](const FrameLog&) {
        return CaptionText{{}, {"the record's measures come from its own run at T = 60; this clip re-runs the same "
                                "configuration on the build named on its card"}};
    };
    film.views.push_back(View{
        "flow", "speed |u| (0 to 1, the lid's), streamlines (white), Ghia's sampling lines x = 0.5 and y = 0.5 (grey)",
        [](const spade::FlowVolume& v, uint32_t, uint32_t, render::RenderScene& scene,
           render::Camera& camera) -> spade::Result<void> {
            render::FieldLayer plane;
            plane.center = glm::vec3(0.5f, 0.5f, 0.0f);
            plane.cells_u = 256;
            plane.cells_v = 256;
            if (auto ok = add_slice(scene, v, render::FlowQuantity::speed, plane, render::ColourMapId::viridis, 0.0f, 1.0f);
                !ok) {
                return ok;
            }
            const float z = 0.5f * v.header.cell;
            std::vector<glm::vec3> seeds = render::seed_rake(glm::vec3(0.5f, 0.04f, z), glm::vec3(0.5f, 0.96f, z), 14);
            for (const glm::vec3& s : render::seed_rake(glm::vec3(0.06f, 0.5f, z), glm::vec3(0.94f, 0.5f, z), 8)) seeds.push_back(s);
            render::StreamlineOptions options;
            options.max_steps = 900;
            const glm::vec3 white(0.95f);
            if (auto ok = add_streamlines(scene, v, seeds, options,
                                          colours_over(render::FlowQuantity::speed, render::ColourMapId::viridis, 0.0f, 1.0f),
                                          &white);
                !ok) {
                return ok;
            }
            const glm::vec3 grey(0.55f);
            const float zl = 1.5f * z;
            scene.flow_polylines.push_back(render::FlowPolyline{{{0.5f, 0.0f, zl}, {0.5f, 1.0f, zl}}, {grey, grey}});
            scene.flow_polylines.push_back(render::FlowPolyline{{{0.0f, 0.5f, zl}, {1.0f, 0.5f, zl}}, {grey, grey}});
            camera = orbit(glm::vec3(0.5f, 0.5f, 0.0f), 1.25f, 0.0f, 0.0f);
            return {};
        }});
    return film;
}

[[nodiscard]] Film taylor_green_film(uint32_t frames, uint32_t cells = 64) {
    Film film;
    film.stem = "taylor-green";
    film.airflow = taylor_green_case(cells);
    film.frames = frames;
    film.expected_speed = 1.0f;  // the vortex's peak speed at the start
    film.time_decimals = 2;
    film.live = [](const airflow::MacGrid& g) {  // kinetic energy per unit mass, as the record measures it
        const airflow::GridShape& s = g.shape;
        double sum = 0.0;
        for (uint32_t k = 0; k < s.nz; ++k) {
            for (uint32_t j = 0; j < s.ny; ++j) {
                for (uint32_t i = 0; i < s.nx; ++i) {
                    const double u = g.u[airflow::u_index(s, i, j, k)], v = g.v[airflow::v_index(s, i, j, k)],
                                 w = g.w[airflow::w_index(s, i, j, k)];
                    sum += u * u + v * v + w * w;
                }
            }
        }
        return std::vector<double>{0.5 * sum / (static_cast<double>(s.nx) * s.ny * s.nz)};
    };
    film.captions = [](const FrameLog& log) {
        const double ke = log.live.empty() ? 0.0 : log.live[0];
        return CaptionText{{"inviscid: the exact solution keeps its energy, so every loss here is the numerics' own "
                            "dissipation"},
                           {"this run: kinetic energy per unit mass " + fixed(ke, 6) + " (" +
                            fixed(100.0 * ke / 0.125, 1) + "% of t = 0)"}};
    };
    film.views.push_back(View{
        "q", "Q-criterion surfaces, Q-hat = 0.5, coloured by speed |u|, 0 to 1",
        [](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
           render::Camera& camera) -> spade::Result<void> {
            render::IsoSurface iso;
            iso.quantity = render::FlowQuantity::q_hat;
            iso.value = 0.5f;
            iso.colour_by = colours_over(render::FlowQuantity::speed, render::ColourMapId::viridis, 0.0f, 1.0f);
            scene.flow_layers.push_back(render::FlowLayer{&v, {}, {iso}});
            camera = orbit(volume_centre(v), 14.5f, sweep_azimuth(30.0f, 80.0f, k, n), 24.0f);
            return {};
        }});
    return film;
}

// --- running a film --------------------------------------------------------------------

// Below this lambda2-hat (1/s) a frame has a vortex worth a surface; above
// it, as in a box of uniform wind, the deepest value is the grid's noise.
constexpr float kVortexFloor = -5.0f;

// A film flown through LiveRun (D1, --case): the flow from two views, with
// the held drone's parts drawn in each where the run held it (render_parts,
// RND-9), and the controls and run line in its captions. `gpu`: its fluid
// on GpuFluid with no parts in the flow (--gpu), which its card says first.
// `particle_views`: the two particle views are web-only (--particles, --particles-preview on S2 and S8): an ordinary film draws every
// view it has, and a particle view with no driver is an error, so they are not added unless asked for.
[[nodiscard]] spade::Result<Film> case_film(LiveCase lc, uint32_t frames, std::string stem, bool gpu = false,
                                            const Look& look = {}, bool particle_views = false,
                                            const std::optional<QLook>& nasa_look = std::nullopt, bool particle_volume = false) {
    namespace an = spade::analysis;
    auto prepared = spade::render_parts::prepare_parts(lc.draw_model);
    if (!prepared) return std::unexpected(prepared.error());
    const auto parts = std::make_shared<const spade::render_parts::PreparedParts>(std::move(*prepared));
    const auto run = std::make_shared<const LiveCase>(std::move(lc));
    Film film;
    film.stem = std::move(stem);
    film.run = run;
    film.gpu = gpu;
    film.frames = frames;
    film.expected_speed = expected_speed(*run);
    film.time_unit = " s";
    film.airflow.name = run->name;
    film.airflow.t_end = static_cast<float>(run->seconds);
    film.airflow.fluid.h = run->c.time.step.value_or(1.0e-3f);
    const char* kinds[] = {"hover", "forward flight", "hover in wind"};
    film.airflow.settings = std::string(gpu ? "the fluid on the GPU (B7's GpuFluid), NO PARTS in the flow, the "
                                              "rotors held at their hover speed; the drone held at the box's centre: "
                                            : "LiveRun, the drone held at the box's centre: ") +
                            kinds[static_cast<uint32_t>(run->c.flight.kind) % 3u] + " at " +
                            fixed(run->c.flight.speed, 1) + " m/s, flying " + run->program_name + " for " +
                            fixed(run->seconds, 2) + " s";
    film.run_labels = std::make_shared<std::map<uint64_t, std::string>>();
    const auto labels = film.run_labels;
    film.captions = [run, labels](const FrameLog& log) {
        CaptionText t;
        // The frame's run_label(): the rotor model, the grid, dx and R/dx,
        // and any limit the propulsion was at.
        const auto label = labels->find(log.step);
        t.top.push_back(label != labels->end() ? label->second
                                               : "(no run label cached for step " + std::to_string(log.step) + ")");
        if (!run->notes.empty()) t.top.push_back(run->notes);
        const an::Controls c = an::controls_at(run->program, static_cast<float>(log.time));
        t.bottom.push_back(std::string("controls: ") + (c.armed ? "armed" : "disarmed") + ", throttle " +
                           fixed(c.throttle, 2) + ", pitch " + fixed(c.pitch_deg, 0) + " deg, roll " +
                           fixed(c.roll_deg, 0) + " deg, yaw " + fixed(c.yaw_deg, 0) + " deg");
        return t;
    };
    const auto pose = std::make_shared<HeldPose>();
    film.held_pose = pose;
    film.particle_hold = std::make_shared<ParticleHold>();
    film.particle_pool = [run](const FrameLog& log) -> std::vector<glm::vec3> {
        if (log.live.size() < 7u) return {};
        const glm::vec3 pos(static_cast<float>(log.live[0]), static_cast<float>(log.live[1]), static_cast<float>(log.live[2]));
        const glm::quat orient(static_cast<float>(log.live[3]), static_cast<float>(log.live[4]), static_cast<float>(log.live[5]),
                               static_cast<float>(log.live[6]));
        std::vector<PreviewRotor> rotors;
        for (const spade::vehicles::RotorDesc& r : run->run_model.rotors) {
            rotors.push_back({pos + orient * r.local_pos, glm::normalize(orient * (r.local_orient * glm::vec3(0.0f, 1.0f, 0.0f))),
                              r.radius});
        }
        const PreviewBox box{glm::vec3(-0.44f, -0.05f, -0.30f), glm::vec3(0.44f, 0.34f, 0.30f)};
        return spread_pool(preview_seeds(rotors, box, 6000u, 30000u));
    };
    film.decorate = [run, parts, pose](const FrameLog& log, render::RenderScene& scene) -> spade::Result<void> {
        if (log.live.size() >= 7u) {
            pose->pos = glm::vec3(static_cast<float>(log.live[0]), static_cast<float>(log.live[1]),
                                  static_cast<float>(log.live[2]));
            pose->orient = glm::quat(static_cast<float>(log.live[3]), static_cast<float>(log.live[4]),
                                     static_cast<float>(log.live[5]), static_cast<float>(log.live[6]));
            pose->known = true;
        }
        if (parts->parts.empty()) return {};
        if (log.live.size() < 7u) {
            return std::unexpected(spade::Error{spade::Code::internal, "a LiveRun frame's log holds no held body"});
        }
        spade::BodyState b{};
        b.pos = glm::vec3(static_cast<float>(log.live[0]), static_cast<float>(log.live[1]),
                          static_cast<float>(log.live[2]));
        b.orient = glm::quat(static_cast<float>(log.live[3]), static_cast<float>(log.live[4]),
                             static_cast<float>(log.live[5]), static_cast<float>(log.live[6]));
        const std::size_t rotors = run->draw_model.rotors.size();
        std::vector<float> omega(rotors, 0.0f), psi(rotors, 0.0f);  // psi: Physics' rule (P11) is to come
        for (std::size_t i = 0; i < rotors && 7u + i < log.live.size(); ++i) omega[i] = static_cast<float>(log.live[7u + i]);
        auto posed = spade::vehicles::posed_parts(run->draw_model, b, omega, psi, spade::vehicles::PartFilter::all);
        if (!posed) return std::unexpected(posed.error());
        const auto base = static_cast<uint32_t>(scene.materials.size());
        scene.materials.push_back(render::Material{.base_color = glm::vec4(0.80f, 0.80f, 0.84f, 1.0f)});
        scene.materials.push_back(render::Material{.base_color = glm::vec4(0.30f, 0.32f, 0.36f, 1.0f)});
        scene.materials.push_back(render::Material{.base_color = glm::vec4(0.55f, 0.62f, 0.72f, 1.0f)});
        return spade::render_parts::bind_parts(
            *parts, *posed, spade::render_parts::PartsLook{.body = base, .rotor = base + 1u, .porous = base + 2u},
            scene);
    };
    const float speed_top = run->speed_max > 0.0f ? run->speed_max : std::max(1.0f, 2.0f * run->c.flight.speed);
    const float vortex_fraction = run->vortex_fraction;
    const render::TransferFunction speed_colours =
        colours_over(render::FlowQuantity::speed, render::ColourMapId::viridis, 0.0f, speed_top);
    film.views.push_back(View{
        "flow",
        (look.l2_level > 0.0f ? "lambda2 vortex surfaces at lambda2-hat = -" + fixed(look.l2_level, 0) + " 1/s"
                              : "lambda2 vortex surfaces at 1/" + fixed(1.0 / vortex_fraction, 0) +
                                    " of the frame's deepest core") +
            ", coloured by speed |u|, 0 to " + fixed(speed_top, 0) + " m/s",
        [speed_colours, vortex_fraction, look](const spade::FlowVolume& v, uint32_t k, uint32_t n,
                                               render::RenderScene& scene, render::Camera& camera) -> spade::Result<void> {
            if (const std::optional<float> level = vortex_iso(v, look, vortex_fraction, kVortexFloor)) {
                render::IsoSurface iso;
                iso.quantity = render::FlowQuantity::lambda2_hat;
                iso.value = *level;
                iso.colour_by = speed_colours;
                scene.flow_layers.push_back(render::FlowLayer{&v, {}, {iso}});
            }
            // The drone and the wake below it: aimed a little under the
            // drone, near enough that it is the picture, not a speck in it.
            const glm::vec3 size = volume_size(v);
            camera = orbit(volume_centre(v) - glm::vec3(0.0f, 0.12f * size.y, 0.0f),
                           0.95f * std::max({size.x, size.y, size.z}), sweep_azimuth(30.0f, 60.0f, k, n), 20.0f);
            return {};
        }});
    // The web masters' D1 views (the lead, 2026-11-10, L54): the quad's own two, on the held drone's rotors. Speed
    // 0 to 12 m/s as the quad's were. The camera is the clip's.
    {
        constexpr float kWebSpeedTop = 12.0f;
        const render::TransferFunction web_colours =
            colours_over(render::FlowQuantity::speed, render::ColourMapId::viridis, 0.0f, kWebSpeedTop);
        const auto frame_on_drone = [](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::Camera& camera) {
            const glm::vec3 size = volume_size(v);
            camera = orbit(volume_centre(v), 0.95f * std::max({size.x, size.y, size.z}), sweep_azimuth(30.0f, 60.0f, k, n),
                           20.0f);
        };
        film.views.push_back(View{
            "speed-volume", "speed |u| as a volume, 0 to 12 m/s",
            [frame_on_drone](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                             render::Camera& camera) -> spade::Result<void> {
                scene.flow_layers.push_back(render::FlowLayer{
                    &v, {volume_transfer(render::FlowQuantity::speed, render::ColourMapId::viridis, kWebSpeedTop, 0.15f, 30.0f)},
                    {}});
                frame_on_drone(v, k, n, camera);
                return {};
            }});
        if (nasa_look) {
            film.views.push_back(View{
                "vorticity-cloud", "vorticity magnitude as a soft cloud (blue low to magenta high), no surfaces",
                [frame_on_drone, q = *nasa_look](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                                                 render::Camera& camera) -> spade::Result<void> {
                    frame_on_drone(v, k, n, camera);
                    if (auto tf = vorticity_cloud(v, q)) scene.flow_layers.push_back(render::FlowLayer{&v, {*tf}, {}});
                    return {};
                }});
            // NASA Ames' multirotor look: Q-criterion surfaces (a ladder of three levels) coloured by vorticity, blue to magenta.
            film.views.push_back(View{
                "nasa", "Q-criterion surfaces at three levels, coloured by vorticity magnitude (blue low to magenta high)",
                [frame_on_drone, q = *nasa_look](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                                                 render::Camera& camera) -> spade::Result<void> {
                    frame_on_drone(v, k, n, camera);
                    std::vector<render::IsoSurface> isos = q_isos(v, q);
                    if (!isos.empty()) scene.flow_layers.push_back(render::FlowLayer{&v, {}, std::move(isos)});
                    return {};
                }});
        }
        if (particle_views) film.views.push_back(View{
            "particles", "points carried through every stored frame, from the air above and round the rotors, through them and down",
            [frame_on_drone, hold = film.particle_hold, with_volume = particle_volume](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                                                         render::Camera& camera) -> spade::Result<void> {
                frame_on_drone(v, k, n, camera);
                // --volume: the speed volume under the points (the same layer as the speed-volume view)
                if (with_volume) {
                    scene.flow_layers.push_back(render::FlowLayer{
                        &v, {volume_transfer(render::FlowQuantity::speed, render::ColourMapId::viridis, kWebSpeedTop, 0.15f, 30.0f)}, {}});
                }
                if (!hold->driver) return std::unexpected(spade::Error{spade::Code::internal, "the particle view ran without a driver"});
                const ParticleField* field = hold->driver->at(hold->first + k);
                if (field == nullptr) {
                    return std::unexpected(spade::Error{spade::Code::internal, "the particle field is not at frame " +
                                                                                     std::to_string(hold->first + k)});
                }
                // Additive streaks: light on black, the capsule path (GL only), a 2 px core with a soft 5 px glow.
                scene.flow_polylines = particle_streaks(*field, v, hold->max_age_s, hold->look);
                scene.flow_line_style = render::FlowLineStyle{hold->look.width_px, hold->look.glow_px, hold->look.glow_strength};
                scene.flow_line_style.additive = true;
                return {};
            }});
        if (particle_views) film.views.push_back(View{
            "particles-preview", "tracers carried through this one frozen frame (a preview of the particle look, not time-accurate)",
            [frame_on_drone, with_volume = particle_volume](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                             render::Camera& camera) -> spade::Result<void> {
                frame_on_drone(v, k, n, camera);
                if (with_volume) {
                    scene.flow_layers.push_back(render::FlowLayer{
                        &v, {volume_transfer(render::FlowQuantity::speed, render::ColourMapId::viridis, kWebSpeedTop, 0.15f, 30.0f)}, {}});
                }
                return {};
            },
            [run, pose](const spade::FlowVolume& v, render::RenderScene& scene) -> spade::Result<std::string> {
                if (!pose->known) {
                    return std::unexpected(spade::Error{spade::Code::internal, "the particle preview ran before a frame's held body was read"});
                }
                std::vector<PreviewRotor> rotors;
                for (const spade::vehicles::RotorDesc& r : run->run_model.rotors) {
                    rotors.push_back({pose->pos + pose->orient * r.local_pos,
                                      glm::normalize(pose->orient * (r.local_orient * glm::vec3(0.0f, 1.0f, 0.0f))), r.radius});
                }
                const PreviewBox box{glm::vec3(-0.42f, -0.05f, -0.28f), glm::vec3(0.42f, 0.40f, 0.28f)};
                std::vector<render::TracerSource> sources;
                for (const glm::vec3& seed : preview_seeds(rotors, box, 120u, 500u)) sources.push_back({seed, 0.005f});
                render::TracerState state;
                render::TracerOptions options;
                options.capacity = 400000u;
                options.max_age_s = 0.8f;
                for (int step = 0; step < 600; ++step) {
                    if (auto ok = render::advance_tracers(state, v, sources, 0.001f, options); !ok) return std::unexpected(ok.error());
                }
                render::FlowPoints points;
                points.size_px = 4u;
                for (std::size_t i = 0; i < state.positions.size(); ++i) {
                    const float t = std::clamp(state.ages[i] / 0.8f, 0.0f, 1.0f);
                    points.positions.push_back(state.positions[i]);
                    points.colours.push_back(glm::mix(glm::vec3(0.65f, 0.92f, 1.0f), glm::vec3(0.12f, 0.28f, 0.75f), t));
                }
                scene.flow_points = std::move(points);
                return std::string();
            }});
        film.views.push_back(View{
            "streamlines", "streamlines seeded 6 cm below each rotor and traced both ways, thin and faded toward their ends, coloured by speed |u|, "
            "0 to 12 m/s",
            [frame_on_drone](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene&,
                             render::Camera& camera) -> spade::Result<void> {
                frame_on_drone(v, k, n, camera);
                return {};
            },
            [run, pose, web_colours](const spade::FlowVolume& v, render::RenderScene& scene) -> spade::Result<std::string> {
                if (!pose->known) {
                    return std::unexpected(spade::Error{spade::Code::internal, "the streamline view ran before a frame's "
                                                                              "held body was read"});
                }
                std::vector<glm::vec3> seeds;
                for (const spade::vehicles::RotorDesc& r : run->run_model.rotors) {
                    const glm::vec3 hub = pose->pos + pose->orient * r.local_pos;
                    const glm::vec3 axis = glm::normalize(pose->orient * (r.local_orient * glm::vec3(0.0f, 1.0f, 0.0f)));
                    for (const float ring : {0.35f, 0.75f}) {
                        for (const glm::vec3& s : render::seed_ring(hub - axis * web::kWebSeedBelowHubM, axis, ring * r.radius, 8)) {
                            seeds.push_back(s);
                        }
                    }
                }
                render::StreamlineOptions options;
                options.max_steps = 1200;
                const std::size_t first_line = scene.flow_polylines.size();
                if (auto ok = add_streamlines(scene, v, seeds, options, web_colours); !ok) {
                    return std::unexpected(ok.error());
                }
                fade_polylines_from(scene, first_line, web::kSoftLineFadeM);  // S1 only draws this view's lines soft
                // The fade is in the lines' alphas, which need a capsule: the view carries the soft capsule style itself, so a film
                // that draws it (the ordinary case film too) never meets alphas on a hairline.
                scene.flow_line_style = render::FlowLineStyle{web::kSoftLineWidthPx, web::kSoftLineGlowPx, web::kSoftLineGlowStrength, true};
                return std::string();
            }});
    }
    film.views.push_back(View{
        "slice", "speed |u| on the vertical plane through the drone, 0 to " + fixed(speed_top, 0) + " m/s",
        [speed_colours](const spade::FlowVolume& v, uint32_t, uint32_t, render::RenderScene& scene,
                        render::Camera& camera) -> spade::Result<void> {
            const glm::vec3 size = volume_size(v), centre = volume_centre(v);
            render::FieldLayer plane;
            plane.center = centre;
            plane.width = size.x;
            plane.height = size.y;
            plane.cells_u = 2u * v.header.nx;
            plane.cells_v = 2u * v.header.ny;
            if (auto ok = add_slice(scene, v, render::FlowQuantity::speed, plane, render::ColourMapId::viridis, 0.0f,
                                    speed_colours.range_max);
                !ok) {
                return ok;
            }
            camera = orbit(centre, 1.7f * std::max(size.x, size.y), 0.0f, 0.0f);
            return {};
        }});
    if (!run->run_model.parts.empty()) {
        // The parts are in the flow (D1): C_p on them, as the editor's C_p
        // view reads it (RND-8, R2.4: one cell off each wall, on the seen
        // side), against the inflow's pressure, over the discs' loading T/A
        // = 1/2 rho (2 v_h)^2. The legend's note goes under the label as
        // apply_view() gives it (D1-4: the caption quotes it verbatim).
        const float two_v_h = std::max(1.0f, run->speed_max - run->c.flight.speed);
        const float q = 0.5f * run->c.fluid.density * two_v_h * two_v_h;
        // `search` is the reach past a solid at the read (FlowSurfaceColouring::search_cells): 0 for S4's C_p, whose parts ARE the flow's solids; the pressure film
        // paints the detailed kit meshes, which are not, so it goes on outward to the nearest fluid cell (O124).
        const auto cp_view = [&](const char* name, float search, float range, bool shade) {
            return View{
                name,
                "C_p = (p - p_inflow) / q on the airframe, q = 1/2 rho (2 v_h)^2 = T/A = " + fixed(q, 1) +
                    " Pa; blue -" + fixed(range, 2) + " to red +" + fixed(range, 2),
                [](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene&,
                   render::Camera& camera) -> spade::Result<void> {
                    // Close on the drone, from above and upstream, so the arms'
                    // tops under the discs show.
                    camera = orbit(volume_centre(v), 0.35f, sweep_azimuth(240.0f, 60.0f, k, n), 40.0f);
                    return {};
                },
                [q, search, range, shade](const spade::FlowVolume& v, render::RenderScene& scene) -> spade::Result<std::string> {
                    render::FlowViewSpec cp;
                    cp.kind = render::FlowView::surface_cp;
                    cp.colours = render::ColourMapId::cool_warm;
                    cp.range_min = -range;
                    cp.range_max = range;
                    cp.q_ref = q;
                    cp.p_ref = static_cast<float>(inflow_pressure(v));
                    cp.search_cells = search;
                    cp.shade = shade;
                    render::RenderOptions options;
                    auto legends = render::apply_view(render::ViewSpec{render::ViewBase::shaded, {cp}}, &v, scene,
                                                      options);
                    if (!legends) return std::unexpected(legends.error());
                    return legends->empty() ? std::string() : legends->front().note;
                }};
        };
        film.views.push_back(cp_view("cp", 0.0f, 1.0f, false));
        film.views.push_back(cp_view("surface-pressure", web::kSurfacePressureSearchCells, web::kSurfacePressureCpRange, false));
        film.views.push_back(cp_view("surface-pressure-shaded", web::kSurfacePressureSearchCells, web::kSurfacePressureCpRange, true));
    }
    return film;
}

// --- the website masters ---------------------------------------------------------------------------
// Cameron's render brief (relayed by the lead, 2026-11-08): S1 to S6 and S8 of web_masters.hpp (S7 dropped), 3840x2160 at 30 fps on pure
// black, no text, one libx264 pass, and a 5120x2880 PNG of four of them. Each clip is one view of a film built here
// from the films above: their shots with the camera replaced. The cameras' numbers are the film's to tune from its
// stills (the timing job's), so they sit together in web_film().

// What a clip is filmed with: its group's frames and window, or the command line's.
struct WebPlan {
    uint32_t frames = 0;
    double from_s = 0.0, to_s = 0.0;
    uint32_t steps_per_frame = 0;  // 0: --window's even spread over [from_s, to_s]
    std::optional<spade::physics::airflow::GridShape> grid;  // D1's, when not kD1Grid
    float sgs_cs = web::kD1WebSgsCs;  // the D1 clips' Smagorinsky Cs: the master's, or --sgs-cs's (a diagnostic)
    std::optional<float> cell_mm;       // --cell-mm 2.5: the X500's box and hold (L106c)
    bool hover_start = false;           // --hover-start (needs --lines)
    bool volume = false;                // --volume (with --particles or --particles-preview)
    std::optional<uint32_t> step_us;    // --step-us N: the solver step in microseconds, in place of kD1WebStepS (a --window run)
    Look look;                        // the look options (look.hpp): declared in the caption, never hidden
    const web::Drone* drone = &web::default_drone();  // the D1 clips' airframe (--drone)
    bool particles = false;                           // S2 and S8: points carried through every stored frame
    bool particles_preview = false;                   // S2 and S8: tracers through one frozen frame, not the volume
    std::string visual_mesh;                          // --visual-mesh: a glTF drawn in place of the model's parts (presentation only)
    std::string build_parts;                          // --build-parts: Core's parts table drawn over the held body (presentation only)
    std::string part_store;                           // --part-store: the store the table's objects are read from
    PropBlur prop_blur;                               // --prop-blur: the rotors' blade blur
    DroneLight drone_light;                           // --drone-light: the key and fill strength the drone is lit with (presentation only)
    std::optional<float> elevation;                   // --elevation DEG: the still's camera elevation in place of the drone's framing
    std::string lines;                                // --lines: the prop part file (under the assets) of the rotors' actuator lines
    bool cloud = false;                               // --nasa-cloud: the vorticity as a soft cloud in place of the surfaces
    bool pressure = false;                            // --surface-pressure: S8 draws the air's pressure on the drawn parts, nothing else of the air
    bool shade = false;                               // --surface-shade: ... lit for shape
    std::optional<QLook> nasa;                        // --nasa: S2 and S8 draw Q-criterion surfaces coloured by vorticity
};

constexpr float kWebFov = 1.0471975511965976f;  // render::Camera's default, 60 degrees
constexpr float kWebAspect = static_cast<float>(web::kWidth) / static_cast<float>(web::kHeight);

using Aim = std::function<render::Camera(const spade::FlowVolume&, uint32_t frame, uint32_t frames)>;

[[nodiscard]] const View* view_named(const Film& film, const std::string& name) {
    for (const View& v : film.views) {
        if (v.name == name) return &v;
    }
    return nullptr;
}

// `base`'s view with its camera replaced: the shot (the layers, and the rims) runs as it did, then `aim` sets the
// camera. `rims`: the discs' rims stay, else they are dropped.
[[nodiscard]] View with_camera(const View& base, Aim aim, bool rims) {
    return View{base.name, base.label,
                [shot = base.shot, aim = std::move(aim), rims](const spade::FlowVolume& v, uint32_t k, uint32_t n,
                                                              render::RenderScene& scene,
                                                              render::Camera& camera) -> spade::Result<void> {
                    if (auto ok = shot(v, k, n, scene, camera); !ok) return ok;
                    if (!rims) scene.flow_polylines.clear();
                    camera = aim(v, k, n);
                    return {};
                },
                base.on_parts};
}

// The website masters' streamlines: capsules 4 px wide with a 10 px glow at 0.3, at 3840 x 2160 (the lead,
// 2026-11-10: line width as quads, coverage anti-aliasing and a glow, on black). The one-pixel line is a hairline
// at 4K; the engine's default stays it, so only a view that asks gets this.
constexpr render::FlowLineStyle kWebLineStyle{4.0f, 10.0f, 0.3f};
// Additive: the strands are light that fades by alpha, overlapping ones add, and the drone behind them is not darkened.
constexpr render::FlowLineStyle kWebSoftLineStyle{web::kSoftLineWidthPx, web::kSoftLineGlowPx, web::kSoftLineGlowStrength, true};

// `base`'s view with its polylines (the streamlines and the rims) drawn in `style`.
[[nodiscard]] View with_line_style(const View& base, render::FlowLineStyle style) {
    return View{base.name, base.label,
                [shot = base.shot, style](const spade::FlowVolume& v, uint32_t k, uint32_t n, render::RenderScene& scene,
                                          render::Camera& camera) -> spade::Result<void> {
                    if (auto ok = shot(v, k, n, scene, camera); !ok) return ok;
                    scene.flow_line_style = style;
                    return {};
                },
                base.on_parts};
}

[[nodiscard]] std::string web_stem(const web::Group& group, const WebPlan& plan) {
    char window[64];
    std::snprintf(window, sizeof window, "%g-%g", plan.from_s, plan.to_s);
    std::string stem = "web-" + std::string(group.name) + "-" + window + "-" + std::to_string(plan.frames) + "f";
    stem += web::sgs_cs_suffix(plan.sgs_cs);  // a run at another Cs has a folder of its own
    stem += web::cell_step_stem_suffix(plan.cell_mm, plan.step_us);  // and one at another cell or step
    stem += web::volume_stem_suffix(plan.volume);
    if (plan.hover_start) stem += "-hoverstart";                      // and one whose rotors start at speed
    stem += web::drone_stem_suffix(*plan.drone);  // and one at another drone
    stem += web::lines_stem_suffix(!plan.lines.empty());  // and one with the rotors as lines
    return stem;
}

[[nodiscard]] spade::Result<Film> web_film(const web::Clip& clip, const web::Group& group, const WebPlan& plan) {
    const std::string id(clip.id);
    const float fit = web::kMargin;
    Film film;
    View view{"", "", View::Shot{}};
    if (clip.scene == web::Scene::quad) {
        auto made = quad_film(plan.frames);
        if (!made) return std::unexpected(made.error());
        film = std::move(*made);
        const AirflowCase q = film.airflow;
        // S5 and S6 stay on the reference quad's CPU film; S1, S2 and S8 moved to D1 (the lead, L54).
        if (id == "S6") {
            view = with_camera(*view_named(film, "above"), [=](const spade::FlowVolume&, uint32_t, uint32_t) {
                return orbit(glm::vec3(0.0f), web::fit_distance(0.30f, kWebFov, kWebAspect, fit), 0.0f, 80.0f);
            }, true);
        } else {  // S5: the plane through the rotors on +x and -x, the frame's own shape, edge to edge
            const float lo = clip.range_lo, hi = clip.range_hi;
            view = View{
                "slice", "speed |u| on the plane through rotors +x and -x",
                [q, lo, hi](const spade::FlowVolume& v, uint32_t, uint32_t, render::RenderScene& scene,
                            render::Camera& camera) -> spade::Result<void> {
                    // The window the frame sees, inside the box; the plane a little larger so no edge shows.
                    const float window_w = 0.78f, window_h = web::plane_window_height(window_w, kWebAspect);
                    const float hub_y = q.discs[0].centre.y + q.origin.y;
                    const float centre_y = hub_y + 0.12f - 0.5f * window_h;  // 12 cm above the hubs at the top
                    render::FieldLayer plane;
                    plane.center = glm::vec3(0.0f, centre_y, 0.0f);
                    plane.width = 1.02f * window_w;
                    plane.height = 1.02f * window_h;
                    plane.cells_u = 2u * static_cast<uint32_t>(std::lround(plane.width / v.header.cell));
                    plane.cells_v = 2u * static_cast<uint32_t>(std::lround(plane.height / v.header.cell));
                    if (auto ok = add_slice(scene, v, render::FlowQuantity::speed, plane, render::ColourMapId::viridis, lo, hi);
                        !ok) {
                        return ok;
                    }
                    camera = orbit(plane.center, web::fit_plane_distance(window_h, kWebFov), 0.0f, 0.0f);
                    return {};
                }};
        }
    } else {  // D1, the parts in the flow, on the device, at 0.2 ms with Cs 0.17: S1 and S8 in still air, the rest in
              // a crosswind (heading 90: the air moves +z)
        const bool still = clip.scene == web::Scene::d1_still;  // S1, S2, S8; S3 and S4 are in the side wind
        auto lc = d1_web_live_case(plan.to_s, plan.grid.value_or(kD1Grid), still ? 0.0f : 90.0f,
                                   plan.step_us ? static_cast<float>(*plan.step_us) * 1.0e-6f : static_cast<float>(web::kD1WebStepS), plan.sgs_cs,
                                   plan.drone->model_file, d1_web_hold(plan.cell_mm, plan.drone->hub_above_origin_m));
        if (!lc) return std::unexpected(lc.error());
        auto made = case_film(std::move(*lc), plan.frames, still ? "web-d1-still" : "web-d1", false, plan.look,
                               plan.particles || plan.particles_preview, plan.nasa, plan.volume);
        if (!made) return std::unexpected(made.error());
        film = std::move(*made);
        film.device = true;
        {   // what is drawn over the held body, presentation only: the visual mesh and/or the parts build, at the logged pose
            std::shared_ptr<const render::MeshData> visual;
            if (!plan.visual_mesh.empty()) {
                auto loaded = render::load_gltf(plan.visual_mesh);
                if (!loaded) return std::unexpected(spade::Error{loaded.error().code, "--visual-mesh " + plan.visual_mesh + ": " + loaded.error().context});
                visual = std::make_shared<const render::MeshData>(std::move(*loaded));
            }
            std::shared_ptr<const std::vector<BuildPart>> build;
            if (!plan.build_parts.empty()) {
                std::ifstream in(plan.build_parts, std::ios::binary);
                if (!in) return std::unexpected(spade::Error{spade::Code::io_error, "--build-parts " + plan.build_parts + ": cannot be read"});
                const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
                auto parts = parse_build_parts(text);
                if (!parts) return std::unexpected(spade::Error{parts.error().code, "--build-parts " + plan.build_parts + ": " + parts.error().context});
                build = std::make_shared<const std::vector<BuildPart>>(std::move(*parts));
            }
            // the turning rotors' disc is made from the prop part's own chord table: the --lines file (the one the physics reads)
            std::optional<BladeChord> chord;
            {
                auto got = chord_for_draw(plan.prop_blur, build != nullptr, plan.lines.empty() ? std::filesystem::path() : std::filesystem::path(SPADE_ASSETS_DIR) / plan.lines);
                if (!got) return std::unexpected(got.error());
                chord = std::move(*got);
            }
            // the table is in the design frame: the model the run flies says how the body is turned and where its centre of mass is
            BodyFrame frame;
            if (build) {
                auto got = body_frame_of(std::filesystem::path(SPADE_ASSETS_DIR) / plan.drone->model_file);
                if (!got) return std::unexpected(got.error());
                frame = *got;
            }
            if (visual || build) {
                film.decorate = [visual, build, chord, frame, blur = plan.prop_blur, light = plan.drone_light, walls = plan.pressure, store =std::filesystem::path(plan.part_store)](
                                    const FrameLog& log, render::RenderScene& scene) -> spade::Result<void> {
                    if (log.live.size() < 7u) return std::unexpected(spade::Error{spade::Code::internal, "the frame's log has no held body"});
                    const glm::vec3 pos(static_cast<float>(log.live[0]), static_cast<float>(log.live[1]), static_cast<float>(log.live[2]));
                    const glm::quat orient(static_cast<float>(log.live[3]), static_cast<float>(log.live[4]), static_cast<float>(log.live[5]),
                                           static_cast<float>(log.live[6]));
                    if (visual) {
                        if (auto r = add_visual_mesh(scene, *visual, pos, orient); !r) return r;
                    }
                    if (build) {
                        apply_drone_light(scene, light);
                        BuildDraw draw;  // the rotors' logged speeds (live[7 + i]) spread each blade over the shutter
                        draw.design_to_principal = frame.design_to_principal;
                        draw.com_offset = frame.com_offset;
                        draw.blur = blur;
                        draw.chord = chord;
                        draw.omega = rotor_speeds_of_live(log.live, rotor_count_of(*build));  // by the table's rotors: the lines' thrust, last, is not a speed
                        draw.psi = rotor_azimuths_of_live(log.live, rotor_count_of(*build));  // --prop-blur mode=blades: the simulation's blade azimuths, the record's last values
                        if (blur.blades && walls) draw.blade_flow_gain = web::blade_flow_gain();  // the blades read the air on their own range
                        draw.flow_walls = walls;  // --surface-pressure: the drawn parts take the pressure colour (the blade blur is no wall)
                        return add_build_parts(scene, *build, store, pos, orient, draw);
                    }
                    return {};
                };
            }
        }
        if (!plan.lines.empty()) {
            auto lines = live_lines_from_prop_file(std::filesystem::path(SPADE_ASSETS_DIR) / plan.lines, 8u);
            if (!lines) return std::unexpected(lines.error());
            film.lines = std::make_shared<const spade::analysis::LiveLines>(std::move(*lines));
            film.hover_start = plan.hover_start;
        }
        film.blade_azimuth = plan.prop_blur.blades;
        // S1, S8 and S2 are framed closer, looking down 8 degrees, so the deeper box's floor stays out of frame (L63;
        // web_masters.hpp's constants, the floor's projection pinned by FlowMovieWeb.TheStillAirFramingKeepsTheFloor...).
        // S3 and S4 are still PROVISIONAL: the quad's radii, not yet fitted on D1's side-wind field.
        const glm::vec3 below_drone = web::subject_point(0.08f);  // the drone's COM, not the box's centre (L60)
        web::Framing framing = plan.drone->framing;  // the reference quad's is the constants this table always had
        if (plan.elevation) framing.elevation_deg = *plan.elevation;  // --elevation: a close-up from higher up
        const glm::vec3 still_aim = web::subject_point(framing.aim_below_m);
        if (id == "S1") {
            view = with_camera(*view_named(film, "streamlines"), [=](const spade::FlowVolume&, uint32_t k, uint32_t n) {
                return orbit(still_aim, web::fit_distance(framing.subject_m, kWebFov, kWebAspect, fit),
                             web::loop_azimuth(web::kD1StillAzimuthDeg, 360.0f, k, n), framing.elevation_deg);
            }, false);
        } else if (id == "S8") {
            view = with_camera(*view_named(film, plan.pressure ? (plan.shade ? "surface-pressure-shaded" : "surface-pressure") : plan.nasa ? (plan.cloud ? "vorticity-cloud" : "nasa") : plan.particles ? "particles" : plan.particles_preview ? "particles-preview" : "speed-volume"),
                               [=](const spade::FlowVolume&, uint32_t k, uint32_t n) {
                return orbit(still_aim, web::fit_distance(framing.subject_m, kWebFov, kWebAspect, fit),
                             web::there_and_back_azimuth(web::kD1StillAzimuthDeg, clip.orbit_degrees, k, n),
                             framing.elevation_deg);
            }, plan.particles);  // the streaks are polylines the view's shot makes: kept (with_camera clears them otherwise)
        } else if (id == "S2") {  // the soft speed volume of S8, from the start-up, in a push-in (Cameron, 2026-11-17)
            view = with_camera(*view_named(film, plan.nasa ? (plan.cloud ? "vorticity-cloud" : "nasa") : plan.particles ? "particles" : plan.particles_preview ? "particles-preview" : "speed-volume"),
                               [=](const spade::FlowVolume&, uint32_t k, uint32_t n) {
                const float wide = web::fit_distance(framing.push_wide_m, kWebFov, kWebAspect, fit);
                const float close = web::fit_distance(framing.push_close_m, kWebFov, kWebAspect, fit);
                return orbit(still_aim, web::push_in_distance(wide, close, k, n), web::kD1StillAzimuthDeg,
                             framing.elevation_deg);
            }, plan.particles);
        } else if (id == "S3") {
            view = with_camera(*view_named(film, "flow"), [=](const spade::FlowVolume&, uint32_t k, uint32_t n) {
                return orbit(below_drone, web::fit_distance(0.42f, kWebFov, kWebAspect, fit),
                             web::loop_azimuth(300.0f, clip.orbit_degrees, k, n), 20.0f);
            }, true);
        } else {  // S4: C_p on the airframe, filling 60% of the frame's height
            view = with_camera(*view_named(film, "cp"), [=](const spade::FlowVolume&, uint32_t k, uint32_t n) {
                return orbit(web::subject_point(), web::fill_distance(0.30f, 0.6f, kWebFov),
                             web::loop_azimuth(240.0f, clip.orbit_degrees, k, n), 40.0f);
            }, true);
        }
    }
    if (id == "S1") view = with_line_style(view, kWebSoftLineStyle);  // soft: thin, glowing, faded at the ends
    if (id == "S6") view = with_line_style(view, kWebLineStyle);
    film.views.clear();
    film.views.push_back(std::move(view));
    film.stem = web_stem(group, plan);
    film.frames = plan.frames;
    film.window_from = plan.from_s;
    film.window_to = plan.to_s;
    film.steps_per_frame = plan.steps_per_frame;
    if (!film.run) film.airflow.t_end = static_cast<float>(plan.to_s);
    return film;
}

// Frame k of the cache, drawn through the film's one view for a clip of n frames: the scene on pure black, the
// view's shot, the held drone's parts and what the view draws on them. `note` takes the view's legend line.

// What the look options keep between frames: the raw frames the mean flow reads, oldest first.
struct LookState {
    Look look;
    struct Held {
        uint32_t index;
        spade::FlowVolume volume;
    };
    std::deque<Held> ring;
    std::size_t peak_ring_frames = 0;  // the most raw frames the ring held at once
    uint64_t peak_ring_bytes = 0;      // and their bytes (the halves of both textures): the mean flow's memory
};

// The look applied to cache frame k, already read into `volume`: the velocity averaged over the window ending at k (the
// cache's own frames, so a clip's first frame averages the ones before it), then lambda2-hat smoothed, then its small
// pieces culled. The ring keeps the window's raw frames, so a clip read in order reads each cached frame once; a jump (a
// still after the clip) reads the window again.
[[nodiscard]] spade::Result<void> apply_look(LookState& state, const fs::path& cache, uint32_t k,
                                             spade::FlowVolume& volume) {
    const Look& look = state.look;
    if (look.mean_frames > 1u) {
        const MeanWindow w = mean_window(k, look.mean_frames);
        auto& ring = state.ring;
        while (!ring.empty() && ring.front().index < w.first) ring.pop_front();
        if (!ring.empty() && ring.back().index + 1u != k) ring.clear();
        for (uint32_t next = ring.empty() ? w.first : ring.back().index + 1u; next < k; ++next) {
            spade::FlowVolume past;
            FrameLog scratch;
            if (auto read = read_frame(cache, next, past, scratch); !read) return read;
            ring.push_back(LookState::Held{next, std::move(past)});
        }
        ring.push_back(LookState::Held{k, volume});
        state.peak_ring_frames = std::max(state.peak_ring_frames, ring.size());
        state.peak_ring_bytes = std::max<uint64_t>(
            state.peak_ring_bytes, uint64_t{ring.size()} * (volume.v.size() + volume.s.size()) * sizeof(uint16_t));
        std::vector<const spade::FlowVolume*> frames;
        for (const LookState::Held& held : ring) frames.push_back(&held.volume);
        if (auto mean = mean_velocity(volume, std::span<const spade::FlowVolume* const>(frames)); !mean) return mean;
    }
    finish_look(look, volume);
    return {};
}

// `ss` is how many times larger than the frame `stage` is (1: the frame's own size): the scene's pixel widths are
// scaled to match, the stage's picture is box-filtered down to the frame, and `draw_s` is the time of the large draw
// (the lead, 2026-11-10: the web masters render at 2x).
[[nodiscard]] spade::Result<void> draw_web_frame(const Film& film, const fs::path& cache, uint32_t first, uint32_t k,
                                                 uint32_t n, GlStage& stage, uint32_t ss, spade::FlowVolume& volume,
                                                 FrameLog& log, std::vector<uint8_t>& rgba, std::string& note,
                                                 double& draw_s, web::FrameTimes* times = nullptr,
                                                 LookState* look_state = nullptr) {
    using Clock = std::chrono::steady_clock;
    const auto since = [](Clock::time_point from) { return std::chrono::duration<double>(Clock::now() - from).count(); };
    auto lap = Clock::now();
    if (auto read = read_frame(cache, first + k, volume, log); !read) return read;
    if (look_state != nullptr && look_state->look.any()) {
        if (auto looked = apply_look(*look_state, cache, first + k, volume); !looked) return looked;
    }
    if (times != nullptr) times->cache_read = since(lap);
    lap = Clock::now();
    const View& view = film.views.front();
    render::RenderScene scene = black_scene(volume);
    render::Camera camera;
    if (auto shot = view.shot(volume, k, n, scene, camera); !shot) return shot;
    note_held_pose(film, log);
    if (film.decorate) {
        const std::size_t first_part_item = scene.statics.size();
        if (auto drawn = film.decorate(log, scene); !drawn) return drawn;
        if (film.role_materials && film.run) {
            std::vector<std::pair<std::string, std::string>> roles;
            for (const spade::vehicles::PartDesc& part : film.run->draw_model.parts) roles.emplace_back(part.name, part.role);
            (void)apply_role_materials(scene, first_part_item, roles);
        }
    }
    note.clear();
    if (view.on_parts) {
        auto on = view.on_parts(volume, scene);
        if (!on) return std::unexpected(on.error());
        note = std::move(*on);
    }
    if (ss > 1u) {
        const auto scale = static_cast<float>(ss);
        scene.flow_line_style.width_px *= scale;
        scene.flow_line_style.glow_px *= scale;
        for (render::FlowPolyline& line : scene.flow_polylines) line.width_px *= scale;
    }
    if (times != nullptr) times->streamline_shot = since(lap);
    const auto start = std::chrono::steady_clock::now();
    // A supersampled frame's row flip and alpha fill are folded into its filter (box_down_flipped): the 4x picture is
    // read once, never copied.
    const bool fold = ss > 1u;
    auto drew = fold ? stage.draw_bottom_first(scene, camera) : stage.draw(scene, camera, rgba);
    draw_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (times != nullptr) {
        times->gl_upload = stage.last_times().upload;
        times->render = stage.last_times().render;
        times->readback = stage.last_times().readback;
        times->flip = stage.last_times().flip;
    }
    lap = Clock::now();
    if (drew && fold) rgba = box_down_flipped(stage.bottom_first(), stage.width() / ss, stage.height() / ss, ss);
    if (times != nullptr) times->box_down = since(lap);
    return drew;
}

// The brief's pure black: the four corners of a frame whose subject keeps its margin are (0, 0, 0).
[[nodiscard]] bool corners_black(const std::vector<uint8_t>& rgba, uint32_t w, uint32_t h) {
    for (const std::size_t at : {std::size_t{0}, static_cast<std::size_t>(w - 1u) * 4u,
                                 static_cast<std::size_t>(h - 1u) * w * 4u,
                                 (static_cast<std::size_t>(h) * w - 1u) * 4u}) {
        if (rgba[at] != 0u || rgba[at + 1u] != 0u || rgba[at + 2u] != 0u) return false;
    }
    return true;
}

[[nodiscard]] std::string utc_now() {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    char out[40];
    std::strftime(out, sizeof out, "%Y-%m-%d %H:%M UTC", &utc);
    return out;
}

struct Options {
    std::string video;
    fs::path out;
    std::string ffmpeg = "ffmpeg";
    double seconds = 0.0;  // 0: the film's own
    uint32_t stills = 0;
    bool resimulate = false;
    bool keep_cache = false;
    bool drop_cache = false;  // --drop-cache: a LiveRun film's cache goes after its render
    bool simulate_only = false;
    bool cpu = false;  // stills on the CPU renderer instead of GL
    std::string view;  // stills of this view only
    int still_frame = -1;  // stills of this frame only (cached; no simulation past it)
    int probe_swirl = -1;  // print each disc's vertical vorticity at this cached frame
    uint32_t n_v = 0;      // 0: the case's own; else its V-cycles (Physics' noise test)
    double budget_seconds = 0.0;  // 0: none
    uint32_t frame_budget = 0;    // 0: none
    fs::path case_file;           // --case: a .case.yaml flown through LiveRun
    fs::path controls_file;       // --controls: its control program (empty: the showcase)
    uint32_t step_time = 0;       // --step-time N: a case's start and N steps timed; no film
    bool arm_pressure = false;    // --arm-pressure: D1's arm-top samples from the cached frames; no film
    std::string synthetic_still;  // --synthetic-still NAME: the web look on a synthetic wake, no simulation
    uint32_t supersample = 0;     // --supersample N: render at N times the frame and box-filter down (0: the mode's
                                  // own: 2 for a web master, 1 for the synthetic still)
    std::string export_name;      // --export-lines NAME: a cached frame's streamlines as JSON (line_export.hpp); no film
    int export_frame = -1;        // --export-frame N: the cached frame to export (default: the last)
    fs::path export_deliver;      // --export-deliver RAW: also copy it to RAW/spade and add a MANIFEST line
    uint32_t subnormal_scan = 0;  // --subnormal-scan N: D1's subnormal scan, N steps of two runs; no film
    uint32_t subnormal_locate = 0;  // --subnormal-locate N: what raises them, N steps of one run; no film
    fs::path from_save;           // --from-save: the scan's runs restore this LiveRun save, not start
    std::optional<spade::physics::airflow::GridShape> grid;  // --grid NXxNYxNZ: d1-hover's grid, at its 4 mm
    bool gpu = false;  // --gpu: a LiveRun film's fluid on GpuFluid, no parts (gpu_live.hpp)
    // --web S1..S6, S8: a website master (web_masters.hpp), at 3840x2160 into <out>/web.
    std::string web_id;
    uint32_t web_frames = 0;       // --web-frames N: a cache of N frames (the timing job's), not the master's
    std::string particle_look;       // --particle-look KEY=VALUE,...: the streaks' numbers (needs --particles)
    std::optional<std::pair<uint32_t, uint32_t>> clip_size;  // --clip-size WxH: a web clip at another size (a diagnostic)
    std::string build_parts;         // --build-parts FILE: the real-parts drone's table (build_parts.hpp), drawn at the held body's pose over the flow
    std::string prop_blur;           // --prop-blur alpha=..,samples=..,shutter=..: the rotors' blade blur (build_parts.hpp)
    std::string drone_light;         // --drone-light key=..,ambient=..: the drone's key and fill light, times the standard (build_parts.hpp)
    std::string part_store;          // --part-store DIR: where the table's store objects are (<DIR>/<sha256>/<file>)
    std::string visual_mesh;         // --visual-mesh FILE: a glTF (metres, the model's frame) drawn at the held body's pose in place of the model's parts; the flow is unchanged
    std::string lines_prop;          // --lines FILE: the rotors of a D1 web clip as actuator lines, the blade from this prop part (under the assets)
    bool nasa_cloud = false;         // --nasa-cloud: --nasa's colours as a soft vorticity cloud, no surfaces
    bool surface_pressure = false;   // --surface-pressure: S8 shows the air's pressure on the drone's surfaces (O124)
    bool surface_shade = false;      // --surface-shade: ... lit for shape (the colour keeps its hue; needs --surface-pressure)
    std::string cache_dir;           // --cache-dir DIR: a --web-still redraw from the frame in DIR (another run's), never a solve
    bool nasa = false;               // --nasa: S2 and S8 draw Q-criterion surfaces coloured by vorticity (NASA Ames' multirotor look)
    std::string q_look;              // --q-look KEY=VALUE,...: the ladder's levels, opacities and colour range (needs --nasa)
    bool particles = false;          // --particles: S2 and S8 draw points carried through every stored frame (needs --stream)
    bool particles_preview = false;  // --particles-preview: S2 and S8 draw tracers carried through one frozen frame (a diagnostic)
    std::string drone;             // --drone NAME: a library drone for the D1 web clips (web::drones()); "" the reference quad
    std::optional<float> elevation;  // --elevation DEG: the camera's elevation for a web still, in place of the drone's (web::parse_elevation_deg)
    std::optional<float> cell_mm;  // --cell-mm X: the D1 box's cell (web::parse_cell_mm; only 2.5 with --drone x500)
    std::optional<uint32_t> step_us;  // --step-us N: the solver step in microseconds (web::parse_step_us), for a --window run
    std::optional<float> sgs_cs;   // --sgs-cs X: a D1 web clip's Smagorinsky Cs, in place of kD1WebSgsCs (0: off, a diagnostic; a --lines run refuses 0: its guard needs sgs_cs above 0)
    Look look;                     // --l2-level, --l2-smooth, --min-cells, --mean-frames (look.hpp)
    bool stream = false;           // --stream: draw each frame as the solve writes it, delete what the clip no longer needs
    bool hover_start = false;      // --hover-start: the rotors start at their commanded (hover) speed, not at rest; needs --lines (L106c)
    bool volume = false;           // --volume: the speed volume under the particle views (L106h; needs --particles or --particles-preview)
    std::vector<uint32_t> keep_frames;  // --keep-frames A,B: frames a streamed run keeps on disk (the still pairs')
    std::optional<std::pair<uint32_t, uint32_t>> still_size;  // --still-size WxH: a web still at another size (a diagnostic)
    double window_from = 0.0, window_to = 0.0;  // --window A:B: the simulated window, in place of the clip's group's
    bool web_still_only = false;   // --web-still: the 5120x2880 PNG only (any clip)
    fs::path web_deliver;          // --web-deliver RAW: check and copy the filmed files to RAW/spade, and MANIFEST.md
    std::string ticket;            // --ticket: the jobs service's ticket the caption names
    std::string command_line;      // the arguments as given, for the caption
};

constexpr int kExitIncomplete = 3;  // stopped by a budget; run again to continue

// --gpu's frames (gpu_live.hpp), which never stop at a budget.
[[nodiscard]] spade::Result<bool> simulate_gpu_film(const Film& film, const fs::path& cache, const CacheMark& mark) {
#ifdef SPADE_FLOW_MOVIE_GPU
    if (auto done = simulate_gpu(*film.run, film.frames, film.stem, cache, mark); !done) {
        return std::unexpected(done.error());
    }
    return true;
#else
    (void)cache;
    (void)mark;
    return std::unexpected(spade::Error{spade::Code::unavailable, film.stem + ": --gpu needs the Vulkan build"});
#endif
}

// --export-lines NAME: a cached frame of a LiveRun film (the last, or --export-frame N) as streamlines in JSON for
// the website's 3-D viewer (line_export.hpp): <out>/export/NAME.json and NAME.md. With --export-deliver RAW, both
// also go to RAW/spade and one line to RAW/MANIFEST.md, through the guard a clip's delivery passes: nothing else
// under the site is written.
[[nodiscard]] spade::Result<void> export_lines_report(const Film& film, const fs::path& cache, const Options& opt) {
    using spade::Code;
    using spade::Error;
    if (!film.run) return std::unexpected(Error{Code::invalid_argument, "--export-lines reads a LiveRun film's frames"});
    const LiveCase& lc = *film.run;
    const uint32_t frames = cached_frames(cache);
    if (frames == 0u) {
        return std::unexpected(Error{Code::not_found, "no cached frames in " + cache.string() + ": film the case first"});
    }
    const uint32_t k = opt.export_frame >= 0 ? static_cast<uint32_t>(opt.export_frame) : frames - 1u;
    if (k >= frames) {
        return std::unexpected(Error{Code::invalid_argument, "--export-frame " + std::to_string(k) + ": the cache holds " +
                                                                 std::to_string(frames) + " frames"});
    }
    spade::FlowVolume volume;
    FrameLog log;
    if (auto read = read_frame(cache, k, volume, log); !read) return read;
    if (log.live.size() < 7u) return std::unexpected(Error{Code::internal, "a LiveRun frame's log holds no held body"});
    ExportBody body;
    body.pos = glm::vec3(static_cast<float>(log.live[0]), static_cast<float>(log.live[1]), static_cast<float>(log.live[2]));
    body.orient = glm::quat(static_cast<float>(log.live[3]), static_cast<float>(log.live[4]),
                            static_cast<float>(log.live[5]), static_cast<float>(log.live[6]));
    std::vector<ExportRotor> rotors;
    for (const spade::vehicles::RotorDesc& r : lc.run_model.rotors) {
        rotors.push_back(ExportRotor{r.local_pos, glm::normalize(r.local_orient * glm::vec3(0.0f, 1.0f, 0.0f)), r.radius,
                                     r.spin_dir});
    }
    const CacheMark mark = read_mark(cache);
    const float wind = lc.c.flight.speed, heading = lc.c.flight.heading_deg;
    ExportInfo info;
    info.name = opt.export_name;
    info.sim = mark.build.empty() ? build_identity() : mark.build;
    info.config = lc.name + "; wind " + fixed(wind, 1) + " m/s from heading " + fixed(heading, 0) + " deg; " + lc.identity;
    info.note = "one frame of the reference quad's run, in the aircraft's body frame; a stand-in, see the caption";
    info.frame = k;
    info.step = log.step;
    info.t_s = log.time;
    auto made = export_streamlines(volume, body, rotors, info);
    if (!made) return std::unexpected(made.error());
    const fs::path dir = opt.out / "export";
    std::error_code ec;
    fs::create_directories(dir, ec);
    const fs::path json = dir / (opt.export_name + ".json"), caption = dir / (opt.export_name + ".md");
    ExportCaption cap;
    cap.info = &info;
    cap.result = &*made;
    cap.commit = build_identity();
    cap.date_utc = utc_now();
    cap.command = opt.command_line;
    cap.ticket = opt.ticket.empty() ? "none" : opt.ticket;
    cap.wind_mps = wind;
    cap.crosswind = wind > 0.0f && std::fabs(heading) > 1.0f;
    cap.cell_mm = volume.header.cell * 1000.0f;
    cap.device = film.device;
    cap.sgs_cs = film.run ? film.run->sgs_cs : 0.0f;
    {
        std::ofstream out(json, std::ios::binary | std::ios::trunc);
        out << made->json;
        std::ofstream md(caption, std::ios::binary | std::ios::trunc);
        md << export_caption_text(cap);
        if (!out || !md) return std::unexpected(Error{Code::io_error, "writing " + json.string() + " or " + caption.string()});
    }
    std::printf("[ export ] %s: frame %u, %u lines, %llu points, %zu bytes, least arc %.1f mm, %u dropped: %s\n",
                opt.export_name.c_str(), k, made->lines, static_cast<unsigned long long>(made->points), made->json.size(),
                static_cast<double>(made->spacing_m) * 1000.0, made->dropped, json.string().c_str());
    if (opt.export_deliver.empty()) return {};

    const web::DeliveryGuard guard{opt.export_deliver};
    const fs::path spade_dir = opt.export_deliver / "spade";
    const fs::path manifest = opt.export_deliver / "MANIFEST.md";
    fs::create_directories(spade_dir, ec);
    if (ec) return std::unexpected(Error{Code::io_error, "cannot create " + spade_dir.string() + ": " + ec.message()});
    for (const fs::path& from : {json, caption}) {
        const fs::path to = spade_dir / from.filename();
        if (!guard.may_write(to)) return std::unexpected(Error{Code::invalid_argument, to.string() + " is outside " + spade_dir.string()});
        fs::copy_file(from, to, fs::copy_options::overwrite_existing, ec);
        if (ec) return std::unexpected(Error{Code::io_error, "copying " + from.string() + ": " + ec.message()});
        std::printf("flow_movie: delivered %s\n", to.string().c_str());
    }
    if (!guard.may_write(manifest)) return std::unexpected(Error{Code::invalid_argument, manifest.string() + " is not the manifest"});
    std::string have;
    {
        std::ifstream in(manifest, std::ios::binary);
        have.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const std::string file = opt.export_name + ".json";
    if (have.find("- " + file + " |") != std::string::npos) {
        std::printf("flow_movie: MANIFEST already lists %s\n", file.c_str());
        return {};
    }
    std::ofstream out(manifest, std::ios::binary | std::ios::app);
    if (!have.empty() && have.back() != '\n') out << '\n';
    out << data_manifest_line(file, made->json.size(), "sim", "Streamlines around the reference quad's rotors in its body frame (JSON)")
        << '\n';
    if (!out) return std::unexpected(Error{Code::io_error, "writing " + manifest.string()});
    std::printf("flow_movie: MANIFEST.md: 1 line added\n");
    return {};
}

// d1-hover's length: D1-3's window ends 0.50 s after arming (armed at 0), and
// the lead's floor is 0.55 s; 18 frames.
constexpr double kD1Seconds = 0.6;

[[nodiscard]] fs::path default_out() {
    const char* profile = std::getenv("USERPROFILE");
    if (profile == nullptr) profile = std::getenv("HOME");
    const std::time_t now = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char date[16];
    std::strftime(date, sizeof date, "%Y-%m-%d", &local);
    return web::default_out_dir(profile != nullptr ? profile : "", date);
}

// The run's labels by step, from the cache's run.txt (each line "step<TAB>label"), for the captions.
[[nodiscard]] spade::Result<void> load_run_labels(const Film& film, const fs::path& cache) {
    if (!film.run_labels) return {};
    film.run_labels->clear();
    std::ifstream lines(cache / "run.txt", std::ios::binary);
    for (std::string line; std::getline(lines, line);) {
        const std::size_t tab = line.find('\t');
        uint64_t step = 0;
        const char* end = line.data() + (tab == std::string::npos ? 0u : tab);
        if (tab == std::string::npos || std::from_chars(line.data(), end, step).ptr != end) {
            return std::unexpected(spade::Error{spade::Code::io_error, (cache / "run.txt").string() +
                                                                           ": a line is not a step and a label"});
        }
        (*film.run_labels)[step] = line.substr(tab + 1u);
    }
    return {};
}

// Simulates (unless cached), then renders its clips, or stills. nullopt: the
// simulation stopped at its budget, to be continued by running again.
[[nodiscard]] spade::Result<std::optional<std::vector<fs::path>>> run_film(const Film& film, const Options& opt,
                                                                           GlStage& stage, const Encoder& encoder) {
    const fs::path cache = opt.out / "cache" / film.stem;
    const CacheMark mine{film_identity(film), build_identity()};
    std::error_code ec;
    if (opt.resimulate) fs::remove_all(cache, ec);
    const bool started = fs::exists(cache, ec) && !fs::is_empty(cache, ec);
    const bool finished = started && cached_frames(cache) == film.frames && !fs::exists(cache / "state.bin", ec) &&
                          !fs::exists(cache / "live.save", ec);
    const CacheMark theirs = read_mark(cache);
    const auto refuse_cache = [&](const std::string& what, const std::string& found, const std::string& wanted) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument,
                                            "the cache " + cache.string() + " holds another " + what + " (\"" +
                                                found + "\"); this run is \"" + wanted +
                                                "\". --resimulate starts it again"});
    };
    if (started && theirs.film != mine.film) return refuse_cache("film", theirs.film, mine.film);
    if (opt.arm_pressure) {
        if (auto measured = arm_pressure_report(film, cache); !measured) return std::unexpected(measured.error());
        return std::optional<std::vector<fs::path>>{std::vector<fs::path>{}};
    }
    if (!opt.export_name.empty()) {
        if (auto exported = export_lines_report(film, cache, opt); !exported) return std::unexpected(exported.error());
        return std::optional<std::vector<fs::path>>{std::vector<fs::path>{}};
    }
    if (opt.probe_swirl >= 0) {
        if (auto probed = probe_swirl(film, cache, static_cast<uint32_t>(opt.probe_swirl)); !probed) {
            return std::unexpected(probed.error());
        }
        return std::optional<std::vector<fs::path>>{std::vector<fs::path>{}};
    }
    if (opt.still_frame >= 0 && started && cached_frames(cache) > static_cast<uint32_t>(opt.still_frame)) {
        if (auto drew = render_stills(film, cache, stage, encoder, opt.out / "stills", 1u, opt.cpu, opt.view,
                                      static_cast<uint32_t>(opt.still_frame));
            !drew) {
            return std::unexpected(drew.error());
        }
        return std::optional<std::vector<fs::path>>{std::vector<fs::path>{}};
    }
    if (film.run && started && !finished && !fs::exists(cache / "live.save", ec)) {
        // Stopped before its first save (a --gpu film keeps none): nothing to
        // continue from.
        std::printf("[ %s ] an unfinished %s cache with no save; starting it again\n", film.stem.c_str(),
                    film.gpu ? "GPU film's" : "LiveRun");
        fs::remove_all(cache, ec);
    } else if (started && !finished && theirs.build != mine.build) {
        return refuse_cache("build's unfinished simulation", theirs.build, mine.build);
    }
    if (!finished) {
        auto done = film.gpu   ? simulate_gpu_film(film, cache, mine)
                    : film.run ? simulate_live(film, cache, mine, opt.budget_seconds, opt.frame_budget)
                               : simulate(film, cache, mine, opt.budget_seconds, opt.frame_budget);
        if (!done) return std::unexpected(done.error());
        if (!*done) return std::optional<std::vector<fs::path>>{};
    } else {
        std::printf("[ %s ] using the %u cached frames in %s\n", film.stem.c_str(), film.frames, cache.string().c_str());
    }
    if (auto labelled = load_run_labels(film, cache); !labelled) return std::unexpected(labelled.error());
    if (opt.simulate_only) return std::optional<std::vector<fs::path>>{std::vector<fs::path>{}};
    if (opt.stills > 0u) {
        if (auto drew = render_stills(film, cache, stage, encoder, opt.out / "stills", opt.stills, opt.cpu, opt.view); !drew) {
            return std::unexpected(drew.error());
        }
        return std::optional<std::vector<fs::path>>{std::vector<fs::path>{}};
    }
    auto clips = render_film(film, cache, stage, encoder, opt.out / "clips");
    if (!clips) return std::unexpected(clips.error());
    // A LiveRun film's cache stays unless --drop-cache: it may hold hours of
    // run (see the header). Any other film's goes unless --keep-cache.
    if (film.run && !opt.drop_cache) {
        std::printf("[ %s ] kept the cache in %s (--drop-cache removes it)\n", film.stem.c_str(),
                    cache.string().c_str());
    } else if (film.run || !opt.keep_cache) {
        fs::remove_all(cache, ec);
    }
    return std::optional<std::vector<fs::path>>{std::move(*clips)};
}

[[nodiscard]] uint32_t frames_for(const Options& opt, double seconds, const Encoder& e) {
    const double s = opt.seconds > 0.0 ? opt.seconds : seconds;
    return std::max(2u, static_cast<uint32_t>(std::lround(s * e.fps)));
}

[[nodiscard]] int fail(const spade::Error& e) {
    std::fprintf(stderr, "flow_movie: %s\n", e.context.c_str());
    return e.code == spade::Code::unavailable ? kExitUnavailable : 1;
}

// Delivers a filmed master: checks the files in <out>/web (the clip's size, frame count, codec, pixel format and no
// audio, by ffprobe; the still's size; the caption beside them), copies them to <raw>/spade and appends one MANIFEST
// line per file to <raw>/MANIFEST.md. Every destination passes the guard: nothing is written under the site but
// <raw>/spade and <raw>/MANIFEST.md (the brief's "write nothing else").
[[nodiscard]] int web_deliver(const Options& opt, const web::Clip& clip, uint32_t n, const fs::path& work,
                              const std::string& base) {
    const auto stop = [](const std::string& why) {
        std::fprintf(stderr, "flow_movie: delivery refused: %s\n", why.c_str());
        return 1;
    };
    if (opt.web_frames > 0u) return stop("a clip filmed from a --web-frames cache is a timing run, not a master");
    if (opt.sgs_cs && *opt.sgs_cs != web::kD1WebSgsCs) {
        return stop("a clip filmed at another Smagorinsky Cs (--sgs-cs) is a diagnostic, not a master");
    }
    if (opt.elevation) return stop("a camera elevation (--elevation) is for stills, not a master");
    if (opt.particles_preview) {
        return stop("a particle preview (--particles-preview) is a diagnostic, not a master");
    }
    const web::DeliveryGuard guard{opt.web_deliver};
    const fs::path spade_dir = opt.web_deliver / "spade";
    const fs::path manifest = opt.web_deliver / "MANIFEST.md";
    struct Item {
        std::string name;
        uint32_t width, height;
        double seconds;
        std::string description;
    };
    std::vector<Item> items;
    {
        const fs::path mp4 = work / (base + "mp4");
        auto info = probe_video(opt.ffmpeg, mp4);
        if (!info) return stop(info.error().context);
        if (info->width != web::kWidth || info->height != web::kHeight || info->frames != clip.frames() ||
            info->codec != "h264" || info->pix_fmt != "yuv420p" || info->has_audio) {
            return stop(mp4.string() + " is " + std::to_string(info->width) + "x" + std::to_string(info->height) + ", " +
                        std::to_string(info->frames) + " frames, " + info->codec + ", " + info->pix_fmt +
                        (info->has_audio ? ", with audio" : "") + "; wanted 3840x2160, " + std::to_string(n) +
                        " frames, h264, yuv420p, no audio");
        }
        items.push_back({base + "mp4", info->width, info->height, static_cast<double>(info->frames) / web::kFps,
                         web::clip_title(clip, *web::find_drone(opt.drone))});
    }
    if (clip.still) {
        const fs::path png = work / (base + "png");
        auto info = probe_video(opt.ffmpeg, png);
        if (!info) return stop(info.error().context);
        if (info->width != web::kStillWidth || info->height != web::kStillHeight) {
            return stop(png.string() + " is " + std::to_string(info->width) + "x" + std::to_string(info->height) +
                        ", not 5120x2880");
        }
        items.push_back({base + "png", info->width, info->height, 0.0, web::clip_title(clip, *web::find_drone(opt.drone)) + " (still)"});
    }
    const fs::path caption = work / (base + "md");
    if (!fs::exists(caption)) return stop(caption.string() + " does not exist");

    std::vector<fs::path> copies;
    for (const Item& item : items) copies.push_back(work / item.name);
    copies.push_back(caption);
    std::error_code ec;
    fs::create_directories(spade_dir, ec);
    if (ec) return stop("cannot create " + spade_dir.string() + ": " + ec.message());
    for (const fs::path& from : copies) {
        const fs::path to = spade_dir / from.filename();
        if (!guard.may_write(to)) return stop(to.string() + " is outside " + spade_dir.string());
        fs::copy_file(from, to, fs::copy_options::overwrite_existing, ec);
        if (ec) return stop("copying " + from.string() + ": " + ec.message());
        std::printf("flow_movie: delivered %s\n", to.string().c_str());
    }
    if (!guard.may_write(manifest)) return stop(manifest.string() + " is not the manifest");
    std::string have;
    {
        std::ifstream in(manifest, std::ios::binary);
        have.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    std::string add;
    for (const Item& item : items) {
        if (have.find("- " + item.name + " |") != std::string::npos) {
            std::printf("flow_movie: MANIFEST already lists %s\n", item.name.c_str());
            continue;
        }
        add += web::manifest_line(item.name, item.width, item.height, item.seconds, "sim", item.description) + "\n";
    }
    if (!add.empty()) {
        std::ofstream out(manifest, std::ios::binary | std::ios::app);
        if (!have.empty() && have.back() != '\n') out << '\n';
        out << add;
        if (!out) return stop("writing " + manifest.string());
        std::printf("flow_movie: MANIFEST.md: %zu line(s) added\n", items.size());
    }
    return 0;
}

// --- filming and delivering a website master -----------------------------------------------------------

// The simulated time a clip of `n` frames shows and what it is: the frames' own times, never an index over 30. Read from
// the first and the last frame the clip draws (their logs), so a streamed run, which has deleted them, needs no cache.
struct WebTimes {
    double per_frame = 0.0;  // simulated time between frames, in the case's units
    uint64_t last_step = 0;
};

[[nodiscard]] std::string grid_line(const Film& film) {
    if (film.run) return film.run_labels && !film.run_labels->empty() ? "Run: " + film.run_labels->rbegin()->second : "";
    const AirflowCase& c = film.airflow;
    return "Run: " + std::to_string(c.shape.nx) + "x" + std::to_string(c.shape.ny) + "x" + std::to_string(c.shape.nz) +
           " cells at " + fixed(c.shape.dx * 1000.0, 2) + " mm, step " + fixed(c.fluid.h * 1000.0, 3) + " ms, n_v " +
           std::to_string(c.fluid.n_v) + ".";
}

// Does the cache folder hold any frame file, in any numbering: a kept frame of an earlier streamed run is one.
[[nodiscard]] bool has_frame_files(const fs::path& dir) {
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.rfind("frame-", 0) == 0 && it->path().extension() == ".bin") return true;
    }
    return false;
}

// The bytes of the files under `dir` (the cache's size, for the ledger's note).
[[nodiscard]] uint64_t dir_bytes(const fs::path& dir) {
    uint64_t total = 0;
    std::error_code ec;
    for (fs::recursive_directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code size_ec;
        if (it->is_regular_file(size_ec)) {
            const auto bytes = it->file_size(size_ec);
            if (!size_ec) total += bytes;
        }
    }
    return total;
}

// One website clip being filmed, a frame at a time: the stage, the encoder's pipe, the frame's scratch and what the caption
// needs. The ordinary run (the cache first, then the clip) and the streamed run (a frame as the solve writes it) draw through
// this one object, so a streamed frame is the very draw of a cached one.
struct WebFilming {
    const Options& opt;
    const web::Clip& clip;
    const Film& film;
    fs::path cache;
    uint32_t first, n, ss;
    GlStage& stage;
    const Encoder& encoder;
    fs::path work;
    std::string base;

    spade::FlowVolume volume;
    FrameLog log;
    std::vector<uint8_t> rgba;
    std::string note;
    double draw_s = 0.0;
    LookState look_state;
    std::unique_ptr<WebClip> webclip;
    std::unique_ptr<GlStage> large;
    double draw_total = 0.0, busy_s = 0.0;  // the draws' own time; and each frame's, draw and pipe together
    std::vector<web::FrameTimes> perf;      // each frame's phases: the ledger's figures are their medians
    double first_time = 0.0, last_time = 0.0;  // the simulated time of the clip's first and last frame
    uint64_t last_step = 0;
    uint32_t drawn = 0;

    WebFilming(const Options& o, const web::Clip& c, const Film& f, fs::path cache_dir, uint32_t first_frame, uint32_t frames,
               uint32_t supersample, GlStage& st, const Encoder& enc, fs::path work_dir, std::string base_name)
        : opt(o), clip(c), film(f), cache(std::move(cache_dir)), first(first_frame), n(frames), ss(supersample), stage(st),
          encoder(enc), work(std::move(work_dir)), base(std::move(base_name)), look_state{o.look, {}} {}

    // Opens the clip's pipe (and the large stage when supersampling).
    [[nodiscard]] spade::Result<void> begin() {
        auto opened = WebClip::open(opt.ffmpeg, encoder.width, encoder.height, work / (base + "mp4"));
        if (!opened) return std::unexpected(opened.error());
        webclip = std::move(*opened);
        if (ss > 1u) {
            auto made = GlStage::create(encoder.width * ss, encoder.height * ss);
            if (!made) return std::unexpected(made.error());
            large = std::move(*made);
        }
        perf.reserve(n);
        return {};
    }

    // The clip's frame k (0-based within the clip), drawn from the cache's frame first + k and added to the clip.
    [[nodiscard]] spade::Result<void> clip_frame(uint32_t k) {
        using Clock = std::chrono::steady_clock;
        GlStage& frame_stage = large ? *large : stage;
        web::FrameTimes ft;
        const auto frame_start = Clock::now();
        if (auto drew = draw_web_frame(film, cache, first, k, n, frame_stage, ss, volume, log, rgba, note, draw_s, &ft,
                                       &look_state);
            !drew) {
            return drew;
        }
        draw_total += draw_s;
        if (clip.id != "S5" && (k == 0u || k + 1u == n) && !corners_black(rgba, encoder.width, encoder.height)) {
            return std::unexpected(spade::Error{spade::Code::internal,
                                                "web " + opt.web_id + " frame " + std::to_string(k) +
                                                    ": a corner is not pure black (0, 0, 0), or the subject runs to the "
                                                    "frame's edge"});
        }
        const auto pipe_start = Clock::now();
        if (auto added = webclip->add_frame(rgba); !added) return added;
        const auto frame_end = Clock::now();
        ft.pipe = std::chrono::duration<double>(frame_end - pipe_start).count();
        ft.frame = std::chrono::duration<double>(frame_end - frame_start).count();
        perf.push_back(ft);
        busy_s += ft.frame;
        if (k == 0u) first_time = log.time;
        last_time = log.time;
        last_step = log.step;
        ++drawn;
        if (k % 30u == 0u || k + 1u == n) {
            std::printf("[ %s ] frame %u/%u, t %s, %.2f s a frame\n", opt.web_id.c_str(), k + 1u, n, fixed(log.time, 4).c_str(),
                        busy_s / static_cast<double>(k + 1u));
            std::fflush(stdout);
        }
        return {};
    }

    // Closes the clip: the timing lines, the caption beside it. `cache_note`: the line saying where the frames came from.
    [[nodiscard]] spade::Result<void> finish_clip(const std::string& cache_note) {
        const auto finish_start = std::chrono::steady_clock::now();
        auto finished = webclip->finish();
        if (!finished) return std::unexpected(finished.error());
        const double through = busy_s;
        const double with_encode =
            through + std::chrono::duration<double>(std::chrono::steady_clock::now() - finish_start).count();
        std::printf("web-timing: %s %ux%u, %u frames: draw %.3f s a frame (GL, read back), draw+pipe %.3f s a frame, "
                    "through the encode %.3f s a frame, %.0f s in all\n",
                    opt.web_id.c_str(), encoder.width, encoder.height, n, draw_total / n, through / n, with_encode / n,
                    with_encode);
        // The ledger's figures (Test/Docs' PERF.md): the median frame and each phase, and which cache they were drawn from.
        for (const std::string& line : web::web_perf_lines(perf, with_encode - through)) std::printf("%s\n", line.c_str());
        std::printf("%s\n", cache_note.c_str());
        if (opt.look.mean_frames > 1u) {
            std::printf("look-ring: the mean flow held at most %zu raw frames, %.2f GB, beside the frame being drawn\n",
                        look_state.peak_ring_frames, static_cast<double>(look_state.peak_ring_bytes) / 1e9);
        }
        const WebTimes times{n > 1u ? (last_time - first_time) / static_cast<double>(n - 1u) : 0.0, last_step};
        web::CaptionInput in;
        in.clip = &clip;
        in.drone = web::find_drone(opt.drone);
        in.cell_mm = opt.cell_mm.value_or(4.0f);
        in.lines = !opt.lines_prop.empty();
        in.particles = opt.particles || opt.particles_preview;
        in.volume = opt.volume;
        in.cloud = opt.nasa_cloud;
        in.pressure = opt.surface_pressure;
        in.shaded = opt.surface_shade;
        if (const auto blades_blur = parse_prop_blur(opt.prop_blur)) in.blades = blades_blur->blades;
        in.step_us = opt.step_us.value_or(200u);
        in.commit = build_identity();
        in.date_utc = utc_now();
        in.command = opt.command_line;
        in.ticket = opt.ticket.empty() ? "(run by hand, not queued)" : opt.ticket;
        in.screen_seconds = static_cast<double>(n) / static_cast<double>(web::kFps);
        in.sim_seconds = times.per_frame * static_cast<double>(n);
        in.extra = grid_line(film);
        if (opt.hover_start) in.extra += (in.extra.empty() ? "" : " ") + std::string("The rotors start at their hover speed (no spool-up).");
        if (film.run && !film.run->domain.empty()) {
            in.extra += (in.extra.empty() ? "" : " ") + std::string("Resolved ") + film.run->domain + ".";
        }
        if (const std::string declared = look_note(opt.look, 1000.0 * times.per_frame); !declared.empty()) {
            in.extra += (in.extra.empty() ? "" : " ") + std::string("Look: ") + declared + ".";
        }
        if (!opt.build_parts.empty()) {
            if (const auto blur = parse_prop_blur(opt.prop_blur)) in.extra += (in.extra.empty() ? "" : " ") + std::string("Look: ") + prop_blur_note(*blur) + ".";
            if (const auto light = parse_drone_light(opt.drone_light); light && light->on()) in.extra += " " + drone_light_note(*light) + ".";
        }
        if (!note.empty()) in.extra += (in.extra.empty() ? "" : " ") + std::string("Legend: ") + note;
        std::ofstream caption(work / (base + "md"), std::ios::binary | std::ios::trunc);
        caption << web::caption_text(in);
        if (!caption) return std::unexpected(spade::Error{spade::Code::io_error, "writing " + (work / (base + "md")).string()});
        std::printf("flow_movie: wrote %s and its caption\n", (work / (base + "mp4")).string().c_str());
        return {};
    }

    // The still of the clip's frame k, `w` x `h`, to `png`.
    [[nodiscard]] spade::Result<void> still(uint32_t k, uint32_t w, uint32_t h, const fs::path& png) {
        auto big = GlStage::create(w * ss, h * ss);
        if (!big) return std::unexpected(big.error());
        const auto start = std::chrono::steady_clock::now();
        if (auto drew = draw_web_frame(film, cache, first, k, n, **big, ss, volume, log, rgba, note, draw_s, nullptr,
                                       &look_state);
            !drew) {
            return drew;
        }
        Encoder still_encoder = encoder;
        still_encoder.width = w;
        still_encoder.height = h;
        if (clip.id != "S5" && !corners_black(rgba, w, h)) {
            return std::unexpected(spade::Error{spade::Code::internal,
                                                "web " + opt.web_id + " still: a corner is not pure black (0, 0, 0)"});
        }
        std::error_code made;
        fs::create_directories(png.parent_path(), made);  // a --web-still run has not made the folder the clip's mp4 would have
        if (auto saved = save_png(still_encoder, rgba, png); !saved) return saved;
        std::printf("web-timing: %s still %ux%u of frame %u: draw %.3f s, %.3f s with the PNG (%s)\n", opt.web_id.c_str(), w, h,
                    k, draw_s, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(),
                    (*big)->limits().c_str());
        std::printf("flow_movie: wrote %s\n", png.string().c_str());
        return {};
    }
};

// Films one website master: simulates (unless cached), draws its frames at 3840x2160 into <out>/web/<name>.mp4 and
// its caption beside it, and, for the four that have one (or --web-still), a 5120x2880 PNG. Nothing leaves <out>.
[[nodiscard]] int run_web(const Options& opt_in, GlStage& stage, const Encoder& encoder) {
    Options opt = opt_in;  // the drone's framing sets the wake's fade (the volume clips' look), declared in the caption
    const web::Clip* clip = web::find_clip(opt.web_id);
    if (clip == nullptr) {
        std::fprintf(stderr, "flow_movie: --web %s is not one of the web clips S1 to S6 and S8\n", opt.web_id.c_str());
        return 2;
    }
    const web::Group* group = web::find_group(clip->group);
    if (opt.sgs_cs && !web::sgs_cs_applies(*clip)) {
        std::fprintf(stderr, "flow_movie: --sgs-cs is for the D1 clips (S1 to S4 and S8); %s runs the reference quad's CPU "
                             "film, which has no subgrid term\n", opt.web_id.c_str());
        return 2;
    }
    if (const std::string why = look_refusal(clip->id, web::sgs_cs_applies(*clip), opt.look); !why.empty()) {
        std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
        return 2;
    }
    const web::Drone* drone = web::find_drone(opt.drone);
    if (drone == nullptr) {
        std::string names;
        for (const web::Drone& d : web::drones()) names += (names.empty() ? "" : ", ") + std::string(d.library);
        std::fprintf(stderr, "flow_movie: --drone %s is not a drone of the web clips (%s)\n", opt.drone.c_str(), names.c_str());
        return 2;
    }
    if (drone != &web::default_drone() && !web::sgs_cs_applies(*clip)) {
        std::fprintf(stderr, "flow_movie: --drone is for the D1 clips (S1 to S4 and S8); %s runs the reference quad's discs\n",
                     opt.web_id.c_str());
        return 2;
    }
    if (const std::string why = web::volume_with_particles_refusal(opt.volume, opt.particles, opt.particles_preview); !why.empty()) {
        std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
        return 2;
    }
    if (opt.hover_start && opt.lines_prop.empty()) {
        std::fprintf(stderr, "flow_movie: --hover-start is for a run with the rotors as lines (--lines): it starts them at the speed the lines' first controls command\n");
        return 2;
    }
    if (opt.cell_mm) {
        const std::string why = opt.grid ? std::string("--cell-mm and --grid both name the box: use one")
                                         : web::cell_mm_refusal(*opt.cell_mm, drone->library, clip->scene == web::Scene::d1_still);
        if (!why.empty()) {
            std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
            return 2;
        }
    }
    if (opt.step_us) {
        const std::string why = !web::sgs_cs_applies(*clip) ? std::string("--step-us is for the D1 clips (S1 to S4 and S8)")
                                                           : web::step_us_refusal(*opt.step_us, opt.window_to > 0.0);
        if (!why.empty()) {
            std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
            return 2;
        }
    }
    // S1's lines and the particle preview are traced in the true flow: no fade for them.
    WebPlan plan;
    plan.drone = drone;
    plan.cell_mm = opt.cell_mm;
    plan.hover_start = opt.hover_start;
    plan.volume = opt.volume;
    plan.step_us = opt.step_us;
    plan.particles_preview = opt.particles_preview && (clip->id == "S2" || clip->id == "S8");
    if (!opt.build_parts.empty() || !opt.part_store.empty()) {
        if (!web::sgs_cs_applies(*clip)) {
            std::fprintf(stderr, "flow_movie: --build-parts is for the D1 clips (S1 to S4 and S8)\n");
            return 2;
        }
        if (opt.build_parts.empty()) {
            std::fprintf(stderr, "flow_movie: --part-store needs --build-parts\n");
            return 2;
        }
        plan.build_parts = opt.build_parts;
        plan.part_store = opt.part_store;
        auto blur = parse_prop_blur(opt.prop_blur);
        if (!blur) return fail(blur.error());
        plan.prop_blur = *blur;
        auto light = parse_drone_light(opt.drone_light);
        if (!light) return fail(light.error());
        plan.drone_light = *light;
    }
    if (!opt.visual_mesh.empty()) {
        if (!web::sgs_cs_applies(*clip)) {
            std::fprintf(stderr, "flow_movie: --visual-mesh is for the D1 clips (S1 to S4 and S8)\n");
            return 2;
        }
        plan.visual_mesh = opt.visual_mesh;
    }
    if (!opt.lines_prop.empty()) {
        if (const std::string why = web::lines_refusal(web::sgs_cs_applies(*clip)); !why.empty()) {
            std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
            return 2;
        }
        plan.lines = opt.lines_prop;
    }
    if (opt.nasa) {
        if (const std::string why = web::nasa_refusal(clip->id == "S2" || clip->id == "S8", opt.window_to > 0.0, opt.web_still_only, opt.stream,
                                                      opt.particles, opt.particles_preview, opt.nasa_cloud);
            !why.empty()) {
            std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
            return 2;
        }
        auto q = parse_q_look(opt.q_look);
        if (!q) return fail(q.error());
        plan.nasa = *q;
        plan.cloud = opt.nasa_cloud;
    } else if (!opt.q_look.empty()) {
        std::fprintf(stderr, "flow_movie: --q-look is for --nasa\n");
        return 2;
    }
    if (opt.surface_pressure) {
        if (const std::string why = web::surface_pressure_refusal(clip->id == "S8", opt.window_to > 0.0, opt.web_still_only, opt.stream, opt.particles,
                                                                  opt.particles_preview, opt.nasa);
            !why.empty()) {
            std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
            return 2;
        }
        plan.pressure = true;
    }
    if (const std::string why = web::surface_shade_refusal(opt.surface_shade, opt.surface_pressure); !why.empty()) {
        std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
        return 2;
    }
    plan.shade = opt.surface_shade;
    if (opt.particles) {
        const char* why = !(clip->id == "S2" || clip->id == "S8") ? "--particles is for S2 and S8"
                          : !web::particles_window_refusal(opt.window_to > 0.0, opt.web_still_only, opt.stream).empty() ? "--particles is not for a --window run without --stream (a --window run that streams, or a --window --web-still redraw from a run's kept frames, is)"
                          : opt.particles_preview                 ? "--particles and --particles-preview are two looks: use one"
                                                                  : "";
        if (*why != '\0') {
            std::fprintf(stderr, "flow_movie: %s\n", why);
            return 2;
        }
        plan.particles = true;
    }
    plan.sgs_cs = opt.sgs_cs.value_or(web::kD1WebSgsCs);
    plan.elevation = opt.elevation;
    plan.frames = opt.web_frames > 0u ? opt.web_frames : group->frames;
    plan.from_s = opt.window_to > 0.0 ? opt.window_from : group->from_s;
    plan.steps_per_frame = opt.window_to > 0.0 ? 0u : group->steps_per_frame;
    plan.to_s = opt.window_to > 0.0
                    ? opt.window_to
                    : web::window_end_s(plan.from_s, plan.frames, plan.steps_per_frame, group->step_s);
    // L63: the still-air clips run in their own, deeper box; the quad's clips (S5, S6) have no D1 grid at all.
    plan.grid = clip->scene == web::Scene::quad ? opt.grid : std::optional(d1_web_grid(clip->scene == web::Scene::d1_still, opt.grid, opt.cell_mm));
    // The clip draws its frames from the group's cache, from `first` on (a --window run spreads its own frames).
    const uint32_t first = opt.window_to > 0.0 ? 0u : clip->first_frame;
    if (first >= plan.frames) {
        std::fprintf(stderr, "flow_movie: web %s starts at frame %u of its group's cache, which holds %u\n",
                     opt.web_id.c_str(), first, plan.frames);
        return 2;
    }
    const uint32_t n = std::min(clip->frames(), plan.frames - first);
    const uint32_t ss = opt.supersample > 0u ? std::min(opt.supersample, 4u) : 2u;  // rendered at ss x, filtered down
    const std::string base = web::web_file_name(clip->id, clip->short_name,
                                                web::clip_label(opt.clip_size ? opt.clip_size->first : 0u, opt.clip_size ? opt.clip_size->second : 0u), "");
    const fs::path work = opt.out / "web";
    std::printf("flow_movie: web %s \"%s\": %u frames of %u, window %g..%g, frame %ux%u (%s)\n", opt.web_id.c_str(),
                std::string(clip->title).c_str(), n, plan.frames, plan.from_s, plan.to_s, encoder.width, encoder.height,
                stage.limits().c_str());

    const uint32_t still_k = opt.still_frame >= 0 ? std::min(static_cast<uint32_t>(opt.still_frame), n - 1u) : n / 2u;
    // The run's look, made once and given to BOTH places that read it: opt.look (the film's look state) and plan.look (the film's setup).
    // S1's lines and the particle preview are traced in the true flow: no fade for them.
    opt.look = web::web_look(opt.look, *drone,
                             web::web_fade_applies(clip->id, opt.particles_preview, opt.particles, opt.nasa, opt.nasa_cloud, opt.surface_pressure),
                             plan.nasa ? &*plan.nasa : nullptr);
    plan.look = opt.look;
    auto film = web_film(*clip, *group, plan);
    if (!film) return fail(film.error());
    film->role_materials = drone != &web::default_drone();
    std::shared_ptr<ParticleDriver> particles;
    if (plan.particles) {
        ParticleOptions popt;
        popt.capacity = 250000u;
        popt.max_age_s = film->particle_hold->max_age_s;
        popt.per_frame = 120u;  // the region is the volume less a cell: left open
        auto look = parse_particle_look(opt.particle_look);
        if (!look) return fail(look.error());
        popt.substeps = look->substeps;  // how a frame is advanced rides in the same string (--particle-look substeps=N)
        particles = std::make_shared<ParticleDriver>(popt, std::vector<uint32_t>{first + still_k});
        film->particle_hold->driver = particles;
        film->particle_hold->first = first;
        film->particle_hold->look = *look;
        film->particle_hold->look.dim_centre = glm::vec3(0.0f, 0.004f, 0.0f);  // the held drone, at the still-air box's centre
    } else if (!opt.particle_look.empty()) {
        std::fprintf(stderr, "flow_movie: --particle-look is for --particles\n");
        return 2;
    }
    const fs::path cache = opt.cache_dir.empty() ? opt.out / "cache" / film->stem : fs::path(opt.cache_dir);
    if (const std::string why = web::cache_dir_refusal(opt.cache_dir, opt.web_still_only, opt.still_frame >= 0,
                                                       fs::exists(frame_file(cache, first + still_k)));
        !why.empty()) {
        std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
        return 2;
    }
    const auto snapshot_file = [&cache](uint32_t frame) {
        char name[48];
        std::snprintf(name, sizeof name, "particles-%05u.bin", frame);
        return cache / name;
    };
    if (plan.particles && !opt.stream) {
        // Not streamed, the points are not carried by a solve: a still is redrawn from kept frames. Either the particle snapshot an earlier streamed run wrote beside
        // the frame (--stream --keep-frames N --web-still), or, with none, frames 0 to the still's all kept in the cache folder (another run's, a harness that keeps every
        // frame): the points are carried through them in order here, no solve (L106b).
        const uint32_t frame = first + still_k;
        const bool redraw = opt.web_still_only && opt.still_frame >= 0 && fs::exists(frame_file(cache, frame));
        if (redraw && fs::exists(snapshot_file(frame))) {
            auto field = read_particle_field(snapshot_file(frame));
            if (!field) return fail(field.error());
            particles->add_snapshot(frame, std::move(*field));
        } else if (redraw && cached_frames(cache) > frame) {
            FrameLog pool_log;
            const auto read = [&](uint32_t k, spade::FlowVolume& volume, double& time_s) -> spade::Result<void> {
                if (auto got = read_frame(cache, k, volume, pool_log); !got) return got;
                time_s = pool_log.time;
                return {};
            };
            const auto pool_of = [&]() { return film->particle_pool(pool_log); };
            const auto carry_start = std::chrono::steady_clock::now();
            if (auto carried = advance_through_frames(*particles, frame, read, pool_of); !carried) return fail(carried.error());
            std::printf("flow_movie: web %s: the points carried through frames 0 to %u of %s in %.1f s (no solve)\n", opt.web_id.c_str(), frame,
                        cache.generic_string().c_str(), std::chrono::duration<double>(std::chrono::steady_clock::now() - carry_start).count());
        } else {
            std::fprintf(stderr,
                         "flow_movie: --particles carries points through every frame in order, so it needs --stream, or --web-still "
                         "--still-frame N redrawing a still from frame %u's kept frame and either its particle snapshot or every kept frame from 0, in %s\n",
                         frame, cache.generic_string().c_str());
            return 2;
        }
    }
    {
        StreamAsk ask;
        ask.stream = opt.stream;
        ask.keep_given = !opt.keep_frames.empty();
        ask.d1 = web::sgs_cs_applies(*clip);
        ask.deliver = !opt.web_deliver.empty();
        ask.export_lines = !opt.export_name.empty();
        ask.simulate_only = opt.simulate_only;
        ask.resimulate = opt.resimulate;
        ask.frames_on_disk = has_frame_files(cache);
        if (const std::string why = stream_refusal(ask); !why.empty()) {
            std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
            return 2;
        }
    }
    if (!opt.web_deliver.empty()) return web_deliver(opt, *clip, n, work, base);
    if (!opt.export_name.empty()) {  // a cached frame's streamlines, not a clip
        auto exported = run_film(*film, opt, stage, encoder);
        if (!exported) return fail(exported.error());
        return 0;
    }

    // The frames: the simulation, from rest, over the window, then the clip's draw from the cache; or, with --stream, the
    // draw of each frame as the solve writes it, deleting what the clip no longer needs (stream_plan.hpp).
    const bool wants_clip = !opt.web_still_only;
    const bool wants_still = clip->still || opt.web_still_only;
    const auto [still_w, still_h] = opt.still_size.value_or(std::make_pair(web::kStillWidth, web::kStillHeight));
    // A diagnostic still (another Cs or size) is named for what it shows and never takes a master's file name.
    const bool diagnostic = opt.still_size.has_value() || plan.sgs_cs != web::kD1WebSgsCs;
    const fs::path still_png = diagnostic ? opt.out / "web-diag" /
                                                diag_still_name(opt.web_id, first + still_k, plan.sgs_cs, still_w, still_h,
                                                                opt.look)
                                          : work / (base + "png");
    WebFilming filming(opt, *clip, *film, cache, first, n, ss, stage, encoder, work, base);

    if (opt.stream) {
        std::vector<uint32_t> draws;
        if (wants_clip) {
            for (uint32_t k = 0; k < n; ++k) draws.push_back(first + k);
        }
        if (wants_still) draws.push_back(first + still_k);
        if (particles) draws = with_every_frame(draws);  // the points are carried through every frame, 0 to the last draw
        // The still is drawn AFTER the solve (a stage made mid-solve ends the clip's GL context: a GlStage's destructor calls
        // glfwTerminate), so its frame and its mean window stay on disk until then.
        const std::optional<uint32_t> still_frame =
            wants_still ? std::optional<uint32_t>(first + still_k) : std::nullopt;
        const StreamPlan stream_plan(draws, opt.look.mean_frames,
                                     keep_with_still(opt.keep_frames, still_frame, opt.look.mean_frames));
        if (stream_plan.last_frame() >= plan.frames) {
            std::fprintf(stderr, "flow_movie: frame %u is past the solve's last frame, %u (--web-frames %u)\n",
                         stream_plan.last_frame(), plan.frames - 1u, plan.frames);
            return 2;
        }
        std::error_code ec;
        if (opt.resimulate) fs::remove_all(cache, ec);
        // The frames on disk now and the most there have been: the measured peak of the cache folder.
        std::map<uint32_t, uint64_t> on_disk;
        uint64_t live_bytes = 0, peak_bytes = 0;
        std::size_t peak_frames = 0;
        spade::FlowVolume particle_volume;  // each frame, read once more for the particles
        FrameLog particle_log;
        auto hooks = std::make_shared<StreamHooks>();
        hooks->last_frame = stream_plan.last_frame();
        hooks->wanted = [&stream_plan](uint32_t k) { return stream_plan.needed(k); };
        hooks->after = [&](uint32_t k, const FrameLog&) -> spade::Result<void> {
            if (stream_plan.needed(k)) {
                std::error_code size_ec;
                const uintmax_t bytes = fs::file_size(frame_file(cache, k), size_ec);
                if (size_ec) {
                    return std::unexpected(spade::Error{spade::Code::io_error, "frame " + std::to_string(k) + " is not on disk: " +
                                                                                   size_ec.message()});
                }
                on_disk[k] = bytes;
                live_bytes += bytes;
                peak_bytes = std::max(peak_bytes, live_bytes);
                peak_frames = std::max(peak_frames, on_disk.size());
            }
            if (particles && stream_plan.needed(k)) {
                if (auto read = read_frame(cache, k, particle_volume, particle_log); !read) return read;
                if (!particles->has_pool()) particles->set_pool(film->particle_pool(particle_log));
                if (auto stepped = particles->advance(k, particle_volume, particle_log.time); !stepped) return stepped;
            }
            if (wants_clip && k >= first && k < first + n) {
                if (auto drew = filming.clip_frame(k - first); !drew) return drew;
            }
            for (auto it = on_disk.begin(); it != on_disk.end();) {
                if (stream_plan.spent(it->first, k)) {
                    std::error_code gone;
                    fs::remove(frame_file(cache, it->first), gone);
                    live_bytes -= it->second;
                    it = on_disk.erase(it);
                } else {
                    ++it;
                }
            }
            return {};
        };
        film->stream = hooks;
        if (wants_clip) {
            if (auto opened = filming.begin(); !opened) return fail(opened.error());
        }
        const auto sim_start = std::chrono::steady_clock::now();
        const CacheMark mine{film_identity(*film), build_identity()};
        auto simmed = simulate_live(*film, cache, mine, 0.0, 0u);
        if (!simmed) return fail(simmed.error());
        std::printf("flow_movie: web %s: simulation and draw %.0f s (streamed)\n", opt.web_id.c_str(),
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - sim_start).count());
        if (auto labelled = load_run_labels(*film, cache); !labelled) return fail(labelled.error());
        const std::string peak_note = "the cache folder held at most " + fixed(static_cast<double>(peak_bytes) / 1e9, 2) +
                                      " GB (" + std::to_string(peak_frames) + " frames at once)";
        if (wants_clip) {
            if (filming.drawn != n) {
                return fail(spade::Error{spade::Code::internal, "web " + opt.web_id + ": the solve ended after " +
                                                                    std::to_string(filming.drawn) + " of the clip's " +
                                                                    std::to_string(n) + " frames"});
            }
            if (auto finished = filming.finish_clip("stream: " + peak_note + " in " + cache.generic_string());
                !finished) {
                return fail(finished.error());
            }
        }
        // The still, from the frame the plan kept for it; then those frames go unless --keep-frames keeps them.
        if (particles && wants_still) {  // the field at the still's frame, beside the kept frame, so a still is redrawn in seconds
            if (const ParticleField* held = particles->snapshot_at(first + still_k)) {
                if (auto wrote = write_particle_field(snapshot_file(first + still_k), *held); !wrote) return fail(wrote.error());
                std::printf("flow_movie: web %s: the particle field at frame %u saved in %s\n", opt.web_id.c_str(), first + still_k,
                            snapshot_file(first + still_k).generic_string().c_str());
            }
        }
        if (wants_still) {
            if (auto drew = filming.still(still_k, still_w, still_h, still_png); !drew) return fail(drew.error());
            std::printf("look-ring: the mean flow held at most %zu raw frames, %.2f GB\n", filming.look_state.peak_ring_frames,
                        static_cast<double>(filming.look_state.peak_ring_bytes) / 1e9);
        }
        for (auto it = on_disk.begin(); it != on_disk.end();) {
            if (!std::binary_search(opt.keep_frames.begin(), opt.keep_frames.end(), it->first)) {
                std::error_code gone;
                fs::remove(frame_file(cache, it->first), gone);
                it = on_disk.erase(it);
            } else {
                ++it;
            }
        }
        std::printf("flow_movie: web %s stream: %s; %zu frame(s) kept in %s\n", opt.web_id.c_str(), peak_note.c_str(),
                    on_disk.size(), cache.generic_string().c_str());
        return 0;
    }

    // Not streamed: a still from a frame an earlier streamed run kept needs no simulation, and the cache is not whole.
    const bool kept_frame = opt.web_still_only && opt.still_frame >= 0 && fs::exists(frame_file(cache, first + still_k));
    if (kept_frame) {
        std::printf("flow_movie: web %s: the still is drawn from frame %u, kept in %s; no simulation\n", opt.web_id.c_str(),
                    first + still_k, cache.generic_string().c_str());
    } else {
        if (plan.grid) {  // the D1 clips: 253 MB a frame in the deep box
            const uint32_t cached = cached_frames(cache);
            fs::path on = opt.out;
            while (!on.empty() && !fs::exists(on) && on.has_parent_path() && on.parent_path() != on) on = on.parent_path();
            std::error_code space_ec;
            const fs::space_info space = fs::space(on.empty() ? fs::path(".") : on, space_ec);
            if (!space_ec) {
                const std::string why = disk_refusal(plan.frames > cached ? plan.frames - cached : 0u,
                                                     frame_bytes(plan.grid->nx, plan.grid->ny, plan.grid->nz), space.available);
                if (!why.empty()) {
                    std::fprintf(stderr, "flow_movie: web %s: %s\n", opt.web_id.c_str(), why.c_str());
                    return 2;
                }
            }
        }
        Options sim = opt;
        sim.simulate_only = true;
        sim.stills = 0;
        sim.still_frame = -1;
        sim.arm_pressure = false;
        sim.probe_swirl = -1;
        const auto sim_start = std::chrono::steady_clock::now();
        auto simmed = run_film(*film, sim, stage, encoder);
        if (!simmed) return fail(simmed.error());
        if (!*simmed) return kExitIncomplete;
        std::printf("flow_movie: web %s: simulation %.0f s\n", opt.web_id.c_str(),
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - sim_start).count());
        if (opt.simulate_only) return 0;  // the group's cache is all this run was for; a clip's own run draws from it
    }

    if (wants_clip) {
        if (auto opened = filming.begin(); !opened) return fail(opened.error());
        for (uint32_t k = 0; k < n; ++k) {
            if (auto drew = filming.clip_frame(k); !drew) return fail(drew.error());
        }
        const std::string cache_note = web::web_cache_note(cache.generic_string(), cached_frames(cache), dir_bytes(cache));
        if (auto finished = filming.finish_clip(cache_note); !finished) return fail(finished.error());
    }
    if (wants_still) {
        if (auto drew = filming.still(still_k, still_w, still_h, still_png); !drew) return fail(drew.error());
    }
    return 0;
}
// --synthetic-still NAME: the look of the web masters' streamlines on a SYNTHETIC wake, with no simulation (the
// lead, 2026-11-10: a test still before any sim). Four rotors' wakes written straight into a FlowVolume at the
// quad's grid (80 x 128 x 80 at 1 cm): a downwash jet under each disc, inflow above it, and a decaying swirl
// about each axis by spin. It is a picture of the look, never of a flow: nothing here is a result.
// --supersample N renders at N times the frame and box-filters it down, the lines' widths scaled to match.
[[nodiscard]] spade::FlowVolume synthetic_wake() {
    constexpr uint32_t nx = 80, ny = 128, nz = 80;
    constexpr float cell = 0.01f, radius = 0.0762f, hub_y = 0.024f, v_h = 5.3f;
    struct Rotor {
        float x, z, spin;
    };
    constexpr Rotor rotors[4] = {{0.11f, 0.0f, +1.0f}, {0.0f, 0.11f, -1.0f}, {-0.11f, 0.0f, +1.0f}, {0.0f, -0.11f, -1.0f}};
    spade::FlowVolume volume;
    spade::FlowVolumeHeader& h = volume.header;
    h.origin = glm::vec3(-0.40f, -0.96f, -0.40f);
    h.cell = cell;
    h.nx = nx;
    h.ny = ny;
    h.nz = nz;
    const std::size_t cells = std::size_t{nx} * ny * nz;
    volume.v.assign(cells * spade::kFlowHalvesPerCell, 0u);
    volume.s.assign(cells * spade::kFlowHalvesPerCell, 0u);
    std::array<bool, spade::kFlowChannelCount> seen{};
    for (uint32_t k = 0; k < nz; ++k) {
        for (uint32_t j = 0; j < ny; ++j) {
            for (uint32_t i = 0; i < nx; ++i) {
                const glm::vec3 p =
                    h.origin + (glm::vec3(static_cast<float>(i), static_cast<float>(j), static_cast<float>(k)) + glm::vec3(0.5f)) * cell;
                glm::vec3 u(0.0f);
                for (const Rotor& r : rotors) {
                    const float dx = p.x - r.x, dz = p.z - r.z;
                    const float rho = std::sqrt(dx * dx + dz * dz);
                    const float below = hub_y - p.y;  // m under the disc plane
                    if (below >= 0.0f) {
                        const float rw = radius * (1.0f + 0.35f * below);
                        const float t = rho / rw, t2 = t * t;
                        u.y -= (v_h + 5.0f * std::min(below / 0.3f, 1.0f)) * std::exp(-t2 * t2 * t2);
                        if (rho > 1.0e-6f) {
                            const float rc = 0.35f * radius * (1.0f + 2.0f * below);
                            const float swirl = 0.4f * (1.0f - std::exp(-rho * rho / (rc * rc))) / (6.2831853f * rho) *
                                                std::exp(-below / 0.7f);
                            u.x += -r.spin * swirl * dz / rho;
                            u.z += r.spin * swirl * dx / rho;
                        }
                    } else {
                        u.y -= v_h * std::exp(-rho * rho / (4.0f * radius * radius)) * std::exp(below / (1.2f * radius));
                    }
                }
                const std::size_t base = spade::flow_cell_index(h, i, j, k) * spade::kFlowHalvesPerCell;
                const float channel[3] = {u.x, u.y, u.z};
                for (uint32_t c = 0; c < 3; ++c) {
                    const spade::fp16::Half half = spade::fp16::from_f32(channel[c]);
                    volume.v[base + c] = half.bits;
                    const float x = spade::fp16::to_f32(half.bits);
                    h.channel_min[c] = seen[c] ? std::min(h.channel_min[c], x) : x;
                    h.channel_max[c] = seen[c] ? std::max(h.channel_max[c], x) : x;
                    seen[c] = true;
                }
            }
        }
    }
    volume.generation = spade::next_flow_generation();
    return volume;
}

// box_down (the supersample's filter down to the frame) is tools/flow_movie/box_filter.hpp's.

[[nodiscard]] int run_synthetic_still(const Options& opt, const Encoder& encoder) {
    const uint32_t s = std::max(1u, std::min(opt.supersample, 4u));
    const spade::FlowVolume volume = synthetic_wake();
    render::RenderScene scene = black_scene(volume);
    const auto speed_colours = render::colours_over(render::FlowQuantity::speed, render::ColourMapId::viridis, 0.0f, 12.0f);
    std::vector<glm::vec3> seeds;
    constexpr float radius = 0.0762f;
    const glm::vec3 hubs[4] = {{0.11f, 0.024f, 0.0f}, {0.0f, 0.024f, 0.11f}, {-0.11f, 0.024f, 0.0f}, {0.0f, 0.024f, -0.11f}};
    for (const glm::vec3& hub : hubs) {
        for (const float r : {0.35f, 0.75f}) {
            for (const glm::vec3& p :
                 render::seed_ring(hub + glm::vec3(0.0f, 0.06f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), r * radius, 12)) {
                seeds.push_back(p);
            }
        }
    }
    render::StreamlineOptions lines;
    lines.max_steps = 1200;
    if (auto ok = render::add_streamlines(scene, volume, seeds, lines, speed_colours); !ok) return fail(ok.error());
    scene.flow_line_style = render::FlowLineStyle{kWebLineStyle.width_px * static_cast<float>(s),
                                                  kWebLineStyle.glow_px * static_cast<float>(s), kWebLineStyle.glow_strength};
    const render::Camera camera =
        orbit(glm::vec3(0.0f, -0.30f, 0.0f), web::fit_distance(0.55f, kWebFov, kWebAspect, web::kMargin), 25.0f, 20.0f);
    auto stage = GlStage::create(encoder.width * s, encoder.height * s);
    if (!stage) return fail(stage.error());
    std::vector<uint8_t> rgba;
    const auto start = std::chrono::steady_clock::now();
    if (auto drew = (*stage)->draw(scene, camera, rgba); !drew) return fail(drew.error());
    const double draw_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (s > 1u) rgba = box_down(rgba, encoder.width, encoder.height, s);
    if (!corners_black(rgba, encoder.width, encoder.height)) {
        std::fprintf(stderr, "flow_movie: synthetic still: a corner is not pure black (0, 0, 0)\n");
        return 1;
    }
    const fs::path dir = opt.out / "synthetic";
    std::error_code ec;
    fs::create_directories(dir, ec);
    const fs::path png = dir / (opt.synthetic_still + "-x" + std::to_string(s) + ".png");
    if (auto saved = save_png(encoder, rgba, png); !saved) return fail(saved.error());
    std::printf("synthetic-timing: %s %ux%u rendered at %ux%u (%ux), draw %.3f s, %zu lines (%s)\n", opt.synthetic_still.c_str(),
                encoder.width, encoder.height, encoder.width * s, encoder.height * s, s, draw_s, scene.flow_polylines.size(),
                (*stage)->limits().c_str());
    std::printf("flow_movie: wrote %s (a synthetic wake, not a simulation)\n", png.string().c_str());
    return 0;
}

// The builds behind a film's frames: the one that simulated them (its cache's
// mark) and, when different, the one drawing them.
[[nodiscard]] std::string builds_line(const Film& film, const fs::path& out) {
    const CacheMark mark = read_mark(out / "cache" / film.stem);
    const std::string drawn = build_identity();
    if (mark.build.empty() || mark.build == drawn) return "this clip: " + drawn;
    return "this clip: simulated on " + mark.build + "; drawn on " + drawn;
}

// A film's card: its name, settings and record, on a still frame.
[[nodiscard]] spade::Result<fs::path> film_card(const Encoder& encoder, const fs::path& dir, const Film& film,
                                                double seconds, const std::string& builds) {
    const CaptionText extra = film.captions && false ? CaptionText{} : CaptionText{};
    std::vector<std::string> top{film.airflow.settings};
    if (film.stem == "quad-wake") {
        top.push_back("actuator discs: uniform thrust, constant-circulation swirl; no drone body yet (D1 adds the "
                      "solid drone)");
        top.push_back("inviscid: the shear layers break down to the grid's 1 cm scale");
    }
    (void)extra;
    std::vector<std::string> bottom = film.record;
    bottom.push_back(builds);
    return title_card(encoder, dir, film.stem + "-title", seconds, layout(film.airflow.name, top, bottom));
}

// The newest V1 record in `source` whose identity begins `identity`, as
// caption lines, every string verbatim from Interface's one reader (TD-9).
// A long run of measures (C0's, every 0.5) shows its first and last.
[[nodiscard]] spade::Result<std::vector<std::string>> record_lines(
    const std::vector<spade::analysis::ValidationRecord>& records, const std::string& source,
    const std::string& identity) {
    for (const spade::analysis::ValidationRecord& r : records) {
        if (r.source != source || r.identity.rfind(identity, 0) != 0) continue;
        std::vector<std::string> lines{"V1 record " + r.source + ": commit " + r.commit + ", " + r.date + ", " + r.path};
        std::string last_reference;
        const auto add = [&](const spade::analysis::ValidationMeasure& m) {
            std::string line = m.quantity + " = " + m.value + ": ";
            if (m.tolerance) line += "tolerance " + m.tolerance->bound + ", ";
            line += std::string(spade::analysis::verdict_name(m.verdict));
            lines.push_back(std::move(line));
            if (m.reference != last_reference) lines.push_back("reference: " + m.reference);
            last_reference = m.reference;
        };
        if (r.measures.size() > 3u) {
            add(r.measures.front());
            lines.push_back("(" + std::to_string(r.measures.size() - 2u) + " more measures between)");
            add(r.measures.back());
        } else {
            for (const auto& m : r.measures) add(m);
        }
        return lines;
    }
    return std::unexpected(spade::Error{spade::Code::not_found, "no V1 record \"" + identity + "\" in " + source});
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    for (int a = 1; a < argc; ++a) {
        const std::string arg = argv[a];
        opt.command_line += (a > 1 ? " " : "") + (arg.find(' ') == std::string::npos ? arg : "\"" + arg + "\"");
    }
    opt.command_line = "flow_movie " + opt.command_line;
    for (int a = 1; a < argc; ++a) {
        const std::string arg = argv[a];
        const auto value = [&]() -> std::string { return a + 1 < argc ? argv[++a] : std::string(); };
        if (arg == "--video") {
            opt.video = value();
        } else if (arg == "--out") {
            opt.out = value();
        } else if (arg == "--ffmpeg") {
            opt.ffmpeg = value();
        } else if (arg == "--seconds") {
            opt.seconds = std::atof(value().c_str());
        } else if (arg == "--stills") {
            opt.stills = static_cast<uint32_t>(std::atoi(value().c_str()));
        } else if (arg == "--resimulate") {
            opt.resimulate = true;
        } else if (arg == "--keep-cache") {
            opt.keep_cache = true;
        } else if (arg == "--drop-cache") {
            opt.drop_cache = true;
        } else if (arg == "--cpu") {
            opt.cpu = true;
        } else if (arg == "--view") {
            opt.view = value();
        } else if (arg == "--case") {
            opt.case_file = value();
            if (opt.video.empty()) opt.video = "case";
        } else if (arg == "--controls") {
            opt.controls_file = value();
        } else if (arg == "--still-frame") {
            opt.still_frame = std::atoi(value().c_str());
        } else if (arg == "--probe-swirl") {
            opt.probe_swirl = std::atoi(value().c_str());
        } else if (arg == "--nv") {
            opt.n_v = static_cast<uint32_t>(std::atoi(value().c_str()));
        } else if (arg == "--simulate-only") {
            opt.simulate_only = true;
        } else if (arg == "--budget-seconds") {
            opt.budget_seconds = std::atof(value().c_str());
        } else if (arg == "--frame-budget") {
            opt.frame_budget = static_cast<uint32_t>(std::atoi(value().c_str()));
        } else if (arg == "--arm-pressure") {
            opt.arm_pressure = true;
        } else if (arg == "--synthetic-still") {
            opt.synthetic_still = value();
            if (opt.video.empty()) opt.video = "synthetic-still";
        } else if (arg == "--supersample") {
            opt.supersample = static_cast<uint32_t>(std::atoi(value().c_str()));
        } else if (arg == "--export-lines") {
            opt.export_name = value();
        } else if (arg == "--export-frame") {
            opt.export_frame = std::atoi(value().c_str());
        } else if (arg == "--export-deliver") {
            opt.export_deliver = value();
        } else if (arg == "--step-time") {
            opt.step_time = static_cast<uint32_t>(std::atoi(value().c_str()));
        } else if (arg == "--subnormal-scan") {
            opt.subnormal_scan = static_cast<uint32_t>(std::atoi(value().c_str()));
        } else if (arg == "--subnormal-locate") {
            opt.subnormal_locate = static_cast<uint32_t>(std::atoi(value().c_str()));
        } else if (arg == "--from-save") {
            opt.from_save = value();
        } else if (arg == "--grid") {
            spade::physics::airflow::GridShape g = kD1Grid;
            if (std::sscanf(value().c_str(), "%ux%ux%u", &g.nx, &g.ny, &g.nz) != 3) {
                std::fprintf(stderr, "flow_movie: --grid takes NXxNYxNZ, as 224x160x168\n");
                return 2;
            }
            opt.grid = g;
        } else if (arg == "--gpu") {
            opt.gpu = true;
        } else if (arg == "--web") {
            opt.web_id = value();
            opt.video = "web";
        } else if (arg == "--web-frames") {
            opt.web_frames = static_cast<uint32_t>(std::atoi(value().c_str()));
        } else if (arg == "--elevation") {
            opt.elevation = web::parse_elevation_deg(value());
            if (!opt.elevation) {
                std::fprintf(stderr, "flow_movie: --elevation takes one number above 0 and at most 85 (degrees above the drone), as 55\n");
                return 2;
            }
        } else if (arg == "--sgs-cs") {
            opt.sgs_cs = web::parse_sgs_cs(value());
            if (!opt.sgs_cs) {
                std::fprintf(stderr, "flow_movie: --sgs-cs takes one number from 0 to 1 (0 turns the subgrid term off, a diagnostic; a --lines run refuses 0, as actuator lines need a subgrid model), as 0.17\n");
                return 2;
            }
        } else if (arg == "--cell-mm") {
            opt.cell_mm = web::parse_cell_mm(value());
            if (!opt.cell_mm) {
                std::fprintf(stderr, "flow_movie: --cell-mm takes one number above 0 and at most 10 (only 2.5 is supported, with --drone x500)\n");
                return 2;
            }
        } else if (arg == "--step-us") {
            opt.step_us = web::parse_step_us(value());
            if (!opt.step_us) {
                std::fprintf(stderr, "flow_movie: --step-us takes a whole number of microseconds from 1 to 1000, as 50\n");
                return 2;
            }
        } else if (arg == "--drone") {
            opt.drone = value();
        } else if (arg == "--particles-preview") {
            opt.particles_preview = true;
        } else if (arg == "--visual-mesh") {
            opt.visual_mesh = value();
        } else if (arg == "--build-parts") {
            opt.build_parts = value();
        } else if (arg == "--prop-blur") {
            opt.prop_blur = value();
        } else if (arg == "--drone-light") {
            opt.drone_light = value();
        } else if (arg == "--part-store") {
            opt.part_store = value();
        } else if (arg == "--lines") {
            opt.lines_prop = value();
        } else if (arg == "--nasa") {
            opt.nasa = true;
        } else if (arg == "--nasa-cloud") {
            opt.nasa = true;
            opt.nasa_cloud = true;
        } else if (arg == "--surface-pressure") {
            opt.surface_pressure = true;
        } else if (arg == "--surface-shade") {
            opt.surface_shade = true;
        } else if (arg == "--cache-dir") {
            opt.cache_dir = value();
        } else if (arg == "--q-look") {
            opt.q_look = value();
        } else if (arg == "--particles") {
            opt.particles = true;
        } else if (arg == "--particle-look") {
            opt.particle_look = value();
        } else if (arg == "--clip-size") {
            opt.clip_size = parse_size(value());
            if (!opt.clip_size) {
                std::fprintf(stderr, "flow_movie: --clip-size takes WIDTHxHEIGHT, as 1920x1080\n");
                return 2;
            }
            if (const std::string why = web::clip_size_refusal(opt.clip_size->first, opt.clip_size->second); !why.empty()) {
                std::fprintf(stderr, "flow_movie: %s\n", why.c_str());
                return 2;
            }
        } else if (arg == "--stream") {
            opt.stream = true;
        } else if (arg == "--hover-start") {
            opt.hover_start = true;
        } else if (arg == "--volume") {
            opt.volume = true;
        } else if (arg == "--keep-frames") {
            const auto frames = parse_frame_list(value());
            if (!frames) {
                std::fprintf(stderr, "flow_movie: --keep-frames takes whole frame numbers split by commas, as 150,300\n");
                return 2;
            }
            opt.keep_frames = *frames;
        } else if (arg == "--still-size") {
            opt.still_size = parse_size(value());
            if (!opt.still_size) {
                std::fprintf(stderr, "flow_movie: --still-size takes WIDTHxHEIGHT from 64 to 16384, as 1920x1080\n");
                return 2;
            }
        } else if (arg == "--l2-level") {
            const std::optional<float> level = parse_l2_level(value());
            if (!level) {
                std::fprintf(stderr, "flow_movie: --l2-level takes one number above 0 and below 10000 (1/s), as 120\n");
                return 2;
            }
            opt.look.l2_level = *level;
        } else if (arg == "--l2-smooth") {
            opt.look.l2_smooth = true;
        } else if (arg == "--min-cells") {
            const std::optional<uint32_t> cells = parse_count(value());
            if (!cells) {
                std::fprintf(stderr, "flow_movie: --min-cells takes a whole number of cells, as 27\n");
                return 2;
            }
            opt.look.min_cells = *cells;
        } else if (arg == "--mean-frames") {
            const std::optional<uint32_t> frames = parse_count(value());
            if (!frames || *frames == 0u) {
                std::fprintf(stderr, "flow_movie: --mean-frames takes a whole number of frames above 0, as 10\n");
                return 2;
            }
            opt.look.mean_frames = *frames;
        } else if (arg == "--window") {
            if (std::sscanf(value().c_str(), "%lf:%lf", &opt.window_from, &opt.window_to) != 2 ||
                opt.window_to <= opt.window_from) {
                std::fprintf(stderr, "flow_movie: --window takes T0:T1, as 0.9:1.5, with T1 past T0\n");
                return 2;
            }
        } else if (arg == "--web-still") {
            opt.web_still_only = true;
        } else if (arg == "--web-deliver") {
            opt.web_deliver = value();
        } else if (arg == "--ticket") {
            opt.ticket = value();
        } else {
            std::fprintf(stderr, "flow_movie: unknown argument %s\n", arg.c_str());
            return 2;
        }
    }
    if (opt.gpu) {
#ifndef SPADE_FLOW_MOVIE_GPU
        std::fprintf(stderr, "flow_movie: --gpu needs the Vulkan build (SPADE_VULKAN), which builds GpuFluid\n");
        return 2;
#else
        const bool live = opt.video == "case" || opt.video == "case-smoke" || opt.video == "case-quad" ||
                          opt.video == "d1-hover" || opt.video == "d1-still";
        if (!live || opt.step_time > 0u || opt.subnormal_scan > 0u || opt.subnormal_locate > 0u) {
            std::fprintf(stderr, "flow_movie: --gpu films a LiveRun case (d1-hover, case, case-*), and only films "
                                 "it\n");
            return 2;
        }
#endif
    }
    if (opt.video.empty()) {
        std::fprintf(stderr, "usage: flow_movie --video quad|validation|smoke|cavity|taylor-green|disc|case-smoke "
                             "[--out DIR] [--ffmpeg EXE] [--seconds S] [--stills N] [--resimulate] [--keep-cache]\n"
                             "       flow_movie --case FILE.case.yaml [--controls FILE] [--out DIR] [--seconds S] "
                             "[--drop-cache] [--gpu]\n");
        return 2;
    }
    if (opt.out.empty()) opt.out = default_out();
    Encoder encoder;
    encoder.ffmpeg = opt.ffmpeg;
    if (opt.video == "smoke" || opt.video == "case-smoke") {
        encoder.width = 640;
        encoder.height = 360;
    }
    if (!opt.web_id.empty() || !opt.synthetic_still.empty()) {
        encoder.width = web::kWidth;
        encoder.height = web::kHeight;
        if (opt.clip_size && !opt.web_id.empty()) {  // a diagnostic clip (web_masters.hpp's clip_size_refusal passed)
            encoder.width = opt.clip_size->first;
            encoder.height = opt.clip_size->second;
        }
    }

    if (auto ok = check_ffmpeg(encoder); !ok) return fail(ok.error());
    auto stage = GlStage::create(encoder.width, encoder.height);
    if (!stage) return fail(stage.error());
    std::printf("flow_movie: %s on %s, %ux%u at %u fps, into %s\n", opt.video.c_str(), (*stage)->device().c_str(),
                encoder.width, encoder.height, encoder.fps, opt.out.string().c_str());
    std::printf("flow_movie: built from %s\n", build_identity().c_str());

    if (!opt.web_id.empty()) return run_web(opt, **stage, encoder);
    if (!opt.synthetic_still.empty()) return run_synthetic_still(opt, encoder);

    std::vector<Film> films;
    if (opt.video == "smoke") {
        films.push_back(disc_film(opt.seconds > 0.0 ? frames_for(opt, 0.0, encoder) : 2u, smoke_case(), "smoke"));
    } else if (opt.video == "quad") {
        auto film = quad_film(frames_for(opt, 14.0, encoder));
        if (!film) return fail(film.error());
        if (opt.n_v > 0u) film->airflow.fluid.n_v = opt.n_v;  // Physics' noise test only
        films.push_back(std::move(*film));
    } else if (opt.video == "case" || opt.video == "case-smoke" || opt.video == "case-quad" ||
               opt.video == "d1-hover" || opt.video == "d1-still") {
        const bool smoke = opt.video == "case-smoke", quad = opt.video == "case-quad", d1 = opt.video == "d1-hover";
        const bool still = opt.video == "d1-still";  // D1 in still air: the hover run, wind 0
        if (!smoke && !quad && !d1 && !still && opt.case_file.empty()) {
            std::fprintf(stderr, "flow_movie: --video case needs --case FILE.case.yaml\n");
            return 2;
        }
        auto lc = smoke  ? smoke_live_case(opt.seconds > 0.0 ? opt.seconds : 2.0 / spade::analysis::kFramesPerSecond)
                  : quad ? quad_live_case(opt.seconds > 0.0 ? opt.seconds : spade::analysis::showcase_seconds())
                  : d1   ? d1_live_case(opt.seconds > 0.0 ? opt.seconds : kD1Seconds, opt.grid.value_or(kD1Grid),
                                        !opt.gpu)
                  : still ? d1_still_air_live_case(opt.seconds > 0.0 ? opt.seconds : kD1Seconds,
                                                   opt.grid.value_or(kD1Grid))
                         : live_case_from_files(opt.case_file, opt.controls_file, opt.seconds);
        if (!lc) return fail(lc.error());
        if (opt.step_time > 0u) return step_time_report(*lc, opt.step_time);
        if (opt.subnormal_scan > 0u) return subnormal_scan(*lc, opt.subnormal_scan, opt.from_save, opt.out);
        if (opt.subnormal_locate > 0u) return subnormal_locate(*lc, opt.subnormal_locate, opt.from_save);
        const auto frames = std::max<uint32_t>(
            2u, static_cast<uint32_t>(std::lround(lc->seconds * spade::analysis::kFramesPerSecond)));
        auto film = case_film(std::move(*lc), frames,
                              std::string(smoke ? "case-smoke"
                                          : quad ? "case-quad"
                                          : d1   ? "d1-hover"
                                          : still ? "d1-still"
                                                 : "case-" + opt.case_file.stem().string()) +
                                  (opt.gpu ? "-gpu" : ""),
                              opt.gpu);
        if (!film) return fail(film.error());
        films.push_back(std::move(*film));
    } else if (opt.video == "cavity") {
        films.push_back(cavity_film(frames_for(opt, 12.0, encoder)));
    } else if (opt.video == "taylor-green") {
        films.push_back(taylor_green_film(frames_for(opt, 12.0, encoder)));
    } else if (opt.video == "disc") {
        films.push_back(disc_film(frames_for(opt, 10.0, encoder), disc_wake_case(), "disc-wake"));
    } else if (opt.video == "validation") {
        // Every record read first: a malformed one stops the reel before any
        // frame, never a partial reel (Interface's rule).
        auto records =
            spade::analysis::validation_report(fs::path(SPADE_TESTS_DIR) / "data" / "airflow" / "records");
        if (!records) return fail(records.error());
        Film cavity = cavity_film(frames_for(opt, 12.0, encoder));
        Film tgv = taylor_green_film(frames_for(opt, 12.0, encoder));
        Film disc = disc_film(frames_for(opt, 10.0, encoder), disc_wake_case(), "disc-wake");
        const struct {
            Film* film;
            const char* source;
            const char* identity;
        } reel[] = {{&cavity, "c1-cavity.md", "C1 lid-driven cavity, Re 400"},
                    {&tgv, "c0-taylor-green.md", "C0 Taylor-Green vortex"},
                    {&disc, "c8-actuator-disc.md", "C8 actuator-disc far wake"}};
        for (const auto& item : reel) {
            auto lines = record_lines(*records, item.source, item.identity);
            if (!lines) return fail(lines.error());
            item.film->record = std::move(*lines);
            item.film->picture_x = 0.45f;
        }
        std::vector<fs::path> parts;
        auto opening = title_card(
            encoder, opt.out / "clips", "reel-title", 5.0,
            layout("Spade airflow: validation reel",
                   {"The CPU reference solver (plan A), each case filmed as its V1 record ran it, drawn by Spade's GL "
                    "flow views at full quality.",
                    "Each case shows its record: the measure, the reference, and the tolerance fixed before the first "
                    "run."},
                   {"Records read by spade::analysis::validation_report() from tests/data/airflow/records/",
                    "this reel: " + build_identity()}));
        if (!opening) return fail(opening.error());
        parts.push_back(*opening);
        for (const auto& item : reel) {
            const std::string builds = builds_line(*item.film, opt.out);  // before the cache goes
            auto clips = run_film(*item.film, opt, **stage, encoder);
            if (!clips) return fail(clips.error());
            if (!*clips) return kExitIncomplete;
            if (opt.stills > 0u || opt.simulate_only || opt.still_frame >= 0 || opt.probe_swirl >= 0) continue;
            auto card = film_card(encoder, opt.out / "clips", *item.film, 5.0, builds);
            if (!card) return fail(card.error());
            parts.push_back(*card);
            parts.insert(parts.end(), (*clips)->begin(), (*clips)->end());
        }
        if (opt.stills > 0u || opt.simulate_only || opt.still_frame >= 0 || opt.probe_swirl >= 0) return 0;
        const fs::path out = opt.out / "validation-reel.mp4";
        if (auto joined = concatenate(encoder, parts, out); !joined) return fail(joined.error());
        std::printf("validation-reel: wrote %s\n", out.string().c_str());
        return 0;
    } else {
        std::fprintf(stderr, "flow_movie: unknown video %s\n", opt.video.c_str());
        return 2;
    }

    for (const Film& film : films) {
        const std::string builds = builds_line(film, opt.out);  // before the cache goes
        auto clips = run_film(film, opt, **stage, encoder);
        if (!clips) return fail(clips.error());
        if (!*clips) return kExitIncomplete;
        if (opt.stills > 0u || opt.simulate_only || opt.still_frame >= 0 || opt.probe_swirl >= 0 ||
            !opt.export_name.empty()) {
            continue;
        }
        const bool smoke = opt.video == "smoke" || opt.video == "case-smoke";
        auto card = film_card(encoder, opt.out / "clips", film, smoke ? 0.2 : 4.0, builds);
        if (!card) return fail(card.error());
        std::vector<fs::path> parts{*card};
        for (const fs::path& clip : **clips) parts.push_back(clip);
        const fs::path out = opt.out / (film.stem + ".mp4");
        if (auto joined = concatenate(encoder, parts, out); !joined) return fail(joined.error());
        std::printf("%s: wrote %s (%u frames per view, %zu views)\n", film.stem.c_str(), out.string().c_str(),
                    film.frames, film.views.size());
    }
    return 0;
}
