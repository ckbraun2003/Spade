// flow_movie: films the CPU airflow solver through the engine's views.
//
// Each filmed case is simulated once, from rest, with every frame's
// published volume cached to disk at a fixed SIMULATED time; then each view
// is drawn on GL at 1920x1080 at full flow quality and piped to ffmpeg,
// with captions burned in. The video's clock is simulated time only.
//
//   flow_movie --video quad|validation|smoke|cavity|taylor-green|disc
//              [--out DIR] [--ffmpeg EXE] [--seconds S] [--stills N]
//              [--resimulate] [--keep-cache] [--simulate-only]
//              [--budget-seconds S] [--frame-budget N] [--cpu] [--view NAME]
//
// --cpu draws stills on the CPU renderer, the GL views' reference; --view
// limits stills to one view.
//
// The simulation keeps the solver's state every 20 frames, so a run stopped
// by --budget-seconds or --frame-budget (exit 3), or by the box, continues
// from there when run again. A cache from another case or build is refused;
// --resimulate starts again from rest.
//
// --stills N renders N evenly spaced frames of each view as PNGs instead of
// a video, to look at a shot. Exit 0 done, 1 failed, 77 no GL or no ffmpeg.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <functional>
#include <fstream>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "cache.hpp"
#include "cases.hpp"
#include "lambda2_fill.hpp"
#include "recorder.hpp"
#include "views.hpp"

#include "physics/airflow/publish.hpp"
#include "core/fp16.hpp"
#include "render/raster_cpu.hpp"

#ifdef FLOW_MOVIE_HAS_ANALYSIS
#include "analysis/validation.hpp"
#endif

namespace {

namespace fs = std::filesystem;
namespace render = spade::render;
using namespace flow_movie;

constexpr int kExitUnavailable = 77;

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
[[nodiscard]] std::vector<Caption> layout(const std::string& title, const std::vector<std::string>& top,
                                          const std::vector<std::string>& bottom) {
    constexpr std::size_t kBodyWidth = 118;  // 30 px Segoe UI across 1840 px
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
    std::string name;   // the clip's file name
    std::string label;  // the caption naming what it shows
    std::function<spade::Result<void>(const spade::FlowVolume&, uint32_t frame, uint32_t frames,
                                      render::RenderScene&, render::Camera&)>
        shot;
};

struct Film {
    std::string stem;
    AirflowCase airflow;
    uint32_t frames = 300;
    bool lambda2 = false;       // fill lambda2 from the published velocity while Publish leaves it 0
    std::string time_unit;      // " s", or "" for a case in its own units
    int time_decimals = 3;
    std::function<std::vector<double>(const airflow::MacGrid&)> live;  // the case's measures per frame
    std::function<CaptionText(const FrameLog&)> captions;              // the case's lines past the standard ones
    std::vector<std::string> record;  // its V1 record's lines, above the view's (the validation reel)
    std::vector<View> views;
};

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
[[nodiscard]] std::string film_identity(const Film& film) {
    const AirflowCase& c = film.airflow;
    return film.stem + ": " + std::to_string(film.frames) + " frames over " + std::to_string(total_steps(c)) +
           " steps of h " + fixed(c.fluid.h, 6) + " on " + std::to_string(c.shape.nx) + "x" +
           std::to_string(c.shape.ny) + "x" + std::to_string(c.shape.nz) + " at dx " + fixed(c.shape.dx, 6) +
           ", n_v " + std::to_string(c.fluid.n_v) + ", nu " + fixed(c.fluid.nu, 8) + ", rho " + fixed(c.fluid.rho, 4);
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
    airflow::PublishInput in;
    in.origin = c.origin;
    in.periodic = airflow::periodic_axes(c.bc);
    spade::FlowVolume volume;
    uint64_t step = 0;
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
            film.frames <= 1u ? total : static_cast<uint64_t>(std::llround(static_cast<double>(k) * total / (film.frames - 1u)));
        for (; step < target; ++step) {
            for (const airflow::ActuatorDisc& d : c.discs) {
                airflow::spread_actuator(d, c.shape, c.fluid.rho, c.fluid.h, scratch->du, scratch->dv, scratch->dw);
            }
            const airflow::FluidHealth health = airflow::fluid_step(g, c.bc, c.fluid, *scratch);
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
        const bool publish_has_lambda2 =
            h.channel_min[spade::kFlowLambda2Hat] != 0.0f || h.channel_max[spade::kFlowLambda2Hat] != 0.0f;
        if (film.lambda2 && !publish_has_lambda2) fill_lambda2(volume);
        FrameLog log{step, static_cast<double>(step) * c.fluid.h, film.live ? film.live(g) : std::vector<double>{}};
        if (auto wrote = write_frame(cache, k, volume, log); !wrote) return std::unexpected(wrote.error());
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
            std::fflush(stdout);
        }
    }
    fs::remove(cache / "state.bin", ec);
    return true;
}

// --- render many times -----------------------------------------------------------------

[[nodiscard]] std::vector<Caption> standard_captions(const Film& film, const View& view, const FrameLog& log) {
    const AirflowCase& c = film.airflow;
    const CaptionText extra = film.captions ? film.captions(log) : CaptionText{};
    std::vector<std::string> top{c.settings};
    top.insert(top.end(), extra.top.begin(), extra.top.end());
    std::vector<std::string> bottom = film.record;
    bottom.insert(bottom.end(), extra.bottom.begin(), extra.bottom.end());
    bottom.push_back(view.label);
    bottom.push_back("t = " + fixed(log.time, film.time_decimals) + film.time_unit + "   (step " +
                     std::to_string(log.step) + " of " + std::to_string(total_steps(c)) + ")");
    return layout(c.name, top, bottom);
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
            if (auto drew = stage.draw(scene, camera, rgba); !drew) return std::unexpected(drew.error());
            if (auto added = clips[v]->add_frame(rgba, standard_captions(film, film.views[v], log)); !added) {
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
            if (cpu) {
                if (auto drew = draw_cpu(scene, camera, encoder, rgba); !drew) return drew;
            } else if (auto drew = stage.draw(scene, camera, rgba); !drew) {
                return drew;
            }
            const fs::path png =
                dir / (film.stem + "-" + view.name + "-" + std::to_string(k) + (cpu ? "-cpu" : "") + ".png");
            if (auto saved = save_png(encoder, rgba, png); !saved) return saved;
            std::printf("still: %s (t %s)\n", png.string().c_str(), fixed(log.time, film.time_decimals).c_str());
        }
    }
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
    film.lambda2 = true;
    film.time_unit = " s";
    film.captions = [](const FrameLog&) {
        return CaptionText{{"actuator discs: uniform thrust, constant-circulation swirl; no drone body yet "
                            "(D1 adds the solid drone)",
                            "inviscid: the shear layers break down to the grid's 1 cm scale"},
                           {}};
    };
    const AirflowCase& q = film.airflow;
    const float speed_max = 12.0f;  // the far wake: about 2 v_h = 10.7 m/s
    const glm::vec3 target(0.0f, -0.3f, 0.0f);
    const auto speed_colours = colours_over(render::FlowQuantity::speed, render::ColourMapId::viridis, 0.0f, speed_max);
    const auto orbit_at = [target](uint32_t k, uint32_t n) {
        return orbit(target, 1.4f, sweep_azimuth(25.0f, 90.0f, k, n), 18.0f);
    };
    film.views.push_back(View{
        "lambda2", "lambda2 vortex surfaces (lambda2 from the published velocity), coloured by speed |u|, 0 to 12 m/s",
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
    const glm::vec3 target = hub - glm::vec3(0.0f, 2.5f * r, 0.0f);
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
            camera = orbit(target, 10.0f * r, sweep_azimuth(20.0f, 70.0f, k, n), 12.0f);
            return {};
        }});
    return film;
}

[[nodiscard]] Film cavity_film(uint32_t frames) {
    Film film;
    film.stem = "cavity-re400";
    film.airflow = cavity_re400_case();
    film.frames = frames;
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

[[nodiscard]] Film taylor_green_film(uint32_t frames) {
    Film film;
    film.stem = "taylor-green";
    film.airflow = taylor_green_case();
    film.frames = frames;
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
            camera = orbit(volume_centre(v), 12.5f, sweep_azimuth(30.0f, 80.0f, k, n), 24.0f);
            return {};
        }});
    return film;
}

// --- running a film --------------------------------------------------------------------

struct Options {
    std::string video;
    fs::path out;
    std::string ffmpeg = "ffmpeg";
    double seconds = 0.0;  // 0: the film's own
    uint32_t stills = 0;
    bool resimulate = false;
    bool keep_cache = false;
    bool simulate_only = false;
    bool cpu = false;  // stills on the CPU renderer instead of GL
    std::string view;  // stills of this view only
    int still_frame = -1;  // stills of this frame only (cached; no simulation past it)
    int probe_swirl = -1;  // print each disc's vertical vorticity at this cached frame
    uint32_t n_v = 0;      // 0: the case's own; else its V-cycles (Physics' noise test)
    double budget_seconds = 0.0;  // 0: none
    uint32_t frame_budget = 0;    // 0: none
};

constexpr int kExitIncomplete = 3;  // stopped by a budget; run again to continue

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
    return fs::path(profile != nullptr ? profile : ".") / "Desktop" / "spade-videos" / date;
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
    const bool finished = started && cached_frames(cache) == film.frames && !fs::exists(cache / "state.bin", ec);
    const CacheMark theirs = read_mark(cache);
    const auto refuse_cache = [&](const std::string& what, const std::string& found, const std::string& wanted) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument,
                                            "the cache " + cache.string() + " holds another " + what + " (\"" +
                                                found + "\"); this run is \"" + wanted +
                                                "\". --resimulate starts it again"});
    };
    if (started && theirs.film != mine.film) return refuse_cache("film", theirs.film, mine.film);
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
    if (started && !finished && theirs.build != mine.build) {
        return refuse_cache("build's unfinished simulation", theirs.build, mine.build);
    }
    if (!finished) {
        auto done = simulate(film, cache, mine, opt.budget_seconds, opt.frame_budget);
        if (!done) return std::unexpected(done.error());
        if (!*done) return std::optional<std::vector<fs::path>>{};
    } else {
        std::printf("[ %s ] using the %u cached frames in %s\n", film.stem.c_str(), film.frames, cache.string().c_str());
    }
    if (opt.simulate_only) return std::optional<std::vector<fs::path>>{std::vector<fs::path>{}};
    if (opt.stills > 0u) {
        if (auto drew = render_stills(film, cache, stage, encoder, opt.out / "stills", opt.stills, opt.cpu, opt.view); !drew) {
            return std::unexpected(drew.error());
        }
        return std::optional<std::vector<fs::path>>{std::vector<fs::path>{}};
    }
    auto clips = render_film(film, cache, stage, encoder, opt.out / "clips");
    if (!clips) return std::unexpected(clips.error());
    if (!opt.keep_cache) fs::remove_all(cache, ec);
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

#ifdef FLOW_MOVIE_HAS_ANALYSIS
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
#endif

}  // namespace

int main(int argc, char** argv) {
    Options opt;
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
        } else if (arg == "--cpu") {
            opt.cpu = true;
        } else if (arg == "--view") {
            opt.view = value();
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
        } else {
            std::fprintf(stderr, "flow_movie: unknown argument %s\n", arg.c_str());
            return 2;
        }
    }
    if (opt.video.empty()) {
        std::fprintf(stderr, "usage: flow_movie --video quad|validation|smoke|cavity|taylor-green|disc [--out DIR] "
                             "[--ffmpeg EXE] [--seconds S] [--stills N] [--resimulate] [--keep-cache]\n");
        return 2;
    }
    if (opt.out.empty()) opt.out = default_out();
    Encoder encoder;
    encoder.ffmpeg = opt.ffmpeg;
    if (opt.video == "smoke") {
        encoder.width = 640;
        encoder.height = 360;
    }

    if (auto ok = check_ffmpeg(encoder); !ok) return fail(ok.error());
    auto stage = GlStage::create(encoder.width, encoder.height);
    if (!stage) return fail(stage.error());
    std::printf("flow_movie: %s on %s, %ux%u at %u fps, into %s\n", opt.video.c_str(), (*stage)->device().c_str(),
                encoder.width, encoder.height, encoder.fps, opt.out.string().c_str());

    std::vector<Film> films;
    if (opt.video == "smoke") {
        films.push_back(disc_film(opt.seconds > 0.0 ? frames_for(opt, 0.0, encoder) : 2u, smoke_case(), "smoke"));
    } else if (opt.video == "quad") {
        auto film = quad_film(frames_for(opt, 14.0, encoder));
        if (!film) return fail(film.error());
        if (opt.n_v > 0u) film->airflow.fluid.n_v = opt.n_v;  // Physics' noise test only
        films.push_back(std::move(*film));
    } else if (opt.video == "cavity") {
        films.push_back(cavity_film(frames_for(opt, 12.0, encoder)));
    } else if (opt.video == "taylor-green") {
        films.push_back(taylor_green_film(frames_for(opt, 12.0, encoder)));
    } else if (opt.video == "disc") {
        films.push_back(disc_film(frames_for(opt, 10.0, encoder), disc_wake_case(), "disc-wake"));
    } else if (opt.video == "validation") {
#ifdef FLOW_MOVIE_HAS_ANALYSIS
        // Every record read first: a malformed one stops the reel before any
        // frame, never a partial reel (Interface's rule).
        auto records = spade::analysis::validation_report(FLOW_MOVIE_RECORDS_DIR);
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
#else
        return fail(spade::Error{spade::Code::unavailable,
                                 "the validation reel needs spade::analysis's validation_report (Interface)"});
#endif
    } else {
        std::fprintf(stderr, "flow_movie: unknown video %s\n", opt.video.c_str());
        return 2;
    }

    for (const Film& film : films) {
        const std::string builds = builds_line(film, opt.out);  // before the cache goes
        auto clips = run_film(film, opt, **stage, encoder);
        if (!clips) return fail(clips.error());
        if (!*clips) return kExitIncomplete;
        if (opt.stills > 0u || opt.simulate_only || opt.still_frame >= 0 || opt.probe_swirl >= 0) continue;
        auto card = film_card(encoder, opt.out / "clips", film, opt.video == "smoke" ? 0.2 : 4.0, builds);
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
