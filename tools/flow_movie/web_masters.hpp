// tools/flow_movie/web_masters.hpp -- the website clips' pure parts (Cameron's render brief, relayed by the
// lead 2026-11-08): the table of the eight clips S1 to S8, the camera maths, the window's frame-to-step map,
// the encode's arguments, the file name, the MANIFEST line, the delivery guard and the caption's standing
// caveats. Header-only and free of GL, ffmpeg and the solver, so a test links nothing of the tool.
//
// Every claim a caption makes is one the footage and the records carry (the lead's rule): colour is relative
// speed, no speed figure appears, the clip is slow motion by a stated factor, the camera loops but the flow
// does not, and the D1 clips say what was and was not checked (Physics' caveats, 2026-11-08).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "render/scene.hpp"

namespace flow_movie::web {

inline constexpr uint32_t kWidth = 3840, kHeight = 2160;
inline constexpr uint32_t kStillWidth = 5120, kStillHeight = 2880;
inline constexpr uint32_t kFps = 30;
inline constexpr float kMargin = 0.10f;  // clear space on every side, a fraction of the frame
inline constexpr int kCrf = 15;          // the brief's 14 to 16

enum class Scene : uint8_t { quad, d1, taylor_green };
enum class Quantity : uint8_t { speed, vorticity, cp, q_surface };
enum class Camera : uint8_t { loop_orbit, half_orbit, slow_orbit, there_and_back, push_in, still, slow_rotate };

// One clip of the brief.
struct Clip {
    std::string_view id;          // "S1"
    std::string_view short_name;  // the file name's middle
    std::string_view title;       // the caption's title
    std::string_view shows;       // one plain sentence: what the footage shows
    std::string_view group;       // the simulation its frames come from (web_groups()): clips of a group share a cache
    Scene scene = Scene::quad;
    Quantity quantity = Quantity::speed;
    Camera camera = Camera::still;
    double seconds = 0.0;        // the clip's length on screen
    float orbit_degrees = 0.0f;  // the camera's sweep
    float range_lo = 0.0f;       // the colour range, fixed within a scene's quantity (0, 0: the case's own)
    float range_hi = 0.0f;
    bool still = false;  // a 5120x2880 PNG as well

    [[nodiscard]] uint32_t frames() const noexcept { return static_cast<uint32_t>(std::lround(seconds * kFps)); }
    // The camera's last frame is followed by its first: one full orbit, or a swing out and back.
    [[nodiscard]] bool camera_loops() const noexcept {
        return orbit_degrees >= 360.0f || camera == Camera::there_and_back;
    }
};

// A simulation behind one or more clips: its frames are spread evenly over [from_s, to_s] of simulated time
// from rest (a window), and a clip of the group draws the first `clip.frames()` of its `frames`.
struct Group {
    std::string_view name;
    double from_s = 0.0;
    double to_s = 0.0;
    uint32_t frames = 0;
};

// The windows. The quad's developed wake (0.9 to 1.5 s of simulated time) is a default: it is filmed only once
// the frames after 0.6 s have been looked at (Physics: the box's open faces over 1.5 s are unmeasured). The
// Taylor-Green case's time is dimensionless.
[[nodiscard]] inline const std::array<Group, 4>& web_groups() {
    static const std::array<Group, 4> table{{
        {"quad-developed", 0.9, 1.5, 450},
        {"quad-startup", 0.0, 0.6, 360},
        {"d1-crosswind", 0.0, 0.567, 360},
        {"taylor-green", 0.0, 10.0, 300},
    }};
    return table;
}

[[nodiscard]] inline const Group* find_group(std::string_view name) {
    for (const Group& g : web_groups()) {
        if (g.name == name) return &g;
    }
    return nullptr;
}

// The brief's table. Cameras: S1 one full orbit (the brief's), S3 a half orbit, S4 and S7 slow one-way arcs (their
// flow evolves, so nothing loops), S8 a smooth swing out and back that loops, S5 and S6 still.
[[nodiscard]] inline const std::array<Clip, 8>& web_clips() {
    static const std::array<Clip, 8> table{{
        {"S1", "rotor-wake-streamlines", "Reference quad: streamlines of the rotor wake",
         "Streamlines seeded above each of four actuator discs, coloured by relative speed, in one slow orbit.",
         "quad-developed", Scene::quad, Quantity::speed, Camera::loop_orbit, 15.0, 360.0f, 0.0f, 12.0f, true},
        {"S2", "rotor-wake-vortex-surfaces", "Reference quad: vortex surfaces from the start-up",
         "Vortex surfaces (lambda2) of four actuator discs from the start-up roll-up, in a slow push-in.",
         "quad-startup", Scene::quad, Quantity::speed, Camera::push_in, 12.0, 0.0f, 0.0f, 12.0f, false},
        {"S3", "d1-crosswind-vortex-surfaces", "D1 quad in a crosswind: vortex surfaces",
         "Vortex surfaces around the reference quad's rigid parts, held at hover in a crosswind, in a slow half orbit.",
         "d1-crosswind", Scene::d1, Quantity::speed, Camera::half_orbit, 12.0, 180.0f, 0.0f, 0.0f, true},
        {"S4", "d1-airframe-cp", "D1 quad in a crosswind: pressure on the airframe",
         "Pressure coefficient on the reference quad's airframe, held at hover in a crosswind, in a slow orbit.",
         "d1-crosswind", Scene::d1, Quantity::cp, Camera::slow_orbit, 10.0, 120.0f, -1.0f, 1.0f, true},
        {"S5", "rotor-wake-velocity-slice", "Reference quad: a velocity slice through the wake",
         "Relative speed on the vertical plane through two opposite actuator discs.", "quad-developed", Scene::quad,
         Quantity::speed, Camera::still, 10.0, 0.0f, 0.0f, 12.0f, false},
        {"S6", "rotor-swirl-from-above", "Reference quad: the rotor swirl from above",
         "Streamlines through each of four actuator discs seen from above, the rims drawn by spin direction.",
         "quad-developed", Scene::quad, Quantity::speed, Camera::still, 10.0, 0.0f, 0.0f, 12.0f, false},
        {"S7", "taylor-green-q-surfaces", "Taylor-Green vortex: Q surfaces breaking down",
         "Q surfaces of the Taylor-Green vortex in a periodic box as it breaks down, in a slow rotate.",
         "taylor-green", Scene::taylor_green, Quantity::q_surface, Camera::slow_rotate, 10.0, 90.0f, 0.0f, 1.0f, true},
        {"S8", "rotor-wake-volume", "Reference quad: a volume render of the rotor wake",
         "Vorticity magnitude of four actuator discs' wake as a volume, in a slow orbit.", "quad-developed",
         Scene::quad, Quantity::vorticity, Camera::there_and_back, 10.0, 60.0f, 0.0f, 400.0f, false},
    }};
    return table;
}

[[nodiscard]] inline const Clip* find_clip(std::string_view id) {
    for (const Clip& c : web_clips()) {
        if (c.id == id) return &c;
    }
    return nullptr;
}

// ---- the camera --------------------------------------------------------------------------------------

// Azimuth of frame k of n, in degrees. Over k / n, not k / (n - 1): frame n would be frame 0 a sweep on, so a
// full orbit loops with no frame shown twice.
[[nodiscard]] inline float loop_azimuth(float start_deg, float sweep_deg, uint32_t k, uint32_t n) {
    return start_deg + sweep_deg * (n == 0u ? 0.0f : static_cast<float>(k) / static_cast<float>(n));
}

// Azimuth of frame k of n for a swing of `sweep` degrees out and back: start + sweep (1 - cos(2 pi k / n)) / 2. It
// starts and ends at rest, and frame n would be frame 0, so the camera loops with no fast turn.
[[nodiscard]] inline float there_and_back_azimuth(float start_deg, float sweep_deg, uint32_t k, uint32_t n) {
    const double u = n == 0u ? 0.0 : static_cast<double>(k) / static_cast<double>(n);
    return start_deg + sweep_deg * static_cast<float>(0.5 * (1.0 - std::cos(2.0 * 3.14159265358979323846 * u)));
}

// A distance from `wide` to `close` over n frames, eased (a smoothstep) so the move starts and ends at rest.
[[nodiscard]] inline float push_in_distance(float wide, float close, uint32_t k, uint32_t n) {
    const float u = n <= 1u ? 1.0f : static_cast<float>(k) / static_cast<float>(n - 1u);
    const float s = u * u * (3.0f - 2.0f * u);
    return wide + (close - wide) * s;
}

// The distance at which a sphere of `radius` leaves `margin` of the frame clear on every side: along the more
// restrictive axis it covers 1 - 2 margin of the half-frame.
[[nodiscard]] inline float fit_distance(float radius, float fov_y_radians, float aspect, float margin) {
    const double tan_y = std::tan(0.5 * static_cast<double>(fov_y_radians));
    const double tan_half = std::min(tan_y, tan_y * static_cast<double>(aspect)) * (1.0 - 2.0 * static_cast<double>(margin));
    return static_cast<float>(static_cast<double>(radius) * std::sqrt(1.0 + 1.0 / (tan_half * tan_half)));
}

// The distance at which an object `extent` long fills `fraction` of the frame's height.
[[nodiscard]] inline float fill_distance(float extent, float fraction, float fov_y_radians) {
    return extent / (2.0f * fraction * std::tan(0.5f * fov_y_radians));
}

// A slice shown edge to edge: the window of a plane `width` wide that has the frame's shape, and the distance at
// which a camera looking straight at it sees exactly that window.
[[nodiscard]] inline float plane_window_height(float width, float aspect) { return width / aspect; }
[[nodiscard]] inline float fit_plane_distance(float window_height, float fov_y_radians) {
    return window_height / (2.0f * std::tan(0.5f * fov_y_radians));
}

// ---- the window ---------------------------------------------------------------------------------------

// The solver step at which frame k of n ends, when the n frames span the simulated window [t0, t1] and the
// step is h: the frames are spread evenly over the window, the last ending at t1.
[[nodiscard]] inline uint64_t frame_end_step(uint32_t k, uint32_t n, double t0, double t1, double h) {
    const double t = t0 + (t1 - t0) * (static_cast<double>(k) + 1.0) / static_cast<double>(n == 0u ? 1u : n);
    return static_cast<uint64_t>(std::llround(t / h));
}

// ---- the encode, the names, the delivery ----------------------------------------------------------------

// ffmpeg's arguments for one web master: raw RGBA frames in on stdin, one libx264 pass, no captions, no audio.
[[nodiscard]] inline std::vector<std::string> ffmpeg_web_args(const std::string& ffmpeg, uint32_t width,
                                                              uint32_t height, const std::string& out) {
    return {ffmpeg,        "-hide_banner", "-loglevel", "error",  "-y",
            "-f",          "rawvideo",     "-pix_fmt",  "rgba",   "-s",
            std::to_string(width) + "x" + std::to_string(height),
            "-r",          std::to_string(kFps),        "-i",     "-",
            "-c:v",        "libx264",      "-preset",   "slow",   "-crf",
            std::to_string(kCrf),          "-pix_fmt",  "yuv420p", "-movflags",
            "+faststart",  "-an",          out};
}

[[nodiscard]] inline std::string web_file_name(std::string_view id, std::string_view short_name, std::string_view label,
                                               std::string_view ext) {
    return std::string(id) + "_" + std::string(short_name) + "_" + std::string(label) + "." + std::string(ext);
}

[[nodiscard]] inline std::string manifest_line(const std::string& file, uint32_t width, uint32_t height, double seconds,
                                               const std::string& label, const std::string& description) {
    char secs[32];
    std::snprintf(secs, sizeof secs, "%g", seconds);
    return "- " + file + " | " + std::to_string(width) + "x" + std::to_string(height) + " | " + secs + " | " + label +
           " | " + description;
}

// What the delivery may write: files under <raw>/spade, and <raw>/MANIFEST.md. Nothing else under the site.
struct DeliveryGuard {
    std::filesystem::path raw_dir;

    [[nodiscard]] bool may_write(const std::filesystem::path& file) const {
        const std::filesystem::path f = file.lexically_normal();
        const std::filesystem::path raw = raw_dir.lexically_normal();
        if (f == (raw / "MANIFEST.md").lexically_normal()) return true;
        const std::filesystem::path dir = (raw / "spade").lexically_normal();
        const std::filesystem::path rel = f.lexically_relative(dir);
        if (rel.empty() || rel == "." || *rel.begin() == "..") return false;
        return !f.filename().empty();
    }
};

// ---- the captions ---------------------------------------------------------------------------------------

[[nodiscard]] inline std::string slow_motion_line(double sim_seconds, double screen_seconds) {
    char out[160];
    const double per_screen = screen_seconds > 0.0 ? sim_seconds / screen_seconds : 0.0;
    std::snprintf(out, sizeof out,
                  "Slow motion: %.4f simulated seconds in each screen second (%.1f times slower than real time).",
                  per_screen, per_screen > 0.0 ? 1.0 / per_screen : 0.0);
    return out;
}

// The caveats every caption of a clip carries, by what the clip is (Physics', the lead's rulings): no speed
// figure anywhere, colour is relative speed, the stand-ins by name.
[[nodiscard]] inline std::string standing_caveats(const Clip& c) {
    std::string t;
    if (c.scene == Scene::quad) {
        t += "Stand-in: the reference quad is four actuator discs (uniform thrust and swirl), with no airframe, in "
             "still air; the run is inviscid, so the shear layers break down to the grid scale. ";
        t += "Colour shows relative speed, not validated speed, and the picture carries no speed figures. ";
        t += "The wake is a single run on the device of record, not a measurement.";
    } else if (c.scene == Scene::d1) {
        t += "Stand-in: the reference quad's rigid parts as solids in the flow at 4 mm, its rotors as actuator discs "
             "of fixed thrust and swirl, the drone held level at its still-air hover throttle in a steady wind from "
             "the side (crosswind setting supported but not validated). The wind bends and carries the wake and "
             "flows round the airframe's parts, but the rotors do not respond to it and the held drone does not "
             "need to compensate: this is a visualisation of the air, not a prediction of the thrust, power or "
             "attitude a real quad would need in a crosswind. ";
        t += c.quantity == Quantity::cp
                 ? "Pressure is a coefficient against the inflow's pressure, read one cell off each wall (first "
                   "order in the cell size); its sign on the arms' tops has not been measured. "
                 : "The wake below the quad has not been measured against a downwash record. ";
        t += "Colour shows relative speed, not validated speed, and the picture carries no speed figures. ";
        t += "The fluid is stepped by the GPU solver (GpuFluid), checked bit-equal to the CPU's through about 0.18 s "
             "of simulated time only: a single run on the device of record, not the CPU's run.";
    } else {
        t += "Stand-in: the C0 case at 128^3, not the validation record (64^3): a periodic, inviscid Taylor-Green "
             "box, not a drone, its energy loss the numerics' own. A single run on the CPU reference solver.";
    }
    return t;
}

// The camera's line: what loops and what does not (no crossfade, no repeated frames).
[[nodiscard]] inline std::string loop_line(const Clip& c) {
    if (c.camera == Camera::there_and_back) {
        return "the camera swings out and back and loops; the flow does not, so the last frame does not match the "
               "first.";
    }
    if (c.camera_loops()) {
        return "the camera completes one full orbit and loops; the flow does not, so the last frame does not match "
               "the first.";
    }
    if (c.camera == Camera::still) {
        return "the camera is still; the flow does not loop, so the last frame does not match the first.";
    }
    char deg[32];
    std::snprintf(deg, sizeof deg, "%.0f", static_cast<double>(c.orbit_degrees));
    return std::string("the camera sweeps ") + deg +
           " degrees and does not return to its start; the flow does not loop either.";
}

struct CaptionInput {
    const Clip* clip = nullptr;
    std::string commit;
    std::string date_utc;
    std::string command;
    std::string ticket;
    double sim_seconds = 0.0;  // simulated time shown, in the case's own units
    double screen_seconds = 0.0;
    std::string extra;  // anything else the run's log carries (the device, the grid, a legend's note)
};

// Time on the clip, in words: seconds of simulated time per screen second, and how much slower than real time;
// the Taylor-Green case's time has no unit.
[[nodiscard]] inline std::string time_line(const Clip& c, double sim, double screen) {
    if (c.scene != Scene::taylor_green) return slow_motion_line(sim, screen);
    char out[160];
    std::snprintf(out, sizeof out, "Time: the case's own dimensionless time, %.4f units in each screen second.",
                  screen > 0.0 ? sim / screen : 0.0);
    return out;
}

// The .md beside a clip: its title, commit, date, how it was rendered, what it shows, the slow motion, the loop
// and the stand-in. It never claims more than the footage shows.
[[nodiscard]] inline std::string caption_text(const CaptionInput& in) {
    const Clip& c = *in.clip;
    std::string t = "# " + std::string(c.title) + "\n\n";
    t += "- Commit: " + in.commit + "\n";
    t += "- Date: " + in.date_utc + "\n";
    t += "- How rendered: tools/flow_movie, `" + in.command + "`, jobrun ticket " + in.ticket + "\n";
    t += "- Shows: " + std::string(c.shows) + "\n";
    t += "- " + time_line(c, in.sim_seconds, in.screen_seconds) + "\n";
    t += "- Loop: " + loop_line(c) + "\n";
    t += "- " + standing_caveats(c) + "\n";
    if (!in.extra.empty()) t += "- " + in.extra + "\n";
    return t;
}

// A scene whose background is pure black (#000000): no sky, so nothing but the flow and the parts is drawn.
inline void blacken(spade::render::RenderScene& scene) {
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
}

}  // namespace flow_movie::web
