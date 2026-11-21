// tools/flow_movie/web_masters.hpp -- the website clips' pure parts (Cameron's render brief, relayed by the
// lead 2026-11-08): the table of the seven clips S1 to S6 and S8, the camera maths, the window's frame-to-step map,
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
#include <charconv>
#include <format>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <glm/glm.hpp>

#include "look.hpp"
#include "q_look.hpp"
#include "render/scene.hpp"
#include "wake_fade.hpp"

namespace flow_movie::web {

inline constexpr uint32_t kWidth = 3840, kHeight = 2160;
inline constexpr uint32_t kStillWidth = 5120, kStillHeight = 2880;
inline constexpr uint32_t kFps = 30;
inline constexpr float kMargin = 0.10f;  // clear space on every side, a fraction of the frame
inline constexpr int kCrf = 15;          // the brief's 14 to 16

// S1's streamlines start this far below each rotor's hub along its axis (main.cpp seeds them there); the clip's caption says
// "6 cm below", pinned to this number by FlowMovieWeb.S1sCaptionSaysWhereTheStreamlinesAreSeeded...
inline constexpr float kWebSeedBelowHubM = 0.06f;

// S1's soft streamlines (Cameron, 2026-11-17): thin additive lines with a glow, each faded out (by alpha) over its last kSoftLineFadeM at both
// ends (look.hpp's fade_polyline_ends), not the 4 px tubes of S6. The caption's look line says so.
inline constexpr float kSoftLineWidthPx = 1.5f;
inline constexpr float kSoftLineGlowPx = 6.0f;
inline constexpr float kSoftLineGlowStrength = 0.35f;
inline constexpr float kSoftLineFadeM = 0.15f;

// quad: the reference quad's discs on the CPU film at 1 cm (S5, S6). d1: D1, the parts in the flow, in a side wind
// (S3, S4). d1_still: D1 in still air (S1, S2, S8). (S7, the Taylor-Green clip, was dropped by Cameron: L53.)
enum class Scene : uint8_t { quad, d1, d1_still };
enum class Quantity : uint8_t { speed, cp };
enum class Camera : uint8_t { loop_orbit, half_orbit, slow_orbit, there_and_back, push_in, still };

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
    // The first frame of the group's cache the clip draws: clips of a group that start at different times share one
    // simulation (S2 from the group's frame 30, S1 and S8 from frame 150).
    uint32_t first_frame = 0;

    [[nodiscard]] uint32_t frames() const noexcept { return static_cast<uint32_t>(std::lround(seconds * kFps)); }
    // The camera's last frame is followed by its first: one full orbit, or a swing out and back.
    [[nodiscard]] bool camera_loops() const noexcept {
        return orbit_degrees >= 360.0f || camera == Camera::there_and_back;
    }
};

// A simulation behind one or more clips: from `from_s` of simulated time (from rest), frame k ends
// (k + 1) * steps_per_frame solver steps on, and a clip of the group draws the first `clip.frames()` of its
// `frames`. A WHOLE number of steps a frame, so every frame shows the same advance of time: a frame spacing that
// is not a whole step count rounds to 2, 3, 3 steps and the flow beats at a tenth of the frame rate (the lead,
// 2026-11-10). A cache of fewer frames is a prefix of the full run.
struct Group {
    std::string_view name;
    double from_s = 0.0;
    uint32_t steps_per_frame = 0;
    uint32_t frames = 0;
    double step_s = 0.0005;  // the group's solver step h, s
};

// The solver step of the quad (LiveRun's), s.
inline constexpr double kLiveStepS = 0.0005;

// D1's web masters (the lead, 2026-11-10, L54): the 4 mm GPU sim with the drone's parts in the flow, at the film's
// step of 0.2 ms and with the L52 Smagorinsky term at Cs 0.17.
inline constexpr double kD1WebStepS = 0.0002;
inline constexpr float kD1WebSgsCs = 0.17f;

// --sgs-cs X (Physics' ask, L86): a diagnostic run of a D1 clip at another Smagorinsky Cs, 0 turning the term off, so a
// still pair at one developed time shows what the term does. One finite number from 0 to 1, whole: no sign, no blanks,
// no exponent, no trailing text. nullopt otherwise.
[[nodiscard]] inline std::optional<float> parse_sgs_cs(std::string_view text) {
    if (text.empty()) return std::nullopt;
    bool digit = false, point = false;
    for (const char c : text) {
        if (c >= '0' && c <= '9') {
            digit = true;
        } else if (c == '.' && !point) {
            point = true;
        } else {
            return std::nullopt;
        }
    }
    if (!digit) return std::nullopt;
    float value = 0.0f;
    const auto done = std::from_chars(text.data(), text.data() + text.size(), value);
    if (done.ec != std::errc{} || done.ptr != text.data() + text.size()) return std::nullopt;
    if (!(value >= 0.0f && value <= 1.0f)) return std::nullopt;
    return value;
}

// L106c (the lead, 2026-11-19): the website film at the L106 config. --cell-mm X is the D1 box's cell size in millimetres: one number above 0 and at
// most 10, whole (no sign, blanks, exponent or trailing text); only a cell a drone entry declares is supported (Drone::fine_cell_mm: the X500 2.5, 270 x 237 x 270, the FPV 1.25), see cell_mm_refusal. nullopt otherwise.
[[nodiscard]] inline std::optional<float> parse_cell_mm(std::string_view text) {
    if (text.empty()) return std::nullopt;
    bool digit = false, point = false;
    for (const char c : text) {
        if (c >= '0' && c <= '9') {
            digit = true;
        } else if (c == '.' && !point) {
            point = true;
        } else {
            return std::nullopt;
        }
    }
    if (!digit) return std::nullopt;
    float value = 0.0f;
    const auto done = std::from_chars(text.data(), text.data() + text.size(), value);
    if (done.ec != std::errc{} || done.ptr != text.data() + text.size()) return std::nullopt;
    if (!(value > 0.0f && value <= 10.0f)) return std::nullopt;
    return value;
}

// --step-us N: the solver step in whole microseconds, 1 to 1000 (50 is L106's 0.05 ms): digits only, no sign, blanks, point, exponent or trailing text.
[[nodiscard]] inline std::optional<uint32_t> parse_step_us(std::string_view text) {
    if (text.empty()) return std::nullopt;
    for (const char c : text) {
        if (c < '0' || c > '9') return std::nullopt;
    }
    uint32_t value = 0;
    const auto done = std::from_chars(text.data(), text.data() + text.size(), value);
    if (done.ec != std::errc{} || done.ptr != text.data() + text.size()) return std::nullopt;
    if (value < 1u || value > 1000u) return std::nullopt;
    return value;
}

// The step is for a --window run: a group's frames are 5 whole steps at 0.2 ms, so another step there would change the frame spacing silently.
[[nodiscard]] inline std::string step_us_refusal(uint32_t step_us, bool window) {
    (void)step_us;
    if (window) return {};
    return "--step-us is for a --window run (the group's frames are whole steps at 0.2 ms; --window spreads its frames over the window)";
}

// A run at another cell or step keeps its frames in a folder of its own, as a run at another Cs does: "" when neither is given.
[[nodiscard]] inline std::string cell_step_stem_suffix(std::optional<float> cell_mm, std::optional<uint32_t> step_us) {
    std::string out;
    if (cell_mm) out += std::format("-cell{:g}", *cell_mm);
    if (step_us) out += std::format("-step{}us", *step_us);
    return out;
}

// --particles with --window: the points are carried frame to frame, so a --window run that solves (--stream) feeds them in order, and a --web-still
// redraw from kept frames carries them through the files (L106b). A --window run that does neither has no frames to carry them through.
[[nodiscard]] inline std::string particles_window_refusal(bool window, bool still_redraw, bool stream) {
    if (!window || still_redraw || stream) return {};
    return "--particles is not for a --window run without --stream (a --window run that streams, or a --window --web-still redraw from a run's kept frames, is)";
}

// --nasa and --nasa-cloud (L106i, O113): S2 and S8 only; a --window run is for the cloud and only when it streams (as --particles is), or is a --web-still redraw
// of kept frames (either look); the particle looks are a different look. Empty when the flags are fine, else the refusal's words.
[[nodiscard]] inline std::string nasa_refusal(bool s2_or_s8, bool window, bool still_redraw, bool stream, bool particles, bool particles_preview, bool cloud) {
    if (!s2_or_s8) return "--nasa is for S2 and S8";
    if (window && !still_redraw && !(stream && cloud)) {
        return cloud ? "--nasa-cloud is not for a --window run without --stream (a --window run that streams, or a --window --web-still redraw from a run's kept frames, is)"
                     : "--nasa is not for a --window run (a --window --web-still redraw from a run's kept frames is; --nasa-cloud streams a --window run)";
    }
    if (particles || particles_preview) return "--nasa and the particle looks are different looks: use one";
    return {};
}

// --surface-pressure (O124): the air pressure on the airframe's surfaces, S8 only. Like the cloud it streams a --window run (or redraws kept frames), and it is a different
// look from --nasa, --nasa-cloud and the particle looks. Empty when the flags are fine, else the refusal's words.
[[nodiscard]] inline std::string surface_pressure_refusal(bool s8, bool window, bool still_redraw, bool stream, bool particles, bool particles_preview, bool nasa) {
    if (!s8) return "--surface-pressure is for S8";
    if (window && !still_redraw && !stream) {
        return "--surface-pressure is not for a --window run without --stream (a --window run that streams, or a --window --web-still redraw from a run's kept frames, is)";
    }
    if (particles || particles_preview) return "--surface-pressure and the particle looks are different looks: use one";
    if (nasa) return "--surface-pressure and --nasa or --nasa-cloud are different looks: use one";
    return {};
}

// --surface-shade (O124): the pressure colour lit for shape; it is a way of drawing --surface-pressure, so it needs it. Empty when the flags are fine.
[[nodiscard]] inline std::string surface_shade_refusal(bool shade, bool pressure) {
    if (shade && !pressure) return "--surface-shade is for --surface-pressure (it lights that film's colour for shape)";
    return {};
}

// How far past a solid the pressure film reads (cells): it paints the detailed kit meshes with the air of Physics' proxy solids, so a drawn point can sit inside a solid
// cell, and the read goes on outward along the normal to the nearest fluid cell, up to this far (render::FlowSurfaceColouring::search_cells).
inline constexpr float kSurfacePressureSearchCells = 3.0f;
// FIXED for the whole film, symmetric about ambient (Oversight, O124). The first film's was +-0.75 (1.5 x the 95th percentile of |C_p| at 0.03 s, 0.36, rounded up to 0.25); Cameron
// asked for stronger colours (O129), so it is +-0.4, read off the DEVELOPED film (the delivered one's frames at 3.0 to 9.8 s: of the coloured pixels, C_p p1 -0.32, p5 -0.19,
// p50 +0.07, p95 +0.29, p99 +0.53): the arm strips saturate (2.7% of the coloured pixels, 0.5% of the parts' pixels clip) and the blue tail reads at about half the ramp.
// The pressure film's colour range, C_p = (p - p_inflow) / q over [-range, range] (q = the discs' loading T/A), centred on the still air's pressure: blue below, red above.
inline constexpr float kSurfacePressureCpRange = 0.4f;
// The blades' OWN fixed range (O128), in the same units, symmetric about the still air's pressure, measured from the blades' own test stills (the 600-step run's frames at
// steps 600 and 655, the 70 degree start camera, 38,534 blade pixels over the background): the blades are seen from above, so the colour is the suction on the upper face,
// all below ambient (C_p p1 -7.06, p5 -6.37, p25 -4.11, p50 -1.98, p95 -0.28). +-6 uses the map's whole blue half and clips 8.3% at the blue end (the roots); +-7 would clip 1.1%.
inline constexpr float kBladePressureCpRange = 6.0f;
// The gain that makes the blades read on their own range under the parts' colouring (DrawItem::flow_gain): the colouring's range over the blades'.
[[nodiscard]] constexpr float blade_flow_gain(float blade_range) { return kSurfacePressureCpRange / blade_range; }
[[nodiscard]] constexpr float blade_flow_gain() { return blade_flow_gain(kBladePressureCpRange); }

// --volume (L106h): the S8 particle views draw the points ALONE (their views have no flow layer), so a "volume plus particles" film needs the speed volume
// under them; --volume adds it. It has nothing to add to without a particle view, and a film with it keeps a folder of its own.
[[nodiscard]] inline std::string volume_with_particles_refusal(bool volume, bool particles, bool particles_preview) {
    if (!volume || particles || particles_preview) return {};
    return "--volume adds the speed volume under the particle views: use it with --particles or --particles-preview";
}

[[nodiscard]] inline std::string volume_stem_suffix(bool volume) { return volume ? "-volume" : ""; }

// The default working folder (--out unset): <profile>/Desktop/Vantage/Videos/Footage/Spade/<date> (Cameron, O74, moved by O84: all videos live under
// Vantage/Videos). A posted or kept film goes wherever --out names; this is only the default. No profile: relative to the working directory.
[[nodiscard]] inline std::filesystem::path default_out_dir(std::string_view profile, std::string_view date) {
    return std::filesystem::path(profile.empty() ? std::string(".") : std::string(profile)) / "Desktop" / "Vantage" / "Videos" / "Footage" / "Spade" /
           std::filesystem::path(std::string(date));
}

// --elevation DEG: how far above the drone the still's camera looks down from, in place of the drone's own framing (the X500's 20): a close-up of the
// top plate's boards wants a higher camera. One whole number above 0 and at most 85 (straight overhead has no "up" for the orbit): no sign, no
// blanks, no exponent, no trailing text. nullopt otherwise.
[[nodiscard]] inline std::optional<float> parse_elevation_deg(std::string_view text) {
    if (text.empty()) return std::nullopt;
    bool digit = false, point = false;
    for (const char c : text) {
        if (c >= '0' && c <= '9') {
            digit = true;
        } else if (c == '.' && !point) {
            point = true;
        } else {
            return std::nullopt;
        }
    }
    if (!digit) return std::nullopt;
    float value = 0.0f;
    const auto done = std::from_chars(text.data(), text.data() + text.size(), value);
    if (done.ec != std::errc{} || done.ptr != text.data() + text.size()) return std::nullopt;
    if (!(value > 0.0f && value <= 85.0f)) return std::nullopt;
    return value;
}

// The folder suffix of a run at `cs`: none at the master's Cs, else "-cs<cs>". A run at another Cs keeps its frames in a
// folder of its own, so it can never be mistaken for, or overwrite, the master's cache.
[[nodiscard]] inline std::string sgs_cs_suffix(float cs) {
    return cs == kD1WebSgsCs ? std::string() : std::format("-cs{:g}", cs);
}

// The windows. The quad's (S5, S6; 0.5 ms a step): developed wake 0.9 to 1.35 s at 2 steps a frame (Physics: the
// box's open faces past 1.5 s are unmeasured). D1's (0.2 ms a step): ONE still-air run from rest, 5 steps (1 ms) a
// frame, 600 frames to 0.60 s: S2 draws frames 0 to 359 (the start-up roll-up, to 0.36 s), S1 and S8 frames 150 to
// 599 (0.15 s on, 450 frames; 0.15 s is the wake's development time AS ASSUMED: the 0.15 s test run confirms it).
// The side wind's run from rest at 7 steps a frame (1.4 ms), 360 frames, to 0.504 s (S3, S4).
[[nodiscard]] inline const std::array<Group, 3>& web_groups() {
    static const std::array<Group, 3> table{{
        {"quad-developed", 0.9, 2, 450, kLiveStepS},
        {"d1-still", 0.0, 5, 600, kD1WebStepS},
        {"d1-crosswind", 0.0, 7, 360, kD1WebStepS},
    }};
    return table;
}

[[nodiscard]] inline const Group* find_group(std::string_view name) {
    for (const Group& g : web_groups()) {
        if (g.name == name) return &g;
    }
    return nullptr;
}

// The brief's table. Cameras: S1 one full orbit (the brief's), S3 a half orbit, S4 a slow one-way arc (its
// flow evolves, so nothing loops), S8 a smooth swing out and back that loops, S5 and S6 still.
[[nodiscard]] inline const std::array<Clip, 7>& web_clips() {
    static const std::array<Clip, 7> table{{
        {"S1", "rotor-wake-streamlines", "Reference quad in hover: streamlines of the rotor wake",
         "Streamlines seeded 6 cm below each of four actuator discs (two rings of eight round each axis) and traced both "
         "ways, round the reference quad's rigid parts, coloured by relative speed and drawn as thin glowing lines that "
         "fade out toward their ends, in one slow orbit.",
         "d1-still", Scene::d1_still, Quantity::speed, Camera::loop_orbit, 15.0, 360.0f, 0.0f, 12.0f, true, 150},
        {"S2", "rotor-wake-start-up-volume", "Reference quad in hover: the rotor wake as a soft volume, from the start-up",
         "Relative speed of the wake of four actuator discs round the reference quad's rigid parts as a soft volume, held "
         "at hover in still air, from the start-up roll-up into the developed column, in a slow push-in.",
         "d1-still", Scene::d1_still, Quantity::speed, Camera::push_in, 12.0, 0.0f, 0.0f, 12.0f, false, 30},
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
        {"S8", "rotor-wake-volume", "Reference quad in hover: a volume render of the rotor wake",
         "Relative speed of the wake of four actuator discs round the reference quad's rigid parts as a volume, in a "
         "slow orbit.",
         "d1-still", Scene::d1_still, Quantity::speed, Camera::there_and_back, 10.0, 60.0f, 0.0f, 12.0f, false, 150},
    }};
    return table;
}

[[nodiscard]] inline const Clip* find_clip(std::string_view id) {
    for (const Clip& c : web_clips()) {
        if (c.id == id) return &c;
    }
    return nullptr;
}

// --sgs-cs is for the D1 clips: S5 and S6 run the reference quad's CPU film, which has no subgrid term.
[[nodiscard]] inline bool sgs_cs_applies(const Clip& clip) { return clip.scene != Scene::quad; }

// ---- the camera --------------------------------------------------------------------------------------

// The point a D1 web camera aims at, `below_m` below the drone's centre of mass, in world coordinates (the COM is
// the world origin: the live run holds it there). It is NOT the box's centre: the cameras aimed at that until L60,
// when the box stopped being symmetric about the drone (a deeper floor, the drone 0.354 m below the top face), and
// the centre moved 0.43 m off it. In the old box (0.354 m either side in y) the two are the same point.
[[nodiscard]] inline glm::vec3 subject_point(float below_m = 0.0f) { return glm::vec3(0.0f, -below_m, 0.0f); }

// The still-air D1 clips' framing (S1, S8, S2; the lead, L63: cameras closer, the floor out of frame). The subject is
// a sphere about subject_point(kD1StillAimBelowM) that the 10% margin fits; the camera looks DOWN only 8 degrees, so
// the box's floor face stays below the frame's bottom edge (a steeper look at the same size puts it back in: at the
// old 20 degrees and a 0.55 m subject the floor topped out 17% up the frame).
inline constexpr float kD1StillAimBelowM = 0.08f;
inline constexpr float kD1StillSubjectM = 0.40f;    // S1 and S8: the airframe (about 0.3 m) and about 0.1 m of wake
inline constexpr float kD1PushWideM = 0.45f;        // S2: from this subject radius ...
inline constexpr float kD1PushCloseM = 0.30f;       // ... to this one (the airframe)
inline constexpr float kD1StillElevationDeg = 8.0f;
inline constexpr float kD1StillAzimuthDeg = 25.0f;  // where S1, S8 and S2 start

// ---- the drone ----------------------------------------------------------------------------------------

// A library drone the D1 web clips can film (--drone NAME; the lead, 2026-11-17: Cameron wants a real drone model, and no
// caption or file name carries a brand). `library` is the internal name, the model file's stem, and is never printed in a
// caption or a delivered name; `title_noun` and `the_noun` are what a title and a sentence call the drone, `description`
// the caption's Drone line ("" for the reference quad, whose words are today's).
// How a drone is framed in the still-air clips (S1, S8, S2): the subject sphere's radius the camera fits, how far below the drone's
// centre it aims, S2's push-in from wide to close, and where the wake fades out (metres below the drone's centre, and the width of
// the soft edge at the box's side faces). A fade of zero is none: the reference quad's frames are what they always were.
struct Framing {
    float subject_m;
    float aim_below_m;
    float push_wide_m;
    float push_close_m;
    float wake_fade_from_below_m;
    float wake_fade_to_below_m;
    float side_fade_m;
    float wake_fade_top_from_above_m;  // the inflow haze above the rotors fades from here (above the centre) ...
    float wake_fade_top_to_above_m;    // ... to nothing here; 0, 0 is none
    float elevation_deg;               // how far above the drone the camera looks down from

    [[nodiscard]] WakeFade fade() const noexcept {
        return WakeFade{side_fade_m, -wake_fade_from_below_m, -wake_fade_to_below_m, wake_fade_top_from_above_m,
                        wake_fade_top_to_above_m};
    }
};

struct Drone {
    std::string_view library;
    const char* model_file;  // under SPADE_ASSETS_DIR
    std::string_view alias;  // what the caption's command line shows in place of the library name (--drone takes both)
    std::string_view title_noun;
    std::string_view the_noun;
    std::string_view description;
    Framing framing;
    float body_y_m = 0.004f;  // the held body's height above the box's origin (the cloud is thinned round it)
    float reach_m = 0.30f;    // the farthest rotor tip from the vertical axis (the cloud's thinning radius is a multiple of it)
    // The fine box this drone's web films can run in (--cell-mm): the cell in mm, 0 for none. The box and hold are the harness's (live_case.hpp's d1_web_grid and d1_web_hold): 2.5 is
    // the X500's, 1.25 the 5 inch FPV's. hub_above_origin_m is the hub disc's height above the model's DESIGN origin, the world origin the box holds (not the centre of mass), for a box whose hold depends on it (the FPV's d1_fpv_hold; the X500's
    // hold is fixed by the harness, so it leaves this 0).
    float fine_cell_mm = 0.0f;
    float hub_above_origin_m = 0.0f;
    // What a blades film says of this drone (--prop-blur mode=blades): the blades' OWN fixed range in disc loadings (measured from this airframe's own test stills; the X500's is
    // kBladePressureCpRange), what the blades are called, what the pressure colour is on, and any card lines of its own (a bullet after the caveats; the X500 has none).
    float blade_cp_range = kBladePressureCpRange;
    std::string_view blade_words = "the kit's real blades";
    std::string_view parts_words = "the frame, arms, legs, boards and motors";
    std::string_view card_extra = "";
};

[[nodiscard]] inline const std::array<Drone, 4>& drones() {
    static const std::array<Drone, 4> table{{
        {"reference_quad", "library/drones/reference_quad.model.yaml", "reference-quad", "Reference quad",
         "the reference quad", "",
         Framing{kD1StillSubjectM, kD1StillAimBelowM, kD1PushWideM, kD1PushCloseM, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, kD1StillElevationDeg}},
        {"phantom3_class", "library/drones/phantom3_class.model.yaml", "camera-quadcopter", "Camera quadcopter",
         "the camera quadcopter", "a 1.28 kg camera quadcopter (generated from published dimensions)",
         // Larger in the frame, in the upper third (aimed 0.17 m below it); the wake dissolves over its last 0.22 m before the
         // frame's bottom edge, and over 0.10 m before the box's side faces (its disc edges are 0.09 m from them).
         // The slow inflow above the rotors fades out over the 5 cm above their plane, so the drone reads in front of it; seen
         // from 25 degrees above, so its body is not hidden behind the wake of its near rotors (the floor is out of frame: the fade).
         Framing{0.34f, 0.17f, 0.50f, 0.34f, 0.38f, 0.60f, 0.10f, 0.01f, 0.06f, 25.0f}, 0.004f, 0.2963f},  // its proxy radius
        // The X500's flow model (Physics' tool data, found from the assets directory; 10 in rotors reach 0.37 m from the axis, so a
        // shot about twice as wide as the camera quadcopter's), the held body 0.014 m above the box's origin in its 3 mm box.
        {"x500", "../tools/flow_movie/data/x500/x500.model.yaml", "open-source-quadcopter", "Open-source quadcopter",
         "the open-source quadcopter", "a 1.63 kg open-source quadcopter with 10 inch rotors (its parts' masses summed)",
         Framing{0.40f, 0.20f, 0.55f, 0.40f, 0.22f, 0.30f, 0.05f, 0.01f, 0.06f, 20.0f}, 0.014f, 0.373f, 2.5f},  // 0.2461 hub + 0.127 rotor; its 2.5 mm box (270 x 237 x 270, L106)
        // The OpenFrame-5F (O138): the real open-hardware 5 inch FPV frame, Interface's parts table (tools/flow_movie/data/openframe5f_build) over Physics' flow model
        // (tools/flow_movie/data/openframe5f). Its 5 inch rotors reach 0.1764 m from the axis (hubs at +-0.0695 m along and +-0.089 m across, plus the 0.0635 m rotor), so the
        // framing is the X500's scaled by about 0.47, aimed at the kit's centre of mass (0.0300 m above the design origin: aim_below -0.030, so the drone sits mid-frame); its fine box is the FPV's 1.25 mm one, held at the DESIGN origin (the world origin; the centre of mass sits above it), so the hub disc, 0.0436 m above the design origin in the table's frame, is the hold's input: d1_fpv_hold(0.0436), y 0.00875 m, 7 rows (Physics, tried on the device: 0.0136 left the line outside the grid). The blade
        // range is the X500's +-6, kept because the kit's own test stills (steps 600 and 667, 70 degrees, 67,572 blade pixels) read the same: C_p p1 -7.1, p5 -6.2, p25 -3.9, p50 -2.0, p95 -0.4, min -7.9;
        // +-6 clips 6.0% at the blue end (the roots), +-7 would clip 1.4%. The card names the kit factually and carries the stand-in propeller and published-dimensions sentences.
        {"openframe5f", "../tools/flow_movie/data/openframe5f/openframe5f.model.yaml", "fpv-quadcopter", "5 inch FPV quadcopter",
         "the 5 inch FPV quadcopter", "a 0.57 kg 5 inch FPV quadcopter built on the OpenFrame-5F open-hardware frame (its parts' masses summed)",
         Framing{0.19f, -0.030f, 0.26f, 0.19f, 0.10f, 0.14f, 0.025f, 0.005f, 0.03f, 70.0f}, 0.00875f, 0.1764f, 1.25f, 0.0436f, kBladePressureCpRange,
         "generated three-blade stand-in blades", "the frame, arms, stack, motors and battery",
         "Propellers: a measured 5x4 three-blade stand-in for the kit's 5x4.3x3, its rotor model run above the speeds it was measured at, carrying about 84% of the weight in a 0.05 s check; "
         "motors and flight stack modelled from published dimensions; battery: an assumed typical 6S pack."},
    }};
    return table;
}

[[nodiscard]] inline const Drone& default_drone() { return drones().front(); }

// The drone named `name`, by its library name or its alias ("" is the default); nullptr when there is none by that name.
[[nodiscard]] inline const Drone* find_drone(std::string_view name) {
    if (name.empty()) return &default_drone();
    for (const Drone& d : drones()) {
        if (d.library == name || d.alias == name) return &d;
    }
    return nullptr;
}

// Where --cell-mm applies: the cell the drone entry declares (Drone::fine_cell_mm: the X500's 2.5, the FPV's 1.25) on a still-air D1 clip (S1, S2, S8), whose box and hold the
// harness and the film then share. A refusal names what is supported, from the entries.
[[nodiscard]] inline std::string cell_mm_refusal(float cell_mm, std::string_view drone_library, bool still_air_clip) {
    const Drone* d = find_drone(drone_library);
    if (d != nullptr && d->fine_cell_mm > 0.0f && cell_mm == d->fine_cell_mm && still_air_clip) return {};
    std::string supported;
    for (const Drone& e : drones()) {
        if (e.fine_cell_mm > 0.0f) supported += std::format("{}{:g} with --drone {}", supported.empty() ? "" : ", ", e.fine_cell_mm, e.library);
    }
    if (d != nullptr && d->fine_cell_mm <= 0.0f) {
        return std::format("--cell-mm: --drone {} has no fine box (only {})", drone_library, supported);
    }
    return std::format("--cell-mm supports only {}, on a still-air clip (S1, S2, S8)", supported);
}

// The command line as a caption may show it: the drone's library name replaced by its alias.
// The file arguments of --visual-mesh, --lines, --build-parts and --part-store are replaced by plain words too (a model's name, a propeller maker and a vendor's
// folder are brands, and captions name none; Oversight, 2026-11-17): the value after the flag, quoted or not, is the word.
[[nodiscard]] inline std::string command_in_words(std::string command, const Drone& d) {
    const std::string from(d.library);
    for (std::size_t at = command.find(from); at != std::string::npos; at = command.find(from, at + d.alias.size())) {
        command.replace(at, from.size(), d.alias);
    }
    // The model's and the table's words are the drone's own alias plus a noun: the X500's stay what they were, the kit's are its own.
    const std::string model_word = std::string(d.alias) + "-model";
    const std::string table_word = std::string(d.alias) + "-parts-table";
    const std::pair<std::string_view, std::string_view> files[] = {{"--visual-mesh", model_word},
                                                                   {"--lines", "measured-propeller-polar"},
                                                                   {"--build-parts", table_word},
                                                                   {"--part-store", "part-store"}};
    for (const auto& [flag, word] : files) {
        const std::string key = std::string(flag) + " ";
        for (std::size_t at = command.find(key); at != std::string::npos; at = command.find(key, at + key.size() + word.size())) {
            const std::size_t start = at + key.size();
            if (start >= command.size() || command.compare(start, 2, "--") == 0) continue;  // no value: left alone
            std::size_t end = 0;
            if (command[start] == '"') {
                const std::size_t close = command.find('"', start + 1);
                end = close == std::string::npos ? command.size() : close + 1;
            } else {
                const std::size_t space = command.find(' ', start);
                end = space == std::string::npos ? command.size() : space;
            }
            command.replace(start, end - start, word);
        }
    }
    // --out and --cache-dir name this box's folders: shown from build-ninja/ on, else by the folder's own name, so a caption carries no user path.
    for (const std::string_view flag : {std::string_view("--out"), std::string_view("--cache-dir")}) {
        const std::string key = std::string(flag) + " ";
        for (std::size_t at = command.find(key); at != std::string::npos; at = command.find(key, at + key.size())) {
            if (at != 0 && command[at - 1] != ' ') continue;
            const std::size_t start = at + key.size();
            if (start >= command.size() || command.compare(start, 2, "--") == 0) continue;
            const std::size_t space = command.find(' ', start);
            const std::size_t end = space == std::string::npos ? command.size() : space;
            std::string value = command.substr(start, end - start);
            for (char& ch : value) {
                if (ch == '\\') ch = '/';
            }
            while (value.size() > 1u && value.back() == '/') value.pop_back();
            const std::size_t build = value.find("build-ninja/");
            if (build != std::string::npos) {
                value = value.substr(build);
            } else {
                const std::size_t slash = value.rfind('/');
                if (slash != std::string::npos) value = value.substr(slash + 1u);
            }
            command.replace(start, end - start, value);
        }
    }
    return command;
}

// Whether a web run draws the drone's wake fade (the soft edge of a drawn wake, hiding the box's cut): S2 and S8, in the plain looks and
// the nasa CLOUD (a soft volume); not for S1's lines, the particle looks, the nasa Q SURFACES (a faded surface vanishes), nor the surface-pressure film (no wake is drawn).
[[nodiscard]] inline bool web_fade_applies(std::string_view clip_id, bool particles_preview, bool particles, bool nasa, bool nasa_cloud, bool pressure = false) {
    return (clip_id == "S2" || clip_id == "S8") && !particles_preview && !particles && !pressure && (!nasa || nasa_cloud);
}

// A WEB RUN'S LOOK, made ONCE (the lead, 2026-11-18, after the cloud thinning was set on the plan's copy while the film's look state read
// the options' copy and it never reached a frame): the options' look whole, plus the drone's wake fade where it applies (S2 and S8 without
// a particle or nasa look), plus the nasa cloud's thinning round the held body (the drone's own height) when `q` asks for one. main
// assigns this one value to opt.look (the film's look state) and plan.look (the film's setup), so the two cannot differ.
[[nodiscard]] inline Look web_look(Look from_options, const Drone& drone, bool fade_applies, const QLook* q) {
    Look look = from_options;
    if (fade_applies) look.fade = drone.framing.fade();
    if (q != nullptr && q->thin_r > 0.0f) {
        look.thin.centre = glm::vec3(0.0f, drone.body_y_m, 0.0f);
        look.thin.radius = q->thin_r * drone.reach_m;  // a multiple of the drone's own reach: one value fits every drone
        look.thin.floor = q->thin_floor;
    }
    return look;
}

// --loop-blend S (Oversight via the lead, 2026-11-17: the site loops each clip and the flow does not repeat): the clip renders E extra
// frames past its end and its first E frames are a crossfade from the continuation (frame N + j) to its own start (frame j). E is S x
// 30 rounded; the blend must be under half the clip, and a non-zero S that rounds to no frame is refused rather than ignored.
[[nodiscard]] inline uint32_t loop_blend_frames(double seconds, uint32_t clip_frames) {
    (void)clip_frames;
    return seconds > 0.0 ? static_cast<uint32_t>(std::lround(seconds * kFps)) : 0u;
}
[[nodiscard]] inline std::string loop_blend_refusal(double seconds, uint32_t clip_frames) {
    if (seconds == 0.0) return {};
    if (!(seconds > 0.0)) return "--loop-blend takes a length in seconds above 0 (0 is off)";
    const uint32_t e = loop_blend_frames(seconds, clip_frames);
    if (e == 0u) return "--loop-blend rounds to no frame at " + std::to_string(static_cast<int>(kFps)) + " fps: give at least 0.02 s";
    if (2u * e >= clip_frames) return "--loop-blend must be under half the clip (" + std::to_string(clip_frames) + " frames): " + std::to_string(e) + " is not";
    return {};
}
[[nodiscard]] inline uint32_t loop_blend_total_frames(uint32_t clip_frames, uint32_t blend_frames) { return clip_frames + blend_frames; }

// The start's weight in the blended frame j of E: a smoothstep of (j + 1) / (E + 1), so the first is almost the continuation and the last
// almost the start (strictly between 0 and 1, rising).
[[nodiscard]] inline float loop_blend_weight(uint32_t j, uint32_t blend_frames) {
    const float x = static_cast<float>(j + 1u) / static_cast<float>(blend_frames + 1u);
    return x * x * (3.0f - 2.0f * x);
}

// (1 - w) x tail + w x head, per byte, rounded; alpha (every fourth byte) is the tail's. Frames of different sizes give nothing.
[[nodiscard]] inline std::vector<uint8_t> blend_rgba(const std::vector<uint8_t>& tail, const std::vector<uint8_t>& head, float w) {
    if (tail.size() != head.size()) return {};
    std::vector<uint8_t> out(tail.size());
    for (std::size_t i = 0; i < tail.size(); ++i) {
        if (i % 4u == 3u) {
            out[i] = tail[i];
            continue;
        }
        const float v = (1.0f - w) * static_cast<float>(tail[i]) + w * static_cast<float>(head[i]);
        out[i] = static_cast<uint8_t>(std::lround(std::min(std::max(v, 0.0f), 255.0f)));
    }
    return out;
}

// The caption's sentence naming the method; "" without a blend.
[[nodiscard]] inline std::string loop_blend_caption(uint32_t blend_frames) {
    if (blend_frames == 0u) return {};
    char length[32];
    std::snprintf(length, sizeof length, "%g s", static_cast<double>(blend_frames) / static_cast<double>(kFps));
    return std::string("Made to loop: its first ") + length +
           " is a crossfade from the flow's continuation past the end into the clip's own start, so the flow does not jump when the clip repeats.";
}

// --cache-dir DIR: a still redrawn from a frame another run wrote. The run reads DIR instead of its own cache folder and never
// simulates or writes there, so it is refused unless it is a --web-still redraw of a named still (--still-frame N) whose frame file is
// already in DIR. "" when it will do (and when there is no --cache-dir), else why not, naming the missing piece.
[[nodiscard]] inline std::string cache_dir_refusal(std::string_view dir, bool still_only, bool has_still_frame, bool frame_present) {
    if (dir.empty()) return {};
    if (!still_only) return "--cache-dir is for a --web-still redraw: it reads a frame another run wrote and never simulates";
    if (!has_still_frame) return "--cache-dir needs --still-frame N, the frame to redraw";
    if (!frame_present) {
        return "--cache-dir " + std::string(dir) + ": the frame is not there; a run with --cache-dir never simulates or writes in it";
    }
    return {};
}

// A cache folder's suffix for a drone: none for the default (today's caches keep their names), "-<library>" else.
[[nodiscard]] inline std::string drone_stem_suffix(const Drone& d) {
    return &d == &default_drone() ? std::string() : "-" + std::string(d.library);
}

// --lines (the rotors as actuator lines): a lines run is another simulation, so its cache folder has its own name ("-lines"); and
// only the D1 clips take lines (S5 and S6 are the reference quad's discs on the CPU film). "" when it applies, else why not.
[[nodiscard]] inline std::string lines_stem_suffix(bool lines) { return lines ? "-lines" : ""; }
[[nodiscard]] inline std::string lines_refusal(bool d1) {
    return d1 ? std::string() : "--lines is for the D1 clips (S1 to S4 and S8): S5 and S6 run the reference quad's discs on the CPU";
}

// `text` (a clip's title or sentences, written for the reference quad) in the drone's own words.
[[nodiscard]] inline std::string in_words(std::string text, const Drone& d) {
    const auto swap = [&text](const std::string& from, std::string_view to) {
        for (std::size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) {
            text.replace(at, from.size(), to);
        }
    };
    if (&d == &default_drone()) return text;
    swap("the reference quad", d.the_noun);
    swap("Reference quad", d.title_noun);
    swap("D1 quad", d.title_noun);
    return text;
}

// The deeper D1 box's floor face (Physics' plan A, L60/L63), world coordinates (the COM is the origin): 1.214 m below
// the COM, the box ±0.48 m in x and ±0.336 m in z. No wall: the face is open, so what the solve shows there is an
// artifact the clips keep out of frame.
inline constexpr float kDeepFloorY = -1.214f;
inline constexpr float kBoxHalfX = 0.48f, kBoxHalfZ = 0.336f;

// The highest the floor face reaches in the frame, as normalised device y (+1 the top edge, -1 the bottom edge), for a
// camera `distance` from `target` at an azimuth about +y (0 on +z) and an elevation, looking at it (views.cpp's
// orbit()), with a vertical field of view `fov_y_radians`. The floor is out of frame when this is at most -1. A
// projection keeps a straight edge straight, so its highest point is a corner of the face. A corner at or behind the
// camera's plane answers FLT_MAX (not known to be out of frame).
[[nodiscard]] inline float floor_top_ndc(glm::vec3 target, float distance, float azimuth_deg, float elevation_deg,
                                         float fov_y_radians) {
    const double az = static_cast<double>(azimuth_deg) * 3.14159265358979323846 / 180.0;
    const double el = static_cast<double>(elevation_deg) * 3.14159265358979323846 / 180.0;
    const glm::dvec3 from(std::cos(el) * std::sin(az), std::sin(el), std::cos(el) * std::cos(az));
    const glm::dvec3 eye = glm::dvec3(target) + static_cast<double>(distance) * from;
    const glm::dvec3 forward = -from;
    const glm::dvec3 right = glm::normalize(glm::cross(forward, glm::dvec3(0.0, 1.0, 0.0)));
    const glm::dvec3 up = glm::cross(right, forward);
    const double tan_half = std::tan(0.5 * static_cast<double>(fov_y_radians));
    double top = -1.0e30;
    for (const float sx : {-kBoxHalfX, kBoxHalfX}) {
        for (const float sz : {-kBoxHalfZ, kBoxHalfZ}) {
            const glm::dvec3 rel = glm::dvec3(sx, kDeepFloorY, sz) - eye;
            const double depth = glm::dot(rel, forward);
            if (depth <= 1.0e-6) return std::numeric_limits<float>::max();
            top = std::max(top, glm::dot(rel, up) / depth / tan_half);
        }
    }
    return static_cast<float>(top);
}

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

// The solver step at which frame k ends in a whole-step group: `steps_per_frame` after the frame before, the
// first that many past `from_step` (the step of the window's start).
[[nodiscard]] inline uint64_t whole_frame_end_step(uint32_t k, uint64_t from_step, uint32_t steps_per_frame) {
    return from_step + (static_cast<uint64_t>(k) + 1u) * steps_per_frame;
}

// Where a whole-step window of `frames` frames ends, s, at step h.
[[nodiscard]] inline double window_end_s(double from_s, uint32_t frames, uint32_t steps_per_frame, double h) {
    return from_s + static_cast<double>(frames) * static_cast<double>(steps_per_frame) * h;
}

// The solver step at which frame k of n ends, when the n frames span the simulated window [t0, t1] and the
// step is h: the frames are spread evenly over the window, the last ending at t1. (A window given by --window
// A:B: its frames are not a whole number of steps apart unless the window happens to make them so.)
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

// A DIAGNOSTIC CLIP SIZE (--clip-size WxH): the web clips are kWidth x kHeight masters; a clip at another size is a diagnostic that
// must never pass for one. 16:9 (the framing is fitted to that aspect), a whole multiple of 16 wide so the encoder takes it, and
// no larger than the master. "" when it will do, else why not. Its file is labelled with its size (clip_label), and delivery,
// which checks the size, refuses it.
[[nodiscard]] inline std::string clip_size_refusal(uint32_t w, uint32_t h) {
    if (w == 0u || h == 0u) return "--clip-size needs a width and a height above 0";
    if (uint64_t{h} * 16u != uint64_t{w} * 9u) return "--clip-size must be 16:9 (the framing is fitted to it), as 1920x1080";
    if (w % 16u != 0u) return "--clip-size's width must be a multiple of 16";
    if (w > kWidth || h > kHeight) return "--clip-size is a diagnostic: no larger than the master, " + std::to_string(kWidth) + "x" + std::to_string(kHeight);
    return {};
}

// The label of a clip's file name: "sim" for a master, "diag-WxH" for a clip at another size (0, 0: the master's).
[[nodiscard]] inline std::string clip_label(uint32_t w, uint32_t h) {
    return w == 0u && h == 0u ? std::string("sim") : "diag-" + std::to_string(w) + "x" + std::to_string(h);
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
[[nodiscard]] inline std::string standing_caveats(const Clip& c, float cell_mm = 4.0f, uint32_t step_us = 200u, bool lines = false, bool cloud = false, bool pressure = false, bool shaded = false, bool blades = false,
                                                    float blade_range = kBladePressureCpRange, std::string_view blade_words = "the kit's real blades",
                                                    std::string_view parts_words = "the frame, arms, legs, boards and motors") {
    // The rotors' words: actuator discs of fixed thrust and swirl, or (--lines) actuator lines with the blade polar fitted to the propeller.
    const std::string rotors = lines ? "actuator lines with a fitted blade polar" : "actuator discs of fixed thrust and swirl";
    // The cell and the step the film ran at, in the caveats' own words (today's: 4 mm, 0.2 ms; --cell-mm / --step-us change them).
    const std::string cell = std::format("{:g}", cell_mm);
    const std::string step_ms = std::format("{:g}", static_cast<double>(step_us) / 1000.0);
    // What the colour is: relative speed, or (--nasa-cloud) vorticity, the local spin of the air, blue low to magenta high, or (--surface-pressure) the simulated air's pressure
    // beside the simulated solids, painted on the drawn parts, blue below the still air's pressure to red above (the lead's words, O124). Whichever, no figures.
    const std::string colour =
        pressure ? "Colour shows pressure on the surfaces (" + std::string(parts_words) + "), blue below the air's ambient pressure to red above, centred on the pressure of the still air, "
                   "and the picture carries no pressure or speed figures. The pressure is the simulated air's, read beside the simulated solids (the parts as solids at " + cell + " mm) and painted on the "
                   "drawn parts: not a surface-resolved pressure, first order in the cell size, showing where the push and the suction are and not validated magnitudes. " +
                   (blades ? "The rotors are " + rotors + " and their blades are " + std::string(blade_words) + ", drawn at the simulation's rotor angle. The blade colour is the air pressure the rotor model makes "
                             "beside the blades, spread over about 5 mm by the model's force kernel, not a pressure resolved on the blade surface. "
                           : "The rotors are " + rotors + ", which have no surface, so they carry no pressure colour. ") +
                   "Parts inside the airframe's body, with no air nearby in the simulation, are drawn in their material colour. " +
                   (blades ? "The colour range is fixed at +-" + std::format("{:g}", static_cast<double>(kSurfacePressureCpRange)) + " of the disc loading on the parts and at +-" +
                                 std::format("{:g}", static_cast<double>(blade_range)) + " of the disc loading on the blades, each fixed, symmetric about the still air's pressure, and the same in every frame"
                           : "The colour range is fixed at +-" + std::format("{:g}", static_cast<double>(kSurfacePressureCpRange)) + " of the disc loading, symmetric about the still air's pressure"
                             ", the same in every frame") +
                   (shaded ? "; the surfaces are lit for shape; the colour is the pressure. " : ". ")
        : cloud ? "Colour shows vorticity (the local spin of the air), blue low to magenta high, and the picture carries no vorticity or speed figures. "
                : "Colour shows relative speed, not validated speed, and the picture carries no speed figures. ";
    std::string t;
    if (c.scene == Scene::quad) {
        t += "Stand-in: the reference quad is four actuator discs (uniform thrust and swirl), with no airframe, in "
             "still air; the run is inviscid, so the shear layers break down to the grid scale. ";
        t += colour;
        t += "The wake is a single run on the device of record, not a measurement.";
    } else if (c.scene == Scene::d1_still) {
        t += "Stand-in: the reference quad's rigid parts as solids in the flow at " + cell + " mm, its rotors as " + rotors + ", the drone held level at its hover throttle in still air. This is a "
             "visualisation of the air, not a prediction of the thrust, power or attitude of a real quad. ";
        t += "The wake below the quad has not been measured against a downwash record. ";
        t += colour;
        t += "The fluid is stepped by the GPU solver (GpuFluid) at " + step_ms + " ms with a Smagorinsky subgrid viscosity "
             "(Cs 0.17), which damps the grid-scale motion the " + cell + " mm cells cannot hold: a single run on the device of "
             "record, not a measurement.";
    } else {
        t += "Stand-in: the reference quad's rigid parts as solids in the flow at " + cell + " mm, its rotors as " + rotors + ", the drone held level at its still-air hover throttle in a steady wind from "
             "the side (crosswind setting supported but not validated). The wind bends and carries the wake and "
             "flows round the airframe's parts, but the rotors do not respond to it and the held drone does not "
             "need to compensate. In plain words: the rotor model's push does not respond to the side wind, and the "
             "grid carries the wake downwind. This is a visualisation of the air, not a prediction of the thrust, power or "
             "attitude a real quad would need in a crosswind. ";
        t += c.quantity == Quantity::cp
                 ? "Pressure is a coefficient against the inflow's pressure, read one cell off each wall (first "
                   "order in the cell size); its sign on the arms' tops has not been measured. "
                 : "The wake below the quad has not been measured against a downwash record. ";
        t += colour;
        t += "The fluid is stepped by the GPU solver (GpuFluid) at " + step_ms + " ms with a Smagorinsky subgrid viscosity "
             "(Cs 0.17), which damps the grid-scale motion the " + cell + " mm cells cannot hold: a single run on the device of "
             "record, not a measurement.";
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

// A clip's title in the drone's words (the caption's heading and the manifest's description).
[[nodiscard]] inline std::string clip_title(const Clip& c, const Drone& d) { return in_words(std::string(c.title), d); }

// The .md's heading: the clip's title, except an S8 whose look is the pressure or the vorticity cloud (the same flags as shows_line), which names what it shows
// and never says "volume render" (the lead, O124: the pressure master's .md carried the S8 fallback title).
[[nodiscard]] inline std::string look_title(const Clip& c, const Drone& d, bool particles, bool cloud, bool pressure, bool blades = false) {
    if (c.id == "S8" && !particles) {
        if (pressure && blades) return in_words("Reference quad in hover: the air pressure on the airframe and beside the rotor blades", d);
        if (pressure) return in_words("Reference quad in hover: the air pressure on the airframe in the rotor wake", d);
        if (cloud) return in_words("Reference quad in hover: the vorticity cloud of the rotor wake", d);
    }
    return clip_title(c, d);
}

// What a film shows, in its own words: a --particles S8 shows air points (and the speed volume only when --volume put it under them), a --lines film's rotors
// are actuator lines; every other film keeps its clip's text.
[[nodiscard]] inline std::string shows_line(const Clip& c, bool lines, bool particles, bool volume, bool cloud = false, bool pressure = false, bool blades = false) {
    const std::string rotors = lines ? "actuator lines" : "actuator discs";
    if (pressure && blades && !particles && c.id == "S8") {  // with --prop-blur mode=blades the rotor blades are drawn too, and take the air's pressure beside them
        return "The air pressure on the surfaces of the reference quad's rigid parts and on its rotor blades, drawn at the simulation's rotor angle, blue (below ambient) to red (above), with four " + rotors +
               " driving the air, in a slow orbit.";
    }
    if (pressure && !particles && c.id == "S8") {  // --surface-pressure: the air's pressure on the surfaces, nothing else drawn of the air
        return "The air pressure on the surfaces of the reference quad's rigid parts, blue (below ambient) to red (above), with four " + rotors + " driving the air, in a slow orbit.";
    }
    if (cloud && !particles && c.id == "S8") {  // --nasa-cloud: the vorticity as a soft cloud
        return "The vorticity of the air round the reference quad's rigid parts and four " + rotors + ", as a soft cloud from blue (low) to magenta (high), in a slow orbit.";
    }
    if (particles && c.id == "S8") {
        return "Air points carried by the flow round the reference quad's rigid parts and four " + rotors +
               (volume ? ", over the relative-speed volume of the wake" : "") + ", in a slow orbit.";
    }
    std::string t(c.shows);
    if (lines) {
        for (std::size_t at = t.find("actuator discs"); at != std::string::npos; at = t.find("actuator discs", at + 14u)) {
            t.replace(at, 14u, "actuator lines");
        }
    }
    return t;
}

struct CaptionInput {
    const Clip* clip = nullptr;
    const Drone* drone = nullptr;  // null: the reference quad
    std::string commit;
    std::string date_utc;
    std::string command;
    std::string ticket;
    double sim_seconds = 0.0;  // simulated time shown, in the case's own units
    double screen_seconds = 0.0;
    std::string extra;  // anything else the run's log carries (the device, the grid, a legend's note)
    float cell_mm = 4.0f;       // the D1 box's cell, for the standing caveats (--cell-mm; today's 4 mm)
    uint32_t step_us = 200u;    // the solver step in microseconds, for the same words (--step-us; today's 0.2 ms)
    bool lines = false;         // --lines: the rotors are actuator lines with the fitted blade polar, not discs
    bool particles = false;     // --particles / --particles-preview: the film shows air points carried by the flow
    bool volume = false;        // --volume: ... over the speed volume (without it the points stand alone and no volume is claimed)
    bool cloud = false;         // --nasa-cloud: the film shows the vorticity as a soft cloud, and its colour is vorticity, not relative speed
    bool pressure = false;      // --surface-pressure: the film shows the air's pressure on the surfaces, and its colour is pressure, not relative speed
    bool shaded = false;        // --surface-shade: ... lit for shape
    bool blades = false;        // --prop-blur mode=blades: ... and the rotors' real blades, drawn at the simulation's angle, take the air's pressure beside them on their own range
};

// The .md beside a clip: its title, commit, date, how it was rendered, what it shows, the slow motion, the loop
// and the stand-in. It never claims more than the footage shows.
[[nodiscard]] inline std::string caption_text(const CaptionInput& in) {
    const Clip& c = *in.clip;
    const Drone& drone = in.drone != nullptr ? *in.drone : default_drone();
    std::string t = "# " + look_title(c, drone, in.particles, in.cloud, in.pressure, in.blades) + "\n\n";
    t += "- Commit: " + in.commit + "\n";
    t += "- Date: " + in.date_utc + "\n";
    t += "- How rendered: tools/flow_movie, `" + command_in_words(in.command, drone) + "`, job ticket " + in.ticket + "\n";
    t += "- Shows: " + in_words(shows_line(c, in.lines, in.particles, in.volume, in.cloud, in.pressure, in.blades), drone) + "\n";
    if (!drone.description.empty()) t += "- Drone: " + std::string(drone.description) + "\n";
    t += "- " + slow_motion_line(in.sim_seconds, in.screen_seconds) + "\n";
    t += "- Loop: " + loop_line(c) + "\n";
    t += "- " + in_words(standing_caveats(c, in.cell_mm, in.step_us, in.lines, in.cloud, in.pressure, in.shaded, in.blades, drone.blade_cp_range, drone.blade_words, drone.parts_words), drone) + "\n";
    if (!drone.card_extra.empty()) t += "- " + in_words(std::string(drone.card_extra), drone) + "\n";
    if (!in.extra.empty()) t += "- " + in.extra + "\n";
    return t;
}

// The perf stopwatch (L74, Test/Docs' PERF.md): the seconds each phase of one frame of a web clip took. `frame` is the
// whole loop body (every phase but the encode's tail); `render` is the GL draw's submit and the GPU's time to finish it,
// `readback` the copy of the picture to memory, `flip` the top-row-first reorder and the alpha fill, `box_down` the
// 2x supersample's filter down to the frame, `pipe` the write of the frame into ffmpeg (it stalls when the encoder is
// behind), `cache_read` the frame's read from the cache, `streamline_shot` the view's streamlines and parts.
struct FrameTimes {
    double frame = 0.0, cache_read = 0.0, streamline_shot = 0.0, gl_upload = 0.0, render = 0.0, readback = 0.0,
           flip = 0.0, box_down = 0.0, pipe = 0.0;
};

// The middle value (an even count: the mean of the middle two), 0 for none: a first frame's one-off costs do not move it.
[[nodiscard]] inline double median_of(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t mid = v.size() / 2u;
    return v.size() % 2u == 1u ? v[mid] : 0.5 * (v[mid - 1u] + v[mid]);
}

// One figure of the ledger: `PERF <name> <value> <unit>`, the value to four places.
[[nodiscard]] inline std::string perf_line(const std::string& name, double value, const std::string& unit) {
    char text[64];
    std::snprintf(text, sizeof text, "%.4f", value);
    return "PERF " + name + " " + text + " " + unit;
}

// Where a web run's frames came from, for the ledger's note: the cache's folder, its frames and its bytes.
[[nodiscard]] inline std::string web_cache_note(const std::string& cache, uint64_t frames, uint64_t bytes) {
    return "PERF-NOTE web_cache " + cache + " " + std::to_string(frames) + " frames " + std::to_string(bytes) + " bytes";
}

// What a web run prints: the frame's median first (`rendering.web_frame_4k`), then each phase's median
// (`rendering.diag_web_<phase>`), then the encode's tail after the last frame (the clip's finish) shared over its frames.
// Nothing for a run of no frames.
[[nodiscard]] inline std::vector<std::string> web_perf_lines(const std::vector<FrameTimes>& frames, double encode_tail_s) {
    std::vector<std::string> lines;
    if (frames.empty()) return lines;
    const auto median = [&](double FrameTimes::*field) {
        std::vector<double> v;
        v.reserve(frames.size());
        for (const FrameTimes& f : frames) v.push_back(f.*field);
        return median_of(std::move(v));
    };
    lines.push_back(perf_line("rendering.web_frame_4k", median(&FrameTimes::frame), "s"));
    lines.push_back(perf_line("rendering.diag_web_cache_read", median(&FrameTimes::cache_read), "s"));
    lines.push_back(perf_line("rendering.diag_web_streamline_shot", median(&FrameTimes::streamline_shot), "s"));
    lines.push_back(perf_line("rendering.diag_web_gl_upload", median(&FrameTimes::gl_upload), "s"));
    lines.push_back(perf_line("rendering.diag_web_render", median(&FrameTimes::render), "s"));
    lines.push_back(perf_line("rendering.diag_web_readback", median(&FrameTimes::readback), "s"));
    lines.push_back(perf_line("rendering.diag_web_flip", median(&FrameTimes::flip), "s"));
    lines.push_back(perf_line("rendering.diag_web_box_down", median(&FrameTimes::box_down), "s"));
    lines.push_back(perf_line("rendering.diag_web_pipe", median(&FrameTimes::pipe), "s"));
    lines.push_back(perf_line("rendering.diag_web_encode_tail", encode_tail_s / static_cast<double>(frames.size()), "s"));
    return lines;
}

// A scene whose background is pure black (#000000): no sky, so nothing but the flow and the parts is drawn.
inline void blacken(spade::render::RenderScene& scene) {
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
}

}  // namespace flow_movie::web
