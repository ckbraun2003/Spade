// See live_tour.hpp.
//
// TIME IS COUNTED IN FRAMES. The window reports a fixed 1/60 s per frame
// (GlTargetSink::set_fixed_delta), so the tour steps the drone and moves the
// cameras the same way on any machine, however long a frame really takes.
// The video takes every second frame at 30 fps, so it plays in real time.

#include "live_tour.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "analysis/case.hpp"
#include "analysis/live_run.hpp"
#include "core/path_text.hpp"
#include "render_parts/render_parts.hpp"
#include "editor_session.hpp"
#include "pilot_keys.hpp"
#include "end_goal_checks.hpp"
#include "gl_target_sink.hpp"
#include "png_reader.hpp"
#include "png_writer.hpp"
#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "scene_sessions.hpp"
#include "sim/design_frame.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/part_pose.hpp"
#include "vehicles/propulsion_steady.hpp"
#include "world/aabb.hpp"
#include "world/mesh_file.hpp"
#include "world/mesh_import.hpp"

#ifdef _WIN32
#define SPADE_POPEN _popen
#define SPADE_PCLOSE _pclose
#define SPADE_POPEN_WRITE "wb"
#define SPADE_POPEN_READ "rb"
#else
#define SPADE_POPEN popen
#define SPADE_PCLOSE pclose
#define SPADE_POPEN_WRITE "w"
#define SPADE_POPEN_READ "r"
#endif

namespace spade::sandbox::live {
namespace {

namespace fs = std::filesystem;

constexpr float kDt = 1.0f / 60.0f;
constexpr uint64_t kVideoEvery = 2;      // 60 frames a second rendered, 30 recorded
constexpr uint32_t kVideoFps = 30;
constexpr uint64_t kFallbackEvery = 15;  // without ffmpeg, a PNG every 15th recorded frame (2 a second)
// The watching stretches (frames with no scripted input) run this many times
// longer than their counts in the steps below, so a person can follow the
// tour. Scripted input is not paced: its counts set how far a key moves the
// camera or the drone, which the checks rely on.
constexpr int kIdlePace = 2;
constexpr int kOpeningFrames = 90;   // paced: 3 s
constexpr int kClosingFrames = 120;  // paced: 4 s
constexpr int kChapterCardFrames = 60;  // paced: 2 s

// THE LEDGER. Every sub-step the tour runs, in order. A step listed here that
// never runs fails the run; a step the tour runs that is not listed is an
// anomaly.
struct StepName {
    std::string_view main;
    std::string_view sub;
};
// Three chapters (INT-9, the storyboard plans/2026-10-13-live-tour-storyboard.md):
// the drone box, its airflow, and the editor.
constexpr std::array kSteps = {
    StepName{"drone_box", "open"},       StepName{"drone_box", "orbit"},
    StepName{"drone_box", "attitude"},   StepName{"drone_box", "level"},
    StepName{"drone_box", "throttle"},   StepName{"drone_box", "hud"},
    StepName{"airflow", "live"},
    StepName{"editor", "open"},          StepName{"editor", "picker"},
    StepName{"editor", "select"},        StepName{"editor", "place"},
    StepName{"editor", "move"},          StepName{"editor", "inspect"},
    StepName{"editor", "undo"},          StepName{"editor", "run"},
    StepName{"editor", "world"},         StepName{"editor", "conflict"},
    StepName{"editor", "views"},         StepName{"editor", "save"},
    StepName{"editor", "library"},       StepName{"editor", "add_drone"},
    StepName{"editor", "airflow"},       StepName{"editor", "add_objects"},
    StepName{"editor", "fly"},
};

// The extra stills a run writes beside the posters (Tour::snapshot), declared
// so that a later run into the same folder owns them.
constexpr std::array<std::string_view, 12> kSnapshots = {"editor_picker",   "editor_vehicle",  "editor_place",
                                                         "editor_gizmo",    "editor_inspect",  "editor_running",
                                                         "editor_world",    "editor_conflict", "editor_objects",
                                                         "editor_part",     "editor_drone",    "editor_airflow"};

// The main functions, in tour order. Each has one poster, numbered by its
// place here, so a poster's name never depends on which steps ran.
constexpr std::array<std::string_view, 3> kMains = {"drone_box", "airflow", "editor"};

// Tolerated anomalies, each with why. An entry here must fire on every run:
// one that stops firing fails the run (it has expired, so delete it).
// airflow/not-available went when the chapter went live on the library's
// reference quad (N4).
// - editor.add_drone/placed-at (the lead, 2026-10-22): its stillness half.
//   The rest half is the step's hard "rests" check.
const std::vector<KnownOpen> kKnownOpen = {
    {"editor.add_drone/placed-at",
     "ground contact is the model's one proxy sphere (reference quad 0.150 m, its rotor reach); resting on the "
     "lowest posed part buries it 106 mm, so Run lifts the drone. Physics owns the fix: contact against the posed "
     "parts."},
};

[[nodiscard]] std::string fmt(float v) {
    char text[32];
    std::snprintf(text, sizeof text, "%.3f", static_cast<double>(v));
    return text;
}

// N15's tour crate (editor.add_objects): a closed box mesh named `name`,
// `side` m, standing on y = 0 about its origin, as a .gltf whose buffer is a
// .bin beside it (read_gltf() reads a relative buffer file). Built here, so
// assets/ gains nothing. True when both are written.
[[nodiscard]] bool write_box_gltf(const fs::path& gltf, std::string_view name, float side) {
    static constexpr uint32_t kTriangles[12][3] = {
        {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
        {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
    };
    std::string bin;
    for (uint32_t v = 0; v < 8; ++v) {
        const float p[3] = {side * (static_cast<float>(v & 1u) - 0.5f), side * static_cast<float>((v >> 1) & 1u),
                            side * (static_cast<float>((v >> 2) & 1u) - 0.5f)};
        bin.append(reinterpret_cast<const char*>(p), sizeof p);
    }
    for (const auto& t : kTriangles) bin.append(reinterpret_cast<const char*>(t), sizeof t);
    const auto g = [](float v) {
        char text[32];
        std::snprintf(text, sizeof text, "%.9g", static_cast<double>(v));
        return std::string(text);
    };
    const fs::path bin_file = fs::path(gltf).replace_extension(".bin");
    const std::string doc =
        "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"mesh\":0}],"
        "\"meshes\":[{\"name\":\"" + std::string(name) + "\",\"primitives\":[{\"attributes\":{\"POSITION\":0},"
        "\"indices\":1}]}],\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":8,\"type\":\"VEC3\","
        "\"min\":[" + g(-0.5f * side) + ",0," + g(-0.5f * side) + "],\"max\":[" + g(0.5f * side) + "," + g(side) +
        "," + g(0.5f * side) + "]},{\"bufferView\":1,\"componentType\":5125,\"count\":36,\"type\":\"SCALAR\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":96},{\"buffer\":0,\"byteOffset\":96,"
        "\"byteLength\":144}],\"buffers\":[{\"byteLength\":" + std::to_string(bin.size()) + ",\"uri\":\"" +
        path_text(bin_file.filename()) + "\"}]}";
    std::ofstream b(bin_file, std::ios::binary);
    b.write(bin.data(), static_cast<std::streamsize>(bin.size()));
    std::ofstream j(gltf, std::ios::binary);
    j.write(doc.data(), static_cast<std::streamsize>(doc.size()));
    return b.good() && j.good();
}

[[nodiscard]] float mean_rotor_omega(const DroneReadouts& r) {
    float sum = 0.0f;
    for (const float w : r.rotor_omega) {
        sum += w;
    }
    return sum / static_cast<float>(r.rotor_omega.size());
}

// True when ffmpeg answers on the PATH.
[[nodiscard]] bool ffmpeg_available() {
    std::FILE* p = SPADE_POPEN("ffmpeg -hide_banner -version", SPADE_POPEN_READ);
    if (p == nullptr) {
        return false;
    }
    char buf[256];
    while (std::fread(buf, 1, sizeof buf, p) > 0) {
    }
    return SPADE_PCLOSE(p) == 0;
}

// ---------------------------------------------------------------------------
// The recording: frames read back from the window, streamed raw to ffmpeg, or
// kept as PNG stills when ffmpeg is missing; and one poster per main function.
// ---------------------------------------------------------------------------
class Recorder {
  public:
    Recorder(fs::path out_dir, Smoke& smoke) : out_(std::move(out_dir)), smoke_(smoke) {}
    ~Recorder() { finish(); }
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    void start(uint32_t width, uint32_t height) {
        width_ = width;
        height_ = height;
        if (!ffmpeg_available()) {
            smoke_.anomaly("recording/ffmpeg-missing",
                           "ffmpeg did not answer on the PATH, so there is no tour.mp4; a PNG still is kept "
                           "twice a second instead. Install ffmpeg or put it on the PATH");
            return;
        }
        const fs::path mp4 = out_ / "tour.mp4";
        const std::string cmd = ffmpeg_command("ffmpeg", mp4, width, height, kVideoFps);
        pipe_ = SPADE_POPEN(cmd.c_str(), SPADE_POPEN_WRITE);
        if (pipe_ == nullptr) {
            smoke_.anomaly("recording/ffmpeg-start", "ffmpeg is on the PATH but did not start: " + cmd);
            return;
        }
        piped_ = true;
        smoke_.note("recording " + mp4.string() + " at " + std::to_string(kVideoFps) + " fps, " +
                    std::to_string(width) + "x" + std::to_string(height));
    }

    void want_video_frame() noexcept { video_pending_ = true; }
    void want_poster(fs::path path) { poster_ = std::move(path); }
    // The next captured frame, kept for a step to judge (rows bottom-up).
    void want_grab() noexcept {
        grab_pending_ = true;
        grab_.clear();  // a frame that is not captured leaves nothing, never the last one
        grab_w_ = grab_h_ = 0;
    }
    [[nodiscard]] const std::vector<uint8_t>& grabbed() const noexcept { return grab_; }
    [[nodiscard]] uint32_t grabbed_width() const noexcept { return grab_w_; }
    [[nodiscard]] uint32_t grabbed_height() const noexcept { return grab_h_; }
    [[nodiscard]] bool wants_capture() const noexcept { return video_pending_ || !poster_.empty() || grab_pending_; }

    // The window's frame tap: rows bottom-up, as GL reads them.
    void on_frame(const uint8_t* rgba, uint32_t w, uint32_t h) {
        if (grab_pending_) {
            grab_.assign(rgba, rgba + static_cast<std::size_t>(w) * h * 4u);
            grab_w_ = w;
            grab_h_ = h;
            grab_pending_ = false;
        }
        if (!poster_.empty()) {
            if (!png::write_file(poster_, png::encode_rgba(rgba, w, h, /*bottom_up=*/true))) {
                smoke_.anomaly("recording/poster", "could not write " + poster_.string());
            } else {
                smoke_.note("poster " + poster_.string());
            }
            poster_.clear();
        }
        if (!video_pending_) {
            return;
        }
        video_pending_ = false;
        if (w != width_ || h != height_) {
            if (!size_reported_) {
                smoke_.anomaly("recording/size-changed", "the window became " + std::to_string(w) + "x" +
                                                             std::to_string(h) + "; frames of another size are "
                                                             "left out of the video");
                size_reported_ = true;
            }
            return;
        }
        if (pipe_ != nullptr) {
            const std::size_t n = static_cast<std::size_t>(w) * h * 4u;
            if (std::fwrite(rgba, 1, n, pipe_) != n && !write_failed_) {
                smoke_.anomaly("recording/ffmpeg-write", "a frame did not reach ffmpeg; tour.mp4 is incomplete");
                write_failed_ = true;
            }
        } else if (video_frames_ % kFallbackEvery == 0) {
            if (stills_ < kMaxStills) {
                const fs::path still = out_ / still_name(stills_++);
                if (!png::write_file(still, png::encode_rgba(rgba, w, h, true))) {
                    smoke_.anomaly("recording/still", "could not write " + still.string());
                }
            } else if (!stills_capped_) {
                smoke_.note("stills stop at " + std::to_string(kMaxStills) + ", the names a run owns");
                stills_capped_ = true;
            }
        }
        ++video_frames_;
    }

    // Closes ffmpeg and reports how it ended. Safe to call twice.
    void finish() {
        if (pipe_ == nullptr) {
            return;
        }
        const int status = SPADE_PCLOSE(pipe_);
        pipe_ = nullptr;
        if (status != 0) {
            smoke_.anomaly("recording/ffmpeg-exit",
                           "ffmpeg exited with status " + std::to_string(status) + "; tour.mp4 may be unplayable");
        } else {
            mp4_written_ = true;
        }
    }

    [[nodiscard]] bool mp4_written() const noexcept { return mp4_written_; }
    [[nodiscard]] bool piped() const noexcept { return piped_; }
    [[nodiscard]] uint64_t video_frames() const noexcept { return video_frames_; }

  private:
    fs::path out_;
    Smoke& smoke_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    std::FILE* pipe_ = nullptr;
    bool video_pending_ = false;
    fs::path poster_;
    bool grab_pending_ = false;
    std::vector<uint8_t> grab_;
    uint32_t grab_w_ = 0;
    uint32_t grab_h_ = 0;
    uint64_t video_frames_ = 0;
    uint32_t stills_ = 0;
    bool stills_capped_ = false;
    bool piped_ = false;
    bool size_reported_ = false;
    bool write_failed_ = false;
    bool mp4_written_ = false;
};

// ---------------------------------------------------------------------------
// The tour's frame pump and its step runner.
// ---------------------------------------------------------------------------
using InputFn = std::function<FrameInput(int frame)>;
using DrawFn = std::function<Result<void>(const FrameInput&)>;

class Tour {
  public:
    Tour(const TourOptions& options, GlTargetSink& sink, Smoke& smoke, Recorder& recorder)
        : options_(options), sink_(sink), smoke_(smoke), rec_(recorder) {
        for (const Injection& inj : options.injections) {
            if (inj.kind == Injection::Kind::skip) {
                skip_.push_back(inj.target);
            }
        }
    }

    // Runs one declared step around `body`, unless the run has stopped or an
    // injection skips it. A skipped step never begins, so the ledger fails it.
    void step(std::string_view main, std::string_view sub, const std::function<void()>& body) {
        const std::string name = std::string(main) + "." + std::string(sub);
        if (stopped_) {
            return;
        }
        if (std::find(skip_.begin(), skip_.end(), name) != skip_.end()) {
            smoke_.note("skipped by --inject: " + name);
            return;
        }
        ++index_;
        smoke_.begin(name);
        sink_.set_caption(std::string(main) + "  -  " + std::string(sub) + "    (" + std::to_string(index_) + "/" +
                          std::to_string(kSteps.size()) + ")");
        body();
        if (!stopped_) {
            smoke_.done();
        }
    }

    // Renders `frames` frames through `draw`, feeding it `input(i)`. The OS
    // events are still pumped every frame so the window stays responsive, but
    // the script drives, not the mouse. False when the window was closed or a
    // frame failed; the tour then stops and the ledger reports what never ran.
    bool pump(int frames, const InputFn& input, const DrawFn& draw) {
        if (!input) {
            frames *= kIdlePace;
        }
        for (int i = 0; i < frames && !stopped_; ++i) {
            const FrameInput real = sink_.poll();
            if (real.want_close) {
                smoke_.anomaly("harness/window-closed", "the window was closed during " +
                                                            (smoke_.current_step().empty() ? std::string("the tour")
                                                                                           : smoke_.current_step()));
                stopped_ = true;
                break;
            }
            const FrameInput in = input ? input(i) : FrameInput{};
            if (frame_ % kVideoEvery == 0) {
                rec_.want_video_frame();
            }
            if (rec_.wants_capture()) {
                sink_.capture_next_present();
            }
            if (const Result<void> drew = draw(in); !drew) {
                smoke_.anomaly("harness/frame-failed", drew.error().context);
                stopped_ = true;
                break;
            }
            ++frame_;
        }
        return !stopped_;
    }

    // The next captured frame becomes this main function's poster.
    void poster(std::string_view main) {
        const auto it = std::find(kMains.begin(), kMains.end(), main);
        if (it == kMains.end()) {
            smoke_.anomaly("harness/poster-undeclared", main);
            return;
        }
        rec_.want_poster(options_.out_dir /
                         poster_name(static_cast<uint32_t>(std::distance(kMains.begin(), it)) + 1u, main));
    }

    // An extra still beside the posters, <out>/<name>.png: the next captured
    // frame. The editor's panels each get one for the user to review (§13).
    void snapshot(std::string_view name) {
        if (std::find(kSnapshots.begin(), kSnapshots.end(), name) == kSnapshots.end()) {
            smoke_.anomaly("harness/snapshot-undeclared", name);
            return;
        }
        rec_.want_poster(options_.out_dir / (std::string(name) + ".png"));
    }
    [[nodiscard]] const fs::path& out_dir() const noexcept { return options_.out_dir; }

    [[nodiscard]] bool stopped() const noexcept { return stopped_; }
    [[nodiscard]] uint64_t frames() const noexcept { return frame_; }
    [[nodiscard]] Smoke& smoke() noexcept { return smoke_; }
    [[nodiscard]] GlTargetSink& sink() noexcept { return sink_; }
    [[nodiscard]] Recorder& recorder() noexcept { return rec_; }

  private:
    const TourOptions& options_;
    GlTargetSink& sink_;
    Smoke& smoke_;
    Recorder& rec_;
    std::vector<std::string> skip_;
    bool stopped_ = false;
    uint64_t frame_ = 0;
    uint32_t index_ = 0;
};

// A frame with nothing behind the overlay: the title cards.
[[nodiscard]] DrawFn blank_frame(GlTargetSink& sink) {
    return [&sink](const FrameInput&) -> Result<void> {
        sink.begin_gpu_frame();
        sink.present_overlay(0.0f, 0.0f);
        return {};
    };
}

// ---------------------------------------------------------------------------
// MAIN FUNCTION 1: THE DRONE SIM BOX.
// ---------------------------------------------------------------------------
void drone_box(Tour& t, DroneSession& d) {
    Smoke& s = t.smoke();
    const DrawFn draw = [&d](const FrameInput& in) { return d.frame(in, kDt); };
    const auto hold = [](FrameInput in) { return [in](int) { return in; }; };
    const auto idle = InputFn{};

    t.step("drone_box", "open", [&] {
        t.pump(90, idle, draw);
        s.check(d.gpu_active(), "gpu-path", d.panel().render_path);
        s.check(d.panel().readouts.tick > 0u, "stepping", "tick " + std::to_string(d.panel().readouts.tick));
        // §13: the scene renders only beside the Drone panel, not under it.
        const ViewportRect v = t.sink().viewport();
        const uint32_t whole = t.sink().framebuffer_width();
        s.check(v.x + v.width < whole && !v.empty(), "view-beside-panel",
                "view " + std::to_string(v.width) + "x" + std::to_string(v.height) + " of " + std::to_string(whole));
    });

    t.step("drone_box", "orbit", [&] {
        OrbitCamera& cam = d.camera();
        const float yaw0 = cam.yaw;
        FrameInput around;
        around.move_right = 1.0f;
        t.pump(150, hold(around), draw);
        s.check(std::abs(cam.yaw - yaw0) > 3.0f, "around", "yaw " + fmt(yaw0) + " -> " + fmt(cam.yaw));

        const float pitch0 = cam.pitch;
        FrameInput over;
        over.move_up = 1.0f;
        t.pump(45, hold(over), draw);
        s.check(cam.pitch > pitch0 + 0.3f, "over", "pitch " + fmt(pitch0) + " -> " + fmt(cam.pitch));
        FrameInput under;
        under.move_up = -1.0f;
        t.pump(45, hold(under), draw);

        FrameInput nearer;
        nearer.move_forward = 1.0f;
        t.pump(60, hold(nearer), draw);
        s.check(cam.distance <= kDroneMinDistance + 1e-4f, "nearer-clamps",
                "distance " + fmt(cam.distance) + ", minimum " + fmt(kDroneMinDistance));
        FrameInput further;
        further.move_forward = -1.0f;
        t.pump(90, hold(further), draw);
        s.check(cam.distance > 3.0f && cam.distance <= kDroneMaxDistance, "further",
                "distance " + fmt(cam.distance));

        const float yaw1 = cam.yaw;
        FrameInput drag;
        drag.orbit_dx = -4.0f;  // a mouse drag left, 4 px a frame
        t.pump(60, hold(drag), draw);
        s.check(cam.yaw > yaw1 + 1.0f, "drag", "yaw " + fmt(yaw1) + " -> " + fmt(cam.yaw));
        // Back in to the view the tour started from, so the drone fills the
        // frame for the steps that follow.
        t.pump(40, hold(nearer), draw);
        t.pump(30, idle, draw);
    });

    t.step("drone_box", "attitude", [&] {
        const auto attitude = [&d] { return attitude_from_quat(d.panel().readouts.orientation); };
        FrameInput nose;
        nose.attitude_pitch = 1.0f;
        t.pump(20, hold(nose), draw);
        t.pump(60, idle, draw);
        const float target_pitch = d.drone().target.pitch;
        const float pitch = attitude().pitch;
        s.check(target_pitch > 0.3f && pitch * target_pitch > 0.0f && std::abs(pitch) > 0.5f * std::abs(target_pitch),
                "pitch-follows", "target " + fmt(target_pitch) + ", actual " + fmt(pitch));

        FrameInput roll;
        roll.attitude_roll = 1.0f;
        t.pump(20, hold(roll), draw);
        t.pump(60, idle, draw);
        const float target_roll = d.drone().target.roll;
        const float actual_roll = attitude().roll;
        s.check(target_roll > 0.3f && actual_roll * target_roll > 0.0f &&
                    std::abs(actual_roll) > 0.5f * std::abs(target_roll),
                "roll-follows", "target " + fmt(target_roll) + ", actual " + fmt(actual_roll));

        const float yaw0 = attitude().yaw;
        FrameInput yaw;
        yaw.attitude_yaw = 1.0f;
        t.pump(40, hold(yaw), draw);
        t.pump(80, idle, draw);
        const float yaw1 = attitude().yaw;
        s.check(std::abs(yaw1 - yaw0) > 0.5f, "yaw-follows",
                "yaw " + fmt(yaw0) + " -> " + fmt(yaw1) + ", target " + fmt(d.drone().target.yaw));
    });

    t.step("drone_box", "level", [&] {
        FrameInput r;
        r.level_pressed = true;
        t.pump(1, hold(r), draw);
        s.check(d.drone().target.pitch == 0.0f && d.drone().target.roll == 0.0f, "target-levels",
                "target pitch " + fmt(d.drone().target.pitch) + ", roll " + fmt(d.drone().target.roll));
        t.pump(150, idle, draw);
        const AttitudeTarget a = attitude_from_quat(d.panel().readouts.orientation);
        s.check(std::abs(a.pitch) < 0.087f && std::abs(a.roll) < 0.087f, "drone-levels",
                "pitch " + fmt(a.pitch) + ", roll " + fmt(a.roll) + " (within 5 degrees)");
    });

    t.step("drone_box", "throttle", [&] {
        const float omega0 = mean_rotor_omega(d.panel().readouts);
        d.panel().edited.throttle = 1.4f;
        t.pump(120, idle, draw);
        t.poster("drone_box");
        const float omega1 = mean_rotor_omega(d.panel().readouts);
        s.check(omega1 > omega0 * 1.1f, "rotors-speed-up",
                "mean rotor speed " + fmt(omega0) + " -> " + fmt(omega1) + " rad/s");
        d.panel().edited.throttle = 1.0f;
        t.pump(60, idle, draw);
    });

    // F1's own path: poll() calls toggle_help() on the key's press. The checks
    // read whether the presented frames drew the HUD's stats line, not the
    // flag (the key legend is a folded section, §13).
    t.step("drone_box", "hud", [&] {
        const auto drawn = [&t] { return std::string(t.sink().help_drawn() ? "drawn" : "not drawn"); };
        const bool before = t.sink().help_drawn();
        t.sink().toggle_help();
        t.pump(60, idle, draw);
        s.check(before && !t.sink().help_drawn(), "hidden",
                "HUD stats line " + std::string(before ? "drawn" : "not drawn") + " -> " + drawn());
        t.sink().toggle_help();
        t.pump(60, idle, draw);
        s.check(t.sink().help_drawn(), "shown", "HUD stats line " + drawn());
    });
}

// ---------------------------------------------------------------------------
// MAIN FUNCTION 2: THE AIRFLOW (the storyboard's chapter 2). The library's
// reference quad (N4) flown live at 32^3 through LiveRun under the showcase
// program, its vortex cores, streamlines and a speed slice drawn over it each
// frame (AirflowSession; airflow_live.hpp decides). Test/Docs' live floor,
// downwash-present, judges the hover segment on screen.
// ---------------------------------------------------------------------------
void airflow_chapter(Tour& t, const fs::path& assets, const DrawFn& blank) {
    Smoke& s = t.smoke();
    t.step("airflow", "live", [&] {
        const fs::path model_file = assets / "library" / "drones" / "reference_quad.model.yaml";
        Result<std::unique_ptr<AirflowSession>> made = AirflowSession::create(model_file);
        if (!s.check(made.has_value(), "starts", made ? (*made)->run().run_label() : made.error().context)) return;
        AirflowSession& a = **made;
        a.attach(t.sink());

        // What stands in for what, before the run: the rotors are actuator
        // discs, the parts are drawn but not in the air.
        t.sink().set_card("Airflow, live\n" + wrap_card_line(a.run().run_label(), 70) +
                          "\nthe rotors act on the air as discs; the drone's parts are drawn, not simulated"
                          "\nshown at 1/" +
                          std::to_string(static_cast<int>(1.0f / airflow::kPlayback + 0.5f)) + " speed");
        t.pump(45, {}, blank);
        t.sink().set_card({});

        uint32_t refused = 0;
        std::string first_refusal;
        bool poster_asked = false;
        const DrawFn draw = [&](const FrameInput& in) -> Result<void> {
            if (Result<void> f = a.frame(in, kDt); !f) return f;
            if (!a.view_refusal().empty()) {
                if (refused++ == 0) first_refusal = a.view_refusal();
            }
            return {};
        };
        // The poster near the hover's end, the wake developed.
        const double armed = airflow::armed_at(a.quad().program).value_or(0.0);
        while (!a.finished() && !t.stopped()) {
            if (!poster_asked && a.run().state().t_s >= armed + 0.9) {
                t.poster("airflow");
                poster_asked = true;
            }
            t.pump(1, {}, draw);
        }
        if (t.stopped()) return;
        s.check(a.finished(), "ran", a.run().run_line());
        s.check(refused == 0, "views-drawn",
                refused == 0 ? std::to_string(a.legends().size()) + " legends on the last frame"
                             : std::to_string(refused) + " frames drawn shaded: " + first_refusal);

        // Test/Docs' live floor over the frames whose controls are the hover's.
        end_goal::DownwashCase c;
        c.density = a.quad().resolved.spec.fluid.density;
        c.mass = a.quad().run_model.body.mass;
        const end_goal::Verdict verdict = end_goal::downwash(a.downwash_frames(), c, end_goal::kDownwashPresent);
        s.check(verdict.ok, "downwash-present", verdict.detail);
        t.pump(30, {}, draw);  // the last frame held, under the verdict

        // Then the wake's slice on its own, faced.
        a.show_slice();
        const uint32_t before = refused;
        t.pump(60, {}, draw);
        s.check(refused == before, "slice-drawn",
                refused == before ? "speed through the farthest hubs, " + std::to_string(a.legends().size()) + " legend"
                                  : "drawn shaded: " + a.view_refusal());
    });
}

// ---------------------------------------------------------------------------
// MAIN FUNCTION 3: THE EDITOR (editor plan Task 10). The editor's window on the
// hover scene: the default layout, a vehicle selected, a box placed from the
// library, the world's settings, and a run. The stills are §13's review set.
// Edits stay in memory: nothing is saved into assets/.
// ---------------------------------------------------------------------------
void editor_tour(Tour& t, const fs::path& assets) {
    Smoke& s = t.smoke();
    std::unique_ptr<EditorSession> session;
    const DrawFn draw = [&session, &t](const FrameInput& in) { return session->frame(in, t.sink().delta_seconds()); };
    const auto idle = InputFn{};
    const auto open = [&]() -> bool { return s.check(session != nullptr, "open", "the editor is open"); };
    const auto w = [&]() -> editor::EditorWorkspace& { return session->workspace(); };
    const auto scene_now = [&]() -> const scene::SceneDesc& { return session->workspace().documents().scene.desc(); };
    const auto text_of = [](const scene::SceneDesc& d) {
        const Result<std::string> t = scene::scene_to_yaml(d);
        return t ? *t : std::string("<refused: ") + t.error().context + ">";
    };
    // The panels' queue is applied by the session's next frame, as a click would be.
    const auto then = [&](int frames) { t.pump(frames, idle, draw); };
    const auto command = [&](editor::Command c) {
        w().panels().commands.push_back(c);
        then(5);
    };
    // A click in the view along the ray from the camera's eye to `at`.
    const auto click_towards = [&](glm::vec3 at) {
        const glm::vec3 eye = session->camera().to_render_camera().position;
        w().click(eye, at - eye);
    };
    const auto arm = [&](std::string_view item) {
        for (std::size_t i = 0; i < w().panels().library.size(); ++i) {
            if (w().panels().library[i].name == item) w().panels().armed = i;
        }
    };
    const auto model_of = [&](std::string_view vehicle) -> const vehicles::ModelType* {
        const scene::SceneVehicle* v = editor::EditorWorkspace::find_vehicle(scene_now(), vehicle);
        if (v == nullptr) return nullptr;
        for (const vehicles::ModelType& m : scene_now().models) {
            if (m.name == v->model) return &m;
        }
        return nullptr;
    };
    // THE WORK COPY. Saving writes the world's own file, so the editor works on
    // a copy of assets/scenes and assets/worlds in the run's folder, never on
    // assets/; the copy is removed when the section ends.
    const fs::path work = t.out_dir() / "editor_work";
    std::string drone;

    t.step("editor", "open", [&] {
        std::error_code ec;
        fs::remove_all(work, ec);
        for (const char* dir : {"scenes", "worlds"}) {
            fs::create_directories(work / dir, ec);
            for (fs::directory_iterator it(assets / dir, ec), end; !ec && it != end; it.increment(ec)) {
                if (it->is_regular_file()) {
                    fs::copy_file(it->path(), work / dir / it->path().filename(), fs::copy_options::overwrite_existing,
                                  ec);
                }
            }
        }
        // The library too: the workspace reads it beside the scenes (INT-14).
        if (!ec) {
            fs::copy(assets / "library", work / "library",
                     fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
        }
        if (!s.check(!ec, "work-copy", ec ? ec.message() : "assets/scenes, assets/worlds and assets/library copied")) {
            return;
        }
        const fs::path file = work / "scenes" / "hover.scene.yaml";
        Result<editor::OpenedScene> opened = editor::open_scene(file);
        if (!s.check(opened.has_value(), "opens", opened ? file.filename().string() : opened.error().context)) return;
        Result<std::unique_ptr<EditorSession>> made = EditorSession::create(std::move(*opened));
        if (!s.check(made.has_value(), "composes", made ? "hover" : made.error().context)) return;
        session = std::move(*made);
        session->attach(t.sink());
        drone = scene_now().vehicles.at(0).name;
        then(60);
        s.check(session->gpu_active(), "gpu-path", session->gpu_active() ? "GPU (OpenGL)" : "CPU raster");
        // §13's default layout: the toolbar above, the hierarchy and the
        // inspector at the sides, the library below, and the scene between.
        const ViewportRect v = t.sink().viewport();
        const uint32_t fw = t.sink().framebuffer_width();
        const uint32_t fh = t.sink().framebuffer_height();
        s.check(v.x > 0u && v.y > 0u && v.x + v.width < fw && v.y + v.height < fh && !v.empty(),
                "view-between-panels",
                "view " + std::to_string(v.width) + "x" + std::to_string(v.height) + " at (" + std::to_string(v.x) +
                    ", " + std::to_string(v.y) + ") of " + std::to_string(fw) + "x" + std::to_string(fh));
        t.poster("editor");
        then(30);
    });

    t.step("editor", "picker", [&] {
        if (!open()) return;
        const std::size_t n = w().panels().sources.size();
        s.check(n > 1u, "lists-scenes", std::to_string(n) + " scenes in the File menu");
        w().panels().file_menu = true;
        then(40);
        t.snapshot("editor_picker");
        then(20);
        w().panels().file_menu = false;
        then(10);
    });

    t.step("editor", "select", [&] {
        if (!open()) return;
        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        click_towards(at);
        s.check(w().panels().selection == editor::Selection{editor::Selection::Kind::object, drone}, "selects-vehicle",
                "selected '" + w().panels().selection.name + "'");
        // F frames it ("Frame selected"), from wherever the camera was.
        session->camera().target = glm::vec3(0.0f);
        session->camera().distance = 12.0f;
        then(20);
        t.pump(1, [](int) { FrameInput in; in.frame_pressed = true; return in; }, draw);
        const float off = glm::length(session->camera().target - at);
        s.check(off < 1e-4f && session->camera().distance < 2.0f, "frames-selection",
                "camera " + fmt(off) + " m off the drone, " + fmt(session->camera().distance) + " m away");
        then(30);
        t.snapshot("editor_vehicle");
        then(30);
        session->camera().distance = 5.0f;  // back out, so the boxes placed next are in view
    });

    t.step("editor", "place", [&] {
        if (!open()) return;
        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        arm("Box");
        click_towards(glm::vec3(at.x + 1.0f, 0.0f, at.z));
        arm("Box");
        click_towards(glm::vec3(at.x - 1.0f, 0.0f, at.z));
        std::string names;
        for (const scene::SceneAsset& a : scene_now().assets) names += (names.empty() ? "" : ", ") + a.name;
        s.check(scene_now().assets.size() == 2u && scene_now().assets[0].name != scene_now().assets[1].name,
                "unique-names", names);
        then(30);
        t.snapshot("editor_place");
        then(20);
    });

    // The gizmo: the box's x arm dragged as a hand drags it (over the arm,
    // press, move, release), one undo step.
    t.step("editor", "move", [&] {
        if (!open() || scene_now().assets.empty()) return;
        const scene::SceneAsset box = scene_now().assets[0];
        w().panels().selection = editor::Selection{editor::Selection::Kind::object, box.name};
        w().panels().tool = editor::GizmoTool::move;
        then(10);
        const ViewportRect v = t.sink().viewport();
        const render::Camera cam = session->camera().to_render_camera();
        const glm::vec3 x(1.0f, 0.0f, 0.0f);
        const float arm = editor::gizmo_size(cam, box.pose.position);
        const std::optional<glm::vec2> from = editor::project(cam, box.pose.position + 0.6f * arm * x, v.width, v.height);
        const std::optional<glm::vec2> to =
            editor::project(cam, box.pose.position + (0.6f * arm + 1.0f) * x, v.width, v.height);
        if (!s.check(from.has_value() && to.has_value(), "handle-in-view", "the x arm and 1 m past it are in view")) {
            return;
        }
        const auto mouse = [](glm::vec2 at, bool click, bool down, bool release) {
            return [=](int) {
                FrameInput in;
                in.mouse_x = at.x;
                in.mouse_y = at.y;
                in.left_click = click;
                in.left_down = down;
                in.left_release = release;
                return in;
            };
        };
        t.pump(5, mouse(*from, false, false, false), draw);  // over the arm: it lights
        t.pump(1, mouse(*from, true, true, false), draw);    // press
        constexpr int kSteps = 30;
        for (int i = 1; i <= kSteps; ++i) {
            const float f = static_cast<float>(i) / static_cast<float>(kSteps);
            t.pump(1, mouse(*from + (*to - *from) * f, false, true, false), draw);
            if (i == kSteps / 2) t.snapshot("editor_gizmo");
        }
        t.pump(1, mouse(*to, false, false, true), draw);  // release
        const scene::SceneAsset* now = editor::EditorWorkspace::find_asset(scene_now(), box.name);
        const glm::vec3 d = now != nullptr ? now->pose.position - box.pose.position : glm::vec3(0.0f);
        s.check(now != nullptr && std::abs(d.x - 1.0f) < 0.05f && d.y == 0.0f && d.z == 0.0f, "drags-along-x",
                "moved " + fmt(d.x) + " m along x, " + fmt(d.y) + " in y, " + fmt(d.z) + " in z");
        command(editor::Command::undo);
        const scene::SceneAsset* back = editor::EditorWorkspace::find_asset(scene_now(), box.name);
        s.check(back != nullptr && back->pose.position == box.pose.position, "one-undo",
                "one undo puts the whole drag back");
        command(editor::Command::redo);
        then(20);
    });

    // A parameter edit reaches its vehicle alone (copy on edit, Task 3).
    t.step("editor", "inspect", [&] {
        if (!open()) return;
        w().panels().pending.push_back(editor::DuplicateObject{drone});
        then(5);
        const std::string twin = scene_now().vehicles.back().name;
        const vehicles::ModelType* mine = model_of(drone);
        if (!s.check(mine != nullptr && twin != drone, "duplicated", "'" + twin + "'")) return;
        const float before = mine->rotors.at(0).radius;
        w().panels().selection = editor::Selection{editor::Selection::Kind::object, drone};
        w().panels().pending.push_back(
            editor::SetModelParam{drone, vehicles::ModelIssue::Element::rotor, 0, "radius", before + 0.02f});
        then(5);
        const vehicles::ModelType* edited = model_of(drone);
        const vehicles::ModelType* other = model_of(twin);
        s.check(edited != nullptr && std::abs(edited->rotors.at(0).radius - (before + 0.02f)) < 1e-6f, "edited",
                "rotor 0 radius " + fmt(before) + " -> " + fmt(edited != nullptr ? edited->rotors.at(0).radius : 0.0f));
        s.check(other != nullptr && other->rotors.at(0).radius == before, "other-unchanged",
                "'" + twin + "' keeps " + fmt(other != nullptr ? other->rotors.at(0).radius : 0.0f));
        then(30);
        t.snapshot("editor_inspect");
        then(20);
    });

    t.step("editor", "undo", [&] {
        if (!open()) return;
        const std::string before = text_of(scene_now());
        command(editor::Command::undo);
        const bool changed = text_of(scene_now()) != before;
        command(editor::Command::redo);
        s.check(changed && text_of(scene_now()) == before, "bytes-equal",
                changed ? "undo changed it; redo restored every byte" : "undo changed nothing");
        then(20);
    });

    // An edit while running rebuilds the run, carrying each vehicle's state.
    t.step("editor", "run", [&] {
        if (!open()) return;
        w().panels().selection = editor::Selection{};
        command(editor::Command::run);
        then(90);
        const editor::EditorRun* run = w().run();
        if (!s.check(run != nullptr && run->tick() > 0u, "runs",
                     "tick " + std::to_string(run != nullptr ? run->tick() : 0u))) {
            return;
        }
        const uint64_t tick = run->tick();
        const glm::vec3 before = run->state_of(drone).value_or(FrameState{}).pos;
        // Queued, then applied as the session's next frame applies it, before it steps.
        SdfPose moved = scene_now().assets.at(0).pose;
        moved.position.x += 0.5f;
        w().panels().pending.push_back(editor::SetAssetPose{scene_now().assets.at(0).name, moved});
        w().apply_panels();
        run = w().run();
        const glm::vec3 after = run != nullptr ? run->state_of(drone).value_or(FrameState{}).pos : glm::vec3(0.0f);
        s.check(run != nullptr && run->tick() == tick && glm::length(after - before) < 1e-5f, "carries-pose",
                "tick " + std::to_string(tick) + ", drone moved " + fmt(glm::length(after - before)) + " m in the rebuild");
        then(60);
        t.snapshot("editor_running");
        then(20);
        command(editor::Command::stop);
    });

    // A world edit, saved: the world first, then the scene re-pinned to it.
    t.step("editor", "world", [&] {
        if (!open()) return;
        w().panels().selection = w().panels().hierarchy.at(1).target;  // what the World row does
        Environment windy = w().documents().world.desc().environment;
        windy.wind = glm::vec3(2.0f, 0.0f, 0.0f);
        w().panels().pending_world.push_back(editor::SetEnvironment{windy});
        then(10);
        command(editor::Command::save);
        const Result<uint64_t> hash = scene::world_hash(w().documents().world.desc());
        const bool pinned = hash.has_value() && scene_now().world.hash == *hash;
        s.check(pinned && !w().documents().scene.dirty() && !w().documents().world.dirty(), "re-pinned",
                pinned ? "the scene pins the saved world; both clean" : "status: " + w().panels().status);
        then(20);
        t.snapshot("editor_world");
        then(20);
    });

    // The scene follows its world: a world edit that no longer composes with
    // the scene is shown, by name, and undo resolves it.
    t.step("editor", "conflict", [&] {
        if (!open()) return;
        MaterialDesc red;
        red.name = "red";
        w().panels().pending.push_back(editor::AddMaterial{red});
        w().panels().pending_world.push_back(editor::AddWorldMaterial{red});
        then(10);
        const std::string status = w().panels().status;
        s.check(status.find("'red'") != std::string::npos, "shown", status.empty() ? "no status" : status);
        then(20);
        t.snapshot("editor_conflict");
        then(20);
        command(editor::Command::undo);
        s.check(w().panels().status.empty(), "undo-resolves", "status after undo: '" + w().panels().status + "'");
    });

    // N19: every mode in the View menu, chosen as a click on it chooses, each
    // frame judged by Test/Docs' modes_drawn() (end_goal_checks.hpp): drawn,
    // honoured, not blank and changed from the last mode drawn; a kind this
    // view can't draw shows supports()' why_not, which the menu draws beside it.
    t.step("editor", "views", [&] {
        if (!open()) return;
        const std::vector<editor::ModeEntry> menu = w().panels().modes;  // one snapshot: supported and why_not
        if (!s.check(!menu.empty(), "menu", std::to_string(menu.size()) + " modes in the View menu")) return;
        // The HUD's stats box sits inside the view and changes every frame:
        // hidden for the step, so the grabbed view holds only the scene.
        const bool hud = t.sink().help_drawn();
        if (hud) t.sink().toggle_help();
        then(5);
        // Each mode's whole frames, three per mode (the view, again, and its
        // background); the ModeShown views point into them, so no reallocation.
        std::vector<std::vector<uint8_t>> frames;
        frames.reserve(3u * menu.size());
        std::vector<end_goal::ModeShown> shown;
        bool framed = true;
        // Grabs the next frame and cuts the view out of it, or fails the step:
        // a frame that was not captured, or a view not inside it.
        const auto grab_view = [&](const std::string& what, end_goal::PixelsView& into) -> bool {
            t.recorder().want_grab();
            then(1);
            frames.push_back(t.recorder().grabbed());
            const uint32_t fw = t.recorder().grabbed_width();
            const uint32_t fh = t.recorder().grabbed_height();
            const ViewportRect v = t.sink().viewport();
            if (frames.back().empty()) {
                s.check(false, "grabbed", "absent: no frame captured for " + what);
                return false;
            }
            if (v.x + v.width > fw || v.y + v.height > fh) {
                s.check(false, "view-in-frame", "the view " + std::to_string(v.width) + "x" + std::to_string(v.height) +
                                                    " at (" + std::to_string(v.x) + ", " + std::to_string(v.y) +
                                                    ") is not inside the " + std::to_string(fw) + "x" +
                                                    std::to_string(fh) + " frame (" + what + ")");
                return false;
            }
            // The frame's rows run bottom-up; the viewport's y runs down from the top.
            into = end_goal::region_of(end_goal::pixels(fw, fh, frames.back()), v.x, fh - v.y - v.height, v.width,
                                       v.height);
            return true;
        };
        const std::string refused_prefix = "this view can't draw ";
        for (std::size_t i = 0; i < menu.size() && framed; ++i) {
            w().panels().status.clear();
            w().panels().mode_request = menu[i].base;
            then(15);
            end_goal::ModeShown m;
            m.kind = menu[i].name;
            m.supported = menu[i].drawable;
            m.why_not = menu[i].why_not;
            if (!menu[i].drawable) {
                // The request's refusal, as the status shows it, without its
                // "this view can't draw <kind>: " lead: choose_mode()'s path,
                // judged against the menu's supports().
                const std::string& line = w().panels().status;
                const std::string lead = refused_prefix + menu[i].name + ": ";
                m.shown_text = line.rfind(lead, 0) == 0 ? line.substr(lead.size()) : line;
            }
            // The view; the same view two frames later, with no request between
            // (the still premise); and its background, the items hidden.
            if (!grab_view(menu[i].name, m.frame)) {
                framed = false;
                break;
            }
            // What the judged frame did with the mode (Test/Docs' nit: the
            // facts of the frame grabbed, not of the one before it).
            m.applied = session->last_view().applied && session->last_view().base == menu[i].base;
            m.drawn = session->last_view().drawn;
            m.unhonoured = session->last_view().unhonoured;
            then(2);
            if (!grab_view(menu[i].name + ", again", m.again)) {
                framed = false;
                break;
            }
            session->set_background_only(true);
            then(2);
            const bool background = grab_view(menu[i].name + "'s background", m.background);
            session->set_background_only(false);
            then(2);
            if (!background) {
                framed = false;
                break;
            }
            shown.push_back(std::move(m));
        }
        if (framed) {
            const end_goal::Verdict verdict = end_goal::modes_drawn(shown);
            s.check(verdict.ok, "each-mode-drawn", verdict.detail);
        }
        w().panels().mode_request = render::ViewBase::shaded;
        if (hud) t.sink().toggle_help();
        then(20);
    });

    t.step("editor", "save", [&] {
        if (!open()) return;
        std::error_code ec;
        fs::create_directories(work / "saved", ec);
        const fs::path copy = work / "saved" / "hover_copy.scene.yaml";
        const auto bytes_of = [](const fs::path& f) {
            std::ifstream in(f, std::ios::binary);
            return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        };
        w().panels().path = copy.string();
        command(editor::Command::save_as);
        const std::string mine = text_of(scene_now());
        const Result<editor::OpenedScene> back = editor::open_scene(copy);
        const bool equal = back.has_value() && text_of(back->scene.desc()) == mine &&
                           bytes_of(copy) == mine;
        s.check(equal, "byte-equal",
                back ? (equal ? "saved, reopened, every byte equal" : "differs") : back.error().context);
        then(20);
    });

    // N14a (INT-14): the Library panel lists the default library's index, in
    // its order (Test/Docs' lists-index). N14b: each entry's tile shows its own
    // thumbnail file's image (thumbnails).
    t.step("editor", "library", [&] {
        if (!open()) return;
        const Result<scene::Library> index = scene::load_library(assets / "library" / "library.yaml");
        if (!s.check(index.has_value(), "index", index ? "read" : index.error().context)) return;
        std::vector<end_goal::LibraryEntry> want;
        for (const scene::LibraryEntry& e : index->entries) want.push_back({e.name, e.category, e.file});
        std::vector<end_goal::LibraryEntry> shown;
        for (const editor::LibraryItem& i : w().panels().library) shown.push_back({i.name, i.category, i.file});
        const end_goal::Verdict listed = end_goal::lists_index(want, shown);
        s.check(listed.ok, "lists-index", listed.detail);

        // thumbnails: each file decoded here, never the panel's copy. The
        // panel scrolls, so the step brings the first tile not yet seen into
        // view, grabs the frame, and cuts out every tile the panel reports
        // wholly visible, until each is seen or a scroll shows nothing new.
        std::vector<png::Image> files;
        for (const scene::LibraryEntry& e : index->entries) {
            Result<png::Image> f = png::read_file(index->dir / e.thumbnail);
            if (!f) {
                s.check(false, "thumbnails", "thumbnails: absent: '" + e.name + "': " + f.error().context);
                return;
            }
            files.push_back(std::move(*f));
        }
        std::vector<std::vector<uint8_t>> cut(files.size());  // each tile as shown, top row first
        std::vector<ViewportRect> where(files.size());
        std::vector<bool> seen(files.size(), false);
        for (std::size_t page = 0; page < files.size(); ++page) {
            const auto next = std::find(seen.begin(), seen.end(), false);
            if (next == seen.end()) break;
            w().panels().library_scroll_to = static_cast<std::size_t>(next - seen.begin());
            then(3);  // the scroll lands, then a frame draws the tiles there
            t.recorder().want_grab();
            then(1);
            const std::vector<uint8_t>& frame = t.recorder().grabbed();
            const uint32_t fw = t.recorder().grabbed_width();
            const uint32_t fh = t.recorder().grabbed_height();
            if (frame.empty()) {
                s.check(false, "grabbed", "absent: no frame captured for the library's tiles");
                return;
            }
            const std::vector<std::optional<ViewportRect>>& tiles = w().panels().library_tiles;
            bool any = false;
            for (std::size_t i = 0; i < files.size() && i < tiles.size(); ++i) {
                const std::optional<ViewportRect>& r = tiles[i];
                if (seen[i] || !r || r->empty() || r->x + r->width > fw || r->y + r->height > fh) continue;
                const std::size_t row_bytes = static_cast<std::size_t>(r->width) * 4u;
                cut[i].resize(row_bytes * r->height);
                for (uint32_t row = 0; row < r->height; ++row) {
                    // The frame's rows run bottom-up; the tile's y runs down from the top.
                    const std::size_t from =
                        (static_cast<std::size_t>(fh - 1u - (r->y + row)) * fw + r->x) * 4u;
                    std::copy_n(frame.begin() + static_cast<std::ptrdiff_t>(from), row_bytes,
                                cut[i].begin() + static_cast<std::ptrdiff_t>(row * row_bytes));
                }
                where[i] = *r;
                seen[i] = true;
                any = true;
            }
            if (!any) break;
        }
        w().panels().library_scroll_to = 0;
        std::vector<end_goal::PixelsView> shown_tiles;
        std::vector<end_goal::PixelsView> decoded;
        for (std::size_t i = 0; i < files.size(); ++i) {
            decoded.push_back(end_goal::PixelsView{files[i].width, files[i].height,
                                                   static_cast<std::size_t>(files[i].width) * 4u, files[i].rgba});
            shown_tiles.push_back(seen[i] ? end_goal::PixelsView{where[i].width, where[i].height,
                                                                 static_cast<std::size_t>(where[i].width) * 4u, cut[i]}
                                          : end_goal::PixelsView{});
        }
        const end_goal::Verdict thumbs = end_goal::thumbnails(shown_tiles, decoded);
        s.check(thumbs.ok, "thumbnails", thumbs.detail);

        // INT-14's grades: the first part with a field assumed is inspected
        // as its tile's click inspects it (inspect_item()) and drawn: the
        // inspector shows it in place of the selection (part-inspected), and
        // each tile is marked assumed exactly when its part has a field
        // assumed, as the panel drew the marks (assumed-marked).
        std::optional<std::size_t> part;
        for (std::size_t i = 0; i < w().panels().library.size() && !part; ++i) {
            const editor::LibraryItem& candidate = w().panels().library[i];
            if (candidate.kind == editor::LibraryItem::Kind::part && editor::any_assumed(candidate)) part = i;
        }
        if (!part) {
            s.check(false, "part-inspected", "absent: no library part has a field assumed");
            return;
        }
        const editor::LibraryItem item = w().panels().library[*part];
        w().panels().selection = editor::Selection{editor::Selection::Kind::scene, ""};
        editor::inspect_item(w().panels(), *part);
        w().panels().library_scroll_to = *part;
        then(3);
        const Result<std::vector<editor::PartRow>> rows = editor::part_rows(*item.part);
        std::size_t assumed_rows = 0;
        if (rows) {
            for (const editor::PartRow& r : *rows) assumed_rows += r.grade == scene::Grade::assumed ? 1u : 0u;
        }
        const bool in_inspector =
            w().panels().inspected == part && w().panels().selection.kind == editor::Selection::Kind::none;
        s.check(in_inspector && rows.has_value(), "part-inspected",
                "'" + item.name + "': " +
                    (rows ? std::to_string(rows->size()) + " fields, " + std::to_string(assumed_rows) + " assumed"
                          : rows.error().context) +
                    (in_inspector ? "; the inspector shows it in place of the selection" : "; NOT shown"));
        t.snapshot("editor_part");
        then(1);  // the still is the next frame drawn: it draws while the part is inspected
        const std::vector<editor::LibraryItem>& lib = w().panels().library;
        const std::vector<bool>& badges = w().panels().library_badges;
        bool match = badges.size() == lib.size();
        std::size_t marked = 0;
        std::string wrong;
        for (std::size_t i = 0; match && i < lib.size(); ++i) {
            const bool want = editor::any_assumed(lib[i]);
            if (badges[i]) ++marked;
            if (badges[i] != want) {
                match = false;
                wrong = "'" + lib[i].name + "' is " + (badges[i] ? "marked" : "not marked") +
                        (want ? " with a field assumed" : " with none assumed");
            }
        }
        s.check(match, "assumed-marked",
                match ? std::to_string(marked) + " of " + std::to_string(lib.size()) +
                            " tiles marked assumed, each a part or drone with a grade assumed"
                      : (wrong.empty() ? std::to_string(badges.size()) + " marks drawn for " +
                                             std::to_string(lib.size()) + " tiles"
                                       : wrong));

        // L5's sidecars (the lead's Q2): the first library drone with an
        // input assumed is inspected as its tile's click inspects it, its
        // inputs' grades in place of the selection (drone-inspected).
        std::optional<std::size_t> drone;
        for (std::size_t i = 0; i < lib.size() && !drone; ++i) {
            if (lib[i].kind == editor::LibraryItem::Kind::model && editor::any_assumed(lib[i])) drone = i;
        }
        if (!drone) {
            s.check(false, "drone-inspected", "absent: no library drone has an input assumed in its sources");
        } else {
            const editor::LibraryItem d = lib[*drone];
            w().panels().selection = editor::Selection{editor::Selection::Kind::scene, ""};
            editor::inspect_item(w().panels(), *drone);
            w().panels().library_scroll_to = *drone;
            then(3);
            const std::vector<editor::PartRow> rows = editor::model_rows(*d.sources);
            const auto assumed_inputs = std::count_if(rows.begin(), rows.end(), [](const editor::PartRow& r) {
                return r.grade == scene::Grade::assumed;
            });
            const bool drone_shown =
                w().panels().inspected == drone && w().panels().selection.kind == editor::Selection::Kind::none;
            s.check(drone_shown && !rows.empty(), "drone-inspected",
                    "'" + d.name + "': " + std::to_string(rows.size()) + " inputs, " +
                        std::to_string(assumed_inputs) + " assumed, built from " +
                        std::to_string(d.sources->parts.size()) + " part files" +
                        (drone_shown ? "; the inspector shows it in place of the selection" : "; NOT shown"));
            t.snapshot("editor_drone");
            then(1);  // the still is the next frame drawn: it draws while the drone is inspected
        }
        w().panels().inspected.reset();
        w().panels().library_scroll_to = 0;
        then(10);
    });

    // N14a: the library's drone, placed by a click, rests on its own lowest
    // part and stays still for 1.0 s of Run (placed-at), and keeps its model,
    // parts and propulsion identities (propulsion-kept). Undone after.
    t.step("editor", "add_drone", [&] {
        if (!open()) return;
        const Result<scene::ModelFile> file =
            scene::load_model_file(assets / "library" / "drones" / "reference_quad.model.yaml");
        if (!s.check(file.has_value(), "file", file ? "read" : file.error().context)) return;
        const std::size_t before = scene_now().vehicles.size();
        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        const glm::vec3 target(at.x, 0.0f, at.z + 1.5f);  // clear of the boxes placed at x +-1
        const glm::vec3 eye = session->camera().to_render_camera().position;
        const glm::vec3 dir = glm::normalize(target - eye);
        const std::optional<float> hit = editor::detail::march(w().composed().world.sdf, eye, dir);
        arm("Reference quad (test fixture)");
        click_towards(target);
        if (!s.check(hit.has_value() && scene_now().vehicles.size() == before + 1, "placed",
                     hit ? w().panels().status : "the click's ray met no surface")) {
            return;
        }
        const scene::SceneVehicle placed = scene_now().vehicles.back();
        const vehicles::ModelType* model = model_of(placed.name);
        if (!s.check(model != nullptr && model->propulsion.has_value() && file->model.propulsion.has_value(),
                     "has-chain", model == nullptr ? "no model" : placed.model)) {
            return;
        }
        const std::vector<end_goal::Identity> ids{
            {"model", vehicles::model_identity(file->model), vehicles::model_identity(*model)},
            {"parts", vehicles::parts_identity(file->model), vehicles::parts_identity(*model)},
            {"propulsion", vehicles::propulsion_identity(*file->model.propulsion),
             vehicles::propulsion_identity(*model->propulsion)}};
        const end_goal::Verdict kept = end_goal::identities_kept("propulsion-kept", ids, end_goal::kDroneIdentities);
        s.check(kept.ok, "propulsion-kept", kept.detail);

        // The rest: its lowest posed part over the surface beneath, by the
        // scene's SDF (the drone is not in it).
        end_goal::Placement p;
        p.pick = eye + dir * *hit;
        p.position = placed.start.pos;
        // The start pose is the design frame's; the body is the sim's
        // to_body_state() of it (design_to_principal, com_offset).
        const FrameState spawn =
            to_body_state(FrameState{placed.start.pos, placed.start.orient, glm::vec3(0.0f), glm::vec3(0.0f)},
                          glm::normalize(model->design_to_principal), model->com_offset);
        BodyState body{};
        body.pos = spawn.pos;
        body.orient = spawn.orient;
        const std::vector<float> still(model->rotors.size(), 0.0f);
        const Result<vehicles::PosedParts> posed =
            vehicles::posed_parts(*model, body, still, still, vehicles::PartFilter::all);
        float low = std::numeric_limits<float>::infinity();
        if (posed) {
            for (const vehicles::PosedPart& part : posed->parts) {
                const Result<Aabb> box = vehicles::part_bounds(model->parts[part.part]);
                if (!box) continue;
                for (uint32_t c = 0; c < 8; ++c) {
                    const glm::vec3 corner((c & 1u) != 0 ? box->max.x : box->min.x,
                                           (c & 2u) != 0 ? box->max.y : box->min.y,
                                           (c & 4u) != 0 ? box->max.z : box->min.z);
                    low = std::min(low, (part.origin + part.orient * corner).y);
                }
            }
        }
        const glm::vec3 above(placed.start.pos.x, placed.start.pos.y + 5.0f, placed.start.pos.z);
        const std::optional<float> down =
            editor::detail::march(w().composed().world.sdf, above, glm::vec3(0.0f, -1.0f, 0.0f));
        p.rest_gap = down ? low - (above.y - *down) : std::numeric_limits<float>::quiet_NaN();
        // placed-at's rest half, a hard check by its own limits: where asked,
        // resting on its own lowest part. Its stillness half is known-open.
        const float dx = std::abs(p.position.x - p.pick.x);
        const float dz = std::abs(p.position.z - p.pick.z);
        s.check(dx <= end_goal::kPlaceXZ && dz <= end_goal::kPlaceXZ && p.rest_gap >= end_goal::kRestLo &&
                    p.rest_gap <= end_goal::kRestHi,
                "rests",
                fmt(std::max(dx, dz) * 1000.0f) + " mm from the pick; its lowest part " + fmt(p.rest_gap * 1000.0f) +
                    " mm above the surface (within [-2, +5])");

        // drawn: one drawable per part (render_parts), each at its posed
        // part's transform. Measured against this step's own posing and
        // prepare_parts(), never the session's: a part counts as drawn when
        // the frame holds as many items labelled "<vehicle>/<part>" as the
        // part has pieces, each the posed part's frame times its piece's.
        then(2);  // a frame binds the placed drone
        p.drawables_declared = model->parts.size();
        const Result<render_parts::PreparedParts> prepared = render_parts::prepare_parts(*model);
        if (posed && prepared) {
            const render::RenderScene& frame = session->render_scene();
            const auto label = [&](const std::string& name) -> std::optional<uint32_t> {
                for (uint32_t i = 0; i < frame.labels.size(); ++i) {
                    if (frame.labels[i] == name) return i;
                }
                return std::nullopt;
            };
            const std::optional<uint32_t> object = label(placed.name);
            for (const vehicles::PosedPart& part : posed->parts) {
                const render_parts::PreparedPart& pieces = prepared->parts.at(part.part);
                const std::optional<uint32_t> mine = label(placed.name + "/" + pieces.name);
                std::vector<const render::DrawItem*> items;
                for (const render::DrawItem& item : frame.statics) {
                    if (object && mine && item.object_label == *object && item.part_label == *mine) items.push_back(&item);
                }
                if (items.empty() || items.size() != pieces.pieces.size()) continue;
                ++p.drawables_added;
                const glm::mat4 world_from_part = glm::translate(glm::mat4(1.0f), part.origin) * glm::mat4_cast(part.orient);
                for (std::size_t k = 0; k < items.size(); ++k) {
                    const glm::mat4 want = world_from_part * pieces.pieces[k].part_from_piece;
                    const glm::mat4& got = items[k]->local_to_world;
                    p.drawable_offset = std::max(p.drawable_offset, glm::length(glm::vec3(got[3]) - glm::vec3(want[3])));
                    for (int c = 0; c < 3; ++c) {
                        const glm::vec3 a = glm::normalize(glm::vec3(got[c]));
                        const glm::vec3 b = glm::normalize(glm::vec3(want[c]));
                        p.drawable_turn = std::max(p.drawable_turn, std::asin(std::min(1.0f, glm::length(glm::cross(a, b)))));
                    }
                }
            }
        }
        const end_goal::Verdict drawn =
            prepared ? end_goal::drawn_as_declared(p)
                     : end_goal::Verdict{false, "drawn: absent: prepare_parts() refused: " + prepared.error().context};
        s.check(drawn.ok, "drawn", drawn.detail);

        // Still: 1.0 s of Run (240 ticks at the editor's 240 Hz), disarmed.
        w().panels().selection = editor::Selection{};
        command(editor::Command::run);
        const editor::EditorRun* run = w().run();
        const uint64_t from = run != nullptr ? run->tick() : 0u;
        for (int i = 0; i < 200 && run != nullptr && run->tick() < from + 240u; ++i) {
            then(1);
            run = w().run();
        }
        const std::optional<FrameState> now = run != nullptr ? run->state_of(placed.name) : std::nullopt;
        p.moved_after_run = now ? glm::length(now->pos - placed.start.pos) : std::numeric_limits<float>::quiet_NaN();
        p.tilted_after_run = now ? end_goal::tilt_of(now->orient) : std::numeric_limits<float>::quiet_NaN();
        command(editor::Command::stop);
        const end_goal::Verdict rests = end_goal::placed_at(p);
        s.check(rests.ok, "placed-at", rests.detail);
        command(editor::Command::undo);
        then(10);
    });

    // INT-15 (T3): Run air on the library's quad, placed as add_drone places
    // it and given an airflow region, the Pilot section arming it. The
    // worker's frames reach the view, which draws the streamlines and, once
    // there are any, the vortex cores around the drone (§5's default view).
    // At 32^3 a frame of air takes about a second of wall time: the step
    // waits for kAirFrames of them (a minute at most). Stopped and undone after.
    t.step("editor", "airflow", [&] {
        if (!open()) return;
        constexpr uint64_t kAirFrames = 12;  // 0.4 s of air
        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        arm("Reference quad (test fixture)");
        click_towards(glm::vec3(at.x, 0.0f, at.z + 1.5f));  // where editor.add_drone placed it
        const std::string quad = w().panels().selection.name;
        if (!s.check(!quad.empty() && model_of(quad) != nullptr, "placed", w().panels().status)) return;
        w().panels().pending.push_back(editor::AddAirflowRegion{quad});
        w().apply_panels();
        if (!s.check(!scene_now().airflow.empty(), "region", w().panels().status)) return;
        command(editor::Command::run_air);
        if (!s.check(w().air() != nullptr, "runs", w().panels().status)) return;
        w().panels().pilot.controls = analysis::Controls{true, 0.6f, 0.0f, 0.0f, 0.0f};
        // The camera on the drone and its box, as the airflow chapter frames its
        // poster; put back after.
        const OrbitCamera before = session->camera();
        for (const scene::SceneVehicle& v : scene_now().vehicles) {
            if (v.name == quad) session->camera().target = v.start.pos - glm::vec3(0.0f, 0.1f, 0.0f);
        }
        session->camera().yaw = 0.52f;
        session->camera().pitch = 0.35f;
        session->camera().distance = 1.6f;
        std::shared_ptr<const editor::AirFrame> frame;
        for (int i = 0; i < 3600; ++i) {
            then(1);
            frame = w().air() != nullptr ? w().air()->latest() : nullptr;
            if (frame && (!frame->error.empty() || frame->frames >= kAirFrames)) break;
        }
        s.check(frame && frame->error.empty() && frame->frames >= kAirFrames, "publishes",
                !frame ? "no frame handed over" : !frame->error.empty() ? frame->error
                                                                         : std::to_string(frame->frames) + " frames");
        then(5);
        t.snapshot("editor_airflow");
        then(20);  // the snapshot is the next frame drawn: the air still running
        command(editor::Command::stop);
        session->camera() = before;
        command(editor::Command::undo);  // the region
        command(editor::Command::undo);  // the drone
        then(10);
    });

    // N15 (I5): a gate and a wall from the library, and a glTF crate written
    // here into the work folder and imported by Load from file, each placed
    // by a click on clear ground: object-added for each (Test/Docs' §3),
    // import-named for the crate. Measured from raw inputs, never through the
    // editor's own placing:
    // - the world's SDF from before the click is the world without it;
    // - an SDF object's lowest points are Test/Docs' own measure
    //   (collider_rest_points(), §7a): its collider, posed by the object's
    //   pose, marched up under its drawables' lowest vertices (a CSG object's
    //   mesh, the library gate's, sits up to half a cell inside its surface,
    //   so its vertices can't judge a rest); the crate's are its posed
    //   vertices, which lie on its surface;
    // - the editor passes scene_from_world() no labels, so the object's
    //   drawables are those labelled with the nodes and the prop it added
    //   (render/scene.hpp's defaults); a primitive's pose is the object's
    //   times its collider transform, a CSG root's mesh is in world space
    //   (the identity), the crate's the object's;
    // - each drawable's material is the object's: its collider node's, or
    //   the palette default for a collider naming none, or its visual's.
    // The objects are placed upright at scale 1. Undone after.
    t.step("editor", "add_objects", [&] {
        if (!open()) return;
        const fs::path crate = work / "crate.gltf";
        if (!s.check(write_box_gltf(crate, "crate", 0.4f), "crate-file", path_text(crate))) return;
        w().panels().item_path = path_text(crate);
        command(editor::Command::load_item);
        const bool armed = w().panels().armed.has_value();
        if (!s.check(armed, "imported",
                     armed ? "Load from file armed '" + w().panels().library.at(*w().panels().armed).name + "'"
                           : w().panels().status)) {
            return;
        }
        const std::string crate_item = w().panels().library.at(*w().panels().armed).name;
        w().panels().armed.reset();

        const auto pose_matrix = [](const SdfPose& pose) {
            return glm::translate(glm::mat4(1.0f), pose.position) * glm::mat4_cast(pose.rotation) *
                   glm::scale(glm::mat4(1.0f), glm::vec3(pose.scale));
        };
        // The largest gap and turn between two frames: their origins, and
        // their axes' directions.
        const auto widen = [](const glm::mat4& got, const glm::mat4& want, float& offset, float& turn) {
            offset = std::max(offset, glm::length(glm::vec3(got[3]) - glm::vec3(want[3])));
            for (int c = 0; c < 3; ++c) {
                const glm::vec3 a = glm::normalize(glm::vec3(got[c]));
                const glm::vec3 b = glm::normalize(glm::vec3(want[c]));
                turn = std::max(turn, std::asin(std::min(1.0f, glm::length(glm::cross(a, b)))));
            }
        };
        // The object's drawables in the current frame, each with its node's
        // index in the collider, or -1 for the prop.
        struct Mine {
            render::DrawItem item;
            int node = -1;
        };
        const auto mine = [&](const WorldDesc& before, const WorldDesc& now) {
            std::vector<Mine> out;
            const render::RenderScene& frame = session->render_scene();
            for (const render::DrawItem& item : frame.statics) {
                if (item.object_label >= frame.labels.size()) continue;
                const std::string& label = frame.labels[item.object_label];
                for (std::size_t i = before.sdf.nodes.size(); i < now.sdf.nodes.size(); ++i) {
                    if (label == "sdf node " + std::to_string(i)) {
                        out.push_back(Mine{item, static_cast<int>(i - before.sdf.nodes.size())});
                    }
                }
                for (std::size_t p = before.props.size(); p < now.props.size(); ++p) {
                    if (label == "prop " + std::to_string(p) + ": " + now.props[p].mesh_ref) out.push_back(Mine{item, -1});
                }
            }
            return out;
        };
        const auto items_of = [](const std::vector<Mine>& m) {
            std::vector<render::DrawItem> out;
            for (const Mine& x : m) out.push_back(x.item);
            return out;
        };

        // Places `item` by a click towards `target` and judges it; its name,
        // or none when nothing was placed.
        const auto place_and_judge = [&](const std::string& what, const std::string& item,
                                         glm::vec3 target) -> std::optional<std::string> {
            const WorldDesc before = w().composed().world;
            end_goal::Placement p;
            p.assets_before = scene_now().assets.size();
            const glm::vec3 eye = session->camera().to_render_camera().position;
            const glm::vec3 dir = glm::normalize(target - eye);
            const std::optional<float> hit = editor::detail::march(before.sdf, eye, dir);
            arm(item);
            click_towards(target);
            then(2);  // a frame binds it
            p.assets_after = scene_now().assets.size();
            const bool placed = hit.has_value() && p.assets_after > p.assets_before;
            if (!s.check(placed, "placed",
                         what + (placed   ? " placed as '" + scene_now().assets.back().name + "'"
                                 : !hit ? ": the click met no surface"
                                        : ": nothing placed: " + w().panels().status))) {
                return std::nullopt;
            }
            const scene::SceneAsset a = scene_now().assets.back();
            p.name_unique = std::count_if(scene_now().assets.begin(), scene_now().assets.end(),
                                          [&](const scene::SceneAsset& x) { return x.name == a.name; }) == 1;
            p.pick = eye + dir * *hit;
            p.position = a.pose.position;
            const WorldDesc& now = w().composed().world;
            const std::vector<Mine> drawn = mine(before, now);
            p.drawables_added = drawn.size();
            p.drawables_declared = !a.collider.nodes.empty() ? end_goal::declared_drawables(a.collider)
                                   : a.visual.mesh_ref.empty() ? 0u
                                                               : 1u;
            const auto material = [&](const std::string& name) -> std::optional<uint32_t> {
                for (uint32_t i = 0; i < now.materials.size(); ++i) {
                    if (now.materials[i].name == name) return i;
                }
                return std::nullopt;
            };
            const glm::mat4 pose = pose_matrix(a.pose);
            p.materials_match = !drawn.empty();
            for (const Mine& m : drawn) {
                glm::mat4 want = pose;
                uint32_t got = m.item.material_override;
                bool ok = false;
                if (m.node >= 0 && static_cast<std::size_t>(m.node) < a.collider.nodes.size()) {
                    const SdfNode& n = a.collider.nodes[static_cast<std::size_t>(m.node)];
                    want = static_cast<SdfOp>(n.op) != SdfOp::none
                               ? glm::mat4(1.0f)
                               : pose * glm::inverse(a.collider.transforms.at(n.transform).world_to_local);
                    ok = a.collider_materials.empty()
                             ? (got == render::kNoMaterial || got == 0u)
                             : material(a.collider_materials.at(static_cast<std::size_t>(m.node))) == got;
                } else if (m.node < 0) {
                    ok = material(a.visual.material) == got;
                }
                p.materials_match = p.materials_match && ok;
                widen(m.item.local_to_world, want, p.drawable_offset, p.drawable_turn);
            }

            // The rest, over the world without it.
            const render::RenderScene& frame = session->render_scene();
            const std::vector<render::DrawItem> items = items_of(drawn);
            const std::vector<glm::vec3> vertices = end_goal::posed_vertices(frame, items);
            const std::vector<glm::vec3> lowest =
                a.collider.nodes.empty() ? vertices : end_goal::collider_rest_points(a.collider, pose, vertices);
            p.rest_gap = end_goal::rest_gap(before.sdf, lowest);

            // Still: 1.0 s of Run (240 ticks), its drawables judged in the
            // run's own frame against where they were drawn.
            w().panels().selection = editor::Selection{};
            command(editor::Command::run);
            const editor::EditorRun* run = w().run();
            const uint64_t start_tick = run != nullptr ? run->tick() : 0u;
            for (int i = 0; i < 200 && run != nullptr && run->tick() < start_tick + 240u; ++i) {
                then(1);
                run = w().run();
            }
            const std::vector<Mine> after = mine(before, w().composed().world);
            if (run == nullptr || run->tick() < start_tick + 240u || after.size() != drawn.size()) {
                p.moved_after_run = std::numeric_limits<float>::quiet_NaN();
            } else {
                for (std::size_t k = 0; k < after.size(); ++k) {
                    widen(after[k].item.local_to_world, drawn[k].item.local_to_world, p.moved_after_run,
                          p.tilted_after_run);
                }
            }
            command(editor::Command::stop);
            const end_goal::Verdict v = end_goal::object_added(p);
            s.check(v.ok, "object-added", what + ": " + v.detail);
            return a.name;
        };

        // The crate's name and source as shown, and its sidecar loading back
        // with what read_gltf() read.
        const auto judge_import = [&](const std::string& name) {
            end_goal::ImportShown m;
            m.shown_name = name;
            m.node_name = "";  // write_box_gltf() names the mesh, not its node
            m.mesh_name = "crate";
            m.file = path_text(crate);
            w().panels().selection = editor::Selection{editor::Selection::Kind::object, name};
            w().refresh_panels();
            m.shown_source = w().panels().selected_source;
            if (const Result<std::vector<MeshObject>> read = read_gltf(crate); read && read->size() == 1) {
                m.gltf_vertices = read->front().mesh->positions().size();
                m.gltf_triangles = read->front().mesh->triangle_count();
            }
            std::string ref;
            for (const scene::SceneAsset& a : scene_now().assets) {
                if (a.name == name) ref = a.visual.mesh_ref;
            }
            const Result<LoadedMesh> sidecar = load_mesh_file(w().import_staging() / fs::path(ref + ".spmesh"));
            m.sidecar_loaded = sidecar.has_value() && sidecar->mesh != nullptr;
            if (m.sidecar_loaded) {
                m.sidecar_vertices = sidecar->mesh->positions().size();
                m.sidecar_triangles = sidecar->mesh->triangle_count();
            }
            const end_goal::Verdict v = end_goal::import_named(m);
            s.check(v.ok, "import-named", v.detail);
            w().panels().selection = editor::Selection{};
        };

        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        struct Placing {
            std::string what;
            std::string item;
            glm::vec3 target;
        };
        const std::vector<Placing> placings{{"gate", "Gate", glm::vec3(at.x - 3.0f, 0.0f, at.z - 3.0f)},
                                            {"wall", "Wall", glm::vec3(at.x + 3.0f, 0.0f, at.z - 3.0f)},
                                            {"crate", crate_item, glm::vec3(at.x, 0.0f, at.z - 2.5f)}};
        std::size_t placed = 0;
        for (const Placing& pl : placings) {
            const std::optional<std::string> name = place_and_judge(pl.what, pl.item, pl.target);
            if (!name) break;
            ++placed;
            if (pl.what == "crate") judge_import(*name);
        }
        if (placed == placings.size()) {
            t.snapshot("editor_objects");
            then(1);  // the still is the next frame drawn: it draws before the undos
        }
        for (std::size_t i = 0; i < placed; ++i) command(editor::Command::undo);
        then(10);
    });

    // N17: the placed reference quad flown from the pilot's keys (the lead's
    // Q1 to Q3). Run pilots the selected drone, disarmed at rest; Space arms it
    // to idle; PageUp, held, throttles it to 1.3 m g, inside the band climbs
    // was simulated for (full throttle climbs past it); Up, held, tilts the
    // nose toward the keys' 30 degrees down. Judged by armed, climbs and
    // follows-attitude on the run's own frames, so none depends on the spawn
    // height (Run's pop: editor.add_drone/placed-at). follows-attitude counts
    // from the press, the setpoint's 0.5 s ramp included; the headless twin
    // is FlightController.TheToursStagingFromThePilotsKeysPassesArmedClimbsAndFollows.
    t.step("editor", "fly", [&] {
        if (!open()) return;
        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        const std::size_t before = scene_now().vehicles.size();
        arm("Reference quad (test fixture)");
        click_towards(glm::vec3(at.x, 0.0f, at.z + 1.5f));  // where editor.add_drone placed it
        if (!s.check(scene_now().vehicles.size() == before + 1, "placed", w().panels().status)) return;
        const std::string quad = scene_now().vehicles.back().name;
        const vehicles::ModelType* placed = model_of(quad);
        if (!s.check(placed != nullptr && placed->propulsion.has_value(), "has-chain", quad)) return;
        const vehicles::ModelType m = *placed;
        w().panels().selection = editor::Selection{editor::Selection::Kind::object, quad};
        command(editor::Command::run);
        const editor::PilotPanel& pilot = w().panels().pilot;
        if (!s.check(w().run() != nullptr && pilot.vehicle == quad, "piloted",
                     pilot.vehicle.empty() ? pilot.why_not : "flying " + pilot.vehicle)) {
            command(editor::Command::stop);
            command(editor::Command::undo);
            return;
        }

        const WorldDesc& world = w().documents().world.desc();
        const float density = world.environment.air_density;
        const float gravity = glm::length(world.environment.gravity);
        const auto now_s = [&] {
            const editor::EditorRun* run = w().run();
            return run != nullptr ? static_cast<double>(run->tick()) * static_cast<double>(kBuiltinStepNs) * 1e-9 : 0.0;
        };
        // `n` frames with `keys` held, each recorded as editor.fly records it:
        // the piloted drone's design-frame attitude (+X the nose) and rotors.
        std::vector<end_goal::FlightFrame> frames;
        const auto fly = [&](int n, const FrameInput& keys) -> bool {
            for (int i = 0; i < n; ++i) {
                if (!t.pump(1, [keys](int) { return keys; }, draw)) return false;
                const editor::EditorRun* run = w().run();
                const std::optional<VehicleRef> ref = run != nullptr ? run->vehicle_ref(quad) : std::nullopt;
                if (!ref) return s.check(false, "flying", "the run or the drone went away");
                const Result<const BodyState*> body = run->sim().body(ref->body);
                if (!body) return s.check(false, "flying", body.error().context);
                end_goal::FlightFrame f;
                f.t_s = now_s();
                f.height = (*body)->pos.y;
                f.velocity = (*body)->vel;
                f.attitude = (*body)->orient * m.design_to_principal;
                std::vector<end_goal::RotorProbe> probes;
                for (uint32_t r = 0; r < ref->rotor_count; ++r) {
                    const Result<const vehicles::RotorRow*> row = run->sim().rotor(*ref, r);
                    if (!row) continue;  // armed() names the frame that lacks it
                    probes.push_back(end_goal::probe_rotor(**row, **body));
                    f.rotor_omega.push_back((*row)->omega);
                }
                f.thrust_up = end_goal::vertical_thrust(probes);
                frames.push_back(std::move(f));
            }
            return true;
        };
        const int per_second = static_cast<int>(std::lround(1.0f / kDt));
        FrameInput none;
        FrameInput space;
        space.arm_pressed = true;
        FrameInput page_up;
        page_up.throttle = 1.0f;
        FrameInput up_arrow;
        up_arrow.attitude_pitch = -1.0f;  // Up: the nose down

        if (!fly(per_second * 3 / 2, none)) return;  // disarmed: Run's pop, then at rest
        const double arm_s = now_s();
        if (!fly(1, space) || !fly(per_second * 3 / 2, none)) return;
        const double throttle_up_s = now_s();
        const std::size_t until_up = frames.size();
        const double per_rotor = 1.3 * static_cast<double>(m.body.mass) * static_cast<double>(gravity) /
                                 static_cast<double>(m.rotors.size());
        const vehicles::SteadyPoint hold = vehicles::steady_state_for_thrust(*m.propulsion, per_rotor, density, 0.0,
                                                                            analysis::kDefaultStateOfCharge);
        const double idle_duty = analysis::kDefaultIdleDuty;
        const float target = static_cast<float>((hold.duty - idle_duty) / (1.0 - idle_duty));
        for (int i = 0; i < per_second * 3 && pilot.controls.throttle < target; ++i) {
            if (!fly(1, page_up)) return;
        }
        const int rest = static_cast<int>(std::lround((throttle_up_s + 3.0 - now_s()) * per_second));
        if (!fly(std::max(rest, 0), none)) return;
        const double press_s = now_s();
        if (!fly(per_second * 5 / 2, up_arrow)) return;
        command(editor::Command::stop);

        const Result<vehicles::DutyCommand> idle = vehicles::rotor_omega_for_duty(
            *m.propulsion, analysis::kDefaultIdleDuty, density, analysis::kDefaultStateOfCharge);
        const std::vector<float> idle_command(m.rotors.size(), idle ? idle->omega_cmd : 0.0f);
        const std::vector<end_goal::FlightFrame> before_up(frames.begin(),
                                                           frames.begin() + static_cast<std::ptrdiff_t>(until_up));
        const end_goal::Verdict armed = end_goal::armed(before_up, arm_s, idle_command);
        s.check(armed.ok, "armed", idle ? armed.detail : idle.error().context);
        const end_goal::Verdict climbed =
            end_goal::climbs(frames, throttle_up_s, m.body.mass, gravity, m.rotors.front().radius);
        s.check(climbed.ok, "climbs", climbed.detail);
        const end_goal::Verdict followed =
            end_goal::follows_attitude(frames, press_s, -flight::kTiltMaxDeg * 0.017453292519943295f);
        s.check(followed.ok, "follows-attitude", followed.detail);
        command(editor::Command::undo);
        then(10);
    });

    session.reset();  // detaches its panels while the window is still open
    std::error_code ec;
    fs::remove_all(work, ec);
    if (ec) s.note("the editor's work copy was not removed: " + ec.message());
}

// Clears an earlier run's files so a stale frame never poses as a new one.
// Deletes only names a run writes (plan_sweep in live_smoke.hpp); a folder
// that holds anything else is refused before anything is deleted.
[[nodiscard]] Result<void> sweep(const fs::path& dir, const std::vector<std::string>& owned) {
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        return std::unexpected(Error{Code::io_error, "cannot create " + dir.string() + ": " + ec.message()});
    }
    std::vector<std::string> entries;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        const std::u8string u8 = it->path().filename().u8string();
        std::string name(u8.begin(), u8.end());
        // Only a plain file can be a run's own; a directory or a link is not.
        std::error_code type_ec;
        if (it->symlink_status(type_ec).type() != fs::file_type::regular) {
            name += "/";
        }
        entries.push_back(std::move(name));
    }
    if (ec) {
        return std::unexpected(Error{Code::io_error, "cannot list " + dir.string() + ": " + ec.message()});
    }
    const Result<std::vector<std::string>> plan = plan_sweep(entries, owned);
    if (!plan) {
        return std::unexpected(Error{plan.error().code, "--out " + dir.string() + " " + plan.error().context});
    }
    for (const std::string& name : *plan) {
        fs::remove(dir / fs::path(std::u8string(name.begin(), name.end())), ec);
        if (ec) {
            return std::unexpected(Error{Code::io_error, "cannot remove the earlier run's " + name + " in " +
                                                             dir.string() + ": " + ec.message()});
        }
    }
    return {};
}

}  // namespace

int run_live_smoke(const TourOptions& options) {
    std::vector<std::string> declared;
    for (const StepName& st : kSteps) {
        declared.push_back(std::string(st.main) + "." + std::string(st.sub));
    }
    // Before anything is touched: an injection that could not make the run
    // red is refused, so a red run can never pass by mistake.
    if (const Result<void> valid = validate_injections(options.injections, declared, kKnownOpen); !valid) {
        std::fprintf(stderr, "spade_sandbox: live smoke: %s\n", valid.error().context.c_str());
        return 2;
    }
    std::vector<std::string> owned = owned_names(kMains);
    for (const std::string_view snap : kSnapshots) owned.push_back(std::string(snap) + ".png");
    if (const Result<void> swept = sweep(options.out_dir, owned); !swept) {
        std::fprintf(stderr, "spade_sandbox: live smoke: %s\n", swept.error().context.c_str());
        return 2;
    }
    const fs::path journal_path = options.out_dir / "journal.txt";
    std::ofstream journal(journal_path, std::ios::binary | std::ios::trunc);
    if (!journal) {
        std::fprintf(stderr, "spade_sandbox: live smoke: cannot write %s\n", journal_path.string().c_str());
        return 2;
    }
    // Each line is written and flushed the moment it happens, and echoed, so
    // a crash leaves a true prefix on disk and on the console.
    const Smoke::LineWriter write = [&journal](std::string_view line) {
        journal.write(line.data(), static_cast<std::streamsize>(line.size()));
        journal.put('\n');
        journal.flush();
        std::printf("live-smoke: %.*s\n", static_cast<int>(line.size()), line.data());
        std::fflush(stdout);
    };

    Smoke smoke(std::move(declared), kKnownOpen, write);
    smoke.note("label " + (options.label.empty() ? std::string("(none)") : options.label));
    for (const Injection& inj : options.injections) {
        if (inj.kind == Injection::Kind::anomaly) {
            smoke.anomaly(inj.target, "injected by --inject");
        }
    }

    // Declared first, so it is destroyed last: the sessions detach from it.
    std::unique_ptr<GlTargetSink> sink;
    // Both scenes are built before the window opens, so a scene that will not
    // build is journalled with no window, and the tour runs what did build.
    spade::Result<std::unique_ptr<DroneSession>> drone = DroneSession::create(options.blur);
    if (!drone) {
        smoke.anomaly("harness/drone-session", drone.error().context);
    }

    sink = open_window(options.width, options.height, /*vsync=*/true);
    if (!sink) {
        // No tour, so no verdict: the journal is left without its END line,
        // and the exit code says the run could not be judged.
        smoke.anomaly("harness/no-window", "the window could not be opened (see above)");
        smoke.note("no window, so the tour did not run and there is no verdict");
        return 2;
    }
    sink->set_fixed_delta(kDt);
    sink->set_scripted_input(true);  // no tooltip or hover from wherever the box's cursor is parked

    Recorder rec(options.out_dir, smoke);
    rec.start(sink->framebuffer_width(), sink->framebuffer_height());
    sink->set_frame_tap([&rec](const uint8_t* rgba, uint32_t w, uint32_t h) { rec.on_frame(rgba, w, h); });

    Tour tour(options, *sink, smoke, rec);
    const DrawFn blank = blank_frame(*sink);

    sink->set_card(opening_card(options.label));
    tour.pump(kOpeningFrames, {}, blank);
    sink->set_card({});

    // Each chapter opens with a card naming it (the storyboard's captions).
    const auto chapter = [&](std::string_view card) {
        sink->set_card(std::string(card));
        tour.pump(kChapterCardFrames, {}, blank);
        sink->set_card({});
    };
    if (drone && !tour.stopped()) {
        chapter("1  The drone box\nthe drone held at the centre, flown by hand:\nattitude, thrust, the camera");
        (*drone)->attach(*sink);
        drone_box(tour, **drone);
    }
    if (drone) {
        drone->reset();  // detaches its panel before the editor attaches
    }
    if (!tour.stopped()) {
        chapter("2  Airflow\nthe air around the library's quad, live at 32^3:\nvortex cores, streamlines, a speed slice");
        airflow_chapter(tour, options.assets_dir, blank);
    }
    if (!tour.stopped()) {
        chapter("3  The editor\nopen a scene, place, select, move, edit, undo, run, save");
        editor_tour(tour, options.assets_dir);
    }

    // The closing card shows the tour's verdict. Closing ffmpeg comes after it
    // and can still add an anomaly, so the journal's END line is the run's
    // verdict of record.
    sink->set_caption({});
    sink->set_card(closing_card(options.label, smoke.verdict()));
    tour.pump(kClosingFrames, {}, blank);
    sink->set_card({});
    sink->set_frame_tap({});
    rec.finish();
    smoke.note("frames " + std::to_string(tour.frames()) + ", recorded " + std::to_string(rec.video_frames()) +
               " (" + std::to_string(rec.video_frames() / kVideoFps) + " s of video)");

    // What the run leaves behind, checked rather than assumed: ffmpeg given no
    // frames still exits 0, and a poster request can go unanswered.
    (void)smoke.check(rec.video_frames() > 0u, "frames-recorded", std::to_string(rec.video_frames()) + " frames");
    // The UI font (the editor design's §13): a missing font is announced, so
    // the recording must not quietly show ImGui's built-in one instead.
    (void)smoke.check(sink->ui_font_loaded(), "ui-font",
                      sink->ui_font_loaded() ? "Inter loaded" : "the UI font did not load; see stderr");
    if (rec.piped()) {
        std::error_code ec;
        const std::uintmax_t size = fs::file_size(options.out_dir / "tour.mp4", ec);
        (void)smoke.check(!ec && size > 0u, "mp4-written",
                          ec ? "tour.mp4: " + ec.message() : "tour.mp4 " + std::to_string(size) + " bytes");
        // So whatever the script prints can be sent as it is.
        (void)smoke.check(!ec && size < kSendableBytes, "mp4-sendable",
                          "tour.mp4 " + std::to_string(size) + " bytes, the limit " + std::to_string(kSendableBytes));
    }
    for (std::size_t i = 0; i < kMains.size(); ++i) {
        const std::string name = poster_name(static_cast<uint32_t>(i) + 1u, kMains[i]);
        std::error_code ec;
        const std::uintmax_t size = fs::file_size(options.out_dir / name, ec);
        (void)smoke.check(!ec && size > 0u, "poster-" + std::string(kMains[i]),
                          ec ? name + ": " + ec.message() : name + " " + std::to_string(size) + " bytes");
    }

    const Verdict v = smoke.finish();
    std::printf("\nspade_sandbox: LIVE SMOKE %s -- anomalies %d, known-open %d, unmarked %zu, expired %zu\n",
                v.pass ? "PASSED" : "FAILED", v.anomalies, v.known_open_fired, v.unmarked.size(), v.expired.size());
    std::printf("spade_sandbox:   journal %s\n", journal_path.string().c_str());
    if (rec.mp4_written()) {
        std::printf("spade_sandbox:   video   %s\n", (options.out_dir / "tour.mp4").string().c_str());
    }
    return v.pass ? 0 : 1;
}

}  // namespace spade::sandbox::live
