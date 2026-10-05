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
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "gl_target_sink.hpp"
#include "png_writer.hpp"
#include "scene_sessions.hpp"

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

// THE LEDGER. Every sub-step the tour runs, in order. A step listed here that
// never runs fails the run; a step the tour runs that is not listed is an
// anomaly.
struct StepName {
    std::string_view main;
    std::string_view sub;
};
constexpr std::array kSteps = {
    StepName{"drone_box", "open"},       StepName{"drone_box", "orbit"},
    StepName{"drone_box", "attitude"},   StepName{"drone_box", "level"},
    StepName{"drone_box", "heatmap"},    StepName{"drone_box", "wind"},
    StepName{"drone_box", "turbulence"}, StepName{"drone_box", "throttle"},
    StepName{"drone_box", "vulkan_refused"}, StepName{"drone_box", "hud"},
    StepName{"builder", "open"},         StepName{"builder", "place"},
    StepName{"builder", "select_drag"},  StepName{"builder", "recolour"},
    StepName{"builder", "duplicate"},    StepName{"builder", "delete"},
    StepName{"builder", "camera"},
};

// The main functions, in tour order. Each has one poster, numbered by its
// place here, so a poster's name never depends on which steps ran.
constexpr std::array<std::string_view, 2> kMains = {"drone_box", "builder"};

// Tolerated anomalies, each with why. An entry here must fire on every run:
// one that stops firing fails the run (it has expired, so delete it).
const std::vector<KnownOpen> kKnownOpen = {};

[[nodiscard]] std::string fmt(float v) {
    char text[32];
    std::snprintf(text, sizeof text, "%.3f", static_cast<double>(v));
    return text;
}

// The angle between two orientations, radians.
[[nodiscard]] float angle_between(const glm::quat& a, const glm::quat& b) {
    const float d = std::min(1.0f, std::abs(glm::dot(a, b)));
    return 2.0f * std::acos(d);
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
    [[nodiscard]] bool wants_capture() const noexcept { return video_pending_ || !poster_.empty(); }

    // The window's frame tap: rows bottom-up, as GL reads them.
    void on_frame(const uint8_t* rgba, uint32_t w, uint32_t h) {
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

    [[nodiscard]] bool stopped() const noexcept { return stopped_; }
    [[nodiscard]] uint64_t frames() const noexcept { return frame_; }
    [[nodiscard]] Smoke& smoke() noexcept { return smoke_; }
    [[nodiscard]] GlTargetSink& sink() noexcept { return sink_; }

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
        s.check(cam.distance < 2.2f, "back-in", "distance " + fmt(cam.distance));
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

    t.step("drone_box", "heatmap", [&] {
        FrameInput v;
        v.toggle_view_pressed = true;
        t.pump(1, hold(v), draw);
        s.check(d.panel().view_heatmap, "view-on", "view_heatmap " + std::to_string(d.panel().view_heatmap));
        FrameInput slow;
        slow.move_right = 0.4f;  // the slice keeps facing the camera as it goes round
        t.pump(150, hold(slow), draw);
        t.poster("drone_box");
        t.pump(90, hold(slow), draw);
        s.check(d.panel().observed_max > 1.0f, "downwash-shows",
                "slice maximum " + fmt(d.panel().observed_max) + " m/s");
        t.pump(1, hold(v), draw);
        s.check(!d.panel().view_heatmap, "view-off", "view_heatmap " + std::to_string(d.panel().view_heatmap));
        t.pump(30, idle, draw);
    });

    // A panel edit is applied when the widget is let go (INT-2): the tour
    // writes the panel's edited values, as a released slider does, and the
    // session's frame applies them.
    t.step("drone_box", "wind", [&] {
        const uint32_t rebuilds = d.rebuilds();
        const glm::quat before = d.panel().readouts.orientation;
        d.panel().edited.wind_speed_mps = 8.0f;
        d.panel().edited.wind_heading_deg = 45.0f;
        t.pump(1, idle, draw);
        s.check(d.rebuilds() == rebuilds + 1u, "rebuilds", std::to_string(rebuilds) + " -> " + std::to_string(d.rebuilds()));
        s.check(d.drone().options().wind_speed_mps == 8.0f && d.panel().status.empty(), "applied",
                "wind " + fmt(d.drone().options().wind_speed_mps) + " m/s, status '" + d.panel().status + "'");
        const float carried = angle_between(before, d.panel().readouts.orientation);
        s.check(carried < 0.1f, "attitude-carried", "moved " + fmt(carried) + " rad across the rebuild");
        t.pump(239, idle, draw);
    });

    t.step("drone_box", "turbulence", [&] {
        const uint32_t rebuilds = d.rebuilds();
        d.panel().edited.turbulence = TurbulenceLevel::moderate;
        t.pump(1, idle, draw);
        s.check(d.rebuilds() == rebuilds + 1u && d.drone().options().turbulence == TurbulenceLevel::moderate,
                "applied", "rebuilds " + std::to_string(rebuilds) + " -> " + std::to_string(d.rebuilds()));
        t.pump(239, idle, draw);
        d.panel().edited = DronePhysicsOptions{};  // calm air again
        t.pump(1, idle, draw);
        s.check(d.drone().options() == DronePhysicsOptions{}, "calm-again",
                "wind " + fmt(d.drone().options().wind_speed_mps) + " m/s");
        t.pump(29, idle, draw);
    });

    t.step("drone_box", "throttle", [&] {
        const float omega0 = mean_rotor_omega(d.panel().readouts);
        d.panel().edited.throttle = 1.4f;
        t.pump(120, idle, draw);
        const float omega1 = mean_rotor_omega(d.panel().readouts);
        s.check(omega1 > omega0 * 1.1f, "rotors-speed-up",
                "mean rotor speed " + fmt(omega0) + " -> " + fmt(omega1) + " rad/s");
        d.panel().edited.throttle = 1.0f;
        t.pump(60, idle, draw);
    });

    // Vulkan is refused while behaviors are CPU-only: the running simulation
    // stays, the reason shows, and the control goes back (INT-2).
    t.step("drone_box", "vulkan_refused", [&] {
        const uint32_t rebuilds = d.rebuilds();
        d.panel().edited.vulkan = true;
        t.pump(1, idle, draw);
        s.check(!d.drone().options().vulkan && d.rebuilds() == rebuilds, "kept-cpu",
                "backend " + std::string(d.drone().options().vulkan ? "Vulkan" : "CPU"));
        s.check(!d.panel().status.empty(), "reason-shown", d.panel().status);
        s.check(!d.panel().edited.vulkan, "control-put-back", "edited backend " +
                                                                 std::string(d.panel().edited.vulkan ? "Vulkan" : "CPU"));
        t.pump(149, idle, draw);
    });

    // F1's own path: poll() calls toggle_help() on the key's press. The checks
    // read whether the presented frames drew the legend, not the flag.
    t.step("drone_box", "hud", [&] {
        const auto drawn = [&t] { return std::string(t.sink().help_drawn() ? "drawn" : "not drawn"); };
        const bool before = t.sink().help_drawn();
        t.sink().toggle_help();
        t.pump(60, idle, draw);
        s.check(before && !t.sink().help_drawn(), "hidden",
                "legend " + std::string(before ? "drawn" : "not drawn") + " -> " + drawn());
        t.sink().toggle_help();
        t.pump(60, idle, draw);
        s.check(t.sink().help_drawn(), "shown", "legend " + drawn());
    });
}

// ---------------------------------------------------------------------------
// MAIN FUNCTION 2: THE BUILDER.
// ---------------------------------------------------------------------------
void builder(Tour& t, BuilderSession& b) {
    Smoke& s = t.smoke();
    const DrawFn draw = [&b](const FrameInput& in) { return b.frame(in); };
    const auto idle = InputFn{};
    BuilderScene& model = b.builder();
    const float fw = static_cast<float>(t.sink().framebuffer_width());
    const float fh = static_cast<float>(t.sink().framebuffer_height());
    // Three ground points in the lower half, where the ground is: the same
    // points the builder's --smoke clicks.
    const float x1 = fw * 0.50f, y1 = fh * 0.72f;
    const float x2 = fw * 0.38f, y2 = fh * 0.66f;
    const float x3 = fw * 0.62f, y3 = fh * 0.66f;
    const auto click_at = [](float x, float y) {
        return [x, y](int frame) {
            FrameInput in;
            in.mouse_x = x;
            in.mouse_y = y;
            in.left_click = frame == 0;
            in.left_down = frame == 0;
            return in;
        };
    };

    t.step("builder", "open", [&] {
        t.pump(60, idle, draw);
        s.check(b.gpu() != nullptr, "gpu-path", b.gpu() != nullptr ? "GPU (OpenGL)" : "CPU raster");
        s.check(model.objects.empty(), "empty", std::to_string(model.objects.size()) + " objects");
    });

    t.step("builder", "place", [&] {
        model.placing = true;
        model.pending_shape = Shape::box;
        t.pump(45, click_at(x1, y1), draw);
        model.pending_shape = Shape::sphere;
        t.pump(45, click_at(x2, y2), draw);
        model.pending_shape = Shape::cylinder;
        t.pump(60, click_at(x3, y3), draw);
        s.check(model.objects.size() == 3u, "three-placed", std::to_string(model.objects.size()) + " objects");
        bool on_ground = !model.objects.empty();
        for (const BuilderObject& o : model.objects) {
            on_ground = on_ground && std::abs(o.position.y - (b.scene().ground_y + o.scale.y * 0.5f)) < 1e-3f;
        }
        s.check(on_ground, "on-ground", "ground_y " + fmt(b.scene().ground_y));
        s.check(b.scene().dynamics.size() == model.objects.size(), "rendered",
                std::to_string(b.scene().dynamics.size()) + " draw items");
    });

    t.step("builder", "select_drag", [&] {
        model.placing = false;
        t.pump(20, click_at(x1, y1), draw);
        if (!s.check(model.selected == 0 && !model.objects.empty(), "selects",
                     "selected " + std::to_string(model.selected))) {
            t.pump(60, idle, draw);
            return;
        }
        const glm::vec3 before = model.objects[0].position;
        const float dx = fw * 0.12f;
        t.pump(60,
               [&](int frame) {
                   FrameInput in;
                   in.mouse_x = x1 + dx * static_cast<float>(frame + 1) / 60.0f;
                   in.mouse_y = y1;
                   in.left_down = true;
                   return in;
               },
               draw);
        FrameInput release;
        release.mouse_x = x1 + dx;
        release.mouse_y = y1;
        release.left_release = true;
        t.pump(1, [release](int) { return release; }, draw);
        const glm::vec3 after = model.objects[0].position;
        s.check(std::abs(after.x - before.x) > 0.05f, "moves-in-x", "x " + fmt(before.x) + " -> " + fmt(after.x));
        s.check(std::abs(after.y - before.y) < 1e-4f, "height-kept", "y " + fmt(before.y) + " -> " + fmt(after.y));
        t.pump(45, idle, draw);
    });

    t.step("builder", "recolour", [&] {
        if (!s.check(!model.objects.empty(), "has-object", std::to_string(model.objects.size()) + " objects")) {
            return;
        }
        const glm::vec4 red(1.0f, 0.1f, 0.05f, 1.0f);
        model.objects[0].color = red;
        model.materials_dirty = true;
        t.pump(90, idle, draw);
        // Object 0 draws with material material_base + 0 (builder_scene.hpp).
        const uint32_t m = b.binding().material_base;
        const bool has = m < b.scene().materials.size();
        const glm::vec4 got = has ? b.scene().materials[m].base_color : glm::vec4(0.0f);
        s.check(has && got == red, "reaches-material",
                "material " + std::to_string(m) + " base colour (" + fmt(got.r) + ", " + fmt(got.g) + ", " +
                    fmt(got.b) + "), set (1.000, 0.100, 0.050)");
    });

    t.step("builder", "duplicate", [&] {
        const std::size_t n = model.objects.size();
        model.selected = 0;
        FrameInput dup;
        dup.duplicate_pressed = true;
        t.pump(1, [dup](int) { return dup; }, draw);
        s.check(model.objects.size() == n + 1u, "adds-one",
                std::to_string(n) + " -> " + std::to_string(model.objects.size()) + " objects");
        t.pump(45, idle, draw);
        t.poster("builder");
        t.pump(44, idle, draw);
    });

    t.step("builder", "delete", [&] {
        const std::size_t n = model.objects.size();
        model.selected = static_cast<int>(n) - 1;
        FrameInput del;
        del.delete_pressed = true;
        t.pump(1, [del](int) { return del; }, draw);
        s.check(n > 0u && model.objects.size() == n - 1u, "removes-one",
                std::to_string(n) + " -> " + std::to_string(model.objects.size()) + " objects");
        t.pump(89, idle, draw);
    });

    t.step("builder", "camera", [&] {
        OrbitCamera& cam = b.camera();
        const float yaw0 = cam.yaw;
        FrameInput orbit;
        orbit.orbit_dx = 3.0f;  // a right-drag, 3 px a frame
        t.pump(90, [orbit](int) { return orbit; }, draw);
        s.check(std::abs(cam.yaw - yaw0) > 0.5f, "orbits", "yaw " + fmt(yaw0) + " -> " + fmt(cam.yaw));

        const float d0 = cam.distance;
        FrameInput dolly;
        dolly.dolly = 0.2f;  // a fifth of a scroll notch a frame, toward the target
        t.pump(60, [dolly](int) { return dolly; }, draw);
        s.check(cam.distance < d0, "dollies", "distance " + fmt(d0) + " -> " + fmt(cam.distance));

        const glm::vec3 target0 = cam.target;
        FrameInput pan;
        pan.move_right = 1.0f;
        t.pump(60, [pan](int) { return pan; }, draw);
        FrameInput up;
        up.move_up = 1.0f;
        t.pump(30, [up](int) { return up; }, draw);
        s.check(glm::length(cam.target - target0) > 1.0f, "pans",
                "target moved " + fmt(glm::length(cam.target - target0)) + " m");
        t.pump(60, idle, draw);
    });
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
    if (const Result<void> swept = sweep(options.out_dir, owned_names(kMains)); !swept) {
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
    spade::Result<std::unique_ptr<BuilderSession>> b = BuilderSession::create(true, options.blur);
    if (!b) {
        smoke.anomaly("harness/builder-session", b.error().context);
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

    Recorder rec(options.out_dir, smoke);
    rec.start(sink->framebuffer_width(), sink->framebuffer_height());
    sink->set_frame_tap([&rec](const uint8_t* rgba, uint32_t w, uint32_t h) { rec.on_frame(rgba, w, h); });

    Tour tour(options, *sink, smoke, rec);
    const DrawFn blank = blank_frame(*sink);

    sink->set_card(opening_card(options.label));
    tour.pump(kOpeningFrames, {}, blank);
    sink->set_card({});

    if (drone && !tour.stopped()) {
        (*drone)->attach(*sink);
        drone_box(tour, **drone);
    }
    if (drone) {
        drone->reset();  // detaches its panel before the builder attaches
    }
    if (b && !tour.stopped()) {
        (*b)->attach(*sink);
        builder(tour, **b);
    }
    if (b) {
        b->reset();
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
    if (rec.piped()) {
        std::error_code ec;
        const std::uintmax_t size = fs::file_size(options.out_dir / "tour.mp4", ec);
        (void)smoke.check(!ec && size > 0u, "mp4-written",
                          ec ? "tour.mp4: " + ec.message() : "tour.mp4 " + std::to_string(size) + " bytes");
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
