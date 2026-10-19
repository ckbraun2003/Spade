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
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "analysis/case.hpp"
#include "analysis/live_run.hpp"
#include "editor_session.hpp"
#include "gl_target_sink.hpp"
#include "png_writer.hpp"
#include "scene_sessions.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/quadrotor.hpp"

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
    StepName{"airflow", "capability"},
    StepName{"editor", "open"},          StepName{"editor", "picker"},
    StepName{"editor", "select"},        StepName{"editor", "place"},
    StepName{"editor", "move"},          StepName{"editor", "inspect"},
    StepName{"editor", "undo"},          StepName{"editor", "run"},
    StepName{"editor", "world"},         StepName{"editor", "conflict"},
    StepName{"editor", "save"},
};

// The extra stills a run writes beside the posters (Tour::snapshot), declared
// so that a later run into the same folder owns them.
constexpr std::array<std::string_view, 8> kSnapshots = {"editor_picker",   "editor_vehicle", "editor_place",
                                                        "editor_gizmo",    "editor_inspect", "editor_running",
                                                        "editor_world",    "editor_conflict"};

// The main functions, in tour order. Each has one poster, numbered by its
// place here, so a poster's name never depends on which steps ran.
constexpr std::array<std::string_view, 3> kMains = {"drone_box", "airflow", "editor"};

// Tolerated anomalies, each with why. An entry here must fire on every run:
// one that stops firing fails the run (it has expired, so delete it).
const std::vector<KnownOpen> kKnownOpen = {
    // The airflow chapter's capability card (Test/Docs' expiring entry). It
    // fires while LiveRun refuses the sandbox's drone; the day the sandbox has
    // a drone with its propulsion chain, it stops firing, the run fails as
    // expired, and the chapter must be made live.
    KnownOpen{"airflow/not-available",
              "the sandbox has no drone with its propulsion chain until the library's reference quad (N4), so "
              "LiveRun refuses to turn its rotors"},
};

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
// MAIN FUNCTION 2: THE AIRFLOW (the storyboard's chapter 2). The drone-box
// case's LiveRun, live, once the sandbox has a drone with its propulsion
// chain (the library's reference quad, N4). Until then the chapter is a card
// that shows LiveRun's refusal verbatim and raises airflow/not-available,
// which kKnownOpen tolerates until the day it stops firing.
// ---------------------------------------------------------------------------
void airflow(Tour& t, const DrawFn& blank) {
    Smoke& s = t.smoke();
    t.step("airflow", "capability", [&] {
        // The drone the sandbox flies, in the drone-box case LiveRun runs.
        vehicles::QuadrotorParams params = drone_stand_params();
        const Result<vehicles::ModelType> model = vehicles::make_quadrotor(params);
        if (!s.check(model.has_value(), "model", model ? model->name : model.error().context)) return;
        analysis::AnalysisCase c;
        c.model = analysis::ModelRef{"sandbox.model.yaml", vehicles::model_identity(*model),
                                     vehicles::parts_identity(*model)};
        c.rotors.assign(model->rotors.size(), analysis::RotorSetup{});
        c.grid.box = 1.0f;
        c.grid.spacing = 1.0f / 32.0f;  // the chapter's 32^3, live
        c.time.duration = 2.0f;
        c.time.step = 0.001f;
        const auto resolved = analysis::resolve(c, *model);
        if (!s.check(resolved.has_value(), "resolves",
                     resolved ? "32^3" : resolved.error().front().field + ": " + resolved.error().front().message)) {
            return;
        }
        const Result<analysis::LiveRun> run = analysis::LiveRun::start(*resolved, *model);
        if (run) {
            // The chapter could run live: its known-open entry stops firing
            // and fails the run as expired, which is the signal to build it.
            s.note("airflow: LiveRun starts the sandbox's drone; the chapter can go live");
            return;
        }
        s.anomaly("airflow/not-available", run.error().context);
        t.sink().set_card("Airflow\nnot yet available in the sandbox:\n" + run.error().context);
        t.pump(60, {}, blank);
        t.poster("airflow");
        t.pump(60, {}, blank);
        t.sink().set_card({});
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
        if (!s.check(!ec, "work-copy", ec ? ec.message() : "assets/scenes and assets/worlds copied")) return;
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
        arm("box");
        click_towards(glm::vec3(at.x + 1.0f, 0.0f, at.z));
        arm("box");
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
        chapter("2  Airflow\nthe air around the drone: vortex cores, streamlines,\na pressure slice, C_p on the drone");
        airflow(tour, blank);
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
