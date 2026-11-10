#pragma once

// The recorder's two halves. A GL stage draws a scene offscreen at full flow
// quality and reads it back. A clip pipes those frames to ffmpeg, then burns
// per-frame captions in (libass) and encodes H.264 at a fixed frame rate.
// Frames are indexed, never timed: a frame's simulated time is the case's
// business, so the solver's speed changes nothing in the video.

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

struct GLFWwindow;
namespace spade::render_gl {
class GlRenderer;
}

namespace flow_movie {

class GlStage {
  public:
    // Refused with Code::unavailable where no window or GL 4.3 context opens.
    [[nodiscard]] static spade::Result<std::unique_ptr<GlStage>> create(uint32_t width, uint32_t height);
    ~GlStage();
    GlStage(const GlStage&) = delete;
    GlStage& operator=(const GlStage&) = delete;

    // One frame at full flow quality: RGBA8, the top row first.
    [[nodiscard]] spade::Result<void> draw(const spade::render::RenderScene& scene,
                                           const spade::render::Camera& camera, std::vector<uint8_t>& rgba);
    [[nodiscard]] std::string device() const;
    // The GL limits this stage's frame was checked against: "max renderbuffer 16384, max viewport 16384x16384, max
    // texture 16384" (create() refuses a frame past them, naming the limit).
    [[nodiscard]] const std::string& limits() const noexcept { return limits_; }
    [[nodiscard]] uint32_t width() const noexcept { return width_; }
    [[nodiscard]] uint32_t height() const noexcept { return height_; }

  private:
    GlStage() = default;
    GLFWwindow* window_ = nullptr;
    std::unique_ptr<spade::render_gl::GlRenderer> renderer_;
    unsigned fbo_ = 0, colour_ = 0, depth_ = 0;
    uint32_t width_ = 0, height_ = 0;
    std::string limits_;
    std::vector<uint8_t> bottom_first_;
};

// A caption line. Rows count down from the top left (row >= 0), or up from
// the bottom left (row < 0; -1 is the bottom line). A title row is larger.
struct Caption {
    int row = 0;
    std::string text;  // UTF-8
    bool title = false;
};

struct Encoder {
    std::string ffmpeg = "ffmpeg";
    uint32_t fps = 30;
    uint32_t width = 1920, height = 1080;
};

// One clip of the final video. Every clip is encoded with the same settings,
// so concatenate() joins them without re-encoding.
class Clip {
  public:
    // Starts ffmpeg writing <dir>/<name>.raw.mkv, near lossless.
    [[nodiscard]] static spade::Result<std::unique_ptr<Clip>> open(const Encoder& encoder,
                                                                 const std::filesystem::path& dir,
                                                                 const std::string& name);
    ~Clip();
    Clip(const Clip&) = delete;
    Clip& operator=(const Clip&) = delete;

    // One frame (RGBA8, top row first, the encoder's size) and its captions.
    [[nodiscard]] spade::Result<void> add_frame(std::span<const uint8_t> rgba, std::vector<Caption> captions);
    // Closes the pipe, writes <name>.ass, burns the captions in: <name>.mp4.
    [[nodiscard]] spade::Result<std::filesystem::path> finish();
    [[nodiscard]] uint32_t frames() const noexcept { return static_cast<uint32_t>(captions_.size()); }

  private:
    Clip() = default;
    Encoder encoder_;
    std::filesystem::path dir_;
    std::string name_;
    std::FILE* pipe_ = nullptr;
    std::vector<std::vector<Caption>> captions_;  // per frame
};

// A website master (web_masters.hpp): raw RGBA frames straight into one libx264 pass, CRF 15, yuv420p, faststart,
// no audio, and no captions burned in. The file is written to `out` as the frames arrive.
class WebClip {
  public:
    [[nodiscard]] static spade::Result<std::unique_ptr<WebClip>> open(const std::string& ffmpeg, uint32_t width,
                                                                    uint32_t height,
                                                                    const std::filesystem::path& out);
    ~WebClip();
    WebClip(const WebClip&) = delete;
    WebClip& operator=(const WebClip&) = delete;

    // One frame, RGBA8, top row first, width x height x 4 bytes.
    [[nodiscard]] spade::Result<void> add_frame(std::span<const uint8_t> rgba);
    // Closes the pipe; refused when ffmpeg did not finish cleanly. The file exists only after this returns.
    [[nodiscard]] spade::Result<std::filesystem::path> finish();
    [[nodiscard]] uint32_t frames() const noexcept { return frames_; }

  private:
    WebClip() = default;
    uint32_t width_ = 0, height_ = 0, frames_ = 0;
    std::filesystem::path out_;
    std::FILE* pipe_ = nullptr;
};

// `seconds` of a still frame (a dark background) with captions.
[[nodiscard]] spade::Result<std::filesystem::path> title_card(const Encoder& encoder, const std::filesystem::path& dir,
                                                              const std::string& name, double seconds,
                                                              const std::vector<Caption>& captions);

// Joins clips, all from this file's encoder, into one MP4 without re-encoding.
[[nodiscard]] spade::Result<void> concatenate(const Encoder& encoder, const std::vector<std::filesystem::path>& clips,
                                              const std::filesystem::path& out);

// What ffprobe reads of a delivered file: its first video stream, and whether it has audio.
struct VideoInfo {
    uint32_t width = 0, height = 0, frames = 0;
    std::string codec, pix_fmt;
    bool has_audio = false;
};
// Probes `file` (a video or a PNG) with ffprobe, found beside `ffmpeg`, counting its frames by decoding them.
[[nodiscard]] spade::Result<VideoInfo> probe_video(const std::string& ffmpeg, const std::filesystem::path& file);

// One frame (RGBA8, top row first) as a PNG, for looking at a shot.
[[nodiscard]] spade::Result<void> save_png(const Encoder& encoder, std::span<const uint8_t> rgba,
                                         const std::filesystem::path& path);

// The code that makes the frames, for every card: "commit <sha12>[ with
// uncommitted changes], <toolchain>, <preset>", the commit and changes as they
// were when this binary was BUILT (the engine's stamp, core/build_identity.hpp).
// The records name theirs.
[[nodiscard]] std::string build_identity();

// Whether ffmpeg runs, with libx264 and libass.
[[nodiscard]] spade::Result<void> check_ffmpeg(const Encoder& encoder);

}  // namespace flow_movie
