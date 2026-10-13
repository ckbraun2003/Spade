#include "recorder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <utility>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "render_gl/gl_renderer.hpp"

namespace flow_movie {
namespace {

namespace fs = std::filesystem;
using spade::Code;
using spade::Error;

std::string g_glfw_error;

void record_glfw_error(int code, const char* description) {
    g_glfw_error = "GLFW error " + std::to_string(code) + ": " + (description != nullptr ? description : "");
}

[[nodiscard]] auto refuse(Code code, const std::string& why) { return std::unexpected(Error{code, why}); }

// A path or program for the shell, quoted.
[[nodiscard]] std::string in_quotes(const std::string& s) { return "\"" + s + "\""; }

// cmd.exe strips the first and last quote of a command that starts with one,
// so the whole command goes in one more pair. POSIX shells need none.
[[nodiscard]] std::string shell_command(const std::string& command) {
#ifdef _WIN32
    return "\"" + command + "\"";
#else
    return command;
#endif
}

[[nodiscard]] std::FILE* open_pipe(const std::string& command, const char* mode) {
#ifdef _WIN32
    return _popen(shell_command(command).c_str(), mode);
#else
    return popen(shell_command(command).c_str(), mode[0] == 'w' ? "w" : "r");
#endif
}

[[nodiscard]] int close_pipe(std::FILE* pipe) {
#ifdef _WIN32
    return _pclose(pipe);
#else
    return pclose(pipe);
#endif
}

// What a command prints, and whether it exited 0.
[[nodiscard]] std::pair<bool, std::string> run_capture(const std::string& command) {
    std::FILE* pipe = open_pipe(command, "r");
    if (pipe == nullptr) return {false, ""};
    std::string out;
    char buffer[4096];
    while (const std::size_t got = std::fread(buffer, 1, sizeof buffer, pipe)) out.append(buffer, got);
    return {close_pipe(pipe) == 0, out};
}

// Runs `command` with `dir` as the working directory, so filter arguments can
// name files there without a drive letter (a colon is an ffmpeg separator).
[[nodiscard]] bool run_in(const fs::path& dir, const std::string& command) {
    const fs::path before = fs::current_path();
    fs::current_path(dir);
    const int status = std::system(shell_command(command).c_str());
    fs::current_path(before);
    return status == 0;
}

// The near-lossless intermediate and the final encode. Every clip uses these,
// which is what lets concatenate() copy streams.
[[nodiscard]] std::string intermediate_args() {
    return "-c:v libx264 -preset veryfast -crf 6 -pix_fmt yuv444p -threads 4";
}

[[nodiscard]] std::string final_args(const Encoder& e) {
    return "-c:v libx264 -preset slow -crf 17 -profile:v high -pix_fmt yuv420p -g " + std::to_string(2u * e.fps) +
           " -r " + std::to_string(e.fps) + " -threads 4 -movflags +faststart";
}

// ASS time: H:MM:SS.cc, from centiseconds.
[[nodiscard]] std::string ass_time(uint64_t cs) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%llu:%02llu:%02llu.%02llu", static_cast<unsigned long long>(cs / 360000u),
                  static_cast<unsigned long long>(cs / 6000u % 60u), static_cast<unsigned long long>(cs / 100u % 60u),
                  static_cast<unsigned long long>(cs % 100u));
    return buffer;
}

// A caption's text inside an ASS event: braces would open an override block.
[[nodiscard]] std::string ass_text(const std::string& s) {
    std::string out;
    for (const char c : s) {
        if (c == '{') {
            out += '(';
        } else if (c == '}') {
            out += ')';
        } else if (c == '\n') {
            out += "\\N";
        } else {
            out += c;
        }
    }
    return out;
}

// Each row's runs of unchanged text become one event, timed so that frame k
// (shown from k / fps) is the only frame of its run boundary: an event spans
// [floor(100 k0 / fps), floor(100 (k1 + 1) / fps)) in centiseconds.
[[nodiscard]] bool write_ass(const fs::path& path, const Encoder& e, const std::vector<std::vector<Caption>>& frames) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    // Laid out on a 1920 x 1080 canvas, which libass scales to the frame.
    constexpr int kCanvasHeight = 1080;
    out << "[Script Info]\nScriptType: v4.00+\nPlayResX: 1920\nPlayResY: " << kCanvasHeight
        << "\nWrapStyle: 2\nScaledBorderAndShadow: yes\n\n[V4+ Styles]\n"
           "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, "
           "Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, "
           "MarginL, MarginR, MarginV, Encoding\n"
           "Style: Body,Segoe UI,30,&H00F0F0F0,&H00F0F0F0,&H00101010,&H80000000,0,0,0,0,100,100,0,0,1,2,1,7,40,40,"
           "30,1\n"
           "Style: Title,Segoe UI Semibold,42,&H00FFFFFF,&H00FFFFFF,&H00101010,&H80000000,-1,0,0,0,100,100,0,0,1,2,"
           "1,7,40,40,30,1\n\n[Events]\n"
           "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n";
    std::set<int> rows;
    for (const auto& frame : frames) {
        for (const Caption& c : frame) rows.insert(c.row);
    }
    const auto text_at = [&](std::size_t k, int row) -> const Caption* {
        for (const Caption& c : frames[k]) {
            if (c.row == row) return &c;
        }
        return nullptr;
    };
    const auto cs = [&](std::size_t k) { return static_cast<uint64_t>(100u * k / e.fps); };
    for (const int row : rows) {
        const int x = 40;
        const int y = row >= 0 ? 30 + (row == 0 ? 0 : 62 + 42 * (row - 1))
                               : kCanvasHeight - 30 - 42 * (-row - 1);
        const char* anchor = row >= 0 ? "\\an7" : "\\an1";
        std::size_t k = 0;
        while (k < frames.size()) {
            const Caption* c = text_at(k, row);
            std::size_t end = k + 1;
            while (end < frames.size()) {
                const Caption* d = text_at(end, row);
                const bool same = (c == nullptr && d == nullptr) ||
                                  (c != nullptr && d != nullptr && c->text == d->text && c->title == d->title);
                if (!same) break;
                ++end;
            }
            if (c != nullptr && !c->text.empty()) {
                out << "Dialogue: 0," << ass_time(cs(k)) << ',' << ass_time(cs(end)) << ','
                    << (c->title ? "Title" : "Body") << ",,0,0,0,,{" << anchor << "\\pos(" << x << ',' << y << ")}"
                    << ass_text(c->text) << '\n';
            }
            k = end;
        }
    }
    return static_cast<bool>(out);
}

}  // namespace

// --- GlStage -------------------------------------------------------------------------

spade::Result<std::unique_ptr<GlStage>> GlStage::create(uint32_t width, uint32_t height) {
    glfwSetErrorCallback(record_glfw_error);
    if (glfwInit() != GLFW_TRUE) return refuse(Code::unavailable, "GLFW could not initialise: " + g_glfw_error);
    std::unique_ptr<GlStage> stage(new GlStage());
    stage->width_ = width;
    stage->height_ = height;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    stage->window_ = glfwCreateWindow(64, 64, "spade flow_movie", nullptr, nullptr);
    if (stage->window_ == nullptr) {
        glfwTerminate();
        return refuse(Code::unavailable, "no OpenGL 4.3 core context: " + g_glfw_error);
    }
    glfwMakeContextCurrent(stage->window_);
    if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) == 0) {
        return refuse(Code::unavailable, "glad could not load GL entry points");
    }
    auto made = spade::render_gl::GlRenderer::create(reinterpret_cast<spade::render_gl::GlProcLoader>(glfwGetProcAddress));
    if (!made) return std::unexpected(made.error());
    stage->renderer_ = std::move(*made);
    // Recordings march every frame at full resolution (RND-7: full is the
    // frame the bands measure, and the one for recordings).
    spade::render_gl::GlRenderer::FlowQuality quality;
    quality.scale = spade::render_gl::GlRenderer::FlowScale::full;
    stage->renderer_->set_flow_quality(quality);

    glGenFramebuffers(1, &stage->fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, stage->fbo_);
    glGenRenderbuffers(1, &stage->colour_);
    glBindRenderbuffer(GL_RENDERBUFFER, stage->colour_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, static_cast<GLsizei>(width), static_cast<GLsizei>(height));
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, stage->colour_);
    glGenRenderbuffers(1, &stage->depth_);
    glBindRenderbuffer(GL_RENDERBUFFER, stage->depth_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, static_cast<GLsizei>(width),
                          static_cast<GLsizei>(height));
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, stage->depth_);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        return refuse(Code::unavailable, "the offscreen framebuffer is incomplete");
    }
    stage->bottom_first_.resize(static_cast<std::size_t>(width) * height * 4u);
    return stage;
}

GlStage::~GlStage() {
    if (window_ == nullptr) return;
    renderer_.reset();  // its GL objects die with the context, which must still be current
    if (fbo_ != 0) glDeleteFramebuffers(1, &fbo_);
    if (colour_ != 0) glDeleteRenderbuffers(1, &colour_);
    if (depth_ != 0) glDeleteRenderbuffers(1, &depth_);
    glfwDestroyWindow(window_);
    glfwTerminate();
}

spade::Result<void> GlStage::draw(const spade::render::RenderScene& scene, const spade::render::Camera& camera,
                                  std::vector<uint8_t>& rgba) {
    // The flow views carry no meshes, so an upload per frame costs nothing,
    // and a scene that does carry them never draws against a stale set.
    if (auto up = renderer_->upload_scene(scene); !up) return up;
    spade::render::RenderOptions options;
    options.shadows = false;
    options.overlays = false;
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, static_cast<GLsizei>(width_), static_cast<GLsizei>(height_));
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (auto drew = renderer_->draw(scene, camera, options, width_, height_); !drew) return drew;
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, static_cast<GLsizei>(width_), static_cast<GLsizei>(height_), GL_RGBA, GL_UNSIGNED_BYTE,
                 bottom_first_.data());
    const std::size_t row = static_cast<std::size_t>(width_) * 4u;
    rgba.resize(bottom_first_.size());
    for (uint32_t y = 0; y < height_; ++y) {
        std::memcpy(rgba.data() + y * row, bottom_first_.data() + (height_ - 1u - y) * row, row);
    }
    // The march leaves its transmittance in alpha (0 behind an opaque iso):
    // a frame is a picture, so every pixel is opaque.
    for (std::size_t p = 3; p < rgba.size(); p += 4u) rgba[p] = 255u;
    return {};
}

std::string GlStage::device() const { return renderer_->renderer_name() + " (" + renderer_->version_string() + ")"; }

// --- Clip -----------------------------------------------------------------------------

spade::Result<std::unique_ptr<Clip>> Clip::open(const Encoder& encoder, const fs::path& dir, const std::string& name) {
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) return refuse(Code::io_error, "cannot create " + dir.string() + ": " + ec.message());
    std::unique_ptr<Clip> clip(new Clip());
    clip->encoder_ = encoder;
    clip->dir_ = dir;
    clip->name_ = name;
    const std::string command = in_quotes(encoder.ffmpeg) + " -hide_banner -loglevel error -y -f rawvideo -pix_fmt rgba -s " +
                                std::to_string(encoder.width) + "x" + std::to_string(encoder.height) + " -r " +
                                std::to_string(encoder.fps) + " -i - " + intermediate_args() + " " +
                                in_quotes((dir / (name + ".raw.mkv")).string());
    clip->pipe_ = open_pipe(command, "wb");
    if (clip->pipe_ == nullptr) return refuse(Code::unavailable, "cannot start ffmpeg: " + command);
    return clip;
}

Clip::~Clip() {
    if (pipe_ != nullptr) (void)close_pipe(pipe_);
}

spade::Result<void> Clip::add_frame(std::span<const uint8_t> rgba, std::vector<Caption> captions) {
    const std::size_t want = static_cast<std::size_t>(encoder_.width) * encoder_.height * 4u;
    if (pipe_ == nullptr) return refuse(Code::invalid_argument, "clip " + name_ + " is finished");
    if (rgba.size() != want) {
        return refuse(Code::invalid_argument, "clip " + name_ + ": a frame of " + std::to_string(rgba.size()) +
                                                  " bytes, not " + std::to_string(want));
    }
    if (std::fwrite(rgba.data(), 1, rgba.size(), pipe_) != rgba.size()) {
        return refuse(Code::io_error, "clip " + name_ + ": ffmpeg stopped reading frames");
    }
    captions_.push_back(std::move(captions));
    return {};
}

spade::Result<fs::path> Clip::finish() {
    if (pipe_ == nullptr) return refuse(Code::invalid_argument, "clip " + name_ + " is finished");
    const int status = close_pipe(pipe_);
    pipe_ = nullptr;
    if (status != 0) return refuse(Code::io_error, "clip " + name_ + ": ffmpeg's intermediate encode failed");
    if (!write_ass(dir_ / (name_ + ".ass"), encoder_, captions_)) {
        return refuse(Code::io_error, "clip " + name_ + ": cannot write its captions");
    }
    const std::string command = in_quotes(encoder_.ffmpeg) + " -hide_banner -loglevel error -y -i " +
                                in_quotes(name_ + ".raw.mkv") + " -vf " + in_quotes("ass=" + name_ + ".ass") + " " +
                                final_args(encoder_) + " " + in_quotes(name_ + ".mp4");
    if (!run_in(dir_, command)) return refuse(Code::io_error, "clip " + name_ + ": burning the captions in failed");
    std::error_code ec;
    fs::remove(dir_ / (name_ + ".raw.mkv"), ec);
    return dir_ / (name_ + ".mp4");
}

// --- cards, joining, checks -------------------------------------------------------------

spade::Result<fs::path> title_card(const Encoder& encoder, const fs::path& dir, const std::string& name,
                                   double seconds, const std::vector<Caption>& captions) {
    auto clip = Clip::open(encoder, dir, name);
    if (!clip) return std::unexpected(clip.error());
    std::vector<uint8_t> frame(static_cast<std::size_t>(encoder.width) * encoder.height * 4u);
    for (std::size_t p = 0; p < frame.size(); p += 4u) {  // a dark slate
        frame[p] = 14u;
        frame[p + 1u] = 17u;
        frame[p + 2u] = 22u;
        frame[p + 3u] = 255u;
    }
    const auto n = static_cast<uint32_t>(std::lround(seconds * encoder.fps));
    for (uint32_t k = 0; k < std::max(n, 1u); ++k) {
        if (auto added = (*clip)->add_frame(frame, captions); !added) return std::unexpected(added.error());
    }
    return (*clip)->finish();
}

spade::Result<void> concatenate(const Encoder& encoder, const std::vector<fs::path>& clips, const fs::path& out) {
    const fs::path list = out.parent_path() / (out.stem().string() + ".concat.txt");
    {
        std::ofstream f(list, std::ios::binary);
        for (const fs::path& clip : clips) f << "file '" << fs::absolute(clip).generic_string() << "'\n";
        if (!f) return refuse(Code::io_error, "cannot write " + list.string());
    }
    const std::string command = in_quotes(encoder.ffmpeg) + " -hide_banner -loglevel error -y -f concat -safe 0 -i " +
                                in_quotes(list.string()) + " -c copy -movflags +faststart " + in_quotes(out.string());
    const auto [ok, printed] = run_capture(command);
    std::error_code ec;
    fs::remove(list, ec);
    if (!ok) return refuse(Code::io_error, "joining the clips into " + out.string() + " failed: " + printed);
    return {};
}

spade::Result<void> save_png(const Encoder& encoder, std::span<const uint8_t> rgba, const fs::path& path) {
    const std::string command = in_quotes(encoder.ffmpeg) + " -hide_banner -loglevel error -y -f rawvideo -pix_fmt rgba -s " +
                                std::to_string(encoder.width) + "x" + std::to_string(encoder.height) +
                                " -i - -frames:v 1 " + in_quotes(path.string());
    std::FILE* pipe = open_pipe(command, "wb");
    if (pipe == nullptr) return refuse(Code::unavailable, "cannot start ffmpeg: " + command);
    const bool wrote = std::fwrite(rgba.data(), 1, rgba.size(), pipe) == rgba.size();
    if (close_pipe(pipe) != 0 || !wrote) return refuse(Code::io_error, "writing " + path.string() + " failed");
    return {};
}

std::string build_identity() {
    // Any folder in the tree names its commit, and status covers the whole tree.
    const std::string git = "git -C " + in_quotes(SPADE_ENGINE_DIR);
    auto [ok, sha] = run_capture(git + " rev-parse --short HEAD");
    while (!sha.empty() && (sha.back() == '\n' || sha.back() == '\r')) sha.pop_back();
    const auto [listed, changes] = run_capture(git + " status --porcelain --untracked-files=no");
    std::string id = ok && !sha.empty() ? "commit " + sha : "commit unknown";
    if (listed && !changes.empty()) id += " with uncommitted changes";
#if defined(_MSC_VER)
    id += ", MSVC " + std::to_string(_MSC_VER);
#elif defined(__clang__)
    id += ", clang " + std::to_string(__clang_major__) + "." + std::to_string(__clang_minor__);
#elif defined(__GNUC__)
    id += ", gcc " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__);
#endif
#ifdef NDEBUG
    id += ", release";
#else
    id += ", debug";
#endif
    return id;
}

spade::Result<void> check_ffmpeg(const Encoder& encoder) {
    const auto [ran, encoders] = run_capture(in_quotes(encoder.ffmpeg) + " -hide_banner -encoders");
    if (!ran) return refuse(Code::unavailable, "ffmpeg does not run: " + encoder.ffmpeg);
    if (encoders.find("libx264") == std::string::npos) return refuse(Code::unavailable, "ffmpeg has no libx264");
    const auto [listed, filters] = run_capture(in_quotes(encoder.ffmpeg) + " -hide_banner -filters");
    if (!listed || filters.find(" ass ") == std::string::npos) {
        return refuse(Code::unavailable, "ffmpeg has no ass filter (libass)");
    }
    return {};
}

}  // namespace flow_movie
