// tools/flow_movie/stream_plan.hpp -- streaming a website master (the lead's L86b disk plan). A solve of the deep D1 box writes
// 253 MB a frame, so 600 frames is 152 GB: more than the disk. A streamed run draws each frame as the solve writes it and
// deletes what the clip no longer needs, so at most the mean window and the kept frames are ever on disk.
//
// This is the plan alone, pure: no GL, no solver, no files, so a test pins it without a GPU. The run (main.cpp) asks it three
// things of every frame: is it written at all (needed), may a written frame go now (spent), which stay (kept).
//
// A draw at frame d reads the frames d - M + 1 .. d (M: --mean-frames; 1, the frame alone; the start clips it at 0).
#pragma once

#include <algorithm>
#include <cstdint>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "look.hpp"  // parse_count

namespace flow_movie {

class StreamPlan {
public:
    // `draws`: the frames the run draws, in any order and with repeats (the clip's, and a still's). `keep`: the frames that
    // stay on disk (--keep-frames).
    StreamPlan(std::vector<uint32_t> draws, uint32_t mean_frames, std::vector<uint32_t> keep)
        : draws_(std::move(draws)), mean_(std::max(mean_frames, 1u)), keep_(std::move(keep)) {
        std::sort(draws_.begin(), draws_.end());
        draws_.erase(std::unique(draws_.begin(), draws_.end()), draws_.end());
        std::sort(keep_.begin(), keep_.end());
        keep_.erase(std::unique(keep_.begin(), keep_.end()), keep_.end());
    }

    [[nodiscard]] bool kept(uint32_t j) const { return std::binary_search(keep_.begin(), keep_.end(), j); }

    // Does the solve write frame j at all: it stays (kept), or a draw reads it. A frame no draw reads is solved and dropped.
    [[nodiscard]] bool needed(uint32_t j) const { return kept(j) || last_read(j).has_value(); }

    // May frame j go, once frame k's draw is done (k at or past j): it is not kept and no draw after k reads it.
    [[nodiscard]] bool spent(uint32_t j, uint32_t k) const {
        if (kept(j)) return false;
        const std::optional<uint32_t> last = last_read(j);
        return last.has_value() && *last <= k;
    }

    // The last frame the run must solve: the last it draws or keeps. Nothing after it is needed, so the solve stops there.
    [[nodiscard]] uint32_t last_frame() const {
        uint32_t last = 0;
        if (!draws_.empty()) last = std::max(last, draws_.back());
        if (!keep_.empty()) last = std::max(last, keep_.back());
        return last;
    }

    [[nodiscard]] uint32_t mean_frames() const { return mean_; }

private:
    // The last draw that reads frame j: the largest draw d with j <= d <= j + M - 1. Nothing when no draw does.
    [[nodiscard]] std::optional<uint32_t> last_read(uint32_t j) const {
        const uint32_t reach = j + (mean_ - 1u);
        const auto above = std::upper_bound(draws_.begin(), draws_.end(), reach);  // the first draw past reach
        if (above == draws_.begin()) return std::nullopt;
        const uint32_t d = *(above - 1);
        if (d < j) return std::nullopt;
        return d;
    }

    std::vector<uint32_t> draws_;
    uint32_t mean_;
    std::vector<uint32_t> keep_;
};

// The frames a streamed run keeps until its solve is done: the --keep-frames list, and the frames its still reads (the
// still's own and the mean window ending at it), each once, sorted. The still is drawn after the solve, from them.
[[nodiscard]] inline std::vector<uint32_t> keep_with_still(std::vector<uint32_t> keep, const std::optional<uint32_t>& still,
                                                           uint32_t mean_frames) {
    if (still) {
        const uint32_t m = std::max(mean_frames, 1u);
        const uint32_t lo = *still + 1u > m ? *still + 1u - m : 0u;
        for (uint32_t j = lo; j <= *still; ++j) keep.push_back(j);
    }
    std::sort(keep.begin(), keep.end());
    keep.erase(std::unique(keep.begin(), keep.end()), keep.end());
    return keep;
}

// --keep-frames text: whole frame numbers split by commas, no blanks, none empty; sorted, each once.
[[nodiscard]] inline std::optional<std::vector<uint32_t>> parse_frame_list(std::string_view text) {
    if (text.empty()) return std::nullopt;
    std::vector<uint32_t> frames;
    std::size_t start = 0;
    while (true) {
        const std::size_t comma = text.find(',', start);
        const std::string_view item = text.substr(start, comma == std::string_view::npos ? comma : comma - start);
        const std::optional<uint32_t> frame = parse_count(item);
        if (!frame) return std::nullopt;
        frames.push_back(*frame);
        if (comma == std::string_view::npos) break;
        start = comma + 1u;
    }
    std::sort(frames.begin(), frames.end());
    frames.erase(std::unique(frames.begin(), frames.end()), frames.end());
    return frames;
}

// --still-size text: WIDTHxHEIGHT, whole numbers from 64 to 16384 each, one lower-case x, nothing else.
[[nodiscard]] inline std::optional<std::pair<uint32_t, uint32_t>> parse_size(std::string_view text) {
    const std::size_t x = text.find('x');
    if (x == std::string_view::npos || text.find('x', x + 1u) != std::string_view::npos) return std::nullopt;
    const std::optional<uint32_t> w = parse_count(text.substr(0, x));
    const std::optional<uint32_t> h = parse_count(text.substr(x + 1u));
    if (!w || !h) return std::nullopt;
    const auto ok = [](uint32_t v) { return v >= 64u && v <= 16384u; };
    if (!ok(*w) || !ok(*h)) return std::nullopt;
    return std::make_pair(*w, *h);
}

// The file name of a diagnostic still (another Cs, another size): the clip, the frame, the Cs, the size and the look, so two
// of them cannot overwrite one another or a delivered master's still.
[[nodiscard]] inline std::string diag_still_name(std::string_view clip_id, uint32_t frame, float sgs_cs, uint32_t width,
                                                 uint32_t height, const Look& look) {
    std::string name = std::format("{}-f{}-cs{:g}-{}x{}", clip_id, frame, sgs_cs, width, height);
    if (look.l2_level > 0.0f) name += std::format("-l2-{:g}", look.l2_level);
    if (look.l2_smooth) name += "-smooth";
    if (look.min_cells > 0u) name += std::format("-cull{}", look.min_cells);
    if (look.mean_frames > 1u) name += std::format("-mean{}", look.mean_frames);
    return name + ".png";
}

// What a run asks of streaming, for stream_refusal().
struct StreamAsk {
    bool stream = false;
    bool keep_given = false;      // --keep-frames
    bool d1 = false;              // the clip runs the D1 film on the device (S5 and S6 run the quad's CPU film)
    bool deliver = false;         // --web-deliver
    bool export_lines = false;    // --export-lines
    bool simulate_only = false;   // --simulate-only
    bool frames_on_disk = false;  // the clip's cache folder already holds frames
    bool resimulate = false;      // --resimulate: the folder is cleared first
};

// Why the request cannot stream ("" when it can, or when it does not ask to).
[[nodiscard]] inline std::string stream_refusal(const StreamAsk& ask) {
    if (ask.keep_given && !ask.stream) return "--keep-frames needs --stream: without it every frame stays on disk";
    if (!ask.stream) return {};
    if (!ask.d1) return "--stream is for the D1 clips (S1 to S4 and S8); S5 and S6 run the reference quad's CPU film";
    if (ask.deliver) return "--stream draws a clip as it solves; --web-deliver works from a finished clip's files, so run it after";
    if (ask.export_lines) return "--export-lines reads a kept frame; run it after the streamed run, without --stream";
    if (ask.simulate_only) return "--simulate-only keeps every frame and --stream deletes them as they are drawn: use one";
    if (ask.frames_on_disk && !ask.resimulate) {
        return "the cache folder already holds frames, and a streamed run starts from none; --resimulate starts it again";
    }
    return {};
}

// The draws of a particle run: the clip's and still's, and every frame from 0 to the last of them, each once, sorted. The points
// are carried through all of them in order, so the plan writes the frames before the clip and the ones between the clip and its
// still too (a frame no draw reads is solved and dropped, and a missing frame would end the carry).
[[nodiscard]] inline std::vector<uint32_t> with_every_frame(const std::vector<uint32_t>& draws) {
    std::vector<uint32_t> all;
    if (draws.empty()) return all;
    const uint32_t last = *std::max_element(draws.begin(), draws.end());
    all.reserve(last + 1u);
    for (uint32_t j = 0; j <= last; ++j) all.push_back(j);
    return all;
}

// THE DISK GUARD (the lead, 2026-11-17). A non-streamed run writes every frame of its solve, 253 MB a frame in the deep box: a
// frame it should not have asked for once simulated and wrote 152 GB onto a drive with 45 GB free, taking it to 3.5 GB and
// close to failing every job on the machine. It refuses first, by name, when what it would still write leaves under the floor.
inline constexpr uint64_t kDiskFloorBytes = 20000000000ull;  // 20 GB

// A cached frame's size: 16 bytes a cell (u, v, w, dye and the four scalar channels, as halves) and a small header.
[[nodiscard]] inline uint64_t frame_bytes(uint32_t nx, uint32_t ny, uint32_t nz) {
    return uint64_t{nx} * ny * nz * 16u + 4096u;
}

// "" when `frames_to_write` frames of `bytes_per_frame` leave at least `floor_bytes` of `free_bytes`; else why not, naming
// --stream, which keeps only the frames the clip still needs.
[[nodiscard]] inline std::string disk_refusal(uint64_t frames_to_write, uint64_t bytes_per_frame, uint64_t free_bytes,
                                              uint64_t floor_bytes = kDiskFloorBytes) {
    if (frames_to_write == 0u) return {};
    const uint64_t need = frames_to_write * bytes_per_frame;
    if (free_bytes >= need && free_bytes - need >= floor_bytes) return {};
    return std::format(
        "a run without --stream would write {} more frame(s) of {:.2f} GB, {:.0f} GB in all, and the drive has {:.0f} GB free: "
        "that would leave under {:.0f} GB. Use --stream (it keeps only the frames the clip still needs), or free the space",
        frames_to_write, static_cast<double>(bytes_per_frame) / 1e9, static_cast<double>(need) / 1e9,
        static_cast<double>(free_bytes) / 1e9, static_cast<double>(floor_bytes) / 1e9);
}

}  // namespace flow_movie
