#include "cache.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <type_traits>

namespace flow_movie {
namespace {

namespace fs = std::filesystem;

constexpr char kMagic[8] = {'S', 'P', 'F', 'R', 'A', 'M', 'E', '1'};
static_assert(std::is_trivially_copyable_v<spade::FlowVolumeHeader>);

[[nodiscard]] fs::path frame_path(const fs::path& dir, uint32_t frame) {
    char name[32];
    std::snprintf(name, sizeof name, "frame-%05u.bin", frame);
    return dir / name;
}

[[nodiscard]] auto refuse(const std::string& why) {
    return std::unexpected(spade::Error{spade::Code::io_error, why});
}

template <class T>
void put(std::ofstream& out, const T& x) {
    out.write(reinterpret_cast<const char*>(&x), sizeof x);
}

template <class T>
[[nodiscard]] bool get(std::ifstream& in, T& x) {
    return static_cast<bool>(in.read(reinterpret_cast<char*>(&x), sizeof x));
}

}  // namespace

spade::Result<void> write_frame(const fs::path& dir, uint32_t frame, const spade::FlowVolume& volume,
                                const FrameLog& log) {
    const fs::path path = frame_path(dir, frame);
    const fs::path tmp = dir / "frame.tmp";
    {
        std::ofstream out(tmp, std::ios::binary);
        if (!out) return refuse("cannot write " + tmp.string());
        out.write(kMagic, sizeof kMagic);
        put(out, volume.header);
        put(out, log.step);
        put(out, log.time);
        put(out, static_cast<uint64_t>(log.live.size()));
        for (const double x : log.live) put(out, x);
        put(out, static_cast<uint64_t>(volume.v.size()));
        out.write(reinterpret_cast<const char*>(volume.v.data()), static_cast<std::streamsize>(volume.v.size() * 2u));
        out.write(reinterpret_cast<const char*>(volume.s.data()), static_cast<std::streamsize>(volume.s.size() * 2u));
        if (!out) return refuse("writing " + tmp.string() + " failed");
    }
    std::error_code ec;
    fs::rename(tmp, path, ec);  // atomic: a frame is whole or absent
    if (ec) return refuse("cannot write " + path.string() + ": " + ec.message());
    return {};
}

spade::Result<void> read_frame(const fs::path& dir, uint32_t frame, spade::FlowVolume& volume, FrameLog& log) {
    volume.generation = 0;  // unnamed until it is whole (core/flow_volume.hpp)
    const fs::path path = frame_path(dir, frame);
    std::ifstream in(path, std::ios::binary);
    char magic[sizeof kMagic] = {};
    if (!in || !in.read(magic, sizeof magic) || !std::equal(magic, magic + sizeof magic, kMagic)) {
        return refuse(path.string() + " is not a cached frame");
    }
    uint64_t live = 0, halves = 0;
    if (!get(in, volume.header) || !get(in, log.step) || !get(in, log.time) || !get(in, live) || live > 64u) {
        return refuse(path.string() + ": a short or malformed header");
    }
    log.live.resize(live);
    for (double& x : log.live) {
        if (!get(in, x)) return refuse(path.string() + ": short");
    }
    if (!get(in, halves) || halves > (uint64_t{1} << 34)) return refuse(path.string() + ": a malformed size");
    volume.v.resize(halves);
    volume.s.resize(halves);
    in.read(reinterpret_cast<char*>(volume.v.data()), static_cast<std::streamsize>(halves * 2u));
    in.read(reinterpret_cast<char*>(volume.s.data()), static_cast<std::streamsize>(halves * 2u));
    if (!in) return refuse(path.string() + ": short");
    if (auto valid = spade::validate_flow_volume(volume); !valid) return valid;
    volume.generation = spade::next_flow_generation();  // so a frame's views upload it once
    return {};
}

uint32_t cached_frames(const fs::path& dir) {
    uint32_t n = 0;
    std::error_code ec;
    while (fs::exists(frame_path(dir, n), ec)) ++n;
    return n;
}

void mark_cache(const fs::path& dir, const CacheMark& mark) {
    std::ofstream(dir / "film.txt", std::ios::binary) << mark.film << '\n' << mark.build << '\n';
}

CacheMark read_mark(const fs::path& dir) {
    std::ifstream in(dir / "film.txt", std::ios::binary);
    CacheMark mark;
    std::getline(in, mark.film);
    std::getline(in, mark.build);
    return mark;
}

spade::Result<void> write_state(const fs::path& dir, uint32_t frame, uint64_t step,
                                const spade::physics::airflow::MacGrid& g) {
    const fs::path tmp = dir / "state.tmp";
    {
        std::ofstream out(tmp, std::ios::binary);
        if (!out) return refuse("cannot write " + tmp.string());
        out.write(kMagic, sizeof kMagic);
        put(out, frame);
        put(out, step);
        for (const std::vector<float>* f : {&g.u, &g.v, &g.w, &g.p}) {
            put(out, static_cast<uint64_t>(f->size()));
            out.write(reinterpret_cast<const char*>(f->data()), static_cast<std::streamsize>(f->size() * 4u));
        }
        if (!out) return refuse("writing " + tmp.string() + " failed");
    }
    std::error_code ec;
    fs::rename(tmp, dir / "state.bin", ec);  // atomic: a kill leaves the old state or the new
    if (ec) return refuse("cannot replace " + (dir / "state.bin").string() + ": " + ec.message());
    return {};
}

spade::Result<uint32_t> read_state(const fs::path& dir, uint64_t& step, spade::physics::airflow::MacGrid& g) {
    const fs::path path = dir / "state.bin";
    std::ifstream in(path, std::ios::binary);
    char magic[sizeof kMagic] = {};
    uint32_t frame = 0;
    if (!in || !in.read(magic, sizeof magic) || !std::equal(magic, magic + sizeof magic, kMagic) || !get(in, frame) ||
        !get(in, step)) {
        return refuse(path.string() + " is not a solver state");
    }
    for (std::vector<float>* f : {&g.u, &g.v, &g.w, &g.p}) {
        uint64_t n = 0;
        if (!get(in, n) || n != f->size()) return refuse(path.string() + ": a field's size is not this grid's");
        if (!in.read(reinterpret_cast<char*>(f->data()), static_cast<std::streamsize>(n * 4u))) {
            return refuse(path.string() + ": short");
        }
    }
    return frame;
}

}  // namespace flow_movie
