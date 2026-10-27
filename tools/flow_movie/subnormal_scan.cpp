#include "subnormal_scan.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__) || defined(__SSE__)
#include <xmmintrin.h>
#define FLOW_MOVIE_HAS_MXCSR 1
#else
#define FLOW_MOVIE_HAS_MXCSR 0
#endif

// The locator traps, which needs Windows' vectored handlers and x64's unwinder.
#if FLOW_MOVIE_HAS_MXCSR && defined(_WIN32) && defined(_M_X64)
#define FLOW_MOVIE_HAS_LOCATE 1
#else
#define FLOW_MOVIE_HAS_LOCATE 0
#endif

#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "physics/airflow/grid_io.hpp"

#if FLOW_MOVIE_HAS_LOCATE
// After the engine's headers, so none of them meets windows.h's macros.
#include <windows.h>
#endif

namespace flow_movie {

#if FLOW_MOVIE_HAS_MXCSR

namespace {

namespace fs = std::filesystem;
namespace an = spade::analysis;
namespace af = spade::physics::airflow;

constexpr unsigned kFlagMask = 0x3Fu;   // MXCSR's six sticky exception flags
constexpr unsigned kDenormal = 0x02u;   // DE: an operand was subnormal
constexpr unsigned kUnderflow = 0x10u;  // UE: a result was tiny and inexact
constexpr unsigned kFlush = 0x8040u;    // FTZ (bit 15) and DAZ (bit 6)

// Clears the sticky flags; restores the whole MXCSR on every exit.
class ScopedMxcsr {
public:
    explicit ScopedMxcsr(bool flush) : saved_(_mm_getcsr()) {
        _mm_setcsr((saved_ & ~kFlagMask) | (flush ? kFlush : 0u));
    }
    ~ScopedMxcsr() { _mm_setcsr(saved_); }
    ScopedMxcsr(const ScopedMxcsr&) = delete;
    ScopedMxcsr& operator=(const ScopedMxcsr&) = delete;
    [[nodiscard]] static unsigned flags() { return _mm_getcsr() & kFlagMask; }

private:
    unsigned saved_;
};

// One run's flags over the scan.
struct FlagTally {
    uint32_t denormal_steps = 0;
    uint32_t first_denormal = 0;  // 0: none
    uint32_t underflow_steps = 0;
    uint32_t first_underflow = 0;

    void add(unsigned flags, uint32_t step) {
        if ((flags & kDenormal) != 0u) {
            ++denormal_steps;
            if (first_denormal == 0u) first_denormal = step;
        }
        if ((flags & kUnderflow) != 0u) {
            ++underflow_steps;
            if (first_underflow == 0u) first_underflow = step;
        }
    }
};

template <class T>
[[nodiscard]] T read_at(std::string_view s, std::size_t at) {
    T v;
    std::memcpy(&v, s.data() + at, sizeof v);
    return v;
}

constexpr std::array<const char*, 4> kArrays{"u", "v", "w", "p"};

// The grid's arrays in a LiveRun save: LiveRun::save() writes the grid's
// blob (grid_io.hpp) last, after its u64 length and before the save's u64
// hash. Found by the blob's magic and checked against its header, so a save
// laid out otherwise is refused, never misread.
struct GridInSave {
    uint32_t nx = 0, ny = 0, nz = 0;
    std::array<std::size_t, 4> at{};  // u, v, w, p's first byte
    std::array<uint64_t, 4> count{};
};

[[nodiscard]] std::optional<GridInSave> grid_in_save(std::string_view save, std::string& why) {
    const std::size_t at = save.rfind("SPADEGRD");
    if (at == std::string_view::npos || at < 8u || save.size() < at + af::kGridBlobHeaderBytes + 8u) {
        why = "no grid blob before the save's hash";
        return std::nullopt;
    }
    if (read_at<uint64_t>(save, at - 8u) != save.size() - 8u - at) {
        why = "the grid blob's length is not what runs to the save's hash";
        return std::nullopt;
    }
    if (read_at<uint32_t>(save, at + 8u) != af::kGridBlobVersion) {
        why = "the grid blob is version " + std::to_string(read_at<uint32_t>(save, at + 8u));
        return std::nullopt;
    }
    GridInSave g;
    g.nx = read_at<uint32_t>(save, at + 12u);
    g.ny = read_at<uint32_t>(save, at + 16u);
    g.nz = read_at<uint32_t>(save, at + 20u);
    std::size_t next = at + af::kGridBlobHeaderBytes;
    for (std::size_t a = 0; a < 4u; ++a) {
        g.count[a] = read_at<uint64_t>(save, at + 28u + 8u * a);
        g.at[a] = next;
        next += sizeof(float) * g.count[a];
    }
    if (next != save.size() - 8u) {
        why = "the grid blob's arrays don't fill it";
        return std::nullopt;
    }
    return g;
}

// One array's subnormals over the scan, by bits, so the scan itself performs
// no float operation.
struct ArrayTally {
    uint64_t subnormals = 0;
    uint64_t this_step = 0;
    uint32_t first_step = 0;                  // 0: none
    uint32_t smallest = 0x7F800000u;          // the smallest nonzero magnitude's bits; inf: none
    uint32_t smallest_subnormal = 0x7F800000u;

    void scan(std::string_view save, std::size_t at, uint64_t count, uint32_t step) {
        this_step = 0;
        for (uint64_t i = 0; i < count; ++i) {
            const uint32_t b = read_at<uint32_t>(save, at + sizeof(float) * i) & 0x7FFFFFFFu;
            if (b == 0u || b >= 0x7F800000u) continue;
            smallest = std::min(smallest, b);
            if (b < 0x00800000u) {
                ++this_step;
                smallest_subnormal = std::min(smallest_subnormal, b);
            }
        }
        subnormals += this_step;
        if (this_step > 0u && first_step == 0u) first_step = step;
    }
};

// Where two saves first differ; empty when they are equal.
[[nodiscard]] std::string first_difference(std::string_view a, std::string_view b, const GridInSave& g) {
    if (a == b) return {};
    if (a.size() != b.size()) return "their sizes";
    const auto at = static_cast<std::size_t>(std::mismatch(a.begin(), a.end(), b.begin()).first - a.begin());
    for (std::size_t k = 0; k < 4u; ++k) {
        if (at >= g.at[k] && at < g.at[k] + sizeof(float) * g.count[k]) return std::string("the grid's ") + kArrays[k];
    }
    return "the run's state before the grid (body, rotors, controls)";
}

// Each array's largest |a - b| over two saves of one layout.
[[nodiscard]] std::array<double, 4> max_difference(std::string_view a, std::string_view b, const GridInSave& g) {
    std::array<double, 4> out{};
    if (a.size() != b.size()) return out;
    for (std::size_t k = 0; k < 4u; ++k) {
        for (uint64_t i = 0; i < g.count[k]; ++i) {
            const std::size_t at = g.at[k] + sizeof(float) * i;
            const double d = std::fabs(static_cast<double>(read_at<float>(a, at)) - read_at<float>(b, at));
            if (d > out[k]) out[k] = d;
        }
    }
    return out;
}

[[nodiscard]] std::optional<std::string> read_whole(const fs::path& file) {
    std::ifstream f(file, std::ios::binary | std::ios::ate);
    if (!f) return std::nullopt;
    std::string out(static_cast<std::size_t>(f.tellg()), '\0');
    f.seekg(0);
    f.read(out.data(), static_cast<std::streamsize>(out.size()));
    if (!f) return std::nullopt;
    return out;
}

[[nodiscard]] std::string first_text(uint32_t step) { return step == 0u ? "none" : "step " + std::to_string(step); }

[[nodiscard]] std::string magnitude_text(uint32_t bits) {
    if (bits >= 0x7F800000u) return "none";
    char text[32];
    std::snprintf(text, sizeof text, "%.3e", static_cast<double>(std::bit_cast<float>(bits)));
    return text;
}

[[nodiscard]] std::string differences_text(const std::array<double, 4>& d) {
    char text[128];
    std::snprintf(text, sizeof text, "u %.3e, v %.3e, w %.3e, p %.3e", d[0], d[1], d[2], d[3]);
    return text;
}

// Where a run starts: "from rest", or the save and the step and time it holds.
[[nodiscard]] std::string from_text(const fs::path& from_save, const an::LiveRun& run) {
    if (from_save.empty()) return "from rest";
    return "from " + from_save.string() + " (step " + std::to_string(run.state().steps) + ", t " +
           std::to_string(run.state().t_s) + " s)";
}

}  // namespace

int subnormal_scan(const LiveCase& lc, uint32_t steps, const fs::path& from_save, const fs::path& work_dir) {
    using Clock = std::chrono::steady_clock;
    auto resolved = resolve_live(lc);
    if (!resolved) {
        std::fprintf(stderr, "flow_movie: %s\n", resolved.error().context.c_str());
        return 1;
    }
    std::error_code ec;
    fs::create_directories(work_dir, ec);
    const std::array<fs::path, 2> saves{work_dir / "subnormal-scan-default.save",
                                        work_dir / "subnormal-scan-flushed.save"};
    const auto open_run = [&]() {
        return from_save.empty() ? an::LiveRun::start(*resolved, lc.run_model)
                                 : an::LiveRun::restore(from_save, *resolved, lc.run_model);
    };
    // Both runs start (or restore) unflushed: the host builds the state, the
    // twin flushes only in step(), as the GPU would step it.
    unsigned open_flags = 0u;
    spade::Result<an::LiveRun> plain = [&]() {
        const ScopedMxcsr scope(false);
        auto run = open_run();
        open_flags = ScopedMxcsr::flags();
        return run;
    }();
    spade::Result<an::LiveRun> flushed = open_run();
    if (!plain || !flushed) {
        std::fprintf(stderr, "flow_movie: %s\n", (!plain ? plain : flushed).error().context.c_str());
        return 1;
    }
    std::array<an::LiveRun*, 2> runs{&*plain, &*flushed};
    const af::GridShape& shape = resolved->base;
    const std::string from = from_text(from_save, *plain);
    std::printf("[ subnormal scan ] %s: %u x %u x %u at %.4f m, h %.3f ms; %u steps %s\n", lc.name.c_str(), shape.nx,
                shape.ny, shape.nz, static_cast<double>(shape.dx), plain->step_s() * 1e3, steps, from.c_str());
    std::fflush(stdout);

    std::array<FlagTally, 2> flags{};
    std::array<ArrayTally, 4> arrays{};
    uint32_t differs_from = 0;  // the first step whose saves differ; 0: none
    std::string differs_where;
    std::array<double, 4> differs_then{};
    std::array<double, 4> differs_last{};
    for (uint32_t s = 1; s <= steps; ++s) {
        const Clock::time_point t0 = Clock::now();
        std::array<unsigned, 2> f{};
        for (std::size_t r = 0; r < 2u; ++r) {
            const an::Controls controls = an::controls_at(lc.program, static_cast<float>(runs[r]->state().t_s));
            spade::Result<void> stepped;
            {
                const ScopedMxcsr scope(r == 1u);
                stepped = runs[r]->step(controls);
                f[r] = ScopedMxcsr::flags();
            }
            if (!stepped) {
                std::fprintf(stderr, "flow_movie: step %u, %s run: %s\n", s, r == 0u ? "default" : "flushed",
                             stepped.error().context.c_str());
                return 1;
            }
            if (auto saved = runs[r]->save(saves[r]); !saved) {
                std::fprintf(stderr, "flow_movie: %s\n", saved.error().context.c_str());
                return 1;
            }
            flags[r].add(f[r], s);
        }
        const std::optional<std::string> a = read_whole(saves[0]);
        const std::optional<std::string> b = read_whole(saves[1]);
        if (!a || !b) {
            std::fprintf(stderr, "flow_movie: could not read %s back\n", (!a ? saves[0] : saves[1]).string().c_str());
            return 1;
        }
        std::string why;
        const std::optional<GridInSave> g = grid_in_save(*a, why);
        if (!g) {
            std::fprintf(stderr, "flow_movie: %s: %s\n", saves[0].string().c_str(), why.c_str());
            return 1;
        }
        for (std::size_t k = 0; k < 4u; ++k) arrays[k].scan(*a, g->at[k], g->count[k], s);
        const std::string where = first_difference(*a, *b, *g);
        std::string twin = "the same";
        if (!where.empty()) {
            differs_last = max_difference(*a, *b, *g);
            if (differs_from == 0u) {
                differs_from = s;
                differs_where = where;
                differs_then = differs_last;
            }
            twin = "differs in " + where + " (max |d| " + differences_text(differs_last) + ")";
        }
        std::printf("[ subnormal scan ] step %u (run step %llu): DE %s, UE %s; grid subnormals u %llu v %llu w %llu "
                    "p %llu; twin %s; residual %.2e / %.2e; %.1f s\n",
                    s, static_cast<unsigned long long>(plain->state().steps), (f[0] & kDenormal) != 0u ? "yes" : "no",
                    (f[0] & kUnderflow) != 0u ? "yes" : "no", static_cast<unsigned long long>(arrays[0].this_step),
                    static_cast<unsigned long long>(arrays[1].this_step),
                    static_cast<unsigned long long>(arrays[2].this_step),
                    static_cast<unsigned long long>(arrays[3].this_step), twin.c_str(),
                    static_cast<double>(plain->solids().pressure_residual),
                    static_cast<double>(flushed->solids().pressure_residual),
                    std::chrono::duration<double>(Clock::now() - t0).count());
        std::fflush(stdout);
    }

    std::printf("=== subnormal scan: %s, %u steps %s ===\n", lc.name.c_str(), steps, from.c_str());
    std::printf("grid: %u x %u x %u at %.4f m, h %.3f ms\n", shape.nx, shape.ny, shape.nz,
                static_cast<double>(shape.dx), plain->step_s() * 1e3);
    std::printf("flags around %s, the default run: DE %s, UE %s\n", from_save.empty() ? "start()" : "restore()",
                (open_flags & kDenormal) != 0u ? "yes" : "no", (open_flags & kUnderflow) != 0u ? "yes" : "no");
    const char* names[2] = {"the default run", "the flushed twin"};
    for (std::size_t r = 0; r < 2u; ++r) {
        std::printf("flags around step(), %s: DE on %u of %u steps (first: %s); UE on %u of %u steps (first: %s)\n",
                    names[r], flags[r].denormal_steps, steps, first_text(flags[r].first_denormal).c_str(),
                    flags[r].underflow_steps, steps, first_text(flags[r].first_underflow).c_str());
    }
    for (std::size_t k = 0; k < 4u; ++k) {
        std::printf("grid %s, the default run: %llu subnormals over the scan (first: %s); smallest nonzero |x| %s; "
                    "smallest subnormal %s\n",
                    kArrays[k], static_cast<unsigned long long>(arrays[k].subnormals),
                    first_text(arrays[k].first_step).c_str(), magnitude_text(arrays[k].smallest).c_str(),
                    magnitude_text(arrays[k].smallest_subnormal).c_str());
    }
    if (differs_from == 0u) {
        std::printf("flushed twin (FTZ and DAZ in step()): the same state in every bit through step %u\n", steps);
    } else {
        std::printf("flushed twin (FTZ and DAZ in step()): first differs at step %u, in %s; max |d| then %s; at step "
                    "%u %s\n",
                    differs_from, differs_where.c_str(), differences_text(differs_then).c_str(), steps,
                    differences_text(differs_last).c_str());
    }
    std::printf("residual at step %u: default %.3e, flushed %.3e\n", steps,
                static_cast<double>(plain->solids().pressure_residual),
                static_cast<double>(flushed->solids().pressure_residual));
    return 0;
}

#else

int subnormal_scan(const LiveCase&, uint32_t, const std::filesystem::path&, const std::filesystem::path&) {
    std::fprintf(stderr, "flow_movie: --subnormal-scan reads x86's MXCSR; this build has none\n");
    return 77;
}

#endif

#if FLOW_MOVIE_HAS_LOCATE

namespace {

constexpr unsigned kDenormalMask = 0x0100u;   // DM: DE masked
constexpr unsigned kUnderflowMask = 0x0800u;  // UM: UE masked
constexpr std::size_t kFrames = 8;

// One kind's first fault in a step: the faulting instruction, then the
// return addresses under it.
struct Fault {
    bool seen = false;
    std::size_t depth = 0;
    std::array<uint64_t, kFrames> frames{};
};

// Written only by the handler, on the locating thread.
struct Trap {
    DWORD thread = 0;
    Fault denormal;
    Fault underflow;
    Fault unclassified;  // a float fault MXCSR doesn't name
};
Trap trap_state;

// The calls under a fault, unwound from its context (no allocation: this runs
// in the handler).
void unwind(const CONTEXT& at, Fault& out) {
    CONTEXT c = at;
    out.depth = 0;
    while (out.depth < kFrames && c.Rip != 0) {
        out.frames[out.depth++] = c.Rip;
        DWORD64 image = 0;
        PRUNTIME_FUNCTION fn = RtlLookupFunctionEntry(c.Rip, &image, nullptr);
        if (fn == nullptr) {  // a leaf: its return address is at the stack's top
            c.Rip = *reinterpret_cast<const DWORD64*>(c.Rsp);
            c.Rsp += 8;
            continue;
        }
        void* data = nullptr;
        DWORD64 frame = 0;
        RtlVirtualUnwind(UNW_FLAG_NHANDLER, image, c.Rip, fn, &c, &data, &frame, nullptr);
    }
}

// Windows names an SSE fault by a flag MXCSR holds, which may be a masked
// one set earlier in the step (a fault on DE came as INVALID_OPERATION), so
// every float code is taken and the kind read from MXCSR: a flag set whose
// mask is clear.
[[nodiscard]] bool float_fault(DWORD code) {
    switch (code) {
    case STATUS_FLOAT_DENORMAL_OPERAND:
    case STATUS_FLOAT_DIVIDE_BY_ZERO:
    case STATUS_FLOAT_INEXACT_RESULT:
    case STATUS_FLOAT_INVALID_OPERATION:
    case STATUS_FLOAT_OVERFLOW:
    case STATUS_FLOAT_UNDERFLOW:
    case STATUS_FLOAT_MULTIPLE_FAULTS:
    case STATUS_FLOAT_MULTIPLE_TRAPS:
        return true;
    default:
        return false;
    }
}

// The first DE and the first UE of a step: recorded, that kind masked, and
// the instruction run again, masked, as the default run runs it.
LONG CALLBACK on_fault(EXCEPTION_POINTERS* e) {
    if (!float_fault(e->ExceptionRecord->ExceptionCode) || GetCurrentThreadId() != trap_state.thread) {
        return EXCEPTION_CONTINUE_SEARCH;
    }
    CONTEXT& c = *e->ContextRecord;
    DWORD csr = c.MxCsr;
    bool ours = false;
    if ((csr & kDenormal) != 0u && (csr & kDenormalMask) == 0u) {
        if (!trap_state.denormal.seen) unwind(c, trap_state.denormal);
        trap_state.denormal.seen = true;
        csr |= kDenormalMask;
        ours = true;
    }
    if ((csr & kUnderflow) != 0u && (csr & kUnderflowMask) == 0u) {
        if (!trap_state.underflow.seen) unwind(c, trap_state.underflow);
        trap_state.underflow.seen = true;
        csr |= kUnderflowMask;
        ours = true;
    }
    // A float fault here that MXCSR doesn't name: only DE and UE are
    // unmasked, so it is one of them; recorded unclassified, both masked.
    if (!ours && (csr & (kDenormalMask | kUnderflowMask)) != (kDenormalMask | kUnderflowMask)) {
        if (!trap_state.unclassified.seen) unwind(c, trap_state.unclassified);
        trap_state.unclassified.seen = true;
        csr |= kDenormalMask | kUnderflowMask;
        ours = true;
    }
    if (!ours) return EXCEPTION_CONTINUE_SEARCH;
    c.MxCsr = csr;
    c.FltSave.MxCsr = csr;
    return EXCEPTION_CONTINUE_EXECUTION;
}

// The link's map (/MAP): each symbol's address from the image's base, its
// name and its object file.
struct MapSymbol {
    uint64_t rva = 0;
    std::string name;
    std::string object;
};

[[nodiscard]] std::vector<MapSymbol> read_map(const fs::path& file) {
    std::ifstream in(file);
    std::vector<MapSymbol> out;
    uint64_t preferred = 0;
    constexpr std::string_view kBase = "Preferred load address is ";
    for (std::string line; std::getline(in, line);) {
        if (const std::size_t at = line.find(kBase); at != std::string::npos) {
            const std::string_view hex = std::string_view(line).substr(at + kBase.size());
            if (std::from_chars(hex.data(), hex.data() + std::min<std::size_t>(hex.size(), 16u), preferred, 16).ec !=
                std::errc{}) {
                preferred = 0;
            }
            continue;
        }
        std::istringstream words(line);
        std::vector<std::string> w;
        for (std::string s; words >> s;) w.push_back(s);
        if (w.size() < 4u || w[0].size() != 13u || w[0][4] != ':' || w[2].size() != 16u) continue;
        uint64_t va = 0;
        if (std::from_chars(w[2].data(), w[2].data() + 16, va, 16).ec != std::errc{}) continue;
        if (preferred == 0u || va < preferred) continue;
        out.push_back({va - preferred, w[1], w.back()});
    }
    std::sort(out.begin(), out.end(), [](const MapSymbol& a, const MapSymbol& b) { return a.rva < b.rva; });
    return out;
}

// An address as "function (object)" and its offset into it; another
// module's as its file's name.
struct Named {
    std::string function;
    uint64_t offset = 0;
};

[[nodiscard]] Named name_of(uint64_t address, const std::vector<MapSymbol>& map) {
    HMODULE module = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(address), &module);
    const auto base = reinterpret_cast<uint64_t>(module);
    if (module != GetModuleHandleW(nullptr)) {
        wchar_t file[MAX_PATH] = {};
        const DWORD n = module != nullptr ? GetModuleFileNameW(module, file, MAX_PATH) : 0u;
        return {n > 0u ? fs::path(std::wstring(file, n)).filename().string() : std::string("an unknown module"),
                address - base};
    }
    const uint64_t rva = address - base;
    const auto it = std::upper_bound(map.begin(), map.end(), rva,
                                     [](uint64_t r, const MapSymbol& s) { return r < s.rva; });
    if (it == map.begin()) return {"flow_movie.exe", rva};
    const MapSymbol& s = *std::prev(it);
    return {s.name + " (" + s.object + ")", rva - s.rva};
}

[[nodiscard]] std::string named_text(const Named& n) {
    char offset[32];
    std::snprintf(offset, sizeof offset, "+0x%llx", static_cast<unsigned long long>(n.offset));
    return n.function + offset;
}

struct Site {
    uint32_t steps = 0;
    uint32_t first = 0;
};

}  // namespace

int subnormal_locate(const LiveCase& lc, uint32_t steps, const fs::path& from_save) {
    using Clock = std::chrono::steady_clock;
    auto resolved = resolve_live(lc);
    if (!resolved) {
        std::fprintf(stderr, "flow_movie: %s\n", resolved.error().context.c_str());
        return 1;
    }
    auto run = from_save.empty() ? an::LiveRun::start(*resolved, lc.run_model)
                                 : an::LiveRun::restore(from_save, *resolved, lc.run_model);
    if (!run) {
        std::fprintf(stderr, "flow_movie: %s\n", run.error().context.c_str());
        return 1;
    }
    wchar_t exe[MAX_PATH] = {};
    const DWORD exe_chars = GetModuleFileNameW(nullptr, exe, MAX_PATH);
    const fs::path map_file = fs::path(std::wstring(exe, exe_chars)).replace_extension(".map");
    const std::vector<MapSymbol> map = read_map(map_file);
    const std::string from = from_text(from_save, *run);
    std::printf("[ subnormal locate ] %s: %u steps %s; %zu symbols from %s\n", lc.name.c_str(), steps, from.c_str(),
                map.size(), map_file.string().c_str());
    std::fflush(stdout);

    trap_state = Trap{};
    trap_state.thread = GetCurrentThreadId();
    void* handler = AddVectoredExceptionHandler(1, on_fault);
    if (handler == nullptr) {
        std::fprintf(stderr, "flow_movie: could not add the fault handler\n");
        return 1;
    }
    std::map<std::string, Site> denormal_sites;
    std::map<std::string, Site> underflow_sites;
    std::map<std::string, Site> unclassified_sites;
    const auto report = [&](const char* kind, const Fault& f, std::map<std::string, Site>& sites, uint32_t s) {
        if (!f.seen) return;
        std::string chain = named_text(name_of(f.frames[0], map));
        for (std::size_t i = 1; i < f.depth; ++i) chain += "\n      <- " + named_text(name_of(f.frames[i], map));
        std::printf("  the step's first %s: %s\n", kind, chain.c_str());
        Site& site = sites[name_of(f.frames[0], map).function];
        ++site.steps;
        if (site.first == 0u) site.first = s;
    };
    int rc = 0;
    for (uint32_t s = 1; s <= steps; ++s) {
        const an::Controls controls = an::controls_at(lc.program, static_cast<float>(run->state().t_s));
        trap_state.denormal = Fault{};
        trap_state.underflow = Fault{};
        trap_state.unclassified = Fault{};
        const Clock::time_point t0 = Clock::now();
        const unsigned saved = _mm_getcsr();
        _mm_setcsr(saved & ~(kFlagMask | kDenormalMask | kUnderflowMask));
        const spade::Result<void> stepped = run->step(controls);
        const unsigned flags = _mm_getcsr() & kFlagMask;
        _mm_setcsr(saved);
        if (!stepped) {
            std::fprintf(stderr, "flow_movie: step %u: %s\n", s, stepped.error().context.c_str());
            rc = 1;
            break;
        }
        std::printf("[ subnormal locate ] step %u (run step %llu): DE %s, UE %s; %.1f s\n", s,
                    static_cast<unsigned long long>(run->state().steps), (flags & kDenormal) != 0u ? "yes" : "no",
                    (flags & kUnderflow) != 0u ? "yes" : "no",
                    std::chrono::duration<double>(Clock::now() - t0).count());
        report("DE", trap_state.denormal, denormal_sites, s);
        report("UE", trap_state.underflow, underflow_sites, s);
        report("DE or UE, unclassified", trap_state.unclassified, unclassified_sites, s);
        std::fflush(stdout);
    }
    RemoveVectoredExceptionHandler(handler);
    if (rc != 0) return rc;

    std::printf("=== subnormal locate: %s, %u steps %s ===\n", lc.name.c_str(), steps, from.c_str());
    std::printf("each step's first DE and first UE only: later ones in a step ran masked, unrecorded\n");
    const std::pair<const char*, const std::map<std::string, Site>*> kinds[3] = {
        {"DE", &denormal_sites}, {"UE", &underflow_sites}, {"DE or UE, unclassified", &unclassified_sites}};
    for (const auto& [kind, sites] : kinds) {
        if (sites->empty()) std::printf("first %s: none\n", kind);
        for (const auto& [function, site] : *sites) {
            std::printf("first %s in %s: on %u of %u steps (first: step %u)\n", kind, function.c_str(), site.steps,
                        steps, site.first);
        }
    }
    return 0;
}

#else

int subnormal_locate(const LiveCase&, uint32_t, const std::filesystem::path&) {
    std::fprintf(stderr, "flow_movie: --subnormal-locate traps on Windows x64 only\n");
    return 77;
}

#endif

}  // namespace flow_movie
