#include "fc_link/run_support.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iterator>
#include <vector>

#include "world/sha256.hpp"

#ifdef _WIN32
#ifndef PSAPI_VERSION
#define PSAPI_VERSION 2  // GetProcessMemoryInfo from kernel32 itself: no psapi.lib
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#endif

namespace spade::fc_link {

namespace {

[[nodiscard]] std::string_view trim_blanks(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) s.remove_suffix(1);
    return s;
}

}  // namespace

Result<uint64_t> parse_peak_resident_kb(std::string_view text) {
    constexpr std::string_view kKey = "VmHWM:";
    std::size_t pos = 0;
    while (pos < text.size()) {
        const std::size_t eol = text.find('\n', pos);
        const std::string_view line = text.substr(pos, eol == std::string_view::npos ? std::string_view::npos : eol - pos);
        pos = eol == std::string_view::npos ? text.size() : eol + 1;
        if (!line.starts_with(kKey)) continue;
        const std::string_view rest = trim_blanks(line.substr(kKey.size()));
        uint64_t kb = 0;
        const char* first = rest.data();
        const char* last = rest.data() + rest.size();
        const std::from_chars_result r = std::from_chars(first, last, kb);
        if (r.ec != std::errc() || r.ptr == first || trim_blanks(std::string_view(r.ptr, static_cast<std::size_t>(last - r.ptr))) != "kB") {
            return std::unexpected(Error{Code::invalid_argument, "parse_peak_resident_kb: the VmHWM line is not '<number> kB'"});
        }
        if (kb == 0) return std::unexpected(Error{Code::invalid_argument, "parse_peak_resident_kb: the VmHWM line says 0 kB, which is no measurement"});
        return kb * 1024ull;
    }
    return std::unexpected(Error{Code::not_found, "parse_peak_resident_kb: no VmHWM line"});
}

Result<uint64_t> measure_memory_peak_bytes() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS counters{};
    counters.cb = sizeof(counters);
    if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) == 0 || counters.PeakWorkingSetSize == 0) {
        return std::unexpected(Error{Code::unavailable, "measure_memory_peak_bytes: the process's peak working set cannot be read"});
    }
    return static_cast<uint64_t>(counters.PeakWorkingSetSize);
#elif defined(__linux__)
    std::ifstream in("/proc/self/status");
    if (!in) return std::unexpected(Error{Code::unavailable, "measure_memory_peak_bytes: /proc/self/status cannot be read"});
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return parse_peak_resident_kb(text);
#else
    return std::unexpected(Error{Code::unavailable, "measure_memory_peak_bytes: no way to read the peak memory on this platform"});
#endif
}

Result<std::string> hash_file(const std::filesystem::path& file) {
    // The errors say what failed and never which file: the caller knows, and a record holds no folder of the user's machine.
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::unexpected(Error{Code::io_error, "hash_file: the file cannot be opened"});
    const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (in.bad()) return std::unexpected(Error{Code::io_error, "hash_file: the file cannot be read"});
    return sha256_hex(bytes);
}

Result<fc_host::StageLatency> summarise_stage(std::string_view name, std::span<const double> samples_ns) {
    const bool name_ok = !name.empty() && std::all_of(name.begin(), name.end(), [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
    if (!name_ok) return std::unexpected(Error{Code::invalid_argument, "summarise_stage: the name must be [a-z0-9_]+"});
    if (samples_ns.empty()) return std::unexpected(Error{Code::invalid_argument, "summarise_stage: no samples"});
    for (const double s : samples_ns) {
        if (!std::isfinite(s) || s < 0.0) return std::unexpected(Error{Code::invalid_argument, "summarise_stage: a sample is not a finite, non-negative duration"});
    }
    std::vector<double> sorted(samples_ns.begin(), samples_ns.end());
    std::sort(sorted.begin(), sorted.end());
    double sum = 0.0;
    for (const double s : sorted) sum += s;
    const std::size_t n = sorted.size();
    const std::size_t rank = (99 * n + 99) / 100;  // ceil(0.99 n), in integers
    fc_host::StageLatency out;
    out.name = std::string(name);
    out.min_ns = sorted.front();
    out.mean_ns = sum / static_cast<double>(n);
    out.p99_ns = sorted[rank - 1];
    out.max_ns = sorted.back();
    return out;
}

}  // namespace spade::fc_link
