// Whole-file writes and shared reads (see the header).

#include "analysis/detail/atomic_file.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <thread>

#include "core/path_text.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace spade::analysis::detail {

namespace {

namespace fs = std::filesystem;

// How long a writer or reader waits out the other on Windows: 100 tries
// 10 ms apart. Host-side file I/O, not stepped code.
constexpr int kTries = 100;
constexpr std::chrono::milliseconds kPause{10};

[[nodiscard]] unsigned long process_id() {
#ifdef _WIN32
    return static_cast<unsigned long>(GetCurrentProcessId());
#else
    return static_cast<unsigned long>(getpid());
#endif
}

// A name beside `file` no other writer in this or another process uses.
[[nodiscard]] fs::path temporary_beside(const fs::path& file) {
    static std::atomic<unsigned long> counter{0};
    fs::path tmp = file;
    tmp += ".tmp-" + std::to_string(process_id()) + "-" + std::to_string(counter.fetch_add(1));
    return tmp;
}

[[nodiscard]] Error io(std::string what, const fs::path& file) {
    return Error{Code::io_error, std::move(what) + " '" + path_text(file) + "'"};
}

}  // namespace

Result<void> replace_file(const fs::path& file, std::string_view text) {
    const fs::path tmp = temporary_beside(file);
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) return std::unexpected(io("cannot write", tmp));
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
        out.flush();
        if (!out) {
            out.close();
            std::error_code ec;
            fs::remove(tmp, ec);
            return std::unexpected(io("failed writing", tmp));
        }
    }
#ifdef _WIN32
    DWORD last = 0;
    for (int i = 0; i < kTries; ++i) {
        if (MoveFileExW(tmp.c_str(), file.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0) {
            return {};
        }
        last = GetLastError();
        if (last != ERROR_ACCESS_DENIED && last != ERROR_SHARING_VIOLATION) break;
        std::this_thread::sleep_for(kPause);  // a reader holds it open
    }
    std::error_code ec;
    fs::remove(tmp, ec);
    return std::unexpected(io("cannot replace (Windows error " + std::to_string(last) + ")", file));
#else
    std::error_code ec;
    fs::rename(tmp, file, ec);  // rename(2): atomic, and a reader keeps what it opened
    if (ec) {
        std::error_code rm;
        fs::remove(tmp, rm);
        return std::unexpected(io("cannot replace (" + ec.message() + ")", file));
    }
    return {};
#endif
}

Result<std::string> read_file_shared(const fs::path& file) {
#ifdef _WIN32
    HANDLE h = INVALID_HANDLE_VALUE;
    DWORD last = 0;
    for (int i = 0; i < kTries; ++i) {
        h = CreateFileW(file.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (h != INVALID_HANDLE_VALUE) break;
        last = GetLastError();
        if (last == ERROR_FILE_NOT_FOUND || last == ERROR_PATH_NOT_FOUND) return std::unexpected(io("no file", file));
        if (last != ERROR_ACCESS_DENIED && last != ERROR_SHARING_VIOLATION) break;
        std::this_thread::sleep_for(kPause);  // a replace is in flight
    }
    if (h == INVALID_HANDLE_VALUE) {
        return std::unexpected(io("cannot open (Windows error " + std::to_string(last) + ")", file));
    }
    std::string text;
    LARGE_INTEGER size{};
    if (GetFileSizeEx(h, &size) == 0 || size.QuadPart < 0) {
        CloseHandle(h);
        return std::unexpected(io("cannot size", file));
    }
    text.resize(static_cast<std::size_t>(size.QuadPart));
    std::size_t got = 0;
    while (got < text.size()) {
        DWORD n = 0;
        const DWORD want = static_cast<DWORD>(std::min<std::size_t>(text.size() - got, 1u << 30));
        if (ReadFile(h, text.data() + got, want, &n, nullptr) == 0 || n == 0) break;
        got += n;
    }
    CloseHandle(h);
    if (got != text.size()) return std::unexpected(io("short read of", file));
    return text;
#else
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        std::error_code ec;
        return std::unexpected(io(fs::exists(file, ec) ? "cannot open" : "no file", file));
    }
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
#endif
}

}  // namespace spade::analysis::detail
