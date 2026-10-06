// Temporary paths for tests that cannot collide: the one place that names them
// (TD-9).
//
// WHY. Every realm's worktree, the main tree and a consumer's clone run this
// suite on one machine at once, and they share one temp folder. A path named
// only after its test ("spade_test_maximal.world.yaml") is the same in every
// process, so one run's cleanup can delete, or its write overwrite, another's
// file mid-test. The lead's gate 5 lost two editor tests that way on
// 2026-10-06.
//
// HOW. Each name is "spade_<process id>_<n>_<stem>": the process id separates
// concurrent runs, and n, a per-process counter, separates two paths in one
// test. A folder left by a crashed run whose process id comes round again is
// cleared before reuse. No two live processes share an id, so that clear
// cannot touch a running test's files.

#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>

#ifdef _WIN32
#include <process.h>  // _getpid
#else
#include <unistd.h>  // getpid
#endif

namespace spade::test {

[[nodiscard]] inline uint64_t process_id() noexcept {
#ifdef _WIN32
    return static_cast<uint64_t>(_getpid());
#else
    return static_cast<uint64_t>(getpid());
#endif
}

// A file or folder path in the temp folder that no other call, in this
// process or another, returns. It is named, not created.
[[nodiscard]] inline std::filesystem::path unique_temp_path(std::string_view stem) {
    static std::atomic<uint64_t> counter{0};
    const uint64_t n = counter.fetch_add(1, std::memory_order_relaxed);
    return std::filesystem::temp_directory_path() /
           ("spade_" + std::to_string(process_id()) + "_" + std::to_string(n) + "_" + std::string(stem));
}

// A new, empty folder at a unique_temp_path(). The caller removes it; TempDir
// does that for you.
[[nodiscard]] inline std::filesystem::path unique_temp_dir(std::string_view tag) {
    const std::filesystem::path dir = unique_temp_path(tag);
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);  // only a crashed run's leftover can be here
    std::filesystem::create_directories(dir);
    return dir;
}

// A unique_temp_dir() removed, with everything in it, when it goes out of scope.
class TempDir {
  public:
    explicit TempDir(std::string_view tag) : path_(unique_temp_dir(tag)) {}
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

  private:
    std::filesystem::path path_;
};

}  // namespace spade::test
