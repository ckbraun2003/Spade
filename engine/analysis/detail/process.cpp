// The analysis's processes, per platform (see the header).

#include "analysis/detail/process.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

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
#include <cerrno>
#include <climits>
#include <fcntl.h>
#include <fstream>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;
#endif

namespace spade::analysis::detail {

namespace {

namespace fs = std::filesystem;

// One argument as CommandLineToArgvW reads it back (the header's rules).
void append_quoted(std::string& out, std::string_view arg) {
    if (!arg.empty() && arg.find_first_of(" \t\n\v\"") == std::string_view::npos) {
        out += arg;
        return;
    }
    out += '"';
    for (std::size_t i = 0;; ++i) {
        std::size_t backslashes = 0;
        while (i < arg.size() && arg[i] == '\\') {
            ++backslashes;
            ++i;
        }
        if (i == arg.size()) {
            out.append(backslashes * 2, '\\');  // doubled before the closing quote
            break;
        }
        if (arg[i] == '"') {
            out.append(backslashes * 2 + 1, '\\');  // doubled, then one to escape the quote
        } else {
            out.append(backslashes, '\\');  // literal: no quote follows
        }
        out += arg[i];
    }
    out += '"';
}

[[nodiscard]] Error refused(Code code, const fs::path& exe, const std::string& why) {
    return Error{code, "cannot launch '" + path_text(exe) + "': " + why};
}

#ifdef _WIN32
[[nodiscard]] Result<std::wstring> widen(std::string_view text) {
    if (text.empty()) return std::wstring();
    const int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()),
                                      nullptr, 0);
    if (n <= 0) return std::unexpected(Error{Code::invalid_argument, "an argument is not valid UTF-8"});
    std::wstring out(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), out.data(), n);
    return out;
}
#endif

}  // namespace

std::string windows_command_line(const std::vector<std::string>& argv) {
    std::string out;
    for (std::size_t i = 0; i < argv.size(); ++i) {
        if (i > 0) out += ' ';
        append_quoted(out, argv[i]);
    }
    return out;
}

#ifdef _WIN32

Result<uint64_t> launch_detached(const ProcessSpec& spec) {
    std::error_code ec;
    if (!fs::is_regular_file(spec.executable, ec)) return std::unexpected(refused(Code::not_found, spec.executable, "no such file"));
    std::vector<std::string> argv{path_text(spec.executable)};
    argv.insert(argv.end(), spec.args.begin(), spec.args.end());
    Result<std::wstring> command = widen(windows_command_line(argv));
    if (!command) return std::unexpected(command.error());
    const std::wstring exe = spec.executable.wstring();
    const std::wstring cwd = spec.working_dir.wstring();
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION info{};
    // No console, its own process group, no inherited handles; out of the
    // host's job where the job allows breakaway, so closing the host's job
    // doesn't end the run. Where it doesn't, the child stays in the job.
    const DWORD flags = DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP | CREATE_UNICODE_ENVIRONMENT;
    BOOL ok = CreateProcessW(exe.c_str(), command->data(), nullptr, nullptr, FALSE, flags | CREATE_BREAKAWAY_FROM_JOB,
                             nullptr, cwd.empty() ? nullptr : cwd.c_str(), &startup, &info);
    if (!ok && GetLastError() == ERROR_ACCESS_DENIED) {
        ok = CreateProcessW(exe.c_str(), command->data(), nullptr, nullptr, FALSE, flags, nullptr,
                            cwd.empty() ? nullptr : cwd.c_str(), &startup, &info);
    }
    if (!ok) {
        const DWORD error = GetLastError();
        return std::unexpected(refused(Code::io_error, spec.executable,
                                       "CreateProcessW failed: " + std::system_category().message(static_cast<int>(error))));
    }
    CloseHandle(info.hThread);
    CloseHandle(info.hProcess);
    return static_cast<uint64_t>(info.dwProcessId);
}

bool process_running(uint64_t pid) {
    if (pid == 0 || pid > 0xffffffffu) return false;
    HANDLE h = OpenProcess(SYNCHRONIZE | PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
    if (h == nullptr) return GetLastError() == ERROR_ACCESS_DENIED;  // it exists, but isn't ours to ask
    const DWORD wait = WaitForSingleObject(h, 0);
    CloseHandle(h);
    return wait == WAIT_TIMEOUT;
}

uint64_t free_memory_bytes() {
    MEMORYSTATUSEX status{};
    status.dwLength = sizeof(status);
    if (!GlobalMemoryStatusEx(&status)) return 0;
    return std::min<uint64_t>(status.ullAvailPhys, status.ullAvailPageFile);
}

#else

Result<uint64_t> launch_detached(const ProcessSpec& spec) {
    std::error_code ec;
    if (!fs::is_regular_file(spec.executable, ec)) return std::unexpected(refused(Code::not_found, spec.executable, "no such file"));
    std::vector<std::string> storage{spec.executable.string()};
    storage.insert(storage.end(), spec.args.begin(), spec.args.end());
    std::vector<char*> argv;
    for (std::string& a : storage) argv.push_back(a.data());
    argv.push_back(nullptr);

    // Its own session, so the host's terminal closing doesn't reach it, and
    // no stdio of the host's.
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&actions, 1, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_addopen(&actions, 2, "/dev/null", O_WRONLY, 0);
    if (!spec.working_dir.empty()) posix_spawn_file_actions_addchdir_np(&actions, spec.working_dir.c_str());
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
#ifdef POSIX_SPAWN_SETSID
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETSID);
#else
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attributes, 0);
#endif
    pid_t pid = 0;
    const int rc = posix_spawn(&pid, spec.executable.c_str(), &actions, &attributes, argv.data(), environ);
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    if (rc != 0) {
        return std::unexpected(refused(Code::io_error, spec.executable, "posix_spawn failed: " + std::generic_category().message(rc)));
    }
    return static_cast<uint64_t>(pid);
}

bool process_running(uint64_t pid) {
    if (pid == 0 || pid > static_cast<uint64_t>(INT_MAX)) return false;
    const pid_t p = static_cast<pid_t>(pid);
    int status = 0;
    const pid_t waited = waitpid(p, &status, WNOHANG);
    if (waited == p) return false;  // our child, and it has exited: reaped now
    if (waited == 0) return true;   // our child, still running
    // Not our child: ask the system, and count a zombie (exited, not yet
    // reaped by its parent) as gone.
    if (kill(p, 0) != 0 && errno != EPERM) return false;
#ifdef __linux__
    std::ifstream stat("/proc/" + std::to_string(pid) + "/stat");
    std::string line;
    if (std::getline(stat, line)) {
        const std::size_t close = line.rfind(')');
        if (close != std::string::npos && close + 2 < line.size()) {
            const char state = line[close + 2];
            if (state == 'Z' || state == 'X') return false;
        }
    }
#endif
    return true;
}

uint64_t free_memory_bytes() {
#ifdef __linux__
    std::ifstream meminfo("/proc/meminfo");
    std::string key;
    uint64_t kib = 0;
    std::string unit;
    while (meminfo >> key >> kib >> unit) {
        if (key == "MemAvailable:") return kib * 1024;
    }
#endif
    const long pages = sysconf(_SC_AVPHYS_PAGES);
    const long size = sysconf(_SC_PAGESIZE);
    return pages > 0 && size > 0 ? static_cast<uint64_t>(pages) * static_cast<uint64_t>(size) : 0;
}

#endif

}  // namespace spade::analysis::detail
