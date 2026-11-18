#include "process.hpp"

#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace spade::partimport {

namespace {

// What one pipe's reader thread has collected. Lines are cut as the bytes arrive; the caller's thread takes them with take_lines().
struct Collected {
    std::mutex mutex;
    std::string text;
    std::string pending;
    std::vector<std::string> lines;
    bool cut_lines = false;

    void add(const char* data, std::size_t n) {
        const std::lock_guard<std::mutex> lock(mutex);
        text.append(data, n);
        if (!cut_lines) return;
        pending.append(data, n);
        for (std::size_t nl; (nl = pending.find('\n')) != std::string::npos;) {
            std::string line = pending.substr(0, nl);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            lines.push_back(std::move(line));
            pending.erase(0, nl + 1);
        }
    }
    std::vector<std::string> take_lines(bool final) {
        const std::lock_guard<std::mutex> lock(mutex);
        if (final && !pending.empty()) {
            lines.push_back(std::move(pending));
            pending.clear();
        }
        return std::exchange(lines, {});
    }
};

#if defined(_WIN32)

[[nodiscard]] std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}

// The inverse of CommandLineToArgvW: backslashes before a quote are doubled, a quote is escaped, the argument is quoted when it holds
// a space, a tab, a quote or nothing.
[[nodiscard]] std::wstring quote_argument(const std::wstring& a) {
    if (!a.empty() && a.find_first_of(L" \t\n\v\"") == std::wstring::npos) return a;
    std::wstring q = L"\"";
    for (std::size_t i = 0;; ++i) {
        std::size_t slashes = 0;
        while (i < a.size() && a[i] == L'\\') {
            ++slashes;
            ++i;
        }
        if (i == a.size()) {
            q.append(slashes * 2, L'\\');
            break;
        }
        if (a[i] == L'"') {
            q.append(slashes * 2 + 1, L'\\');
        } else {
            q.append(slashes, L'\\');
        }
        q.push_back(a[i]);
    }
    q.push_back(L'"');
    return q;
}

[[nodiscard]] std::string last_error_text(const char* what) {
    return std::string(what) + " failed (Windows error " + std::to_string(GetLastError()) + ")";
}

void read_pipe(HANDLE h, Collected* into) {
    char buf[8192];
    for (;;) {
        DWORD got = 0;
        if (!ReadFile(h, buf, sizeof buf, &got, nullptr) || got == 0) break;
        into->add(buf, got);
    }
}

ProcessResult run_native(const ProcessOptions& o, Collected& out, Collected& err) {
    ProcessResult r;
    SECURITY_ATTRIBUTES sa{sizeof sa, nullptr, TRUE};
    HANDLE out_r = nullptr, out_w = nullptr, err_r = nullptr, err_w = nullptr;
    if (!CreatePipe(&out_r, &out_w, &sa, 0) || !CreatePipe(&err_r, &err_w, &sa, 0)) {
        r.start_error = last_error_text("CreatePipe");
        return r;
    }
    SetHandleInformation(out_r, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(err_r, HANDLE_FLAG_INHERIT, 0);
    HANDLE nul = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);

    std::wstring command;
    for (const std::string& a : o.argv) {
        if (!command.empty()) command.push_back(L' ');
        command += quote_argument(widen(a));
    }
    STARTUPINFOW si{};
    si.cb = sizeof si;
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = nul;
    si.hStdOutput = out_w;
    si.hStdError = err_w;
    PROCESS_INFORMATION pi{};
    const BOOL created = CreateProcessW(nullptr, command.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
    const std::string failure = created ? std::string() : last_error_text("CreateProcess");
    CloseHandle(out_w);
    CloseHandle(err_w);
    if (nul != INVALID_HANDLE_VALUE) CloseHandle(nul);
    if (!created) {
        CloseHandle(out_r);
        CloseHandle(err_r);
        r.start_error = "cannot start '" + o.argv.front() + "': " + failure;
        return r;
    }
    r.started = true;
    std::thread t_out(read_pipe, out_r, &out), t_err(read_pipe, err_r, &err);

    const auto deliver = [&](bool final) {
        for (const std::string& l : out.take_lines(final)) {
            if (o.on_stdout_line) o.on_stdout_line(l);
        }
    };
    for (;;) {
        if (WaitForSingleObject(pi.hProcess, static_cast<DWORD>(o.poll.count())) == WAIT_OBJECT_0) break;
        deliver(false);
        if (o.cancelled && o.cancelled()) {
            TerminateProcess(pi.hProcess, 1);
            WaitForSingleObject(pi.hProcess, INFINITE);
            r.killed = true;
            break;
        }
    }
    DWORD code = 0;
    GetExitCodeProcess(pi.hProcess, &code);
    r.exit = static_cast<int>(code);
    t_out.join();
    t_err.join();
    CloseHandle(out_r);
    CloseHandle(err_r);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    deliver(true);
    return r;
}

#else

void read_fd(int fd, Collected* into) {
    char buf[8192];
    for (;;) {
        const ssize_t got = read(fd, buf, sizeof buf);
        if (got < 0 && errno == EINTR) continue;
        if (got <= 0) break;
        into->add(buf, static_cast<std::size_t>(got));
    }
}

ProcessResult run_native(const ProcessOptions& o, Collected& out, Collected& err) {
    ProcessResult r;
    int out_p[2], err_p[2], fail_p[2];
    if (pipe(out_p) != 0 || pipe(err_p) != 0 || pipe(fail_p) != 0) {
        r.start_error = std::string("pipe failed: ") + std::strerror(errno);
        return r;
    }
    fcntl(fail_p[1], F_SETFD, FD_CLOEXEC);
    std::vector<char*> argv;
    std::vector<std::string> copy = o.argv;
    for (std::string& a : copy) argv.push_back(a.data());
    argv.push_back(nullptr);
    const pid_t pid = fork();
    if (pid < 0) {
        r.start_error = std::string("fork failed: ") + std::strerror(errno);
        return r;
    }
    if (pid == 0) {
        const int devnull = open("/dev/null", O_RDONLY);
        if (devnull >= 0) dup2(devnull, 0);
        dup2(out_p[1], 1);
        dup2(err_p[1], 2);
        close(out_p[0]);
        close(err_p[0]);
        close(fail_p[0]);
        execvp(argv[0], argv.data());
        const int e = errno;
        const ssize_t ignored = write(fail_p[1], &e, sizeof e);
        (void)ignored;
        _exit(127);
    }
    close(out_p[1]);
    close(err_p[1]);
    close(fail_p[1]);
    int exec_errno = 0;
    ssize_t n;
    do {
        n = read(fail_p[0], &exec_errno, sizeof exec_errno);
    } while (n < 0 && errno == EINTR);
    close(fail_p[0]);
    if (n == static_cast<ssize_t>(sizeof exec_errno)) {
        int status = 0;
        waitpid(pid, &status, 0);
        close(out_p[0]);
        close(err_p[0]);
        r.start_error = "cannot start '" + o.argv.front() + "': " + std::strerror(exec_errno);
        return r;
    }
    r.started = true;
    std::thread t_out(read_fd, out_p[0], &out), t_err(read_fd, err_p[0], &err);
    const auto deliver = [&](bool final) {
        for (const std::string& l : out.take_lines(final)) {
            if (o.on_stdout_line) o.on_stdout_line(l);
        }
    };
    int status = 0;
    for (;;) {
        const pid_t w = waitpid(pid, &status, WNOHANG);
        if (w == pid) break;
        if (w < 0 && errno != EINTR) break;
        deliver(false);
        if (o.cancelled && o.cancelled()) {
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
            r.killed = true;
            break;
        }
        std::this_thread::sleep_for(o.poll);
    }
    r.exit = WIFEXITED(status) ? WEXITSTATUS(status) : (WIFSIGNALED(status) ? 128 + WTERMSIG(status) : -1);
    t_out.join();
    t_err.join();
    close(out_p[0]);
    close(err_p[0]);
    deliver(true);
    return r;
}

#endif

}  // namespace

ProcessResult run_process(const ProcessOptions& options) {
    if (options.argv.empty() || options.argv.front().empty()) {
        ProcessResult r;
        r.start_error = "no program to run";
        return r;
    }
    Collected out, err;
    out.cut_lines = true;
    ProcessResult r = run_native(options, out, err);
    r.out = std::move(out.text);
    r.err = std::move(err.text);
    return r;
}

}  // namespace spade::partimport
