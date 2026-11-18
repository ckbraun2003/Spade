// spade_part_import (L88): the program around run_part_import(). Everything the library leaves to its caller is chosen here: the process
// environment, where the program lives (step_convert.py and .venv-step are looked for beside it), a temporary-name token, and Ctrl+C /
// Ctrl+Break / SIGTERM as the cancel flag the library polls (it kills the converter, removes its temporary directory and exits 5).
// Nothing here is tested by a unit test: the behaviour is run_part_import()'s, tested in-process; tests/CMakeLists.txt runs the built
// program for its usage, and scripts/package-test.ps1 runs the installed one.

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "run.hpp"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <shellapi.h>
#else
#include <csignal>
#include <unistd.h>
#endif

namespace {

std::atomic<bool> g_cancel{false};

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // std::getenv: the process environment is read on one thread, between stages
#endif
[[nodiscard]] const char* environment(const char* name) { return std::getenv(name); }
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#if defined(_WIN32)

BOOL WINAPI on_console_event(DWORD type) {
    if (type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT || type == CTRL_CLOSE_EVENT) {
        g_cancel = true;
        return TRUE;
    }
    return FALSE;
}

[[nodiscard]] std::string to_utf8(const wchar_t* w) {
    const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (n <= 1) return {};
    std::string s(static_cast<std::size_t>(n - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
    return s;
}

// The arguments as UTF-8 (the narrow argv is the ANSI code page: a path with other characters would be mangled).
[[nodiscard]] std::vector<std::string> arguments(int, char**) {
    int n = 0;
    wchar_t** w = CommandLineToArgvW(GetCommandLineW(), &n);
    std::vector<std::string> args;
    for (int i = 1; w != nullptr && i < n; ++i) args.push_back(to_utf8(w[i]));
    if (w != nullptr) LocalFree(w);
    return args;
}

[[nodiscard]] std::filesystem::path program_dir() {
    wchar_t buf[MAX_PATH * 4];
    const DWORD n = GetModuleFileNameW(nullptr, buf, static_cast<DWORD>(sizeof buf / sizeof buf[0]));
    return n == 0 ? std::filesystem::path() : std::filesystem::path(std::wstring(buf, n)).parent_path();
}

[[nodiscard]] std::string process_token() { return std::to_string(static_cast<unsigned long>(GetCurrentProcessId())); }

void install_cancel_handlers() { SetConsoleCtrlHandler(on_console_event, TRUE); }

#else

void on_signal(int) { g_cancel = true; }

[[nodiscard]] std::vector<std::string> arguments(int argc, char** argv) { return std::vector<std::string>(argv + 1, argv + argc); }

[[nodiscard]] std::filesystem::path program_dir() {
    std::error_code ec;
    const std::filesystem::path p = std::filesystem::read_symlink("/proc/self/exe", ec);
    return ec ? std::filesystem::path() : p.parent_path();
}

[[nodiscard]] std::string process_token() { return std::to_string(static_cast<long>(getpid())); }

void install_cancel_handlers() {
    struct sigaction sa {};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

#endif

}  // namespace

int main(int argc, char** argv) {
    using namespace spade::partimport;
    install_cancel_handlers();
    RunEnv env;
    env.getenv = environment;
    env.exe_dir = program_dir();
    env.cancelled = [] { return g_cancel.load(); };
    env.token = process_token();
    std::cout << std::unitbuf;  // a line at a time, so a caller reading the progress through a pipe sees it as it happens
    return run_part_import(arguments(argc, argv), env, std::cout, std::cerr);
}
