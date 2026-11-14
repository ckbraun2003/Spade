// SpadeEditor.exe -- the launcher (L73a; the lead, 2026-11-11). A WIN32-subsystem program, so a Start-menu launch has no
// console window. It starts spade_sandbox.exe (beside it) with the launcher's own arguments, with the sandbox's stdout
// and stderr going into a log file in %LOCALAPPDATA%\Spade\logs (the last kLogsKept runs kept), waits, and returns the
// sandbox's exit code. spade_sandbox.exe itself stays a console program, so the live smoke's exit code and CI's output
// are as they were. What it decides is in launcher_logic.hpp (tested); this file is the Win32 calls around it.
//
// With no spade_sandbox.exe beside it, a message box says so in plain words.

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "../launcher_logic.hpp"

namespace {

namespace fs = std::filesystem;
namespace launcher = spade::sandbox::launcher;

fs::path own_folder() {
    std::wstring path(32768, L'\0');
    const DWORD n = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (n == 0 || n >= path.size()) return {};
    path.resize(n);
    return fs::path(path).parent_path();
}

void say(const std::wstring& text) {
    MessageBoxW(nullptr, text.c_str(), L"Spade Editor", MB_OK | MB_ICONERROR | MB_SETFOREGROUND);
}

// %LOCALAPPDATA%\Spade\logs, made if it is not there; empty when it cannot be (the editor then runs with no log).
fs::path logs_folder() {
    std::wstring base(32768, L'\0');
    const DWORD n = GetEnvironmentVariableW(L"LOCALAPPDATA", base.data(), static_cast<DWORD>(base.size()));
    if (n == 0 || n >= base.size()) return {};
    base.resize(n);
    const fs::path dir = fs::path(base) / "Spade" / "logs";
    std::error_code ec;
    fs::create_directories(dir, ec);
    return ec ? fs::path{} : dir;
}

// Keeps the newest logs (the new one included), then opens the new file for the child to inherit: shared for reading,
// so a tester can open it while the editor runs.
HANDLE open_log(const fs::path& dir) {
    if (dir.empty()) return INVALID_HANDLE_VALUE;
    std::vector<std::string> names;
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (it->is_regular_file(ec)) names.push_back(it->path().filename().string());
    }
    for (const std::string& old : launcher::logs_to_delete(std::move(names), launcher::kKeepLogs - 1u)) {
        fs::remove(dir / old, ec);
    }
    SYSTEMTIME t;
    GetLocalTime(&t);
    const std::string name = launcher::log_file_name(t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
    SECURITY_ATTRIBUTES inherit{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    return CreateFileW((dir / name).c_str(), GENERIC_WRITE, FILE_SHARE_READ, &inherit, CREATE_ALWAYS,
                       FILE_ATTRIBUTE_NORMAL, nullptr);
}

}  // namespace

int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR, int) {
    const fs::path here = own_folder();
    const fs::path program = here / L"spade_sandbox.exe";
    std::error_code ec;
    if (here.empty() || !fs::is_regular_file(program, ec)) {
        say(L"Spade Editor could not start: spade_sandbox.exe is missing from its folder (" + here.wstring() +
            L").\n\nInstall Spade Editor again.");
        return 1;
    }

    const HANDLE log = open_log(logs_folder());
    std::wstring command = L"\"" + program.wstring() + L"\"";
    const std::wstring args = launcher::arguments_after_program<std::wstring>(GetCommandLineW());
    if (!args.empty()) command += L" " + args;

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    if (log != INVALID_HANDLE_VALUE) {
        startup.dwFlags = STARTF_USESTDHANDLES;
        startup.hStdOutput = log;
        startup.hStdError = log;
        startup.hStdInput = INVALID_HANDLE_VALUE;
    }
    PROCESS_INFORMATION child{};
    // The sandbox's working directory is its own folder, so ./assets works too (an install that moved its assets
    // beside it is found either way). CREATE_NO_WINDOW: a console program with no console window.
    const BOOL started = CreateProcessW(program.c_str(), command.data(), nullptr, nullptr,
                                        log != INVALID_HANDLE_VALUE ? TRUE : FALSE, CREATE_NO_WINDOW, nullptr,
                                        here.c_str(), &startup, &child);
    if (!started) {
        const DWORD why = GetLastError();
        if (log != INVALID_HANDLE_VALUE) CloseHandle(log);
        say(L"Spade Editor could not start (Windows error " + std::to_wstring(why) +
            L").\n\nInstall Spade Editor again.");
        return 1;
    }
    if (log != INVALID_HANDLE_VALUE) CloseHandle(log);  // the child holds its own copy
    WaitForSingleObject(child.hProcess, INFINITE);
    DWORD code = 0;
    if (!GetExitCodeProcess(child.hProcess, &code)) code = 1;
    CloseHandle(child.hThread);
    CloseHandle(child.hProcess);
    return static_cast<int>(code);
}
