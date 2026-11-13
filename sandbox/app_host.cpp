// See app_host.hpp.

#include "app_host.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#else
#include <cstdio>
#endif

#include <chrono>
#include <cstdint>

namespace spade::sandbox::app {

namespace {

// Since this file's statics were set up: near the process's start, and all there is off Windows.
const std::chrono::steady_clock::time_point kStatic = std::chrono::steady_clock::now();

double fallback_age_ms() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - kStatic).count();
}

}  // namespace

#ifdef _WIN32
namespace {

std::wstring wide(const std::string& utf8) {
    if (utf8.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), nullptr, 0);
    std::wstring out(static_cast<std::size_t>(n > 0 ? n : 0), L'\0');
    if (n > 0) MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), out.data(), n);
    return out;
}

}  // namespace

void show_error(const std::string& title, const std::string& body) {
    MessageBoxW(nullptr, wide(body).c_str(), wide(title).c_str(), MB_OK | MB_ICONERROR | MB_SETFOREGROUND);
}

double process_age_ms() {
    FILETIME created{}, exited{}, kernel{}, user{}, now{};
    if (!GetProcessTimes(GetCurrentProcess(), &created, &exited, &kernel, &user)) return fallback_age_ms();
    GetSystemTimePreciseAsFileTime(&now);
    const auto ticks = [](const FILETIME& f) { return (static_cast<uint64_t>(f.dwHighDateTime) << 32) | f.dwLowDateTime; };
    return static_cast<double>(ticks(now) - ticks(created)) / 10000.0;  // 100 ns ticks
}

bool open_in_default_app(const std::filesystem::path& file) {
    const HINSTANCE r = ShellExecuteW(nullptr, L"open", file.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
    return reinterpret_cast<INT_PTR>(r) > 32;  // ShellExecute's own convention: above 32 is success
}
#else
void show_error(const std::string& title, const std::string& body) {
    std::fprintf(stderr, "%s: %s\n", title.c_str(), body.c_str());
}

bool open_in_default_app(const std::filesystem::path& file) {
    (void)file;
    return false;
}

double process_age_ms() { return fallback_age_ms(); }
#endif

}  // namespace spade::sandbox::app
