// The one OS call of app_paths.hpp: the folder this executable is in.

#include "app_paths.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <string>
#endif

namespace spade::sandbox::app {

std::filesystem::path executable_dir() {
    std::error_code ec;
#ifdef _WIN32
    std::wstring path(32768, L'\0');
    const DWORD n = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (n == 0 || n >= path.size()) return std::filesystem::current_path(ec);
    path.resize(n);
    return std::filesystem::path(path).parent_path();
#else
    const std::filesystem::path self = std::filesystem::read_symlink("/proc/self/exe", ec);
    return ec ? std::filesystem::current_path(ec) : self.parent_path();
#endif
}

}  // namespace spade::sandbox::app
