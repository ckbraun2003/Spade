// WHERE THE APP LIVES AND WHAT A BARE LAUNCH DOES (L73a, the Windows tester release of "Spade Editor"; the lead,
// 2026-11-11). Display-free: every decision here is tested in spade_tests without a window; the one OS call (the
// executable's own folder) is in app_paths.cpp.
//
// A tester starts the editor from the Start menu, so the working directory is wherever Windows puts it. The install
// layout is  <exe dir>/spade_sandbox.exe, <exe dir>/assets/{scenes,worlds,library,fonts}, <exe dir>/docs/user-guide/.
// A developer's build has no assets/ beside the exe, and the live smoke is run from the repository: both fall back to
// the working directory's assets/.

#pragma once

#include <filesystem>
#include <system_error>

namespace spade::sandbox::app {

// What a bare command line does, once the flags are read. The editor is the default (L73a); the drone sim box is
// --window or --scene drone; --headless and --live-smoke are their own.
enum class Mode { editor, drone_box, headless, live_smoke };

[[nodiscard]] constexpr Mode default_mode(bool headless, bool window, bool scene_named, bool live_smoke) noexcept {
    if (live_smoke) return Mode::live_smoke;
    if (headless) return Mode::headless;
    return window || scene_named ? Mode::drone_box : Mode::editor;
}

// An assets folder the editor can use: it holds scenes/ and worlds/ (the built-in scenes and the worlds they name).
[[nodiscard]] inline bool is_assets_root(const std::filesystem::path& dir) {
    std::error_code ec;
    return std::filesystem::is_directory(dir / "scenes", ec) && std::filesystem::is_directory(dir / "worlds", ec);
}

// The assets folder: --assets as given when there is one (so a wrong one is named in the refusal, not replaced), else
// <exe dir>/assets when it is an assets root, else <cwd>/assets (returned even when it is not there, so the refusal
// that follows names the place that was looked in).
[[nodiscard]] inline std::filesystem::path resolve_assets(const std::filesystem::path& override_dir,
                                                          const std::filesystem::path& exe_dir,
                                                          const std::filesystem::path& cwd) {
    if (!override_dir.empty()) return override_dir;
    if (const std::filesystem::path beside = exe_dir / "assets"; is_assets_root(beside)) return beside;
    return cwd / "assets";
}

// The scene a bare launch opens.
[[nodiscard]] inline std::filesystem::path default_scene(const std::filesystem::path& assets) {
    return assets / "scenes" / "hover.scene.yaml";
}

// The UI fonts: beside the exe where a build puts them (<exe dir>/fonts), else the assets folder's, where the
// installer puts them.
[[nodiscard]] inline std::filesystem::path fonts_dir(const std::filesystem::path& exe_dir,
                                                     const std::filesystem::path& assets) {
    std::error_code ec;
    if (std::filesystem::exists(exe_dir / "fonts" / "Inter-Regular.ttf", ec)) return exe_dir / "fonts";
    return assets / "fonts";
}

// The folder holding this executable (not the working directory); the working directory's when the OS won't say.
[[nodiscard]] std::filesystem::path executable_dir();

}  // namespace spade::sandbox::app
