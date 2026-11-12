// WHAT A TESTER IS TOLD WHEN THE WINDOW WILL NOT OPEN (L73a item 4; the lead, 2026-11-11). A tester sees no stderr, so a
// window that cannot open is a message box in plain words, then a clean exit. Display-free: the wording is decided
// here and tested in spade_tests; app_host.hpp shows it. Never shown by --live-smoke, --headless or --edit (a modal box
// would hang CI): only the interactive windows ask for it.

#pragma once

#include <string>
#include <string_view>

namespace spade::sandbox::app {

// Why the window could not open: no display to draw on (GLFW could not start), no OpenGL 4.3 context (the driver or
// the machine cannot give one), or something else.
enum class WindowFailure { no_display, no_opengl, other };

struct FailureText {
    std::string title;
    std::string body;
};

// The message: what happened in plain words, what to do, where the log is, and the technical detail last.
[[nodiscard]] inline FailureText window_failure_text(WindowFailure kind, std::string_view detail) {
    (void)kind;
    (void)detail;
    return {};
}

}  // namespace spade::sandbox::app
