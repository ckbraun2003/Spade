// WHAT A TESTER IS TOLD WHEN THE WINDOW WILL NOT OPEN (L73a item 4; the lead, 2026-11-11). A tester sees no stderr, so a
// window that cannot open is a message box in plain words, then a clean exit. Display-free: the wording is decided
// here and tested in spade_tests; app_host.hpp shows it. Never shown by --live-smoke, --headless or --edit (a modal box
// would hang CI): only the interactive windows ask for it.

#pragma once

#include <string>
#include <string_view>
#include <utility>

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
    std::string body;
    switch (kind) {
        case WindowFailure::no_display:
            body =
                "Spade Editor could not open its window: Windows did not give it a display to draw on.\n\n"
                "Close other programs and start Spade Editor again. If it keeps happening, restart the computer.";
            break;
        case WindowFailure::no_opengl:
            body =
                "Spade Editor needs a graphics card with OpenGL 4.3, and this computer's graphics driver does not "
                "offer it.\n\n"
                "Update your graphics driver (from NVIDIA, AMD or Intel, whichever made your graphics card) and "
                "start Spade Editor again. A remote desktop or a virtual machine often has no OpenGL 4.3: use the "
                "computer itself.";
            break;
        case WindowFailure::other:
            body =
                "Spade Editor could not open its window.\n\n"
                "Close it and start it again. If it keeps happening, update your graphics driver.";
            break;
    }
    body += "\n\nIf you ask for help, send the newest file in %LOCALAPPDATA%\\Spade\\logs (type that into the "
            "address bar of a folder window).";
    if (!detail.empty()) {
        body += "\n\nDetail: ";
        body += detail;
    }
    return FailureText{"Spade Editor", std::move(body)};
}

}  // namespace spade::sandbox::app
