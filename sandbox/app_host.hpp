// THE TWO OS CALLS A TESTER'S LAUNCH NEEDS (L73a): a message box, and the default program for a file (the browser for
// the User Guide). The decisions around them are display-free in app_failure.hpp and the workspace; these are the
// calls. Off Windows the box is stderr and the open says no.

#pragma once

#include <filesystem>
#include <string>

namespace spade::sandbox::app {

// A box with one OK button: `body` is the message, UTF-8. Returns when it is dismissed.
void show_error(const std::string& title, const std::string& body);

// Opens `file` in the program the OS has for it; true when the OS took it.
[[nodiscard]] bool open_in_default_app(const std::filesystem::path& file);

// Milliseconds since this process was created (on Windows from its creation time, so the loader and the start-up
// before main() count), for the editor's "first frame" PERF line (L74).
[[nodiscard]] double process_age_ms();

}  // namespace spade::sandbox::app
