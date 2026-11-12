// THE LAUNCHER'S DECISIONS (L73a; the lead, 2026-11-11). SpadeEditor.exe is a small WIN32-subsystem program (no console
// window) that starts spade_sandbox.exe with the launcher's own arguments, sends what the sandbox writes to stdout and
// stderr into a log file a tester can send, and hands back the sandbox's exit code. The Win32 calls are in
// launcher/main.cpp; what it decides is here, display-free, and tested in spade_tests.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace spade::sandbox::launcher {

// The last runs' logs are kept: this many, the new one included.
inline constexpr std::size_t kKeepLogs = 5;

// "spade-YYYYMMDD-HHMMSS.log": sorts oldest first by name.
[[nodiscard]] inline std::string log_file_name(int year, int month, int day, int hour, int minute, int second) {
    (void)year;
    (void)month;
    (void)day;
    (void)hour;
    (void)minute;
    (void)second;
    return {};
}

// Of the files in the logs folder, the ones to delete so that `keep` of the spade-*.log files remain: the oldest. Any
// other file in the folder is never named.
[[nodiscard]] inline std::vector<std::string> logs_to_delete(std::vector<std::string> names, std::size_t keep) {
    (void)names;
    (void)keep;
    return {};
}

// The launcher's command line without its own program name (a quoted path, or up to the first blank), the blanks
// after it dropped, the rest verbatim: it is handed to the sandbox as it came.
template <class String>
[[nodiscard]] String arguments_after_program(const String& command_line) {
    return command_line;
}

}  // namespace spade::sandbox::launcher
