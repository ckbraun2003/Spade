// THE LAUNCHER'S DECISIONS (L73a; the lead, 2026-11-11). SpadeEditor.exe is a small WIN32-subsystem program (no console
// window) that starts spade_sandbox.exe with the launcher's own arguments, sends what the sandbox writes to stdout and
// stderr into a log file a tester can send, and hands back the sandbox's exit code. The Win32 calls are in
// launcher/main.cpp; what it decides is here, display-free, and tested in spade_tests.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

namespace spade::sandbox::launcher {

// The last runs' logs are kept: this many, the new one included.
inline constexpr std::size_t kKeepLogs = 5;

// "spade-YYYYMMDD-HHMMSS.log": sorts oldest first by name.
[[nodiscard]] inline std::string log_file_name(int year, int month, int day, int hour, int minute, int second) {
    char buffer[40];
    std::snprintf(buffer, sizeof buffer, "spade-%04d%02d%02d-%02d%02d%02d.log", year, month, day, hour, minute,
                  second);
    return buffer;
}

// Of the files in the logs folder, the ones to delete so that `keep` of the spade-*.log files remain: the oldest. Any
// other file in the folder is never named.
[[nodiscard]] inline std::vector<std::string> logs_to_delete(std::vector<std::string> names, std::size_t keep) {
    const auto is_log = [](const std::string& name) {
        constexpr std::string_view kStart = "spade-";
        constexpr std::string_view kEnd = ".log";
        return name.size() > kStart.size() + kEnd.size() && name.compare(0, kStart.size(), kStart) == 0 &&
               name.compare(name.size() - kEnd.size(), kEnd.size(), kEnd) == 0;
    };
    std::vector<std::string> logs;
    for (std::string& name : names) {
        if (is_log(name)) logs.push_back(std::move(name));
    }
    std::sort(logs.begin(), logs.end());
    if (logs.size() <= keep) return {};
    logs.resize(logs.size() - keep);
    return logs;
}

// The launcher's command line without its own program name (a quoted path, or up to the first blank), the blanks
// after it dropped, the rest verbatim: it is handed to the sandbox as it came.
template <class String>
[[nodiscard]] String arguments_after_program(const String& command_line) {
    using Char = typename String::value_type;
    const auto blank = [](Char c) { return c == Char(' ') || c == Char('\t'); };
    std::size_t i = 0;
    if (!command_line.empty() && command_line[0] == Char('"')) {
        const std::size_t close = command_line.find(Char('"'), 1);
        i = close == String::npos ? command_line.size() : close + 1;
    } else {
        while (i < command_line.size() && !blank(command_line[i])) ++i;
    }
    while (i < command_line.size() && blank(command_line[i])) ++i;
    return command_line.substr(i);
}

}  // namespace spade::sandbox::launcher
