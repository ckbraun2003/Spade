// The analysis's processes, per platform (INT-7): the real launcher and the
// liveness check behind analysis/launch.hpp. Internal: not installed.
//
// Windows: CreateProcessW with the executable as lpApplicationName (no
// search), a command line the child splits back into exactly the arguments
// given, DETACHED_PROCESS and a new process group, and out of the host's job
// where the job allows it. POSIX: posix_spawn (not posix_spawnp) in a new
// session, with stdin, stdout and stderr on /dev/null.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "analysis/launch.hpp"
#include "core/error.hpp"

namespace spade::analysis::detail {

// The one string CreateProcessW takes for `argv` (argv[0] the program), quoted
// by CommandLineToArgvW's rules so the child reads back every argument
// exactly: an argument is quoted only when it is empty or holds a space, a tab
// or a quote; inside quotes, a quote is escaped and the backslashes before it,
// or before the closing quote, are doubled. UTF-8 in, UTF-8 out.
[[nodiscard]] std::string windows_command_line(const std::vector<std::string>& argv);

// Starts `spec` detached and returns its PID. A missing executable is refused
// by name (not_found) before anything is started.
[[nodiscard]] Result<uint64_t> launch_detached(const ProcessSpec& spec);

[[nodiscard]] bool process_running(uint64_t pid);

// The memory a new run could have now: on Windows the smaller of free
// physical memory and free commit (the box rule watches both); on Linux
// MemAvailable; elsewhere the free pages.
[[nodiscard]] uint64_t free_memory_bytes();

}  // namespace spade::analysis::detail
