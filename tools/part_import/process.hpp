#pragma once

// ---------------------------------------------------------------------------
// part_import/process.hpp -- run one child process, capture its output, and kill it on cancel (L88, step 4). Only the STEP route needs a
// child (python running step_convert.py); this is the whole of the platform code: CreateProcessW and pipes on Windows, fork/exec on POSIX.
//
// No shell is involved: the arguments are passed as given. The child's standard input is closed. Standard output and error are read to the
// end on their own threads (a child that fills a pipe never blocks), and the caller sees each stdout line as it arrives (progress).
// ---------------------------------------------------------------------------

#include <chrono>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace spade::partimport {

struct ProcessOptions {
    std::vector<std::string> argv;  // argv[0] is the program: a path, or a name looked up on PATH
    std::function<bool()> cancelled;                            // polled while the child runs; true kills it
    std::function<void(std::string_view)> on_stdout_line;       // each complete stdout line, on the calling thread
    std::chrono::milliseconds poll{25};
};

struct ProcessResult {
    bool started = false;     // false: the program could not be started (start_error says why); nothing else is set
    bool killed = false;      // the cancel flag ended it
    int exit = -1;            // the child's exit code (meaningless when killed)
    std::string out, err;     // everything the child wrote
    std::string start_error;
};

[[nodiscard]] ProcessResult run_process(const ProcessOptions& options);

}  // namespace spade::partimport
