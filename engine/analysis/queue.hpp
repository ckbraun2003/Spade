// The analysis's run queue (W2c; INT-7; the CFD spec §5 "Queued"): queued
// runs start one at a time, each only when its memory need plus the box's
// margin is free; until then it waits and its progress says why.
//
// The queue keeps no state in the host. Its order is runs_dir/queue.txt (the
// run folders, relative to runs_dir, in the order they were queued), and
// each run's state is its own progress.yaml and run.yaml. A host calls
// step_queue() on a timer, and one that closes and reopens finds the queue
// as it left it; a run started before it closed goes on in its own process.
//
// Display-free. Owned by Interface (INT-6, INT-7).

#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "analysis/launch.hpp"
#include "core/error.hpp"

namespace spade::analysis {

// The free memory a run leaves the box beside its own need: the box rule's
// 6 GB.
inline constexpr uint64_t kQueueMarginBytes = 6ull << 30;

struct QueueSetup {
    std::filesystem::path runs_dir;        // holds queue.txt and, below it, the run folders
    std::filesystem::path runner;          // the runner tool, by its absolute path (launch_run())
    LaunchOptions options;                 // the kind (dry runs until W1) and a dry run's pace
    uint64_t margin_bytes = kQueueMarginBytes;
};

// What the queue asks of the machine; each defaults to the real one, and a
// test passes fakes.
struct QueueProbes {
    Launcher launch;                         // starts a runner; default: the detached launch
    std::function<bool(uint64_t)> running;   // whether a PID still runs; default: process_running()
    std::function<uint64_t()> free_bytes;    // the memory free now; default: the smaller of free
                                             // physical memory and free commit (Windows), or
                                             // MemAvailable (Linux)
};

// Adds the queued run in `folder` (made by create_run_folder() under
// runs_dir) to the end of the queue. Refused when it isn't under runs_dir,
// isn't queued, or is queued already.
[[nodiscard]] Result<void> enqueue_run(const QueueSetup& setup, const std::filesystem::path& folder);

// What one step of the queue found and did. The paths are run folders.
struct QueueStep {
    std::filesystem::path started;               // launched by this step; empty if none
    std::filesystem::path active;                // the run in progress now; empty if none
    std::vector<std::filesystem::path> waiting;  // the queued runs, in order
    std::vector<std::filesystem::path> dropped;  // queue entries whose folder is gone
    std::string why;                             // why the first waiting run waits; empty if it doesn't
};

// One step of the serial queue, for a host's timer:
//   - a launched run whose runner is gone and whose progress isn't final is
//     marked faulted ("the runner (pid N) exited without finishing"), its
//     results kept as partial;
//   - a queued run with a cancel file is marked cancelled and never started;
//   - a run whose folder is gone leaves the queue, named in `dropped`;
//   - finished runs leave the queue;
//   - with no run in progress, the first queued run is launched when its
//     memory need (its queued progress's memory_bytes) plus the margin is
//     free; otherwise it waits, and its progress message says why. A launch
//     that is refused is that run's refusal, in its progress, and the next
//     run is tried.
// Refused (io_error) only when queue.txt or a progress file can't be read
// or written.
[[nodiscard]] Result<QueueStep> step_queue(const QueueSetup& setup, const QueueProbes& probes = {});

}  // namespace spade::analysis
