#pragma once

// The recorder simulates once and renders as often as a shot needs: each
// filmed frame's published volume goes to disk, with its step, its simulated
// time and the case's live measures, so a view can be re-aimed or re-coloured
// without running the solver again. The solver's own state is kept beside
// them every few frames, so a simulation stopped part-way (a time budget, or
// the box running short of memory) continues from there instead of from
// rest. Raw, for this machine only; it is scratch, deleted once the video is
// made.

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace flow_movie {

struct FrameLog {
    uint64_t step = 0;
    double time = 0.0;         // simulated
    std::vector<double> live;  // the case's own measures at this frame, in its order
};

[[nodiscard]] spade::Result<void> write_frame(const std::filesystem::path& dir, uint32_t frame,
                                              const spade::FlowVolume& volume, const FrameLog& log);
[[nodiscard]] spade::Result<void> read_frame(const std::filesystem::path& dir, uint32_t frame,
                                             spade::FlowVolume& volume, FrameLog& log);
// How many frames the cache holds, counted from 0 without a gap.
[[nodiscard]] uint32_t cached_frames(const std::filesystem::path& dir);

// Which film a cache belongs to: its case (the case, frames, steps and grid)
// and the build that simulated it. A cache is only ever continued by its own
// case AND build; a finished one may be drawn by another build, which its
// card then names beside the simulating one.
struct CacheMark {
    std::string film;
    std::string build;
};
void mark_cache(const std::filesystem::path& dir, const CacheMark& mark);
// The mark, or empty strings for a cache without one.
[[nodiscard]] CacheMark read_mark(const std::filesystem::path& dir);

// The solver's state after `frame` was written: the grid's u, v, w, p and the
// step. One per cache, replaced atomically. read_state() fills a grid made
// to the same shape and returns the frame.
[[nodiscard]] spade::Result<void> write_state(const std::filesystem::path& dir, uint32_t frame, uint64_t step,
                                              const spade::physics::airflow::MacGrid& g);
[[nodiscard]] spade::Result<uint32_t> read_state(const std::filesystem::path& dir, uint64_t& step,
                                                 spade::physics::airflow::MacGrid& g);

}  // namespace flow_movie
