#pragma once

// The recorder simulates once and renders as often as a shot needs: each
// filmed frame's published volume goes to disk, with its step, its simulated
// time and the case's live measures, so a view can be re-aimed or re-coloured
// without running the solver again. Raw, for this machine only; it is
// scratch, deleted once the video is made.

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "core/flow_volume.hpp"

namespace flow_movie {

struct FrameLog {
    uint64_t step = 0;
    double time = 0.0;           // simulated
    std::vector<double> live;    // the case's own measures at this frame, in its order
};

[[nodiscard]] spade::Result<void> write_frame(const std::filesystem::path& dir, uint32_t frame,
                                              const spade::FlowVolume& volume, const FrameLog& log);
[[nodiscard]] spade::Result<void> read_frame(const std::filesystem::path& dir, uint32_t frame,
                                             spade::FlowVolume& volume, FrameLog& log);
// How many frames the cache holds, counted from 0 without a gap.
[[nodiscard]] uint32_t cached_frames(const std::filesystem::path& dir);

}  // namespace flow_movie
