// cadence.hpp -- when a pass runs (module API stage 7, Task B2; docs/design/core/plans/2026-10-12-module-api-stage7-design.md
// 4.2). A pass with `every_steps` = j > 1 runs in substep 0 of every j-th step and not otherwise; j = 1 is every substep of
// every step, as always. This is the ONE rule: the CPU scheduler skips by it, and the recorder reads it at record time to
// decide which recording holds the pass. A function of the tick and the substep only, so a snapshot taken between fluid
// steps restores onto the same cadence.
#pragma once

#include <cstdint>

namespace spade::modules {

// Whether a pass with `every_steps` runs at (tick, substep). every_steps 0 is never due (compile_schedule refuses it).
[[nodiscard]] constexpr bool due(uint64_t tick, uint32_t substep, uint32_t every_steps) noexcept {
    if (every_steps == 0) return false;
    if (every_steps == 1) return true;
    return substep == 0 && tick % every_steps == 0;
}

}  // namespace spade::modules
