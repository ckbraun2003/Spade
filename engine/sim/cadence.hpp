// cadence.hpp -- when a pass runs (module API stage 7, Task B2; docs/design/core/plans/2026-10-12-module-api-stage7-design.md
// 4.2). A pass with `every_steps` = j > 1 runs in substep 0 of every j-th step and not otherwise; j = 1 is every substep of
// every step, as always. This is the ONE rule: the CPU scheduler skips by it, and the recorder reads it at record time to
// decide which recording holds the pass. A function of the tick and the substep only, so a snapshot taken between fluid
// steps restores onto the same cadence.
#pragma once

#include <cstdint>

namespace spade::modules {

// RED STUB (stage 7 Task B2): due everywhere. The rule arrives with the GREEN commit.
[[nodiscard]] constexpr bool due(uint64_t /*tick*/, uint32_t /*substep*/, uint32_t /*every_steps*/) noexcept {
    return true;
}

}  // namespace spade::modules
