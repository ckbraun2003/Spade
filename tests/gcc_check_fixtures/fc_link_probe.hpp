// A fixture for scripts/gcc-check-selftest.sh: a header the self-test copies to tools/fc_link/gcc_check_probe.hpp, so the check meets code that lives
// under tools/fc_link and includes its own headers as "fc_link/...", the way the flight computer link's sources do. Nothing builds it.
#pragma once

#include <cstdint>

namespace fc_link_probe {
inline std::uint32_t probe_twice(std::uint32_t n) { return n * 2u; }
}  // namespace fc_link_probe
