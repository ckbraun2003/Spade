// A fixture for scripts/gcc-check-selftest.sh: copied to tools/fc_link/gcc_check_probe.cpp with fc_link_probe.hpp beside it. It includes the header
// as "fc_link/gcc_check_probe.hpp", which only resolves with tools/ on the include path. Nothing builds it.
#include "fc_link/gcc_check_probe.hpp"

std::uint32_t fc_link_probe_use(std::uint32_t n) { return fc_link_probe::probe_twice(n) + 1u; }
