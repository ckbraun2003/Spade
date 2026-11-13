// L74 (the lead, 2026-11-13): the editor's two timing notes, measurement only. RED STUBS: the bodies are filled in
// by the GREEN commit; every test in tests/test_sandbox_perf.cpp fails on them.

#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace spade::sandbox::perf {

[[nodiscard]] inline std::string line(std::string_view, double, std::string_view) { return {}; }

[[nodiscard]] inline double median(std::vector<double>) { return -1.0; }

[[nodiscard]] inline bool enabled_from(const char*) noexcept { return true; }

[[nodiscard]] inline bool enabled() noexcept { return true; }

class AirPerf {
  public:
    void step(double) {}
    void frame(double, double, double) {}
    [[nodiscard]] std::vector<std::string> lines() const { return {}; }
};

}  // namespace spade::sandbox::perf
