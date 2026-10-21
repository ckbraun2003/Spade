#pragma once
// The build a validation record names on its path line (I2's grammar): the
// compiler and the preset, shared by every airflow record's test.
#include <string>

namespace airflow_test {

[[nodiscard]] inline std::string build_line() {
#if defined(NDEBUG)
    const char* preset = "release";
#else
    const char* preset = "debug";
#endif
#if defined(_MSC_VER)
    return "MSVC " + std::to_string(_MSC_VER) + ", " + preset;
#else
    return std::string(__VERSION__) + ", " + preset;
#endif
}

}  // namespace airflow_test
