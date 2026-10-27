#pragma once
// What every airflow record's test shares: the build a validation record names
// on its path line (I2's grammar), the block's text, and the block read back
// as the record reader reads a pasted one, so a format slip fails in the test
// that printed it and not at the paste (Test/Docs and the lead, 2026-10-26).
#include <cstddef>
#include <string>
#include <vector>

#include "analysis/validation.hpp"

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

// The text `print` writes, whatever its length. `print(buffer, size)` is one
// std::snprintf call whose format is a literal, so the compiler still checks
// the format against its arguments. It is called twice: to measure, then to
// write.
template <class Print>
[[nodiscard]] std::string formatted(Print&& print) {
    const int n = print(nullptr, std::size_t{0});
    if (n <= 0) return {};
    std::string text(static_cast<std::size_t>(n), '\0');
    print(text.data(), text.size() + 1u);
    return text;
}

// `block` as printed, its opening line first, read as the record reader reads
// it once pasted: fenced, with the paste's commit and date lines after the
// opening line.
[[nodiscard]] inline spade::Result<std::vector<spade::analysis::ValidationRecord>> read_as_record(
    const std::string& block, const std::string& source) {
    const std::size_t first = block.find('\n') + 1;
    const std::string text = "```text\n" + block.substr(0, first) + "commit: 0000000 (a test)\ndate: 2026-10-26\n" +
                             block.substr(first) + "```\n";
    return spade::analysis::validation_records(text, source);
}

}  // namespace airflow_test
