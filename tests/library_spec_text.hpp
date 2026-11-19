#pragma once

// A part's v1 flight spec as the `spec:` block of a blocks part (scene/part_spec.hpp), for tests: the body of a v1 part file text with its part_version
// line (and comments, blank lines) dropped and every other line indented two spaces under `spec:`.
#include <string>

namespace spade::test {

[[nodiscard]] inline std::string as_spec(const std::string& v1_text) {
    std::string out = "spec:\n";
    std::size_t at = 0;
    while (at < v1_text.size()) {
        std::size_t end = v1_text.find('\n', at);
        if (end == std::string::npos) end = v1_text.size();
        std::string line = v1_text.substr(at, end - at);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        at = end + 1;
        if (line.rfind("part_version:", 0) == 0 || line.empty() || line[0] == '#') continue;
        out += "  " + line + "\n";
    }
    return out;
}

}  // namespace spade::test
