#pragma once

// ---------------------------------------------------------------------------
// part_import/report.hpp -- what the tool says: its stable codes and their exit codes, and the JSON lines it prints (L88;
// docs/design/interface/plans/2026-11-17-part-import-design.md, sections 1 and 4).
//
// A code is a stable string, never renamed; the exit code is one of seven; the advice is text a person can act on, kept here by code
// (never found by matching a message).
// ---------------------------------------------------------------------------

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>
#include <vector>

namespace spade::partimport {

inline constexpr int kExitOk = 0;
inline constexpr int kExitIo = 1;           // an io or internal failure
inline constexpr int kExitUsage = 2;        // nothing was read
inline constexpr int kExitUnavailable = 3;  // a format feature this build does not read, or the STEP converter
inline constexpr int kExitRefused = 4;      // the file is not a good solid
inline constexpr int kExitCancelled = 5;
inline constexpr int kExitFlagged = 6;      // written, but the STEP cross-check flagged a solid

struct CodeInfo {
    std::string_view code;
    int exit = 0;
    std::string_view advice;  // for a person; empty for "ok"
};

// Every code, in the design note's order.
[[nodiscard]] const std::vector<CodeInfo>& all_codes();
// One code's entry; a string that is no code gives the `internal` entry.
[[nodiscard]] const CodeInfo& code_info(std::string_view code);

// Why a step failed: a code from the table and a sentence naming what was wrong.
struct Problem {
    std::string code;
    std::string message;
};
template <class T>
using Expected = std::expected<T, Problem>;

// JSON, one object a line. Keys keep the order they were added in; strings are UTF-8 with a quote, a backslash and the control
// characters escaped.
[[nodiscard]] std::string json_quote(std::string_view s);
// The shortest text that reads back as `v` (std::format "{}"), `null` for a value that is not finite.
[[nodiscard]] std::string json_number(double v);

class JsonObject {
public:
    JsonObject& str(std::string_view key, std::string_view value);
    JsonObject& num(std::string_view key, double value);
    JsonObject& integer(std::string_view key, std::int64_t value);
    JsonObject& boolean(std::string_view key, bool value);
    JsonObject& null(std::string_view key);
    JsonObject& raw(std::string_view key, std::string_view json);  // a value already in JSON form: an array, an object
    JsonObject& merge(const JsonObject& other);                   // the other object's keys after this one's
    [[nodiscard]] std::string line() const;                        // {"a":1,"b":"x"}, no newline

private:
    std::string body_;
};

}  // namespace spade::partimport
