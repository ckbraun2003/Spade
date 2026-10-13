// The validation report (see the header).

#include "analysis/validation.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>

#include "core/path_text.hpp"

namespace spade::analysis {

namespace {

namespace fs = std::filesystem;

constexpr std::string_view kOpening = "=== validation record: ";
constexpr std::string_view kOpeningEnd = " ===";
constexpr std::string_view kFence = "```";
constexpr std::array<std::string_view, 3> kVerdictNames = {"within", "MISS", "report only"};
constexpr std::array<std::string_view, 4> kHeaderKeys = {"commit", "date", "path", "config"};
constexpr std::array<std::string_view, 4> kMeasureKeys = {"measure", "reference", "tolerance", "verdict"};

struct Line {
    std::string_view text;
    std::size_t number = 0;  // 1-based, in the file
};

[[nodiscard]] std::string_view trim_right(std::string_view s) {
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) s.remove_suffix(1);
    return s;
}

[[nodiscard]] Error refuse(std::string_view source, std::size_t line, const std::string& why) {
    return Error{Code::schema_mismatch, std::string(source) + ":" + std::to_string(line) + ": " + why};
}

// Reads a record's lines one key at a time, in the format's order.
class Reader {
  public:
    Reader(std::string_view source, const std::vector<Line>& lines) : source_(source), lines_(lines) {}

    [[nodiscard]] bool done() const { return next_ >= lines_.size(); }

    // The value of the next line, which must be `key: value`.
    [[nodiscard]] Result<std::string_view> take(std::string_view key, std::string_view why_missing = {}) {
        if (done()) {
            return std::unexpected(refuse(source_, lines_.back().number,
                                          "the record ends before its '" + std::string(key) + ":'" +
                                              std::string(why_missing)));
        }
        const Line& line = lines_[next_];
        const std::size_t colon = line.text.find(':');
        if (colon == std::string_view::npos || line.text.substr(0, colon) != key) {
            return std::unexpected(refuse(source_, line.number,
                                          "expected '" + std::string(key) + ":' here, found '" +
                                              std::string(line.text) + "'" + std::string(why_missing)));
        }
        std::string_view value = line.text.substr(colon + 1);
        if (!value.empty() && value.front() == ' ') value.remove_prefix(1);
        if (value.empty()) {
            return std::unexpected(refuse(source_, line.number, "'" + std::string(key) + ":' is empty"));
        }
        last_ = line.number;
        ++next_;
        return value;
    }

    [[nodiscard]] std::size_t last_line() const { return last_; }

  private:
    std::string_view source_;
    const std::vector<Line>& lines_;
    std::size_t next_ = 1;  // past the opening line
    std::size_t last_ = 0;
};

[[nodiscard]] Result<ValidationMeasure> read_measure(std::string_view source, Reader& in, std::size_t index) {
    ValidationMeasure m;
    const std::string which = " (measure " + std::to_string(index + 1) + ")";
    const Result<std::string_view> measure = in.take(kMeasureKeys[0]);
    if (!measure) return std::unexpected(measure.error());
    const std::size_t line = in.last_line();
    // The quantity may hold " = " itself; the value follows the last one.
    const std::size_t eq = measure->rfind(" = ");
    if (eq == std::string_view::npos) {
        return std::unexpected(refuse(source, line, "the measure has no ' = ' before its value: '" + std::string(*measure) + "'"));
    }
    m.quantity = std::string(measure->substr(0, eq));
    m.value = std::string(measure->substr(eq + 3));
    const std::string_view value = measure->substr(eq + 3);
    const std::size_t space = value.find(' ');
    const std::string_view token = value.substr(0, space);
    const std::from_chars_result r = std::from_chars(token.data(), token.data() + token.size(), m.number);
    if (token.empty() || r.ec != std::errc{} || r.ptr != token.data() + token.size()) {
        return std::unexpected(refuse(source, line, "the measure's value '" + m.value + "' does not start with a number"));
    }
    if (space != std::string_view::npos) m.unit = std::string(value.substr(space + 1));

    const Result<std::string_view> reference = in.take(kMeasureKeys[1], which);
    if (!reference) return std::unexpected(reference.error());
    m.reference = std::string(*reference);

    const Result<std::string_view> tolerance = in.take(kMeasureKeys[2], which);
    if (!tolerance) return std::unexpected(tolerance.error());
    const std::size_t tolerance_line = in.last_line();
    if (tolerance->substr(0, 4) != "none") {
        // "<bound> (<where it was fixed>)"
        const std::size_t open = tolerance->find(" (");
        if (open == std::string_view::npos || tolerance->back() != ')' || open == 0) {
            return std::unexpected(refuse(source, tolerance_line,
                                          "the tolerance '" + std::string(*tolerance) +
                                              "' does not say where it was fixed, in parentheses after its bound"));
        }
        m.tolerance = Tolerance{std::string(tolerance->substr(0, open)),
                                std::string(tolerance->substr(open + 2, tolerance->size() - open - 3))};
    }

    const Result<std::string_view> verdict = in.take(kMeasureKeys[3], which);
    if (!verdict) return std::unexpected(verdict.error());
    const auto known = std::find(kVerdictNames.begin(), kVerdictNames.end(), *verdict);
    if (known == kVerdictNames.end()) {
        return std::unexpected(refuse(source, in.last_line(),
                                      "'" + std::string(*verdict) + "' is not a verdict: within, MISS or report only"));
    }
    m.verdict = static_cast<Verdict>(known - kVerdictNames.begin());
    if (m.verdict == Verdict::report_only && m.tolerance) {
        return std::unexpected(refuse(source, in.last_line(),
                                      "a report-only measure carries no tolerance, but this one has '" +
                                          std::string(*tolerance) + "'"));
    }
    if (m.verdict != Verdict::report_only && !m.tolerance) {
        return std::unexpected(refuse(source, in.last_line(),
                                      "a measure without a tolerance is report only, never '" + std::string(*verdict) + "'"));
    }
    return m;
}

// One ```text block's lines, the fences left out.
[[nodiscard]] Result<ValidationRecord> read_record(std::string_view source, const std::vector<Line>& lines) {
    const Line& first = lines.front();
    if (first.text.substr(0, kOpening.size()) != kOpening || first.text.size() <= kOpening.size() + kOpeningEnd.size() ||
        first.text.substr(first.text.size() - kOpeningEnd.size()) != kOpeningEnd) {
        return std::unexpected(refuse(source, first.number,
                                      "a record opens with '=== validation record: <case> ===', not '" +
                                          std::string(first.text) + "'"));
    }
    ValidationRecord rec;
    rec.source = std::string(source);
    rec.identity = std::string(first.text.substr(kOpening.size(), first.text.size() - kOpening.size() - kOpeningEnd.size()));

    Reader in(source, lines);
    const std::string unpasted = " (whoever pastes the test's block adds commit and date after its opening line)";
    std::array<std::string, 4> header;
    for (std::size_t i = 0; i < kHeaderKeys.size(); ++i) {
        const Result<std::string_view> v = in.take(kHeaderKeys[i], i < 2 ? unpasted : std::string());
        if (!v) return std::unexpected(v.error());
        header[i] = std::string(*v);
    }
    rec.commit = header[0];
    rec.sha = rec.commit.substr(0, rec.commit.find(' '));
    rec.date = header[1];
    rec.path = header[2];
    rec.config = header[3];
    while (!in.done()) {
        Result<ValidationMeasure> m = read_measure(source, in, rec.measures.size());
        if (!m) return std::unexpected(m.error());
        rec.measures.push_back(std::move(*m));
    }
    if (rec.measures.empty()) {
        return std::unexpected(refuse(source, in.last_line(), "the record '" + rec.identity + "' holds no measure"));
    }
    return rec;
}

}  // namespace

std::string_view verdict_name(Verdict verdict) noexcept { return kVerdictNames[static_cast<std::size_t>(verdict)]; }

Result<std::vector<ValidationRecord>> validation_records(std::string_view text, std::string_view source) {
    if (text.substr(0, 3) == "\xef\xbb\xbf") text.remove_prefix(3);
    std::vector<ValidationRecord> out;
    std::unordered_map<std::string, std::size_t> newest;  // identity -> its place in `out`
    enum class In { prose, record, other_fence } in = In::prose;
    std::size_t fence_line = 0;
    std::vector<Line> block;
    std::size_t number = 0;
    for (std::size_t start = 0; start < text.size();) {
        const std::size_t end = std::min(text.find('\n', start), text.size());
        const std::string_view line = trim_right(text.substr(start, end - start));
        start = end + 1;
        ++number;
        const bool fence = line.substr(0, kFence.size()) == kFence;
        if (in == In::prose) {
            if (fence) {
                in = line == "```text" ? In::record : In::other_fence;
                fence_line = number;
                block.clear();
            }
        } else if (fence) {
            if (in == In::record) {
                if (block.empty()) return std::unexpected(refuse(source, fence_line, "an empty ```text block"));
                Result<ValidationRecord> rec = read_record(source, block);
                if (!rec) return std::unexpected(rec.error());
                const auto [at, added] = newest.try_emplace(rec->identity, out.size());
                if (added) {
                    out.push_back(std::move(*rec));
                } else {
                    out[at->second] = std::move(*rec);  // a later block of the same configuration is newer
                }
            }
            in = In::prose;
        } else if (in == In::record) {
            block.push_back(Line{line, number});
        }
    }
    if (in != In::prose) {
        return std::unexpected(refuse(source, fence_line, "the fence opened here is left open"));
    }
    return out;
}

Result<std::vector<ValidationRecord>> validation_report(const fs::path& records_dir) {
    std::error_code ec;
    if (!fs::is_directory(records_dir, ec)) {
        return std::unexpected(Error{Code::io_error, "no records folder '" + path_text(records_dir) + "'"});
    }
    std::vector<fs::path> files;
    for (const fs::directory_entry& e : fs::directory_iterator(records_dir, ec)) {
        if (e.is_regular_file(ec) && e.path().extension() == ".md") files.push_back(e.path());
    }
    if (ec) return std::unexpected(Error{Code::io_error, "cannot list '" + path_text(records_dir) + "': " + ec.message()});
    std::sort(files.begin(), files.end(),
              [](const fs::path& a, const fs::path& b) { return a.filename().generic_u8string() < b.filename().generic_u8string(); });
    std::vector<ValidationRecord> out;
    for (const fs::path& file : files) {
        std::ifstream in(file, std::ios::binary);
        if (!in) return std::unexpected(Error{Code::io_error, "cannot open '" + path_text(file) + "'"});
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        if (in.bad()) return std::unexpected(Error{Code::io_error, "failed reading '" + path_text(file) + "'"});
        Result<std::vector<ValidationRecord>> records = validation_records(text, path_text(file.filename()));
        if (!records) return std::unexpected(records.error());
        std::move(records->begin(), records->end(), std::back_inserter(out));
    }
    return out;
}

}  // namespace spade::analysis
