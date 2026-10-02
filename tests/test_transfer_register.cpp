// test_transfer_register.cpp -- Plan A Task 11 (24th spec SL7/SL18): the v1
// transfer register is machine-checked, not merely written down.
//
// SL7: "a silent drop is not a disposition." This guard is what stops v1 being
// quarantined (Plan C, spec section 7) while it is still the only
// implementation of something. A register nobody parses is a document; a
// register a test parses is a gate.

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Row {
    std::string system;
    std::string surface;
    std::string disposition;
    std::string evidence;
};

[[nodiscard]] std::string register_path() {
    // The engine dir is the anchor CMake already defines; the register lives
    // beside it under docs/.
    return std::string(SPADE_ENGINE_DIR) + "/../docs/v1-transfer-register.md";
}

[[nodiscard]] std::string trim(std::string_view s) {
    const auto first = s.find_first_not_of(" \t`");
    if (first == std::string_view::npos) return {};
    const auto last = s.find_last_not_of(" \t`");
    return std::string(s.substr(first, last - first + 1));
}

// Parses the REGISTER table only. The file also contains a vocabulary table
// with the same shape, so rows are taken from after the "## Register" heading
// -- a parser that swept the whole file would silently ingest the vocabulary's
// four rows and report a register that is four rows longer than it is.
[[nodiscard]] std::vector<Row> parse_transfer_register() {
    std::vector<Row> rows;
    std::ifstream in(register_path());
    if (!in) {
        ADD_FAILURE() << "cannot open the transfer register: " << register_path();
        return rows;
    }

    bool in_register = false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("## Register", 0) == 0) {
            in_register = true;
            continue;
        }
        if (in_register && line.rfind("## ", 0) == 0) break;  // next section ends the table
        if (!in_register || line.empty() || line.front() != '|') continue;

        std::vector<std::string> cells;
        std::stringstream cell_stream(line);
        std::string cell;
        while (std::getline(cell_stream, cell, '|')) cells.push_back(trim(cell));
        // A row splits into FIVE tokens on an LF file -- the trailing pipe
        // yields no final token -- and six on a CRLF one, where the carriage
        // return becomes a sixth that trims to empty. Requiring six rejected
        // every row on LF and reported an EMPTY register, which was visible
        // immediately only because EveryRowIsDispositioned asserts the parse is
        // non-empty BEFORE it checks anything. Index 0 is the empty cell before
        // the leading pipe.
        if (cells.size() < 5) continue;
        const std::string& system = cells[1];
        if (system.empty() || system == "v1 system") continue;   // header
        if (system.find("---") != std::string::npos) continue;   // separator
        rows.push_back(Row{system, cells[2], cells[3], cells[4]});
    }
    return rows;
}

[[nodiscard]] std::optional<Row> find_row(const std::vector<Row>& rows, std::string_view system) {
    const auto it = std::find_if(rows.begin(), rows.end(),
                                 [&](const Row& r) { return r.system == system; });
    if (it == rows.end()) return std::nullopt;
    return *it;
}

}  // namespace

TEST(TransferRegister, EveryRowIsDispositioned) {
    const std::vector<Row> rows = parse_transfer_register();
    ASSERT_FALSE(rows.empty()) << "no rows parsed -- the guard would pass vacuously";
    for (const Row& row : rows) {
        EXPECT_TRUE(row.disposition == "transferred" || row.disposition == "to-transfer" ||
                    row.disposition == "retired-with-reason" ||
                    row.disposition == "retired-to-sandbox")
            << "undispositioned row: " << row.system << " (disposition: '" << row.disposition << "')";
    }
}

// The row count is pinned so that a truncated file, a renamed heading or a
// parser that silently stops early cannot turn this suite green by reading
// fewer rows than the register has. Raising it is a deliberate edit made
// alongside adding a row.
//
// WHAT THIS ASSERTION CAN AND CANNOT DO -- read before trusting it.
//
// This is a CHANGE detector, not a completeness check, and the distinction is
// not academic: the expected count is maintained by hand alongside the table,
// so it can only catch a row lost AFTER the number was last set. It is
// structurally blind to a row that was never transcribed in the first place --
// and that is exactly what happened. The register carried 13 rows against the
// 24th spec's 14 (SL9f, barycentric wireframe, missing), this line said 13u,
// and the guard sided with the file over the authority for as long as both
// were wrong together.
//
// It cannot be made self-checking. The spec that enumerated the systems is now
// a frozen record (docs/design/superseded/2026-09-consolidation/
// 06-sandbox-and-v1.md), and tests do not read the design library.
// v1-transfer-register.md is therefore the AUTHORITY rather than a copy (its
// header says so), and this count is a change detector maintained beside it,
// not a derived fact (docs/design/test-docs/00-decisions.md, TD-4). Adding a row means
// editing both, deliberately, in the same change.
TEST(TransferRegister, HasEveryRowTheSpecEnumerates) {
    const std::vector<Row> rows = parse_transfer_register();
    EXPECT_EQ(rows.size(), 14u) << "the register gained or lost a row";
    for (const Row& row : rows) {
        EXPECT_FALSE(row.surface.empty()) << row.system << " has no v1 surface";
        EXPECT_FALSE(row.evidence.empty()) << row.system << " has no evidence";
    }
}

// A POSITIVE assertion about the one row that matters right now. If this ever
// reads "transferred" without Plan B having landed, the register is lying and
// Plan C would quarantine v1 over a live gap -- retiring the only
// implementation of the thing the 50,000-body fluid scene runs on.
TEST(TransferRegister, KnowsSphIsStillOpen) {
    const std::vector<Row> rows = parse_transfer_register();
    const std::optional<Row> sph = find_row(rows, "SPH fluid");
    ASSERT_TRUE(sph.has_value()) << "the SPH row is missing entirely";
    EXPECT_EQ(sph->disposition, "to-transfer");
}

// The rows Plan A itself closed. Asserted positively rather than left implicit,
// so that a later edit which quietly reopens one is a failure here rather than
// a surprise at the quarantine gate.
TEST(TransferRegister, RecordsTheRowsPlanAClosed) {
    const std::vector<Row> rows = parse_transfer_register();
    for (const std::string_view system : {"Velocity render mode", "Camera component",
                                          "Instancing helpers"}) {
        const std::optional<Row> row = find_row(rows, std::string(system));
        ASSERT_TRUE(row.has_value()) << "missing row: " << system;
        EXPECT_EQ(row->disposition, "transferred") << system;
    }
    const std::optional<Row> input = find_row(rows, "Input component");
    ASSERT_TRUE(input.has_value());
    EXPECT_EQ(input->disposition, "retired-to-sandbox");
}
