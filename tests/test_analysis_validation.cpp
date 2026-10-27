// The validation report (the CFD spec §6; TD-9: one reader of the record
// format, docs/design/test-docs/01-verification.md "Validation records").
// A record's opening line is its identity, and the newest record of a
// configuration is the last fenced block with that line. Each measure keeps
// its quantity and notes verbatim, splits its value at the last " = ", and
// carries its reference, its tolerance with where that was fixed, and its
// verdict. A report-only measure has no tolerance and is never a pass.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "analysis/validation.hpp"
#include "unique_temp.hpp"

namespace {

namespace an = spade::analysis;
namespace fs = std::filesystem;

// "§", as the records hold it (UTF-8; MSVC builds without /utf-8).
constexpr std::string_view kSection = "\xc2\xa7";

// A record block as the records hold one, fences included.
[[nodiscard]] std::string block(std::string_view identity, std::string_view commit, std::string_view measures) {
    return "```text\n=== validation record: " + std::string(identity) + " ===\ncommit: " + std::string(commit) +
           "\ndate: 2026-10-12\npath: CPU reference, MSVC 1943, release\nconfig: 32x32, Re 100\n" +
           std::string(measures) + "```\n";
}

[[nodiscard]] std::string within(std::string_view quantity_and_value) {
    return "measure: " + std::string(quantity_and_value) +
           "\nreference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)\n"
           "tolerance: <= 0.02 (plan A Task 11, spec " +
           std::string(kSection) + "11.2, fixed before the first run)\nverdict: within\n";
}

[[nodiscard]] std::string report_only(std::string_view quantity_and_value) {
    return "measure: " + std::string(quantity_and_value) +
           "\nreference: none (report only; DNS data enter at V3)\ntolerance: none (report only)\nverdict: report only\n";
}

// The refusal's message for one file's text, or "" when it parses.
[[nodiscard]] std::string refusal(const std::string& text) {
    const spade::Result<std::vector<an::ValidationRecord>> r = an::validation_records(text, "case.md");
    return r ? std::string() : r.error().context;
}

}  // namespace

// ---- which records ----------------------------------------------------------

TEST(AnalysisValidation, TheNewestRecordOfAConfigurationIsItsLastBlock) {
    const std::string text = "# C1 -- validation records\n\nProse, skipped.\n\n" +
                             block("C1 cavity, Re 100, 32x32", "aaa1111", within("max |u| = 0.0064")) + "\n" +
                             block("C1 cavity, Re 400, 128x128", "bbb2222", within("max |u| = 0.0076")) + "\n" +
                             block("C1 cavity, Re 100, 32x32", "ccc3333", within("max |u| = 0.0051"));
    const spade::Result<std::vector<an::ValidationRecord>> r = an::validation_records(text, "c1-cavity.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_EQ(r->size(), 2u) << "one record per opening line";
    // In the order each configuration first appears; the Re 100 one is its
    // last block, though the file's last block is not always a configuration's.
    EXPECT_EQ(r->at(0).identity, "C1 cavity, Re 100, 32x32");
    EXPECT_EQ(r->at(0).commit, "ccc3333");
    EXPECT_EQ(r->at(0).measures.at(0).value, "0.0051");
    EXPECT_EQ(r->at(1).identity, "C1 cavity, Re 400, 128x128");
    EXPECT_EQ(r->at(1).commit, "bbb2222");
    EXPECT_EQ(r->at(0).source, "c1-cavity.md");
    EXPECT_EQ(r->at(0).sha, "ccc3333");
    EXPECT_EQ(r->at(0).date, "2026-10-12");
    EXPECT_EQ(r->at(0).path, "CPU reference, MSVC 1943, release");
    EXPECT_EQ(r->at(0).config, "32x32, Re 100");
}

// A commit line may carry a note; its sha is the first token.
TEST(AnalysisValidation, ACommitKeepsItsNoteAndNamesItsSha) {
    const std::string commit = "36153e0 (physics/airflow-core, rebased from dd497a4 with the same content)";
    const spade::Result<std::vector<an::ValidationRecord>> r =
        an::validation_records(block("C1 cavity, Re 100, 32x32", commit, within("max |u| = 0.0064")), "c.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->at(0).commit, commit);
    EXPECT_EQ(r->at(0).sha, "36153e0");
}

// Only ```text fences hold records: another fence's contents, and prose
// that looks like a record, are skipped.
TEST(AnalysisValidation, OnlyTextFencesHoldRecords) {
    const std::string lookalike = "=== validation record: C9 not a record ===\ncommit: zzz\n";
    const std::string text = lookalike + "\n```cpp\n" + lookalike + "verdict: pass\n```\n\n```\n" + lookalike + "```\n" +
                             block("C1 cavity, Re 100, 32x32", "aaa1111", within("max |u| = 0.0064"));
    const spade::Result<std::vector<an::ValidationRecord>> r = an::validation_records(text, "c.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_EQ(r->size(), 1u);
    EXPECT_EQ(r->at(0).identity, "C1 cavity, Re 100, 32x32");
}

// Only a bare ``` closes a fence (CommonMark): a "```text" line inside
// another fence is that fence's content, not a record.
TEST(AnalysisValidation, OnlyABareFenceClosesOne) {
    const std::string text = "```cpp\n```text\n=== validation record: C9 not a record ===\ncommit: zzz\n```\n" +
                             block("C1 cavity, Re 100, 32x32", "aaa1111", within("max |u| = 0.0064"));
    const spade::Result<std::vector<an::ValidationRecord>> r = an::validation_records(text, "c.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_EQ(r->size(), 1u);
    EXPECT_EQ(r->at(0).identity, "C1 cavity, Re 100, 32x32");
}

// ---- each measure -------------------------------------------------------------

// Split at the LAST " = ": the quantity keeps its own "=" and its notes,
// verbatim.
TEST(AnalysisValidation, AMeasureKeepsItsQuantityAndNotesVerbatim) {
    const std::string quantity =
        "max |v - v_Ghia| on y = 0.5, Table II point 117 left out (printed -0.23827, a suspected typo in the original)";
    const spade::Result<std::vector<an::ValidationRecord>> r =
        an::validation_records(block("C1 cavity, Re 400, 128x128", "81a87da", within(quantity + " = 0.0065")), "c.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    const an::ValidationMeasure& m = r->at(0).measures.at(0);
    EXPECT_EQ(m.quantity, quantity);
    EXPECT_EQ(m.value, "0.0065");
    EXPECT_DOUBLE_EQ(m.number, 0.0065);
    EXPECT_EQ(m.unit, "");
    EXPECT_EQ(m.reference, "Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)");
    EXPECT_EQ(m.verdict, an::Verdict::within);
}

TEST(AnalysisValidation, AValueCarriesItsUnit) {
    const spade::Result<std::vector<an::ValidationRecord>> r = an::validation_records(
        block("C8 actuator-disc far wake", "05099fa", within("axial velocity on the axis 2R below the disc = 7.4124 m/s")),
        "c8.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    const an::ValidationMeasure& m = r->at(0).measures.at(0);
    EXPECT_EQ(m.value, "7.4124 m/s");
    EXPECT_DOUBLE_EQ(m.number, 7.4124);
    EXPECT_EQ(m.unit, "m/s");
}

TEST(AnalysisValidation, AToleranceSaysWhereItWasFixed) {
    const spade::Result<std::vector<an::ValidationRecord>> r =
        an::validation_records(block("C1 cavity, Re 100, 32x32", "aaa1111", within("max |u| = 0.0064")), "c.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    const an::ValidationMeasure& m = r->at(0).measures.at(0);
    ASSERT_TRUE(m.tolerance.has_value());
    EXPECT_EQ(m.tolerance->bound, "<= 0.02");
    EXPECT_EQ(m.tolerance->origin, "plan A Task 11, spec " + std::string(kSection) + "11.2, fixed before the first run");
}

TEST(AnalysisValidation, AMissIsReportedAsAMiss) {
    std::string measure = within("max |u| = 0.031");
    measure.replace(measure.find("verdict: within"), 15, "verdict: MISS");
    const spade::Result<std::vector<an::ValidationRecord>> r =
        an::validation_records(block("C1 cavity, Re 100, 32x32", "aaa1111", measure), "c.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->at(0).measures.at(0).verdict, an::Verdict::miss);
    EXPECT_EQ(an::verdict_name(an::Verdict::miss), "MISS");
}

// C0: no tolerance, and never a pass.
TEST(AnalysisValidation, AReportOnlyMeasureHasNoTolerance) {
    const spade::Result<std::vector<an::ValidationRecord>> r = an::validation_records(
        block("C0 Taylor-Green vortex, 64^3", "b794356",
              report_only("kinetic energy per unit mass at t = 0.000 (the first step at or past 0.0) = 0.125000") +
                  report_only("kinetic energy per unit mass at t = 2.503 (the first step at or past 2.5) = 0.121745")),
        "c0.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_EQ(r->at(0).measures.size(), 2u);
    for (const an::ValidationMeasure& m : r->at(0).measures) {
        EXPECT_EQ(m.verdict, an::Verdict::report_only);
        EXPECT_FALSE(m.tolerance.has_value());
    }
    EXPECT_EQ(r->at(0).measures.at(1).quantity, "kinetic energy per unit mass at t = 2.503 (the first step at or past 2.5)");
    EXPECT_EQ(an::verdict_name(an::Verdict::report_only), "report only");
}

TEST(AnalysisValidation, AReportOnlyMeasureIsNeverAPass) {
    std::string passed = report_only("kinetic energy = 0.125");
    passed.replace(passed.find("verdict: report only"), 20, "verdict: within");
    EXPECT_NE(refusal(block("C0", "b794356", passed)).find("report only"), std::string::npos)
        << refusal(block("C0", "b794356", passed));
    std::string bounded = within("max |u| = 0.0064");
    bounded.replace(bounded.find("verdict: within"), 15, "verdict: report only");
    EXPECT_NE(refusal(block("C1", "aaa1111", bounded)).find("tolerance"), std::string::npos)
        << refusal(block("C1", "aaa1111", bounded));
}

// ---- the shape, refused by file and line ----------------------------------------

TEST(AnalysisValidation, AMalformedRecordIsRefusedByLine) {
    const std::string good = block("C1 cavity, Re 100, 32x32", "aaa1111", within("max |u| = 0.0064"));
    // Line 3 of the file is the block's "commit:" line.
    std::string unknown = good;
    unknown.replace(unknown.find("commit:"), 7, "commits:");
    EXPECT_NE(refusal(unknown).find("case.md:3"), std::string::npos) << refusal(unknown);
    // A block that does not open with its opening line.
    EXPECT_NE(refusal("```text\ncommit: aaa\n```\n").find("case.md:2"), std::string::npos)
        << refusal("```text\ncommit: aaa\n```\n");
    // config missing.
    std::string no_config = good;
    no_config.erase(no_config.find("config:"), no_config.find('\n', no_config.find("config:")) + 1 - no_config.find("config:"));
    EXPECT_NE(refusal(no_config).find("config"), std::string::npos) << refusal(no_config);
    // A measure cut short of its verdict.
    std::string no_verdict = good;
    no_verdict.erase(no_verdict.find("verdict:"), no_verdict.find('\n', no_verdict.find("verdict:")) + 1 - no_verdict.find("verdict:"));
    EXPECT_NE(refusal(no_verdict).find("verdict"), std::string::npos) << refusal(no_verdict);
    // A record without a measure, a value that is not a number, a verdict
    // that is not one of the three, a tolerance without its origin, a fence
    // left open.
    EXPECT_NE(refusal(block("C1", "aaa1111", "")).find("measure"), std::string::npos);
    EXPECT_NE(refusal(block("C1", "aaa1111", within("max |u| = about 0.01"))).find("number"), std::string::npos);
    std::string pass = within("max |u| = 0.0064");
    pass.replace(pass.find("verdict: within"), 15, "verdict: pass");
    EXPECT_NE(refusal(block("C1", "aaa1111", pass)).find("pass"), std::string::npos);
    std::string bare = within("max |u| = 0.0064");
    bare.replace(bare.find(" (plan A"), bare.find('\n', bare.find(" (plan A")) - bare.find(" (plan A"), "");
    EXPECT_NE(refusal(block("C1", "aaa1111", bare)).find("where"), std::string::npos) << refusal(block("C1", "aaa1111", bare));
    EXPECT_NE(refusal("```text\n=== validation record: C1 ===\ncommit: aaa\n").find("open"), std::string::npos);
    // A measure without " = ".
    EXPECT_NE(refusal(block("C1", "aaa1111", within("max |u| is small"))).find(" = "), std::string::npos)
        << refusal(block("C1", "aaa1111", within("max |u| is small")));
    // Not yet pasted: the test's own print, without commit and date.
    std::string unpasted = good;
    unpasted.erase(unpasted.find("commit:"), unpasted.find("path:") - unpasted.find("commit:"));
    EXPECT_NE(refusal(unpasted).find("commit"), std::string::npos) << refusal(unpasted);
    EXPECT_EQ(refusal(good), "");
}

// Checked out with CRLF, or saved with a byte-order mark, a file reads the same.
TEST(AnalysisValidation, LineEndingsAndAByteOrderMarkDoNotMatter) {
    std::string crlf = "\xef\xbb\xbf";
    for (const char ch : block("C1 cavity, Re 100, 32x32", "aaa1111", within("max |u| = 0.0064"))) {
        if (ch == '\n') crlf += '\r';
        crlf += ch;
    }
    const spade::Result<std::vector<an::ValidationRecord>> r = an::validation_records(crlf, "c.md");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->at(0).measures.at(0).value, "0.0064");
    EXPECT_EQ(r->at(0).config, "32x32, Re 100");
}

// ---- a records folder -------------------------------------------------------------

TEST(AnalysisValidation, AFolderIsReadFileByFileInNameOrder) {
    const spade::test::TempDir dir("validation");
    std::ofstream(dir.path() / "c8-disc.md", std::ios::binary) << block("C8 disc", "c8c8c8c", within("w = 7.4 m/s"));
    std::ofstream(dir.path() / "c1-cavity.md", std::ios::binary) << block("C1 cavity", "c1c1c1c", within("u = 0.006"));
    std::ofstream(dir.path() / "notes.txt", std::ios::binary) << "not a records file\n";
    const spade::Result<std::vector<an::ValidationRecord>> r = an::validation_report(dir.path());
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_EQ(r->size(), 2u);
    EXPECT_EQ(r->at(0).source, "c1-cavity.md");
    EXPECT_EQ(r->at(1).source, "c8-disc.md");
    EXPECT_FALSE(an::validation_report(dir.path() / "missing").has_value());
}

// The V1 records themselves (tests/data/airflow/records/), whatever runs are
// appended later: every record reads, C1's three configurations are there,
// C0 is report-only throughout, and Re 400's note is kept.
TEST(AnalysisValidation, TheV1RecordsRead) {
    const spade::Result<std::vector<an::ValidationRecord>> r =
        an::validation_report(fs::path(SPADE_TESTS_DIR) / "data" / "airflow" / "records");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    std::vector<std::string> c1;
    bool c0 = false;
    bool c8 = false;
    for (const an::ValidationRecord& rec : *r) {
        EXPECT_FALSE(rec.commit.empty());
        EXPECT_FALSE(rec.date.empty());
        ASSERT_FALSE(rec.measures.empty()) << rec.identity;
        if (rec.source == "c1-cavity.md") c1.push_back(rec.identity);
        if (rec.source == "c0-taylor-green.md") {
            c0 = true;
            // Its file holds the same opening line twice: the first block
            // (81a87da) is never the newest, whatever is appended later.
            EXPECT_NE(rec.sha, "81a87da");
            for (const an::ValidationMeasure& m : rec.measures) {
                EXPECT_EQ(m.verdict, an::Verdict::report_only) << m.quantity;
                EXPECT_FALSE(m.tolerance.has_value()) << m.quantity;
            }
        }
        // C8's own record, the CPU's: its first measure a speed. The file's
        // other record, the GPU gather in the CPU step, opens on a count since
        // K2b (the increments differing, 0 of all).
        if (rec.source == "c8-actuator-disc.md" && rec.identity == "C8 actuator-disc far wake") {
            c8 = true;
            EXPECT_EQ(rec.measures.at(0).unit, "m/s");
        }
        for (const an::ValidationMeasure& m : rec.measures) {
            if (m.verdict != an::Verdict::report_only) {
                ASSERT_TRUE(m.tolerance.has_value()) << m.quantity;
                EXPECT_FALSE(m.tolerance->origin.empty()) << m.quantity;
            }
        }
        if (rec.identity == "C1 lid-driven cavity, Re 400, 128x128") {
            EXPECT_NE(rec.measures.at(1).quantity.find("point 117 left out"), std::string::npos)
                << rec.measures.at(1).quantity;
        }
    }
    EXPECT_TRUE(c0);
    EXPECT_TRUE(c8);
    EXPECT_EQ(c1, (std::vector<std::string>{"C1 lid-driven cavity, Re 100, 32x32", "C1 lid-driven cavity, Re 100, 128x128",
                                            "C1 lid-driven cavity, Re 400, 128x128"}));
}
