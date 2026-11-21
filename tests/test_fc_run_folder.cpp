// ---------------------------------------------------------------------------
// test_fc_run_folder.cpp -- the run-folder checker (tests/fc_run_folder_check.hpp), shown red against folders with one thing wrong.
//
// WHY. A flight's record (docs/design/core/plans/2026-11-21-fc-core.md section 4) is what a sweep and a release are judged from, so the
// check on it has to be shown to fail (docs/design/test-docs/plans/2026-11-21-fc-tests.md section 3, K1-K11), and it has to be shown
// without a PX4 run. Each case builds a good folder IN MEMORY (the format below is the checker's reading of Core's draft, a proposal
// until Core's writer lands), breaks one thing, and asks which check names it. The checker is written apart from Core's writer on
// purpose (TD-4): a checker that shares the writer's code shares its bugs.
//
// Checks that need Core's trace format, PX4's log or Core's replay (K3's trace hash, K4 past the header, K6's recount, K11) are reported
// `unchecked`, never `pass`; the good folder's test pins exactly which ones, so one cannot go quiet unnoticed.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "fc_run_folder_check.hpp"
#include "unique_temp.hpp"
#include "world/sha256.hpp"

namespace {

using spade::fc::Files;
using spade::fc::Report;
using spade::fc::Verdict;

std::string hex(char c) { return std::string(64, c); }

std::string replace_line(std::string text, const std::string& key, const std::string& new_line) {
    const std::string needle = key + ":";
    const size_t a = text.find(needle);
    EXPECT_NE(a, std::string::npos) << key;
    if (a == std::string::npos) return text;
    const size_t b = text.find('\n', a);
    text.replace(a, b - a, new_line);
    return text;
}

std::string drop_line(std::string text, const std::string& key) {
    const size_t a = text.find(key + ":");
    EXPECT_NE(a, std::string::npos) << key;
    if (a == std::string::npos) return text;
    const size_t b = text.find('\n', a);
    text.erase(a, b - a + 1);
    return text;
}

// Rebuild manifest.yaml from the other files, so a test that edits one file isolates the check it means to hit.
void rehash(Files& f) {
    std::string m = "files:\n";
    for (const auto& [name, bytes] : f) {
        if (name == "manifest.yaml") continue;
        m += "  " + name + ": " + spade::sha256_hex(bytes) + "\n";
    }
    f["manifest.yaml"] = m;
}

// A budget-mode folder that every implemented check passes.
Files good_budget() {
    Files f;
    f["profile.board.yaml"] = "board_version: 1\nid: board.fixture\nkind: companion\n";
    f["costs.yaml"] = "costs_version: 1\nstage_a: 1000\n";
    f["trace.bin"] = std::string("\x01\x02\x03\x04", 4);
    f["commands.bin"] = std::string("\x05\x06\x07\x08", 4);
    f["meters.yaml"] =
        "overruns: 0\nlate_samples: 0\nskipped_samples: 0\n"
        "stages:\n  chain:\n    latency_ns:\n      min: 1000\n      mean: 1500.5\n      p99: 2000\n      max: 2100\n";
    std::string run =
        "run_id: fc-fixture-0001\nmode: budget\nspade_build_id: msvc-release\nspade_commit: " + std::string(40, '0') +
        "\nengine_config_hash: " + hex('1') + "\nworld_file_hash: " + hex('2') + "\nmodel_hash: " + hex('3') +
        "\nseeds: [1, 2]\ndt_ns: 500000\nsubsteps: 1\nsensor_periods_ticks: [8, 40, 20, 200]\nprofile_hash: " +
        spade::sha256_hex(f["profile.board.yaml"]) + "\ncosts_hash: " + spade::sha256_hex(f["costs.yaml"]) +
        "\nlink_kind: px4\nlink_version: v1.17.0\nsoftware_identity: fixture-package\npackage_hash: " + hex('4') +
        "\ngrade_world: reference\ngrade_mode: reference\ngrade_link: best-effort\ngrade_run: best-effort\n"
        "fc_compute_not_modelled: true\ntrace_ticks: 3\ntrace_hash: " + hex('5') + "\n";
    f["run.yaml"] = run;
    rehash(f);
    return f;
}

// A measured-mode folder: no costs file, a box and a tree, a memory peak.
Files good_measured() {
    Files f = good_budget();
    f.erase("costs.yaml");
    f["run.yaml"] = replace_line(f["run.yaml"], "mode", "mode: measured");
    f["run.yaml"] = drop_line(f["run.yaml"], "costs_hash");
    f["run.yaml"] += "box: fixture-box\ntree: 0123456789ab\n";
    f["run.yaml"] = replace_line(f["run.yaml"], "grade_mode", "grade_mode: best-effort");
    f["meters.yaml"] += "memory_peak_bytes: 123456\n";
    rehash(f);
    return f;
}

Report check(const Files& f) { return spade::fc::check_run_folder(spade::fc::RunFolder{f}); }

const spade::fc::CheckResult* failing(const Report& r, const std::string& id) {
    return r.find(id, Verdict::fail);
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------------------
// The hash the checker leans on, pinned by FIPS 180-4 known answers (so a bug in it cannot make a tampered folder look whole).
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, Sha256KnownAnswers) {
    EXPECT_EQ(spade::sha256_hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    EXPECT_EQ(spade::sha256_hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(FcRunFolder, FlatYamlReadsNestedKeysAndRefusesAmbiguity) {
    std::string err;
    const auto m = spade::fc::read_flat_yaml("# c\na: 1\nb:\n  c: \"two\"\n  d:\n    e: 3.5\n", &err);
    EXPECT_TRUE(err.empty()) << err;
    ASSERT_EQ(m.count("a"), 1u);
    EXPECT_EQ(m.at("a"), "1");
    EXPECT_EQ(m.at("b.c"), "two");
    EXPECT_EQ(m.at("b.d.e"), "3.5");

    err.clear();
    (void)spade::fc::read_flat_yaml("a: 1\na: 2\n", &err);
    EXPECT_NE(err.find("duplicate"), std::string::npos) << err;
    err.clear();
    (void)spade::fc::read_flat_yaml("a:\n\tb: 1\n", &err);
    EXPECT_NE(err.find("tab"), std::string::npos) << err;
}

// ---------------------------------------------------------------------------------------------------------------------------------
// The good folders, and exactly what is still unchecked.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, TheGoodBudgetFolderPassesEveryImplementedCheckAndNamesWhatIsUnchecked) {
    const Report r = check(good_budget());
    EXPECT_TRUE(r.ok()) << r.text();
    EXPECT_FALSE(r.complete()) << "a folder is not complete while a check is unchecked";
    std::set<std::string> unchecked;
    for (const auto& c : r.results) {
        if (c.verdict == Verdict::unchecked) unchecked.insert(c.id);
        EXPECT_NE(c.verdict, Verdict::fail) << c.id << ": " << c.detail;
        EXPECT_NE(c.verdict, Verdict::absent) << c.id << ": " << c.detail;
    }
    EXPECT_EQ(unchecked, (std::set<std::string>{"K3.trace_hash", "K4.log", "K6.recount", "K11.replay"})) << r.text();
    for (const char* id : {"K1", "K2", "K3", "K5", "K6", "K7", "K9", "K10"}) {
        const auto* c = r.find(id, Verdict::pass);
        EXPECT_NE(c, nullptr) << id << " has no passing result\n" << r.text();
    }
}

TEST(FcRunFolder, TheGoodMeasuredFolderNeedsNoCostsFileButABoxAndATree) {
    const Report r = check(good_measured());
    EXPECT_TRUE(r.ok()) << r.text();
    for (const char* id : {"K1", "K2", "K3", "K5", "K6", "K7", "K9", "K10"})
        EXPECT_NE(r.find(id, Verdict::pass), nullptr) << id << " has no passing result\n" << r.text();
}

TEST(FcRunFolder, EveryRequiredRunKeyIsInTheGoodFolder) {
    // A key the checker requires that the fixture lacks would make the loop tests below vacuous.
    std::string err;
    const auto kv = spade::fc::read_flat_yaml(good_budget().at("run.yaml"), &err);
    ASSERT_TRUE(err.empty()) << err;
    for (const std::string_view key : spade::fc::kRequiredRunKeys) EXPECT_EQ(kv.count(std::string(key)), 1u) << key;
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K1 -- every required file is present, by name; absence is `absent`, never a pass.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K1_AMissingRequiredFileIsAbsentAndNamed) {
    for (const char* name : {"run.yaml", "trace.bin", "commands.bin", "meters.yaml", "profile.board.yaml", "manifest.yaml", "costs.yaml"}) {
        Files f = good_budget();
        ASSERT_EQ(f.erase(name), 1u) << name;
        if (std::string(name) != "manifest.yaml") rehash(f);
        const Report r = check(f);
        EXPECT_FALSE(r.ok()) << name;
        const auto* c = r.find("K1", Verdict::absent);
        ASSERT_NE(c, nullptr) << name << "\n" << r.text();
        EXPECT_NE(c->detail.find(name), std::string::npos) << c->detail;
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K2 / K8 -- the manifest catches tampering: a byte flipped, a file truncated, one added, one removed, the manifest itself changed.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K2_AFlippedByteInAnyListedFileIsCaughtAndNamed) {
    const Files base = good_budget();
    for (const auto& [name, bytes] : base) {
        if (name == "manifest.yaml") continue;
        Files f = base;
        ASSERT_FALSE(f[name].empty()) << name;
        f[name][0] = static_cast<char>(f[name][0] ^ 0x01);
        const Report r = check(f);
        const auto* c = failing(r, "K2");
        ASSERT_NE(c, nullptr) << name << "\n" << r.text();
        EXPECT_NE(c->detail.find(name), std::string::npos) << c->detail;
        EXPECT_FALSE(r.ok());
    }
}

TEST(FcRunFolder, K2_ATruncatedFileAnUnlistedFileAndARemovedListedFileAreCaught) {
    Files f = good_budget();
    f["trace.bin"].pop_back();
    EXPECT_NE(failing(check(f), "K2"), nullptr) << "truncated";

    f = good_budget();
    f["stray.txt"] = "left behind";
    const Report un = check(f);
    ASSERT_NE(failing(un, "K2"), nullptr) << "unlisted";
    EXPECT_NE(failing(un, "K2")->detail.find("stray.txt"), std::string::npos);

    f = good_budget();
    f.erase("commands.bin");  // still listed in the manifest
    const Report gone = check(f);
    EXPECT_FALSE(gone.ok());
    EXPECT_TRUE(gone.find("K1", Verdict::absent) != nullptr || gone.find("K2", Verdict::absent) != nullptr) << gone.text();
}

TEST(FcRunFolder, K2_ABadManifestEntryAndAChangedManifestHashAreCaught) {
    Files f = good_budget();
    f["manifest.yaml"] = replace_line(f["manifest.yaml"], "  trace.bin", "  trace.bin: XYZ");
    EXPECT_NE(failing(check(f), "K2"), nullptr) << "not a hash";

    f = good_budget();
    f["manifest.yaml"] = replace_line(f["manifest.yaml"], "  commands.bin", "  commands.bin: " + hex('0'));
    EXPECT_NE(failing(check(f), "K2"), nullptr) << "wrong hash";

    f = good_budget();
    f["manifest.yaml"] = "notes: none\n";
    EXPECT_NE(failing(check(f), "K2"), nullptr) << "no files section";
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K3 -- run.yaml carries every field; the numbers are numbers.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K3_EveryRequiredKeyIsRequiredAndNamedWhenMissing) {
    for (const std::string_view key : spade::fc::kRequiredRunKeys) {
        Files f = good_budget();
        f["run.yaml"] = drop_line(f["run.yaml"], std::string(key));
        rehash(f);
        const Report r = check(f);
        EXPECT_FALSE(r.ok()) << key;
        bool named = false;
        for (const auto& c : r.results)
            if (c.verdict == Verdict::fail && c.detail.find(std::string(key)) != std::string::npos) named = true;
        EXPECT_TRUE(named) << "no failing check names " << key << "\n" << r.text();
    }
}

TEST(FcRunFolder, K3_TheStepAndSubstepsMustBePositiveIntegers) {
    for (const char* bad : {"dt_ns: 0", "dt_ns: -500000", "dt_ns: abc", "dt_ns: 1.5"}) {
        Files f = good_budget();
        f["run.yaml"] = replace_line(f["run.yaml"], "dt_ns", bad);
        rehash(f);
        EXPECT_NE(failing(check(f), "K3"), nullptr) << bad;
    }
    Files f = good_budget();
    f["run.yaml"] = replace_line(f["run.yaml"], "substeps", "substeps: 0");
    rehash(f);
    EXPECT_NE(failing(check(f), "K3"), nullptr);
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K5 -- the copies match the hashes the report states; the mode is stated; a measured run names its box and tree.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K5_AProfileOrCostsCopyThatDisagreesWithItsHashIsCaught) {
    Files f = good_budget();
    f["profile.board.yaml"] += "extra: 1\n";  // changed after the report hashed it
    rehash(f);
    const Report r = check(f);
    ASSERT_NE(failing(r, "K5"), nullptr) << r.text();
    EXPECT_NE(failing(r, "K5")->detail.find("profile"), std::string::npos);

    f = good_budget();
    f["costs.yaml"] += "extra: 1\n";
    rehash(f);
    ASSERT_NE(failing(check(f), "K5"), nullptr);
}

TEST(FcRunFolder, K5_AnUnknownModeAndAMeasuredRunWithoutABoxOrTreeAreCaught) {
    Files f = good_budget();
    f["run.yaml"] = replace_line(f["run.yaml"], "mode", "mode: emulated");
    rehash(f);
    EXPECT_NE(failing(check(f), "K5"), nullptr);

    for (const char* key : {"box", "tree"}) {
        Files m = good_measured();
        m["run.yaml"] = drop_line(m["run.yaml"], key);
        rehash(m);
        const Report r = check(m);
        ASSERT_NE(failing(r, "K5"), nullptr) << key << "\n" << r.text();
        EXPECT_NE(failing(r, "K5")->detail.find(key), std::string::npos);
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K6 -- the meters are present and finite.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K6_MetersThatAreMissingNegativeOrNotFiniteAreCaught) {
    for (const char* key : {"overruns", "late_samples", "skipped_samples"}) {
        Files f = good_budget();
        f["meters.yaml"] = drop_line(f["meters.yaml"], key);
        rehash(f);
        EXPECT_NE(failing(check(f), "K6"), nullptr) << "missing " << key;
    }
    Files f = good_budget();
    f["meters.yaml"] = replace_line(f["meters.yaml"], "overruns", "overruns: -1");
    rehash(f);
    EXPECT_NE(failing(check(f), "K6"), nullptr) << "negative";

    for (const char* bad : {"      mean: nan", "      mean: inf", "      mean: fast"}) {
        f = good_budget();
        f["meters.yaml"] = replace_line(f["meters.yaml"], "      mean", bad);
        rehash(f);
        EXPECT_NE(failing(check(f), "K6"), nullptr) << bad;
    }
    f = good_budget();
    f["meters.yaml"] = "overruns: 0\nlate_samples: 0\nskipped_samples: 0\n";  // no stage at all
    rehash(f);
    EXPECT_NE(failing(check(f), "K6"), nullptr) << "no stage latency";

    f = good_measured();
    f["meters.yaml"] = drop_line(f["meters.yaml"], "memory_peak_bytes");
    rehash(f);
    EXPECT_NE(failing(check(f), "K6"), nullptr) << "measured without its memory peak";
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K7 -- the package hash is a hash.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K7_APackageHashOfTheWrongShapeIsCaught) {
    for (const char* bad : {"package_hash: abc", "package_hash: ", "package_hash: ZZ"}) {
        Files f = good_budget();
        f["run.yaml"] = replace_line(f["run.yaml"], "package_hash", bad);
        rehash(f);
        EXPECT_NE(failing(check(f), "K7"), nullptr) << bad;
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K9 -- the not-modelled flag, and the run's grade is the weakest of the three.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K9_TheNotModelledFlagMustBeTrueAndTheRunGradeTheWeakestOfTheThree) {
    Files f = good_budget();
    f["run.yaml"] = replace_line(f["run.yaml"], "fc_compute_not_modelled", "fc_compute_not_modelled: false");
    rehash(f);
    EXPECT_NE(failing(check(f), "K9"), nullptr) << "flag false";

    f = good_budget();
    f["run.yaml"] = drop_line(f["run.yaml"], "fc_compute_not_modelled");
    rehash(f);
    EXPECT_NE(failing(check(f), "K9"), nullptr) << "flag absent";

    f = good_budget();  // world reference, mode reference, link best-effort: the run is best-effort, so `reference` overstates it
    f["run.yaml"] = replace_line(f["run.yaml"], "grade_run", "grade_run: reference");
    rehash(f);
    EXPECT_NE(failing(check(f), "K9"), nullptr) << "run grade stronger than its weakest part";

    f = good_budget();
    f["run.yaml"] = replace_line(f["run.yaml"], "grade_link", "grade_link: banded");
    f["run.yaml"] = replace_line(f["run.yaml"], "grade_run", "grade_run: banded");
    rehash(f);
    EXPECT_EQ(failing(check(f), "K9"), nullptr) << "banded is the weakest here, so banded is right";

    f = good_budget();
    f["run.yaml"] = replace_line(f["run.yaml"], "grade_world", "grade_world: golden");
    rehash(f);
    EXPECT_NE(failing(check(f), "K9"), nullptr) << "a grade outside the three";
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K10 -- no user folder in any text file; a redacted token is fine; the message never repeats the path.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K10_AUserFolderInATextFileIsCaughtAndNeverQuoted) {
    const std::string win = std::string("C:") + "\\Users\\Someone\\box";
    const std::string fwd = std::string("C:") + "/Users/Someone/box";
    const std::string lin = std::string("/ho") + "me/someone/box";
    for (const std::string& path : {win, fwd, lin}) {
        Files f = good_budget();
        f["meters.yaml"] += "note: " + path + "\n";
        rehash(f);
        const Report r = check(f);
        const auto* c = failing(r, "K10");
        ASSERT_NE(c, nullptr) << path << "\n" << r.text();
        EXPECT_NE(c->detail.find("meters.yaml"), std::string::npos);
        EXPECT_EQ(c->detail.find("Someone"), std::string::npos) << "the detail quotes the path: " << c->detail;
        EXPECT_EQ(c->detail.find("someone"), std::string::npos) << c->detail;
    }
    Files f = good_budget();
    f["meters.yaml"] += "note: %USERPROFILE%\\box\n";
    rehash(f);
    EXPECT_EQ(failing(check(f), "K10"), nullptr) << "a redacted token is the point";

    f = good_budget();
    f["trace.bin"] = std::string("\0", 1) + win;  // binary: not scanned
    rehash(f);
    EXPECT_EQ(failing(check(f), "K10"), nullptr);
}

// ---------------------------------------------------------------------------------------------------------------------------------
// K4 (the part that exists without PX4's log): the .ulg header.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, K4_TheUlogHeaderIsCheckedAndAShortOrForeignFileIsNot) {
    std::string good = std::string("ULog\x01\x12\x35", 7) + std::string(1, '\x01');
    for (int i = 0; i < 8; ++i) good += static_cast<char>(i == 0 ? 0x10 : 0);  // a start time, little-endian
    EXPECT_EQ(spade::fc::check_ulog_header(good).verdict, Verdict::pass);

    std::string bad_magic = good;
    bad_magic[0] = 'X';
    EXPECT_EQ(spade::fc::check_ulog_header(bad_magic).verdict, Verdict::fail);
    EXPECT_EQ(spade::fc::check_ulog_header(good.substr(0, 10)).verdict, Verdict::fail);
    EXPECT_EQ(spade::fc::check_ulog_header("").verdict, Verdict::fail);
}

// ---------------------------------------------------------------------------------------------------------------------------------
// The folder as it is on disk: the loader reads what a writer leaves, with forward-slash names, and the checker gives the same verdict.
// ---------------------------------------------------------------------------------------------------------------------------------
TEST(FcRunFolder, TheLoaderReadsAFolderFromDiskAndTheVerdictIsTheSame) {
    const spade::test::TempDir dir("fc_run_folder");
    const Files f = good_budget();
    for (const auto& [name, bytes] : f) {
        std::ofstream out(dir.path() / name, std::ios::binary);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        ASSERT_TRUE(out.good()) << name;
    }
    const spade::fc::RunFolder loaded = spade::fc::load_run_folder(dir.path());
    EXPECT_EQ(loaded.files, f);
    const Report r = spade::fc::check_run_folder(loaded);
    EXPECT_TRUE(r.ok()) << r.text();
}

TEST(FcRunFolder, AnEmptyOrMissingFolderIsAbsentNeverAPass) {
    EXPECT_FALSE(check(Files{}).ok());
    const spade::fc::RunFolder none = spade::fc::load_run_folder(spade::test::unique_temp_path("fc_run_folder_missing"));
    EXPECT_TRUE(none.files.empty());
    EXPECT_FALSE(spade::fc::check_run_folder(none).ok());
}
