// ---------------------------------------------------------------------------
// test_fc_run_folder.cpp -- the run-folder checker (tests/fc_run_folder_check.hpp), shown red against folders with one thing wrong.
//
// WHY. A flight's record (docs/design/core/plans/2026-11-21-fc-core.md section 4) is what a sweep and a release are judged from, so the
// check on it has to be shown to fail (docs/design/test-docs/plans/2026-11-21-fc-tests.md section 3, K1-K11), and it has to be shown
// without a PX4 run. Each case builds a good folder IN MEMORY (the format below is the checker's reading of Core's draft, a proposal
// until Core's writer lands), breaks one thing, and asks which check names it. The checker is written apart from Core's writer on
// purpose (TD-4): a checker that shares the writer's code shares its bugs.
//
// Core's trace.bin v1 and commands.bin v1 are pinned, so K3's trace hash and K11's commands are read from the bytes (fc_trace_fixture.hpp writes
// them from the spec). What the record cannot give is reported `unchecked`, never `pass`: K4 past the log's framing (the ULog reader checks a log WELL-FORMED when one is given), K6's recount (the
// record has no per-tick costs) and K11's replay (a replay needs the world, which the folder holds only by hash). The good folder's test pins
// exactly which ones, so one cannot go quiet unnoticed.
//
// SPADE_FC_SAMPLE_RUN names a run folder Core's writer filled; the case that reads it skips by name when it is unset. Core's own write test
// (FcHostRunRecord) REFUSES a non-empty folder there (a record is never overwritten), so give every run a FRESH folder, and set the variable only
// for the test that reads it.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "fc_run_folder_check.hpp"
#include "fc_trace_fixture.hpp"
#include "fc_ulog_fixture.hpp"
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
    const spade::fc::fixture::Built trace = spade::fc::fixture::build_trace();  // a trace.bin v1 and a commands.bin v1 written from the spec
    f["trace.bin"] = trace.trace;
    f["commands.bin"] = trace.commands;
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
        "fc_compute_not_modelled: true\ntrace_ticks: 3\ntrace_hash: " + trace.hash + "\n";
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
    EXPECT_EQ(unchecked, (std::set<std::string>{"K4.log", "K4.span", "K4.firmware", "K4.params", "K4.cadence", "K4.estimator", "K6.recount", "K11.replay"})) << r.text();
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
// ---------------------------------------------------------------------------------------------------------------------------------
// K3.trace_hash and K11.commands -- trace.bin v1 and commands.bin v1 (docs/design/core/plans/2026-11-21-fc-core.md section 4.1), read from
// the bytes alone. The fixture (fc_trace_fixture.hpp) writes them from the spec, apart from Core's writer.
//   K3.trace_hash   the header, the file's size and the sample records agree; the trailer is the SHA-256 of everything before it; the
//                   rolling hash recomputed tick by tick is the header's trace_hash and run.yaml's; run.yaml's trace_ticks is the header's.
//   K11.commands    commands.bin is well formed and agrees with the trace's applied rotor command at every tick. This is NOT a replay:
//                   K11.replay stays `unchecked` (a replay needs the world, which the folder holds only by hash).
// ---------------------------------------------------------------------------------------------------------------------------------
namespace {

namespace fx = spade::fc::fixture;

constexpr size_t kTickOff = 192;                          // the first tick record
constexpr size_t kRec = 76 + 12 * 4;                      // a tick record of the 4-rotor fixture
constexpr size_t kSampleOff = kTickOff + 3 * kRec;        // the sample section of the 3-tick fixture

std::string fix_trailer(std::string t) {
    t.resize(t.size() - 64);
    return t + spade::sha256_hex(t);
}

// trace.bin replaced, the manifest made honest again; run.yaml's trace_hash and trace_ticks are left as they were.
Files with_trace(Files f, const std::string& trace) {
    f["trace.bin"] = trace;
    rehash(f);
    return f;
}

Files with_commands(Files f, const std::string& commands) {
    f["commands.bin"] = commands;
    rehash(f);
    return f;
}

const std::string& good_trace() {
    static const std::string t = fx::build_trace().trace;
    return t;
}

const spade::fc::CheckResult* trace_fail(const Report& r) { return r.find("K3.trace_hash", Verdict::fail); }

// A good folder around another trace: run.yaml's trace_ticks and trace_hash follow it.
Files good_around(const fx::TraceSpec& spec) {
    Files f = good_budget();
    const fx::Built b = fx::build_trace(spec);
    f["trace.bin"] = b.trace;
    f["commands.bin"] = b.commands;
    f["run.yaml"] = replace_line(f["run.yaml"], "trace_ticks", "trace_ticks: " + std::to_string(spec.ticks));
    f["run.yaml"] = replace_line(f["run.yaml"], "trace_hash", "trace_hash: " + b.hash);
    rehash(f);
    return f;
}

}  // namespace

TEST(FcRunFolder, K3_TheTraceHashIsRecomputedFromTheBytesAndPasses) {
    const Report r = check(good_budget());
    EXPECT_NE(r.find("K3.trace_hash", Verdict::pass), nullptr) << r.text();
    EXPECT_EQ(r.find("K3.trace_hash", Verdict::unchecked), nullptr) << r.text();
    // Other shapes of the same format: six rotors, 40 ticks, not starting at tick 0.
    const Report big = check(good_around(fx::TraceSpec{6, 40, 100, 250000}));
    EXPECT_NE(big.find("K3.trace_hash", Verdict::pass), nullptr) << big.text();
    EXPECT_NE(big.find("K11.commands", Verdict::pass), nullptr) << big.text();
    EXPECT_TRUE(big.ok()) << big.text();
}

TEST(FcRunFolder, K3_ATamperedRecordIsCaughtEvenWhenTheTrailerAndTheManifestAreMadeHonest) {
    // The rolling hash is what holds a record to the run, so an edit that fixes the trailer and the manifest is still caught.
    for (const size_t at : {kTickOff + 20, kTickOff + kRec + 40, kTickOff + 2 * kRec + 8 + 8, kSampleOff + 16 + 3, kSampleOff + 5 * 16 + 20}) {
        std::string t = good_trace();
        ASSERT_LT(at, t.size() - 64);
        t[at] = static_cast<char>(t[at] ^ 0x01);
        const Report r = check(with_trace(good_budget(), fix_trailer(t)));
        const auto* c = trace_fail(r);
        ASSERT_NE(c, nullptr) << "byte " << at << "\n" << r.text();
        EXPECT_NE(c->detail.find("rolling"), std::string::npos) << c->detail;
    }
}

TEST(FcRunFolder, K3_AByteChangedWithoutFixingTheTrailerIsCaughtByTheTrailer) {
    std::string t = good_trace();
    t[kTickOff + 20] = static_cast<char>(t[kTickOff + 20] ^ 0x01);
    const Report r = check(with_trace(good_budget(), t));
    const auto* c = trace_fail(r);
    ASSERT_NE(c, nullptr) << r.text();
    EXPECT_NE(c->detail.find("trailer"), std::string::npos) << c->detail;
}

TEST(FcRunFolder, K3_TheHashesAndCountsMustAgreeWithEachOther) {
    {
        std::string t = good_trace();
        t.replace(96, 64, std::string(64, 'a'));  // the header's trace_hash
        const Report r = check(with_trace(good_budget(), fix_trailer(t)));
        const auto* c = trace_fail(r);
        ASSERT_NE(c, nullptr) << r.text();
        EXPECT_NE(c->detail.find("header"), std::string::npos) << c->detail;
    }
    {
        Files f = good_budget();
        f["run.yaml"] = replace_line(f["run.yaml"], "trace_hash", "trace_hash: " + hex('7'));
        rehash(f);
        const Report r = check(f);
        const auto* c = trace_fail(r);
        ASSERT_NE(c, nullptr) << r.text();
        EXPECT_NE(c->detail.find("run.yaml"), std::string::npos) << c->detail;
    }
    {
        Files f = good_budget();
        f["run.yaml"] = replace_line(f["run.yaml"], "trace_ticks", "trace_ticks: 4");
        rehash(f);
        const Report r = check(f);
        const auto* c = trace_fail(r);
        ASSERT_NE(c, nullptr) << r.text();
        EXPECT_NE(c->detail.find("trace_ticks"), std::string::npos) << c->detail;
    }
}

TEST(FcRunFolder, K3_AMalformedTraceIsFailedAndNamed) {
    struct Case {
        const char* what;
        std::string (*edit)(std::string);
    };
    const Case cases[] = {
        {"magic", [](std::string t) { t[0] = 'X'; return t; }},
        {"version", [](std::string t) { fx::set32(t, 4, 2); return t; }},
        {"header size", [](std::string t) { fx::set32(t, 8, 200); return t; }},
        {"tick count too large", [](std::string t) { fx::set64(t, 32, 1000); return t; }},
        {"tick count off by one", [](std::string t) { fx::set64(t, 32, 2); return t; }},
        {"rotor count", [](std::string t) { fx::set32(t, 28, 5); return t; }},
        {"tick record size", [](std::string t) { fx::set32(t, 48, 100); return t; }},
        {"sample count too small", [](std::string t) { fx::set64(t, 72, 1); return t; }},
        {"sample section offset", [](std::string t) { fx::set64(t, 64, 400); return t; }},
        {"tick number", [](std::string t) { fx::set64(t, kTickOff + kRec, 7); return t; }},
        {"tick time", [](std::string t) { fx::set64(t, kTickOff + 8, 1); return t; }},
        {"first sample of a tick", [](std::string t) { fx::set32(t, kTickOff + 68 + 16, 9); return t; }},
        {"sample count of a tick", [](std::string t) { fx::set32(t, kTickOff + 72 + 16, 9); return t; }},
        {"sample kind", [](std::string t) { fx::set32(t, kSampleOff, 3); return t; }},
        {"sample reserved word", [](std::string t) { fx::set32(t, kSampleOff + 12, 1); return t; }},
        {"a byte too many before the trailer", [](std::string t) { t.insert(t.size() - 64, 1, 'x'); return t; }},
        {"a byte too few before the trailer", [](std::string t) { t.erase(t.size() - 65, 1); return t; }},
    };
    for (const Case& c : cases) {
        const Report r = check(with_trace(good_budget(), fix_trailer(c.edit(good_trace()))));
        const auto* fail = trace_fail(r);
        ASSERT_NE(fail, nullptr) << c.what << "\n" << r.text();
        EXPECT_FALSE(fail->detail.empty()) << c.what;
    }
    // A file shorter than a header and a trailer, and the stub the first fixtures used: failed, never unchecked and never a pass.
    for (const std::string& stub : {std::string("\x01\x02\x03\x04", 4), std::string(), std::string(255, 'S')}) {
        const Report r = check(with_trace(good_budget(), stub));
        EXPECT_NE(trace_fail(r), nullptr) << r.text();
        EXPECT_EQ(r.find("K3.trace_hash", Verdict::pass), nullptr);
        EXPECT_EQ(r.find("K3.trace_hash", Verdict::unchecked), nullptr);
    }
}

TEST(FcRunFolder, K3_AMissingTraceIsAbsentNotAPass) {
    Files f = good_budget();
    f.erase("trace.bin");
    rehash(f);
    const Report r = check(f);
    EXPECT_NE(r.find("K3.trace_hash", Verdict::absent), nullptr) << r.text();
    EXPECT_EQ(r.find("K3.trace_hash", Verdict::pass), nullptr);
}

TEST(FcRunFolder, K3_TheDetailNeverQuotesTraceBytes) {
    std::string t = good_trace();
    t[kTickOff + 20] = static_cast<char>(t[kTickOff + 20] ^ 0x01);
    const Report r = check(with_trace(good_budget(), fix_trailer(t)));
    const auto* c = trace_fail(r);
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(c->detail.find(spade::sha256_hex(t)), std::string::npos);
    for (const char ch : c->detail) EXPECT_TRUE(ch >= 0x20 && ch < 0x7F) << "a non-printable byte in the detail";
}

TEST(FcRunFolder, K11_TheCommandsAreWellFormedAndConsistentWithTheTrace) {
    const Report r = check(good_budget());
    EXPECT_NE(r.find("K11.commands", Verdict::pass), nullptr) << r.text();
    // It is NOT a replay: that check stays unchecked, with its reason.
    const auto* replay = r.find("K11.replay", Verdict::unchecked);
    ASSERT_NE(replay, nullptr) << r.text();
    EXPECT_NE(replay->detail.find("world"), std::string::npos) << replay->detail;
}

TEST(FcRunFolder, K11_AMalformedOrDisagreeingCommandsFileIsFailedAndNamed) {
    const std::string good = fx::build_trace().commands;
    struct Case {
        const char* what;
        std::string (*edit)(std::string);
    };
    const Case cases[] = {
        {"magic", [](std::string c) { c[0] = 'X'; return c; }},
        {"version", [](std::string c) { fx::set32(c, 4, 2); return c; }},
        {"rotor count", [](std::string c) { fx::set32(c, 8, 3); return c; }},
        {"reserved word", [](std::string c) { fx::set32(c, 12, 1); return c; }},
        {"count says 3, two records", [](std::string c) { fx::set64(c, 16, 3); return c; }},
        {"count says 1, two records", [](std::string c) { fx::set64(c, 16, 1); return c; }},
        {"ticks not increasing", [](std::string c) { fx::set64(c, 24 + 24, 0); return c; }},
        {"a tick outside the trace", [](std::string c) { fx::set64(c, 24 + 24, 9); return c; }},
        {"a command the trace does not show", [](std::string c) { fx::set32(c, 24 + 24 + 8, 0x42280000u); return c; }},
        {"too short", [](std::string c) { return c.substr(0, 10); }},
    };
    for (const Case& c : cases) {
        const Report r = check(with_commands(good_budget(), c.edit(good)));
        const auto* fail = r.find("K11.commands", Verdict::fail);
        ASSERT_NE(fail, nullptr) << c.what << "\n" << r.text();
        EXPECT_FALSE(fail->detail.empty()) << c.what;
    }
}

TEST(FcRunFolder, K11_AMissingCommandsFileIsAbsentAndAnUnreadableTraceLeavesItUnchecked) {
    Files f = good_budget();
    f.erase("commands.bin");
    rehash(f);
    EXPECT_NE(check(f).find("K11.commands", Verdict::absent), nullptr);

    const Report r = check(with_trace(good_budget(), std::string(300, 'S')));
    EXPECT_EQ(r.find("K11.commands", Verdict::pass), nullptr) << r.text();
    EXPECT_NE(r.find("K11.commands", Verdict::unchecked), nullptr) << r.text();
}

// Core's own sample run (tests/test_fc_host_run_record.cpp leaves one in the folder SPADE_FC_SAMPLE_RUN names): the checker, written
// from the spec, must agree with Core's writer on bytes Core wrote. Unset: skipped by name.
namespace {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)  // getenv
#endif
std::string sample_run_dir() {
    const char* v = std::getenv("SPADE_FC_SAMPLE_RUN");
    return v != nullptr ? std::string(v) : std::string();
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif
}  // namespace

TEST(FcRunFolder, TheTraceAndCommandsOfCoresSampleRunAreRecomputedAndAgree) {
    const std::string dir = sample_run_dir();
    if (dir.empty()) GTEST_SKIP() << "SPADE_FC_SAMPLE_RUN is not set (Core's test_fc_host_run_record leaves a run folder there)";
    const spade::fc::RunFolder folder = spade::fc::load_run_folder(dir);
    ASSERT_FALSE(folder.files.empty()) << "no run folder at SPADE_FC_SAMPLE_RUN";
    const Report r = spade::fc::check_run_folder(folder);
    SCOPED_TRACE(r.text());
    EXPECT_NE(r.find("K3.trace_hash", Verdict::pass), nullptr);
    EXPECT_NE(r.find("K11.commands", Verdict::pass), nullptr);
    EXPECT_TRUE(r.ok());
    std::set<std::string> unchecked;
    for (const auto& c : r.results)
        if (c.verdict == Verdict::unchecked) unchecked.insert(c.id);
    EXPECT_EQ(unchecked, (std::set<std::string>{"K4.log", "K4.span", "K4.firmware", "K4.params", "K4.cadence", "K4.estimator", "K6.recount", "K11.replay"}));
}

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

// ---------------------------------------------------------------------------------------------------------------------------------
// K4.log: a ULog file is WELL-FORMED (tests/fc_ulog_check.hpp, from the ULog specification; the fixture writes the files from the same
// specification). The claim is the framing and the section rules, not that PX4's data is right.
// ---------------------------------------------------------------------------------------------------------------------------------
namespace {

namespace ux = spade::fc::ulogfx;
using spade::fc::UlogResult;

[[nodiscard]] UlogResult read_msgs(const std::vector<ux::Msg>& m, uint8_t version = 1) { return spade::fc::read_ulog(ux::assemble(m, version)); }

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // getenv
#endif
[[nodiscard]] std::string ulog_env_path() {
    const char* v = std::getenv("SPADE_FC_ULOG");
    return v != nullptr ? std::string(v) : std::string();
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

struct UlogCase {
    const char* name;
    std::string bytes;
    const char* expect;  // a phrase the detail must hold
};

}  // namespace

TEST(FcRunFolder, K4_AWellFormedLogIsReadAndCounted) {
    const UlogResult r = read_msgs(ux::good_messages());
    EXPECT_EQ(r.kind, UlogResult::Kind::well_formed) << r.detail;
    EXPECT_EQ(r.messages, 18u);
    EXPECT_EQ(r.definitions, 9u);
    EXPECT_EQ(r.data_messages, 3u);
    EXPECT_EQ(r.subscriptions, 2u);
    EXPECT_EQ(r.syncs, 1u);
    EXPECT_EQ(r.dropouts, 1u);
    EXPECT_NE(r.detail.find("well-formed"), std::string::npos) << r.detail;
}

TEST(FcRunFolder, K4_EveryMalformedLogIsFailedAndNamed) {
    using namespace ux;
    const std::string good = assemble(good_messages());
    const auto with = [](const std::function<void(std::vector<Msg>&)>& f) {
        std::vector<Msg> v = good_messages();
        f(v);
        return assemble(v);
    };
    std::vector<UlogCase> cases;
    cases.push_back({"short header", good.substr(0, 10), "shorter than the 16-byte header"});
    cases.push_back({"bad magic", std::string("X") + good.substr(1), "ULog magic"});
    cases.push_back({"version 2", assemble(good_messages(), 2), "version byte is 2"});
    cases.push_back({"truncated body", good.substr(0, good.size() - 3), "is cut off"});
    cases.push_back({"cut header", good + std::string("\x05\x00", 2), "message header is cut off"});
    cases.push_back({"size past the end", good + std::string("\xFF\x00", 2) + "D", "is cut off"});
    cases.push_back({"first is not B", with([](auto& v) { v.erase(v.begin()); }), "not the flag bits message"});
    cases.push_back({"B too short", with([](auto& v) { v[kB].body.resize(39); }), "under 40"});
    cases.push_back({"unknown incompat bit", with([](auto& v) { v[kB].body[9] = 1; }), "does not know"});
    cases.push_back({"an unknown type as the first message", with([](auto& v) { v.insert(v.begin(), Msg{'Z', "x"}); }), "not the flag bits message"});
    cases.push_back({"an unknown type, then a truncated tail", [&] { return with([](auto& v) { v.insert(v.begin() + kSubGyro, Msg{'Z', "x"}); }).substr(0, with([](auto& v) { v.insert(v.begin() + kSubGyro, Msg{'Z', "x"}); }).size() - 3); }(), "is cut off"});
    cases.push_back({"D in the definitions", with([](auto& v) { v.insert(v.begin() + kMulti, Msg{'D', std::string("\x00\x00", 2)}); }), "definitions section"});
    cases.push_back({"D for an unsubscribed id", with([](auto& v) { v[kDataGyro0].body[0] = 9; }), "was not subscribed"});
    cases.push_back({"A for an undefined format", with([](auto& v) { v[kSubGyro].body = std::string("\0\0\0nope", 7); }), "did not define"});
    cases.push_back({"A twice", with([](auto& v) { v.insert(v.begin() + kSubStatus, v[kSubGyro]); }), "subscribed twice"});
    cases.push_back({"F in the data section", with([](auto& v) { v.insert(v.begin() + kDataGyro0, v[kFGyro]); }), "in the data section"});
    cases.push_back({"second B", with([](auto& v) { v.insert(v.begin() + kFStatus, v[kB]); }), "not the first"});
    cases.push_back({"F that is not name:field;", with([](auto& v) { v[kFGyro].body = "nocolon"; }), "name:field"});
    cases.push_back({"I key longer than the message", with([](auto& v) { v[kInfoName].body = std::string("\x32", 1) + "abc"; }), "key length"});
    cases.push_back({"Q with no default bit", with([](auto& v) { v[kDefault].body[0] = 0; }), "no default type bit"});
    cases.push_back({"S with another magic", with([](auto& v) { v[kSync].body[0] = 0; }), "synchronization"});
    cases.push_back({"O of 3 bytes", with([](auto& v) { v[kDropout].body += "x"; }), "dropout message of 3"});
    cases.push_back({"L under 9 bytes", with([](auto& v) { v[kLogged].body.resize(5); }), "logged string of 5"});
    cases.push_back({"C under 11 bytes", with([](auto& v) { v[kTagged].body.resize(10); }), "tagged logged string of 10"});
    cases.push_back({"no data message", with([](auto& v) { v.resize(kSubStatus + 1); }), "no data message"});
    cases.push_back({"only the header", good.substr(0, 16), "no messages after the header"});
    for (const UlogCase& c : cases) {
        const UlogResult r = spade::fc::read_ulog(c.bytes);
        EXPECT_EQ(r.kind, UlogResult::Kind::malformed) << c.name << ": " << r.detail;
        EXPECT_NE(r.detail.find(c.expect), std::string::npos) << c.name << ": " << r.detail;
    }
}

// The spec says a parser "must ignore unknown messages" and a new message type can be announced by a new compat flag bit, so a type this reader does not
// know, with its framing intact, is skipped and named: `unread`, never a pass (the type was not read) and never a failure (it may be valid).
TEST(FcRunFolder, K4_AnUnknownTypeWithIntactFramingIsUnreadAndNamedNeverAPassOrAFailure) {
    using namespace ux;
    std::vector<Msg> v = good_messages();
    v.insert(v.begin() + kSubGyro, Msg{'Z', "x"});            // in the definitions' end
    v.insert(v.begin() + kDataGyro0 + 1, Msg{'\x01', "yy"});  // in the data section, a type that is not printable
    v.insert(v.begin() + kDataGyro0 + 2, Msg{'Z', ""});       // the same type again, an empty body
    const UlogResult r = read_msgs(v);
    EXPECT_EQ(r.kind, UlogResult::Kind::unread) << r.detail;
    EXPECT_EQ(r.unknown_messages, 3u);
    EXPECT_NE(r.detail.find("unknown message type 'Z', 0x01"), std::string::npos) << r.detail;
    EXPECT_NE(r.detail.find("3 messages"), std::string::npos) << r.detail;
    EXPECT_EQ(r.data_messages, 3u) << "the known messages around them are still read";

    const Report folder = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), assemble(v)});
    EXPECT_NE(folder.find("K4.log", Verdict::unchecked), nullptr) << folder.text();
    EXPECT_EQ(folder.find("K4.log", Verdict::fail), nullptr) << folder.text();
    EXPECT_EQ(folder.find("K4.log", Verdict::pass), nullptr) << folder.text();
    EXPECT_TRUE(folder.ok()) << folder.text();
}

TEST(FcRunFolder, K4_AppendedDataIsUnreadNeverAPass) {
    using namespace ux;
    for (const size_t byte : {size_t{8}, size_t{16}}) {  // incompat bit 0 (DATA_APPENDED), then a non-zero appended offset
        std::vector<Msg> v = good_messages();
        v[kB].body[byte] = 1;
        const UlogResult r = read_msgs(v);
        EXPECT_EQ(r.kind, UlogResult::Kind::unread) << byte << ": " << r.detail;
        EXPECT_NE(r.detail.find("appended data"), std::string::npos) << r.detail;
    }
}

TEST(FcRunFolder, K4_TheDetailNeverQuotesTheLogsBytes) {
    using namespace ux;
    std::vector<Msg> v = good_messages();
    v[kInfoName].body = keyed("char[24] note", "C:\\Users\\Zed\\SECRETVALUE");
    v[kSubGyro].body = std::string("\0\0\0SECRETFORMAT", 15);
    const UlogResult a = read_msgs(v);
    EXPECT_EQ(a.kind, UlogResult::Kind::malformed);
    v = good_messages();
    v[kInfoName].body = keyed("char[24] note", "C:\\Users\\Zed\\SECRETVALUE");
    std::string bytes = assemble(v);
    bytes.resize(bytes.size() - 2);  // a truncated tail
    const UlogResult b = spade::fc::read_ulog(bytes);
    EXPECT_EQ(b.kind, UlogResult::Kind::malformed);
    for (const UlogResult* r : {&a, &b}) {
        EXPECT_EQ(r->detail.find("SECRET"), std::string::npos) << r->detail;
        EXPECT_EQ(r->detail.find("Zed"), std::string::npos) << r->detail;
    }
}

TEST(FcRunFolder, K4_TheLogIsReadFromTheFolderAndAMalformedOneFailsTheCheck) {
    const std::string good = ux::assemble(ux::good_messages());
    const Report pass = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), good});
    EXPECT_TRUE(pass.ok()) << pass.text();
    EXPECT_NE(pass.find("K4.log", Verdict::pass), nullptr) << pass.text();

    const Report bad = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), good.substr(0, good.size() - 3)});
    EXPECT_FALSE(bad.ok()) << bad.text();
    ASSERT_NE(bad.find("K4.log", Verdict::fail), nullptr) << bad.text();
    EXPECT_NE(bad.find("K4.log", Verdict::fail)->detail.find("is cut off"), std::string::npos);

    std::vector<ux::Msg> v = ux::good_messages();
    v[ux::kB].body[8] = 1;
    const Report unread = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), ux::assemble(v)});
    EXPECT_NE(unread.find("K4.log", Verdict::unchecked), nullptr) << unread.text();
    EXPECT_TRUE(unread.ok()) << unread.text();

    const Report none = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured()});
    EXPECT_NE(none.find("K4.log", Verdict::unchecked), nullptr) << none.text();
}

// K4.span: the log's simulated-time span against the run's simulated time. The stamp is the FIRST field of a data message whose format declares it
// `uint64_t timestamp`; a 0 is counted per format and kept out of the span; a format that does not start that way is counted "not timed".
namespace {

[[nodiscard]] std::string timed_d(uint16_t id, uint64_t t, size_t extra = 16) {
    std::string d;
    ux::put16(d, id);
    ux::put64(d, t);
    d += std::string(extra, '\0');
    return d;
}

[[nodiscard]] const spade::fc::CheckResult* span_of(const Report& r) {
    for (const auto& c : r.results)
        if (c.id == "K4.span") return &c;
    return nullptr;
}

[[nodiscard]] Report check_with(const std::string& log, std::optional<uint64_t> sim_us) {
    return spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), log, sim_us});
}

}  // namespace

TEST(FcRunFolder, K4_TheSpanIsReadFromTheFirstFieldStampsZerosCountedNotUsedUntimedCounted) {
    using namespace ux;
    std::vector<Msg> v = good_messages();  // gyro 1000, status 4000, gyro 5000
    v.insert(v.begin() + kFStatus + 1, Msg{'F', "weird:uint32_t count;uint64_t timestamp;"});  // not timed: the stamp is not the first field
    std::string a2(1, '\0');
    put16(a2, 2);
    a2 += "weird";
    v.insert(v.begin() + kSubStatus + 1, Msg{'A', a2});
    v.push_back({'D', timed_d(0, 0)});                                  // a gyro message stamped 0
    v.push_back({'D', timed_d(0, 0)});                                  // and another
    v.push_back({'D', std::string("\x02\x00\x07\x00\x00\x00\x09\x00\x00\x00\x00\x00\x00\x00", 14)});  // a `weird` message
    const UlogResult r = read_msgs(v);
    ASSERT_EQ(r.kind, UlogResult::Kind::well_formed) << r.detail;
    EXPECT_TRUE(r.has_time);
    EXPECT_EQ(r.first_stamp_us, 1000u);
    EXPECT_EQ(r.last_stamp_us, 5000u);
    EXPECT_EQ(r.timed_messages, 5u);
    EXPECT_EQ(r.zero_stamps, 2u);
    EXPECT_EQ(r.zero_formats, "sensor_gyro 2");
    EXPECT_EQ(r.untimed_messages, 1u);
    EXPECT_EQ(r.untimed_formats, "weird");
    EXPECT_EQ(r.header_start_us, 316000u) << "the header's start time is reported";
    EXPECT_NE(r.header_start_us, r.first_stamp_us) << "and it is not the span's start";

    const Report folder = check_with(assemble(v), 5000);
    const auto* span = span_of(folder);
    ASSERT_NE(span, nullptr) << folder.text();
    EXPECT_EQ(span->verdict, Verdict::pass) << span->detail;
    EXPECT_NE(span->detail.find("gap 0 us"), std::string::npos) << span->detail;
    EXPECT_NE(span->detail.find("1000 to 5000 us over 3 messages"), std::string::npos) << span->detail;
    EXPECT_NE(span->detail.find("2 stamped 0 and left out (sensor_gyro 2)"), std::string::npos) << span->detail;
    EXPECT_NE(span->detail.find("1 not timed (weird)"), std::string::npos) << span->detail;
}

TEST(FcRunFolder, K4_TheSpanMustEndAtTheRunsTimeNeverAfterAndNotFarBefore) {
    const std::string log = ux::assemble(ux::good_messages());  // the last stamp is 5000
    const auto verdict = [&](uint64_t sim) {
        const Report r = check_with(log, sim);
        const auto* s = span_of(r);
        return s != nullptr ? s->verdict : Verdict::absent;
    };
    EXPECT_EQ(verdict(5000), Verdict::pass);
    EXPECT_EQ(verdict(5000 + 100'000), Verdict::pass) << "a log may end 100 ms before the run's last time";
    EXPECT_EQ(verdict(5000 + 100'001), Verdict::fail) << "one microsecond more is a log that ended early";
    EXPECT_EQ(verdict(4999), Verdict::fail) << "a log cannot hold a time that had not come";
    const Report early = check_with(log, 5000 + 250'000);
    ASSERT_NE(span_of(early), nullptr);
    EXPECT_NE(span_of(early)->detail.find("250000 us before"), std::string::npos) << span_of(early)->detail;
    EXPECT_FALSE(early.ok()) << early.text();
    const Report late = check_with(log, 4000);
    EXPECT_NE(span_of(late)->detail.find("1000 us AFTER"), std::string::npos) << span_of(late)->detail;
    EXPECT_FALSE(late.ok()) << late.text();
}

TEST(FcRunFolder, K4_TheSpanIsUncheckedWhenItCannotBeCompared) {
    using namespace ux;
    // no simulated time given
    EXPECT_EQ(span_of(check_with(assemble(good_messages()), std::nullopt))->verdict, Verdict::unchecked);
    // no log given
    const Report none = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), std::nullopt, 5000});
    ASSERT_NE(span_of(none), nullptr);
    EXPECT_EQ(span_of(none)->verdict, Verdict::unchecked);
    // a log with an unknown type is unread, so its stamps were not read
    std::vector<Msg> unk = good_messages();
    unk.insert(unk.begin() + kSubGyro, Msg{'Z', "x"});
    EXPECT_EQ(span_of(check_with(assemble(unk), 5000))->verdict, Verdict::unchecked);
    // a malformed log fails K4.log; its span is not also failed
    const std::string good = assemble(good_messages());
    EXPECT_EQ(span_of(check_with(good.substr(0, good.size() - 3), 5000))->verdict, Verdict::unchecked);
    // only zero stamps: no span
    std::vector<Msg> zeros = good_messages();
    zeros[kDataGyro0].body = timed_d(0, 0);
    zeros[kDataStatus].body = timed_d(1, 0, 1);
    zeros[kDataGyro1].body = timed_d(0, 0);
    const UlogResult r = read_msgs(zeros);
    ASSERT_EQ(r.kind, UlogResult::Kind::well_formed) << r.detail;
    EXPECT_FALSE(r.has_time);
    const Report zero_folder = check_with(assemble(zeros), 5000);
    const auto* span = span_of(zero_folder);
    ASSERT_NE(span, nullptr);
    EXPECT_EQ(span->verdict, Verdict::unchecked);
    EXPECT_NE(span->detail.find("nonzero timestamp"), std::string::npos) << span->detail;
}

TEST(FcRunFolder, K4_ADataMessageTooShortForItsTimestampIsMalformed) {
    using namespace ux;
    std::vector<Msg> v = good_messages();
    v[kDataGyro1].body = std::string("\x00\x00\x01\x02\x03", 5);  // id 0 and 3 bytes of a 64-bit stamp
    const UlogResult r = read_msgs(v);
    EXPECT_EQ(r.kind, UlogResult::Kind::malformed) << r.detail;
    EXPECT_NE(r.detail.find("under 10"), std::string::npos) << r.detail;
}

TEST(FcRunFolder, K4_TheRunsSimulatedTimeIsReadFromTheLockstepReport) {
    using spade::fc::last_time_usec_of;
    EXPECT_EQ(last_time_usec_of("steps_done: 10000\nlast_time_usec: 40032000\nboot_steps: 8\n"), std::optional<uint64_t>(40032000));
    EXPECT_EQ(last_time_usec_of("last_time_usec: 7\r\nx: 1\r\n"), std::optional<uint64_t>(7));
    EXPECT_EQ(last_time_usec_of("last_time_usec:   12  "), std::optional<uint64_t>(12));
    EXPECT_FALSE(last_time_usec_of("steps_done: 1\n").has_value());
    EXPECT_FALSE(last_time_usec_of("last_time_usec: \n").has_value());
    EXPECT_FALSE(last_time_usec_of("last_time_usec: 12x\n").has_value());
    EXPECT_FALSE(last_time_usec_of("last_time_usec: -5\n").has_value());
    EXPECT_FALSE(last_time_usec_of("last_time_usec: 99999999999999999999\n").has_value()) << "too many digits";
    EXPECT_FALSE(last_time_usec_of("not_last_time_usec: 5\n").has_value()) << "the key is the line's start";
}

// K4.firmware: the log's `ver_sw` (the git commit) and `ver_sw_release` (PX4's 0xMMmmPPtt) must be the pinned v1.17.0, d6f12ad1.
namespace {

[[nodiscard]] const spade::fc::CheckResult* result_of(const Report& r, std::string_view id) {
    for (const auto& c : r.results)
        if (c.id == id) return &c;
    return nullptr;
}

[[nodiscard]] std::string float_bytes(float f) {
    std::string s(4, '\0');
    std::memcpy(s.data(), &f, 4);
    return s;
}

[[nodiscard]] std::string int_bytes(int32_t v) {
    std::string s(4, '\0');
    std::memcpy(s.data(), &v, 4);
    return s;
}

}  // namespace

TEST(FcRunFolder, K4_TheFirmwareMustBeThePinnedCommitAndRelease) {
    using namespace ux;
    const auto verdict_of = [](const std::vector<Msg>& v) {
        const Report r = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), assemble(v)});
        const auto* c = result_of(r, "K4.firmware");
        return c != nullptr ? std::make_pair(c->verdict, c->detail) : std::make_pair(Verdict::absent, std::string("no K4.firmware row"));
    };
    const std::vector<Msg> good = good_messages();
    const auto ok = verdict_of(good);
    EXPECT_EQ(ok.first, Verdict::pass) << ok.second;
    EXPECT_NE(ok.second.find("PX4 1.17.0 (release), commit d6f12ad1"), std::string::npos) << ok.second;

    std::vector<Msg> other_commit = good;
    other_commit[kInfoSw].body = keyed("char[40] ver_sw", "0123456789abcdef0123456789abcdef01234567");
    auto r = verdict_of(other_commit);
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("commit is 01234567, not the pinned d6f12ad1"), std::string::npos) << r.second;

    std::vector<Msg> old_release = good;
    old_release[kInfoVer].body = keyed("uint32_t ver_sw_release", std::string("\xFF\x02\x04\x01", 4));  // 1.4.2
    r = verdict_of(old_release);
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("1.4.2 (release), not the pinned 1.17.0 (release)"), std::string::npos) << r.second;

    std::vector<Msg> dev_build = good;
    dev_build[kInfoVer].body = keyed("uint32_t ver_sw_release", std::string("\x01\x00\x11\x01", 4));  // type 1, not a release
    r = verdict_of(dev_build);
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("(type 1)"), std::string::npos) << r.second;

    std::vector<Msg> no_sw = good;
    no_sw.erase(no_sw.begin() + kInfoSw);
    r = verdict_of(no_sw);
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("no ver_sw information message"), std::string::npos) << r.second;

    std::vector<Msg> no_release = good;
    no_release.erase(no_release.begin() + kInfoVer);
    r = verdict_of(no_release);
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("no ver_sw_release information message"), std::string::npos) << r.second;

    std::vector<Msg> not_hash = good;
    not_hash[kInfoSw].body = keyed("char[40] ver_sw", "D6F12AD1C4F70AD3230AFD7D86E971421E02FEF4");  // upper case
    r = verdict_of(not_hash);
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("not a 40-digit lower-case commit hash"), std::string::npos) << r.second;
    EXPECT_EQ(r.second.find("D6F12AD1"), std::string::npos) << "the detail never quotes the log's bytes: " << r.second;

    std::vector<Msg> unknown = good;
    unknown.insert(unknown.begin() + kSubGyro, Msg{'Z', "x"});
    EXPECT_EQ(verdict_of(unknown).first, Verdict::unchecked) << "an unread log's version is not read";
    const Report none = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured()});
    EXPECT_NE(none.find("K4.firmware", Verdict::unchecked), nullptr) << none.text();
}

// K4.params: every `param set-default NAME VALUE` of the airframe file is the log's value of that parameter (the log holds the parameters in effect).
TEST(FcRunFolder, K4_TheAirframeFilesParametersMustEqualTheLogs) {
    using namespace ux;
    std::vector<Msg> v = good_messages();
    v.insert(v.begin() + kMulti, Msg{'P', keyed("float CA_ROTOR0_PX", float_bytes(0.13F))});
    v.insert(v.begin() + kMulti, Msg{'P', keyed("int32_t CA_ROTOR_COUNT", int_bytes(4))});
    v.insert(v.begin() + kMulti, Msg{'P', keyed("float MPC_THR_HOVER", float_bytes(0.60F))});
    v.insert(v.begin() + kMulti, Msg{'P', keyed("char[8] SOME_STRING", "abcdefgh")});  // another type: not kept
    const std::string log = assemble(v);
    const UlogResult read = read_msgs(v);
    ASSERT_EQ(read.kind, UlogResult::Kind::well_formed) << read.detail;
    EXPECT_EQ(read.params.size(), 4u) << "SYS_AUTOSTART of the fixture and the three added; the string parameter is not kept";

    const std::string header_lines =
        "#!/bin/sh\n#\n# @name Test x500\n#\n. ${R}etc/init.d/rc.mc_defaults\n\n";
    const auto check = [&](const std::string& airframe) {
        const Report r = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), log, std::nullopt, airframe});
        const auto* c = result_of(r, "K4.params");
        return c != nullptr ? std::make_pair(c->verdict, c->detail) : std::make_pair(Verdict::absent, std::string("no K4.params row"));
    };
    auto r = check(header_lines + "param set-default CA_ROTOR_COUNT 4\nparam set-default CA_ROTOR0_PX 0.13\nparam set-default MPC_THR_HOVER 0.60\n");
    EXPECT_EQ(r.first, Verdict::pass) << r.second;
    EXPECT_NE(r.second.find("all 3"), std::string::npos) << r.second;
    EXPECT_NE(r.second.find("the log holds 4"), std::string::npos) << r.second;

    r = check(header_lines + "param set-default CA_ROTOR_COUNT 4\r\nparam set CA_ROTOR0_PX 0.13\r\n  param   set-default\tMPC_THR_HOVER   0.6  \r\n");
    EXPECT_EQ(r.first, Verdict::pass) << "CRLF, `param set`, extra blanks: " << r.second;

    r = check(header_lines + "param set-default MPC_THR_HOVER 0.55\n");
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("MPC_THR_HOVER is 0.6"), std::string::npos) << r.second;
    EXPECT_NE(r.second.find("0.55 in the file"), std::string::npos) << r.second;

    r = check(header_lines + "param set-default NAV_DLL_ACT 0\n");
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("NAV_DLL_ACT is not in the log's parameters"), std::string::npos) << r.second;

    r = check(header_lines + "param set-default CA_ROTOR_COUNT 4.0\n");
    EXPECT_EQ(r.first, Verdict::fail) << "an integer parameter written with a decimal point is not the same text";

    r = check(header_lines + "param set-default CA_ROTOR_COUNT 5\nparam set-default MPC_THR_HOVER 0.55\nparam set-default NAV_DLL_ACT 0\n");
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("CA_ROTOR_COUNT is 4 in the log, 5 in the file; MPC_THR_HOVER"), std::string::npos) << "every difference is named: " << r.second;
    EXPECT_NE(r.second.find("NAV_DLL_ACT is not in"), std::string::npos) << r.second;

    r = check(header_lines + "param set-default CA_ROTOR_COUNT 4\nparam set-default PWM_MAIN_FUNC1 $FUNC\n");
    EXPECT_EQ(r.first, Verdict::unchecked) << "a value that is not a plain number is named and not compared: " << r.second;
    EXPECT_NE(r.second.find("PWM_MAIN_FUNC1"), std::string::npos) << r.second;

    r = check(header_lines);
    EXPECT_EQ(r.first, Verdict::unchecked) << "a file with no parameter line has nothing to compare: " << r.second;

    const Report none = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), log});
    EXPECT_NE(none.find("K4.params", Verdict::unchecked), nullptr) << none.text();
}

TEST(FcRunFolder, K4_TheAirframeFilesParameterLinesAreRead) {
    const spade::fc::AirframeParams p = spade::fc::airframe_params_of(
        "#!/bin/sh\r\n# param set-default IN_A_COMMENT 1\r\n. ${R}etc/init.d/rc.mc_defaults\r\nparam set-default A_B 0.25\r\n"
        "\tparam set C_D -3\nparam show X\nparam set-default NO_VALUE\nparam set-default E_F 1e5\nparam set-default G_H +7\nparam reset_all\n");
    ASSERT_EQ(p.params.size(), 3u);
    EXPECT_EQ(p.params[0].name, "A_B");
    EXPECT_EQ(p.params[0].value, "0.25");
    EXPECT_EQ(p.params[1].name, "C_D");
    EXPECT_EQ(p.params[1].value, "-3");
    EXPECT_EQ(p.params[2].name, "G_H");
    EXPECT_EQ(p.unread, "NO_VALUE") << "the first line without a plain number value";
    EXPECT_TRUE(spade::fc::plain_number("-0.5"));
    EXPECT_FALSE(spade::fc::plain_number("1e5"));
    EXPECT_FALSE(spade::fc::plain_number("."));
    EXPECT_FALSE(spade::fc::plain_number("1.2.3"));
    EXPECT_FALSE(spade::fc::plain_number(""));
}

// K4.cadence: the IMU stream (`sensor_combined`, instance 0) must run on the lockstep clock: every gap between its stamps a multiple of the link step.
// A gap of more than one step is a sample the log does not hold: counted and printed, not judged (see the commit message: the two real runs hold ~95%).
namespace {

[[nodiscard]] std::vector<ux::Msg> imu_log(const std::vector<uint64_t>& stamps, uint8_t multi = 0) {
    using namespace ux;
    std::vector<Msg> v = good_messages();
    v.insert(v.begin() + kFStatus + 1, Msg{'F', "sensor_combined:uint64_t timestamp;float[3] gyro_rad;"});
    std::string a(1, static_cast<char>(multi));
    put16(a, 5);
    a += "sensor_combined";
    v.insert(v.begin() + kSubStatus + 1, Msg{'A', a});
    for (const uint64_t t : stamps) v.push_back({'D', timed_d(5, t, 12)});
    return v;
}

[[nodiscard]] std::pair<Verdict, std::string> cadence_of(const std::vector<ux::Msg>& v, std::optional<uint64_t> step = 4000) {
    const Report r = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), ux::assemble(v), std::nullopt, std::nullopt, step});
    const auto* c = result_of(r, "K4.cadence");
    return c != nullptr ? std::make_pair(c->verdict, c->detail) : std::make_pair(Verdict::absent, std::string("no K4.cadence row"));
}

}  // namespace

TEST(FcRunFolder, K4_TheImuStreamsGapsAreMultiplesOfTheLinkStepAndMissingStepsAreCounted) {
    const std::vector<ux::Msg> v = imu_log({4000, 8000, 12000, 20000, 24000});
    const UlogResult read = read_msgs(v);
    ASSERT_EQ(read.kind, UlogResult::Kind::well_formed) << read.detail;
    const UlogResult::Stream* imu = nullptr;
    for (const auto& s : read.streams)
        if (s.name == "sensor_combined") imu = &s;
    ASSERT_NE(imu, nullptr);
    EXPECT_EQ(imu->n, 5u);
    EXPECT_EQ(imu->first_us, 4000u);
    EXPECT_EQ(imu->last_us, 24000u);
    EXPECT_EQ(imu->gcd_us, 4000u);
    EXPECT_EQ(imu->min_gap_us, 4000u);
    EXPECT_EQ(imu->max_gap_us, 8000u);
    EXPECT_TRUE(imu->monotone);

    const auto r = cadence_of(v);
    EXPECT_EQ(r.first, Verdict::pass) << r.second;
    EXPECT_NE(r.second.find("sensor_combined 5 samples, stamps 4000 to 24000 us, gaps 4000 to 8000 us; 1 of 6 link steps have no logged sample"), std::string::npos) << r.second;
    EXPECT_EQ(cadence_of(imu_log({0, 4000, 8000})).first, Verdict::pass) << "a stamp of 0 is not a sample";
    EXPECT_EQ(cadence_of(imu_log({4000, 8000}), 2000).first, Verdict::pass) << "the step is the caller's";
}

TEST(FcRunFolder, K4_TheImuStreamFailsWhenOffTheClockBackwardsOrRepeated) {
    auto r = cadence_of(imu_log({4000, 8000, 12001}));
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("not all multiples of the 4000 us link step (their greatest common divisor is 1 us)"), std::string::npos) << r.second;
    r = cadence_of(imu_log({4000, 6000, 10000}));
    EXPECT_EQ(r.first, Verdict::fail) << "2000 is not a multiple of 4000: " << r.second;
    r = cadence_of(imu_log({4000, 12000, 8000}));
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("earlier than the one before"), std::string::npos) << r.second;
    r = cadence_of(imu_log({4000, 4000, 8000}));
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("1 sensor_combined stamps repeat"), std::string::npos) << r.second;
}

TEST(FcRunFolder, K4_TheCadenceIsUncheckedWhenItCannotBeRead) {
    EXPECT_EQ(cadence_of(imu_log({4000, 8000}), std::nullopt).first, Verdict::unchecked) << "no link step given";
    EXPECT_EQ(cadence_of(imu_log({4000, 8000}), 0).first, Verdict::unchecked) << "a step of 0";
    EXPECT_EQ(cadence_of(imu_log({4000})).first, Verdict::unchecked) << "one sample has no gap";
    EXPECT_EQ(cadence_of(ux::good_messages()).first, Verdict::unchecked) << "the log holds no sensor_combined";
    EXPECT_EQ(cadence_of(imu_log({4000, 8000}, 1)).first, Verdict::unchecked) << "only instance 1 is there; instance 0 is read";
    std::vector<ux::Msg> unknown = imu_log({4000, 8000});
    unknown.insert(unknown.begin() + ux::kSubGyro, ux::Msg{'Z', "x"});
    EXPECT_EQ(cadence_of(unknown).first, Verdict::unchecked) << "an unread log's samples are not read";
    const Report none = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured()});
    EXPECT_NE(none.find("K4.cadence", Verdict::unchecked), nullptr) << none.text();
}

// K4.estimator: the estimator ran at every link step. PX4's `ekf2_timestamps` (instance 0) holds a sample per EKF2 update; in a lockstep run the update
// is once per link step, so every step between its first and last stamp must have a sample. (sensor_combined's missing steps are information, K4.cadence.)
namespace {

[[nodiscard]] std::vector<ux::Msg> ekf_log(const std::vector<uint64_t>& stamps, uint8_t multi = 0) {
    using namespace ux;
    std::vector<Msg> v = good_messages();
    v.insert(v.begin() + kFStatus + 1, Msg{'F', "ekf2_timestamps:uint64_t timestamp;int16_t gyro_timestamp_rel;"});
    std::string a(1, static_cast<char>(multi));
    put16(a, 6);
    a += "ekf2_timestamps";
    v.insert(v.begin() + kSubStatus + 1, Msg{'A', a});
    for (const uint64_t t : stamps) v.push_back({'D', timed_d(6, t, 2)});
    return v;
}

[[nodiscard]] std::pair<Verdict, std::string> estimator_of(const std::vector<ux::Msg>& v, std::optional<uint64_t> step = 4000) {
    const Report r = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured(), ux::assemble(v), std::nullopt, std::nullopt, step});
    const auto* c = result_of(r, "K4.estimator");
    return c != nullptr ? std::make_pair(c->verdict, c->detail) : std::make_pair(Verdict::absent, std::string("no K4.estimator row"));
}

}  // namespace

TEST(FcRunFolder, K4_TheReaderNotesWhereEachKindOfGapFirstOccurs) {
    const UlogResult read = read_msgs(ekf_log({4000, 8000, 16000, 20000, 32000, 36000}));
    ASSERT_EQ(read.kind, UlogResult::Kind::well_formed) << read.detail;
    const UlogResult::Stream* ekf = nullptr;
    for (const auto& s : read.streams)
        if (s.name == "ekf2_timestamps") ekf = &s;
    ASSERT_NE(ekf, nullptr);
    ASSERT_EQ(ekf->first_at_gap.size(), 3u);
    EXPECT_EQ(ekf->first_at_gap.at(4000), 4000u) << "the first 4000 us gap follows the stamp 4000";
    EXPECT_EQ(ekf->first_at_gap.at(8000), 8000u) << "the first 8000 us gap follows the stamp 8000";
    EXPECT_EQ(ekf->first_at_gap.count(12000), 1u);
    EXPECT_EQ(ekf->first_at_gap.at(12000), 20000u);
}

TEST(FcRunFolder, K4_TheEstimatorHasASampleAtEveryLinkStep) {
    const auto r = estimator_of(ekf_log({4000, 8000, 12000, 16000, 20000, 24000}));
    EXPECT_EQ(r.first, Verdict::pass) << r.second;
    EXPECT_NE(r.second.find("ekf2_timestamps 6 samples, stamps 4000 to 24000 us, one every 4000 us link step: 6 of 6 steps"), std::string::npos) << r.second;
    EXPECT_EQ(estimator_of(ekf_log({0, 4000, 8000})).first, Verdict::pass) << "a stamp of 0 is not a sample";
    EXPECT_EQ(estimator_of(ekf_log({4000, 6000, 8000}), 2000).first, Verdict::pass) << "the step is the caller's";
}

TEST(FcRunFolder, K4_AMissingEstimatorStepFailsAndTheFirstIsNamed) {
    auto r = estimator_of(ekf_log({4000, 8000, 16000, 20000}));
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("the first missing step is 12000 us"), std::string::npos) << r.second;
    EXPECT_NE(r.second.find("1 of 5 link steps have no sample"), std::string::npos) << r.second;
    r = estimator_of(ekf_log({4000, 12000, 16000, 28000}));
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("the first missing step is 8000 us"), std::string::npos) << "the earliest of two gaps: " << r.second;
    EXPECT_NE(r.second.find("3 of 7 link steps have no sample"), std::string::npos) << r.second;
    r = estimator_of(ekf_log({4000, 16000}));
    EXPECT_EQ(r.first, Verdict::fail) << "a gap of three steps";
    EXPECT_NE(r.second.find("the first missing step is 8000 us"), std::string::npos) << r.second;
    EXPECT_NE(r.second.find("2 of 4 link steps have no sample"), std::string::npos) << r.second;
}

TEST(FcRunFolder, K4_TheEstimatorFailsWhenOffTheClockBackwardsOrRepeated) {
    auto r = estimator_of(ekf_log({4000, 8000, 12001}));
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("not on the 4000 us link step"), std::string::npos) << r.second;
    r = estimator_of(ekf_log({4000, 12000, 8000}));
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("earlier than the one before"), std::string::npos) << r.second;
    r = estimator_of(ekf_log({4000, 4000, 8000}));
    EXPECT_EQ(r.first, Verdict::fail);
    EXPECT_NE(r.second.find("1 ekf2_timestamps stamps repeat"), std::string::npos) << r.second;
}

TEST(FcRunFolder, K4_TheEstimatorIsUncheckedWhenItIsNotThere) {
    EXPECT_EQ(estimator_of(ekf_log({4000, 8000}), std::nullopt).first, Verdict::unchecked) << "no link step given";
    EXPECT_EQ(estimator_of(ekf_log({4000, 8000}), 0).first, Verdict::unchecked) << "a step of 0";
    EXPECT_EQ(estimator_of(ekf_log({4000})).first, Verdict::unchecked) << "one sample has no gap";
    EXPECT_EQ(estimator_of(ux::good_messages()).first, Verdict::unchecked) << "the log holds no ekf2_timestamps";
    EXPECT_EQ(estimator_of(ekf_log({4000, 8000}, 1)).first, Verdict::unchecked) << "only instance 1 is there; instance 0 is read";
    std::vector<ux::Msg> unknown = ekf_log({4000, 8000});
    unknown.insert(unknown.begin() + ux::kSubGyro, ux::Msg{'Z', "x"});
    EXPECT_EQ(estimator_of(unknown).first, Verdict::unchecked) << "an unread log's samples are not read";
    const Report none = spade::fc::check_run_folder(spade::fc::RunFolder{good_measured()});
    EXPECT_NE(none.find("K4.estimator", Verdict::unchecked), nullptr) << none.text();
}

// A real PX4 log: SPADE_FC_ULOG names a .ulg file (for example one a live fc run left in px4-log/); unset, the case skips by name, so the 12 MB logs of
// the live runs are never committed. It checks the reader against the real thing, including the message types and sizes the spec page extract lacked.
TEST(FcRunFolder, TheRealPx4LogIsWellFormed) {
    const std::string path = ulog_env_path();
    if (path.empty()) GTEST_SKIP() << "SPADE_FC_ULOG is not set (the path of a .ulg to read)";
    std::ifstream in(path, std::ios::binary);
    ASSERT_TRUE(in.good()) << "cannot read the file SPADE_FC_ULOG names";
    const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const UlogResult r = spade::fc::read_ulog(bytes);
    std::cout << "SPADE_FC_ULOG: " << bytes.size() << " bytes: " << r.detail << "\n";
    EXPECT_EQ(r.kind, UlogResult::Kind::well_formed) << r.detail;
}

// The same real log against its run's lockstep report: SPADE_FC_ULOG and SPADE_FC_LOCKSTEP_REPORT (the run folder's lockstep-report.txt); either unset, skipped by name.
namespace {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // getenv
#endif
[[nodiscard]] std::string airframe_env_path() {
    const char* v = std::getenv("SPADE_FC_AIRFRAME");
    return v != nullptr ? std::string(v) : std::string();
}
[[nodiscard]] std::string lockstep_report_env_path() {
    const char* v = std::getenv("SPADE_FC_LOCKSTEP_REPORT");
    return v != nullptr ? std::string(v) : std::string();
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}  // namespace

TEST(FcRunFolder, TheRealPx4LogCoversTheRunsSimulatedTime) {
    const std::string log_path = ulog_env_path();
    const std::string report_path = lockstep_report_env_path();
    if (log_path.empty() || report_path.empty()) GTEST_SKIP() << "SPADE_FC_ULOG and SPADE_FC_LOCKSTEP_REPORT are not both set";
    std::ifstream lin(log_path, std::ios::binary);
    std::ifstream rin(report_path, std::ios::binary);
    ASSERT_TRUE(lin.good() && rin.good());
    const std::string log((std::istreambuf_iterator<char>(lin)), std::istreambuf_iterator<char>());
    const std::string report((std::istreambuf_iterator<char>(rin)), std::istreambuf_iterator<char>());
    const auto sim = spade::fc::last_time_usec_of(report);
    ASSERT_TRUE(sim.has_value()) << "the report holds no last_time_usec";
    const std::string airframe_path = airframe_env_path();
    std::string airframe;
    if (!airframe_path.empty()) {
        std::ifstream ain(airframe_path, std::ios::binary);
        ASSERT_TRUE(ain.good()) << "cannot read the file SPADE_FC_AIRFRAME names";
        airframe.assign((std::istreambuf_iterator<char>(ain)), std::istreambuf_iterator<char>());
    }
    const Report folder = spade::fc::check_run_folder(
        spade::fc::RunFolder{good_measured(), log, sim, airframe_path.empty() ? std::nullopt : std::optional<std::string>(airframe), 4000});
    const auto* span = span_of(folder);
    ASSERT_NE(span, nullptr);
    std::cout << "K4.span: " << span->detail << "\n";
    EXPECT_EQ(span->verdict, Verdict::pass) << span->detail;
    const auto* fw = result_of(folder, "K4.firmware");
    ASSERT_NE(fw, nullptr);
    std::cout << "K4.firmware: " << fw->detail << "\n";
    EXPECT_EQ(fw->verdict, Verdict::pass) << fw->detail;
    const auto* cad = result_of(folder, "K4.cadence");
    ASSERT_NE(cad, nullptr);
    std::cout << "K4.cadence: " << cad->detail << "\n";
    EXPECT_EQ(cad->verdict, Verdict::pass) << cad->detail;
    const auto* est = result_of(folder, "K4.estimator");
    ASSERT_NE(est, nullptr);
    std::cout << "K4.estimator: " << est->detail << "\n";
    EXPECT_EQ(est->verdict, Verdict::pass) << est->detail;
    const auto* params = result_of(folder, "K4.params");
    ASSERT_NE(params, nullptr);
    std::cout << "K4.params: " << static_cast<int>(params->verdict) << " " << params->detail << "\n";
    if (!airframe_path.empty()) {
        EXPECT_EQ(params->verdict, Verdict::pass) << params->detail;
    }
}
