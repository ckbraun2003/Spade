// The live smoke's verdict machinery and its artefacts, asserted with no window.
//
// The tour itself needs a display (scripts/live-smoke.ps1 runs it), so
// everything that DECIDES something lives in display-free headers and is
// checked here: the journal and its terminator, the declared-step ledger, the
// known-open entries that expire, the inject hooks the red runs use and
// their validation, which files a run may sweep or must refuse, the ffmpeg command, the title cards, and the PNG
// encoder. Kat's e2e smoke (KAT design-specs/dev/01-e2e-live-smoke.md, sections
// 2.1 and 4.4) is the source of the three mechanisms.
//
// Expectations are hand-written literals: journal lines spelled out, known
// CRC-32 and Adler-32 check values, and PNG bytes decoded back by a separate,
// deliberately small reader below.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "../sandbox/editor_shown_file.hpp"
#include "../sandbox/live_ledger.hpp"
#include "../sandbox/live_smoke.hpp"
#include "../sandbox/png_writer.hpp"

namespace {

using spade::sandbox::live::KnownOpen;
using spade::sandbox::live::Smoke;
using spade::sandbox::live::Verdict;

// A journal kept in memory: every line the smoke writes, in order.
struct Lines {
    std::vector<std::string> lines;
    Smoke::LineWriter writer() {
        return [this](std::string_view line) { lines.emplace_back(line); };
    }
    [[nodiscard]] bool has(std::string_view line) const {
        for (const std::string& l : lines) {
            if (l == line) return true;
        }
        return false;
    }
};

Smoke two_step_smoke(Lines& out, std::vector<KnownOpen> known = {}) {
    return Smoke({"drone_box.open", "builder.place"}, std::move(known), out.writer());
}

}  // namespace

// ===========================================================================
// The ledger and the verdict
// ===========================================================================

TEST(LiveSmokeLedger, EveryDeclaredStepRunAndNoAnomalyPasses) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    EXPECT_TRUE(smoke.check(true, "gpu-path", "GPU (OpenGL)"));
    smoke.done();
    smoke.begin("builder.place");
    EXPECT_TRUE(smoke.check(true, "objects", "3 objects"));
    smoke.done();
    const Verdict v = smoke.finish();
    EXPECT_TRUE(v.pass);
    EXPECT_EQ(v.anomalies, 0);
    EXPECT_TRUE(v.unmarked.empty());
    EXPECT_TRUE(v.expired.empty());
    ASSERT_FALSE(out.lines.empty());
    EXPECT_EQ(out.lines.back(), "END verdict=PASS");
}

// A declared step that never ran is a label claiming coverage that did not
// happen: it fails the run, and the journal names it.
TEST(LiveSmokeLedger, ADeclaredStepThatNeverRanFailsTheRun) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    smoke.done();
    const Verdict v = smoke.finish();
    EXPECT_FALSE(v.pass);
    EXPECT_EQ(v.unmarked, (std::vector<std::string>{"builder.place"}));
    EXPECT_TRUE(out.has("UNMARKED builder.place"));
    EXPECT_EQ(out.lines.back(), "END verdict=FAIL");
}

// A step that began but never reached done() is not marked either.
TEST(LiveSmokeLedger, AStepThatBeganButNeverFinishedIsUnmarked) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    smoke.done();
    smoke.begin("builder.place");
    const Verdict v = smoke.finish();
    EXPECT_FALSE(v.pass);
    EXPECT_EQ(v.unmarked, (std::vector<std::string>{"builder.place"}));
}

// Anomalies are logged and the run continues; any one of them fails it.
TEST(LiveSmokeLedger, AFailedCheckIsAnAnomalyAndTheRunContinues) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    EXPECT_FALSE(smoke.check(false, "gpu-path", "CPU raster"));
    smoke.done();
    smoke.begin("builder.place");
    (void)smoke.check(true, "objects", "3 objects");
    smoke.done();
    const Verdict v = smoke.finish();
    EXPECT_FALSE(v.pass);
    EXPECT_EQ(v.anomalies, 1);
    EXPECT_TRUE(v.unmarked.empty());
    EXPECT_TRUE(out.has("ANOMALY drone_box.open/gpu-path | CPU raster"));
    EXPECT_TRUE(out.has("DONE builder.place"));
}

TEST(LiveSmokeLedger, APassingCheckIsJournalledWithItsDetail) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    (void)smoke.check(true, "gpu-path", "GPU (OpenGL)");
    EXPECT_TRUE(out.has("CHECK drone_box.open/gpu-path | GPU (OpenGL)"));
}

// A known-open entry that fires is tolerated, and counted.
TEST(LiveSmokeLedger, AKnownOpenEntryThatFiresIsTolerated) {
    Lines out;
    Smoke smoke = two_step_smoke(out, {{"drone_box.open/gpu-path", "no GPU on the leg"}});
    smoke.begin("drone_box.open");
    (void)smoke.check(false, "gpu-path", "CPU raster");
    smoke.done();
    smoke.begin("builder.place");
    (void)smoke.check(true, "objects", "3 objects");
    smoke.done();
    const Verdict v = smoke.finish();
    EXPECT_TRUE(v.pass);
    EXPECT_EQ(v.anomalies, 0);
    EXPECT_EQ(v.known_open_fired, 1);
    EXPECT_TRUE(out.has("KNOWN-OPEN drone_box.open/gpu-path | CPU raster"));
}

// A tolerated failure is a pending verdict, not a suppression: an entry that
// stops firing fails the run, so it cannot outlive its cause.
TEST(LiveSmokeLedger, AKnownOpenEntryThatDoesNotFireExpiresAndFailsTheRun) {
    Lines out;
    Smoke smoke = two_step_smoke(out, {{"builder.place/objects", "placement misses on HiDPI"}});
    smoke.begin("drone_box.open");
    (void)smoke.check(true, "gpu-path", "GPU (OpenGL)");
    smoke.done();
    smoke.begin("builder.place");
    (void)smoke.check(true, "objects", "3");
    smoke.done();
    const Verdict v = smoke.finish();
    EXPECT_FALSE(v.pass);
    EXPECT_EQ(v.expired, (std::vector<std::string>{"builder.place/objects"}));
    EXPECT_TRUE(out.has("EXPIRED builder.place/objects | placement misses on HiDPI"));
}

// A step the ledger does not declare is an anomaly, so the ledger cannot drift
// from what the tour runs.
TEST(LiveSmokeLedger, AnUndeclaredStepIsAnAnomaly) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    (void)smoke.check(true, "gpu-path", "GPU (OpenGL)");
    smoke.done();
    smoke.begin("builder.surprise");
    (void)smoke.check(true, "x", "y");
    smoke.done();
    smoke.begin("builder.place");
    (void)smoke.check(true, "objects", "3 objects");
    smoke.done();
    const Verdict v = smoke.finish();
    EXPECT_FALSE(v.pass);
    EXPECT_EQ(v.anomalies, 1);
    EXPECT_TRUE(out.has("ANOMALY harness/undeclared-step | builder.surprise"));
}

// A step that checked nothing proves nothing, however it looked: it is an
// anomaly, raised as the step ends, before its DONE line.
TEST(LiveSmokeLedger, AStepThatCheckedNothingIsAnAnomaly) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    (void)smoke.check(true, "gpu-path", "GPU (OpenGL)");
    smoke.done();
    smoke.begin("builder.place");
    smoke.done();
    const Verdict v = smoke.finish();
    EXPECT_FALSE(v.pass);
    EXPECT_EQ(v.anomalies, 1);
    EXPECT_TRUE(v.unmarked.empty());
    EXPECT_TRUE(out.has("ANOMALY builder.place/no-checks | the step checked nothing"));
    EXPECT_TRUE(out.has("DONE builder.place"));
}

// A failed check counts as checking: the step is not also blamed for silence.
TEST(LiveSmokeLedger, AFailedCheckIsStillACheck) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    (void)smoke.check(false, "gpu-path", "CPU raster");
    smoke.done();
    EXPECT_FALSE(out.has("ANOMALY drone_box.open/no-checks | the step checked nothing"));
}

// verdict() is the same arithmetic as finish(), without writing anything, so
// the end card can show the verdict the journal will record.
TEST(LiveSmokeLedger, VerdictWritesNothing) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    smoke.done();
    const std::size_t before = out.lines.size();
    const Verdict v = smoke.verdict();
    EXPECT_FALSE(v.pass);
    EXPECT_EQ(out.lines.size(), before);
    EXPECT_FALSE(smoke.finished());
}

// ===========================================================================
// The journal: written as it happens, terminated explicitly
// ===========================================================================

// The header lists the ledger and the known-open table, so a reader of a
// partial journal still knows what the run promised.
TEST(LiveSmokeJournal, TheHeaderNamesTheLedgerAndTheKnownOpenTable) {
    Lines out;
    const Smoke smoke = two_step_smoke(out, {{"builder.place/objects", "why"}});
    EXPECT_EQ(out.lines, (std::vector<std::string>{"JOURNAL spade_sandbox live smoke v1",
                                                   "DECLARED drone_box.open", "DECLARED builder.place",
                                                   "KNOWN-OPEN-LISTED builder.place/objects | why"}));
    EXPECT_FALSE(smoke.finished());
}

// Each event is a line the moment it happens. A run that dies before finish()
// leaves a true prefix with no END line, which is what marks it unfinished.
TEST(LiveSmokeJournal, LinesAreWrittenAsEventsHappenAndOnlyFinishWritesEnd) {
    Lines out;
    Smoke smoke = two_step_smoke(out);
    smoke.begin("drone_box.open");
    EXPECT_EQ(out.lines.back(), "BEGIN drone_box.open");
    smoke.anomaly("harness/window-closed", "closed by the user");
    EXPECT_EQ(out.lines.back(), "ANOMALY harness/window-closed | closed by the user");
    (void)smoke.check(true, "gpu-path", "GPU (OpenGL)");
    smoke.done();
    EXPECT_EQ(out.lines.back(), "DONE drone_box.open");
    for (const std::string& l : out.lines) {
        EXPECT_NE(l.rfind("END ", 0), 0u) << l;
    }
    (void)smoke.finish();
    EXPECT_TRUE(smoke.finished());
    EXPECT_TRUE(out.has("RESULT anomalies=1 known_open_fired=0 unmarked=1 expired=0"));
    EXPECT_EQ(out.lines.back(), "END verdict=FAIL");
}

TEST(LiveSmokeJournal, IsTerminatedRecognisesOnlyAFinalEndLine) {
    using spade::sandbox::live::is_terminated;
    EXPECT_TRUE(is_terminated("JOURNAL x\nBEGIN a\nDONE a\nRESULT anomalies=0\nEND verdict=PASS\n"));
    EXPECT_TRUE(is_terminated("JOURNAL x\nEND verdict=FAIL"));
    EXPECT_FALSE(is_terminated("JOURNAL x\nBEGIN a\nDONE a\n"));
    EXPECT_FALSE(is_terminated("JOURNAL x\nEND verdict=PASS\nBEGIN a\n"));
    EXPECT_FALSE(is_terminated(""));
}

// ===========================================================================
// The inject hooks the red runs use
// ===========================================================================

TEST(LiveSmokeInject, TheTwoKindsParse) {
    using spade::sandbox::live::Injection;
    using spade::sandbox::live::parse_inject;
    const auto skip = parse_inject("skip:builder.place");
    ASSERT_TRUE(skip.has_value());
    EXPECT_EQ(skip->kind, Injection::Kind::skip);
    EXPECT_EQ(skip->target, "builder.place");
    const auto anomaly = parse_inject("anomaly:drone_box.orbit/injected");
    ASSERT_TRUE(anomaly.has_value());
    EXPECT_EQ(anomaly->kind, Injection::Kind::anomaly);
    EXPECT_EQ(anomaly->target, "drone_box.orbit/injected");
}

TEST(LiveSmokeInject, AnythingElseIsRefusedWithTheCause) {
    using spade::sandbox::live::parse_inject;
    // known-open is not injectable: a tolerated entry added from the command
    // line could cancel a real anomaly and turn a FAIL into a PASS.
    for (const char* bad : {"", "skip", "skip:", "bogus:x", "anomaly", ":x", "known-open:builder.place/x"}) {
        const auto r = parse_inject(bad);
        EXPECT_FALSE(r.has_value()) << bad;
        if (!r) {
            EXPECT_NE(r.error().context.find("skip:"), std::string::npos) << r.error().context;
        }
    }
}

// An injection only ever makes a run red, so one that could not is refused
// before the run starts (exit 2): a skip of a step the ledger does not
// declare, an anomaly outside a declared step, or an anomaly a known-open
// entry would tolerate.
TEST(LiveSmokeInject, TargetsAreCheckedAgainstTheLedger) {
    using spade::sandbox::live::Injection;
    using spade::sandbox::live::validate_injections;
    const std::vector<std::string> declared = {"drone_box.open", "builder.place"};
    const std::vector<KnownOpen> known = {{"builder.place/hidpi", "why"}};
    const auto ok = [&](std::vector<Injection> inj) { return validate_injections(inj, declared, known); };

    EXPECT_TRUE(ok({}).has_value());
    EXPECT_TRUE(ok({{Injection::Kind::skip, "builder.place"}}).has_value());
    EXPECT_TRUE(ok({{Injection::Kind::anomaly, "drone_box.open/injected"}}).has_value());

    const auto skip_unknown = ok({{Injection::Kind::skip, "builder.nope"}});
    ASSERT_FALSE(skip_unknown.has_value());
    EXPECT_NE(skip_unknown.error().context.find("builder.nope"), std::string::npos);
    EXPECT_NE(skip_unknown.error().context.find("drone_box.open, builder.place"), std::string::npos)
        << skip_unknown.error().context;
    // What `-Inject skip:a,anomaly:x` became under powershell -File: one string.
    EXPECT_FALSE(ok({{Injection::Kind::skip, "builder.place,anomaly:drone_box.open/x"}}).has_value());
    EXPECT_FALSE(ok({{Injection::Kind::anomaly, "nowhere/injected"}}).has_value());
    EXPECT_FALSE(ok({{Injection::Kind::anomaly, "drone_box.open"}}).has_value());
    EXPECT_FALSE(ok({{Injection::Kind::anomaly, "drone_box.open/"}}).has_value());
    const auto tolerated = ok({{Injection::Kind::anomaly, "builder.place/hidpi"}});
    ASSERT_FALSE(tolerated.has_value());
    EXPECT_NE(tolerated.error().context.find("known-open"), std::string::npos) << tolerated.error().context;
}

// ===========================================================================
// The output folder: a run deletes only the exact names it writes, and
// refuses a folder that holds anything else
// ===========================================================================

TEST(LiveSmokeArtifacts, TheRunOwnsExactlyItsOwnNames) {
    using spade::sandbox::live::kMaxStills;
    using spade::sandbox::live::owned_names;
    using spade::sandbox::live::poster_name;
    using spade::sandbox::live::still_name;
    EXPECT_EQ(poster_name(1, "drone_box"), "01_drone_box.png");
    EXPECT_EQ(poster_name(12, "builder"), "12_builder.png");
    EXPECT_EQ(still_name(7), "still_00007.png");
    const std::vector<std::string_view> mains = {"drone_box", "builder"};
    const std::vector<std::string> owned = owned_names(mains);
    ASSERT_EQ(owned.size(), 4u + kMaxStills);
    EXPECT_EQ(owned[0], "tour.mp4");
    EXPECT_EQ(owned[1], "journal.txt");
    EXPECT_EQ(owned[2], "01_drone_box.png");
    EXPECT_EQ(owned[3], "02_builder.png");
    EXPECT_EQ(owned[4], "still_00000.png");
    EXPECT_EQ(owned.back(), still_name(kMaxStills - 1));
}

TEST(LiveSmokeArtifacts, AnEmptyOrEarlierRunFolderIsSweptByExactName) {
    using spade::sandbox::live::owned_names;
    using spade::sandbox::live::plan_sweep;
    const std::vector<std::string_view> mains = {"drone_box", "builder"};
    const std::vector<std::string> owned = owned_names(mains);
    const auto empty = plan_sweep({}, owned);
    ASSERT_TRUE(empty.has_value());
    EXPECT_TRUE(empty->empty());
    const std::vector<std::string> earlier = {"journal.txt", "tour.mp4", "01_drone_box.png", "02_builder.png",
                                              "still_00000.png", "still_00001.png"};
    const auto plan = plan_sweep(earlier, owned);
    ASSERT_TRUE(plan.has_value());
    EXPECT_EQ(*plan, earlier);
}

// The lead's review: a folder of someone's own frames, or a 01_holiday.png,
// is not the run's to delete, so the whole run is refused and nothing goes.
// A directory's entry ends in '/'.
TEST(LiveSmokeArtifacts, AFolderHoldingAnythingElseIsRefusedAndNothingIsSwept) {
    using spade::sandbox::live::kMaxStills;
    using spade::sandbox::live::owned_names;
    using spade::sandbox::live::plan_sweep;
    using spade::sandbox::live::still_name;
    const std::vector<std::string_view> mains = {"drone_box", "builder"};
    const std::vector<std::string> owned = owned_names(mains);
    for (const std::string& foreign : {std::string("01_holiday.png"), std::string("frame_00001.png"),
                                       std::string("03_builder.png"), still_name(kMaxStills), std::string("notes/"),
                                       std::string("tour.mp4.bak"), std::string("Journal.txt")}) {
        const auto plan = plan_sweep({"journal.txt", "tour.mp4", foreign}, owned);
        ASSERT_FALSE(plan.has_value()) << foreign;
        EXPECT_NE(plan.error().context.find("'" + foreign + "'"), std::string::npos) << plan.error().context;
    }
}

// ===========================================================================
// The recording
// ===========================================================================

// Raw RGBA frames from glReadPixels arrive bottom row first, so ffmpeg flips
// them; yuv420p needs even dimensions, so it crops to them. The filter is
// quoted because popen() hands the line to sh, where its parentheses are syntax.
// CRF 28 keeps the recording sendable: at 20 the 129 s tour was 35.6 MB, over
// the 30 MB a file can be to reach the user.
TEST(LiveSmokeRecording, TheFfmpegCommandStreamsRawRgbaIntoAnH264File) {
    using spade::sandbox::live::ffmpeg_command;
    EXPECT_EQ(ffmpeg_command("ffmpeg", "C:/out dir/tour.mp4", 1280, 720, 30),
              "ffmpeg -hide_banner -loglevel error -y -f rawvideo -pix_fmt rgba -s 1280x720 -r 30 -i - "
              "-vf \"vflip,crop=trunc(iw/2)*2:trunc(ih/2)*2\" -c:v libx264 -preset veryfast -crf 28 "
              "-pix_fmt yuv420p \"C:/out dir/tour.mp4\"");
}

// ===========================================================================
// The title cards
// ===========================================================================

TEST(LiveSmokeCards, TheOpeningCardSaysWhatTheVideoShows) {
    using spade::sandbox::live::opening_card;
    EXPECT_EQ(opening_card("ebcc548a1b2c 2026-10-05"),
              "Spade Editor -- live smoke\nebcc548a1b2c 2026-10-05\nverdict at the end");
}

// A card's text is drawn a line at a time, centred, so a long line runs off
// both edges (the airflow card's refusal did, 2026-10-19). wrap_card_line()
// breaks at spaces, each line at most `width` characters; a word longer than
// the width stands alone rather than being cut.
TEST(LiveSmokeCards, ALongCardLineWrapsAtSpacesWithinItsWidth) {
    using spade::sandbox::live::wrap_card_line;
    EXPECT_EQ(wrap_card_line("the model 'sandbox_quad' has 4 rotors, and a live run needs their propulsion", 30),
              "the model 'sandbox_quad' has 4\nrotors, and a live run needs\ntheir propulsion");
    EXPECT_EQ(wrap_card_line("short", 30), "short");
    EXPECT_EQ(wrap_card_line("", 30), "");
    EXPECT_EQ(wrap_card_line("a supercalifragilisticexpialidocious word", 10),
              "a\nsupercalifragilisticexpialidocious\nword");
}

TEST(LiveSmokeCards, TheClosingCardCarriesTheVerdictAndItsCounts) {
    using spade::sandbox::live::closing_card;
    Verdict pass;
    pass.pass = true;
    EXPECT_EQ(closing_card("ebcc548a1b2c 2026-10-05", pass),
              "Spade Editor -- live smoke\nebcc548a1b2c 2026-10-05\n"
              "verdict PASS\nanomalies 0   known-open 0   unmarked 0   expired 0");
    Verdict fail;
    fail.anomalies = 2;
    fail.known_open_fired = 1;
    fail.unmarked = {"builder.place"};
    EXPECT_EQ(closing_card("x", fail),
              "Spade Editor -- live smoke\nx\nverdict FAIL\nanomalies 2   known-open 1   unmarked 1   expired 0");
}

// ===========================================================================
// The PNG encoder
// ===========================================================================

namespace {

uint32_t be32(const std::vector<uint8_t>& b, std::size_t at) {
    return (uint32_t{b[at]} << 24) | (uint32_t{b[at + 1]} << 16) | (uint32_t{b[at + 2]} << 8) | uint32_t{b[at + 3]};
}

// A deliberately small PNG reader for what the encoder promises and nothing
// more: one IHDR, IDAT chunks, an IEND, a zlib stream of STORED deflate blocks.
// It returns the raw scanlines (filter byte + pixels per row) and how many
// stored blocks carried them.
struct Decoded {
    uint32_t width = 0, height = 0;
    uint8_t depth = 0, colour = 0;
    std::vector<uint8_t> raw;
    int blocks = 0;
    bool ok = false;
};

Decoded decode_stored_png(const std::vector<uint8_t>& png) {
    using spade::sandbox::png::adler32;
    using spade::sandbox::png::crc32;
    Decoded d;
    static const uint8_t kSig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (png.size() < 8 || std::memcmp(png.data(), kSig, 8) != 0) return d;
    std::vector<uint8_t> zlib;
    std::size_t at = 8;
    bool ended = false;
    while (at + 12 <= png.size()) {
        const uint32_t len = be32(png, at);
        const std::string type(reinterpret_cast<const char*>(&png[at + 4]), 4);
        if (at + 12 + len > png.size()) return d;
        if (crc32(&png[at + 4], 4 + len) != be32(png, at + 8 + len)) return d;
        const uint8_t* body = &png[at + 8];
        if (type == "IHDR") {
            d.width = be32(png, at + 8);
            d.height = be32(png, at + 12);
            d.depth = body[8];
            d.colour = body[9];
        } else if (type == "IDAT") {
            zlib.insert(zlib.end(), body, body + len);
        } else if (type == "IEND") {
            ended = true;
        }
        at += 12 + len;
    }
    if (!ended || zlib.size() < 6 || zlib[0] != 0x78 || ((zlib[0] << 8) | zlib[1]) % 31 != 0) return d;
    std::size_t z = 2;
    bool final_block = false;
    while (!final_block) {
        if (z + 5 > zlib.size()) return d;
        final_block = (zlib[z] & 1u) != 0;
        if ((zlib[z] >> 1) != 0) return d;  // only stored blocks
        const uint16_t n = static_cast<uint16_t>(zlib[z + 1] | (zlib[z + 2] << 8));
        const uint16_t nn = static_cast<uint16_t>(zlib[z + 3] | (zlib[z + 4] << 8));
        if (static_cast<uint16_t>(~n) != nn || z + 5 + n > zlib.size()) return d;
        d.raw.insert(d.raw.end(), zlib.begin() + static_cast<std::ptrdiff_t>(z + 5),
                     zlib.begin() + static_cast<std::ptrdiff_t>(z + 5 + n));
        z += 5 + n;
        ++d.blocks;
    }
    if (z + 4 != zlib.size()) return d;
    const uint32_t stored_adler = (uint32_t{zlib[z]} << 24) | (uint32_t{zlib[z + 1]} << 16) |
                                  (uint32_t{zlib[z + 2]} << 8) | uint32_t{zlib[z + 3]};
    d.ok = stored_adler == adler32(d.raw.data(), d.raw.size());
    return d;
}

}  // namespace

// The standard check values: CRC-32 of "123456789" and Adler-32 of "Wikipedia".
TEST(LiveSmokePng, TheChecksumsMatchTheirStandardCheckValues) {
    using spade::sandbox::png::adler32;
    using spade::sandbox::png::crc32;
    const char* digits = "123456789";
    EXPECT_EQ(crc32(reinterpret_cast<const uint8_t*>(digits), 9), 0xCBF43926u);
    const char* wiki = "Wikipedia";
    EXPECT_EQ(adler32(reinterpret_cast<const uint8_t*>(wiki), 9), 0x11E60398u);
    const char* iend = "IEND";
    EXPECT_EQ(crc32(reinterpret_cast<const uint8_t*>(iend), 4), 0xAE426082u);
}

// A 2x2 image, top row first: the scanlines come back as written, each with
// filter type 0, in an 8-bit RGBA PNG.
TEST(LiveSmokePng, ATwoByTwoImageRoundTripsTopRowFirst) {
    const std::vector<uint8_t> rgba = {1, 2, 3, 4, 5, 6, 7, 8,  // top row
                                       9, 10, 11, 12, 13, 14, 15, 16};  // bottom row
    const std::vector<uint8_t> png = spade::sandbox::png::encode_rgba(rgba.data(), 2, 2, /*bottom_up=*/false);
    const Decoded d = decode_stored_png(png);
    ASSERT_TRUE(d.ok);
    EXPECT_EQ(d.width, 2u);
    EXPECT_EQ(d.height, 2u);
    EXPECT_EQ(d.depth, 8u);
    EXPECT_EQ(d.colour, 6u);
    EXPECT_EQ(d.raw, (std::vector<uint8_t>{0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 9, 10, 11, 12, 13, 14, 15, 16}));
}

// GL reads the bottom row first; bottom_up puts it last in the file, so the
// picture is the right way up.
TEST(LiveSmokePng, BottomUpRowsAreWrittenTopRowFirst) {
    const std::vector<uint8_t> rgba = {9, 10, 11, 12, 13, 14, 15, 16,  // GL row 0 = the bottom
                                       1, 2, 3, 4, 5, 6, 7, 8};
    const Decoded d = decode_stored_png(spade::sandbox::png::encode_rgba(rgba.data(), 2, 2, /*bottom_up=*/true));
    ASSERT_TRUE(d.ok);
    EXPECT_EQ(d.raw, (std::vector<uint8_t>{0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 9, 10, 11, 12, 13, 14, 15, 16}));
}

// A stored deflate block holds at most 65535 bytes, so a frame-sized image
// spans several, and every byte still comes back.
TEST(LiveSmokePng, AnImageLargerThanOneStoredBlockSpansSeveral) {
    const uint32_t w = 200, h = 100;  // 100 rows of 1 + 800 bytes = 80,100 raw bytes
    std::vector<uint8_t> rgba(static_cast<std::size_t>(w) * h * 4u);
    for (std::size_t i = 0; i < rgba.size(); ++i) {
        rgba[i] = static_cast<uint8_t>(i * 7u);
    }
    const Decoded d = decode_stored_png(spade::sandbox::png::encode_rgba(rgba.data(), w, h, false));
    ASSERT_TRUE(d.ok);
    EXPECT_EQ(d.blocks, 2);
    ASSERT_EQ(d.raw.size(), 80'100u);
    EXPECT_EQ(d.raw[0], 0u);                // row 0's filter byte
    EXPECT_EQ(d.raw[1], rgba[0]);           // its first pixel byte
    EXPECT_EQ(d.raw[801], 0u);              // row 1's filter byte
    EXPECT_EQ(d.raw[802], rgba[800]);       // row 1 starts 800 bytes into the image
    EXPECT_EQ(d.raw.back(), rgba.back());
}

// ===========================================================================
// The tour's ledger itself (sandbox/live_ledger.hpp, a header so that this can read it): the steps and stills the tour declares, in order. A
// tour that counts steps or stills re-counts when this changes (docs/design/consumers.md).
// ===========================================================================
namespace {
std::vector<std::string> ledger_steps() {
    std::vector<std::string> out;
    for (const auto& st : spade::sandbox::live::kSteps) out.push_back(std::string(st.main) + "." + std::string(st.sub));
    return out;
}
}  // namespace

TEST(LiveSmokeLedger, TheTourDeclaresItsStepsAndStillsInOrder) {
    const std::vector<std::string> steps = ledger_steps();
    EXPECT_EQ(steps.size(), 25u);  // 24 + editor.x500_feet (video #2)
    EXPECT_EQ(spade::sandbox::live::kSnapshots.size(), 22u);  // 21 + editor_x500_feet
    const auto at = [&](std::string_view name) {
        return std::find(steps.begin(), steps.end(), std::string(name)) - steps.begin();
    };
    EXPECT_LT(at("editor.add_drone"), at("editor.x500_feet"));
    EXPECT_LT(at("editor.x500_feet"), at("editor.airflow"));
    EXPECT_NE(std::find(spade::sandbox::live::kSnapshots.begin(), spade::sandbox::live::kSnapshots.end(), "editor_x500_feet"),
              spade::sandbox::live::kSnapshots.end());
    EXPECT_LT(at("editor.airflow"), at("editor.add_objects"));
    // a skip injection of a declared step is accepted (it makes the run red); of one the ledger lacks, refused
    using spade::sandbox::live::Injection;
    using spade::sandbox::live::validate_injections;
    const std::vector<Injection> skip_airflow = {{Injection::Kind::skip, "editor.airflow"}};
    const std::vector<Injection> skip_x500 = {{Injection::Kind::skip, "editor.x500_feet"}};
    const std::vector<Injection> anomaly_x500 = {{Injection::Kind::anomaly, "editor.x500_feet/injected"}};
    const std::vector<KnownOpen> none;
    EXPECT_TRUE(validate_injections(skip_airflow, steps, none).has_value());
    EXPECT_TRUE(validate_injections(skip_x500, steps, none).has_value()) << "a skip of the new step makes the run red";
    EXPECT_TRUE(validate_injections(anomaly_x500, steps, none).has_value()) << "an anomaly in the new step makes the run red";
}

// ===========================================================================
// --only: a short run of chosen steps (a feature film runs one step, not the whole tour)
// ===========================================================================
TEST(LiveSmokeOnly, TheListIsParsedAndCheckedAgainstTheLedger) {
    using spade::sandbox::live::parse_only;
    using spade::sandbox::live::validate_only;
    const auto one = parse_only("editor.x500_feet");
    ASSERT_TRUE(one.has_value());
    ASSERT_EQ(one->size(), 1u);
    EXPECT_EQ((*one)[0], "editor.x500_feet");
    const auto two = parse_only("editor.x500_feet,drone_box.attitude");
    ASSERT_TRUE(two.has_value());
    EXPECT_EQ(two->size(), 2u);
    EXPECT_FALSE(parse_only("").has_value());
    EXPECT_FALSE(parse_only("editor.x500_feet,").has_value()) << "a trailing comma names nothing";
    EXPECT_FALSE(parse_only("editor").has_value()) << "a step is main.sub";

    const std::vector<std::string> declared = ledger_steps();
    EXPECT_TRUE(validate_only(*one, declared).has_value());
    const std::vector<std::string> nope = {"editor.nope"};
    const auto bad = validate_only(nope, declared);
    ASSERT_FALSE(bad.has_value());
    EXPECT_NE(bad.error().context.find("editor.nope"), std::string::npos) << bad.error().context;
    EXPECT_NE(bad.error().context.find("editor.x500_feet"), std::string::npos) << "it lists the steps there are: " << bad.error().context;
}

TEST(LiveSmokeOnly, TheStepsToRunAreTheChosenOnesAndWhatTheyNeed) {
    using spade::sandbox::live::steps_to_run;
    const std::vector<std::string> declared = ledger_steps();
    EXPECT_EQ(steps_to_run({}, declared), declared) << "no --only: the whole tour";
    const std::vector<std::string> x = {"editor.x500_feet"};
    const std::vector<std::string> want_x = {"editor.open", "editor.x500_feet"};
    EXPECT_EQ(steps_to_run(x, declared), want_x) << "an editor step needs the editor open, and runs in ledger order";
    const std::vector<std::string> d = {"drone_box.attitude"};
    const std::vector<std::string> want_d = {"drone_box.attitude"};
    EXPECT_EQ(steps_to_run(d, declared), want_d) << "a step of another chapter needs nothing added";
    const std::vector<std::string> both = {"editor.x500_feet", "drone_box.attitude"};
    const std::vector<std::string> want_both = {"drone_box.attitude", "editor.open", "editor.x500_feet"};
    EXPECT_EQ(steps_to_run(both, declared), want_both) << "ledger order, not the list's";
}

// What a run owes at its end under --only: the verification run of editor.x500_feet failed 25 whole-tour checks (every poster and still of the steps
// it had not run).
TEST(LiveSmokeOnly, AShortRunOwesOnlyTheFilesOfItsOwnChapters) {
    using spade::sandbox::live::missing_still_is_anomaly;
    using spade::sandbox::live::poster_expected;
    using spade::sandbox::live::steps_to_run;
    const std::vector<std::string> declared = ledger_steps();
    const std::vector<std::string> only = {"editor.x500_feet"};
    const std::vector<std::string> ran = steps_to_run(only, declared);  // editor.open, editor.x500_feet
    EXPECT_TRUE(poster_expected(ran, "editor")) << "editor.open writes the editor's poster";
    EXPECT_FALSE(poster_expected(ran, "drone_box"));
    EXPECT_FALSE(poster_expected(ran, "airflow"));
    EXPECT_FALSE(poster_expected(ran, "bodies"));
    EXPECT_FALSE(poster_expected(ran, "batch"));
    for (const char* main : {"drone_box", "airflow", "editor", "bodies", "batch"}) {
        EXPECT_TRUE(poster_expected(declared, main)) << "the whole tour owes every poster: " << main;
    }
    EXPECT_TRUE(missing_still_is_anomaly(std::vector<std::string>{})) << "the whole tour still fails a still that was not written";
    EXPECT_FALSE(missing_still_is_anomaly(only)) << "a short run writes only its own stills";
}

// A path on this machine must never be on screen in a still or a film: the first verification run of editor.x500_feet showed one in the Load
// from file field.
TEST(LiveSmokePaths, TextThatNamesAPathOnThisMachineIsRecognised) {
    using spade::sandbox::live::shows_a_path;
    EXPECT_TRUE(shows_a_path("C:\\Users\\Someone\\Desktop\\Projects\\spade-wt\\interface\\build-smoke\\editor_work\\x500_contact.model.yaml"));
    EXPECT_TRUE(shows_a_path("c:/Users/Someone/work/x.yaml"));
    EXPECT_TRUE(shows_a_path("D:\\builds\\run1\\x.yaml")) << "any drive path";
    EXPECT_TRUE(shows_a_path("saved 'x' to /Users/someone/lib/x.yaml (undo removes it)"));
    EXPECT_TRUE(shows_a_path("loaded from /home/someone/x.yaml"));
    EXPECT_TRUE(shows_a_path("User Guide: the guide is missing (C:\\Program Files\\Spade\\guide.html)"));
}

TEST(LiveSmokePaths, OrdinaryOnScreenTextIsNotTakenForAPath) {
    using spade::sandbox::live::shows_a_path;
    EXPECT_FALSE(shows_a_path(""));
    EXPECT_FALSE(shows_a_path("x500_contact, armed"));
    EXPECT_FALSE(shows_a_path("hover.scene.yaml *"));
    EXPECT_FALSE(shows_a_path("Flying x500_contact, disarmed"));
    EXPECT_FALSE(shows_a_path("tick 420  1005 frames  at 15:30")) << "a colon in a clock is not a drive";
    EXPECT_FALSE(shows_a_path("Position (m) 0.000 0.315 -1.500"));
    EXPECT_FALSE(shows_a_path("library/drones/phantom.yaml")) << "a relative name inside the library is not a path on this machine";
}

// The general check: what the tour's steps leave on screen is read at every step's end and before every still. These are the texts the editor draws
// after the tour's add_objects, save and part steps as they stand today (gate 50's code), with a made-up user folder.
TEST(LiveSmokePaths, TheWatchNamesTheStepAndTheFieldOfEveryPathAndNeverTheText) {
    using spade::sandbox::live::PathWatch;
    using spade::sandbox::live::ScreenText;
    PathWatch watch;
    const std::vector<ScreenText> after_add_objects = {
        {"item_path", "C:\\Users\\Someone\\Desktop\\Projects\\spade-wt\\interface\\build-smoke\\editor_work\\crate.gltf"},
        {"status", "armed 'crate'"},
        {"title", "hover.scene.yaml *"},
    };
    const std::vector<ScreenText> after_save = {
        {"item_path", ""},
        {"status", "saved 'hover_copy' to C:/Users/Someone/work/library/drones/hover_copy.yaml"},
        {"title", "hover_copy.scene.yaml"},
    };
    const std::vector<ScreenText> after_part = {
        {"selected_source", "C:\\Users\\Someone\\work\\crate.gltf"},
        {"status", ""},
    };
    const std::vector<ScreenText> clean = {{"item_path", ""}, {"status", "x500_contact, armed"}, {"title", "hover.scene.yaml *"}};
    watch.read("editor.add_objects", after_add_objects);
    watch.read("editor.save", after_save);
    watch.read("editor.part", after_part);
    watch.read("editor.undo", clean);
    EXPECT_EQ(watch.reads(), 4u);
    EXPECT_EQ(watch.texts_read(), 11u) << "every text of every read is counted, the clean ones too";
    ASSERT_EQ(watch.offences().size(), 3u);
    EXPECT_EQ(watch.offences().at(0), "editor.add_objects: item_path");
    EXPECT_EQ(watch.offences().at(1), "editor.save: status");
    EXPECT_EQ(watch.offences().at(2), "editor.part: selected_source");
    for (const std::string& line : watch.offences()) {
        EXPECT_EQ(line.find("Someone"), std::string::npos) << "an offence never repeats the path: " << line;
    }
}

TEST(LiveSmokePaths, ACleanScreenLeavesNoOffence) {
    using spade::sandbox::live::PathWatch;
    using spade::sandbox::live::ScreenText;
    PathWatch watch;
    const std::vector<ScreenText> clean = {{"item_path", ""}, {"status", "saved 'hover_copy' to library/drones/hover_copy.yaml"}, {"title", "hover.scene.yaml *"}};
    watch.read("editor.save", clean);
    watch.read("still editor_save", clean);
    EXPECT_EQ(watch.reads(), 2u);
    EXPECT_EQ(watch.texts_read(), 6u);
    EXPECT_TRUE(watch.offences().empty());
}

// What the save status shows of the saved file (the lead's rule): inside the library folder, relative to it; anywhere else, the file name alone; never
// a ..\ chain.
TEST(LiveSmokePaths, ASavedFileInsideTheLibraryIsShownRelativeToIt) {
    using spade::sandbox::editor::shown_file;
    const std::filesystem::path lib = std::filesystem::path("C:/Users/Someone/work/assets/library");
    EXPECT_EQ(shown_file(lib / "drones" / "hover_copy.yaml", lib), "drones/hover_copy.yaml");
    EXPECT_EQ(shown_file(lib / "library.yaml", lib), "library.yaml");
    EXPECT_EQ(shown_file(lib / "drones" / ".." / "drones" / "x.yaml", lib), "drones/x.yaml") << "an inside file written with a .. in it is still inside";
}

TEST(LiveSmokePaths, ASavedFileOutsideTheLibraryIsShownByItsNameAlone) {
    using spade::sandbox::editor::shown_file;
    const std::filesystem::path lib = std::filesystem::path("C:/Users/Someone/work/assets/library");
    for (const char* outside : {"C:/Users/Someone/Desktop/hover_copy.yaml", "C:/Users/Someone/work/assets/library_old/x.yaml",
                                "D:/elsewhere/hover_copy.yaml", "C:/Users/Someone/work/assets/hover_copy.yaml"}) {
        const std::string shown = shown_file(std::filesystem::path(outside), lib);
        EXPECT_EQ(shown, std::filesystem::path(outside).filename().generic_string()) << outside;
        EXPECT_EQ(shown.find(".."), std::string::npos) << "never a .. chain: " << shown;
        EXPECT_EQ(shown.find("Someone"), std::string::npos) << "never the user's folders: " << shown;
    }
}
