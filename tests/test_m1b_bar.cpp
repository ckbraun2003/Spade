#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "core/rng.hpp"
#include "sensors/imu.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"

// ---------------------------------------------------------------------------
// Task 20 -- the M1B charter bar, as executable asserts.
//
// The brief's six bullets, and what "executable" means for each one here:
//
//   1. FIXED-STEP (no wall-clock symbol in the engine). A SOURCE-LEVEL SCAN,
//      not a link-level dumpbin audit -- see "THE CANARY MECHANISM, AND ITS
//      HONESTY TRADEOFF" below for why that choice was made and what it does
//      and does not prove.
//   2. HEADLESS (Simulation runs with SPADE_BUILD_V1=OFF). The SAME scanning
//      mechanism, pointed at a different forbidden pattern (a v1 #include),
//      PLUS an actual end-to-end vehicle+sensor run in this very process --
//      see bullet 5's test, which is exactly that run, and which succeeds in
//      a process whose link closure never pulled in `Spade` (tests/
//      CMakeLists.txt's target_link_libraries for spade_tests names
//      spade::core/world/state/physics/vehicles/sim and GTest, never
//      `Spade` -- a fact this test's own OWN scan re-verifies against the
//      CURRENT source tree rather than trusting the CMakeLists.txt prose).
//   3. SEEDED DETERMINISM (corpus green). A real determinism-replay run
//      (spade::testing -- the SAME mechanism test_determinism.cpp's corpus
//      uses) at SMOKE SCALE: one small scenario, not the four-scenario
//      corpus (which already gates in test_determinism.cpp -- rerunning it
//      here would test nothing this suite does not already test, at ten
//      times the cost).
//   4. SNAPSHOT (round-trip green). The SAME replay-guarantee shape
//      (snapshot mid-run, restore into a FRESH Simulation, resume, compare
//      against the uninterrupted digest) test_determinism.cpp's corpus-wide
//      test performs, at smoke scale and on its own small scenario.
//   5. 6-DOF + QUADROTOR + IMU (suites green). A short, real quadrotor
//      flight: spawn in trim, hold hover, step, poll the IMU, check both the
//      body's 6-DOF state and the sensor's output are physically sane --
//      NOT a rerun of test_quadrotor.cpp's or test_imu.cpp's own exhaustive
//      suites (which already gate), just enough of the same mechanism to
//      prove the three systems compose in one process.
//   6. THE SUMMARY TABLE. Prints PASS/FAIL for all six rows by calling the
//      SAME helper functions the five tests above call -- so the printed
//      table cannot silently drift from what was actually checked.
//
// ---------------------------------------------------------------------------
// THE CANARY MECHANISM, AND ITS HONESTY TRADEOFF (bullets 1 and 2)
//
// The brief's own words: "if a true link-level canary is not achievable
// cleanly with MSVC+Ninja, implement the strongest honest alternative (e.g.
// a dumpbin symbol audit wired as a test, or an include-level static check)
// and report the tradeoff." This file takes the SECOND option -- a
// SOURCE-LEVEL scan of every .hpp/.cpp under spade/engine/ (skipping
// tools/, the viewer's documented wall-clock exemption) for a short list of
// forbidden substrings -- and the reasoning is worth stating rather than
// just asserting:
//
//   * PORTABILITY. spade/CMakeLists.txt's own comment records that Task 4's
//     Linux CI configures SPADE_BUILD_V1=OFF specifically to build/test the
//     v2 engine tree on its own -- i.e. this suite is expected to run on a
//     platform where dumpbin.exe does not exist at all. A dumpbin-based
//     canary would SKIP unconditionally there, providing zero coverage on
//     the one CI job the "headless" bullet is actually about. A source scan
//     runs identically everywhere ctest runs.
//   * WHAT IT DOES NOT PROVE, stated rather than hidden. This is a TEXT
//     search, not a linker's closure: it cannot see a wall-clock call
//     reached only through an indirect function pointer whose target is
//     never named in the scanned text, and it cannot see one introduced by
//     a vendored dependency's own translation unit. A true link-level
//     symbol audit (e.g. dumpbin /symbols on each engine .lib, restricted to
//     the .lib files themselves and NOT the spade_tests executable --
//     GTest's own runtime legitimately reads a wall clock for test timing,
//     which would poison a whole-executable scan) would close that gap on
//     this box; it was not added here because a Windows-only bonus check
//     that provides no coverage on the platform the "headless" bullet cares
//     about was judged not worth the added complexity for THIS task. If a
//     future task wants the stronger guarantee, the .lib targets to point
//     it at are spade_core/spade_state/spade_world/spade_physics/
//     spade_sim/spade_vehicles (engine/CMakeLists.txt).
//   * WHAT IT DOES PROVE. Every wall-clock API this codebase's own
//     documentation names (world/medium.hpp, core/rng.hpp, this task's own
//     context) as the thing to avoid, and every literal v1 #include, in
//     every file this build actually compiles into the engine and its
//     tests -- checked baseline-clean (grep, by hand) before this list was
//     written, so the patterns are calibrated against a KNOWN-good tree
//     rather than guessed.
// ---------------------------------------------------------------------------

namespace {

namespace fs = std::filesystem;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}

#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// ---------------------------------------------------------------------------
// Source scanning -- the shared mechanism behind bullets 1 and 2.
// ---------------------------------------------------------------------------

// Every .hpp/.cpp under `root`, at any depth, EXCLUDING any path whose
// component list contains one of `skip_dirs` -- e.g. "tools", the viewer's
// documented wall-clock and v1 exemption (global-constraints.md; this
// task's own context).
//
// S5 final-review fix wave (I3a): the exclusion check walks the path
// RELATIVE TO `root`, not `p`'s own absolute path. Walking the absolute path
// meant a checkout sitting under any directory literally named "tools" (or
// any other skip_dirs entry) matched on that PREFIX component and silently
// excluded the entire scan -- zero files, not just a real tools/ subtree
// under root. This mirrors the discipline tests/CMakeLists.txt:69-80 already
// applies to SPADE_ENGINE_DIR itself (cmake_path(NORMALIZE) there so the
// baked-in string never carries a spurious "tests" component); the analogous
// fix here is to compare only the portion of the path this scan actually
// controls -- everything under `root`.
[[nodiscard]] std::vector<fs::path> collect_sources(const fs::path& root,
                                                     std::initializer_list<std::string_view> skip_dirs) {
    std::vector<fs::path> out;
    if (!fs::exists(root)) return out;
    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(root)) {
        if (!entry.is_regular_file()) continue;
        const fs::path& p = entry.path();
        const std::string ext = p.extension().string();
        if (ext != ".hpp" && ext != ".cpp") continue;

        const fs::path relative = fs::relative(p, root);
        bool skip = false;
        for (const fs::path& part : relative) {
            for (const std::string_view skip_name : skip_dirs) {
                if (part.string() == skip_name) {
                    skip = true;
                    break;
                }
            }
            if (skip) break;
        }
        if (!skip) out.push_back(p);
    }
    std::sort(out.begin(), out.end());  // deterministic report order
    return out;
}

// Every line of `file`, 1-based line numbers implied by index + 1.
[[nodiscard]] std::vector<std::string> read_lines(const fs::path& file) {
    std::ifstream in(file);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) lines.push_back(std::move(line));
    return lines;
}

// Returns `text` with every `//` line comment and `/* ... */` block comment
// replaced by whitespace of the same shape -- every newline is preserved, so
// splitting the result back into lines keeps the ORIGINAL line numbers, and
// code sharing a line with a trailing comment survives untouched (S5 T9
// ticket C: makes the libm canary below comment-aware, so a doc comment
// naming `std::sin` is not mistaken for a call to it).
//
// Tracks double- and single-quoted literals (with backslash-escape handling)
// so a string constant containing "//" or "/*" is not mistaken for a comment
// either. NOT a full lexer -- raw string literals (R"(...)") are not
// special-cased -- which is a fine tradeoff for scanning this codebase's own
// engine source (it has none) rather than arbitrary C++.
[[nodiscard]] std::string strip_comments(const std::string& text) {
    enum class State { code, line_comment, block_comment, string_lit, char_lit };
    std::string out;
    out.reserve(text.size());
    State state = State::code;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        const char next = (i + 1 < text.size()) ? text[i + 1] : '\0';
        switch (state) {
            case State::code:
                if (c == '/' && next == '/') {
                    state = State::line_comment;
                    out.append("  ");
                    ++i;
                } else if (c == '/' && next == '*') {
                    state = State::block_comment;
                    out.append("  ");
                    ++i;
                } else if (c == '"') {
                    state = State::string_lit;
                    out.push_back(c);
                } else if (c == '\'') {
                    state = State::char_lit;
                    out.push_back(c);
                } else {
                    out.push_back(c);
                }
                break;
            case State::line_comment:
                if (c == '\n') {
                    state = State::code;
                    out.push_back(c);
                } else {
                    out.push_back(' ');
                }
                break;
            case State::block_comment:
                if (c == '*' && next == '/') {
                    state = State::code;
                    out.append("  ");
                    ++i;
                } else if (c == '\n') {
                    out.push_back(c);  // preserve line breaks inside the comment
                } else {
                    out.push_back(' ');
                }
                break;
            case State::string_lit:
            case State::char_lit:
                out.push_back(c);
                if (c == '\\' && next != '\0') {
                    out.push_back(next);
                    ++i;
                } else if ((state == State::string_lit && c == '"') ||
                           (state == State::char_lit && c == '\'')) {
                    state = State::code;
                }
                break;
        }
    }
    return out;
}

// Same shape as read_lines(), but with every comment blanked out first.
[[nodiscard]] std::vector<std::string> read_lines_no_comments(const fs::path& file) {
    std::ifstream in(file);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    const std::string stripped = strip_comments(buffer.str());
    std::vector<std::string> lines;
    std::string line;
    std::istringstream stream(stripped);
    while (std::getline(stream, line)) lines.push_back(std::move(line));
    return lines;
}

// True iff `pattern` occurs in `line`, subject to `require_non_identifier_before`:
// when set, a candidate occurrence is accepted only if the character
// immediately before it is absent or is not a C++ identifier character
// (alnum or `_`). This is what keeps a bare, unqualified pattern like
// "sinf(" from matching inside an unrelated longer identifier such as
// "asinf(" -- `asinf` is a different function (not one of the patterns this
// scan is asked to forbid), and a plain substring search cannot otherwise
// tell "asinf(" apart from "a" followed by a genuine call to "sinf(". A
// pattern that already carries its own qualifier (e.g. "std::sin(") is
// naturally boundary-safe in the overwhelmingly common case, but the check
// is applied uniformly rather than only to the bare C spellings -- it can
// only REJECT a match that would otherwise be a false positive, never
// suppress a real one, so there is no cost to applying it everywhere a
// caller opts in.
[[nodiscard]] bool line_matches(const std::string& line, std::string_view pattern,
                                 bool require_non_identifier_before) {
    std::size_t pos = 0;
    while ((pos = line.find(pattern, pos)) != std::string::npos) {
        const bool boundary_ok = !require_non_identifier_before || pos == 0 ||
                                  !(std::isalnum(static_cast<unsigned char>(line[pos - 1])) || line[pos - 1] == '_');
        if (boundary_ok) return true;
        ++pos;
    }
    return false;
}

// Scans `files` for any of `forbidden` (plain substring match -- these are
// C++/Win32 identifiers, not regexes, so a substring match is exact and
// cannot under-match). Returns every "path:line: matched substring" hit, so
// a failure names exactly what to go fix rather than just "something is
// wrong somewhere".
//
// `strip_comments_first` (S5 T9 ticket C): when true, reads each file through
// read_lines_no_comments() instead of read_lines(), so a forbidden substring
// appearing only in a `//` or `/* */` comment is not a hit. Defaults to
// false, unchanged from every existing caller's behavior (bullets 1 and 2
// below scan for wall-clock APIs and v1 #includes respectively, and neither
// has ever needed comment-awareness).
//
// `require_non_identifier_before` (S5 final-review fix wave, I3b): see
// line_matches() above. Defaults to false, unchanged from every existing
// caller's behavior -- only the widened libm scan opts in.
[[nodiscard]] std::vector<std::string> scan_for_forbidden(const std::vector<fs::path>& files,
                                                           std::initializer_list<std::string_view> forbidden,
                                                           bool strip_comments_first = false,
                                                           bool require_non_identifier_before = false) {
    std::vector<std::string> hits;
    for (const fs::path& file : files) {
        const std::vector<std::string> lines = strip_comments_first ? read_lines_no_comments(file) : read_lines(file);
        for (std::size_t i = 0; i < lines.size(); ++i) {
            for (const std::string_view pattern : forbidden) {
                if (line_matches(lines[i], pattern, require_non_identifier_before)) {
                    std::ostringstream hit;
                    hit << file.string() << ":" << (i + 1) << ": matched \"" << pattern << "\"";
                    hits.push_back(hit.str());
                }
            }
        }
    }
    return hits;
}

// True iff `line` is a GENUINE #include DIRECTIVE naming something under the
// v1 `Spade/` tree -- i.e. the line, after trimming leading whitespace,
// syntactically STARTS WITH "#include". A plain substring search for
// "#include <Spade" would also match that very string appearing as DATA --
// which is exactly what happened the first time this ran: it flagged this
// file's OWN pattern-list literal (scan_for_forbidden's caller, below,
// necessarily writes the text "#include <Spade" somewhere so there is
// something to search FOR). Requiring the match to be the line's own leading
// token is what tells a real preprocessor directive apart from a string
// holding one, and it is the more precise check on its own merits, not just
// a workaround.
[[nodiscard]] bool is_v1_include_directive(const std::string& line) {
    std::size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
    const std::string_view rest(line.data() + i, line.size() - i);
    if (rest.substr(0, 8) != "#include") return false;
    return rest.find("<Spade") != std::string_view::npos || rest.find("\"Spade") != std::string_view::npos;
}

[[nodiscard]] std::vector<std::string> scan_for_v1_includes(const std::vector<fs::path>& files) {
    std::vector<std::string> hits;
    for (const fs::path& file : files) {
        const std::vector<std::string> lines = read_lines(file);
        for (std::size_t i = 0; i < lines.size(); ++i) {
            if (is_v1_include_directive(lines[i])) {
                std::ostringstream hit;
                hit << file.string() << ":" << (i + 1) << ": " << lines[i];
                hits.push_back(hit.str());
            }
        }
    }
    return hits;
}

// SPADE_ENGINE_DIR / SPADE_TESTS_DIR: absolute paths baked in at configure
// time (tests/CMakeLists.txt), the same discipline SPADE_GOLDEN_DIR already
// uses -- see that compile definition's own comment for why (the kat
// testing spec's no-CWD rule: a data location a test MUST find cannot be
// CWD-relative).

// Bullet 1 -- FIXED-STEP: no wall-clock symbol anywhere under spade/engine/
// except tools/ (the viewer's documented pacing exemption).
[[nodiscard]] bool no_wallclock_symbols_in_engine_source(std::string& detail) {
    const std::vector<fs::path> files = collect_sources(fs::path(SPADE_ENGINE_DIR), {"tools"});
    // S5 final-review fix wave (I3a): a scan of ZERO files is not a pass --
    // it means the source tree was not found (e.g. SPADE_ENGINE_DIR pointed
    // nowhere real), and "0 hits in 0 files" would otherwise report the same
    // green PASS as a genuine clean scan of the whole engine tree. Equivalent
    // to ASSERT_FALSE(files.empty()), expressed as an early failing return
    // because this helper returns bool rather than void.
    if (files.empty()) {
        detail = "0 source files found under engine/ (tools/ excluded) -- the scan root is empty or "
                 "missing, which would otherwise vacuously PASS; treating it as a failure instead";
        return false;
    }
    const std::vector<std::string> hits = scan_for_forbidden(
        files, {"chrono", "QueryPerformanceCounter", "QueryPerformanceFrequency", "GetSystemTime",
                "GetLocalTime", "GetTickCount", "timeGetTime", "std::time(", "::time(", "_ftime",
                "glfwGetTime"});
    if (hits.empty()) {
        std::ostringstream ok;
        ok << "0 wall-clock references in " << files.size() << " source files under engine/ (tools/ excluded)";
        detail = ok.str();
        return true;
    }
    std::ostringstream bad;
    bad << hits.size() << " wall-clock reference(s): ";
    for (std::size_t i = 0; i < hits.size() && i < 5; ++i) bad << (i == 0 ? "" : "; ") << hits[i];
    detail = bad.str();
    return false;
}

// Bullet 2's source-level half -- HEADLESS: no engine or test source file
// includes v1 (Spade/Spade.hpp or anything else under the `Spade/` tree)
// outside tools/. See this file's header comment for the functional half.
[[nodiscard]] bool no_v1_include_in_engine_or_test_source(std::string& detail) {
    std::vector<fs::path> files = collect_sources(fs::path(SPADE_ENGINE_DIR), {"tools"});
    const std::vector<fs::path> test_files = collect_sources(fs::path(SPADE_TESTS_DIR), {});
    files.insert(files.end(), test_files.begin(), test_files.end());

    // S5 final-review fix wave (I3a): same vacuous-pass hazard as
    // no_wallclock_symbols_in_engine_source() above -- see that function's
    // comment. Equivalent to ASSERT_FALSE(files.empty()).
    if (files.empty()) {
        detail = "0 source files found under engine/ (tools/ excluded) + tests/ -- the scan roots are "
                 "empty or missing, which would otherwise vacuously PASS; treating it as a failure instead";
        return false;
    }

    const std::vector<std::string> hits = scan_for_v1_includes(files);
    if (hits.empty()) {
        std::ostringstream ok;
        ok << "0 v1 (Spade/*) includes in " << files.size()
           << " source files under engine/ (tools/ excluded) + tests/";
        detail = ok.str();
        return true;
    }
    std::ostringstream bad;
    bad << hits.size() << " v1 include(s): ";
    for (std::size_t i = 0; i < hits.size() && i < 5; ++i) bad << (i == 0 ? "" : "; ") << hits[i];
    detail = bad.str();
    return false;
}

// ---------------------------------------------------------------------------
// S5 T9 ticket C -- NO LIBM TRANSCENDENTAL anywhere under spade/engine/
// (tools/ excluded, the same render-only exemption bullet 1 above uses --
// see this file's header comment and Ticket A's note in
// tools/viewer/scenes.cpp: viewer code is not digest-feeding).
// global-constraints.md's bit-portability rule names the forbidden calls
// directly: "no libm transcendental (std::sin/cos/exp/log/pow) may execute
// on any path that feeds registered state or the digest -- IEEE mandates
// correct rounding only for + - x / sqrt. In-engine fp32_math kernels only."
// This is that rule, as an executable canary, mirroring bullet 1's shape.
//
// COMMENT-AWARE, UNLIKE BULLETS 1/2 ABOVE, DELIBERATELY (this is the whole
// point of strip_comments()/read_lines_no_comments() above): a doc comment
// that NAMES std::sin -- exactly what core/fp32_math.hpp's own header
// comment does, to explain why its kernels exist instead -- is not a CALL to
// std::sin, and a plain substring scan cannot tell the difference. Without
// comment-awareness, documentation would have to spell the forbidden names
// with a contorted gap or split ("st d::sin") to dodge the scan, which is
// worse than the scan being slightly more permissive: a reader unaware of
// the dodge has no way to guess it is there.
//
// PATTERN BREADTH (S5 final-review fix wave, I3b). The original five
// (std::sin/cos/exp/log/pow) covered only the qualified C++ spellings --
// blind to the f-suffixed C spellings (sinf/cosf/.../tanf), to std::tan and
// its inverse-trig siblings, and, critically, to glm's transcendental
// wrappers (glm::angleAxis, glm::rotate, glm::eulerAngles, glm::slerp,
// glm::quatLookAt) -- each of those calls libm internally and is exactly the
// kind of cross-libm hazard this program's Task 1 vindicated the sqrt/sin/cos
// spelling against; before this widening, code that reached the same
// platform divergence through glm::angleAxis instead of std::sin would pass
// this canary green. Every added pattern is boundary-anchored
// (require_non_identifier_before=true, see line_matches() above) so a bare
// spelling like "sinf(" cannot fire on an unrelated identifier such as
// "asinf(" -- verified against this tree with a temporary mutation (this
// task's report carries the transcript).
//
// SCOPE WIDENED A SECOND TIME (S7a Task R5, controller ruling SR-14): the
// scan root used to be SPADE_ENGINE_DIR alone, and that left a real gap --
// this program hit the libm-portability trap a THIRD time when Task R3
// removed std::tan from the engine correctly, then computed a COMMITTED
// GOLDEN's camera quaternion with glm::angleAxis in test_render_raster.cpp
// (a test file, never scanned here), whose sin/cos go through libm exactly
// as the pattern list above already knows to forbid -- the pattern was
// already on the list; the ROOT SET just never reached the file that used
// it. Two conforming libms disagree on such a call by one ulp; the committed
// hash would move between platforms; and the failure would present as a
// rasterizer regression, not as what it actually is. The fix that landed at
// Task R3 (test_render_raster.cpp's camera_top_down()/box_spin literal-
// float-component quaternions, replacing glm::angleAxis) is a real,
// already-fixed instance of exactly the class of bug this widened scan now
// catches for good.
//
// MECHANISM: a test source file joins the scan iff its own text (comment-
// stripped, exactly like the pattern matching itself) contains the
// identifier SPADE_GOLDEN_DIR -- collect_golden_feeding_test_sources()
// below. That one macro is ALREADY the established idiom every golden-
// consuming test in this codebase uses to locate its own committed corpus
// (tests/CMakeLists.txt's own comment: "the same discipline SPADE_GOLDEN_DIR
// already uses" -- test_render_raster.cpp's frame manifest,
// test_render_tessellate.cpp's tessellation manifest, test_world_file.cpp's
// fixtures, test_determinism.cpp/test_gpu_*.cpp's scenario corpora, and this
// task's own test_render_csg.cpp all reach their corpus this way), so
// requiring a genuine reference to it is neither a hand-maintained list (a
// SECOND list to remember to update is exactly the propagation-discipline
// failure mode this codebase's own history warns about -- see fp32_math.hpp's
// "nobody re-checked this one for four tasks") nor a marker comment a future
// author could forget to add: a NEW golden-feeding test file opts itself in
// merely by using the idiom it already has to use to find its own manifest.
// Comment-stripped so a file that only TALKS ABOUT the macro in prose --
// this very comment block, one section up, is exactly that case -- is not
// swept in on the strength of a mention alone; verified this scans exactly
// the intended set below (BitPortability's own report carries the file
// list).
// True iff `token` appears in `line` OUTSIDE of a quoted string/char
// literal. Needed because a plain substring search over comment-stripped
// text is fooled by exactly the self-referential case this file's own
// is_v1_include_directive() was hardened against for a different scanner
// (that function's own comment: "it flagged this file's OWN pattern-list
// literal") -- collect_golden_feeding_test_sources() below has to name the
// string "SPADE_GOLDEN_DIR" AS DATA to look for it, and a naive
// line.find("SPADE_GOLDEN_DIR") would then match that very literal inside
// THIS file, making test_m1b_bar.cpp "golden-feeding" by its own detection
// code and sweeping its own forbidden-pattern list (the literal
// "std::sin(", etc., a few lines above) into the scan. Skipping quoted
// content is what tells "the token, as code" apart from "the token's own
// name, as a string" -- not a full lexer (raw strings are not special-cased,
// same limitation strip_comments() above already documents), but sufficient
// for the one macro name this check looks for.
[[nodiscard]] bool line_has_unquoted_token(const std::string& line, std::string_view token) {
    bool in_string = false, in_char = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (in_string || in_char) {
            if (c == '\\' && i + 1 < line.size()) {
                ++i;  // skip the escaped character
            } else if ((in_string && c == '"') || (in_char && c == '\'')) {
                in_string = false;
                in_char = false;
            }
            continue;
        }
        if (c == '"') {
            in_string = true;
            continue;
        }
        if (c == '\'') {
            in_char = true;
            continue;
        }
        if (line.compare(i, token.size(), token) == 0) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] std::vector<fs::path> collect_golden_feeding_test_sources() {
    std::vector<fs::path> out;
    for (const fs::path& file : collect_sources(fs::path(SPADE_TESTS_DIR), {})) {
        for (const std::string& line : read_lines_no_comments(file)) {
            if (line_has_unquoted_token(line, "SPADE_GOLDEN_DIR")) {
                out.push_back(file);
                break;
            }
        }
    }
    return out;
}

[[nodiscard]] bool no_libm_transcendental_in_engine_or_golden_test_source(std::string& detail) {
    std::vector<fs::path> files = collect_sources(fs::path(SPADE_ENGINE_DIR), {"tools"});
    const std::vector<fs::path> golden_tests = collect_golden_feeding_test_sources();
    files.insert(files.end(), golden_tests.begin(), golden_tests.end());
    std::sort(files.begin(), files.end());  // collect_sources()'s own deterministic-report-order contract
    // S5 final-review fix wave (I3a): same vacuous-pass hazard as
    // no_wallclock_symbols_in_engine_source() above -- see that function's
    // comment. Equivalent to ASSERT_FALSE(files.empty()).
    if (files.empty()) {
        detail = "0 source files found under engine/ (tools/ excluded) + golden-feeding test sources -- "
                 "the scan roots are empty or missing, which would otherwise vacuously PASS; treating it "
                 "as a failure instead";
        return false;
    }
    const std::vector<std::string> hits =
        scan_for_forbidden(files,
                            {// the original five, qualified C++ spellings
                             "std::sin(", "std::cos(", "std::exp(", "std::log(", "std::pow(",
                             // std::tan and its inverse-trig siblings
                             "std::tan(", "std::atan(", "std::atan2(", "std::asin(", "std::acos(",
                             // f-suffixed C spellings
                             "sinf(", "cosf(", "expf(", "logf(", "powf(", "tanf(",
                             // glm's transcendental wrappers -- each calls libm internally, and
                             // glm::angleAxis is the exact cross-libm hazard this program vindicated
                             // sqrt-spelling against
                             "glm::angleAxis(", "glm::rotate(", "glm::eulerAngles(", "glm::slerp(",
                             "glm::quatLookAt("},
                            /*strip_comments_first=*/true, /*require_non_identifier_before=*/true);
    if (hits.empty()) {
        std::ostringstream ok;
        ok << "0 libm transcendental references in " << files.size() << " source files under engine/ "
           << "(tools/ excluded, comments excluded) + " << golden_tests.size() << " golden-feeding test file(s)";
        detail = ok.str();
        return true;
    }
    std::ostringstream bad;
    bad << hits.size() << " libm transcendental reference(s): ";
    for (std::size_t i = 0; i < hits.size() && i < 5; ++i) bad << (i == 0 ? "" : "; ") << hits[i];
    detail = bad.str();
    return false;
}

// ---------------------------------------------------------------------------
// Bullet 3 -- SEEDED DETERMINISM (corpus green), smoke scale.
//
// One small scenario -- a body under a scripted drag+wrench, no geometry --
// run twice via spade::testing::run_scenario(), the EXACT mechanism
// test_determinism.cpp's four-scenario corpus uses. 40 steps, one body: this
// is a SMOKE test of the mechanism, not a rerun of the corpus (which stays
// exactly where it is, gating test_determinism.cpp).
// ---------------------------------------------------------------------------
[[nodiscard]] spade::testing::Scenario m1b_smoke_scenario() {
    spade::testing::Scenario s;
    s.name = "m1b_smoke";
    s.dt_ns = 2'000'000;  // 2 ms step
    s.substeps = 2;       // 1 ms substep
    s.steps = 40;

    s.build = []() -> spade::Result<spade::WorldSetDesc> {
        const spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                          .name("m1b_void")
                                                          .environment(spade::Environment{})
                                                          .capacities(spade::Capacities{2, 2, 1, 1})
                                                          .build();
        if (!world) return std::unexpected(world.error());

        spade::WorldInstanceDesc instance;
        instance.world = *world;
        instance.seed = 0x5EEDED7ULL;
        instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::light);
        instance.contacts.restitution_e = 0.0f;
        instance.contacts.friction_mu = 0.0f;
        instance.contacts.proxy_radius = 0.1f;
        instance.grid.cell_size = 0.5f;
        return spade::WorldSetDesc{{instance}};
    };

    s.setup = [](spade::Simulation& sim) -> spade::Result<void> {
        spade::BodySpawn body;
        body.pos = glm::vec3(0.0f, 50.0f, 0.0f);
        body.vel = glm::vec3(1.0f, 0.0f, -0.5f);
        body.mass = 0.8f;
        body.inv_inertia_diag = glm::vec3(100.0f);
        const spade::Result<spade::BodyRef> ref = sim.spawn(0, body);
        if (!ref) return std::unexpected(ref.error());

        spade::DragElementSpawn drag;
        drag.mode = spade::physics::drag_mode::quadratic;
        drag.area = 0.03f;
        drag.coeffs = glm::vec3(0.8f, 0.0f, 0.0f);
        const spade::Result<spade::DragElementRef> elem = sim.add_drag_element(*ref, drag);
        if (!elem) return std::unexpected(elem.error());
        return {};
    };

    s.input = [](spade::Simulation& sim, spade::Tick tick) -> spade::Result<void> {
        const spade::Result<spade::BodyRef> ref = sim.body_ref_at(0, 0);
        if (!ref) return {};  // not yet flushed on tick 0; nothing to command
        const float a = static_cast<float>(tick.value % 5u) - 2.0f;
        return sim.apply_wrench(*ref, glm::vec3(0.02f * a, 0.0f, 0.0f), glm::vec3(0.0f));
    };

    return s;
}

[[nodiscard]] bool seeded_determinism_smoke(std::string& detail) {
    const spade::testing::Scenario scenario = m1b_smoke_scenario();
    const spade::Result<uint64_t> first = spade::testing::run_scenario(scenario);
    if (!first) {
        detail = std::string("first run failed: ") + first.error().context;
        return false;
    }
    const spade::Result<uint64_t> second = spade::testing::run_scenario(scenario);
    if (!second) {
        detail = std::string("second run failed: ") + second.error().context;
        return false;
    }
    std::ostringstream out;
    out << "two independent runs of a " << scenario.steps << "-step smoke scenario both digest to 0x" << std::hex
        << *first;
    detail = out.str();
    return *first == *second;
}

// ---------------------------------------------------------------------------
// Bullet 4 -- SNAPSHOT (round-trip green), smoke scale.
//
// The SAME replay-guarantee shape test_determinism.cpp's
// SnapshotRestoreIntoAFreshSimulationResumesIdentically performs: run
// uninterrupted to `steps` for a reference digest, then run again but stop
// at steps/2, snapshot, restore the blob into a BRAND NEW Simulation (never
// the one that produced it -- the part that actually proves the blob, not
// the live object, carries the state), resume to `steps`, and compare.
// ---------------------------------------------------------------------------
[[nodiscard]] bool snapshot_roundtrip_smoke(std::string& detail) {
    const spade::testing::Scenario scenario = m1b_smoke_scenario();
    const uint64_t k = scenario.steps / 2;

    spade::Result<spade::Simulation> reference = spade::testing::start_scenario(scenario);
    if (!reference) {
        detail = std::string("reference start failed: ") + reference.error().context;
        return false;
    }
    if (spade::Result<void> r = spade::testing::advance_scenario(scenario, *reference, scenario.steps); !r) {
        detail = std::string("reference advance failed: ") + r.error().context;
        return false;
    }
    const uint64_t expected = spade::testing::state_digest(*reference);

    spade::Result<spade::Simulation> interrupted = spade::testing::start_scenario(scenario);
    if (!interrupted) {
        detail = std::string("interrupted start failed: ") + interrupted.error().context;
        return false;
    }
    if (spade::Result<void> r = spade::testing::advance_scenario(scenario, *interrupted, k); !r) {
        detail = std::string("interrupted advance failed: ") + r.error().context;
        return false;
    }
    const spade::Result<spade::SnapshotBlob> blob = interrupted->snapshot();
    if (!blob) {
        detail = std::string("snapshot failed: ") + blob.error().context;
        return false;
    }
    if (blob->tick().value != k) {
        detail = "snapshot tick mismatch";
        return false;
    }

    spade::Result<spade::Simulation> resumed = spade::testing::start_scenario(scenario);  // a FRESH Simulation
    if (!resumed) {
        detail = std::string("resumed start failed: ") + resumed.error().context;
        return false;
    }
    if (spade::Result<void> r = resumed->restore(*blob); !r) {
        detail = std::string("restore failed: ") + r.error().context;
        return false;
    }
    if (spade::Result<void> r = spade::testing::advance_scenario(scenario, *resumed, scenario.steps); !r) {
        detail = std::string("resumed advance failed: ") + r.error().context;
        return false;
    }
    const uint64_t actual = spade::testing::state_digest(*resumed);

    std::ostringstream out;
    out << "snapshot at tick " << k << ", restored into a fresh Simulation, resumed to " << scenario.steps
        << " -- digest 0x" << std::hex << actual << (actual == expected ? " == " : " != ") << "uninterrupted 0x"
        << expected;
    detail = out.str();
    return actual == expected;
}

// ---------------------------------------------------------------------------
// Bullet 5 -- 6-DOF + QUADROTOR + IMU (suites green), smoke scale.
//
// One quadrotor, spawned in trim (VehicleSpawn::rotor_omega ==
// hover_command(), sim/simulation.hpp's "avoids a rotor-spin-up transient"),
// held for half a second at 1 kHz, IMU polled. Checks 6-DOF (the body barely
// moves -- a hovering rigid body's full 6-DOF state staying near its spawn
// pose is only true if translation AND rotation both integrated correctly),
// quadrotor (the hold itself -- vehicles/quadrotor.hpp's whole derivation),
// and IMU (a hovering accelerometer reads +g opposite gravity -- sensors/
// imu.hpp section 1 -- which is only true if specific-force capture, the
// mount transform and the ring write all composed correctly).
// ---------------------------------------------------------------------------
[[nodiscard]] bool quadrotor_imu_smoke(std::string& detail) {
    constexpr float kG = 9.80665f;

    const spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                      .name("m1b_quad_void")
                                                      .environment(spade::Environment{})
                                                      .capacities(spade::Capacities{1, 5, 1, 1})
                                                      .build();
    if (!world) {
        detail = std::string("world build failed: ") + world.error().context;
        return false;
    }
    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0xB17;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.15f;
    instance.grid.cell_size = 0.4f;

    spade::Result<spade::Simulation> sim = spade::Simulation::create(spade::WorldSetDesc{{instance}}, 1'000'000, 1);
    if (!sim) {
        detail = std::string("Simulation::create failed: ") + sim.error().context;
        return false;
    }

    spade::vehicles::QuadrotorParams params;
    params.name = "m1b_quad";
    params.mass = 1.0f;
    params.inertia_diag = glm::vec3(0.02f, 0.03f, 0.025f);
    params.arm_length = 0.15f;
    params.rotor_height = 0.0f;
    for (spade::vehicles::RotorParams& rotor : params.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.12f;
        rotor.thrust_coeff = 1.0e-5f;
        rotor.torque_coeff = 1.6e-7f;
    }
    params.drag.mode = spade::physics::drag_mode::quadratic;
    params.imu.rate_divider = 1;  // ideal sensor, every substep

    const spade::Result<spade::vehicles::ModelType> model = spade::vehicles::make_quadrotor(params);
    if (!model) {
        detail = std::string("make_quadrotor failed: ") + model.error().context;
        return false;
    }
    const spade::Result<spade::ModelTypeId> id = sim->register_model(*model);
    if (!id) {
        detail = std::string("register_model failed: ") + id.error().context;
        return false;
    }

    const float hover = spade::vehicles::hover_command(params);
    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where.rotor_omega = hover;  // IN TRIM
    const spade::Result<spade::VehicleRef> vehicle = sim->spawn(0, *id, where);
    if (!vehicle) {
        detail = std::string("vehicle spawn failed: ") + vehicle.error().context;
        return false;
    }
    if (spade::Result<void> r = sim->flush_structural(); !r) {
        detail = std::string("flush_structural failed: ") + r.error().context;
        return false;
    }

    constexpr uint64_t kSubsteps = 500;  // 0.5 s at 1 kHz
    if (spade::Result<void> r = sim->step(kSubsteps); !r) {
        detail = std::string("step failed: ") + r.error().context;
        return false;
    }

    const spade::Result<const spade::BodyState*> body = sim->body(vehicle->body);
    if (!body) {
        detail = std::string("body read failed: ") + body.error().context;
        return false;
    }
    const float position_drift = glm::length((*body)->pos - where.pos);
    const float attitude_w = (*body)->orient.w;

    if (vehicle->imu_count == 0) {
        detail = "vehicle spawned with no IMU sensor";
        return false;
    }
    std::vector<spade::sensors::ImuSample> samples(spade::sensors::kRingDepth);
    const spade::Result<spade::ImuPoll> poll = sim->poll_imu(vehicle->imu_sensors[0], 0, samples);
    if (!poll) {
        detail = std::string("poll_imu failed: ") + poll.error().context;
        return false;
    }
    if (poll->samples.empty()) {
        detail = "IMU produced no samples over 500 substeps";
        return false;
    }
    const spade::sensors::ImuSample& latest = poll->samples.back();

    std::ostringstream out;
    out << kSubsteps << " substeps hovering: position drift " << position_drift << " m, orient.w " << attitude_w
        << ", " << poll->samples.size() << " IMU samples, latest accel.y " << latest.accel.y << " (+g = " << kG
        << "), latest gyro " << glm::length(latest.gyro) << " rad/s";
    detail = out.str();

    // THE CONTRACT, deliberately loose (this is a smoke test, not
    // test_quadrotor.cpp's ten-thousand-substep hold): position drift under
    // 5 cm, attitude within a hair of level, accel.y within 5% of +g
    // (sensors/imu.hpp section 1: level hover reads +g OPPOSITE gravity, not
    // ~0 -- imu.hpp section 5's "resting on the ground reads ~0" caveat does
    // NOT apply here; this vehicle is airborne under thrust the whole time),
    // gyro near zero (level, not rotating).
    if (position_drift >= 0.05f) return false;
    if (std::fabs(attitude_w) <= 0.999f) return false;
    if (latest.accel.y <= 0.95f * kG || latest.accel.y >= 1.05f * kG) return false;
    if (glm::length(latest.gyro) >= 0.05f) return false;
    return true;
}

}  // namespace

// ===========================================================================
// The six tests -- one clear responsibility each, per this task's own
// self-review contract.
// ===========================================================================

TEST(M1B, FixedStepNoWallClockSymbolsInEngineSource) {
    std::string detail;
    const bool ok = no_wallclock_symbols_in_engine_source(detail);
    EXPECT_TRUE(ok) << detail;
}

TEST(M1B, HeadlessNoV1IncludeInEngineOrTestSource) {
    std::string detail;
    const bool ok = no_v1_include_in_engine_or_test_source(detail);
    EXPECT_TRUE(ok) << detail;
}

// S5 T9 ticket C -- not one of the brief's original six bullets (it is the
// program-wide bit-portability rule, not an M1B charter item), kept in this
// suite because it reuses bullet 1's exact scanning MECHANISM (collect_
// sources()/scan_for_forbidden()) -- though, since S7a Task R5's SR-14
// widening above, no longer bullet 1's exact SCOPE: this one also sweeps
// every golden-feeding test source, which bullet 1 (engine-source-only,
// by design) never needs to.
TEST(BitPortability, NoLibmTranscendentalInEngineOrGoldenTestSource) {
    std::string detail;
    const bool ok = no_libm_transcendental_in_engine_or_golden_test_source(detail);
    EXPECT_TRUE(ok) << detail;
}

TEST(M1B, SeededDeterminismCorpusMechanismGreenAtSmokeScale) {
    std::string detail;
    const bool ok = seeded_determinism_smoke(detail);
    EXPECT_TRUE(ok) << detail;
}

TEST(M1B, SnapshotRoundTripMechanismGreenAtSmokeScale) {
    std::string detail;
    const bool ok = snapshot_roundtrip_smoke(detail);
    EXPECT_TRUE(ok) << detail;
}

TEST(M1B, SixDofQuadrotorImuSuitesMechanismGreenAtSmokeScale) {
    std::string detail;
    const bool ok = quadrotor_imu_smoke(detail);
    EXPECT_TRUE(ok) << detail;
}

// ---------------------------------------------------------------------------
// The summary bar table. Re-derives all five booleans above through the SAME
// helper functions the five tests each call -- so this table can never
// report a row it did not actually (re-)check, and a failure here is not a
// sixth independent risk, it is one of the five above surfacing a second
// time with the full table for context.
//
// VISIBILITY NOTE, stated rather than assumed: ctest (test.ps1's own
// invocation) suppresses a passing test's stdout unless run with
// --verbose/-V or --output-on-failure (which only shows it for a FAILING
// test). This table therefore prints unconditionally into THIS test's own
// stdout either way; seeing it on a green run means passing -V to ctest, the
// same as any other diagnostic print in this suite -- there is no special
// exemption from that for M1B.
// ---------------------------------------------------------------------------
TEST(M1B, SummaryBarTable) {
    struct Row {
        const char* charter_item;
        bool ok;
        std::string detail;
    };
    std::vector<Row> rows;
    {
        std::string detail;
        const bool ok = no_wallclock_symbols_in_engine_source(detail);
        rows.push_back({"fixed-step   (no wall-clock symbol in engine source)", ok, detail});
    }
    {
        std::string detail;
        const bool ok = no_v1_include_in_engine_or_test_source(detail);
        rows.push_back({"headless     (no v1 include in engine/test source)", ok, detail});
    }
    {
        std::string detail;
        const bool ok = seeded_determinism_smoke(detail);
        rows.push_back({"determinism  (seeded, corpus mechanism, smoke)", ok, detail});
    }
    {
        std::string detail;
        const bool ok = snapshot_roundtrip_smoke(detail);
        rows.push_back({"snapshot     (round-trip, smoke)", ok, detail});
    }
    {
        std::string detail;
        const bool ok = quadrotor_imu_smoke(detail);
        rows.push_back({"6-dof+quad+imu (suites mechanism, smoke)", ok, detail});
    }

    std::printf("\n");
    std::printf("=================== Spade v2 M1B charter bar ===================\n");
    for (const Row& row : rows) {
        std::printf("[%s] %s\n", row.ok ? "PASS" : "FAIL", row.charter_item);
        std::printf("       %s\n", row.detail.c_str());
    }
    std::printf("==================================================================\n\n");
    std::fflush(stdout);

    for (const Row& row : rows) {
        EXPECT_TRUE(row.ok) << row.charter_item << ": " << row.detail;
    }
}

// ===========================================================================
// Addendum A3, CASE 1 of 2 -- "reset-preserves-roster round-trip"
// (written at Task 21 close-out; case 2, reseed determinism, follows below
// and was added by S5 Task 4 once Simulation::reseed() existed).
//
// RULING, from the Task 21 brief: do not build new engine surface during
// close-out. Addendum A3 names two conformance cases. reseed determinism
// needed Simulation::reseed(seed), which did not exist then --
// WorldParams::seed was fixed at create() from the WorldSetDesc and never
// rewritten after -- so that case was deferred to a follow-up ticket.
// THIS case needed no new surface: restore()'s own doc comment says a
// restore overwrites EVERY arena byte -- including each array's derived
// free lists and live counts -- and rewinds the tick to match, all in one
// call, which is exactly what "reset to an earlier point" means.
// register_model() is NOT part of the blob (restore() doc comment, again),
// so restoring into the SAME Simulation object -- one that never forgot its
// model registrations -- is what makes this a RESET rather than the
// resume-into-a-fresh-object shape test_quadrotor.cpp's
// AFlyingVehicleSurvivesASnapshotIntoASimulationThatNeverSpawnedIt already
// covers.
// ===========================================================================
TEST(M1B, ResetPreservesRosterRoundTrip) {
    const spade::Result<spade::WorldDesc> world =
        spade::WorldBuilder()
            .name("m1b_roster_void")
            .environment(spade::Environment{})
            .capacities(spade::Capacities{3, 15, 3, 1})  // headroom for 3 vehicles' worth transiently
            .build();
    ASSERT_OK(world);

    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0xA3;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.15f;
    instance.grid.cell_size = 0.4f;

    spade::Result<spade::Simulation> sim = spade::Simulation::create(spade::WorldSetDesc{{instance}}, 1'000'000, 1);
    ASSERT_OK(sim);

    spade::vehicles::QuadrotorParams params;
    params.name = "m1b_roster_quad";
    params.mass = 1.0f;
    params.inertia_diag = glm::vec3(0.02f, 0.03f, 0.025f);
    params.arm_length = 0.15f;
    for (spade::vehicles::RotorParams& rotor : params.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.12f;
        rotor.thrust_coeff = 1.0e-5f;
        rotor.torque_coeff = 1.6e-7f;
    }
    params.drag.mode = spade::physics::drag_mode::quadratic;
    params.imu.rate_divider = 1;  // ideal sensor, every substep

    const spade::Result<spade::vehicles::ModelType> model = spade::vehicles::make_quadrotor(params);
    ASSERT_OK(model);
    const spade::Result<spade::ModelTypeId> id = sim->register_model(*model);
    ASSERT_OK(id);

    const float hover = spade::vehicles::hover_command(params);

    // THE ROSTER: two vehicles, A and B, in trim.
    spade::VehicleSpawn where_a;
    where_a.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where_a.rotor_omega = hover;
    ASSERT_OK(sim->spawn(0, *id, where_a));

    spade::VehicleSpawn where_b;
    where_b.pos = glm::vec3(2.0f, 12.0f, -1.0f);
    where_b.rotor_omega = hover;
    ASSERT_OK(sim->spawn(0, *id, where_b));

    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->step(30));

    // THE SNAPSHOT -- the point a "reset" below will return to.
    const spade::Result<uint32_t> baseline_bodies = sim->live_body_count(0);
    ASSERT_OK(baseline_bodies);
    const spade::Result<uint32_t> baseline_rotors = sim->live_rotor_count(0);
    ASSERT_OK(baseline_rotors);
    const spade::Result<uint32_t> baseline_sensors = sim->live_imu_sensor_count(0);
    ASSERT_OK(baseline_sensors);
    const spade::Tick baseline_tick = sim->tick();
    const spade::Result<spade::SnapshotBlob> roster_blob = sim->snapshot();
    ASSERT_OK(roster_blob);
    const uint64_t baseline_digest = spade::testing::state_digest(*sim);

    // MUTATE THE ROSTER past the snapshot point: despawn A, spawn a THIRD
    // vehicle C (landing on A's just-freed slot -- lowest-free-first, same
    // as test_quadrotor.cpp's despawn-cascade test relies on), step further.
    // If restore() left any of A's, B's or C's rows behind, or the free
    // list pointed at the wrong slot afterward, the round trip below would
    // not come out exact.
    const spade::Result<spade::BodyRef> body_a = sim->body_ref_at(0, 0);
    ASSERT_OK(body_a);
    ASSERT_OK(sim->despawn(*body_a));
    spade::VehicleSpawn where_c;
    where_c.pos = glm::vec3(-3.0f, 8.0f, 4.0f);
    where_c.rotor_omega = hover * 0.7f;
    ASSERT_OK(sim->spawn(0, *id, where_c));
    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->step(20));

    ASSERT_NE(spade::testing::state_digest(*sim), baseline_digest) << "the mutation step above must actually "
                                                                       "change state, or the round-trip check "
                                                                       "below proves nothing";

    // THE RESET: restore the earlier snapshot into the SAME Simulation.
    ASSERT_OK(sim->restore(*roster_blob));

    // THE ROSTER IS BACK. Not just "same counts" -- the exact same
    // registered state the pre-mutation snapshot captured, byte for byte.
    EXPECT_EQ(sim->tick().value, baseline_tick.value);
    const spade::Result<uint32_t> restored_bodies = sim->live_body_count(0);
    ASSERT_OK(restored_bodies);
    EXPECT_EQ(*restored_bodies, *baseline_bodies);
    const spade::Result<uint32_t> restored_rotors = sim->live_rotor_count(0);
    ASSERT_OK(restored_rotors);
    EXPECT_EQ(*restored_rotors, *baseline_rotors);
    const spade::Result<uint32_t> restored_sensors = sim->live_imu_sensor_count(0);
    ASSERT_OK(restored_sensors);
    EXPECT_EQ(*restored_sensors, *baseline_sensors);
    EXPECT_EQ(spade::testing::state_digest(*sim), baseline_digest);

    // THE ROSTER IS USABLE, not just byte-identical dead arena bytes: refs
    // captured before a restore are stale (restore rewinds generation
    // counters too, per body_ref_at()'s own doc comment), so re-deriving
    // from the state the restore actually produced is the honest way to
    // confirm vehicle A -- gone a moment ago -- is genuinely flying again.
    const spade::Result<spade::BodyRef> restored_a = sim->body_ref_at(0, 0);
    ASSERT_OK(restored_a);
    const spade::Result<const spade::BodyState*> restored_a_body = sim->body(*restored_a);
    ASSERT_OK(restored_a_body);
    EXPECT_TRUE(std::isfinite((*restored_a_body)->pos.y));
    EXPECT_GT((*restored_a_body)->pos.y, 5.0f) << "vehicle A should be back near its hover altitude, not fallen";
}

// ===========================================================================
// Addendum A3, CASE 2 of 2 -- RESEED DETERMINISM (S5 Task 4).
//
// The property, stated as the thing a training loop depends on: reseeding a
// running world set to a new SCENE seed is a pure function of (state, scene
// seed) that actually re-randomizes everything the old seed had touched --
// so episode k+1 is a genuinely different roll of the dice AND is
// reproducible from the seed alone.
//
// Three asserts carry it, one test each, because a failure in any one of
// them means something different:
//
//   1. DETERMINISM. Two identical runs reseeded to the same S at the same
//      tick stay bit-identical. Fails if reseed reads anything the state
//      does not carry.
//   2. DISCRIMINATION. A reseeded run diverges from the same run left
//      alone -- and diverges IN THE TRAJECTORY, not merely in the seed
//      field the call just wrote. Fails if a reseed is cosmetic.
//   3. THE DERIVATION PIN. The seeds are replicate()'s, and the streams
//      derived from them are the ones sensors/imu.hpp specifies. Fails if
//      reseed invents its own derivation, which would make "reseed to S"
//      and "rebuild the set at scene seed S" two different scenes.
//
// Plus two the S5 Task 4 brief adds: the structural-queue precondition, and
// the snapshot interaction (the new seeds live in registered state, so a
// blob taken after a reseed must carry them).
//
// WHY THE EXPECTATIONS ARE DERIVED AND NOT ECHOED. Every predicted value
// below is computed in THIS file from the pinned specification -- the
// derivation sim/world_set.hpp documents (splitmix64(scene_seed ^
// fnv1a64("world") ^ index), with the tag spelled as a LITERAL so a change
// to the pinned constant fails rather than following along) and the noise
// model sensors/imu.hpp section 4 states (twelve draws per sample, in the
// documented order). Nothing here reads the engine's answer and asserts it
// equals itself.
// ===========================================================================

namespace {

// Two arbitrary, fixed, DIFFERENT scene seeds. Nothing below is a statement
// about these particular values.
constexpr uint64_t kSceneSeedA = 0xA3000001ULL;
constexpr uint64_t kSceneSeedB = 0xA3000002ULL;

constexpr uint32_t kReseedWorlds = 3;  // > 1, so the per-world derivation is exercised

// The sensor configuration the reseed tests use. Non-zero on all four
// channels, so both the white noise and the bias walk are in play.
constexpr float kSigmaA = 0.05f;
constexpr float kSigmaG = 0.01f;
constexpr float kSigmaBa = 0.001f;
constexpr float kSigmaBg = 0.0005f;

// The world these tests replicate: TURBULENCE ACTIVE, and (see
// reseed_sim()) a drag element on the body, so the Dryden stream actually
// reaches the trajectory. Without that coupling a reseed would still move
// the digest -- WorldParams::seed and DrydenState are both registered state
// -- while changing nothing anyone could feel, and a "discrimination"
// assert that only sees the seed field the call just wrote would prove
// nothing.
[[nodiscard]] spade::Result<spade::WorldInstanceDesc> reseed_prototype() {
    const spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
                                                      .name("a3_reseed_void")
                                                      .environment(spade::Environment{})
                                                      .capacities(spade::Capacities{2, 2, 2, 1})
                                                      .build();
    if (!world) return std::unexpected(world.error());

    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0;  // replicate() overwrites this; it is never read
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::severe);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.1f;
    instance.grid.cell_size = 0.5f;
    return instance;
}

// kReseedWorlds copies of that world at `scene_seed` -- built through
// replicate(), which is the seeding path reseed() claims to reproduce -- each
// holding one body with one quadratic drag element (the turbulence coupling)
// and one noisy IMU (the sensor-stream coupling), flushed and sitting at
// tick 0.
[[nodiscard]] spade::Result<spade::Simulation> reseed_sim(uint64_t scene_seed) {
    const spade::Result<spade::WorldInstanceDesc> prototype = reseed_prototype();
    if (!prototype) return std::unexpected(prototype.error());

    spade::Result<spade::Simulation> sim =
        spade::Simulation::create(spade::replicate(*prototype, kReseedWorlds, scene_seed), 2'000'000, 2);
    if (!sim) return std::unexpected(sim.error());

    for (uint32_t w = 0; w < kReseedWorlds; ++w) {
        spade::BodySpawn body;
        body.pos = glm::vec3(0.0f, 50.0f, 0.0f);
        body.mass = 0.5f;
        body.inv_inertia_diag = glm::vec3(50.0f);
        const spade::Result<spade::BodyRef> ref = sim->spawn(w, body);
        if (!ref) return std::unexpected(ref.error());

        spade::DragElementSpawn drag;
        drag.mode = spade::physics::drag_mode::quadratic;
        drag.area = 0.05f;
        drag.coeffs = glm::vec3(1.0f, 0.0f, 0.0f);  // Cd; y/z unused in this mode
        if (const spade::Result<spade::DragElementRef> elem = sim->add_drag_element(*ref, drag); !elem) {
            return std::unexpected(elem.error());
        }

        spade::ImuSensorSpawn imu;
        imu.sigma_a = kSigmaA;
        imu.sigma_g = kSigmaG;
        imu.sigma_ba = kSigmaBa;
        imu.sigma_bg = kSigmaBg;
        if (const spade::Result<spade::ImuSensorRef> sensor = sim->add_imu_sensor(*ref, imu); !sensor) {
            return std::unexpected(sensor.error());
        }
    }

    if (const spade::Result<void> flushed = sim->flush_structural(); !flushed) {
        return std::unexpected(flushed.error());
    }
    return sim;
}

// replicate()'s derivation, RE-WRITTEN FROM THE SPEC rather than called:
// sim/world_set.hpp pins both the expression and the domain tag. The tag is
// spelled as a LITERAL on purpose -- a test that reused
// kWorldSeedDomainTag would keep passing if that pinned constant were
// changed, which is precisely the change it exists to catch.
[[nodiscard]] uint64_t expected_world_seed(uint64_t scene_seed, uint32_t world) {
    return spade::rng::splitmix64(scene_seed ^ spade::rng::fnv1a64("world") ^ uint64_t{world});
}

// Three standard normals in the pinned x, y, z order (sensors/imu.cpp's
// draw_gauss3 exists to force exactly this sequencing, and the order is part
// of the contract rather than an implementation detail).
[[nodiscard]] glm::vec3 draw_gauss3(spade::rng::Stream& stream) {
    const float x = stream.next_gauss();
    const float y = stream.next_gauss();
    const float z = stream.next_gauss();
    return glm::vec3(x, y, z);
}

struct NoiseOnlySample {
    glm::vec3 accel{0.0f};
    glm::vec3 gyro{0.0f};
};

// One emitted sample of a COM-mounted, identity-mount sensor riding a body in
// FREE FALL, predicted from sensors/imu.hpp section 4's stated model alone.
//
// Free fall is what makes a complete prediction possible: specific_force is
// EXACTLY zero there (test_imu.cpp's FreeFallingBodyReadsZeroSpecificForce
// pins it -- gravity never enters force_acc) and an untorqued body's omega
// stays exactly zero, so accel_true and gyro_true vanish and the sample IS
// the noise. Twelve draws per sample in the header's pinned order (accel bias
// walk, gyro bias walk, accel white, gyro white), the walks advance the
// caller's bias state once per SAMPLE, and both sums are grouped left to
// right exactly as section 4 writes them -- fp32 addition is not associative,
// so the grouping is part of the expectation.
[[nodiscard]] NoiseOnlySample next_free_fall_sample(spade::rng::Stream& stream, glm::vec3& bias_a,
                                                     glm::vec3& bias_g) {
    const glm::vec3 walk_a = draw_gauss3(stream);
    const glm::vec3 walk_g = draw_gauss3(stream);
    const glm::vec3 white_a = draw_gauss3(stream);
    const glm::vec3 white_g = draw_gauss3(stream);

    bias_a += kSigmaBa * walk_a;
    bias_g += kSigmaBg * walk_g;

    NoiseOnlySample out;
    out.accel = glm::vec3(0.0f) + bias_a + kSigmaA * white_a;
    out.gyro = glm::vec3(0.0f) + bias_g + kSigmaG * white_g;
    return out;
}

// World `w`'s single drag body, re-derived from the state rather than from a
// ref captured earlier.
[[nodiscard]] spade::Result<glm::vec3> body_position(const spade::Simulation& sim, uint32_t w) {
    const spade::Result<spade::BodyRef> ref = sim.body_ref_at(w, 0);
    if (!ref) return std::unexpected(ref.error());
    const spade::Result<const spade::BodyState*> body = sim.body(*ref);
    if (!body) return std::unexpected(body.error());
    return (*body)->pos;
}

constexpr uint64_t kPreReseedSteps = 20;   // 40 ms at a 2 ms step
constexpr uint64_t kPostReseedSteps = 60;  // 120 ms more

}  // namespace

// (1) DETERMINISM. Two independently built, identically stepped Simulations,
// reseeded to the same scene seed at the same tick, must remain bit-identical
// -- immediately (reseed is a pure function of the state and the seed) and
// after stepping on (the streams it re-derived are the same streams).
TEST(M1B, ReseedIsDeterministic) {
    spade::Result<spade::Simulation> a = reseed_sim(kSceneSeedA);
    ASSERT_OK(a);
    spade::Result<spade::Simulation> b = reseed_sim(kSceneSeedA);
    ASSERT_OK(b);

    ASSERT_OK(a->step(kPreReseedSteps));
    ASSERT_OK(b->step(kPreReseedSteps));
    ASSERT_EQ(spade::testing::state_digest(*a), spade::testing::state_digest(*b))
        << "the two runs diverged BEFORE any reseed; nothing below would mean anything";

    ASSERT_OK(a->reseed(kSceneSeedB));
    ASSERT_OK(b->reseed(kSceneSeedB));
    EXPECT_EQ(spade::testing::state_digest(*a), spade::testing::state_digest(*b))
        << "reseed is not a pure function of (state, scene seed)";

    ASSERT_OK(a->step(kPostReseedSteps));
    ASSERT_OK(b->step(kPostReseedSteps));
    EXPECT_EQ(spade::testing::state_digest(*a), spade::testing::state_digest(*b))
        << "two identically reseeded runs diverged while stepping on";
}

// (2) DISCRIMINATION. The same run, reseeded versus left alone, must part
// company -- and the assert that carries the weight is the TRAJECTORY one:
// the digest necessarily differs the instant WorldParams::seed is rewritten,
// so digest inequality alone would be satisfied by a reseed that re-derived
// nothing. The bodies only move apart if the new Dryden stream is actually
// blowing on them.
TEST(M1B, ReseedDivergesFromAnUnreseededContinuation) {
    spade::Result<spade::Simulation> reseeded = reseed_sim(kSceneSeedA);
    ASSERT_OK(reseeded);
    spade::Result<spade::Simulation> untouched = reseed_sim(kSceneSeedA);
    ASSERT_OK(untouched);

    ASSERT_OK(reseeded->step(kPreReseedSteps));
    ASSERT_OK(untouched->step(kPreReseedSteps));
    ASSERT_EQ(spade::testing::state_digest(*reseeded), spade::testing::state_digest(*untouched));

    ASSERT_OK(reseeded->reseed(kSceneSeedB));  // ...and `untouched` is left exactly as it was

    ASSERT_OK(reseeded->step(kPostReseedSteps));
    ASSERT_OK(untouched->step(kPostReseedSteps));

    EXPECT_NE(spade::testing::state_digest(*reseeded), spade::testing::state_digest(*untouched));

    for (uint32_t w = 0; w < kReseedWorlds; ++w) {
        const spade::Result<glm::vec3> moved = body_position(*reseeded, w);
        ASSERT_OK(moved);
        const spade::Result<glm::vec3> original = body_position(*untouched, w);
        ASSERT_OK(original);
        EXPECT_NE(*moved, *original)
            << "world " << w
            << ": the reseeded run's body followed the SAME trajectory as the un-reseeded one, so "
               "the new Dryden stream never reached the physics -- the reseed was cosmetic";
    }
}

// (3) THE DERIVATION PIN. Two halves: the seeds themselves are replicate()'s,
// and a sensor seeded after the reseed draws the stream sensors/imu.hpp
// specifies from the new world seed.
TEST(M1B, ReseedDerivesEveryWorldSeedByReplicatesFormula) {
    spade::Result<spade::Simulation> sim = reseed_sim(kSceneSeedA);
    ASSERT_OK(sim);
    ASSERT_OK(sim->step(kPreReseedSteps));
    ASSERT_OK(sim->reseed(kSceneSeedB));

    // Half one: every world's registered seed is the formula's value -- and,
    // separately, is byte-for-byte what replicate() itself would have handed
    // a set built fresh at kSceneSeedB. Two independent statements of the
    // same claim: the first pins the EXPRESSION and the domain tag, the
    // second pins the equivalence of "reseed to S" and "rebuild at S".
    const spade::Result<spade::WorldInstanceDesc> prototype = reseed_prototype();
    ASSERT_OK(prototype);
    const spade::WorldSetDesc rebuilt = spade::replicate(*prototype, kReseedWorlds, kSceneSeedB);
    ASSERT_EQ(rebuilt.worlds.size(), kReseedWorlds);

    for (uint32_t w = 0; w < kReseedWorlds; ++w) {
        const spade::Result<const spade::WorldParams*> params = sim->world_params(w);
        ASSERT_OK(params);
        EXPECT_EQ((*params)->seed, expected_world_seed(kSceneSeedB, w)) << "world " << w;
        EXPECT_EQ((*params)->seed, rebuilt.worlds[w].seed)
            << "world " << w
            << ": a reseeded set must be seed-for-seed the set replicate() would have built from "
               "the same scene seed";
    }

    // Half two: a sensor whose stream is derived AFTER the reseed draws from
    // the NEW seed. It is bolted to a second, element-free body so that body
    // is in free fall and its samples are pure noise -- which is what lets an
    // independently constructed stream predict them exactly rather than
    // approximately.
    spade::BodySpawn faller;
    faller.pos = glm::vec3(20.0f, 60.0f, 0.0f);  // far from world 0's drag body: no contact pairs
    faller.mass = 1.0f;
    faller.inv_inertia_diag = glm::vec3(1.0f);
    const spade::Result<spade::BodyRef> faller_ref = sim->spawn(0, faller);
    ASSERT_OK(faller_ref);

    spade::ImuSensorSpawn spec;  // COM mount, identity orientation, divider 1
    spec.sigma_a = kSigmaA;
    spec.sigma_g = kSigmaG;
    spec.sigma_ba = kSigmaBa;
    spec.sigma_bg = kSigmaBg;
    const spade::Result<spade::ImuSensorRef> fresh_sensor = sim->add_imu_sensor(*faller_ref, spec);
    ASSERT_OK(fresh_sensor);
    ASSERT_OK(sim->flush_structural());

    const uint32_t local_slot =
        fresh_sensor->slot - fresh_sensor->world_index * sim->layout().sensor_capacity;

    constexpr uint64_t kPinSteps = 3;
    ASSERT_OK(sim->step(kPinSteps));

    std::vector<spade::sensors::ImuSample> buffer(spade::sensors::kRingDepth);
    const spade::Result<spade::ImuPoll> poll = sim->poll_imu(*fresh_sensor, 0, buffer);
    ASSERT_OK(poll);
    // Divider 1 at two substeps per step: one sample per substep.
    ASSERT_EQ(poll->samples.size(), kPinSteps * 2);

    spade::rng::Stream predicted =
        spade::sensors::imu_noise_stream(expected_world_seed(kSceneSeedB, 0), local_slot);
    glm::vec3 bias_a(0.0f);
    glm::vec3 bias_g(0.0f);
    for (std::size_t i = 0; i < poll->samples.size(); ++i) {
        const NoiseOnlySample want = next_free_fall_sample(predicted, bias_a, bias_g);
        EXPECT_EQ(poll->samples[i].accel, want.accel)
            << "sample " << i
            << ": a sensor seeded after the reseed is not drawing imu_noise_stream(new world seed, "
               "world-local slot)";
        EXPECT_EQ(poll->samples[i].gyro, want.gyro) << "sample " << i;
    }
}

// The structural-queue precondition, mirroring snapshot()'s. A pending op
// carries its own seeding, so a reseed issued over one would depend on how
// the two interleave; the call refuses instead.
TEST(M1B, ReseedRefusesOverAPendingStructuralQueue) {
    spade::Result<spade::Simulation> sim = reseed_sim(kSceneSeedA);
    ASSERT_OK(sim);
    ASSERT_OK(sim->step(1));

    spade::BodySpawn body;
    body.pos = glm::vec3(5.0f, 40.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    ASSERT_OK(sim->spawn(0, body));
    ASSERT_GT(sim->pending_structural_ops(), 0u);

    const spade::Result<void> refused = sim->reseed(kSceneSeedB);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(static_cast<int>(refused.error().code), static_cast<int>(spade::Code::invalid_argument));

    // The refusal left the queue AND the seeds alone -- it is a rejection,
    // not a partial application.
    EXPECT_GT(sim->pending_structural_ops(), 0u);
    const spade::Result<const spade::WorldParams*> params = sim->world_params(0);
    ASSERT_OK(params);
    EXPECT_EQ((*params)->seed, expected_world_seed(kSceneSeedA, 0));

    // Flushing makes it legal.
    ASSERT_OK(sim->flush_structural());
    EXPECT_OK(sim->reseed(kSceneSeedB));
}

// THE SNAPSHOT INTERACTION. The new seeds live in registered state
// (WorldParams) and so do the streams re-derived from them (DrydenState,
// ImuSensorRow::noise), so a blob taken after a reseed must carry the whole
// reseed with it. The twin is created at the OLD scene seed and never
// reseeded, which is what makes this a real test: if the blob did not carry
// the seeds, the twin would resume under kSceneSeedA's weather and diverge.
TEST(M1B, ReseedSurvivesASnapshotIntoATwin) {
    spade::Result<spade::Simulation> sim = reseed_sim(kSceneSeedA);
    ASSERT_OK(sim);
    ASSERT_OK(sim->step(kPreReseedSteps));
    ASSERT_OK(sim->reseed(kSceneSeedB));

    const spade::Result<spade::SnapshotBlob> blob = sim->snapshot();
    ASSERT_OK(blob);

    spade::Result<spade::Simulation> twin = reseed_sim(kSceneSeedA);  // still holds the OLD seeds
    ASSERT_OK(twin);
    {
        const spade::Result<const spade::WorldParams*> before = twin->world_params(0);
        ASSERT_OK(before);
        ASSERT_EQ((*before)->seed, expected_world_seed(kSceneSeedA, 0))
            << "the twin must start from the OLD scene seed, or it proves nothing";
    }
    ASSERT_OK(twin->restore(*blob));

    for (uint32_t w = 0; w < kReseedWorlds; ++w) {
        const spade::Result<const spade::WorldParams*> params = twin->world_params(w);
        ASSERT_OK(params);
        EXPECT_EQ((*params)->seed, expected_world_seed(kSceneSeedB, w))
            << "world " << w << ": the blob did not carry the reseeded world seed";
    }

    ASSERT_OK(sim->step(kPostReseedSteps));
    ASSERT_OK(twin->step(kPostReseedSteps));
    EXPECT_EQ(spade::testing::state_digest(*sim), spade::testing::state_digest(*twin))
        << "a run resumed from a post-reseed blob diverged from the run that produced it";
}
