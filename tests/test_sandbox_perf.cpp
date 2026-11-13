// L74 (the lead, 2026-11-13): the editor's two timing notes. Both are measurement only, off unless SPADE_EDITOR_PERF
// is set, and print `PERF <name> <value> <unit>` lines as Test/Docs' harness reads them (tests/bench/PERF.md). These
// tests pin the pure parts: the line's format, the median, the switch and the Run air recorder. The worker's own
// timing is in test_editor_airflow.cpp; the first-frame figure's clock (process_age_ms() in app_host.cpp, an OS call
// the test target does not compile) is read from a real run of the window.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../sandbox/perf_lines.hpp"

namespace {

namespace perf = spade::sandbox::perf;

// One line, name then value to four places then unit, with nothing that changes with the locale or the size.
TEST(SandboxPerf, ALineIsNameValueUnitWithFourPlaces) {
    EXPECT_EQ(perf::line("interface.editor_first_frame", 812.34567, "ms"), "PERF interface.editor_first_frame 812.3457 ms");
    EXPECT_EQ(perf::line("interface.diag_run_air_step", 0.00001, "ms"), "PERF interface.diag_run_air_step 0.0000 ms");
    EXPECT_EQ(perf::line("a.b", 123456.5, "s"), "PERF a.b 123456.5000 s") << "no exponent, no grouping";
    EXPECT_EQ(perf::line("a.b", 2.0, "ms").find('\n'), std::string::npos);
}

TEST(SandboxPerf, TheMedianIsTheMiddleAndTheMeanOfTheTwoMiddlesAndEmptyIsZero) {
    EXPECT_EQ(perf::median({3.0, 1.0, 2.0}), 2.0) << "unsorted in, odd count";
    EXPECT_EQ(perf::median({4.0, 1.0, 3.0, 2.0}), 2.5) << "even count";
    EXPECT_EQ(perf::median({7.0}), 7.0);
    EXPECT_EQ(perf::median({}), 0.0);
    EXPECT_EQ(perf::median({1.0, 1.0, 100.0}), 1.0) << "a spike (a shader build in the first frame) does not move it";
}

// Off unless asked: nothing, empty and "0" are off; any other text is on.
TEST(SandboxPerf, OffUnlessTheVariableAsksForIt) {
    EXPECT_FALSE(perf::enabled_from(nullptr));
    EXPECT_FALSE(perf::enabled_from(""));
    EXPECT_FALSE(perf::enabled_from("0"));
    EXPECT_TRUE(perf::enabled_from("1"));
    EXPECT_TRUE(perf::enabled_from("yes"));
    EXPECT_FALSE(perf::enabled()) << "the test run does not set SPADE_EDITOR_PERF";
}

// Run air: the whole frame, one step, the publish (the frame's last step and the publish, less one step) and the
// host's assembly, each the median over the run, in the order the harness reads them.
TEST(SandboxPerf, TheAirRecorderPrintsTheFourFiguresInOrder) {
    perf::AirPerf air;
    for (const double ms : {1.0, 2.0, 3.0}) air.step(ms);
    air.frame(/*advance_ms=*/5.0, /*assemble_ms=*/0.5, /*total_ms=*/20.0);
    air.frame(6.0, 0.5, 21.0);
    air.frame(7.0, 0.5, 22.0);
    const std::vector<std::string> expected = {
        "PERF interface.run_air_frame 21.0000 ms",
        "PERF interface.diag_run_air_step 2.0000 ms",
        "PERF interface.diag_run_air_publish 4.0000 ms",
        "PERF interface.diag_run_air_assemble 0.5000 ms",
    };
    EXPECT_EQ(air.lines(), expected);
}

TEST(SandboxPerf, NothingMeasuredPrintsNothingAndPublishIsNeverNegative) {
    perf::AirPerf none;
    EXPECT_TRUE(none.lines().empty()) << "no frame finished: no line, not a zero";
    perf::AirPerf steps_only;
    steps_only.step(2.0);
    EXPECT_TRUE(steps_only.lines().empty());
    perf::AirPerf odd;
    odd.step(9.0);
    odd.frame(/*advance_ms=*/4.0, 0.1, 10.0);  // an advance shorter than a step: the difference is noise
    const std::vector<std::string> got = odd.lines();
    ASSERT_EQ(got.size(), 4u);
    EXPECT_EQ(got[2], "PERF interface.diag_run_air_publish 0.0000 ms");
}

}  // namespace
