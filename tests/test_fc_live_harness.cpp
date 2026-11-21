// ---------------------------------------------------------------------------
// test_fc_live_harness.cpp -- the first FcLive* suite: what scripts/fc-run.ps1 hands the autopilot tests.
//
// WHY. Tests that need a real autopilot (PX4 SITL in a container) carry the ctest label `fc`, not `spade` (TD-17; tests/AppendSpadeLabels.cmake, rule
// "FcLive*"), and run through scripts/fc-run.ps1, which starts one container for the run and sets three variables for the tests:
//   SPADE_FC_RUN_ID     the run's name (also the container's: spade-px4-<id>, and the run folder's)
//   SPADE_FC_HOST_ADDR  what PX4, inside its container, connects to: this machine
//   SPADE_FC_OUT_DIR    the run folder (build-fc/<run id>); the autopilot's log lands there
// This suite checks that contract, so the link and flight tests (docs/design/test-docs/plans/2026-11-21-fc-tests.md, FC-L1 and FC-F1), which start from the
// same three variables, can trust them. With no autopilot run (the variables unset, as under a bare `ctest -L fc`) it SKIPS by name; a skip never counts
// toward green (TD-13's reading), and fc-run.ps1 says so.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

namespace {

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)  // getenv
#endif
std::string env_or_empty(const char* name) {
    const char* v = std::getenv(name);
    return v != nullptr ? std::string(v) : std::string();
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif

}  // namespace

TEST(FcLiveHarness, TheRunnerHandsTheTestsItsRunAndFolder) {
    const std::string run = env_or_empty("SPADE_FC_RUN_ID");
    if (run.empty()) GTEST_SKIP() << "no autopilot run: run the fc tests through scripts/fc-run.ps1";

    const std::string host = env_or_empty("SPADE_FC_HOST_ADDR");
    const std::string out = env_or_empty("SPADE_FC_OUT_DIR");
    EXPECT_FALSE(host.empty()) << "SPADE_FC_HOST_ADDR is not set";
    ASSERT_FALSE(out.empty()) << "SPADE_FC_OUT_DIR is not set";

    std::error_code ec;
    EXPECT_TRUE(std::filesystem::is_directory(out, ec)) << "the run folder does not exist: " << out;
    EXPECT_EQ(std::filesystem::path(out).filename().string(), run) << "the run folder is not named for the run";
    // A run id is a plain name: it names a container and a folder, so nothing in it can be a path.
    EXPECT_EQ(run.find_first_of("/\\:"), std::string::npos) << run;
}
