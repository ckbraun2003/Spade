// The runner's side of the run record (tools/fc_link/run_support.hpp): the memory peak (an error, never 0), the file hash (no path in its error)
// and the stage latency summary. Core's writer refuses a record with no peak; these are what make one honestly.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "fc_link/run_support.hpp"
#include "unique_temp.hpp"

namespace {

namespace fcl = spade::fc_link;

TEST(FcLinkRunSupport, TheLinuxPeakIsTheVmHWMLineInKilobytesAsBytes) {
    const std::string status = "Name:\tspade_tests\nVmPeak:\t  999999 kB\nVmHWM:\t   12345 kB\nVmRSS:\t    1000 kB\n";
    const auto peak = fcl::parse_peak_resident_kb(status);
    ASSERT_TRUE(peak.has_value());
    EXPECT_EQ(*peak, 12345ull * 1024ull);
}

TEST(FcLinkRunSupport, AnAbsentMalformedOrZeroPeakIsAnErrorNeverAZero) {
    for (const char* text : {"Name:\tx\nVmRSS:\t 1 kB\n", "VmHWM:\t abc kB\n", "VmHWM:\t 0 kB\n", "VmHWM:\t\n", ""}) {
        const auto peak = fcl::parse_peak_resident_kb(text);
        EXPECT_FALSE(peak.has_value()) << "'" << text << "'";
    }
}

TEST(FcLinkRunSupport, TheProcessPeakIsReadNeverZeroAndGrowsWithWhatTheProcessHolds) {
    const auto before = fcl::measure_memory_peak_bytes();
    ASSERT_TRUE(before.has_value());
    EXPECT_GT(*before, 1ull << 20) << "a test process holds more than a megabyte";
    std::vector<char> held(64u << 20, 1);  // touched, so it is resident
    held[held.size() / 2] = 2;
    const auto after = fcl::measure_memory_peak_bytes();
    ASSERT_TRUE(after.has_value());
    EXPECT_GE(*after, *before);
    EXPECT_GE(*after, 64ull << 20) << "the peak covers the 64 MB this test holds";
    EXPECT_EQ(held[held.size() / 2], 2);
}

TEST(FcLinkRunSupport, AFilesHashIsItsSha256AndAMissingFileIsAnErrorWithoutItsPath) {
    spade::test::TempDir tmp{"fc_run_support"};
    const std::filesystem::path abc = tmp.path() / "abc.txt";
    const std::filesystem::path empty = tmp.path() / "empty.txt";
    { std::ofstream(abc, std::ios::binary) << "abc"; }
    { std::ofstream(empty, std::ios::binary); }
    const auto h = fcl::hash_file(abc);
    ASSERT_TRUE(h.has_value());
    EXPECT_EQ(*h, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    const auto he = fcl::hash_file(empty);
    ASSERT_TRUE(he.has_value());
    EXPECT_EQ(*he, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    const std::filesystem::path missing = tmp.path() / "not_here_anywhere.txt";
    const auto none = fcl::hash_file(missing);
    ASSERT_FALSE(none.has_value());
    EXPECT_EQ(none.error().code, spade::Code::io_error);
    EXPECT_EQ(none.error().context.find("not_here_anywhere"), std::string::npos) << none.error().context;
    EXPECT_EQ(none.error().context.find(tmp.path().filename().string()), std::string::npos) << none.error().context;
}

TEST(FcLinkRunSupport, AStageSummaryIsMinMeanNearestRankP99AndMax) {
    const std::vector<double> five = {40.0, 10.0, 30.0, 20.0, 50.0};
    const auto a = fcl::summarise_stage("host_step", five);
    ASSERT_TRUE(a.has_value());
    EXPECT_EQ(a->name, "host_step");
    EXPECT_DOUBLE_EQ(a->min_ns, 10.0);
    EXPECT_DOUBLE_EQ(a->mean_ns, 30.0);
    EXPECT_DOUBLE_EQ(a->p99_ns, 50.0) << "ceil(0.99 * 5) = 5: the fifth sorted sample";
    EXPECT_DOUBLE_EQ(a->max_ns, 50.0);
    std::vector<double> hundred(100);
    for (std::size_t i = 0; i < hundred.size(); ++i) hundred[i] = static_cast<double>(100 - i);  // 100 down to 1, unsorted on purpose
    const auto b = fcl::summarise_stage("link", hundred);
    ASSERT_TRUE(b.has_value());
    EXPECT_DOUBLE_EQ(b->p99_ns, 99.0) << "ceil(0.99 * 100) = 99: the 99th sorted sample";
    EXPECT_DOUBLE_EQ(b->max_ns, 100.0);
    EXPECT_DOUBLE_EQ(b->min_ns, 1.0);
}

TEST(FcLinkRunSupport, AStageSummaryRefusesWhatItCannotSummariseNamingWhat) {
    const std::vector<double> ok = {1.0, 2.0};
    const auto no_samples = fcl::summarise_stage("host_step", std::vector<double>{});
    ASSERT_FALSE(no_samples.has_value());
    EXPECT_NE(no_samples.error().context.find("samples"), std::string::npos) << no_samples.error().context;
    for (const char* bad : {"", "Host Step", "host-step", "HOST"}) {
        const auto r = fcl::summarise_stage(bad, ok);
        ASSERT_FALSE(r.has_value()) << "'" << bad << "'";
        EXPECT_NE(r.error().context.find("name"), std::string::npos) << r.error().context;
    }
    const std::vector<double> negative = {1.0, -2.0};
    const std::vector<double> nan = {1.0, std::numeric_limits<double>::quiet_NaN()};
    for (const auto* samples : {&negative, &nan}) {
        const auto r = fcl::summarise_stage("host_step", *samples);
        ASSERT_FALSE(r.has_value());
        EXPECT_NE(r.error().context.find("sample"), std::string::npos) << r.error().context;
    }
}

}  // namespace
