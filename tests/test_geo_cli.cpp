#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../tools/geo_fetch/cli.hpp"
#include "geo_fixtures.hpp"
#include "unique_temp.hpp"

// spade_geo_fetch's command line, progress lines and exit codes (L85, SP-5; the design note section 2), on a fake network.

namespace {

namespace fs = std::filesystem;
using namespace spade;
using namespace spade::geofetch;
using namespace spade::geofetchtest;

struct CliRig {
    test::TempDir tmp{"geo_cli"};
    FakeHttp http;
    std::map<std::string, std::string> vars;
    CliEnv cli;
    std::ostringstream out, err;

    explicit CliRig(FakeHttp::Handler handler = FakeHttp::serve(GeoFixture{}))
        : http(std::move(handler)),
          cli{FetchEnv{http,
                       [](double, double, double) { return Bbox{-113.0, 40.0, -112.99, 40.01}; },
                       [] { return std::string("2026-11-17T01:23:45Z"); },
                       [](int) {},
                       {},
                       "t1"},
              [this](const char* name) -> const char* {
                  const auto it = vars.find(name);
                  return it == vars.end() ? nullptr : it->second.c_str();
              },
              MapPlatform::windows} {}

    [[nodiscard]] std::vector<std::string> base(bool with_cache = true) const {
        std::vector<std::string> a = {"--lat", "40.005", "--lon", "-112.995", "--side", "40", "--dem-cell", "10", "--image-cell", "20"};
        if (with_cache) {
            a.push_back("--cache");
            a.push_back((tmp.path() / "maps").string());
        }
        return a;
    }
    int run(std::vector<std::string> args) {
        out.str("");
        err.str("");
        return run_cli(args, cli, out, err);
    }
    [[nodiscard]] std::vector<std::string> lines() const {
        std::vector<std::string> v;
        std::istringstream in(out.str());
        for (std::string l; std::getline(in, l);) v.push_back(l);
        return v;
    }
    [[nodiscard]] std::string last() const {
        const auto v = lines();
        return v.empty() ? std::string() : v.back();
    }
};

[[nodiscard]] bool starts_with(const std::string& s, const std::string& p) { return s.rfind(p, 0) == 0; }

TEST(GeoCli, AFetchPrintsTheStagesAndEndsWithTheResultLine) {
    CliRig rig;
    EXPECT_EQ(rig.run(rig.base()), 0) << rig.err.str();
    EXPECT_TRUE(rig.err.str().empty());
    const std::string key = map_key(40.005, -112.995, 40.0);
    EXPECT_TRUE(starts_with(rig.last(), "fetched " + key + " ")) << rig.last();
    const std::string all = rig.out.str();
    for (const char* stage : {"geoid", "elevation", "image", "store"}) EXPECT_NE(all.find(stage), std::string::npos) << stage;
    EXPECT_NE(all.find("done"), std::string::npos);
}

TEST(GeoCli, JsonGivesOneObjectALineAndTheResultLast) {
    CliRig rig;
    auto args = rig.base();
    args.push_back("--json");
    EXPECT_EQ(rig.run(args), 0) << rig.err.str();
    const auto lines = rig.lines();
    ASSERT_GE(lines.size(), 8u);
    for (const std::string& l : lines) {
        EXPECT_EQ(l.front(), '{') << l;
        EXPECT_EQ(l.back(), '}') << l;
        EXPECT_NE(l.find("\"event\":"), std::string::npos) << l;
        // every backslash begins a valid JSON escape (a Windows path must be escaped)
        for (std::size_t i = 0; i < l.size(); ++i) {
            if (l[i] == '\\') {
                ASSERT_LT(i + 1, l.size());
                EXPECT_NE(std::string("\\\"/bfnrtu").find(l[i + 1]), std::string::npos) << l;
                ++i;
            }
        }
    }
    EXPECT_NE(lines.front().find("\"event\":\"stage\""), std::string::npos) << lines.front();
    EXPECT_NE(lines.front().find("\"stage\":\"geoid\""), std::string::npos) << lines.front();
    bool saw_progress = false;
    for (const std::string& l : lines) saw_progress = saw_progress || (l.find("\"event\":\"progress\"") != std::string::npos && l.find("\"bytes\":") != std::string::npos && l.find("\"total\":") != std::string::npos);
    EXPECT_TRUE(saw_progress);
    const std::string last = lines.back();
    EXPECT_NE(last.find("\"event\":\"result\""), std::string::npos) << last;
    EXPECT_NE(last.find("\"result\":\"fetched\""), std::string::npos) << last;
    EXPECT_NE(last.find("\"key\":\"" + map_key(40.005, -112.995, 40.0) + "\""), std::string::npos) << last;
    EXPECT_NE(last.find("\"geoid_height_m\":-23.4"), std::string::npos) << last;
    EXPECT_NE(last.find("\"anchor_ellipsoid_height_m\":"), std::string::npos) << last;
    EXPECT_NE(last.find("\"dir\":\""), std::string::npos) << last;
}

TEST(GeoCli, ASecondRunIsAHitAndAMissWithCheckIsExitThree) {
    CliRig rig;
    auto check = rig.base();
    check.push_back("--check");
    EXPECT_EQ(rig.run(check), 3);
    EXPECT_TRUE(starts_with(rig.last(), "miss ")) << rig.last();
    EXPECT_TRUE(rig.http.urls.empty());
    EXPECT_EQ(rig.run(rig.base()), 0);
    EXPECT_EQ(rig.run(rig.base()), 0);
    EXPECT_TRUE(starts_with(rig.last(), "hit ")) << rig.last();
    EXPECT_EQ(rig.http.urls.size(), 3u);
    EXPECT_EQ(rig.run(check), 0);
    EXPECT_TRUE(starts_with(rig.last(), "hit ")) << rig.last();
}

TEST(GeoCli, OfflineIsExitTenAndTheLastLineSaysWhy) {
    CliRig rig([](const std::string&, int) {
        HttpResponse r;
        r.error = HttpError::offline;
        r.detail = "no route to host";
        return r;
    });
    EXPECT_EQ(rig.run(rig.base()), 10);
    EXPECT_TRUE(starts_with(rig.last(), "offline: ")) << rig.last();
    EXPECT_NE(rig.last().find("no route to host"), std::string::npos) << rig.last();
    auto json = rig.base();
    json.push_back("--json");
    EXPECT_EQ(rig.run(json), 10);
    EXPECT_NE(rig.last().find("\"result\":\"offline\""), std::string::npos) << rig.last();
    EXPECT_NE(rig.last().find("\"message\":\""), std::string::npos) << rig.last();
}

TEST(GeoCli, AServiceErrorIsExitElevenAndNamesTheService) {
    GeoFixture f;
    f.png = "<?xml version=\"1.0\"?><Error/>";
    CliRig rig(FakeHttp::serve(f));
    EXPECT_EQ(rig.run(rig.base()), 11);
    EXPECT_TRUE(starts_with(rig.last(), "service_error: ")) << rig.last();
    EXPECT_NE(rig.last().find("naip"), std::string::npos) << rig.last();
    auto json = rig.base();
    json.push_back("--json");
    EXPECT_EQ(rig.run(json), 11);
    EXPECT_NE(rig.last().find("\"service\":\"naip\""), std::string::npos) << rig.last();
}

TEST(GeoCli, ARefusedRequestIsExitTwelveAndAFullCacheIsThirteen) {
    CliRig rig;
    auto big = rig.base();
    big[5] = "1500";  // --side 1500
    EXPECT_EQ(rig.run(big), 12);
    EXPECT_TRUE(starts_with(rig.last(), "refused: ")) << rig.last();
    EXPECT_TRUE(rig.http.urls.empty());

    auto full = rig.base();
    full.push_back("--cache-limit-mb");
    full.push_back("0");
    EXPECT_EQ(rig.run(full), 13);
    EXPECT_TRUE(starts_with(rig.last(), "cache_full: ")) << rig.last();
}

TEST(GeoCli, UsageProblemsAreExitTwoOnTheErrorStreamWithNothingOnOut) {
    CliRig rig;
    const std::vector<std::vector<std::string>> bad = {
        {},
        {"--lat", "40"},                                  // no --lon
        {"--lon", "-113"},                                // no --lat
        {"--lat", "forty", "--lon", "-113"},              // not a number
        {"--lat", "40", "--lon", "-113", "--frob"},       // an option that does not exist
        {"--lat", "40", "--lon", "-113", "--side"},       // a missing value
        {"--lat", "40", "--lon", "-113", "--cache-limit-mb", "-5"},
    };
    for (const auto& args : bad) {
        EXPECT_EQ(rig.run(args), kExitUsage);
        EXPECT_FALSE(rig.err.str().empty());
        EXPECT_TRUE(rig.out.str().empty()) << rig.out.str();
    }
    EXPECT_TRUE(rig.http.urls.empty());
}

TEST(GeoCli, HelpIsExitZeroAndListsTheOptions) {
    CliRig rig;
    EXPECT_EQ(rig.run({"--help"}), 0);
    for (const char* opt : {"--lat", "--lon", "--side", "--cache", "--cache-limit-mb", "--dem-cell", "--image-cell", "--check", "--refresh", "--json"}) {
        EXPECT_NE(rig.out.str().find(opt), std::string::npos) << opt;
    }
}

TEST(GeoCli, WithoutACacheOptionTheDefaultRootComesFromTheEnvironment) {
    CliRig rig;
    EXPECT_EQ(rig.run(rig.base(false)), 12);  // nothing set: refused, and it says why
    EXPECT_NE(rig.last().find("cache"), std::string::npos) << rig.last();
    rig.vars["SPADE_MAP_CACHE"] = (rig.tmp.path() / "envmaps").string();
    EXPECT_EQ(rig.run(rig.base(false)), 0) << rig.last();
    EXPECT_TRUE(fs::exists(rig.tmp.path() / "envmaps" / "v1" / map_key(40.005, -112.995, 40.0) / "georef.yaml"));
}

}  // namespace
