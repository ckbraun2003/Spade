#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "../tools/geo_fetch/cache_store.hpp"
#include "../tools/geo_fetch/fetch.hpp"
#include "geo_fixtures.hpp"
#include "unique_temp.hpp"
#include "world/map_cache.hpp"
#include "world/sha256.hpp"

// One map fetch, end to end, on fixtures (L85, SP-5; the design note sections 2 to 5, 8): a hit, a miss and a fetch, offline, the
// services' bad answers, the refusals, the size limit, refresh, progress. No test touches the network.

namespace {

namespace fs = std::filesystem;
using namespace spade;
using namespace spade::geofetch;
using namespace spade::geofetchtest;

const Bbox kBox{-113.0, 40.0, -112.99, 40.01};

[[nodiscard]] std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// A fetch rig: a temporary cache, a fake network, and the request the fixtures answer (a 4 x 4 grid and a 2 x 2 image).
struct Rig {
    test::TempDir tmp{"geo_fetch"};
    FakeHttp http;
    std::vector<ProgressEvent> events;
    std::vector<int> sleeps;
    FetchRequest request;
    FetchEnv env;

    explicit Rig(FakeHttp::Handler handler = FakeHttp::serve(GeoFixture{}))
        : http(std::move(handler)),
          env{http,
              [](double, double, double) { return kBox; },
              [] { return std::string("2026-11-17T01:23:45Z"); },
              [this](int ms) { sleeps.push_back(ms); },
              [this](const ProgressEvent& e) { events.push_back(e); },
              "t1"} {
        request.lat_deg = 40.005;
        request.lon_deg = -112.995;
        request.side_m = 40.0;
        request.dem_cell_m = 10.0;
        request.image_cell_m = 20.0;
        request.cache_root = tmp.path() / "maps";
    }

    [[nodiscard]] std::string key() const { return map_key(request.lat_deg, request.lon_deg, request.side_m); }
    [[nodiscard]] fs::path entry() const { return map_entry_dir(request.cache_root, key()); }
    [[nodiscard]] FetchResult run() { return fetch_map(request, env); }
};

TEST(GeoFetch, AMissFetchesThreeServicesInOrderAndStoresTheEntryWithEllipsoidHeights) {
    Rig rig;
    const FetchResult r = rig.run();
    ASSERT_EQ(r.kind, FetchResultKind::fetched) << r.message;
    EXPECT_EQ(rig.http.urls, (std::vector<std::string>{geoid_url(40.005, -112.995), elevation_url(kBox, 4, 4), image_url(kBox, 2, 2)}));
    EXPECT_EQ(r.key, rig.key());
    EXPECT_EQ(r.dir, rig.entry());
    EXPECT_DOUBLE_EQ(r.geoid_height_m, -23.4);
    EXPECT_NEAR(r.anchor_ellipsoid_height_m, 1003.0 + -23.4, 1e-9);  // the grid's middle, plus N

    // height.f32: each NAVD88 sample plus N, added in double and stored as float32 (little-endian, row 0 first).
    const std::string heights = slurp(rig.entry() / "height.f32");
    ASSERT_EQ(heights.size(), 16u * 4u);
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            float v = 0.0f;
            std::memcpy(&v, heights.data() + 4 * (row * 4 + col), 4);
            EXPECT_EQ(v, static_cast<float>(static_cast<double>(1000.0f + 2.0f * static_cast<float>(col)) + -23.4)) << row << "," << col;
        }
    }
    EXPECT_EQ(slurp(rig.entry() / "image.png"), GeoFixture{}.png);  // as received

    const Result<MapGeoref> g = parse_georef(slurp(rig.entry() / "georef.yaml"));
    ASSERT_TRUE(g.has_value()) << g.error().context;
    EXPECT_EQ(g->key, rig.key());
    EXPECT_EQ(g->anchor_lat_deg, 40.005);
    EXPECT_EQ(g->anchor_lon_deg, -112.995);
    EXPECT_EQ(g->side_m, 40.0);
    EXPECT_EQ(g->west_deg, kBox.west_deg);
    EXPECT_EQ(g->north_deg, kBox.north_deg);
    EXPECT_EQ(g->elevation.width, 4u);
    EXPECT_EQ(g->elevation.height, 4u);
    EXPECT_EQ(g->elevation.cell_m, 10.0);
    EXPECT_EQ(g->image.width, 2u);
    EXPECT_EQ(g->image.cell_m, 20.0);
    EXPECT_EQ(g->geoid_model, "GEOID18");
    EXPECT_EQ(g->geoid_height_m, -23.4);
    EXPECT_EQ(g->vertical, "wgs84_ellipsoid_m");
    EXPECT_EQ(g->height_sha256, sha256_hex(heights));
    EXPECT_EQ(g->image_sha256, sha256_hex(GeoFixture{}.png));
    EXPECT_EQ(g->elevation_source.name, "USGS 3DEP");
    EXPECT_EQ(g->elevation_source.url, elevation_url(kBox, 4, 4));
    EXPECT_EQ(g->image_source.url, image_url(kBox, 2, 2));
    EXPECT_EQ(g->geoid_source.name, "NOAA NGS");
    EXPECT_EQ(g->geoid_source.retrieved_utc, "2026-11-17T01:23:45Z");
    EXPECT_NE(g->datum_note.find("NAD83"), std::string::npos) << g->datum_note;  // recorded, not corrected (L85d Q3)
    EXPECT_EQ(g->fetched_utc, "2026-11-17T01:23:45Z");
    EXPECT_EQ(r.bytes, cache_size_bytes(rig.request.cache_root));
}

TEST(GeoFetch, ASecondRunOfTheSameAreaIsAHitWithNoRequest) {  // REQ-G12
    Rig rig;
    const FetchResult first = rig.run();
    ASSERT_EQ(first.kind, FetchResultKind::fetched) << first.message;
    ASSERT_EQ(rig.http.urls.size(), 3u);
    const FetchResult second = rig.run();
    EXPECT_EQ(second.kind, FetchResultKind::hit) << second.message;
    EXPECT_EQ(rig.http.urls.size(), 3u);  // nothing more asked
    EXPECT_EQ(second.dir, first.dir);
    EXPECT_DOUBLE_EQ(second.geoid_height_m, first.geoid_height_m);
    EXPECT_DOUBLE_EQ(second.anchor_ellipsoid_height_m, first.anchor_ellipsoid_height_m);
}

TEST(GeoFetch, CheckOnlyReportsHitOrMissAndNeverAsks) {
    Rig rig;
    rig.request.check_only = true;
    EXPECT_EQ(rig.run().kind, FetchResultKind::miss);
    EXPECT_TRUE(rig.http.urls.empty());
    rig.request.check_only = false;
    ASSERT_EQ(rig.run().kind, FetchResultKind::fetched);
    const std::size_t asked = rig.http.urls.size();
    rig.request.check_only = true;
    EXPECT_EQ(rig.run().kind, FetchResultKind::hit);
    EXPECT_EQ(rig.http.urls.size(), asked);
}

TEST(GeoFetch, RefreshFetchesAgainAndReplacesTheEntry) {
    GeoFixture changed;
    changed.geoid = R"({"geoidModel":"GEOID18","geoidHeight":-20.0})";
    Rig rig;
    ASSERT_EQ(rig.run().kind, FetchResultKind::fetched);
    Rig other(FakeHttp::serve(changed));
    other.request.cache_root = rig.request.cache_root;
    other.request.refresh = true;
    const FetchResult r = other.run();
    ASSERT_EQ(r.kind, FetchResultKind::fetched) << r.message;
    EXPECT_EQ(other.http.urls.size(), 3u);
    const Result<MapGeoref> g = parse_georef(slurp(rig.entry() / "georef.yaml"));
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->geoid_height_m, -20.0);
}

TEST(GeoFetch, OfflineIsNamedAfterTheFirstRequestAndLeavesNothing) {
    Rig rig([](const std::string&, int) {
        HttpResponse r;
        r.error = HttpError::offline;
        r.detail = "the host could not be resolved";
        return r;
    });
    const FetchResult r = rig.run();
    EXPECT_EQ(r.kind, FetchResultKind::offline);
    EXPECT_NE(r.message.find("could not be resolved"), std::string::npos) << r.message;
    EXPECT_EQ(rig.http.urls.size(), 1u);  // the geoid was first, and offline is not retried
    EXPECT_TRUE(rig.sleeps.empty());
    EXPECT_FALSE(fs::exists(rig.entry()));
}

TEST(GeoFetch, ANoHttpLayerPlatformIsOfflineToo) {
    Rig rig([](const std::string&, int) {
        HttpResponse r;
        r.error = HttpError::no_http_layer;
        r.detail = "no HTTP layer on this platform yet";
        return r;
    });
    const FetchResult r = rig.run();
    EXPECT_EQ(r.kind, FetchResultKind::offline);
    EXPECT_NE(r.message.find("no HTTP layer on this platform yet"), std::string::npos) << r.message;
}

TEST(GeoFetch, AServerErrorOrATimeoutIsRetriedOnceAfterTwoSeconds) {
    Rig rig([](const std::string& url, int nth) {
        HttpResponse r = FakeHttp::serve(GeoFixture{})(url, nth);
        if (url.find("geodesy") != std::string::npos && nth == 0) {
            r.status = 503;
            r.body.clear();
        }
        if (url.find("elevation") != std::string::npos && nth == 0) {
            r = HttpResponse{};
            r.error = HttpError::timeout;
            r.detail = "timed out";
        }
        return r;
    });
    const FetchResult r = rig.run();
    ASSERT_EQ(r.kind, FetchResultKind::fetched) << r.message;
    EXPECT_EQ(rig.http.urls.size(), 5u);  // geoid twice, elevation twice, image once
    EXPECT_EQ(rig.sleeps, (std::vector<int>{2000, 2000}));
}

TEST(GeoFetch, AServiceThatKeepsFailingIsAServiceErrorNamingIt) {
    Rig rig([](const std::string& url, int nth) {
        HttpResponse r = FakeHttp::serve(GeoFixture{})(url, nth);
        if (url.find("geodesy") != std::string::npos) {
            r.status = 500;
            r.body.clear();
        }
        return r;
    });
    const FetchResult r = rig.run();
    EXPECT_EQ(r.kind, FetchResultKind::service_error);
    EXPECT_EQ(r.service, "ngs");
    EXPECT_NE(r.message.find("500"), std::string::npos) << r.message;
    EXPECT_EQ(rig.http.urls.size(), 2u);  // asked, retried once, stopped
    EXPECT_FALSE(fs::exists(rig.entry()));
}

// Each answer the tool cannot use is a service error naming the service, and nothing is stored.
struct BadAnswer {
    const char* what;
    const char* service;
    std::function<void(GeoFixture&)> spoil;
    const char* says;  // a word the message must hold
};

TEST(GeoFetch, EachUnusableAnswerIsAServiceErrorWithNothingStored) {
    std::vector<float> plane(16);
    for (uint32_t i = 0; i < 16; ++i) plane[i] = 1000.0f + static_cast<float>(i % 4);
    const BadAnswer cases[] = {
        {"geoid not json", "ngs", [](GeoFixture& f) { f.geoid = "<html>nope</html>"; }, "geoidHeight"},
        {"geoid other model", "ngs", [](GeoFixture& f) { f.geoid = R"({"geoidModel":"GEOID12B","geoidHeight":-23})"; }, "GEOID18"},
        {"tiff wrong size", "3dep", [&](GeoFixture& f) { f.tiff = make_tiff(3, 3, std::vector<float>(9, 1000.0f)); }, "3 x 3"},
        {"tiff no-data", "3dep",
         [&](GeoFixture& f) {
             auto p = plane;
             p[5] = -9999.0f;
             f.tiff = make_tiff(4, 4, p);
         },
         "no-data"},
        {"tiff NaN", "3dep",
         [&](GeoFixture& f) {
             auto p = plane;
             p[7] = std::numeric_limits<float>::quiet_NaN();
             f.tiff = make_tiff(4, 4, p);
         },
         "no-data"},
        {"tiff compressed", "3dep",
         [&](GeoFixture& f) {
             TiffOptions o;
             o.compression = 8;
             f.tiff = make_tiff(4, 4, plane, o);
         },
         "compress"},
        {"tiff is html", "3dep", [](GeoFixture& f) { f.tiff = "<html>busy</html>"; }, "TIFF"},
        {"png wrong size", "naip", [](GeoFixture& f) { f.png = make_png(3, 3, 8, 2, "x"); }, "3 x 3"},
        {"png has alpha", "naip", [](GeoFixture& f) { f.png = make_png(2, 2, 8, 6, "x"); }, "RGB"},
        {"png 16-bit", "naip", [](GeoFixture& f) { f.png = make_png(2, 2, 16, 2, "x"); }, "8-bit"},
        {"png is xml", "naip", [](GeoFixture& f) { f.png = "<?xml version=\"1.0\"?><Error/>"; }, "PNG"},
    };
    for (const BadAnswer& c : cases) {
        GeoFixture f;
        c.spoil(f);
        Rig rig(FakeHttp::serve(f));
        const FetchResult r = rig.run();
        EXPECT_EQ(r.kind, FetchResultKind::service_error) << c.what << ": " << r.message;
        EXPECT_EQ(r.service, c.service) << c.what;
        EXPECT_NE(r.message.find(c.says), std::string::npos) << c.what << ": " << r.message;
        EXPECT_FALSE(fs::exists(rig.entry())) << c.what;
        EXPECT_EQ(cache_size_bytes(rig.request.cache_root), 0u) << c.what;
    }
}

TEST(GeoFetch, ARequestOutOfRangeIsRefusedWithoutAskingAnyone) {
    struct Case {
        const char* what;
        std::function<void(FetchRequest&)> change;
    };
    const Case cases[] = {
        {"side over 1 km", [](FetchRequest& q) { q.side_m = 1001.0; }},  // REQ-G07
        {"side under 10 m", [](FetchRequest& q) { q.side_m = 9.0; }},
        {"latitude 91", [](FetchRequest& q) { q.lat_deg = 91.0; }},
        {"longitude 181", [](FetchRequest& q) { q.lon_deg = 181.0; }},
        {"NaN latitude", [](FetchRequest& q) { q.lat_deg = std::numeric_limits<double>::quiet_NaN(); }},
        {"no cache root", [](FetchRequest& q) { q.cache_root.clear(); }},
        {"a grid over 1024", [](FetchRequest& q) {
             q.side_m = 1000.0;
             q.dem_cell_m = 0.5;
         }},
        {"an image over 4096", [](FetchRequest& q) {
             q.side_m = 1000.0;
             q.image_cell_m = 0.2;
         }},
        {"a zero cell", [](FetchRequest& q) { q.dem_cell_m = 0.0; }},
    };
    for (const Case& c : cases) {
        Rig rig;
        c.change(rig.request);
        const FetchResult r = rig.run();
        EXPECT_EQ(r.kind, FetchResultKind::refused) << c.what << ": " << r.message;
        EXPECT_FALSE(r.message.empty()) << c.what;
        EXPECT_TRUE(rig.http.urls.empty()) << c.what;
    }
}

TEST(GeoFetch, ACacheInsideAGitWorkTreeIsRefused) {  // REQ-G10
    Rig rig;
    fs::create_directories(rig.tmp.path() / "repo" / ".git");
    rig.request.cache_root = rig.tmp.path() / "repo" / "maps";
    const FetchResult r = rig.run();
    EXPECT_EQ(r.kind, FetchResultKind::refused);
    EXPECT_NE(r.message.find("git"), std::string::npos) << r.message;
    EXPECT_TRUE(rig.http.urls.empty());
    EXPECT_FALSE(fs::exists(rig.tmp.path() / "repo" / "maps"));
}

TEST(GeoFetch, AnEntryBiggerThanTheLimitIsCacheFullAndNotStored) {
    Rig rig;
    rig.request.cache_limit_bytes = 10;
    const FetchResult r = rig.run();
    EXPECT_EQ(r.kind, FetchResultKind::cache_full);
    EXPECT_FALSE(r.message.empty());
    EXPECT_FALSE(fs::exists(rig.entry()));
}

TEST(GeoFetch, AFetchTrimsTheOldestEntriesToStayInsideTheLimit) {
    Rig probe;
    ASSERT_EQ(probe.run().kind, FetchResultKind::fetched);
    const std::uint64_t one = cache_size_bytes(probe.request.cache_root);

    Rig rig;
    MapGeoref old;
    old.fetched_utc = "2026-11-01T00:00:00Z";
    old.key = "n1_e1_s1000";
    ASSERT_TRUE(store_entry(rig.request.cache_root, old, std::string(16, 'a'), std::string(8, 'b'), "o1").has_value());
    old.key = "n2_e2_s1000";
    old.fetched_utc = "2026-11-02T00:00:00Z";
    ASSERT_TRUE(store_entry(rig.request.cache_root, old, std::string(16, 'a'), std::string(8, 'b'), "o2").has_value());
    rig.request.cache_limit_bytes = one + 1;  // room for the new entry and no old one
    const FetchResult r = rig.run();
    ASSERT_EQ(r.kind, FetchResultKind::fetched) << r.message;
    EXPECT_FALSE(fs::exists(map_entry_dir(rig.request.cache_root, "n1_e1_s1000")));
    EXPECT_FALSE(fs::exists(map_entry_dir(rig.request.cache_root, "n2_e2_s1000")));
    EXPECT_TRUE(fs::exists(rig.entry()));
}

TEST(GeoFetch, ProgressNamesTheStagesInOrderWithTheBytesOfEachDownload) {
    Rig rig;
    ASSERT_EQ(rig.run().kind, FetchResultKind::fetched);
    std::vector<std::string> seq;
    for (const ProgressEvent& e : rig.events) {
        const char* state = e.state == ProgressEvent::State::start ? "start" : e.state == ProgressEvent::State::done ? "done" : "bytes";
        const std::string item = e.stage + ":" + state;
        if (seq.empty() || seq.back() != item) seq.push_back(item);
        if (e.state == ProgressEvent::State::bytes) {
            EXPECT_GT(e.total, 0u);
            EXPECT_LE(e.received, e.total);
        }
    }
    EXPECT_EQ(seq, (std::vector<std::string>{"geoid:start", "geoid:bytes", "geoid:done", "elevation:start", "elevation:bytes",
                                             "elevation:done", "image:start", "image:bytes", "image:done", "store:start",
                                             "store:done"}));
}

TEST(GeoFetch, TheSameAnswersGiveByteEqualFiles) {
    Rig a, b;
    ASSERT_EQ(a.run().kind, FetchResultKind::fetched);
    ASSERT_EQ(b.run().kind, FetchResultKind::fetched);
    for (const char* name : {"height.f32", "image.png", "georef.yaml"}) EXPECT_EQ(slurp(a.entry() / name), slurp(b.entry() / name)) << name;
}

TEST(GeoFetch, TheExitCodesAreTheDesignNotesTable) {
    EXPECT_EQ(exit_code_of(FetchResultKind::hit), 0);
    EXPECT_EQ(exit_code_of(FetchResultKind::fetched), 0);
    EXPECT_EQ(exit_code_of(FetchResultKind::miss), 3);
    EXPECT_EQ(exit_code_of(FetchResultKind::offline), 10);
    EXPECT_EQ(exit_code_of(FetchResultKind::service_error), 11);
    EXPECT_EQ(exit_code_of(FetchResultKind::refused), 12);
    EXPECT_EQ(exit_code_of(FetchResultKind::cache_full), 13);
    EXPECT_STREQ(name_of(FetchResultKind::service_error), "service_error");
    EXPECT_STREQ(name_of(FetchResultKind::hit), "hit");
}

}  // namespace
