#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include "unique_temp.hpp"
#include "world/map_cache.hpp"

// The geo map cache's layout (L85, geo G2, SP-5; the design note section 4): the key, the default root, the "never inside a
// git work tree" guard, and the georef file's round trip.

namespace {

namespace fs = std::filesystem;
using namespace spade;

TEST(MapCache, TheKeyNamesTheAnchorAndTheSide) {
    EXPECT_EQ(map_key(40.76, -113.90, 1000.0), "n40760000_w113900000_s1000");
    EXPECT_EQ(map_key(-33.8568, 151.2153, 500.0), "s33856800_e151215300_s500");
    EXPECT_EQ(map_key(0.0, 0.0, 10.0), "n0_e0_s10");
}

TEST(MapCache, TwoAnchorsWithinARoundingStepShareAKeyAndFarOnesDoNot) {
    EXPECT_EQ(map_key(40.7600001, -113.9000004, 1000.0), map_key(40.76, -113.90, 1000.0));
    EXPECT_NE(map_key(40.760001, -113.90, 1000.0), map_key(40.76, -113.90, 1000.0));
    EXPECT_NE(map_key(40.76, -113.90, 999.0), map_key(40.76, -113.90, 1000.0));
}

TEST(MapCache, AnEntryLivesUnderV1AndItsKey) {
    EXPECT_EQ(map_entry_dir(fs::path("root"), "n1_e2_s3"), fs::path("root") / "v1" / "n1_e2_s3");
}

TEST(MapCache, TheDefaultRootFollowsTheEnvironment) {
    std::map<std::string, std::string> vars;
    const EnvLookup env = [&](const char* name) -> const char* {
        const auto it = vars.find(name);
        return it == vars.end() ? nullptr : it->second.c_str();
    };
    EXPECT_TRUE(default_map_cache_root(env, MapPlatform::windows).empty());
    EXPECT_TRUE(default_map_cache_root(env, MapPlatform::posix).empty());

    vars["LOCALAPPDATA"] = "C:/Users/t/AppData/Local";
    EXPECT_EQ(default_map_cache_root(env, MapPlatform::windows), fs::path("C:/Users/t/AppData/Local") / "Spade" / "maps");

    vars["HOME"] = "/home/t";
    EXPECT_EQ(default_map_cache_root(env, MapPlatform::posix), fs::path("/home/t") / ".cache" / "spade" / "maps");
    vars["XDG_CACHE_HOME"] = "/var/cache/t";
    EXPECT_EQ(default_map_cache_root(env, MapPlatform::posix), fs::path("/var/cache/t") / "spade" / "maps");

    vars["SPADE_MAP_CACHE"] = "/data/maps";  // names the root for both
    EXPECT_EQ(default_map_cache_root(env, MapPlatform::windows), fs::path("/data/maps"));
    EXPECT_EQ(default_map_cache_root(env, MapPlatform::posix), fs::path("/data/maps"));
}

TEST(MapCache, ADirectoryInsideAGitWorkTreeIsRefusedWhetherOrNotItExistsYet) {
    const spade::test::TempDir tmp("map_cache_git");
    const fs::path base = tmp.path();
    fs::create_directories(base / "repo" / ".git");
    fs::create_directories(base / "plain");
    EXPECT_TRUE(inside_git_work_tree(base / "repo"));
    EXPECT_TRUE(inside_git_work_tree(base / "repo" / "maps"));              // not created yet
    EXPECT_TRUE(inside_git_work_tree(base / "repo" / "deep" / "er" / "maps"));
    EXPECT_FALSE(inside_git_work_tree(base / "plain"));
    EXPECT_FALSE(inside_git_work_tree(base / "plain" / "maps"));
}

[[nodiscard]] MapGeoref sample_georef() {
    MapGeoref g;
    g.key = "n40760000_w113900000_s1000";
    g.anchor_lat_deg = 40.76;
    g.anchor_lon_deg = -113.9;
    g.side_m = 1000.0;
    g.west_deg = -113.90587654321;
    g.south_deg = 40.755503;
    g.east_deg = -113.89412345679;
    g.north_deg = 40.764497;
    g.elevation = {"height.f32", 1001, 1001, 1.0};
    g.image = {"image.png", 1667, 1667, 0.6};
    g.geoid_model = "GEOID18";
    g.geoid_height_m = -23.412;
    g.anchor_ground_ellipsoid_height_m = 1262.7;
    g.height_sha256 = std::string(64, 'a');
    g.image_sha256 = std::string(64, 'b');
    g.elevation_source = {"USGS 3DEP", "https://elevation.example/exportImage?x=1&y=2", "2026-11-17T01:23:45Z", ""};
    g.image_source = {"USGS NAIP", "https://imagery.example/exportImage?x=1", "2026-11-17T01:23:46Z", "2023-06-15"};
    g.geoid_source = {"NOAA NGS", "https://geodesy.example/api/geoid/ght?lat=40.76", "2026-11-17T01:23:44Z", ""};
    g.datum_note = "heights are NAVD88 plus the GEOID18 height, whose ellipsoid is NAD83(2011), not WGS84";
    g.fetched_utc = "2026-11-17T01:23:47Z";
    return g;
}

TEST(MapCache, AGeorefRoundTripsExactlyAndSerializesTheSameTwice) {
    const MapGeoref g = sample_georef();
    const std::string text = serialize_georef(g);
    EXPECT_EQ(text, serialize_georef(g));
    const Result<MapGeoref> back = parse_georef(text);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->key, g.key);
    EXPECT_EQ(back->anchor_lat_deg, g.anchor_lat_deg);
    EXPECT_EQ(back->anchor_lon_deg, g.anchor_lon_deg);
    EXPECT_EQ(back->west_deg, g.west_deg);  // exact: the text keeps the shortest digits that read back equal
    EXPECT_EQ(back->north_deg, g.north_deg);
    EXPECT_EQ(back->elevation.width, 1001u);
    EXPECT_EQ(back->image.cell_m, 0.6);
    EXPECT_EQ(back->geoid_height_m, g.geoid_height_m);
    EXPECT_EQ(back->anchor_ground_ellipsoid_height_m, g.anchor_ground_ellipsoid_height_m);
    EXPECT_EQ(back->height_sha256, g.height_sha256);
    EXPECT_EQ(back->image_source.acquired, "2023-06-15");
    EXPECT_EQ(back->elevation_source.acquired, "");
    EXPECT_EQ(back->datum_note, g.datum_note);
    EXPECT_EQ(serialize_georef(*back), text);
}

TEST(MapCache, AGeorefMissingAKeyOrOfAnotherVersionIsInvalidAndNamesIt) {
    std::string text = serialize_georef(sample_georef());
    const std::size_t at = text.find("geoid:");
    ASSERT_NE(at, std::string::npos);
    const std::string without = text.substr(0, at);  // the tail (geoid, anchor_ground, hashes, ...) is gone
    const Result<MapGeoref> r = parse_georef(without);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::schema_mismatch);
    EXPECT_EQ(r.error().context.rfind("map_invalid: georef.yaml: ", 0), 0u) << r.error().context;
    EXPECT_NE(r.error().context.find("geoid"), std::string::npos) << r.error().context;

    std::string other = text;
    other.replace(other.find("georef_version: 1"), 17, "georef_version: 2");
    const Result<MapGeoref> v = parse_georef(other);
    ASSERT_FALSE(v.has_value());
    EXPECT_NE(v.error().context.find("georef_version"), std::string::npos) << v.error().context;

    EXPECT_FALSE(parse_georef("not: [yaml").has_value());
    EXPECT_FALSE(parse_georef("").has_value());
}

}  // namespace
