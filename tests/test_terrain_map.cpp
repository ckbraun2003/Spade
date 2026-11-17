#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "../tools/geo_fetch/area.hpp"
#include "../tools/geo_fetch/fetch.hpp"
#include "geo_fixtures.hpp"
#include "unique_temp.hpp"
#include "world/map_cache.hpp"
#include "world/sha256.hpp"
#include "world/terrain_map.hpp"

// The cache reader (world/terrain_map.hpp; L85e part 2, the design note sections 1 and 4): an entry the fetch tool wrote, read back as
// the terrain's input already in metres from the anchor. The entries come from the tool itself on a fake network, so the two halves
// are tested against each other, and the damaged ones are made by changing what it wrote.

namespace {

namespace fs = std::filesystem;
using namespace spade;
using namespace spade::geofetch;
using namespace spade::geofetchtest;

constexpr double kLat = 40.005;
constexpr double kLon = -112.995;
constexpr double kSide = 40.0;

[[nodiscard]] std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void spit(const fs::path& p, const std::string& bytes) {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

// A cache the fetch tool filled: the fixture's 4 x 4 grid (1000 + 2 * column, plus N = -23.4) and 2 x 2 image over the geodesy's box.
struct Cache {
    test::TempDir tmp{"terrain_map"};
    fs::path root = tmp.path() / "maps";

    Cache() {
        FakeHttp http(FakeHttp::serve(GeoFixture{}));
        FetchEnv env{http, nullptr, [] { return std::string("2026-11-17T01:23:45Z"); }, [](int) {}, nullptr, "t1"};
        FetchRequest q;
        q.lat_deg = kLat;
        q.lon_deg = kLon;
        q.side_m = kSide;
        q.dem_cell_m = 10.0;
        q.image_cell_m = 20.0;
        q.cache_root = root;
        const FetchResult r = fetch_map(q, env);
        EXPECT_EQ(r.kind, FetchResultKind::fetched) << r.message;
    }

    [[nodiscard]] fs::path dir() const { return map_entry_dir(root, map_key(kLat, kLon, kSide)); }
    [[nodiscard]] Result<world::TerrainInput> read(double lat = kLat, double lon = kLon, double side = kSide) const {
        return read_terrain_map(root, lat, lon, side);
    }

    // Replaces the two files and writes a georef that agrees with them (their hashes), after `edit` has changed it.
    template <class Edit>
    void rewrite(const std::string& heights, const std::string& png, Edit edit) const {
        Result<MapGeoref> g = parse_georef(slurp(dir() / "georef.yaml"));
        ASSERT_TRUE(g.has_value());
        spit(dir() / "height.f32", heights);
        spit(dir() / "image.png", png);
        g->height_sha256 = sha256_hex(heights);
        g->image_sha256 = sha256_hex(png);
        edit(*g);
        spit(dir() / "georef.yaml", serialize_georef(*g));
    }
};

[[nodiscard]] std::string floats(const std::vector<float>& v) {
    std::string s(v.size() * 4, '\0');
    std::memcpy(s.data(), v.data(), s.size());
    return s;
}

void expect_invalid(const Result<world::TerrainInput>& r, const char* file, const char* says, const char* what) {
    ASSERT_FALSE(r.has_value()) << what;
    EXPECT_EQ(r.error().code, Code::schema_mismatch) << what << ": " << r.error().context;
    EXPECT_EQ(r.error().context.rfind(std::string("map_invalid: ") + file, 0), 0u) << what << ": " << r.error().context;
    EXPECT_NE(r.error().context.find(says), std::string::npos) << what << ": " << r.error().context;
}

TEST(TerrainMap, AnEntryTheToolWroteReadsBackInMetresFromTheAnchor) {
    Cache c;
    const Result<world::TerrainInput> r = c.read();
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->nx, 4u);
    EXPECT_EQ(r->ny, 4u);
    ASSERT_EQ(r->heights.size(), 16u);
    // The grid's middle (1003) plus N is the ground at the anchor; every sample is its height above that: 2 * column - 3.
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            EXPECT_NEAR(r->heights[row * 4 + col], 2.0 * col - 3.0, 2e-4) << row << "," << col;
        }
    }
    // The box is the geodesy's, +-20 m of the anchor, east and north, rows north to south. The tool asked for it at height 0 and the ground
    // is about 980 m up, where the same degrees are 980 / 6.37e6 = 0.016 % longer: 3 mm at 20 m, so the tolerance is a centimetre.
    EXPECT_NEAR(r->extent.east_min, -20.0, 1e-2);
    EXPECT_NEAR(r->extent.east_max, 20.0, 1e-2);
    EXPECT_NEAR(r->extent.north_min, -20.0, 1e-2);
    EXPECT_NEAR(r->extent.north_max, 20.0, 1e-2);
    // The image, decoded: 2 x 2 RGB, top row first.
    EXPECT_EQ(r->image.width, 2u);
    EXPECT_EQ(r->image.height, 2u);
    EXPECT_EQ(r->image.rgb, (std::vector<std::uint8_t>{10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120}));
}

TEST(TerrainMap, ReadingChangesNothingInTheCache) {
    Cache c;
    const auto listing = [&] {
        std::map<std::string, std::string> files;
        for (const auto& e : fs::recursive_directory_iterator(c.root)) {
            if (e.is_regular_file()) files[e.path().string()] = slurp(e.path());
        }
        return files;
    };
    const auto before = listing();
    ASSERT_FALSE(before.empty());
    ASSERT_TRUE(c.read().has_value());
    ASSERT_TRUE(c.read().has_value());
    EXPECT_EQ(listing(), before);
}

TEST(TerrainMap, TheExtentIsMeasuredFromTheCallersAnchorNotFromTheKey) {
    // 4e-7 degrees (0.044 m) north of the fetched anchor rounds to the same key; the georef's box is where it is, so it moves 0.044 m
    // south of the new anchor and east does not move.
    Cache c;
    const Result<world::TerrainInput> r = c.read(kLat + 4e-7, kLon, kSide);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_NEAR(r->extent.north_max, 20.0 - 0.0444, 1e-2);  // (a centimetre: the ground's height scales the degrees, as above)
    EXPECT_NEAR(r->extent.north_min, -20.0 - 0.0444, 1e-2);
    EXPECT_NEAR(r->extent.east_min, -20.0, 1e-2);
}

TEST(TerrainMap, NoEntryIsMapMissingNamingTheKeyAndTheDirectory) {
    test::TempDir tmp{"terrain_map_missing"};
    const Result<world::TerrainInput> r = read_terrain_map(tmp.path(), kLat, kLon, kSide);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::not_found);
    EXPECT_EQ(r.error().context.rfind("map_missing: ", 0), 0u) << r.error().context;
    EXPECT_NE(r.error().context.find(map_key(kLat, kLon, kSide)), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find(tmp.path().string()), std::string::npos) << r.error().context;
}

TEST(TerrainMap, AnEntryWithNoGeorefIsMissingNotInvalid) {  // georef.yaml is written last: without it the entry is not complete
    Cache c;
    fs::remove(c.dir() / "georef.yaml");
    const Result<world::TerrainInput> r = c.read();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::not_found);
    EXPECT_EQ(r.error().context.rfind("map_missing: ", 0), 0u) << r.error().context;
}

TEST(TerrainMap, AFileThatDoesNotMatchItsHashIsMapInvalidNamingTheFile) {
    {
        Cache c;
        std::string heights = slurp(c.dir() / "height.f32");
        heights[10] = static_cast<char>(heights[10] ^ 0x01);
        spit(c.dir() / "height.f32", heights);
        expect_invalid(c.read(), "height.f32", "sha256", "one bit of the heights");
    }
    {
        Cache c;
        std::string png = slurp(c.dir() / "image.png");
        png[png.size() / 2] = static_cast<char>(png[png.size() / 2] ^ 0x01);
        spit(c.dir() / "image.png", png);
        expect_invalid(c.read(), "image.png", "sha256", "one bit of the image");
    }
}

TEST(TerrainMap, AFileThatIsGoneIsMapInvalid) {
    {
        Cache c;
        fs::remove(c.dir() / "height.f32");
        expect_invalid(c.read(), "height.f32", "missing", "no heights");
    }
    {
        Cache c;
        fs::remove(c.dir() / "image.png");
        expect_invalid(c.read(), "image.png", "missing", "no image");
    }
}

TEST(TerrainMap, AGeorefThatCannotBeTrustedIsMapInvalid) {
    {
        Cache c;
        spit(c.dir() / "georef.yaml", "georef_version: 1\nkey: nonsense\n");
        expect_invalid(c.read(), "georef.yaml", "", "a georef missing its keys");
    }
    {
        Cache c;  // an entry copied under another area's name
        c.rewrite(slurp(c.dir() / "height.f32"), slurp(c.dir() / "image.png"), [](MapGeoref& g) { g.key = "n40005000_w112995000_s500"; });
        expect_invalid(c.read(), "georef.yaml", "key", "a georef that names another area");
    }
    {
        Cache c;  // heights above something other than the ellipsoid would put the terrain the wrong height
        c.rewrite(slurp(c.dir() / "height.f32"), slurp(c.dir() / "image.png"), [](MapGeoref& g) { g.vertical = "navd88_m"; });
        expect_invalid(c.read(), "georef.yaml", "wgs84_ellipsoid_m", "heights in another datum");
    }
}

TEST(TerrainMap, HeightsThatAreNotWhatTheGeorefSaysAreMapInvalidEvenWithAMatchingHash) {
    const std::string png = GeoFixture{}.png;
    {
        Cache c;
        std::vector<float> h(16, 1000.0f);
        h[5] = std::numeric_limits<float>::quiet_NaN();
        c.rewrite(floats(h), png, [](MapGeoref&) {});
        expect_invalid(c.read(), "height.f32", "finite", "a NaN height");
    }
    {
        Cache c;
        std::vector<float> h(16, 1000.0f);
        h[5] = std::numeric_limits<float>::infinity();
        c.rewrite(floats(h), png, [](MapGeoref&) {});
        expect_invalid(c.read(), "height.f32", "finite", "an infinite height");
    }
    {
        Cache c;
        c.rewrite(floats(std::vector<float>(15, 1000.0f)), png, [](MapGeoref&) {});
        expect_invalid(c.read(), "height.f32", "size", "15 samples for a 4 x 4 grid");
    }
    {
        Cache c;  // a grid the tool would never have written: over 1024 a side. Refused before the file is read
        c.rewrite(floats(std::vector<float>(16, 1000.0f)), png, [](MapGeoref& g) {
            g.elevation.width = 2000;
            g.elevation.height = 2000;
        });
        expect_invalid(c.read(), "georef.yaml", "1024", "a grid of 2000 x 2000");
    }
    {
        Cache c;  // a grid of one sample has no extent between samples
        c.rewrite(floats(std::vector<float>(1, 1000.0f)), png, [](MapGeoref& g) {
            g.elevation.width = 1;
            g.elevation.height = 1;
        });
        expect_invalid(c.read(), "georef.yaml", "grid", "a grid of 1 x 1");
    }
}

TEST(TerrainMap, AnImageDecodePngRefusesIsMapInvalidAndSaysWhy) {
    const std::string heights = floats(std::vector<float>(16, 1000.0f));
    {
        Cache c;
        c.rewrite(heights, make_png(2, 2, 16, 2, "x"), [](MapGeoref&) {});
        expect_invalid(c.read(), "image.png", "png:", "a 16-bit PNG");
    }
    {
        Cache c;
        c.rewrite(heights, "not a png", [](MapGeoref&) {});
        expect_invalid(c.read(), "image.png", "png:", "not a PNG");
    }
    {
        Cache c;  // decodes, but is not the size the georef says
        c.rewrite(heights, make_rgb_png(3, 3, std::vector<std::uint8_t>(27, 7)), [](MapGeoref&) {});
        expect_invalid(c.read(), "image.png", "3 x 3", "a 3 x 3 image for a 2 x 2 georef");
    }
}

TEST(TerrainMap, AnAnchorOrASideThatIsNotValidIsInvalidArgument) {
    Cache c;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    struct Case {
        const char* what;
        double lat, lon, side;
    };
    const Case cases[] = {
        {"NaN latitude", nan, kLon, kSide}, {"latitude 89.5", 89.5, kLon, kSide}, {"longitude 181", kLat, 181.0, kSide},
        {"side 5", kLat, kLon, 5.0},        {"side 1001", kLat, kLon, 1001.0},    {"NaN side", kLat, kLon, nan},
    };
    for (const Case& k : cases) {
        const Result<world::TerrainInput> r = c.read(k.lat, k.lon, k.side);
        ASSERT_FALSE(r.has_value()) << k.what;
        EXPECT_EQ(r.error().code, Code::invalid_argument) << k.what << ": " << r.error().context;
    }
}

TEST(TerrainMap, TheHeaderNamesItsLinkTargetAndPullsInNeitherMeshImportNorMiniz) {
    // A world-only consumer includes this header: it must not drag mesh_import.hpp or miniz in, and it must say why linking needs
    // spade::mesh_import (the definition is in that archive). The pointer back from image8.hpp names the reader.
    const std::string header = slurp(fs::path(SPADE_ENGINE_DIR) / "world" / "terrain_map.hpp");
    ASSERT_FALSE(header.empty());
    EXPECT_EQ(header.find("#include \"world/mesh_import.hpp\""), std::string::npos);
    EXPECT_EQ(header.find("miniz"), std::string::npos);
    EXPECT_NE(header.find("spade::mesh_import"), std::string::npos);
    EXPECT_NE(header.find("terrain_input.hpp"), std::string::npos);
    const std::string image8 = slurp(fs::path(SPADE_ENGINE_DIR) / "world" / "image8.hpp");
    EXPECT_NE(image8.find("read_terrain_map"), std::string::npos);
}

// ---- the real entry ------------------------------------------------------------------------------------------------------------

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // std::getenv: read once, on the test's thread, before anything else runs
#endif
[[nodiscard]] const char* real_env(const char* name) { return std::getenv(name); }
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

// The map the live check fetched (L85g: Bonneville, 40.76 -113.90, 1 km, 3DEP and NAIP and NGS), read by the reader as the engine would.
// It needs the real cache: SPADE_MAP_CACHE names it, else the tool's default root. Without the entry the test SKIPS, naming how to make
// it (a skip is not a pass). The numbers are what the live check recorded: a 1000 x 1000 grid at 1 m, a 1667 x 1667 image, heights over
// the flats within a metre from the 1st to the 99th percentile (1.54 m max to min: a few outliers, not the surface).
TEST(TerrainMapReal, TheBonnevilleEntryTheLiveCheckWroteReadsBackAsMetresFromTheAnchor) {
    constexpr double kBonnevilleLat = 40.76;
    constexpr double kBonnevilleLon = -113.90;
    constexpr double kBonnevilleSide = 1000.0;
#if defined(_WIN32)
    constexpr MapPlatform kPlatform = MapPlatform::windows;
#else
    constexpr MapPlatform kPlatform = MapPlatform::posix;
#endif
    const char* named = real_env("SPADE_MAP_CACHE");
    const fs::path root = (named != nullptr && *named != '\0') ? fs::path(named) : default_map_cache_root(real_env, kPlatform);
    const std::string key = map_key(kBonnevilleLat, kBonnevilleLon, kBonnevilleSide);
    if (root.empty() || !fs::is_regular_file(map_entry_dir(root, key) / "georef.yaml")) {
        GTEST_SKIP() << "no Bonneville map (" << key << ") in the cache '" << root.string()
                     << "': run spade_geo_fetch --lat 40.76 --lon -113.90 --side 1000 --cache <dir>, then set SPADE_MAP_CACHE to <dir>";
    }
    const Result<world::TerrainInput> r = read_terrain_map(root, kBonnevilleLat, kBonnevilleLon, kBonnevilleSide);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->nx, 1000u);
    EXPECT_EQ(r->ny, 1000u);
    ASSERT_EQ(r->heights.size(), 1000u * 1000u);
    EXPECT_EQ(r->image.width, 1667u);
    EXPECT_EQ(r->image.height, 1667u);
    ASSERT_EQ(r->image.rgb.size(), 1667u * 1667u * 3u);
    // The box is 1 km square about the anchor; read at the ground's height (about 1267 m up) it is 0.02 % longer than at the ellipsoid.
    EXPECT_NEAR(r->extent.east_min, -500.0, 0.5);
    EXPECT_NEAR(r->extent.east_max, 500.0, 0.5);
    EXPECT_NEAR(r->extent.north_min, -500.0, 0.5);
    EXPECT_NEAR(r->extent.north_max, 500.0, 0.5);
    // Heights are metres above the anchor's ground: finite, and the ground at the anchor is the grid's middle.
    bool all_finite = true;
    for (const float h : r->heights) all_finite = all_finite && std::isfinite(h);
    ASSERT_TRUE(all_finite);
    EXPECT_NEAR(r->heights[500u * 1000u + 500u], 0.0f, 1.0f);
    std::vector<float> sorted = r->heights;
    std::sort(sorted.begin(), sorted.end());
    const float p1 = sorted[sorted.size() / 100];
    const float p99 = sorted[sorted.size() - 1 - sorted.size() / 100];
    EXPECT_LT(p99 - p1, 1.0f) << "the flats' 1st to 99th percentile spread";
    // The image is a picture, not a flat colour.
    bool varies = false;
    for (std::size_t i = 3; i + 2 < r->image.rgb.size() && !varies; i += 3) {
        varies = r->image.rgb[i] != r->image.rgb[0] || r->image.rgb[i + 1] != r->image.rgb[1] || r->image.rgb[i + 2] != r->image.rgb[2];
    }
    EXPECT_TRUE(varies);
}

}  // namespace
