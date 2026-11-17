#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "../tools/geo_fetch/cache_store.hpp"
#include "unique_temp.hpp"
#include "world/sha256.hpp"

// Writing the map cache and keeping it inside its limit (L85, SP-5; the design note, section 4).

namespace {

namespace fs = std::filesystem;
using namespace spade;
using namespace spade::geofetch;

[[nodiscard]] std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

[[nodiscard]] MapGeoref georef_for(const std::string& key, const std::string& fetched_utc) {
    MapGeoref g;
    g.key = key;
    g.anchor_lat_deg = 40.76;
    g.anchor_lon_deg = -113.9;
    g.side_m = 1000.0;
    g.elevation = {"height.f32", 2, 2, 1.0};
    g.image = {"image.png", 4, 4, 0.6};
    g.geoid_model = "GEOID18";
    g.geoid_height_m = -23.4;
    g.fetched_utc = fetched_utc;
    return g;
}

const std::string kHeights(16, 'h');  // 4 floats' worth of bytes
const std::string kImage(100, 'i');

TEST(GeoCacheStore, AnEntryIsWrittenUnderV1WithItsHashesAndNoTemporaryLeft) {
    const test::TempDir tmp("geo_store");
    const auto r = store_entry(tmp.path(), georef_for("n1_e2_s1000", "2026-11-17T00:00:01Z"), kHeights, kImage, "t1");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_TRUE(r->stored);
    EXPECT_EQ(r->dir, tmp.path() / "v1" / "n1_e2_s1000");
    EXPECT_TRUE(entry_complete(r->dir));
    EXPECT_EQ(slurp(r->dir / "height.f32"), kHeights);
    EXPECT_EQ(slurp(r->dir / "image.png"), kImage);
    const auto g = parse_georef(slurp(r->dir / "georef.yaml"));
    ASSERT_TRUE(g.has_value()) << g.error().context;
    EXPECT_EQ(g->height_sha256, sha256_hex(kHeights));
    EXPECT_EQ(g->image_sha256, sha256_hex(kImage));
    EXPECT_EQ(g->key, "n1_e2_s1000");
    std::size_t entries = 0;
    for (const auto& e : fs::directory_iterator(tmp.path() / "v1")) {
        ++entries;
        EXPECT_EQ(e.path().filename().string().rfind(".tmp-", 0), std::string::npos) << e.path();
    }
    EXPECT_EQ(entries, 1u);
}

TEST(GeoCacheStore, ACompleteEntryAlreadyThereIsKeptAndTheNewOneDiscarded) {
    const test::TempDir tmp("geo_store");
    ASSERT_TRUE(store_entry(tmp.path(), georef_for("n1_e2_s1000", "2026-11-17T00:00:01Z"), kHeights, kImage, "t1").has_value());
    const auto again = store_entry(tmp.path(), georef_for("n1_e2_s1000", "2026-11-17T00:00:09Z"), std::string(16, 'X'), kImage, "t2");
    ASSERT_TRUE(again.has_value()) << again.error().context;
    EXPECT_FALSE(again->stored);
    EXPECT_EQ(slurp(again->dir / "height.f32"), kHeights);  // the first one's bytes
    for (const auto& e : fs::directory_iterator(tmp.path() / "v1")) {
        EXPECT_EQ(e.path().filename().string().rfind(".tmp-", 0), std::string::npos) << e.path();
    }
}

TEST(GeoCacheStore, AnIncompleteDirectoryAtTheKeyIsReplaced) {
    const test::TempDir tmp("geo_store");
    const fs::path dir = tmp.path() / "v1" / "n1_e2_s1000";
    fs::create_directories(dir);
    std::ofstream(dir / "height.f32", std::ios::binary) << "half-written";
    EXPECT_FALSE(entry_complete(dir));
    const auto r = store_entry(tmp.path(), georef_for("n1_e2_s1000", "2026-11-17T00:00:01Z"), kHeights, kImage, "t1");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_TRUE(r->stored);
    EXPECT_TRUE(entry_complete(dir));
    EXPECT_EQ(slurp(dir / "height.f32"), kHeights);
}

TEST(GeoCacheStore, TheCacheSizeCountsEveryFileOfTheEntriesAndNotTheTemporaries) {
    const test::TempDir tmp("geo_store");
    EXPECT_EQ(cache_size_bytes(tmp.path()), 0u);  // nothing there yet
    const auto r = store_entry(tmp.path(), georef_for("n1_e2_s1000", "2026-11-17T00:00:01Z"), kHeights, kImage, "t1");
    ASSERT_TRUE(r.has_value());
    const std::uint64_t one = kHeights.size() + kImage.size() + fs::file_size(r->dir / "georef.yaml");
    EXPECT_EQ(cache_size_bytes(tmp.path()), one);
    fs::create_directories(tmp.path() / "v1" / ".tmp-9-x");
    std::ofstream(tmp.path() / "v1" / ".tmp-9-x" / "height.f32", std::ios::binary) << std::string(500, 'z');
    EXPECT_EQ(cache_size_bytes(tmp.path()), one);
}

TEST(GeoCacheStore, TrimRemovesTheOldestFetchedFirstAndNeverTheKeptOne) {
    const test::TempDir tmp("geo_store");
    ASSERT_TRUE(store_entry(tmp.path(), georef_for("n1_e1_s1000", "2026-11-17T00:00:01Z"), kHeights, kImage, "a").has_value());
    ASSERT_TRUE(store_entry(tmp.path(), georef_for("n2_e2_s1000", "2026-11-17T00:00:02Z"), kHeights, kImage, "b").has_value());
    ASSERT_TRUE(store_entry(tmp.path(), georef_for("n3_e3_s1000", "2026-11-17T00:00:03Z"), kHeights, kImage, "c").has_value());
    const std::uint64_t each = cache_size_bytes(tmp.path()) / 3;
    ASSERT_GT(each, 0u);

    // Room for two entries: the oldest goes.
    TrimResult t = trim_cache(tmp.path(), 2 * each + 10, "n3_e3_s1000");
    ASSERT_EQ(t.removed.size(), 1u);
    EXPECT_EQ(t.removed[0], "n1_e1_s1000");
    EXPECT_LE(t.total_after, 2 * each + 10);
    EXPECT_FALSE(fs::exists(tmp.path() / "v1" / "n1_e1_s1000"));
    EXPECT_TRUE(fs::exists(tmp.path() / "v1" / "n2_e2_s1000"));

    // Room for none: everything but the kept (newest) entry goes, and the kept one stays although it is over the limit.
    t = trim_cache(tmp.path(), 1, "n2_e2_s1000");
    ASSERT_EQ(t.removed.size(), 1u);
    EXPECT_EQ(t.removed[0], "n3_e3_s1000");
    EXPECT_TRUE(fs::exists(tmp.path() / "v1" / "n2_e2_s1000"));
    EXPECT_GT(t.total_after, 1u);
}

TEST(GeoCacheStore, TrimRemovesIncompleteEntriesAndAnUnreadableGeorefCountsAsOldest) {
    const test::TempDir tmp("geo_store");
    ASSERT_TRUE(store_entry(tmp.path(), georef_for("n1_e1_s1000", "2026-11-17T00:00:05Z"), kHeights, kImage, "a").has_value());
    ASSERT_TRUE(store_entry(tmp.path(), georef_for("n2_e2_s1000", "2026-11-17T00:00:06Z"), kHeights, kImage, "b").has_value());
    // n1's georef is damaged (still present): it sorts as the oldest.
    std::ofstream(tmp.path() / "v1" / "n1_e1_s1000" / "georef.yaml", std::ios::binary | std::ios::trunc) << "garbage: [";
    // An incomplete directory: no georef at all.
    fs::create_directories(tmp.path() / "v1" / "n9_e9_s1000");
    std::ofstream(tmp.path() / "v1" / "n9_e9_s1000" / "height.f32", std::ios::binary) << "x";
    const std::uint64_t total = cache_size_bytes(tmp.path());
    const TrimResult t = trim_cache(tmp.path(), total - 2, "n2_e2_s1000");
    ASSERT_EQ(t.removed.size(), 2u);
    EXPECT_EQ(t.removed[0], "n9_e9_s1000");  // the incomplete one first
    EXPECT_EQ(t.removed[1], "n1_e1_s1000");  // then the unreadable one, which counts as the oldest
    EXPECT_TRUE(fs::exists(tmp.path() / "v1" / "n2_e2_s1000"));
}

TEST(GeoCacheStore, TrimOnAnEmptyOrAbsentCacheDoesNothing) {
    const test::TempDir tmp("geo_store");
    const TrimResult t = trim_cache(tmp.path() / "nothing", 10, "");
    EXPECT_TRUE(t.removed.empty());
    EXPECT_EQ(t.total_after, 0u);
}

}  // namespace
