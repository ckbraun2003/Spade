#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../tools/geo_fetch/geo_services.hpp"

// The three services' request URLs and the geoid answer (L85, SP-5; the design note, section 7). The URLs are written from the
// services' documentation and are UNVERIFIED until the live check; these tests pin what the tool sends, not what the services
// accept.

namespace {

namespace fs = std::filesystem;
using namespace spade::geofetch;

const Bbox kArea{-113.9058765, 40.7555030, -113.8941235, 40.7644970};

TEST(GeoServices, TheElevationRequestAsksFor3depAFloatTiffOverTheArea) {
    EXPECT_EQ(elevation_url(kArea, 1001, 1001),
              "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
              "?bbox=-113.905876500,40.755503000,-113.894123500,40.764497000&bboxSR=4326&imageSR=4326&size=1001,1001"
              "&format=tiff&pixelType=F32&interpolation=RSP_BilinearInterpolation&noData=-9999&f=image");
}

TEST(GeoServices, TheImageRequestAsksForNaipAnRgbPngOverTheArea) {
    EXPECT_EQ(image_url(kArea, 1667, 1667),
              "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer/exportImage"
              "?bbox=-113.905876500,40.755503000,-113.894123500,40.764497000&bboxSR=4326&imageSR=4326&size=1667,1667"
              "&format=png&interpolation=RSP_BilinearInterpolation&bandIds=0,1,2&f=image");
}

TEST(GeoServices, TheGeoidRequestAsksNgsForGeoid18AtTheAnchor) {
    EXPECT_EQ(geoid_url(40.76, -113.9), "https://geodesy.noaa.gov/api/geoid/ght?lat=40.760000000&lon=-113.900000000&model=18");
}

TEST(GeoServices, EveryUrlNamesOneOfTheThreeHosts) {
    const std::string urls[] = {elevation_url(kArea, 10, 10), image_url(kArea, 10, 10), geoid_url(1.0, 2.0)};
    for (const std::string& u : urls) {
        ASSERT_EQ(u.rfind("https://", 0), 0u) << u;
        const std::string host = u.substr(8, u.find('/', 8) - 8);
        EXPECT_NE(std::find(kServiceHosts.begin(), kServiceHosts.end(), host), kServiceHosts.end()) << host;
    }
}

// REQ-G08 and REQ-G09: no other host. Every host after "://" in the tool's own sources is one of the three.
TEST(GeoServices, NoSourceFileOfTheToolNamesAnotherHost) {
    const fs::path dir = (fs::path(SPADE_TESTS_DIR) / ".." / "tools" / "geo_fetch").lexically_normal();
    ASSERT_TRUE(fs::is_directory(dir)) << dir;
    std::size_t files = 0, urls = 0;
    for (const auto& entry : fs::directory_iterator(dir)) {
        const std::string ext = entry.path().extension().string();
        if (!entry.is_regular_file() || (ext != ".cpp" && ext != ".hpp")) continue;
        ++files;
        std::ifstream in(entry.path());
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        for (std::size_t at = text.find("://"); at != std::string::npos; at = text.find("://", at + 3)) {
            const std::size_t begin = at + 3;
            std::size_t end = begin;
            while (end < text.size() && (std::isalnum(static_cast<unsigned char>(text[end])) || text[end] == '.' || text[end] == '-')) {
                ++end;
            }
            const std::string host = text.substr(begin, end - begin);
            if (host.empty()) continue;  // a format string or a pattern, not a host
            ++urls;
            EXPECT_NE(std::find(kServiceHosts.begin(), kServiceHosts.end(), host), kServiceHosts.end())
                << host << " in " << entry.path().filename();
        }
    }
    EXPECT_GE(files, 3u);
    EXPECT_GE(urls, 3u);  // the three services' own URLs were seen
}

TEST(GeoServices, TheGeoidAnswerGivesTheHeightAndTheModel) {
    const auto r = parse_geoid_json(R"({"geoidModel":"GEOID18","latitude":40.76,"longitude":-113.9,"geoidHeight":-23.412})");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->height_m, -23.412);
    EXPECT_EQ(r->model, "GEOID18");
}

TEST(GeoServices, TheGeoidAnswerWithoutAModelKeyIsTakenAsGeoid18) {
    const auto r = parse_geoid_json(R"({ "geoidHeight" : -24.5 })");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->height_m, -24.5);
    EXPECT_EQ(r->model, "GEOID18");
}

TEST(GeoServices, AGeoidAnswerThatIsNotUsableIsNamed) {
    const char* bad[] = {"",
                         "not json",
                         "{}",
                         R"({"geoidHeight":"x"})",
                         R"({"geoidHeight":null})",
                         R"({"geoidModel":"GEOID12B","geoidHeight":-23.4})",
                         R"({"geoidHeight":1e999})"};
    for (const char* b : bad) {
        const auto r = parse_geoid_json(b);
        ASSERT_FALSE(r.has_value()) << b;
        EXPECT_EQ(r.error().code, spade::Code::schema_mismatch) << b;
        EXPECT_FALSE(r.error().context.empty()) << b;
    }
}

}  // namespace
