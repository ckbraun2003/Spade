#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "../tools/geo_fetch/raster_formats.hpp"
#include "geo_fixtures.hpp"

// The two answers' formats (L85, SP-5): 3DEP's float32 TIFF and NAIP's PNG header. The fixtures are built here, byte by byte.

namespace {

using namespace spade::geofetch;
using namespace spade::geofetchtest;

std::vector<float> ramp(uint32_t n) {
    std::vector<float> v(n);
    for (uint32_t i = 0; i < n; ++i) v[i] = 1262.5f + 0.25f * static_cast<float>(i);
    return v;
}

TEST(GeoTiff, AnUncompressedFloatTiffReadsBackSampleForSample) {
    const std::vector<float> in = ramp(12);
    const auto g = read_tiff_f32(make_tiff(4, 3, in));
    ASSERT_TRUE(g.has_value()) << g.error().context;
    EXPECT_EQ(g->width, 4u);
    EXPECT_EQ(g->height, 3u);
    ASSERT_EQ(g->samples.size(), 12u);
    EXPECT_EQ(std::memcmp(g->samples.data(), in.data(), 12 * sizeof(float)), 0);
}

TEST(GeoTiff, SeveralStripsAndBothByteOrdersReadTheSame) {
    const std::vector<float> in = ramp(30);
    TiffOptions strips;
    strips.rows_per_strip = 2;  // 5 rows -> 3 strips
    const auto a = read_tiff_f32(make_tiff(6, 5, in, strips));
    ASSERT_TRUE(a.has_value()) << a.error().context;
    EXPECT_EQ(a->samples, in);
    TiffOptions be;
    be.big_endian = true;
    be.rows_per_strip = 3;
    const auto b = read_tiff_f32(make_tiff(6, 5, in, be));
    ASSERT_TRUE(b.has_value()) << b.error().context;
    EXPECT_EQ(b->samples, in);
}

TEST(GeoTiff, EachThingItDoesNotReadIsRefusedByName) {
    const std::vector<float> in = ramp(12);
    TiffOptions c;
    c.compression = 8;
    auto r = read_tiff_f32(make_tiff(4, 3, in, c));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("compress"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find('8'), std::string::npos) << r.error().context;

    TiffOptions bits;
    bits.bits = 16;
    bits.sample_format = 1;
    r = read_tiff_f32(make_tiff(4, 3, in, bits));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("float32"), std::string::npos) << r.error().context;

    TiffOptions bands;
    bands.samples_per_pixel = 3;
    r = read_tiff_f32(make_tiff(4, 3, in, bands));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("band"), std::string::npos) << r.error().context;

    TiffOptions tiled;
    tiled.tiled = true;
    r = read_tiff_f32(make_tiff(4, 3, in, tiled));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("tile"), std::string::npos) << r.error().context;

    TiffOptions cut;
    cut.truncate = true;
    EXPECT_FALSE(read_tiff_f32(make_tiff(4, 3, in, cut)).has_value());

    EXPECT_FALSE(read_tiff_f32("").has_value());
    EXPECT_FALSE(read_tiff_f32("<html>Service Unavailable</html>").has_value());
}

TEST(GeoPng, TheHeaderGivesSizeDepthAndColourType) {
    const auto h = read_png_header(make_png(1667, 1500, 8, 2));
    ASSERT_TRUE(h.has_value()) << h.error().context;
    EXPECT_EQ(h->width, 1667u);
    EXPECT_EQ(h->height, 1500u);
    EXPECT_EQ(h->bit_depth, 8);
    EXPECT_EQ(h->color_type, 2);
    const auto rgba = read_png_header(make_png(4, 4, 16, 6));
    ASSERT_TRUE(rgba.has_value());
    EXPECT_EQ(rgba->bit_depth, 16);
    EXPECT_EQ(rgba->color_type, 6);
}

TEST(GeoPng, ABodyThatIsNotAPngIsRefused) {
    EXPECT_FALSE(read_png_header("").has_value());
    EXPECT_FALSE(read_png_header("<?xml version=\"1.0\"?><error>no</error>").has_value());
    std::string cut = make_png(4, 4, 8, 2);
    cut.resize(20);
    EXPECT_FALSE(read_png_header(cut).has_value());
    std::string wrong = make_png(4, 4, 8, 2);
    wrong.replace(12, 4, "IDAT");
    EXPECT_FALSE(read_png_header(wrong).has_value());
}

}  // namespace
