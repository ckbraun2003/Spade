#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "../tools/geo_fetch/raster_formats.hpp"

// The two answers' formats (L85, SP-5): 3DEP's float32 TIFF and NAIP's PNG header. The fixtures are built here, byte by byte.

namespace {

using namespace spade::geofetch;

struct TiffOptions {
    bool big_endian = false;
    uint16_t compression = 1;
    uint16_t bits = 32;
    uint16_t sample_format = 3;  // IEEE float
    uint16_t samples_per_pixel = 1;
    uint32_t rows_per_strip = 0;  // 0: one strip
    bool tiled = false;
    bool truncate = false;
};

void put16(std::string& s, uint16_t v, bool be) {
    if (be) {
        s.push_back(static_cast<char>(v >> 8));
        s.push_back(static_cast<char>(v & 0xff));
    } else {
        s.push_back(static_cast<char>(v & 0xff));
        s.push_back(static_cast<char>(v >> 8));
    }
}

void put32(std::string& s, uint32_t v, bool be) {
    if (be) {
        for (int i = 3; i >= 0; --i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    } else {
        for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    }
}

// A TIFF of `w * h` float32 samples: header, the strips' bytes, the strip offset and count arrays, then the IFD.
std::string make_tiff(uint32_t w, uint32_t h, const std::vector<float>& samples, const TiffOptions& o = {}) {
    const bool be = o.big_endian;
    std::string s = be ? "MM" : "II";
    put16(s, 42, be);
    put32(s, 0, be);  // the IFD offset, patched below
    const uint32_t rps = o.rows_per_strip == 0 ? h : o.rows_per_strip;
    std::vector<uint32_t> offsets, counts;
    for (uint32_t row = 0; row < h; row += rps) {
        const uint32_t rows = std::min(rps, h - row);
        offsets.push_back(static_cast<uint32_t>(s.size()));
        counts.push_back(rows * w * 4);
        for (uint32_t i = 0; i < rows * w; ++i) put32(s, std::bit_cast<uint32_t>(samples[row * w + i]), be);
    }
    const uint32_t offsets_at = static_cast<uint32_t>(s.size());
    for (uint32_t v : offsets) put32(s, v, be);
    const uint32_t counts_at = static_cast<uint32_t>(s.size());
    for (uint32_t v : counts) put32(s, v, be);
    const uint32_t ifd_at = static_cast<uint32_t>(s.size());
    struct Entry {
        uint16_t tag, type;
        uint32_t count, value;  // a SHORT value sits in the first two bytes of the value field
    };
    const uint32_t n = static_cast<uint32_t>(offsets.size());
    std::vector<Entry> e = {{256, 4, 1, w},
                            {257, 4, 1, h},
                            {258, 3, 1, o.bits},
                            {259, 3, 1, o.compression},
                            {262, 3, 1, 1},
                            {273, 4, n, n == 1 ? offsets[0] : offsets_at},
                            {277, 3, 1, o.samples_per_pixel},
                            {278, 4, 1, rps},
                            {279, 4, n, n == 1 ? counts[0] : counts_at},
                            {339, 3, 1, o.sample_format}};
    if (o.tiled) e.push_back({322, 4, 1, 256});
    put16(s, static_cast<uint16_t>(e.size()), be);
    for (const Entry& x : e) {
        put16(s, x.tag, be);
        put16(s, x.type, be);
        put32(s, x.count, be);
        if (x.type == 3) {
            put16(s, static_cast<uint16_t>(x.value), be);
            put16(s, 0, be);
        } else {
            put32(s, x.value, be);
        }
    }
    put32(s, 0, be);  // no next IFD
    for (int i = 0; i < 4; ++i) s[4 + i] = static_cast<char>(be ? (ifd_at >> (8 * (3 - i))) & 0xff : (ifd_at >> (8 * i)) & 0xff);
    if (o.truncate) s.resize(s.size() / 2);
    return s;
}

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

std::string make_png_header(uint32_t w, uint32_t h, int depth, int color) {
    std::string s = "\x89PNG\r\n\x1a\n";
    const auto be32 = [&](uint32_t v) {
        for (int i = 3; i >= 0; --i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    };
    be32(13);
    s += "IHDR";
    be32(w);
    be32(h);
    s.push_back(static_cast<char>(depth));
    s.push_back(static_cast<char>(color));
    s.push_back(0);
    s.push_back(0);
    s.push_back(0);
    be32(0);  // the CRC is not checked here
    return s;
}

TEST(GeoPng, TheHeaderGivesSizeDepthAndColourType) {
    const auto h = read_png_header(make_png_header(1667, 1500, 8, 2));
    ASSERT_TRUE(h.has_value()) << h.error().context;
    EXPECT_EQ(h->width, 1667u);
    EXPECT_EQ(h->height, 1500u);
    EXPECT_EQ(h->bit_depth, 8);
    EXPECT_EQ(h->color_type, 2);
    const auto rgba = read_png_header(make_png_header(4, 4, 16, 6));
    ASSERT_TRUE(rgba.has_value());
    EXPECT_EQ(rgba->bit_depth, 16);
    EXPECT_EQ(rgba->color_type, 6);
}

TEST(GeoPng, ABodyThatIsNotAPngIsRefused) {
    EXPECT_FALSE(read_png_header("").has_value());
    EXPECT_FALSE(read_png_header("<?xml version=\"1.0\"?><error>no</error>").has_value());
    std::string cut = make_png_header(4, 4, 8, 2);
    cut.resize(20);
    EXPECT_FALSE(read_png_header(cut).has_value());
    std::string wrong = make_png_header(4, 4, 8, 2);
    wrong.replace(12, 4, "IDAT");
    EXPECT_FALSE(read_png_header(wrong).has_value());
}

}  // namespace
