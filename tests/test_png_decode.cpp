#include <gtest/gtest.h>

#include <miniz.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "world/mesh_import.hpp"

// decode_png (L85e; Rendering's decoder table in docs/design/rendering/plans/2026-11-17-geo-terrain-design.md): 8-bit grey, RGB and
// RGBA, non-interlaced, all five filters, ancillary chunks ignored, every chunk's CRC checked. Refused by name: JPEG, 16-bit,
// palette, grey with alpha, interlaced, a bad CRC, truncation, a size bomb, a dimension that overflows. The PNGs are assembled
// here: miniz's tdefl compresses, this file's own forward filters and chunk writer do the rest, and three small PNGs encoded by
// Python's zlib (an independent encoder) pin the decoder against bytes this file did not produce. No image is committed.

namespace {

using spade::Code;

std::span<const std::byte> bytes_of(const std::string& s) { return std::as_bytes(std::span(s.data(), s.size())); }

// ---- the encoder ---------------------------------------------------------------------------------------------------------------

std::string be32(uint32_t v) {
    std::string s;
    for (int i = 3; i >= 0; --i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    return s;
}

std::string chunk(const std::string& type, const std::string& data, bool bad_crc = false) {
    uint32_t crc = static_cast<uint32_t>(mz_crc32(MZ_CRC32_INIT, reinterpret_cast<const unsigned char*>(type.data()), type.size()));
    crc = static_cast<uint32_t>(mz_crc32(crc, reinterpret_cast<const unsigned char*>(data.data()), data.size()));
    if (bad_crc) crc ^= 0x1u;
    return be32(static_cast<uint32_t>(data.size())) + type + data + be32(crc);
}

int paeth(int a, int b, int c) {
    const int p = a + b - c;
    const int pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
    return pa <= pb && pa <= pc ? a : (pb <= pc ? b : c);
}

// The forward filter of one scanline: what an encoder writes after the filter-type byte.
std::string filter_row(int type, const std::string& row, const std::string& prev, std::size_t bpp) {
    std::string out(row.size(), '\0');
    for (std::size_t i = 0; i < row.size(); ++i) {
        const int x = static_cast<uint8_t>(row[i]);
        const int a = i >= bpp ? static_cast<uint8_t>(row[i - bpp]) : 0;
        const int b = static_cast<uint8_t>(prev[i]);
        const int c = i >= bpp ? static_cast<uint8_t>(prev[i - bpp]) : 0;
        int v = x;
        switch (type) {
            case 1: v = x - a; break;
            case 2: v = x - b; break;
            case 3: v = x - ((a + b) / 2); break;
            case 4: v = x - paeth(a, b, c); break;
            default: break;
        }
        out[i] = static_cast<char>(v & 0xff);
    }
    return out;
}

std::string zlib_compress(const std::string& raw) {
    size_t n = 0;
    void* p = tdefl_compress_mem_to_heap(raw.data(), raw.size(), &n, TDEFL_WRITE_ZLIB_HEADER | 128);
    EXPECT_NE(p, nullptr);
    std::string out(static_cast<const char*>(p), n);
    mz_free(p);
    return out;
}

std::size_t channels_of(int color_type) { return color_type == 0 ? 1 : (color_type == 2 ? 3 : 4); }

struct Pixels {
    uint32_t w = 0, h = 0;
    int color_type = 2;
    std::vector<std::string> rows;  // unfiltered scanlines, w * channels bytes each
};

Pixels make_pixels(uint32_t w, uint32_t h, int color_type, uint32_t seed = 12345) {
    Pixels p{w, h, color_type, {}};
    uint32_t s = seed;
    for (uint32_t y = 0; y < h; ++y) {
        std::string row(w * channels_of(color_type), '\0');
        for (char& c : row) {
            s = s * 1664525u + 1013904223u;
            c = static_cast<char>(s >> 24);
        }
        p.rows.push_back(row);
    }
    return p;
}

// What the decoder must return: grey is repeated to RGB, alpha is dropped.
std::vector<uint8_t> expected_rgb(const Pixels& p) {
    std::vector<uint8_t> out;
    const std::size_t ch = channels_of(p.color_type);
    for (const std::string& row : p.rows) {
        for (uint32_t x = 0; x < p.w; ++x) {
            const auto at = [&](std::size_t c) { return static_cast<uint8_t>(row[x * ch + c]); };
            if (p.color_type == 0) {
                out.insert(out.end(), {at(0), at(0), at(0)});
            } else {
                out.insert(out.end(), {at(0), at(1), at(2)});
            }
        }
    }
    return out;
}

struct Build {
    int bit_depth = 8;
    int color_type = -1;          // -1: the pixels' own
    int interlace = 0;
    int filter_for_row = -1;      // -1: row % 5
    int filter_byte_override = -1;
    uint32_t idat_chunks = 1;
    bool extra_chunks = false;    // ancillary chunks and a PLTE, all with valid CRCs
    bool bad_idat_crc = false;
    bool bad_ihdr_crc = false;
    bool drop_iend = false;
    int64_t w_override = -1, h_override = -1;  // -1: the pixels' own
    std::string unknown_critical;  // a critical chunk's name to put before the IDATs
    bool short_data = false;       // compress one row fewer than the header says
    bool long_data = false;        // one row more
    bool garbage_idat = false;
};

std::string make_png(const Pixels& p, const Build& b = {}) {
    const int ct = b.color_type >= 0 ? b.color_type : p.color_type;
    const std::size_t bpp = channels_of(p.color_type);
    std::string raw;
    std::string prev(p.w * bpp, '\0');
    const std::size_t rows = p.rows.size() - (b.short_data ? 1 : 0);
    for (std::size_t y = 0; y < rows; ++y) {
        const int f = b.filter_for_row >= 0 ? b.filter_for_row : static_cast<int>(y % 5);
        raw.push_back(static_cast<char>(b.filter_byte_override >= 0 ? b.filter_byte_override : f));
        raw += filter_row(f, p.rows[y], prev, bpp);
        prev = p.rows[y];
    }
    if (b.long_data) raw += std::string(1 + p.w * bpp, '\0');
    std::string z = b.garbage_idat ? std::string("\x78\x9c\xff\xff\xff\xff garbage", 14) : zlib_compress(raw);

    std::string ihdr = be32(b.w_override >= 0 ? static_cast<uint32_t>(b.w_override) : p.w) +
                       be32(b.h_override >= 0 ? static_cast<uint32_t>(b.h_override) : p.h);
    ihdr.push_back(static_cast<char>(b.bit_depth));
    ihdr.push_back(static_cast<char>(ct));
    ihdr.push_back(0);
    ihdr.push_back(0);
    ihdr.push_back(static_cast<char>(b.interlace));

    std::string png = "\x89PNG\r\n\x1a\n";
    png += chunk("IHDR", ihdr, b.bad_ihdr_crc);
    if (b.extra_chunks) {
        png += chunk("gAMA", be32(45455));
        png += chunk("tEXt", std::string("Comment\0hello", 13));
        png += chunk("prVt", "private ancillary data");
        png += chunk("PLTE", std::string("\x01\x02\x03\x04\x05\x06", 6));
    }
    if (!b.unknown_critical.empty()) png += chunk(b.unknown_critical, "xx");
    const uint32_t parts = std::max<uint32_t>(1, b.idat_chunks);
    const std::size_t each = (z.size() + parts - 1) / parts;
    for (uint32_t i = 0; i < parts; ++i) {
        const std::size_t from = std::min(z.size(), i * each);
        const std::size_t len = std::min(each, z.size() - from);
        png += chunk("IDAT", z.substr(from, len), b.bad_idat_crc && i == 0);
        if (b.extra_chunks && i + 1 < parts) png += chunk("tEXt", std::string("k\0v", 3));
    }
    if (b.extra_chunks) png += chunk("tIME", std::string(7, '\x01'));
    if (!b.drop_iend) png += chunk("IEND", "");
    return png;
}

void expect_decodes(const std::string& png, const Pixels& p, const std::string& what) {
    const auto r = spade::decode_png(bytes_of(png));
    ASSERT_TRUE(r.has_value()) << what << ": " << r.error().context;
    EXPECT_EQ(r->width, p.w) << what;
    EXPECT_EQ(r->height, p.h) << what;
    EXPECT_EQ(r->rgb, expected_rgb(p)) << what;
}

// ---- what it reads ---------------------------------------------------------------------------------------------------------------

TEST(PngDecode, EachColourTypeAndEachFilterRoundTrips) {
    const struct {
        uint32_t w, h;
    } sizes[] = {{7, 5}, {1, 1}, {1, 9}, {33, 1}, {16, 16}};
    for (const int ct : {0, 2, 6}) {
        for (const auto& sz : sizes) {
            const Pixels p = make_pixels(sz.w, sz.h, ct, 7u + sz.w * 31u + sz.h);
            for (int filter = -1; filter <= 4; ++filter) {  // -1: a different filter on each row
                Build b;
                b.filter_for_row = filter;
                expect_decodes(make_png(p, b), p,
                               "colour type " + std::to_string(ct) + ", " + std::to_string(sz.w) + " x " + std::to_string(sz.h) +
                                   ", filter " + std::to_string(filter));
            }
        }
    }
}

std::string from_hex(const std::string& hex) {
    std::string out;
    for (std::size_t i = 0; i + 1 < hex.size(); i += 2) out.push_back(static_cast<char>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    return out;
}

// Three PNGs written by Python (its own filters and zlib, a filter 0 to 4 on rows 0 to 4) and the pixels they hold.
TEST(PngDecode, PngsFromAnIndependentEncoderDecodeToTheirPixels) {
    struct Golden {
        const char* name;
        const char* png;
        const char* rgb;
        uint32_t w, h;
    };
    const Golden g[] = {
        {"grey",
         "89504e470d0a1a0a0000000d494844520000000400000005080000000047c61207000000214944415478da636050f5ca678c363030608a4e2bac61de762d46842"
         "53a902510004b2a067f065b4b820000000049454e44ae426082",
         "0000002525254a4a4a6f6f6f5b5b5b8b8b8bbbbbbbebebebb6b6b6f1f1f12c2c2c6767671111115757579d9d9d9696966c6c6cbdbdbdc1c1c1121212", 4, 5},
        {"rgb",
         "89504e470d0a1a0a0000000d4948445200000004000000050802000000edcfda8c000000414944415478da636030cd528deaf7aadf92bfe42663f484a30630c01"
         "41d1d9d9696565858585353c3bced62f6b5b0b09898181191292c4099c0c040161615200900e5d315f28b30548e0000000049454e44ae426082",
         "00356a255a8f4a7fb46fa4d95b90c58bc0f5bbf025eb2055b6eb20f1265b2c6196679cd111467b578cc19dd20796cb006ca1d6bdf227c1f62b12477c", 4, 5},
        {"rgba",
         "89504e470d0a1a0a0000000d4948445200000003000000050806000000807156a20000003b4944415478da636030cd9aaf1ad57fc4ab7ecb4bc6e809477f1940"
         "01533410a401412110306fbb98dd762d2c2c2c06085840328140c0c2a2a202002473173318ded64a0000000049454e44ae426082",
         "00356a255a8f4a7fb45b90c58bc0f5bbf025b6eb20f1265b2c619611467b578cc19dd2076ca1d6bdf227c1f62b", 3, 5},
    };
    for (const Golden& x : g) {
        const std::string png = from_hex(x.png);
        const auto r = spade::decode_png(bytes_of(png));
        ASSERT_TRUE(r.has_value()) << x.name << ": " << r.error().context;
        EXPECT_EQ(r->width, x.w) << x.name;
        EXPECT_EQ(r->height, x.h) << x.name;
        const std::string want = from_hex(x.rgb);
        EXPECT_EQ(std::string(r->rgb.begin(), r->rgb.end()), want) << x.name;
    }
}

TEST(PngDecode, SeveralIdatChunksAndAncillaryChunksAreRead) {
    const Pixels p = make_pixels(40, 30, 2);
    Build b;
    b.idat_chunks = 4;
    b.extra_chunks = true;  // gAMA, tEXt, a private ancillary chunk, a PLTE, tIME, tEXt between the IDATs
    expect_decodes(make_png(p, b), p, "split IDAT with ancillary chunks");
}

TEST(PngDecode, ALargeImageDecodesAndTwoRunsAgree) {
    const Pixels p = make_pixels(1000, 800, 2, 99);
    const std::string png = make_png(p);
    const auto a = spade::decode_png(bytes_of(png));
    const auto c = spade::decode_png(bytes_of(png));
    ASSERT_TRUE(a.has_value()) << a.error().context;
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(a->rgb, expected_rgb(p));
    EXPECT_EQ(a->rgb, c->rgb);
}

// ---- what it refuses, by name -------------------------------------------------------------------------------------------------------

struct Refusal {
    const char* what;
    std::function<std::string()> input;
    Code code;
    const char* says;  // a word the message must hold
};

TEST(PngDecode, EachThingItCannotReadIsRefusedByName) {
    const Pixels p = make_pixels(6, 6, 2);
    const std::string good = make_png(p);
    const auto with = [&](const std::function<void(Build&)>& f) {
        return [&p, f] {
            Build b;
            f(b);
            return make_png(p, b);
        };
    };
    const Refusal cases[] = {
        {"empty", [] { return std::string(); }, Code::invalid_argument, "PNG"},
        {"not a png", [] { return std::string("hello, this is not an image"); }, Code::invalid_argument, "PNG"},
        {"jpeg", [] { return std::string("\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01", 13); }, Code::unavailable, "JPEG"},
        {"16-bit", with([](Build& b) { b.bit_depth = 16; }), Code::unavailable, "16-bit"},
        {"1-bit", with([](Build& b) { b.bit_depth = 1; }), Code::unavailable, "bit depth"},
        {"palette", with([](Build& b) { b.color_type = 3; }), Code::unavailable, "palette"},
        {"grey with alpha", with([](Build& b) { b.color_type = 4; }), Code::unavailable, "alpha"},
        {"interlaced", with([](Build& b) { b.interlace = 1; }), Code::unavailable, "interlaced"},
        {"bad IHDR crc", with([](Build& b) { b.bad_ihdr_crc = true; }), Code::invalid_argument, "CRC"},
        {"bad IDAT crc", with([](Build& b) { b.bad_idat_crc = true; }), Code::invalid_argument, "CRC"},
        {"truncated in the data", [&] { return good.substr(0, good.size() / 2); }, Code::invalid_argument, "truncated"},
        {"truncated in the signature", [] { return std::string("\x89PNG\r\n", 6); }, Code::invalid_argument, "truncated"},
        {"no IEND", with([](Build& b) { b.drop_iend = true; }), Code::invalid_argument, "IEND"},
        {"size bomb", with([](Build& b) { b.w_override = 60000; b.h_override = 60000; }), Code::capacity_exceeded, "too large"},
        {"dimension overflow", with([](Build& b) { b.w_override = 0xFFFFFFFFll; b.h_override = 0xFFFFFFFFll; }), Code::capacity_exceeded, "too large"},
        {"zero width", with([](Build& b) { b.w_override = 0; }), Code::invalid_argument, "zero"},
        {"zero height", with([](Build& b) { b.h_override = 0; }), Code::invalid_argument, "zero"},
        {"filter type 7", with([](Build& b) { b.filter_byte_override = 7; }), Code::invalid_argument, "filter"},
        {"unknown critical chunk", with([](Build& b) { b.unknown_critical = "ABCD"; }), Code::invalid_argument, "ABCD"},
        {"data one row short", with([](Build& b) { b.short_data = true; }), Code::invalid_argument, "image data"},
        {"data one row long", with([](Build& b) { b.long_data = true; }), Code::invalid_argument, "image data"},
        {"damaged compressed data", with([](Build& b) { b.garbage_idat = true; }), Code::invalid_argument, "image data"},
        {"IDAT before IHDR",
         [] {
             std::string s("\x89PNG\r\n\x1a\n", 8);
             return s + chunk("IDAT", "x") + chunk("IEND", "");
         },
         Code::invalid_argument, "IHDR"},
    };
    for (const Refusal& c : cases) {
        const std::string in = c.input();
        const auto r = spade::decode_png(bytes_of(in));
        ASSERT_FALSE(r.has_value()) << c.what;
        EXPECT_EQ(r.error().code, c.code) << c.what << ": " << r.error().context;
        EXPECT_NE(r.error().context.find(c.says), std::string::npos) << c.what << ": " << r.error().context;
        EXPECT_EQ(r.error().context.rfind("png: ", 0), 0u) << c.what << ": " << r.error().context;
    }
}

TEST(PngDecode, ABombIsRefusedBeforeAnythingIsAllocated) {
    // 40000 x 40000 RGB would be 4.8 GB. The data is a few bytes: the refusal is from the header alone.
    Pixels tiny = make_pixels(2, 2, 2);
    Build b;
    b.w_override = 40000;
    b.h_override = 40000;
    const std::string png = make_png(tiny, b);
    ASSERT_LT(png.size(), 200u);
    const auto r = spade::decode_png(bytes_of(png));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::capacity_exceeded);
}

}  // namespace
