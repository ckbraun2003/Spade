// The library thumbnails' PNG reader (INT-14, N14b): sandbox/png_reader.hpp
// reads what sandbox/png_writer.hpp writes (8-bit RGBA, filter type 0, a zlib
// stream of STORED deflate blocks) and refuses everything else by name, never
// misreading it (the lead, 2026-10-22). A PNG dropped into a project's
// library/ by hand then gets a clear message, not a garbled tile.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "../sandbox/png_reader.hpp"
#include "../sandbox/png_writer.hpp"
#include "unique_temp.hpp"

namespace {

namespace png = spade::sandbox::png;

void put_be32(std::vector<uint8_t>& out, uint32_t v) {
    for (int s = 24; s >= 0; s -= 8) out.push_back(static_cast<uint8_t>(v >> s));
}

// One chunk: length, type, body, and the body's CRC.
std::vector<uint8_t> chunk(const char (&type)[5], const std::vector<uint8_t>& body) {
    std::vector<uint8_t> out;
    put_be32(out, static_cast<uint32_t>(body.size()));
    const std::size_t type_at = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), body.begin(), body.end());
    put_be32(out, png::crc32(&out[type_at], 4u + body.size()));
    return out;
}

std::vector<uint8_t> ihdr(uint32_t w, uint32_t h, uint8_t depth = 8, uint8_t colour = 6, uint8_t interlace = 0) {
    std::vector<uint8_t> body;
    put_be32(body, w);
    put_be32(body, h);
    body.insert(body.end(), {depth, colour, 0u, 0u, interlace});
    return body;
}

// `w` x `h` scanlines, every row's filter byte `filter`, each pixel's bytes
// counting up from its index.
std::vector<uint8_t> scanlines(uint32_t w, uint32_t h, uint8_t filter = 0) {
    std::vector<uint8_t> raw;
    for (uint32_t r = 0; r < h; ++r) {
        raw.push_back(filter);
        for (uint32_t i = 0; i < w * 4u; ++i) raw.push_back(static_cast<uint8_t>(r * w * 4u + i));
    }
    return raw;
}

// A zlib stream of ONE deflate block of type `btype` holding `raw` as a
// stored block would (so only its type bits say otherwise), and the Adler.
std::vector<uint8_t> zlib_of(const std::vector<uint8_t>& raw, uint8_t btype = 0) {
    std::vector<uint8_t> z = {0x78u, 0x01u};
    z.push_back(static_cast<uint8_t>(1u | (btype << 1)));
    const auto n = static_cast<uint16_t>(raw.size());
    const auto nn = static_cast<uint16_t>(~n);
    z.insert(z.end(), {static_cast<uint8_t>(n & 0xFFu), static_cast<uint8_t>(n >> 8), static_cast<uint8_t>(nn & 0xFFu),
                       static_cast<uint8_t>(nn >> 8)});
    z.insert(z.end(), raw.begin(), raw.end());
    put_be32(z, png::adler32(raw.data(), raw.size()));
    return z;
}

std::vector<uint8_t> png_of(const std::vector<uint8_t>& header, const std::vector<uint8_t>& zlib) {
    std::vector<uint8_t> out = {0x89u, 'P', 'N', 'G', 0x0Du, 0x0Au, 0x1Au, 0x0Au};
    for (const std::vector<uint8_t>& c : {chunk("IHDR", header), chunk("IDAT", zlib), chunk("IEND", {})}) {
        out.insert(out.end(), c.begin(), c.end());
    }
    return out;
}

// The pixels scanlines(w, h) carries, top row first.
std::vector<uint8_t> pixels(uint32_t w, uint32_t h) {
    std::vector<uint8_t> rgba;
    for (uint32_t i = 0; i < w * h * 4u; ++i) rgba.push_back(static_cast<uint8_t>(i));
    return rgba;
}

}  // namespace

// The fixtures build what the writer writes, byte for byte.
TEST(PngReader, TheFixturesAreWhatTheWriterWrites) {
    const std::vector<uint8_t> rgba = pixels(3, 2);
    EXPECT_EQ(png_of(ihdr(3, 2), zlib_of(scanlines(3, 2))), png::encode_rgba(rgba.data(), 3, 2, false));
}

// Top row first, whichever way the writer was handed the rows.
TEST(PngReader, WhatTheWriterWritesReadsBackTopRowFirst) {
    const std::vector<uint8_t> rgba = pixels(3, 2);
    const spade::Result<png::Image> top = png::decode_rgba(png::encode_rgba(rgba.data(), 3, 2, false));
    ASSERT_TRUE(top.has_value()) << top.error().context;
    EXPECT_EQ(top->width, 3u);
    EXPECT_EQ(top->height, 2u);
    EXPECT_EQ(top->rgba, rgba);

    const spade::Result<png::Image> flipped = png::decode_rgba(png::encode_rgba(rgba.data(), 3, 2, true));
    ASSERT_TRUE(flipped.has_value()) << flipped.error().context;
    const std::vector<uint8_t> bottom_row(rgba.begin() + 12, rgba.end());
    EXPECT_EQ(std::vector<uint8_t>(flipped->rgba.begin(), flipped->rgba.begin() + 12), bottom_row);
}

// More than one stored block: a block holds at most 65535 bytes.
TEST(PngReader, AnImageOverSeveralStoredBlocksReadsBack) {
    const uint32_t w = 200, h = 100;  // 80,100 bytes of scanlines: two blocks
    std::vector<uint8_t> rgba(static_cast<std::size_t>(w) * h * 4u);
    for (std::size_t i = 0; i < rgba.size(); ++i) rgba[i] = static_cast<uint8_t>(i * 7u);
    const spade::Result<png::Image> image = png::decode_rgba(png::encode_rgba(rgba.data(), w, h, false));
    ASSERT_TRUE(image.has_value()) << image.error().context;
    EXPECT_EQ(image->rgba, rgba);
}

// Everything the writer never writes is refused, naming what it is.
TEST(PngReader, WhatTheWriterNeverWritesIsRefusedByName) {
    const std::vector<uint8_t> good = png_of(ihdr(2, 2), zlib_of(scanlines(2, 2)));

    std::vector<uint8_t> unsigned_ = good;
    unsigned_[1] = 'Q';
    std::vector<uint8_t> damaged = good;
    damaged[8 + 25 + 8 + 4] ^= 0x01u;  // a byte inside IDAT's body: its CRC no longer matches
    const std::vector<uint8_t> truncated(good.begin(), good.end() - 20);
    std::vector<uint8_t> broken_adler = zlib_of(scanlines(2, 2));
    broken_adler.back() ^= 0x01u;
    std::vector<uint8_t> no_end = good;
    no_end.resize(no_end.size() - 12);  // IEND is 12 bytes

    struct Case {
        const char* what;
        std::vector<uint8_t> bytes;
        const char* named;
    };
    const std::vector<Case> cases = {
        {"no signature", unsigned_, "not a PNG"},
        {"a damaged chunk", damaged, "CRC"},
        {"a truncated file", truncated, "truncated"},
        {"no IEND", no_end, "no IEND"},
        {"a palette image", png_of(ihdr(2, 2, 8, 3), zlib_of(scanlines(2, 2))), "palette"},
        {"an RGB image", png_of(ihdr(2, 2, 8, 2), zlib_of(scanlines(2, 2))), "RGB without alpha"},
        {"a grey image", png_of(ihdr(2, 2, 8, 0), zlib_of(scanlines(2, 2))), "grey"},
        {"16-bit samples", png_of(ihdr(2, 2, 16, 6), zlib_of(scanlines(2, 2))), "bit depth 16"},
        {"an interlaced image", png_of(ihdr(2, 2, 8, 6, 1), zlib_of(scanlines(2, 2))), "interlaced"},
        {"a compressed block", png_of(ihdr(2, 2), zlib_of(scanlines(2, 2), 1)), "compressed"},
        {"a filtered row", png_of(ihdr(2, 2), zlib_of(scanlines(2, 2, 1))), "filter type 1"},
        {"a bad Adler-32", png_of(ihdr(2, 2), broken_adler), "Adler-32"},
        {"too few pixels", png_of(ihdr(3, 2), zlib_of(scanlines(2, 2))), "for a 3x2 image"},
        {"an empty image", png_of(ihdr(0, 2), zlib_of({})), "empty"},
    };
    for (const Case& c : cases) {
        const spade::Result<png::Image> image = png::decode_rgba(c.bytes);
        ASSERT_FALSE(image.has_value()) << c.what << " was read";
        EXPECT_NE(image.error().context.find(c.named), std::string::npos)
            << c.what << ": '" << image.error().context << "' doesn't say '" << c.named << "'";
    }
}

// A file is read whole; a refusal names the file.
TEST(PngReader, ReadFileNamesTheFileItRefuses) {
    spade::test::TempDir tmp("png_reader");
    const std::vector<uint8_t> rgba = pixels(2, 2);
    ASSERT_TRUE(png::write_file(tmp.path() / "good.png", png::encode_rgba(rgba.data(), 2, 2, false)));
    const spade::Result<png::Image> good = png::read_file(tmp.path() / "good.png");
    ASSERT_TRUE(good.has_value()) << good.error().context;
    EXPECT_EQ(good->rgba, rgba);

    const spade::Result<png::Image> missing = png::read_file(tmp.path() / "missing.png");
    ASSERT_FALSE(missing.has_value());
    EXPECT_NE(missing.error().context.find("missing.png"), std::string::npos) << missing.error().context;

    ASSERT_TRUE(png::write_file(tmp.path() / "palette.png", png_of(ihdr(2, 2, 8, 3), zlib_of(scanlines(2, 2)))));
    const spade::Result<png::Image> palette = png::read_file(tmp.path() / "palette.png");
    ASSERT_FALSE(palette.has_value());
    EXPECT_NE(palette.error().context.find("palette.png"), std::string::npos) << palette.error().context;
    EXPECT_NE(palette.error().context.find("palette"), std::string::npos) << palette.error().context;
}
