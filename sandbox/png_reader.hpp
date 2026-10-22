// A minimal PNG reader for the library thumbnails (INT-14, N14b): it reads
// exactly what png_writer.hpp writes (8-bit RGBA, filter type 0 on every row,
// a zlib stream of STORED deflate blocks) and refuses everything else by name,
// never misreading it (the lead, 2026-10-22). A PNG dropped into a project's
// library/ by hand gets a clear message, not a garbled tile. Display-free;
// tests/test_png_reader.cpp holds it to each refusal.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "png_writer.hpp"

namespace spade::sandbox::png {

// An image, top row first, 4 bytes (R, G, B, A) a pixel, rows packed.
struct Image {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint8_t> rgba;
};

namespace detail {

[[nodiscard]] inline uint32_t be32(std::span<const uint8_t> b, std::size_t at) {
    return (uint32_t{b[at]} << 24) | (uint32_t{b[at + 1]} << 16) | (uint32_t{b[at + 2]} << 8) | uint32_t{b[at + 3]};
}

[[nodiscard]] inline std::unexpected<Error> refuse(const std::string& why) {
    return std::unexpected(Error{Code::invalid_argument, why});
}

// What a PNG's colour type is, in words.
[[nodiscard]] inline std::string colour_name(uint8_t colour) {
    switch (colour) {
        case 0: return "grey";
        case 2: return "RGB without alpha";
        case 3: return "palette";
        case 4: return "grey with alpha";
        default: return "unknown";
    }
}

// The pixel bytes a decoded image may hold, at most (256 MiB: far past any
// thumbnail, short of a size whose arithmetic could overflow).
inline constexpr uint64_t kMaxPixelBytes = uint64_t{1} << 28;

}  // namespace detail

// `bytes`, read as png_writer writes it. invalid_argument, naming what it
// found, for anything else: no signature, a damaged or truncated chunk, no
// IHDR or IEND, an unknown critical chunk, a colour type other than RGBA, a
// bit depth other than 8, interlace, a compressed deflate block, a filtered
// row, a bad Adler-32, or more or fewer pixels than the header's size.
[[nodiscard]] inline Result<Image> decode_rgba(std::span<const uint8_t> bytes) {
    static constexpr uint8_t kSignature[8] = {0x89u, 'P', 'N', 'G', 0x0Du, 0x0Au, 0x1Au, 0x0Au};
    if (bytes.size() < 8 || std::memcmp(bytes.data(), kSignature, 8) != 0) {
        return detail::refuse("not a PNG: its first 8 bytes aren't the PNG signature");
    }

    Image image;
    bool header = false;
    bool ended = false;
    std::vector<uint8_t> zlib;
    std::size_t at = 8;
    while (!ended && at < bytes.size()) {
        if (at + 12 > bytes.size()) {
            return detail::refuse("truncated: " + std::to_string(bytes.size() - at) + " stray bytes at byte " +
                                  std::to_string(at));
        }
        const uint32_t len = detail::be32(bytes, at);
        const std::string type(reinterpret_cast<const char*>(&bytes[at + 4]), 4);
        if (len > bytes.size() - at - 12) {
            return detail::refuse("truncated: the " + type + " chunk at byte " + std::to_string(at) +
                                  " runs past the end");
        }
        if (crc32(&bytes[at + 4], 4u + len) != detail::be32(bytes, at + 8 + len)) {
            return detail::refuse("a damaged " + type + " chunk at byte " + std::to_string(at) +
                                  ": its CRC doesn't match");
        }
        const std::span<const uint8_t> body = bytes.subspan(at + 8, len);
        if (type == "IHDR") {
            if (len != 13) return detail::refuse("an IHDR of " + std::to_string(len) + " bytes (13 expected)");
            image.width = detail::be32(body, 0);
            image.height = detail::be32(body, 4);
            const uint8_t depth = body[8], colour = body[9];
            if (colour != 6) {
                return detail::refuse("colour type " + std::to_string(colour) + " (" + detail::colour_name(colour) +
                                      "); only 8-bit RGBA is read");
            }
            if (depth != 8) return detail::refuse("bit depth " + std::to_string(depth) + "; only 8 is read");
            if (body[10] != 0) return detail::refuse("compression method " + std::to_string(body[10]));
            if (body[11] != 0) return detail::refuse("filter method " + std::to_string(body[11]));
            if (body[12] != 0) return detail::refuse("interlaced (Adam7); only non-interlaced images are read");
            if (image.width == 0 || image.height == 0) {
                return detail::refuse("an empty image (" + std::to_string(image.width) + "x" +
                                      std::to_string(image.height) + ")");
            }
            if (uint64_t{image.width} * image.height * 4u > detail::kMaxPixelBytes) {
                return detail::refuse("too large: " + std::to_string(image.width) + "x" +
                                      std::to_string(image.height));
            }
            header = true;
        } else if (type == "IDAT") {
            if (!header) return detail::refuse("image data before the IHDR");
            zlib.insert(zlib.end(), body.begin(), body.end());
        } else if (type == "IEND") {
            ended = true;
        } else if ((static_cast<unsigned char>(type[0]) & 0x20u) == 0) {
            // A lower-case first letter is ancillary (skippable); upper-case is critical.
            return detail::refuse("an unknown critical chunk '" + type + "'");
        }
        at += 12u + len;
    }
    if (!header) return detail::refuse("no IHDR");
    if (!ended) return detail::refuse("no IEND: the file stops after byte " + std::to_string(bytes.size()));

    // zlib: CMF (deflate), FLG (no preset dictionary, the check bits), then
    // stored blocks, then Adler-32 of what they carry.
    if (zlib.size() < 6) return detail::refuse("image data of " + std::to_string(zlib.size()) + " bytes");
    if ((zlib[0] & 0x0Fu) != 8u) return detail::refuse("image data that isn't deflate");
    if (((zlib[0] << 8) | zlib[1]) % 31 != 0) return detail::refuse("a damaged zlib header");
    if ((zlib[1] & 0x20u) != 0) return detail::refuse("a preset zlib dictionary");
    std::vector<uint8_t> raw;
    std::size_t z = 2;
    for (bool last = false; !last;) {
        if (z + 5 > zlib.size()) return detail::refuse("truncated image data");
        last = (zlib[z] & 1u) != 0;
        const uint32_t btype = (zlib[z] >> 1) & 3u;
        if (btype != 0) {
            return detail::refuse("a compressed deflate block (type " + std::to_string(btype) +
                                  "); only stored blocks, as png_writer writes, are read");
        }
        const auto n = static_cast<uint16_t>(zlib[z + 1] | (zlib[z + 2] << 8));
        const auto nn = static_cast<uint16_t>(zlib[z + 3] | (zlib[z + 4] << 8));
        if (static_cast<uint16_t>(~n) != nn) return detail::refuse("a damaged stored block");
        if (z + 5 + n > zlib.size()) return detail::refuse("truncated image data");
        raw.insert(raw.end(), zlib.begin() + static_cast<std::ptrdiff_t>(z + 5),
                   zlib.begin() + static_cast<std::ptrdiff_t>(z + 5 + n));
        z += 5u + n;
    }
    if (z + 4 != zlib.size()) {
        return detail::refuse(std::to_string(zlib.size() - std::min(zlib.size(), z)) +
                              " bytes after the last block (4, the Adler-32, expected)");
    }
    if (detail::be32(zlib, z) != adler32(raw.data(), raw.size())) {
        return detail::refuse("the image data's Adler-32 doesn't match");
    }

    const std::size_t row_bytes = std::size_t{image.width} * 4u;
    const std::size_t want = (row_bytes + 1u) * image.height;
    if (raw.size() != want) {
        return detail::refuse(std::to_string(raw.size()) + " bytes of scanlines for a " + std::to_string(image.width) +
                              "x" + std::to_string(image.height) + " image (" + std::to_string(want) + " expected)");
    }
    image.rgba.reserve(row_bytes * image.height);
    for (uint32_t r = 0; r < image.height; ++r) {
        const std::size_t row = r * (row_bytes + 1u);
        if (raw[row] != 0) {
            return detail::refuse("filter type " + std::to_string(raw[row]) + " on row " + std::to_string(r) +
                                  "; only 0 (none), as png_writer writes, is read");
        }
        image.rgba.insert(image.rgba.end(), raw.begin() + static_cast<std::ptrdiff_t>(row + 1),
                          raw.begin() + static_cast<std::ptrdiff_t>(row + 1 + row_bytes));
    }
    return image;
}

// The file at `path`, read whole and decoded; a refusal names it.
[[nodiscard]] inline Result<Image> read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::unexpected(Error{Code::io_error, "'" + path.generic_string() + "': can't be read"});
    const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    Result<Image> image = decode_rgba(bytes);
    if (!image) {
        return std::unexpected(Error{image.error().code, "'" + path.generic_string() + "': " + image.error().context});
    }
    return image;
}

}  // namespace spade::sandbox::png
