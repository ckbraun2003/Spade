// decode_png (L85e): an 8-bit PNG to RGB8, over miniz's tinfl inflate (compiled into spade_mesh_import for the 3MF reader's ZIP).
// The contract and the refusals are in world/mesh_import.hpp. Everything is bounds-checked against the input's own length, the
// output is sized from the header before anything is inflated, and the header's size is capped first, so no input makes this
// allocate more than kMaxPngPixels pixels' worth.

#include <miniz.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "world/mesh_import.hpp"

namespace spade {

namespace {

[[nodiscard]] std::unexpected<Error> refuse(Code code, const std::string& why) {
    return std::unexpected(Error{code, "png: " + why});
}

constexpr unsigned char kSignature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};

[[nodiscard]] uint32_t be32(const unsigned char* p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) | (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

[[nodiscard]] int paeth(int a, int b, int c) {
    const int p = a + b - c;
    const int pa = p > a ? p - a : a - p;
    const int pb = p > b ? p - b : b - p;
    const int pc = p > c ? p - c : c - p;
    return pa <= pb && pa <= pc ? a : (pb <= pc ? b : c);
}

}  // namespace

Result<world::Image8> decode_png(std::span<const std::byte> bytes) {
    const auto* d = reinterpret_cast<const unsigned char*>(bytes.data());
    const std::size_t n = bytes.size();
    if (n == 0) return refuse(Code::invalid_argument, "the data is empty, so it is not a PNG");
    if (n >= 3 && d[0] == 0xff && d[1] == 0xd8 && d[2] == 0xff) {
        return refuse(Code::unavailable, "this is a JPEG, not a PNG; JPEG is not supported");
    }
    if (std::memcmp(d, kSignature, n < 8 ? n : 8) != 0) return refuse(Code::invalid_argument, "the data is not a PNG (no PNG signature)");
    if (n < 8) return refuse(Code::invalid_argument, "truncated: the PNG signature is cut off");

    uint32_t width = 0, height = 0;
    int color_type = 0;
    std::size_t channels = 0;
    bool have_ihdr = false, have_iend = false;
    std::vector<unsigned char> zdata;

    std::size_t pos = 8;
    while (pos < n && !have_iend) {
        if (n - pos < 12) return refuse(Code::invalid_argument, "truncated: a chunk header is cut off");
        const uint32_t length = be32(d + pos);
        const unsigned char* type = d + pos + 4;
        const std::string name(reinterpret_cast<const char*>(type), 4);
        if (length > n - pos - 12) return refuse(Code::invalid_argument, "truncated: chunk " + name + " runs past the end of the data");
        for (const unsigned char c : name) {
            if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))) {
                return refuse(Code::invalid_argument, "the data is not a PNG (a chunk has a type that is not letters)");
            }
        }
        const unsigned char* data = d + pos + 8;
        uint32_t crc = static_cast<uint32_t>(mz_crc32(MZ_CRC32_INIT, type, 4));
        crc = static_cast<uint32_t>(mz_crc32(crc, data, length));
        if (crc != be32(data + length)) return refuse(Code::invalid_argument, "bad CRC in chunk " + name);
        pos += 12 + static_cast<std::size_t>(length);

        if (!have_ihdr) {
            if (name != "IHDR") return refuse(Code::invalid_argument, "the first chunk is " + name + ", not IHDR");
            if (length != 13) return refuse(Code::invalid_argument, "IHDR is not 13 bytes long");
            width = be32(data);
            height = be32(data + 4);
            const int depth = data[8];
            color_type = data[9];
            const int compression = data[10], filter_method = data[11], interlace = data[12];
            if (color_type == 3) return refuse(Code::unavailable, "a palette PNG (colour type 3) is not supported");
            if (color_type == 4) return refuse(Code::unavailable, "a grey PNG with alpha (colour type 4) is not supported");
            if (color_type != 0 && color_type != 2 && color_type != 6) {
                return refuse(Code::invalid_argument, "colour type " + std::to_string(color_type) + " does not exist");
            }
            if (depth == 16) return refuse(Code::unavailable, "a 16-bit PNG is not supported (8-bit only)");
            if (depth != 8) return refuse(Code::unavailable, "bit depth " + std::to_string(depth) + " is not supported (8-bit only)");
            if (compression != 0 || filter_method != 0) {
                return refuse(Code::invalid_argument, "an unknown compression or filter method in IHDR");
            }
            if (interlace == 1) return refuse(Code::unavailable, "an interlaced PNG is not supported");
            if (interlace != 0) return refuse(Code::invalid_argument, "an unknown interlace method in IHDR");
            if (width == 0 || height == 0) return refuse(Code::invalid_argument, "the width or height is zero");
            const uint64_t pixels = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);  // fits: each is under 2^32
            if (pixels > kMaxPngPixels) {
                return refuse(Code::capacity_exceeded, "the image is " + std::to_string(width) + " x " + std::to_string(height) +
                                                           " pixels, too large (at most " + std::to_string(kMaxPngPixels) + " pixels)");
            }
            channels = color_type == 0 ? 1 : (color_type == 2 ? 3 : 4);
            have_ihdr = true;
            continue;
        }
        if (name == "IHDR") return refuse(Code::invalid_argument, "a second IHDR chunk");
        if (name == "IDAT") {
            zdata.insert(zdata.end(), data, data + length);
        } else if (name == "IEND") {
            have_iend = true;
        } else if (name == "PLTE") {
            // a suggested palette beside RGB or RGBA; nothing here uses it
        } else if ((type[0] & 0x20u) == 0) {
            return refuse(Code::invalid_argument, "unknown critical chunk " + name);
        }  // else an ancillary chunk: ignored (its CRC was checked above)
    }
    if (!have_ihdr) return refuse(Code::invalid_argument, "truncated: there is no IHDR chunk");
    if (!have_iend) return refuse(Code::invalid_argument, "truncated: there is no IEND chunk");
    if (zdata.empty()) return refuse(Code::invalid_argument, "the PNG has no image data (no IDAT chunk)");

    // Inflate into exactly the size the header gives: one filter byte and w * channels bytes a row.
    const std::size_t stride = static_cast<std::size_t>(width) * channels;
    const std::size_t row = stride + 1;
    const std::size_t total = row * static_cast<std::size_t>(height);
    std::vector<unsigned char> raw(total);
    const std::size_t got = tinfl_decompress_mem_to_mem(raw.data(), total, zdata.data(), zdata.size(), TINFL_FLAG_PARSE_ZLIB_HEADER);
    if (got == TINFL_DECOMPRESS_MEM_TO_MEM_FAILED) {
        return refuse(Code::invalid_argument, "the compressed image data is damaged, or longer than the header's size");
    }
    if (got != total) {
        return refuse(Code::invalid_argument, "the image data is shorter than the header's size (" + std::to_string(got) + " of " +
                                                  std::to_string(total) + " bytes)");
    }

    // Undo the filters in place: each row's bytes are rebuilt from the row before, already unfiltered.
    for (std::size_t y = 0; y < height; ++y) {
        unsigned char* cur = raw.data() + y * row;
        const int filter = cur[0];
        if (filter > 4) return refuse(Code::invalid_argument, "filter type " + std::to_string(filter) + " on row " + std::to_string(y));
        unsigned char* line = cur + 1;
        const unsigned char* above = y == 0 ? nullptr : line - row;
        for (std::size_t i = 0; i < stride; ++i) {
            const int a = i >= channels ? line[i - channels] : 0;
            const int b = above != nullptr ? above[i] : 0;
            const int c = (above != nullptr && i >= channels) ? above[i - channels] : 0;
            int predictor = 0;
            switch (filter) {
                case 1: predictor = a; break;
                case 2: predictor = b; break;
                case 3: predictor = (a + b) / 2; break;
                case 4: predictor = paeth(a, b, c); break;
                default: break;
            }
            line[i] = static_cast<unsigned char>(line[i] + predictor);
        }
    }

    world::Image8 out;
    out.width = width;
    out.height = height;
    out.rgb.resize(static_cast<std::size_t>(width) * height * 3);
    unsigned char* dst = out.rgb.data();
    for (std::size_t y = 0; y < height; ++y) {
        const unsigned char* line = raw.data() + y * row + 1;
        for (std::size_t x = 0; x < width; ++x) {
            if (color_type == 0) {
                dst[0] = dst[1] = dst[2] = line[x];
            } else {
                dst[0] = line[x * channels];
                dst[1] = line[x * channels + 1];
                dst[2] = line[x * channels + 2];
            }
            dst += 3;
        }
    }
    return out;
}

}  // namespace spade
