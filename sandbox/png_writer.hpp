// A minimal PNG encoder for the live smoke's stills (live_smoke.hpp):
// 8-bit RGBA, filter type 0 on every row, and a zlib stream of STORED deflate
// blocks. Nothing is compressed, so a 1280x720 still is about 3.7 MB, and in
// exchange the encoder is short enough to check by eye and needs no new
// dependency. Display-free; tests/test_sandbox_live_smoke.cpp decodes its
// output back.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

namespace spade::sandbox::png {

// CRC-32 as PNG and zlib define it (reflected, polynomial 0xEDB88320).
// `crc` continues a previous call, so crc32(b, crc32(a)) == crc32(a + b).
[[nodiscard]] inline uint32_t crc32(const uint8_t* data, std::size_t n, uint32_t crc = 0) {
    static constexpr std::array<uint32_t, 256> kTable = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256u; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1u) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            }
            t[i] = c;
        }
        return t;
    }();
    crc = ~crc;
    for (std::size_t i = 0; i < n; ++i) {
        crc = kTable[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return ~crc;
}

// Adler-32, zlib's stream checksum.
[[nodiscard]] inline uint32_t adler32(const uint8_t* data, std::size_t n) {
    constexpr uint32_t kMod = 65521u;
    uint32_t a = 1u, b = 0u;
    for (std::size_t i = 0; i < n; ++i) {
        a = (a + data[i]) % kMod;
        b = (b + a) % kMod;
    }
    return (b << 16) | a;
}

namespace detail {
inline void put_be32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v >> 24));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v));
}

inline void put_chunk(std::vector<uint8_t>& out, const char (&type)[5], const std::vector<uint8_t>& body) {
    put_be32(out, static_cast<uint32_t>(body.size()));
    const std::size_t type_at = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), body.begin(), body.end());
    put_be32(out, crc32(&out[type_at], 4u + body.size()));
}
}  // namespace detail

// `rgba` is width * height pixels, 4 bytes each, rows packed. `bottom_up`
// says the first row in memory is the BOTTOM of the picture, as glReadPixels
// returns it; the file is always written top row first.
[[nodiscard]] inline std::vector<uint8_t> encode_rgba(const uint8_t* rgba, uint32_t width, uint32_t height,
                                                      bool bottom_up) {
    const std::size_t row_bytes = static_cast<std::size_t>(width) * 4u;
    std::vector<uint8_t> raw;
    raw.reserve((row_bytes + 1u) * height);
    for (uint32_t r = 0; r < height; ++r) {
        const uint32_t src = bottom_up ? height - 1u - r : r;
        raw.push_back(0u);  // filter type 0: none
        const uint8_t* row = rgba + static_cast<std::size_t>(src) * row_bytes;
        raw.insert(raw.end(), row, row + row_bytes);
    }

    // zlib: CMF 0x78 (deflate, 32 KiB window) and FLG 0x01 (0x7801 % 31 == 0),
    // then stored blocks of at most 65535 bytes, then Adler-32, big-endian.
    std::vector<uint8_t> z = {0x78u, 0x01u};
    std::size_t at = 0;
    do {
        const std::size_t n = std::min<std::size_t>(raw.size() - at, 65535u);
        const bool final_block = at + n == raw.size();
        z.push_back(final_block ? 1u : 0u);
        const auto len = static_cast<uint16_t>(n);
        const auto nlen = static_cast<uint16_t>(~len);
        z.push_back(static_cast<uint8_t>(len & 0xFFu));
        z.push_back(static_cast<uint8_t>(len >> 8));
        z.push_back(static_cast<uint8_t>(nlen & 0xFFu));
        z.push_back(static_cast<uint8_t>(nlen >> 8));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(at),
                 raw.begin() + static_cast<std::ptrdiff_t>(at + n));
        at += n;
    } while (at < raw.size());
    detail::put_be32(z, adler32(raw.data(), raw.size()));

    std::vector<uint8_t> png = {0x89u, 'P', 'N', 'G', 0x0Du, 0x0Au, 0x1Au, 0x0Au};
    std::vector<uint8_t> ihdr;
    detail::put_be32(ihdr, width);
    detail::put_be32(ihdr, height);
    ihdr.insert(ihdr.end(), {8u, 6u, 0u, 0u, 0u});  // 8-bit, RGBA, deflate, filter 0, no interlace
    detail::put_chunk(png, "IHDR", ihdr);
    detail::put_chunk(png, "IDAT", z);
    detail::put_chunk(png, "IEND", {});
    return png;
}

// Writes `bytes` to `path`. False when the file could not be written.
[[nodiscard]] inline bool write_file(const std::filesystem::path& path, const std::vector<uint8_t>& bytes) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(out);
}

}  // namespace spade::sandbox::png
