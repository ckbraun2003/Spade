#pragma once
// png.hpp -- a tiny PNG encoder for the sample frames (V11): 8-bit grey or RGB, zlib "stored" blocks (no compression), so it is about thirty
// lines and cannot be wrong. Bench code.

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace cs {

inline uint32_t png_crc(const uint8_t* p, size_t n, uint32_t crc = 0) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        init = true;
    }
    crc = ~crc;
    for (size_t i = 0; i < n; ++i) crc = table[(crc ^ p[i]) & 255u] ^ (crc >> 8);
    return ~crc;
}

inline void png_put32(std::vector<uint8_t>& v, uint32_t x) {
    for (int s = 24; s >= 0; s -= 8) v.push_back(static_cast<uint8_t>(x >> s));
}

inline void png_chunk(std::vector<uint8_t>& out, const char type[5], const std::vector<uint8_t>& data) {
    png_put32(out, static_cast<uint32_t>(data.size()));
    std::vector<uint8_t> body(type, type + 4);
    body.insert(body.end(), data.begin(), data.end());
    out.insert(out.end(), body.begin(), body.end());
    png_put32(out, png_crc(body.data(), body.size()));
}

// planes: 1 (grey) or 3 (RGB); the pixel (x, y) channel c is src[c * W * H + y * W + x] (the spike's NCHW layout).
inline bool write_png(const std::string& path, uint32_t W, uint32_t H, uint32_t planes, const uint8_t* src) {
    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>(H) * (W * planes + 1));
    for (uint32_t y = 0; y < H; ++y) {
        raw.push_back(0);  // filter: none
        for (uint32_t x = 0; x < W; ++x)
            for (uint32_t c = 0; c < planes; ++c) raw.push_back(src[(static_cast<size_t>(c) * H + y) * W + x]);
    }
    std::vector<uint8_t> z{0x78, 0x01};
    uint32_t a = 1, b = 0;
    for (uint8_t v : raw) {
        a = (a + v) % 65521u;
        b = (b + a) % 65521u;
    }
    for (size_t off = 0; off < raw.size();) {
        const size_t n = std::min<size_t>(65535, raw.size() - off);
        const bool last = off + n == raw.size();
        z.push_back(last ? 1 : 0);
        z.push_back(static_cast<uint8_t>(n & 255));
        z.push_back(static_cast<uint8_t>(n >> 8));
        z.push_back(static_cast<uint8_t>(~n & 255));
        z.push_back(static_cast<uint8_t>((~n >> 8) & 255));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(off), raw.begin() + static_cast<std::ptrdiff_t>(off + n));
        off += n;
    }
    png_put32(z, (b << 16) | a);
    std::vector<uint8_t> out{0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<uint8_t> ihdr;
    png_put32(ihdr, W);
    png_put32(ihdr, H);
    ihdr.insert(ihdr.end(), {8, static_cast<uint8_t>(planes == 1 ? 0 : 2), 0, 0, 0});
    png_chunk(out, "IHDR", ihdr);
    png_chunk(out, "IDAT", z);
    png_chunk(out, "IEND", {});
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    return static_cast<bool>(f);
}

}  // namespace cs
