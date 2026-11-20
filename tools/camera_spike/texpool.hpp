#pragma once
// texpool.hpp -- the spike's procedural texture pool (128 textures of 512x512 RGBA8 with a full mip chain, in ONE buffer plus a table) and
// the CPU reference of the shader's filter: exact integer texel loads, a bilinear blend within a mip and a linear blend between two mips,
// both with 8-bit fractional weights, and the mip taken from the pixel footprint with log2 read off the float's exponent bits. No hardware
// sampler, no log2 or pow. Bench code: the generator uses libm (it builds data, it feeds no pinned number).

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

#include "scene.hpp"

namespace cs {

inline constexpr uint32_t kTexSize = 512;   // square, a power of two
inline constexpr uint32_t kTexMips = 10;    // 512 down to 1
inline constexpr uint32_t kTexCount = 128;

inline uint32_t mip_words_before(uint32_t size, uint32_t m) {  // the closed form the shader uses: texels in mips 0..m-1 of a square power-of-two texture
    return ((size * size) >> (2 * m)) * ((4u * ((1u << (2 * m)) - 1u)) / 3u);
}
inline uint32_t tex_words_total() { return (kTexSize * kTexSize * 4u - 1u) / 3u; }  // the whole chain, mips 0..9: 349,525 words a texture

inline uint32_t ihash(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t h = (a * 0x9E3779B1u) ^ ((b + 0x7F4A7C15u) * 0x85EBCA6Bu) ^ ((c + 0x165667B1u) * 0xC2B2AE35u);
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15;
    return h;
}
inline double uh(uint32_t h) { return (h >> 8) * (1.0 / 16777216.0); }

// Tileable value noise: lattice of `freq` cells, smooth interpolation.
inline double vnoise(uint32_t seed, double x, double y, uint32_t freq) {
    const double fx = x * freq, fy = y * freq;
    const double ix = std::floor(fx), iy = std::floor(fy);
    const double tx = fx - ix, ty = fy - iy;
    const double sx = tx * tx * (3 - 2 * tx), sy = ty * ty * (3 - 2 * ty);
    const uint32_t x0 = static_cast<uint32_t>(static_cast<int64_t>(ix)) % freq, y0 = static_cast<uint32_t>(static_cast<int64_t>(iy)) % freq;
    const uint32_t x1 = (x0 + 1) % freq, y1 = (y0 + 1) % freq;
    const double a = uh(ihash(seed, x0, y0)), b = uh(ihash(seed, x1, y0)), c = uh(ihash(seed, x0, y1)), d = uh(ihash(seed, x1, y1));
    return (a + (b - a) * sx) + ((c + (d - c) * sx) - (a + (b - a) * sx)) * sy;
}
inline double fbm(uint32_t seed, double x, double y, uint32_t octaves, uint32_t base_freq, double persist) {
    double sum = 0, amp = 1, norm = 0;
    uint32_t f = base_freq;
    for (uint32_t o = 0; o < octaves; ++o) {
        sum += amp * vnoise(seed + o * 101, x, y, f);
        norm += amp;
        amp *= persist;
        f *= 2;
    }
    return sum / norm;
}

struct TexPool {
    std::vector<uint32_t> words;  // kTexCount x tex_words_total() RGBA8 words, mip 0 first
    std::vector<uint32_t> table;  // per texture: base word, size, mips, type
};

inline void gen_texture(uint32_t idx, uint32_t* out) {  // fills mip 0 then the chain at `out`
    const uint32_t type = idx % 4;
    double c1[3], c2[3];
    for (int k = 0; k < 3; ++k) {
        c1[k] = 0.08 + 0.87 * uh(ihash(idx, 1, k));
        c2[k] = 0.08 + 0.87 * uh(ihash(idx, 2, k));
    }
    const uint32_t N = kTexSize;
    for (uint32_t y = 0; y < N; ++y)
        for (uint32_t x = 0; x < N; ++x) {
            const double u = (x + 0.5) / N, v = (y + 0.5) / N;
            double t = 0;
            if (type == 0) {  // checker
                const uint32_t cells = 4 + ihash(idx, 3, 0) % 13;
                t = (((x * cells / N) + (y * cells / N)) & 1u) ? 1.0 : 0.0;
            } else if (type == 1) {  // stripes: horizontal, vertical or diagonal, an integer number of periods so it tiles
                const uint32_t f = 6 + ihash(idx, 4, 0) % 24, dir = ihash(idx, 5, 0) % 3;
                const double phase = (dir == 0 ? u : (dir == 1 ? v : (u + v))) * f;
                t = (phase - std::floor(phase)) < 0.5 ? 1.0 : 0.0;
            } else if (type == 2) {  // value noise, six octaves
                t = std::min(std::max((fbm(idx, u, v, 6, 4, 0.55) - 0.5) * 1.8 + 0.5, 0.0), 1.0);
            } else {  // feature-rich: noise, a checker overlay, dots and thin lines
                t = fbm(idx, u, v, 5, 8, 0.6);
                t += ((((x / 32) + (y / 32)) & 1u) ? 0.12 : -0.12);
                const uint32_t cx = x / 32, cy = y / 32;
                const double dx = (x % 32) - (4 + 24 * uh(ihash(idx, cx + 7, cy))), dy = (y % 32) - (4 + 24 * uh(ihash(idx, cx, cy + 9)));
                const double rad = 3 + 5 * uh(ihash(idx, cx, cy + 11));
                if (dx * dx + dy * dy < rad * rad) t = (ihash(idx, cx + 3, cy + 5) & 1u) ? 1.0 : 0.0;
                if ((x % 64) == 0 || (y % 64) == 0) t = 0.0;
                t = std::min(std::max(t, 0.0), 1.0);
            }
            uint32_t ch[3];
            for (int k = 0; k < 3; ++k) ch[k] = static_cast<uint32_t>(std::min(std::max(255.0 * (c1[k] + (c2[k] - c1[k]) * t) + 0.5, 0.0), 255.0));
            out[y * N + x] = ch[0] | (ch[1] << 8) | (ch[2] << 16) | (255u << 24);
        }
    uint32_t* src = out;
    for (uint32_t m = 1; m < kTexMips; ++m) {  // the box-filtered mip chain
        uint32_t* dst = out + mip_words_before(kTexSize, m);
        const uint32_t sw = N >> (m - 1), dw = N >> m;
        for (uint32_t y = 0; y < dw; ++y)
            for (uint32_t x = 0; x < dw; ++x) {
                uint32_t r = 0;
                for (int k = 0; k < 4; ++k) {
                    uint32_t sum = 0;
                    for (uint32_t t = 0; t < 4; ++t) sum += (src[(2 * y + (t >> 1)) * sw + 2 * x + (t & 1)] >> (8 * k)) & 255u;
                    r |= ((sum + 2) / 4) << (8 * k);
                }
                dst[y * dw + x] = r;
            }
        src = dst;
    }
}

inline void fill_pool_table(TexPool& p) {
    const uint32_t per = tex_words_total();
    p.table.resize(static_cast<size_t>(kTexCount) * 4);
    for (uint32_t i = 0; i < kTexCount; ++i) {
        p.table[i * 4 + 0] = i * per;
        p.table[i * 4 + 1] = kTexSize;
        p.table[i * 4 + 2] = kTexMips;
        p.table[i * 4 + 3] = i % 4;
    }
}

inline TexPool make_pool() {
    TexPool p;
    const uint32_t per = tex_words_total();
    p.words.assign(static_cast<size_t>(per) * kTexCount, 0);
    const uint32_t nt = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> th;
    for (uint32_t t = 0; t < nt; ++t)
        th.emplace_back([&, t] {
            for (uint32_t i = t; i < kTexCount; i += nt) gen_texture(i, p.words.data() + static_cast<size_t>(i) * per);
        });
    for (auto& t : th) t.join();
    fill_pool_table(p);
    return p;
}

// ---- the CPU reference of the shader's filter (tex.slang), float arithmetic in the same order ----
inline float log2_approx(float f) {
    const uint32_t b = std::bit_cast<uint32_t>(f);
    const int e = static_cast<int>((b >> 23) & 255u) - 127;
    const float m = static_cast<float>(b & 0x7FFFFFu) * (1.0f / 8388608.0f);
    return static_cast<float>(e) + m;
}
inline void bilinear_mip(const TexPool& p, uint32_t idx, uint32_t m, float u, float v, uint32_t out[3]) {
    const uint32_t base = p.table[idx * 4], size = p.table[idx * 4 + 1];
    const uint32_t ms = size >> m;
    const float x = u * static_cast<float>(ms) - 0.5f, y = v * static_cast<float>(ms) - 0.5f;
    const float fx = std::floor(x), fy = std::floor(y);
    const int32_t ix = static_cast<int32_t>(fx), iy = static_cast<int32_t>(fy);
    const uint32_t wx = std::min(static_cast<uint32_t>((x - fx) * 256.0f + 0.5f), 256u);
    const uint32_t wy = std::min(static_cast<uint32_t>((y - fy) * 256.0f + 0.5f), 256u);
    const uint32_t off = base + mip_words_before(size, m);
    const uint32_t mask = ms - 1u;
    auto texel = [&](int32_t tx, int32_t ty) { return p.words[off + (static_cast<uint32_t>(ty) & mask) * ms + (static_cast<uint32_t>(tx) & mask)]; };
    const uint32_t t00 = texel(ix, iy), t10 = texel(ix + 1, iy), t01 = texel(ix, iy + 1), t11 = texel(ix + 1, iy + 1);
    for (int k = 0; k < 3; ++k) {
        const uint32_t sh = 8u * static_cast<uint32_t>(k);
        const uint32_t top = ((t00 >> sh) & 255u) * (256u - wx) + ((t10 >> sh) & 255u) * wx;
        const uint32_t bot = ((t01 >> sh) & 255u) * (256u - wx) + ((t11 >> sh) & 255u) * wx;
        out[k] = (top * (256u - wy) + bot * wy + 32768u) >> 16;
    }
}
inline uint32_t sample_bytes(const TexPool& p, uint32_t idx, float u, float v, float foot) {
    const uint32_t mips = p.table[idx * 4 + 2];
    const float lod = std::min(std::max(log2_approx(std::max(foot, 1e-6f)), 0.0f), static_cast<float>(mips - 1));
    const uint32_t m0 = static_cast<uint32_t>(lod), m1 = std::min(m0 + 1, mips - 1);
    const uint32_t lw = std::min(static_cast<uint32_t>((lod - static_cast<float>(m0)) * 256.0f + 0.5f), 256u);
    uint32_t c0[3], c1[3];
    bilinear_mip(p, idx, m0, u, v, c0);
    bilinear_mip(p, idx, m1, u, v, c1);
    uint32_t r = 0;
    for (int k = 0; k < 3; ++k) r |= (((c0[k] * (256u - lw) + c1[k] * lw + 128u) >> 8) & 255u) << (8 * k);
    return r;
}

}  // namespace cs
