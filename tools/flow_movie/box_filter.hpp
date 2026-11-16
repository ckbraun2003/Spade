// tools/flow_movie/box_filter.hpp -- the web masters' supersample filter: a frame rendered at s times its size, averaged
// down over s x s blocks (RGBA8, the top row first). L74b (Cameron's quick wins): it was the 4K frame's biggest CPU
// phase, 0.207 s of 0.47, a scalar loop over 132 MB.
//
// BYTE-IDENTICAL to the loop it replaces, integers only: each channel of each output pixel is
// (the s x s block's sum + s*s/2) / (s*s), rounded half up. The 2x filter (the web's) does it with SSE2 (two output
// pixels a step: sums of four bytes at most 1020 in 16 bits, +2, >> 2, packed back); the other sizes take the plain loop
// over hoisted row pointers. The output rows are split into bands, one thread each, which write disjoint rows and read
// only the input, so the thread count changes no bit. Header-only and free of GL, so a test links nothing of the tool.
//
// box_down_flipped() is the same filter reading GL's own picture, the BOTTOM row first and the alpha as the march left it,
// and writing the top row first with every alpha 255: GlStage::draw's row flip and alpha fill folded into it, so a frame
// 4x the output's size is read once and never copied. An alpha of 255 everywhere averages to 255 at every s, so the fill
// is the output's alpha set to 255, the same bits.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define SPADE_BOX_FILTER_SSE2 1
#endif

namespace flow_movie {
namespace detail {

// One output row of the 2x filter: `r0` and `r1` the two big rows (4 * 2w bytes each), `out` w pixels.
inline void box_row2(const uint8_t* r0, const uint8_t* r1, uint8_t* out, uint32_t w, bool opaque) {
    uint32_t x = 0;
#ifdef SPADE_BOX_FILTER_SSE2
    const __m128i zero = _mm_setzero_si128();
    const __m128i two = _mm_set1_epi16(2);
    const __m128i alpha = opaque ? _mm_set_epi16(0xFF, 0, 0, 0, 0xFF, 0, 0, 0) : zero;  // lanes 3 and 7: the alpha bytes
    for (; x + 2u <= w; x += 2u) {  // two output pixels: four big pixels of each row, 16 bytes
        const __m128i a = _mm_loadu_si128(reinterpret_cast<const __m128i*>(r0 + static_cast<std::size_t>(x) * 8u));
        const __m128i b = _mm_loadu_si128(reinterpret_cast<const __m128i*>(r1 + static_cast<std::size_t>(x) * 8u));
        const __m128i lo = _mm_add_epi16(_mm_unpacklo_epi8(a, zero), _mm_unpacklo_epi8(b, zero));  // pixels 0, 1
        const __m128i hi = _mm_add_epi16(_mm_unpackhi_epi8(a, zero), _mm_unpackhi_epi8(b, zero));  // pixels 2, 3
        __m128i sum = _mm_add_epi16(_mm_unpacklo_epi64(lo, hi), _mm_unpackhi_epi64(lo, hi));      // 0 + 1, 2 + 3
        sum = _mm_or_si128(_mm_srli_epi16(_mm_add_epi16(sum, two), 2), alpha);  // <= 255, so the OR is 255 in alpha
        _mm_storel_epi64(reinterpret_cast<__m128i*>(out + static_cast<std::size_t>(x) * 4u), _mm_packus_epi16(sum, sum));
    }
#endif
    for (; x < w; ++x) {
        const uint8_t* a = r0 + static_cast<std::size_t>(x) * 8u;
        const uint8_t* b = r1 + static_cast<std::size_t>(x) * 8u;
        for (uint32_t c = 0; c < 4u; ++c) {
            out[static_cast<std::size_t>(x) * 4u + c] =
                static_cast<uint8_t>((static_cast<uint32_t>(a[c]) + a[4u + c] + b[c] + b[4u + c] + 2u) >> 2);
        }
        if (opaque) out[static_cast<std::size_t>(x) * 4u + 3u] = 255u;
    }
}

// Rows [y0, y1) of the output at any s, of a picture `h * s` rows tall. `flipped`: the picture's rows run bottom first
// (the output's row y averages its rows h*s - 1 - (y*s + dy)); `opaque`: every output alpha is 255.
inline void box_rows(const uint8_t* big, uint8_t* out, uint32_t w, uint32_t h, uint32_t s, uint32_t y0, uint32_t y1,
                     bool flipped, bool opaque) {
    const std::size_t big_row = static_cast<std::size_t>(w) * s * 4u;
    const std::size_t out_row = static_cast<std::size_t>(w) * 4u;
    const uint32_t area = s * s;
    const uint32_t big_h = h * s;
    for (uint32_t y = y0; y < y1; ++y) {
        const auto row_of = [&](uint32_t dy) {
            const uint32_t r = y * s + dy;
            return big + static_cast<std::size_t>(flipped ? big_h - 1u - r : r) * big_row;
        };
        uint8_t* dst = out + static_cast<std::size_t>(y) * out_row;
        if (s == 1u) {
            std::memcpy(dst, row_of(0u), out_row);
            if (opaque) {
                for (uint32_t x = 0; x < w; ++x) dst[static_cast<std::size_t>(x) * 4u + 3u] = 255u;
            }
        } else if (s == 2u) {
            box_row2(row_of(0u), row_of(1u), dst, w, opaque);
        } else {
            for (uint32_t x = 0; x < w; ++x) {
                for (uint32_t c = 0; c < 4u; ++c) {
                    if (opaque && c == 3u) {
                        dst[static_cast<std::size_t>(x) * 4u + 3u] = 255u;
                        continue;
                    }
                    uint32_t sum = 0;
                    for (uint32_t dy = 0; dy < s; ++dy) {
                        const uint8_t* p = row_of(dy) + static_cast<std::size_t>(x) * s * 4u + c;
                        for (uint32_t dx = 0; dx < s; ++dx) sum += p[static_cast<std::size_t>(dx) * 4u];
                    }
                    dst[static_cast<std::size_t>(x) * 4u + c] = static_cast<uint8_t>((sum + area / 2u) / area);
                }
            }
        }
    }
}

// The filter, banded over threads.
[[nodiscard]] inline std::vector<uint8_t> box_down_impl(const std::vector<uint8_t>& big, uint32_t w, uint32_t h, uint32_t s,
                                                        uint32_t threads, bool flipped) {
    std::vector<uint8_t> out(static_cast<std::size_t>(w) * h * 4u);
    if (w == 0u || h == 0u || s == 0u) return out;
    if (big.size() < static_cast<std::size_t>(w) * s * h * s * 4u) return out;  // no such picture: nothing to average
    if (threads == 0u) {
        const uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
        threads = static_cast<std::size_t>(w) * h < 65536u ? 1u : std::min(hardware, 8u);
    }
    threads = std::min(threads, h);
    const uint8_t* src = big.data();
    uint8_t* dst = out.data();
    if (threads <= 1u) {
        box_rows(src, dst, w, h, s, 0u, h, flipped, flipped);
        return out;
    }
    std::vector<std::thread> workers;
    workers.reserve(threads - 1u);
    const auto band_start = [&](uint32_t t) {
        return static_cast<uint32_t>(static_cast<uint64_t>(h) * t / threads);
    };
    for (uint32_t t = 1u; t < threads; ++t) {
        workers.emplace_back([=] { box_rows(src, dst, w, h, s, band_start(t), band_start(t + 1u), flipped, flipped); });
    }
    box_rows(src, dst, w, h, s, band_start(0u), band_start(1u), flipped, flipped);
    for (std::thread& worker : workers) worker.join();
    return out;
}

}  // namespace detail

// The frame's pixels averaged over s x s blocks (RGBA8, the top row first), for a render at s times the size.
// `threads` bands of output rows run at once (0: the hardware's, at most 8, and one for a small picture).
[[nodiscard]] inline std::vector<uint8_t> box_down(const std::vector<uint8_t>& big, uint32_t w, uint32_t h, uint32_t s,
                                                   uint32_t threads = 0) {
    return detail::box_down_impl(big, w, h, s, threads, false);
}

// The same for GL's picture: the bottom row first, alpha not yet 255. The output is top row first with every alpha 255.
[[nodiscard]] inline std::vector<uint8_t> box_down_flipped(const std::vector<uint8_t>& big, uint32_t w, uint32_t h,
                                                           uint32_t s, uint32_t threads = 0) {
    return detail::box_down_impl(big, w, h, s, threads, true);
}

}  // namespace flow_movie
