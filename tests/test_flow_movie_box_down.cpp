// ---------------------------------------------------------------------------
// test_flow_movie_box_down.cpp -- tools/flow_movie/box_filter.hpp, the web masters' 2x supersample filter (L74b, Cameron's
// quick wins: the 4K frame's biggest CPU phase was this filter, 0.207 s of 0.47).
//
//   FlowMovieBoxDown  the filter is BYTE-IDENTICAL to the original: the average of each s x s block, per channel, rounded
//                     half up, ((sum + s*s/2) / (s*s)) in integers (the original triple loop is kept here as the
//                     oracle). Over random RGBA8 pictures of every shape the filter may meet (odd and tiny sizes,
//                     s = 1 to 4, one thread to more threads than rows), the extremes (all 0, all 255, alternating),
//                     and a frame at the web size, 8K to 4K.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "../tools/flow_movie/box_filter.hpp"

namespace {

// The original, as it was in tools/flow_movie/main.cpp before the speed-up: the oracle.
[[nodiscard]] std::vector<uint8_t> box_down_original(const std::vector<uint8_t>& big, uint32_t w, uint32_t h, uint32_t s) {
    std::vector<uint8_t> out(static_cast<std::size_t>(w) * h * 4u);
    const uint32_t bw = w * s;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            for (uint32_t c = 0; c < 4u; ++c) {
                uint32_t sum = 0;
                for (uint32_t dy = 0; dy < s; ++dy) {
                    for (uint32_t dx = 0; dx < s; ++dx) {
                        sum += big[((static_cast<std::size_t>(y) * s + dy) * bw + (x * s + dx)) * 4u + c];
                    }
                }
                out[(static_cast<std::size_t>(y) * w + x) * 4u + c] = static_cast<uint8_t>((sum + s * s / 2u) / (s * s));
            }
        }
    }
    return out;
}

[[nodiscard]] std::vector<uint8_t> random_picture(std::mt19937& rng, uint32_t w, uint32_t h, uint32_t s) {
    std::vector<uint8_t> big(static_cast<std::size_t>(w) * s * h * s * 4u);
    for (uint8_t& b : big) b = static_cast<uint8_t>(rng() & 0xffu);
    return big;
}

}  // namespace

TEST(FlowMovieBoxDown, TheFilterIsByteIdenticalToTheOriginalOverRandomPicturesAndEveryThreadCount) {
    std::mt19937 rng(0xB0C5D0u);
    const uint32_t shapes[][2] = {{1, 1}, {2, 1}, {1, 2}, {3, 5}, {7, 3}, {16, 9}, {17, 11}, {64, 36}, {5, 40}, {129, 3}};
    for (const auto& shape : shapes) {
        for (uint32_t s = 1; s <= 4; ++s) {
            const std::vector<uint8_t> big = random_picture(rng, shape[0], shape[1], s);
            const std::vector<uint8_t> want = box_down_original(big, shape[0], shape[1], s);
            for (const uint32_t threads : {0u, 1u, 2u, 3u, 8u, 64u}) {
                EXPECT_EQ(flow_movie::box_down(big, shape[0], shape[1], s, threads), want)
                    << shape[0] << "x" << shape[1] << " at s " << s << ", " << threads << " threads";
            }
        }
    }
}

TEST(FlowMovieBoxDown, TheExtremesRoundTheSameWay) {
    for (const uint32_t s : {2u, 3u, 4u}) {
        const uint32_t w = 21, h = 6;
        const std::size_t bytes = static_cast<std::size_t>(w) * s * h * s * 4u;
        std::vector<uint8_t> zeros(bytes, 0), ones(bytes, 255), alt(bytes), edge(bytes);
        for (std::size_t i = 0; i < bytes; ++i) {
            alt[i] = (i / 4u) % 2u == 0u ? 255u : 0u;       // alternating pixels
            edge[i] = (i % 7u == 0u) ? 255u : 128u;        // a sum that lands on the rounding half
        }
        for (const std::vector<uint8_t>* picture : {&zeros, &ones, &alt, &edge}) {
            EXPECT_EQ(flow_movie::box_down(*picture, w, h, s), box_down_original(*picture, w, h, s)) << "s " << s;
        }
        EXPECT_EQ(flow_movie::box_down(ones, w, h, s).front(), 255u) << "an opaque picture stays opaque";
    }
}

TEST(FlowMovieBoxDown, AFrameAtTheWebSizeEightKToFourKIsIdentical) {
    std::mt19937 rng(0x4B8Cu);
    const uint32_t w = 3840, h = 2160, s = 2;
    // A smooth-ish picture with noise, as a render is (a flat black with lines on it), not pure noise.
    std::vector<uint8_t> big(static_cast<std::size_t>(w) * s * h * s * 4u, 0);
    for (std::size_t p = 0; p + 3 < big.size(); p += 4) {
        const uint32_t r = rng();
        if ((r & 0x7u) == 0u) {
            big[p] = static_cast<uint8_t>(r >> 8);
            big[p + 1] = static_cast<uint8_t>(r >> 16);
            big[p + 2] = static_cast<uint8_t>(r >> 24);
        }
        big[p + 3] = 255u;
    }
    EXPECT_EQ(flow_movie::box_down(big, w, h, s), box_down_original(big, w, h, s));
}
