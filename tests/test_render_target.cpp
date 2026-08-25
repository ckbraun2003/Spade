#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <vector>

#include "render/target.hpp"

// ---------------------------------------------------------------------------
// validate_target() -- the render/target.hpp contract (S7a Task 0): stride
// must equal width * 4, pixels.size() must equal stride * height exactly,
// and both width and height must be nonzero. Each rejection case below
// mutates exactly ONE field away from a well-formed baseline (keeping every
// other check satisfied) so a failure names precisely which rule tripped,
// independent of validate_target()'s internal check order.
// ---------------------------------------------------------------------------

namespace {

constexpr uint32_t kWidth = 4;
constexpr uint32_t kHeight = 2;
constexpr uint32_t kStride = kWidth * 4;  // bgrx8: 4 bytes/pixel

[[nodiscard]] spade::render::RenderTarget well_formed_target(std::vector<uint8_t>& storage) {
    storage.assign(static_cast<size_t>(kStride) * kHeight, 0);
    return spade::render::RenderTarget{
        .pixels = std::span<uint8_t>(storage),
        .width = kWidth,
        .height = kHeight,
        .stride = kStride,
        .format = spade::render::PixelFormat::bgrx8,
    };
}

}  // namespace

TEST(ValidateTarget, AcceptsWellFormedTarget) {
    std::vector<uint8_t> storage;
    const spade::render::RenderTarget target = well_formed_target(storage);

    const spade::Result<void> result = spade::render::validate_target(target);
    EXPECT_TRUE(result.has_value());
}

TEST(ValidateTarget, RejectsStrideNotEqualToWidthTimesFour) {
    std::vector<uint8_t> storage;
    spade::render::RenderTarget target = well_formed_target(storage);
    target.stride = kWidth * 4 + 4;  // wrong stride
    // Keep pixels sized to the (now wrong) stride*height so this case isolates
    // the stride == width*4 check rather than also tripping the size check.
    storage.assign(static_cast<size_t>(target.stride) * target.height, 0);
    target.pixels = std::span<uint8_t>(storage);

    const spade::Result<void> result = spade::render::validate_target(target);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::invalid_argument);
}

TEST(ValidateTarget, RejectsPixelsSizeMismatch) {
    std::vector<uint8_t> storage;
    spade::render::RenderTarget target = well_formed_target(storage);
    storage.assign(static_cast<size_t>(kStride) * kHeight - 1, 0);  // one byte short
    target.pixels = std::span<uint8_t>(storage);

    const spade::Result<void> result = spade::render::validate_target(target);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::invalid_argument);
}

TEST(ValidateTarget, RejectsZeroWidth) {
    std::vector<uint8_t> storage;
    spade::render::RenderTarget target = well_formed_target(storage);
    target.width = 0;
    target.stride = 0;  // stride == width*4 must still hold, to isolate width==0
    storage.assign(0, 0);
    target.pixels = std::span<uint8_t>(storage);

    const spade::Result<void> result = spade::render::validate_target(target);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::invalid_argument);
}

TEST(ValidateTarget, RejectsZeroHeight) {
    std::vector<uint8_t> storage;
    spade::render::RenderTarget target = well_formed_target(storage);
    target.height = 0;
    storage.assign(0, 0);  // stride*height == 0, to isolate height==0
    target.pixels = std::span<uint8_t>(storage);

    const spade::Result<void> result = spade::render::validate_target(target);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::invalid_argument);
}

TEST(ValidateTarget, RejectsUnrecognizedFormat) {
    // validate_target()'s own contract comment says "stride/size/format
    // sanity" -- PixelFormat's only defined enumerator today is bgrx8 (1), so
    // 0 is a value no authored PixelFormat ever holds.
    std::vector<uint8_t> storage;
    spade::render::RenderTarget target = well_formed_target(storage);
    target.format = static_cast<spade::render::PixelFormat>(0);

    const spade::Result<void> result = spade::render::validate_target(target);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::invalid_argument);
}
