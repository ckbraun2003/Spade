#include "render/target.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

namespace spade::render {
namespace {

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

}  // namespace

Result<void> validate_target(const RenderTarget& target) {
    if (target.width == 0) {
        return std::unexpected(invalid("render target width must be > 0"));
    }
    if (target.height == 0) {
        return std::unexpected(invalid("render target height must be > 0"));
    }
    if (target.stride != target.width * 4) {
        return std::unexpected(invalid("render target stride must equal width * 4"));
    }
    if (target.pixels.size() != static_cast<std::size_t>(target.stride) * target.height) {
        return std::unexpected(invalid("render target pixels must be exactly stride * height bytes"));
    }
    if (target.format != PixelFormat::bgrx8) {
        return std::unexpected(invalid("render target has an unrecognized pixel format"));
    }
    return {};
}

Result<void> validate_line_options(const RenderOptions& options) {
    if (options.slot >= 32u) {
        return std::unexpected(invalid("render options: the slot must be 0 to 31"));
    }
    if (!(std::isfinite(options.ui_scale) && options.ui_scale >= 0.25f && options.ui_scale <= 8.0f)) {
        return std::unexpected(invalid("render options: ui_scale must be finite, 0.25 to 8"));
    }
    return {};
}

}  // namespace spade::render
