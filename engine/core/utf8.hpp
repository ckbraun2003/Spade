// utf8.hpp -- whether text is well-formed UTF-8 (RFC 3629): no overlong form,
// no surrogate, nothing past U+10FFFF, no sequence cut short.
//
// THE ONE CHECK for text read from a file into a name that crosses a Spade
// API (TD-9): such names are UTF-8, as paths are (core/path_text.hpp).
#pragma once

#include <string_view>

namespace spade {

[[nodiscard]] constexpr bool is_utf8(std::string_view s) noexcept {
    (void)s;
    return true;  // RED: not built yet
}

}  // namespace spade
