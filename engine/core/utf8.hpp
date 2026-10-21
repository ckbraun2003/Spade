// utf8.hpp -- whether text is well-formed UTF-8 (RFC 3629): no overlong form,
// no surrogate, nothing past U+10FFFF, no sequence cut short.
//
// THE ONE CHECK for text read from a file into a name that crosses a Spade
// API (TD-9): such names are UTF-8, as paths are (core/path_text.hpp).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace spade {

[[nodiscard]] constexpr bool is_utf8(std::string_view s) noexcept {
    for (std::size_t i = 0; i < s.size();) {
        const auto b = static_cast<unsigned char>(s[i]);
        if (b < 0x80u) {
            ++i;
            continue;
        }
        std::size_t n = 0;  // continuation bytes
        uint32_t cp = 0;
        if ((b & 0xE0u) == 0xC0u) {
            n = 1;
            cp = b & 0x1Fu;
        } else if ((b & 0xF0u) == 0xE0u) {
            n = 2;
            cp = b & 0x0Fu;
        } else if ((b & 0xF8u) == 0xF0u) {
            n = 3;
            cp = b & 0x07u;
        } else {
            return false;
        }
        if (s.size() - i <= n) return false;  // cut short
        for (std::size_t k = 1; k <= n; ++k) {
            const auto c = static_cast<unsigned char>(s[i + k]);
            if ((c & 0xC0u) != 0x80u) return false;
            cp = (cp << 6) | (c & 0x3Fu);
        }
        constexpr uint32_t kLeast[] = {0, 0x80, 0x800, 0x10000};  // the least code point n continuations encode
        if (cp < kLeast[n] || cp > 0x10FFFFu || (cp >= 0xD800u && cp <= 0xDFFFu)) return false;
        i += n + 1;
    }
    return true;
}

}  // namespace spade
