// ---------------------------------------------------------------------------
// text_fit.hpp -- where a text too long for its cell is cut from the left.
//
// A field path's end tells rows apart ("rotors[0].hub.position.x", ".y",
// ".z"), so its cell keeps the end: the ellipsis first, then the longest tail
// that fits (the lead, 2026-10-26). ui_theme.hpp's cell_path() draws it; the
// cut itself is display-free, so spade_tests checks it with no window, given
// any width function (SL15b).
// ---------------------------------------------------------------------------
#pragma once

#include <cstddef>
#include <string_view>

namespace spade::sandbox::ui {

// Where the kept tail of `text` begins when it is cut from the left to fit
// `width`, `ellipsis` wide in front of it: 0 when the whole fits; else the
// start of the longest tail that fits after the ellipsis; else the last
// character's, so something is always shown. Never inside a UTF-8 sequence.
// `measure(std::string_view)` gives a text's width.
template <class Measure>
[[nodiscard]] std::size_t left_cut(std::string_view text, float width, float ellipsis, Measure&& measure) {
    const auto continuation = [&](std::size_t i) { return (static_cast<unsigned char>(text[i]) & 0xC0u) == 0x80u; };
    if (text.empty() || measure(text) <= width) return 0;
    std::size_t last = 0;
    for (std::size_t i = 1; i < text.size(); ++i) {
        if (continuation(i)) continue;
        if (ellipsis + measure(text.substr(i)) <= width) return i;
        last = i;
    }
    return last;
}

}  // namespace spade::sandbox::ui
