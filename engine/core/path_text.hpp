// path_text.hpp -- a filesystem path as UTF-8 text, for error messages.
//
// THE ONE HELPER for every message that names a file (TD-9). UTF-8, because
// consumers treat every path across their interfaces as UTF-8 (Kat's host
// does, through its C interfaces). path::string() converts to the native narrow
// encoding instead, which on Windows is the ANSI code page: it garbles a path
// with e-acute in it and THROWS for one with a CJK character -- out of
// functions that return Result. path::u8string() is UTF-8 on every platform.
//
// Never throws: an error message is never worth an exception, and none may
// cross a Spade API. A path the conversion cannot represent (a lone UTF-16
// surrogate on Windows) prints as "<unprintable path>".
#pragma once

#include <filesystem>
#include <string>

namespace spade {

[[nodiscard]] inline std::string path_text(const std::filesystem::path& path) noexcept {
    try {
        const std::u8string text = path.u8string();
        return std::string(text.begin(), text.end());
    } catch (...) {
        return "<unprintable path>";
    }
}

}  // namespace spade
