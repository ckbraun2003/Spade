// path_text.hpp -- a filesystem path as text, for error messages.
#pragma once

#include <filesystem>
#include <string>

namespace spade {

[[nodiscard]] inline std::string path_text(const std::filesystem::path& path) noexcept {
    try {
        return path.string();
    } catch (...) {
        return "<unprintable path>";
    }
}

}  // namespace spade
