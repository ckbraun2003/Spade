// How the editor names a file on screen (the status line after Save selection to library). Display-free (SL15b).
//
// A path on this machine must not be drawn: it names a user's folders in a screenshot or a video. A file INSIDE the library folder is shown relative
// to it ("drones/hover_copy.yaml", forward slashes). Any other file is shown by its FILE NAME alone. Never a "..\" chain, which can walk out into the
// user's folders.

#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace spade::sandbox::editor {

[[nodiscard]] inline std::string shown_file(const std::filesystem::path& file, const std::filesystem::path& library_dir) {
    const std::filesystem::path relative = file.lexically_normal().lexically_relative(library_dir.lexically_normal());
    const bool inside = !relative.empty() && relative != "." && *relative.begin() != "..";
    return inside ? relative.generic_string() : file.filename().generic_string();
}

// The inspector's line for the file an imported asset's mesh came from: the file's NAME alone, never the folders it sat in on the user's machine
// (the same rule as the title). Split on both kinds of separator, so a Windows path is cut the same on every platform. The panels draw this and the
// live smoke's path watch reads this (drawn_texts), so the watch sees what is drawn.
[[nodiscard]] inline std::string imported_from_text(std::string_view source) {
    return "imported from " + std::string(source);  // RED: the whole path, as drawn before
}

}  // namespace spade::sandbox::editor
