// How the editor names a file on screen (the status line after Save selection to library). Display-free (SL15b).
//
// A path on this machine must not be drawn: it names a user's folders in a screenshot or a video. A file INSIDE the library folder is shown relative
// to it ("drones/hover_copy.yaml", forward slashes). Any other file is shown by its FILE NAME alone. Never a "..\" chain, which can walk out into the
// user's folders. STUB: the full path, as the editor drew it until now (RED).

#pragma once

#include <filesystem>
#include <string>

namespace spade::sandbox::editor {

[[nodiscard]] inline std::string shown_file(const std::filesystem::path& file, const std::filesystem::path& /*library_dir*/) {
    return file.generic_string();
}

}  // namespace spade::sandbox::editor
