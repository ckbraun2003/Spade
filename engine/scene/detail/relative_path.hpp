#pragma once

// ---------------------------------------------------------------------------
// scene/detail/relative_path.hpp -- the scene's one rule for a path a file
// names relative to its own directory (TD-9): a scene's world file, a mesh
// part's sidecar (the scene's and the model file's), and the library index's
// entries (scene/library.hpp). Defined in scene_file.cpp, where the rule was
// born.
//
// INTERNAL, NOT INSTALLED: the install rule for scene/ excludes detail/.
// ---------------------------------------------------------------------------

#include <string>
#include <string_view>

namespace spade::scene::detail {

// What is wrong with `file` as a path relative to the <kind> file's directory
// with forward slashes, or empty: a backslash, or a leading '/' or any ':'
// ("must be relative to the <kind> file"). `file` is not empty.
[[nodiscard]] std::string relative_path_problem(std::string_view file, std::string_view kind);

}  // namespace spade::scene::detail
