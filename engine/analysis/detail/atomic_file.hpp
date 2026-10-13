// Whole-file writes a reader never sees half of, and reads that let them
// happen (INT-7). Internal to spade::analysis: not installed.

#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "core/error.hpp"

namespace spade::analysis::detail {

// Writes `text` to a temporary file beside `file`, then moves it over `file`
// in one step: MoveFileExW(MOVEFILE_REPLACE_EXISTING) on Windows, rename()
// elsewhere. On Windows a reader that opened `file` without FILE_SHARE_DELETE
// blocks the move, so it is retried for up to about a second before it is
// refused; the temporary file never outlives the call.
[[nodiscard]] Result<void> replace_file(const std::filesystem::path& file, std::string_view text);

// The whole of `file`, opened with read, write and delete sharing on Windows
// so a replace_file() may move over it meanwhile (the reader keeps the
// contents it opened). A missing file is an io_error.
[[nodiscard]] Result<std::string> read_file_shared(const std::filesystem::path& file);

}  // namespace spade::analysis::detail
