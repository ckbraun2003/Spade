// ---------------------------------------------------------------------------
// scene/library_validate.hpp -- validating a document's TEXT as one of the library's
// file kinds (spade::scene). Slice S3c of the library work (L88i); the shape is the
// contract's `validate` call, docs/design/core/plans/2026-11-17-spade-library-json-
// contract.md section 4.
//
// validate_text() runs the existing loader for the kind on the text and reports what
// it refuses. A document that fails is an ANSWER (valid = false, the loader's problem
// as an issue), not a refusal: a refusal is only a malformed request (an `as` it does
// not know, a relative base directory) or a kind not built yet. The loaders stop at
// their first error, so a failing document has one issue today; the shape already
// carries many, and the version 2 part blocks' validator will add them without
// changing it.
//
// `as` is one of: "model", "object", "world", "part.motor", "part.prop", "part.esc",
// "part.battery", "index" (a version 1 or 2 library.yaml's text; a version 3 root is
// read from disk by open_libraries()). "drone" is not built yet (not_implemented).
// A model with mesh parts reads its sidecars from `base_dir` (an absolute folder);
// without one such a model is invalid, as the loader refuses it.
// ---------------------------------------------------------------------------
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "scene/library_set.hpp"

namespace spade::scene {

// The contract's issue object (section 2). Empty strings stand for null.
struct LibraryIssue {
    std::string code{};  // a Code name: invalid_argument, schema_mismatch, io_error, not_found, ...
    std::string severity = "error";
    std::string kind{};  // the block the problem is in ("motor"), when the loader says
    std::size_t index = 0;
    std::string id{};
    std::string path{};  // the field path, when the loader's message gives one
    std::optional<uint32_t> line{};    // 1-based, when the loader's message carries a position
    std::optional<uint32_t> column{};  // 1-based
    std::string message{};             // the loader's whole message
};

struct ValidateOptions {
    std::string as{};
    std::filesystem::path base_dir{};  // absolute, or empty
};

struct ValidationResult {
    bool valid = false;
    std::vector<LibraryIssue> issues{};
};

// Refused: invalid_request (an `as` that is not one of the eight), relative_path (a base_dir that is not
// absolute), not_implemented ("drone").
[[nodiscard]] LibraryResult<ValidationResult> validate_text(std::string_view text, const ValidateOptions& options);

}  // namespace spade::scene
