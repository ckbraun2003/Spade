#pragma once

// ---------------------------------------------------------------------------
// library_c/library_json.hpp -- the library contract's JSON calls over spade::scene (L88l, S4;
// docs/design/core/plans/2026-11-17-spade-library-json-contract.md). The C ABI (spade_library.h, c_api.cpp) is a thin layer over this:
// text in, text out, no exception out. The tests drive the same functions the ABI does.
// ---------------------------------------------------------------------------

#include <memory>
#include <string>
#include <string_view>

namespace spade::libc {

inline constexpr std::string_view kFormat = "spade.library/1";
inline constexpr int kMinor = 0;

// An opened set of roots. Defined in library_json.cpp.
class Library;
struct LibraryDeleter {
    void operator()(Library* lib) const;
};
using LibraryPtr = std::unique_ptr<Library, LibraryDeleter>;

// What a call answers: the ABI's return value (0 ok, 1 refused, 2 misuse) and the response text.
struct Answer {
    int rc = 0;
    std::string json{};
};

// `open`: on rc 0, `out` is the handle; else it is null.
[[nodiscard]] Answer open_library(std::string_view request_json, LibraryPtr& out);

// Any other op on an open handle. Never throws.
[[nodiscard]] Answer call_library(const Library& lib, std::string_view op, std::string_view request_json);

// ABI misuse (a null handle or argument): rc 2 and the invalid_request refusal "no library is open".
[[nodiscard]] Answer misuse(std::string_view op);

}  // namespace spade::libc
