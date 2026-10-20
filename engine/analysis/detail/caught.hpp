// The refusal for an exception a reader caught (Spade Host's finding: an
// out-of-memory read as a broken file). Internal to spade::analysis: not
// installed.

#pragma once

#include <exception>
#include <new>
#include <string>
#include <string_view>

#include "core/error.hpp"

namespace spade::analysis::detail {

// Out of memory is capacity_exceeded, "out of memory" (the glTF importer's
// precedent), never `otherwise`: an io_error or a parse failure names the
// file, and the file is not at fault. Any other exception is `otherwise`,
// with its what() when it has one. `doing` says what was being done
// ("reading case file 'a.case.yaml'").
[[nodiscard]] inline Error caught_error(std::exception_ptr caught, Code otherwise, std::string_view doing) {
    try {
        std::rethrow_exception(caught);
    } catch (const std::bad_alloc&) {
        return Error{Code::capacity_exceeded, "out of memory " + std::string(doing)};
    } catch (const std::exception& e) {
        return Error{otherwise, std::string(doing) + ": " + e.what()};
    } catch (...) {
        return Error{otherwise, std::string(doing) + ": an unknown exception"};
    }
}

}  // namespace spade::analysis::detail
