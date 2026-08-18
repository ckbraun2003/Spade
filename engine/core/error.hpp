#pragma once

#include <expected>
#include <string>

namespace spade {

// Error taxonomy for the public API. Kept small and closed (not an
// open-ended string code) so callers can switch on it; `context` carries the
// human-readable detail a log line or diagnostic wants.
enum class Code {
    invalid_argument,
    capacity_exceeded,
    not_found,
    io_error,
    schema_mismatch,
    internal,
    // S6 Task 1: a requested resource is architecturally absent from this
    // process/environment rather than merely missing data (not_found) or
    // malformed input (invalid_argument) -- e.g. compute::VulkanContext::
    // create() when no Vulkan loader or physical device is present. Callers
    // that can degrade or skip (a GPU test's GTEST_SKIP(), a future runtime
    // backend fallback) switch on this specifically rather than treating it
    // as a generic failure.
    unavailable,
};

// Deliberately a plain aggregate (no user-declared constructors) so
// `Error{Code::not_found, "world.yaml"}` aggregate-initializes.
struct Error {
    Code code;
    std::string context;

    friend bool operator==(const Error&, const Error&) = default;
};

// Public API return type: no exceptions cross module boundaries (see the
// engine design spec's C++23 rationale). `T` is the success payload;
// failure carries a spade::Error.
template <class T>
using Result = std::expected<T, Error>;

}  // namespace spade
