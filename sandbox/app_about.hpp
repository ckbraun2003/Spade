// WHAT THE APP SAYS ABOUT ITSELF (L73a item 2; the lead, 2026-11-11): the line --version prints and the Help > About
// Spade Editor text. The version string itself is Core's generated spade_version.hpp (spade::version::kString,
// "0.1.0+<sha>"); main.cpp hands it here, so these are display-free and tested without it.

#pragma once

#include <string>
#include <string_view>

namespace spade::sandbox::app {

// `spade_sandbox --version` prints this and exits 0.
[[nodiscard]] inline std::string version_line(std::string_view version) {
    return "Spade Editor " + std::string(version);
}

// Help > About Spade Editor: the version, the licence, and where the third-party notices are.
[[nodiscard]] inline std::string about_text(std::string_view version) {
    return version_line(version) +
           "\n\nLicensed under the Apache License, Version 2.0.\n"
           "The libraries Spade Editor is built with, and their licences, are listed in THIRD_PARTY_NOTICES.md, in the "
           "folder Spade Editor is installed in.";
}

}  // namespace spade::sandbox::app
