// ---------------------------------------------------------------------------
// scene/library_sources.hpp -- the default library's sources and grades
// (spade::scene; Task L1 of docs/design/core/plans/2026-10-21-default-drone-
// library-plan.md).
//
// Every number in a library entry cites a source and a grade (the plan's
// Global Constraints). The sources are one registry, assets/library/
// sources.yaml, with its bibliography for people beside it (SOURCES.md, one
// "### `<id>`" heading per source; tests/test_library_sources.cpp keeps the
// two in step). An entry cites a source by its id. This header is the
// registry's reader and the one spelling of the grades and the licenses
// (TD-9): the part files (L2) and the drones' sidecars (L5) read and write
// grades through it.
//
// THE TEXT, top-level keys in this order:
//   sources_version: 1
//   sources:   a sequence, one mapping per source:
//     id:               [a-z0-9_]+, unique; what an entry cites
//     citation:         non-empty: as the source asks to be cited, or authors,
//                       title, venue and year
//     url:              OPTIONAL; leave the key out for a source with none
//                       (Kat's files, read in place)
//     retrieved:        the date it was read, YYYY-MM-DD
//     license:          public_domain | facts_cite | not_usable
//     license_evidence: non-empty: the words or the ruling that decided it
// Unknown, missing and duplicate keys are refused at every level, blamed on
// "sources schema v1", with the dotted path ("sources[2].license") and the
// line and column, as the library index's are. A sources_version this build
// does not read is schema_mismatch, read before any other key.
//
// LICENSES. public_domain: no copyright (a US Government work). facts_cite:
// the numbers are facts, used and cited, never the source's own file.
// not_usable: the source forbids redistribution (APC's terms; ruling 4), so it
// feeds no number. It stays listed, so a citation of it is refused by name,
// not as an unknown id.
//
// GRADES, strongest first:
//   measured   a test stand or a tunnel, in a primary source
//   published  the manufacturer's own specification
//   retailer   a seller's or a third party's listing; weaker, and named so
//   derived    computed from cited numbers by code in this repo, which the
//              note names
//   fitted     fitted against data (Kat's flight logs of the VQ1 simulator)
//   assumed    a plausible default with no source for this airframe
//
// A CITATION (SourceRef) is a source id, a grade and a note.
// check_source_ref() is its one rule: every grade but `assumed` cites a listed,
// usable source; an `assumed` value may cite none. A `derived` or an `assumed`
// value says in its note how it was made or why.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"

namespace spade::scene {

// The sources-registry schema this build reads (the only one so far).
inline constexpr uint32_t kSourcesVersion = 1;

enum class License : uint32_t {
    public_domain = 0,
    facts_cite = 1,
    not_usable = 2,
};

enum class Grade : uint32_t {
    measured = 0,
    published = 1,
    retailer = 2,
    derived = 3,
    fitted = 4,
    assumed = 5,
};

// The text spelling of each: what the registry, the part files and the
// sidecars write, and what parse_grade() reads.
[[nodiscard]] std::string_view license_name(License license) noexcept;
[[nodiscard]] std::string_view grade_name(Grade grade) noexcept;

// Text -> Grade: one of the six names, exactly. invalid_argument otherwise,
// naming `what`, the text and the six.
[[nodiscard]] Result<Grade> parse_grade(std::string_view text, std::string_view what);

// One source, as the registry spells it.
struct Source {
    std::string id{};
    std::string citation{};
    std::string url{};  // empty when the source has none
    std::string retrieved{};
    License license = License::not_usable;
    std::string license_evidence{};
};

// The registry: its sources in file order.
struct SourceRegistry {
    std::vector<Source> sources{};

    // The source with this id, or nullptr.
    [[nodiscard]] const Source* find(std::string_view id) const noexcept;
};

// A citation: what an entry's field carries.
struct SourceRef {
    std::string source{};  // a registry id; empty only for an assumed value
    Grade grade = Grade::assumed;
    std::string note{};
};

// Text -> SourceRegistry, no disk. Codes: schema_mismatch for a
// sources_version this build does not read; invalid_argument for every other
// rule of THE TEXT above, with its dotted path and its line and column.
[[nodiscard]] Result<SourceRegistry> sources_from_yaml(std::string_view text);

// Reads the registry at `path` and runs sources_from_yaml(). io_error when it
// cannot be read; a parse diagnostic comes back prefixed with the path.
[[nodiscard]] Result<SourceRegistry> load_sources(const std::filesystem::path& path);

// The one rule for a citation (see A CITATION above), for the entry's field
// `field` ("prop.ct_static", "parts[2].mass"). invalid_argument naming the
// field and the source id: for an id the registry does not list, for a
// not_usable source, for a grade other than assumed that cites no source, and
// for a derived or an assumed value with no note.
[[nodiscard]] Result<void> check_source_ref(const SourceRegistry& registry, const SourceRef& ref,
                                            std::string_view field);

}  // namespace spade::scene
