// The default library's sources registry and grades (Task L1 of
// docs/design/core/plans/2026-10-21-default-drone-library-plan.md;
// scene/library_sources.hpp). The real registry, assets/library/sources.yaml,
// and its bibliography, SOURCES.md, are read in place; the rules are pinned on
// small texts.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <set>
#include <string>
#include <string_view>

#include "core/error.hpp"
#include "scene/library_sources.hpp"

namespace {

namespace fs = std::filesystem;

using spade::Code;
using spade::scene::check_source_ref;
using spade::scene::Grade;
using spade::scene::grade_name;
using spade::scene::License;
using spade::scene::license_name;
using spade::scene::load_sources;
using spade::scene::parse_grade;
using spade::scene::Source;
using spade::scene::SourceRef;
using spade::scene::SourceRegistry;
using spade::scene::sources_from_yaml;

[[nodiscard]] fs::path library_dir() { return fs::path(SPADE_ASSETS_DIR) / "library"; }

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

// Refused as `code`, naming every needle.
template <typename T>
void expect_refused(const spade::Result<T>& r, Code code, std::initializer_list<std::string_view> needles) {
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, code) << r.error().context;
    for (const std::string_view needle : needles) {
        EXPECT_TRUE(contains(r.error().context, needle)) << "missing '" << needle << "' in: " << r.error().context;
    }
}

// A registry of two sources, one usable and one not, as text.
constexpr std::string_view kTwoSources = R"(sources_version: 1
sources:
  - id: uiuc_propdb
    citation: "UIUC Propeller Database"
    url: "https://m-selig.ae.illinois.edu/props/propDB.html"
    retrieved: 2026-10-21
    license: facts_cite
    license_evidence: "facts, cited"
  - id: apc_data_files
    citation: "APC PE0 and PER files"
    retrieved: 2026-10-21
    license: not_usable
    license_evidence: "redistribution prohibited"
)";

[[nodiscard]] SourceRegistry two_sources() {
    spade::Result<SourceRegistry> r = sources_from_yaml(kTwoSources);
    EXPECT_TRUE(r.has_value()) << r.error().context;
    return r.has_value() ? *r : SourceRegistry{};
}

// kTwoSources with `from` replaced by `to` (once; it must occur).
[[nodiscard]] std::string edited(std::string_view from, std::string_view to) {
    std::string text(kTwoSources);
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) text.replace(at, from.size(), to);
    return text;
}

[[nodiscard]] bool is_date(std::string_view s) {
    if (s.size() != 10 || s[4] != '-' || s[7] != '-') return false;
    for (const std::size_t i : {0u, 1u, 2u, 3u, 5u, 6u, 8u, 9u}) {
        if (s[i] < '0' || s[i] > '9') return false;
    }
    return true;
}

}  // namespace

TEST(LibrarySources, SourcesListEveryLicense) {
    const spade::Result<SourceRegistry> r = load_sources(library_dir() / "sources.yaml");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_FALSE(r->sources.empty());
    for (const Source& s : r->sources) {
        EXPECT_FALSE(s.citation.empty()) << s.id;
        EXPECT_FALSE(s.license_evidence.empty()) << s.id;
        EXPECT_TRUE(is_date(s.retrieved)) << s.id << ": '" << s.retrieved << "'";
        EXPECT_TRUE(s.license == License::public_domain || s.license == License::facts_cite ||
                    s.license == License::not_usable)
            << s.id;
    }
    // The plan's license check, as ruled: NASA is public domain, UIUC is facts
    // cited, and APC and the shapes Kat's catalog took from APC feed nothing.
    const auto license_of = [&](std::string_view id) {
        const Source* s = r->find(id);
        EXPECT_NE(s, nullptr) << id;
        return s != nullptr ? s->license : License::not_usable;
    };
    EXPECT_EQ(license_of("nasa_ames_2016"), License::public_domain);
    EXPECT_EQ(license_of("uiuc_propdb"), License::facts_cite);
    EXPECT_EQ(license_of("kat_competition_700"), License::facts_cite);
    EXPECT_EQ(license_of("apc_data_files"), License::not_usable);
    EXPECT_EQ(license_of("kat_catalog_apc_shaped"), License::not_usable);
}

TEST(LibrarySources, TheBibliographyListsEverySource) {
    const spade::Result<SourceRegistry> r = load_sources(library_dir() / "sources.yaml");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    std::ifstream in(library_dir() / "SOURCES.md", std::ios::binary);
    ASSERT_TRUE(in) << "SOURCES.md";
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    // Every "### `<id>`" heading, in order.
    std::set<std::string> headings;
    constexpr std::string_view kHeading = "\n### `";
    for (std::size_t at = text.find(kHeading); at != std::string::npos; at = text.find(kHeading, at + 1)) {
        const std::size_t begin = at + kHeading.size();
        const std::size_t end = text.find('`', begin);
        ASSERT_NE(end, std::string::npos);
        headings.insert(text.substr(begin, end - begin));
    }
    std::set<std::string> ids;
    for (const Source& s : r->sources) ids.insert(s.id);
    EXPECT_FALSE(ids.empty());
    EXPECT_EQ(headings, ids) << "SOURCES.md's headings and sources.yaml's ids must be the same set";
}

TEST(LibrarySources, ALicenseOutsideTheThreeIsRefused) {
    expect_refused(sources_from_yaml(edited("license: facts_cite", "license: cc_by")), Code::invalid_argument,
                   {"sources[0].license", "'cc_by'", "public_domain", "facts_cite", "not_usable"});
    expect_refused(sources_from_yaml(edited("    license_evidence: \"facts, cited\"\n", "")), Code::invalid_argument,
                   {"missing required key 'license_evidence'", "sources[0]"});
    expect_refused(sources_from_yaml(edited("license_evidence: \"facts, cited\"", "license_evidence: \"\"")),
                   Code::invalid_argument, {"sources[0].license_evidence must not be empty"});
}

TEST(LibrarySources, IdsAreUniqueAndLowercase) {
    expect_refused(sources_from_yaml(edited("id: apc_data_files", "id: uiuc_propdb")), Code::invalid_argument,
                   {"sources[1].id 'uiuc_propdb'", "sources[0]"});
    expect_refused(sources_from_yaml(edited("id: apc_data_files", "id: APC-files")), Code::invalid_argument,
                   {"sources[1].id 'APC-files'", "[a-z0-9_]"});
    expect_refused(sources_from_yaml(edited("retrieved: 2026-10-21\n    license: not_usable",
                                            "retrieved: last week\n    license: not_usable")),
                   Code::invalid_argument, {"sources[1].retrieved 'last week'", "YYYY-MM-DD"});
}

TEST(LibrarySources, UnknownKeysAreRefusedBlamedOnTheSchema) {
    expect_refused(sources_from_yaml(edited("    citation: \"APC PE0 and PER files\"\n",
                                            "    citation: \"APC PE0 and PER files\"\n    grade: measured\n")),
                   Code::invalid_argument, {"unknown key 'grade'", "sources[1]", "sources schema v1"});
    expect_refused(sources_from_yaml(edited("sources_version: 1", "sources_version: 2")), Code::schema_mismatch,
                   {"schema version 2", "reads version 1"});
}

TEST(LibrarySources, UnknownGradeIsRefused) {
    for (const Grade g : {Grade::measured, Grade::published, Grade::retailer, Grade::derived, Grade::fitted,
                          Grade::assumed}) {
        const spade::Result<Grade> back = parse_grade(grade_name(g), "the grade");
        ASSERT_TRUE(back.has_value()) << grade_name(g);
        EXPECT_EQ(*back, g) << grade_name(g);
    }
    EXPECT_EQ(grade_name(Grade::retailer), "retailer");
    EXPECT_EQ(license_name(License::facts_cite), "facts_cite");
    expect_refused(parse_grade("guessed", "prop.ct_static's grade"), Code::invalid_argument,
                   {"prop.ct_static's grade", "'guessed'", "measured", "published", "retailer", "derived", "fitted",
                    "assumed"});
    expect_refused(parse_grade("Measured", "the grade"), Code::invalid_argument, {"'Measured'"});
}

TEST(LibrarySources, UnusableSourcesFeedNothing) {
    const SourceRegistry registry = two_sources();
    EXPECT_TRUE(check_source_ref(registry, SourceRef{"uiuc_propdb", Grade::measured, ""}, "prop.ct_table").has_value());
    expect_refused(check_source_ref(registry, SourceRef{"apc_data_files", Grade::measured, ""}, "prop.ct_table"),
                   Code::invalid_argument, {"prop.ct_table", "'apc_data_files'", "not_usable", "feeds no number"});
    // Not even as the source of an assumed or a derived value.
    expect_refused(check_source_ref(registry, SourceRef{"apc_data_files", Grade::assumed, "shape only"}, "prop.beta"),
                   Code::invalid_argument, {"prop.beta", "'apc_data_files'", "not_usable"});
}

TEST(LibrarySources, AnUnlistedSourceIsRefusedByName) {
    const SourceRegistry registry = two_sources();
    expect_refused(check_source_ref(registry, SourceRef{"nasa_tm_2018", Grade::measured, ""}, "parts[2].mass"),
                   Code::invalid_argument, {"parts[2].mass", "'nasa_tm_2018'", "does not list"});
}

TEST(LibrarySources, OnlyAnAssumedValueMayCiteNoSource) {
    const SourceRegistry registry = two_sources();
    EXPECT_TRUE(check_source_ref(registry, SourceRef{"", Grade::assumed, "a typical 6S pack"}, "battery.cell_resistance")
                    .has_value());
    for (const Grade g : {Grade::measured, Grade::published, Grade::retailer, Grade::derived, Grade::fitted}) {
        expect_refused(check_source_ref(registry, SourceRef{"", g, "a note"}, "motor.kv"), Code::invalid_argument,
                       {"motor.kv", "cites no source", grade_name(g)});
    }
    expect_refused(check_source_ref(registry, SourceRef{"", Grade::assumed, ""}, "battery.cell_resistance"),
                   Code::invalid_argument, {"battery.cell_resistance", "assumed", "note"});
    expect_refused(check_source_ref(registry, SourceRef{"uiuc_propdb", Grade::derived, ""}, "prop.diameter"),
                   Code::invalid_argument, {"prop.diameter", "derived", "note"});
}
