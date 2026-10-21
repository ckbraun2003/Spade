#include "scene/library_sources.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "world/detail/yaml_text.hpp"

namespace spade::scene {
namespace {

using yaml_text::at;
using yaml_text::dec;
using yaml_text::field;
using yaml_text::has;
using yaml_text::mark_of;
using yaml_text::parse_uint;
using yaml_text::path_text;
using yaml_text::scalar;

// Every diagnostic blames the registry's own schema.
constexpr std::string_view kSchema = "sources schema v1";
constexpr std::string_view kTop = "sources registry";

// The spellings, in enum order (TD-9: the only place each is written).
constexpr std::array<std::string_view, 3> kLicenseNames = {"public_domain", "facts_cite", "not_usable"};
constexpr std::array<std::string_view, 6> kGradeNames = {"measured", "published", "retailer",
                                                         "derived",  "fitted",    "assumed"};

// "a, b or c", for a diagnostic.
template <std::size_t N>
[[nodiscard]] std::string one_of(const std::array<std::string_view, N>& names) {
    std::string out;
    for (std::size_t i = 0; i < N; ++i) {
        if (i > 0) out += i + 1 == N ? " or " : ", ";
        out += names[i];
    }
    return out;
}

// The non-empty string at `node[key]` (required), refused by name when empty.
[[nodiscard]] Result<std::string> non_empty_field(const YAML::Node& node, const char* key, const std::string& what) {
    const Result<YAML::Node> value = field(node, key, what);
    if (!value) return std::unexpected(value.error());
    const std::string where = what + "." + key;
    Result<std::string> text = scalar(*value, where);
    if (!text) return text;
    if (text->empty()) return std::unexpected(at(*value, where + " must not be empty"));
    return text;
}

[[nodiscard]] bool is_id(std::string_view id) {
    if (id.empty()) return false;
    for (const char c : id) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    }
    return true;
}

[[nodiscard]] bool is_date(std::string_view s) {
    if (s.size() != 10 || s[4] != '-' || s[7] != '-') return false;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (i == 4 || i == 7) continue;
        if (s[i] < '0' || s[i] > '9') return false;
    }
    return true;
}

[[nodiscard]] Result<SourceRegistry> parse_registry(const YAML::Node& root) {
    if (!root.IsMap()) {
        return std::unexpected(
            at(root, "a sources registry's top level must be a mapping with a 'sources_version' key"));
    }

    // VERSION FIRST, before the unknown-key sweep (the scene's rule).
    const Result<YAML::Node> version_node = field(root, "sources_version", kTop);
    if (!version_node) return std::unexpected(version_node.error());
    const Result<uint32_t> version = parse_uint<uint32_t>(*version_node, "sources_version");
    if (!version) return std::unexpected(version.error());
    if (*version != kSourcesVersion) {
        return std::unexpected(Error{Code::schema_mismatch, "sources registry is schema version " + dec(*version) +
                                                                ", this build reads version " + dec(kSourcesVersion) +
                                                                " only" + mark_of(*version_node)});
    }
    if (Result<void> r = yaml_text::check_map(root, kTop, {"sources_version", "sources"}, kSchema); !r) {
        return std::unexpected(r.error());
    }

    const Result<YAML::Node> sources = field(root, "sources", kTop);
    if (!sources) return std::unexpected(sources.error());
    if (!sources->IsSequence()) return std::unexpected(at(*sources, "sources must be a sequence"));

    SourceRegistry out{};
    out.sources.reserve(sources->size());
    for (std::size_t i = 0; i < sources->size(); ++i) {
        const YAML::Node node = (*sources)[i];
        const std::string what = "sources[" + dec(i) + "]";
        if (Result<void> r = yaml_text::check_map(
                node, what, {"id", "citation", "url", "retrieved", "license", "license_evidence"}, kSchema);
            !r) {
            return std::unexpected(r.error());
        }
        Source source{};

        Result<std::string> id = non_empty_field(node, "id", what);
        if (!id) return std::unexpected(id.error());
        if (!is_id(*id)) {
            return std::unexpected(at(node["id"], what + ".id '" + *id + "' must be [a-z0-9_]+: lower-case ASCII "
                                                                   "letters, digits and underscores"));
        }
        for (std::size_t j = 0; j < out.sources.size(); ++j) {
            if (out.sources[j].id == *id) {
                return std::unexpected(at(node["id"], what + ".id '" + *id + "' is sources[" + dec(j) +
                                                          "]'s id; ids are unique"));
            }
        }
        source.id = std::move(*id);

        Result<std::string> citation = non_empty_field(node, "citation", what);
        if (!citation) return std::unexpected(citation.error());
        source.citation = std::move(*citation);

        // url: optional; absent is none.
        if (has(node, "url")) {
            Result<std::string> url = non_empty_field(node, "url", what);
            if (!url) return std::unexpected(url.error());
            source.url = std::move(*url);
        }

        Result<std::string> retrieved = non_empty_field(node, "retrieved", what);
        if (!retrieved) return std::unexpected(retrieved.error());
        if (!is_date(*retrieved)) {
            return std::unexpected(
                at(node["retrieved"], what + ".retrieved '" + *retrieved + "' must be a date, YYYY-MM-DD"));
        }
        source.retrieved = std::move(*retrieved);

        Result<std::string> license = non_empty_field(node, "license", what);
        if (!license) return std::unexpected(license.error());
        std::size_t k = 0;
        while (k < kLicenseNames.size() && kLicenseNames[k] != *license) ++k;
        if (k == kLicenseNames.size()) {
            return std::unexpected(at(node["license"], what + ".license '" + *license + "' must be " +
                                                           one_of(kLicenseNames)));
        }
        source.license = static_cast<License>(k);

        Result<std::string> evidence = non_empty_field(node, "license_evidence", what);
        if (!evidence) return std::unexpected(evidence.error());
        source.license_evidence = std::move(*evidence);

        out.sources.push_back(std::move(source));
    }
    return out;
}

}  // namespace

std::string_view license_name(License license) noexcept {
    const auto i = static_cast<std::size_t>(license);
    return i < kLicenseNames.size() ? kLicenseNames[i] : std::string_view{};
}

std::string_view grade_name(Grade grade) noexcept {
    const auto i = static_cast<std::size_t>(grade);
    return i < kGradeNames.size() ? kGradeNames[i] : std::string_view{};
}

Result<Grade> parse_grade(std::string_view text, std::string_view what) {
    for (std::size_t i = 0; i < kGradeNames.size(); ++i) {
        if (kGradeNames[i] == text) return static_cast<Grade>(i);
    }
    return std::unexpected(Error{Code::invalid_argument, std::string(what) + " '" + std::string(text) +
                                                             "' is not a grade: it must be " + one_of(kGradeNames)});
}

const Source* SourceRegistry::find(std::string_view id) const noexcept {
    for (const Source& s : sources) {
        if (s.id == id) return &s;
    }
    return nullptr;
}

// yaml-cpp reports every syntax failure by throwing; nothing may leave a Spade
// API that way (the scene file's rule).
Result<SourceRegistry> sources_from_yaml(std::string_view text) {
    try {
        const YAML::Node root = YAML::Load(std::string(text));
        return parse_registry(root);
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, std::string("sources registry parse failed: ") + e.what()});
    }
}

Result<SourceRegistry> load_sources(const std::filesystem::path& path) {
    std::string text;
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return std::unexpected(Error{Code::io_error, "cannot open sources registry '" + path_text(path) + "'"});
        }
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) {
            return std::unexpected(Error{Code::io_error, "failed reading sources registry '" + path_text(path) + "'"});
        }
    } catch (const std::exception& e) {
        return std::unexpected(
            Error{Code::io_error, "failed reading sources registry '" + path_text(path) + "': " + e.what()});
    }
    Result<SourceRegistry> r = sources_from_yaml(text);
    if (!r) return std::unexpected(Error{r.error().code, path_text(path) + ": " + r.error().context});
    return r;
}

Result<void> check_source_ref(const SourceRegistry& registry, const SourceRef& ref, std::string_view field_path) {
    const std::string field(field_path);
    const std::string_view grade = grade_name(ref.grade);
    if (grade.empty()) {
        return std::unexpected(Error{Code::invalid_argument, field + " has grade " +
                                                                 dec(static_cast<uint32_t>(ref.grade)) +
                                                                 ", which is not one of the six"});
    }
    if (ref.source.empty()) {
        if (ref.grade != Grade::assumed) {
            return std::unexpected(Error{Code::invalid_argument, field + " is graded " + std::string(grade) +
                                                                     " but cites no source; only an assumed value "
                                                                     "may cite none"});
        }
    } else {
        const Source* source = registry.find(ref.source);
        if (source == nullptr) {
            return std::unexpected(Error{Code::invalid_argument, field + " cites source '" + ref.source +
                                                                     "', which the sources registry does not list"});
        }
        if (source->license == License::not_usable) {
            return std::unexpected(Error{Code::invalid_argument,
                                         field + " cites source '" + ref.source +
                                             "', whose license is not_usable: it feeds no number in the library (" +
                                             source->license_evidence + ")"});
        }
    }
    if ((ref.grade == Grade::assumed || ref.grade == Grade::derived) && ref.note.empty()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     field + " is graded " + std::string(grade) + " and needs a note saying " +
                                         (ref.grade == Grade::assumed ? "why" : "which code derived it")});
    }
    return {};
}

}  // namespace spade::scene
