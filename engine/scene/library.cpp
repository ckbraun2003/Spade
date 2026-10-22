#include "scene/library.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "scene/detail/relative_path.hpp"
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

// The top level's name in a diagnostic, and what an entry's paths are
// relative to (relative_path_problem's "must be relative to the <kind> file").
constexpr std::string_view kTop = "library index";

// An entry's kind, by its file's extension.
constexpr std::string_view kModelSuffix = ".model.yaml";
constexpr std::string_view kObjectSuffix = ".object.yaml";

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

// The part kind a file's suffix names (scene/part_file.hpp), if any.
[[nodiscard]] std::optional<PartKind> part_kind_of(std::string_view file) {
    for (const PartKind kind : {PartKind::motor, PartKind::prop, PartKind::esc, PartKind::battery}) {
        if (file.ends_with(part_suffix(kind))) return kind;
    }
    return std::nullopt;
}

[[nodiscard]] bool is_part(LibraryKind kind) { return kind != LibraryKind::model && kind != LibraryKind::object; }

// What an entry is, for a loader's refusal: "a model", "an object", "a motor
// part".
[[nodiscard]] std::string kind_phrase(LibraryKind kind) {
    if (kind == LibraryKind::model) return "a model";
    if (kind == LibraryKind::object) return "an object";
    const auto part = static_cast<PartKind>(static_cast<uint32_t>(kind) - static_cast<uint32_t>(LibraryKind::motor));
    return "a " + std::string(part_kind_name(part)) + " part";
}

// The loader for an entry of `kind`.
[[nodiscard]] std::string_view loader_of(LibraryKind kind) {
    if (kind == LibraryKind::model) return "load_library_model";
    if (kind == LibraryKind::object) return "load_library_object";
    return "load_library_part";
}

// `text` with ASCII A-Z lowered: what "ignoring case" compares. A path
// differing in a non-ASCII letter's case is not caught; the scene's path rule
// and the mesh_ref rule keep the library's own names ASCII in practice.
[[nodiscard]] std::string folded(std::string_view text) {
    std::string out(text);
    for (char& c : out) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return out;
}

// What is wrong with `path` as an entry's file or thumbnail, or empty: the
// scene's relative-path rule (TD-9), then no '..' segment, so an entry stays
// inside the library's folder. `path` is not empty.
[[nodiscard]] std::string entry_path_problem(std::string_view path) {
    if (std::string problem = detail::relative_path_problem(path, kTop); !problem.empty()) {
        return problem;
    }
    std::string_view rest = path;
    while (true) {
        const std::size_t slash = rest.find('/');
        if (rest.substr(0, slash) == "..") {
            return "must not have a '..' segment: an entry stays inside the library's folder";
        }
        if (slash == std::string_view::npos) return {};
        rest.remove_prefix(slash + 1);
    }
}

// The non-empty string at `node[key]` (required), refused by name when empty.
[[nodiscard]] Result<std::string> non_empty_field(const YAML::Node& node, const char* key, const std::string& what,
                                                  std::string_view empty_hint = {}) {
    const Result<YAML::Node> value = field(node, key, what);
    if (!value) return std::unexpected(value.error());
    const std::string where = what + "." + key;
    Result<std::string> text = scalar(*value, where);
    if (!text) return text;
    if (text->empty()) {
        return std::unexpected(at(*value, where + " must not be empty" + std::string(empty_hint)));
    }
    return text;
}

// A path key of an entry (`file`, `thumbnail`): non-empty and by
// entry_path_problem()'s rules.
[[nodiscard]] Result<std::string> path_field(const YAML::Node& node, const char* key, const std::string& what,
                                             std::string_view empty_hint = {}) {
    Result<std::string> path = non_empty_field(node, key, what, empty_hint);
    if (!path) return path;
    if (const std::string problem = entry_path_problem(*path); !problem.empty()) {
        return std::unexpected(at(node[key], what + "." + key + " '" + *path + "' " + problem));
    }
    return path;
}

[[nodiscard]] Result<Library> parse_index(const YAML::Node& root) {
    if (!root.IsMap()) {
        return std::unexpected(at(root, "a library index's top level must be a mapping with a 'library_version' key"));
    }

    // VERSION FIRST, before the unknown-key sweep (the scene's rule): a later
    // version's keys would otherwise be reported as unknown.
    const Result<YAML::Node> version_node = field(root, "library_version", kTop);
    if (!version_node) return std::unexpected(version_node.error());
    const Result<uint32_t> version = parse_uint<uint32_t>(*version_node, "library_version");
    if (!version) return std::unexpected(version.error());
    if (*version < 1 || *version > kLibraryVersion) {
        return std::unexpected(Error{Code::schema_mismatch, "library index is schema version " + dec(*version) +
                                                                ", this build reads versions 1 to " +
                                                                dec(kLibraryVersion) + mark_of(*version_node)});
    }
    // Every diagnostic blames the index's own version's schema.
    const std::string schema = "library schema v" + dec(*version);
    if (Result<void> r = yaml_text::check_map(root, kTop, {"library_version", "entries"}, schema); !r) {
        return std::unexpected(r.error());
    }

    const Result<YAML::Node> entries = field(root, "entries", kTop);
    if (!entries) return std::unexpected(entries.error());
    if (!entries->IsSequence()) {
        return std::unexpected(at(*entries, "entries must be a sequence"));
    }

    Library out{};
    out.entries.reserve(entries->size());
    for (std::size_t i = 0; i < entries->size(); ++i) {
        const YAML::Node node = (*entries)[i];
        const std::string what = "entries[" + dec(i) + "]";
        if (Result<void> r = yaml_text::check_map(node, what, {"name", "category", "file", "thumbnail"}, schema);
            !r) {
            return std::unexpected(r.error());
        }
        LibraryEntry entry{};

        Result<std::string> name = non_empty_field(node, "name", what);
        if (!name) return std::unexpected(name.error());
        for (std::size_t j = 0; j < out.entries.size(); ++j) {
            if (folded(out.entries[j].name) == folded(*name)) {
                return std::unexpected(at(node["name"], what + ".name '" + *name + "' is entries[" + dec(j) +
                                                            "]'s name '" + out.entries[j].name +
                                                            "' ignoring case; names are unique ignoring case"));
            }
        }
        entry.name = std::move(*name);

        Result<std::string> category = non_empty_field(node, "category", what);
        if (!category) return std::unexpected(category.error());
        entry.category = std::move(*category);

        Result<std::string> file = path_field(node, "file", what);
        if (!file) return std::unexpected(file.error());
        const std::optional<PartKind> part = part_kind_of(*file);
        if (file->ends_with(kModelSuffix)) {
            entry.kind = LibraryKind::model;
        } else if (file->ends_with(kObjectSuffix)) {
            entry.kind = LibraryKind::object;
        } else if (part && *version >= 2) {
            entry.kind = static_cast<LibraryKind>(static_cast<uint32_t>(LibraryKind::motor) +
                                                  static_cast<uint32_t>(*part));
        } else if (part) {
            return std::unexpected(at(node["file"], what + ".file '" + *file +
                                                        "' is a part file, which needs library_version 2"));
        } else if (*version >= 2) {
            return std::unexpected(at(node["file"], what + ".file '" + *file + "' must end in " +
                                                        std::string(kModelSuffix) + " (a model file), " +
                                                        std::string(kObjectSuffix) +
                                                        " (an object file), or .motor.yaml, .prop.yaml, "
                                                        ".esc.yaml or .battery.yaml (a part file)"));
        } else {
            return std::unexpected(at(node["file"], what + ".file '" + *file + "' must end in " +
                                                        std::string(kModelSuffix) + " (a model file) or " +
                                                        std::string(kObjectSuffix) + " (an object file)"));
        }
        for (std::size_t j = 0; j < out.entries.size(); ++j) {
            if (folded(out.entries[j].file) == folded(*file)) {
                return std::unexpected(at(node["file"], what + ".file '" + *file + "' is entries[" + dec(j) +
                                                            "]'s file '" + out.entries[j].file +
                                                            "' ignoring case, and on Windows they are one file"));
            }
        }
        entry.file = std::move(*file);

        // thumbnail: optional; absent is none.
        if (has(node, "thumbnail")) {
            Result<std::string> thumbnail =
                path_field(node, "thumbnail", what, ": leave the key out for an entry with none");
            if (!thumbnail) return std::unexpected(thumbnail.error());
            entry.thumbnail = std::move(*thumbnail);
        }
        out.entries.push_back(std::move(entry));
    }
    return out;
}

// yaml-cpp reports every syntax failure by throwing; nothing may leave a Spade
// API that way (the scene file's rule).
[[nodiscard]] Result<Library> parse_text(std::string_view text) {
    try {
        const YAML::Node root = YAML::Load(std::string(text));
        return parse_index(root);
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, std::string("library index parse failed: ") + e.what()});
    }
}

// library.dir / `relative`, the text read as UTF-8 (path(std::string) would
// read it in the ANSI code page on Windows). io_error naming the entry when
// this platform cannot form the path.
[[nodiscard]] Result<std::filesystem::path> entry_file(const Library& library, const LibraryEntry& entry,
                                                       const std::string& relative) {
    try {
        std::u8string utf8;
        utf8.reserve(relative.size());
        for (const char c : relative) utf8.push_back(static_cast<char8_t>(c));
        return library.dir / std::filesystem::path(utf8);
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::io_error, "library entry '" + entry.name + "': '" + relative +
                                                         "' is not a path this platform can open: " + e.what()});
    }
}

// A loader's refusal, after the entry's name.
[[nodiscard]] Error for_entry(const LibraryEntry& entry, const Error& e) {
    return Error{e.code, "library entry '" + entry.name + "': " + e.context};
}

}  // namespace

// ===========================================================================
// Public API
// ===========================================================================

Result<Library> library_from_yaml(std::string_view text) {
    return parse_text(text);
}

Result<std::string> library_to_yaml(const Library& library) {
    (void)library;
    return std::unexpected(Error{Code::internal, "library_to_yaml: not written yet"});
}

Result<Library> load_library(const std::filesystem::path& path) {
    std::string text;
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return std::unexpected(Error{Code::io_error, "cannot open library index '" + path_text(path) + "'"});
        }
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) {
            return std::unexpected(Error{Code::io_error, "failed reading library index '" + path_text(path) + "'"});
        }
    } catch (const std::exception& e) {
        return std::unexpected(
            Error{Code::io_error, "failed reading library index '" + path_text(path) + "': " + e.what()});
    }

    Result<Library> library = library_from_yaml(text);
    if (!library) {
        return std::unexpected(Error{library.error().code, path_text(path) + ": " + library.error().context});
    }
    // The entries are relative to the index's directory, "." for a bare name.
    library->dir = path.has_parent_path() ? path.parent_path() : std::filesystem::path(".");

    // Each named file on disk: the entry's, and a thumbnail it names.
    const auto exists = [&](const LibraryEntry& entry, const std::string& relative,
                            std::string_view key) -> Result<void> {
        const Result<std::filesystem::path> file = entry_file(*library, entry, relative);
        if (!file) return std::unexpected(Error{file.error().code, path_text(path) + ": " + file.error().context});
        std::error_code ec;
        if (!std::filesystem::is_regular_file(*file, ec)) {
            return std::unexpected(Error{Code::io_error, path_text(path) + ": library entry '" + entry.name + "': " +
                                                             std::string(key) + " '" + relative + "' is missing (" +
                                                             path_text(*file) + ")"});
        }
        return {};
    };
    for (const LibraryEntry& entry : library->entries) {
        if (Result<void> r = exists(entry, entry.file, "file"); !r) return std::unexpected(r.error());
        if (!entry.thumbnail.empty()) {
            if (Result<void> r = exists(entry, entry.thumbnail, "thumbnail"); !r) return std::unexpected(r.error());
        }
    }
    return library;
}

Result<ModelFile> load_library_model(const Library& library, const LibraryEntry& entry) {
    if (entry.kind != LibraryKind::model) {
        return std::unexpected(invalid("library entry '" + entry.name + "' is " + kind_phrase(entry.kind) +
                                       ", not a model: load it with " + std::string(loader_of(entry.kind))));
    }
    const Result<std::filesystem::path> file = entry_file(library, entry, entry.file);
    if (!file) return std::unexpected(file.error());
    Result<ModelFile> model = load_model_file(*file);
    if (!model) return std::unexpected(for_entry(entry, model.error()));
    return model;
}

Result<ObjectFile> load_library_object(const Library& library, const LibraryEntry& entry) {
    if (entry.kind != LibraryKind::object) {
        return std::unexpected(invalid("library entry '" + entry.name + "' is " + kind_phrase(entry.kind) +
                                       ", not an object: load it with " + std::string(loader_of(entry.kind))));
    }
    const Result<std::filesystem::path> file = entry_file(library, entry, entry.file);
    if (!file) return std::unexpected(file.error());
    Result<ObjectFile> object = load_object_file(*file);
    if (!object) return std::unexpected(for_entry(entry, object.error()));
    return object;
}

Result<SourceRegistry> load_library_sources(const Library& library) {
    return load_sources(library.dir / "sources.yaml");
}

Result<PartFile> load_library_part(const Library& library, const LibraryEntry& entry,
                                   const SourceRegistry& registry) {
    if (!is_part(entry.kind)) {
        return std::unexpected(invalid("library entry '" + entry.name + "' is " + kind_phrase(entry.kind) +
                                       ", not a part: load it with " + std::string(loader_of(entry.kind))));
    }
    const Result<std::filesystem::path> file = entry_file(library, entry, entry.file);
    if (!file) return std::unexpected(file.error());
    Result<PartFile> part = load_part_file(*file, registry);
    if (!part) return std::unexpected(for_entry(entry, part.error()));
    return part;
}

Result<void> check_library_contents(const Library& library) {
    // The registry, read once, when any entry is a part.
    std::optional<SourceRegistry> registry;
    for (const LibraryEntry& entry : library.entries) {
        if (!is_part(entry.kind)) continue;
        Result<SourceRegistry> r = load_library_sources(library);
        if (!r) return std::unexpected(r.error());
        registry = std::move(*r);
        break;
    }
    // Each object's visual mesh_ref, folded, with the entry that holds it.
    struct Ref {
        std::string ref;
        std::string key;
        const LibraryEntry* entry = nullptr;
    };
    std::vector<Ref> refs;
    for (const LibraryEntry& entry : library.entries) {
        if (entry.kind == LibraryKind::model) {
            if (Result<ModelFile> m = load_library_model(library, entry); !m) return std::unexpected(m.error());
            continue;
        }
        if (is_part(entry.kind)) {
            if (Result<PartFile> p = load_library_part(library, entry, *registry); !p) {
                return std::unexpected(p.error());
            }
            continue;
        }
        const Result<ObjectFile> object = load_library_object(library, entry);
        if (!object) return std::unexpected(object.error());
        const std::string& ref = object->object.visual.mesh_ref;
        if (ref.empty()) continue;
        std::string key = folded(ref);
        for (const Ref& seen : refs) {
            if (seen.key == key && seen.ref != ref) {
                return std::unexpected(invalid("library entries '" + seen.entry->name + "' and '" + entry.name +
                                               "': visual mesh_refs '" + seen.ref + "' and '" + ref +
                                               "' differ only in case, and on Windows they name one file"));
            }
        }
        refs.push_back(Ref{ref, std::move(key), &entry});
    }
    return {};
}

}  // namespace spade::scene
