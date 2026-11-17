// ---------------------------------------------------------------------------
// scene/library_set.cpp -- see library_set.hpp. Reads the indexes of N roots,
// checks every rule of the contract's section 3, and refuses by name.
// ---------------------------------------------------------------------------
#include "scene/library_set.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <set>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "scene/detail/relative_path.hpp"
#include "scene/library.hpp"

namespace spade::scene {
namespace {

namespace fs = std::filesystem;

LibraryRefusal make(std::string code, std::string message, std::string root = {}, std::string path = {},
                    std::string id = {}, std::string field = {}) {
    return LibraryRefusal{std::move(code), std::move(message), std::move(root), std::move(path),
                          std::move(id),   std::move(field),   {}};
}

bool is_lower_digit_us(char c) {
    return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
}

bool valid_root_name(std::string_view s) {
    if (s.empty() || !(s[0] >= 'a' && s[0] <= 'z')) {
        return false;
    }
    return std::all_of(s.begin(), s.end(), is_lower_digit_us);
}

// Dotted lower-case segments, at least two: "drone.x500", "part.motor.fpv_2306".
bool valid_local_id(std::string_view s) {
    std::size_t segments = 0;
    std::size_t start = 0;
    while (start <= s.size()) {
        const std::size_t dot = s.find('.', start);
        const std::string_view seg = s.substr(start, dot == std::string_view::npos ? s.size() - start : dot - start);
        if (seg.empty() || !std::all_of(seg.begin(), seg.end(), is_lower_digit_us)) {
            return false;
        }
        ++segments;
        if (dot == std::string_view::npos) {
            break;
        }
        start = dot + 1;
    }
    return segments >= 2;
}

bool valid_tag(std::string_view s) {
    return !s.empty() &&
           std::all_of(s.begin(), s.end(), [](char c) { return is_lower_digit_us(c) || c == '-'; });
}

std::string lower(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return s;
}

// "<root>:<local>" -> {root, local}; false when there is no ':' (a bare id).
bool split_id(std::string_view full, std::string& root, std::string& local) {
    const std::size_t colon = full.find(':');
    if (colon == std::string_view::npos) {
        return false;
    }
    root = std::string(full.substr(0, colon));
    local = std::string(full.substr(colon + 1));
    return true;
}

bool scalar(const YAML::Node& n, std::string& out) {
    if (!n.IsScalar()) {
        return false;
    }
    out = n.Scalar();
    return true;
}

// A relative path the library may name: the scene's rule, no '..' and no empty segment.
std::string path_problem(std::string_view file, std::string_view what) {
    if (file.empty()) {
        return std::string(what) + " must not be empty";
    }
    if (std::string p = detail::relative_path_problem(file, "library index"); !p.empty()) {
        return p;
    }
    std::size_t start = 0;
    while (start <= file.size()) {
        const std::size_t slash = file.find('/', start);
        const std::string_view seg = file.substr(start, slash == std::string_view::npos ? file.size() - start : slash - start);
        if (seg.empty() || seg == "..") {
            return std::string(what) + " must not have an empty or '..' segment";
        }
        if (slash == std::string_view::npos) {
            break;
        }
        start = slash + 1;
    }
    return {};
}

bool ends_with(std::string_view s, std::string_view suffix) {
    return s.size() >= suffix.size() && s.substr(s.size() - suffix.size()) == suffix;
}

// ----- canonical JSON of a YAML node: the parts_map the entry hash covers -------------------------------------------
void json_string(std::string& out, const std::string& s) {
    static const char* hex = "0123456789abcdef";
    out += '"';
    for (const char c : s) {
        switch (c) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out += "\\u00";
                    out += hex[(static_cast<unsigned char>(c) >> 4) & 0xF];
                    out += hex[static_cast<unsigned char>(c) & 0xF];
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

// Object keys sorted bytewise, no whitespace; a quoted scalar is a string; a plain one is null, a bool, an integer,
// a finite number (the shortest decimal that reads back) or a string.
void canonical_json(const YAML::Node& n, std::string& out) {
    switch (n.Type()) {
        case YAML::NodeType::Null:
        case YAML::NodeType::Undefined:
            out += "null";
            return;
        case YAML::NodeType::Scalar: {
            const std::string& s = n.Scalar();
            if (n.Tag() != "!") {
                if (s == "true" || s == "false") {
                    out += s;
                    return;
                }
                const char* first = s.data();
                const char* last = s.data() + s.size();
                std::int64_t i = 0;
                if (const auto r = std::from_chars(first, last, i); r.ec == std::errc() && r.ptr == last) {
                    out += std::to_string(i);
                    return;
                }
                double d = 0.0;
                if (const auto r = std::from_chars(first, last, d);
                    r.ec == std::errc() && r.ptr == last && std::isfinite(d)) {
                    char buf[40];
                    const auto w = std::to_chars(buf, buf + sizeof(buf), d);
                    out.append(buf, w.ptr);
                    return;
                }
            }
            json_string(out, s);
            return;
        }
        case YAML::NodeType::Sequence: {
            out += '[';
            bool first = true;
            for (const YAML::Node& item : n) {
                out += first ? "" : ",";
                first = false;
                canonical_json(item, out);
            }
            out += ']';
            return;
        }
        case YAML::NodeType::Map: {
            // Sort the keys, then look each up: a YAML::Node assigns by value, so nodes must not be sorted themselves.
            std::vector<std::string> keys;
            for (const auto& kv : n) {
                keys.push_back(kv.first.IsScalar() ? kv.first.Scalar() : std::string());
            }
            std::sort(keys.begin(), keys.end());
            out += '{';
            bool first = true;
            for (const std::string& key : keys) {
                out += first ? "" : ",";
                first = false;
                json_string(out, key);
                out += ':';
                canonical_json(n[key], out);
            }
            out += '}';
            return;
        }
    }
}

struct Alias {
    std::string key;
    std::string target;
    std::string path;  // the index file the alias is written in
};

struct RootData {
    LibraryRootSpec spec{};
    SetRoot info{};
    std::vector<SetEntry> entries{};
    std::vector<Alias> aliases{};
    // The full ids that entries and aliases name, for the existence pass.
    struct Ref {
        std::string target;
        std::string path;
        std::string field;
        std::string from_id;  // empty for an alias
    };
    std::vector<Ref> refs{};
    bool read = false;  // the root's own index was read (its dependencies can be checked)
};

using Issues = std::vector<LibraryRefusal>;

constexpr std::array<std::string_view, 4> kPartKinds = {"motor", "prop", "esc", "battery"};

bool is_part_kind(std::string_view s) {
    return std::find(kPartKinds.begin(), kPartKinds.end(), s) != kPartKinds.end();
}

// ----- version 1 and 2: the existing reader, with derived ids -------------------------------------------------------
void read_old(RootData& r, Issues& issues, const fs::path& index) {
    const Result<Library> lib = load_library(index);
    if (!lib) {
        issues.push_back(make("bad_index", lib.error().context, r.spec.name, index.string()));
        return;
    }
    std::set<std::string> seen;
    std::size_t i = 0;
    for (const LibraryEntry& e : lib->entries) {
        SetEntry s;
        s.root = r.spec.name;
        s.name = e.name;
        s.category = e.category;
        s.rel_file = e.file;
        s.file = lib->dir / e.file;
        s.rel_thumbnail = e.thumbnail;
        if (!e.thumbnail.empty()) {
            s.thumbnail = lib->dir / e.thumbnail;
        }
        std::string suffix;
        std::string prefix;
        switch (e.kind) {
            case LibraryKind::model:
                prefix = "model";
                suffix = ".model.yaml";
                s.kind = "model";
                break;
            case LibraryKind::object:
                prefix = "object";
                suffix = ".object.yaml";
                s.kind = "object";
                break;
            case LibraryKind::motor:
                prefix = "part.motor";
                suffix = ".motor.yaml";
                s.kind = "part";
                s.part_kind = "motor";
                break;
            case LibraryKind::prop:
                prefix = "part.prop";
                suffix = ".prop.yaml";
                s.kind = "part";
                s.part_kind = "prop";
                break;
            case LibraryKind::esc:
                prefix = "part.esc";
                suffix = ".esc.yaml";
                s.kind = "part";
                s.part_kind = "esc";
                break;
            case LibraryKind::battery:
                prefix = "part.battery";
                suffix = ".battery.yaml";
                s.kind = "part";
                s.part_kind = "battery";
                break;
        }
        std::string stem = fs::path(e.file).filename().string();
        if (ends_with(stem, suffix)) {
            stem.resize(stem.size() - suffix.size());
        }
        stem = lower(stem);
        for (char& c : stem) {
            if (!is_lower_digit_us(c)) {
                c = '_';
            }
        }
        s.local_id = prefix + "." + stem;
        s.id = r.spec.name + ":" + s.local_id;
        if (!seen.insert(s.local_id).second) {
            issues.push_back(make("bad_index", "the derived id '" + s.local_id + "' is already taken by an earlier entry",
                                  r.spec.name, index.string(), s.id, "entries[" + std::to_string(i) + "].file"));
        }
        r.entries.push_back(std::move(s));
        ++i;
    }
    r.info.entry_count = r.entries.size();
}

// ----- version 3 ------------------------------------------------------------------------------------------------------
struct EntryReader {
    RootData& root;
    Issues& issues;
    const fs::path& index_path;
    std::string field;  // "entries[i]"
    std::string local_id_for_messages;

    void bad(const std::string& key, const std::string& message) {
        issues.push_back(make("bad_index", message, root.spec.name, index_path.string(),
                              local_id_for_messages.empty() ? std::string() : root.spec.name + ":" + local_id_for_messages,
                              field + (key.empty() ? "" : "." + key)));
    }
};

void check_rel(EntryReader& er, const std::string& key, const std::string& value, fs::path& joined, bool must_exist) {
    if (std::string p = path_problem(value, "'" + key + "'"); !p.empty()) {
        er.bad(key, p);
        return;
    }
    joined = er.root.spec.path / value;
    std::error_code ec;
    if (must_exist && !fs::is_regular_file(joined, ec)) {
        er.bad(key, "'" + value + "' does not exist");
    }
}

void read_entry(RootData& r, Issues& issues, const fs::path& index_path, const YAML::Node& node, std::size_t i,
                std::set<std::string>& ids, std::set<std::string>& names_by_kind) {
    EntryReader er{r, issues, index_path, "entries[" + std::to_string(i) + "]", {}};
    if (!node.IsMap()) {
        er.bad("", "an entry must be a mapping");
        return;
    }
    static const std::set<std::string> known = {"id",        "name",   "category",       "file",
                                                "type",      "tags",   "thumbnail",      "requires",
                                                "files",     "generated_from", "parts_map"};
    for (const auto& kv : node) {
        std::string k;
        if (!scalar(kv.first, k) || known.count(k) == 0) {
            er.bad(kv.first.IsScalar() ? kv.first.Scalar() : "?", "unknown key");
        }
    }
    auto get = [&](const char* key) -> YAML::Node {
        const YAML::Node n = node[key];
        return n.IsDefined() && !n.IsNull() ? n : YAML::Node(YAML::NodeType::Undefined);
    };
    SetEntry s;
    s.root = r.spec.name;

    // The required keys, in order.
    std::string id;
    const YAML::Node idn = get("id");
    if (!idn) {
        er.bad("id", "missing key");
    } else if (!scalar(idn, id) || !valid_local_id(id)) {
        er.bad("id", "an id is two or more dotted lower-case segments of [a-z0-9_], like 'drone.x500'");
        id.clear();
    }
    er.local_id_for_messages = id;
    if (!id.empty() && !ids.insert(id).second) {
        er.bad("id", "the id is used by an earlier entry");
    }
    s.local_id = id;
    s.id = id.empty() ? std::string() : r.spec.name + ":" + id;

    const YAML::Node namen = get("name");
    if (!namen) {
        er.bad("name", "missing key");
    } else if (!scalar(namen, s.name) || s.name.empty()) {
        er.bad("name", "a name is a non-empty string");
        s.name.clear();
    }
    const YAML::Node catn = get("category");
    if (!catn) {
        er.bad("category", "missing key");
    } else if (!scalar(catn, s.category)) {
        er.bad("category", "a category is a string");
    }

    // Kind: the id's first segment, or opaque when `type` is given.
    std::string first;
    std::string second;
    if (!id.empty()) {
        const std::size_t d1 = id.find('.');
        first = id.substr(0, d1);
        const std::size_t d2 = id.find('.', d1 + 1);
        second = id.substr(d1 + 1, d2 == std::string::npos ? std::string::npos : d2 - d1 - 1);
    }
    static const std::set<std::string> spade_kinds = {"model", "object", "part", "drone", "world", "mesh"};
    const YAML::Node typen = get("type");
    if (typen) {
        if (!scalar(typen, s.type) || s.type.empty()) {
            er.bad("type", "a type is a non-empty string");
            s.type.clear();
        } else {
            s.kind = "opaque";
            if (!first.empty() && spade_kinds.count(first) != 0) {
                er.bad("id", "an opaque entry's id must not start with the Spade kind name '" + first + "'");
            }
        }
    } else if (!first.empty()) {
        if (spade_kinds.count(first) == 0) {
            er.bad("id", "the id's first segment is the kind: model, object, part, drone, world or mesh (or give a 'type')");
        } else {
            s.kind = first;
            if (first == "part") {
                if (!is_part_kind(second)) {
                    er.bad("id", "a part id is 'part.<motor|prop|esc|battery>.<name>'");
                } else {
                    s.part_kind = second;
                }
            }
        }
    }
    if (!s.name.empty()) {
        const std::string key = (s.kind.empty() ? "?" : s.kind) + "\n" + lower(s.name);
        if (!names_by_kind.insert(key).second) {
            er.bad("name", "another entry of the same kind has this name (ignoring case)");
        }
    }

    // The file.
    const YAML::Node filen = get("file");
    const YAML::Node partsn = get("parts_map");
    if (partsn) {
        if (!partsn.IsSequence()) {
            er.bad("parts_map", "a parts_map is a sequence");
        } else if (s.kind != "drone") {
            er.bad("parts_map", "only a drone entry has a parts_map");
        } else {
            s.has_parts_map = true;
            canonical_json(partsn, s.parts_map_json);
        }
    }
    if (!filen) {
        if (!(s.kind == "drone" && s.has_parts_map)) {
            er.bad("file", s.kind == "drone" ? "a drone needs a 'file' or a 'parts_map'" : "missing key");
        }
    } else if (!scalar(filen, s.rel_file)) {
        er.bad("file", "a file is a string");
        s.rel_file.clear();
    } else {
        check_rel(er, "file", s.rel_file, s.file, true);
        if (!s.file.empty()) {
            const std::string name = s.rel_file;
            bool ok = true;
            if (s.kind == "model" || s.kind == "drone") {
                ok = ends_with(name, ".model.yaml");
            } else if (s.kind == "object") {
                ok = ends_with(name, ".object.yaml");
            } else if (s.kind == "part" && !s.part_kind.empty()) {
                ok = ends_with(name, "." + s.part_kind + ".yaml");
            } else if (s.kind == "world") {
                ok = ends_with(name, ".world.yaml");
            } else if (s.kind == "mesh") {
                ok = ends_with(name, ".gltf") || ends_with(name, ".glb") || ends_with(name, ".spmesh");
            }
            if (!ok) {
                er.bad("file", "the file's extension does not match the kind '" + s.kind + "'");
            }
        }
    }

    // Optional lists and values.
    if (const YAML::Node n = get("tags")) {
        if (!n.IsSequence()) {
            er.bad("tags", "tags is a sequence");
        } else {
            for (std::size_t j = 0; j < n.size(); ++j) {
                std::string t;
                if (!scalar(n[j], t) || !valid_tag(t)) {
                    er.bad("tags[" + std::to_string(j) + "]", "a tag is lower-case [a-z0-9_-]");
                } else {
                    s.tags.push_back(t);
                }
            }
        }
    }
    if (const YAML::Node n = get("thumbnail")) {
        if (!scalar(n, s.rel_thumbnail)) {
            er.bad("thumbnail", "a thumbnail is a path string");
            s.rel_thumbnail.clear();
        } else {
            check_rel(er, "thumbnail", s.rel_thumbnail, s.thumbnail, true);
        }
    }
    if (const YAML::Node n = get("files")) {
        if (!n.IsSequence()) {
            er.bad("files", "files is a sequence");
        } else {
            for (std::size_t j = 0; j < n.size(); ++j) {
                std::string f;
                const std::string key = "files[" + std::to_string(j) + "]";
                if (!scalar(n[j], f)) {
                    er.bad(key, "a companion file is a path string");
                } else {
                    fs::path joined;
                    check_rel(er, key, f, joined, true);
                    s.files.push_back(f);
                }
            }
        }
    }
    if (const YAML::Node n = get("requires")) {
        if (!n.IsSequence()) {
            er.bad("requires", "requires is a sequence");
        } else {
            for (std::size_t j = 0; j < n.size(); ++j) {
                std::string t;
                const std::string key = "requires[" + std::to_string(j) + "]";
                if (!scalar(n[j], t)) {
                    er.bad(key, "a required id is a string");
                    continue;
                }
                s.requires_ids.push_back(t);
                r.refs.push_back({t, index_path.string(), er.field + "." + key, s.id});
            }
        }
    }
    if (const YAML::Node n = get("generated_from")) {
        if (!n.IsMap() || n.size() != 2 || !n["recipe"] || !n["hash"] || !scalar(n["recipe"], s.generated_recipe) ||
            !scalar(n["hash"], s.generated_hash)) {
            er.bad("generated_from", "generated_from is a mapping of exactly 'recipe' and 'hash' strings");
            s.generated_recipe.clear();
            s.generated_hash.clear();
        }
    }
    r.entries.push_back(std::move(s));
}

void read_v3(RootData& r, Issues& issues, const fs::path& index, const YAML::Node& top) {
    auto bad = [&](const std::string& field, const std::string& message) {
        issues.push_back(make("bad_index", message, r.spec.name, index.string(), {}, field));
    };
    static const std::set<std::string> known = {"library_version", "name", "depends", "aliases", "indexes"};
    for (const auto& kv : top) {
        std::string k;
        if (!scalar(kv.first, k) || known.count(k) == 0) {
            bad(kv.first.IsScalar() ? kv.first.Scalar() : "?", "unknown key");
        }
    }
    std::string name;
    if (!top["name"] || !scalar(top["name"], name)) {
        bad("name", "a version 3 root names itself with a 'name' key");
    } else if (name != r.spec.name) {
        issues.push_back(make("root_name_mismatch",
                              "the index names itself '" + name + "' but the host opened it as '" + r.spec.name + "'",
                              r.spec.name, r.spec.path.string(), {}, "name"));
        return;
    }
    if (const YAML::Node d = top["depends"]; d && !d.IsNull()) {
        if (!d.IsSequence()) {
            bad("depends", "depends is a sequence of root names");
        } else {
            for (std::size_t j = 0; j < d.size(); ++j) {
                std::string dep;
                if (!scalar(d[j], dep) || !valid_root_name(dep)) {
                    bad("depends[" + std::to_string(j) + "]", "a dependency is a root name");
                } else {
                    r.info.depends.push_back(dep);
                }
            }
        }
    }
    if (const YAML::Node a = top["aliases"]; a && !a.IsNull()) {
        if (!a.IsMap()) {
            bad("aliases", "aliases is a mapping of old string to full id");
        } else {
            for (const auto& kv : a) {
                std::string key;
                std::string target;
                if (!scalar(kv.first, key) || key.empty() || !scalar(kv.second, target)) {
                    bad("aliases", "an alias is a non-empty string mapped to a full id string");
                    continue;
                }
                r.aliases.push_back({key, target, index.string()});
                r.refs.push_back({target, index.string(), "aliases." + key, {}});
            }
        }
    }
    const YAML::Node idx = top["indexes"];
    if (!idx || !idx.IsSequence() || idx.size() == 0) {
        bad("indexes", "a version 3 root lists its per-kind index files under 'indexes'");
        return;
    }
    std::set<std::string> ids;
    std::set<std::string> names_by_kind;
    for (std::size_t j = 0; j < idx.size(); ++j) {
        std::string rel;
        if (!scalar(idx[j], rel)) {
            bad("indexes[" + std::to_string(j) + "]", "an index is a path string");
            continue;
        }
        if (std::string p = path_problem(rel, "an index path"); !p.empty()) {
            bad("indexes[" + std::to_string(j) + "]", p);
            continue;
        }
        const fs::path ipath = r.spec.path / rel;
        std::error_code ec;
        if (!fs::is_regular_file(ipath, ec)) {
            bad("indexes[" + std::to_string(j) + "]", "'" + rel + "' does not exist");
            continue;
        }
        YAML::Node doc;
        try {
            doc = YAML::LoadFile(ipath.string());
        } catch (const std::exception& e) {
            issues.push_back(make("bad_index", e.what(), r.spec.name, ipath.string()));
            continue;
        }
        if (!doc.IsMap()) {
            issues.push_back(make("bad_index", "an index file is a mapping with 'index_version' and 'entries'", r.spec.name,
                                  ipath.string()));
            continue;
        }
        for (const auto& kv : doc) {
            std::string k;
            if (!scalar(kv.first, k) || (k != "index_version" && k != "entries")) {
                issues.push_back(make("bad_index", "unknown key", r.spec.name, ipath.string(), {},
                                      kv.first.IsScalar() ? kv.first.Scalar() : "?"));
            }
        }
        std::string v;
        if (!doc["index_version"] || !scalar(doc["index_version"], v) || v != "1") {
            issues.push_back(make("bad_index", "index_version must be 1", r.spec.name, ipath.string(), {}, "index_version"));
        }
        const YAML::Node entries = doc["entries"];
        if (!entries || !entries.IsSequence()) {
            issues.push_back(make("bad_index", "'entries' is a sequence", r.spec.name, ipath.string(), {}, "entries"));
            continue;
        }
        for (std::size_t i = 0; i < entries.size(); ++i) {
            read_entry(r, issues, ipath, entries[i], i, ids, names_by_kind);
        }
    }
    r.info.entry_count = r.entries.size();
}

void read_root(RootData& r, Issues& issues) {
    const fs::path index = r.spec.path / "library.yaml";
    std::error_code ec;
    if (!fs::is_directory(r.spec.path, ec) || !fs::is_regular_file(index, ec)) {
        issues.push_back(make("root_unavailable",
                              "root '" + r.spec.name + "': the folder or its library.yaml is absent or unreadable",
                              r.spec.name, r.spec.path.string()));
        return;
    }
    YAML::Node top;
    try {
        top = YAML::LoadFile(index.string());
    } catch (const std::exception& e) {
        issues.push_back(make("bad_index", e.what(), r.spec.name, index.string()));
        return;
    }
    std::string v;
    if (!top.IsMap() || !top["library_version"] || !scalar(top["library_version"], v)) {
        issues.push_back(make("bad_index", "the top level must be a mapping with a 'library_version' key", r.spec.name,
                              index.string()));
        return;
    }
    r.info.name = r.spec.name;
    r.info.path = r.spec.path;
    r.info.writable = r.spec.writable;
    if (v == "1" || v == "2") {
        r.info.library_version = v == "1" ? 1u : 2u;
        r.read = true;
        read_old(r, issues, index);
    } else if (v == "3") {
        r.info.library_version = 3;
        read_v3(r, issues, index, top);
        r.read = true;
    } else {
        issues.push_back(make("bad_index",
                              "library_version " + v + " is not read by this build (it reads 1 to " +
                                  std::to_string(kLibrarySetVersion) + ")",
                              r.spec.name, index.string(), {}, "library_version"));
    }
}

}  // namespace

LibraryResult<LibrarySet> open_libraries(std::span<const LibraryRootSpec> roots) {
    Issues issues;
    std::vector<RootData> data;
    std::set<std::string> names;

    // The specs themselves.
    for (const LibraryRootSpec& spec : roots) {
        bool ok = true;
        if (!valid_root_name(spec.name)) {
            issues.push_back(make("invalid_request", "a root name is [a-z][a-z0-9_]*, got '" + spec.name + "'", spec.name,
                                  spec.path.string()));
            ok = false;
        } else if (!names.insert(spec.name).second) {
            issues.push_back(make("duplicate_root", "the root '" + spec.name + "' is given twice", spec.name,
                                  spec.path.string()));
            ok = false;
        }
        if (ok && (spec.path.empty() || !spec.path.is_absolute())) {
            issues.push_back(make("relative_path", "the path of root '" + spec.name + "' must be absolute", spec.name,
                                  spec.path.string()));
            ok = false;
        }
        if (ok) {
            RootData r;
            r.spec = spec;
            data.push_back(std::move(r));
        }
    }

    for (RootData& r : data) {
        read_root(r, issues);
    }

    // Dependencies: each must be opened; no cycles.
    std::unordered_map<std::string, const RootData*> by_name;
    for (const RootData& r : data) {
        by_name[r.spec.name] = &r;
    }
    for (const RootData& r : data) {
        for (const std::string& dep : r.info.depends) {
            if (by_name.count(dep) == 0) {
                issues.push_back(make("dependency_missing",
                                      "root '" + r.spec.name + "' depends on '" + dep + "', which was not opened",
                                      r.spec.name, r.spec.path.string()));
            }
        }
    }
    {
        // Depth-first with colours: 0 unseen, 1 on the stack, 2 done. A cycle is reported at the root that closes it.
        std::unordered_map<std::string, int> colour;
        std::set<std::string> reported;
        auto visit = [&](auto&& self, const RootData& r) -> void {
            colour[r.spec.name] = 1;
            for (const std::string& dep : r.info.depends) {
                const auto it = by_name.find(dep);
                if (it == by_name.end()) {
                    continue;
                }
                const int c = colour[dep];
                if (c == 1 && reported.insert(r.spec.name).second) {
                    issues.push_back(make("dependency_cycle",
                                          "root '" + r.spec.name + "' depends on '" + dep + "', which depends back on it",
                                          r.spec.name, r.spec.path.string()));
                } else if (c == 0) {
                    self(self, *it->second);
                }
            }
            colour[r.spec.name] = 2;
        };
        for (const RootData& r : data) {
            if (colour[r.spec.name] == 0) {
                visit(visit, r);
            }
        }
    }

    // Every id that exists, for the references.
    std::unordered_set<std::string> exists;
    for (const RootData& r : data) {
        for (const SetEntry& e : r.entries) {
            if (!e.id.empty()) {
                exists.insert(e.id);
            }
        }
    }
    // References: full ids of this root or a dependency, naming an entry that exists.
    for (const RootData& r : data) {
        for (const RootData::Ref& ref : r.refs) {
            std::string target_root;
            std::string local;
            if (!split_id(ref.target, target_root, local)) {
                issues.push_back(make("bare_id", "'" + ref.target + "' is not a full id: say '<root>:<id>'", r.spec.name,
                                      ref.path, ref.from_id, ref.field));
                continue;
            }
            const bool allowed = target_root == r.spec.name ||
                                 std::find(r.info.depends.begin(), r.info.depends.end(), target_root) != r.info.depends.end();
            if (!allowed) {
                issues.push_back(make("foreign_root_reference",
                                      "root '" + r.spec.name + "' names '" + ref.target + "' but does not depend on '" +
                                          target_root + "'",
                                      r.spec.name, ref.path, ref.target, ref.field));
                continue;
            }
            if (by_name.count(target_root) != 0 && exists.count(ref.target) == 0) {
                issues.push_back(make("bad_index", "'" + ref.target + "' names no entry", r.spec.name, ref.path, ref.target,
                                      ref.field));
            }
        }
    }
    // Aliases: merged; a key that is an id, or defined twice, is a conflict.
    std::map<std::string, std::string> aliases;
    for (const RootData& r : data) {
        for (const Alias& a : r.aliases) {
            if (exists.count(a.key) != 0) {
                issues.push_back(make("alias_conflict", "the alias '" + a.key + "' is also an id", r.spec.name, a.path, a.key,
                                      "aliases." + a.key));
            } else if (!aliases.emplace(a.key, a.target).second) {
                issues.push_back(make("alias_conflict", "the alias '" + a.key + "' is defined twice", r.spec.name, a.path,
                                      a.key, "aliases." + a.key));
            }
        }
    }

    if (!issues.empty()) {
        LibraryRefusal top = make("open_failed",
                                  std::to_string(issues.size()) + " problem(s) opening the libraries; the first: " +
                                      issues.front().message);
        top.issues = std::move(issues);
        return std::unexpected(std::move(top));
    }
    LibrarySet set;
    set.aliases = std::move(aliases);
    for (RootData& r : data) {
        set.roots.push_back(std::move(r.info));
        for (SetEntry& e : r.entries) {
            set.entries.push_back(std::move(e));
        }
    }
    return set;
}

}  // namespace spade::scene
