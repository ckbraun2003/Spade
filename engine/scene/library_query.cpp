// ---------------------------------------------------------------------------
// scene/library_query.cpp -- see library_query.hpp.
// ---------------------------------------------------------------------------
#include "scene/library_query.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <set>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "world/sha256.hpp"

#include <charconv>

namespace spade::scene {
namespace {

namespace fs = std::filesystem;

LibraryRefusal make(std::string code, std::string message, std::string root = {}, std::string path = {},
                    std::string id = {}, std::string field = {}) {
    return LibraryRefusal{std::move(code), std::move(message), std::move(root), std::move(path),
                          std::move(id),   std::move(field),   {}};
}

bool read_bytes(const fs::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return !in.bad();
}

std::string lower_ascii(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return s;
}

const SetRoot* find_root(const LibrarySet& set, const std::string& name) {
    for (const SetRoot& r : set.roots) {
        if (r.name == name) {
            return &r;
        }
    }
    return nullptr;
}

// The files whose line endings the hash ignores (a fixed list; the contract's section 3).
bool is_text_file(const std::string& rel) {
    const std::string ext = lower_ascii(fs::path(rel).extension().string());
    return ext == ".yaml" || ext == ".yml" || ext == ".json" || ext == ".gltf" || ext == ".md" || ext == ".txt" ||
           ext == ".csv";
}

// CR LF pairs read as LF; a lone CR stays.
std::string fold_crlf(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\r' && i + 1 < s.size() && s[i + 1] == '\n') {
            continue;
        }
        out += s[i];
    }
    return out;
}

bool to_double(const YAML::Node& n, double& out) {
    if (!n.IsScalar()) {
        return false;
    }
    const std::string& s = n.Scalar();
    const char* first = s.data();
    const char* last = s.data() + s.size();
    const auto r = std::from_chars(first, last, out);
    return r.ec == std::errc() && r.ptr == last;
}

// A child of a mapping, or an undefined node when `n` is no mapping or has no such key (never throws).
YAML::Node get(const YAML::Node& n, const char* key) {
    if (!n.IsDefined() || !n.IsMap()) {
        return YAML::Node(YAML::NodeType::Undefined);
    }
    const YAML::Node v = n[key];
    return v.IsDefined() ? YAML::Node(v) : YAML::Node(YAML::NodeType::Undefined);
}

bool entry_has_text(const SetEntry& e, const std::string& needle_lower) {
    if (lower_ascii(e.name).find(needle_lower) != std::string::npos ||
        lower_ascii(e.category).find(needle_lower) != std::string::npos) {
        return true;
    }
    return std::any_of(e.tags.begin(), e.tags.end(),
                       [&](const std::string& t) { return lower_ascii(t).find(needle_lower) != std::string::npos; });
}

bool accepts(const SetEntry& e, const ListFilter& f, const std::string& text_lower) {
    if (!f.roots.empty() && std::find(f.roots.begin(), f.roots.end(), e.root) == f.roots.end()) {
        return false;
    }
    if (!f.kind.empty() && e.kind != f.kind) {
        return false;
    }
    if (!f.part_kind.empty() && e.part_kind != f.part_kind) {
        return false;
    }
    if (!f.type.empty() && e.type != f.type) {
        return false;
    }
    if (!text_lower.empty() && !entry_has_text(e, text_lower)) {
        return false;
    }
    for (const std::string& t : f.tags) {
        if (std::find(e.tags.begin(), e.tags.end(), t) == e.tags.end()) {
            return false;
        }
    }
    return true;
}

}  // namespace

LibraryResult<FindResult> find_entry(const LibrarySet& set, std::string_view id, const FindOptions& options) {
    const std::string full(id);
    const std::size_t colon = full.find(':');
    if (colon == std::string::npos) {
        return std::unexpected(make("bare_id", "'" + full + "' is not a full id: say '<root>:<id>'", {}, {}, full));
    }
    const std::string root = full.substr(0, colon);
    if (find_root(set, root) == nullptr) {
        return std::unexpected(make("root_not_loaded", "root '" + root + "' is not loaded", root, {}, full));
    }
    const SetEntry* entry = nullptr;
    for (const SetEntry& e : set.entries) {
        if (e.id == full) {
            entry = &e;
            break;
        }
    }
    if (entry == nullptr) {
        return std::unexpected(make("entry_not_found", "root '" + root + "' has no entry '" + full + "'", root, {}, full));
    }
    if (options.expect_hash.has_value()) {
        const std::string& pin = *options.expect_hash;
        const bool ok = pin.size() == 7 + 64 && pin.compare(0, 7, "sha256:") == 0 &&
                        std::all_of(pin.begin() + 7, pin.end(),
                                    [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
        if (!ok) {
            return std::unexpected(make("invalid_request", "expect_hash is 'sha256:' and 64 lower-case hex digits", {}, {},
                                        full, "expect_hash"));
        }
    }
    if (entry->in_store) {
        std::error_code ec;
        if (entry->file.empty() || !fs::exists(entry->file, ec)) {
            return std::unexpected(make("store_file_missing",
                                        entry->file.empty() ? "no store was given, so '" + entry->store_file + "' cannot be found"
                                                            : "the store has no '" + entry->store_file + "' at its address",
                                        root, entry->file.string(), full, "store"));
        }
    }
    LibraryResult<std::string> h = entry_hash(set, *entry);
    if (!h) {
        return std::unexpected(h.error());
    }
    FindResult result;
    result.entry = entry;
    result.hash = *h;
    if (options.expect_hash.has_value()) {
        result.hash_matches = *options.expect_hash == result.hash;
        if (!*result.hash_matches) {
            result.warnings.push_back(make("hash_mismatch", "the entry's hash is " + result.hash + ", not the pinned " + *options.expect_hash,
                                           root, {}, full));
        }
    }
    if (options.with_record && entry->kind == "part" && !entry->file.empty()) {
        LibraryResult<std::string> json = yaml_file_to_canonical_json(entry->file);
        if (!json) {
            LibraryRefusal why = json.error();
            why.id = full;
            why.root = root;
            return std::unexpected(std::move(why));
        }
        result.record_json = *json;
    }
    return result;
}

LibraryResult<std::string> entry_hash(const LibrarySet& set, const SetEntry& entry) {
    const SetRoot* root = find_root(set, entry.root);
    if (root == nullptr) {
        return std::unexpected(make("root_not_loaded", "root '" + entry.root + "' is not loaded", entry.root, {}, entry.id));
    }
    std::vector<std::pair<std::string, std::string>> lines;  // relative path, sha256 hex of the bytes
    auto add = [&](const std::string& rel) -> bool {
        std::string bytes;
        const fs::path path = root->path / rel;
        if (!read_bytes(path, bytes)) {
            return false;
        }
        lines.emplace_back(rel, spade::sha256_hex(is_text_file(rel) ? fold_crlf(bytes) : bytes));
        return true;
    };
    if (entry.in_store) {
        // The content address is the proof: nothing is read, so the hash is the same with or without the store.
        lines.emplace_back("store/" + entry.store_sha256 + "/" + entry.store_file, entry.store_sha256);
    }
    if (!entry.rel_file.empty() && !add(entry.rel_file)) {
        return std::unexpected(make("io_error", "cannot read '" + entry.rel_file + "'", entry.root,
                                    (root->path / entry.rel_file).string(), entry.id, "file"));
    }
    for (const std::string& f : entry.files) {
        if (!add(f)) {
            return std::unexpected(make("io_error", "cannot read '" + f + "'", entry.root, (root->path / f).string(),
                                        entry.id, "files"));
        }
    }
    std::sort(lines.begin(), lines.end());
    std::string text;
    for (const auto& [rel, hex] : lines) {
        text += rel + "\t" + hex + "\n";
    }
    if (entry.has_parts_map) {
        text += "parts_map\t" + spade::sha256_hex(entry.parts_map_json) + "\n";
    }
    return "sha256:" + spade::sha256_hex(text);
}

std::optional<EntryPeek> peek_entry(const SetEntry& entry, std::string* why) {
    auto fail = [&](const std::string& message) -> std::optional<EntryPeek> {
        if (why != nullptr) {
            *why = message;
        }
        return std::nullopt;
    };
    EntryPeek peek;
    const bool has_header = entry.kind == "model" || entry.kind == "drone" || entry.kind == "part" || entry.kind == "world";
    if (!has_header || entry.file.empty()) {
        return peek;  // a mesh, an opaque entry, an object, a file-less drone: nothing to read, and nothing wrong
    }
    YAML::Node doc;
    try {
        doc = YAML::LoadFile(entry.file.string());
    } catch (const std::exception& e) {
        return fail(e.what());
    }
    if (!doc.IsMap()) {
        return fail("the file is not a mapping");
    }
    if (entry.kind == "world") {
        double v = 0.0;
        if (to_double(get(doc, "world_version"), v) && v >= 0.0) {
            peek.world_version = static_cast<uint32_t>(v);
        }
    } else if (entry.kind == "model" || entry.kind == "drone") {
        const YAML::Node model = get(doc, "model");
        double m = 0.0;
        if (to_double(get(get(model, "body"), "mass"), m)) {
            peek.mass_kg = m;
        }
        if (const YAML::Node rotors = get(model, "rotors"); rotors.IsSequence()) {
            peek.rotor_count = static_cast<uint32_t>(rotors.size());
        }
    } else {  // part
        const YAML::Node block = get(doc, entry.part_kind.c_str());
        double m = 0.0;
        if (to_double(get(block, "mass"), m)) {
            peek.mass_kg = m;
        }
        if (const YAML::Node p = get(doc, "provides"); p.IsSequence()) {
            for (const YAML::Node& item : p) {
                if (item.IsScalar()) {
                    peek.provides.push_back(item.Scalar());
                }
            }
        }
        if (const YAML::Node l = get(doc, "limits"); l.IsMap()) {
            for (const auto& kv : l) {
                double v = 0.0;
                if (kv.first.IsScalar() && to_double(kv.second, v)) {
                    peek.limits[kv.first.Scalar()] = v;
                }
            }
        }
        if (const YAML::Node pts = get(doc, "install_points"); pts.IsSequence()) {
            std::set<std::string> kinds;
            for (const YAML::Node& pt : pts) {
                if (const YAML::Node k = get(pt, "kind"); k.IsScalar()) {
                    kinds.insert(k.Scalar());
                }
            }
            peek.install_kinds.assign(kinds.begin(), kinds.end());
        }
    }
    return peek;
}

LibraryResult<ListPage> list_entries(const LibrarySet& set, const ListFilter& filter) {
    static const std::set<std::string> kinds = {"",      "model", "object", "part",
                                                "drone", "world", "mesh",   "opaque"};
    if (kinds.count(filter.kind) == 0) {
        return std::unexpected(make("invalid_request",
                                    "kind must be model, object, part, drone, world, mesh or opaque, got '" + filter.kind + "'",
                                    {}, {}, {}, "kind"));
    }
    for (const std::string& r : filter.roots) {
        if (find_root(set, r) == nullptr) {
            return std::unexpected(make("root_not_loaded", "root '" + r + "' is not loaded", r));
        }
    }
    const std::string text = lower_ascii(filter.text);
    ListPage page;
    page.offset = filter.offset;
    std::size_t index = 0;
    for (const SetEntry& e : set.entries) {
        if (!accepts(e, filter, text)) {
            continue;
        }
        const std::size_t i = index++;
        if (i < filter.offset || (filter.limit.has_value() && i >= filter.offset + *filter.limit)) {
            continue;
        }
        ListedEntry row;
        row.entry = &e;
        if (LibraryResult<std::string> h = entry_hash(set, e)) {
            row.hash = *h;
        } else {
            page.warnings.push_back(h.error());
        }
        std::string why;
        std::error_code ec;
        if (e.in_store && (e.file.empty() || !fs::exists(e.file, ec))) {
            // A store file that is not there: a warning, and an empty peek (not a failed one).
            page.warnings.push_back(make("store_file_missing",
                                         e.file.empty() ? "no store was given, so '" + e.store_file + "' cannot be found"
                                                        : "the store has no '" + e.store_file + "' at its address",
                                         e.root, e.file.string(), e.id, "store"));
            row.peek = EntryPeek{};
        } else {
            row.peek = peek_entry(e, &why);
        }
        if (!row.peek.has_value()) {
            page.warnings.push_back(make("peek_failed", "cannot read '" + e.rel_file + "': " + why, e.root, e.file.string(),
                                         e.id, "file"));
        }
        page.rows.push_back(std::move(row));
    }
    page.total = index;
    return page;
}

LibraryResult<ResolveResult> resolve_alias(const LibrarySet& set, std::string_view alias, std::string_view root) {
    const std::string key(alias);
    if (key.empty()) {
        LibraryRefusal r = make("invalid_request", "an alias is a non-empty string");
        r.field = "alias";
        return std::unexpected(std::move(r));
    }
    const std::map<std::string, std::string>* table = &set.aliases;
    if (!root.empty()) {
        const auto it = std::find_if(set.roots.begin(), set.roots.end(), [&](const SetRoot& r) { return r.name == root; });
        if (it == set.roots.end()) {
            LibraryRefusal r = make("root_not_loaded", "root '" + std::string(root) + "' is not loaded", std::string(root));
            r.field = "root";
            return std::unexpected(std::move(r));
        }
        table = &it->aliases;
    }
    const auto found = table->find(key);
    if (found == table->end()) {
        LibraryRefusal r = make("entry_not_found", "no alias '" + key + "'" + (root.empty() ? std::string() : " in root '" + std::string(root) + "'"));
        r.field = "alias";
        if (!root.empty()) r.root = std::string(root);
        return std::unexpected(std::move(r));
    }
    ResolveResult out;
    out.id = found->second;
    out.root = out.id.substr(0, out.id.find(':'));
    return out;
}

std::string hash_text(std::string_view text) { return "sha256:" + spade::sha256_hex(text); }

}  // namespace spade::scene
