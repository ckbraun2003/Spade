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
        row.peek = peek_entry(e, &why);
        if (!row.peek.has_value()) {
            page.warnings.push_back(make("peek_failed", "cannot read '" + e.rel_file + "': " + why, e.root, e.file.string(),
                                         e.id, "file"));
        }
        page.rows.push_back(std::move(row));
    }
    page.total = index;
    return page;
}

}  // namespace spade::scene
