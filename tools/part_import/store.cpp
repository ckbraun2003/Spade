#include "store.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <format>
#include <fstream>
#include <iterator>
#include <set>
#include <system_error>

#include <nlohmann/json.hpp>

#include "world/sha256.hpp"

namespace spade::partimport {

namespace fs = std::filesystem;

std::string cache_key_text(const KeyInputs& in) {
    return "spade-part-import-key-1;converter=" + in.converter + ";tolerance=" + json_number(in.tolerance_m) + ";angle=" +
           json_number(in.angle_rad) + ";unit=" + in.unit + ";axes=" + in.axes.text() + ";scale=" + json_number(in.scale) +
           ";object=" + in.object;
}

std::string cache_key(const KeyInputs& in) { return sha256_hex(cache_key_text(in)).substr(0, 16); }

namespace {

[[nodiscard]] std::string lower_ascii(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return s;
}

[[nodiscard]] bool is_device_name(const std::string& name) {
    const std::string stem = lower_ascii(name.substr(0, name.find('.')));
    if (stem == "con" || stem == "prn" || stem == "aux" || stem == "nul") return true;
    return stem.size() == 4 && (stem.compare(0, 3, "com") == 0 || stem.compare(0, 3, "lpt") == 0) && stem[3] >= '1' && stem[3] <= '9';
}

[[nodiscard]] Problem io_problem(const std::string& what, const std::error_code& ec) {
    return Problem{"store_io", what + (ec ? ": " + ec.message() : std::string())};
}

[[nodiscard]] std::string hex16(std::uint64_t v) { return std::format("{:016x}", v); }

}  // namespace

std::string safe_file_name(std::string_view name) {
    std::string out;
    for (const char ch : name) {
        const auto c = static_cast<unsigned char>(ch);
        const bool bad = c < 0x20 || ch == '<' || ch == '>' || ch == ':' || ch == '"' || ch == '/' || ch == '\\' || ch == '|' ||
                         ch == '?' || ch == '*';
        out.push_back(bad ? '_' : ch);
    }
    while (!out.empty() && (out.back() == '.' || out.back() == ' ')) out.pop_back();
    if (out.empty()) out = "object";
    if (is_device_name(out)) out.insert(out.begin(), '_');
    return out;
}

std::vector<std::string> unique_file_names(const std::vector<std::string>& names) {
    std::vector<std::string> out;
    std::set<std::string> used;
    for (const std::string& n : names) {
        const std::string base = safe_file_name(n);
        std::string candidate = base;
        for (int k = 2; used.contains(lower_ascii(candidate)); ++k) candidate = base + "-" + std::to_string(k);
        used.insert(lower_ascii(candidate));
        out.push_back(candidate);
    }
    return out;
}

fs::path entry_dir(const fs::path& store, std::string_view sha256, std::string_view key) {
    return store / std::string(sha256) / std::string(key);
}

namespace {

[[nodiscard]] std::string read_text(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

[[nodiscard]] bool write_text(const fs::path& p, std::string_view bytes) {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    out.close();
    return static_cast<bool>(out);
}

[[nodiscard]] std::string record_json(const EntryRecord& r) {
    std::string objects;
    for (const SidecarRecord& s : r.sidecars) {
        if (!objects.empty()) objects.push_back(',');
        objects += JsonObject{}
                       .str("object", s.object)
                       .str("file", s.file)
                       .str("content_hash", s.content_hash)
                       .integer("triangles", s.triangles)
                       .num("volume_m3", s.volume_m3)
                       .line();
    }
    return JsonObject{}
               .integer("import_version", 1)
               .str("sha256", r.sha256)
               .str("key", r.key)
               .str("key_text", r.key_text)
               .str("converter", r.converter)
               .str("source", r.source_name)
               .raw("objects", "[" + objects + "]")
               .line() +
           "\n";
}

}  // namespace

std::optional<EntryRecord> find_entry(const fs::path& store, std::string_view sha256, std::string_view key) {
    const fs::path dir = entry_dir(store, sha256, key);
    std::error_code ec;
    if (!fs::is_regular_file(dir / "import.json", ec)) return std::nullopt;
    const nlohmann::json j = nlohmann::json::parse(read_text(dir / "import.json"), nullptr, false);
    if (j.is_discarded() || !j.is_object()) return std::nullopt;
    try {
        if (j.at("import_version").get<int>() != 1) return std::nullopt;
        EntryRecord r;
        r.sha256 = j.at("sha256").get<std::string>();
        r.key = j.at("key").get<std::string>();
        r.key_text = j.at("key_text").get<std::string>();
        r.converter = j.at("converter").get<std::string>();
        r.source_name = j.at("source").get<std::string>();
        r.dir = dir;
        if (r.sha256 != sha256 || r.key != key) return std::nullopt;
        for (const nlohmann::json& o : j.at("objects")) {
            SidecarRecord s;
            s.object = o.at("object").get<std::string>();
            s.file = o.at("file").get<std::string>();
            s.content_hash = o.at("content_hash").get<std::string>();
            s.triangles = o.at("triangles").get<std::uint32_t>();
            s.volume_m3 = o.at("volume_m3").get<double>();
            if (s.file.empty() || fs::path(s.file).filename().string() != s.file || s.file == "..") return std::nullopt;
            std::uint64_t expected = 0;
            const auto parsed = std::from_chars(s.content_hash.data(), s.content_hash.data() + s.content_hash.size(), expected, 16);
            if (parsed.ec != std::errc() || parsed.ptr != s.content_hash.data() + s.content_hash.size()) return std::nullopt;
            if (!load_mesh_file(dir / s.file, expected).has_value()) return std::nullopt;
            r.sidecars.push_back(std::move(s));
        }
        if (r.sidecars.empty()) return std::nullopt;
        return r;
    } catch (const nlohmann::json::exception&) {
        return std::nullopt;
    }
}

Expected<EntryRecord> write_entry(const fs::path& store, std::string_view source_bytes, std::string_view source_name,
                                  const KeyInputs& inputs, std::span<const StoredObject> objects, std::string_view token) {
    const std::string sha = sha256_hex(source_bytes);
    const std::string key = cache_key(inputs);
    const fs::path sha_dir = store / sha;
    std::error_code ec;
    fs::create_directories(sha_dir, ec);
    if (ec) return std::unexpected(io_problem("cannot create " + sha_dir.string(), ec));

    // The source, as received: kept when it is already there.
    const std::string safe_source = safe_file_name(source_name);
    const fs::path source_file = sha_dir / safe_source;
    std::error_code size_ec;
    if (!(fs::is_regular_file(source_file, ec) && fs::file_size(source_file, size_ec) == source_bytes.size() && !size_ec)) {
        const fs::path tmp_source = sha_dir / (".tmp-" + std::string(token) + "-source");
        if (!write_text(tmp_source, source_bytes)) {
            fs::remove(tmp_source, ec);
            return std::unexpected(io_problem("cannot write " + source_file.string(), {}));
        }
        fs::rename(tmp_source, source_file, ec);
        if (ec) {
            std::error_code ignore;
            fs::remove(tmp_source, ignore);
            return std::unexpected(io_problem("cannot write " + source_file.string(), ec));
        }
    }

    const fs::path dir = sha_dir / key;
    if (auto complete = find_entry(store, sha, key)) return *complete;
    if (fs::exists(dir, ec)) {
        fs::remove_all(dir, ec);
        if (ec) return std::unexpected(io_problem("cannot replace " + dir.string(), ec));
    }

    const fs::path tmp = sha_dir / (".tmp-" + std::string(token) + "-" + key);
    fs::remove_all(tmp, ec);
    fs::create_directories(tmp, ec);
    if (ec) return std::unexpected(io_problem("cannot create " + tmp.string(), ec));
    const auto cleanup_and_fail = [&](Problem p) {
        std::error_code ignore;
        fs::remove_all(tmp, ignore);
        return std::unexpected(std::move(p));
    };

    EntryRecord rec;
    rec.sha256 = sha;
    rec.key = key;
    rec.key_text = cache_key_text(inputs);
    rec.converter = inputs.converter;
    rec.source_name = safe_source;
    rec.dir = dir;
    std::vector<std::string> names;
    for (const StoredObject& o : objects) names.push_back(o.name);
    const std::vector<std::string> files = unique_file_names(names);
    for (std::size_t i = 0; i < objects.size(); ++i) {
        const std::string file = files[i] + ".spmesh";
        if (const auto saved = save_mesh_file(tmp / file, *objects[i].mesh, objects[i].provenance); !saved) {
            return cleanup_and_fail(Problem{"store_io", saved.error().context});
        }
        rec.sidecars.push_back({objects[i].name, file, hex16(objects[i].mesh->content_hash()), objects[i].mesh->triangle_count(),
                                objects[i].mesh->mass_properties().volume});
    }
    if (!write_text(tmp / "import.json", record_json(rec))) return cleanup_and_fail(io_problem("cannot write import.json", {}));
    fs::rename(tmp, dir, ec);
    if (ec) return cleanup_and_fail(io_problem("cannot place " + dir.string(), ec));
    return rec;
}

std::size_t remove_stale_temps(const fs::path& store, std::chrono::seconds max_age) {
    std::size_t removed = 0;
    std::error_code ec;
    if (!fs::is_directory(store, ec)) return 0;
    for (const fs::directory_entry& sha_dir : fs::directory_iterator(store, ec)) {
        std::error_code e1;
        if (!sha_dir.is_directory(e1)) continue;
        for (const fs::directory_entry& child : fs::directory_iterator(sha_dir.path(), e1)) {
            if (child.path().filename().string().rfind(".tmp-", 0) != 0) continue;
            std::error_code e2;
            const auto written = fs::last_write_time(child.path(), e2);
            if (e2 || fs::file_time_type::clock::now() - written <= max_age) continue;
            fs::remove_all(child.path(), e2);
            if (!e2) ++removed;
        }
    }
    return removed;
}

}  // namespace spade::partimport
