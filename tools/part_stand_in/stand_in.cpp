#include "stand_in.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <optional>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>

#include "render/gltf.hpp"
#include "scene/part_blocks.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"

namespace spade::stand_in {

namespace {

namespace fs = std::filesystem;

[[nodiscard]] std::string lower_extension(const fs::path& file) {
    std::string ext = file.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

struct Bounds {
    glm::dvec3 lo{1e300};
    glm::dvec3 hi{-1e300};
    bool any = false;
    void add(const glm::vec3& p) {
        const glm::dvec3 q(p);
        lo = glm::min(lo, q);
        hi = glm::max(hi, q);
        any = true;
    }
    [[nodiscard]] Box box() const { return {(lo + hi) * 0.5, hi - lo}; }
};

[[nodiscard]] bool ends_with(const std::string& text, const std::string& tail) {
    return !tail.empty() && text.size() >= tail.size() && text.compare(text.size() - tail.size(), tail.size(), tail) == 0;
}

// The names of meshes[0].primitives[i]'s materials, by primitive; "" for a primitive with none.
[[nodiscard]] std::expected<std::vector<std::string>, std::string> primitive_material_names(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::unexpected(file.string() + ": cannot be opened");
    const nlohmann::json doc = nlohmann::json::parse(in, nullptr, false);
    if (doc.is_discarded() || !doc.is_object()) return std::unexpected(file.string() + ": is not a glTF JSON document");
    std::vector<std::string> names;
    const auto meshes = doc.find("meshes");
    if (meshes == doc.end() || !meshes->is_array() || meshes->empty()) return names;
    const auto prims = (*meshes)[0].find("primitives");
    if (prims == (*meshes)[0].end() || !prims->is_array()) return names;
    for (const nlohmann::json& p : *prims) {
        std::string name;
        if (p.is_object() && p.contains("material") && p["material"].is_number_unsigned() && doc.contains("materials") && doc["materials"].is_array()) {
            const std::size_t idx = p["material"].get<std::size_t>();
            if (idx < doc["materials"].size() && doc["materials"][idx].is_object() && doc["materials"][idx].contains("name") && doc["materials"][idx]["name"].is_string()) {
                name = doc["materials"][idx]["name"].get<std::string>();
            }
        }
        names.push_back(std::move(name));
    }
    return names;
}

[[nodiscard]] std::string number_text(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.5f", v);
    std::string s = buf;
    while (!s.empty() && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    if (s == "-0" || s.empty()) s = "0";
    return s;
}

[[nodiscard]] std::string triple_text(const glm::dvec3& v) {
    return "[" + number_text(v.x) + ", " + number_text(v.y) + ", " + number_text(v.z) + "]";
}

[[nodiscard]] double volume_of(const Box& b) { return b.size.x * b.size.y * b.size.z; }

// The key of the last top-level (column 0, not a comment, not blank) line of the text, "" if none.
[[nodiscard]] std::string last_top_level_key(const std::string& text) {
    std::string key;
    std::size_t at = 0;
    while (at < text.size()) {
        std::size_t end = text.find('\n', at);
        if (end == std::string::npos) end = text.size();
        const std::string line = text.substr(at, end - at);
        if (!line.empty() && line[0] != ' ' && line[0] != '\t' && line[0] != '#' && line[0] != '\r' && line[0] != '-') {
            const std::size_t colon = line.find(':');
            if (colon != std::string::npos) key = line.substr(0, colon);
        }
        at = end + 1;
    }
    return key;
}

}  // namespace

std::expected<std::vector<Box>, std::string> boxes_of_file(const fs::path& file, const Options& options) {
    const std::string ext = lower_extension(file);
    std::vector<Box> out;
    if (ext == ".spmesh") {
        auto loaded = load_mesh_file(file);
        if (!loaded) return std::unexpected(file.string() + ": " + loaded.error().context);
        Bounds b;
        for (const glm::vec3& p : loaded->mesh->positions()) b.add(p);
        if (!b.any) return std::unexpected(file.string() + ": the mesh has no vertices");
        out.push_back(b.box());
        return out;
    }
    if (ext == ".gltf") {
        auto data = render::load_gltf(file);
        if (!data) return std::unexpected(file.string() + ": " + data.error().context);
        auto names = primitive_material_names(file);
        if (!names) return std::unexpected(names.error());
        const bool whole = data->submesh_first_index.empty();
        const std::size_t count = whole ? 1u : data->submesh_first_index.size();
        for (std::size_t s = 0; s < count; ++s) {
            const std::string name = s < names->size() ? (*names)[s] : std::string();
            if (std::any_of(options.skip_materials.begin(), options.skip_materials.end(), [&](const std::string& tail) { return ends_with(name, tail); })) continue;
            const std::size_t first = whole ? 0u : data->submesh_first_index[s];
            const std::size_t n = whole ? data->indices.size() : data->submesh_index_count[s];
            Bounds b;
            for (std::size_t i = first; i < first + n && i < data->indices.size(); ++i) {
                const uint32_t v = data->indices[i];
                if (v < data->positions.size()) b.add(data->positions[v]);
            }
            if (b.any) out.push_back(b.box());
        }
        if (out.empty()) return std::unexpected(file.string() + ": no submesh is left after skipping the named materials");
        return out;
    }
    return std::unexpected(file.string() + ": the extension '" + ext + "' is neither spmesh nor gltf");
}

std::expected<StandInResult, std::string> add_stand_in(std::string_view part_text, const fs::path& store, const Options& options) {
    std::string body(part_text);
    // A block of ours from an earlier run goes: the text before the marker is the part's own.
    if (const std::size_t m = body.find(kMarker); m != std::string::npos) {
        std::size_t line = body.rfind('\n', m);
        line = line == std::string::npos ? 0u : line + 1u;
        body.erase(line);
    }
    const std::string eol = body.find("\r\n") != std::string::npos ? "\r\n" : "\n";
    const auto blocks = scene::read_part_blocks(body);
    if (!blocks) return std::unexpected("the text is not a part: " + blocks.error().context);
    StandInResult result;
    result.text = body;
    if (blocks->meshes.empty()) return result;
    if (!blocks->source || blocks->source->id.empty()) return std::unexpected(std::string("the part has no `source`: the citation of the shapes needs one"));
    if (last_top_level_key(body) != "sources") return std::unexpected(std::string("the part's last top-level key must be `sources`: the generated citation continues it"));

    std::vector<Box> all;
    for (const scene::PartMesh& m : blocks->meshes) {
        const fs::path file = store / m.sha256 / m.file;
        std::error_code ec;
        if (!fs::exists(file, ec)) return std::unexpected("store file missing: " + file.string());
        auto boxes = boxes_of_file(file, options);
        if (!boxes) return std::unexpected(boxes.error());
        all.insert(all.end(), boxes->begin(), boxes->end());
    }
    std::vector<Box> kept;
    for (const Box& b : all) {
        if (volume_of(b) >= options.min_volume_m3) kept.push_back(b);
    }
    if (kept.empty()) {
        kept.push_back(*std::max_element(all.begin(), all.end(), [](const Box& a, const Box& b) { return volume_of(a) < volume_of(b); }));
    }
    result.boxes = kept.size();
    result.skipped = all.size() - kept.size();

    if (!body.empty() && body.back() != '\n') result.text += eol;
    result.text += std::string(kMarker) + eol;
    result.text += "  shapes: {source: \"" + blocks->source->id + "\", grade: derived, note: \"generated by tools/part_stand_in from " + std::to_string(all.size()) +
                   " store objects: " + std::to_string(kept.size()) + " axis-aligned boxes in the part's own frame, " + std::to_string(result.skipped) +
                   " objects under 0.2 cm3 left out; a stand-in, drawn only when the store is absent\"}" + eol;
    result.text += "shapes:" + eol;
    for (const Box& b : kept) {
        result.text += "  - {kind: box, size: " + triple_text(b.size) + ", pos: " + triple_text(b.center) + "}" + eol;
    }
    const auto back = scene::read_part_blocks(result.text);
    if (!back) return std::unexpected("the result does not read back: " + back.error().context);
    if (back->shapes.size() != kept.size()) return std::unexpected(std::string("the result does not read back: the shapes differ in number"));
    return result;
}

}  // namespace spade::stand_in
