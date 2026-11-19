#include "mesh_info.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include "render/gltf.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "world/mesh_import.hpp"
#include "world/sha256.hpp"

namespace spade::libc {

namespace fs = std::filesystem;
namespace sc = spade::scene;
using Json = nlohmann::json;

namespace {

sc::LibraryRefusal refusal(std::string code, std::string message, std::string field = {}, std::string path = {}) {
    sc::LibraryRefusal r;
    r.code = std::move(code);
    r.message = std::move(message);
    r.field = std::move(field);
    r.path = std::move(path);
    return r;
}

// The reader failed: invalid_entry with the reader's message as its one issue.
sc::LibraryRefusal unreadable(const std::string& message, const fs::path& file) {
    sc::LibraryRefusal top = refusal("invalid_entry", "the file does not read as a mesh: " + message, {}, file.generic_string());
    sc::LibraryRefusal issue = refusal("invalid_argument", message, {}, file.generic_string());
    top.issues.push_back(std::move(issue));
    return top;
}

struct UnitName {
    const char* text;
    MeshUnit unit;
};
constexpr UnitName kUnits[] = {{"m", MeshUnit::metre},       {"mm", MeshUnit::millimetre}, {"cm", MeshUnit::centimetre},
                               {"um", MeshUnit::micrometre}, {"in", MeshUnit::inch},       {"ft", MeshUnit::foot}};

Json unit_json(MeshUnit unit) {
    for (const UnitName& u : kUnits) {
        if (u.unit == unit) return Json(u.text);
    }
    return Json(nullptr);
}

std::string lower_extension(const fs::path& file) {
    std::string e = file.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return e;
}

struct Totals {
    uint64_t triangles = 0;
    uint64_t vertices = 0;
    double lo[3] = {0, 0, 0};
    double hi[3] = {0, 0, 0};
    bool any = false;

    void add(std::span<const glm::vec3> positions) {
        for (const glm::vec3& p : positions) {
            const double v[3] = {p.x, p.y, p.z};
            for (int a = 0; a < 3; ++a) {
                lo[a] = any ? std::min(lo[a], v[a]) : v[a];
                hi[a] = any ? std::max(hi[a], v[a]) : v[a];
            }
            any = true;
        }
        vertices += positions.size();
    }
};

Json result(const char* container, const Totals& t, Json volume, Json unit, const std::string& sha, Json solid) {
    return Json{{"container", container},
                {"triangles", t.triangles},
                {"vertices", t.vertices},
                {"bbox_min", Json::array({t.lo[0], t.lo[1], t.lo[2]})},
                {"bbox_max", Json::array({t.hi[0], t.hi[1], t.hi[2]})},
                {"volume_m3", std::move(volume)},
                {"unit", std::move(unit)},
                {"sha256", "sha256:" + sha},
                {"solid", std::move(solid)}};
}

Json from_meshes(const char* container, const std::vector<std::shared_ptr<const TriangleMesh>>& meshes, MeshUnit unit, const std::string& sha) {
    Totals t;
    double volume = 0.0;
    for (const auto& m : meshes) {
        t.triangles += m->triangle_count();
        t.add(m->positions());
        volume += m->mass_properties().volume;
    }
    return result(container, t, Json(volume), unit_json(unit), sha, Json(true));
}

}  // namespace

std::expected<Json, sc::LibraryRefusal> mesh_file_info(const fs::path& file, const std::string& unit_text) {
    MeshUnit unit = MeshUnit::unspecified;
    if (!unit_text.empty()) {
        for (const UnitName& u : kUnits) {
            if (unit_text == u.text) unit = u.unit;
        }
        if (unit == MeshUnit::unspecified) {
            return std::unexpected(refusal("invalid_request", "'unit' is one of m, mm, cm, um, in, ft, not '" + unit_text + "'", "unit"));
        }
    }
    const std::string ext = lower_extension(file);
    const bool known = ext == ".spmesh" || ext == ".gltf" || ext == ".glb" || ext == ".3mf" || ext == ".stl" || ext == ".obj";
    if (!known) {
        return std::unexpected(refusal("invalid_request", "the extension '" + ext + "' is none of spmesh, gltf, glb, 3mf, stl, obj", "path", file.generic_string()));
    }
    if ((ext == ".stl" || ext == ".obj") && unit == MeshUnit::unspecified) {
        return std::unexpected(refusal("invalid_request", "an STL or OBJ file declares no unit: say it in `unit` (m, mm, cm, um, in, ft)", "unit"));
    }
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::unexpected(refusal("io_error", "the file cannot be read", {}, file.generic_string()));
    const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const std::string sha = spade::sha256_hex(bytes);

    auto objects = [&](Result<std::vector<MeshObject>> read, const char* container, MeshUnit declared) -> std::expected<Json, sc::LibraryRefusal> {
        if (!read) return std::unexpected(unreadable(read.error().context, file));
        std::vector<std::shared_ptr<const TriangleMesh>> meshes;
        MeshUnit u = declared;
        for (const MeshObject& o : *read) {
            meshes.push_back(o.mesh);
            if (declared == MeshUnit::unspecified) u = o.unit;
        }
        return from_meshes(container, meshes, u, sha);
    };

    if (ext == ".spmesh") {
        auto loaded = load_mesh_file(file);
        if (!loaded) return std::unexpected(unreadable(loaded.error().context, file));
        return from_meshes("spmesh", {loaded->mesh}, MeshUnit::metre, sha);
    }
    if (ext == ".gltf" || ext == ".glb") {
        auto read = read_gltf(file);
        if (read) return objects(std::move(read), "gltf", MeshUnit::metre);
        // A presentation glTF that is not a closed solid still has its counts and box: the render loader reads the positions.
        if (ext == ".gltf") {
            auto drawn = render::load_gltf(file);
            if (drawn) {
                Totals t;
                t.triangles = drawn->indices.size() / 3;
                t.add(drawn->positions);
                return result("gltf", t, Json(nullptr), unit_json(MeshUnit::metre), sha, Json(nullptr));
            }
        }
        return std::unexpected(unreadable(read.error().context, file));
    }
    if (ext == ".stl" || ext == ".obj") {
        return ext == ".stl" ? objects(read_stl(file, unit), "stl", unit) : objects(read_obj(file, unit), "obj", unit);
    }
    if (ext == ".3mf") return objects(read_3mf(file), "3mf", MeshUnit::unspecified);
    return std::unexpected(refusal("internal_error", "unreachable container", {}, file.generic_string()));
}

}  // namespace spade::libc
