#include "render/mesh_resolver.hpp"

#include <algorithm>
#include <system_error>
#include <utility>

#include <glm/geometric.hpp>

#include "core/path_text.hpp"
#include "render/tessellate.hpp"
#include "world/mesh_file.hpp"
#include "world/mesh_ref.hpp"

namespace spade::render {

MeshData flat_mesh(const TriangleMesh& mesh) {
    MeshData out;
    const std::span<const glm::vec3> p = mesh.positions();
    const std::span<const uint32_t> idx = mesh.indices();
    out.positions.reserve(idx.size());
    out.normals.reserve(idx.size());
    out.indices.reserve(idx.size());
    for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
        const glm::vec3 a = p[idx[t]], b = p[idx[t + 1]], c = p[idx[t + 2]];
        const glm::vec3 n = glm::cross(b - a, c - a);
        const float len = glm::length(n);
        const glm::vec3 unit = len > 0.0f ? n / len : glm::vec3(0.0f, 1.0f, 0.0f);
        for (const glm::vec3& v : {a, b, c}) {
            out.indices.push_back(static_cast<uint32_t>(out.positions.size()));
            out.positions.push_back(v);
            out.normals.push_back(unit);
        }
    }
    return out;
}

MeshData missing_mesh_stand_in() {
    // tessellate_primitive()'s box, a half-extent of 0.1 m about its origin,
    // with one material of its own: unlit magenta, which no lit surface
    // takes, so it reads as missing at a glance.
    const Aabb around{.min = glm::vec3(-1.0f), .max = glm::vec3(1.0f)};
    auto box = tessellate_primitive(SdfPrim::box, glm::vec4(0.1f, 0.1f, 0.1f, 0.0f), around);
    MeshData out = box ? std::move(*box) : MeshData{};
    out.source_materials = {Material{.base_color = glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), .shading = 1u}};
    out.submesh_first_index = {0u};
    out.submesh_index_count = {static_cast<uint32_t>(out.indices.size())};
    out.submesh_material = {0u};
    return out;
}

namespace {

// `ref` from the first root that holds it, or why not.
struct Lookup {
    std::optional<MeshData> mesh;
    std::size_t root = 0;
    std::filesystem::path file;
    std::string why;
};

[[nodiscard]] Lookup look_up(const std::string& ref, std::span<const std::filesystem::path> roots) {
    Lookup out;
    if (const std::string_view problem = mesh_ref_problem(ref); !problem.empty()) {
        out.why = "not a mesh_ref: " + std::string(problem);
        return out;
    }
    if (roots.empty()) {
        out.why = "no roots to look in";
        return out;
    }
    const std::string name = ref + ".spmesh";  // UTF-8, as the scene file holds it
    const std::filesystem::path relative(std::u8string(name.begin(), name.end()));
    std::string looked;
    for (std::size_t i = 0; i < roots.size(); ++i) {
        const std::filesystem::path file = roots[i] / relative;
        std::error_code ec;
        if (!std::filesystem::is_regular_file(file, ec)) {
            looked += (looked.empty() ? "" : ", ") + path_text(file);
            continue;
        }
        auto loaded = load_mesh_file(file);
        if (!loaded) {
            out.why = loaded.error().context;
            return out;
        }
        out.mesh = flat_mesh(*loaded->mesh);
        out.root = i;
        out.file = file;
        return out;
    }
    out.why = "no " + ref + ".spmesh in its roots: " + looked;
    return out;
}

}  // namespace

ResolvedMeshes resolve_meshes(const WorldDesc& world, std::span<const std::filesystem::path> roots) {
    std::vector<std::string> refs;
    const auto add = [&refs](const std::string& ref) {
        if (std::find(refs.begin(), refs.end(), ref) == refs.end()) refs.push_back(ref);
    };
    for (const PropDesc& prop : world.props) add(prop.mesh_ref);
    for (const std::string& ref : world.visual_refs) add(ref);

    ResolvedMeshes out;
    out.meshes.reserve(refs.size());
    for (const std::string& ref : refs) {
        Lookup found = look_up(ref, roots);
        if (found.mesh) {
            out.meshes.push_back(NamedMesh{ref, std::move(*found.mesh), false});
            if (found.root > 0u) out.from_fallback.push_back(FallbackMesh{ref, std::move(found.file), found.root});
        } else {
            out.meshes.push_back(NamedMesh{ref, missing_mesh_stand_in(), true});
            out.unresolved.push_back(UnresolvedMesh{ref, std::move(found.why)});
        }
    }
    return out;
}

}  // namespace spade::render
