// N15: the editor's glTF import (the plan, tasks/n15-plan.md, approved by
// the lead 2026-10-22). Core does the reading and the writing: read_gltf()
// (world/mesh_import.hpp) turns the file into validated meshes, refusing an
// open or non-manifold solid in its own words, and save_mesh_file()
// (world/mesh_file.hpp) writes the sidecar. This header mints the mesh ref,
// refuses a clash, and stages the sidecar.
//
// THE REF (the lead, 2026-10-20): imported/<stem>, each UTF-8 code point
// outside [A-Za-z0-9_-] one '_', then mesh_ref_problem() before anything is
// written. A stem with no letter or digit is refused.
//
// A CLASH IS REFUSED, NEVER RENAMED (D3): a ref this scene's assets, its
// world, or its models' part sidecars already name, a file beside the scene,
// or a sidecar staged this session. The refusal names who holds it.
//
// STAGED, NOT BESIDE THE SCENE (D1): the sidecar is written to
// <staging>/<ref>.spmesh, the editor's own folder, which the view resolves
// after the scene's folder. Save copies it beside the scene. Nothing is
// written beside a scene until Save, so an import into a built-in scene never
// writes into assets/.
//
// AN OBJECT IS ONE MESH (D2): a file holding several is refused as an object;
// a drone's parts take one each, under imported/<stem>/<mesh>, staged all or
// none.
//
// A PART (Physics' D4, 2026-10-22): imported_part() puts it on the body at
// glTF's convention, its +Z front to the design frame's +X nose (+90 degrees
// about +Y), and declares its min_feature as the smallest extent of its own
// box, the classifier's own value for an undeclared one (ModelType's
// validator wants one declared, CORE-8 D5). It is a solid in the airflow
// (analysis::kImportedPartRole), or drawn only (vehicles::kDrawnOnlyRole)
// when under 8 mm in two extents, which the grid can't resolve (L7). It is
// massless and drags nothing in flight, and its rows say so.
//
// Its name is read_gltf()'s (the node's, else the mesh's), or the file's stem
// when the reader fell back to "node[i]". Its source is the path imported,
// kept in the sidecar's provenance, so it survives a reopen.

#pragma once

#include <algorithm>
#include <filesystem>
#include <format>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "analysis/live_run.hpp"
#include "core/error.hpp"
#include "core/path_text.hpp"
#include "scene/scene_file.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/model_part.hpp"
#include "world/builder.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "world/mesh_import.hpp"
#include "world/mesh_ref.hpp"

namespace spade::sandbox::editor {

// One imported mesh, staged.
struct ImportedMesh {
    std::string name;                         // what the editor names it
    std::string ref;                          // imported/<stem>
    std::shared_ptr<const TriangleMesh> mesh;  // read_gltf()'s, in metres
    std::filesystem::path sidecar;            // <staging>/<ref>.spmesh
    std::string source;                       // the path imported, UTF-8
};

// glTF's frame on the design frame's (D4): +90 degrees about +Y, so its +Z
// front is the nose (+X) and its -X right is +Z. (w, x, y, z).
inline const glm::quat kImportedMount{0.7071067811865476f, 0.0f, 0.7071067811865476f, 0.0f};

// Under this in two extents, an imported part is drawn only (D4, L7).
inline constexpr float kDrawnOnlyExtent = 0.008f;  // m

namespace detail {

[[nodiscard]] inline std::string import_prefix(const std::filesystem::path& file) {
    return "import '" + path_text(file) + "': ";
}

[[nodiscard]] inline bool ref_char(unsigned char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
}

// `text` (UTF-8) with each code point outside [A-Za-z0-9_-] as one '_'.
[[nodiscard]] inline std::string ref_segment(std::string_view text) {
    std::string out;
    for (std::size_t i = 0; i < text.size();) {
        const auto c = static_cast<unsigned char>(text[i++]);
        if (ref_char(c)) {
            out += static_cast<char>(c);
            continue;
        }
        out += '_';
        while (i < text.size() && (static_cast<unsigned char>(text[i]) & 0xC0u) == 0x80u) ++i;
    }
    return out;
}

// read_gltf()'s name for a node and mesh that have none: "node[<digits>]".
[[nodiscard]] inline bool reader_fallback_name(std::string_view name) {
    constexpr std::string_view head = "node[";
    if (name.size() <= head.size() + 1 || name.substr(0, head.size()) != head || name.back() != ']') return false;
    for (const char c : name.substr(head.size(), name.size() - head.size() - 1)) {
        if (c < '0' || c > '9') return false;
    }
    return true;
}

// Who already holds `ref`, as a clause; empty when nobody does.
[[nodiscard]] inline std::string ref_holder(std::string_view ref, const scene::SceneDesc& scene,
                                            const WorldDesc& world, const std::filesystem::path& scene_dir,
                                            const std::filesystem::path& staging) {
    for (const scene::SceneAsset& a : scene.assets) {
        if (a.visual.mesh_ref == ref) return "this scene's asset '" + a.name + "' uses it";
    }
    for (const PropDesc& p : world.props) {
        if (p.mesh_ref == ref) return "the world's props use it";
    }
    for (const std::string& r : world.visual_refs) {
        if (r == ref) return "the world's visuals use it";
    }
    const std::string file = std::string(ref) + ".spmesh";
    for (const vehicles::ModelType& m : scene.models) {
        for (const vehicles::PartDesc& part : m.parts) {
            if (part.mesh_file == file) return "model '" + m.name + "', part '" + part.name + "' uses it";
        }
    }
    std::error_code ec;
    if (const std::filesystem::path beside = scene_dir / file; std::filesystem::exists(beside, ec)) {
        return "a file beside the scene holds it (" + path_text(beside) + ")";
    }
    if (const std::filesystem::path staged = staging / file; std::filesystem::exists(staged, ec)) {
        return "it was imported already this session (" + path_text(staged) + ")";
    }
    return {};
}

// Writes `m`'s sidecar, its folder first.
[[nodiscard]] inline Result<void> stage(const ImportedMesh& m, const std::string& prefix) {
    std::error_code ec;
    std::filesystem::create_directories(m.sidecar.parent_path(), ec);
    if (ec) {
        return std::unexpected(Error{Code::io_error, prefix + "its sidecar's folder " +
                                                         path_text(m.sidecar.parent_path()) +
                                                         " could not be made: " + ec.message()});
    }
    MeshProvenance provenance;
    provenance.source = m.source;
    provenance.unit = MeshUnit::metre;  // glTF's, by its specification (3.6.1)
    if (Result<void> saved = save_mesh_file(m.sidecar, *m.mesh, provenance); !saved) {
        return std::unexpected(Error{saved.error().code, prefix + "its sidecar was not staged: " +
                                                             saved.error().context});
    }
    return {};
}

// The extents of `mesh`'s box, in its own frame.
[[nodiscard]] inline glm::vec3 extent(const TriangleMesh& mesh) {
    const std::span<const glm::vec3> ps = mesh.positions();
    if (ps.empty()) return glm::vec3(0.0f);
    glm::vec3 lo = ps.front();
    glm::vec3 hi = ps.front();
    for (const glm::vec3& p : ps) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }
    return hi - lo;
}

[[nodiscard]] inline float smallest(glm::vec3 e) { return std::min({e.x, e.y, e.z}); }

[[nodiscard]] inline std::string mm(float m) { return std::format("{:g} mm", m * 1000.0f); }

}  // namespace detail

// The mesh ref an import of `file` takes: imported/<stem>.
[[nodiscard]] inline Result<std::string> mint_ref(const std::filesystem::path& file) {
    const std::string stem = path_text(file.stem());
    const std::string segment = detail::ref_segment(stem);
    bool named = false;
    for (const char c : segment) named = named || (c != '_' && c != '-');
    if (!named) {
        return std::unexpected(Error{Code::invalid_argument, detail::import_prefix(file) + "its name '" + stem +
                                                                 "' has no letter or digit to mint a mesh ref from"});
    }
    std::string ref = "imported/" + segment;
    if (const std::string_view why = mesh_ref_problem(ref); !why.empty()) {
        return std::unexpected(Error{Code::invalid_argument, detail::import_prefix(file) + "the mesh ref '" + ref +
                                                                 "' is refused: " + std::string(why)});
    }
    return ref;
}

// Reads `file` as one object and stages its sidecar. The scene, its world and
// its folder are read for clashes only; nothing is written but the sidecar,
// and nothing at all on a refusal.
[[nodiscard]] inline Result<ImportedMesh> import_gltf_object(const std::filesystem::path& file,
                                                            const std::filesystem::path& staging,
                                                            const scene::SceneDesc& scene, const WorldDesc& world,
                                                            const std::filesystem::path& scene_dir) {
    const std::string prefix = detail::import_prefix(file);
    Result<std::string> ref = mint_ref(file);
    if (!ref) return std::unexpected(ref.error());
    if (const std::string holder = detail::ref_holder(*ref, scene, world, scene_dir, staging); !holder.empty()) {
        return std::unexpected(Error{Code::invalid_argument, prefix + "the mesh ref '" + *ref + "' is taken: " +
                                                                 holder +
                                                                 "; rename the file to import it again (an import "
                                                                 "is never renamed)"});
    }
    Result<std::vector<MeshObject>> read = read_gltf(file);
    if (!read) return std::unexpected(Error{read.error().code, prefix + read.error().context});
    if (read->size() != 1) {
        return std::unexpected(Error{Code::invalid_argument,
                                     prefix + "it holds " + std::to_string(read->size()) +
                                         " meshes; an object is one mesh: import it as a drone's parts, or split it"});
    }
    const MeshObject& object = read->front();
    ImportedMesh out;
    out.name = detail::reader_fallback_name(object.name) ? path_text(file.stem()) : object.name;
    out.ref = *ref;
    out.mesh = object.mesh;
    out.source = path_text(file);
    out.sidecar = staging / std::filesystem::path(*ref + ".spmesh");
    if (Result<void> staged = detail::stage(out, prefix); !staged) return std::unexpected(staged.error());
    return out;
}

// Reads `file` as a drone's parts: every mesh, each under
// imported/<stem>/<its name>, staged all or none. Two meshes of one name, or
// of one ref, are refused naming them; so is a ref already held (D3).
[[nodiscard]] inline Result<std::vector<ImportedMesh>> import_gltf_parts(const std::filesystem::path& file,
                                                                        const std::filesystem::path& staging,
                                                                        const scene::SceneDesc& scene,
                                                                        const WorldDesc& world,
                                                                        const std::filesystem::path& scene_dir) {
    const std::string prefix = detail::import_prefix(file);
    const auto refuse = [&](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, prefix + why});
    };
    Result<std::string> stem = mint_ref(file);
    if (!stem) return std::unexpected(stem.error());
    Result<std::vector<MeshObject>> read = read_gltf(file);
    if (!read) return std::unexpected(Error{read.error().code, prefix + read.error().context});
    if (read->empty()) return refuse("it holds no mesh");
    std::vector<ImportedMesh> out;
    for (std::size_t i = 0; i < read->size(); ++i) {
        const MeshObject& object = (*read)[i];
        ImportedMesh m;
        m.name = object.name;
        if (detail::reader_fallback_name(object.name)) {
            m.name = path_text(file.stem()) + (read->size() == 1 ? "" : "_" + std::to_string(i));
        }
        m.ref = *stem + "/" + detail::ref_segment(m.name);
        for (const ImportedMesh& before : out) {
            if (before.name == m.name) {
                return refuse("two of its meshes are named '" + m.name +
                              "'; a drone's parts need a name each: rename one in the file");
            }
            if (before.ref == m.ref) {
                return refuse("its meshes '" + before.name + "' and '" + m.name + "' both take the mesh ref '" +
                              m.ref + "': rename one in the file");
            }
        }
        if (const std::string_view why = mesh_ref_problem(m.ref); !why.empty()) {
            return refuse("the mesh ref '" + m.ref + "' is refused: " + std::string(why));
        }
        if (const std::string holder = detail::ref_holder(m.ref, scene, world, scene_dir, staging); !holder.empty()) {
            return refuse("the mesh ref '" + m.ref + "' is taken: " + holder +
                          "; rename the file to import it again (an import is never renamed)");
        }
        m.mesh = object.mesh;
        m.source = path_text(file);
        m.sidecar = staging / std::filesystem::path(m.ref + ".spmesh");
        out.push_back(std::move(m));
    }
    for (const ImportedMesh& m : out) {
        if (Result<void> staged = detail::stage(m, prefix); !staged) {
            std::error_code ec;
            for (const ImportedMesh& written : out) std::filesystem::remove(written.sidecar, ec);
            return std::unexpected(staged.error());
        }
    }
    return out;
}

// `m` as a part of a drone (D4, and Physics' min_feature ruling (a)).
[[nodiscard]] inline vehicles::PartDesc imported_part(const ImportedMesh& m) {
    vehicles::PartDesc p;
    p.name = m.name;
    p.attach = vehicles::PartAttach::body;
    p.mount_pos = glm::vec3(0.0f);
    p.mount_orient = kImportedMount;
    p.mesh = m.mesh;
    p.mesh_file = m.ref + ".spmesh";
    const glm::vec3 e = m.mesh ? detail::extent(*m.mesh) : glm::vec3(0.0f);
    p.min_feature = detail::smallest(e);
    const int thin = int{e.x < kDrawnOnlyExtent} + int{e.y < kDrawnOnlyExtent} + int{e.z < kDrawnOnlyExtent};
    p.role = std::string(thin >= 2 ? vehicles::kDrawnOnlyRole : analysis::kImportedPartRole);
    return p;
}

// A part imported_part() made: its role is one of the two it gives.
[[nodiscard]] inline bool is_imported_part(const vehicles::PartDesc& p) {
    return p.role == analysis::kImportedPartRole || p.role == vehicles::kDrawnOnlyRole;
}

// What an imported part's rows say it is, so nobody takes the flight as
// including it (D4); empty for an authored part.
[[nodiscard]] inline std::string imported_part_note(const vehicles::PartDesc& p) {
    if (p.role == analysis::kImportedPartRole) {
        return "drawn and a solid in the airflow; not weighed, and no drag in flight";
    }
    if (p.role == vehicles::kDrawnOnlyRole) return "drawn only: not in the airflow, not weighed, and no drag in flight";
    return {};
}

// Where an imported part's min_feature came from: its box's, or the user's.
[[nodiscard]] inline std::string min_feature_note(const vehicles::PartDesc& p) {
    const std::string set = "min_feature: " + detail::mm(p.min_feature);
    if (p.mesh) {
        if (const float box = detail::smallest(detail::extent(*p.mesh)); p.min_feature != box) {
            return set + ", set by hand (the bounding box's is " + detail::mm(box) + ")";
        }
    }
    return set + ", the bounding box's (not measured from the mesh)";
}

}  // namespace spade::sandbox::editor
