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
// a drone's parts take one each.
//
// Its name is read_gltf()'s (the node's, else the mesh's), or the file's stem
// when the reader fell back to "node[i]". Its source is the path imported,
// kept in the sidecar's provenance, so it survives a reopen.

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "scene/scene_file.hpp"
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
    std::error_code ec;
    std::filesystem::create_directories(out.sidecar.parent_path(), ec);
    if (ec) {
        return std::unexpected(Error{Code::io_error, prefix + "its sidecar's folder " +
                                                         path_text(out.sidecar.parent_path()) +
                                                         " could not be made: " + ec.message()});
    }
    MeshProvenance provenance;
    provenance.source = out.source;
    provenance.unit = MeshUnit::metre;  // glTF's, by its specification (3.6.1)
    if (Result<void> saved = save_mesh_file(out.sidecar, *out.mesh, provenance); !saved) {
        return std::unexpected(Error{saved.error().code, prefix + "its sidecar was not staged: " +
                                                             saved.error().context});
    }
    return out;
}

}  // namespace spade::sandbox::editor
