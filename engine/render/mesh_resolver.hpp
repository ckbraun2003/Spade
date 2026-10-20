// ---------------------------------------------------------------------------
// render/mesh_resolver.hpp -- a world's visual meshes, found on disk (N11;
// the lead approved it 2026-10-20).
//
// scene_from_world() draws a prop only from a mesh the caller resolved. This
// resolves them: each distinct mesh_ref of a world (its props', then its
// visual_refs) to the FIRST <root>/<ref>.spmesh in `roots`, read through
// load_mesh_file() and drawn flat. A ref is a relative path under its root
// (world/mesh_ref.hpp): "props/ball" is <root>/props/ball.spmesh.
//
// NEVER SILENT. A ref that does not resolve -- not a valid mesh_ref, found in
// no root, or refused by load_mesh_file() -- is stood in for by a visible
// mesh (a 0.2 m box in an unlit magenta of its own) and named, with why, in
// `unresolved`. The caller chooses the policy: the editor draws the stand-in
// and names the ref; a headless render refuses while `unresolved` is not
// empty.
//
// SELF-CONTAINED SCENES. A scene copies what it places from the library, so
// a saved scene resolves from its own folder (EDT-012). The editor passes
// [scene dir, assets/library/meshes]; a ref found only past the first root is
// listed in `from_fallback`, for the editor's Save to copy beside the scene.
// ---------------------------------------------------------------------------
#pragma once

#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include "render/scene.hpp"
#include "world/builder.hpp"
#include "world/mesh.hpp"

namespace spade::render {

// A TriangleMesh drawn flat: each triangle its own three vertices and its
// face normal, one submesh in the scene's palette. The one conversion (TD-9):
// render_parts draws its mesh parts with it too.
[[nodiscard]] MeshData flat_mesh(const TriangleMesh& mesh);

// The stand-in for a ref that does not resolve: a 0.2 m box about its
// origin, in an unlit magenta material of its own.
[[nodiscard]] MeshData missing_mesh_stand_in();

struct UnresolvedMesh {
    std::string ref;
    std::string why;  // which rule it breaks, where it was looked for, or load_mesh_file()'s refusal
};

struct FallbackMesh {
    std::string ref;
    std::filesystem::path file;  // the sidecar it was read from
    std::size_t root = 0;        // its root's index in `roots`, > 0
};

struct ResolvedMeshes {
    // One per distinct ref, in first-use order, resolved or stood in (its
    // stand_in set): pass them to scene_from_world().
    std::vector<NamedMesh> meshes;
    std::vector<UnresolvedMesh> unresolved;  // the stood-in ones, with why
    std::vector<FallbackMesh> from_fallback;  // resolved past the first root
};

// Every distinct mesh_ref of `world`, resolved against `roots` in order.
[[nodiscard]] ResolvedMeshes resolve_meshes(const WorldDesc& world, std::span<const std::filesystem::path> roots);

}  // namespace spade::render
