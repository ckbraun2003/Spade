// ---------------------------------------------------------------------------
// world/mesh_ref.hpp -- what a visual mesh_ref may be (N11; the lead and Core,
// 2026-10-20). One rule for every reader of one (TD-9): the renderer's mesh
// resolver (render/mesh_resolver.hpp) and, as Core applies it, the scene's,
// the object file's and the library index's validators. It lives here, in the
// lowest library all of them link.
//
// A mesh_ref names a mesh by a RELATIVE PATH under a root, without its
// extension: "props/ball" is <root>/props/ball.spmesh. Segments are joined by
// '/'; each is non-empty and holds no '.', '\' or ':' (so no "." or ".."
// segment, no extension, no drive, no scheme) and no control character. It is
// never absolute. So a ref cannot leave the root it is looked up in.
// ---------------------------------------------------------------------------
#pragma once

#include <string_view>

namespace spade {

// Empty when `ref` is a valid mesh_ref; otherwise a fixed description of the
// first rule it breaks, fit to show after the ref.
[[nodiscard]] inline std::string_view mesh_ref_problem(std::string_view ref) noexcept {
    if (ref.empty()) return "it is empty";
    if (ref.front() == '/') return "it is absolute: it begins with '/'";
    for (const char c : ref) {
        if (c == '\\') return "it holds a '\\': segments are joined by '/'";
        if (c == ':') return "it holds a ':', as a drive or a scheme would";
        if (static_cast<unsigned char>(c) < 0x20u || c == 0x7f) return "it holds a control character";
    }
    std::string_view rest = ref;
    while (true) {
        const std::size_t slash = rest.find('/');
        const std::string_view segment = rest.substr(0, slash);
        if (segment.empty()) return "it has an empty segment: a '//', or a '/' at its end";
        if (segment == "." || segment == "..") return "it has a '.' or '..' segment";
        if (segment.find('.') != std::string_view::npos) {
            return "it holds a '.': a mesh_ref names no extension (.spmesh is added)";
        }
        if (slash == std::string_view::npos) return {};
        rest.remove_prefix(slash + 1u);
    }
}

[[nodiscard]] inline bool valid_mesh_ref(std::string_view ref) noexcept { return mesh_ref_problem(ref).empty(); }

}  // namespace spade
