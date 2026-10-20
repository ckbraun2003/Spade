// ---------------------------------------------------------------------------
// world/mesh_ref.hpp -- what a visual mesh_ref may be (N11; the lead and Core,
// 2026-10-20). One rule for every reader of one (TD-9): the renderer's mesh
// resolver (render/mesh_resolver.hpp) and, as Core applies it (N3), the
// world's validator (validate_world_desc: its visual references and props'
// mesh_refs) and the scene's and the object file's (their shared asset rules,
// a non-empty visual mesh_ref). The default library adds one rule across its
// objects: no two refs that differ only in case (scene/library.hpp's
// check_library_contents). It lives here, in the lowest library all of them
// link.
//
// A mesh_ref names a mesh by a RELATIVE PATH under a root, without its
// extension: "props/ball" is <root>/props/ball.spmesh. Segments are joined by
// '/'; it is never absolute. Each segment is non-empty ASCII [A-Za-z0-9_-]:
// so no "." or "..", no extension, no drive or scheme, and nothing that is
// not a file name on Windows and Linux alike (Core's portability rule). Nor
// is a segment a name Windows reserves for a device, in any case: CON, PRN,
// AUX, NUL, COM1-COM9, LPT1-LPT9 (the lead's). So a ref cannot leave the
// root it is looked up in, and names the same file on every platform.
// ---------------------------------------------------------------------------
#pragma once

#include <cstddef>
#include <string_view>

namespace spade {

namespace detail {

[[nodiscard]] constexpr bool mesh_ref_char(char c) noexcept {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
}

[[nodiscard]] constexpr char ascii_upper(char c) noexcept { return c >= 'a' && c <= 'z' ? static_cast<char>(c - 32) : c; }

// CON, PRN, AUX, NUL, COM1-COM9 or LPT1-LPT9, in any case.
[[nodiscard]] constexpr bool windows_device_name(std::string_view segment) noexcept {
    const auto is = [segment](std::string_view name) {
        if (segment.size() != name.size()) return false;
        for (std::size_t i = 0; i < name.size(); ++i) {
            if (ascii_upper(segment[i]) != name[i]) return false;
        }
        return true;
    };
    if (is("CON") || is("PRN") || is("AUX") || is("NUL")) return true;
    if (segment.size() != 4u || segment[3] < '1' || segment[3] > '9') return false;
    const std::string_view stem = segment.substr(0, 3);
    const auto stem_is = [stem](std::string_view name) {
        for (std::size_t i = 0; i < 3u; ++i) {
            if (ascii_upper(stem[i]) != name[i]) return false;
        }
        return true;
    };
    return stem_is("COM") || stem_is("LPT");
}

}  // namespace detail

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
        for (const char c : segment) {
            if (!detail::mesh_ref_char(c)) {
                return "it holds a character other than an ASCII letter, a digit, '_' or '-' in a segment";
            }
        }
        if (detail::windows_device_name(segment)) {
            return "it has a segment Windows reserves for a device (CON, PRN, AUX, NUL, COM1-9, LPT1-9)";
        }
        if (slash == std::string_view::npos) return {};
        rest.remove_prefix(slash + 1u);
    }
}

[[nodiscard]] inline bool valid_mesh_ref(std::string_view ref) noexcept { return mesh_ref_problem(ref).empty(); }

}  // namespace spade
